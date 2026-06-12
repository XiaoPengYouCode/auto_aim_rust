// 这里主要放一些 yolo 模型检测过程中的工具
// 感谢 wjt, tyk 对神经网络的贡献

use std::collections::HashMap;

use half::f16;
use image::imageops::FilterType;

use crate::rbt_base::rbt_geometry::rbt_point2::RbtImgPoint2;
use crate::rbt_infra::rbt_cfg::ArmorDetectorCfg;
use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;
use crate::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::{EnemyFaction, EnemyId};

pub const LETTERBOX_PAD_VALUE: f32 = 0.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LetterboxTransform {
    pub input_width: u32,
    pub input_height: u32,
    pub image_width: u32,
    pub image_height: u32,
    pub resized_width: u32,
    pub resized_height: u32,
    pub pad_x: f32,
    pub pad_y: f32,
    pub scale: f32,
}

impl Default for LetterboxTransform {
    fn default() -> Self {
        Self {
            input_width: 640,
            input_height: 640,
            image_width: 640,
            image_height: 640,
            resized_width: 640,
            resized_height: 640,
            pad_x: 0.0,
            pad_y: 0.0,
            scale: 1.0,
        }
    }
}

impl LetterboxTransform {
    pub fn restore_point(self, x: f32, y: f32) -> (f32, f32) {
        let scale = if self.scale.abs() > f32::EPSILON {
            self.scale
        } else {
            1.0
        };
        ((x - self.pad_x) / scale, (y - self.pad_y) / scale)
    }
}

pub fn preprocess_letterbox_f16(
    mut input_array: nd::ArrayViewMut4<'_, f16>,
    image: &image::DynamicImage,
) -> LetterboxTransform {
    input_array.fill(f16::from_f32(LETTERBOX_PAD_VALUE));

    let shape = input_array.shape();
    let target_height = shape[2] as u32;
    let target_width = shape[3] as u32;
    let image_width = image.width();
    let image_height = image.height();

    if target_width == 0 || target_height == 0 || image_width == 0 || image_height == 0 {
        return LetterboxTransform {
            input_width: target_width,
            input_height: target_height,
            image_width,
            image_height,
            resized_width: 0,
            resized_height: 0,
            pad_x: 0.0,
            pad_y: 0.0,
            scale: 1.0,
        };
    }

    let scale =
        (target_width as f32 / image_width as f32).min(target_height as f32 / image_height as f32);
    let resized_width = ((image_width as f32 * scale).floor() as u32).clamp(1, target_width);
    let resized_height = ((image_height as f32 * scale).floor() as u32).clamp(1, target_height);
    let pad_x = 0;
    let pad_y = 0;

    let rgb = image.to_rgb8();
    let resized =
        image::imageops::resize(&rgb, resized_width, resized_height, FilterType::Triangle);

    for (x, y, pixel) in resized.enumerate_pixels() {
        let x_new = x as usize;
        let y_new = y as usize;
        let [r, g, b] = pixel.0;

        input_array[[0, 0, y_new, x_new]] = f16::from_f32(r as f32 / 255.0);
        input_array[[0, 1, y_new, x_new]] = f16::from_f32(g as f32 / 255.0);
        input_array[[0, 2, y_new, x_new]] = f16::from_f32(b as f32 / 255.0);
    }

    LetterboxTransform {
        input_width: target_width,
        input_height: target_height,
        image_width,
        image_height,
        resized_width,
        resized_height,
        pad_x: pad_x as f32,
        pad_y: pad_y as f32,
        scale,
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ArmorYoloPostprocessCfg {
    pub score_threshold: f32,
    pub confidence_threshold: f32,
    pub nms_iou_threshold: f32,
    pub self_fraction: Option<EnemyFaction>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ArmorYoloDecodeStats {
    pub rows: usize,
    pub score_pass: usize,
    pub color_pass: usize,
    pub self_color_pass: usize,
    pub number_pass: usize,
    pub geometry_pass: usize,
    pub nms_kept: usize,
    pub confidence_pass: usize,
}

impl ArmorYoloPostprocessCfg {
    pub fn from_armor_cfg(
        cfg: &ArmorDetectorCfg,
        self_fraction: Option<EnemyFaction>,
    ) -> ArmorYoloPostprocessCfg {
        ArmorYoloPostprocessCfg {
            score_threshold: cfg.score_threshold,
            confidence_threshold: cfg.confidence_threshold,
            nms_iou_threshold: cfg.nms_iou_threshold,
            self_fraction,
        }
    }
}

pub fn decode_armor_output(
    output: &nd::ArrayView2<'_, f32>,
    transform: LetterboxTransform,
    cfg: &ArmorYoloPostprocessCfg,
) -> HashMap<EnemyId, Vec<DetectedArmor>> {
    decode_armor_output_with_stats(output, transform, cfg).0
}

pub fn decode_armor_output_with_stats(
    output: &nd::ArrayView2<'_, f32>,
    transform: LetterboxTransform,
    cfg: &ArmorYoloPostprocessCfg,
) -> (HashMap<EnemyId, Vec<DetectedArmor>>, ArmorYoloDecodeStats) {
    let mut candidates = Vec::new();
    let mut stats = ArmorYoloDecodeStats::default();

    for row in output.axis_iter(nd::Axis(0)) {
        stats.rows += 1;
        if row.len() < 22 {
            continue;
        }

        let score = sigmoid(row[8]);
        if score < cfg.score_threshold {
            continue;
        }
        stats.score_pass += 1;

        let Some(color) = decode_color(argmax(&row, 9, 13)) else {
            continue;
        };
        stats.color_pass += 1;
        if is_self_color(color, cfg.self_fraction) {
            continue;
        }
        stats.self_color_pass += 1;

        let Some(armor_id) = decode_number(argmax(&row, 13, 22)) else {
            continue;
        };
        stats.number_pass += 1;

        let points = [
            transform.restore_point(row[0], row[1]),
            transform.restore_point(row[6], row[7]),
            transform.restore_point(row[4], row[5]),
            transform.restore_point(row[2], row[3]),
        ];
        let Some(corners) = ArmorCorners::from_points(points) else {
            continue;
        };
        stats.geometry_pass += 1;

        candidates.push(ArmorCandidate {
            bbox: corners.bbox(),
            armor_id,
            score,
            corners,
        });
    }

    let mut armors = HashMap::with_capacity(candidates.len());
    let nms_candidates = nms_armor_candidates(candidates, cfg.nms_iou_threshold);
    stats.nms_kept = nms_candidates.len();
    for (id, candidate) in nms_candidates
        .into_iter()
        .filter(|candidate| {
            let keep = candidate.score >= cfg.confidence_threshold;
            if keep {
                stats.confidence_pass += 1;
            }
            keep
        })
        .enumerate()
    {
        armors
            .entry(candidate.armor_id)
            .or_insert_with(Vec::new)
            .push(candidate.corners.to_detected_armor(id));
    }

    (armors, stats)
}

fn argmax(row: &nd::ArrayView1<'_, f32>, start: usize, end: usize) -> usize {
    row.slice(nd::s![start..end])
        .iter()
        .enumerate()
        .max_by(|left, right| left.1.total_cmp(right.1))
        .map(|(idx, _)| idx)
        .unwrap_or_default()
}

fn sigmoid(x: f32) -> f32 {
    1.0 / (1.0 + (-x).exp())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ArmorYoloColor {
    Blue,
    Red,
    Gray,
    Purple,
}

fn decode_color(color_idx: usize) -> Option<ArmorYoloColor> {
    match color_idx {
        0 => Some(ArmorYoloColor::Blue),
        1 => Some(ArmorYoloColor::Red),
        2 => Some(ArmorYoloColor::Gray),
        3 => Some(ArmorYoloColor::Purple),
        _ => None,
    }
}

fn is_self_color(color: ArmorYoloColor, self_fraction: Option<EnemyFaction>) -> bool {
    matches!(
        (color, self_fraction),
        (ArmorYoloColor::Blue, Some(EnemyFaction::B))
            | (ArmorYoloColor::Red, Some(EnemyFaction::R))
    )
}

fn decode_number(num_idx: usize) -> Option<EnemyId> {
    match num_idx {
        0 => Some(EnemyId::Sentry7),
        1 => Some(EnemyId::Hero1),
        2 => Some(EnemyId::Engineer2),
        3 => Some(EnemyId::Infantry3),
        4 => Some(EnemyId::Infantry4),
        6..=8 => Some(EnemyId::Outpost8),
        _ => None,
    }
}

#[derive(Debug, Clone)]
struct ArmorCandidate {
    bbox: BBox,
    armor_id: EnemyId,
    score: f32,
    corners: ArmorCorners,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct ArmorCorners {
    lt: (f32, f32),
    lb: (f32, f32),
    rb: (f32, f32),
    rt: (f32, f32),
}

impl ArmorCorners {
    fn from_points(mut points: [(f32, f32); 4]) -> Option<Self> {
        points.sort_by(|left, right| left.0.total_cmp(&right.0));
        if points[0].1 > points[1].1 {
            points.swap(0, 1);
        }
        if points[2].1 > points[3].1 {
            points.swap(2, 3);
        }

        let corners = Self {
            lt: points[0],
            lb: points[1],
            rt: points[2],
            rb: points[3],
        };
        let bbox = corners.bbox();
        if bbox.width() < 1.0 || bbox.height() < 1.0 {
            return None;
        }
        Some(corners)
    }

    fn bbox(&self) -> BBox {
        let points = [self.lt, self.lb, self.rb, self.rt];
        let min_x = points
            .iter()
            .map(|point| point.0)
            .fold(f32::INFINITY, f32::min);
        let min_y = points
            .iter()
            .map(|point| point.1)
            .fold(f32::INFINITY, f32::min);
        let max_x = points
            .iter()
            .map(|point| point.0)
            .fold(f32::NEG_INFINITY, f32::max);
        let max_y = points
            .iter()
            .map(|point| point.1)
            .fold(f32::NEG_INFINITY, f32::max);

        BBox::new(min_x, min_y, max_x, max_y)
    }

    fn center(&self) -> (f32, f32) {
        let points = [self.lt, self.lb, self.rb, self.rt];
        let x = points.iter().map(|point| point.0).sum::<f32>() / points.len() as f32;
        let y = points.iter().map(|point| point.1).sum::<f32>() / points.len() as f32;
        (x, y)
    }

    fn to_detected_armor(self, id: usize) -> DetectedArmor {
        let center = self.center();
        DetectedArmor::new(
            RbtImgPoint2::new_screen_pixel(center.0, center.1),
            RbtImgPoint2::new_screen_pixel(self.lt.0, self.lt.1),
            RbtImgPoint2::new_screen_pixel(self.lb.0, self.lb.1),
            RbtImgPoint2::new_screen_pixel(self.rb.0, self.rb.1),
            RbtImgPoint2::new_screen_pixel(self.rt.0, self.rt.1),
            id,
        )
    }
}

fn nms_armor_candidates(
    mut candidates: Vec<ArmorCandidate>,
    iou_threshold: f32,
) -> Vec<ArmorCandidate> {
    candidates.sort_by(|left, right| right.score.total_cmp(&left.score));

    let mut kept = Vec::new();
    while !candidates.is_empty() {
        let current = candidates.remove(0);
        candidates.retain(|candidate| bbox_iou(&current.bbox, &candidate.bbox) < iou_threshold);
        kept.push(current);
    }

    kept
}

pub struct YoloLabel(pub EnemyFaction, pub EnemyId);

impl YoloLabel {
    pub fn id(&self) -> &EnemyId {
        &self.1
    }
    pub fn color(&self) -> &EnemyFaction {
        &self.0
    }
}

/// BoundingBox yolo模型候选框
/// 因为目前跟神经网络交互的部分暂时还是 f32，所以暂时没有提供泛型实现
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BBox(f32, f32, f32, f32);

impl BBox {
    pub fn new(x1: f32, y1: f32, x2: f32, y2: f32) -> Self {
        BBox(x1, y1, x2, y2)
    }

    #[inline(always)]
    fn x1(&self) -> f32 {
        self.0
    }

    #[inline(always)]
    fn y1(&self) -> f32 {
        self.1
    }

    #[inline(always)]
    fn x2(&self) -> f32 {
        self.2
    }

    #[inline(always)]
    fn y2(&self) -> f32 {
        self.3
    }

    #[inline(always)]
    fn width(&self) -> f32 {
        (self.x2() - self.x1()).max(0.0)
    }

    #[inline(always)]
    fn height(&self) -> f32 {
        (self.y2() - self.y1()).max(0.0)
    }

    #[inline(always)]
    fn area(&self) -> f32 {
        self.width() * self.height()
    }
}

/// 计算 BBox 的交集
pub fn intersection(box1: &BBox, box2: &BBox) -> f32 {
    let width = (box1.x2().min(box2.x2()) - box1.x1().max(box2.x1())).max(0.0);
    let height = (box1.y2().min(box2.y2()) - box1.y1().max(box2.y1())).max(0.0);
    width * height
}

/// 计算 BBox 的并集
pub fn union(box1: &BBox, box2: &BBox) -> f32 {
    box1.area() + box2.area() - intersection(box1, box2)
}

fn bbox_iou(box1: &BBox, box2: &BBox) -> f32 {
    let union = union(box1, box2);
    if union <= f32::EPSILON {
        0.0
    } else {
        intersection(box1, box2) / union
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::{DynamicImage, ImageBuffer, Rgba};

    fn identity_transform() -> LetterboxTransform {
        LetterboxTransform::default()
    }

    fn cfg(self_fraction: Option<EnemyFaction>) -> ArmorYoloPostprocessCfg {
        ArmorYoloPostprocessCfg {
            score_threshold: 0.5,
            confidence_threshold: 0.5,
            nms_iou_threshold: 0.45,
            self_fraction,
        }
    }

    fn put_detection(
        output: &mut nd::Array2<f32>,
        row: usize,
        corners: [(f32, f32); 4],
        score_logit: f32,
        color_idx: usize,
        num_idx: usize,
    ) {
        // 模型字段顺序对齐 vivsionn: [0,1], [6,7], [4,5], [2,3] 是四个角点。
        output[[row, 0]] = corners[0].0;
        output[[row, 1]] = corners[0].1;
        output[[row, 6]] = corners[1].0;
        output[[row, 7]] = corners[1].1;
        output[[row, 4]] = corners[2].0;
        output[[row, 5]] = corners[2].1;
        output[[row, 2]] = corners[3].0;
        output[[row, 3]] = corners[3].1;
        output[[row, 8]] = score_logit;
        output[[row, 9 + color_idx]] = 10.0;
        output[[row, 13 + num_idx]] = 10.0;
    }

    #[test]
    fn letterbox_top_left_pads_black_and_normalizes_rgb() {
        let image = DynamicImage::ImageRgba8(ImageBuffer::from_pixel(2, 2, Rgba([1, 2, 3, 255])));
        let mut input = nd::Array4::<f16>::zeros((1, 3, 4, 4));

        let transform = preprocess_letterbox_f16(input.view_mut(), &image);

        assert_eq!(transform.resized_width, 4);
        assert_eq!(transform.resized_height, 4);
        assert_eq!(transform.pad_x, 0.0);
        assert_eq!(transform.pad_y, 0.0);
        assert!((input[[0, 0, 0, 0]].to_f32() - 1.0 / 255.0).abs() < 0.001);
        assert!((input[[0, 1, 0, 0]].to_f32() - 2.0 / 255.0).abs() < 0.001);
        assert!((input[[0, 2, 0, 0]].to_f32() - 3.0 / 255.0).abs() < 0.001);

        let wide = DynamicImage::ImageRgba8(ImageBuffer::from_pixel(4, 2, Rgba([9, 8, 7, 255])));
        let mut padded = nd::Array4::<f16>::zeros((1, 3, 4, 4));
        let transform = preprocess_letterbox_f16(padded.view_mut(), &wide);

        assert_eq!(transform.resized_width, 4);
        assert_eq!(transform.resized_height, 2);
        assert_eq!(transform.pad_y, 0.0);
        assert!((padded[[0, 0, 0, 0]].to_f32() - 9.0 / 255.0).abs() < 0.001);
        assert!((padded[[0, 0, 2, 0]].to_f32() - LETTERBOX_PAD_VALUE).abs() < 0.001);
    }

    #[test]
    fn decodes_color_number_and_corner_order() {
        let mut output = nd::Array2::<f32>::zeros((25_200, 22));
        put_detection(
            &mut output,
            0,
            [(50.0, 40.0), (10.0, 20.0), (50.0, 20.0), (10.0, 40.0)],
            10.0,
            1,
            0,
        );

        let armors = decode_armor_output(&output.view(), identity_transform(), &cfg(None));
        let armor = &armors[&EnemyId::Sentry7][0];

        assert_eq!(armor.lt().x, 10.0);
        assert_eq!(armor.lt().y, 20.0);
        assert_eq!(armor.lb().x, 10.0);
        assert_eq!(armor.lb().y, 40.0);
        assert_eq!(armor.rb().x, 50.0);
        assert_eq!(armor.rb().y, 40.0);
        assert_eq!(armor.rt().x, 50.0);
        assert_eq!(armor.rt().y, 20.0);
        assert_eq!(armor.center().x, 30.0);
        assert_eq!(armor.center().y, 30.0);
    }

    #[test]
    fn restores_letterbox_coordinates_to_original_image() {
        let mut output = nd::Array2::<f32>::zeros((25_200, 22));
        put_detection(
            &mut output,
            0,
            [(120.0, 90.0), (120.0, 130.0), (200.0, 130.0), (200.0, 90.0)],
            10.0,
            2,
            6,
        );
        let transform = LetterboxTransform {
            pad_x: 100.0,
            pad_y: 50.0,
            scale: 2.0,
            ..LetterboxTransform::default()
        };

        let armors = decode_armor_output(&output.view(), transform, &cfg(Some(EnemyFaction::B)));
        let armor = &armors[&EnemyId::Outpost8][0];

        assert_eq!(armor.lt().x, 10.0);
        assert_eq!(armor.lt().y, 20.0);
        assert_eq!(armor.rb().x, 50.0);
        assert_eq!(armor.rb().y, 40.0);
    }

    #[test]
    fn filters_self_color_but_allows_neutral_colors() {
        let mut output = nd::Array2::<f32>::zeros((25_200, 22));
        let corners = [(10.0, 10.0), (10.0, 30.0), (40.0, 30.0), (40.0, 10.0)];
        put_detection(&mut output, 0, corners, 10.0, 1, 1);
        put_detection(
            &mut output,
            1,
            corners.map(|(x, y)| (x + 50.0, y)),
            10.0,
            0,
            2,
        );
        put_detection(
            &mut output,
            2,
            corners.map(|(x, y)| (x + 100.0, y)),
            10.0,
            2,
            3,
        );
        put_detection(
            &mut output,
            3,
            corners.map(|(x, y)| (x + 150.0, y)),
            10.0,
            3,
            4,
        );

        let armors = decode_armor_output(
            &output.view(),
            identity_transform(),
            &cfg(Some(EnemyFaction::R)),
        );

        assert!(!armors.contains_key(&EnemyId::Hero1));
        assert!(armors.contains_key(&EnemyId::Engineer2));
        assert!(armors.contains_key(&EnemyId::Infantry3));
        assert!(armors.contains_key(&EnemyId::Infantry4));
    }

    #[test]
    fn nms_keeps_highest_scored_overlapping_armor() {
        let mut output = nd::Array2::<f32>::zeros((25_200, 22));
        put_detection(
            &mut output,
            0,
            [(10.0, 10.0), (10.0, 40.0), (50.0, 40.0), (50.0, 10.0)],
            10.0,
            0,
            1,
        );
        put_detection(
            &mut output,
            1,
            [(12.0, 12.0), (12.0, 42.0), (52.0, 42.0), (52.0, 12.0)],
            8.0,
            0,
            1,
        );

        let armors = decode_armor_output(&output.view(), identity_transform(), &cfg(None));

        assert_eq!(armors[&EnemyId::Hero1].len(), 1);
        assert_eq!(armors[&EnemyId::Hero1][0].lt().x, 10.0);
    }
}
