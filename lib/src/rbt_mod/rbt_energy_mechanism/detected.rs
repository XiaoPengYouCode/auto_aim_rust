use image::imageops::FilterType;

use crate::rbt_infra::rbt_cfg::EnergyMechanismDetectorCfg;
use crate::rbt_mod::rbt_comm::rbt_comm_frame::TaskMode;
use crate::rbt_mod::rbt_detector::rbt_yolo::LetterboxTransform;
use crate::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::EnemyFaction;

pub const ENERGY_MECHANISM_INPUT_WIDTH: usize = 640;
pub const ENERGY_MECHANISM_INPUT_HEIGHT: usize = 640;
pub const ENERGY_MECHANISM_KEYPOINTS: usize = 5;
pub const ENERGY_MECHANISM_OUTPUT_MIN_CHANNELS: usize = 18;
pub const ENERGY_MECHANISM_OUTPUT_MAX_CHANNELS: usize = 23;
const ENERGY_MECHANISM_PAD_VALUE: f32 = 114.0 / 255.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnergyMechanismMode {
    Large,
    Small,
}

impl EnergyMechanismMode {
    pub fn from_task_mode(task_mode: TaskMode) -> Option<Self> {
        match task_mode {
            TaskMode::HitBigBuff => Some(Self::Large),
            TaskMode::HitSmallBuff => Some(Self::Small),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnergyMechanismClass {
    Target,
    CenterMarker,
    RedTarget,
    RedHit,
    BlueTarget,
    BlueHit,
}

impl EnergyMechanismClass {
    fn from_index(index: usize, class_count: usize) -> Option<Self> {
        match (class_count, index) {
            (2, 0) => Some(Self::RedTarget),
            (2, 1) => Some(Self::BlueTarget),
            (4, 0) => Some(Self::RedTarget),
            (4, 1) => Some(Self::RedHit),
            (4, 2) => Some(Self::BlueTarget),
            (4, 3) => Some(Self::BlueHit),
            _ => None,
        }
    }

    pub fn is_target(self) -> bool {
        matches!(self, Self::Target | Self::RedTarget | Self::BlueTarget)
    }

    pub fn faction(self) -> Option<EnemyFaction> {
        match self {
            Self::RedTarget | Self::RedHit => Some(EnemyFaction::R),
            Self::BlueTarget | Self::BlueHit => Some(EnemyFaction::B),
            Self::Target | Self::CenterMarker => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnergyMechanismBBox {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
}

impl EnergyMechanismBBox {
    pub fn from_center_size(cx: f32, cy: f32, width: f32, height: f32) -> Self {
        Self {
            x1: cx - width * 0.5,
            y1: cy - height * 0.5,
            x2: cx + width * 0.5,
            y2: cy + height * 0.5,
        }
    }

    pub fn center(self) -> na::Point2<f32> {
        na::Point2::new((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)
    }

    fn width(self) -> f32 {
        (self.x2 - self.x1).max(0.0)
    }

    fn height(self) -> f32 {
        (self.y2 - self.y1).max(0.0)
    }

    fn area(self) -> f32 {
        self.width() * self.height()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct EnergyMechanismObject {
    pub bbox: EnergyMechanismBBox,
    pub class: EnergyMechanismClass,
    pub confidence: f32,
    /// Keypoint order: top, left, bottom, right, R center.
    pub keypoints: [na::Point2<f32>; ENERGY_MECHANISM_KEYPOINTS],
}

impl EnergyMechanismObject {
    pub fn target_center(&self) -> na::Point2<f32> {
        let sum = self.keypoints[..4]
            .iter()
            .fold(na::Vector2::zeros(), |acc, point| acc + point.coords);
        na::Point2::from(sum / 4.0)
    }

    pub fn r_center(&self) -> na::Point2<f32> {
        self.keypoints[4]
    }
}

pub struct EnergyMechanismFrame {
    time: tokio::time::Instant,
    id: u64,
    mode: EnergyMechanismMode,
    pre_infer: nd::Array4<f32>,
    infer_post: nd::Array2<f32>,
    letterbox: LetterboxTransform,
}

impl EnergyMechanismFrame {
    pub fn new(mode: EnergyMechanismMode) -> Self {
        Self {
            time: tokio::time::Instant::now(),
            id: 0,
            mode,
            pre_infer: nd::Array4::zeros((
                1,
                3,
                ENERGY_MECHANISM_INPUT_HEIGHT,
                ENERGY_MECHANISM_INPUT_WIDTH,
            )),
            infer_post: nd::Array2::zeros((0, 0)),
            letterbox: LetterboxTransform::default(),
        }
    }

    pub fn id(&self) -> u64 {
        self.id
    }

    pub fn set_id(&mut self, id: u64) {
        self.id = id;
    }

    pub fn mode(&self) -> EnergyMechanismMode {
        self.mode
    }

    pub fn time_used(&self) -> std::time::Duration {
        self.time.elapsed()
    }

    pub fn pre_data(&mut self) -> nd::ArrayViewMut4<'_, f32> {
        self.pre_infer.view_mut()
    }

    pub fn pre_data_ref(&self) -> nd::ArrayView4<'_, f32> {
        self.pre_infer.view()
    }

    pub fn infer_data_ref(&self) -> nd::ArrayView2<'_, f32> {
        self.infer_post.view()
    }

    pub fn set_infer_output(&mut self, output: nd::Array2<f32>) {
        self.infer_post = output;
    }

    pub fn set_letterbox_transform(&mut self, transform: LetterboxTransform) {
        self.letterbox = transform;
    }

    pub fn letterbox_transform(&self) -> LetterboxTransform {
        self.letterbox
    }
}

#[derive(Debug, Clone, Copy)]
pub struct EnergyMechanismYoloPostprocessCfg {
    pub confidence_threshold: f32,
    pub nms_iou_threshold: f32,
    pub self_fraction: Option<EnemyFaction>,
}

impl EnergyMechanismYoloPostprocessCfg {
    pub fn from_detector_cfg(
        cfg: &EnergyMechanismDetectorCfg,
        self_fraction: Option<EnemyFaction>,
    ) -> Self {
        Self {
            confidence_threshold: cfg.confidence_threshold,
            nms_iou_threshold: cfg.nms_iou_threshold,
            self_fraction,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EnergyMechanismYoloDecodeStats {
    pub anchors: usize,
    pub confidence_pass: usize,
    pub class_pass: usize,
    pub target_pass: usize,
    pub self_color_pass: usize,
    pub geometry_pass: usize,
    pub nms_kept: usize,
}

pub fn preprocess_energy_mechanism_letterbox_f32(
    mut input_array: nd::ArrayViewMut4<'_, f32>,
    image: &image::DynamicImage,
) -> LetterboxTransform {
    input_array.fill(ENERGY_MECHANISM_PAD_VALUE);

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
    let resized_width = ((image_width as f32 * scale).round() as u32).clamp(1, target_width);
    let resized_height = ((image_height as f32 * scale).round() as u32).clamp(1, target_height);
    let pad_x = ((target_width - resized_width) / 2) as usize;
    let pad_y = ((target_height - resized_height) / 2) as usize;

    let rgb = image.to_rgb8();
    let resized =
        image::imageops::resize(&rgb, resized_width, resized_height, FilterType::Triangle);

    for (x, y, pixel) in resized.enumerate_pixels() {
        let x_new = pad_x + x as usize;
        let y_new = pad_y + y as usize;
        let [r, g, b] = pixel.0;

        input_array[[0, 0, y_new, x_new]] = r as f32 / 255.0;
        input_array[[0, 1, y_new, x_new]] = g as f32 / 255.0;
        input_array[[0, 2, y_new, x_new]] = b as f32 / 255.0;
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

pub fn decode_energy_mechanism_output_with_stats(
    output: &nd::ArrayView2<'_, f32>,
    transform: LetterboxTransform,
    cfg: &EnergyMechanismYoloPostprocessCfg,
) -> (Vec<EnergyMechanismObject>, EnergyMechanismYoloDecodeStats) {
    let mut stats = EnergyMechanismYoloDecodeStats {
        anchors: output.shape().get(1).copied().unwrap_or_default(),
        ..EnergyMechanismYoloDecodeStats::default()
    };
    let channels = output.shape().first().copied().unwrap_or_default();
    if channels < ENERGY_MECHANISM_OUTPUT_MIN_CHANNELS {
        return (Vec::new(), stats);
    }

    let Some((class_count, keypoint_dim)) = infer_output_contract(channels) else {
        return (Vec::new(), stats);
    };
    let keypoint_base = 4 + class_count;

    let mut candidates = Vec::new();
    for anchor_idx in 0..stats.anchors {
        let mut best_score = f32::NEG_INFINITY;
        let mut best_class_idx = 0_usize;
        for class_idx in 0..class_count {
            let score = output[[4 + class_idx, anchor_idx]];
            if score > best_score {
                best_score = score;
                best_class_idx = class_idx;
            }
        }

        if best_score < cfg.confidence_threshold {
            continue;
        }
        stats.confidence_pass += 1;

        let Some(class) = EnergyMechanismClass::from_index(best_class_idx, class_count) else {
            continue;
        };
        stats.class_pass += 1;
        if !class.is_target() {
            continue;
        }
        stats.target_pass += 1;
        if is_self_color(class, cfg.self_fraction) {
            continue;
        }
        stats.self_color_pass += 1;

        let cx = output[[0, anchor_idx]];
        let cy = output[[1, anchor_idx]];
        let width = output[[2, anchor_idx]];
        let height = output[[3, anchor_idx]];
        if ![cx, cy, width, height]
            .iter()
            .all(|value| value.is_finite())
            || width <= 1.0
            || height <= 1.0
        {
            continue;
        }

        let mut keypoints = [na::Point2::origin(); ENERGY_MECHANISM_KEYPOINTS];
        let mut finite_keypoints = true;
        for (keypoint_idx, keypoint) in keypoints.iter_mut().enumerate() {
            let base = keypoint_base + keypoint_idx * keypoint_dim;
            let (x, y) =
                transform.restore_point(output[[base, anchor_idx]], output[[base + 1, anchor_idx]]);
            if !x.is_finite() || !y.is_finite() {
                finite_keypoints = false;
                break;
            }
            *keypoint = na::Point2::new(x, y);
        }
        if !finite_keypoints {
            continue;
        }

        let (x1, y1) = transform.restore_point(cx - width * 0.5, cy - height * 0.5);
        let (x2, y2) = transform.restore_point(cx + width * 0.5, cy + height * 0.5);
        let bbox = EnergyMechanismBBox {
            x1: x1.min(x2),
            y1: y1.min(y2),
            x2: x1.max(x2),
            y2: y1.max(y2),
        };
        if bbox.width() < 1.0 || bbox.height() < 1.0 {
            continue;
        }

        stats.geometry_pass += 1;
        candidates.push(EnergyMechanismObject {
            bbox,
            class,
            confidence: best_score,
            keypoints,
        });
    }

    candidates.sort_by(|left, right| right.confidence.total_cmp(&left.confidence));
    let mut kept = Vec::new();
    while let Some(candidate) = candidates.first().cloned() {
        candidates.remove(0);
        candidates.retain(|other| bbox_iou(candidate.bbox, other.bbox) < cfg.nms_iou_threshold);
        kept.push(candidate);
    }
    stats.nms_kept = kept.len();

    (kept, stats)
}

fn is_self_color(class: EnergyMechanismClass, self_fraction: Option<EnemyFaction>) -> bool {
    matches!(
        (class.faction(), self_fraction),
        (Some(EnemyFaction::B), Some(EnemyFaction::B))
            | (Some(EnemyFaction::R), Some(EnemyFaction::R))
    )
}

fn infer_output_contract(channels: usize) -> Option<(usize, usize)> {
    for class_count in [4_usize, 2] {
        if channels < 4 + class_count {
            continue;
        }
        let keypoint_channels = channels - 4 - class_count;
        if !keypoint_channels.is_multiple_of(ENERGY_MECHANISM_KEYPOINTS) {
            continue;
        }
        let keypoint_dim = keypoint_channels / ENERGY_MECHANISM_KEYPOINTS;
        if keypoint_dim == 2 || keypoint_dim == 3 {
            return Some((class_count, keypoint_dim));
        }
    }
    None
}

fn bbox_iou(lhs: EnergyMechanismBBox, rhs: EnergyMechanismBBox) -> f32 {
    let x1 = lhs.x1.max(rhs.x1);
    let y1 = lhs.y1.max(rhs.y1);
    let x2 = lhs.x2.min(rhs.x2);
    let y2 = lhs.y2.min(rhs.y2);
    let intersection = (x2 - x1).max(0.0) * (y2 - y1).max(0.0);
    let union = lhs.area() + rhs.area() - intersection;
    if union <= f32::EPSILON {
        0.0
    } else {
        intersection / union
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::{DynamicImage, ImageBuffer, Rgb};

    #[test]
    fn centered_letterbox_tracks_padding() {
        let image = DynamicImage::ImageRgb8(ImageBuffer::from_pixel(320, 160, Rgb([255, 0, 0])));
        let mut input = nd::Array4::<f32>::zeros((1, 3, 640, 640));

        let transform = preprocess_energy_mechanism_letterbox_f32(input.view_mut(), &image);

        assert_eq!(transform.resized_width, 640);
        assert_eq!(transform.resized_height, 320);
        assert_eq!(transform.pad_x, 0.0);
        assert_eq!(transform.pad_y, 160.0);
    }

    #[test]
    fn decode_keeps_four_class_enemy_target_with_keypoint_dim_two() {
        let mut output = nd::Array2::<f32>::zeros((18, 3));
        output[[0, 0]] = 320.0;
        output[[1, 0]] = 320.0;
        output[[2, 0]] = 100.0;
        output[[3, 0]] = 80.0;
        output[[4, 0]] = 0.9;
        for idx in 0..ENERGY_MECHANISM_KEYPOINTS {
            output[[8 + idx * 2, 0]] = 300.0 + idx as f32;
            output[[9 + idx * 2, 0]] = 310.0 + idx as f32;
        }
        let cfg = EnergyMechanismYoloPostprocessCfg {
            confidence_threshold: 0.5,
            nms_iou_threshold: 0.4,
            self_fraction: Some(EnemyFaction::B),
        };

        let (objects, stats) = decode_energy_mechanism_output_with_stats(
            &output.view(),
            LetterboxTransform::default(),
            &cfg,
        );

        assert_eq!(objects.len(), 1);
        assert_eq!(objects[0].class, EnergyMechanismClass::RedTarget);
        assert_eq!(stats.nms_kept, 1);
    }

    #[test]
    fn decode_keeps_two_class_color_target_with_keypoint_dim_three() {
        let mut output = nd::Array2::<f32>::zeros((21, 1));
        output[[0, 0]] = 320.0;
        output[[1, 0]] = 320.0;
        output[[2, 0]] = 100.0;
        output[[3, 0]] = 80.0;
        output[[4, 0]] = 0.9;
        for idx in 0..ENERGY_MECHANISM_KEYPOINTS {
            let base = 6 + idx * 3;
            output[[base, 0]] = 300.0 + idx as f32;
            output[[base + 1, 0]] = 310.0 + idx as f32;
            output[[base + 2, 0]] = 1.0;
        }
        let cfg = EnergyMechanismYoloPostprocessCfg {
            confidence_threshold: 0.5,
            nms_iou_threshold: 0.4,
            self_fraction: Some(EnemyFaction::B),
        };

        let (objects, stats) = decode_energy_mechanism_output_with_stats(
            &output.view(),
            LetterboxTransform::default(),
            &cfg,
        );

        assert_eq!(objects.len(), 1);
        assert_eq!(objects[0].class, EnergyMechanismClass::RedTarget);
        assert_eq!(stats.nms_kept, 1);
    }

    #[test]
    fn decode_rejects_self_color_and_hit_classes() {
        let mut output = nd::Array2::<f32>::zeros((23, 2));
        for anchor in 0..2 {
            output[[0, anchor]] = 320.0;
            output[[1, anchor]] = 320.0;
            output[[2, anchor]] = 100.0;
            output[[3, anchor]] = 100.0;
            for idx in 0..ENERGY_MECHANISM_KEYPOINTS {
                let base = 8 + idx * 3;
                output[[base, anchor]] = 300.0 + idx as f32;
                output[[base + 1, anchor]] = 310.0 + idx as f32;
                output[[base + 2, anchor]] = 1.0;
            }
        }
        output[[6, 0]] = 0.9; // Blue target, self when self is blue.
        output[[5, 1]] = 0.95; // Red hit, not a target.
        let cfg = EnergyMechanismYoloPostprocessCfg {
            confidence_threshold: 0.5,
            nms_iou_threshold: 0.4,
            self_fraction: Some(EnemyFaction::B),
        };

        let (objects, stats) = decode_energy_mechanism_output_with_stats(
            &output.view(),
            LetterboxTransform::default(),
            &cfg,
        );

        assert!(objects.is_empty());
        assert_eq!(stats.confidence_pass, 2);
        assert_eq!(stats.target_pass, 1);
    }

    #[test]
    fn decode_two_class_target_respects_self_color_filter() {
        let mut output = nd::Array2::<f32>::zeros((21, 2));
        for anchor in 0..2 {
            output[[0, anchor]] = 320.0;
            output[[1, anchor]] = 320.0;
            output[[2, anchor]] = 100.0;
            output[[3, anchor]] = 80.0;
            for idx in 0..ENERGY_MECHANISM_KEYPOINTS {
                let base = 6 + idx * 3;
                output[[base, anchor]] = 300.0 + idx as f32;
                output[[base + 1, anchor]] = 310.0 + idx as f32;
                output[[base + 2, anchor]] = 1.0;
            }
        }
        output[[4, 0]] = 0.92; // Red target, should be filtered when self is red.
        output[[5, 1]] = 0.95; // Blue target, should be kept.
        let cfg = EnergyMechanismYoloPostprocessCfg {
            confidence_threshold: 0.5,
            nms_iou_threshold: 0.4,
            self_fraction: Some(EnemyFaction::R),
        };

        let (objects, stats) = decode_energy_mechanism_output_with_stats(
            &output.view(),
            LetterboxTransform::default(),
            &cfg,
        );

        assert_eq!(objects.len(), 1);
        assert_eq!(objects[0].class, EnergyMechanismClass::BlueTarget);
        assert_eq!(stats.confidence_pass, 2);
        assert_eq!(stats.target_pass, 2);
        assert_eq!(stats.self_color_pass, 1);
    }
}
