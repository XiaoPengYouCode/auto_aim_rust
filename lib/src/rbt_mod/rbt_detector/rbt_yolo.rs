// 这里主要放一些 yolo 模型检测过程中的工具
// 感谢 wjt, tyk 对神经网络的贡献

use image::GenericImageView;

use crate::rbt_mod::rbt_armor::{ArmorColor, ArmorId, ArmorLabel};

const GRAY: f32 = 114.0;

pub fn letterbox(input_array: &mut nd::Array4<f32>, resized_img: &image::DynamicImage) {
    for (x, y, pixel) in resized_img.pixels() {
        let x = x as usize;
        let y = y as usize;
        let [r, g, b, _] = pixel.0;

        let y_new = y + 12; // 调整 y 坐标以适应预处理要求

        // 将像素值填充到数组中
        if (12..372).contains(&y_new) {
            input_array[[0, 0, y_new, x]] = r as f32;
            input_array[[0, 1, y_new, x]] = g as f32;
            input_array[[0, 2, y_new, x]] = b as f32;
        } else {
            input_array[[0, 0, y, x]] = GRAY;
            input_array[[0, 1, y, x]] = GRAY;
            input_array[[0, 2, y, x]] = GRAY;
        }
    }
}

pub fn nms(mut boxes: Vec<(BBox, usize, f32, usize)>) -> Vec<(BBox, usize, f32, usize)> {
    boxes.sort_by(|box1, box2| box2.2.total_cmp(&box1.2));
    // 按置信度降序排序
    let mut result = Vec::new();
    while !boxes.is_empty() {
        result.push(boxes[0]); // 选择置信度最高的框
        boxes = boxes
            .iter()
            .filter(|box1| (intersection(&boxes[0].0, &box1.0) / union(&boxes[0].0, &box1.0)) < 0.7)
            .copied()
            .collect(); // 去除与当前框重叠度较高的框
    }
    result
}

pub struct YoloLabel(ArmorColor, ArmorId);

impl YoloLabel {
    pub(crate) fn id(&self) -> &ArmorId {
        &self.1
    }
    pub(crate) fn color(&self) -> &ArmorColor {
        &self.0
    }
}

pub const YOLO_LABEL_TABLE: [ArmorLabel; 36] = [
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Hero1),
    YoloLabel(ArmorColor::B, ArmorId::Engineer2),
    YoloLabel(ArmorColor::B, ArmorId::Infantry3),
    YoloLabel(ArmorColor::B, ArmorId::Infantry4),
    YoloLabel(ArmorColor::B, ArmorId::Sentry7),
    YoloLabel(ArmorColor::B, ArmorId::Outpost8),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::B, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Hero1),
    YoloLabel(ArmorColor::R, ArmorId::Engineer2),
    YoloLabel(ArmorColor::R, ArmorId::Infantry3),
    YoloLabel(ArmorColor::R, ArmorId::Infantry4),
    YoloLabel(ArmorColor::R, ArmorId::Sentry7),
    YoloLabel(ArmorColor::R, ArmorId::Outpost8),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
    YoloLabel(ArmorColor::R, ArmorId::Invalid),
];

/// BoundingBox yolo模型候选框
/// 因为目前跟神经网络交互的部分暂时还是 f32，所以暂时没有提供泛型实现
#[derive(Debug, Clone, Copy)]
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
}

/// 计算 BBox 的交集
pub fn intersection(box1: &BBox, box2: &BBox) -> f32 {
    (box1.x2().min(box2.x2()) - box1.x1().max(box2.x1()))
        * (box1.y2().min(box2.y2()) - box1.y1().max(box2.y1()))
}

/// 计算 BBox 的并集
pub fn union(box1: &BBox, box2: &BBox) -> f32 {
    ((box1.x2() - box1.x1()) * (box1.y2() - box1.y1()))
        + ((box2.x2() - box2.x1()) * (box2.y2() - box2.y1()))
        - intersection(box1, box2)
}
