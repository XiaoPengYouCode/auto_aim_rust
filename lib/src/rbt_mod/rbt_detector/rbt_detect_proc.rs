// 这里主要放一些模型检测过程中所需要的处理函数

use image::GenericImageView;

use crate::rbt_mod::rbt_detector::{BBox, intersection, union};

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
