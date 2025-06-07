#[allow(unused)]

pub mod img_dbg {
    use image::{ImageBuffer, Rgb};

    pub fn draw_big_dot(
        img: &mut ImageBuffer<Rgb<u8>, Vec<u8>>,
        x: u32,
        y: u32,
        color: Option<Rgb<u8>>,
    ) {
        let color = color.unwrap_or(Rgb([255, 0, 0])); // Default to red color
        for dx in 0..=6 {
            for dy in 0..=6 {
                let new_x = x + dx - 3;
                let new_y = y + dy - 3;
                img.put_pixel(new_x, new_y, color);
            }
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct BoundingBox {
        pub(crate) x1: f32,
        pub(crate) y1: f32,
        pub(crate) x2: f32,
        pub(crate) y2: f32,
    }

    pub fn intersection(box1: &BoundingBox, box2: &BoundingBox) -> f32 {
        (box1.x2.min(box2.x2) - box1.x1.max(box2.x1))
            * (box1.y2.min(box2.y2) - box1.y1.max(box2.y1))
    }

    pub fn union(box1: &BoundingBox, box2: &BoundingBox) -> f32 {
        ((box1.x2 - box1.x1) * (box1.y2 - box1.y1)) + ((box2.x2 - box2.x1) * (box2.y2 - box2.y1))
            - intersection(box1, box2)
    }
}
