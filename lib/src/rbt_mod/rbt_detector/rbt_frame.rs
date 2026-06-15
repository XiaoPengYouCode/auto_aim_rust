use tokio::time::Instant;

use crate::rbt_infra::rbt_global::FAILED_COUNT;
use crate::rbt_mod::rbt_detector::rbt_yolo::LetterboxTransform;
use image::GrayImage;
use log::{debug, error, warn};

pub const ARMOR_INPUT_WIDTH: usize = 640;
pub const ARMOR_INPUT_HEIGHT: usize = 640;
pub const ARMOR_OUTPUT_ROWS: usize = 25_200;
pub const ARMOR_OUTPUT_COLS: usize = 22;

pub struct RbtFrame {
    time: Instant,
    pub data: RbtFrameData,
    id: u64,
    stage: RbtFrameStage,
}

pub enum RbtFrameStage {
    Pre,
    Infer,
    Post,
    Init,
}

impl Default for RbtFrame {
    fn default() -> Self {
        Self::new()
    }
}

impl RbtFrame {
    pub fn new() -> Self {
        Self {
            time: Instant::now(),
            data: RbtFrameData {
                pre_infer: nd::Array4::<half::f16>::zeros([
                    1,
                    3,
                    ARMOR_INPUT_HEIGHT,
                    ARMOR_INPUT_WIDTH,
                ]),
                infer_post: nd::Array2::<f32>::zeros([ARMOR_OUTPUT_ROWS, ARMOR_OUTPUT_COLS]),
                letterbox: LetterboxTransform::default(),
                gray_frame: None,
            },
            id: 0,
            stage: RbtFrameStage::Init,
        }
    }

    pub fn id(&self) -> u64 {
        self.id
    }

    pub fn set_id(&mut self, id: u64) {
        self.id = id;
    }

    pub fn set_state(&mut self, state: RbtFrameStage) {
        self.stage = state;
    }

    pub fn time(&self) -> Instant {
        self.time
    }

    pub fn pre_data(&mut self) -> nd::ArrayViewMut4<'_, half::f16> {
        self.data.pre_infer.view_mut()
    }

    pub fn pre_data_ref(&self) -> nd::ArrayView4<'_, half::f16> {
        self.data.pre_infer.view()
    }

    pub fn infer_data(&mut self) -> nd::ArrayViewMut2<'_, f32> {
        self.data.infer_post.view_mut()
    }

    pub fn infer_data_ref(&self) -> nd::ArrayView2<'_, f32> {
        self.data.infer_post.view()
    }

    pub fn set_letterbox_transform(&mut self, transform: LetterboxTransform) {
        self.data.letterbox = transform;
    }

    pub fn letterbox_transform(&self) -> LetterboxTransform {
        self.data.letterbox
    }

    pub fn set_gray_frame(&mut self, gray_frame: GrayImage) {
        self.data.gray_frame = Some(gray_frame);
    }

    pub fn gray_frame(&self) -> Option<&GrayImage> {
        self.data.gray_frame.as_ref()
    }

    pub fn time_used(&self) -> std::time::Duration {
        self.time.elapsed()
    }
}

impl Drop for RbtFrame {
    fn drop(&mut self) {
        match &self.stage {
            RbtFrameStage::Init => {
                // 初始状态，该状态仅仅用于创建空的 RbtFrame
                FAILED_COUNT.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                error!(
                    "RbtFrame with id {} is being dropped in Pre, with lifetime {:?}",
                    self.id,
                    self.time.elapsed()
                )
            }
            RbtFrameStage::Pre => {
                // Pre 属于生产者，而且生产速度很快，被丢弃的情况较多
                FAILED_COUNT.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                debug!(
                    "RbtFrame with id {} is being dropped in Pre, with lifetime {:?}",
                    self.id,
                    self.time.elapsed()
                )
            }
            RbtFrameStage::Infer => {
                // Infer 属于消费者，速度较慢，丢弃的情况较少，所以使用 warn 级别
                FAILED_COUNT.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                warn!(
                    "RbtFrame with id {} is being dropped in Infer, with lifetime {:?}",
                    self.id,
                    self.time.elapsed()
                );
            }
            RbtFrameStage::Post => {
                // Post 属于消费者，速度较快，但是位于下游，所以速度受到 infer 的限制
                debug!(
                    "RbtFrame with id {} is being dropped in Post, with lifetime {:?}",
                    self.id,
                    self.time.elapsed()
                );
            }
        }
    }
}

pub struct RbtFrameData {
    pre_infer: nd::Array4<half::f16>,
    infer_post: nd::Array2<f32>,
    letterbox: LetterboxTransform,
    gray_frame: Option<GrayImage>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rbt_frame_carries_original_gray_frame() {
        let mut frame = RbtFrame::new();
        let gray = GrayImage::new(32, 24);

        frame.set_gray_frame(gray);

        let stored = frame.gray_frame().expect("gray frame should be stored");
        assert_eq!(stored.width(), 32);
        assert_eq!(stored.height(), 24);
    }
}
