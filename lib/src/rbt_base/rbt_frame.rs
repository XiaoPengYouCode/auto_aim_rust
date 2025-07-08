use tokio::time::Instant;

use crate::rbt_global::FAILED_COUNT;
use tracing::{debug, error, warn};

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

impl RbtFrame {
    pub fn new() -> Self {
        Self {
            time: Instant::now(),
            data: RbtFrameData {
                pre_infer: nd::Array4::<f32>::zeros([1, 3, 384, 640]),
                infer_post: nd::Array3::<f32>::zeros([5040, 48, 1]),
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

    pub fn pre_data(&mut self) -> nd::ArrayViewMut4<f32> {
        self.data.pre_infer.view_mut()
    }

    pub fn infer_data(&mut self) -> nd::ArrayViewMut3<f32> {
        self.data.infer_post.view_mut()
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
    pre_infer: nd::Array4<f32>,
    infer_post: nd::Array3<f32>,
}
