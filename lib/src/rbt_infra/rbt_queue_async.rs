// 使用时候需要使用 Arc 将队列进行包装
// 然后分别传递给生产者和消费者

//! Async latest-value queue for realtime vision pipelines.
//!
//! This queue is intentionally not a FIFO work queue. Each stage only needs the
//! freshest frame or solved result; processing stale backlog increases
//! end-to-end aiming latency and can make the estimator act on old world state.
//!
//! The design requirement is "latest sample wins":
//! - `push_latest` never blocks producers. If the queue is full, the oldest item
//!   is discarded.
//! - `pop_latest` and `try_pop_latest` drain pending backlog and return only the
//!   newest item.
//! - Capacity greater than 1 is only a short burst buffer. It does not change
//!   the consumer-side latest-only semantics.
//!
//! This mirrors vivsionn's `FixedSafeQueue` usage in the vision thread manager,
//! where frame queues are used to keep latency bounded instead of preserving
//! every intermediate frame.

use crossbeam_queue::ArrayQueue;
use tokio::sync::Notify;

/// 异步单生产者单消费者队列实现
///
/// 该结构体封装了一个固定容量的数组队列和通知机制，
/// 支持异步的推送和弹出操作
pub struct RbtSPSCQueueAsync<T> {
    queue: ArrayQueue<T>,
    notify: Notify,
}

impl<T> RbtSPSCQueueAsync<T> {
    /// 创建一个新的异步SPSC队列
    ///
    /// # 参数
    /// * `capacity` - 队列的最大容量
    ///
    /// # 返回值
    /// 返回一个指定容量的RbtSPSCQueueAsync实例
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "queue capacity must be greater than zero");
        RbtSPSCQueueAsync {
            queue: ArrayQueue::new(capacity),
            notify: Notify::new(),
        }
    }

    /// 推入最新数据；如果队列已满，则丢弃最老的数据。
    ///
    /// 该行为对齐 `vivsionn/src/ArmorDetector/fixed_queue.hpp` 的
    /// `FixedSafeQueue::push`，用于视觉流水线中“保最新帧、控延迟”的队列。
    pub fn push_latest(&self, item: T) {
        let _dropped_oldest = self.queue.force_push(item);
        self.notify.notify_one();
    }

    /// 异步弹出队列中最新的元素，并丢弃同一批次中更旧的元素。
    ///
    /// 这等价于 `vivsionn` 调用端常见的 `pop` 后继续清空队列，只保留最后一帧。
    pub async fn pop_latest(&self) -> Option<T> {
        loop {
            if let Some(item) = self.try_pop_latest() {
                return Some(item);
            }
            self.notify.notified().await;
        }
    }

    /// 非阻塞弹出最新元素，并丢弃队列中更旧的待处理元素。
    pub fn try_pop_latest(&self) -> Option<T> {
        let mut latest = self.queue.pop()?;
        while let Some(item) = self.queue.pop() {
            latest = item;
        }
        Some(latest)
    }

    /// 清空队列中所有待处理元素。
    pub fn clear(&self) {
        while self.queue.pop().is_some() {}
    }

    /// 检查当前队列元素的长度
    ///
    /// # 返回值
    /// 返回队列中当前存储的元素数量
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// 检查当前队列是否为空
    ///
    /// # 返回值
    /// 队列为空时返回 true
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    /// 检查当前队列的容量
    ///
    /// # 返回值
    /// 返回队列的最大容量
    pub fn capacity(&self) -> usize {
        self.queue.capacity()
    }
}

#[cfg(test)]
mod tests {
    use super::RbtSPSCQueueAsync;

    #[test]
    fn push_latest_drops_oldest_when_full() {
        let queue = RbtSPSCQueueAsync::new(2);

        queue.push_latest(1);
        queue.push_latest(2);
        queue.push_latest(3);

        assert_eq!(queue.try_pop_latest(), Some(3));
        assert!(queue.is_empty());
    }

    #[test]
    fn try_pop_latest_keeps_newest_and_discards_stale_items() {
        let queue = RbtSPSCQueueAsync::new(3);

        queue.push_latest(10);
        queue.push_latest(20);
        queue.push_latest(30);

        assert_eq!(queue.try_pop_latest(), Some(30));
        assert!(queue.is_empty());
    }

    #[test]
    fn clear_removes_pending_items() {
        let queue = RbtSPSCQueueAsync::new(2);

        queue.push_latest(1);
        queue.push_latest(2);
        queue.clear();

        assert_eq!(queue.try_pop_latest(), None);
        assert!(queue.is_empty());
    }

    #[tokio::test]
    async fn pop_latest_waits_for_data() {
        let queue = RbtSPSCQueueAsync::new(1);

        queue.push_latest(7);

        assert_eq!(queue.pop_latest().await, Some(7));
        assert!(queue.is_empty());
    }
}
