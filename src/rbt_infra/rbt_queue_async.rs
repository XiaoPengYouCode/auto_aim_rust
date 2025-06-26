use crossbeam_queue::ArrayQueue;
// use std::sync::atomic::AtomicBool;
use tokio::sync::Notify;

// use crate::rbt_global::IS_RUNNING;

pub struct RbtQueueAsync<T> {
    queue: ArrayQueue<T>,
    notify: Notify,
}

impl<T> RbtQueueAsync<T> {
    pub fn new(capacity: usize) -> Self {
        RbtQueueAsync {
            queue: ArrayQueue::new(capacity),
            notify: Notify::new(),
        }
    }

    pub fn force_push(&self, item: T) {
        self.queue.force_push(item);
        self.notify.notify_one();
        // todo!("需要停止逻辑");
    }

    pub async fn pop(&self) -> Option<T> {
        self.notify.notified().await;
        self.queue.pop()
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }
}
