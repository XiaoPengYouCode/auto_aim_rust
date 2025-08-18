// 使用时候需要使用 Arc 将队列进行包装
// 然后分别传递给生产者和消费者

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
        RbtSPSCQueueAsync {
            queue: ArrayQueue::new(capacity),
            notify: Notify::new(),
        }
    }

    /// 如果队列已满，则强制替换最老的元素
    ///
    /// # 参数
    /// * `item` - 要推入队列的元素
    pub fn force_push(&self, item: T) {
        self.queue.force_push(item);
        self.notify.notify_one();
    }

    /// 异步弹出队列中的元素
    ///
    /// 该方法会等待直到队列中有元素可弹出
    ///
    /// # 返回值
    /// 返回队列中的一个元素，如果队列为空则返回None
    pub async fn pop(&self) -> Option<T> {
        // Check if queue already has elements before waiting
        if let Some(item) = self.queue.pop() {
            return Some(item);
        }
        self.notify.notified().await;
        self.queue.pop()
    }

    /// 检查当前队列元素的长度
    ///
    /// # 返回值
    /// 返回队列中当前存储的元素数量
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// 检查当前队列的容量
    ///
    /// # 返回值
    /// 返回队列的最大容量
    pub fn capacity(&self) -> usize {
        self.queue.capacity()
    }
}
