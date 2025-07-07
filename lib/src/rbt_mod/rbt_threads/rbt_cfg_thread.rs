use notify::{Event, EventKind, RecursiveMode, Watcher, recommended_watcher};
use std::future::Future;
use std::future::pending;
use std::path::Path;
use std::pin::Pin;
use tokio::sync::broadcast::Receiver;
use tokio::sync::mpsc::channel;
use tokio::sync::watch::Sender;
use tokio::task::JoinHandle;
use tokio::time::{Duration, Instant, sleep_until};
use tracing::{error, info, warn};

pub use crate::rbt_err::RbtResult;
pub use crate::rbt_infra::rbt_cfg;

use self::rbt_cfg::RbtCfg;

pub async fn rbt_cfg_thread(
    mut shutdown_rx: Receiver<()>,
    cfg_sender: Sender<RbtCfg>,
    init_cfg: RbtCfg,
) -> JoinHandle<RbtResult<()>> {
    tokio::spawn(async move {
        let (event_tx, mut event_rx) = channel::<(Event, u64)>(10);
        let mut cfg_update_count = 0;
        let mut event_counter = 0;
        // the res maybe occur error when something wrong like file was delete
        let mut watcher = recommended_watcher(move |res: Result<Event, notify::Error>| {
            match res {
                Ok(event) => {
                    event_counter += 1;
                    if event.paths.iter().any(|p| p.ends_with("rbt_cfg.toml"))
                        && matches!(event.kind, EventKind::Modify(_))
                    {
                        // info!("Receive event: {:?}, counter:{}", event.kind, event_counter);
                        if event_tx.blocking_send((event, event_counter)).is_err() {
                            error!("Failed to send file watcher event throw channel 参数不会更新");
                        };
                    }
                }
                Err(e) => {
                    error!("Something was wrong durning watching file(err: {})", e);
                }
            }
        })
        .unwrap();

        // 这里需要监听整个目录，因为 vim 在修改文件过程中会创建一个新文件，然后把旧文件删除
        if watcher
            .watch(Path::new("cfg/"), RecursiveMode::NonRecursive)
            .is_err()
        {
            error!("Can't find config file");
        };
        let mut old_version_cfg = init_cfg;
        let mut debounce_deadline: Option<Instant> = None;
        let debounce_duration = Duration::from_millis(200);
        loop {
            let debounce_future: Pin<Box<dyn Future<Output = ()> + Send>> = match debounce_deadline
            {
                Some(deadline) => Box::pin(sleep_until(deadline)),
                None => Box::pin(pending()),
            };
            tokio::select! {
                _ = shutdown_rx.recv() => {
                    info!("rbt_cfg_thread exit.");
                    break;
                }

                // 事件防抖 (Debounce)
                // 某些编辑器（如 Vim）在保存文件时可能会触发多个事件（例如，Modify 然后是 Remove 再 Create）
                // 防抖逻辑
                // 当收到第一个文件变更事件后，等待一个短暂的窗口期（比如 200ms）
                // 如果在此期间没有新的事件到来，才执行配置重载
                // 如果来了新事件，则重置等待计时器, 这样可以确保一系列密集的写操作只触发一次重载

                // 实际测试下来修改一次文件可能会有10多次事件，modify事件可能也有3-4次
                // 但是不知道为什么这里只会收到一次，我使用了release模式也不行
                // 感觉可能操作比release模式还要快导致的

                // 实际情况是过滤条件给太狠了，需要放宽监听条件
                // 但是也不能太宽，因为各种命令行工具也可能会读取这些文件，产生事件
                // 最终选择 modify 作为过滤条件，一般一次改动中，开始是几次 data modify，最后一次是 metadata modify
                Some((_, event_counter)) = event_rx.recv() => {
                    // info!("Receive event from channel: {}", event_counter);
                    debounce_deadline = Some(Instant::now() + debounce_duration);
                }

                // 有 deadline 才更新 cfg
                _ = debounce_future, if debounce_deadline.is_some() => {
                    info!("reload cfg");
                    debounce_deadline = None;
                    match rbt_cfg::RbtCfg::from_toml_async().await {
                        Ok(rbt_cfg_new) => {
                            // if the new and old version are same, skip cfg update
                            if rbt_cfg_new == old_version_cfg {
                                warn!("New cfg are same to old version, skip cfg update");
                                continue;
                            };
                            old_version_cfg = rbt_cfg_new.clone();
                            if cfg_sender.send(rbt_cfg_new).is_err() {
                                error!("Failed to send cfg from watch channel");
                            };
                            cfg_update_count += 1;
                            info!("cfg update count = {}", cfg_update_count);
                        }
                        Err(e) => {
                            error!("Failed to read config file cause by: {}, use old version", e);
                        }
                    }
                }
            }
        }
        Ok(())
    })
}
