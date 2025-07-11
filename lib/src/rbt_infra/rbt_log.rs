use tracing::info;
use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::Layer;
use tracing_subscriber::{fmt::layer, layer::SubscriberExt, registry, util::SubscriberInitExt};

use crate::rbt_err::RbtResult;
use crate::rbt_global::GENERIC_RBT_CFG;

/// 注意函数的调用方只需要维持 Option 就行
/// 可以用于区分是否存在 appender 守护
/// 在本场景中没有作用，无需解包
pub async fn logger_init() -> RbtResult<Option<WorkerGuard>> {
    let logger_cfg = GENERIC_RBT_CFG.read().unwrap().logger_cfg.clone();
    if !logger_cfg.file_log_enable && !logger_cfg.console_log_enable {
        return Ok(None);
    }

    let terminal_log_filter =
        tracing_subscriber::EnvFilter::try_new(&logger_cfg.console_log_filter)?;
    let file_log_filter = tracing_subscriber::EnvFilter::try_new(&logger_cfg.file_log_filter)?;

    let console_layer = if logger_cfg.console_log_enable {
        Some(
            layer()
                .with_writer(std::io::stdout)
                .without_time()
                .with_line_number(true)
                .with_filter(terminal_log_filter),
        )
    } else {
        None
    };

    let (file_layer, guard) = if logger_cfg.file_log_enable {
        // 获取当前时间戳，用于生成唯一文件名
        let now = chrono::Local::now();
        let file_name = format!("{}", now.format("%H:%M:%S")); // 添加 .log 后缀
        let directory_name = format!("log/{}", now.format("%Y/%m/%d"));
        tokio::fs::create_dir_all(&directory_name).await?; // 确保目录存在

        let file_appender = tracing_appender::rolling::never(directory_name, file_name); // 使用 never
        let (non_blocking, guard) = tracing_appender::non_blocking(file_appender);
        (
            Some(
                layer()
                    .with_writer(non_blocking)
                    .with_filter(file_log_filter),
            ),
            Some(guard),
        )
    } else {
        (None, None)
    };

    registry::Registry::default()
        .with(console_layer)
        .with(file_layer)
        .init();

    info!(
        "log initialized with filter: {}",
        logger_cfg.console_log_filter
    );
    info!("log initialized with output:");
    info!(
        "file_log_enable: {}, console_log_enable: {}",
        logger_cfg.file_log_enable, logger_cfg.console_log_enable
    );
    Ok(guard)
}
