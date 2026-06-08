use std::{
    path::{Path, PathBuf},
    sync::Arc,
};

use jiff::Zoned;
use log::info;
use logforth::append;
use logforth::bridge::log::LogBridge;
use logforth::core::Logger;
use logforth::filter::RustLogFilter;
use logforth::layout::TextLayout;
use logforth_append_async::AsyncBuilder;
use logforth_append_file::FileBuilder;

use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use crate::rbt_infra::rbt_global::GENERIC_RBT_CFG;

pub struct RbtLoggerGuard {
    logger: Arc<Logger>,
}

impl Drop for RbtLoggerGuard {
    fn drop(&mut self) {
        self.logger.flush();
    }
}

fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")))
}

fn daily_log_dir(now: &Zoned) -> PathBuf {
    workspace_root()
        .join("log")
        .join(format!("{:04}", now.year()))
        .join(format!("{:02}", now.month()))
        .join(format!("{:02}", now.day()))
}

/// 初始化日志系统
///
/// 使用 logforth 作为日志后端，支持控制台和文件双输出。
/// 控制台使用带颜色的 TextLayout，文件使用无颜色版本。
/// 文件日志按小时自动滚动，异步写入不阻塞主线程。
///
/// 返回 RbtLoggerGuard，持有它可确保退出时日志被 flush。
/// 调用方只需维持 Option<RbtLoggerGuard> 即可。
pub fn logger_init() -> RbtResult<Option<RbtLoggerGuard>> {
    let logger_cfg = GENERIC_RBT_CFG.read().unwrap().logger_cfg.clone();
    if !logger_cfg.file_log_enable && !logger_cfg.console_log_enable {
        return Ok(None);
    }

    let mut logger_builder = logforth::core::builder();

    if logger_cfg.console_log_enable {
        let console_filter: RustLogFilter = logger_cfg.console_log_filter.as_str().into();
        let console_appender = append::Stdout::default().with_layout(TextLayout::default());
        logger_builder = logger_builder
            .dispatch(|dispatch| dispatch.filter(console_filter).append(console_appender));
    }

    if logger_cfg.file_log_enable {
        let file_filter: RustLogFilter = logger_cfg.file_log_filter.as_str().into();

        // 使用当前时间生成日志目录和文件名（兼容原有目录结构）
        let now = Zoned::now();
        let file_name = format!("{:02}:{:02}:{:02}", now.hour(), now.minute(), now.second());
        let directory_name = daily_log_dir(&now);
        std::fs::create_dir_all(&directory_name)
            .map_err(|e| RbtError::LoggerInitError(e.to_string()))?;

        let file_appender = FileBuilder::new(directory_name, file_name)
            .layout(TextLayout::default().no_color())
            .build()
            .map_err(|e| RbtError::LoggerInitError(e.to_string()))?;

        let async_file_appender = AsyncBuilder::new("rbt-log-file")
            .buffered_lines_limit(Some(8192))
            .overflow_block()
            .append(file_appender)
            .build();

        logger_builder = logger_builder
            .dispatch(|dispatch| dispatch.filter(file_filter).append(async_file_appender));
    }

    let logger = Arc::new(logger_builder.build());
    let bridge = LogBridge::new(logger.clone());
    log::set_boxed_logger(Box::new(bridge))
        .map_err(|err| RbtError::LoggerInitError(err.to_string()))?;
    log::set_max_level(log::LevelFilter::Trace);

    info!(
        "log initialized with console filter: {}",
        logger_cfg.console_log_filter
    );
    info!(
        "log initialized with file filter: {}",
        logger_cfg.file_log_filter
    );
    info!("log initialized with output:");
    info!(
        "file_log_enable: {}, console_log_enable: {}",
        logger_cfg.file_log_enable, logger_cfg.console_log_enable
    );

    Ok(Some(RbtLoggerGuard { logger }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use log::{debug, error, info, trace, warn};
    use std::fs;

    /// 测试 logger_init 完整流程：
    /// 1. 成功初始化
    /// 2. 各日志级别正常输出
    /// 3. 日志文件正确创建
    /// 4. RbtLoggerGuard drop 不 panic
    #[test]
    fn test_logger_init_full_flow() {
        // 初始化
        let guard = logger_init().expect("logger_init should succeed");
        assert!(guard.is_some(), "guard should be Some when logging enabled");

        // 各日志级别
        trace!("[test] this is a trace message");
        debug!("[test] this is a debug message");
        info!("[test] this is an info message");
        warn!("[test] this is a warn message");
        error!("[test] this is an error message");

        // 带参数格式化
        let x = 42;
        info!("[test] formatted value: x={}", x);

        // 检查日志目录和文件已创建
        let log_dir = workspace_root().join("log");
        assert!(fs::metadata(log_dir).is_ok(), "log directory should exist");

        // 查找今天日期的子目录
        let daily_dir = daily_log_dir(&Zoned::now());
        assert!(
            fs::metadata(&daily_dir).is_ok(),
            "daily log directory '{}' should exist",
            daily_dir.display()
        );

        // 检查目录中有日志文件
        let entries: Vec<_> = fs::read_dir(&daily_dir)
            .expect("should read daily dir")
            .filter_map(|e| e.ok())
            .collect();
        assert!(
            !entries.is_empty(),
            "daily log directory should contain at least one log file"
        );

        // drop guard — 不应该 panic
        drop(guard);
    }
}
