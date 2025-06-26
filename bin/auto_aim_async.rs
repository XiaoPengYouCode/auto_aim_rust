use auto_aim_rust::{rbt_app::multi_threads, rbt_err::RbtResult, rbt_infra::rbt_log};

#[tokio::main]
async fn main() -> RbtResult<()> {
    // init logger
    let _logger_guard = rbt_log::logger_init().await?;
    multi_threads::multi_thread_pipeline().await
}
