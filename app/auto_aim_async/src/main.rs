extern crate rerun as rr;

use auto_aim_rust::rbt_infra::rbt_log;
use lib as auto_aim_rust;
use lib::rbt_infra::rbt_err::RbtResult;

pub mod rbt_threads;
mod threads;

#[tokio::main]
async fn main() -> RbtResult<()> {
    // init logger
    let _logger_guard = rbt_log::logger_init().await?;
    // init rerun logger
    let rec = rr::RecordingStreamBuilder::new("rbt_async").save("test.rrd")?;
    threads::multi_thread_pipeline(rec).await
}
