extern crate ndarray as nd;
extern crate rerun as rr;

use crate::rbt_threads::{infer, post_process, pre_process};
use auto_aim_rust::rbt_infra::rbt_log;
use lib as auto_aim_rust;
use lib::rbt_infra::rbt_err::RbtResult;
use lib::rbt_infra::rbt_global::GENERIC_RBT_CFG;
use lib::rbt_infra::rbt_queue_async::RbtSPSCQueueAsync;
use lib::rbt_mod::rbt_detector::rbt_frame::RbtFrame;
use ort::execution_providers;
use ort::session::Session;
use std::sync::Arc;
use tracing::info;

pub mod rbt_threads;

#[tokio::main]
async fn main() -> RbtResult<()> {
    // init logger
    let _logger_guard = rbt_log::logger_init().await?;
    // init rerun logger
    // let rec = rr::RecordingStreamBuilder::new("rbt_async").save("test.rrd")?;

    let pre_infer_queue = Arc::new(RbtSPSCQueueAsync::<RbtFrame>::new(1));
    let infer_post_queue = Arc::new(RbtSPSCQueueAsync::<RbtFrame>::new(1));

    // build orrtruntime session
    let session = Session::builder()?
        .with_execution_providers([execution_providers::OpenVINOExecutionProvider::default()
            .with_device_type("GPU")
            .build()
            .error_on_failure()])?
        .with_optimization_level(ort::session::builder::GraphOptimizationLevel::Level3)?
        .with_inter_threads(8)?
        .commit_from_file(
            GENERIC_RBT_CFG
                .read()
                .unwrap()
                .detector_cfg
                .armor_detect_model_path
                .as_str(),
        )?;

    // let session = Arc::new(Mutex::new(session));
    let pre_task_handler = pre_process(pre_infer_queue.clone());
    let infer_task_handler = infer(pre_infer_queue, session, infer_post_queue.clone());
    let post_task_handler = post_process(infer_post_queue);

    let tim = std::time::Instant::now();
    let (_, _, _) = tokio::join!(pre_task_handler, infer_task_handler, post_task_handler);
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await; // wait for post process to finish
    info!("multi_thread_pipeline finished in {:?}", tim.elapsed());

    Ok(())
}
