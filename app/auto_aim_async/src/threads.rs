// use tokio::sync::watch to send image
// 我们的应用分为三个线程
// 1. 取图+前处理
// 2. 推理
// 3. 后处理

use ort::{execution_providers, session::Session};
use std::sync::Arc;
use tracing::info;

use crate::rbt_threads::{infer, post_process, pre_process};
use lib::rbt_infra::rbt_err::RbtResult;
use lib::rbt_infra::rbt_global::GENERIC_RBT_CFG;
use lib::rbt_infra::rbt_queue_async::RbtSPSCQueueAsync;
use lib::rbt_mod::rbt_detector::rbt_frame::RbtFrame;

/// 1. Pre-processing (image acquisition and preprocessing)
/// 2. Inference (running the model on the pre-processed image using OpenVINO on GPU)
/// 3. Post-processing (final processing of inference output)
///
/// This function orchestrates three concurrent tasks:
/// - `pre_process`: handles image acquisition and initial preprocessing
/// - `infer`: runs the ONNX model inference using OpenVINO with GPU acceleration
/// - `post_process`: performs final post-processing on the inference output
///
/// The pipeline uses a bounded, single-producer-single-consumer (SPSC) queue to pass frames between stages.
/// The function returns an error if any stage fails or if the configuration is invalid.
///
/// The execution is asynchronous and leverages Tokio for concurrency. After all tasks complete,
/// a 100ms sleep is added to ensure the post-process stage has fully finished.
///
/// # Arguments
/// * `rec` - A recording stream source providing raw image frames
///
/// # Returns
/// * `RbtResult<()>` - Ok if pipeline completes successfully, Err otherwise
pub async fn multi_thread_pipeline(rec: rr::RecordingStream) -> RbtResult<()> {
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
