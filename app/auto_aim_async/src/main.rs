extern crate ndarray as nd;
extern crate rerun as rr;

use crate::rbt_threads::{
    PlannerTrackSnapshot, control_loop_250hz, estimate_process, infer, post_process, pre_process,
    static_image_path,
};
use auto_aim_rust::rbt_infra::rbt_log;
use lib as auto_aim_rust;
use lib::rbt_infra::rbt_err::{RbtError, RbtResult};
use lib::rbt_infra::rbt_global::GENERIC_RBT_CFG;
use lib::rbt_infra::rbt_queue_async::RbtSPSCQueueAsync;
use lib::rbt_mod::rbt_comm::rbt_comm_frame::SensData;
use lib::rbt_mod::rbt_detector::rbt_frame::RbtFrame;
use lib::rbt_mod::rbt_solver::RbtSolvedResults;
use log::{info, warn};
use ort::ep;
use ort::session::Session;
use std::path::Path;
use std::sync::Arc;

pub mod rbt_threads;

fn ensure_required_file(path: &Path, description: &str) -> RbtResult<()> {
    if path.is_file() {
        return Ok(());
    }

    Err(RbtError::PreconditionFailed(format!(
        "{description} is not a file: {}",
        path.display()
    )))
}

#[tokio::main]
async fn main() -> RbtResult<()> {
    // init logger
    let _logger_guard = rbt_log::logger_init()?;
    // init rerun logger
    let rerun_path = Path::new("rerun-log").join("rbt_async.rrd");
    if let Some(parent) = rerun_path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let rec = rr::RecordingStreamBuilder::new("rbt_async").save(rerun_path)?;

    let pre_infer_queue = Arc::new(RbtSPSCQueueAsync::<RbtFrame>::new(1));
    let infer_post_queue = Arc::new(RbtSPSCQueueAsync::<RbtFrame>::new(1));
    let solved_queue = Arc::new(RbtSPSCQueueAsync::<RbtSolvedResults>::new(1));
    let track_queue = Arc::new(RbtSPSCQueueAsync::<PlannerTrackSnapshot>::new(1));
    let feedback_queue = Arc::new(RbtSPSCQueueAsync::<SensData>::new(1));
    let cfg = GENERIC_RBT_CFG.read().unwrap().clone();

    let model_path = Path::new(cfg.detector_cfg.armor_detect_model_path.as_str());
    ensure_required_file(model_path, "armor model file")?;
    let image_path = static_image_path();
    ensure_required_file(&image_path, "static input image")?;

    // build onnxruntime session
    let session_builder = Session::builder()?;
    let session_builder = match cfg.detector_cfg.ort_ep.as_str() {
        "OpenVINO" => {
            session_builder.with_execution_providers([ep::OpenVINOExecutionProvider::default()
                .with_device_type("GPU")
                .build()])?
        }
        "TensorRT" => {
            session_builder.with_execution_providers([ep::TensorRTExecutionProvider::default()
                .with_engine_cache(true)
                .with_engine_cache_path(cfg.detector_cfg.armor_detect_engine_path.as_str())
                .with_fp16(true)
                .build()])?
        }
        "CPU" => session_builder.with_execution_providers([ep::CPUExecutionProvider::default()
            .with_arena_allocator(true)
            .build()])?,
        other => {
            warn!("unsupported ort_ep `{other}`, falling back to CPU");
            session_builder
        }
    };
    let session = session_builder
        .with_optimization_level(ort::session::builder::GraphOptimizationLevel::Level3)?
        .with_inter_threads(8)?
        .commit_from_file(cfg.detector_cfg.armor_detect_model_path.as_str())?;

    // let session = Arc::new(Mutex::new(session));
    let pre_task_handler = pre_process(pre_infer_queue.clone());
    let infer_task_handler = infer(pre_infer_queue, session, infer_post_queue.clone());
    let post_task_handler = post_process(infer_post_queue, solved_queue.clone(), cfg, rec);
    let estimate_task_handler = estimate_process(solved_queue, track_queue.clone());
    let control_task_handler = control_loop_250hz(track_queue, feedback_queue);

    let tim = std::time::Instant::now();
    let (_, _, _, _, _) = tokio::join!(
        pre_task_handler,
        infer_task_handler,
        post_task_handler,
        estimate_task_handler,
        control_task_handler
    );
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await; // wait for post process to finish
    info!("multi_thread_pipeline finished in {:?}", tim.elapsed());

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn required_file_rejects_directory() {
        let err = ensure_required_file(Path::new(env!("CARGO_MANIFEST_DIR")), "test file")
            .expect_err("a directory must not satisfy a file precondition");

        assert!(matches!(err, RbtError::PreconditionFailed(_)));
    }
}
