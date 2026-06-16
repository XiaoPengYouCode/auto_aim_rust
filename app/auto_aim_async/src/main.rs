extern crate ndarray as nd;
extern crate rerun as rr;

use crate::rbt_threads::{
    ArmorPipelineQueues, EnergyMechanismPipelineQueues, EnergyMechanismTrackPacket,
    PlannerTrackSnapshot, RuntimePipelineCompletion, RuntimePipelineQueues, can_io_process,
    control_loop_250hz, energy_mechanism_estimate_process, energy_mechanism_infer,
    energy_mechanism_post_process, estimate_process, infer, post_process, pre_process,
    video_input_path,
};
use auto_aim_rust::rbt_infra::rbt_log;
use lib as auto_aim_rust;
use lib::rbt_infra::rbt_err::{RbtError, RbtResult};
use lib::rbt_infra::rbt_global::GENERIC_RBT_CFG;
use lib::rbt_infra::rbt_ort_ep::configure_session_builder;
use lib::rbt_infra::rbt_queue_async::RbtSPSCQueueAsync;
use lib::rbt_mod::rbt_comm::rbt_comm_frame::{CtrlData, SensData};
use lib::rbt_mod::rbt_detector::rbt_frame::RbtFrame;
use lib::rbt_mod::rbt_energy_mechanism::{EnergyMechanismFrame, EnergyMechanismSolvedFrame};
use lib::rbt_mod::rbt_runtime_router::RuntimeRouter;
use lib::rbt_mod::rbt_solver::RbtSolvedResults;
use log::info;
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

fn rerun_recording_stream() -> RbtResult<rr::RecordingStream> {
    if !GENERIC_RBT_CFG.read().unwrap().general_cfg.img_dbg {
        return Ok(rr::RecordingStream::disabled());
    }

    let rerun_path = Path::new("rerun-log").join("rbt_async.rrd");
    if let Some(parent) = rerun_path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    Ok(rr::RecordingStreamBuilder::new("rbt_async").save(rerun_path)?)
}

#[tokio::main]
async fn main() -> RbtResult<()> {
    // init logger
    let _logger_guard = rbt_log::logger_init()?;
    // init rerun logger
    let rec = rerun_recording_stream()?;

    let pre_infer_queue = Arc::new(RbtSPSCQueueAsync::<RbtFrame>::new(1));
    let infer_post_queue = Arc::new(RbtSPSCQueueAsync::<RbtFrame>::new(1));
    let solved_queue = Arc::new(RbtSPSCQueueAsync::<RbtSolvedResults>::new(1));
    let track_queue = Arc::new(RbtSPSCQueueAsync::<PlannerTrackSnapshot>::new(1));
    let energy_pre_infer_queue = Arc::new(RbtSPSCQueueAsync::<EnergyMechanismFrame>::new(1));
    let energy_infer_post_queue = Arc::new(RbtSPSCQueueAsync::<EnergyMechanismFrame>::new(1));
    let energy_solved_queue = Arc::new(RbtSPSCQueueAsync::<EnergyMechanismSolvedFrame>::new(1));
    let energy_track_queue = Arc::new(RbtSPSCQueueAsync::<EnergyMechanismTrackPacket>::new(1));
    let feedback_queue = Arc::new(RbtSPSCQueueAsync::<SensData>::new(1));
    let control_tx_queue = Arc::new(RbtSPSCQueueAsync::<CtrlData>::new(1));
    let armor_queues = ArmorPipelineQueues::new(
        pre_infer_queue.clone(),
        infer_post_queue.clone(),
        solved_queue.clone(),
        track_queue.clone(),
    );
    let energy_mechanism_queues = EnergyMechanismPipelineQueues::new(
        energy_pre_infer_queue.clone(),
        energy_infer_post_queue.clone(),
        energy_solved_queue.clone(),
        energy_track_queue.clone(),
    );
    let runtime_queues = RuntimePipelineQueues::new(armor_queues, energy_mechanism_queues);
    let runtime_router = RuntimeRouter::default();
    let runtime_completion = RuntimePipelineCompletion::new();
    let cfg = GENERIC_RBT_CFG.read().unwrap().clone();

    let model_path = Path::new(cfg.detector_cfg.armor.model_path.as_str());
    ensure_required_file(model_path, "armor model file")?;
    let energy_mechanism_model_path =
        Path::new(cfg.detector_cfg.energy_mechanism.model_path.as_str());
    ensure_required_file(energy_mechanism_model_path, "energy mechanism model file")?;
    let video_path = video_input_path();
    ensure_required_file(&video_path, "video input file")?;

    // build armor onnxruntime session
    let session_builder = Session::builder()?;
    let (session_builder, ort_ep) = configure_session_builder(
        session_builder,
        cfg.detector_cfg.ort_ep.as_str(),
        cfg.detector_cfg.armor.engine_path.as_str(),
    )?;
    info!("using ONNX Runtime execution provider: {}", ort_ep.as_str());
    let session = session_builder
        .with_optimization_level(ort::session::builder::GraphOptimizationLevel::Level3)?
        .with_inter_threads(8)?
        .commit_from_file(cfg.detector_cfg.armor.model_path.as_str())?;

    let energy_session_builder = Session::builder()?;
    let (energy_session_builder, energy_ort_ep) = configure_session_builder(
        energy_session_builder,
        cfg.detector_cfg.ort_ep.as_str(),
        cfg.detector_cfg.energy_mechanism.engine_path.as_str(),
    )?;
    info!(
        "using ONNX Runtime execution provider for energy mechanism: {}",
        energy_ort_ep.as_str()
    );
    let energy_session = energy_session_builder
        .with_optimization_level(ort::session::builder::GraphOptimizationLevel::Level3)?
        .with_inter_threads(8)?
        .commit_from_file(cfg.detector_cfg.energy_mechanism.model_path.as_str())?;

    // let session = Arc::new(Mutex::new(session));
    let pre_task_handler = pre_process(
        pre_infer_queue.clone(),
        energy_pre_infer_queue.clone(),
        feedback_queue.clone(),
        cfg.detector_cfg.clone(),
        runtime_router.clone(),
        runtime_completion.clone(),
    );
    let infer_task_handler = infer(
        pre_infer_queue.clone(),
        session,
        infer_post_queue.clone(),
        runtime_router.clone(),
        runtime_completion.clone(),
    );
    let post_task_handler = post_process(
        infer_post_queue.clone(),
        solved_queue.clone(),
        cfg.clone(),
        rec.clone(),
        runtime_router.clone(),
        runtime_completion.clone(),
    );
    let energy_infer_task_handler = energy_mechanism_infer(
        energy_pre_infer_queue.clone(),
        energy_session,
        energy_infer_post_queue.clone(),
        runtime_router.clone(),
        runtime_completion.clone(),
    );
    let energy_post_task_handler = energy_mechanism_post_process(
        energy_infer_post_queue.clone(),
        energy_solved_queue.clone(),
        cfg.clone(),
        runtime_router.clone(),
        runtime_completion.clone(),
    );
    let energy_estimate_task_handler = energy_mechanism_estimate_process(
        energy_solved_queue.clone(),
        energy_track_queue.clone(),
        rec.clone(),
        runtime_router.clone(),
        runtime_completion.clone(),
    );
    let estimate_task_handler = estimate_process(
        solved_queue.clone(),
        track_queue.clone(),
        rec,
        runtime_router.clone(),
        runtime_completion.clone(),
    );
    let control_task_handler = control_loop_250hz(
        track_queue.clone(),
        energy_track_queue.clone(),
        feedback_queue.clone(),
        control_tx_queue.clone(),
        runtime_router,
        runtime_queues,
        runtime_completion.clone(),
    );
    let can_task_handler =
        can_io_process(control_tx_queue, feedback_queue, cfg, runtime_completion);

    let tim = std::time::Instant::now();
    let (_, _, _, _, _, _, _, _, _) = tokio::join!(
        pre_task_handler,
        infer_task_handler,
        post_task_handler,
        energy_infer_task_handler,
        energy_post_task_handler,
        energy_estimate_task_handler,
        estimate_task_handler,
        control_task_handler,
        can_task_handler
    );
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await; // wait for post process to finish
    info!("multi_thread_pipeline finished in {:?}", tim.elapsed());

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ort::ep;
    use ort::value::{TensorElementType, ValueType};

    #[test]
    fn required_file_rejects_directory() {
        let err = ensure_required_file(Path::new(env!("CARGO_MANIFEST_DIR")), "test file")
            .expect_err("a directory must not satisfy a file precondition");

        assert!(matches!(err, RbtError::PreconditionFailed(_)));
    }

    #[test]
    fn armor_model_metadata_matches_pipeline_contract() {
        let model_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("model")
            .join("armor")
            .join("Armor.onnx");
        if !model_path.is_file() {
            return;
        }

        let session = Session::builder()
            .expect("session builder should be available")
            .with_execution_providers([ep::CPUExecutionProvider::default()
                .with_arena_allocator(true)
                .build()])
            .expect("CPU execution provider should be configurable")
            .commit_from_file(&model_path)
            .expect("Armor.onnx should load");

        let input = &session.inputs()[0];
        assert_eq!(input.name(), "images");
        match input.dtype() {
            ValueType::Tensor { ty, shape, .. } => {
                assert_eq!(*ty, TensorElementType::Float16);
                assert_eq!(&**shape, &[1, 3, 640, 640]);
            }
            other => panic!("unexpected input type: {other:?}"),
        }

        let output = &session.outputs()[0];
        assert_eq!(output.name(), "output");
        match output.dtype() {
            ValueType::Tensor { ty, shape, .. } => {
                assert_eq!(*ty, TensorElementType::Float32);
                assert_eq!(&**shape, &[1, 25_200, 22]);
            }
            other => panic!("unexpected output type: {other:?}"),
        }
    }

    #[test]
    fn energy_mechanism_model_metadata_matches_pipeline_contract() {
        let model_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("model")
            .join("engine_mechanism")
            .join("EngineMechanism.onnx");
        if !model_path.is_file() {
            return;
        }

        let session = Session::builder()
            .expect("session builder should be available")
            .with_execution_providers([ep::CPUExecutionProvider::default()
                .with_arena_allocator(true)
                .build()])
            .expect("CPU execution provider should be configurable")
            .commit_from_file(&model_path)
            .expect("EngineMechanism.onnx should load");

        let input = &session.inputs()[0];
        assert_eq!(input.name(), "images");
        match input.dtype() {
            ValueType::Tensor { ty, shape, .. } => {
                assert_eq!(*ty, TensorElementType::Float32);
                assert_eq!(&**shape, &[1, 3, 640, 640]);
            }
            other => panic!("unexpected input type: {other:?}"),
        }

        let output = &session.outputs()[0];
        assert_eq!(output.name(), "output0");
        match output.dtype() {
            ValueType::Tensor { ty, shape, .. } => {
                assert_eq!(*ty, TensorElementType::Float32);
                assert_eq!(shape.len(), 3);
                assert_eq!(shape[0], 1);
                let channels = shape[1];
                assert!(
                    matches!(channels, 16 | 18 | 21 | 23),
                    "energy mechanism output channels should match 2/4-class 5-keypoint pose output, got {channels}"
                );
            }
            other => panic!("unexpected output type: {other:?}"),
        }
    }
}
