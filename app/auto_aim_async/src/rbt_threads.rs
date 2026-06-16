use log::{debug, error, info, warn};
use ort::inputs;
use ort::value::TensorRef;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdout, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant as StdInstant};
use tokio::task::JoinHandle;
use tokio::time::Instant;

// use crate::rbt_cfg::{self, DetectorConfig, RbtCfg};
// use lib::rbt_mod::rbt_armor::ArmorKeyPoints;
use lib::rbt_mod::{
    rbt_energy_mechanism::{
        EnergyMechanismControlInput, EnergyMechanismController, EnergyMechanismFrame,
        EnergyMechanismMode, EnergyMechanismSolvedFrame, EnergyMechanismTracker,
        EnergyMechanismYoloDecodeStats, EnergyMechanismYoloPostprocessCfg,
        decode_energy_mechanism_output_with_stats, preprocess_energy_mechanism_letterbox_f32,
        solve_energy_mechanism,
    },
    rbt_estimator::rbt_enemy_dynamic_model::EnemyId,
    rbt_solver::RbtSolvedResults,
};
use lib::{
    rbt_infra::rbt_cfg::{DetectorCfg, RbtCfg},
    rbt_infra::{
        rbt_global::{GENERIC_RBT_CFG, IS_RUNNING},
        rbt_queue_async::RbtSPSCQueueAsync,
    },
    rbt_mod::{
        rbt_comm::rbt_comm_device::rbt_can::{
            FeedbackPairDecoder, SocketCanDevice, control_to_can_payload,
        },
        rbt_comm::rbt_comm_frame::{
            AimingState, CAN_FRAME_SIZE, CONTROL_LOOP_PERIOD_MS, CtrlData,
            DEFAULT_BULLET_SPEED_MPS, FEEDBACK_STALE_TIMEOUT_MS, SelfFraction, SensData,
            ShotBuffMode, ShotMode, TaskMode,
        },
        rbt_detector::{
            rbt_frame::{ARMOR_OUTPUT_COLS, ARMOR_OUTPUT_ROWS, RbtFrame, RbtFrameStage},
            rbt_yolo::{
                ArmorYoloDecodeStats, ArmorYoloPostprocessCfg, decode_armor_output_with_stats,
                preprocess_letterbox_f16,
            },
        },
        rbt_estimator::{EnemyTrackSnapshot, RbtHandlerPoll},
        rbt_fire_control::{FireControlController, FireControlInput},
        rbt_runtime_router::RuntimeRouter,
        rbt_solver::enemys_solver,
    },
};

const DEFAULT_VIDEO_FILE: &str = "offline_capture_bundle.avi";
const RAW_RGB_CHANNELS: usize = 3;
const FIRE_CONTROL_SNAPSHOT_STALE_MS: f64 = 180.0;
const CONTROL_STATUS_LOG_PERIOD_TICKS: u64 = 50;
const CAN_IO_POP_TIMEOUT_MS: u64 = 20;
const PIPELINE_POP_TIMEOUT_MS: u64 = 100;
const RERUN_FILTER_RAW_CENTER_COLOR: u32 = 0xFFBE14FF;
const RERUN_FILTER_RAW_ARMOR_COLOR: u32 = 0xFF5014FF;
const RERUN_FILTER_FILTERED_CENTER_COLOR: u32 = 0x14DC78FF;
const RERUN_FILTER_FILTERED_ARMOR_COLOR: u32 = 0x288CFFFF;
const RERUN_FILTER_VELOCITY_COLOR: u32 = 0xFFFFFFFF;
const RERUN_FILTER_VELOCITY_ARROW_SCALE_S: f64 = 0.2;
const RERUN_ENERGY_CENTER_COLOR: u32 = 0xF59E0BFF;
const RERUN_ENERGY_TARGET_CENTER_COLOR: u32 = 0x10B981FF;
const RERUN_ENERGY_PREDICT_COLOR: u32 = 0x38BDF8FF;

#[derive(Debug, Clone)]
pub struct PlannerTrackSnapshot {
    seq: u64,
    target: Option<EnemyTrackSnapshot>,
    publish_tp: Instant,
}

#[derive(Debug, Clone)]
pub struct EnergyMechanismTrackPacket {
    seq: u64,
    target: Option<lib::rbt_mod::rbt_energy_mechanism::EnergyMechanismTrackSnapshot>,
    publish_tp: Instant,
}

#[derive(Clone)]
pub struct ArmorPipelineQueues {
    pre_infer_queue: Arc<RbtSPSCQueueAsync<RbtFrame>>,
    infer_post_queue: Arc<RbtSPSCQueueAsync<RbtFrame>>,
    solved_queue: Arc<RbtSPSCQueueAsync<RbtSolvedResults>>,
    track_queue: Arc<RbtSPSCQueueAsync<PlannerTrackSnapshot>>,
}

#[derive(Clone)]
pub struct EnergyMechanismPipelineQueues {
    pre_infer_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
    infer_post_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
    solved_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismSolvedFrame>>,
    track_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismTrackPacket>>,
}

#[derive(Clone)]
pub struct RuntimePipelineQueues {
    armor: ArmorPipelineQueues,
    energy_mechanism: EnergyMechanismPipelineQueues,
}

#[derive(Debug, Clone, Default)]
pub struct RuntimePipelineCompletion {
    source_done: Arc<AtomicBool>,
    armor_infer_done: Arc<AtomicBool>,
    armor_post_done: Arc<AtomicBool>,
    armor_estimate_done: Arc<AtomicBool>,
    energy_infer_done: Arc<AtomicBool>,
    energy_post_done: Arc<AtomicBool>,
    energy_estimate_done: Arc<AtomicBool>,
}

impl RuntimePipelineCompletion {
    pub fn new() -> Self {
        Self::default()
    }

    fn source_done(&self) -> bool {
        self.source_done.load(Ordering::SeqCst)
    }

    fn armor_infer_done(&self) -> bool {
        self.armor_infer_done.load(Ordering::SeqCst)
    }

    fn armor_post_done(&self) -> bool {
        self.armor_post_done.load(Ordering::SeqCst)
    }

    fn armor_estimate_done(&self) -> bool {
        self.armor_estimate_done.load(Ordering::SeqCst)
    }

    fn energy_infer_done(&self) -> bool {
        self.energy_infer_done.load(Ordering::SeqCst)
    }

    fn energy_post_done(&self) -> bool {
        self.energy_post_done.load(Ordering::SeqCst)
    }

    fn energy_estimate_done(&self) -> bool {
        self.energy_estimate_done.load(Ordering::SeqCst)
    }

    fn mark_source_done(&self) {
        self.source_done.store(true, Ordering::SeqCst);
    }

    fn mark_armor_infer_done(&self) {
        self.armor_infer_done.store(true, Ordering::SeqCst);
    }

    fn mark_armor_post_done(&self) {
        self.armor_post_done.store(true, Ordering::SeqCst);
    }

    fn mark_armor_estimate_done(&self) {
        self.armor_estimate_done.store(true, Ordering::SeqCst);
    }

    fn mark_energy_infer_done(&self) {
        self.energy_infer_done.store(true, Ordering::SeqCst);
    }

    fn mark_energy_post_done(&self) {
        self.energy_post_done.store(true, Ordering::SeqCst);
    }

    fn mark_energy_estimate_done(&self) {
        self.energy_estimate_done.store(true, Ordering::SeqCst);
    }
}

impl ArmorPipelineQueues {
    pub fn new(
        pre_infer_queue: Arc<RbtSPSCQueueAsync<RbtFrame>>,
        infer_post_queue: Arc<RbtSPSCQueueAsync<RbtFrame>>,
        solved_queue: Arc<RbtSPSCQueueAsync<RbtSolvedResults>>,
        track_queue: Arc<RbtSPSCQueueAsync<PlannerTrackSnapshot>>,
    ) -> Self {
        Self {
            pre_infer_queue,
            infer_post_queue,
            solved_queue,
            track_queue,
        }
    }

    fn clear(&self) {
        self.pre_infer_queue.clear();
        self.infer_post_queue.clear();
        self.solved_queue.clear();
        self.track_queue.clear();
    }
}

impl EnergyMechanismPipelineQueues {
    pub fn new(
        pre_infer_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
        infer_post_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
        solved_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismSolvedFrame>>,
        track_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismTrackPacket>>,
    ) -> Self {
        Self {
            pre_infer_queue,
            infer_post_queue,
            solved_queue,
            track_queue,
        }
    }

    fn clear(&self) {
        self.pre_infer_queue.clear();
        self.infer_post_queue.clear();
        self.solved_queue.clear();
        self.track_queue.clear();
    }
}

impl RuntimePipelineQueues {
    pub fn new(
        armor: ArmorPipelineQueues,
        energy_mechanism: EnergyMechanismPipelineQueues,
    ) -> Self {
        Self {
            armor,
            energy_mechanism,
        }
    }

    fn clear_for_route_transition(&self) {
        self.armor.clear();
        self.energy_mechanism.clear();
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct PreprocessSummary {
    frames: u64,
    read_total: Duration,
    preprocess_total: Duration,
    elapsed: Duration,
}

impl PreprocessSummary {
    fn avg_read(self) -> Duration {
        avg_duration(self.read_total, self.frames)
    }

    fn avg_preprocess(self) -> Duration {
        avg_duration(self.preprocess_total, self.frames)
    }
}

fn avg_duration(total: Duration, count: u64) -> Duration {
    if count == 0 {
        Duration::ZERO
    } else {
        total / count as u32
    }
}

fn enemy_rerun_name(enemy_id: EnemyId) -> &'static str {
    match enemy_id {
        EnemyId::Hero1 => "hero1",
        EnemyId::Engineer2 => "engineer2",
        EnemyId::Infantry3 => "infantry3",
        EnemyId::Infantry4 => "infantry4",
        EnemyId::Sentry7 => "sentry7",
        EnemyId::Outpost8 => "outpost8",
        EnemyId::Invalid => "invalid",
    }
}

fn solved_enemy_center_position(enemy: &lib::rbt_mod::rbt_solver::RbtSolvedResult) -> [f32; 3] {
    let center = enemy.coord.to_xy();
    let armor_z = enemy
        .armors
        .first()
        .map(|armor| armor.pose().translation.vector.z)
        .unwrap_or(0.0);
    [center.x as f32, center.y as f32, armor_z as f32]
}

fn solved_enemy_armor_positions(
    enemy: &lib::rbt_mod::rbt_solver::RbtSolvedResult,
) -> Vec<[f32; 3]> {
    enemy
        .armors
        .iter()
        .map(|armor| {
            let translation = armor.pose().translation.vector;
            [
                translation.x as f32,
                translation.y as f32,
                translation.z as f32,
            ]
        })
        .collect()
}

fn filtered_center_position(snapshot: EnemyTrackSnapshot) -> [f32; 3] {
    [
        (snapshot.center_xy_m.x * 1_000.0) as f32,
        (snapshot.center_xy_m.y * 1_000.0) as f32,
        (snapshot.armor_z_m * 1_000.0) as f32,
    ]
}

fn filtered_velocity_arrow(snapshot: EnemyTrackSnapshot) -> [f32; 3] {
    [
        (snapshot.center_velocity_xy_mps.x * 1_000.0 * RERUN_FILTER_VELOCITY_ARROW_SCALE_S) as f32,
        (snapshot.center_velocity_xy_mps.y * 1_000.0 * RERUN_FILTER_VELOCITY_ARROW_SCALE_S) as f32,
        (snapshot.armor_z_velocity_mps * 1_000.0 * RERUN_FILTER_VELOCITY_ARROW_SCALE_S) as f32,
    ]
}

fn filtered_armor_positions(snapshot: EnemyTrackSnapshot) -> Vec<[f32; 3]> {
    let center_x_mm = snapshot.center_xy_m.x * 1_000.0;
    let center_y_mm = snapshot.center_xy_m.y * 1_000.0;
    let z_mm = snapshot.armor_z_m * 1_000.0;
    let primary_radius_mm = snapshot.primary_radius_m * 1_000.0;
    let secondary_radius_mm =
        (snapshot.primary_radius_m + snapshot.secondary_radius_delta_m) * 1_000.0;
    let height_delta_mm = snapshot.height_delta_m * 1_000.0;
    let armor_count = snapshot.armor_count.clamp(1, 4);
    let radial_sign = if armor_count == 3 { 1.0 } else { -1.0 };

    (0..armor_count)
        .map(|idx| {
            let angle =
                snapshot.body_yaw_rad + idx as f64 * std::f64::consts::TAU / armor_count as f64;
            let radius_mm = if armor_count == 4 && (idx == 1 || idx == 3) {
                secondary_radius_mm
            } else {
                primary_radius_mm
            };
            let z_offset_mm = if armor_count == 4 {
                if idx == 1 || idx == 3 {
                    height_delta_mm
                } else {
                    0.0
                }
            } else if idx == 1 {
                snapshot.secondary_radius_delta_m * 1_000.0
            } else if idx == 2 {
                height_delta_mm
            } else {
                0.0
            };

            [
                (center_x_mm + radial_sign * radius_mm * angle.cos()) as f32,
                (center_y_mm + radial_sign * radius_mm * angle.sin()) as f32,
                (z_mm + z_offset_mm) as f32,
            ]
        })
        .collect()
}

fn log_rerun_filter_snapshot(
    rec: &rr::RecordingStream,
    seq: u64,
    raw_enemies: &RbtSolvedResults,
    filtered: Option<EnemyTrackSnapshot>,
    raw_visible: &mut bool,
    filtered_visible: &mut bool,
) -> Result<(), rr::RecordingStreamError> {
    if !rec.is_enabled() {
        return Ok(());
    }

    rec.set_time_sequence("estimate_seq", seq as i64);

    let mut raw_centers = Vec::new();
    let mut raw_armors = Vec::new();
    let mut raw_labels = Vec::new();
    for (enemy_id, enemy) in raw_enemies.iter() {
        let Some(enemy) = enemy else {
            continue;
        };
        raw_centers.push(solved_enemy_center_position(enemy));
        raw_labels.push(enemy_rerun_name(*enemy_id));
        raw_armors.extend(solved_enemy_armor_positions(enemy));
    }

    if raw_centers.is_empty() {
        if *raw_visible {
            rec.log(
                "world/filter/raw/centers",
                &rr::Points3D::new([] as [[f32; 3]; 0]),
            )?;
            rec.log(
                "world/filter/raw/armors",
                &rr::Points3D::new([] as [[f32; 3]; 0]),
            )?;
            *raw_visible = false;
        }
    } else {
        rec.log(
            "world/filter/raw/centers",
            &rr::Points3D::new(raw_centers)
                .with_colors([RERUN_FILTER_RAW_CENTER_COLOR])
                .with_radii([24.0])
                .with_labels(raw_labels),
        )?;
        rec.log(
            "world/filter/raw/armors",
            &rr::Points3D::new(raw_armors)
                .with_colors([RERUN_FILTER_RAW_ARMOR_COLOR])
                .with_radii([12.0]),
        )?;
        *raw_visible = true;
    }

    if let Some(snapshot) = filtered {
        let center = filtered_center_position(snapshot);
        let velocity = filtered_velocity_arrow(snapshot);
        rec.log(
            "world/filter/filtered/center",
            &rr::Points3D::new([center])
                .with_colors([RERUN_FILTER_FILTERED_CENTER_COLOR])
                .with_radii([30.0])
                .with_labels([enemy_rerun_name(snapshot.enemy_id)]),
        )?;
        rec.log(
            "world/filter/filtered/armors",
            &rr::Points3D::new(filtered_armor_positions(snapshot))
                .with_colors([RERUN_FILTER_FILTERED_ARMOR_COLOR])
                .with_radii([14.0]),
        )?;
        rec.log(
            "world/filter/filtered/velocity",
            &rr::Arrows3D::from_vectors([velocity])
                .with_origins([center])
                .with_colors([RERUN_FILTER_VELOCITY_COLOR])
                .with_radii([4.0]),
        )?;
        *filtered_visible = true;
    } else {
        if *filtered_visible {
            rec.log(
                "world/filter/filtered/center",
                &rr::Points3D::new([] as [[f32; 3]; 0]),
            )?;
            rec.log(
                "world/filter/filtered/armors",
                &rr::Points3D::new([] as [[f32; 3]; 0]),
            )?;
            rec.log(
                "world/filter/filtered/velocity",
                &rr::Arrows3D::from_vectors([] as [[f32; 3]; 0]),
            )?;
            *filtered_visible = false;
        }
    }

    Ok(())
}

fn energy_position_mm(x: f64, y: f64, z: f64) -> [f32; 3] {
    [
        (x * 1_000.0) as f32,
        (y * 1_000.0) as f32,
        (z * 1_000.0) as f32,
    ]
}

fn log_rerun_energy_mechanism_snapshot(
    rec: &rr::RecordingStream,
    seq: u64,
    target: Option<lib::rbt_mod::rbt_energy_mechanism::EnergyMechanismTrackSnapshot>,
    center_visible: &mut bool,
    blade_center_visible: &mut bool,
    horizon_visible: &mut bool,
) -> Result<(), rr::RecordingStreamError> {
    if !rec.is_enabled() {
        return Ok(());
    }

    rec.set_time_sequence("energy_estimate_seq", seq as i64);

    if let Some(snapshot) = target {
        let center = energy_position_mm(
            snapshot.rune_center_world_m.x,
            snapshot.rune_center_world_m.y,
            snapshot.rune_center_world_m.z,
        );
        let target_center = energy_position_mm(
            snapshot.target_center_world_m.x,
            snapshot.target_center_world_m.y,
            snapshot.target_center_world_m.z,
        );
        let predict_horizon = snapshot
            .predict_target_horizon(&[0.02, 0.04, 0.06, 0.08, 0.10])
            .into_iter()
            .map(|point| energy_position_mm(point.x, point.y, point.z))
            .collect::<Vec<_>>();

        rec.log(
            "world/energy_mechanism/center",
            &rr::Points3D::new([center])
                .with_colors([RERUN_ENERGY_CENTER_COLOR])
                .with_radii([26.0]),
        )?;
        rec.log(
            "world/energy_mechanism/blade_center",
            &rr::Points3D::new([target_center])
                .with_colors([RERUN_ENERGY_TARGET_CENTER_COLOR])
                .with_radii([22.0]),
        )?;
        rec.log(
            "world/energy_mechanism/predict_horizon",
            &rr::LineStrips3D::new([predict_horizon]).with_colors([RERUN_ENERGY_PREDICT_COLOR]),
        )?;

        rec.log(
            "time/energy_mechanism/roll_rad",
            &rr::Scalars::new([snapshot.roll_rad]),
        )?;
        rec.log(
            "time/energy_mechanism/roll_rate_rad_s",
            &rr::Scalars::new([snapshot.roll_rate_rad_s]),
        )?;
        rec.log(
            "time/energy_mechanism/track_valid",
            &rr::Scalars::new([if snapshot.track_valid { 1.0 } else { 0.0 }]),
        )?;
        rec.log(
            "time/energy_mechanism/lost",
            &rr::Scalars::new([if snapshot.lost { 1.0 } else { 0.0 }]),
        )?;

        *center_visible = true;
        *blade_center_visible = true;
        *horizon_visible = true;
    } else {
        if *center_visible {
            rec.log(
                "world/energy_mechanism/center",
                &rr::Points3D::new([] as [[f32; 3]; 0]),
            )?;
            *center_visible = false;
        }
        if *blade_center_visible {
            rec.log(
                "world/energy_mechanism/blade_center",
                &rr::Points3D::new([] as [[f32; 3]; 0]),
            )?;
            *blade_center_visible = false;
        }
        if *horizon_visible {
            rec.log(
                "world/energy_mechanism/predict_horizon",
                &rr::LineStrips3D::new([] as [Vec<[f32; 3]>; 0]),
            )?;
            *horizon_visible = false;
        }
    }

    Ok(())
}

pub fn video_input_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("videos")
        .join(DEFAULT_VIDEO_FILE)
}

fn default_feedback(bullet_speed_mps: f64, task_mode: TaskMode) -> SensData {
    SensData {
        task_mode,
        self_fraction: SelfFraction::Blue,
        bullet_speed: if bullet_speed_mps.is_finite() && bullet_speed_mps > 0.0 {
            bullet_speed_mps as f32
        } else {
            DEFAULT_BULLET_SPEED_MPS
        },
        gimbal_roll: 0.0,
        gimbal_yaw: 0.0,
        gimbal_pitch: 0.0,
        yaw_speed: 0.0,
        mcu_fire_permit: false,
        raw_task_mode: task_mode.into(),
        mapped_task_mode: task_mode,
    }
}

/// 视频预处理阶段：读取原始视频帧，再 resize + letterbox 到模型输入张量。
pub fn pre_process(
    queue: Arc<RbtSPSCQueueAsync<RbtFrame>>,
    energy_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
    feedback_queue: Arc<RbtSPSCQueueAsync<SensData>>,
    detector_cfg: DetectorCfg,
    runtime_router: RuntimeRouter,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let completion_guard = completion.clone();
        let result = tokio::task::spawn_blocking(move || {
            run_video_preprocess_loop(
                queue,
                energy_queue,
                feedback_queue,
                detector_cfg,
                runtime_router,
            )
        })
        .await;

        match result {
            Ok(Ok(summary)) => {
                info!(
                    "pre_process: video source finished after {} frames in {:?}, avg read {:?}, avg preprocess {:?}",
                    summary.frames,
                    summary.elapsed,
                    summary.avg_read(),
                    summary.avg_preprocess()
                );
                IS_RUNNING.store(false, Ordering::SeqCst);
            }
            Err(err) => {
                error!("pre_process: video worker join failed: {err}");
                IS_RUNNING.store(false, Ordering::SeqCst);
            }
            Ok(Err(err)) => {
                error!("pre_process: {err}");
                IS_RUNNING.store(false, Ordering::SeqCst);
            }
        }
        completion_guard.mark_source_done();
    })
}

fn run_video_preprocess_loop(
    queue: Arc<RbtSPSCQueueAsync<RbtFrame>>,
    energy_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
    feedback_queue: Arc<RbtSPSCQueueAsync<SensData>>,
    _detector_cfg: DetectorCfg,
    runtime_router: RuntimeRouter,
) -> Result<PreprocessSummary, String> {
    let video_path = video_input_path();
    let mut reader = FfmpegVideoReader::open(&video_path)?;
    let mut frame_id = 0_u64;
    let mut summary = PreprocessSummary::default();
    let started = StdInstant::now();

    info!(
        "pre_process: streaming {} as original {}x{} rgb frames",
        video_path.display(),
        reader.width,
        reader.height
    );

    let cfg = GENERIC_RBT_CFG.read().unwrap().clone();
    let offline_feedback = default_feedback(
        cfg.general_cfg.bullet_speed,
        cfg.general_cfg.offline_task_mode(),
    );
    feedback_queue.push_latest(offline_feedback);

    loop {
        if !IS_RUNNING.load(Ordering::SeqCst) {
            info!("pre_process: stopping video source as IS_RUNNING is false");
            break;
        }

        let read_started = StdInstant::now();
        let Some(frame_img) = reader.read_frame()? else {
            break;
        };
        summary.read_total += read_started.elapsed();
        feedback_queue.push_latest(offline_feedback);
        let frame_img = frame_img.rotate180();

        let route_state = runtime_router.state();
        if route_state.armor_pipeline_active() {
            frame_id = frame_id.wrapping_add(1);
            let mut rbt_frame = RbtFrame::new();
            let preprocess_started = StdInstant::now();
            let gray_frame = frame_img.to_luma8();
            let transform = preprocess_letterbox_f16(rbt_frame.pre_data(), &frame_img);
            summary.preprocess_total += preprocess_started.elapsed();
            rbt_frame.set_gray_frame(gray_frame);
            rbt_frame.set_letterbox_transform(transform);
            rbt_frame.set_id(frame_id);
            rbt_frame.set_state(RbtFrameStage::Pre);
            queue.push_latest(rbt_frame);
        } else if route_state.energy_mechanism_active() {
            let Some(mode) = EnergyMechanismMode::from_task_mode(route_state.task_mode) else {
                continue;
            };
            frame_id = frame_id.wrapping_add(1);
            let mut energy_frame = EnergyMechanismFrame::new(mode);
            let preprocess_started = StdInstant::now();
            let transform =
                preprocess_energy_mechanism_letterbox_f32(energy_frame.pre_data(), &frame_img);
            summary.preprocess_total += preprocess_started.elapsed();
            energy_frame.set_letterbox_transform(transform);
            energy_frame.set_id(frame_id);
            energy_queue.push_latest(energy_frame);
        } else {
            continue;
        }

        if frame_id == 1 || frame_id.is_multiple_of(60) {
            info!("pre_process: pushed video frame {frame_id}");
        }
    }

    summary.frames = frame_id;
    summary.elapsed = started.elapsed();
    Ok(summary)
}

struct FfmpegVideoReader {
    child: Child,
    stdout: ChildStdout,
    frame_buf: Vec<u8>,
    width: u32,
    height: u32,
}

impl FfmpegVideoReader {
    fn open(path: &Path) -> Result<Self, String> {
        let (width, height) = probe_video_size(path)?;
        let mut child = Command::new("ffmpeg")
            .arg("-hide_banner")
            .arg("-loglevel")
            .arg("error")
            .arg("-i")
            .arg(path)
            .arg("-an")
            .arg("-f")
            .arg("rawvideo")
            .arg("-pix_fmt")
            .arg("rgb24")
            .arg("pipe:1")
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|err| format!("failed to start ffmpeg for {}: {err}", path.display()))?;

        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| "failed to capture ffmpeg stdout".to_string())?;
        let frame_len = (width as usize)
            .checked_mul(height as usize)
            .and_then(|pixels| pixels.checked_mul(RAW_RGB_CHANNELS))
            .ok_or_else(|| format!("raw frame size overflows usize: {width}x{height}"))?;

        Ok(Self {
            child,
            stdout,
            frame_buf: vec![0; frame_len],
            width,
            height,
        })
    }

    fn read_frame(&mut self) -> Result<Option<image::DynamicImage>, String> {
        let mut read_len = 0;
        while read_len < self.frame_buf.len() {
            let n = self
                .stdout
                .read(&mut self.frame_buf[read_len..])
                .map_err(|err| {
                    format!(
                        "failed to read ffmpeg raw frame {}x{}: {err}",
                        self.width, self.height
                    )
                })?;
            if n == 0 {
                if read_len == 0 {
                    return Ok(None);
                }
                return Err(format!(
                    "ffmpeg ended in the middle of a frame: read {read_len}/{} bytes",
                    self.frame_buf.len()
                ));
            }
            read_len += n;
        }

        let image = image::RgbImage::from_raw(self.width, self.height, self.frame_buf.clone())
            .ok_or_else(|| {
                format!(
                    "failed to build rgb image from raw frame {}x{}",
                    self.width, self.height
                )
            })?;

        Ok(Some(image::DynamicImage::ImageRgb8(image)))
    }
}

fn probe_video_size(path: &Path) -> Result<(u32, u32), String> {
    let output = Command::new("ffprobe")
        .arg("-v")
        .arg("error")
        .arg("-select_streams")
        .arg("v:0")
        .arg("-show_entries")
        .arg("stream=width,height")
        .arg("-of")
        .arg("csv=p=0:s=x")
        .arg(path)
        .output()
        .map_err(|err| format!("failed to start ffprobe for {}: {err}", path.display()))?;

    if !output.status.success() {
        return Err(format!(
            "ffprobe failed for {}: {}",
            path.display(),
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    let size = stdout
        .lines()
        .find(|line| !line.trim().is_empty())
        .ok_or_else(|| format!("ffprobe did not report video size for {}", path.display()))?
        .trim();
    let (width, height) = size.split_once('x').ok_or_else(|| {
        format!(
            "unexpected ffprobe video size `{size}` for {}",
            path.display()
        )
    })?;
    let width = width
        .parse::<u32>()
        .map_err(|err| format!("invalid ffprobe width `{width}`: {err}"))?;
    let height = height
        .parse::<u32>()
        .map_err(|err| format!("invalid ffprobe height `{height}`: {err}"))?;

    if width == 0 || height == 0 {
        return Err(format!(
            "ffprobe reported empty video size {width}x{height}"
        ));
    }

    Ok((width, height))
}

impl Drop for FfmpegVideoReader {
    fn drop(&mut self) {
        if let Ok(Some(_)) = self.child.try_wait() {
            return;
        }

        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// 推理阶段：接收预处理后的数据，执行模型推理，并将结果发送到后续处理阶段
pub fn infer(
    pre_infer_queue: Arc<RbtSPSCQueueAsync<RbtFrame>>, // 接收预处理阶段的输出
    mut session: ort::session::Session,                // ONNX Runtime 推理会话
    infer_post_queue: Arc<RbtSPSCQueueAsync<RbtFrame>>, // 发送推理结果到后续处理阶段
    runtime_router: RuntimeRouter,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut infer_count = 0_u64;
        let mut infer_total = Duration::ZERO;
        let output_name = match session
            .outputs()
            .iter()
            .find(|output| output.name() == "output")
            .or_else(|| session.outputs().first())
            .map(|output| output.name().to_string())
        {
            Some(output_name) => output_name,
            None => {
                warn!("infer: armor session has no output");
                IS_RUNNING.store(false, Ordering::SeqCst);
                completion.mark_armor_infer_done();
                return;
            }
        };

        loop {
            if completion.source_done() && pre_infer_queue.is_empty() {
                info!(
                    "infer: stopping after {infer_count} frames, avg infer {:?}",
                    avg_duration(infer_total, infer_count)
                );
                break;
            }
            if let Some(mut frame) = pop_latest_until_running(
                &pre_infer_queue,
                Duration::from_millis(PIPELINE_POP_TIMEOUT_MS),
            )
            .await
            {
                let route_state = runtime_router.state();
                if !route_state.armor_pipeline_active() {
                    continue;
                }
                debug!(
                    "infer: Frame ID {} received form processing, time used: {:?}",
                    frame.id(),
                    frame.time_used()
                );
                frame.set_state(RbtFrameStage::Infer);
                let id = frame.id(); // 获取帧 ID，用于日志记录
                let output_name = output_name.clone();
                // 在阻塞线程中执行推理操作
                let output_result = tokio::task::spawn_blocking(move || {
                    let started = StdInstant::now();
                    let output_array = {
                        let outputs = session
                            .run(inputs![TensorRef::from_array_view(frame.pre_data_ref())
                                .map_err(|err| err.to_string())?])
                            .map_err(|err| err.to_string())?;
                        outputs[output_name.as_str()]
                            .try_extract_array::<f32>()
                            .map_err(|err| err.to_string())?
                            .as_standard_layout()
                            .to_owned()
                            .into_shape_with_order((ARMOR_OUTPUT_ROWS, ARMOR_OUTPUT_COLS))
                            .map_err(|err| {
                                format!(
                                    "failed to reshape armor output to [{ARMOR_OUTPUT_ROWS},{ARMOR_OUTPUT_COLS}]: {err}"
                                )
                            })?
                    };
                    frame.infer_data().assign(&output_array);
                    Ok::<_, String>((session, frame, started.elapsed())) // 返回会话和处理后的帧
                })
                .await;

                // 处理推理结果
                match output_result {
                    Ok(Ok((session_return, output, infer_elapsed))) => {
                        infer_count = infer_count.wrapping_add(1);
                        infer_total += infer_elapsed;
                        let latest_route_state = runtime_router.state();
                        if latest_route_state.transition_seq == route_state.transition_seq
                            && latest_route_state.armor_pipeline_active()
                        {
                            infer_post_queue.push_latest(output); // 将最新推理结果发送到后处理阶段
                        }
                        session = session_return; // 确保会话在闭包外部可用
                    }
                    Ok(Err(err)) => {
                        warn!("infer: Failed to process frame ID {id}: {err}");
                        IS_RUNNING.store(false, Ordering::SeqCst);
                        break;
                    }
                    Err(err) => {
                        warn!("infer: Failed to join worker for frame ID {id}: {err}");
                        IS_RUNNING.store(false, Ordering::SeqCst);
                        break;
                    }
                }
            }
        }
        completion.mark_armor_infer_done();
    })
}

/// 后处理阶段：接收推理结果，执行目标检测框处理，并提取装甲板信息
pub fn post_process(
    frame: Arc<RbtSPSCQueueAsync<RbtFrame>>,
    solved_queue: Arc<RbtSPSCQueueAsync<RbtSolvedResults>>,
    cfg: RbtCfg,
    rec: rr::RecordingStream,
    runtime_router: RuntimeRouter,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut post_count = 0_u64;
        let mut post_total = Duration::ZERO;
        let mut decoded_zero_count = 0_u64;
        let mut decoded_one_count = 0_u64;
        let mut decoded_multi_count = 0_u64;
        let mut solved_zero_count = 0_u64;
        let mut solved_one_count = 0_u64;
        let mut solved_multi_count = 0_u64;
        let mut max_decoded_armors = 0_usize;
        let mut max_solved_armors = 0_usize;
        let mut decode_stats = ArmorYoloDecodeStats::default();
        loop {
            if completion.armor_infer_done() && frame.is_empty() {
                info!(
                    "post_process: stopping after {post_count} frames, avg post {:?}, decoded armors 0/1/>1={decoded_zero_count}/{decoded_one_count}/{decoded_multi_count}, max decoded {max_decoded_armors}, solved armors 0/1/>1={solved_zero_count}/{solved_one_count}/{solved_multi_count}, max solved {max_solved_armors}, decode stats {:?}",
                    avg_duration(post_total, post_count),
                    decode_stats,
                );
                break;
            }
            let armor_cfg = cfg.detector_cfg.armor.clone();
            let self_fraction = cfg.game_cfg.self_fraction();
            let cam_k = cfg.cam_cfg.cam_k();
            let rec = rec.clone();
            if let Some(mut frame) =
                pop_latest_until_running(&frame, Duration::from_millis(PIPELINE_POP_TIMEOUT_MS))
                    .await
            {
                let route_state = runtime_router.state();
                if !route_state.armor_pipeline_active() {
                    continue;
                }
                let time_used = frame.time_used(); // 获取处理时间
                debug!(
                    "post_process: Frame ID {} received in {:?}",
                    frame.id(),
                    time_used
                );
                frame.set_state(RbtFrameStage::Post); // 更新状态为后处理
                let id = frame.id(); // 获取帧 ID，用于日志记录
                // 在阻塞线程中执行后处理操作
                let result = tokio::task::spawn_blocking(move || {
                    let started = StdInstant::now();
                    let output = frame.infer_data_ref();
                    let postprocess_cfg =
                        ArmorYoloPostprocessCfg::from_armor_cfg(&armor_cfg, self_fraction);
                    let (armors, stats) = decode_armor_output_with_stats(
                        &output,
                        frame.letterbox_transform(),
                        &postprocess_cfg,
                    );
                    let decoded_armor_count = armors.values().map(Vec::len).sum::<usize>();
                    let decoded_enemy_count = armors.len();
                    let solved_enemies = enemys_solver(armors, &cam_k, frame.gray_frame(), &rec)?;
                    let solved_armor_count = solved_enemies
                        .values()
                        .filter_map(|result| result.as_ref())
                        .map(|result| result.armors.len())
                        .sum::<usize>();
                    Ok::<_, lib::rbt_infra::rbt_err::RbtError>((
                        frame,
                        solved_enemies,
                        started.elapsed(),
                        decoded_armor_count,
                        decoded_enemy_count,
                        solved_armor_count,
                        stats,
                    ))
                })
                .await;

                if let Ok(Ok((
                    _frame,
                    solved_enemies,
                    post_elapsed,
                    decoded_armor_count,
                    decoded_enemy_count,
                    solved_armor_count,
                    stats,
                ))) = result
                {
                    post_count = post_count.wrapping_add(1);
                    post_total += post_elapsed;
                    match decoded_armor_count {
                        0 => decoded_zero_count = decoded_zero_count.wrapping_add(1),
                        1 => decoded_one_count = decoded_one_count.wrapping_add(1),
                        _ => decoded_multi_count = decoded_multi_count.wrapping_add(1),
                    }
                    match solved_armor_count {
                        0 => solved_zero_count = solved_zero_count.wrapping_add(1),
                        1 => solved_one_count = solved_one_count.wrapping_add(1),
                        _ => solved_multi_count = solved_multi_count.wrapping_add(1),
                    }
                    max_decoded_armors = max_decoded_armors.max(decoded_armor_count);
                    max_solved_armors = max_solved_armors.max(solved_armor_count);
                    decode_stats.rows += stats.rows;
                    decode_stats.score_pass += stats.score_pass;
                    decode_stats.color_pass += stats.color_pass;
                    decode_stats.self_color_pass += stats.self_color_pass;
                    decode_stats.number_pass += stats.number_pass;
                    decode_stats.geometry_pass += stats.geometry_pass;
                    decode_stats.nms_kept += stats.nms_kept;
                    decode_stats.confidence_pass += stats.confidence_pass;
                    if decoded_armor_count > 1 || solved_armor_count > 1 {
                        debug!(
                            "post_process: frame {id} decoded {decoded_armor_count} armors across {decoded_enemy_count} ids, solved {solved_armor_count} armors"
                        );
                    }
                    let latest_route_state = runtime_router.state();
                    if latest_route_state.transition_seq == route_state.transition_seq
                        && latest_route_state.armor_pipeline_active()
                    {
                        solved_queue.push_latest(solved_enemies);
                    }
                    let time_used = _frame.time_used(); // 获取处理时间
                    debug!(
                        "post_process: Frame ID {} processed successfully, time used: {:?}",
                        id, time_used
                    );
                } else if let Ok(Err(err)) = result {
                    warn!("post_process: Failed to solve frame ID {}: {}", id, err);
                } else {
                    warn!("post_process: Failed to process frame ID: {}", id);
                }
            } else {
                continue;
            }
        }
        completion.mark_armor_post_done();
    })
}

pub fn energy_mechanism_infer(
    pre_infer_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
    mut session: ort::session::Session,
    infer_post_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
    runtime_router: RuntimeRouter,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut infer_count = 0_u64;
        let mut infer_total = Duration::ZERO;
        let output_name = match session
            .outputs()
            .iter()
            .find(|output| output.name() == "output")
            .or_else(|| session.outputs().first())
            .map(|output| output.name().to_string())
        {
            Some(output_name) => output_name,
            None => {
                warn!("energy_mechanism_infer: session has no output");
                IS_RUNNING.store(false, Ordering::SeqCst);
                completion.mark_energy_infer_done();
                return;
            }
        };

        loop {
            if completion.source_done() && pre_infer_queue.is_empty() {
                info!(
                    "energy_mechanism_infer: stopping after {infer_count} frames, avg infer {:?}",
                    avg_duration(infer_total, infer_count)
                );
                break;
            }
            let Some(mut frame) = pop_latest_until_running(
                &pre_infer_queue,
                Duration::from_millis(PIPELINE_POP_TIMEOUT_MS),
            )
            .await
            else {
                continue;
            };
            let route_state = runtime_router.state();
            if !route_state.energy_mechanism_active() {
                continue;
            }
            let id = frame.id();
            let output_name = output_name.clone();
            let output_result = tokio::task::spawn_blocking(move || {
                let started = StdInstant::now();
                let output_2d = {
                    let outputs = session
                        .run(inputs![TensorRef::from_array_view(frame.pre_data_ref())
                            .map_err(|err| err.to_string())?])
                        .map_err(|err| err.to_string())?;
                    let output = outputs[output_name.as_str()]
                        .try_extract_array::<f32>()
                        .map_err(|err| err.to_string())?
                        .as_standard_layout()
                        .to_owned();
                    let shape = output.shape().to_vec();
                    match shape.as_slice() {
                        [1, channels, anchors] => output
                            .into_shape_with_order((*channels, *anchors))
                            .map_err(|err| {
                                format!(
                                    "failed to reshape energy mechanism output [{channels},{anchors}]: {err}"
                                )
                            })?,
                        [channels, anchors] => output
                            .into_shape_with_order((*channels, *anchors))
                            .map_err(|err| {
                                format!(
                                    "failed to reshape energy mechanism output [{channels},{anchors}]: {err}"
                                )
                            })?,
                        other => {
                            return Err(format!(
                                "unsupported energy mechanism output shape: {other:?}"
                            ));
                        }
                    }
                };
                frame.set_infer_output(output_2d);
                Ok::<_, String>((session, frame, started.elapsed()))
            })
            .await;

            match output_result {
                Ok(Ok((session_return, output, infer_elapsed))) => {
                    infer_count = infer_count.wrapping_add(1);
                    infer_total += infer_elapsed;
                    let latest_route_state = runtime_router.state();
                    if latest_route_state.transition_seq == route_state.transition_seq
                        && latest_route_state.energy_mechanism_active()
                    {
                        infer_post_queue.push_latest(output);
                    }
                    session = session_return;
                }
                Ok(Err(err)) => {
                    warn!("energy_mechanism_infer: failed to process frame {id}: {err}");
                    IS_RUNNING.store(false, Ordering::SeqCst);
                    break;
                }
                Err(err) => {
                    warn!("energy_mechanism_infer: worker join failed for frame {id}: {err}");
                    IS_RUNNING.store(false, Ordering::SeqCst);
                    break;
                }
            }
        }
        completion.mark_energy_infer_done();
    })
}

pub fn energy_mechanism_post_process(
    frame_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismFrame>>,
    solved_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismSolvedFrame>>,
    cfg: RbtCfg,
    runtime_router: RuntimeRouter,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut post_count = 0_u64;
        let mut post_total = Duration::ZERO;
        let mut decode_stats = EnergyMechanismYoloDecodeStats::default();
        loop {
            if completion.energy_infer_done() && frame_queue.is_empty() {
                info!(
                    "energy_mechanism_post_process: stopping after {post_count} frames, avg post {:?}, decode stats {:?}",
                    avg_duration(post_total, post_count),
                    decode_stats,
                );
                break;
            }
            let Some(frame) = pop_latest_until_running(
                &frame_queue,
                Duration::from_millis(PIPELINE_POP_TIMEOUT_MS),
            )
            .await
            else {
                continue;
            };
            let route_state = runtime_router.state();
            if !route_state.energy_mechanism_active() {
                continue;
            }
            let detector_cfg = cfg.detector_cfg.energy_mechanism.clone();
            let self_fraction = cfg.game_cfg.self_fraction();
            let cam_k = cfg.cam_cfg.cam_k();
            let id = frame.id();
            let result = tokio::task::spawn_blocking(move || {
                let started = StdInstant::now();
                let post_cfg = EnergyMechanismYoloPostprocessCfg::from_detector_cfg(
                    &detector_cfg,
                    self_fraction,
                );
                let (objects, stats) = decode_energy_mechanism_output_with_stats(
                    &frame.infer_data_ref(),
                    frame.letterbox_transform(),
                    &post_cfg,
                );
                let solved = solve_energy_mechanism(frame.mode(), objects, &cam_k)?;
                Ok::<_, lib::rbt_infra::rbt_err::RbtError>((solved, stats, started.elapsed()))
            })
            .await;

            match result {
                Ok(Ok((solved, stats, post_elapsed))) => {
                    post_count = post_count.wrapping_add(1);
                    post_total += post_elapsed;
                    decode_stats.anchors += stats.anchors;
                    decode_stats.confidence_pass += stats.confidence_pass;
                    decode_stats.class_pass += stats.class_pass;
                    decode_stats.target_pass += stats.target_pass;
                    decode_stats.self_color_pass += stats.self_color_pass;
                    decode_stats.geometry_pass += stats.geometry_pass;
                    decode_stats.nms_kept += stats.nms_kept;
                    let latest_route_state = runtime_router.state();
                    if latest_route_state.transition_seq == route_state.transition_seq
                        && latest_route_state.energy_mechanism_active()
                    {
                        solved_queue.push_latest(solved);
                    }
                }
                Ok(Err(err)) => {
                    warn!("energy_mechanism_post_process: failed to solve frame {id}: {err}")
                }
                Err(err) => {
                    warn!("energy_mechanism_post_process: worker join failed for frame {id}: {err}")
                }
            }
        }
        completion.mark_energy_post_done();
    })
}

pub fn energy_mechanism_estimate_process(
    solved_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismSolvedFrame>>,
    track_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismTrackPacket>>,
    rec: rr::RecordingStream,
    runtime_router: RuntimeRouter,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut ticker = tokio::time::interval(Duration::from_millis(2));
        let tracker_cfg = GENERIC_RBT_CFG
            .read()
            .unwrap()
            .energy_mechanism_cfg
            .tracker
            .clone();
        let mut tracker =
            EnergyMechanismTracker::from_tracker_cfg(EnergyMechanismMode::Small, &tracker_cfg);
        let mut snapshot_seq = 0_u64;
        let mut center_visible = false;
        let mut blade_center_visible = false;
        let mut horizon_visible = false;
        let mut last_transition_seq = runtime_router.state().transition_seq;
        loop {
            ticker.tick().await;
            if completion.energy_post_done() && solved_queue.is_empty() {
                info!("energy_mechanism_estimate_process: stopping");
                break;
            }

            let route_state = runtime_router.state();
            if route_state.transition_seq != last_transition_seq {
                let mode = EnergyMechanismMode::from_task_mode(route_state.task_mode)
                    .unwrap_or(EnergyMechanismMode::Small);
                tracker.reset(mode);
                last_transition_seq = route_state.transition_seq;
            }
            if !route_state.energy_mechanism_active() {
                continue;
            }

            let Some(mode) = EnergyMechanismMode::from_task_mode(route_state.task_mode) else {
                continue;
            };
            let solved = solved_queue
                .try_pop_latest()
                .unwrap_or(EnergyMechanismSolvedFrame {
                    mode,
                    target: None,
                    candidates: Vec::new(),
                });
            snapshot_seq = snapshot_seq.wrapping_add(1);
            let target = tracker.update(mode, solved.target.as_ref());
            if let Err(err) = log_rerun_energy_mechanism_snapshot(
                &rec,
                snapshot_seq,
                target,
                &mut center_visible,
                &mut blade_center_visible,
                &mut horizon_visible,
            ) {
                warn!("energy_mechanism_estimate_process: failed to log rerun snapshot: {err}");
            }
            track_queue.push_latest(EnergyMechanismTrackPacket {
                seq: snapshot_seq,
                target,
                publish_tp: Instant::now(),
            });
        }
        completion.mark_energy_estimate_done();
    })
}

/// 500Hz 估计器，将当前选中目标快照送入发控。
pub fn estimate_process(
    solved_queue: Arc<RbtSPSCQueueAsync<RbtSolvedResults>>,
    track_queue: Arc<RbtSPSCQueueAsync<PlannerTrackSnapshot>>,
    rec: rr::RecordingStream,
    runtime_router: RuntimeRouter,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut ticker = tokio::time::interval(Duration::from_millis(2));
        let mut estimator_poll = RbtHandlerPoll::new();
        let mut snapshot_seq = 0_u64;
        let mut raw_visible = false;
        let mut filtered_visible = false;
        let mut last_transition_seq = runtime_router.state().transition_seq;
        loop {
            ticker.tick().await;
            if completion.armor_post_done() && solved_queue.is_empty() {
                info!("estimate_process: Stopping processing as IS_RUNNING is false");
                break;
            }

            let route_state = runtime_router.state();
            if route_state.transition_seq != last_transition_seq {
                estimator_poll = RbtHandlerPoll::new();
                last_transition_seq = route_state.transition_seq;
            }
            if !route_state.armor_pipeline_active() {
                continue;
            }

            let enemys = solved_queue.try_pop_latest().unwrap_or_default();
            let raw_enemies = if rec.is_enabled() {
                Some(enemys.clone())
            } else {
                None
            };
            estimator_poll.update(&GENERIC_RBT_CFG.read().unwrap().estimator_cfg, enemys);
            snapshot_seq = snapshot_seq.wrapping_add(1);
            let target = estimator_poll.selected_snapshot();
            if let Some(raw_enemies) = raw_enemies
                && let Err(err) = log_rerun_filter_snapshot(
                    &rec,
                    snapshot_seq,
                    &raw_enemies,
                    target,
                    &mut raw_visible,
                    &mut filtered_visible,
                )
            {
                warn!("estimate_process: failed to log filter snapshot to rerun: {err}");
            }
            track_queue.push_latest(PlannerTrackSnapshot {
                seq: snapshot_seq,
                target,
                publish_tp: Instant::now(),
            });
        }
        completion.mark_armor_estimate_done();
    })
}

pub fn control_loop_250hz(
    track_queue: Arc<RbtSPSCQueueAsync<PlannerTrackSnapshot>>,
    energy_track_queue: Arc<RbtSPSCQueueAsync<EnergyMechanismTrackPacket>>,
    feedback_queue: Arc<RbtSPSCQueueAsync<SensData>>,
    control_tx_queue: Arc<RbtSPSCQueueAsync<CtrlData>>,
    runtime_router: RuntimeRouter,
    runtime_queues: RuntimePipelineQueues,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let cfg = GENERIC_RBT_CFG.read().unwrap().clone();
        let mut fire_control = match FireControlController::new() {
            Ok(controller) => controller,
            Err(err) => {
                error!("control_loop_250hz: failed to build fire-control controller: {err}");
                IS_RUNNING.store(false, Ordering::SeqCst);
                return;
            }
        };
        let mut energy_mechanism_control = EnergyMechanismController::from_aimer_cfg(
            &cfg.energy_mechanism_cfg.aimer,
            &cfg.energy_mechanism_cfg.mpc,
        );
        let mut latest_snapshot: Option<PlannerTrackSnapshot> = None;
        let mut latest_energy_snapshot: Option<EnergyMechanismTrackPacket> = None;
        let mut latest_feedback: Option<(SensData, Instant)> = None;
        let mut frame_seq = 0_u8;
        let mut tick_count = 0_u64;
        let dt_s = CONTROL_LOOP_PERIOD_MS * 1e-3;
        let mut ticker = tokio::time::interval(Duration::from_secs_f64(dt_s));
        let mut last_route_seq = runtime_router.state().transition_seq;

        loop {
            ticker.tick().await;
            let track_queues_empty = track_queue.is_empty() && energy_track_queue.is_empty();
            if completion.armor_estimate_done()
                && completion.energy_estimate_done()
                && track_queues_empty
            {
                info!("control_loop_250hz: Stopping processing as IS_RUNNING is false");
                break;
            }

            if let Some(snapshot) = track_queue.try_pop_latest() {
                latest_snapshot = Some(snapshot);
            }
            if let Some(snapshot) = energy_track_queue.try_pop_latest() {
                latest_energy_snapshot = Some(snapshot);
            }
            if let Some(feedback) = feedback_queue.try_pop_latest() {
                latest_feedback = Some((feedback, Instant::now()));
            }

            let feedback_for_route = latest_feedback.map(|(feedback, _)| feedback);
            if let Some(feedback) = feedback_for_route {
                let update = runtime_router.apply_feedback(&feedback);
                if update.transition_seq != last_route_seq {
                    runtime_queues.clear_for_route_transition();
                    fire_control.reset();
                    energy_mechanism_control.reset();
                    latest_snapshot = None;
                    latest_energy_snapshot = None;
                    latest_feedback = Some((feedback, Instant::now()));
                    last_route_seq = update.transition_seq;
                }
            }

            let route_state = runtime_router.state();
            let feedback_fresh = latest_feedback.as_ref().is_some_and(|(_, tp)| {
                tp.elapsed() <= Duration::from_millis(FEEDBACK_STALE_TIMEOUT_MS)
            });
            let feedback = if feedback_fresh {
                latest_feedback
                    .as_ref()
                    .map(|(feedback, _)| *feedback)
                    .unwrap_or_else(|| {
                        default_feedback(
                            cfg.general_cfg.bullet_speed,
                            cfg.general_cfg.offline_task_mode(),
                        )
                    })
            } else {
                default_feedback(
                    cfg.general_cfg.bullet_speed,
                    cfg.general_cfg.offline_task_mode(),
                )
            };

            if route_state.energy_mechanism_active() {
                let snapshot_age_ms = latest_energy_snapshot
                    .as_ref()
                    .map(|snapshot| snapshot.publish_tp.elapsed().as_secs_f64() * 1_000.0)
                    .unwrap_or(f64::INFINITY);
                let control_data = energy_mechanism_control.update(EnergyMechanismControlInput {
                    target: latest_energy_snapshot
                        .as_ref()
                        .and_then(|snapshot| snapshot.target),
                    feedback,
                    feedback_fresh,
                    dt_s,
                    snapshot_age_ms,
                });
                let stats = energy_mechanism_control.last_stats();
                let mut payload = [0_u8; CAN_FRAME_SIZE];
                if let Err(err) = control_data.serialize_with_seq(frame_seq, &mut payload) {
                    warn!("control_loop_250hz: failed to serialize energy mechanism frame: {err}");
                }
                control_tx_queue.push_latest(control_data);
                tick_count = tick_count.wrapping_add(1);
                frame_seq = frame_seq.wrapping_add(1);
                if tick_count == 1 || tick_count.is_multiple_of(CONTROL_STATUS_LOG_PERIOD_TICKS) {
                    let snapshot_seq = latest_energy_snapshot
                        .as_ref()
                        .map(|snapshot| snapshot.seq)
                        .unwrap_or(0);
                    info!(
                        "control_loop_250hz: route=energy_mechanism seq={} target={} valid={} fb={} yaw={:.2}->{:.2} pitch={:.2}->{:.2} shot={:?} can={:02X?}",
                        snapshot_seq,
                        stats.target_detected,
                        stats.track_valid,
                        feedback_fresh,
                        feedback.gimbal_yaw,
                        control_data.gimbal_yaw,
                        feedback.gimbal_pitch,
                        control_data.gimbal_pitch,
                        control_data.shot_mode,
                        payload,
                    );
                }
                continue;
            }

            if !route_state.fire_control_active() {
                let control_data = route_disabled_control(feedback);
                let mut payload = [0_u8; CAN_FRAME_SIZE];
                if let Err(err) = control_data.serialize_with_seq(frame_seq, &mut payload) {
                    warn!("control_loop_250hz: failed to serialize disabled-route frame: {err}");
                }
                control_tx_queue.push_latest(control_data);
                tick_count = tick_count.wrapping_add(1);
                frame_seq = frame_seq.wrapping_add(1);
                if tick_count == 1 || tick_count.is_multiple_of(CONTROL_STATUS_LOG_PERIOD_TICKS) {
                    info!(
                        "control_loop_250hz: route={:?} fire_control=off shot={:?} can={:02X?}",
                        route_state.route, control_data.shot_mode, payload
                    );
                }
                continue;
            }

            let snapshot_age_ms = latest_snapshot
                .as_ref()
                .map(|snapshot| snapshot.publish_tp.elapsed().as_secs_f64() * 1_000.0)
                .unwrap_or(f64::INFINITY);
            let stale = snapshot_age_ms > FIRE_CONTROL_SNAPSHOT_STALE_MS;
            let control_data = fire_control.update(FireControlInput {
                target: latest_snapshot
                    .as_ref()
                    .and_then(|snapshot| snapshot.target),
                feedback,
                feedback_fresh,
                dt_s,
                snapshot_age_ms,
            });
            let stats = fire_control.last_stats();
            let mut payload = [0_u8; CAN_FRAME_SIZE];
            if let Err(err) = control_data.serialize_with_seq(frame_seq, &mut payload) {
                warn!("control_loop_250hz: failed to serialize control frame: {err}");
            }
            control_tx_queue.push_latest(control_data);

            tick_count = tick_count.wrapping_add(1);
            frame_seq = frame_seq.wrapping_add(1);

            if tick_count == 1 || tick_count.is_multiple_of(CONTROL_STATUS_LOG_PERIOD_TICKS) {
                let snapshot_seq = latest_snapshot
                    .as_ref()
                    .map(|snapshot| snapshot.seq)
                    .unwrap_or(0);
                info!(
                    "control_loop_250hz: seq={} target={} stale={} fb={} yaw={:.2}->{:.2} pitch={:.2}->{:.2} err={:.2} tol={:.2} slot={} first={} next={} gate=V{}{}{}{}{}{}{}{}{} shot={:?} can={:02X?}",
                    snapshot_seq,
                    stats.target_detected,
                    stale,
                    feedback_fresh,
                    feedback.gimbal_yaw,
                    control_data.gimbal_yaw,
                    feedback.gimbal_pitch,
                    control_data.gimbal_pitch,
                    stats.yaw_error_deg,
                    stats.tolerance_deg,
                    stats.viable_slot_count,
                    format_status_value(stats.first_slot_error_deg),
                    format_status_value(stats.next_slot_delay_ms),
                    if stats.gate_mcu { "U" } else { "x" },
                    if stats.gate_command_stable { "C" } else { "x" },
                    if stats.gate_follow { "F" } else { "x" },
                    if stats.gate_preview { "P" } else { "x" },
                    if stats.gate_impact { "A" } else { "x" },
                    if stats.gate_slot { "S" } else { "x" },
                    if stats.gate_motion { "M" } else { "x" },
                    if stats.gate_observation { "O" } else { "x" },
                    if stats.static_bypass_active {
                        "D"
                    } else if stats.preview_mpc_active {
                        "2"
                    } else {
                        "x"
                    },
                    control_data.shot_mode,
                    payload,
                );
            }
        }
    })
}

pub fn can_io_process(
    control_tx_queue: Arc<RbtSPSCQueueAsync<CtrlData>>,
    feedback_queue: Arc<RbtSPSCQueueAsync<SensData>>,
    cfg: RbtCfg,
    completion: RuntimePipelineCompletion,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        if !cfg.general_cfg.can_enabled {
            info!("can_io_process: CAN disabled by config");
            return;
        }

        let interface = cfg.general_cfg.can_interface.clone();
        let device = match SocketCanDevice::open(&interface) {
            Ok(device) => device,
            Err(err) => {
                error!("can_io_process: failed to open SocketCAN interface {interface}: {err}");
                IS_RUNNING.store(false, Ordering::SeqCst);
                return;
            }
        };
        info!("can_io_process: opened SocketCAN interface {interface}");

        let mut feedback_decoder = FeedbackPairDecoder::new();
        let mut frame_seq = 0_u8;
        let mut rx_count = 0_u64;
        let mut tx_count = 0_u64;

        loop {
            if completion.armor_estimate_done()
                && completion.energy_estimate_done()
                && control_tx_queue.is_empty()
            {
                info!(
                    "can_io_process: stopping after tx={} rx={}",
                    tx_count, rx_count
                );
                break;
            }

            tokio::select! {
                frame = device.receive() => {
                    match frame {
                        Ok(frame) => {
                            match feedback_decoder.push(frame, StdInstant::now()) {
                                Ok(Some(feedback)) => {
                                    rx_count = rx_count.wrapping_add(1);
                                    feedback_queue.push_latest(feedback);
                                }
                                Ok(None) => {}
                                Err(err) => warn!("can_io_process: dropped corrupted feedback pair: {err}"),
                            }
                        }
                        Err(err) => {
                            error!("can_io_process: CAN receive failed: {err}");
                            IS_RUNNING.store(false, Ordering::SeqCst);
                            break;
                        }
                    }
                }
                control = pop_latest_until_running(
                    &control_tx_queue,
                    Duration::from_millis(CAN_IO_POP_TIMEOUT_MS),
                ) => {
                    if let Some(control) = control {
                        match control_to_can_payload(control, frame_seq) {
                            Ok(payload) => {
                                if let Err(err) = device.send(payload).await {
                                    error!("can_io_process: CAN send failed: {err}");
                                    IS_RUNNING.store(false, Ordering::SeqCst);
                                    break;
                                }
                                tx_count = tx_count.wrapping_add(1);
                                frame_seq = frame_seq.wrapping_add(1);
                            }
                            Err(err) => warn!("can_io_process: failed to serialize control frame: {err}"),
                        }
                    }
                }
            }
        }
    })
}

fn format_status_value(value: f64) -> String {
    if value.is_finite() {
        format!("{value:.2}")
    } else {
        "-".to_string()
    }
}

fn route_disabled_control(feedback: SensData) -> CtrlData {
    CtrlData {
        gimbal_yaw: feedback.gimbal_yaw,
        gimbal_pitch: feedback.gimbal_pitch,
        shot_mode: ShotMode::DoNothing,
        shot_buff_mode: ShotBuffMode::ShotBuffOff,
        aiming_state: AimingState::AimingNoTarget,
    }
}

async fn pop_latest_until_running<T>(queue: &RbtSPSCQueueAsync<T>, timeout: Duration) -> Option<T> {
    loop {
        if let Some(item) = queue.try_pop_latest() {
            return Some(item);
        }
        if let Ok(item) = tokio::time::timeout(timeout, queue.pop_latest()).await {
            return item;
        }
        if !IS_RUNNING.load(Ordering::SeqCst) {
            return None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_feedback_disables_mcu_fire_permit() {
        let feedback = default_feedback(24.0, TaskMode::AutoShot);

        assert_eq!(feedback.gimbal_yaw, 0.0);
        assert_eq!(feedback.gimbal_pitch, 0.0);
        assert!(!feedback.mcu_fire_permit);
        assert_eq!(feedback.task_mode, TaskMode::AutoShot);
    }

    #[test]
    fn default_feedback_can_override_task_mode_for_offline_simulation() {
        let feedback = default_feedback(24.0, TaskMode::HitBigBuff);

        assert_eq!(feedback.task_mode, TaskMode::HitBigBuff);
        assert_eq!(feedback.raw_task_mode, TaskMode::HitBigBuff as u8);
        assert_eq!(feedback.mapped_task_mode, TaskMode::HitBigBuff);
    }

    #[test]
    fn route_disabled_control_keeps_gimbal_and_stops_fire() {
        let mut feedback = default_feedback(24.0, TaskMode::AutoShot);
        feedback.gimbal_yaw = 12.5;
        feedback.gimbal_pitch = -3.0;

        let control = route_disabled_control(feedback);

        assert_eq!(control.gimbal_yaw, 12.5);
        assert_eq!(control.gimbal_pitch, -3.0);
        assert_eq!(control.shot_mode, ShotMode::DoNothing);
        assert_eq!(control.shot_buff_mode, ShotBuffMode::ShotBuffOff);
        assert_eq!(control.aiming_state, AimingState::AimingNoTarget);
    }

    #[test]
    fn video_reader_decodes_default_video_and_letterboxes_frame() {
        let video_path = video_input_path();
        if !video_path.is_file() {
            return;
        }

        let mut reader = match FfmpegVideoReader::open(&video_path) {
            Ok(reader) => reader,
            Err(err)
                if err.contains("failed to start ffmpeg")
                    || err.contains("failed to start ffprobe") =>
            {
                return;
            }
            Err(err) => panic!("{err}"),
        };
        let frame = reader
            .read_frame()
            .expect("ffmpeg should decode the first video frame")
            .expect("video should contain at least one frame");
        assert!(frame.width() > 0);
        assert!(frame.height() > 0);

        let mut input = nd::Array4::<half::f16>::zeros((1, 3, 640, 640));
        let transform = preprocess_letterbox_f16(input.view_mut(), &frame);

        assert_eq!(input.shape(), &[1, 3, 640, 640]);
        assert_eq!(transform.image_width, frame.width());
        assert_eq!(transform.image_height, frame.height());
        assert!(transform.scale > 0.0);

        let gray = frame.to_luma8();
        assert_eq!(gray.width(), frame.width());
        assert_eq!(gray.height(), frame.height());
    }
}
