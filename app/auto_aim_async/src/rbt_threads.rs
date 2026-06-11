use log::{error, info, warn};
use ort::inputs;
use ort::value::TensorRef;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;
use tokio::task::JoinHandle;
use tokio::time::Instant;

// use crate::rbt_cfg::{self, DetectorConfig, RbtCfg};
// use lib::rbt_mod::rbt_armor::ArmorKeyPoints;
use lib::rbt_mod::rbt_solver::RbtSolvedResults;
use lib::{
    rbt_base::rbt_geometry::rbt_point2::RbtImgPoint2,
    rbt_infra::rbt_cfg::RbtCfg,
    rbt_infra::{
        rbt_global::{GENERIC_RBT_CFG, IS_RUNNING},
        rbt_queue_async::RbtSPSCQueueAsync,
    },
    rbt_mod::{
        rbt_armor::detected_armor::DetectedArmor,
        rbt_comm::rbt_comm_frame::{
            AimingState, CAN_FRAME_SIZE, CONTROL_LOOP_PERIOD_MS, CtrlData,
            DEFAULT_BULLET_SPEED_MPS, FEEDBACK_STALE_TIMEOUT_MS, SelfFraction, SensData,
            ShotBuffMode, ShotMode, TaskMode,
        },
        rbt_detector::{
            BBox,
            rbt_frame::{RbtFrame, RbtFrameStage},
            rbt_yolo::{YOLO_LABEL_TABLE, letterbox, nms},
        },
        rbt_estimator::rbt_enemy_dynamic_model::EnemyId,
        rbt_estimator::{RbtHandlerPoll, RbtTargetSnapshot},
        rbt_fire_control::{FireControl, SecondOrderPositionMpc, SecondOrderPositionMpcConfig},
        rbt_solver::enemys_solver,
    },
};

const STATIC_IMAGE_FRAME_PERIOD_MS: u64 = 16;
const FIRE_CONTROL_SNAPSHOT_STALE_MS: f64 = 180.0;
const CONTROL_STATUS_LOG_PERIOD_TICKS: u64 = 50;
const PIPELINE_POP_TIMEOUT_MS: u64 = 100;

#[derive(Debug, Clone)]
pub struct FireControlSnapshot {
    seq: u64,
    target: Option<RbtTargetSnapshot>,
    publish_tp: Instant,
}

pub fn static_image_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("imgs")
        .join("test_resize.jpg")
}

fn fallback_feedback(bullet_speed_mps: f64) -> SensData {
    SensData {
        task_mode: TaskMode::AutoShot,
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
        raw_task_mode: TaskMode::AutoShot.into(),
        mapped_task_mode: TaskMode::AutoShot,
    }
}

fn hold_current_gimbal_control(feedback: SensData) -> CtrlData {
    CtrlData {
        gimbal_yaw: feedback.gimbal_yaw,
        gimbal_pitch: feedback.gimbal_pitch,
        shot_mode: ShotMode::DoNothing,
        shot_buff_mode: ShotBuffMode::ShotBuffOff,
        aiming_state: AimingState::AimingNoTarget,
    }
}

fn normalize_angle_deg(angle_deg: f64) -> f64 {
    let mut result = (angle_deg + 180.0) % 360.0;
    if result < 0.0 {
        result += 360.0;
    }
    result - 180.0
}

fn shortest_angle_delta_deg(target_deg: f64, current_deg: f64) -> f64 {
    normalize_angle_deg(target_deg - current_deg)
}

/// 图像预处理阶段：读取图像并通过通道发送到下一阶段。
/// 此函数负责读取图像、调整图像大小、转换为归一化格式，并为推理阶段准备数据。
pub fn pre_process(queue: Arc<RbtSPSCQueueAsync<RbtFrame>>) -> JoinHandle<()> {
    tokio::spawn(async move {
        let input_template = match tokio::task::spawn_blocking(|| {
            let image_path = static_image_path();
            let resized_img = image::open(&image_path)
                .map_err(|err| format!("failed to open {}: {err}", image_path.display()))?;
            let mut input_array = nd::Array4::zeros((1, 3, 384, 640));
            letterbox(&mut input_array, &resized_img);
            Ok::<_, String>(input_array)
        })
        .await
        {
            Ok(Ok(input_template)) => input_template,
            Ok(Err(err)) => {
                error!("pre_process: {err}");
                IS_RUNNING.store(false, Ordering::SeqCst);
                return;
            }
            Err(err) => {
                error!("pre_process: failed to prepare static image: {err}");
                IS_RUNNING.store(false, Ordering::SeqCst);
                return;
            }
        };

        info!(
            "pre_process: loaded static image {}",
            static_image_path().display()
        );

        let mut frame_id = 0_u64;
        let mut ticker = tokio::time::interval(Duration::from_millis(STATIC_IMAGE_FRAME_PERIOD_MS));
        loop {
            ticker.tick().await;
            if !IS_RUNNING.load(Ordering::SeqCst) {
                info!("pre_process: Stopping processing as IS_RUNNING is false");
                break;
            }

            frame_id = frame_id.wrapping_add(1);
            let mut rbt_frame = RbtFrame::new();
            rbt_frame.pre_data().assign(&input_template.view());
            rbt_frame.set_id(frame_id);
            rbt_frame.set_state(RbtFrameStage::Pre);
            queue.push_latest(rbt_frame);

            if frame_id == 1 || frame_id.is_multiple_of(60) {
                info!("pre_process: replayed static frame {}", frame_id);
            }
        }
    })
}

/// 推理阶段：接收预处理后的数据，执行模型推理，并将结果发送到后续处理阶段
pub fn infer(
    pre_infer_queue: Arc<RbtSPSCQueueAsync<RbtFrame>>, // 接收预处理阶段的输出
    mut session: ort::session::Session,                // ONNX Runtime 推理会话
    infer_post_queue: Arc<RbtSPSCQueueAsync<RbtFrame>>, // 发送推理结果到后续处理阶段
) -> JoinHandle<()> {
    tokio::spawn(async move {
        loop {
            if !IS_RUNNING.load(Ordering::SeqCst) {
                info!("infer: Stopping processing as IS_RUNNING is false");
                break;
            }
            if let Some(mut frame) = pop_latest_until_running(
                &pre_infer_queue,
                Duration::from_millis(PIPELINE_POP_TIMEOUT_MS),
            )
            .await
            {
                info!(
                    "infer: Frame ID {} received form processing, time used: {:?}",
                    frame.id(),
                    frame.time_used()
                );
                frame.set_state(RbtFrameStage::Infer);
                let id = frame.id(); // 获取帧 ID，用于日志记录
                // 在阻塞线程中执行推理操作
                let output_result = tokio::task::spawn_blocking(move || {
                    // 执行模型推理
                    let output_array = {
                        let outputs = session
                            .run(inputs![
                                TensorRef::from_array_view(frame.pre_data()).unwrap()
                            ])
                            .unwrap();
                        outputs["output0"]
                            .try_extract_array::<f32>()
                            .unwrap()
                            .t()
                            .into_owned()
                            .as_standard_layout()
                            .into_shape_with_order((5040, 48, 1)) // 重塑输出形状，基于先验的模型尺寸
                            .expect("Failed to reshape output")
                            .to_owned()
                    };
                    frame.infer_data().assign(&output_array);
                    (session, frame) // 返回会话和处理后的帧
                })
                .await;

                // 处理推理结果
                if let Ok((session_return, output)) = output_result {
                    infer_post_queue.push_latest(output); // 将最新推理结果发送到后处理阶段
                    session = session_return; // 确保会话在闭包外部可用
                } else {
                    warn!("infer: Failed to process frame ID: {}", id);
                    IS_RUNNING.store(false, Ordering::SeqCst);
                    break;
                }
            }
        }
    })
}

/// 后处理阶段：接收推理结果，执行目标检测框处理，并提取装甲板信息
pub fn post_process(
    frame: Arc<RbtSPSCQueueAsync<RbtFrame>>,
    solved_queue: Arc<RbtSPSCQueueAsync<RbtSolvedResults>>,
    cfg: RbtCfg,
    rec: rr::RecordingStream,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        loop {
            if !IS_RUNNING.load(Ordering::SeqCst) {
                info!("post_process: Stopping processing as IS_RUNNING is false");
                break;
            }
            let detector_cfg = cfg.detector_cfg.clone();
            let game_cfg = cfg.game_cfg.clone();
            let cam_k = cfg.cam_cfg.cam_k();
            let rec = rec.clone();
            if let Some(mut frame) =
                pop_latest_until_running(&frame, Duration::from_millis(PIPELINE_POP_TIMEOUT_MS))
                    .await
            {
                let time_used = frame.time_used(); // 获取处理时间
                info!(
                    "post_process: Frame ID {} received in {:?}",
                    frame.id(),
                    time_used
                );
                frame.set_state(RbtFrameStage::Post); // 更新状态为后处理
                let id = frame.id(); // 获取帧 ID，用于日志记录
                // 在阻塞线程中执行后处理操作
                let result = tokio::task::spawn_blocking(move || {
                    let binding = frame.infer_data();
                    let output = binding.slice(nd::s![.., .., 0]);

                    let mut boxes = Vec::new(); // 存储目标检测框

                    // 遍历每一行输出，提取目标检测框信息
                    for (idx, row) in output.axis_iter(nd::Axis(0)).enumerate() {
                        let row: Vec<_> = row.iter().copied().collect();
                        let (class_id, prob) = row[4..40]
                            .iter()
                            .enumerate()
                            .map(|(index, value)| (index, *value))
                            .reduce(|accum, row| if row.1 > accum.1 { row } else { accum })
                            .unwrap();

                        // 如果置信度低于阈值，跳过该检测框
                        if prob < detector_cfg.confidence_threshold {
                            continue;
                        }

                        let xc = row[0]; // 中心点 x 坐标
                        let yc = row[1]; // 中心点 y 坐标
                        let w = row[2]; // 检测框宽度
                        let h = row[3]; // 检测框高度

                        let half_w = w / 2.0;
                        let half_h = h / 2.0;

                        boxes.push((
                            BBox::new(xc - half_w, yc - half_h, xc + half_w, yc + half_h),
                            class_id,
                            prob,
                            idx,
                        ));
                    }

                    // 非极大值抑制：去除重叠的检测框，保留最优框
                    let result = nms(boxes);

                    let mut id = 0usize;
                    // 收集装甲板信息
                    let mut armors =
                        HashMap::<EnemyId, Vec<DetectedArmor>>::with_capacity(result.len());
                    for (_, class_id, _, idx) in result {
                        //     let armor = ArmorKeyPoints::new(
                        //         ImgCoord::from_f32(output[[idx, 0]], output[[idx, 1]]), // 中心点坐标
                        //         ImgCoord::from_f32(output[[idx, 40]], output[[idx, 41]]), // 特征点 1
                        //         ImgCoord::from_f32(output[[idx, 42]], output[[idx, 43]]), // 特征点 2
                        //         ImgCoord::from_f32(output[[idx, 44]], output[[idx, 45]]), // 特征点 3
                        //         ImgCoord::from_f32(output[[idx, 46]], output[[idx, 47]]), // 特征点 4
                        //     );
                        //     armors.push(armor); // 添加到装甲板列表
                        let armor_label = &YOLO_LABEL_TABLE[class_id];
                        if armor_label.color() == &game_cfg.self_fraction().unwrap() {
                            continue;
                        }
                        let armor_id = *armor_label.id();

                        let armor = DetectedArmor::new(
                            RbtImgPoint2::new_screen_pixel(output[[idx, 0]], output[[idx, 1]]),
                            RbtImgPoint2::new_screen_pixel(output[[idx, 40]], output[[idx, 41]]),
                            RbtImgPoint2::new_screen_pixel(output[[idx, 42]], output[[idx, 43]]),
                            RbtImgPoint2::new_screen_pixel(output[[idx, 44]], output[[idx, 45]]),
                            RbtImgPoint2::new_screen_pixel(output[[idx, 46]], output[[idx, 47]]),
                            id,
                        );

                        id += 1;
                        armors.entry(armor_id).or_default().push(armor);
                    }

                    let solved_enemies = enemys_solver(armors, &cam_k, &rec)?;
                    Ok::<_, lib::rbt_infra::rbt_err::RbtError>((frame, solved_enemies))
                })
                .await;

                if let Ok(Ok((_frame, solved_enemies))) = result {
                    solved_queue.push_latest(solved_enemies);
                    let time_used = _frame.time_used(); // 获取处理时间
                    info!(
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
    })
}

/// 500Hz 频率通讯
pub fn estimate_process(
    solved_queue: Arc<RbtSPSCQueueAsync<RbtSolvedResults>>,
    fire_control_queue: Arc<RbtSPSCQueueAsync<FireControlSnapshot>>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut ticker = tokio::time::interval(Duration::from_millis(2));
        let mut estimator_poll = RbtHandlerPoll::new();
        let mut snapshot_seq = 0_u64;
        loop {
            ticker.tick().await;
            if !IS_RUNNING.load(Ordering::SeqCst) && solved_queue.is_empty() {
                info!("estimate_process: Stopping processing as IS_RUNNING is false");
                break;
            }

            let enemys = solved_queue.try_pop_latest().unwrap_or_default();
            estimator_poll.update(&GENERIC_RBT_CFG.read().unwrap().estimator_cfg, enemys);
            snapshot_seq = snapshot_seq.wrapping_add(1);
            fire_control_queue.push_latest(FireControlSnapshot {
                seq: snapshot_seq,
                target: estimator_poll.selected_target_snapshot(),
                publish_tp: Instant::now(),
            });
        }
    })
}

pub fn control_loop_250hz(
    fire_control_queue: Arc<RbtSPSCQueueAsync<FireControlSnapshot>>,
    feedback_queue: Arc<RbtSPSCQueueAsync<SensData>>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        let cfg = GENERIC_RBT_CFG.read().unwrap().clone();
        let fire_control = FireControl::new(cfg.general_cfg.bullet_speed);
        let fire_gate = fire_control.fire_gate_config();
        let mut yaw_mpc = match SecondOrderPositionMpc::new(SecondOrderPositionMpcConfig::default())
        {
            Ok(mpc) => mpc,
            Err(err) => {
                error!("control_loop_250hz: failed to build yaw MPC: {err}");
                IS_RUNNING.store(false, Ordering::SeqCst);
                return;
            }
        };
        let mut latest_snapshot: Option<FireControlSnapshot> = None;
        let mut latest_feedback: Option<(SensData, Instant)> = None;
        let mut frame_seq = 0_u8;
        let mut tick_count = 0_u64;
        let dt_s = CONTROL_LOOP_PERIOD_MS * 1e-3;
        let mut ticker = tokio::time::interval(Duration::from_secs_f64(dt_s));

        loop {
            ticker.tick().await;
            if !IS_RUNNING.load(Ordering::SeqCst) && fire_control_queue.is_empty() {
                info!("control_loop_250hz: Stopping processing as IS_RUNNING is false");
                break;
            }

            if let Some(snapshot) = fire_control_queue.try_pop_latest() {
                latest_snapshot = Some(snapshot);
            }
            if let Some(feedback) = feedback_queue.try_pop_latest() {
                latest_feedback = Some((feedback, Instant::now()));
            }

            let feedback_fresh = latest_feedback.as_ref().is_some_and(|(_, tp)| {
                tp.elapsed() <= Duration::from_millis(FEEDBACK_STALE_TIMEOUT_MS)
            });
            let feedback = if feedback_fresh {
                latest_feedback
                    .as_ref()
                    .map(|(feedback, _)| *feedback)
                    .unwrap_or_else(|| fallback_feedback(cfg.general_cfg.bullet_speed))
            } else {
                fallback_feedback(cfg.general_cfg.bullet_speed)
            };

            let mut control_data = hold_current_gimbal_control(feedback);
            let mut target_seen = false;
            let mut stale = false;
            let mut yaw_error_deg = 0.0;
            let mut pitch_error_deg = 0.0;
            let mut tolerance_deg = fire_gate.yaw_tolerance_deg(0.0);

            if let Some(snapshot) = latest_snapshot.as_ref() {
                let snapshot_age_ms = snapshot.publish_tp.elapsed().as_secs_f64() * 1_000.0;
                stale = snapshot_age_ms > FIRE_CONTROL_SNAPSHOT_STALE_MS;

                if !stale && let Some(target) = snapshot.target {
                    target_seen = true;
                    match fire_control.aim_at_base_point(target.target_base_mm) {
                        Ok(aim) => {
                            let yaw_command = match yaw_mpc.update(
                                aim.yaw_deg,
                                feedback.gimbal_yaw as f64,
                                feedback.yaw_speed as f64,
                                dt_s,
                            ) {
                                Ok(output) => output.command_deg,
                                Err(err) => {
                                    warn!("control_loop_250hz: yaw MPC failed: {err}");
                                    aim.yaw_deg
                                }
                            };

                            tolerance_deg =
                                fire_gate.yaw_tolerance_deg(target.distance_mm / 1_000.0);
                            yaw_error_deg =
                                shortest_angle_delta_deg(aim.yaw_deg, feedback.gimbal_yaw as f64)
                                    .abs();
                            pitch_error_deg = (aim.pitch_deg - feedback.gimbal_pitch as f64).abs();
                            let command_stable = fire_gate.command_is_stable(
                                shortest_angle_delta_deg(yaw_command, feedback.gimbal_yaw as f64)
                                    .abs(),
                                (aim.pitch_deg - feedback.gimbal_pitch as f64).abs(),
                                tolerance_deg,
                            );
                            let follow_ready = fire_gate.follow_is_ready(
                                yaw_error_deg,
                                pitch_error_deg,
                                tolerance_deg,
                            );
                            let fire_ready = feedback_fresh
                                && feedback.mcu_fire_permit
                                && target.fire_permit
                                && command_stable
                                && follow_ready;

                            control_data = CtrlData {
                                gimbal_yaw: yaw_command as f32,
                                gimbal_pitch: aim.pitch_deg as f32,
                                shot_mode: if fire_ready {
                                    ShotMode::AutoFire
                                } else {
                                    ShotMode::AimOnly
                                },
                                shot_buff_mode: ShotBuffMode::ShotBuffOff,
                                aiming_state: AimingState::AimingWithTarget,
                            };
                        }
                        Err(err) => {
                            warn!("control_loop_250hz: failed to aim target: {err}");
                        }
                    }
                }
            }

            let mut payload = [0_u8; CAN_FRAME_SIZE];
            if let Err(err) = control_data.serialize_with_seq(frame_seq, &mut payload) {
                warn!("control_loop_250hz: failed to serialize control frame: {err}");
            }

            tick_count = tick_count.wrapping_add(1);
            frame_seq = frame_seq.wrapping_add(1);

            if tick_count == 1 || tick_count.is_multiple_of(CONTROL_STATUS_LOG_PERIOD_TICKS) {
                let snapshot_seq = latest_snapshot
                    .as_ref()
                    .map(|snapshot| snapshot.seq)
                    .unwrap_or(0);
                info!(
                    "control_loop_250hz: seq={} target={} stale={} fb={} yaw={:.2}->{:.2} pitch={:.2}->{:.2} err=({:.2},{:.2}) tol={:.2} shot={:?} can={:02X?}",
                    snapshot_seq,
                    target_seen,
                    stale,
                    feedback_fresh,
                    feedback.gimbal_yaw,
                    control_data.gimbal_yaw,
                    feedback.gimbal_pitch,
                    control_data.gimbal_pitch,
                    yaw_error_deg,
                    pitch_error_deg,
                    tolerance_deg,
                    control_data.shot_mode,
                    payload,
                );
            }
        }
    })
}

async fn pop_latest_until_running<T>(queue: &RbtSPSCQueueAsync<T>, timeout: Duration) -> Option<T> {
    loop {
        if let Some(item) = queue.try_pop_latest() {
            return Some(item);
        }
        if !IS_RUNNING.load(Ordering::SeqCst) {
            return None;
        }
        if let Ok(item) = tokio::time::timeout(timeout, queue.pop_latest()).await {
            return item;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fallback_feedback_disables_mcu_fire_permit() {
        let feedback = fallback_feedback(24.0);

        assert_eq!(feedback.gimbal_yaw, 0.0);
        assert_eq!(feedback.gimbal_pitch, 0.0);
        assert!(!feedback.mcu_fire_permit);
        assert_eq!(feedback.task_mode, TaskMode::AutoShot);
    }
}
