//! 状态估计器模块
//!
//! 该模块实现了基于 YPD 角度 tracker 的敌方单位状态估计功能。
//! 通过融合视觉测量数据和几何运动模型，对敌方单位的位置、速度等状态进行估计和预测。
//!
//! 主要组件：
//! - EstimatorStateMachine: 估计器状态机，管理估计器的不同工作状态
//! - RbtEstimator: 单个敌方单位的状态估计器实现
//! - RbtHandlerPoll: 所有敌方单位估计器的管理池
//!

use std::collections::HashMap;
use std::time::Instant;

use crate::rbt_infra::rbt_cfg::EstimatorCfg;
use crate::rbt_mod::rbt_solver::{RbtSolvedResult, RbtSolvedResults};

use rbt_enemy_dynamic_model::EnemyId;
use rbt_enemy_select::{EnemySelectHandler, TRACKED_ENEMY_IDS};
use rbt_estimator_state::EstimatorStateMachine;
use rbt_ypd_angle_tracker::{YpdAngleTracker, YpdObservation, YpdTrackerSnapshot};

/// 敌方单位基础模型
pub mod rbt_enemy_dynamic_model;
mod rbt_enemy_select;
pub mod rbt_ypd_angle_tracker;

/// Snapshot exported by the estimator for the fire-control planner.
///
/// Units are intentionally aligned with `vivsionn::Target`: meters and radians.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetMotionState {
    Static,
    Dynamic,
}

impl TargetMotionState {
    pub fn is_static(self) -> bool {
        self == TargetMotionState::Static
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnemyTrackSnapshot {
    pub enemy_id: EnemyId,
    pub armor_count: usize,
    pub center_xy_m: na::Point2<f64>,
    pub center_velocity_xy_mps: na::Vector2<f64>,
    pub armor_z_m: f64,
    pub armor_z_velocity_mps: f64,
    pub body_yaw_rad: f64,
    pub body_yaw_rate_rad_s: f64,
    pub primary_radius_m: f64,
    pub secondary_radius_delta_m: f64,
    pub height_delta_m: f64,
    pub state_age_s: f64,
    pub track_valid: bool,
    pub fire_permit: bool,
    pub motion_state: TargetMotionState,
    pub motion_uniform: bool,
    pub observation_stable: bool,
    pub motion_translation_burst_metric: f64,
    pub motion_translation_drift_metric: f64,
    pub motion_yaw_accel_metric: f64,
}

impl EnemyTrackSnapshot {
    pub fn planner_state(&self) -> [f64; 11] {
        [
            self.center_xy_m.x,
            self.center_velocity_xy_mps.x,
            self.center_xy_m.y,
            self.center_velocity_xy_mps.y,
            self.armor_z_m,
            self.armor_z_velocity_mps,
            self.body_yaw_rad,
            self.body_yaw_rate_rad_s,
            self.primary_radius_m,
            self.secondary_radius_delta_m,
            self.height_delta_m,
        ]
    }
}

pub mod rbt_estimator_state {
    use crate::rbt_infra::rbt_cfg::EstimatorCfg;

    use crate::rbt_mod::rbt_solver::RbtSolvedResult;

    /// 顶层状态机
    #[derive(Debug, Clone, PartialEq, strum::Display)]
    pub enum EstimatorStateMachine {
        Init, // 初始化
        Sleep,
        WakeUp, // 从睡眠中恢复
        Track {
            jump: bool,
        }, // 跟踪状态
        Switching, // 云台移动中
        Lost {
            // 目标丢失（未识别，装甲板灭）
            time_stamp: tokio::time::Instant, // 丢失时间戳
        },
        Recovery, // 从丢失状态中恢复
    }

    impl EstimatorStateMachine {
        pub fn update(&mut self, solved_enemy: &Option<RbtSolvedResult>, cfg: &EstimatorCfg) {
            use EstimatorStateMachine::*;
            match self {
                Init => {
                    // 只会在初始化用到，然后在第一次 update 流转至其他状态
                    *self = match solved_enemy {
                        Some(_) => WakeUp,
                        None => Sleep,
                    }
                }
                Sleep => {
                    // 看到装甲板则唤醒估计器
                    if solved_enemy.is_some() {
                        *self = WakeUp;
                    }
                    // 没看到就继续休眠
                }
                WakeUp => {
                    // 看到装甲板则进入追踪
                    *self = match solved_enemy {
                        Some(_) => Track { jump: false },
                        None => Lost {
                            time_stamp: tokio::time::Instant::now(),
                        },
                    }
                }
                Track { jump } => {
                    if *jump {
                        *self = Switching;
                    }
                    // 如果solved_enemy 是 None 进入Lost状态，并记录当前时间戳
                    if solved_enemy.is_none() {
                        *self = Lost {
                            time_stamp: tokio::time::Instant::now(),
                        };
                    }
                }
                Switching => {
                    // 检查是否到位，如果到位则回到Track
                    // TODO: 实现云台到位检查
                    *self = Track { jump: false };
                }
                Lost { time_stamp } => {
                    *self = match (
                        solved_enemy.is_some(),                             // 是否检测到装甲板
                        time_stamp.elapsed() > cfg.lost_wait_duration_ms(), // 是否超时
                    ) {
                        (true, _) => Recovery,  // 如果检测到装甲板，进入Recovery状态
                        (false, true) => Sleep, // 如果没检测到装甲板且超时，进入Sleep状态
                        (false, false) => Lost {
                            time_stamp: *time_stamp, /* copy */
                        }, // 如果没检测到装甲板且未超时，保持Lost状态
                    };
                }
                Recovery => {
                    *self = match solved_enemy {
                        Some(_) => Track { jump: false },
                        None => Lost {
                            time_stamp: tokio::time::Instant::now(),
                        },
                    }
                }
            }
        }
    }
}

/// 状态估计器
#[derive(Debug, Clone)]
pub struct RbtEstimator {
    state: EstimatorStateMachine,
    ypd_angle_tracker: YpdAngleTracker,
    latest_tracker_snapshot: Option<YpdTrackerSnapshot>,
    last_update_tp: Option<Instant>,
    fire_observation_hold_frames: usize,
    previous_multi_armor_observation: bool,
    pub enemy_id: EnemyId,
    pub fire: bool,             // 当前是否开火
    pub single_or_double: bool, // 当前帧是否有多装甲板观测
}

impl RbtEstimator {
    pub fn new(enemy_id: EnemyId) -> Self {
        Self {
            state: EstimatorStateMachine::Init,
            ypd_angle_tracker: YpdAngleTracker::new(),
            latest_tracker_snapshot: None,
            last_update_tp: None,
            fire_observation_hold_frames: 0,
            previous_multi_armor_observation: false,
            enemy_id,
            fire: false,
            single_or_double: false,
        }
    }

    pub fn update(&mut self, cfg: &EstimatorCfg, solved_enemy: &Option<RbtSolvedResult>) {
        let dt_s = self.update_dt_s();

        self.state.update(solved_enemy, cfg);

        if matches!(
            self.state,
            EstimatorStateMachine::Init | EstimatorStateMachine::Sleep
        ) {
            self.ypd_angle_tracker.reset();
            self.latest_tracker_snapshot = None;
            self.reset_fire_observation_hold();
        }

        self.update_global_vars(solved_enemy);
        let tracker_was_initialized = self.ypd_angle_tracker.is_initialized();
        self.update_tracker(cfg, solved_enemy.as_ref(), dt_s);
        self.update_fire_observation_hold(cfg, solved_enemy.is_some(), tracker_was_initialized);
    }

    pub fn snapshot(&self) -> Option<EnemyTrackSnapshot> {
        if matches!(
            self.state,
            EstimatorStateMachine::Init | EstimatorStateMachine::Sleep
        ) {
            return None;
        }
        let tracker_snapshot = self.tracker_snapshot()?;
        let state = tracker_snapshot.state11d;
        let state_age_s = match self.state {
            EstimatorStateMachine::Lost { time_stamp } => time_stamp.elapsed().as_secs_f64(),
            _ => 0.0,
        };

        Some(EnemyTrackSnapshot {
            enemy_id: self.enemy_id,
            armor_count: tracker_snapshot.armor_num,
            center_xy_m: na::Point2::new(state[0] * 0.001, state[2] * 0.001),
            center_velocity_xy_mps: na::Vector2::new(state[1] * 0.001, state[3] * 0.001),
            armor_z_m: state[4] * 0.001,
            armor_z_velocity_mps: state[5] * 0.001,
            body_yaw_rad: state[6],
            body_yaw_rate_rad_s: state[7],
            primary_radius_m: state[8] * 0.001,
            secondary_radius_delta_m: state[9] * 0.001,
            height_delta_m: state[10] * 0.001,
            state_age_s,
            track_valid: !tracker_snapshot.diverged,
            fire_permit: self.fire,
            motion_state: target_motion_state(&state),
            motion_uniform: motion_uniform(tracker_snapshot),
            observation_stable: observation_stable(tracker_snapshot)
                && self.fire_observation_hold_frames == 0,
            motion_translation_burst_metric: metric_mm_to_m(
                tracker_snapshot.motion_translation_burst_metric,
            ),
            motion_translation_drift_metric: metric_mm_to_m(
                tracker_snapshot.motion_translation_drift_metric,
            ),
            motion_yaw_accel_metric: tracker_snapshot.motion_yaw_accel_metric,
        })
    }

    fn update_global_vars(&mut self, solved_enemy: &Option<RbtSolvedResult>) {
        use EstimatorStateMachine::*;
        // 设置fire
        self.fire = matches!(self.state, Track { .. });

        // 设置single_or_double
        self.single_or_double = solved_enemy
            .as_ref()
            .map(|s| s.armors.len() > 1)
            .unwrap_or(false);
    }

    fn update_fire_observation_hold(
        &mut self,
        cfg: &EstimatorCfg,
        has_solution: bool,
        tracker_was_initialized: bool,
    ) {
        if !has_solution {
            self.reset_fire_observation_hold();
            return;
        }

        let entering_multi_armor_observation =
            self.single_or_double && !self.previous_multi_armor_observation;
        if cfg.fire_block_on_armor_jump
            && tracker_was_initialized
            && entering_multi_armor_observation
        {
            self.fire_observation_hold_frames = self
                .fire_observation_hold_frames
                .max(cfg.fire_armor_jump_block_frames);
        } else if self.fire_observation_hold_frames > 0 {
            self.fire_observation_hold_frames -= 1;
        }
        self.previous_multi_armor_observation = self.single_or_double;
    }

    fn reset_fire_observation_hold(&mut self) {
        self.fire_observation_hold_frames = 0;
        self.previous_multi_armor_observation = false;
    }

    pub fn tracker_snapshot(&self) -> Option<&YpdTrackerSnapshot> {
        self.latest_tracker_snapshot.as_ref()
    }

    fn update_dt_s(&mut self) -> f64 {
        let now = Instant::now();
        let dt_s = self
            .last_update_tp
            .map(|last| now.duration_since(last).as_secs_f64())
            .unwrap_or(0.01);
        self.last_update_tp = Some(now);
        dt_s.clamp(0.001, 0.05)
    }

    fn update_tracker(
        &mut self,
        cfg: &EstimatorCfg,
        solved_enemy: Option<&RbtSolvedResult>,
        dt_s: f64,
    ) {
        use EstimatorStateMachine::*;

        match &self.state {
            Init | Sleep => {}
            WakeUp | Recovery | Track { .. } => {
                self.predict_or_reset_tracker(dt_s);
                if let Some(solved) = solved_enemy {
                    self.correct_tracker_with_solution(cfg, solved);
                }
                self.sync_tracker_snapshot();
            }
            Lost { .. } | Switching => {
                self.predict_or_reset_tracker(dt_s);
                self.sync_tracker_snapshot();
            }
        }
    }

    fn predict_or_reset_tracker(&mut self, dt_s: f64) {
        if self.ypd_angle_tracker.diverged() || self.ypd_angle_tracker.bad_convergence() {
            self.ypd_angle_tracker.reset();
            self.latest_tracker_snapshot = None;
            return;
        }
        self.ypd_angle_tracker.predict(dt_s);
    }

    fn correct_tracker_with_solution(&mut self, cfg: &EstimatorCfg, solved: &RbtSolvedResult) {
        let observations = self.ypd_observations(solved);
        let Some(preferred_index) = preferred_observation_index(&observations) else {
            return;
        };
        let armor_num = armor_num_for_enemy(self.enemy_id);

        if !self.ypd_angle_tracker.is_initialized() {
            self.ypd_angle_tracker
                .init(&observations[preferred_index], armor_num);
        } else {
            self.ypd_angle_tracker
                .note_observation_jump(self.single_or_double, cfg);
            self.ypd_angle_tracker
                .update_batch(&observations, Some(preferred_index), cfg);
        }
    }

    fn sync_tracker_snapshot(&mut self) {
        self.latest_tracker_snapshot = self.ypd_angle_tracker.snapshot();
    }

    fn ypd_observations(&self, solved: &RbtSolvedResult) -> Vec<YpdObservation> {
        let center = solved.coord.to_xy();
        let armor_num = armor_num_for_enemy(self.enemy_id);
        let sign = tracker_radial_sign(armor_num);

        solved
            .armors
            .iter()
            .map(|armor| {
                let position_vec = armor.pose().translation.vector;
                let position = na::Point3::new(position_vec.x, position_vec.y, position_vec.z);
                let dx = position.x - center.x;
                let dy = position.y - center.y;
                let radius_from_center = dx.hypot(dy);
                let radius_hint = if armor.radius().is_finite() && armor.radius() > 1e-6 {
                    armor.radius()
                } else {
                    radius_from_center
                };
                let yaw_rad = if armor_num == 3 {
                    armor.observed_yaw_rad()
                } else if radius_from_center > 1e-6 {
                    (dy / sign).atan2(dx / sign)
                } else {
                    armor.observed_yaw_rad()
                };
                let image_center = armor.center();

                YpdObservation {
                    position_mm: position,
                    yaw_rad,
                    image_center: na::Point2::new(image_center.x, image_center.y),
                    radius_hint_mm: radius_hint,
                }
            })
            .collect()
    }
}

fn armor_num_for_enemy(enemy_id: EnemyId) -> usize {
    if enemy_id == EnemyId::Outpost8 { 3 } else { 4 }
}

fn tracker_radial_sign(armor_num: usize) -> f64 {
    if armor_num == 3 { 1.0 } else { -1.0 }
}

fn preferred_observation_index(observations: &[YpdObservation]) -> Option<usize> {
    observations
        .iter()
        .enumerate()
        .min_by(|(_, lhs), (_, rhs)| image_center_score(lhs).total_cmp(&image_center_score(rhs)))
        .map(|(index, _)| index)
}

fn image_center_score(observation: &YpdObservation) -> f64 {
    let dx = observation.image_center.x - 320.0;
    let dy = observation.image_center.y - 192.0;
    dx * dx + dy * dy
}

fn target_motion_state(state: &[f64; 11]) -> TargetMotionState {
    let translation_speed_mps = (state[1] * 0.001).hypot(state[3] * 0.001);
    let z_speed_mps = (state[5] * 0.001).abs();
    let yaw_rate_rad_s = state[7].abs();

    if translation_speed_mps < 0.25 && z_speed_mps < 0.20 && yaw_rate_rad_s < 0.35 {
        TargetMotionState::Static
    } else {
        TargetMotionState::Dynamic
    }
}

fn observation_stable(snapshot: &YpdTrackerSnapshot) -> bool {
    !snapshot.diverged && (snapshot.converged || snapshot.recent_nis_failures <= 1)
}

fn motion_uniform(snapshot: &YpdTrackerSnapshot) -> bool {
    metric_under(snapshot.motion_translation_burst_metric, 8_000.0)
        && metric_under(snapshot.motion_translation_drift_metric, 4_000.0)
        && metric_under(snapshot.motion_yaw_accel_metric, 20.0)
}

fn metric_under(value: f64, threshold: f64) -> bool {
    !value.is_finite() || value.abs() < threshold
}

fn metric_mm_to_m(value: f64) -> f64 {
    if value.is_finite() {
        value * 0.001
    } else {
        f64::NAN
    }
}

/// 管理所有敌方单位的估计器。
#[derive(Debug, Clone)]
pub struct RbtHandlerPoll {
    estimators: HashMap<EnemyId, RbtEstimator>,
    enemy_selector: EnemySelectHandler,
}

impl RbtHandlerPoll {
    pub fn new() -> Self {
        let mut estimators = HashMap::with_capacity(6);
        for enemy_id in TRACKED_ENEMY_IDS {
            estimators.insert(enemy_id, RbtEstimator::new(enemy_id));
        }

        Self {
            estimators,
            enemy_selector: EnemySelectHandler::default(),
        }
    }

    pub fn update(&mut self, cfg: &EstimatorCfg, solved_enemies: RbtSolvedResults) {
        let selected_enemy_id = self.enemy_selector.select(cfg, &solved_enemies);
        let no_solution = None;

        for enemy_id in TRACKED_ENEMY_IDS {
            let solved_enemy = if selected_enemy_id == Some(enemy_id) {
                solved_enemies.get(&enemy_id).unwrap_or(&no_solution)
            } else {
                &no_solution
            };

            self.estimators
                .entry(enemy_id)
                .or_insert_with(|| RbtEstimator::new(enemy_id))
                .update(cfg, solved_enemy);
        }
    }

    pub fn selected_enemy_id(&self) -> Option<EnemyId> {
        self.enemy_selector.selected_enemy_id()
    }

    pub fn selected_snapshot(&self) -> Option<EnemyTrackSnapshot> {
        let enemy_id = self.selected_enemy_id()?;
        self.estimators.get(&enemy_id)?.snapshot()
    }
}

impl Default for RbtHandlerPoll {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rbt_base::rbt_geometry::rbt_cylindrical2::RbtCylindricalPoint2;
    use crate::rbt_base::rbt_geometry::rbt_point2::RbtImgPoint2;
    use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;
    use crate::rbt_mod::rbt_armor::solved_armor::SolvedArmor;
    use na::Isometry3;

    fn estimator_cfg(enemy_lost_wait_duration_ms: u64) -> EstimatorCfg {
        toml::from_str(&format!(
            "\
armor_lost_wait_duration_ms = 100
enemy_lost_wait_duration_ms = {enemy_lost_wait_duration_ms}
fire_block_on_armor_jump = true
fire_armor_jump_block_frames = 3
"
        ))
        .unwrap()
    }

    fn solved_enemy(center_x: f32, center_y: f32) -> RbtSolvedResult {
        let detected_armor = DetectedArmor::new(
            RbtImgPoint2::new_screen_pixel(center_x, center_y),
            RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y - 5.0),
            RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y + 5.0),
            RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y + 5.0),
            RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y - 5.0),
            0,
        );

        RbtSolvedResult {
            coord: RbtCylindricalPoint2::new(1_000.0, 0.0),
            armors: vec![SolvedArmor::new(
                detected_armor,
                Isometry3::identity(),
                0.0,
                0.0,
                200.0,
            )],
        }
    }

    fn solved_enemy_with_armors(centers: &[(f32, f32)]) -> RbtSolvedResult {
        let mut armors = Vec::with_capacity(centers.len());
        for (idx, (center_x, center_y)) in centers.iter().copied().enumerate() {
            let detected_armor = DetectedArmor::new(
                RbtImgPoint2::new_screen_pixel(center_x, center_y),
                RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y - 5.0),
                RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y + 5.0),
                RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y + 5.0),
                RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y - 5.0),
                idx,
            );
            let pose = Isometry3::translation(200.0 + idx as f64 * 20.0, idx as f64 * 200.0, 100.0);
            armors.push(SolvedArmor::new(
                detected_armor,
                pose,
                idx as f64 * 90.0,
                0.0,
                200.0,
            ));
        }

        RbtSolvedResult {
            coord: RbtCylindricalPoint2::new(1_000.0, 0.0),
            armors,
        }
    }

    fn frame(targets: &[(EnemyId, (f32, f32))]) -> RbtSolvedResults {
        let mut solved_enemies = RbtSolvedResults::default();
        for (enemy_id, (x, y)) in targets {
            solved_enemies.insert(*enemy_id, Some(solved_enemy(*x, *y)));
        }
        solved_enemies
    }

    fn stable_tracker_snapshot() -> YpdTrackerSnapshot {
        let mut state11d = [0.0; 11];
        state11d[8] = 200.0;

        YpdTrackerSnapshot {
            state11d,
            state9: [0.0; 9],
            tracked_id: 0,
            armor_num: 4,
            tracked_armor_xyza: [200.0, 0.0, 0.0, 0.0],
            predicted_armors_xyza: Vec::new(),
            last_nis: 0.0,
            converged: true,
            diverged: false,
            recent_nis_failures: 0,
            motion_translation_burst_metric: 0.0,
            motion_translation_drift_metric: 0.0,
            motion_yaw_accel_metric: 0.0,
        }
    }

    fn estimator_with_stable_snapshot() -> RbtEstimator {
        let mut estimator = RbtEstimator::new(EnemyId::Hero1);
        estimator.state = EstimatorStateMachine::Track { jump: false };
        estimator.fire = true;
        estimator.latest_tracker_snapshot = Some(stable_tracker_snapshot());
        estimator
    }

    #[test]
    fn handler_poll_feeds_only_the_selected_estimator() {
        let cfg = estimator_cfg(1_000);
        let mut handler_poll = RbtHandlerPoll::new();

        handler_poll.update(
            &cfg,
            frame(&[
                (EnemyId::Hero1, (320.0, 192.0)),
                (EnemyId::Infantry3, (321.0, 192.0)),
            ]),
        );

        assert_eq!(handler_poll.selected_enemy_id(), Some(EnemyId::Hero1));
        assert!(
            handler_poll.estimators[&EnemyId::Hero1]
                .tracker_snapshot()
                .is_some()
        );
        assert!(
            handler_poll.estimators[&EnemyId::Infantry3]
                .tracker_snapshot()
                .is_none()
        );
    }

    #[test]
    fn handler_poll_does_not_switch_while_selected_enemy_is_visible() {
        let cfg = estimator_cfg(1_000);
        let mut handler_poll = RbtHandlerPoll::new();

        handler_poll.update(
            &cfg,
            frame(&[
                (EnemyId::Hero1, (320.0, 192.0)),
                (EnemyId::Infantry3, (321.0, 192.0)),
            ]),
        );
        handler_poll.update(
            &cfg,
            frame(&[
                (EnemyId::Hero1, (600.0, 192.0)),
                (EnemyId::Infantry3, (320.0, 192.0)),
            ]),
        );

        assert_eq!(handler_poll.selected_enemy_id(), Some(EnemyId::Hero1));
        assert!(
            handler_poll.estimators[&EnemyId::Hero1]
                .tracker_snapshot()
                .is_some()
        );
        assert!(
            handler_poll.estimators[&EnemyId::Infantry3]
                .tracker_snapshot()
                .is_none()
        );
    }

    #[test]
    fn selected_snapshot_exports_selected_enemy_in_planner_units() {
        let cfg = estimator_cfg(1_000);
        let mut handler_poll = RbtHandlerPoll::new();

        handler_poll.update(&cfg, frame(&[(EnemyId::Hero1, (320.0, 192.0))]));
        handler_poll.update(&cfg, frame(&[(EnemyId::Hero1, (320.0, 192.0))]));

        let snapshot = handler_poll.selected_snapshot().unwrap();
        assert_eq!(snapshot.enemy_id, EnemyId::Hero1);
        assert_eq!(snapshot.armor_count, 4);
        assert!(snapshot.track_valid);
        assert!(snapshot.fire_permit);
        assert_eq!(snapshot.motion_state, TargetMotionState::Static);
        assert!(snapshot.motion_uniform);
        assert!(snapshot.observation_stable);
        assert!((snapshot.center_xy_m.x - 0.2).abs() < 1e-9);
        assert!(snapshot.center_xy_m.y.abs() < 1e-9);
        assert!(snapshot.body_yaw_rad.abs() < 1e-9);
        assert!((snapshot.primary_radius_m - 0.2).abs() < 1e-9);
        assert_eq!(snapshot.planner_state()[0], snapshot.center_xy_m.x);
    }

    #[test]
    fn selected_snapshot_is_none_without_target() {
        let cfg = estimator_cfg(1_000);
        let mut handler_poll = RbtHandlerPoll::new();

        handler_poll.update(&cfg, RbtSolvedResults::default());

        assert!(handler_poll.selected_snapshot().is_none());
    }

    #[test]
    fn fire_hold_blocks_stable_tracker_observation() {
        let mut estimator = estimator_with_stable_snapshot();

        assert!(estimator.snapshot().unwrap().observation_stable);

        estimator.fire_observation_hold_frames = 1;

        assert!(!estimator.snapshot().unwrap().observation_stable);
    }

    #[test]
    fn armor_jump_starts_and_releases_fire_hold_for_configured_frames() {
        let cfg = estimator_cfg(1_000);
        let mut estimator = RbtEstimator::new(EnemyId::Hero1);
        let single = Some(solved_enemy(320.0, 192.0));
        let double = Some(solved_enemy_with_armors(&[(320.0, 192.0), (350.0, 192.0)]));

        estimator.update(&cfg, &single);
        estimator.update(&cfg, &single);
        assert_eq!(estimator.fire_observation_hold_frames, 0);

        estimator.update(&cfg, &double);
        assert_eq!(estimator.fire_observation_hold_frames, 3);

        estimator.update(&cfg, &single);
        assert_eq!(estimator.fire_observation_hold_frames, 2);
        estimator.update(&cfg, &single);
        assert_eq!(estimator.fire_observation_hold_frames, 1);
        estimator.update(&cfg, &single);
        assert_eq!(estimator.fire_observation_hold_frames, 0);
    }

    #[test]
    fn armor_jump_fire_block_can_be_disabled() {
        let mut cfg = estimator_cfg(1_000);
        cfg.fire_block_on_armor_jump = false;
        let mut estimator = RbtEstimator::new(EnemyId::Hero1);
        let single = Some(solved_enemy(320.0, 192.0));
        let double = Some(solved_enemy_with_armors(&[(320.0, 192.0), (350.0, 192.0)]));

        estimator.update(&cfg, &single);
        estimator.update(&cfg, &single);
        estimator.update(&cfg, &double);

        assert_eq!(estimator.fire_observation_hold_frames, 0);
    }

    #[test]
    fn no_target_resets_armor_jump_fire_hold() {
        let cfg = estimator_cfg(1_000);
        let mut estimator = RbtEstimator::new(EnemyId::Hero1);
        let single = Some(solved_enemy(320.0, 192.0));
        let double = Some(solved_enemy_with_armors(&[(320.0, 192.0), (350.0, 192.0)]));

        estimator.update(&cfg, &single);
        estimator.update(&cfg, &single);
        estimator.update(&cfg, &double);
        assert!(estimator.fire_observation_hold_frames > 0);

        estimator.update(&cfg, &None);

        assert_eq!(estimator.fire_observation_hold_frames, 0);
        assert!(!estimator.previous_multi_armor_observation);
    }
}
