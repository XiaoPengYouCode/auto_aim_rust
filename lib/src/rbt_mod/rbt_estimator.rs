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
    pub enemy_id: EnemyId,
    pub fire: bool,             // 当前是否开火
    pub single_or_double: bool, // 当前帧是否有多装甲板观测
}

/// Latest target point for the fire-control loop.
///
/// The point is expressed in base coordinates, in millimeters, and is intended
/// to be consumed as a latest-value snapshot. It deliberately contains no
/// control-loop state; the gimbal feedback loop owns convergence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RbtTargetSnapshot {
    pub enemy_id: EnemyId,
    pub target_base_mm: na::Point3<f64>,
    pub distance_mm: f64,
    pub yaw_deg: f64,
    pub fire_permit: bool,
}

impl RbtEstimator {
    pub fn new(enemy_id: EnemyId) -> Self {
        Self {
            state: EstimatorStateMachine::Init,
            ypd_angle_tracker: YpdAngleTracker::new(),
            latest_tracker_snapshot: None,
            last_update_tp: None,
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
        }

        self.update_global_vars(solved_enemy);
        self.update_tracker(solved_enemy.as_ref(), dt_s);
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

    fn update_tracker(&mut self, solved_enemy: Option<&RbtSolvedResult>, dt_s: f64) {
        use EstimatorStateMachine::*;

        match &self.state {
            Init | Sleep => {}
            WakeUp | Recovery | Track { .. } => {
                self.predict_or_reset_tracker(dt_s);
                if let Some(solved) = solved_enemy {
                    self.correct_tracker_with_solution(solved);
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

    fn correct_tracker_with_solution(&mut self, solved: &RbtSolvedResult) {
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
                .update_batch(&observations, Some(preferred_index));
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
                let yaw_rad = if radius_from_center > 1e-6 {
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

    pub fn selected_target_snapshot(&self) -> Option<RbtTargetSnapshot> {
        let enemy_id = self.selected_enemy_id()?;
        let estimator = self.estimators.get(&enemy_id)?;
        let snapshot = estimator.tracker_snapshot()?;
        let tracked_armor = snapshot.tracked_armor_xyza;
        let target_base_mm = na::Point3::new(tracked_armor[0], tracked_armor[1], tracked_armor[2]);

        if !target_base_mm.coords.iter().all(|value| value.is_finite()) {
            return None;
        }

        let distance_mm = target_base_mm.x.hypot(target_base_mm.y);
        if distance_mm <= 1e-6 {
            return None;
        }

        Some(RbtTargetSnapshot {
            enemy_id,
            target_base_mm,
            distance_mm,
            yaw_deg: target_base_mm.y.atan2(target_base_mm.x).to_degrees(),
            fire_permit: estimator.fire,
        })
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

    fn frame(targets: &[(EnemyId, (f32, f32))]) -> RbtSolvedResults {
        let mut solved_enemies = RbtSolvedResults::default();
        for (enemy_id, (x, y)) in targets {
            solved_enemies.insert(*enemy_id, Some(solved_enemy(*x, *y)));
        }
        solved_enemies
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
}
