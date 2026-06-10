//! 发控前置计算。
//!
//! 该模块只负责把 base 坐标系下的目标点转换为云台 yaw/pitch 指令。
//! 弹道 pitch 使用 `rbt_antigravity` 中与参考工程一致的低弧线抛物线解。

use crate::rbt_base::rbt_algorithm::rbt_antigravity::solve_ballistic_trajectory;
use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use crate::rbt_mod::rbt_comm::rbt_comm_frame::{AimingState, CtrlData, ShotBuffMode, ShotMode};

pub mod fire_gate;
pub mod second_order_position_mpc;
pub use fire_gate::{FireGateConfig, ImpactAngleCheck, ShotSlotGate, ShotSlotGateResult};
pub use second_order_position_mpc::{
    SECOND_ORDER_POSITION_MPC_HORIZON, SecondOrderPositionMpc, SecondOrderPositionMpcConfig,
    SecondOrderPositionMpcOutput,
};

const MM_PER_M: f64 = 1_000.0;
const EPSILON_MM: f64 = 1e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AimCommand {
    pub yaw_deg: f64,
    pub pitch_deg: f64,
    pub fly_time_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FireControl {
    bullet_speed_mps: f64,
    yaw_bias_deg: f64,
    pitch_bias_deg: f64,
    gravity_compensation: bool,
    fire_gate_config: FireGateConfig,
}

impl FireControl {
    pub fn new(bullet_speed_mps: f64) -> Self {
        Self {
            bullet_speed_mps,
            yaw_bias_deg: 0.0,
            pitch_bias_deg: 0.0,
            gravity_compensation: true,
            fire_gate_config: FireGateConfig::default(),
        }
    }

    pub fn with_yaw_bias_deg(mut self, yaw_bias_deg: f64) -> Self {
        self.yaw_bias_deg = yaw_bias_deg;
        self
    }

    pub fn with_pitch_bias_deg(mut self, pitch_bias_deg: f64) -> Self {
        self.pitch_bias_deg = pitch_bias_deg;
        self
    }

    pub fn with_gravity_compensation(mut self, enabled: bool) -> Self {
        self.gravity_compensation = enabled;
        self
    }

    pub fn with_fire_gate_config(mut self, fire_gate_config: FireGateConfig) -> Self {
        self.fire_gate_config = fire_gate_config;
        self
    }

    pub fn fire_gate_config(&self) -> FireGateConfig {
        self.fire_gate_config
    }

    /// 根据 base 坐标系目标点计算云台指令。
    ///
    /// 坐标单位为 mm，x 向前、y 向左、z 向上；输出角度单位为 deg。
    pub fn aim_at_base_point(&self, target_base_mm: na::Point3<f64>) -> RbtResult<AimCommand> {
        if !target_base_mm.coords.iter().all(|value| value.is_finite()) {
            return Err(RbtError::StringError("target point must be finite".into()));
        }

        let horizontal_distance_mm = target_base_mm.x.hypot(target_base_mm.y);
        if horizontal_distance_mm <= EPSILON_MM {
            return Err(RbtError::StringError(
                "target horizontal distance must be positive".into(),
            ));
        }

        let yaw_deg = target_base_mm.y.atan2(target_base_mm.x).to_degrees() + self.yaw_bias_deg;
        let geometric_pitch_rad = target_base_mm.z.atan2(horizontal_distance_mm);

        let (pitch_deg, fly_time_s) = if self.gravity_compensation {
            let trajectory = solve_ballistic_trajectory(
                self.bullet_speed_mps,
                horizontal_distance_mm / MM_PER_M,
                target_base_mm.z / MM_PER_M,
            )
            .map_err(|err| RbtError::StringError(err.into()))?;
            (trajectory.pitch_deg(), trajectory.fly_time_s)
        } else {
            let fly_time_s = horizontal_distance_mm / MM_PER_M / self.bullet_speed_mps;
            if !fly_time_s.is_finite() || fly_time_s <= 0.0 {
                return Err(RbtError::StringError("fly time is invalid".into()));
            }
            (geometric_pitch_rad.to_degrees(), fly_time_s)
        };

        Ok(AimCommand {
            yaw_deg,
            pitch_deg: pitch_deg + self.pitch_bias_deg,
            fly_time_s,
        })
    }

    pub fn ctrl_data_for_target(
        &self,
        target_base_mm: na::Point3<f64>,
        shot_mode: ShotMode,
    ) -> RbtResult<CtrlData> {
        let aim = self.aim_at_base_point(target_base_mm)?;
        Ok(CtrlData {
            gimbal_yaw: aim.yaw_deg as f32,
            gimbal_pitch: aim.pitch_deg as f32,
            shot_mode,
            shot_buff_mode: ShotBuffMode::ShotBuffOff,
            aiming_state: AimingState::AimingWithTarget,
        })
    }

    pub fn no_target_ctrl_data() -> CtrlData {
        CtrlData {
            gimbal_yaw: 0.0,
            gimbal_pitch: 0.0,
            shot_mode: ShotMode::DoNothing,
            shot_buff_mode: ShotBuffMode::ShotBuffOff,
            aiming_state: AimingState::AimingNoTarget,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aims_straight_target_with_positive_ballistic_pitch() {
        let fire_control = FireControl::new(20.0);

        let aim = fire_control
            .aim_at_base_point(na::Point3::new(10_000.0, 0.0, 0.0))
            .unwrap();

        assert!(aim.yaw_deg.abs() < 1e-9);
        assert!((0.0..10.0).contains(&aim.pitch_deg));
        assert!((0.4..0.6).contains(&aim.fly_time_s));
    }

    #[test]
    fn computes_left_target_yaw() {
        let fire_control = FireControl::new(24.0).with_gravity_compensation(false);

        let aim = fire_control
            .aim_at_base_point(na::Point3::new(1_000.0, 1_000.0, 0.0))
            .unwrap();

        assert!((aim.yaw_deg - 45.0).abs() < 1e-9);
        assert!(aim.pitch_deg.abs() < 1e-9);
    }

    #[test]
    fn fills_ctrl_data_for_visible_target() {
        let fire_control = FireControl::new(24.0).with_gravity_compensation(false);

        let ctrl = fire_control
            .ctrl_data_for_target(na::Point3::new(1_000.0, 0.0, 0.0), ShotMode::AimOnly)
            .unwrap();

        assert_eq!(ctrl.shot_mode, ShotMode::AimOnly);
        assert_eq!(ctrl.shot_buff_mode, ShotBuffMode::ShotBuffOff);
        assert_eq!(ctrl.aiming_state, AimingState::AimingWithTarget);
        assert!(ctrl.gimbal_yaw.abs() < 1e-6);
        assert!(ctrl.gimbal_pitch.abs() < 1e-6);
    }

    #[test]
    fn returns_no_target_ctrl_data() {
        let ctrl = FireControl::no_target_ctrl_data();

        assert_eq!(ctrl.shot_mode, ShotMode::DoNothing);
        assert_eq!(ctrl.aiming_state, AimingState::AimingNoTarget);
    }

    #[test]
    fn fire_gate_config_is_attached_to_fire_control() {
        let cfg = FireGateConfig {
            yaw_tolerance_max_deg: 2.0,
            ..Default::default()
        };
        let fire_control = FireControl::new(24.0).with_fire_gate_config(cfg);

        assert_eq!(fire_control.fire_gate_config(), cfg);
    }
}
