use log::warn;

use crate::rbt_base::rbt_algorithm::rbt_antigravity::solve_ballistic_trajectory;
use crate::rbt_infra::rbt_err::RbtResult;
use crate::rbt_mod::rbt_comm::rbt_comm_frame::{
    AimingState, CtrlData, DEFAULT_BULLET_SPEED_MPS, SensData, ShotBuffMode, ShotMode,
};
use crate::rbt_mod::rbt_estimator::EnemyTrackSnapshot;

use super::fire_gate::{FireGateConfig, ShotSlotGate};
use super::second_order_position_mpc::{
    SecondOrderPositionMpc, SecondOrderPositionMpcConfig, SecondOrderPositionMpcOutput,
};
use super::shot_phase::{ShotPhaseConfig, ShotPhaseController, ShotPhaseInput};
use super::yaw_planner::{PlannerTarget, YawPlan, YawPlanner, YawPlannerConfig};

const FIRE_CONTROL_SNAPSHOT_STALE_MS: f64 = 180.0;
const FIRE_IMPACT_DIRECTION_EPSILON_RAD_S: f64 = 0.2;
const PREVIEW_INDEX: usize = 2;

#[derive(Debug, Clone, Copy)]
pub struct FireControlInput {
    pub target: Option<EnemyTrackSnapshot>,
    pub feedback: SensData,
    pub feedback_fresh: bool,
    pub dt_s: f64,
    pub snapshot_age_ms: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FireControlStats {
    pub target_detected: bool,
    pub static_bypass_active: bool,
    pub planner_active: bool,
    pub preview_mpc_active: bool,
    pub tolerance_deg: f64,
    pub yaw_error_deg: f64,
    pub pitch_error_deg: f64,
    pub shot_mode: ShotMode,
    pub viable_slot_count: usize,
    pub first_slot_error_deg: f64,
    pub next_slot_delay_ms: f64,
    pub gate_valid: bool,
    pub gate_mcu: bool,
    pub gate_preview: bool,
    pub gate_impact: bool,
    pub gate_slot: bool,
    pub gate_motion: bool,
    pub gate_observation: bool,
    pub gate_follow: bool,
    pub gate_command_stable: bool,
}

impl Default for FireControlStats {
    fn default() -> Self {
        Self {
            target_detected: false,
            static_bypass_active: false,
            planner_active: false,
            preview_mpc_active: false,
            tolerance_deg: FireGateConfig::default().yaw_tolerance_deg(f64::NAN),
            yaw_error_deg: 0.0,
            pitch_error_deg: 0.0,
            shot_mode: ShotMode::DoNothing,
            viable_slot_count: 0,
            first_slot_error_deg: f64::NAN,
            next_slot_delay_ms: f64::NAN,
            gate_valid: false,
            gate_mcu: false,
            gate_preview: false,
            gate_impact: false,
            gate_slot: false,
            gate_motion: false,
            gate_observation: false,
            gate_follow: false,
            gate_command_stable: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct TargetControlState {
    control_yaw_deg: f64,
    control_pitch_deg: f64,
    tolerance_deg: f64,
    yaw_error_deg: f64,
    pitch_error_deg: f64,
    viable_slot_count: usize,
    first_slot_error_deg: f64,
    gate_mcu: bool,
    gate_preview: bool,
    gate_impact: bool,
    gate_slot: bool,
    gate_motion: bool,
    gate_observation: bool,
    gate_follow: bool,
    gate_command_stable: bool,
    planner_active: bool,
    preview_mpc_active: bool,
    static_bypass_active: bool,
}

pub struct FireControlController {
    fire_gate: FireGateConfig,
    yaw_planner: YawPlanner,
    yaw_mpc: SecondOrderPositionMpc,
    shot_phase: ShotPhaseController,
    last_auto_command: Option<CtrlData>,
    last_stats: FireControlStats,
}

impl FireControlController {
    pub fn new() -> RbtResult<Self> {
        let fire_gate = FireGateConfig::default();
        let shot_phase = ShotPhaseController::new(ShotPhaseConfig {
            model_dt_s: fire_gate.model_dt_s,
            fire_rate_hz: fire_gate.fire_rate_hz,
            ..ShotPhaseConfig::default()
        });
        Ok(Self {
            fire_gate,
            yaw_planner: YawPlanner::new(YawPlannerConfig {
                preview_dt_s: fire_gate.model_dt_s,
                preview_horizon: super::SECOND_ORDER_POSITION_MPC_HORIZON,
                armor_enter_angle_deg: 55.0,
                armor_leave_angle_deg: 20.0,
                ..YawPlannerConfig::default()
            }),
            yaw_mpc: SecondOrderPositionMpc::new(SecondOrderPositionMpcConfig::default())?,
            shot_phase,
            last_auto_command: None,
            last_stats: FireControlStats::default(),
        })
    }

    pub fn reset(&mut self) {
        self.yaw_planner.reset();
        self.yaw_mpc.reset(0.0, 0.0);
        self.shot_phase.reset();
        self.last_auto_command = None;
        self.last_stats = FireControlStats::default();
    }

    pub fn last_stats(&self) -> FireControlStats {
        self.last_stats
    }

    pub fn update(&mut self, input: FireControlInput) -> CtrlData {
        let Some(target) = input.target else {
            return self.no_target(input.feedback);
        };
        if input.snapshot_age_ms > FIRE_CONTROL_SNAPSHOT_STALE_MS || !target.track_valid {
            return self.no_target(input.feedback);
        }

        let planner_target = PlannerTarget::from_snapshot(&target);
        let bullet_speed_mps = feedback_bullet_speed(&input.feedback);
        let plan = self
            .yaw_planner
            .plan(Some(planner_target), bullet_speed_mps);
        if !plan.control {
            self.yaw_mpc.reset(
                input.feedback.gimbal_yaw as f64,
                input.feedback.yaw_speed as f64,
            );
            self.shot_phase.reset();
            return self.aim_only(
                input.feedback,
                FireControlStats {
                    target_detected: true,
                    gate_motion: target.motion_uniform,
                    gate_observation: target.observation_stable,
                    shot_mode: ShotMode::AimOnly,
                    ..FireControlStats::default()
                },
            );
        }

        let control_state = if target.motion_state.is_static() {
            self.apply_static_target(&target, &plan, input)
        } else {
            self.apply_dynamic_target(&target, planner_target, &plan, input)
        };

        let hard_gate_ok =
            control_state.gate_mcu && control_state.gate_motion && control_state.gate_observation;
        let phase = self.shot_phase.update(ShotPhaseInput {
            hard_gate_ok,
            viable_slot_count: control_state.viable_slot_count,
            dt_s: input.dt_s,
        });
        let shot_mode = phase.shot_mode;

        let control_data = CtrlData {
            gimbal_yaw: control_state.control_yaw_deg as f32,
            gimbal_pitch: control_state.control_pitch_deg as f32,
            shot_mode,
            shot_buff_mode: ShotBuffMode::ShotBuffOff,
            aiming_state: AimingState::AimingWithTarget,
        };
        self.last_auto_command = Some(control_data);
        self.last_stats = FireControlStats {
            target_detected: true,
            static_bypass_active: control_state.static_bypass_active,
            planner_active: control_state.planner_active,
            preview_mpc_active: control_state.preview_mpc_active,
            tolerance_deg: control_state.tolerance_deg,
            yaw_error_deg: control_state.yaw_error_deg,
            pitch_error_deg: control_state.pitch_error_deg,
            shot_mode,
            viable_slot_count: control_state.viable_slot_count,
            first_slot_error_deg: control_state.first_slot_error_deg,
            next_slot_delay_ms: phase.next_slot_delay_ms,
            gate_valid: true,
            gate_mcu: control_state.gate_mcu,
            gate_preview: control_state.gate_preview,
            gate_impact: control_state.gate_impact,
            gate_slot: control_state.gate_slot,
            gate_motion: control_state.gate_motion,
            gate_observation: control_state.gate_observation,
            gate_follow: control_state.gate_follow,
            gate_command_stable: control_state.gate_command_stable,
        };
        control_data
    }

    fn no_target(&mut self, feedback: SensData) -> CtrlData {
        self.yaw_planner.reset();
        self.yaw_mpc
            .reset(feedback.gimbal_yaw as f64, feedback.yaw_speed as f64);
        self.shot_phase.reset();
        self.last_auto_command = None;
        self.last_stats = FireControlStats::default();

        CtrlData {
            gimbal_yaw: feedback.gimbal_yaw,
            gimbal_pitch: feedback.gimbal_pitch,
            shot_mode: ShotMode::DoNothing,
            shot_buff_mode: ShotBuffMode::ShotBuffOff,
            aiming_state: AimingState::AimingNoTarget,
        }
    }

    fn aim_only(&mut self, feedback: SensData, stats: FireControlStats) -> CtrlData {
        let control_data = CtrlData {
            gimbal_yaw: feedback.gimbal_yaw,
            gimbal_pitch: feedback.gimbal_pitch,
            shot_mode: ShotMode::AimOnly,
            shot_buff_mode: ShotBuffMode::ShotBuffOff,
            aiming_state: AimingState::AimingWithTarget,
        };
        self.last_auto_command = Some(control_data);
        self.last_stats = stats;
        control_data
    }

    fn apply_static_target(
        &mut self,
        target: &EnemyTrackSnapshot,
        plan: &YawPlan,
        input: FireControlInput,
    ) -> TargetControlState {
        // 稳定目标直接用当前命中角。预瞄 MPC 在这里反而会把模型滞后带进指令。
        let control_yaw_deg = normalize_angle_deg(plan.target_yaw_rad.to_degrees());
        let control_pitch_deg = armor_pitch_deg(plan.target_position_m, input.feedback);
        self.yaw_mpc
            .reset(control_yaw_deg, input.feedback.yaw_speed as f64);

        let tolerance_deg = self
            .fire_gate
            .yaw_tolerance_deg(plan.target_position_m.x.hypot(plan.target_position_m.y));
        let yaw_error_deg = angle_diff_deg(control_yaw_deg, input.feedback.gimbal_yaw as f64);
        let pitch_error_deg = angle_diff_deg(control_pitch_deg, input.feedback.gimbal_pitch as f64);
        let gate_mcu = input.feedback_fresh && input.feedback.mcu_fire_permit && target.fire_permit;
        let current_error_ok =
            self.fire_gate
                .follow_is_ready(yaw_error_deg, pitch_error_deg, tolerance_deg);
        let ready_slot_count = self
            .shot_phase
            .config()
            .auto_enter_slot_count
            .max(self.shot_phase.config().auto_hold_slot_count)
            .max(1);
        let viable_slot_count =
            if current_error_ok && gate_mcu && target.motion_uniform && target.observation_stable {
                ready_slot_count
            } else {
                0
            };

        TargetControlState {
            control_yaw_deg,
            control_pitch_deg,
            tolerance_deg,
            yaw_error_deg,
            pitch_error_deg,
            viable_slot_count,
            first_slot_error_deg: yaw_error_deg,
            gate_mcu,
            gate_preview: current_error_ok,
            gate_impact: true,
            gate_slot: viable_slot_count >= self.shot_phase.config().auto_hold_slot_count.max(1),
            gate_motion: target.motion_uniform,
            gate_observation: target.observation_stable,
            gate_follow: current_error_ok,
            gate_command_stable: current_error_ok,
            planner_active: true,
            preview_mpc_active: false,
            static_bypass_active: true,
        }
    }

    fn apply_dynamic_target(
        &mut self,
        target: &EnemyTrackSnapshot,
        planner_target: PlannerTarget,
        plan: &YawPlan,
        input: FireControlInput,
    ) -> TargetControlState {
        let yaw_ref_deg: Vec<f64> = plan
            .yaw_ref_rad
            .iter()
            .map(|value| value.to_degrees())
            .collect();
        let yaw_rate_ref_deg_s: Vec<f64> = plan
            .yaw_rate_ref_rad_s
            .iter()
            .map(|value| value.to_degrees())
            .collect();

        self.yaw_mpc
            .set_preview_window(PREVIEW_INDEX, PREVIEW_INDEX);
        let mpc_output = self
            .yaw_mpc
            .update_trajectory(
                &yaw_ref_deg,
                &yaw_rate_ref_deg_s,
                input.feedback.gimbal_yaw as f64,
                input.feedback.yaw_speed as f64,
                input.dt_s,
            )
            .map_err(|err| {
                warn!("fire_control: yaw trajectory MPC failed: {err}");
                err
            })
            .ok();

        let control_yaw_deg = mpc_output
            .as_ref()
            .map(|output| output.command_deg)
            .unwrap_or_else(|| normalize_angle_deg(plan.target_yaw_rad.to_degrees()));
        let control_pitch_deg = armor_pitch_deg(plan.target_position_m, input.feedback);
        if mpc_output.is_none() {
            self.yaw_mpc.reset(
                input.feedback.gimbal_yaw as f64,
                input.feedback.yaw_speed as f64,
            );
        }

        let tolerance_deg = self
            .fire_gate
            .yaw_tolerance_deg(plan.target_position_m.x.hypot(plan.target_position_m.y));
        let gate_mcu = input.feedback_fresh && input.feedback.mcu_fire_permit && target.fire_permit;
        let target_omega_rad_s = planner_target.state()[7];
        let require_impact_angle_gate =
            target_omega_rad_s.abs() > FIRE_IMPACT_DIRECTION_EPSILON_RAD_S;

        let Some(output) = mpc_output.as_ref() else {
            let yaw_error_deg = angle_diff_deg(control_yaw_deg, input.feedback.gimbal_yaw as f64);
            let pitch_error_deg =
                angle_diff_deg(control_pitch_deg, input.feedback.gimbal_pitch as f64);
            return TargetControlState {
                control_yaw_deg,
                control_pitch_deg,
                tolerance_deg,
                yaw_error_deg,
                pitch_error_deg,
                viable_slot_count: 0,
                first_slot_error_deg: f64::NAN,
                gate_mcu,
                gate_preview: false,
                gate_impact: !require_impact_angle_gate,
                gate_slot: false,
                gate_motion: target.motion_uniform,
                gate_observation: target.observation_stable,
                gate_follow: false,
                gate_command_stable: self.command_is_stable(
                    control_yaw_deg,
                    control_pitch_deg,
                    tolerance_deg,
                ),
                planner_active: true,
                preview_mpc_active: false,
                static_bypass_active: false,
            };
        };

        self.dynamic_gate_state(
            target,
            planner_target,
            plan,
            input,
            output,
            control_yaw_deg,
            control_pitch_deg,
            tolerance_deg,
            gate_mcu,
            require_impact_angle_gate,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn dynamic_gate_state(
        &self,
        target: &EnemyTrackSnapshot,
        planner_target: PlannerTarget,
        plan: &YawPlan,
        input: FireControlInput,
        output: &SecondOrderPositionMpcOutput,
        control_yaw_deg: f64,
        control_pitch_deg: f64,
        tolerance_deg: f64,
        gate_mcu: bool,
        require_impact_angle_gate: bool,
    ) -> TargetControlState {
        let impact_ref_deg: Vec<f64> = plan
            .impact_delta_angle_ref_rad
            .iter()
            .map(|value| value.to_degrees())
            .collect();
        let gate_result = self.fire_gate.count_viable_shot_slots(ShotSlotGate {
            predicted_yaw_deg: &output.predicted_yaw_deg,
            reference_yaw_deg: &output.reference_yaw_deg,
            impact_delta_angle_ref_deg: Some(&impact_ref_deg),
            tolerance_deg,
            first_slot_time_s: self.shot_phase.first_slot_time_s(),
            target_omega_rad_s: planner_target.state()[7],
            require_impact_angle_gate,
            mcu_fire_permit: gate_mcu,
        });
        let first_slot_error_deg = gate_result.first_slot_error_deg.unwrap_or(f64::NAN);
        let yaw_error_deg = gate_result
            .first_slot_error_deg
            .unwrap_or(output.preview_tracking_error_deg)
            .abs();
        let gate_preview = if first_slot_error_deg.is_finite() {
            first_slot_error_deg < tolerance_deg
        } else {
            output.preview_tracking_valid && output.preview_tracking_error_deg < tolerance_deg
        };
        let gate_impact = if require_impact_angle_gate {
            gate_result
                .first_slot_impact
                .is_some_and(|impact| impact.in_window)
        } else {
            true
        };
        let pitch_error_deg = angle_diff_deg(control_pitch_deg, input.feedback.gimbal_pitch as f64);
        let gate_follow =
            self.fire_gate
                .follow_is_ready(yaw_error_deg, pitch_error_deg, tolerance_deg);
        let hold_slot_count = self.shot_phase.config().auto_hold_slot_count.max(1);

        TargetControlState {
            control_yaw_deg,
            control_pitch_deg,
            tolerance_deg,
            yaw_error_deg,
            pitch_error_deg,
            viable_slot_count: gate_result.viable_slot_count,
            first_slot_error_deg,
            gate_mcu,
            gate_preview,
            gate_impact,
            gate_slot: gate_result.viable_slot_count >= hold_slot_count,
            gate_motion: target.motion_uniform,
            gate_observation: target.observation_stable,
            gate_follow,
            gate_command_stable: self.command_is_stable(
                control_yaw_deg,
                control_pitch_deg,
                tolerance_deg,
            ),
            planner_active: true,
            preview_mpc_active: true,
            static_bypass_active: false,
        }
    }

    fn command_is_stable(
        &self,
        control_yaw_deg: f64,
        control_pitch_deg: f64,
        tolerance_deg: f64,
    ) -> bool {
        let Some(last) = self.last_auto_command else {
            return false;
        };
        let yaw_delta_deg = angle_diff_deg(control_yaw_deg, last.gimbal_yaw as f64);
        let pitch_delta_deg = angle_diff_deg(control_pitch_deg, last.gimbal_pitch as f64);
        self.fire_gate
            .command_is_stable(yaw_delta_deg, pitch_delta_deg, tolerance_deg)
    }
}

fn feedback_bullet_speed(feedback: &SensData) -> f64 {
    if feedback.bullet_speed.is_finite() && feedback.bullet_speed > 1.0 {
        f64::from(feedback.bullet_speed)
    } else {
        f64::from(DEFAULT_BULLET_SPEED_MPS)
    }
}

fn normalize_angle_deg(angle_deg: f64) -> f64 {
    let mut result = (angle_deg + 180.0) % 360.0;
    if result < 0.0 {
        result += 360.0;
    }
    result - 180.0
}

fn angle_diff_deg(a: f64, b: f64) -> f64 {
    normalize_angle_deg(a - b).abs()
}

fn armor_pitch_deg(target_position_m: na::Point3<f64>, feedback: SensData) -> f64 {
    let horizontal_distance_m = target_position_m.x.hypot(target_position_m.y);
    solve_ballistic_trajectory(
        feedback_bullet_speed(&feedback),
        horizontal_distance_m,
        target_position_m.z,
    )
    .map(|solution| solution.pitch_deg())
    .unwrap_or_else(|err| {
        warn!("fire_control: ballistic pitch fallback used: {err}");
        target_position_m
            .z
            .atan2(horizontal_distance_m.max(1e-6))
            .to_degrees()
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rbt_mod::rbt_comm::rbt_comm_frame::{SelfFraction, TaskMode};
    use crate::rbt_mod::rbt_estimator::TargetMotionState;

    fn feedback(mcu_fire_permit: bool) -> SensData {
        SensData {
            task_mode: TaskMode::AutoShot,
            self_fraction: SelfFraction::Blue,
            bullet_speed: DEFAULT_BULLET_SPEED_MPS,
            gimbal_roll: 0.0,
            gimbal_yaw: 0.0,
            gimbal_pitch: 1.5,
            yaw_speed: 0.0,
            mcu_fire_permit,
            raw_task_mode: TaskMode::AutoShot.into(),
            mapped_task_mode: TaskMode::AutoShot,
        }
    }

    fn target(motion_state: TargetMotionState) -> EnemyTrackSnapshot {
        EnemyTrackSnapshot {
            enemy_id: crate::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::EnemyId::Infantry3,
            armor_count: 4,
            center_xy_m: na::Point2::new(3.0, 0.0),
            center_velocity_xy_mps: na::Vector2::zeros(),
            armor_z_m: 0.0,
            armor_z_velocity_mps: 0.0,
            body_yaw_rad: 0.0,
            body_yaw_rate_rad_s: if motion_state.is_static() { 0.0 } else { 1.0 },
            primary_radius_m: 0.20,
            secondary_radius_delta_m: 0.0,
            height_delta_m: 0.0,
            state_age_s: 0.0,
            track_valid: true,
            fire_permit: true,
            motion_state,
            motion_uniform: true,
            observation_stable: true,
            motion_translation_burst_metric: 0.0,
            motion_translation_drift_metric: 0.0,
            motion_yaw_accel_metric: 0.0,
        }
    }

    fn raised_target(motion_state: TargetMotionState) -> EnemyTrackSnapshot {
        EnemyTrackSnapshot {
            armor_z_m: 0.5,
            ..target(motion_state)
        }
    }

    fn input(
        target: Option<EnemyTrackSnapshot>,
        feedback_fresh: bool,
        mcu: bool,
    ) -> FireControlInput {
        FireControlInput {
            target,
            feedback: feedback(mcu),
            feedback_fresh,
            dt_s: 0.004,
            snapshot_age_ms: 10.0,
        }
    }

    #[test]
    fn no_target_resets_to_do_nothing() {
        let mut controller = FireControlController::new().unwrap();

        let control = controller.update(input(None, true, true));

        assert_eq!(control.shot_mode, ShotMode::DoNothing);
        assert_eq!(control.aiming_state, AimingState::AimingNoTarget);
        assert!(!controller.last_stats().target_detected);
    }

    #[test]
    fn stale_or_invalid_target_is_suppressed() {
        let mut controller = FireControlController::new().unwrap();
        let mut stale = input(Some(target(TargetMotionState::Dynamic)), true, true);
        stale.snapshot_age_ms = 250.0;

        let control = controller.update(stale);

        assert_eq!(control.shot_mode, ShotMode::DoNothing);
        assert_eq!(control.aiming_state, AimingState::AimingNoTarget);
    }

    #[test]
    fn static_target_uses_direct_yaw_without_preview_mpc() {
        let mut controller = FireControlController::new().unwrap();

        let control = controller.update(input(Some(target(TargetMotionState::Static)), true, true));
        let stats = controller.last_stats();

        assert!(stats.target_detected);
        assert!(stats.static_bypass_active);
        assert!(!stats.preview_mpc_active);
        assert!(control.gimbal_yaw.abs() < 1e-4);
    }

    #[test]
    fn armor_target_outputs_ballistic_pitch_command() {
        let mut controller = FireControlController::new().unwrap();

        let control = controller.update(input(
            Some(raised_target(TargetMotionState::Static)),
            true,
            true,
        ));
        let stats = controller.last_stats();

        assert!(control.gimbal_pitch > feedback(true).gimbal_pitch);
        assert!(stats.pitch_error_deg > 0.0);
    }

    #[test]
    fn dynamic_target_uses_preview_mpc_and_can_auto_fire() {
        let mut controller = FireControlController::new().unwrap();

        let control =
            controller.update(input(Some(target(TargetMotionState::Dynamic)), true, true));
        let stats = controller.last_stats();

        assert!(stats.preview_mpc_active);
        assert!(stats.viable_slot_count >= 1);
        assert_eq!(control.shot_mode, ShotMode::AutoFire);
    }

    #[test]
    fn mcu_or_stale_feedback_blocks_fire() {
        let mut controller = FireControlController::new().unwrap();

        let mcu_blocked =
            controller.update(input(Some(target(TargetMotionState::Dynamic)), true, false));
        let stale_feedback =
            controller.update(input(Some(target(TargetMotionState::Dynamic)), false, true));

        assert_eq!(mcu_blocked.shot_mode, ShotMode::AimOnly);
        assert_eq!(stale_feedback.shot_mode, ShotMode::AimOnly);
        assert!(!controller.last_stats().gate_mcu);
    }

    #[test]
    fn unstable_observation_blocks_fire_but_keeps_aiming() {
        let mut controller = FireControlController::new().unwrap();
        let mut target = target(TargetMotionState::Static);
        target.observation_stable = false;

        let control = controller.update(input(Some(target), true, true));
        let stats = controller.last_stats();

        assert_eq!(control.shot_mode, ShotMode::AimOnly);
        assert!(stats.gate_mcu);
        assert!(!stats.gate_observation);
    }
}
