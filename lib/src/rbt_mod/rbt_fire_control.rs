//! 发控规划链路。
//!
//! 主线为 yaw planner 生成预瞄轨迹，MPC 计算云台 yaw 指令，fire gate 决定是否开火。

use crate::rbt_mod::rbt_comm::rbt_comm_frame::{AimingState, CtrlData, ShotBuffMode, ShotMode};

pub mod controller;
pub mod fire_gate;
pub mod second_order_position_mpc;
pub mod shot_phase;
pub mod yaw_planner;
pub use controller::{FireControlController, FireControlInput, FireControlStats};
pub use fire_gate::{FireGateConfig, ImpactAngleCheck, ShotSlotGate, ShotSlotGateResult};
pub use second_order_position_mpc::{
    SECOND_ORDER_POSITION_MPC_HORIZON, SecondOrderPositionMpc, SecondOrderPositionMpcConfig,
    SecondOrderPositionMpcOutput,
};
pub use shot_phase::{ShotPhaseConfig, ShotPhaseController};
pub use yaw_planner::{PlannerTarget, YawPlan, YawPlanner, YawPlannerConfig};

pub fn no_target_ctrl_data() -> CtrlData {
    CtrlData {
        gimbal_yaw: 0.0,
        gimbal_pitch: 0.0,
        shot_mode: ShotMode::DoNothing,
        shot_buff_mode: ShotBuffMode::ShotBuffOff,
        aiming_state: AimingState::AimingNoTarget,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn returns_no_target_ctrl_data() {
        let ctrl = no_target_ctrl_data();

        assert_eq!(ctrl.gimbal_yaw, 0.0);
        assert_eq!(ctrl.gimbal_pitch, 0.0);
        assert_eq!(ctrl.shot_mode, ShotMode::DoNothing);
        assert_eq!(ctrl.shot_buff_mode, ShotBuffMode::ShotBuffOff);
        assert_eq!(ctrl.aiming_state, AimingState::AimingNoTarget);
    }

    #[test]
    fn planner_mpc_and_fire_gate_chain_accepts_preview_trajectory() {
        let mut planner = YawPlanner::default();
        let target =
            PlannerTarget::new(4, [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.20, 0.0, 0.0]);

        let plan = planner.plan(Some(target), 24.0);

        assert!(plan.control);
        let mut mpc = SecondOrderPositionMpc::default();
        mpc.set_preview_window(2, 2);
        mpc.reset(0.0, 0.0);
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
        let output = mpc
            .update_trajectory(&yaw_ref_deg, &yaw_rate_ref_deg_s, 0.0, 0.0, 0.004)
            .unwrap();
        let impact_ref_deg: Vec<f64> = plan
            .impact_delta_angle_ref_rad
            .iter()
            .map(|value| value.to_degrees())
            .collect();
        let gate_result = FireGateConfig::default().count_viable_shot_slots(ShotSlotGate {
            predicted_yaw_deg: &output.predicted_yaw_deg,
            reference_yaw_deg: &output.reference_yaw_deg,
            impact_delta_angle_ref_deg: Some(&impact_ref_deg),
            tolerance_deg: 180.0,
            first_slot_time_s: 0.004,
            target_omega_rad_s: target.state()[7],
            require_impact_angle_gate: false,
            mcu_fire_permit: true,
        });

        assert_eq!(
            output.reference_yaw_deg.len(),
            SECOND_ORDER_POSITION_MPC_HORIZON
        );
        assert!(gate_result.first_slot_index.is_some());
    }
}
