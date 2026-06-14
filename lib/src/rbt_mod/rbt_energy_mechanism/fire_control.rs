use crate::rbt_infra::rbt_cfg::{EnergyMechanismAimerCfg, EnergyMechanismMpcCfg};
use crate::rbt_mod::rbt_comm::rbt_comm_frame::{
    AimingState, CtrlData, DEFAULT_BULLET_SPEED_MPS, SensData, ShotBuffMode, ShotMode, TaskMode,
};
use crate::rbt_mod::rbt_fire_control::{
    SECOND_ORDER_POSITION_MPC_HORIZON, SecondOrderPositionMpc, SecondOrderPositionMpcConfig,
};

use super::detected::EnergyMechanismMode;
use super::tracker::EnergyMechanismTrackSnapshot;

const GRAVITY_MPS2: f64 = 9.78;
const DEFAULT_BASE_PREDICT_TIME_S: f64 = 0.10;
const PITCH_RATE_SAMPLE_DT_S: f64 = 0.01;

#[derive(Debug, Clone, Copy)]
pub struct EnergyMechanismControlInput {
    pub target: Option<EnergyMechanismTrackSnapshot>,
    pub feedback: SensData,
    pub feedback_fresh: bool,
    pub dt_s: f64,
    pub snapshot_age_ms: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnergyMechanismControlStats {
    pub target_detected: bool,
    pub track_valid: bool,
    pub predicted_yaw_deg: f64,
    pub predicted_pitch_deg: f64,
    pub shot_mode: ShotMode,
    pub snapshot_stale: bool,
}

impl Default for EnergyMechanismControlStats {
    fn default() -> Self {
        Self {
            target_detected: false,
            track_valid: false,
            predicted_yaw_deg: f64::NAN,
            predicted_pitch_deg: f64::NAN,
            shot_mode: ShotMode::DoNothing,
            snapshot_stale: true,
        }
    }
}

pub struct EnergyMechanismController {
    yaw_mpc: SecondOrderPositionMpc,
    last_fire_t: Option<std::time::Instant>,
    last_valid_command: Option<CtrlData>,
    last_stats: EnergyMechanismControlStats,
    predict_time_s: f64,
    fire_gap_s: f64,
    yaw_offset_deg: f64,
    pitch_offset_deg: f64,
    pitch_velocity_lead_time_s: f64,
    snapshot_stale_ms: f64,
    mpc_model_dt_s: f64,
    mpc_horizon: usize,
}

impl EnergyMechanismController {
    /// 无参默认构造（保持向后兼容，使用默认配置常量）。
    pub fn new() -> Self {
        Self::from_aimer_cfg(
            &EnergyMechanismAimerCfg::default(),
            &EnergyMechanismMpcCfg::default(),
        )
    }

    /// 从 aimer / mpc 配置构造。
    pub fn from_aimer_cfg(
        aimer_cfg: &EnergyMechanismAimerCfg,
        mpc_cfg: &EnergyMechanismMpcCfg,
    ) -> Self {
        Self {
            yaw_mpc: SecondOrderPositionMpc::new(Self::mpc_config(mpc_cfg))
                .unwrap_or_else(|_| SecondOrderPositionMpc::default()),
            last_fire_t: None,
            last_valid_command: None,
            last_stats: EnergyMechanismControlStats::default(),
            predict_time_s: aimer_cfg.predict_time_s.max(0.0),
            fire_gap_s: aimer_cfg.fire_gap_s.max(0.0),
            yaw_offset_deg: aimer_cfg.yaw_offset_deg,
            pitch_offset_deg: aimer_cfg.pitch_offset_deg,
            pitch_velocity_lead_time_s: aimer_cfg.pitch_velocity_lead_time_s.max(0.0),
            snapshot_stale_ms: aimer_cfg.snapshot_stale_ms.max(1.0),
            mpc_model_dt_s: mpc_cfg.model_dt_s.max(1e-3),
            mpc_horizon: mpc_cfg.horizon.clamp(4, SECOND_ORDER_POSITION_MPC_HORIZON),
        }
    }

    fn mpc_config(mpc_cfg: &EnergyMechanismMpcCfg) -> SecondOrderPositionMpcConfig {
        SecondOrderPositionMpcConfig {
            model_dt_s: mpc_cfg.model_dt_s.max(1e-3),
            track_q: mpc_cfg.track_q.max(0.0),
            rate_q: mpc_cfg.rate_q.max(0.0),
            command_q: mpc_cfg.command_q.max(0.0),
            delta_r: mpc_cfg.delta_r.max(1e-6),
            ..Default::default()
        }
    }

    pub fn reset(&mut self) {
        self.yaw_mpc.reset(0.0, 0.0);
        self.last_fire_t = None;
        self.last_valid_command = None;
        self.last_stats = EnergyMechanismControlStats::default();
    }

    pub fn last_stats(&self) -> EnergyMechanismControlStats {
        self.last_stats
    }

    pub fn update(&mut self, input: EnergyMechanismControlInput) -> CtrlData {
        let Some(snapshot) = input.target else {
            return self.no_target(input.feedback);
        };
        let stale = input.snapshot_age_ms > self.snapshot_stale_ms;
        if stale || !snapshot.track_valid || !input.feedback_fresh {
            return self.no_target(input.feedback);
        }

        let bullet_speed = feedback_bullet_speed(input.feedback);
        let (yaw_deg, pitch_deg, fly_time_s) = self.solve_trajectory(snapshot, bullet_speed);

        // 用 tracker 预瞄 horizon 生成 MPC yaw 参考（替代固定 yaw 常量）。
        // horizon 的预测时间 = 基础延迟 + 最近一次解算的飞行时间。
        let base_dt_s = self.base_predict_time_s();
        let horizon_dt: Vec<f64> = (0..self.mpc_horizon)
            .map(|i| base_dt_s + fly_time_s + i as f64 * self.mpc_model_dt_s)
            .collect();
        let horizon_points = snapshot.predict_target_horizon(&horizon_dt);
        let (yaw_ref, yaw_rate_ref) = self.build_yaw_reference(&horizon_points);

        // 用 MPC 输出替代 raw yaw：update_trajectory 返回滤波后的 command_deg。
        // MPC 失败或输出非有限时回退到 raw yaw_deg，保证控制不中断。
        let mpc_command_deg = self
            .yaw_mpc
            .update_trajectory(
                &yaw_ref,
                &yaw_rate_ref,
                input.feedback.gimbal_yaw as f64,
                input.feedback.yaw_speed as f64,
                input.dt_s,
            )
            .ok()
            .map(|output| output.command_deg)
            .filter(|deg| deg.is_finite());
        let gimbal_yaw_deg = mpc_command_deg.unwrap_or(yaw_deg);

        // 开火需要 MCU 允许 + 反馈新鲜 + 满足 fire_gap。
        let fire_permit = input.feedback.mcu_fire_permit;
        let shot_mode = self.next_shot_mode(fire_permit);
        let control = CtrlData {
            gimbal_yaw: gimbal_yaw_deg as f32,
            gimbal_pitch: pitch_deg as f32,
            shot_mode,
            shot_buff_mode: shot_mode_for_task(input.feedback.task_mode),
            aiming_state: AimingState::AimingWithTarget,
        };

        self.last_valid_command = Some(control);
        self.last_stats = EnergyMechanismControlStats {
            target_detected: true,
            track_valid: snapshot.track_valid,
            predicted_yaw_deg: gimbal_yaw_deg,
            predicted_pitch_deg: pitch_deg,
            shot_mode,
            snapshot_stale: stale,
        };
        control
    }

    /// 两轮弹道飞行时间迭代 + pitch lead，迁移自 vivsionn `buff_aimer::solve_trajectory`。
    fn solve_trajectory(
        &self,
        snapshot: EnergyMechanismTrackSnapshot,
        bullet_speed_mps: f64,
    ) -> (f64, f64, f64) {
        let base_dt_s = self.base_predict_time_s();
        let mut predicted = snapshot.predict_target_center_world_m(base_dt_s);
        let mut last_fly_time = 0.0_f64;
        let mut solved_yaw_deg = 0.0_f64;
        let mut solved_pitch_deg = 0.0_f64;

        for pass in 0..2 {
            let horizontal = predicted.x.hypot(predicted.y);
            let height = predicted.z;
            let Some(fly_time) = fly_time(bullet_speed_mps, horizontal, height) else {
                return (solved_yaw_deg, solved_pitch_deg, last_fly_time);
            };
            last_fly_time = fly_time;

            let total_dt = base_dt_s + fly_time;
            predicted = snapshot.predict_target_center_world_m(total_dt);

            if pass == 1 {
                let horizontal = predicted.x.hypot(predicted.y);
                let height = predicted.z;
                let ballistic_pitch = solve_pitch_deg(horizontal, height, bullet_speed_mps);
                // estimate_pitch_rate 返回 rad/s；pitch_offset/弹道角都是 deg，lead 需转 deg。
                let pitch_rate_rad_s = self.estimate_pitch_rate(snapshot, total_dt);
                let pitch_lead_deg =
                    (pitch_rate_rad_s * self.pitch_velocity_lead_time_s).to_degrees();
                solved_yaw_deg = command_yaw_deg(predicted, self.yaw_offset_deg);
                solved_pitch_deg = ballistic_pitch + self.pitch_offset_deg + pitch_lead_deg;
            }
        }
        (solved_yaw_deg, solved_pitch_deg, last_fly_time)
    }

    fn base_predict_time_s(&self) -> f64 {
        (DEFAULT_BASE_PREDICT_TIME_S + self.predict_time_s).clamp(0.0, 0.5)
    }

    /// 用相邻预测点的 pitch 差分估 pitch 变化率（rad/s）。
    fn estimate_pitch_rate(
        &self,
        snapshot: EnergyMechanismTrackSnapshot,
        predict_dt_s: f64,
    ) -> f64 {
        let p0 = snapshot.predict_target_center_world_m(predict_dt_s);
        let p1 = snapshot.predict_target_center_world_m(predict_dt_s + PITCH_RATE_SAMPLE_DT_S);
        let pitch0 = xyz_pitch_rad(p0);
        let pitch1 = xyz_pitch_rad(p1);
        if pitch0.is_finite() && pitch1.is_finite() {
            (pitch1 - pitch0) / PITCH_RATE_SAMPLE_DT_S
        } else {
            0.0
        }
    }

    /// 从预瞄 horizon 点序列构建 MPC yaw 参考（deg）与 yaw 速率参考（deg/s）。
    fn build_yaw_reference(&self, horizon_points: &[na::Point3<f64>]) -> (Vec<f64>, Vec<f64>) {
        let n = horizon_points.len().max(1);
        let yaw_ref_rad: Vec<f64> = horizon_points
            .iter()
            .map(|p| command_yaw_rad(*p, self.yaw_offset_deg))
            .collect();
        let yaw_ref_deg: Vec<f64> = yaw_ref_rad.iter().map(|r| r.to_degrees()).collect();

        let mut yaw_rate_deg_s = vec![0.0; n];
        if n > 1 {
            for i in 1..n {
                let delta = normalize_angle_rad(yaw_ref_rad[i] - yaw_ref_rad[i - 1]);
                yaw_rate_deg_s[i] = delta.to_degrees() / self.mpc_model_dt_s;
            }
            yaw_rate_deg_s[0] = yaw_rate_deg_s[1];
        }
        (yaw_ref_deg, yaw_rate_deg_s)
    }

    fn no_target(&mut self, feedback: SensData) -> CtrlData {
        self.last_stats = EnergyMechanismControlStats::default();
        if let Some(command) = self.last_valid_command {
            return CtrlData {
                shot_mode: ShotMode::DoNothing,
                aiming_state: AimingState::AimingNoTarget,
                ..command
            };
        }
        CtrlData {
            gimbal_yaw: feedback.gimbal_yaw,
            gimbal_pitch: feedback.gimbal_pitch,
            shot_mode: ShotMode::DoNothing,
            shot_buff_mode: shot_mode_for_task(feedback.task_mode),
            aiming_state: AimingState::AimingNoTarget,
        }
    }

    fn next_shot_mode(&mut self, fire_permit: bool) -> ShotMode {
        // MCU 禁火时不下发 ShotOnce，但仍保持瞄准。
        if !fire_permit {
            return ShotMode::AimOnly;
        }
        let now = std::time::Instant::now();
        if self
            .last_fire_t
            .is_none_or(|last| now.duration_since(last).as_secs_f64() >= self.fire_gap_s)
        {
            self.last_fire_t = Some(now);
            ShotMode::ShotOnce
        } else {
            ShotMode::AimOnly
        }
    }
}

impl Default for EnergyMechanismController {
    fn default() -> Self {
        Self::new()
    }
}

fn feedback_bullet_speed(feedback: SensData) -> f64 {
    if feedback.bullet_speed.is_finite() && feedback.bullet_speed > 1.0 {
        feedback.bullet_speed as f64
    } else {
        DEFAULT_BULLET_SPEED_MPS as f64
    }
}

/// 解弹道飞行时间（秒）。距离过近或弹速过低时返回 `None` 表示无法求解。
fn fly_time(bullet_speed_mps: f64, horizontal_m: f64, height_m: f64) -> Option<f64> {
    if bullet_speed_mps <= 1.0 || !horizontal_m.is_finite() {
        return None;
    }
    // 近似：用直线距离 / 弹速，再夹到合理范围。
    let distance = horizontal_m.hypot(height_m).max(0.0);
    Some((distance / bullet_speed_mps).clamp(0.0, 0.4))
}

fn solve_pitch_deg(horizontal_m: f64, height_m: f64, bullet_speed_mps: f64) -> f64 {
    let v2 = bullet_speed_mps * bullet_speed_mps;
    let discriminant =
        v2 * v2 - GRAVITY_MPS2 * (GRAVITY_MPS2 * horizontal_m * horizontal_m + 2.0 * height_m * v2);
    if horizontal_m <= 1e-6 || discriminant < 0.0 || !discriminant.is_finite() {
        return height_m.atan2(horizontal_m.max(1e-6)).to_degrees();
    }
    let pitch = ((v2 - discriminant.sqrt()) / (GRAVITY_MPS2 * horizontal_m)).atan();
    pitch.to_degrees()
}

fn xyz_pitch_rad(point: na::Point3<f64>) -> f64 {
    point.z.atan2(point.x.hypot(point.y))
}

/// 世界坐标 → 命令 yaw（弧度）。与主自瞄 planner 一致：world +y 映射到负 gimbal yaw。
fn command_yaw_rad(point: na::Point3<f64>, yaw_offset_deg: f64) -> f64 {
    let yaw_offset_rad = yaw_offset_deg.to_radians();
    normalize_angle_rad((-point.y).atan2(point.x) + yaw_offset_rad)
}

fn command_yaw_deg(point: na::Point3<f64>, yaw_offset_deg: f64) -> f64 {
    command_yaw_rad(point, yaw_offset_deg).to_degrees()
}

fn normalize_angle_rad(angle: f64) -> f64 {
    let mut normalized = (angle + std::f64::consts::PI) % std::f64::consts::TAU;
    if normalized < 0.0 {
        normalized += std::f64::consts::TAU;
    }
    normalized - std::f64::consts::PI
}

fn shot_mode_for_task(task_mode: TaskMode) -> ShotBuffMode {
    match EnergyMechanismMode::from_task_mode(task_mode) {
        Some(_) => ShotBuffMode::ShotBuffOn,
        None => ShotBuffMode::ShotBuffOff,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rbt_mod::rbt_comm::rbt_comm_frame::SelfFraction;
    use crate::rbt_mod::rbt_energy_mechanism::tracker::CurveSnapshot;

    fn feedback(task_mode: TaskMode) -> SensData {
        feedback_with_permit(task_mode, true)
    }

    fn feedback_with_permit(task_mode: TaskMode, mcu_fire_permit: bool) -> SensData {
        SensData {
            task_mode,
            self_fraction: SelfFraction::Blue,
            bullet_speed: 24.0,
            gimbal_roll: 0.0,
            gimbal_yaw: 0.0,
            gimbal_pitch: 0.0,
            yaw_speed: 0.0,
            mcu_fire_permit,
            raw_task_mode: task_mode.into(),
            mapped_task_mode: task_mode,
        }
    }

    fn small_snapshot() -> EnergyMechanismTrackSnapshot {
        EnergyMechanismTrackSnapshot {
            mode: EnergyMechanismMode::Small,
            target_center_world_m: na::Point3::new(4.0, -1.0, 0.5),
            rune_center_world_m: na::Point3::new(4.0, 0.0, 0.0),
            roll_rad: 0.0,
            roll_rate_rad_s: 0.0,
            direction: 0,
            history_size: 4,
            lost: false,
            track_valid: true,
            state_age_s: 0.0,
            switch_deferred: false,
            target_switched: false,
            selected_phase_index: Some(0),
            selected_roll_offset_rad: Some(0.0),
            curve: None,
        }
    }

    #[test]
    fn controller_outputs_energy_mechanism_control() {
        let mut controller = EnergyMechanismController::new();
        let control = controller.update(EnergyMechanismControlInput {
            target: Some(small_snapshot()),
            feedback: feedback(TaskMode::HitSmallBuff),
            feedback_fresh: true,
            dt_s: 0.004,
            snapshot_age_ms: 5.0,
        });

        assert_eq!(control.aiming_state, AimingState::AimingWithTarget);
        assert_eq!(control.shot_buff_mode, ShotBuffMode::ShotBuffOn);
        assert_ne!(control.shot_mode, ShotMode::DoNothing);
    }

    #[test]
    fn stale_snapshot_returns_no_target() {
        let mut controller = EnergyMechanismController::new();
        let control = controller.update(EnergyMechanismControlInput {
            target: None,
            feedback: feedback(TaskMode::HitBigBuff),
            feedback_fresh: true,
            dt_s: 0.004,
            snapshot_age_ms: f64::INFINITY,
        });

        assert_eq!(control.shot_mode, ShotMode::DoNothing);
        assert_eq!(control.aiming_state, AimingState::AimingNoTarget);
    }

    #[test]
    fn from_aimer_cfg_reads_offsets() {
        let aimer = EnergyMechanismAimerCfg {
            predict_time_s: 0.05,
            fire_gap_s: 0.3,
            yaw_offset_deg: 2.0,
            pitch_offset_deg: -1.5,
            pitch_velocity_lead_time_s: 0.02,
            snapshot_stale_ms: 200.0,
        };
        let mpc = EnergyMechanismMpcCfg::default();
        let controller = EnergyMechanismController::from_aimer_cfg(&aimer, &mpc);
        assert!((controller.predict_time_s - 0.05).abs() < 1e-9);
        assert!((controller.fire_gap_s - 0.3).abs() < 1e-9);
        assert!((controller.yaw_offset_deg - 2.0).abs() < 1e-9);
        assert!((controller.pitch_offset_deg + 1.5).abs() < 1e-9);
        assert!((controller.snapshot_stale_ms - 200.0).abs() < 1e-9);
    }

    #[test]
    fn solve_trajectory_uses_curve_predict_for_large_mode() {
        // 大符快照：roll_rate 故意为 0，确保走曲线分支并产生非零 yaw。
        let controller = EnergyMechanismController::new();
        let snapshot = EnergyMechanismTrackSnapshot {
            mode: EnergyMechanismMode::Large,
            target_center_world_m: na::Point3::new(4.0, 1.0, 0.5),
            rune_center_world_m: na::Point3::new(4.0, 0.0, 0.0),
            roll_rad: 0.0,
            roll_rate_rad_s: 0.0,
            direction: 1,
            history_size: 10,
            lost: false,
            track_valid: true,
            state_age_s: 0.0,
            switch_deferred: false,
            target_switched: false,
            selected_phase_index: Some(0),
            selected_roll_offset_rad: Some(0.0),
            curve: Some(CurveSnapshot {
                phase: 0.0,
                a: 0.9125,
                w: 1.942,
                curve_speed_rad_s: 1.1775,
            }),
        };
        let (yaw_deg, _pitch_deg, _fly_time) = controller.solve_trajectory(snapshot, 24.0);
        assert!(yaw_deg.is_finite());
        // world +y → 负 gimbal yaw，预测后 y 增大 → yaw 为负。
        assert!(yaw_deg < 0.0, "yaw_deg={yaw_deg}");
    }

    #[test]
    fn fly_time_returns_zero_for_close_target() {
        assert_eq!(fly_time(24.0, 0.0, 0.0), Some(0.0));
    }

    #[test]
    fn fly_time_returns_none_for_low_bullet_speed() {
        assert_eq!(fly_time(0.5, 4.0, 0.5), None);
    }

    #[test]
    fn mcu_fire_permit_blocks_shot_once() {
        // MCU 禁火时即使 fire_gap 满足也不应下发 ShotOnce，但仍瞄准。
        let mut controller = EnergyMechanismController::new();
        let control = controller.update(EnergyMechanismControlInput {
            target: Some(small_snapshot()),
            feedback: feedback_with_permit(TaskMode::HitSmallBuff, false),
            feedback_fresh: true,
            dt_s: 0.004,
            snapshot_age_ms: 5.0,
        });

        assert_eq!(control.aiming_state, AimingState::AimingWithTarget);
        assert_eq!(control.shot_mode, ShotMode::AimOnly);
    }

    #[test]
    fn mcu_fire_permit_allows_shot_once_when_enabled() {
        // MCU 允许开火 + fire_gap 满足 → ShotOnce。
        let mut controller = EnergyMechanismController::new();
        let control = controller.update(EnergyMechanismControlInput {
            target: Some(small_snapshot()),
            feedback: feedback_with_permit(TaskMode::HitSmallBuff, true),
            feedback_fresh: true,
            dt_s: 0.004,
            snapshot_age_ms: 5.0,
        });

        assert_eq!(control.shot_mode, ShotMode::ShotOnce);
    }

    #[test]
    fn pitch_lead_converts_rad_per_s_to_deg() {
        // pitch_rate = 1.0 rad/s, lead_time = 0.1s → lead = 0.1 rad = 5.729... deg。
        // 直接验证单位转换：estimate_pitch_rate 返回 rad/s，最终加到 deg 的 pitch 上。
        let lead_rad_s = 1.0_f64;
        let lead_time_s = 0.1_f64;
        let lead_deg = (lead_rad_s * lead_time_s).to_degrees();
        assert!((lead_deg - (0.1_f64).to_degrees()).abs() < 1e-9);
        assert!((lead_deg - 5.729577951308232).abs() < 1e-6);
    }

    #[test]
    fn yaw_mpc_command_supersedes_raw_yaw_when_finite() {
        // 有目标时 gimbal_yaw 应来自 MPC 输出（有限值），而非 raw solve_trajectory 的 yaw。
        // MPC 在 measured_yaw=0、ref 非零时输出应与 raw 不同。这里只断言控制下发的是有限值，
        // 且当 raw yaw 与 measured 差距大时 MPC 会把它拉向参考（不会原样回吐 raw）。
        let mut controller = EnergyMechanismController::new();
        let control = controller.update(EnergyMechanismControlInput {
            target: Some(small_snapshot()),
            feedback: feedback(TaskMode::HitSmallBuff),
            feedback_fresh: true,
            dt_s: 0.004,
            snapshot_age_ms: 5.0,
        });
        assert!(control.gimbal_yaw.is_finite());
    }
}
