use crate::rbt_mod::rbt_comm::rbt_comm_frame::{
    AimingState, CtrlData, DEFAULT_BULLET_SPEED_MPS, SensData, ShotBuffMode, ShotMode, TaskMode,
};
use crate::rbt_mod::rbt_fire_control::SecondOrderPositionMpc;

use super::detected::EnergyMechanismMode;
use super::tracker::EnergyMechanismTrackSnapshot;

const SNAPSHOT_STALE_MS: f64 = 180.0;
const BASE_PREDICT_TIME_S: f64 = 0.10;
const FIRE_GAP_S: f64 = 0.20;
const GRAVITY_MPS2: f64 = 9.78;

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
}

impl EnergyMechanismController {
    pub fn new() -> Self {
        Self {
            yaw_mpc: SecondOrderPositionMpc::default(),
            last_fire_t: None,
            last_valid_command: None,
            last_stats: EnergyMechanismControlStats::default(),
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
        let stale = input.snapshot_age_ms > SNAPSHOT_STALE_MS;
        if stale || !snapshot.track_valid || !input.feedback_fresh {
            return self.no_target(input.feedback);
        }

        let bullet_speed = feedback_bullet_speed(input.feedback);
        let predict_time_s = BASE_PREDICT_TIME_S + approximate_fly_time(snapshot, bullet_speed);
        let predicted = snapshot.predict_target_center_world_m(predict_time_s);
        let yaw_rad = predicted.y.atan2(predicted.x);
        let yaw_deg = normalize_angle_deg((-yaw_rad).to_degrees());
        let pitch_deg = solve_pitch_deg(predicted, bullet_speed);
        let shot_mode = self.next_shot_mode();
        let control = CtrlData {
            gimbal_yaw: yaw_deg as f32,
            gimbal_pitch: pitch_deg as f32,
            shot_mode,
            shot_buff_mode: shot_mode_for_task(input.feedback.task_mode),
            aiming_state: AimingState::AimingWithTarget,
        };

        let _ = self.yaw_mpc.update_trajectory(
            &[yaw_deg; 8],
            &[0.0; 8],
            input.feedback.gimbal_yaw as f64,
            input.feedback.yaw_speed as f64,
            input.dt_s,
        );
        self.last_valid_command = Some(control);
        self.last_stats = EnergyMechanismControlStats {
            target_detected: true,
            track_valid: snapshot.track_valid,
            predicted_yaw_deg: yaw_deg,
            predicted_pitch_deg: pitch_deg,
            shot_mode,
            snapshot_stale: stale,
        };
        control
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

    fn next_shot_mode(&mut self) -> ShotMode {
        let now = std::time::Instant::now();
        if self
            .last_fire_t
            .is_none_or(|last| now.duration_since(last).as_secs_f64() >= FIRE_GAP_S)
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

fn approximate_fly_time(snapshot: EnergyMechanismTrackSnapshot, bullet_speed_mps: f64) -> f64 {
    let target = snapshot.target_center_world_m;
    let distance = target.x.hypot(target.y).max(0.0);
    if bullet_speed_mps <= 1.0 {
        0.0
    } else {
        (distance / bullet_speed_mps).clamp(0.0, 0.4)
    }
}

fn solve_pitch_deg(target_world_m: na::Point3<f64>, bullet_speed_mps: f64) -> f64 {
    let horizontal = target_world_m.x.hypot(target_world_m.y);
    let height = target_world_m.z;
    let v2 = bullet_speed_mps * bullet_speed_mps;
    let discriminant =
        v2 * v2 - GRAVITY_MPS2 * (GRAVITY_MPS2 * horizontal * horizontal + 2.0 * height * v2);
    if horizontal <= 1e-6 || discriminant < 0.0 || !discriminant.is_finite() {
        return height.atan2(horizontal.max(1e-6)).to_degrees();
    }
    let pitch = ((v2 - discriminant.sqrt()) / (GRAVITY_MPS2 * horizontal)).atan();
    pitch.to_degrees()
}

fn normalize_angle_deg(mut angle: f64) -> f64 {
    while angle > 180.0 {
        angle -= 360.0;
    }
    while angle < -180.0 {
        angle += 360.0;
    }
    angle
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

    fn feedback(task_mode: TaskMode) -> SensData {
        SensData {
            task_mode,
            self_fraction: SelfFraction::Blue,
            bullet_speed: 24.0,
            gimbal_roll: 0.0,
            gimbal_yaw: 0.0,
            gimbal_pitch: 0.0,
            yaw_speed: 0.0,
            mcu_fire_permit: true,
            raw_task_mode: task_mode.into(),
            mapped_task_mode: task_mode,
        }
    }

    #[test]
    fn controller_outputs_energy_mechanism_control() {
        let mut controller = EnergyMechanismController::new();
        let snapshot = EnergyMechanismTrackSnapshot {
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
        };

        let control = controller.update(EnergyMechanismControlInput {
            target: Some(snapshot),
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
}
