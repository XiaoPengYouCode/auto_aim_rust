use std::collections::VecDeque;

use super::detected::EnergyMechanismMode;
use super::solved::EnergyMechanismSolvedTarget;

const HISTORY_CAPACITY: usize = 48;
const LOST_TIMEOUT_S: f64 = 0.35;
const LARGE_CURVE_A: f64 = 0.78;
const LARGE_CURVE_W: f64 = 1.884;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnergyMechanismTrackSnapshot {
    pub mode: EnergyMechanismMode,
    pub target_center_world_m: na::Point3<f64>,
    pub rune_center_world_m: na::Point3<f64>,
    pub roll_rad: f64,
    pub roll_rate_rad_s: f64,
    pub direction: i32,
    pub history_size: usize,
    pub lost: bool,
    pub track_valid: bool,
    pub state_age_s: f64,
}

impl EnergyMechanismTrackSnapshot {
    pub fn predict_target_center_world_m(self, dt_s: f64) -> na::Point3<f64> {
        let radius = self.target_center_world_m - self.rune_center_world_m;
        let angle = if self.mode == EnergyMechanismMode::Large {
            self.roll_rad
                + self.direction as f64
                    * LARGE_CURVE_A
                    * ((LARGE_CURVE_W * dt_s.max(0.0)).sin() / LARGE_CURVE_W)
        } else {
            self.roll_rad + self.roll_rate_rad_s * dt_s.max(0.0)
        };
        let radius_norm = radius.norm();
        if radius_norm <= 1e-9 {
            return self.target_center_world_m;
        }
        na::Point3::new(
            self.rune_center_world_m.x,
            self.rune_center_world_m.y + radius_norm * angle.cos(),
            self.rune_center_world_m.z + radius_norm * angle.sin(),
        )
    }
}

#[derive(Debug, Clone)]
struct RollSample {
    time_s: f64,
    roll_rad: f64,
}

#[derive(Debug, Clone)]
pub struct EnergyMechanismTracker {
    mode: EnergyMechanismMode,
    initialized: bool,
    start_tp: Option<std::time::Instant>,
    last_update_tp: Option<std::time::Instant>,
    last_seen_tp: Option<std::time::Instant>,
    last_target_center_world_m: na::Point3<f64>,
    last_rune_center_world_m: na::Point3<f64>,
    filtered_roll_rad: f64,
    filtered_roll_rate_rad_s: f64,
    direction: i32,
    history: VecDeque<RollSample>,
}

impl EnergyMechanismTracker {
    pub fn new(mode: EnergyMechanismMode) -> Self {
        Self {
            mode,
            initialized: false,
            start_tp: None,
            last_update_tp: None,
            last_seen_tp: None,
            last_target_center_world_m: na::Point3::origin(),
            last_rune_center_world_m: na::Point3::origin(),
            filtered_roll_rad: 0.0,
            filtered_roll_rate_rad_s: 0.0,
            direction: 0,
            history: VecDeque::new(),
        }
    }

    pub fn reset(&mut self, mode: EnergyMechanismMode) {
        *self = Self::new(mode);
    }

    pub fn update(
        &mut self,
        mode: EnergyMechanismMode,
        target: Option<&EnergyMechanismSolvedTarget>,
    ) -> Option<EnergyMechanismTrackSnapshot> {
        if self.mode != mode {
            self.reset(mode);
        }
        let now = std::time::Instant::now();
        let start = *self.start_tp.get_or_insert(now);
        let time_s = now.duration_since(start).as_secs_f64();
        let dt_s = self
            .last_update_tp
            .map(|last| now.duration_since(last).as_secs_f64().clamp(0.001, 0.08))
            .unwrap_or(0.01);
        self.last_update_tp = Some(now);

        if let Some(target) = target {
            self.correct(target, time_s, dt_s, now);
        }

        self.snapshot(now)
    }

    pub fn snapshot(&self, now: std::time::Instant) -> Option<EnergyMechanismTrackSnapshot> {
        if !self.initialized {
            return None;
        }
        let state_age_s = self
            .last_seen_tp
            .map(|last| now.duration_since(last).as_secs_f64())
            .unwrap_or(f64::INFINITY);
        let lost = state_age_s > LOST_TIMEOUT_S;
        Some(EnergyMechanismTrackSnapshot {
            mode: self.mode,
            target_center_world_m: self.last_target_center_world_m,
            rune_center_world_m: self.last_rune_center_world_m,
            roll_rad: self.filtered_roll_rad,
            roll_rate_rad_s: self.filtered_roll_rate_rad_s,
            direction: self.direction,
            history_size: self.history.len(),
            lost,
            track_valid: !lost && self.history.len() >= 2,
            state_age_s,
        })
    }

    fn correct(
        &mut self,
        target: &EnergyMechanismSolvedTarget,
        time_s: f64,
        dt_s: f64,
        now: std::time::Instant,
    ) {
        let observed_roll = normalize_angle(target.observed_roll_rad);
        if !self.initialized {
            self.initialized = true;
            self.filtered_roll_rad = observed_roll;
            self.filtered_roll_rate_rad_s = 0.0;
        } else {
            let delta = normalize_angle(observed_roll - self.filtered_roll_rad);
            let raw_rate = delta / dt_s;
            self.filtered_roll_rad = normalize_angle(self.filtered_roll_rad + delta * 0.55);
            self.filtered_roll_rate_rad_s =
                0.80 * self.filtered_roll_rate_rad_s + 0.20 * raw_rate.clamp(-3.5, 3.5);
            if raw_rate.abs() > 0.05 {
                self.direction = if raw_rate > 0.0 { 1 } else { -1 };
            }
        }

        self.last_target_center_world_m = target.pose.target_center_world_m;
        self.last_rune_center_world_m = target.pose.rune_center_world_m;
        self.last_seen_tp = Some(now);
        self.history.push_back(RollSample {
            time_s,
            roll_rad: observed_roll,
        });
        while self.history.len() > HISTORY_CAPACITY {
            self.history.pop_front();
        }
        self.fit_direction_from_history();
    }

    fn fit_direction_from_history(&mut self) {
        if self.history.len() < 3 {
            return;
        }
        let mut total_delta = 0.0;
        let mut total_time = 0.0;
        for pair in self.history.as_slices().0.windows(2) {
            total_delta += normalize_angle(pair[1].roll_rad - pair[0].roll_rad);
            total_time += pair[1].time_s - pair[0].time_s;
        }
        if total_time > 1e-6 {
            let rate = total_delta / total_time;
            if rate.abs() > 0.03 {
                self.direction = if rate > 0.0 { 1 } else { -1 };
                self.filtered_roll_rate_rad_s = rate.clamp(-3.0, 3.0);
            }
        }
    }
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle > std::f64::consts::PI {
        angle -= std::f64::consts::TAU;
    }
    while angle < -std::f64::consts::PI {
        angle += std::f64::consts::TAU;
    }
    angle
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rbt_mod::rbt_energy_mechanism::solved::EnergyMechanismPose;

    fn target(roll_rad: f64) -> EnergyMechanismSolvedTarget {
        EnergyMechanismSolvedTarget {
            mode: EnergyMechanismMode::Small,
            pose: EnergyMechanismPose {
                rune_center_world_m: na::Point3::origin(),
                target_center_world_m: na::Point3::new(1.0, roll_rad.cos(), roll_rad.sin()),
                yaw_rad: 0.0,
                pitch_rad: 0.0,
                roll_rad,
                reprojection_error_px: 1.0,
            },
            image_r_center: na::Point2::new(320.0, 320.0),
            image_target_center: na::Point2::new(
                (320.0 + 100.0 * roll_rad.cos()) as f32,
                (320.0 + 100.0 * roll_rad.sin()) as f32,
            ),
            confidence: 0.9,
            selected_phase_index: 0,
            observed_roll_rad: roll_rad,
        }
    }

    #[test]
    fn tracker_reports_valid_after_two_observations() {
        let mut tracker = EnergyMechanismTracker::new(EnergyMechanismMode::Small);

        tracker.update(EnergyMechanismMode::Small, Some(&target(0.0)));
        let snapshot = tracker
            .update(EnergyMechanismMode::Small, Some(&target(0.1)))
            .unwrap();

        assert!(snapshot.track_valid);
        assert_eq!(snapshot.direction, 1);
        assert!(snapshot.roll_rate_rad_s > 0.0);
    }

    #[test]
    fn tracker_resets_on_mode_change() {
        let mut tracker = EnergyMechanismTracker::new(EnergyMechanismMode::Small);
        tracker.update(EnergyMechanismMode::Small, Some(&target(0.0)));

        let snapshot = tracker.update(EnergyMechanismMode::Large, None);

        assert!(snapshot.is_none());
    }
}
