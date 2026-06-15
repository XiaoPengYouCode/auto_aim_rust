use std::collections::VecDeque;

#[cfg(test)]
use super::big_buff_curve_ekf::{BIG_BUFF_BASE_SPEED, big_buff_speed};
use super::big_buff_curve_ekf::{BigBuffCurveEskf, big_buff_angle_delta};
use super::detected::EnergyMechanismMode;
use super::solved::EnergyMechanismSolvedTarget;
use crate::rbt_infra::rbt_cfg::EnergyMechanismTrackerCfg;

const HISTORY_CAPACITY: usize = 48;
const TARGET_SWITCH_SEGMENT_RAD: f64 = std::f64::consts::TAU / 5.0 * 0.45;
const TARGET_REACQUIRE_ROLL_GATE_RAD: f64 = 0.12;
const TARGET_REACQUIRE_DISTANCE_GATE_M: f64 = 0.45;
const TARGET_SWITCH_CONFIRM_FRAMES: usize = 2;

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
    pub switch_deferred: bool,
    pub target_switched: bool,
    pub selected_phase_index: Option<usize>,
    pub selected_roll_offset_rad: Option<f64>,
    /// 大符曲线参数（小符时为 `None`）。
    pub curve: Option<CurveSnapshot>,
}

/// 大符曲线 EKF 的对外快照（供 aimer 做曲线预测预瞄）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveSnapshot {
    pub phase: f64,
    pub a: f64,
    pub w: f64,
    pub curve_speed_rad_s: f64,
}

impl EnergyMechanismTrackSnapshot {
    /// 预测 `dt_s` 秒后的目标中心世界坐标。大符用曲线积分，小符用常速线性外推。
    pub fn predict_target_center_world_m(self, dt_s: f64) -> na::Point3<f64> {
        let radius = self.target_center_world_m - self.rune_center_world_m;
        let angle = match (self.mode, self.curve) {
            (EnergyMechanismMode::Large, Some(curve)) => {
                // 大符：用曲线 EKF 的角度增量积分。
                let delta = big_buff_angle_delta(curve.a, curve.w, curve.phase, dt_s.max(0.0));
                self.roll_rad + self.direction as f64 * delta
            }
            _ => self.roll_rad + self.roll_rate_rad_s * dt_s.max(0.0),
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

    /// 为 aimer 生成 yaw 预瞄 horizon：对每个 `dt_s` 步，预测目标中心并返回世界坐标。
    /// 大符用曲线 EKF 推进相位，小符用常速外推。
    pub fn predict_target_horizon(&self, dt_steps: &[f64]) -> Vec<na::Point3<f64>> {
        let radius = self.target_center_world_m - self.rune_center_world_m;
        let radius_norm = radius.norm().max(1e-9);
        let mut out = Vec::with_capacity(dt_steps.len());
        for &dt_s in dt_steps {
            let angle = match (self.mode, self.curve) {
                (EnergyMechanismMode::Large, Some(curve)) => {
                    let delta = big_buff_angle_delta(curve.a, curve.w, curve.phase, dt_s.max(0.0));
                    self.roll_rad + self.direction as f64 * delta
                }
                _ => self.roll_rad + self.roll_rate_rad_s * dt_s.max(0.0),
            };
            out.push(na::Point3::new(
                self.rune_center_world_m.x,
                self.rune_center_world_m.y + radius_norm * angle.cos(),
                self.rune_center_world_m.z + radius_norm * angle.sin(),
            ));
        }
        out
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
    selected_phase_index: Option<usize>,
    selected_roll_offset_rad: Option<f64>,
    pending_switch_phase_index: Option<usize>,
    pending_switch_streak: usize,
    last_switch_deferred: bool,
    last_target_switched: bool,
    /// 大符曲线 EKF。小符时为 `None`；大符时持有并承担相位/曲线预测。
    curve_eskf: Option<BigBuffCurveEskf>,
    lost_timeout_s: f64,
    /// 大符专用丢失超时（比小符更短，激活时目标扇叶会熄灭切换）。
    big_lost_timeout_s: f64,
    /// 大符模型完全重置超时：丢失超过这个时间才清掉曲线 EKF，避免短暂遮挡就重学。
    big_model_reset_timeout_s: f64,
    /// 保存构造时的 tracker 配置，模式切换到 Large 时用它重建曲线 EKF，
    /// 避免从 Small 切 Large 时丢失 rbt_cfg.toml 里的大符参数。
    tracker_cfg: EnergyMechanismTrackerCfg,
}

impl EnergyMechanismTracker {
    /// 无参默认构造（保持向后兼容）。
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
            selected_phase_index: None,
            selected_roll_offset_rad: None,
            pending_switch_phase_index: None,
            pending_switch_streak: 0,
            last_switch_deferred: false,
            last_target_switched: false,
            curve_eskf: None,
            lost_timeout_s: 0.35,
            big_lost_timeout_s: 0.08,
            big_model_reset_timeout_s: 0.35,
            tracker_cfg: EnergyMechanismTrackerCfg::default(),
        }
    }

    /// 从配置构造（读 `lost_timeout_s` 等）。大符模式会预建曲线 EKF。
    pub fn from_tracker_cfg(mode: EnergyMechanismMode, cfg: &EnergyMechanismTrackerCfg) -> Self {
        let mut tracker = Self::new(mode);
        tracker.lost_timeout_s = cfg.lost_timeout_s.max(0.0);
        tracker.big_lost_timeout_s = cfg.big_lost_timeout_s.max(0.0);
        // model reset timeout 必须 >= lost timeout，否则会在 lost 之前就重置。
        tracker.big_model_reset_timeout_s = cfg
            .big_model_reset_timeout_s
            .max(tracker.big_lost_timeout_s);
        tracker.tracker_cfg = cfg.clone();
        if mode == EnergyMechanismMode::Large {
            tracker.curve_eskf = Some(BigBuffCurveEskf::from_tracker_cfg(cfg));
        }
        tracker
    }

    pub fn reset(&mut self, mode: EnergyMechanismMode) {
        let lost_timeout_s = self.lost_timeout_s;
        let big_lost_timeout_s = self.big_lost_timeout_s;
        let big_model_reset_timeout_s = self.big_model_reset_timeout_s;
        let tracker_cfg = self.tracker_cfg.clone();
        let mut next = Self::new(mode);
        next.lost_timeout_s = lost_timeout_s;
        next.big_lost_timeout_s = big_lost_timeout_s;
        next.big_model_reset_timeout_s = big_model_reset_timeout_s;
        next.tracker_cfg = tracker_cfg;
        if mode == EnergyMechanismMode::Large {
            // 始终用保存的真实配置重建曲线 EKF，避免 Small→Large 切换时丢 rbt_cfg.toml 参数。
            next.curve_eskf = Some(BigBuffCurveEskf::from_tracker_cfg(&next.tracker_cfg));
        }
        *self = next;
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
        self.last_switch_deferred = false;
        self.last_target_switched = false;

        // target 缺失时按 mode 判定丢失，大符长时间丢失则清掉曲线 EKF。
        if target.is_none()
            && self.mode == EnergyMechanismMode::Large
            && self
                .last_seen_tp
                .map(|last| now.duration_since(last).as_secs_f64())
                .unwrap_or(f64::INFINITY)
                > self.big_model_reset_timeout_s
            && let Some(eskf) = &mut self.curve_eskf
        {
            eskf.reset();
        }

        if let Some(target) = target {
            if self.mode == EnergyMechanismMode::Large
                && self.should_defer_target_switch(target)
                && self.pending_switch_streak < TARGET_SWITCH_CONFIRM_FRAMES
            {
                self.last_switch_deferred = true;
            } else {
                let reinitialize = self.mode == EnergyMechanismMode::Large
                    && self.should_reinitialize_for_target_switch(target);
                self.correct(target, time_s, dt_s, now, reinitialize);
            }
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
        // 大符用更短的 big_lost_timeout_s 判 lost（激活时目标切换快）。
        let lost_timeout = match self.mode {
            EnergyMechanismMode::Large => self.big_lost_timeout_s,
            EnergyMechanismMode::Small => self.lost_timeout_s,
        };
        let lost = state_age_s > lost_timeout;
        let curve = self.curve_eskf.as_ref().map(|eskf| CurveSnapshot {
            phase: eskf.phase(),
            a: eskf.a(),
            w: eskf.w(),
            curve_speed_rad_s: eskf.curve_speed(),
        });
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
            switch_deferred: self.last_switch_deferred,
            target_switched: self.last_target_switched,
            selected_phase_index: self.selected_phase_index,
            selected_roll_offset_rad: self.selected_roll_offset_rad,
            curve,
        })
    }

    fn correct(
        &mut self,
        target: &EnergyMechanismSolvedTarget,
        time_s: f64,
        dt_s: f64,
        now: std::time::Instant,
        reinitialize: bool,
    ) {
        let observed_roll = normalize_angle(target.observed_roll_rad);
        if !self.initialized || reinitialize {
            let retained_rate = if reinitialize {
                self.filtered_roll_rate_rad_s
            } else {
                0.0
            };
            let retained_direction = self.direction;
            self.initialized = true;
            self.filtered_roll_rad = observed_roll;
            self.filtered_roll_rate_rad_s = retained_rate;
            self.direction = retained_direction;
            self.history.clear();
            self.last_target_switched = reinitialize;
            if let Some(eskf) = &mut self.curve_eskf {
                eskf.reset();
                eskf.set_direction(retained_direction);
            }
        } else {
            let delta = normalize_angle(observed_roll - self.filtered_roll_rad);
            let raw_rate = delta / dt_s;
            self.filtered_roll_rad = normalize_angle(self.filtered_roll_rad + delta * 0.55);
            self.filtered_roll_rate_rad_s =
                0.80 * self.filtered_roll_rate_rad_s + 0.20 * raw_rate.clamp(-3.5, 3.5);
            if raw_rate.abs() > 0.05 {
                self.direction = if raw_rate > 0.0 { 1 } else { -1 };
            }
            // 大符：推进曲线 EKF 并尝试速度测量更新。
            if let Some(eskf) = &mut self.curve_eskf {
                eskf.set_direction(self.direction);
                eskf.predict(dt_s);
                let _ = eskf.update_with_speed(dt_s);
                eskf.refresh_smoothed_speed(dt_s);
            }
        }

        self.last_target_center_world_m = target.pose.target_center_world_m;
        self.last_rune_center_world_m = target.pose.rune_center_world_m;
        self.selected_phase_index = Some(target.selected_phase_index);
        self.selected_roll_offset_rad = target.selected_roll_offset_rad;
        self.pending_switch_phase_index = None;
        self.pending_switch_streak = 0;
        self.last_seen_tp = Some(now);
        self.history.push_back(RollSample {
            time_s,
            roll_rad: observed_roll,
        });
        while self.history.len() > HISTORY_CAPACITY {
            self.history.pop_front();
        }
        // 大符：记录 roll 样本供曲线 EKF 的 rolling-window 估速与 φ seed。
        if let Some(eskf) = &mut self.curve_eskf {
            eskf.record_roll(time_s, observed_roll);
        }
        self.fit_direction_from_history();
    }

    fn should_defer_target_switch(&mut self, target: &EnergyMechanismSolvedTarget) -> bool {
        if !self.initialized {
            return false;
        }
        let switching = target.switch_deferred || self.is_target_switch_candidate(target);
        if !switching {
            self.pending_switch_phase_index = None;
            self.pending_switch_streak = 0;
            return false;
        }

        if self.pending_switch_phase_index == Some(target.selected_phase_index) {
            self.pending_switch_streak = self.pending_switch_streak.saturating_add(1);
        } else {
            self.pending_switch_phase_index = Some(target.selected_phase_index);
            self.pending_switch_streak = 1;
        }
        true
    }

    fn should_reinitialize_for_target_switch(&self, target: &EnergyMechanismSolvedTarget) -> bool {
        self.initialized
            && (target.target_switched
                || self.pending_switch_streak >= TARGET_SWITCH_CONFIRM_FRAMES
                || self.is_target_switch_candidate(target))
    }

    fn is_target_switch_candidate(&self, target: &EnergyMechanismSolvedTarget) -> bool {
        if !self.initialized {
            return false;
        }
        if self
            .selected_phase_index
            .is_some_and(|phase| phase != target.selected_phase_index)
        {
            return true;
        }
        let roll_jump = normalize_angle(target.observed_roll_rad - self.filtered_roll_rad).abs();
        let target_jump =
            (target.pose.target_center_world_m - self.last_target_center_world_m).norm();
        let offset_jump = match (
            self.selected_roll_offset_rad,
            target.selected_roll_offset_rad,
        ) {
            (Some(previous), Some(current)) => normalize_angle(current - previous).abs(),
            _ => 0.0,
        };
        roll_jump > TARGET_REACQUIRE_ROLL_GATE_RAD
            || target_jump > TARGET_REACQUIRE_DISTANCE_GATE_M
            || offset_jump > TARGET_SWITCH_SEGMENT_RAD
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
        target_with_mode(EnergyMechanismMode::Small, roll_rad, 0)
    }

    fn target_with_mode(
        mode: EnergyMechanismMode,
        roll_rad: f64,
        selected_phase_index: usize,
    ) -> EnergyMechanismSolvedTarget {
        EnergyMechanismSolvedTarget {
            mode,
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
            image_r_center_corrected: false,
            confidence: 0.9,
            selected_phase_index,
            observed_roll_rad: roll_rad,
            switch_deferred: false,
            target_switched: false,
            selected_roll_offset_rad: Some(normalize_angle(
                roll_rad - selected_phase_index as f64 * std::f64::consts::TAU / 5.0,
            )),
        }
    }

    fn default_tracker_cfg() -> EnergyMechanismTrackerCfg {
        EnergyMechanismTrackerCfg::default()
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
        // 小符没有曲线 EKF 快照。
        assert!(snapshot.curve.is_none());
    }

    #[test]
    fn tracker_resets_on_mode_change() {
        let mut tracker = EnergyMechanismTracker::new(EnergyMechanismMode::Small);
        tracker.update(EnergyMechanismMode::Small, Some(&target(0.0)));

        let snapshot = tracker.update(EnergyMechanismMode::Large, None);

        assert!(snapshot.is_none());
    }

    #[test]
    fn large_tracker_defers_then_rebinds_confirmed_target_switch() {
        let mut tracker = EnergyMechanismTracker::from_tracker_cfg(
            EnergyMechanismMode::Large,
            &default_tracker_cfg(),
        );
        tracker.update(
            EnergyMechanismMode::Large,
            Some(&target_with_mode(EnergyMechanismMode::Large, 0.0, 0)),
        );
        tracker.update(
            EnergyMechanismMode::Large,
            Some(&target_with_mode(EnergyMechanismMode::Large, 0.05, 0)),
        );

        let first_switch = tracker
            .update(
                EnergyMechanismMode::Large,
                Some(&target_with_mode(
                    EnergyMechanismMode::Large,
                    std::f64::consts::TAU / 5.0,
                    1,
                )),
            )
            .unwrap();
        assert!(first_switch.switch_deferred);
        assert!(!first_switch.target_switched);
        assert_eq!(first_switch.selected_phase_index, Some(0));

        let confirmed_switch = tracker
            .update(
                EnergyMechanismMode::Large,
                Some(&target_with_mode(
                    EnergyMechanismMode::Large,
                    std::f64::consts::TAU / 5.0,
                    1,
                )),
            )
            .unwrap();

        assert!(!confirmed_switch.switch_deferred);
        assert!(confirmed_switch.target_switched);
        assert_eq!(confirmed_switch.selected_phase_index, Some(1));
    }

    #[test]
    fn large_tracker_exposes_curve_snapshot_after_observations() {
        let mut tracker = EnergyMechanismTracker::from_tracker_cfg(
            EnergyMechanismMode::Large,
            &default_tracker_cfg(),
        );
        for i in 0..6 {
            tracker.update(
                EnergyMechanismMode::Large,
                Some(&target_with_mode(
                    EnergyMechanismMode::Large,
                    i as f64 * 0.05,
                    0,
                )),
            );
        }
        let snapshot = tracker.snapshot(std::time::Instant::now()).unwrap();
        let curve = snapshot
            .curve
            .expect("curve snapshot present for large mode");
        assert!(curve.curve_speed_rad_s > 0.0);
        assert!(curve.curve_speed_rad_s < BIG_BUFF_BASE_SPEED * 2.0);
    }

    #[test]
    fn predict_horizon_returns_one_point_per_step() {
        let tracker = EnergyMechanismTracker::new(EnergyMechanismMode::Small);
        // 直接构造一个快照测试 horizon。
        let snapshot = EnergyMechanismTrackSnapshot {
            mode: EnergyMechanismMode::Small,
            target_center_world_m: na::Point3::new(1.0, 1.0, 0.0),
            rune_center_world_m: na::Point3::origin(),
            roll_rad: 0.0,
            roll_rate_rad_s: 1.0,
            direction: 1,
            history_size: 4,
            lost: false,
            track_valid: true,
            state_age_s: 0.0,
            switch_deferred: false,
            target_switched: false,
            selected_phase_index: Some(0),
            selected_roll_offset_rad: Some(0.0),
            curve: None,
        };
        let horizon = snapshot.predict_target_horizon(&[0.0, 0.1, 0.2]);
        assert_eq!(horizon.len(), 3);
        // 半径 = target_center - rune_center = √2，angle = roll_rate·dt = 0.1。
        let radius = 2_f64.sqrt();
        assert!((horizon[1].y - radius * 0.1_f64.cos()).abs() < 1e-6);
        let _ = tracker;
    }

    #[test]
    fn from_tracker_cfg_reads_lost_timeout() {
        let mut cfg = default_tracker_cfg();
        cfg.lost_timeout_s = 0.5;
        let tracker = EnergyMechanismTracker::from_tracker_cfg(EnergyMechanismMode::Small, &cfg);
        assert!((tracker.lost_timeout_s - 0.5).abs() < 1e-9);
    }

    #[test]
    fn small_to_large_switch_preserves_tracker_cfg() {
        // 生产路径：先以 Small + 自定义配置构造，再切 Large。
        // 切换后必须用真实配置（而非 default）重建曲线 EKF。
        let mut cfg = default_tracker_cfg();
        cfg.big_phase_process_noise = 0.123;
        cfg.lost_timeout_s = 0.42;
        let mut tracker =
            EnergyMechanismTracker::from_tracker_cfg(EnergyMechanismMode::Small, &cfg);
        // Small 时无曲线 EKF。
        assert!(tracker.curve_eskf.is_none());

        // 切 Large：reset 后应有用真实配置重建的曲线 EKF，且 lost_timeout 保留。
        tracker.reset(EnergyMechanismMode::Large);
        assert!(tracker.curve_eskf.is_some());
        assert!((tracker.lost_timeout_s - 0.42).abs() < 1e-9);
        // tracker_cfg 应保留自定义值。
        assert!((tracker.tracker_cfg.big_phase_process_noise - 0.123).abs() < 1e-9);
    }

    #[test]
    fn large_to_small_then_back_to_large_keeps_cfg() {
        let mut cfg = default_tracker_cfg();
        cfg.big_a_process_noise = 0.007;
        let mut tracker =
            EnergyMechanismTracker::from_tracker_cfg(EnergyMechanismMode::Large, &cfg);
        tracker.reset(EnergyMechanismMode::Small);
        tracker.reset(EnergyMechanismMode::Large);
        assert!(tracker.curve_eskf.is_some());
        assert!((tracker.tracker_cfg.big_a_process_noise - 0.007).abs() < 1e-9);
    }

    #[test]
    fn from_tracker_cfg_reads_big_timeout_fields() {
        let mut cfg = default_tracker_cfg();
        cfg.big_lost_timeout_s = 0.05;
        cfg.big_model_reset_timeout_s = 0.4;
        let tracker = EnergyMechanismTracker::from_tracker_cfg(EnergyMechanismMode::Large, &cfg);
        assert!((tracker.big_lost_timeout_s - 0.05).abs() < 1e-9);
        assert!((tracker.big_model_reset_timeout_s - 0.4).abs() < 1e-9);
    }

    #[test]
    fn big_model_reset_timeout_clamped_above_big_lost_timeout() {
        // model reset timeout 必须 >= big_lost_timeout，配置里写更小值时自动抬升。
        let mut cfg = default_tracker_cfg();
        cfg.big_lost_timeout_s = 0.3;
        cfg.big_model_reset_timeout_s = 0.1; // 比 big_lost 还小
        let tracker = EnergyMechanismTracker::from_tracker_cfg(EnergyMechanismMode::Large, &cfg);
        assert!(tracker.big_model_reset_timeout_s >= tracker.big_lost_timeout_s);
    }

    #[test]
    fn predict_target_center_uses_curve_for_large_mode() {
        // 大符快照用曲线 EKF 的 angle_delta，而非 roll_rate 线性外推。
        let snapshot = EnergyMechanismTrackSnapshot {
            mode: EnergyMechanismMode::Large,
            target_center_world_m: na::Point3::new(4.0, 1.0, 0.0),
            rune_center_world_m: na::Point3::new(4.0, 0.0, 0.0),
            roll_rad: 0.0,
            roll_rate_rad_s: 0.0, // 故意设 0，确保大符不走常速分支
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
        let predicted = snapshot.predict_target_center_world_m(0.1);
        // 曲线分支：dt=0 时 angle=0 → y=cos(0)=1.0；dt=0.1 时 angle≈0.118 → y 略小于 1。
        // 关键是大符用了曲线 delta（非 roll_rate=0 的零外推），y 应严格小于 1。
        assert!(
            predicted.y < 1.0,
            "y={} should be < 1.0 (curve delta applied)",
            predicted.y
        );
        assert!(
            predicted.y > 0.98,
            "y={} should be > 0.98 (curve delta is small)",
            predicted.y
        );
    }

    #[test]
    fn curve_speed_via_big_buff_speed_helper() {
        // 确认 big_buff_speed 导出可用且与曲线快照一致。
        let speed = big_buff_speed(0.9125, std::f64::consts::FRAC_PI_2);
        assert!((speed - BIG_BUFF_BASE_SPEED).abs() < 1e-9);
    }
}
