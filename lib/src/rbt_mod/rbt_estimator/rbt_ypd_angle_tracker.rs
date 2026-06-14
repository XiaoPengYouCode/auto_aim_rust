use std::collections::VecDeque;

use crate::rbt_base::rbt_algorithm::rbt_ekf::ExtendedKalmanFilter;
use crate::rbt_infra::rbt_cfg::EstimatorCfg;

const STATE_DIM: usize = 11;
const PRIMARY_RADIUS: usize = 8;
const DELTA_RADIUS: usize = 9;
const HEIGHT_DIFF: usize = 10;
const MIN_RADIUS_MM: f64 = 50.0;
const MAX_RADIUS_MM: f64 = 500.0;
const OUTPOST_RADIUS_MM: f64 = 276.5;
const OUTPOST_MAX_HEIGHT_OFFSET_MM: f64 = 600.0;
const OUTPOST_RADIUS_PRIOR_SIGMA_MM: f64 = 18.0;
const OUTPOST_PLANE_TO_RADIAL_YAW_OFFSET_RAD: f64 = 153.0 * std::f64::consts::PI / 180.0;
const OUTPOST_HEIGHT_STEP_MM: f64 = 105.0;
const OUTPOST_HEIGHT_PHASE_SIGMA_MM: f64 = 35.0;
const OUTPOST_HEIGHT_PHASE_LOCK_MARGIN: f64 = 9.0;
const OUTPOST_HEIGHT_PHASE_CENTER_CAPACITY: usize = 96;
const OUTPOST_HEIGHT_PHASE_MIN_CENTER_SAMPLES: usize = 36;
const OUTPOST_HEIGHT_PHASE_MIN_SAMPLES_PER_ID: usize = 6;
const OUTPOST_PRIOR_GATE_MIN_UPDATES: usize = 8;
const OUTPOST_REINIT_AFTER_REJECTED_UPDATES: usize = 4;
const OUTPOST_PRIOR_NIS_REJECT_THRESHOLD: f64 = 16.0;
const OUTPOST_PRIMARY_ONLY_IMAGE_Y_THRESHOLD: f64 = 600.0;
const OUTPOST_LOCKED_HEIGHT_VARIANCE_MM2: f64 = 1.0;
const OUTPOST_HEIGHT_RANK_CANDIDATES: [[i8; 3]; 6] = [
    [1, 0, -1],
    [0, -1, 1],
    [-1, 1, 0],
    [1, -1, 0],
    [-1, 0, 1],
    [0, 1, -1],
];
const MOTION_HISTORY_CAPACITY: usize = 128;
const NIS_WINDOW_SIZE: usize = 100;
const M2_TO_MM2: f64 = 1_000_000.0;

#[derive(Debug, Clone, Copy)]
pub struct YpdObservation {
    pub position_mm: na::Point3<f64>,
    pub yaw_rad: f64,
    pub image_center: na::Point2<f64>,
    pub radius_hint_mm: f64,
}

#[derive(Debug, Clone)]
pub struct YpdTrackerSnapshot {
    pub state11d: [f64; STATE_DIM],
    pub state9: [f64; 9],
    pub tracked_id: usize,
    pub armor_num: usize,
    pub tracked_armor_xyza: [f64; 4],
    pub predicted_armors_xyza: Vec<[f64; 4]>,
    pub last_nis: f64,
    pub converged: bool,
    pub diverged: bool,
    pub recent_nis_failures: usize,
    pub motion_translation_burst_metric: f64,
    pub motion_translation_drift_metric: f64,
    pub motion_yaw_accel_metric: f64,
}

#[derive(Debug, Clone, Copy)]
struct MotionSample {
    t_s: f64,
    center_x: f64,
    center_y: f64,
    yaw_rate: f64,
}

#[derive(Debug, Clone, Copy)]
struct GeometryRecoverySample {
    xy_residual_mm: f64,
    z_residual_mm: f64,
}

struct ArmorAssignmentSearch<'a> {
    tracker: &'a YpdAngleTracker,
    observations: &'a [YpdObservation],
    used: Vec<bool>,
    current: Vec<Option<usize>>,
    best_cost: f64,
    best: Vec<Option<usize>>,
}

impl<'a> ArmorAssignmentSearch<'a> {
    fn new(tracker: &'a YpdAngleTracker, observations: &'a [YpdObservation]) -> Self {
        let count = observations.len().min(tracker.armor_num);
        Self {
            tracker,
            observations: &observations[..count],
            used: vec![false; tracker.armor_num],
            current: vec![None; count],
            best_cost: f64::INFINITY,
            best: vec![None; count],
        }
    }

    fn run(mut self) -> Vec<Option<usize>> {
        self.visit(0, 0.0);
        self.best
    }

    fn visit(&mut self, index: usize, total: f64) {
        if total >= self.best_cost {
            return;
        }
        if index == self.observations.len() {
            self.best_cost = total;
            self.best.copy_from_slice(&self.current);
            return;
        }

        for id in 0..self.tracker.armor_num {
            if self.used[id] {
                continue;
            }
            self.used[id] = true;
            self.current[index] = Some(id);
            let next_total = total + self.tracker.match_cost(&self.observations[index], id);
            self.visit(index + 1, next_total);
            self.current[index] = None;
            self.used[id] = false;
        }
    }
}

#[derive(Debug, Clone)]
pub struct YpdAngleTracker {
    initialized: bool,
    is_outpost: bool,
    armor_num: usize,
    tracked_id: usize,
    update_count: usize,
    tracker_time_s: f64,
    is_converged: bool,
    ekf: ExtendedKalmanFilter,
    last_nis: f64,
    recent_nis_failures: VecDeque<bool>,
    last_batch_match_ids: Vec<isize>,
    motion_history: VecDeque<MotionSample>,
    geometry_recovery_window_remaining: usize,
    geometry_recovery_cooldown_remaining: usize,
    geometry_mismatch_streak: usize,
    consecutive_rejected_updates: usize,
    outpost_height_phase_scores: [f64; 6],
    outpost_height_phase_observations: usize,
    outpost_height_phase: Option<usize>,
    outpost_height_phase_id_samples: VecDeque<usize>,
    outpost_height_phase_id_counts: [usize; 3],
    outpost_height_phase_center_samples: [VecDeque<f64>; 6],
}

impl YpdAngleTracker {
    /// 当前名义状态（11 维 CV 模型）。
    pub fn x(&self) -> &na::DVector<f64> {
        &self.ekf.x
    }

    /// 当前协方差矩阵。
    pub fn p(&self) -> &na::DMatrix<f64> {
        &self.ekf.p
    }

    /// 可变借用名义状态。
    fn x_mut(&mut self) -> &mut na::DVector<f64> {
        &mut self.ekf.x
    }

    /// 可变借用协方差矩阵。
    fn p_mut(&mut self) -> &mut na::DMatrix<f64> {
        &mut self.ekf.p
    }

    pub fn new() -> Self {
        let mut tracker = Self {
            initialized: false,
            is_outpost: false,
            armor_num: 4,
            tracked_id: 0,
            update_count: 0,
            tracker_time_s: 0.0,
            is_converged: false,
            ekf: ExtendedKalmanFilter::with_initial(
                na::DVector::zeros(STATE_DIM),
                na::DMatrix::identity(STATE_DIM, STATE_DIM),
            ),
            last_nis: 0.0,
            recent_nis_failures: VecDeque::from([false]),
            last_batch_match_ids: Vec::new(),
            motion_history: VecDeque::new(),
            geometry_recovery_window_remaining: 0,
            geometry_recovery_cooldown_remaining: 0,
            geometry_mismatch_streak: 0,
            consecutive_rejected_updates: 0,
            outpost_height_phase_scores: [0.0; 6],
            outpost_height_phase_observations: 0,
            outpost_height_phase: None,
            outpost_height_phase_id_samples: VecDeque::new(),
            outpost_height_phase_id_counts: [0; 3],
            outpost_height_phase_center_samples: std::array::from_fn(|_| VecDeque::new()),
        };
        tracker.reset();
        tracker
    }

    pub fn reset(&mut self) {
        self.initialized = false;
        self.is_outpost = false;
        self.armor_num = 4;
        self.tracked_id = 0;
        self.update_count = 0;
        self.tracker_time_s = 0.0;
        self.is_converged = false;
        self.ekf.init(
            na::DVector::zeros(STATE_DIM),
            na::DMatrix::identity(STATE_DIM, STATE_DIM),
        );
        self.last_nis = 0.0;
        self.recent_nis_failures.clear();
        self.recent_nis_failures.push_back(false);
        self.last_batch_match_ids.clear();
        self.motion_history.clear();
        self.geometry_recovery_window_remaining = 0;
        self.geometry_recovery_cooldown_remaining = 0;
        self.geometry_mismatch_streak = 0;
        self.consecutive_rejected_updates = 0;
        self.reset_outpost_height_phase();
    }

    pub fn init(&mut self, observation: &YpdObservation, armor_num: usize) {
        self.reset();
        self.armor_num = armor_num.clamp(3, 4);
        self.is_outpost = self.armor_num == 3;

        let radius = if self.is_outpost {
            OUTPOST_RADIUS_MM
        } else if observation.radius_hint_mm.is_finite()
            && observation.radius_hint_mm > MIN_RADIUS_MM
        {
            observation
                .radius_hint_mm
                .clamp(MIN_RADIUS_MM, MAX_RADIUS_MM)
        } else {
            200.0
        };

        let yaw = radial_yaw_from_observed_yaw(observation.yaw_rad, self.armor_num);
        let sign = radial_sign(self.armor_num);
        let x0 = {
            let mut v = na::DVector::zeros(STATE_DIM);
            v[0] = observation.position_mm.x - sign * radius * yaw.cos();
            v[2] = observation.position_mm.y - sign * radius * yaw.sin();
            v[4] = observation.position_mm.z;
            v[6] = yaw;
            v[8] = radius;
            v
        };
        let p0 = if self.is_outpost {
            diagonal_matrix([
                1_000.0, 64_000.0, 1_000.0, 64_000.0, 1_000.0, 81_000.0, 0.4, 100.0, 100.0,
                90_000.0, 90_000.0,
            ])
        } else {
            diagonal_matrix([
                1_000.0, 64_000.0, 1_000.0, 64_000.0, 1_000.0, 64_000.0, 0.4, 100.0, 10_000.0,
                10_000.0, 10_000.0,
            ])
        };
        self.ekf.init(x0, p0);

        self.initialized = true;
        self.tracked_id = self.select_best_armor_id(observation);
        self.append_motion_sample();
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub fn predict(&mut self, dt_s: f64) {
        if !self.initialized {
            return;
        }

        let dt = if dt_s.is_finite() && dt_s > 0.0 {
            dt_s.clamp(0.001, 0.05)
        } else {
            0.006
        };
        self.tracker_time_s += dt;

        if self.is_outpost && self.converged() && self.x()[7].abs() > 2.0 {
            self.x_mut()[7] = self.x()[7].signum() * 2.51;
        }

        let mut f = na::DMatrix::<f64>::identity(STATE_DIM, STATE_DIM);
        f[(0, 1)] = dt;
        f[(2, 3)] = dt;
        f[(4, 5)] = dt;
        f[(6, 7)] = dt;
        let q = self.process_noise(dt);

        self.ekf.predict(&f, &q);
        self.x_mut()[6] = normalize_angle(self.x()[6]);
        self.clamp_geometry();
    }

    pub fn note_observation_jump(&mut self, jumped: bool, cfg: &EstimatorCfg) {
        if jumped && self.initialized && self.armor_num == 4 {
            self.geometry_recovery_window_remaining =
                cfg.ypd_geometry_recovery_window_frames.max(1);
        } else if !jumped && self.geometry_recovery_window_remaining == 0 {
            self.geometry_mismatch_streak = 0;
        }
    }

    pub fn update_batch(
        &mut self,
        observations: &[YpdObservation],
        preferred_index: Option<usize>,
        cfg: &EstimatorCfg,
    ) {
        self.last_batch_match_ids.clear();
        if !self.initialized || observations.is_empty() {
            return;
        }

        let limit = observations.len().min(self.armor_num);
        self.last_batch_match_ids = vec![-1; observations.len()];
        let assignment = self.assign_armor_ids(&observations[..limit]);
        let tracked_index = preferred_index.filter(|index| *index < limit).unwrap_or(0);
        let mut primary_match = None;
        let mut recovery_samples = Vec::new();
        let use_primary_only = self.is_outpost
            && limit > 1
            && preferred_index.is_some_and(|index| {
                index < limit
                    && observations[index].image_center.y < OUTPOST_PRIMARY_ONLY_IMAGE_Y_THRESHOLD
            });
        let mut accepted_count = 0;

        for index in 0..limit {
            if use_primary_only && index != tracked_index {
                continue;
            }
            let matched_id = assignment[index]
                .unwrap_or_else(|| self.select_best_armor_id(&observations[index]));
            if self.geometry_recovery_window_remaining > 0 && self.armor_num == 4 {
                recovery_samples
                    .push(self.geometry_recovery_sample(&observations[index], matched_id));
            }
            if self.correct_with_observation(&observations[index], matched_id) {
                self.last_batch_match_ids[index] = matched_id as isize;
                self.update_outpost_height_phase(&observations[index], matched_id);
                accepted_count += 1;
                if index == tracked_index {
                    primary_match = Some(matched_id);
                }
            }
        }

        if self.is_outpost
            && accepted_count == 0
            && self.consecutive_rejected_updates >= OUTPOST_REINIT_AFTER_REJECTED_UPDATES
        {
            self.init(&observations[tracked_index], self.armor_num);
            self.tracked_id = self.select_best_armor_id(&observations[tracked_index]);
            self.last_batch_match_ids = vec![-1; observations.len()];
            self.last_batch_match_ids[tracked_index] = self.tracked_id as isize;
            return;
        }

        self.apply_locked_outpost_height_offsets();
        if let Some(matched_id) = primary_match {
            self.tracked_id = matched_id;
        }
        self.update_geometry_recovery(&recovery_samples, cfg);
        self.clamp_geometry();
        self.append_motion_sample();
    }

    pub fn snapshot(&self) -> Option<YpdTrackerSnapshot> {
        if !self.initialized {
            return None;
        }

        let tracked_armor_xyza = self.predicted_armor_state(self.tracked_id);
        let mut state11d = [0.0; STATE_DIM];
        state11d.copy_from_slice(self.x().as_slice());

        let mut state9 = [0.0; 9];
        state9[0] = self.x()[0];
        state9[1] = self.x()[1];
        state9[2] = self.x()[2];
        state9[3] = self.x()[3];
        state9[4] = tracked_armor_xyza[2];
        state9[5] = self.x()[5];
        state9[6] = tracked_armor_xyza[3];
        state9[7] = self.x()[7];
        state9[8] = self.armor_radius(self.tracked_id);

        Some(YpdTrackerSnapshot {
            state11d,
            state9,
            tracked_id: self.tracked_id,
            armor_num: self.armor_num,
            tracked_armor_xyza,
            predicted_armors_xyza: (0..self.armor_num)
                .map(|id| self.predicted_armor_state(id))
                .collect(),
            last_nis: self.last_nis,
            converged: self.is_converged,
            diverged: self.diverged(),
            recent_nis_failures: self.recent_nis_failure_count(),
            motion_translation_burst_metric: self.center_quadratic_accel(12),
            motion_translation_drift_metric: self.center_quadratic_accel(48),
            motion_yaw_accel_metric: self.yaw_rate_linear_accel(24),
        })
    }

    pub fn diverged(&self) -> bool {
        if !self.initialized {
            return false;
        }
        let primary_ok =
            self.x()[PRIMARY_RADIUS] > MIN_RADIUS_MM && self.x()[PRIMARY_RADIUS] < MAX_RADIUS_MM;
        if !primary_ok {
            return true;
        }
        if self.armor_num == 4 {
            let secondary = self.x()[PRIMARY_RADIUS] + self.x()[DELTA_RADIUS];
            !(secondary > MIN_RADIUS_MM && secondary < MAX_RADIUS_MM)
        } else {
            !self.x()[DELTA_RADIUS].is_finite()
                || !self.x()[HEIGHT_DIFF].is_finite()
                || self.x()[DELTA_RADIUS].abs() > OUTPOST_MAX_HEIGHT_OFFSET_MM
                || self.x()[HEIGHT_DIFF].abs() > OUTPOST_MAX_HEIGHT_OFFSET_MM
        }
    }

    pub fn bad_convergence(&self) -> bool {
        self.initialized
            && self.recent_nis_failures.len() >= NIS_WINDOW_SIZE
            && self.recent_nis_failure_count() * 5 >= NIS_WINDOW_SIZE * 2
    }

    pub fn last_batch_match_ids(&self) -> &[isize] {
        &self.last_batch_match_ids
    }

    fn converged(&mut self) -> bool {
        let min_updates = if self.is_outpost { 10 } else { 3 };
        if self.update_count > min_updates && !self.diverged() {
            self.is_converged = true;
        }
        self.is_converged
    }

    fn process_noise(&self, dt: f64) -> na::DMatrix<f64> {
        let pos_noise = if self.is_outpost { 10_000.0 } else { 100_000.0 };
        let yaw_noise = if self.is_outpost { 0.1 } else { 400.0 };
        let a = dt.powi(4) / 4.0;
        let b = dt.powi(3) / 2.0;
        let c = dt * dt;

        let mut q = na::DMatrix::<f64>::zeros(STATE_DIM, STATE_DIM);
        for (pos, vel, noise) in [
            (0, 1, pos_noise),
            (2, 3, pos_noise),
            (4, 5, pos_noise),
            (6, 7, yaw_noise),
        ] {
            q[(pos, pos)] = a * noise;
            q[(pos, vel)] = b * noise;
            q[(vel, pos)] = b * noise;
            q[(vel, vel)] = c * noise;
        }
        q
    }

    fn predicted_armor_position(&self, state: &na::DVector<f64>, id: usize) -> na::Point3<f64> {
        let angle =
            normalize_angle(state[6] + id as f64 * std::f64::consts::TAU / self.armor_num as f64);
        let radius = radius_from_state(state, self.armor_num, id);
        let sign = radial_sign(self.armor_num);
        na::Point3::new(
            state[0] + sign * radius * angle.cos(),
            state[2] + sign * radius * angle.sin(),
            state[4] + self.height_offset_for_id(state, id),
        )
    }

    fn predicted_armor_state(&self, id: usize) -> [f64; 4] {
        let clamped_id = id.min(self.armor_num.saturating_sub(1));
        let angle = normalize_angle(
            self.x()[6] + clamped_id as f64 * std::f64::consts::TAU / self.armor_num as f64,
        );
        let position = self.predicted_armor_position(self.x(), clamped_id);
        [
            position.x,
            position.y,
            position.z,
            observed_yaw_from_radial_yaw(angle, self.armor_num),
        ]
    }

    fn predicted_measurement(&self, state: &na::DVector<f64>, id: usize) -> na::DVector<f64> {
        let position = self.predicted_armor_position(state, id);
        let ypd = xyz_to_ypd(position);
        let angle =
            normalize_angle(state[6] + id as f64 * std::f64::consts::TAU / self.armor_num as f64);
        na::DVector::from_vec(vec![
            ypd.x,
            ypd.y,
            ypd.z,
            observed_yaw_from_radial_yaw(angle, self.armor_num),
        ])
    }

    fn measurement_jacobian(&self, state: &na::DVector<f64>, id: usize) -> na::DMatrix<f64> {
        let angle =
            normalize_angle(state[6] + id as f64 * std::f64::consts::TAU / self.armor_num as f64);
        let use_secondary_radius = self.armor_num == 4 && (id == 1 || id == 3);
        let sign = radial_sign(self.armor_num);
        let radius = if use_secondary_radius {
            state[PRIMARY_RADIUS] + state[DELTA_RADIUS]
        } else {
            state[PRIMARY_RADIUS]
        };

        let mut h_xyza = na::DMatrix::<f64>::zeros(4, STATE_DIM);
        h_xyza[(0, 0)] = 1.0;
        h_xyza[(0, 6)] = -sign * radius * angle.sin();
        h_xyza[(0, PRIMARY_RADIUS)] = sign * angle.cos();
        h_xyza[(0, DELTA_RADIUS)] = if use_secondary_radius {
            sign * angle.cos()
        } else {
            0.0
        };

        h_xyza[(1, 2)] = 1.0;
        h_xyza[(1, 6)] = sign * radius * angle.cos();
        h_xyza[(1, PRIMARY_RADIUS)] = sign * angle.sin();
        h_xyza[(1, DELTA_RADIUS)] = if use_secondary_radius {
            sign * angle.sin()
        } else {
            0.0
        };

        h_xyza[(2, 4)] = 1.0;
        let outpost_height_locked = self.outpost_height_phase_locked();
        h_xyza[(2, DELTA_RADIUS)] = if self.armor_num == 3 && !outpost_height_locked && id == 1 {
            1.0
        } else {
            0.0
        };
        h_xyza[(2, HEIGHT_DIFF)] = if (self.armor_num == 4 && (id == 1 || id == 3))
            || (self.armor_num == 3 && !outpost_height_locked && id == 2)
        {
            1.0
        } else {
            0.0
        };
        h_xyza[(3, 6)] = 1.0;

        let h_ypd = xyz_to_ypd_jacobian(self.predicted_armor_position(state, id));
        let mut h_ypda = na::DMatrix::<f64>::zeros(4, 4);
        for row in 0..3 {
            for col in 0..3 {
                h_ypda[(row, col)] = h_ypd[(row, col)];
            }
        }
        h_ypda[(3, 3)] = 1.0;
        h_ypda * h_xyza
    }

    fn measurement_noise(&self, observation: &YpdObservation) -> na::DMatrix<f64> {
        let ypd = xyz_to_ypd(observation.position_mm);
        let center_yaw = observation.position_mm.y.atan2(observation.position_mm.x);
        let delta_angle = normalize_angle(observation.yaw_rad - center_yaw).abs();
        let distance_sigma_mm = (ypd.z.abs() * 0.03).clamp(10.0, 250.0);

        let mut r = na::DMatrix::<f64>::zeros(4, 4);
        r[(0, 0)] = 4e-3;
        r[(1, 1)] = 4e-3;
        r[(2, 2)] = distance_sigma_mm * distance_sigma_mm;
        r[(3, 3)] = (delta_angle.abs() + 1.0).ln() / 20.0 + 9e-2;
        if self.is_outpost {
            let mut distance_scale = 1.0;
            if observation.image_center.y < 600.0 {
                distance_scale *= 3.0;
            }
            if observation.image_center.y < 450.0 {
                distance_scale *= 2.0;
            }
            let obs_pitch_deg = ypd.y.to_degrees();
            if obs_pitch_deg < 10.0 {
                distance_scale *= 2.0;
            }
            if obs_pitch_deg < 5.0 {
                distance_scale *= 1.5;
            }
            r[(2, 2)] *= distance_scale;
        }
        r
    }

    fn correct_with_observation(&mut self, observation: &YpdObservation, id: usize) -> bool {
        let matched_id = id.min(self.armor_num.saturating_sub(1));
        let ypd = xyz_to_ypd(observation.position_mm);
        let z = na::DVector::from_vec(vec![ypd.x, ypd.y, ypd.z, observation.yaw_rad]);
        let h = self.measurement_jacobian(self.x(), matched_id);
        let r = self.measurement_noise(observation);
        let predicted = self.predicted_measurement(self.x(), matched_id);
        let residual_fn = |a: &na::DVector<f64>, b: &na::DVector<f64>| {
            let mut diff = a - b;
            diff[0] = normalize_angle(diff[0]);
            diff[1] = normalize_angle(diff[1]);
            diff[3] = normalize_angle(diff[3]);
            diff
        };
        let prior_nis = self.ekf.nis(&z, &h, &r, &predicted, residual_fn);

        if self.is_outpost
            && self.update_count >= OUTPOST_PRIOR_GATE_MIN_UPDATES
            && prior_nis.is_finite()
            && prior_nis > OUTPOST_PRIOR_NIS_REJECT_THRESHOLD
        {
            self.consecutive_rejected_updates = self.consecutive_rejected_updates.saturating_add(1);
            self.record_nis(prior_nis);
            return false;
        }

        let (accepted, nis) = self.ekf.update(&z, &h, &r, &predicted, residual_fn);
        if !accepted {
            self.record_nis(f64::INFINITY);
            return false;
        }
        self.x_mut()[6] = normalize_angle(self.x()[6]);
        self.apply_outpost_radius_prior();
        self.clamp_geometry();

        self.record_nis(nis);
        self.update_count += 1;
        self.consecutive_rejected_updates = 0;
        true
    }

    fn geometry_recovery_sample(
        &self,
        observation: &YpdObservation,
        id: usize,
    ) -> GeometryRecoverySample {
        let predicted = self.predicted_armor_position(self.x(), id.min(self.armor_num - 1));
        let residual = observation.position_mm - predicted;
        GeometryRecoverySample {
            xy_residual_mm: residual.x.hypot(residual.y),
            z_residual_mm: residual.z.abs(),
        }
    }

    fn update_geometry_recovery(&mut self, samples: &[GeometryRecoverySample], cfg: &EstimatorCfg) {
        if self.geometry_recovery_cooldown_remaining > 0 {
            self.geometry_recovery_cooldown_remaining -= 1;
        }
        if self.geometry_recovery_window_remaining == 0 || self.armor_num != 4 {
            return;
        }
        self.geometry_recovery_window_remaining -= 1;
        if samples.len() < cfg.ypd_geometry_recovery_min_matched_count.max(1) {
            self.geometry_mismatch_streak = 0;
            return;
        }

        let mean_xy = samples
            .iter()
            .map(|sample| sample.xy_residual_mm)
            .sum::<f64>()
            / samples.len() as f64;
        let mean_z = samples
            .iter()
            .map(|sample| sample.z_residual_mm)
            .sum::<f64>()
            / samples.len() as f64;
        let sigma_dr = self.p()[(DELTA_RADIUS, DELTA_RADIUS)].max(1e-9).sqrt();
        let sigma_h = self.p()[(HEIGHT_DIFF, HEIGHT_DIFF)].max(1e-9).sqrt();
        let xy_over_sigma_dr = mean_xy / sigma_dr;
        let z_over_sigma_h = mean_z / sigma_h;
        let mismatch = z_over_sigma_h.is_finite()
            && xy_over_sigma_dr.is_finite()
            && ((z_over_sigma_h >= cfg.ypd_geometry_recovery_z_sigma_threshold
                && xy_over_sigma_dr >= cfg.ypd_geometry_recovery_xy_sigma_threshold)
                || z_over_sigma_h >= cfg.ypd_geometry_recovery_z_sigma_threshold + 1.0);

        if mismatch {
            self.geometry_mismatch_streak = self.geometry_mismatch_streak.saturating_add(1);
        } else {
            self.geometry_mismatch_streak = 0;
        }

        if self.geometry_recovery_cooldown_remaining == 0
            && self.geometry_mismatch_streak
                >= cfg.ypd_geometry_recovery_mismatch_required_streak.max(1)
        {
            self.inflate_geometry_covariance(cfg);
            self.geometry_mismatch_streak = 0;
            self.geometry_recovery_cooldown_remaining =
                cfg.ypd_geometry_recovery_cooldown_frames.max(1);
            self.geometry_recovery_window_remaining = 0;
        }
    }

    fn inflate_geometry_covariance(&mut self, cfg: &EstimatorCfg) {
        let scale = cfg
            .ypd_geometry_recovery_cov_inflation_scale
            .clamp(1.0, 1_000.0);
        let min_dr_var_mm2 = cfg.ypd_geometry_recovery_min_dr_variance.max(0.0) * M2_TO_MM2;
        let min_h_var_mm2 = cfg.ypd_geometry_recovery_min_h_variance.max(0.0) * M2_TO_MM2;

        let p = self.p_mut();
        for index in [DELTA_RADIUS, HEIGHT_DIFF] {
            for col in 0..STATE_DIM {
                p[(index, col)] *= scale.sqrt();
                p[(col, index)] = p[(index, col)];
            }
        }
        p[(DELTA_RADIUS, DELTA_RADIUS)] =
            (p[(DELTA_RADIUS, DELTA_RADIUS)] * scale).max(min_dr_var_mm2);
        p[(HEIGHT_DIFF, HEIGHT_DIFF)] = (p[(HEIGHT_DIFF, HEIGHT_DIFF)] * scale).max(min_h_var_mm2);
        let sym = symmetrize_dynamic(p);
        *p = sym;
    }

    fn assign_armor_ids(&self, observations: &[YpdObservation]) -> Vec<Option<usize>> {
        ArmorAssignmentSearch::new(self, observations).run()
    }

    fn select_best_armor_id(&self, observation: &YpdObservation) -> usize {
        (0..self.armor_num)
            .min_by(|lhs, rhs| {
                self.match_cost(observation, *lhs)
                    .total_cmp(&self.match_cost(observation, *rhs))
            })
            .unwrap_or(0)
    }

    fn match_cost(&self, observation: &YpdObservation, id: usize) -> f64 {
        let predicted = self.predicted_armor_state(id);
        let obs_camera_yaw = observation.position_mm.y.atan2(observation.position_mm.x);
        let pred_camera_yaw = predicted[1].atan2(predicted[0]);
        normalize_angle(observation.yaw_rad - predicted[3]).abs()
            + normalize_angle(obs_camera_yaw - pred_camera_yaw).abs()
    }

    fn armor_radius(&self, id: usize) -> f64 {
        radius_from_state(self.x(), self.armor_num, id)
    }

    fn height_offset_for_id(&self, state: &na::DVector<f64>, id: usize) -> f64 {
        if self.outpost_height_phase_locked() {
            self.outpost_height_offset_for_id(id)
        } else {
            height_offset_from_state(state, self.armor_num, id)
        }
    }

    fn record_nis(&mut self, nis: f64) {
        self.last_nis = nis;
        self.recent_nis_failures
            .push_back(!nis.is_finite() || nis > 9.4877);
        while self.recent_nis_failures.len() > NIS_WINDOW_SIZE {
            self.recent_nis_failures.pop_front();
        }
    }

    fn recent_nis_failure_count(&self) -> usize {
        self.recent_nis_failures
            .iter()
            .filter(|failed| **failed)
            .count()
    }

    fn clamp_geometry(&mut self) {
        let armor_num = self.armor_num;
        let locked = self.outpost_height_phase_locked();
        {
            let x = self.x_mut();
            x[PRIMARY_RADIUS] = x[PRIMARY_RADIUS].clamp(MIN_RADIUS_MM, MAX_RADIUS_MM);
            if armor_num == 4 {
                let secondary =
                    (x[PRIMARY_RADIUS] + x[DELTA_RADIUS]).clamp(MIN_RADIUS_MM, MAX_RADIUS_MM);
                x[DELTA_RADIUS] = secondary - x[PRIMARY_RADIUS];
            } else {
                x[PRIMARY_RADIUS] = OUTPOST_RADIUS_MM;
                if !locked {
                    x[DELTA_RADIUS] = x[DELTA_RADIUS]
                        .clamp(-OUTPOST_MAX_HEIGHT_OFFSET_MM, OUTPOST_MAX_HEIGHT_OFFSET_MM);
                    x[HEIGHT_DIFF] = x[HEIGHT_DIFF]
                        .clamp(-OUTPOST_MAX_HEIGHT_OFFSET_MM, OUTPOST_MAX_HEIGHT_OFFSET_MM);
                }
            }
        }
        if armor_num == 3 && locked {
            self.apply_locked_outpost_height_offsets();
        }
    }

    fn reset_outpost_height_phase(&mut self) {
        self.outpost_height_phase_scores = [0.0; 6];
        self.outpost_height_phase_observations = 0;
        self.outpost_height_phase = None;
        self.outpost_height_phase_id_samples.clear();
        self.outpost_height_phase_id_counts = [0; 3];
        for samples in &mut self.outpost_height_phase_center_samples {
            samples.clear();
        }
    }

    fn outpost_height_phase_locked(&self) -> bool {
        self.is_outpost && self.outpost_height_phase.is_some()
    }

    fn outpost_height_offset_for_id(&self, id: usize) -> f64 {
        self.outpost_height_phase
            .and_then(|phase| outpost_height_offset_from_phase(phase, id))
            .unwrap_or(0.0)
    }

    fn apply_locked_outpost_height_offsets(&mut self) {
        if !self.outpost_height_phase_locked() {
            return;
        }
        self.x_mut()[DELTA_RADIUS] = self.outpost_height_offset_for_id(1);
        self.x_mut()[HEIGHT_DIFF] = self.outpost_height_offset_for_id(2);
        self.p_mut()[(DELTA_RADIUS, DELTA_RADIUS)] = OUTPOST_LOCKED_HEIGHT_VARIANCE_MM2;
        self.p_mut()[(HEIGHT_DIFF, HEIGHT_DIFF)] = OUTPOST_LOCKED_HEIGHT_VARIANCE_MM2;
    }

    fn update_outpost_height_phase(&mut self, observation: &YpdObservation, matched_id: usize) {
        if !self.is_outpost || matched_id >= 3 || !observation.position_mm.z.is_finite() {
            return;
        }

        self.outpost_height_phase_id_samples.push_back(matched_id);
        self.outpost_height_phase_id_counts[matched_id] += 1;
        for phase in 0..OUTPOST_HEIGHT_RANK_CANDIDATES.len() {
            let center_z = observation.position_mm.z
                - outpost_height_offset_from_phase(phase, matched_id).unwrap_or(0.0);
            self.outpost_height_phase_center_samples[phase].push_back(center_z);
        }

        while self.outpost_height_phase_id_samples.len() > OUTPOST_HEIGHT_PHASE_CENTER_CAPACITY {
            if let Some(old_id) = self.outpost_height_phase_id_samples.pop_front()
                && old_id < self.outpost_height_phase_id_counts.len()
            {
                self.outpost_height_phase_id_counts[old_id] =
                    self.outpost_height_phase_id_counts[old_id].saturating_sub(1);
            }
            for samples in &mut self.outpost_height_phase_center_samples {
                samples.pop_front();
            }
        }

        self.outpost_height_phase_observations = self.outpost_height_phase_id_samples.len();
        let mut candidate_center_z = [f64::NAN; 6];
        for (phase, center_slot) in candidate_center_z
            .iter_mut()
            .enumerate()
            .take(self.outpost_height_phase_scores.len())
        {
            let (score, center_z) = self.outpost_height_phase_score_from_samples(phase);
            self.outpost_height_phase_scores[phase] = score;
            *center_slot = center_z;
        }

        if self.outpost_height_phase_locked()
            || self.outpost_height_phase_observations < OUTPOST_HEIGHT_PHASE_MIN_CENTER_SAMPLES
            || !self.outpost_height_phase_has_enough_id_coverage()
        {
            return;
        }

        let mut phases: Vec<_> = (0..self.outpost_height_phase_scores.len()).collect();
        phases.sort_by(|lhs, rhs| {
            self.outpost_height_phase_scores[*lhs]
                .total_cmp(&self.outpost_height_phase_scores[*rhs])
        });
        let best = phases[0];
        let second = phases[1];
        let margin =
            self.outpost_height_phase_scores[second] - self.outpost_height_phase_scores[best];
        if candidate_center_z[best].is_finite() && margin >= OUTPOST_HEIGHT_PHASE_LOCK_MARGIN {
            self.outpost_height_phase = Some(best);
            self.x_mut()[4] = candidate_center_z[best];
            self.apply_locked_outpost_height_offsets();
        }
    }

    fn outpost_height_phase_score_from_samples(&self, phase: usize) -> (f64, f64) {
        let Some(center_z) = median_value(
            self.outpost_height_phase_center_samples[phase]
                .iter()
                .copied(),
        ) else {
            return (f64::INFINITY, f64::NAN);
        };
        let sigma_sq = OUTPOST_HEIGHT_PHASE_SIGMA_MM * OUTPOST_HEIGHT_PHASE_SIGMA_MM;
        let mut score = 0.0;
        let mut count = 0;
        for sample in &self.outpost_height_phase_center_samples[phase] {
            if !sample.is_finite() {
                continue;
            }
            let residual = sample - center_z;
            score += residual * residual / sigma_sq;
            count += 1;
        }
        if count > 0 {
            (score, center_z)
        } else {
            (f64::INFINITY, f64::NAN)
        }
    }

    fn outpost_height_phase_has_enough_id_coverage(&self) -> bool {
        self.outpost_height_phase_id_counts
            .iter()
            .all(|count| *count >= OUTPOST_HEIGHT_PHASE_MIN_SAMPLES_PER_ID)
    }

    fn apply_outpost_radius_prior(&mut self) {
        if !self.is_outpost {
            return;
        }
        let prior_variance = OUTPOST_RADIUS_PRIOR_SIGMA_MM * OUTPOST_RADIUS_PRIOR_SIGMA_MM;
        let innovation_variance = self.p()[(PRIMARY_RADIUS, PRIMARY_RADIUS)] + prior_variance;
        if !innovation_variance.is_finite() || innovation_variance <= 1e-9 {
            return;
        }

        let k = self.p().column(PRIMARY_RADIUS).into_owned() / innovation_variance;
        let innovation = OUTPOST_RADIUS_MM - self.x()[PRIMARY_RADIUS];
        {
            let x = self.x_mut();
            for row in 0..STATE_DIM {
                x[row] += k[row] * innovation;
            }
            x[6] = normalize_angle(x[6]);
        }

        let mut i_kh = na::DMatrix::<f64>::identity(STATE_DIM, STATE_DIM);
        for row in 0..STATE_DIM {
            i_kh[(row, PRIMARY_RADIUS)] -= k[row];
        }
        let rk = prior_variance * (&k * k.transpose());
        let p_new = symmetrize_dynamic(&(&i_kh * self.p() * &i_kh.transpose() + &rk));
        *self.p_mut() = p_new;
    }

    fn append_motion_sample(&mut self) {
        let (cx, cy, yaw_rate) = (self.x()[0], self.x()[2], self.x()[7]);
        self.motion_history.push_back(MotionSample {
            t_s: self.tracker_time_s,
            center_x: cx,
            center_y: cy,
            yaw_rate,
        });
        while self.motion_history.len() > MOTION_HISTORY_CAPACITY {
            self.motion_history.pop_front();
        }
    }

    fn yaw_rate_linear_accel(&self, window: usize) -> f64 {
        if self.motion_history.len() < window.max(2) {
            return f64::NAN;
        }
        let samples: Vec<_> = self
            .motion_history
            .iter()
            .rev()
            .take(window.max(2))
            .collect();
        linear_slope_abs(samples.iter().map(|sample| (sample.t_s, sample.yaw_rate)))
    }

    fn center_quadratic_accel(&self, window: usize) -> f64 {
        if self.motion_history.len() < window.max(3) {
            return f64::NAN;
        }
        let samples: Vec<_> = self
            .motion_history
            .iter()
            .rev()
            .take(window.max(3))
            .collect();
        let ax = quadratic_accel(samples.iter().map(|sample| (sample.t_s, sample.center_x)));
        let ay = quadratic_accel(samples.iter().map(|sample| (sample.t_s, sample.center_y)));
        if ax.is_finite() && ay.is_finite() {
            ax.hypot(ay)
        } else {
            f64::NAN
        }
    }
}

impl Default for YpdAngleTracker {
    fn default() -> Self {
        Self::new()
    }
}

fn diagonal_matrix(values: [f64; STATE_DIM]) -> na::DMatrix<f64> {
    let mut matrix = na::DMatrix::<f64>::zeros(STATE_DIM, STATE_DIM);
    for (index, value) in values.into_iter().enumerate() {
        matrix[(index, index)] = value;
    }
    matrix
}

/// 对称化动态大小协方差矩阵：`(M + Mᵀ) / 2`。
fn symmetrize_dynamic(matrix: &na::DMatrix<f64>) -> na::DMatrix<f64> {
    (matrix + matrix.transpose()) * 0.5
}

fn radial_sign(armor_num: usize) -> f64 {
    if armor_num == 3 { 1.0 } else { -1.0 }
}

fn radial_yaw_from_observed_yaw(observed_yaw: f64, armor_num: usize) -> f64 {
    if armor_num == 3 {
        normalize_angle(observed_yaw + OUTPOST_PLANE_TO_RADIAL_YAW_OFFSET_RAD)
    } else {
        normalize_angle(observed_yaw)
    }
}

fn observed_yaw_from_radial_yaw(radial_yaw: f64, armor_num: usize) -> f64 {
    if armor_num == 3 {
        normalize_angle(radial_yaw - OUTPOST_PLANE_TO_RADIAL_YAW_OFFSET_RAD)
    } else {
        normalize_angle(radial_yaw)
    }
}

fn radius_from_state(state: &na::DVector<f64>, armor_num: usize, id: usize) -> f64 {
    if armor_num == 4 && (id == 1 || id == 3) {
        state[PRIMARY_RADIUS] + state[DELTA_RADIUS]
    } else {
        state[PRIMARY_RADIUS]
    }
}

fn height_offset_from_state(state: &na::DVector<f64>, armor_num: usize, id: usize) -> f64 {
    if armor_num == 4 {
        if id == 1 || id == 3 {
            state[HEIGHT_DIFF]
        } else {
            0.0
        }
    } else if id == 1 {
        state[DELTA_RADIUS]
    } else if id == 2 {
        state[HEIGHT_DIFF]
    } else {
        0.0
    }
}

fn outpost_height_offset_from_phase(phase: usize, id: usize) -> Option<f64> {
    OUTPOST_HEIGHT_RANK_CANDIDATES
        .get(phase)
        .and_then(|candidate| candidate.get(id))
        .map(|rank| f64::from(*rank) * OUTPOST_HEIGHT_STEP_MM)
}

fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = (angle + std::f64::consts::PI) % std::f64::consts::TAU;
    if normalized < 0.0 {
        normalized += std::f64::consts::TAU;
    }
    normalized - std::f64::consts::PI
}

fn xyz_to_ypd(position: na::Point3<f64>) -> na::Vector3<f64> {
    let xy = position.x.hypot(position.y);
    na::Vector3::new(
        position.y.atan2(position.x),
        position.z.atan2(xy),
        position.coords.norm(),
    )
}

fn xyz_to_ypd_jacobian(position: na::Point3<f64>) -> na::SMatrix<f64, 3, 3> {
    let x = position.x;
    let y = position.y;
    let z = position.z;
    let xy_sq = (x * x + y * y).max(1e-9);
    let xy = xy_sq.sqrt();
    let xyz_sq = (xy_sq + z * z).max(1e-9);
    let pitch_den = z * z / xy_sq + 1.0;

    na::SMatrix::<f64, 3, 3>::from_row_slice(&[
        -y / xy_sq,
        x / xy_sq,
        0.0,
        -(x * z) / (pitch_den * xy_sq.powf(1.5)),
        -(y * z) / (pitch_den * xy_sq.powf(1.5)),
        1.0 / (pitch_den * xy),
        x / xyz_sq.sqrt(),
        y / xyz_sq.sqrt(),
        z / xyz_sq.sqrt(),
    ])
}

fn median_value(values: impl Iterator<Item = f64>) -> Option<f64> {
    let mut values: Vec<_> = values.filter(|value| value.is_finite()).collect();
    if values.is_empty() {
        return None;
    }
    values.sort_by(f64::total_cmp);
    let mid = values.len() / 2;
    if values.len() % 2 == 0 {
        Some((values[mid - 1] + values[mid]) * 0.5)
    } else {
        Some(values[mid])
    }
}

fn linear_slope_abs(samples: impl Iterator<Item = (f64, f64)>) -> f64 {
    let values: Vec<_> = samples.collect();
    if values.len() < 2 {
        return f64::NAN;
    }
    let t_base = values[0].0;
    let mut sum_t = 0.0;
    let mut sum_y = 0.0;
    let mut sum_tt = 0.0;
    let mut sum_ty = 0.0;
    for (t, y) in &values {
        let dt = *t - t_base;
        if !dt.is_finite() || !y.is_finite() {
            return f64::NAN;
        }
        sum_t += dt;
        sum_y += y;
        sum_tt += dt * dt;
        sum_ty += dt * y;
    }
    let count = values.len() as f64;
    let denom = count * sum_tt - sum_t * sum_t;
    if denom.abs() < 1e-9 {
        f64::NAN
    } else {
        ((count * sum_ty - sum_t * sum_y) / denom).abs()
    }
}

fn quadratic_accel(samples: impl Iterator<Item = (f64, f64)>) -> f64 {
    let values: Vec<_> = samples.collect();
    if values.len() < 3 {
        return f64::NAN;
    }
    let t_base = values[0].0;
    let mut a = na::SMatrix::<f64, 3, 3>::zeros();
    let mut b = na::SVector::<f64, 3>::zeros();
    for (t, y) in values {
        let dt = t - t_base;
        if !dt.is_finite() || !y.is_finite() {
            return f64::NAN;
        }
        let dt2 = dt * dt;
        let row = na::SVector::<f64, 3>::new(dt2, dt, 1.0);
        a += row * row.transpose();
        b += row * y;
    }
    a.lu().solve(&b).map_or(f64::NAN, |coeffs| 2.0 * coeffs[0])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn estimator_cfg() -> EstimatorCfg {
        toml::from_str(
            "\
armor_lost_wait_duration_ms = 100
enemy_lost_wait_duration_ms = 1000
ypd_geometry_recovery_window_frames = 24
ypd_geometry_recovery_cooldown_frames = 12
ypd_geometry_recovery_mismatch_required_streak = 2
ypd_geometry_recovery_min_matched_count = 2
ypd_geometry_recovery_z_sigma_threshold = 3.0
ypd_geometry_recovery_xy_sigma_threshold = 2.0
ypd_geometry_recovery_cov_inflation_scale = 48.0
ypd_geometry_recovery_min_dr_variance = 0.0025
ypd_geometry_recovery_min_h_variance = 0.000625
",
        )
        .unwrap()
    }

    fn observation(center: na::Point3<f64>, yaw: f64, radius: f64) -> YpdObservation {
        let position = na::Point3::new(
            center.x - radius * yaw.cos(),
            center.y - radius * yaw.sin(),
            center.z,
        );
        YpdObservation {
            position_mm: position,
            yaw_rad: yaw,
            image_center: na::Point2::new(320.0, 192.0),
            radius_hint_mm: radius,
        }
    }

    fn outpost_phase_observation(center_z: f64, phase: usize, id: usize) -> YpdObservation {
        let radial_yaw = id as f64 * std::f64::consts::TAU / 3.0;
        let position = na::Point3::new(
            1_000.0 + OUTPOST_RADIUS_MM * radial_yaw.cos(),
            OUTPOST_RADIUS_MM * radial_yaw.sin(),
            center_z + outpost_height_offset_from_phase(phase, id).unwrap(),
        );
        YpdObservation {
            position_mm: position,
            yaw_rad: observed_yaw_from_radial_yaw(radial_yaw, 3),
            image_center: na::Point2::new(320.0, 520.0),
            radius_hint_mm: OUTPOST_RADIUS_MM,
        }
    }

    fn initialize_outpost_tracker() -> YpdAngleTracker {
        let mut tracker = YpdAngleTracker::new();
        tracker.init(&outpost_phase_observation(200.0, 0, 0), 3);
        tracker
    }

    #[test]
    fn batch_update_assigns_unique_armor_ids() {
        let center = na::Point3::new(1_000.0, 0.0, 100.0);
        let mut tracker = YpdAngleTracker::new();
        tracker.init(&observation(center, 0.0, 200.0), 4);
        tracker.predict(0.01);

        let observations = [
            observation(center, 0.0, 200.0),
            observation(center, std::f64::consts::FRAC_PI_2, 200.0),
        ];
        tracker.update_batch(&observations, Some(0), &estimator_cfg());

        assert_eq!(tracker.last_batch_match_ids().len(), 2);
        assert_ne!(
            tracker.last_batch_match_ids()[0],
            tracker.last_batch_match_ids()[1]
        );
    }

    #[test]
    fn pure_prediction_advances_center_and_yaw() {
        let obs = observation(na::Point3::new(1_000.0, 0.0, 100.0), 0.0, 200.0);
        let mut tracker = YpdAngleTracker::new();
        tracker.init(&obs, 4);
        tracker.x_mut()[1] = 100.0;
        tracker.x_mut()[7] = 1.0;
        tracker.predict(0.05);

        let snapshot = tracker.snapshot().unwrap();

        assert!((snapshot.state11d[0] - 1_005.0).abs() < 1e-6);
        assert!((snapshot.state11d[6] - 0.05).abs() < 1e-6);
    }

    #[test]
    fn geometry_recovery_inflates_dr_and_height_covariance_after_mismatch() {
        let cfg = estimator_cfg();
        let center = na::Point3::new(1_000.0, 0.0, 100.0);
        let mut tracker = YpdAngleTracker::new();
        tracker.init(&observation(center, 0.0, 200.0), 4);
        tracker.p_mut()[(DELTA_RADIUS, DELTA_RADIUS)] = 10.0;
        tracker.p_mut()[(HEIGHT_DIFF, HEIGHT_DIFF)] = 10.0;
        let before_dr = tracker.p()[(DELTA_RADIUS, DELTA_RADIUS)];
        let before_h = tracker.p()[(HEIGHT_DIFF, HEIGHT_DIFF)];
        let mismatched = [
            observation(na::Point3::new(1_000.0, 0.0, 260.0), 0.0, 320.0),
            observation(
                na::Point3::new(1_000.0, 0.0, 260.0),
                std::f64::consts::FRAC_PI_2,
                320.0,
            ),
        ];

        tracker.note_observation_jump(true, &cfg);
        tracker.update_batch(&mismatched, Some(0), &cfg);
        tracker.note_observation_jump(true, &cfg);
        tracker.update_batch(&mismatched, Some(0), &cfg);

        assert!(tracker.p()[(DELTA_RADIUS, DELTA_RADIUS)] > before_dr);
        assert!(tracker.p()[(HEIGHT_DIFF, HEIGHT_DIFF)] > before_h);
    }

    #[test]
    fn outpost_observed_yaw_converts_to_radial_state_and_back() {
        let observed_yaw = 0.25;
        let mut tracker = YpdAngleTracker::new();
        let obs = YpdObservation {
            position_mm: na::Point3::new(1_000.0, 0.0, 100.0),
            yaw_rad: observed_yaw,
            image_center: na::Point2::new(320.0, 500.0),
            radius_hint_mm: OUTPOST_RADIUS_MM,
        };

        tracker.init(&obs, 3);
        let snapshot = tracker.snapshot().unwrap();

        assert!(
            (normalize_angle(snapshot.state11d[6] - observed_yaw)
                - OUTPOST_PLANE_TO_RADIAL_YAW_OFFSET_RAD)
                .abs()
                < 1e-9
        );
        assert!((snapshot.tracked_armor_xyza[3] - observed_yaw).abs() < 1e-9);
    }

    #[test]
    fn outpost_height_phase_locks_offsets_and_freezes_jacobian() {
        let mut tracker = initialize_outpost_tracker();
        let phase = 0;
        for round in 0..OUTPOST_HEIGHT_PHASE_MIN_CENTER_SAMPLES {
            let id = round % 3;
            tracker.update_outpost_height_phase(&outpost_phase_observation(200.0, phase, id), id);
        }

        assert_eq!(tracker.outpost_height_phase, Some(phase));
        assert_eq!(
            tracker.x()[DELTA_RADIUS],
            outpost_height_offset_from_phase(phase, 1).unwrap()
        );
        assert_eq!(
            tracker.x()[HEIGHT_DIFF],
            outpost_height_offset_from_phase(phase, 2).unwrap()
        );
        assert_eq!(
            tracker.p()[(DELTA_RADIUS, DELTA_RADIUS)],
            OUTPOST_LOCKED_HEIGHT_VARIANCE_MM2
        );

        let h_id1 = tracker.measurement_jacobian(tracker.x(), 1);
        let h_id2 = tracker.measurement_jacobian(tracker.x(), 2);
        assert_eq!(h_id1[(2, DELTA_RADIUS)], 0.0);
        assert_eq!(h_id2[(2, HEIGHT_DIFF)], 0.0);
    }

    #[test]
    fn outpost_low_image_primary_only_skips_secondary_observations() {
        let cfg = estimator_cfg();
        let mut tracker = initialize_outpost_tracker();
        let mut primary = outpost_phase_observation(200.0, 0, 0);
        primary.image_center.y = 500.0;
        let mut secondary = outpost_phase_observation(200.0, 0, 1);
        secondary.image_center.y = 650.0;

        tracker.update_batch(&[primary, secondary], Some(0), &cfg);

        assert_eq!(tracker.last_batch_match_ids().len(), 2);
        assert!(tracker.last_batch_match_ids()[0] >= 0);
        assert_eq!(tracker.last_batch_match_ids()[1], -1);
    }
}
