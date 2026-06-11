use std::collections::VecDeque;

const STATE_DIM: usize = 11;
const PRIMARY_RADIUS: usize = 8;
const DELTA_RADIUS: usize = 9;
const HEIGHT_DIFF: usize = 10;
const MIN_RADIUS_MM: f64 = 50.0;
const MAX_RADIUS_MM: f64 = 500.0;
const OUTPOST_RADIUS_MM: f64 = 276.5;
const OUTPOST_MAX_HEIGHT_OFFSET_MM: f64 = 600.0;
const MOTION_HISTORY_CAPACITY: usize = 128;
const NIS_WINDOW_SIZE: usize = 100;

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
    x: na::SVector<f64, STATE_DIM>,
    p: na::SMatrix<f64, STATE_DIM, STATE_DIM>,
    last_nis: f64,
    recent_nis_failures: VecDeque<bool>,
    last_batch_match_ids: Vec<isize>,
    motion_history: VecDeque<MotionSample>,
}

impl YpdAngleTracker {
    pub fn new() -> Self {
        let mut tracker = Self {
            initialized: false,
            is_outpost: false,
            armor_num: 4,
            tracked_id: 0,
            update_count: 0,
            tracker_time_s: 0.0,
            is_converged: false,
            x: na::SVector::<f64, STATE_DIM>::zeros(),
            p: na::SMatrix::<f64, STATE_DIM, STATE_DIM>::identity(),
            last_nis: 0.0,
            recent_nis_failures: VecDeque::from([false]),
            last_batch_match_ids: Vec::new(),
            motion_history: VecDeque::new(),
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
        self.x.fill(0.0);
        self.p = na::SMatrix::<f64, STATE_DIM, STATE_DIM>::identity();
        self.last_nis = 0.0;
        self.recent_nis_failures.clear();
        self.recent_nis_failures.push_back(false);
        self.last_batch_match_ids.clear();
        self.motion_history.clear();
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

        let yaw = normalize_angle(observation.yaw_rad);
        let sign = radial_sign(self.armor_num);
        self.x[0] = observation.position_mm.x - sign * radius * yaw.cos();
        self.x[2] = observation.position_mm.y - sign * radius * yaw.sin();
        self.x[4] = observation.position_mm.z;
        self.x[6] = yaw;
        self.x[8] = radius;

        self.p = if self.is_outpost {
            diagonal_matrix([
                1_000.0, 64_000.0, 1_000.0, 64_000.0, 1_000.0, 81_000.0, 0.4, 100.0, 1.0, 90_000.0,
                90_000.0,
            ])
        } else {
            diagonal_matrix([
                1_000.0, 64_000.0, 1_000.0, 64_000.0, 1_000.0, 64_000.0, 0.4, 100.0, 10_000.0,
                10_000.0, 10_000.0,
            ])
        };

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

        if self.is_outpost && self.converged() && self.x[7].abs() > 2.0 {
            self.x[7] = self.x[7].signum() * 2.51;
        }

        let mut f = na::SMatrix::<f64, STATE_DIM, STATE_DIM>::identity();
        f[(0, 1)] = dt;
        f[(2, 3)] = dt;
        f[(4, 5)] = dt;
        f[(6, 7)] = dt;

        self.x = f * self.x;
        self.x[6] = normalize_angle(self.x[6]);
        self.clamp_geometry();
        self.p = symmetrize(f * self.p * f.transpose() + self.process_noise(dt));
    }

    pub fn update_batch(
        &mut self,
        observations: &[YpdObservation],
        preferred_index: Option<usize>,
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

        for index in 0..limit {
            let matched_id = assignment[index]
                .unwrap_or_else(|| self.select_best_armor_id(&observations[index]));
            if self.correct_with_observation(&observations[index], matched_id) {
                self.last_batch_match_ids[index] = matched_id as isize;
                if index == tracked_index {
                    primary_match = Some(matched_id);
                }
            }
        }

        if let Some(matched_id) = primary_match {
            self.tracked_id = matched_id;
        }
        self.clamp_geometry();
        self.append_motion_sample();
    }

    pub fn snapshot(&self) -> Option<YpdTrackerSnapshot> {
        if !self.initialized {
            return None;
        }

        let tracked_armor_xyza = self.predicted_armor_state(self.tracked_id);
        let mut state11d = [0.0; STATE_DIM];
        state11d.copy_from_slice(self.x.as_slice());

        let mut state9 = [0.0; 9];
        state9[0] = self.x[0];
        state9[1] = self.x[1];
        state9[2] = self.x[2];
        state9[3] = self.x[3];
        state9[4] = tracked_armor_xyza[2];
        state9[5] = self.x[5];
        state9[6] = tracked_armor_xyza[3];
        state9[7] = self.x[7];
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
            self.x[PRIMARY_RADIUS] > MIN_RADIUS_MM && self.x[PRIMARY_RADIUS] < MAX_RADIUS_MM;
        if !primary_ok {
            return true;
        }
        if self.armor_num == 4 {
            let secondary = self.x[PRIMARY_RADIUS] + self.x[DELTA_RADIUS];
            !(secondary > MIN_RADIUS_MM && secondary < MAX_RADIUS_MM)
        } else {
            !self.x[DELTA_RADIUS].is_finite()
                || !self.x[HEIGHT_DIFF].is_finite()
                || self.x[DELTA_RADIUS].abs() > OUTPOST_MAX_HEIGHT_OFFSET_MM
                || self.x[HEIGHT_DIFF].abs() > OUTPOST_MAX_HEIGHT_OFFSET_MM
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

    fn process_noise(&self, dt: f64) -> na::SMatrix<f64, STATE_DIM, STATE_DIM> {
        let pos_noise = if self.is_outpost { 10_000.0 } else { 100_000.0 };
        let yaw_noise = if self.is_outpost { 0.1 } else { 400.0 };
        let a = dt.powi(4) / 4.0;
        let b = dt.powi(3) / 2.0;
        let c = dt * dt;

        let mut q = na::SMatrix::<f64, STATE_DIM, STATE_DIM>::zeros();
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

    fn predicted_armor_position(
        &self,
        state: &na::SVector<f64, STATE_DIM>,
        id: usize,
    ) -> na::Point3<f64> {
        let angle =
            normalize_angle(state[6] + id as f64 * std::f64::consts::TAU / self.armor_num as f64);
        let radius = radius_from_state(state, self.armor_num, id);
        let sign = radial_sign(self.armor_num);
        na::Point3::new(
            state[0] + sign * radius * angle.cos(),
            state[2] + sign * radius * angle.sin(),
            state[4] + height_offset_from_state(state, self.armor_num, id),
        )
    }

    fn predicted_armor_state(&self, id: usize) -> [f64; 4] {
        let clamped_id = id.min(self.armor_num.saturating_sub(1));
        let angle = normalize_angle(
            self.x[6] + clamped_id as f64 * std::f64::consts::TAU / self.armor_num as f64,
        );
        let position = self.predicted_armor_position(&self.x, clamped_id);
        [position.x, position.y, position.z, angle]
    }

    fn predicted_measurement(
        &self,
        state: &na::SVector<f64, STATE_DIM>,
        id: usize,
    ) -> na::SVector<f64, 4> {
        let position = self.predicted_armor_position(state, id);
        let ypd = xyz_to_ypd(position);
        let angle =
            normalize_angle(state[6] + id as f64 * std::f64::consts::TAU / self.armor_num as f64);
        na::SVector::<f64, 4>::new(ypd.x, ypd.y, ypd.z, angle)
    }

    fn measurement_jacobian(
        &self,
        state: &na::SVector<f64, STATE_DIM>,
        id: usize,
    ) -> na::SMatrix<f64, 4, STATE_DIM> {
        let angle =
            normalize_angle(state[6] + id as f64 * std::f64::consts::TAU / self.armor_num as f64);
        let use_secondary_radius = self.armor_num == 4 && (id == 1 || id == 3);
        let sign = radial_sign(self.armor_num);
        let radius = if use_secondary_radius {
            state[PRIMARY_RADIUS] + state[DELTA_RADIUS]
        } else {
            state[PRIMARY_RADIUS]
        };

        let mut h_xyza = na::SMatrix::<f64, 4, STATE_DIM>::zeros();
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
        h_xyza[(2, DELTA_RADIUS)] = if self.armor_num == 3 && id == 1 {
            1.0
        } else {
            0.0
        };
        h_xyza[(2, HEIGHT_DIFF)] =
            if (self.armor_num == 4 && (id == 1 || id == 3)) || (self.armor_num == 3 && id == 2) {
                1.0
            } else {
                0.0
            };
        h_xyza[(3, 6)] = 1.0;

        let h_ypd = xyz_to_ypd_jacobian(self.predicted_armor_position(state, id));
        let mut h_ypda = na::SMatrix::<f64, 4, 4>::zeros();
        h_ypda.fixed_view_mut::<3, 3>(0, 0).copy_from(&h_ypd);
        h_ypda[(3, 3)] = 1.0;
        h_ypda * h_xyza
    }

    fn measurement_noise(&self, observation: &YpdObservation) -> na::SMatrix<f64, 4, 4> {
        let ypd = xyz_to_ypd(observation.position_mm);
        let center_yaw = observation.position_mm.y.atan2(observation.position_mm.x);
        let delta_angle = normalize_angle(observation.yaw_rad - center_yaw).abs();
        let distance_sigma_mm = (ypd.z.abs() * 0.03).clamp(10.0, 250.0);

        let mut r = na::SMatrix::<f64, 4, 4>::zeros();
        r[(0, 0)] = 4e-3;
        r[(1, 1)] = 4e-3;
        r[(2, 2)] = distance_sigma_mm * distance_sigma_mm;
        r[(3, 3)] = (delta_angle.abs() + 1.0).ln() / 20.0 + 9e-2;
        r
    }

    fn correct_with_observation(&mut self, observation: &YpdObservation, id: usize) -> bool {
        let matched_id = id.min(self.armor_num.saturating_sub(1));
        let ypd = xyz_to_ypd(observation.position_mm);
        let z = na::SVector::<f64, 4>::new(ypd.x, ypd.y, ypd.z, observation.yaw_rad);
        let h = self.measurement_jacobian(&self.x, matched_id);
        let r = self.measurement_noise(observation);
        let predicted = self.predicted_measurement(&self.x, matched_id);
        let mut residual = z - predicted;
        residual[0] = normalize_angle(residual[0]);
        residual[1] = normalize_angle(residual[1]);
        residual[3] = normalize_angle(residual[3]);

        let s = h * self.p * h.transpose() + r;
        let Some(s_inv) = s.try_inverse() else {
            self.record_nis(f64::INFINITY);
            return false;
        };
        let prior_nis = residual.transpose() * s_inv * residual;
        let prior_nis = prior_nis[(0, 0)];

        let k = self.p * h.transpose() * s_inv;
        let i = na::SMatrix::<f64, STATE_DIM, STATE_DIM>::identity();
        self.x += k * residual;
        self.x[6] = normalize_angle(self.x[6]);
        self.clamp_geometry();
        self.p = symmetrize((i - k * h) * self.p * (i - k * h).transpose() + k * r * k.transpose());

        self.record_nis(prior_nis);
        self.update_count += 1;
        true
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
        radius_from_state(&self.x, self.armor_num, id)
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
        self.x[PRIMARY_RADIUS] = self.x[PRIMARY_RADIUS].clamp(MIN_RADIUS_MM, MAX_RADIUS_MM);
        if self.armor_num == 4 {
            let secondary =
                (self.x[PRIMARY_RADIUS] + self.x[DELTA_RADIUS]).clamp(MIN_RADIUS_MM, MAX_RADIUS_MM);
            self.x[DELTA_RADIUS] = secondary - self.x[PRIMARY_RADIUS];
        } else {
            self.x[PRIMARY_RADIUS] = OUTPOST_RADIUS_MM;
            self.x[DELTA_RADIUS] = self.x[DELTA_RADIUS]
                .clamp(-OUTPOST_MAX_HEIGHT_OFFSET_MM, OUTPOST_MAX_HEIGHT_OFFSET_MM);
            self.x[HEIGHT_DIFF] = self.x[HEIGHT_DIFF]
                .clamp(-OUTPOST_MAX_HEIGHT_OFFSET_MM, OUTPOST_MAX_HEIGHT_OFFSET_MM);
        }
    }

    fn append_motion_sample(&mut self) {
        self.motion_history.push_back(MotionSample {
            t_s: self.tracker_time_s,
            center_x: self.x[0],
            center_y: self.x[2],
            yaw_rate: self.x[7],
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

fn diagonal_matrix(values: [f64; STATE_DIM]) -> na::SMatrix<f64, STATE_DIM, STATE_DIM> {
    let mut matrix = na::SMatrix::<f64, STATE_DIM, STATE_DIM>::zeros();
    for (index, value) in values.into_iter().enumerate() {
        matrix[(index, index)] = value;
    }
    matrix
}

fn symmetrize(
    matrix: na::SMatrix<f64, STATE_DIM, STATE_DIM>,
) -> na::SMatrix<f64, STATE_DIM, STATE_DIM> {
    (matrix + matrix.transpose()) * 0.5
}

fn radial_sign(armor_num: usize) -> f64 {
    if armor_num == 3 { 1.0 } else { -1.0 }
}

fn radius_from_state(state: &na::SVector<f64, STATE_DIM>, armor_num: usize, id: usize) -> f64 {
    if armor_num == 4 && (id == 1 || id == 3) {
        state[PRIMARY_RADIUS] + state[DELTA_RADIUS]
    } else {
        state[PRIMARY_RADIUS]
    }
}

fn height_offset_from_state(
    state: &na::SVector<f64, STATE_DIM>,
    armor_num: usize,
    id: usize,
) -> f64 {
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
        tracker.update_batch(&observations, Some(0));

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
        tracker.x[1] = 100.0;
        tracker.x[7] = 1.0;
        tracker.predict(0.05);

        let snapshot = tracker.snapshot().unwrap();

        assert!((snapshot.state11d[0] - 1_005.0).abs() < 1e-6);
        assert!((snapshot.state11d[6] - 0.05).abs() < 1e-6);
    }
}
