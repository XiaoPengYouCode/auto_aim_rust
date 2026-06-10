use std::time::Instant;

use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use tinympc_rs::policy::FixedPolicy;
use tinympc_rs::{Constraint, ProjectMulti, Solver, TerminationReason};

pub const SECOND_ORDER_POSITION_MPC_HORIZON: usize = 50;

const NX: usize = 4;
const NU: usize = 1;
const HX: usize = SECOND_ORDER_POSITION_MPC_HORIZON;
const HU: usize = HX - 1;
const EPSILON_DT_S: f64 = 1e-3;
const DEFAULT_RHO: f64 = 1.0;
const DEFAULT_POLICY_ITERS: usize = 1_000;

type MpcPolicy = FixedPolicy<f64, NX, NU>;
type MpcSolver = Solver<f64, MpcPolicy, NX, NU, HX, HU>;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SecondOrderPositionMpcConfig {
    pub model_dt_s: f64,
    pub track_q: f64,
    pub rate_q: f64,
    pub command_q: f64,
    pub command_track_target: bool,
    pub delta_r: f64,
    pub input_gain: f64,
    pub input_lag_s: f64,
    pub wn_rad_s: f64,
    pub zeta: f64,
    pub max_rate_deg_s: f64,
    pub max_lead_deg: f64,
    pub max_state_rate_deg_s: f64,
    pub output_stage_ratio: f64,
    pub max_iter: usize,
    pub primal_tolerance: f64,
    pub dual_tolerance: f64,
    pub relaxation: f64,
}

impl Default for SecondOrderPositionMpcConfig {
    fn default() -> Self {
        Self {
            model_dt_s: 0.004,
            track_q: 3_198.0,
            rate_q: 0.0,
            command_q: 1_000.0,
            command_track_target: true,
            delta_r: 48_343.0,
            input_gain: 0.99,
            input_lag_s: 0.0,
            wn_rad_s: 43.897,
            zeta: 0.4266,
            max_rate_deg_s: 720.0,
            max_lead_deg: 4.0,
            max_state_rate_deg_s: 720.0,
            output_stage_ratio: 0.0,
            max_iter: 20,
            primal_tolerance: 1e-3,
            dual_tolerance: 1e-3,
            relaxation: 1.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SecondOrderPositionMpcOutput {
    pub command_deg: f64,
    pub output_state_index: usize,
    pub iterations: usize,
    pub converged: bool,
    pub primal_residual: f64,
    pub dual_residual: f64,
    pub preview_tracking_error_deg: f64,
    pub preview_tracking_valid: bool,
    pub predicted_yaw_deg: Vec<f64>,
    pub predicted_command_deg: Vec<f64>,
    pub reference_yaw_deg: Vec<f64>,
    pub reference_rate_deg_s: Vec<f64>,
    pub solve_us: f64,
    pub update_us: f64,
}

#[derive(Debug, Clone, Copy)]
struct StateBounds {
    rate_limit_deg_s: f64,
    command_lower_deg: f64,
    command_upper_deg: f64,
    command_lead_enabled: bool,
}

impl Default for StateBounds {
    fn default() -> Self {
        Self {
            rate_limit_deg_s: f64::INFINITY,
            command_lower_deg: f64::NEG_INFINITY,
            command_upper_deg: f64::INFINITY,
            command_lead_enabled: false,
        }
    }
}

impl StateBounds {
    fn update(&mut self, measured_unwrapped_deg: f64, config: &SecondOrderPositionMpcConfig) {
        self.rate_limit_deg_s =
            if config.max_state_rate_deg_s.is_finite() && config.max_state_rate_deg_s > 0.0 {
                config.max_state_rate_deg_s
            } else {
                f64::INFINITY
            };

        self.command_lead_enabled = config.max_lead_deg.is_finite() && config.max_lead_deg > 0.0;
        if self.command_lead_enabled {
            self.command_lower_deg = measured_unwrapped_deg - config.max_lead_deg;
            self.command_upper_deg = measured_unwrapped_deg + config.max_lead_deg;
        } else {
            self.command_lower_deg = f64::NEG_INFINITY;
            self.command_upper_deg = f64::INFINITY;
        }
    }
}

impl ProjectMulti<f64, NX, HX> for StateBounds {
    fn project_multi(&self, points: &mut na::SMatrix<f64, NX, HX>) {
        if self.rate_limit_deg_s.is_finite() {
            for col in 0..HX {
                points[(1, col)] =
                    points[(1, col)].clamp(-self.rate_limit_deg_s, self.rate_limit_deg_s);
            }
        }

        if self.command_lead_enabled {
            for col in 1..HX {
                points[(3, col)] =
                    points[(3, col)].clamp(self.command_lower_deg, self.command_upper_deg);
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct InputBounds {
    max_delta_deg: f64,
}

impl Default for InputBounds {
    fn default() -> Self {
        Self {
            max_delta_deg: f64::INFINITY,
        }
    }
}

impl InputBounds {
    fn update(&mut self, rate_dt_s: f64, config: &SecondOrderPositionMpcConfig) {
        self.max_delta_deg = if config.max_rate_deg_s.is_finite() && config.max_rate_deg_s > 0.0 {
            config.max_rate_deg_s * rate_dt_s.max(EPSILON_DT_S)
        } else {
            f64::INFINITY
        };
    }
}

impl ProjectMulti<f64, NU, HU> for InputBounds {
    fn project_multi(&self, points: &mut na::SMatrix<f64, NU, HU>) {
        if !self.max_delta_deg.is_finite() {
            return;
        }
        for col in 0..HU {
            points[(0, col)] = points[(0, col)].clamp(-self.max_delta_deg, self.max_delta_deg);
        }
    }
}

pub struct SecondOrderPositionMpc {
    config: SecondOrderPositionMpcConfig,
    solver: MpcSolver,
    x_constraints: [Constraint<f64, StateBounds, NX, HX>; 1],
    u_constraints: [Constraint<f64, InputBounds, NU, HU>; 1],
    x_ref: na::SMatrix<f64, NX, HX>,
    u_ref: na::SMatrix<f64, NU, HU>,
    initialized: bool,
    last_command_deg: f64,
    last_effective_command_deg: f64,
    last_measured_deg: f64,
    filtered_rate_deg_s: f64,
    preview_window_start_index: usize,
    preview_window_end_index: usize,
    last_output: Option<SecondOrderPositionMpcOutput>,
}

impl SecondOrderPositionMpc {
    pub fn new(config: SecondOrderPositionMpcConfig) -> RbtResult<Self> {
        let solver = build_solver(config)?;
        Ok(Self {
            config,
            solver,
            x_constraints: [Constraint::new(StateBounds::default())],
            u_constraints: [Constraint::new(InputBounds::default())],
            x_ref: na::SMatrix::<f64, NX, HX>::zeros(),
            u_ref: na::SMatrix::<f64, NU, HU>::zeros(),
            initialized: false,
            last_command_deg: 0.0,
            last_effective_command_deg: 0.0,
            last_measured_deg: 0.0,
            filtered_rate_deg_s: 0.0,
            preview_window_start_index: 2,
            preview_window_end_index: 2,
            last_output: None,
        })
    }

    pub fn config(&self) -> SecondOrderPositionMpcConfig {
        self.config
    }

    pub fn configure(&mut self, config: SecondOrderPositionMpcConfig) -> RbtResult<()> {
        if self.config != config {
            self.solver = build_solver(config)?;
            self.config = config;
            self.x_constraints[0].reset();
            self.u_constraints[0].reset();
        }
        Ok(())
    }

    pub fn reset(&mut self, measured_deg: f64, measured_rate_deg_s: f64) {
        self.last_command_deg = normalize_angle_deg(measured_deg);
        self.last_effective_command_deg = self.last_command_deg;
        self.last_measured_deg = self.last_command_deg;
        self.filtered_rate_deg_s = if measured_rate_deg_s.is_finite() {
            measured_rate_deg_s
        } else {
            0.0
        };
        self.initialized = true;
        self.x_constraints[0].reset();
        self.u_constraints[0].reset();
        self.last_output = None;
    }

    pub fn set_preview_window(&mut self, start_index: usize, end_index: usize) {
        self.preview_window_start_index = start_index.min(HX - 1);
        self.preview_window_end_index = end_index.clamp(self.preview_window_start_index, HX - 1);
    }

    pub fn update(
        &mut self,
        target_deg: f64,
        measured_deg: f64,
        measured_rate_deg_s: f64,
        applied_dt_s: f64,
    ) -> RbtResult<SecondOrderPositionMpcOutput> {
        let target = [target_deg; HX];
        let target_rate = [0.0; HX];
        self.update_trajectory(
            &target,
            &target_rate,
            measured_deg,
            measured_rate_deg_s,
            applied_dt_s,
        )
    }

    pub fn update_trajectory(
        &mut self,
        target_deg_traj: &[f64],
        target_rate_deg_s_traj: &[f64],
        measured_deg: f64,
        measured_rate_deg_s: f64,
        applied_dt_s: f64,
    ) -> RbtResult<SecondOrderPositionMpcOutput> {
        let update_begin = Instant::now();
        if !self.initialized {
            self.reset(measured_deg, measured_rate_deg_s);
        }

        let model_dt_s = self.config.model_dt_s.max(EPSILON_DT_S);
        let mut rate_dt_s = if applied_dt_s.is_finite() && applied_dt_s > 0.0 {
            applied_dt_s
        } else {
            model_dt_s
        };
        rate_dt_s = rate_dt_s.clamp(EPSILON_DT_S, model_dt_s);

        let measured_unwrapped = closest_equivalent_angle_deg(self.last_measured_deg, measured_deg);
        let last_command_unwrapped =
            closest_equivalent_angle_deg(measured_unwrapped, self.last_command_deg);
        let last_effective_command_unwrapped =
            closest_equivalent_angle_deg(last_command_unwrapped, self.last_effective_command_deg);

        let max_state_rate = if self.config.max_state_rate_deg_s.is_finite()
            && self.config.max_state_rate_deg_s > 0.0
        {
            self.config.max_state_rate_deg_s
        } else {
            f64::INFINITY
        };
        let mut measured_rate_raw = if measured_rate_deg_s.is_finite() {
            measured_rate_deg_s
        } else {
            (measured_unwrapped - self.last_measured_deg) / rate_dt_s
        };
        if max_state_rate.is_finite() {
            measured_rate_raw = measured_rate_raw.clamp(-max_state_rate, max_state_rate);
        }
        self.filtered_rate_deg_s = measured_rate_raw;

        let x0 = na::SVector::<f64, NX>::from_column_slice(&[
            measured_unwrapped,
            self.filtered_rate_deg_s,
            last_effective_command_unwrapped,
            last_command_unwrapped,
        ]);

        let target_unwrapped = unwrap_reference_trajectory(target_deg_traj, last_command_unwrapped);
        let target_rate_ref = build_rate_reference_trajectory(
            &target_unwrapped,
            target_rate_deg_s_traj,
            rate_dt_s,
            &self.config,
        );
        self.fill_references(
            &target_unwrapped,
            &target_rate_ref,
            last_effective_command_unwrapped,
            last_command_unwrapped,
        );

        self.x_constraints[0]
            .projector_mut()
            .update(measured_unwrapped, &self.config);
        self.u_constraints[0]
            .projector_mut()
            .update(rate_dt_s, &self.config);

        let solve_begin = Instant::now();
        let solution = self
            .solver
            .initial_condition(x0)
            .x_constraints(&mut self.x_constraints)
            .u_constraints(&mut self.u_constraints)
            .x_reference(&self.x_ref)
            .u_reference(&self.u_ref)
            .solve();
        let solve_us = solve_begin.elapsed().as_secs_f64() * 1e6;

        let x_prediction = solution.x_prediction_full();
        let u_prediction = solution.u_prediction_full();
        let iterations = solution.iterations;
        let converged = solution.reason == TerminationReason::Converged;
        let primal_residual = solution.prim_residual;
        let dual_residual = solution.dual_residual;

        let output_state_index = output_state_index(self.config.output_stage_ratio);
        let raw_command_unwrapped = if output_state_index == 0 {
            let delta = finite_or(u_prediction[(0, 0)], 0.0);
            last_command_unwrapped + delta
        } else {
            finite_or(
                x_prediction[(3, output_state_index)],
                last_command_unwrapped,
            )
        };
        let command_unwrapped = constrain_output_command(
            raw_command_unwrapped,
            last_command_unwrapped,
            measured_unwrapped,
            rate_dt_s,
            &self.config,
        );

        let (preview_error, preview_valid) = preview_tracking_error(
            &x_prediction,
            &target_unwrapped,
            self.preview_window_start_index,
            self.preview_window_end_index,
        );

        let effective_command_unwrapped =
            finite_or(x_prediction[(2, 1)], last_effective_command_unwrapped);
        self.last_effective_command_deg = normalize_angle_deg(effective_command_unwrapped);
        self.last_command_deg = normalize_angle_deg(command_unwrapped);
        self.last_measured_deg = measured_unwrapped;

        let output = SecondOrderPositionMpcOutput {
            command_deg: self.last_command_deg,
            output_state_index,
            iterations,
            converged,
            primal_residual,
            dual_residual,
            preview_tracking_error_deg: preview_error,
            preview_tracking_valid: preview_valid,
            predicted_yaw_deg: row_to_vec::<0>(&x_prediction),
            predicted_command_deg: row_to_vec::<3>(&x_prediction),
            reference_yaw_deg: target_unwrapped.to_vec(),
            reference_rate_deg_s: target_rate_ref.to_vec(),
            solve_us,
            update_us: update_begin.elapsed().as_secs_f64() * 1e6,
        };
        self.last_output = Some(output.clone());
        Ok(output)
    }

    pub fn last_output(&self) -> Option<&SecondOrderPositionMpcOutput> {
        self.last_output.as_ref()
    }

    fn fill_references(
        &mut self,
        target_unwrapped: &[f64; HX],
        target_rate_ref: &[f64; HX],
        last_effective_command_unwrapped: f64,
        last_command_unwrapped: f64,
    ) {
        self.x_ref.fill(0.0);
        self.u_ref.fill(0.0);
        for col in 0..HX {
            self.x_ref[(0, col)] = target_unwrapped[col];
            self.x_ref[(1, col)] = target_rate_ref[col];
            self.x_ref[(2, col)] = last_effective_command_unwrapped;
            self.x_ref[(3, col)] = if self.config.command_track_target {
                target_unwrapped[col]
            } else {
                last_command_unwrapped
            };
        }
    }
}

impl Default for SecondOrderPositionMpc {
    fn default() -> Self {
        Self::new(SecondOrderPositionMpcConfig::default())
            .expect("default second-order MPC config should be valid")
    }
}

fn build_solver(config: SecondOrderPositionMpcConfig) -> RbtResult<MpcSolver> {
    let model_dt_s = config.model_dt_s.max(EPSILON_DT_S);
    let input_gain = config.input_gain.max(0.0);
    let input_lag_s = config.input_lag_s.max(0.0);
    let wn = config.wn_rad_s.max(1e-3);
    let zeta = config.zeta.max(0.0);
    let wn2 = wn * wn;
    let input_alpha = if input_lag_s > 1e-6 {
        (-model_dt_s / input_lag_s).exp()
    } else {
        0.0
    };
    let input_blend = 1.0 - input_alpha;

    let theta_theta = 1.0 - 0.5 * wn2 * model_dt_s * model_dt_s;
    let theta_rate = model_dt_s - zeta * wn * model_dt_s * model_dt_s;
    let theta_effective_command = 0.5 * input_gain * wn2 * model_dt_s * model_dt_s;
    let omega_theta = -wn2 * model_dt_s;
    let omega_rate = 1.0 - 2.0 * zeta * wn * model_dt_s;
    let omega_effective_command = input_gain * wn2 * model_dt_s;

    let a = na::SMatrix::<f64, NX, NX>::from_row_slice(&[
        theta_theta,
        theta_rate,
        theta_effective_command,
        0.0,
        omega_theta,
        omega_rate,
        omega_effective_command,
        0.0,
        0.0,
        0.0,
        input_alpha,
        input_blend,
        0.0,
        0.0,
        0.0,
        1.0,
    ]);
    let b = na::SMatrix::<f64, NX, NU>::from_column_slice(&[0.0, 0.0, input_blend, 1.0]);

    let q =
        na::SMatrix::<f64, NX, NX>::from_diagonal(&na::SVector::<f64, NX>::from_column_slice(&[
            config.track_q.max(0.0),
            config.rate_q.max(0.0),
            0.0,
            config.command_q.max(0.0),
        ]));
    let r = na::SMatrix::<f64, NU, NU>::from_element(config.delta_r.max(1e-6));
    let s = na::SMatrix::<f64, NX, NU>::zeros();
    let policy = MpcPolicy::new(DEFAULT_RHO, DEFAULT_POLICY_ITERS, &a, &b, &q, &r, &s)
        .map_err(|err| RbtError::StringError(format!("failed to build TinyMPC policy: {err:?}")))?;

    let mut solver = MpcSolver::new(a, b, policy);
    solver.config.max_iter = config.max_iter.max(1);
    solver.config.prim_tol = config.primal_tolerance.max(1e-9);
    solver.config.dual_tol = config.dual_tolerance.max(1e-9);
    solver.config.do_check = 1;
    solver.config.relaxation = config.relaxation.clamp(1.0, 1.9);
    Ok(solver)
}

fn unwrap_reference_trajectory(target_deg_traj: &[f64], reference_deg: f64) -> [f64; HX] {
    let mut unwrapped = [reference_deg; HX];
    if target_deg_traj.is_empty() {
        return unwrapped;
    }

    let copy_count = target_deg_traj.len().min(HX);
    let mut prev = closest_equivalent_angle_deg(reference_deg, target_deg_traj[0]);
    unwrapped[0] = prev;
    for index in 1..copy_count {
        prev = closest_equivalent_angle_deg(prev, target_deg_traj[index]);
        unwrapped[index] = prev;
    }
    for value in unwrapped.iter_mut().take(HX).skip(copy_count) {
        *value = prev;
    }
    unwrapped
}

fn build_rate_reference_trajectory(
    target_unwrapped: &[f64; HX],
    target_rate_deg_s_traj: &[f64],
    reference_dt_s: f64,
    config: &SecondOrderPositionMpcConfig,
) -> [f64; HX] {
    let dt_s = reference_dt_s.max(EPSILON_DT_S);
    let mut rate_ref = [0.0; HX];
    rate_ref[0] = (target_unwrapped[1] - target_unwrapped[0]) / dt_s;
    for index in 1..HX - 1 {
        rate_ref[index] =
            (target_unwrapped[index + 1] - target_unwrapped[index - 1]) / (2.0 * dt_s);
    }
    rate_ref[HX - 1] = (target_unwrapped[HX - 1] - target_unwrapped[HX - 2]) / dt_s;

    for (index, rate) in target_rate_deg_s_traj.iter().take(HX).enumerate() {
        if rate.is_finite() {
            rate_ref[index] = *rate;
        }
    }

    if config.max_state_rate_deg_s.is_finite() && config.max_state_rate_deg_s > 0.0 {
        let limit = config.max_state_rate_deg_s;
        for rate in &mut rate_ref {
            if rate.is_finite() {
                *rate = rate.clamp(-limit, limit);
            }
        }
    }
    rate_ref
}

fn preview_tracking_error(
    x_prediction: &na::SMatrix<f64, NX, HX>,
    target_unwrapped: &[f64; HX],
    start_index: usize,
    end_index: usize,
) -> (f64, bool) {
    let start = start_index.min(HX - 1);
    let end = end_index.clamp(start, HX - 1);
    let mut max_error = 0.0;
    let mut valid = false;
    for index in start..=end {
        let error = (target_unwrapped[index] - x_prediction[(0, index)]).abs();
        if error.is_finite() {
            max_error = f64::max(max_error, error);
            valid = true;
        }
    }
    (max_error, valid)
}

fn row_to_vec<const ROW: usize>(matrix: &na::SMatrix<f64, NX, HX>) -> Vec<f64> {
    (0..HX).map(|col| matrix[(ROW, col)]).collect()
}

fn output_state_index(output_stage_ratio: f64) -> usize {
    let ratio = if output_stage_ratio.is_finite() {
        output_stage_ratio.clamp(0.0, 1.0)
    } else {
        0.0
    };
    (ratio * (HX - 1) as f64).round() as usize
}

fn finite_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}

fn constrain_output_command(
    command_unwrapped: f64,
    last_command_unwrapped: f64,
    measured_unwrapped: f64,
    rate_dt_s: f64,
    config: &SecondOrderPositionMpcConfig,
) -> f64 {
    let mut constrained = command_unwrapped;
    if config.max_rate_deg_s.is_finite() && config.max_rate_deg_s > 0.0 {
        let max_delta = config.max_rate_deg_s * rate_dt_s.max(EPSILON_DT_S);
        constrained = constrained.clamp(
            last_command_unwrapped - max_delta,
            last_command_unwrapped + max_delta,
        );
    }
    if config.max_lead_deg.is_finite() && config.max_lead_deg > 0.0 {
        constrained = constrained.clamp(
            measured_unwrapped - config.max_lead_deg,
            measured_unwrapped + config.max_lead_deg,
        );
    }
    constrained
}

fn normalize_angle_deg(angle_deg: f64) -> f64 {
    let mut result = (angle_deg + 180.0) % 360.0;
    if result < 0.0 {
        result += 360.0;
    }
    result - 180.0
}

fn closest_equivalent_angle_deg(reference_deg: f64, angle_deg: f64) -> f64 {
    reference_deg + normalize_angle_deg(angle_deg - reference_deg)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> SecondOrderPositionMpcConfig {
        SecondOrderPositionMpcConfig {
            model_dt_s: 0.01,
            track_q: 50.0,
            rate_q: 0.1,
            command_q: 5.0,
            delta_r: 1.0,
            input_gain: 1.0,
            wn_rad_s: 18.0,
            zeta: 0.85,
            max_rate_deg_s: 100.0,
            max_lead_deg: 5.0,
            max_state_rate_deg_s: 300.0,
            max_iter: 30,
            ..Default::default()
        }
    }

    #[test]
    fn holds_zero_target_near_zero() {
        let mut mpc = SecondOrderPositionMpc::new(test_config()).unwrap();
        mpc.reset(0.0, 0.0);

        let output = mpc.update(0.0, 0.0, 0.0, 0.01).unwrap();

        assert!(output.command_deg.abs() < 1e-6);
        assert_eq!(
            output.predicted_yaw_deg.len(),
            SECOND_ORDER_POSITION_MPC_HORIZON
        );
        assert_eq!(
            output.reference_yaw_deg.len(),
            SECOND_ORDER_POSITION_MPC_HORIZON
        );
    }

    #[test]
    fn command_moves_toward_target_and_respects_delta_limit() {
        let mut mpc = SecondOrderPositionMpc::new(test_config()).unwrap();
        mpc.reset(0.0, 0.0);

        let output = mpc.update(30.0, 0.0, 0.0, 0.01).unwrap();

        assert!(output.command_deg > 0.0);
        assert!(output.command_deg <= 1.0 + 1e-6);
    }

    #[test]
    fn unwraps_reference_across_angle_boundary() {
        let mut mpc = SecondOrderPositionMpc::new(test_config()).unwrap();
        mpc.reset(179.0, 0.0);

        let output = mpc.update(-179.0, 179.0, 0.0, 0.01).unwrap();

        assert!((output.reference_yaw_deg[0] - 181.0).abs() < 1e-9);
        assert!(output.command_deg > 179.0 || output.command_deg < -175.0);
    }

    #[test]
    fn reports_preview_tracking_error() {
        let mut mpc = SecondOrderPositionMpc::new(test_config()).unwrap();
        mpc.set_preview_window(2, 4);
        mpc.reset(0.0, 0.0);

        let output = mpc.update(10.0, 0.0, 0.0, 0.01).unwrap();

        assert!(output.preview_tracking_valid);
        assert!(output.preview_tracking_error_deg.is_finite());
        assert_eq!(output.output_state_index, 0);
    }
}
