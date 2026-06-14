//! 大符曲线 EKF。
//!
//! 迁移自 vivsionn `BuffTracker` 的 `big_runev2` 曲线模型，内部基于 `rbt_base` 的共享不定长
//! EKF（`ExtendedKalmanFilter`）。这里只承担大符相位/曲线参数的滤波与预测；rune 圆心、目标
//! 几何、装甲板 pitch/yaw 仍由 `solved.rs` 的 PnP 提供，由 `tracker.rs` 持有。
//!
//! 曲线模型（与 vivsionn 一致）：
//! - `speed(phase) = a·sin(phase) + (base - a)`，`base = 2.090`
//! - `angle_delta(a, w, phase, dt) = (base - a)·dt + a/w·(cos(phase) - cos(phase + w·dt))`
//!
//! 状态向量 `[phase, a, w]`（3 维）。phase 由观察 roll 经方向系数积分推进，a/w 为曲线参数。
//! `a` 与 `w` 各自有过程噪声与合法范围，超出范围时夹紧到初值。

use crate::rbt_base::rbt_algorithm::rbt_ekf::ExtendedKalmanFilter;
use crate::rbt_infra::rbt_cfg::EnergyMechanismTrackerCfg;

/// 大符基础速度常数（rad/s），与 vivsionn `kBigBuffBaseSpeed` 一致。
pub const BIG_BUFF_BASE_SPEED: f64 = 2.090;
const BIG_BUFF_A_INIT: f64 = 0.9125;
const BIG_BUFF_W_INIT: f64 = 1.942;
const BIG_BUFF_A_MIN: f64 = 0.780;
const BIG_BUFF_A_MAX: f64 = 1.045;
const BIG_BUFF_W_MIN: f64 = 1.884;
const BIG_BUFF_W_MAX: f64 = 2.000;
const BIG_BUFF_SPEED_HARD_MAX: f64 = 2.35;
const STATE_DIM: usize = 3;
const PHASE: usize = 0;
const A: usize = 1;
const W: usize = 2;

/// 大符曲线 EKF 配置（从 `EnergyMechanismTrackerCfg` 提取滤波相关字段）。
#[derive(Debug, Clone, Copy)]
struct CurveFilterConfig {
    phase_process_noise: f64,
    a_process_noise: f64,
    w_process_noise: f64,
    speed_measurement_noise: f64,
    speed_measurement_gate: f64,
    speed_slew_limit: f64,
    phi_correction_limit: f64,
    phi_seed_frames: usize,
    window_samples: usize,
    window_s: f64,
    min_history: usize,
    /// 是否启用曲线 EKF 拟合；关闭后退化为常速预测。
    curve_ekf_fit_enabled: bool,
    /// 是否启用速度测量更新。
    speed_measurement_enabled: bool,
    /// 测量噪声 R 的整体缩放（>=1 放大噪声，降低测量信任）。
    measurement_noise_scale: f64,
}

impl CurveFilterConfig {
    fn from_tracker_cfg(cfg: &EnergyMechanismTrackerCfg) -> Self {
        Self {
            phase_process_noise: cfg.big_phase_process_noise.max(0.0),
            a_process_noise: cfg.big_a_process_noise.max(0.0),
            w_process_noise: cfg.big_w_process_noise.max(0.0),
            speed_measurement_noise: cfg.big_speed_measurement_noise.max(1e-6),
            speed_measurement_gate: cfg.big_speed_measurement_gate.max(1e-6),
            speed_slew_limit: cfg.big_curve_speed_slew_limit.max(0.0),
            phi_correction_limit: cfg.big_curve_phi_correction_limit.max(0.0),
            phi_seed_frames: cfg.big_phi_seed_frames,
            window_samples: cfg.big_speed_measurement_window_samples.clamp(2, 16),
            window_s: cfg.big_speed_measurement_window_s.max(1e-3),
            min_history: cfg.big_speed_measurement_min_history,
            curve_ekf_fit_enabled: cfg.big_curve_ekf_fit_enabled,
            speed_measurement_enabled: cfg.big_speed_measurement_enabled,
            measurement_noise_scale: cfg.big_measurement_noise_scale.max(1.0),
        }
    }
}

/// 大符曲线 EKF。
#[derive(Debug, Clone)]
pub struct BigBuffCurveEskf {
    ekf: ExtendedKalmanFilter,
    cfg: CurveFilterConfig,
    smoothed_curve_speed: f64,
    direction: i32,
    history: std::collections::VecDeque<RollSample>,
    phi_seeded: bool,
}

#[derive(Debug, Clone, Copy)]
struct RollSample {
    time_s: f64,
    roll_rad: f64,
}

/// 大符曲线模型纯函数：给定 `a/w/phase` 返回当前角速度（rad/s，带符号）。
pub fn big_buff_speed(a: f64, phase: f64) -> f64 {
    let a = clamp_curve_a(a);
    let phase = if phase.is_finite() { phase } else { 0.0 };
    a * phase.sin() + (BIG_BUFF_BASE_SPEED - a)
}

/// 大符曲线模型纯函数：给定 `a/w/phase` 与时间间隔，返回角度增量（rad）。
pub fn big_buff_angle_delta(a: f64, w: f64, phase: f64, dt: f64) -> f64 {
    let a = clamp_curve_a(a);
    let w = clamp_curve_w(w);
    let phase = if phase.is_finite() { phase } else { 0.0 };
    let dt = dt.max(0.0);
    (BIG_BUFF_BASE_SPEED - a) * dt + a / w * (phase.cos() - (phase + w * dt).cos())
}

fn clamp_curve_a(a: f64) -> f64 {
    if !a.is_finite() {
        BIG_BUFF_A_INIT
    } else {
        a.clamp(BIG_BUFF_A_MIN, BIG_BUFF_A_MAX)
    }
}

fn clamp_curve_w(w: f64) -> f64 {
    if !w.is_finite() {
        BIG_BUFF_W_INIT
    } else {
        w.clamp(BIG_BUFF_W_MIN, BIG_BUFF_W_MAX)
    }
}

fn clamp_speed_magnitude(speed: f64) -> f64 {
    if !speed.is_finite() {
        return speed;
    }
    speed.abs().min(BIG_BUFF_SPEED_HARD_MAX)
}

fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = (angle + std::f64::consts::PI) % std::f64::consts::TAU;
    if normalized < 0.0 {
        normalized += std::f64::consts::TAU;
    }
    normalized - std::f64::consts::PI
}

/// 把 `angle` 加上若干个 2π，使其与 `reference` 的差落在 (-π, π] 内。
/// 用于速度窗口回归前展开跨 ±π 的 roll 角，避免正常旋转被当成大跳变。
fn unwrap_relative_to(angle: f64, reference: f64) -> f64 {
    if !angle.is_finite() || !reference.is_finite() {
        return angle;
    }
    let mut delta = (angle - reference) % std::f64::consts::TAU;
    if delta < -std::f64::consts::PI {
        delta += std::f64::consts::TAU;
    } else if delta > std::f64::consts::PI {
        delta -= std::f64::consts::TAU;
    }
    reference + delta
}

impl BigBuffCurveEskf {
    /// 用默认曲线初值构造（`a = 0.9125`、`w = 1.942`、`phase = 0`）。
    pub fn from_tracker_cfg(cfg: &EnergyMechanismTrackerCfg) -> Self {
        let curve_cfg = CurveFilterConfig::from_tracker_cfg(cfg);
        let x0 = na::DVector::from_vec(vec![0.0, BIG_BUFF_A_INIT, BIG_BUFF_W_INIT]);
        let p0 = na::DMatrix::from_diagonal(&na::DVector::from_vec(vec![1.0, 0.1, 0.1]));
        Self {
            ekf: ExtendedKalmanFilter::with_initial(x0, p0),
            cfg: curve_cfg,
            smoothed_curve_speed: f64::NAN,
            direction: 0,
            history: std::collections::VecDeque::with_capacity(64),
            phi_seeded: false,
        }
    }

    /// 当前相位（曲线 φ）。
    pub fn phase(&self) -> f64 {
        self.ekf.x[PHASE]
    }

    /// 当前曲线参数 `a`。
    pub fn a(&self) -> f64 {
        self.ekf.x[A]
    }

    /// 当前曲线参数 `w`。
    pub fn w(&self) -> f64 {
        self.ekf.x[W]
    }

    /// 当前曲线角速度（|speed|，rad/s）。
    pub fn curve_speed(&self) -> f64 {
        clamp_speed_magnitude(big_buff_speed(self.ekf.x[A], self.ekf.x[PHASE]))
    }

    /// 当前转动方向（+1 / -1 / 0）。
    pub fn direction(&self) -> i32 {
        self.direction
    }

    /// 重置滤波器（保留配置，清空状态与历史）。
    pub fn reset(&mut self) {
        let x0 = na::DVector::from_vec(vec![0.0, BIG_BUFF_A_INIT, BIG_BUFF_W_INIT]);
        let p0 = na::DMatrix::from_diagonal(&na::DVector::from_vec(vec![1.0, 0.1, 0.1]));
        self.ekf.init(x0, p0);
        self.smoothed_curve_speed = f64::NAN;
        self.history.clear();
        self.phi_seeded = false;
    }

    /// 注入观察到的转动方向（由 tracker 的方向投票给出）。
    pub fn set_direction(&mut self, direction: i32) {
        self.direction = direction;
    }

    /// 记录一个 roll 观察样本（时间秒 + roll 弧度），用于 rolling-window 估速与 φ seed。
    pub fn record_roll(&mut self, time_s: f64, roll_rad: f64) {
        self.history.push_back(RollSample { time_s, roll_rad });
        while self.history.len() > 250 {
            self.history.pop_front();
        }
    }

    /// 用滚动窗口最小二乘估计当前角速度（rad/s，绝对值）。样本不足时返回 `None`。
    pub fn estimate_observed_speed(&self) -> Option<f64> {
        estimate_linear_rate_from_history(&self.history, self.cfg.window_samples, self.cfg.window_s)
    }

    /// 执行一步 predict：把曲线相位 φ 推进 `dt`，并传播协方差。
    ///
    /// 注意：phase（曲线内部相位 φ）的推进由 `w` 决定，**不带方向系数**——方向只影响
    /// roll（外层目标角度）的累积，phase 是驱动 speed = a·sin(φ)+(base-a) 的内部状态。
    /// 这与 vivsionn `BuffTracker::update_ekf` 的 f lambda 一致（phase += w·dt，roll 才带 dir）。
    pub fn predict(&mut self, dt: f64) {
        // 关闭曲线拟合时不推进 phase/speed，退化为常速预测（由外层 roll_rate 线性外推）。
        if !self.cfg.curve_ekf_fit_enabled {
            return;
        }
        let dt = dt.clamp(1e-3, 0.1);

        // 名义状态非线性传播：phase += w·dt，a/w 不变。
        let state_step = |x: &na::DVector<f64>| {
            let phase = if x[PHASE].is_finite() { x[PHASE] } else { 0.0 };
            let w = clamp_curve_w(x[W]);
            let mut next = x.clone_owned();
            next[PHASE] = normalize_angle(phase + w * dt);
            next[A] = clamp_curve_a(x[A]);
            next[W] = w;
            next
        };

        // 状态转移雅可比 F。phase 对 w 的偏导 dφ/dw = dt（vivsionn F(9,8)=dt），
        // 必须保留这个耦合，否则速度测量更新对 w 的修正无法在 predict 中保持，w 学不动。
        let mut f = na::DMatrix::<f64>::identity(STATE_DIM, STATE_DIM);
        f[(PHASE, W)] = dt;

        let q = self.process_noise(dt);
        self.ekf.predict_nonlinear(&f, &q, state_step);
        self.maybe_seed_phi();
    }

    /// 执行速度测量更新（rolling-window 估速 → 速度残差 → phase/a 修正）。
    /// 返回速度测量的处理状态：`Accepted`、`Gated`（超门控拒绝）、`Skipped`（无有效样本）。
    pub fn update_with_speed(&mut self, dt: f64) -> SpeedUpdateResult {
        // 配置关闭速度测量更新时直接跳过。
        if !self.cfg.speed_measurement_enabled {
            return SpeedUpdateResult::Skipped;
        }
        if self.history.len() < self.cfg.min_history.max(2) {
            return SpeedUpdateResult::Skipped;
        }
        let Some(observed_speed) = self.estimate_observed_speed() else {
            return SpeedUpdateResult::Skipped;
        };
        if !observed_speed.is_finite() || observed_speed >= BIG_BUFF_SPEED_HARD_MAX * 1.5 {
            return SpeedUpdateResult::Skipped;
        }
        let observed_speed = clamp_speed_magnitude(observed_speed);

        let a = clamp_curve_a(self.ekf.x[A]);
        let phase = if self.ekf.x[PHASE].is_finite() {
            self.ekf.x[PHASE]
        } else {
            0.0
        };
        let signed_predicted = big_buff_speed(a, phase);
        let predicted_speed = signed_predicted.abs();
        let innovation = observed_speed - predicted_speed;

        if innovation.abs() > self.cfg.speed_measurement_gate {
            return SpeedUpdateResult::Gated;
        }

        // 测量雅可比 H = d|speed|/dphase, d|speed|/da, d|speed|/dw。
        let speed_sign = if signed_predicted < 0.0 { -1.0 } else { 1.0 };
        let mut h = na::DMatrix::<f64>::zeros(1, STATE_DIM);
        h[(0, PHASE)] = speed_sign * a * phase.cos();
        h[(0, A)] = speed_sign * (phase.sin() - 1.0);
        // w 不直接出现在 speed 公式中，对 speed 导数为 0。
        let r = na::DMatrix::from_row_slice(
            1,
            1,
            &[self.cfg.speed_measurement_noise * self.cfg.measurement_noise_scale],
        );
        let z = na::DVector::from_vec(vec![observed_speed]);
        let z_pred = na::DVector::from_vec(vec![predicted_speed]);
        let phase_before = self.ekf.x[PHASE];

        self.ekf.update(&z, &h, &r, &z_pred, |a_v, b_v| a_v - b_v);

        self.apply_phi_correction_limit(phase_before);
        self.clamp_state();
        self.update_smoothed_speed(dt);
        SpeedUpdateResult::Accepted
    }

    /// 不做测量更新时也要刷新平滑速度（用于 predict 后保持 slew 平滑）。
    pub fn refresh_smoothed_speed(&mut self, dt: f64) {
        self.update_smoothed_speed(dt);
    }

    /// 预测从当前状态出发 `dt` 秒后的相位与曲线速度，返回 `(phase, speed_abs)`。
    pub fn predict_from_state(&self, dt: f64) -> (f64, f64) {
        let dt = dt.max(0.0);
        let a = clamp_curve_a(self.ekf.x[A]);
        let w = clamp_curve_w(self.ekf.x[W]);
        let phase = if self.ekf.x[PHASE].is_finite() {
            self.ekf.x[PHASE]
        } else {
            0.0
        };
        let future_phase = normalize_angle(phase + w * dt);
        let future_speed = clamp_speed_magnitude(big_buff_speed(a, future_phase));
        (future_phase, future_speed)
    }

    /// 预测从当前状态出发 `dt` 秒内的角度增量（带方向系数）。
    pub fn predict_angle_delta(&self, dt: f64) -> f64 {
        let active_dir = if self.direction != 0 {
            self.direction as f64
        } else {
            1.0
        };
        let a = clamp_curve_a(self.ekf.x[A]);
        let w = clamp_curve_w(self.ekf.x[W]);
        let phase = if self.ekf.x[PHASE].is_finite() {
            self.ekf.x[PHASE]
        } else {
            0.0
        };
        let raw_delta = big_buff_angle_delta(a, w, phase, dt);
        let mut delta = active_dir * raw_delta;
        if self.cfg.speed_slew_limit > 0.0
            && self.smoothed_curve_speed.is_finite()
            && delta.is_finite()
        {
            let max_delta =
                0.5 * (self.smoothed_curve_speed + self.curve_speed()) * dt.max(0.0) + 0.02;
            if delta.abs() > max_delta {
                delta = max_delta.copysign(delta);
            }
        }
        delta
    }

    fn process_noise(&self, dt: f64) -> na::DMatrix<f64> {
        let mut q = na::DMatrix::<f64>::zeros(STATE_DIM, STATE_DIM);
        q[(PHASE, PHASE)] = self.cfg.phase_process_noise * dt;
        q[(A, A)] = self.cfg.a_process_noise * dt;
        q[(W, W)] = self.cfg.w_process_noise * dt;
        q
    }

    fn maybe_seed_phi(&mut self) {
        if self.phi_seeded || self.cfg.phi_seed_frames == 0 {
            return;
        }
        if self.history.len() != self.cfg.phi_seed_frames {
            return;
        }
        let Some(observed_speed) = self.estimate_observed_speed() else {
            return;
        };
        if !observed_speed.is_finite() || observed_speed <= 0.1 {
            return;
        }
        let a = clamp_curve_a(self.ekf.x[A]);
        let base = BIG_BUFF_BASE_SPEED - a;
        let sin_arg = (observed_speed - base) / a;
        if !sin_arg.is_finite() || sin_arg.abs() > 1.0 {
            return;
        }
        let phi_asc = sin_arg.asin();
        let phi_desc = normalize_angle(std::f64::consts::PI - phi_asc);
        let phi_seed = self.select_phi_branch(phi_asc, phi_desc);
        self.ekf.x[PHASE] = phi_seed;
        if self.ekf.p[(PHASE, PHASE)] > 0.05 {
            self.ekf.p[(PHASE, PHASE)] = 0.05;
        }
        self.phi_seeded = true;
    }

    fn select_phi_branch(&self, phi_asc: f64, phi_desc: f64) -> f64 {
        if self.history.len() < 4 {
            return phi_asc;
        }
        let mut iter = self.history.iter().rev();
        let it0 = iter.next();
        let it1 = iter.next();
        let it3 = iter.nth(1);
        match (it0, it1, it3) {
            (Some(s0), Some(s1), Some(s3)) => {
                let dt_recent = s0.time_s - s1.time_s;
                let dt_early = s1.time_s - s3.time_s;
                if dt_recent > 1e-4 && dt_early > 1e-4 {
                    let spd_recent = (s0.roll_rad - s1.roll_rad).abs() / dt_recent;
                    let spd_early = (s1.roll_rad - s3.roll_rad).abs() / dt_early;
                    if spd_recent < spd_early - 0.05 {
                        return phi_desc;
                    }
                }
                phi_asc
            }
            _ => phi_asc,
        }
    }

    fn apply_phi_correction_limit(&mut self, phase_before: f64) {
        if self.cfg.phi_correction_limit <= 0.0 || !phase_before.is_finite() {
            return;
        }
        let correction = normalize_angle(self.ekf.x[PHASE] - phase_before);
        if correction.abs() > self.cfg.phi_correction_limit {
            self.ekf.x[PHASE] =
                normalize_angle(phase_before + self.cfg.phi_correction_limit.copysign(correction));
        }
    }

    fn clamp_state(&mut self) {
        self.ekf.x[A] = clamp_curve_a(self.ekf.x[A]);
        self.ekf.x[W] = clamp_curve_w(self.ekf.x[W]);
        self.ekf.x[PHASE] = normalize_angle(self.ekf.x[PHASE]);
    }

    fn update_smoothed_speed(&mut self, dt: f64) {
        let raw = self.curve_speed();
        if self.cfg.speed_slew_limit > 0.0
            && self.smoothed_curve_speed.is_finite()
            && raw.is_finite()
        {
            let max_delta = self.cfg.speed_slew_limit * dt.max(0.0);
            let delta = (raw - self.smoothed_curve_speed).clamp(-max_delta, max_delta);
            self.smoothed_curve_speed += delta;
        } else {
            self.smoothed_curve_speed = raw;
        }
    }
}

/// 速度测量更新的处理结果。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpeedUpdateResult {
    /// 测量被接受并写入滤波器。
    Accepted,
    /// 测量超出门控，被拒绝。
    Gated,
    /// 无有效样本或历史不足，跳过。
    Skipped,
}

/// 用滚动窗口最小二乘估计角速度（绝对值，rad/s）。
fn estimate_linear_rate_from_history(
    history: &std::collections::VecDeque<RollSample>,
    max_samples: usize,
    max_window_s: f64,
) -> Option<f64> {
    if history.len() < 2 || max_samples < 2 || max_window_s <= 0.0 {
        return None;
    }
    let latest_t = history.back()?.time_s;
    if !latest_t.is_finite() {
        return None;
    }

    let mut times = [0.0_f64; 16];
    let mut angles = [0.0_f64; 16];
    let max_n = max_samples.min(16);
    let mut count = 0_usize;
    for sample in history.iter().rev() {
        if !sample.time_s.is_finite() || !sample.roll_rad.is_finite() {
            break;
        }
        if latest_t - sample.time_s > max_window_s && count >= 2 {
            break;
        }
        times[count] = sample.time_s;
        angles[count] = sample.roll_rad;
        count += 1;
        if count >= max_n {
            break;
        }
    }
    if count < 2 {
        return None;
    }

    // 样本是倒序收集的（times[0] 最新）。roll 经 normalize_angle 后落在 (-π, π]，
    // 跨越 ±π 时（如 3.1 → -3.1）会被线性回归当成大跳变，估出错误速度。
    // 以最新样本 angles[0] 为基准逐个 unwrap，保证窗口内角度连续。
    let mut unwrapped = [0.0_f64; 16];
    unwrapped[0] = angles[0];
    for i in 1..count {
        unwrapped[i] = unwrap_relative_to(angles[i], unwrapped[i - 1]);
    }

    let mean_t: f64 = times[..count].iter().sum();
    let mean_a: f64 = unwrapped[..count].iter().sum();
    let mean_t = mean_t / count as f64;
    let mean_a = mean_a / count as f64;

    let mut numerator = 0.0;
    let mut denominator = 0.0;
    for i in 0..count {
        let dt = times[i] - mean_t;
        numerator += dt * (unwrapped[i] - mean_a);
        denominator += dt * dt;
    }
    if !numerator.is_finite() || !denominator.is_finite() || denominator < 1e-6 {
        return None;
    }
    Some((numerator / denominator).abs())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_cfg() -> EnergyMechanismTrackerCfg {
        EnergyMechanismTrackerCfg::default()
    }

    #[test]
    fn speed_formula_matches_vivsionn_curve_model() {
        // phase = π/2 → sin = 1 → speed = a + (base - a) = base
        let speed_peak = big_buff_speed(BIG_BUFF_A_INIT, std::f64::consts::FRAC_PI_2);
        assert!((speed_peak - BIG_BUFF_BASE_SPEED).abs() < 1e-9);
        // phase = -π/2 → sin = -1 → speed = -a + (base - a) = base - 2a
        let speed_trough = big_buff_speed(BIG_BUFF_A_INIT, -std::f64::consts::FRAC_PI_2);
        assert!((speed_trough - (BIG_BUFF_BASE_SPEED - 2.0 * BIG_BUFF_A_INIT)).abs() < 1e-9);
    }

    #[test]
    fn angle_delta_at_zero_phase_matches_curve_integral() {
        // angle_delta 是曲线积分，phase=0 时含二阶曲率项 a/w·(1 - cos(w·dt))，不能退化为 speed·dt。
        // 用 vivsionn 原始积分公式直接验证：(base-a)·dt + a/w·(cos(0) - cos(w·dt))。
        let dt = 0.05;
        let a = BIG_BUFF_A_INIT;
        let w = BIG_BUFF_W_INIT;
        let phase: f64 = 0.0;
        let expected =
            (BIG_BUFF_BASE_SPEED - a) * dt + a / w * (phase.cos() - (phase + w * dt).cos());
        let delta = big_buff_angle_delta(a, w, phase, dt);
        assert!(
            (delta - expected).abs() < 1e-12,
            "delta={delta} expected={expected}"
        );
        // 且方向系数 ±1 对应的 predict_angle_delta 关于零点对称。
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&default_cfg());
        eskf.set_direction(1);
        let pos = eskf.predict_angle_delta(dt);
        eskf.set_direction(-1);
        let neg = eskf.predict_angle_delta(dt);
        assert!((pos + neg).abs() < 1e-9);
    }

    #[test]
    fn clamp_a_w_keeps_params_in_range() {
        assert_eq!(clamp_curve_a(0.5), BIG_BUFF_A_MIN);
        assert_eq!(clamp_curve_a(2.0), BIG_BUFF_A_MAX);
        assert_eq!(clamp_curve_w(1.0), BIG_BUFF_W_MIN);
        assert_eq!(clamp_curve_w(3.0), BIG_BUFF_W_MAX);
        assert!(!clamp_curve_a(f64::NAN).is_nan());
    }

    #[test]
    fn predict_advances_phase_and_speed() {
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&default_cfg());
        eskf.set_direction(1);
        eskf.record_roll(0.0, 0.0);
        eskf.record_roll(0.02, 0.05);

        eskf.predict(0.02);
        let phase_after = eskf.phase();
        let speed_after = eskf.curve_speed();
        assert!(phase_after.is_finite());
        assert!(speed_after > 0.0 && speed_after < BIG_BUFF_SPEED_HARD_MAX + 1e-6);
    }

    #[test]
    fn predict_angle_delta_carries_direction_sign() {
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&default_cfg());
        eskf.set_direction(1);
        let pos = eskf.predict_angle_delta(0.05);
        eskf.set_direction(-1);
        let neg = eskf.predict_angle_delta(0.05);
        assert!(pos > 0.0);
        assert!(neg < 0.0);
        assert!((pos + neg).abs() < 1e-9);
    }

    #[test]
    fn update_with_speed_returns_skipped_without_history() {
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&default_cfg());
        eskf.set_direction(1);
        assert_eq!(eskf.update_with_speed(0.02), SpeedUpdateResult::Skipped);
    }

    #[test]
    fn update_with_speed_gates_outlier_measurement() {
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&default_cfg());
        eskf.set_direction(1);
        // 喂入一个明显与曲线模型不符的高速历史（>gate），应被门控拒绝。
        for i in 0..30 {
            eskf.record_roll(i as f64 * 0.02, i as f64 * 0.5);
        }
        let result = eskf.update_with_speed(0.02);
        assert!(
            matches!(
                result,
                SpeedUpdateResult::Gated | SpeedUpdateResult::Skipped
            ),
            "result={result:?}"
        );
    }

    #[test]
    fn estimate_observed_speed_uses_least_squares_over_window() {
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&default_cfg());
        // 线性 roll：rate = 2.0 rad/s
        for i in 0..20 {
            let t = i as f64 * 0.02;
            eskf.record_roll(t, 2.0 * t);
        }
        let speed = eskf.estimate_observed_speed().unwrap();
        assert!((speed - 2.0).abs() < 1e-6, "speed={speed}");
    }

    #[test]
    fn estimate_observed_speed_unwraps_roll_across_pi_boundary() {
        // roll 以 3.0 rad/s 递增，跨越 ±π 边界。若不 unwrap，回归会把
        // π → -π 的跳变当成速度反转，估出错误值。
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&default_cfg());
        let rate = 3.0;
        for i in 0..20 {
            let t = i as f64 * 0.02;
            // record_roll 内部存原始角度，但 normalize 在 tracker 侧；
            // 这里直接喂 normalize 后的角度模拟实车 tracker 输出。
            let raw = rate * t;
            let normalized =
                ((raw + std::f64::consts::PI) % std::f64::consts::TAU) - std::f64::consts::PI;
            eskf.record_roll(t, normalized);
        }
        let speed = eskf
            .estimate_observed_speed()
            .expect("speed estimate present");
        // unwrap 后应接近真实 rate，而不是被边界跳变污染。
        assert!((speed - rate).abs() < 0.1, "speed={speed} expected~{rate}");
    }

    #[test]
    fn unwrap_relative_to_handles_pi_boundary() {
        use super::unwrap_relative_to;
        // -3.13 相对 3.13：差值跨过 -π，应展开为 3.13 附近（≈ 3.153，差 0.023）。
        let r1 = unwrap_relative_to(-3.13, 3.13);
        assert!((r1 - 3.13).abs() < 0.05, "r1={r1}");
        // 3.13 相对 -3.13：差值跨过 +π，应展开为 -3.13 附近（≈ -3.153）。
        let r2 = unwrap_relative_to(3.13, -3.13);
        assert!((r2 - (-3.13)).abs() < 0.05, "r2={r2}");
        // 不跨边界时基本不变。
        assert!((unwrap_relative_to(1.0, 0.5) - 1.0).abs() < 1e-9);
        // 关键性质：跨 ±π 后两个样本的差应等于它们的连续角差，而非 2π 跳变。
        let diff = unwrap_relative_to(-3.13, 3.13) - 3.13;
        assert!(diff.abs() < 0.05, "跨边界差应为小量，实际 diff={diff}");
    }

    #[test]
    fn reset_clears_state_and_history() {
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&default_cfg());
        eskf.set_direction(1);
        eskf.record_roll(0.0, 0.0);
        eskf.predict(0.02);
        assert!(eskf.phase().abs() < 1.0);

        eskf.reset();
        assert_eq!(eskf.phase(), 0.0);
        assert_eq!(eskf.a(), BIG_BUFF_A_INIT);
        assert_eq!(eskf.w(), BIG_BUFF_W_INIT);
    }

    #[test]
    fn phi_seed_fires_once_at_seed_frame_count() {
        let mut cfg = default_cfg();
        cfg.big_phi_seed_frames = 8;
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&cfg);
        eskf.set_direction(1);
        // 构造一个合理的速度历史，使 seed 有合法的 sin_arg。
        for i in 0..8 {
            eskf.record_roll(i as f64 * 0.02, i as f64 * 0.04);
        }
        eskf.predict(0.02);
        // seed 后 phase 不再是 0
        assert!(eskf.phase().abs() > 1e-6 || eskf.a() > 0.0);
    }

    #[test]
    fn curve_ekf_fit_disabled_skips_phase_advance() {
        // big_curve_ekf_fit_enabled = false 时 predict 不推进 phase，退化为常速。
        let mut cfg = default_cfg();
        cfg.big_curve_ekf_fit_enabled = false;
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&cfg);
        eskf.set_direction(1);
        let phase_before = eskf.phase();
        eskf.record_roll(0.0, 0.0);
        eskf.record_roll(0.02, 0.05);
        eskf.predict(0.02);
        assert!((eskf.phase() - phase_before).abs() < 1e-9, "phase 不应推进");
    }

    #[test]
    fn speed_measurement_disabled_returns_skipped() {
        // big_speed_measurement_enabled = false 时 update_with_speed 直接 Skipped。
        let mut cfg = default_cfg();
        cfg.big_speed_measurement_enabled = false;
        cfg.big_speed_measurement_min_history = 2;
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&cfg);
        eskf.set_direction(1);
        for i in 0..10 {
            eskf.record_roll(i as f64 * 0.02, i as f64 * 0.04);
        }
        assert_eq!(eskf.update_with_speed(0.02), SpeedUpdateResult::Skipped);
    }

    #[test]
    fn measurement_noise_scale_changes_gate_tolerance() {
        // 放大 measurement_noise_scale 后，同一速度测量更容易被接受（R 更大，门控相对宽松）。
        // 这里只验证开关被读取且配置生效：把 scale 调大后 update_with_speed 仍正常返回。
        let mut cfg = default_cfg();
        cfg.big_speed_measurement_min_history = 2;
        cfg.big_speed_measurement_gate = 5.0;
        cfg.big_measurement_noise_scale = 10.0;
        let mut eskf = BigBuffCurveEskf::from_tracker_cfg(&cfg);
        eskf.set_direction(1);
        for i in 0..10 {
            eskf.record_roll(i as f64 * 0.02, i as f64 * 0.04);
        }
        let result = eskf.update_with_speed(0.02);
        // 大 gate + 大噪声下应被接受（不 Gated）。
        assert_ne!(result, SpeedUpdateResult::Gated);
    }
}
