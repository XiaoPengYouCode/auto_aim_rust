//! 动态大小扩展卡尔曼滤波器 (EKF)。
//!
//! 这是一个通用的、不定长 (dynamic-size) EKF 滤波核，直接维护名义状态 `x` 和协方差 `P`，
//! 提供 predict / update 标准步骤。
//!
//! 设计目标：让 armor (`YpdAngleTracker`) 和能量机关大符 (`BigBuffCurveEskf`) 复用同一份
//! 滤波数学。业务模块负责构造 `F`/`Q`/`H`/`R` 以及残差与非线性传播函数；本模块只承担
//! `x = F·x`、`P = F·P·Fᵀ + Q`、Joseph 形式协方差更新和 NIS 门控这些通用步骤。
//!
//! 与同目录 `rbt_eskf.rs` 的关系：那是误差状态 (ESKF) 形式，目前是占位代码；本模块是直接形式
//! EKF，与 vivsionn `BuffTracker` / `TongjiTracker` 的实现风格一致。两者并存，互不依赖。

/// 动态大小直接形式 EKF。
#[derive(Debug, Clone)]
pub struct ExtendedKalmanFilter {
    /// 名义状态 (nominal state)。
    pub x: na::DVector<f64>,
    /// 状态协方差矩阵。
    pub p: na::DMatrix<f64>,
    initialized: bool,
}

impl ExtendedKalmanFilter {
    /// 构造一个未初始化、维度为 `n` 的滤波器，状态与协方差归零。
    pub fn new(n: usize) -> Self {
        Self {
            x: na::DVector::zeros(n),
            p: na::DMatrix::zeros(n, n),
            initialized: false,
        }
    }

    /// 用初始状态与协方差初始化（或重置）滤波器。
    pub fn init(&mut self, x0: na::DVector<f64>, p0: na::DMatrix<f64>) {
        self.x = x0;
        self.p = p0;
        self.initialized = true;
    }

    /// 用初始状态与协方差构造并初始化滤波器。
    pub fn with_initial(x0: na::DVector<f64>, p0: na::DMatrix<f64>) -> Self {
        Self {
            x: x0,
            p: p0,
            initialized: true,
        }
    }

    /// 是否已初始化。
    pub fn initialized(&self) -> bool {
        self.initialized
    }

    /// 线性 predict：`x = F·x`，`P = F·P·Fᵀ + Q`。
    pub fn predict(&mut self, f: &na::DMatrix<f64>, q: &na::DMatrix<f64>) {
        self.x = f * &self.x;
        self.p = symmetrize(&(f * &self.p * f.transpose() + q));
    }

    /// 非线性名义传播 predict：先调用 `f` 把名义状态非线性推进一步，再用线性化的 `F` 传播协方差。
    /// `f` 接收当前 `x` 并返回推进后的 `x`；`F` 是该步对应的状态转移雅可比矩阵。
    pub fn predict_nonlinear(
        &mut self,
        f: &na::DMatrix<f64>,
        q: &na::DMatrix<f64>,
        state_step: impl Fn(&na::DVector<f64>) -> na::DVector<f64>,
    ) {
        self.x = state_step(&self.x);
        self.p = symmetrize(&(f * &self.p * f.transpose() + q));
    }

    /// 标准 EKF 更新。返回 `(是否接受, 残差 NIS)`。
    ///
    /// - `z`: 测量向量
    /// - `h`: 当前线性化点处的测量雅可比
    /// - `r`: 测量噪声协方差
    /// - `z_pred`: 当前状态下预测的测量 `h(x)`
    /// - `residual_fn`: 计算残差 `z - z_pred`，用于处理角度等需要归一化的分量
    ///
    /// `S` 奇异时返回 `(false, +inf)` 且不修改状态。协方差采用 Joseph 形式
    /// `P = (I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ` 以保证对称正定。
    pub fn update(
        &mut self,
        z: &na::DVector<f64>,
        h: &na::DMatrix<f64>,
        r: &na::DMatrix<f64>,
        z_pred: &na::DVector<f64>,
        residual_fn: impl Fn(&na::DVector<f64>, &na::DVector<f64>) -> na::DVector<f64>,
    ) -> (bool, f64) {
        let residual = residual_fn(z, z_pred);
        let s = h * &self.p * h.transpose() + r;
        let Some(s_inv) = s.clone().try_inverse() else {
            return (false, f64::INFINITY);
        };
        let nis = (residual.transpose() * &s_inv * &residual)[(0, 0)];

        let k = &self.p * h.transpose() * &s_inv;
        let i = na::DMatrix::<f64>::identity(self.p.nrows(), self.p.ncols());
        self.x += &k * &residual;
        let i_kh = &i - &k * h;
        self.p = symmetrize(&(&i_kh * &self.p * i_kh.transpose() + &k * r * k.transpose()));
        (true, nis)
    }

    /// 仅计算更新会产生的 NIS，但不修改状态。用于门控判断。
    /// `S` 奇异时返回 `+inf`。
    pub fn nis(
        &self,
        z: &na::DVector<f64>,
        h: &na::DMatrix<f64>,
        r: &na::DMatrix<f64>,
        z_pred: &na::DVector<f64>,
        residual_fn: impl Fn(&na::DVector<f64>, &na::DVector<f64>) -> na::DVector<f64>,
    ) -> f64 {
        let residual = residual_fn(z, z_pred);
        let s = h * &self.p * h.transpose() + r;
        let Some(s_inv) = s.try_inverse() else {
            return f64::INFINITY;
        };
        (residual.transpose() * s_inv * &residual)[(0, 0)]
    }
}

/// 对称化协方差矩阵：`(M + Mᵀ) / 2`。
fn symmetrize(matrix: &na::DMatrix<f64>) -> na::DMatrix<f64> {
    (matrix + matrix.transpose()) * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1D 常速模型：状态 `[pos, vel]`，恒定速度观测，滤波器应收敛到真实速度与位置。
    #[test]
    fn constant_velocity_model_converges() {
        let mut ekf = ExtendedKalmanFilter::with_initial(
            na::DVector::from_vec(vec![0.0, 0.0]),
            na::DMatrix::identity(2, 2) * 10.0,
        );

        let f = na::DMatrix::from_row_slice(2, 2, &[1.0, 0.1, 0.0, 1.0]);
        let q = na::DMatrix::identity(2, 2) * 1e-3;
        // 只观测位置
        let h = na::DMatrix::from_row_slice(1, 2, &[1.0, 0.0]);
        let r = na::DMatrix::from_row_slice(1, 1, &[0.1]);

        let true_vel = 2.0;
        for step in 0..40 {
            ekf.predict(&f, &q);
            let true_pos = (step as f64 + 1.0) * 0.1 * true_vel;
            let z = na::DVector::from_vec(vec![true_pos]);
            let z_pred = na::DVector::from_vec(vec![ekf.x[0]]);
            ekf.update(&z, &h, &r, &z_pred, |a, b| a - b);
        }

        assert!(
            (ekf.x[0] - 40.0 * 0.1 * true_vel).abs() < 1.0,
            "pos converged"
        );
        assert!((ekf.x[1] - true_vel).abs() < 0.5, "vel converged");
    }

    /// `S` 奇异（零测量噪声且 `H` 行退化）时 update 不修改状态并返回拒绝。
    #[test]
    fn update_rejects_when_s_is_singular() {
        let mut ekf = ExtendedKalmanFilter::with_initial(
            na::DVector::from_vec(vec![1.0]),
            na::DMatrix::identity(1, 1),
        );
        // H = 0 行导致 S = 0，奇异不可逆
        let h = na::DMatrix::from_row_slice(1, 1, &[0.0]);
        let r = na::DMatrix::from_row_slice(1, 1, &[0.0]);
        let z = na::DVector::from_vec(vec![5.0]);
        let z_pred = na::DVector::from_vec(vec![1.0]);

        let x_before = ekf.x[0];
        let (accepted, nis) = ekf.update(&z, &h, &r, &z_pred, |a, _b| a.clone());
        assert!(!accepted);
        assert!(nis.is_infinite());
        assert_eq!(ekf.x[0], x_before);
    }

    /// Joseph 形式协方差更新后 P 保持对称。
    #[test]
    fn covariance_stays_symmetric_after_update() {
        let mut ekf = ExtendedKalmanFilter::with_initial(
            na::DVector::from_vec(vec![0.0, 0.0]),
            na::DMatrix::identity(2, 2) * 5.0,
        );
        let h = na::DMatrix::from_row_slice(1, 2, &[1.0, 0.0]);
        let r = na::DMatrix::from_row_slice(1, 1, &[0.5]);
        let z = na::DVector::from_vec(vec![1.0]);
        let z_pred = na::DVector::from_vec(vec![0.0]);
        ekf.update(&z, &h, &r, &z_pred, |a, b| a - b);

        let diff = &ekf.p - ekf.p.transpose();
        let max_asym = diff.abs().max();
        assert!(max_asym < 1e-12, "P symmetric, max asym = {max_asym}");
    }

    /// NIS 门控：离群测量产生远大于正常测量的 NIS，可用 `nis()` 预判再决定是否 update。
    #[test]
    fn nis_distinguishes_inlier_from_outlier() {
        let ekf = ExtendedKalmanFilter::with_initial(
            na::DVector::from_vec(vec![0.0]),
            na::DMatrix::identity(1, 1),
        );
        let h = na::DMatrix::from_row_slice(1, 1, &[1.0]);
        let r = na::DMatrix::from_row_slice(1, 1, &[0.1]);
        let z_pred = na::DVector::from_vec(vec![0.0]);

        let inlier = na::DVector::from_vec(vec![0.05]);
        let outlier = na::DVector::from_vec(vec![50.0]);
        let nis_in = ekf.nis(&inlier, &h, &r, &z_pred, |a, b| a - b);
        let nis_out = ekf.nis(&outlier, &h, &r, &z_pred, |a, b| a - b);

        assert!(nis_in < 1.0, "inlier nis small: {nis_in}");
        assert!(nis_out > 100.0, "outlier nis large: {nis_out}");
    }
}
