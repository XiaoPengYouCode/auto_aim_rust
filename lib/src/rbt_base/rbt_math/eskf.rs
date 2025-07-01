pub struct Eskf<const S_D: usize, const M_D: usize>
where
    na::Const<S_D>: na::DimName,
    na::Const<M_D>: na::DimName,
{
    pub error_estimate: na::SVector<f64, S_D>, // 误差状态
    pub error_estimate_p: na::SMatrix<f64, S_D, S_D>, // 误差状态协方差矩阵
    pub q: na::SMatrix<f64, S_D, S_D>,         // 过程噪声协方差矩阵
    pub r: na::SMatrix<f64, M_D, M_D>,         // 传感器噪声协方差矩阵
    pub dt: f64,                               // 时间步长
}

impl<const S_D: usize, const M_D: usize> Eskf<S_D, M_D>
where
    na::Const<S_D>: na::DimName,
    na::Const<M_D>: na::DimName,
{
    pub fn new(
        initial_p: na::SMatrix<f64, S_D, S_D>,
        q: na::SMatrix<f64, S_D, S_D>, // 过程噪声协方差矩阵
        r: na::SMatrix<f64, M_D, M_D>, // 传感器噪声协方差矩阵
        dt: f64,
    ) -> Self {
        Eskf {
            error_estimate: na::SVector::<f64, S_D>::zeros(),
            error_estimate_p: initial_p,
            q,
            r,
            dt,
        }
    }

    pub fn predict<M>(&mut self, model: &M, nominal_state: &M::NominalState, input: &M::Input)
    where
        M: EskfDynamic<S_D, M_D>,
    {
        let f = model.state_transition_matrix_f(nominal_state, self.dt, input);

        let q_discrete = self.q * self.dt;

        self.error_estimate_p = f * self.error_estimate_p * f.transpose() + q_discrete;

        self.error_estimate_p =
            (self.error_estimate_p + self.error_estimate_p.transpose()) * 0.5_f64;
    }

    pub fn update<M>(
        &mut self,
        model: &M,
        nominal_state: &mut M::NominalState,
        measurement: &M::Measurement,
    ) where
        M: EskfDynamic<S_D, M_D>,
    {
        let h = model.measurement_matrix_h(nominal_state);

        let s = h * self.error_estimate_p * h.transpose() + self.r;

        if let Some(s_inv) = s.try_inverse() {
            let kalman_gain = &self.error_estimate_p * h.transpose() * s_inv;

            let y = model.measurement_residual_y(nominal_state, measurement);

            self.error_estimate = kalman_gain * y;

            let i_kh = na::SMatrix::<f64, S_D, S_D>::identity() - kalman_gain * h;
            self.error_estimate_p = i_kh * self.error_estimate_p * i_kh.transpose()
                + kalman_gain * self.r * kalman_gain.transpose();

            self.error_estimate_p =
                (self.error_estimate_p + self.error_estimate_p.transpose()) * 0.5_f64;

            *nominal_state = model.inject_error(nominal_state.clone(), &self.error_estimate);
            self.error_estimate.fill(na::convert(0.0));
        } else {
            tracing::error!("Failed to solve s inverse")
        }
    }

    // 设置时间步长
    pub fn set_dt(&mut self, dt: f64) {
        self.dt = dt;
    }
}

/// all the Eskf instant need impl this trait
pub trait EskfDynamic<const S_D: usize, const M_D: usize> {
    // 定义模型可能需要的输入类型 (例如：陀螺仪和加速度计读数)
    type Input;
    // 定义模型可能产生的名义状态类型 (例如：四元数)
    type NominalState: Clone;
    // 定义测量值的类型
    type Measurement;

    /// 根据输入更新名义状态
    fn update_nominal_state(
        &self,
        nominal_state: &Self::NominalState,
        dt: f64,
        u: &Self::Input,
    ) -> Self::NominalState;

    /// 计算状态转移矩阵 F
    fn state_transition_matrix_f(
        &self,
        nominal_state: &Self::NominalState,
        dt: f64,
        u: &Self::Input,
    ) -> na::SMatrix<f64, S_D, S_D>;

    /// 计算测量矩阵 H
    fn measurement_matrix_h(
        &self,
        nominal_state: &Self::NominalState,
    ) -> na::SMatrix<f64, M_D, S_D>;

    /// 计算测量残差 y = z - h(x)
    fn measurement_residual_y(
        &self,
        nominal_state: &Self::NominalState,
        z: &Self::Measurement,
    ) -> na::SVector<f64, M_D>;

    /// 将计算出的误差状态注入回名义状态，并重置误差状态
    fn inject_error(
        &self,
        nominal_state: Self::NominalState,
        error_estimate: &na::SVector<f64, S_D>,
    ) -> Self::NominalState;
}
