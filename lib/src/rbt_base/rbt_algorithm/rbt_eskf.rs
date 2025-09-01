//! 扩展卡尔曼滤波器(ESKF)实现模块
//!
//! 该模块提供了扩展卡尔曼滤波器的核心实现，用于处理非线性系统的状态估计问题。
//! ESKF通过将非线性系统在名义状态点线性化来近似处理非线性问题。
//!
//! 核心概念：
//! - 标称状态(Nominal State): 系统状态的主要部分，通常用非线性方式传播
//! - 误差状态(Error State): 标称状态的小扰动，用线性方式传播
//! - 状态转移矩阵: 描述误差状态如何随时间演化
//! - 测量矩阵: 描述测量值与误差状态之间的关系
//!
//! 主要组件：
//! - StrategyESKF: ESKF滤波器数学原理实现
//! - StrategyESKFDynamicModel: 动态模型接口，需要用户实现具体的系统模型

use crate::rbt_mod::rbt_estimator::rbt_estimator_state::EstimatorStateMachine;

#[derive(Debug, Clone)]
pub struct ESKF<const S_D: usize, const M_D: usize>
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

impl<const S_D: usize, const M_D: usize> ESKF<S_D, M_D>
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
        ESKF {
            error_estimate: na::SVector::<f64, S_D>::zeros(),
            error_estimate_p: initial_p,
            q,
            r,
            dt,
        }
    }

    pub fn predict<M>(
        &mut self,
        model: &M,
        nominal_state: &M::NominalState,
        input: &M::Input,
        strategy: &M::Strategy,
    ) where
        M: StrategyDynamicModel<S_D, M_D>,
    {
        // 获取状态转移矩阵 F
        let f = model.state_transition_matrix_f(nominal_state, self.dt, input, strategy);
        // 获取过程噪声协方差矩阵 Q
        let q_discrete = self.q * self.dt;
        // 更新误差状态协方差矩阵，并执行对角线归一化
        self.error_estimate_p = f * self.error_estimate_p * f.transpose() + q_discrete;
        self.error_estimate_p =
            (self.error_estimate_p + self.error_estimate_p.transpose()) * 0.5_f64;
    }

    pub fn update<M>(
        &mut self,
        model: &M,
        nominal_state: &mut M::NominalState,
        measurement: &M::Measurement,
        strategy: &M::Strategy,
    ) where
        M: StrategyDynamicModel<S_D, M_D>,
    {
        // 获得测量矩阵 H
        let h = model.measurement_matrix_h(nominal_state, strategy);
        // 获得测量协方差矩阵 S
        let s = h * self.error_estimate_p * h.transpose() + self.r;
        if let Some(s_inv) = s.try_inverse() {
            // 计算卡尔曼增益
            let kalman_gain = &self.error_estimate_p * h.transpose() * s_inv;
            // 计算测量残差 y
            let y = model.measurement_residual_y(nominal_state, measurement, strategy);
            // 更新误差状态
            self.error_estimate = kalman_gain * y;
            // 更新误差状态协方差矩阵
            let i_kh = na::SMatrix::<f64, S_D, S_D>::identity() - kalman_gain * h;
            self.error_estimate_p = i_kh * self.error_estimate_p * i_kh.transpose()
                + kalman_gain * self.r * kalman_gain.transpose();
            self.error_estimate_p =
                (self.error_estimate_p + self.error_estimate_p.transpose()) * 0.5_f64;
            // 将计算出的误差状态注入回名义状态，并重置误差状态
            model.inject_error(nominal_state, &self.error_estimate, strategy);
            self.error_estimate.fill(na::convert(0.0));
        } else {
            tracing::error!("Failed to solve s inverse")
        }
    }

    // 设置时间步长
    pub fn set_dt(&mut self, dt: f64) {
        self.dt = dt;
    }

    // 设置过程噪声
    pub fn set_q(&mut self, q: na::SMatrix<f64, S_D, S_D>) {
        self.q = q;
    }

    // 设置测量噪声
    pub fn set_r(&mut self, r: na::SMatrix<f64, M_D, M_D>) {
        self.r = r;
    }
}

/// 所有策略动力学模型都需要实现此接口
pub trait StrategyDynamicModel<const S_D: usize, const M_D: usize> {
    // 定义模型可能需要的输入类型 (例如：陀螺仪和加速度计读数)
    type Input: AsRef<[f64]> + From<[f64; S_D]>;
    // 定义模型可能产生的名义状态类型 (例如：四元数)
    type NominalState: Clone;
    // 定义测量值的类型
    type Measurement: AsRef<[f64]> + From<[f64; M_D]>;
    type Strategy;

    /// 根据输入更新名义状态
    fn update_nominal_state(
        &self,
        nominal_state: &mut Self::NominalState, // 需要传入系统的状态变量可变引用
        dt: f64,
        u: &Self::Input,
        strategy: &Self::Strategy,
    );

    /// 计算状态转移矩阵 F
    fn state_transition_matrix_f(
        &self,
        nominal_state: &Self::NominalState,
        dt: f64,
        u: &Self::Input,
        strategy: &Self::Strategy,
    ) -> na::SMatrix<f64, S_D, S_D>;

    /// 计算测量矩阵 H
    fn measurement_matrix_h(
        &self,
        nominal_state: &Self::NominalState,
        strategy: &Self::Strategy,
    ) -> na::SMatrix<f64, M_D, S_D>;

    /// 计算测量残差 y = z - h(x)
    fn measurement_residual_y(
        &self,
        nominal_state: &Self::NominalState,
        z: &Self::Measurement,
        strategy: &Self::Strategy,
    ) -> na::SVector<f64, M_D>;

    /// 将计算出的误差状态注入回名义状态，并重置误差状态
    fn inject_error(
        &self,
        nominal_state: &mut Self::NominalState,
        error_estimate: &na::SVector<f64, S_D>,
        strategy: &Self::Strategy,
    );
}
