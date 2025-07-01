use na::{Unit, UnitQuaternion};
use tracing::error;

use crate::rbt_base::rbt_math::eskf::{Eskf, EskfDynamic};

trait RbtEularAngle<T> {
    fn euler_angle_ryp(&self) -> (T, T, T);
}

impl RbtEularAngle<f64> for na::UnitQuaternion<f64> {
    fn euler_angle_ryp(&self) -> (f64, f64, f64) {
        // 获取四元数分量
        let q = self.quaternion();
        let (w, x, y, z) = (q.coords[0], q.coords[1], q.coords[2], q.coords[3]);

        // 计算 Roll (X轴旋转)
        let roll = f64::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));

        // 计算 Yaw (Z轴旋转)
        let yaw = f64::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

        // 计算 Pitch (Y轴旋转)
        let sin_pitch = 2.0 * (w * y - z * x);
        let pitch = if sin_pitch.abs() >= 1.0 {
            f64::copysign(std::f64::consts::PI / 2.0, sin_pitch)
        } else {
            f64::asin(sin_pitch)
        };

        (roll, yaw, pitch)
    }
}

/// 自己云台的位姿
/// 根据真实情况，定义欧拉角顺序为: roll(来自底盘的roll, 忽略底盘的pitch和yaw，因为我没底盘观测，只能最多认为底盘有一个自由度为roll)->yaw(来自yaw轴转动)->pitch(来自pitch轴转动)
pub struct GimbalSolver {
    state: na::UnitQuaternion<f64>,
    solver: ImuDynamic,
    eskf: Eskf<6, 3>,
}

impl GimbalSolver {
    pub const ROLL: usize = 0;
    pub const PITCH: usize = 1;
    pub const YAW: usize = 2;
    pub fn new() -> Self {
        // 初始化位姿
        let init_state = na::UnitQuaternion::identity();
        // 初始化测量
        Self {
            state: init_state,
            solver: ImuDynamic {},
            eskf: Eskf::new(
                na::Matrix6::identity() * 0.001,
                na::Matrix6::identity() * 0.001,
                na::Matrix3::identity() * 0.001,
                0.01,
            ),
        }
    }

    pub fn solve(&mut self, solver: &ImuDynamic) {
        // measure
        // solve
        // inject state
    }
    pub fn roll(&self) -> f64 {
        self.state.euler_angles().0
    }
    pub fn yaw(&self) -> f64 {
        0.0
    }
    pub fn pitch(&self) -> f64 {
        9.9
    }
}

/// 自己云台位姿求解器
pub struct ImuDynamic {}

const STATIC_G: na::Vector3<f64> = na::Vector3::new(0.0, 0.0, -9.81);

impl EskfDynamic<6, 3> for ImuDynamic {
    type Input = na::Vector3<f64>;
    type NominalState = na::UnitQuaternion<f64>;
    type Measurement = na::Vector3<f64>;

    fn update_nominal_state(
        &self,
        nominal_state: &Self::NominalState,
        dt: f64,
        gyro: &Self::Input,
    ) -> Self::NominalState {
        let rotation_increment = na::UnitQuaternion::from_scaled_axis(gyro * dt);
        let nominal_q = nominal_state * rotation_increment;
        let nominal_q = na::UnitQuaternion::new_normalize(nominal_q.into_inner());
        nominal_q
    }

    fn state_transition_matrix_f(
        &self,
        nominal_state: &Self::NominalState,
        dt: f64,
        gyro: &Self::Input,
    ) -> na::SMatrix<f64, 6, 6> {
        let omega = skew_symmetric(&gyro);
        let mut f = na::Matrix6::zeros();
        let f00 = na::Matrix3::identity() - omega * dt;
        let upper_right = na::Matrix3::<f64>::identity() * dt;
        let f11 = na::Matrix3::<f64>::identity() - omega;
        f.fixed_view_mut::<3, 3>(0, 0).copy_from(&f00);
        f.fixed_view_mut::<3, 3>(0, 3).copy_from(&upper_right);
        f.fixed_view_mut::<3, 3>(3, 3).copy_from(&f11);
        f
    }

    fn measurement_matrix_h(&self, nominal_state: &Self::NominalState) -> na::SMatrix<f64, 3, 6> {
        let mut h = na::SMatrix::<f64, 3, 6>::zeros();
        let predicted_accel = nominal_state.inverse() * STATIC_G;
        let h11 = -skew_symmetric(&predicted_accel);
        h.fixed_view_mut::<3, 3>(0, 3).copy_from(&h11);
        h
    }

    fn measurement_residual_y(
        &self,
        nominal_state: &Self::NominalState,
        accel: &Self::Measurement,
    ) -> na::SVector<f64, 3> {
        let g = na::Vector3::new(0.0, 0.0, -9.81);
        let predicted_accel = nominal_state.inverse() * STATIC_G;
        let y = accel - predicted_accel;
        y
    }

    fn inject_error(
        &self,
        nominal_state: Self::NominalState,
        error_estimate: &na::SVector<f64, 6>,
    ) -> Self::NominalState {
        let error_angle = error_estimate.fixed_rows::<3>(3);
        let angle_nrom = error_angle.norm();
        if angle_nrom < 1e-6 {
            error!("Failed to inject error");
        }
        let corrention = UnitQuaternion::from_axis_angle(
            &na::Unit::new_unchecked(error_angle.normalize()),
            error_angle.norm(),
        );
        let new_nominal = nominal_state * corrention;
        let new_nominal = UnitQuaternion::new_normalize(new_nominal.into_inner());
        new_nominal
    }
}

fn skew_symmetric(v: &na::Vector3<f64>) -> na::Matrix3<f64> {
    na::Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}
