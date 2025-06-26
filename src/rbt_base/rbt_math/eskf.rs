pub struct Eskf {}

// use defmt::{debug, error};
// use na::{Matrix3, Matrix3x6, Matrix6, Matrix6x3, Unit, UnitQuaternion, Vector3, Vector6};
// use nalgebra as na;

// #[allow(unused)]
// pub struct ESKF {
//     pub nominal_q: UnitQuaternion<f32>, // 名义状态 四元数
//     pub error_estimate: Vector6<f32>,   // 误差状态 [角速度误差, 角度误差]
//     pub error_estimate_p: Matrix6<f32>, // 误差状态协方差矩阵
//     pub f: Matrix6<f32>,                // 状态转移矩阵
//     pub q: Matrix6<f32>,                // 过程噪声协方差矩阵
//     pub r: Matrix3<f32>,                // 传感器噪声协方差矩阵
//     pub kalman_gain: Matrix6x3<f32>,    // 卡尔曼增益
//     pub dt: f32,                        // 时间步长
// }

// impl ESKF {
//     pub fn init() -> Self {
//         let mut process_noise = Matrix6::<f32>::zeros();
//         // Gyro noise (typically higher than accel)
//         process_noise
//             .fixed_view_mut::<3, 3>(0, 0)
//             .copy_from(&(Matrix3::<f32>::identity() * 0.0001));
//         // Orientation error process noise (typically lower)
//         process_noise
//             .fixed_view_mut::<3, 3>(3, 3)
//             .copy_from(&(Matrix3::<f32>::identity() * 0.001));
//         let measurement_noise = Matrix3::<f32>::identity() * 0.01; // Adjust based on accelerometer quality
//         let default_estimate = Vector6::zeros();
//         let default_estimate_p = Matrix6::identity() * 0.001;

//         ESKF {
//             nominal_q: UnitQuaternion::identity(),
//             error_estimate: default_estimate,
//             error_estimate_p: default_estimate_p,
//             f: Matrix6::<f32>::identity(), // 初始化为单位矩阵
//             q: process_noise,
//             r: measurement_noise,
//             kalman_gain: Matrix6x3::<f32>::zeros(),
//             dt: 0.001,
//         }
//     }

//     pub fn predict(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>) {
//         // 使用陀螺仪数据更新名义状态
//         let rotation_increment = UnitQuaternion::from_scaled_axis(gyro * self.dt);
//         self.nominal_q = self.nominal_q * rotation_increment;
//         self.nominal_q = UnitQuaternion::new_normalize(self.nominal_q.into_inner());

//         // 更新状态转移矩阵 (F)
//         // 先计算角速度的反对称矩阵（不带 dt）
//         // 对于匀速旋转模型，F 可以使用 skew-symmetric 矩阵表示
//         let omega = skew_symmetric(&gyro);
//         let upper_right = Matrix3::<f32>::identity() * self.dt;
//         // 离散化: F = I - [ω×] * dt
//         self.f
//             .fixed_view_mut::<3, 3>(0, 0)
//             .copy_from(&(Matrix3::identity() - omega * self.dt));
//         self.f.fixed_view_mut::<3, 3>(0, 3).copy_from(&upper_right);
//         self.f
//             .fixed_view_mut::<3, 3>(3, 0)
//             .copy_from(&Matrix3::zeros());
//         self.f
//             .fixed_view_mut::<3, 3>(3, 3)
//             .copy_from(&(Matrix3::identity() - omega));

//         // 修改后：离散化过程噪声
//         let q_continuous = self.q; // 连续时间噪声协方差
//         let q_discrete = q_continuous * self.dt; // 离散化 Q = Q_continuous * dt
//         self.error_estimate_p = self.f * self.error_estimate_p * self.f.transpose() + q_discrete;

//         self.error_estimate_p = (self.error_estimate_p + self.error_estimate_p.transpose()) * 0.5;
//     }

//     pub fn update(&mut self, accel: Vector3<f32>) {
//         let g = Vector3::new(0.0, 0.0, -9.81); // 重力向量
//         let predicted_accel = self.nominal_q.inverse() * g;

//         // 测量残差 (3×1 向量)
//         let y = accel - predicted_accel;

//         // 构建测量矩阵 (将状态映射到测量空间)
//         let mut h = Matrix3x6::<f32>::zeros();
//         h.fixed_view_mut::<3, 3>(0, 3)
//             .copy_from(&-skew_symmetric(&predicted_accel));

//         // 计算创新协方差 (3×3 矩阵)
//         let s = h * self.error_estimate_p * h.transpose() + self.r;

//         let s_stability = s + Matrix3::identity() * 1e-6;

//         // 计算卡尔曼增益 (6×3 矩阵)
//         if let Some(s_inv) = s_stability.try_inverse() {
//             self.kalman_gain = self.error_estimate_p * h.transpose() * s_inv;

//             // 更新误差估计 (6×1 向量)
//             self.error_estimate += self.kalman_gain * y * 0.01;

//             // 使用误差状态更正名义状态
//             let error_angle = self.error_estimate.fixed_rows::<3>(3);
//             let angle_norm = error_angle.norm();
//             if angle_norm < 1e-6 {
//                 return;
//             }
//             let correction = UnitQuaternion::from_axis_angle(
//                 &Unit::new_unchecked(error_angle.normalize()),
//                 error_angle.norm(),
//             );
//             self.nominal_q = self.nominal_q * correction;
//             self.nominal_q = UnitQuaternion::new_normalize(self.nominal_q.into_inner());

//             // 使用 Joseph 形式更新协方差以提高数值稳定性
//             let i_kh = Matrix6::<f32>::identity() - self.kalman_gain * h;
//             self.error_estimate_p = i_kh * self.error_estimate_p * i_kh.transpose()
//                 + self.kalman_gain * self.r * self.kalman_gain.transpose();

//             self.error_estimate_p =
//                 (self.error_estimate_p + self.error_estimate_p.transpose()) * 0.5;

//             self.error_estimate.fill(0.0);
//         } else {
//             // 处理矩阵求逆失败的情况
//             error!("创新协方差矩阵求逆失败");
//         }
//     }

//     // 获取当前欧拉角(度)
//     pub fn get_euler_angles_d(&self) -> [f32; 3] {
//         let euler = self.nominal_q.euler_angles();
//         [
//             euler.0.to_degrees(),
//             euler.1.to_degrees(),
//             euler.2.to_degrees(),
//         ]
//     }

//     // 获取当前四元数
//     pub fn get_quaternion(&self) -> UnitQuaternion<f32> {
//         self.nominal_q
//     }

//     // 设置时间步长
//     pub fn set_dt(&mut self, dt: f32) {
//         self.dt = dt;
//     }

//     // 设置过程噪声协方差
//     pub fn set_process_noise(&mut self, q: Matrix6<f32>) {
//         self.q = q;
//     }

//     // 设置测量噪声协方差
//     pub fn set_measurement_noise(&mut self, r: Matrix3<f32>) {
//         self.r = r;
//     }

//     pub fn print_kalman_gain(&self) {
//         debug!("Kalman Gain: {:?}", self.kalman_gain.as_slice());
//     }

//     pub fn print_estimate_p(&self) {
//         debug!("estimate_p: {:?}", self.error_estimate_p.as_slice());
//     }
// }

// fn skew_symmetric(v: &Vector3<f32>) -> Matrix3<f32> {
//     Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
// }
