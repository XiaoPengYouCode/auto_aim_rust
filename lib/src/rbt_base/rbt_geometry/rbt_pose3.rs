use crate::rbt_base::rbt_algorithm::rbt_ippe::{ARMOR_LIGHT_HEIGHT, ARMOR_LIGHT_WEIGHT};
use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use na::{Isometry3, Vector3};
use tracing::error;

#[derive(PartialEq, Clone, Debug)]
pub enum RbtPoseCoordSys {
    Camera,
    BaseXyz,
    ArmorXyz,
    WorldXyz,
}

/// 只对应坐标轴变换，实际使用时还需要左乘 Pitch 轴角度
pub const CAMERA_AXES_TO_BODY_AXES_ROTATION: na::Rotation<f64, 3> =
    na::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
        0.0, 0.0, 1.0, // X_cam → Z_body
        -1.0, 0.0, 0.0, // Y_cam → -X_body
        0.0, -1.0, 0.0, // Z_cam → -Y_body
    ));

impl RbtPoseCoordSys {
    // 根据当前坐标系和目标坐标系，给出对应的位姿变换
    fn get_isometry(&self, target_coord: &Self) -> Isometry3<f64> {
        match (self, target_coord) {
            (Self::Camera, Self::BaseXyz) => {
                let translation = na::Translation3::new(0.0, 15.0, 430.0);
                // 把相机坐标轴（右-下-前）表示为在机体坐标系（前-左-上）中的方向
                let pitch_rad = 0_f64.to_radians();
                let pitch_rotation = na::Rotation3::from_euler_angles(0.0, pitch_rad, 0.0);
                let total_rotation = pitch_rotation * CAMERA_AXES_TO_BODY_AXES_ROTATION;
                let isometry = na::Isometry3::from_parts(
                    translation,
                    <na::UnitQuaternion<f64>>::from(total_rotation),
                );
                isometry
            }
            (Self::BaseXyz, Self::WorldXyz) => {
                // 假设Yaw为0，实际使用时应根据机器人当前朝向设置
                let yaw_rad = 0_f64.to_radians();
                let yaw_rotation = na::Rotation3::from_euler_angles(0.0, 0.0, yaw_rad);
                let isometry = na::Isometry3::from_parts(
                    na::Translation3::new(0.0, 0.0, 0.0),
                    <na::UnitQuaternion<f64>>::from(yaw_rotation),
                );
                isometry
            }
            (Self::WorldXyz, Self::BaseXyz) => {
                // 反向转换
                let yaw_rad = 0_f64.to_radians();
                let yaw_rotation = na::Rotation3::from_euler_angles(0.0, 0.0, -yaw_rad);
                let isometry = na::Isometry3::from_parts(
                    na::Translation3::new(0.0, 0.0, 0.0),
                    <na::UnitQuaternion<f64>>::from(yaw_rotation),
                );
                isometry
            }
            _ => {
                let isometry = Isometry3::new(na::Vector3::new(0.0, 0.0, 0.0), na::zero());
                isometry
            }
        }
    }
}

/// 三维空间中的机器人位姿表示
///
/// 该结构体封装了机器人在三维空间中的位置和姿态信息，使用 nalgebra 库的 Isometry3 类型
/// 来表示 SE(3) 变换（即同时包含平移和旋转的刚体变换）。同时维护了该位姿所处的坐标系信息。
///
/// # 字段
/// - `isometry`: Isometry3<f64> 类型，表示从参考坐标系到当前位姿的变换
/// - `coord_sys`: RbtPoseCoordSys 枚举，表示当前位姿所在的坐标系类型
#[derive(Clone, Debug)]
pub struct RbtPose3 {
    isometry: Isometry3<f64>,
    coord_sys: RbtPoseCoordSys,
}

impl std::ops::Deref for RbtPose3 {
    type Target = Isometry3<f64>;

    fn deref(&self) -> &Self::Target {
        &self.isometry
    }
}

impl RbtPose3 {
    pub fn new_camera(isometry3: Isometry3<f64>) -> Self {
        Self {
            isometry: isometry3,
            coord_sys: RbtPoseCoordSys::Camera,
        }
    }
    pub fn armor_visualize(&self, rec: &rr::RecordingStream, idx: usize) -> RbtResult<()> {
        let armor_translation_rr = [
            self.isometry.translation.vector.x as f32,
            self.isometry.translation.vector.y as f32,
            self.isometry.translation.vector.z as f32,
        ];
        let armor_rotation_q = &self.isometry.rotation;
        let armor_rotation_q_rr = [
            armor_rotation_q.i as f32,
            armor_rotation_q.j as f32,
            armor_rotation_q.k as f32,
            armor_rotation_q.w as f32,
        ];
        rec.log(
            format!("armor/{}", idx),
            &[
                &rerun::Boxes3D::from_half_sizes([(
                    ARMOR_LIGHT_WEIGHT as f32 / 2.0,
                    (ARMOR_LIGHT_HEIGHT as f32 + 30.0) / 2.0,
                    10.0,
                )])
                .with_fill_mode(rerun::FillMode::Solid)
                .with_colors([rr::Color::from_unmultiplied_rgba(20, 20, 240, 100)])
                    as &dyn rerun::AsComponents,
                &rerun::Transform3D::default()
                    .with_axis_length(200.0)
                    .with_translation(armor_translation_rr)
                    .with_rotation(rr::Rotation3D::Quaternion(
                        rr::components::RotationQuat::from(armor_rotation_q_rr),
                    )),
            ],
        )?;
        Ok(())
    }
}

impl RbtPose3 {
    pub fn new(isometry: Isometry3<f64>, coord: RbtPoseCoordSys) -> Self {
        Self {
            isometry,
            coord_sys: coord,
        }
    }

    /// for rerun Point3D type
    pub fn xyz_f32(&self) -> [f32; 3] {
        [
            self.isometry.translation.vector.x as f32,
            self.isometry.translation.vector.y as f32,
            self.isometry.translation.vector.z as f32,
        ]
    }

    pub fn coord_trans_mut(&mut self, target_coord: RbtPoseCoordSys) {
        let rigid = self.coord_sys.get_isometry(&target_coord);
        let new_isometry = rigid * self.isometry;
        self.isometry = new_isometry;
        self.coord_sys = target_coord;
    }

    /// 计算当前位姿的偏航角（Yaw）
    ///
    /// 该函数计算从机器人机体坐标系（BaseXyz）原点到目标点的偏航角。
    /// 偏航角定义为从X轴正方向逆时针旋转到目标点与原点连线的角度。
    ///
    /// # 参数
    /// - `yaw_bias`: 偏航角偏置，用于补偿系统误差，单位为度。
    ///
    /// # 返回值
    /// 返回计算得到的偏航角，单位为度。如果当前坐标系不是机体坐标系，则返回错误。
    ///
    /// # 错误
    /// 如果当前坐标系不是 `RbtPoseCoordSys::BaseXyz`，则返回 `RbtError::CalAngleDisUnderOtherCoord` 错误。
    pub fn cal_yaw(&self, yaw_bias: f64) -> RbtResult<f64> {
        if self.coord_sys != RbtPoseCoordSys::BaseXyz {
            error!("需要先将坐标系转换为 base 坐标系");
            return Err(RbtError::CalAngleDisUnderOtherCoord);
        }
        let yaw = (self.isometry.translation.vector.y / self.isometry.translation.vector.x)
            .atan()
            .to_degrees();
        Ok(yaw + yaw_bias)
    }

    /// 计算当前位姿在机体坐标系下的水平距离
    ///
    /// 该函数计算从机器人机体坐标系（BaseXyz）原点到目标点的水平距离。
    /// 距离定义为在X-Y平面上从原点到目标点的欧几里得距离。
    ///
    /// # 返回值
    /// 返回计算得到的水平距离，单位与坐标系单位一致。如果当前坐标系不是机体坐标系，则返回错误。
    ///
    /// # 错误
    /// 如果当前坐标系不是 `RbtPoseCoordSys::BaseXyz`，则返回 `RbtError::CalAngleDisUnderOtherCoord` 错误。
    pub fn cal_distance(&self) -> RbtResult<f64> {
        if self.coord_sys != RbtPoseCoordSys::BaseXyz {
            error!("需要先将坐标系转换为 base 坐标系");
            return Err(RbtError::CalAngleDisUnderOtherCoord);
        }
        Ok(
            (self.isometry.translation.vector.x * self.isometry.translation.vector.x
                + self.isometry.translation.vector.y * self.isometry.translation.vector.y)
                .sqrt(),
        )
    }

    // 提供对 translation 和 rotation 的访问方法
    pub fn translation(&self) -> &na::Translation3<f64> {
        &self.isometry.translation
    }

    pub fn rotation(&self) -> &na::UnitQuaternion<f64> {
        &self.isometry.rotation
    }
}

/// 实现从 RbtPose3 到 rerun::Transform3D 的转换
///
/// 该实现用于将机器人位姿数据转换为 rerun 可视化框架中的变换表示。
/// 转换包括平移和旋转两个部分：
/// - 平移：直接从 RbtPose3 的 translation 字段提取 x、y、z 分量并转换为 f32 类型
/// - 旋转：将 RbtPose3 的旋转矩阵转换为四元数表示，然后提取 i、j、k、w 分量并转换为 f32 类型
///
/// # 参数
/// - `pose`: &RbtPose3 类型引用，包含位置和旋转信息
///
/// # 返回值
/// 返回对应的 rerun::Transform3D 实例，可用于 rerun 可视化
impl From<&RbtPose3> for rerun::Transform3D {
    fn from(pose: &RbtPose3) -> Self {
        let translation = [
            pose.isometry.translation.vector.x as f32,
            pose.isometry.translation.vector.y as f32,
            pose.isometry.translation.vector.z as f32,
        ];
        let rotation = [
            pose.isometry.rotation.i as f32,
            pose.isometry.rotation.j as f32,
            pose.isometry.rotation.k as f32,
            pose.isometry.rotation.w as f32,
        ];

        rerun::Transform3D::default()
            .with_translation(translation)
            .with_rotation(rerun::Rotation3D::Quaternion(
                rerun::components::RotationQuat::from(rotation),
            ))
    }
}
