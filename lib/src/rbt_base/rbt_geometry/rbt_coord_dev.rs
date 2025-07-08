use crate::rbt_err::{RbtError, RbtResult};
use na::{Isometry3, Vector3};
use tracing::error;

#[derive(PartialEq)]
pub enum RbtPoseCoordSys {
    Camera,
    BaseXyz,
    ArmorXyz,
}

/// 只对应坐标轴变换，实际使用时还需要左乘 Pitch 轴角度
const CAMERA_AXES_TO_BODY_AXES_ROTATION: na::Rotation<f64, 3> =
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
                let pitch_rad = 5_f64.to_radians();
                let pitch_rotation = na::Rotation3::from_euler_angles(0.0, pitch_rad, 0.0);
                let total_rotation = pitch_rotation * CAMERA_AXES_TO_BODY_AXES_ROTATION;
                let isometry = na::Isometry3::from_parts(
                    translation,
                    <na::UnitQuaternion<f64>>::from(total_rotation),
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

pub struct RbtPose {
    pub translation: na::Point3<f64>, // 位置
    pub rotation: na::Rotation3<f64>, // 姿态
    coord_sys: RbtPoseCoordSys,
}

impl RbtPose {
    pub fn new(
        translation: na::Point3<f64>,
        rotation: na::Rotation3<f64>,
        coord: RbtPoseCoordSys,
    ) -> Self {
        Self {
            translation,
            rotation,
            coord_sys: coord,
        }
    }

    /// for rerun Point3D type
    pub fn xyz_f32(&self) -> [f32; 3] {
        [self.translation.x as f32, self.translation.y as f32, self.translation.z as f32]
    }

    pub fn coord_trans(&self, target_coord: RbtPoseCoordSys) -> Self {
        let rigid = self.coord_sys.get_isometry(&target_coord);
        let new_translation = rigid.transform_point(&self.translation);
        let new_rotation = rigid.rotation * self.rotation;
        Self {
            translation: new_translation,
            rotation: <na::Rotation3<f64>>::from(new_rotation),
            coord_sys: target_coord,
        }
    }

    pub fn cal_yaw(&self) -> RbtResult<f64> {
        if self.coord_sys != RbtPoseCoordSys::BaseXyz {
            error!("需要先将坐标系转换为 base 坐标系");
            return Err(RbtError::CalAngleDisUnderOtherCoord);
        }
        Ok((self.translation.y / self.translation.x).atan().to_degrees())
    }

    pub fn cal_distance(&self) -> RbtResult<f64> {
        if self.coord_sys != RbtPoseCoordSys::BaseXyz {
            error!("需要先将坐标系转换为 base 坐标系");
            return Err(RbtError::CalAngleDisUnderOtherCoord);
        }
        Ok(
            (self.translation.x * self.translation.x + self.translation.y * self.translation.y)
                .sqrt(),
        )
    }
}

pub struct RbtCylindricalCoord {
    pub r: f64,
    pub theta_d: f64,
    pub z: f64,
}

impl RbtCylindricalCoord {
    /// theta d means degree angle
    pub fn new(r: f64, theta_d: f64, z: f64) -> Self {
        Self { r, theta_d, z }
    }
}
