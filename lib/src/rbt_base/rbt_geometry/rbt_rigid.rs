/// todo!("要不要考虑分离 position 和 orientation")
pub struct RbtRigidPose {
    isometry: na::Isometry3<f64>,
}

impl RbtRigidPose {
    /// 创建一个新的 RbtRigidPose 实例
    pub fn new(position: na::Point3<f64>, orientation: na::UnitQuaternion<f64>) -> Self {
        Self {
            isometry: na::Isometry3::from_parts(
                na::Translation3::from(position),
                orientation.into(),
            ),
        }
    }

    /// 获取位置
    pub fn position(&self) -> &na::Translation3<f64> {
        &self.isometry.translation
    }

    /// 获取方向
    pub fn orientation(&self) -> &na::UnitQuaternion<f64> {
        &self.isometry.rotation
    }

    /// 将世界坐标系的点转换到刚体坐标系
    pub fn world_to_rigid(&self, point: &na::Point3<f64>) -> na::Point3<f64> {
        self.isometry.inverse().transform_point(point)
    }
}
