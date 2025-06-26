/// 相机内参矩阵 (3x3)
#[derive(Debug, Clone)]
pub struct RbtCamIntrinsics {
    matrix: na::Matrix3<f64>,     // 3x3 矩阵
    distortion: Option<Vec<f64>>, // 畸变参数 (e.g., k1, k2, p1, p2, k3)
}

impl RbtCamIntrinsics {
    /// [ fx, 0, cx;
    ///   0, fy, cy;
    ///   0, 0, 1 ]
    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self {
            matrix: na::Matrix3::new(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0),
            distortion: None,
        }
    }

    /// 投影 3D 点 (相机坐标系) 到 2D 像素坐标
    pub fn project(&self, point: &na::Point3<f64>) -> Option<na::Point2<f64>> {
        let v = self.matrix * point.coords;
        if v.z.abs() < std::f64::EPSILON {
            return None;
        }
        Some(na::Point2::new(v.x / v.z, v.y / v.z))
    }

    pub fn matrix(&self) -> &na::Matrix3<f64> {
        &self.matrix
    }

    pub fn distortion(&self) -> Option<&Vec<f64>> {
        self.distortion.as_ref()
    }
}

/// 相机外参 (旋转 + 平移)
#[derive(Debug, Clone)]
pub struct RbtCamExtrinsics {
    isometry: na::Isometry3<f64>, // 包含 Rotation3 + Translation3
}

impl RbtCamExtrinsics {
    /// 从旋转和平移构造
    pub fn new(rotation: na::Rotation3<f64>, translation: na::Translation3<f64>) -> Self {
        Self {
            isometry: na::Isometry3::from_parts(translation, rotation.into()),
        }
    }

    /// 将世界坐标系的点转换到相机坐标系
    pub fn world_to_camera(&self, point: &na::Point3<f64>) -> na::Point3<f64> {
        // 修正: 应该用 isometry.transform_point
        self.isometry.transform_point(point)
    }

    pub fn isometry(&self) -> &na::Isometry3<f64> {
        &self.isometry
    }
}
