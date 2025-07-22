// 目前暂时考虑尽量使用基础类型来表示数据，在方法中转换为 nalgebra 进行转换，并返回基础数据类型

use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use std::cmp::PartialEq;
use std::ops::Deref;
use tracing::{debug, error, warn};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RbtImgPoint2CoordSys {
    /// 原点在左上角，y向下，x向右
    ScreenPixel,
    /// 原点在中心，y向下，x向右
    CameraNorm, // cam
}

#[derive(Debug, Clone, Copy)]
pub struct RbtImgPoint2 {
    point: na::Point2<f64>,
    coord_sys: RbtImgPoint2CoordSys,
}

impl RbtImgPoint2 {
    pub fn new(x: f64, y: f64, coord_sys: RbtImgPoint2CoordSys) -> Self {
        Self {
            point: na::Point2::new(x, y),
            coord_sys,
        }
    }

    pub fn new_screen_pixel(x_f32: f32, y_f32: f32) -> Self {
        Self::new(x_f32 as f64, y_f32 as f64, RbtImgPoint2CoordSys::ScreenPixel)
    }

    pub fn from_point2(point: na::Point2<f64>, coord_sys: RbtImgPoint2CoordSys) -> Self {
        (point, coord_sys).into()
    }

    pub fn coord_sys(&self) -> &RbtImgPoint2CoordSys {
        &self.coord_sys
    }

    pub fn img_to_cam_mut(&mut self, cam_k: &na::SMatrix<f64, 3, 3>) -> RbtResult<()> {
        if self.coord_sys == RbtImgPoint2CoordSys::CameraNorm {
            warn!("The coordinate system is already Camera");
            return Ok(());
        }
        // 使用内参归一化图像坐标系点到相机坐标系点
        let adjusted = cam_k.try_inverse().ok_or(RbtError::StringError(
            "Camera matrix is not invertible".into(),
        ))? * na::Vector3::new(self.x, self.y, 1.0);
        if adjusted.z.abs() < f64::EPSILON {
            error!("adjusted z is zero");
            return Err(RbtError::StringError("adjusted z is zero".into()));
        }
        self.point.x = adjusted.x / adjusted.z;
        self.point.y = adjusted.y / adjusted.z;
        self.coord_sys = RbtImgPoint2CoordSys::CameraNorm;
        debug!("The coord system has set to camera");
        Ok(())
    }
}

impl Deref for RbtImgPoint2 {
    type Target = na::Point2<f64>;

    fn deref(&self) -> &Self::Target {
        &self.point
    }
}

impl From<RbtImgPoint2> for na::Point2<f64> {
    fn from(point: RbtImgPoint2) -> Self {
        point.point
    }
}

impl From<(na::Point2<f64>, RbtImgPoint2CoordSys)> for RbtImgPoint2 {
    fn from((point, coord_sys): (na::Point2<f64>, RbtImgPoint2CoordSys)) -> Self {
        RbtImgPoint2 { point, coord_sys }
    }
}
