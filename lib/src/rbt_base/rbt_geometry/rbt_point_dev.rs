// 目前暂时考虑尽量使用基础类型来表示数据，在方法中转换为 nalgebra 进行转换，并返回基础数据类型

use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
use std::cmp::PartialEq;
use tracing::{debug, error, warn};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RbtImgPoint2Coord {
    /// 原点在左上角，y向下，x向右
    Screen,
    /// 原点在中心，y向下，x向右
    Camera, // cam
}

#[derive(Debug, Clone, Copy)]
pub struct RbtImgPoint2 {
    pub x: f64,
    pub y: f64,
    coord: RbtImgPoint2Coord,
}

impl RbtImgPoint2 {
    pub fn new(x: f64, y: f64, coord: RbtImgPoint2Coord) -> Self {
        Self { x, y, coord }
    }

    pub fn get_coord(&self) -> (f64, f64) {
        (self.x, self.y)
    }

    pub fn img_to_cam(&mut self, cam_k: &na::SMatrix<f64, 3, 3>) -> RbtResult<Self> {
        if self.coord == RbtImgPoint2Coord::Camera {
            warn!("The coordinate system is already Camera");
            return Ok(*self);
        }
        // 使用内参归一化图像坐标系点到相机坐标系点
        let adjusted = cam_k.try_inverse().ok_or(RbtError::StringError(
            "Camera matrix is not invertible".into(),
        ))? * na::Vector3::new(self.x, self.y, 1.0);
        if adjusted.z.abs() < f64::EPSILON {
            error!("adjusted z is zero");
            return Err(RbtError::StringError("adjusted z is zero".into()));
        }
        self.x = adjusted.x / adjusted.z;
        self.y = adjusted.y / adjusted.z;
        self.coord = RbtImgPoint2Coord::Camera;
        debug!("The coord system has set to camera");
        Ok(*self)
    }

    pub fn to_na_point(&self) -> na::Point2<f64> {
        na::Point2::new(self.x, self.y)
    }
}
