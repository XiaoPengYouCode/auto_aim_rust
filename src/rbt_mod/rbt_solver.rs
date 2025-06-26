#![allow(unused)]

// 需要完成的事情，

// 1. 解算画面中每一块装甲板的位姿
// 2. 计算车体中心的坐标

// 坐标系定义，以当前车体的中心点的地面投影为原点

use crate::rbt_err::RbtError;
use crate::rbt_infra::rbt_cfg;
use crate::rbt_mod::rbt_armor::ArmorStaticMsg;
use nalgebra::Vector2;
use rerun::Vec3D;
use tracing::{debug, info};

// mod rbt_sqpnp;

#[derive(Debug)]
enum RbtClass {
    // Hero,
    Engineer,
    // Infantry,
    // Sentry,
}

#[derive(Debug)]
pub struct RbtMsg {
    _class: RbtClass,
}

impl RbtMsg {
    fn new(class: RbtClass) -> RbtMsg {
        RbtMsg { _class: class }
    }
}

#[derive(Debug)]
pub struct RbtSolver {
    armors: Vec<ArmorStaticMsg>,
}

impl RbtSolver {
    // 根据 ArmorStaticMsg, 构建 RobotSolver
    pub fn new() -> Result<Self, RbtError> {
        Ok(RbtSolver { armors: vec![] })
    }

    // pub fn solve_pnp(
    //     &mut self,
    //     rbt_cfg: &rbt_cfg::CameraConfig,
    //     img_dbg: &Option<rerun::RecordingStream>,
    //     armors: &[ArmorStaticMsg],
    // ) -> Result<Vec<PnpResult>, RbtError> {
    //     let mut results = vec![];
    //     // pnp solver
    //     for armor in armors.iter() {
    //         let image_points = armor.corner_points_na();
    //         results.push(solve_pnp(rbt_cfg, image_points).unwrap()); // todo: convert argmin error to RbtError
    //     }

    //     debug!("PNP成功解算装甲板数量: {}", results.len());
    //     for (idx, result) in results.iter_mut().enumerate() {
    //         debug!(
    //             "pnp result {}: yaw = {}, translation = {:?}, cost = {}",
    //             idx,
    //             result.world_yaw().to_degrees() % 360.0,
    //             (result.x, result.y, result.z),
    //             result.cost
    //         );
    //     }

    //     Ok(results)
    // }
}
