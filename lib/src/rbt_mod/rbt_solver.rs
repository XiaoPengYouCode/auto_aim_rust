#![allow(unused)]

// 需要完成的事情，

// 1. 解算画面中每一块装甲板的位姿
// 2. 计算车体中心的坐标

// 坐标系定义，以当前车体的中心点的地面投影为原点

use crate::rbt_err::RbtError;
use crate::rbt_infra::rbt_cfg;
use crate::rbt_mod::rbt_armor::ArmorStaticMsg;
use nalgebra::Vector2;
use tracing::{debug, info};

pub mod rbt_orientation_solver;
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
}
