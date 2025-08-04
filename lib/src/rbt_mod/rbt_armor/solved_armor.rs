use crate::rbt_base::rbt_geometry::rbt_pose3::{RbtPose3, RbtPoseCoordSys};
use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;
use na::Isometry3;
use std::ops::{Deref, DerefMut};

/// 该结构体一部分从 PNP 算法中计算得到
/// 一部分从 enemy 反解得到
#[derive(Debug, Clone)]
pub(crate) struct SolvedArmor {
    detected_armor: DetectedArmor,
    pose: RbtPose3, // 从 pnp 中得到
    enemy_yaw: f64,
    base_yaw: f64,
    radius: f64,
}

// 在 solved_armor.rs 中添加
impl SolvedArmor {
    pub fn new(
        detected_armor: DetectedArmor,
        pose: Isometry3<f64>,
        enemy_yaw: f64,
        base_yaw: f64,
        radius: f64,
    ) -> Self {
        SolvedArmor {
            detected_armor,
            pose: RbtPose3::new(pose, RbtPoseCoordSys::Camera),
            enemy_yaw,
            base_yaw,
            radius,
        }
    }

    pub fn update_measurement(&mut self, radius: f64) {
        self.radius = radius;
    }
}

impl Deref for SolvedArmor {
    type Target = DetectedArmor;

    fn deref(&self) -> &Self::Target {
        &self.detected_armor
    }
}

impl DerefMut for SolvedArmor {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.detected_armor
    }
}

impl SolvedArmor {
    pub fn pose(&self) -> &RbtPose3 {
        &self.pose
    }

    pub fn pose_mut(&mut self) -> &mut RbtPose3 {
        &mut self.pose
    }
}
