use crate::rbt_mod::rbt_detector::rbt_yolo::YoloLabel;
use crate::rbt_mod::rbt_estimator::rbt_enemy_model::{EnemyArmorType, EnemyFaction, EnemyId};
use std::fmt::Display;

/// detected 中会用到的独占部分
pub mod detected_armor;
/// solver 中会用到的独占部分
pub mod solved_armor;
/// estimator 中会用到的部分
pub mod tracked_armor;

pub type ArmorId = EnemyId;

// pub struct ArmorLabel(ArmorColor, ArmorId);
pub type ArmorLabel = YoloLabel;

impl ArmorLabel {
    fn armor_type(&self) -> ArmorType {
        if self.id() == &ArmorId::Hero1 {
            ArmorType::Large
        } else {
            ArmorType::Small
        }
    }
}

pub type ArmorColor = EnemyFaction;

/// ArmorType 是根据 ID 判断的，所以不放在 Label 里面
pub type ArmorType = EnemyArmorType;
impl ArmorType {
    /// 小装甲板灯条关键点尺寸，用于输入 pnp
    pub const SMALL_ARMOR_POINT3: [na::Point3<f64>; 4] = [
        na::Point3::new(-135.0 / 2.0, 55.0 / 2.0, 5.1),
        na::Point3::new(-135.0 / 2.0, -55.0 / 2.0, 5.2),
        na::Point3::new(135.0 / 2.0, -55.0 / 2.0, 5.3),
        na::Point3::new(135.0 / 2.0, 55.0 / 2.0, 5.4),
    ];

    /// 大装甲板灯条关键点尺寸，用于输入 pnp
    pub const LARGE_ARMOR_POINT3: [na::Point3<f64>; 4] = [
        na::Point3::new(-135.0 / 2.0, 85.0 / 2.0, 1e-6),
        na::Point3::new(-135.0 / 2.0, -85.0 / 2.0, 1e-6),
        na::Point3::new(135.0 / 2.0, -85.0 / 2.0, 1e-6),
        na::Point3::new(135.0 / 2.0, 85.0 / 2.0, 1e-6),
    ];

    pub fn armor_corner_points(&self) -> [na::Point3<f64>; 4] {
        match self {
            Self::Large => Self::LARGE_ARMOR_POINT3,
            Self::Small => Self::SMALL_ARMOR_POINT3,
        }
    }
}
