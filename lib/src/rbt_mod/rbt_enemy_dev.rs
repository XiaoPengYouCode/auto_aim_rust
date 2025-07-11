use crate::rbt_base::rbt_game::EnemyFaction;
use crate::rbt_mod::rbt_armor::{ArmorId, ArmorType};

/// 描述地方装甲板大或者小
pub type EnemyArmorType = ArmorType;

#[derive(Debug)]
pub enum EnemyId {
    Hero1,
    Engineer2,
    Infantry3,
    Infantry4,
    Sentry7,
    Outpost8,
}

impl EnemyId {
    pub fn to_usize(&self) -> usize {
        match self {
            Self::Hero1 => 1_usize,
            Self::Engineer2 => 2_usize,
            Self::Infantry3 => 3_usize,
            Self::Infantry4 => 4_usize,
            Self::Sentry7 => 7_usize,
            Self::Outpost8 => 8_usize,
        }
    }
}

/// A_N 代表装甲板数量，其他兵种为 4, 前哨站为 3
pub struct Enemy<const A_N: usize> {
    // 装甲板类型（大小装甲板）
    armor_type: EnemyArmorType,
    armor_id: EnemyId,
    // 选择第一次看到该车的第一块装甲板为 idx = 0
    armor_rads_and_zs: [(f64, f64); A_N],
}

pub struct EnemyManager {
    enemy_faction: EnemyFaction,
    enemys: [EnemyDev; 6],
}

impl EnemyManager {
    pub fn new(enemy_faction: EnemyFaction) -> Self {
        Self {
            enemy_faction,
            enemys: [
                EnemyDev::Enemy4Armor {
                    enemy_id: EnemyId::Hero1,
                    armor_type: ArmorType::Large,
                    armor_rads_and_zs: [(200.0, 150.0); 4],
                },
                EnemyDev::Enemy4Armor {
                    enemy_id: EnemyId::Engineer2,
                    armor_type: ArmorType::Large,
                    armor_rads_and_zs: [(200.0, 150.0); 4],
                },
                EnemyDev::Enemy4Armor {
                    enemy_id: EnemyId::Infantry3,
                    armor_type: ArmorType::Large,
                    armor_rads_and_zs: [(200.0, 150.0); 4],
                },
                EnemyDev::Enemy4Armor {
                    enemy_id: EnemyId::Infantry4,
                    armor_type: ArmorType::Large,
                    armor_rads_and_zs: [(200.0, 150.0); 4],
                },
                EnemyDev::Enemy4Armor {
                    enemy_id: EnemyId::Sentry7,
                    armor_type: ArmorType::Large,
                    armor_rads_and_zs: [(200.0, 150.0); 4],
                },
                EnemyDev::Enemy3Armor {
                    enemy_id: EnemyId::Outpost8,
                    armor_type: ArmorType::Large,
                    armor_rad: 250.0,
                    armor_z: 150.0,
                },
            ],
        }
    }
}

pub enum EnemyDev {
    Enemy4Armor {
        enemy_id: EnemyId,
        // 装甲板类型（大小装甲板）
        armor_type: EnemyArmorType,
        // 选择第一次看到该车的第一块装甲板为 idx = 0
        armor_rads_and_zs: [(f64, f64); 4],
    },
    // 针对 outpost
    Enemy3Armor {
        armor_type: EnemyArmorType, // 大装甲板
        enemy_id: EnemyId,          // outpost id
        armor_rad: f64,
        armor_z: f64,
    },
}
