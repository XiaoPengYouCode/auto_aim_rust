use crate::rbt_base::rbt_game::EnemyFaction;
use crate::rbt_mod::rbt_solver::RbtSolver;
use std::collections::HashMap;
use strum::EnumCount;

/// 描述敌方装甲板大或者小
pub enum EnemyArmorType {
    Small,
    Large,
}

#[derive(Debug, Eq, PartialEq, Hash, EnumCount)]
pub enum EnemyId {
    Hero1,
    Engineer2,
    Infantry3,
    Infantry4,
    Sentry7,
    Outpost8,
}

/// 描述装甲板的物理布局
#[derive(Debug, Clone, PartialEq)]
pub enum ArmorLayout {
    // 适用于大多数车辆的对称4装甲板布局
    Symmetric4 {
        layout: [(f64, f64); 4],
    },
    // 适用于前哨站的3装甲板布局
    Tripod3 {
        // 对于前哨站这种3个装甲板位置相同的特殊情况，可以简化
        // 如果3个位置也不同，则使用 positions: [(f64, f64); 3]
        radius: f64,
        height: f64,
    },
}

/// A_N 代表装甲板数量，其他兵种为 4, 前哨站为 3
pub struct Enemy {
    // 装甲板类型（大小装甲板）
    armor_type: EnemyArmorType,
    armor_id: EnemyId,
    // 选择第一次看到该车的第一块装甲板为 idx = 0
    armor_layout: ArmorLayout,
    enemy_solver: RbtSolver,
}

pub struct EnemyDatabase {
    enemy_faction: EnemyFaction, // 敌方阵营
    enemys: HashMap<EnemyId, Enemy>,
}

impl EnemyDatabase {
    pub fn new(enemy_faction: EnemyFaction) -> Self {
        let mut enemy_database = Self {
            enemy_faction,
            enemys: HashMap::new(),
        };
        enemy_database.enemys.insert(
            EnemyId::Hero1,
            Enemy {
                armor_type: EnemyArmorType::Large,
                armor_id: EnemyId::Hero1,
                armor_layout: ArmorLayout::Symmetric4 {
                    layout: [(200.0, 150.0); 4],
                },
                enemy_solver: RbtSolver::new().unwrap(),
            },
        );
        enemy_database.enemys.insert(
            EnemyId::Engineer2,
            Enemy {
                armor_type: EnemyArmorType::Small,
                armor_id: EnemyId::Engineer2,
                armor_layout: ArmorLayout::Symmetric4 {
                    layout: [(200.0, 150.0); 4],
                },
                enemy_solver: RbtSolver::new().unwrap(),
            },
        );
        enemy_database.enemys.insert(
            EnemyId::Infantry3,
            Enemy {
                armor_type: EnemyArmorType::Small,
                armor_id: EnemyId::Infantry3,
                armor_layout: ArmorLayout::Symmetric4 {
                    layout: [(200.0, 150.0); 4],
                },
                enemy_solver: RbtSolver::new().unwrap(),
            },
        );
        enemy_database.enemys.insert(
            EnemyId::Infantry4,
            Enemy {
                armor_type: EnemyArmorType::Small,
                armor_id: EnemyId::Infantry4,
                armor_layout: ArmorLayout::Symmetric4 {
                    layout: [(200.0, 150.0); 4],
                },
                enemy_solver: RbtSolver::new().unwrap(),
            },
        );
        enemy_database.enemys.insert(
            EnemyId::Sentry7,
            Enemy {
                armor_type: EnemyArmorType::Small,
                armor_id: EnemyId::Sentry7,
                armor_layout: ArmorLayout::Symmetric4 {
                    layout: [(200.0, 150.0); 4],
                },
                enemy_solver: RbtSolver::new().unwrap(),
            },
        );
        enemy_database.enemys.insert(
            EnemyId::Outpost8,
            Enemy {
                armor_type: EnemyArmorType::Small,
                armor_id: EnemyId::Outpost8,
                armor_layout: ArmorLayout::Tripod3 {
                    radius: 200.0,
                    height: 100.0,
                },
                enemy_solver: RbtSolver::new().unwrap(),
            },
        );
        enemy_database
    }

    /// 根据ID安全地获取敌人蓝图
    pub fn get(&self, id: &EnemyId) -> Option<&Enemy> {
        self.enemys.get(id)
    }

    /// 获取所有蓝图的迭代器
    pub fn iter(&self) -> impl Iterator<Item = (&EnemyId, &Enemy)> {
        self.enemys.iter()
    }
}
