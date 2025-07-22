use crate::rbt_mod::rbt_solver::RbtSolver;
use std::collections::HashMap;
use std::fmt::Display;

/// 描述敌方装甲板大或者小
pub enum EnemyArmorType {
    Small,
    Large,
}

/// 用于描述装甲板/敌方车辆的唯一标记型 ID
#[derive(Clone, Copy, Eq, PartialEq, Hash)]
pub enum EnemyId {
    Hero1,
    Engineer2,
    Infantry3,
    Infantry4,
    Sentry7,
    Outpost8,
    Invalid,
}

impl Display for EnemyId {
    /// 手写反射哈哈哈
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EnemyId::Hero1 => write!(f, "Hero 1"),
            EnemyId::Engineer2 => write!(f, "Engineer 2"),
            EnemyId::Infantry3 => write!(f, "Infantry3"),
            EnemyId::Infantry4 => write!(f, "Infantry4"),
            EnemyId::Sentry7 => write!(f, "Sentry 7"),
            EnemyId::Outpost8 => write!(f, "Outpost 8"),
            EnemyId::Invalid => write!(f, "Invalid"),
        }
    }
}

/// 描述敌方阵营
#[derive(PartialEq, Eq)]
pub enum EnemyFaction {
    R,
    B,
}

impl Display for EnemyFaction {
    /// 同样手写反射
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EnemyFaction::R => write!(f, "R"),
            EnemyFaction::B => write!(f, "B"),
        }
    }
}

/// 描述装甲板的物理布局
#[derive(Debug, Clone, PartialEq)]
pub enum ArmorLayout {
    // 适用于大多数车辆的对称4装甲板布局
    Symmetric4 { layout: [(f64, f64); 4] },
    // 适用于前哨站的 3 块等距装甲板布局
    Tripod3 { radius: f64, height: f64 },
}

/// 用于描述一个敌方车辆的全部信息
/// A_N 代表装甲板数量，其他兵种为 4, 前哨站为 3
pub struct Enemy {
    /// 装甲板类型（大小装甲板）
    armor_type: EnemyArmorType,
    armor_id: EnemyId,
    /// 选择第一次看到该车的第一块装甲板为 idx = 0
    armor_layout: ArmorLayout,
    enemy_solver: RbtSolver,
}

pub struct EnemyDatabase {
    /// 敌方阵营
    enemy_faction: EnemyFaction,
    enemys: HashMap<EnemyId, Enemy>,
}

impl EnemyDatabase {
    /// 根据比赛情况，直接描述所有的车辆
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

    /// 获取所有敌人的迭代器
    pub fn iter(&self) -> impl Iterator<Item = (&EnemyId, &Enemy)> {
        self.enemys.iter()
    }
}
