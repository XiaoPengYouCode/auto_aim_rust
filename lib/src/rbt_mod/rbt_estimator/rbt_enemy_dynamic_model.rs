//! 敌方单位模型定义模块
//!
//! 该模块定义了RoboMaster比赛中敌方单位（装甲板）的模型和相关数据结构。
//! 包括装甲板类型、敌方ID、阵营、布局等基本信息。
//!
//! 主要组件：
//! - EnemyId: 敌方单位标识枚举
//! - EnemyArmorType: 装甲板大小类型
//! - EnemyArmorLayout: 装甲板布局定义

#[derive(Debug, Clone)]
/// 描述敌方装甲板大或者小
pub enum EnemyArmorType {
    Small,
    Large,
}

impl EnemyArmorType {
    pub fn from_enemy_id(enemy_id: &EnemyId) -> Self {
        match enemy_id {
            EnemyId::Hero1 => EnemyArmorType::Large,
            _ => EnemyArmorType::Small,
        }
    }
}

/// 用于描述装甲板/敌方车辆的唯一标记型 ID
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash, strum::Display)]
pub enum EnemyId {
    Hero1,
    Engineer2,
    Infantry3,
    Infantry4,
    Sentry7,
    Outpost8,
    Invalid,
}

/// 描述敌方阵营
#[derive(Debug, Clone, Copy, PartialEq, Eq, strum::Display)]
pub enum EnemyFaction {
    R,
    B,
}

#[derive(Clone, Copy, Debug)]
pub struct ArmorRH {
    _radius: f64,
    _height: f64,
}

/// 描述装甲板的物理布局
#[derive(Debug, Clone)]
pub enum EnemyArmorLayout {
    // 适用于大多数车辆的对称4装甲板布局
    Symmetric4([ArmorRH; 4]),
    // 适用于前哨站的 3 块等距装甲板布局
    Tripod3(ArmorRH),
}

impl EnemyArmorLayout {
    fn new_3(rh: ArmorRH) -> Self {
        EnemyArmorLayout::Tripod3(rh)
    }

    fn new_4(rh: ArmorRH) -> Self {
        EnemyArmorLayout::Symmetric4([rh; 4])
    }

    pub fn from_enemy_id(enemy_id: &EnemyId) -> Self {
        match enemy_id {
            EnemyId::Outpost8 => EnemyArmorLayout::new_3(ArmorRH {
                _radius: 200.0,
                _height: 500.0,
            }),
            _ => EnemyArmorLayout::new_4(ArmorRH {
                _radius: 200.0,
                _height: 10.0,
            }),
        }
    }
}
