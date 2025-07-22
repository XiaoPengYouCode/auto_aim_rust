use crate::rbt_mod::rbt_armor::tracked_armor::TrackedArmor;
use crate::rbt_mod::rbt_enemy::Enemy;

pub mod enemy_model;
pub mod rbt_estimator_def;
pub mod rbt_estimator_impl;

pub struct RbtEstimator {
    tracked_enemy: Enemy,
    tracked_armor: TrackedArmor,
}

// /// role: 从所有检测到的目标，以及上一次击打的敌人，综合考虑，得到目标击打的敌人
// /// brief:
// fn enemy_select_filter(enemys: Vec<Enemy>) -> Enemy {
// }

// impl RbtEstimator {
//     /// role: 从敌人的所有可能装甲板中选中目标击打的装甲板
//     /// input: 敌人的所有装甲板
//     /// output: 目标 Armor
//     fn armor_select_filter(&self, armors: ArmorLayout) -> Armor {
//         // 1. 将当前追踪的装甲板和输入的装甲板进行匹配
//         if self.tracked_armor.
//     }
// }
