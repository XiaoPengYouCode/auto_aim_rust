use super::rbt_estimator_def::TopLevel;
use crate::rbt_base::rbt_math::eskf::{Eskf, EskfDynamic};
use crate::rbt_mod::rbt_armor::ArmorLabel;

pub mod outpost_model;

pub struct EnemyModel<const N: usize> {
    // ESKF 滤波器实例
    pub eskf: Eskf<N, N>,
    radius: [f64; N],
    dz: [f64; N],
    enemy_id: ArmorLabel,
    top_level: TopLevel,
}
