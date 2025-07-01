// #![allow(unused)]

use tokio::time::Instant;

use crate::rbt_mod::rbt_armor::EnemyId;

pub mod enemy_model;
pub mod rbt_estimator_def;
pub mod rbt_estimator_impl;

use enemy_model::EnemyModel;

pub trait EnemyInstant {}

impl EnemyInstant for EnemyModel<3> {}

impl EnemyInstant for EnemyModel<4> {}

pub struct EstimatorHandle {
    estimator_models: Vec<Box<dyn EnemyInstant>>,
}

pub struct EstimatorManager {}
