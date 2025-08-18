use serde::Serialize;
use std::collections::HashMap;
use std::fmt::Display;
use std::ops::{Deref, DerefMut};
use tracing::info;

use crate::rbt_base::rbt_eskf::{StrategyESKF, StrategyESKFDynamic};
use crate::rbt_base::rbt_geometry::rbt_cylindrical2::RbtCylindricalPoint2;
use crate::rbt_infra::rbt_cfg::EstimatorCfg;
use crate::rbt_mod::rbt_armor::tracked_armor::TrackedArmor;
use crate::rbt_mod::rbt_estimator::rbt_enemy_model::{EnemyESKFState, EnemyModel};
pub(crate) use crate::rbt_mod::rbt_estimator::rbt_estimator_state::EstimatorStateMachine;
use crate::rbt_mod::rbt_solver::{RbtSolvedResult, RbtSolvedResults};
use rbt_enemy_model::{Enemy, EnemyId};

/// 动力学模型
pub mod rbt_enemy_model;

pub mod rbt_estimator_state {
    use crate::rbt_base::rbt_geometry::rbt_cylindrical2::RbtCylindricalPoint2;
    use crate::rbt_infra::rbt_cfg::EstimatorCfg;
    use crate::rbt_mod::rbt_solver::{RbtSolvedResult, RbtSolvedResults};

    /// 顶层状态机
    #[derive(Debug, Clone, PartialEq, strum::Display)]
    pub enum EstimatorStateMachine {
        Init,
        Sleep,
        WakeUp, // 从睡眠中恢复
        Aim,
        Lost {
            time_stamp: tokio::time::Instant, // 丢失时间辍
        },
        Recovery, // 从丢失状态中恢复
    }

    impl EstimatorStateMachine {
        pub fn update(&mut self, solved_enemy: &Option<RbtSolvedResult>, cfg: &EstimatorCfg) {
            use EstimatorStateMachine::*;
            match self {
                Init => {
                    *self = match solved_enemy {
                        Some(_) => WakeUp,
                        None => Sleep,
                    }
                }
                Sleep => {
                    if solved_enemy.is_some() {
                        *self = WakeUp;
                    }
                    // 其他情况保持不变
                }
                WakeUp => {
                    *self = match solved_enemy {
                        Some(_) => Aim,
                        None => Lost {
                            // 处理装甲板打灭的情况
                            // 以及可能出现的跳变
                            time_stamp: tokio::time::Instant::now(),
                        },
                    }
                }
                Aim => {
                    if solved_enemy.is_some() {
                        *self = Lost {
                            time_stamp: tokio::time::Instant::now(),
                        };
                    }
                    // 其他情况保持不变
                }
                Lost { time_stamp } => {
                    if let Some(_) = solved_enemy {
                        *self = Recovery; // 从 Lost 状态恢复需要一次唤醒，更新数据
                    } else {
                        if time_stamp.elapsed() > cfg.lost_wait_duration_ms() {
                            *self = Sleep;
                        }
                        // 否则继续保持 Lost 状态和时间戳不变
                        // 等待直到超时 Sleep
                    }
                }
                Recovery => {
                    // 更新数据
                    // self.enemy_estimator.update(dt, solved_enemy.unwrap());

                    if solved_enemy.is_some() {
                        *self = Aim;
                    } else {
                        *self = Lost {
                            time_stamp: tokio::time::Instant::now(),
                        };
                        // 否则继续保持 Recovery 状态和时间戳不变
                        // 继续等待直到超时 Lost
                    }
                }
            }
        }
    }
}

/// 状态估计器
#[derive(Debug, Clone)]
pub struct RbtEstimator {
    tracked_enemy: Option<Enemy>,
    last_tracked_enemy: Option<Enemy>,
    tracked_armor: Option<TrackedArmor>,
    last_tracked_armor: Option<TrackedArmor>,
    state: EstimatorStateMachine,
    eskf: StrategyESKF<5, 3>, // 原始 ESKF 求解器
    enemy_model: EnemyModel,
}

impl RbtEstimator {
    pub fn new() -> Self {
        Self {
            tracked_enemy: None,
            last_tracked_enemy: None,
            tracked_armor: None,
            last_tracked_armor: None,
            state: EstimatorStateMachine::Init,
            eskf: StrategyESKF::<5, 3>::new(
                na::SMatrix::<f64, 5, 5>::zeros(),
                na::SMatrix::<f64, 5, 5>::zeros(),
                na::SMatrix::<f64, 3, 3>::zeros(),
                0.01,
            ),
            enemy_model: EnemyModel {},
        }
    }

    pub fn update(
        &mut self,
        cfg: &EstimatorCfg,
        solved_enemy: &Option<RbtSolvedResult>,
        id: &EnemyId,
    ) {
        // 先执行状态机的更新
        self.state.update(solved_enemy, cfg);
        // 保存上一帧的状态
        self.last_tracked_enemy = self.tracked_enemy.take();
        self.tracked_enemy = Some(Enemy::new(id, solved_enemy.clone()));
        // 根据状态执行对应的处理
        self.handle_state(cfg);
    }

    pub fn handle_state(&mut self, _cfg: &EstimatorCfg) {
        use EstimatorStateMachine::*;
        match &self.state {
            Init => info!("{}", self.state.to_string()),
            Sleep => info!("{}", self.state.to_string()),
            WakeUp => {
                info!("{}", self.state.to_string());
                let enemy = self.tracked_enemy.as_mut().unwrap();
                let measurement = enemy.get_eskf_measurement();
                let nominal_state = enemy.get_mut_nominal_state();
                // 提高单次的置信度，加速模型收敛
                self.eskf
                    .update(&self.enemy_model, nominal_state, &measurement, &self.state);
                info!("{}", Aim.to_string());
            }
            Aim => {
                let enemy = self.tracked_enemy.as_mut().unwrap();
                let measurement = enemy.get_eskf_measurement();
                let nominal_state = enemy.get_mut_nominal_state();
                // 执行正常更新
                self.eskf
                    .update(&self.enemy_model, nominal_state, &measurement, &self.state);
                info!("{}", Aim.to_string());
            }
            Lost { .. } => {
                // 纯依靠模型进行预测，具体实现在 update 中
                let enemy = self.tracked_enemy.as_mut().unwrap();
                let measurement = enemy.get_eskf_measurement();
                let nominal_state = enemy.get_mut_nominal_state();
                // 执行正常更新
                self.eskf
                    .update(&self.enemy_model, nominal_state, &measurement, &self.state);
                info!("{}", Aim.to_string());
            }
            Recovery => {
                // 将测量置信度提高，加快新一轮收敛
                let enemy = self.tracked_enemy.as_mut().unwrap();
                let measurement = enemy.get_eskf_measurement();
                let nominal_state = enemy.get_mut_nominal_state();
                // 执行正常更新
                self.eskf
                    .update(&self.enemy_model, nominal_state, &measurement, &self.state);
                info!("{}", Aim.to_string());
            }
        }
    }
}

/// 状态估计器池
/// 管理所有敌方单位对应的估计器实例
#[derive(Debug, Clone)]
pub struct RbtHandlerPoll {
    estimators: HashMap<EnemyId, RbtEstimator>,
}

impl RbtHandlerPoll {
    /// 创建所有敌方单位的估计器实例
    pub fn new() -> Self {
        let mut estimators = HashMap::with_capacity(6);
        estimators.insert(EnemyId::Hero1, RbtEstimator::new());
        estimators.insert(EnemyId::Engineer2, RbtEstimator::new());
        estimators.insert(EnemyId::Infantry3, RbtEstimator::new());
        estimators.insert(EnemyId::Infantry4, RbtEstimator::new());
        estimators.insert(EnemyId::Sentry7, RbtEstimator::new());
        estimators.insert(EnemyId::Outpost8, RbtEstimator::new());
        Self { estimators }
    }

    /// 针对所有成功 Solved 的敌方单位进行更新
    pub async fn update(&mut self, cfg: &EstimatorCfg, solved_enemies: RbtSolvedResults) {
        // 针对每个已解的敌方单位，更新估计器
        for (solved_enemy_id, solved_enemy) in solved_enemies.deref() {
            info!("updated enemy id:{}", solved_enemy_id);
            // self.estimators.get_mut(solved_enemy_id).unwrap().update(
            //     cfg,
            //     solved_enemy,
            //     solved_enemy_id,
            // );
        }
        info!("todo: 针对解算的敌方单位进行更新")
    }
}
