//! 状态估计器模块
//!
//! 该模块实现了基于扩展卡尔曼滤波器的敌方单位状态估计功能。
//! 通过融合视觉测量数据和运动学模型，对敌方单位的位置、速度等状态进行估计和预测。
//!
//! 主要组件：
//! - EstimatorStateMachine: 估计器状态机，管理估计器的不同工作状态
//! - RbtEstimator: 单个敌方单位的状态估计器实现
//! - RbtHandlerPoll: 所有敌方单位估计器的管理池
//!

use serde::Serialize;
use std::collections::HashMap;
use std::fmt::Display;
use std::ops::{Deref, DerefMut};
use tracing::info;

use crate::rbt_base::rbt_algorithm::rbt_eskf::{ESKF, StrategyDynamicModel};
use crate::rbt_base::rbt_geometry::rbt_cylindrical2::RbtCylindricalPoint2;
use crate::rbt_infra::rbt_cfg::EstimatorCfg;
use crate::rbt_mod::rbt_armor::tracked_armor::TrackedArmor;
use crate::rbt_mod::rbt_solver::{RbtSolvedResult, RbtSolvedResults};

use rbt_enemy_dynamic_model::{
    Enemy, EnemyESKFState, EnemyId, EnemyModel, armor_switch_decision, handle_switch,
};
use rbt_estimator_state::EstimatorStateMachine;

/// 动力学模型
pub mod rbt_enemy_dynamic_model;

pub mod rbt_estimator_state {
    use crate::rbt_infra::rbt_cfg::EstimatorCfg;
    use crate::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::handle_switch;
    use crate::rbt_mod::rbt_solver::RbtSolvedResult;

    /// 顶层状态机
    #[derive(Debug, Clone, PartialEq, strum::Display)]
    pub enum EstimatorStateMachine {
        Init, // 初始化
        Sleep,
        WakeUp, // 从睡眠中恢复
        Track {
            jump: bool,
        }, // 跟踪状态
        Switching, // 云台移动中
        Lost {
            // 目标丢失（未识别，装甲板灭）
            time_stamp: tokio::time::Instant, // 丢失时间戳
        },
        Recovery, // 从丢失状态中恢复
    }

    impl EstimatorStateMachine {
        pub fn update(&mut self, solved_enemy: &Option<RbtSolvedResult>, cfg: &EstimatorCfg) {
            use EstimatorStateMachine::*;
            match self {
                Init => {
                    // 只会在初始化用到，然后在第一次 update 流转至其他状态
                    *self = match solved_enemy {
                        Some(_) => WakeUp,
                        None => Sleep,
                    }
                }
                Sleep => {
                    // 看到装甲板则唤醒估计器
                    if solved_enemy.is_some() {
                        *self = WakeUp;
                    }
                    // 没看到就继续休眠
                }
                WakeUp => {
                    // 看到装甲板则进入追踪
                    *self = match solved_enemy {
                        Some(_) => Track { jump: false },
                        None => Lost {
                            time_stamp: tokio::time::Instant::now(),
                        },
                    }
                }
                Track { jump } => {
                    if *jump {
                        *self = Switching;
                    }
                    // 如果solved_enemy 是 None 进入Lost状态，并记录当前时间戳
                    if solved_enemy.is_none() {
                        *self = Lost {
                            time_stamp: tokio::time::Instant::now(),
                        };
                    }
                }
                Switching => {
                    // 检查是否到位，如果到位则回到Track
                    // TODO: 实现云台到位检查
                    *self = Track { jump: false };
                }
                Lost { time_stamp } => {
                    *self = match (
                        solved_enemy.is_some(),                             // 是否检测到装甲板
                        time_stamp.elapsed() > cfg.lost_wait_duration_ms(), // 是否超时
                    ) {
                        (true, _) => Recovery,  // 如果检测到装甲板，进入Recovery状态
                        (false, true) => Sleep, // 如果没检测到装甲板且超时，进入Sleep状态
                        (false, false) => Lost {
                            time_stamp: *time_stamp, /* copy */
                        }, // 如果没检测到装甲板且未超时，保持Lost状态
                    };
                }
                Recovery => {
                    *self = match solved_enemy {
                        Some(_) => Track { jump: false },
                        None => Lost {
                            time_stamp: tokio::time::Instant::now(),
                        },
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
    eskf: ESKF<11, 4>, // 原始 ESKF 求解器
    enemy_model: EnemyModel,
    pub enemy_id: EnemyId,
    pub fire: bool, // 当前是否开火
    pub single_or_double: bool, // 单或双装甲板更新，用于设置ESKF测量噪声
}

impl RbtEstimator {
    pub fn new(enemy_id: EnemyId) -> Self {
        Self {
            tracked_enemy: None,
            last_tracked_enemy: None,
            tracked_armor: None,
            last_tracked_armor: None,
            state: EstimatorStateMachine::Init,
            eskf: ESKF::<11, 4>::new(
                na::SMatrix::<f64, 11, 11>::identity() * 100.0, // 初始协方差矩阵
                na::SMatrix::<f64, 11, 11>::identity() * 0.1,   // 过程噪声
                na::SMatrix::<f64, 4, 4>::identity() * 0.1,     // 测量噪声
                0.01,
            ),
            enemy_model: EnemyModel {},
            enemy_id,
            fire: false,
            single_or_double: false,
        }
    }

    pub fn update(
        &mut self,
        cfg: &EstimatorCfg,
        solved_enemy: &Option<RbtSolvedResult>,
    ) {
        // 1. 保存上一帧状态
        self.last_tracked_enemy = self.tracked_enemy.clone();
        self.last_tracked_armor = self.tracked_armor.clone();

        // 2. 仅在检测到有效敌人时更新当前状态
        if let Some(solved) = solved_enemy {
            // 如果tracked_enemy不存在，创建新的；如果存在，更新其状态
            if let Some(enemy) = &mut self.tracked_enemy {
                // 更新现有enemy的状态
                enemy.nominal_state.theta = solved.coord.theta_d;
                enemy.nominal_state.distance = solved.coord.rho;
                enemy.nominal_state.armor_yaw = solved.coord.theta_d;
                enemy.enemy_yaw = solved.coord.theta_d;
                enemy.enemy_cy = solved.coord.clone();
                enemy.solved_enemy = solved.clone();
            } else {
                // 创建新的enemy
                self.tracked_enemy = Some(Enemy::new(&self.enemy_id, Some(solved.clone())));
            }
            self.tracked_armor = Some(TrackedArmor::new(solved.armors.clone()[0].clone(), 0.0));
        } else {
            self.tracked_enemy = None;
            self.tracked_armor = None;
        }

        // 3. 更新状态机
        self.state.update(solved_enemy, cfg);

        // 4. 设置全局变量
        self.update_global_vars(solved_enemy);

        // 5. 装甲板切换决策
        let jump = if let Some(enemy) = &self.tracked_enemy {
            armor_switch_decision(
                self.tracked_armor.as_ref().map(|a| a.enemy_yaw).unwrap_or(0.0),
                enemy.nominal_state.armor_yaw,
                enemy.nominal_state.v_spin,
                0.01, // dt
            )
        } else {
            false
        };

        // 6. 根据状态更新估计器
        self.handle_state(cfg, solved_enemy, jump);
    }

    fn update_global_vars(&mut self, solved_enemy: &Option<RbtSolvedResult>) {
        use EstimatorStateMachine::*;
        // 设置fire
        self.fire = matches!(self.state, Track { .. });

        // 设置single_or_double
        self.single_or_double = solved_enemy
            .as_ref()
            .map(|s| s.armors.len() > 1)
            .unwrap_or(false);
    }

    // 修改：添加jump参数处理装甲板切换
    pub fn handle_state(&mut self, cfg: &EstimatorCfg, solved_enemy: &Option<RbtSolvedResult>, jump: bool) {
        use EstimatorStateMachine::*;

        // 仅在有有效解时解包，否则直接返回
        let solved_enemy = match solved_enemy {
            Some(solved) => solved,
            None => return // 无有效解时不处理
        };

        info!("State: {}", self.state);

        match &self.state {
            Init | Sleep => {} // 待机状态不处理
            WakeUp => {
                // 获得 enemy 的可变引用并初始化状态
                if let Some(enemy) = self.tracked_enemy.as_mut() {
                    enemy.nominal_state.theta = solved_enemy.coord.theta_d;
                    enemy.nominal_state.distance = solved_enemy.coord.rho;
                    enemy.enemy_yaw = solved_enemy.coord.theta_d;
                    enemy.enemy_cy = solved_enemy.coord.clone();
                }
            }
            Track { jump: state_jump } => {
                if let Some(enemy) = self.tracked_enemy.as_mut() {
                    // 先复制armor_layout以避免借用冲突
                    let armor_layout = enemy.armor_layout.clone();

                    let input = enemy.get_eskf_input();
                    let measurement = enemy.get_eskf_measurement();
                    let nominal_state = enemy.get_mut_nominal_state();

                    // 处理装甲板跳变
                    if *state_jump {
                        let new_target_yaw = handle_switch(
                            self.tracked_armor.as_ref().map(|a| a.enemy_yaw).unwrap_or(0.0),
                            &armor_layout
                        );
                        // 更新跟踪装甲板的目标角度
                        if let Some(armor) = &mut self.tracked_armor {
                            armor.enemy_yaw = new_target_yaw;
                        }
                        // 重置跳变标志
                        if let Track { jump: jump_flag } = &mut self.state {
                            *jump_flag = false;
                        }
                    }

                    self.eskf.predict(&self.enemy_model, nominal_state, &input, &self.state);
                    // 恢复状态增加过程噪声加快收敛
                    self.eskf.set_r(na::SMatrix::<f64, 4, 4>::identity() * 0.5);
                    self.eskf.predict(&self.enemy_model, nominal_state, &input, &self.state);
                    self.eskf.update(&self.enemy_model, nominal_state, &measurement, &self.state);
                    self.eskf.set_r(na::SMatrix::<f64, 4, 4>::identity() * 0.1); // 恢复默认噪声
                }
            }
            Switching => {
                // 云台移动中，不进行预测和更新
                // TODO: 实现云台到位检查逻辑
            }
            Lost { .. } => {
                if let Some(enemy) = self.tracked_enemy.as_mut() {
                    let enemy = self.tracked_enemy.as_mut().unwrap();
                    let input = enemy.get_eskf_input();
                    let nominal_state = enemy.get_mut_nominal_state();
                    self.eskf
                        .predict(&self.enemy_model, nominal_state, &input, &self.state);
                }
            }
            Recovery => {
                if let Some(enemy) = self.tracked_enemy.as_mut() {
                    let enemy = self.tracked_enemy.as_mut().unwrap();
                    let input = enemy.get_eskf_input();
                    let measurement = enemy.get_eskf_measurement();
                    let nominal_state = enemy.get_mut_nominal_state();

                    // 增加过程噪声以加快收敛
                    self.eskf.set_r(na::SMatrix::<f64, 4, 4>::identity() * 0.5);
                    self.eskf.predict(&self.enemy_model, nominal_state, &input, &self.state);
                    self.eskf
                        .update(&self.enemy_model, nominal_state, &measurement, &self.state);
                    // 恢复默认过程噪声
                    self.eskf.set_r(na::SMatrix::<f64, 4, 4>::identity() * 0.1);
                }
            }
        }
    }
}
