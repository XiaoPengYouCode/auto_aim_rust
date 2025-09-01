//! 敌方单位模型定义模块
//!
//! 该模块定义了RoboMaster比赛中敌方单位（装甲板）的模型和相关数据结构。
//! 包括装甲板类型、敌方ID、阵营、布局等基本信息，以及敌方单位的状态表示和运动学模型。
//!
//! 主要组件：
//! - EnemyId: 敌方单位标识枚举
//! - EnemyArmorType: 装甲板大小类型
//! - EnemyArmorLayout: 装甲板布局定义
//! - Enemy: 敌方单位完整信息描述
//! - EnemyESKFState: 敌方单位状态表示（用于ESKF）
//! - EnemyModel: 敌方单位运动学模型实现

use crate::rbt_base::rbt_algorithm::rbt_eskf::StrategyDynamicModel;
use crate::rbt_base::rbt_geometry::rbt_cylindrical2::RbtCylindricalPoint2;
use crate::rbt_mod::rbt_estimator::EstimatorStateMachine;
use crate::rbt_mod::rbt_solver::{RbtSolvedResult, RbtSolvedResults, RbtSolver};
use std::fmt::Display;

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
#[derive(PartialEq, Eq, strum::Display)]
pub enum EnemyFaction {
    R,
    B,
}

#[derive(Clone, Copy, Debug)]
pub struct ArmorRH {
    radius: f64,
    height: f64,
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
                radius: 200.0,
                height: 500.0,
            }),
            _ => EnemyArmorLayout::new_4(ArmorRH {
                radius: 200.0,
                height: 10.0,
            }),
        }
    }
}

/// 用于描述一个敌方车辆的全部信息
/// A_N 代表装甲板数量，其他兵种为 4, 前哨站为 3
#[derive(Debug, Clone)]
pub struct Enemy {
    /// 装甲板类型（大小装甲板）
    pub armor_type: EnemyArmorType,
    /// 选择第一次看到该车的第一块装甲板为 idx = 0
    pub armor_layout: EnemyArmorLayout,
    pub enemy_solver: RbtSolver,
    pub enemy_cy: RbtCylindricalPoint2,
    pub enemy_yaw: f64,
    pub nominal_state: EnemyESKFState,
    pub solved_enemy: RbtSolvedResult,
}

impl Enemy {
    pub fn new(enemy_id: &EnemyId, solved_enemy: Option<RbtSolvedResult>) -> Self {
        let solved_enemy = solved_enemy.unwrap_or_else(|| {
            // 如果没有提供solved_enemy，创建一个默认的
            panic!("Enemy::new requires a valid RbtSolvedResult")
        });

        Self {
            armor_type: EnemyArmorType::from_enemy_id(enemy_id),
            armor_layout: EnemyArmorLayout::from_enemy_id(enemy_id),
            enemy_solver: RbtSolver::new().unwrap(),
            enemy_cy: RbtCylindricalPoint2::new(0.0, 0.0),
            enemy_yaw: 0.0,
            nominal_state: EnemyESKFState {
                theta: 0.0,
                distance: 0.0,
                v_tang: 0.0,
                v_norm: 0.0,
                v_spin: 0.0,
                a_tang: 0.0,
                a_norm: 0.0,
                a_spin: 0.0,
                armor_yaw: 0.0,
                armor_r: 250.0, // 先验半径
                armor_height: 150.0,
            },
            solved_enemy,
        }
    }

    pub fn get_eskf_input(&self) -> [f64; 11] {
        [
            self.nominal_state.theta,
            self.nominal_state.distance,
            self.nominal_state.v_tang,
            self.nominal_state.v_norm,
            self.nominal_state.v_spin,
            self.nominal_state.a_tang,
            self.nominal_state.a_norm,
            self.nominal_state.a_spin,
            self.nominal_state.armor_yaw,
            self.nominal_state.armor_r,
            self.nominal_state.armor_height,
        ]
    }
    pub fn get_eskf_measurement(&self) -> [f64; 4] {
        [self.enemy_cy.theta_d, self.enemy_cy.rho, self.enemy_yaw, self.nominal_state.armor_height]
    }

    pub fn get_mut_nominal_state(&mut self) -> &mut EnemyESKFState {
        &mut self.nominal_state
    }
}

/// 用于 Enemy 和 EnemyEstimator 之间的数据交换
#[derive(Debug, Clone)]
pub struct EnemyESKFState {
    pub theta: f64,      // 车体中心相对于世界坐标系的角度 deg
    pub distance: f64,   // 敌方车体中心相对于己方中心的距离 mm
    pub v_tang: f64,     // 切向速度 mm/s
    pub v_norm: f64,     // 法向速度 mm/s
    pub v_spin: f64,     // 陀螺速度 deg/s
    pub a_tang: f64,     // 切向加速度 mm/s^2
    pub a_norm: f64,     // 法向加速度 mm/s^2
    pub a_spin: f64,     // 陀螺加速度 deg/s^2
    pub armor_yaw: f64,  // 追踪装甲板的 yaw 角 deg
    pub armor_r: f64,    // 追踪装甲板的半径 mm
    pub armor_height: f64, // 追踪装甲板的高度 mm
}

/// 敌方单位运动学模型
#[derive(Clone, Debug)]
pub struct EnemyModel {}

/// 敌方单位运动学模型实现
impl StrategyDynamicModel<11, 4> for EnemyModel {
    type Input = [f64; 11];
    type NominalState = EnemyESKFState;
    type Measurement = [f64; 4];
    type Strategy = EstimatorStateMachine;

    fn update_nominal_state(
        &self,
        nominal_state: &mut Self::NominalState,
        dt: f64,
        u: &Self::Input,
        strategy: &Self::Strategy,
    ) {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {}
            Track { .. } | Recovery => {
                // 匀加速模型更新
                nominal_state.theta += nominal_state.v_spin * dt + 0.5 * nominal_state.a_spin * dt * dt;
                nominal_state.distance += nominal_state.v_norm * dt + 0.5 * nominal_state.a_norm * dt * dt;
                nominal_state.v_tang += nominal_state.a_tang * dt;
                nominal_state.v_norm += nominal_state.a_norm * dt;
                nominal_state.v_spin += nominal_state.a_spin * dt;
                // 加速度保持不变（匀加速假设）
                nominal_state.armor_yaw += nominal_state.v_spin * dt;
                // armor_r 和 armor_height 保持不变
            }
            Switching => {
                // 云台移动中，保持状态不变
            }
            Lost { .. } => {
                // 纯依靠模型进行预测
                nominal_state.theta += nominal_state.v_spin * dt + 0.5 * nominal_state.a_spin * dt * dt;
                nominal_state.distance += nominal_state.v_norm * dt + 0.5 * nominal_state.a_norm * dt * dt;
                nominal_state.v_tang += nominal_state.a_tang * dt;
                nominal_state.v_norm += nominal_state.a_norm * dt;
                nominal_state.v_spin += nominal_state.a_spin * dt;
                nominal_state.armor_yaw += nominal_state.v_spin * dt;
            }
        }
    }

    fn state_transition_matrix_f(
        &self,
        s: &Self::NominalState,
        dt: f64,
        u: &Self::Input,
        strategy: &Self::Strategy,
    ) -> na::SMatrix<f64, 11, 11> {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {
                na::SMatrix::<f64, 11, 11>::identity()
            }
            Track { .. } | Recovery => {
                let mut f = na::SMatrix::<f64, 11, 11>::identity();
                // theta
                f[(0, 0)] = 1.0;
                f[(0, 4)] = dt;
                f[(0, 7)] = 0.5 * dt * dt;
                // distance
                f[(1, 1)] = 1.0;
                f[(1, 3)] = dt;
                f[(1, 6)] = 0.5 * dt * dt;
                // v_tang
                f[(2, 2)] = 1.0;
                f[(2, 5)] = dt;
                // v_norm
                f[(3, 3)] = 1.0;
                f[(3, 6)] = dt;
                // v_spin
                f[(4, 4)] = 1.0;
                f[(4, 7)] = dt;
                // a_tang, a_norm, a_spin 保持不变 (identity)
                // armor_yaw
                f[(8, 4)] = dt;
                f[(8, 8)] = 1.0;
                // armor_r, armor_height 保持不变
                f
            }
            Switching => {
                na::SMatrix::<f64, 11, 11>::identity()
            }
            Lost { .. } => {
                let mut f = na::SMatrix::<f64, 11, 11>::identity();
                // 与Aim相同，但不使用测量更新
                f[(0, 0)] = 1.0;
                f[(0, 4)] = dt;
                f[(0, 7)] = 0.5 * dt * dt;
                f[(1, 1)] = 1.0;
                f[(1, 3)] = dt;
                f[(1, 6)] = 0.5 * dt * dt;
                f[(2, 2)] = 1.0;
                f[(2, 5)] = dt;
                f[(3, 3)] = 1.0;
                f[(3, 6)] = dt;
                f[(4, 4)] = 1.0;
                f[(4, 7)] = dt;
                f[(8, 4)] = dt;
                f[(8, 8)] = 1.0;
                f
            }
        }
    }

    fn measurement_matrix_h(
        &self,
        nominal_state: &Self::NominalState,
        strategy: &Self::Strategy,
    ) -> na::SMatrix<f64, 4, 11> {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {
                na::SMatrix::<f64, 4, 11>::zeros()
            }
            Track { .. } | Lost { .. } | Recovery => {
                let mut h = na::SMatrix::<f64, 4, 11>::zeros();
                h[(0, 0)] = 1.0; // theta
                h[(1, 1)] = 1.0; // distance
                h[(2, 8)] = 1.0; // armor_yaw
                h[(3, 10)] = 1.0; // armor_height
                h
            }
            Switching => {
                na::SMatrix::<f64, 4, 11>::zeros()
            }
        }
    }

    fn measurement_residual_y(
        &self,
        nominal_state: &Self::NominalState,
        z: &Self::Measurement,
        strategy: &Self::Strategy,
    ) -> na::SVector<f64, 4> {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {
                na::SVector::<f64, 4>::zeros()
            }
            Track { .. } | Recovery => {
                let predicted_theta = nominal_state.theta % (2.0 * std::f64::consts::PI);
                let predicted_distance = nominal_state.distance;
                let predicted_armor_yaw = nominal_state.armor_yaw;
                let predicted_armor_height = nominal_state.armor_height;

                na::SVector::from([
                    z[0] - predicted_theta,
                    z[1] - predicted_distance,
                    z[2] - predicted_armor_yaw,
                    z[3] - predicted_armor_height,
                ])
            }
            Switching => {
                na::SVector::<f64, 4>::zeros()
            }
            Lost { .. } => {
                na::SVector::<f64, 4>::zeros()
            }
        }
    }

    fn inject_error(
        &self,
        nominal_state: &mut Self::NominalState,
        error_estimate: &na::SVector<f64, 11>,
        strategy: &Self::Strategy,
    ) {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {}
            Track { .. } | Lost { .. } | Recovery => {
                nominal_state.theta += error_estimate[0];
                nominal_state.distance += error_estimate[1];
                nominal_state.v_tang += error_estimate[2];
                nominal_state.v_norm += error_estimate[3];
                nominal_state.v_spin += error_estimate[4];
                nominal_state.a_tang += error_estimate[5];
                nominal_state.a_norm += error_estimate[6];
                nominal_state.a_spin += error_estimate[7];
                nominal_state.armor_yaw += error_estimate[8];
                nominal_state.armor_r += error_estimate[9];
                nominal_state.armor_height += error_estimate[10];
            }
            Switching => {
                // 云台移动中不注入误差
            }
        }
    }
}

/// 判断是否需要进行装甲板切换
pub fn armor_switch_decision(current_yaw: f64, predicted_yaw: f64, v_spin: f64, dt: f64) -> bool {
    // 小陀螺处理：连续跟踪同一装甲板，动态调整击打区间
    // 基于预测的角度判断是否需要切换装甲板

    // 计算预测的装甲板角度（考虑陀螺旋转）
    let future_yaw = predicted_yaw + v_spin * dt;

    // 定义可击打的角度区间（相对于当前瞄准角度）
    // 云台转速上限和响应时间会影响这个区间
    let max_gimbal_speed = 300.0; // deg/s，云台最大转速
    let response_time = 0.1; // 秒，响应时间
    let max_angle_change = max_gimbal_speed * response_time;

    // 如果陀螺速度过快，减小可击打区间甚至直接瞄准中心
    let effective_range = if v_spin.abs() > 200.0 {
        10.0 // 高速旋转时只允许小角度范围
    } else if v_spin.abs() > 100.0 {
        20.0 // 中速旋转
    } else {
        30.0 // 低速旋转
    };

    // 计算相对于当前瞄准角度的偏差
    let angle_diff = (future_yaw - current_yaw).abs() % 360.0;
    let min_diff = angle_diff.min(360.0 - angle_diff);

    // 如果预测角度超出可击打范围，需要切换
    min_diff > effective_range
}

/// 处理装甲板切换
pub fn handle_switch(current_yaw: f64, enemy_layout: &EnemyArmorLayout) -> f64 {
    // 切换到下一块装甲板
    // 根据敌方装甲板布局计算下一块装甲板的角度

    match enemy_layout {
        EnemyArmorLayout::Symmetric4(_) => {
            // 标准4块装甲板布局，间隔90度
            let next_yaw = (current_yaw + 90.0) % 360.0;
            tracing::info!("Switching armor: {:.1}° -> {:.1}°", current_yaw, next_yaw);
            next_yaw
        }
        EnemyArmorLayout::Tripod3(_) => {
            // 前哨站3块装甲板布局，间隔120度
            let next_yaw = (current_yaw + 120.0) % 360.0;
            tracing::info!("Switching outpost armor: {:.1}° -> {:.1}°", current_yaw, next_yaw);
            next_yaw
        }
    }
}

/// 角度归一化到[0, 360)范围
pub fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = angle % 360.0;
    if normalized < 0.0 {
        normalized += 360.0;
    }
    normalized
}
