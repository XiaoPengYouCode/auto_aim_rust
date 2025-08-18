use crate::rbt_base::rbt_eskf::StrategyESKFDynamic;
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
    armor_type: EnemyArmorType,
    /// 选择第一次看到该车的第一块装甲板为 idx = 0
    armor_layout: EnemyArmorLayout,
    enemy_solver: RbtSolver,
    enemy_cy: RbtCylindricalPoint2,
    enemy_yaw: f64,
    nominal_state: EnemyESKFState,
    solved_enemy: RbtSolvedResult,
}

impl Enemy {
    pub fn new(enemy_id: &EnemyId, solved_enemy: Option<RbtSolvedResult>) -> Self {
        Self {
            armor_type: EnemyArmorType::from_enemy_id(enemy_id),
            armor_layout: EnemyArmorLayout::from_enemy_id(enemy_id),
            enemy_solver: RbtSolver::new().unwrap(),
            enemy_cy: RbtCylindricalPoint2::new(0.0, 0.0),
            enemy_yaw: 0.0,
            nominal_state: EnemyESKFState {
                theta: 0.0,
                r: 0.0,
                v_tan: 0.0,
                v_norm: 0.0,
                v_spin: 0.0,
            },
            solved_enemy: solved_enemy.clone().unwrap(),
        }
    }
    pub fn get_eskf_measurement(&self) -> [f64; 3] {
        [self.enemy_cy.rho, self.enemy_cy.theta_d, self.enemy_yaw]
    }

    pub fn get_mut_nominal_state(&mut self) -> &mut EnemyESKFState {
        &mut self.nominal_state
    }
}

/// 用于 Enemy 和 EnemyEstimator 之间的数据交换
#[derive(Debug, Clone)]
pub struct EnemyESKFState {
    pub theta: f64,
    pub r: f64,
    pub v_tan: f64,
    pub v_norm: f64,
    pub v_spin: f64,
}

#[derive(Clone, Debug)]
pub struct EnemyModel {}

impl StrategyESKFDynamic<5, 3> for EnemyModel {
    type Input = [f64; 5];
    type NominalState = EnemyESKFState;
    type Measurement = [f64; 3];
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
            Aim => {
                // 更新标称状态
                nominal_state.theta += nominal_state.v_spin * dt;
                nominal_state.r += nominal_state.v_norm * dt;
                nominal_state.v_tan += u[2] * dt; // 假设输入包含切向加速度
                nominal_state.v_norm += u[3] * dt; // 假设输入包含法向加速度
                nominal_state.v_spin += u[4] * dt; // 假设输入包含旋转加速度
            }
            Lost { .. } => {
                // 纯依靠模型进行预测
                nominal_state.theta += nominal_state.v_spin * dt;
                nominal_state.r += nominal_state.v_norm * dt;
            }
            Recovery => {
                // 从丢失状态恢复，可能需要特殊处理
                nominal_state.theta += nominal_state.v_spin * dt;
                nominal_state.r += nominal_state.v_norm * dt;
                nominal_state.v_tan += u[2] * dt;
                nominal_state.v_norm += u[3] * dt;
                nominal_state.v_spin += u[4] * dt;
            }
        }
    }

    fn state_transition_matrix_f(
        &self,
        s: &Self::NominalState,
        dt: f64,
        u: &Self::Input,
        strategy: &Self::Strategy,
    ) -> na::SMatrix<f64, 5, 5> {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {
                // 初始化、休眠或唤醒状态下，状态转移矩阵为单位矩阵
                na::SMatrix::<f64, 5, 5>::identity()
            }
            Aim | Recovery => {
                // 正常瞄准或恢复状态下使用完整模型
                na::matrix![
                    1.0, -s.v_tan/(s.r*s.r)*dt, dt / s.r, 0.0, 0.0;
                    0.0,          1.0,            0.0,    dt,  0.0;
                    0.0,          0.0,            1.0,    0.0, 0.0;
                    0.0,          0.0,            0.0,    1.0, 0.0;
                    0.0,          0.0,            0.0,    0.0, 1.0;
                ]
            }
            Lost { .. } => {
                // 丢失状态下使用简化模型
                na::matrix![
                    1.0, 0.0, 0.0, 0.0, dt;  // theta 受 v_spin 影响
                    0.0, 1.0, 0.0, dt, 0.0;  // r 受 v_norm 影响
                    0.0, 0.0, 1.0, 0.0, 0.0; // v_tan 不变
                    0.0, 0.0, 0.0, 1.0, 0.0; // v_norm 不变
                    0.0, 0.0, 0.0, 0.0, 1.0; // v_spin 不变
                ]
            }
        }
    }

    fn measurement_matrix_h(
        &self,
        nominal_state: &Self::NominalState,
        strategy: &Self::Strategy,
    ) -> na::SMatrix<f64, 3, 5> {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {
                // 初始化、休眠或唤醒状态下，测量矩阵为零矩阵
                na::SMatrix::<f64, 3, 5>::zeros()
            }
            Aim | Lost { .. } | Recovery => {
                // 正常状态、丢失或恢复状态下使用标准测量矩阵
                na::matrix! [
                    1.0, 0.0, 0.0, 0.0, 0.0;  // theta
                    0.0, 1.0, 0.0, 0.0, 0.0;  // r
                    0.0, 0.0, 0.0, 0.0, 1.0;  // v_spin
                ]
            }
        }
    }

    fn measurement_residual_y(
        &self,
        nominal_state: &Self::NominalState,
        z: &Self::Measurement,
        strategy: &Self::Strategy,
    ) -> na::SVector<f64, 3> {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {
                // 初始化、休眠或唤醒状态下，残差为零向量
                na::SVector::<f64, 3>::zeros()
            }
            Aim | Recovery => {
                // 正常瞄准或恢复状态下计算完整残差
                // z: 实际测量值 [theta_measured, r_measured, v_spin_measured]
                let predicted_theta = nominal_state.theta % (2.0 * std::f64::consts::PI);
                let predicted_r = nominal_state.r;
                let predicted_v_spin = nominal_state.v_spin;

                na::SVector::from([
                    z[0] - predicted_theta,  // theta残差
                    z[1] - predicted_r,      // r残差
                    z[2] - predicted_v_spin, // v_spin残差
                ])
            }
            Lost { .. } => {
                // 丢失状态下暂时不使用测量更新
                // 依靠模型进行更新
                na::SVector::<f64, 3>::zeros()
            }
        }
    }

    fn inject_error(
        &self,
        nominal_state: &mut Self::NominalState,
        error_estimate: &na::SVector<f64, 5>,
        strategy: &Self::Strategy,
    ) {
        use EstimatorStateMachine::*;
        match strategy {
            Init | Sleep | WakeUp => {
                // 初始化、休眠或唤醒状态下不注入误差
            }
            Aim | Lost { .. } | Recovery => {
                // 将误差估计注入到标称状态中
                nominal_state.theta += error_estimate[0];
                nominal_state.r += error_estimate[1];
                nominal_state.v_tan += error_estimate[2];
                nominal_state.v_norm += error_estimate[3];
                nominal_state.v_spin += error_estimate[4];
            }
        }
    }
}
