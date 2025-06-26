// #![allow(unused)]

pub mod armor_model;
pub mod enemy_model;

// const OUTPOST_ROTATE_RL: f64 = 553.0;
// 目标运动模式，用于辅助响应
pub enum RbtRigidState {
    Static, // 静止状态
    Trans,  // 横移
    Spin,   // 主要指陀螺，不考虑偏心陀螺（帧率高，可以认为是平移）
    Rigid,  // 复合运动
}
