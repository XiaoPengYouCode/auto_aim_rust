/// 该文件定义了上下位机通讯的数据结构
/// 卢钟瑾 2025.08.06
use crate::rbt_infra::rbt_err::{CommError, RbtResult};
use tracing::warn;

/// 帧结构 trait
/// 类型大小确定
pub trait CommData: Sized {
    const FRAME_SIZE: usize;
    const SOF: u8;
    const EOF: u8;

    fn serialize(&self, buffer: &mut [u8]) -> RbtResult<()>;

    fn deserialize(buffer: &[u8]) -> RbtResult<Self>;

    /// 检查帧长度，帧头和帧尾
    fn validate_frame(buffer: &[u8]) -> RbtResult<()> {
        if buffer.len() != Self::FRAME_SIZE {
            return Err(CommError::FrameLengthError.into());
        }
        if buffer[0] != Self::SOF {
            return Err(CommError::InvalidStartOfFrame.into());
        }
        if buffer[buffer.len() - 1] != Self::EOF {
            return Err(CommError::InvalidEndOfFrame.into());
        }
        Ok(())
    }
}

/// 下发控制数据
///
/// * `gimbal_yaw` - 云台偏航角
/// * `gimbal_pitch` - 云台俯仰角
/// * `shot_mode` - 射击模式
/// * `shot_buff_mode` - 射击缓冲模式
/// * `aiming_state` - 瞄准状态
#[derive(Debug)]
pub struct CtrlData {
    pub gimbal_yaw: f32,
    pub gimbal_pitch: f32,
    pub shot_mode: ShotMode,
    pub shot_buff_mode: ShotBuffMode,
    pub aiming_state: AimingState,
}

/// 瞄准状态枚举
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum AimingState {
    NoAimingNoTarget = 0x00,
    CommunicateNoCamera = 0x11,
    AimingNoTarget = 0x22,
    AimingWithTarget = 0x33,
}

/// 射击缓冲模式枚举
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ShotBuffMode {
    ShotBuffOff = 0x00, // 禁用射击缓冲模式
    ShotBuffOn = 0x01,  // 启用射击缓冲模式
}

/// 射击模式枚举
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ShotMode {
    DoNothing = 0x00, // 不瞄准也不发射
    AimOnly = 0x01,   // 只瞄准不发射
    AutoFire = 0x02,  // 瞄准且发射
    ShotOnce = 0x03,  // 单次射击模式
}

impl AimingState {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0x00 => AimingState::NoAimingNoTarget,
            0x11 => AimingState::CommunicateNoCamera,
            0x22 => AimingState::AimingNoTarget,
            0x33 => AimingState::AimingWithTarget,
            _ => {
                // 默认值
                warn!("Invalid aim mode state {}", value);
                AimingState::NoAimingNoTarget
            }
        }
    }
}

impl ShotBuffMode {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0x00 => ShotBuffMode::ShotBuffOff,
            0x01 => ShotBuffMode::ShotBuffOn,
            _ => {
                // 默认值
                warn!("Invalid shot buff mode value {}", value);
                ShotBuffMode::ShotBuffOff
            }
        }
    }
}

impl ShotMode {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0x00 => ShotMode::DoNothing,
            0x01 => ShotMode::AimOnly,
            0x02 => ShotMode::AutoFire,
            0x03 => ShotMode::ShotOnce,
            _ => {
                // 默认值
                warn!("Invalid shot mode value: {}", value);
                ShotMode::DoNothing
            }
        }
    }
}

impl From<AimingState> for u8 {
    fn from(state: AimingState) -> u8 {
        state as u8
    }
}

impl From<ShotBuffMode> for u8 {
    fn from(mode: ShotBuffMode) -> u8 {
        mode as u8
    }
}

impl From<ShotMode> for u8 {
    fn from(mode: ShotMode) -> u8 {
        mode as u8
    }
}

impl CommData for CtrlData {
    const FRAME_SIZE: usize = 13;
    const SOF: u8 = 0x33;
    const EOF: u8 = 0xEE;

    /// 序列化
    fn serialize(&self, buffer: &mut [u8]) -> RbtResult<()> {
        Self::validate_frame(buffer)?;

        buffer[0] = Self::SOF;
        let gimbal_yaw_bytes = self.gimbal_yaw.to_le_bytes();
        buffer[1..5].copy_from_slice(&gimbal_yaw_bytes);
        let gimbal_pitch_bytes = self.gimbal_pitch.to_le_bytes();
        buffer[5..9].copy_from_slice(&gimbal_pitch_bytes);
        buffer[9] = self.shot_mode.into();
        buffer[10] = self.shot_buff_mode.into();
        buffer[11] = self.aiming_state.into();
        buffer[12] = Self::EOF;

        Ok(())
    }

    /// 反序列化
    fn deserialize(buffer: &[u8]) -> RbtResult<Self> {
        Self::validate_frame(buffer)?;

        let mut bytes = [0u8; 13];
        bytes.copy_from_slice(&buffer[0..13]);

        let gimbal_yaw = f32::from_le_bytes([bytes[1], bytes[2], bytes[3], bytes[4]]);
        let gimbal_pitch = f32::from_le_bytes([bytes[5], bytes[6], bytes[7], bytes[8]]);

        Ok(Self {
            gimbal_yaw,
            gimbal_pitch,
            shot_mode: ShotMode::from_u8(bytes[9]),
            shot_buff_mode: ShotBuffMode::from_u8(bytes[10]),
            aiming_state: AimingState::from_u8(bytes[11]),
        })
    }
}

/// 接受反馈数据
///
/// - task_mode: 任务模式
/// - self_team: 自身队伍
/// - bullet_speed: 子弹速度
/// - gimbal_roll: 云台横滚角
/// - gimbal_yaw: 云台偏航角
/// - gimbal_pitch: 云台俯仰角,
/// - yaw_speed: 偏航速度
#[derive(Debug)]
pub struct SensData {
    pub task_mode: TaskMode,
    pub self_fraction: SelfFraction,
    pub bullet_speed: f32,
    pub gimbal_roll: f32,
    pub gimbal_yaw: f32,
    pub gimbal_pitch: f32,
    pub yaw_speed: f32,
}

/// 任务模式枚举
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum TaskMode {
    AutoShot = 0x01,     // 自瞄
    HitBigBuff = 0x02,   // 大符
    HitSmallBuff = 0x03, // 小符
}

/// 自身队伍枚举
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum SelfFraction {
    Red = 0xAA,  // 红色队伍
    Blue = 0xBB, // 蓝色队伍
}

impl TaskMode {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0x01 => TaskMode::AutoShot,
            0x02 => TaskMode::HitBigBuff,
            0x03 => TaskMode::HitSmallBuff,
            _ => {
                // 默认值
                warn!("Invalid self fraction value: {}", value);
                TaskMode::AutoShot
            }
        }
    }
}

impl SelfFraction {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0xAA => SelfFraction::Red,
            0xBB => SelfFraction::Blue,
            _ => {
                // 默认值
                warn!("Invalid self fraction value: {}", value);
                SelfFraction::Red
            }
        }
    }
}

impl From<TaskMode> for u8 {
    fn from(mode: TaskMode) -> u8 {
        mode as u8
    }
}

impl From<SelfFraction> for u8 {
    fn from(team: SelfFraction) -> u8 {
        team as u8
    }
}

impl CommData for SensData {
    const FRAME_SIZE: usize = 24;
    const SOF: u8 = 0x33;
    const EOF: u8 = 0xEE;

    /// 序列化操作
    fn serialize(&self, buffer: &mut [u8]) -> RbtResult<()> {
        Self::validate_frame(buffer)?;

        buffer[0] = Self::SOF;
        buffer[1] = self.task_mode.into();
        buffer[2] = self.self_fraction.into();
        let bullet_speed_bytes = self.bullet_speed.to_le_bytes();
        buffer[3..7].copy_from_slice(&bullet_speed_bytes);
        let gimbal_roll_bytes = self.gimbal_roll.to_le_bytes();
        buffer[7..11].copy_from_slice(&gimbal_roll_bytes);
        let gimbal_yaw_bytes = self.gimbal_yaw.to_le_bytes();
        buffer[11..15].copy_from_slice(&gimbal_yaw_bytes);
        let gimbal_pitch_bytes = self.gimbal_pitch.to_le_bytes();
        buffer[15..19].copy_from_slice(&gimbal_pitch_bytes);
        let yaw_speed_bytes = self.yaw_speed.to_le_bytes();
        buffer[19..23].copy_from_slice(&yaw_speed_bytes);
        buffer[23] = Self::EOF;

        Ok(())
    }

    /// 反序列化操作
    fn deserialize(buffer: &[u8]) -> RbtResult<Self> {
        Self::validate_frame(buffer)?;

        let mut bytes = [0u8; 24];
        bytes.copy_from_slice(&buffer[0..24]);

        let task_mode = TaskMode::from_u8(bytes[1]);
        let self_fraction = SelfFraction::from_u8(bytes[2]);
        let bullet_speed = f32::from_le_bytes([bytes[3], bytes[4], bytes[5], bytes[6]]);
        let gimbal_roll = f32::from_le_bytes([bytes[7], bytes[8], bytes[9], bytes[10]]);
        let gimbal_yaw = f32::from_le_bytes([bytes[11], bytes[12], bytes[13], bytes[14]]);
        let gimbal_pitch = f32::from_le_bytes([bytes[15], bytes[16], bytes[17], bytes[18]]);
        let yaw_speed = f32::from_le_bytes([bytes[19], bytes[20], bytes[21], bytes[22]]);

        Ok(Self {
            task_mode,
            self_fraction,
            bullet_speed,
            gimbal_roll,
            gimbal_yaw,
            gimbal_pitch,
            yaw_speed,
        })
    }
}

/// 带时间戳记录的传感器帧
pub struct SensFrame {
    data: SensData,
    time_stamp: tokio::time::Instant,
}

impl SensFrame {
    pub fn new(data: SensData) -> Self {
        SensFrame {
            data,
            time_stamp: tokio::time::Instant::now(),
        }
    }

    pub fn data(&self) -> &SensData {
        &self.data
    }

    pub fn time_stamp(&self) -> &tokio::time::Instant {
        &self.time_stamp
    }
}

/// 带时间戳记录的控制帧
pub struct CtrlFrame {
    data: CtrlData,
    time_stamp: tokio::time::Instant,
}

impl CtrlFrame {
    pub fn new(data: CtrlData) -> Self {
        CtrlFrame {
            data,
            time_stamp: tokio::time::Instant::now(),
        }
    }

    pub fn data(&self) -> &CtrlData {
        &self.data
    }

    pub fn time_stamp(&self) -> &tokio::time::Instant {
        &self.time_stamp
    }
}
