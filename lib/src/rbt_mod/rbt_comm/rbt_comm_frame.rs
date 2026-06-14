//! 上下位机通讯帧定义。
//!
//! 对齐参考工程 `vivsionn/src/Serial/Serial.{h,cpp}` 的主线 SocketCAN 协议：
//! - 下发：CAN ID `0x100`，单个 8 字节 payload
//! - 上传：CAN ID `0x203` + `0x204`，两个 8 字节 payload 配对
//! - 角度和角速度均使用 `i16 * 0.01` 定点数，单位为 deg 或 deg/s
//! - 控制线程默认 250 Hz；反馈头尾帧配对超时 20 ms，反馈陈旧超时 500 ms

use crate::rbt_infra::rbt_err::{CommError, RbtResult};
use log::warn;

pub const TX_ID_GIMBAL: u32 = 0x100;
pub const RX_ID_FEEDBACK_HEAD: u32 = 0x203;
pub const RX_ID_FEEDBACK_TAIL: u32 = 0x204;

pub const CAN_FRAME_SIZE: usize = 8;
pub const FEEDBACK_PAIR_SIZE: usize = CAN_FRAME_SIZE * 2;
pub const CAN_FRAME_SOF: u8 = 0x33;
pub const CAN_FRAME_EOF: u8 = 0xEE;
pub const FIXED_POINT_SCALE: f32 = 100.0;

pub const CONTROL_LOOP_HZ: f64 = 250.0;
pub const CONTROL_LOOP_PERIOD_MS: f64 = 1_000.0 / CONTROL_LOOP_HZ;
pub const FEEDBACK_PAIR_TIMEOUT_MS: u64 = 20;
pub const FEEDBACK_STALE_TIMEOUT_MS: u64 = 500;
pub const DEFAULT_BULLET_SPEED_MPS: f32 = 23.5;

/// 固定长度通讯数据。
pub trait CommData: Sized {
    const FRAME_SIZE: usize;
    const SOF: u8 = CAN_FRAME_SOF;
    const EOF: u8 = CAN_FRAME_EOF;

    fn serialize(&self, buffer: &mut [u8]) -> RbtResult<()>;

    fn deserialize(buffer: &[u8]) -> RbtResult<Self>;

    fn validate_frame(buffer: &[u8]) -> RbtResult<()> {
        if buffer.len() != Self::FRAME_SIZE {
            return Err(CommError::FrameLengthError.into());
        }
        Ok(())
    }
}

/// 下发控制数据。
///
/// `gimbal_yaw` 和 `gimbal_pitch` 的业务单位是 deg。落到 CAN payload 时会按
/// `round(value * 100)` 编码为 little-endian `i16`。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CtrlData {
    pub gimbal_yaw: f32,
    pub gimbal_pitch: f32,
    pub shot_mode: ShotMode,
    pub shot_buff_mode: ShotBuffMode,
    pub aiming_state: AimingState,
}

/// 瞄准状态。
///
/// 业务枚举值保留旧协议的 `0x00/0x11/0x22/0x33` 语义；CAN flags 中只占 2 bit。
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum AimingState {
    NoAimingNoTarget = 0x00,
    CommunicateNoCamera = 0x11,
    AimingNoTarget = 0x22,
    AimingWithTarget = 0x33,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ShotBuffMode {
    /// 电控线协议兼容名：内部能量机关模块只在通讯边界转换到该枚举。
    ShotBuffOff = 0x00,
    /// 电控线协议兼容名：内部能量机关模块只在通讯边界转换到该枚举。
    ShotBuffOn = 0x01,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ShotMode {
    DoNothing = 0x00,
    AimOnly = 0x01,
    AutoFire = 0x02,
    ShotOnce = 0x03,
}

impl CtrlData {
    /// 序列化为 `TX_ID_GIMBAL = 0x100` 使用的 8 字节 CAN payload。
    pub fn serialize_with_seq(&self, frame_seq: u8, buffer: &mut [u8]) -> RbtResult<()> {
        if buffer.len() != CAN_FRAME_SIZE {
            return Err(CommError::FrameLengthError.into());
        }

        buffer[0] = CAN_FRAME_SOF;
        buffer[1] = build_control_flags(*self);
        buffer[2] = frame_seq;
        buffer[3] = CAN_FRAME_EOF;
        write_i16_le(&mut buffer[4..6], encode_i16_x100(self.gimbal_pitch));
        write_i16_le(&mut buffer[6..8], encode_i16_x100(self.gimbal_yaw));
        Ok(())
    }
}

impl AimingState {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0x00 => AimingState::NoAimingNoTarget,
            0x11 => AimingState::CommunicateNoCamera,
            0x22 => AimingState::AimingNoTarget,
            0x33 => AimingState::AimingWithTarget,
            _ => {
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
                warn!("Invalid energy mechanism shot protocol value {}", value);
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
    const FRAME_SIZE: usize = CAN_FRAME_SIZE;

    fn serialize(&self, buffer: &mut [u8]) -> RbtResult<()> {
        self.serialize_with_seq(0, buffer)
    }

    fn deserialize(buffer: &[u8]) -> RbtResult<Self> {
        Self::validate_frame(buffer)?;
        if buffer[0] != CAN_FRAME_SOF {
            return Err(CommError::InvalidStartOfFrame.into());
        }
        if buffer[3] != CAN_FRAME_EOF {
            return Err(CommError::InvalidEndOfFrame.into());
        }

        let flags = buffer[1];
        Ok(Self {
            gimbal_pitch: decode_i16_x100(read_i16_le(&buffer[4..6])),
            gimbal_yaw: decode_i16_x100(read_i16_le(&buffer[6..8])),
            shot_mode: decode_shot_mode_flags(flags),
            shot_buff_mode: decode_shot_buff_flags(flags),
            aiming_state: decode_aiming_state_flags(flags),
        })
    }
}

/// 上传反馈数据。
///
/// 当前 CAN 主线反馈没有携带弹速；`bullet_speed` 为视觉侧固定业务常量。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensData {
    pub task_mode: TaskMode,
    pub self_fraction: SelfFraction,
    pub bullet_speed: f32,
    pub gimbal_roll: f32,
    pub gimbal_yaw: f32,
    pub gimbal_pitch: f32,
    pub yaw_speed: f32,
    pub mcu_fire_permit: bool,
    pub raw_task_mode: u8,
    pub mapped_task_mode: TaskMode,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum TaskMode {
    AutoShot = 0x01,
    /// 电控线协议兼容名：视觉侧映射到内部 EnergyMechanism route。
    HitBigBuff = 0x02,
    /// 电控线协议兼容名：视觉侧映射到内部 EnergyMechanism route。
    HitSmallBuff = 0x03,
    HitOutpost = 0x04,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum SelfFraction {
    Red = 0xAA,
    Blue = 0xBB,
}

impl SensData {
    pub fn serialize_pair(&self, head: &mut [u8], tail: &mut [u8]) -> RbtResult<()> {
        if head.len() != CAN_FRAME_SIZE || tail.len() != CAN_FRAME_SIZE {
            return Err(CommError::FrameLengthError.into());
        }

        head[0] = CAN_FRAME_SOF;
        head[1] = self.task_mode.into();
        head[2] = self.self_fraction.into();
        head[3] = 0;
        write_i16_le(&mut head[4..6], encode_i16_x100(self.gimbal_roll));
        write_i16_le(&mut head[6..8], encode_i16_x100(self.gimbal_yaw));

        write_i16_le(&mut tail[0..2], encode_i16_x100(self.gimbal_pitch));
        write_i16_le(&mut tail[2..4], encode_i16_x100(self.yaw_speed));
        tail[4] = u8::from(self.mcu_fire_permit);
        tail[5] = 0;
        tail[6] = 0;
        tail[7] = CAN_FRAME_EOF;
        Ok(())
    }

    pub fn deserialize_pair(head: &[u8], tail: &[u8]) -> RbtResult<Self> {
        if head.len() != CAN_FRAME_SIZE || tail.len() != CAN_FRAME_SIZE {
            return Err(CommError::FrameLengthError.into());
        }
        if head[0] != CAN_FRAME_SOF {
            return Err(CommError::InvalidStartOfFrame.into());
        }
        if tail[7] != CAN_FRAME_EOF {
            return Err(CommError::InvalidEndOfFrame.into());
        }

        let mapped_task_mode = TaskMode::try_from_wire(head[1])?;
        Ok(Self {
            task_mode: mapped_task_mode,
            self_fraction: SelfFraction::from_u8(head[2]),
            bullet_speed: DEFAULT_BULLET_SPEED_MPS,
            gimbal_roll: decode_i16_x100(read_i16_le(&head[4..6])),
            gimbal_yaw: decode_i16_x100(read_i16_le(&head[6..8])),
            gimbal_pitch: decode_i16_x100(read_i16_le(&tail[0..2])),
            yaw_speed: decode_i16_x100(read_i16_le(&tail[2..4])),
            mcu_fire_permit: tail[4] != 0,
            raw_task_mode: head[1],
            mapped_task_mode,
        })
    }
}

impl TaskMode {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0x01 => TaskMode::AutoShot,
            0x02 => TaskMode::HitBigBuff,
            0x03 => TaskMode::HitSmallBuff,
            0x04 => TaskMode::HitOutpost,
            _ => {
                warn!("Invalid task mode value: {}", value);
                TaskMode::AutoShot
            }
        }
    }

    fn try_from_wire(value: u8) -> RbtResult<Self> {
        match value {
            0x01 => Ok(TaskMode::AutoShot),
            0x02 => Ok(TaskMode::HitBigBuff),
            0x03 => Ok(TaskMode::HitSmallBuff),
            0x04 => Ok(TaskMode::HitOutpost),
            _ => Err(CommError::CorruptedFrame.into()),
        }
    }
}

impl SelfFraction {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0xAA => SelfFraction::Red,
            0xBB => SelfFraction::Blue,
            _ => {
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
    const FRAME_SIZE: usize = FEEDBACK_PAIR_SIZE;

    fn serialize(&self, buffer: &mut [u8]) -> RbtResult<()> {
        Self::validate_frame(buffer)?;
        let (head, tail) = buffer.split_at_mut(CAN_FRAME_SIZE);
        self.serialize_pair(head, tail)
    }

    fn deserialize(buffer: &[u8]) -> RbtResult<Self> {
        Self::validate_frame(buffer)?;
        let (head, tail) = buffer.split_at(CAN_FRAME_SIZE);
        Self::deserialize_pair(head, tail)
    }
}

/// 带时间戳记录的传感器帧。
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

/// 带时间戳记录的控制帧。
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

fn build_control_flags(control_data: CtrlData) -> u8 {
    (encode_shot_mode(control_data.shot_mode) & 0x03)
        | ((u8::from(control_data.shot_buff_mode) & 0x03) << 2)
        | ((encode_aiming_state(control_data.aiming_state) & 0x03) << 4)
}

fn encode_shot_mode(mode: ShotMode) -> u8 {
    match mode {
        ShotMode::AimOnly => 1,
        ShotMode::AutoFire => 2,
        ShotMode::ShotOnce => 3,
        ShotMode::DoNothing => 0,
    }
}

fn encode_aiming_state(state: AimingState) -> u8 {
    match state {
        AimingState::CommunicateNoCamera => 1,
        AimingState::AimingNoTarget => 2,
        AimingState::AimingWithTarget => 3,
        AimingState::NoAimingNoTarget => 0,
    }
}

fn decode_shot_mode_flags(flags: u8) -> ShotMode {
    ShotMode::from_u8(flags & 0x03)
}

fn decode_shot_buff_flags(flags: u8) -> ShotBuffMode {
    ShotBuffMode::from_u8((flags >> 2) & 0x03)
}

fn decode_aiming_state_flags(flags: u8) -> AimingState {
    match (flags >> 4) & 0x03 {
        1 => AimingState::CommunicateNoCamera,
        2 => AimingState::AimingNoTarget,
        3 => AimingState::AimingWithTarget,
        _ => AimingState::NoAimingNoTarget,
    }
}

fn encode_i16_x100(value: f32) -> i16 {
    if !value.is_finite() {
        return 0;
    }
    let scaled = f64::from(value) * f64::from(FIXED_POINT_SCALE);
    scaled
        .round()
        .clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16
}

fn decode_i16_x100(value: i16) -> f32 {
    f32::from(value) / FIXED_POINT_SCALE
}

fn write_i16_le(dst: &mut [u8], value: i16) {
    dst.copy_from_slice(&value.to_le_bytes());
}

fn read_i16_le(src: &[u8]) -> i16 {
    i16::from_le_bytes([src[0], src[1]])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-6,
            "actual={actual}, expected={expected}"
        );
    }

    #[test]
    fn control_frame_matches_vivsionn_can_payload() {
        let data = CtrlData {
            gimbal_yaw: -12.34,
            gimbal_pitch: 5.67,
            shot_mode: ShotMode::AutoFire,
            shot_buff_mode: ShotBuffMode::ShotBuffOn,
            aiming_state: AimingState::AimingWithTarget,
        };
        let mut payload = [0u8; CAN_FRAME_SIZE];

        data.serialize_with_seq(0x7A, &mut payload).unwrap();

        assert_eq!(
            payload,
            [
                CAN_FRAME_SOF,
                0x36,
                0x7A,
                CAN_FRAME_EOF,
                0x37,
                0x02,
                0x2E,
                0xFB
            ]
        );
        let decoded = CtrlData::deserialize(&payload).unwrap();
        assert_close(decoded.gimbal_pitch, 5.67);
        assert_close(decoded.gimbal_yaw, -12.34);
        assert_eq!(decoded.shot_mode, ShotMode::AutoFire);
        assert_eq!(decoded.shot_buff_mode, ShotBuffMode::ShotBuffOn);
        assert_eq!(decoded.aiming_state, AimingState::AimingWithTarget);
    }

    #[test]
    fn control_frame_clamps_fixed_point_range() {
        let data = CtrlData {
            gimbal_yaw: 500.0,
            gimbal_pitch: -500.0,
            shot_mode: ShotMode::AimOnly,
            shot_buff_mode: ShotBuffMode::ShotBuffOff,
            aiming_state: AimingState::AimingNoTarget,
        };
        let mut payload = [0u8; CAN_FRAME_SIZE];

        data.serialize(&mut payload).unwrap();

        assert_eq!(read_i16_le(&payload[4..6]), i16::MIN);
        assert_eq!(read_i16_le(&payload[6..8]), i16::MAX);
    }

    #[test]
    fn feedback_pair_decodes_head_and_tail_payloads() {
        let mut head = [0u8; CAN_FRAME_SIZE];
        let mut tail = [0u8; CAN_FRAME_SIZE];
        head[0] = CAN_FRAME_SOF;
        head[1] = TaskMode::HitOutpost.into();
        head[2] = SelfFraction::Blue.into();
        write_i16_le(&mut head[4..6], -125);
        write_i16_le(&mut head[6..8], 3050);
        write_i16_le(&mut tail[0..2], -675);
        write_i16_le(&mut tail[2..4], 12345);
        tail[4] = 1;
        tail[7] = CAN_FRAME_EOF;

        let decoded = SensData::deserialize_pair(&head, &tail).unwrap();

        assert_eq!(decoded.task_mode, TaskMode::HitOutpost);
        assert_eq!(decoded.self_fraction, SelfFraction::Blue);
        assert_close(decoded.gimbal_roll, -1.25);
        assert_close(decoded.gimbal_yaw, 30.5);
        assert_close(decoded.gimbal_pitch, -6.75);
        assert_close(decoded.yaw_speed, 123.45);
        assert!(decoded.mcu_fire_permit);
        assert_eq!(decoded.raw_task_mode, 0x04);
        assert_eq!(decoded.mapped_task_mode, TaskMode::HitOutpost);
        assert_eq!(decoded.bullet_speed, DEFAULT_BULLET_SPEED_MPS);
    }

    #[test]
    fn feedback_pair_serializes_to_two_can_payloads() {
        let data = SensData {
            task_mode: TaskMode::AutoShot,
            self_fraction: SelfFraction::Red,
            bullet_speed: 22.0,
            gimbal_roll: 1.23,
            gimbal_yaw: -4.56,
            gimbal_pitch: 7.89,
            yaw_speed: -10.0,
            mcu_fire_permit: true,
            raw_task_mode: 0x01,
            mapped_task_mode: TaskMode::AutoShot,
        };
        let mut payload = [0u8; FEEDBACK_PAIR_SIZE];

        data.serialize(&mut payload).unwrap();

        assert_eq!(payload[0], CAN_FRAME_SOF);
        assert_eq!(payload[1], TaskMode::AutoShot as u8);
        assert_eq!(payload[2], SelfFraction::Red as u8);
        assert_eq!(read_i16_le(&payload[4..6]), 123);
        assert_eq!(read_i16_le(&payload[6..8]), -456);
        assert_eq!(read_i16_le(&payload[8..10]), 789);
        assert_eq!(read_i16_le(&payload[10..12]), -1000);
        assert_eq!(payload[12], 1);
        assert_eq!(payload[13], 0);
        assert_eq!(payload[14], 0);
        assert_eq!(payload[15], CAN_FRAME_EOF);
    }

    #[test]
    fn rejects_invalid_feedback_pair_markers() {
        let head = [0u8; CAN_FRAME_SIZE];
        let tail = [0u8; CAN_FRAME_SIZE];

        assert!(matches!(
            SensData::deserialize_pair(&head, &tail).unwrap_err(),
            crate::rbt_infra::rbt_err::RbtError::CommError(CommError::InvalidStartOfFrame)
        ));
    }

    #[test]
    fn exposes_mainline_timing_constants() {
        assert_eq!(TX_ID_GIMBAL, 0x100);
        assert_eq!(RX_ID_FEEDBACK_HEAD, 0x203);
        assert_eq!(RX_ID_FEEDBACK_TAIL, 0x204);
        assert_eq!(CONTROL_LOOP_HZ, 250.0);
        assert_eq!(CONTROL_LOOP_PERIOD_MS, 4.0);
        assert_eq!(FEEDBACK_PAIR_TIMEOUT_MS, 20);
        assert_eq!(FEEDBACK_STALE_TIMEOUT_MS, 500);
        assert_eq!(DEFAULT_BULLET_SPEED_MPS, 23.5);
    }
}
