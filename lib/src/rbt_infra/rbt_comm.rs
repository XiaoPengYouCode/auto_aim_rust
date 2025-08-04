// todo!(Under development)

use crate::rbt_infra::rbt_err;
use crate::rbt_infra::rbt_err::{CommError, RbtResult};
use std::collections::HashMap;
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::{SerialPortBuilderExt, SerialStream};

const IN_SOF: u8 = 0x00;
const OUT_SOF: u8 = 0x0F;
const IN_EOF: u8 = 0x00;
const OUT_EOF: u8 = 0x10;

/// 下发控制数据
///
/// * `gimbal_yaw` - 云台偏航角
/// * `gimbal_pitch` - 云台俯仰角
/// * `shot_mode` - 射击模式
/// * `shot_buff_mode` - 射击缓冲模式
/// * `aiming_state` - 瞄准状态
#[derive(Debug)]
pub struct CtrlData {
    pub sof: u8,
    pub gimbal_yaw: f32,
    pub gimbal_pitch: f32,
    pub shot_mode: u8,
    pub shot_buff_mode: u8,
    pub aiming_state: u8,
    pub eof: u8,
}

impl CtrlData {
    /// 创建新的控制数据实例
    pub fn new(
        gimbal_yaw: f32,
        gimbal_pitch: f32,
        shot_mode: u8,
        shot_buff_mode: u8,
        aiming_state: u8,
    ) -> Self {
        Self {
            sof: IN_SOF,
            gimbal_yaw,
            gimbal_pitch,
            shot_mode,
            shot_buff_mode,
            aiming_state,
            eof: IN_EOF,
        }
    }

    /// 将控制数据转换为字节数组
    ///
    /// # 返回值
    /// 返回包含控制数据的13字节长度的数组
    pub fn to_bytes(&self) -> [u8; 13] {
        let mut bytes = [0u8; 13];
        bytes[0] = self.sof;

        // 将f32类型的gimbal_yaw转换为4个字节
        let yaw_bytes = self.gimbal_yaw.to_le_bytes();
        bytes[1] = yaw_bytes[0];
        bytes[2] = yaw_bytes[1];
        bytes[3] = yaw_bytes[2];
        bytes[4] = yaw_bytes[3];

        // 将f32类型的gimbal_pitch转换为4个字节
        let pitch_bytes = self.gimbal_pitch.to_le_bytes();
        bytes[5] = pitch_bytes[0];
        bytes[6] = pitch_bytes[1];
        bytes[7] = pitch_bytes[2];
        bytes[8] = pitch_bytes[3];

        bytes[9] = self.shot_mode;
        bytes[10] = self.shot_buff_mode;
        bytes[11] = self.aiming_state;
        bytes[12] = self.eof;
        bytes
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
pub struct FeedBackData {
    pub sof: u8,
    pub task_mode: u8,
    pub self_team: u8,
    pub bullet_speed: f32,
    pub gimbal_roll: f32,
    pub gimbal_yaw: f32,
    pub gimbal_pitch: f32,
    pub yaw_speed: f32,
    pub eof: u8,
}

impl FeedBackData {
    /// 创建新的反馈数据实例
    ///
    /// # 返回值
    /// 返回初始化后的FeedBackData结构体实例，所有字段初始化为默认值
    pub fn new() -> Self {
        Self {
            sof: 0,
            task_mode: 0,
            self_team: 0,
            bullet_speed: 0.0,
            gimbal_roll: 0.0,
            gimbal_yaw: 0.0,
            gimbal_pitch: 0.0,
            yaw_speed: 0.0,
            eof: 0,
        }
    }

    /// 从字节数组解析反馈数据
    ///
    /// # 参数
    /// * `bytes` - 包含反馈数据的字节数组
    ///
    /// # 返回值
    /// 如果解析成功返回Some(FeedBackData)，否则返回None
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 28 {
            return None;
        }

        // 检查帧头帧尾
        if bytes[0] != IN_SOF || bytes[27] != IN_EOF {
            return None;
        }

        // 解析f32字段
        let bullet_speed = f32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let gimbal_roll = f32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        let gimbal_yaw = f32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]);
        let gimbal_pitch = f32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
        let yaw_speed = f32::from_le_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]);

        Some(Self {
            sof: bytes[0],
            task_mode: bytes[1],
            self_team: bytes[2],
            bullet_speed,
            gimbal_roll,
            gimbal_yaw,
            gimbal_pitch,
            yaw_speed,
            eof: bytes[27],
        })
    }

    /// 将反馈数据转换为字节数组
    ///
    /// # 返回值
    /// 返回包含反馈数据的28字节长度的数组
    pub fn to_bytes(&self) -> [u8; 28] {
        let mut bytes = [0u8; 28];
        bytes[0] = self.sof;
        bytes[1] = self.task_mode;
        bytes[2] = self.self_team;

        // 将f32字段转换为字节数组
        let bullet_speed_bytes = self.bullet_speed.to_le_bytes();
        bytes[4] = bullet_speed_bytes[0];
        bytes[5] = bullet_speed_bytes[1];
        bytes[6] = bullet_speed_bytes[2];
        bytes[7] = bullet_speed_bytes[3];

        let gimbal_roll_bytes = self.gimbal_roll.to_le_bytes();
        bytes[8] = gimbal_roll_bytes[0];
        bytes[9] = gimbal_roll_bytes[1];
        bytes[10] = gimbal_roll_bytes[2];
        bytes[11] = gimbal_roll_bytes[3];

        let gimbal_yaw_bytes = self.gimbal_yaw.to_le_bytes();
        bytes[12] = gimbal_yaw_bytes[0];
        bytes[13] = gimbal_yaw_bytes[1];
        bytes[14] = gimbal_yaw_bytes[2];
        bytes[15] = gimbal_yaw_bytes[3];

        let gimbal_pitch_bytes = self.gimbal_pitch.to_le_bytes();
        bytes[16] = gimbal_pitch_bytes[0];
        bytes[17] = gimbal_pitch_bytes[1];
        bytes[18] = gimbal_pitch_bytes[2];
        bytes[19] = gimbal_pitch_bytes[3];

        let yaw_speed_bytes = self.yaw_speed.to_le_bytes();
        bytes[20] = yaw_speed_bytes[0];
        bytes[21] = yaw_speed_bytes[1];
        bytes[22] = yaw_speed_bytes[2];
        bytes[23] = yaw_speed_bytes[3];

        bytes[27] = self.eof;
        bytes
    }
}

pub struct FeedBackFrame {
    data: FeedBackData,
    time_stamp: tokio::time::Instant,
}

impl FeedBackFrame {
    pub fn new(data: FeedBackData, time_stamp: tokio::time::Instant) -> Self {
        FeedBackFrame { data, time_stamp }
    }

    pub fn data(&self) -> &FeedBackData {
        &self.data
    }

    pub fn time_stamp(&self) -> &tokio::time::Instant {
        &self.time_stamp
    }
}

pub struct CtrlFrame {
    data: CtrlData,
    time_stamp: tokio::time::Instant,
}

impl CtrlFrame {
    pub fn new(data: CtrlData, time_stamp: tokio::time::Instant) -> Self {
        CtrlFrame { data, time_stamp }
    }

    pub fn data(&self) -> &CtrlData {
        &self.data
    }

    pub fn time_stamp(&self) -> &tokio::time::Instant {
        &self.time_stamp
    }
}

pub struct Serial {
    serial_stream: Option<SerialStream>,
    last_record_seq: u8,
}

impl Serial {
    pub fn new() -> Self {
        Serial {
            serial_stream: None,
            last_record_seq: 0,
        }
    }

    pub async fn open_port(&mut self) -> RbtResult<()> {
        println!("尝试打开串口中......");
        let mut try_time = 0;

        loop {
            let ports = vec!["THS0", "USB0", "ACM0"];
            let mut break_flag = false;

            for port in ports {
                let path = format!("/dev/tty{}", port);
                println!("尝试打开串口{}......", path);

                match tokio_serial::new(&path, 1000000).open_native_async() {
                    Ok(mut stream) => {
                        // 配置串口参数
                        // let mut settings = stream.into_settings();
                        // settings.set_baud_rate(1000000)?;
                        // settings.set_data_bits(tokio_serial::DataBits::Eight);
                        // settings.set_parity(tokio_serial::Parity::None);
                        // settings.set_stop_bits(tokio_serial::StopBits::One);
                        // settings.set_flow_control(tokio_serial::FlowControl::None);

                        // 尝试读取反馈数据来验证连接
                        // 这里需要根据实际需求实现try_feed_back方法
                        // if let Ok(_) = self.try_feed_back(Duration::from_millis(100)).await {
                        println!("成功！");
                        break_flag = true;
                        break;
                        // }
                    }
                    Err(e) => {
                        println!("失败！错误: {}", e);
                    }
                }
            }

            if break_flag {
                break;
            } else {
                tokio::time::sleep(Duration::from_millis(1000)).await;
                println!("尝试第{}次", try_time);
                try_time += 1;
            }
        }

        Ok(())
    }

    pub fn is_opened(&self) -> bool {
        self.serial_stream.is_some()
    }

    pub async fn setup(&mut self) -> RbtResult<()> {
        // 实现setup逻辑
        // 这里需要根据实际需求实现接收反馈数据的逻辑
        Ok(())
    }

    pub async fn try_control(
        &mut self,
        control_data: &CtrlData,
        time_duration: Duration,
    ) -> RbtResult<()> {
        // 实现控制逻辑
        self.control(control_data).await?;
        Ok(())
    }

    pub async fn try_control_simple(&mut self, control_data: &CtrlData) -> RbtResult<()> {
        self.control(control_data).await?;
        Ok(())
    }

    async fn control(&mut self, control_data: &CtrlData) -> RbtResult<()> {
        self.send(control_data).await
    }

    async fn send(&mut self, control_data: &CtrlData) -> RbtResult<()> {
        // 将ControlData转换为字节数组发送
        // 这里需要根据ControlData的实际内存布局来实现
        let data = self.control_data_to_bytes(control_data);
        if let Some(ref mut stream) = self.serial_stream {
            match stream.write_all(&data).await {
                Ok(_) => Ok(()),
                Err(_) => Err(CommError::IoError.into()),
            }
        } else {
            Err(CommError::NoPort.into())
        }
    }

    fn control_data_to_bytes(&self, control_data: &CtrlData) -> Vec<u8> {
        // 根据ControlData结构体转换为字节数组
        // 这里只是一个示例，实际需要根据内存布局来实现
        let mut bytes = Vec::new();
        bytes.push(control_data.sof);
        // 添加其他字段...
        bytes.push(control_data.eof);
        bytes
    }

    pub async fn try_feed_back(&mut self, time_duration: Duration) -> RbtResult<FeedBackData> {
        // 实现接收反馈数据的逻辑
        let mut feed_back_data = FeedBackData {
            sof: 0,
            task_mode: 0,
            self_team: 0,
            bullet_speed: 0.0,
            gimbal_roll: 0.0,
            gimbal_yaw: 0.0,
            gimbal_pitch: 0.0,
            yaw_speed: 0.0,
            eof: 0,
        };

        self.send_output(&mut feed_back_data).await?;
        Ok(feed_back_data)
    }

    async fn send_output(&mut self, feed_back_data: &mut FeedBackData) -> RbtResult<()> {
        let error_code = self.receive(feed_back_data).await?;
        Ok(error_code)
    }

    async fn receive(&mut self, feed_back_data: &mut FeedBackData) -> RbtResult<()> {
        // 实现接收数据逻辑
        if let Some(ref mut stream) = self.serial_stream {
            // 接收数据并解析
            // 根据C++代码，需要查找帧头帧尾并验证数据完整性
            let mut buffer = [0u8; 28]; // 假设反馈数据大小为28字节
            match stream.read_exact(&mut buffer).await {
                Ok(_) => {
                    // 解析数据
                    if buffer[0] == IN_SOF && buffer[buffer.len() - 1] == IN_EOF {
                        // 解析有效数据
                        feed_back_data.sof = buffer[0];
                        // 解析其他字段...
                        feed_back_data.eof = buffer[buffer.len() - 1];
                        Ok(())
                    } else {
                        Err(CommError::CorruptedFrame.into())
                    }
                }
                Err(_) => Err(CommError::IoError.into()),
            }
        } else {
            Err(CommError::NoPort.into())
        }
    }
}
