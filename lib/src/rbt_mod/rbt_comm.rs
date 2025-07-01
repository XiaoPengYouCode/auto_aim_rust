#![allow(unused)]

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::{SerialPortBuilderExt, SerialStream};

const IN_BOF: u8 = 0x00;
const OUT_BOF: u8 = 0x0F;
const IN_EOF: u8 = 0x00;
const OUT_EOF: u8 = 0x10;

struct SensMsgData {
    yaw: u8,
}

impl SensMsgData {
    pub fn from_msg(msg: &[u8]) -> Self {
        assert_eq!(msg.len(), 3);
        let data = &msg[1..msg.len() - 2];
        SensMsgData { yaw: msg[0] }
    }
}

pub struct SensMsg {
    data: SensMsgData,
    tim: tokio::time::Instant,
}

pub struct CtrlMsgData {
    yaw: u8,
    pitch: u8,
}

impl CtrlMsgData {
    pub fn to_msg(&self) -> [u8; 4] {
        [OUT_BOF, self.yaw, self.pitch, OUT_EOF]
    }
}

pub struct CtrlMsg {
    data: CtrlMsgData,
    tim: tokio::time::Instant,
}

impl CtrlMsg {
    fn msg(&self) -> [u8; 4] {
        self.data.to_msg()
    }
}

pub async fn async_serial_example() -> tokio_serial::Result<()> {
    // 配置串口
    let port_name = "/dev/ttyTHS0";
    let baud_rate = 2_000_000;

    let mut port = tokio_serial::new(port_name, baud_rate).open_native_async()?;

    tracing::info!("异步串口已打开: {} @ {} baud", port_name, baud_rate);

    let mut ctrl_msg = CtrlMsg {
        data: CtrlMsgData {
            yaw: 0x00,
            pitch: 0x00,
        },
        tim: tokio::time::Instant::now(),
    };

    // 异步写入
    let cmd = ctrl_msg.msg();
    port.write_all(&cmd).await?;
    tracing::info!("已发送命令");

    // 异步读取
    let mut buffer = [0u8; 24];
    let bytes_read = port.read(&mut buffer).await?;
    let sens = SensMsgData::from_msg(&buffer[..bytes_read]);
    tracing::info!("收到 {} 字节: {:?}", bytes_read, &buffer[..bytes_read]);

    Ok(())
}
