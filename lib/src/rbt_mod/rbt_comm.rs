// use tokio::io::{AsyncReadExt, AsyncWriteExt};
// use tokio_serial::{SerialPortBuilderExt, SerialStream};

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
