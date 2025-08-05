/// 串口通讯(usart)
/// 基于 tokio_serial 封装
use tokio_serial::SerialStream;

pub struct Serial {
    serial_stream: Option<SerialStream>,
    last_record_seq: u8,
}
