use rbt_infra::rbt_err::RbtResult;
use rbt_mod::rbt_comm::{CommData, CtrlData, CtrlFrame, SensData, SensFrame};

pub trait RbtComm {
    fn open(&mut self) -> RbtResult<()>;
    fn send(&mut self, data: &[u8]) -> RbtResult<()>;
    fn receive(&mut self) -> RbtResult<Vec<u8>>;
    fn close(&mut self) -> RbtResult<()>;
}

pub mod rbt_usb {
    /// USB通讯
    /// 基于 rusb 封装
    use rusb::DeviceHandle;

    pub struct RbtUsb {
        device_handle: Option<DeviceHandle<rusb::Context>>,
        last_record_seq: u8,
    }

    impl RbtComm for RbtUsb {
        fn open(&mut self) -> RbtResult<()> {
            Ok(())
        }

        fn send(&mut self, data: &[u8]) -> RbtResult<()> {
            Ok(())
        }

        fn receive(&mut self) -> RbtResult<Vec<u8>> {
            Ok(vec![])
        }

        fn close(&mut self) -> RbtResult<()> {
            Ok(())
        }
    }
}

pub mod rbt_serial {
    /// 串口通讯(usart)
    /// 基于 tokio_serial 封装
    use tokio_serial::SerialStream;

    pub struct RbtSerial {
        serial_stream: Option<SerialStream>,
        last_record_seq: u8,
    }

    impl RbtComm for RbtSerial {
        fn open(&mut self) -> RbtResult<()> {
            Ok(())
        }

        fn send(&mut self, data: &[u8]) -> RbtResult<()> {
            Ok(())
        }

        fn receive(&mut self) -> RbtResult<Vec<u8>> {
            Ok(vec![])
        }

        fn close(&mut self) -> RbtResult<()> {
            Ok(())
        }
    }
}

pub mod rbt_udp {
    /// UDP通讯
    /// 基于 tokio_udp 封装
    use tokio_udp::UdpSocket;

    pub struct RbtUdp {
        udp_socket: Option<UdpSocket>,
        last_record_seq: u8,
    }

    impl RbtComm for RbtUdp {
        fn open(&mut self) -> RbtResult<()> {
            Ok(())
        }

        fn send(&mut self, data: &[u8]) -> RbtResult<()> {
            Ok(())
        }

        fn receive(&mut self) -> RbtResult<Vec<u8>> {
            Ok(vec![])
        }

        fn close(&mut self) -> RbtResult<()> {
            Ok(())
        }
    }
}
