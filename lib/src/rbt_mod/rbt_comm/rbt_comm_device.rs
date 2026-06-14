use crate::rbt_infra::rbt_err::RbtResult;
// use crate::rbt_mod::rbt_comm::{CommData, CtrlData, CtrlFrame, SensData, SensFrame};

pub trait RbtComm {
    fn open(&mut self) -> RbtResult<()>;
    fn send(&mut self, data: &[u8]) -> RbtResult<()>;
    fn receive(&mut self) -> RbtResult<Vec<u8>>;
    fn close(&mut self) -> RbtResult<()>;
}

pub mod rbt_can {
    use std::time::{Duration, Instant};

    use crate::rbt_infra::rbt_err::{RbtError, RbtResult};
    use crate::rbt_mod::rbt_comm::rbt_comm_frame::{
        CAN_FRAME_SIZE, CtrlData, FEEDBACK_PAIR_TIMEOUT_MS, RX_ID_FEEDBACK_HEAD,
        RX_ID_FEEDBACK_TAIL, SensData, TX_ID_GIMBAL,
    };

    #[derive(Debug, Clone)]
    pub struct CanFramePayload {
        pub id: u32,
        pub data: [u8; CAN_FRAME_SIZE],
    }

    #[derive(Debug)]
    struct PendingFeedbackHead {
        data: [u8; CAN_FRAME_SIZE],
        received_at: Instant,
    }

    #[derive(Debug, Default)]
    pub struct FeedbackPairDecoder {
        pending_head: Option<PendingFeedbackHead>,
    }

    impl FeedbackPairDecoder {
        pub fn new() -> Self {
            Self::default()
        }

        pub fn push(
            &mut self,
            frame: CanFramePayload,
            now: Instant,
        ) -> RbtResult<Option<SensData>> {
            match frame.id {
                RX_ID_FEEDBACK_HEAD => {
                    self.pending_head = Some(PendingFeedbackHead {
                        data: frame.data,
                        received_at: now,
                    });
                    Ok(None)
                }
                RX_ID_FEEDBACK_TAIL => {
                    let Some(head) = self.pending_head.take() else {
                        return Ok(None);
                    };
                    if now.duration_since(head.received_at)
                        > Duration::from_millis(FEEDBACK_PAIR_TIMEOUT_MS)
                    {
                        return Ok(None);
                    }
                    SensData::deserialize_pair(&head.data, &frame.data).map(Some)
                }
                _ => Ok(None),
            }
        }
    }

    pub fn control_to_can_payload(control: CtrlData, frame_seq: u8) -> RbtResult<CanFramePayload> {
        let mut data = [0_u8; CAN_FRAME_SIZE];
        control.serialize_with_seq(frame_seq, &mut data)?;
        Ok(CanFramePayload {
            id: TX_ID_GIMBAL,
            data,
        })
    }

    #[cfg(target_os = "linux")]
    pub struct SocketCanDevice {
        socket: socketcan::tokio::CanSocket,
    }

    #[cfg(target_os = "linux")]
    impl SocketCanDevice {
        pub fn open(interface: &str) -> RbtResult<Self> {
            let socket = socketcan::tokio::CanSocket::open(interface).map_err(RbtError::from)?;
            Ok(Self { socket })
        }

        pub async fn send(&self, frame: CanFramePayload) -> RbtResult<()> {
            use crate::rbt_infra::rbt_err::CommError;
            use socketcan::embedded_can::{Frame, StandardId};

            let id = StandardId::new(frame.id as u16).ok_or(CommError::CorruptedFrame)?;
            let can_frame =
                socketcan::CanFrame::new(id, &frame.data).ok_or(CommError::CorruptedFrame)?;
            self.socket.write_frame(can_frame).await?;
            Ok(())
        }

        pub async fn receive(&self) -> RbtResult<CanFramePayload> {
            use socketcan::EmbeddedFrame;

            loop {
                let frame = self.socket.read_frame().await?;
                let socketcan::CanFrame::Data(frame) = frame else {
                    continue;
                };
                let id = match frame.id() {
                    socketcan::Id::Standard(id) => u32::from(id.as_raw()),
                    socketcan::Id::Extended(id) => id.as_raw(),
                };
                if frame.data().len() != CAN_FRAME_SIZE {
                    continue;
                }
                let mut data = [0_u8; CAN_FRAME_SIZE];
                data.copy_from_slice(frame.data());
                return Ok(CanFramePayload { id, data });
            }
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub struct SocketCanDevice;

    #[cfg(not(target_os = "linux"))]
    impl SocketCanDevice {
        pub fn open(interface: &str) -> RbtResult<Self> {
            Err(RbtError::UnsupportedExecutionProvider(format!(
                "SocketCAN interface {interface} is only supported on Linux"
            )))
        }

        pub async fn send(&self, _frame: CanFramePayload) -> RbtResult<()> {
            Err(RbtError::UnsupportedExecutionProvider(
                "SocketCAN send is only supported on Linux".to_string(),
            ))
        }

        pub async fn receive(&self) -> RbtResult<CanFramePayload> {
            Err(RbtError::UnsupportedExecutionProvider(
                "SocketCAN receive is only supported on Linux".to_string(),
            ))
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use crate::rbt_mod::rbt_comm::rbt_comm_frame::{
            AimingState, CAN_FRAME_EOF, CAN_FRAME_SOF, DEFAULT_BULLET_SPEED_MPS, SelfFraction,
            ShotBuffMode, ShotMode, TaskMode,
        };

        #[test]
        fn control_payload_uses_gimbal_can_id() {
            let payload = control_to_can_payload(
                CtrlData {
                    gimbal_yaw: -1.25,
                    gimbal_pitch: 2.5,
                    shot_mode: ShotMode::AimOnly,
                    shot_buff_mode: ShotBuffMode::ShotBuffOff,
                    aiming_state: AimingState::AimingWithTarget,
                },
                3,
            )
            .unwrap();

            assert_eq!(payload.id, TX_ID_GIMBAL);
            assert_eq!(payload.data[0], CAN_FRAME_SOF);
            assert_eq!(payload.data[2], 3);
            assert_eq!(payload.data[3], CAN_FRAME_EOF);
        }

        #[test]
        fn feedback_decoder_pairs_head_and_tail() {
            let mut head = [0_u8; CAN_FRAME_SIZE];
            let mut tail = [0_u8; CAN_FRAME_SIZE];
            let feedback = SensData {
                task_mode: TaskMode::HitOutpost,
                self_fraction: SelfFraction::Blue,
                bullet_speed: DEFAULT_BULLET_SPEED_MPS,
                gimbal_roll: 1.0,
                gimbal_yaw: 2.0,
                gimbal_pitch: 3.0,
                yaw_speed: 4.0,
                mcu_fire_permit: true,
                raw_task_mode: TaskMode::HitOutpost.into(),
                mapped_task_mode: TaskMode::HitOutpost,
            };
            feedback.serialize_pair(&mut head, &mut tail).unwrap();

            let now = Instant::now();
            let mut decoder = FeedbackPairDecoder::new();
            assert!(
                decoder
                    .push(
                        CanFramePayload {
                            id: RX_ID_FEEDBACK_HEAD,
                            data: head,
                        },
                        now,
                    )
                    .unwrap()
                    .is_none()
            );
            let decoded = decoder
                .push(
                    CanFramePayload {
                        id: RX_ID_FEEDBACK_TAIL,
                        data: tail,
                    },
                    now + Duration::from_millis(1),
                )
                .unwrap()
                .unwrap();

            assert_eq!(decoded.task_mode, TaskMode::HitOutpost);
            assert_eq!(decoded.self_fraction, SelfFraction::Blue);
            assert!(decoded.mcu_fire_permit);
        }

        #[test]
        fn feedback_decoder_drops_stale_pair() {
            let mut decoder = FeedbackPairDecoder::new();
            let now = Instant::now();

            decoder
                .push(
                    CanFramePayload {
                        id: RX_ID_FEEDBACK_HEAD,
                        data: [0_u8; CAN_FRAME_SIZE],
                    },
                    now,
                )
                .unwrap();
            let decoded = decoder
                .push(
                    CanFramePayload {
                        id: RX_ID_FEEDBACK_TAIL,
                        data: [0_u8; CAN_FRAME_SIZE],
                    },
                    now + Duration::from_millis(FEEDBACK_PAIR_TIMEOUT_MS + 1),
                )
                .unwrap();

            assert!(decoded.is_none());
        }
    }
}

pub mod rbt_usb {
    // /// USB通讯
    // /// 基于 rusb 封装
    // use rusb::DeviceHandle;

    // pub struct RbtUsb {
    //     device_handle: Option<DeviceHandle<rusb::Context>>,
    //     last_record_seq: u8,
    // }

    // impl RbtComm for RbtUsb {
    //     fn open(&mut self) -> RbtResult<()> {
    //         Ok(())
    //     }

    //     fn send(&mut self, data: &[u8]) -> RbtResult<()> {
    //         Ok(())
    //     }

    //     fn receive(&mut self) -> RbtResult<Vec<u8>> {
    //         Ok(vec![])
    //     }

    //     fn close(&mut self) -> RbtResult<()> {
    //         Ok(())
    //     }
    // }
}

/// 串口通讯(usart)
/// 基于 tokio_serial 封装
pub mod rbt_serial {
    use tokio_serial::SerialStream;

    use super::RbtComm;
    use crate::rbt_infra::rbt_err::RbtResult;

    pub struct RbtSerial {
        _serial_stream: Option<SerialStream>,
        _last_record_seq: u8,
    }

    impl RbtComm for RbtSerial {
        fn open(&mut self) -> RbtResult<()> {
            Ok(())
        }

        fn send(&mut self, _data: &[u8]) -> RbtResult<()> {
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
    // /// UDP通讯
    // /// 基于 tokio_udp 封装
    // use tokio_udp::UdpSocket;
    // use crate::rbt_infra::rbt_err::RbtResult;

    // pub struct RbtUdp {
    //     udp_socket: Option<UdpSocket>,
    //     last_record_seq: u8,
    // }

    // impl RbtComm for RbtUdp {
    //     fn open(&mut self) -> RbtResult<()> {
    //         Ok(())
    //     }

    //     fn send(&mut self, data: &[u8]) -> RbtResult<()> {
    //         Ok(())
    //     }

    //     fn receive(&mut self) -> RbtResult<Vec<u8>> {
    //         Ok(vec![])
    //     }

    //     fn close(&mut self) -> RbtResult<()> {
    //         Ok(())
    //     }
    // }
}
