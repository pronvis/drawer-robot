use rtic_sync::channel::{Receiver, Sender};

use crate::PS3_CHANNEL_CAPACITY;

//two header bits come first
const HEADER_BYTE: u8 = 0x11;
const COMMAND_DIGITAL: u8 = 0x12;
const COMMAND_ANALOG: u8 = 0x13;
const COMMAND_CONNECT: u8 = 0x14;
const END_BYTE: u8 = 0x15;

enum Ps3ReaderState {
    WaitingForHeader(u8),
    WaitingForCommandType,
    WaitingForDigitalData,
    WaitingForAnalogData(u8),
    WaitingForEndByte,
}

// impl Ps3ReaderState {
//     fn is_waiting_for_header(&self) -> bool {
//         match self {
//             Ps3ReaderState::WaitingForHeader(_) => true,
//             _ => false,
//         }
//     }

//     fn is_waiting_for_command_type(&self) -> bool {
//         match self {
//             Ps3ReaderState::WaitingForCommandType => true,
//             _ => false,
//         }
//     }

//     fn is_waiting_for_data(&self) -> bool {
//         match self {
//             Ps3ReaderState::WaitingForHeader(_) => true,
//             _ => false,
//         }
//     }
// }

#[derive(Debug)]
pub enum Ps3Event {
    OnConnect,
    DigitalSignal(u8),
    AnalogSignal([u8; 3]),
    // CrossDown,
    // CrossUp,
    // SquareUp,
    // SquareDown,
    // TriangleUp,
    // TriangleDown,
    // CurcleUp,
    // CurcleDown,
    // UpUp,
    // UpDown,
    // DownUp,
    // DownDown,
    // LeftUp,
    // LeftDown,
    // RightUp,
    // RightDown,
    // LeftStick { x: i8, y: i8 },
    // RightStick { x: i8, y: i8 },
}

pub struct Ps3Reader {
    bytes_receiver: Receiver<'static, u8, PS3_CHANNEL_CAPACITY>,
    state: Ps3ReaderState,
    // channel to send commands from PS3
    events_sender: Sender<'static, Ps3Event, PS3_CHANNEL_CAPACITY>,
    event_builder: Option<Ps3Event>,
}

impl Ps3Reader {
    pub fn new(
        bytes_receiver: Receiver<'static, u8, PS3_CHANNEL_CAPACITY>,
        events_sender: Sender<'static, Ps3Event, PS3_CHANNEL_CAPACITY>,
    ) -> Self {
        Ps3Reader {
            bytes_receiver,
            state: Ps3ReaderState::WaitingForHeader(0),
            events_sender,
            event_builder: None,
        }
    }

    fn reset_state(&mut self) {
        self.state = Ps3ReaderState::WaitingForHeader(0);
    }

    pub async fn work(&mut self) {
        match self.bytes_receiver.recv().await {
            Ok(b) => self.receive_byte(b).await,
            Err(err) => {
                defmt::error!(
                    "fail to receive data from channel: {}",
                    defmt::Debug2Format(&err)
                );
                self.reset_state();
                return;
            }
        };
    }

    async fn receive_byte(&mut self, byte: u8) {
        match self.state {
            Ps3ReaderState::WaitingForHeader(mut h) => {
                if byte == HEADER_BYTE {
                    self.catch_header(&mut h);
                } else {
                    self.reset_state();
                }
                return;
            }

            Ps3ReaderState::WaitingForCommandType => {
                self.catch_command_type(byte);
            }

            Ps3ReaderState::WaitingForDigitalData => {
                self.event_builder.replace(Ps3Event::DigitalSignal(byte));
                self.state = Ps3ReaderState::WaitingForEndByte;
            }

            Ps3ReaderState::WaitingForAnalogData(mut index) => {
                match self.event_builder.as_ref() {
                    Some(event) => {
                        match event {
                            Ps3Event::AnalogSignal(mut bytes) => {
                                bytes[index as usize - 1] = byte;
                            }
                            _ => {
                                //unreachable
                            }
                        }
                    }
                    None => {
                        self.event_builder
                            .replace(Ps3Event::AnalogSignal([0, 0, byte]));
                    }
                };

                index -= 1;
                if index == 0 {
                    self.state = Ps3ReaderState::WaitingForEndByte;
                }
            }

            Ps3ReaderState::WaitingForEndByte => {
                if byte == END_BYTE {
                    self.catch_end_byte().await;
                }
                self.reset_state();
                return;
            }
        }
    }

    fn catch_header(&mut self, header_counter: &mut u8) {
        *header_counter += 1;
        if *header_counter == 2 {
            self.state = Ps3ReaderState::WaitingForCommandType;
        }
    }

    async fn catch_end_byte(&mut self) {
        let event = self.event_builder.take();
        match event {
            Some(event) => {
                let send_res = self.events_sender.send(event).await;
                send_res
                    .err()
                    .map(|_| defmt::error!("fail to send event - No Receiver"));
            }
            None => {
                // unreachable
            }
        }
    }

    fn catch_command_type(&mut self, command: u8) {
        match command {
            COMMAND_CONNECT => {
                self.event_builder.replace(Ps3Event::OnConnect);
                self.state = Ps3ReaderState::WaitingForEndByte;
            }

            COMMAND_DIGITAL => {
                self.state = Ps3ReaderState::WaitingForDigitalData;
            }

            COMMAND_ANALOG => {
                self.state = Ps3ReaderState::WaitingForAnalogData(3);
            }

            _ => {
                defmt::warn!("Unknown command byte: {} - resetting state", command);
                self.reset_state();
            }
        }
    }
}
