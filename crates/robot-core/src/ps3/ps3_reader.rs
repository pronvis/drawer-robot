use rtic_sync::channel::{Receiver, Sender};

//two header bytes come first
const HEADER_BYTE: u8 = 0x11;
const COMMAND_DIGITAL: u8 = 0x12;
const COMMAND_ANALOG: u8 = 0x13;
const COMMAND_CONNECT: u8 = 0x14;
const END_BYTE: u8 = 0x15;
const PS3_CHANNEL_CAPACITY: usize = crate::ps3::CHANNEL_CAPACITY;

const ANALOG_SIGNAL_DATA_LEN: u8 = 3;

#[derive(Debug)]
enum Ps3ReaderState {
    WaitingForHeader(u8),
    WaitingForCommandType,
    WaitingForDigitalData,
    WaitingForAnalogData(u8),
    WaitingForEndByte,
}

struct Ps3EventBuilder {
    buffer: [u8; 3],
    event_type: Ps3EventType,
}

impl Ps3EventBuilder {
    fn new() -> Self {
        Self {
            buffer: [0u8; 3],
            event_type: Ps3EventType::OnConnect,
        }
    }

    fn analog_signal(&mut self, index: u8, byte: u8) {
        self.buffer[index as usize] = byte;
    }

    fn digital_signal(&mut self, byte: u8) {
        self.buffer[0] = byte;
    }

    fn set_event_type(&mut self, et: Ps3EventType) {
        self.event_type = et;
    }

    fn get_event(&self) -> Ps3Event {
        match self.event_type {
            Ps3EventType::OnConnect => Ps3Event::OnConnect,
            Ps3EventType::DigitalSignal => Ps3Event::DigitalSignal(self.buffer[0]),
            Ps3EventType::AnalogSignal => Ps3Event::AnalogSignal(self.buffer),
        }
    }
}

enum Ps3EventType {
    OnConnect,
    DigitalSignal,
    AnalogSignal,
}

/// For Analog Signal:
///   First byte is a 'type':
///      0x01 means left stick
///      0x02 means right stick
///   Second byte is for X axis
///   Third byte is for Y axis
#[derive(Debug)]
pub enum Ps3Event {
    OnConnect,
    DigitalSignal(u8),
    AnalogSignal([u8; 3]),
}

pub struct Ps3Reader {
    bytes_receiver: Receiver<'static, u8, PS3_CHANNEL_CAPACITY>,
    state: Ps3ReaderState,
    // channel to send commands from PS3
    events_sender: Sender<'static, Ps3Event, PS3_CHANNEL_CAPACITY>,
    event_builder: Ps3EventBuilder,
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
            event_builder: Ps3EventBuilder::new(),
        }
    }

    fn reset_state(&mut self) {
        self.state = Ps3ReaderState::WaitingForHeader(0);
    }

    pub async fn work(&mut self) {
        match self.bytes_receiver.recv().await {
            Ok(b) => self.receive_byte(b).await,
            Err(err) => {
                defmt::error!("fail to receive data from channel: {}", defmt::Debug2Format(&err));
                self.reset_state();
                return;
            }
        };
    }

    async fn receive_byte(&mut self, byte: u8) {
        match self.state {
            Ps3ReaderState::WaitingForHeader(h) => {
                if byte == HEADER_BYTE {
                    self.catch_header(h);
                } else {
                    self.reset_state();
                }
                return;
            }

            Ps3ReaderState::WaitingForCommandType => {
                self.catch_command_type(byte);
            }

            Ps3ReaderState::WaitingForDigitalData => {
                self.event_builder.digital_signal(byte);
                self.state = Ps3ReaderState::WaitingForEndByte;
            }

            Ps3ReaderState::WaitingForAnalogData(mut index) => {
                self.event_builder.analog_signal(index, byte);

                index += 1;
                if index == ANALOG_SIGNAL_DATA_LEN {
                    self.state = Ps3ReaderState::WaitingForEndByte;
                } else {
                    self.state = Ps3ReaderState::WaitingForAnalogData(index);
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

    fn catch_header(&mut self, header_counter: u8) {
        if header_counter == 1 {
            self.state = Ps3ReaderState::WaitingForCommandType;
        } else {
            self.state = Ps3ReaderState::WaitingForHeader(header_counter + 1);
        }
    }

    async fn catch_end_byte(&mut self) {
        let event = self.event_builder.get_event();
        let send_res = self.events_sender.send(event).await;
        send_res.err().map(|_| defmt::error!("fail to send event - No Receiver"));
    }

    fn catch_command_type(&mut self, command: u8) {
        match command {
            COMMAND_CONNECT => {
                self.event_builder.set_event_type(Ps3EventType::OnConnect);
                self.state = Ps3ReaderState::WaitingForEndByte;
            }

            COMMAND_DIGITAL => {
                self.event_builder.set_event_type(Ps3EventType::DigitalSignal);
                self.state = Ps3ReaderState::WaitingForDigitalData;
            }

            COMMAND_ANALOG => {
                self.event_builder.set_event_type(Ps3EventType::AnalogSignal);
                self.state = Ps3ReaderState::WaitingForAnalogData(0);
            }

            _ => {
                defmt::warn!("Unknown command byte: {} - resetting state", command);
                self.reset_state();
            }
        }
    }
}
