use crate::CHANNEL_CAPACITY;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::{ExtU32, TimerDurationU32};
use rtic_sync::channel::*;
use stm32f1xx_hal::gpio::*;
use stm32f1xx_hal::gpio::{Cr, Dynamic, Pin, HL};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::timer::{Counter, Event, Instance};

trait Tmc2209Request {
    fn bytes(&self) -> &[u8];
}

impl Tmc2209Request for tmc2209::WriteRequest {
    fn bytes(&self) -> &[u8] {
        self.bytes()
    }
}

impl Tmc2209Request for tmc2209::ReadRequest {
    fn bytes(&self) -> &[u8] {
        self.bytes()
    }
}

#[derive(Debug, Clone)]
pub enum TMC2209SoftSerialCommands {
    Write(tmc2209::WriteRequest),
    Read(tmc2209::ReadRequest),
}

enum ByteSendingState {
    Start,
    Data,
    Stop,
}

pub struct SoftSerial<const pinC: char, const pinN: u8, Tim, const TIMER_CLOCK_FREQ: u32>
where
    Pin<pinC, pinN, Dynamic>: HL,
{
    index: u8, // need for logging
    timer: Counter<Tim, TIMER_CLOCK_FREQ>,

    commands_channel: Receiver<'static, TMC2209SoftSerialCommands, CHANNEL_CAPACITY>,

    bit_index: u8,
    byte_index: u8,
    data_to_write: Option<tmc2209::WriteRequest>,
    data_to_read: Option<tmc2209::ReadRequest>,
    byte_sending_state: ByteSendingState,
    reading: bool,
    reading_starts: bool,
    reading_index: u32,
    pin: Pin<pinC, pinN, Dynamic>,
    reading_data: [u8; 8],
    reading_data_cur: u8,
    bit_value_catcher: u8,
    cathicng_bit_value: bool,
}

impl<const pinC: char, const pinN: u8, Tim: Instance, const TIMER_CLOCK_FREQ: u32>
    SoftSerial<pinC, pinN, Tim, TIMER_CLOCK_FREQ>
where
    Pin<pinC, pinN, Dynamic>: HL,
{
    const BIT_SEND_MICROS_DUR: u32 = 180;
    const READING_BIT_SEND_MICROS_DUR: u32 = 18;
    const READ_BIT_CATCH_COUNT: u8 = 10;
    //TODO: maybe should be smaller when processing g-code. But for PS3 controller it is fine for
    //now.
    const AWAITING_MICROS_DUR: u32 = 500;

    pub fn new(
        index: u8,
        mut pin: Pin<pinC, pinN, Dynamic>,
        timer: Counter<Tim, TIMER_CLOCK_FREQ>,
        commands_channel: Receiver<'static, TMC2209SoftSerialCommands, CHANNEL_CAPACITY>,
        cr: &mut <Pin<pinC, pinN, Dynamic> as HL>::Cr,
    ) -> Self {
        let timer_dur: TimerDurationU32<TIMER_CLOCK_FREQ> = Self::BIT_SEND_MICROS_DUR.micros();
        let bit_send_micros = timer_dur.to_micros();
        assert!(bit_send_micros < 330, "bit send duration > 330. experimentally found that 330 is the maximum which TMC2209 can understand");

        pin.make_push_pull_output(cr);
        pin.set_high().ok().unwrap();

        Self {
            index,
            pin,
            timer,
            commands_channel,
            bit_index: 0,
            byte_index: 0,
            data_to_write: None,
            data_to_read: None,
            byte_sending_state: ByteSendingState::Start,
            reading_index: 0,
            reading_starts: false,
            reading: false,
            reading_data: [0; 8],
            reading_data_cur: 0,
            bit_value_catcher: 0,
            cathicng_bit_value: false,
        }
    }

    /// Receive message from channel and send it via UART.
    /// If new message comes when in process of sending previous one,
    /// then stop sending previous and start last one.
    pub fn work(&mut self, cr: &mut <Pin<pinC, pinN, Dynamic> as HL>::Cr) {
        self.timer.clear_interrupt(Event::Update);

        match self.commands_channel.try_recv() {
            Err(err) => match err {
                ReceiveError::NoSender => {
                    defmt::error!("stepper #{}: commands sender dropped", self.index);
                }
                ReceiveError::Empty => (),
            },

            Ok(new_command) => {
                self.reset_state();
                self.handle_command(new_command, cr);
            }
        }

        if let Some(w_data) = self.data_to_write {
            self.sending_logic(&w_data);
            self.timer
                .start(Self::BIT_SEND_MICROS_DUR.micros())
                .unwrap();
        } else if let Some(r_data) = self.data_to_read {
            self.sending_logic(&r_data);
            if self.reading {
                self.pin.make_pull_up_input(cr);
                // self.pin.make_pull_down_input(cr);
                self.timer
                    .start(Self::READING_BIT_SEND_MICROS_DUR.micros())
                    .unwrap();
            } else {
                self.timer
                    .start(Self::BIT_SEND_MICROS_DUR.micros())
                    .unwrap();
            }
        } else {
            if self.reading {
                self.reading_response();
                self.timer
                    .start(Self::READING_BIT_SEND_MICROS_DUR.micros())
                    .unwrap();
            } else {
                self.timer
                    .start(Self::AWAITING_MICROS_DUR.micros())
                    .unwrap();
            }
        }
    }

    fn reset_state(&mut self) {
        self.byte_index = 0;
        self.bit_index = 0;
        self.data_to_write = None;
        self.bit_value_catcher = 0;
        self.cathicng_bit_value = false;
    }

    fn handle_command(
        &mut self,
        command: TMC2209SoftSerialCommands,
        cr: &mut <Pin<pinC, pinN, Dynamic> as HL>::Cr,
    ) {
        match command {
            TMC2209SoftSerialCommands::Write(wr_req) => {
                self.data_to_write = Some(wr_req);
                self.pin.make_push_pull_output(cr);
                self.data_to_read = None;
                self.reading = false;
                self.reading_index = 0;
                self.reading_starts = false;
                self.reading_data = [0; 8];
                self.reading_data_cur = 0;
            }
            TMC2209SoftSerialCommands::Read(r_req) => {
                self.data_to_read = Some(r_req);
                self.pin.make_push_pull_output(cr);
                self.reading_index = 0;
                self.data_to_write = None;
            }
        }
    }

    fn sending_logic(&mut self, request: &dyn Tmc2209Request) {
        match self.byte_sending_state {
            ByteSendingState::Start => {
                self.byte_sending_state = ByteSendingState::Data;
                self.pin.set_low().ok();
            }

            ByteSendingState::Data => {
                let current_byte: &u8 = request.bytes().get(self.byte_index as usize).unwrap();
                let bit_to_send = Self::bit_value(*current_byte, self.bit_index);
                self.bit_index += 1;
                if self.bit_index == 8 {
                    self.byte_sending_state = ByteSendingState::Stop;
                    self.bit_index = 0;
                }
                if bit_to_send {
                    self.pin.set_high().ok();
                } else {
                    self.pin.set_low().ok();
                }
            }

            ByteSendingState::Stop => {
                self.byte_sending_state = ByteSendingState::Start;
                self.byte_index += 1;
                self.pin.set_high().ok();
                if self.byte_index as usize == request.bytes().len() {
                    defmt::debug!("request have been sent");
                    self.data_to_write = None;
                    self.byte_index = 0;
                    if self.data_to_read.is_some() {
                        self.data_to_read = None;
                        self.reading = true;
                    }
                }
            }
        };
    }

    fn reading_response(&mut self) {
        let bit_value = self.pin.is_low().ok().unwrap();
        if self.bit_value_catcher == 0 {
            self.cathicng_bit_value = bit_value;
            self.bit_value_catcher += 1;
            return;
        }

        if bit_value && self.cathicng_bit_value {
            self.bit_value_catcher += 1;
        } else {
            defmt::debug!("bit change value on: {}", self.bit_value_catcher);
            self.bit_value_catcher = 0;
            self.cathicng_bit_value = bit_value;
        }

        if self.bit_value_catcher < Self::READ_BIT_CATCH_COUNT {
            return;
        }
        self.bit_value_catcher = 0;

        match self.byte_sending_state {
            ByteSendingState::Start => {
                if bit_value {
                    defmt::debug!("receive start bit");
                    self.byte_sending_state = ByteSendingState::Data;
                }
            }

            ByteSendingState::Data => {
                defmt::debug!(
                    "receive data bit #{}, value: {}",
                    self.reading_index,
                    bit_value
                );
                Self::change_byte(
                    &mut self.reading_data_cur,
                    ((self.reading_index) % 8) as u8,
                    bit_value,
                );
                self.reading_index += 1;

                if self.reading_index % 8 == 0 {
                    // defmt::debug!("read 8 bits, byte: {}", self.reading_data_cur);
                    self.byte_sending_state = ByteSendingState::Stop;
                    self.reading_data[self.byte_index as usize] = self.reading_data_cur;
                    self.reading_data_cur = 0;
                    self.byte_index += 1;
                }

                // if self.reading_index % 8 == 0 {
                //     let byte_index = (self.reading_index / 8 - 1) as usize;
                //     self.reading_data[byte_index] = self.reading_data_cur;
                //     self.reading_data_cur = 0;
                // }

                if self.reading_index == 64 {
                    self.reading = false;
                    defmt::debug!("read data: {}", self.reading_data);
                    let mut reader = Reader::default();
                    let (bytes_read, read_resp) = reader.read_response(&self.reading_data);
                    if let Some(read_resp) = read_resp {
                        defmt::debug!("crc is valid: {}", read_resp.crc_is_valid());
                    } else {
                        defmt::debug!("read response is None, bytes read: {}", bytes_read);
                    }

                    self.reading_data = [0; 8];
                    self.reading_data_cur = 0;
                    self.reading_starts = false;
                    self.byte_sending_state = ByteSendingState::Start;
                }
            }

            ByteSendingState::Stop => {
                if !bit_value {
                    defmt::debug!("receive stop bit");
                    self.byte_sending_state = ByteSendingState::Start;
                }
            }
        }

        //if bit_value && !self.reading_starts {
        //    self.reading_starts = true;
        //    self.byte_sending_state = ByteSendingState::Start;
        //}

        ////waiting for tmc2209 to start sending
        //if !self.reading_starts {
        //    return;
        //}
    }

    fn bit_value(byte: u8, index: u8) -> bool {
        (byte >> index & 1) == 1
    }

    fn change_byte(byte: &mut u8, index: u8, data: bool) {
        if data {
            let mask = 1 << index;
            *byte = *byte | mask;
        }
    }
}

#[derive(Default)]
pub struct Reader {
    /// The current index into the response data that we're looking for.
    index: usize,
    /// The data for a response being interpreted.
    response_data: ReadResponseData,
}

impl Reader {
    /// Read a **ReadResponse** from the given slice of bytes.
    ///
    /// The **Reader** will preserve its state between calls to `read`.
    ///
    /// This method is particularly useful when reading from a UART receiver in a non-blocking
    /// manner. For example, if the UART receiver does not have the full response buffered (perhaps
    /// within a UART interrupt), the `Reader` retains its progress and allows for other work to be
    /// performed before attempting to read the remainder of the message.
    ///
    /// Returns the number of bytes read and the **ReadResponse** if one could be read.
    ///
    /// This method works by reading through the bytes for a `SYNC_AND_RESERVED` byte. Once found,
    /// the next byte must be a `MASTER_ADDR` byte. If found, the rest of the data is read directly
    /// into the remainder of the byte slice. Otherwise, the state is rest and the reader goes back
    /// to searching for a `SYNC_AND_RESERVED` byte.
    ///
    /// This function does **not** check the validity of the CRC, the slave address or the register
    /// address.
    pub fn read_response(&mut self, mut bytes: &[u8]) -> (usize, Option<ReadResponse>) {
        let start_len = bytes.len();
        loop {
            // If we're at the first index, we're looking for the sync byte.
            while self.index == ReadResponse::SYNC_AND_RESERVED_IX {
                match bytes.get(0) {
                    Some(&SYNC_AND_RESERVED) => {
                        defmt::debug!("first byte is fine");
                        self.response_data[self.index] = SYNC_AND_RESERVED;
                        self.index += 1;
                    }
                    None => {
                        defmt::debug!(
                            "first byte is None. start_len: {}, bytes.len: {}",
                            start_len,
                            bytes.len()
                        );
                        let read_bytes = start_len - bytes.len();
                        return (read_bytes, None);
                    }
                    _ => (),
                };
                bytes = &bytes[1..];
            }

            // Make sure the following byte is addressed to the master.
            if self.index == ReadResponse::MASTER_ADDR_IX {
                match bytes.get(0) {
                    Some(&MASTER_ADDR) => {
                        self.response_data[self.index] = MASTER_ADDR;
                        self.index += 1;
                        bytes = &bytes[1..];
                        defmt::debug!("second byte is fine");
                    }
                    None => {
                        let read_bytes = start_len - bytes.len();
                        return (read_bytes, None);
                    }
                    _ => {
                        self.index = ReadResponse::SYNC_AND_RESERVED_IX;
                        continue;
                    }
                }
            }

            // Copy the remaining data.
            let remaining_data = &mut self.response_data[self.index..];
            let to_copy = core::cmp::min(remaining_data.len(), bytes.len());
            remaining_data
                .iter_mut()
                .zip(bytes)
                .for_each(|(d, b)| *d = *b);
            self.index += to_copy;
            bytes = &bytes[to_copy..];

            // Return a response if we've read one.
            let read_bytes = start_len - bytes.len();
            return if self.index == ReadResponse::LEN_BYTES {
                self.index = 0;
                (read_bytes, Some(ReadResponse(self.response_data)))
            } else {
                (read_bytes, None)
            };
        }
    }
}

/// The first four bits for synchronisation, the last four are zeroed reserved bytes.
pub const SYNC_AND_RESERVED: u8 = 0b00000101;

/// Responses are always addressed to the master with this value.
pub const MASTER_ADDR: u8 = 0b11111111;

type ReadResponseData = [u8; ReadResponse::LEN_BYTES];
pub struct ReadResponse(ReadResponseData);
impl ReadResponse {
    /// The length of the message in bytes.
    pub const LEN_BYTES: usize = 8;
    /// The first byte is the synchronisation byte.
    pub const SYNC_AND_RESERVED_IX: usize = 0;
    /// The index of the master address byte.
    pub const MASTER_ADDR_IX: usize = 1;
    /// The index of the register address.
    pub const REG_ADDR_IX: usize = 2;
    /// The range of byte representing the data field.
    pub const DATA_RANGE: core::ops::Range<usize> = 3..7;
    /// The index of the cyclic rendundency check.
    pub const CRC_IX: usize = 7;
    /// Returns `true` if the CRC is valid, `false` otherwise.
    pub fn crc_is_valid(&self) -> bool {
        self.0[Self::CRC_IX] == crc(&self.0[..Self::CRC_IX])
    }
}

/// Cyclic redundancy check.
///
/// Given all data of a datagram apart from the final CRC byte, return what the CRC byte should be.
pub fn crc(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for mut byte in data.iter().cloned() {
        for _ in 0..8 {
            if ((crc >> 7) ^ (byte & 0x01)) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    crc
}
