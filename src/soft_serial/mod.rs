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
    reading_index: u32,
    pin: Pin<pinC, pinN, Dynamic>,
    reading_data: [u8; 8],
    reading_data_cur: u8,
}

impl<const pinC: char, const pinN: u8, Tim: Instance, const TIMER_CLOCK_FREQ: u32>
    SoftSerial<pinC, pinN, Tim, TIMER_CLOCK_FREQ>
where
    Pin<pinC, pinN, Dynamic>: HL,
{
    const BIT_SEND_MICROS_DUR: u32 = 18;
    const READING_BIT_SEND_MICROS_DUR: u32 = 50;
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
            reading: false,
            reading_data: [0; 8],
            reading_data_cur: 0,
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
            self.timer
                .start(Self::BIT_SEND_MICROS_DUR.micros())
                .unwrap();
            if self.reading {
                self.pin.make_pull_up_input(cr);
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
                if self.bit_index > 7 {
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
                    self.data_to_write = None;
                    if self.data_to_read.is_some() {
                        self.data_to_read = None;
                        self.reading = true;
                    }
                }
            }
        };
    }

    fn reading_response(&mut self) {
        let bit_value = self.pin.is_high().ok().unwrap();
        Self::change_byte(
            &mut self.reading_data_cur,
            ((self.reading_index) % 8) as u8,
            bit_value,
        );
        defmt::debug!(
            "{} bit value: {}, and current byte: {}",
            self.reading_index,
            bit_value,
            self.reading_data_cur
        );
        self.reading_index += 1;

        if self.reading_index % 8 == 0 {
            let byte_index = (self.reading_index / 8 - 1) as usize;
            self.reading_data[byte_index] = self.reading_data_cur;
            self.reading_data_cur = 0;
        }

        if self.reading_index == 64 {
            self.reading = false;
            defmt::debug!("read data: {}", self.reading_data);
            self.reading_data = [0; 8];
            self.reading_data_cur = 0;
        }
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
