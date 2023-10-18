use crate::CHANNEL_CAPACITY;
use embedded_hal::digital::v2::OutputPin;
use fugit::{ExtU32, TimerDurationU32};
use rtic_sync::channel::*;
use stm32f1xx_hal::timer::{Counter, Event, Instance};

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

pub struct SoftSerial<T, Tim, const TIMER_CLOCK_FREQ: u32> {
    index: u8, // need for logging
    pin: T,
    timer: Counter<Tim, TIMER_CLOCK_FREQ>,

    commands_channel: Receiver<'static, TMC2209SoftSerialCommands, CHANNEL_CAPACITY>,

    bit_index: u8,
    byte_index: u8,
    data_to_write: Option<tmc2209::WriteRequest>,
    byte_sending_state: ByteSendingState,
}

impl<T: OutputPin, Tim: Instance, const TIMER_CLOCK_FREQ: u32>
    SoftSerial<T, Tim, TIMER_CLOCK_FREQ>
{
    const BIT_SEND_MICROS_DUR: u32 = 18;
    //TODO: maybe should be smaller when processing g-code. But for PS3 controller it is fine for
    //now.
    const AWAITING_MICROS_DUR: u32 = 500;

    pub fn new(
        index: u8,
        mut pin: T,
        timer: Counter<Tim, TIMER_CLOCK_FREQ>,
        commands_channel: Receiver<'static, TMC2209SoftSerialCommands, CHANNEL_CAPACITY>,
    ) -> Self {
        let timer_dur: TimerDurationU32<TIMER_CLOCK_FREQ> = Self::BIT_SEND_MICROS_DUR.micros();
        let bit_send_micros = timer_dur.to_micros();
        assert!(bit_send_micros < 330, "bit send duration > 330. experimentally found that 330 is the maximum which TMC2209 can understand");

        pin.set_high().ok();
        Self {
            index,
            pin,
            timer,
            commands_channel,
            bit_index: 0,
            byte_index: 0,
            data_to_write: None,
            byte_sending_state: ByteSendingState::Start,
        }
    }

    /// Receive message from channel and send it via UART.
    /// If new message comes when in process of sending previous one,
    /// then stop sending previous and start last one.
    pub fn work(&mut self) {
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
                self.handle_command(new_command);
            }
        }

        match self.data_to_write {
            None => self
                .timer
                .start(Self::AWAITING_MICROS_DUR.micros())
                .unwrap(),
            Some(wr_data) => {
                self.sending_logic(wr_data);
                self.timer
                    .start(Self::BIT_SEND_MICROS_DUR.micros())
                    .unwrap()
            }
        }
    }

    fn reset_state(&mut self) {
        self.byte_index = 0;
        self.bit_index = 0;
        self.data_to_write = None;
    }

    fn handle_command(&mut self, command: TMC2209SoftSerialCommands) {
        match command {
            TMC2209SoftSerialCommands::Write(wr_req) => {
                self.data_to_write = Some(wr_req);
            }
            _ => {
                todo!();
            }
        }
    }

    fn sending_logic(&mut self, wr_data: tmc2209::WriteRequest) {
        match self.byte_sending_state {
            ByteSendingState::Start => {
                self.byte_sending_state = ByteSendingState::Data;
                self.pin.set_low().ok();
            }

            ByteSendingState::Data => {
                let current_byte: &u8 = wr_data.bytes().get(self.byte_index as usize).unwrap();
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
                if self.byte_index as usize == wr_data.bytes().len() {
                    self.data_to_write = None;
                }
            }
        };
    }

    fn bit_value(byte: u8, index: u8) -> bool {
        (byte >> index & 1) == 1
    }
}
