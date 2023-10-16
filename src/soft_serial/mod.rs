use embedded_hal::digital::v2::OutputPin;
use fugit::{ExtU32, TimerDurationU32};
use stm32f1xx_hal::timer::{Counter, Event, Instance};

pub struct SoftSerial<T, Tim, const TIMER_CLOCK_FREQ: u32> {
    pin: T,
    timer: Counter<Tim, TIMER_CLOCK_FREQ>,

    //move this data to some other struct
    current_bit: u8,
    current_byte: usize,
    sending_state: SendingState,
    bytes_to_send: [u8; 8],
}

enum SendingState {
    Start,
    Data,
    Stop,
}

impl<T: OutputPin, Tim: Instance, const TIMER_CLOCK_FREQ: u32>
    SoftSerial<T, Tim, TIMER_CLOCK_FREQ>
{
    const BIT_SEND_MICROS_DUR: u32 = 18;

    pub fn new(mut pin: T, timer: Counter<Tim, TIMER_CLOCK_FREQ>, bytes_to_send: [u8; 8]) -> Self {
        let timer_dur: TimerDurationU32<TIMER_CLOCK_FREQ> = Self::BIT_SEND_MICROS_DUR.micros();
        let bit_send_micros = timer_dur.to_micros();
        assert!(bit_send_micros < 330, "bit send duration > 330. experimentally found that 330 is the maximum which TMC2209 can understand");

        pin.set_high().ok();
        Self {
            pin,
            timer,
            current_bit: 0,
            current_byte: 0,
            sending_state: SendingState::Start,
            bytes_to_send,
        }
    }

    pub fn work(&mut self) {
        if self.current_byte == self.bytes_to_send.len() {
            return;
        }

        let current_byte: &u8 = self.bytes_to_send.get(self.current_byte).unwrap();
        self.send_byte(*current_byte);
    }

    fn send_byte(&mut self, byte: u8) {
        match self.sending_state {
            SendingState::Start => {
                self.sending_state = SendingState::Data;
                self.pin.set_low().ok();
            }

            SendingState::Data => {
                let bit_to_send = Self::bit_value(byte, self.current_bit);
                self.current_bit += 1;
                if self.current_bit > 7 {
                    self.sending_state = SendingState::Stop;
                    self.current_bit = 0;
                }
                if bit_to_send {
                    self.pin.set_high().ok();
                } else {
                    self.pin.set_low().ok();
                }
            }

            SendingState::Stop => {
                self.sending_state = SendingState::Start;
                self.current_byte += 1;
                self.pin.set_high().ok();
            }
        };

        self.timer
            .start(Self::BIT_SEND_MICROS_DUR.micros())
            .unwrap();
    }

    fn bit_value(byte: u8, index: u8) -> bool {
        (byte >> index & 1) == 1
    }

    // fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
    //     for &w in buffer {
    //         nb::block!(self.write(w))?;
    //     }
    //     Ok(())
    // }

    // pub fn write(&mut self, word: u8) -> nb::Result<(), Infallible> {
    //     self.write_u16(word as u16)
    // }

    // pub fn write_u16(&mut self, word: u16) -> nb::Result<(), Infallible> {
    //     let usart = unsafe { &*USART::ptr() };

    //     if usart.sr.read().txe().bit_is_set() {
    //         usart.dr.write(|w| w.dr().bits(word));
    //         Ok(())
    //     } else {
    //         Err(nb::Error::WouldBlock)
    //     }
    // }
}
