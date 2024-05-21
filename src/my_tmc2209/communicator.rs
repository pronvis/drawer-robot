use embedded_hal::digital::v2::{InputPin, OutputPin};
use stm32f1xx_hal::{
    gpio::{Dynamic, Pin, HL},
    timer::{Counter, Event, Instance},
};
use tmc2209::{ReadRequest, ReadResponse, WriteRequest};

const OVERSAMPLE: u8 = 3;
//'default=8 bit times', but looks like it is too much and we skip some data if 'SWITCH_DELAY=8'
const SWITCH_DELAY: u8 = 8 - 1;
const MAX_RX_BUFFER_SIZE: usize = 64;

use defmt::Format;
#[derive(PartialEq, Debug, Format)]
enum CommunicatorState {
    Nothing,
    Writing,
    Reading,
}

pub struct TMC2209SerialCommunicator<const PIN_C: char, const PIN_N: u8, TIM: Instance, const TIMER_CLOCK_FREQ: u32>
where
    Pin<PIN_C, PIN_N, Dynamic>: HL,
{
    tx_tick_counter: u8, // interrupt tick counter for TX
    tx_bit_counter: u8,  //TODO: probably I dont need it, cause can use 'tx_bits_buffer == 0' to
    //find the end
    tx_bits_buffer: u16,     // buffer for byte + stop & start bits
    bytes_to_send: [u8; 8],  // buffer for bytes to send
    tx_bytes_counter: usize, // counter for sended bytes

    current_state: CommunicatorState, // should be Sending, Receiving, Nothing
    read_after_write: bool,

    receive_buffer: [u8; MAX_RX_BUFFER_SIZE],
    rx_bit_counter: i8,
    rx_tick_counter: u8,
    rx_buffer: u8,
    receive_buffer_tail: usize,
    receive_buffer_head: usize,
    bytes_to_read: u8,
    already_prepared_to_read_response: bool,

    pin: Pin<PIN_C, PIN_N, Dynamic>,
    cr: <Pin<PIN_C, PIN_N, Dynamic> as HL>::Cr,

    timer: Counter<TIM, TIMER_CLOCK_FREQ>,
}

impl<const PIN_C: char, const PIN_N: u8, TIM: Instance, const TIMER_CLOCK_FREQ: u32>
    TMC2209SerialCommunicator<PIN_C, PIN_N, TIM, TIMER_CLOCK_FREQ>
where
    Pin<PIN_C, PIN_N, Dynamic>: HL,
{
    pub fn new(timer: Counter<TIM, TIMER_CLOCK_FREQ>, pin: Pin<PIN_C, PIN_N, Dynamic>, cr: <Pin<PIN_C, PIN_N, Dynamic> as HL>::Cr) -> Self {
        Self {
            tx_tick_counter: 0,
            tx_bit_counter: 0,
            tx_bits_buffer: 0,
            bytes_to_send: [0; 8],
            tx_bytes_counter: 0,

            current_state: CommunicatorState::Nothing,
            read_after_write: false,

            already_prepared_to_read_response: false,
            receive_buffer: [0; MAX_RX_BUFFER_SIZE],
            rx_bit_counter: 0,
            rx_tick_counter: 0,
            rx_buffer: 0,
            receive_buffer_tail: 0,
            receive_buffer_head: 0,
            bytes_to_read: 0,

            pin,
            cr,

            timer,
        }
    }

    pub fn send(&mut self, data: &[u8]) {
        match data.len() {
            WriteRequest::LEN_BYTES => {
                self.prepare_to_send_write_req(data);
            }
            ReadRequest::LEN_BYTES => {
                self.prepare_to_send_read_req(data);
            }
            //TODO: return Error
            _ => return,
        }
        self.timer.listen(Event::Update);
    }

    fn prepare_to_send(&mut self, data: &[u8]) {
        self.tx_bit_counter = 0;
        self.tx_tick_counter = 1; //send first bit without OVERSAMPLING
        self.tx_bytes_counter = data.len();
        // cause 'tx_bytes_counter' starts from 'len' we need to reverse
        for (index, elem) in data.iter().rev().enumerate() {
            self.bytes_to_send[index] = *elem;
        }

        self.tx_bits_buffer = Self::add_start_and_stop_bits(self.bytes_to_send[self.tx_bytes_counter - 1]);
        self.current_state = CommunicatorState::Writing;
        self.pin.make_push_pull_output(&mut self.cr);
        self.pin.set_high().ok().unwrap();
    }

    fn prepare_to_send_write_req(&mut self, data: &[u8]) {
        if self.current_state != CommunicatorState::Nothing {
            //TODO: return Error
            return;
        }

        self.prepare_to_send(data);
        self.read_after_write = false;
    }

    fn prepare_to_send_read_req(&mut self, data: &[u8]) {
        if self.current_state != CommunicatorState::Nothing {
            //TODO: return Error
            return;
        }

        self.prepare_to_send(data);
        self.already_prepared_to_read_response = false;
        self.read_after_write = true;
    }

    fn write_bit(&mut self) {
        // if tx_tick_counter > 0 interrupt is discarded. Only when tx_tick_counter reach 0 we set TX pin.
        self.tx_tick_counter -= 1;
        if self.tx_tick_counter == 0 {
            //send first bit without OVERSAMPLING
            self.tx_bit_counter += 1;
            // 10 = stop, end + 8 bits
            if self.tx_bit_counter <= 10 {
                if self.tx_bits_buffer & 1 == 1 {
                    self.pin.set_high().ok().unwrap();
                } else {
                    self.pin.set_low().ok().unwrap();
                }
                self.tx_bits_buffer >>= 1;
                self.tx_tick_counter = OVERSAMPLE;
            } else {
                // byte has been sended
                if self.tx_bytes_counter > 0 {
                    self.tx_bytes_counter -= 1;
                }

                if self.tx_bytes_counter == 0 {
                    // all bytes has been sended
                    self.tx_tick_counter = 1;

                    if !self.read_after_write {
                        self.current_state = CommunicatorState::Nothing;
                    } else {
                        // we want to change state of the Pin immediately
                        // and wait for SWITCH_DELAY after
                        if self.tx_bit_counter > 10 + OVERSAMPLE * SWITCH_DELAY {
                            self.current_state = CommunicatorState::Reading;
                        } else {
                            if !self.already_prepared_to_read_response {
                                self.prepare_to_read_response();
                            }
                        }
                    }
                } else {
                    // continue to next byte
                    self.tx_bits_buffer = Self::add_start_and_stop_bits(self.bytes_to_send[self.tx_bytes_counter - 1]);
                    self.tx_tick_counter = OVERSAMPLE;
                    self.tx_bit_counter = 0;
                }
            }
        }
    }

    fn prepare_to_read_response(&mut self) {
        self.already_prepared_to_read_response = true;
        self.rx_bit_counter = -1; // rx_bit_counter = -1 :  waiting for start bit
        self.rx_tick_counter = 2; // 2 : next interrupt will be discarded. 2 interrupts required to consider RX pin level
        self.rx_buffer = 0;
        self.bytes_to_read = ReadResponse::LEN_BYTES as u8;
        self.pin.make_pull_up_input(&mut self.cr);
    }

    fn read_bit(&mut self) {
        self.rx_tick_counter -= 1;
        if self.rx_tick_counter <= 0 {
            // if rx_tick_counter > 0 interrupt is discarded. Only when rx_tick_counter reach 0 RX pin is considered
            let bit_value = self.pin.is_high().ok().unwrap();
            if self.rx_bit_counter == -1 {
                // rx_bit_counter = -1 :  waiting for start bit
                if !bit_value {
                    // got start bit
                    self.rx_bit_counter = 0; // rx_bit_counter == 0 : start bit received
                    self.rx_tick_counter = OVERSAMPLE + 1; // Wait 1 bit (OVERSAMPLE ticks) + 1 tick in order to sample RX pin in the middle of the edge (and not too close to the edge)
                    self.rx_buffer = 0;
                } else {
                    self.rx_tick_counter = 1; // Waiting for start bit, but we don't get right level. Wait for next Interrupt to ckech RX pin level
                }
            } else if self.rx_bit_counter >= 8 {
                // rx_bit_counter >= 8 : waiting for stop bit
                if bit_value {
                    // stop bit read complete, add byte to buffer
                    let next = (self.receive_buffer_tail + 1) % MAX_RX_BUFFER_SIZE;
                    if next != self.receive_buffer_head {
                        // save new data in buffer: tail points to where byte goes
                        self.receive_buffer[self.receive_buffer_tail] = self.rx_buffer; // save new byte
                        self.receive_buffer_tail = next;
                    } else {
                        // TODO: what to do if overflow?
                        defmt::debug!(
                            "buffer overflow. tail: {}, head: {}",
                            self.receive_buffer_tail,
                            self.receive_buffer_head
                        );
                    }
                }
                // if there is no stop bit then skip current byte.
                // Full frame received. Restart waiting for start bit at next interrupt
                self.rx_tick_counter = 1;
                self.rx_bit_counter = -1;
                self.bytes_to_read -= 1;
                if self.bytes_to_read == 0 {
                    self.change_state_to_end_reading();
                }
            } else {
                // data bits
                self.rx_buffer >>= 1;
                if bit_value {
                    self.rx_buffer |= 0x80;
                }
                self.rx_bit_counter += 1; // Prepare for next bit
                self.rx_tick_counter = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
            }
        }
    }

    fn change_state_to_end_reading(&mut self) {
        self.current_state = CommunicatorState::Nothing;
        self.pin.make_push_pull_output(&mut self.cr);
    }

    pub fn get_response(&mut self) -> Option<[u8; ReadResponse::LEN_BYTES]> {
        if self.have_response() {
            let mut response: [u8; ReadResponse::LEN_BYTES] = [0; ReadResponse::LEN_BYTES];
            for i in 0..ReadResponse::LEN_BYTES {
                response[i] = self.receive_buffer[self.receive_buffer_head];
                self.receive_buffer_head = (self.receive_buffer_head + 1) % MAX_RX_BUFFER_SIZE;
            }

            return Some(response);
        }

        return None;
    }

    pub fn handle_interrupt(&mut self) {
        match self.current_state {
            CommunicatorState::Reading => self.read_bit(),
            CommunicatorState::Writing => self.write_bit(),
            CommunicatorState::Nothing => self.stop_listening(),
        }
        self.timer.clear_interrupt(Event::Update);
    }

    fn stop_listening(&mut self) {
        self.timer.unlisten(Event::Update);
    }

    fn add_start_and_stop_bits(byte_to_send: u8) -> u16 {
        return (byte_to_send as u16) << 1 | 0x200;
    }

    pub fn have_response(&self) -> bool {
        let tail = self.receive_buffer_tail;
        let head = self.receive_buffer_head;
        if head > tail {
            return (head + ReadResponse::LEN_BYTES) % MAX_RX_BUFFER_SIZE >= tail;
        } else if tail > head {
            return tail - head >= ReadResponse::LEN_BYTES;
        } else {
            // head == tail
            return false;
        }
    }
}
