use crate::my_tmc2209::Request;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use rtic_sync::channel::{Receiver, Sender};
use stm32f1xx_hal::gpio::{Dynamic, Pin, HL};
use tmc2209::ReadResponse;

const OVERSAMPLE: u8 = 3;
//'default=8 bit times', but looks like it is too much and we skip some data if 'SWITCH_DELAY=8'
//I choose 5 for the safety
const SWITCH_TO_READ_DELAY: u8 = 5;
const MAX_RX_BUFFER_SIZE: usize = ReadResponse::LEN_BYTES;

pub const CHANNEL_CAPACITY: usize = 8;

use defmt::Format;
#[derive(PartialEq, Debug, Format)]
enum CommunicatorState {
    Nothing,
    Writing,
    Reading,
}

pub struct TMC2209SerialCommunicator<const PIN_C: char, const PIN_N: u8>
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
    bytes_to_read: u8,
    already_prepared_to_read_response: bool,
    bytes_read: u8,

    idle_counter: u32,
    already_waited_for_idle: bool,
    transmission_restart_counter: u32,
    bits_to_wait_after_get_response: u32,

    pin: Pin<PIN_C, PIN_N, Dynamic>,
    cr: <Pin<PIN_C, PIN_N, Dynamic> as HL>::Cr,

    commands_channel: Receiver<'static, Request, CHANNEL_CAPACITY>,
    responses_channel: Sender<'static, u32, CHANNEL_CAPACITY>,
}

impl<const PIN_C: char, const PIN_N: u8> TMC2209SerialCommunicator<PIN_C, PIN_N>
where
    Pin<PIN_C, PIN_N, Dynamic>: HL,
{
    pub fn new(
        commands_channel: Receiver<'static, Request, CHANNEL_CAPACITY>,
        responses_channel: Sender<'static, u32, CHANNEL_CAPACITY>,
        pin: Pin<PIN_C, PIN_N, Dynamic>,
        cr: <Pin<PIN_C, PIN_N, Dynamic> as HL>::Cr,
    ) -> Self {
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
            bytes_to_read: 0,
            bytes_read: 0,

            idle_counter: 0,
            already_waited_for_idle: true,
            transmission_restart_counter: 0,
            bits_to_wait_after_get_response: 0,

            pin,
            cr,

            commands_channel,
            responses_channel,
        }
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
        self.prepare_to_send(data);
        self.read_after_write = false;
    }

    fn prepare_to_send_read_req(&mut self, data: &[u8]) {
        self.prepare_to_send(data);
        self.already_prepared_to_read_response = false;
        self.read_after_write = true;
    }

    fn write_bit(&mut self) {
        // if tx_tick_counter > 0 interrupt is discarded. Only when tx_tick_counter reach 0 we set TX pin.
        self.tx_tick_counter -= 1;
        if self.tx_tick_counter == 0 {
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
                        if self.tx_bit_counter > 10 + OVERSAMPLE * SWITCH_TO_READ_DELAY {
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
        self.rx_tick_counter = 1; // 2 : next interrupt will be discarded. 2 interrupts required to consider RX pin level
        self.rx_buffer = 0;
        self.bytes_to_read = ReadResponse::LEN_BYTES as u8;
        self.pin.make_pull_up_input(&mut self.cr);
        self.bytes_read = 0;
        self.receive_buffer.fill(0);
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
                    self.rx_tick_counter = OVERSAMPLE + OVERSAMPLE / 2; // Wait 1 bit (OVERSAMPLE ticks) + 1 tick in order to sample RX pin in the middle of the edge (and not too close to the edge)
                    self.rx_buffer = 0;
                } else {
                    self.rx_tick_counter = 1; // Waiting for start bit, but we don't get right level. Wait for next Interrupt to ckech RX pin level
                }
            } else if self.rx_bit_counter >= 8 {
                // rx_bit_counter >= 8 : waiting for stop bit
                if bit_value {
                    let byte_index = ReadResponse::LEN_BYTES - self.bytes_to_read as usize;
                    self.bytes_read += 1;
                    // stop bit read complete, add byte to buffer
                    self.receive_buffer[byte_index] = self.rx_buffer; // save new byte
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

    fn have_response(&self) -> bool {
        self.bytes_read == ReadResponse::LEN_BYTES as u8
    }

    fn send_fail_response(&mut self) {
        let _ = self.responses_channel.try_send(u32::MAX);
    }

    fn change_state_to_end_reading(&mut self) {
        if self.have_response() {
            let mut reader = tmc2209::Reader::default();
            let (_, tmc_response) = reader.read_response(&self.receive_buffer);
            if let Some(response_data) = tmc_response {
                if !response_data.crc_is_valid() {
                    defmt::debug!("CRC IS NOT VALID!");
                }
                let _ = self.responses_channel.try_send(response_data.data_u32());
            } else {
                defmt::warn!("fail to parse response, data: {:?}", self.receive_buffer);
                self.send_fail_response();
            }
        } else {
            // defmt::debug!("response is not fully read, bytes read: {}", self.bytes_read);
            self.send_fail_response();
        }

        //from tmc2209 datasheed: "In case of a read access, it switches on its output drivers and sends its response using the same baud rate.
        // The output becomes switched off four bit times after transfer of the last stop bit"
        self.bits_to_wait_after_get_response = 4 * OVERSAMPLE as u32;
        self.current_state = CommunicatorState::Nothing;
        self.pin.make_push_pull_output(&mut self.cr);
        self.pin.set_high().ok().unwrap();
    }

    pub fn handle_interrupt(&mut self) {
        match self.current_state {
            CommunicatorState::Reading => self.read_bit(),
            CommunicatorState::Writing => self.write_bit(),
            CommunicatorState::Nothing => {
                if self.deal_with_output_driver_delay() {
                    return;
                }

                if self.deal_with_tmc_communication_reset() {
                    return;
                }

                if let Some(command) = self.commands_channel.try_recv().ok() {
                    self.already_waited_for_idle = false;
                    self.idle_counter = 0;
                    match command {
                        Request::Read(read_req) => self.prepare_to_send_read_req(read_req.bytes()),
                        Request::Write(write_req) => self.prepare_to_send_write_req(write_req.bytes()),
                    };
                }
            }
        }
    }

    fn deal_with_output_driver_delay(&mut self) -> bool {
        if self.bits_to_wait_after_get_response > 0 {
            self.bits_to_wait_after_get_response -= 1;
            return true;
        }

        return false;
    }

    fn deal_with_tmc_communication_reset(&mut self) -> bool {
        if self.already_waited_for_idle {
            return false;
        }

        if self.transmission_restart_counter > 0 {
            self.transmission_restart_counter -= 1;
            if self.transmission_restart_counter == 0 {
                self.already_waited_for_idle = true;
            }

            return true;
        } else {
            self.idle_counter += 1;
            let bit_time_idle = self.idle_counter / OVERSAMPLE as u32;

            // from tmc2209 datasheet:
            // "The communication becomes reset if a pause time of longer than 63 bit times between the start bits of two successive bytes occurs"
            if bit_time_idle >= 64 {
                // wait for minimum 12 bit times
                self.transmission_restart_counter = 12 * OVERSAMPLE as u32;
                self.idle_counter = 0;
                return true;
            } else {
                return false;
            }
        }
    }

    fn add_start_and_stop_bits(byte_to_send: u8) -> u16 {
        return (byte_to_send as u16) << 1 | 0x200;
    }
}
