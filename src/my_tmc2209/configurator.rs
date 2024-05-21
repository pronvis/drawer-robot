use stm32f1xx_hal::{
    gpio::{Dynamic, Pin, HL},
    timer::Instance,
};

use super::communicator::TMC2209SerialCommunicator;

pub struct Configurator {
    response_index: u32,
    request_index: u32,
    ifcnt: u8,
}

impl Configurator {
    pub fn new() -> Self {
        Self {
            response_index: 0,
            request_index: 0,
            ifcnt: 0,
        }
    }

    pub fn setup<const PIN_C: char, const PIN_N: u8, TIM: Instance, const TIMER_CLOCK_FREQ: u32>(
        &mut self,
        communicator: &mut TMC2209SerialCommunicator<PIN_C, PIN_N, TIM, TIMER_CLOCK_FREQ>,
    ) where
        Pin<PIN_C, PIN_N, Dynamic>: HL,
    {
        if self.request_index == self.response_index && self.request_index == 0 {
            let read_req = tmc2209::read_request::<tmc2209::reg::IFCNT>(0);
            communicator.send(read_req.bytes());
            self.request_index += 1;
            defmt::debug!("Read request has been sent");
        } else if self.request_index > self.response_index {
            if let Some(response_data) = communicator.get_response() {
                let mut reader = tmc2209::Reader::default();
                let (bytes_read, tmc_response) = reader.read_response(&response_data);
                match tmc_response {
                    Some(response) => {
                        let ifcnt = tmc2209::reg::IFCNT::from(response.data_u32());
                        self.ifcnt = ifcnt.0 as u8;
                        defmt::debug!("Get Resp in Configurator: {}", self.ifcnt);
                    }
                    None => {
                        defmt::debug!("No response, bytes_read: {}, data from reader was: {:?}", bytes_read, response_data);
                    }
                }
            } else {
                defmt::debug!("no response");
            }
        }
    }

    pub fn finished(&self) -> bool {
        //TODO:
        return self.ifcnt > 0;
    }
}
