use rtic_monotonics::systick::Systick;
use stm32f1xx_hal::gpio::{Dynamic, Pin, HL};

use fugit::ExtU32;
use rtic_sync::channel::*;

const CHANNEL_CAPACITY: usize = crate::my_tmc2209::communicator::CHANNEL_CAPACITY;

pub struct Configurator {
    response_index: u32,
    request_index: u32,
    ifcnt: u8,
    finished: bool,
    first_call: bool,
    sender: Sender<'static, crate::my_tmc2209::Request, CHANNEL_CAPACITY>,
}

impl Configurator {
    pub fn new(sender: Sender<'static, crate::my_tmc2209::Request, CHANNEL_CAPACITY>) -> Self {
        Self {
            sender,
            response_index: 0,
            request_index: 0,
            ifcnt: 0,
            finished: false,
            first_call: true,
        }
    }

    pub async fn setup(&mut self, tmc2209_rsp_receiver: &mut Receiver<'static, u32, CHANNEL_CAPACITY>) {
        self.update_ifcnt_val(tmc2209_rsp_receiver).await.ok();

        // SET SENDDELAY time setting to 8 bit times
        let ifcnt_before_req = self.ifcnt;
        while self.ifcnt == ifcnt_before_req {
            defmt::debug!("IN WHILE LOOP. ifcnt_before_req: {}, self.ifcnt: {}", ifcnt_before_req, self.ifcnt);
            let write_req = tmc2209::write_request(0, tmc2209::reg::SLAVECONF(0));
            let req = crate::my_tmc2209::Request::write(write_req);
            let _ = self.sender.send(req).await;
            // Systick::delay(100.millis()).await;
            self.update_ifcnt_val(tmc2209_rsp_receiver).await.ok();
        }
    }

    async fn update_ifcnt_val(&mut self, tmc2209_rsp_receiver: &mut Receiver<'static, u32, CHANNEL_CAPACITY>) -> Result<(), ()> {
        self.send_ifcnt_req().await;
        let ifcnt = Self::read_ifcnt_resp(tmc2209_rsp_receiver).await?;

        // let mut resp = Self::read_ifcnt_resp(tmc2209_rsp_receiver).await;
        // if resp.is_err() {
        //     return Result::Err(());
        // }
        // while resp.is_err() {
        //     self.send_ifcnt_req().await;
        //     resp = Self::read_ifcnt_resp(tmc2209_rsp_receiver).await;
        // }
        // let ifcnt = resp.ok().unwrap();

        defmt::debug!("receive ifcnt response: {}; curr ifcnt: {}", ifcnt, self.ifcnt);

        if !self.first_call && self.ifcnt + 1 != ifcnt {
            defmt::debug!("returning error for some case");
            return Result::Err(());
        } else {
            if self.first_call {
                self.first_call = false;
            }
            self.ifcnt = ifcnt;
            return Ok(());
        }
    }

    async fn send_ifcnt_req(&mut self) {
        let read_req = tmc2209::read_request::<tmc2209::reg::IFCNT>(0);
        let req = crate::my_tmc2209::Request::read(read_req);
        self.sender.send(req).await.ok();
    }

    async fn read_ifcnt_resp(tmc2209_rsp_receiver: &mut Receiver<'static, u32, CHANNEL_CAPACITY>) -> Result<u8, ()> {
        let response = tmc2209_rsp_receiver.recv().await.unwrap();
        if response == u32::MAX {
            return Result::Err(());
        }

        let ifcnt_resp = tmc2209::reg::IFCNT::from(response);
        return Ok(ifcnt_resp.0 as u8);
    }
}
