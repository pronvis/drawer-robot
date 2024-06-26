use crate::my_tmc2209::{Tmc2209RequestProducer, Tmc2209ResponseConsumer};

pub struct TMC2209Configurator {
    ifcnt: u8,
    first_call: bool,
    req_sender: Tmc2209RequestProducer,
    rsp_receiver: Tmc2209ResponseConsumer,
}

impl TMC2209Configurator {
    const MAX_SAME_REQ_COUNT: u8 = 5;

    pub fn new(req_sender: Tmc2209RequestProducer, rsp_receiver: Tmc2209ResponseConsumer) -> Self {
        Self {
            req_sender,
            rsp_receiver,
            ifcnt: 0,
            first_call: true,
        }
    }

    pub async fn setup(&mut self) -> Result<(), ()> {
        self.update_ifcnt_val().await?;

        // set wait time before sending read response to 8 bit times
        let write_req = tmc2209::write_request(0, tmc2209::reg::SLAVECONF(0));
        self.send_req_and_check(write_req).await?;

        let mut gconf = tmc2209::reg::GCONF(0);
        gconf.set_i_scale_analog(true);
        gconf.set_internal_rsense(false);
        gconf.set_en_spread_cycle(true);
        gconf.set_pdn_disable(true);
        gconf.set_mstep_reg_select(true);
        gconf.set_multistep_filt(true);
        gconf.set_test_mode(false);
        let write_req = tmc2209::write_request(0, gconf);
        self.send_req_and_check(write_req).await?;

        let mut i_hold_irun = tmc2209::reg::IHOLD_IRUN(0);
        i_hold_irun.set_ihold(31); // maximum current when holding
        i_hold_irun.set_irun(31); // maximum current when moving
        i_hold_irun.set_ihold_delay(0);
        let write_req = tmc2209::write_request(0, i_hold_irun);
        self.send_req_and_check(write_req).await?;

        //set speed to 0
        let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(0));
        self.send_req_and_check(write_req).await?;

        return Result::Ok(());
    }

    async fn send_req_and_check(&mut self, write_req: tmc2209::WriteRequest) -> Result<(), ()> {
        let mut req_count: u8 = 0;
        let ifcnt_before_req = self.ifcnt;
        while self.ifcnt == ifcnt_before_req {
            if req_count >= Self::MAX_SAME_REQ_COUNT {
                return Result::Err(());
            }
            let _ = self.send_write_req(write_req).await;

            self.update_ifcnt_val().await.ok();
            req_count += 1;
        }

        return Result::Ok(());
    }

    async fn send_write_req(
        &mut self,
        write_req: tmc2209::WriteRequest,
    ) -> Result<(), rtic_sync::channel::NoReceiver<crate::my_tmc2209::Request>> {
        let req = crate::my_tmc2209::Request::write(write_req);
        self.req_sender.send(req).await
    }

    async fn update_ifcnt_val(&mut self) -> Result<(), ()> {
        let mut req_count: u8 = 0;
        let mut resp = Result::Err(());
        while resp.is_err() {
            if req_count >= Self::MAX_SAME_REQ_COUNT {
                return Result::Err(());
            }

            self.send_ifcnt_req().await;
            resp = Self::read_ifcnt_resp(&mut self.rsp_receiver).await;
            req_count += 1;
        }
        let ifcnt = resp.ok().unwrap();

        if !self.first_call && self.ifcnt == ifcnt {
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
        self.req_sender.send(req).await.ok();
    }

    async fn read_ifcnt_resp(tmc2209_rsp_receiver: &mut Tmc2209ResponseConsumer) -> Result<u8, ()> {
        let response = tmc2209_rsp_receiver.recv().await.unwrap();
        if response == u32::MAX {
            return Result::Err(());
        }

        let ifcnt_resp = tmc2209::reg::IFCNT::from(response);
        return Ok(ifcnt_resp.0 as u8);
    }
}
