use fugit::ExtU32;
use rtic_monotonics::systick::Systick;
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
    const MAX_SAME_REQ_COUNT: u8 = 5;

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

    pub async fn setup(&mut self, tmc2209_rsp_receiver: &mut Receiver<'static, u32, CHANNEL_CAPACITY>) -> Result<(), ()> {
        self.update_ifcnt_val(tmc2209_rsp_receiver).await?;

        let mut gconf = tmc2209::reg::GCONF(0);
        gconf.set_i_scale_analog(true);
        gconf.set_internal_rsense(true);
        gconf.set_en_spread_cycle(true);
        gconf.set_pdn_disable(true);
        gconf.set_mstep_reg_select(true);
        gconf.set_multistep_filt(true);
        gconf.set_test_mode(false);
        let write_req = tmc2209::write_request(0, gconf);
        self.send_req_and_check(write_req, tmc2209_rsp_receiver).await?;

        let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(100_000));
        self.send_req_and_check(write_req, tmc2209_rsp_receiver).await?;

        return Result::Ok(());
    }

    async fn send_req_and_check(
        &mut self,
        write_req: tmc2209::WriteRequest,
        tmc2209_rsp_receiver: &mut Receiver<'static, u32, CHANNEL_CAPACITY>,
    ) -> Result<(), ()> {
        let mut req_count: u8 = 0;
        let ifcnt_before_req = self.ifcnt;
        while self.ifcnt == ifcnt_before_req {
            if req_count >= Self::MAX_SAME_REQ_COUNT {
                return Result::Err(());
            }
            let _ = self.send_write_req(write_req).await;

            self.update_ifcnt_val(tmc2209_rsp_receiver).await.ok();
            req_count += 1;
        }

        return Result::Ok(());
    }

    async fn send_write_req(
        &mut self,
        write_req: tmc2209::WriteRequest,
    ) -> Result<(), rtic_sync::channel::NoReceiver<crate::my_tmc2209::Request>> {
        let req = crate::my_tmc2209::Request::write(write_req);
        self.sender.send(req).await
    }

    async fn update_ifcnt_val(&mut self, tmc2209_rsp_receiver: &mut Receiver<'static, u32, CHANNEL_CAPACITY>) -> Result<(), ()> {
        let mut req_count: u8 = 0;
        let mut resp = Result::Err(());
        while resp.is_err() {
            if req_count >= Self::MAX_SAME_REQ_COUNT {
                return Result::Err(());
            }

            self.send_ifcnt_req().await;
            resp = Self::read_ifcnt_resp(tmc2209_rsp_receiver).await;
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
