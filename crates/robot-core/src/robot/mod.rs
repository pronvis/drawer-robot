mod arm;

use crate::{
    ps3::{Ps3Command, Ps3DigitalCommand},
    DisplayMemoryPool, DisplayString,
};
use arm::RobotArms;
use heapless::pool::singleton::Box;
use rtic_sync::channel::{ReceiveError, Receiver, Sender};

pub const HC05_CALIBRATE_ADS1256_CODE: u8 = 221;
pub const TENSION_DATA_CHANNEL_CAPACITY: usize = 32;

const DISPLAY_CHANNEL_CAPACITY: usize = crate::display::CHANNEL_CAPACITY;
const PS3_CHANNEL_CAPACITY: usize = crate::ps3::CHANNEL_CAPACITY;
const COMMUNICATOR_CHANNEL_CAPACITY: usize = crate::my_tmc2209::communicator::CHANNEL_CAPACITY;

pub type Tmc2209Request = crate::my_tmc2209::Request;
pub type Ps3CommandsReceiver = Receiver<'static, Ps3Command, PS3_CHANNEL_CAPACITY>;
pub type TensionDataReceiver = Receiver<'static, TensionData, TENSION_DATA_CHANNEL_CAPACITY>;
pub type Tmc2209CommandsSender = Sender<'static, Tmc2209Request, COMMUNICATOR_CHANNEL_CAPACITY>;

#[derive(Debug)]
pub struct TensionData {
    pub t0: i32,
    pub t1: i32,
    pub t2: i32,
    pub t3: i32,
}

pub enum RobotCommand {
    CalibrateAds1256,
    SetFreeTension,
    SetDesiredTension,
    ManualControlNextArm,
}

impl From<Ps3Command> for Option<RobotCommand> {
    fn from(value: Ps3Command) -> Self {
        match value {
            Ps3Command::Digital(command) => match command {
                Ps3DigitalCommand::TRIANGLE_DOWN => Some(RobotCommand::CalibrateAds1256),
                Ps3DigitalCommand::CIRCLE_DOWN => Some(RobotCommand::SetDesiredTension),
                Ps3DigitalCommand::SQUARE_DOWN => Some(RobotCommand::SetFreeTension),
                Ps3DigitalCommand::RIGHT_DOWN => Some(RobotCommand::ManualControlNextArm),
                _ => None,
            },
            Ps3Command::Stick(_) => None,
            Ps3Command::OnConnect => None,
        }
    }
}

enum RobotState {
    Stay(bool),
}

pub struct Robot {
    stepper_0: Tmc2209CommandsSender,
    state: RobotState,
    ps3_commands_receiver: Ps3CommandsReceiver,
    tension_data_receiver: TensionDataReceiver,
    hc05_tx: stm32f1xx_hal::serial::Tx1,
    display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    arms: RobotArms,
}

impl Robot {
    pub fn new(
        stepper_0: Tmc2209CommandsSender,
        ps3_commands_receiver: Ps3CommandsReceiver,
        tension_data_receiver: TensionDataReceiver,
        hc05_tx: stm32f1xx_hal::serial::Tx1,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    ) -> Self {
        Robot {
            stepper_0,
            ps3_commands_receiver,
            tension_data_receiver,
            state: RobotState::Stay(false),
            hc05_tx,
            display_sender,
            arms: Default::default(),
        }
    }

    pub async fn work(&mut self) {
        self.ps3_handler();
        self.tension_handler();

        //test code
        let arms_speed = self.arms.get_speed();
        let arm0_speed = arms_speed.s0;
        let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(arm0_speed));
        let req = crate::my_tmc2209::Request::write(write_req);
        self.stepper_0.send(req).await.ok();
    }

    fn ps3_handler(&mut self) {
        match self.ps3_commands_receiver.try_recv() {
            Ok(event) => {
                let command: Option<RobotCommand> = event.into();
                if let Some(c) = command {
                    self.receive_command(c);
                }
            }
            Err(err) => {
                if err == ReceiveError::NoSender {
                    defmt::error!("fail to receive command from channel: ReceiveError::NoSender");
                    return;
                }
            }
        }
    }

    fn tension_handler(&mut self) {
        let mut tension_data: Option<TensionData> = None;
        //read all messages from channel
        while let Ok(data) = self.tension_data_receiver.try_recv() {
            tension_data = Some(data);
        }

        if let Some(tension_data) = tension_data {
            self.arms.set_tension(tension_data);
        }
    }

    #[rustfmt::skip]
    fn receive_command(&mut self, event: RobotCommand) {
        match event {
            RobotCommand::SetFreeTension => {
                self.arms.remove_desired_tension();
            },
            RobotCommand::SetDesiredTension => {
                self.arms.set_desired_tension();
            },
            RobotCommand::ManualControlNextArm => {
                //TODO:
            },
            RobotCommand::CalibrateAds1256 => {
                if let Err(err) = self.hc05_tx.write(HC05_CALIBRATE_ADS1256_CODE) {
                    defmt::error!("Fail to send 'Calibrate' message to hc05: {:?}", defmt::Debug2Format(&err))
                } else {
                    defmt::debug!("Successfully send message via hc05");
                }
            },
        }
    }
}
