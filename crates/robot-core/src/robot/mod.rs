use crate::{
    ps3::{Ps3Command, Ps3DigitalCommand},
    DisplayMemoryPool, DisplayString,
};
use heapless::pool::singleton::Box;
use rtic_sync::channel::{Receiver, Sender};

const DISPLAY_CHANNEL_CAPACITY: usize = crate::display::CHANNEL_CAPACITY;
const PS3_CHANNEL_CAPACITY: usize = crate::ps3::CHANNEL_CAPACITY;
const COMMUNICATOR_CHANNEL_CAPACITY: usize = crate::my_tmc2209::communicator::CHANNEL_CAPACITY;

pub type Tmc2209Request = crate::my_tmc2209::Request;
pub type Ps3CommandsReceiver = Receiver<'static, Ps3Command, PS3_CHANNEL_CAPACITY>;
pub type Tmc2209CommandsSender = Sender<'static, Tmc2209Request, COMMUNICATOR_CHANNEL_CAPACITY>;

pub enum RobotCommand {
    Free,
    Balance,
    SetBalance,
    SetSpeed(u32),
}

impl From<Ps3Command> for Option<RobotCommand> {
    fn from(value: Ps3Command) -> Self {
        match value {
            Ps3Command::Digital(command) => match command {
                Ps3DigitalCommand::CROSS_DOWN => Some(RobotCommand::Balance),
                Ps3DigitalCommand::TRIANGLE_DOWN => Some(RobotCommand::SetSpeed(50_000)),
                Ps3DigitalCommand::TRIANGLE_UP => Some(RobotCommand::SetSpeed(0)),
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
    display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    //TODO: experiment
    speed: u32,
}

impl Robot {
    pub fn new(
        stepper_0: Tmc2209CommandsSender,
        ps3_commands_receiver: Ps3CommandsReceiver,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    ) -> Self {
        Robot {
            stepper_0,
            ps3_commands_receiver,
            state: RobotState::Stay(false),
            display_sender,
            speed: 0,
        }
    }

    pub async fn work(&mut self) {
        match self.ps3_commands_receiver.recv().await {
            Ok(event) => {
                let command: Option<RobotCommand> = event.into();
                if let Some(c) = command {
                    self.receive_command(c).await;
                }
            }
            Err(err) => {
                defmt::error!("fail to receive command from channel: {}", defmt::Debug2Format(&err));
                return;
            }
        };

        self.control_steppers().await;
    }

    async fn control_steppers(&mut self) {
        //TODO: implement some sophisticated logic
        let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(self.speed));
        let req = Tmc2209Request::write(write_req);
        self.stepper_0.send(req).await.ok();
    }

    #[rustfmt::skip]
    async fn receive_command(&mut self, event: RobotCommand) {
        match event {
            RobotCommand::Free => {
                self.state = RobotState::Stay(false);
            },
            RobotCommand::Balance => {
                self.state = RobotState::Stay(true);
            },
            RobotCommand::SetBalance => (),
            RobotCommand::SetSpeed(speed) => {
                self.speed = speed;
            },
        }
    }
}
