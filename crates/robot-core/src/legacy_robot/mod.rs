use rtic_sync::channel::{Receiver, Sender};

use crate::{
    display,
    my_stepper::{MyStepperCommands, MyStepperCommandsSender},
    ps3::{Ps3Command, Ps3DigitalCommand, Ps3Stick},
    DisplayMemoryPool, DisplayString,
};
use core::fmt::Write;
use heapless::pool::singleton::Box;

const DISPLAY_CHANNEL_CAPACITY: usize = crate::display::CHANNEL_CAPACITY;
const PS3_CHANNEL_CAPACITY: usize = crate::ps3::CHANNEL_CAPACITY;

pub type Ps3CommandsReceiver = Receiver<'static, Ps3Command, PS3_CHANNEL_CAPACITY>;

pub enum RobotCommand {
    Stay,
    StartMove,
    AddSteps(u32),
    ReduceSpeed,
    IncreaseSpeed,
    MoveToRight(u8),
    MoveToLeft(u8),
    SelectStepper(u8),
    AllMode,
    ChangeDirection(bool),
}

struct RobotState {
    separate_mode: bool,
    stepper_index: u8,
}

pub struct Robot {
    stepper_0: MyStepperCommandsSender,
    stepper_1: MyStepperCommandsSender,
    stepper_2: MyStepperCommandsSender,
    stepper_3: MyStepperCommandsSender,
    ps3_commands_receiver: Ps3CommandsReceiver,
    state: RobotState,
    display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    speed: u32,
}

impl From<Ps3Command> for Option<RobotCommand> {
    fn from(value: Ps3Command) -> Self {
        match value {
            Ps3Command::Digital(command) => match command {
                Ps3DigitalCommand::CROSS_DOWN => Some(RobotCommand::SelectStepper(3)),
                Ps3DigitalCommand::SQUARE_DOWN => Some(RobotCommand::SelectStepper(0)),
                Ps3DigitalCommand::TRIANGLE_DOWN => Some(RobotCommand::SelectStepper(1)),
                Ps3DigitalCommand::CIRCLE_DOWN => Some(RobotCommand::SelectStepper(2)),
                Ps3DigitalCommand::UP_DOWN => Some(RobotCommand::AddSteps(100)),
                Ps3DigitalCommand::RIGHT_DOWN => Some(RobotCommand::IncreaseSpeed),
                Ps3DigitalCommand::DOWN_DOWN => Some(RobotCommand::Stay),
                Ps3DigitalCommand::LEFT_DOWN => Some(RobotCommand::ReduceSpeed),
                Ps3DigitalCommand::START_DOWN => Some(RobotCommand::StartMove),
                Ps3DigitalCommand::SELECT_DOWN => Some(RobotCommand::AllMode),
                Ps3DigitalCommand::L1_DOWN => Some(RobotCommand::AddSteps(1)),
                Ps3DigitalCommand::L2_DOWN => Some(RobotCommand::ChangeDirection(false)),
                Ps3DigitalCommand::R1_DOWN => Some(RobotCommand::AddSteps(10)),
                Ps3DigitalCommand::R2_DOWN => Some(RobotCommand::ChangeDirection(true)),
                _ => None,
            },
            Ps3Command::Stick(command) => {
                if command.stick == Ps3Stick::Left {
                    let byte_1 = command.value.x_axis;
                    let byte_2 = command.value.y_axis;

                    if byte_1 == -1 && byte_2 == -1 {
                        return Some(RobotCommand::Stay);
                    }

                    let speed = byte_1 / 10;
                    if speed < 0 {
                        Some(RobotCommand::MoveToLeft(speed.abs() as u8))
                    } else {
                        Some(RobotCommand::MoveToRight(speed as u8))
                    }
                } else {
                    None
                }
            }
            Ps3Command::OnConnect => None,
        }
    }
}

impl Robot {
    pub fn new(
        stepper_0: MyStepperCommandsSender,
        stepper_1: MyStepperCommandsSender,
        stepper_2: MyStepperCommandsSender,
        stepper_3: MyStepperCommandsSender,
        ps3_commands_receiver: Ps3CommandsReceiver,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    ) -> Self {
        Robot {
            stepper_0,
            stepper_1,
            stepper_2,
            stepper_3,
            ps3_commands_receiver,
            state: RobotState {
                separate_mode: false,
                stepper_index: 0,
            },
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
    }

    #[rustfmt::skip]
    async fn receive_command(&mut self, event: RobotCommand) {
        match event {
            RobotCommand::Stay => {
                let command = MyStepperCommands::Stay;
                self.send_command(command).await;
            },

            RobotCommand::MoveToLeft(speed) => {
                let stepper_dir_command = MyStepperCommands::Direction(false);
                let stepper_speed_command = MyStepperCommands::Move(speed);
                self.send_command_2(stepper_dir_command, stepper_speed_command).await;
            },

            RobotCommand::MoveToRight(speed) => {
                let stepper_dir_command = MyStepperCommands::Direction(true);
                let stepper_speed_command = MyStepperCommands::Move(speed);
                self.send_command_2(stepper_dir_command, stepper_speed_command).await;
            },

            RobotCommand::AddSteps(steps_amount) => {
                let command = MyStepperCommands::AddSteps(steps_amount);
                self.send_command(command).await;
            },

            RobotCommand::ReduceSpeed => {
                self.speed -= 100;
                let command = MyStepperCommands::ReduceSpeed;
                self.send_command(command).await;
            },

            RobotCommand::IncreaseSpeed => {
                self.speed += 100;
                let command = MyStepperCommands::IncreaseSpeed;
                self.send_command(command).await;
            },

            RobotCommand::SelectStepper(index) => {
                self.state.separate_mode = true;
                self.state.stepper_index = index;
                defmt::debug!(
                    "robot: controlling stepper {}", index
                );
                let mut data_str = DisplayString::new();
                let stepper_index = self.state.stepper_index;
                write!(data_str, "Robot: s{stepper_index}").expect("not written");
                display::display_str(data_str, &mut self.display_sender)
                    .await
                    .unwrap();
            },

            RobotCommand::StartMove => {
                let command = MyStepperCommands::StartMove;
                self.send_command(command).await;
            }

            RobotCommand::AllMode => {
                self.state.separate_mode = false;
                defmt::debug!(
                    "robot: controlling all steppers",
                );
                let data_str = DisplayString::from("Robot: all mode");
                display::display_str(data_str, &mut self.display_sender)
                    .await
                    .unwrap();
            }

            RobotCommand::ChangeDirection(is_right) => {
                let stepper_dir_command = MyStepperCommands::Direction(is_right);
                self.send_command(stepper_dir_command).await;
            }
        }
    }

    async fn send_command(&mut self, command: MyStepperCommands) {
        if self.state.separate_mode {
            let stepper = self.get_stepper_sender();
            stepper.send(command).await.ok();
        } else {
            // self.stepper_0.send(command.clone()).await.ok();
            self.stepper_1.send(command.clone()).await.ok();
            self.stepper_2.send(command.clone()).await.ok();
            self.stepper_3.send(command.clone()).await.ok();
        }
    }

    async fn send_command_2(&mut self, command: MyStepperCommands, command_2: MyStepperCommands) {
        if self.state.separate_mode {
            let stepper = self.get_stepper_sender();
            stepper.send(command).await.ok();
            stepper.send(command_2).await.ok();
        } else {
            // self.stepper_0.send(command.clone()).await.ok();
            // self.stepper_0.send(command_2.clone()).await.ok();
            self.stepper_1.send(command.clone()).await.ok();
            self.stepper_1.send(command_2.clone()).await.ok();
            self.stepper_2.send(command.clone()).await.ok();
            self.stepper_2.send(command_2.clone()).await.ok();
            self.stepper_3.send(command.clone()).await.ok();
            self.stepper_3.send(command_2.clone()).await.ok();
        }
    }

    fn get_stepper_sender(&mut self) -> &mut MyStepperCommandsSender {
        let index = self.state.stepper_index;
        if index == 0 {
            return &mut self.stepper_1; //instead of a comment to not bother with return type
        } else if index == 1 {
            return &mut self.stepper_1;
        } else if index == 2 {
            return &mut self.stepper_2;
        } else if index == 3 {
            return &mut self.stepper_3;
        }

        return &mut self.stepper_1;
    }
}