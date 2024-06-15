use rtic_sync::channel::{Receiver, Sender};

use crate::{
    display,
    my_stepper::{MyStepperCommands, MyStepperCommandsSender},
    DisplayMemoryPool, DisplayString,
};
use core::fmt::Write;
use heapless::pool::singleton::Box;

const COMMUNICATOR_CHANNEL_CAPACITY: usize = crate::my_tmc2209::communicator::CHANNEL_CAPACITY;
const DISPLAY_CHANNEL_CAPACITY: usize = crate::display::CHANNEL_CAPACITY;

pub type RobotCommandsSender = Sender<'static, RobotCommand, COMMUNICATOR_CHANNEL_CAPACITY>;
pub type RobotCommandsReceiver = Receiver<'static, RobotCommand, COMMUNICATOR_CHANNEL_CAPACITY>;

//TODO: improve
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
    stepper_0: Sender<'static, crate::my_tmc2209::Request, COMMUNICATOR_CHANNEL_CAPACITY>,
    stepper_1: MyStepperCommandsSender,
    stepper_2: MyStepperCommandsSender,
    stepper_3: MyStepperCommandsSender,
    commands_receiver: RobotCommandsReceiver,
    state: RobotState,
    display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    speed: u32,
}

impl Robot {
    pub fn new(
        stepper_0: Sender<'static, crate::my_tmc2209::Request, COMMUNICATOR_CHANNEL_CAPACITY>,
        stepper_1: MyStepperCommandsSender,
        stepper_2: MyStepperCommandsSender,
        stepper_3: MyStepperCommandsSender,
        commands_receiver: RobotCommandsReceiver,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    ) -> Self {
        Robot {
            stepper_0,
            stepper_1,
            stepper_2,
            stepper_3,
            commands_receiver,
            state: RobotState {
                separate_mode: false,
                stepper_index: 0,
            },
            display_sender,
            speed: 0,
        }
    }

    pub async fn work(&mut self) {
        match self.commands_receiver.recv().await {
            Ok(event) => self.receive_command(event).await,
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
