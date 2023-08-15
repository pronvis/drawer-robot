use rtic_sync::channel::{Receiver, Sender};

use crate::{
    my_stepper::{MyStepperCommands, MyStepperCommandsSender},
    CHANNEL_CAPACITY,
};

pub type RobotCommandsSender = Sender<'static, RobotCommand, CHANNEL_CAPACITY>;
pub type RobotCommandsReceiver = Receiver<'static, RobotCommand, CHANNEL_CAPACITY>;

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
    stepper_1: MyStepperCommandsSender,
    stepper_2: MyStepperCommandsSender,
    stepper_3: MyStepperCommandsSender,
    stepper_4: MyStepperCommandsSender,
    commands_receiver: RobotCommandsReceiver,
    state: RobotState,
}

impl Robot {
    pub fn new(
        stepper_1: MyStepperCommandsSender,
        stepper_2: MyStepperCommandsSender,
        stepper_3: MyStepperCommandsSender,
        stepper_4: MyStepperCommandsSender,
        commands_receiver: RobotCommandsReceiver,
    ) -> Self {
        Robot {
            stepper_1,
            stepper_2,
            stepper_3,
            stepper_4,
            commands_receiver,
            state: RobotState {
                separate_mode: false,
                stepper_index: 0,
            },
        }
    }

    pub async fn work(&mut self) {
        match self.commands_receiver.recv().await {
            Ok(event) => self.receive_command(event).await,
            Err(err) => {
                defmt::error!(
                    "fail to receive command from channel: {}",
                    defmt::Debug2Format(&err)
                );
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
                let command = MyStepperCommands::ReduceSpeed;
                self.send_command(command).await;
            },

            RobotCommand::IncreaseSpeed => {
                let command = MyStepperCommands::IncreaseSpeed;
                self.send_command(command).await;
            },

            RobotCommand::SelectStepper(index) => {
                self.state.separate_mode = true;
                self.state.stepper_index = index;
                defmt::debug!(
                    "robot: controlling stepper {}", index
                );
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
            self.stepper_1.send(command.clone()).await.ok();
            self.stepper_2.send(command.clone()).await.ok();
            self.stepper_3.send(command.clone()).await.ok();
            self.stepper_4.send(command.clone()).await.ok();
        }
    }

    async fn send_command_2(&mut self, command: MyStepperCommands, command_2: MyStepperCommands) {
        if self.state.separate_mode {
            let stepper = self.get_stepper_sender();
            stepper.send(command).await.ok();
            stepper.send(command_2).await.ok();
        } else {
            self.stepper_1.send(command.clone()).await.ok();
            self.stepper_1.send(command_2.clone()).await.ok();
            self.stepper_2.send(command.clone()).await.ok();
            self.stepper_2.send(command_2.clone()).await.ok();
            self.stepper_3.send(command.clone()).await.ok();
            self.stepper_3.send(command_2.clone()).await.ok();
            self.stepper_4.send(command.clone()).await.ok();
            self.stepper_4.send(command_2.clone()).await.ok();
        }
    }

    fn get_stepper_sender(&mut self) -> &mut MyStepperCommandsSender {
        let index = self.state.stepper_index;
        if index == 0 {
            return &mut self.stepper_1;
        } else if index == 1 {
            return &mut self.stepper_2;
        } else if index == 2 {
            return &mut self.stepper_3;
        } else if index == 3 {
            return &mut self.stepper_4;
        }

        return &mut self.stepper_1;
    }
}
