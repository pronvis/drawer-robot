use rtic_sync::channel::{Receiver, Sender};

use crate::{my_stepper::MyStepperCommands, CHANNEL_CAPACITY};

//TODO: improve
pub enum RobotCommand {
    Stay,
    MoveToRight(u8),
    MoveToLeft(u8),
}

pub struct Robot {
    stepper_1: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    stepper_2: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    stepper_3: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    stepper_4: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    commands_receiver: Receiver<'static, RobotCommand, CHANNEL_CAPACITY>,
}

impl Robot {
    pub fn new(
        stepper_1: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
        stepper_2: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
        stepper_3: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
        stepper_4: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
        commands_receiver: Receiver<'static, RobotCommand, CHANNEL_CAPACITY>,
    ) -> Self {
        Robot {
            stepper_1,
            stepper_2,
            stepper_3,
            stepper_4,
            commands_receiver,
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

    async fn receive_command(&mut self, event: RobotCommand) {
        match event {
            RobotCommand::Stay => {
                let stepper_1_command = MyStepperCommands::Stay;
                self.stepper_1.send(stepper_1_command.clone()).await.ok();
                self.stepper_2.send(stepper_1_command.clone()).await.ok();
                self.stepper_3.send(stepper_1_command.clone()).await.ok();
                self.stepper_4.send(stepper_1_command.clone()).await.ok();
            }
            #[rustfmt::skip]
            RobotCommand::MoveToLeft(speed) => {
                let stepper_1_dir_command = MyStepperCommands::Direction(false);
                let stepper_1_speed_command = MyStepperCommands::Move(speed);
                self.stepper_1.send(stepper_1_dir_command.clone()).await.ok();
                self.stepper_1.send(stepper_1_speed_command.clone()).await.ok();
                self.stepper_2.send(stepper_1_dir_command.clone()).await.ok();
                self.stepper_2.send(stepper_1_speed_command.clone()).await.ok();
                self.stepper_3.send(stepper_1_dir_command.clone()).await.ok();
                self.stepper_3.send(stepper_1_speed_command.clone()).await.ok();
                self.stepper_4.send(stepper_1_dir_command.clone()).await.ok();
                self.stepper_4.send(stepper_1_speed_command.clone()).await.ok();
            }
            #[rustfmt::skip]
            RobotCommand::MoveToRight(speed) => {
                let stepper_1_dir_command = MyStepperCommands::Direction(true);
                let stepper_1_speed_command = MyStepperCommands::Move(speed);
                self.stepper_1.send(stepper_1_dir_command.clone()).await.ok();
                self.stepper_1.send(stepper_1_speed_command.clone()).await.ok();
                self.stepper_2.send(stepper_1_dir_command.clone()).await.ok();
                self.stepper_2.send(stepper_1_speed_command.clone()).await.ok();
                self.stepper_3.send(stepper_1_dir_command.clone()).await.ok();
                self.stepper_3.send(stepper_1_speed_command.clone()).await.ok();
                self.stepper_4.send(stepper_1_dir_command.clone()).await.ok();
                self.stepper_4.send(stepper_1_speed_command.clone()).await.ok();
            }
        }
    }
}
