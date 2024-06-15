use crate::{
    display,
    robot::{RobotCommand, RobotCommandsSender},
    DisplayMemoryPool, DisplayString,
};
use heapless::pool::singleton::Box;
use rtic_sync::channel::{Receiver, Sender};

use super::Ps3Event;

const DISPLAY_CHANNEL_CAPACITY: usize = crate::display::CHANNEL_CAPACITY;
const PS3_CHANNEL_CAPACITY: usize = crate::ps3::CHANNEL_CAPACITY;

pub struct Ps3EventParser {
    ps3_events_receiver: Receiver<'static, Ps3Event, PS3_CHANNEL_CAPACITY>,
    robot_commands_sender: RobotCommandsSender,
    display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
}

impl Ps3EventParser {
    pub fn new(
        ps3_events_receiver: Receiver<'static, Ps3Event, PS3_CHANNEL_CAPACITY>,
        robot_commands_sender: RobotCommandsSender,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
    ) -> Self {
        Ps3EventParser {
            ps3_events_receiver,
            robot_commands_sender,
            display_sender,
        }
    }

    pub async fn work(&mut self) {
        match self.ps3_events_receiver.recv().await {
            Ok(event) => self.receive_event(event).await,
            Err(err) => {
                defmt::error!("fail to receive data from channel: {}", defmt::Debug2Format(&err));
                return;
            }
        };
    }

    async fn receive_event(&mut self, event: Ps3Event) {
        match event {
            Ps3Event::OnConnect => {
                let data_str = DisplayString::from("PS3 connected");
                display::display_str(data_str, &mut self.display_sender).await.unwrap();
            }

            Ps3Event::DigitalSignal(signal) => {
                let robot_command = digital_signal_to_robot_command(signal);
                if let Some(robot_command) = robot_command {
                    self.robot_commands_sender.send(robot_command).await.ok();
                }
            }

            Ps3Event::AnalogSignal(data) => {
                let is_left_stick = data[0] == 0x01;
                let x_axis = data[1] as i8;
                let y_axis = data[2] as i8;
                let right_left_str = match is_left_stick {
                    true => "left",
                    false => "right",
                };
                defmt::debug!("sticker event:\t{}\tx: {}\ty: {}", right_left_str, x_axis, y_axis);

                let robot_command = analog_signal_to_robot_command(is_left_stick, x_axis, y_axis);
                if let Some(robot_command) = robot_command {
                    self.robot_commands_sender.send(robot_command).await.ok();
                }
            }
        }
    }
}

fn digital_signal_to_robot_command(signal: u8) -> Option<RobotCommand> {
    let robot_command = match signal {
        // up button down
        0x55 => Some(RobotCommand::AddSteps(100)),
        // down button down
        0x44 => Some(RobotCommand::Stay),
        // right button down
        0x52 => Some(RobotCommand::IncreaseSpeed),
        // left button down
        0x4c => Some(RobotCommand::ReduceSpeed),
        // square button down
        0x53 => Some(RobotCommand::SelectStepper(0)),
        // triangle button down
        0x54 => Some(RobotCommand::SelectStepper(1)),
        // circle button down
        0x43 => Some(RobotCommand::SelectStepper(2)),
        // cross button down
        0x58 => Some(RobotCommand::SelectStepper(3)),
        // Start button down
        0x57 => Some(RobotCommand::StartMove),
        // Select button down
        0x59 => Some(RobotCommand::AllMode),
        // L1 button down
        0x60 => Some(RobotCommand::AddSteps(1)),
        // L2 button down
        0x61 => Some(RobotCommand::ChangeDirection(false)),
        // R1 button down
        0x62 => Some(RobotCommand::AddSteps(10)),
        // R2 button down
        0x65 => Some(RobotCommand::ChangeDirection(true)),
        _ => None,
    };
    robot_command
}

fn analog_signal_to_robot_command(is_left_stick: bool, byte_1: i8, byte_2: i8) -> Option<RobotCommand> {
    let mut robot_command = None;
    if is_left_stick {
        if byte_1 == -1 && byte_2 == -1 {
            return Some(RobotCommand::Stay);
        }

        let speed = byte_1 / 10;
        if speed < 0 {
            robot_command.replace(RobotCommand::MoveToLeft(speed.abs() as u8));
        } else {
            robot_command.replace(RobotCommand::MoveToRight(speed as u8));
        }
    }

    robot_command
}
