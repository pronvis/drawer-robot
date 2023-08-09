use crate::{
    display, robot::RobotCommand, DisplayMemoryPool, DisplayString, CHANNEL_CAPACITY,
    PS3_CHANNEL_CAPACITY,
};
use heapless::pool::singleton::Box;
use rtic_sync::channel::{Receiver, Sender};

use super::Ps3Event;
use core::fmt::Write;

// CrossDown,
// CrossUp,
// SquareUp,
// SquareDown,
// TriangleUp,
// TriangleDown,
// CurcleUp,
// CurcleDown,
// UpUp,
// UpDown,
// DownUp,
// DownDown,
// LeftUp,
// LeftDown,
// RightUp,
// RightDown,
// LeftStick { x: i8, y: i8 },
// RightStick { x: i8, y: i8 },

pub struct Ps3EventParser {
    ps3_events_receiver: Receiver<'static, Ps3Event, PS3_CHANNEL_CAPACITY>,
    robot_commands_sender: Sender<'static, RobotCommand, CHANNEL_CAPACITY>,
    display_sender: Sender<'static, Box<DisplayMemoryPool>, CHANNEL_CAPACITY>,
}

impl Ps3EventParser {
    pub fn new(
        ps3_events_receiver: Receiver<'static, Ps3Event, PS3_CHANNEL_CAPACITY>,
        robot_commands_sender: Sender<'static, RobotCommand, CHANNEL_CAPACITY>,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, CHANNEL_CAPACITY>,
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
                defmt::error!(
                    "fail to receive data from channel: {}",
                    defmt::Debug2Format(&err)
                );
                return;
            }
        };
    }

    async fn receive_event(&mut self, event: Ps3Event) {
        match event {
            Ps3Event::OnConnect => {
                let data_str = DisplayString::from("PS3 connected");
                display::display_str(data_str, &mut self.display_sender)
                    .await
                    .unwrap();
            }

            Ps3Event::DigitalSignal(signal) => {
                let mut data_str = DisplayString::new();
                write!(data_str, "receive signal: {signal}").expect("not written");
                display::display_str(data_str, &mut self.display_sender)
                    .await
                    .unwrap();
            }

            Ps3Event::AnalogSignal(data) => {
                let is_left_stick = data[0] == 0x01;
                let x_axis = data[1] as i8;
                let y_axis = data[2] as i8;
                let right_left_str = match is_left_stick {
                    true => "left",
                    false => "right",
                };
                defmt::debug!(
                    "sticker event:\t{}\tx: {}\ty: {}",
                    right_left_str,
                    x_axis,
                    y_axis
                );

                let robot_command = analog_signal_to_robot_command(data[0], x_axis, y_axis);
                self.robot_commands_sender.send(robot_command).await.ok();

                // let mut data_str = DisplayString::new();
                // write!(data_str, "{right_left_str}: {x_axis} : {y_axis}").expect("not written");
                // display::display_str(data_str, &mut self.display_sender)
                //     .await
                //     .unwrap();
            }
        }
    }
}

fn analog_signal_to_robot_command(byte_0: u8, byte_1: i8, byte_2: i8) -> RobotCommand {
    let mut robot_command = RobotCommand::Stay;
    if byte_0 == 0x01 {
        if byte_1 == -1 && byte_2 == -1 {
            return RobotCommand::Stay;
        }

        let speed = byte_1 / 10;
        if speed < 0 {
            robot_command = RobotCommand::MoveToLeft(speed.abs() as u8);
        } else {
            robot_command = RobotCommand::MoveToRight(speed as u8);
        }
    }

    robot_command
}
