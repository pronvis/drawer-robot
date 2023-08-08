use crate::{display, DisplayMemoryPool, DisplayString, CHANNEL_CAPACITY, PS3_CHANNEL_CAPACITY};
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
    display_sender: Sender<'static, Box<DisplayMemoryPool>, CHANNEL_CAPACITY>,
}

impl Ps3EventParser {
    pub fn new(
        ps3_events_receiver: Receiver<'static, Ps3Event, PS3_CHANNEL_CAPACITY>,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, CHANNEL_CAPACITY>,
    ) -> Self {
        Ps3EventParser {
            ps3_events_receiver,
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

                let mut data_str = DisplayString::new();
                write!(data_str, "{right_left_str}: {x_axis} : {y_axis}").expect("not written");
                display::display_str(data_str, &mut self.display_sender)
                    .await
                    .unwrap();
            }
        }
    }
}
