use crate::{CHANNEL_CAPACITY, *};
use fugit::RateExtU32;
use rtic_sync::channel::Sender;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{
    afio,
    i2c::{BlockingI2c, Mode},
    rcc::Clocks,
};

use crate::DisplayMemoryPool;
use core::future::Future;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X12, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use heapless::pool::singleton::Box;
use heapless::Deque;

const FONT: embedded_graphics::mono_font::MonoFont = FONT_6X12;

pub struct OledDisplay {
    disp: Ssd1306Display,
    string_queue: Deque<DisplayString, 5>,
}

impl OledDisplay {
    pub fn new(
        scl: Scl1Pin,
        sda: Sda1Pin,
        parts: &mut afio::Parts,
        clocks: Clocks,
        i2c1: I2C1,
    ) -> Self {
        let i2c = BlockingI2c::i2c1(
            i2c1,
            (scl, sda),
            &mut parts.mapr,
            Mode::Standard {
                frequency: 400_000.Hz(),
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_terminal_mode();
        display.init().unwrap();

        OledDisplay {
            disp: display,
            string_queue: Deque::new(),
        }
    }

    //wrap into the Future
    pub fn print(&mut self, boxed_text: Box<DisplayMemoryPool>) {
        let text = boxed_text.clone();
        drop(boxed_text);

        if self.string_queue.is_full() {
            let _ = self.string_queue.pop_front();
        }
        self.string_queue.push_back(text).unwrap();

        self.disp.clear().unwrap();
        let text_style = MonoTextStyle::new(&FONT, BinaryColor::On);

        let queue_len = self.string_queue.len();
        for (text, i) in self.string_queue.iter().zip(0..queue_len) {
            for ch in text.chars() {
                self.disp.print_char(ch).unwrap();
            }
            self.disp.print_char('\n').unwrap();
        }
    }
}

/// to create String:
/// ```
/// let index: u32 = 0;
/// let mut data_str = DisplayString::new();
/// write!(data_str, "Hello world: {index}").expect("not written");
/// display::display_str(data_str, &mut cx.local.display_sender)
///     .await
///     .unwrap();
/// ```
pub async fn display_str(
    text: DisplayString,
    display_sender: &mut Sender<'static, Box<DisplayMemoryPool>, CHANNEL_CAPACITY>,
) -> Result<(), rtic_sync::channel::NoReceiver<Box<DisplayMemoryPool>>> {
    let memory_block = DisplayMemoryPool::alloc().unwrap().init(text);
    display_sender.send(memory_block).await
}
