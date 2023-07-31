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
use embedded_graphics::{
    mono_font::{ascii::FONT_6X12, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use heapless::pool::singleton::Box;
use heapless::Deque;

pub struct OledDisplay {
    disp: Ssd1306Display,
    string_queue: Deque<heapless::String<21>, 5>,
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
            .into_buffered_graphics_mode();
        display.init().unwrap();

        OledDisplay {
            disp: display,
            string_queue: Deque::new(),
        }
    }

    pub fn print(&mut self, text: Box<DisplayMemoryPool>) {
        if self.string_queue.is_full() {
            let _ = self.string_queue.pop_front();
        }
        self.string_queue.push_back(text.clone()).unwrap();

        self.disp.clear(BinaryColor::Off).unwrap();
        let text_style = MonoTextStyle::new(&FONT_6X12, BinaryColor::On);

        let queue_len = self.string_queue.len();
        for (text, i) in self.string_queue.iter().zip(0..queue_len) {
            let _ = Text::new(
                text,
                Point::new(0, (12 * (queue_len - i)) as i32),
                text_style,
            )
            .draw(&mut self.disp)
            .unwrap();
        }

        drop(text);
        self.disp.flush().unwrap();
    }
}

pub async fn display_str(
    text: heapless::String<21>,
    display_sender: &mut Sender<'static, Box<DisplayMemoryPool>, CHANNEL_CAPACITY>,
) {
    let memory_block = DisplayMemoryPool::alloc().unwrap().init(text);
    // TODO: USE IT
    display_sender.send(memory_block).await;
}
