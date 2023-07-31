use crate::*;
use fugit::RateExtU32;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{
    afio,
    i2c::{BlockingI2c, DutyCycle, Mode},
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

pub struct OledDisplay {
    disp: Ssd1306Display,
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
            Mode::Fast {
                frequency: 400_000.Hz(),
                duty_cycle: DutyCycle::Ratio2to1,
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

        OledDisplay { disp: display }
    }

    pub fn print(&mut self, text: Box<DisplayMemoryPool>) {
        let text_style_1 = MonoTextStyle::new(&FONT_6X12, BinaryColor::On);

        let text_to_show = text.as_str();
        for i in 1..6 {
            let _ = Text::new(text_to_show, Point::new(0, 12 * i), text_style_1)
                .draw(&mut self.disp)
                .unwrap();
        }

        drop(text);
        self.disp.flush().unwrap();
    }
}
