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
        // let mut buf = [0u8; 128];

        // defmt::debug!("size of memory_pool that I've receive: {}", text.len());
        let text_style_1 = MonoTextStyle::new(&FONT_6X12, BinaryColor::On);

        // let text_1 = write_to::show(&mut buf, format_args!("{:?}", &text)).unwrap();
        let text_to_show = core::str::from_utf8(text.as_slice()).unwrap();
        // TODO: USE IT INSTEAD:
        // use core::str::from_utf8_unchecked;
        // Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })

        for i in 1..6 {
            let _ = Text::new(text_to_show, Point::new(0, 12 * i), text_style_1)
                .draw(&mut self.disp)
                .unwrap();
        }

        drop(text);
        self.disp.flush().unwrap();
    }
}

// from https://stackoverflow.com/questions/50200268/how-can-i-use-the-format-macro-in-a-no-std-environment
pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(buffer: &'a mut [u8], args: fmt::Arguments) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}
