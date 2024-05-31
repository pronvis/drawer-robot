#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
    // dispatchers = [PVD, WWDG, RTC, SPI1]
)]
mod app {
    use hx711_spi::Hx711;
    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::{
        device::SPI2,
        gpio::{Alternate, Pin},
        prelude::*,
        spi::{self, Spi, Spi2NoRemap},
    };

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        hx711_sensor: Hx711<Spi<SPI2, Spi2NoRemap, (Pin<'B', 13, Alternate>, Pin<'B', 14>, Pin<'B', 15, Alternate>), u8>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash: stm32f1xx_hal::flash::Parts = cx.device.FLASH.constrain();
        let rcc: stm32f1xx_hal::rcc::Rcc = cx.device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
        // `clocks`
        // with those values one second equals 'delay_cycles: u32 = 72_000_000'
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(MAIN_CLOCK_FREQ.Hz())
            .pclk1(36.MHz())
            .pclk2(MAIN_CLOCK_FREQ.Hz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        let mut gpiob = cx.device.GPIOB.split();

        let hx711_spi_pins = (
            gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
            gpiob.pb14.into_floating_input(&mut gpiob.crh),
            gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
        );

        let hx711_spi = spi::Spi::spi2(cx.device.SPI2, hx711_spi_pins, embedded_hal::spi::MODE_1, 1.MHz(), clocks);
        let mut hx711_sensor = Hx711::new(hx711_spi);
        hx711_sensor.reset().unwrap();
        hx711_sensor.set_mode(hx711_spi::Mode::ChAGain128).unwrap(); // x128 works up to +-20mV

        hx711_read_task::spawn().ok();

        (Shared {}, Local { hx711_sensor })
    }

    #[task(priority = 1, local = [hx711_sensor])]
    async fn hx711_read_task(cx: hx711_read_task::Context) {
        loop {
            match cx.local.hx711_sensor.read() {
                Ok(resp) => defmt::debug!("Read from HX711, value: {}", resp),
                Err(_) => defmt::error!("Fail to read from HX711"),
            }

            Systick::delay(200.millis()).await;
        }
    }
}
