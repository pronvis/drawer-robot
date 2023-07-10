#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use drawer_robot as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
    // dispatchers = [PVD, WWDG, RTC, SPI1]
)]
mod app {

    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::{
        adc::{self, Adc},
        device::ADC1,
        pac,
        prelude::*,
    };

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        adc1: Adc<ADC1>,
        pb0: stm32f1xx_hal::gpio::Pin<'B', 0, stm32f1xx_hal::gpio::Analog>,
    }

    const TIMER_CLOCK_FREQ: u32 = 2_000_000;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash: stm32f1xx_hal::flash::Parts = cx.device.FLASH.constrain();
        let mut rcc: stm32f1xx_hal::rcc::Rcc = cx.device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
        // `clocks`
        // with those values one second equals 'delay_cycles: u32 = 72_000_000'
        let clocks = rcc
            .cfgr
            .adcclk(2.MHz())
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        let adc1 = adc::Adc::adc1(cx.device.ADC1, clocks);
        let mut gpiob = cx.device.GPIOB.split();
        // Configure pb0 as an analog input
        let pb0 = gpiob.pb0.into_analog(&mut gpiob.crl);

        task1::spawn().ok();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 72_000_000, systick_mono_token);

        (Shared {}, Local { adc1, pb0 })
    }

    #[task(priority = 1, local = [adc1, pb0])]
    async fn task1(mut cx: task1::Context) {
        loop {
            let data: u16 = cx.local.adc1.read(cx.local.pb0).unwrap();
            defmt::debug!("data read: {}", data);
            Systick::delay(100.millis()).await;
        }
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            cortex_m::asm::nop();
        }
    }
}
