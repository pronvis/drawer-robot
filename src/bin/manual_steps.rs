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
    use stepper::{
        compat, fugit::NanosDurationU32 as Nanoseconds, motion_control,
        motion_control::SoftwareMotionControl, ramp_maker, Direction, Stepper,
    };
    use stm32f1xx_hal::{pac, prelude::*, rcc, timer::Timer};

    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        step_pin: stm32f1xx_hal::gpio::Pin<'C', 14, stm32f1xx_hal::gpio::Output>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
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
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        // Acquire the GPIOC peripheral
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();

        ////////////////////////////
        ////////////////////////////
        // Motor Driver Configuration

        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        // let step = gpioc.pc6.into_push_pull_output(&mut gpioc.crl);
        //let dir = gpiob.pb15.into_push_pull_output(&mut gpiob.crl);

        let mut en = gpioc.pc15.into_push_pull_output(&mut gpioc.crh);
        en.set_low();
        let step = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
        let mut dir = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        dir.set_low();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, STEPPER_CLOCK_FREQ, systick_mono_token); // default STM32F303 clock-rate is 36MHz

        task::spawn().ok();

        (Shared {}, Local { step_pin: step })
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            rtic::export::nop();
        }
    }

    #[task(priority = 3, local = [ step_pin ])]
    async fn task(mut cx: task::Context) {
        defmt::debug!("Move motor!");

        loop {
            cx.local.step_pin.set_high();
            // Systick::delay(4000.nanos()).await;
            cx.local.step_pin.set_low();
            // Systick::delay(4000.nanos()).await;
            Systick::delay(1_000_000.nanos()).await;
        }
    }
}
