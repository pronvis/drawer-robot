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

    use robot_core::*;
    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::prelude::*;

    // Shared resources go here
    #[shared]
    struct Shared {
        led: OutLed,
    }

    // Local resources go here
    #[local]
    struct Local {}

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
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        // Acquire the GPIOC peripheral
        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let led = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 72_000_000, systick_mono_token);

        task1::spawn().ok();
        task2::spawn().ok();

        (Shared { led }, Local {})
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(priority = 1, shared = [led])]
    async fn task1(mut cx: task1::Context) {
        loop {
            defmt::debug!("Hello from blink task-1!");
            cx.shared.led.lock(|x| {
                x.toggle();
            });
            Systick::delay(1000.millis()).await;
        }
    }

    #[task(priority = 2, shared = [led])]
    async fn task2(mut cx: task2::Context) {
        loop {
            defmt::debug!("Hello from blink task-2!");
            cx.shared.led.lock(|x| {
                x.toggle();
            });
            Systick::delay(300.millis()).await;
        }
    }
}
