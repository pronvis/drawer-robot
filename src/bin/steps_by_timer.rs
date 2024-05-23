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

    use drawer_robot::*;
    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::{
        gpio::PinState,
        pac,
        prelude::*,
        rcc,
        rcc::*,
        timer::Timer,
        timer::{Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 72_000_000; // step = 1000 nanos
    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;

    #[shared]
    struct Shared {
        nanos_between_steps: u32,
    }

    #[local]
    struct Local {
        step_pin: StepPin,
        timer_1: Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
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
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();
        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let step_pin = gpioc.pc6.into_push_pull_output_with_state(&mut gpioc.crl, PinState::Low);

        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>::new(cx.device.TIM1, &clocks);
        let mut timer_1: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ> = timer.counter();
        timer_1.start(2000.nanos()).unwrap();
        timer_1.listen(Event::Update);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, STEPPER_CLOCK_FREQ, systick_mono_token);

        (
            Shared {
                nanos_between_steps: 710_000,
            },
            Local { step_pin, timer_1 },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = TIM1_UP, priority = 3, local = [  timer_1, step_pin, is_step: bool = true], shared = [&nanos_between_steps])]
    fn delay_task_1(cx: delay_task_1::Context) {
        if *cx.local.is_step {
            cx.local.step_pin.set_low();
            *cx.local.is_step = false;
            cx.local.timer_1.start(cx.shared.nanos_between_steps.nanos()).unwrap();
        } else {
            cx.local.step_pin.set_high();
            *cx.local.is_step = true;
            cx.local.timer_1.start(1900.nanos()).unwrap();
        }

        cx.local.timer_1.clear_interrupt(Event::Update);
    }
}
