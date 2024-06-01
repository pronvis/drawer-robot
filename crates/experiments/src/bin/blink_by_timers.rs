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

    use defmt_brtt as _; // global logger
    use robot_core::*;
    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::{
        gpio::PinState,
        pac,
        prelude::*,
        timer::{Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 72_000;
    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        internal_led: InternalLed,
        out_led: OutLed,
        timer_1: Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>,
        timer_2: Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>,
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
        let internal_led = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low);

        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let out_led = gpiob.pb12.into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low);

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>::new(cx.device.TIM1, &clocks);
        let mut timer_1: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ> = timer.counter();
        timer_1.start(1.millis()).unwrap();
        timer_1.listen(Event::Update);

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>::new(cx.device.TIM2, &clocks);
        let mut timer_2: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ> = timer.counter();
        timer_2.start(5.millis()).unwrap();
        timer_2.listen(Event::Update);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, STEPPER_CLOCK_FREQ, systick_mono_token);

        (
            Shared {},
            Local {
                internal_led,
                out_led,
                timer_1,
                timer_2,
            },
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

    #[task(binds = TIM1_UP, priority = 3, local = [  timer_1, internal_led, led_state: bool = true])]
    fn delay_task_1(mut cx: delay_task_1::Context) {
        defmt::debug!("delay1: timer task");
        if *cx.local.led_state {
            cx.local.internal_led.set_high();
            *cx.local.led_state = false;
        } else {
            cx.local.internal_led.set_low();
            *cx.local.led_state = true;
        }

        cx.local.timer_1.start(300.millis()).unwrap();
        cx.local.timer_1.clear_interrupt(Event::Update);
        defmt::debug!("delay1: after 1 secs");
    }

    #[task(binds = TIM2, priority = 4, local = [  timer_2, out_led, led_state: bool = true ])]
    fn delay_task_2(mut cx: delay_task_2::Context) {
        defmt::debug!("delay2: timer task");

        if *cx.local.led_state {
            cx.local.out_led.set_high();
            *cx.local.led_state = false;
        } else {
            cx.local.out_led.set_low();
            *cx.local.led_state = true;
        }
        cx.local.timer_2.start(700.millis()).unwrap();
        cx.local.timer_2.clear_interrupt(Event::Update);
        defmt::debug!("delay2: after 1 secs");
    }
}
