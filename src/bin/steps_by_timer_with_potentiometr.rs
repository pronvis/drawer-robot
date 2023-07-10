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
    use rtic_sync::{channel::*, make_channel};
    use stepper::{
        compat, fugit::NanosDurationU32 as Nanoseconds, motion_control,
        motion_control::SoftwareMotionControl, ramp_maker, Direction, Stepper,
    };
    use stm32f1xx_hal::{
        adc::{self, Adc},
        device::ADC1,
        gpio::PinState,
        pac,
        prelude::*,
        timer::{Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 1_000_000;
    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;
    const CHANNEL_CAPACITY: usize = 1;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        step_pin: StepPin,
        timer_1: Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>,
        adc1: Adc<ADC1>,
        pb0: stm32f1xx_hal::gpio::Pin<'B', 0, stm32f1xx_hal::gpio::Analog>,
        receiver: Receiver<'static, u32, CHANNEL_CAPACITY>,
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
        let step_pin = gpioc
            .pc14
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low);

        let mut en = gpioc.pc15.into_push_pull_output(&mut gpioc.crh);
        en.set_low();

        let mut adc1 = adc::Adc::adc1(cx.device.ADC1, clocks);
        let mut gpiob = cx.device.GPIOB.split();
        // Configure pb0 as an analog input
        let mut pb0 = gpiob.pb0.into_analog(&mut gpiob.crl);

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM1,
            &clocks,
        );
        let mut timer_1: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ> =
            timer.counter();
        timer_1.start(2000.nanos()).unwrap();
        timer_1.listen(Event::Update);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, STEPPER_CLOCK_FREQ, systick_mono_token);

        let critical_section: rtic::export::CriticalSection = cx.cs;
        let (sender, receiver) = make_channel!(u32, CHANNEL_CAPACITY);
        task1::spawn(sender).ok();
        (
            Shared {},
            Local {
                step_pin,
                timer_1,
                adc1,
                pb0,
                receiver,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            rtic::export::nop();
        }
    }

    #[task(binds = TIM1_UP, priority = 3, local = [  timer_1, step_pin,receiver, is_step: bool = true, current_delay: u32 = 1_000_000])]
    fn delay_task_1(mut cx: delay_task_1::Context) {
        if cx.local.receiver.is_full() {
            let new_delay = cx.local.receiver.try_recv().unwrap();
            if new_delay != *cx.local.current_delay {
                *cx.local.current_delay = new_delay;
                defmt::debug!("new delay: {}", new_delay);
            }
        }

        if *cx.local.is_step {
            cx.local.step_pin.set_low();
            *cx.local.is_step = false;
            cx.local
                .timer_1
                .start(cx.local.current_delay.nanos())
                .unwrap();
        } else {
            cx.local.step_pin.set_high();
            *cx.local.is_step = true;
            cx.local.timer_1.start(2000.nanos()).unwrap();
        }

        cx.local.timer_1.clear_interrupt(Event::Update);
    }

    #[task(priority = 1, local = [adc1, pb0])]
    async fn task1(cx: task1::Context, mut sender: Sender<'static, u32, CHANNEL_CAPACITY>) {
        // potentiometr min value: 0; max: 4100

        loop {
            let potent_value = read_potent(cx.local.adc1, cx.local.pb0).await;
            let new_steps_delay = calc_steps_delay(potent_value);
            if sender.is_empty() {
                sender.send(new_steps_delay).await.unwrap();
            }
            Systick::delay(500.millis()).await;
        }
    }
}
