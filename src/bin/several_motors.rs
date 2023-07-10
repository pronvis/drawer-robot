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

    use drawer_robot::my_stepper::*;
    use drawer_robot::*;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stepper::{
        compat, fugit::NanosDurationU32 as Nanoseconds, motion_control,
        motion_control::SoftwareMotionControl, ramp_maker, Direction, Stepper,
    };
    use stm32f1xx_hal::{
        gpio::PinState,
        pac,
        prelude::*,
        rcc,
        rcc::*,
        timer::Timer,
        timer::{Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 10_000; // step = 100_000 nanos, 100 micros
    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;
    const CHANNEL_CAPACITY: usize = 1;

    pub type Y_EnPin = stm32f1xx_hal::gpio::Pin<'B', 16, stm32f1xx_hal::gpio::Output>;
    pub type Y_StepPin = stm32f1xx_hal::gpio::Pin<'B', 13, stm32f1xx_hal::gpio::Output>;
    pub type Y_DirPin = stm32f1xx_hal::gpio::Pin<'B', 12, stm32f1xx_hal::gpio::Output>;

    pub type Z_EnPin = stm32f1xx_hal::gpio::Pin<'B', 11, stm32f1xx_hal::gpio::Output>;
    pub type Z_StepPin = stm32f1xx_hal::gpio::Pin<'B', 10, stm32f1xx_hal::gpio::Output>;
    pub type Z_DirPin = stm32f1xx_hal::gpio::Pin<'B', 2, stm32f1xx_hal::gpio::Output>;

    pub type E_EnPin = stm32f1xx_hal::gpio::Pin<'B', 1, stm32f1xx_hal::gpio::Output>;
    pub type E_StepPin = stm32f1xx_hal::gpio::Pin<'B', 0, stm32f1xx_hal::gpio::Output>;
    pub type E_DirPin = stm32f1xx_hal::gpio::Pin<'C', 5, stm32f1xx_hal::gpio::Output>;

    #[shared]
    struct Shared {
        stepper_state: MyStepperState,
    }

    #[local]
    struct Local {
        step_pin: StepPin,
        timer_1: Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>,
        timer_2: Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>,
        timer_3: Counter<stm32f1xx_hal::pac::TIM3, TIMER_CLOCK_FREQ>,
        timer_4: Counter<stm32f1xx_hal::pac::TIM4, TIMER_CLOCK_FREQ>,
        y_step_pin: Y_StepPin,
        z_step_pin: Z_StepPin,
        e_step_pin: E_StepPin,
        default_stepper_1: MyStepperState,
        default_stepper_2: MyStepperState,
        default_stepper_3: MyStepperState,
        default_stepper_4: MyStepperState,
        default_stepper_5: MyStepperState,
        stepper_receiver_1: Receiver<'static, u32, CHANNEL_CAPACITY>,
        stepper_receiver_2: Receiver<'static, u32, CHANNEL_CAPACITY>,
        stepper_receiver_3: Receiver<'static, u32, CHANNEL_CAPACITY>,
        stepper_receiver_4: Receiver<'static, u32, CHANNEL_CAPACITY>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let mut stepper_state = MyStepperState {
            micros_between_steps: 1500,
            micros_pulse_duration: 200,
        };

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
        let step_pin = gpioc
            .pc6
            .into_push_pull_output_with_state(&mut gpioc.crl, PinState::Low);
        let y_step_pin = gpiob
            .pb13
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low);
        let z_step_pin = gpiob
            .pb10
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low);
        let e_step_pin = gpiob
            .pb0
            .into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low);

        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();

        let mut y_en = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        y_en.set_low();

        let mut z_en = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
        z_en.set_low();

        let mut e_en = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        e_en.set_low();

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM1,
            &clocks,
        );
        let mut timer_1: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ> =
            timer.counter();
        timer_1
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_1.listen(Event::Update);

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM2,
            &clocks,
        );
        let mut timer_2: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ> =
            timer.counter();
        timer_2
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_2.listen(Event::Update);

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM3, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM3,
            &clocks,
        );
        let mut timer_3: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM3, TIMER_CLOCK_FREQ> =
            timer.counter();
        timer_3
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_3.listen(Event::Update);

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM4, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM4,
            &clocks,
        );
        let mut timer_4: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM4, TIMER_CLOCK_FREQ> =
            timer.counter();
        timer_4
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_4.listen(Event::Update);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, STEPPER_CLOCK_FREQ, systick_mono_token);

        let (sender_1, stepper_receiver_1) = make_channel!(u32, CHANNEL_CAPACITY);
        let (sender_2, stepper_receiver_2) = make_channel!(u32, CHANNEL_CAPACITY);
        let (sender_3, stepper_receiver_3) = make_channel!(u32, CHANNEL_CAPACITY);
        let (sender_4, stepper_receiver_4) = make_channel!(u32, CHANNEL_CAPACITY);
        speed_changer::spawn(sender_1, sender_2, sender_3, sender_4).ok();

        (
            Shared {
                stepper_state: stepper_state.clone(),
            },
            Local {
                step_pin,
                timer_1,
                timer_2,
                timer_3,
                timer_4,
                y_step_pin,
                z_step_pin,
                e_step_pin,
                default_stepper_1: stepper_state.clone(),
                default_stepper_2: stepper_state.clone(),
                default_stepper_3: stepper_state.clone(),
                default_stepper_4: stepper_state.clone(),
                default_stepper_5: stepper_state.clone(),
                stepper_receiver_1,
                stepper_receiver_2,
                stepper_receiver_3,
                stepper_receiver_4,
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

    #[task(priority = 1, local = [ default_stepper_1, increasing: bool = true ])]
    async fn speed_changer(
        cx: speed_changer::Context,
        mut sender_1: Sender<'static, u32, CHANNEL_CAPACITY>,
        mut sender_2: Sender<'static, u32, CHANNEL_CAPACITY>,
        mut sender_3: Sender<'static, u32, CHANNEL_CAPACITY>,
        mut sender_4: Sender<'static, u32, CHANNEL_CAPACITY>,
    ) {
        loop {
            Systick::delay(100.millis()).await;

            if *cx.local.increasing {
                cx.local.default_stepper_1.micros_between_steps += 100;
            } else {
                cx.local.default_stepper_1.micros_between_steps -= 100;
            }

            if cx.local.default_stepper_1.micros_between_steps > 1500 {
                *cx.local.increasing = false;
                cx.local.default_stepper_1.micros_between_steps = 1400;
            }

            if cx.local.default_stepper_1.micros_between_steps < 300 {
                *cx.local.increasing = true;
                cx.local.default_stepper_1.micros_between_steps = 400;
            }

            sender_1
                .send(cx.local.default_stepper_1.micros_between_steps)
                .await
                .unwrap();
            sender_2
                .send(cx.local.default_stepper_1.micros_between_steps)
                .await
                .unwrap();
            sender_3
                .send(cx.local.default_stepper_1.micros_between_steps)
                .await
                .unwrap();
            sender_4
                .send(cx.local.default_stepper_1.micros_between_steps)
                .await
                .unwrap();
        }
    }

    #[task(binds = TIM1_UP, priority = 3, local = [  default_stepper_2, stepper_receiver_1, timer_1, step_pin, is_step: bool = true])]
    fn delay_task_1(cx: delay_task_1::Context) {
        if !cx.local.stepper_receiver_1.is_empty() {
            let new_micros_between_steps = cx.local.stepper_receiver_1.try_recv().unwrap();
            cx.local.default_stepper_2.micros_between_steps = new_micros_between_steps;
            defmt::info!(
                "new speed on stepper_1: {}",
                cx.local.default_stepper_2.micros_between_steps
            );
        }

        if *cx.local.is_step {
            cx.local.step_pin.set_low();
            *cx.local.is_step = false;
            cx.local
                .timer_1
                .start(cx.local.default_stepper_2.micros_between_steps.micros())
                .unwrap();
        } else {
            cx.local.step_pin.set_high();
            *cx.local.is_step = true;
            cx.local
                .timer_1
                .start(cx.local.default_stepper_2.micros_pulse_duration.micros())
                .unwrap();
        }

        cx.local.timer_1.clear_interrupt(Event::Update);
    }

    #[task(binds = TIM2, priority = 3, local = [  default_stepper_3, stepper_receiver_2, timer_2, y_step_pin, is_step: bool = true])]
    fn delay_task_2(cx: delay_task_2::Context) {
        if !cx.local.stepper_receiver_2.is_empty() {
            let new_micros_between_steps = cx.local.stepper_receiver_2.try_recv().unwrap();
            cx.local.default_stepper_3.micros_between_steps = new_micros_between_steps;
            defmt::info!(
                "new speed on stepper_2: {}",
                cx.local.default_stepper_3.micros_between_steps
            );
        }
        if *cx.local.is_step {
            cx.local.y_step_pin.set_low();
            *cx.local.is_step = false;
            cx.local
                .timer_2
                .start(cx.local.default_stepper_3.micros_between_steps.micros())
                .unwrap();
        } else {
            cx.local.y_step_pin.set_high();
            *cx.local.is_step = true;
            cx.local
                .timer_2
                .start(cx.local.default_stepper_3.micros_pulse_duration.micros())
                .unwrap();
        }

        cx.local.timer_2.clear_interrupt(Event::Update);
    }

    #[task(binds = TIM3, priority = 3, local = [  default_stepper_4, stepper_receiver_3, timer_3, z_step_pin, is_step: bool = true])]
    fn delay_task_3(cx: delay_task_3::Context) {
        if !cx.local.stepper_receiver_3.is_empty() {
            let new_micros_between_steps = cx.local.stepper_receiver_3.try_recv().unwrap();
            cx.local.default_stepper_4.micros_between_steps = new_micros_between_steps;
            defmt::info!(
                "new speed on stepper_3: {}",
                cx.local.default_stepper_4.micros_between_steps
            );
        }
        if *cx.local.is_step {
            cx.local.z_step_pin.set_low();
            *cx.local.is_step = false;
            cx.local
                .timer_3
                .start(cx.local.default_stepper_4.micros_between_steps.micros())
                .unwrap();
        } else {
            cx.local.z_step_pin.set_high();
            *cx.local.is_step = true;
            cx.local
                .timer_3
                .start(cx.local.default_stepper_4.micros_pulse_duration.micros())
                .unwrap();
        }

        cx.local.timer_3.clear_interrupt(Event::Update);
    }

    #[task(binds = TIM4, priority = 3, local = [  default_stepper_5, stepper_receiver_4, timer_4, e_step_pin, is_step: bool = true])]
    fn delay_task_4(cx: delay_task_4::Context) {
        if !cx.local.stepper_receiver_4.is_empty() {
            let new_micros_between_steps = cx.local.stepper_receiver_4.try_recv().unwrap();
            cx.local.default_stepper_5.micros_between_steps = new_micros_between_steps;
            defmt::info!(
                "new speed on stepper_4: {}",
                cx.local.default_stepper_5.micros_between_steps
            );
        }
        if *cx.local.is_step {
            cx.local.e_step_pin.set_low();
            *cx.local.is_step = false;
            cx.local
                .timer_4
                .start(cx.local.default_stepper_5.micros_between_steps.micros())
                .unwrap();
        } else {
            cx.local.e_step_pin.set_high();
            *cx.local.is_step = true;
            cx.local
                .timer_4
                .start(cx.local.default_stepper_5.micros_pulse_duration.micros())
                .unwrap();
        }

        cx.local.timer_4.clear_interrupt(Event::Update);
    }
}
