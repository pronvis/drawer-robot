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

    pub type X_EnPin = stm32f1xx_hal::gpio::Pin<'C', 7, stm32f1xx_hal::gpio::Output>;
    pub type X_StepPin = stm32f1xx_hal::gpio::Pin<'C', 6, stm32f1xx_hal::gpio::Output>;
    pub type X_DirPin = stm32f1xx_hal::gpio::Pin<'B', 15, stm32f1xx_hal::gpio::Output>;

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
        speed: u8,
        stepper_1: MyStepper<stm32f1xx_hal::pac::TIM2, X_StepPin, TIMER_CLOCK_FREQ>,
        stepper_2: MyStepper<stm32f1xx_hal::pac::TIM3, Y_StepPin, TIMER_CLOCK_FREQ>,
        stepper_3: MyStepper<stm32f1xx_hal::pac::TIM4, Z_StepPin, TIMER_CLOCK_FREQ>,
        stepper_4: MyStepper<stm32f1xx_hal::pac::TIM5, E_StepPin, TIMER_CLOCK_FREQ>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let mut stepper_state = MyStepperState::new(1500, 200);

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
        let x_step_pin = gpioc
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

        let mut x_en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        x_en.set_low();

        let mut y_en = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        y_en.set_low();

        let mut z_en = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
        z_en.set_low();

        let mut e_en = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        e_en.set_low();

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

        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM5, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM5,
            &clocks,
        );
        let mut timer_5: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM5, TIMER_CLOCK_FREQ> =
            timer.counter();
        timer_5
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_5.listen(Event::Update);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, STEPPER_CLOCK_FREQ, systick_mono_token);

        let (sender_1, stepper_1_commands_receiver) =
            make_channel!(MyStepperCommands, CHANNEL_CAPACITY);
        let (sender_2, stepper_2_commands_receiver) =
            make_channel!(MyStepperCommands, CHANNEL_CAPACITY);
        let (sender_3, stepper_3_commands_receiver) =
            make_channel!(MyStepperCommands, CHANNEL_CAPACITY);
        let (sender_4, stepper_4_commands_receiver) =
            make_channel!(MyStepperCommands, CHANNEL_CAPACITY);
        speed_changer::spawn(sender_1, sender_2, sender_3, sender_4).ok();

        let stepper_1 = MyStepper::new(
            1,
            stepper_state.clone(),
            timer_2,
            x_step_pin,
            stepper_1_commands_receiver,
        );
        let stepper_2 = MyStepper::new(
            2,
            stepper_state.clone(),
            timer_3,
            y_step_pin,
            stepper_2_commands_receiver,
        );
        let stepper_3 = MyStepper::new(
            3,
            stepper_state.clone(),
            timer_4,
            z_step_pin,
            stepper_3_commands_receiver,
        );
        let stepper_4 = MyStepper::new(
            4,
            stepper_state.clone(),
            timer_5,
            e_step_pin,
            stepper_4_commands_receiver,
        );

        (
            Shared {
                stepper_state: stepper_state.clone(),
            },
            Local {
                speed: 0,
                stepper_1,
                stepper_2,
                stepper_3,
                stepper_4,
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

    #[task(priority = 3, local = [ speed, increasing: bool = true, stop_index: u8 = 0 ])]
    async fn speed_changer(
        cx: speed_changer::Context,
        mut sender_1: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
        mut sender_2: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
        mut sender_3: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
        mut sender_4: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    ) {
        loop {
            Systick::delay(200.millis()).await;

            // if *cx.local.stop_index == 25 {
            //     *cx.local.stop_index = 0;
            //     let command = MyStepperCommands::Stay;

            //     sender_1.send(command.clone()).await.unwrap();
            //     sender_2.send(command.clone()).await.unwrap();
            //     sender_3.send(command.clone()).await.unwrap();
            //     sender_4.send(command.clone()).await.unwrap();

            //     Systick::delay(500.millis()).await;
            //     continue;
            // }

            if *cx.local.increasing {
                *cx.local.speed += 1;
            } else {
                *cx.local.speed -= 1;
            }

            if *cx.local.speed > MAX_SPEED_VAL {
                *cx.local.increasing = false;
                *cx.local.speed = MAX_SPEED_VAL - 1;
            }

            if *cx.local.speed == 0 {
                *cx.local.increasing = true;
            }

            let command = MyStepperCommands::Move(*cx.local.speed);
            sender_1.send(command.clone()).await.unwrap();
            sender_2.send(command.clone()).await.unwrap();
            sender_3.send(command.clone()).await.unwrap();
            sender_4.send(command.clone()).await.unwrap();

            // *cx.local.stop_index += 1;
        }
    }

    #[task(binds = TIM2, priority = 3, local = [ stepper_1 ])]
    fn delay_task_1(cx: delay_task_1::Context) {
        cx.local.stepper_1.work();
    }

    #[task(binds = TIM3, priority = 3, local = [ stepper_2 ])]
    fn delay_task_2(cx: delay_task_2::Context) {
        cx.local.stepper_2.work();
    }

    #[task(binds = TIM4, priority = 3, local = [ stepper_3 ])]
    fn delay_task_3(cx: delay_task_3::Context) {
        cx.local.stepper_3.work();
    }

    #[task(binds = TIM5, priority = 3, local = [ stepper_4 ])]
    fn delay_task_4(cx: delay_task_4::Context) {
        cx.local.stepper_4.work();
    }
}
