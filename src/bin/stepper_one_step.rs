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
    use stepper::{
        compat, fugit::NanosDurationU32 as Nanoseconds, motion_control,
        motion_control::SoftwareMotionControl, ramp_maker, Direction, Stepper,
    };
    use stm32f1xx_hal::{pac, prelude::*};

    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;

    type Num = fixed::FixedI64<typenum::U32>;

    // type Profile_Type = ramp_maker::Trapezoidal<fixed::FixedI64<Num>>;
    type Profile_Type = ramp_maker::Trapezoidal<
        fixed::FixedI64<
            typenum::UInt<
                typenum::UInt<
                    typenum::UInt<
                        typenum::UInt<
                            typenum::UInt<typenum::UInt<typenum::UTerm, typenum::B1>, typenum::B0>,
                            typenum::B0,
                        >,
                        typenum::B0,
                    >,
                    typenum::B0,
                >,
                typenum::B0,
            >,
        >,
    >;

    type Stepper_Type = Stepper<
        stepper::drivers::drv8825::DRV8825<
            (),
            (),
            (),
            (),
            (),
            (),
            (),
            compat::Pin<StepPin>,
            compat::Pin<DirPin>,
        >,
    >;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        stepper: Stepper_Type,
        timer: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM2, STEPPER_CLOCK_FREQ>,
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

        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();
        let step = gpioc.pc6.into_push_pull_output(&mut gpioc.crl);
        let mut dir = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
        dir.set_low();
        // Define the numeric type we're going to use. We'll use a fixed-point type
        // here, as that's the most widely supported. If your target hardware has
        // support for floating point, it might be more convenient (and possibly
        // efficient) to use that instead.

        // Define the target acceleration and maximum speed using timer ticks as the
        // unit of time. We could also use seconds or any other unit of time
        // (Stepper doesn't care), but then we'd need to provide a conversion from
        // seconds to timer ticks. This way, we save that conversion.
        //
        // These values assume a 1 MHz (values were 0.001) timer, but that depends on the timer you're
        // using, of course.
        // TODO: calculate right values for your clock speed
        let target_accel = Num::from_num(0.0000008); // steps / tick^2; 1000 steps / s^2
        let max_speed = Num::from_num(0.000000008);

        // Now we need to initialize the stepper API. We do this by initializing a
        // driver (`MyDriver`), then wrapping that into the generic API (`Stepper`).
        // `MyDriver` is a placeholder. In a real use-case, you'd typically use one
        // of the drivers from the `stepper::drivers` module, but any driver that
        // implements the traits from `stepper::traits` will do.
        //
        // By default, drivers can't do anything after being initialized. This means
        // they also don't require any hardware resources, which makes them easier
        // to use when you don't need all features.
        let mut timer =
            stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM2, STEPPER_CLOCK_FREQ>::new(
                cx.device.TIM2,
                &clocks,
            );
        let mut timer: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM2, STEPPER_CLOCK_FREQ> =
            timer.counter();
        let stepper: Stepper_Type = Stepper::from_driver(stepper::drivers::drv8825::DRV8825::new())
            // Enable direction control
            .enable_direction_control(compat::Pin { 0: dir }, Direction::Backward, &mut timer)
            .unwrap()
            // Enable step control
            .enable_step_control(compat::Pin { 0: step });

        //////////////////////////
        //////////////////////////
        //////////////////////////

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 72_000_000, systick_mono_token);

        task3::spawn().ok();

        (Shared {}, Local { stepper, timer })
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            rtic::export::nop();
        }
    }

    #[task(priority = 3, local = [ stepper, timer ])]
    async fn task3(mut cx: task3::Context) {
        defmt::debug!("Move motor!");

        cx.local
            .stepper
            .set_direction(stepper::Direction::Forward, cx.local.timer)
            .wait()
            .unwrap();

        loop {
            cx.local.stepper.step(cx.local.timer).wait().unwrap();
            Systick::delay(500.micros()).await;
        }
    }

    use num_traits::cast::ToPrimitive;
    pub struct DelayToTicks;
    impl<const TIMER_HZ: u32> stepper::motion_control::DelayToTicks<Num, TIMER_HZ> for DelayToTicks {
        type Error = core::convert::Infallible;

        fn delay_to_ticks(
            &self,
            delay: Num,
        ) -> Result<fugit::TimerDurationU32<TIMER_HZ>, Self::Error> {
            Ok(fugit::TimerDurationU32::<TIMER_HZ>::from_ticks(
                Num::to_u32(&delay).expect("the delay to convert"),
            ))
        }
    }
}
