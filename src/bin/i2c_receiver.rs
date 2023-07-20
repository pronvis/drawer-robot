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
    use embedded_hal::digital::v2::OutputPin;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stepper::{
        compat, fugit::NanosDurationU32 as Nanoseconds, motion_control,
        motion_control::SoftwareMotionControl, ramp_maker, Direction, Stepper,
    };
    use stm32f1xx_hal::i2c::{BlockingI2c, DutyCycle, Mode};
    use stm32f1xx_hal::{
        gpio::Alternate,
        gpio::OpenDrain,
        gpio::PinState,
        gpio::{PB6, PB7},
        pac,
        prelude::*,
        rcc,
        rcc::*,
        serial::{Config, Serial},
        timer::Timer,
        timer::{Counter, Event},
    };

    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        i2c: BlockingI2c<
            stm32f1xx_hal::pac::I2C1,
            (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>),
        >,
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

        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let mut afio = cx.device.AFIO.constrain();

        let scl: PB6<Alternate<OpenDrain>> = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda: PB7<Alternate<OpenDrain>> = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let i2c: BlockingI2c<
            stm32f1xx_hal::pac::I2C1,
            (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>),
        > = BlockingI2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Standard {
                frequency: 400_000.Hz(),
            },
            clocks,
            1000,
            100,
            1000,
            1000,
        );

        i2c_reder::spawn().ok();
        (Shared {}, Local { i2c })
    }

    #[task(priority = 1, local = [i2c])]
    async fn i2c_reder(cx: i2c_reder::Context) {
        let i2c = cx.local.i2c;
        loop {
            let mut buffer: [u8; 1] = [0];
            match i2c.read(0, &mut buffer) {
                Ok(read) => defmt::debug!("i2c read result: {}", buffer),

                Err(err) => {
                    defmt::debug!("data via i2c read err: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            rtic::export::nop();
        }
    }
}
