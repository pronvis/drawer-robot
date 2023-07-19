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
        serial::{Config, Serial},
        timer::Timer,
        timer::{Counter, Event},
    };

    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx_usart2: stm32f1xx_hal::serial::Tx2,
        rx_usart2: stm32f1xx_hal::serial::Rx2,
        tx_usart3: stm32f1xx_hal::serial::Tx3,
        rx_usart3: stm32f1xx_hal::serial::Rx3,
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
        let mut gpioa: stm32f1xx_hal::gpio::gpioa::Parts = cx.device.GPIOA.split();
        // USART2
        let tx_usart2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx_usart2 = gpioa.pa3;
        // USART3
        let tx_usart3 = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
        let rx_usart3 = gpiob.pb11;

        let mut afio = cx.device.AFIO.constrain();
        let mut serial_usart2 = Serial::new(
            cx.device.USART2,
            (tx_usart2, rx_usart2),
            &mut afio.mapr,
            Config::default()
                .baudrate(9600.bps())
                .wordlength_8bits()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1)
                .parity_none(),
            &clocks,
        );
        let mut serial_usart3 = Serial::new(
            cx.device.USART3,
            (tx_usart3, rx_usart3),
            &mut afio.mapr,
            Config::default()
                .baudrate(9600.bps())
                .wordlength_8bits()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1)
                .parity_none(),
            &clocks,
        );

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, STEPPER_CLOCK_FREQ, systick_mono_token);

        serial_usart2.listen(stm32f1xx_hal::serial::Event::Rxne);
        serial_usart3.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (tx_usart2, mut rx_usart2) = serial_usart2.split();
        let (tx_usart3, mut rx_usart3) = serial_usart3.split();
        (
            Shared {},
            Local {
                rx_usart2,
                tx_usart2,
                rx_usart3,
                tx_usart3,
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

    #[task(binds = USART2, priority = 1, local = [  rx_usart2, tx_usart2  ])]
    fn esp32_reader(cx: esp32_reader::Context) {
        let rx = cx.local.rx_usart2;
        if rx.is_rx_not_empty() {
            defmt::debug!("receive data from esp32");
            let received = rx.read();
            match received {
                Ok(read) => defmt::debug!("data from esp32: {}", read),

                Err(err) => {
                    defmt::debug!("data from esp32 read err: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }

    #[task(binds = USART3, priority = 1, local = [  rx_usart3, tx_usart3  ])]
    fn bluetooth_reader(cx: bluetooth_reader::Context) {
        let rx = cx.local.rx_usart3;
        if rx.is_rx_not_empty() {
            defmt::debug!("receive data via bluetooth");
            let received = rx.read();
            match received {
                Ok(read) => defmt::debug!("data via bluetooth: {}", read),

                Err(err) => {
                    defmt::debug!(
                        "data via bluetooth read err: {:?}",
                        defmt::Debug2Format(&err)
                    );
                }
            }
        }
    }
}
