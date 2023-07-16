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
        tx_pin: stm32f1xx_hal::serial::Tx3,
        rx_pin: stm32f1xx_hal::serial::Rx3,
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
        let tx_pin = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
        let rx_pin = gpiob.pb11; //.into_pull_down_input(&mut gpiob.crh);

        let channels = cx.device.DMA1.split();
        let mut afio = cx.device.AFIO.constrain();
        let mut serial = Serial::new(
            cx.device.USART3,
            (tx_pin, rx_pin),
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
        // bluetooth_reader::spawn().ok();

        serial.listen(stm32f1xx_hal::serial::Event::Rxne);
        rx.listen();
        (
            Shared {},
            Local {
                rx_pin: rx,
                tx_pin: tx,
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

    #[task(binds = USART3, priority = 1, local = [  rx_pin, tx_pin  ])]
    fn bluetooth_reader(cx: bluetooth_reader::Context) {
        let rx = cx.local.rx_pin;
        loop {
            // let sent = b'X';
            // match cx.local.tx_pin.write(sent) {
            //     Ok(_) => defmt::debug!("write successful"),
            //     Err(_) => defmt::debug!("fail to write"),
            // }

            if rx.is_rx_not_empty() {
                defmt::debug!("is empty: false");
                let received = rx.read();
                match received {
                    Ok(read) => defmt::debug!("receive: {}", read),

                    Err(err) => {
                        defmt::debug!("read err: {:?}", defmt::Debug2Format(&err));
                    }
                }
            }
        }
    }
}
