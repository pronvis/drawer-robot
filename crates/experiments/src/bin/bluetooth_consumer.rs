#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(slice_as_chunks)]

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
    // dispatchers = [PVD, WWDG, RTC, SPI1]
)]
mod app {

    use defmt_brtt as _; // global logger
    use robot_core::my_stepper::*;
    use robot_core::*;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
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
        tx_usart1: stm32f1xx_hal::serial::Tx1,
        rx_usart1: stm32f1xx_hal::serial::Rx1,
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

        let mut gpioa: stm32f1xx_hal::gpio::gpioa::Parts = cx.device.GPIOA.split();
        // USART1
        let tx_usart1 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx_usart1 = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

        let mut afio = cx.device.AFIO.constrain();
        let mut serial_usart1 = Serial::new(
            cx.device.USART1,
            (tx_usart1, rx_usart1),
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

        serial_usart1.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (tx_usart1, mut rx_usart1) = serial_usart1.split();
        (Shared {}, Local { rx_usart1, tx_usart1 })
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = USART1, priority = 1, local = [ rx_usart1, tx_usart1, data_to_receive: [u8; 16] = [0; 16], counter: usize = 0 ])]
    fn bluetooth_reader(cx: bluetooth_reader::Context) {
        let rx = cx.local.rx_usart1;
        let mut counter = cx.local.counter;
        if rx.is_rx_not_empty() {
            let received = rx.read();
            match received {
                Ok(read) => {
                    if read == robot_core::COMPANION_SYNC {
                        if *counter != 0 {
                            defmt::debug!("receive SYNC byte when counter = {}", *counter);
                        }
                        *counter = 0;
                    } else {
                        cx.local.data_to_receive[*counter] = read;
                        *counter += 1;
                    }

                    if *counter == 16 {
                        *counter = 0;
                        let splitted = cx.local.data_to_receive[0..4].as_chunks::<4>().0;
                        let bytes: [u8; 4] = splitted[0];
                        let sensor_0_data = i32::from_be_bytes(bytes);

                        let conversion_rate: f32 = 0.035274;
                        let gramms = sensor_0_data as f32 * conversion_rate;
                        let kilogramms = gramms / 1000f32;
                        defmt::debug!("sensor 0 data: {}", kilogramms);
                    }
                }

                Err(err) => {
                    defmt::debug!("read err: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }
}
