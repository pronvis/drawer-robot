#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
)]
mod app {

    use defmt_brtt as _; // global logger
    use fugit::ExtU32;
    use robot_core::*;
    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::{
        gpio::PinState,
        prelude::*,
        serial::{Config, Serial},
        timer::{counter, Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 72_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx_usart1: stm32f1xx_hal::serial::Tx1,
        send_timer: Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>,
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
        let mut afio = cx.device.AFIO.constrain();

        let tx_usart1 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx_usart1 = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
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

        let (tx_usart1, _) = serial_usart1.split();

        let mut send_timer = robot_core::get_counter(cx.device.TIM1, &clocks);
        send_timer.start(10.millis()).unwrap();
        send_timer.listen(Event::Update);

        (Shared {}, Local { tx_usart1, send_timer })
    }

    #[task(binds = TIM1_UP, priority = 10, local = [ tx_usart1, send_timer, counter: u8 = 0 ])]
    fn hc05_send(cx: hc05_send::Context) {
        cx.local.send_timer.clear_interrupt(Event::Update);

        let mut tx = cx.local.tx_usart1;
        let _ = tx.write(*cx.local.counter);
        defmt::debug!("wrote byte");

        *cx.local.counter += 1;
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            cortex_m::asm::nop();
        }
    }
}
