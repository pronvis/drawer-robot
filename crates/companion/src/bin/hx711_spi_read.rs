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
    use hx711_spi::*;
    use robot_core::*;
    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::{
        gpio::{Alternate, Pin, PinState},
        pac::SPI2,
        prelude::*,
        serial::{Config, Serial},
        spi::{self, Mode, Phase, Polarity, Spi, Spi2NoRemap},
        timer::{counter, Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 72_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx_usart1: stm32f1xx_hal::serial::Tx1,
        send_timer: Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>,
        read_timer: Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>,
        hx711_sensor: Hx711<Spi<SPI2, Spi2NoRemap, (Pin<'B', 13, Alternate>, Pin<'B', 14>, Pin<'B', 15, Alternate>), u8>>,
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
        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let mut afio = cx.device.AFIO.constrain();

        // ============== HC-05 set up ===============
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
        // ===========================================

        // ============== HX-711 set up ===============
        let hx711_spi_pins = (
            gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
            gpiob.pb14.into_floating_input(&mut gpiob.crh),
            gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
        );
        let mut hx711_spi = spi::Spi::spi2(cx.device.SPI2, hx711_spi_pins, embedded_hal::spi::MODE_1, 1.MHz(), clocks);
        let mut hx711_sensor = Hx711::new(hx711_spi);
        hx711_sensor.reset().unwrap();
        hx711_sensor.set_mode(hx711_spi::Mode::ChAGain128).unwrap();

        let mut read_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        read_timer.start(20.millis()).unwrap();
        read_timer.listen(Event::Update);
        // ===========================================

        (
            Shared {},
            Local {
                tx_usart1,
                send_timer,
                read_timer,
                hx711_sensor,
            },
        )
    }

    #[task(binds = TIM1_UP, priority = 1, local = [ tx_usart1, send_timer, counter: u8 = 0 ])]
    fn hc05_send(cx: hc05_send::Context) {
        cx.local.send_timer.clear_interrupt(Event::Update);

        let mut tx = cx.local.tx_usart1;
        let _ = tx.write(*cx.local.counter);

        *cx.local.counter += 1;
    }

    #[task(binds = TIM2, priority = 5, local = [ read_timer, hx711_sensor, counter: u8 = 0 ])]
    fn hx711_read(cx: hx711_read::Context) {
        let conversion_rate: f32 = 0.035274;
        cx.local.read_timer.clear_interrupt(Event::Update);
        let hx711_data = cx.local.hx711_sensor.read();
        if let Ok(data) = hx711_data {
            let gramms = data as f32 * conversion_rate;
            let kilogramms = gramms / 1000f32;
            defmt::debug!("#{}: DYMH-06 data: {}", *cx.local.counter, kilogramms)
        }
        *cx.local.counter += 1;
    }
}
