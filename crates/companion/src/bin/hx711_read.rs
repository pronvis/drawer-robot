#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
    // dispatchers = [PVD, WWDG, RTC, SPI1]
)]
mod app {
    use cortex_m::delay::Delay;
    use defmt_brtt as _; // global logger
    use robot_core::*;
    //This is 4 years old library
    use hx711::Hx711;
    use stm32f1xx_hal::{
        gpio::{Output, Pin},
        prelude::*,
        timer::{counter, Counter, Event},
    };

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const TIMER_CLOCK_FREQ: u32 = 72_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        hx711: Hx711<Delay, Pin<'A', 6>, Pin<'A', 7, Output>>,
        read_timer: Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>,
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
            .sysclk(MAIN_CLOCK_FREQ.Hz())
            .pclk1(36.MHz())
            .pclk2(MAIN_CLOCK_FREQ.Hz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        let mut gpioa = cx.device.GPIOA.split();

        // Configure the hx711 load cell driver:
        //
        // | HX  | dout   -> PA6 | STM |
        // | 711 | pd_sck <- PA7 | 32  |
        //
        let dout = gpioa.pa6.into_floating_input(&mut gpioa.crl);
        let pd_sck = gpioa.pa7.into_push_pull_output(&mut gpioa.crl);
        let hx711 = Hx711::new(Delay::new(cx.core.SYST, 1_000_000), dout, pd_sck).unwrap();

        let mut read_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        read_timer.start(100.millis()).unwrap();
        read_timer.listen(Event::Update);

        (Shared {}, Local { hx711, read_timer })
    }

    #[task(binds = TIM2, priority = 5, local = [ read_timer, hx711, counter: u8 = 0 ])]
    fn hx711_read(cx: hx711_read::Context) {
        let conversion_rate: f32 = 0.035274;
        cx.local.read_timer.clear_interrupt(Event::Update);
        let hx711_data = cx.local.hx711.retrieve();
        if let Ok(data) = hx711_data {
            let gramms = data as f32 * conversion_rate;
            let kilogramms = gramms / 1000f32;
            defmt::debug!("#{}: DYMH-06 data: {}", *cx.local.counter, kilogramms)
        }
        *cx.local.counter += 1;
    }
}
