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
    //This is 4 years old library
    use hx711::Hx711;
    use stm32f1xx_hal::{
        gpio::{Output, Pin},
        prelude::*,
    };

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        hx711: Hx711<Delay, Pin<'A', 6>, Pin<'A', 7, Output>>,
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

        hx711_read_task::spawn().ok();

        (Shared {}, Local { hx711 })
    }

    #[task(priority = 1, local = [hx711])]
    async fn hx711_read_task(cx: hx711_read_task::Context) {
        let n: i32 = 8;
        let mut val: i32 = 0;

        // Obtain the tara value
        defmt::debug!("Obtaining tara ...");
        for _ in 0..n {
            match cx.local.hx711.retrieve() {
                Ok(curr_val) => val += curr_val,
                Err(_) => defmt::error!("fail to get data from hx711"),
            }
        }

        let tara = val / n;
        defmt::debug!("Tara: {}", tara);

        loop {
            // Measurement loop
            val = 0;
            for _ in 0..n {
                match cx.local.hx711.retrieve() {
                    Ok(curr_val) => val += curr_val,
                    Err(_) => defmt::error!("fail to get data from hx711"),
                }
            }
            let weight = val / n - tara;
            defmt::debug!("weight: {}", weight);
        }
    }
}
