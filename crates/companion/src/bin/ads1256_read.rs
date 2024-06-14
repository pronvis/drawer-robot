#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use core::fmt::Pointer;

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
    // dispatchers = [PVD, WWDG, RTC, SPI1]
)]
mod app {
    use ads1256::{Channel, Config as Ads1256Config, SamplingRate, ADS1256, PGA};
    use defmt_brtt as _; // global logger
    use embedded_hal::digital::OutputPin;
    use robot_core::*;
    use stm32f1xx_hal::{
        gpio::{gpioa, Output, Pin, PullUp},
        pac::SPI1,
        prelude::*,
        spi::{self, Mode, Phase, Polarity, Spi, Spi1NoRemap},
        timer::{counter, Counter, Delay, Event},
    };

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const TIMER_CLOCK_FREQ: u32 = 10_000;
    const ADS1256_TIMER_CLOCK_FREQ: u32 = 2_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        read_timer: Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>,
        ads1256: ADS1256<
            Spi<
                stm32f1xx_hal::pac::SPI1,
                Spi1NoRemap,
                (
                    stm32f1xx_hal::gpio::Pin<'A', 5, stm32f1xx_hal::gpio::Alternate>,
                    stm32f1xx_hal::gpio::Pin<'A', 6>,
                    stm32f1xx_hal::gpio::Pin<'A', 7, stm32f1xx_hal::gpio::Alternate>,
                ),
                u8,
            >,
            stm32f1xx_hal::gpio::Pin<'A', 4, stm32f1xx_hal::gpio::Output>,
            stm32f1xx_hal::gpio::Pin<'A', 3, stm32f1xx_hal::gpio::Output>,
            stm32f1xx_hal::gpio::Pin<'A', 2, stm32f1xx_hal::gpio::Input<PullUp>>,
            Delay<stm32f1xx_hal::pac::TIM3, 2000000>,
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
            .sysclk(MAIN_CLOCK_FREQ.Hz())
            .pclk1(36.MHz())
            .pclk2(MAIN_CLOCK_FREQ.Hz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        let mut gpioa = cx.device.GPIOA.split();
        let mut afio = cx.device.AFIO.constrain();

        // Configure the ads1256  driver:
        //
        // SPI pins:
        // MOSI  - PA7 // DIN
        // MISO  - PA6 // DOUT
        // SCK	 - PA5 // SCLK
        // SS	 - PA4 // CS
        // --------------------
        // --------------------
        // Other pins:
        // DRDY  - PA2
        // PDWN  - +3.3 V
        //
        let spi_pins = (
            gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa6.into_floating_input(&mut gpioa.crl),
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
        );
        let ads1256_spi = spi::Spi::spi1(cx.device.SPI1, spi_pins, &mut afio.mapr, embedded_hal::spi::MODE_1, 8.MHz(), clocks);
        let cs_pin = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        // reset_pin does not used anywhere in ADS1256 codebase
        let reset_pin = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let data_ready_pin = gpioa.pa2.into_pull_up_input(&mut gpioa.crl);
        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM3, ADS1256_TIMER_CLOCK_FREQ>::new(cx.device.TIM3, &clocks);
        let mut ads1256 = ADS1256::new(ads1256_spi, cs_pin, reset_pin, data_ready_pin, timer.delay()).unwrap();

        let config = Ads1256Config::new(SamplingRate::Sps30000, PGA::Gain1);
        ads1256.set_config(&config).unwrap();

        // Debug Timer
        let mut read_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        read_timer.start(1.Hz::<1, 1>().into_duration()).unwrap();
        read_timer.listen(Event::Update);

        (Shared {}, Local { read_timer, ads1256 })
    }

    #[task(binds = TIM2, priority = 5, local = [read_timer, ads1256])]
    fn hx711_read(cx: hx711_read::Context) {
        cx.local.read_timer.clear_interrupt(Event::Update);
        let conversion_rate: f32 = 0.035274;

        let mut ads1256 = cx.local.ads1256;
        for ch in &[
            Channel::AIN0,
            Channel::AIN1,
            // Channel::AIN2,
            // Channel::AIN3,
            // Channel::AIN4,
            // Channel::AIN5,
            // Channel::AIN6,
            // Channel::AIN7,
        ] {
            let code = ads1256.read_channel(*ch, Channel::AINCOM).unwrap();
            let in_volt = ads1256.convert_to_volt(code);
            let gramms = code as f32 * conversion_rate;
            let kilogramms = gramms / 1000f32;
            defmt::debug!("Channel {:?} : {:#08x}, {} V, kg: {}", ch, code, in_volt, kilogramms);
        }
        defmt::debug!("============================================================");
    }
}
