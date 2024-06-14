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
        gpio::{gpioa, Alternate, Output, Pin, PullUp},
        pac::SPI1,
        prelude::*,
        spi::{self, Mode, Phase, Polarity, Slave, Spi, Spi1NoRemap},
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
        //SLAVE SPI
        // ads1256: ADS1256<
        //     Spi<SPI1, Spi1NoRemap, (Pin<'A', 5>, Pin<'A', 6, Alternate>, Pin<'A', 7>), u8, Slave>,
        //     Pin<'A', 4, Output>,
        //     Pin<'A', 3, Output>,
        //     Pin<'A', 2, stm32f1xx_hal::gpio::Input<PullUp>>,
        //     Delay<stm32f1xx_hal::pac::TIM3, 2000000>,
        // >,
        // MASTER SPI
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
        // SLAVE SPI
        // let spi_pins = (gpioa.pa5, gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl), gpioa.pa7);
        // let ads1256_spi = spi::Spi::spi1_slave(cx.device.SPI1, spi_pins, &mut afio.mapr, embedded_hal::spi::MODE_1);
        // MASTER SPI
        let spi_pins = (
            gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa6,
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
        );
        let ads1256_spi = spi::Spi::spi1(
            cx.device.SPI1,
            spi_pins,
            &mut afio.mapr,
            embedded_hal::spi::MODE_1,
            1920000.Hz(),
            clocks,
        );
        let cs_pin = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        // reset_pin does not used anywhere in ADS1256 codebase
        let reset_pin = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        let data_ready_pin = gpioa.pa2.into_pull_up_input(&mut gpioa.crl);
        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM3, ADS1256_TIMER_CLOCK_FREQ>::new(cx.device.TIM3, &clocks);
        let mut ads1256 = ADS1256::new(ads1256_spi, cs_pin, reset_pin, data_ready_pin, timer.delay()).unwrap();

        let config = Ads1256Config::new(SamplingRate::Sps7500, PGA::Gain1);
        ads1256.set_config(&config).unwrap();

        // Debug Timer
        let mut read_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        read_timer.start(2.Hz::<1, 1>().into_duration()).unwrap();
        read_timer.listen(Event::Update);

        (Shared {}, Local { read_timer, ads1256 })
    }

    #[task(binds = TIM2, priority = 5, local = [read_timer, ads1256])]
    fn hx711_read(cx: hx711_read::Context) {
        cx.local.read_timer.clear_interrupt(Event::Update);

        let mut ads1256 = cx.local.ads1256;

        let code_res = ads1256.read_channel(Channel::AIN0, Channel::AIN1);
        match code_res {
            Ok(code) => {
                let in_volt = ads1256.convert_to_volt(code);
                defmt::debug!("Channel 0 : {:#08x}, {} V", code, in_volt);
            }
            Err(err) => match err {
                stm32f1xx_hal::spi::Error::Crc => defmt::error!("error CRC"),
                stm32f1xx_hal::spi::Error::Overrun => defmt::error!("error Overrun"),
                stm32f1xx_hal::spi::Error::ModeFault => defmt::error!("error ModeFault"),
                _ => defmt::error!("unknown error"),
            },
        }
        defmt::debug!("============================================================");
    }
}
