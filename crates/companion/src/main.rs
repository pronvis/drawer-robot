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
    use robot_core::CompanionMessage;
    use robot_core::*;
    use stm32f1xx_hal::{
        gpio::{gpioa, Alternate, Output, Pin, PullUp},
        pac::SPI1,
        prelude::*,
        serial::{Config, Serial},
        spi::{self, Mode, Phase, Polarity, Slave, Spi, Spi1NoRemap},
        timer::{counter, Counter, Delay, Event},
    };

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const READ_TIMER_CLOCK_FREQ: u32 = 10_000;
    const SEND_TIMER_CLOCK_FREQ: u32 = 10_000;
    const ADS1256_TIMER_CLOCK_FREQ: u32 = 2_000_000;
    const DYMH_106_RESP_FREQ: u32 = 1_000;

    #[shared]
    struct Shared {
        companion_message: CompanionMessage,
    }

    #[local]
    struct Local {
        tx_usart1: stm32f1xx_hal::serial::Tx1,
        send_timer: Counter<stm32f1xx_hal::pac::TIM1, SEND_TIMER_CLOCK_FREQ>,
        read_timer: Counter<stm32f1xx_hal::pac::TIM2, READ_TIMER_CLOCK_FREQ>,
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

        //DYMH_106_RESP_FREQ is 1_000
        let config = Ads1256Config::new(SamplingRate::Sps2000, PGA::Gain64);
        ads1256.set_config(&config).unwrap();

        // Configure the USART1:
        //
        let tx_usart1 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx_usart1 = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
        let mut serial_usart1 = Serial::new(
            cx.device.USART1,
            (tx_usart1, rx_usart1),
            &mut afio.mapr,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8bits()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1)
                .parity_none(),
            &clocks,
        );
        let (tx_usart1, _) = serial_usart1.split();

        // Timer to read from ADS1256
        let mut read_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        read_timer.start(DYMH_106_RESP_FREQ.Hz::<1, 1>().into_duration()).unwrap();
        read_timer.listen(Event::Update);

        // Timer to send data via bluetooth
        let mut send_timer = robot_core::get_counter(cx.device.TIM1, &clocks);
        send_timer.start(30.millis()).unwrap();
        send_timer.listen(Event::Update);

        (
            Shared {
                companion_message: Default::default(),
            },
            Local {
                ads1256,
                read_timer,
                tx_usart1,
                send_timer,
            },
        )
    }

    //for PGA::Gain64
    trait DymhVal {
        //global max & min possible values from sensor
        const max_val: i32 = 50_000;
        const min_val: i32 = -100_000;
        fn dymh_norm_val(self) -> i32;
    }

    const dymh_default_min: i32 = -18000;
    const dymh_default_max: i32 = -12000;
    impl DymhVal for i32 {
        fn dymh_norm_val(self) -> i32 {
            if self <= dymh_default_max && self >= dymh_default_min {
                return 0;
            }

            if self < dymh_default_min {
                return self + (dymh_default_min * -1);
            }

            return self + (dymh_default_max * -1);
        }
    }

    #[task(binds = TIM2, priority = 10, local = [ read_timer, ads1256, counter: u32 = 0, max: i32 = i32::MIN, min: i32 = i32::MAX, first_start: bool = true, last_elems: i32 = 0, last_elems_counter: u32 = 0 ], shared = [companion_message])]
    fn ads1256_read(mut cx: ads1256_read::Context) {
        cx.local.read_timer.clear_interrupt(Event::Update);
        if *cx.local.first_start {
            for _ in 0..READ_TIMER_CLOCK_FREQ {
                cortex_m::asm::nop();
            }
            *cx.local.first_start = false;
        }

        let last_elems_counter: u32 = *cx.local.last_elems_counter;
        let mut last_elems: i32 = *cx.local.last_elems;

        let slice_size = 100;
        if last_elems_counter == 5000 {
            defmt::debug!("mean: {}", last_elems as f32 / slice_size as f32);
            *cx.local.last_elems_counter = 0;
        } else {
            *cx.local.last_elems_counter += 1;
        }

        if last_elems_counter % slice_size == 0 {
            *cx.local.last_elems = 0;
        }

        let mut ads1256 = cx.local.ads1256;

        //TODO: I have only one connected sensor for now
        let code_res = ads1256.read_channel(Channel::AIN0, Channel::AIN1);
        let mut buffer: [i32; 4] = [0, i32::MIN, i32::MIN, i32::MIN];
        let mut current: i32 = 0;
        if let Ok(result) = code_res {
            *cx.local.max = (*cx.local.max).max(result);
            *cx.local.min = (*cx.local.min).min(result);
            current = result;
            *cx.local.last_elems += current;
            let in_volt = ads1256.convert_to_volt(result);
            buffer[0] = result.dymh_norm_val();
        } else {
            defmt::error!("fail to read from ads1256");
        }

        if *cx.local.counter == DYMH_106_RESP_FREQ {
            *cx.local.counter = 0;
            defmt::debug!(
                "max: {}\tmin: {}\tcurrent: {}\tcurrent normalized: {}",
                cx.local.max,
                cx.local.min,
                current,
                current.dymh_norm_val()
            );
        } else {
            *cx.local.counter += 1;
        }

        cx.shared.companion_message.lock(|cm| {
            cm.load_sensor_0 = buffer[0];
            cm.load_sensor_1 = buffer[1];
            cm.load_sensor_2 = buffer[2];
            cm.load_sensor_3 = buffer[3];
        });
    }

    #[task(binds = TIM1_UP, priority = 5, local = [ tx_usart1, send_timer, curr_tensor_0_val: i32 = 0 ], shared = [companion_message])]
    fn hc05_send(mut cx: hc05_send::Context) {
        cx.local.send_timer.clear_interrupt(Event::Update);

        let companion_message = cx.shared.companion_message.lock(|cm| *cm);

        //lets try to send data only if sensor update data at least for 100gr
        let prev_tensor_0_val: &mut i32 = cx.local.curr_tensor_0_val;
        let curr_tensor_0_val = companion_message.load_sensor_0;
        let tensor_diff = robot_core::i32_diff(curr_tensor_0_val, *prev_tensor_0_val);

        if tensor_diff >= 100 {
            // defmt::debug!("sending data, diff: {}, curr: {}", tensor_diff, curr_tensor_0_val);
            *prev_tensor_0_val = curr_tensor_0_val;

            let mut data_to_send: [u8; 17] = [0; 17];
            data_to_send[0] = robot_core::COMPANION_SYNC;
            //here I send only 0 sensor data, cause it is only one attached
            let sensor_0 = curr_tensor_0_val.to_be_bytes();
            for i in 0..4 {
                data_to_send[i + 1] = sensor_0[i];
            }

            let mut tx = cx.local.tx_usart1;
            for elem in data_to_send.iter() {
                let mut write_res = tx.write(*elem);
                while write_res.is_err() {
                    //TODO: Make it async
                    for _ in 0..72 {
                        cortex_m::asm::nop();
                    }
                    write_res = tx.write(*elem);
                }
            }
        }
    }
}
