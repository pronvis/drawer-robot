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
    use robot_core::DEFAULT_DYMH06_VALUE;
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

    type Ads1256 = ADS1256<
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
        Delay<stm32f1xx_hal::pac::TIM3, ADS1256_TIMER_CLOCK_FREQ>,
    >;

    #[shared]
    struct Shared {
        companion_message: CompanionMessage,
        ads1256: Ads1256,
    }

    #[local]
    struct Local {
        hc05_tx: stm32f1xx_hal::serial::Tx1,
        hc05_rx: stm32f1xx_hal::serial::Rx1,
        send_timer: Counter<stm32f1xx_hal::pac::TIM1, SEND_TIMER_CLOCK_FREQ>,
        read_timer: Counter<stm32f1xx_hal::pac::TIM2, READ_TIMER_CLOCK_FREQ>,
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
        let config = Ads1256Config::new(SamplingRate::Sps30000, PGA::Gain64);
        ads1256.set_config(&config).unwrap();

        // Configure the USART1:
        // USART1 for HC-05
        // hc-05 RX - PA9
        // hc-05 TX - PA10
        let hc05_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let hc05_rx = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
        let mut serial_usart1 = Serial::new(
            cx.device.USART1,
            (hc05_tx, hc05_rx),
            &mut afio.mapr,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8bits()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1)
                .parity_none(),
            &clocks,
        );
        serial_usart1.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (hc05_tx, hc05_rx) = serial_usart1.split();

        // Timer to read from ADS1256
        let mut read_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        read_timer.start(DYMH_106_RESP_FREQ.Hz::<1, 1>().into_duration()).unwrap();
        read_timer.listen(Event::Update);

        // Timer to send data via bluetooth
        let mut send_timer = robot_core::get_counter(cx.device.TIM1, &clocks);
        //TODO: 30 times per second... looks like not enough
        send_timer.start(30.millis()).unwrap();
        send_timer.listen(Event::Update);

        (
            Shared {
                companion_message: Default::default(),
                ads1256,
            },
            Local {
                read_timer,
                hc05_tx,
                hc05_rx,
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

    #[task(binds = TIM2, priority = 10, local = [ read_timer, counter: u32 = 0, max: i32 = i32::MIN, min: i32 = i32::MAX, first_start: bool = true ], shared = [ads1256, companion_message])]
    fn ads1256_read(mut cx: ads1256_read::Context) {
        cx.local.read_timer.clear_interrupt(Event::Update);
        if *cx.local.first_start {
            for _ in 0..READ_TIMER_CLOCK_FREQ {
                cortex_m::asm::nop();
            }
            *cx.local.first_start = false;
        }

        let mut buffer: [i32; 4] = [
            DEFAULT_DYMH06_VALUE,
            DEFAULT_DYMH06_VALUE,
            DEFAULT_DYMH06_VALUE,
            DEFAULT_DYMH06_VALUE,
        ];

        let first_channel_res = cx.shared.ads1256.lock(|ads1256| {
            let res = read_from_ads_into_buffer(ads1256, 0, &mut buffer);
            read_from_ads_into_buffer(ads1256, 1, &mut buffer);
            read_from_ads_into_buffer(ads1256, 2, &mut buffer);
            read_from_ads_into_buffer(ads1256, 3, &mut buffer);
            res
        });
        // cx.shared.ads1256.lock(|ads1256| {
        //     read_from_ads_into_buffer(ads1256, 1, &mut buffer);
        // });
        // cx.shared.ads1256.lock(|ads1256| {
        //     read_from_ads_into_buffer(ads1256, 2, &mut buffer);
        // });
        // cx.shared.ads1256.lock(|ads1256| {
        //     read_from_ads_into_buffer(ads1256, 3, &mut buffer);
        // });

        //INFO: Debug related
        // ================================
        let mut current: i32 = 0;
        if first_channel_res != DEFAULT_DYMH06_VALUE {
            *cx.local.max = (*cx.local.max).max(first_channel_res);
            *cx.local.min = (*cx.local.min).min(first_channel_res);
            current = first_channel_res;
        }

        if *cx.local.counter == DYMH_106_RESP_FREQ {
            *cx.local.counter = 0;
            defmt::debug!(
                "max: {}\tmin: {}\tcurrent: {}\tcurrent normalized: {}\t\t\tbuffer: {:?}",
                cx.local.max,
                cx.local.min,
                current,
                current.dymh_norm_val(),
                buffer
            );
        } else {
            *cx.local.counter += 1;
        }
        // ================================

        cx.shared.companion_message.lock(|cm| {
            cm.load_sensor_0 = buffer[0];
            cm.load_sensor_1 = buffer[1];
            cm.load_sensor_2 = buffer[2];
            cm.load_sensor_3 = buffer[3];
        });
    }

    fn read_from_ads_into_buffer(ads1256: &mut Ads1256, channel: usize, buffer: &mut [i32; 4]) -> i32 {
        let (channel_0, channel_1) = match channel {
            0 => (Channel::AIN0, Channel::AIN1),
            1 => (Channel::AIN2, Channel::AIN3),
            2 => (Channel::AIN4, Channel::AIN5),
            3 => (Channel::AIN6, Channel::AIN7),
            _ => panic!("wrong channel"),
        };

        ads1256.write_register(ads1256::Register::MUX, channel_0.bits() << 4 | channel_1.bits());
        if let Ok(result) = ads1256.read_channel(channel_0, channel_1) {
            buffer[channel] = result.dymh_norm_val();
            return result;
        } else {
            defmt::error!("fail to read from ads1256 channel: {}", channel);
            return DEFAULT_DYMH06_VALUE;
        }
    }

    #[task(binds = TIM1_UP, priority = 5, local = [ hc05_tx, send_timer, curr_tensor_0_val: i32 = 0 ], shared = [companion_message])]
    fn hc05_send(mut cx: hc05_send::Context) {
        cx.local.send_timer.clear_interrupt(Event::Update);

        let companion_message = cx.shared.companion_message.lock(|cm| *cm);

        //lets try to send data only if sensor update data at least for 100gr
        let prev_tensor_0_val: &mut i32 = cx.local.curr_tensor_0_val;
        let curr_tensor_0_val = companion_message.load_sensor_0;
        let tensor_diff = robot_core::i32_diff(curr_tensor_0_val, *prev_tensor_0_val);

        //TODO: remove 'tensor_diff' condition?
        // if tensor_diff >= 100 {
        // defmt::debug!("sending data, diff: {}, curr: {}", tensor_diff, curr_tensor_0_val);
        *prev_tensor_0_val = curr_tensor_0_val;

        let mut data_to_send: [u8; 17] = [0; 17];
        data_to_send[0] = robot_core::COMPANION_SYNC;
        write_sensor_data(&mut data_to_send, companion_message.load_sensor_0.to_be_bytes(), 1);
        write_sensor_data(&mut data_to_send, companion_message.load_sensor_1.to_be_bytes(), 5);
        write_sensor_data(&mut data_to_send, companion_message.load_sensor_2.to_be_bytes(), 9);
        write_sensor_data(&mut data_to_send, companion_message.load_sensor_3.to_be_bytes(), 13);

        let mut tx = cx.local.hc05_tx;
        tx.bwrite_all(&data_to_send);
        //for elem in data_to_send.iter() {
        //    let mut write_res = tx.write(*elem);
        //    while write_res.is_err() {
        //        //TODO: Make it async
        //        for _ in 0..72 {
        //            cortex_m::asm::nop();
        //        }
        //        write_res = tx.write(*elem);
        //    }
        //}
        // }
    }

    fn write_sensor_data(data_to_send: &mut [u8; 17], sensor_data: [u8; 4], start_index: usize) {
        for i in 0..4 {
            data_to_send[start_index + i] = sensor_data[i];
        }
    }

    #[task(binds = USART1, priority = 6, local = [ hc05_rx ], shared = [ ads1256 ])]
    fn hc05_reader(mut cx: hc05_reader::Context) {
        let rx = cx.local.hc05_rx;
        if rx.is_rx_not_empty() {
            let received = rx.read();
            match received {
                Ok(read) => {
                    if read == robot::HC05_CALIBRATE_ADS1256_CODE {
                        cx.shared.ads1256.lock(|ads1256| {
                            let send_res = ads1256.init();
                            match send_res {
                                Err(_) => defmt::error!("ads1256: fail to send SELFCAL message"),
                                Ok(_) => defmt::debug!("ads1256: SELFCAL finished"),
                                // {
                                //     let mut self_cal_finish = false;
                                //     while !self_cal_finish {
                                //         let is_rdy = ads1256.wait_for_ready();
                                //         if let Ok(is_rdy) = is_rdy {
                                //             self_cal_finish = is_rdy;
                                //             if is_rdy {
                                //                 defmt::debug!("ads1256: SELFCAL finished");
                                //             }
                                //         }
                                //     }
                                // }
                            }
                        });
                    }
                }

                Err(err) => {
                    defmt::debug!("data from hc05 read err: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }
}
