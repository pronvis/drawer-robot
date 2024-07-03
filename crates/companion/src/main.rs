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
    use heapless::spsc::{Consumer, Producer, Queue};
    use robot_core::{robot::TensionData, DEFAULT_DYMH06_VALUE, *};
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

    const DYMH_106_REQ_FREQ: u32 = 500;
    const DYMH_106_QUEUE_SIZE: usize = 30;
    //TODO: 30 times per second... looks like not enough
    const SEND_TIMER_MILLIS_DELAY: u32 = 40;

    //NOTE: To configure dymh106 data sending frequency think about those elements:
    // 1) DYMH_106_RESP_FREQ - freq that you get the data from ads1256 (for all 4 sensors)
    // 2) ads1256 'SamplingRate' - small value gets small noise, but require more time to get data.
    // 3) 'send_timer' delay
    // 4) HC-05 Baud Rate - you cant send data fast with small baud rate
    // 5) DYMH_106_QUEUE_SIZE - queue to buffering DYMH106 data
    // Current values:
    // DYMH_106_REQ_FREQ = 500
    // send_timer.delay = 40 millis
    // DYMH_106_REQ_FREQ / (1.secs() / send_timer.delay) = 20
    // Means averaging on ~20 elements in cache
    // DYMH_106_QUEUE_SIZE = 30
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
        ads1256: Ads1256,
    }

    #[local]
    struct Local {
        hc05_tx: stm32f1xx_hal::serial::Tx1,
        hc05_rx: stm32f1xx_hal::serial::Rx1,
        send_timer: Counter<stm32f1xx_hal::pac::TIM1, SEND_TIMER_CLOCK_FREQ>,
        read_timer: Counter<stm32f1xx_hal::pac::TIM2, READ_TIMER_CLOCK_FREQ>,
        dymh_106_data_producer: Producer<'static, TensionData, DYMH_106_QUEUE_SIZE>,
        dymh_106_data_consumer: Consumer<'static, TensionData, DYMH_106_QUEUE_SIZE>,
    }

    #[init(local = [dymh106_queue: Queue<TensionData, DYMH_106_QUEUE_SIZE> = Queue::new()])]
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

        let config = Ads1256Config::new(SamplingRate::Sps7500, PGA::Gain64);
        ads1256.set_config(&config).unwrap();

        // Configure the USART1:
        // USART1 for HC-05
        // hc-05 RX - PA9
        // hc-05 TX - PA10
        let hc05_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let hc05_rx = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
        // 115200 -> 115200 bits/s
        // 230400 -> 230400 bits/s
        // 460800 -> 460800 bits/s
        // 921600 -> 921600 bits/s
        // 1382400 -> 1382400 bits/s
        let mut serial_usart1 = Serial::new(
            cx.device.USART1,
            (hc05_tx, hc05_rx),
            &mut afio.mapr,
            Config::default()
                .baudrate(460800.bps())
                .wordlength_8bits()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1)
                .parity_none(),
            &clocks,
        );
        serial_usart1.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (hc05_tx, hc05_rx) = serial_usart1.split();

        // Timer to read from ADS1256
        let mut read_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        read_timer.start(DYMH_106_REQ_FREQ.Hz::<1, 1>().into_duration()).unwrap();
        read_timer.listen(Event::Update);

        // Timer to send data via bluetooth
        let mut send_timer = robot_core::get_counter(cx.device.TIM1, &clocks);
        send_timer.start(SEND_TIMER_MILLIS_DELAY.millis()).unwrap();
        send_timer.listen(Event::Update);

        let (dymh_106_data_producer, dymh_106_data_consumer) = cx.local.dymh106_queue.split();
        (
            Shared { ads1256 },
            Local {
                read_timer,
                hc05_tx,
                hc05_rx,
                send_timer,
                dymh_106_data_producer,
                dymh_106_data_consumer,
            },
        )
    }

    //for PGA::Gain64
    trait DymhVal {
        //global max & min possible values from sensor
        //calculated based on possible Nema17 Speed which is calculated
        //by multiply DymhValue on 10
        const max_val: i32 = 80_000;
        const min_val: i32 = -80_000;

        const default_min: i32 = -18000;
        const default_max: i32 = -12000;

        fn apply_default_min_max(self) -> i32;
        fn dymh_norm_val(self) -> i32;
    }

    impl DymhVal for i32 {
        fn apply_default_min_max(self) -> i32 {
            if self <= Self::default_max && self >= Self::default_min {
                return 0;
            }

            if self < Self::default_min {
                return self + (Self::default_min * -1);
            }

            return self + (Self::default_max * -1);
        }

        fn dymh_norm_val(self) -> i32 {
            let default_applied = self.apply_default_min_max();
            if default_applied >= Self::max_val {
                return Self::max_val;
            } else if default_applied <= Self::min_val {
                return Self::min_val;
            }

            default_applied
        }
    }

    #[task(binds = TIM2, priority = 10, local = [ read_timer, dymh_106_data_producer, counter: u32 = 0, max: i32 = i32::MIN, min: i32 = i32::MAX, first_start: bool = true ], shared = [ads1256])]
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

        //INFO: Debug related
        // ================================
        let mut current: i32 = 0;
        if first_channel_res != DEFAULT_DYMH06_VALUE {
            *cx.local.max = (*cx.local.max).max(first_channel_res);
            *cx.local.min = (*cx.local.min).min(first_channel_res);
            current = first_channel_res;
        }

        if *cx.local.counter == DYMH_106_REQ_FREQ {
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

        cx.local
            .dymh_106_data_producer
            .enqueue(TensionData {
                t0: buffer[0],
                t1: buffer[1],
                t2: buffer[2],
                t3: buffer[3],
            }).map_err(|_| defmt::error!("fail to add dymh_106 data into queue, cause it is full"));
    }

    fn read_from_ads_into_buffer(ads1256: &mut Ads1256, channel: usize, buffer: &mut [i32; 4]) -> i32 {
        let (channel_0, channel_1) = match channel {
            0 => (Channel::AIN0, Channel::AIN1),
            1 => (Channel::AIN2, Channel::AIN3),
            2 => (Channel::AIN4, Channel::AIN5),
            3 => (Channel::AIN6, Channel::AIN7),
            _ => panic!("wrong channel"),
        };

        if let Ok(result) = ads1256.read_channel(channel_0, channel_1) {
            buffer[channel] = result.dymh_norm_val();
            return result;
        } else {
            defmt::error!("fail to read from ads1256 channel: {}", channel);
            return DEFAULT_DYMH06_VALUE;
        }
    }

    #[task(binds = TIM1_UP, priority = 15, local = [ hc05_tx, send_timer, dymh_106_data_consumer, prev_tension_data: TensionData = TensionData { t0: 0, t1: 0, t2: 0, t3: 0 } ])]
    fn hc05_send(mut cx: hc05_send::Context) {
        cx.local.send_timer.clear_interrupt(Event::Update);

        let elems_count = cx.local.dymh_106_data_consumer.len() as i32;
        let mut t0: i32 = 0;
        let mut t1: i32 = 0;
        let mut t2: i32 = 0;
        let mut t3: i32 = 0;

        // let mut x: [i32; 10] = [0; 10];
        // let mut i: usize = 0;
        while let Some(data) = cx.local.dymh_106_data_consumer.dequeue() {
            // x[i] = data.t0;
            // i += 1;

            t0 += data.t0;
            t1 += data.t1;
            t2 += data.t2;
            t3 += data.t3;
        }

        let curr_tension_data = TensionData {
            t0: t0 / elems_count,
            t1: t1 / elems_count,
            t2: t2 / elems_count,
            t3: t3 / elems_count,
        };

        // if curr_tension_data.t0 != 0 {
        //     defmt::debug!("elems: {:?}", x);
        // }

        let prev_tension_data: &mut TensionData = cx.local.prev_tension_data;
        let mut tensor_diff = robot_core::i32_diff(curr_tension_data.t0, prev_tension_data.t0);
        tensor_diff = i32::max(tensor_diff, robot_core::i32_diff(curr_tension_data.t1, prev_tension_data.t1));
        tensor_diff = i32::max(tensor_diff, robot_core::i32_diff(curr_tension_data.t2, prev_tension_data.t2));
        tensor_diff = i32::max(tensor_diff, robot_core::i32_diff(curr_tension_data.t3, prev_tension_data.t3));

        *prev_tension_data = curr_tension_data.clone();

        //lets try to send data only if sensor update data at least for X
        if tensor_diff >= 500 {
            // defmt::debug!(
            //     "sending data, max diff: {}, curr: t0 = {}, t1 = {}, t2 = {}, t3 = {}",
            //     tensor_diff,
            //     curr_tension_data.t0,
            //     curr_tension_data.t1,
            //     curr_tension_data.t2,
            //     curr_tension_data.t3
            // );

            let mut data_to_send: [u8; 17] = [0; 17];
            data_to_send[0] = robot_core::COMPANION_SYNC;
            write_sensor_data(&mut data_to_send, curr_tension_data.t0.to_be_bytes(), 1);
            write_sensor_data(&mut data_to_send, curr_tension_data.t1.to_be_bytes(), 5);
            write_sensor_data(&mut data_to_send, curr_tension_data.t2.to_be_bytes(), 9);
            write_sensor_data(&mut data_to_send, curr_tension_data.t3.to_be_bytes(), 13);

            let mut tx = cx.local.hc05_tx;
            tx.bwrite_all(&data_to_send);
        }
    }

    fn write_sensor_data(data_to_send: &mut [u8; 17], sensor_data: [u8; 4], start_index: usize) {
        for i in 0..4 {
            data_to_send[start_index + i] = sensor_data[i];
        }
    }

    #[task(binds = USART1, priority = 5, local = [ hc05_rx ], shared = [ ads1256 ])]
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
