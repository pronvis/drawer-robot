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
    use defmt_brtt as _;
    // global logger
    use hx711::Hx711;
    use robot_core::CompanionMessage;
    use robot_core::*;
    use stm32f1xx_hal::{
        gpio::{Output, Pin},
        prelude::*,
        serial::{Config, Serial},
        timer::{counter, Counter, Event},
    };

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const TIMER_CLOCK_FREQ: u32 = 72_000;
    const HX711_TIMER_CLOCK_FREQ: u32 = 2_000_000;

    #[shared]
    struct Shared {
        companion_message: CompanionMessage,
    }

    #[local]
    struct Local {
        tx_usart1: stm32f1xx_hal::serial::Tx1,
        send_timer: Counter<stm32f1xx_hal::pac::TIM1, TIMER_CLOCK_FREQ>,
        hx711: Hx711<stm32f1xx_hal::timer::Delay<stm32f1xx_hal::pac::TIM3, HX711_TIMER_CLOCK_FREQ>, Pin<'A', 6>, Pin<'A', 7, Output>>,
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
        let mut afio = cx.device.AFIO.constrain();

        // Configure the hx711 load cell driver:
        //
        // | HX  | dout   -> PA6 | STM |
        // | 711 | pd_sck <- PA7 | 32  |
        //
        let dout = gpioa.pa6.into_floating_input(&mut gpioa.crl);
        let pd_sck = gpioa.pa7.into_push_pull_output(&mut gpioa.crl);
        let timer = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM3, HX711_TIMER_CLOCK_FREQ>::new(cx.device.TIM3, &clocks);
        let hx711 = Hx711::new(timer.delay(), dout, pd_sck).unwrap();

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

        let mut read_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        read_timer.start(80.Hz::<1, 1>().into_duration()).unwrap();
        // read_timer.start(100.millis()).unwrap();
        read_timer.listen(Event::Update);

        let mut send_timer = robot_core::get_counter(cx.device.TIM1, &clocks);
        send_timer.start(30.millis()).unwrap();
        send_timer.listen(Event::Update);

        (
            Shared {
                companion_message: Default::default(),
            },
            Local {
                hx711,
                read_timer,
                tx_usart1,
                send_timer,
            },
        )
    }

    #[task(binds = TIM2, priority = 10, local = [ read_timer, hx711, counter: u8 = 0 ], shared = [companion_message])]
    fn hx711_read(mut cx: hx711_read::Context) {
        cx.local.read_timer.clear_interrupt(Event::Update);
        // This conversion works fine.
        // But for protocol simplicity I will send raw data.
        // let conversion_rate: f32 = 0.035274;
        // let gramms = data as f32 * conversion_rate;
        // let kilogramms = gramms / 1000f32;
        let mut buffer: [i32; 4] = [0, i32::MIN, i32::MIN, i32::MIN];
        //TODO: I have only one connected sensor for now
        for i in 0..1 {
            if let Ok(data) = cx.local.hx711.retrieve() {
                buffer[i] = data;
            } else {
                buffer[i] = i32::MIN;
                defmt::debug!("fail to read data from hx711 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ");
            }
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
        let tensor_diff = robot_core::f32_diff(
            robot_core::tensor_to_kg(curr_tensor_0_val),
            robot_core::tensor_to_kg(*prev_tensor_0_val),
        );

        if tensor_diff >= 1.0 {
            defmt::debug!("sending data, diff: {}, curr: {}", tensor_diff, curr_tensor_0_val);
            *prev_tensor_0_val = curr_tensor_0_val;

            let mut data_to_send: [u8; 17] = [0; 17];
            data_to_send[0] = robot_core::COMPANION_SYNC;
            //here I send only 0 sensor data, cause it is only one attached
            let sensor_0 = companion_message.load_sensor_0.to_be_bytes();
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
