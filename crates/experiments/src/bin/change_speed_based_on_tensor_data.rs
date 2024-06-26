#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(slice_as_chunks)]

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
    // dispatchers = [PVD, WWDG, RTC, SPI1]
)]
mod app {

    use defmt_brtt as _; // global logger
    use fugit::Duration;
    use robot_core::my_tmc2209::communicator::TMC2209SerialCommunicator;
    use robot_core::my_tmc2209::configurator::TMC2209Configurator;
    use robot_core::my_tmc2209::Tmc2209Constructor;
    use robot_core::my_tmc2209::{
        Tmc2209RequestCh, Tmc2209RequestProducer, Tmc2209ResponseCh, Tmc2209ResponseConsumer, TMC2209COMMUNICATOR_CLOCK_FREQ,
    };
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f1xx_hal::{
        gpio::PinState,
        pac,
        prelude::*,
        rcc,
        rcc::*,
        serial::{Config, Serial},
        timer::Timer,
        timer::{Counter, Event},
    };

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const COMMUNICATOR_CHANNEL_CAPACITY: usize = robot_core::my_tmc2209::communicator::CHANNEL_CAPACITY;
    const BIT_SEND_TICKS: u32 = 40;
    const ADS1256_DEFAULT_VALUE: i32 = 0;

    #[shared]
    struct Shared {
        tensor_0_val: i32,
    }

    #[local]
    struct Local {
        configurator: TMC2209Configurator,
        tmc2209_communicator_timer: Counter<stm32f1xx_hal::pac::TIM2, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        tmc2209_msg_sender: Tmc2209RequestProducer,
        communicator: TMC2209SerialCommunicator<'C', 10>,
        hc05_rx: stm32f1xx_hal::serial::Rx1,
    }

    #[init(local = [
           tmc2209_req_ch_0: Tmc2209RequestCh = Channel::new(),
           tmc2209_rsp_ch_0: Tmc2209ResponseCh = Channel::new(),
    ])]
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

        let mut gpioa: stm32f1xx_hal::gpio::gpioa::Parts = cx.device.GPIOA.split();
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();

        // USART1 for HC-05
        let hc05_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let hc05_rx = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

        let mut afio = cx.device.AFIO.constrain();
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
        let (_, hc05_rx) = serial_usart1.split();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        // TMC2209
        let en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        let x_stepper_uart_pin = gpioc.pc10.into_dynamic(&mut gpioc.crh);
        let tmc2209_0 = Tmc2209Constructor::new(
            en,
            x_stepper_uart_pin,
            cx.device.TIM2,
            &clocks,
            cx.local.tmc2209_req_ch_0,
            cx.local.tmc2209_rsp_ch_0,
        );

        stepper_conf_task::spawn().ok();
        stepper_change_speed_task::spawn().ok();

        (
            Shared {
                tensor_0_val: ADS1256_DEFAULT_VALUE,
            },
            Local {
                configurator: tmc2209_0.configurator,
                tmc2209_communicator_timer: tmc2209_0.timer,
                tmc2209_msg_sender: tmc2209_0.req_sender,
                communicator: tmc2209_0.communicator,
                hc05_rx,
            },
        )
    }

    #[task(binds = USART1, priority = 9, shared = [ tensor_0_val ], local = [ hc05_rx, data_to_receive: [u8; 16] = [0; 16], counter: usize = 0 ])]
    fn bluetooth_reader(mut cx: bluetooth_reader::Context) {
        let rx = cx.local.hc05_rx;
        let mut counter = cx.local.counter;
        if rx.is_rx_not_empty() {
            let received = rx.read();
            match received {
                Ok(read) => {
                    if read == robot_core::COMPANION_SYNC {
                        if *counter != 0 {
                            defmt::debug!("receive SYNC byte when counter = {}", *counter);
                        }
                        *counter = 0;
                    } else {
                        cx.local.data_to_receive[*counter] = read;
                        *counter += 1;
                    }

                    if *counter == 16 {
                        *counter = 0;
                        let splitted = cx.local.data_to_receive[0..4].as_chunks::<4>().0;
                        let bytes: [u8; 4] = splitted[0];
                        let sensor_0_data = i32::from_be_bytes(bytes);

                        // let kilogramms = robot_core::tensor_to_kg(sensor_0_data);
                        // let kilogramms = sensor_0_data as f32 * 0.025f32;
                        let kilogramms = sensor_0_data;
                        if kilogramms != robot_core::DEFAULT_DYMH06_VALUE {
                            defmt::debug!("sensor 0 data: {}", kilogramms);
                            cx.shared.tensor_0_val.lock(|tval| *tval = kilogramms);
                        }
                    }
                }

                Err(err) => {
                    defmt::debug!("read err: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }

    #[task(priority = 1, shared = [ tensor_0_val ], local = [tmc2209_msg_sender, curr_tensor_0_val: i32 = ADS1256_DEFAULT_VALUE ])]
    async fn stepper_change_speed_task(mut cx: stepper_change_speed_task::Context) {
        //wait for Conf task
        Systick::delay(1.secs()).await;

        loop {
            let tensor_0_val: i32 = cx.shared.tensor_0_val.lock(|tval| *tval);
            // defmt::debug!("change speed: {}", tensor_0_val);
            let curr_tensor_0_val = *cx.local.curr_tensor_0_val;
            let tensor_diff = robot_core::i32_diff(curr_tensor_0_val, tensor_0_val);

            let speed: u32 = robot_core::tension_to_speed(tensor_0_val);

            let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(speed));
            let req = robot_core::my_tmc2209::Request::write(write_req);
            cx.local.tmc2209_msg_sender.send(req).await.ok();
        }
    }

    #[task(binds = TIM2, priority = 10, local = [communicator, tmc2209_communicator_timer])]
    fn communicator_task(cx: communicator_task::Context) {
        cx.local.tmc2209_communicator_timer.clear_interrupt(Event::Update);
        cx.local.communicator.handle_interrupt();
    }

    #[task(priority = 1, local = [configurator])]
    async fn stepper_conf_task(cx: stepper_conf_task::Context) {
        let setup_res = cx.local.configurator.setup().await;
        match setup_res {
            Ok(_) => {
                defmt::debug!("Stepper driver configured successfully!");
            }
            Err(_) => {
                panic!("Fail to configure stepper driver");
            }
        }
    }
}
