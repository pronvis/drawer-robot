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
    use robot_core::my_stepper::*;
    use robot_core::my_tmc2209::communicator::TMC2209SerialCommunicator;
    use robot_core::*;
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
    const TMC2209COMMUNICATOR_CLOCK_FREQ: u32 = 72_0_000;
    const ADS1256_DEFAULT_VALUE: i32 = 0;

    #[shared]
    struct Shared {
        tensor_0_val: i32,
    }

    #[local]
    struct Local {
        configurator: robot_core::my_tmc2209::configurator::Configurator,
        tmc2209_communicator_timer: Counter<stm32f1xx_hal::pac::TIM2, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        tmc2209_msg_sender: Sender<'static, robot_core::my_tmc2209::Request, COMMUNICATOR_CHANNEL_CAPACITY>,
        tmc2209_rsp_receiver: Receiver<'static, u32, COMMUNICATOR_CHANNEL_CAPACITY>,
        communicator: TMC2209SerialCommunicator<'C', 10>,
        bluetooth_rx: stm32f1xx_hal::serial::Rx1,
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

        let mut gpioa: stm32f1xx_hal::gpio::gpioa::Parts = cx.device.GPIOA.split();
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();

        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();
        let x_stepper_uart_pin = gpioc.pc10.into_dynamic(&mut gpioc.crh);

        // USART1 for HC-05
        let tx_usart1 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx_usart1 = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

        let mut afio = cx.device.AFIO.constrain();
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

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        serial_usart1.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (_, mut bluetooth_rx) = serial_usart1.split();

        // TMC2209
        let mut tmc2209_communicator_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        let tmc2209_timer_ticks = Duration::<u32, 1, TMC2209COMMUNICATOR_CLOCK_FREQ>::from_ticks(BIT_SEND_TICKS);
        tmc2209_communicator_timer.start(tmc2209_timer_ticks).unwrap();
        defmt::debug!("tmc2209_communicator_timer period: {} nanos", tmc2209_timer_ticks.to_nanos());
        tmc2209_communicator_timer.listen(Event::Update);

        let (tmc2209_msg_sender, tmc2209_msg_receiver) = make_channel!(robot_core::my_tmc2209::Request, COMMUNICATOR_CHANNEL_CAPACITY);
        let (tmc2209_rsp_sender, tmc2209_rsp_receiver) = make_channel!(u32, COMMUNICATOR_CHANNEL_CAPACITY);

        let communicator = TMC2209SerialCommunicator::new(tmc2209_msg_receiver, tmc2209_rsp_sender, x_stepper_uart_pin, gpioc.crh);
        let configurator = robot_core::my_tmc2209::configurator::Configurator::new(tmc2209_msg_sender.clone());

        stepper_conf_task::spawn().ok();
        stepper_change_speed_task::spawn().ok();

        (
            Shared {
                tensor_0_val: ADS1256_DEFAULT_VALUE,
            },
            Local {
                configurator,
                tmc2209_communicator_timer,
                tmc2209_msg_sender,
                tmc2209_rsp_receiver,
                communicator,
                bluetooth_rx,
            },
        )
    }

    #[task(binds = USART1, priority = 9, shared = [ tensor_0_val ], local = [ bluetooth_rx, data_to_receive: [u8; 16] = [0; 16], counter: usize = 0 ])]
    fn bluetooth_reader(mut cx: bluetooth_reader::Context) {
        let rx = cx.local.bluetooth_rx;
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
                        if kilogramms != i32::MIN {
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

            let speed: u32 = tensor_0_val as u32 * 1_0;

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

    #[task(priority = 1, local = [configurator, tmc2209_rsp_receiver])]
    async fn stepper_conf_task(cx: stepper_conf_task::Context) {
        let setup_res = cx.local.configurator.setup(cx.local.tmc2209_rsp_receiver).await;
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
