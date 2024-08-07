#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(slice_as_chunks)]

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
)]
mod app {

    use defmt_brtt as _; // global logger
    use heapless::pool::singleton::{Box, Pool};
    use robot::Robot;
    use robot_core::display::OledDisplay;
    use robot_core::my_tmc2209::{
        communicator::TMC2209SerialCommunicator, configurator::TMC2209Configurator, Tmc2209Constructor, Tmc2209RequestCh,
        Tmc2209ResponseCh, TMC2209COMMUNICATOR_CLOCK_FREQ,
    };
    use robot_core::robot::{TensionData, TENSION_DATA_CHANNEL_CAPACITY};
    use robot_core::DisplayMemoryPool;
    use robot_core::*;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f1xx_hal::{
        prelude::*,
        serial::{Config, Serial},
        timer::{Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 72_000_000;
    static mut DISPLAY_MEMORY_POOL_MEMORY: [u8; 96] = [32u8; 96];

    const PS3_CHANNEL_CAPACITY: usize = robot_core::ps3::CHANNEL_CAPACITY;
    const DISPLAY_CHANNEL_CAPACITY: usize = robot_core::display::CHANNEL_CAPACITY;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        ps3_reader: ps3::Ps3Reader,
        ps3_bytes_sender: Sender<'static, u8, PS3_CHANNEL_CAPACITY>,
        tension_data_sender: Sender<'static, TensionData, TENSION_DATA_CHANNEL_CAPACITY>,
        ps3_rx: stm32f1xx_hal::serial::Rx2,
        hc05_rx: stm32f1xx_hal::serial::Rx1,
        display_receiver: Receiver<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
        display: OledDisplay,

        //TMC2209 related
        configurator_0: TMC2209Configurator,
        tmc2209_communicator_timer_0: Counter<stm32f1xx_hal::pac::TIM2, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        communicator_0: TMC2209SerialCommunicator<'C', 10>,

        configurator_1: TMC2209Configurator,
        tmc2209_communicator_timer_1: Counter<stm32f1xx_hal::pac::TIM3, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        communicator_1: TMC2209SerialCommunicator<'C', 11>,

        tmc2209_communicator_timer_2: Counter<stm32f1xx_hal::pac::TIM4, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        configurator_2: TMC2209Configurator,
        communicator_2: TMC2209SerialCommunicator<'C', 12>,

        tmc2209_communicator_timer_3: Counter<stm32f1xx_hal::pac::TIM5, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        configurator_3: TMC2209Configurator,
        communicator_3: TMC2209SerialCommunicator<'D', 2>,
        robot: Robot,
    }

    #[init(local = [
           tmc2209_req_ch_0: Tmc2209RequestCh = Channel::new(),
           tmc2209_rsp_ch_0: Tmc2209ResponseCh = Channel::new(),

           tmc2209_req_ch_1: Tmc2209RequestCh = Channel::new(),
           tmc2209_rsp_ch_1: Tmc2209ResponseCh = Channel::new(),

           tmc2209_req_ch_2: Tmc2209RequestCh = Channel::new(),
           tmc2209_rsp_ch_2: Tmc2209ResponseCh = Channel::new(),

           tmc2209_req_ch_3: Tmc2209RequestCh = Channel::new(),
           tmc2209_rsp_ch_3: Tmc2209ResponseCh = Channel::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        unsafe {
            DisplayMemoryPool::grow(&mut DISPLAY_MEMORY_POOL_MEMORY);
        }

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
            .sysclk(TIMER_CLOCK_FREQ.Hz())
            .pclk1(36.MHz())
            .pclk2(TIMER_CLOCK_FREQ.Hz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        let mut gpioa: stm32f1xx_hal::gpio::gpioa::Parts = cx.device.GPIOA.split();
        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();
        let mut gpiod: stm32f1xx_hal::gpio::gpiod::Parts = cx.device.GPIOD.split();
        let mut afio = cx.device.AFIO.constrain();

        // STEPPER 0
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

        // // STEPPER 1
        let en = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        let y_stepper_uart_pin = gpioc.pc11.into_dynamic(&mut gpioc.crh);
        let tmc2209_1 = Tmc2209Constructor::new(
            en,
            y_stepper_uart_pin,
            cx.device.TIM3,
            &clocks,
            cx.local.tmc2209_req_ch_1,
            cx.local.tmc2209_rsp_ch_1,
        );

        // // STEPPER 2
        let en = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
        let z_stepper_uart_pin = gpioc.pc12.into_dynamic(&mut gpioc.crh);
        let tmc2209_2 = Tmc2209Constructor::new(
            en,
            z_stepper_uart_pin,
            cx.device.TIM4,
            &clocks,
            cx.local.tmc2209_req_ch_2,
            cx.local.tmc2209_rsp_ch_2,
        );

        // // STEPPER 3
        let en = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        let e_stepper_uart_pin = gpiod.pd2.into_dynamic(&mut gpiod.crl);
        let tmc2209_3 = Tmc2209Constructor::new(
            en,
            e_stepper_uart_pin,
            cx.device.TIM5,
            &clocks,
            cx.local.tmc2209_req_ch_3,
            cx.local.tmc2209_rsp_ch_3,
        );

        // USART1 for HC-05
        let hc05_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let hc05_rx = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

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

        // USART2 (esp32)
        let ps3_tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let ps3_rx = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);
        let mut serial_usart2 = Serial::new(
            cx.device.USART2,
            (ps3_tx, ps3_rx),
            &mut afio.mapr,
            Config::default()
                .baudrate(57600.bps())
                .wordlength_8bits()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1)
                .parity_none(),
            &clocks,
        );
        serial_usart2.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (_, ps3_rx) = serial_usart2.split();

        let (ps3_bytes_sender, ps3_bytes_receiver) = make_channel!(u8, PS3_CHANNEL_CAPACITY);
        let (ps3_commands_sender, ps3_commands_receiver) = make_channel!(ps3::Ps3Command, PS3_CHANNEL_CAPACITY);
        let ps3_reader = ps3::Ps3Reader::new(ps3_bytes_receiver, ps3_commands_sender);

        // SSD1306 display pins
        let scl: Scl1Pin = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda: Sda1Pin = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        let display = OledDisplay::new(scl, sda, &mut afio, clocks.clone(), cx.device.I2C1);
        let (display_sender, display_receiver) = make_channel!(Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY);

        // ROBOT
        let (tension_data_sender, tension_data_receiver) = make_channel!(TensionData, TENSION_DATA_CHANNEL_CAPACITY);
        let robot = Robot::new(
            tmc2209_0.req_sender,
            tmc2209_1.req_sender,
            tmc2209_2.req_sender,
            tmc2209_3.req_sender,
            ps3_commands_receiver,
            tension_data_receiver,
            hc05_tx,
            display_sender.clone(),
        );

        steppers_conf_task::spawn().ok();
        ps3_reader_task::spawn().unwrap();
        robot_task::spawn().unwrap();

        (
            Shared {},
            Local {
                ps3_rx,
                hc05_rx,
                ps3_bytes_sender,
                ps3_reader,
                tension_data_sender,
                display_receiver,
                display_sender,
                display,

                //TMC2209 related
                configurator_3: tmc2209_3.configurator,
                communicator_3: tmc2209_3.communicator,
                tmc2209_communicator_timer_3: tmc2209_3.timer,

                configurator_2: tmc2209_2.configurator,
                communicator_2: tmc2209_2.communicator,
                tmc2209_communicator_timer_2: tmc2209_2.timer,

                configurator_1: tmc2209_1.configurator,
                communicator_1: tmc2209_1.communicator,
                tmc2209_communicator_timer_1: tmc2209_1.timer,

                configurator_0: tmc2209_0.configurator,
                communicator_0: tmc2209_0.communicator,
                tmc2209_communicator_timer_0: tmc2209_0.timer,
                robot,
            },
        )
    }

    #[idle(local = [display_receiver, display])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            if let Ok(message) = cx.local.display_receiver.try_recv() {
                cx.local.display.print(message);
            }
        }
    }

    #[task(priority = 9, local = [ configurator_0, configurator_1, configurator_2, configurator_3 ])]
    async fn steppers_conf_task(cx: steppers_conf_task::Context) {
        debug_configurator_res(0, cx.local.configurator_0.setup().await);
        debug_configurator_res(1, cx.local.configurator_1.setup().await);
        debug_configurator_res(2, cx.local.configurator_2.setup().await);
        debug_configurator_res(3, cx.local.configurator_3.setup().await);
    }

    fn debug_configurator_res(i: u8, result: Result<(), ()>) {
        match result {
            Ok(_) => {
                defmt::debug!("Stepper #{}: driver configured successfully!", i);
            }
            Err(_) => {
                panic!("Stepper #{}: fail to configure driver!", i);
            }
        }
    }

    #[task( priority = 4, local = [  ps3_reader  ])]
    async fn ps3_reader_task(cx: ps3_reader_task::Context) {
        loop {
            cx.local.ps3_reader.work().await;
        }
    }

    #[task( priority = 1, local = [robot])]
    async fn robot_task(cx: robot_task::Context) {
        loop {
            cx.local.robot.work().await;
        }
    }

    #[task(binds = USART1, priority = 15, local = [ tension_data_sender, hc05_rx, data_to_receive: [u8; 16] = [0; 16], counter: usize = 0 ])]
    fn hc05_reader(cx: hc05_reader::Context) {
        let rx = cx.local.hc05_rx;
        let counter = cx.local.counter;
        if rx.is_rx_not_empty() {
            let received = rx.read();
            match received {
                Ok(read) => {
                    if read == robot_core::COMPANION_SYNC {
                        if *counter != 0 {
                            defmt::warn!("receive SYNC byte when counter = {}", *counter);
                        }
                        *counter = 0;
                    } else {
                        cx.local.data_to_receive[*counter] = read;
                        *counter += 1;
                    }

                    if *counter == 16 {
                        *counter = 0;
                        let splitted = cx.local.data_to_receive[0..16].as_chunks::<4>().0;
                        let tension_data = TensionData {
                            t0: i32::from_be_bytes(splitted[0]),
                            t1: i32::from_be_bytes(splitted[1]),
                            t2: i32::from_be_bytes(splitted[2]),
                            t3: i32::from_be_bytes(splitted[3]),
                        };

                        defmt::debug!(
                            "get tension data: {}:{}:{}:{}",
                            tension_data.t0,
                            tension_data.t1,
                            tension_data.t2,
                            tension_data.t3
                        );
                        cx.local.tension_data_sender.try_send(tension_data).err().map(|err| {
                            defmt::error!("fail to send tension_data to tension_data_sender: {:?}", defmt::Debug2Format(&err));
                        });
                    }
                }

                Err(err) => {
                    defmt::error!("read err: {:?}", defmt::Debug2Format(&err));
                    rx.clear_idle_interrupt();
                }
            }
        }
    }

    #[task(binds = USART2, priority = 9, local = [ ps3_rx, ps3_bytes_sender ])]
    fn esp32_reader(cx: esp32_reader::Context) {
        let rx = cx.local.ps3_rx;
        if rx.is_rx_not_empty() {
            let received = rx.read();
            match received {
                Ok(read) => {
                    cx.local.ps3_bytes_sender.try_send(read).err().map(|err| {
                        defmt::debug!("fail to send bytes to ps3_reader: {:?}", defmt::Debug2Format(&err));
                    });
                }

                Err(err) => {
                    defmt::trace!("data from esp32 read err: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }

    #[task(binds = TIM2, priority = 10, local = [communicator_0, tmc2209_communicator_timer_0])]
    fn communicator_task_0(cx: communicator_task_0::Context) {
        cx.local.tmc2209_communicator_timer_0.clear_interrupt(Event::Update);
        cx.local.communicator_0.handle_interrupt();
    }

    #[task(binds = TIM3, priority = 10, local = [communicator_1, tmc2209_communicator_timer_1])]
    fn communicator_task_1(cx: communicator_task_1::Context) {
        cx.local.tmc2209_communicator_timer_1.clear_interrupt(Event::Update);
        cx.local.communicator_1.handle_interrupt();
    }

    #[task(binds = TIM4, priority = 10, local = [communicator_2, tmc2209_communicator_timer_2])]
    fn communicator_task_2(cx: communicator_task_2::Context) {
        cx.local.tmc2209_communicator_timer_2.clear_interrupt(Event::Update);
        cx.local.communicator_2.handle_interrupt();
    }

    #[task(binds = TIM5, priority = 10, local = [communicator_3, tmc2209_communicator_timer_3])]
    fn communicator_task_3(cx: communicator_task_3::Context) {
        cx.local.tmc2209_communicator_timer_3.clear_interrupt(Event::Update);
        cx.local.communicator_3.handle_interrupt();
    }
}
