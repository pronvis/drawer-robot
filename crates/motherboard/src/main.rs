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

    use defmt_brtt as _; // global logger
    use fugit::Duration;
    use heapless::{
        pool,
        pool::singleton::{Box, Pool},
        String,
    };
    use robot::Robot;
    use robot_core::display::OledDisplay;
    use robot_core::my_tmc2209::communicator::TMC2209SerialCommunicator;
    use robot_core::DisplayMemoryPool;
    use robot_core::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f1xx_hal::{
        afio,
        gpio::PinState,
        gpio::{Alternate, OpenDrain, Pin},
        pac,
        pac::interrupt,
        pac::I2C1,
        prelude::*,
        rcc,
        rcc::*,
        serial::{Config, Serial},
        timer::Timer,
        timer::{Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 72_000_000;
    const TMC2209COMMUNICATOR_CLOCK_FREQ: u32 = 72_0_000;
    const TMC2209_BIT_SEND_TICKS: u32 = 40;

    const COMMUNICATOR_CHANNEL_CAPACITY: usize = robot_core::my_tmc2209::communicator::CHANNEL_CAPACITY;
    const PS3_CHANNEL_CAPACITY: usize = robot_core::ps3::CHANNEL_CAPACITY;
    const DISPLAY_CHANNEL_CAPACITY: usize = robot_core::display::CHANNEL_CAPACITY;

    type Tmc2209Request = robot_core::my_tmc2209::Request;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        ps3_reader: ps3::Ps3Reader,
        ps3_bytes_sender: Sender<'static, u8, PS3_CHANNEL_CAPACITY>,
        ps3_usart_rx: stm32f1xx_hal::serial::Rx2,
        display_receiver: Receiver<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
        display: OledDisplay,
        configurator: robot_core::my_tmc2209::configurator::Configurator,
        tmc2209_rsp_receiver: Receiver<'static, u32, COMMUNICATOR_CHANNEL_CAPACITY>,
        tmc2209_communicator_timer: Counter<stm32f1xx_hal::pac::TIM2, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        communicator: TMC2209SerialCommunicator<'C', 10>,
        robot: Robot,
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
        let mut afio = cx.device.AFIO.constrain();

        // STEPPER 0
        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();
        let x_stepper_uart_pin = gpioc.pc10.into_dynamic(&mut gpioc.crh);

        let mut tmc2209_communicator_timer = robot_core::get_counter(cx.device.TIM2, &clocks);
        let tmc2209_timer_ticks = Duration::<u32, 1, TMC2209COMMUNICATOR_CLOCK_FREQ>::from_ticks(TMC2209_BIT_SEND_TICKS);
        tmc2209_communicator_timer.start(tmc2209_timer_ticks).unwrap();
        defmt::debug!("tmc2209_communicator_timer period: {} nanos", tmc2209_timer_ticks.to_nanos());
        tmc2209_communicator_timer.listen(Event::Update);

        let (tmc2209_msg_sender, tmc2209_msg_receiver) = make_channel!(Tmc2209Request, COMMUNICATOR_CHANNEL_CAPACITY);
        let (tmc2209_rsp_sender, tmc2209_rsp_receiver) = make_channel!(u32, COMMUNICATOR_CHANNEL_CAPACITY);
        let communicator = TMC2209SerialCommunicator::new(tmc2209_msg_receiver, tmc2209_rsp_sender, x_stepper_uart_pin, gpioc.crh);
        let configurator = robot_core::my_tmc2209::configurator::Configurator::new(tmc2209_msg_sender.clone());

        // UART2 (esp32)
        let tx_usart2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx_usart2 = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);
        let mut serial_usart2 = Serial::new(
            cx.device.USART2,
            (tx_usart2, rx_usart2),
            &mut afio.mapr,
            Config::default()
                .baudrate(57600.bps())
                .wordlength_8bits()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1)
                .parity_none(),
            &clocks,
        );
        serial_usart2.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (_, ps3_usart_rx) = serial_usart2.split();

        let (mut ps3_bytes_sender, ps3_bytes_receiver) = make_channel!(u8, PS3_CHANNEL_CAPACITY);
        let (mut ps3_commands_sender, ps3_commands_receiver) = make_channel!(ps3::Ps3Command, PS3_CHANNEL_CAPACITY);
        let ps3_reader = ps3::Ps3Reader::new(ps3_bytes_receiver, ps3_commands_sender);

        // SSD1306 display pins
        let scl: Scl1Pin = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda: Sda1Pin = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        let display = OledDisplay::new(scl, sda, &mut afio, clocks.clone(), cx.device.I2C1);
        let (mut display_sender, display_receiver) = make_channel!(Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY);

        // ROBOT
        let robot = Robot::new(tmc2209_msg_sender, ps3_commands_receiver, display_sender);

        stepper_conf_task::spawn().ok();
        ps3_reader_task::spawn().unwrap();
        robot_task::spawn().unwrap();

        (
            Shared {},
            Local {
                ps3_usart_rx,
                ps3_bytes_sender,
                display_receiver,
                display,
                tmc2209_rsp_receiver,
                configurator,
                ps3_reader,
                communicator,
                tmc2209_communicator_timer,
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

    #[task( priority = 4, local = [  ps3_reader  ])]
    async fn ps3_reader_task(mut cx: ps3_reader_task::Context) {
        loop {
            cx.local.ps3_reader.work().await;
        }
    }

    #[task( priority = 5, local = [robot])]
    async fn robot_task(mut cx: robot_task::Context) {
        loop {
            cx.local.robot.work().await;
        }
    }

    #[task(binds = USART2, priority = 10, local = [ ps3_usart_rx, ps3_bytes_sender ])]
    fn esp32_reader(cx: esp32_reader::Context) {
        let rx = cx.local.ps3_usart_rx;
        if rx.is_rx_not_empty() {
            let received = rx.read();
            match received {
                Ok(read) => {
                    cx.local.ps3_bytes_sender.try_send(read).err().map(|err| {
                        defmt::debug!("fail to send bytes to ps3_reader: {:?}", defmt::Debug2Format(&err));
                    });
                }

                Err(err) => {
                    defmt::debug!("data from esp32 read err: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }

    #[task(binds = TIM2, priority = 10, local = [communicator, tmc2209_communicator_timer])]
    fn communicator_task(cx: communicator_task::Context) {
        cx.local.tmc2209_communicator_timer.clear_interrupt(Event::Update);
        cx.local.communicator.handle_interrupt();
    }
}
