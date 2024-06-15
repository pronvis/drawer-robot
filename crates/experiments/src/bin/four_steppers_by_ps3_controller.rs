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
    use robot_core::display::OledDisplay;
    use robot_core::my_stepper::*;
    use robot_core::DisplayMemoryPool;
    use robot_core::*;

    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f1xx_hal::pac::interrupt;
    use stm32f1xx_hal::{
        afio,
        gpio::PinState,
        gpio::{Alternate, OpenDrain, Pin},
        pac,
        pac::I2C1,
        prelude::*,
        rcc,
        rcc::*,
        serial::{Config, Serial},
        timer::Timer,
        timer::{Counter, Event},
    };

    use heapless::{
        pool,
        pool::singleton::{Box, Pool},
        String,
    };

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const SOFT_CLOCK_FREQ: u32 = 115_200; // this doesn't mean 115_200 baudrate
    static mut DISPLAY_MEMORY_POOL_MEMORY: [u8; 96] = [32u8; 96];
    const DISPLAY_CHANNEL_CAPACITY: usize = robot_core::display::CHANNEL_CAPACITY;
    const PS3_CHANNEL_CAPACITY: usize = robot_core::ps3::CHANNEL_CAPACITY;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx_usart1: stm32f1xx_hal::serial::Tx1,
        rx_usart1: stm32f1xx_hal::serial::Rx1,
        tx_usart2: stm32f1xx_hal::serial::Tx2,
        rx_usart2: stm32f1xx_hal::serial::Rx2,
        stepper_1: MyStepper1,
        stepper_2: MyStepper2,
        stepper_3: MyStepper3,
        stepper_4: MyStepper4,
        display_receiver: Receiver<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY>,
        display: OledDisplay,
        ps3_bytes_sender: Sender<'static, u8, PS3_CHANNEL_CAPACITY>,
        ps3_reader: ps3::Ps3Reader,
        robot: legacy_robot::Robot,
        cr: stm32f1xx_hal::gpio::Cr<'C', true>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        unsafe {
            DisplayMemoryPool::grow(&mut DISPLAY_MEMORY_POOL_MEMORY);
        }

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash: stm32f1xx_hal::flash::Parts = cx.device.FLASH.constrain();
        let mut rcc: stm32f1xx_hal::rcc::Rcc = cx.device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
        // `clocks`
        // with those values one second equals 'delay_cycles: u32 = 72_000_000'
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values is wrong!");
        }

        let mut gpioa: stm32f1xx_hal::gpio::gpioa::Parts = cx.device.GPIOA.split();
        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();
        let mut gpiod: stm32f1xx_hal::gpio::gpiod::Parts = cx.device.GPIOD.split();
        let mut afio = cx.device.AFIO.constrain();

        let mut x_stepper_uart_pin = gpioc.pc10.into_dynamic(&mut gpioc.crh);
        let mut y_stepper_uart_pin = gpioc.pc11.into_push_pull_output(&mut gpioc.crh);
        let mut z_stepper_uart_pin = gpioc.pc12.into_push_pull_output(&mut gpioc.crh);
        let mut e_stepper_uart_pin = gpiod.pd2.into_push_pull_output(&mut gpiod.crl);

        // SSD1306 display pins
        let scl: Scl1Pin = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda: Sda1Pin = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        //init ssd1306 display
        let mut display = OledDisplay::new(scl, sda, &mut afio, clocks.clone(), cx.device.I2C1);

        // UART1 (bluetooth HC-05)
        let tx_usart1 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx_usart1 = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
        let mut serial_usart1 = Serial::new(
            cx.device.USART1,
            (tx_usart1, rx_usart1),
            &mut afio.mapr,
            Config::default()
                .baudrate(9600.bps())
                .wordlength_8bits()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1)
                .parity_none(),
            &clocks,
        );
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

        let (mut display_sender, display_receiver) = make_channel!(Box<DisplayMemoryPool>, DISPLAY_CHANNEL_CAPACITY);

        let ((stepper_1, stepper_1_sender), (stepper_2, stepper_2_sender), (stepper_3, stepper_3_sender), (stepper_4, stepper_4_sender)) =
            create_steppers(
                &clocks,
                gpioc.pc5,
                gpioc.pc6,
                gpioc.pc7,
                gpiob.pb0,
                gpiob.pb1,
                gpiob.pb2,
                gpiob.pb10,
                gpiob.pb11,
                gpiob.pb12,
                gpiob.pb13,
                gpiob.pb14,
                gpiob.pb15,
                &mut gpioc.crl,
                &mut gpiob.crl,
                &mut gpiob.crh,
                cx.device.TIM1,
                cx.device.TIM2,
                cx.device.TIM3,
                cx.device.TIM4,
                display_sender.clone(),
            );

        let (mut ps3_bytes_sender, ps3_bytes_receiver) = make_channel!(u8, PS3_CHANNEL_CAPACITY);
        let (mut ps3_commands_sender, ps3_commands_receiver) = make_channel!(ps3::Ps3Command, PS3_CHANNEL_CAPACITY);
        let ps3_reader = ps3::Ps3Reader::new(ps3_bytes_receiver, ps3_commands_sender);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        serial_usart1.listen(stm32f1xx_hal::serial::Event::Rxne);
        serial_usart2.listen(stm32f1xx_hal::serial::Event::Rxne);
        let (tx_usart1, rx_usart1) = serial_usart1.split();
        let (tx_usart2, rx_usart2) = serial_usart2.split();

        let robot = legacy_robot::Robot::new(
            stepper_1_sender,
            stepper_2_sender,
            stepper_3_sender,
            stepper_4_sender,
            ps3_commands_receiver,
            display_sender.clone(),
        );

        // display_task_writer::spawn().unwrap();
        ps3_reader_task::spawn().unwrap();
        robot_task::spawn().unwrap();

        (
            Shared {},
            Local {
                rx_usart1,
                tx_usart1,
                rx_usart2,
                tx_usart2,
                stepper_1,
                stepper_2,
                stepper_3,
                stepper_4,
                display_receiver,
                display_sender,
                display,
                ps3_bytes_sender,
                ps3_reader,
                robot,
                cr: gpioc.crh,
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

    #[task(binds = USART2, priority = 12, local = [ speed: u8 = 0, rx_usart2, tx_usart2, ps3_bytes_sender ])]
    fn esp32_reader(cx: esp32_reader::Context) {
        let rx = cx.local.rx_usart2;
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

    #[task(binds = USART1, priority = 8, local = [  rx_usart1, tx_usart1  ])]
    fn bluetooth_reader(cx: bluetooth_reader::Context) {
        let rx = cx.local.rx_usart1;
        if rx.is_rx_not_empty() {
            let received = rx.read();
            match received {
                Ok(read) => defmt::debug!("data via bluetooth: {}", read),

                Err(err) => {
                    defmt::debug!("data via bluetooth read err: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }

    #[task(binds = TIM2, priority = 10, local = [ stepper_1 ])]
    fn delay_task_1(cx: delay_task_1::Context) {
        cx.local.stepper_1.work();
    }

    #[task(binds = TIM3, priority = 10, local = [ stepper_2 ])]
    fn delay_task_2(cx: delay_task_2::Context) {
        cx.local.stepper_2.work();
    }

    #[task(binds = TIM4, priority = 10, local = [ stepper_3 ])]
    fn delay_task_3(cx: delay_task_3::Context) {
        cx.local.stepper_3.work();
    }

    #[task(binds = TIM1_UP, priority = 10, local = [ stepper_4 ])]
    fn delay_task_4(cx: delay_task_4::Context) {
        cx.local.stepper_4.work();
    }

    use core::fmt::Write;
    #[task(priority = 2, local = [display_sender])]
    async fn display_task_writer(mut cx: display_task_writer::Context) {
        let mut index: u32 = 0;
        loop {
            let mut data_str = DisplayString::new();
            write!(data_str, "Hello world: {index}").expect("not written");
            display::display_str(data_str, &mut cx.local.display_sender).await.unwrap();

            Systick::delay(500.millis()).await;
            index += 1;
        }
    }
}
