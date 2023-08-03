#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use drawer_robot as _; // global logger + panicking-behavior + memory layout
use heapless::{
    pool,
    pool::singleton::{Box, Pool},
    String,
};

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
    // dispatchers = [PVD, WWDG, RTC, SPI1]
)]
mod app {

    use core::mem::MaybeUninit;
    use cortex_m::singleton;
    use drawer_robot::display::OledDisplay;
    use drawer_robot::my_stepper::*;
    use drawer_robot::*;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stepper::{
        compat, fugit::NanosDurationU32 as Nanoseconds, motion_control,
        motion_control::SoftwareMotionControl, ramp_maker, Direction, Stepper,
    };

    use stm32f1xx_hal::{
        afio, dma,
        dma::{dma1, RxDma, Transfer},
        gpio::PinState,
        gpio::{Alternate, OpenDrain, Pin},
        pac,
        pac::I2C1,
        prelude::*,
        rcc,
        rcc::*,
        serial::{Config, Rx, RxDma2, Serial},
        timer::Timer,
        timer::{Counter, Event},
    };

    const TIMER_CLOCK_FREQ: u32 = 10_000; // step = 100_000 nanos, 100 micros
    const STEPPER_CLOCK_FREQ: u32 = 72_000_000;
    // if `delay` is active in 'display_task_writer' function, then size of the pool could be 32
    // for some reason. Why?
    static mut DISPLAY_MEMORY_POOL_MEMORY: [u8; 96] = [32u8; 96];
    static mut PS3_BUF: [u8; 4] = [0u8; 4];

    // Import the memory pool into scope
    use drawer_robot::DisplayMemoryPool;
    use heapless::{
        pool,
        pool::singleton::{Box, Pool},
    };

    #[shared]
    struct Shared {
        rx_usart2: Option<Rx<pac::USART2>>,
        // transfer: Option<Transfer<dma::W, &'static mut [u8; 4], RxDma<>>>,
    }

    #[local]
    struct Local {
        rx_usart1: stm32f1xx_hal::serial::Rx1,
        stepper_1: MyStepper<stm32f1xx_hal::pac::TIM2, XStepPin, TIMER_CLOCK_FREQ>,
        stepper_1_sender: Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
        display_receiver: Receiver<'static, Box<DisplayMemoryPool>, CHANNEL_CAPACITY>,
        display_sender: Sender<'static, Box<DisplayMemoryPool>, CHANNEL_CAPACITY>,
        display: OledDisplay,
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
            panic!("Clock parameter values are wrong!");
        }

        let mut gpioa: stm32f1xx_hal::gpio::gpioa::Parts = cx.device.GPIOA.split();
        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();
        let mut afio = cx.device.AFIO.constrain();

        // SSD1306 display pins
        let scl: Scl1Pin = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda: Sda1Pin = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        //init ssd1306 display
        let mut display = OledDisplay::new(scl, sda, &mut afio, clocks.clone(), cx.device.I2C1);

        // UART1
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
        // UART2
        let mut dma1_channel_6: dma1::C6 = cx.device.DMA1.split().6;
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

        //
        //
        // STEPPERS
        let mut stepper_state = MyStepperState::new(1500, 200);

        let x_step_pin = gpioc
            .pc6
            .into_push_pull_output_with_state(&mut gpioc.crl, PinState::Low);
        let y_step_pin = gpiob
            .pb13
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low);
        let z_step_pin = gpiob
            .pb10
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low);
        let e_step_pin = gpiob
            .pb0
            .into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low);

        let mut x_en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        x_en.set_low();

        let mut y_en = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        y_en.set_low();

        let mut z_en = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
        z_en.set_low();

        let mut e_en = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        e_en.set_low();

        let tim2 = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM2,
            &clocks,
        );
        let mut timer_2: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ> =
            tim2.counter();
        timer_2
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_2.listen(Event::Update);

        let tim3 = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM3, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM3,
            &clocks,
        );
        let mut timer_3: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM3, TIMER_CLOCK_FREQ> =
            tim3.counter();
        timer_3
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_3.listen(Event::Update);

        let tim4 = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM4, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM4,
            &clocks,
        );
        let mut timer_4: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM4, TIMER_CLOCK_FREQ> =
            tim4.counter();
        timer_4
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_4.listen(Event::Update);

        let tim5 = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM5, TIMER_CLOCK_FREQ>::new(
            cx.device.TIM5,
            &clocks,
        );
        let mut timer_5: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM5, TIMER_CLOCK_FREQ> =
            tim5.counter();
        timer_5
            .start(stepper_state.micros_pulse_duration.micros())
            .unwrap();
        timer_5.listen(Event::Update);

        let (sender_1, stepper_1_commands_receiver) =
            make_channel!(MyStepperCommands, CHANNEL_CAPACITY);
        let (sender_2, stepper_2_commands_receiver) =
            make_channel!(MyStepperCommands, CHANNEL_CAPACITY);
        let (sender_3, stepper_3_commands_receiver) =
            make_channel!(MyStepperCommands, CHANNEL_CAPACITY);
        let (sender_4, stepper_4_commands_receiver) =
            make_channel!(MyStepperCommands, CHANNEL_CAPACITY);
        let (mut display_sender, display_receiver) =
            make_channel!(Box<DisplayMemoryPool>, CHANNEL_CAPACITY);

        let stepper_1 = MyStepper::new(
            1,
            stepper_state.clone(),
            timer_2,
            x_step_pin,
            stepper_1_commands_receiver,
        );
        let stepper_2 = MyStepper::new(
            2,
            stepper_state.clone(),
            timer_3,
            y_step_pin,
            stepper_2_commands_receiver,
        );
        let stepper_3 = MyStepper::new(
            3,
            stepper_state.clone(),
            timer_4,
            z_step_pin,
            stepper_3_commands_receiver,
        );
        let stepper_4 = MyStepper::new(
            4,
            stepper_state.clone(),
            timer_5,
            e_step_pin,
            stepper_4_commands_receiver,
        );
        //
        //
        //
        //

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, STEPPER_CLOCK_FREQ, systick_mono_token);

        serial_usart1.listen(stm32f1xx_hal::serial::Event::Rxne);
        serial_usart2.listen(stm32f1xx_hal::serial::Event::Rxne);
        dma1_channel_6.listen(stm32f1xx_hal::dma::Event::TransferComplete);
        let (_, rx_usart1) = serial_usart1.split();
        let (_, rx_usart2) = serial_usart2.split();

        display_task::spawn().unwrap();
        display_task_writer::spawn().unwrap();
        (
            Shared {
                rx_usart2: Some(rx_usart2),
                // transfer: None,
            },
            Local {
                rx_usart1,
                stepper_1,
                stepper_1_sender: sender_1,
                display_receiver,
                display_sender,
                display,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            rtic::export::nop();
        }
    }

    #[task(priority = 8, local = [display_receiver, display])]
    async fn display_task(cx: display_task::Context) {
        loop {
            defmt::debug!("'display_task' loop start");
            let message = cx.local.display_receiver.recv().await.unwrap();
            cx.local.display.print(message).await;
            defmt::debug!("'display_task' loop finish");
        }
    }

    static PS3_CROSS_DATA: &'static [u8] = &[0x10, 0, 0x54, 0xFF];
    // TODO: WHY???
    // TODO: WHY???
    // TODO: WHY???
    // TODO: WHY???
    // TODO: WHY???
    // TODO: WHY???
    // Only if priority > 'display_task' priority it works.
    // Otherwise stucks in some unknown place.
    //#[task(binds = DMA1_CHANNEL6, priority = 7, local = [  speed: u8 = 0, stepper_1_sender ], shared = [ rx_usart2, transfer ])]
    //fn esp32_reader_finish(mut cx: esp32_reader_finish::Context) {
    //    defmt::debug!("esp32_reader_finish start");
    //    cx.shared.transfer.lock(|transfer| {
    //        if transfer.is_none() {
    //            defmt::error!("esp32_reader_finish: transfer is None!");
    //            return;
    //        }
    //        let (buf, mut rx) = transfer.take().unwrap().wait();
    //        cx.shared.rx_usart2.lock(|rx_usart2| {
    //            // changing registr for nothing. Try to avoid that part.
    //            // here you only want to stop listening for UART2
    //            let (mut rx, c6) = rx.release();
    //            let is_DMA_complete = c6.isr().tcif6().is_complete();
    //            defmt::debug!("esp32_reader_finish:: is DMA complete: {}", is_DMA_complete);
    //            rx.listen();
    //            let rx = rx.with_dma(c6);
    //            ////////////////////////

    //            // rx.channel.isr().tcif6().is_complete();
    //            // rx.channel.ifcr().as_ptr().read()

    //            // stm32f1xx_hal::pac::dma1::isr::R.call(args)

    //            rx_usart2.replace(rx);
    //        });

    //        defmt::debug!("data from esp32: {:?}", defmt::Debug2Format(buf));

    //        let buf_iter = buf.iter();
    //        let is_cross_button: bool = buf_iter
    //            .zip(PS3_CROSS_DATA.iter())
    //            .map(|(e1, e2)| e1.eq(e2))
    //            .all(|x| x == true);
    //        defmt::debug!("is_cross_button: {}", is_cross_button);
    //        if is_cross_button {
    //            let speed = cx.local.speed;
    //            let mut command = MyStepperCommands::Stay;

    //            *speed = *speed + 1;
    //            if *speed == MAX_SPEED_VAL + 1 {
    //                command = MyStepperCommands::Stay;
    //            } else if *speed > MAX_SPEED_VAL + 1 {
    //                *speed = 0;
    //                command = MyStepperCommands::Move(*speed);
    //            } else {
    //                command = MyStepperCommands::Move(*speed);
    //            }

    //            cx.local
    //                .stepper_1_sender
    //                .try_send(command)
    //                .err()
    //                .map(|err| {
    //                    defmt::debug!(
    //                        "fail to send command to stepper_1: {:?}",
    //                        defmt::Debug2Format(&err)
    //                    )
    //                });
    //        };
    //        // if rx.is_rx_not_empty() {
    //        //     let received = rx.read();
    //        //     match received {
    //        //         Ok(read) => {
    //        //             defmt::debug!("data from esp32: {}", read);
    //        //             let speed = cx.local.speed;
    //        //             let mut command = MyStepperCommands::Stay;

    //        //             *speed = *speed + 1;
    //        //             if *speed == MAX_SPEED_VAL + 1 {
    //        //                 command = MyStepperCommands::Stay;
    //        //             } else if *speed > MAX_SPEED_VAL + 1 {
    //        //                 *speed = 0;
    //        //                 command = MyStepperCommands::Move(*speed);
    //        //             } else {
    //        //                 command = MyStepperCommands::Move(*speed);
    //        //             }

    //        //             cx.local
    //        //                 .stepper_1_sender
    //        //                 .try_send(command)
    //        //                 .err()
    //        //                 .map(|err| {
    //        //                     defmt::debug!(
    //        //                         "fail to send command to stepper_1: {:?}",
    //        //                         defmt::Debug2Format(&err)
    //        //                     )
    //        //                 });
    //        //         }

    //        //         Err(err) => {
    //        //             defmt::debug!("data from esp32 read err: {:?}", defmt::Debug2Format(&err));
    //        //         }
    //        //     }
    //        // }
    //    });
    //    defmt::debug!("esp32_reader_finish finish");
    //}

    // Only if priority > display_priority it works.
    // Otherwise stucks in some unknown place.
    //Task reads 4 bytes from PS3 controller that connected with ESP32 via bluetooth.
    //ESP32 connected to 'motherboard' via UART.
    #[task(binds = USART2, priority = 12, local = [], shared = [ rx_usart2 ])]
    fn esp32_reader(mut cx: esp32_reader::Context) {
        cortex_m::interrupt::free(|_| {
            defmt::debug!("esp32_reader start");
            cx.shared.rx_usart2.lock(|rx_usart2| {
                if rx_usart2.is_none() {
                    defmt::error!("esp32_reader: rx_usart2 is None!");
                    return;
                }
                rx_usart2.as_mut().map(|rx| match rx.read() {
                    Ok(b) => {
                        defmt::debug!("byte readed: {}", b);
                    }
                    Err(_) => {
                        defmt::error!("byte read ERROR");
                    }
                });
            });
            //if rx.channel.isr().gif6().is_event() {
            //    defmt::debug!("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            //}
            //if rx.channel.isr().tcif6().is_complete() {
            //    defmt::debug!(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            //}
            //// changing registr for nothing. Try to avoid that part.
            //// here you only want to stop listening for UART2
            //let (mut rx, c6) = rx.release();
            //let is_DMA_complete = c6.isr().tcif6().is_complete();
            //defmt::debug!("esp32_reader:: is DMA complete: {}", is_DMA_complete);
            //// rx.unlisten();
            //let rx = rx.with_dma(c6);
            //////////////////////////
            //let new_transfer = unsafe { rx.read(&mut PS3_BUF) };
            //cx.shared.transfer.lock(|transfer| {
            //    transfer.replace(new_transfer);
            //});
        });
        defmt::debug!("esp32_reader finish");
    }

    #[task(binds = USART1, priority = 2, local = [ rx_usart1 ])]
    fn bluetooth_reader(cx: bluetooth_reader::Context) {
        let rx = cx.local.rx_usart1;
        if rx.is_rx_not_empty() {
            let received = rx.read();
            match received {
                Ok(read) => defmt::debug!("data via bluetooth: {}", read),

                Err(err) => {
                    defmt::debug!(
                        "data via bluetooth read err: {:?}",
                        defmt::Debug2Format(&err)
                    );
                }
            }
        }
    }

    #[task(binds = TIM2, priority = 1, local = [ stepper_1 ])]
    fn delay_task_1(cx: delay_task_1::Context) {
        cx.local.stepper_1.work();
    }

    // #[task(binds = TIM3, priority = 3, local = [ stepper_2 ])]
    // fn delay_task_2(cx: delay_task_2::Context) {
    //     cx.local.stepper_2.work();
    // }

    // #[task(binds = TIM4, priority = 3, local = [ stepper_3 ])]
    // fn delay_task_3(cx: delay_task_3::Context) {
    //     cx.local.stepper_3.work();
    // }

    // #[task(binds = TIM5, priority = 3, local = [ stepper_4 ])]
    // fn delay_task_4(cx: delay_task_4::Context) {
    //     cx.local.stepper_4.work();
    // }

    use core::fmt::Write;
    #[task(priority = 9, local = [display_sender])]
    async fn display_task_writer(mut cx: display_task_writer::Context) {
        let mut index: u32 = 0;
        loop {
            defmt::debug!("in 'display_task_writer' loop");
            let mut data_str = DisplayString::new();
            write!(data_str, "Hello world: {index}").expect("not written");
            display::display_str(data_str, &mut cx.local.display_sender)
                .await
                .unwrap();

            Systick::delay(1000.millis()).await;
            index += 1;
        }
    }
}
