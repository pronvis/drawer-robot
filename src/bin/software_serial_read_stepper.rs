#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use drawer_robot as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32f1xx_hal::pac,
    peripherals = true,
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [EXTI0, EXTI1, EXTI2, EXTI3, EXTI4]
    // dispatchers = [PVD, WWDG, RTC, SPI1]
)]
mod app {
    use drawer_robot::my_tmc2209::communicator::TMC2209SerialCommunicator;

    use core::ops::Not;
    use fugit::Duration;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f1xx_hal::{
        prelude::*,
        timer::{Counter, Event},
    };

    const CHANNEL_CAPACITY: usize = drawer_robot::my_tmc2209::communicator::CHANNEL_CAPACITY;

    const WRITE_CLOCK_FREQ: u32 = 72_00; // tick = 138.89 micros
    const READ_CLOCK_FREQ: u32 = 72_00; // tick = 138.89 micros
    const MAIN_CLOCK_FREQ: u32 = 72_000_000;

    //MIN WORKING VALUE = 111 micros:
    //  242 ticks with FREQ: 72_0_000 (OVERSAMPLE = 1)
    //  80 ticks with FREQ: 72_0_000 (OVERSAMPLE = 3)
    //
    //MAX WORKING VALUE = 10416 nanos:
    //  75 ticks with FREQ: 72_00_000 (OVERSAMPLE = 3)
    //
    //LETS CHOOSE MIDDLE: 55555 nanos
    const BIT_SEND_TICKS: u32 = 4;
    const TMC2209COMMUNICATOR_CLOCK_FREQ: u32 = 72_000;

    #[shared]
    struct Shared {
        x: u64,
    }

    #[local]
    struct Local {
        communicator: TMC2209SerialCommunicator<'C', 10>,
        tmc2209_communicator_timer: Counter<stm32f1xx_hal::pac::TIM2, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        tmc2209_msg_sender_1: Sender<'static, drawer_robot::my_tmc2209::Request, CHANNEL_CAPACITY>,
        tmc2209_msg_sender_2: Sender<'static, drawer_robot::my_tmc2209::Request, CHANNEL_CAPACITY>,
        tmc2209_rsp_receiver: Receiver<'static, u32, CHANNEL_CAPACITY>,

        read_timer: Counter<stm32f1xx_hal::pac::TIM3, READ_CLOCK_FREQ>,
        write_timer: Counter<stm32f1xx_hal::pac::TIM4, WRITE_CLOCK_FREQ>,
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

        // Acquire the GPIOC peripheral
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();

        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();
        let x_stepper_uart_pin = gpioc.pc10.into_dynamic(&mut gpioc.crh);

        let mut read_timer = drawer_robot::get_counter(cx.device.TIM3, &clocks);
        let mut write_timer = drawer_robot::get_counter(cx.device.TIM4, &clocks);
        let mut tmc2209_communicator_timer = drawer_robot::get_counter(cx.device.TIM2, &clocks);

        //TODO: I dont need it. But! If remove, then commenting useless check in 'prepare_to_send_read_req'
        //will not change behavior anymore...
        // let systick_mono_token = rtic_monotonics::create_systick_token!();
        // Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        read_timer.start(1.secs()).unwrap();
        write_timer.start(200.millis()).unwrap();
        let tmc2209_timer_ticks = Duration::<u32, 1, TMC2209COMMUNICATOR_CLOCK_FREQ>::from_ticks(BIT_SEND_TICKS);
        tmc2209_communicator_timer.start(tmc2209_timer_ticks).unwrap();
        defmt::debug!("tmc2209_communicator_timer perior: {} nanos", tmc2209_timer_ticks.to_nanos());

        read_timer.listen(Event::Update);
        write_timer.listen(Event::Update);
        tmc2209_communicator_timer.listen(Event::Update);

        let (tmc2209_msg_sender, tmc2209_msg_receiver) = make_channel!(drawer_robot::my_tmc2209::Request, CHANNEL_CAPACITY);
        let (tmc2209_rsp_sender, tmc2209_rsp_receiver) = make_channel!(u32, CHANNEL_CAPACITY);

        let communicator = TMC2209SerialCommunicator::new(tmc2209_msg_receiver, tmc2209_rsp_sender, x_stepper_uart_pin, gpioc.crh);
        (
            Shared { x: 0 },
            Local {
                communicator,
                tmc2209_communicator_timer,
                tmc2209_msg_sender_1: tmc2209_msg_sender.clone(),
                tmc2209_msg_sender_2: tmc2209_msg_sender,
                tmc2209_rsp_receiver,

                read_timer,
                write_timer,
            },
        )
    }

    #[task(binds = TIM4, priority = 5, local = [write_timer, tmc2209_msg_sender_1, speed: u32 = 1000, direction: bool = true])]
    fn write_task(cx: write_task::Context) {
        cx.local.write_timer.clear_interrupt(Event::Update);
        let speed_step: i32 = {
            if *cx.local.direction {
                300
            } else {
                -300
            }
        };

        *cx.local.speed = (*cx.local.speed as i32 + speed_step) as u32;
        if *cx.local.speed >= 10000 || *cx.local.speed <= 100 {
            *cx.local.direction = cx.local.direction.not();
        }

        // defmt::debug!("current speed: {}", cx.local.speed);

        let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(*cx.local.speed));
        let req = drawer_robot::my_tmc2209::Request::write(write_req);
        let _ = cx.local.tmc2209_msg_sender_1.try_send(req).ok();
    }

    #[task(binds = TIM2, priority = 10, local = [communicator, tmc2209_communicator_timer], shared = [x])]
    fn communicator_task(mut cx: communicator_task::Context) {
        cx.local.tmc2209_communicator_timer.clear_interrupt(Event::Update);
        cx.local.communicator.handle_interrupt();
    }

    //TODO: Works only if PRIORITY of 'read_task' is higher then 'communicator_task'
    #[task(binds = TIM3, priority = 1, local = [read_timer, tmc2209_msg_sender_2, tmc2209_rsp_receiver, first: bool = true], shared = [x])]
    fn read_task(mut cx: read_task::Context) {
        cx.local.read_timer.clear_interrupt(Event::Update);

        if *cx.local.first {
            *cx.local.first = false;

            let write_req = tmc2209::write_request(0, tmc2209::reg::SLAVECONF(0));
            let req = drawer_robot::my_tmc2209::Request::write(write_req);
            let _ = cx.local.tmc2209_msg_sender_2.try_send(req).ok();
        } else {
            match cx.local.tmc2209_rsp_receiver.try_recv() {
                Err(_) => {}
                Ok(response) => {
                    let ifcnt_resp = tmc2209::reg::IFCNT::from(response);
                    defmt::info!("Transmission counter: {}", ifcnt_resp.0);
                } // Err(err) => defmt::warn!("Fail to receive response from communicator"),
            }

            let read_req = tmc2209::read_request::<tmc2209::reg::IFCNT>(0);
            let req = drawer_robot::my_tmc2209::Request::read(read_req);
            let res = cx.local.tmc2209_msg_sender_2.try_send(req).ok();
            // match res {
            //     None => defmt::debug!("fail to send message to Communicator"),
            //     Some(_) => defmt::debug!("Successfully send READ message to Communicator"),
            // }
        }
    }
}
