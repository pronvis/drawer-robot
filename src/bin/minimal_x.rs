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

    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f1xx_hal::prelude::*;
    use stm32f1xx_hal::timer::{Counter, Event};

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;

    const CHANNEL_CAPACITY: usize = drawer_robot::my_tmc2209::communicator::CHANNEL_CAPACITY;
    const READ_STEPPER_DRIVER_STATE_CLOCK_FREQ: u32 = 72_000; // tick = 13.889 micros
    const TMC2209COMMUNICATOR_CLOCK_FREQ: u32 = 72_0_000; // tick = 13.89 nanos
                                                          // const TMC2209COMMUNICATOR_CLOCK_FREQ: u32 = 500_000; // tick = 2 micros
                                                          // in 1 second 72_000_000 ticks happens
                                                          // If I set BIT_SEND_NANOS_DUR = 280 && TMC2209COMMUNICATOR_CLOCK_FREQ = 72_00_00
                                                          // then it doesnt work with COMMENTED debug log, even with 'opt-level=0'...
                                                          // No idea why!
                                                          //TODO:
                                                          // НАЙТИ МИНИМУМ ПРИ КОТОРОМ БУДЕТ РАБОТАТЬ ЧТЕНИЕ!!!
    const BIT_SEND_NANOS_DUR: u32 = 2800;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        configurator: drawer_robot::my_tmc2209::configurator::Configurator,
        tmc2209_communicator_timer: Counter<stm32f1xx_hal::pac::TIM2, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        tmc2209_msg_sender: Sender<'static, drawer_robot::my_tmc2209::Request, CHANNEL_CAPACITY>,
        tmc2209_rsp_receiver: Receiver<'static, u32, CHANNEL_CAPACITY>,
        communicator: TMC2209SerialCommunicator<'C', 10>,
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
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        // Acquire the GPIOC peripheral
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();

        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();
        let x_stepper_uart_pin = gpioc.pc10.into_dynamic(&mut gpioc.crh);

        let mut tmc2209_communicator_timer = drawer_robot::get_counter(cx.device.TIM2, &clocks);
        defmt::debug!(
            "tmc2209_communicator_timer {}",
            BIT_SEND_NANOS_DUR.nanos::<1, TMC2209COMMUNICATOR_CLOCK_FREQ>().ticks()
        );

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        tmc2209_communicator_timer.start(BIT_SEND_NANOS_DUR.nanos()).unwrap();
        tmc2209_communicator_timer.listen(Event::Update);

        let (mut tmc2209_msg_sender, tmc2209_msg_receiver) = make_channel!(drawer_robot::my_tmc2209::Request, CHANNEL_CAPACITY);
        let (mut tmc2209_rsp_sender, tmc2209_rsp_receiver) = make_channel!(u32, CHANNEL_CAPACITY);

        task1::spawn().ok();

        let configurator = drawer_robot::my_tmc2209::configurator::Configurator::new();
        let communicator = TMC2209SerialCommunicator::new(tmc2209_msg_receiver, tmc2209_rsp_sender, x_stepper_uart_pin, gpioc.crh);
        (
            Shared {},
            Local {
                configurator,
                tmc2209_communicator_timer,
                tmc2209_msg_sender,
                tmc2209_rsp_receiver,
                communicator,
            },
        )
    }

    #[task(priority = 1, local = [tmc2209_msg_sender, tmc2209_rsp_receiver])]
    async fn task1(mut cx: task1::Context) {
        loop {
            if let Ok(message) = cx.local.tmc2209_rsp_receiver.try_recv() {
                defmt::info!("Receive response: {}", message);
            }
            let read_req = tmc2209::read_request::<tmc2209::reg::IFCNT>(0);
            let req = drawer_robot::my_tmc2209::Request::read(read_req);
            let res = cx.local.tmc2209_msg_sender.try_send(req).ok();
            match res {
                None => defmt::debug!("fail to send message to initiate Communicator"),
                Some(_) => defmt::debug!("Successfully send READ message to Communicator"),
            }

            Systick::delay(1000.millis()).await;
        }
    }

    #[task(binds = TIM2, priority = 5, local = [communicator, tmc2209_communicator_timer])]
    fn communicator_task(mut cx: communicator_task::Context) {
        cx.local.communicator.handle_interrupt();
        cx.local.tmc2209_communicator_timer.clear_interrupt(Event::Update);
    }
}
