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

    use fugit::Duration;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f1xx_hal::prelude::*;
    use stm32f1xx_hal::timer::{Counter, Event};

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const CHANNEL_CAPACITY: usize = drawer_robot::my_tmc2209::communicator::CHANNEL_CAPACITY;

    // > 6 ticks Configurator need Systick::delay for some reason...
    const BIT_SEND_TICKS: u32 = 40;
    const TMC2209COMMUNICATOR_CLOCK_FREQ: u32 = 72_0_000;

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

        let mut tmc2209_communicator_timer = drawer_robot::get_counter(cx.device.TIM2, &clocks);
        let tmc2209_timer_ticks = Duration::<u32, 1, TMC2209COMMUNICATOR_CLOCK_FREQ>::from_ticks(BIT_SEND_TICKS);
        tmc2209_communicator_timer.start(tmc2209_timer_ticks).unwrap();
        defmt::debug!("tmc2209_communicator_timer perior: {} nanos", tmc2209_timer_ticks.to_nanos());
        tmc2209_communicator_timer.listen(Event::Update);

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        let (mut tmc2209_msg_sender, tmc2209_msg_receiver) = make_channel!(drawer_robot::my_tmc2209::Request, CHANNEL_CAPACITY);
        let (mut tmc2209_rsp_sender, tmc2209_rsp_receiver) = make_channel!(u32, CHANNEL_CAPACITY);

        let communicator = TMC2209SerialCommunicator::new(tmc2209_msg_receiver, tmc2209_rsp_sender, x_stepper_uart_pin, gpioc.crh);
        let configurator = drawer_robot::my_tmc2209::configurator::Configurator::new(tmc2209_msg_sender.clone());

        stepper_conf_task::spawn().ok();
        stepper_change_speed_task::spawn().ok();
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

    #[task(priority = 1, local = [configurator, tmc2209_rsp_receiver])]
    async fn stepper_conf_task(mut cx: stepper_conf_task::Context) {
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

    #[task(priority = 1, local = [tmc2209_msg_sender])]
    async fn stepper_change_speed_task(mut cx: stepper_change_speed_task::Context) {
        let mut speed: i32 = 100;
        let mut step: i32 = 1000;
        let max_speed = 600_000;
        loop {
            let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(speed as u32));
            let req = drawer_robot::my_tmc2209::Request::write(write_req);
            cx.local.tmc2209_msg_sender.send(req).await.ok();
            speed += step;

            if speed >= max_speed || speed <= 0 {
                step = -1 * step;
                defmt::debug!("step: {}", step);
            }
        }
    }

    #[task(binds = TIM2, priority = 5, local = [communicator, tmc2209_communicator_timer])]
    fn communicator_task(mut cx: communicator_task::Context) {
        cx.local.tmc2209_communicator_timer.clear_interrupt(Event::Update);
        cx.local.communicator.handle_interrupt();
    }
}
