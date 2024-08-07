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
    use robot_core::my_tmc2209::communicator::TMC2209SerialCommunicator;
    use robot_core::my_tmc2209::configurator::TMC2209Configurator;
    use robot_core::my_tmc2209::Tmc2209Constructor;
    use robot_core::my_tmc2209::{Tmc2209RequestCh, Tmc2209RequestProducer, Tmc2209ResponseCh, Tmc2209ResponseConsumer};

    use defmt_brtt as _; // global logger
    use fugit::Duration;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f1xx_hal::prelude::*;
    use stm32f1xx_hal::timer::{Counter, Event};

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const CHANNEL_CAPACITY: usize = robot_core::my_tmc2209::communicator::CHANNEL_CAPACITY;

    const BIT_SEND_TICKS: u32 = 40;
    const TMC2209COMMUNICATOR_CLOCK_FREQ: u32 = 72_0_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        configurator: TMC2209Configurator,
        tmc2209_communicator_timer: Counter<stm32f1xx_hal::pac::TIM2, TMC2209COMMUNICATOR_CLOCK_FREQ>,
        tmc2209_msg_sender: Tmc2209RequestProducer,
        communicator: TMC2209SerialCommunicator<'C', 10>,
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

        // Acquire the GPIOC peripheral
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();

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
            Shared {},
            Local {
                configurator: tmc2209_0.configurator,
                tmc2209_communicator_timer: tmc2209_0.timer,
                tmc2209_msg_sender: tmc2209_0.req_sender,
                communicator: tmc2209_0.communicator,
            },
        )
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

    #[task(priority = 1, local = [tmc2209_msg_sender])]
    async fn stepper_change_speed_task(cx: stepper_change_speed_task::Context) {
        //wait for Conf task
        Systick::delay(1.secs()).await;

        let min_speed = 200_000;
        let max_speed = 790_000; // empirical value
        let mut speed: i32 = min_speed;
        let mut step: i32 = 10_000;
        loop {
            let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(speed as u32));
            let req = robot_core::my_tmc2209::Request::write(write_req);
            cx.local.tmc2209_msg_sender.send(req).await.ok();

            speed += step;
            if speed >= max_speed || speed <= min_speed {
                step = -1 * step;
                defmt::debug!("step: {}", step);
            }
        }
    }

    #[task(binds = TIM2, priority = 5, local = [communicator, tmc2209_communicator_timer])]
    fn communicator_task(cx: communicator_task::Context) {
        cx.local.tmc2209_communicator_timer.clear_interrupt(Event::Update);
        cx.local.communicator.handle_interrupt();
    }
}
