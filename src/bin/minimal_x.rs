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
    use stm32f1xx_hal::prelude::*;

    const MAIN_CLOCK_FREQ: u32 = 72_000_000;

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
    struct Shared {
        communicator: TMC2209SerialCommunicator<'C', 10, stm32f1xx_hal::pac::TIM2, TMC2209COMMUNICATOR_CLOCK_FREQ>,
    }

    #[local]
    struct Local {}

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

        task1::spawn().ok();

        let communicator = TMC2209SerialCommunicator::new(tmc2209_communicator_timer, x_stepper_uart_pin, gpioc.crh);
        (Shared { communicator }, Local {})
    }

    #[task(priority = 1, shared = [communicator])]
    async fn task1(mut cx: task1::Context) {
        let mut configurator = drawer_robot::my_tmc2209::configurator::Configurator::new();
        while !configurator.finished() {
            cx.shared.communicator.lock(|communicator| {
                configurator.setup(communicator);
            });
            Systick::delay(1000.millis()).await;
        }
    }

    #[task(binds = TIM2, priority = 5, shared = [communicator])]
    fn communicator_task(mut cx: communicator_task::Context) {
        cx.shared.communicator.lock(|communicator| communicator.handle_interrupt());
    }
}
