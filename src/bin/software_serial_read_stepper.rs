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
    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::{
        prelude::*,
        timer::{Counter, Event},
    };

    const SEND_WRITE_AND_READ_REQ_CLOCK_FREQ: u32 = 72_00; // tick = 138.89 micros
    const SEND_WRITE_REQ_CLOCK_FREQ: u32 = 72_00; // tick = 138.89 micros
    const READ_RESPONSE_CLOCK_FREQ: u32 = 72_00; // tick = 138.89 micros
    const READ_STEPPER_DRIVER_STATE_CLOCK_FREQ: u32 = 72_000; // tick = 13.889 micros
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
    struct Local {
        speed_write_only: u32,
        speed_write_and_read: u32,
        send_write_and_read_timer: Counter<stm32f1xx_hal::pac::TIM5, SEND_WRITE_AND_READ_REQ_CLOCK_FREQ>,
        send_write_req_timer: Counter<stm32f1xx_hal::pac::TIM4, SEND_WRITE_REQ_CLOCK_FREQ>,
        read_response_timer: Counter<stm32f1xx_hal::pac::TIM3, READ_RESPONSE_CLOCK_FREQ>,
        read_stepper_driver_state_timer: Counter<stm32f1xx_hal::pac::TIM6, READ_STEPPER_DRIVER_STATE_CLOCK_FREQ>,
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
        let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(72.MHz()).pclk1(36.MHz()).pclk2(72.MHz()).freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        // Acquire the GPIOC peripheral
        let mut gpioc: stm32f1xx_hal::gpio::gpioc::Parts = cx.device.GPIOC.split();

        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();
        let x_stepper_uart_pin = gpioc.pc10.into_dynamic(&mut gpioc.crh);

        let mut read_response_timer = drawer_robot::get_counter(cx.device.TIM3, &clocks);
        let mut send_write_req_timer = drawer_robot::get_counter(cx.device.TIM4, &clocks);
        let mut send_write_and_read_timer = drawer_robot::get_counter(cx.device.TIM5, &clocks);
        let mut read_stepper_driver_state_timer = drawer_robot::get_counter(cx.device.TIM6, &clocks);
        let mut tmc2209_communicator_timer = drawer_robot::get_counter(cx.device.TIM2, &clocks);
        defmt::debug!(
            "tmc2209_communicator_timer {}",
            BIT_SEND_NANOS_DUR.nanos::<1, TMC2209COMMUNICATOR_CLOCK_FREQ>().ticks()
        );

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        read_response_timer.start(500.millis()).unwrap();
        send_write_and_read_timer.start(200.millis()).unwrap();
        tmc2209_communicator_timer.start(BIT_SEND_NANOS_DUR.nanos()).unwrap();
        read_stepper_driver_state_timer.start(100.micros()).unwrap();
        send_write_req_timer.start(200.millis()).unwrap();

        // read_response_timer.listen(Event::Update);
        // send_write_and_read_timer.listen(Event::Update);
        send_write_req_timer.listen(Event::Update);
        read_stepper_driver_state_timer.listen(Event::Update);

        let communicator = TMC2209SerialCommunicator::new(tmc2209_communicator_timer, x_stepper_uart_pin, gpioc.crh);
        (
            Shared { communicator },
            Local {
                send_write_and_read_timer,
                send_write_req_timer,
                read_response_timer,
                read_stepper_driver_state_timer,
                speed_write_only: 1000,
                speed_write_and_read: 1000,
            },
        )
    }

    #[task(binds = TIM6, priority = 3, local = [read_stepper_driver_state_timer, request_index: u8 = 0, response_index: u8 = 0], shared = [communicator])]
    fn read_stepper_driver_state(mut cx: read_stepper_driver_state::Context) {
        let resp_index = *cx.local.response_index;
        let req_index = cx.local.request_index;

        if *req_index == resp_index && *req_index == 0 {
            let read_req = tmc2209::read_request::<tmc2209::reg::IFCNT>(0);
            cx.shared.communicator.lock(|communicator| {
                communicator.send(read_req.bytes());
            });
            *req_index += 1;
        }

        if *req_index > resp_index {
            let mut reader = tmc2209::Reader::default();
            let response_data = cx.shared.communicator.lock(|communicator| {
                return communicator.get_response();
            });

            if response_data.is_none() {
                return;
            }

            let (bytes_read, tmc_response) = reader.read_response(&response_data.unwrap());
            match tmc_response {
                Some(response) => {
                    let ifcnt = tmc2209::reg::IFCNT::from(response.data_u32());
                    defmt::debug!("GOT RESPONSE: Interface transmission counter: {}", ifcnt.0);
                }
                None => {
                    defmt::debug!(
                        "No response, bytes_read: {}, data from reader was: {:?}",
                        bytes_read,
                        response_data
                    );
                }
            }
        }

        cx.local.read_stepper_driver_state_timer.clear_interrupt(Event::Update);
    }

    #[task(binds = TIM4, priority = 3, local = [send_write_req_timer, speed_write_only, direction: bool = true, overrun: u8 = 0], shared = [communicator])]
    fn send_command_timer(mut cx: send_command_timer::Context) {
        let speed_step: i32 = {
            if *cx.local.direction {
                300
            } else {
                -300
            }
        };

        *cx.local.speed_write_only = (*cx.local.speed_write_only as i32 + speed_step) as u32;
        if *cx.local.speed_write_only >= 10000 || *cx.local.speed_write_only <= 100 {
            *cx.local.direction = cx.local.direction.not();
        }

        // defmt::debug!("current speed_write_only: {}", cx.local.speed_write_only);
        cx.shared.communicator.lock(|communicator| {
            let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(*cx.local.speed_write_only));
            communicator.send(write_req.bytes());
        });

        cx.local.send_write_req_timer.clear_interrupt(Event::Update);
    }

    #[task(binds = TIM5, priority = 3, local = [send_write_and_read_timer, speed_write_and_read, direction: bool = true, overrun: u8 = 0], shared = [communicator])]
    fn send_command_timer_with_read(mut cx: send_command_timer_with_read::Context) {
        if *cx.local.overrun == 0 {
            *cx.local.overrun = 1;
            let speed_step: i32 = {
                if *cx.local.direction {
                    300
                } else {
                    -300
                }
            };

            *cx.local.speed_write_and_read = (*cx.local.speed_write_and_read as i32 + speed_step) as u32;
            if *cx.local.speed_write_and_read >= 10000 || *cx.local.speed_write_and_read <= 100 {
                *cx.local.direction = cx.local.direction.not();
            }

            // defmt::debug!("current speed_write_and_read: {}", cx.local.speed_write_and_read);
            cx.shared.communicator.lock(|communicator| {
                let write_req = tmc2209::write_request(0, tmc2209::reg::VACTUAL(*cx.local.speed_write_and_read));
                communicator.send(write_req.bytes());
            });
        } else {
            *cx.local.overrun -= 1;
            // defmt::debug!("send reading");
            cx.shared.communicator.lock(|communicator| {
                let read_req = tmc2209::read_request::<tmc2209::reg::GSTAT>(0);
                communicator.send(read_req.bytes());
            });
        }
        cx.local.send_write_and_read_timer.clear_interrupt(Event::Update);
    }

    #[task(binds = TIM2, priority = 5, shared = [communicator])]
    fn communicator_task(mut cx: communicator_task::Context) {
        cx.shared.communicator.lock(|communicator| communicator.handle_interrupt());
    }

    #[task(binds = TIM3, priority = 2, local=[read_response_timer], shared = [communicator])]
    fn println_task(mut cx: println_task::Context) {
        let mut reader = tmc2209::Reader::default();
        let response_data = cx.shared.communicator.lock(|communicator| {
            //TODO: fix unwrap
            return communicator.get_response().unwrap();
        });
        let (bytes_read, tmc_response) = reader.read_response(&response_data);
        match tmc_response {
            Some(response) => {
                let gstat = tmc2209::reg::GSTAT::from(response.data_u32());
                defmt::debug!("GOT RESPONSE: gconf: {}", gstat.drv_err());
            }
            None => {
                defmt::debug!(
                    "No response, bytes_read: {}, data from reader was: {:?}",
                    bytes_read,
                    response_data
                );
            }
        }
        cx.local.read_response_timer.clear_interrupt(Event::Update);
    }
}
