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

    use core::borrow::BorrowMut;
    use core::ops::Not;

    use drawer_robot::*;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use rtic_monotonics::systick::*;
    use stm32f1xx_hal::gpio::{Cr, Dynamic, Pin, HL};
    use stm32f1xx_hal::{
        gpio::PinState,
        pac,
        prelude::*,
        rcc,
        rcc::*,
        timer::Timer,
        timer::{Counter, Event},
    };
    use fugit::{HertzU32 as Hertz, MicrosDurationU32, TimerDurationU32, TimerInstantU32};

    const TIMER_1_CLOCK_FREQ: u32 = 72_00; // tick = 138.89 micros
    const TIMER_2_CLOCK_FREQ: u32 = 72_000_000; // tick = 2 micros
    // const TIMER_2_CLOCK_FREQ: u32 = 500_000; // tick = 2 micros
    
    // in 1 second 72_000_000 ticks happens
    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const BIT_SEND_MICROS_DUR: u32 = 4;
    const BIT_SEND_NANOS_DUR: u32 = 28;
    const WRITE_REQ_BYTE_COUNT: usize = 8;

    #[shared]
    struct Shared {
        write_req: Option<tmc2209::WriteRequest>,
        timer_2: Counter<stm32f1xx_hal::pac::TIM2, TIMER_2_CLOCK_FREQ>,
    }

    type X_UART_PIN = Pin<'C', 10, Dynamic>;

    #[local]
    struct Local {
        speed: u32,
        x_stepper_uart_pin: X_UART_PIN,
        timer_1: Counter<stm32f1xx_hal::pac::TIM1, TIMER_1_CLOCK_FREQ>,
        cr: <X_UART_PIN as HL>::Cr,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
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
        let mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts = cx.device.GPIOB.split();
        let step_pin = gpioc
            .pc6
            .into_push_pull_output_with_state(&mut gpioc.crl, PinState::Low);

        let mut en = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        en.set_low();
        let mut x_stepper_uart_pin = gpioc.pc10.into_dynamic(&mut gpioc.crh);
        x_stepper_uart_pin.make_push_pull_output(&mut gpioc.crh);
        x_stepper_uart_pin.set_high().ok().unwrap();

        let timer =
            stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM1, TIMER_1_CLOCK_FREQ>::new(
                cx.device.TIM1,
                &clocks,
            );
        let mut timer_1: stm32f1xx_hal::timer::Counter<
            stm32f1xx_hal::pac::TIM1,
            TIMER_1_CLOCK_FREQ,
        > = timer.counter();
        timer_1.start(1.secs()).unwrap();
        timer_1.listen(Event::Update);

        let timer =
            stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM2, TIMER_2_CLOCK_FREQ>::new(
                cx.device.TIM2,
                &clocks,
            );
        let mut timer_2: stm32f1xx_hal::timer::Counter<
            stm32f1xx_hal::pac::TIM2,
            TIMER_2_CLOCK_FREQ,
        > = timer.counter();
        // defmt::debug!("timer2_ticks: {}", BIT_SEND_MICROS_DUR.micros::<1, TIMER_2_CLOCK_FREQ>().ticks());
        // timer_2.start(BIT_SEND_MICROS_DUR.micros()).unwrap();
        defmt::debug!("timer2_ticks: {}", BIT_SEND_NANOS_DUR.nanos::<1, TIMER_2_CLOCK_FREQ>().ticks());
        timer_2.start(BIT_SEND_NANOS_DUR.nanos()).unwrap();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        (
            Shared {
                timer_2,
                write_req: None,
            },
            Local {
                timer_1,
                x_stepper_uart_pin,
                cr: gpioc.crh,
                speed: 1000,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::debug!("idle");

        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = TIM1_UP, priority = 3, local = [timer_1, speed, direction: bool = true], shared = [write_req, timer_2])]
    fn send_command_timer(mut cx: send_command_timer::Context) {
        // let speed_step: i32 = {
        //     if *cx.local.direction {
        //         100
        //     } else {
        //         -100
        //     }
        // };

        // *cx.local.speed = (*cx.local.speed as i32 + speed_step) as u32;
        // if *cx.local.speed >= 1200 || *cx.local.speed <= 100 {
        //     cx.local.direction.not();
        // }

        if *cx.local.direction {
            *cx.local.speed = 8000;
        } else {
            *cx.local.speed = 100;
        };
        *cx.local.direction = cx.local.direction.not();

        cx.shared.write_req.lock(|write_req| {
            if write_req.is_some() {
                return;
            }

            *write_req = Some(tmc2209::write_request(
                0,
                tmc2209::reg::VACTUAL(*cx.local.speed),
            ));

            cx.shared.timer_2.lock(|timer_2| {
                timer_2.listen(Event::Update);
            });
        });

        cx.local.timer_1.start(1.secs()).unwrap();
    }

    #[task(binds = TIM2, priority = 5, local = [
           x_stepper_uart_pin, 
           cr, 
           bytes_to_send: [u8; 8] = [0; 8], 
           initialized: bool = false,
           tx_buffer: u16 = 0,
           sending_byte_index: usize = 0,
    ], shared = [write_req, timer_2])]
    fn write_command_task(mut cx: write_command_task::Context) {
        if !*cx.local.initialized {
            cx.shared.write_req.lock(|write_req| {
                if write_req.is_none() {
                    defmt::debug!("Write request is None! Fail to initialize");
                    return;
                }

                let write_req_unwrapped = write_req.take().unwrap();
                for i in 0..WRITE_REQ_BYTE_COUNT {
                    cx.local.bytes_to_send[i] = write_req_unwrapped.bytes()[i];
                }
                *cx.local.initialized = true;
                *cx.local.tx_buffer = 0;
                *cx.local.sending_byte_index = 0;
                cx.local.x_stepper_uart_pin.make_push_pull_output(cx.local.cr);
                cx.local.x_stepper_uart_pin.set_high();

                defmt::debug!("bytes to send: {:?}", cx.local.bytes_to_send);
            });
        }

        if !*cx.local.initialized || (*cx.local.sending_byte_index == WRITE_REQ_BYTE_COUNT  && *cx.local.tx_buffer == 0) {
            *cx.local.initialized = false;
            cx.shared.timer_2.lock(|timer_2| {
                timer_2.unlisten(Event::Update);
                timer_2.clear_interrupt(Event::Update);
            });
            return;
        }

        if *cx.local.tx_buffer == 0 {
            *cx.local.tx_buffer = (cx.local.bytes_to_send[*cx.local.sending_byte_index] as u16) << 1 | 0x200;
            // defmt::debug!("sending byte: {:b}", cx.local.tx_buffer);
            *cx.local.sending_byte_index += 1;
        }

        if (*cx.local.tx_buffer & 1 == 1) {
            if cx.local.x_stepper_uart_pin.set_high().is_err() {
                defmt::debug!("fail to set pin to high");
            }
          } else {
              if cx.local.x_stepper_uart_pin.set_low().is_err() {
                defmt::debug!("fail to set pin to low");
              }
        }
        *cx.local.tx_buffer >>= 1;
 
        cx.shared.timer_2.lock(|timer_2| {
            timer_2.clear_interrupt(Event::Update);
        });
    }

    // fn send_stepper_command(
    //     pin: &mut X_UART_PIN,
    //     cr: &mut <X_UART_PIN as HL>::Cr,
    //     command: tmc2209::WriteRequest,
    // ) {
    //     pin.make_push_pull_output(cr);
    //     pin.set_low().ok();
    // }

    // fn sending_logic(&mut self, request: &dyn Tmc2209Request) {
    //     match self.byte_sending_state {
    //         ByteSendingState::Start => {
    //             self.byte_sending_state = ByteSendingState::Data;
    //             self.pin.set_low().ok();
    //         }

    //         ByteSendingState::Data => {
    //             let current_byte: &u8 = request.bytes().get(self.byte_index as usize).unwrap();
    //             let bit_to_send = Self::bit_value(*current_byte, self.bit_index);
    //             self.bit_index += 1;
    //             if self.bit_index > 7 {
    //                 self.byte_sending_state = ByteSendingState::Stop;
    //                 self.bit_index = 0;
    //             }
    //             if bit_to_send {
    //                 self.pin.set_high().ok();
    //             } else {
    //                 self.pin.set_low().ok();
    //             }
    //         }

    //         ByteSendingState::Stop => {
    //             self.byte_sending_state = ByteSendingState::Start;
    //             self.byte_index += 1;
    //             self.pin.set_high().ok();
    //             if self.byte_index as usize == request.bytes().len() {
    //                 self.data_to_write = None;
    //                 if self.data_to_read.is_some() {
    //                     self.data_to_read = None;
    //                     self.reading = true;
    //                 }
    //             }
    //         }
    //     };
    // }
}
