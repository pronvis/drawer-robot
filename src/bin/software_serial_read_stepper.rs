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
    const TIMER_2_CLOCK_FREQ: u32 = 72_000_000; // tick = 13.89 nanos
    // const TIMER_2_CLOCK_FREQ: u32 = 500_000; // tick = 2 micros
    
    // in 1 second 72_000_000 ticks happens
    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const BIT_SEND_NANOS_DUR: u32 = 28;
    const WRITE_REQ_BYTE_COUNT: usize = 8;
    const OVERSAMPLE: u8 = 3;

    #[shared]
    struct Shared {
        write_req: Option<tmc2209::WriteRequest>,
        timer_2: Counter<stm32f1xx_hal::pac::TIM2, TIMER_2_CLOCK_FREQ>,
    }

    type X_UART_PIN = Pin<'C', 10, Dynamic>;

    #[local]
    struct Local {
        speed: u32,
        tmc2209: TMC2209Communicator<'C', 10>,
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
        timer_1.start(100.millis()).unwrap();
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
                cr: gpioc.crh,
                speed: 1000,
                tmc2209: TMC2209Communicator::new(x_stepper_uart_pin)
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

        defmt::debug!("curr speed: {}", *cx.local.speed);
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

        cx.local.timer_1.clear_interrupt(Event::Update);
    }

    #[task(binds = TIM2, priority = 5, local = [
           cr, 
           tmc2209,
           bytes_to_send: [u8; 8] = [0; 8], 
           initialized: bool = false,
           tx_buffer: u16 = 0,
           sending_byte_index: usize = 0,
    ], shared = [write_req, timer_2])]
    fn write_command_task(mut cx: write_command_task::Context) {
         if !*cx.local.initialized {
            cx.shared.write_req.lock(|write_req| {
                if write_req.is_none() {
                    return;
                }

                cx.local.tmc2209.write(write_req.take().unwrap());
            });
            *cx.local.initialized = true;
         }

         if cx.local.tmc2209.working() {
            cx.local.tmc2209.handle_interrupt();
         } else {
            cx.shared.timer_2.lock(|timer_2| {
                timer_2.unlisten(Event::Update);
                *cx.local.initialized = false;
            });
         }

        
        // if !*cx.local.initialized {
        //     cx.shared.write_req.lock(|write_req| {
        //         if write_req.is_none() {
        //             defmt::debug!("Write request is None! Fail to initialize");
        //             return;
        //         }

        //         let write_req_unwrapped = write_req.take().unwrap();
        //         for i in 0..WRITE_REQ_BYTE_COUNT {
        //             cx.local.bytes_to_send[i] = write_req_unwrapped.bytes()[i];
        //         }
        //         *cx.local.initialized = true;
        //         *cx.local.tx_buffer = 0;
        //         *cx.local.sending_byte_index = 0;
        //         cx.local.x_stepper_uart_pin.make_push_pull_output(cx.local.cr);
        //         cx.local.x_stepper_uart_pin.set_high();

        //         // defmt::debug!("bytes to send: {:?}", cx.local.bytes_to_send);
        //     });
        // }

        // if !*cx.local.initialized || (*cx.local.sending_byte_index == WRITE_REQ_BYTE_COUNT  && *cx.local.tx_buffer == 0) {
        //     *cx.local.initialized = false;
        //     cx.shared.timer_2.lock(|timer_2| {
        //         timer_2.unlisten(Event::Update);
        //         timer_2.clear_interrupt(Event::Update);
        //     });
        //     return;
        // }

        // if *cx.local.tx_buffer == 0 {
        //     *cx.local.tx_buffer = (cx.local.bytes_to_send[*cx.local.sending_byte_index] as u16) << 1 | 0x200;
        //     // defmt::debug!("sending byte: {:b}", cx.local.tx_buffer);
        //     *cx.local.sending_byte_index += 1;
        // }

        // if (*cx.local.tx_buffer & 1 == 1) {
        //     if cx.local.x_stepper_uart_pin.set_high().is_err() {
        //         defmt::debug!("fail to set pin to high");
        //     }
        //   } else {
        //       if cx.local.x_stepper_uart_pin.set_low().is_err() {
        //         defmt::debug!("fail to set pin to low");
        //       }
        // }
        // *cx.local.tx_buffer >>= 1;
 
        // cx.shared.timer_2.lock(|timer_2| {
        //     timer_2.clear_interrupt(Event::Update);
        // });
    }

//     #[task(binds = TIM3, priority = 6, local = [
//            x_stepper_uart_pin,
//            cr,
//            bytes_to_send: [u8; 8] = [0; 8], 
//            initialized: bool = false,
//            tx_buffer: u16 = 0,
//            sending_byte_index: usize = 0,
//     ], shared = [write_req, timer_2])]
//     fn read_command_task(mut cx: read_command_task::Context) {

//     }

    struct TMC2209Communicator<const pinC: char, const pinN: u8>
    where
        Pin<pinC, pinN, Dynamic>: HL,
    {
        tx_tick_counter: u8, // interrupt tick counter for TX
        tx_bit_counter: u8, //TODO: probably I dont need it, cause can use 'tx_bits_buffer == 0' to
                            //find the end

        tx_bits_buffer: u16, // buffer for byte + stop & start bits
        bytes_to_send: [u8; 8], // buffer for bytes to send
        tx_bytes_counter: usize,  // counter for sended bytes

        current_state: bool, // should be Sending, Receiving, Nothing

        pin: Pin<pinC, pinN, Dynamic>,
    }

    impl<const pinC: char, const pinN: u8>
        TMC2209Communicator<pinC, pinN>
    where
        Pin<pinC, pinN, Dynamic>: HL,
    {

        pub fn new(pin: Pin<pinC, pinN, Dynamic>) -> Self {
            Self {
                tx_tick_counter: 0,
                tx_bit_counter: 0,
                tx_bits_buffer: 0,
                bytes_to_send: [0; 8],
                tx_bytes_counter: 0,

                current_state: false,
                pin
            }
        }

        pub fn working(&self) -> bool {
            return self.current_state;
        }

        pub fn write(&mut self, req: tmc2209::WriteRequest) {
            //TODO: What if we in a process of sending now?
            
            // cause 'tx_bytes_counter' starts from 'len' we need to reverse
            for (index, elem) in req.bytes().iter().rev().enumerate() {
                self.bytes_to_send[index] = *elem;
            }
            self.tx_bit_counter = 0;
            self.tx_tick_counter = OVERSAMPLE;
            self.tx_bytes_counter = req.bytes().len();
            self.tx_bits_buffer = (self.bytes_to_send[self.tx_bytes_counter - 1] as u16) << 1 | 0x200;
            self.current_state = true;
        }
    
        fn send(&mut self) {
             // if tx_tick_counter > 0 interrupt is discarded. Only when tx_tick_counter reach 0 we set TX pin.
            self.tx_tick_counter -= 1;
            // defmt::debug!("message after decrement tick_counter, curr value: {}", self.tx_tick_counter);
            if self.tx_tick_counter == 0 {
                if self.tx_bit_counter < 10 {
                    // defmt::debug!("writing bit: {}", self.tx_bit_counter);
                    self.tx_bit_counter += 1;
                    
                    if (self.tx_bits_buffer & 1 == 1) {
                        if self.pin.set_high().is_err() {
                            defmt::debug!("fail to set pin to high");
                        }
                      } else {
                          if self.pin.set_low().is_err() {
                            defmt::debug!("fail to set pin to low");
                          }
                    }
                    self.tx_bits_buffer >>= 1;
                    self.tx_tick_counter = OVERSAMPLE;
                } else {
                    // byte has been sended
                    self.tx_bytes_counter -= 1;

                    if self.tx_bytes_counter == 0 {
                        // all bytes has been sended
                        self.current_state = false;

                                // When in half-duplex mode, wait for HALFDUPLEX_SWITCH_DELAY bit-periods after the byte has
                                // been transmitted before allowing the switch to RX mode
                              // } else if (tx_bit_cnt > 10 + OVERSAMPLE * HALFDUPLEX_SWITCH_DELAY) {
                              //   if (_half_duplex && active_listener == this) {
                              //     setRXTX(true);
                              //   }
                              //   active_out = nullptr;
                              // }

                    } else {
                        // continue to next byte
                        // defmt::debug!("going to byte #{}", self.tx_bytes_counter - 1);
                        self.tx_bits_buffer = (self.bytes_to_send[self.tx_bytes_counter - 1] as u16) << 1 | 0x200;
                        // defmt::debug!("sending byte: {:b}", self.tx_bits_buffer);
                        self.tx_tick_counter = OVERSAMPLE;
                        self.tx_bit_counter = 0;
                    }
                    
                }

            }
        }

        pub fn handle_interrupt(&mut self) {
            if self.current_state {
                self.send();
            }
            //TODO:
        }

    }
}
