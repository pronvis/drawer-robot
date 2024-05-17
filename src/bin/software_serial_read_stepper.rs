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

    use drawer_robot::my_stepper::MAX_DELAY_BETWEEN_STEPS;
    use drawer_robot::*;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use fugit::{HertzU32 as Hertz, MicrosDurationU32, TimerDurationU32, TimerInstantU32};
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

    const TIMER_1_CLOCK_FREQ: u32 = 72_00; // tick = 138.89 micros
    const TIMER_2_CLOCK_FREQ: u32 = 72_000_000; // tick = 13.89 nanos
                                                // const TIMER_2_CLOCK_FREQ: u32 = 500_000; // tick = 2 micros

    // in 1 second 72_000_000 ticks happens
    const MAIN_CLOCK_FREQ: u32 = 72_000_000;
    const BIT_SEND_NANOS_DUR: u32 = 28;
    const WRITE_REQ_BYTE_COUNT: usize = 8;
    const OVERSAMPLE: u8 = 3;
    const SWITCH_DELAY: u8 = 8;
    const MAX_RX_BUFFER_SIZE: usize = 64;
    const READ_RESPONSE_LENGTH: u8 = 8;

    #[shared]
    struct Shared {
        write_req: Option<tmc2209::WriteRequest>,
        read_req: Option<tmc2209::ReadRequest>,
        timer_2: Counter<stm32f1xx_hal::pac::TIM2, TIMER_2_CLOCK_FREQ>,
    }

    type X_UART_PIN = Pin<'C', 10, Dynamic>;

    #[local]
    struct Local {
        speed: u32,
        tmc2209: TMC2209Communicator<'C', 10>,
        timer_1: Counter<stm32f1xx_hal::pac::TIM1, TIMER_1_CLOCK_FREQ>,
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
        timer_1.start(200.millis()).unwrap();
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
        defmt::debug!(
            "timer2_ticks: {}",
            BIT_SEND_NANOS_DUR.nanos::<1, TIMER_2_CLOCK_FREQ>().ticks()
        );
        timer_2.start(BIT_SEND_NANOS_DUR.nanos()).unwrap();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MAIN_CLOCK_FREQ, systick_mono_token);

        (
            Shared {
                timer_2,
                write_req: None,
                read_req: None,
            },
            Local {
                timer_1,
                speed: 1000,
                tmc2209: TMC2209Communicator::new(x_stepper_uart_pin, gpioc.crh),
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

    #[task(binds = TIM1_UP, priority = 3, local = [timer_1, speed, direction: bool = true, overrun: u8 = 0], shared = [write_req, read_req, timer_2])]
    fn send_command_timer(mut cx: send_command_timer::Context) {
        if *cx.local.overrun == 0 {
            *cx.local.overrun = 1;

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

            // IF I REMOVE THIS DEBUG PRINT - DATA SENDING WILL NOT WORK
            // defmt::debug!("curr speed: {}", *cx.local.speed);
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
        } else {
            *cx.local.overrun -= 1;

            cx.shared.read_req.lock(|read_req| {
                if read_req.is_some() {
                    return;
                }
                // IF I REMOVE THIS DEBUG PRINT - DATA RECEIVED WILL BE ZEROES ONLY
                // defmt::debug!("send reading");

                *read_req = Some(tmc2209::read_request::<tmc2209::reg::IFCNT>(0));

                cx.shared.timer_2.lock(|timer_2| {
                    timer_2.listen(Event::Update);
                });
            });
        }

        cx.local.timer_1.clear_interrupt(Event::Update);
    }

    #[task(binds = TIM2, priority = 5, local = [
           tmc2209,
           bytes_to_send:[u8; 8] = [0; 8],
           initialized: bool = false,
           tx_buffer: u16 = 0,
           sending_byte_index: usize = 0
    ], shared = [write_req, read_req, timer_2])]
    fn write_command_task(mut cx: write_command_task::Context) {
        if !*cx.local.initialized {
            cx.shared.write_req.lock(|write_req| {
                if write_req.is_none() {
                    return;
                }

                cx.local.tmc2209.write(write_req.take().unwrap());
            });

            cx.shared.read_req.lock(|read_req| {
                if read_req.is_none() {
                    return;
                }

                cx.local.tmc2209.read(read_req.take().unwrap());
            });

            *cx.local.initialized = true;
        }

        if cx.local.tmc2209.working() {
            cx.local.tmc2209.handle_interrupt();
        } else {
            let (_, opt_response) = cx.local.tmc2209.get_response();
            if opt_response.is_some() {
                defmt::debug!("Response: {:?}", opt_response.unwrap().data_u32());
            }
            cx.shared.timer_2.lock(|timer_2| {
                timer_2.unlisten(Event::Update);
                *cx.local.initialized = false;
            });
        }
    }

    use defmt::Format;
    #[derive(PartialEq, Debug, Format)]
    enum CommunicatorState {
        Nothing,
        Writing,
        Reading,
    }

    struct TMC2209Communicator<const pinC: char, const pinN: u8>
    where
        Pin<pinC, pinN, Dynamic>: HL,
    {
        tx_tick_counter: u8, // interrupt tick counter for TX
        tx_bit_counter: u8,  //TODO: probably I dont need it, cause can use 'tx_bits_buffer == 0' to
        //find the end
        tx_bits_buffer: u16,     // buffer for byte + stop & start bits
        bytes_to_send: [u8; 8],  // buffer for bytes to send
        tx_bytes_counter: usize, // counter for sended bytes

        current_state: CommunicatorState, // should be Sending, Receiving, Nothing
        read_after_write: bool,

        receive_buffer: [u8; MAX_RX_BUFFER_SIZE],
        rx_bit_counter: i8,
        rx_tick_counter: u8,
        rx_buffer: u8,
        receive_buffer_tail: usize,
        receive_buffer_head: usize,
        bytes_to_read: u8,

        pin: Pin<pinC, pinN, Dynamic>,
        cr: <Pin<pinC, pinN, Dynamic> as HL>::Cr,
    }

    impl<const pinC: char, const pinN: u8> TMC2209Communicator<pinC, pinN>
    where
        Pin<pinC, pinN, Dynamic>: HL,
    {
        pub fn new(
            pin: Pin<pinC, pinN, Dynamic>,
            cr: <Pin<pinC, pinN, Dynamic> as HL>::Cr,
        ) -> Self {
            Self {
                tx_tick_counter: 0,
                tx_bit_counter: 0,
                tx_bits_buffer: 0,
                bytes_to_send: [0; 8],
                tx_bytes_counter: 0,

                current_state: CommunicatorState::Nothing,
                read_after_write: false,

                receive_buffer: [0; MAX_RX_BUFFER_SIZE],
                rx_bit_counter: 0,
                rx_tick_counter: 0,
                rx_buffer: 0,
                receive_buffer_tail: 0,
                receive_buffer_head: 0,
                bytes_to_read: 0,

                pin,
                cr,
            }
        }

        pub fn working(&self) -> bool {
            return self.current_state != CommunicatorState::Nothing;
        }

        //TODO: replace read/write with one generic function
        pub fn write(&mut self, req: tmc2209::WriteRequest) {
            if self.current_state != CommunicatorState::Nothing {
                defmt::debug!(
                    "Error, Trying to write when in state: {:?}",
                    self.current_state
                );
                return;
            }
            //TODO: What if we in a process of sending now?

            // cause 'tx_bytes_counter' starts from 'len' we need to reverse
            for (index, elem) in req.bytes().iter().rev().enumerate() {
                self.bytes_to_send[index] = *elem;
            }
            self.tx_bit_counter = 0;
            self.tx_tick_counter = OVERSAMPLE;
            self.tx_bytes_counter = req.bytes().len();
            self.tx_bits_buffer =
                (self.bytes_to_send[self.tx_bytes_counter - 1] as u16) << 1 | 0x200;
            self.current_state = CommunicatorState::Writing;
            self.pin.make_push_pull_output(&mut self.cr);
            self.read_after_write = false;
        }

        //TODO: replace read/write with one generic function
        pub fn read(&mut self, req: tmc2209::ReadRequest) {
            if self.current_state != CommunicatorState::Nothing {
                defmt::debug!(
                    "Error, Trying to read when in state: {:?}",
                    self.current_state
                );
                return;
            }
            //TODO: What if we in a process of sending now?

            // cause 'tx_bytes_counter' starts from 'len' we need to reverse
            for (index, elem) in req.bytes().iter().rev().enumerate() {
                self.bytes_to_send[index] = *elem;
            }
            self.tx_bit_counter = 0;
            self.tx_tick_counter = OVERSAMPLE;
            self.tx_bytes_counter = req.bytes().len();
            self.tx_bits_buffer =
                (self.bytes_to_send[self.tx_bytes_counter - 1] as u16) << 1 | 0x200;
            self.current_state = CommunicatorState::Writing;
            self.pin.make_push_pull_output(&mut self.cr);
            self.read_after_write = true;
        }

        fn send(&mut self) {
            // if tx_tick_counter > 0 interrupt is discarded. Only when tx_tick_counter reach 0 we set TX pin.
            self.tx_tick_counter -= 1;
            if self.tx_tick_counter == 0 {
                self.tx_bit_counter += 1;
                if self.tx_bit_counter <= 10 {
                    // stop, end and 8 bits
                    if self.tx_bits_buffer & 1 == 1 {
                        self.pin.set_high();
                    } else {
                        self.pin.set_low();
                    }
                    self.tx_bits_buffer >>= 1;
                    self.tx_tick_counter = OVERSAMPLE;
                } else {
                    // byte has been sended
                    if self.tx_bytes_counter > 0 {
                        self.tx_bytes_counter -= 1;
                    }

                    if self.tx_bytes_counter == 0 {
                        // all bytes has been sended
                        self.tx_tick_counter = 1;

                        if !self.read_after_write {
                            self.current_state = CommunicatorState::Nothing;
                        } else if self.tx_bit_counter > 10 + OVERSAMPLE * SWITCH_DELAY {
                            self.change_state_to_start_reading(READ_RESPONSE_LENGTH);
                        }
                    } else {
                        // continue to next byte
                        self.tx_bits_buffer =
                            (self.bytes_to_send[self.tx_bytes_counter - 1] as u16) << 1 | 0x200;
                        self.tx_tick_counter = OVERSAMPLE;
                        self.tx_bit_counter = 0;
                    }
                }
            }
        }

        pub fn change_state_to_start_reading(&mut self, bytes_to_read: u8) {
            self.current_state = CommunicatorState::Reading;
            self.rx_bit_counter = -1; // rx_bit_counter = -1 :  waiting for start bit
            self.rx_tick_counter = 2; // 2 : next interrupt will be discarded. 2 interrupts required to consider RX pin level
            self.rx_buffer = 0;
            self.bytes_to_read = bytes_to_read;
            self.pin.make_pull_up_input(&mut self.cr);
        }

        pub fn change_state_to_end_reading(&mut self) {
            defmt::debug!("end reading. current bytes: {:?}", self.receive_buffer);
            self.current_state = CommunicatorState::Nothing;
            self.pin.make_push_pull_output(&mut self.cr);
        }

        fn reading(&mut self) {
            self.rx_tick_counter -= 1;
            if self.rx_tick_counter <= 0 {
                // if rx_tick_counter > 0 interrupt is discarded. Only when rx_tick_counter reach 0 RX pin is considered
                let bit_value = self.pin.is_high().ok().unwrap();
                if self.rx_bit_counter == -1 {
                    // rx_bit_counter = -1 :  waiting for start bit
                    if !bit_value {
                        // defmt::debug!(
                        //     "got start bit. buffer tail: {}, buffer head: {}",
                        //     self.receive_buffer_tail,
                        //     self.receive_buffer_head,
                        // );
                        // got start bit
                        self.rx_bit_counter = 0; // rx_bit_counter == 0 : start bit received
                        self.rx_tick_counter = OVERSAMPLE + 1; // Wait 1 bit (OVERSAMPLE ticks) + 1 tick in order to sample RX pin in the middle of the edge (and not too close to the edge)
                        self.rx_buffer = 0;
                    } else {
                        self.rx_tick_counter = 1; // Waiting for start bit, but we don't get right level. Wait for next Interrupt to ckech RX pin level
                    }
                } else if self.rx_bit_counter >= 8 {
                    // rx_bit_counter >= 8 : waiting for stop bit
                    if bit_value {
                        // stop bit read complete add to buffer
                        let next = (self.receive_buffer_tail + 1) % MAX_RX_BUFFER_SIZE;
                        if next != self.receive_buffer_head {
                            // save new data in buffer: tail points to where byte goes
                            self.receive_buffer[self.receive_buffer_tail] = self.rx_buffer; // save new byte
                            self.receive_buffer_tail = next;
                        } else {
                            // TODO: what to do if overflow?
                            // rx_bit_counter = x  with x = [0..7] correspond to new bit x received
                            defmt::debug!("buffer overflow");
                        }
                    }
                    // Full frame received. Restart waiting for start bit at next interrupt
                    self.rx_tick_counter = 1;
                    self.rx_bit_counter = -1;
                    self.bytes_to_read -= 1;
                    if self.bytes_to_read == 0 {
                        self.change_state_to_end_reading();
                    }
                } else {
                    // defmt::debug!("got data bit; bit counter: {}", self.rx_bit_counter);
                    // data bits
                    self.rx_buffer >>= 1;
                    if bit_value {
                        self.rx_buffer |= 0x80;
                    }
                    self.rx_bit_counter += 1; // Prepare for next bit
                    self.rx_tick_counter = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
                }
            }
        }

        //TODO: this is super ugly and for test purpose only!
        pub fn get_response(&mut self) -> (usize, Option<tmc2209::ReadResponse>) {
            if self.receive_buffer_tail - self.receive_buffer_head > 7 {
                let mut response: [u8; tmc2209::ReadResponse::LEN_BYTES] =
                    [0; tmc2209::ReadResponse::LEN_BYTES];
                for i in 0..tmc2209::ReadResponse::LEN_BYTES {
                    response[i] = self.receive_buffer[self.receive_buffer_head];
                    self.receive_buffer_head = (self.receive_buffer_head + 1) % MAX_RX_BUFFER_SIZE;
                }

                let mut reader = tmc2209::Reader::default();
                let (bytes_read, tmc_response) = reader.read_response(&mut response);

                return (bytes_read, tmc_response);
            }

            return (0, None);
        }

        pub fn handle_interrupt(&mut self) {
            match self.current_state {
                CommunicatorState::Reading => self.reading(),
                CommunicatorState::Writing => self.send(),
                _ => {}
            }
            //TODO:
        }
    }
}
