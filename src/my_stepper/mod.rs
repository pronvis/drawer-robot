use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
use rtic_sync::{channel::*, make_channel};
use stm32f1xx_hal::{
    device::{TIM2, TIM3, TIM4, TIM5},
    gpio,
    gpio::PinState,
    rcc::Clocks,
    timer::{Counter, Event, Instance},
};

use crate::CHANNEL_CAPACITY;
use crate::{EDirPin, EStepPin, XDirPin, XStepPin, YDirPin, YStepPin, ZDirPin, ZStepPin};

const TIMER_CLOCK_FREQ: u32 = 10_000;
// step = 100_000 nanos, 100 micros
// I use 1/2 microstep mode, so min speed = 200
// with delay = 200 speed of a thread on a coil with diameter 40mm = 7.85 mm/s
pub const MIN_DELAY_BETWEEN_STEPS: u32 = 200;
// with delay = 1500 speed of a thread on a coil with diameter 40mm = 1.84 mm/s
pub const MAX_DELAY_BETWEEN_STEPS: u32 = 1500;
pub const DELAY_BETWEEN_STEPS_STEP: u32 = 100;
pub const MAX_SPEED_VAL: u8 =
    ((MAX_DELAY_BETWEEN_STEPS - MIN_DELAY_BETWEEN_STEPS) / DELAY_BETWEEN_STEPS_STEP) as u8;

pub type MyStepper1 = MyStepper<stm32f1xx_hal::pac::TIM2, XStepPin, XDirPin>;
pub type MyStepper2 = MyStepper<stm32f1xx_hal::pac::TIM3, YStepPin, YDirPin>;
pub type MyStepper3 = MyStepper<stm32f1xx_hal::pac::TIM4, ZStepPin, ZDirPin>;
pub type MyStepper4 = MyStepper<stm32f1xx_hal::pac::TIM5, EStepPin, EDirPin>;

#[derive(Clone)]
pub struct MyStepperState {
    //Regulate Speed with that value.
    //min value that works as fast as possible:
    // 400 micros for full step
    // 200 micros for 1/2 and smaller steps
    //max value that works smooth: 1500 micros
    micros_between_steps: u32,

    //min value that works as fast as possible: 1 micros
    //max value that works smooth: 900 micros
    micros_pulse_duration: u32,

    step_phase: bool,
}
impl MyStepperState {
    pub fn new() -> Self {
        Self {
            micros_between_steps: MAX_DELAY_BETWEEN_STEPS,
            micros_pulse_duration: 200,
            step_phase: false,
        }
    }

    pub fn micros_pulse_duration(&self) -> u32 {
        self.micros_pulse_duration
    }
}

#[derive(Debug, Clone)]
pub enum MyStepperCommands {
    Stay,
    Direction(bool),
    Move(u8),
}

pub struct MyStepper<Tim, StepPin, DirPin> {
    index: u8, // need for logging
    state: MyStepperState,
    timer: Counter<Tim, TIMER_CLOCK_FREQ>,
    step_pin: StepPin,
    direction_pin: DirPin,
    commands_channel: Receiver<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    is_moving: bool, //TODO: move to state
}

impl<Tim: Instance, StepPin: OutputPin, DirPin: OutputPin> MyStepper<Tim, StepPin, DirPin> {
    pub fn new(
        index: u8,
        state: MyStepperState,
        timer: Counter<Tim, TIMER_CLOCK_FREQ>,
        step_pin: StepPin,
        dir_pin: DirPin,
        commands_channel: Receiver<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    ) -> Self {
        Self {
            index,
            state,
            timer,
            step_pin,
            direction_pin: dir_pin,
            commands_channel,
            is_moving: false,
        }
    }

    pub fn work(&mut self) {
        match self.commands_channel.try_recv() {
            Err(err) => match err {
                ReceiveError::NoSender => {
                    defmt::error!("stepper #{}: commands sender dropped", self.index);
                    self.is_moving = false;
                }
                ReceiveError::Empty => (),
            },

            Ok(new_command) => self.handle_command(new_command),
        }

        if !self.is_moving {
            self.timer.clear_interrupt(Event::Update);
            return;
        }

        // self.timer.start have 'clear_interrupt' inside
        if self.state.step_phase {
            self.step_pin.set_low().ok();
            self.state.step_phase = false;
            self.timer
                .start(self.state.micros_between_steps.micros())
                .unwrap(); //TODO: unwrap
        } else {
            self.step_pin.set_high().ok();
            self.state.step_phase = true;
            self.timer
                .start(self.state.micros_pulse_duration.micros())
                .unwrap(); //TODO: unwrap
        }
    }

    fn handle_command(&mut self, command: MyStepperCommands) {
        match command {
            MyStepperCommands::Stay => self.is_moving = false,
            MyStepperCommands::Direction(is_right) => {
                if is_right {
                    self.direction_pin.set_low().ok();
                } else {
                    self.direction_pin.set_high().ok();
                }
            }
            MyStepperCommands::Move(speed) => {
                self.is_moving = true;
                self.state.micros_between_steps = speed_to_delays(speed);
                defmt::debug!(
                    "stepper #{}: update micros_between_steps = {}",
                    self.index,
                    self.state.micros_between_steps
                );
            }
        }
    }
}

fn speed_to_delays(speed: u8) -> u32 {
    let new_speed = core::cmp::min(speed, MAX_SPEED_VAL);
    let new_delays: u32 =
        MIN_DELAY_BETWEEN_STEPS + DELAY_BETWEEN_STEPS_STEP * u32::from(MAX_SPEED_VAL - new_speed);
    new_delays
}

pub fn create_steppers(
    clocks: &Clocks,
    pc5: gpio::PC5,
    pc6: gpio::PC6,
    pc7: gpio::PC7,
    pb0: gpio::PB0,
    pb1: gpio::PB1,
    pb2: gpio::PB2,
    pb10: gpio::PB10,
    pb11: gpio::PB11,
    pb12: gpio::PB12,
    pb13: gpio::PB13,
    pb14: gpio::PB14,
    pb15: gpio::PB15,
    gpioc_crl: &mut gpio::Cr<'C', false>,
    gpiob_crl: &mut gpio::Cr<'B', false>,
    gpiob_crh: &mut gpio::Cr<'B', true>,
    tim2: TIM2,
    tim3: TIM3,
    tim4: TIM4,
    tim5: TIM5,
) -> (
    (
        MyStepper1,
        Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    ),
    (
        MyStepper2,
        Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    ),
    (
        MyStepper3,
        Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    ),
    (
        MyStepper4,
        Sender<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    ),
) {
    let stepper_state = MyStepperState::new();

    let x_step_pin = pc6.into_push_pull_output_with_state(gpioc_crl, PinState::Low);
    let x_dir_pin = pb15.into_push_pull_output_with_state(gpiob_crh, PinState::Low);
    let y_step_pin = pb13.into_push_pull_output_with_state(gpiob_crh, PinState::Low);
    let y_dir_pin = pb12.into_push_pull_output_with_state(gpiob_crh, PinState::Low);
    let z_step_pin = pb10.into_push_pull_output_with_state(gpiob_crh, PinState::Low);
    let z_dir_pin = pb2.into_push_pull_output_with_state(gpiob_crl, PinState::Low);
    let e_step_pin = pb0.into_push_pull_output_with_state(gpiob_crl, PinState::Low);
    let e_dir_pin = pc5.into_push_pull_output_with_state(gpioc_crl, PinState::Low);

    let mut x_en = pc7.into_push_pull_output(gpioc_crl);
    x_en.set_low();

    let mut y_en = pb14.into_push_pull_output(gpiob_crh);
    y_en.set_low();

    let mut z_en = pb11.into_push_pull_output(gpiob_crh);
    z_en.set_low();

    let mut e_en = pb1.into_push_pull_output(gpiob_crl);
    e_en.set_low();

    let tim2 = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ>::new(
        tim2, clocks,
    );
    let mut timer_2: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM2, TIMER_CLOCK_FREQ> =
        tim2.counter();
    timer_2
        .start(stepper_state.micros_pulse_duration().micros())
        .unwrap();
    timer_2.listen(Event::Update);

    let tim3 = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM3, TIMER_CLOCK_FREQ>::new(
        tim3, clocks,
    );
    let mut timer_3: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM3, TIMER_CLOCK_FREQ> =
        tim3.counter();
    timer_3
        .start(stepper_state.micros_pulse_duration().micros())
        .unwrap();
    timer_3.listen(Event::Update);

    let tim4 = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM4, TIMER_CLOCK_FREQ>::new(
        tim4, clocks,
    );
    let mut timer_4: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM4, TIMER_CLOCK_FREQ> =
        tim4.counter();
    timer_4
        .start(stepper_state.micros_pulse_duration().micros())
        .unwrap();
    timer_4.listen(Event::Update);

    let tim5 = stm32f1xx_hal::timer::FTimer::<stm32f1xx_hal::pac::TIM5, TIMER_CLOCK_FREQ>::new(
        tim5, clocks,
    );
    let mut timer_5: stm32f1xx_hal::timer::Counter<stm32f1xx_hal::pac::TIM5, TIMER_CLOCK_FREQ> =
        tim5.counter();
    timer_5
        .start(stepper_state.micros_pulse_duration().micros())
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

    let stepper_1 = MyStepper::new(
        1,
        stepper_state.clone(),
        timer_2,
        x_step_pin,
        x_dir_pin,
        stepper_1_commands_receiver,
    );
    let stepper_2 = MyStepper::new(
        2,
        stepper_state.clone(),
        timer_3,
        y_step_pin,
        y_dir_pin,
        stepper_2_commands_receiver,
    );
    let stepper_3 = MyStepper::new(
        3,
        stepper_state.clone(),
        timer_4,
        z_step_pin,
        z_dir_pin,
        stepper_3_commands_receiver,
    );
    let stepper_4 = MyStepper::new(
        4,
        stepper_state,
        timer_5,
        e_step_pin,
        e_dir_pin,
        stepper_4_commands_receiver,
    );

    (
        (stepper_1, sender_1),
        (stepper_2, sender_2),
        (stepper_3, sender_3),
        (stepper_4, sender_4),
    )
}
