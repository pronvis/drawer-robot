use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
use rtic_sync::channel::*;
use stm32f1xx_hal::timer::{Counter, Event, Instance};

pub const CHANNEL_CAPACITY: usize = 1;
// I use 1/2 microstep mode, so min speed = 200
// with delay = 200 speed of a thread on a coil with diameter 40mm = 7.85 mm/s
pub const MIN_DELAY_BETWEEN_STEPS: u32 = 200;
// with delay = 1500 speed of a thread on a coil with diameter 40mm = 1.84 mm/s
pub const MAX_DELAY_BETWEEN_STEPS: u32 = 1500;
pub const DELAY_BETWEEN_STEPS_STEP: u32 = 100;
pub const MAX_SPEED_VAL: u8 =
    ((MAX_DELAY_BETWEEN_STEPS - MIN_DELAY_BETWEEN_STEPS) / DELAY_BETWEEN_STEPS_STEP) as u8;

#[derive(Clone)]
pub struct MyStepperState {
    //Regulate Speed with that value.
    //min value that works as fast as possible:
    // 400 micros for full step
    // 200 micros for 1/2 and smaller steps
    //max value that works smooth: 1500 micros
    pub micros_between_steps: u32,

    //min value that works as fast as possible: 1 micros
    //max value that works smooth: 900 micros
    pub micros_pulse_duration: u32,

    step_phase: bool,
}
impl MyStepperState {
    // TODO: change constructor - caller dont know about micros.
    pub fn new(micros_between_steps: u32, micros_pulse_duration: u32) -> Self {
        Self {
            micros_between_steps,
            micros_pulse_duration,
            step_phase: false,
        }
    }
}

#[derive(Debug, Clone)]
pub enum MyStepperCommands {
    Stay,
    Move(u8),
}

pub struct MyStepper<TIM, STEP_PIN, const FREQ: u32> {
    index: u8, // need for logging
    state: MyStepperState,
    timer: Counter<TIM, FREQ>,
    step_pin: STEP_PIN,
    commands_channel: Receiver<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    is_moving: bool, //TODO: move to state
}

impl<TIM: Instance, STEP_PIN: OutputPin, const FREQ: u32> MyStepper<TIM, STEP_PIN, FREQ> {
    pub fn new(
        index: u8,
        state: MyStepperState,
        timer: Counter<TIM, FREQ>,
        step_pin: STEP_PIN,
        commands_channel: Receiver<'static, MyStepperCommands, CHANNEL_CAPACITY>,
    ) -> Self {
        Self {
            index,
            state,
            timer,
            step_pin,
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
            return;
        }

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

            self.timer.clear_interrupt(Event::Update);
        }
    }

    fn handle_command(&mut self, command: MyStepperCommands) {
        match command {
            MyStepperCommands::Stay => self.is_moving = false,
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
    let new_delays: u32 = MIN_DELAY_BETWEEN_STEPS + DELAY_BETWEEN_STEPS_STEP * u32::from(new_speed);
    new_delays
}
