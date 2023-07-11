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
}

// My choice: 1/2 step'a
