use super::TensionData;

#[derive(Default)]
struct Arm {
    tension: i32,
    desired_tension: Option<i32>,
}

impl Arm {
    pub fn set_desired_tension(&mut self) {
        self.desired_tension = Some(self.tension);
    }

    pub fn remove_desired_tension(&mut self) {
        self.desired_tension = None;
    }

    pub fn get_speed(&self) -> u32 {
        //test code
        self.tension as u32 * 10
    }
}

#[derive(Default)]
pub struct RobotArms {
    arm0: Arm,
    arm1: Arm,
    arm2: Arm,
    arm3: Arm,
}

pub struct ArmsSpeed {
    pub s0: u32,
    pub s1: u32,
    pub s2: u32,
    pub s3: u32,
}

impl RobotArms {
    pub fn set_tension(&mut self, tension_data: TensionData) {
        if tension_data.t0 != crate::DEFAULT_DYMH06_VALUE {
            self.arm0.tension = tension_data.t0;
        }
        if tension_data.t1 != crate::DEFAULT_DYMH06_VALUE {
            self.arm1.tension = tension_data.t1;
        }
        if tension_data.t2 != crate::DEFAULT_DYMH06_VALUE {
            self.arm2.tension = tension_data.t2;
        }
        if tension_data.t3 != crate::DEFAULT_DYMH06_VALUE {
            self.arm3.tension = tension_data.t3;
        }
    }

    pub fn set_desired_tension(&mut self) {
        self.arm0.set_desired_tension();
        self.arm1.set_desired_tension();
        self.arm2.set_desired_tension();
        self.arm3.set_desired_tension();
    }

    pub fn remove_desired_tension(&mut self) {
        self.arm0.remove_desired_tension();
        self.arm1.remove_desired_tension();
        self.arm2.remove_desired_tension();
        self.arm3.remove_desired_tension();
    }

    pub fn get_speed(&self) -> ArmsSpeed {
        ArmsSpeed {
            s0: self.arm0.get_speed(),
            s1: self.arm1.get_speed(),
            s2: self.arm2.get_speed(),
            s3: self.arm3.get_speed(),
        }
    }
}
