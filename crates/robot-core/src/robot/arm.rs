use super::TensionData;
use crate::{i32_diff, tension_to_speed};

#[derive(Eq, PartialEq)]
enum ArmMode {
    ByTension,
    Free,
}

impl Default for ArmMode {
    fn default() -> Self {
        ArmMode::ByTension
    }
}

#[derive(Default)]
struct Arm {
    tension: i32,
    desired_tension: Option<i32>,
    last_sended_tension: Option<i32>,
    mode: ArmMode,
}

impl Arm {
    pub fn set_desired_tension(&mut self) {
        self.desired_tension = Some(self.tension);
    }

    pub fn remove_desired_tension(&mut self) {
        self.desired_tension = None;
    }

    pub fn get_speed(&mut self) -> Option<u32> {
        if self.mode == ArmMode::Free {
            return None;
        }

        if let Some(last_sended_tension) = self.last_sended_tension {
            if last_sended_tension == self.tension {
                return None;
            }
        }

        self.last_sended_tension = Some(self.tension);
        return Some(tension_to_speed(self.tension + self.desired_tension.unwrap_or(0)));
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
    pub s0: Option<u32>,
    pub s1: Option<u32>,
    pub s2: Option<u32>,
    pub s3: Option<u32>,
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
        self.arm0.mode = ArmMode::ByTension;
        self.arm1.mode = ArmMode::ByTension;
        self.arm2.mode = ArmMode::ByTension;
        self.arm3.mode = ArmMode::ByTension;
        self.arm0.set_desired_tension();
        self.arm1.set_desired_tension();
        self.arm2.set_desired_tension();
        self.arm3.set_desired_tension();
    }

    pub fn set_free_tenstion(&mut self) {
        self.arm0.mode = ArmMode::Free;
        self.arm1.mode = ArmMode::Free;
        self.arm2.mode = ArmMode::Free;
        self.arm3.mode = ArmMode::Free;
    }

    pub fn get_speed(&mut self) -> ArmsSpeed {
        ArmsSpeed {
            s0: self.arm0.get_speed(),
            s1: self.arm1.get_speed(),
            s2: self.arm2.get_speed(),
            s3: self.arm3.get_speed(),
        }
    }
}
