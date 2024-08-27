use super::TensionData;
use crate::tension_to_speed;

#[derive(Eq, PartialEq)]
enum ArmMode {
    ByTension,
    Free,
}

impl Default for ArmMode {
    fn default() -> Self {
        ArmMode::Free
    }
}

#[derive(Default)]
pub struct Arm {
    tension: i32,
    last_sended: Option<u32>,
    mode: ArmMode,
    free_mode_speed: u32,
}

impl Arm {
    pub fn set_desired_tension(&mut self) {
        self.mode = ArmMode::ByTension;
    }

    pub fn increase_speed(&mut self, speed: u32) {
        self.free_mode_speed += speed;
    }

    pub fn decrease_speed(&mut self, speed: u32) {
        self.free_mode_speed -= speed;
    }

    pub fn stop(&mut self) {
        self.free_mode_speed = 0;
    }

    pub fn get_speed(&mut self, arm_index: u8) -> Option<u32> {
        match self.mode {
            ArmMode::Free => {
                if self.check_and_update_last_sended(self.free_mode_speed) {
                    return None;
                }

                return Some(self.free_mode_speed);
            }
            ArmMode::ByTension => {
                if self.check_and_update_last_sended(self.tension as u32) {
                    return None;
                }

                let desired_tension = Self::desired_tension_by_arm(arm_index);
                let speed = tension_to_speed(desired_tension - self.tension);
                return Some(speed);
            }
        }
    }

    pub fn check_and_update_last_sended(&mut self, value: u32) -> bool {
        if let Some(last_sended) = self.last_sended {
            if last_sended == value {
                return true;
            }
        }

        self.last_sended = Some(value);
        return false;
    }

    pub fn desired_tension_by_arm(arm_index: u8) -> i32 {
        match arm_index {
            0 => 8_000,
            1 => 16_000,
            2 => 11_000,
            3 => 15_000,
            _ => 0,
        }
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

    pub fn set_by_tenstion_mode(&mut self) {
        self.arm0.set_desired_tension();
        self.arm1.set_desired_tension();
        self.arm2.set_desired_tension();
        self.arm3.set_desired_tension();
    }

    pub fn set_free_tenstion_mode(&mut self) {
        self.arm0.mode = ArmMode::Free;
        self.arm1.mode = ArmMode::Free;
        self.arm2.mode = ArmMode::Free;
        self.arm3.mode = ArmMode::Free;
    }

    pub fn get_speed(&mut self) -> ArmsSpeed {
        ArmsSpeed {
            s0: self.arm0.get_speed(0),
            s1: self.arm1.get_speed(1),
            s2: self.arm2.get_speed(2),
            s3: self.arm3.get_speed(3),
        }
    }

    pub fn get_arm(&mut self, i: u8) -> Option<&mut Arm> {
        match i {
            0 => Some(&mut self.arm0),
            1 => Some(&mut self.arm1),
            2 => Some(&mut self.arm2),
            3 => Some(&mut self.arm3),
            _ => None,
        }
    }
}
