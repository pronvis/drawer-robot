use stm32f1xx_hal::gpio::{Dynamic, Pin};
use stm32f1xx_hal::pac::{self, gpioa::RegisterBlock};

/// Changing pin mode via stm32f1xx_hal::gpio::make_pull_up_input (for example) requires
/// 'cr: &mut <Self as HL>::Cr' which is non-clonable, so I cant create several TMC2209SerialCommunicator
/// (each one requires 'cr').
/// In order to avoid that I implemented manual pin mode changing functions that directly calls
/// PAC. It requires critical session when modify, cause it consists of Read, Modify, Write
/// operations.
pub trait ManualPin<const PIN_C: char, const PIN_N: u8> {
    fn change_mode_to_pull_up_input(&self);
    fn get_register_block(&self) -> *const RegisterBlock;
    fn change_mode_to_output_push_pull(&self);
    fn change_mode(&self, cnf: u32, mode: u32, register_block: *const RegisterBlock);
    fn set_high(&self);
    fn set_low(&self);
    fn is_low(&self) -> bool;
    fn is_high(&self) -> bool;
}

impl<const PIN_C: char, const PIN_N: u8> ManualPin<PIN_C, PIN_N> for Pin<PIN_C, PIN_N, Dynamic> {
    fn change_mode_to_pull_up_input(&self) {
        let register_block = Self::get_register_block(&self);
        // for Output<PushPull>
        let cnf = 0b10;
        let mode: u32 = 0b00;

        //Input<PullUp> requires changing 'Port bit set/reset register'(bsrr)
        unsafe {
            (*register_block).bsrr.write(|w| w.bits(1 << PIN_N));
        }
        self.change_mode(cnf, mode, register_block);
    }

    fn change_mode_to_output_push_pull(&self) {
        // for Output<PushPull>
        let cnf = 0b00;
        let mode: u32 = 0b11;
        self.change_mode(cnf, mode, self.get_register_block());
    }

    fn get_register_block(&self) -> *const RegisterBlock {
        match PIN_C {
            'A' => pac::GPIOA::ptr(),
            'B' => pac::GPIOB::ptr(),
            'C' => pac::GPIOC::ptr(),
            'D' => pac::GPIOD::ptr(),
            'E' => pac::GPIOE::ptr(),
            _ => unreachable!(),
        }
    }

    fn change_mode(&self, cnf: u32, mode: u32, register_block: *const RegisterBlock) {
        let offset: u32 = (4 * (PIN_N as u32)) % 32;
        let bits = (cnf << 2) | mode;
        critical_section::with(|_cs| unsafe {
            match PIN_N {
                0..=7 => {
                    (*register_block)
                        .crl
                        .modify(|r, w| w.bits((r.bits() & !(0b1111 << offset)) | (bits << offset)));
                }
                8..=15 => {
                    (*register_block)
                        .crh
                        .modify(|r, w| w.bits((r.bits() & !(0b1111 << offset)) | (bits << offset)));
                }
                _ => unreachable!(),
            };
        });
    }

    fn set_high(&self) {
        let register_block = self.get_register_block();
        unsafe {
            (*register_block).bsrr.write(|w| w.bits(1 << PIN_N));
        }
    }

    fn set_low(&self) {
        let register_block = self.get_register_block();
        unsafe {
            (*register_block).bsrr.write(|w| w.bits(1 << (16 + PIN_N)));
        }
    }

    fn is_high(&self) -> bool {
        !self.is_low()
    }

    fn is_low(&self) -> bool {
        let register_block = self.get_register_block();
        unsafe { (*register_block).idr.read().bits() & (1 << PIN_N) == 0 }
    }
}
