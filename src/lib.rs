#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger
use rtic_monotonics::systick::*;
use stm32f1xx_hal::{
    adc::{self, Adc},
    device::ADC1,
    prelude::*,
};

pub mod my_stepper;

use panic_probe as _;

use stm32f1xx_hal as _; // memory layout

// BLUE PILL PINS
// pub type EnPin = stm32f1xx_hal::gpio::Pin<'C', 15, stm32f1xx_hal::gpio::Output>;
// pub type StepPin = stm32f1xx_hal::gpio::Pin<'C', 14, stm32f1xx_hal::gpio::Output>;
// pub type DirPin = stm32f1xx_hal::gpio::Pin<'C', 13, stm32f1xx_hal::gpio::Output>;
// pub type InternalLed = stm32f1xx_hal::gpio::Pin<'C', 13, stm32f1xx_hal::gpio::Output>;
// pub type OutLed = stm32f1xx_hal::gpio::Pin<'B', 12, stm32f1xx_hal::gpio::Output>;

// BigTreeTech SKR E3-DIP v1.1
// X-PINS
pub type EnPin = stm32f1xx_hal::gpio::Pin<'C', 7, stm32f1xx_hal::gpio::Output>;
pub type StepPin = stm32f1xx_hal::gpio::Pin<'C', 6, stm32f1xx_hal::gpio::Output>;
pub type DirPin = stm32f1xx_hal::gpio::Pin<'B', 15, stm32f1xx_hal::gpio::Output>;
pub type InternalLed = stm32f1xx_hal::gpio::Pin<'C', 13, stm32f1xx_hal::gpio::Output>; // no led there
pub type OutLed = stm32f1xx_hal::gpio::Pin<'B', 12, stm32f1xx_hal::gpio::Output>;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub async fn read_potent(
    adc1: &mut Adc<ADC1>,
    pb0: &mut stm32f1xx_hal::gpio::Pin<'B', 0, stm32f1xx_hal::gpio::Analog>,
) -> u16 {
    //sum 10 measurments - max value 4.1k, so 41k which can be still stored in u16
    let mut sum: u16 = 0;
    for _ in 0..10 {
        let data: u16 = adc1.read(pb0).unwrap();
        sum += data;
        Systick::delay(10.millis()).await;
    }

    return sum / 10;
}

pub fn calc_steps_delay(potent_value: u16) -> u32 {
    let min_delay: u32 = 700_000;
    let potent_shift: u32 = u32::from(potent_value) / 100 * 30_0000;
    return min_delay + potent_shift;
}
