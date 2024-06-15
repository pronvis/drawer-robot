#![no_std]

pub mod display;
pub mod my_stepper;
pub mod my_tmc2209;
pub mod ps3;
pub mod robot;

use core::sync::atomic::{AtomicUsize, Ordering};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, Ssd1306};
use stm32f1xx_hal::{
    gpio::gpiob::{PB0, PB1, PB10, PB11, PB12, PB13, PB15, PB2, PB8, PB9},
    gpio::gpioc::{PC13, PC5, PC6, PC7},
    gpio::{Alternate, OpenDrain, Output, Pin},
    i2c::BlockingI2c,
    pac::I2C1,
    rcc::Clocks,
    timer::{self, Counter},
};

use heapless::{pool, pool::singleton::Pool};

use panic_probe as _;

use stm32f1xx_hal as _; // memory layout

// Declare a pool of 24-byte memory blocks
// actually we need only 21 bytes (oled display size limit),
// but "word size" is 32, so 32 is a minimum. But we need
// to store additional info, such as `vec: usize` in String.
// And 4 bytes somewhere else, probably to link: Box<DisplayMemoryPool>
// when send String into channel.
pub type DisplayString = heapless::String<24>;
pool!(DisplayMemoryPool: DisplayString);

// BLUE PILL PINS
// pub type EnPin = stm32f1xx_hal::gpio::Pin<'C', 15, stm32f1xx_hal::gpio::Output>;
// pub type StepPin = stm32f1xx_hal::gpio::Pin<'C', 14, stm32f1xx_hal::gpio::Output>;
// pub type DirPin = stm32f1xx_hal::gpio::Pin<'C', 13, stm32f1xx_hal::gpio::Output>;
// pub type InternalLed = stm32f1xx_hal::gpio::Pin<'C', 13, stm32f1xx_hal::gpio::Output>;
// pub type OutLed = stm32f1xx_hal::gpio::Pin<'B', 12, stm32f1xx_hal::gpio::Output>;

// BigTreeTech SKR E3-DIP v1.1
// X-PINS

pub type Scl1Pin = PB8<Alternate<OpenDrain>>;
pub type Sda1Pin = PB9<Alternate<OpenDrain>>;
pub type Ssd1306Display =
    Ssd1306<I2CInterface<BlockingI2c<I2C1, (Scl1Pin, Sda1Pin)>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

pub type XEnPin = PC7<Output>;
pub type XStepPin = PC6<Output>;
pub type XDirPin = PB15<Output>;

pub type YEnPin = Pin<'B', 16, Output>;
pub type YStepPin = PB13<Output>;
pub type YDirPin = PB12<Output>;

pub type ZEnPin = PB11<Output>;
pub type ZStepPin = PB10<Output>;
pub type ZDirPin = PB2<Output>;

pub type EEnPin = PB1<Output>;
pub type EStepPin = PB0<Output>;
pub type EDirPin = PC5<Output>;

//For ./bin files
pub type EnPin = XEnPin;
pub type StepPin = XStepPin;
pub type DirPin = XDirPin;
pub type InternalLed = PC13<Output>; // no led there
pub type OutLed = PB12<Output>;

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

pub fn get_counter<TIM: timer::Instance, const FREQ: u32>(tim: TIM, clocks: &Clocks) -> Counter<TIM, FREQ> {
    let timer = stm32f1xx_hal::timer::FTimer::<TIM, FREQ>::new(tim, clocks);
    let counter: stm32f1xx_hal::timer::Counter<TIM, FREQ> = timer.counter();
    return counter;
}

pub const COMPANION_SYNC: u8 = 0b01000101;
#[derive(Default, Copy, Clone)]
pub struct CompanionMessage {
    pub load_sensor_0: i32,
    pub load_sensor_1: i32,
    pub load_sensor_2: i32,
    pub load_sensor_3: i32,
}

const TENSOR_CONVERSION_RATE: f32 = 0.035274;
pub fn tensor_to_kg(data: i32) -> f32 {
    if data == i32::MIN {
        return f32::MIN;
    }

    let gramms = data as f32 * TENSOR_CONVERSION_RATE;
    let kilogramms = gramms / 1000f32;
    return kilogramms;
}

pub fn f32_diff(f1: f32, f2: f32) -> f32 {
    if f2 > f1 {
        f2 - f1
    } else {
        f1 - f2
    }
}

pub fn i32_diff(i1: i32, i2: i32) -> i32 {
    if i2 > i1 {
        i2 - i1
    } else {
        i1 - i2
    }
}
