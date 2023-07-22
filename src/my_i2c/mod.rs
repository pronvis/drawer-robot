//! Support for the Inter-Integrated Circuit (I2C) bus peripheral. Also supports SMBUS.
//! Provides APIs to configure, read, and write from
//! I2C, with blocking, nonblocking, and DMA functionality.

use core::ops::Deref;

use cortex_m::interrupt::free;

#[cfg(feature = "embedded_hal")]
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use crate::{
    clocks::Clocks,
    pac::{self, RCC},
    util::RccPeriph,
    MAX_ITERS,
};
use stm32f1xx_hal::rcc::Clocks;

#[cfg(not(any(feature = "l552", feature = "h5")))]
use crate::dma::{self, ChannelCfg, DmaChannel};

#[cfg(any(feature = "f3", feature = "l4"))]
use crate::dma::DmaInput;

#[cfg(feature = "g0")]
use crate::pac::DMA as DMA1;
#[cfg(not(any(feature = "g0", feature = "h5")))]
use crate::pac::DMA1;
