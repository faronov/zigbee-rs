//! Physical wiring and platform resources for PHY62x2 development boards.

#![no_std]

#[cfg(all(feature = "phy6222", feature = "phy6252"))]
compile_error!("select exactly one of the phy6222 or phy6252 features");
#[cfg(not(any(feature = "phy6222", feature = "phy6252")))]
compile_error!("select exactly one of the phy6222 or phy6252 features");

mod flash;
mod pins;
mod resources;
pub mod time;
pub mod vectors;

pub use flash::{INTERNAL_FLASH_CAPACITY, InternalFlash};
pub use phy6222_hal::adc::AdcError;
pub use resources::{
    BoardError, Resources, SensorI2cToken, StatusLed, SupplyMonitor, SupplyMonitorToken, UserButton,
};
