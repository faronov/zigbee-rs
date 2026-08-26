//! Product definition for the PHY62x2 EVK environmental sensor.
//!
//! The board crate owns fitted wiring and whole-device peripheral resources.
//! This crate owns identity, battery chemistry, endpoint/profile composition,
//! lifecycle policy, linker layout, and the protected durable journal.

#![no_std]

#[cfg(all(feature = "phy6222", feature = "phy6252"))]
compile_error!("select exactly one of the phy6222 or phy6252 features");
#[cfg(not(any(feature = "phy6222", feature = "phy6252")))]
compile_error!("select exactly one of the phy6222 or phy6252 features");

pub mod battery;
pub mod environment;
pub mod identity;
pub mod policy;
pub mod profile;
pub mod storage;

use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "PHY62x2-Sensor";
pub const DATE_CODE: &str = "20260718";
pub const SW_BUILD: &str = "0.2.0";
pub const ENDPOINT: u8 = 1;
pub const CHANNELS: ChannelMask = ChannelMask::ALL_2_4GHZ;
pub const POWER_SOURCE: PowerSource = PowerSource::Battery;
