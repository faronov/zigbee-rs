//! Product definition for the Ai-Thinker PB-03F-Kit Zigbee demo.
//!
//! The kit has no sensor, so this bring-up product joins as a non-sleeping
//! end device and publishes a synthetic temperature/humidity series; its
//! purpose is to prove the pure-Rust PHY6252 radio and the Zigbee stack end
//! to end (join, interview, reporting, persistence) on real hardware.
//!
//! The board crate owns fitted wiring; this crate owns identity, the radio
//! package/PA selection, endpoint/profile composition, lifecycle policy,
//! linker layout and the protected durable journal.

#![no_std]

pub mod battery;
pub mod environment;
pub mod identity;
pub mod policy;
pub mod profile;
pub mod radio;
pub mod storage;

use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "PB03F-Kit-Demo";
pub const DATE_CODE: &str = "20261030";
pub const SW_BUILD: &str = "0.1.0";
pub const ENDPOINT: u8 = 1;
pub const CHANNELS: ChannelMask = ChannelMask::ALL_2_4GHZ;
/// The kit is powered from USB (CH340 port) through an LDO.
pub const POWER_SOURCE: PowerSource = PowerSource::DcSource;
