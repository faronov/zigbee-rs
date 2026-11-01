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
//!
//! Feature `roles` adds the mains roles of the role template: an always-on
//! (receiver-on-when-idle) End Device ([`end_device`]) and a Router size
//! probe ([`router`]).

#![no_std]

pub mod battery;
#[cfg(feature = "roles")]
pub mod end_device;
pub mod environment;
pub mod identity;
pub mod policy;
pub mod profile;
pub mod radio;
#[cfg(feature = "roles")]
pub mod router;
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

/// Security-journal role tags (`SecurityStateJournal::select_role_tag`).
///
/// All role images share one security journal so outgoing frame counters
/// stay monotonic across a reflash; the tag stops one role from resuming
/// another role's network state. The polling `pb03f-zigbee` image predates
/// the tags and writes none, so leave the network before flashing it after
/// a tagged role.
pub mod role {
    pub const ROUTER: u16 = 0x5201;
    pub const END_DEVICE: u16 = 0x5202;
}

/// Model identifiers of the role images.
pub const ROUTER_MODEL: &str = "PB03F-Kit-Router";
pub const END_DEVICE_MODEL: &str = "PB03F-Kit-ED";

/// Mains roles re-arm commissioning on their own this long after a Leave
/// (the button does it immediately); longer than a ZHA permit-join window
/// so a removed device does not rejoin the window that removed it.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;
