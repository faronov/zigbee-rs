//! Product definition for the PHY62x2 EVK environmental sensor, plus the
//! mains roles of the role template (feature `roles`): an always-on End
//! Device ([`end_device`]) and a Router size probe ([`router`]).
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
#[cfg(feature = "roles")]
pub mod end_device;
pub mod environment;
pub mod identity;
pub mod policy;
pub mod profile;
#[cfg(feature = "roles")]
pub mod router;
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

/// Security-journal role tags (`SecurityStateJournal::select_role_tag`).
///
/// All role images share one security journal so outgoing frame counters
/// stay monotonic across a reflash; the tag stops one role from resuming
/// another role's network state. The sleepy sensor predates the tags and
/// writes none, so leave the network before flashing it after a tagged
/// role.
pub mod role {
    pub const ROUTER: u16 = 0x5201;
    pub const END_DEVICE: u16 = 0x5202;
}

/// Model identifiers of the role images.
pub const ROUTER_MODEL: &str = "PHY62x2-Router";
pub const END_DEVICE_MODEL: &str = "PHY62x2-ED";

/// Mains roles re-arm commissioning on their own this long after a Leave
/// (the button does it immediately); longer than a ZHA permit-join window
/// so a removed device does not rejoin the window that removed it.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;
