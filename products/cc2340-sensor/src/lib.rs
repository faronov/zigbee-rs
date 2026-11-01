//! Product policy for the LP-EM-CC2340R5 Zigbee sensor, plus the mains
//! roles of the role template (feature `roles`): an always-on End Device
//! ([`end_device`]) and a Router size probe ([`router`]) that does not fit
//! the 36 KiB SRAM.
//!
//! The board crate owns physical wiring. This crate owns Zigbee identity,
//! the typed endpoint profile, sleepy-sensor lifecycle policy, the BATMON
//! measurement sources (synthetic humidity until a sensor is fitted), and the
//! protected flash partition used for durable security state.

#![no_std]

pub mod battery;
#[cfg(feature = "roles")]
pub mod end_device;
pub mod environment;
pub mod policy;
pub mod profile;
#[cfg(feature = "roles")]
pub mod router;
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "CC2340-Sensor";
pub const DATE_CODE: &str = "20260402";
pub const SW_BUILD: &str = "0.1.0";
pub const ENDPOINT: u8 = 1;

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
pub const ROUTER_MODEL: &str = "CC2340-Router";
pub const END_DEVICE_MODEL: &str = "CC2340-ED";

/// Mains roles re-arm commissioning on their own this long after a Leave
/// (the button does it immediately); longer than a ZHA permit-join window
/// so a removed device does not rejoin the window that removed it.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;

/// BATMON has not completed its first measurement since reset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatmonNotReady;
