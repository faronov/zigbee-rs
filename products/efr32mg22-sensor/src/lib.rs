//! Product policy for the BRD4183A EFR32MG22 node in three roles: the
//! sleepy environmental sensor (the original image), an always-on End
//! Device ([`end_device`]) and a Router ([`router`]).
//!
//! Physical PB1/PB0/VCOM wiring stays in `efr32mg22-devkit`; this crate
//! selects Zigbee identity, the typed endpoint profile, lifecycle timing,
//! the flash layout and the security-journal opening policy.

#![no_std]

pub mod end_device;
mod journal;
#[cfg(feature = "sed")]
pub mod policy;
pub mod profile;
pub mod router;
pub mod storage;

pub use journal::{
    APPLICATION_END, APPLICATION_START, APS_TABLE_PARTITION_SIZE, APS_TABLE_PARTITION_START,
    CHILD_TABLE_PARTITION_SIZE, CHILD_TABLE_PARTITION_START, PERSISTENCE_PARTITION_END,
    PERSISTENCE_PARTITION_SIZE, PERSISTENCE_PARTITION_START,
};

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "EFR32MG22-Sensor";
pub const DATE_CODE: &str = "20261031";
pub const SW_BUILD: &str = "0.1.0";
pub const APPLICATION_VERSION: u8 = 1;
pub const ENDPOINT: u8 = 1;

/// Security-journal role tags (`SecurityStateJournal::select_role_tag`).
///
/// All role images share one security journal so outgoing frame counters
/// stay monotonic across a reflash; the tag stops one role from resuming
/// another role's network state. The original sensor image predates the
/// tags and writes none, so leave the network before flashing it after a
/// tagged role.
pub mod role {
    pub const ROUTER: u16 = 0x5201;
    pub const END_DEVICE: u16 = 0x5202;
}

/// Model identifiers of the role images.
pub const ROUTER_MODEL: &str = "EFR32MG22-Router";
pub const END_DEVICE_MODEL: &str = "EFR32MG22-ED";

/// Mains roles re-arm commissioning on their own this long after a Leave
/// (BTN0 does it immediately); longer than a ZHA permit-join window so a
/// removed device does not rejoin the window that removed it.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;
