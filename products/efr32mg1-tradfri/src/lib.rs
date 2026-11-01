//! Product configuration for the EFR32MG1P TRADFRI firmware in three roles:
//! the sleepy environmental sensor (the original image, feature `sed`), an
//! always-on End Device ([`end_device`]) and a Router size probe
//! ([`router`]) that does not fit the part.

#![no_std]

#[cfg(feature = "sed")]
pub mod battery;
#[cfg(feature = "roles")]
pub mod end_device;
#[cfg(feature = "sed")]
pub mod ota;
#[cfg(feature = "sed")]
pub mod policy;
#[cfg(feature = "sed")]
pub mod profile;
#[cfg(feature = "roles")]
pub mod router;
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "EFR32MG1-Sensor";
pub const DATE_CODE: &str = "20260402";

pub const ENDPOINT: u8 = 1;
pub const OTA_MANUFACTURER_CODE: u16 = 0x1049;
pub const OTA_IMAGE_TYPE: u16 = 0x0002;

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
pub const ROUTER_MODEL: &str = "EFR32MG1-Router";
pub const END_DEVICE_MODEL: &str = "EFR32MG1-ED";

/// Mains roles re-arm commissioning on their own this long after a Leave
/// (the button does it immediately); longer than a ZHA permit-join window
/// so a removed device does not rejoin the window that removed it.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;
