//! Product configuration for the nRF52833-DK Zigbee node in three roles:
//! the sleepy environmental sensor (the original image), an always-on End
//! Device ([`end_device`]) and a Router ([`router`]).
//!
//! Owns manufacturer/model identity, the flash memory layout and the
//! crash-safe security persistence partition (`storage`), the battery
//! chemistry policy (`battery`), and the concrete Zigbee profile
//! (`profile`) built from shared `zigbee-runtime` archetypes. See
//! `boards/nrf52833-dk` for the physical wiring this product selects. Its
//! composition root runs the shared lifecycle from `apps/sensor-sed`.
//!
//! The only intentional differences from `products/nrf52840-sensor` are the
//! ones the silicon forces:
//!
//! | | nRF52840 product | nRF52833 product |
//! |-|------------------|------------------|
//! | Flash | 1 MiB | 512 KiB |
//! | RAM | 256 KiB | 128 KiB |
//! | Application flash | 1016 KiB | 488 KiB |
//! | Router child/APS journals | — | `0x0007_A000..0x0007_E000` |
//! | Security journal | `0x000F_E000` | `0x0007_E000` |
//! | Model string | `nRF52840-Sensor` | `nRF52833-Sensor` |
//!
//! Endpoint, clusters, reporting defaults, battery curve, and the whole
//! commissioning lifecycle are identical.

#![no_std]

pub mod battery;
#[cfg(feature = "mains")]
pub mod end_device;
#[cfg(all(target_os = "none", feature = "mains"))]
mod partition;
#[cfg(feature = "sed")]
pub mod policy;
pub mod profile;
#[cfg(feature = "mains")]
pub mod router;
#[cfg(target_os = "none")]
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "nRF52833-Sensor";
pub const DATE_CODE: &str = "20260401";
pub const SW_BUILD: &str = "0.1.0";

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
pub const ROUTER_MODEL: &str = "nRF52833-Router";
pub const END_DEVICE_MODEL: &str = "nRF52833-ED";

/// Mains roles re-arm commissioning on their own this long after a Leave
/// (Button 1 does it immediately); longer than a ZHA permit-join window so
/// a removed device does not rejoin the window that removed it.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;
