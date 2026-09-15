//! Product configuration for nRF52840 Zigbee sensor firmware.
//!
//! Owns manufacturer/model identity, the flash memory layout and the
//! crash-safe security persistence partitions (`storage`/`uf2_storage`), the
//! battery chemistry policy (`battery`), and the concrete Zigbee profile
//! (`profile`) built from shared `zigbee-runtime` archetypes. Physical board
//! crates provide fitted pin wiring independently of these product choices.

#![no_std]

pub mod battery;
pub mod deployment;
pub mod policy;
pub mod profile;
#[cfg(all(
    target_os = "none",
    not(any(
        feature = "deployment-promicro-uf2",
        feature = "deployment-mdk-uf2",
        feature = "deployment-pca10059-uf2",
        feature = "deployment-dk"
    ))
))]
pub mod storage;
#[cfg(all(
    target_os = "none",
    any(
        feature = "deployment-promicro-uf2",
        feature = "deployment-mdk-uf2",
        feature = "deployment-pca10059-uf2",
        feature = "deployment-dk"
    )
))]
pub mod uf2_storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "nRF52840-Sensor";
pub const DATE_CODE: &str = "20260401";
pub const SW_BUILD: &str = "0.1.0";

pub const ENDPOINT: u8 = 1;
