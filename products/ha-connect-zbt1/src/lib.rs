//! Product policy for the Home Assistant Connect ZBT-1 as an always-on
//! (mains-powered, RX-on-when-idle) Zigbee End Device.
//!
//! The ZBT-1 is a USB dongle with no sensor, LED or button wired to the
//! EFR32MG21, so the product is a Basic + Identify endpoint whose value is
//! proving the EFR32 Series 2 radio and the shared runtime against a real
//! coordinator. It is an End Device and not a Router: the shared
//! `SoftMacCore` has no parent (coordinator/router) MAC role yet.
//!
//! Physical wiring stays in `ha-connect-zbt1`; this crate selects the Zigbee
//! identity, the endpoint profile, the bounded lifecycle policy, the default
//! TX power and the persistence partition at the top of the 1 MiB flash.

#![no_std]

mod journal;
pub mod policy;
pub mod profile;
pub mod storage;

pub use journal::{
    APPLICATION_END, APPLICATION_START, PERSISTENCE_PARTITION_END, PERSISTENCE_PARTITION_SIZE,
    PERSISTENCE_PARTITION_START,
};

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "ZBT-1-AlwaysOn-ED";
pub const DATE_CODE: &str = "20261101";
pub const SW_BUILD: &str = "0.1.0";
pub const ENDPOINT: u8 = 1;

/// Default `phyTransmitPower`. The EFR32MG21A020 PA can do +20 dBm, but the
/// certified limit of the ZBT-1 board is not documented (the Nabu Casa
/// firmware manifests leave TX power to the stack/host), so the product
/// stays at a conservative +10 dBm. Hardware-validated against ZHA.
pub const TX_POWER_DBM: i8 = 10;
