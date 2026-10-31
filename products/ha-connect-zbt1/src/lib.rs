//! Product policy for the Home Assistant Connect ZBT-1 in three roles:
//! always-on End Device (the default), Router ([`router`]) and Sleepy End
//! Device ([`sed`], feature `sed`).
//!
//! The ZBT-1 is a USB dongle with no sensor, LED or button wired to the
//! EFR32MG21, so every role is a Basic + Identify endpoint whose value is
//! proving the EFR32 Series 2 radio and the shared runtime against a real
//! coordinator.
//!
//! Physical wiring stays in `ha-connect-zbt1`; this crate selects the Zigbee
//! identity, the endpoint profile, the bounded lifecycle policy, the default
//! TX power and the persistence partition at the top of the 1 MiB flash.

#![no_std]

mod journal;
pub mod policy;
pub mod profile;
pub mod router;
#[cfg(feature = "sed")]
pub mod sed;
pub mod storage;

pub use journal::{
    APPLICATION_END, APPLICATION_START, APS_TABLE_PARTITION_SIZE, APS_TABLE_PARTITION_START,
    CHILD_TABLE_PARTITION_SIZE, CHILD_TABLE_PARTITION_START, PERSISTENCE_PARTITION_END,
    PERSISTENCE_PARTITION_SIZE, PERSISTENCE_PARTITION_START,
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

/// Security-journal role tags (`SecurityStateJournal::select_role_tag`).
///
/// All role images share one security journal so outgoing frame counters
/// stay monotonic across a reflash; the tag stops one role from resuming
/// another role's network state. The original end-device image predates
/// the tags and writes none, so leave the network before flashing it after
/// a tagged role.
pub mod role {
    pub const ROUTER: u16 = 0x5201;
    pub const SLEEPY_END_DEVICE: u16 = 0x5203;
}

/// Model identifiers of the role images.
pub const ROUTER_MODEL: &str = "ZBT-1-Router";
pub const SED_MODEL: &str = "ZBT-1-SED";
