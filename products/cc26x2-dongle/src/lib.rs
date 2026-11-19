//! Product policy for CC2652P / CC2652P7 USB dongles: a Zigbee Router.
//!
//! The dongle has no sensor, LED or button wired to the CC26x2, so the
//! router is a Basic + Identify HA Range Extender whose value is proving the
//! CC26x2 RF core backend and the shared runtime against a real network.
//!
//! Physical wiring and the CCFG options stay in `cc26x2-dongle`; this crate
//! selects the Zigbee identity, the profile, the lifecycle policy, the
//! default TX power, the flash partitions (`link/memory-*.x`,
//! [`journal`]) and links the board CCFG into the image ([`ccfg`]).

#![no_std]

pub mod ccfg;
mod journal;
pub mod policy;
pub mod router;
pub mod storage;

pub use journal::{
    APPLICATION_END, APS_TABLE_PARTITION_SIZE, APS_TABLE_PARTITION_START,
    CHILD_TABLE_PARTITION_SIZE, CHILD_TABLE_PARTITION_START, PERSISTENCE_PARTITION_END,
    PERSISTENCE_PARTITION_SIZE, PERSISTENCE_PARTITION_START,
};
#[cfg(feature = "cc2652p7-dongle")]
pub use journal::{GP_PROXY_TABLE_PARTITION_SIZE, GP_PROXY_TABLE_PARTITION_START};

pub const MANUFACTURER: &str = "Zigbee-RS";
#[cfg(feature = "zbdongle-p")]
pub const ROUTER_MODEL: &str = "CC2652P-Router";
#[cfg(feature = "cc2652p7-dongle")]
pub const ROUTER_MODEL: &str = "CC2652P7-Router";
pub const DATE_CODE: &str = "20261119";
pub const SW_BUILD: &str = "0.1.0";
pub const ENDPOINT: u8 = 1;

/// Default `phyTransmitPower`: +5 dBm, the top of the standard PA. The
/// +20 dBm amplifier (14..20 dBm) is available through `set_tx_power`, but
/// a 2 MHz-wide 802.15.4 signal above roughly +13 dBm conducted exceeds the
/// ETSI EN 300 328 10 dBm/MHz density for an unknown antenna gain, so the
/// product does not select it by default.
pub const TX_POWER_DBM: i8 = 5;

/// Security-journal role tags (`SecurityStateJournal::select_role_tag`).
pub mod role {
    pub const ROUTER: u16 = 0x5201;
}
