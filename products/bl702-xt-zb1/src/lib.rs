//! Product policy for the BL702 XT-ZB1 Zigbee sensor.

#![no_std]

pub mod battery;
pub mod environment;
pub mod identity;
pub mod policy;
pub mod profile;
pub mod storage;

use zigbee_types::ChannelMask;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "XT-ZB1 Sensor";
pub const DATE_CODE: &str = "20260402";
pub const SW_BUILD: &str = "0.1.0";
pub const ENDPOINT: u8 = 1;
pub const CHANNEL: u8 = 15;
pub const CHANNEL_MASK: ChannelMask = ChannelMask(1u32 << CHANNEL);
pub const TX_POWER_DBM: i8 = 0;
