//! Product policy for the CC2340R5 "flower" soil-moisture sensor.
//!
//! The board crate (`flower-cc2340r5`) owns physical wiring. This crate owns
//! the Zigbee identity, the soil-probe calibration (cargo feature
//! `koptserg` or `diyzi`), detection and persistence of the optional I2C
//! sensors, the endpoint profile built from that detection result, the
//! measurement cadence, the lifecycle policy, and the flash layout.

#![no_std]

pub mod battery;
pub mod boot;
pub mod calibration;
pub mod detection;
pub mod measurement;
pub mod policy;
pub mod profile;
pub mod storage;

pub const MANUFACTURER: &str = "zigbee-rs";
pub const MODEL: &str = "CC2340-Flower";
pub const DATE_CODE: &str = "20260415";
pub const SW_BUILD: &str = "0.1.0";
pub const ENDPOINT: u8 = 1;

#[cfg(all(feature = "koptserg", feature = "diyzi"))]
compile_error!("select exactly one soil calibration: `koptserg` or `diyzi`");
#[cfg(not(any(feature = "koptserg", feature = "diyzi")))]
compile_error!("select a soil calibration feature: `koptserg` or `diyzi`");
