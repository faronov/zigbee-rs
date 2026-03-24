//! Zigbee Device Runtime — the top-level integration layer.
//!
//! This crate provides:
//! - `ZigbeeDevice` builder API for easy device creation
//! - Event loop that drives MAC→NWK→APS→ZCL processing
//! - NV storage abstraction for persistent state
//! - Power management hooks for sleepy end devices
//! - Pre-built device type templates (sensor, light, switch, etc.)
//!
//! # Example
//! ```rust,no_run,ignore
//! use zigbee_runtime::ZigbeeDevice;
//! use zigbee_mac::mock::MockMac;
//!
//! let mac = MockMac::new([1,2,3,4,5,6,7,8]);
//! let device = ZigbeeDevice::builder(mac)
//!     .device_type(DeviceType::EndDevice)
//!     .endpoint(1, 0x0104, 0x0302, |ep| {
//!         ep.cluster_server(0x0000)  // Basic
//!           .cluster_server(0x0402)  // Temperature Measurement
//!     })
//!     .build();
//!
//! device.start().await;
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

pub mod builder;
pub mod event_loop;
pub mod nv_storage;
pub mod power;
pub mod templates;

use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;
use zigbee_types::*;

/// Maximum number of endpoints on a device (endpoint 0 is ZDO, 1-240 are application)
pub const MAX_ENDPOINTS: usize = 8;
/// Maximum clusters per endpoint
pub const MAX_CLUSTERS_PER_ENDPOINT: usize = 16;

/// Endpoint configuration.
#[derive(Debug, Clone)]
pub struct EndpointConfig {
    pub endpoint: u8,
    pub profile_id: u16,
    pub device_id: u16,
    pub device_version: u8,
    pub server_clusters: heapless::Vec<u16, MAX_CLUSTERS_PER_ENDPOINT>,
    pub client_clusters: heapless::Vec<u16, MAX_CLUSTERS_PER_ENDPOINT>,
}

/// Device configuration built by the builder.
pub struct DeviceConfig<M: MacDriver> {
    pub mac: M,
    pub device_type: DeviceType,
    pub endpoints: heapless::Vec<EndpointConfig, MAX_ENDPOINTS>,
    pub manufacturer_name: &'static str,
    pub model_identifier: &'static str,
    pub sw_build_id: &'static str,
    pub channel_mask: ChannelMask,
}

/// The running Zigbee device.
pub struct ZigbeeDevice<M: MacDriver> {
    pub config: DeviceConfig<M>,
}

impl<M: MacDriver> ZigbeeDevice<M> {
    /// Create a new device builder.
    pub fn builder(mac: M) -> builder::DeviceBuilder<M> {
        builder::DeviceBuilder::new(mac)
    }
}
