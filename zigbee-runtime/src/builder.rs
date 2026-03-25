//! Device builder — fluent API for configuring a Zigbee device.

use crate::{DeviceConfig, EndpointConfig, MAX_CLUSTERS_PER_ENDPOINT, MAX_ENDPOINTS, ZigbeeDevice};
use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;
use zigbee_types::*;

/// Fluent builder for creating a ZigbeeDevice.
pub struct DeviceBuilder<M: MacDriver> {
    mac: M,
    device_type: DeviceType,
    endpoints: heapless::Vec<EndpointConfig, MAX_ENDPOINTS>,
    manufacturer_name: &'static str,
    model_identifier: &'static str,
    sw_build_id: &'static str,
    channel_mask: ChannelMask,
}

impl<M: MacDriver> DeviceBuilder<M> {
    pub fn new(mac: M) -> Self {
        Self {
            mac,
            device_type: DeviceType::EndDevice,
            endpoints: heapless::Vec::new(),
            manufacturer_name: "zigbee-rs",
            model_identifier: "Generic",
            sw_build_id: "0.1.0",
            channel_mask: ChannelMask::ALL_2_4GHZ,
        }
    }

    /// Set the device type (Coordinator, Router, EndDevice).
    pub fn device_type(mut self, dt: DeviceType) -> Self {
        self.device_type = dt;
        self
    }

    /// Set the manufacturer name (Basic cluster attribute).
    pub fn manufacturer(mut self, name: &'static str) -> Self {
        self.manufacturer_name = name;
        self
    }

    /// Set the model identifier (Basic cluster attribute).
    pub fn model(mut self, model: &'static str) -> Self {
        self.model_identifier = model;
        self
    }

    /// Set the software build ID.
    pub fn sw_build(mut self, build: &'static str) -> Self {
        self.sw_build_id = build;
        self
    }

    /// Set the channel mask for scanning.
    pub fn channels(mut self, mask: ChannelMask) -> Self {
        self.channel_mask = mask;
        self
    }

    /// Add an endpoint with the given profile, device ID, and cluster configuration.
    pub fn endpoint(
        mut self,
        endpoint: u8,
        profile_id: u16,
        device_id: u16,
        configure: impl FnOnce(EndpointBuilder) -> EndpointBuilder,
    ) -> Self {
        let ep_builder = EndpointBuilder {
            endpoint,
            profile_id,
            device_id,
            device_version: 1,
            server_clusters: heapless::Vec::new(),
            client_clusters: heapless::Vec::new(),
        };
        let configured = configure(ep_builder);
        let _ = self.endpoints.push(EndpointConfig {
            endpoint: configured.endpoint,
            profile_id: configured.profile_id,
            device_id: configured.device_id,
            device_version: configured.device_version,
            server_clusters: configured.server_clusters,
            client_clusters: configured.client_clusters,
        });
        self
    }

    /// Build the ZigbeeDevice.
    pub fn build(self) -> ZigbeeDevice<M> {
        ZigbeeDevice {
            config: DeviceConfig {
                mac: self.mac,
                device_type: self.device_type,
                endpoints: self.endpoints,
                manufacturer_name: self.manufacturer_name,
                model_identifier: self.model_identifier,
                sw_build_id: self.sw_build_id,
                channel_mask: self.channel_mask,
            },
        }
    }
}

/// Builder for configuring a single endpoint's clusters.
pub struct EndpointBuilder {
    pub endpoint: u8,
    pub profile_id: u16,
    pub device_id: u16,
    pub device_version: u8,
    pub server_clusters: heapless::Vec<u16, MAX_CLUSTERS_PER_ENDPOINT>,
    pub client_clusters: heapless::Vec<u16, MAX_CLUSTERS_PER_ENDPOINT>,
}

impl EndpointBuilder {
    /// Add a server-side cluster.
    pub fn cluster_server(mut self, cluster_id: u16) -> Self {
        let _ = self.server_clusters.push(cluster_id);
        self
    }

    /// Add a client-side cluster.
    pub fn cluster_client(mut self, cluster_id: u16) -> Self {
        let _ = self.client_clusters.push(cluster_id);
        self
    }

    /// Set the device version.
    pub fn device_version(mut self, version: u8) -> Self {
        self.device_version = version;
        self
    }
}
