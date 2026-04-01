//! Device builder — fluent API for configuring a Zigbee device.

use crate::power::{PowerManager, PowerMode};
use crate::{EndpointConfig, MAX_CLUSTERS_PER_ENDPOINT, MAX_ENDPOINTS, ZigbeeDevice};
use zigbee_aps::ApsLayer;
use zigbee_bdb::BdbLayer;
use zigbee_mac::MacDriver;
use zigbee_nwk::{DeviceType, NwkLayer};
use zigbee_types::*;
use zigbee_zcl::foundation::reporting::ReportingEngine;
use zigbee_zdo::ZdoLayer;

/// Fluent builder for creating a ZigbeeDevice.
pub struct DeviceBuilder<M: MacDriver> {
    mac: M,
    device_type: DeviceType,
    endpoints: heapless::Vec<EndpointConfig, MAX_ENDPOINTS>,
    manufacturer_name: &'static str,
    model_identifier: &'static str,
    sw_build_id: &'static str,
    date_code: &'static str,
    channel_mask: ChannelMask,
    power_mode: PowerMode,
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
            date_code: "",
            channel_mask: ChannelMask::ALL_2_4GHZ,
            power_mode: PowerMode::AlwaysOn,
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

    /// Set the date code (Basic cluster attribute, e.g. "20260325").
    pub fn date_code(mut self, code: &'static str) -> Self {
        self.date_code = code;
        self
    }

    /// Set the channel mask for scanning.
    pub fn channels(mut self, mask: ChannelMask) -> Self {
        self.channel_mask = mask;
        self
    }

    /// Set the power mode (AlwaysOn, Sleepy, DeepSleep).
    pub fn power_mode(mut self, mode: PowerMode) -> Self {
        self.power_mode = mode;
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

    /// Build the ZigbeeDevice with the full BDB→ZDO→APS→NWK→MAC stack.
    pub fn build(self) -> ZigbeeDevice<M> {
        // Construct the layer stack: MAC → NWK → APS → ZDO → BDB
        let mut nwk = NwkLayer::new(self.mac, self.device_type);

        // For sleepy/deep-sleep modes, set rx_on_when_idle = false so the
        // association capability info correctly tells the coordinator we're a SED.
        let rx_on = match self.power_mode {
            PowerMode::AlwaysOn => true,
            PowerMode::Sleepy { .. } | PowerMode::DeepSleep { .. } => false,
        };
        nwk.set_rx_on_when_idle(rx_on);
        let aps = ApsLayer::new(nwk);
        let mut zdo = ZdoLayer::new(aps);

        // Register application endpoints into ZDO so that
        // Simple_Desc_req, Active_EP_req, Match_Desc_req return correct data.
        for ep in &self.endpoints {
            let mut input_clusters = heapless::Vec::new();
            for &c in &ep.server_clusters {
                let _ = input_clusters.push(c);
            }
            let mut output_clusters = heapless::Vec::new();
            for &c in &ep.client_clusters {
                let _ = output_clusters.push(c);
            }
            let desc = zigbee_zdo::descriptors::SimpleDescriptor {
                endpoint: ep.endpoint,
                profile_id: ep.profile_id,
                device_id: ep.device_id,
                device_version: ep.device_version,
                input_clusters,
                output_clusters,
            };
            let _ = zdo.register_endpoint(desc);
        }

        // Set IEEE address from MAC layer — deferred to start() since mlme_get is async.
        // For now, leave as default; it will be updated after join.

        // Set node/power descriptors based on device type
        let logical_type = match self.device_type {
            DeviceType::Coordinator => zigbee_zdo::descriptors::LogicalType::Coordinator,
            DeviceType::Router => zigbee_zdo::descriptors::LogicalType::Router,
            DeviceType::EndDevice => zigbee_zdo::descriptors::LogicalType::EndDevice,
        };
        let node_desc = zigbee_zdo::descriptors::NodeDescriptor {
            logical_type,
            mac_capabilities: if rx_on { 0x88 } else { 0x80 }, // bit7=AllocAddr, bit3=RxOnWhenIdle
            ..Default::default()
        };
        zdo.set_node_descriptor(node_desc);
        zdo.set_power_descriptor(zigbee_zdo::descriptors::PowerDescriptor::default());

        let bdb = BdbLayer::new(zdo);

        ZigbeeDevice {
            bdb,
            endpoints: self.endpoints,
            reporting: ReportingEngine::new(),
            power: PowerManager::new(self.power_mode),
            pending_action: None,
            zcl_seq: 0,
            manufacturer_name: self.manufacturer_name,
            model_identifier: self.model_identifier,
            sw_build_id: self.sw_build_id,
            date_code: self.date_code,
            channel_mask: self.channel_mask,
            pending_responses: heapless::Vec::new(),
            state_dirty: false,
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
        if self.server_clusters.push(cluster_id).is_err() {
            log::warn!(
                "EndpointBuilder: server cluster table full, dropping cluster 0x{:04X}",
                cluster_id,
            );
        }
        self
    }

    /// Add a client-side cluster.
    pub fn cluster_client(mut self, cluster_id: u16) -> Self {
        if self.client_clusters.push(cluster_id).is_err() {
            log::warn!(
                "EndpointBuilder: client cluster table full, dropping cluster 0x{:04X}",
                cluster_id,
            );
        }
        self
    }

    /// Set the device version.
    pub fn device_version(mut self, version: u8) -> Self {
        self.device_version = version;
        self
    }
}
