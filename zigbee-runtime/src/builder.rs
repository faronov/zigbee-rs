//! Device builder — fluent API for configuring a Zigbee device.

use core::mem::MaybeUninit;

use crate::power::{PowerManager, PowerMode};
use crate::{
    EndpointConfig, EndpointIdentifyCluster, MAX_CLUSTERS_PER_ENDPOINT, MAX_ENDPOINTS, ZigbeeDevice,
};
use zigbee_aps::ApsLayer;
use zigbee_bdb::BdbLayer;
use zigbee_mac::MacDriver;
use zigbee_nwk::{DeviceType, NwkLayer};
use zigbee_types::*;
use zigbee_zcl::clusters::basic::{BasicCluster, PowerSource};
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::foundation::reporting::ReportingEngine;
use zigbee_zcl::{ClusterId, DeviceId};
use zigbee_zdo::ZdoLayer;

fn build_identify_clusters(
    endpoints: &[EndpointConfig],
) -> heapless::Vec<EndpointIdentifyCluster, MAX_ENDPOINTS> {
    let mut clusters = heapless::Vec::new();
    for endpoint in endpoints {
        if endpoint.server_clusters.contains(&ClusterId::IDENTIFY) {
            let _ = clusters.push(EndpointIdentifyCluster {
                endpoint: endpoint.endpoint,
                cluster: IdentifyCluster::new(),
            });
        }
    }
    clusters
}

/// Fluent builder for creating a ZigbeeDevice.
pub struct DeviceBuilder<M: MacDriver> {
    mac: M,
    device_type: DeviceType,
    endpoints: heapless::Vec<EndpointConfig, MAX_ENDPOINTS>,
    manufacturer_name: &'static str,
    model_identifier: &'static str,
    sw_build_id: &'static str,
    date_code: &'static str,
    power_source: PowerSource,
    channel_mask: ChannelMask,
    power_mode: PowerMode,
    automatic_polling: bool,
    concentrator: Option<(zigbee_nwk::routing::ConcentratorType, u16, u8)>,
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
            power_source: PowerSource::Unknown,
            channel_mask: ChannelMask::ALL_2_4GHZ,
            power_mode: PowerMode::AlwaysOn,
            automatic_polling: true,
            concentrator: None,
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

    /// Set the Basic-cluster power source.
    pub fn power_source(mut self, source: PowerSource) -> Self {
        self.power_source = source;
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

    /// Enable or disable parent polling from `tick()`.
    ///
    /// Disable this when the application owns the SED poll loop and calls
    /// [`ZigbeeDevice::poll`] directly.
    pub fn automatic_polling(mut self, enabled: bool) -> Self {
        self.automatic_polling = enabled;
        self
    }

    /// Enable concentrator (many-to-one) mode for this device.
    ///
    /// Only valid for Router or Coordinator device types.
    /// - `ctype`: LowRam (devices re-send Route Records each time) or HighRam (cached)
    /// - `interval_secs`: how often to broadcast MTOR RREQ (default 60s)
    /// - `radius`: hop limit for MTOR RREQ (default 5)
    pub fn concentrator(
        mut self,
        ctype: zigbee_nwk::routing::ConcentratorType,
        interval_secs: u16,
        radius: u8,
    ) -> Self {
        self.concentrator = Some((ctype, interval_secs, radius));
        self
    }

    /// Add an endpoint with the given profile, device ID, and cluster configuration.
    pub fn endpoint(
        mut self,
        endpoint: u8,
        profile_id: u16,
        device_id: DeviceId,
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
    #[inline(never)]
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

        // Enable concentrator mode if requested
        if let Some((ctype, interval, radius)) = self.concentrator {
            nwk.start_concentrator(ctype, interval, radius);
        }

        let aps = ApsLayer::new(nwk);
        let mut zdo = ZdoLayer::new(aps);

        // Register application endpoints into ZDO so that
        // Simple_Desc_req, Active_EP_req, Match_Desc_req return correct data.
        for ep in &self.endpoints {
            let mut input_clusters = heapless::Vec::new();
            for &c in &ep.server_clusters {
                let _ = input_clusters.push(c.0);
            }
            let mut output_clusters = heapless::Vec::new();
            for &c in &ep.client_clusters {
                let _ = output_clusters.push(c.0);
            }
            let desc = zigbee_zdo::descriptors::SimpleDescriptor {
                endpoint: ep.endpoint,
                profile_id: ep.profile_id,
                device_id: ep.device_id.0,
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
            // bit7=AllocAddr, bit3=RxOnWhenIdle. Zigbee PRO security is
            // provided by NWK/APS, not the IEEE 802.15.4 MAC security bit.
            mac_capabilities: if rx_on { 0x88 } else { 0x80 },
            ..Default::default()
        };
        zdo.set_node_descriptor(node_desc);
        zdo.set_power_descriptor(zigbee_zdo::descriptors::PowerDescriptor::default());

        let mut bdb = BdbLayer::new(zdo);
        bdb.attributes_mut().primary_channel_set = self.channel_mask;
        bdb.attributes_mut().secondary_channel_set = ChannelMask(0);
        let identify_clusters = build_identify_clusters(&self.endpoints);

        ZigbeeDevice {
            bdb,
            endpoints: self.endpoints,
            reporting: ReportingEngine::new(),
            power: PowerManager::new(self.power_mode),
            power_now_ms: 0,
            automatic_polling: self.automatic_polling,
            pending_action: None,
            zcl_seq: 0,
            basic_cluster: BasicCluster::new(
                self.manufacturer_name,
                self.model_identifier,
                self.date_code,
                self.sw_build_id,
                self.power_source,
            ),
            identify_clusters,
            channel_mask: self.channel_mask,
            pending_responses: heapless::Vec::new(),
            scratch: super::RuntimeScratch::new(),
            state_dirty: false,
            secure_rejoin_retry_at: None,
        }
    }

    /// Build the ZigbeeDevice into caller-provided storage.
    ///
    /// This avoids the extra closure frame introduced by
    /// `StaticCell::init_with(|| builder.build())` on small MCUs.
    #[inline(never)]
    pub fn build_into(self, dst: &mut MaybeUninit<ZigbeeDevice<M>>) -> &mut ZigbeeDevice<M> {
        let Self {
            mac,
            device_type,
            endpoints,
            manufacturer_name,
            model_identifier,
            sw_build_id,
            date_code,
            power_source,
            channel_mask,
            power_mode,
            automatic_polling,
            concentrator,
        } = self;

        let rx_on = match &power_mode {
            PowerMode::AlwaysOn => true,
            PowerMode::Sleepy { .. } | PowerMode::DeepSleep { .. } => false,
        };
        let identify_clusters = build_identify_clusters(&endpoints);

        let dst = dst.as_mut_ptr();
        unsafe {
            BdbLayer::write_into(core::ptr::addr_of_mut!((*dst).bdb), mac, device_type);
            (*dst).bdb.attributes_mut().primary_channel_set = channel_mask;
            (*dst).bdb.attributes_mut().secondary_channel_set = ChannelMask(0);

            {
                let zdo = (*dst).bdb.zdo_mut();
                let nwk = zdo.aps_mut().nwk_mut();
                nwk.set_rx_on_when_idle(rx_on);

                if let Some((ctype, interval, radius)) = concentrator {
                    nwk.start_concentrator(ctype, interval, radius);
                }
            }

            {
                let zdo = (*dst).bdb.zdo_mut();
                for ep in &endpoints {
                    let mut input_clusters = heapless::Vec::new();
                    for &c in &ep.server_clusters {
                        let _ = input_clusters.push(c.0);
                    }
                    let mut output_clusters = heapless::Vec::new();
                    for &c in &ep.client_clusters {
                        let _ = output_clusters.push(c.0);
                    }
                    let desc = zigbee_zdo::descriptors::SimpleDescriptor {
                        endpoint: ep.endpoint,
                        profile_id: ep.profile_id,
                        device_id: ep.device_id.0,
                        device_version: ep.device_version,
                        input_clusters,
                        output_clusters,
                    };
                    let _ = zdo.register_endpoint(desc);
                }

                let logical_type = match device_type {
                    DeviceType::Coordinator => zigbee_zdo::descriptors::LogicalType::Coordinator,
                    DeviceType::Router => zigbee_zdo::descriptors::LogicalType::Router,
                    DeviceType::EndDevice => zigbee_zdo::descriptors::LogicalType::EndDevice,
                };
                let node_desc = zigbee_zdo::descriptors::NodeDescriptor {
                    logical_type,
                    mac_capabilities: if rx_on { 0x88 } else { 0x80 },
                    ..Default::default()
                };
                zdo.set_node_descriptor(node_desc);
                zdo.set_power_descriptor(zigbee_zdo::descriptors::PowerDescriptor::default());
            }

            core::ptr::addr_of_mut!((*dst).endpoints).write(endpoints);
            core::ptr::addr_of_mut!((*dst).reporting).write(ReportingEngine::new());
            core::ptr::addr_of_mut!((*dst).power).write(PowerManager::new(power_mode));
            core::ptr::addr_of_mut!((*dst).power_now_ms).write(0);
            core::ptr::addr_of_mut!((*dst).automatic_polling).write(automatic_polling);
            core::ptr::addr_of_mut!((*dst).pending_action).write(None);
            core::ptr::addr_of_mut!((*dst).zcl_seq).write(0);
            core::ptr::addr_of_mut!((*dst).basic_cluster).write(BasicCluster::new(
                manufacturer_name,
                model_identifier,
                date_code,
                sw_build_id,
                power_source,
            ));
            core::ptr::addr_of_mut!((*dst).identify_clusters).write(identify_clusters);
            core::ptr::addr_of_mut!((*dst).channel_mask).write(channel_mask);
            core::ptr::addr_of_mut!((*dst).pending_responses).write(heapless::Vec::new());
            core::ptr::addr_of_mut!((*dst).scratch).write(super::RuntimeScratch::new());
            core::ptr::addr_of_mut!((*dst).state_dirty).write(false);
            core::ptr::addr_of_mut!((*dst).secure_rejoin_retry_at).write(None);

            &mut *dst
        }
    }
}

/// Builder for configuring a single endpoint's clusters.
pub struct EndpointBuilder {
    pub endpoint: u8,
    pub profile_id: u16,
    pub device_id: DeviceId,
    pub device_version: u8,
    pub server_clusters: heapless::Vec<ClusterId, MAX_CLUSTERS_PER_ENDPOINT>,
    pub client_clusters: heapless::Vec<ClusterId, MAX_CLUSTERS_PER_ENDPOINT>,
}

impl EndpointBuilder {
    /// Add a server-side cluster to the endpoint descriptor.
    ///
    /// Basic and Identify use the runtime-owned instances configured by
    /// `DeviceBuilder`; other clusters must also be supplied as `ClusterRef`s.
    pub fn cluster_server(mut self, cluster_id: ClusterId) -> Self {
        if self.server_clusters.push(cluster_id).is_err() {
            log::warn!(
                "EndpointBuilder: server cluster table full, dropping cluster 0x{:04X}",
                cluster_id.0,
            );
        }
        self
    }

    /// Add a client-side cluster.
    pub fn cluster_client(mut self, cluster_id: ClusterId) -> Self {
        if self.client_clusters.push(cluster_id).is_err() {
            log::warn!(
                "EndpointBuilder: client cluster table full, dropping cluster 0x{:04X}",
                cluster_id.0,
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
