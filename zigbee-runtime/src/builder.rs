//! Device builder — fluent API for configuring a Zigbee device.

use core::mem::MaybeUninit;

use crate::power::{PowerManager, PowerMode};
use crate::role::{DeviceRole, EndDevice, RelayRouter, RoleState, Router};
use crate::{
    EndpointConfig, EndpointIdentifyCluster, MAX_CLUSTERS_PER_ENDPOINT, MAX_ENDPOINTS, ZigbeeDevice,
};
use zigbee_aps::ApsLayer;
use zigbee_bdb::BdbLayer;
use zigbee_mac::{MacDriver, ParentMacDriver};
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

fn is_mains_powered(source: PowerSource) -> bool {
    matches!(
        source,
        PowerSource::MainsSinglePhase
            | PowerSource::MainsThreePhase
            | PowerSource::EmergencyMainsConstantlyPowered
            | PowerSource::EmergencyMainsTransferSwitch
            | PowerSource::MainsSinglePhaseWithBatteryBackup
            | PowerSource::MainsThreePhaseWithBatteryBackup
            | PowerSource::EmergencyMainsConstantlyPoweredWithBatteryBackup
            | PowerSource::EmergencyMainsTransferSwitchWithBatteryBackup
    )
}

fn node_mac_capabilities(device_type: DeviceType, rx_on: bool, power_source: PowerSource) -> u8 {
    let mut capabilities = 0x80; // AllocateAddress
    if matches!(device_type, DeviceType::Coordinator) {
        capabilities |= 0x01; // AlternatePanCoordinator
    }
    if !matches!(device_type, DeviceType::EndDevice) {
        capabilities |= 0x02; // DeviceType: FFD
    }
    if is_mains_powered(power_source) {
        capabilities |= 0x04; // PowerSource: mains
    }
    if rx_on {
        capabilities |= 0x08; // ReceiverOnWhenIdle
    }
    capabilities
}

/// Node Descriptor server mask for `device_type`.
///
/// Always advertises the Core R22 Stack Compliance Revision (bits 9..=14);
/// certified coordinators downgrade a device reporting revision 0 to pre-R21
/// join and security behaviour.
///
/// Service bits are only claimed where the stack actually provides the
/// service: a coordinator forms a centralized network, owns the network key
/// and transports it to joiners, so it is the Primary Trust Center. The
/// Network Manager and cache bits stay clear — this stack answers
/// Mgmt_NWK_Update_req but never drives frequency agility, and it keeps no
/// binding or discovery cache on behalf of other devices.
fn node_server_mask(device_type: DeviceType) -> u16 {
    let services = match device_type {
        DeviceType::Coordinator => {
            zigbee_zdo::descriptors::NodeDescriptor::SERVER_PRIMARY_TRUST_CENTER
        }
        DeviceType::Router | DeviceType::EndDevice => 0,
    };
    zigbee_zdo::descriptors::NodeDescriptor::server_mask(
        services,
        zigbee_zdo::descriptors::STACK_COMPLIANCE_REVISION,
    )
}

/// A device could not be built because the requested logical role and the
/// configured [`DeviceType`] disagree.
///
/// This makes an otherwise silent misconfiguration explicit: e.g. asking for an
/// end-device build ([`build`](DeviceBuilder::build)) while
/// [`device_type`](DeviceBuilder::device_type) is set to
/// [`DeviceType::Router`], or a relay build while it is set to
/// [`DeviceType::Coordinator`]. The ergonomic `build*` methods panic on this;
/// the `try_build*` methods return it so a caller can handle it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildError {
    /// The role does not accept the configured [`DeviceType`].
    ///
    /// - end-device roles accept only [`DeviceType::EndDevice`],
    /// - a relay router accepts only [`DeviceType::Router`],
    /// - a router/parent accepts [`DeviceType::Router`] or
    ///   [`DeviceType::Coordinator`].
    RoleRejectsDeviceType {
        role: &'static str,
        device_type: DeviceType,
    },
}

/// Whether logical role `R` may run as `device_type`.
///
/// A parent role may be a router or coordinator; a non-parent routing role
/// (relay) may only be a router; a leaf end device may only be an end device.
const fn role_accepts<R: DeviceRole>(device_type: DeviceType) -> bool {
    if R::IS_PARENT {
        matches!(device_type, DeviceType::Router | DeviceType::Coordinator)
    } else if R::CAN_ROUTE {
        matches!(device_type, DeviceType::Router)
    } else {
        matches!(device_type, DeviceType::EndDevice)
    }
}

/// The canonical [`DeviceType`] a role runs as when none was set explicitly.
const fn default_device_type<R: DeviceRole>() -> DeviceType {
    if R::CAN_ROUTE {
        DeviceType::Router
    } else {
        DeviceType::EndDevice
    }
}

/// Whether the ergonomic (panicking) build path may proceed for role `R` with
/// `device_type` — `None` means "use the role's canonical type".
///
/// Kept a tiny `const fn` predicate so the sensor's `build()`/`build_into()`
/// carry only a comparison and a cold panic call, not the `BuildError`
/// construction (that lives in the `try_build*` paths).
#[inline]
const fn ergonomic_device_type<R: DeviceRole>(
    device_type: Option<DeviceType>,
) -> Option<DeviceType> {
    match device_type {
        Some(dt) if role_accepts::<R>(dt) => Some(dt),
        Some(_) => None,
        None => Some(default_device_type::<R>()),
    }
}

/// Explicit, cold failure for a role/device-type misconfiguration.
///
/// Kept `#[cold]`/`#[inline(never)]` with a static message so the ergonomic
/// build path carries no formatting machinery.
#[cold]
#[inline(never)]
fn build_panic() -> ! {
    panic!("build(): device_type conflicts with the build method's role")
}

/// Fluent builder for creating a ZigbeeDevice.
pub struct DeviceBuilder<M: MacDriver> {
    mac: M,
    /// Explicitly requested device type, if any. `None` lets each terminal
    /// `build*` method choose the role's canonical type, so a caller need not
    /// repeat it; a set value is validated against the role.
    device_type: Option<DeviceType>,
    endpoints: heapless::Vec<EndpointConfig, MAX_ENDPOINTS>,
    manufacturer_name: &'static str,
    model_identifier: &'static str,
    application_version: u8,
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
            device_type: None,
            endpoints: heapless::Vec::new(),
            manufacturer_name: "zigbee-rs",
            model_identifier: "Generic",
            application_version: 1,
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
    ///
    /// Optional: each terminal `build*` method already selects the role's
    /// canonical device type. Setting a value that disagrees with the chosen
    /// build method's role is rejected by [`BuildError`] rather than silently
    /// overridden — e.g. `.device_type(Router).build()` is an error, and
    /// `.device_type(Coordinator).build_router()` selects a coordinator.
    pub fn device_type(mut self, dt: DeviceType) -> Self {
        self.device_type = Some(dt);
        self
    }

    /// Resolve the effective device type for role `R`, validating any value
    /// the caller set explicitly.
    #[inline]
    fn resolve_device_type<R: DeviceRole>(&self) -> Result<DeviceType, BuildError> {
        match self.device_type {
            Some(dt) if role_accepts::<R>(dt) => Ok(dt),
            Some(dt) => Err(BuildError::RoleRejectsDeviceType {
                role: R::NAME,
                device_type: dt,
            }),
            None => Ok(default_device_type::<R>()),
        }
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

    /// Set the Basic-cluster application version.
    pub fn application_version(mut self, version: u8) -> Self {
        self.application_version = version;
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
    ///
    /// Produces an [`EndDevice`]-role device (the default role), preserving
    /// existing `ZigbeeDevice<M>` source. To build a routing device, use
    /// [`build_relay`](Self::build_relay) (forwarding-only) or
    /// [`build_router`](Self::build_router) (child-accepting parent, bounded on
    /// a genuine [`ParentMacDriver`] backend).
    ///
    /// # Panics
    ///
    /// Panics if [`device_type`](Self::device_type) was set to a non-end-device
    /// type — that is a role/device-type misconfiguration, not a valid end
    /// device. Use [`try_build`](Self::try_build) for a non-panicking result.
    #[inline]
    pub fn build(self) -> ZigbeeDevice<M, EndDevice> {
        match ergonomic_device_type::<EndDevice>(self.device_type) {
            Some(device_type) => self.assemble::<EndDevice>(device_type),
            None => build_panic(),
        }
    }

    /// Fallible [`build`](Self::build): errors instead of panicking when the
    /// configured device type is not an end-device type.
    #[inline]
    pub fn try_build(self) -> Result<ZigbeeDevice<M, EndDevice>, BuildError> {
        let device_type = self.resolve_device_type::<EndDevice>()?;
        Ok(self.assemble::<EndDevice>(device_type))
    }

    /// Build a forwarding-only [`RelayRouter`] device.
    ///
    /// A relay is an always-on FFD that relays NWK traffic and runs router
    /// maintenance but **cannot accept children**, so it needs only the base
    /// [`MacDriver`] surface — it honestly models a backend whose MAC has no
    /// parent-side association primitives. It builds as [`DeviceType::Router`].
    ///
    /// # Panics
    ///
    /// Panics if [`device_type`](Self::device_type) was set to anything other
    /// than [`DeviceType::Router`]. Use
    /// [`try_build_relay`](Self::try_build_relay) for a non-panicking result.
    #[inline]
    pub fn build_relay(self) -> ZigbeeDevice<M, RelayRouter> {
        match ergonomic_device_type::<RelayRouter>(self.device_type) {
            Some(device_type) => self.assemble::<RelayRouter>(device_type),
            None => build_panic(),
        }
    }

    /// Fallible [`build_relay`](Self::build_relay).
    #[inline]
    pub fn try_build_relay(self) -> Result<ZigbeeDevice<M, RelayRouter>, BuildError> {
        let device_type = self.resolve_device_type::<RelayRouter>()?;
        Ok(self.assemble::<RelayRouter>(device_type))
    }

    /// Build a router/parent-role device.
    ///
    /// Bounded on [`ParentMacDriver`]: a MAC backend that cannot accept
    /// children cannot construct a router, so the logical role and the physical
    /// parent capability cannot disagree. Builds as [`DeviceType::Router`]
    /// unless [`device_type`](Self::device_type) selects
    /// [`DeviceType::Coordinator`].
    ///
    /// # Panics
    ///
    /// Panics if [`device_type`](Self::device_type) was set to
    /// [`DeviceType::EndDevice`]. Use
    /// [`try_build_router`](Self::try_build_router) for a non-panicking result.
    #[inline]
    pub fn build_router(self) -> ZigbeeDevice<M, Router>
    where
        M: ParentMacDriver,
    {
        match ergonomic_device_type::<Router>(self.device_type) {
            Some(device_type) => self.assemble::<Router>(device_type),
            None => build_panic(),
        }
    }

    /// Fallible [`build_router`](Self::build_router).
    #[inline]
    pub fn try_build_router(self) -> Result<ZigbeeDevice<M, Router>, BuildError>
    where
        M: ParentMacDriver,
    {
        let device_type = self.resolve_device_type::<Router>()?;
        Ok(self.assemble::<Router>(device_type))
    }

    /// Build a coordinator: a parent-role device that forms a centralized
    /// network. Sugar for [`build_router`](Self::build_router) with
    /// [`DeviceType::Coordinator`]; bounded on [`ParentMacDriver`].
    ///
    /// # Panics
    ///
    /// Panics if [`device_type`](Self::device_type) was set to a
    /// non-coordinator type.
    #[inline]
    pub fn build_coordinator(mut self) -> ZigbeeDevice<M, Router>
    where
        M: ParentMacDriver,
    {
        if matches!(
            self.device_type,
            Some(device_type) if device_type != DeviceType::Coordinator
        ) {
            build_panic();
        }
        self.device_type = Some(DeviceType::Coordinator);
        self.build_router()
    }

    /// Assemble the layer stack into a device of the requested role.
    ///
    /// The role selects the zero-sized `_role` marker and the per-role inline
    /// `role_state` (`R::State::new()`); everything else in the constructed
    /// stack is identical across roles, so this preserves the exact prior
    /// end-device build. `device_type` is the value already resolved and
    /// validated against role `R`.
    #[inline(never)]
    fn assemble<R: DeviceRole>(self, device_type: DeviceType) -> ZigbeeDevice<M, R> {
        // Construct the layer stack: MAC → NWK → APS → ZDO → BDB
        let mut nwk = NwkLayer::new(self.mac, device_type);

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
        let logical_type = match device_type {
            DeviceType::Coordinator => zigbee_zdo::descriptors::LogicalType::Coordinator,
            DeviceType::Router => zigbee_zdo::descriptors::LogicalType::Router,
            DeviceType::EndDevice => zigbee_zdo::descriptors::LogicalType::EndDevice,
        };
        let node_desc = zigbee_zdo::descriptors::NodeDescriptor {
            logical_type,
            // Zigbee PRO security is provided by NWK/APS, not the IEEE
            // 802.15.4 MAC security-capability bit.
            mac_capabilities: node_mac_capabilities(device_type, rx_on, self.power_source),
            server_mask: node_server_mask(device_type),
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
            basic_cluster: BasicCluster::new_with_application_version(
                self.manufacturer_name,
                self.model_identifier,
                self.date_code,
                self.sw_build_id,
                self.application_version,
                self.power_source,
            ),
            identify_clusters,
            channel_mask: self.channel_mask,
            pending_responses: heapless::Vec::new(),
            scratch: super::RuntimeScratch::new(),
            state_dirty: false,
            secure_rejoin_retry_at: None,
            role_state: <R as DeviceRole>::State::new(),
            _role: core::marker::PhantomData,
        }
    }

    /// Build the ZigbeeDevice into caller-provided storage.
    ///
    /// This avoids the extra closure frame introduced by
    /// `StaticCell::init_with(|| builder.build())` on small MCUs. Produces an
    /// [`EndDevice`]-role device; use
    /// [`build_relay_into`](Self::build_relay_into) or
    /// [`build_router_into`](Self::build_router_into) for a routing device.
    ///
    /// # Panics
    ///
    /// Panics on a role/device-type mismatch — see [`build`](Self::build). Use
    /// [`try_build_into`](Self::try_build_into) for a non-panicking result.
    #[inline]
    pub fn build_into(
        self,
        dst: &mut MaybeUninit<ZigbeeDevice<M, EndDevice>>,
    ) -> &mut ZigbeeDevice<M, EndDevice> {
        match ergonomic_device_type::<EndDevice>(self.device_type) {
            Some(device_type) => self.assemble_into::<EndDevice>(device_type, dst),
            None => build_panic(),
        }
    }

    /// Fallible [`build_into`](Self::build_into).
    #[inline]
    pub fn try_build_into(
        self,
        dst: &mut MaybeUninit<ZigbeeDevice<M, EndDevice>>,
    ) -> Result<&mut ZigbeeDevice<M, EndDevice>, BuildError> {
        let device_type = self.resolve_device_type::<EndDevice>()?;
        Ok(self.assemble_into::<EndDevice>(device_type, dst))
    }

    /// Build a forwarding-only [`RelayRouter`] device into caller-provided
    /// storage — see [`build_relay`](Self::build_relay).
    ///
    /// # Panics
    ///
    /// Panics on a role/device-type mismatch.
    #[inline]
    pub fn build_relay_into(
        self,
        dst: &mut MaybeUninit<ZigbeeDevice<M, RelayRouter>>,
    ) -> &mut ZigbeeDevice<M, RelayRouter> {
        match ergonomic_device_type::<RelayRouter>(self.device_type) {
            Some(device_type) => self.assemble_into::<RelayRouter>(device_type, dst),
            None => build_panic(),
        }
    }

    /// Fallible [`build_relay_into`](Self::build_relay_into).
    #[inline]
    pub fn try_build_relay_into(
        self,
        dst: &mut MaybeUninit<ZigbeeDevice<M, RelayRouter>>,
    ) -> Result<&mut ZigbeeDevice<M, RelayRouter>, BuildError> {
        let device_type = self.resolve_device_type::<RelayRouter>()?;
        Ok(self.assemble_into::<RelayRouter>(device_type, dst))
    }

    /// Build a router/parent-role device into caller-provided storage.
    ///
    /// Bounded on [`ParentMacDriver`] — see [`build_router`](Self::build_router).
    ///
    /// # Panics
    ///
    /// Panics on a role/device-type mismatch.
    #[inline]
    pub fn build_router_into(
        self,
        dst: &mut MaybeUninit<ZigbeeDevice<M, Router>>,
    ) -> &mut ZigbeeDevice<M, Router>
    where
        M: ParentMacDriver,
    {
        match ergonomic_device_type::<Router>(self.device_type) {
            Some(device_type) => self.assemble_into::<Router>(device_type, dst),
            None => build_panic(),
        }
    }

    /// Fallible [`build_router_into`](Self::build_router_into).
    #[inline]
    pub fn try_build_router_into(
        self,
        dst: &mut MaybeUninit<ZigbeeDevice<M, Router>>,
    ) -> Result<&mut ZigbeeDevice<M, Router>, BuildError>
    where
        M: ParentMacDriver,
    {
        let device_type = self.resolve_device_type::<Router>()?;
        Ok(self.assemble_into::<Router>(device_type, dst))
    }

    /// Build a coordinator into caller-provided storage — see
    /// [`build_coordinator`](Self::build_coordinator).
    ///
    /// # Panics
    ///
    /// Panics on a role/device-type mismatch.
    #[inline]
    pub fn build_coordinator_into(
        mut self,
        dst: &mut MaybeUninit<ZigbeeDevice<M, Router>>,
    ) -> &mut ZigbeeDevice<M, Router>
    where
        M: ParentMacDriver,
    {
        if matches!(
            self.device_type,
            Some(device_type) if device_type != DeviceType::Coordinator
        ) {
            build_panic();
        }
        self.device_type = Some(DeviceType::Coordinator);
        self.build_router_into(dst)
    }

    /// Assemble the layer stack into caller-provided storage of the requested
    /// role. The role selects the zero-sized `_role` marker and the per-role
    /// inline `role_state` (`R::State::new()`). `device_type` is the value
    /// already resolved and validated against role `R`.
    #[inline(never)]
    fn assemble_into<R: DeviceRole>(
        self,
        device_type: DeviceType,
        dst: &mut MaybeUninit<ZigbeeDevice<M, R>>,
    ) -> &mut ZigbeeDevice<M, R> {
        let Self {
            mac,
            device_type: _,
            endpoints,
            manufacturer_name,
            model_identifier,
            application_version,
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
                    mac_capabilities: node_mac_capabilities(device_type, rx_on, power_source),
                    server_mask: node_server_mask(device_type),
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
            core::ptr::addr_of_mut!((*dst).basic_cluster).write(
                BasicCluster::new_with_application_version(
                    manufacturer_name,
                    model_identifier,
                    date_code,
                    sw_build_id,
                    application_version,
                    power_source,
                ),
            );
            core::ptr::addr_of_mut!((*dst).identify_clusters).write(identify_clusters);
            core::ptr::addr_of_mut!((*dst).channel_mask).write(channel_mask);
            core::ptr::addr_of_mut!((*dst).pending_responses).write(heapless::Vec::new());
            core::ptr::addr_of_mut!((*dst).scratch).write(super::RuntimeScratch::new());
            core::ptr::addr_of_mut!((*dst).state_dirty).write(false);
            core::ptr::addr_of_mut!((*dst).secure_rejoin_retry_at).write(None);
            core::ptr::addr_of_mut!((*dst).role_state).write(<R as DeviceRole>::State::new());
            core::ptr::addr_of_mut!((*dst)._role).write(core::marker::PhantomData);

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
