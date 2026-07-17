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
//! let mut device = ZigbeeDevice::builder(mac)
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

#[cfg(feature = "efr32-trace")]
macro_rules! rt_trace {
    ($($arg:tt)*) => { rtt_target::rprintln!($($arg)*); };
}
#[cfg(not(feature = "efr32-trace"))]
macro_rules! rt_trace {
    ($($arg:tt)*) => {};
}

pub mod builder;
pub mod event_loop;
pub mod firmware_writer;
pub mod log_nv;
pub mod nv_storage;
#[cfg(feature = "ota")]
pub mod ota;
pub mod power;
pub mod security_journal;
pub mod security_store;
pub mod templates;

use zigbee_aps::ApsAddress;
use zigbee_bdb::BdbLayer;
use zigbee_mac::{MacDriver, MacError, McpsDataIndication};
use zigbee_types::*;
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::foundation::reporting::ReportingEngine;
use zigbee_zcl::frame::ZclFrame;
use zigbee_zcl::{ClusterDirection, CommandId, ZclStatus};

use crate::nv_storage::{NvItemId, NvStorage};
use crate::power::PowerManager;
use crate::security_store::{
    CommissioningSecurityPersistence, PersistentSecurityState, SecurityStateStore,
    SecurityStoreError,
};

struct SyncUnsafeCell<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for SyncUnsafeCell<T> {}

impl<T> SyncUnsafeCell<T> {
    const fn new(value: T) -> Self {
        Self(core::cell::UnsafeCell::new(value))
    }

    fn get(&self) -> *mut T {
        self.0.get()
    }
}

static PROCESS_INCOMING_NWK_BUF: SyncUnsafeCell<[u8; 128]> = SyncUnsafeCell::new([0u8; 128]);
static PROCESS_INCOMING_AAD_BUF: SyncUnsafeCell<[u8; 64]> = SyncUnsafeCell::new([0u8; 64]);
static PROCESS_INCOMING_APS_BUF: SyncUnsafeCell<zigbee_aps::apsde::ApsFrameBuffer> =
    SyncUnsafeCell::new(zigbee_aps::apsde::ApsFrameBuffer {
        data: [0u8; 128],
        len: 0,
    });
static PROCESS_INCOMING_ZCL_BUF: SyncUnsafeCell<[u8; 253]> = SyncUnsafeCell::new([0u8; 253]);

/// A queued ZCL response to be sent in the next tick().
///
/// Because `process_incoming()` is sync but sending requires async MAC access,
/// we queue responses here and drain them in `tick()`.
struct PendingZclResponse {
    dst_addr: ShortAddress,
    dst_endpoint: u8,
    src_endpoint: u8,
    cluster_id: u16,
    #[cfg(feature = "router")]
    zcl_data: heapless::Vec<u8, 128>,
    #[cfg(not(feature = "router"))]
    zcl_data: heapless::Vec<u8, 128>,
}

/// Maximum number of endpoints on a device (endpoint 0 is ZDO, 1-240 are application)
#[cfg(feature = "router")]
pub const MAX_ENDPOINTS: usize = 8;
#[cfg(not(feature = "router"))]
pub const MAX_ENDPOINTS: usize = 4;
/// Maximum clusters per endpoint
#[cfg(feature = "router")]
pub const MAX_CLUSTERS_PER_ENDPOINT: usize = 16;
#[cfg(not(feature = "router"))]
pub const MAX_CLUSTERS_PER_ENDPOINT: usize = 8;

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

/// A reference to a cluster instance, tagged with its endpoint.
///
/// Pass a slice of these to `tick()` and `process_incoming()` so the runtime
/// can dispatch commands, read/write attributes, and send reports automatically.
pub struct ClusterRef<'a> {
    pub endpoint: u8,
    pub cluster: &'a mut dyn Cluster,
}

/// User-initiated actions, triggered by button presses or application logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UserAction {
    /// Join a network (BDB commissioning).
    Join,
    /// Rejoin a previously-joined network using stored NWK key.
    /// Use after `restore_state()` succeeds — skips full BDB commissioning
    /// and performs NWK-level rejoin on the last-known channel.
    Rejoin,
    /// Leave the current network.
    Leave,
    /// Toggle join/leave based on current state.
    Toggle,
    /// Open permit joining (coordinator/router only).
    PermitJoin(u8),
    /// Factory reset — leave network and clear all state.
    FactoryReset,
}

/// The running Zigbee device — owns the full BDB→ZDO→APS→NWK→MAC stack.
pub struct ZigbeeDevice<M: MacDriver> {
    /// BDB layer (transitively owns ZDO → APS → NWK → MAC).
    bdb: BdbLayer<M>,
    /// Application endpoint configurations.
    endpoints: heapless::Vec<EndpointConfig, MAX_ENDPOINTS>,
    /// ZCL attribute reporting engine.
    reporting: ReportingEngine,
    /// Power management.
    power: PowerManager,
    /// Pending user action (set by button press, consumed by tick).
    pending_action: Option<UserAction>,
    /// ZCL transaction sequence counter.
    zcl_seq: u8,
    /// Device metadata.
    manufacturer_name: &'static str,
    model_identifier: &'static str,
    sw_build_id: &'static str,
    date_code: &'static str,
    /// Channel mask for network scanning.
    channel_mask: ChannelMask,
    /// Queued ZCL responses to send in next tick().
    #[cfg(feature = "router")]
    pending_responses: heapless::Vec<PendingZclResponse, 4>,
    #[cfg(not(feature = "router"))]
    pending_responses: heapless::Vec<PendingZclResponse, 2>,
    /// Flag: network state has changed and should be persisted.
    state_dirty: bool,
}

impl<M: MacDriver> ZigbeeDevice<M> {
    /// Create a new device builder.
    pub fn builder(mac: M) -> builder::DeviceBuilder<M> {
        builder::DeviceBuilder::new(mac)
    }

    /// Allocate the next ZCL sequence number.
    fn next_zcl_seq(&mut self) -> u8 {
        let s = self.zcl_seq;
        self.zcl_seq = self.zcl_seq.wrapping_add(1);
        s
    }

    // ── Network lifecycle ───────────────────────────────────

    /// Initialize and join a Zigbee network via BDB commissioning.
    ///
    /// Performs BDB initialize → commission (network steering).
    /// Returns the assigned short address on success.
    #[inline(never)]
    pub async fn start(&mut self) -> Result<u16, event_loop::StartError> {
        rt_trace!("[RT] start: init");
        // Inline initialize + commission to avoid async state machine losing `self`
        // reference across await points (observed: self becomes NULL in separate
        // #[inline(never)] async methods after first await in release mode).
        let r = self.bdb.initialize().await;
        rt_trace!("[RT] bdb_init={}", if r.is_ok() { "ok" } else { "ERR" });
        if r.is_err() {
            return Err(event_loop::StartError::InitFailed);
        }
        rt_trace!("[RT] start: commission");
        let r = self.bdb.commission().await;
        rt_trace!("[RT] bdb_comm={}", if r.is_ok() { "ok" } else { "ERR" });
        if let Err(status) = r {
            return Err(event_loop::StartError::CommissioningFailed(status));
        }
        rt_trace!("[RT] start: finish");
        self.finish_join()
    }

    /// Initialize and join while durably reserving all security counters.
    #[inline(never)]
    pub async fn start_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<u16, event_loop::StartError> {
        let r = self.bdb.initialize().await;
        if r.is_err() {
            return Err(event_loop::StartError::InitFailed);
        }

        let mut persistence = CommissioningSecurityPersistence::new(store)
            .map_err(event_loop::StartError::PersistenceFailed)?;
        let result = self
            .bdb
            .network_steering_with_persistence(&mut persistence)
            .await;
        if let Some(error) = persistence.take_error() {
            return Err(event_loop::StartError::PersistenceFailed(error));
        }
        if let Err(status) = result {
            return Err(event_loop::StartError::CommissioningFailed(status));
        }
        self.finish_join()
    }

    /// Resume a committed network when available, otherwise commission a new
    /// one while using the same durable security store.
    #[inline(never)]
    pub async fn start_or_resume_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<u16, event_loop::StartError> {
        if self.bdb.initialize().await.is_err() {
            return Err(event_loop::StartError::InitFailed);
        }

        if self
            .restore_security_state(store)
            .map_err(event_loop::StartError::PersistenceFailed)?
        {
            return self.rejoin().await;
        }

        let mut persistence = CommissioningSecurityPersistence::new(store)
            .map_err(event_loop::StartError::PersistenceFailed)?;
        let result = self
            .bdb
            .network_steering_with_persistence(&mut persistence)
            .await;
        if let Some(error) = persistence.take_error() {
            return Err(event_loop::StartError::PersistenceFailed(error));
        }
        if let Err(status) = result {
            return Err(event_loop::StartError::CommissioningFailed(status));
        }
        self.finish_join()
    }

    #[inline(never)]
    fn finish_join(&mut self) -> Result<u16, event_loop::StartError> {
        let addr = self.bdb.zdo().nwk().nib().network_address.0;
        let ieee = self.bdb.zdo().nwk().nib().ieee_address;
        log::info!("[Runtime] Joined network as 0x{:04X}", addr);

        self.bdb.zdo_mut().set_local_nwk_addr(ShortAddress(addr));
        self.bdb.zdo_mut().set_local_ieee_addr(ieee);

        self.state_dirty = true;
        Ok(addr)
    }

    /// Rejoin a previously-joined network using stored NWK credentials.
    ///
    /// Uses a "silent resume" approach: restores MAC-layer addresses (PAN ID,
    /// short address, channel) so the device can immediately start polling
    /// its parent and responding to frames. A Device_annce broadcast notifies
    /// the coordinator that we're back online.
    ///
    /// This avoids the NWK Rejoin Request/Response exchange which some
    /// coordinators (e.g. ZHA/EZSP) handle unreliably. If the parent has
    /// disappeared, the caller should fall back to `start()` for a full
    /// BDB steering join.
    ///
    /// Call `restore_security_state()` first — it sets up NIB, security keys,
    /// and marks `node_is_on_a_network = true`.
    #[inline(never)]
    pub async fn rejoin(&mut self) -> Result<u16, event_loop::StartError> {
        log::info!("[Runtime] Resuming on previous network…");

        let nib = self.bdb.zdo().nwk().nib();
        let addr = nib.network_address.0;
        let channel = nib.logical_channel;
        let pan_id = nib.pan_id;
        let parent = nib.parent_address;

        log::info!(
            "[Runtime] Resume: addr=0x{:04X} PAN=0x{:04X} ch={} parent=0x{:04X}",
            addr,
            pan_id.0,
            channel,
            parent.0
        );

        // Configure MAC layer with restored addresses so it accepts
        // unicast frames and can transmit with the correct source address.
        let mac = self.bdb.zdo_mut().nwk_mut().mac_mut();
        mac.mlme_set(
            zigbee_mac::PibAttribute::PhyCurrentChannel,
            zigbee_mac::PibValue::U8(channel),
        )
        .await
        .map_err(|_| event_loop::StartError::InitFailed)?;
        mac.mlme_set(
            zigbee_mac::PibAttribute::MacPanId,
            zigbee_mac::PibValue::PanId(PanId(pan_id.0)),
        )
        .await
        .map_err(|_| event_loop::StartError::InitFailed)?;
        mac.mlme_set(
            zigbee_mac::PibAttribute::MacShortAddress,
            zigbee_mac::PibValue::ShortAddress(ShortAddress(addr)),
        )
        .await
        .map_err(|_| event_loop::StartError::InitFailed)?;

        // Read IEEE address from MAC hardware and set it in NIB.
        let hw_ieee = match self
            .bdb
            .zdo_mut()
            .nwk_mut()
            .mac_mut()
            .mlme_get(zigbee_mac::PibAttribute::MacExtendedAddress)
            .await
        {
            Ok(zigbee_mac::PibValue::ExtendedAddress(address)) => address,
            _ => return Err(event_loop::StartError::InitFailed),
        };
        let restored_ieee = self.bdb.zdo().nwk().nib().ieee_address;
        if restored_ieee != [0; 8] && restored_ieee != hw_ieee {
            return Err(event_loop::StartError::PersistenceFailed(
                SecurityStoreError::Corrupt,
            ));
        }
        self.bdb.zdo_mut().nwk_mut().nib_mut().ieee_address = hw_ieee;
        log::info!("[Runtime] NIB IEEE set from MAC: {:02X?}", hw_ieee);

        let ieee = self.bdb.zdo().nwk().nib().ieee_address;

        // Mark as joined so NWK/ZDO accept frames
        self.bdb.zdo_mut().nwk_mut().set_joined(true);

        // Sync addresses into ZDO so interview responses are correct
        self.bdb.zdo_mut().set_local_nwk_addr(ShortAddress(addr));
        self.bdb.zdo_mut().set_local_ieee_addr(ieee);

        // Announce immediately so coordinators and home automation stacks
        // that dropped our registry entry can rediscover the device.
        // Repeating the announce is cheap and avoids relying on stale state.
        self.send_device_annce()
            .await
            .map_err(|_| event_loop::StartError::InitFailed)?;
        log::info!("[Runtime] Device_annce sent after resume");

        self.state_dirty = true;
        Ok(addr)
    }

    /// Leave the current Zigbee network.
    pub async fn leave(&mut self) -> Result<(), event_loop::StartError> {
        log::info!("[Runtime] Leaving network…");
        self.bdb
            .zdo_mut()
            .nwk_mut()
            .nlme_leave(false)
            .await
            .map_err(|_| event_loop::StartError::InitFailed)?;
        self.bdb.attributes_mut().node_is_on_a_network = false;
        self.state_dirty = true;
        log::info!("[Runtime] Left network");
        Ok(())
    }

    /// Factory reset: leave network, clear all state, wipe NV.
    ///
    /// After this the device is in a "fresh out of box" state and
    /// must be commissioned again.
    pub async fn factory_reset(&mut self, nv: Option<&mut dyn NvStorage>) {
        log::info!("[Runtime] Factory reset…");

        // BDB factory_reset handles leave + state clearing
        let _ = self.bdb.factory_reset().await;

        // Clear NV storage if provided
        if let Some(nv) = nv {
            let items = [
                NvItemId::NwkPanId,
                NvItemId::NwkChannel,
                NvItemId::NwkShortAddress,
                NvItemId::NwkExtendedPanId,
                NvItemId::NwkIeeeAddress,
                NvItemId::NwkKey,
                NvItemId::NwkKeySeqNum,
                NvItemId::NwkFrameCounter,
                NvItemId::NwkDepth,
                NvItemId::NwkParentAddress,
                NvItemId::NwkUpdateId,
                NvItemId::BdbNodeIsOnNetwork,
                NvItemId::BdbCommissioningMode,
                NvItemId::BdbPrimaryChannelSet,
                NvItemId::BdbSecondaryChannelSet,
                NvItemId::BdbCommissioningGroupId,
                NvItemId::ApsBindingTable,
                NvItemId::ApsGroupTable,
            ];
            for id in &items {
                let _ = nv.delete(*id);
            }
        }

        log::info!("[Runtime] Factory reset complete");
    }

    // ── User action API ─────────────────────────────────────

    /// Queue a user action (e.g., from a button press).
    /// Will be processed on the next call to `tick()`.
    pub fn user_action(&mut self, action: UserAction) {
        self.pending_action = Some(action);
    }

    // ── Query state ─────────────────────────────────────────

    /// Whether the device is currently joined to a network.
    pub fn is_joined(&self) -> bool {
        self.bdb.is_on_network()
    }

    /// The device's NWK short address (0xFFFF if not joined).
    pub fn short_address(&self) -> u16 {
        self.bdb.zdo().nwk().nib().network_address.0
    }

    /// The current operating channel (0 if not joined).
    pub fn channel(&self) -> u8 {
        self.bdb.zdo().nwk().nib().logical_channel
    }

    /// The current PAN ID (0xFFFF if not joined).
    pub fn pan_id(&self) -> u16 {
        self.bdb.zdo().nwk().nib().pan_id.0
    }

    /// The device type (coordinator / router / end device).
    pub fn device_type(&self) -> zigbee_nwk::DeviceType {
        self.bdb.zdo().nwk().device_type()
    }

    /// The configured application endpoints.
    pub fn endpoints(&self) -> &[EndpointConfig] {
        &self.endpoints
    }

    /// The manufacturer name.
    pub fn manufacturer_name(&self) -> &str {
        self.manufacturer_name
    }

    /// The model identifier.
    pub fn model_identifier(&self) -> &str {
        self.model_identifier
    }

    /// The configured channel mask.
    pub fn channel_mask(&self) -> ChannelMask {
        self.channel_mask
    }

    pub fn steering_diagnostics(&self) -> zigbee_bdb::SteeringDiagnostics {
        self.bdb.steering_diagnostics()
    }

    pub fn nwk_rx_security_stats(&self) -> zigbee_nwk::NwkRxSecurityStats {
        self.bdb.zdo().aps().nwk().rx_security_stats()
    }

    pub fn aps_security_handshake_stats(&self) -> zigbee_aps::ApsSecurityHandshakeStats {
        self.bdb.zdo().aps().security_handshake_stats()
    }

    pub fn zdo_diagnostics(&self) -> zigbee_zdo::ZdoDiagnostics {
        self.bdb.zdo().diagnostics()
    }

    /// The software build identifier.
    pub fn sw_build_id(&self) -> &str {
        self.sw_build_id
    }

    /// The date code (Basic cluster attribute).
    pub fn date_code(&self) -> &str {
        self.date_code
    }

    /// Access the power manager (for sleep decisions).
    pub fn power(&self) -> &PowerManager {
        &self.power
    }

    /// Access the power manager mutably.
    pub fn power_mut(&mut self) -> &mut PowerManager {
        &mut self.power
    }

    /// Whether this device is configured as a sleepy end device.
    pub fn is_sleepy(&self) -> bool {
        !matches!(self.power.mode(), power::PowerMode::AlwaysOn)
    }

    /// Whether the network state has changed since last save.
    ///
    /// Check this after `tick()` returns — if true, call `save_state(nv)`
    /// and then `clear_state_dirty()` to persist the new state.
    pub fn state_dirty(&self) -> bool {
        self.state_dirty
    }

    /// Clear the dirty flag after saving state.
    pub fn clear_state_dirty(&mut self) {
        self.state_dirty = false;
    }

    // ── Reporting / Interview Detection ────────────────────

    /// Check if reporting has been configured for a specific cluster.
    ///
    /// Returns `true` after ZHA sends Configure Reporting for this cluster,
    /// which is the last step of the interview process per-cluster.
    pub fn is_cluster_reporting_configured(&self, endpoint: u8, cluster_id: u16) -> bool {
        self.reporting.has_cluster_configured(endpoint, cluster_id)
    }

    /// Count how many distinct clusters have reporting configured on an endpoint.
    pub fn configured_cluster_count(&self, endpoint: u8) -> usize {
        self.reporting.configured_cluster_count(endpoint)
    }

    // ── NV Persistence ─────────────────────────────────────

    /// Restore a fully commissioned network and reserve fresh counter ranges
    /// before any secured rejoin traffic can be sent.
    pub fn restore_security_state<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<bool, SecurityStoreError> {
        let Some(mut state) = store.load()? else {
            return Ok(false);
        };
        state.validate()?;
        if !state.commissioned {
            return Ok(false);
        }
        let configured_ieee = self.bdb.zdo().nwk().nib().ieee_address;
        if configured_ieee != [0; 8] && configured_ieee != state.ieee_address {
            return Err(SecurityStoreError::Corrupt);
        }

        let global_current = state.global_counter_limit;
        let global_limit = global_current
            .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
            .ok_or(SecurityStoreError::CounterExhausted)?;
        let tclk_current = state.tclk_counter_limit;
        let tclk_limit = tclk_current
            .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
            .ok_or(SecurityStoreError::CounterExhausted)?;

        state.global_counter_limit = global_limit;
        state.tclk_counter_limit = tclk_limit;
        store.store(&state)?;

        {
            let nwk = self.bdb.zdo_mut().nwk_mut();
            nwk.security_mut()
                .set_network_key(state.network_key, state.key_sequence);
            let nib = nwk.nib_mut();
            nib.extended_pan_id = state.extended_pan_id;
            nib.pan_id = PanId(state.pan_id);
            nib.network_address = ShortAddress(state.short_address);
            nib.ieee_address = state.ieee_address;
            nib.logical_channel = state.channel;
            nib.depth = state.depth;
            nib.parent_address = ShortAddress(state.parent_address);
            nib.update_id = state.update_id;
            nib.active_key_seq_number = state.key_sequence;
            nib.security_enabled = true;
            if !nib.set_frame_counter_reservation(global_current, global_limit) {
                return Err(SecurityStoreError::Corrupt);
            }
        }

        {
            let aps = self.bdb.zdo_mut().aps_mut();
            aps.aib_mut().aps_trust_center_address = state.trust_center_address;
            aps.security_mut()
                .add_key(zigbee_aps::security::ApsLinkKeyEntry {
                    partner_address: state.trust_center_address,
                    key: state.trust_center_link_key,
                    key_type: zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
                    outgoing_frame_counter: tclk_current,
                    outgoing_frame_counter_limit: tclk_limit,
                    incoming_frame_counter: state.tclk_incoming_counter,
                    incoming_frame_counter_valid: state.tclk_incoming_counter_valid,
                })
                .map_err(|_| SecurityStoreError::Full)?;
        }

        self.bdb.attributes_mut().node_is_on_a_network = true;
        self.bdb.attributes_mut().primary_channel_set = ChannelMask(1u32 << state.channel);
        self.bdb.attributes_mut().secondary_channel_set = ChannelMask(0);
        self.state_dirty = false;
        Ok(true)
    }

    /// Persist updated incoming counters and extend low outgoing reservations.
    ///
    /// Call before and after runtime operations that may send or accept secured
    /// frames. Storage is committed before in-memory limits are extended.
    pub fn refresh_security_state<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<bool, SecurityStoreError> {
        const LOW_WATER: u32 = 32;

        let Some(mut state) = store.load()? else {
            return Ok(false);
        };
        state.validate()?;
        if !state.commissioned {
            return Ok(false);
        }

        let nib = self.bdb.zdo().nwk().nib();
        if nib.ieee_address != state.ieee_address
            || nib.pan_id.0 != state.pan_id
            || nib.outgoing_frame_counter > nib.outgoing_frame_counter_limit
            || nib.outgoing_frame_counter_limit != state.global_counter_limit
        {
            return Err(SecurityStoreError::Corrupt);
        }

        let tclk = self
            .bdb
            .zdo()
            .aps()
            .security()
            .find_key(
                &state.trust_center_address,
                zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
            )
            .ok_or(SecurityStoreError::Corrupt)?;
        if tclk.key != state.trust_center_link_key
            || tclk.outgoing_frame_counter > tclk.outgoing_frame_counter_limit
            || tclk.outgoing_frame_counter_limit != state.tclk_counter_limit
        {
            return Err(SecurityStoreError::Corrupt);
        }

        let mut changed = false;
        let mut new_global_limit = nib.outgoing_frame_counter_limit;
        if nib
            .outgoing_frame_counter_limit
            .saturating_sub(nib.outgoing_frame_counter)
            <= LOW_WATER
        {
            new_global_limit = nib
                .outgoing_frame_counter_limit
                .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
                .ok_or(SecurityStoreError::CounterExhausted)?;
            state.global_counter_limit = new_global_limit;
            changed = true;
        }

        let mut new_tclk_limit = tclk.outgoing_frame_counter_limit;
        if tclk
            .outgoing_frame_counter_limit
            .saturating_sub(tclk.outgoing_frame_counter)
            <= LOW_WATER
        {
            new_tclk_limit = tclk
                .outgoing_frame_counter_limit
                .checked_add(zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE)
                .ok_or(SecurityStoreError::CounterExhausted)?;
            state.tclk_counter_limit = new_tclk_limit;
            changed = true;
        }

        if state.tclk_incoming_counter != tclk.incoming_frame_counter
            || state.tclk_incoming_counter_valid != tclk.incoming_frame_counter_valid
        {
            state.tclk_incoming_counter = tclk.incoming_frame_counter;
            state.tclk_incoming_counter_valid = tclk.incoming_frame_counter_valid;
            changed = true;
        }

        if !changed {
            return Ok(false);
        }
        store.store(&state)?;

        self.bdb
            .zdo_mut()
            .nwk_mut()
            .nib_mut()
            .outgoing_frame_counter_limit = new_global_limit;
        self.bdb
            .zdo_mut()
            .aps_mut()
            .security_mut()
            .find_key_mut(
                &state.trust_center_address,
                zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
            )
            .ok_or(SecurityStoreError::Corrupt)?
            .outgoing_frame_counter_limit = new_tclk_limit;
        Ok(true)
    }

    /// Clear commissioned state while preserving the global counter bound.
    pub fn factory_reset_security_state<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<(), SecurityStoreError> {
        let global_counter_limit = store
            .load()?
            .map(|state| state.global_counter_limit)
            .unwrap_or(0);
        let mut state = PersistentSecurityState::empty();
        state.global_counter_limit = global_counter_limit;
        store.store(&state)
    }

    /// Factory-reset the stack while retaining the global counter bound that
    /// prevents key/counter reuse on a later commissioning attempt.
    pub async fn factory_reset_with_security_store<S: SecurityStateStore>(
        &mut self,
        store: &mut S,
    ) -> Result<(), event_loop::StartError> {
        self.factory_reset_security_state(store)
            .map_err(event_loop::StartError::PersistenceFailed)?;
        self.bdb
            .factory_reset()
            .await
            .map_err(event_loop::StartError::CommissioningFailed)?;
        self.state_dirty = false;
        Ok(())
    }

    /// Process an incoming frame with crash-safe counter maintenance.
    pub async fn process_incoming_with_security_store<S: SecurityStateStore>(
        &mut self,
        indication: &McpsDataIndication,
        clusters: &mut [ClusterRef<'_>],
        store: &mut S,
    ) -> Result<Option<event_loop::StackEvent>, SecurityStoreError> {
        self.refresh_security_state(store)?;
        let event = self.process_incoming(indication, clusters).await;
        self.refresh_security_state(store)?;
        Ok(event)
    }

    /// Tick reporting and pending responses with crash-safe counter
    /// maintenance.
    pub async fn tick_with_security_store<S: SecurityStateStore>(
        &mut self,
        elapsed_secs: u16,
        clusters: &mut [ClusterRef<'_>],
        store: &mut S,
    ) -> Result<event_loop::TickResult, SecurityStoreError> {
        self.refresh_security_state(store)?;
        let result = self.tick(elapsed_secs, clusters).await;
        self.refresh_security_state(store)?;
        Ok(result)
    }

    /// Save critical network state to non-volatile storage.
    ///
    /// Call after: join, key update, bind/unbind, group changes, or before sleep.
    ///
    /// This legacy item-by-item format is not crash-safe for Zigbee security
    /// counters or unique Trust Center link keys. New secured devices must use
    /// `SecurityStateStore` and `start_or_resume_with_security_store()`.
    pub fn save_state(&self, nv: &mut dyn NvStorage) {
        let nib = self.bdb.zdo().nwk().nib();

        // Network identity
        let _ = nv.write(NvItemId::NwkPanId, &nib.pan_id.0.to_le_bytes());
        let _ = nv.write(NvItemId::NwkChannel, &[nib.logical_channel]);
        let _ = nv.write(
            NvItemId::NwkShortAddress,
            &nib.network_address.0.to_le_bytes(),
        );
        let _ = nv.write(NvItemId::NwkExtendedPanId, &nib.extended_pan_id);
        let _ = nv.write(NvItemId::NwkIeeeAddress, &nib.ieee_address);
        let _ = nv.write(NvItemId::NwkDepth, &[nib.depth]);
        let _ = nv.write(
            NvItemId::NwkParentAddress,
            &nib.parent_address.0.to_le_bytes(),
        );
        let _ = nv.write(NvItemId::NwkUpdateId, &[nib.update_id]);

        // NWK security — active key + frame counter
        if let Some(key_entry) = self.bdb.zdo().nwk().security().active_key() {
            let _ = nv.write(NvItemId::NwkKey, &key_entry.key);
            let _ = nv.write(NvItemId::NwkKeySeqNum, &[key_entry.seq_number]);
        }
        let fc = nib.outgoing_frame_counter;
        let _ = nv.write(NvItemId::NwkFrameCounter, &fc.to_le_bytes());

        // BDB state
        let on_network: u8 = if self.bdb.is_on_network() { 1 } else { 0 };
        let _ = nv.write(NvItemId::BdbNodeIsOnNetwork, &[on_network]);
        let _ = nv.write(
            NvItemId::BdbCommissioningMode,
            &[self.bdb.attributes().commissioning_mode.0],
        );
        let _ = nv.write(
            NvItemId::BdbPrimaryChannelSet,
            &self.bdb.attributes().primary_channel_set.0.to_le_bytes(),
        );
        let _ = nv.write(
            NvItemId::BdbSecondaryChannelSet,
            &self.bdb.attributes().secondary_channel_set.0.to_le_bytes(),
        );
        let _ = nv.write(
            NvItemId::BdbCommissioningGroupId,
            &self.bdb.attributes().commissioning_group_id.to_le_bytes(),
        );

        log::debug!(
            "[NV] Saved network state (PAN=0x{:04X}, ch={}, addr=0x{:04X})",
            nib.pan_id.0,
            nib.logical_channel,
            nib.network_address.0
        );
    }

    /// Restore network state from non-volatile storage.
    ///
    /// Call on startup before `start()`. If state is found, the device can
    /// attempt rejoin instead of full commissioning.
    /// Returns `true` if valid state was restored.
    ///
    /// This legacy format is not suitable for production secured restore; use
    /// `restore_security_state()` through
    /// `start_or_resume_with_security_store()` instead.
    pub fn restore_state(&mut self, nv: &dyn NvStorage) -> bool {
        let mut buf = [0u8; 16];

        // Check if we have stored network state
        let on_network = match nv.read(NvItemId::BdbNodeIsOnNetwork, &mut buf) {
            Ok(1) => buf[0] != 0,
            _ => return false,
        };
        if !on_network {
            return false;
        }

        // Restore network identity
        let pan_id = match nv.read(NvItemId::NwkPanId, &mut buf) {
            Ok(2) => PanId(u16::from_le_bytes([buf[0], buf[1]])),
            _ => return false,
        };
        let channel = match nv.read(NvItemId::NwkChannel, &mut buf) {
            Ok(1) => buf[0],
            _ => return false,
        };
        let short_addr = match nv.read(NvItemId::NwkShortAddress, &mut buf) {
            Ok(2) => ShortAddress(u16::from_le_bytes([buf[0], buf[1]])),
            _ => return false,
        };
        let mut epid = [0u8; 8];
        if nv.read(NvItemId::NwkExtendedPanId, &mut epid).is_err() {
            return false;
        }
        let depth = match nv.read(NvItemId::NwkDepth, &mut buf) {
            Ok(1) => buf[0],
            _ => 1,
        };
        let parent = match nv.read(NvItemId::NwkParentAddress, &mut buf) {
            Ok(2) => ShortAddress(u16::from_le_bytes([buf[0], buf[1]])),
            _ => ShortAddress(0x0000),
        };
        let update_id = match nv.read(NvItemId::NwkUpdateId, &mut buf) {
            Ok(1) => buf[0],
            _ => 0,
        };

        // Apply to NIB
        {
            let nib = self.bdb.zdo_mut().nwk_mut().nib_mut();
            nib.pan_id = pan_id;
            nib.logical_channel = channel;
            nib.network_address = short_addr;
            nib.extended_pan_id = epid;
            nib.depth = depth;
            nib.parent_address = parent;
            nib.update_id = update_id;
            // Restore IEEE address (critical for NWK security headers)
            let mut ieee_buf = [0u8; 8];
            if let Ok(8) = nv.read(NvItemId::NwkIeeeAddress, &mut ieee_buf) {
                nib.ieee_address = ieee_buf;
            }
        }

        // Restore NWK security key
        let mut key_buf = [0u8; 16];
        if let Ok(16) = nv.read(NvItemId::NwkKey, &mut key_buf) {
            let seq = match nv.read(NvItemId::NwkKeySeqNum, &mut buf) {
                Ok(1) => buf[0],
                _ => 0,
            };
            let fc = match nv.read(NvItemId::NwkFrameCounter, &mut buf) {
                Ok(4) => u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
                _ => 0,
            };
            self.bdb
                .zdo_mut()
                .nwk_mut()
                .security_mut()
                .set_network_key(key_buf, seq);
            {
                let nib = self.bdb.zdo_mut().nwk_mut().nib_mut();
                nib.active_key_seq_number = seq;
                nib.security_enabled = true;
            }
            // Restore frame counter with safety margin: frames may have been
            // sent after the last NV save, so the coordinator's expected counter
            // is higher than what we saved. Add 1000 to avoid replay rejection.
            const FC_SAFETY_MARGIN: u32 = 1000;
            let fc_safe = fc.saturating_add(FC_SAFETY_MARGIN);
            log::info!(
                "[NV] Restored NWK key seq={}, fc={} (saved={} +{})",
                seq,
                fc_safe,
                fc,
                FC_SAFETY_MARGIN
            );
            self.bdb
                .zdo_mut()
                .nwk_mut()
                .nib_mut()
                .outgoing_frame_counter = fc_safe;
        }

        // Mark as on-network in BDB
        self.bdb.attributes_mut().node_is_on_a_network = true;

        // Restore BDB attributes
        if let Ok(1) = nv.read(NvItemId::BdbCommissioningMode, &mut buf) {
            self.bdb.attributes_mut().commissioning_mode = zigbee_bdb::CommissioningMode(buf[0]);
        }
        if let Ok(4) = nv.read(NvItemId::BdbPrimaryChannelSet, &mut buf) {
            self.bdb.attributes_mut().primary_channel_set =
                ChannelMask(u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]));
        }
        if let Ok(4) = nv.read(NvItemId::BdbSecondaryChannelSet, &mut buf) {
            self.bdb.attributes_mut().secondary_channel_set =
                ChannelMask(u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]));
        }
        if let Ok(2) = nv.read(NvItemId::BdbCommissioningGroupId, &mut buf) {
            self.bdb.attributes_mut().commissioning_group_id = u16::from_le_bytes([buf[0], buf[1]]);
        }

        log::info!(
            "[NV] Restored network state (PAN=0x{:04X}, ch={}, addr=0x{:04X})",
            pan_id.0,
            channel,
            short_addr.0
        );
        true
    }

    // ── MAC proxy ───────────────────────────────────────────

    /// Wait for an incoming MAC frame. Blocks until a frame arrives.
    ///
    /// Use with `select!` and a timer for non-blocking operation:
    /// ```rust,ignore
    /// select! {
    ///     frame = device.receive() => { device.process_incoming(&frame.unwrap()); }
    ///     _ = Timer::after(Duration::from_secs(1)) => { device.tick(1).await; }
    /// }
    /// ```
    pub async fn receive(&mut self) -> Result<McpsDataIndication, MacError> {
        self.bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .mcps_data_indication()
            .await
    }

    /// Poll the parent for pending data (Sleepy End Device).
    ///
    /// Sends a MAC Data Request to the coordinator/parent and returns
    /// any queued frame. Returns `None` if no data is pending.
    /// After calling this, feed the result into `process_incoming()`.
    pub async fn poll(&mut self) -> Result<Option<McpsDataIndication>, MacError> {
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        match mac.mlme_poll().await? {
            Some(frame) => {
                // Wrap the raw poll response in a McpsDataIndication.
                // The parent address comes from NIB; LQI is unknown from poll.
                let parent = self.bdb.zdo().nwk().nib().parent_address;
                let pan_id = self.bdb.zdo().nwk().nib().pan_id;
                let our_addr = self.bdb.zdo().nwk().nib().network_address;
                Ok(Some(McpsDataIndication {
                    src_address: zigbee_types::MacAddress::Short(pan_id, parent),
                    dst_address: zigbee_types::MacAddress::Short(pan_id, our_addr),
                    lqi: 0, // not available from poll
                    payload: frame,
                    security_use: false,
                }))
            }
            None => Ok(None),
        }
    }

    // ── Incoming frame processing ───────────────────────────

    /// Process an incoming MAC frame through the full stack.
    ///
    /// MAC → NWK → APS → ZDO (endpoint 0) or ZCL (app endpoints).
    /// Async because ZDO handling sends responses directly through the stack.
    ///
    /// Pass registered cluster instances so the runtime can automatically:
    /// - Handle Read/Write/Discover Attributes using cluster attribute stores
    /// - Dispatch cluster-specific commands to `Cluster::handle_command()`
    /// - Sync Groups cluster actions to the APS group table
    #[inline(never)]
    pub async fn process_incoming(
        &mut self,
        indication: &McpsDataIndication,
        clusters: &mut [ClusterRef<'_>],
    ) -> Option<event_loop::StackEvent> {
        let mac_payload = indication.payload.as_slice();

        // NWK layer: parse NWK header, check if frame is for us, decrypt if secured
        let nwk_indication = {
            let nwk = self.bdb.zdo_mut().aps_mut().nwk_mut();
            let (header, consumed) = match zigbee_nwk::frames::NwkHeader::parse(mac_payload) {
                Some(v) => v,
                None => {
                    rt_trace!("[RT][EFR32] nwk_parse=fail len={}", mac_payload.len());
                    log::warn!(
                        "[RX] NWK header parse failed, {} bytes: {:02X?}",
                        mac_payload.len(),
                        &mac_payload[..mac_payload.len().min(8)]
                    );
                    return None;
                }
            };

            let dst = header.dst_addr;
            let src = header.src_addr;
            let nwk_fc = header.frame_control;
            let nwk_addr = nwk.nib().network_address;
            let is_for_us = dst == nwk_addr
                || dst == ShortAddress::BROADCAST
                || dst == ShortAddress(0xFFFF)
                || dst == ShortAddress(0xFFFD);

            rt_trace!(
                "[RT][EFR32] nwk type={} src=0x{:04X} dst=0x{:04X} sec={} for_us={} len={}",
                nwk_fc.frame_type,
                src.0,
                dst.0,
                nwk_fc.security as u8,
                is_for_us as u8,
                mac_payload.len().saturating_sub(consumed)
            );
            log::info!(
                "[RX] NWK type={} src=0x{:04X} dst=0x{:04X} sec={} for_us={} len={}",
                nwk_fc.frame_type,
                src.0,
                dst.0,
                nwk_fc.security,
                is_for_us,
                mac_payload.len() - consumed
            );

            if !is_for_us {
                rt_trace!("[RT][EFR32] drop not_for_us");
                return None;
            }

            if src == nwk_addr {
                rt_trace!("[RT][EFR32] drop self_originated src=0x{:04X}", src.0);
                return None;
            }

            // Only NWK Data frames (type=0) carry APS payloads.
            // NWK Command frames (type=1) are NWK-level management (Link Status,
            // Route Reply, Leave, etc.) — handle or drop them here.
            if nwk_fc.frame_type != 0 {
                // For unicast NWK commands addressed to us, log the command ID
                if is_for_us && nwk_fc.frame_type == 1 {
                    log::info!("[RX] NWK cmd for us from 0x{:04X}, will decode", src.0);
                    // Don't return — fall through to decrypt and inspect
                } else {
                    return None;
                }
            }

            let after_header = &mac_payload[consumed..];
            let buf = unsafe { &mut *PROCESS_INCOMING_NWK_BUF.get() };
            let len;

            if header.frame_control.security {
                let count = nwk.rx_security_stats().secured_frames.wrapping_add(1);
                nwk.rx_security_stats_mut().secured_frames = count;
                // Parse NWK security auxiliary header
                let (sec_hdr, sec_consumed) =
                    match zigbee_nwk::security::NwkSecurityHeader::parse(after_header) {
                        Some(v) => v,
                        None => {
                            let count = nwk
                                .rx_security_stats()
                                .security_header_parse_failures
                                .wrapping_add(1);
                            nwk.rx_security_stats_mut().security_header_parse_failures = count;
                            rt_trace!("[RT][EFR32] nwk_sec=parse_fail");
                            log::warn!("[NWK] Failed to parse security header");
                            return None;
                        }
                    };

                // Look up decryption key by sequence number
                let key = match nwk.security().key_by_seq(sec_hdr.key_seq_number) {
                    Some(k) => k.key,
                    None => {
                        let count = nwk.rx_security_stats().missing_keys.wrapping_add(1);
                        nwk.rx_security_stats_mut().missing_keys = count;
                        rt_trace!("[RT][EFR32] nwk_key=missing seq={}", sec_hdr.key_seq_number);
                        log::warn!("[NWK] No key for seq {}", sec_hdr.key_seq_number);
                        return None;
                    }
                };

                // Replay protection — check BEFORE decrypt (don't commit yet)
                if !nwk
                    .security()
                    .check_frame_counter(&sec_hdr.source_address, sec_hdr.frame_counter)
                {
                    let count = nwk.rx_security_stats().replay_rejections.wrapping_add(1);
                    nwk.rx_security_stats_mut().replay_rejections = count;
                    rt_trace!(
                        "[RT][EFR32] nwk_replay src={:02X?} fc={}",
                        sec_hdr.source_address,
                        sec_hdr.frame_counter
                    );
                    log::warn!("[NWK] Frame counter replay detected");
                    return None;
                }

                // Build authenticated data (a = NWK header || security aux header)
                // AAD must use ACTUAL security level (5), not OTA value (0).
                let aad_len = consumed + sec_consumed;
                let aad_buf = unsafe { &mut *PROCESS_INCOMING_AAD_BUF.get() };
                let aad_copy_len = aad_len.min(aad_buf.len());
                aad_buf[..aad_copy_len].copy_from_slice(&mac_payload[..aad_copy_len]);
                // Patch security control byte at offset `consumed` with actual level 5
                aad_buf[consumed] = (aad_buf[consumed] & !0x07) | 0x05;
                let ciphertext_and_mic = &after_header[sec_consumed..];

                // Decrypt
                match nwk.security().decrypt(
                    &aad_buf[..aad_copy_len],
                    ciphertext_and_mic,
                    &key,
                    &sec_hdr,
                ) {
                    Some(plaintext) => {
                        let count = nwk.rx_security_stats().decrypt_successes.wrapping_add(1);
                        nwk.rx_security_stats_mut().decrypt_successes = count;
                        rt_trace!("[RT][EFR32] nwk_decrypt=ok len={}", plaintext.len());
                        // MIC verified — NOW commit frame counter
                        nwk.security_mut()
                            .commit_frame_counter(&sec_hdr.source_address, sec_hdr.frame_counter);
                        len = plaintext.len().min(128);
                        buf[..len].copy_from_slice(&plaintext[..len]);
                    }
                    None => {
                        let count = nwk.rx_security_stats().decrypt_failures.wrapping_add(1);
                        nwk.rx_security_stats_mut().decrypt_failures = count;
                        rt_trace!("[RT][EFR32] nwk_decrypt=fail");
                        log::warn!("[NWK] Decryption failed (MIC mismatch)");
                        return None;
                    }
                }
            } else {
                // No security — pass through
                rt_trace!("[RT][EFR32] nwk_unsecured len={}", after_header.len());
                len = after_header.len().min(128);
                buf[..len].copy_from_slice(&after_header[..len]);
            }

            (
                dst,
                src,
                header.frame_control.security,
                nwk_fc.frame_type,
                len,
            )
        };

        let (dst, src, nwk_security, frame_type, len) = nwk_indication;
        let buf = unsafe { &*PROCESS_INCOMING_NWK_BUF.get() };

        // NWK Command frames (type=1) — parse and handle at runtime level
        if frame_type == 1 {
            if len > 0 {
                let cmd_id = buf[0];
                rt_trace!(
                    "[RT][EFR32] nwk_cmd id=0x{:02X} src=0x{:04X} dst=0x{:04X} len={}",
                    cmd_id,
                    src.0,
                    dst.0,
                    len
                );
                log::info!(
                    "[RX] NWK Command id=0x{:02X} from 0x{:04X} ({} bytes)",
                    cmd_id,
                    src.0,
                    len
                );
                // NWK Leave command (0x04) — signal application to rejoin
                if cmd_id == 0x04 && len >= 2 {
                    let nwk_addr = self.bdb.zdo().nwk().nib().network_address;
                    if dst != nwk_addr {
                        rt_trace!(
                            "[RT][EFR32] ignore broadcast/foreign leave src=0x{:04X} dst=0x{:04X}",
                            src.0,
                            dst.0
                        );
                        return None;
                    }
                    let options = buf[1];
                    let remove_children = (options & 0x40) != 0;
                    let rejoin = (options & 0x20) != 0;
                    rt_trace!(
                        "[RT][EFR32] leave_req src=0x{:04X} opt=0x{:02X}",
                        src.0,
                        options
                    );
                    log::warn!(
                        "[RX] NWK Leave from 0x{:04X} (remove_children={}, rejoin={})",
                        src.0,
                        remove_children,
                        rejoin
                    );
                    // Mark as not joined so the stack stops sending
                    self.bdb.zdo_mut().nwk_mut().set_joined(false);
                    return Some(event_loop::StackEvent::LeaveRequested);
                }
            }
            return None;
        }

        // APS decryption buffer (for APS-secured frames like Transport Key)
        let aps_decrypt_buf = unsafe { &mut *PROCESS_INCOMING_APS_BUF.get() };

        // APS layer: parse APS header
        let aps_indication = self.bdb.zdo_mut().aps_mut().process_incoming_aps_frame(
            &buf[..len],
            src,
            dst,
            indication.lqi,
            nwk_security,
            aps_decrypt_buf,
        );
        let aps_indication = match aps_indication {
            Some(v) => v,
            None => {
                rt_trace!("[RT][EFR32] aps_process=none");
                return None;
            }
        };

        // Route by destination endpoint
        let dst_ep = aps_indication.dst_endpoint;
        let cluster_id = aps_indication.cluster_id;
        let profile_id = aps_indication.profile_id;
        let src_addr = match aps_indication.src_address {
            ApsAddress::Short(a) => a.0,
            _ => 0,
        };

        rt_trace!(
            "[RT][EFR32] aps dst_ep={} prof=0x{:04X} cluster=0x{:04X} src=0x{:04X} payload={}",
            dst_ep,
            profile_id,
            cluster_id,
            src_addr,
            aps_indication.payload.len()
        );
        log::info!(
            "[RX] APS dst_ep={} prof=0x{:04X} cluster=0x{:04X} src=0x{:04X} len={}",
            dst_ep,
            profile_id,
            cluster_id,
            src_addr,
            aps_indication.payload.len()
        );

        // Send APS ACK now if the incoming frame requested one. This must
        // happen for *every* endpoint (ZDO and application clusters alike),
        // not just for ZDO — otherwise the coordinator/TC retransmits ZCL
        // Read Attributes (e.g. Basic Manufacturer/Model) until the ZHA
        // interview times out, leaving the device as `unk_manufacturer /
        // unk_model` with empty endpoints. Spec: APS sub-layer ACKs precede
        // any application-level response (ZB R22 §2.2.5.1).
        let _ = self.bdb.zdo_mut().aps_mut().send_pending_aps_ack().await;

        if dst_ep == 0x00 {
            // ZDO endpoint — dispatch to ZDP handler which sends responses
            // directly through the APS layer.
            rt_trace!(
                "[RT][EFR32] zdo_req cluster=0x{:04X} from=0x{:04X} len={}",
                cluster_id,
                src_addr,
                aps_indication.payload.len()
            );
            log::info!(
                "[Runtime] ZDO request: cluster=0x{:04X} from 0x{:04X} len={}",
                cluster_id,
                src_addr,
                aps_indication.payload.len(),
            );
            match self.bdb.zdo_mut().handle_indication(&aps_indication).await {
                Ok(()) => {
                    rt_trace!("[RT][EFR32] zdo_ok cluster=0x{:04X}", cluster_id);
                    log::info!("[Runtime] ZDO OK cluster=0x{:04X}", cluster_id);
                }
                Err(e) => {
                    rt_trace!(
                        "[RT][EFR32] zdo_fail cluster=0x{:04X} err={:?}",
                        cluster_id,
                        e
                    );
                    log::warn!("[Runtime] ZDO FAIL cluster=0x{:04X}: {:?}", cluster_id, e,)
                }
            }

            // After ZDO processes Mgmt_Leave_req, execute the actual leave
            if cluster_id == 0x0034 {
                log::info!("[Runtime] Executing NLME-LEAVE after Mgmt_Leave response sent");
                let _ = self
                    .bdb
                    .zdo_mut()
                    .aps_mut()
                    .nwk_mut()
                    .nlme_leave(false)
                    .await;
                return Some(event_loop::StackEvent::Left);
            }

            return None;
        }

        // Application endpoint — parse ZCL frame
        rt_trace!(
            "[RT][EFR32] zcl ep={} cluster=0x{:04X} from=0x{:04X} len={}",
            dst_ep,
            cluster_id,
            src_addr,
            aps_indication.payload.len()
        );
        log::info!(
            "[Runtime] ZCL frame: ep={} cluster=0x{:04X} from 0x{:04X} len={}",
            dst_ep,
            cluster_id,
            src_addr,
            aps_indication.payload.len()
        );
        let zcl_frame = match ZclFrame::parse(aps_indication.payload) {
            Ok(f) => f,
            Err(_) => {
                log::warn!("[Runtime] Failed to parse ZCL frame on ep {}", dst_ep);
                return None;
            }
        };

        let cmd_id = zcl_frame.header.command_id.0;
        rt_trace!(
            "[RT][EFR32] zcl_cmd ep={} cluster=0x{:04X} cmd=0x{:02X} seq={} dir={:?} payload={}",
            dst_ep,
            cluster_id,
            cmd_id,
            zcl_frame.header.seq_number,
            zcl_frame.header.direction(),
            zcl_frame.payload.len(),
        );

        // Check if this is a Report Attributes (0x0A) — incoming report from remote
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x0A
        {
            return Some(event_loop::StackEvent::AttributeReport {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                attr_id: if aps_indication.payload.len() >= 5 {
                    u16::from_le_bytes([aps_indication.payload[3], aps_indication.payload[4]])
                } else {
                    0
                },
            });
        }

        // Check if this is a Default Response (0x0B) — received from remote
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x0B
        {
            let (resp_cmd, resp_status) = if zcl_frame.payload.len() >= 2 {
                (zcl_frame.payload[0], zcl_frame.payload[1])
            } else {
                (0, 0)
            };
            log::debug!(
                "[Runtime] Default Response for cmd 0x{:02X} status=0x{:02X} from 0x{:04X}",
                resp_cmd,
                resp_status,
                src_addr,
            );
            return Some(event_loop::StackEvent::DefaultResponse {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: resp_cmd,
                status: resp_status,
            });
        }

        // Check if this is Configure Reporting (0x06) — coordinator configuring our reports
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x06
        {
            use zigbee_zcl::foundation::reporting::{
                ConfigureReportingResponse, ConfigureReportingStatusRecord, ReportDirection,
                ReportingConfig,
            };
            let payload = zcl_frame.payload.as_slice();
            let mut response = ConfigureReportingResponse {
                records: heapless::Vec::new(),
            };
            let cluster_ref = clusters.iter().find(|c| {
                c.endpoint == dst_ep && c.cluster.cluster_id() == zigbee_zcl::ClusterId(cluster_id)
            });
            let mut i = 0usize;
            let mut records = 0usize;
            let mut parse_ok = true;
            rt_trace!(
                "[RT][EFR32] zcl_cfg_reporting ep={} cluster=0x{:04X} len={}",
                dst_ep,
                cluster_id,
                payload.len(),
            );

            while i < payload.len() {
                let direction = match payload[i] {
                    0x00 => ReportDirection::Send,
                    0x01 => ReportDirection::Receive,
                    _other => {
                        rt_trace!("[RT][EFR32] zcl_cfg bad_dir=0x{:02X}", _other);
                        parse_ok = false;
                        break;
                    }
                };
                i += 1;
                if i + 2 > payload.len() {
                    parse_ok = false;
                    break;
                }
                let attribute_id =
                    zigbee_zcl::AttributeId(u16::from_le_bytes([payload[i], payload[i + 1]]));
                i += 2;

                let cfg = if direction == ReportDirection::Send {
                    if i + 5 > payload.len() {
                        parse_ok = false;
                        break;
                    }
                    let Some(data_type) = zigbee_zcl::data_types::ZclDataType::from_u8(payload[i])
                    else {
                        rt_trace!("[RT][EFR32] zcl_cfg bad_type=0x{:02X}", payload[i]);
                        parse_ok = false;
                        break;
                    };
                    i += 1;
                    let min_interval = u16::from_le_bytes([payload[i], payload[i + 1]]);
                    i += 2;
                    let max_interval = u16::from_le_bytes([payload[i], payload[i + 1]]);
                    i += 2;
                    let reportable_change = if zigbee_zcl::data_types::is_analog_type(data_type) {
                        let Some((val, consumed)) =
                            zigbee_zcl::data_types::ZclValue::deserialize(data_type, &payload[i..])
                        else {
                            parse_ok = false;
                            break;
                        };
                        i += consumed;
                        Some(val)
                    } else {
                        None
                    };
                    ReportingConfig {
                        direction,
                        attribute_id,
                        data_type,
                        min_interval,
                        max_interval,
                        reportable_change,
                    }
                } else {
                    if i + 2 > payload.len() {
                        parse_ok = false;
                        break;
                    }
                    let timeout = u16::from_le_bytes([payload[i], payload[i + 1]]);
                    i += 2;
                    ReportingConfig {
                        direction,
                        attribute_id,
                        data_type: zigbee_zcl::data_types::ZclDataType::NoData,
                        min_interval: 0,
                        max_interval: timeout,
                        reportable_change: None,
                    }
                };

                let attr_def = cluster_ref
                    .as_ref()
                    .and_then(|c| c.cluster.attributes().find(cfg.attribute_id));
                let status = if let Some(def) = attr_def {
                    if cfg.direction == ReportDirection::Send && !def.access.is_reportable() {
                        ZclStatus::UnreportableAttribute
                    } else {
                        match self
                            .reporting
                            .configure_for_cluster(dst_ep, cluster_id, cfg.clone())
                        {
                            Ok(()) => ZclStatus::Success,
                            Err(s) => s,
                        }
                    }
                } else {
                    ZclStatus::UnsupportedAttribute
                };
                let _ = response.records.push(ConfigureReportingStatusRecord {
                    status,
                    direction: cfg.direction,
                    attribute_id: cfg.attribute_id,
                });
                records += 1;
                rt_trace!(
                    "[RT][EFR32] zcl_cfg attr=0x{:04X} dir={} status=0x{:02X}",
                    cfg.attribute_id.0,
                    cfg.direction as u8,
                    status as u8,
                );
            }

            if parse_ok && records > 0 {
                // Queue Configure Reporting Response (0x07)
                self.queue_reporting_response(
                    ShortAddress(src_addr),
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    &response,
                );
                log::info!(
                    "[Runtime] Configure Reporting: ep={} cluster=0x{:04X} ({} attrs)",
                    dst_ep,
                    cluster_id,
                    records
                );
            } else {
                rt_trace!(
                    "[RT][EFR32] zcl_cfg_reporting parse_fail ep={} cluster=0x{:04X} len={}",
                    dst_ep,
                    cluster_id,
                    zcl_frame.payload.len(),
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // Check if this is Read Reporting Config (0x08)
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x08
        {
            use zigbee_zcl::foundation::reporting::{
                ReadReportingConfigRequest, ReadReportingConfigResponse,
                ReadReportingConfigResponseRecord,
            };
            if let Some(req) = ReadReportingConfigRequest::parse(zcl_frame.payload.as_slice()) {
                let mut response = ReadReportingConfigResponse {
                    records: heapless::Vec::new(),
                };
                for rec in &req.records {
                    if let Some(cfg) = self.reporting.get_config(
                        dst_ep,
                        cluster_id,
                        rec.direction,
                        rec.attribute_id,
                    ) {
                        if rec.direction == zigbee_zcl::foundation::reporting::ReportDirection::Send
                        {
                            let _ = response.records.push(ReadReportingConfigResponseRecord {
                                status: ZclStatus::Success,
                                direction: rec.direction,
                                attribute_id: rec.attribute_id,
                                config: Some(cfg.clone()),
                                timeout: None,
                            });
                        } else {
                            // Receive direction: return timeout only
                            let _ = response.records.push(ReadReportingConfigResponseRecord {
                                status: ZclStatus::Success,
                                direction: rec.direction,
                                attribute_id: rec.attribute_id,
                                config: None,
                                timeout: Some(cfg.max_interval),
                            });
                        }
                    } else {
                        let _ = response.records.push(ReadReportingConfigResponseRecord {
                            status: ZclStatus::UnsupportedAttribute,
                            direction: rec.direction,
                            attribute_id: rec.attribute_id,
                            config: None,
                            timeout: None,
                        });
                    }
                }
                self.queue_read_reporting_response(
                    ShortAddress(src_addr),
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    &response,
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Read Attributes (0x00) ──────────────────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x00
        {
            if let Some(req) = zigbee_zcl::foundation::read_attributes::ReadAttributesRequest::parse(
                zcl_frame.payload.as_slice(),
            ) {
                rt_trace!(
                    "[RT][EFR32] zcl_read ep={} cluster=0x{:04X} attrs={} from=0x{:04X}",
                    dst_ep,
                    cluster_id,
                    req.attributes.len(),
                    src_addr,
                );
                log::info!(
                    "[ZCL] ReadAttr ep={} cluster=0x{:04X} attrs={} from 0x{:04X}",
                    dst_ep,
                    cluster_id,
                    req.attributes.len(),
                    src_addr,
                );
                // Find the cluster's attribute store
                if let Some(cr) = clusters
                    .iter()
                    .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
                {
                    let response = zigbee_zcl::foundation::read_attributes::process_read_dyn(
                        cr.cluster.attributes(),
                        &req,
                    );
                    let payload_buf = unsafe { &mut *PROCESS_INCOMING_ZCL_BUF.get() };
                    let payload_len = response.serialize(payload_buf).min(payload_buf.len());
                    rt_trace!(
                        "[RT][EFR32] zcl_read_rsp cluster=0x{:04X} len={} records={}",
                        cluster_id,
                        payload_len,
                        response.records.len(),
                    );
                    log::info!(
                        "[ZCL] ReadAttr response: {} bytes, {} records queued",
                        payload_len,
                        response.records.len(),
                    );
                    self.queue_global_response(
                        src_addr,
                        aps_indication.src_endpoint,
                        dst_ep,
                        cluster_id,
                        zcl_frame.header.seq_number,
                        0x01, // Read Attributes Response
                        &payload_buf[..payload_len],
                    );
                } else {
                    rt_trace!(
                        "[RT][EFR32] zcl_read no_cluster ep={} cluster=0x{:04X} have={}",
                        dst_ep,
                        cluster_id,
                        clusters.len(),
                    );
                    log::warn!(
                        "[ZCL] ReadAttr: no cluster found for ep={} cluster=0x{:04X} (have {} clusters)",
                        dst_ep,
                        cluster_id,
                        clusters.len(),
                    );
                }
            } else {
                rt_trace!(
                    "[RT][EFR32] zcl_read parse_fail ep={} cluster=0x{:04X} len={}",
                    dst_ep,
                    cluster_id,
                    zcl_frame.payload.len(),
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Write Attributes (0x02) ─────────────────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x02
        {
            if let Some(req) =
                zigbee_zcl::foundation::write_attributes::WriteAttributesRequest::parse(
                    zcl_frame.payload.as_slice(),
                )
                && let Some(cr) = clusters
                    .iter_mut()
                    .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
            {
                let response = zigbee_zcl::foundation::write_attributes::process_write_dyn(
                    cr.cluster.attributes_mut(),
                    &req,
                );
                let payload_buf = unsafe { &mut *PROCESS_INCOMING_ZCL_BUF.get() };
                let payload_len = response.serialize(payload_buf);
                self.queue_global_response(
                    src_addr,
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    0x04, // Write Attributes Response
                    &payload_buf[..payload_len],
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Write Attributes Undivided (0x03) ────────────────────
        // All-or-nothing: if any attribute fails, none are written.
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x03
        {
            if let Some(req) =
                zigbee_zcl::foundation::write_attributes::WriteAttributesRequest::parse(
                    zcl_frame.payload.as_slice(),
                )
                && let Some(cr) = clusters
                    .iter_mut()
                    .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
            {
                let response =
                    zigbee_zcl::foundation::write_attributes::process_write_undivided_dyn(
                        cr.cluster.attributes_mut(),
                        &req,
                    );
                let payload_buf = unsafe { &mut *PROCESS_INCOMING_ZCL_BUF.get() };
                let payload_len = response.serialize(payload_buf);
                self.queue_global_response(
                    src_addr,
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    0x04, // Write Attributes Response (same response cmd for undivided)
                    &payload_buf[..payload_len],
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Write Attributes No Response (0x05) ─────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x05
        {
            if let Some(req) =
                zigbee_zcl::foundation::write_attributes::WriteAttributesRequest::parse(
                    zcl_frame.payload.as_slice(),
                )
                && let Some(cr) = clusters
                    .iter_mut()
                    .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
            {
                let _ = zigbee_zcl::foundation::write_attributes::process_write_dyn(
                    cr.cluster.attributes_mut(),
                    &req,
                );
                // No response sent for 0x05
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Discover Attributes (0x0C) ──────────────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x0C
        {
            if let Some(req) = zigbee_zcl::foundation::discover::DiscoverAttributesRequest::parse(
                zcl_frame.payload.as_slice(),
            ) && let Some(cr) = clusters
                .iter()
                .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
            {
                let response = zigbee_zcl::foundation::discover::process_discover_dyn(
                    cr.cluster.attributes(),
                    &req,
                );
                let payload_buf = unsafe { &mut *PROCESS_INCOMING_ZCL_BUF.get() };
                let payload_len = response.serialize(payload_buf);
                self.queue_global_response(
                    src_addr,
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    0x0D, // Discover Attributes Response
                    &payload_buf[..payload_len],
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Discover Commands Received (0x11) ───────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x11
        {
            if let Some(req) = zigbee_zcl::foundation::discover::DiscoverCommandsRequest::parse(
                zcl_frame.payload.as_slice(),
            ) && let Some(cr) = clusters
                .iter()
                .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
            {
                let all = cr.cluster.received_commands();
                let response = zigbee_zcl::foundation::discover::process_discover_commands(
                    &all,
                    req.start_command_id,
                    req.max_results,
                );
                let payload_buf = unsafe { &mut *PROCESS_INCOMING_ZCL_BUF.get() };
                let payload_len = response.serialize(payload_buf);
                self.queue_global_response(
                    src_addr,
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    0x12, // Discover Commands Received Response
                    &payload_buf[..payload_len],
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Discover Commands Generated (0x13) ──────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x13
        {
            if let Some(req) = zigbee_zcl::foundation::discover::DiscoverCommandsRequest::parse(
                zcl_frame.payload.as_slice(),
            ) && let Some(cr) = clusters
                .iter()
                .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
            {
                let all = cr.cluster.generated_commands();
                let response = zigbee_zcl::foundation::discover::process_discover_commands(
                    &all,
                    req.start_command_id,
                    req.max_results,
                );
                let payload_buf = unsafe { &mut *PROCESS_INCOMING_ZCL_BUF.get() };
                let payload_len = response.serialize(payload_buf);
                self.queue_global_response(
                    src_addr,
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    0x14, // Discover Commands Generated Response
                    &payload_buf[..payload_len],
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Discover Attributes Extended (0x15) ─────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x15
        {
            if let Some(req) = zigbee_zcl::foundation::discover::DiscoverAttributesRequest::parse(
                zcl_frame.payload.as_slice(),
            ) && let Some(cr) = clusters
                .iter()
                .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
            {
                let response = zigbee_zcl::foundation::discover::process_discover_extended_dyn(
                    cr.cluster.attributes(),
                    &req,
                );
                let payload_buf = unsafe { &mut *PROCESS_INCOMING_ZCL_BUF.get() };
                let payload_len = response.serialize(payload_buf);
                self.queue_global_response(
                    src_addr,
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    0x16, // Discover Attributes Extended Response
                    &payload_buf[..payload_len],
                );
            }
            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // ── Cluster-specific command dispatch ────────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::ClusterSpecific {
            // Intercept Identify Query Response (cluster 0x0003, cmd 0x00, server→client)
            // for F&B initiator target collection
            if cluster_id == 0x0003
                && cmd_id == 0x00
                && zcl_frame.header.direction() == ClusterDirection::ServerToClient
            {
                let _ = self
                    .bdb
                    .fb_identify_responses
                    .push((src_addr, aps_indication.src_endpoint));
                log::debug!(
                    "[Runtime] F&B: Identify Query Response from 0x{:04X} ep {}",
                    src_addr,
                    aps_indication.src_endpoint,
                );
            }

            let mut cmd_status = ZclStatus::Success;
            let mut response_payload: Option<heapless::Vec<u8, 64>> = None;
            let mut cluster_found = false;

            if let Some(cr) = clusters
                .iter_mut()
                .find(|cr| cr.endpoint == dst_ep && cr.cluster.cluster_id().0 == cluster_id)
            {
                cluster_found = true;
                match cr
                    .cluster
                    .handle_command(CommandId(cmd_id), zcl_frame.payload.as_slice())
                {
                    Ok(resp) => {
                        response_payload = if resp.is_empty() { None } else { Some(resp) };
                    }
                    Err(status) => {
                        cmd_status = status;
                    }
                }

                // Groups cluster → APS group table bridge
                if cluster_id == 0x0004 {
                    // Parse group action from command ID and sync to APS table.
                    // Can't use GroupsCluster::take_action() through trait object,
                    // so we infer the action from the ZCL command directly.
                    match cmd_id {
                        0x00 if zcl_frame.payload.len() >= 2 => {
                            // Add Group — group_id is first 2 bytes of payload
                            let gid =
                                u16::from_le_bytes([zcl_frame.payload[0], zcl_frame.payload[1]]);
                            let _ = self.bdb.zdo_mut().aps_mut().apsme_add_group(
                                &zigbee_aps::apsme::ApsmeAddGroupRequest {
                                    group_address: gid,
                                    endpoint: dst_ep,
                                },
                            );
                        }
                        0x03 if zcl_frame.payload.len() >= 2 => {
                            // Remove Group — group_id is first 2 bytes
                            let gid =
                                u16::from_le_bytes([zcl_frame.payload[0], zcl_frame.payload[1]]);
                            let _ = self.bdb.zdo_mut().aps_mut().apsme_remove_group(
                                &zigbee_aps::apsme::ApsmeRemoveGroupRequest {
                                    group_address: gid,
                                    endpoint: dst_ep,
                                },
                            );
                        }
                        0x04 => {
                            // Remove All Groups
                            let _ = self.bdb.zdo_mut().aps_mut().apsme_remove_all_groups(
                                &zigbee_aps::apsme::ApsmeRemoveAllGroupsRequest {
                                    endpoint: dst_ep,
                                },
                            );
                        }
                        0x05 if zcl_frame.payload.len() >= 2 => {
                            // Add Group If Identifying — only add if Identify cluster
                            // on this endpoint has IdentifyTime > 0
                            let gid =
                                u16::from_le_bytes([zcl_frame.payload[0], zcl_frame.payload[1]]);
                            let is_identifying = clusters.iter().any(|c| {
                                c.endpoint == dst_ep
                                    && c.cluster.cluster_id().0 == 0x0003
                                    && c.cluster.attributes()
                                        .get(zigbee_zcl::AttributeId(0x0000))
                                        .map(|v| matches!(v, zigbee_zcl::data_types::ZclValue::U16(t) if *t > 0))
                                        .unwrap_or(false)
                            });
                            if is_identifying {
                                // Add to APS group table
                                let _ = self.bdb.zdo_mut().aps_mut().apsme_add_group(
                                    &zigbee_aps::apsme::ApsmeAddGroupRequest {
                                        group_address: gid,
                                        endpoint: dst_ep,
                                    },
                                );
                                // Also add to GroupsCluster internal list via CMD_ADD_GROUP
                                // (cluster's handle_command for 0x05 is a no-op; use 0x00 to sync)
                                for c in clusters.iter_mut() {
                                    if c.endpoint == dst_ep && c.cluster.cluster_id().0 == 0x0004 {
                                        let add_payload = gid.to_le_bytes();
                                        let _ = c.cluster.handle_command(
                                            CommandId(0x00), // CMD_ADD_GROUP
                                            &add_payload,
                                        );
                                    }
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }

            // Send cluster-specific response if the cluster produced one
            if let Some(resp) = response_payload {
                // Determine the response command ID.
                // For most clusters, the response uses the same cmd_id.
                // Exceptions per ZCL spec:
                // - Identify Query (0x01) → IdentifyQueryResponse (0x00)
                let response_cmd_id = match (cluster_id, cmd_id) {
                    (0x0003, 0x01) => 0x00, // Identify Query → IdentifyQueryResponse
                    _ => cmd_id,
                };
                let mut frame = ZclFrame::new_cluster_specific(
                    zcl_frame.header.seq_number,
                    CommandId(response_cmd_id),
                    ClusterDirection::ServerToClient,
                    true,
                );
                for &b in resp.as_slice() {
                    let _ = frame.payload.push(b);
                }
                let zcl_buf = unsafe { &mut *PROCESS_INCOMING_ZCL_BUF.get() };
                if let Ok(len) = frame.serialize(zcl_buf) {
                    let mut data = heapless::Vec::new();
                    for &b in &zcl_buf[..len] {
                        let _ = data.push(b);
                    }
                    if self
                        .pending_responses
                        .push(PendingZclResponse {
                            dst_addr: ShortAddress(src_addr),
                            dst_endpoint: aps_indication.src_endpoint,
                            src_endpoint: dst_ep,
                            cluster_id,
                            zcl_data: data,
                        })
                        .is_err()
                    {
                        log::warn!("[ZCL] Response queue full");
                    }
                }
            } else if cluster_found && !zcl_frame.header.disable_default_response() {
                // Only send Default Response for clusters we handle in ClusterRef.
                // Unmatched clusters (e.g. OTA 0x0019) are app-handled — don't
                // send spurious Default Responses that confuse the coordinator.
                self.queue_default_response(
                    ShortAddress(src_addr),
                    aps_indication.src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    cmd_id,
                    cmd_status,
                );
            }

            // Basic cluster factory reset → distinct event
            if cluster_id == 0x0000 && cmd_id == 0x00 {
                return Some(event_loop::StackEvent::FactoryResetRequested);
            }

            return Some(event_loop::StackEvent::CommandReceived {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: cmd_id,
                seq_number: zcl_frame.header.seq_number,
                payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice())
                    .unwrap_or_default(),
            });
        }

        // Other global commands — send Default Response for unsupported, then pass through
        if !zcl_frame.header.disable_default_response() {
            // Send UNSUP_GENERAL_COMMAND for unhandled foundation commands
            self.queue_default_response(
                ShortAddress(src_addr),
                aps_indication.src_endpoint,
                dst_ep,
                cluster_id,
                zcl_frame.header.seq_number,
                cmd_id,
                ZclStatus::UnsupGeneralCommand,
            );
        }
        Some(event_loop::StackEvent::CommandReceived {
            src_addr,
            endpoint: dst_ep,
            cluster_id,
            command_id: cmd_id,
            seq_number: zcl_frame.header.seq_number,
            payload: heapless::Vec::from_slice(zcl_frame.payload.as_slice()).unwrap_or_default(),
        })
    }

    /// Queue a ZCL Default Response to be sent in next tick().
    #[allow(clippy::too_many_arguments)]
    fn queue_default_response(
        &mut self,
        dst_addr: ShortAddress,
        dst_endpoint: u8,
        src_endpoint: u8,
        cluster_id: u16,
        seq: u8,
        triggering_cmd: u8,
        status: ZclStatus,
    ) {
        let mut frame = ZclFrame::new_global(
            seq,
            CommandId(0x0B), // Default Response
            ClusterDirection::ServerToClient,
            true,
        );
        let _ = frame.payload.push(triggering_cmd);
        let _ = frame.payload.push(status as u8);

        let mut zcl_buf = [0u8; 128];
        if let Ok(len) = frame.serialize(&mut zcl_buf) {
            let mut data = heapless::Vec::new();
            for &b in &zcl_buf[..len] {
                let _ = data.push(b);
            }
            if self
                .pending_responses
                .push(PendingZclResponse {
                    dst_addr,
                    dst_endpoint,
                    src_endpoint,
                    cluster_id,
                    zcl_data: data,
                })
                .is_err()
            {
                log::warn!("[ZCL] Response queue full");
            }
        }
    }

    /// Queue a Configure Reporting Response (0x07).
    fn queue_reporting_response(
        &mut self,
        dst_addr: ShortAddress,
        dst_endpoint: u8,
        src_endpoint: u8,
        cluster_id: u16,
        seq: u8,
        response: &zigbee_zcl::foundation::reporting::ConfigureReportingResponse,
    ) {
        let mut frame =
            ZclFrame::new_global(seq, CommandId(0x07), ClusterDirection::ServerToClient, true);
        let mut payload_buf = [0u8; 64];
        let payload_len = response.serialize(&mut payload_buf);
        for &b in &payload_buf[..payload_len] {
            let _ = frame.payload.push(b);
        }

        let mut zcl_buf = [0u8; 128];
        if let Ok(len) = frame.serialize(&mut zcl_buf) {
            let mut data = heapless::Vec::new();
            for &b in &zcl_buf[..len] {
                let _ = data.push(b);
            }
            if self
                .pending_responses
                .push(PendingZclResponse {
                    dst_addr,
                    dst_endpoint,
                    src_endpoint,
                    cluster_id,
                    zcl_data: data,
                })
                .is_err()
            {
                log::warn!("[ZCL] Response queue full");
            }
        }
    }

    /// Queue a Read Reporting Configuration Response (0x09).
    fn queue_read_reporting_response(
        &mut self,
        dst_addr: ShortAddress,
        dst_endpoint: u8,
        src_endpoint: u8,
        cluster_id: u16,
        seq: u8,
        response: &zigbee_zcl::foundation::reporting::ReadReportingConfigResponse,
    ) {
        let mut frame =
            ZclFrame::new_global(seq, CommandId(0x09), ClusterDirection::ServerToClient, true);
        let mut payload_buf = [0u8; 128];
        let payload_len = response.serialize(&mut payload_buf);
        for &b in &payload_buf[..payload_len] {
            let _ = frame.payload.push(b);
        }

        let mut zcl_buf = [0u8; 128];
        if let Ok(len) = frame.serialize(&mut zcl_buf) {
            let mut data = heapless::Vec::new();
            for &b in &zcl_buf[..len] {
                let _ = data.push(b);
            }
            if self
                .pending_responses
                .push(PendingZclResponse {
                    dst_addr,
                    dst_endpoint,
                    src_endpoint,
                    cluster_id,
                    zcl_data: data,
                })
                .is_err()
            {
                log::warn!("[ZCL] Response queue full");
            }
        }
    }

    /// Send a raw ZCL frame via APS→NWK→MAC.
    pub async fn send_zcl_frame(
        &mut self,
        dst_addr: ShortAddress,
        dst_endpoint: u8,
        src_endpoint: u8,
        cluster_id: u16,
        zcl_data: &[u8],
    ) -> Result<(), ()> {
        if !self.is_joined() {
            return Err(());
        }

        let req = zigbee_aps::apsde::ApsdeDataRequest {
            dst_addr_mode: zigbee_aps::ApsAddressMode::Short,
            dst_address: ApsAddress::Short(dst_addr),
            dst_endpoint,
            profile_id: 0x0104, // Home Automation
            cluster_id,
            src_endpoint,
            payload: zcl_data,
            tx_options: zigbee_aps::ApsTxOptions {
                use_nwk_key: true,
                ..zigbee_aps::ApsTxOptions::default()
            },
            radius: 0,
            alias_src_addr: None,
            alias_seq: None,
        };

        match self.bdb.zdo_mut().aps_mut().apsde_data_request(&req).await {
            Ok(_) => Ok(()),
            Err(e) => {
                log::warn!("[Runtime] ZCL frame send failed: {:?}", e);
                Err(())
            }
        }
    }

    // ── Reporting ───────────────────────────────────────────

    /// Access the reporting engine (e.g., to configure reports).
    pub fn reporting(&self) -> &ReportingEngine {
        &self.reporting
    }

    /// Mutable access to the reporting engine.
    pub fn reporting_mut(&mut self) -> &mut ReportingEngine {
        &mut self.reporting
    }

    /// Access the underlying MAC driver (e.g., for platform-specific power management).
    pub fn mac_mut(&mut self) -> &mut M {
        self.bdb.zdo_mut().nwk_mut().mac_mut()
    }

    /// Check if any attribute reports are due for a cluster and send them.
    ///
    /// Call this after updating cluster attributes (e.g., after reading sensors).
    /// The reporting engine checks configured min/max intervals and value changes,
    /// then sends a ZCL Report Attributes (0x0A) frame if needed.
    ///
    /// Returns `true` if a report was sent.
    ///
    /// # Example
    /// ```rust,no_run,ignore
    /// temp_cluster.set_temperature(2350);
    /// let sent = device.check_and_send_cluster_reports(
    ///     1,          // endpoint
    ///     0x0402,     // Temperature Measurement cluster
    ///     temp_cluster.attributes(),
    /// ).await;
    /// ```
    pub async fn check_and_send_cluster_reports(
        &mut self,
        endpoint: u8,
        cluster_id: u16,
        store: &dyn zigbee_zcl::clusters::AttributeStoreAccess,
    ) -> bool {
        // We need to work through the reporting engine, which requires AttributeStore<N>.
        // Since we have a trait object, we build reports manually by checking each config.
        use zigbee_zcl::foundation::reporting::{AttributeReport, ReportAttributes};

        let mut reports: heapless::Vec<AttributeReport, 16> = heapless::Vec::new();
        self.reporting
            .check_and_collect_dyn(endpoint, cluster_id, store, &mut reports);

        if reports.is_empty() {
            return false;
        }

        let report = ReportAttributes { reports };
        self.send_report(endpoint, cluster_id, &report)
            .await
            .is_ok()
    }

    // ── ZCL global command response helpers ──────────────────

    /// Queue a ZCL global command response for sending in the next tick.
    ///
    /// Used by applications to respond to Read Attributes (0x00→0x01),
    /// Write Attributes (0x02→0x04), and Discover Attributes (0x0C→0x0D).
    #[allow(clippy::too_many_arguments)]
    pub fn queue_global_response(
        &mut self,
        dst_addr: u16,
        dst_endpoint: u8,
        src_endpoint: u8,
        cluster_id: u16,
        seq: u8,
        response_cmd: u8,
        payload: &[u8],
    ) {
        let mut frame = ZclFrame::new_global(
            seq,
            CommandId(response_cmd),
            ClusterDirection::ServerToClient,
            true,
        );
        for &b in payload {
            if frame.payload.push(b).is_err() {
                rt_trace!(
                    "[RT][EFR32] zcl_queue payload_truncated cluster=0x{:04X} cap={}",
                    cluster_id,
                    frame.payload.capacity(),
                );
                break;
            }
        }

        let mut zcl_buf = [0u8; 256];
        if let Ok(len) = frame.serialize(&mut zcl_buf) {
            let mut data = heapless::Vec::new();
            for &b in &zcl_buf[..len] {
                if data.push(b).is_err() {
                    rt_trace!(
                        "[RT][EFR32] zcl_queue frame_truncated cluster=0x{:04X} len={} cap={}",
                        cluster_id,
                        len,
                        data.capacity(),
                    );
                    return;
                }
            }
            rt_trace!(
                "[RT][EFR32] zcl_queue dst=0x{:04X} src_ep={} dst_ep={} cluster=0x{:04X} len={}",
                dst_addr,
                src_endpoint,
                dst_endpoint,
                cluster_id,
                data.len(),
            );
            if self
                .pending_responses
                .push(PendingZclResponse {
                    dst_addr: ShortAddress(dst_addr),
                    dst_endpoint,
                    src_endpoint,
                    cluster_id,
                    zcl_data: data,
                })
                .is_err()
            {
                rt_trace!("[RT][EFR32] zcl_queue full");
                log::warn!("[ZCL] Response queue full");
            }
        } else {
            rt_trace!(
                "[RT][EFR32] zcl_queue serialize_fail cluster=0x{:04X}",
                cluster_id,
            );
        }
    }

    // ── Layer access (for advanced use) ─────────────────────

    /// Access the BDB layer.
    pub fn bdb(&self) -> &BdbLayer<M> {
        &self.bdb
    }

    /// Mutable access to the BDB layer.
    pub fn bdb_mut(&mut self) -> &mut BdbLayer<M> {
        &mut self.bdb
    }

    /// Re-send Device_annce broadcast. Useful after join to retry if
    /// the coordinator missed the initial announcement.
    pub async fn send_device_annce(&mut self) -> Result<(), zigbee_zdo::ZdpStatus> {
        let nwk_addr = self.bdb.zdo().local_nwk_addr();
        let ieee_addr = self.bdb.zdo().local_ieee_addr();
        self.bdb.zdo_mut().device_annce(nwk_addr, ieee_addr).await
    }

    /// Send End Device Timeout Request to parent.
    ///
    /// Requests the maximum timeout (~11 days) so the parent keeps our
    /// entry during extended sleep. Call after join/rejoin.
    /// Only sends for end devices (no-op for routers).
    pub async fn send_ed_timeout_request(&mut self) {
        let _ = self.bdb.zdo_mut().nwk_mut().send_ed_timeout_request().await;
    }
}
