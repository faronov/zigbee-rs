//! Zigbee PRO R22 ZDO (Zigbee Device Object) / ZDP (Zigbee Device Profile).
//!
//! This crate implements the ZDO layer sitting on top of APS endpoint 0.
//! It handles device and service discovery, binding management, network
//! management, and the Device_annce broadcast.
//!
//! # Architecture
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  Application / ZCL / BDB                │
//! └───────────────┬─────────────────────────┘
//! ┌───────────────┴─────────────────────────┐
//! │  ZDO Layer (this crate)                 │
//! │  ├── descriptors   — node/power/simple  │
//! │  ├── discovery     — addr/desc/EP/match │
//! │  ├── binding_mgmt  — bind/unbind        │
//! │  ├── network_mgmt  — mgmt LQI/RTG/…    │
//! │  ├── device_announce                    │
//! │  └── handler       — ZDP dispatcher     │
//! └───────────────┬─────────────────────────┘
//!                 │  APS endpoint 0
//! ┌───────────────┴─────────────────────────┐
//! │  APS Layer (zigbee-aps)                 │
//! └─────────────────────────────────────────┘
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

pub mod binding_mgmt;
pub mod descriptors;
pub mod device_announce;
pub mod discovery;
pub mod handler;
pub mod network_mgmt;

use zigbee_aps::ApsLayer;
use zigbee_aps::binding::BindingEntry;
use zigbee_mac::MacDriver;
use zigbee_nwk::nlme::{JoinMethod, NetworkDescriptor};
use zigbee_nwk::{NwkLayer, NwkStatus};
use zigbee_types::{ChannelMask, IeeeAddress, ShortAddress};

use crate::descriptors::{NodeDescriptor, PowerDescriptor, SimpleDescriptor};

// ── Well-known ZDP constants ────────────────────────────────────

/// ZDO endpoint — all ZDP traffic is carried on APS endpoint 0.
pub const ZDO_ENDPOINT: u8 = 0x00;

/// Zigbee Device Profile identifier.
pub const ZDP_PROFILE_ID: u16 = 0x0000;

/// Broadcast address for RxOnWhenIdle devices (used for Device_annce).
pub const BROADCAST_RX_ON_IDLE: u16 = 0xFFFD;

// ── ZDP cluster identifiers ────────────────────────────────────

// Device and service discovery
pub const NWK_ADDR_REQ: u16 = 0x0000;
pub const NWK_ADDR_RSP: u16 = 0x8000;
pub const IEEE_ADDR_REQ: u16 = 0x0001;
pub const IEEE_ADDR_RSP: u16 = 0x8001;
pub const NODE_DESC_REQ: u16 = 0x0002;
pub const NODE_DESC_RSP: u16 = 0x8002;
pub const POWER_DESC_REQ: u16 = 0x0003;
pub const POWER_DESC_RSP: u16 = 0x8003;
pub const SIMPLE_DESC_REQ: u16 = 0x0004;
pub const SIMPLE_DESC_RSP: u16 = 0x8004;
pub const ACTIVE_EP_REQ: u16 = 0x0005;
pub const ACTIVE_EP_RSP: u16 = 0x8005;
pub const MATCH_DESC_REQ: u16 = 0x0006;
pub const MATCH_DESC_RSP: u16 = 0x8006;
pub const DEVICE_ANNCE: u16 = 0x0013;

// Binding management
pub const BIND_REQ: u16 = 0x0021;
pub const BIND_RSP: u16 = 0x8021;
pub const UNBIND_REQ: u16 = 0x0022;
pub const UNBIND_RSP: u16 = 0x8022;

// Network management
pub const MGMT_LQI_REQ: u16 = 0x0031;
pub const MGMT_LQI_RSP: u16 = 0x8031;
pub const MGMT_RTG_REQ: u16 = 0x0032;
pub const MGMT_RTG_RSP: u16 = 0x8032;
pub const MGMT_BIND_REQ: u16 = 0x0033;
pub const MGMT_BIND_RSP: u16 = 0x8033;
pub const MGMT_LEAVE_REQ: u16 = 0x0034;
pub const MGMT_LEAVE_RSP: u16 = 0x8034;
pub const MGMT_PERMIT_JOINING_REQ: u16 = 0x0036;
pub const MGMT_PERMIT_JOINING_RSP: u16 = 0x8036;
pub const MGMT_NWK_UPDATE_REQ: u16 = 0x0038;
pub const MGMT_NWK_UPDATE_RSP: u16 = 0x8038;

/// Legacy module re-exporting cluster IDs for backwards compatibility.
pub mod cluster_id {
    pub use super::{
        ACTIVE_EP_REQ, ACTIVE_EP_RSP, BIND_REQ, BIND_RSP, DEVICE_ANNCE, MATCH_DESC_REQ,
        MATCH_DESC_RSP, MGMT_LEAVE_REQ, MGMT_PERMIT_JOINING_REQ, MGMT_PERMIT_JOINING_RSP,
        SIMPLE_DESC_REQ, SIMPLE_DESC_RSP, UNBIND_REQ, UNBIND_RSP,
    };
}

// ── ZDP response status codes (Zigbee spec Table 2-138) ────────

/// ZDP status returned in every ZDP response frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ZdpStatus {
    Success = 0x00,
    InvRequestType = 0x80,
    DeviceNotFound = 0x81,
    InvalidEp = 0x82,
    NotActive = 0x83,
    NotSupported = 0x84,
    Timeout = 0x85,
    NoMatch = 0x86,
    TableFull = 0x87,
    NoEntry = 0x88,
    NoDescriptor = 0x89,
}

impl ZdpStatus {
    /// Parse a ZDP status byte.  Returns `None` for unknown codes.
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(Self::Success),
            0x80 => Some(Self::InvRequestType),
            0x81 => Some(Self::DeviceNotFound),
            0x82 => Some(Self::InvalidEp),
            0x83 => Some(Self::NotActive),
            0x84 => Some(Self::NotSupported),
            0x85 => Some(Self::Timeout),
            0x86 => Some(Self::NoMatch),
            0x87 => Some(Self::TableFull),
            0x88 => Some(Self::NoEntry),
            0x89 => Some(Self::NoDescriptor),
            _ => None,
        }
    }
}

/// Backwards-compatible alias.
pub type ZdoStatus = ZdpStatus;

impl From<NwkStatus> for ZdpStatus {
    fn from(s: NwkStatus) -> Self {
        match s {
            NwkStatus::Success => ZdpStatus::Success,
            NwkStatus::NoNetworks => ZdpStatus::DeviceNotFound,
            NwkStatus::NotPermitted => ZdpStatus::NotSupported,
            NwkStatus::InvalidRequest => ZdpStatus::InvRequestType,
            NwkStatus::NeighborTableFull => ZdpStatus::TableFull,
            _ => ZdpStatus::NotActive,
        }
    }
}

// ── ZDO error type ──────────────────────────────────────────────

/// Errors originating from the ZDO layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZdoError {
    /// Serialization buffer is too small for the frame.
    BufferTooSmall,
    /// Input data is shorter than required by the frame format.
    InvalidLength,
    /// A parsed field contains an invalid / reserved value.
    InvalidData,
    /// The underlying APS layer returned an error.
    ApsError(zigbee_aps::ApsStatus),
    /// An internal fixed-capacity table is full.
    TableFull,
}

// ── The ZDO layer ───────────────────────────────────────────────

/// Zigbee Device Object layer, generic over the MAC driver.
///
/// Owns the APS layer and all ZDO-local state (descriptors, endpoint
/// registry, address caches, etc.).
pub struct ZdoLayer<M: MacDriver> {
    /// Underlying APS layer.
    aps: ApsLayer<M>,
    /// ZDP transaction-sequence-number counter.
    seq: u8,
    /// Registered application endpoints with their simple descriptors.
    endpoints: heapless::Vec<SimpleDescriptor, 32>,
    /// This node's node descriptor.
    node_descriptor: NodeDescriptor,
    /// This node's power descriptor.
    power_descriptor: PowerDescriptor,
    /// Cached local NWK (short) address.
    local_nwk_addr: ShortAddress,
    /// Cached local IEEE (extended) address.
    local_ieee_addr: IeeeAddress,
    /// Pending ZDP request-response table for TSN correlation.
    pending_responses: [PendingZdpResponse; MAX_PENDING_ZDP],
}

/// Maximum concurrent pending ZDP requests.
const MAX_PENDING_ZDP: usize = 4;

/// A pending ZDP request awaiting its response.
#[derive(Clone)]
struct PendingZdpResponse {
    active: bool,
    tsn: u8,
    /// Expected response cluster ID.
    rsp_cluster: u16,
    /// Response payload (copied when received).
    payload: heapless::Vec<u8, 128>,
    /// Whether the response has been received.
    completed: bool,
}

impl Default for PendingZdpResponse {
    fn default() -> Self {
        Self {
            active: false,
            tsn: 0,
            rsp_cluster: 0,
            payload: heapless::Vec::new(),
            completed: false,
        }
    }
}

impl<M: MacDriver> ZdoLayer<M> {
    /// Create a new ZDO layer wrapping the given APS layer.
    pub fn new(aps: ApsLayer<M>) -> Self {
        Self {
            aps,
            seq: 0,
            endpoints: heapless::Vec::new(),
            node_descriptor: NodeDescriptor::default(),
            power_descriptor: PowerDescriptor::default(),
            local_nwk_addr: ShortAddress::UNASSIGNED,
            local_ieee_addr: [0u8; 8],
            pending_responses: core::array::from_fn(|_| PendingZdpResponse::default()),
        }
    }

    // ── ZDP sequence number ─────────────────────────────────

    /// Allocate the next ZDP transaction-sequence number (wrapping).
    pub fn next_seq(&mut self) -> u8 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    // ── Pending ZDP request-response ────────────────────────

    /// Register a pending request, returns the slot index.
    fn register_pending(&mut self, tsn: u8, rsp_cluster: u16) -> Option<usize> {
        for (i, slot) in self.pending_responses.iter_mut().enumerate() {
            if !slot.active {
                slot.active = true;
                slot.tsn = tsn;
                slot.rsp_cluster = rsp_cluster;
                slot.payload.clear();
                slot.completed = false;
                return Some(i);
            }
        }
        None
    }

    /// Try to deliver an incoming ZDP response to a pending request.
    /// Returns true if the response was consumed.
    pub fn deliver_response(&mut self, cluster: u16, tsn: u8, payload: &[u8]) -> bool {
        for slot in &mut self.pending_responses {
            if slot.active && !slot.completed && slot.rsp_cluster == cluster && slot.tsn == tsn {
                slot.payload.clear();
                for &b in payload {
                    let _ = slot.payload.push(b);
                }
                slot.completed = true;
                return true;
            }
        }
        false
    }

    /// Check if a pending request at the given slot has completed and take the result.
    pub fn take_response(&mut self, slot: usize) -> Option<heapless::Vec<u8, 128>> {
        if slot < MAX_PENDING_ZDP && self.pending_responses[slot].completed {
            self.pending_responses[slot].active = false;
            self.pending_responses[slot].completed = false;
            let payload = self.pending_responses[slot].payload.clone();
            self.pending_responses[slot].payload.clear();
            Some(payload)
        } else {
            None
        }
    }

    /// Cancel a pending request slot.
    pub fn cancel_pending(&mut self, slot: usize) {
        if slot < MAX_PENDING_ZDP {
            self.pending_responses[slot].active = false;
            self.pending_responses[slot].completed = false;
        }
    }

    // ── Layer access ────────────────────────────────────────

    /// Immutable reference to the underlying APS layer.
    pub fn aps(&self) -> &ApsLayer<M> {
        &self.aps
    }

    /// Mutable reference to the underlying APS layer.
    pub fn aps_mut(&mut self) -> &mut ApsLayer<M> {
        &mut self.aps
    }

    /// Shortcut: immutable reference to the NWK layer.
    pub fn nwk(&self) -> &NwkLayer<M> {
        self.aps.nwk()
    }

    /// Shortcut: mutable reference to the NWK layer.
    pub fn nwk_mut(&mut self) -> &mut NwkLayer<M> {
        self.aps.nwk_mut()
    }

    // ── Endpoint registry ───────────────────────────────────

    /// Register an application endpoint with its simple descriptor.
    pub fn register_endpoint(&mut self, desc: SimpleDescriptor) -> Result<(), ZdpStatus> {
        self.endpoints.push(desc).map_err(|_| ZdpStatus::TableFull)
    }

    /// Return the list of registered simple descriptors.
    pub fn endpoints(&self) -> &[SimpleDescriptor] {
        &self.endpoints
    }

    /// Backwards-compatible alias for [`Self::endpoints`].
    pub fn local_descriptors(&self) -> &[SimpleDescriptor] {
        &self.endpoints
    }

    /// Find a registered simple descriptor by endpoint number.
    pub fn find_endpoint(&self, ep: u8) -> Option<&SimpleDescriptor> {
        self.endpoints.iter().find(|d| d.endpoint == ep)
    }

    /// Backwards-compatible alias for [`Self::find_endpoint`].
    pub fn get_local_descriptor(&self, endpoint: u8) -> Option<&SimpleDescriptor> {
        self.find_endpoint(endpoint)
    }

    // ── Node / power descriptors ────────────────────────────

    pub fn set_node_descriptor(&mut self, desc: NodeDescriptor) {
        self.node_descriptor = desc;
    }
    pub fn node_descriptor(&self) -> &NodeDescriptor {
        &self.node_descriptor
    }

    pub fn set_power_descriptor(&mut self, desc: PowerDescriptor) {
        self.power_descriptor = desc;
    }
    pub fn power_descriptor(&self) -> &PowerDescriptor {
        &self.power_descriptor
    }

    // ── Local addresses ─────────────────────────────────────

    pub fn set_local_nwk_addr(&mut self, addr: ShortAddress) {
        self.local_nwk_addr = addr;
    }
    pub fn local_nwk_addr(&self) -> ShortAddress {
        self.local_nwk_addr
    }

    pub fn set_local_ieee_addr(&mut self, addr: IeeeAddress) {
        self.local_ieee_addr = addr;
    }
    pub fn local_ieee_addr(&self) -> IeeeAddress {
        self.local_ieee_addr
    }

    // ── Internal helpers used by handler / device_announce ───

    /// Send a ZDP frame (unicast to `dst`).
    async fn send_zdp_unicast(
        &mut self,
        dst: ShortAddress,
        cluster_id: u16,
        payload: &[u8],
    ) -> Result<(), ZdoError> {
        use zigbee_aps::apsde::ApsdeDataRequest;
        use zigbee_aps::{ApsAddress, ApsAddressMode, ApsTxOptions};

        let req = ApsdeDataRequest {
            dst_addr_mode: ApsAddressMode::Short,
            dst_address: ApsAddress::Short(dst),
            dst_endpoint: ZDO_ENDPOINT,
            profile_id: ZDP_PROFILE_ID,
            cluster_id,
            src_endpoint: ZDO_ENDPOINT,
            payload,
            tx_options: ApsTxOptions::default(),
            radius: 0,
            alias_src_addr: None,
            alias_seq: None,
        };
        self.aps
            .apsde_data_request(&req)
            .await
            .map(|_| ())
            .map_err(ZdoError::ApsError)
    }

    /// Send a ZDP broadcast frame (to 0xFFFD — RxOnWhenIdle devices).
    async fn send_zdp_broadcast(
        &mut self,
        cluster_id: u16,
        payload: &[u8],
    ) -> Result<(), ZdoError> {
        self.send_zdp_unicast(ShortAddress(BROADCAST_RX_ON_IDLE), cluster_id, payload)
            .await
    }

    // ── ZDP outgoing request helpers (used by BDB / app) ────

    /// Broadcast Device_annce (ZDP cluster 0x0013) after joining.
    ///
    /// Provided for backwards compatibility — prefer
    /// [`device_announce::ZdoLayer::send_device_annce`] for the full
    /// implementation.
    pub async fn device_annce(
        &mut self,
        nwk_addr: ShortAddress,
        ieee_addr: IeeeAddress,
    ) -> Result<(), ZdpStatus> {
        let mut buf = [0u8; 12]; // TSN(1) + NWK_addr(2) + IEEE_addr(8) + Cap(1)
        buf[0] = self.next_seq();
        buf[1..3].copy_from_slice(&nwk_addr.0.to_le_bytes());
        buf[3..11].copy_from_slice(&ieee_addr);
        buf[11] = self.node_descriptor.mac_capabilities;
        log::debug!(
            "[ZDO] Device_annce nwk=0x{:04X} ieee={:02X?}",
            nwk_addr.0,
            ieee_addr,
        );
        self.send_zdp_broadcast(DEVICE_ANNCE, &buf)
            .await
            .map_err(|_| ZdpStatus::NotActive)
    }

    /// Send Mgmt_Permit_Joining_req (ZDP cluster 0x0036).
    pub async fn mgmt_permit_joining_req(
        &mut self,
        dst: ShortAddress,
        duration: u8,
        tc_significance: bool,
    ) -> Result<(), ZdpStatus> {
        let mut buf = [0u8; 4]; // TSN(1) + duration(1) + tc_sig(1)
        buf[0] = self.next_seq();
        buf[1] = duration;
        buf[2] = if tc_significance { 1 } else { 0 };
        log::debug!(
            "[ZDO] Mgmt_Permit_Joining_req dst=0x{:04X} dur={} tc={}",
            dst.0,
            duration,
            tc_significance,
        );
        self.send_zdp_unicast(dst, MGMT_PERMIT_JOINING_REQ, &buf[..3])
            .await
            .map_err(|_| ZdpStatus::NotActive)
    }

    /// Request a Simple_Desc from a remote node.
    pub async fn simple_desc_req(
        &mut self,
        dst: ShortAddress,
        endpoint: u8,
    ) -> Result<descriptors::SimpleDescriptor, ZdpStatus> {
        let tsn = self.next_seq();
        let mut buf = [0u8; 4]; // TSN(1) + addr(2) + ep(1)
        buf[0] = tsn;
        buf[1..3].copy_from_slice(&dst.0.to_le_bytes());
        buf[3] = endpoint;
        log::debug!("[ZDO] Simple_Desc_req dst=0x{:04X} ep={}", dst.0, endpoint,);
        let _slot = self.register_pending(tsn, SIMPLE_DESC_RSP);
        let _ = self.send_zdp_unicast(dst, SIMPLE_DESC_REQ, &buf).await;
        Err(ZdpStatus::Timeout)
    }

    /// Request active endpoints from a remote node.
    pub async fn active_ep_req(
        &mut self,
        dst: ShortAddress,
    ) -> Result<heapless::Vec<u8, 32>, ZdpStatus> {
        let tsn = self.next_seq();
        let mut buf = [0u8; 3]; // TSN(1) + addr(2)
        buf[0] = tsn;
        buf[1..3].copy_from_slice(&dst.0.to_le_bytes());
        log::debug!("[ZDO] Active_EP_req dst=0x{:04X}", dst.0);
        let _slot = self.register_pending(tsn, ACTIVE_EP_RSP);
        let _ = self.send_zdp_unicast(dst, ACTIVE_EP_REQ, &buf).await;
        Err(ZdpStatus::Timeout)
    }

    /// Create a binding on a remote device via Bind_req (0x0021).
    ///
    /// Sends the request and registers it for response matching.
    /// The response can be checked via `take_response()`.
    /// Returns the pending slot index on success, or Err on send failure.
    pub async fn bind_req(
        &mut self,
        dst: ShortAddress,
        entry: &BindingEntry,
    ) -> Result<(), ZdpStatus> {
        let tsn = self.next_seq();
        let mut buf = [0u8; 32];
        buf[0] = tsn;
        buf[1..9].copy_from_slice(&entry.src_addr);
        buf[9] = entry.src_endpoint;
        buf[10..12].copy_from_slice(&entry.cluster_id.to_le_bytes());
        log::debug!(
            "[ZDO] Bind_req dst=0x{:04X} cluster=0x{:04X}",
            dst.0,
            entry.cluster_id,
        );
        let _slot = self.register_pending(tsn, BIND_RSP);
        self.send_zdp_unicast(dst, BIND_REQ, &buf[..12])
            .await
            .map_err(|_| ZdpStatus::Timeout)
    }

    /// Remove a binding on a remote device via Unbind_req (0x0022).
    pub async fn unbind_req(
        &mut self,
        dst: ShortAddress,
        entry: &BindingEntry,
    ) -> Result<(), ZdpStatus> {
        let tsn = self.next_seq();
        let mut buf = [0u8; 32];
        buf[0] = tsn;
        buf[1..9].copy_from_slice(&entry.src_addr);
        buf[9] = entry.src_endpoint;
        buf[10..12].copy_from_slice(&entry.cluster_id.to_le_bytes());
        log::debug!(
            "[ZDO] Unbind_req dst=0x{:04X} cluster=0x{:04X}",
            dst.0,
            entry.cluster_id,
        );
        let _slot = self.register_pending(tsn, UNBIND_RSP);
        self.send_zdp_unicast(dst, UNBIND_REQ, &buf[..12])
            .await
            .map_err(|_| ZdpStatus::Timeout)
    }

    /// Send Match_Desc_req to discover endpoints with matching clusters.
    pub async fn match_desc_req(
        &mut self,
        dst: ShortAddress,
        profile_id: u16,
        input_clusters: &[u16],
        output_clusters: &[u16],
    ) -> Result<heapless::Vec<u8, 32>, ZdpStatus> {
        let tsn = self.next_seq();
        let mut buf = [0u8; 64];
        buf[0] = tsn;
        buf[1..3].copy_from_slice(&dst.0.to_le_bytes());
        buf[3..5].copy_from_slice(&profile_id.to_le_bytes());
        buf[5] = input_clusters.len() as u8;
        let mut off = 6;
        for &c in input_clusters {
            buf[off..off + 2].copy_from_slice(&c.to_le_bytes());
            off += 2;
        }
        buf[off] = output_clusters.len() as u8;
        off += 1;
        for &c in output_clusters {
            buf[off..off + 2].copy_from_slice(&c.to_le_bytes());
            off += 2;
        }
        log::debug!(
            "[ZDO] Match_Desc_req dst=0x{:04X} profile=0x{:04X} in={} out={}",
            dst.0,
            profile_id,
            input_clusters.len(),
            output_clusters.len(),
        );
        let _slot = self.register_pending(tsn, MATCH_DESC_RSP);
        let _ = self
            .send_zdp_unicast(dst, MATCH_DESC_REQ, &buf[..off])
            .await;
        // Response will be delivered asynchronously via handle_indication
        Err(ZdpStatus::Timeout)
    }

    // ── NWK primitive wrappers ──────────────────────────────

    /// Discover available networks (wraps NLME-NETWORK-DISCOVERY).
    pub async fn nlme_network_discovery(
        &mut self,
        channel_mask: ChannelMask,
        scan_duration: u8,
    ) -> Result<heapless::Vec<NetworkDescriptor, 16>, ZdpStatus> {
        self.nwk_mut()
            .nlme_network_discovery(channel_mask, scan_duration)
            .await
            .map_err(ZdpStatus::from)
    }

    /// Join a network (wraps NLME-JOIN with association method).
    pub async fn nlme_join(
        &mut self,
        network: &NetworkDescriptor,
    ) -> Result<ShortAddress, ZdpStatus> {
        self.nwk_mut()
            .nlme_join(network, JoinMethod::Association)
            .await
            .map_err(ZdpStatus::from)
    }

    /// Form a new network — coordinator only (wraps NLME-NETWORK-FORMATION).
    pub async fn nlme_network_formation(
        &mut self,
        channel_mask: ChannelMask,
        scan_duration: u8,
    ) -> Result<(), ZdpStatus> {
        self.nwk_mut()
            .nlme_network_formation(channel_mask, scan_duration)
            .await
            .map_err(ZdpStatus::from)
    }

    /// Open or close permit joining (wraps NLME-PERMIT-JOINING).
    pub async fn nlme_permit_joining(&mut self, duration: u8) -> Result<(), ZdpStatus> {
        self.nwk_mut()
            .nlme_permit_joining(duration)
            .await
            .map_err(ZdpStatus::from)
    }

    /// Start router operation (wraps NLME-START-ROUTER).
    pub async fn nlme_start_router(&mut self) -> Result<(), ZdpStatus> {
        self.nwk_mut()
            .nlme_start_router()
            .await
            .map_err(ZdpStatus::from)
    }

    /// Reset the stack (wraps NLME-RESET).
    pub async fn nlme_reset(&mut self, warm_start: bool) -> Result<(), ZdpStatus> {
        self.nwk_mut()
            .nlme_reset(warm_start)
            .await
            .map_err(ZdpStatus::from)
    }
}
