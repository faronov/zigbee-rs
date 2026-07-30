//! Zigbee PRO R22 Network (NWK) Layer.
//!
//! This crate implements the NWK layer of the Zigbee stack, providing:
//! - Network discovery, formation, join, and leave
//! - NWK frame construction and parsing
//! - Neighbor and routing tables
//! - NWK data service (NLDE-DATA)
//! - NWK security (encryption/decryption of NWK frames)
//!
//! # Architecture
//! ```text
//! ┌──────────────────────────────────────┐
//! │  APS Layer (zigbee-aps)              │
//! └──────────────┬───────────────────────┘
//!                │ NLDE-DATA / NLME-*
//! ┌──────────────┴───────────────────────┐
//! │  NWK Layer (this crate)              │
//! │  ├── nlme: management primitives     │
//! │  ├── nlde: data service              │
//! │  ├── nib: network information base   │
//! │  ├── frames: NWK frame codec         │
//! │  ├── neighbor: neighbor table        │
//! │  ├── routing: tree + AODV routing    │
//! │  └── security: NWK encryption        │
//! └──────────────┬───────────────────────┘
//!                │ MacDriver trait
//! ┌──────────────┴───────────────────────┐
//! │  MAC Layer (zigbee-mac)              │
//! └──────────────────────────────────────┘
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

#[cfg(test)]
extern crate std;

pub mod frames;
pub mod indirect;
pub mod neighbor;
pub mod nib;
pub mod nlde;
pub mod nlme;
pub mod nwk_commands;
pub mod routing;
pub mod security;

use zigbee_mac::{AddressMode, CapabilityInfo, MacDriver, MacError, McpsDataRequest, TxOptions};
use zigbee_types::{IeeeAddress, MacAddress, ShortAddress};

const UNAUTHENTICATED_CHILD_TIMEOUT_SECS: u16 = 10;

/// NWK layer status codes (Zigbee spec Table 3-70)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NwkStatus {
    Success = 0x00,
    InvalidParameter = 0xC1,
    InvalidRequest = 0xC2,
    NotPermitted = 0xC3,
    StartupFailure = 0xC4,
    AlreadyPresent = 0xC5,
    SyncFailure = 0xC6,
    NeighborTableFull = 0xC7,
    UnknownDevice = 0xC8,
    UnsupportedAttribute = 0xC9,
    NoNetworks = 0xCA,
    MaxFrmCounterReached = 0xCC,
    NoKey = 0xCD,
    BadCcmOutput = 0xCE,
    RouteDiscoveryFailed = 0xD0,
    RouteError = 0xD1,
    BtTableFull = 0xD2,
    FrameNotBuffered = 0xD3,
    FrameTooLong = 0xD4,
}

/// Device type in the Zigbee network
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceType {
    Coordinator,
    Router,
    EndDevice,
}

/// Available child slots advertised in Zigbee beacon capacity bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChildCapacity {
    pub router: bool,
    pub end_device: bool,
}

/// Result of servicing one MAC Data Request from a possible child.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChildPollOutcome {
    /// Extended-address polls are reserved for the MAC-owned Association
    /// Response transaction and are intentionally not drained here.
    AssociationResponsePoll,
    UnknownChild,
    NoData,
    Delivered {
        child: ShortAddress,
        more_pending: bool,
    },
}

/// How a Rejoin Response was delivered to the requesting child.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejoinResponseDelivery {
    Direct,
    Indirect,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NwkRxSecurityStats {
    pub secured_frames: u32,
    pub security_header_parse_failures: u32,
    pub missing_keys: u32,
    pub replay_rejections: u32,
    pub decrypt_successes: u32,
    pub decrypt_failures: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RejoinDiagnostics {
    pub stage: u8,
    pub candidate_attempts: u8,
    pub tx_attempts: u8,
    pub poll_attempts: u8,
    pub rx_frames: u8,
    pub last_status: u8,
    pub last_parent: u16,
    pub no_ack_failures: u8,
    pub channel_access_failures: u8,
    pub other_tx_failures: u8,
}

/// The NWK layer — owns all NWK state and the MAC driver.
///
/// Generic over:
/// - `M`: MAC driver implementation (ESP, nRF, mock, etc.)
///
/// # Usage
/// ```rust,no_run,ignore
/// use zigbee_nwk::NwkLayer;
/// use zigbee_mac::mock::MockMac;
///
/// let mac = MockMac::new([1,2,3,4,5,6,7,8]);
/// let mut nwk = NwkLayer::new(mac, DeviceType::EndDevice);
///
/// A deferred route reply or route-reply-forward to be sent asynchronously.
#[derive(Debug, Clone)]
pub struct PendingRouteReply {
    /// Short address to send the RREP toward
    pub next_hop: ShortAddress,
    /// Originator of the original RREQ
    pub originator: ShortAddress,
    /// Responder (the node that can reach the destination)
    pub responder: ShortAddress,
    /// Accumulated path cost
    pub path_cost: u8,
    /// Route request ID from the original RREQ
    pub route_request_id: u8,
}

/// A deferred RREQ rebroadcast (queued from sync handler, sent async).
///
/// Retained for API compatibility. Route Request propagation no longer
/// *rebroadcasts* a locally originated copy: it forwards the originator's
/// frame, which needs the original NWK header as well, so the queue holds a
/// crate-private `QueuedRreqForward` instead.
#[derive(Debug, Clone)]
pub struct PendingRreqRebroadcast {
    pub command_options: u8,
    pub route_request_id: u8,
    pub dst_addr: ShortAddress,
    pub path_cost: u8,
}

/// A Route Request this device has accepted for propagation, queued from the
/// synchronous command handler and forwarded on the next maintenance pass.
///
/// A forwarded RREQ is the *originator's* broadcast carried one hop further,
/// not a new one of ours. Everything that identifies the broadcast — the NWK
/// source address, the NWK sequence number and the end-device-initiator bit —
/// is preserved, only the radius (decremented) and the RREQ path cost change,
/// and the frame is re-secured hop by hop with this device's own IEEE address
/// and durable frame counter. Re-originating it instead would defeat every
/// receiver's broadcast transaction record and let two routers pass the same
/// discovery back and forth forever.
#[derive(Debug, Clone)]
pub(crate) struct QueuedRreqForward {
    /// Frame control of the received frame — protocol version, discover-route
    /// bits, security flag and the end-device-initiator bit are preserved.
    pub(crate) frame_control: frames::NwkFrameControl,
    /// Broadcast address the originator used (0xFFFC, 0xFFFD, 0xFFFF...).
    pub(crate) dst_addr: ShortAddress,
    /// NWK source address of the originator — never this device.
    pub(crate) originator: ShortAddress,
    /// Originator's IEEE address, when the received header carried one.
    pub(crate) src_ieee: Option<IeeeAddress>,
    /// The originator's NWK sequence number, preserved for BTR suppression.
    pub(crate) seq_number: u8,
    /// Radius to send with — already decremented from the received radius.
    pub(crate) radius: u8,
    /// RREQ command options (many-to-one bits, destination IEEE flag).
    pub(crate) command_options: u8,
    pub(crate) route_request_id: u8,
    /// RREQ destination (the concentrator for a many-to-one request).
    pub(crate) rreq_dst: ShortAddress,
    pub(crate) rreq_dst_ieee: Option<IeeeAddress>,
    /// Accumulated path cost including our link from the previous hop.
    pub(crate) path_cost: u8,
}

/// A deferred Network Status (route error) to be sent asynchronously.
#[derive(Debug, Clone)]
pub struct PendingNetworkStatus {
    /// Address to send the status toward (NWK source of the failed frame)
    pub destination: ShortAddress,
    /// Status code (e.g., 0x00 = no route available)
    pub status_code: u8,
    /// The unreachable destination that triggered the error
    pub failed_destination: ShortAddress,
}

/// // Discover networks
/// let networks = nwk.nlme_network_discovery(ChannelMask::ALL_2_4GHZ, 3).await?;
///
/// // Join best network
/// nwk.nlme_join(&networks[0]).await?;
/// ```
pub struct NwkLayer<M: MacDriver> {
    mac: M,
    nib: nib::Nib,
    neighbors: neighbor::NeighborTable,
    routing: routing::RoutingTable,
    btr: routing::BtrTable,
    security: security::NwkSecurity,
    rx_security_stats: NwkRxSecurityStats,
    rejoin_diagnostics: RejoinDiagnostics,
    device_type: DeviceType,
    joined: bool,
    /// Whether this device listens when idle.
    /// true = non-sleepy (RFD/FFD that stays awake)
    /// false = sleepy end device (polls parent for data)
    rx_on_when_idle: bool,
    /// Pending route replies to be sent asynchronously.
    #[cfg(feature = "router")]
    pending_route_replies: heapless::Vec<PendingRouteReply, 4>,
    #[cfg(not(feature = "router"))]
    pending_route_replies: heapless::Vec<PendingRouteReply, 0>,
    /// Route Requests accepted for propagation (queued from the sync handler,
    /// forwarded async by [`NwkLayer::process_pending_routing`]).
    #[cfg(feature = "router")]
    pending_rreq_forwards: heapless::Vec<QueuedRreqForward, 4>,
    #[cfg(not(feature = "router"))]
    pending_rreq_forwards: heapless::Vec<QueuedRreqForward, 0>,
    /// Route requests already acted upon, keyed by originator and request ID.
    rreq_records: routing::RreqRecordTable,
    /// Pending Network Status (route error) notifications.
    #[cfg(feature = "router")]
    pending_route_errors: heapless::Vec<PendingNetworkStatus, 4>,
    #[cfg(not(feature = "router"))]
    pending_route_errors: heapless::Vec<PendingNetworkStatus, 0>,
    /// Indirect frame queue for sleeping end device children.
    indirect: indirect::IndirectQueue,
    /// Link status periodic timer counter (seconds).
    link_status_counter: u16,
    /// Flag: link status should be sent in next async context.
    link_status_due: bool,
    /// Whether this device is operating as a concentrator (many-to-one).
    concentrator_active: bool,
    /// Concentrator type (LowRam or HighRam).
    concentrator_type: routing::ConcentratorType,
    /// Concentrator RREQ interval counter (seconds).
    concentrator_counter: u16,
    /// Concentrator RREQ interval (seconds, default 60).
    concentrator_interval: u16,
    /// Flag: concentrator RREQ should be sent in next async context.
    concentrator_rreq_due: bool,
    /// nwkConcentratorRadius — hop limit for MTOR RREQ (default 5).
    concentrator_radius: u8,
    /// Source route table — stores full relay paths from Route Records.
    source_route_table: routing::SourceRouteTable,
    /// Counter for stochastic child address assignment.
    next_child_addr_offset: u16,
    /// Lifecycle outcome of the NWK command in the frame currently being
    /// processed, collected by the layer above with
    /// [`take_command_outcome`](Self::take_command_outcome).
    ///
    /// Reset at the start of every `process_incoming_nwk_frame` call so it can
    /// only ever describe that call's frame.
    pending_command_outcome: Option<nlde::NwkCommandOutcome>,
}

impl<M: MacDriver> NwkLayer<M> {
    /// Create a new NWK layer with the given MAC driver.
    #[inline(never)]
    pub fn new(mac: M, device_type: DeviceType) -> Self {
        // Default: FFD/Router always rx_on, EndDevice defaults to true (non-sleepy)
        let rx_on_when_idle = true;
        Self {
            mac,
            nib: nib::Nib::new(),
            neighbors: neighbor::NeighborTable::new(),
            routing: routing::RoutingTable::new(),
            btr: routing::BtrTable::new(),
            security: security::NwkSecurity::new(),
            rx_security_stats: NwkRxSecurityStats::default(),
            rejoin_diagnostics: RejoinDiagnostics::default(),
            device_type,
            joined: false,
            rx_on_when_idle,
            #[cfg(feature = "router")]
            pending_route_replies: heapless::Vec::new(),
            #[cfg(not(feature = "router"))]
            pending_route_replies: heapless::Vec::new(),
            #[cfg(feature = "router")]
            pending_rreq_forwards: heapless::Vec::new(),
            #[cfg(not(feature = "router"))]
            pending_rreq_forwards: heapless::Vec::new(),
            rreq_records: routing::RreqRecordTable::new(),
            #[cfg(feature = "router")]
            pending_route_errors: heapless::Vec::new(),
            #[cfg(not(feature = "router"))]
            pending_route_errors: heapless::Vec::new(),
            indirect: indirect::IndirectQueue::new(),
            link_status_counter: 0,
            link_status_due: false,
            concentrator_active: false,
            concentrator_type: routing::ConcentratorType::LowRam,
            concentrator_counter: 0,
            concentrator_interval: 60,
            concentrator_rreq_due: false,
            concentrator_radius: 5,
            source_route_table: routing::SourceRouteTable::new(),
            next_child_addr_offset: 1,
            pending_command_outcome: None,
        }
    }

    /// Construct a NWK layer directly into caller-provided storage.
    ///
    /// # Safety
    /// `slot` must point to valid, properly aligned, uninitialized storage for `Self`.
    #[inline(never)]
    pub unsafe fn write_into(slot: *mut Self, mac: M, device_type: DeviceType) {
        unsafe {
            core::ptr::addr_of_mut!((*slot).mac).write(mac);
            core::ptr::addr_of_mut!((*slot).nib).write(nib::Nib::new());
            core::ptr::addr_of_mut!((*slot).neighbors).write(neighbor::NeighborTable::new());
            core::ptr::addr_of_mut!((*slot).routing).write(routing::RoutingTable::new());
            core::ptr::addr_of_mut!((*slot).btr).write(routing::BtrTable::new());
            core::ptr::addr_of_mut!((*slot).security).write(security::NwkSecurity::new());
            core::ptr::addr_of_mut!((*slot).rx_security_stats).write(NwkRxSecurityStats::default());
            core::ptr::addr_of_mut!((*slot).rejoin_diagnostics).write(RejoinDiagnostics::default());
            core::ptr::addr_of_mut!((*slot).device_type).write(device_type);
            core::ptr::addr_of_mut!((*slot).joined).write(false);
            core::ptr::addr_of_mut!((*slot).rx_on_when_idle).write(true);
            #[cfg(feature = "router")]
            core::ptr::addr_of_mut!((*slot).pending_route_replies).write(heapless::Vec::new());
            #[cfg(not(feature = "router"))]
            core::ptr::addr_of_mut!((*slot).pending_route_replies).write(heapless::Vec::new());
            #[cfg(feature = "router")]
            core::ptr::addr_of_mut!((*slot).pending_rreq_forwards).write(heapless::Vec::new());
            #[cfg(not(feature = "router"))]
            core::ptr::addr_of_mut!((*slot).pending_rreq_forwards).write(heapless::Vec::new());
            core::ptr::addr_of_mut!((*slot).rreq_records).write(routing::RreqRecordTable::new());
            #[cfg(feature = "router")]
            core::ptr::addr_of_mut!((*slot).pending_route_errors).write(heapless::Vec::new());
            #[cfg(not(feature = "router"))]
            core::ptr::addr_of_mut!((*slot).pending_route_errors).write(heapless::Vec::new());
            core::ptr::addr_of_mut!((*slot).indirect).write(indirect::IndirectQueue::new());
            core::ptr::addr_of_mut!((*slot).link_status_counter).write(0);
            core::ptr::addr_of_mut!((*slot).link_status_due).write(false);
            core::ptr::addr_of_mut!((*slot).concentrator_active).write(false);
            core::ptr::addr_of_mut!((*slot).concentrator_type)
                .write(routing::ConcentratorType::LowRam);
            core::ptr::addr_of_mut!((*slot).concentrator_counter).write(0);
            core::ptr::addr_of_mut!((*slot).concentrator_interval).write(60);
            core::ptr::addr_of_mut!((*slot).concentrator_rreq_due).write(false);
            core::ptr::addr_of_mut!((*slot).concentrator_radius).write(5);
            core::ptr::addr_of_mut!((*slot).source_route_table)
                .write(routing::SourceRouteTable::new());
            core::ptr::addr_of_mut!((*slot).next_child_addr_offset).write(1);
            core::ptr::addr_of_mut!((*slot).pending_command_outcome).write(None);
        }
    }

    /// Set rx_on_when_idle (call before joining).
    /// false = sleepy end device (must poll parent for indirect frames).
    /// true = device listens continuously (default for Efekta sensor).
    pub fn set_rx_on_when_idle(&mut self, rx_on: bool) {
        self.rx_on_when_idle = rx_on;
    }

    /// Get rx_on_when_idle setting.
    pub fn rx_on_when_idle(&self) -> bool {
        self.rx_on_when_idle
    }

    pub fn rejoin_diagnostics(&self) -> RejoinDiagnostics {
        self.rejoin_diagnostics
    }

    pub fn reset_rejoin_diagnostics(&mut self) {
        self.rejoin_diagnostics = RejoinDiagnostics::default();
    }

    /// Get reference to the NIB.
    pub fn nib(&self) -> &nib::Nib {
        &self.nib
    }

    /// Get mutable reference to the NIB.
    pub fn nib_mut(&mut self) -> &mut nib::Nib {
        &mut self.nib
    }

    /// Whether this device has joined a network.
    pub fn is_joined(&self) -> bool {
        self.joined
    }

    /// Set the joined flag (used during silent resume after NV restore).
    pub fn set_joined(&mut self, joined: bool) {
        self.joined = joined;
    }

    /// Take the lifecycle outcome of the NWK command just processed.
    ///
    /// Call this immediately after [`process_incoming_nwk_frame`] — that call
    /// clears the slot on entry and sets it only for the frame it processed,
    /// so a stale Leave can never be attributed to a later frame.
    ///
    /// Routing and neighbour maintenance commands (RREQ, RREP, Route Record,
    /// Link Status, Network Status) are handled entirely inside the NWK layer
    /// and never produce an outcome here. Only commands that change this
    /// device's network lifecycle do, and the NWK layer has already applied
    /// the network-level effect (including dropping the joined flag) after
    /// validating security and parent authorization.
    ///
    /// [`process_incoming_nwk_frame`]: Self::process_incoming_nwk_frame
    pub fn take_command_outcome(&mut self) -> Option<nlde::NwkCommandOutcome> {
        self.pending_command_outcome.take()
    }

    /// Get the device type.
    pub fn device_type(&self) -> DeviceType {
        self.device_type
    }

    /// Whether this build and device type may forward traffic for other nodes.
    ///
    /// Routing needs the `router` feature: without it the routing, broadcast
    /// transaction (BTR), source-route and indirect tables are compiled to
    /// zero capacity, so a device could neither suppress broadcast duplicates
    /// nor remember a next hop. Relaying in that configuration would produce
    /// unbounded rebroadcast storms, so forwarding is refused outright instead
    /// of being attempted with tables that cannot hold state.
    ///
    /// Routing also requires membership of the network. A device that has not
    /// joined yet — or that was told to leave — has no PAN ID, no network
    /// address and no key material it may legitimately act under, so relaying,
    /// rebroadcasting and route discovery are all refused. Otherwise a
    /// commissioning or orphaned router would transmit on behalf of a network
    /// it is not part of and burn durably reserved outgoing frame counters
    /// doing it.
    pub fn can_route(&self) -> bool {
        cfg!(feature = "router") && self.device_type != DeviceType::EndDevice && self.joined
    }

    /// Get reference to the MAC driver.
    pub fn mac(&self) -> &M {
        &self.mac
    }

    /// Get mutable reference to the MAC driver.
    pub fn mac_mut(&mut self) -> &mut M {
        &mut self.mac
    }

    /// Get reference to the NWK security context.
    pub fn security(&self) -> &security::NwkSecurity {
        &self.security
    }

    /// Get mutable reference to the NWK security context.
    pub fn security_mut(&mut self) -> &mut security::NwkSecurity {
        &mut self.security
    }

    pub fn rx_security_stats(&self) -> NwkRxSecurityStats {
        self.rx_security_stats
    }

    /// Mutable security telemetry for stack front-ends that perform NWK
    /// receive processing directly.
    pub fn rx_security_stats_mut(&mut self) -> &mut NwkRxSecurityStats {
        &mut self.rx_security_stats
    }

    /// Read-only access to the neighbor table.
    pub fn neighbor_table(&self) -> &neighbor::NeighborTable {
        &self.neighbors
    }

    /// Remove a device that sent a NWK Leave indication.
    pub fn remove_neighbor(&mut self, address: ShortAddress) {
        self.neighbors.remove(address);
    }

    /// Read-only access to the routing table.
    pub fn routing_table(&self) -> &routing::RoutingTable {
        &self.routing
    }

    /// Look up a short address by IEEE address from the neighbor table.
    pub fn find_short_by_ieee(&self, ieee: &IeeeAddress) -> Option<ShortAddress> {
        self.neighbors.find_by_ieee(ieee).map(|e| e.network_address)
    }

    /// Look up an IEEE address by short address from the neighbor table.
    pub fn find_ieee_by_short(&self, short: ShortAddress) -> Option<IeeeAddress> {
        for entry in self.neighbors.iter() {
            if entry.network_address == short {
                return Some(entry.ieee_address);
            }
        }
        None
    }

    /// Update or insert a neighbor entry when a Device_annce is received.
    /// This keeps the NWK address → IEEE address mapping current.
    pub fn update_neighbor_address(&mut self, nwk_addr: ShortAddress, ieee_addr: IeeeAddress) {
        // Try to update existing entry by NWK addr or IEEE addr
        for entry in self.neighbors.iter_mut_all() {
            if entry.network_address == nwk_addr || entry.ieee_address == ieee_addr {
                entry.network_address = nwk_addr;
                entry.ieee_address = ieee_addr;
                return;
            }
        }
        // Not found — add a new entry via add_or_update
        let mut entry = neighbor::NeighborEntry::new_from_annce(nwk_addr, ieee_addr);
        entry.active = true;
        let _ = self.neighbors.add_or_update(entry);
    }

    /// Periodic router maintenance — call every second from the runtime tick.
    ///
    /// Ages BTR and indirect queues, triggers periodic link status broadcasts,
    /// expires stale routing entries, and schedules concentrator RREQs.
    pub fn tick_router_maintenance(&mut self, elapsed_secs: u16) {
        // Age BTR entries, route-request forwarding records and the indirect
        // queue. A forwarding record must outlive its BTR entry, so both are
        // aged on the same tick.
        for _ in 0..elapsed_secs {
            self.btr.age();
            self.rreq_records.age();
            self.neighbors.age_tick();
            let expired_children = self.indirect.age();
            for child in expired_children {
                if !self.indirect.has_pending(child) {
                    let _ = self.mac.set_indirect_data_pending(
                        MacAddress::Short(self.nib.pan_id, child),
                        false,
                    );
                }
            }
            let unauthorized = self
                .neighbors
                .expire_unauthenticated(UNAUTHENTICATED_CHILD_TIMEOUT_SECS);
            for child in unauthorized {
                self.indirect.remove_all(child);
                self.routing.remove(child);
                let _ = self
                    .mac
                    .set_indirect_data_pending(MacAddress::Short(self.nib.pan_id, child), false);
                log::warn!("[NWK] Unauthenticated child 0x{:04X} expired", child.0);
            }
        }

        // Age routing table entries
        self.routing.age_tick();

        // Fail route discoveries that never produced a Route Reply so a later
        // data request may retry instead of being suppressed forever.
        if self.can_route() {
            let now = self.mac.monotonic_micros();
            self.routing
                .expire_discoveries(crate::routing::ROUTE_DISCOVERY_TIMEOUT_US, now);
        }

        // Periodic link status for routers/coordinators
        if self.device_type != DeviceType::EndDevice && self.joined {
            self.link_status_counter = self.link_status_counter.saturating_add(elapsed_secs);
            if self.link_status_counter >= 15 {
                self.link_status_counter = 0;
                self.link_status_due = true;
            }
        }

        // Periodic many-to-one RREQ for concentrators
        if self.concentrator_active && self.joined {
            self.concentrator_counter = self.concentrator_counter.saturating_add(elapsed_secs);
            if self.concentrator_counter >= self.concentrator_interval {
                self.concentrator_counter = 0;
                self.concentrator_rreq_due = true;
            }
        }
    }

    /// Whether a link status broadcast is due (set by tick, cleared after send).
    pub fn link_status_due(&self) -> bool {
        self.link_status_due
    }

    /// Clear the link status due flag after sending.
    pub fn clear_link_status_due(&mut self) {
        self.link_status_due = false;
    }

    /// Read-only access to the indirect frame queue.
    pub fn indirect_queue(&self) -> &indirect::IndirectQueue {
        &self.indirect
    }

    /// Enable concentrator mode (periodic many-to-one RREQ broadcasts).
    ///
    /// Only valid for coordinators and routers. The interval is in seconds
    /// (default 60s per Zigbee spec recommendation).
    pub fn start_concentrator(
        &mut self,
        concentrator_type: routing::ConcentratorType,
        interval_secs: u16,
        radius: u8,
    ) {
        if self.device_type == DeviceType::EndDevice {
            log::warn!("[NWK] Cannot start concentrator on end device");
            return;
        }
        self.concentrator_active = true;
        self.concentrator_type = concentrator_type;
        self.concentrator_interval = interval_secs;
        self.concentrator_counter = interval_secs; // Trigger immediately on first tick
        self.concentrator_radius = radius;
        log::info!(
            "[NWK] Concentrator mode enabled ({:?}, interval={}s, radius={})",
            concentrator_type,
            interval_secs,
            radius,
        );
    }

    /// Disable concentrator mode.
    pub fn stop_concentrator(&mut self) {
        self.concentrator_active = false;
        self.concentrator_rreq_due = false;
        log::info!("[NWK] Concentrator mode disabled");
    }

    /// Whether this device is operating as a concentrator.
    pub fn is_concentrator(&self) -> bool {
        self.concentrator_active
    }

    /// Return whether router and end-device child slots remain.
    pub fn child_capacity(&self) -> ChildCapacity {
        if !self.can_route() || !self.neighbors.has_child_slot() {
            return ChildCapacity {
                router: false,
                end_device: false,
            };
        }
        let mut routers = 0usize;
        let mut end_devices = 0usize;
        for child in self.neighbors.children() {
            match child.device_type {
                neighbor::NeighborDeviceType::Router => routers += 1,
                neighbor::NeighborDeviceType::EndDevice => end_devices += 1,
                _ => {}
            }
        }
        let total_capacity = routers + end_devices < usize::from(self.nib.max_children);
        ChildCapacity {
            router: total_capacity
                && self.nib.depth < self.nib.max_depth
                && routers < usize::from(self.nib.max_routers),
            end_device: total_capacity,
        }
    }

    /// Resolve a MAC short source to a child admitted through this parent.
    pub fn known_child_short(&self, source: &MacAddress) -> Option<ShortAddress> {
        let MacAddress::Short(pan_id, address) = source else {
            return None;
        };
        if *pan_id != self.nib.pan_id || address.0 >= 0xFFF8 {
            return None;
        }
        self.neighbors.find_by_short(*address).and_then(|entry| {
            matches!(
                entry.relationship,
                neighbor::Relationship::Child | neighbor::Relationship::UnauthenticatedChild
            )
            .then_some(*address)
        })
    }

    pub fn known_child_by_ieee(&self, ieee: &IeeeAddress) -> Option<ShortAddress> {
        self.neighbors.find_by_ieee(ieee).and_then(|entry| {
            matches!(
                entry.relationship,
                neighbor::Relationship::Child | neighbor::Relationship::UnauthenticatedChild
            )
            .then_some(entry.network_address)
        })
    }

    pub fn child_security_capable(&self, ieee: &IeeeAddress) -> Option<bool> {
        self.neighbors
            .find_by_ieee(ieee)
            .map(|entry| entry.security_capable)
    }

    pub fn child_rx_on_when_idle(&self, ieee: &IeeeAddress) -> Option<bool> {
        self.neighbors
            .find_by_ieee(ieee)
            .map(|entry| entry.rx_on_when_idle)
    }

    pub fn child_is_authorized(&self, ieee: &IeeeAddress) -> bool {
        self.neighbors
            .find_by_ieee(ieee)
            .is_some_and(|entry| entry.relationship == neighbor::Relationship::Child)
    }

    pub fn child_is_unauthenticated(&self, address: ShortAddress) -> bool {
        self.neighbors
            .find_by_short(address)
            .is_some_and(|entry| entry.relationship == neighbor::Relationship::UnauthenticatedChild)
    }

    /// Remove a child only while it is still awaiting network-key proof.
    pub fn remove_unauthenticated_child(
        &mut self,
        ieee: &IeeeAddress,
        address: ShortAddress,
    ) -> bool {
        let removable = self.neighbors.find_by_ieee(ieee).is_some_and(|entry| {
            entry.network_address == address
                && entry.relationship == neighbor::Relationship::UnauthenticatedChild
        });
        if !removable {
            return false;
        }
        self.indirect.remove_all(address);
        self.routing.remove(address);
        let _ = self
            .mac
            .set_indirect_data_pending(MacAddress::Short(self.nib.pan_id, address), false);
        self.neighbors.remove(address);
        true
    }

    /// Mark a provisionally admitted child as authorized after it proves
    /// possession of the active network key.
    pub fn authorize_child(&mut self, address: ShortAddress) -> bool {
        let Some(entry) = self.neighbors.find_by_short_mut(address) else {
            return false;
        };
        if entry.relationship != neighbor::Relationship::UnauthenticatedChild {
            return false;
        }
        entry.relationship = neighbor::Relationship::Child;
        entry.age = 0;
        true
    }

    /// Queue one already-built NWK frame for a sleepy child and arm the MAC
    /// Frame Pending state atomically from the caller's perspective.
    pub fn enqueue_indirect_for_child(
        &mut self,
        child: ShortAddress,
        frame: &[u8],
    ) -> Result<(), NwkStatus> {
        let is_sleepy_child = self.neighbors.find_by_short(child).is_some_and(|entry| {
            !entry.rx_on_when_idle
                && matches!(
                    entry.relationship,
                    neighbor::Relationship::Child | neighbor::Relationship::UnauthenticatedChild
                )
        });
        if !is_sleepy_child {
            return Err(NwkStatus::UnknownDevice);
        }
        let Some(slot) = self.indirect.enqueue_with_slot(child, frame) else {
            return Err(NwkStatus::FrameNotBuffered);
        };
        if self
            .mac
            .set_indirect_data_pending(MacAddress::Short(self.nib.pan_id, child), true)
            .is_err()
        {
            self.indirect.remove_slot(slot);
            return Err(NwkStatus::FrameNotBuffered);
        }
        Ok(())
    }

    /// Service at most one queued transaction for one physical child poll.
    ///
    /// Extended-source Data Requests are left to the MAC's retained
    /// Association Response transaction. A failed data transmission leaves
    /// the NWK entry queued and Frame Pending armed for a fresh poll.
    pub async fn service_child_data_request(
        &mut self,
        source: MacAddress,
    ) -> Result<ChildPollOutcome, MacError> {
        if matches!(source, MacAddress::Extended(_, _)) {
            return Ok(ChildPollOutcome::AssociationResponsePoll);
        }
        let MacAddress::Short(pan_id, child) = source else {
            return Ok(ChildPollOutcome::AssociationResponsePoll);
        };
        if pan_id != self.nib.pan_id
            || (self.known_child_short(&source).is_none() && !self.indirect.has_pending(child))
        {
            return Ok(ChildPollOutcome::UnknownChild);
        }
        let Some(frame) = self.indirect.peek(child) else {
            self.mac.set_indirect_data_pending(source, false)?;
            return Ok(ChildPollOutcome::NoData);
        };
        let more_pending = self.indirect.pending_count(child) > 1;

        let result = self
            .mac
            .mcps_indirect_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: source,
                payload: frame.as_slice(),
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: true,
                    frame_pending: more_pending,
                    indirect: true,
                    ..Default::default()
                },
            })
            .await;
        if let Err(error) = result {
            let _ = self.mac.set_indirect_data_pending(source, true);
            return Err(error);
        }

        let remaining_pending = self.indirect.complete_one(child).unwrap_or(false);
        debug_assert_eq!(remaining_pending, more_pending);
        self.mac
            .set_indirect_data_pending(source, remaining_pending)?;
        Ok(ChildPollOutcome::Delivered {
            child,
            more_pending: remaining_pending,
        })
    }

    /// Assign a short address to a new child device using stochastic addressing.
    ///
    /// Generates a pseudo-random address based on the child's IEEE address
    /// and a monotonic counter to avoid collisions.
    pub fn assign_child_address(&mut self, child_ieee: &IeeeAddress) -> ShortAddress {
        // Stochastic addressing: hash IEEE address + offset counter
        let mut hash: u16 = 0;
        for &b in child_ieee.iter() {
            hash = hash.wrapping_mul(31).wrapping_add(b as u16);
        }
        hash = hash.wrapping_add(self.next_child_addr_offset);
        self.next_child_addr_offset = self.next_child_addr_offset.wrapping_add(1);

        // Preserve valid hashes and fold only reserved values into the
        // allocated unicast range.
        let address = if (0x0001..=0xFFF7).contains(&hash) {
            hash
        } else {
            (hash % 0xFFF7).saturating_add(1)
        };
        ShortAddress(address)
    }

    /// Handle a child association request (called by the runtime when MAC
    /// delivers an association indication from a joining device).
    ///
    /// Returns the assigned short address on success.
    pub fn handle_child_association(
        &mut self,
        child_ieee: IeeeAddress,
        capability_info: u8,
    ) -> Result<ShortAddress, NwkStatus> {
        if !self.joined {
            return Err(NwkStatus::InvalidRequest);
        }
        if self.device_type == DeviceType::EndDevice {
            return Err(NwkStatus::InvalidRequest);
        }
        if !self.nib.permit_joining {
            return Err(NwkStatus::NotPermitted);
        }

        let capability_info = CapabilityInfo::from_byte(capability_info);
        if !capability_info.allocate_address
            || (capability_info.device_type_ffd && !capability_info.rx_on_when_idle)
        {
            return Err(NwkStatus::NotPermitted);
        }

        let existing_neighbor = self
            .neighbors
            .find_by_ieee(&child_ieee)
            .map(|entry| (entry.network_address, entry.relationship));
        if existing_neighbor
            .is_some_and(|(_, relationship)| relationship == neighbor::Relationship::Parent)
        {
            return Err(NwkStatus::NotPermitted);
        }

        let capacity = if existing_neighbor.is_some() {
            let mut routers = 0usize;
            let mut end_devices = 0usize;
            for child in self.neighbors.children() {
                if Some(child.network_address) == existing_neighbor.map(|entry| entry.0) {
                    continue;
                }
                match child.device_type {
                    neighbor::NeighborDeviceType::Router => routers += 1,
                    neighbor::NeighborDeviceType::EndDevice => end_devices += 1,
                    _ => {}
                }
            }
            let total_capacity = routers + end_devices < usize::from(self.nib.max_children);
            ChildCapacity {
                router: total_capacity
                    && self.nib.depth < self.nib.max_depth
                    && routers < usize::from(self.nib.max_routers),
                end_device: total_capacity,
            }
        } else {
            self.child_capacity()
        };
        if (capability_info.device_type_ffd && !capacity.router)
            || (!capability_info.device_type_ffd && !capacity.end_device)
        {
            return Err(NwkStatus::NeighborTableFull);
        }

        // Determine child type from capability info.
        let is_ffd = capability_info.device_type_ffd;
        let rx_on = capability_info.rx_on_when_idle;
        let dev_type = if is_ffd {
            neighbor::NeighborDeviceType::Router
        } else {
            neighbor::NeighborDeviceType::EndDevice
        };

        let assigned_addr = if let Some((address, _)) = existing_neighbor
            && (0x0001..=0xFFF7).contains(&address.0)
            && address != self.nib.network_address
        {
            address
        } else {
            let mut assigned_addr = None;
            for _ in 0..=neighbor::MAX_NEIGHBORS {
                let candidate = self.assign_child_address(&child_ieee);
                if candidate != self.nib.network_address
                    && self.neighbors.find_by_short(candidate).is_none()
                {
                    assigned_addr = Some(candidate);
                    break;
                }
            }
            assigned_addr.ok_or(NwkStatus::NeighborTableFull)?
        };

        // Add to neighbor table as child
        let entry = neighbor::NeighborEntry {
            ieee_address: child_ieee,
            network_address: assigned_addr,
            device_type: dev_type,
            rx_on_when_idle: rx_on,
            security_capable: capability_info.security_capable,
            relationship: neighbor::Relationship::UnauthenticatedChild,
            lqi: 0xFF,
            outgoing_cost: 1,
            depth: self.nib.depth + 1,
            permit_joining: false,
            age: 0,
            extended_pan_id: self.nib.extended_pan_id,
            active: true,
        };

        self.neighbors
            .add_or_update(entry)
            .map_err(|_| NwkStatus::NeighborTableFull)?;
        self.security.clear_frame_counters_for_source(&child_ieee);
        if let Some((previous_address, _)) = existing_neighbor {
            self.indirect.remove_all(previous_address);
            self.routing.remove(previous_address);
            let _ = self.mac.set_indirect_data_pending(
                MacAddress::Short(self.nib.pan_id, previous_address),
                false,
            );
        }

        log::info!(
            "[NWK] Child associated: IEEE={:02X?} → addr=0x{:04X} type={:?}",
            &child_ieee[..4],
            assigned_addr.0,
            dev_type,
        );

        Ok(assigned_addr)
    }

    /// Admit a device that sent a NWK Rejoin Request.
    ///
    /// A secured rejoin is authorized by successful NWK authentication.
    /// An unsecured Trust Center rejoin remains provisional until the child
    /// receives the current network key and sends a secured frame.
    pub fn handle_child_rejoin(
        &mut self,
        requested_address: ShortAddress,
        child_ieee: IeeeAddress,
        capability_info: u8,
        secured: bool,
    ) -> Result<ShortAddress, NwkStatus> {
        if !self.can_route() {
            return Err(NwkStatus::InvalidRequest);
        }

        let capability_info = CapabilityInfo::from_byte(capability_info);
        if capability_info.device_type_ffd && !capability_info.rx_on_when_idle {
            return Err(NwkStatus::NotPermitted);
        }
        let requested_type = if capability_info.device_type_ffd {
            neighbor::NeighborDeviceType::Router
        } else {
            neighbor::NeighborDeviceType::EndDevice
        };

        if let Some(existing) = self.neighbors.find_by_ieee(&child_ieee) {
            let existing_address = existing.network_address;
            let existing_type = existing.device_type;
            let existing_relationship = existing.relationship;
            if !matches!(
                existing_relationship,
                neighbor::Relationship::Child
                    | neighbor::Relationship::UnauthenticatedChild
                    | neighbor::Relationship::PreviousChild
                    | neighbor::Relationship::Sibling
            ) {
                return Err(NwkStatus::NotPermitted);
            }
            let was_child = matches!(
                existing_relationship,
                neighbor::Relationship::Child
                    | neighbor::Relationship::UnauthenticatedChild
                    | neighbor::Relationship::PreviousChild
            );
            if (!was_child || existing_type != requested_type)
                && !secured
                && !self.nib.permit_joining
            {
                return Err(NwkStatus::NotPermitted);
            }

            let mut routers = 0usize;
            let mut end_devices = 0usize;
            for child in self.neighbors.children() {
                if child.network_address == existing_address {
                    continue;
                }
                match child.device_type {
                    neighbor::NeighborDeviceType::Router => routers += 1,
                    neighbor::NeighborDeviceType::EndDevice => end_devices += 1,
                    _ => {}
                }
            }
            if routers + end_devices >= usize::from(self.nib.max_children)
                || (requested_type == neighbor::NeighborDeviceType::Router
                    && (self.nib.depth >= self.nib.max_depth
                        || routers >= usize::from(self.nib.max_routers)))
            {
                return Err(NwkStatus::NeighborTableFull);
            }

            {
                let entry = self
                    .neighbors
                    .find_by_ieee_mut(&child_ieee)
                    .expect("the child entry still exists");
                entry.device_type = requested_type;
                entry.rx_on_when_idle = capability_info.rx_on_when_idle;
                entry.security_capable = capability_info.security_capable;
                entry.relationship = if secured {
                    neighbor::Relationship::Child
                } else {
                    neighbor::Relationship::UnauthenticatedChild
                };
                entry.age = 0;
            }
            if !secured {
                self.security.clear_frame_counters_for_source(&child_ieee);
                self.indirect.remove_all(existing_address);
                self.routing.remove(existing_address);
                let _ = self.mac.set_indirect_data_pending(
                    MacAddress::Short(self.nib.pan_id, existing_address),
                    false,
                );
            }
            return Ok(existing_address);
        }

        // A known child may perform a Trust Center rejoin while joining is
        // closed. A new unsecured device is admitted only while permit joining
        // is active; the Trust Center still makes the authorization decision.
        if !secured && !self.nib.permit_joining {
            return Err(NwkStatus::NotPermitted);
        }

        let capacity = self.child_capacity();
        if (capability_info.device_type_ffd && !capacity.router)
            || (!capability_info.device_type_ffd && !capacity.end_device)
        {
            return Err(NwkStatus::NeighborTableFull);
        }

        let requested_is_usable = (0x0001..=0xFFF7).contains(&requested_address.0)
            && requested_address != self.nib.network_address
            && self.neighbors.find_by_short(requested_address).is_none();
        let assigned_address = if requested_is_usable {
            requested_address
        } else {
            let mut assigned = None;
            for _ in 0..=neighbor::MAX_NEIGHBORS {
                let candidate = self.assign_child_address(&child_ieee);
                if candidate != self.nib.network_address
                    && self.neighbors.find_by_short(candidate).is_none()
                {
                    assigned = Some(candidate);
                    break;
                }
            }
            assigned.ok_or(NwkStatus::NeighborTableFull)?
        };

        let entry = neighbor::NeighborEntry {
            ieee_address: child_ieee,
            network_address: assigned_address,
            device_type: requested_type,
            rx_on_when_idle: capability_info.rx_on_when_idle,
            security_capable: capability_info.security_capable,
            relationship: if secured {
                neighbor::Relationship::Child
            } else {
                neighbor::Relationship::UnauthenticatedChild
            },
            lqi: 0xFF,
            outgoing_cost: 1,
            depth: self.nib.depth.saturating_add(1),
            permit_joining: false,
            age: 0,
            extended_pan_id: self.nib.extended_pan_id,
            active: true,
        };
        self.neighbors
            .add_or_update(entry)
            .map_err(|_| NwkStatus::NeighborTableFull)?;
        if !secured {
            self.security.clear_frame_counters_for_source(&child_ieee);
        }
        Ok(assigned_address)
    }
}
