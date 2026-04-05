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

pub mod frames;
pub mod indirect;
pub mod neighbor;
pub mod nib;
pub mod nlde;
pub mod nlme;
pub mod nwk_commands;
pub mod routing;
pub mod security;

use zigbee_mac::MacDriver;
use zigbee_types::{IeeeAddress, ShortAddress};

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
#[derive(Debug, Clone)]
pub struct PendingRreqRebroadcast {
    pub command_options: u8,
    pub route_request_id: u8,
    pub dst_addr: ShortAddress,
    pub path_cost: u8,
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
    device_type: DeviceType,
    joined: bool,
    /// Whether this device listens when idle.
    /// true = non-sleepy (RFD/FFD that stays awake)
    /// false = sleepy end device (polls parent for data)
    rx_on_when_idle: bool,
    /// Pending route replies to be sent asynchronously.
    pending_route_replies: heapless::Vec<PendingRouteReply, 4>,
    /// Pending RREQ rebroadcasts (queued from sync handler, sent async).
    pending_rreq_rebroadcasts: heapless::Vec<PendingRreqRebroadcast, 4>,
    /// Pending Network Status (route error) notifications.
    pending_route_errors: heapless::Vec<PendingNetworkStatus, 4>,
    /// Indirect frame queue for sleeping end device children.
    indirect: indirect::IndirectQueue,
    /// Link status periodic timer counter (seconds).
    link_status_counter: u16,
    /// Flag: link status should be sent in next async context.
    link_status_due: bool,
    /// Whether this device is operating as a concentrator (many-to-one).
    concentrator_active: bool,
    /// Concentrator RREQ interval counter (seconds).
    concentrator_counter: u16,
    /// Concentrator RREQ interval (seconds, default 60).
    concentrator_interval: u16,
    /// Flag: concentrator RREQ should be sent in next async context.
    concentrator_rreq_due: bool,
    /// Counter for stochastic child address assignment.
    next_child_addr_offset: u16,
}

impl<M: MacDriver> NwkLayer<M> {
    /// Create a new NWK layer with the given MAC driver.
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
            device_type,
            joined: false,
            rx_on_when_idle,
            pending_route_replies: heapless::Vec::new(),
            pending_rreq_rebroadcasts: heapless::Vec::new(),
            pending_route_errors: heapless::Vec::new(),
            indirect: indirect::IndirectQueue::new(),
            link_status_counter: 0,
            link_status_due: false,
            concentrator_active: false,
            concentrator_counter: 0,
            concentrator_interval: 60,
            concentrator_rreq_due: false,
            next_child_addr_offset: 1,
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

    /// Get the device type.
    pub fn device_type(&self) -> DeviceType {
        self.device_type
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

    /// Read-only access to the neighbor table.
    pub fn neighbor_table(&self) -> &neighbor::NeighborTable {
        &self.neighbors
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
        // Age BTR entries
        for _ in 0..elapsed_secs {
            self.btr.age();
            self.indirect.age();
        }

        // Age routing table entries
        self.routing.age_tick();

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
    pub fn start_concentrator(&mut self, interval_secs: u16) {
        if self.device_type == DeviceType::EndDevice {
            log::warn!("[NWK] Cannot start concentrator on end device");
            return;
        }
        self.concentrator_active = true;
        self.concentrator_interval = interval_secs;
        self.concentrator_counter = interval_secs; // Trigger immediately on first tick
        log::info!(
            "[NWK] Concentrator mode enabled (interval={}s)",
            interval_secs
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

        // Ensure address is valid (not broadcast, not unassigned, not coordinator)
        let addr = match hash {
            0x0000 | 0xFFFC..=0xFFFF => hash.wrapping_add(0x0100),
            other => other,
        };
        ShortAddress(addr)
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

        // Determine child type from capability info
        let is_ffd = capability_info & 0x02 != 0;
        let rx_on = capability_info & 0x08 != 0;
        let dev_type = if is_ffd {
            neighbor::NeighborDeviceType::Router
        } else {
            neighbor::NeighborDeviceType::EndDevice
        };

        let assigned_addr = self.assign_child_address(&child_ieee);

        // Add to neighbor table as child
        let entry = neighbor::NeighborEntry {
            ieee_address: child_ieee,
            network_address: assigned_addr,
            device_type: dev_type,
            rx_on_when_idle: rx_on,
            relationship: neighbor::Relationship::Child,
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

        log::info!(
            "[NWK] Child associated: IEEE={:02X?} → addr=0x{:04X} type={:?}",
            &child_ieee[..4],
            assigned_addr.0,
            dev_type,
        );

        Ok(assigned_addr)
    }
}
