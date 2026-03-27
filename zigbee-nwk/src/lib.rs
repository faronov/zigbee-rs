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
    security: security::NwkSecurity,
    device_type: DeviceType,
    joined: bool,
    /// Whether this device listens when idle.
    /// true = non-sleepy (RFD/FFD that stays awake)
    /// false = sleepy end device (polls parent for data)
    rx_on_when_idle: bool,
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
            security: security::NwkSecurity::new(),
            device_type,
            joined: false,
            rx_on_when_idle,
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
}
