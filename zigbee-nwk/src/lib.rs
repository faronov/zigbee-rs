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
pub mod routing;
pub mod security;

use zigbee_mac::MacDriver;

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
}

impl<M: MacDriver> NwkLayer<M> {
    /// Create a new NWK layer with the given MAC driver.
    pub fn new(mac: M, device_type: DeviceType) -> Self {
        Self {
            mac,
            nib: nib::Nib::new(),
            neighbors: neighbor::NeighborTable::new(),
            routing: routing::RoutingTable::new(),
            security: security::NwkSecurity::new(),
            device_type,
            joined: false,
        }
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
}
