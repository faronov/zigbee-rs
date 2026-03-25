//! Touchlink commissioning (BDB v3.0.1 spec §8.7).
//!
//! Touchlink (formerly ZLL commissioning) is a proximity-based method
//! that allows devices to join a network by being brought physically
//! close to each other. It uses Inter-PAN frames on the MAC layer,
//! bypassing the normal NWK/APS routing.
//!
//! ## Overview
//! 1. **Initiator** sends a Scan Request (Inter-PAN) on each channel
//! 2. **Target** responds with Scan Response if RSSI exceeds threshold
//! 3. Initiator sends Network Start/Join Request with network parameters
//! 4. Target applies parameters and joins the network
//!
//! ## Security
//! Touchlink uses a pre-configured link key for encrypting the network
//! key transport during commissioning.
//!
//! ## Status
//! This module provides the type definitions and stub implementations.
//! Full Touchlink support requires Inter-PAN frame handling in the MAC
//! layer, which is not yet implemented.
//!
//! TODO:
//! - Inter-PAN frame TX/RX support in MAC layer
//! - Touchlink Scan Request/Response handling
//! - Network Start Request/Response
//! - Network Join Router/End-Device Request/Response
//! - Factory New reset via Touchlink
//! - RSSI-based proximity filtering
//! - Touchlink key encryption/decryption

use zigbee_mac::MacDriver;
use zigbee_types::IeeeAddress;

use crate::{BdbLayer, BdbStatus};

// ── Touchlink constants ─────────────────────────────────────

/// Touchlink Inter-PAN transaction ID (randomly generated per session).
pub type TouchlinkTransactionId = u32;

/// Minimum RSSI threshold for Touchlink proximity detection (dBm).
/// Devices must be physically close for Touchlink to work.
pub const TOUCHLINK_RSSI_THRESHOLD: i8 = -40;

/// Touchlink scan channels (primary: 11, 15, 20, 25).
pub const TOUCHLINK_PRIMARY_CHANNELS: [u8; 4] = [11, 15, 20, 25];

/// ZLL / Touchlink pre-configured link key (used for key transport).
pub const TOUCHLINK_PRECONFIGURED_LINK_KEY: [u8; 16] = [
    0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
];

// ── Inter-PAN command IDs ───────────────────────────────────

/// Touchlink ZCL command IDs (cluster 0x1000 — Touchlink Commissioning).
pub mod command_id {
    pub const SCAN_REQUEST: u8 = 0x00;
    pub const SCAN_RESPONSE: u8 = 0x01;
    pub const DEVICE_INFO_REQUEST: u8 = 0x02;
    pub const DEVICE_INFO_RESPONSE: u8 = 0x03;
    pub const IDENTIFY_REQUEST: u8 = 0x06;
    pub const FACTORY_NEW_RESET: u8 = 0x07;
    pub const NETWORK_START_REQUEST: u8 = 0x10;
    pub const NETWORK_START_RESPONSE: u8 = 0x11;
    pub const NETWORK_JOIN_ROUTER_REQUEST: u8 = 0x12;
    pub const NETWORK_JOIN_ROUTER_RESPONSE: u8 = 0x13;
    pub const NETWORK_JOIN_ED_REQUEST: u8 = 0x14;
    pub const NETWORK_JOIN_ED_RESPONSE: u8 = 0x15;
    pub const NETWORK_UPDATE_REQUEST: u8 = 0x16;
}

// ── Scan response ───────────────────────────────────────────

/// Information from a Touchlink Scan Response.
#[derive(Debug, Clone)]
pub struct TouchlinkScanResponse {
    /// Transaction ID (must match our request)
    pub transaction_id: TouchlinkTransactionId,
    /// Responder's IEEE address
    pub ieee_address: IeeeAddress,
    /// RSSI of the response
    pub rssi: i8,
    /// Whether the target is factory new
    pub factory_new: bool,
    /// Whether the target is address assignment capable
    pub address_assignment: bool,
    /// Channel on which the target's current network operates
    pub logical_channel: u8,
    /// Extended PAN ID of target's current network (all-zero if factory new)
    pub extended_pan_id: IeeeAddress,
}

// ── Implementation stubs ────────────────────────────────────

impl<M: MacDriver> BdbLayer<M> {
    /// Execute Touchlink commissioning as initiator (BDB spec §8.7).
    ///
    /// # Current status
    /// **Stub implementation** — returns `TouchlinkFailure`.
    /// Full implementation requires Inter-PAN frame support in the MAC layer.
    ///
    /// # Procedure (when implemented)
    /// 1. Generate random transaction ID
    /// 2. For each Touchlink primary channel:
    ///    a. Send Scan Request (Inter-PAN broadcast)
    ///    b. Collect Scan Responses with RSSI > threshold
    /// 3. Select best target (highest RSSI)
    /// 4. Send Identify Request (optional — blink target LED)
    /// 5. Send Network Start/Join Request with network parameters
    /// 6. Wait for Network Start/Join Response
    /// 7. Target joins our network
    pub async fn touchlink_commissioning(&mut self) -> Result<(), BdbStatus> {
        log::warn!("[BDB:Touchlink] Not implemented — Inter-PAN support required");

        // TODO: Implement the full Touchlink procedure:
        //
        // let transaction_id = generate_random_u32();
        //
        // for channel in TOUCHLINK_PRIMARY_CHANNELS {
        //     // Switch to channel
        //     // Send Inter-PAN Scan Request
        //     // Wait for responses (timeout ~250ms per channel)
        //     // Collect responses with RSSI > TOUCHLINK_RSSI_THRESHOLD
        // }
        //
        // if no targets found:
        //     return Err(BdbStatus::TouchlinkFailure)
        //
        // // Select target with highest RSSI
        // let target = best_target;
        //
        // // Optionally send Identify Request
        // // Send Network Start Request (if we're forming) or Network Join Request
        // // Wait for response
        // // Apply network parameters
        //
        // self.attributes.node_is_on_a_network = true;
        // Ok(())

        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::TlNoScanResponse;
        Err(BdbStatus::TouchlinkFailure)
    }

    /// Handle a received Touchlink Scan Request (target role).
    ///
    /// # Current status
    /// **Stub** — always returns `Err`.
    pub async fn touchlink_handle_scan_request(
        &mut self,
        _transaction_id: TouchlinkTransactionId,
        _rssi: i8,
    ) -> Result<(), BdbStatus> {
        // TODO: Check RSSI threshold, build and send Scan Response
        log::warn!("[BDB:Touchlink] Scan Request handler not implemented");
        Err(BdbStatus::TouchlinkFailure)
    }

    /// Perform a Touchlink factory reset on a nearby target device.
    ///
    /// # Current status
    /// **Stub** — always returns `Err`.
    pub async fn touchlink_factory_reset(&mut self) -> Result<(), BdbStatus> {
        // TODO: Send Touchlink Identify Request + Factory New Reset via Inter-PAN
        log::warn!("[BDB:Touchlink] Factory reset not implemented");
        Err(BdbStatus::TouchlinkFailure)
    }
}
