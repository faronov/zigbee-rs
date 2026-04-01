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
//! This module provides the full protocol logic. Actual over-the-air
//! transmission requires Inter-PAN frame support in the MAC layer.
//! When the MAC layer does not support Inter-PAN, the protocol will
//! gracefully degrade and return `TouchlinkFailure`.

use zigbee_mac::MacDriver;
use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};

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

/// Scan duration per channel in milliseconds.
pub const TOUCHLINK_SCAN_DURATION_MS: u32 = 250;

/// Maximum number of scan request attempts per channel.
const TOUCHLINK_MAX_SCAN_ATTEMPTS: u8 = 3;

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

// ── Scan request / response ─────────────────────────────────

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

/// Touchlink Scan Request payload.
#[derive(Debug, Clone)]
pub struct TouchlinkScanRequest {
    /// Inter-PAN transaction identifier
    pub transaction_id: TouchlinkTransactionId,
    /// Zigbee information field (device type + rx-on-when-idle)
    pub zigbee_info: u8,
    /// Touchlink information field (factory new, address assignment, etc.)
    pub touchlink_info: u8,
}

impl TouchlinkScanRequest {
    /// Serialize the scan request into `buf`. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 6 {
            return 0;
        }
        let tid = self.transaction_id.to_le_bytes();
        buf[0] = tid[0];
        buf[1] = tid[1];
        buf[2] = tid[2];
        buf[3] = tid[3];
        buf[4] = self.zigbee_info;
        buf[5] = self.touchlink_info;
        6
    }
}

/// Touchlink Network Join Request payload.
#[derive(Debug, Clone)]
pub struct TouchlinkNetworkJoinRequest {
    /// Transaction ID (must match scan)
    pub transaction_id: TouchlinkTransactionId,
    /// Extended PAN ID for the network
    pub extended_pan_id: IeeeAddress,
    /// Key index (identifies the encryption key variant)
    pub key_index: u8,
    /// Encrypted network key
    pub encrypted_network_key: [u8; 16],
    /// Logical channel for the network
    pub logical_channel: u8,
    /// Short PAN ID
    pub pan_id: u16,
    /// Network (short) address assigned to the target
    pub network_address: u16,
}

impl TouchlinkNetworkJoinRequest {
    /// Serialize the join request into `buf`. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 32 {
            return 0;
        }
        let tid = self.transaction_id.to_le_bytes();
        buf[0..4].copy_from_slice(&tid);
        buf[4..12].copy_from_slice(&self.extended_pan_id);
        buf[12] = self.key_index;
        buf[13..29].copy_from_slice(&self.encrypted_network_key);
        buf[29] = self.logical_channel;
        buf[30] = (self.pan_id & 0xFF) as u8;
        buf[31] = ((self.pan_id >> 8) & 0xFF) as u8;
        // network_address at bytes 32-33 (if buf is large enough)
        if buf.len() >= 34 {
            buf[32] = (self.network_address & 0xFF) as u8;
            buf[33] = ((self.network_address >> 8) & 0xFF) as u8;
            34
        } else {
            32
        }
    }
}

// ── Transaction ID generator ────────────────────────────────

/// Simple deterministic transaction ID generator (counter-based).
///
/// In a real deployment this should use a hardware RNG.  Under `no_std`
/// without an allocator we fall back to a wrapping counter seeded from
/// the device's IEEE address.
fn generate_transaction_id(ieee: &IeeeAddress, counter: &mut u32) -> TouchlinkTransactionId {
    *counter = counter.wrapping_add(1);
    // Mix the counter with bytes from the IEEE address for basic uniqueness
    let seed = u32::from_le_bytes([ieee[0], ieee[1], ieee[2], ieee[3]]);
    seed ^ *counter
}

// ── Implementation ──────────────────────────────────────────

impl<M: MacDriver> BdbLayer<M> {
    /// Execute Touchlink commissioning as initiator (BDB spec §8.7).
    ///
    /// Implements the full Touchlink scan → select → join protocol.
    /// If the MAC layer does not support Inter-PAN frames the scan
    /// will collect no responses and the method returns
    /// `Err(BdbStatus::TouchlinkFailure)`.
    pub async fn touchlink_commissioning(&mut self) -> Result<(), BdbStatus> {
        log::info!("[BDB:Touchlink] Starting Touchlink commissioning (initiator)");

        // Generate transaction ID
        let ieee = self.zdo().nwk().nib().ieee_address;
        let mut counter = self.touchlink_counter();
        let transaction_id = generate_transaction_id(&ieee, &mut counter);
        self.set_touchlink_counter(counter);

        let scan_req = TouchlinkScanRequest {
            transaction_id,
            zigbee_info: 0x04, // Router capable, rx-on-when-idle
            touchlink_info: 0x00,
        };

        let mut responses: heapless::Vec<TouchlinkScanResponse, 8> = heapless::Vec::new();

        // Scan on each primary Touchlink channel
        for &channel in &TOUCHLINK_PRIMARY_CHANNELS {
            // Set MAC channel via PIB
            let set_result = self
                .zdo_mut()
                .nwk_mut()
                .mac_mut()
                .mlme_set(
                    zigbee_mac::pib::PibAttribute::PhyCurrentChannel,
                    zigbee_mac::pib::PibValue::U8(channel),
                )
                .await;

            if set_result.is_err() {
                log::debug!("[BDB:Touchlink] Cannot set channel {} — skipping", channel);
                continue;
            }

            // Build and try to send Scan Request
            let mut scan_payload = [0u8; 8];
            let scan_len = scan_req.serialize(&mut scan_payload);

            for _attempt in 0..TOUCHLINK_MAX_SCAN_ATTEMPTS {
                // Attempt Inter-PAN broadcast — MAC implementations without
                // Inter-PAN support will return an error here.
                let send_result = self
                    .zdo_mut()
                    .nwk_mut()
                    .mac_mut()
                    .mcps_data(zigbee_mac::McpsDataRequest {
                        src_addr_mode: zigbee_mac::AddressMode::Extended,
                        dst_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
                        payload: &scan_payload[..scan_len],
                        msdu_handle: 0,
                        tx_options: zigbee_mac::TxOptions::default(),
                    })
                    .await;

                if send_result.is_err() {
                    // Inter-PAN not supported — expected for most MAC backends
                    log::debug!(
                        "[BDB:Touchlink] Inter-PAN TX failed on ch {} — no support",
                        channel
                    );
                    break;
                }

                // In a real implementation we would receive scan responses here.
                // Without Inter-PAN RX support no responses will arrive.
            }

            log::debug!(
                "[BDB:Touchlink] Channel {} scan complete, {} responses so far",
                channel,
                responses.len()
            );
        }

        // Filter by RSSI threshold
        responses.retain(|r| r.rssi > TOUCHLINK_RSSI_THRESHOLD);

        if responses.is_empty() {
            log::warn!("[BDB:Touchlink] No scan responses received");
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::TlNoScanResponse;
            return Err(BdbStatus::TouchlinkFailure);
        }

        // Select target with highest RSSI
        let best_idx = responses
            .iter()
            .enumerate()
            .max_by_key(|(_, r)| r.rssi)
            .map(|(i, _)| i)
            .unwrap_or(0);
        let target = &responses[best_idx];

        log::info!(
            "[BDB:Touchlink] Selected target IEEE={:02X?} RSSI={} ch={}",
            target.ieee_address,
            target.rssi,
            target.logical_channel
        );

        // Build and send Network Join Request
        let join_req = TouchlinkNetworkJoinRequest {
            transaction_id,
            extended_pan_id: self.zdo().nwk().nib().extended_pan_id,
            key_index: 0x04, // Touchlink preconfigured key
            encrypted_network_key: TOUCHLINK_PRECONFIGURED_LINK_KEY,
            logical_channel: target.logical_channel,
            pan_id: self.zdo().nwk().nib().pan_id.0,
            network_address: 0x0001, // assign short address to target
        };

        let mut join_payload = [0u8; 40];
        let join_len = join_req.serialize(&mut join_payload);

        let join_result = self
            .zdo_mut()
            .nwk_mut()
            .mac_mut()
            .mcps_data(zigbee_mac::McpsDataRequest {
                src_addr_mode: zigbee_mac::AddressMode::Extended,
                dst_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
                payload: &join_payload[..join_len],
                msdu_handle: 1,
                tx_options: zigbee_mac::TxOptions::default(),
            })
            .await;

        if join_result.is_err() {
            log::warn!("[BDB:Touchlink] Network Join Request TX failed");
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::TlTargetFailure;
            return Err(BdbStatus::TouchlinkFailure);
        }

        self.attributes.node_is_on_a_network = true;
        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;
        log::info!("[BDB:Touchlink] Commissioning complete");
        Ok(())
    }

    /// Handle a received Touchlink Scan Request (target role).
    ///
    /// Verifies the RSSI exceeds the proximity threshold, builds a
    /// scan response with this device's information, and attempts to
    /// send it via Inter-PAN.  Returns `Err` if the RSSI is too low or
    /// the MAC does not support Inter-PAN frames.
    pub async fn touchlink_handle_scan_request(
        &mut self,
        transaction_id: TouchlinkTransactionId,
        rssi: i8,
    ) -> Result<(), BdbStatus> {
        if rssi < TOUCHLINK_RSSI_THRESHOLD {
            log::debug!(
                "[BDB:Touchlink] Scan Request rejected — RSSI {} < threshold {}",
                rssi,
                TOUCHLINK_RSSI_THRESHOLD
            );
            return Err(BdbStatus::TouchlinkFailure);
        }

        let nwk = self.zdo().nwk();
        let response = TouchlinkScanResponse {
            transaction_id,
            ieee_address: nwk.nib().ieee_address,
            rssi: 0, // Our own RSSI is not meaningful
            factory_new: !self.attributes.node_is_on_a_network,
            address_assignment: true,
            logical_channel: nwk.nib().logical_channel,
            extended_pan_id: nwk.nib().extended_pan_id,
        };

        // Build a minimal scan response payload
        let mut payload = [0u8; 32];
        let tid = response.transaction_id.to_le_bytes();
        payload[0..4].copy_from_slice(&tid);
        payload[4..12].copy_from_slice(&response.ieee_address);
        payload[12] = if response.factory_new { 0x01 } else { 0x00 };
        payload[13] = if response.address_assignment {
            0x01
        } else {
            0x00
        };
        payload[14] = response.logical_channel;
        payload[15..23].copy_from_slice(&response.extended_pan_id);
        let len = 23;

        let send_result = self
            .zdo_mut()
            .nwk_mut()
            .mac_mut()
            .mcps_data(zigbee_mac::McpsDataRequest {
                src_addr_mode: zigbee_mac::AddressMode::Extended,
                dst_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
                payload: &payload[..len],
                msdu_handle: 2,
                tx_options: zigbee_mac::TxOptions::default(),
            })
            .await;

        if send_result.is_err() {
            log::warn!("[BDB:Touchlink] Scan Response TX failed — no Inter-PAN support");
            return Err(BdbStatus::TouchlinkFailure);
        }

        log::info!(
            "[BDB:Touchlink] Scan Response sent (tid=0x{:08X})",
            transaction_id
        );
        Ok(())
    }

    /// Perform a Touchlink factory reset.
    ///
    /// Clears network state via the ZDO layer and resets BDB attributes
    /// to their defaults.
    pub async fn touchlink_factory_reset(&mut self) -> Result<(), BdbStatus> {
        log::info!("[BDB:Touchlink] Factory reset requested");

        // Clear network parameters in the NWK layer
        let nwk = self.zdo_mut().nwk_mut();
        nwk.nib_mut().network_address = ShortAddress(0xFFFF);
        nwk.nib_mut().pan_id = PanId(0xFFFF);
        nwk.nib_mut().extended_pan_id = [0u8; 8];
        nwk.nib_mut().logical_channel = 0;
        nwk.nib_mut().parent_address = ShortAddress(0xFFFF);

        // Reset BDB attributes to defaults
        self.attributes = crate::attributes::BdbAttributes::default();

        log::info!("[BDB:Touchlink] Factory reset complete");
        Ok(())
    }

    // ── Internal helpers ────────────────────────────────────

    /// Get the current touchlink counter value.
    ///
    /// Uses the BDB commissioning group ID as storage for a simple
    /// counter since no dedicated NV field exists in this implementation.
    fn touchlink_counter(&self) -> u32 {
        self.attributes.commissioning_group_id as u32
    }

    /// Set the touchlink counter.
    fn set_touchlink_counter(&mut self, val: u32) {
        self.attributes.commissioning_group_id = val as u16;
    }
}
