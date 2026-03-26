//! APSDE — APS Data Entity.
//!
//! Provides the APSDE-DATA service for sending and receiving application
//! data through the APS layer (Zigbee spec 2.2.4).
//!
//! - `APSDE-DATA.request`:    send data from upper layer to a peer
//! - `APSDE-DATA.confirm`:    transmission result
//! - `APSDE-DATA.indication`: received data delivered to upper layer

use crate::frames::{ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};
use crate::{ApsAddress, ApsAddressMode, ApsLayer, ApsStatus, ApsTxOptions, PendingApsAck};
use zigbee_mac::MacDriver;
use zigbee_nwk::NwkStatus;
use zigbee_types::ShortAddress;

// ── APSDE-DATA.request ──────────────────────────────────────────

/// Parameters for APSDE-DATA.request (Zigbee spec Table 2-2).
#[derive(Debug)]
pub struct ApsdeDataRequest<'a> {
    /// Destination address mode
    pub dst_addr_mode: ApsAddressMode,
    /// Destination address (short, extended, or group)
    pub dst_address: ApsAddress,
    /// Destination endpoint (0x00-0xFF)
    pub dst_endpoint: u8,
    /// Profile identifier
    pub profile_id: u16,
    /// Cluster identifier
    pub cluster_id: u16,
    /// Source endpoint
    pub src_endpoint: u8,
    /// Application payload
    pub payload: &'a [u8],
    /// Transmission options
    pub tx_options: ApsTxOptions,
    /// NWK radius (0 = use NIB default)
    pub radius: u8,
    /// Alias source address (for ZDP commissioning, usually not used)
    pub alias_src_addr: Option<ShortAddress>,
    /// Alias sequence number
    pub alias_seq: Option<u8>,
}

// ── APSDE-DATA.confirm ──────────────────────────────────────────

/// Result of APSDE-DATA.request (Zigbee spec Table 2-4).
#[derive(Debug)]
pub struct ApsdeDataConfirm {
    /// Status of the transmission
    pub status: ApsStatus,
    /// Destination address mode
    pub dst_addr_mode: ApsAddressMode,
    /// Destination address
    pub dst_address: ApsAddress,
    /// Destination endpoint
    pub dst_endpoint: u8,
    /// Source endpoint
    pub src_endpoint: u8,
    /// APS counter used for this transmission
    pub aps_counter: u8,
}

// ── APSDE-DATA.indication ───────────────────────────────────────

/// Received data delivered to the upper layer (Zigbee spec Table 2-5).
#[derive(Debug)]
pub struct ApsdeDataIndication<'a> {
    /// Destination address mode
    pub dst_addr_mode: ApsAddressMode,
    /// Destination address
    pub dst_address: ApsAddress,
    /// Destination endpoint
    pub dst_endpoint: u8,
    /// Source address mode
    pub src_addr_mode: ApsAddressMode,
    /// Source address
    pub src_address: ApsAddress,
    /// Source endpoint
    pub src_endpoint: u8,
    /// Profile identifier
    pub profile_id: u16,
    /// Cluster identifier
    pub cluster_id: u16,
    /// Application payload
    pub payload: &'a [u8],
    /// APS counter
    pub aps_counter: u8,
    /// Whether the frame was secured at the APS level
    pub security_status: bool,
    /// Link quality indication from the MAC/NWK layer
    pub lqi: u8,
}

// ── APS frame buffer for parsed indication ──────────────────────

/// Internal buffer that owns the payload for a parsed APS indication.
///
/// Since `ApsdeDataIndication` borrows its payload, we need this to
/// hold the data while the upper layer processes it.
pub struct ApsFrameBuffer {
    pub data: [u8; 128],
    pub len: usize,
}

impl ApsFrameBuffer {
    pub fn new() -> Self {
        Self {
            data: [0u8; 128],
            len: 0,
        }
    }

    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len]
    }
}

impl Default for ApsFrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

// ── APSDE-DATA service implementation ───────────────────────────

impl<M: MacDriver> ApsLayer<M> {
    /// APSDE-DATA.request — transmit application data through APS.
    ///
    /// Builds an APS header, serializes it + payload into a NWK NSDU,
    /// and calls `nlde_data_request` to send via the NWK layer.
    pub async fn apsde_data_request(
        &mut self,
        req: &ApsdeDataRequest<'_>,
    ) -> Result<ApsdeDataConfirm, ApsStatus> {
        // Determine NWK destination and APS delivery mode
        let (nwk_dst, delivery_mode) = match req.dst_addr_mode {
            ApsAddressMode::Short => {
                let addr = match req.dst_address {
                    ApsAddress::Short(a) => a,
                    _ => return Err(ApsStatus::InvalidParameter),
                };
                (addr, ApsDeliveryMode::Unicast)
            }
            ApsAddressMode::Group => {
                let _group = match req.dst_address {
                    ApsAddress::Group(g) => g,
                    _ => return Err(ApsStatus::InvalidParameter),
                };
                // Group messages are broadcast at the NWK level
                (ShortAddress(0xFFFF), ApsDeliveryMode::Group)
            }
            ApsAddressMode::Extended => {
                // TODO: resolve IEEE → short address via address map
                // For now, return error (upper layer should resolve first)
                return Err(ApsStatus::NoShortAddress);
            }
            ApsAddressMode::Indirect => {
                // Look up binding table to find destinations
                let ieee = self.nwk.nib().ieee_address;
                let has_binding = self
                    .binding_table
                    .find_by_source(&ieee, req.src_endpoint, req.cluster_id)
                    .next()
                    .is_some();
                if !has_binding {
                    return Err(ApsStatus::NoBoundDevice);
                }
                // Send to coordinator for indirect delivery
                (ShortAddress::COORDINATOR, ApsDeliveryMode::Indirect)
            }
        };

        // Allocate APS counter
        let aps_counter = self.next_aps_counter();

        // Build APS header
        let aps_header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Data as u8,
                delivery_mode: delivery_mode as u8,
                ack_format: false,
                security: req.tx_options.security_enabled,
                ack_request: req.tx_options.ack_request,
                extended_header: false,
            },
            dst_endpoint: match delivery_mode {
                ApsDeliveryMode::Unicast | ApsDeliveryMode::Broadcast => Some(req.dst_endpoint),
                _ => None,
            },
            group_address: match delivery_mode {
                ApsDeliveryMode::Group => {
                    if let ApsAddress::Group(g) = req.dst_address {
                        Some(g)
                    } else {
                        None
                    }
                }
                _ => None,
            },
            cluster_id: Some(req.cluster_id),
            profile_id: Some(req.profile_id),
            src_endpoint: Some(req.src_endpoint),
            aps_counter,
            extended_header: None,
        };

        // Serialize APS frame into buffer
        let mut aps_buf = [0u8; 128];
        let hdr_len = aps_header.serialize(&mut aps_buf);

        // Copy payload after header
        let total_len = hdr_len + req.payload.len();
        if total_len > aps_buf.len() {
            return Err(ApsStatus::AsduTooLong);
        }
        aps_buf[hdr_len..total_len].copy_from_slice(req.payload);

        // Determine radius (0 = use NIB default, typically 2×max_depth)
        let radius = if req.radius == 0 {
            self.nwk.nib().max_depth.saturating_mul(2)
        } else {
            req.radius
        };

        // Send via NWK layer
        let nwk_result = self
            .nwk
            .nlde_data_request(
                nwk_dst,
                radius,
                &aps_buf[..total_len],
                req.tx_options.use_nwk_key,
                true, // discover_route
            )
            .await;

        match nwk_result {
            Ok(_confirm) => Ok(ApsdeDataConfirm {
                status: ApsStatus::Success,
                dst_addr_mode: req.dst_addr_mode,
                dst_address: req.dst_address,
                dst_endpoint: req.dst_endpoint,
                src_endpoint: req.src_endpoint,
                aps_counter,
            }),
            Err(nwk_err) => {
                log::warn!("APSDE-DATA.request failed: NWK error {:?}", nwk_err);
                let aps_err = match nwk_err {
                    NwkStatus::FrameTooLong => ApsStatus::AsduTooLong,
                    NwkStatus::InvalidRequest => ApsStatus::IllegalRequest,
                    NwkStatus::RouteError | NwkStatus::RouteDiscoveryFailed => {
                        ApsStatus::NoShortAddress
                    }
                    _ => ApsStatus::NoAck,
                };
                Err(aps_err)
            }
        }
    }

    /// Process an incoming APS frame from a NWK data indication.
    ///
    /// Parses the APS header from the NWK payload and returns an
    /// `ApsdeDataIndication` for the upper layer.
    ///
    /// For APS-secured frames (Transport Key, etc.), this decrypts using
    /// the appropriate link key before processing commands.
    pub fn process_incoming_aps_frame<'a>(
        &mut self,
        nwk_payload: &'a [u8],
        nwk_src: ShortAddress,
        nwk_dst: ShortAddress,
        lqi: u8,
        nwk_security: bool,
        decrypted_buf: &'a mut ApsFrameBuffer,
    ) -> Option<ApsdeDataIndication<'a>> {
        let (header, consumed) = ApsHeader::parse(nwk_payload)?;

        // Determine if APS security is applied
        let aps_secured = header.frame_control.security;

        // Get the payload after the APS header
        let after_header = &nwk_payload[consumed..];

        // If APS-secured, we need to decrypt before processing
        let (effective_payload, _aps_sec_overhead) = if aps_secured {
            // Parse APS security auxiliary header
            let (sec_hdr, sec_consumed) = crate::security::ApsSecurityHeader::parse(after_header)?;

            let ciphertext = &after_header[sec_consumed..];

            // Build AAD = APS header bytes || APS security header bytes
            let aad_end = consumed + sec_consumed;
            let aad = &nwk_payload[..aad_end];

            // Determine the decryption key:
            // For key_id=0 (Data Key / Link Key), use TC link key
            let key_id =
                crate::security::ApsSecurityHeader::key_identifier(sec_hdr.security_control);
            let key = if key_id == crate::security::KEY_ID_DATA_KEY {
                // Try partner-specific key first, then default TC link key
                if let Some(addr) = &sec_hdr.source_address {
                    if let Some(entry) = self.security.find_any_key(addr) {
                        entry.key
                    } else {
                        *self.security.default_tc_link_key()
                    }
                } else {
                    *self.security.default_tc_link_key()
                }
            } else if key_id == crate::security::KEY_ID_KEY_TRANSPORT {
                // Key-Transport Key: derive from TC link key via HMAC-MMO (spec §4.5.3.4)
                crate::security::derive_key_transport_key(self.security.default_tc_link_key())
            } else if key_id == crate::security::KEY_ID_KEY_LOAD {
                // Key-Load Key: derive from TC link key via HMAC-MMO
                crate::security::derive_key_load_key(self.security.default_tc_link_key())
            } else {
                log::warn!("[APS] Unsupported key_id={} in APS security", key_id);
                return None;
            };

            // Two-phase replay protection: check BEFORE decrypt, commit AFTER
            let replay_key_type = if key_id == crate::security::KEY_ID_DATA_KEY {
                crate::security::ApsKeyType::TrustCenterLinkKey
            } else {
                crate::security::ApsKeyType::NetworkKey
            };
            if let Some(addr) = &sec_hdr.source_address
                && !self
                    .security
                    .check_frame_counter(addr, replay_key_type, sec_hdr.frame_counter)
            {
                log::warn!(
                    "[APS] Replay detected: frame counter {} from src",
                    sec_hdr.frame_counter
                );
                return None;
            }

            // Decrypt (includes MIC verification)
            match self.security.decrypt(aad, ciphertext, &key, &sec_hdr) {
                Some(plaintext) => {
                    // MIC verified — commit frame counter (phase 2 of replay protection)
                    if let Some(addr) = &sec_hdr.source_address {
                        self.security.commit_frame_counter(
                            addr,
                            replay_key_type,
                            sec_hdr.frame_counter,
                        );
                    }
                    // Copy to owned buffer
                    let pt_len = plaintext.len().min(decrypted_buf.data.len());
                    decrypted_buf.data[..pt_len].copy_from_slice(&plaintext[..pt_len]);
                    decrypted_buf.len = pt_len;
                    decrypted_buf.len = pt_len;
                    log::debug!(
                        "[APS] Decrypted APS frame ({} bytes) from 0x{:04X}",
                        pt_len,
                        nwk_src.0
                    );
                    (decrypted_buf.payload(), sec_consumed)
                }
                None => {
                    log::warn!(
                        "[APS] APS decryption failed from 0x{:04X} (key_id={})",
                        nwk_src.0,
                        key_id
                    );
                    return None;
                }
            }
        } else {
            (after_header, 0)
        };

        // Only deliver Data frames to the upper layer
        let ft = crate::frames::ApsFrameType::from_u8(header.frame_control.frame_type)?;
        match ft {
            ApsFrameType::Data => {
                // APS duplicate rejection — drop duplicate data frames
                if self.is_aps_duplicate(nwk_src.0, header.aps_counter) {
                    log::debug!(
                        "APS duplicate rejected: src=0x{:04X} counter={}",
                        nwk_src.0,
                        header.aps_counter
                    );
                    return None;
                }
            }
            ApsFrameType::Ack => {
                // TODO: match APS ack to pending request
                log::debug!("APS ACK received (counter={})", header.aps_counter);
                return None;
            }
            ApsFrameType::Command => {
                // APS command frame — parse command ID and handle key management
                if effective_payload.is_empty() {
                    log::warn!("APS command frame with empty payload");
                    return None;
                }
                let cmd_id = effective_payload[0];
                let cmd_data = &effective_payload[1..];
                match crate::frames::ApsCommandId::from_u8(cmd_id) {
                    Some(crate::frames::ApsCommandId::TransportKey) => {
                        self.handle_transport_key(cmd_data, nwk_src);
                    }
                    Some(crate::frames::ApsCommandId::SwitchKey) => {
                        self.handle_switch_key(cmd_data, nwk_src);
                    }
                    Some(crate::frames::ApsCommandId::VerifyKey) => {
                        log::debug!("APS Verify-Key from 0x{:04X}", nwk_src.0);
                    }
                    Some(crate::frames::ApsCommandId::ConfirmKey) => {
                        log::debug!("APS Confirm-Key from 0x{:04X}", nwk_src.0);
                    }
                    Some(other) => {
                        log::debug!("APS command {:?} from 0x{:04X}", other, nwk_src.0);
                    }
                    None => {
                        log::debug!("Unknown APS command 0x{:02X}", cmd_id);
                    }
                }
                return None;
            }
            ApsFrameType::InterPan => {
                log::debug!("Inter-PAN frame received");
                return None;
            }
        }

        // Generate APS ACK if requested
        if header.frame_control.ack_request {
            self.pending_aps_ack = Some(PendingApsAck {
                dst_addr: nwk_src,
                dst_endpoint: header.src_endpoint.unwrap_or(0),
                src_endpoint: header.dst_endpoint.unwrap_or(0),
                cluster_id: header.cluster_id.unwrap_or(0),
                profile_id: header.profile_id.unwrap_or(0),
                aps_counter: header.aps_counter,
            });
        }

        // Determine addressing
        let dm = crate::frames::ApsDeliveryMode::from_u8(header.frame_control.delivery_mode)?;
        let (dst_addr_mode, dst_address, dst_ep) = match dm {
            ApsDeliveryMode::Unicast | ApsDeliveryMode::Broadcast => (
                ApsAddressMode::Short,
                ApsAddress::Short(nwk_dst),
                header.dst_endpoint.unwrap_or(0),
            ),
            ApsDeliveryMode::Group => {
                let ga = header.group_address.unwrap_or(0);
                (ApsAddressMode::Group, ApsAddress::Group(ga), 0xFF)
            }
            ApsDeliveryMode::Indirect => (
                ApsAddressMode::Indirect,
                ApsAddress::Short(nwk_dst),
                header.dst_endpoint.unwrap_or(0),
            ),
        };

        let payload = effective_payload;

        Some(ApsdeDataIndication {
            dst_addr_mode,
            dst_address,
            dst_endpoint: dst_ep,
            src_addr_mode: ApsAddressMode::Short,
            src_address: ApsAddress::Short(nwk_src),
            src_endpoint: header.src_endpoint.unwrap_or(0),
            profile_id: header.profile_id.unwrap_or(0),
            cluster_id: header.cluster_id.unwrap_or(0),
            payload,
            aps_counter: header.aps_counter,
            security_status: aps_secured || nwk_security,
            lqi,
        })
    }

    /// Handle an incoming APS Switch-Key command.
    ///
    /// Activates the network key with the specified sequence number.
    fn handle_switch_key(&mut self, data: &[u8], src: ShortAddress) {
        if data.is_empty() {
            log::warn!("[APS] Switch-Key too short");
            return;
        }
        let key_seq = data[0];
        log::info!(
            "[APS] Switch-Key: activate key seq={} from 0x{:04X}",
            key_seq,
            src.0
        );
        // The NWK security layer already has both keys; just update the active seq
        self.nwk_mut().nib_mut().active_key_seq_number = key_seq;
    }

    /// Build and send an APSME-REQUEST-KEY to the Trust Center.
    ///
    /// After receiving the NWK key via Transport-Key, the device must request
    /// a unique TC link key. Z2M requires this within ~10s of joining.
    pub async fn send_request_key(&mut self, tc_addr: ShortAddress) -> Result<(), ApsStatus> {
        // Request-Key frame: APS command with key_type=0x04 (TC Link Key)
        let aps_counter = self.next_aps_counter();

        // APS command header: frame_type=Command, no endpoints, no cluster/profile
        let aps_header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Command as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false, // Request-Key is sent without APS security
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: None,
            group_address: None,
            cluster_id: None,
            profile_id: None,
            src_endpoint: None,
            aps_counter,
            extended_header: None,
        };

        let mut buf = [0u8; 32];
        let hdr_len = aps_header.serialize(&mut buf);

        // APS command payload: cmd_id(1) + key_type(1)
        buf[hdr_len] = crate::frames::ApsCommandId::RequestKey as u8; // 0x08
        buf[hdr_len + 1] = 0x04; // key_type = TC Link Key
        let total = hdr_len + 2;

        log::info!("[APS] Sending APSME-REQUEST-KEY to TC 0x{:04X}", tc_addr.0);

        self.nwk
            .nlde_data_request(tc_addr, 1, &buf[..total], true, false)
            .await
            .map_err(|_| ApsStatus::NoAck)?;

        Ok(())
    }

    /// Send a pending APS ACK if one is queued.
    pub async fn send_pending_aps_ack(&mut self) -> Result<(), ApsStatus> {
        let ack_info = match self.pending_aps_ack.take() {
            Some(info) => info,
            None => return Ok(()),
        };

        let aps_counter = ack_info.aps_counter;
        let aps_header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Ack as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false,
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: Some(ack_info.dst_endpoint),
            group_address: None,
            cluster_id: Some(ack_info.cluster_id),
            profile_id: Some(ack_info.profile_id),
            src_endpoint: Some(ack_info.src_endpoint),
            aps_counter,
            extended_header: None,
        };

        let mut buf = [0u8; 16];
        let hdr_len = aps_header.serialize(&mut buf);

        let _ = self
            .nwk
            .nlde_data_request(ack_info.dst_addr, 1, &buf[..hdr_len], true, false)
            .await;

        log::debug!(
            "[APS] Sent ACK (counter={}) to 0x{:04X}",
            aps_counter,
            ack_info.dst_addr.0
        );
        Ok(())
    }

    /// Handle an incoming APS Transport-Key command.
    ///
    /// Parses the key data and installs it into the appropriate security
    /// context (NWK key → NwkSecurity, link key → APS security table).
    fn handle_transport_key(&mut self, data: &[u8], src: ShortAddress) {
        // Transport-Key payload: key_type(1) + key(16) + ...
        // For Network Key: key_type(1) + key(16) + key_seq(1) + dst_addr(8) + src_addr(8)
        // Minimum: 1 + 16 = 17 bytes
        if data.len() < 17 {
            log::warn!("[APS] Transport-Key too short ({} bytes)", data.len());
            return;
        }

        let key_type = data[0];
        let mut key = [0u8; 16];
        key.copy_from_slice(&data[1..17]);

        match key_type {
            0x01 => {
                // Standard Network Key
                let key_seq = if data.len() > 17 { data[17] } else { 0 };
                log::info!(
                    "[APS] Transport-Key: Network Key (seq={}) from 0x{:04X}",
                    key_seq,
                    src.0,
                );
                // Install into NWK security
                self.nwk_mut().security_mut().set_network_key(key, key_seq);
                self.nwk_mut().nib_mut().active_key_seq_number = key_seq;
                log::info!("[APS] Network key installed");
            }
            0x03 => {
                // Trust Center Link Key
                log::info!("[APS] Transport-Key: TC Link Key from 0x{:04X}", src.0,);
                let entry = crate::security::ApsLinkKeyEntry {
                    partner_address: [0u8; 8], // TC address
                    key,
                    key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                    outgoing_frame_counter: 0,
                    incoming_frame_counter: 0,
                };
                let _ = self.security_mut().add_key(entry);
            }
            0x04 => {
                // Application Link Key
                log::info!("[APS] Transport-Key: App Link Key from 0x{:04X}", src.0,);
            }
            _ => {
                log::debug!("[APS] Transport-Key: unknown key_type=0x{:02X}", key_type,);
            }
        }
    }
}
