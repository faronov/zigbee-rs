//! APSDE — APS Data Entity.
//!
//! Provides the APSDE-DATA service for sending and receiving application
//! data through the APS layer (Zigbee spec 2.2.4).
//!
//! - `APSDE-DATA.request`:    send data from upper layer to a peer
//! - `APSDE-DATA.confirm`:    transmission result
//! - `APSDE-DATA.indication`: received data delivered to upper layer

use crate::frames::{ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};
use crate::{ApsAddress, ApsAddressMode, ApsLayer, ApsStatus, ApsTxOptions};
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
    pub fn process_incoming_aps_frame<'a>(
        &mut self,
        nwk_payload: &'a [u8],
        nwk_src: ShortAddress,
        nwk_dst: ShortAddress,
        lqi: u8,
        nwk_security: bool,
    ) -> Option<ApsdeDataIndication<'a>> {
        let (header, consumed) = ApsHeader::parse(nwk_payload)?;

        // Only deliver Data frames to the upper layer
        let ft = crate::frames::ApsFrameType::from_u8(header.frame_control.frame_type)?;
        match ft {
            ApsFrameType::Data => {}
            ApsFrameType::Ack => {
                // TODO: match APS ack to pending request
                log::debug!("APS ACK received (counter={})", header.aps_counter);
                return None;
            }
            ApsFrameType::Command => {
                // APS command frame — parse command ID and handle key management
                let cmd_payload = &nwk_payload[consumed..];
                if cmd_payload.is_empty() {
                    log::warn!("APS command frame with empty payload");
                    return None;
                }
                let cmd_id = cmd_payload[0];
                let cmd_data = &cmd_payload[1..];
                match crate::frames::ApsCommandId::from_u8(cmd_id) {
                    Some(crate::frames::ApsCommandId::TransportKey) => {
                        self.handle_transport_key(cmd_data, nwk_src);
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

        let payload = &nwk_payload[consumed..];

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
            security_status: header.frame_control.security || nwk_security,
            lqi,
        })
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
                log::info!(
                    "[APS] Transport-Key: TC Link Key from 0x{:04X}",
                    src.0,
                );
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
                log::info!(
                    "[APS] Transport-Key: App Link Key from 0x{:04X}",
                    src.0,
                );
            }
            _ => {
                log::debug!(
                    "[APS] Transport-Key: unknown key_type=0x{:02X}",
                    key_type,
                );
            }
        }
    }
}
