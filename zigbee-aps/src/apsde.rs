//! APSDE — APS Data Entity.
//!
//! Provides the APSDE-DATA service for sending and receiving application
//! data through the APS layer (Zigbee spec 2.2.4).
//!
//! - `APSDE-DATA.request`:    send data from upper layer to a peer
//! - `APSDE-DATA.confirm`:    transmission result
//! - `APSDE-DATA.indication`: received data delivered to upper layer

use crate::frames::{
    ApsDeliveryMode, ApsExtendedHeader, ApsFrameControl, ApsFrameType, ApsHeader, FRAG_FIRST,
    FRAG_NONE, FRAG_SUBSEQUENT,
};
use crate::{ApsAddress, ApsAddressMode, ApsLayer, ApsStatus, ApsTxOptions, PendingApsAck};
use zigbee_mac::MacDriver;
use zigbee_nwk::NwkStatus;
use zigbee_types::{IeeeAddress, ShortAddress};

#[cfg(feature = "trace")]
macro_rules! aps_diag {
    ($($arg:tt)*) => {
        log::trace!($($arg)*);
    };
}
#[cfg(not(feature = "trace"))]
macro_rules! aps_diag {
    ($($arg:tt)*) => {};
}

/// Maximum APS payload size (bytes) before fragmentation is required.
/// Accounts for APS header + APS security overhead in the NWK frame.
pub const APS_MAX_PAYLOAD: usize = 80;

const WIRE_KEY_TYPE_TC_LINK: u8 = 0x04;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ConfirmKeyCommand {
    status: u8,
    key_type: u8,
    destination: IeeeAddress,
}

fn parse_confirm_key_command(data: &[u8]) -> Option<ConfirmKeyCommand> {
    if data.len() < 10 {
        return None;
    }

    let mut destination = [0u8; 8];
    destination.copy_from_slice(&data[2..10]);
    Some(ConfirmKeyCommand {
        status: data[0],
        key_type: data[1],
        destination,
    })
}

fn trust_center_link_key(
    security: &crate::security::ApsSecurity,
    trust_center: &IeeeAddress,
) -> crate::security::AesKey {
    nonzero_ieee(*trust_center)
        .and_then(|address| {
            security
                .find_key(&address, crate::security::ApsKeyType::TrustCenterLinkKey)
                .map(|entry| entry.key)
        })
        .unwrap_or(*security.default_tc_link_key())
}

fn build_verify_key_command(src_ieee: &IeeeAddress, key_type: u8, hash: &[u8; 16]) -> [u8; 26] {
    let mut payload = [0u8; 26];
    payload[0] = crate::frames::ApsCommandId::VerifyKey as u8;
    payload[1] = key_type;
    payload[2..10].copy_from_slice(src_ieee);
    payload[10..26].copy_from_slice(hash);
    payload
}

#[allow(clippy::too_many_arguments)]
fn build_tc_secured_command_frame(
    security: &crate::security::ApsSecurity,
    link_key: &crate::security::AesKey,
    src_ieee: &IeeeAddress,
    aps_counter: u8,
    frame_counter: u32,
    key_identifier: u8,
    ack_request: bool,
    command: &[u8],
    frame: &mut [u8],
) -> Option<usize> {
    // APS command header (2 bytes) + Data-Key auxiliary header (13 bytes).
    if frame.len() < 15 {
        return None;
    }

    let header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Command as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: true,
            ack_request,
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

    // Trust Center commands carry the sender IEEE in the extended nonce so the
    // recipient can construct the CCM nonce before authentication completes.
    let security_header = crate::security::ApsSecurityHeader {
        security_control: (key_identifier << 3) | (1 << 5),
        frame_counter,
        source_address: Some(*src_ieee),
        key_seq_number: None,
    };

    let header_len = header.serialize(frame);
    let security_header_len = security_header.serialize(&mut frame[header_len..]);
    let aad_len = header_len + security_header_len;

    // Zigbee transmits security level 0, but CCM* authenticates the actual
    // level 5 value.
    let mut authenticated_header = [0u8; 16];
    authenticated_header[..aad_len].copy_from_slice(&frame[..aad_len]);
    authenticated_header[header_len] |= crate::security::SEC_LEVEL_ENC_MIC_32;
    let nonce_header = security_header.clone();
    let encrypted = security.encrypt(
        &authenticated_header[..aad_len],
        command,
        link_key,
        &nonce_header,
    )?;

    if aad_len + encrypted.len() > frame.len() {
        return None;
    }
    frame[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    Some(aad_len + encrypted.len())
}

fn nonzero_ieee(address: IeeeAddress) -> Option<IeeeAddress> {
    (address != [0u8; 8]).then_some(address)
}

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
    fn current_trust_center_link_key(&self) -> crate::security::AesKey {
        trust_center_link_key(&self.security, &self.aib.aps_trust_center_address)
    }

    /// APSDE-DATA.request — transmit application data through APS.
    ///
    /// Builds an APS header, optionally encrypts with a link key, fragments
    /// if needed, serializes into NWK NSDUs, and calls `nlde_data_request`.
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
                let mode = if addr.0 >= 0xFFF8 {
                    ApsDeliveryMode::Broadcast
                } else {
                    ApsDeliveryMode::Unicast
                };
                (addr, mode)
            }
            ApsAddressMode::Group => {
                let _group = match req.dst_address {
                    ApsAddress::Group(g) => g,
                    _ => return Err(ApsStatus::InvalidParameter),
                };
                (ShortAddress(0xFFFF), ApsDeliveryMode::Group)
            }
            ApsAddressMode::Extended => {
                let ieee = match req.dst_address {
                    ApsAddress::Extended(addr) => addr,
                    _ => return Err(ApsStatus::InvalidParameter),
                };
                match self.nwk.find_short_by_ieee(&ieee) {
                    Some(short) => (short, ApsDeliveryMode::Unicast),
                    None => return Err(ApsStatus::NoShortAddress),
                }
            }
            ApsAddressMode::Indirect => {
                let ieee = self.nwk.nib().ieee_address;
                let has_binding = self
                    .binding_table
                    .find_by_source(&ieee, req.src_endpoint, req.cluster_id)
                    .next()
                    .is_some();
                if !has_binding {
                    return Err(ApsStatus::NoBoundDevice);
                }
                (ShortAddress::COORDINATOR, ApsDeliveryMode::Indirect)
            }
        };

        let radius = if req.radius == 0 {
            self.nwk.nib().max_depth.saturating_mul(2)
        } else {
            req.radius
        };

        // APS-level encryption
        if req.tx_options.security_enabled {
            // If the payload needs fragmentation, use fragment-then-encrypt path
            if req.payload.len() > APS_MAX_PAYLOAD && req.tx_options.fragmentation_permitted {
                return self
                    .send_fragmented_secured(req, nwk_dst, delivery_mode, radius)
                    .await;
            }

            let dst_ieee = self.nwk.find_ieee_by_short(nwk_dst);
            let (key, frame_counter) = self
                .next_aps_link_key_material(dst_ieee.as_ref())
                .ok_or(ApsStatus::SecurityFail)?;
            let src_ieee = self.nwk.nib().ieee_address;
            let sec_hdr = crate::security::ApsSecurityHeader {
                security_control: crate::security::ApsSecurityHeader::APS_DEFAULT_EXT_NONCE,
                frame_counter,
                source_address: Some(src_ieee),
                key_seq_number: None,
            };

            let aps_counter = self.next_aps_counter();
            let aps_header = self.build_data_header(delivery_mode, req, aps_counter, true, false);

            // Serialize header for AAD
            let mut aad_buf = [0u8; 32];
            let hdr_len = aps_header.serialize(&mut aad_buf);
            let sec_hdr_len = sec_hdr.serialize(&mut aad_buf[hdr_len..]);
            let aad = &aad_buf[..hdr_len + sec_hdr_len];

            if let Some(enc) = self.security.encrypt(aad, req.payload, &key, &sec_hdr) {
                let mut encrypted_buf = [0u8; 128];
                let mut offset = 0;
                let aps_hdr_len = aps_header.serialize(&mut encrypted_buf);
                offset += aps_hdr_len;
                let sec_len = sec_hdr.serialize(&mut encrypted_buf[offset..]);
                offset += sec_len;
                if offset + enc.len() > encrypted_buf.len() {
                    return Err(ApsStatus::AsduTooLong);
                }
                encrypted_buf[offset..offset + enc.len()].copy_from_slice(&enc);
                let total = offset + enc.len();

                let nwk_result = self
                    .nwk
                    .nlde_data_request(
                        nwk_dst,
                        radius,
                        &encrypted_buf[..total],
                        req.tx_options.use_nwk_key,
                        true,
                    )
                    .await;

                match nwk_result {
                    Ok(_) => {
                        if req.tx_options.ack_request {
                            self.register_ack_pending(
                                aps_counter,
                                nwk_dst.0,
                                &encrypted_buf[..total],
                            );
                        }
                        return Ok(ApsdeDataConfirm {
                            status: ApsStatus::Success,
                            dst_addr_mode: req.dst_addr_mode,
                            dst_address: req.dst_address,
                            dst_endpoint: req.dst_endpoint,
                            src_endpoint: req.src_endpoint,
                            aps_counter,
                        });
                    }
                    Err(nwk_err) => {
                        return Err(nwk_status_to_aps(nwk_err));
                    }
                }
            } else {
                log::warn!("[APS] APS encryption failed");
                return Err(ApsStatus::SecurityFail);
            }
        }

        // Check if fragmentation is needed
        if req.payload.len() > APS_MAX_PAYLOAD && req.tx_options.fragmentation_permitted {
            return self
                .send_fragmented(req, nwk_dst, delivery_mode, radius)
                .await;
        }

        // Normal (non-encrypted, non-fragmented) send
        let aps_counter = self.next_aps_counter();
        let aps_header = self.build_data_header(delivery_mode, req, aps_counter, false, false);

        let mut aps_buf = [0u8; 128];
        let hdr_len = aps_header.serialize(&mut aps_buf);
        let total_len = hdr_len + req.payload.len();
        if total_len > aps_buf.len() {
            return Err(ApsStatus::AsduTooLong);
        }
        aps_buf[hdr_len..total_len].copy_from_slice(req.payload);

        log::info!(
            "[APS TX] ep={}/{} cl=0x{:04X} prof=0x{:04X} cnt={} hdr={:02X?}",
            req.src_endpoint,
            req.dst_endpoint,
            req.cluster_id,
            req.profile_id,
            aps_counter,
            &aps_buf[..core::cmp::min(12, total_len)],
        );

        let nwk_result = self
            .nwk
            .nlde_data_request(
                nwk_dst,
                radius,
                &aps_buf[..total_len],
                req.tx_options.use_nwk_key,
                true,
            )
            .await;

        match nwk_result {
            Ok(_) => {
                if req.tx_options.ack_request {
                    self.register_ack_pending(aps_counter, nwk_dst.0, &aps_buf[..total_len]);
                }
                Ok(ApsdeDataConfirm {
                    status: ApsStatus::Success,
                    dst_addr_mode: req.dst_addr_mode,
                    dst_address: req.dst_address,
                    dst_endpoint: req.dst_endpoint,
                    src_endpoint: req.src_endpoint,
                    aps_counter,
                })
            }
            Err(nwk_err) => {
                log::warn!("APSDE-DATA.request failed: NWK error {:?}", nwk_err);
                Err(nwk_status_to_aps(nwk_err))
            }
        }
    }

    /// Build a standard APS Data header.
    fn build_data_header(
        &self,
        delivery_mode: ApsDeliveryMode,
        req: &ApsdeDataRequest<'_>,
        aps_counter: u8,
        security: bool,
        extended_header: bool,
    ) -> ApsHeader {
        ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Data as u8,
                delivery_mode: delivery_mode as u8,
                ack_format: false,
                security,
                ack_request: req.tx_options.ack_request,
                extended_header,
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
        }
    }

    /// Send a payload as multiple APS fragments.
    async fn send_fragmented(
        &mut self,
        req: &ApsdeDataRequest<'_>,
        nwk_dst: ShortAddress,
        delivery_mode: ApsDeliveryMode,
        radius: u8,
    ) -> Result<ApsdeDataConfirm, ApsStatus> {
        let aps_counter = self.next_aps_counter();
        let total_blocks = req.payload.len().div_ceil(APS_MAX_PAYLOAD) as u8;

        for block_num in 0..total_blocks {
            let start = block_num as usize * APS_MAX_PAYLOAD;
            let end = (start + APS_MAX_PAYLOAD).min(req.payload.len());
            let chunk = &req.payload[start..end];

            let (fragmentation, ack_bitfield) = if block_num == 0 {
                (FRAG_FIRST, Some(0u8))
            } else {
                (FRAG_SUBSEQUENT, None)
            };

            let frag_header = ApsHeader {
                frame_control: ApsFrameControl {
                    frame_type: ApsFrameType::Data as u8,
                    delivery_mode: delivery_mode as u8,
                    ack_format: false,
                    security: false,
                    ack_request: req.tx_options.ack_request && block_num == total_blocks - 1,
                    extended_header: true,
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
                extended_header: Some(ApsExtendedHeader {
                    fragmentation,
                    block_number: if block_num == 0 {
                        total_blocks
                    } else {
                        block_num
                    },
                    ack_bitfield,
                }),
            };

            let mut frag_buf = [0u8; 128];
            let hdr_len = frag_header.serialize(&mut frag_buf);
            let total = hdr_len + chunk.len();
            if total > frag_buf.len() {
                return Err(ApsStatus::AsduTooLong);
            }
            frag_buf[hdr_len..total].copy_from_slice(chunk);

            let nwk_result = self
                .nwk
                .nlde_data_request(
                    nwk_dst,
                    radius,
                    &frag_buf[..total],
                    req.tx_options.use_nwk_key,
                    true,
                )
                .await;

            if let Err(nwk_err) = nwk_result {
                log::warn!(
                    "[APS] Fragment {}/{} send failed: {:?}",
                    block_num,
                    total_blocks,
                    nwk_err
                );
                return Err(nwk_status_to_aps(nwk_err));
            }
        }

        Ok(ApsdeDataConfirm {
            status: ApsStatus::Success,
            dst_addr_mode: req.dst_addr_mode,
            dst_address: req.dst_address,
            dst_endpoint: req.dst_endpoint,
            src_endpoint: req.src_endpoint,
            aps_counter,
        })
    }

    /// Send a large payload as fragments, encrypting each fragment individually.
    ///
    /// This implements the correct fragment-then-encrypt approach for APS security:
    /// 1. Split plaintext into APS_MAX_PAYLOAD-sized chunks
    /// 2. For each chunk, build APS header with security flag
    /// 3. Encrypt the chunk with the APS key
    /// 4. Send via NWK
    async fn send_fragmented_secured(
        &mut self,
        req: &ApsdeDataRequest<'_>,
        nwk_dst: ShortAddress,
        delivery_mode: ApsDeliveryMode,
        radius: u8,
    ) -> Result<ApsdeDataConfirm, ApsStatus> {
        log::debug!(
            "[APS] Sending secured fragmented: {} bytes → {} fragments",
            req.payload.len(),
            req.payload.len().div_ceil(APS_MAX_PAYLOAD),
        );

        let dst_ieee = self.nwk.find_ieee_by_short(nwk_dst);
        let src_ieee = self.nwk.nib().ieee_address;

        let aps_counter = self.next_aps_counter();
        let total_blocks = req.payload.len().div_ceil(APS_MAX_PAYLOAD) as u8;

        for block_num in 0..total_blocks {
            let start = block_num as usize * APS_MAX_PAYLOAD;
            let end = (start + APS_MAX_PAYLOAD).min(req.payload.len());
            let chunk = &req.payload[start..end];

            let (fragmentation, ack_bitfield) = if block_num == 0 {
                (FRAG_FIRST, Some(0u8))
            } else {
                (FRAG_SUBSEQUENT, None)
            };

            let frag_header = ApsHeader {
                frame_control: ApsFrameControl {
                    frame_type: ApsFrameType::Data as u8,
                    delivery_mode: delivery_mode as u8,
                    ack_format: false,
                    security: true,
                    ack_request: req.tx_options.ack_request && block_num == total_blocks - 1,
                    extended_header: true,
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
                extended_header: Some(ApsExtendedHeader {
                    fragmentation,
                    block_number: if block_num == 0 {
                        total_blocks
                    } else {
                        block_num
                    },
                    ack_bitfield,
                }),
            };

            // Encrypt this fragment
            let (link_key, frame_counter) = self
                .next_aps_link_key_material(dst_ieee.as_ref())
                .ok_or(ApsStatus::SecurityFail)?;
            let sec_hdr = crate::security::ApsSecurityHeader {
                security_control: crate::security::ApsSecurityHeader::APS_DEFAULT_EXT_NONCE,
                frame_counter,
                source_address: Some(src_ieee),
                key_seq_number: None,
            };

            let mut aad_buf = [0u8; 32];
            let hdr_len = frag_header.serialize(&mut aad_buf);
            let sec_hdr_len = sec_hdr.serialize(&mut aad_buf[hdr_len..]);
            let aad = &aad_buf[..hdr_len + sec_hdr_len];

            if let Some(enc) = self.security.encrypt(aad, chunk, &link_key, &sec_hdr) {
                let mut frag_buf = [0u8; 128];
                let mut offset = frag_header.serialize(&mut frag_buf);
                let sec_len = sec_hdr.serialize(&mut frag_buf[offset..]);
                offset += sec_len;
                if offset + enc.len() > frag_buf.len() {
                    return Err(ApsStatus::AsduTooLong);
                }
                frag_buf[offset..offset + enc.len()].copy_from_slice(&enc);
                let total = offset + enc.len();

                let nwk_result = self
                    .nwk
                    .nlde_data_request(
                        nwk_dst,
                        radius,
                        &frag_buf[..total],
                        req.tx_options.use_nwk_key,
                        true,
                    )
                    .await;

                if let Err(nwk_err) = nwk_result {
                    log::warn!(
                        "[APS] Secured fragment {}/{} send failed: {:?}",
                        block_num,
                        total_blocks,
                        nwk_err
                    );
                    return Err(nwk_status_to_aps(nwk_err));
                }
            } else {
                log::warn!(
                    "[APS] Fragment {}/{} encryption failed",
                    block_num,
                    total_blocks
                );
                return Err(ApsStatus::SecurityFail);
            }
        }

        Ok(ApsdeDataConfirm {
            status: ApsStatus::Success,
            dst_addr_mode: req.dst_addr_mode,
            dst_address: req.dst_address,
            dst_endpoint: req.dst_endpoint,
            src_endpoint: req.src_endpoint,
            aps_counter,
        })
    }
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
        decrypted_buf: &'a mut ApsFrameBuffer,
    ) -> Option<ApsdeDataIndication<'a>> {
        aps_diag!("[APS] RX {} bytes", nwk_payload.len());

        let (header, consumed) = ApsHeader::parse(nwk_payload)?;
        aps_diag!(
            "[APS] type={} sec={} consumed={}",
            header.frame_control.frame_type,
            header.frame_control.security,
            consumed
        );

        let aps_secured = header.frame_control.security;
        let after_header = &nwk_payload[consumed..];
        let mut used_decrypted_buf = false;
        let mut aps_security_source = None;

        // Phase 1: APS security decryption
        if aps_secured {
            aps_diag!("[APS] secured payload has {} bytes", after_header.len());
            #[allow(clippy::question_mark)]
            let Some((mut sec_hdr, sec_consumed)) =
                crate::security::ApsSecurityHeader::parse(after_header)
            else {
                aps_diag!(
                    "[APS] security header parse failed for {} bytes",
                    after_header.len()
                );
                return None;
            };
            if sec_hdr.source_address.is_none() {
                sec_hdr.source_address = self
                    .nwk
                    .find_ieee_by_short(nwk_src)
                    .or_else(|| nonzero_ieee(self.aib.aps_trust_center_address));
            }
            aps_security_source = sec_hdr.source_address;
            aps_diag!(
                "[APS] sec: ctrl={:02X} fc={} sc={} ct={}",
                sec_hdr.security_control,
                sec_hdr.frame_counter,
                sec_consumed,
                after_header.len() - sec_consumed
            );
            let ciphertext = &after_header[sec_consumed..];
            let aad_end = consumed + sec_consumed;
            // AAD must use the ACTUAL security level (5 = ENC-MIC-32), not the OTA value (0).
            // The sender computes CCM* with actual level, then zeroes it for transmission.
            // Copy AAD and patch the security control byte with actual level.
            let mut aad_buf_patched = [0u8; 64];
            let aad_len = aad_end.min(aad_buf_patched.len());
            aad_buf_patched[..aad_len].copy_from_slice(&nwk_payload[..aad_len]);
            // The security control byte is at offset `consumed` (first byte of aux header)
            aad_buf_patched[consumed] =
                (aad_buf_patched[consumed] & !0x07) | crate::security::SEC_LEVEL_ENC_MIC_32;
            let aad = &aad_buf_patched[..aad_len];

            let key_id =
                crate::security::ApsSecurityHeader::key_identifier(sec_hdr.security_control);
            aps_diag!(
                "[APS] key_id={} aad_len={} ct_len={} src_ieee={}",
                key_id,
                aad_len,
                ciphertext.len(),
                sec_hdr.source_address.is_some() as u8,
            );

            let key = if key_id == crate::security::KEY_ID_DATA_KEY {
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
                let tck = *self.security.default_tc_link_key();
                crate::security::derive_key_transport_key(&tck)
            } else if key_id == crate::security::KEY_ID_KEY_LOAD {
                let tc_key = self.current_trust_center_link_key();
                crate::security::derive_key_load_key(&tc_key)
            } else {
                log::warn!("[APS] Unsupported key_id={} in APS security", key_id);
                return None;
            };

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

            // Try decrypt with patched AAD (standard: OTA level→5).
            // If that fails AND this is a key-transport frame, try fallback approaches:
            //   1. AAD with original OTA security level (some coordinators don't strip)
            //   2. Raw TC link key instead of derived key-transport key
            let mut decrypt_ok = false;
            if let Some(plaintext) = self.security.decrypt(aad, ciphertext, &key, &sec_hdr) {
                aps_diag!("[APS] decrypt succeeded with patched AAD");
                let pt_len = plaintext.len().min(decrypted_buf.data.len());
                decrypted_buf.data[..pt_len].copy_from_slice(&plaintext[..pt_len]);
                decrypted_buf.len = pt_len;
                used_decrypted_buf = true;
                decrypt_ok = true;
            }

            // Fallback: try with un-patched AAD (original OTA security level)
            if !decrypt_ok {
                let aad_raw = &nwk_payload[..aad_end.min(nwk_payload.len())];
                if let Some(plaintext) = self.security.decrypt(aad_raw, ciphertext, &key, &sec_hdr)
                {
                    aps_diag!("[APS] decrypt succeeded with raw AAD");
                    let pt_len = plaintext.len().min(decrypted_buf.data.len());
                    decrypted_buf.data[..pt_len].copy_from_slice(&plaintext[..pt_len]);
                    decrypted_buf.len = pt_len;
                    used_decrypted_buf = true;
                    decrypt_ok = true;
                }
            }

            // Fallback for key-transport: try raw TC link key (some impls don't derive)
            if !decrypt_ok && key_id == crate::security::KEY_ID_KEY_TRANSPORT {
                let tc_key = *self.security.default_tc_link_key();
                if let Some(plaintext) = self.security.decrypt(aad, ciphertext, &tc_key, &sec_hdr) {
                    aps_diag!("[APS] key-transport decrypt succeeded with raw TC key");
                    let pt_len = plaintext.len().min(decrypted_buf.data.len());
                    decrypted_buf.data[..pt_len].copy_from_slice(&plaintext[..pt_len]);
                    decrypted_buf.len = pt_len;
                    used_decrypted_buf = true;
                    decrypt_ok = true;
                }
                // Try with un-patched AAD
                if !decrypt_ok {
                    let aad_raw = &nwk_payload[..aad_end.min(nwk_payload.len())];
                    if let Some(plaintext) = self
                        .security
                        .decrypt(aad_raw, ciphertext, &tc_key, &sec_hdr)
                    {
                        aps_diag!(
                            "[APS] key-transport decrypt succeeded with raw TC key and raw AAD"
                        );
                        let pt_len = plaintext.len().min(decrypted_buf.data.len());
                        decrypted_buf.data[..pt_len].copy_from_slice(&plaintext[..pt_len]);
                        decrypted_buf.len = pt_len;
                        used_decrypted_buf = true;
                        decrypt_ok = true;
                    }
                }
            }

            if decrypt_ok {
                if let Some(addr) = &sec_hdr.source_address {
                    self.security.commit_frame_counter(
                        addr,
                        replay_key_type,
                        sec_hdr.frame_counter,
                    );
                }
            } else {
                aps_diag!(
                    "[APS] decrypt ALL FAILED key_id={} ct_len={}",
                    key_id,
                    ciphertext.len()
                );
                return None;
            }
        }

        // Phase 2: Frame type dispatch
        let ft = crate::frames::ApsFrameType::from_u8(header.frame_control.frame_type)?;
        match ft {
            ApsFrameType::Data => {
                if self.is_aps_duplicate(nwk_src.0, header.aps_counter) {
                    log::info!(
                        "APS duplicate rejected: src=0x{:04X} counter={}",
                        nwk_src.0,
                        header.aps_counter
                    );
                    return None;
                }

                // Handle fragmented frames
                if header.frame_control.extended_header
                    && let Some(ref ext) = header.extended_header
                    && ext.fragmentation != FRAG_NONE
                {
                    let total_blocks = if ext.fragmentation == FRAG_FIRST {
                        ext.block_number
                    } else {
                        0
                    };
                    let block_num = if ext.fragmentation == FRAG_FIRST {
                        0
                    } else {
                        ext.block_number
                    };

                    // Copy fragment data to temp buffer to avoid borrow conflict
                    let mut frag_tmp = [0u8; 128];
                    let frag_len = if used_decrypted_buf {
                        let l = decrypted_buf.len.min(frag_tmp.len());
                        frag_tmp[..l].copy_from_slice(&decrypted_buf.data[..l]);
                        l
                    } else {
                        let l = after_header.len().min(frag_tmp.len());
                        frag_tmp[..l].copy_from_slice(&after_header[..l]);
                        l
                    };

                    let is_complete;
                    {
                        let result = self.fragment_rx.insert_fragment(
                            nwk_src.0,
                            header.aps_counter,
                            block_num,
                            total_blocks,
                            &frag_tmp[..frag_len],
                        );
                        if let Some(reassembled) = result {
                            let rlen = reassembled.len().min(decrypted_buf.data.len());
                            decrypted_buf.data[..rlen].copy_from_slice(&reassembled[..rlen]);
                            decrypted_buf.len = rlen;
                            is_complete = true;
                        } else {
                            is_complete = false;
                        }
                    }

                    if is_complete {
                        self.fragment_rx
                            .complete_entry(nwk_src.0, header.aps_counter);
                        used_decrypted_buf = true;
                    } else {
                        return None;
                    }
                }
            }
            ApsFrameType::Ack => {
                if !self.confirm_ack(nwk_src.0, header.aps_counter) {
                    log::debug!(
                        "APS ACK received (counter={}) - no matching pending",
                        header.aps_counter
                    );
                }
                return None;
            }
            ApsFrameType::Command => {
                log::info!("[APS RX] APS Command frame, sec={}", aps_secured);
                let cmd_payload = if used_decrypted_buf {
                    &decrypted_buf.data[..decrypted_buf.len]
                } else {
                    after_header
                };
                if cmd_payload.is_empty() {
                    log::warn!("APS command frame with empty payload");
                    return None;
                }
                let cmd_id = cmd_payload[0];
                let cmd_data = &cmd_payload[1..];
                aps_diag!("[APS] command ID={:02X} data={}", cmd_id, cmd_data.len());
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
                        self.security_handshake_stats.confirm_key_received = self
                            .security_handshake_stats
                            .confirm_key_received
                            .wrapping_add(1);
                        self.security_handshake_stats.last_confirm_key_source = nwk_src.0;
                        self.security_handshake_stats.last_confirm_key_source_ieee =
                            aps_security_source.unwrap_or([0u8; 8]);
                        let valid = if let Some(command) = parse_confirm_key_command(cmd_data) {
                            self.security_handshake_stats.last_confirm_key_status = command.status;
                            self.security_handshake_stats.last_confirm_key_type = command.key_type;
                            self.security_handshake_stats.last_confirm_key_destination =
                                command.destination;
                            command.status == 0x00
                                && command.key_type == WIRE_KEY_TYPE_TC_LINK
                                && aps_secured
                                && nwk_src.0 == 0x0000
                                && aps_security_source
                                    == nonzero_ieee(self.aib.aps_trust_center_address)
                                && command.destination == self.nwk.nib().ieee_address
                        } else {
                            self.security_handshake_stats.last_confirm_key_status = 0xFF;
                            self.security_handshake_stats.last_confirm_key_type = 0xFF;
                            self.security_handshake_stats.last_confirm_key_destination = [0u8; 8];
                            false
                        };
                        if valid {
                            self.security_handshake_stats.confirm_key_successes = self
                                .security_handshake_stats
                                .confirm_key_successes
                                .wrapping_add(1);
                        } else {
                            self.security_handshake_stats.confirm_key_rejections = self
                                .security_handshake_stats
                                .confirm_key_rejections
                                .wrapping_add(1);
                        }
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

        let payload = if used_decrypted_buf {
            decrypted_buf.payload()
        } else {
            after_header
        };

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

    /// Build and send an APS command frame.
    ///
    /// Common helper for APSME-TRANSPORT-KEY, REQUEST-KEY, SWITCH-KEY, VERIFY-KEY.
    async fn send_aps_command(
        &mut self,
        dst: ShortAddress,
        cmd_payload: &[u8],
        secured: bool,
    ) -> Result<(), ApsStatus> {
        let aps_counter = self.next_aps_counter();
        let aps_header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Command as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: secured,
                ack_request: true,
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

        let mut buf = [0u8; 80];
        let hdr_len = aps_header.serialize(&mut buf);
        let payload_len = cmd_payload.len();
        if hdr_len + payload_len > buf.len() {
            return Err(ApsStatus::IllegalRequest);
        }
        buf[hdr_len..hdr_len + payload_len].copy_from_slice(cmd_payload);
        let total = hdr_len + payload_len;

        let radius = self.nwk.nib().max_depth.saturating_mul(2);
        self.nwk
            .nlde_data_request(dst, radius, &buf[..total], true, false)
            .await
            .map(|_| ())
            .map_err(|_| ApsStatus::NoAck)
    }

    /// Build and send an APSME-REQUEST-KEY to the Trust Center.
    ///
    /// After receiving the NWK key via Transport-Key, the device must request
    /// a unique TC link key. Z2M requires this within ~10s of joining.
    pub async fn send_request_key(&mut self, tc_addr: ShortAddress) -> Result<(), ApsStatus> {
        log::info!("[APS] Sending APSME-REQUEST-KEY to TC 0x{:04X}", tc_addr.0);
        let local_ieee = self.nwk.nib().ieee_address;
        let command = [
            crate::frames::ApsCommandId::RequestKey as u8, // 0x08
            0x04,                                          // key_type = TC Link Key
        ];
        let key = *self.security.default_tc_link_key();
        let frame_counter = self
            .next_default_tc_link_key_frame_counter()
            .ok_or(ApsStatus::SecurityFail)?;
        self.send_link_key_secured_command(
            tc_addr,
            &local_ieee,
            &key,
            frame_counter,
            crate::security::KEY_ID_DATA_KEY,
            false,
            &command,
        )
        .await
    }

    /// Build and send an APSME-TRANSPORT-KEY command frame.
    pub async fn send_transport_key(
        &mut self,
        dst: ShortAddress,
        key_type: u8,
        key: &[u8; 16],
        key_seq_number: u8,
        src_ieee: &IeeeAddress,
    ) -> Result<(), ApsStatus> {
        log::info!(
            "[APS] Sending Transport-Key to 0x{:04X} type={key_type}",
            dst.0
        );
        // cmd_id(1) + key_type(1) + key(16) + key_seq(1) + src_ieee(8)
        let mut payload = [0u8; 27];
        payload[0] = crate::frames::ApsCommandId::TransportKey as u8;
        payload[1] = key_type;
        payload[2..18].copy_from_slice(key);
        payload[18] = key_seq_number;
        payload[19..27].copy_from_slice(src_ieee);
        self.send_aps_command(dst, &payload, true).await
    }

    /// Build and send an APSME-SWITCH-KEY command frame.
    pub async fn send_switch_key(
        &mut self,
        dst: ShortAddress,
        key_seq_number: u8,
    ) -> Result<(), ApsStatus> {
        log::info!(
            "[APS] Sending Switch-Key to 0x{:04X} seq={key_seq_number}",
            dst.0
        );
        // cmd_id(1) + key_seq(1)
        let payload = [crate::frames::ApsCommandId::SwitchKey as u8, key_seq_number];
        self.send_aps_command(dst, &payload, true).await
    }

    /// Build and send an APSME-VERIFY-KEY command frame.
    pub async fn send_verify_key(
        &mut self,
        dst: ShortAddress,
        src_ieee: &IeeeAddress,
        key_type: u8,
        hash: &[u8; 16],
    ) -> Result<(), ApsStatus> {
        log::info!(
            "[APS] Sending Verify-Key to 0x{:04X} type={key_type}",
            dst.0
        );
        let payload = build_verify_key_command(src_ieee, key_type, hash);
        let (key, frame_counter) = self
            .next_current_tc_link_key_material()
            .ok_or(ApsStatus::SecurityFail)?;
        self.send_link_key_secured_command(
            dst,
            src_ieee,
            &key,
            frame_counter,
            crate::security::KEY_ID_DATA_KEY,
            true,
            &payload,
        )
        .await?;
        self.security_handshake_stats.verify_key_sent = self
            .security_handshake_stats
            .verify_key_sent
            .wrapping_add(1);
        Ok(())
    }

    /// Send APSME-VERIFY-KEY using the installed per-device Trust Center link
    /// key, falling back to the preconfigured global key if no unique key has
    /// been received.
    pub async fn send_tc_verify_key(&mut self, tc_addr: ShortAddress) -> Result<(), ApsStatus> {
        let local_ieee = self.nwk.nib().ieee_address;
        let tc_key = self.current_trust_center_link_key();
        let hash = crate::security::derive_verify_key_hash(&tc_key);
        self.send_verify_key(tc_addr, &local_ieee, WIRE_KEY_TYPE_TC_LINK, &hash)
            .await
    }

    fn next_default_tc_link_key_frame_counter(&mut self) -> Option<u32> {
        // The preconfigured global TC link key uses the same persistent
        // outgoing security counter as NWK security. This matches the Telink
        // stack and prevents Request-Key from restarting at zero after boot.
        self.nwk.nib_mut().next_frame_counter()
    }

    fn next_aps_link_key_material(
        &mut self,
        destination: Option<&IeeeAddress>,
    ) -> Option<(crate::security::AesKey, u32)> {
        if let Some(destination) = destination
            && let Some((key, key_type)) = self
                .security
                .find_any_key(destination)
                .map(|entry| (entry.key, entry.key_type))
        {
            let frame_counter = self.security.next_frame_counter(destination, key_type)?;
            return Some((key, frame_counter));
        }

        let key = *self.security.default_tc_link_key();
        let frame_counter = self.next_default_tc_link_key_frame_counter()?;
        Some((key, frame_counter))
    }

    fn next_current_tc_link_key_material(&mut self) -> Option<(crate::security::AesKey, u32)> {
        if let Some(tc_ieee) = nonzero_ieee(self.aib.aps_trust_center_address)
            && let Some(key) = self
                .security
                .find_key(&tc_ieee, crate::security::ApsKeyType::TrustCenterLinkKey)
                .map(|entry| entry.key)
        {
            let frame_counter = self
                .security
                .next_frame_counter(&tc_ieee, crate::security::ApsKeyType::TrustCenterLinkKey)?;
            return Some((key, frame_counter));
        }

        Some((
            *self.security.default_tc_link_key(),
            self.next_default_tc_link_key_frame_counter()?,
        ))
    }

    #[allow(clippy::too_many_arguments)]
    async fn send_link_key_secured_command(
        &mut self,
        dst: ShortAddress,
        src_ieee: &IeeeAddress,
        link_key: &crate::security::AesKey,
        frame_counter: u32,
        key_identifier: u8,
        ack_request: bool,
        command: &[u8],
    ) -> Result<(), ApsStatus> {
        let aps_counter = self.next_aps_counter();

        let mut frame = [0u8; 80];
        let total = build_tc_secured_command_frame(
            &self.security,
            link_key,
            src_ieee,
            aps_counter,
            frame_counter,
            key_identifier,
            ack_request,
            command,
            &mut frame,
        )
        .ok_or(ApsStatus::SecurityFail)?;
        let radius = self.nwk.nib().max_depth.saturating_mul(2);

        self.nwk
            .nlde_data_request(dst, radius, &frame[..total], true, false)
            .await
            .map(|_| ())
            .map_err(|_| ApsStatus::NoAck)
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

        let radius = self.nwk.nib().max_depth.saturating_mul(2);
        let _ = self
            .nwk
            .nlde_data_request(ack_info.dst_addr, radius, &buf[..hdr_len], true, false)
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
        aps_diag!(
            "[APS] Transport-Key! {} bytes from 0x{:04X}",
            data.len(),
            src.0
        );
        if data.len() < 17 {
            aps_diag!("[APS] Transport-Key payload too short");
            return;
        }

        let key_type = data[0];
        let mut key = [0u8; 16];
        key.copy_from_slice(&data[1..17]);
        aps_diag!("[APS] Transport-Key type={}", key_type);

        match key_type {
            0x01 => {
                // Standard Network Key
                let key_seq = if data.len() > 17 { data[17] } else { 0 };
                if data.len() >= 34 {
                    let mut tc_ieee = [0u8; 8];
                    tc_ieee.copy_from_slice(&data[26..34]);
                    if let Some(tc_ieee) = nonzero_ieee(tc_ieee) {
                        self.aib_mut().aps_trust_center_address = tc_ieee;
                    }
                }
                aps_diag!("[APS] Installing NWK key seq={}", key_seq);
                self.nwk_mut().security_mut().set_network_key(key, key_seq);
                let nib = self.nwk_mut().nib_mut();
                nib.active_key_seq_number = key_seq;
                nib.security_enabled = true;
                aps_diag!("[APS] NWK key installed");
            }
            0x03 => {
                // Application Link Key
                // Payload: key_type(1) + key(16) + partner_ieee(8) + initiator_flag(1)
                if data.len() < 26 {
                    log::warn!(
                        "[APS] Transport-Key: App Link Key too short ({} bytes)",
                        data.len()
                    );
                    return;
                }
                let mut partner_ieee = [0u8; 8];
                partner_ieee.copy_from_slice(&data[17..25]);
                log::info!(
                    "[APS] Transport-Key: App Link Key from 0x{:04X}, partner={:02X?}",
                    src.0,
                    partner_ieee,
                );
                let entry = crate::security::ApsLinkKeyEntry {
                    partner_address: partner_ieee,
                    key,
                    key_type: crate::security::ApsKeyType::ApplicationLinkKey,
                    outgoing_frame_counter: 0,
                    outgoing_frame_counter_limit: u32::MAX,
                    incoming_frame_counter: 0,
                    incoming_frame_counter_valid: false,
                };
                let _ = self.security_mut().add_key(entry);
                log::info!(
                    "[APS] Application link key installed for partner {:02X?}",
                    partner_ieee
                );
            }
            0x04 => {
                // Trust Center Link Key
                // Payload: key_type(1) + key(16) + dest_ieee(8) + src_ieee(8)
                if data.len() < 33 {
                    log::warn!(
                        "[APS] Transport-Key: TC Link Key too short ({} bytes)",
                        data.len()
                    );
                    return;
                }
                let mut destination_ieee = [0u8; 8];
                destination_ieee.copy_from_slice(&data[17..25]);
                if destination_ieee != self.nwk.nib().ieee_address {
                    log::warn!("[APS] Transport-Key: TC Link Key is for a different destination");
                    return;
                }
                let mut tc_ieee = [0u8; 8];
                tc_ieee.copy_from_slice(&data[25..33]);
                let Some(tc_ieee) = nonzero_ieee(tc_ieee) else {
                    log::warn!("[APS] Transport-Key: TC Link Key has no source IEEE");
                    return;
                };
                if let Some(mapped_ieee) = self.nwk.find_ieee_by_short(src)
                    && mapped_ieee != tc_ieee
                {
                    log::warn!("[APS] Transport-Key: TC source address mismatch");
                    return;
                }
                log::info!(
                    "[APS] Transport-Key: TC Link Key from 0x{:04X}, TC IEEE={:02X?}",
                    src.0,
                    tc_ieee,
                );
                self.aib_mut().aps_trust_center_address = tc_ieee;
                let entry = crate::security::ApsLinkKeyEntry {
                    partner_address: tc_ieee,
                    key,
                    key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                    outgoing_frame_counter: 0,
                    outgoing_frame_counter_limit: u32::MAX,
                    incoming_frame_counter: 0,
                    incoming_frame_counter_valid: false,
                };
                if self.security_mut().add_key(entry).is_err() {
                    log::warn!("[APS] Transport-Key: APS key table is full");
                }
            }
            _ => {
                log::debug!("[APS] Transport-Key: unknown key_type=0x{:02X}", key_type,);
            }
        }
    }
}

/// Convert NWK status to APS status.
fn nwk_status_to_aps(nwk_err: NwkStatus) -> ApsStatus {
    match nwk_err {
        NwkStatus::FrameTooLong => ApsStatus::AsduTooLong,
        NwkStatus::InvalidRequest => ApsStatus::IllegalRequest,
        NwkStatus::RouteError | NwkStatus::RouteDiscoveryFailed => ApsStatus::NoShortAddress,
        _ => ApsStatus::NoAck,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verify_key_command_uses_spec_field_order() {
        let ieee = [0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55, 0x02];
        let hash = [
            0x1A, 0xB1, 0x28, 0xDF, 0x16, 0x39, 0xA1, 0x24, 0x6A, 0xAB, 0xA7, 0x2A, 0x6A, 0x55,
            0x91, 0x24,
        ];

        assert_eq!(
            build_verify_key_command(&ieee, 0x04, &hash),
            [
                0x0F, 0x04, 0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55, 0x02, 0x1A, 0xB1, 0x28, 0xDF,
                0x16, 0x39, 0xA1, 0x24, 0x6A, 0xAB, 0xA7, 0x2A, 0x6A, 0x55, 0x91, 0x24,
            ]
        );
    }

    #[test]
    fn request_key_frame_matches_telink_data_key_security() {
        let ieee = [0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55, 0x02];
        let command = [crate::frames::ApsCommandId::RequestKey as u8, 0x04];
        let security = crate::security::ApsSecurity::new();
        let mut frame = [0u8; 80];

        let len = build_tc_secured_command_frame(
            &security,
            security.default_tc_link_key(),
            &ieee,
            0x5A,
            0x0102_0304,
            crate::security::KEY_ID_DATA_KEY,
            false,
            &command,
            &mut frame,
        )
        .unwrap();

        assert_eq!(
            &frame[..len],
            &[
                0x21, 0x5A, 0x20, 0x04, 0x03, 0x02, 0x01, 0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55,
                0x02, 0x02, 0x0C, 0xFA, 0xE9, 0x6B, 0x8A,
            ]
        );

        assert_secured_command_round_trip(
            &security,
            security.default_tc_link_key(),
            crate::security::KEY_ID_DATA_KEY,
            &frame[..len],
            &command,
        );
    }

    #[test]
    fn request_key_frame_matches_key_load_security() {
        let ieee = [0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55, 0x02];
        let command = [crate::frames::ApsCommandId::RequestKey as u8, 0x04];
        let security = crate::security::ApsSecurity::new();
        let key = crate::security::derive_key_load_key(security.default_tc_link_key());
        let mut frame = [0u8; 80];

        let len = build_tc_secured_command_frame(
            &security,
            &key,
            &ieee,
            0x5A,
            0x0102_0304,
            crate::security::KEY_ID_KEY_LOAD,
            false,
            &command,
            &mut frame,
        )
        .unwrap();

        assert_eq!(
            &frame[..len],
            &[
                0x21, 0x5A, 0x38, 0x04, 0x03, 0x02, 0x01, 0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55,
                0x02, 0x88, 0xFB, 0x6A, 0xD1, 0xF0, 0x35,
            ]
        );

        assert_secured_command_round_trip(
            &security,
            &key,
            crate::security::KEY_ID_KEY_LOAD,
            &frame[..len],
            &command,
        );
    }

    #[test]
    fn request_key_frame_matches_key_transport_security() {
        let ieee = [0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55, 0x02];
        let command = [crate::frames::ApsCommandId::RequestKey as u8, 0x04];
        let security = crate::security::ApsSecurity::new();
        let key = crate::security::derive_key_transport_key(security.default_tc_link_key());
        let mut frame = [0u8; 80];

        let len = build_tc_secured_command_frame(
            &security,
            &key,
            &ieee,
            0x5A,
            0x0102_0304,
            crate::security::KEY_ID_KEY_TRANSPORT,
            false,
            &command,
            &mut frame,
        )
        .unwrap();

        assert_eq!(
            &frame[..len],
            &[
                0x21, 0x5A, 0x30, 0x04, 0x03, 0x02, 0x01, 0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55,
                0x02, 0xF2, 0x7E, 0x19, 0x2D, 0xA8, 0x27,
            ]
        );

        assert_secured_command_round_trip(
            &security,
            &key,
            crate::security::KEY_ID_KEY_TRANSPORT,
            &frame[..len],
            &command,
        );
    }

    #[test]
    fn verify_key_frame_uses_data_key_security() {
        let ieee = [0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55, 0x02];
        let hash = [
            0x1A, 0xB1, 0x28, 0xDF, 0x16, 0x39, 0xA1, 0x24, 0x6A, 0xAB, 0xA7, 0x2A, 0x6A, 0x55,
            0x91, 0x24,
        ];
        let command = build_verify_key_command(&ieee, 0x04, &hash);
        let security = crate::security::ApsSecurity::new();
        let mut frame = [0u8; 80];

        let len = build_tc_secured_command_frame(
            &security,
            security.default_tc_link_key(),
            &ieee,
            0x5A,
            0x0102_0304,
            crate::security::KEY_ID_DATA_KEY,
            true,
            &command,
            &mut frame,
        )
        .unwrap();

        assert_eq!(
            &frame[..len],
            &[
                0x61, 0x5A, 0x20, 0x04, 0x03, 0x02, 0x01, 0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55,
                0x02, 0x05, 0x0C, 0xAA, 0xF6, 0xA2, 0xAF, 0xE2, 0x3A, 0x0E, 0xF8, 0xC6, 0x1B, 0x66,
                0x6B, 0xBE, 0xAA, 0x33, 0x6F, 0x9E, 0x24, 0xC4, 0xE7, 0x7E, 0x23, 0x3C, 0x74, 0xB0,
                0x12, 0x60, 0x62,
            ]
        );

        assert_secured_command_round_trip(
            &security,
            security.default_tc_link_key(),
            crate::security::KEY_ID_DATA_KEY,
            &frame[..len],
            &command,
        );
    }

    fn assert_secured_command_round_trip(
        security: &crate::security::ApsSecurity,
        key: &crate::security::AesKey,
        expected_key_identifier: u8,
        frame: &[u8],
        command: &[u8],
    ) {
        let (_, header_len) = ApsHeader::parse(frame).unwrap();
        let (security_header, security_header_len) =
            crate::security::ApsSecurityHeader::parse(&frame[header_len..]).unwrap();
        assert_eq!(
            crate::security::ApsSecurityHeader::key_identifier(security_header.security_control),
            expected_key_identifier
        );
        assert!(security_header.source_address.is_some());

        let aad_len = header_len + security_header_len;
        let mut authenticated_header = [0u8; 16];
        authenticated_header[..aad_len].copy_from_slice(&frame[..aad_len]);
        authenticated_header[header_len] |= crate::security::SEC_LEVEL_ENC_MIC_32;
        let plaintext = security
            .decrypt(
                &authenticated_header[..aad_len],
                &frame[aad_len..],
                key,
                &security_header,
            )
            .unwrap();
        assert_eq!(plaintext.as_slice(), command);
    }

    #[test]
    fn confirm_key_command_uses_spec_field_order() {
        let destination = [0x25, 0x34, 0x36, 0x39, 0x33, 0x4E, 0x55, 0x02];
        let mut command = [0u8; 10];
        command[0] = 0x00;
        command[1] = WIRE_KEY_TYPE_TC_LINK;
        command[2..].copy_from_slice(&destination);

        assert_eq!(
            parse_confirm_key_command(&command),
            Some(ConfirmKeyCommand {
                status: 0x00,
                key_type: WIRE_KEY_TYPE_TC_LINK,
                destination,
            })
        );
        assert_eq!(parse_confirm_key_command(&command[..9]), None);
    }

    #[test]
    fn unique_trust_center_link_key_replaces_global_key() {
        let trust_center = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF0, 0x01, 0x02];
        let unique_key = [0xA5; 16];
        let mut security = crate::security::ApsSecurity::new();
        security
            .add_key(crate::security::ApsLinkKeyEntry {
                partner_address: trust_center,
                key: unique_key,
                key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0,
                outgoing_frame_counter_limit: u32::MAX,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
            .unwrap();

        assert_eq!(trust_center_link_key(&security, &trust_center), unique_key);
        assert_eq!(
            trust_center_link_key(&security, &[0xBB; 8]),
            *security.default_tc_link_key()
        );
    }
}
