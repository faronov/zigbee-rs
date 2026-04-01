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

/// Maximum APS payload size (bytes) before fragmentation is required.
/// Accounts for APS header + APS security overhead in the NWK frame.
pub const APS_MAX_PAYLOAD: usize = 80;

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
                (addr, ApsDeliveryMode::Unicast)
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
            let link_key = if let Some(ref ieee) = dst_ieee {
                if let Some(entry) = self.security.find_any_key(ieee) {
                    Some(entry.key)
                } else {
                    Some(*self.security.default_tc_link_key())
                }
            } else {
                Some(*self.security.default_tc_link_key())
            };

            if let Some(key) = link_key {
                let src_ieee = self.nwk.nib().ieee_address;
                let frame_counter = if let Some(ref ieee) = dst_ieee {
                    self.security
                        .next_frame_counter(ieee, crate::security::ApsKeyType::TrustCenterLinkKey)
                        .unwrap_or(0)
                } else {
                    0
                };
                let sec_hdr = crate::security::ApsSecurityHeader {
                    security_control: crate::security::ApsSecurityHeader::APS_DEFAULT_EXT_NONCE,
                    frame_counter,
                    source_address: Some(src_ieee),
                    key_seq_number: None,
                };

                let aps_counter = self.next_aps_counter();
                let aps_header =
                    self.build_data_header(delivery_mode, req, aps_counter, true, false);

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

        // Resolve encryption key
        let dst_ieee = self.nwk.find_ieee_by_short(nwk_dst);
        let link_key = if let Some(ref ieee) = dst_ieee {
            if let Some(entry) = self.security.find_any_key(ieee) {
                entry.key
            } else {
                *self.security.default_tc_link_key()
            }
        } else {
            *self.security.default_tc_link_key()
        };
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
            let frame_counter = if let Some(ref ieee) = dst_ieee {
                self.security
                    .next_frame_counter(ieee, crate::security::ApsKeyType::TrustCenterLinkKey)
                    .unwrap_or(0)
            } else {
                0
            };
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
        log::info!(
            "[APS RX] process_incoming_aps_frame: {} bytes from 0x{:04X}",
            nwk_payload.len(),
            nwk_src.0,
        );
        if nwk_payload.len() >= 2 {
            log::info!(
                "[APS RX] FC=0x{:02X} (type={}, sec={})",
                nwk_payload[0],
                nwk_payload[0] & 0x03,
                (nwk_payload[0] >> 5) & 1,
            );
        }

        let (header, consumed) = ApsHeader::parse(nwk_payload)?;
        log::info!(
            "[APS RX] Parsed: type={:?}, sec={}, consumed={}",
            header.frame_control.frame_type,
            header.frame_control.security,
            consumed,
        );

        let aps_secured = header.frame_control.security;
        let after_header = &nwk_payload[consumed..];
        let mut used_decrypted_buf = false;

        // Phase 1: APS security decryption
        if aps_secured {
            log::info!("[APS RX] APS security enabled, parsing security header...");
            let parsed = crate::security::ApsSecurityHeader::parse(after_header);
            if parsed.is_none() {
                log::warn!(
                    "[APS RX] APS security header parse FAILED, after_header len={}",
                    after_header.len()
                );
                if after_header.len() >= 2 {
                    log::warn!(
                        "[APS RX] sec_ctrl=0x{:02X} next=0x{:02X}",
                        after_header[0],
                        after_header[1]
                    );
                }
                return None;
            }
            let (sec_hdr, sec_consumed) = parsed.unwrap();
            log::info!(
                "[APS RX] APS sec header: ctrl=0x{:02X}, fc={}, sec_consumed={}, ct_len={}",
                sec_hdr.security_control,
                sec_hdr.frame_counter,
                sec_consumed,
                after_header.len() - sec_consumed,
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
            log::info!("[APS RX] key_id={}", key_id);
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
                crate::security::derive_key_transport_key(self.security.default_tc_link_key())
            } else if key_id == crate::security::KEY_ID_KEY_LOAD {
                crate::security::derive_key_load_key(self.security.default_tc_link_key())
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
                log::info!(
                    "[APS RX] APS decrypt SUCCESS, plaintext {} bytes",
                    plaintext.len()
                );
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
                    log::info!(
                        "[APS RX] APS decrypt SUCCESS (raw AAD), plaintext {} bytes",
                        plaintext.len()
                    );
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
                // Try with patched AAD
                if let Some(plaintext) = self.security.decrypt(aad, ciphertext, &tc_key, &sec_hdr) {
                    log::info!("[APS RX] APS decrypt SUCCESS (raw TC key + patched AAD)");
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
                        log::info!("[APS RX] APS decrypt SUCCESS (raw TC key + raw AAD)");
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
                log::warn!(
                    "[APS] APS decryption failed from 0x{:04X} (key_id={}, ct_len={})",
                    nwk_src.0,
                    key_id,
                    ciphertext.len(),
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
                log::info!(
                    "[APS RX] Command ID=0x{:02X}, data_len={}",
                    cmd_id,
                    cmd_data.len()
                );
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

        let mut buf = [0u8; 80];
        let hdr_len = aps_header.serialize(&mut buf);
        let payload_len = cmd_payload.len();
        if hdr_len + payload_len > buf.len() {
            return Err(ApsStatus::IllegalRequest);
        }
        buf[hdr_len..hdr_len + payload_len].copy_from_slice(cmd_payload);
        let total = hdr_len + payload_len;

        self.nwk
            .nlde_data_request(dst, 1, &buf[..total], true, false)
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
        // APS command payload: cmd_id(1) + key_type(1)
        let cmd_payload = [
            crate::frames::ApsCommandId::RequestKey as u8, // 0x08
            0x04,                                          // key_type = TC Link Key
        ];
        self.send_aps_command(tc_addr, &cmd_payload, false).await
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
        // cmd_id(1) + src_ieee(8) + key_type(1) + hash(16)
        let mut payload = [0u8; 26];
        payload[0] = crate::frames::ApsCommandId::VerifyKey as u8;
        payload[1..9].copy_from_slice(src_ieee);
        payload[9] = key_type;
        payload[10..26].copy_from_slice(hash);
        self.send_aps_command(dst, &payload, true).await
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
                // Trust Center Link Key (spec §4.4.9.2.3)
                // Payload: key_type(1) + key(16) + dest_ieee(8) + src_ieee(8) = 33 bytes
                // src_ieee is the TC's IEEE address
                let tc_ieee = if data.len() >= 33 {
                    let mut addr = [0u8; 8];
                    addr.copy_from_slice(&data[25..33]);
                    addr
                } else {
                    // Short payload — resolve TC IEEE from NWK neighbor table
                    self.nwk().find_ieee_by_short(src).unwrap_or([0u8; 8])
                };
                log::info!(
                    "[APS] Transport-Key: TC Link Key from 0x{:04X}, TC IEEE={:02X?}",
                    src.0,
                    tc_ieee,
                );
                let entry = crate::security::ApsLinkKeyEntry {
                    partner_address: tc_ieee,
                    key,
                    key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                    outgoing_frame_counter: 0,
                    incoming_frame_counter: 0,
                };
                let _ = self.security_mut().add_key(entry);
            }
            0x04 => {
                // Application Link Key (spec §4.4.9.2.4)
                // Payload: key_type(1) + key(16) + partner_ieee(8) + initiator_flag(1)
                if data.len() < 25 {
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
                    incoming_frame_counter: 0,
                };
                let _ = self.security_mut().add_key(entry);
                log::info!(
                    "[APS] Application link key installed for partner {:02X?}",
                    partner_ieee
                );
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
