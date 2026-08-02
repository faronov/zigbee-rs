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
use crate::{
    ApsAddress, ApsAddressMode, ApsLayer, ApsStatus, ApsTxOptions, PendingApsAck, PendingApsTunnel,
};
use zigbee_crypto::ForwardAesProvider;
#[cfg(test)]
use zigbee_crypto::SoftwareAesProvider;
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
const BROADCAST_IEEE: IeeeAddress = [0xFF; 8];

#[derive(Debug, Clone, Copy)]
struct IncomingCommandSecurity {
    nwk_secured: bool,
    nwk_source: Option<IeeeAddress>,
    aps_secured: bool,
    aps_source: Option<IeeeAddress>,
    aps_key_identifier: Option<u8>,
}

impl IncomingCommandSecurity {
    fn nwk_authenticated(self) -> bool {
        self.nwk_secured && self.nwk_source.is_some()
    }
}

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

#[cfg(test)]
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

#[cfg(test)]
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
    // Software-AES wrapper (host tests) over the provider-keyed builder; the
    // embedded send path uses `_with` with the MAC.
    build_tc_secured_command_frame_with(
        &mut SoftwareAesProvider::new(),
        security,
        link_key,
        src_ieee,
        aps_counter,
        frame_counter,
        key_identifier,
        ack_request,
        command,
        frame,
    )
}

#[allow(clippy::too_many_arguments)]
fn build_tc_secured_command_frame_with<P: ForwardAesProvider>(
    provider: &mut P,
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
    let encrypted = security.encrypt_with(
        provider,
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

fn centralized_trust_center(address: IeeeAddress) -> Option<IeeeAddress> {
    nonzero_ieee(address).filter(|address| *address != BROADCAST_IEEE)
}

/// Attempt one APS CCM* decryption, copying the plaintext into `buf` on
/// success and returning whether the frame authenticated.
///
/// The incoming APS security path tries the same decryption up to four times
/// (patched vs. raw AAD × derived vs. raw Trust-Center link key). Each attempt
/// shares an identical tail — clamp the plaintext to the frame buffer, copy it
/// in and record its length. Folding that tail into one `#[inline(never)]`
/// helper emits the copy (and its `memcpy`) once per image instead of four
/// times. It is now generic over the [`ForwardAesProvider`] (so CCM* can use
/// a hardware AES backend) but is still a single function emitted once — a
/// firmware links exactly one MAC, hence one `P`. The replay check and the
/// single post-success frame-counter commit stay with the caller, so the R22
/// "commit only after MIC success, exactly once" ordering is untouched. A
/// hardware AES failure surfaces as a decrypt miss (returns `false`), never a
/// software fall-back.
#[inline(never)]
fn decrypt_into<P: ForwardAesProvider>(
    provider: &mut P,
    security: &crate::security::ApsSecurity,
    aad: &[u8],
    ciphertext: &[u8],
    key: &crate::security::AesKey,
    security_header: &crate::security::ApsSecurityHeader,
    buf: &mut ApsFrameBuffer,
) -> bool {
    let Some(plaintext) = security.decrypt_with(provider, aad, ciphertext, key, security_header)
    else {
        return false;
    };
    let pt_len = plaintext.len().min(buf.data.len());
    buf.data[..pt_len].copy_from_slice(&plaintext[..pt_len]);
    buf.len = pt_len;
    true
}

/// Result of the incoming APS security-decryption phase.
struct ApsDecryptOutcome {
    /// APS security source IEEE (auxiliary header, or the resolved fallback).
    aps_security_source: Option<IeeeAddress>,
    /// APS key identifier that selected the decryption key.
    aps_key_identifier: Option<u8>,
}

/// Verify and decrypt a secured incoming APS frame into `decrypted_buf`.
///
/// This is the synchronous APS security phase, lifted out of
/// [`ApsLayer::process_incoming_aps_frame`] so its auxiliary-header parse,
/// 64-byte AAD-patch buffer, key derivation and four decrypt attempts are
/// emitted once as an `#[inline(never)]` function. Keeping that scratch and
/// control flow out of the receive method shrinks the caller's stack frame
/// (the dominant cost of the `tloadr [pc]`-heavy TC32 codegen). It is generic
/// over the [`ForwardAesProvider`] so CCM* and AES-MMO key derivation can use
/// a hardware AES backend, but a firmware links exactly one MAC, so it is
/// still emitted once per image.
///
/// The two `NwkLayer<M>`-derived inputs (`nwk_src_ieee`, `nwk_has_active_key`)
/// are resolved by the caller so this helper never touches the generic NWK
/// layer; `provider` is the caller's MAC, handed in as a disjoint field
/// borrow. R22 replay ordering is preserved verbatim: the frame counter is
/// checked before any decryption and committed exactly once, only after a MIC
/// succeeds. Returns `None` to drop the frame (including on a hardware AES
/// failure — never a software fall-back).
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn aps_decrypt_incoming<P: ForwardAesProvider>(
    provider: &mut P,
    security: &mut crate::security::ApsSecurity,
    nwk_src_ieee: Option<IeeeAddress>,
    nwk_has_active_key: bool,
    tc_address: IeeeAddress,
    nwk_payload: &[u8],
    consumed: usize,
    decrypted_buf: &mut ApsFrameBuffer,
) -> Option<ApsDecryptOutcome> {
    let after_header = &nwk_payload[consumed..];
    aps_diag!("[APS] secured payload has {} bytes", after_header.len());
    #[allow(clippy::question_mark)]
    let Some((mut sec_hdr, sec_consumed)) = crate::security::ApsSecurityHeader::parse(after_header)
    else {
        aps_diag!(
            "[APS] security header parse failed for {} bytes",
            after_header.len()
        );
        return None;
    };
    if sec_hdr.source_address.is_none() {
        sec_hdr.source_address = nwk_src_ieee.or_else(|| nonzero_ieee(tc_address));
    }
    let aps_security_source = sec_hdr.source_address;
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

    let key_id = crate::security::ApsSecurityHeader::key_identifier(sec_hdr.security_control);
    let aps_key_identifier = Some(key_id);
    aps_diag!(
        "[APS] key_id={} aad_len={} ct_len={} src_ieee={}",
        key_id,
        aad_len,
        ciphertext.len(),
        sec_hdr.source_address.is_some() as u8,
    );

    let installed_link_key = sec_hdr
        .source_address
        .as_ref()
        .and_then(|address| security.find_any_key(address))
        .map(|entry| entry.key);
    let uses_default_link_key = installed_link_key.is_none();
    let base_link_key = installed_link_key.unwrap_or(*security.default_tc_link_key());
    let key = if key_id == crate::security::KEY_ID_DATA_KEY {
        base_link_key
    } else if key_id == crate::security::KEY_ID_KEY_TRANSPORT {
        crate::security::derive_key_transport_key_with(provider, &base_link_key)?
    } else if key_id == crate::security::KEY_ID_KEY_LOAD {
        crate::security::derive_key_load_key_with(provider, &base_link_key)?
    } else {
        log::warn!("[APS] Unsupported key_id={} in APS security", key_id);
        return None;
    };

    let replay_key_type = crate::security::ApsKeyType::TrustCenterLinkKey;
    if let Some(addr) = &sec_hdr.source_address
        && !security.check_frame_counter(addr, replay_key_type, sec_hdr.frame_counter)
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
    if decrypt_into(
        provider,
        security,
        aad,
        ciphertext,
        &key,
        &sec_hdr,
        decrypted_buf,
    ) {
        aps_diag!("[APS] decrypt succeeded with patched AAD");
        decrypt_ok = true;
    }

    // Fallback: try with un-patched AAD (original OTA security level)
    if !decrypt_ok {
        let aad_raw = &nwk_payload[..aad_end.min(nwk_payload.len())];
        if decrypt_into(
            provider,
            security,
            aad_raw,
            ciphertext,
            &key,
            &sec_hdr,
            decrypted_buf,
        ) {
            aps_diag!("[APS] decrypt succeeded with raw AAD");
            decrypt_ok = true;
        }
    }

    // Fallback for key-transport: try raw TC link key (some impls don't derive)
    if !decrypt_ok
        && key_id == crate::security::KEY_ID_KEY_TRANSPORT
        && uses_default_link_key
        && !nwk_has_active_key
    {
        let tc_key = *security.default_tc_link_key();
        if decrypt_into(
            provider,
            security,
            aad,
            ciphertext,
            &tc_key,
            &sec_hdr,
            decrypted_buf,
        ) {
            aps_diag!("[APS] key-transport decrypt succeeded with raw TC key");
            decrypt_ok = true;
        }
        // Try with un-patched AAD
        if !decrypt_ok {
            let aad_raw = &nwk_payload[..aad_end.min(nwk_payload.len())];
            if decrypt_into(
                provider,
                security,
                aad_raw,
                ciphertext,
                &tc_key,
                &sec_hdr,
                decrypted_buf,
            ) {
                aps_diag!("[APS] key-transport decrypt succeeded with raw TC key and raw AAD");
                decrypt_ok = true;
            }
        }
    }

    if decrypt_ok {
        if let Some(addr) = &sec_hdr.source_address {
            security.commit_frame_counter(addr, replay_key_type, sec_hdr.frame_counter);
        }
    } else {
        aps_diag!(
            "[APS] decrypt ALL FAILED key_id={} ct_len={}",
            key_id,
            ciphertext.len()
        );
        return None;
    }

    Some(ApsDecryptOutcome {
        aps_security_source,
        aps_key_identifier,
    })
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

/// NWK authentication metadata accompanying an APS payload.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct IncomingNwkSecurity {
    secured: bool,
    source: Option<IeeeAddress>,
}

impl IncomingNwkSecurity {
    pub const fn new(secured: bool, source: Option<IeeeAddress>) -> Self {
        Self { secured, source }
    }
}

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

            if let Some(enc) =
                self.security
                    .encrypt_with(self.nwk.mac_mut(), aad, req.payload, &key, &sec_hdr)
            {
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

            if let Some(enc) =
                self.security
                    .encrypt_with(self.nwk.mac_mut(), aad, chunk, &link_key, &sec_hdr)
            {
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
        nwk_security: IncomingNwkSecurity,
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
        let mut aps_key_identifier = None;

        // Phase 1: APS security decryption.
        //
        // The decrypt, replay check and key derivation are `MacDriver`-
        // independent, so they run in the non-generic `aps_decrypt_incoming`
        // helper (see its doc comment). The two `NwkLayer<M>`-derived inputs are
        // resolved here and handed in by value so the helper never sees `M`.
        if aps_secured {
            let nwk_src_ieee = self.nwk.find_ieee_by_short(nwk_src);
            let nwk_has_active_key = self.nwk.security().active_key().is_some();
            let tc_address = self.aib.aps_trust_center_address;
            let outcome = aps_decrypt_incoming(
                self.nwk.mac_mut(),
                &mut self.security,
                nwk_src_ieee,
                nwk_has_active_key,
                tc_address,
                nwk_payload,
                consumed,
                decrypted_buf,
            )?;
            used_decrypted_buf = true;
            aps_security_source = outcome.aps_security_source;
            aps_key_identifier = outcome.aps_key_identifier;
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
                let command_security = IncomingCommandSecurity {
                    nwk_secured: nwk_security.secured,
                    nwk_source: nwk_security.source,
                    aps_secured,
                    aps_source: aps_security_source,
                    aps_key_identifier,
                };
                aps_diag!("[APS] command ID={:02X} data={}", cmd_id, cmd_data.len());
                match crate::frames::ApsCommandId::from_u8(cmd_id) {
                    Some(crate::frames::ApsCommandId::TransportKey) => {
                        self.handle_transport_key(cmd_data, nwk_src, command_security);
                    }
                    Some(crate::frames::ApsCommandId::SwitchKey) => {
                        self.handle_switch_key(cmd_data, nwk_src, command_security);
                    }
                    Some(crate::frames::ApsCommandId::Tunnel) => {
                        self.handle_tunnel(cmd_data, nwk_src, nwk_security.secured, aps_secured);
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
                        self.security_handshake_stats
                            .last_confirm_key_key_identifier = aps_key_identifier.unwrap_or(0xFF);
                        self.security_handshake_stats.last_confirm_key_aps_secured = aps_secured;
                        self.security_handshake_stats.last_confirm_key_nwk_secured =
                            nwk_security.secured;
                        let valid = if let Some(command) = parse_confirm_key_command(cmd_data) {
                            self.security_handshake_stats.last_confirm_key_status = command.status;
                            self.security_handshake_stats.last_confirm_key_type = command.key_type;
                            self.security_handshake_stats.last_confirm_key_destination =
                                command.destination;
                            command.status == 0x00
                                && command.key_type == WIRE_KEY_TYPE_TC_LINK
                                && aps_secured
                                && aps_key_identifier == Some(crate::security::KEY_ID_DATA_KEY)
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
            security_status: aps_secured || nwk_security.secured,
            lqi,
        })
    }

    fn authenticated_trust_center_source(
        &self,
        src: ShortAddress,
        security: IncomingCommandSecurity,
    ) -> Option<IeeeAddress> {
        if security.aps_secured {
            let source = security.aps_source?;
            if let Some(configured) = centralized_trust_center(self.aib.aps_trust_center_address)
                && configured != source
            {
                return None;
            }
            return Some(source);
        }
        if src == ShortAddress::COORDINATOR && security.nwk_authenticated() {
            return centralized_trust_center(self.aib.aps_trust_center_address);
        }
        None
    }

    /// Handle an incoming APS Switch-Key command.
    ///
    /// Activates the network key with the specified sequence number.
    fn handle_switch_key(
        &mut self,
        data: &[u8],
        src: ShortAddress,
        security: IncomingCommandSecurity,
    ) {
        if data.is_empty() {
            log::warn!("[APS] Switch-Key too short");
            return;
        }
        if !security.nwk_authenticated()
            || self
                .authenticated_trust_center_source(src, security)
                .is_none()
            || (security.aps_secured
                && security.aps_key_identifier != Some(crate::security::KEY_ID_DATA_KEY))
        {
            log::warn!("[APS] rejecting unauthenticated Switch-Key command");
            return;
        }
        let key_seq = data[0];
        if !self.nwk_mut().security_mut().activate_network_key(key_seq) {
            log::warn!(
                "[APS] Switch-Key references unknown network key sequence {}",
                key_seq
            );
            return;
        }
        log::info!(
            "[APS] Switch-Key: activate key seq={} from 0x{:04X}",
            key_seq,
            src.0
        );
        self.nwk_mut().nib_mut().active_key_seq_number = key_seq;
    }

    async fn send_unsecured_aps_command(
        &mut self,
        dst: ShortAddress,
        cmd_payload: &[u8],
    ) -> Result<(), ApsStatus> {
        let aps_header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Command as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false,
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: None,
            group_address: None,
            cluster_id: None,
            profile_id: None,
            src_endpoint: None,
            aps_counter: self.next_aps_counter(),
            extended_header: None,
        };

        let mut frame = [0u8; 80];
        let header_len = aps_header.serialize(&mut frame);
        if header_len + cmd_payload.len() > frame.len() {
            return Err(ApsStatus::IllegalRequest);
        }
        frame[header_len..header_len + cmd_payload.len()].copy_from_slice(cmd_payload);
        let total = header_len + cmd_payload.len();
        let radius = self.nwk.nib().max_depth.saturating_mul(2);
        self.nwk
            .nlde_data_request(dst, radius, &frame[..total], true, false)
            .await
            .map(|_| ())
            .map_err(nwk_status_to_aps)
    }

    /// Notify the centralized Trust Center that a child joined or rejoined.
    ///
    /// A unique Trust Center link key requires APS encryption. With the
    /// preconfigured global key, Zigbee interoperability rules permit sending
    /// both the APS-encrypted and NWK-only forms.
    pub async fn send_update_device(
        &mut self,
        device_address: &IeeeAddress,
        device_short_address: ShortAddress,
        status: crate::apsme::ApsUpdateDeviceStatus,
    ) -> Result<(), ApsStatus> {
        let trust_center = self.aib.aps_trust_center_address;
        if trust_center == [0u8; 8] || trust_center == [0xFFu8; 8] {
            return Err(ApsStatus::InvalidParameter);
        }
        if !self.nwk.nib().security_enabled {
            return Err(ApsStatus::SecurityFail);
        }

        let mut command = [0u8; 12];
        command[0] = crate::frames::ApsCommandId::UpdateDevice as u8;
        command[1..9].copy_from_slice(device_address);
        command[9..11].copy_from_slice(&device_short_address.0.to_le_bytes());
        command[11] = status as u8;

        let local_ieee = self.nwk.nib().ieee_address;
        let default_key = *self.security.default_tc_link_key();
        let has_unique_key = self
            .security
            .find_key(
                &trust_center,
                crate::security::ApsKeyType::TrustCenterLinkKey,
            )
            .is_some_and(|entry| entry.key != default_key);
        let (link_key, frame_counter) = self
            .next_current_tc_link_key_material()
            .ok_or(ApsStatus::SecurityFail)?;
        let encrypted = self
            .send_link_key_secured_command(
                ShortAddress::COORDINATOR,
                &local_ieee,
                &link_key,
                frame_counter,
                crate::security::KEY_ID_DATA_KEY,
                false,
                true,
                &command,
            )
            .await;

        if has_unique_key {
            return encrypted;
        }

        let nwk_only = self
            .send_unsecured_aps_command(ShortAddress::COORDINATOR, &command)
            .await;
        match (encrypted, nwk_only) {
            (Ok(()), Ok(())) => Ok(()),
            (Ok(()), Err(error)) => {
                log::warn!("[APS] NWK-only Update-Device copy failed: {error:?}");
                Ok(())
            }
            (Err(error), Ok(())) => {
                log::warn!("[APS] APS-secured Update-Device copy failed: {error:?}");
                Ok(())
            }
            (Err(_), Err(error)) => Err(error),
        }
    }

    /// Return and clear the APS Tunnel command captured during receive.
    pub fn take_pending_tunnel(&mut self) -> Option<PendingApsTunnel> {
        self.pending_tunnel.take()
    }

    /// Forward one tunneled APS command to its joining child.
    ///
    /// The tunneled APDU remains APS-encrypted end to end. Only the outer
    /// Tunnel command used the active network key; the child-facing NWK frame
    /// must be unsecured because the child does not know that key yet.
    pub async fn forward_tunnel(&mut self, tunnel: &PendingApsTunnel) -> Result<(), ApsStatus> {
        let destination = self
            .nwk
            .known_child_by_ieee(&tunnel.destination)
            .ok_or(ApsStatus::NoShortAddress)?;
        let radius = self.nwk.nib().max_depth.saturating_mul(2);
        self.nwk
            .nlde_data_request(destination, radius, tunnel.frame(), false, false)
            .await
            .map(|_| ())
            .map_err(nwk_status_to_aps)
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
        let (key, frame_counter) = self
            .next_current_tc_link_key_material()
            .ok_or(ApsStatus::SecurityFail)?;
        self.send_link_key_secured_command(
            tc_addr,
            &local_ieee,
            &key,
            frame_counter,
            crate::security::KEY_ID_DATA_KEY,
            false,
            true,
            &command,
        )
        .await
    }

    /// Build and send an APSME-TRANSPORT-KEY command frame.
    pub async fn send_transport_key(
        &mut self,
        dst: ShortAddress,
        dst_ieee: &IeeeAddress,
        key_type: u8,
        key: &[u8; 16],
        key_seq_number: u8,
        src_ieee: &IeeeAddress,
    ) -> Result<(), ApsStatus> {
        log::info!(
            "[APS] Sending Transport-Key to 0x{:04X} type={key_type}",
            dst.0
        );
        let mut payload = [0u8; 35];
        payload[0] = crate::frames::ApsCommandId::TransportKey as u8;
        payload[1] = key_type;
        payload[2..18].copy_from_slice(key);
        let payload_len = match key_type {
            0x01 => {
                payload[18] = key_seq_number;
                payload[19..27].copy_from_slice(dst_ieee);
                payload[27..35].copy_from_slice(src_ieee);
                35
            }
            0x03 => {
                payload[18..26].copy_from_slice(dst_ieee);
                payload[26] = 0;
                27
            }
            0x04 => {
                payload[18..26].copy_from_slice(dst_ieee);
                payload[26..34].copy_from_slice(src_ieee);
                34
            }
            _ => return Err(ApsStatus::InvalidParameter),
        };
        if key_type == 0x01 && *dst_ieee == BROADCAST_IEEE {
            return self
                .send_unsecured_aps_command(ShortAddress::BROADCAST, &payload[..payload_len])
                .await;
        }

        let (base_key, frame_counter) = self
            .next_link_key_material_for(dst_ieee)
            .ok_or(ApsStatus::SecurityFail)?;
        let (security_key, key_identifier) = if key_type == 0x01 {
            (
                crate::security::derive_key_transport_key_with(self.nwk.mac_mut(), &base_key)
                    .ok_or(ApsStatus::SecurityFail)?,
                crate::security::KEY_ID_KEY_TRANSPORT,
            )
        } else {
            (
                crate::security::derive_key_load_key_with(self.nwk.mac_mut(), &base_key)
                    .ok_or(ApsStatus::SecurityFail)?,
                crate::security::KEY_ID_KEY_LOAD,
            )
        };
        let joining_child = key_type == 0x01
            && self.nwk.known_child_by_ieee(dst_ieee).is_some()
            && !self.nwk.child_is_authorized(dst_ieee);
        self.send_link_key_secured_command(
            dst,
            src_ieee,
            &security_key,
            frame_counter,
            key_identifier,
            true,
            !joining_child,
            &payload[..payload_len],
        )
        .await
    }

    /// Build and send an APSME-SWITCH-KEY command frame.
    pub async fn send_switch_key(
        &mut self,
        dst: ShortAddress,
        dst_ieee: &IeeeAddress,
        key_seq_number: u8,
    ) -> Result<(), ApsStatus> {
        log::info!(
            "[APS] Sending Switch-Key to 0x{:04X} seq={key_seq_number}",
            dst.0
        );
        // cmd_id(1) + key_seq(1)
        let payload = [crate::frames::ApsCommandId::SwitchKey as u8, key_seq_number];
        if *dst_ieee == BROADCAST_IEEE {
            return self
                .send_unsecured_aps_command(ShortAddress::BROADCAST, &payload)
                .await;
        }
        let local_ieee = self.nwk.nib().ieee_address;
        let (link_key, frame_counter) = self
            .next_link_key_material_for(dst_ieee)
            .ok_or(ApsStatus::SecurityFail)?;
        self.send_link_key_secured_command(
            dst,
            &local_ieee,
            &link_key,
            frame_counter,
            crate::security::KEY_ID_DATA_KEY,
            true,
            true,
            &payload,
        )
        .await
    }

    /// Build and send an APSME-VERIFY-KEY command frame.
    pub async fn send_verify_key(
        &mut self,
        dst: ShortAddress,
        src_ieee: &IeeeAddress,
        key_type: u8,
        hash: &[u8; 16],
    ) -> Result<(), ApsStatus> {
        let (key, frame_counter) = self
            .next_current_tc_link_key_material()
            .ok_or(ApsStatus::SecurityFail)?;
        self.send_verify_key_with_material(dst, src_ieee, key_type, hash, &key, frame_counter)
            .await
    }

    async fn send_verify_key_with_material(
        &mut self,
        dst: ShortAddress,
        src_ieee: &IeeeAddress,
        key_type: u8,
        hash: &[u8; 16],
        key: &crate::security::AesKey,
        frame_counter: u32,
    ) -> Result<(), ApsStatus> {
        log::info!(
            "[APS] Sending Verify-Key to 0x{:04X} type={key_type}",
            dst.0
        );
        let payload = build_verify_key_command(src_ieee, key_type, hash);
        self.security_handshake_stats.last_verify_key_frame_counter = frame_counter;
        self.send_link_key_secured_command(
            dst,
            src_ieee,
            key,
            frame_counter,
            crate::security::KEY_ID_DATA_KEY,
            true,
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
    /// key. BDB calls this only after Transport-Key installed that exact key;
    /// falling back here would hash and encrypt with the wrong security state.
    pub async fn send_tc_verify_key(&mut self, tc_addr: ShortAddress) -> Result<(), ApsStatus> {
        let local_ieee = self.nwk.nib().ieee_address;
        let tc_ieee =
            nonzero_ieee(self.aib.aps_trust_center_address).ok_or(ApsStatus::SecurityFail)?;
        let tc_key = self
            .security
            .find_key(&tc_ieee, crate::security::ApsKeyType::TrustCenterLinkKey)
            .map(|entry| entry.key)
            .ok_or(ApsStatus::SecurityFail)?;
        let frame_counter = self
            .security
            .next_frame_counter(&tc_ieee, crate::security::ApsKeyType::TrustCenterLinkKey)
            .ok_or(ApsStatus::SecurityFail)?;
        let hash = crate::security::derive_verify_key_hash_with(self.nwk.mac_mut(), &tc_key)
            .ok_or(ApsStatus::SecurityFail)?;
        self.security_handshake_stats.last_verify_key_trust_center = tc_ieee;
        self.send_verify_key_with_material(
            tc_addr,
            &local_ieee,
            WIRE_KEY_TYPE_TC_LINK,
            &hash,
            &tc_key,
            frame_counter,
        )
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

    fn next_link_key_material_for(
        &mut self,
        partner: &IeeeAddress,
    ) -> Option<(crate::security::AesKey, u32)> {
        if let Some(key) = self
            .security
            .find_key(partner, crate::security::ApsKeyType::TrustCenterLinkKey)
            .map(|entry| entry.key)
        {
            let frame_counter = self
                .security
                .next_frame_counter(partner, crate::security::ApsKeyType::TrustCenterLinkKey)?;
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
        nwk_security: bool,
        command: &[u8],
    ) -> Result<(), ApsStatus> {
        let aps_counter = self.next_aps_counter();

        let mut frame = [0u8; 80];
        let total = build_tc_secured_command_frame_with(
            self.nwk.mac_mut(),
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
            .nlde_data_request(dst, radius, &frame[..total], nwk_security, false)
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
    fn handle_transport_key(
        &mut self,
        data: &[u8],
        src: ShortAddress,
        security: IncomingCommandSecurity,
    ) {
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
        let Some(authenticated_tc) = self.authenticated_trust_center_source(src, security) else {
            log::warn!("[APS] rejecting unauthenticated Transport-Key command");
            return;
        };
        if self.nwk.security().active_key().is_some() && !security.nwk_authenticated() {
            log::warn!("[APS] rejecting Transport-Key without NWK authentication");
            return;
        }

        match key_type {
            0x01 => {
                // Standard Network Key
                if (security.aps_secured
                    && security.aps_key_identifier != Some(crate::security::KEY_ID_KEY_TRANSPORT))
                    || (!security.aps_secured && !security.nwk_authenticated())
                {
                    log::warn!("[APS] Transport-Key: invalid network-key protection");
                    return;
                }
                if data.len() < 34 {
                    log::warn!(
                        "[APS] Transport-Key: network-key descriptor too short ({} bytes)",
                        data.len()
                    );
                    return;
                }
                let key_seq = data[17];
                let mut destination_ieee = [0u8; 8];
                destination_ieee.copy_from_slice(&data[18..26]);
                let broadcast_update = destination_ieee == BROADCAST_IEEE;
                if destination_ieee != self.nwk.nib().ieee_address && !broadcast_update {
                    log::warn!("[APS] Transport-Key: network key is for another device");
                    return;
                }
                if broadcast_update
                    && (!security.nwk_authenticated() || self.nwk.security().active_key().is_none())
                {
                    log::warn!(
                        "[APS] Transport-Key: broadcast network-key update lacks NWK authentication"
                    );
                    return;
                }
                let mut tc_ieee = [0u8; 8];
                tc_ieee.copy_from_slice(&data[26..34]);
                let Some(tc_ieee) = centralized_trust_center(tc_ieee) else {
                    log::warn!("[APS] Transport-Key: network key has no Trust Center source");
                    return;
                };
                if tc_ieee != authenticated_tc {
                    log::warn!("[APS] Transport-Key: Trust Center source mismatch");
                    return;
                }
                self.aib_mut().aps_trust_center_address = tc_ieee;
                aps_diag!("[APS] Installing NWK key seq={}", key_seq);
                let initial_key = self.nwk.security().active_key().is_none();
                if initial_key {
                    self.nwk_mut().security_mut().set_network_key(key, key_seq);
                    let nib = self.nwk_mut().nib_mut();
                    nib.active_key_seq_number = key_seq;
                    nib.security_enabled = true;
                    aps_diag!("[APS] NWK key installed");
                } else if !self
                    .nwk_mut()
                    .security_mut()
                    .stage_network_key(key, key_seq)
                {
                    log::warn!(
                        "[APS] Transport-Key: sequence {} conflicts with the active key",
                        key_seq
                    );
                } else {
                    aps_diag!("[APS] NWK key installed");
                }
            }
            0x03 => {
                // Application Link Key
                // Payload: key_type(1) + key(16) + partner_ieee(8) + initiator_flag(1)
                if !security.aps_secured
                    || security.aps_key_identifier != Some(crate::security::KEY_ID_KEY_LOAD)
                    || centralized_trust_center(self.aib.aps_trust_center_address)
                        != Some(authenticated_tc)
                {
                    log::warn!("[APS] Transport-Key: invalid application-key protection");
                    return;
                }
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
                if !security.aps_secured
                    || security.aps_key_identifier != Some(crate::security::KEY_ID_KEY_LOAD)
                {
                    log::warn!("[APS] Transport-Key: invalid Trust Center key protection");
                    return;
                }
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
                let Some(tc_ieee) = centralized_trust_center(tc_ieee) else {
                    log::warn!("[APS] Transport-Key: TC Link Key has no source IEEE");
                    return;
                };
                if tc_ieee != authenticated_tc {
                    log::warn!("[APS] Transport-Key: TC source address mismatch");
                    return;
                }
                log::info!(
                    "[APS] Transport-Key: TC Link Key from 0x{:04X}, TC IEEE={:02X?}",
                    src.0,
                    tc_ieee,
                );
                self.aib_mut().aps_trust_center_address = tc_ieee;
                // ZHA's Ember Trust Center commonly returns its current TCLK
                // rather than generating a new key. Request-Key has already
                // advanced the TC's incoming APS counter, so restarting the
                // replacement entry at zero makes Verify-Key look like a
                // replay. Carry the per-peer counter forward across rekeys.
                let nwk_counter = self.nwk.nib().outgoing_frame_counter;
                let prior = self
                    .security
                    .find_key(&tc_ieee, crate::security::ApsKeyType::TrustCenterLinkKey)
                    .map(|entry| {
                        (
                            entry.key,
                            entry.outgoing_frame_counter,
                            entry.incoming_frame_counter,
                            entry.incoming_frame_counter_valid,
                        )
                    });
                let outgoing_frame_counter = prior
                    .map(|(_, counter, _, _)| counter)
                    .unwrap_or(0)
                    .max(nwk_counter);
                let (incoming_frame_counter, incoming_frame_counter_valid) = match prior {
                    Some((prior_key, _, counter, valid)) if prior_key == key => (counter, valid),
                    _ => (0, false),
                };
                let entry = crate::security::ApsLinkKeyEntry {
                    partner_address: tc_ieee,
                    key,
                    key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                    outgoing_frame_counter,
                    outgoing_frame_counter_limit: u32::MAX,
                    incoming_frame_counter,
                    incoming_frame_counter_valid,
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

    fn handle_tunnel(
        &mut self,
        data: &[u8],
        src: ShortAddress,
        nwk_security: bool,
        aps_secured: bool,
    ) {
        if !nwk_security || aps_secured || src != ShortAddress::COORDINATOR {
            log::warn!(
                "[APS] Rejecting Tunnel from 0x{:04X}: NWK security={} APS security={}",
                src.0,
                nwk_security,
                aps_secured
            );
            return;
        }
        if data.len() < 8 {
            log::warn!("[APS] Tunnel payload too short: {}", data.len());
            return;
        }

        let mut destination = [0u8; 8];
        destination.copy_from_slice(&data[..8]);
        if self.nwk.known_child_by_ieee(&destination).is_none() {
            log::warn!("[APS] Tunnel destination is not a child of this device");
            return;
        }

        let tunneled_frame = &data[8..];
        let Some((header, header_len)) = ApsHeader::parse(tunneled_frame) else {
            log::warn!("[APS] Tunnel contains a malformed APS frame");
            return;
        };
        if ApsFrameType::from_u8(header.frame_control.frame_type) != Some(ApsFrameType::Command)
            || !header.frame_control.security
        {
            log::warn!("[APS] Tunnel does not contain an APS-secured command");
            return;
        }
        let Some((security_header, security_header_len)) =
            crate::security::ApsSecurityHeader::parse(&tunneled_frame[header_len..])
        else {
            log::warn!("[APS] Tunnel contains a malformed APS security header");
            return;
        };
        let key_identifier =
            crate::security::ApsSecurityHeader::key_identifier(security_header.security_control);
        if key_identifier != crate::security::KEY_ID_KEY_TRANSPORT {
            log::warn!(
                "[APS] Tunnel contains embedded key identifier {}",
                key_identifier
            );
            return;
        }
        if security_header.source_address != Some(self.aib.aps_trust_center_address)
            || header_len + security_header_len + 5 > tunneled_frame.len()
        {
            log::warn!("[APS] Tunnel command has an invalid source or encrypted payload");
            return;
        }
        if self.pending_tunnel.is_some() {
            log::warn!("[APS] Tunnel slot already occupied");
            return;
        }
        let Ok(frame) = heapless::Vec::from_slice(tunneled_frame) else {
            log::warn!("[APS] Tunneled APS frame is too large");
            return;
        };
        self.pending_tunnel = Some(PendingApsTunnel { destination, frame });
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
    #[cfg(feature = "router")]
    use core::future::Future;
    #[cfg(feature = "router")]
    use core::task::{Context, Poll, Waker};
    #[cfg(feature = "router")]
    use std::sync::Arc;
    #[cfg(feature = "router")]
    use std::task::Wake;
    #[cfg(feature = "router")]
    use zigbee_mac::CapabilityInfo;
    #[cfg(feature = "router")]
    use zigbee_mac::mock::{MockMac, TxRecord};
    #[cfg(feature = "router")]
    use zigbee_nwk::{DeviceType, NwkLayer};
    #[cfg(feature = "router")]
    use zigbee_types::{MacAddress, PanId};

    #[cfg(feature = "router")]
    const TEST_PAN: PanId = PanId(0x1234);
    #[cfg(feature = "router")]
    const TEST_NETWORK_KEY: [u8; 16] = [0x31; 16];
    #[cfg(feature = "router")]
    const LOCAL_IEEE: IeeeAddress = [0x10; 8];
    #[cfg(feature = "router")]
    const TC_IEEE: IeeeAddress = [0x20; 8];
    #[cfg(feature = "router")]
    const CHILD_IEEE: IeeeAddress = [0x30; 8];
    #[cfg(feature = "router")]
    const LOCAL_SHORT: ShortAddress = ShortAddress(0x1111);
    #[cfg(feature = "router")]
    const CHILD_SHORT: ShortAddress = ShortAddress(0x2222);

    #[cfg(feature = "router")]
    struct NoopWake;

    #[cfg(feature = "router")]
    impl Wake for NoopWake {
        fn wake(self: Arc<Self>) {}
    }

    #[cfg(feature = "router")]
    fn block_on<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(NoopWake));
        let mut context = Context::from_waker(&waker);
        let mut future = std::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    #[cfg(feature = "router")]
    fn aps_node(device_type: DeviceType, address: ShortAddress) -> ApsLayer<MockMac> {
        let mut nwk = NwkLayer::new(MockMac::new(LOCAL_IEEE), device_type);
        nwk.set_joined(true);
        {
            let nib = nwk.nib_mut();
            nib.pan_id = TEST_PAN;
            nib.network_address = address;
            nib.parent_address = ShortAddress::COORDINATOR;
            nib.ieee_address = LOCAL_IEEE;
            nib.security_enabled = true;
            nib.active_key_seq_number = 0;
        }
        nwk.security_mut().set_network_key(TEST_NETWORK_KEY, 0);
        ApsLayer::new(nwk)
    }

    #[cfg(feature = "router")]
    fn unsecured_command_frame(command: &[u8], aps_counter: u8) -> heapless::Vec<u8, 128> {
        let header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Command as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false,
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
        let mut frame = [0u8; 128];
        let header_len = header.serialize(&mut frame);
        frame[header_len..header_len + command.len()].copy_from_slice(command);
        heapless::Vec::from_slice(&frame[..header_len + command.len()]).unwrap()
    }

    #[cfg(feature = "router")]
    fn network_key_command(key: [u8; 16], sequence: u8, destination: IeeeAddress) -> [u8; 35] {
        let mut command = [0u8; 35];
        command[0] = crate::frames::ApsCommandId::TransportKey as u8;
        command[1] = 0x01;
        command[2..18].copy_from_slice(&key);
        command[18] = sequence;
        command[19..27].copy_from_slice(&destination);
        command[27..35].copy_from_slice(&TC_IEEE);
        command
    }

    #[cfg(feature = "router")]
    fn nwk_payload(record: &TxRecord) -> heapless::Vec<u8, 128> {
        let bytes = record.payload.as_slice();
        let (header, header_len) =
            zigbee_nwk::frames::NwkHeader::parse(bytes).expect("NWK frame parses");
        if !header.frame_control.security {
            return heapless::Vec::from_slice(&bytes[header_len..]).unwrap();
        }
        let (security_header, security_header_len) =
            zigbee_nwk::security::NwkSecurityHeader::parse(&bytes[header_len..])
                .expect("NWK security header parses");
        let aad_len = header_len + security_header_len;
        let mut aad = [0u8; 64];
        aad[..aad_len].copy_from_slice(&bytes[..aad_len]);
        aad[header_len] = (aad[header_len] & !0x07) | 0x05;
        zigbee_nwk::security::NwkSecurity::new()
            .decrypt(
                &aad[..aad_len],
                &bytes[aad_len..],
                &TEST_NETWORK_KEY,
                &security_header,
            )
            .expect("NWK payload decrypts")
    }

    #[cfg(feature = "router")]
    fn aps_command(frame: &[u8], key: &crate::security::AesKey) -> heapless::Vec<u8, 128> {
        let (header, header_len) = ApsHeader::parse(frame).expect("APS header parses");
        assert_eq!(
            ApsFrameType::from_u8(header.frame_control.frame_type),
            Some(ApsFrameType::Command)
        );
        if !header.frame_control.security {
            return heapless::Vec::from_slice(&frame[header_len..]).unwrap();
        }
        let (security_header, security_header_len) =
            crate::security::ApsSecurityHeader::parse(&frame[header_len..])
                .expect("APS security header parses");
        let aad_len = header_len + security_header_len;
        let mut aad = [0u8; 32];
        aad[..aad_len].copy_from_slice(&frame[..aad_len]);
        aad[header_len] |= crate::security::SEC_LEVEL_ENC_MIC_32;
        crate::security::ApsSecurity::new()
            .decrypt(&aad[..aad_len], &frame[aad_len..], key, &security_header)
            .expect("APS command decrypts")
    }

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

    #[test]
    #[cfg(feature = "router")]
    fn trust_center_key_install_carries_the_outgoing_counter_forward() {
        let mut aps = aps_node(DeviceType::Router, LOCAL_SHORT);
        aps.nwk_mut().nib_mut().outgoing_frame_counter = 0x1234;

        let unique_key = [0xA5; 16];
        let mut transport_key = [0u8; 33];
        transport_key[0] = WIRE_KEY_TYPE_TC_LINK;
        transport_key[1..17].copy_from_slice(&unique_key);
        transport_key[17..25].copy_from_slice(&LOCAL_IEEE);
        transport_key[25..33].copy_from_slice(&TC_IEEE);

        aps.handle_transport_key(
            &transport_key,
            ShortAddress::COORDINATOR,
            IncomingCommandSecurity {
                nwk_secured: true,
                nwk_source: Some(TC_IEEE),
                aps_secured: true,
                aps_source: Some(TC_IEEE),
                aps_key_identifier: Some(crate::security::KEY_ID_KEY_LOAD),
            },
        );

        let entry = aps
            .security()
            .find_key(&TC_IEEE, crate::security::ApsKeyType::TrustCenterLinkKey)
            .unwrap();
        assert_eq!(entry.key, unique_key);
        assert_eq!(entry.outgoing_frame_counter, 0x1234);
    }

    #[test]
    #[cfg(feature = "router")]
    fn trust_center_key_reinstall_never_regresses_existing_counters() {
        let mut aps = aps_node(DeviceType::Router, LOCAL_SHORT);
        aps.aib_mut().aps_trust_center_address = TC_IEEE;
        aps.nwk_mut().nib_mut().outgoing_frame_counter = 0x1200;
        let key = [0xA5; 16];
        aps.security_mut()
            .add_key(crate::security::ApsLinkKeyEntry {
                partner_address: TC_IEEE,
                key,
                key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0x1400,
                outgoing_frame_counter_limit: 0x1800,
                incoming_frame_counter: 0x2200,
                incoming_frame_counter_valid: true,
            })
            .unwrap();

        let mut transport_key = [0u8; 33];
        transport_key[0] = WIRE_KEY_TYPE_TC_LINK;
        transport_key[1..17].copy_from_slice(&key);
        transport_key[17..25].copy_from_slice(&LOCAL_IEEE);
        transport_key[25..33].copy_from_slice(&TC_IEEE);
        aps.handle_transport_key(
            &transport_key,
            ShortAddress::COORDINATOR,
            IncomingCommandSecurity {
                nwk_secured: true,
                nwk_source: Some(TC_IEEE),
                aps_secured: true,
                aps_source: Some(TC_IEEE),
                aps_key_identifier: Some(crate::security::KEY_ID_KEY_LOAD),
            },
        );

        let entry = aps
            .security()
            .find_key(&TC_IEEE, crate::security::ApsKeyType::TrustCenterLinkKey)
            .unwrap();
        assert_eq!(entry.outgoing_frame_counter, 0x1400);
        assert_eq!(entry.incoming_frame_counter, 0x2200);
        assert!(entry.incoming_frame_counter_valid);
    }

    #[test]
    #[cfg(feature = "router")]
    fn tc_verify_key_requires_the_installed_key_and_uses_its_counter() {
        let mut aps = aps_node(DeviceType::Router, LOCAL_SHORT);
        aps.aib_mut().aps_trust_center_address = TC_IEEE;
        assert_eq!(
            block_on(aps.send_tc_verify_key(ShortAddress::COORDINATOR)),
            Err(ApsStatus::SecurityFail)
        );
        assert!(aps.nwk().mac().tx_history().is_empty());

        let key = [0xA5; 16];
        aps.security_mut()
            .add_key(crate::security::ApsLinkKeyEntry {
                partner_address: TC_IEEE,
                key,
                key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0x1234,
                outgoing_frame_counter_limit: 0x1800,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
            .unwrap();

        block_on(aps.send_tc_verify_key(ShortAddress::COORDINATOR)).unwrap();
        let history = aps.nwk().mac().tx_history();
        assert_eq!(history.len(), 1);
        let frame = nwk_payload(&history[0]);
        let (_, header_len) = ApsHeader::parse(&frame).unwrap();
        let (security_header, _) =
            crate::security::ApsSecurityHeader::parse(&frame[header_len..]).unwrap();
        assert_eq!(security_header.frame_counter, 0x1234);
        assert_eq!(
            aps.security_handshake_stats().last_verify_key_frame_counter,
            0x1234
        );
        assert_eq!(
            aps.security_handshake_stats().last_verify_key_trust_center,
            TC_IEEE
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn network_transport_key_has_the_complete_descriptor_and_real_aps_security() {
        let mut aps = aps_node(DeviceType::Coordinator, ShortAddress::COORDINATOR);
        aps.nwk_mut().nib_mut().permit_joining = true;
        let child = aps
            .nwk_mut()
            .handle_child_association(
                CHILD_IEEE,
                CapabilityInfo {
                    device_type_ffd: false,
                    mains_powered: false,
                    rx_on_when_idle: true,
                    security_capable: true,
                    allocate_address: true,
                }
                .to_byte(),
            )
            .unwrap();
        let transported_key = [0xA5; 16];

        block_on(aps.send_transport_key(
            child,
            &CHILD_IEEE,
            0x01,
            &transported_key,
            7,
            &LOCAL_IEEE,
        ))
        .unwrap();

        let history = aps.nwk().mac().tx_history();
        assert_eq!(history.len(), 1);
        let on_air = history[0].payload.as_slice();
        let (nwk_header, nwk_header_len) = zigbee_nwk::frames::NwkHeader::parse(on_air).unwrap();
        assert!(
            !nwk_header.frame_control.security,
            "the joining child does not know the NWK key yet"
        );
        let aps_frame = &on_air[nwk_header_len..];
        let (_, aps_header_len) = ApsHeader::parse(aps_frame).unwrap();
        let (security_header, _) =
            crate::security::ApsSecurityHeader::parse(&aps_frame[aps_header_len..]).unwrap();
        assert_eq!(
            crate::security::ApsSecurityHeader::key_identifier(security_header.security_control),
            crate::security::KEY_ID_KEY_TRANSPORT
        );
        let transport_key =
            crate::security::derive_key_transport_key(aps.security.default_tc_link_key());
        let command = aps_command(aps_frame, &transport_key);
        assert_eq!(command.len(), 35);
        assert_eq!(command[0], crate::frames::ApsCommandId::TransportKey as u8);
        assert_eq!(command[1], 0x01);
        assert_eq!(&command[2..18], &transported_key);
        assert_eq!(command[18], 7);
        assert_eq!(&command[19..27], &CHILD_IEEE);
        assert_eq!(&command[27..35], &LOCAL_IEEE);
    }

    #[test]
    #[cfg(feature = "router")]
    fn apsme_broadcast_key_rotation_uses_nwk_security_and_requested_sequence() {
        let mut aps = aps_node(DeviceType::Coordinator, ShortAddress::COORDINATOR);
        let next_key = [0xB4; 16];
        assert_eq!(
            block_on(
                aps.apsme_transport_key(&crate::apsme::ApsmeTransportKeyRequest {
                    dst_address: BROADCAST_IEEE,
                    key_type: crate::security::ApsKeyType::NetworkKey,
                    key: next_key,
                    key_seq_number: 7,
                })
            ),
            ApsStatus::Success
        );

        let history = aps.nwk().mac().tx_history();
        assert_eq!(history.len(), 1);
        let (nwk_header, _) =
            zigbee_nwk::frames::NwkHeader::parse(history[0].payload.as_slice()).unwrap();
        assert_eq!(nwk_header.dst_addr, ShortAddress::BROADCAST);
        assert!(nwk_header.frame_control.security);
        let transport = nwk_payload(&history[0]);
        let command = aps_command(&transport, &[0; 16]);
        assert_eq!(command[0], crate::frames::ApsCommandId::TransportKey as u8);
        assert_eq!(command[1], 0x01);
        assert_eq!(&command[2..18], &next_key);
        assert_eq!(command[18], 7);
        assert_eq!(&command[19..27], &BROADCAST_IEEE);

        assert_eq!(
            block_on(aps.apsme_switch_key(&crate::apsme::ApsmeSwitchKeyRequest {
                dst_address: BROADCAST_IEEE,
                key_seq_number: 7,
            })),
            ApsStatus::Success
        );
        let history = aps.nwk().mac().tx_history();
        assert_eq!(history.len(), 2);
        let switch = nwk_payload(&history[1]);
        assert_eq!(
            aps_command(&switch, &[0; 16]).as_slice(),
            &[crate::frames::ApsCommandId::SwitchKey as u8, 7]
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn global_and_unique_update_device_variants_follow_security_policy() {
        let expected = [
            crate::frames::ApsCommandId::UpdateDevice as u8,
            CHILD_IEEE[0],
            CHILD_IEEE[1],
            CHILD_IEEE[2],
            CHILD_IEEE[3],
            CHILD_IEEE[4],
            CHILD_IEEE[5],
            CHILD_IEEE[6],
            CHILD_IEEE[7],
            CHILD_SHORT.0 as u8,
            (CHILD_SHORT.0 >> 8) as u8,
            crate::apsme::ApsUpdateDeviceStatus::HighSecurityDeviceUnsecuredJoin as u8,
        ];

        let mut global = aps_node(DeviceType::Router, LOCAL_SHORT);
        global.aib_mut().aps_trust_center_address = TC_IEEE;
        block_on(global.send_update_device(
            &CHILD_IEEE,
            CHILD_SHORT,
            crate::apsme::ApsUpdateDeviceStatus::HighSecurityDeviceUnsecuredJoin,
        ))
        .unwrap();
        let history = global.nwk().mac().tx_history();
        assert_eq!(
            history.len(),
            2,
            "global keys send both interoperable copies"
        );
        let encrypted = nwk_payload(&history[0]);
        let plain = nwk_payload(&history[1]);
        assert_eq!(
            aps_command(&encrypted, global.security.default_tc_link_key()).as_slice(),
            expected
        );
        assert_eq!(aps_command(&plain, &[0; 16]).as_slice(), expected);

        let unique_key = [0x77; 16];
        let mut unique = aps_node(DeviceType::Router, LOCAL_SHORT);
        unique.aib_mut().aps_trust_center_address = TC_IEEE;
        unique
            .security
            .add_key(crate::security::ApsLinkKeyEntry {
                partner_address: TC_IEEE,
                key: unique_key,
                key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0,
                outgoing_frame_counter_limit: u32::MAX,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
            .unwrap();
        block_on(unique.send_update_device(
            &CHILD_IEEE,
            CHILD_SHORT,
            crate::apsme::ApsUpdateDeviceStatus::HighSecurityDeviceUnsecuredJoin,
        ))
        .unwrap();
        let history = unique.nwk().mac().tx_history();
        assert_eq!(history.len(), 1, "a unique key needs no NWK-only copy");
        assert_eq!(
            aps_command(&nwk_payload(&history[0]), &unique_key).as_slice(),
            expected
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn tunneled_aps_frame_is_forwarded_unchanged_and_without_nwk_security() {
        let mut nwk = NwkLayer::new(MockMac::new(LOCAL_IEEE), DeviceType::Router);
        nwk.set_joined(true);
        {
            let nib = nwk.nib_mut();
            nib.pan_id = TEST_PAN;
            nib.network_address = LOCAL_SHORT;
            nib.ieee_address = LOCAL_IEEE;
            nib.permit_joining = true;
        }
        let child = nwk
            .handle_child_association(
                CHILD_IEEE,
                CapabilityInfo {
                    device_type_ffd: false,
                    mains_powered: false,
                    rx_on_when_idle: false,
                    security_capable: true,
                    allocate_address: true,
                }
                .to_byte(),
            )
            .unwrap();
        let mut aps = ApsLayer::new(nwk);
        aps.aib_mut().aps_trust_center_address = TC_IEEE;

        let command = [crate::frames::ApsCommandId::TransportKey as u8, 0x01, 0xA5];
        let transport_key =
            crate::security::derive_key_transport_key(aps.security.default_tc_link_key());
        let mut embedded = [0u8; 80];
        let embedded_len = build_tc_secured_command_frame(
            &aps.security,
            &transport_key,
            &TC_IEEE,
            1,
            2,
            crate::security::KEY_ID_KEY_TRANSPORT,
            false,
            &command,
            &mut embedded,
        )
        .unwrap();
        let mut tunnel_data = [0u8; 88];
        tunnel_data[..8].copy_from_slice(&CHILD_IEEE);
        tunnel_data[8..8 + embedded_len].copy_from_slice(&embedded[..embedded_len]);

        aps.handle_tunnel(
            &tunnel_data[..8 + embedded_len],
            ShortAddress::COORDINATOR,
            true,
            false,
        );
        let tunnel = aps.take_pending_tunnel().expect("the Tunnel is accepted");
        assert_eq!(tunnel.frame(), &embedded[..embedded_len]);
        block_on(aps.forward_tunnel(&tunnel)).unwrap();
        assert!(aps.nwk().indirect_queue().has_pending(child));

        block_on(
            aps.nwk_mut()
                .service_child_data_request(MacAddress::Short(TEST_PAN, child)),
        )
        .unwrap();
        let record = &aps.nwk().mac().tx_history()[0];
        let bytes = record.payload.as_slice();
        let (nwk_header, nwk_header_len) = zigbee_nwk::frames::NwkHeader::parse(bytes).unwrap();
        assert!(!nwk_header.frame_control.security);
        assert_eq!(&bytes[nwk_header_len..], &embedded[..embedded_len]);
    }

    #[test]
    #[cfg(feature = "router")]
    fn transport_key_requires_authenticated_trust_center_security() {
        let mut nwk = NwkLayer::new(MockMac::new(LOCAL_IEEE), DeviceType::EndDevice);
        {
            let nib = nwk.nib_mut();
            nib.pan_id = TEST_PAN;
            nib.network_address = LOCAL_SHORT;
            nib.parent_address = ShortAddress::COORDINATOR;
            nib.ieee_address = LOCAL_IEEE;
        }
        let mut aps = ApsLayer::new(nwk);
        let transported_key = [0xA5; 16];
        let command = network_key_command(transported_key, 7, LOCAL_IEEE);
        let unsecured = unsecured_command_frame(&command, 1);
        let mut decrypted = ApsFrameBuffer::new();

        let _ = aps.process_incoming_aps_frame(
            &unsecured,
            ShortAddress::COORDINATOR,
            LOCAL_SHORT,
            42,
            IncomingNwkSecurity::new(false, None),
            &mut decrypted,
        );
        assert!(aps.nwk().security().active_key().is_none());

        let transport_key =
            crate::security::derive_key_transport_key(aps.security.default_tc_link_key());
        let mut secured = [0u8; 80];
        let secured_len = build_tc_secured_command_frame(
            &aps.security,
            &transport_key,
            &TC_IEEE,
            2,
            1,
            crate::security::KEY_ID_KEY_TRANSPORT,
            false,
            &command,
            &mut secured,
        )
        .unwrap();
        let _ = aps.process_incoming_aps_frame(
            &secured[..secured_len],
            CHILD_SHORT,
            LOCAL_SHORT,
            42,
            IncomingNwkSecurity::new(false, None),
            &mut decrypted,
        );

        let active = aps.nwk().security().active_key().unwrap();
        assert_eq!(active.key, transported_key);
        assert_eq!(active.seq_number, 7);
        assert_eq!(aps.aib().aps_trust_center_address, TC_IEEE);
    }

    #[test]
    #[cfg(feature = "router")]
    fn public_default_key_cannot_override_an_installed_unique_tc_key() {
        let mut aps = aps_node(DeviceType::EndDevice, LOCAL_SHORT);
        aps.aib_mut().aps_trust_center_address = TC_IEEE;
        let public_key = *aps.security.default_tc_link_key();
        aps.security
            .add_key(crate::security::ApsLinkKeyEntry {
                partner_address: TC_IEEE,
                key: [0x77; 16],
                key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0,
                outgoing_frame_counter_limit: u32::MAX,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
            .unwrap();

        let command = network_key_command([0xE8; 16], 9, LOCAL_IEEE);
        let mut forged = [0u8; 80];
        let forged_len = build_tc_secured_command_frame(
            &aps.security,
            &public_key,
            &TC_IEEE,
            6,
            1,
            crate::security::KEY_ID_KEY_TRANSPORT,
            false,
            &command,
            &mut forged,
        )
        .unwrap();
        let mut decrypted = ApsFrameBuffer::new();
        let _ = aps.process_incoming_aps_frame(
            &forged[..forged_len],
            ShortAddress::COORDINATOR,
            LOCAL_SHORT,
            42,
            IncomingNwkSecurity::new(false, None),
            &mut decrypted,
        );

        assert!(aps.nwk().security().key_by_seq(9).is_none());
        assert_eq!(aps.nwk().security().active_key().unwrap().seq_number, 0);
    }

    #[test]
    #[cfg(feature = "router")]
    fn authenticated_broadcast_network_key_is_staged_until_switch_key() {
        let mut aps = aps_node(DeviceType::Router, LOCAL_SHORT);
        aps.aib_mut().aps_trust_center_address = TC_IEEE;
        let next_key = [0xB6; 16];
        let transport =
            unsecured_command_frame(&network_key_command(next_key, 1, BROADCAST_IEEE), 3);
        let mut decrypted = ApsFrameBuffer::new();

        let _ = aps.process_incoming_aps_frame(
            &transport,
            ShortAddress::COORDINATOR,
            LOCAL_SHORT,
            42,
            IncomingNwkSecurity::new(true, Some([0x44; 8])),
            &mut decrypted,
        );
        assert_eq!(aps.nwk().security().active_key().unwrap().seq_number, 0);
        assert_eq!(aps.nwk().security().key_by_seq(1).unwrap().key, next_key);

        let forged =
            unsecured_command_frame(&network_key_command([0xC7; 16], 2, BROADCAST_IEEE), 4);
        let _ = aps.process_incoming_aps_frame(
            &forged,
            ShortAddress::COORDINATOR,
            LOCAL_SHORT,
            42,
            IncomingNwkSecurity::new(false, None),
            &mut decrypted,
        );
        assert!(aps.nwk().security().key_by_seq(2).is_none());

        let switch = unsecured_command_frame(&[crate::frames::ApsCommandId::SwitchKey as u8, 1], 5);
        let _ = aps.process_incoming_aps_frame(
            &switch,
            ShortAddress::COORDINATOR,
            LOCAL_SHORT,
            42,
            IncomingNwkSecurity::new(false, None),
            &mut decrypted,
        );
        assert_eq!(aps.nwk().security().active_key().unwrap().seq_number, 0);

        let _ = aps.process_incoming_aps_frame(
            &switch,
            ShortAddress::COORDINATOR,
            LOCAL_SHORT,
            42,
            IncomingNwkSecurity::new(true, Some([0x44; 8])),
            &mut decrypted,
        );
        assert_eq!(aps.nwk().security().active_key().unwrap().seq_number, 1);
        assert_eq!(aps.nwk().nib().active_key_seq_number, 1);
    }

    /// End-to-end cover for the extracted `aps_decrypt_incoming` /
    /// `decrypt_into` phase: a real CCM*-secured APS command must decrypt and
    /// be acted on exactly once, and its frame counter must be committed only
    /// after the MIC verifies so an identical replay is refused before any
    /// second decryption. This pins the R22 "replay-check before decrypt,
    /// commit once after MIC" ordering that the phase extraction must preserve.
    #[test]
    #[cfg(feature = "router")]
    fn secured_aps_command_decrypts_once_then_rejects_the_replay() {
        let mut aps = aps_node(DeviceType::EndDevice, LOCAL_SHORT);
        aps.aib_mut().aps_trust_center_address = TC_IEEE;
        let link_key = [0x7A; 16];
        aps.security_mut()
            .add_key(crate::security::ApsLinkKeyEntry {
                partner_address: TC_IEEE,
                key: link_key,
                key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 0,
                outgoing_frame_counter_limit: 0x1000,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
            .unwrap();

        // The Trust Center encrypts a Confirm-Key command (status SUCCESS,
        // TC-link key type, addressed to us) with the shared link key.
        let tc_security = crate::security::ApsSecurity::new();
        let mut confirm_key = [0u8; 11];
        confirm_key[0] = crate::frames::ApsCommandId::ConfirmKey as u8;
        confirm_key[1] = 0x00;
        confirm_key[2] = WIRE_KEY_TYPE_TC_LINK;
        confirm_key[3..11].copy_from_slice(&LOCAL_IEEE);
        let mut frame = [0u8; 64];
        let len = build_tc_secured_command_frame(
            &tc_security,
            &link_key,
            &TC_IEEE,
            0x11,
            0x0000_0100,
            crate::security::KEY_ID_DATA_KEY,
            false,
            &confirm_key,
            &mut frame,
        )
        .unwrap();
        let nwk_security = IncomingNwkSecurity::new(true, Some(TC_IEEE));

        // First delivery decrypts, dispatches Confirm-Key and commits 0x100.
        let mut buf = ApsFrameBuffer::new();
        assert!(
            aps.process_incoming_aps_frame(
                &frame[..len],
                ShortAddress::COORDINATOR,
                LOCAL_SHORT,
                180,
                nwk_security,
                &mut buf,
            )
            .is_none()
        );
        assert_eq!(aps.security_handshake_stats().confirm_key_received, 1);
        assert_eq!(aps.security_handshake_stats().confirm_key_successes, 1);
        let entry = aps
            .security()
            .find_key(&TC_IEEE, crate::security::ApsKeyType::TrustCenterLinkKey)
            .unwrap();
        assert!(entry.incoming_frame_counter_valid);
        assert_eq!(entry.incoming_frame_counter, 0x0000_0100);

        // The identical frame replays counter 0x100: the pre-decrypt replay
        // check drops it, so Confirm-Key is not processed a second time and the
        // committed counter is untouched.
        let mut replay_buf = ApsFrameBuffer::new();
        assert!(
            aps.process_incoming_aps_frame(
                &frame[..len],
                ShortAddress::COORDINATOR,
                LOCAL_SHORT,
                180,
                nwk_security,
                &mut replay_buf,
            )
            .is_none()
        );
        assert_eq!(aps.security_handshake_stats().confirm_key_received, 1);
        let after = aps
            .security()
            .find_key(&TC_IEEE, crate::security::ApsKeyType::TrustCenterLinkKey)
            .unwrap();
        assert_eq!(after.incoming_frame_counter, 0x0000_0100);

        // A forged frame at a higher counter but with a corrupted MIC must fail
        // the MIC and must NOT advance the committed counter.
        let mut forged = [0u8; 64];
        let forged_len = build_tc_secured_command_frame(
            &tc_security,
            &link_key,
            &TC_IEEE,
            0x12,
            0x0000_0200,
            crate::security::KEY_ID_DATA_KEY,
            false,
            &confirm_key,
            &mut forged,
        )
        .unwrap();
        forged[forged_len - 1] ^= 0xFF;
        let mut forged_buf = ApsFrameBuffer::new();
        assert!(
            aps.process_incoming_aps_frame(
                &forged[..forged_len],
                ShortAddress::COORDINATOR,
                LOCAL_SHORT,
                180,
                nwk_security,
                &mut forged_buf,
            )
            .is_none()
        );
        assert_eq!(aps.security_handshake_stats().confirm_key_received, 1);
        let unchanged = aps
            .security()
            .find_key(&TC_IEEE, crate::security::ApsKeyType::TrustCenterLinkKey)
            .unwrap();
        assert_eq!(unchanged.incoming_frame_counter, 0x0000_0100);
    }
}
