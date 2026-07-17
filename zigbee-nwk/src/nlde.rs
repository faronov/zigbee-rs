//! NLDE — NWK Layer Data Entity.
//!
//! Handles sending and receiving NWK data frames via the MAC layer.
//! - NLDE-DATA.request: send NWK data to a destination
//! - NLDE-DATA.indication: receive NWK data from the network
//! - Frame relay for routers/coordinators

use crate::frames::{NwkCommandId, NwkFrameControl, NwkFrameType, NwkHeader};
use crate::{DeviceType, NwkLayer, NwkStatus};
use zigbee_mac::{AddressMode, MacDriver, McpsDataRequest, TxOptions};
use zigbee_types::*;

#[cfg(feature = "efr32-trace")]
macro_rules! nwk_trace {
    ($($arg:tt)*) => { rtt_target::rprintln!($($arg)*); };
}
#[cfg(not(feature = "efr32-trace"))]
macro_rules! nwk_trace {
    ($($arg:tt)*) => {
        ()
    };
}

// ── Telink TLSR8258 debug markers (NWK TX path) ────────────────────────────
// Absolute SRAM map (matches the convention in zigbee-bdb/steering.rs):
//   BDB_BASE = 0x0084_F450
//   BDB+0x230 nlde_data_request entry counter (every call)
//   BDB+0x234 entry-state latch (first call only, MSB=1):
//             bit24 = joined, bit16 = security_enabled,
//             bit8  = has_active_key, bit0  = discover_route
//   BDB+0x238 dst_addr (low u16) | src_addr (high u16) — last call
//   BDB+0x23C frame_counter used (last secure call)
//   BDB+0x240 hdr_len (low u16) | total_len (high u16) — last call
//   BDB+0x244 NWK CCM encrypt entries (from security::aes_ccm_encrypt)
//   BDB+0x248 SP at CCM encrypt entry (must stay > 0x0084_B400)
//   BDB+0x24C NWK CCM encrypt successful exits
//   BDB+0x250 self.security.encrypt() returned Some count
//   BDB+0x254 self.security.encrypt() returned None count
//   BDB+0x258 MAC mcps_data Ok count
//   BDB+0x25C MAC mcps_data Err count
//
// Wire-bytes capture region (only FIRST nlde_data_request call):
//   0x0084_FB00..+0x10  pre-encrypt: NWK header bytes (16)
//   0x0084_FB10..+0x10  pre-encrypt: NWK security aux header (14, padded)
//   0x0084_FB20..+0x40  post-encrypt: assembled NWK frame (up to 64 B)
//   0x0084_FB60         one-shot guard (0xDEADBEEF after first capture)
//   0x0084_FB64         hdr_len | (sec_hdr_len << 16)
//   0x0084_FB68         total_len | (payload_len << 16)
//   0x0084_FB6C         dst_addr | (next_hop << 16) — populated post-resolve
#[cfg(feature = "telink-debug")]
mod tlnk_dbg {
    pub const BASE: u32 = 0x0084_F450;
    pub const WIRE_BASE: u32 = 0x0084_FB00;
    pub const WIRE_GUARD: u32 = 0x0084_FB60;

    #[inline(always)]
    pub fn bump(off: u32) {
        unsafe {
            let p = (BASE + off) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
    }
    #[inline(always)]
    pub fn set(off: u32, val: u32) {
        unsafe {
            core::ptr::write_volatile((BASE + off) as *mut u32, val);
        }
    }
    #[inline(always)]
    pub fn set_once(off: u32, val: u32) {
        unsafe {
            let p = (BASE + off) as *mut u32;
            if core::ptr::read_volatile(p) == 0 {
                core::ptr::write_volatile(p, val | 0x8000_0000);
            }
        }
    }
    /// Capture the first nlde_data_request frame to 0x0084_FB00..+0x70.
    /// Returns true if the capture slot was taken (caller should fill it).
    #[inline(always)]
    pub fn wire_claim() -> bool {
        unsafe {
            let g = WIRE_GUARD as *mut u32;
            if core::ptr::read_volatile(g) == 0xDEAD_BEEF {
                return false;
            }
            core::ptr::write_volatile(g, 0xDEAD_BEEF);
            true
        }
    }
    #[inline(always)]
    pub fn wire_copy(off: u32, src: &[u8]) {
        unsafe {
            let base = (WIRE_BASE + off) as *mut u8;
            for (i, b) in src.iter().enumerate() {
                core::ptr::write_volatile(base.add(i), *b);
            }
        }
    }
    #[inline(always)]
    pub fn wire_set(off: u32, val: u32) {
        unsafe {
            core::ptr::write_volatile((WIRE_BASE + off) as *mut u32, val);
        }
    }
}

#[cfg(feature = "telink-debug")]
macro_rules! tdbg_bump {
    ($off:expr) => {
        tlnk_dbg::bump($off)
    };
}
#[cfg(not(feature = "telink-debug"))]
macro_rules! tdbg_bump {
    ($off:expr) => {
        ()
    };
}
#[cfg(feature = "telink-debug")]
macro_rules! tdbg_set {
    ($off:expr, $val:expr) => {
        tlnk_dbg::set($off, $val)
    };
}
#[cfg(not(feature = "telink-debug"))]
macro_rules! tdbg_set {
    ($off:expr, $val:expr) => {
        ()
    };
}

#[cfg(feature = "telink-tx-capture")]
mod tx_capture {
    use zigbee_types::ShortAddress;

    const BASE: u32 = 0x0084_FF00;
    const MAGIC: u32 = 0x5854_4B52; // "RKTX"
    const PAYLOAD_OFFSET: u32 = 0x20;
    const PAYLOAD_CAPACITY: usize = 64;
    const FRAME_OFFSET: u32 = 0x60;
    const FRAME_CAPACITY: usize = 96;
    const TIMESTAMP_OFFSET: u32 = 0xC0;

    #[inline(always)]
    unsafe fn read_u32(off: u32) -> u32 {
        unsafe { core::ptr::read_volatile((BASE + off) as *const u32) }
    }

    #[inline(always)]
    unsafe fn write_u32(off: u32, value: u32) {
        unsafe { core::ptr::write_volatile((BASE + off) as *mut u32, value) }
    }

    #[inline(always)]
    unsafe fn clear_bytes(off: u32, len: usize) {
        let dst = (BASE + off) as *mut u8;
        for index in 0..len {
            unsafe { core::ptr::write_volatile(dst.add(index), 0) };
        }
    }

    #[inline(always)]
    unsafe fn copy_bytes(off: u32, src: &[u8], capacity: usize) {
        let dst = (BASE + off) as *mut u8;
        for (index, byte) in src.iter().take(capacity).enumerate() {
            unsafe { core::ptr::write_volatile(dst.add(index), *byte) };
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn record(
        dst: ShortAddress,
        next_hop: ShortAddress,
        source: ShortAddress,
        sequence: u8,
        secured: bool,
        header_len: usize,
        security_header_len: usize,
        frame_counter: u32,
        timestamp_micros: u32,
        payload: &[u8],
        frame: &[u8],
    ) {
        unsafe {
            let count = if read_u32(0) == MAGIC {
                read_u32(4).wrapping_add(1)
            } else {
                1
            };
            write_u32(0, MAGIC);
            write_u32(4, count);
            write_u32(8, dst.0 as u32 | ((next_hop.0 as u32) << 16));
            write_u32(
                12,
                (header_len as u32 & 0xFFFF) | ((security_header_len as u32 & 0xFFFF) << 16),
            );
            write_u32(
                16,
                (payload.len() as u32 & 0xFFFF) | ((frame.len() as u32 & 0xFFFF) << 16),
            );
            write_u32(20, frame_counter);
            write_u32(24, u32::MAX);
            write_u32(
                28,
                source.0 as u32 | ((sequence as u32) << 16) | ((secured as u32) << 24),
            );
            clear_bytes(PAYLOAD_OFFSET, PAYLOAD_CAPACITY);
            copy_bytes(PAYLOAD_OFFSET, payload, PAYLOAD_CAPACITY);
            clear_bytes(FRAME_OFFSET, FRAME_CAPACITY);
            copy_bytes(FRAME_OFFSET, frame, FRAME_CAPACITY);
            write_u32(TIMESTAMP_OFFSET, timestamp_micros);
        }
    }

    pub fn set_result(success: bool) {
        unsafe {
            write_u32(24, if success { 0 } else { 1 });
        }
    }
}

/// NWK data indication — received NWK-level data.
#[derive(Debug)]
pub struct NldeDataIndication<'a> {
    pub dst_addr: ShortAddress,
    pub src_addr: ShortAddress,
    pub payload: &'a [u8],
    pub lqi: u8,
    pub security_use: bool,
}

/// Owned NWK data indication — for decrypted frames where payload is owned.
#[derive(Debug)]
pub struct NldeDataIndicationOwned {
    pub dst_addr: ShortAddress,
    pub src_addr: ShortAddress,
    pub payload: heapless::Vec<u8, 128>,
    pub lqi: u8,
    pub security_use: bool,
}

/// Result of processing an incoming NWK frame.
#[derive(Debug)]
pub enum NwkIndication<'a> {
    /// Unsecured frame — payload borrows from MAC buffer
    Borrowed(NldeDataIndication<'a>),
    /// Decrypted frame — payload is owned
    Owned(NldeDataIndicationOwned),
}

/// NWK data confirm — result of NLDE-DATA.request.
#[derive(Debug)]
pub struct NldeDataConfirm {
    pub status: NwkStatus,
    pub nsdu_handle: u8,
}

impl<M: MacDriver> NwkLayer<M> {
    /// Send NWK data to a destination address.
    ///
    /// This is the primary data service used by the APS layer above.
    /// It builds a NWK frame, determines the MAC next-hop, and sends via MAC.
    pub async fn nlde_data_request(
        &mut self,
        dst_addr: ShortAddress,
        radius: u8,
        payload: &[u8],
        security_enable: bool,
        discover_route: bool,
    ) -> Result<NldeDataConfirm, NwkStatus> {
        #[cfg(feature = "telink-tx-capture")]
        let mut capture_security_header_len = 0usize;
        #[cfg(feature = "telink-tx-capture")]
        let mut capture_frame_counter = 0u32;
        #[cfg(feature = "telink-tx-capture")]
        let capture_this_frame =
            payload.len() == 21 && payload[0] == 0x21 && payload[2] & 0xE7 == 0x20;

        // BDB+0x230: nlde_data_request entries (every call)
        tdbg_bump!(0x230);
        // BDB+0x234: latch entry flags on FIRST call only (joined / sec_enabled
        // / has_active_key / discover_route). MSB is set by set_once().
        #[cfg(feature = "telink-debug")]
        {
            let joined = self.joined as u32;
            let sec_en = self.nib.security_enabled as u32;
            let has_key = self.security.active_key().is_some() as u32;
            let dr = discover_route as u32;
            let flags = (joined << 24) | (sec_en << 16) | (has_key << 8) | dr;
            // Use plain set (not set_once) so each call updates — easier to
            // read in the dump. The MSB bit is informational only.
            tdbg_set!(0x234, 0x4E57_0000 | flags);
        }

        if !self.joined {
            log::warn!(
                "[NWK] nlde_data_request called but not joined! dst=0x{:04X}",
                dst_addr.0
            );
            return Err(NwkStatus::InvalidRequest);
        }

        let seq = self.nib.next_seq();

        // Build NWK header
        // Note: multicast flag is ONLY for group-addressed frames (via APS group delivery).
        // Broadcast addresses (0xFFF8-0xFFFF) must NOT set the multicast flag.
        // End devices suppress route discovery (parent handles routing).

        // If we're a concentrator with a cached source route, attach it to the header
        let source_route_subframe = if self.concentrator_active {
            self.source_route_table.lookup(dst_addr).map(|relays| {
                let mut relay_list = heapless::Vec::new();
                for addr in relays {
                    let _ = relay_list.push(*addr);
                }
                crate::frames::SourceRoute {
                    relay_count: relay_list.len() as u8,
                    relay_index: relay_list.len() as u8,
                    relay_list,
                }
            })
        } else {
            None
        };
        let has_source_route = source_route_subframe.is_some();

        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Data as u8,
                protocol_version: 0x02,
                discover_route: if discover_route && self.device_type != DeviceType::EndDevice {
                    1
                } else {
                    0
                },
                multicast: false,
                security: security_enable && self.nib.security_enabled,
                source_route: has_source_route,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: false, // Maximise compatibility with older stacks
            },
            dst_addr,
            src_addr: self.nib.network_address,
            radius,
            seq_number: seq,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: source_route_subframe,
        };

        // Serialize NWK frame
        let mut nwk_buf = [0u8; 128];
        let hdr_len = header.serialize(&mut nwk_buf);

        let total_len;
        if security_enable && self.nib.security_enabled {
            // Check key availability BEFORE allocating frame counter
            let key_entry = match self.security.active_key() {
                Some(k) => k.clone(),
                None => {
                    log::warn!("[NWK] No active network key for encryption");
                    return Err(NwkStatus::InvalidRequest);
                }
            };

            // Build NWK security auxiliary header
            let sec_hdr = crate::security::NwkSecurityHeader {
                security_control: crate::security::NwkSecurityHeader::ZIGBEE_DEFAULT,
                frame_counter: self
                    .nib
                    .next_frame_counter()
                    .ok_or(NwkStatus::InvalidRequest)?,
                source_address: self.nib.ieee_address,
                key_seq_number: self.nib.active_key_seq_number,
            };
            log::info!(
                "[NWK TX] sec: fc={} key_seq={} ieee={:02X?}",
                sec_hdr.frame_counter,
                sec_hdr.key_seq_number,
                &sec_hdr.source_address[..4],
            );

            // Serialize security header right after NWK header
            let sec_hdr_len = sec_hdr.serialize(&mut nwk_buf[hdr_len..]);
            #[cfg(feature = "telink-tx-capture")]
            {
                capture_security_header_len = sec_hdr_len;
                capture_frame_counter = sec_hdr.frame_counter;
            }

            // BDB+0x238: dst_addr | (src_addr << 16) — last call
            tdbg_set!(
                0x238,
                dst_addr.0 as u32 | ((self.nib.network_address.0 as u32) << 16)
            );
            // BDB+0x23C: frame counter actually used
            tdbg_set!(0x23C, sec_hdr.frame_counter);

            // Capture FIRST nlde_data_request pre-encrypt context to
            // 0x0084_FB00 (header) / 0x0084_FB10 (security aux header).
            #[cfg(feature = "telink-debug")]
            {
                if tlnk_dbg::wire_claim() {
                    let n = core::cmp::min(hdr_len, 16);
                    tlnk_dbg::wire_copy(0x00, &nwk_buf[..n]);
                    let m = core::cmp::min(sec_hdr_len, 16);
                    tlnk_dbg::wire_copy(0x10, &nwk_buf[hdr_len..hdr_len + m]);
                    tlnk_dbg::wire_set(0x64, (hdr_len as u32) | ((sec_hdr_len as u32) << 16));
                    tlnk_dbg::wire_set(0x68, (payload.len() as u32) | ((dst_addr.0 as u32) << 16));
                } else {
                    // Subsequent calls: re-arm by clearing the guard so the
                    // dump still shows the most recent FIRST capture.
                    // (No-op: we keep the first one to preserve evidence.)
                }
            }

            // Build authenticated data (a = NWK header || security aux header)
            let aad_len = hdr_len + sec_hdr_len;

            // Encrypt payload with NWK key
            if let Some(encrypted) =
                self.security
                    .encrypt(&nwk_buf[..aad_len], payload, &key_entry.key, &sec_hdr)
            {
                // BDB+0x250: security.encrypt returned Some
                tdbg_bump!(0x250);
                if aad_len + encrypted.len() > nwk_buf.len() {
                    return Err(NwkStatus::FrameTooLong);
                }
                nwk_buf[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
                total_len = aad_len + encrypted.len();
                // Zero security level bits for OTA transmission (spec §4.3.1.2)
                nwk_buf[hdr_len] &= !0x07;
            } else {
                // BDB+0x254: security.encrypt returned None
                tdbg_bump!(0x254);
                log::warn!("[NWK] Encryption failed");
                return Err(NwkStatus::InvalidRequest);
            }
        } else {
            // No security — copy plaintext payload directly
            if hdr_len + payload.len() > nwk_buf.len() {
                return Err(NwkStatus::FrameTooLong);
            }
            nwk_buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
            total_len = hdr_len + payload.len();
        }

        // Determine MAC-level next hop
        let next_hop = self.resolve_next_hop(dst_addr)?;

        // Auto Route Record: if the destination route requires a Route Record
        // (many-to-one concentrator), send it BEFORE the data frame.
        // This is how devices inform the concentrator of the reverse path.
        if dst_addr.0 < 0xFFF8 {
            let needs_rr = self
                .routing
                .get_entry(dst_addr)
                .map(|e| e.route_record_required)
                .unwrap_or(false);
            if needs_rr {
                // Send Route Record with empty relay list (we're the originator)
                // Intermediate routers will append their addresses as it's forwarded
                log::debug!(
                    "[NWK] Sending Route Record to concentrator 0x{:04X}",
                    dst_addr.0
                );
                let _ = self.send_route_record(dst_addr, &[]).await;
                self.routing.clear_route_record_required(dst_addr);
            }
        }

        #[cfg(feature = "telink-tx-capture")]
        if capture_this_frame {
            tx_capture::record(
                dst_addr,
                next_hop,
                self.nib.network_address,
                seq,
                security_enable && self.nib.security_enabled,
                hdr_len,
                capture_security_header_len,
                capture_frame_counter,
                self.mac.monotonic_micros().unwrap_or(u32::MAX),
                payload,
                &nwk_buf[..total_len],
            );
        }

        // BDB+0x240: hdr_len | (total_len << 16)
        tdbg_set!(
            0x240,
            (hdr_len as u32 & 0xFFFF) | ((total_len as u32 & 0xFFFF) << 16)
        );
        // Capture post-encrypt frame bytes (up to 64) to 0x0084_FB20.
        #[cfg(feature = "telink-debug")]
        {
            let n = core::cmp::min(total_len, 64);
            tlnk_dbg::wire_copy(0x20, &nwk_buf[..n]);
            tlnk_dbg::wire_set(0x6C, dst_addr.0 as u32 | ((next_hop.0 as u32) << 16));
        }

        log::info!(
            "[NWK TX] dst=0x{:04X} next_hop=0x{:04X} sec={} len={} hdr={:02X?}",
            dst_addr.0,
            next_hop.0,
            security_enable && self.nib.security_enabled,
            total_len,
            &nwk_buf[..core::cmp::min(8, total_len)],
        );
        nwk_trace!(
            "[NWK][EFR32] tx dst=0x{:04X} nh=0x{:04X} src=0x{:04X} sec={} len={} seq={}",
            dst_addr.0,
            next_hop.0,
            self.nib.network_address.0,
            (security_enable && self.nib.security_enabled) as u8,
            total_len,
            seq
        );

        // Send via MAC
        let mac_result = self
            .mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &nwk_buf[..total_len],
                msdu_handle: seq,
                tx_options: TxOptions {
                    // Fix 9: No MAC ACK for broadcast
                    ack_tx: next_hop.0 != 0xFFFF,
                    ..Default::default()
                },
            })
            .await;

        #[cfg(feature = "telink-tx-capture")]
        if capture_this_frame {
            tx_capture::set_result(mac_result.is_ok());
        }

        if let Err(ref e) = mac_result {
            // BDB+0x25C: MAC mcps_data Err count
            tdbg_bump!(0x25C);
            nwk_trace!(
                "[NWK][EFR32] tx_err dst=0x{:04X} nh=0x{:04X} seq={} err={:?}",
                dst_addr.0,
                next_hop.0,
                seq,
                e
            );
            log::warn!("[NWK TX] MAC send failed: {:?}", e);
        }

        mac_result.map_err(|_| NwkStatus::RouteError)?;
        // BDB+0x258: MAC mcps_data Ok count
        tdbg_bump!(0x258);
        nwk_trace!(
            "[NWK][EFR32] tx_ok dst=0x{:04X} nh=0x{:04X} seq={}",
            dst_addr.0,
            next_hop.0,
            seq
        );

        Ok(NldeDataConfirm {
            status: NwkStatus::Success,
            nsdu_handle: seq,
        })
    }

    /// Process incoming MAC data indication as a NWK frame.
    ///
    /// Parses the NWK header and either:
    /// - Delivers to upper layer (if destined for us)
    /// - Relays the frame (if we're a router/coordinator)
    pub async fn process_incoming_nwk_frame<'a>(
        &mut self,
        mac_payload: &'a [u8],
        lqi: u8,
    ) -> Option<NwkIndication<'a>> {
        // Parse NWK header
        let (header, consumed) = NwkHeader::parse(mac_payload)?;

        let dst = header.dst_addr;
        let src = header.src_addr;
        let is_broadcast = dst.0 >= 0xFFF8;
        let is_for_us = dst == self.nib.network_address || is_broadcast;

        // ── Broadcast deduplication (BTR) ──
        if is_broadcast && self.device_type != DeviceType::EndDevice {
            if self.btr.is_duplicate(src, header.seq_number) {
                log::debug!(
                    "[NWK] BTR dup: src=0x{:04X} seq={}",
                    src.0,
                    header.seq_number
                );
                return None;
            }
            self.btr.record(src, header.seq_number);
        }

        // ── Broadcast relay (routers/coordinators rebroadcast) ──
        if is_broadcast && self.device_type != DeviceType::EndDevice && header.radius > 1 {
            let _ = self.relay_broadcast(mac_payload, &header).await;
        }

        if is_for_us {
            if header.frame_control.security {
                self.rx_security_stats.secured_frames =
                    self.rx_security_stats.secured_frames.wrapping_add(1);
                // Parse NWK security auxiliary header
                let after_header = &mac_payload[consumed..];
                let Some((sec_hdr, sec_consumed)) =
                    crate::security::NwkSecurityHeader::parse(after_header)
                else {
                    self.rx_security_stats.security_header_parse_failures = self
                        .rx_security_stats
                        .security_header_parse_failures
                        .wrapping_add(1);
                    return None;
                };

                // Look up key
                let Some(key_entry) = self.security.key_by_seq(sec_hdr.key_seq_number) else {
                    self.rx_security_stats.missing_keys =
                        self.rx_security_stats.missing_keys.wrapping_add(1);
                    return None;
                };
                let key = key_entry.key;

                log::info!(
                    "[NWK SEC] sc=0x{:02X} fc={} key_seq={}",
                    sec_hdr.security_control,
                    sec_hdr.frame_counter,
                    sec_hdr.key_seq_number,
                );

                // Step 1: Check frame counter WITHOUT committing (replay protection)
                if !self
                    .security
                    .check_frame_counter(&sec_hdr.source_address, sec_hdr.frame_counter)
                {
                    self.rx_security_stats.replay_rejections =
                        self.rx_security_stats.replay_rejections.wrapping_add(1);
                    log::warn!("[NWK] Frame counter replay from 0x{:04X}", src.0);
                    return None;
                }

                // Step 2: Decrypt and verify MIC
                let aad_len = consumed + sec_consumed;
                // AAD must use ACTUAL security level (5), not OTA value (0).
                // The security control byte in the aux header is at offset `consumed`.
                let mut aad_buf = [0u8; 64];
                let aad_copy_len = aad_len.min(aad_buf.len());
                aad_buf[..aad_copy_len].copy_from_slice(&mac_payload[..aad_copy_len]);
                aad_buf[consumed] = (aad_buf[consumed] & !0x07) | 0x05;
                let plaintext = self.security.decrypt(
                    &aad_buf[..aad_copy_len],
                    &after_header[sec_consumed..],
                    &key,
                    &sec_hdr,
                );

                match plaintext {
                    Some(pt) => {
                        self.rx_security_stats.decrypt_successes =
                            self.rx_security_stats.decrypt_successes.wrapping_add(1);
                        // Step 3: MIC verified — NOW commit frame counter
                        self.security
                            .commit_frame_counter(&sec_hdr.source_address, sec_hdr.frame_counter);

                        log::debug!(
                            "[NWK] Decrypted frame from 0x{:04X} ({} bytes)",
                            src.0,
                            pt.len()
                        );

                        // NWK command frames are handled internally, not passed to APS
                        if header.frame_control.frame_type == NwkFrameType::Command as u8 {
                            self.dispatch_nwk_command(src, &pt);
                            return None;
                        }

                        return Some(NwkIndication::Owned(NldeDataIndicationOwned {
                            dst_addr: dst,
                            src_addr: src,
                            payload: pt,
                            lqi,
                            security_use: true,
                        }));
                    }
                    None => {
                        self.rx_security_stats.decrypt_failures =
                            self.rx_security_stats.decrypt_failures.wrapping_add(1);
                        log::warn!("[NWK] Decrypt/MIC failed from 0x{:04X}", src.0);
                        // Do NOT commit frame counter — frame is forged/corrupted
                        return None;
                    }
                }
            }

            // Unsecured frame
            let payload = &mac_payload[consumed..];

            // NWK command frames are handled internally, not passed to APS
            if header.frame_control.frame_type == NwkFrameType::Command as u8 {
                self.dispatch_nwk_command(src, payload);
                return None;
            }

            return Some(NwkIndication::Borrowed(NldeDataIndication {
                dst_addr: dst,
                src_addr: src,
                payload,
                lqi,
                security_use: false,
            }));
        }

        // Not for us — relay unicast if router/coordinator
        if self.device_type != DeviceType::EndDevice && header.radius > 1 {
            let _ = self.relay_frame(mac_payload, &header).await;
        }

        None
    }

    /// Relay a NWK frame (router/coordinator duty).
    async fn relay_frame(&mut self, original: &[u8], header: &NwkHeader) -> Result<(), NwkStatus> {
        // Decrement radius
        let new_radius = header.radius.saturating_sub(1);
        if new_radius == 0 {
            return Ok(()); // TTL expired
        }

        // ── Source routing: use relay list instead of routing table ──
        if let Some(ref sr) = header.source_route {
            return self
                .relay_frame_source_routed(original, header, sr, new_radius)
                .await;
        }

        // Determine next hop for the final destination
        let next_hop = self.resolve_next_hop(header.dst_addr)?;

        // Check if next hop is a sleepy child — buffer in indirect queue
        if let Some(neighbor) = self.neighbors.find_by_short(next_hop)
            && !neighbor.rx_on_when_idle
        {
            // Sleepy child — buffer for indirect delivery
            let mut relay_buf = [0u8; 128];
            let mut new_header = header.clone();
            new_header.radius = new_radius;
            let hdr_len = new_header.serialize(&mut relay_buf);
            let (_, orig_hdr_len) = match NwkHeader::parse(original) {
                Some(parsed) => parsed,
                None => return Err(NwkStatus::InvalidParameter),
            };
            let payload = &original[orig_hdr_len..];
            if hdr_len + payload.len() > relay_buf.len() {
                return Err(NwkStatus::FrameTooLong);
            }
            relay_buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
            let total = hdr_len + payload.len();
            if self.indirect.enqueue(next_hop, &relay_buf[..total]) {
                log::debug!(
                    "[NWK] Buffered indirect frame for sleepy child 0x{:04X}",
                    next_hop.0
                );
                return Ok(());
            }
            log::warn!("[NWK] Indirect queue full for 0x{:04X}", next_hop.0);
            return Err(NwkStatus::FrameNotBuffered);
        }

        // Rebuild frame with decremented radius
        let mut relay_buf = [0u8; 128];
        let mut new_header = header.clone();
        new_header.radius = new_radius;
        let hdr_len = new_header.serialize(&mut relay_buf);

        // Copy original payload (everything after header)
        let (_, orig_hdr_len) = match NwkHeader::parse(original) {
            Some(parsed) => parsed,
            None => {
                log::warn!("[NWK] Failed to re-parse NWK header for relay");
                return Err(NwkStatus::InvalidParameter);
            }
        };
        let payload = &original[orig_hdr_len..];
        if hdr_len + payload.len() > relay_buf.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        relay_buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
        let total = hdr_len + payload.len();

        let mac_result = self
            .mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &relay_buf[..total],
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: next_hop.0 != 0xFFFF,
                    ..Default::default()
                },
            })
            .await;

        // ── Route repair: if MAC TX fails, handle relay failure ──
        if mac_result.is_err() {
            self.handle_relay_failure(header.dst_addr, header.src_addr, next_hop);
            return Err(NwkStatus::RouteError);
        }

        Ok(())
    }

    /// Relay a frame using source routing (relay list in NWK header).
    async fn relay_frame_source_routed(
        &mut self,
        original: &[u8],
        header: &NwkHeader,
        sr: &crate::frames::SourceRoute,
        new_radius: u8,
    ) -> Result<(), NwkStatus> {
        let our_addr = self.nib.network_address;

        // Find next hop from source route relay list.
        // Relay list is ordered [closest-to-dest, ..., closest-to-source].
        // relay_index points to current position; decrement to advance toward destination.
        let (next_hop, new_index) = process_source_route(sr, our_addr, header.dst_addr)?;

        // Build new header with updated source route
        let mut new_header = header.clone();
        new_header.radius = new_radius;
        if let Some(ref mut new_sr) = new_header.source_route {
            new_sr.relay_index = new_index;
        }

        let mut relay_buf = [0u8; 128];
        let hdr_len = new_header.serialize(&mut relay_buf);

        let (_, orig_hdr_len) = match NwkHeader::parse(original) {
            Some(parsed) => parsed,
            None => return Err(NwkStatus::InvalidParameter),
        };
        let payload = &original[orig_hdr_len..];
        if hdr_len + payload.len() > relay_buf.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        relay_buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
        let total = hdr_len + payload.len();

        log::debug!(
            "[NWK] Source-route relay: next_hop=0x{:04X} index={}→{}",
            next_hop.0,
            sr.relay_index,
            new_index,
        );

        let mac_result = self
            .mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &relay_buf[..total],
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: true,
                    ..Default::default()
                },
            })
            .await;

        if mac_result.is_err() {
            self.handle_relay_failure(header.dst_addr, header.src_addr, next_hop);
            return Err(NwkStatus::RouteError);
        }

        Ok(())
    }

    /// Insert our address into a Route Record relay list in the NWK header.
    ///
    /// If the header doesn't have a source route subframe, create one.
    /// This allows the concentrator to learn the reverse path.
    /// Handle relay failure: remove broken route and queue Network Status error.
    fn handle_relay_failure(
        &mut self,
        failed_dest: ShortAddress,
        frame_source: ShortAddress,
        _failed_next_hop: ShortAddress,
    ) {
        log::warn!(
            "[NWK] Relay failure for dst=0x{:04X}, removing route",
            failed_dest.0,
        );

        // Remove the broken route
        self.routing.remove(failed_dest);

        // Queue a Network Status (route error) to send toward the frame source
        if frame_source != self.nib.network_address {
            let _ = self.pending_route_errors.push(crate::PendingNetworkStatus {
                destination: frame_source,
                status_code: crate::frames::NetworkStatusCommand::NO_ROUTE_AVAILABLE,
                failed_destination: failed_dest,
            });
        }
    }

    /// Relay a broadcast NWK frame via MAC broadcast with decremented radius.
    async fn relay_broadcast(
        &mut self,
        original: &[u8],
        header: &NwkHeader,
    ) -> Result<(), NwkStatus> {
        let new_radius = header.radius.saturating_sub(1);
        if new_radius == 0 {
            return Ok(());
        }

        let mut relay_buf = [0u8; 128];
        let mut new_header = header.clone();
        new_header.radius = new_radius;
        let hdr_len = new_header.serialize(&mut relay_buf);

        let (_, orig_hdr_len) = match NwkHeader::parse(original) {
            Some(parsed) => parsed,
            None => return Err(NwkStatus::InvalidParameter),
        };
        let payload = &original[orig_hdr_len..];
        if hdr_len + payload.len() > relay_buf.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        relay_buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
        let total = hdr_len + payload.len();

        log::debug!(
            "[NWK] Relaying broadcast from 0x{:04X} (radius {} → {})",
            header.src_addr.0,
            header.radius,
            new_radius
        );

        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, ShortAddress::BROADCAST),
                payload: &relay_buf[..total],
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: false, // No ACK for broadcast
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::RouteError)?;

        Ok(())
    }

    /// Resolve the MAC next hop for a given NWK destination.
    ///
    /// Strategy:
    /// 1. If destination is a neighbor → send directly
    /// 2. If destination is in routing table → use next_hop
    /// 3. If we're an end device → send to parent
    /// 4. For broadcast → send to all neighbors (simplified: send to parent)
    pub(crate) fn resolve_next_hop(
        &self,
        destination: ShortAddress,
    ) -> Result<ShortAddress, NwkStatus> {
        // Broadcast: send to parent (end device) or all neighbors (router)
        if destination.0 >= 0xFFF8 {
            if self.device_type == DeviceType::EndDevice {
                return Ok(self.nib.parent_address);
            }
            // Routers broadcast via MAC broadcast
            return Ok(ShortAddress::BROADCAST);
        }

        // Direct neighbor?
        if self.neighbors.find_by_short(destination).is_some() {
            return Ok(destination);
        }

        // Routing table lookup
        if let Some(next) = self.routing.next_hop(destination) {
            return Ok(next);
        }

        // End device fallback: always route through parent
        if self.device_type == DeviceType::EndDevice {
            return Ok(self.nib.parent_address);
        }

        // Tree routing fallback
        if let Some(next) = self.routing.tree_route(
            self.nib.network_address,
            destination,
            self.nib.depth,
            self.nib.max_routers,
            self.nib.max_depth,
        ) {
            return Ok(next);
        }

        // Route to parent as last resort
        if self.nib.parent_address.0 != 0xFFFF {
            Ok(self.nib.parent_address)
        } else {
            Err(NwkStatus::RouteError)
        }
    }

    // ── NWK Command Dispatch ─────────────────────────────────

    /// Dispatch an incoming NWK command frame to the appropriate handler.
    fn dispatch_nwk_command(&mut self, src: ShortAddress, payload: &[u8]) {
        if payload.is_empty() {
            log::warn!("[NWK] Empty NWK command payload from 0x{:04X}", src.0);
            return;
        }

        let cmd_id_byte = payload[0];
        let cmd_payload = &payload[1..];

        match NwkCommandId::from_u8(cmd_id_byte) {
            Some(NwkCommandId::Leave) => self.handle_nwk_leave(src, cmd_payload),
            Some(NwkCommandId::RouteRequest) => self.handle_route_request(src, cmd_payload),
            Some(NwkCommandId::RouteReply) => self.handle_route_reply(src, cmd_payload),
            Some(NwkCommandId::RouteRecord) => self.handle_route_record(src, cmd_payload),
            Some(NwkCommandId::LinkStatus) => self.handle_link_status(src, cmd_payload),
            Some(NwkCommandId::NetworkStatus) => self.handle_network_status(src, cmd_payload),
            Some(NwkCommandId::EdTimeoutResponse) => {
                if let Some(resp) = crate::frames::EdTimeoutResponse::parse(cmd_payload) {
                    log::info!(
                        "[NWK] ED Timeout Response from 0x{:04X}: status={} parent_info=0x{:02X}",
                        src.0,
                        resp.status,
                        resp.parent_info,
                    );
                }
            }
            Some(other) => {
                log::debug!(
                    "[NWK] Ignoring NWK command {:?} from 0x{:04X}",
                    other,
                    src.0
                );
            }
            None => {
                log::warn!(
                    "[NWK] Unknown NWK command ID 0x{:02X} from 0x{:04X}",
                    cmd_id_byte,
                    src.0
                );
            }
        }
    }

    // ── NWK Command Handlers ─────────────────────────────────

    /// Handle incoming NWK Leave command.
    fn handle_nwk_leave(&mut self, src: ShortAddress, payload: &[u8]) {
        let Some(leave) = crate::frames::LeaveCommand::parse(payload) else {
            log::warn!("[NWK] Malformed Leave command from 0x{:04X}", src.0);
            return;
        };

        log::info!(
            "[NWK] Leave from 0x{:04X} (remove_children={}, rejoin={})",
            src.0,
            leave.remove_children,
            leave.rejoin
        );

        if leave.remove_children {
            // We are being asked to leave the network
            log::warn!(
                "[NWK] Received leave-with-remove-children from 0x{:04X}",
                src.0
            );
            self.joined = false;
        }

        // Remove the leaving device from our neighbor table
        self.neighbors.remove(src);
    }

    /// Handle incoming Route Request (RREQ).
    fn handle_route_request(&mut self, src: ShortAddress, payload: &[u8]) {
        let Some(rreq) = crate::frames::RouteRequest::parse(payload) else {
            log::warn!("[NWK] Malformed RREQ from 0x{:04X}", src.0);
            return;
        };

        let is_many_to_one = rreq.command_options & 0x08 != 0;

        log::debug!(
            "[NWK] RREQ from 0x{:04X}: id={}, dst=0x{:04X}, cost={}, m2o={}",
            src.0,
            rreq.route_request_id,
            rreq.dst_addr.0,
            rreq.path_cost,
            is_many_to_one,
        );

        let our_addr = self.nib.network_address;

        // ── Many-to-one RREQ: install route to concentrator, rebroadcast, no RREP ──
        if is_many_to_one {
            // Determine concentrator type from RREQ command_options bits 3-4:
            // bit 3 set, bit 4 clear = LowRam (0x08)
            // bit 3 set, bit 4 set = HighRam (0x18)
            let conc_type = if rreq.command_options & 0x10 != 0 {
                crate::routing::ConcentratorType::HighRam
            } else {
                crate::routing::ConcentratorType::LowRam
            };

            // Install route to the concentrator (RREQ originator = dst_addr in RREQ)
            // via the sender
            let _ = self.routing.update_route_many_to_one(
                rreq.dst_addr,
                src,
                rreq.path_cost,
                conc_type,
            );

            log::info!(
                "[NWK] Many-to-one route installed: concentrator=0x{:04X} via 0x{:04X}",
                rreq.dst_addr.0,
                src.0,
            );

            // Rebroadcast if we're a router (no RREP for many-to-one)
            if self.device_type != DeviceType::EndDevice {
                let link_cost = self
                    .neighbors
                    .find_by_short(src)
                    .map(|n| n.outgoing_cost)
                    .unwrap_or(7);
                let new_cost = rreq.path_cost.saturating_add(link_cost);

                let _ = self
                    .pending_rreq_rebroadcasts
                    .push(crate::PendingRreqRebroadcast {
                        command_options: rreq.command_options,
                        route_request_id: rreq.route_request_id,
                        dst_addr: rreq.dst_addr,
                        path_cost: new_cost,
                    });
            }
            return;
        }

        // ── Standard RREQ handling ──
        // If destination is us, or we have a route, we can reply
        let have_route =
            rreq.dst_addr == our_addr || self.routing.next_hop(rreq.dst_addr).is_some();

        if have_route {
            // Record route discovery and complete it
            let _ = self.routing.add_discovery(crate::routing::RouteDiscovery {
                request_id: rreq.route_request_id,
                destination: rreq.dst_addr,
                sender: src,
                forward_cost: rreq.path_cost,
                residual_cost: 0,
                timestamp: 0,
                active: true,
            });
            self.routing.complete_discovery(rreq.route_request_id);

            // Update route back to the originator via the sender
            let _ = self.routing.update_route(src, src, rreq.path_cost);

            // Queue RREP to be sent asynchronously back toward the RREQ originator
            let responder = if rreq.dst_addr == our_addr {
                our_addr
            } else {
                rreq.dst_addr
            };
            let _ = self.pending_route_replies.push(crate::PendingRouteReply {
                next_hop: src,
                originator: src,
                responder,
                path_cost: rreq.path_cost,
                route_request_id: rreq.route_request_id,
            });
            log::info!(
                "[NWK] RREQ destination 0x{:04X} reachable — RREP queued",
                rreq.dst_addr.0
            );
        } else if self.device_type != DeviceType::EndDevice {
            // Router: record discovery and rebroadcast with incremented cost
            let link_cost = self
                .neighbors
                .find_by_short(src)
                .map(|n| n.outgoing_cost)
                .unwrap_or(7);
            let new_cost = rreq.path_cost.saturating_add(link_cost);

            let _ = self.routing.add_discovery(crate::routing::RouteDiscovery {
                request_id: rreq.route_request_id,
                destination: rreq.dst_addr,
                sender: src,
                forward_cost: new_cost,
                residual_cost: 0xFF,
                timestamp: 0,
                active: true,
            });

            log::debug!(
                "[NWK] Rebroadcasting RREQ for 0x{:04X} with cost {}",
                rreq.dst_addr.0,
                new_cost
            );

            // Queue RREQ rebroadcast (async send happens in process_pending_routing)
            let _ = self
                .pending_rreq_rebroadcasts
                .push(crate::PendingRreqRebroadcast {
                    command_options: rreq.command_options,
                    route_request_id: rreq.route_request_id,
                    dst_addr: rreq.dst_addr,
                    path_cost: new_cost,
                });
        }
    }

    /// Handle incoming Route Reply (RREP).
    fn handle_route_reply(&mut self, src: ShortAddress, payload: &[u8]) {
        let Some(rrep) = crate::frames::RouteReply::parse(payload) else {
            log::warn!("[NWK] Malformed RREP from 0x{:04X}", src.0);
            return;
        };

        log::debug!(
            "[NWK] RREP from 0x{:04X}: id={}, orig=0x{:04X}, resp=0x{:04X}, cost={}",
            src.0,
            rrep.route_request_id,
            rrep.originator.0,
            rrep.responder.0,
            rrep.path_cost
        );

        // Update routing table: route to responder via the sender
        let _ = self
            .routing
            .update_route(rrep.responder, src, rrep.path_cost);

        // Complete the route discovery
        self.routing.complete_discovery(rrep.route_request_id);

        let our_addr = self.nib.network_address;

        if rrep.originator != our_addr {
            // Not the originator — forward RREP toward originator via routing
            let forward_hop = self
                .routing
                .next_hop(rrep.originator)
                .unwrap_or(self.nib.parent_address);
            let _ = self.pending_route_replies.push(crate::PendingRouteReply {
                next_hop: forward_hop,
                originator: rrep.originator,
                responder: rrep.responder,
                path_cost: rrep.path_cost,
                route_request_id: rrep.route_request_id,
            });
            log::debug!(
                "[NWK] Forwarding RREP toward originator 0x{:04X} via 0x{:04X}",
                rrep.originator.0,
                forward_hop.0,
            );
        } else {
            log::info!(
                "[NWK] Route discovered to 0x{:04X} via 0x{:04X} (cost={})",
                rrep.responder.0,
                src.0,
                rrep.path_cost
            );
        }
    }

    /// Handle incoming Route Record.
    fn handle_route_record(&mut self, src: ShortAddress, payload: &[u8]) {
        if payload.is_empty() {
            log::warn!("[NWK] Malformed RouteRecord from 0x{:04X}", src.0);
            return;
        }

        let relay_count = payload[0] as usize;
        let expected_len = 1 + relay_count * 2;
        if payload.len() < expected_len {
            log::warn!(
                "[NWK] RouteRecord too short from 0x{:04X}: need {}, have {}",
                src.0,
                expected_len,
                payload.len()
            );
            return;
        }

        // Parse the full relay list from the payload
        let mut relay_list: heapless::Vec<
            ShortAddress,
            { crate::routing::MAX_SOURCE_ROUTE_RELAYS },
        > = heapless::Vec::new();
        for i in 0..relay_count.min(crate::routing::MAX_SOURCE_ROUTE_RELAYS) {
            let offset = 1 + i * 2;
            let addr = u16::from_le_bytes([payload[offset], payload[offset + 1]]);
            let _ = relay_list.push(ShortAddress(addr));
        }

        log::debug!(
            "[NWK] RouteRecord from 0x{:04X}: {} relays {:?}",
            src.0,
            relay_count,
            relay_list.as_slice(),
        );

        // Store the full relay path in the source route table (for concentrator TX)
        self.source_route_table.insert(src, relay_list.as_slice());

        // Also update the regular routing table with first-hop next hop
        if relay_count > 0 {
            let first_relay = relay_list[0];
            let _ = self
                .routing
                .update_route(src, first_relay, relay_count as u8);
        } else {
            // Direct neighbor, no relays
            let _ = self.routing.update_route(src, src, 0);
        }
    }

    /// Handle incoming Link Status command.
    fn handle_link_status(&mut self, src: ShortAddress, payload: &[u8]) {
        let Some(ls) = crate::frames::LinkStatusCommand::parse(payload) else {
            log::warn!("[NWK] Malformed LinkStatus from 0x{:04X}", src.0);
            return;
        };

        log::debug!(
            "[NWK] LinkStatus from 0x{:04X}: {} entries",
            src.0,
            ls.entries.len()
        );

        // Check if any entry references us, and update the neighbor's cost
        let our_addr = self.nib.network_address;
        for entry in &ls.entries {
            if entry.address == our_addr {
                // This neighbor reports its cost to/from us
                if let Some(neighbor) = self.neighbors.find_by_short_mut(src) {
                    neighbor.outgoing_cost = entry.incoming_cost.clamp(1, 7);
                    log::debug!(
                        "[NWK] Updated link cost to 0x{:04X}: outgoing={}",
                        src.0,
                        neighbor.outgoing_cost
                    );
                }
                break;
            }
        }
    }

    /// Handle incoming Network Status command (route error notification).
    fn handle_network_status(&mut self, src: ShortAddress, payload: &[u8]) {
        let Some(ns) = crate::frames::NetworkStatusCommand::parse(payload) else {
            log::warn!("[NWK] Malformed NetworkStatus from 0x{:04X}", src.0);
            return;
        };

        log::info!(
            "[NWK] NetworkStatus from 0x{:04X}: code={} dst=0x{:04X}",
            src.0,
            ns.status_code,
            ns.destination.0,
        );

        // If a route to the failed destination exists, remove it
        self.routing.remove(ns.destination);
    }
}

/// Process a source route relay list to determine the next hop.
///
/// The relay list is ordered `[closest-to-dest, ..., closest-to-source]`.
/// `relay_index` starts at `relay_count - 1` (set by the originator) and is
/// decremented at each hop. The relay at `relay_list[relay_index]` is the
/// current node; the next hop is `relay_list[relay_index - 1]`, or
/// `dst_addr` when `relay_index == 0`.
///
/// Returns `(next_hop, new_relay_index)`.
fn process_source_route(
    sr: &crate::frames::SourceRoute,
    our_addr: ShortAddress,
    dst_addr: ShortAddress,
) -> Result<(ShortAddress, u8), NwkStatus> {
    let idx = sr.relay_index as usize;

    // Validate that relay_index is within bounds
    if idx >= sr.relay_list.len() {
        log::warn!(
            "[NWK] Source route relay_index {} out of bounds (len={})",
            idx,
            sr.relay_list.len(),
        );
        return Err(NwkStatus::InvalidParameter);
    }

    // Verify we are the expected relay at this index
    if sr.relay_list[idx] != our_addr {
        // Try to find ourselves elsewhere in the list
        if let Some(found_idx) = sr.relay_list.iter().position(|&a| a == our_addr) {
            // Use the found position instead
            if found_idx == 0 {
                return Ok((dst_addr, 0));
            }
            return Ok((sr.relay_list[found_idx - 1], (found_idx - 1) as u8));
        }

        log::warn!(
            "[NWK] Our addr 0x{:04X} not found in source route relay list",
            our_addr.0,
        );
        return Err(NwkStatus::InvalidParameter);
    }

    if idx == 0 {
        // We are the last relay — forward directly to destination
        Ok((dst_addr, 0))
    } else {
        // Forward to the next relay (one step closer to destination)
        Ok((sr.relay_list[idx - 1], (idx - 1) as u8))
    }
}
