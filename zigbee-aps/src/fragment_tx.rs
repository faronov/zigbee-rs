//! Transmit side of APS fragmentation (Zigbee PRO R22 §2.2.8.4.5.1).
//!
//! One fragmented transaction is in flight at a time. Its blocks share one
//! APS counter; the first carries the total block count, later ones their
//! block number. Blocks are sent a transmission window
//! (`apsMaxWindowSize` blocks) at a time, separated by `apsInterframeDelay`.
//! The receiver acknowledges a window with an ACK whose bitfield names the
//! blocks it holds:
//!
//! - an ACK that acknowledges at least one new block resets the retry
//!   counter; once the window is complete it advances by `apsMaxWindowSize`
//!   and the next window is sent, otherwise the lowest unacknowledged block
//!   is resent;
//! - when `apscAckWaitDuration` expires the lowest unacknowledged block is
//!   resent and the retry counter incremented; at `apscMaxFrameRetries` the
//!   transaction fails with `NO_ACK`;
//! - when every block is acknowledged the transaction succeeds.
//!
//! The final result is reported by [`ApsLayer::take_fragmented_tx_confirm`].
//! An APS-secured transaction secures every block independently with a fresh
//! frame counter (R22 §4.4.1.1) and is only advanced by an APS-secured ACK.

use zigbee_mac::MacDriver;
use zigbee_types::{IeeeAddress, ShortAddress};

use crate::apsde::{ApsdeDataConfirm, ApsdeDataRequest};
use crate::fragment::{APS_MAX_FRAGMENT_BLOCKS, APS_MAX_FRAGMENTED_ASDU, clamp_window_size};
use crate::frames::{
    ApsDeliveryMode, ApsExtendedHeader, ApsFrameControl, ApsFrameType, ApsHeader, FRAG_FIRST,
    FRAG_SUBSEQUENT,
};
use crate::{APS_ACK_WAIT_DURATION_US, ApsLayer, ApsStatus};

/// ASDU octets per block of an unsecured fragmented transaction.
///
/// Header of a fragment: frame control, endpoints, cluster, profile, counter
/// and the two-octet extended header (10 octets) — 74 octets of APS frame,
/// which fits a NWK-secured unicast with room for NWK source-route and IEEE
/// address fields.
pub const APS_FRAGMENT_BLOCK_SIZE: usize = 64;
/// ASDU octets per block when each block also carries APS security
/// (14-octet auxiliary header with extended nonce + 4-octet MIC).
pub const APS_SECURED_FRAGMENT_BLOCK_SIZE: usize = 48;

/// APS security of a fragmented transaction.
#[derive(Debug, Clone, Copy)]
struct FragmentTxSecurity {
    origin: crate::security::ApsKeyOrigin,
    src_ieee: IeeeAddress,
}

/// State of the outgoing fragmented transaction.
pub(crate) struct FragmentTxSession {
    dst: ShortAddress,
    confirm: ApsdeDataConfirm,
    dst_endpoint: u8,
    src_endpoint: u8,
    cluster_id: u16,
    profile_id: u16,
    radius: u8,
    use_nwk_key: bool,
    security: Option<FragmentTxSecurity>,
    payload: [u8; APS_MAX_FRAGMENTED_ASDU],
    len: u16,
    block_size: u8,
    total: u8,
    window_size: u8,
    window_start: u8,
    /// Bit *n* = block *n* acknowledged.
    acked: u16,
    /// Bit *n* = block *n* must be (re)sent.
    due: u16,
    retries: u8,
    max_retries: u8,
    /// Waiting for an acknowledgement since `waiting_since_us`.
    waiting: bool,
    waiting_since_us: u32,
}

impl FragmentTxSession {
    fn window_end(&self) -> u8 {
        self.window_start
            .saturating_add(self.window_size)
            .min(self.total)
    }

    fn window_mask(&self) -> u16 {
        (self.window_start..self.window_end()).fold(0u16, |mask, block| mask | (1u16 << block))
    }

    fn all_mask(&self) -> u16 {
        if self.total >= 16 {
            u16::MAX
        } else {
            (1u16 << self.total) - 1
        }
    }

    fn lowest_unacked_in_window(&self) -> Option<u8> {
        (self.window_start..self.window_end()).find(|block| self.acked & (1u16 << block) == 0)
    }

    fn block(&self, block: u8) -> &[u8] {
        let size = usize::from(self.block_size);
        let start = usize::from(block) * size;
        let end = (start + size).min(usize::from(self.len));
        &self.payload[start..end]
    }

    fn header(&self, block: u8) -> ApsHeader {
        ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Data as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: self.security.is_some(),
                ack_request: true,
                extended_header: true,
            },
            dst_endpoint: Some(self.dst_endpoint),
            group_address: None,
            cluster_id: Some(self.cluster_id),
            profile_id: Some(self.profile_id),
            src_endpoint: Some(self.src_endpoint),
            aps_counter: self.confirm.aps_counter,
            extended_header: Some(ApsExtendedHeader {
                fragmentation: if block == 0 {
                    FRAG_FIRST
                } else {
                    FRAG_SUBSEQUENT
                },
                block_number: if block == 0 { self.total } else { block },
                ack_bitfield: None,
            }),
        }
    }

    /// Apply a windowed acknowledgement. Returns whether any block was newly
    /// acknowledged.
    fn apply_ack(&mut self, window_start: u8, bitfield: u8) -> bool {
        if window_start != self.window_start {
            return false;
        }
        let mut newly = 0u16;
        for bit in 0..self.window_size {
            let block = self.window_start.saturating_add(bit);
            if block >= self.total {
                break;
            }
            if bitfield & (1 << bit) != 0 && self.acked & (1u16 << block) == 0 {
                newly |= 1u16 << block;
            }
        }
        if newly == 0 {
            return false;
        }
        self.acked |= newly;
        self.due &= !self.acked;
        self.retries = 0;
        self.waiting = false;
        let window = self.window_mask();
        if self.acked & window == window && self.window_end() < self.total {
            // Window complete: advance and send the whole next window.
            self.window_start = self.window_end();
            self.due |= self.window_mask() & !self.acked;
        } else if let Some(block) = self.lowest_unacked_in_window() {
            self.due |= 1u16 << block;
        }
        true
    }

    fn complete(&self) -> bool {
        self.acked == self.all_mask()
    }
}

impl<M: MacDriver> ApsLayer<M> {
    /// Start a fragmented APSDE-DATA transaction (R22 §2.2.8.4.5.1) and send
    /// its first transmission window.
    ///
    /// Fragmentation is defined for acknowledged unicast only; anything else
    /// — and an ASDU beyond [`APS_MAX_FRAGMENTED_ASDU`] — is refused with
    /// `ASDU_TOO_LONG`. While a transaction is in flight a second one is
    /// refused with `INSUFFICIENT_SPACE`.
    ///
    /// `Ok` means the transaction was accepted and its first window handed to
    /// the NWK layer; the delivery result is reported later by
    /// [`Self::take_fragmented_tx_confirm`].
    pub(crate) async fn start_fragmented_tx(
        &mut self,
        req: &ApsdeDataRequest<'_>,
        nwk_dst: ShortAddress,
        delivery_mode: ApsDeliveryMode,
        radius: u8,
    ) -> Result<ApsdeDataConfirm, ApsStatus> {
        if delivery_mode != ApsDeliveryMode::Unicast
            || !req.tx_options.ack_request
            || req.payload.len() > APS_MAX_FRAGMENTED_ASDU
        {
            return Err(ApsStatus::AsduTooLong);
        }
        if self.fragment_tx.is_some() {
            return Err(ApsStatus::InsufficientSpace);
        }
        let security = if req.tx_options.security_enabled {
            let dst_ieee = self.nwk.find_ieee_by_short(nwk_dst);
            let origin = self
                .aps_data_key_origin(dst_ieee.as_ref(), nwk_dst)
                .ok_or(ApsStatus::SecurityFail)?;
            Some(FragmentTxSecurity {
                origin,
                src_ieee: self.nwk.nib().ieee_address,
            })
        } else {
            None
        };
        let block_size = if security.is_some() {
            APS_SECURED_FRAGMENT_BLOCK_SIZE
        } else {
            APS_FRAGMENT_BLOCK_SIZE
        };
        let total = req.payload.len().div_ceil(block_size);
        if total > usize::from(APS_MAX_FRAGMENT_BLOCKS) || total < 2 {
            return Err(ApsStatus::AsduTooLong);
        }

        let aps_counter = self.next_aps_counter();
        let mut payload = [0u8; APS_MAX_FRAGMENTED_ASDU];
        payload[..req.payload.len()].copy_from_slice(req.payload);
        let mut session = FragmentTxSession {
            dst: nwk_dst,
            confirm: ApsdeDataConfirm {
                status: ApsStatus::Success,
                dst_addr_mode: req.dst_addr_mode,
                dst_address: req.dst_address,
                dst_endpoint: req.dst_endpoint,
                src_endpoint: req.src_endpoint,
                aps_counter,
            },
            dst_endpoint: req.dst_endpoint,
            src_endpoint: req.src_endpoint,
            cluster_id: req.cluster_id,
            profile_id: req.profile_id,
            radius,
            use_nwk_key: req.tx_options.use_nwk_key,
            security,
            payload,
            len: req.payload.len() as u16,
            block_size: block_size as u8,
            total: total as u8,
            window_size: clamp_window_size(self.aib.aps_max_window_size),
            window_start: 0,
            acked: 0,
            due: 0,
            retries: 0,
            max_retries: self.aib.aps_max_frame_retries,
            waiting: false,
            waiting_since_us: self.nwk.mac().monotonic_micros(),
        };
        session.due = session.window_mask();
        self.fragment_tx = Some(session);
        self.fragment_tx_confirm = None;

        // The first block must reach the NWK layer, otherwise the request
        // fails synchronously like an unfragmented one.
        let Some((dst, use_nwk_key, radius, frame)) = self.next_due_fragment() else {
            self.drop_fragment_tx();
            return Err(ApsStatus::SecurityFail);
        };
        if let Err(error) = self
            .nwk
            .nlde_data_request(dst, radius, &frame, use_nwk_key, true)
            .await
        {
            self.drop_fragment_tx();
            return Err(crate::apsde::nwk_status_to_aps(error));
        }
        self.send_due_fragments().await;
        Ok(ApsdeDataConfirm {
            status: ApsStatus::Success,
            dst_addr_mode: req.dst_addr_mode,
            dst_address: req.dst_address,
            dst_endpoint: req.dst_endpoint,
            src_endpoint: req.src_endpoint,
            aps_counter,
        })
    }

    fn drop_fragment_tx(&mut self) {
        if let Some(mut session) = self.fragment_tx.take() {
            crate::zeroize(&mut session.payload);
        }
    }

    fn finish_fragment_tx(&mut self, status: ApsStatus) {
        if let Some(session) = self.fragment_tx.as_ref() {
            let mut confirm = session.confirm;
            confirm.status = status;
            log::debug!(
                "[APS frag] transaction counter={} to 0x{:04X} finished: {:?}",
                confirm.aps_counter,
                session.dst.0,
                status
            );
            self.fragment_tx_confirm = Some(confirm);
        }
        self.drop_fragment_tx();
    }

    /// Build the lowest due block and mark it sent. Starts the
    /// acknowledgement timer once no block of the window is left to send.
    pub(crate) fn next_due_fragment(
        &mut self,
    ) -> Option<(ShortAddress, bool, u8, heapless::Vec<u8, 128>)> {
        let session = self.fragment_tx.as_ref()?;
        if session.due == 0 {
            return None;
        }
        let block = session.due.trailing_zeros() as u8;
        let header = session.header(block);
        let security = session.security;
        let (dst, use_nwk_key, radius) = (session.dst, session.use_nwk_key, session.radius);
        let mut header_buf = [0u8; 16];
        let header_len = header.serialize(&mut header_buf);

        let frame = match security {
            None => {
                let session = self.fragment_tx.as_ref()?;
                let chunk = session.block(block);
                let mut frame = heapless::Vec::<u8, 128>::new();
                frame.extend_from_slice(&header_buf[..header_len]).ok()?;
                frame.extend_from_slice(chunk).ok()?;
                Some(frame)
            }
            Some(security) => {
                let mut chunk = [0u8; APS_SECURED_FRAGMENT_BLOCK_SIZE];
                let chunk_len = {
                    let block_data = self.fragment_tx.as_ref()?.block(block);
                    chunk[..block_data.len()].copy_from_slice(block_data);
                    block_data.len()
                };
                let frame =
                    self.secure_fragment(&header_buf[..header_len], &chunk[..chunk_len], &security);
                crate::zeroize(&mut chunk);
                frame
            }
        };
        let Some(frame) = frame else {
            // No key or counter left: the transaction cannot continue.
            log::error!("[APS frag] cannot secure block {}; aborting", block);
            self.finish_fragment_tx(ApsStatus::SecurityFail);
            return None;
        };

        let now = self.nwk.mac().monotonic_micros();
        let session = self.fragment_tx.as_mut()?;
        session.due &= !(1u16 << block);
        if session.due == 0 {
            session.waiting = true;
            session.waiting_since_us = now;
        }
        Some((dst, use_nwk_key, radius, frame))
    }

    fn secure_fragment(
        &mut self,
        header: &[u8],
        chunk: &[u8],
        security: &FragmentTxSecurity,
    ) -> Option<heapless::Vec<u8, 128>> {
        let key = self.security.key_for_origin(&security.origin)?;
        let frame_counter = self.next_frame_counter_for(&security.origin)?;
        let sec_hdr = crate::security::ApsSecurityHeader {
            security_control: crate::security::ApsSecurityHeader::APS_DEFAULT_EXT_NONCE,
            frame_counter,
            source_address: Some(security.src_ieee),
            key_seq_number: None,
        };
        let mut key = key;
        let frame = self.assemble_secured_frame(header, chunk, &key, &sec_hdr);
        crate::zeroize(&mut key);
        frame
    }

    /// Send every due block, `apsInterframeDelay` apart.
    async fn send_due_fragments(&mut self) {
        while let Some((dst, use_nwk_key, radius, frame)) = {
            let pending = self
                .fragment_tx
                .as_ref()
                .is_some_and(|session| session.due != 0);
            if pending {
                let delay_us = u32::from(self.aib.aps_interframe_delay) * 1000;
                self.nwk.mac_mut().delay_micros(delay_us).await;
                self.next_due_fragment()
            } else {
                None
            }
        } {
            // A lost block is recovered by the acknowledgement timer.
            let _ = self
                .nwk
                .nlde_data_request(dst, radius, &frame, use_nwk_key, true)
                .await;
        }
    }

    /// Run the acknowledgement timer of the fragmented transaction.
    pub(crate) fn poll_fragment_tx_timeout(&mut self) {
        let now = self.nwk.mac().monotonic_micros();
        let Some(session) = self.fragment_tx.as_mut() else {
            return;
        };
        if !session.waiting || now.wrapping_sub(session.waiting_since_us) < APS_ACK_WAIT_DURATION_US
        {
            return;
        }
        if session.retries >= session.max_retries {
            log::warn!(
                "[APS frag] no acknowledgement for counter={} after {} retries",
                session.confirm.aps_counter,
                session.retries
            );
            self.finish_fragment_tx(ApsStatus::NoAck);
            return;
        }
        session.retries += 1;
        session.waiting = false;
        if let Some(block) = session.lowest_unacked_in_window() {
            session.due |= 1u16 << block;
        }
    }

    /// Handle an incoming windowed acknowledgement. Returns whether it
    /// belonged to the fragmented transaction in flight.
    pub(crate) fn handle_fragment_ack(
        &mut self,
        src: ShortAddress,
        aps_counter: u8,
        ext: &ApsExtendedHeader,
        aps_secured: bool,
    ) -> bool {
        let Some(session) = self.fragment_tx.as_mut() else {
            return false;
        };
        if session.dst != src || session.confirm.aps_counter != aps_counter {
            return false;
        }
        // An APS-secured transaction is only advanced by an ACK the partner
        // secured with the link key; anyone holding just the network key
        // could forge an unsecured one.
        if session.security.is_some() && !aps_secured {
            log::warn!("[APS frag] ignoring unsecured ACK for an APS-secured transaction");
            return true;
        }
        let Some(bitfield) = ext.ack_bitfield else {
            return true;
        };
        if session.apply_ack(ext.block_number, bitfield) && session.complete() {
            self.finish_fragment_tx(ApsStatus::Success);
        }
        true
    }

    /// Drive the fragmented transaction: run its acknowledgement timer and
    /// send due blocks `apsInterframeDelay` apart.
    ///
    /// Call after processing received frames and from periodic maintenance.
    /// [`Self::age_ack_table`] performs the same work without the
    /// inter-frame delay, returning the blocks for the caller to send.
    pub async fn service_fragment_tx(&mut self) {
        self.poll_fragment_tx_timeout();
        self.send_due_fragments().await;
    }

    /// Whether a fragmented transaction is in flight.
    pub fn fragmented_tx_active(&self) -> bool {
        self.fragment_tx.is_some()
    }
}

#[cfg(all(test, feature = "router"))]
mod tests {
    use super::*;
    use crate::apsde::{ApsFrameBuffer, IncomingNwkSecurity};
    use crate::{ApsAddress, ApsAddressMode, ApsTxOptions};
    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use std::sync::Arc;
    use std::task::Wake;
    use zigbee_mac::PlatformServices;
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::{DeviceType, NwkLayer};
    use zigbee_types::PanId;

    const LOCAL_IEEE: IeeeAddress = [0x10; 8];
    const PEER_IEEE: IeeeAddress = [0x60; 8];
    const LOCAL: ShortAddress = ShortAddress(0x1111);
    const PEER: ShortAddress = ShortAddress(0x0000);
    const NETWORK_KEY: [u8; 16] = [0x31; 16];

    struct NoopWake;
    impl Wake for NoopWake {
        fn wake(self: Arc<Self>) {}
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(NoopWake));
        let mut context = Context::from_waker(&waker);
        let mut future = std::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
        }
    }

    fn node() -> ApsLayer<MockMac> {
        let mut nwk = NwkLayer::new(MockMac::new(LOCAL_IEEE), DeviceType::Router);
        nwk.set_joined(true);
        {
            let nib = nwk.nib_mut();
            nib.pan_id = PanId(0x1234);
            nib.network_address = LOCAL;
            nib.parent_address = ShortAddress::COORDINATOR;
            nib.ieee_address = LOCAL_IEEE;
            nib.security_enabled = true;
            nib.outgoing_frame_counter_limit = 0x1000;
        }
        nwk.security_mut().set_network_key(NETWORK_KEY, 0);
        ApsLayer::new(nwk)
    }

    fn options(security_enabled: bool, ack_request: bool) -> ApsTxOptions {
        ApsTxOptions {
            security_enabled,
            use_nwk_key: true,
            ack_request,
            fragmentation_permitted: true,
            include_extended_nonce: false,
        }
    }

    fn request<'a>(
        payload: &'a [u8],
        dst: ApsAddress,
        mode: ApsAddressMode,
        tx_options: ApsTxOptions,
    ) -> ApsdeDataRequest<'a> {
        ApsdeDataRequest {
            dst_addr_mode: mode,
            dst_address: dst,
            dst_endpoint: 1,
            profile_id: 0x0104,
            cluster_id: 0x0006,
            src_endpoint: 2,
            payload,
            tx_options,
            radius: 0,
            alias_src_addr: None,
            alias_seq: None,
        }
    }

    /// APS frames (NWK payloads) of every transmission so far.
    fn sent_aps_frames(aps: &ApsLayer<MockMac>) -> std::vec::Vec<std::vec::Vec<u8>> {
        aps.nwk()
            .mac()
            .tx_history()
            .iter()
            .map(|record| {
                let bytes = record.payload.as_slice();
                let (header, header_len) = zigbee_nwk::frames::NwkHeader::parse(bytes).unwrap();
                assert!(header.frame_control.security);
                let (sec, sec_len) =
                    zigbee_nwk::security::NwkSecurityHeader::parse(&bytes[header_len..]).unwrap();
                let aad_len = header_len + sec_len;
                let mut aad = [0u8; 64];
                aad[..aad_len].copy_from_slice(&bytes[..aad_len]);
                aad[header_len] = (aad[header_len] & !0x07) | 0x05;
                zigbee_nwk::security::NwkSecurity::new()
                    .decrypt(&aad[..aad_len], &bytes[aad_len..], &NETWORK_KEY, &sec)
                    .unwrap()
                    .to_vec()
            })
            .collect()
    }

    fn ack_frame(counter: u8, window_start: u8, bitfield: u8) -> heapless::Vec<u8, 32> {
        let header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Ack as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false,
                ack_request: false,
                extended_header: true,
            },
            dst_endpoint: Some(2),
            group_address: None,
            cluster_id: Some(0x0006),
            profile_id: Some(0x0104),
            src_endpoint: Some(1),
            aps_counter: counter,
            extended_header: Some(ApsExtendedHeader {
                fragmentation: FRAG_FIRST,
                block_number: window_start,
                ack_bitfield: Some(bitfield),
            }),
        };
        let mut buf = [0u8; 32];
        let len = header.serialize(&mut buf);
        heapless::Vec::from_slice(&buf[..len]).unwrap()
    }

    fn receive(aps: &mut ApsLayer<MockMac>, frame: &[u8]) {
        let mut buf = ApsFrameBuffer::new();
        let _ = aps.process_incoming_aps_frame(
            frame,
            PEER,
            LOCAL,
            200,
            IncomingNwkSecurity::new(true, Some(PEER_IEEE)),
            &mut buf,
        );
    }

    fn advance(aps: &mut ApsLayer<MockMac>, micros: u32) {
        block_on(aps.nwk_mut().mac_mut().delay_micros(micros));
    }

    fn payload() -> [u8; 200] {
        core::array::from_fn(|index| index as u8)
    }

    /// R22 §2.2.8.4.5.1 with window size 2 and 4 blocks: the window is sent,
    /// an ACK advances it, and the final ACK confirms SUCCESS.
    #[test]
    fn windows_advance_on_acknowledgement_and_complete_with_success() {
        let mut aps = node();
        aps.aib_mut().aps_max_window_size = 2;
        let data = payload();
        let confirm = block_on(aps.apsde_data_request(&request(
            &data,
            ApsAddress::Short(PEER),
            ApsAddressMode::Short,
            options(false, true),
        )))
        .unwrap();
        let counter = confirm.aps_counter;
        let frames = sent_aps_frames(&aps);
        assert_eq!(frames.len(), 2, "first window only");
        // Block 0: FC data|AR|ext, ..., counter, ext FC first, total 4.
        assert_eq!(frames[0][0], 0xC0);
        assert_eq!(&frames[0][8..10], &[0x01, 4]);
        assert_eq!(&frames[0][10..], &data[..64]);
        assert_eq!(&frames[1][8..10], &[0x02, 1]);
        assert_eq!(&frames[1][10..], &data[64..128]);
        assert!(aps.has_pending_ack());
        // Fragment windows are driven by the CPU, not by a parent poll.
        assert!(!aps.awaits_unicast_ack_only());

        // A partial ACK (block 1 missing) resends block 1 only.
        receive(&mut aps, &ack_frame(counter, 0, 0b1111_1101));
        block_on(aps.service_fragment_tx());
        let frames = sent_aps_frames(&aps);
        assert_eq!(frames.len(), 3);
        assert_eq!(&frames[2][8..10], &[0x02, 1]);

        // Window [0,1] complete → window [2,3] is sent.
        receive(&mut aps, &ack_frame(counter, 0, 0xFF));
        block_on(aps.service_fragment_tx());
        let frames = sent_aps_frames(&aps);
        assert_eq!(frames.len(), 5);
        assert_eq!(&frames[3][8..10], &[0x02, 2]);
        assert_eq!(&frames[4][8..10], &[0x02, 3]);
        assert_eq!(&frames[4][10..], &data[192..]);
        assert!(aps.take_fragmented_tx_confirm().is_none());

        // An ACK for the wrong window is ignored.
        receive(&mut aps, &ack_frame(counter, 0, 0xFF));
        assert!(aps.fragmented_tx_active());
        receive(&mut aps, &ack_frame(counter, 2, 0xFF));
        let done = aps
            .take_fragmented_tx_confirm()
            .expect("transaction finished");
        assert_eq!(done.status, ApsStatus::Success);
        assert_eq!(done.aps_counter, counter);
        assert!(!aps.fragmented_tx_active());
        assert!(!aps.has_pending_ack());
        assert!(!aps.awaits_unicast_ack_only());
    }

    /// Without acknowledgements the lowest unacknowledged block is resent
    /// every `apscAckWaitDuration` until `apscMaxFrameRetries`, then NO_ACK.
    #[test]
    fn missing_acknowledgements_retry_then_fail_with_no_ack() {
        let mut aps = node();
        let data = payload();
        block_on(aps.apsde_data_request(&request(
            &data,
            ApsAddress::Short(PEER),
            ApsAddressMode::Short,
            options(false, true),
        )))
        .unwrap();
        assert_eq!(
            sent_aps_frames(&aps).len(),
            4,
            "window 8 covers all 4 blocks"
        );

        let retries = aps.aib().aps_max_frame_retries;
        for attempt in 0..retries {
            advance(&mut aps, APS_ACK_WAIT_DURATION_US - 1);
            block_on(aps.service_fragment_tx());
            assert_eq!(sent_aps_frames(&aps).len(), 4 + usize::from(attempt));
            advance(&mut aps, 1);
            // The runtime maintenance path (age_ack_table) emits the retry.
            let retransmissions = aps.age_ack_table();
            assert_eq!(retransmissions.len(), 1);
            assert_eq!(retransmissions[0].dst_addr, PEER);
            assert_eq!(
                &retransmissions[0].frame[8..10],
                &[0x01, 4],
                "block 0 again"
            );
            block_on(aps.nwk_mut().nlde_data_request(
                PEER,
                10,
                &retransmissions[0].frame,
                true,
                true,
            ))
            .unwrap();
        }
        advance(&mut aps, APS_ACK_WAIT_DURATION_US);
        assert!(aps.age_ack_table().is_empty());
        let done = aps
            .take_fragmented_tx_confirm()
            .expect("transaction failed");
        assert_eq!(done.status, ApsStatus::NoAck);
    }

    /// Fragmentation is only for acknowledged unicast (R22 §2.2.8.4.5).
    #[test]
    fn group_broadcast_and_unacknowledged_requests_are_too_long() {
        let mut aps = node();
        let data = payload();
        for (dst, mode, ack) in [
            (ApsAddress::Group(0x1234), ApsAddressMode::Group, false),
            (
                ApsAddress::Short(ShortAddress(0xFFFD)),
                ApsAddressMode::Short,
                false,
            ),
            (ApsAddress::Short(PEER), ApsAddressMode::Short, false),
        ] {
            assert_eq!(
                block_on(aps.apsde_data_request(&request(&data, dst, mode, options(false, ack))))
                    .err(),
                Some(ApsStatus::AsduTooLong)
            );
        }
        let oversize = [0u8; APS_MAX_FRAGMENTED_ASDU + 1];
        assert_eq!(
            block_on(aps.apsde_data_request(&request(
                &oversize,
                ApsAddress::Short(PEER),
                ApsAddressMode::Short,
                options(false, true),
            )))
            .err(),
            Some(ApsStatus::AsduTooLong)
        );
        assert!(aps.nwk().mac().tx_history().is_empty());
    }

    /// A second transaction while one is in flight is refused.
    #[test]
    fn only_one_fragmented_transaction_at_a_time() {
        let mut aps = node();
        let data = payload();
        let req = request(
            &data,
            ApsAddress::Short(PEER),
            ApsAddressMode::Short,
            options(false, true),
        );
        block_on(aps.apsde_data_request(&req)).unwrap();
        assert_eq!(
            block_on(aps.apsde_data_request(&req)).err(),
            Some(ApsStatus::InsufficientSpace)
        );
    }

    /// Every block of an APS-secured transaction is secured on its own with
    /// a fresh frame counter, and an unsecured ACK cannot complete it.
    #[test]
    fn secured_blocks_use_fresh_counters_and_need_a_secured_ack() {
        let mut aps = node();
        let key = [0x77; 16];
        aps.aib_mut().aps_trust_center_address = PEER_IEEE;
        aps.nwk_mut().update_neighbor_address(PEER, PEER_IEEE);
        aps.security_mut()
            .add_key(crate::security::ApsLinkKeyEntry {
                partner_address: PEER_IEEE,
                key,
                key_type: crate::security::ApsKeyType::TrustCenterLinkKey,
                outgoing_frame_counter: 10,
                outgoing_frame_counter_limit: 1000,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
            .unwrap();
        let data = payload();
        let confirm = block_on(aps.apsde_data_request(&request(
            &data,
            ApsAddress::Short(PEER),
            ApsAddressMode::Short,
            options(true, true),
        )))
        .unwrap();
        let frames = sent_aps_frames(&aps);
        assert_eq!(frames.len(), 5, "200 octets in 48-octet secured blocks");
        let mut counters = std::vec::Vec::new();
        for frame in &frames {
            let (header, header_len) = ApsHeader::parse(frame).unwrap();
            assert!(header.frame_control.security);
            let (sec, _) = crate::security::ApsSecurityHeader::parse(&frame[header_len..]).unwrap();
            counters.push(sec.frame_counter);
        }
        counters.dedup();
        assert_eq!(counters.len(), 5, "no frame counter is reused");

        receive(&mut aps, &ack_frame(confirm.aps_counter, 0, 0xFF));
        assert!(aps.fragmented_tx_active(), "unsecured ACK is ignored");
    }
}
