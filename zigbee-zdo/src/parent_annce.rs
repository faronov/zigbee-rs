//! R22 Parent Announce (ZDP clusters `Parent_annce` 0x001F and
//! `Parent_annce_rsp` 0x801F).
//!
//! `Parent_annce` is a **ZDP** (Zigbee Device Profile) primitive carried over
//! APS on endpoint 0, not a NWK-layer command. A router/coordinator that has
//! rebooted and is joined and authenticated broadcasts the IEEE addresses of
//! the **end-device** children it parents, so that any *former* parent of a
//! child that has since moved prunes its stale entry. A recipient that is
//! actively parenting one of the announced children replies with
//! `Parent_annce_rsp` listing those children, and the announcer then
//! relinquishes them.
//!
//! Only routers and coordinators have a child table, so end devices never send
//! it and never act on it (the receive path is gated on `can_route()`).
//!
//! # Exact R22 §2.4.3.1.12 / §2.4.4.2.22 behaviour implemented here
//!
//! - The broadcast destination is **0xFFFC** (all routers and the
//!   coordinator), not the 0xFFFD RxOnWhenIdle broadcast used by
//!   `Device_annce`.
//! - Only neighbour table entries whose Device Type is ZigBee End Device
//!   (0x02) are announced or reconciled; a router child is never touched.
//! - Each broadcast is preceded by `apsParentAnnounceTimer` =
//!   `apsParentAnnounceBaseTimer` (10 s) + a random value in
//!   `0..=apsParentAnnounceJitterMax` (10 s). A child table that needs more
//!   than one message re-arms a fresh jittered timer before **each** additional
//!   message.
//! - Receiving a broadcast `Parent_annce` while this device's own timer is
//!   running re-calculates and restarts that timer. This is the storm/loop
//!   guard: several routers rebooting together spread their broadcasts out
//!   instead of answering each other immediately.
//! - A keepalive received from an end device before an outstanding additional
//!   message has been sent removes that child from the message.
//! - A message constructed with `NumberOfChildren == 0` is discarded, never
//!   sent — including the response, which is unicast and needs no jitter.

use zigbee_mac::MacDriver;
use zigbee_types::IeeeAddress;
#[cfg(any(feature = "router", test))]
use zigbee_types::ShortAddress;

use crate::{BROADCAST_ROUTERS, PARENT_ANNCE, PARENT_ANNCE_RSP, ZdoError, ZdoLayer};
// `Parent_annce_rsp` is only ever *built* by a device with a child table.
#[cfg(feature = "router")]
use crate::ZdpStatus;

/// Largest child list carried in one Parent Announce / Response.
///
/// The response has the larger fixed overhead:
/// `TSN(1) + Status(1) + Count(1) + IEEE[8] * N`. Keeping that whole payload
/// within [`zigbee_aps::apsde::APS_MAX_PAYLOAD`] also keeps the request within
/// the same bound. Larger child tables are split across multiple broadcasts.
pub const MAX_PARENT_ANNCE_CHILDREN: usize =
    (zigbee_aps::apsde::APS_MAX_PAYLOAD - 3) / core::mem::size_of::<IeeeAddress>();

/// How long a broadcast Parent Announce accepts responses from any number of
/// current parents. The entry stays open after the first response because one
/// announced chunk can contain children that moved to different parents.
pub const PARENT_ANNCE_RESPONSE_WINDOW_SECS: u16 = 5;

/// `apsParentAnnounceBaseTimer` (R22 Table 2-23) — the base delay, in seconds,
/// before each broadcast Parent Announce is sent.
pub const APS_PARENT_ANNOUNCE_BASE_TIMER_SECS: u16 = 10;

/// `apsParentAnnounceJitterMax` (R22 Table 2-23) — the maximum jitter, in
/// seconds, added to the base timer before each broadcast.
pub const APS_PARENT_ANNOUNCE_JITTER_MAX_SECS: u16 = 10;

/// A bounded list of child IEEE addresses decoded from a Parent Announce.
pub type ChildList = heapless::Vec<IeeeAddress, MAX_PARENT_ANNCE_CHILDREN>;

/// Parse a `NumberOfChildren(1) || IEEE[8]*N` child list.
///
/// Rejects a declared count above [`MAX_PARENT_ANNCE_CHILDREN`] or a payload
/// too short for the declared count. Trailing bytes are ignored so a future
/// revision that appends fields still decodes.
pub fn parse_child_list(data: &[u8]) -> Option<ChildList> {
    let count = *data.first()? as usize;
    if count > MAX_PARENT_ANNCE_CHILDREN || data.len() < 1 + count * 8 {
        return None;
    }
    let mut list = ChildList::new();
    for index in 0..count {
        let start = 1 + index * 8;
        let mut ieee = [0u8; 8];
        ieee.copy_from_slice(&data[start..start + 8]);
        list.push(ieee).ok()?;
    }
    Some(list)
}

/// Serialize a `NumberOfChildren(1) || IEEE[8]*N` child list, returning the
/// encoded length.
pub fn serialize_child_list(children: &[IeeeAddress], buf: &mut [u8]) -> Result<usize, ZdoError> {
    let count = children.len();
    if count > MAX_PARENT_ANNCE_CHILDREN {
        return Err(ZdoError::InvalidLength);
    }
    let needed = 1 + count * 8;
    if buf.len() < needed {
        return Err(ZdoError::BufferTooSmall);
    }
    buf[0] = count as u8;
    for (index, ieee) in children.iter().enumerate() {
        let start = 1 + index * 8;
        buf[start..start + 8].copy_from_slice(ieee);
    }
    Ok(needed)
}

/// Serialize a `Parent_annce_rsp`: `Status(1) || NumberOfChildren(1) ||
/// IEEE[8]*N`.
pub fn serialize_parent_annce_rsp(
    status: u8,
    children: &[IeeeAddress],
    buf: &mut [u8],
) -> Result<usize, ZdoError> {
    if buf.is_empty() {
        return Err(ZdoError::BufferTooSmall);
    }
    buf[0] = status;
    let list_len = serialize_child_list(children, &mut buf[1..])?;
    Ok(1 + list_len)
}

/// Parse a `Parent_annce_rsp` into `(status, child list)`.
pub fn parse_parent_annce_rsp(data: &[u8]) -> Option<(u8, ChildList)> {
    let status = *data.first()?;
    let list = parse_child_list(data.get(1..)?)?;
    Some((status, list))
}

impl<M: MacDriver> ZdoLayer<M> {
    fn clear_pending_parent_annce(&mut self) {
        for slot in &mut self.pending_responses {
            if slot.active && slot.rsp_cluster == PARENT_ANNCE_RSP {
                slot.active = false;
                slot.completed = false;
                slot.remaining_secs = 0;
                slot.payload.clear();
            }
        }
    }

    fn register_pending_parent_annce(&mut self, tsn: u8) -> Result<usize, ZdoError> {
        let slot = self
            .register_pending(tsn, PARENT_ANNCE_RSP)
            .ok_or(ZdoError::TableFull)?;
        self.pending_responses[slot].remaining_secs = PARENT_ANNCE_RESPONSE_WINDOW_SECS;
        Ok(slot)
    }

    #[cfg(feature = "router")]
    fn accepts_parent_annce_rsp(&self, tsn: u8) -> bool {
        self.pending_responses.iter().any(|slot| {
            slot.active
                && slot.rsp_cluster == PARENT_ANNCE_RSP
                && slot.tsn == tsn
                && slot.remaining_secs > 0
        })
    }

    /// Age the one-to-many Parent Announce response collection windows.
    ///
    /// Unlike ordinary ZDP requests, a broadcast announcement can receive
    /// several valid responses with the same TSN. Entries therefore remain
    /// open until this timeout expires rather than closing on the first reply.
    pub fn tick_parent_annce_transactions(&mut self, elapsed_secs: u16) {
        for slot in &mut self.pending_responses {
            if !slot.active || slot.rsp_cluster != PARENT_ANNCE_RSP {
                continue;
            }
            slot.remaining_secs = slot.remaining_secs.saturating_sub(elapsed_secs);
            if slot.remaining_secs == 0 {
                slot.active = false;
                slot.completed = false;
                slot.payload.clear();
            }
        }
    }

    // ── apsParentAnnounceTimer (R22 §2.4.3.1.12) ────────────────

    #[cfg(feature = "router")]
    /// Current value of `apsParentAnnounceTimer`, in seconds (`0` = stopped).
    pub fn parent_annce_timer_secs(&self) -> u16 {
        self.parent_annce_timer_secs
    }

    #[cfg(feature = "router")]
    /// `apsParentAnnounceBaseTimer + random(0..=apsParentAnnounceJitterMax)`.
    ///
    /// The sample comes from the same bounded xorshift the NWK layer uses for
    /// its R22 broadcast jitter, seeded from the platform monotonic clock, so
    /// no allocation, floating point or external RNG enters this path.
    fn next_parent_annce_timer(&self) -> u16 {
        let seed = self.nwk().mac().monotonic_micros();
        let sample = zigbee_nwk::routing::routing_random_sample(seed);
        let jitter = (sample % u32::from(APS_PARENT_ANNOUNCE_JITTER_MAX_SECS + 1)) as u16;
        APS_PARENT_ANNOUNCE_BASE_TIMER_SECS + jitter
    }

    #[cfg(feature = "router")]
    /// Schedule a R22 Parent Announce.
    ///
    /// Called when the two generating conditions of R22 §2.4.3.1.12 hold: the
    /// router/coordinator has rebooted, and it is joined and authenticated.
    /// Starts (or restarts) `apsParentAnnounceTimer`; the child list is built
    /// when that timer expires, not now, so a child admitted during the delay
    /// is included and one that leaves is not.
    ///
    /// A no-op on a device that cannot route.
    pub fn schedule_parent_annce(&mut self) {
        if !self.nwk().can_route() {
            return;
        }
        // A fresh announcement supersedes any half-finished earlier sequence.
        self.nwk_mut().clear_parent_annce_pending();
        self.parent_annce_sequence_active = false;
        self.parent_annce_timer_secs = self.next_parent_annce_timer();
        log::info!(
            "[ZDO] Parent_annce scheduled in {}s (apsParentAnnounceTimer)",
            self.parent_annce_timer_secs
        );
    }

    #[cfg(feature = "router")]
    /// Stop `apsParentAnnounceTimer` and drop every outstanding announcement
    /// obligation.
    ///
    /// Used when the child table stops being authoritative — a leave, factory
    /// reset or fresh commissioning — so a stale announcement can never be
    /// broadcast for a network this device is no longer part of.
    pub fn cancel_parent_annce(&mut self) {
        self.parent_annce_timer_secs = 0;
        self.parent_annce_sequence_active = false;
        self.nwk_mut().clear_parent_annce_pending();
    }

    #[cfg(feature = "router")]
    /// Age `apsParentAnnounceTimer` and report whether a broadcast is now due.    ///
    /// Returns `true` exactly on the tick the timer reaches zero, so the
    /// caller sends one message and lets the send path re-arm the timer if
    /// more children remain.
    pub fn tick_parent_annce_timer(&mut self, elapsed_secs: u16) -> bool {
        if self.parent_annce_timer_secs == 0 {
            return false;
        }
        self.parent_annce_timer_secs = self.parent_annce_timer_secs.saturating_sub(elapsed_secs);
        self.parent_annce_timer_secs == 0
    }

    #[cfg(feature = "router")]
    /// Broadcast one due `Parent_annce` message.
    ///
    /// On the first expiry after [`schedule_parent_annce`](Self::schedule_parent_annce)
    /// this performs the R22 "construct the message" step — every end-device
    /// child is marked, explicitly without considering Keepalive Received —
    /// and then sends the first chunk. Each later call sends one further
    /// chunk. If children remain afterwards, a fresh jittered
    /// `apsParentAnnounceTimer` is armed for the next message.
    ///
    /// A constructed message with `NumberOfChildren == 0` is discarded and
    /// nothing is transmitted.
    pub async fn send_due_parent_annce(&mut self) -> Result<(), ZdoError> {
        if !self.nwk().can_route() {
            return Ok(());
        }
        if !self.parent_annce_sequence_active {
            self.parent_annce_sequence_active = true;
            if self.nwk_mut().mark_parent_annce_pending() == 0 {
                // Constructed with NumberOfChildren == 0 — discard, do not send.
                self.parent_annce_sequence_active = false;
                return Ok(());
            }
        } else if !self.nwk().has_parent_annce_pending() {
            // Every remaining child kept alive before its outstanding chunk.
            // This ends the existing sequence; it must not construct a new one
            // from the full child table and start broadcasting forever.
            self.parent_annce_sequence_active = false;
            return Ok(());
        }
        let chunk = self
            .nwk_mut()
            .take_parent_annce_chunk::<MAX_PARENT_ANNCE_CHILDREN>();
        if chunk.is_empty() {
            self.parent_annce_sequence_active = false;
            return Ok(());
        }
        self.send_parent_annce_children(&chunk).await?;
        if self.nwk().has_parent_annce_pending() {
            // R22: each additional message needs its own jittered timer.
            self.parent_annce_timer_secs = self.next_parent_annce_timer();
        } else {
            self.parent_annce_sequence_active = false;
        }
        Ok(())
    }

    /// Broadcast a `Parent_annce` listing this parent's end-device children
    /// immediately, bypassing `apsParentAnnounceTimer`.
    ///
    /// Retained for tests and for a product that has to force reconciliation
    /// (for example straight after a manual child-table restore in a lab
    /// binary). Normal operation goes through
    /// [`schedule_parent_annce`](Self::schedule_parent_annce), which is the
    /// jittered, normative path.
    ///
    /// A no-op (returns `Ok`) on a device that cannot route, so an end-device
    /// build never emits it.
    pub async fn send_parent_annce(&mut self) -> Result<(), ZdoError> {
        if !self.nwk().can_route() {
            return Ok(());
        }
        let children = self
            .nwk()
            .authenticated_child_ieees::<{ zigbee_nwk::neighbor::MAX_NEIGHBORS }>();
        self.send_parent_annce_children(&children).await
    }

    async fn send_parent_annce_children(
        &mut self,
        children: &[IeeeAddress],
    ) -> Result<(), ZdoError> {
        self.clear_pending_parent_annce();
        // Nothing to announce means nothing to reconcile.
        if children.is_empty() {
            return Ok(());
        }
        log::info!(
            "[ZDO] Parent_annce for {} children in {} frame(s)",
            children.len(),
            children.len().div_ceil(MAX_PARENT_ANNCE_CHILDREN)
        );
        for chunk in children.chunks(MAX_PARENT_ANNCE_CHILDREN) {
            let mut buf = [0u8; 1 + 1 + MAX_PARENT_ANNCE_CHILDREN * 8];
            let tsn = self.next_seq();
            buf[0] = tsn;
            let list_len = serialize_child_list(chunk, &mut buf[1..])?;
            let pending = self.register_pending_parent_annce(tsn)?;
            if let Err(error) = self
                .send_zdp_broadcast_to(BROADCAST_ROUTERS, PARENT_ANNCE, &buf[..1 + list_len])
                .await
            {
                self.cancel_pending(pending);
                return Err(error);
            }
        }
        Ok(())
    }

    /// Handle an incoming `Parent_annce` (called by the handler).
    ///
    /// Applies the announcement to the end-device child table and, if this
    /// device is keeping any announced child alive, unicasts a
    /// `Parent_annce_rsp` back to the announcer so it prunes its stale copy.
    /// Gated on `can_route()`, so an end device drops it with no further
    /// processing, as R22 requires.
    ///
    /// The dispatcher admits only the normative secured `0xFFFC` broadcast.
    /// If this device's own `apsParentAnnounceTimer` is non-zero, receipt
    /// re-calculates it and restarts the countdown before processing.
    #[cfg(feature = "router")]
    pub(crate) async fn process_parent_annce(
        &mut self,
        announcer: ShortAddress,
        request_tsn: u8,
        payload: &[u8],
    ) -> Result<(), ZdoError> {
        if !self.nwk().can_route() {
            return Ok(());
        }
        if self.parent_annce_timer_secs != 0 {
            self.parent_annce_timer_secs = self.next_parent_annce_timer();
        }
        let Some(children) = parse_child_list(payload) else {
            return Err(ZdoError::InvalidLength);
        };
        let outcome = self.nwk_mut().apply_parent_annce(&children);
        if outcome.kept.is_empty() {
            // NumberOfChildren == 0: discard the response, send nothing.
            return Ok(());
        }
        // The response is unicast, so R22 requires no jitter between chunks.
        for chunk in outcome.kept.chunks(MAX_PARENT_ANNCE_CHILDREN) {
            let mut buf = [0u8; 1 + 1 + 1 + MAX_PARENT_ANNCE_CHILDREN * 8];
            buf[0] = request_tsn;
            let body_len =
                serialize_parent_annce_rsp(ZdpStatus::Success as u8, chunk, &mut buf[1..])?;
            self.send_zdp_unicast(announcer, PARENT_ANNCE_RSP, &buf[..1 + body_len])
                .await?;
        }
        Ok(())
    }

    /// Handle an incoming `Parent_annce_rsp` (called by the handler).
    ///
    /// The responder is actively parenting the listed children, so this device
    /// relinquishes its stale records for them.
    #[cfg(feature = "router")]
    pub(crate) fn process_parent_annce_rsp(&mut self, response_tsn: u8, payload: &[u8]) {
        if !self.nwk().can_route() {
            return;
        }
        let Some((status, children)) = parse_parent_annce_rsp(payload) else {
            return;
        };
        if status != ZdpStatus::Success as u8 || !self.accepts_parent_annce_rsp(response_tsn) {
            return;
        }
        let dropped = self.nwk_mut().remove_children_by_ieee(&children);
        if !dropped.is_empty() {
            log::info!(
                "[ZDO] Parent_annce_rsp: relinquished {} moved children",
                dropped.len()
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ZdpStatus;
    use core::future::Future;
    use zigbee_aps::ApsLayer;
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::{DeviceType, NwkLayer};
    use zigbee_types::{MacAddress, PanId};

    const A: IeeeAddress = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    const B: IeeeAddress = [0xAA; 8];

    fn block_on<F: Future>(future: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
        fn noop(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut context = Context::from_waker(&waker);
        let mut future = core::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
        }
    }

    fn router_zdo() -> ZdoLayer<MockMac> {
        let mac = MockMac::new([0x10; 8]);
        let nwk = NwkLayer::new(mac, DeviceType::Router);
        let aps = ApsLayer::new(nwk);
        let mut zdo = ZdoLayer::new(aps);
        let nwk = zdo.nwk_mut();
        nwk.set_joined(true);
        nwk.nib_mut().pan_id = PanId(0x1234);
        nwk.nib_mut().network_address = ShortAddress(0x3344);
        zdo
    }

    fn sent_parent_annce_payloads(
        zdo: &ZdoLayer<MockMac>,
    ) -> heapless::Vec<
        heapless::Vec<u8, { zigbee_aps::apsde::APS_MAX_PAYLOAD }>,
        { crate::MAX_PENDING_ZDP },
    > {
        let mut payloads = heapless::Vec::new();
        for record in zdo.nwk().mac().tx_history() {
            let frame = record.payload.as_slice();
            let Some((_nwk, nwk_len)) = zigbee_nwk::frames::NwkHeader::parse(frame) else {
                continue;
            };
            let Some(aps_frame) = frame.get(nwk_len..) else {
                continue;
            };
            let Some((aps, aps_len)) = zigbee_aps::frames::ApsHeader::parse(aps_frame) else {
                continue;
            };
            if aps.cluster_id != Some(PARENT_ANNCE) {
                continue;
            }
            let mut payload = heapless::Vec::new();
            payload.extend_from_slice(&aps_frame[aps_len..]).unwrap();
            payloads.push(payload).unwrap();
        }
        payloads
    }

    #[test]
    fn child_list_round_trips_including_the_empty_case() {
        for children in [&[][..], &[A][..], &[A, B][..]] {
            let mut buf = [0u8; 1 + MAX_PARENT_ANNCE_CHILDREN * 8];
            let len = serialize_child_list(children, &mut buf).unwrap();
            assert_eq!(len, 1 + children.len() * 8);
            assert_eq!(buf[0] as usize, children.len());
            let parsed = parse_child_list(&buf[..len]).unwrap();
            assert_eq!(parsed.as_slice(), children);
        }
    }

    #[test]
    fn child_list_encodes_ieee_addresses_least_significant_byte_first() {
        // Golden vector: one child, count byte then the 8 IEEE bytes verbatim.
        let mut buf = [0u8; 9];
        assert_eq!(serialize_child_list(&[A], &mut buf).unwrap(), 9);
        assert_eq!(buf, [0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
    }

    #[test]
    fn parse_rejects_a_truncated_or_oversized_list() {
        // Declares one child but carries only 4 IEEE bytes.
        assert_eq!(parse_child_list(&[0x01, 1, 2, 3, 4]), None);
        // Declares more children than the bound allows.
        assert_eq!(
            parse_child_list(&[(MAX_PARENT_ANNCE_CHILDREN as u8) + 1]),
            None
        );
        // Empty slice has no count byte.
        assert_eq!(parse_child_list(&[]), None);
    }

    #[test]
    fn one_frame_bound_matches_the_aps_payload_budget() {
        assert_eq!(MAX_PARENT_ANNCE_CHILDREN, 9);
        let children = [[0x42; 8]; MAX_PARENT_ANNCE_CHILDREN + 1];
        let mut buf = [0u8; zigbee_aps::apsde::APS_MAX_PAYLOAD];
        assert_eq!(
            serialize_child_list(&children, &mut buf),
            Err(ZdoError::InvalidLength)
        );
    }

    #[test]
    fn large_child_lists_are_split_into_independent_broadcasts() {
        let mut zdo = router_zdo();
        let mut children = [[0u8; 8]; MAX_PARENT_ANNCE_CHILDREN + 1];
        for (index, ieee) in children.iter_mut().enumerate() {
            ieee[0] = index as u8;
        }

        block_on(zdo.send_parent_annce_children(&children)).unwrap();

        let payloads = sent_parent_annce_payloads(&zdo);
        assert_eq!(payloads.len(), 2);
        assert_eq!(payloads[0][1], MAX_PARENT_ANNCE_CHILDREN as u8);
        assert_eq!(payloads[0].len(), 2 + MAX_PARENT_ANNCE_CHILDREN * 8);
        assert_eq!(payloads[1][1], 1);
        assert_eq!(payloads[1].len(), 10);
        assert_ne!(
            payloads[0][0], payloads[1][0],
            "each broadcast chunk is a distinct ZDP transaction"
        );
    }

    #[test]
    fn parse_ignores_trailing_bytes_from_a_future_revision() {
        let frame = [
            0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0xDE, 0xAD,
        ];
        assert_eq!(parse_child_list(&frame).unwrap().as_slice(), &[A]);
    }

    #[test]
    fn response_round_trips_status_and_children() {
        let mut buf = [0u8; 1 + 1 + MAX_PARENT_ANNCE_CHILDREN * 8];
        let len = serialize_parent_annce_rsp(ZdpStatus::Success as u8, &[A, B], &mut buf).unwrap();
        assert_eq!(len, 2 + 2 * 8);
        assert_eq!(buf[0], 0x00);
        let (status, children) = parse_parent_annce_rsp(&buf[..len]).unwrap();
        assert_eq!(status, 0x00);
        assert_eq!(children.as_slice(), &[A, B]);
    }

    #[test]
    fn response_parse_rejects_a_missing_status_byte() {
        assert_eq!(parse_parent_annce_rsp(&[]), None);
    }

    // ── apsParentAnnounceTimer and the 0xFFFC destination ───────

    #[cfg(feature = "router")]
    fn add_end_device_child(zdo: &mut ZdoLayer<MockMac>, short: u16, ieee: IeeeAddress) {
        assert!(zdo.nwk_mut().restore_child(
            ieee,
            ShortAddress(short),
            false,
            true,
            false,
            zigbee_nwk::frames::ED_TIMEOUT_ENUM_DEFAULT,
        ));
    }

    #[cfg(feature = "router")]
    /// Run `apsParentAnnounceTimer` to expiry exactly as the joined tick does.
    fn expire_parent_annce_timer(zdo: &mut ZdoLayer<MockMac>) {
        let armed = zdo.parent_annce_timer_secs();
        assert!(armed > 0, "the timer must be running to expire");
        assert!(zdo.tick_parent_annce_timer(armed));
    }

    #[cfg(feature = "router")]
    /// Destination short address of the last NWK frame this MAC transmitted.
    fn last_nwk_destination(zdo: &ZdoLayer<MockMac>) -> Option<u16> {
        let record = zdo.nwk().mac().tx_history().last()?;
        let (header, _) = zigbee_nwk::frames::NwkHeader::parse(record.payload.as_slice())?;
        Some(header.dst_addr.0)
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_annce_is_broadcast_to_all_routers_and_the_coordinator() {
        // R22 §2.4.3.1.12: 0xFFFC, not the 0xFFFD RxOnWhenIdle broadcast used
        // by Device_annce.
        let mut zdo = router_zdo();
        add_end_device_child(&mut zdo, 0x1111, A);
        zdo.nwk_mut().mac_mut().clear_tx_history();

        block_on(zdo.send_parent_annce()).unwrap();

        assert_eq!(last_nwk_destination(&zdo), Some(BROADCAST_ROUTERS));
        assert_ne!(
            last_nwk_destination(&zdo),
            Some(crate::BROADCAST_RX_ON_IDLE)
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn scheduling_arms_a_jittered_timer_within_the_r22_bounds() {
        let mut zdo = router_zdo();
        assert_eq!(zdo.parent_annce_timer_secs(), 0, "idle by default");

        zdo.schedule_parent_annce();
        let armed = zdo.parent_annce_timer_secs();
        assert!(
            (APS_PARENT_ANNOUNCE_BASE_TIMER_SECS
                ..=APS_PARENT_ANNOUNCE_BASE_TIMER_SECS + APS_PARENT_ANNOUNCE_JITTER_MAX_SECS)
                .contains(&armed),
            "apsParentAnnounceBaseTimer + 0..=apsParentAnnounceJitterMax, got {armed}"
        );

        // Only the tick that reaches zero reports the message as due.
        assert!(!zdo.tick_parent_annce_timer(armed - 1));
        assert!(zdo.tick_parent_annce_timer(1));
        assert_eq!(zdo.parent_annce_timer_secs(), 0);
        assert!(
            !zdo.tick_parent_annce_timer(100),
            "a stopped timer is never due again on its own"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_due_announce_sends_one_chunk_per_timer_and_rearms_for_the_rest() {
        let mut zdo = router_zdo();
        for index in 0..(MAX_PARENT_ANNCE_CHILDREN + 1) {
            let mut ieee = [0u8; 8];
            ieee[0] = (index + 1) as u8;
            add_end_device_child(&mut zdo, 0x1000 + index as u16, ieee);
        }
        zdo.nwk_mut().mac_mut().clear_tx_history();
        zdo.schedule_parent_annce();

        // First expiry: construct, then send exactly one full frame.
        expire_parent_annce_timer(&mut zdo);
        block_on(zdo.send_due_parent_annce()).unwrap();
        let payloads = sent_parent_annce_payloads(&zdo);
        assert_eq!(payloads.len(), 1, "one message per timer expiry");
        assert_eq!(payloads[0][1], MAX_PARENT_ANNCE_CHILDREN as u8);
        assert!(
            zdo.parent_annce_timer_secs() >= APS_PARENT_ANNOUNCE_BASE_TIMER_SECS,
            "a fresh jittered timer is armed before each additional message"
        );

        // Second expiry: the remaining child, then the sequence is finished.
        expire_parent_annce_timer(&mut zdo);
        block_on(zdo.send_due_parent_annce()).unwrap();
        let payloads = sent_parent_annce_payloads(&zdo);
        assert_eq!(payloads.len(), 2);
        assert_eq!(payloads[1][1], 1);
        assert_eq!(
            zdo.parent_annce_timer_secs(),
            0,
            "no further message, so no further timer"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn keepalive_of_every_remaining_child_finishes_the_existing_sequence() {
        let mut zdo = router_zdo();
        for index in 0..(MAX_PARENT_ANNCE_CHILDREN + 1) {
            let mut ieee = [0u8; 8];
            ieee[0] = (index + 1) as u8;
            add_end_device_child(&mut zdo, 0x1000 + index as u16, ieee);
        }
        zdo.nwk_mut().mac_mut().clear_tx_history();
        zdo.schedule_parent_annce();

        expire_parent_annce_timer(&mut zdo);
        block_on(zdo.send_due_parent_annce()).unwrap();
        assert_eq!(sent_parent_annce_payloads(&zdo).len(), 1);

        let remaining = ShortAddress(0x1000 + MAX_PARENT_ANNCE_CHILDREN as u16);
        block_on(
            zdo.nwk_mut()
                .service_child_data_request(MacAddress::Short(PanId(0x1234), remaining)),
        )
        .unwrap();

        expire_parent_annce_timer(&mut zdo);
        block_on(zdo.send_due_parent_annce()).unwrap();
        assert_eq!(
            sent_parent_annce_payloads(&zdo).len(),
            1,
            "the completed sequence must not reconstruct and resend its first chunk"
        );
        assert_eq!(zdo.parent_annce_timer_secs(), 0);
        assert!(!zdo.parent_annce_sequence_active);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_childless_router_constructs_nothing_and_transmits_nothing() {
        // R22: "If NumberOfChildren is 0, it shall discard the previously
        // constructed Parent_annce message and not send it."
        let mut zdo = router_zdo();
        zdo.nwk_mut().mac_mut().clear_tx_history();
        zdo.schedule_parent_annce();

        expire_parent_annce_timer(&mut zdo);
        block_on(zdo.send_due_parent_annce()).unwrap();

        assert!(sent_parent_annce_payloads(&zdo).is_empty());
        assert_eq!(zdo.parent_annce_timer_secs(), 0);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_received_broadcast_announce_restarts_a_running_timer() {
        // R22 §2.4.3.1.12 "Effect on receipt": a running apsParentAnnounceTimer
        // is re-calculated and restarted. This is the storm guard for several
        // routers rebooting together.
        let mut zdo = router_zdo();
        add_end_device_child(&mut zdo, 0x1111, A);
        zdo.schedule_parent_annce();
        // Wind the timer most of the way down so a restart is observable
        // regardless of the jitter sample.
        let armed = zdo.parent_annce_timer_secs();
        assert!(!zdo.tick_parent_annce_timer(armed - 1));
        assert_eq!(zdo.parent_annce_timer_secs(), 1);

        let mut payload = [0u8; 9];
        serialize_child_list(&[B], &mut payload).unwrap();
        block_on(zdo.process_parent_annce(ShortAddress(0x0002), 7, &payload)).unwrap();

        assert!(
            zdo.parent_annce_timer_secs() >= APS_PARENT_ANNOUNCE_BASE_TIMER_SECS,
            "the countdown restarted from the base timer"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_idle_timer_is_not_started_by_a_received_announce() {
        let mut zdo = router_zdo();
        add_end_device_child(&mut zdo, 0x1111, A);
        assert_eq!(zdo.parent_annce_timer_secs(), 0);

        let mut payload = [0u8; 9];
        serialize_child_list(&[B], &mut payload).unwrap();
        block_on(zdo.process_parent_annce(ShortAddress(0x0002), 7, &payload)).unwrap();

        assert_eq!(
            zdo.parent_annce_timer_secs(),
            0,
            "receiving an announcement is not itself a generating condition"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn cancelling_stops_the_timer_and_drops_outstanding_children() {
        let mut zdo = router_zdo();
        add_end_device_child(&mut zdo, 0x1111, A);
        zdo.schedule_parent_annce();
        assert_eq!(zdo.nwk_mut().mark_parent_annce_pending(), 1);

        zdo.cancel_parent_annce();

        assert_eq!(zdo.parent_annce_timer_secs(), 0);
        assert!(!zdo.nwk().has_parent_annce_pending());
        assert!(!zdo.parent_annce_sequence_active);
    }
}
