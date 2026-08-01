//! R22 Parent Announce (ZDP clusters `Parent_annce` 0x001F and
//! `Parent_annce_rsp` 0x801F).
//!
//! `Parent_annce` is a **ZDP** (Zigbee Device Profile) primitive carried over
//! APS on endpoint 0, not a NWK-layer command. A router/coordinator that has
//! restored or rebuilt its child table broadcasts the IEEE addresses of the
//! children it parents so that any *former* parent of a child that has since
//! moved can prune its stale entry. A recipient that is actively parenting one
//! of the announced children replies with `Parent_annce_rsp` listing those
//! children, and the announcer then relinquishes them.
//!
//! Only routers and coordinators have a child table, so end devices never send
//! it and never act on it (the receive path is gated on `can_route()`).

use zigbee_mac::MacDriver;
use zigbee_types::{IeeeAddress, ShortAddress};

use crate::{PARENT_ANNCE, PARENT_ANNCE_RSP, ZdoError, ZdoLayer, ZdpStatus};

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

    /// Broadcast a `Parent_annce` listing this parent's authenticated children.
    ///
    /// A no-op (returns `Ok`) on a device that cannot route, so an end-device
    /// build never emits it. Callers should invoke this only **after** the
    /// child table is authoritative (restored from persistence or freshly
    /// built).
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
                .send_zdp_broadcast(PARENT_ANNCE, &buf[..1 + list_len])
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
    /// Applies the announcement to the child table and, if this device is
    /// keeping any announced child, unicasts a `Parent_annce_rsp` back to the
    /// announcer so it prunes its stale copy. Gated on `can_route()`, so an end
    /// device ignores it entirely.
    pub(crate) async fn process_parent_annce(
        &mut self,
        announcer: ShortAddress,
        request_tsn: u8,
        payload: &[u8],
    ) -> Result<(), ZdoError> {
        if !self.nwk().can_route() {
            return Ok(());
        }
        let Some(children) = parse_child_list(payload) else {
            return Err(ZdoError::InvalidLength);
        };
        let outcome = self.nwk_mut().apply_parent_annce(&children);
        if outcome.kept.is_empty() {
            // No conflict: nothing to report, so no response frame is sent.
            return Ok(());
        }
        let mut buf = [0u8; 1 + 1 + 1 + MAX_PARENT_ANNCE_CHILDREN * 8];
        buf[0] = request_tsn;
        let body_len =
            serialize_parent_annce_rsp(ZdpStatus::Success as u8, &outcome.kept, &mut buf[1..])?;
        self.send_zdp_unicast(announcer, PARENT_ANNCE_RSP, &buf[..1 + body_len])
            .await
    }

    /// Handle an incoming `Parent_annce_rsp` (called by the handler).
    ///
    /// The responder is actively parenting the listed children, so this device
    /// relinquishes its stale records for them.
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
    use core::future::Future;
    use zigbee_aps::ApsLayer;
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::{DeviceType, NwkLayer};
    use zigbee_types::PanId;

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
}
