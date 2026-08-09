//! Pure-Rust TLSR8258 MAC backend.
//!
//! This backend wraps the hardware-proven `tlsr8258-hal` radio path and
//! supports two roles:
//!
//! - **End device** (fully supported): scan, associate, sleepy poll, secure
//!   rejoin.
//! - **Router/parent, EXPERIMENTAL** (see `imp::TelinkMac::mlme_start`
//!   below): joins as an FFD, enters non-beacon continuous-RX mode, relays
//!   traffic, participates in route discovery, and serves sleepy children.
//!
//! Parent-facing MAC primitives (on-demand beacons, Association Response,
//! command events, and per-child ACK Frame Pending state) are implemented.
//! Admission policy, address allocation, and NWK indirect transactions remain
//! upper-layer responsibilities and are integrated by `zigbee-runtime`.
//! Parent mode still requires sniffer validation of TLSR8258 software-ACK
//! turnaround before release hardware is claimed interoperable.

#[cfg(any(target_arch = "tc32", test))]
use crate::MacError;
#[cfg(any(target_arch = "tc32", test))]
use crate::frames::{addressing_size, parse_mac_addresses};
#[cfg(any(target_arch = "tc32", test))]
use crate::primitives::MlmeStartRequest;
#[cfg(any(target_arch = "tc32", test))]
use crate::primitives::{
    AssociationStatus, MacCommandEvent, McpsDataIndication, MlmeAssociateIndication,
    MlmeAssociateResponse, MlmeAssociateResponseDelivery, MlmeBeaconRequestIndication,
    MlmeDataRequestIndication, PanDescriptor, PanDescriptorList,
};
#[cfg(any(target_arch = "tc32", test))]
use zigbee_types::{IeeeAddress, MacAddress, PanId};

#[cfg(any(target_arch = "tc32", test))]
const COMMAND_EVENT_QUEUE_CAPACITY: usize = 8;
#[cfg(any(target_arch = "tc32", test))]
const DATA_INDICATION_QUEUE_CAPACITY: usize = 8;
#[cfg(any(target_arch = "tc32", test))]
const ASSOCIATION_RESPONSE_QUEUE_CAPACITY: usize = 8;
#[cfg(any(target_arch = "tc32", test))]
const ASSOCIATION_DELIVERY_QUEUE_CAPACITY: usize = 16;
#[cfg(any(target_arch = "tc32", test))]
const BASE_SUPERFRAME_DURATION_US: u32 = 960 * 16;

#[cfg(any(target_arch = "tc32", test))]
struct QueuedCommandEvent {
    event: MacCommandEvent,
    /// This physical poll already retried or completed a retained
    /// Association Response transaction.
    association_poll_serviced: bool,
}

#[cfg(any(target_arch = "tc32", test))]
struct CommandEventQueue {
    events: heapless::Deque<QueuedCommandEvent, COMMAND_EVENT_QUEUE_CAPACITY>,
    /// Cumulative events lost because the bounded queue was full. Never
    /// reset by [`Self::clear`]: a loss has to stay attributable across
    /// rejoin and retention-sleep cycles.
    overflow: u32,
    high_water: u8,
}

#[cfg(any(target_arch = "tc32", test))]
impl CommandEventQueue {
    const fn new() -> Self {
        Self {
            events: heapless::Deque::new(),
            overflow: 0,
            high_water: 0,
        }
    }

    /// Retain the oldest events when the bounded queue is full.
    ///
    /// Unlike data indications, MAC command events are self-generated
    /// transaction state (child polls, association requests) rather than
    /// unrelated channel traffic, so there is no lower-value class to
    /// sacrifice and the oldest transaction stays authoritative. The loss
    /// is counted rather than swallowed.
    fn push(&mut self, event: MacCommandEvent) -> bool {
        self.push_with_association_poll_state(event, false)
    }

    /// Queue an event while remembering whether this physical child poll has
    /// already handled a retained Association Response transaction.
    fn push_with_association_poll_state(
        &mut self,
        event: MacCommandEvent,
        association_poll_serviced: bool,
    ) -> bool {
        let queued = self
            .events
            .push_back(QueuedCommandEvent {
                event,
                association_poll_serviced,
            })
            .is_ok();
        if queued {
            self.record_depth();
        } else {
            self.overflow = self.overflow.wrapping_add(1);
        }
        queued
    }

    fn pop_queued(&mut self) -> Option<QueuedCommandEvent> {
        self.events.pop_front()
    }

    fn push_queued_front(&mut self, event: QueuedCommandEvent) -> bool {
        let queued = self.events.push_front(event).is_ok();
        if queued {
            self.record_depth();
        } else {
            self.overflow = self.overflow.wrapping_add(1);
        }
        queued
    }

    fn record_depth(&mut self) {
        let depth = self.events.len() as u8;
        if depth > self.high_water {
            self.high_water = depth;
        }
    }

    #[cfg(test)]
    fn pop(&mut self) -> Option<MacCommandEvent> {
        self.pop_queued().map(|queued| queued.event)
    }

    fn clear(&mut self) {
        self.events.clear();
    }
}

/// Retention class of a queued [`McpsDataIndication`], ordered lowest-value
/// = first to be sacrificed when the bounded queue is full.
///
/// Mirrors `tlsr8258_hal::radio::RxPriority` one layer up, where the full
/// parsed destination address is already available.
#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
enum IndicationPriority {
    /// Accepted only because of promiscuous mode or a broadcast PAN: not
    /// addressed to this node at all.
    Foreign = 0,
    /// Broadcast inside this node's PAN.
    Broadcast = 1,
    /// Unicast to this node's short or extended address.
    Local = 2,
}

/// Classify a parsed destination address against the local identity.
#[cfg(any(target_arch = "tc32", test))]
fn indication_priority(
    pan_id: PanId,
    short_address: zigbee_types::ShortAddress,
    extended_address: &IeeeAddress,
    destination: &MacAddress,
) -> IndicationPriority {
    match destination {
        MacAddress::Short(pan, address) => {
            if address.0 >= 0xFFF8 {
                if pan.0 == pan_id.0 || pan.0 == 0xFFFF {
                    IndicationPriority::Broadcast
                } else {
                    IndicationPriority::Foreign
                }
            } else if pan.0 == pan_id.0 && short_address.0 < 0xFFF8 && address.0 == short_address.0
            {
                IndicationPriority::Local
            } else {
                IndicationPriority::Foreign
            }
        }
        MacAddress::Extended(pan, address) => {
            if pan.0 == pan_id.0 && address == extended_address {
                IndicationPriority::Local
            } else {
                IndicationPriority::Foreign
            }
        }
    }
}

#[cfg(any(target_arch = "tc32", test))]
struct QueuedIndication {
    indication: McpsDataIndication,
    priority: IndicationPriority,
}

/// Bounded queue of MAC data indications that were received while the
/// caller was waiting for something else (an ACK, an Association Response,
/// a child poll).
///
/// Ordering is strict FIFO while there is room. Under overload the queue
/// sacrifices the oldest strictly-lower-priority entry rather than refusing
/// the arrival, because the newest frame addressed to this node is the one
/// an upper layer is actively waiting for while stale broadcast/foreign
/// entries are only relay traffic.
#[cfg(any(target_arch = "tc32", test))]
struct DataIndicationQueue {
    indications: heapless::Vec<QueuedIndication, DATA_INDICATION_QUEUE_CAPACITY>,
    /// Cumulative indications lost at this queue, and how many of those
    /// losses were evictions that saved a more important arrival. Never
    /// reset by [`Self::clear`].
    overflow: u32,
    evicted: u32,
    high_water: u8,
}

#[cfg(any(target_arch = "tc32", test))]
impl DataIndicationQueue {
    const fn new() -> Self {
        Self {
            indications: heapless::Vec::new(),
            overflow: 0,
            evicted: 0,
            high_water: 0,
        }
    }

    /// Queue `indication`, sacrificing the oldest strictly-lower-priority
    /// entry if the queue is full. Returns `false` only when the queue was
    /// full of entries at least as important as this one.
    fn push(&mut self, indication: McpsDataIndication, priority: IndicationPriority) -> bool {
        if self.indications.is_full() {
            self.overflow = self.overflow.wrapping_add(1);
            let Some(victim) = self.victim_for(priority) else {
                return false;
            };
            self.indications.remove(victim);
            self.evicted = self.evicted.wrapping_add(1);
        }
        let queued = self
            .indications
            .push(QueuedIndication {
                indication,
                priority,
            })
            .is_ok();
        debug_assert!(queued);
        let depth = self.indications.len() as u8;
        if depth > self.high_water {
            self.high_water = depth;
        }
        queued
    }

    /// Oldest entry among those of the lowest priority present, but only if
    /// that priority is strictly below the arriving indication's.
    fn victim_for(&self, priority: IndicationPriority) -> Option<usize> {
        self.indications
            .iter()
            .enumerate()
            .filter(|(_, queued)| queued.priority < priority)
            .min_by_key(|(_, queued)| queued.priority)
            .map(|(index, _)| index)
    }

    fn pop(&mut self) -> Option<McpsDataIndication> {
        if self.indications.is_empty() {
            None
        } else {
            Some(self.indications.remove(0).indication)
        }
    }

    fn take_matching(
        &mut self,
        mut predicate: impl FnMut(&McpsDataIndication) -> bool,
    ) -> Option<McpsDataIndication> {
        let index = self
            .indications
            .iter()
            .position(|queued| predicate(&queued.indication))?;
        Some(self.indications.remove(index).indication)
    }

    fn any_matching(&self, mut predicate: impl FnMut(&McpsDataIndication) -> bool) -> bool {
        self.indications
            .iter()
            .any(|queued| predicate(&queued.indication))
    }

    /// Drop every retained indication without discarding the cumulative
    /// loss counters.
    fn clear(&mut self) {
        self.indications.clear();
    }

    const fn counters(&self) -> (u32, u32, u8) {
        (self.overflow, self.evicted, self.high_water)
    }

    #[cfg(test)]
    fn len(&self) -> usize {
        self.indications.len()
    }
}

/// Bounded MAC queue diagnostics, reported separately from the HAL's
/// interrupt receive queue so a loss can be attributed to the exact stage
/// that dropped it.
///
/// `data_indication_overflow - data_indication_evicted` is the number of
/// indications dropped outright; that difference is the number that has to
/// stay at zero. `*_high_water` is what justifies (or refutes) each
/// capacity from a real capture instead of a guess.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MacQueueDiagnostics {
    pub data_indication_overflow: u32,
    pub data_indication_evicted: u32,
    pub data_indication_high_water: u8,
    pub command_event_overflow: u32,
    pub command_event_high_water: u8,
}

#[cfg(any(target_arch = "tc32", test))]
fn take_pending_data(data: &mut DataIndicationQueue) -> Option<McpsDataIndication> {
    data.pop()
}

#[cfg(any(target_arch = "tc32", test))]
fn take_pending_poll_data(
    data: &mut DataIndicationQueue,
    pan_id: PanId,
    short_address: zigbee_types::ShortAddress,
    extended_address: &IeeeAddress,
) -> Option<McpsDataIndication> {
    data.take_matching(|indication| match &indication.dst_address {
        MacAddress::Short(pan, address) => *pan == pan_id && *address == short_address,
        MacAddress::Extended(pan, address) => *pan == pan_id && address == extended_address,
    })
}

#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone)]
struct PendingAssociationResponse {
    response: MlmeAssociateResponse,
    expires_at_us: u32,
}

#[cfg(any(target_arch = "tc32", test))]
struct AssociationResponseQueue {
    responses: heapless::Vec<PendingAssociationResponse, ASSOCIATION_RESPONSE_QUEUE_CAPACITY>,
}

#[cfg(any(target_arch = "tc32", test))]
impl AssociationResponseQueue {
    const fn new() -> Self {
        Self {
            responses: heapless::Vec::new(),
        }
    }

    fn can_enqueue(&self, child: &IeeeAddress) -> bool {
        self.responses.len() < ASSOCIATION_RESPONSE_QUEUE_CAPACITY
            || self
                .responses
                .iter()
                .any(|entry| entry.response.device_address == *child)
    }

    fn enqueue(
        &mut self,
        response: MlmeAssociateResponse,
        now_us: u32,
        persistence_time: u16,
    ) -> Result<(), MlmeAssociateResponse> {
        let expires_at_us =
            now_us.wrapping_add(u32::from(persistence_time) * BASE_SUPERFRAME_DURATION_US);
        if let Some(existing) = self
            .responses
            .iter_mut()
            .find(|existing| existing.response.device_address == response.device_address)
        {
            *existing = PendingAssociationResponse {
                response,
                expires_at_us,
            };
            return Ok(());
        }
        self.responses
            .push(PendingAssociationResponse {
                response,
                expires_at_us,
            })
            .map_err(|entry| entry.response)
    }

    fn active_for(&self, child: &IeeeAddress, now_us: u32) -> Option<MlmeAssociateResponse> {
        self.responses
            .iter()
            .find(|entry| {
                entry.response.device_address == *child
                    && !deadline_reached(now_us, entry.expires_at_us)
            })
            .map(|entry| entry.response.clone())
    }

    fn remove(&mut self, child: &IeeeAddress) -> Option<MlmeAssociateResponse> {
        let index = self
            .responses
            .iter()
            .position(|entry| entry.response.device_address == *child)?;
        Some(self.responses.swap_remove(index).response)
    }

    fn pop_expired(&mut self, now_us: u32) -> Option<MlmeAssociateResponse> {
        let index = self
            .responses
            .iter()
            .position(|entry| deadline_reached(now_us, entry.expires_at_us))?;
        Some(self.responses.swap_remove(index).response)
    }

    fn micros_until_next_expiry(&self, now_us: u32) -> Option<u32> {
        self.responses
            .iter()
            .filter(|entry| !deadline_reached(now_us, entry.expires_at_us))
            .map(|entry| entry.expires_at_us.wrapping_sub(now_us))
            .min()
    }

    fn clear(&mut self) {
        self.responses.clear();
    }

    fn active_child_for_short(
        &self,
        short_address: zigbee_types::ShortAddress,
        now_us: u32,
    ) -> Option<IeeeAddress> {
        self.responses
            .iter()
            .find(|entry| {
                entry.response.status == AssociationStatus::Success
                    && entry.response.short_address == short_address
                    && !deadline_reached(now_us, entry.expires_at_us)
            })
            .map(|entry| entry.response.device_address)
    }
}

#[cfg(any(target_arch = "tc32", test))]
fn deadline_reached(now: u32, deadline: u32) -> bool {
    now.wrapping_sub(deadline) < 0x8000_0000
}

#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PendingAssociationPoll {
    RetryResponse(IeeeAddress),
    ConfirmDelivered(IeeeAddress),
}

#[cfg(any(target_arch = "tc32", test))]
fn pending_association_poll(
    event: &MacCommandEvent,
    responses: &AssociationResponseQueue,
    now_us: u32,
) -> Option<PendingAssociationPoll> {
    let MacCommandEvent::DataRequest(indication) = event else {
        return None;
    };
    match indication.source_address {
        MacAddress::Extended(_, child) => responses
            .active_for(&child, now_us)
            .map(|_| PendingAssociationPoll::RetryResponse(child)),
        MacAddress::Short(_, short_address) => responses
            .active_child_for_short(short_address, now_us)
            .map(PendingAssociationPoll::ConfirmDelivered),
    }
}

#[cfg(any(target_arch = "tc32", test))]
fn queued_association_poll_to_service(
    queued: &QueuedCommandEvent,
    responses: &AssociationResponseQueue,
    now_us: u32,
) -> Option<PendingAssociationPoll> {
    if queued.association_poll_serviced {
        None
    } else {
        pending_association_poll(&queued.event, responses, now_us)
    }
}

#[cfg(any(target_arch = "tc32", test))]
fn queue_command_for_parent_receive(
    events: &mut CommandEventQueue,
    event: MacCommandEvent,
    association_poll_serviced: bool,
) -> bool {
    let is_data_request = matches!(&event, MacCommandEvent::DataRequest(_));
    events.push_with_association_poll_state(event, association_poll_serviced) && is_data_request
}

#[cfg(any(target_arch = "tc32", test))]
fn take_successful_association_delivery(
    associations: &mut AssociationResponseQueue,
    child: &IeeeAddress,
) -> Option<MlmeAssociateResponseDelivery> {
    let response = associations.remove(child)?;
    Some(MlmeAssociateResponseDelivery {
        device_address: response.device_address,
        short_address: response.short_address,
        status: response.status,
        result: Ok(()),
    })
}

#[cfg(any(target_arch = "tc32", test))]
fn clear_transient_queues(
    data: &mut DataIndicationQueue,
    events: &mut CommandEventQueue,
    associations: &mut AssociationResponseQueue,
) {
    data.clear();
    events.clear();
    associations.clear();
}

#[cfg(any(target_arch = "tc32", test))]
enum ParsedIncomingFrame {
    Data(McpsDataIndication),
    Command(MacCommandEvent),
}

#[cfg(any(target_arch = "tc32", test))]
impl ParsedIncomingFrame {
    #[cfg(target_arch = "tc32")]
    fn destination(&self) -> &MacAddress {
        match self {
            Self::Data(indication) => &indication.dst_address,
            Self::Command(MacCommandEvent::BeaconRequest(indication)) => {
                &indication.destination_address
            }
            Self::Command(MacCommandEvent::AssociationRequest(indication)) => {
                &indication.coordinator_address
            }
            Self::Command(MacCommandEvent::AssociationResponseDelivery(_)) => {
                unreachable!("delivery completions are not parsed from received frames")
            }
            Self::Command(MacCommandEvent::DataRequest(indication)) => {
                &indication.destination_address
            }
        }
    }
}

#[cfg(any(target_arch = "tc32", test))]
fn is_unicast_address(address: &MacAddress) -> bool {
    match address {
        MacAddress::Short(_, address) => address.0 < 0xFFF8,
        MacAddress::Extended(_, address) => *address != [0xFF; 8],
    }
}

#[cfg(any(target_arch = "tc32", test))]
fn accepts_telink_destination(
    own_pan: PanId,
    own_short: zigbee_types::ShortAddress,
    own_extended: &zigbee_types::IeeeAddress,
    promiscuous: bool,
    destination: &MacAddress,
) -> bool {
    promiscuous || crate::frames::frame_is_for_us(destination, own_pan, own_short, own_extended)
}

/// Parse one raw PSDU into either the existing data indication or one of the
/// parent-facing MAC command events.
///
/// Command parsing is intentionally strict. TLSR8258 supplies raw, undecrypted
/// PSDUs, so a security-enabled command cannot be identified safely (the
/// command identifier is protected and follows a variable auxiliary security
/// header) and is rejected rather than misclassified.
#[cfg(any(target_arch = "tc32", test))]
fn parse_incoming_frame(data: &[u8], lqi: u8) -> Option<ParsedIncomingFrame> {
    if data.len() < 3 {
        return None;
    }

    let frame_control = u16::from_le_bytes([data[0], data[1]]);
    let frame_type = frame_control & 0x07;
    let security_use = frame_control & (1 << 3) != 0;

    if frame_type == 0x01 {
        let (source, destination, payload_offset, security_use) = parse_mac_addresses(data);
        if payload_offset > data.len() {
            return None;
        }
        return Some(ParsedIncomingFrame::Data(McpsDataIndication {
            src_address: source,
            dst_address: destination,
            lqi,
            payload: crate::primitives::MacFrame::from_slice(&data[payload_offset..])?,
            security_use,
        }));
    }
    if frame_type != 0x03 || security_use {
        return None;
    }

    // This parser supports the 2003/2006 command layout used by Zigbee. It
    // requires a sequence number and rejects reserved addressing modes.
    let frame_version = (frame_control >> 12) & 0x03;
    let dst_mode = (frame_control >> 10) & 0x03;
    let src_mode = (frame_control >> 14) & 0x03;
    let pan_compress = frame_control & (1 << 6) != 0;
    let ack_request = frame_control & (1 << 5) != 0;
    if frame_version > 1
        || frame_control & ((1 << 7) | (1 << 8) | (1 << 9)) != 0
        || dst_mode == 1
        || src_mode == 1
    {
        return None;
    }

    let payload_offset = 3usize.checked_add(addressing_size(frame_control))?;
    if payload_offset >= data.len() {
        return None;
    }
    let (source, destination, parsed_offset, _) = parse_mac_addresses(data);
    if parsed_offset != payload_offset {
        return None;
    }

    match data[payload_offset] {
        // Beacon Request: broadcast short destination, no source, no ACK.
        0x07 if data.len() == payload_offset + 1
            && dst_mode == 2
            && src_mode == 0
            && !pan_compress
            && !ack_request
            && destination
                == MacAddress::Short(PanId::BROADCAST, zigbee_types::ShortAddress::BROADCAST) =>
        {
            Some(ParsedIncomingFrame::Command(
                MacCommandEvent::BeaconRequest(MlmeBeaconRequestIndication {
                    destination_address: destination,
                    lqi,
                    security_use,
                }),
            ))
        }
        // Association Request: unassociated extended source and one
        // Capability Information byte.
        0x01 if data.len() == payload_offset + 2
            && matches!(dst_mode, 2 | 3)
            && src_mode == 3
            && !pan_compress
            && ack_request =>
        {
            let MacAddress::Extended(source_pan, device_address) = source else {
                return None;
            };
            if source_pan != PanId::BROADCAST
                || device_address == [0xFF; 8]
                || destination.pan_id() == PanId::BROADCAST
                || !is_unicast_address(&destination)
            {
                return None;
            }
            Some(ParsedIncomingFrame::Command(
                MacCommandEvent::AssociationRequest(MlmeAssociateIndication {
                    device_address,
                    coordinator_address: destination,
                    capability_info: crate::primitives::CapabilityInfo::from_byte(
                        data[payload_offset + 1],
                    ),
                    lqi,
                    security_use,
                }),
            ))
        }
        // Data Request: an associated short source or an extended source
        // waiting for its Association Response. Both identities are retained.
        0x04 if data.len() == payload_offset + 1
            && matches!(dst_mode, 2 | 3)
            && matches!(src_mode, 2 | 3)
            && ack_request =>
        {
            if !is_unicast_address(&source)
                || !is_unicast_address(&destination)
                || source.pan_id() != destination.pan_id()
                || destination.pan_id() == PanId::BROADCAST
            {
                return None;
            }
            Some(ParsedIncomingFrame::Command(MacCommandEvent::DataRequest(
                MlmeDataRequestIndication {
                    source_address: source,
                    destination_address: destination,
                    lqi,
                    security_use,
                },
            )))
        }
        _ => None,
    }
}

#[cfg(any(target_arch = "tc32", test))]
fn upsert_pan_descriptor(descriptors: &mut PanDescriptorList, mut descriptor: PanDescriptor) {
    if let Some(existing) = descriptors.iter_mut().find(|existing| {
        existing.channel == descriptor.channel && existing.coord_address == descriptor.coord_address
    }) {
        descriptor.lqi = descriptor.lqi.max(existing.lqi);
        *existing = descriptor;
        return;
    }

    if descriptors.push(descriptor.clone()).is_ok() {
        return;
    }

    if let Some((weakest_index, weakest)) = descriptors
        .iter()
        .enumerate()
        .min_by_key(|(_, existing)| existing.lqi)
        && descriptor.lqi > weakest.lqi
    {
        descriptors[weakest_index] = descriptor;
    }
}

/// Validates an `MLME-START.request` for the TLSR8258 router/parent role.
///
/// This is a pure, host-testable function (see the `tests` module below)
/// kept separate from `imp::TelinkMac::mlme_start` so the parameter rules
/// can be exercised without hardware. It intentionally accepts only the
/// exact non-beacon, non-coordinator shape that
/// `zigbee_nwk`'s `Nlme::nlme_start_router()` sends after a router has
/// joined:
///
/// - `pan_coordinator` must be `false`. This backend never starts a PAN as
///   coordinator via `MLME-START` — that role has no independent hardware
///   evidence yet and is out of scope for this router path.
/// - `beacon_order` and `superframe_order` must both be `15` (non-beacon
///   mode). Beaconed superframes require transmitting periodic beacons and
///   are not implemented.
/// - `channel` must be a valid 2.4 GHz Zigbee channel (11..=26).
/// - `pan_id` must not be the broadcast PAN ID `0xFFFF`.
#[cfg(any(target_arch = "tc32", test))]
pub(crate) fn validate_router_start(req: &MlmeStartRequest) -> Result<(), MacError> {
    if req.pan_coordinator {
        return Err(MacError::Unsupported);
    }
    if req.beacon_order != 15 || req.superframe_order != 15 {
        return Err(MacError::Unsupported);
    }
    if !(11..=26).contains(&req.channel) {
        return Err(MacError::InvalidParameter);
    }
    if req.pan_id.0 == 0xFFFF {
        return Err(MacError::InvalidParameter);
    }
    Ok(())
}

/// Classification of one frame surfaced inside a MAC ACK wait window.
///
/// Kept separate from `imp::TelinkMac::transmit_with_ack` so the exact
/// acceptance rule (frame type *and* sequence number) is host-testable and
/// so the diagnostic counters below can distinguish "no ACK ever reached
/// the MAC" from "an ACK reached the MAC but carried a different DSN".
#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AckWindowFrame {
    /// An ACK whose sequence number matches the frame just transmitted.
    Match { frame_pending: bool },
    /// A well-formed ACK for some *other* sequence number. Seeing these
    /// while the expected ACK never matches means the receive path works
    /// and the defect is in sequence-number handling, not in RX enable or
    /// turnaround.
    ForeignAck { sequence: u8 },
    /// Anything that is not an ACK frame (data, command, beacon, runt).
    NotAnAck,
}

/// Frame type field of the 802.15.4 Frame Control Field.
#[cfg(any(target_arch = "tc32", test))]
const FRAME_TYPE_MASK: u16 = 0x0007;
#[cfg(any(target_arch = "tc32", test))]
const FRAME_TYPE_ACK: u16 = 0x0002;
/// Frame Pending bit of the 802.15.4 Frame Control Field.
#[cfg(any(target_arch = "tc32", test))]
const FRAME_PENDING_BIT: u16 = 1 << 4;

/// Classify one received MAC frame against the sequence number of the frame
/// currently awaiting acknowledgement.
///
/// `data` is a MAC frame with the FCS already removed (TLSR8258 validates
/// and strips it in hardware), so a valid ACK is exactly three bytes:
/// two Frame Control bytes plus the sequence number.
#[cfg(any(target_arch = "tc32", test))]
pub(crate) fn classify_ack_window_frame(data: &[u8], expected_sequence: u8) -> AckWindowFrame {
    if data.len() < 3 {
        return AckWindowFrame::NotAnAck;
    }
    let frame_control = u16::from_le_bytes([data[0], data[1]]);
    if frame_control & FRAME_TYPE_MASK != FRAME_TYPE_ACK {
        return AckWindowFrame::NotAnAck;
    }
    let sequence = data[2];
    if sequence == expected_sequence {
        AckWindowFrame::Match {
            frame_pending: frame_control & FRAME_PENDING_BIT != 0,
        }
    } else {
        AckWindowFrame::ForeignAck { sequence }
    }
}

/// Bounded MAC-level transmit/acknowledgement counters.
///
/// These exist to make an on-air retransmission burst attributable from a
/// single RAM read after the fact, without any per-frame logging that would
/// perturb the turnaround-critical radio path. Every field saturates
/// instead of wrapping so a long soak cannot make a large count look small.
///
/// Reading them together with [`tlsr8258_hal::radio::rx_diagnostics`]
/// separates the four candidate failure modes for a retransmission that
/// follows a valid on-air ACK:
///
/// - `ack_windows_expired > 0` with `ack_frames_seen == 0`: no ACK frame
///   ever reached the MAC — the receiver was not listening, was still in
///   turnaround, or the frame was rejected inside the HAL (cross-check the
///   HAL's `invalid_length` / `invalid_crc` / `dma_incomplete` counters).
/// - `foreign_acks > 0` with `last_foreign_ack_sequence` set: ACKs arrive
///   but carry an unexpected DSN — a sequence-number defect.
/// - `window_frames_seen` far below the number of frames on air: the
///   receive path, not the ACK logic, is losing frames.
/// - HAL `queue_overflow > 0`: frames reached the HAL and were dropped by
///   the bounded interrupt queue.
#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AckDiagnostics {
    /// Frames handed to the radio with an ACK requested, including retries.
    pub tx_attempts: u32,
    /// ACK wait windows opened.
    pub ack_windows: u32,
    /// Windows that produced a sequence-matched ACK.
    pub ack_matched: u32,
    /// Windows that ended without a sequence-matched ACK.
    pub ack_windows_expired: u32,
    /// ACK frames (any DSN) surfaced to the MAC inside an ACK window.
    pub ack_frames_seen: u32,
    /// ACK frames surfaced with a DSN other than the expected one.
    pub foreign_acks: u32,
    /// Total frames of any kind surfaced inside ACK windows.
    pub window_frames_seen: u32,
    /// DSN of the most recent unmatched ACK, or `None` if none was seen.
    pub last_foreign_ack_sequence: Option<u8>,
    /// DSN awaited by the most recent expired window.
    pub last_expired_sequence: Option<u8>,
}

#[cfg(any(target_arch = "tc32", test))]
impl AckDiagnostics {
    const fn new() -> Self {
        Self {
            tx_attempts: 0,
            ack_windows: 0,
            ack_matched: 0,
            ack_windows_expired: 0,
            ack_frames_seen: 0,
            foreign_acks: 0,
            window_frames_seen: 0,
            last_foreign_ack_sequence: None,
            last_expired_sequence: None,
        }
    }

    /// Record one classified frame observed inside an ACK wait window.
    fn record_window_frame(&mut self, frame: AckWindowFrame) {
        self.window_frames_seen = self.window_frames_seen.saturating_add(1);
        match frame {
            AckWindowFrame::Match { .. } => {
                self.ack_frames_seen = self.ack_frames_seen.saturating_add(1);
            }
            AckWindowFrame::ForeignAck { sequence } => {
                self.ack_frames_seen = self.ack_frames_seen.saturating_add(1);
                self.foreign_acks = self.foreign_acks.saturating_add(1);
                self.last_foreign_ack_sequence = Some(sequence);
            }
            AckWindowFrame::NotAnAck => {}
        }
    }

    fn record_window_result(&mut self, matched: bool, expected_sequence: u8) {
        self.ack_windows = self.ack_windows.saturating_add(1);
        if matched {
            self.ack_matched = self.ack_matched.saturating_add(1);
        } else {
            self.ack_windows_expired = self.ack_windows_expired.saturating_add(1);
            self.last_expired_sequence = Some(expected_sequence);
        }
    }

    fn record_tx_attempt(&mut self) {
        self.tx_attempts = self.tx_attempts.saturating_add(1);
    }
}

#[cfg(target_arch = "tc32")]
mod imp {
    use crate::frames::{
        self, build_association_request, build_association_response, build_beacon_request,
        build_data_frame, build_data_request, build_data_request_short, build_nonbeacon_beacon,
        parse_association_response, parse_mac_addresses,
    };
    use crate::pib::{PibAttribute, PibPayload, PibValue};
    use crate::primitives::*;
    use crate::{MacCapabilities, MacDriver, MacError, PlatformServices, WrappingTickExtender};
    use tlsr8258_hal::radio::{
        AckPendingAddress, AckPendingError, MAX_MAC_FRAME_LEN, Radio, RawRxOutcome, ReceivedFrame,
        TX_POWER_MAX_DBM, TX_POWER_MIN_DBM, TxOutcome,
    };
    use tlsr8258_hal::rng::Rng;
    use tlsr8258_hal::{flash, timer};
    use zigbee_types::*;

    /// Hardware AES-128 accelerator, owned by the MAC only when the
    /// `hardware-aes` feature selects the on-silicon CCM*/MMO backend. See
    /// [`TelinkMac::install_aes_engine`] and the `ForwardAesProvider`
    /// override below.
    #[cfg(feature = "hardware-aes")]
    use tlsr8258_hal::aes::{AesEngine, AesError};

    const ACK_WAIT_TICKS: u32 = timer::ms(8);
    const ASSOCIATION_DIRECT_WAIT_TICKS: u32 = timer::ms(500);
    const POST_ASSOCIATION_RX_TICKS: u32 = timer::ms(250);
    const POLL_RESPONSE_WAIT_TICKS: u32 = timer::ms(30);
    const RX_INDICATION_WAIT_TICKS: u32 = timer::ms(5_000);
    const MAX_ASSOCIATION_POLLS: u8 = 32;
    const MAX_RECEIVE_FRAMES: u16 = 32;
    const DEFAULT_MAX_FRAME_RETRIES: u8 = 3;

    #[derive(Clone, Copy, Default)]
    struct AckResult {
        frame_pending: bool,
    }

    #[derive(Clone, Copy)]
    struct AddressFilter {
        pan_id: PanId,
        short_address: ShortAddress,
        extended_address: IeeeAddress,
        promiscuous: bool,
    }

    /// TLSR8258 end-device MAC.
    ///
    /// The application must initialize the TLSR8258 clocks before creating
    /// this value. `new()` reads the factory IEEE address (or the stable
    /// flash-UID fallback) and exclusively acquires the RF block.
    pub struct TelinkMac {
        radio: Radio,
        short_address: ShortAddress,
        pan_id: PanId,
        phy_channel: u8,
        extended_address: IeeeAddress,
        coord_short_address: ShortAddress,
        coord_extended_address: IeeeAddress,
        associated_pan_coord: bool,
        rx_on_when_idle: bool,
        association_permit: bool,
        auto_request: bool,
        beacon_order: u8,
        superframe_order: u8,
        response_wait_time: u8,
        transaction_persistence_time: u16,
        max_frame_total_wait_time: u32,
        dsn: u8,
        bsn: u8,
        beacon_payload: PibPayload,
        max_csma_backoffs: u8,
        min_be: u8,
        max_be: u8,
        max_frame_retries: u8,
        promiscuous: bool,
        tx_power: i8,
        pending_association_response: Option<(ShortAddress, u8)>,
        pending_data: super::DataIndicationQueue,
        pending_events: super::CommandEventQueue,
        pending_outgoing_associations: super::AssociationResponseQueue,
        pending_association_deliveries: heapless::Deque<
            MlmeAssociateResponseDelivery,
            { super::ASSOCIATION_DELIVERY_QUEUE_CAPACITY },
        >,
        clock: WrappingTickExtender,
        /// Lazily acquired on first [`PlatformServices::fill_random`] call
        /// so construction never pays the ADC-sampling cost (or fails)
        /// unless random bytes are actually requested. See
        /// `tlsr8258_hal::rng`'s module docs for exactly what this
        /// provides and does not prove about entropy quality.
        rng: Option<Rng>,
        /// Bounded transmit/acknowledgement counters. Diagnostic only: never
        /// read by the MAC itself, so they cannot change any decision or
        /// timing. See [`super::AckDiagnostics`] for how to read them.
        ack_diagnostics: super::AckDiagnostics,
        /// Hardware AES-128 accelerator, present only under the
        /// `hardware-aes` feature. Installed once by the composition root
        /// via [`TelinkMac::install_aes_engine`] from the board's exclusive
        /// `Aes` token, so there is exactly one engine and no global mutable
        /// alias. `None` only in the window between construction and that
        /// single install call.
        #[cfg(feature = "hardware-aes")]
        aes_engine: Option<AesEngine>,
    }

    impl TelinkMac {
        pub fn new() -> Self {
            let radio = Radio::take().expect("TLSR8258 radio already taken");
            Self::from_radio(radio, None)
        }

        /// Verify `geometry` against the fitted JEDEC flash, then construct
        /// the MAC with the factory EUI-64 from that geometry's Telink
        /// factory sector (or the stable flash-UID fallback).
        ///
        /// Non-512-KiB products should use this constructor instead of
        /// [`Self::new`], whose legacy identity path intentionally remains
        /// fixed to the 512-KiB TB-04 layout.
        pub fn new_for_flash_geometry(
            geometry: flash::FlashGeometry,
        ) -> Result<Self, flash::FlashError> {
            let mut address = [0u8; 8];
            flash::factory_ieee_for(geometry, &mut address)?;
            Ok(Self::with_extended_address(address))
        }

        pub fn with_extended_address(extended_address: IeeeAddress) -> Self {
            let radio = Radio::take().expect("TLSR8258 radio already taken");
            Self::from_radio(radio, Some(extended_address))
        }

        /// Take ownership of the AES-128 accelerator and use it for all
        /// subsequent CCM* and AES-MMO operations (via the
        /// `ForwardAesProvider` override below).
        ///
        /// The composition root calls this exactly once, right after
        /// construction, handing over the board's single exclusive
        /// `Aes` token. Constructing the [`AesEngine`] here enables the
        /// AES clock-gate and pulses its reset once; it is never
        /// re-initialised afterwards. Before accepting the engine, this runs
        /// back-to-back AES-128 known-answer tests against the real peripheral,
        /// including a re-key/reuse cycle. Only compiled under the
        /// `hardware-aes` feature.
        #[cfg(feature = "hardware-aes")]
        pub fn install_aes_engine(
            &mut self,
            aes: tlsr8258_hal::peripherals::Aes,
        ) -> Result<(), AesError> {
            let mut engine = AesEngine::new(aes, AesEngine::DEFAULT_TIMEOUT_ITERATIONS)?;
            engine.self_test()?;
            self.aes_engine = Some(engine);
            Ok(())
        }

        fn from_radio(mut radio: Radio, address: Option<IeeeAddress>) -> Self {
            radio.init();
            let extended_address = match address {
                Some(address) => address,
                None => {
                    let mut address = [0u8; 8];
                    flash::factory_ieee(&mut address);
                    address
                }
            };
            let now_ticks = timer::now_ticks();
            let dsn = now_ticks as u8;
            let mut mac = Self {
                radio,
                short_address: ShortAddress(0xFFFF),
                pan_id: PanId(0xFFFF),
                phy_channel: 11,
                extended_address,
                coord_short_address: ShortAddress(0xFFFF),
                coord_extended_address: [0; 8],
                associated_pan_coord: false,
                rx_on_when_idle: false,
                association_permit: false,
                auto_request: true,
                beacon_order: 15,
                superframe_order: 15,
                response_wait_time: 32,
                transaction_persistence_time: 0x01F4,
                max_frame_total_wait_time: 0,
                dsn,
                bsn: 0,
                beacon_payload: PibPayload::new(),
                max_csma_backoffs: 4,
                min_be: 3,
                max_be: 5,
                max_frame_retries: DEFAULT_MAX_FRAME_RETRIES,
                promiscuous: false,
                tx_power: 0,
                pending_association_response: None,
                pending_data: super::DataIndicationQueue::new(),
                pending_events: super::CommandEventQueue::new(),
                pending_outgoing_associations: super::AssociationResponseQueue::new(),
                pending_association_deliveries: heapless::Deque::new(),
                clock: WrappingTickExtender::new(now_ticks),
                rng: None,
                ack_diagnostics: super::AckDiagnostics::new(),
                #[cfg(feature = "hardware-aes")]
                aes_engine: None,
            };
            mac.apply_radio_config();
            mac
        }

        fn next_dsn(&mut self) -> u8 {
            let sequence = self.dsn;
            self.dsn = self.dsn.wrapping_add(1);
            sequence
        }

        /// Snapshot of the bounded transmit/acknowledgement counters.
        ///
        /// Intended to be read once per observation window (for example
        /// into an application RAM metrics block that a debugger dumps
        /// after a sniffer capture), not polled in a hot loop.
        pub fn ack_diagnostics(&self) -> super::AckDiagnostics {
            self.ack_diagnostics
        }

        /// Snapshot of the HAL receive-path counters that pair with
        /// [`TelinkMac::ack_diagnostics`].
        pub fn rx_diagnostics(&self) -> tlsr8258_hal::radio::RxDiagnostics {
            tlsr8258_hal::radio::rx_diagnostics()
        }

        /// Snapshot of the bounded MAC queue counters.
        ///
        /// Deliberately separate from
        /// [`TelinkMac::rx_diagnostics`]`().queue_*`: the HAL counters cover
        /// the interrupt queue between the radio and this MAC, these cover
        /// the MAC's own retained indications and command events. A frame
        /// lost at one stage must never be attributable to the other.
        pub fn queue_diagnostics(&self) -> super::MacQueueDiagnostics {
            let (data_indication_overflow, data_indication_evicted, data_indication_high_water) =
                self.pending_data.counters();
            super::MacQueueDiagnostics {
                data_indication_overflow,
                data_indication_evicted,
                data_indication_high_water,
                command_event_overflow: self.pending_events.overflow,
                command_event_high_water: self.pending_events.high_water,
            }
        }

        fn next_bsn(&mut self) -> u8 {
            let sequence = self.bsn;
            self.bsn = self.bsn.wrapping_add(1);
            sequence
        }

        fn extended_timer_ticks(&self) -> u64 {
            self.clock.extend(timer::now_ticks())
        }

        fn apply_radio_config(&mut self) {
            self.radio.set_channel(self.channel());
            let _ = self.radio.set_tx_power(self.tx_power);
            self.radio
                .set_ack_filter(self.pan_id.0, self.short_address.0, self.extended_address);
            self.radio.set_rx_on_when_idle(self.rx_on_when_idle);
        }

        /// Quiesce the RF/DMA block before entering TLSR8258 retention sleep.
        ///
        /// Call only after the synchronous MAC operation in progress has
        /// completed. Any frame retained from an ACK window is discarded
        /// because its DMA contents are not valid after wake.
        pub fn prepare_for_sleep(&mut self) {
            self.pending_data.clear();
            self.pending_events.clear();
            self.pending_association_deliveries.clear();
            self.radio.prepare_for_sleep();
        }

        /// Restore the RF/DMA block after TLSR8258 retention wake.
        ///
        /// The PHY initialization resets the channel and hardware filters, so
        /// reapply all PIB-backed radio state before the next MAC operation.
        pub fn resume_after_sleep(&mut self) {
            self.radio.init();
            self.pending_association_response = None;
            self.pending_data.clear();
            self.pending_events.clear();
            self.pending_association_deliveries.clear();
            self.apply_radio_config();
        }

        fn channel(&self) -> u8 {
            self.phy_channel
        }

        fn set_channel(&mut self, channel: u8) -> Result<(), MacError> {
            if !(11..=26).contains(&channel) {
                return Err(MacError::InvalidParameter);
            }
            self.phy_channel = channel;
            self.radio.set_channel(channel);
            Ok(())
        }

        fn address_filter(&self) -> AddressFilter {
            AddressFilter {
                pan_id: self.pan_id,
                short_address: self.short_address,
                extended_address: self.extended_address,
                promiscuous: self.promiscuous,
            }
        }

        fn transmit_with_ack(
            &mut self,
            frame: &[u8],
            sequence: u8,
            ack_requested: bool,
            association_response_child: Option<IeeeAddress>,
        ) -> Result<AckResult, MacError> {
            let attempts = if ack_requested {
                self.max_frame_retries.saturating_add(1)
            } else {
                1
            };
            let mut last_error = MacError::NoAck;

            for _ in 0..attempts {
                match self.radio.transmit(frame) {
                    TxOutcome::Sent => {}
                    TxOutcome::InvalidFrame => return Err(MacError::FrameTooLong),
                    TxOutcome::ChannelAccessFailure => {
                        last_error = MacError::ChannelAccessFailure;
                        continue;
                    }
                    TxOutcome::Timeout => {
                        last_error = MacError::RadioError;
                        continue;
                    }
                }
                self.ack_diagnostics.record_tx_attempt();

                if !ack_requested {
                    return Ok(AckResult::default());
                }

                let mut ack = None;
                let mut association_response = None;
                let mut association_delivery_proven = false;
                let filter = self.address_filter();
                let now_us = self.monotonic_micros();
                let pending_events = &mut self.pending_events;
                let pending_data = &mut self.pending_data;
                let pending_associations = &self.pending_outgoing_associations;
                let diagnostics = &mut self.ack_diagnostics;
                self.radio
                    .receive_raw_until(ACK_WAIT_TICKS, MAX_RECEIVE_FRAMES, |outcome| {
                        let RawRxOutcome::Frame(received) = outcome else {
                            return false;
                        };
                        let data = received.as_slice();
                        if data.len() >= 3 {
                            let classified = super::classify_ack_window_frame(data, sequence);
                            diagnostics.record_window_frame(classified);
                            if let super::AckWindowFrame::Match { frame_pending } = classified {
                                ack = Some(AckResult { frame_pending });
                                return true;
                            }
                            let (_, destination, _, _) = parse_mac_addresses(data);
                            if let Some(response) = parse_association_response(data)
                                && Self::accepts_destination(filter, &destination)
                            {
                                association_response = Some(response);
                            } else if let Some(event) =
                                Self::parse_command_event_for(&received, filter)
                            {
                                let proves_delivery = matches!(
                                    super::pending_association_poll(
                                        &event,
                                        pending_associations,
                                        now_us,
                                    ),
                                    Some(super::PendingAssociationPoll::ConfirmDelivered(child))
                                        if Some(child) == association_response_child
                                );
                                if pending_events
                                    .push_with_association_poll_state(event, proves_delivery)
                                    && proves_delivery
                                {
                                    association_delivery_proven = true;
                                    return true;
                                }
                            } else {
                                let mut candidate_filter = filter;
                                if let Some((short_address, 0)) = association_response {
                                    candidate_filter.short_address = short_address;
                                }
                                if let Some(indication) =
                                    Self::parse_data_indication_for(&received, candidate_filter)
                                {
                                    let priority =
                                        Self::indication_priority(candidate_filter, &indication);
                                    let _ = pending_data.push(indication, priority);
                                }
                            }
                        }
                        false
                    });
                self.ack_diagnostics
                    .record_window_result(ack.is_some(), sequence);
                if association_response.is_some() {
                    self.pending_association_response = association_response;
                }
                if association_delivery_proven {
                    return Ok(AckResult::default());
                }
                if let Some(ack) = ack {
                    return Ok(ack);
                }
                last_error = MacError::NoAck;
            }

            Err(last_error)
        }

        fn take_association_response(&mut self) -> Option<(ShortAddress, u8)> {
            self.pending_association_response.take()
        }

        fn wait_for_association_response(
            &mut self,
            timeout_ticks: u32,
        ) -> Option<(ShortAddress, u8)> {
            if let Some(response) = self.take_association_response() {
                return Some(response);
            }

            let mut response = None;
            let filter = self.address_filter();
            let pending_events = &mut self.pending_events;
            let pending_data = &mut self.pending_data;
            self.radio
                .receive_raw_until(timeout_ticks, MAX_RECEIVE_FRAMES, |outcome| {
                    let RawRxOutcome::Frame(received) = outcome else {
                        return false;
                    };
                    let data = received.as_slice();
                    let (_, destination, _, _) = parse_mac_addresses(data);
                    if Self::accepts_destination(filter, &destination)
                        && let Some(candidate) = parse_association_response(data)
                    {
                        response = Some(candidate);
                        return true;
                    }

                    if let Some(event) = Self::parse_command_event_for(&received, filter) {
                        let _ = pending_events.push(event);
                    } else if let Some(indication) =
                        Self::parse_data_indication_for(&received, filter)
                    {
                        let priority = Self::indication_priority(filter, &indication);
                        let _ = pending_data.push(indication, priority);
                    }
                    false
                });
            if let Some((short_address, 0)) = response {
                self.radio
                    .set_ack_filter(self.pan_id.0, short_address.0, self.extended_address);
            }
            response
        }

        fn capture_post_association_frame(&mut self) {
            let filter = self.address_filter();
            let has_exact = self.pending_data.any_matching(|indication| {
                Self::is_exact_destination(filter, &indication.dst_address)
            });
            if !has_exact {
                let pending_events = &mut self.pending_events;
                let pending_data = &mut self.pending_data;
                self.radio.receive_raw_until(
                    POST_ASSOCIATION_RX_TICKS,
                    MAX_RECEIVE_FRAMES,
                    |outcome| {
                        let RawRxOutcome::Frame(received) = outcome else {
                            return false;
                        };
                        if let Some(event) = Self::parse_command_event_for(&received, filter) {
                            let _ = pending_events.push(event);
                        } else if let Some(indication) =
                            Self::parse_data_indication_for(&received, filter)
                        {
                            let exact = Self::is_exact_destination(filter, &indication.dst_address);
                            let priority = Self::indication_priority(filter, &indication);
                            let _ = pending_data.push(indication, priority);
                            return exact;
                        }
                        false
                    },
                );
            }
        }

        fn finish_association(
            &mut self,
            short_address: ShortAddress,
            status: u8,
        ) -> Result<MlmeAssociateConfirm, MacError> {
            let status = match status {
                0x00 => AssociationStatus::Success,
                0x01 => AssociationStatus::PanAtCapacity,
                _ => AssociationStatus::PanAccessDenied,
            };
            if status == AssociationStatus::Success {
                if short_address.0 >= 0xFFF8 {
                    return Err(MacError::AssociationDenied);
                }
                self.short_address = short_address;
                self.associated_pan_coord = true;
                self.apply_radio_config();
                self.capture_post_association_frame();
            }
            Ok(MlmeAssociateConfirm {
                short_address,
                status,
            })
        }

        fn scan_channel(
            &mut self,
            channel: u8,
            duration_ticks: u64,
            active: bool,
            descriptors: &mut PanDescriptorList,
        ) {
            self.radio.set_channel(channel);
            if active {
                let request = build_beacon_request(self.next_dsn());
                if self.radio.transmit(&request) != TxOutcome::Sent {
                    return;
                }
            }

            let mut remaining = duration_ticks;
            while remaining != 0 {
                let chunk = remaining.min(u32::MAX as u64) as u32;
                let elapsed = self.radio.receive_raw_for(chunk, u16::MAX, |outcome| {
                    let RawRxOutcome::Frame(received) = outcome else {
                        return;
                    };
                    if let Some(descriptor) =
                        frames::parse_beacon(channel, received.as_slice(), received.lqi)
                    {
                        super::upsert_pan_descriptor(descriptors, descriptor);
                    }
                });
                remaining = remaining.saturating_sub(elapsed.max(1) as u64);
            }
        }

        fn prune_expired_association_responses(&mut self) {
            let now_us = self.monotonic_micros();
            while let Some(response) = self.pending_outgoing_associations.pop_expired(now_us) {
                let _ = self.radio.set_ack_frame_pending(
                    AckPendingAddress::Extended {
                        pan_id: self.pan_id.0,
                        address: response.device_address,
                    },
                    false,
                );
                if self
                    .pending_association_deliveries
                    .push_back(MlmeAssociateResponseDelivery {
                        device_address: response.device_address,
                        short_address: response.short_address,
                        status: response.status,
                        result: Err(MacError::TransactionExpired),
                    })
                    .is_err()
                {
                    log::error!("[MAC] Association delivery queue overflow");
                }
            }
        }

        fn deliver_association_response(&mut self, child: IeeeAddress) {
            self.prune_expired_association_responses();
            let now_us = self.monotonic_micros();
            let Some(response) = self
                .pending_outgoing_associations
                .active_for(&child, now_us)
            else {
                return;
            };
            let sequence = self.next_dsn();
            let Ok(frame) = build_association_response(
                sequence,
                self.pan_id,
                &self.extended_address,
                &response,
            ) else {
                let response = self.pending_outgoing_associations.remove(&child);
                let _ = self.radio.set_ack_frame_pending(
                    AckPendingAddress::Extended {
                        pan_id: self.pan_id.0,
                        address: child,
                    },
                    false,
                );
                if let Some(response) = response
                    && self
                        .pending_association_deliveries
                        .push_back(MlmeAssociateResponseDelivery {
                            device_address: response.device_address,
                            short_address: response.short_address,
                            status: response.status,
                            result: Err(MacError::FrameTooLong),
                        })
                        .is_err()
                {
                    log::error!("[MAC] Association delivery queue overflow");
                }
                return;
            };

            // Keep the transaction queued after a failed attempt so a later
            // poll can retry it until macTransactionPersistenceTime expires.
            if self
                .transmit_with_ack(&frame, sequence, true, Some(child))
                .is_ok()
            {
                self.confirm_association_response_delivery(child);
            }
        }

        fn confirm_association_response_delivery(&mut self, child: IeeeAddress) {
            let Some(delivery) = super::take_successful_association_delivery(
                &mut self.pending_outgoing_associations,
                &child,
            ) else {
                return;
            };
            let _ = self.radio.set_ack_frame_pending(
                AckPendingAddress::Extended {
                    pan_id: self.pan_id.0,
                    address: child,
                },
                false,
            );
            if self
                .pending_association_deliveries
                .push_back(delivery)
                .is_err()
            {
                log::error!("[MAC] Association delivery queue overflow");
            }
        }

        fn receive_data_indication(&mut self, timeout_ticks: u32) -> Option<McpsDataIndication> {
            let filter = self.address_filter();
            let started = timer::now_ticks();
            loop {
                self.prune_expired_association_responses();
                // An Association Response ACK wait can retain normal data.
                // Recheck on every iteration, including after delivering the
                // response to a matching child poll.
                if let Some(indication) = super::take_pending_data(&mut self.pending_data) {
                    return Some(indication);
                }
                let elapsed = timer::now_ticks().wrapping_sub(started);
                let remaining = timeout_ticks.checked_sub(elapsed)?;
                if remaining == 0 {
                    return None;
                }

                let mut indication = None;
                let mut association_poll = None;
                let mut data_request_queued = false;
                let now_us = self.monotonic_micros();
                let receive_ticks = self
                    .pending_outgoing_associations
                    .micros_until_next_expiry(now_us)
                    .map_or(remaining, |until_expiry_us| {
                        remaining.min(timer::us(until_expiry_us).max(1))
                    });
                let pending_events = &mut self.pending_events;
                let pending_associations = &self.pending_outgoing_associations;
                self.radio
                    .receive_raw_until(receive_ticks, MAX_RECEIVE_FRAMES, |outcome| {
                        let RawRxOutcome::Frame(received) = outcome else {
                            return false;
                        };
                        if let Some(event) = Self::parse_command_event_for(&received, filter) {
                            let association = super::pending_association_poll(
                                &event,
                                pending_associations,
                                now_us,
                            );
                            // The HAL has already sent any requested MAC ACK
                            // before this callback runs. Ending the slice here
                            // preserves the command for immediate runtime
                            // service without shortening the ACK turnaround.
                            let end_receive = super::queue_command_for_parent_receive(
                                pending_events,
                                event,
                                association.is_some(),
                            );
                            if end_receive {
                                association_poll = association;
                                data_request_queued = true;
                            }
                            return end_receive;
                        }
                        if let Some(candidate) = Self::parse_data_indication_for(&received, filter)
                        {
                            indication = Some(candidate);
                            return true;
                        }
                        false
                    });
                if data_request_queued {
                    match association_poll {
                        Some(super::PendingAssociationPoll::RetryResponse(child)) => {
                            self.deliver_association_response(child);
                            continue;
                        }
                        Some(super::PendingAssociationPoll::ConfirmDelivered(child)) => {
                            self.confirm_association_response_delivery(child);
                            return None;
                        }
                        None => return None,
                    }
                }
                if indication.is_some() {
                    return indication;
                }
            }
        }

        fn receive_poll_response(&mut self, timeout_ticks: u32) -> Option<McpsDataIndication> {
            let filter = self.address_filter();
            let started = timer::now_ticks();
            loop {
                self.prune_expired_association_responses();
                // `deliver_association_response()` performs its own ACK wait,
                // which may retain this poll's response. Preserve unrelated
                // queued data and recheck the exact destination each time.
                if let Some(indication) = super::take_pending_poll_data(
                    &mut self.pending_data,
                    filter.pan_id,
                    filter.short_address,
                    &filter.extended_address,
                ) {
                    return Some(indication);
                }
                let elapsed = timer::now_ticks().wrapping_sub(started);
                let remaining = timeout_ticks.checked_sub(elapsed)?;
                if remaining == 0 {
                    return None;
                }

                let mut indication = None;
                let mut association_poll = None;
                let mut data_request_queued = false;
                let now_us = self.monotonic_micros();
                let receive_ticks = self
                    .pending_outgoing_associations
                    .micros_until_next_expiry(now_us)
                    .map_or(remaining, |until_expiry_us| {
                        remaining.min(timer::us(until_expiry_us).max(1))
                    });
                let pending_events = &mut self.pending_events;
                let pending_data = &mut self.pending_data;
                let pending_associations = &self.pending_outgoing_associations;
                self.radio
                    .receive_raw_until(receive_ticks, MAX_RECEIVE_FRAMES, |outcome| {
                        let RawRxOutcome::Frame(received) = outcome else {
                            return false;
                        };
                        if let Some(event) = Self::parse_command_event_for(&received, filter) {
                            let association = super::pending_association_poll(
                                &event,
                                pending_associations,
                                now_us,
                            );
                            let end_receive = super::queue_command_for_parent_receive(
                                pending_events,
                                event,
                                association.is_some(),
                            );
                            if end_receive {
                                association_poll = association;
                                data_request_queued = true;
                            }
                            return end_receive;
                        }
                        if let Some(candidate) = Self::parse_data_indication_for(&received, filter)
                        {
                            if Self::is_exact_destination(filter, &candidate.dst_address) {
                                indication = Some(candidate);
                                return true;
                            }
                            let priority = Self::indication_priority(filter, &candidate);
                            let _ = pending_data.push(candidate, priority);
                        }
                        false
                    });
                if data_request_queued {
                    match association_poll {
                        Some(super::PendingAssociationPoll::RetryResponse(child)) => {
                            self.deliver_association_response(child);
                            continue;
                        }
                        Some(super::PendingAssociationPoll::ConfirmDelivered(child)) => {
                            self.confirm_association_response_delivery(child);
                            return None;
                        }
                        None => return None,
                    }
                }
                if indication.is_some() {
                    return indication;
                }
            }
        }

        fn receive_command_event(&mut self, timeout_ticks: u32) -> Option<MacCommandEvent> {
            self.prune_expired_association_responses();
            if let Some(delivery) = self.pending_association_deliveries.pop_front() {
                return Some(MacCommandEvent::AssociationResponseDelivery(delivery));
            }
            if let Some(mut queued) = self.pending_events.pop_queued() {
                match super::queued_association_poll_to_service(
                    &queued,
                    &self.pending_outgoing_associations,
                    self.monotonic_micros(),
                ) {
                    Some(super::PendingAssociationPoll::RetryResponse(child)) => {
                        self.deliver_association_response(child);
                    }
                    Some(super::PendingAssociationPoll::ConfirmDelivered(child)) => {
                        self.confirm_association_response_delivery(child);
                        if let Some(delivery) = self.pending_association_deliveries.pop_front() {
                            queued.association_poll_serviced = true;
                            let restored = self.pending_events.push_queued_front(queued);
                            debug_assert!(restored);
                            return Some(MacCommandEvent::AssociationResponseDelivery(delivery));
                        }
                    }
                    None => {}
                }
                return Some(queued.event);
            }

            let filter = self.address_filter();
            let started = timer::now_ticks();
            loop {
                self.prune_expired_association_responses();
                let elapsed = timer::now_ticks().wrapping_sub(started);
                let remaining = timeout_ticks.checked_sub(elapsed)?;
                if remaining == 0 {
                    return None;
                }

                let mut event = None;
                let now_us = self.monotonic_micros();
                let receive_ticks = self
                    .pending_outgoing_associations
                    .micros_until_next_expiry(now_us)
                    .map_or(remaining, |until_expiry_us| {
                        remaining.min(timer::us(until_expiry_us).max(1))
                    });
                let pending_data = &mut self.pending_data;
                self.radio
                    .receive_raw_until(receive_ticks, MAX_RECEIVE_FRAMES, |outcome| {
                        let RawRxOutcome::Frame(received) = outcome else {
                            return false;
                        };
                        if let Some(candidate) = Self::parse_command_event_for(&received, filter) {
                            event = Some(candidate);
                            return true;
                        }
                        if let Some(indication) = Self::parse_data_indication_for(&received, filter)
                        {
                            let priority = Self::indication_priority(filter, &indication);
                            let _ = pending_data.push(indication, priority);
                        }
                        false
                    });
                if let Some(event) = event {
                    match super::pending_association_poll(
                        &event,
                        &self.pending_outgoing_associations,
                        self.monotonic_micros(),
                    ) {
                        Some(super::PendingAssociationPoll::RetryResponse(child)) => {
                            self.deliver_association_response(child);
                        }
                        Some(super::PendingAssociationPoll::ConfirmDelivered(child)) => {
                            self.confirm_association_response_delivery(child);
                            if let Some(delivery) = self.pending_association_deliveries.pop_front()
                            {
                                let queued = self
                                    .pending_events
                                    .push_with_association_poll_state(event, true);
                                debug_assert!(queued);
                                return Some(MacCommandEvent::AssociationResponseDelivery(
                                    delivery,
                                ));
                            }
                        }
                        None => {}
                    }
                    return Some(event);
                }
            }
        }

        fn parse_data_indication_for(
            received: &ReceivedFrame,
            filter: AddressFilter,
        ) -> Option<McpsDataIndication> {
            let super::ParsedIncomingFrame::Data(indication) =
                super::parse_incoming_frame(received.as_slice(), received.lqi)?
            else {
                return None;
            };
            Self::accepts_destination(filter, &indication.dst_address).then_some(indication)
        }

        fn parse_command_event_for(
            received: &ReceivedFrame,
            filter: AddressFilter,
        ) -> Option<MacCommandEvent> {
            let parsed = super::parse_incoming_frame(received.as_slice(), received.lqi)?;
            if !Self::accepts_destination(filter, parsed.destination()) {
                return None;
            }
            let super::ParsedIncomingFrame::Command(event) = parsed else {
                return None;
            };
            Some(event)
        }

        fn accepts_destination(filter: AddressFilter, destination: &MacAddress) -> bool {
            super::accepts_telink_destination(
                filter.pan_id,
                filter.short_address,
                &filter.extended_address,
                filter.promiscuous,
                destination,
            )
        }

        fn is_exact_destination(filter: AddressFilter, destination: &MacAddress) -> bool {
            match destination {
                MacAddress::Short(pan, address) => {
                    pan.0 == filter.pan_id.0 && address.0 == filter.short_address.0
                }
                MacAddress::Extended(pan, address) => {
                    pan.0 == filter.pan_id.0 && *address == filter.extended_address
                }
            }
        }

        /// Retention class of `indication` for the bounded
        /// [`super::DataIndicationQueue`]'s overload policy.
        fn indication_priority(
            filter: AddressFilter,
            indication: &McpsDataIndication,
        ) -> super::IndicationPriority {
            super::indication_priority(
                filter.pan_id,
                filter.short_address,
                &filter.extended_address,
                &indication.dst_address,
            )
        }

        fn transmit_data_request(
            &mut self,
            req: McpsDataRequest<'_>,
        ) -> Result<McpsDataConfirm, MacError> {
            if req.tx_options.security_enabled {
                return Err(MacError::SecurityError);
            }
            if req.src_addr_mode == AddressMode::Short && self.short_address.0 >= 0xFFF8 {
                return Err(MacError::InvalidParameter);
            }
            let sequence = self.next_dsn();
            let frame = build_data_frame(
                sequence,
                req.src_addr_mode,
                self.short_address,
                &self.extended_address,
                &req.dst_address,
                req.payload,
                req.tx_options.ack_tx,
                req.tx_options.frame_pending,
            )
            .map_err(|error| match error {
                frames::FrameBuildError::FrameTooLong => MacError::FrameTooLong,
                frames::FrameBuildError::InvalidParameter => MacError::InvalidParameter,
            })?;
            self.transmit_with_ack(&frame, sequence, req.tx_options.ack_tx, None)?;
            Ok(McpsDataConfirm {
                msdu_handle: req.msdu_handle,
                timestamp: Some(timer::now_ticks()),
            })
        }

        fn clear_transient_state(&mut self) {
            self.pending_association_response = None;
            super::clear_transient_queues(
                &mut self.pending_data,
                &mut self.pending_events,
                &mut self.pending_outgoing_associations,
            );
            self.pending_association_deliveries.clear();
            self.radio.clear_ack_frame_pending();
        }

        fn clear_association(&mut self) {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.coord_short_address = ShortAddress(0xFFFF);
            self.coord_extended_address = [0; 8];
            self.associated_pan_coord = false;
            self.clear_transient_state();
            self.apply_radio_config();
        }
    }

    impl MacDriver for TelinkMac {
        async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
            if req.scan_duration > 14 {
                return Err(MacError::InvalidParameter);
            }
            let saved_channel = self.channel();
            let duration_ticks = crate::pib::scan_duration_us(req.scan_duration)
                .saturating_mul((timer::TICKS_PER_MS / 1_000) as u64);
            let mut pan_descriptors = PanDescriptorList::new();
            let mut energy_list = EdList::new();

            for channel in req.channel_mask.iter() {
                let number = channel.number();
                match req.scan_type {
                    ScanType::Active => {
                        self.scan_channel(number, duration_ticks, true, &mut pan_descriptors);
                    }
                    ScanType::Passive => {
                        self.scan_channel(number, duration_ticks, false, &mut pan_descriptors);
                    }
                    ScanType::Ed => {
                        self.radio.set_channel(number);
                        let _ = energy_list.push(EdValue {
                            channel: number,
                            energy: self.radio.measure_energy(),
                        });
                    }
                    ScanType::Orphan => {
                        self.radio.set_channel(saved_channel);
                        return Err(MacError::Unsupported);
                    }
                }
            }
            self.radio.set_channel(saved_channel);

            if matches!(req.scan_type, ScanType::Active | ScanType::Passive)
                && pan_descriptors.is_empty()
            {
                return Err(MacError::NoBeacon);
            }
            Ok(MlmeScanConfirm {
                scan_type: req.scan_type,
                pan_descriptors,
                energy_list,
            })
        }

        async fn mlme_associate(
            &mut self,
            req: MlmeAssociateRequest,
        ) -> Result<MlmeAssociateConfirm, MacError> {
            let MacAddress::Short(pan_id, coordinator) = req.coord_address else {
                return Err(MacError::Unsupported);
            };
            self.clear_transient_state();
            self.set_channel(req.channel)?;
            self.pan_id = pan_id;
            self.coord_short_address = coordinator;
            self.short_address = ShortAddress(0xFFFF);
            self.apply_radio_config();

            let coordinator_address = MacAddress::Short(pan_id, coordinator);
            let sequence = self.next_dsn();
            let request = build_association_request(
                sequence,
                &coordinator_address,
                &self.extended_address,
                &req.capability_info,
            );
            let tx = self.transmit_with_ack(&request, sequence, true, None);
            if let Some((short_address, status)) = self.take_association_response() {
                return self.finish_association(short_address, status);
            }
            tx?;
            if let Some((short_address, status)) =
                self.wait_for_association_response(ASSOCIATION_DIRECT_WAIT_TICKS)
            {
                return self.finish_association(short_address, status);
            }

            for _ in 0..MAX_ASSOCIATION_POLLS {
                let sequence = self.next_dsn();
                let poll =
                    build_data_request(sequence, &coordinator_address, &self.extended_address);
                let tx = self.transmit_with_ack(&poll, sequence, true, None);
                if let Some((short_address, status)) = self.take_association_response() {
                    return self.finish_association(short_address, status);
                }
                let _ack = tx?;
                // Some parents transmit the pending Association Response
                // immediately after the Data Request ACK even when the ACK's
                // frame-pending bit is already clear. Keep RX open for the
                // response on every successful poll.
                if let Some((short_address, status)) =
                    self.wait_for_association_response(POLL_RESPONSE_WAIT_TICKS)
                {
                    return self.finish_association(short_address, status);
                }
                timer::sleep_ticks(timer::ms(100));
            }
            Err(MacError::NoData)
        }

        async fn mlme_associate_response(
            &mut self,
            rsp: MlmeAssociateResponse,
        ) -> Result<(), MacError> {
            if self.pan_id == PanId::BROADCAST {
                return Err(MacError::InvalidParameter);
            }
            // Validate the eventual wire frame now, but retain the response
            // as an indirect transaction until this EUI-64 polls.
            build_association_response(self.dsn, self.pan_id, &self.extended_address, &rsp)
                .map_err(|error| match error {
                    frames::FrameBuildError::FrameTooLong => MacError::FrameTooLong,
                    frames::FrameBuildError::InvalidParameter => MacError::InvalidParameter,
                })?;
            self.prune_expired_association_responses();
            let child = rsp.device_address;
            if !self.pending_outgoing_associations.can_enqueue(&child) {
                return Err(MacError::TransactionOverflow);
            }
            self.radio
                .set_ack_frame_pending(
                    AckPendingAddress::Extended {
                        pan_id: self.pan_id.0,
                        address: child,
                    },
                    true,
                )
                .map_err(|error| match error {
                    AckPendingError::InvalidAddress => MacError::InvalidParameter,
                    AckPendingError::TableFull => MacError::TransactionOverflow,
                })?;
            let now_us = self.monotonic_micros();
            if self
                .pending_outgoing_associations
                .enqueue(rsp, now_us, self.transaction_persistence_time)
                .is_err()
            {
                let _ = self.radio.set_ack_frame_pending(
                    AckPendingAddress::Extended {
                        pan_id: self.pan_id.0,
                        address: child,
                    },
                    false,
                );
                return Err(MacError::TransactionOverflow);
            }
            Ok(())
        }

        async fn mlme_beacon_response(&mut self, rsp: MlmeBeaconResponse) -> Result<(), MacError> {
            let source = if self.short_address.0 < 0xFFF8 {
                MacAddress::Short(self.pan_id, self.short_address)
            } else {
                MacAddress::Extended(self.pan_id, self.extended_address)
            };
            let sequence = self.next_bsn();
            let frame = build_nonbeacon_beacon(
                sequence,
                &source,
                rsp.pan_coordinator,
                rsp.association_permit,
                &rsp.pending_short_addresses,
                &rsp.pending_extended_addresses,
                rsp.beacon_payload.as_slice(),
            )
            .map_err(|error| match error {
                frames::FrameBuildError::FrameTooLong => MacError::FrameTooLong,
                frames::FrameBuildError::InvalidParameter => MacError::InvalidParameter,
            })?;
            match self.radio.transmit(&frame) {
                TxOutcome::Sent => Ok(()),
                TxOutcome::InvalidFrame => Err(MacError::FrameTooLong),
                TxOutcome::ChannelAccessFailure => Err(MacError::ChannelAccessFailure),
                TxOutcome::Timeout => Err(MacError::RadioError),
            }
        }

        async fn mlme_disassociate(
            &mut self,
            _req: MlmeDisassociateRequest,
        ) -> Result<(), MacError> {
            self.clear_association();
            Ok(())
        }

        fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
            self.clear_transient_state();
            if set_default_pib {
                self.clear_association();
                self.phy_channel = 11;
                self.rx_on_when_idle = false;
                self.association_permit = false;
                self.auto_request = true;
                self.beacon_order = 15;
                self.superframe_order = 15;
                self.response_wait_time = 32;
                self.transaction_persistence_time = 0x01F4;
                self.max_frame_total_wait_time = 0;
                self.dsn = timer::now_ticks() as u8;
                self.bsn = 0;
                self.beacon_payload = PibPayload::new();
                self.max_csma_backoffs = 4;
                self.min_be = 3;
                self.max_be = 5;
                self.max_frame_retries = DEFAULT_MAX_FRAME_RETRIES;
                self.promiscuous = false;
                self.tx_power = 0;
            }
            self.apply_radio_config();
            Ok(())
        }

        /// EXPERIMENTAL router/parent start.
        ///
        /// Called by `zigbee_nwk::Nlme::nlme_start_router()` after this
        /// device has already joined a network with the router capability
        /// bit set. Validates the non-beacon, non-coordinator parameter
        /// shape via [`super::validate_router_start`], then:
        ///
        /// 1. Applies `req.channel` and `req.pan_id` to the radio.
        /// 2. Sets `macRxOnWhenIdle = true` so the radio stays in continuous
        ///    RX and can relay unicast/broadcast NWK traffic and rebroadcast
        ///    route-request frames.
        ///
        /// `MLME-START` itself does not schedule periodic beacons or allocate
        /// an indirect queue. On-demand beacon and parent response primitives
        /// are separate operations. Any caller requesting beaconed
        /// superframes or PAN-coordinator start receives
        /// `MacError::Unsupported`.
        async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
            super::validate_router_start(&req)?;
            self.set_channel(req.channel)?;
            self.pan_id = req.pan_id;
            self.rx_on_when_idle = true;
            self.apply_radio_config();
            Ok(())
        }

        async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
            use PibAttribute::*;
            match attr {
                MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
                MacPanId => Ok(PibValue::PanId(self.pan_id)),
                MacExtendedAddress => Ok(PibValue::ExtendedAddress(self.extended_address)),
                MacCoordShortAddress => Ok(PibValue::ShortAddress(self.coord_short_address)),
                MacCoordExtendedAddress => {
                    Ok(PibValue::ExtendedAddress(self.coord_extended_address))
                }
                MacAssociatedPanCoord => Ok(PibValue::Bool(self.associated_pan_coord)),
                MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
                MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
                MacBeaconOrder => Ok(PibValue::U8(self.beacon_order)),
                MacSuperframeOrder => Ok(PibValue::U8(self.superframe_order)),
                MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
                MacBeaconPayloadLength => {
                    Ok(PibValue::U8(self.beacon_payload.as_slice().len() as u8))
                }
                MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
                MacMaxCsmaBackoffs => Ok(PibValue::U8(self.max_csma_backoffs)),
                MacMinBe => Ok(PibValue::U8(self.min_be)),
                MacMaxBe => Ok(PibValue::U8(self.max_be)),
                MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
                MacMaxFrameTotalWaitTime => Ok(PibValue::U32(self.max_frame_total_wait_time)),
                MacResponseWaitTime => Ok(PibValue::U8(self.response_wait_time)),
                MacDsn => Ok(PibValue::U8(self.dsn)),
                MacBsn => Ok(PibValue::U8(self.bsn)),
                MacTransactionPersistenceTime => {
                    Ok(PibValue::U16(self.transaction_persistence_time))
                }
                MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
                PhyCurrentChannel => Ok(PibValue::U8(self.channel())),
                PhyChannelsSupported => Ok(PibValue::U32(ChannelMask::ALL_2_4GHZ.0)),
                PhyTransmitPower => Ok(PibValue::I8(self.tx_power)),
                PhyCcaMode => Ok(PibValue::U8(1)),
                PhyCurrentPage => Ok(PibValue::U8(0)),
            }
        }

        async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
            use PibAttribute::*;
            match (attr, value) {
                (MacShortAddress, PibValue::ShortAddress(value)) => {
                    self.short_address = value;
                    self.apply_radio_config();
                }
                (MacPanId, PibValue::PanId(value)) => {
                    self.pan_id = value;
                    self.apply_radio_config();
                }
                (MacExtendedAddress, PibValue::ExtendedAddress(value)) => {
                    self.extended_address = value;
                    self.apply_radio_config();
                }
                (MacCoordShortAddress, PibValue::ShortAddress(value)) => {
                    self.coord_short_address = value;
                }
                (MacCoordExtendedAddress, PibValue::ExtendedAddress(value)) => {
                    self.coord_extended_address = value;
                }
                (MacAssociatedPanCoord, PibValue::Bool(value)) => {
                    self.associated_pan_coord = value;
                }
                (MacRxOnWhenIdle, PibValue::Bool(value)) => {
                    self.rx_on_when_idle = value;
                    self.radio.set_rx_on_when_idle(value);
                }
                (MacAssociationPermit, PibValue::Bool(value)) => {
                    self.association_permit = value;
                }
                (MacBeaconOrder, PibValue::U8(value)) => self.beacon_order = value,
                (MacSuperframeOrder, PibValue::U8(value)) => {
                    self.superframe_order = value;
                }
                (MacBeaconPayload, PibValue::Payload(value)) => {
                    self.beacon_payload = value;
                }
                (MacBeaconPayloadLength, PibValue::U8(value))
                    if value as usize == self.beacon_payload.as_slice().len() => {}
                (MacAutoRequest, PibValue::Bool(value)) => self.auto_request = value,
                (MacMaxCsmaBackoffs, PibValue::U8(value)) => {
                    self.max_csma_backoffs = value;
                }
                (MacMinBe, PibValue::U8(value)) if value <= 8 => self.min_be = value,
                (MacMaxBe, PibValue::U8(value)) if value <= 8 => self.max_be = value,
                (MacMaxFrameRetries, PibValue::U8(value)) => {
                    self.max_frame_retries = value;
                }
                (MacMaxFrameTotalWaitTime, PibValue::U32(value)) => {
                    self.max_frame_total_wait_time = value;
                }
                (MacResponseWaitTime, PibValue::U8(value)) => {
                    self.response_wait_time = value;
                }
                (MacDsn, PibValue::U8(value)) => self.dsn = value,
                (MacBsn, PibValue::U8(value)) => self.bsn = value,
                (MacTransactionPersistenceTime, PibValue::U16(value)) => {
                    self.transaction_persistence_time = value;
                }
                (MacPromiscuousMode, PibValue::Bool(value)) => {
                    self.promiscuous = value;
                }
                (PhyCurrentChannel, PibValue::U8(value)) => self.set_channel(value)?,
                (PhyTransmitPower, PibValue::I8(value)) => {
                    if !self.radio.set_tx_power(value) {
                        return Err(MacError::InvalidParameter);
                    }
                    self.tx_power = value;
                }
                (PhyCcaMode, PibValue::U8(1)) | (PhyCurrentPage, PibValue::U8(0)) => {}
                _ => return Err(MacError::InvalidParameter),
            }
            Ok(())
        }

        async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
            self.mlme_poll_timeout(
                (POLL_RESPONSE_WAIT_TICKS / u32::from(timer::TICKS_PER_US)).max(1),
            )
            .await
        }

        async fn mlme_poll_timeout(
            &mut self,
            timeout_us: u32,
        ) -> Result<Option<MacFrame>, MacError> {
            if self.coord_short_address.0 == 0xFFFF {
                return Err(MacError::InvalidParameter);
            }
            let started = self.monotonic_micros();
            let coordinator = MacAddress::Short(self.pan_id, self.coord_short_address);
            let sequence = self.next_dsn();
            let request = build_data_request_short(sequence, &coordinator, self.short_address);
            let ack = self.transmit_with_ack(&request, sequence, true, None)?;

            if let Some(indication) = self.receive_poll_response(1) {
                return Ok(Some(indication.payload));
            }
            if !ack.frame_pending {
                return Ok(None);
            }
            let elapsed = self.monotonic_micros().wrapping_sub(started);
            let Some(remaining_us) = timeout_us.checked_sub(elapsed) else {
                return Ok(None);
            };
            Ok(self
                .receive_poll_response(timer::us(remaining_us))
                .map(|indication| indication.payload))
        }

        async fn mcps_data(
            &mut self,
            req: McpsDataRequest<'_>,
        ) -> Result<McpsDataConfirm, MacError> {
            if req.tx_options.indirect {
                return Err(MacError::Unsupported);
            }
            self.transmit_data_request(req)
        }

        fn set_indirect_data_pending(
            &mut self,
            child: MacAddress,
            pending: bool,
        ) -> Result<(), MacError> {
            if child.pan_id() != self.pan_id || self.pan_id == PanId::BROADCAST {
                return Err(MacError::InvalidParameter);
            }
            let child = match child {
                MacAddress::Short(pan_id, address) => AckPendingAddress::Short {
                    pan_id: pan_id.0,
                    address: address.0,
                },
                MacAddress::Extended(pan_id, address) => AckPendingAddress::Extended {
                    pan_id: pan_id.0,
                    address,
                },
            };
            self.radio
                .set_ack_frame_pending(child, pending)
                .map_err(|error| match error {
                    AckPendingError::InvalidAddress => MacError::InvalidParameter,
                    AckPendingError::TableFull => MacError::TransactionOverflow,
                })
        }

        async fn mcps_indirect_data(
            &mut self,
            req: McpsDataRequest<'_>,
        ) -> Result<McpsDataConfirm, MacError> {
            if !req.tx_options.indirect
                || req.dst_address.pan_id() != self.pan_id
                || !super::is_unicast_address(&req.dst_address)
            {
                return Err(MacError::InvalidParameter);
            }
            self.transmit_data_request(req)
        }

        async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
            self.receive_data_indication(RX_INDICATION_WAIT_TICKS)
                .ok_or(MacError::NoData)
        }

        async fn mcps_data_indication_timeout(
            &mut self,
            timeout_us: u32,
        ) -> Result<McpsDataIndication, MacError> {
            self.receive_data_indication(timer::us(timeout_us))
                .ok_or(MacError::NoData)
        }

        async fn mac_command_event(&mut self) -> Result<MacCommandEvent, MacError> {
            self.receive_command_event(RX_INDICATION_WAIT_TICKS)
                .ok_or(MacError::NoData)
        }

        async fn mac_command_event_timeout(
            &mut self,
            timeout_us: u32,
        ) -> Result<MacCommandEvent, MacError> {
            self.receive_command_event(timer::us(timeout_us))
                .ok_or(MacError::NoData)
        }

        fn capabilities(&self) -> MacCapabilities {
            MacCapabilities {
                coordinator: false,
                router: true,
                hardware_security: false,
                max_payload: (MAX_MAC_FRAME_LEN - 23) as u16,
                tx_power_min: TxPower(TX_POWER_MIN_DBM),
                tx_power_max: TxPower(TX_POWER_MAX_DBM),
            }
        }
    }

    /// Generic software provider when `hardware-aes` is off. The production
    /// Telink sensor and router always enable the hardware feature.
    #[cfg(not(feature = "hardware-aes"))]
    impl zigbee_crypto::ForwardAesProvider for TelinkMac {}

    /// Hardware AES-128 backend for CCM* and AES-MMO. Hands back a
    /// [`zigbee_crypto::tlsr8258::HardwareAes128`] borrowing this MAC's
    /// exclusively-owned [`AesEngine`], so the RustCrypto software core is
    /// dead-code-eliminated from the image. A missing engine (composition
    /// root failed to call [`TelinkMac::install_aes_engine`]) is a firmware
    /// wiring bug and traps loudly rather than silently falling back to
    /// software.
    #[cfg(feature = "hardware-aes")]
    impl zigbee_crypto::ForwardAesProvider for TelinkMac {
        fn forward_cipher(
            &mut self,
            key: &zigbee_crypto::AesKey,
        ) -> impl zigbee_crypto::Aes128Forward + '_ {
            let engine = self
                .aes_engine
                .as_mut()
                .expect("AES engine not installed: call TelinkMac::install_aes_engine()");
            zigbee_crypto::tlsr8258::HardwareAes128::new(engine, *key)
        }
    }

    impl PlatformServices for TelinkMac {
        fn monotonic_micros(&self) -> u32 {
            (self.extended_timer_ticks() / u64::from(timer::TICKS_PER_US)) as u32
        }

        async fn delay_micros(&mut self, duration_us: u32) {
            timer::sleep_ticks(timer::us(duration_us));
        }

        fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
            // `tlsr8258_hal::rng::Rng` seeds a NIST SP 800-90A CTR_DRBG
            // (AES-128, no derivation function, verified against an
            // official NIST CAVP known-answer test) from repeated ADC
            // noise sampling of the VBAT/GND channel, conditioned through
            // SHA-256 before use. See that module's docs for exactly what
            // this does and does not prove: the DRBG algorithm is a
            // reviewable, KAT-verified standard construction, but the
            // *entropy quality feeding its seed* has not been
            // independently measured on real hardware (no SP 800-90B
            // assessment was performed) — that remains this path's one
            // honest, hardware-only caveat. This is treated as a hardware
            // entropy source per `PlatformServices::fill_random`'s
            // contract, not as predictable vendor MWC (`rand()`) output,
            // which this backend never exposes through this API.
            //
            // Every failure path below — `Rng::take` (initial ADC
            // harvest), and `fill_bytes` (periodic reseed harvest) —
            // propagates as `Err` rather than ever silently falling back
            // to a weaker source or claiming success with unfilled/
            // partially-filled `output`. `tlsr8258_hal::rng::RngError`'s
            // richer cases (already-taken singleton, ADC analog-bus
            // failure, a wholly stuck noise channel, or a failed post-
            // harvest ADC state restore) are collapsed to
            // `MacError::Unsupported` here only because `MacError` has no
            // dedicated entropy-source variant today (`Unsupported` is
            // this trait's own documented case for "no usable hardware
            // entropy source" — reused here for "one exists but could not
            // be used for this call", not conflated with predictable
            // output). A caller that needs the finer-grained reason should
            // go through `tlsr8258_hal::rng` directly.
            if self.rng.is_none() {
                self.rng = Some(Rng::take().map_err(|_| MacError::Unsupported)?);
            }
            let rng = self.rng.as_mut().expect("just initialized above");
            rng.fill_bytes(output).map_err(|_| MacError::Unsupported)
        }
    }
}

#[cfg(target_arch = "tc32")]
pub use imp::TelinkMac;

// The Telink soft-MAC genuinely implements the parent-side primitives
// (`mlme_beacon_response`, `set_indirect_data_pending`, `mcps_indirect_data`,
// `mac_command_event`), so it may back a router/parent role. Only the real
// tc32 backend implements `MacDriver`; the host placeholder below does not.
#[cfg(target_arch = "tc32")]
impl crate::ParentMacDriver for TelinkMac {}

#[cfg(target_arch = "tc32")]
impl crate::sealed::SealedParent for TelinkMac {}

#[cfg(not(target_arch = "tc32"))]
pub struct TelinkMac;

#[cfg(not(target_arch = "tc32"))]
impl TelinkMac {
    pub const fn new() -> Self {
        Self
    }

    pub const fn with_extended_address(_extended_address: [u8; 8]) -> Self {
        Self
    }
}

#[cfg(not(target_arch = "tc32"))]
impl Default for TelinkMac {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{
        build_association_request, build_beacon_request, build_data_frame, build_data_request,
        build_data_request_short,
    };
    use crate::primitives::{MAX_PAN_DESCRIPTORS, SuperframeSpec, ZigbeeBeaconPayload};
    use crate::{AddressMode, AssociationStatus, CapabilityInfo};
    use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};

    fn descriptor(address: u16, lqi: u8, permit_joining: bool) -> PanDescriptor {
        PanDescriptor {
            channel: 15,
            coord_address: MacAddress::Short(PanId(0xDFE9), ShortAddress(address)),
            superframe_spec: SuperframeSpec {
                association_permit: permit_joining,
                ..SuperframeSpec::default()
            },
            lqi,
            security_use: false,
            zigbee_beacon: ZigbeeBeaconPayload {
                protocol_id: 0,
                stack_profile: 2,
                protocol_version: 2,
                router_capacity: true,
                device_depth: 1,
                end_device_capacity: true,
                extended_pan_id: [1; 8],
                tx_offset: [0; 3],
                update_id: 1,
            },
        }
    }

    /// Build the exact three-byte form the TLSR8258 HAL surfaces for an
    /// ACK: two Frame Control bytes plus the DSN, with the hardware-checked
    /// FCS already stripped.
    fn ack_frame(sequence: u8, frame_pending: bool) -> [u8; 3] {
        let frame_control: u16 = 0x0002 | if frame_pending { 1 << 4 } else { 0 };
        let [low, high] = frame_control.to_le_bytes();
        [low, high, sequence]
    }

    #[test]
    fn ack_window_matches_only_the_transmitted_sequence_number() {
        assert_eq!(
            classify_ack_window_frame(&ack_frame(0x87, false), 0x87),
            AckWindowFrame::Match {
                frame_pending: false
            }
        );
        assert_eq!(
            classify_ack_window_frame(&ack_frame(0x87, true), 0x87),
            AckWindowFrame::Match {
                frame_pending: true
            }
        );
        assert_eq!(
            classify_ack_window_frame(&ack_frame(0x86, false), 0x87),
            AckWindowFrame::ForeignAck { sequence: 0x86 }
        );
    }

    #[test]
    fn ack_window_rejects_non_ack_and_runt_frames() {
        // Data frame carrying the awaited sequence number must not be
        // mistaken for its acknowledgement.
        let data = [0x61u8, 0x88, 0x87, 0x34, 0x12];
        assert_eq!(
            classify_ack_window_frame(&data, 0x87),
            AckWindowFrame::NotAnAck
        );
        // MAC command frame (type 3) with the same sequence number.
        let command = [0x63u8, 0x88, 0x87, 0x34, 0x12, 0x04];
        assert_eq!(
            classify_ack_window_frame(&command, 0x87),
            AckWindowFrame::NotAnAck
        );
        // Truncated frames can never satisfy the match.
        assert_eq!(
            classify_ack_window_frame(&[0x02, 0x00], 0x87),
            AckWindowFrame::NotAnAck
        );
        assert_eq!(
            classify_ack_window_frame(&[], 0x87),
            AckWindowFrame::NotAnAck
        );
    }

    #[test]
    fn ack_window_match_ignores_reserved_frame_control_bits() {
        // Only the frame-type and Frame Pending fields may influence the
        // decision; a parent that sets other reserved/version bits in its
        // ACK must still be accepted.
        let mut frame = ack_frame(0x10, false);
        frame[1] |= 0x10; // frame version field
        assert_eq!(
            classify_ack_window_frame(&frame, 0x10),
            AckWindowFrame::Match {
                frame_pending: false
            }
        );
    }

    #[test]
    fn ack_diagnostics_separate_missing_acks_from_wrong_sequence_numbers() {
        // Window 1: nothing but unrelated traffic, then expiry. This is
        // the capture's signature — a retransmission with no ACK ever
        // surfaced to the MAC.
        let mut diagnostics = AckDiagnostics::new();
        diagnostics.record_tx_attempt();
        diagnostics.record_window_frame(classify_ack_window_frame(
            &[0x61, 0x88, 0x01, 0x34, 0x12],
            0x87,
        ));
        diagnostics.record_window_result(false, 0x87);

        assert_eq!(diagnostics.tx_attempts, 1);
        assert_eq!(diagnostics.ack_windows, 1);
        assert_eq!(diagnostics.ack_windows_expired, 1);
        assert_eq!(diagnostics.ack_frames_seen, 0);
        assert_eq!(diagnostics.foreign_acks, 0);
        assert_eq!(diagnostics.window_frames_seen, 1);
        assert_eq!(diagnostics.last_expired_sequence, Some(0x87));
        assert_eq!(diagnostics.last_foreign_ack_sequence, None);

        // Window 2: an ACK arrives but for the previous DSN. That is a
        // sequence-number defect and must be counted separately.
        diagnostics.record_tx_attempt();
        diagnostics.record_window_frame(classify_ack_window_frame(&ack_frame(0x86, false), 0x87));
        diagnostics.record_window_result(false, 0x87);

        assert_eq!(diagnostics.ack_frames_seen, 1);
        assert_eq!(diagnostics.foreign_acks, 1);
        assert_eq!(diagnostics.last_foreign_ack_sequence, Some(0x86));
        assert_eq!(diagnostics.ack_matched, 0);

        // Window 3: the retransmission is acknowledged correctly.
        diagnostics.record_tx_attempt();
        diagnostics.record_window_frame(classify_ack_window_frame(&ack_frame(0x87, true), 0x87));
        diagnostics.record_window_result(true, 0x87);

        assert_eq!(diagnostics.tx_attempts, 3);
        assert_eq!(diagnostics.ack_windows, 3);
        assert_eq!(diagnostics.ack_matched, 1);
        assert_eq!(diagnostics.ack_windows_expired, 2);
        assert_eq!(diagnostics.ack_frames_seen, 2);
        assert_eq!(diagnostics.window_frames_seen, 3);
    }

    #[test]
    fn ack_diagnostics_saturate_instead_of_wrapping() {
        let mut diagnostics = AckDiagnostics {
            tx_attempts: u32::MAX,
            ack_windows: u32::MAX,
            ack_matched: u32::MAX,
            ack_windows_expired: u32::MAX,
            ack_frames_seen: u32::MAX,
            foreign_acks: u32::MAX,
            window_frames_seen: u32::MAX,
            ..AckDiagnostics::new()
        };
        diagnostics.record_tx_attempt();
        diagnostics.record_window_frame(AckWindowFrame::ForeignAck { sequence: 1 });
        diagnostics.record_window_result(true, 1);
        diagnostics.record_window_result(false, 2);

        assert_eq!(diagnostics.tx_attempts, u32::MAX);
        assert_eq!(diagnostics.ack_windows, u32::MAX);
        assert_eq!(diagnostics.ack_matched, u32::MAX);
        assert_eq!(diagnostics.ack_windows_expired, u32::MAX);
        assert_eq!(diagnostics.foreign_acks, u32::MAX);
        assert_eq!(diagnostics.window_frames_seen, u32::MAX);
    }

    #[test]
    fn repeated_beacons_do_not_consume_descriptor_slots() {
        let mut descriptors = PanDescriptorList::new();

        upsert_pan_descriptor(&mut descriptors, descriptor(0x1234, 80, false));
        upsert_pan_descriptor(&mut descriptors, descriptor(0x1234, 60, true));

        assert_eq!(descriptors.len(), 1);
        assert_eq!(descriptors[0].lqi, 80);
        assert!(descriptors[0].superframe_spec.association_permit);
    }

    #[test]
    fn full_scan_retains_strongest_unique_parents() {
        let mut descriptors = PanDescriptorList::new();
        for address in 0..MAX_PAN_DESCRIPTORS as u16 {
            upsert_pan_descriptor(&mut descriptors, descriptor(address, address as u8, true));
        }

        upsert_pan_descriptor(&mut descriptors, descriptor(0xCAFE, 200, true));

        assert_eq!(descriptors.len(), MAX_PAN_DESCRIPTORS);
        assert!(descriptors.iter().any(|entry| {
            entry.coord_address == MacAddress::Short(PanId(0xDFE9), ShortAddress(0xCAFE))
        }));
        assert!(!descriptors.iter().any(|entry| {
            entry.coord_address == MacAddress::Short(PanId(0xDFE9), ShortAddress(0))
        }));
    }

    // ── validate_router_start (host-testable MLME-START parameter rules) ──

    fn router_start_request() -> MlmeStartRequest {
        MlmeStartRequest {
            pan_id: PanId(0x1AAA),
            channel: 15,
            beacon_order: 15,
            superframe_order: 15,
            pan_coordinator: false,
            battery_life_ext: false,
        }
    }

    #[test]
    fn router_start_accepts_the_exact_non_beacon_shape_nlme_start_router_sends() {
        assert!(validate_router_start(&router_start_request()).is_ok());
    }

    #[test]
    fn router_start_rejects_pan_coordinator_requests() {
        let mut req = router_start_request();
        req.pan_coordinator = true;
        assert_eq!(validate_router_start(&req), Err(MacError::Unsupported));
    }

    #[test]
    fn router_start_rejects_beaconed_superframes() {
        let mut req = router_start_request();
        req.beacon_order = 8;
        req.superframe_order = 8;
        assert_eq!(validate_router_start(&req), Err(MacError::Unsupported));
    }

    #[test]
    fn router_start_rejects_partial_non_beacon_orders() {
        // BO=15 with SO!=15 (or vice versa) is not a valid non-beacon
        // configuration either — both fields must be 15 together.
        let mut req = router_start_request();
        req.superframe_order = 0;
        assert_eq!(validate_router_start(&req), Err(MacError::Unsupported));
    }

    #[test]
    fn router_start_rejects_out_of_range_channels() {
        for channel in [0u8, 10, 27, 255] {
            let mut req = router_start_request();
            req.channel = channel;
            assert_eq!(
                validate_router_start(&req),
                Err(MacError::InvalidParameter),
                "channel {channel} should be rejected"
            );
        }
    }

    #[test]
    fn router_start_accepts_full_channel_range() {
        for channel in 11u8..=26 {
            let mut req = router_start_request();
            req.channel = channel;
            assert!(
                validate_router_start(&req).is_ok(),
                "channel {channel} should be accepted"
            );
        }
    }

    #[test]
    fn router_start_rejects_broadcast_pan_id() {
        let mut req = router_start_request();
        req.pan_id = PanId(0xFFFF);
        assert_eq!(validate_router_start(&req), Err(MacError::InvalidParameter));
    }

    fn expect_command(frame: &[u8], lqi: u8) -> MacCommandEvent {
        match parse_incoming_frame(frame, lqi) {
            Some(ParsedIncomingFrame::Command(event)) => event,
            _ => panic!("expected MAC command event"),
        }
    }

    #[test]
    fn parses_parent_facing_mac_commands_with_exact_child_identity() {
        let beacon = build_beacon_request(1);
        assert_eq!(
            expect_command(&beacon, 90),
            MacCommandEvent::BeaconRequest(MlmeBeaconRequestIndication {
                destination_address: MacAddress::Short(PanId::BROADCAST, ShortAddress::BROADCAST,),
                lqi: 90,
                security_use: false,
            })
        );

        let child_ieee: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];
        let coordinator = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let capability_info = CapabilityInfo {
            device_type_ffd: true,
            mains_powered: false,
            rx_on_when_idle: true,
            security_capable: true,
            allocate_address: true,
        };
        let association = build_association_request(2, &coordinator, &child_ieee, &capability_info);
        assert_eq!(
            expect_command(&association, 91),
            MacCommandEvent::AssociationRequest(MlmeAssociateIndication {
                device_address: child_ieee,
                coordinator_address: coordinator,
                capability_info,
                lqi: 91,
                security_use: false,
            })
        );

        let short_poll = build_data_request_short(3, &coordinator, ShortAddress(0x3344));
        assert_eq!(
            expect_command(&short_poll, 92),
            MacCommandEvent::DataRequest(MlmeDataRequestIndication {
                source_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x3344)),
                destination_address: coordinator,
                lqi: 92,
                security_use: false,
            })
        );

        let extended_poll = build_data_request(4, &coordinator, &child_ieee);
        assert_eq!(
            expect_command(&extended_poll, 93),
            MacCommandEvent::DataRequest(MlmeDataRequestIndication {
                source_address: MacAddress::Extended(PanId(0x1234), child_ieee),
                destination_address: coordinator,
                lqi: 93,
                security_use: false,
            })
        );
    }

    #[test]
    fn parses_clean_capture_extended_source_association_request() {
        // Frame 3084 from telink-parent-bl702-child-clean-20260729.pcap,
        // excluding the hardware-validated FCS.
        let frame = [
            0x23, 0xC8, 0x9F, 0xE9, 0xDF, 0xDF, 0xF8, 0xFF, 0xFF, 0x7C, 0xB9, 0x4C, 0x61, 0x92,
            0x3A, 0x00, 0x00, 0x01, 0x80,
        ];
        assert_eq!(
            expect_command(&frame, 235),
            MacCommandEvent::AssociationRequest(MlmeAssociateIndication {
                device_address: [0x7C, 0xB9, 0x4C, 0x61, 0x92, 0x3A, 0x00, 0x00],
                coordinator_address: MacAddress::Short(PanId(0xDFE9), ShortAddress(0xF8DF),),
                capability_info: CapabilityInfo {
                    device_type_ffd: false,
                    mains_powered: false,
                    rx_on_when_idle: false,
                    security_capable: false,
                    allocate_address: true,
                },
                lqi: 235,
                security_use: false,
            })
        );
    }

    #[test]
    fn rejects_malformed_or_unidentifiable_command_frames() {
        let coordinator = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let child_ieee = [1, 2, 3, 4, 5, 6, 7, 8];

        let association =
            build_association_request(2, &coordinator, &child_ieee, &CapabilityInfo::default());
        assert!(parse_incoming_frame(&association[..association.len() - 1], 1).is_none());

        let mut unknown = build_beacon_request(1);
        unknown[7] = 0x7F;
        assert!(parse_incoming_frame(&unknown, 1).is_none());

        let mut trailing = build_data_request_short(3, &coordinator, ShortAddress(0x3344));
        trailing.push(0).unwrap();
        assert!(parse_incoming_frame(&trailing, 1).is_none());

        let mut reserved_source =
            build_data_request_short(4, &coordinator, ShortAddress::BROADCAST);
        assert!(parse_incoming_frame(&reserved_source, 1).is_none());

        let mut broadcast_destination =
            build_data_request_short(5, &coordinator, ShortAddress(0x3344));
        broadcast_destination[5..7].copy_from_slice(&u16::MAX.to_le_bytes());
        assert!(parse_incoming_frame(&broadcast_destination, 1).is_none());

        // Raw TLSR8258 frames are not MAC-decrypted. Never interpret bytes at
        // the unencrypted payload offset as a protected command identifier.
        reserved_source[0] |= 1 << 3;
        assert!(parse_incoming_frame(&reserved_source, 1).is_none());
    }

    #[test]
    fn command_queue_is_fifo_and_drops_newest_when_full() {
        let mut queue = CommandEventQueue::new();
        for lqi in 0..COMMAND_EVENT_QUEUE_CAPACITY as u8 {
            assert!(queue.push(expect_command(&build_beacon_request(lqi), lqi)));
        }
        assert!(!queue.push(expect_command(&build_beacon_request(99), 99)));

        for lqi in 0..COMMAND_EVENT_QUEUE_CAPACITY as u8 {
            match queue.pop() {
                Some(MacCommandEvent::BeaconRequest(indication)) => {
                    assert_eq!(indication.lqi, lqi);
                }
                _ => panic!("event ordering changed"),
            }
        }
        assert!(queue.pop().is_none());
    }

    #[test]
    fn command_queue_counts_every_overflow_instead_of_swallowing_it() {
        let mut queue = CommandEventQueue::new();
        assert_eq!(queue.overflow, 0);
        for lqi in 0..COMMAND_EVENT_QUEUE_CAPACITY as u8 {
            assert!(queue.push(expect_command(&build_beacon_request(lqi), lqi)));
        }
        assert_eq!(queue.high_water, COMMAND_EVENT_QUEUE_CAPACITY as u8);
        assert_eq!(queue.overflow, 0, "a queue with room reports no loss");
        for extra in 0..3u8 {
            assert!(!queue.push(expect_command(
                &build_beacon_request(90 + extra),
                90 + extra
            )));
        }
        assert_eq!(queue.overflow, 3);
        // Draining and clearing must not erase the cumulative history.
        queue.clear();
        assert_eq!(queue.overflow, 3);
        assert_eq!(queue.high_water, COMMAND_EVENT_QUEUE_CAPACITY as u8);
    }

    #[test]
    fn indication_priority_classifies_short_extended_and_broadcast_destinations() {
        let pan_id = PanId(0x1A62);
        let short_address = ShortAddress(0x9F3C);
        let extended_address = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let classify = |destination: MacAddress| {
            indication_priority(pan_id, short_address, &extended_address, &destination)
        };

        assert_eq!(
            classify(MacAddress::Short(pan_id, short_address)),
            IndicationPriority::Local
        );
        assert_eq!(
            classify(MacAddress::Extended(pan_id, extended_address)),
            IndicationPriority::Local
        );
        assert_eq!(
            classify(MacAddress::Short(pan_id, ShortAddress(0xFFFF))),
            IndicationPriority::Broadcast
        );
        // Zigbee's other MAC-visible broadcast addresses.
        assert_eq!(
            classify(MacAddress::Short(pan_id, ShortAddress(0xFFFD))),
            IndicationPriority::Broadcast
        );
        assert_eq!(
            classify(MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF))),
            IndicationPriority::Broadcast
        );
        // Another node in our PAN, and a broadcast in someone else's PAN.
        assert_eq!(
            classify(MacAddress::Short(pan_id, ShortAddress(0x1234))),
            IndicationPriority::Foreign
        );
        assert_eq!(
            classify(MacAddress::Short(PanId(0x7788), ShortAddress(0xFFFF))),
            IndicationPriority::Foreign
        );
        assert_eq!(
            classify(MacAddress::Extended(pan_id, [0xFE; 8])),
            IndicationPriority::Foreign
        );
        assert_eq!(
            classify(MacAddress::Extended(PanId(0x7788), extended_address)),
            IndicationPriority::Foreign
        );
    }

    #[test]
    fn indication_priority_never_matches_an_unassigned_short_address() {
        let pan_id = PanId(0x1A62);
        let extended_address = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        // Before an Association Response assigns a short address, the PIB
        // still holds 0xFFFF. A broadcast must not be mistaken for a
        // unicast to this node.
        assert_eq!(
            indication_priority(
                pan_id,
                ShortAddress(0xFFFF),
                &extended_address,
                &MacAddress::Short(pan_id, ShortAddress(0xFFFF)),
            ),
            IndicationPriority::Broadcast
        );
        // The extended address still identifies us, which is how the
        // post-association frame arrives.
        assert_eq!(
            indication_priority(
                pan_id,
                ShortAddress(0xFFFF),
                &extended_address,
                &MacAddress::Extended(pan_id, extended_address),
            ),
            IndicationPriority::Local
        );
    }

    #[test]
    fn data_indication_queue_is_fifo_and_reports_no_loss_while_it_has_room() {
        let destination = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let mut queue = DataIndicationQueue::new();
        for payload in 0..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert!(queue.push(
                data_indication(&destination, payload),
                IndicationPriority::Local,
            ));
        }
        assert_eq!(queue.len(), DATA_INDICATION_QUEUE_CAPACITY);
        for payload in 0..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert_eq!(queue.pop().unwrap().payload.as_slice(), &[payload]);
        }
        assert!(queue.pop().is_none());
        assert_eq!(
            queue.counters(),
            (0, 0, DATA_INDICATION_QUEUE_CAPACITY as u8),
            "an un-overflowed queue must not report any loss"
        );
    }

    #[test]
    fn overloaded_data_indication_queue_keeps_a_new_local_frame() {
        let pan_id = PanId(0x1234);
        let broadcast = MacAddress::Short(pan_id, ShortAddress(0xFFFF));
        let local = MacAddress::Short(pan_id, ShortAddress(0x0001));
        let mut queue = DataIndicationQueue::new();
        for payload in 0..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert!(queue.push(
                data_indication(&broadcast, payload),
                IndicationPriority::Broadcast,
            ));
        }
        // The ZDO response addressed to this node arrives while the queue is
        // still full of unrelated broadcast relay traffic.
        assert!(queue.push(data_indication(&local, 0xC1), IndicationPriority::Local));
        assert_eq!(
            queue.counters(),
            (1, 1, DATA_INDICATION_QUEUE_CAPACITY as u8),
            "the loss must be recorded as an eviction, not a dropped arrival"
        );

        // The oldest broadcast was sacrificed; the rest kept their order and
        // the addressed frame is retained at the tail.
        for payload in 1..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert_eq!(queue.pop().unwrap().payload.as_slice(), &[payload]);
        }
        assert_eq!(queue.pop().unwrap().payload.as_slice(), &[0xC1]);
        assert!(queue.pop().is_none());
    }

    #[test]
    fn overloaded_data_indication_queue_evicts_the_lowest_priority_entry_only() {
        let pan_id = PanId(0x1234);
        let broadcast = MacAddress::Short(pan_id, ShortAddress(0xFFFF));
        let foreign = MacAddress::Short(pan_id, ShortAddress(0x2222));
        let local = MacAddress::Short(pan_id, ShortAddress(0x0001));
        let mut queue = DataIndicationQueue::new();
        // Broadcast first, so the oldest entry outranks the foreign ones.
        assert!(queue.push(
            data_indication(&broadcast, 0xB0),
            IndicationPriority::Broadcast,
        ));
        for payload in 1..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert!(queue.push(
                data_indication(&foreign, payload),
                IndicationPriority::Foreign,
            ));
        }
        assert!(queue.push(data_indication(&local, 0xC1), IndicationPriority::Local));

        // The oldest *lowest-priority* entry (payload 1) is gone; the older
        // but more valuable broadcast survived.
        assert_eq!(queue.pop().unwrap().payload.as_slice(), &[0xB0]);
        for payload in 2..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert_eq!(queue.pop().unwrap().payload.as_slice(), &[payload]);
        }
        assert_eq!(queue.pop().unwrap().payload.as_slice(), &[0xC1]);
        assert!(queue.pop().is_none());
    }

    #[test]
    fn overloaded_data_indication_queue_never_evicts_an_equally_important_entry() {
        let pan_id = PanId(0x1234);
        let local = MacAddress::Short(pan_id, ShortAddress(0x0001));
        let mut queue = DataIndicationQueue::new();
        for payload in 0..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert!(queue.push(data_indication(&local, payload), IndicationPriority::Local));
        }
        // Nothing queued is less important, so the arrival is refused and
        // the refusal is reported instead of being swallowed.
        assert!(!queue.push(data_indication(&local, 0xEE), IndicationPriority::Local));
        assert!(!queue.push(
            data_indication(&MacAddress::Short(pan_id, ShortAddress(0x2222)), 0xFA),
            IndicationPriority::Foreign,
        ));
        assert_eq!(
            queue.counters(),
            (2, 0, DATA_INDICATION_QUEUE_CAPACITY as u8),
            "both arrivals count as loss, neither as an eviction"
        );
        for payload in 0..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert_eq!(queue.pop().unwrap().payload.as_slice(), &[payload]);
        }
    }

    #[test]
    fn mac_and_hal_queue_losses_are_reported_through_distinct_counters() {
        // The HAL interrupt queue and this MAC queue have independent
        // capacities and independent overload policies; a loss at one must
        // never be attributable to the other. Overflow the MAC queue only
        // and check that its counters, and nothing else, move.
        let pan_id = PanId(0x1234);
        let local = MacAddress::Short(pan_id, ShortAddress(0x0001));
        let mut data = DataIndicationQueue::new();
        let mut events = CommandEventQueue::new();
        for payload in 0..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert!(data.push(data_indication(&local, payload), IndicationPriority::Local));
        }
        assert!(!data.push(data_indication(&local, 0xEE), IndicationPriority::Local));

        assert_eq!(data.counters().0, 1);
        assert_eq!(
            events.overflow, 0,
            "a data indication loss must not be charged to the command queue"
        );

        // ...and the reverse.
        for lqi in 0..COMMAND_EVENT_QUEUE_CAPACITY as u8 {
            assert!(events.push(expect_command(&build_beacon_request(lqi), lqi)));
        }
        assert!(!events.push(expect_command(&build_beacon_request(99), 99)));
        assert_eq!(events.overflow, 1);
        assert_eq!(data.counters().0, 1, "unchanged by a command queue loss");
    }

    fn association_response(child: IeeeAddress) -> MlmeAssociateResponse {
        MlmeAssociateResponse {
            device_address: child,
            short_address: ShortAddress(0x3344),
            status: AssociationStatus::Success,
        }
    }

    #[test]
    fn association_response_matches_extended_retry_and_short_delivery_proof() {
        let coordinator = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let child = [1, 2, 3, 4, 5, 6, 7, 8];
        let wrong_child = [8, 7, 6, 5, 4, 3, 2, 1];
        let now = 1_000;
        let persistence = 2;
        let deadline = now + u32::from(persistence) * BASE_SUPERFRAME_DURATION_US;
        let mut responses = AssociationResponseQueue::new();
        responses
            .enqueue(association_response(child), now, persistence)
            .unwrap();
        assert_eq!(
            responses.micros_until_next_expiry(now),
            Some(u32::from(persistence) * BASE_SUPERFRAME_DURATION_US)
        );

        let short_poll = expect_command(
            &build_data_request_short(1, &coordinator, ShortAddress(0x3344)),
            1,
        );
        let wrong_short_poll = expect_command(
            &build_data_request_short(2, &coordinator, ShortAddress(0x3345)),
            2,
        );
        let wrong_poll = expect_command(&build_data_request(3, &coordinator, &wrong_child), 3);
        let matching_poll = expect_command(&build_data_request(4, &coordinator, &child), 4);
        let beacon_request = expect_command(&build_beacon_request(5), 5);

        assert_eq!(
            pending_association_poll(&short_poll, &responses, now),
            Some(PendingAssociationPoll::ConfirmDelivered(child))
        );
        assert_eq!(
            pending_association_poll(&wrong_short_poll, &responses, now),
            None
        );
        assert_eq!(pending_association_poll(&wrong_poll, &responses, now), None);
        assert_eq!(
            pending_association_poll(&beacon_request, &responses, now),
            None
        );
        assert_eq!(
            pending_association_poll(&matching_poll, &responses, now),
            Some(PendingAssociationPoll::RetryResponse(child))
        );
        // Recognition alone does not mutate the transaction. The hardware
        // path consumes a short-source proof through the successful-delivery
        // helper after the poll has received its MAC ACK.
        assert!(responses.active_for(&child, now).is_some());
        assert!(responses.active_for(&child, deadline - 1).is_some());
        assert!(responses.active_for(&child, deadline).is_none());
        assert_eq!(
            responses
                .pop_expired(deadline)
                .map(|response| response.device_address),
            Some(child)
        );
        assert!(responses.active_for(&child, deadline).is_none());
    }

    #[test]
    fn short_source_poll_completes_retained_association_response() {
        let coordinator = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let child = [1, 2, 3, 4, 5, 6, 7, 8];
        let now = 100;
        let mut responses = AssociationResponseQueue::new();
        responses
            .enqueue(association_response(child), now, 10)
            .unwrap();
        let poll = expect_command(
            &build_data_request_short(1, &coordinator, ShortAddress(0x3344)),
            1,
        );

        assert_eq!(
            pending_association_poll(&poll, &responses, now),
            Some(PendingAssociationPoll::ConfirmDelivered(child))
        );
        let delivery = take_successful_association_delivery(&mut responses, &child).unwrap();
        assert_eq!(delivery.device_address, child);
        assert_eq!(delivery.short_address, ShortAddress(0x3344));
        assert_eq!(delivery.status, AssociationStatus::Success);
        assert_eq!(delivery.result, Ok(()));
        assert!(responses.active_for(&child, now).is_none());
        assert_eq!(pending_association_poll(&poll, &responses, now), None);
    }

    #[test]
    fn association_transaction_deadline_is_wrap_safe() {
        let child = [1, 2, 3, 4, 5, 6, 7, 8];
        let now = u32::MAX - 1_000;
        let mut responses = AssociationResponseQueue::new();
        responses
            .enqueue(association_response(child), now, 1)
            .unwrap();

        assert_eq!(
            responses.micros_until_next_expiry(now),
            Some(BASE_SUPERFRAME_DURATION_US)
        );
        let deadline = now.wrapping_add(BASE_SUPERFRAME_DURATION_US);
        assert!(responses.active_for(&child, deadline - 1).is_some());
        assert!(responses.active_for(&child, deadline).is_none());
    }

    #[test]
    fn serviced_association_poll_is_not_retried_when_event_is_dequeued() {
        let coordinator = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let child = [1, 2, 3, 4, 5, 6, 7, 8];
        let now = 100;
        let mut responses = AssociationResponseQueue::new();
        responses
            .enqueue(association_response(child), now, 10)
            .unwrap();
        let mut events = CommandEventQueue::new();

        // The receive-data path already attempted delivery for this physical
        // poll. Model a failed transmission by retaining the response.
        let serviced_poll = expect_command(&build_data_request(1, &coordinator, &child), 1);
        assert!(events.push_with_association_poll_state(serviced_poll, true));
        let queued = events.pop_queued().unwrap();
        assert_eq!(
            queued_association_poll_to_service(&queued, &responses, now),
            None
        );
        assert!(responses.active_for(&child, now).is_some());

        // A newly received physical poll remains eligible for the retry.
        let fresh_poll = expect_command(&build_data_request(2, &coordinator, &child), 2);
        assert!(events.push(fresh_poll));
        let queued = events.pop_queued().unwrap();
        assert_eq!(
            queued_association_poll_to_service(&queued, &responses, now),
            Some(PendingAssociationPoll::RetryResponse(child))
        );
    }

    #[test]
    fn ordinary_data_request_queues_and_ends_parent_receive_slice() {
        let coordinator = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let poll = expect_command(
            &build_data_request_short(1, &coordinator, ShortAddress(0x3344)),
            55,
        );
        let mut events = CommandEventQueue::new();

        assert!(queue_command_for_parent_receive(
            &mut events,
            poll.clone(),
            false,
        ));
        assert_eq!(events.pop(), Some(poll));

        let beacon_request = expect_command(&build_beacon_request(2), 56);
        assert!(!queue_command_for_parent_receive(
            &mut events,
            beacon_request.clone(),
            false,
        ));
        assert_eq!(events.pop(), Some(beacon_request));
    }

    fn data_indication(destination: &MacAddress, payload: u8) -> McpsDataIndication {
        let frame = build_data_frame(
            payload,
            AddressMode::Short,
            ShortAddress(0x3344),
            &[8, 7, 6, 5, 4, 3, 2, 1],
            destination,
            &[payload],
            false,
            false,
        )
        .unwrap();
        let Some(ParsedIncomingFrame::Data(indication)) = parse_incoming_frame(&frame, payload)
        else {
            panic!("expected data indication");
        };
        indication
    }

    #[test]
    fn data_retained_during_association_response_ack_wait_is_rechecked() {
        let pan_id = PanId(0x1234);
        let short_address = ShortAddress(0x0001);
        let extended_address = [9, 8, 7, 6, 5, 4, 3, 2];
        let coordinator = MacAddress::Short(pan_id, short_address);
        let child = [1, 2, 3, 4, 5, 6, 7, 8];
        let poll = expect_command(&build_data_request(1, &coordinator, &child), 1);
        let mut associations = AssociationResponseQueue::new();
        associations
            .enqueue(association_response(child), 100, 10)
            .unwrap();
        assert_eq!(
            pending_association_poll(&poll, &associations, 100),
            Some(PendingAssociationPoll::RetryResponse(child))
        );

        // This models transmit_with_ack() retaining a data frame while
        // deliver_association_response() waits for its ACK. The next receive
        // loop iteration must inspect the queue before checking its deadline.
        let mut normal_data = DataIndicationQueue::new();
        assert!(normal_data.push(
            data_indication(&coordinator, 0xAA),
            IndicationPriority::Local
        ));
        assert_eq!(
            take_pending_data(&mut normal_data)
                .unwrap()
                .payload
                .as_slice(),
            &[0xAA]
        );

        // Poll reception must select only an exact local destination and
        // leave an unrelated retained frame queued.
        let mut poll_data = DataIndicationQueue::new();
        let unrelated = MacAddress::Short(pan_id, ShortAddress(0x2222));
        let local_extended = MacAddress::Extended(pan_id, extended_address);
        assert!(poll_data.push(
            data_indication(&unrelated, 0xBB),
            IndicationPriority::Foreign
        ));
        assert!(poll_data.push(
            data_indication(&local_extended, 0xCC),
            IndicationPriority::Local
        ));
        assert_eq!(
            take_pending_poll_data(&mut poll_data, pan_id, short_address, &extended_address,)
                .unwrap()
                .payload
                .as_slice(),
            &[0xCC]
        );
        assert_eq!(
            take_pending_data(&mut poll_data)
                .unwrap()
                .payload
                .as_slice(),
            &[0xBB]
        );
    }

    #[test]
    fn command_and_data_queues_cannot_starve_each_other() {
        let mut events = CommandEventQueue::new();
        for lqi in 0..COMMAND_EVENT_QUEUE_CAPACITY as u8 {
            assert!(events.push(expect_command(&build_beacon_request(lqi), lqi)));
        }

        let source_ieee = [8, 7, 6, 5, 4, 3, 2, 1];
        let destination = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let frame = build_data_frame(
            5,
            AddressMode::Short,
            ShortAddress(0x3344),
            &source_ieee,
            &destination,
            &[0xAA],
            true,
            false,
        )
        .unwrap();
        let Some(ParsedIncomingFrame::Data(indication)) = parse_incoming_frame(&frame, 201) else {
            panic!("expected data indication");
        };
        let mut data = DataIndicationQueue::new();
        assert!(data.push(indication, IndicationPriority::Local));

        // A full command queue cannot consume or block the data queue.
        assert_eq!(data.pop().unwrap().payload.as_slice(), &[0xAA]);
        assert!(matches!(
            events.pop(),
            Some(MacCommandEvent::BeaconRequest(_))
        ));

        events.clear();
        for sequence in 0..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            let frame = build_data_frame(
                sequence,
                AddressMode::Short,
                ShortAddress(0x3344),
                &source_ieee,
                &destination,
                &[sequence],
                false,
                false,
            )
            .unwrap();
            let Some(ParsedIncomingFrame::Data(indication)) =
                parse_incoming_frame(&frame, sequence)
            else {
                panic!("expected data indication");
            };
            assert!(data.push(indication, IndicationPriority::Local));
        }
        assert!(events.push(expect_command(&build_beacon_request(99), 99)));

        // Likewise, a full data queue does not consume the command channel.
        assert!(matches!(
            events.pop(),
            Some(MacCommandEvent::BeaconRequest(indication)) if indication.lqi == 99
        ));
        for sequence in 0..DATA_INDICATION_QUEUE_CAPACITY as u8 {
            assert_eq!(data.pop().unwrap().payload.as_slice(), &[sequence]);
        }
    }

    #[test]
    fn reset_transient_helper_clears_all_queues_without_pib_inputs() {
        let pan_id = PanId(0x1234);
        let short_address = ShortAddress(0x0001);
        let child = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut events = CommandEventQueue::new();
        let mut data = DataIndicationQueue::new();
        let mut associations = AssociationResponseQueue::new();

        assert!(events.push(expect_command(&build_beacon_request(1), 1)));
        let destination = MacAddress::Short(pan_id, short_address);
        let frame = build_data_frame(
            2,
            AddressMode::Short,
            ShortAddress(0x3344),
            &[8, 7, 6, 5, 4, 3, 2, 1],
            &destination,
            &[0xAA],
            false,
            false,
        )
        .unwrap();
        let Some(ParsedIncomingFrame::Data(indication)) = parse_incoming_frame(&frame, 2) else {
            panic!("expected data indication");
        };
        assert!(data.push(indication, IndicationPriority::Local));
        associations
            .enqueue(association_response(child), 100, 10)
            .unwrap();

        clear_transient_queues(&mut data, &mut events, &mut associations);

        assert!(data.pop().is_none());
        assert!(events.pop().is_none());
        assert!(associations.active_for(&child, 100).is_none());
        assert_eq!(pan_id, PanId(0x1234));
        assert_eq!(short_address, ShortAddress(0x0001));
    }

    #[test]
    fn normal_data_frame_delivery_is_unchanged_by_command_parsing() {
        let source_ieee = [8, 7, 6, 5, 4, 3, 2, 1];
        let destination = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let frame = build_data_frame(
            5,
            AddressMode::Short,
            ShortAddress(0x3344),
            &source_ieee,
            &destination,
            &[0xAA, 0xBB, 0xCC],
            true,
            false,
        )
        .unwrap();

        let Some(ParsedIncomingFrame::Data(indication)) = parse_incoming_frame(&frame, 201) else {
            panic!("expected MCPS data indication");
        };
        assert_eq!(
            indication.src_address,
            MacAddress::Short(PanId(0x1234), ShortAddress(0x3344))
        );
        assert_eq!(indication.dst_address, destination);
        assert_eq!(indication.lqi, 201);
        assert_eq!(indication.payload.as_slice(), &[0xAA, 0xBB, 0xCC]);
        assert!(!indication.security_use);
    }

    #[test]
    fn telink_accepts_only_local_and_ieee_mac_broadcast_destinations() {
        let pan = PanId(0x1234);
        let own_short = ShortAddress(0x0001);
        let own_extended = [1, 2, 3, 4, 5, 6, 7, 8];

        for address in [own_short.0, 0xFFFF] {
            assert!(accepts_telink_destination(
                pan,
                own_short,
                &own_extended,
                false,
                &MacAddress::Short(pan, ShortAddress(address)),
            ));
        }
        assert!(accepts_telink_destination(
            pan,
            own_short,
            &own_extended,
            false,
            &MacAddress::Extended(pan, own_extended),
        ));
        for rejected in [0x3344, 0xFFFD, 0xFFFC] {
            assert!(!accepts_telink_destination(
                pan,
                own_short,
                &own_extended,
                false,
                &MacAddress::Short(pan, ShortAddress(rejected)),
            ));
        }
    }
}
