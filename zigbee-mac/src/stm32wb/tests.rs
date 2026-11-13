//! Host tests of the coprocessor MAC mapping with a scripted transport.

extern crate std;

use core::cell::Cell;
use core::future::Future;
use core::task::{Context, Poll, Waker};
use std::collections::VecDeque;
use std::sync::Arc;
use std::task::Wake;
use std::vec::Vec as StdVec;

use zigbee_types::{ChannelMask, MacAddress, PanId, ShortAddress};

use super::sap::{BeaconNotify, DataIndication, Event, Request, WbMacTransport, pib_id, status};
use super::{MAX_INDIRECT_IN_FLIGHT, MAX_PENDING_CHILDREN, Stm32wbMac};
use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacDriver, MacError, ParentMacDriver, PlatformServices};

const IEEE: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
const PAN: PanId = PanId(0xDFE9);

/// Owned record of a request seen by the transport.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Sent {
    Reset(bool),
    Set(u8, StdVec<u8>),
    Scan(u8, u32, u8),
    Associate,
    AssociateResponse([u8; 8], u16, u8),
    OrphanResponse([u8; 8], u16),
    Disassociate,
    Start(u16, u8, bool),
    Poll,
    Data {
        handle: u8,
        indirect: bool,
        len: usize,
    },
    Purge(u8),
}

struct ScriptedTransport {
    now: Cell<u32>,
    events: VecDeque<Event>,
    sent: StdVec<Sent>,
    /// Status returned by the synchronous command channel per request.
    reject_next: Option<u8>,
    start_status: u8,
    data_status: u8,
    /// Extra events delivered before a scan confirm.
    scan_events: StdVec<Event>,
    scan_status: u8,
    /// Simulate a wedged mailbox: accept requests, never confirm.
    swallow_confirms: bool,
}

impl ScriptedTransport {
    fn new() -> Self {
        Self {
            now: Cell::new(0),
            events: VecDeque::new(),
            sent: StdVec::new(),
            reject_next: None,
            start_status: status::SUCCESS,
            data_status: status::SUCCESS,
            scan_events: StdVec::new(),
            scan_status: status::SUCCESS,
            swallow_confirms: false,
        }
    }
}

impl zigbee_crypto::ForwardAesProvider for ScriptedTransport {}

impl PlatformServices for ScriptedTransport {
    fn monotonic_micros(&self) -> u32 {
        self.now.get()
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        self.now.set(self.now.get().wrapping_add(duration_us));
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        output.fill(0x5A);
        Ok(())
    }
}

impl WbMacTransport for ScriptedTransport {
    async fn request(&mut self, request: &Request<'_>) -> Result<(), u8> {
        if let Some(code) = self.reject_next.take() {
            return Err(code);
        }
        let (sent, confirm) = match *request {
            Request::Reset { set_default_pib } => (
                Sent::Reset(set_default_pib),
                Some(Event::ResetConfirm {
                    status: status::SUCCESS,
                }),
            ),
            Request::Set { attribute, value } => (
                Sent::Set(attribute, value.to_vec()),
                Some(Event::SetConfirm {
                    status: status::SUCCESS,
                    attribute,
                }),
            ),
            Request::Scan {
                scan_type,
                channels,
                duration,
            } => {
                for event in self.scan_events.drain(..) {
                    self.events.push_back(event);
                }
                let energy = if scan_type == super::sap::SapScanType::Ed {
                    (0..channels.count_ones() as u8).map(|i| i * 10).collect()
                } else {
                    heapless::Vec::new()
                };
                (
                    Sent::Scan(scan_type as u8, channels, duration),
                    Some(Event::ScanConfirm {
                        status: self.scan_status,
                        scan_type: scan_type as u8,
                        unscanned_channels: 0,
                        energy,
                    }),
                )
            }
            Request::Associate { .. } => (
                Sent::Associate,
                Some(Event::AssociateConfirm {
                    status: status::SUCCESS,
                    short_address: ShortAddress(0x1234),
                }),
            ),
            Request::AssociateResponse {
                device,
                short_address,
                status,
            } => (
                Sent::AssociateResponse(device, short_address.0, status),
                None,
            ),
            Request::OrphanResponse {
                orphan,
                short_address,
                ..
            } => (Sent::OrphanResponse(orphan, short_address.0), None),
            Request::Disassociate { .. } => (
                Sent::Disassociate,
                Some(Event::DisassociateConfirm {
                    status: status::SUCCESS,
                }),
            ),
            Request::Start {
                pan_id,
                channel,
                pan_coordinator,
                ..
            } => (
                Sent::Start(pan_id.0, channel, pan_coordinator),
                Some(Event::StartConfirm {
                    status: self.start_status,
                }),
            ),
            Request::Poll { .. } => (
                Sent::Poll,
                Some(Event::PollConfirm {
                    status: status::NO_DATA,
                }),
            ),
            Request::Data {
                handle,
                indirect,
                msdu,
                ..
            } => (
                Sent::Data {
                    handle,
                    indirect,
                    len: msdu.len(),
                },
                (!indirect).then_some(Event::DataConfirm {
                    handle,
                    status: self.data_status,
                    timestamp: 77,
                }),
            ),
            Request::Purge { handle } => (Sent::Purge(handle), None),
        };
        self.sent.push(sent);
        if let Some(confirm) = confirm.filter(|_| !self.swallow_confirms) {
            self.events.push_back(confirm);
        }
        Ok(())
    }

    async fn event(&mut self, timeout_us: u32) -> Option<Event> {
        let event = self.events.pop_front();
        if event.is_none() {
            // Simulate waiting out the whole timeout.
            self.now.set(self.now.get().wrapping_add(timeout_us.max(1)));
        }
        event
    }
}

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

fn mac() -> Stm32wbMac<ScriptedTransport> {
    Stm32wbMac::new(ScriptedTransport::new(), IEEE)
}

fn router_start() -> MlmeStartRequest {
    MlmeStartRequest {
        pan_id: PAN,
        channel: 15,
        beacon_order: 15,
        superframe_order: 15,
        pan_coordinator: false,
        battery_life_ext: false,
    }
}

fn started_mac() -> Stm32wbMac<ScriptedTransport> {
    let mut mac = mac();
    block_on(mac.mlme_set(
        PibAttribute::MacShortAddress,
        PibValue::ShortAddress(ShortAddress(0x4321)),
    ))
    .unwrap();
    block_on(mac.mlme_start(router_start())).unwrap();
    mac.transport_mut().sent.clear();
    mac
}

fn beacon_payload(permit_byte: u8) -> PibPayload {
    let mut bytes = [0u8; 15];
    bytes[1] = 0x22;
    bytes[2] = permit_byte;
    bytes[3..11].copy_from_slice(&[0xDD; 8]);
    PibPayload::from_slice(&bytes).unwrap()
}

fn assert_parent<M: ParentMacDriver>(_: &M) {}

// ── Construction, reset and PIB ─────────────────────────────────

#[test]
fn backend_is_a_sealed_parent_with_truthful_capabilities() {
    let mac = mac();
    assert_parent(&mac);
    let caps = mac.capabilities();
    assert!(caps.router);
    assert!(!caps.coordinator);
    assert!(!caps.hardware_security);
}

#[test]
fn first_primitive_resets_coprocessor_and_programs_eui64() {
    let mut mac = mac();
    block_on(mac.mlme_set(PibAttribute::MacPanId, PibValue::PanId(PAN))).unwrap();
    assert_eq!(
        mac.transport().sent,
        [
            Sent::Reset(true),
            Sent::Set(pib_id::EXTENDED_ADDRESS, IEEE.to_vec()),
            Sent::Set(pib_id::PAN_ID, PAN.0.to_le_bytes().to_vec()),
        ]
    );
    assert_eq!(
        block_on(mac.mlme_get(PibAttribute::MacPanId)),
        Ok(PibValue::PanId(PAN))
    );
}

#[test]
fn sync_reset_is_deferred_and_clears_the_mirror() {
    let mut mac = started_mac();
    mac.mlme_reset(true).unwrap();
    assert!(mac.transport().sent.is_empty(), "reset must not block");
    assert_eq!(
        block_on(mac.mlme_get(PibAttribute::MacShortAddress)),
        Ok(PibValue::ShortAddress(ShortAddress(0xFFFF)))
    );
    assert!(!mac.started());
    block_on(mac.mlme_set(PibAttribute::MacRxOnWhenIdle, PibValue::Bool(true))).unwrap();
    assert_eq!(mac.transport().sent[0], Sent::Reset(true));
    assert_eq!(
        mac.transport().sent[1],
        Sent::Set(pib_id::EXTENDED_ADDRESS, IEEE.to_vec())
    );
    assert_eq!(
        mac.transport().sent[2],
        Sent::Set(pib_id::RX_ON_WHEN_IDLE, [1].to_vec())
    );
}

#[test]
fn invalid_pib_values_never_reach_the_coprocessor() {
    let mut mac = started_mac();
    assert_eq!(
        block_on(mac.mlme_set(PibAttribute::PhyCurrentChannel, PibValue::U8(27))),
        Err(MacError::InvalidParameter)
    );
    assert_eq!(
        block_on(mac.mlme_set(PibAttribute::PhyChannelsSupported, PibValue::U32(1))),
        Err(MacError::Unsupported)
    );
    assert!(mac.transport().sent.is_empty());
}

#[test]
fn coprocessor_rejection_keeps_the_mirror_unchanged() {
    let mut mac = started_mac();
    mac.transport_mut().reject_next = Some(status::UNSUPPORTED_ATTRIBUTE);
    assert_eq!(
        block_on(mac.mlme_set(PibAttribute::PhyTransmitPower, PibValue::I8(4))),
        Err(MacError::Unsupported)
    );
    assert_eq!(
        block_on(mac.mlme_get(PibAttribute::PhyTransmitPower)),
        Ok(PibValue::I8(0))
    );
}

// ── Start and beacons ───────────────────────────────────────────

#[test]
fn router_start_is_validated_and_sent_as_non_pan_coordinator() {
    let mut mac = mac();
    let mut coordinator = router_start();
    coordinator.pan_coordinator = true;
    assert_eq!(
        block_on(mac.mlme_start(coordinator)),
        Err(MacError::Unsupported)
    );
    assert!(mac.transport().sent.is_empty());

    block_on(mac.mlme_start(router_start())).unwrap();
    assert!(
        mac.transport()
            .sent
            .contains(&Sent::Start(PAN.0, 15, false))
    );
    assert!(mac.started());
}

#[test]
fn failed_start_reports_the_coprocessor_status() {
    let mut mac = mac();
    mac.transport_mut().start_status = status::INVALID_PARAMETER;
    assert_eq!(
        block_on(mac.mlme_start(router_start())),
        Err(MacError::InvalidParameter)
    );
    assert!(!mac.started());
}

#[test]
fn beacon_requests_are_synthesized_only_after_start() {
    let mut fresh = mac();
    assert_eq!(
        block_on(fresh.mac_command_event_timeout(0)),
        Err(MacError::NoData)
    );

    let mut mac = started_mac();
    let event = block_on(mac.mac_command_event_timeout(0)).unwrap();
    assert!(matches!(event, MacCommandEvent::BeaconRequest(_)));
    // Not due again until the refresh period has elapsed.
    assert_eq!(
        block_on(mac.mac_command_event_timeout(0)),
        Err(MacError::NoData)
    );
    let now = mac.transport().now.get();
    mac.transport_mut().now.set(now + super::BEACON_REFRESH_US);
    let event = block_on(mac.mac_command_event_timeout(0)).unwrap();
    assert!(matches!(event, MacCommandEvent::BeaconRequest(_)));
}

#[test]
fn beacon_content_is_programmed_only_when_it_changes() {
    let mut mac = started_mac();
    let mut response = MlmeBeaconResponse::new(beacon_payload(0x84));
    response.association_permit = true;
    block_on(mac.mlme_beacon_response(response.clone())).unwrap();
    let payload = beacon_payload(0x84).as_slice().to_vec();
    assert_eq!(
        mac.transport().sent,
        [
            Sent::Set(pib_id::BEACON_PAYLOAD, payload),
            Sent::Set(pib_id::BEACON_PAYLOAD_LENGTH, [15].to_vec()),
            Sent::Set(pib_id::ASSOCIATION_PERMIT, [1].to_vec()),
        ]
    );
    mac.transport_mut().sent.clear();
    block_on(mac.mlme_beacon_response(response.clone())).unwrap();
    assert!(mac.transport().sent.is_empty(), "unchanged beacon");

    response.association_permit = false;
    block_on(mac.mlme_beacon_response(response)).unwrap();
    assert_eq!(mac.transport().sent.len(), 3);
    assert_eq!(mac.stats().beacon_refreshes, 2);
}

#[test]
fn beacon_response_before_start_or_as_coordinator_fails() {
    let mut mac = mac();
    assert_eq!(
        block_on(mac.mlme_beacon_response(MlmeBeaconResponse::new(beacon_payload(0)))),
        Err(MacError::InvalidParameter)
    );
    let mut mac = started_mac();
    let mut response = MlmeBeaconResponse::new(beacon_payload(0));
    response.pan_coordinator = true;
    assert_eq!(
        block_on(mac.mlme_beacon_response(response)),
        Err(MacError::Unsupported)
    );
}

// ── Association ─────────────────────────────────────────────────

#[test]
fn association_indication_response_and_delivery() {
    let mut mac = started_mac();
    let joiner = [9u8; 8];
    mac.transport_mut()
        .events
        .push_back(Event::AssociateIndication {
            device: joiner,
            capability: 0x80,
            security_use: false,
        });
    let event = block_on(mac.mac_command_event_timeout(0)).unwrap();
    let MacCommandEvent::AssociationRequest(indication) = event else {
        panic!("expected association request, got {event:?}");
    };
    assert_eq!(indication.device_address, joiner);
    assert!(indication.capability_info.allocate_address);
    assert_eq!(
        indication.coordinator_address,
        MacAddress::Short(PAN, ShortAddress(0x4321))
    );

    block_on(mac.mlme_associate_response(MlmeAssociateResponse {
        device_address: joiner,
        short_address: ShortAddress(0x0BAD),
        status: AssociationStatus::Success,
    }))
    .unwrap();
    assert_eq!(
        mac.transport().sent.last(),
        Some(&Sent::AssociateResponse(joiner, 0x0BAD, 0))
    );

    mac.transport_mut().events.push_back(Event::CommStatus {
        pan_id: PAN,
        source: Some(MacAddress::Extended(PAN, IEEE)),
        destination: Some(MacAddress::Extended(PAN, joiner)),
        status: status::SUCCESS,
    });
    // Skip the synthetic beacon refresh that is also due.
    let mut delivery = None;
    for _ in 0..3 {
        if let Ok(MacCommandEvent::AssociationResponseDelivery(d)) =
            block_on(mac.mac_command_event_timeout(0))
        {
            delivery = Some(d);
            break;
        }
    }
    let delivery = delivery.expect("association delivery");
    assert_eq!(delivery.device_address, joiner);
    assert_eq!(delivery.short_address, ShortAddress(0x0BAD));
    assert_eq!(delivery.status, AssociationStatus::Success);
    assert_eq!(delivery.result, Ok(()));
}

#[test]
fn undelivered_association_response_reports_the_failure() {
    let mut mac = started_mac();
    let joiner = [7u8; 8];
    block_on(mac.mlme_associate_response(MlmeAssociateResponse {
        device_address: joiner,
        short_address: ShortAddress(0x0101),
        status: AssociationStatus::Success,
    }))
    .unwrap();
    mac.transport_mut().events.push_back(Event::CommStatus {
        pan_id: PAN,
        source: None,
        destination: Some(MacAddress::Extended(PAN, joiner)),
        status: status::TRANSACTION_EXPIRED,
    });
    let delivery = (0..3)
        .find_map(|_| match block_on(mac.mac_command_event_timeout(0)) {
            Ok(MacCommandEvent::AssociationResponseDelivery(d)) => Some(d),
            _ => None,
        })
        .unwrap();
    assert_eq!(delivery.result, Err(MacError::TransactionExpired));
}

#[test]
fn association_response_table_is_bounded() {
    let mut mac = started_mac();
    for i in 0..super::MAX_PENDING_ASSOCIATIONS as u8 {
        block_on(mac.mlme_associate_response(MlmeAssociateResponse {
            device_address: [i; 8],
            short_address: ShortAddress(u16::from(i) + 1),
            status: AssociationStatus::Success,
        }))
        .unwrap();
    }
    assert_eq!(
        block_on(mac.mlme_associate_response(MlmeAssociateResponse {
            device_address: [0xEE; 8],
            short_address: ShortAddress(0x0EEE),
            status: AssociationStatus::Success,
        })),
        Err(MacError::TransactionOverflow)
    );
    // A repeated request from a known joiner replaces its entry.
    block_on(mac.mlme_associate_response(MlmeAssociateResponse {
        device_address: [0; 8],
        short_address: ShortAddress(0x0002),
        status: AssociationStatus::Success,
    }))
    .unwrap();
}

// ── Indirect data ───────────────────────────────────────────────

fn take_data_request(mac: &mut Stm32wbMac<ScriptedTransport>) -> Option<MacAddress> {
    for _ in 0..3 {
        match block_on(mac.mac_command_event_timeout(0)) {
            Ok(MacCommandEvent::DataRequest(indication)) => {
                return Some(indication.source_address);
            }
            Ok(_) => continue,
            Err(_) => return None,
        }
    }
    None
}

fn indirect_request(child: u16, payload: &[u8]) -> McpsDataRequest<'_> {
    McpsDataRequest {
        src_addr_mode: AddressMode::Short,
        dst_address: MacAddress::Short(PAN, ShortAddress(child)),
        payload,
        msdu_handle: 0x33,
        tx_options: TxOptions {
            ack_tx: true,
            indirect: true,
            ..Default::default()
        },
    }
}

#[test]
fn pending_child_produces_a_virtual_data_request_and_hand_off() {
    let mut mac = started_mac();
    let child = MacAddress::Short(PAN, ShortAddress(0x2222));
    mac.set_indirect_data_pending(child, true).unwrap();
    assert_eq!(take_data_request(&mut mac), Some(child));
    // Consumed: the NWK layer re-arms it if frames remain.
    assert_eq!(take_data_request(&mut mac), None);

    let confirm = block_on(mac.mcps_indirect_data(indirect_request(0x2222, &[1, 2, 3]))).unwrap();
    assert_eq!(confirm.msdu_handle, 0x33);
    let Some(Sent::Data {
        handle, indirect, ..
    }) = mac.transport().sent.last().cloned()
    else {
        panic!("indirect data not sent");
    };
    assert!(indirect);
    assert_eq!(mac.stats().indirect_handed_off, 1);

    // The later delivery confirm is absorbed and counted.
    mac.transport_mut().events.push_back(Event::DataConfirm {
        handle,
        status: status::SUCCESS,
        timestamp: 0,
    });
    let _ = block_on(mac.mac_command_event_timeout(0));
    assert_eq!(mac.stats().indirect_delivered, 1);
}

#[test]
fn clearing_pending_cancels_the_virtual_request() {
    let mut mac = started_mac();
    let child = MacAddress::Short(PAN, ShortAddress(0x2222));
    mac.set_indirect_data_pending(child, true).unwrap();
    mac.set_indirect_data_pending(child, true).unwrap();
    mac.set_indirect_data_pending(child, false).unwrap();
    assert_eq!(take_data_request(&mut mac), None);
}

#[test]
fn pending_child_set_is_bounded() {
    let mut mac = started_mac();
    for i in 0..MAX_PENDING_CHILDREN as u16 {
        mac.set_indirect_data_pending(MacAddress::Short(PAN, ShortAddress(i + 1)), true)
            .unwrap();
    }
    assert_eq!(
        mac.set_indirect_data_pending(MacAddress::Short(PAN, ShortAddress(0x0FFF)), true),
        Err(MacError::TransactionOverflow)
    );
    assert_eq!(
        mac.set_indirect_data_pending(MacAddress::Extended(PAN, IEEE), true),
        Err(MacError::InvalidParameter)
    );
}

#[test]
fn in_flight_indirect_frames_are_capped_and_throttle_virtual_requests() {
    let mut mac = started_mac();
    for i in 0..MAX_INDIRECT_IN_FLIGHT as u16 {
        block_on(mac.mcps_indirect_data(indirect_request(0x100 + i, &[0]))).unwrap();
    }
    assert_eq!(
        block_on(mac.mcps_indirect_data(indirect_request(0x200, &[0])))
            .err()
            .unwrap(),
        MacError::TransactionOverflow
    );
    let child = MacAddress::Short(PAN, ShortAddress(0x0200));
    mac.set_indirect_data_pending(child, true).unwrap();
    assert_eq!(take_data_request(&mut mac), None, "queue full: no hand-off");

    // One expiry frees a slot.
    let Some(Sent::Data { handle, .. }) = mac
        .transport()
        .sent
        .iter()
        .find(|sent| matches!(sent, Sent::Data { .. }))
        .cloned()
    else {
        unreachable!()
    };
    mac.transport_mut().events.push_back(Event::DataConfirm {
        handle,
        status: status::TRANSACTION_EXPIRED,
        timestamp: 0,
    });
    assert_eq!(take_data_request(&mut mac), Some(child));
    assert_eq!(mac.stats().indirect_expired, 1);
}

#[test]
fn handles_in_flight_are_never_reused() {
    let mut mac = started_mac();
    block_on(mac.mcps_indirect_data(indirect_request(0x0101, &[0]))).unwrap();
    for _ in 0..300 {
        block_on(mac.mcps_data(McpsDataRequest {
            src_addr_mode: AddressMode::Short,
            dst_address: MacAddress::Short(PAN, ShortAddress(0x0000)),
            payload: &[1],
            msdu_handle: 1,
            tx_options: TxOptions {
                ack_tx: true,
                ..Default::default()
            },
        }))
        .unwrap();
    }
    let handles: StdVec<(u8, bool)> = mac
        .transport()
        .sent
        .iter()
        .filter_map(|sent| match sent {
            Sent::Data {
                handle, indirect, ..
            } => Some((*handle, *indirect)),
            _ => None,
        })
        .collect();
    let indirect_handle = handles.iter().find(|(_, i)| *i).unwrap().0;
    assert!(
        handles
            .iter()
            .filter(|(_, i)| !*i)
            .all(|(h, _)| *h != indirect_handle)
    );
}

#[test]
fn real_poll_indication_becomes_a_data_request() {
    let mut mac = started_mac();
    let child = MacAddress::Short(PAN, ShortAddress(0x3333));
    mac.set_indirect_data_pending(child, true).unwrap();
    mac.transport_mut()
        .events
        .push_back(Event::PollIndication { requester: child });
    assert_eq!(take_data_request(&mut mac), Some(child));
    // The real poll superseded the virtual request for that child.
    assert_eq!(take_data_request(&mut mac), None);

    let joiner = MacAddress::Extended(PAN, [4; 8]);
    mac.transport_mut()
        .events
        .push_back(Event::PollIndication { requester: joiner });
    assert_eq!(take_data_request(&mut mac), Some(joiner));
}

// ── Direct data and receive ─────────────────────────────────────

#[test]
fn direct_data_maps_the_confirm_status() {
    let mut mac = started_mac();
    let request = || McpsDataRequest {
        src_addr_mode: AddressMode::Short,
        dst_address: MacAddress::Short(PAN, ShortAddress(0x0000)),
        payload: &[1, 2],
        msdu_handle: 9,
        tx_options: TxOptions {
            ack_tx: true,
            ..Default::default()
        },
    };
    let confirm = block_on(mac.mcps_data(request())).unwrap();
    assert_eq!(confirm.msdu_handle, 9);
    assert_eq!(confirm.timestamp, Some(77));

    mac.transport_mut().data_status = status::NO_ACK;
    assert_eq!(
        block_on(mac.mcps_data(request())).err(),
        Some(MacError::NoAck)
    );
    mac.transport_mut().data_status = status::CHANNEL_ACCESS_FAILURE;
    for streak in 1..=2 {
        assert_eq!(
            block_on(mac.mcps_data(request())).err(),
            Some(MacError::ChannelAccessFailure)
        );
        assert_eq!(mac.stats().tx_channel_access_streak, streak);
    }
    let stats = mac.stats();
    assert_eq!(
        (
            stats.tx_ok,
            stats.tx_no_ack,
            stats.tx_channel_access_failures
        ),
        (1, 1, 2)
    );
    mac.transport_mut().data_status = status::SUCCESS;
    block_on(mac.mcps_data(request())).unwrap();
    assert_eq!(mac.stats().tx_channel_access_streak, 0);
    mac.transport_mut().data_status = status::CHANNEL_ACCESS_FAILURE;

    let mut indirect = request();
    indirect.tx_options.indirect = true;
    assert_eq!(
        block_on(mac.mcps_data(indirect)).err(),
        Some(MacError::InvalidParameter)
    );
    let mut secured = request();
    secured.tx_options.security_enabled = true;
    assert_eq!(
        block_on(mac.mcps_data(secured)).err(),
        Some(MacError::Unsupported)
    );
}

#[test]
fn missing_confirm_times_out_instead_of_hanging() {
    let mut mac = started_mac();
    mac.transport_mut().swallow_confirms = true;
    let before = mac.transport().now.get();
    assert_eq!(
        block_on(mac.mcps_data(McpsDataRequest {
            src_addr_mode: AddressMode::Short,
            dst_address: MacAddress::Short(PAN, ShortAddress(0)),
            payload: &[1],
            msdu_handle: 1,
            tx_options: TxOptions::default(),
        }))
        .err(),
        Some(MacError::RadioError)
    );
    let waited = mac.transport().now.get() - before;
    assert!(
        (1_000_000..2_000_000).contains(&waited),
        "waited {waited} us"
    );
}

#[test]
fn data_seen_while_waiting_for_a_confirm_is_retained() {
    let mut mac = started_mac();
    let frame = DataIndication {
        source: MacAddress::Short(PAN, ShortAddress(0x0000)),
        destination: MacAddress::Short(PAN, ShortAddress(0x4321)),
        lqi: 200,
        rssi: -40,
        security_use: false,
        msdu: MacFrame::from_slice(&[0x08, 0x02]).unwrap(),
    };
    mac.transport_mut()
        .events
        .push_back(Event::DataIndication(frame));
    block_on(mac.mlme_set(PibAttribute::MacRxOnWhenIdle, PibValue::Bool(true))).unwrap();
    let indication = block_on(mac.mcps_data_indication_timeout(0)).unwrap();
    assert_eq!(indication.lqi, 200);
    assert_eq!(indication.payload.as_slice(), &[0x08, 0x02]);
    assert_eq!(
        block_on(mac.mcps_data_indication_timeout(1_000)).err(),
        Some(MacError::NoData)
    );
}

#[test]
fn receive_queue_overflow_is_counted() {
    let mut mac = started_mac();
    for _ in 0..super::RX_QUEUE_DEPTH + 2 {
        mac.transport_mut()
            .events
            .push_back(Event::DataIndication(DataIndication {
                source: MacAddress::Short(PAN, ShortAddress(1)),
                destination: MacAddress::Short(PAN, ShortAddress(0x4321)),
                lqi: 1,
                rssi: 0,
                security_use: false,
                msdu: MacFrame::from_slice(&[0]).unwrap(),
            }));
    }
    let _ = block_on(mac.mac_command_event_timeout(0));
    assert_eq!(mac.stats().rx_dropped, 2);
}

// ── Scan ────────────────────────────────────────────────────────

fn beacon(channel: u8, coord: u16, lqi: u8, payload_len: usize) -> Event {
    let mut payload = heapless::Vec::new();
    for i in 0..payload_len {
        payload.push(if i == 3 { 0xAB } else { 0 }).unwrap();
    }
    Event::BeaconNotify(BeaconNotify {
        channel,
        coordinator: MacAddress::Short(PAN, ShortAddress(coord)),
        superframe_spec: 0x8FFF,
        lqi,
        security_use: false,
        payload,
    })
}

#[test]
fn active_scan_collects_zigbee_beacons_from_notify_indications() {
    let mut mac = mac();
    mac.transport_mut().scan_status = status::NO_BEACON;
    mac.transport_mut().scan_events = [
        beacon(15, 0x0000, 100, 15),
        beacon(15, 0x0000, 180, 15), // duplicate, better LQI
        beacon(15, 0x1111, 90, 4),   // not a Zigbee beacon
        beacon(20, 0x2222, 50, 15),
    ]
    .into();
    let confirm = block_on(mac.mlme_scan(MlmeScanRequest {
        scan_type: ScanType::Active,
        channel_mask: ChannelMask(1 << 15 | 1 << 20),
        scan_duration: 3,
    }))
    .unwrap();
    assert_eq!(confirm.pan_descriptors.len(), 2);
    assert_eq!(confirm.pan_descriptors[0].lqi, 180);
    assert!(
        confirm.pan_descriptors[0]
            .superframe_spec
            .association_permit
    );
    assert_eq!(
        confirm.pan_descriptors[0].zigbee_beacon.extended_pan_id[0],
        0xAB
    );
    assert!(
        mac.transport()
            .sent
            .contains(&Sent::Scan(1, 1 << 15 | 1 << 20, 3))
    );
}

#[test]
fn active_scan_without_beacons_reports_no_beacon() {
    let mut mac = mac();
    mac.transport_mut().scan_status = status::NO_BEACON;
    assert_eq!(
        block_on(mac.mlme_scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask(1 << 11),
            scan_duration: 2,
        }))
        .err(),
        Some(MacError::NoBeacon)
    );
    // Beacons after the scan are not collected.
    mac.transport_mut().events.push_back(beacon(11, 0, 1, 15));
    assert_eq!(
        block_on(mac.mac_command_event_timeout(0)).err(),
        Some(MacError::NoData)
    );
}

#[test]
fn energy_scan_maps_results_to_channels() {
    let mut mac = mac();
    let confirm = block_on(mac.mlme_scan(MlmeScanRequest {
        scan_type: ScanType::Ed,
        channel_mask: ChannelMask(1 << 11 | 1 << 15 | 1 << 26),
        scan_duration: 1,
    }))
    .unwrap();
    let pairs: StdVec<(u8, u8)> = confirm
        .energy_list
        .iter()
        .map(|ed| (ed.channel, ed.energy))
        .collect();
    assert_eq!(pairs, [(11, 0), (15, 10), (26, 20)]);
}

#[test]
fn orphan_and_out_of_band_scans_are_rejected() {
    let mut mac = mac();
    assert_eq!(
        block_on(mac.mlme_scan(MlmeScanRequest {
            scan_type: ScanType::Orphan,
            channel_mask: ChannelMask(1 << 11),
            scan_duration: 1,
        }))
        .err(),
        Some(MacError::Unsupported)
    );
    assert_eq!(
        block_on(mac.mlme_scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask(1 << 5),
            scan_duration: 1,
        }))
        .err(),
        Some(MacError::InvalidParameter)
    );
}

// ── Orphan ──────────────────────────────────────────────────────

#[test]
fn orphan_notification_and_response() {
    let mut mac = started_mac();
    let orphan = [6u8; 8];
    mac.transport_mut()
        .events
        .push_back(Event::OrphanIndication { orphan });
    let event = block_on(mac.mac_command_event_timeout(0)).unwrap();
    assert!(matches!(
        event,
        MacCommandEvent::OrphanNotification(MlmeOrphanIndication { orphan_address, .. })
            if orphan_address == orphan
    ));
    block_on(mac.mlme_orphan_response(MlmeOrphanResponse {
        orphan_address: orphan,
        short_address: ShortAddress(0x0042),
        associated_member: false,
    }))
    .unwrap();
    assert!(mac.transport().sent.is_empty(), "not our child: silent");
    block_on(mac.mlme_orphan_response(MlmeOrphanResponse {
        orphan_address: orphan,
        short_address: ShortAddress(0x0042),
        associated_member: true,
    }))
    .unwrap();
    assert_eq!(mac.transport().sent, [Sent::OrphanResponse(orphan, 0x0042)]);
}

#[test]
fn status_mapping_covers_router_relevant_codes() {
    use super::map_status;
    assert_eq!(map_status(status::NO_ACK), MacError::NoAck);
    assert_eq!(
        map_status(status::TRANSACTION_OVERFLOW),
        MacError::TransactionOverflow
    );
    assert_eq!(
        map_status(status::TRANSACTION_EXPIRED),
        MacError::TransactionExpired
    );
    assert_eq!(
        map_status(status::CHANNEL_ACCESS_FAILURE),
        MacError::ChannelAccessFailure
    );
    assert_eq!(map_status(status::PAN_AT_CAPACITY), MacError::PanAtCapacity);
    assert_eq!(
        map_status(status::UNSUPPORTED_ATTRIBUTE),
        MacError::Unsupported
    );
    assert_eq!(map_status(0x77), MacError::Other);
}
