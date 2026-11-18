//! [`CoordinatorPort`] over `router_app::TrustCenterCoordinatorApp`.
//!
//! The NCP never forms or resumes a network on its own: until the host
//! sends `NWK_INIT` (resume, after applying its counter floors) or
//! `NWK_FORM`, the app is composed but not initialized and the platform
//! loop must not step it ([`TcPort::step`] refuses). No frame is
//! transmitted before the host floors are applied (spec 8.4).
//!
//! Counter floors (`NWK_INIT`):
//! * NWK — before a resume, the durable `global_counter_limit` in the
//!   security journal is raised to the floor, so the resume reserves above
//!   it; while running, the live reservation is raised the same way
//!   (journal first, then the NIB).
//! * APS — `TrustCenterRuntime::raise_global_aps_floor` once the Trust
//!   Center is initialized, before the next step.
//!
//! Host events (`host-passthrough` runtime):
//! * `EVT_DEVICE_JOINED` — Trust Center Update-Device decisions (joins through
//!   any parent, the NCP's own children included), with the join type and
//!   whether the device was admitted.
//! * `EVT_DEVICE_LEFT` — NWK Leave indications heard by the NCP (rejoin bit),
//!   children that aged out, departures reported by parent routers, and
//!   Trust Center removals. The NCP's own child departures reach the Trust
//!   Center as a local Update-Device as well; they are reported once, from
//!   the NWK Leave.
//! * `EVT_APS_DATA_CONFIRM` of an `ack_request` unicast — deferred until its
//!   APS ACK arrives or its retries are exhausted; the stack keeps stepping
//!   in between ([`TcPort::step`] polls the outcome).

use heapless::{Deque, Vec};
use router_app::{
    ApsTableLifecycle, Diagnostics, RouterAppError, RouterObserver, StatusSink, StepEvents,
    Supervisor, TrustCenterCoordinatorApp,
};
use zigbee_aps::ApsTxOptions;
use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_aps::apsme::ApsUpdateDeviceStatus;
use zigbee_aps::{ApsAckHandle, ApsAckStatus, ApsAddress, ApsAddressMode, ApsStatus};
use zigbee_mac::ParentMacDriver;
use zigbee_nwk::host_events::{NwkDeparture, NwkDepartureCause};
use zigbee_nwk::neighbor::NeighborDeviceType;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::child_store::ChildTableStore;
use zigbee_runtime::host_passthrough::{self, HostEndpointError};
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::role::Router;
use zigbee_runtime::security_store::SecurityStateStore;
use zigbee_runtime::trust_center_runtime::{GlobalApsCheckpoint, TrustCenterHostEvent};
use zigbee_runtime::trust_center_store::TrustCenterDeviceStore;
use zigbee_types::ShortAddress;

use crate::port::{
    CoordinatorPort, DataConfirm, DataRequest, DeferredConfirm, Destination, DeviceEvent,
    EndpointError, FormParams, FrameCounters, HostEndpoint, Indication, MAX_HOST_ENDPOINTS,
    NetworkInfo, NetworkState, OpError, joined, left,
};

/// NWK reservation size (`FRAME_COUNTER_RESERVATION_SIZE`).
const NWK_STRIDE: u32 = 0x400;

/// The coordinator application the NCP drives.
#[allow(async_fn_in_trait)]
pub trait TcApp {
    type Mac: ParentMacDriver;
    type Store: SecurityStateStore;
    fn device(&self) -> &ZigbeeDevice<Self::Mac, Router>;
    fn device_mut(&mut self) -> &mut ZigbeeDevice<Self::Mac, Router>;
    fn device_and_store_mut(&mut self) -> (&mut ZigbeeDevice<Self::Mac, Router>, &mut Self::Store);
    async fn initialize(&mut self) -> Result<(), RouterAppError>;
    async fn step(&mut self) -> Result<StepEvents, RouterAppError>;
    /// Clear every durable network, child, APS and Trust Center state.
    async fn wipe(&mut self) -> Result<(), RouterAppError>;
    fn global_aps(&self) -> Option<GlobalApsCheckpoint>;
    fn raise_aps_floor(&mut self, floor: u32) -> Result<(), OpError>;
    /// Next Trust Center decision for the host.
    fn take_tc_event(&mut self) -> Option<TrustCenterHostEvent>;
    /// IEEE of a device the Trust Center knows at `short`.
    fn tc_ieee_by_short(&self, short: u16) -> Option<[u8; 8]>;
}

impl<'a, M, S, P, C, T, St, Sv, D, O, A> TcApp
    for TrustCenterCoordinatorApp<'a, M, S, P, C, T, St, Sv, D, O, A>
where
    M: ParentMacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    C: ChildTableStore,
    T: TrustCenterDeviceStore,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, Router>,
    A: ApsTableLifecycle<M, S, P, Router>,
{
    type Mac = M;
    type Store = S;

    fn device(&self) -> &ZigbeeDevice<M, Router> {
        self.node().device()
    }

    fn device_mut(&mut self) -> &mut ZigbeeDevice<M, Router> {
        self.node_mut().device_mut()
    }

    fn device_and_store_mut(&mut self) -> (&mut ZigbeeDevice<M, Router>, &mut S) {
        self.node_mut().device_and_security_store_mut()
    }

    async fn initialize(&mut self) -> Result<(), RouterAppError> {
        TrustCenterCoordinatorApp::initialize(self).await
    }

    async fn step(&mut self) -> Result<StepEvents, RouterAppError> {
        TrustCenterCoordinatorApp::step(self).await
    }

    async fn wipe(&mut self) -> Result<(), RouterAppError> {
        self.urgent_factory_reset_and_recommission().await
    }

    fn global_aps(&self) -> Option<GlobalApsCheckpoint> {
        self.trust_center().global_aps_counter(self.node().device())
    }

    fn raise_aps_floor(&mut self, floor: u32) -> Result<(), OpError> {
        let (trust_center, device) = self.trust_center_and_device_mut();
        trust_center
            .raise_global_aps_floor(device, floor)
            .map_err(|_| OpError::Storage)
    }

    fn take_tc_event(&mut self) -> Option<TrustCenterHostEvent> {
        let trust_center = self.trust_center_mut();
        let dropped = trust_center.take_host_events_dropped();
        if dropped != 0 {
            log::warn!("[NCP] {} Trust Center host events dropped", dropped);
        }
        trust_center.take_host_event()
    }

    fn tc_ieee_by_short(&self, short: u16) -> Option<[u8; 8]> {
        self.trust_center()
            .table()
            .devices()
            .find(|device| device.active && device.short_address.0 == short)
            .map(|device| device.ieee_address)
    }
}

/// Acknowledged sends awaiting their APS ACK outcome (`max_aps_inflight`).
/// The APS layer tracks fewer (8 on a router build) and evicts the oldest,
/// which then resolves as `NO_ACK`.
pub const MAX_PENDING_ACKS: usize = 16;
/// R22 Table 2-27 `NO_ACK` and `TABLE_FULL` (`zigbee_aps::ApsStatus` does
/// not use the R22 numbering for these).
const APS_NO_ACK: u8 = 0xA7;
const APS_TABLE_FULL: u8 = 0xAE;

/// One acknowledged send awaiting its APS ACK (12 bytes; the NCP keeps
/// [`MAX_PENDING_ACKS`] in RAM).
#[derive(Debug, Clone, Copy)]
struct PendingAck {
    handle: ApsAckHandle,
    req_tid: u16,
    /// Low 16 bits of the transmission time (ms) while pending; the elapsed
    /// time once resolved.
    ms: u16,
    state: AckState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AckState {
    Pending,
    Acked,
    NoAck,
}

/// The NCP's view of the Trust Center coordinator.
pub struct TcPort<A: TcApp> {
    app: A,
    running: bool,
    formed: bool,
    events: Deque<DeviceEvent, 8>,
    clock_ms: fn() -> u32,
    ieee: [u8; 8],
    pending_acks: Vec<PendingAck, MAX_PENDING_ACKS>,
    /// Endpoints registered by the host (`NWK_ADD_ENDPOINT`).
    host_endpoints: Vec<u8, MAX_HOST_ENDPOINTS>,
}

impl<A: TcApp> TcPort<A> {
    /// `ieee`: the radio's EUI-64 (the NIB only learns it at start);
    /// `clock_ms`: monotonic milliseconds for indication timestamps.
    pub fn new(mut app: A, ieee: [u8; 8], clock_ms: fn() -> u32) -> Self {
        let formed = commissioned(&mut app);
        Self {
            app,
            running: false,
            formed,
            events: Deque::new(),
            clock_ms,
            ieee,
            pending_acks: Vec::new(),
            host_endpoints: Vec::new(),
        }
    }

    pub fn app(&self) -> &A {
        &self.app
    }

    pub fn app_mut(&mut self) -> &mut A {
        &mut self.app
    }

    /// Whether the stack runs (the platform loop then calls [`Self::step`]).
    pub const fn running(&self) -> bool {
        self.running
    }

    /// One bounded stack step; collects device events and APS ACK outcomes
    /// for the host.
    pub async fn step(&mut self) -> Result<(), RouterAppError> {
        if !self.running {
            return Err(RouterAppError::NotInitialized);
        }
        self.app.step().await?;
        self.collect_device_events();
        self.poll_acks();
        Ok(())
    }

    /// Raise the durable NWK reservation to at least `floor` (journal
    /// first, then the live NIB when running).
    fn raise_nwk_floor(&mut self, floor: u32) -> Result<(), OpError> {
        if floor == 0 {
            return Ok(());
        }
        let running = self.running;
        let (device, store) = self.app.device_and_store_mut();
        let Some(mut state) = store.load().map_err(|_| OpError::Storage)? else {
            return Err(OpError::NotFormed);
        };
        let live = device.aps().nwk().nib().outgoing_frame_counter;
        if floor <= state.global_counter_limit && (!running || floor <= live) {
            return Ok(());
        }
        let start = floor.max(if running { live } else { 0 });
        let limit = start.checked_add(NWK_STRIDE).ok_or(OpError::Storage)?;
        state.global_counter_limit = state.global_counter_limit.max(limit);
        store.store(&state).map_err(|_| OpError::Storage)?;
        if running
            && !device
                .aps_mut()
                .nwk_mut()
                .nib_mut()
                .set_frame_counter_reservation(start, state.global_counter_limit)
        {
            return Err(OpError::Storage);
        }
        Ok(())
    }
}

impl<A: TcApp> TcPort<A> {
    /// Move NWK departures and Trust Center decisions into the host event
    /// queue while it has room; the rest waits in their own bounded queues.
    fn collect_device_events(&mut self) {
        while !self.events.is_full() {
            let nwk = self.app.device_mut().aps_mut().nwk_mut();
            let dropped = nwk.take_host_departures_dropped();
            if dropped != 0 {
                log::warn!("[NCP] {} device departures dropped", dropped);
            }
            let event = match nwk.take_host_departure() {
                Some(departure) => self.departure_event(departure),
                None => match self.app.take_tc_event() {
                    Some(event) => self.trust_center_event(event),
                    None => break,
                },
            };
            if let Some(event) = event {
                let _ = self.events.push_back(event);
            }
        }
    }

    fn departure_event(&self, departure: NwkDeparture) -> Option<DeviceEvent> {
        let nwk = departure.short_address.0;
        let ieee = departure
            .ieee_address
            .or_else(|| self.app.tc_ieee_by_short(nwk))
            .unwrap_or([0; 8]);
        let (flags, reason) = departure_flags(departure.cause);
        Some(DeviceEvent::Left {
            nwk,
            ieee,
            flags,
            reason,
        })
    }

    fn trust_center_event(&self, event: TrustCenterHostEvent) -> Option<DeviceEvent> {
        let nwk = self.app.device().aps().nwk();
        let own = nwk.nib().ieee_address;
        let TrustCenterHostEvent::Joined {
            device_address,
            short_address,
            parent_address,
            status,
            admitted,
        } = event
        else {
            return left_event(event, own);
        };
        let (parent_nwk, capability) = if parent_address == own {
            let capability = nwk
                .neighbor_table()
                .find_by_short(short_address)
                .filter(|entry| entry.ieee_address == device_address)
                .map_or(0, |entry| {
                    // Reconstructed MAC capability of an NCP child: allocate
                    // address, security, receiver on when idle (which also
                    // stands for mains power), and the device type.
                    let mut capability = 0x80;
                    if entry.security_capable {
                        capability |= 0x40;
                    }
                    if entry.rx_on_when_idle {
                        capability |= 0x08 | 0x04;
                    }
                    if entry.device_type == NeighborDeviceType::Router {
                        capability |= 0x02;
                    }
                    capability
                });
            (nwk.nib().network_address.0, capability)
        } else {
            let parent_nwk = nwk
                .find_short_by_ieee(&parent_address)
                .map_or(0xFFFF, |short| short.0);
            (parent_nwk, 0)
        };
        Some(DeviceEvent::Joined {
            nwk: short_address.0,
            ieee: device_address,
            parent_nwk,
            join_type: join_type(status),
            capability,
            decision: if admitted {
                joined::ADMITTED
            } else {
                joined::DENIED
            },
        })
    }

    /// Latch the APS ACK outcome of every pending send. Confirmed handles
    /// are released at once, so the APS layer's bounded completion record
    /// cannot lose them.
    fn poll_acks(&mut self) {
        let now = (self.clock_ms)();
        let aps = self.app.device_mut().aps_mut();
        for pending in self.pending_acks.iter_mut() {
            if pending.state != AckState::Pending {
                continue;
            }
            pending.state = match aps.ack_status(pending.handle) {
                Some(ApsAckStatus::Pending) => continue,
                Some(ApsAckStatus::Confirmed) => {
                    aps.clear_ack_status(pending.handle);
                    AckState::Acked
                }
                // Retries exhausted, tracking evicted or cancelled.
                None => AckState::NoAck,
            };
            pending.ms = (now as u16).wrapping_sub(pending.ms);
        }
    }

    /// Resolve every pending send as failed (the network is gone).
    fn fail_pending_acks(&mut self) {
        for pending in self.pending_acks.iter_mut() {
            if pending.state == AckState::Pending {
                pending.state = AckState::NoAck;
                pending.ms = 0;
            }
        }
    }
}

/// `EVT_DEVICE_LEFT` flags and reason of a NWK departure.
fn departure_flags(cause: NwkDepartureCause) -> (u8, u8) {
    match cause {
        NwkDepartureCause::Leave { rejoin } => (
            if rejoin { left::FLAG_REJOIN } else { 0 },
            left::REASON_LEAVE,
        ),
        NwkDepartureCause::ChildTimedOut => (0, left::REASON_CHILD_TIMEOUT),
    }
}

/// `EVT_DEVICE_JOINED.join_type` of an Update-Device status.
fn join_type(status: ApsUpdateDeviceStatus) -> u8 {
    match status {
        ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin => joined::JOIN_SECURED_REJOIN,
        ApsUpdateDeviceStatus::StandardDeviceTrustCenterRejoin => joined::JOIN_UNSECURED_REJOIN,
        _ => joined::JOIN_ASSOCIATION,
    }
}

/// `EVT_DEVICE_LEFT` of a Trust Center departure report or removal. A
/// departure of the NCP's own child (reported by `own`) was already
/// reported from its NWK Leave.
fn left_event(event: TrustCenterHostEvent, own: [u8; 8]) -> Option<DeviceEvent> {
    match event {
        TrustCenterHostEvent::Joined { .. } => None,
        TrustCenterHostEvent::Left { parent_address, .. } if parent_address == own => None,
        TrustCenterHostEvent::Left {
            device_address,
            short_address,
            ..
        } => Some(DeviceEvent::Left {
            nwk: short_address.0,
            ieee: device_address,
            flags: left::FLAG_REPORTED_BY_PARENT,
            reason: left::REASON_PARENT_REPORT,
        }),
        TrustCenterHostEvent::Removed {
            device_address,
            short_address,
            ..
        } => Some(DeviceEvent::Left {
            nwk: short_address.0,
            ieee: device_address,
            flags: left::FLAG_REMOVED_BY_TC,
            reason: left::REASON_TC_REMOVAL,
        }),
    }
}

fn commissioned<A: TcApp>(app: &mut A) -> bool {
    let (_, store) = app.device_and_store_mut();
    store
        .load()
        .ok()
        .flatten()
        .is_some_and(|state| state.commissioned)
}

impl<A: TcApp> CoordinatorPort for TcPort<A> {
    fn ieee(&self) -> [u8; 8] {
        self.ieee
    }

    fn state(&self) -> NetworkState {
        if self.running && self.app.device().is_joined() {
            NetworkState::Up
        } else if self.formed {
            NetworkState::FormedDown
        } else {
            NetworkState::None
        }
    }

    fn info(&self) -> NetworkInfo {
        let device = self.app.device();
        let nwk = device.aps().nwk();
        let nib = nwk.nib();
        NetworkInfo {
            state: self.state(),
            pan_id: nib.pan_id.0,
            ext_pan_id: nib.extended_pan_id,
            channel: nib.logical_channel,
            channel_mask: device.bdb().attributes().primary_channel_set.0,
            short: nib.network_address.0,
            ieee: self.ieee,
            nwk_update_id: nib.update_id,
            nwk_manager: nib.nwk_manager_addr.0,
            key_seq: nwk.security().active_key().map_or(0, |key| key.seq_number),
            tx_power: 0x7F,
        }
    }

    fn frame_counters(&self) -> FrameCounters {
        let nib = self.app.device().aps().nwk().nib();
        let aps = self.app.global_aps();
        FrameCounters {
            nwk_current: nib.outgoing_frame_counter,
            nwk_limit: nib.outgoing_frame_counter_limit,
            aps_current: aps.map_or(0, |aps| aps.current),
            aps_limit: aps.map_or(0, |aps| aps.durable_limit),
            aps_commits: aps.map_or(0, |aps| aps.commits),
        }
    }

    fn network_key(&self, which: u8) -> Option<([u8; 16], u8)> {
        if !self.running {
            return None;
        }
        let security = self.app.device().aps().nwk().security();
        let entry = match which {
            0 => security.active_key(),
            1 => security.secondary_key(),
            _ => None,
        }?;
        Some((entry.key, entry.seq_number))
    }

    fn permit_join_remaining_s(&self) -> u8 {
        let nib = self.app.device().aps().nwk().nib();
        if nib.permit_joining {
            nib.permit_joining_duration.max(1)
        } else {
            0
        }
    }

    fn take_indication(&mut self) -> Option<Indication> {
        let record = host_passthrough::take()?;
        let asdu = heapless::Vec::from_slice(&record.asdu).ok()?;
        Some(Indication {
            dst_mode: record.dst_mode,
            dst: record.dst,
            dst_ep: record.dst_ep,
            src_nwk: record.src_nwk,
            src_ieee: record.src_ieee,
            src_ep: record.src_ep,
            profile: record.profile,
            cluster: record.cluster,
            aps_secured: record.aps_secured,
            nwk_secured: record.nwk_secured,
            key_source: record.key_source,
            handled_by_ncp: record.handled_by_ncp,
            lqi: record.lqi,
            rssi: record.rssi,
            last_hop_nwk: record.last_hop_nwk,
            aps_counter: record.aps_counter,
            rx_time_ms: (self.clock_ms)(),
            asdu,
        })
    }

    fn take_device_event(&mut self) -> Option<DeviceEvent> {
        if self.events.is_empty() && self.running {
            self.collect_device_events();
        }
        self.events.pop_front()
    }

    async fn init(&mut self, nwk_fc_floor: u32, aps_fc_floor: u32) -> Result<(), OpError> {
        if !self.running {
            if !commissioned(&mut self.app) {
                return Err(OpError::NotFormed);
            }
            self.raise_nwk_floor(nwk_fc_floor)?;
            self.app
                .initialize()
                .await
                .map_err(|_| OpError::Zigbee(0x01))?;
            self.running = true;
        } else {
            self.raise_nwk_floor(nwk_fc_floor)?;
        }
        if aps_fc_floor != 0 {
            self.app.raise_aps_floor(aps_fc_floor)?;
        }
        Ok(())
    }

    async fn form(&mut self, params: FormParams) -> Result<(), OpError> {
        if self.running || commissioned(&mut self.app) {
            return Err(OpError::InvalidState);
        }
        if !params.ncp_chosen_identity() {
            return Err(OpError::Unsupported);
        }
        if params.channel_mask != 0 {
            let attributes = self.app.device_mut().bdb_mut().attributes_mut();
            attributes.primary_channel_set = zigbee_types::ChannelMask(params.channel_mask);
        }
        self.app
            .initialize()
            .await
            .map_err(|_| OpError::Zigbee(0x01))?;
        self.formed = commissioned(&mut self.app);
        if !self.app.device().is_joined() {
            // Formation failed: no stack to step; the host may retry.
            return Err(OpError::Zigbee(0x01));
        }
        self.running = true;
        // BDB formation opens permit joining; on an NCP only the host opens
        // it (NWK_PERMIT_JOIN). Closed before the first receive step.
        self.app
            .device_mut()
            .permit_joining(0)
            .await
            .map_err(|status| OpError::Zigbee(status as u8))?;
        Ok(())
    }

    async fn permit_join(&mut self, duration_s: u8, target: u16) -> Result<(), OpError> {
        if !self.running {
            return Err(OpError::InvalidState);
        }
        let device = self.app.device_mut();
        device
            .permit_joining(duration_s)
            .await
            .map_err(|status| OpError::Zigbee(status as u8))?;
        if target == 0xFFFC {
            device
                .bdb_mut()
                .zdo_mut()
                .mgmt_permit_joining_req(ShortAddress(0xFFFC), duration_s, true)
                .await
                .map_err(|_| OpError::Zigbee(0x01))?;
        }
        Ok(())
    }

    async fn send(&mut self, request: &DataRequest) -> Result<DataConfirm, OpError> {
        if !self.running {
            return Err(OpError::InvalidState);
        }
        let unicast = match request.dst {
            Destination::Nwk(nwk) => nwk < 0xFFF8,
            Destination::Ieee(_) => true,
            Destination::Group(_) | Destination::Broadcast(_) => false,
        };
        let track_ack = unicast && request.tx_options & 0x01 != 0;
        if track_ack && self.pending_acks.is_full() {
            // Make room: resolved outcomes are drained by the host loop,
            // but never dropped here.
            self.poll_acks();
            if self.pending_acks.is_full() {
                return Err(OpError::Zigbee(APS_TABLE_FULL));
            }
        }
        let (mode, address) = match request.dst {
            Destination::Group(group) => (ApsAddressMode::Group, ApsAddress::Group(group)),
            Destination::Nwk(nwk) | Destination::Broadcast(nwk) => {
                (ApsAddressMode::Short, ApsAddress::Short(ShortAddress(nwk)))
            }
            Destination::Ieee(ieee) => (ApsAddressMode::Extended, ApsAddress::Extended(ieee)),
        };
        let aps_request = ApsdeDataRequest {
            dst_addr_mode: mode,
            dst_address: address,
            dst_endpoint: request.dst_ep,
            profile_id: request.profile,
            cluster_id: request.cluster,
            src_endpoint: request.src_ep,
            payload: &request.asdu,
            tx_options: ApsTxOptions {
                security_enabled: request.tx_options & 0x02 != 0,
                ack_request: request.tx_options & 0x01 != 0,
                ..ApsTxOptions::default()
            },
            radius: request.radius,
            alias_src_addr: None,
            alias_seq: None,
        };
        let dst_nwk = match request.dst {
            Destination::Nwk(nwk) | Destination::Broadcast(nwk) => nwk,
            Destination::Group(_) => 0xFFFD,
            Destination::Ieee(ieee) => self
                .app
                .device()
                .aps()
                .nwk()
                .find_short_by_ieee(&ieee)
                .map_or(0xFFFF, |short| short.0),
        };
        let sent_ms = (self.clock_ms)() as u16;
        let aps = self.app.device_mut().aps_mut();
        let confirm = match aps.apsde_data_request(&aps_request).await {
            Ok(confirm) => confirm,
            Err(status) => return Err(OpError::Zigbee(status as u8)),
        };
        let mut result = DataConfirm {
            zb_status: confirm.status as u8,
            aps_counter: confirm.aps_counter,
            dst_nwk,
            flags: 0,
        };
        if track_ack && confirm.status == ApsStatus::Success {
            // The handle the APS layer registered for this frame. Without
            // one (not tracked) the confirm reports the transmission only.
            if let Some(handle) = aps
                .last_ack_handle()
                .filter(|handle| handle.aps_counter() == confirm.aps_counter)
            {
                result.dst_nwk = handle.destination().0;
                let pending = PendingAck {
                    handle,
                    req_tid: request.req_tid,
                    ms: sent_ms,
                    state: AckState::Pending,
                };
                if self.pending_acks.push(pending).is_ok() {
                    result.flags = DataConfirm::DEFERRED;
                }
            }
        }
        Ok(result)
    }

    async fn leave_reset(&mut self) -> Result<(), OpError> {
        self.app.wipe().await.map_err(|_| OpError::Storage)?;
        host_passthrough::clear();
        self.app
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .clear_host_departures();
        while self.app.take_tc_event().is_some() {}
        self.fail_pending_acks();
        self.running = false;
        self.formed = false;
        Ok(())
    }

    fn take_data_confirm(&mut self) -> Option<DeferredConfirm> {
        if self.running {
            self.poll_acks();
        }
        let index = self
            .pending_acks
            .iter()
            .position(|pending| pending.state != AckState::Pending)?;
        let pending = self.pending_acks.remove(index);
        let (zb_status, flags) = match pending.state {
            AckState::Acked => (0, DataConfirm::FLAG_ACK_RECEIVED),
            _ => (APS_NO_ACK, 0),
        };
        let elapsed_ms = pending.ms;
        Some(DeferredConfirm {
            req_tid: pending.req_tid,
            confirm: DataConfirm {
                zb_status,
                aps_counter: pending.handle.aps_counter(),
                dst_nwk: pending.handle.destination().0,
                flags,
            },
            elapsed_ms,
        })
    }

    fn max_host_endpoints(&self) -> u8 {
        MAX_HOST_ENDPOINTS as u8
    }

    fn add_endpoint(&mut self, endpoint: &HostEndpoint) -> Result<(), EndpointError> {
        if !(1..=240).contains(&endpoint.endpoint) {
            return Err(EndpointError::InvalidParameter);
        }
        let known = self.host_endpoints.contains(&endpoint.endpoint);
        if !known && self.host_endpoints.is_full() {
            return Err(EndpointError::NoSpace);
        }
        // A host endpoint replaces a product-profile endpoint with the same
        // number; that one is not restored by `clear_endpoints`.
        self.app
            .device_mut()
            .register_host_endpoint(
                endpoint.endpoint,
                endpoint.profile,
                endpoint.device_id,
                endpoint.version,
                &endpoint.in_clusters,
                &endpoint.out_clusters,
            )
            .map_err(|error| match error {
                HostEndpointError::RegistryFull => EndpointError::NoSpace,
                _ => EndpointError::InvalidParameter,
            })?;
        if !known {
            let _ = self.host_endpoints.push(endpoint.endpoint);
        }
        Ok(())
    }

    fn clear_endpoints(&mut self) {
        let device = self.app.device_mut();
        for endpoint in self.host_endpoints.iter() {
            device.unregister_host_endpoint(*endpoint);
        }
        self.host_endpoints.clear();
    }
}

/// A borrowed coordinator app, so the app can live in its own static
/// storage (constructed in place) and [`TcPort`] holds only a reference.
impl<T: TcApp + ?Sized> TcApp for &mut T {
    type Mac = T::Mac;
    type Store = T::Store;

    fn device(&self) -> &ZigbeeDevice<Self::Mac, Router> {
        (**self).device()
    }

    fn device_mut(&mut self) -> &mut ZigbeeDevice<Self::Mac, Router> {
        (**self).device_mut()
    }

    fn device_and_store_mut(&mut self) -> (&mut ZigbeeDevice<Self::Mac, Router>, &mut Self::Store) {
        (**self).device_and_store_mut()
    }

    async fn initialize(&mut self) -> Result<(), RouterAppError> {
        (**self).initialize().await
    }

    async fn step(&mut self) -> Result<StepEvents, RouterAppError> {
        (**self).step().await
    }

    async fn wipe(&mut self) -> Result<(), RouterAppError> {
        (**self).wipe().await
    }

    fn global_aps(&self) -> Option<GlobalApsCheckpoint> {
        (**self).global_aps()
    }

    fn raise_aps_floor(&mut self, floor: u32) -> Result<(), OpError> {
        (**self).raise_aps_floor(floor)
    }

    fn take_tc_event(&mut self) -> Option<TrustCenterHostEvent> {
        (**self).take_tc_event()
    }

    fn tc_ieee_by_short(&self, short: u16) -> Option<[u8; 8]> {
        (**self).tc_ieee_by_short(short)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OWN: [u8; 8] = [0x10; 8];
    const DEVICE: [u8; 8] = [0x21; 8];
    const ROUTER: [u8; 8] = [0x22; 8];

    #[test]
    fn trust_center_departures_map_to_device_left() {
        let short_address = ShortAddress(0x4455);
        let reported = TrustCenterHostEvent::Left {
            device_address: DEVICE,
            short_address,
            parent_address: ROUTER,
        };
        assert_eq!(
            left_event(reported, OWN),
            Some(DeviceEvent::Left {
                nwk: 0x4455,
                ieee: DEVICE,
                flags: left::FLAG_REPORTED_BY_PARENT,
                reason: left::REASON_PARENT_REPORT,
            })
        );
        let own_child = TrustCenterHostEvent::Left {
            device_address: DEVICE,
            short_address,
            parent_address: OWN,
        };
        assert_eq!(left_event(own_child, OWN), None, "reported from its Leave");
        let removed = TrustCenterHostEvent::Removed {
            device_address: DEVICE,
            short_address,
            parent_address: OWN,
        };
        assert_eq!(
            left_event(removed, OWN),
            Some(DeviceEvent::Left {
                nwk: 0x4455,
                ieee: DEVICE,
                flags: left::FLAG_REMOVED_BY_TC,
                reason: left::REASON_TC_REMOVAL,
            })
        );
    }

    #[test]
    fn join_types_and_departure_causes_use_the_zrhp_values() {
        assert_eq!(
            join_type(ApsUpdateDeviceStatus::StandardDeviceUnsecuredJoin),
            0
        );
        assert_eq!(
            join_type(ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin),
            1
        );
        assert_eq!(
            join_type(ApsUpdateDeviceStatus::StandardDeviceTrustCenterRejoin),
            2
        );
        assert_eq!(
            departure_flags(NwkDepartureCause::Leave { rejoin: true }),
            (0x01, 1)
        );
        assert_eq!(
            departure_flags(NwkDepartureCause::Leave { rejoin: false }),
            (0, 1)
        );
        assert_eq!(departure_flags(NwkDepartureCause::ChildTimedOut), (0, 4));
    }
}
