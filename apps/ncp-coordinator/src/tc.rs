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
use zigbee_runtime::security_store::{PersistentSecurityState, SecurityStateStore};
use zigbee_runtime::trust_center_runtime::{GlobalApsCheckpoint, TrustCenterHostEvent};
use zigbee_runtime::trust_center_store::TrustCenterDeviceStore;
use zigbee_types::ShortAddress;

use crate::port::{
    CoordinatorPort, DataConfirm, DataRequest, DeferredConfirm, Destination, DeviceEvent,
    EndpointError, FormParams, FrameCounters, HostEndpoint, Indication, MAX_HOST_ENDPOINTS,
    NetworkInfo, NetworkState, OpError, joined, left,
};
use crate::port::{DeviceView, LinkView, NetworkImage, RestoredChild, RestoredKey};
use zigbee_mac::{MacDriver as _, PlatformServices as _};

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
    /// Allow `initialize` again after a wipe (see
    /// `TrustCenterCoordinatorApp::rearm_initialize`).
    fn rearm_initialize(&mut self) -> bool;
    fn global_aps(&self) -> Option<GlobalApsCheckpoint>;
    fn raise_aps_floor(&mut self, floor: u32) -> Result<(), OpError>;
    /// Next Trust Center decision for the host.
    fn take_tc_event(&mut self) -> Option<TrustCenterHostEvent>;
    /// IEEE of a device the Trust Center knows at `short`.
    fn tc_ieee_by_short(&self, short: u16) -> Option<[u8; 8]>;
    /// The Trust Center device table.
    fn tc_table(&self) -> &zigbee_bdb::trust_center::TrustCenterTable;
    /// Provision a unique TCLK before join (durable).
    fn provision_link_key(&mut self, ieee: [u8; 8], key: [u8; 16]) -> Result<(), OpError>;
    /// Durably provision the key derived from an install code (with its
    /// CRC) before the device joins; returns the key.
    fn provision_install_code(&mut self, ieee: [u8; 8], code: &[u8]) -> Result<[u8; 16], OpError>;
    /// Durably drop a device's record and key; `false` when unknown.
    fn forget_device(&mut self, ieee: [u8; 8]) -> Result<bool, OpError>;
    /// The highest global outgoing APS counter any key may have used
    /// (live, or read from the journal while the stack is stopped).
    fn aps_counter_bound(&mut self) -> Result<u32, OpError>;
    /// Restore a key from a backup with origin, attributes and location.
    fn restore_link_key(
        &mut self,
        ieee: [u8; 8],
        key: [u8; 16],
        origin: zigbee_bdb::trust_center::TrustCenterKeyOrigin,
        attributes: zigbee_bdb::trust_center::TrustCenterKeyAttributes,
        location: Option<([u8; 8], u16)>,
    ) -> Result<(), OpError>;
    /// Replace the durable child table (restored at the next `initialize`).
    fn store_children(
        &mut self,
        table: &zigbee_runtime::child_store::PersistentChildTable,
    ) -> Result<(), OpError>;
    /// Erase one unit of a journal's spare sector ahead of its rollover
    /// (`journal-pre-erase`). Returns whether it erased.
    fn pre_erase_step(&mut self) -> bool {
        false
    }

    /// Move the Trust Center snapshot into its pre-erased spare sector
    /// when the journal asks (see `TrustCenterRuntime::compact_journal`).
    /// Returns whether it stored.
    fn compact_journal(&mut self) -> bool {
        false
    }
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

    fn rearm_initialize(&mut self) -> bool {
        TrustCenterCoordinatorApp::rearm_initialize(self)
    }

    fn store_children(
        &mut self,
        table: &zigbee_runtime::child_store::PersistentChildTable,
    ) -> Result<(), OpError> {
        self.coordinator_mut()
            .children_mut()
            .store_mut()
            .store(table)
            .map_err(|_| OpError::Storage)
    }

    fn pre_erase_step(&mut self) -> bool {
        // One erase unit per call: the Trust Center journal first (its
        // 32 KiB rollover erase is the long one), then the child journal.
        // A failed pre-erase only leaves the rollover its full erase.
        match self.trust_center_mut().store_mut().pre_erase_step() {
            Ok(true) => return true,
            Ok(false) => {}
            Err(error) => log::warn!("[NCP] Trust Center pre-erase failed: {error:?}"),
        }
        match self
            .coordinator_mut()
            .children_mut()
            .store_mut()
            .pre_erase_step()
        {
            Ok(erased) => erased,
            Err(error) => {
                log::warn!("[NCP] child journal pre-erase failed: {error:?}");
                false
            }
        }
    }

    fn compact_journal(&mut self) -> bool {
        self.trust_center_mut()
            .compact_journal()
            .inspect_err(|error| log::warn!("[NCP] Trust Center compaction failed: {error:?}"))
            .unwrap_or(false)
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

    fn tc_table(&self) -> &zigbee_bdb::trust_center::TrustCenterTable {
        self.trust_center().table()
    }

    fn provision_link_key(&mut self, ieee: [u8; 8], key: [u8; 16]) -> Result<(), OpError> {
        let (trust_center, device) = self.trust_center_and_device_mut();
        trust_center
            .provision_link_key(device, ieee, key)
            .map_err(|_| OpError::Storage)
    }

    fn provision_install_code(&mut self, ieee: [u8; 8], code: &[u8]) -> Result<[u8; 16], OpError> {
        use zigbee_bdb::trust_center::TrustCenterError;
        use zigbee_runtime::trust_center_runtime::TrustCenterRuntimeError;
        let (trust_center, device) = self.trust_center_and_device_mut();
        trust_center
            .provision_install_code(device, ieee, code)
            .map_err(|error| match error {
                TrustCenterRuntimeError::Policy(
                    TrustCenterError::InvalidInstallCode(_) | TrustCenterError::InvalidAddress,
                ) => OpError::InvalidParameter,
                _ => OpError::Storage,
            })
    }

    fn forget_device(&mut self, ieee: [u8; 8]) -> Result<bool, OpError> {
        let (trust_center, device) = self.trust_center_and_device_mut();
        trust_center
            .forget_device(device, ieee)
            .map_err(|_| OpError::Storage)
    }

    fn aps_counter_bound(&mut self) -> Result<u32, OpError> {
        let (trust_center, device) = self.trust_center_and_device_mut();
        trust_center
            .outgoing_aps_bound(device)
            .map_err(|_| OpError::Storage)
    }

    fn restore_link_key(
        &mut self,
        ieee: [u8; 8],
        key: [u8; 16],
        origin: zigbee_bdb::trust_center::TrustCenterKeyOrigin,
        attributes: zigbee_bdb::trust_center::TrustCenterKeyAttributes,
        location: Option<([u8; 8], u16)>,
    ) -> Result<(), OpError> {
        let (trust_center, device) = self.trust_center_and_device_mut();
        trust_center
            .restore_link_key(
                device,
                ieee,
                key,
                origin,
                attributes,
                location.map(|(parent, short)| (parent, ShortAddress(short))),
            )
            .map_err(|_| OpError::Storage)
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
    /// Transmission time (ms) while pending; the elapsed time once
    /// resolved (saturated to `u16` in the confirm: a sleepy destination
    /// can take longer than 65 s over every retry).
    ms: u32,
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
    /// The committed network (formed), read from the store at boot and
    /// after every formation, install and wipe.
    stored: Option<StoredNetwork>,
    events: Deque<DeviceEvent, 8>,
    clock_ms: fn() -> u32,
    ieee: [u8; 8],
    pending_acks: Vec<PendingAck, MAX_PENDING_ACKS>,
    /// Endpoints registered by the host (`NWK_ADD_ENDPOINT`).
    host_endpoints: Vec<u8, MAX_HOST_ENDPOINTS>,
    /// Highest NWK / global APS outgoing counter bounds seen in this
    /// session. A wipe (`NWK_LEAVE_RESET`, re-form) clears the Trust Center
    /// journal, so the next installed network (possibly the same keys from
    /// a restore) floors its counters here. Not durable across a reboot.
    retained_nwk_bound: u32,
    retained_aps_bound: u32,
    /// Journal pre-erase in radio-quiet windows (see `service_pre_erase`).
    pre_erase: PreEraseSchedule,
}

/// When the NCP may spend a flash erase (the ROM driver masks interrupts,
/// the 802.15.4 receiver's included, for ~30 ms per 4 KiB).
#[derive(Debug, Clone, Copy, Default)]
struct PreEraseSchedule {
    /// Secured NWK frames received, as last seen.
    secured_frames: u32,
    /// Start of the current quiet window (ms).
    quiet_since_ms: u32,
    /// Last pre-erase step (ms).
    last_step_ms: u32,
    /// A journal may have a spare sector to erase (activity since the last
    /// pass, or the last step erased).
    armed: bool,
    /// Platform check that the radio is idle-listening right now (no ACK
    /// being sent, no interrupt work pending); `None`: always.
    radio_idle: Option<fn() -> bool>,
}

/// No secured frame for this long before an erase (ms). Short on purpose: a
/// join stores the Trust Center state about ten times and can roll a 32 KiB
/// sector over twice, so the spare must be erased between its exchanges.
const PRE_ERASE_QUIET_MS: u32 = 150;
/// At most one ~30 ms erase per this interval (ms), so MAC retries and the
/// next frames of a burst get through between them.
const PRE_ERASE_SPACING_MS: u32 = 250;

impl<A: TcApp> TcPort<A> {
    /// `ieee`: the radio's EUI-64 (the NIB only learns it at start);
    /// `clock_ms`: monotonic milliseconds for indication timestamps.
    pub fn new(mut app: A, ieee: [u8; 8], clock_ms: fn() -> u32) -> Self {
        crate::gp::reset_defaults(&mut app);
        let mut port = Self {
            app,
            running: false,
            stored: None,
            events: Deque::new(),
            clock_ms,
            ieee,
            pending_acks: Vec::new(),
            host_endpoints: Vec::new(),
            retained_nwk_bound: 0,
            retained_aps_bound: 0,
            pre_erase: PreEraseSchedule {
                armed: true,
                ..PreEraseSchedule::default()
            },
        };
        // Probed after construction so that the port is built in place in
        // the caller's frame (the boot chain).
        port.stored = stored_network(&mut port.app);
        port
    }

    pub fn app(&self) -> &A {
        &self.app
    }

    /// Platform check that the radio is idle-listening, consulted before a
    /// journal pre-erase (an erase masks interrupts; one that starts while
    /// the radio sends an automatic ACK leaves it stuck until recovered).
    pub fn set_radio_idle_check(&mut self, radio_idle: fn() -> bool) {
        self.pre_erase.radio_idle = Some(radio_idle);
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
        self.service_pre_erase();
        Ok(())
    }

    /// Erase one unit of a journal's spare sector when the radio is quiet:
    /// no secured frame for [`PRE_ERASE_QUIET_MS`], no APS ACK awaited, no
    /// frame queued for a sleepy child and joining closed. The erase a
    /// journal rollover would otherwise do inside a store — up to 8 × ~30 ms
    /// of a deaf receiver in the middle of a join — happens here instead,
    /// in steps at least [`PRE_ERASE_SPACING_MS`] apart.
    #[inline(never)]
    fn service_pre_erase(&mut self) {
        let now = (self.clock_ms)();
        let nwk = self.app.device().aps().nwk();
        let secured_frames = nwk.rx_security_stats().secured_frames;
        let busy = secured_frames != self.pre_erase.secured_frames
            || !self.pending_acks.is_empty()
            || nwk.indirect_queue().pending_children().next().is_some()
            || nwk.nib().permit_joining;
        if busy {
            self.pre_erase.secured_frames = secured_frames;
            self.pre_erase.quiet_since_ms = now;
            self.pre_erase.armed = true;
            return;
        }
        if !self.pre_erase.armed
            || now.wrapping_sub(self.pre_erase.quiet_since_ms) < PRE_ERASE_QUIET_MS
            || now.wrapping_sub(self.pre_erase.last_step_ms) < PRE_ERASE_SPACING_MS
            || self.pre_erase.radio_idle.is_some_and(|idle| !idle())
        {
            return;
        }
        self.pre_erase.last_step_ms = now;
        // Spare sectors first; once they are erased, a half-used Trust
        // Center sector moves into its spare (a write, no erase), so the
        // next join starts with a free sector and an erased spare: no
        // erase inside its exchanges (hardware: urgent erases between a
        // join's frames cost Transport-Key deliveries).
        self.pre_erase.armed = self.app.pre_erase_step() || self.app.compact_journal();
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
        if let TrustCenterHostEvent::Authorized {
            device_address,
            short_address,
            result,
            unique_link_key,
        } = event
        {
            let nwk = if short_address.0 < 0xFFF8 {
                short_address.0
            } else {
                nwk.neighbor_table()
                    .children()
                    .find(|entry| entry.ieee_address == device_address)
                    .map_or(0xFFFF, |entry| entry.network_address.0)
            };
            return Some(DeviceEvent::Authorized {
                nwk,
                ieee: device_address,
                result,
                link_key_type: u8::from(unique_link_key),
            });
        }
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
                .map_or(0, child_capability);
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
            pending.ms = now.wrapping_sub(pending.ms);
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
        TrustCenterHostEvent::Joined { .. } | TrustCenterHostEvent::Authorized { .. } => None,
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

fn is_child(entry: &zigbee_nwk::neighbor::NeighborEntry) -> bool {
    matches!(
        entry.relationship,
        zigbee_nwk::neighbor::Relationship::Child
            | zigbee_nwk::neighbor::Relationship::UnauthenticatedChild
    )
}

/// The committed network's identity, as `NWK_GET_INFO` reports it while
/// the network is formed but not running (spec 7.2, 7.11 step 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct StoredNetwork {
    pan_id: u16,
    ext_pan_id: [u8; 8],
    channel: u8,
    update_id: u8,
    key_seq: u8,
}

/// Commit a new operating channel and update ID to the network record.
#[inline(never)]
fn commit_channel<A: TcApp>(app: &mut A, channel: u8, update_id: u8) -> Result<(), OpError> {
    let (_, store) = app.device_and_store_mut();
    let mut state = store
        .load()
        .map_err(|_| OpError::Storage)?
        .ok_or(OpError::NotFormed)?;
    state.channel = channel;
    state.update_id = update_id;
    state.update_id_valid = true;
    store.store(&state).map_err(|_| OpError::Storage)
}

/// The committed network in the security store, if any.
#[inline(never)]
fn stored_network<A: TcApp>(app: &mut A) -> Option<StoredNetwork> {
    let (_, store) = app.device_and_store_mut();
    let state = store.load().ok().flatten()?;
    state.commissioned.then_some(StoredNetwork {
        pan_id: state.pan_id,
        ext_pan_id: state.extended_pan_id,
        channel: state.channel,
        update_id: state.update_id,
        key_seq: state.key_sequence,
    })
}

impl<A: TcApp> CoordinatorPort for TcPort<A> {
    fn ieee(&self) -> [u8; 8] {
        self.ieee
    }

    fn diag_count(&self, table: crate::port::DiagTable) -> usize {
        let nwk = self.app.device().aps().nwk();
        match table {
            crate::port::DiagTable::Neighbors => nwk.neighbor_table().iter().count(),
            crate::port::DiagTable::Routes => nwk.routing_table().iter().count(),
            crate::port::DiagTable::Children => {
                nwk.neighbor_table().iter().filter(|e| is_child(e)).count()
            }
        }
    }

    /// Mgmt_Lqi codes (R22 Table 2-127): device type 0 coordinator,
    /// 1 router, 2 end device, 3 unknown; relationship 0 parent, 1 child,
    /// 2 sibling, 4 previous child, 5 unauthenticated child.
    fn diag_neighbor(&self, index: usize) -> Option<zigbee_zrhp::msg::diag::Neighbor> {
        use zigbee_nwk::neighbor::Relationship;
        let entry = self
            .app
            .device()
            .aps()
            .nwk()
            .neighbor_table()
            .iter()
            .nth(index)?;
        Some(zigbee_zrhp::msg::diag::Neighbor {
            ieee: zigbee_zrhp::types::Ieee(entry.ieee_address),
            nwk: entry.network_address.0,
            device_type: match entry.device_type {
                NeighborDeviceType::Coordinator => 0,
                NeighborDeviceType::Router => 1,
                NeighborDeviceType::EndDevice => 2,
                NeighborDeviceType::Unknown => 3,
            },
            rx_on: u8::from(entry.rx_on_when_idle),
            relationship: match entry.relationship {
                Relationship::Parent => 0,
                Relationship::Child => 1,
                Relationship::Sibling => 2,
                Relationship::PreviousChild => 4,
                Relationship::UnauthenticatedChild => 5,
            },
            lqi: entry.lqi,
            outgoing_cost: entry.outgoing_cost,
            age: entry.age.min(u16::from(u8::MAX)) as u8,
        })
    }

    /// Mgmt_Rtg status codes (R22 Table 2-129); flags bit0 memory
    /// constrained (never), bit1 many-to-one, bit2 route record required
    /// (the Mgmt_Rtg status byte's bits 3–5).
    fn diag_route(&self, index: usize) -> Option<zigbee_zrhp::msg::diag::Route> {
        use zigbee_nwk::routing::RouteStatus;
        let route = self
            .app
            .device()
            .aps()
            .nwk()
            .routing_table()
            .iter()
            .nth(index)?;
        Some(zigbee_zrhp::msg::diag::Route {
            dst: route.destination.0,
            next_hop: route.next_hop.0,
            status: match route.status {
                RouteStatus::Active => 0,
                RouteStatus::DiscoveryUnderway => 1,
                RouteStatus::DiscoveryFailed => 2,
                RouteStatus::Inactive => 3,
                RouteStatus::ValidationUnderway => 4,
            },
            flags: (u8::from(route.many_to_one) << 1)
                | (u8::from(route.route_record_required) << 2),
        })
    }

    /// `last_poll_ms`: time since the child last proved liveness (poll or
    /// keepalive) as the End Device Timeout countdown shows it; `u32::MAX`
    /// until it has been heard this power cycle.
    fn diag_child(&self, index: usize) -> Option<zigbee_zrhp::msg::diag::Child> {
        let entry = self
            .app
            .device()
            .aps()
            .nwk()
            .neighbor_table()
            .iter()
            .filter(|e| is_child(e))
            .nth(index)?;
        let last_poll_ms = if entry.keepalive_confirmed {
            zigbee_nwk::frames::ed_timeout_enum_to_seconds(entry.end_device_timeout).map_or(
                u32::MAX,
                |timeout| {
                    timeout
                        .saturating_sub(entry.keepalive_remaining_secs)
                        .saturating_mul(1000)
                },
            )
        } else {
            u32::MAX
        };
        Some(zigbee_zrhp::msg::diag::Child {
            ieee: zigbee_zrhp::types::Ieee(entry.ieee_address),
            nwk: entry.network_address.0,
            capability: child_capability(entry),
            timeout_exp: entry.end_device_timeout,
            last_poll_ms,
        })
    }

    fn state(&self) -> NetworkState {
        if self.running && self.app.device().is_joined() {
            NetworkState::Up
        } else if self.stored.is_some() {
            NetworkState::FormedDown
        } else {
            NetworkState::None
        }
    }

    fn info(&self) -> NetworkInfo {
        let device = self.app.device();
        let nwk = device.aps().nwk();
        let nib = nwk.nib();
        let state = self.state();
        if let (NetworkState::FormedDown, Some(stored)) = (state, self.stored) {
            // Formed but not started (after a reboot, before `NWK_INIT`):
            // the committed identity, flagged by `state`.
            return NetworkInfo {
                state,
                pan_id: stored.pan_id,
                ext_pan_id: stored.ext_pan_id,
                channel: stored.channel,
                channel_mask: device.bdb().attributes().primary_channel_set.0,
                short: 0x0000,
                ieee: self.ieee,
                nwk_update_id: stored.update_id,
                nwk_manager: 0x0000,
                key_seq: stored.key_seq,
                tx_power: 0x7F,
            };
        }
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
            if stored_network(&mut self.app).is_none() {
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
        if self.running || stored_network(&mut self.app).is_some() {
            return Err(OpError::InvalidState);
        }
        if !params.ncp_chosen_identity() {
            return self.form_with_identity(&params).await;
        }
        if params.channel_mask != 0 {
            let attributes = self.app.device_mut().bdb_mut().attributes_mut();
            attributes.primary_channel_set = zigbee_types::ChannelMask(params.channel_mask);
        }
        // A wipe in this session (`NWK_LEAVE_RESET`) consumed the one
        // initialization.
        self.app.rearm_initialize();
        self.app
            .initialize()
            .await
            .map_err(|_| OpError::Zigbee(0x01))?;
        self.stored = stored_network(&mut self.app);
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
                // Every frame of a secured network carries NWK security
                // (the APS default is off; only the APS retry path set it).
                use_nwk_key: true,
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
        let sent_ms = (self.clock_ms)();
        let aps = self.app.device_mut().aps_mut();
        // bit5: a sleepy destination behind another parent gets that
        // parent's indirect hold added to every ACK wait. A sleepy child of
        // the NCP always waits for its learned hold (zigbee-aps
        // `adaptive-indirect`).
        aps.set_extended_timeout(track_ack && request.tx_options & 0x20 != 0);
        let confirm = aps.apsde_data_request(&aps_request).await;
        let aps = self.app.device_mut().aps_mut();
        aps.set_extended_timeout(false);
        let confirm = match confirm {
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
        if self.stored.is_some() {
            self.retain_counter_bounds()?;
        }
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
        self.stored = None;
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
        let elapsed_ms = u16::try_from(pending.ms).unwrap_or(u16::MAX);
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

    fn channel_change_supported(&self) -> bool {
        true
    }

    fn energy_scan_supported(&self) -> bool {
        true
    }

    async fn energy_detect(&mut self, channel: u8, duration_exp: u8) -> Result<u8, OpError> {
        // MLME-SCAN (ED) of the one channel; the MAC returns to the
        // operating channel and address filter afterwards.
        let scan = self
            .app
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .nlme_ed_scan(zigbee_types::ChannelMask(1 << channel), duration_exp)
            .await
            .map_err(|status| OpError::Zigbee(status as u8))?;
        scan.energy_list
            .iter()
            .find(|ed| ed.channel == channel)
            .map(|ed| ed.energy)
            .ok_or(OpError::Zigbee(0x01))
    }

    fn active_scan_supported(&self) -> bool {
        true
    }

    async fn active_scan(
        &mut self,
        channel: u8,
        duration_exp: u8,
        sink: &mut dyn FnMut(&crate::port::Beacon),
    ) -> Result<(), OpError> {
        use zigbee_mac::{MacError, MlmeScanRequest, ScanType};
        use zigbee_types::MacAddress;
        // MLME-SCAN (active) of the one channel; the MAC returns to the
        // operating channel and address filter afterwards.
        let result = self
            .app
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .mlme_scan(MlmeScanRequest {
                scan_type: ScanType::Active,
                channel_mask: zigbee_types::ChannelMask(1 << channel),
                scan_duration: duration_exp,
            })
            .await;
        let confirm = match result {
            Ok(confirm) => confirm,
            Err(MacError::NoBeacon) => return Ok(()),
            Err(_) => return Err(OpError::Zigbee(0xEA)),
        };
        for pd in confirm.pan_descriptors.iter() {
            // Zigbee networks only: a beacon of another protocol (Thread's
            // protocol ID 3, a plain IEEE 802.15.4 coordinator) parses into
            // zeros, and a Zigbee router always announces its network's
            // EPID.
            if pd.zigbee_beacon.protocol_id != 0 || pd.zigbee_beacon.extended_pan_id == [0; 8] {
                continue;
            }
            let (pan, src) = match pd.coord_address {
                MacAddress::Short(pan, short) => (pan.0, short.0),
                MacAddress::Extended(pan, _) => (pan.0, 0xFFFF),
            };
            sink(&crate::port::Beacon {
                channel: pd.channel,
                pan,
                epid: pd.zigbee_beacon.extended_pan_id,
                src,
                lqi: pd.lqi,
                permit: pd.superframe_spec.association_permit,
                stack_profile: pd.zigbee_beacon.stack_profile,
                update_id: pd.zigbee_beacon.update_id,
            });
        }
        Ok(())
    }

    async fn set_tx_power(&mut self, dbm: i8) -> Result<(), OpError> {
        self.app
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .mlme_set(
                zigbee_mac::PibAttribute::PhyTransmitPower,
                zigbee_mac::PibValue::I8(dbm),
            )
            .await
            .map_err(|_| OpError::InvalidParameter)
    }

    async fn tx_power(&mut self) -> Option<i8> {
        match self
            .app
            .device()
            .aps()
            .nwk()
            .mac()
            .mlme_get(zigbee_mac::PibAttribute::PhyTransmitPower)
            .await
        {
            Ok(zigbee_mac::PibValue::I8(dbm)) => Some(dbm),
            _ => None,
        }
    }

    /// NWK group (`0x0301..`): the NWK layer's receive-security counters.
    fn visit_counters(&self, group_mask: u8, sink: &mut dyn FnMut(u16, u32)) {
        if group_mask & 0x04 == 0 {
            return;
        }
        let stats = self.app.device().aps().nwk().rx_security_stats();
        for (id, value) in [
            (0x0301, stats.secured_frames),
            (0x0302, stats.decrypt_failures),
            (0x0303, stats.missing_keys),
            (0x0304, stats.replay_rejections),
            (0x0305, stats.replay_table_full),
            (0x0306, stats.security_header_parse_failures),
        ] {
            sink(id, value);
        }
    }

    fn reset_counters(&mut self, group_mask: u8) {
        if group_mask & 0x04 != 0 {
            *self
                .app
                .device_mut()
                .aps_mut()
                .nwk_mut()
                .rx_security_stats_mut() = Default::default();
        }
    }

    fn commit_channel(&mut self, channel: u8, update_id: u8) -> Result<(), OpError> {
        if !self.running {
            return Err(OpError::InvalidState);
        }
        commit_channel(&mut self.app, channel, update_id)?;
        // The NWK holds the pending channel too: the security checkpoints
        // then persist it (not the current one) until the switch, and its
        // maintenance retunes after the delivery time even if the NCP loop
        // is late.
        let nwk = self.app.device_mut().aps_mut().nwk_mut();
        nwk.nib_mut().set_nwk_update_id(update_id);
        nwk.arm_channel_change(channel);
        self.stored = stored_network(&mut self.app);
        Ok(())
    }

    async fn switch_channel(&mut self, channel: u8) -> Result<(), OpError> {
        if !self.running {
            return Err(OpError::InvalidState);
        }
        let nwk = self.app.device_mut().aps_mut().nwk_mut();
        if nwk.pending_channel_change().is_none() && nwk.nib().logical_channel == channel {
            // The NWK maintenance already retuned.
            return Ok(());
        }
        nwk.cancel_channel_change();
        nwk.nlme_set_channel(channel)
            .await
            .map_err(|status| OpError::Zigbee(status as u8))
    }

    fn groups_supported(&self) -> bool {
        true
    }

    fn group_membership(&mut self, op: u8, group: u16, endpoint: u8) -> Result<(), EndpointError> {
        // Host endpoints and the NCP-owned GP endpoint (H5: GP
        // Notifications groupcast to the sink group).
        if !(1..=240).contains(&endpoint) && endpoint != 242 || group > 0xFFF7 {
            return Err(EndpointError::InvalidParameter);
        }
        let groups = self.app.device_mut().aps_mut().group_table_mut();
        match op {
            1 if groups.add_group(group, endpoint) => Ok(()),
            1 => Err(EndpointError::NoSpace),
            2 => {
                groups.remove_group(group, endpoint);
                Ok(())
            }
            3 => {
                groups.remove_all_groups(endpoint);
                Ok(())
            }
            _ => Err(EndpointError::InvalidParameter),
        }
    }

    fn gp(&mut self) -> Option<&mut dyn crate::gp::GpPort> {
        Some(self)
    }

    async fn install_network(
        &mut self,
        image: &NetworkImage,
        children: &[RestoredChild],
    ) -> Result<(), OpError> {
        if self.running {
            return Err(OpError::InvalidState);
        }
        if !(11..=26).contains(&image.channel)
            || matches!(image.pan_id, 0x0000 | 0xFFFF)
            || image.network_key == [0; 16]
            || image.ext_pan_id == [0; 8]
            || image.ext_pan_id == [0xFF; 8]
        {
            return Err(OpError::InvalidParameter);
        }
        // Outgoing counters already used by this NCP: the network installed
        // next may reuse the same keys (a restore of its own backup), so it
        // starts above them whatever the image says.
        self.retain_counter_bounds()?;
        if self.stored.is_some() {
            // The previous network's NWK, child, APS and Trust Center state.
            self.app.wipe().await.map_err(|_| OpError::Storage)?;
            host_passthrough::clear();
            self.stored = None;
        }
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = image.ext_pan_id;
        state.pan_id = image.pan_id;
        state.short_address = 0x0000;
        state.ieee_address = self.ieee;
        state.channel = image.channel;
        state.depth = 0;
        state.parent_address = 0xFFFF;
        state.update_id = image.nwk_update_id;
        state.update_id_valid = true;
        state.network_key = image.network_key;
        state.key_sequence = image.key_seq;
        // The resume reserves `[limit, limit + stride)` from here (a
        // commissioned record never holds 0).
        state.global_counter_limit = image.nwk_fc_start.max(self.retained_nwk_bound).max(1);
        state.validate().map_err(|_| OpError::InvalidParameter)?;
        {
            let (_, store) = self.app.device_and_store_mut();
            store.store(&state).map_err(|_| OpError::Storage)?;
        }
        self.stored = stored_network(&mut self.app);
        if !children.is_empty() {
            // Written before the resume, which restores them like after a
            // reboot (same short address, no rejoin).
            store_child_table(&mut self.app, image.ext_pan_id, children)?;
        }
        let attributes = self.app.device_mut().bdb_mut().attributes_mut();
        if image.channel_mask != 0 {
            attributes.primary_channel_set = zigbee_types::ChannelMask(image.channel_mask);
        }
        self.app.rearm_initialize();
        self.app
            .initialize()
            .await
            .map_err(|_| OpError::Zigbee(0x01))?;
        if !self.app.device().is_joined() {
            return Err(OpError::Zigbee(0x01));
        }
        self.running = true;
        let aps_floor = image.aps_fc_start.max(self.retained_aps_bound);
        if aps_floor != 0 {
            self.app.raise_aps_floor(aps_floor)?;
        }
        let device = self.app.device_mut();
        device.aps_mut().nwk_mut().nib_mut().nwk_manager_addr = ShortAddress(image.nwk_manager);
        if let Some(dbm) = image.tx_power {
            let _ = device
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .mlme_set(
                    zigbee_mac::PibAttribute::PhyTransmitPower,
                    zigbee_mac::PibValue::I8(dbm),
                )
                .await;
        }
        device
            .permit_joining(0)
            .await
            .map_err(|status| OpError::Zigbee(status as u8))?;
        Ok(())
    }

    fn tc_link_key(&self) -> Option<[u8; 16]> {
        Some(*self.app.device().aps().security().default_tc_link_key())
    }

    fn device_capacity(&self) -> usize {
        zigbee_bdb::trust_center::MAX_TRUST_CENTER_DEVICES
    }

    fn device(&self, index: usize) -> Option<DeviceView> {
        let table = self.app.tc_table();
        let nwk = self.app.device().aps().nwk();
        let child_of = |ieee: &[u8; 8]| {
            nwk.neighbor_table()
                .children()
                .find(|entry| entry.ieee_address == *ieee)
                .map(|entry| (child_capability(entry), entry.end_device_timeout))
        };
        let known = table.devices().filter(|device| device.active).count();
        if index < known {
            let device = table.devices().filter(|device| device.active).nth(index)?;
            let security = self.app.device().aps().security();
            let rx_floor = security
                .find_key(
                    &device.ieee_address,
                    zigbee_aps::security::ApsKeyType::TrustCenterLinkKey,
                )
                .filter(|entry| entry.incoming_frame_counter_valid)
                .map_or(0, |entry| entry.incoming_frame_counter);
            let link = (device.key_origin
                != zigbee_bdb::trust_center::TrustCenterKeyOrigin::DefaultGlobal)
                .then_some(LinkView {
                    key: device.link_key,
                    attrs: device.key_attributes as u8,
                    origin: match device.key_origin {
                        zigbee_bdb::trust_center::TrustCenterKeyOrigin::DefaultGlobal => 0,
                        zigbee_bdb::trust_center::TrustCenterKeyOrigin::InstallCode => 1,
                        zigbee_bdb::trust_center::TrustCenterKeyOrigin::GeneratedUnique => 2,
                        zigbee_bdb::trust_center::TrustCenterKeyOrigin::ApplicationProvisioned => 3,
                    },
                    rx_floor,
                });
            // A provisioned key has no address yet; a child's comes from the
            // neighbor table.
            let nwk = (device.short_address.0 < 0xFFF8)
                .then_some(device.short_address.0)
                .or_else(|| {
                    nwk.neighbor_table()
                        .children()
                        .find(|entry| entry.ieee_address == device.ieee_address)
                        .map(|entry| entry.network_address.0)
                });
            return Some(DeviceView {
                ieee: device.ieee_address,
                nwk,
                child: child_of(&device.ieee_address),
                link,
            });
        }
        // Children without a Trust Center record.
        let entry = nwk
            .neighbor_table()
            .children()
            .filter(|entry| table.device(&entry.ieee_address).is_none_or(|d| !d.active))
            .nth(index - known)?;
        Some(DeviceView {
            ieee: entry.ieee_address,
            nwk: Some(entry.network_address.0),
            child: Some((child_capability(entry), entry.end_device_timeout)),
            link: None,
        })
    }

    fn put_link_key(&mut self, ieee: [u8; 8], key: [u8; 16]) -> Result<(), OpError> {
        if !self.running {
            return Err(OpError::InvalidState);
        }
        self.app.provision_link_key(ieee, key)
    }

    fn install_codes_supported(&self) -> bool {
        true
    }

    fn add_install_code(&mut self, ieee: [u8; 8], code: &[u8]) -> Result<[u8; 16], OpError> {
        if !self.running {
            return Err(OpError::InvalidState);
        }
        self.app.provision_install_code(ieee, code)
    }

    fn forget_device(&mut self, ieee: [u8; 8]) -> Result<bool, OpError> {
        if !self.running {
            return Err(OpError::InvalidState);
        }
        self.app.forget_device(ieee)
    }

    fn restore_link_key(&mut self, key: &RestoredKey) -> Result<(), OpError> {
        use zigbee_bdb::trust_center::{TrustCenterKeyAttributes, TrustCenterKeyOrigin};
        if !self.running {
            return Err(OpError::InvalidState);
        }
        let origin = match key.origin {
            1 => TrustCenterKeyOrigin::InstallCode,
            2 => TrustCenterKeyOrigin::GeneratedUnique,
            3 => TrustCenterKeyOrigin::ApplicationProvisioned,
            _ => return Err(OpError::InvalidParameter),
        };
        let attributes = match key.attrs {
            0 => TrustCenterKeyAttributes::Provisional,
            1 => TrustCenterKeyAttributes::Unverified,
            2 => TrustCenterKeyAttributes::Verified,
            _ => return Err(OpError::InvalidParameter),
        };
        // The parent of a non-child is not in the backup: its location
        // stays unknown until it is heard (its next Update-Device).
        let location = key.nwk.filter(|_| key.child).map(|nwk| (self.ieee, nwk));
        self.app
            .restore_link_key(key.ieee, key.key, origin, attributes, location)
    }

    async fn set_ieee(&mut self, ieee: [u8; 8]) -> Result<(), OpError> {
        if self.running {
            return Err(OpError::InvalidState);
        }
        self.app
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .mlme_set(
                zigbee_mac::PibAttribute::MacExtendedAddress,
                zigbee_mac::PibValue::ExtendedAddress(ieee),
            )
            .await
            .map_err(|_| OpError::InvalidParameter)?;
        // The resume refuses a NIB IEEE that differs from the MAC's.
        let device = self.app.device_mut();
        device.aps_mut().nwk_mut().nib_mut().ieee_address = ieee;
        device.bdb_mut().zdo_mut().set_local_ieee_addr(ieee);
        self.ieee = ieee;
        Ok(())
    }

    fn table_epoch(&mut self) -> u32 {
        let (_, store) = self.app.device_and_store_mut();
        let Ok(Some(state)) = store.load() else {
            return 0;
        };
        if !state.commissioned {
            return 0;
        }
        // Identity of the network in NV: a wipe or a restore of another
        // network changes it; a reboot does not.
        let mut crc = zigbee_zrhp::crc::Crc32c::new();
        crc.update(&state.extended_pan_id);
        crc.update(&state.pan_id.to_le_bytes());
        crc.update(&state.network_key);
        crc.update(&[state.key_sequence, state.channel]);
        crc.update(&state.ieee_address);
        crc.finish().max(1)
    }
}

/// MAC capability byte of a child neighbor: allocate address, device type
/// and receiver-on-when-idle (the power source is approximated by it).
fn child_capability(entry: &zigbee_nwk::neighbor::NeighborEntry) -> u8 {
    let mut capability = 0x80;
    if entry.device_type == NeighborDeviceType::Router {
        capability |= 0x02;
    }
    if entry.rx_on_when_idle {
        capability |= 0x0C;
    }
    if entry.security_capable {
        capability |= 0x40;
    }
    capability
}

impl<A: TcApp> TcPort<A> {
    /// `NWK_FORM` with host-supplied identity (spec 7.2): fill what the host
    /// left to the NCP (PAN ID, EPID, key, channel by ED scan), then install
    /// the network like a restore.
    async fn form_with_identity(&mut self, params: &FormParams) -> Result<(), OpError> {
        let mut random = [0u8; 18];
        self.app
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .fill_random(&mut random)
            .map_err(|_| OpError::Zigbee(0x01))?;
        let pan_id = if params.pan_id == 0xFFFF {
            u16::from_le_bytes([random[0], random[1]]) & 0x3FFF | 0x0001
        } else {
            params.pan_id
        };
        let ext_pan_id = if params.ext_pan_id == [0; 8] {
            self.ieee
        } else {
            params.ext_pan_id
        };
        let mut network_key = params.network_key;
        if network_key == [0; 16] {
            network_key.copy_from_slice(&random[2..18]);
        }
        let mask = if params.channel_mask == 0 {
            0x07FF_F800
        } else {
            params.channel_mask & 0x07FF_F800
        };
        let channel = self.quietest_channel(mask).await?;
        let image = NetworkImage {
            pan_id,
            ext_pan_id,
            channel,
            channel_mask: mask,
            nwk_update_id: params.nwk_update_id,
            nwk_manager: params.nwk_manager,
            tx_power: (params.tx_power != 0).then_some(params.tx_power),
            network_key,
            key_seq: params.key_seq,
            nwk_fc_start: 0,
            aps_fc_start: 0,
        };
        self.install_network(&image, &[]).await
    }

    /// The channel of `mask` with the lowest energy (a single channel is
    /// taken as is).
    async fn quietest_channel(&mut self, mask: u32) -> Result<u8, OpError> {
        if mask == 0 {
            return Err(OpError::InvalidParameter);
        }
        if mask.is_power_of_two() {
            return Ok(mask.trailing_zeros() as u8);
        }
        let scan = self
            .app
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .nlme_ed_scan(zigbee_types::ChannelMask(mask), 3)
            .await
            .map_err(|status| OpError::Zigbee(status as u8))?;
        scan.energy_list
            .iter()
            .min_by_key(|ed| ed.energy)
            .map(|ed| ed.channel)
            .ok_or(OpError::Zigbee(0x01))
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

    fn rearm_initialize(&mut self) -> bool {
        (**self).rearm_initialize()
    }

    fn pre_erase_step(&mut self) -> bool {
        (**self).pre_erase_step()
    }

    fn compact_journal(&mut self) -> bool {
        (**self).compact_journal()
    }

    fn store_children(
        &mut self,
        table: &zigbee_runtime::child_store::PersistentChildTable,
    ) -> Result<(), OpError> {
        (**self).store_children(table)
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

    fn tc_table(&self) -> &zigbee_bdb::trust_center::TrustCenterTable {
        (**self).tc_table()
    }

    fn provision_link_key(&mut self, ieee: [u8; 8], key: [u8; 16]) -> Result<(), OpError> {
        (**self).provision_link_key(ieee, key)
    }

    fn provision_install_code(&mut self, ieee: [u8; 8], code: &[u8]) -> Result<[u8; 16], OpError> {
        (**self).provision_install_code(ieee, code)
    }

    fn forget_device(&mut self, ieee: [u8; 8]) -> Result<bool, OpError> {
        (**self).forget_device(ieee)
    }

    fn restore_link_key(
        &mut self,
        ieee: [u8; 8],
        key: [u8; 16],
        origin: zigbee_bdb::trust_center::TrustCenterKeyOrigin,
        attributes: zigbee_bdb::trust_center::TrustCenterKeyAttributes,
        location: Option<([u8; 8], u16)>,
    ) -> Result<(), OpError> {
        (**self).restore_link_key(ieee, key, origin, attributes, location)
    }

    fn aps_counter_bound(&mut self) -> Result<u32, OpError> {
        (**self).aps_counter_bound()
    }
}

impl<A: TcApp> TcPort<A> {
    /// Folds the outgoing counter bounds of the network in NV into the
    /// session bounds: the durable NWK reservation (kept by a wipe as a
    /// factory-new tombstone) and the global APS counter. Its own frame: the
    /// security record stays out of the async callers' poll frames.
    #[inline(never)]
    fn retain_counter_bounds(&mut self) -> Result<(), OpError> {
        let nwk = {
            let (_, store) = self.app.device_and_store_mut();
            store
                .load()
                .map_err(|_| OpError::Storage)?
                .map_or(0, |state| state.global_counter_limit)
        };
        let aps = self.app.aps_counter_bound()?;
        self.retained_nwk_bound = self.retained_nwk_bound.max(nwk);
        self.retained_aps_bound = self.retained_aps_bound.max(aps);
        Ok(())
    }
}

/// The durable child table of a restored network. Its own frame: the
/// 64-entry table stays out of the async callers' poll frames.
#[inline(never)]
fn store_child_table<A: TcApp>(
    app: &mut A,
    ext_pan_id: [u8; 8],
    children: &[RestoredChild],
) -> Result<(), OpError> {
    let mut table = zigbee_runtime::child_store::PersistentChildTable::new(ext_pan_id);
    for child in children {
        let entry = zigbee_runtime::child_store::PersistentChild {
            ieee_address: child.ieee,
            short_address: child.nwk,
            rx_on_when_idle: child.capability & 0x08 != 0,
            security_capable: child.capability & 0x40 != 0,
            is_router: child.capability & 0x02 != 0,
            end_device_timeout: child.timeout,
            removal_pending: false,
            removal_attempts: 0,
            reassignment_address: None,
            departure_pending: false,
        };
        table.push(entry).map_err(|_| OpError::InvalidParameter)?;
    }
    table.validate().map_err(|_| OpError::InvalidParameter)?;
    app.store_children(&table)
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
