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

use heapless::Deque;
use router_app::{
    ApsTableLifecycle, Diagnostics, RouterAppError, RouterObserver, StatusSink, StepEvents,
    Supervisor, TrustCenterCoordinatorApp,
};
use zigbee_aps::ApsTxOptions;
use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_aps::{ApsAddress, ApsAddressMode};
use zigbee_mac::ParentMacDriver;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::child_store::ChildTableStore;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::host_passthrough;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::role::Router;
use zigbee_runtime::security_store::SecurityStateStore;
use zigbee_runtime::trust_center_runtime::GlobalApsCheckpoint;
use zigbee_runtime::trust_center_store::TrustCenterDeviceStore;
use zigbee_types::ShortAddress;

use crate::port::{
    CoordinatorPort, DataConfirm, DataRequest, Destination, DeviceEvent, FormParams, FrameCounters,
    Indication, NetworkInfo, NetworkState, OpError,
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
}

/// The NCP's view of the Trust Center coordinator.
pub struct TcPort<A: TcApp> {
    app: A,
    running: bool,
    formed: bool,
    events: Deque<DeviceEvent, 8>,
    clock_ms: fn() -> u32,
    ieee: [u8; 8],
}

impl<A: TcApp> TcPort<A> {
    /// `ieee`: the radio's EUI-64 (the NIB only learns it at start);
    /// `clock_ms`: monotonic milliseconds for indication timestamps.
    pub fn new(mut app: A, ieee: [u8; 8], clock_ms: fn() -> u32) -> Self {
        let formed = commissioned(&mut app);
        crate::gp::reset_defaults(&mut app);
        Self {
            app,
            running: false,
            formed,
            events: Deque::new(),
            clock_ms,
            ieee,
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

    /// One bounded stack step; collects device events for the host.
    pub async fn step(&mut self) -> Result<(), RouterAppError> {
        if !self.running {
            return Err(RouterAppError::NotInitialized);
        }
        let events = self.app.step().await?;
        for event in events.iter() {
            if let StackEvent::DeviceAnnounced {
                address,
                short_address,
                capabilities,
            } = event
            {
                let joined = DeviceEvent::Joined {
                    nwk: short_address.0,
                    ieee: *address,
                    parent_nwk: 0xFFFF,
                    join_type: 0,
                    capability: *capabilities,
                };
                if self.events.push_back(joined).is_err() {
                    log::warn!("[NCP] device event queue full");
                }
            }
        }
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
            aps_secured: false,
            nwk_secured: record.nwk_secured,
            handled_by_ncp: record.handled_by_ncp,
            lqi: record.lqi,
            rssi: -128,
            last_hop_nwk: record.last_hop_nwk,
            aps_counter: record.aps_counter,
            rx_time_ms: (self.clock_ms)(),
            asdu,
        })
    }

    fn take_device_event(&mut self) -> Option<DeviceEvent> {
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
        match self
            .app
            .device_mut()
            .aps_mut()
            .apsde_data_request(&aps_request)
            .await
        {
            Ok(confirm) => Ok(DataConfirm {
                zb_status: confirm.status as u8,
                aps_counter: confirm.aps_counter,
                dst_nwk,
                // The APS ACK outcome is not surfaced by the APS layer yet:
                // the confirm reports the NWK/MAC transmission only.
                flags: 0,
            }),
            Err(status) => Err(OpError::Zigbee(status as u8)),
        }
    }

    async fn leave_reset(&mut self) -> Result<(), OpError> {
        self.app.wipe().await.map_err(|_| OpError::Storage)?;
        host_passthrough::clear();
        self.running = false;
        self.formed = false;
        Ok(())
    }

    fn gp(&mut self) -> Option<&mut dyn crate::gp::GpPort> {
        Some(self)
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
}
