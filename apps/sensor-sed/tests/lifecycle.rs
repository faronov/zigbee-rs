use core::future::Future;
use core::task::{Context, Poll};
use std::cell::{Cell, RefCell};
use std::sync::Arc;
use std::task::{Wake, Waker};

use sensor_sed_app::SensorLifecycleError;
use sensor_sed_app::{
    BatterySource, ButtonPolicy, DiagnosticEvent, Diagnostics, EnvironmentReading,
    EnvironmentSource, NoBattery, NoMeasurement, NoOta, NoStatus, NoUserAction,
    OtaActivationOutcome, OtaEventOutcome, OtaLifecycle, OtaServiceOutcome, SampleTrigger,
    SensorApp, SensorMeasurement, SensorPolicy, SensorSedParts, SensorStatus, SleepDepth,
    StatusPolicy, StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use zigbee_aps::frames::ApsCommandId;
use zigbee_aps::frames::{ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};
use zigbee_aps::security::{ApsSecurity, ApsSecurityHeader, KEY_ID_DATA_KEY, SEC_LEVEL_ENC_MIC_32};
use zigbee_aps::security::{KEY_ID_KEY_TRANSPORT, derive_key_transport_key};
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::{AssociationStatus, MlmeAssociateConfirm};
use zigbee_mac::primitives::{MacFrame, PanDescriptor, SuperframeSpec, ZigbeeBeaconPayload};
use zigbee_mac::{MacDriver, PlatformServices};
use zigbee_nwk::frames::{LeaveCommand, NwkCommandId, NwkFrameControl, NwkFrameType, NwkHeader};
use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};
use zigbee_runtime::ClusterRef;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::builder::EndpointBuilder;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{
    ApplicationClusters, BatteryMeasurement, ExpectedReportClusters, ProfileComponent,
    ProfileError, TemperatureHumidityMeasurement,
};
use zigbee_runtime::profile::{
    ApplicationProfile, BatteryDescriptor, DeviceProfile, EnvironmentalReporting, PlantSensor,
    PlantSensorReporting, RangeExtender, TemperatureHumidityBattery, TemperatureRange,
};
use zigbee_runtime::role::EndDevice;
use zigbee_runtime::security_store::{
    PersistentSecurityState, RamSecurityStateStore, SecurityStateStore, SecurityStoreError,
};
use zigbee_types::{MacAddress, PanId, ShortAddress};
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::poll_control::{PollControlCluster, PollControlConfig};
use zigbee_zcl::frame::{ZclFrameHeader, ZclFrameType};
use zigbee_zcl::{ClusterDirection, ClusterId, DeviceId};

const LOCAL_IEEE: [u8; 8] = [0x11; 8];
const COORDINATOR_IEEE: [u8; 8] = [0xCC; 8];
const NETWORK_KEY: [u8; 16] = [0xA5; 16];
const TRUST_CENTER_LINK_KEY: [u8; 16] = [0x5A; 16];
const EXTENDED_PAN_ID: [u8; 8] = [0xBB; 8];
const PAN_ID: PanId = PanId(0x1234);
const CHANNEL: u8 = 15;
const OLD_SHORT: ShortAddress = ShortAddress(0x5678);
const NEW_SHORT: ShortAddress = ShortAddress(0x6789);
const COORDINATOR: ShortAddress = ShortAddress::COORDINATOR;

const BASE_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 60_000,
    fast_poll_ms: 10,
    slow_poll_ms: 1_000,
    fresh_join_fast_ms: 100,
    restored_fast_ms: 100,
    wake_duration_ms: 1,
    join_retry_ms: 100,
    announce_retry_ms: 100,
    announce_retries: 0,
    secure_rejoin_failure_limit: 2,
    interview_complete_grace_ms: 5,
    button: ButtonPolicy {
        long_press_ms: None,
        debounce_ms: 1,
    },
    status: StatusPolicy {
        unjoined_blink_period_ms: 50,
        blink_on_ms: 1,
        blink_gap_ms: 1,
        reset_blinks: 1,
        reset_phase_ms: 1,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Retention,
    recovery: DETERMINISTIC_RECOVERY,
};

/// Deterministic recovery for scripted tests: no jitter, the largest
/// backoff cap, and no parent-loss heuristics.
const DETERMINISTIC_RECOVERY: sensor_sed_app::RecoveryPolicy = sensor_sed_app::RecoveryPolicy {
    join_retry_max_ms: sensor_sed_app::JOIN_RETRY_MAX_MS,
    join_retry_jitter_pct: 0,
    parent_loss_strikes: 0,
    max_active_ms: 0,
    report_aps_ack: false,
    max_ota_awake_ms: 0,
    max_identify_awake_ms: 0,
    max_fast_poll_ms: 0,
    missed_report_grace_ms: 0,
    sleep_failure_limit: 0,
};

const NO_STATUS_POLICY: SensorPolicy = SensorPolicy {
    status: StatusPolicy {
        unjoined_blink_period_ms: 0,
        blink_on_ms: 0,
        blink_gap_ms: 0,
        reset_blinks: 0,
        reset_phase_ms: 0,
    },
    ..BASE_POLICY
};

type TestProfile = DeviceProfile<TemperatureHumidityBattery>;

struct NoopWake;

impl Wake for NoopWake {
    fn wake(self: Arc<Self>) {}
}

fn block_on<F: Future>(future: F) -> F::Output {
    let waker = Waker::from(Arc::new(NoopWake));
    let mut context = Context::from_waker(&waker);
    let mut future = std::pin::pin!(future);
    loop {
        match future.as_mut().poll(&mut context) {
            Poll::Ready(output) => return output,
            Poll::Pending => std::thread::yield_now(),
        }
    }
}

struct RecordingStatus<'a> {
    events: &'a RefCell<Vec<SensorStatus>>,
}

impl StatusSink for RecordingStatus<'_> {
    fn set(&mut self, status: SensorStatus) {
        self.events.borrow_mut().push(status);
    }
}

struct RecordingDiagnostics<'a> {
    events: &'a RefCell<Vec<DiagnosticEvent>>,
}

impl Diagnostics for RecordingDiagnostics<'_> {
    fn record(&mut self, event: DiagnosticEvent) {
        self.events.borrow_mut().push(event);
    }
}

/// Recording wake controller whose millisecond marks wrap at `MODULUS`, so
/// tests can model a narrow platform counter.
struct RecordingWake<'a, const MODULUS: u64 = { 1 << 32 }> {
    now: u32,
    activity_once: bool,
    /// Wait index → non-timer wake reason returned without elapsed time.
    reasons: Vec<(usize, WakeReason)>,
    batches: Vec<Vec<MacFrame>>,
    next_batch: usize,
    waits: &'a RefCell<Vec<WaitRequest>>,
    poll_counts_at_wait: &'a RefCell<Vec<u32>>,
    delay_calls: &'a Cell<u32>,
    /// Non-active waits that still fail (without elapsed time) before
    /// sleeping works again, and the error they return.
    failing_sleeps: Option<&'a Cell<u32>>,
    failure: WaitFailure,
}

/// Scripted [`RecordingWake`] error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct WaitFailure {
    recoverable: bool,
    code: u32,
}

impl<'a, const MODULUS: u64> RecordingWake<'a, MODULUS> {
    fn new(
        now: u32,
        batches: Vec<Vec<MacFrame>>,
        waits: &'a RefCell<Vec<WaitRequest>>,
        poll_counts_at_wait: &'a RefCell<Vec<u32>>,
        delay_calls: &'a Cell<u32>,
    ) -> Self {
        Self {
            now: (u64::from(now) % MODULUS) as u32,
            activity_once: false,
            reasons: Vec::new(),
            batches,
            next_batch: 0,
            waits,
            poll_counts_at_wait,
            delay_calls,
            failing_sleeps: None,
            failure: WaitFailure {
                recoverable: true,
                code: 0,
            },
        }
    }
}

impl<const MODULUS: u64> WakeController<MockMac> for RecordingWake<'_, MODULUS> {
    type Mark = u32;
    type Error = WaitFailure;

    fn is_recoverable(error: &WaitFailure) -> bool {
        error.recoverable
    }

    fn error_code(error: &WaitFailure) -> u32 {
        error.code
    }

    fn mark(&self) -> Self::Mark {
        self.now
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        ((u64::from(mark) + u64::from(duration_ms)) % MODULUS) as u32
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        ((u64::from(later) + MODULUS - u64::from(earlier)) % MODULUS) as u32
    }

    async fn wait(
        &mut self,
        mac: &mut MockMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        let index = self.waits.borrow().len();
        self.waits.borrow_mut().push(request);
        self.poll_counts_at_wait.borrow_mut().push(mac.poll_count());
        if let Some(failing) = self.failing_sleeps
            && request.sleep_depth != SleepDepth::Active
            && failing.get() > 0
        {
            failing.set(failing.get() - 1);
            return Err(self.failure);
        }
        let scripted = self
            .reasons
            .iter()
            .find(|(at, _)| *at == index)
            .map(|(_, reason)| *reason);
        if self.next_batch < self.batches.len() {
            let batch = core::mem::take(&mut self.batches[self.next_batch]);
            for frame in batch {
                mac.enqueue_poll_response(frame);
            }
        }
        self.next_batch = self.next_batch.saturating_add(1);
        let activity = core::mem::take(&mut self.activity_once);
        let reason = match scripted {
            Some(reason) => reason,
            None if activity => WakeReason::Activity,
            None => WakeReason::Timer,
        };
        let elapsed = if reason == WakeReason::Timer {
            request.timeout_ms
        } else {
            0
        };
        self.now = Self::add_ms(self.now, elapsed);
        mac.delay_micros(elapsed.saturating_mul(1_000)).await;
        Ok(reason)
    }

    async fn button_held_for(&mut self, _duration_ms: u32) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        self.delay_calls
            .set(self.delay_calls.get().saturating_add(1));
        self.now = Self::add_ms(self.now, duration_ms);
    }
}

struct CountingEnvironment<'a> {
    samples: &'a Cell<u32>,
}

impl EnvironmentSource for CountingEnvironment<'_> {
    type Error = ();

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        self.samples.set(self.samples.get().saturating_add(1));
        Ok(EnvironmentReading {
            temperature_centi_celsius: 2_125,
            humidity_centi_percent: 5_050,
            pressure_tenth_kpa: None,
        })
    }
}

struct CountingBattery<'a> {
    samples: &'a Cell<u32>,
}

impl BatterySource for CountingBattery<'_> {
    type Error = ();

    async fn sample(&mut self) -> Result<Option<sensor_sed_app::BatteryReading>, Self::Error> {
        self.samples.set(self.samples.get().saturating_add(1));
        Ok(None)
    }
}

struct RecordingSupervisor<'a> {
    heartbeats: &'a Cell<u32>,
}

impl Supervisor for RecordingSupervisor<'_> {
    fn heartbeat(&mut self) {
        self.heartbeats.set(self.heartbeats.get().saturating_add(1));
    }

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        panic!("unexpected reset")
    }

    fn reset_with_cause(&mut self, cause: sensor_sed_app::ResetCause) -> ! {
        panic!("reset with cause {:#010x}", cause.word())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CheckpointFailure {
    Load,
    Store,
}

struct CountingStore<'a> {
    inner: RamSecurityStateStore,
    store_calls: &'a Cell<u32>,
    load_calls: Option<&'a Cell<u32>>,
    checkpoint_failure: Option<&'a Cell<Option<CheckpointFailure>>>,
}

impl<'a> CountingStore<'a> {
    fn empty(store_calls: &'a Cell<u32>) -> Self {
        Self {
            inner: RamSecurityStateStore::new(),
            store_calls,
            load_calls: None,
            checkpoint_failure: None,
        }
    }

    fn with_state(state: PersistentSecurityState, store_calls: &'a Cell<u32>) -> Self {
        let mut inner = RamSecurityStateStore::new();
        inner.store(&state).expect("seed security state");
        Self {
            inner,
            store_calls,
            load_calls: None,
            checkpoint_failure: None,
        }
    }

    fn with_state_and_loads(
        state: PersistentSecurityState,
        store_calls: &'a Cell<u32>,
        load_calls: &'a Cell<u32>,
    ) -> Self {
        let mut store = Self::with_state(state, store_calls);
        store.load_calls = Some(load_calls);
        store
    }
}

impl SecurityStateStore for CountingStore<'_> {
    fn load(&mut self) -> Result<Option<PersistentSecurityState>, SecurityStoreError> {
        if let Some(load_calls) = self.load_calls {
            load_calls.set(load_calls.get().saturating_add(1));
        }
        if let Some(failure) = self.checkpoint_failure
            && failure.get() == Some(CheckpointFailure::Load)
        {
            failure.set(None);
            return Err(SecurityStoreError::Hardware);
        }
        self.inner.load()
    }

    fn store(&mut self, state: &PersistentSecurityState) -> Result<(), SecurityStoreError> {
        self.store_calls
            .set(self.store_calls.get().saturating_add(1));
        if let Some(failure) = self.checkpoint_failure
            && failure.get() == Some(CheckpointFailure::Store)
        {
            failure.set(None);
            return Err(SecurityStoreError::Hardware);
        }
        self.inner.store(state)
    }

    fn visit_replay_counters(
        &mut self,
        visitor: &mut dyn FnMut(zigbee_runtime::security_store::PersistentReplayCounter),
    ) -> Result<(), SecurityStoreError> {
        self.inner.visit_replay_counters(visitor)
    }

    fn commit_replay_counter(
        &mut self,
        replay: zigbee_runtime::security_store::PersistentReplayCounter,
    ) -> Result<(), SecurityStoreError> {
        self.inner.commit_replay_counter(replay)
    }
}

struct RecordingOta<'a> {
    keep_awake_ms: u32,
    seen_raw_command: &'a Cell<bool>,
    activated: &'a Cell<u32>,
    loads_at_handle: &'a Cell<u32>,
    checkpoint_before_activation: &'a Cell<bool>,
    load_calls: &'a Cell<u32>,
}

impl<M, S, P> OtaLifecycle<M, S, P> for RecordingOta<'_>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
{
    const ENABLED: bool = true;

    fn is_active(&self, _profile: &P) -> bool {
        false
    }

    fn next_deadline_ms(&self, _profile: &P) -> Option<u32> {
        None
    }

    async fn handle_event(
        &mut self,
        _node: &mut ZigbeeNode<'_, M, S, P>,
        event: &StackEvent,
    ) -> OtaEventOutcome {
        if matches!(
            event,
            StackEvent::CommandReceived {
                cluster_id,
                command_id: 0x77,
                ..
            } if *cluster_id == ClusterId::OTA_UPGRADE.0
        ) {
            self.seen_raw_command.set(true);
            self.loads_at_handle.set(self.load_calls.get());
            OtaEventOutcome::Handled {
                keep_awake_ms: Some(self.keep_awake_ms),
                activation_pending: true,
            }
        } else {
            OtaEventOutcome::NotHandled
        }
    }

    async fn service(
        &mut self,
        _node: &mut ZigbeeNode<'_, M, S, P>,
        _elapsed_secs: u16,
    ) -> OtaServiceOutcome {
        OtaServiceOutcome::IDLE
    }

    fn activate(&mut self, _node: &mut ZigbeeNode<'_, M, S, P>) -> OtaActivationOutcome {
        self.checkpoint_before_activation
            .set(self.load_calls.get() > self.loads_at_handle.get());
        self.activated.set(self.activated.get().saturating_add(1));
        OtaActivationOutcome::Activated
    }

    fn abort(&mut self, _node: &mut ZigbeeNode<'_, M, S, P>) {
        unreachable!("never active");
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ActivationTrigger {
    Event,
    Service,
}

struct CheckpointOta<'a> {
    trigger: ActivationTrigger,
    failure: Option<CheckpointFailure>,
    fault: &'a Cell<Option<CheckpointFailure>>,
    requested: &'a Cell<bool>,
    activated: &'a Cell<u32>,
    limit_before: &'a Cell<u32>,
}

impl CheckpointOta<'_> {
    fn request<M: MacDriver, S: SecurityStateStore, P: ApplicationProfile>(
        &self,
        node: &mut ZigbeeNode<'_, M, S, P>,
    ) {
        let nib = node.device_mut().bdb_mut().zdo_mut().nwk_mut().nib_mut();
        self.limit_before.set(nib.outgoing_frame_counter_limit);
        nib.outgoing_frame_counter = nib.outgoing_frame_counter_limit - 1;
        self.fault.set(self.failure);
        self.requested.set(true);
    }
}

impl<M: MacDriver, S: SecurityStateStore, P: ApplicationProfile> OtaLifecycle<M, S, P>
    for CheckpointOta<'_>
{
    const ENABLED: bool = true;

    fn is_active(&self, _profile: &P) -> bool {
        self.requested.get() && self.activated.get() == 0
    }

    fn next_deadline_ms(&self, _profile: &P) -> Option<u32> {
        None
    }

    async fn handle_event(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, P>,
        event: &StackEvent,
    ) -> OtaEventOutcome {
        if self.trigger == ActivationTrigger::Event
            && matches!(event, StackEvent::CommandReceived { cluster_id, command_id: 0x77, .. }
                if *cluster_id == ClusterId::OTA_UPGRADE.0)
        {
            self.request(node);
            OtaEventOutcome::Handled {
                keep_awake_ms: Some(250),
                activation_pending: true,
            }
        } else {
            OtaEventOutcome::NotHandled
        }
    }

    async fn service(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, P>,
        _elapsed_secs: u16,
    ) -> OtaServiceOutcome {
        if self.trigger == ActivationTrigger::Service && !self.requested.get() {
            self.request(node);
            OtaServiceOutcome {
                keep_awake_ms: Some(250),
                activation_pending: true,
            }
        } else {
            OtaServiceOutcome::IDLE
        }
    }

    fn activate(&mut self, node: &mut ZigbeeNode<'_, M, S, P>) -> OtaActivationOutcome {
        let persisted = node.load_security_state().unwrap().unwrap();
        assert!(persisted.global_counter_limit > self.limit_before.get());
        assert_eq!(
            persisted.global_counter_limit,
            node.device()
                .bdb()
                .zdo()
                .nwk()
                .nib()
                .outgoing_frame_counter_limit
        );
        self.activated.set(self.activated.get() + 1);
        OtaActivationOutcome::Activated
    }

    fn abort(&mut self, _node: &mut ZigbeeNode<'_, M, S, P>) {
        unreachable!("the OTA bound is disabled in these tests");
    }
}

fn test_profile() -> TestProfile {
    DeviceProfile::new(
        1,
        0x0104,
        DeviceId::TEMPERATURE_SENSOR,
        TemperatureHumidityBattery::new(
            TemperatureRange {
                min_centi_celsius: -4_000,
                max_centi_celsius: 12_500,
            },
            BatteryDescriptor {
                size: 4,
                quantity: 2,
                rated_voltage_100mv: 15,
            },
            EnvironmentalReporting::default(),
        ),
    )
}

fn test_device<P: ApplicationProfile>(
    profile: &P,
    policy: &SensorPolicy,
) -> ZigbeeDevice<MockMac, EndDevice> {
    let mut device = ZigbeeDevice::builder(MockMac::new(LOCAL_IEEE))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .power_mode(policy.power_mode())
        .automatic_polling(false)
        .build();
    device.bdb_mut().initialize().expect("BDB initialize");
    device
}

fn join_beacon(association_permit: bool) -> PanDescriptor {
    PanDescriptor {
        channel: CHANNEL,
        coord_address: MacAddress::Short(PAN_ID, COORDINATOR),
        superframe_spec: SuperframeSpec {
            association_permit,
            pan_coordinator: true,
            ..Default::default()
        },
        lqi: 220,
        security_use: false,
        zigbee_beacon: ZigbeeBeaconPayload {
            protocol_id: 0,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity: true,
            device_depth: 0,
            end_device_capacity: true,
            extended_pan_id: EXTENDED_PAN_ID,
            tx_offset: [0xFF; 3],
            update_id: 0,
        },
    }
}

fn transport_key_frame() -> MacFrame {
    let mut command = [0u8; 35];
    command[0] = ApsCommandId::TransportKey as u8;
    command[1] = 0x01;
    command[2..18].copy_from_slice(&NETWORK_KEY);
    command[18] = 0;
    command[19..27].copy_from_slice(&LOCAL_IEEE);
    command[27..35].copy_from_slice(&COORDINATOR_IEEE);

    let aps_security = ApsSecurity::new();
    let transport_key = derive_key_transport_key(aps_security.default_tc_link_key());
    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Command as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            security: true,
            ..Default::default()
        },
        aps_counter: 1,
        ..Default::default()
    };
    let security_header = ApsSecurityHeader {
        // Zigbee transmits security level zero and authenticates the actual
        // ENC-MIC-32 value.
        security_control: (KEY_ID_KEY_TRANSPORT << 3) | (1 << 5),
        frame_counter: 1,
        source_address: Some(COORDINATOR_IEEE),
        key_seq_number: None,
    };
    let mut aps = [0u8; 96];
    let aps_header_len = aps_header.serialize(&mut aps);
    let security_header_len = security_header.serialize(&mut aps[aps_header_len..]);
    let aad_len = aps_header_len + security_header_len;
    let mut authenticated_header = [0u8; 16];
    authenticated_header[..aad_len].copy_from_slice(&aps[..aad_len]);
    authenticated_header[aps_header_len] |= SEC_LEVEL_ENC_MIC_32;
    let encrypted = aps_security
        .encrypt(
            &authenticated_header[..aad_len],
            &command,
            &transport_key,
            &security_header,
        )
        .expect("Transport-Key fixture encrypts");
    aps[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    let aps_len = aad_len + encrypted.len();

    let header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            ..Default::default()
        },
        dst_addr: OLD_SHORT,
        src_addr: COORDINATOR,
        radius: 30,
        seq_number: 1,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    let mut bytes = [0u8; 128];
    let header_len = header.serialize(&mut bytes);
    bytes[header_len..header_len + aps_len].copy_from_slice(&aps[..aps_len]);
    MacFrame::from_slice(&bytes[..header_len + aps_len]).expect("Transport-Key frame")
}

fn script_cold_join(device: &mut ZigbeeDevice<MockMac, EndDevice>) {
    {
        let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
        nwk.set_rx_on_when_idle(false);
        let mac = nwk.mac_mut();
        mac.add_beacon(join_beacon(true));
        mac.set_associate_response(MlmeAssociateConfirm {
            short_address: OLD_SHORT,
            status: AssociationStatus::Success,
        });
        mac.enqueue_poll_response(transport_key_frame());
    }
}

fn commissioned_state(parent_information: u8) -> PersistentSecurityState {
    let mut state = PersistentSecurityState::empty();
    state.commissioned = true;
    state.extended_pan_id = EXTENDED_PAN_ID;
    state.pan_id = PAN_ID.0;
    state.short_address = OLD_SHORT.0;
    state.ieee_address = LOCAL_IEEE;
    state.channel = CHANNEL;
    state.depth = 1;
    state.parent_address = COORDINATOR.0;
    state.update_id = 0;
    state.update_id_valid = true;
    state.network_key = NETWORK_KEY;
    state.key_sequence = 0;
    state.global_counter_limit = 0x400;
    state.tclk_present = true;
    state.trust_center_address = COORDINATOR_IEEE;
    state.trust_center_link_key = TRUST_CENTER_LINK_KEY;
    state.tclk_counter_limit = 0x400;
    state.parent_information = parent_information;
    state.parent_information_valid = true;
    state.end_device_timeout = 8;
    state
}

#[allow(clippy::too_many_arguments)]
fn secured_nwk_frame(
    frame_type: NwkFrameType,
    dst_addr: ShortAddress,
    src_addr: ShortAddress,
    dst_ieee: Option<[u8; 8]>,
    src_ieee: Option<[u8; 8]>,
    frame_counter: u32,
    sequence: u8,
    plaintext: &[u8],
) -> MacFrame {
    let header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: frame_type as u8,
            protocol_version: 0x02,
            security: true,
            dst_ieee_present: dst_ieee.is_some(),
            src_ieee_present: src_ieee.is_some(),
            ..Default::default()
        },
        dst_addr,
        src_addr,
        radius: 30,
        seq_number: sequence,
        dst_ieee,
        src_ieee,
        multicast_control: None,
        source_route: None,
    };
    let security_header = NwkSecurityHeader {
        security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
        frame_counter,
        source_address: COORDINATOR_IEEE,
        key_seq_number: 0,
    };
    let mut bytes = [0u8; 127];
    let header_len = header.serialize(&mut bytes);
    let security_len = security_header.serialize(&mut bytes[header_len..]);
    let encrypted = NwkSecurity::new()
        .encrypt(
            &bytes[..header_len + security_len],
            plaintext,
            &NETWORK_KEY,
            &security_header,
        )
        .expect("encrypt fixture");
    let end = header_len + security_len + encrypted.len();
    bytes[header_len + security_len..end].copy_from_slice(&encrypted);
    // Zigbee transmits zero in the OTA security-level bits.
    bytes[header_len] &= !0x07;
    MacFrame::from_slice(&bytes[..end]).expect("secured frame")
}

fn zcl_data_frame(
    cluster_id: ClusterId,
    zcl_frame_control: u8,
    transaction: u8,
    command_id: u8,
    payload: &[u8],
    frame_counter: u32,
) -> MacFrame {
    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ..Default::default()
        },
        dst_endpoint: Some(1),
        cluster_id: Some(cluster_id.0),
        profile_id: Some(0x0104),
        src_endpoint: Some(1),
        aps_counter: transaction,
        ..Default::default()
    };
    let mut aps = [0u8; 96];
    let aps_len = aps_header.serialize(&mut aps);
    aps[aps_len] = zcl_frame_control;
    aps[aps_len + 1] = transaction;
    aps[aps_len + 2] = command_id;
    aps[aps_len + 3..aps_len + 3 + payload.len()].copy_from_slice(payload);
    secured_nwk_frame(
        NwkFrameType::Data,
        OLD_SHORT,
        COORDINATOR,
        None,
        None,
        frame_counter,
        transaction,
        &aps[..aps_len + 3 + payload.len()],
    )
}

fn configure_reporting_frame(
    cluster_id: ClusterId,
    attribute_id: u16,
    data_type: u8,
    reportable_change: &[u8],
    frame_counter: u32,
) -> MacFrame {
    let mut payload = [0u8; 16];
    payload[0] = ClusterDirection::ClientToServer as u8;
    payload[1..3].copy_from_slice(&attribute_id.to_le_bytes());
    payload[3] = data_type;
    payload[4..6].copy_from_slice(&1u16.to_le_bytes());
    payload[6..8].copy_from_slice(&60u16.to_le_bytes());
    payload[8..8 + reportable_change.len()].copy_from_slice(reportable_change);
    let frame_control = ZclFrameHeader::build_frame_control(
        ZclFrameType::Global,
        false,
        ClusterDirection::ClientToServer,
        false,
    );
    zcl_data_frame(
        cluster_id,
        frame_control,
        frame_counter as u8,
        0x06,
        &payload[..8 + reportable_change.len()],
        frame_counter,
    )
}

fn identify_query_frame(frame_counter: u32) -> MacFrame {
    let frame_control = ZclFrameHeader::build_frame_control(
        ZclFrameType::ClusterSpecific,
        false,
        ClusterDirection::ClientToServer,
        false,
    );
    zcl_data_frame(
        ClusterId::IDENTIFY,
        frame_control,
        frame_counter as u8,
        0x01,
        &[],
        frame_counter,
    )
}

fn basic_reset_request(frame_counter: u32) -> MacFrame {
    let frame_control = ZclFrameHeader::build_frame_control(
        ZclFrameType::ClusterSpecific,
        false,
        ClusterDirection::ClientToServer,
        false,
    );
    zcl_data_frame(
        ClusterId::BASIC,
        frame_control,
        frame_counter as u8,
        zigbee_zcl::clusters::basic::CMD_RESET_TO_FACTORY_DEFAULTS.0,
        &[],
        frame_counter,
    )
}

fn leave_request(rejoin: bool, frame_counter: u32) -> MacFrame {
    let leave = LeaveCommand {
        remove_children: false,
        request: true,
        rejoin,
    };
    secured_nwk_frame(
        NwkFrameType::Command,
        OLD_SHORT,
        COORDINATOR,
        None,
        None,
        frame_counter,
        frame_counter as u8,
        &[NwkCommandId::Leave as u8, leave.serialize()],
    )
}

fn secured_rejoin_response(frame_counter: u32) -> MacFrame {
    secured_nwk_frame(
        NwkFrameType::Command,
        OLD_SHORT,
        COORDINATOR,
        Some(LOCAL_IEEE),
        Some(COORDINATOR_IEEE),
        frame_counter,
        frame_counter as u8,
        &[
            NwkCommandId::RejoinResponse as u8,
            NEW_SHORT.0 as u8,
            (NEW_SHORT.0 >> 8) as u8,
            0,
        ],
    )
}

fn raw_ota_command(frame_counter: u32) -> MacFrame {
    let frame_control = ZclFrameHeader::build_frame_control(
        ZclFrameType::ClusterSpecific,
        false,
        ClusterDirection::ServerToClient,
        true,
    );
    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            security: true,
            ..Default::default()
        },
        dst_endpoint: Some(1),
        cluster_id: Some(ClusterId::OTA_UPGRADE.0),
        profile_id: Some(0x0104),
        src_endpoint: Some(1),
        aps_counter: frame_counter as u8,
        ..Default::default()
    };
    let security_header = ApsSecurityHeader {
        security_control: (KEY_ID_DATA_KEY << 3) | (1 << 5),
        frame_counter,
        source_address: Some(COORDINATOR_IEEE),
        key_seq_number: None,
    };
    let zcl = [frame_control, frame_counter as u8, 0x77, 0xAA];
    let mut aps = [0u8; 96];
    let aps_header_len = aps_header.serialize(&mut aps);
    let security_header_len = security_header.serialize(&mut aps[aps_header_len..]);
    let aad_len = aps_header_len + security_header_len;
    let mut authenticated_header = [0u8; 32];
    authenticated_header[..aad_len].copy_from_slice(&aps[..aad_len]);
    authenticated_header[aps_header_len] |= SEC_LEVEL_ENC_MIC_32;
    let encrypted = ApsSecurity::new()
        .encrypt(
            &authenticated_header[..aad_len],
            &zcl,
            &TRUST_CENTER_LINK_KEY,
            &security_header,
        )
        .expect("OTA APS fixture encrypts");
    aps[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    secured_nwk_frame(
        NwkFrameType::Data,
        OLD_SHORT,
        COORDINATOR,
        None,
        None,
        frame_counter,
        frame_counter as u8,
        &aps[..aad_len + encrypted.len()],
    )
}

fn invalid_poll_batch() -> Vec<MacFrame> {
    (0..8)
        .map(|_| MacFrame::from_slice(&[0]).expect("invalid frame fixture"))
        .collect()
}

fn parts<'a, O, St>(
    wake: RecordingWake<'a>,
    status: St,
    ota: O,
    environment_samples: &'a Cell<u32>,
    battery_samples: &'a Cell<u32>,
    heartbeats: &'a Cell<u32>,
    diagnostics: &'a RefCell<Vec<DiagnosticEvent>>,
) -> SensorSedParts<
    RecordingWake<'a>,
    St,
    CountingEnvironment<'a>,
    CountingBattery<'a>,
    O,
    NoUserAction,
    RecordingSupervisor<'a>,
    RecordingDiagnostics<'a>,
> {
    parts_with(
        wake,
        status,
        ota,
        NoUserAction,
        environment_samples,
        battery_samples,
        heartbeats,
        diagnostics,
    )
}

#[allow(clippy::too_many_arguments)]
fn parts_with<'a, O, St, A, const MODULUS: u64>(
    wake: RecordingWake<'a, MODULUS>,
    status: St,
    ota: O,
    actions: A,
    environment_samples: &'a Cell<u32>,
    battery_samples: &'a Cell<u32>,
    heartbeats: &'a Cell<u32>,
    diagnostics: &'a RefCell<Vec<DiagnosticEvent>>,
) -> SensorSedParts<
    RecordingWake<'a, MODULUS>,
    St,
    CountingEnvironment<'a>,
    CountingBattery<'a>,
    O,
    A,
    RecordingSupervisor<'a>,
    RecordingDiagnostics<'a>,
> {
    SensorSedParts {
        wake,
        status,
        environment: CountingEnvironment {
            samples: environment_samples,
        },
        battery: CountingBattery {
            samples: battery_samples,
        },
        ota,
        actions,
        supervisor: RecordingSupervisor { heartbeats },
        diagnostics: RecordingDiagnostics {
            events: diagnostics,
        },
    }
}

#[test]
fn cold_join_and_one_time_initialization_use_real_node_lifecycle() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::empty(&store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);
    script_cold_join(&mut device);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .expect("construct sensor app");

        assert_eq!(
            block_on(app.step()),
            Err(SensorLifecycleError::NotInitialized)
        );
        assert_eq!(block_on(app.initialize()), Ok(()));
        assert_eq!(
            block_on(app.initialize()),
            Err(SensorLifecycleError::AlreadyInitialized)
        );
    }

    assert!(device.is_joined());
    assert_eq!(device.short_address(), OLD_SHORT.0);
    assert_eq!(environment_samples.get(), 1);
    assert_eq!(battery_samples.get(), 1);
    let persisted = store.load().unwrap().unwrap();
    assert!(
        !persisted.commissioned,
        "the cold join remains staged until the unique-TCLK exchange completes"
    );
    assert_eq!(persisted.short_address, OLD_SHORT.0);
    assert_eq!(persisted.network_key, NETWORK_KEY);
    assert!(persisted.global_counter_limit > 0);
    assert!(device.bdb().tclk_exchange_active());
    assert!(diagnostics.borrow().iter().any(|event| matches!(
        event,
        DiagnosticEvent::JoinedOrResumed {
            short_address,
            channel: CHANNEL,
            pan_id: 0x1234,
        } if *short_address == OLD_SHORT.0
    )));
    assert!(
        diagnostics
            .borrow()
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::FastPollStarted { duration_ms: 100 }))
    );
}

#[test]
fn identity_only_sleepy_node_joins_without_sensors_battery_or_status() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::empty(&store_calls);
    let mut profile = DeviceProfile::new(1, 0x0104, DeviceId::SIMPLE_SENSOR, RangeExtender);
    let mut device = ZigbeeDevice::builder(MockMac::new(LOCAL_IEEE))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .power_mode(BASE_POLICY.power_mode())
        .automatic_polling(false)
        .build();
    device.bdb_mut().initialize().expect("BDB initialize");
    script_cold_join(&mut device);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake: RecordingWake<'_> =
            RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            SensorSedParts {
                wake,
                status: NoStatus,
                environment: NoMeasurement,
                battery: NoBattery,
                ota: NoOta,
                actions: NoUserAction,
                supervisor: RecordingSupervisor {
                    heartbeats: &heartbeats,
                },
                diagnostics: RecordingDiagnostics {
                    events: &diagnostics,
                },
            },
        )
        .expect("construct identity-only sensor app");
        assert_eq!(block_on(app.initialize()), Ok(()));
    }

    assert!(device.is_joined());
    assert_eq!(device.short_address(), OLD_SHORT.0);
    let descriptor = device.bdb().zdo().find_endpoint(1).unwrap();
    assert_eq!(descriptor.input_clusters.as_slice(), &[0x0000, 0x0003]);
    assert_eq!(profile.expected_report_clusters(), 0);
}

#[test]
fn finding_binding_target_applies_identify_time_and_answers_query() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(
            0,
            vec![Vec::new(), vec![identify_query_frame(1)]],
            &waits,
            &poll_counts,
            &delay_calls,
        );
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .expect("construct sensor app");

        assert_eq!(block_on(app.initialize()), Ok(()));
        assert_eq!(block_on(app.finding_binding_target()), Ok(()));
        block_on(app.step()).expect("apply IdentifyTime");
        block_on(app.step()).expect("process Identify Query");
    }

    assert!(device.is_identifying(1));
    let record = device
        .mac()
        .tx_history()
        .last()
        .expect("Identify Query must produce a response");
    let bytes = record.payload.as_slice();
    let (_, nwk_len) = NwkHeader::parse(bytes).expect("response NWK header");
    let (security_header, security_len) =
        NwkSecurityHeader::parse(&bytes[nwk_len..]).expect("response NWK security header");
    let aad_len = nwk_len + security_len;
    let mut aad = [0u8; 64];
    aad[..aad_len].copy_from_slice(&bytes[..aad_len]);
    aad[nwk_len] = (aad[nwk_len] & !0x07) | 0x05;
    let plaintext = NwkSecurity::new()
        .decrypt(
            &aad[..aad_len],
            &bytes[aad_len..],
            &NETWORK_KEY,
            &security_header,
        )
        .expect("Identify Query response must authenticate");
    let (aps_header, aps_len) = ApsHeader::parse(&plaintext).expect("response APS header");
    assert_eq!(aps_header.cluster_id, Some(ClusterId::IDENTIFY.0));
    assert_eq!(plaintext[aps_len + 2], 0x00);
}

#[test]
fn warm_resume_is_silent_until_the_finite_step_is_requested() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x01), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
    }

    assert!(device.is_joined());
    assert_eq!(device.mac().poll_count(), 0);
    assert!(device.mac().tx_history().is_empty());
    assert!(waits.borrow().is_empty());
    assert!(
        diagnostics
            .borrow()
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::FastPollStarted { duration_ms: 100 }))
    );
}

#[test]
fn each_step_has_one_bounded_four_round_poll_owner() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(
            0,
            vec![invalid_poll_batch(), invalid_poll_batch()],
            &waits,
            &poll_counts,
            &delay_calls,
        );
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        block_on(app.step()).unwrap();
        block_on(app.step()).unwrap();
    }

    assert_eq!(poll_counts.borrow().as_slice(), &[0, 4]);
    assert_eq!(device.mac().poll_count(), 8);
    assert_eq!(waits.borrow().len(), 2);
}

/// Slow-poll cadence for the stale-runtime-clock regression: 10 s polls
/// with a 25 s sample interval, so every third slow wait is cut short to
/// meet the sample deadline.
const OFF_PHASE_SLOW_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 25_000,
    fast_poll_ms: 250,
    slow_poll_ms: 10_000,
    restored_fast_ms: 2_500,
    ..BASE_POLICY
};

#[test]
fn slow_polls_keep_the_slow_cadence_after_an_off_phase_wait() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &OFF_PHASE_SLOW_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        let mut app = SensorApp::new(
            node,
            &OFF_PHASE_SLOW_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        for _ in 0..30 {
            block_on(app.step()).unwrap();
        }
    }

    let slow: Vec<u32> = waits
        .borrow()
        .iter()
        .filter(|wait| wait.sleep_depth == SleepDepth::Retention)
        .map(|wait| wait.timeout_ms)
        .collect();
    assert!(slow.len() >= 12, "slow waits: {slow:?}");
    // Every Timer wake polls, so a slow wait shorter than the poll period is
    // an extra Data Request. After the first sample-deadline cut the 25 s
    // sample period must hold exactly 10 s + 10 s + 5 s; the old order
    // (poll stamped with the stale runtime clock, then tick) collapsed it
    // into 7 s / 3 s and finally a fixed 5 s / 5 s alternation.
    let first_cut = slow.iter().position(|&timeout| timeout != 10_000).unwrap();
    for period in slow[first_cut + 1..].chunks_exact(3) {
        assert_eq!(period, [10_000, 10_000, 5_000], "slow waits: {slow:?}");
    }
}

#[test]
fn reporting_completion_uses_short_grace_then_the_slow_wait_depth() {
    exercise_reporting_completion(false);
}

/// A report deadline 1 ms past a whole number of slow polls.
const RESIDUE_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 10_001,
    fast_poll_ms: 250,
    slow_poll_ms: 1_000,
    restored_fast_ms: 2_500,
    slow_sleep_depth: SleepDepth::Idle,
    ..BASE_POLICY
};

/// The lifecycle shortens a low-power wait to any deadline residue, down to
/// 1 ms. Every `WakeController` must accept that at each sleep depth; TLSR8258
/// SUSPEND/LOW32K reject a sleep below their PM wake floor (`TooShort`), so its
/// controller busy-waits such a residue instead of failing the wait.
#[test]
fn a_report_deadline_residue_requests_a_one_ms_wait_at_the_slow_depth() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &RESIDUE_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        let mut app = SensorApp::new(
            node,
            &RESIDUE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        for _ in 0..30 {
            block_on(app.step()).unwrap();
        }
    }

    let waits = waits.borrow();
    assert!(
        waits.contains(&WaitRequest {
            timeout_ms: 1,
            sleep_depth: SleepDepth::Idle,
        }),
        "waits: {waits:?}"
    );
}

#[test]
fn receive_activity_without_sleep_is_processed_before_rechecking_idle() {
    exercise_reporting_completion(true);
}

fn exercise_reporting_completion(activity: bool) {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);
    let reports = vec![
        configure_reporting_frame(ClusterId::TEMPERATURE, 0x0000, 0x29, &1i16.to_le_bytes(), 1),
        configure_reporting_frame(ClusterId::HUMIDITY, 0x0000, 0x21, &1u16.to_le_bytes(), 2),
        configure_reporting_frame(ClusterId::POWER_CONFIG, 0x0021, 0x20, &[1], 3),
    ];

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let mut wake = RecordingWake::new(0, vec![reports], &waits, &poll_counts, &delay_calls);
        wake.activity_once = activity;
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        block_on(app.step()).unwrap();
        block_on(app.step()).unwrap();
        block_on(app.step()).unwrap();
    }

    assert_eq!(
        waits.borrow().as_slice(),
        &[
            WaitRequest {
                timeout_ms: 10,
                sleep_depth: SleepDepth::Active,
            },
            WaitRequest {
                timeout_ms: 5,
                sleep_depth: SleepDepth::Active,
            },
            WaitRequest {
                timeout_ms: 1_000,
                sleep_depth: SleepDepth::Retention,
            },
        ],
        "configured={}, security={:?}, diagnostics={:?}",
        device.remote_reporting_cluster_count(1),
        device.bdb().zdo().aps().nwk().rx_security_stats(),
        diagnostics.borrow().as_slice(),
    );
    assert_eq!(device.remote_reporting_cluster_count(1), 3);
    assert!(diagnostics.borrow().iter().any(|event| matches!(
        event,
        DiagnosticEvent::InterviewConfigurationComplete {
            configured: 3,
            expected: 3,
        }
    )));
    assert!(diagnostics.borrow().iter().any(|event| matches!(
        event,
        DiagnosticEvent::FastPollStopped {
            configured: 3,
            expected: 3,
        }
    )));
}

#[test]
fn unconfirmed_aps_delivery_vetoes_idle_until_confirmed_or_cancelled() {
    let profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);
    assert!(!device.has_pending_protocol_work());
    assert!(
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .register_ack_pending(7, 0x1234, &[0, 1, 2])
            .is_some()
    );
    assert!(device.has_pending_protocol_work());
    assert!(!device.bdb_mut().zdo_mut().aps_mut().confirm_ack(0x5678, 7));
    assert!(device.has_pending_protocol_work());
    assert!(device.bdb_mut().zdo_mut().aps_mut().confirm_ack(0x1234, 7));
    assert!(!device.has_pending_protocol_work());
    assert!(
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .register_ack_pending(8, 0x1234, &[0, 1, 2])
            .is_some()
    );
    assert!(device.has_pending_protocol_work());
    device
        .bdb_mut()
        .zdo_mut()
        .aps_mut()
        .cancel_all_ack_tracking();
    assert!(!device.has_pending_protocol_work());
}

#[test]
fn parent_leave_clears_joined_state_and_durable_commissioning() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(
            0,
            vec![vec![leave_request(false, 1)]],
            &waits,
            &poll_counts,
            &delay_calls,
        );
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        block_on(app.step()).unwrap();
    }

    assert!(
        !diagnostics
            .borrow()
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::JoinRetry { .. })),
        "a leave without rejoin must not start a new search in the same step"
    );
    assert!(
        !device.is_joined(),
        "diagnostics={:?}, polls={}, security={:?}",
        diagnostics.borrow().as_slice(),
        device.mac().poll_count(),
        device.bdb().zdo().aps().nwk().rx_security_stats(),
    );
    assert!(!store.load().unwrap().unwrap().commissioned);
    assert!(
        diagnostics
            .borrow()
            .contains(&DiagnosticEvent::LeaveRequested)
    );
}

#[test]
fn basic_reset_preserves_sleepy_network_credentials_and_commissioning() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let original = commissioned_state(0x02);
    let mut store = CountingStore::with_state(original, &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(
            0,
            vec![vec![basic_reset_request(1)]],
            &waits,
            &poll_counts,
            &delay_calls,
        );
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        block_on(app.step()).unwrap();
    }

    let persisted = store.load().unwrap().unwrap();
    assert!(device.is_joined());
    assert!(persisted.commissioned);
    assert_eq!(persisted.pan_id, original.pan_id);
    assert_eq!(persisted.network_key, original.network_key);
    assert!(
        persisted.tclk_counter_limit >= original.tclk_counter_limit,
        "receiving a secured Basic command may reserve a later counter floor,
         but must never erase or roll it back"
    );
    assert!(
        diagnostics
            .borrow()
            .contains(&DiagnosticEvent::BasicResetToFactoryDefaults)
    );
    assert!(
        !diagnostics
            .borrow()
            .contains(&DiagnosticEvent::LeaveRequested)
    );
}

#[test]
fn secure_rejoin_request_recovers_through_the_mock_parent() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);
    device.mac_mut().add_beacon(join_beacon(false));

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(
            0,
            vec![vec![leave_request(true, 1), secured_rejoin_response(2)]],
            &waits,
            &poll_counts,
            &delay_calls,
        );
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        block_on(app.step()).unwrap();
    }

    assert!(device.is_joined());
    assert_eq!(
        device.short_address(),
        NEW_SHORT.0,
        "diagnostics={:?}, polls={}, security={:?}",
        diagnostics.borrow().as_slice(),
        device.mac().poll_count(),
        device.bdb().zdo().aps().nwk().rx_security_stats(),
    );
    let persisted = store.load().unwrap().unwrap();
    assert!(persisted.commissioned);
    assert!(!persisted.rejoin_pending);
    assert_eq!(persisted.short_address, NEW_SHORT.0);
    assert!(
        diagnostics
            .borrow()
            .contains(&DiagnosticEvent::RejoinRequested)
    );
    assert!(diagnostics.borrow().iter().any(|event| matches!(
        event,
        DiagnosticEvent::SecureRejoinSucceeded { short_address }
            if *short_address == NEW_SHORT.0
    )));
}

fn check_raw_ota_routing(keep_awake_ms: u32) {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let load_calls = Cell::new(0);
    let seen_raw_command = Cell::new(false);
    let activated = Cell::new(0);
    let loads_at_handle = Cell::new(0);
    let checkpoint_before_activation = Cell::new(false);
    let mut store =
        CountingStore::with_state_and_loads(commissioned_state(0x02), &store_calls, &load_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(
            0,
            vec![vec![raw_ota_command(1)]],
            &waits,
            &poll_counts,
            &delay_calls,
        );
        let ota = RecordingOta {
            keep_awake_ms,
            seen_raw_command: &seen_raw_command,
            activated: &activated,
            loads_at_handle: &loads_at_handle,
            checkpoint_before_activation: &checkpoint_before_activation,
            load_calls: &load_calls,
        };
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                ota,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        block_on(app.step()).unwrap();
        for _ in 0..keep_awake_ms.div_ceil(BASE_POLICY.fast_poll_ms) {
            block_on(app.step()).unwrap();
        }
        assert!(
            waits
                .borrow()
                .iter()
                .all(|wait| wait.sleep_depth == SleepDepth::Active),
            "generic event handling must not shorten the OTA keep-awake window: {:?}",
            waits.borrow()
        );
        assert_eq!(
            waits
                .borrow()
                .iter()
                .map(|wait| wait.timeout_ms)
                .sum::<u32>(),
            BASE_POLICY.fast_poll_ms + keep_awake_ms
        );
        block_on(app.step()).unwrap();
    }

    assert!(
        seen_raw_command.get(),
        "diagnostics={:?}, polls={}, security={:?}",
        diagnostics.borrow().as_slice(),
        device.mac().poll_count(),
        device.bdb().zdo().aps().nwk().rx_security_stats(),
    );
    assert_eq!(activated.get(), 1);
    assert!(checkpoint_before_activation.get());
    assert_eq!(
        waits.borrow().last().unwrap().sleep_depth,
        SleepDepth::Retention
    );
    assert!(!diagnostics.borrow().iter().any(|event| matches!(
        event,
        DiagnosticEvent::UnhandledCommand {
            cluster_id,
            command_id: 0x77,
            ..
        } if *cluster_id == ClusterId::OTA_UPGRADE.0
    )));
}

#[test]
fn raw_ota_command_is_handled_before_generic_matching_and_activation_is_checkpointed() {
    for keep_awake_ms in [25, 250] {
        check_raw_ota_routing(keep_awake_ms);
    }
}

fn check_ota_activation_checkpoints(trigger: ActivationTrigger) {
    for failure in [
        None,
        Some(CheckpointFailure::Load),
        Some(CheckpointFailure::Store),
    ] {
        let waits = RefCell::new(Vec::new());
        let poll_counts = RefCell::new(Vec::new());
        let delay_calls = Cell::new(0);
        let environment_samples = Cell::new(0);
        let battery_samples = Cell::new(0);
        let heartbeats = Cell::new(0);
        let diagnostics = RefCell::new(Vec::new());
        let status = RefCell::new(Vec::new());
        let store_calls = Cell::new(0);
        let fault = Cell::new(None);
        let requested = Cell::new(false);
        let activated = Cell::new(0);
        let limit_before = Cell::new(0);
        let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
        store.checkpoint_failure = Some(&fault);
        let mut profile = test_profile();
        let mut device = test_device(&profile, &BASE_POLICY);
        {
            let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
            let batches = if trigger == ActivationTrigger::Event {
                vec![vec![raw_ota_command(1)]]
            } else {
                Vec::new()
            };
            let wake = RecordingWake::new(0, batches, &waits, &poll_counts, &delay_calls);
            let ota = CheckpointOta {
                trigger,
                failure,
                fault: &fault,
                requested: &requested,
                activated: &activated,
                limit_before: &limit_before,
            };
            let mut app = SensorApp::new(
                node,
                &BASE_POLICY,
                parts(
                    wake,
                    RecordingStatus { events: &status },
                    ota,
                    &environment_samples,
                    &battery_samples,
                    &heartbeats,
                    &diagnostics,
                ),
            )
            .unwrap();
            block_on(app.initialize()).unwrap();
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                block_on(app.step()).unwrap();
            }));
            assert!(
                requested.get(),
                "{trigger:?}/{failure:?} must reach activation intent"
            );
            assert_eq!(fault.get(), None, "the selected fault must actually fire");
            if failure.is_some() {
                let panic = result.expect_err("checkpoint failure must stop the lifecycle");
                assert_eq!(
                    panic.downcast_ref::<&str>().copied(),
                    Some("security persistence failure")
                );
                assert_eq!(activated.get(), 0);
                assert!(
                    diagnostics
                        .borrow()
                        .contains(&DiagnosticEvent::SecurityFailure(
                            SecurityStoreError::Hardware
                        ))
                );
                assert_eq!(status.borrow().last(), Some(&SensorStatus::Fault));
            } else {
                assert!(result.is_ok());
                assert_eq!(activated.get(), 1);
            }
        }
        let persisted = store.load().unwrap().unwrap();
        assert!(persisted.commissioned);
        assert_eq!(persisted.network_key, NETWORK_KEY);
        if failure.is_some() {
            assert_eq!(persisted.global_counter_limit, limit_before.get());
            assert_eq!(
                device.bdb().zdo().nwk().nib().outgoing_frame_counter_limit,
                limit_before.get(),
                "a failed checkpoint must not release a new live reservation"
            );
        }
        let mut rebooted = test_device(&profile, &BASE_POLICY);
        let mut node = ZigbeeNode::new(&mut rebooted, &mut store, &mut profile);
        block_on(node.start_or_resume()).unwrap();
        assert!(
            node.device().bdb().zdo().nwk().nib().outgoing_frame_counter
                >= persisted.global_counter_limit,
            "reboot must skip the entire old reservation after {trigger:?}/{failure:?}"
        );
    }
}

#[test]
fn ota_event_activation_requires_successful_security_checkpoint() {
    check_ota_activation_checkpoints(ActivationTrigger::Event);
}

#[test]
fn ota_service_activation_requires_successful_security_checkpoint() {
    check_ota_activation_checkpoints(ActivationTrigger::Service);
}

#[test]
fn no_status_omits_blink_deadlines_and_delays_across_rollover() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::empty(&store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &NO_STATUS_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(
            u32::MAX - 50,
            Vec::new(),
            &waits,
            &poll_counts,
            &delay_calls,
        );
        let mut app = SensorApp::new(
            node,
            &NO_STATUS_POLICY,
            parts(
                wake,
                NoStatus,
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .expect("zero status timings are valid without a status sink");
        block_on(app.initialize()).unwrap();
        block_on(app.step()).unwrap();
    }

    assert_eq!(
        waits.borrow().as_slice(),
        &[WaitRequest {
            timeout_ms: 100,
            sleep_depth: SleepDepth::Retention,
        }],
        "an unjoined sensor sleeps at the steady-state depth between attempts"
    );
    assert_eq!(delay_calls.get(), 0);
    assert_eq!(
        diagnostics
            .borrow()
            .iter()
            .filter(|event| matches!(event, DiagnosticEvent::CommissioningFailed { .. }))
            .count(),
        2,
        "the retry deadline must still expire correctly across u32 rollover"
    );
}

/// Everything a scripted unjoined/leave scenario recorded.
struct Scenario {
    waits: Vec<WaitRequest>,
    poll_counts: Vec<u32>,
    delay_calls: u32,
    diagnostics: Vec<DiagnosticEvent>,
    status: Vec<SensorStatus>,
    joined: bool,
    commissioned: Option<bool>,
}

impl Scenario {
    fn count(&self, predicate: impl Fn(&DiagnosticEvent) -> bool) -> usize {
        self.diagnostics
            .iter()
            .filter(|event| predicate(event))
            .count()
    }

    fn backoffs(&self) -> Vec<u32> {
        self.diagnostics
            .iter()
            .filter_map(|event| match event {
                DiagnosticEvent::JoinBackoff { delay_ms } => Some(*delay_ms),
                _ => None,
            })
            .collect()
    }

    fn attempts(&self) -> usize {
        self.count(|event| matches!(event, DiagnosticEvent::JoinRetry { .. }))
    }
}

/// Script for [`run_scenario`].
struct Script {
    policy: &'static SensorPolicy,
    stored: Option<PersistentSecurityState>,
    start_ms: u32,
    batches: Vec<Vec<MacFrame>>,
    reasons: Vec<(usize, WakeReason)>,
    /// Offer a permit-join beacon and successful association response, so
    /// any network search the app starts would succeed.
    beacon: bool,
    steps: usize,
    /// Emulate a reset-on-wake platform: each iteration's sleep is taken by
    /// the platform (time advances by the request) and the app resumes with
    /// `resume_after_timer_wake` instead of returning from `wait`.
    reset_on_wake: bool,
}

impl Script {
    fn new(policy: &'static SensorPolicy, steps: usize) -> Self {
        Self {
            policy,
            stored: None,
            start_ms: 0,
            batches: Vec::new(),
            reasons: Vec::new(),
            beacon: false,
            steps,
            reset_on_wake: false,
        }
    }
}

fn run_scenario<A: sensor_sed_app::UserActionPolicy, const MODULUS: u64>(
    script: Script,
    actions: A,
) -> Scenario {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = match script.stored {
        Some(state) => CountingStore::with_state(state, &store_calls),
        None => CountingStore::empty(&store_calls),
    };
    let mut profile = test_profile();
    let mut device = test_device(&profile, script.policy);
    if script.beacon {
        let mac = device.bdb_mut().zdo_mut().aps_mut().nwk_mut().mac_mut();
        mac.add_beacon(join_beacon(true));
        mac.set_associate_response(MlmeAssociateConfirm {
            short_address: OLD_SHORT,
            status: AssociationStatus::Success,
        });
    }

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let mut wake: RecordingWake<'_, MODULUS> = RecordingWake::new(
            script.start_ms,
            script.batches,
            &waits,
            &poll_counts,
            &delay_calls,
        );
        wake.reasons = script.reasons;
        let mut app = SensorApp::new(
            node,
            script.policy,
            parts_with(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                actions,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .expect("valid scenario policy");
        block_on(app.initialize()).unwrap();
        for _ in 0..script.steps {
            if script.reset_on_wake {
                block_on(app.resume_after_timer_wake()).unwrap();
            } else {
                block_on(app.step()).unwrap();
            }
        }
    }

    let joined = device.is_joined();
    Scenario {
        waits: waits.into_inner(),
        poll_counts: poll_counts.into_inner(),
        delay_calls: delay_calls.get(),
        diagnostics: diagnostics.into_inner(),
        status: status.into_inner(),
        joined,
        commissioned: store.load().unwrap().map(|state| state.commissioned),
    }
}

const ONE_MINUTE: WaitRequest = WaitRequest {
    timeout_ms: 60_000,
    sleep_depth: SleepDepth::Retention,
};

#[test]
fn leave_without_rejoin_waits_for_the_button_then_the_button_commissions() {
    let mut script = Script::new(&BASE_POLICY, 5);
    script.stored = Some(commissioned_state(0x02));
    script.beacon = true;
    // Wait 0 delivers the leave; waits 1-2 sleep; wait 3 is a button press
    // and queues the Transport-Key for the button-started join.
    script.batches = vec![
        vec![leave_request(false, 1)],
        Vec::new(),
        Vec::new(),
        vec![transport_key_frame()],
    ];
    script.reasons = vec![(3, WakeReason::Button)];

    let before_button = run_scenario::<_, { 1 << 32 }>(
        Script {
            steps: 3,
            batches: vec![vec![leave_request(false, 1)]],
            reasons: Vec::new(),
            stored: Some(commissioned_state(0x02)),
            ..Script::new(&BASE_POLICY, 3)
        }
        .with_beacon(),
        sensor_sed_app::JoinOnlyAction,
    );
    assert!(!before_button.joined);
    assert_eq!(before_button.commissioned, Some(false));
    assert!(
        before_button
            .diagnostics
            .contains(&DiagnosticEvent::WaitingForButton)
    );
    assert_eq!(
        before_button.attempts(),
        0,
        "a joinable network is in range, but an explicit leave must not search: {:?}",
        before_button.diagnostics
    );
    assert!(before_button.backoffs().is_empty());
    assert_eq!(&before_button.waits[1..], &[ONE_MINUTE, ONE_MINUTE]);

    let pressed = run_scenario::<_, { 1 << 32 }>(script, sensor_sed_app::JoinOnlyAction);
    assert!(pressed.joined, "diagnostics={:?}", pressed.diagnostics);
    assert_eq!(
        pressed.count(|event| *event == DiagnosticEvent::ButtonJoin),
        1
    );
    assert_eq!(pressed.attempts(), 0, "the press joins directly");
}

impl Script {
    fn with_beacon(mut self) -> Self {
        self.beacon = true;
        self
    }
}

#[test]
fn leave_without_a_button_action_falls_back_to_the_bounded_backoff() {
    let mut script = Script::new(&BASE_POLICY, 1).with_beacon();
    script.stored = Some(commissioned_state(0x02));
    script.batches = vec![vec![leave_request(false, 1)]];
    let left = run_scenario::<_, { 1 << 32 }>(script, NoUserAction);
    assert!(!left.joined);
    assert!(
        !left
            .diagnostics
            .contains(&DiagnosticEvent::WaitingForButton)
    );
    assert_eq!(left.backoffs(), vec![100]);
    assert_eq!(left.attempts(), 0, "no immediate rejoin after the leave");
}

#[test]
fn unjoined_backoff_doubles_sleeps_and_blinks_only_during_attempts() {
    let unjoined = run_scenario::<_, { 1 << 32 }>(Script::new(&BASE_POLICY, 5), NoUserAction);
    assert!(!unjoined.joined);
    assert_eq!(unjoined.backoffs(), vec![100, 200, 400, 800, 1_600, 3_200]);
    assert_eq!(
        unjoined.waits,
        [100, 200, 400, 800, 1_600]
            .map(|timeout_ms| WaitRequest {
                timeout_ms,
                sleep_depth: SleepDepth::Retention,
            })
            .to_vec(),
        "no Active waits and no idle wakeups between attempts"
    );
    // Cold start plus one attempt per expired backoff, each with exactly
    // one double blink (on, gap, on delays).
    assert_eq!(unjoined.attempts(), 5);
    let blinks = unjoined
        .status
        .iter()
        .filter(|status| **status == SensorStatus::Joining { on: true })
        .count();
    assert_eq!(blinks, 2 * 6);
    assert_eq!(unjoined.delay_calls, 3 * 6);
}

#[test]
fn reset_on_wake_resume_finishes_the_slept_iteration_without_a_new_wait() {
    let mut script = Script::new(&BASE_POLICY, 5);
    script.reset_on_wake = true;
    let resumed = run_scenario::<_, { 1 << 32 }>(script, NoUserAction);
    assert!(!resumed.joined);
    assert!(resumed.waits.is_empty(), "the platform owned every sleep");
    // Each resumed timer wake charges the whole backoff chunk, so it
    // makes exactly the attempt `step` would have made after its wait.
    assert_eq!(resumed.backoffs(), vec![100, 200, 400, 800, 1_600, 3_200]);
    assert_eq!(resumed.attempts(), 5);
}

#[test]
fn reset_on_wake_resume_polls_a_joined_parent_and_requires_initialization() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BASE_POLICY);

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        let mut app = SensorApp::new(
            node,
            &BASE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        assert_eq!(
            block_on(app.resume_after_timer_wake()),
            Err(SensorLifecycleError::NotInitialized)
        );
        block_on(app.initialize()).unwrap();
        block_on(app.resume_after_timer_wake()).unwrap();
    }

    assert!(device.is_joined());
    assert!(waits.borrow().is_empty());
    assert!(device.mac().poll_count() > 0, "the slept poll is serviced");
}

#[test]
fn unjoined_backoff_caps_at_thirty_minutes_in_bounded_sleeps_across_rollover() {
    const POLICY: SensorPolicy = SensorPolicy {
        join_retry_ms: 15_000,
        ..BASE_POLICY
    };
    // An 18-bit millisecond counter (262 s) wraps many times over the
    // backoff, like the TLSR8258 24 MHz Timer0 (179 s).
    let unjoined = run_scenario::<_, { 1 << 18 }>(Script::new(&POLICY, 100), NoUserAction);
    assert!(
        unjoined
            .waits
            .iter()
            .all(|wait| wait.timeout_ms <= 60_000 && wait.sleep_depth == SleepDepth::Retention)
    );
    assert_eq!(
        unjoined.backoffs(),
        vec![
            15_000, 30_000, 60_000, 120_000, 240_000, 480_000, 960_000, 1_800_000, 1_800_000,
            1_800_000
        ]
    );
    // 93 one-minute-or-shorter sleeps cover the first nine delays exactly;
    // the remaining seven are part of the tenth.
    assert_eq!(unjoined.attempts(), 9);
    let slept: u64 = unjoined
        .waits
        .iter()
        .map(|wait| u64::from(wait.timeout_ms))
        .sum();
    let scheduled: u64 = unjoined.backoffs()[..9]
        .iter()
        .map(|delay| u64::from(*delay))
        .sum();
    assert_eq!(slept - scheduled, 7 * 60_000);
}

#[test]
fn button_press_resets_the_backoff_and_tries_immediately() {
    let mut script = Script::new(&BASE_POLICY, 5);
    // Waits 0..2 consume 100, 200, 400 ms backoffs; wait 3 (800 ms) is
    // interrupted by the button.
    script.reasons = vec![(3, WakeReason::Button)];
    let pressed = run_scenario::<_, { 1 << 32 }>(script, sensor_sed_app::JoinOnlyAction);
    assert!(!pressed.joined);
    assert_eq!(
        pressed.count(|event| *event == DiagnosticEvent::ButtonJoin),
        1
    );
    assert_eq!(pressed.backoffs(), vec![100, 200, 400, 800, 100, 200]);
    assert_eq!(pressed.waits[4].timeout_ms, 100);
}

#[test]
fn toggle_join_leave_stays_factory_new_until_the_next_press() {
    let mut script = Script::new(&BASE_POLICY, 6).with_beacon();
    script.stored = Some(commissioned_state(0x02));
    script.reasons = vec![(1, WakeReason::Button)];
    let left = run_scenario::<_, { 1 << 32 }>(script, sensor_sed_app::ToggleJoinAction);
    assert!(!left.joined, "diagnostics={:?}", left.diagnostics);
    assert_eq!(left.commissioned, Some(false));
    assert!(left.diagnostics.contains(&DiagnosticEvent::ButtonLeave));
    assert!(
        left.diagnostics
            .contains(&DiagnosticEvent::WaitingForButton)
    );
    assert_eq!(left.attempts(), 0, "{:?}", left.diagnostics);
    assert!(left.waits[2..].iter().all(|wait| *wait == ONE_MINUTE));
}

#[test]
fn timed_fast_poll_window_does_not_recur_after_counter_rollover() {
    let mut script = Script::new(&BASE_POLICY, 200);
    script.stored = Some(commissioned_state(0x01));
    // Marks wrap every 99.95 s. The slow polls land on whole seconds after
    // the 60 s sample, so at t = 100 s the counter reads 50 ms after the
    // restored fast-poll start; a window that is never cleared reopens there.
    let resumed = run_scenario::<_, 99_950>(script, NoUserAction);
    assert!(resumed.joined);
    let first_slow = resumed
        .waits
        .iter()
        .position(|wait| wait.timeout_ms == BASE_POLICY.slow_poll_ms)
        .expect("restored fast poll ends");
    assert!(
        resumed.waits[first_slow..]
            .iter()
            .all(|wait| wait.timeout_ms > BASE_POLICY.fast_poll_ms
                && wait.sleep_depth == SleepDepth::Retention),
        "fast poll restarted after rollover: {:?}",
        &resumed.waits[first_slow..]
    );
    assert_eq!(
        resumed.count(|event| matches!(event, DiagnosticEvent::FastPollStopped { .. })),
        1
    );
}

#[test]
fn activity_wake_polls_the_parent_only_when_the_poll_is_due() {
    const AT: usize = 40;
    let mut script = Script::new(&BASE_POLICY, AT + 3);
    script.stored = Some(commissioned_state(0x01));
    script.reasons = vec![(AT, WakeReason::Activity)];
    let resumed = run_scenario::<_, { 1 << 32 }>(script, NoUserAction);
    assert!(resumed.joined);
    assert_eq!(resumed.waits[AT].timeout_ms, BASE_POLICY.slow_poll_ms);
    assert_eq!(
        resumed.poll_counts[AT + 1],
        resumed.poll_counts[AT],
        "a skipped sleep must not send an extra Data Request"
    );
    assert!(
        resumed.poll_counts[AT + 2] > resumed.poll_counts[AT + 1],
        "the following timer wake polls normally"
    );
}

type PlantProfile = DeviceProfile<PlantSensor>;

/// Product-style plant measurement: records which trigger the lifecycle
/// used and writes a soil reading through the profile.
struct RecordingPlantMeasurement<'a> {
    triggers: &'a RefCell<Vec<SampleTrigger>>,
}

impl SensorMeasurement<PlantProfile> for RecordingPlantMeasurement<'_> {
    async fn measure<D: Diagnostics>(
        &mut self,
        profile: &mut PlantProfile,
        trigger: SampleTrigger,
        diagnostics: &mut D,
    ) {
        self.triggers.borrow_mut().push(trigger);
        profile.component_mut().update_soil_moisture(4_200);
        diagnostics.record(DiagnosticEvent::Measurement {
            cluster_id: ClusterId::SOIL_MOISTURE.0,
            value: 4_200,
        });
    }
}

#[test]
fn plant_profile_measurement_receives_cold_start_and_periodic_triggers() {
    const PLANT_POLICY: SensorPolicy = SensorPolicy {
        sample_interval_ms: 1_000,
        ..BASE_POLICY
    };
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let triggers = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x01), &store_calls);
    let mut profile = DeviceProfile::new(
        1,
        0x0104,
        DeviceId::SIMPLE_SENSOR,
        PlantSensor::new(
            TemperatureRange {
                min_centi_celsius: -4_000,
                max_centi_celsius: 8_500,
            },
            0xFFFE,
            PlantSensorReporting::default(),
        )
        .without_illuminance()
        .with_battery(BatteryDescriptor {
            size: 10,
            quantity: 1,
            rated_voltage_100mv: 30,
        }),
    );
    let mut device = ZigbeeDevice::builder(MockMac::new(LOCAL_IEEE))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .power_mode(PLANT_POLICY.power_mode())
        .automatic_polling(false)
        .build();
    device.bdb_mut().initialize().expect("BDB initialize");

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake: RecordingWake<'_> =
            RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        let mut app = SensorApp::new(
            node,
            &PLANT_POLICY,
            SensorSedParts {
                wake,
                status: NoStatus,
                environment: RecordingPlantMeasurement {
                    triggers: &triggers,
                },
                battery: CountingBattery {
                    samples: &battery_samples,
                },
                ota: NoOta,
                actions: NoUserAction,
                supervisor: RecordingSupervisor {
                    heartbeats: &heartbeats,
                },
                diagnostics: RecordingDiagnostics {
                    events: &diagnostics,
                },
            },
        )
        .expect("construct plant sensor app");
        block_on(app.initialize()).expect("initialize");
        for _ in 0..200 {
            if triggers.borrow().len() >= 3 {
                break;
            }
            block_on(app.step()).expect("step");
        }
    }

    let triggers = triggers.borrow();
    assert_eq!(triggers.first(), Some(&SampleTrigger::ColdStart));
    assert!(
        triggers[1..]
            .iter()
            .all(|trigger| *trigger == SampleTrigger::Periodic)
    );
    assert_eq!(triggers.len(), 3, "periodic samples must follow");
    assert_eq!(battery_samples.get() as usize, triggers.len());
    assert_eq!(
        profile
            .component()
            .soil_moisture()
            .attributes()
            .get(zigbee_zcl::clusters::soil_moisture::ATTR_MEASURED_VALUE),
        Some(&zigbee_zcl::data_types::ZclValue::U16(4_200))
    );
    assert!(diagnostics.borrow().iter().any(|event| matches!(
        event,
        DiagnosticEvent::Measurement {
            cluster_id: 0x0408,
            value: 4_200
        }
    )));
}

/// A commissioned sensor that keeps failing its rejoin keeps its network
/// (`secure_rejoin_failure_limit = 0`), sleeps at the slow depth between
/// attempts, and spaces them on the jittered exponential backoff capped at
/// the product's `join_retry_max_ms`.
static NEVER_GIVE_UP_POLICY: SensorPolicy = SensorPolicy {
    join_retry_ms: 6_000,
    secure_rejoin_failure_limit: 0,
    recovery: sensor_sed_app::RecoveryPolicy {
        join_retry_max_ms: 24_000,
        join_retry_jitter_pct: 25,
        ..DETERMINISTIC_RECOVERY
    },
    ..BASE_POLICY
};

#[test]
fn failed_rejoins_keep_the_network_and_sleep_on_the_jittered_capped_backoff() {
    let mut stored = commissioned_state(0x02);
    stored.rejoin_pending = true;
    let mut script = Script::new(&NEVER_GIVE_UP_POLICY, 12);
    script.stored = Some(stored);
    let lost = run_scenario::<_, { 1 << 32 }>(script, NoUserAction);

    assert!(!lost.joined);
    assert_eq!(lost.commissioned, Some(true), "the network is kept");
    assert_eq!(
        lost.count(|event| matches!(
            event,
            DiagnosticEvent::SecureRejoinLimitReached { .. }
                | DiagnosticEvent::FactoryResetFailed { .. }
                | DiagnosticEvent::JoinedOrResumed { .. }
        )),
        0,
        "{:?}",
        lost.diagnostics
    );

    let backoffs = lost.backoffs();
    assert!(backoffs.len() >= 5, "{backoffs:?}");
    let nominal = [6_000u32, 12_000, 24_000, 24_000, 24_000, 24_000];
    for (delay, nominal) in backoffs.iter().zip(nominal) {
        assert!(
            *delay <= nominal && *delay >= nominal - nominal / 4,
            "delay {delay} outside the 25 % jitter below {nominal}: {backoffs:?}"
        );
    }
    assert!(
        backoffs.windows(2).any(|pair| pair[0] != pair[1]) || backoffs[0] != nominal[0],
        "jitter must actually vary the schedule: {backoffs:?}"
    );
    // Every failure was counted, and every wait between the attempts is a
    // radio-off sleep at the slow depth that never exceeds the backoff.
    assert_eq!(
        lost.count(|event| matches!(event, DiagnosticEvent::SecureRejoinPending { .. })),
        backoffs.len()
    );
    assert!(
        lost.waits
            .iter()
            .all(|wait| wait.sleep_depth == SleepDepth::Retention && wait.timeout_ms <= 24_000),
        "{:?}",
        lost.waits
    );
    assert_eq!(
        lost.waits.iter().map(|wait| wait.timeout_ms).sum::<u32>(),
        backoffs[..lost.waits.len().min(backoffs.len())]
            .iter()
            .sum::<u32>(),
        "the device sleeps the whole backoff, not polls through it"
    );
}

/// Pending protocol work may keep the chip awake only for `max_active_ms`;
/// afterwards it sleeps at its normal depth, and persisting work earns
/// parent-loss strikes until the staged rejoin starts. The failed rejoin
/// keeps the network and continues on the sleeping backoff.
static BOUNDED_ACTIVE_POLICY: SensorPolicy = SensorPolicy {
    fast_sleep_depth: SleepDepth::Idle,
    secure_rejoin_failure_limit: 0,
    join_retry_ms: 6_000,
    recovery: sensor_sed_app::RecoveryPolicy {
        parent_loss_strikes: 3,
        max_active_ms: 50,
        ..DETERMINISTIC_RECOVERY
    },
    ..BASE_POLICY
};

#[test]
fn pending_work_is_bounded_and_persisting_work_starts_the_rejoin() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = test_profile();
    let mut device = test_device(&profile, &BOUNDED_ACTIVE_POLICY);
    let pending_from;

    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        let mut app = SensorApp::new(
            node,
            &BOUNDED_ACTIVE_POLICY,
            parts(
                wake,
                RecordingStatus { events: &status },
                NoOta,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        // Let the restored fast-poll window expire.
        for _ in 0..20 {
            block_on(app.step()).unwrap();
        }
        assert!(app.device().is_joined());
        assert!(
            waits
                .borrow()
                .iter()
                .all(|wait| wait.sleep_depth != SleepDepth::Active)
        );
        // An acknowledged unicast whose APS ACK never comes back.
        assert!(
            app.device_mut()
                .bdb_mut()
                .zdo_mut()
                .aps_mut()
                .register_ack_pending(0x77, 0x0000, &[0, 1, 2])
                .is_some()
        );
        pending_from = waits.borrow().len();
        for _ in 0..40 {
            block_on(app.step()).unwrap();
        }
    }

    let waits = waits.into_inner();
    let diagnostics = diagnostics.into_inner();
    let after = &waits[pending_from..];
    let active: Vec<_> = after
        .iter()
        .take_while(|wait| wait.sleep_depth == SleepDepth::Active)
        .collect();
    assert!(
        !active.is_empty(),
        "pending work first gets the fast active wait"
    );
    assert!(
        active.iter().map(|wait| wait.timeout_ms).sum::<u32>() <= 50 + 10,
        "but no longer than max_active_ms: {after:?}"
    );
    assert!(
        after[active.len()..]
            .iter()
            .all(|wait| wait.sleep_depth != SleepDepth::Active),
        "the expired work never forces the chip awake again: {after:?}"
    );
    assert!(
        diagnostics
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::ParentLossSuspected { strikes: 3 })),
        "{diagnostics:?}"
    );
    assert!(
        diagnostics
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::SecureRejoinFailed { .. })),
        "the strikes started a real rejoin: {diagnostics:?}"
    );
    assert!(!device.is_joined());
    let persisted = store.load().unwrap().unwrap();
    assert!(persisted.commissioned, "the network is kept");
    assert!(
        !diagnostics
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::SecureRejoinLimitReached { .. }))
    );
    assert_eq!(
        waits.last().unwrap().sleep_depth,
        SleepDepth::Retention,
        "the lost device sleeps on the backoff"
    );
}

/// Environmental component with a Poll Control server, as a product that
/// fits one composes it.
struct PolledEnvironment {
    inner: TemperatureHumidityBattery,
    poll_control: PollControlCluster,
}

impl ProfileComponent for PolledEnvironment {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        self.inner
            .configure_endpoint(endpoint)
            .cluster_server(ClusterId::POLL_CONTROL)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        self.inner.collect_clusters(endpoint, clusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.poll_control,
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    fn expected_report_cluster_ids(&self, out: &mut ExpectedReportClusters) {
        self.inner.expected_report_cluster_ids(out);
    }
}

impl sensor_sed_app::NonOtaComponent for PolledEnvironment {}

impl sensor_sed_app::EnvironmentSink for PolledEnvironment {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.inner.update_environment(measurement);
    }

    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.inner.update_battery(measurement);
    }

    fn poll_control_mut(&mut self) -> Option<&mut PollControlCluster> {
        Some(&mut self.poll_control)
    }
}

const POLL_CONTROL_CONFIG: PollControlConfig = PollControlConfig {
    check_in_interval: 40,
    long_poll_interval: 8,
    short_poll_interval: 1,
    fast_poll_timeout: 8,
    check_in_interval_min: 0,
    long_poll_interval_min: 4,
    fast_poll_timeout_max: 40,
    long_poll_interval_max: 40,
    check_in_response_timeout: 4,
};

fn poll_control_frame(command_id: u8, payload: &[u8], frame_counter: u32) -> MacFrame {
    let frame_control = ZclFrameHeader::build_frame_control(
        ZclFrameType::ClusterSpecific,
        false,
        ClusterDirection::ClientToServer,
        true,
    );
    zcl_data_frame(
        ClusterId::POLL_CONTROL,
        frame_control,
        frame_counter as u8,
        command_id,
        payload,
        frame_counter,
    )
}

/// The Poll Control server drives the joined cadence: its LongPollInterval
/// and ShortPollInterval replace the policy's, a Check-in opens the response
/// window, Check-in Response starts the requested fast poll, Fast Poll Stop
/// ends it, and SetLongPollInterval changes the slow cadence.
#[test]
fn poll_control_server_drives_check_in_and_fast_poll() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = DeviceProfile::new(
        1,
        0x0104,
        DeviceId::TEMPERATURE_SENSOR,
        PolledEnvironment {
            inner: TemperatureHumidityBattery::new(
                TemperatureRange {
                    min_centi_celsius: -4_000,
                    max_centi_celsius: 12_500,
                },
                BatteryDescriptor {
                    size: 4,
                    quantity: 2,
                    rated_voltage_100mv: 15,
                },
                EnvironmentalReporting::default(),
            ),
            poll_control: PollControlCluster::with_config(POLL_CONTROL_CONFIG),
        },
    );
    let mut device = test_device(&profile, &BASE_POLICY);
    let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
    let wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
    let mut app = SensorApp::new(
        node,
        &BASE_POLICY,
        parts(
            wake,
            RecordingStatus { events: &status },
            NoOta,
            &environment_samples,
            &battery_samples,
            &heartbeats,
            &diagnostics,
        ),
    )
    .unwrap();
    block_on(app.initialize()).unwrap();
    let step_until = |app: &mut SensorApp<'_, _, _, _, _>, wanted: u32, limit: usize| {
        for _ in 0..limit {
            block_on(app.step()).unwrap();
            if waits.borrow().last().map(|wait| wait.timeout_ms) == Some(wanted) {
                return;
            }
        }
        panic!("no {wanted} ms wait in {:?}", waits.borrow());
    };

    // Long poll 8 qs replaces the policy's 1 s slow poll.
    step_until(&mut app, 2_000, 20);
    let tx_before = app.device().mac().tx_history().len();
    // The 10 s check-in transmits and opens the 1 s response window at the
    // 1 qs short poll.
    step_until(&mut app, 250, 10);
    assert!(app.device().mac().tx_history().len() > tx_before);
    let opened = waits.borrow().len();
    step_until(&mut app, 2_000, 10);
    let window: u32 = waits.borrow()[opened - 1..waits.borrow().len() - 1]
        .iter()
        .map(|wait| wait.timeout_ms)
        .sum();
    assert_eq!(window, 1_000, "{:?}", waits.borrow());

    // Check-in Response: fast poll for 8 qs.
    app.device_mut()
        .mac_mut()
        .enqueue_poll_response(poll_control_frame(0x00, &[1, 8, 0], 0x40));
    step_until(&mut app, 250, 3);
    let started = waits.borrow().len();
    step_until(&mut app, 2_000, 20);
    let window: u32 = waits.borrow()[started - 1..waits.borrow().len() - 1]
        .iter()
        .map(|wait| wait.timeout_ms)
        .sum();
    assert!((1_750..=2_000).contains(&window), "{:?}", waits.borrow());

    // A long fast poll is cut short by Fast Poll Stop.
    app.device_mut()
        .mac_mut()
        .enqueue_poll_response(poll_control_frame(0x00, &[1, 40, 0], 0x41));
    step_until(&mut app, 250, 3);
    app.device_mut()
        .mac_mut()
        .enqueue_poll_response(poll_control_frame(0x01, &[], 0x42));
    step_until(&mut app, 2_000, 3);

    // SetLongPollInterval 3 s; 61 s is above the product maximum.
    app.device_mut()
        .mac_mut()
        .enqueue_poll_response(poll_control_frame(0x02, &12u32.to_le_bytes(), 0x43));
    step_until(&mut app, 3_000, 3);
    app.device_mut()
        .mac_mut()
        .enqueue_poll_response(poll_control_frame(0x02, &244u32.to_le_bytes(), 0x44));
    let refused = waits.borrow().len();
    for _ in 0..12 {
        block_on(app.step()).unwrap();
    }
    // Only the 3 s long poll and check-in response windows (250 ms) remain.
    assert!(
        waits.borrow()[refused..]
            .iter()
            .all(|wait| matches!(wait.timeout_ms, 250 | 3_000)),
        "{:?}",
        &waits.borrow()[refused..]
    );
}

// ---------------------------------------------------------------------------
// Keep-awake bounds and the anomaly record
// ---------------------------------------------------------------------------

/// OTA lifecycle whose session never finishes on its own (a server that
/// keeps answering WAIT_FOR_DATA, or a lost block response).
struct StuckOta<'a> {
    active: &'a Cell<bool>,
    aborts: &'a Cell<u32>,
}

impl<M, S, P> OtaLifecycle<M, S, P> for StuckOta<'_>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
{
    const ENABLED: bool = true;

    fn is_active(&self, _profile: &P) -> bool {
        self.active.get()
    }

    fn next_deadline_ms(&self, _profile: &P) -> Option<u32> {
        None
    }

    async fn handle_event(
        &mut self,
        _node: &mut ZigbeeNode<'_, M, S, P>,
        _event: &StackEvent,
    ) -> OtaEventOutcome {
        OtaEventOutcome::NotHandled
    }

    async fn service(
        &mut self,
        _node: &mut ZigbeeNode<'_, M, S, P>,
        _elapsed_secs: u16,
    ) -> OtaServiceOutcome {
        OtaServiceOutcome::IDLE
    }

    fn activate(&mut self, _node: &mut ZigbeeNode<'_, M, S, P>) -> OtaActivationOutcome {
        unreachable!("the stuck session never completes");
    }

    fn abort(&mut self, _node: &mut ZigbeeNode<'_, M, S, P>) {
        self.active.set(false);
        self.aborts.set(self.aborts.get() + 1);
    }
}

struct BoundRun {
    waits: Vec<WaitRequest>,
    /// Index of the first wait after `stimulus` ran.
    from: usize,
    diagnostics: Vec<DiagnosticEvent>,
    record: sensor_sed_app::AnomalyRecord,
    /// Records handed out by `take_anomaly_update`.
    updates: Vec<sensor_sed_app::AnomalyRecord>,
    joined: bool,
    commissioned: bool,
}

impl BoundRun {
    fn after(&self) -> &[WaitRequest] {
        &self.waits[self.from..]
    }

    fn anomalies(&self, reason: sensor_sed_app::AnomalyReason) -> usize {
        self.diagnostics
            .iter()
            .filter(
                |event| matches!(event, DiagnosticEvent::Anomaly { reason: r, .. } if *r == reason),
            )
            .count()
    }
}

/// Restore a commissioned sensor, let the restored fast window expire, run
/// `stimulus` once against the joined device, then step `steps` times while
/// `each_step` may feed the device.
fn run_bounded<O>(
    policy: &'static SensorPolicy,
    profile: TestProfile,
    ota: O,
    stimulus: impl FnOnce(&mut ZigbeeDevice<MockMac, EndDevice>),
    steps: usize,
    each_step: impl FnMut(usize, &mut ZigbeeDevice<MockMac, EndDevice>),
) -> BoundRun
where
    O: for<'s> OtaLifecycle<MockMac, CountingStore<'s>, TestProfile>,
{
    run_bounded_with(policy, profile, ota, stimulus, steps, each_step, None)
}

/// [`run_bounded`] whose non-active waits fail with `failure` while the
/// shared count is non-zero (set it from `stimulus` to start after warm-up).
fn run_bounded_with<O>(
    policy: &'static SensorPolicy,
    profile: TestProfile,
    ota: O,
    stimulus: impl FnOnce(&mut ZigbeeDevice<MockMac, EndDevice>),
    steps: usize,
    mut each_step: impl FnMut(usize, &mut ZigbeeDevice<MockMac, EndDevice>),
    failing_sleeps: Option<(&Cell<u32>, WaitFailure)>,
) -> BoundRun
where
    O: for<'s> OtaLifecycle<MockMac, CountingStore<'s>, TestProfile>,
{
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = profile;
    let mut device = test_device(&profile, policy);
    let from;
    let record;
    let mut updates = Vec::new();
    {
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let mut wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
        if let Some((count, failure)) = failing_sleeps {
            wake.failing_sleeps = Some(count);
            wake.failure = failure;
        }
        let mut app = SensorApp::new(
            node,
            policy,
            parts(
                wake,
                RecordingStatus { events: &status },
                ota,
                &environment_samples,
                &battery_samples,
                &heartbeats,
                &diagnostics,
            ),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        for _ in 0..20 {
            block_on(app.step()).unwrap();
            updates.extend(app.take_anomaly_update());
        }
        assert!(app.device().is_joined());
        stimulus(app.device_mut());
        from = waits.borrow().len();
        for step in 0..steps {
            each_step(step, app.device_mut());
            block_on(app.step()).unwrap();
            updates.extend(app.take_anomaly_update());
        }
        record = *app.anomalies();
    }
    let commissioned = store
        .load()
        .unwrap()
        .is_some_and(|state| state.commissioned);
    BoundRun {
        waits: waits.into_inner(),
        from,
        diagnostics: diagnostics.into_inner(),
        record,
        updates,
        joined: device.is_joined(),
        commissioned,
    }
}

static OTA_BOUND_POLICY: SensorPolicy = SensorPolicy {
    fast_sleep_depth: SleepDepth::Idle,
    recovery: sensor_sed_app::RecoveryPolicy {
        max_ota_awake_ms: 200,
        ..DETERMINISTIC_RECOVERY
    },
    ..BASE_POLICY
};

#[test]
fn a_stuck_ota_session_is_aborted_after_its_awake_limit() {
    let active = Cell::new(false);
    let aborts = Cell::new(0);
    let run = run_bounded(
        &OTA_BOUND_POLICY,
        test_profile(),
        StuckOta {
            active: &active,
            aborts: &aborts,
        },
        |_| active.set(true),
        60,
        |_, _| {},
    );
    let after = run.after();
    let awake: Vec<_> = after
        .iter()
        .take_while(|wait| wait.sleep_depth == SleepDepth::Active)
        .collect();
    assert!(!awake.is_empty(), "OTA first keeps the radio awake");
    assert!(
        awake.iter().map(|wait| wait.timeout_ms).sum::<u32>() <= 200 + 10,
        "but only for max_ota_awake_ms: {after:?}"
    );
    assert_eq!(aborts.get(), 1, "the stuck session is aborted once");
    assert!(!active.get());
    assert!(
        after[awake.len()..]
            .iter()
            .all(|wait| wait.sleep_depth != SleepDepth::Active)
    );
    assert!(
        after
            .iter()
            .rev()
            .take(5)
            .all(|wait| wait.timeout_ms == 1_000),
        "back on the slow poll: {after:?}"
    );
    assert_eq!(run.anomalies(sensor_sed_app::AnomalyReason::Ota), 1);
    assert_eq!(run.record.count(sensor_sed_app::AnomalyReason::Ota), 1);
    assert_eq!(run.record.total, 1);
    assert_eq!(
        run.updates,
        vec![run.record],
        "one durable update per anomaly"
    );
    assert!(run.joined, "an aborted OTA is not parent-loss evidence");
}

static IDENTIFY_BOUND_POLICY: SensorPolicy = SensorPolicy {
    recovery: sensor_sed_app::RecoveryPolicy {
        max_identify_awake_ms: 500,
        ..DETERMINISTIC_RECOVERY
    },
    ..BASE_POLICY
};

fn identify_frame(seconds: u16, frame_counter: u32) -> MacFrame {
    let frame_control = ZclFrameHeader::build_frame_control(
        ZclFrameType::ClusterSpecific,
        false,
        ClusterDirection::ClientToServer,
        true,
    );
    zcl_data_frame(
        ClusterId::IDENTIFY,
        frame_control,
        frame_counter as u8,
        0x00,
        &seconds.to_le_bytes(),
        frame_counter,
    )
}

#[test]
fn identify_holds_fast_poll_only_for_its_awake_limit() {
    let run = run_bounded(
        &IDENTIFY_BOUND_POLICY,
        test_profile(),
        NoOta,
        |device| {
            device
                .mac_mut()
                .enqueue_poll_response(identify_frame(600, 0x50))
        },
        200,
        |_, _| {},
    );
    let after = run.after();
    let fast = after
        .iter()
        .filter(|wait| wait.timeout_ms == BASE_POLICY.fast_poll_ms)
        .count();
    assert!(fast > 10, "Identify first polls fast: {after:?}");
    let fast_ms: u32 = after
        .iter()
        .filter(|wait| wait.timeout_ms == BASE_POLICY.fast_poll_ms)
        .map(|wait| wait.timeout_ms)
        .sum();
    assert!(fast_ms <= 500 + 2 * BASE_POLICY.fast_poll_ms, "{after:?}");
    assert!(
        after
            .iter()
            .rev()
            .take(20)
            .all(|wait| wait.timeout_ms == 1_000),
        "the device still identifies, but on the slow poll: {after:?}"
    );
    assert_eq!(run.anomalies(sensor_sed_app::AnomalyReason::Identify), 1);
    assert_eq!(run.record.count(sensor_sed_app::AnomalyReason::Identify), 1);
    assert!(run.joined);
}

#[test]
fn a_short_identify_never_reaches_its_limit() {
    let run = run_bounded(
        &IDENTIFY_BOUND_POLICY,
        test_profile(),
        NoOta,
        |device| {
            device
                .mac_mut()
                .enqueue_poll_response(identify_frame(0, 0x50))
        },
        50,
        |_, _| {},
    );
    assert_eq!(run.record.total, 0, "{:?}", run.diagnostics);
    assert!(run.updates.is_empty());
}

static FAST_WINDOW_BOUND_POLICY: SensorPolicy = SensorPolicy {
    recovery: sensor_sed_app::RecoveryPolicy {
        max_fast_poll_ms: 3_000,
        ..DETERMINISTIC_RECOVERY
    },
    ..BASE_POLICY
};

#[test]
fn a_requested_fast_poll_window_is_cut_at_its_limit() {
    let waits = RefCell::new(Vec::new());
    let poll_counts = RefCell::new(Vec::new());
    let delay_calls = Cell::new(0);
    let environment_samples = Cell::new(0);
    let battery_samples = Cell::new(0);
    let heartbeats = Cell::new(0);
    let diagnostics = RefCell::new(Vec::new());
    let status = RefCell::new(Vec::new());
    let store_calls = Cell::new(0);
    let mut store = CountingStore::with_state(commissioned_state(0x02), &store_calls);
    let mut profile = DeviceProfile::new(
        1,
        0x0104,
        DeviceId::TEMPERATURE_SENSOR,
        PolledEnvironment {
            inner: TemperatureHumidityBattery::new(
                TemperatureRange {
                    min_centi_celsius: -4_000,
                    max_centi_celsius: 12_500,
                },
                BatteryDescriptor {
                    size: 4,
                    quantity: 2,
                    rated_voltage_100mv: 15,
                },
                EnvironmentalReporting::default(),
            ),
            poll_control: PollControlCluster::with_config(PollControlConfig {
                // No check-in: only the requested window keeps the radio up.
                check_in_interval: 0,
                ..POLL_CONTROL_CONFIG
            }),
        },
    );
    let mut device = test_device(&profile, &FAST_WINDOW_BOUND_POLICY);
    let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
    let wake = RecordingWake::new(0, Vec::new(), &waits, &poll_counts, &delay_calls);
    let mut app = SensorApp::new(
        node,
        &FAST_WINDOW_BOUND_POLICY,
        parts(
            wake,
            RecordingStatus { events: &status },
            NoOta,
            &environment_samples,
            &battery_samples,
            &heartbeats,
            &diagnostics,
        ),
    )
    .unwrap();
    block_on(app.initialize()).unwrap();
    for _ in 0..20 {
        block_on(app.step()).unwrap();
    }
    // Check-in Response asking for 40 qs (10 s) of fast poll.
    app.device_mut()
        .mac_mut()
        .enqueue_poll_response(poll_control_frame(0x00, &[1, 40, 0], 0x40));
    let from = waits.borrow().len();
    for _ in 0..80 {
        block_on(app.step()).unwrap();
    }
    let record = *app.anomalies();
    drop(app);
    let waits = waits.borrow().clone();
    let after = &waits[from..];
    let fast_ms: u32 = after
        .iter()
        .filter(|wait| wait.timeout_ms == 250)
        .map(|wait| wait.timeout_ms)
        .sum();
    assert!(
        (2_750..=3_500).contains(&fast_ms),
        "fast poll lasts max_fast_poll_ms, not 10 s: {fast_ms} {after:?}"
    );
    assert!(
        after
            .iter()
            .rev()
            .take(10)
            .all(|wait| wait.timeout_ms == 2_000)
    );
    assert_eq!(record.count(sensor_sed_app::AnomalyReason::FastPoll), 1);
    assert!(diagnostics.borrow().iter().any(|event| matches!(
        event,
        DiagnosticEvent::Anomaly {
            reason: sensor_sed_app::AnomalyReason::FastPoll,
            total: 1
        }
    )));
}

#[test]
fn restored_fast_window_ending_on_time_is_not_an_anomaly() {
    static POLICY: SensorPolicy = SensorPolicy {
        restored_fast_ms: 400,
        recovery: sensor_sed_app::RecoveryPolicy {
            max_fast_poll_ms: 400,
            max_identify_awake_ms: 400,
            max_ota_awake_ms: 400,
            max_active_ms: 400,
            ..DETERMINISTIC_RECOVERY
        },
        ..BASE_POLICY
    };
    let run = run_bounded(&POLICY, test_profile(), NoOta, |_| {}, 30, |_, _| {});
    assert_eq!(run.record.total, 0, "{:?}", run.diagnostics);
    assert!(run.updates.is_empty());
}

/// Temperature reported at least every 2 s; the rest never periodically.
fn fast_reporting_profile() -> TestProfile {
    DeviceProfile::new(
        1,
        0x0104,
        DeviceId::TEMPERATURE_SENSOR,
        TemperatureHumidityBattery::new(
            TemperatureRange {
                min_centi_celsius: -4_000,
                max_centi_celsius: 12_500,
            },
            BatteryDescriptor {
                size: 4,
                quantity: 2,
                rated_voltage_100mv: 15,
            },
            EnvironmentalReporting {
                temperature_min_secs: 1,
                temperature_max_secs: 2,
                humidity_max_secs: 0xFFFF,
                battery_max_secs: 0xFFFF,
                ..EnvironmentalReporting::default()
            },
        ),
    )
}

static MISSED_REPORT_POLICY: SensorPolicy = SensorPolicy {
    secure_rejoin_failure_limit: 0,
    join_retry_ms: 6_000,
    recovery: sensor_sed_app::RecoveryPolicy {
        report_aps_ack: true,
        parent_loss_strikes: 2,
        missed_report_grace_ms: 1_000,
        ..DETERMINISTIC_RECOVERY
    },
    ..BASE_POLICY
};

#[test]
fn reports_that_are_never_acknowledged_count_as_parent_loss() {
    // The parent still ACKs polls at the MAC level, but nothing ever
    // returns end to end.
    let run = run_bounded(
        &MISSED_REPORT_POLICY,
        fast_reporting_profile(),
        NoOta,
        |_| {},
        2_000,
        |_, _| {},
    );
    assert!(
        run.anomalies(sensor_sed_app::AnomalyReason::MissedReport) >= 1,
        "{:?}",
        run.diagnostics
    );
    assert!(
        run.diagnostics
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::ParentLossSuspected { .. })),
        "{:?}",
        run.diagnostics
    );
    assert!(run.record.count(sensor_sed_app::AnomalyReason::ParentLoss) >= 1);
    assert!(!run.joined, "the staged rejoin started");
    assert!(run.commissioned, "the network is kept");
}

#[test]
fn acknowledged_traffic_is_proof_and_never_a_missed_report() {
    let mut counter = 0x10u8;
    let run = run_bounded(
        &MISSED_REPORT_POLICY,
        fast_reporting_profile(),
        NoOta,
        |_| {},
        60,
        |step, device| {
            let aps = device.bdb_mut().zdo_mut().aps_mut();
            // Every pending ACK (reports included) comes back promptly.
            if step % 2 == 0 {
                counter = counter.wrapping_add(1);
                aps.register_ack_pending(counter, 0x0000, &[0, 1, 2]);
            } else {
                for c in 0..=u8::MAX {
                    aps.confirm_ack(0x0000, c);
                }
            }
        },
    );
    assert_eq!(
        run.anomalies(sensor_sed_app::AnomalyReason::MissedReport),
        0,
        "{:?}",
        run.diagnostics
    );
    assert!(run.joined);
}

#[test]
fn pending_work_bound_and_restore_feed_the_anomaly_record() {
    static POLICY: SensorPolicy = SensorPolicy {
        fast_sleep_depth: SleepDepth::Idle,
        recovery: sensor_sed_app::RecoveryPolicy {
            parent_loss_strikes: 0,
            max_active_ms: 50,
            ..DETERMINISTIC_RECOVERY
        },
        ..BASE_POLICY
    };
    let run = run_bounded(
        &POLICY,
        test_profile(),
        NoOta,
        |device| {
            device
                .bdb_mut()
                .zdo_mut()
                .aps_mut()
                .register_ack_pending(0x77, 0x0000, &[0, 1, 2]);
        },
        20,
        |_, _| {},
    );
    assert_eq!(run.anomalies(sensor_sed_app::AnomalyReason::PendingWork), 1);
    assert_eq!(run.updates.len(), 1);
    let record = run.updates[0];
    let bytes: [u8; sensor_sed_app::ANOMALY_RECORD_LEN] = record.encode();
    assert_eq!(sensor_sed_app::AnomalyRecord::decode(&bytes), Some(record));
}

static SLEEP_FAILURE_POLICY: SensorPolicy = SensorPolicy {
    slow_sleep_depth: SleepDepth::Idle,
    fast_sleep_depth: SleepDepth::Idle,
    recovery: sensor_sed_app::RecoveryPolicy {
        sleep_failure_limit: 3,
        ..DETERMINISTIC_RECOVERY
    },
    ..BASE_POLICY
};

const RECOVERABLE: WaitFailure = WaitFailure {
    recoverable: true,
    code: 0x0400_0003,
};

#[test]
fn isolated_sleep_failures_finish_the_iteration_awake_without_reset() {
    let failing = Cell::new(0);
    let run = run_bounded_with(
        &SLEEP_FAILURE_POLICY,
        test_profile(),
        NoOta,
        |_| failing.set(2),
        10,
        |_, _| {},
        Some((&failing, RECOVERABLE)),
    );
    let after = run.after();
    // Each failed sleep is retried as an active wait of the same length.
    assert_eq!(after[0].sleep_depth, SleepDepth::Idle);
    assert_eq!(after[1].sleep_depth, SleepDepth::Active);
    assert_eq!(after[1].timeout_ms, after[0].timeout_ms);
    assert_eq!(after[2].sleep_depth, SleepDepth::Idle);
    assert_eq!(after[3].sleep_depth, SleepDepth::Active);
    // Below the limit sleep resumes and nothing is recorded.
    assert!(after[4..].iter().all(|w| w.sleep_depth == SleepDepth::Idle));
    assert_eq!(
        run.anomalies(sensor_sed_app::AnomalyReason::SleepDisabled),
        0
    );
    assert_eq!(
        run.diagnostics
            .iter()
            .filter(|e| matches!(e, DiagnosticEvent::WakeFailed))
            .count(),
        2
    );
    assert!(run.joined);
}

#[test]
fn consecutive_sleep_failures_disable_sleep_and_record_the_error() {
    let failing = Cell::new(0);
    let run = run_bounded_with(
        &SLEEP_FAILURE_POLICY,
        test_profile(),
        NoOta,
        |_| failing.set(u32::MAX),
        10,
        |_, _| {},
        Some((&failing, RECOVERABLE)),
    );
    let after = run.after();
    // Three failed attempts, each followed by an active retry; afterwards
    // every wait is active and the device keeps running.
    let idle = after
        .iter()
        .filter(|w| w.sleep_depth == SleepDepth::Idle)
        .count();
    assert_eq!(idle, 3);
    assert!(
        after[6..]
            .iter()
            .all(|w| w.sleep_depth == SleepDepth::Active)
    );
    assert_eq!(
        run.anomalies(sensor_sed_app::AnomalyReason::SleepDisabled),
        1
    );
    assert_eq!(run.updates.len(), 1);
    assert_eq!(
        run.record.last,
        Some(sensor_sed_app::AnomalyReason::SleepDisabled)
    );
    assert_eq!(
        run.record.last_cause,
        sensor_sed_app::ResetCause::wake(RECOVERABLE.code).word()
    );
    assert!(run.joined);
}

#[test]
#[should_panic(expected = "reset with cause 0x17000000")]
fn unrecoverable_sleep_failure_resets_with_its_code() {
    let failing = Cell::new(0);
    run_bounded_with(
        &SLEEP_FAILURE_POLICY,
        test_profile(),
        NoOta,
        |_| failing.set(1),
        4,
        |_, _| {},
        Some((
            &failing,
            WaitFailure {
                recoverable: false,
                code: 0x0700_0000,
            },
        )),
    );
}

#[test]
#[should_panic(expected = "reset with cause 0x14000003")]
fn a_zero_failure_limit_keeps_the_legacy_reset() {
    static POLICY: SensorPolicy = SensorPolicy {
        recovery: sensor_sed_app::RecoveryPolicy {
            sleep_failure_limit: 0,
            ..DETERMINISTIC_RECOVERY
        },
        ..SLEEP_FAILURE_POLICY
    };
    let failing = Cell::new(0);
    run_bounded_with(
        &POLICY,
        test_profile(),
        NoOta,
        |_| failing.set(1),
        4,
        |_, _| {},
        Some((&failing, RECOVERABLE)),
    );
}
