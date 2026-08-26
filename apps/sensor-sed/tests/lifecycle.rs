use core::future::Future;
use core::task::{Context, Poll};
use std::cell::{Cell, RefCell};
use std::sync::Arc;
use std::task::{Wake, Waker};

use sensor_sed_app::{
    BatterySource, ButtonPolicy, DiagnosticEvent, Diagnostics, EnvironmentReading,
    EnvironmentSource, NoOta, NoStatus, NoUserAction, OtaActivationOutcome, OtaEventOutcome,
    OtaLifecycle, OtaServiceOutcome, SensorApp, SensorLifecycleError, SensorPolicy, SensorSedParts,
    SensorStatus, SleepDepth, StatusPolicy, StatusSink, Supervisor, WaitRequest, WakeController,
    WakeReason,
};
use zigbee_aps::frames::{ApsCommandId, ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};
use zigbee_aps::security::{
    ApsSecurity, ApsSecurityHeader, KEY_ID_DATA_KEY, KEY_ID_KEY_TRANSPORT, SEC_LEVEL_ENC_MIC_32,
    derive_key_transport_key,
};
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::{
    AssociationStatus, MacFrame, MlmeAssociateConfirm, PanDescriptor, SuperframeSpec,
    ZigbeeBeaconPayload,
};
use zigbee_mac::{MacDriver, PlatformServices};
use zigbee_nwk::frames::{LeaveCommand, NwkCommandId, NwkFrameControl, NwkFrameType, NwkHeader};
use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{
    ApplicationProfile, BatteryDescriptor, DeviceProfile, EnvironmentalReporting,
    TemperatureHumidityBattery, TemperatureRange,
};
use zigbee_runtime::role::EndDevice;
use zigbee_runtime::security_store::{
    PersistentSecurityState, RamSecurityStateStore, SecurityStateStore, SecurityStoreError,
};
use zigbee_types::{MacAddress, PanId, ShortAddress};
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

struct RecordingWake<'a> {
    now: u32,
    batches: Vec<Vec<MacFrame>>,
    next_batch: usize,
    waits: &'a RefCell<Vec<WaitRequest>>,
    poll_counts_at_wait: &'a RefCell<Vec<u32>>,
    delay_calls: &'a Cell<u32>,
}

impl<'a> RecordingWake<'a> {
    fn new(
        now: u32,
        batches: Vec<Vec<MacFrame>>,
        waits: &'a RefCell<Vec<WaitRequest>>,
        poll_counts_at_wait: &'a RefCell<Vec<u32>>,
        delay_calls: &'a Cell<u32>,
    ) -> Self {
        Self {
            now,
            batches,
            next_batch: 0,
            waits,
            poll_counts_at_wait,
            delay_calls,
        }
    }
}

impl WakeController<MockMac> for RecordingWake<'_> {
    type Mark = u32;
    type Error = ();

    fn mark(&self) -> Self::Mark {
        self.now
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark.wrapping_add(duration_ms)
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later.wrapping_sub(earlier)
    }

    async fn wait(
        &mut self,
        mac: &mut MockMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        self.waits.borrow_mut().push(request);
        self.poll_counts_at_wait.borrow_mut().push(mac.poll_count());
        if self.next_batch < self.batches.len() {
            let batch = core::mem::take(&mut self.batches[self.next_batch]);
            for frame in batch {
                mac.enqueue_poll_response(frame);
            }
        }
        self.next_batch = self.next_batch.saturating_add(1);
        self.now = Self::add_ms(self.now, request.timeout_ms);
        mac.delay_micros(request.timeout_ms.saturating_mul(1_000))
            .await;
        Ok(WakeReason::Timer)
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
}

struct CountingStore<'a> {
    inner: RamSecurityStateStore,
    store_calls: &'a Cell<u32>,
    load_calls: Option<&'a Cell<u32>>,
}

impl<'a> CountingStore<'a> {
    fn empty(store_calls: &'a Cell<u32>) -> Self {
        Self {
            inner: RamSecurityStateStore::new(),
            store_calls,
            load_calls: None,
        }
    }

    fn with_state(state: PersistentSecurityState, store_calls: &'a Cell<u32>) -> Self {
        let mut inner = RamSecurityStateStore::new();
        inner.store(&state).expect("seed security state");
        Self {
            inner,
            store_calls,
            load_calls: None,
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
        self.inner.load()
    }

    fn store(&mut self, state: &PersistentSecurityState) -> Result<(), SecurityStoreError> {
        self.store_calls
            .set(self.store_calls.get().saturating_add(1));
        self.inner.store(state)
    }
}

struct RecordingOta<'a> {
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
                keep_awake_ms: Some(25),
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

fn test_device(profile: &TestProfile, policy: &SensorPolicy) -> ZigbeeDevice<MockMac, EndDevice> {
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
        actions: NoUserAction,
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

#[test]
fn reporting_completion_uses_short_grace_then_the_slow_wait_depth() {
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
        let wake = RecordingWake::new(0, vec![reports], &waits, &poll_counts, &delay_calls);
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

#[test]
fn raw_ota_command_is_handled_before_generic_matching_and_activation_is_checkpointed() {
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
            sleep_depth: SleepDepth::Active,
        }]
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
