use core::future::Future;
use core::mem::size_of;
use core::task::{Context, Poll, Waker};
use std::sync::Mutex;

use router_app::{
    AlwaysOnEndDeviceApp, CoordinatorApp, DiagnosticEvent, Diagnostics, NoChildren, NoDiagnostics,
    NoObserver, NoStatus, NodeArchetype, ParentRouterApp, PersistentChildren, RelayRouterApp,
    RouterAppError, RouterObserver, RouterParts, RouterPolicy, RouterStatus, StatusSink,
    Supervisor,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_aps::frames::{ApsCommandId, ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};
use zigbee_aps::security::{
    ApsSecurity, ApsSecurityHeader, KEY_ID_KEY_TRANSPORT, SEC_LEVEL_ENC_MIC_32,
    derive_key_transport_key,
};
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::{
    AssociationStatus, MacFrame, McpsDataIndication, MlmeAssociateConfirm, PanDescriptor,
    SuperframeSpec, ZigbeeBeaconPayload,
};
use zigbee_mac::{EdValue, MacDriver, PlatformServices};
use zigbee_nwk::DeviceType;
use zigbee_nwk::frames::{NwkFrameControl, NwkFrameType, NwkHeader};
use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};
use zigbee_runtime::UserAction;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::child_store::{
    ChildStoreError, ChildTableStore, PersistentChild, PersistentChildTable,
};
use zigbee_runtime::event_loop::{StackEvent, StartError};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::{EndDevice, RelayRouter, Router};
use zigbee_runtime::security_store::{
    PersistentSecurityState, RamSecurityStateStore, SecurityStateStore, SecurityStoreError,
};
use zigbee_types::{ChannelMask, MacAddress, PanId, ShortAddress};
use zigbee_zcl::clusters::basic::CMD_RESET_TO_FACTORY_DEFAULTS;
use zigbee_zcl::frame::ZclFrame;
use zigbee_zcl::{ClusterDirection, DeviceId};

const LOCAL_IEEE: [u8; 8] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
const COORDINATOR_IEEE: [u8; 8] = [0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x00, 0x01];
const EXTENDED_PAN_ID: [u8; 8] = [0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE];
const PAN_ID: u16 = 0x1A62;
const SHORT_ADDRESS: u16 = 0x3344;
const CHANNEL: u8 = 15;
const NETWORK_KEY: [u8; 16] = [0x42; 16];

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 3,
};

static REJOIN_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 2,
};

static RUN_AGAIN_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 200_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 3,
};

static FAST_RETRY_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 40,
    join_retry_max_ms: 80,
    secure_rejoin_failure_limit: 3,
};

type TestProfile = DeviceProfile<RangeExtender>;

fn block_on<F: Future>(future: F) -> F::Output {
    let mut context = Context::from_waker(Waker::noop());
    let mut future = std::pin::pin!(future);
    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        std::thread::yield_now();
    }
}

fn profile() -> TestProfile {
    DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::RANGE_EXTENDER,
        RangeExtender,
    )
}

fn endpoint_builder<M: MacDriver>(
    mac: M,
    profile: &mut TestProfile,
) -> zigbee_runtime::builder::DeviceBuilder<M> {
    ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
}

fn relay_device(profile: &mut TestProfile) -> ZigbeeDevice<MockMac, RelayRouter> {
    let mut mac = MockMac::new(LOCAL_IEEE);
    mac.set_rx_delay_us(u32::MAX);
    endpoint_builder(mac, profile).build_relay()
}

fn always_on_end_device(profile: &mut TestProfile) -> ZigbeeDevice<MockMac, EndDevice> {
    let mut mac = MockMac::new(LOCAL_IEEE);
    mac.set_rx_delay_us(u32::MAX);
    endpoint_builder(mac, profile)
        .device_type(DeviceType::EndDevice)
        .build()
}

fn parent_device(profile: &mut TestProfile) -> ZigbeeDevice<MockMac, Router> {
    let mut mac = MockMac::new(LOCAL_IEEE);
    mac.set_rx_delay_us(u32::MAX);
    endpoint_builder(mac, profile).build_router()
}

fn coordinator_device(profile: &mut TestProfile) -> ZigbeeDevice<MockMac, Router> {
    let mut mac = MockMac::new(LOCAL_IEEE);
    mac.set_rx_delay_us(u32::MAX);
    mac.add_energy(EdValue {
        channel: 11,
        energy: 90,
    });
    mac.add_energy(EdValue {
        channel: 15,
        energy: 20,
    });
    endpoint_builder(mac, profile)
        .channels(ChannelMask((1 << 11) | (1 << 15)))
        .build_coordinator()
}

fn commissioned_router_state(rejoin_pending: bool) -> PersistentSecurityState {
    let mut state = PersistentSecurityState::empty();
    state.commissioned = true;
    state.extended_pan_id = EXTENDED_PAN_ID;
    state.pan_id = PAN_ID;
    state.short_address = SHORT_ADDRESS;
    state.ieee_address = LOCAL_IEEE;
    state.channel = CHANNEL;
    state.depth = 1;
    state.parent_address = 0x0000;
    state.update_id = 0;
    state.update_id_valid = true;
    state.network_key = NETWORK_KEY;
    state.key_sequence = 0;
    state.global_counter_limit = 0x400;
    state.tclk_present = true;
    state.trust_center_address = COORDINATOR_IEEE;
    state.trust_center_link_key = [0x5A; 16];
    state.tclk_counter_limit = 0x400;
    state.rejoin_pending = rejoin_pending;
    state
}

fn commissioned_coordinator_state() -> PersistentSecurityState {
    let mut state = PersistentSecurityState::empty();
    state.commissioned = true;
    state.extended_pan_id = EXTENDED_PAN_ID;
    state.pan_id = PAN_ID;
    state.short_address = ShortAddress::COORDINATOR.0;
    state.ieee_address = LOCAL_IEEE;
    state.channel = CHANNEL;
    state.depth = 0;
    state.parent_address = 0xFFFF;
    state.update_id_valid = true;
    state.network_key = NETWORK_KEY;
    state.global_counter_limit = 0x800;
    state.tclk_counter_limit = 0x600;
    state
}

fn join_beacon() -> PanDescriptor {
    PanDescriptor {
        channel: CHANNEL,
        coord_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        superframe_spec: SuperframeSpec {
            association_permit: true,
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
        .unwrap();
    aps[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    let aps_len = aad_len + encrypted.len();

    let header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            ..Default::default()
        },
        dst_addr: ShortAddress(SHORT_ADDRESS),
        src_addr: ShortAddress::COORDINATOR,
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
    MacFrame::from_slice(&bytes[..header_len + aps_len]).unwrap()
}

fn basic_reset_frame() -> MacFrame {
    let zcl = ZclFrame::new_cluster_specific(
        0x42,
        CMD_RESET_TO_FACTORY_DEFAULTS,
        ClusterDirection::ClientToServer,
        true,
    );
    let mut zcl_bytes = [0u8; 16];
    let zcl_len = zcl.serialize(&mut zcl_bytes).unwrap();

    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_request: true,
            ..Default::default()
        },
        dst_endpoint: Some(1),
        cluster_id: Some(zigbee_zcl::ClusterId::BASIC.0),
        profile_id: Some(PROFILE_HOME_AUTOMATION),
        src_endpoint: Some(1),
        aps_counter: 2,
        ..Default::default()
    };
    let mut aps = [0u8; 64];
    let aps_header_len = aps_header.serialize(&mut aps);
    aps[aps_header_len..aps_header_len + zcl_len].copy_from_slice(&zcl_bytes[..zcl_len]);
    let aps_len = aps_header_len + zcl_len;

    let nwk_header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            security: true,
            ..Default::default()
        },
        dst_addr: ShortAddress(SHORT_ADDRESS),
        src_addr: ShortAddress::COORDINATOR,
        radius: 5,
        seq_number: 2,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    let mut bytes = [0u8; 128];
    let nwk_header_len = nwk_header.serialize(&mut bytes);
    let security_header = NwkSecurityHeader {
        security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
        frame_counter: 2,
        source_address: COORDINATOR_IEEE,
        key_seq_number: 0,
    };
    let security_header_len = security_header.serialize(&mut bytes[nwk_header_len..]);
    let aad_len = nwk_header_len + security_header_len;
    let encrypted = NwkSecurity::new()
        .encrypt(
            &bytes[..aad_len],
            &aps[..aps_len],
            &NETWORK_KEY,
            &security_header,
        )
        .unwrap();
    bytes[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    bytes[nwk_header_len] &= !0x07;
    MacFrame::from_slice(&bytes[..aad_len + encrypted.len()]).unwrap()
}

fn script_fresh_router_join(device: &mut ZigbeeDevice<MockMac, Router>) {
    let mac = device.bdb_mut().zdo_mut().aps_mut().nwk_mut().mac_mut();
    mac.add_beacon(join_beacon());
    mac.set_associate_response(MlmeAssociateConfirm {
        short_address: ShortAddress(SHORT_ADDRESS),
        status: AssociationStatus::Success,
    });
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: transport_key_frame(),
        security_use: false,
    });
}

fn security_store(rejoin_pending: bool) -> RamSecurityStateStore {
    let mut store = RamSecurityStateStore::new();
    store
        .store(&commissioned_router_state(rejoin_pending))
        .unwrap();
    store
}

#[derive(Debug, Default)]
struct RecordingStatus {
    events: Vec<RouterStatus>,
}

impl StatusSink for RecordingStatus {
    fn set(&mut self, status: RouterStatus) {
        self.events.push(status);
    }
}

#[derive(Debug, Default)]
struct TestSupervisor {
    heartbeats: u32,
    max_wait_ms: Option<u32>,
}

impl Supervisor for TestSupervisor {
    fn heartbeat(&mut self) {
        self.heartbeats = self.heartbeats.wrapping_add(1);
    }

    fn max_wait_ms(&self) -> Option<u32> {
        self.max_wait_ms
    }

    fn reset(&mut self) -> ! {
        panic!("unexpected supervisor reset")
    }
}

#[derive(Debug, Default)]
struct RecordingDiagnostics {
    events: Vec<DiagnosticEvent>,
}

impl Diagnostics for RecordingDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        self.events.push(event);
    }
}

#[derive(Debug, Default)]
struct CountingChildStore {
    table: Option<PersistentChildTable>,
    loads: u32,
    stores: u32,
}

impl CountingChildStore {
    fn with_table(table: PersistentChildTable) -> Self {
        Self {
            table: Some(table),
            loads: 0,
            stores: 0,
        }
    }
}

impl ChildTableStore for CountingChildStore {
    fn load(&mut self) -> Result<Option<PersistentChildTable>, ChildStoreError> {
        self.loads = self.loads.wrapping_add(1);
        Ok(self.table.clone())
    }

    fn store(&mut self, table: &PersistentChildTable) -> Result<(), ChildStoreError> {
        self.stores = self.stores.wrapping_add(1);
        self.table = Some(table.clone());
        Ok(())
    }
}

static RELAY_OBSERVER_COUNTS: Mutex<(u32, u32, u32)> = Mutex::new((0, 0, 0));

struct RelayMetricsObserver;

impl RouterObserver<MockMac, RelayRouter> for RelayMetricsObserver {
    fn on_commissioning_attempt(
        _device: &ZigbeeDevice<MockMac, RelayRouter>,
        _attempt: u32,
        _started_us: u32,
    ) {
        RELAY_OBSERVER_COUNTS.lock().unwrap().0 += 1;
    }

    fn on_network_ready(_device: &ZigbeeDevice<MockMac, RelayRouter>) {
        RELAY_OBSERVER_COUNTS.lock().unwrap().1 += 1;
    }

    fn on_before_receive(_device: &ZigbeeDevice<MockMac, RelayRouter>, _timeout_us: u32) {
        RELAY_OBSERVER_COUNTS.lock().unwrap().2 += 1;
    }
}

static PARENT_ORDER: Mutex<Vec<&'static str>> = Mutex::new(Vec::new());
static URGENT_RESET_ORDER: Mutex<Vec<&'static str>> = Mutex::new(Vec::new());

#[derive(Debug)]
struct OrderedChildStore {
    inner: CountingChildStore,
}

impl ChildTableStore for OrderedChildStore {
    fn load(&mut self) -> Result<Option<PersistentChildTable>, ChildStoreError> {
        PARENT_ORDER.lock().unwrap().push("restore");
        self.inner.load()
    }

    fn store(&mut self, table: &PersistentChildTable) -> Result<(), ChildStoreError> {
        self.inner.store(table)
    }
}

struct ParentOrderingObserver;

impl RouterObserver<MockMac, Router> for ParentOrderingObserver {
    fn on_before_receive(_device: &ZigbeeDevice<MockMac, Router>, _timeout_us: u32) {
        PARENT_ORDER.lock().unwrap().push("receive");
    }
}

#[derive(Debug, Default)]
struct OrderedResetSecurityStore {
    state: Option<PersistentSecurityState>,
}

impl SecurityStateStore for OrderedResetSecurityStore {
    fn load(&mut self) -> Result<Option<PersistentSecurityState>, SecurityStoreError> {
        Ok(self.state)
    }

    fn store(&mut self, state: &PersistentSecurityState) -> Result<(), SecurityStoreError> {
        if !state.commissioned {
            URGENT_RESET_ORDER.lock().unwrap().push("security-reset");
        }
        self.state = Some(*state);
        Ok(())
    }
}

#[derive(Debug, Default)]
struct OrderedResetChildStore {
    table: Option<PersistentChildTable>,
}

impl ChildTableStore for OrderedResetChildStore {
    fn load(&mut self) -> Result<Option<PersistentChildTable>, ChildStoreError> {
        Ok(self.table.clone())
    }

    fn store(&mut self, table: &PersistentChildTable) -> Result<(), ChildStoreError> {
        if table.is_empty() {
            URGENT_RESET_ORDER.lock().unwrap().push("child-clear");
        }
        self.table = Some(table.clone());
        Ok(())
    }
}

struct UrgentResetObserver;

impl RouterObserver<MockMac, Router> for UrgentResetObserver {
    fn on_commissioning_attempt(
        _device: &ZigbeeDevice<MockMac, Router>,
        _attempt: u32,
        _started_us: u32,
    ) {
        URGENT_RESET_ORDER.lock().unwrap().push("start");
    }

    fn on_urgent_factory_reset_result(
        _device: &ZigbeeDevice<MockMac, Router>,
        result: Result<(), RouterAppError>,
    ) {
        URGENT_RESET_ORDER.lock().unwrap().push(if result.is_ok() {
            "reset-observer"
        } else {
            "reset-error"
        });
    }
}

#[test]
fn relay_is_zero_child_forwarding_frontend_with_bounded_receive_and_static_observer() {
    assert_eq!(size_of::<NoChildren>(), 0);
    *RELAY_OBSERVER_COUNTS.lock().unwrap() = (0, 0, 0);

    let mut profile = profile();
    let mut device = relay_device(&mut profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(
        RecordingStatus::default(),
        TestSupervisor::default(),
        RecordingDiagnostics::default(),
    );
    let mut app = RelayRouterApp::<_, _, _, _, _, _, RelayMetricsObserver>::new_observed(
        node, NoChildren, &POLICY, parts,
    )
    .unwrap();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::NotInitialized)
    ));
    block_on(app.initialize()).unwrap();
    assert_eq!(
        block_on(app.initialize()),
        Err(RouterAppError::AlreadyInitialized)
    );
    assert!(app.node().device().is_joined());
    assert_eq!(app.node().device().device_type(), DeviceType::Router);
    assert_eq!(*RELAY_OBSERVER_COUNTS.lock().unwrap(), (1, 1, 0));

    let before = app.node().device().mac().monotonic_micros();
    let events = block_on(app.step()).unwrap();
    let after = app.node().device().mac().monotonic_micros();
    assert!(events.is_empty());
    assert_eq!(after.wrapping_sub(before), POLICY.max_receive_slice_us);
    assert_eq!(*RELAY_OBSERVER_COUNTS.lock().unwrap(), (1, 1, 1));
    assert!(app.parts().status.events.iter().any(|status| matches!(
        status,
        RouterStatus::Online {
            archetype: NodeArchetype::RelayRouter,
            ..
        }
    )));
    assert!(app.parts().supervisor.heartbeats >= 2);
}

#[test]
fn always_on_end_device_is_a_non_routing_rx_on_leaf_with_reset_and_rejoin_lifecycle() {
    let mut profile = profile();
    let mut device = always_on_end_device(&mut profile);
    assert_eq!(device.device_type(), DeviceType::EndDevice);
    assert!(!device.is_sleepy());
    assert!(
        device.rx_on_when_idle(),
        "PowerMode::AlwaysOn must advertise macRxOnWhenIdle"
    );

    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(
        RecordingStatus::default(),
        TestSupervisor::default(),
        RecordingDiagnostics::default(),
    );
    let mut app = AlwaysOnEndDeviceApp::new(node, &POLICY, parts).unwrap();

    block_on(app.initialize()).unwrap();
    assert!(app.node().device().is_joined());
    assert_eq!(app.node().device().device_type(), DeviceType::EndDevice);
    assert!(app.node().device().rx_on_when_idle());
    assert!(app.parts().status.events.iter().any(|status| matches!(
        status,
        RouterStatus::Online {
            archetype: NodeArchetype::AlwaysOnEndDevice,
            ..
        }
    )));

    block_on(app.urgent_factory_reset_and_recommission()).unwrap();
    assert!(!app.node().device().is_joined());
    assert!(
        !app.node_mut()
            .load_security_state()
            .unwrap()
            .unwrap()
            .commissioned
    );

    block_on(app.step()).unwrap();
    assert_eq!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .filter(|event| matches!(event, DiagnosticEvent::CommissioningAttempt { .. }))
            .count(),
        2,
        "the frontend must restart Network Steering after a local reset"
    );
}

#[test]
fn always_on_end_device_keeps_a_persisted_secure_rejoin_pending() {
    let mut profile = profile();
    let mut device = always_on_end_device(&mut profile);
    let mut security = security_store(true);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(
        NoStatus,
        TestSupervisor::default(),
        RecordingDiagnostics::default(),
    );
    let mut app = AlwaysOnEndDeviceApp::new(node, &REJOIN_POLICY, parts).unwrap();

    block_on(app.initialize()).unwrap();
    assert!(!app.node().device().is_joined());
    assert!(app.node().device().secure_rejoin_pending());
    assert!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .any(|event| { matches!(event, DiagnosticEvent::SecureRejoinPending { failures: 1 }) })
    );
}

#[test]
fn public_frontends_reject_role_mismatches_and_sleepy_routing_devices() {
    {
        let mut profile = profile();
        let mut device = coordinator_device(&mut profile);
        let mut security = RamSecurityStateStore::new();
        let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
        let result = ParentRouterApp::<_, _, _, _, _, _, _, NoObserver>::new(
            node,
            PersistentChildren::new(CountingChildStore::default()),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        );
        assert!(matches!(
            result,
            Err(RouterAppError::WrongDeviceType {
                expected: DeviceType::Router,
                actual: DeviceType::Coordinator,
            })
        ));
    }

    {
        let mut profile = profile();
        let mac = MockMac::new(LOCAL_IEEE);
        let mut device = ZigbeeDevice::builder(mac)
            .power_mode(PowerMode::Sleepy {
                poll_interval_ms: 1_000,
                wake_duration_ms: 100,
            })
            .endpoint(
                profile.endpoint(),
                profile.profile_id(),
                profile.device_id(),
                |endpoint| profile.configure_endpoint(endpoint),
            )
            .build_relay();
        let mut security = RamSecurityStateStore::new();
        let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
        let result = RelayRouterApp::<_, _, _, _, _, _, NoObserver>::new(
            node,
            NoChildren,
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        );
        assert!(matches!(result, Err(RouterAppError::NotAlwaysOnDevice)));
    }
}

#[test]
fn always_on_end_device_rejects_sleepy_leaf_construction() {
    let mut profile = profile();
    let mut device = ZigbeeDevice::builder(MockMac::new(LOCAL_IEEE))
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 1_000,
            wake_duration_ms: 100,
        })
        .device_type(DeviceType::EndDevice)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build();
    let mut security = RamSecurityStateStore::new();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let result = AlwaysOnEndDeviceApp::new(
        node,
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    );
    assert!(matches!(result, Err(RouterAppError::NotAlwaysOnDevice)));
}

#[test]
fn parent_frontend_rejects_coordinator_persistence_without_consuming_counters() {
    let original = commissioned_coordinator_state();
    original.validate().unwrap();
    let mut security = RamSecurityStateStore::new();
    security.store(&original).unwrap();
    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    assert_eq!(
        block_on(app.initialize()),
        Err(RouterAppError::Start(StartError::PersistenceFailed(
            SecurityStoreError::Corrupt
        )))
    );
    drop(app);
    assert_eq!(
        security.load().unwrap(),
        Some(original),
        "typed parent startup must reject coordinator state before reserving counters"
    );
}

#[test]
fn failed_fresh_commissioning_retries_with_bounded_exponential_delay() {
    let mut profile = profile();
    let mut device = relay_device(&mut profile);
    let mut security = RamSecurityStateStore::new();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(
        NoStatus,
        TestSupervisor::default(),
        RecordingDiagnostics::default(),
    );
    let mut app = RelayRouterApp::new(node, NoChildren, &FAST_RETRY_POLICY, parts).unwrap();

    block_on(app.initialize()).unwrap();
    assert!(!app.node().device().is_joined());
    assert_eq!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .filter(|event| matches!(event, DiagnosticEvent::CommissioningAttempt { .. }))
            .count(),
        1
    );

    assert!(block_on(app.step()).unwrap().is_empty());
    assert!(block_on(app.step()).unwrap().is_empty());
    assert!(block_on(app.step()).unwrap().is_empty());
    assert_eq!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .filter(|event| matches!(event, DiagnosticEvent::CommissioningAttempt { .. }))
            .count(),
        2
    );
    assert!(app.parts().diagnostics.events.iter().any(|event| {
        matches!(
            event,
            DiagnosticEvent::RetryScheduled {
                attempt: 3,
                delay_ms: 80
            }
        )
    }));
}

#[test]
fn basic_reset_preserves_parent_network_and_child_state() {
    let mut child_table = PersistentChildTable::new(EXTENDED_PAN_ID);
    child_table
        .push(PersistentChild {
            ieee_address: [0x66; 8],
            short_address: 0x5678,
            rx_on_when_idle: false,
            security_capable: true,
            is_router: false,
            end_device_timeout: 8,
        })
        .unwrap();
    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = ParentRouterApp::<_, _, _, _, _, _, _, NoObserver>::new(
        node,
        PersistentChildren::new(CountingChildStore::with_table(child_table)),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert!(app.node().device().is_joined());
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: basic_reset_frame(),
        security_use: false,
    });

    let events = block_on(app.step()).unwrap();

    assert!(
        matches!(
            events.incoming,
            Some(StackEvent::BasicResetToFactoryDefaults)
        ),
        "{events:?}"
    );
    assert!(app.node().device().is_joined());
    assert!(
        app.node_mut()
            .load_security_state()
            .unwrap()
            .unwrap()
            .commissioned
    );
    assert_eq!(app.children().store().stores, 0);
    assert_eq!(app.children().store().table.as_ref().unwrap().len(), 1);
}

#[test]
fn urgent_parent_reset_preempts_due_retry_and_clears_journals_before_steering() {
    URGENT_RESET_ORDER.lock().unwrap().clear();

    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let mut security = OrderedResetSecurityStore::default();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(
        RecordingStatus::default(),
        TestSupervisor::default(),
        RecordingDiagnostics::default(),
    );
    let mut app = ParentRouterApp::<_, _, _, _, _, _, _, UrgentResetObserver>::new_observed(
        node,
        PersistentChildren::new(OrderedResetChildStore::default()),
        &FAST_RETRY_POLICY,
        parts,
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert!(!app.node().device().is_joined());
    assert_eq!(&*URGENT_RESET_ORDER.lock().unwrap(), &["start"]);

    URGENT_RESET_ORDER.lock().unwrap().clear();
    app.parts_mut().status.events.clear();
    app.parts_mut().diagnostics.events.clear();
    block_on(
        app.node_mut()
            .device_mut()
            .mac_mut()
            .delay_micros(FAST_RETRY_POLICY.join_retry_initial_ms * 1_000),
    );

    block_on(app.urgent_factory_reset_and_recommission()).unwrap();

    assert_eq!(
        &*URGENT_RESET_ORDER.lock().unwrap(),
        &["security-reset", "child-clear", "reset-observer"],
        "the due retry must not enter steering while the urgent reset is running"
    );
    assert!(!app.node().device().is_joined());
    assert!(
        !app.node_mut()
            .load_security_state()
            .unwrap()
            .unwrap()
            .commissioned
    );
    assert_eq!(
        app.parts().status.events,
        [
            RouterStatus::Resetting {
                archetype: NodeArchetype::ParentRouter,
            },
            RouterStatus::Recommissioning {
                archetype: NodeArchetype::ParentRouter,
                attempt: 2,
                retry_in_ms: 0,
            },
        ]
    );
    assert_eq!(
        app.parts().diagnostics.events,
        [
            DiagnosticEvent::FactoryReset,
            DiagnosticEvent::ChildTableCleared,
            DiagnosticEvent::RetryScheduled {
                attempt: 2,
                delay_ms: 0,
            },
        ]
    );

    assert!(block_on(app.step()).unwrap().is_empty());
    assert_eq!(
        &*URGENT_RESET_ORDER.lock().unwrap(),
        &["security-reset", "child-clear", "reset-observer", "start"],
        "fresh steering may begin only on the subsequent step"
    );
}

#[test]
fn parent_restores_children_before_receive_and_does_not_rewrite_clean_table() {
    PARENT_ORDER.lock().unwrap().clear();
    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(PersistentChild {
            ieee_address: [0x22; 8],
            short_address: 0x4567,
            rx_on_when_idle: false,
            security_capable: true,
            is_router: false,
            end_device_timeout: 8,
        })
        .unwrap();

    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let child_store = OrderedChildStore {
        inner: CountingChildStore::with_table(table),
    };
    let parts = RouterParts::new(
        NoStatus,
        TestSupervisor::default(),
        RecordingDiagnostics::default(),
    );
    let mut app = ParentRouterApp::<_, _, _, _, _, _, _, ParentOrderingObserver>::new_observed(
        node,
        PersistentChildren::new(child_store),
        &POLICY,
        parts,
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert_eq!(&*PARENT_ORDER.lock().unwrap(), &["restore"]);
    assert!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .any(|event| { matches!(event, DiagnosticEvent::ChildrenRestored { count: 1 }) })
    );

    block_on(app.step()).unwrap();
    assert_eq!(&*PARENT_ORDER.lock().unwrap(), &["restore", "receive"]);
    assert_eq!(app.children().store().inner.stores, 0);
}

#[test]
fn parent_persists_only_when_dirty_and_clears_children_before_recommission() {
    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        &POLICY,
        parts,
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert_eq!(app.children().store().loads, 1);
    assert_eq!(app.children().store().stores, 0);

    block_on(app.step()).unwrap();
    assert_eq!(
        app.children().store().stores,
        1,
        "the initially absent snapshot is committed once"
    );
    block_on(app.step()).unwrap();
    assert_eq!(
        app.children().store().stores,
        1,
        "an unchanged child table writes no flash"
    );

    app.node_mut()
        .device_mut()
        .user_action(UserAction::FactoryReset);
    let events = block_on(app.step()).unwrap();
    assert!(matches!(events.tick, Some(StackEvent::Left)));
    assert_eq!(
        app.children().store().stores,
        2,
        "recommissioning explicitly replaces the durable child table"
    );
    assert!(app.children().store().table.as_ref().unwrap().is_empty());
    assert!(!app.node().device().is_joined());
}

#[test]
fn deferred_parent_reset_preserves_journals_until_the_caller_commits() {
    let mut child_table = PersistentChildTable::new(EXTENDED_PAN_ID);
    child_table
        .push(PersistentChild {
            ieee_address: [0x77; 8],
            short_address: 0x6789,
            rx_on_when_idle: false,
            security_capable: true,
            is_router: false,
            end_device_timeout: 8,
        })
        .unwrap();
    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(CountingChildStore::with_table(child_table)),
        &POLICY,
        parts,
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    app.node_mut()
        .device_mut()
        .user_action(UserAction::FactoryReset);

    let events = block_on(app.step_deferred_factory_reset()).unwrap();
    assert!(matches!(events.tick, Some(StackEvent::Left)));
    assert!(app.factory_reset_pending());
    assert!(app.node().device().is_joined());
    assert!(
        app.node_mut()
            .load_security_state()
            .unwrap()
            .unwrap()
            .commissioned
    );
    assert_eq!(app.children().store().stores, 0);

    block_on(app.complete_pending_factory_reset_and_recommission()).unwrap();
    assert!(!app.factory_reset_pending());
    assert!(!app.node().device().is_joined());
    assert_eq!(app.children().store().stores, 1);
    assert!(app.children().store().table.as_ref().unwrap().is_empty());
}

#[test]
fn runtime_run_again_shortens_the_next_monotonic_receive_window() {
    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    script_fresh_router_join(&mut device);
    let mut security = RamSecurityStateStore::new();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(
        NoStatus,
        TestSupervisor::default(),
        RecordingDiagnostics::default(),
    );
    let mut app = ParentRouterApp::<_, _, _, _, _, _, _, NoObserver>::new(
        node,
        PersistentChildren::new(CountingChildStore::with_table(
            PersistentChildTable::default(),
        )),
        &RUN_AGAIN_POLICY,
        parts,
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert!(app.node().device().is_joined());

    block_on(app.step()).unwrap();
    assert!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .any(|event| { matches!(event, DiagnosticEvent::RunAgain { delay_ms: 50 }) })
    );

    let before = app.node().device().mac().monotonic_micros();
    block_on(app.step()).unwrap();
    let after = app.node().device().mac().monotonic_micros();
    assert_eq!(
        after.wrapping_sub(before),
        50_000,
        "RunAgain(50) must preempt the 200 ms default receive slice"
    );
}

#[test]
fn coordinator_frontend_forms_then_restarts_the_same_persisted_pan() {
    let mut security = RamSecurityStateStore::new();

    {
        let mut stale_children = PersistentChildTable::new(EXTENDED_PAN_ID);
        stale_children
            .push(PersistentChild {
                ieee_address: [0x77; 8],
                short_address: 0x6789,
                rx_on_when_idle: false,
                security_capable: true,
                is_router: false,
                end_device_timeout: 8,
            })
            .unwrap();
        let mut profile = profile();
        let mut device = coordinator_device(&mut profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
        let parts = RouterParts::new(
            RecordingStatus::default(),
            TestSupervisor::default(),
            RecordingDiagnostics::default(),
        );
        let mut app = CoordinatorApp::<_, _, _, _, _, _, _, NoObserver>::new(
            node,
            PersistentChildren::new(CountingChildStore::with_table(stale_children)),
            &POLICY,
            parts,
        )
        .unwrap();

        block_on(app.initialize()).unwrap();
        assert_eq!(app.node().device().device_type(), DeviceType::Coordinator);
        assert_eq!(app.node().device().short_address(), 0x0000);
        assert_eq!(app.node().device().channel(), 15);
        assert_eq!(app.children().store().stores, 1);
        assert!(app.children().store().table.as_ref().unwrap().is_empty());
        assert!(app.parts().status.events.iter().any(|status| {
            matches!(
                status,
                RouterStatus::Online {
                    archetype: NodeArchetype::Coordinator,
                    short_address: 0,
                    ..
                }
            )
        }));
    }

    let formed = security.load().unwrap().unwrap();
    assert!(formed.commissioned);
    let formed_key = formed.network_key;
    let formed_pan = formed.pan_id;
    let formed_epid = formed.extended_pan_id;

    {
        let mut profile = profile();
        let mut device = coordinator_device(&mut profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
        let parts = RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics);
        let mut app = CoordinatorApp::<_, _, _, _, _, _, _, NoObserver>::new(
            node,
            PersistentChildren::new(CountingChildStore::with_table(
                PersistentChildTable::default(),
            )),
            &POLICY,
            parts,
        )
        .unwrap();

        block_on(app.initialize()).unwrap();
        assert_eq!(app.node().device().short_address(), 0x0000);
        assert_eq!(app.node().device().pan_id(), formed_pan);
        assert_eq!(app.node().device().channel(), formed.channel);
    }

    let restarted = security.load().unwrap().unwrap();
    assert_eq!(restarted.pan_id, formed_pan);
    assert_eq!(restarted.extended_pan_id, formed_epid);
    assert_eq!(restarted.network_key, formed_key);
    assert!(restarted.global_counter_limit >= formed.global_counter_limit);
}

#[test]
fn repeated_secure_rejoin_failure_resets_and_clears_parent_state() {
    let mut child_table = PersistentChildTable::new(EXTENDED_PAN_ID);
    child_table
        .push(PersistentChild {
            ieee_address: [0x66; 8],
            short_address: 0x5678,
            rx_on_when_idle: false,
            security_capable: true,
            is_router: false,
            end_device_timeout: 8,
        })
        .unwrap();
    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let mut security = security_store(true);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let parts = RouterParts::new(
        NoStatus,
        TestSupervisor::default(),
        RecordingDiagnostics::default(),
    );
    let mut app = ParentRouterApp::<_, _, _, _, _, _, _, NoObserver>::new(
        node,
        PersistentChildren::new(CountingChildStore::with_table(child_table)),
        &REJOIN_POLICY,
        parts,
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert!(!app.node().device().is_joined());
    assert!(app.node().device().secure_rejoin_pending());
    assert!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .any(|event| { matches!(event, DiagnosticEvent::SecureRejoinPending { failures: 1 }) })
    );

    for _ in 0..4 {
        assert!(block_on(app.step()).unwrap().is_empty());
        assert_eq!(
            app.children().store().stores,
            0,
            "a pending secured rejoin must retain the durable child snapshot"
        );
        assert_eq!(app.children().store().table.as_ref().unwrap().len(), 1);
    }
    let events = block_on(app.step()).unwrap();
    assert!(matches!(
        events.tick,
        Some(StackEvent::CommissioningComplete { success: false })
    ));
    assert!(!app.node().device().secure_rejoin_pending());
    assert_eq!(
        app.children().store().stores,
        1,
        "only the reset-time clear may replace the retained snapshot"
    );
    assert!(app.children().store().table.as_ref().unwrap().is_empty());
    assert!(app.parts().diagnostics.events.iter().any(|event| {
        matches!(
            event,
            DiagnosticEvent::SecureRejoinLimitReached { failures: 2 }
        )
    }));
}
