use core::future::Future;
use core::mem::size_of;
use core::task::{Context, Poll, Waker};
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::Mutex;

#[cfg(feature = "trust-center")]
use router_app::TrustCenterCoordinatorApp;
use router_app::{
    AlwaysOnEndDeviceApp, CoordinatorApp, DiagnosticEvent, Diagnostics, DistributedRouterApp,
    NoChildren, NoDiagnostics, NoObserver, NoStatus, NodeArchetype, ParentRouterApp,
    PersistentApsTables, PersistentChildren, RelayRouterApp, RouterAppError, RouterObserver,
    RouterParts, RouterPolicy, RouterStatus, StatusSink, Supervisor,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
#[cfg(feature = "trust-center")]
use zigbee_aps::apsme::ApsUpdateDeviceStatus;
use zigbee_aps::binding::{BindingEntry, BindingTable};
use zigbee_aps::frames::{ApsCommandId, ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};
use zigbee_aps::group::GroupTable;
use zigbee_aps::security::{
    ApsKeyOrigin, ApsKeyType, ApsLinkKeyEntry, ApsReplayCounter, ApsSecurity, ApsSecurityHeader,
    DISTRIBUTED_SECURITY_TEST_LINK_KEY, KEY_ID_DATA_KEY, KEY_ID_KEY_LOAD, KEY_ID_KEY_TRANSPORT,
    SEC_LEVEL_ENC_MIC_32, derive_key_load_key, derive_key_transport_key,
};
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::{
    AssociationStatus, MacFrame, McpsDataIndication, MlmeAssociateConfirm,
    MlmeDataRequestIndication, PanDescriptor, SuperframeSpec, ZigbeeBeaconPayload,
};
use zigbee_mac::{EdValue, MacCommandEvent, MacDriver, PlatformServices};
use zigbee_nwk::DeviceType;
use zigbee_nwk::frames::{
    LeaveCommand, NetworkStatusCommand, NwkCommandId, NwkFrameControl, NwkFrameType, NwkHeader,
};
use zigbee_nwk::security::{NwkReplayCounter, NwkSecurity, NwkSecurityHeader};
use zigbee_runtime::UserAction;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::aps_table_store::{
    ApsTableStore, PersistentApsTables as ApsTableSnapshot, RamApsTableStore,
};
use zigbee_runtime::child_store::{
    ChildStoreError, ChildTableStore, PersistentChild, PersistentChildTable,
};
use zigbee_runtime::event_loop::{StackEvent, StartError};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::{EndDevice, RelayRouter, Router};
use zigbee_runtime::security_store::{
    PersistentReplayCounter, PersistentSecurityState, RamSecurityStateStore, SecurityStateStore,
    SecurityStoreError,
};
#[cfg(feature = "trust-center")]
use zigbee_runtime::trust_center_runtime::TrustCenterRuntimeError;
#[cfg(feature = "trust-center")]
use zigbee_runtime::trust_center_store::{
    PersistentTrustCenterState, RamTrustCenterDeviceStore, TrustCenterDeviceStore,
    TrustCenterStoreError,
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

fn distributed_router_device(profile: &mut TestProfile) -> ZigbeeDevice<MockMac, Router> {
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
        .build_router()
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

fn secured_nwk_frame(
    frame_type: NwkFrameType,
    source_short: ShortAddress,
    source_ieee: [u8; 8],
    destination: ShortAddress,
    sequence: u8,
    frame_counter: u32,
    payload: &[u8],
) -> MacFrame {
    let header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: frame_type as u8,
            protocol_version: 0x02,
            security: true,
            ..Default::default()
        },
        dst_addr: destination,
        src_addr: source_short,
        radius: 5,
        seq_number: sequence,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    let mut bytes = [0u8; 128];
    let header_len = header.serialize(&mut bytes);
    let security_header = NwkSecurityHeader {
        security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
        frame_counter,
        source_address: source_ieee,
        key_seq_number: 0,
    };
    let security_len = security_header.serialize(&mut bytes[header_len..]);
    let aad_len = header_len + security_len;
    let encrypted = NwkSecurity::new()
        .encrypt(&bytes[..aad_len], payload, &NETWORK_KEY, &security_header)
        .unwrap();
    bytes[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    bytes[header_len] &= !0x07;
    MacFrame::from_slice(&bytes[..aad_len + encrypted.len()]).unwrap()
}

fn secured_aps_command_frame(
    source_short: ShortAddress,
    source_ieee: [u8; 8],
    destination: ShortAddress,
    link_key: [u8; 16],
    counters: (u8, u32, u32),
    command: &[u8],
) -> MacFrame {
    let (aps_counter, aps_frame_counter, nwk_frame_counter) = counters;
    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Command as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            security: true,
            ack_request: true,
            ..Default::default()
        },
        aps_counter,
        ..Default::default()
    };
    let security_header = ApsSecurityHeader {
        security_control: (KEY_ID_DATA_KEY << 3) | (1 << 5),
        frame_counter: aps_frame_counter,
        source_address: Some(source_ieee),
        key_seq_number: None,
    };
    let mut aps = [0u8; 96];
    let header_len = aps_header.serialize(&mut aps);
    let security_len = security_header.serialize(&mut aps[header_len..]);
    let aad_len = header_len + security_len;
    let mut authenticated_header = [0u8; 16];
    authenticated_header[..aad_len].copy_from_slice(&aps[..aad_len]);
    authenticated_header[header_len] |= SEC_LEVEL_ENC_MIC_32;
    let encrypted = ApsSecurity::new()
        .encrypt(
            &authenticated_header[..aad_len],
            command,
            &link_key,
            &security_header,
        )
        .unwrap();
    aps[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    secured_nwk_frame(
        NwkFrameType::Data,
        source_short,
        source_ieee,
        destination,
        nwk_frame_counter as u8,
        nwk_frame_counter,
        &aps[..aad_len + encrypted.len()],
    )
}

fn remove_device_frame(
    child: [u8; 8],
    aps_counter: u8,
    aps_frame_counter: u32,
    nwk_frame_counter: u32,
) -> MacFrame {
    let mut command = [0u8; 9];
    command[0] = ApsCommandId::RemoveDevice as u8;
    command[1..].copy_from_slice(&child);
    secured_aps_command_frame(
        ShortAddress::COORDINATOR,
        COORDINATOR_IEEE,
        ShortAddress(SHORT_ADDRESS),
        [0x5A; 16],
        (aps_counter, aps_frame_counter, nwk_frame_counter),
        &command,
    )
}

#[cfg(feature = "trust-center")]
fn update_device_frame(
    source_short: ShortAddress,
    source_ieee: [u8; 8],
    link_key: [u8; 16],
    device_address: [u8; 8],
    device_short_address: ShortAddress,
    counter: u32,
    status: ApsUpdateDeviceStatus,
) -> MacFrame {
    let mut command = [0u8; 12];
    command[0] = ApsCommandId::UpdateDevice as u8;
    command[1..9].copy_from_slice(&device_address);
    command[9..11].copy_from_slice(&device_short_address.0.to_le_bytes());
    command[11] = status as u8;
    secured_aps_command_frame(
        source_short,
        source_ieee,
        ShortAddress::COORDINATOR,
        link_key,
        (counter as u8, counter, counter),
        &command,
    )
}

fn child_leave_frame_to(
    child_short: u16,
    child_ieee: [u8; 8],
    destination: ShortAddress,
    counter: u32,
) -> MacFrame {
    let command = [
        NwkCommandId::Leave as u8,
        LeaveCommand {
            remove_children: false,
            request: false,
            rejoin: false,
        }
        .serialize(),
    ];
    secured_nwk_frame(
        NwkFrameType::Command,
        ShortAddress(child_short),
        child_ieee,
        destination,
        counter as u8,
        counter,
        &command,
    )
}

fn child_leave_frame(child_short: u16, child_ieee: [u8; 8], counter: u32) -> MacFrame {
    child_leave_frame_to(
        child_short,
        child_ieee,
        ShortAddress(SHORT_ADDRESS),
        counter,
    )
}

fn remove_children_frame(rejoin: bool, counter: u32) -> MacFrame {
    let command = [
        NwkCommandId::Leave as u8,
        LeaveCommand {
            remove_children: true,
            request: true,
            rejoin,
        }
        .serialize(),
    ];
    secured_nwk_frame(
        NwkFrameType::Command,
        ShortAddress::COORDINATOR,
        COORDINATOR_IEEE,
        ShortAddress(SHORT_ADDRESS),
        counter as u8,
        counter,
        &command,
    )
}

fn child_conflict_frame(child_short: u16, counter: u32) -> MacFrame {
    let command = [
        NwkCommandId::NetworkStatus as u8,
        NetworkStatusCommand::ADDRESS_CONFLICT,
        child_short as u8,
        (child_short >> 8) as u8,
    ];
    secured_nwk_frame(
        NwkFrameType::Command,
        ShortAddress::COORDINATOR,
        COORDINATOR_IEEE,
        ShortAddress(SHORT_ADDRESS),
        counter as u8,
        counter,
        &command,
    )
}

fn data_request(child_short: u16) -> MacCommandEvent {
    MacCommandEvent::DataRequest(MlmeDataRequestIndication {
        source_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(child_short)),
        destination_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 170,
        security_use: false,
    })
}

fn outbound_nwk_command_count(mac: &MockMac, command: NwkCommandId) -> usize {
    mac.tx_history()
        .iter()
        .filter(|record| {
            let Some((header, payload)) = decrypt_outbound_nwk(&record.payload) else {
                return false;
            };
            if header.frame_control.frame_type != NwkFrameType::Command as u8 {
                return false;
            }
            payload.first().copied() == Some(command as u8)
        })
        .count()
}

fn decrypt_outbound_nwk(frame: &MacFrame) -> Option<(NwkHeader, Vec<u8>)> {
    let bytes = frame.as_slice();
    let (header, header_len) = NwkHeader::parse(bytes)?;
    if !header.frame_control.security {
        return Some((header, bytes[header_len..].to_vec()));
    }
    let (security, security_len) = NwkSecurityHeader::parse(&bytes[header_len..])?;
    let aad_len = header_len + security_len;
    let mut aad = [0u8; 64];
    aad[..aad_len].copy_from_slice(&bytes[..aad_len]);
    aad[header_len] = (aad[header_len] & !0x07) | 0x05;
    let payload =
        NwkSecurity::new().decrypt(&aad[..aad_len], &bytes[aad_len..], &NETWORK_KEY, &security)?;
    Some((header, payload.to_vec()))
}

fn outbound_aps_ack_count(mac: &MockMac) -> usize {
    mac.tx_history()
        .iter()
        .filter(|record| {
            let Some((header, payload)) = decrypt_outbound_nwk(&record.payload) else {
                return false;
            };
            if header.frame_control.frame_type != NwkFrameType::Data as u8 {
                return false;
            }
            ApsHeader::parse(&payload).is_some_and(|(header, _)| {
                header.frame_control.frame_type == ApsFrameType::Ack as u8
            })
        })
        .count()
}

fn outbound_unacked_aps_command_counters(mac: &MockMac, command: ApsCommandId) -> Vec<u8> {
    assert!(
        outbound_aps_command_counters(mac, command, true).is_empty(),
        "outgoing security commands must not request APS ACKs"
    );
    outbound_aps_command_counters(mac, command, false)
}

fn outbound_aps_command_counters(
    mac: &MockMac,
    command: ApsCommandId,
    ack_request: bool,
) -> Vec<u8> {
    mac.tx_history()
        .iter()
        .filter_map(|record| {
            let (nwk, payload) = decrypt_outbound_nwk(&record.payload)?;
            if nwk.frame_control.frame_type != NwkFrameType::Data as u8 {
                return None;
            }
            let (header, header_len) = ApsHeader::parse(&payload)?;
            if header.frame_control.frame_type != ApsFrameType::Command as u8
                || header.frame_control.ack_request != ack_request
            {
                return None;
            }
            let command_id = if header.frame_control.security {
                let (security, security_len) = ApsSecurityHeader::parse(&payload[header_len..])?;
                let aad_len = header_len + security_len;
                let mut aad = [0u8; 32];
                aad[..aad_len].copy_from_slice(&payload[..aad_len]);
                aad[header_len] = (aad[header_len] & !0x07) | SEC_LEVEL_ENC_MIC_32;
                ApsSecurity::new()
                    .decrypt(&aad[..aad_len], &payload[aad_len..], &[0x5A; 16], &security)?
                    .first()
                    .copied()
            } else {
                payload.get(header_len).copied()
            };
            (command_id == Some(command as u8)).then_some(header.aps_counter)
        })
        .collect()
}

fn application_transport_key_frame(
    partner: [u8; 8],
    application_key: [u8; 16],
    nwk_frame_counter: u32,
) -> MacFrame {
    let mut command = [0u8; 27];
    command[0] = ApsCommandId::TransportKey as u8;
    command[1] = 0x03;
    command[2..18].copy_from_slice(&application_key);
    command[18..26].copy_from_slice(&partner);
    command[26] = 1;

    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Command as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            security: true,
            ack_request: true,
            ..Default::default()
        },
        aps_counter: 7,
        ..Default::default()
    };
    let security_header = ApsSecurityHeader {
        security_control: (KEY_ID_KEY_LOAD << 3) | (1 << 5),
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
    let key_load_key = derive_key_load_key(&[0x5A; 16]);
    let encrypted = ApsSecurity::new()
        .encrypt(
            &authenticated_header[..aad_len],
            &command,
            &key_load_key,
            &security_header,
        )
        .unwrap();
    aps[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    let aps_len = aad_len + encrypted.len();

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
        seq_number: nwk_frame_counter as u8,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    let mut bytes = [0u8; 128];
    let nwk_header_len = nwk_header.serialize(&mut bytes);
    let nwk_security_header = NwkSecurityHeader {
        security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
        frame_counter: nwk_frame_counter,
        source_address: COORDINATOR_IEEE,
        key_seq_number: 0,
    };
    let security_header_len = nwk_security_header.serialize(&mut bytes[nwk_header_len..]);
    let nwk_aad_len = nwk_header_len + security_header_len;
    let encrypted = NwkSecurity::new()
        .encrypt(
            &bytes[..nwk_aad_len],
            &aps[..aps_len],
            &NETWORK_KEY,
            &nwk_security_header,
        )
        .unwrap();
    bytes[nwk_aad_len..nwk_aad_len + encrypted.len()].copy_from_slice(&encrypted);
    bytes[nwk_header_len] &= !0x07;
    MacFrame::from_slice(&bytes[..nwk_aad_len + encrypted.len()]).unwrap()
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

fn persistent_child(
    ieee_address: [u8; 8],
    short_address: u16,
    rx_on_when_idle: bool,
) -> PersistentChild {
    PersistentChild {
        ieee_address,
        short_address,
        rx_on_when_idle,
        security_capable: true,
        is_router: false,
        end_device_timeout: 8,
        removal_pending: false,
        removal_attempts: 0,
        reassignment_address: None,
        departure_pending: false,
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

#[derive(Debug, Default)]
struct FaultChildStoreState {
    table: Option<PersistentChildTable>,
    store_attempts: u32,
    successful_stores: u32,
    fail_next_store: bool,
    fail_store_attempt: Option<u32>,
}

#[derive(Debug, Clone, Default)]
struct FaultChildStore {
    state: Rc<RefCell<FaultChildStoreState>>,
}

impl FaultChildStore {
    fn with_table(table: PersistentChildTable) -> Self {
        Self {
            state: Rc::new(RefCell::new(FaultChildStoreState {
                table: Some(table),
                ..Default::default()
            })),
        }
    }

    fn fail_next_store(&self) {
        self.state.borrow_mut().fail_next_store = true;
    }

    fn fail_store_after(&self, successful_stores_before_failure: u32) {
        let mut state = self.state.borrow_mut();
        state.fail_store_attempt = Some(
            state
                .store_attempts
                .wrapping_add(successful_stores_before_failure)
                .wrapping_add(1),
        );
    }

    fn table(&self) -> Option<PersistentChildTable> {
        self.state.borrow().table.clone()
    }

    fn store_attempts(&self) -> u32 {
        self.state.borrow().store_attempts
    }

    fn successful_stores(&self) -> u32 {
        self.state.borrow().successful_stores
    }
}

impl ChildTableStore for FaultChildStore {
    fn load(&mut self) -> Result<Option<PersistentChildTable>, ChildStoreError> {
        Ok(self.state.borrow().table.clone())
    }

    fn store(&mut self, table: &PersistentChildTable) -> Result<(), ChildStoreError> {
        let mut state = self.state.borrow_mut();
        state.store_attempts = state.store_attempts.wrapping_add(1);
        if state.fail_next_store || state.fail_store_attempt == Some(state.store_attempts) {
            state.fail_next_store = false;
            state.fail_store_attempt = None;
            return Err(ChildStoreError::Hardware);
        }
        state.successful_stores = state.successful_stores.wrapping_add(1);
        state.table = Some(table.clone());
        Ok(())
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

#[derive(Debug, Default)]
struct FailingApsStore {
    inner: RamApsTableStore,
    fail_next_store: bool,
}

impl ApsTableStore for FailingApsStore {
    fn load(
        &mut self,
    ) -> Result<Option<ApsTableSnapshot>, zigbee_runtime::aps_table_store::ApsTableStoreError> {
        self.inner.load()
    }

    fn store(
        &mut self,
        tables: &ApsTableSnapshot,
    ) -> Result<(), zigbee_runtime::aps_table_store::ApsTableStoreError> {
        if self.fail_next_store {
            self.fail_next_store = false;
            return Err(zigbee_runtime::aps_table_store::ApsTableStoreError::Hardware);
        }
        self.inner.store(tables)
    }
}

#[derive(Default)]
struct FailingApsReplayStore {
    inner: RamSecurityStateStore,
    fail_next_aps_replay: bool,
    fail_next_nwk_replay: bool,
    fail_next_tombstone: bool,
    fail_replay_visit: Option<Rc<Cell<bool>>>,
}

impl SecurityStateStore for FailingApsReplayStore {
    fn load(&mut self) -> Result<Option<PersistentSecurityState>, SecurityStoreError> {
        self.inner.load()
    }

    fn store(&mut self, state: &PersistentSecurityState) -> Result<(), SecurityStoreError> {
        self.inner.store(state)
    }

    fn visit_replay_counters(
        &mut self,
        visitor: &mut dyn FnMut(PersistentReplayCounter),
    ) -> Result<(), SecurityStoreError> {
        if self
            .fail_replay_visit
            .as_ref()
            .is_some_and(|fail| fail.get())
        {
            return Err(SecurityStoreError::Hardware);
        }
        self.inner.visit_replay_counters(visitor)
    }

    fn commit_replay_counter(
        &mut self,
        replay: PersistentReplayCounter,
    ) -> Result<(), SecurityStoreError> {
        if self.fail_next_aps_replay && matches!(replay, PersistentReplayCounter::Aps(_)) {
            self.fail_next_aps_replay = false;
            return Err(SecurityStoreError::Hardware);
        }
        if self.fail_next_nwk_replay && matches!(replay, PersistentReplayCounter::Nwk(_)) {
            self.fail_next_nwk_replay = false;
            return Err(SecurityStoreError::Hardware);
        }
        self.inner.commit_replay_counter(replay)
    }

    fn tombstone_replay_counters(
        &mut self,
        tombstone: zigbee_runtime::security_store::ReplayCounterTombstone,
    ) -> Result<(), SecurityStoreError> {
        if self.fail_next_tombstone {
            self.fail_next_tombstone = false;
            return Err(SecurityStoreError::Hardware);
        }
        self.inner.tombstone_replay_counters(tombstone)
    }
}

#[cfg(feature = "trust-center")]
#[derive(Debug, Default)]
struct FaultTrustCenterStoreState {
    state: Option<PersistentTrustCenterState>,
    store_attempts: u32,
    fail_next_store: bool,
}

#[cfg(feature = "trust-center")]
#[derive(Debug, Clone, Default)]
struct FaultTrustCenterStore {
    state: Rc<RefCell<FaultTrustCenterStoreState>>,
}

#[cfg(feature = "trust-center")]
impl FaultTrustCenterStore {
    fn fail_next_store(&self) {
        self.state.borrow_mut().fail_next_store = true;
    }

    fn state(&self) -> Option<PersistentTrustCenterState> {
        self.state.borrow().state.clone()
    }

    fn store_attempts(&self) -> u32 {
        self.state.borrow().store_attempts
    }
}

#[cfg(feature = "trust-center")]
impl TrustCenterDeviceStore for FaultTrustCenterStore {
    fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError> {
        Ok(self.state.borrow().state.clone())
    }

    fn store(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError> {
        state.validate()?;
        let mut fault = self.state.borrow_mut();
        fault.store_attempts = fault.store_attempts.wrapping_add(1);
        if fault.fail_next_store {
            fault.fail_next_store = false;
            return Err(TrustCenterStoreError::Hardware);
        }
        fault.state = Some(state.clone());
        Ok(())
    }

    fn clear(&mut self) -> Result<(), TrustCenterStoreError> {
        self.state.borrow_mut().state = None;
        Ok(())
    }
}

fn aps_replay_count(store: &mut impl SecurityStateStore) -> usize {
    let mut count = 0;
    store
        .visit_replay_counters(&mut |replay| {
            if matches!(replay, PersistentReplayCounter::Aps(_)) {
                count += 1;
            }
        })
        .unwrap();
    count
}

fn nwk_replay_count(store: &mut impl SecurityStateStore) -> usize {
    let mut count = 0;
    store
        .visit_replay_counters(&mut |replay| {
            if matches!(replay, PersistentReplayCounter::Nwk(_)) {
                count += 1;
            }
        })
        .unwrap();
    count
}

fn nwk_replay_sources(store: &mut impl SecurityStateStore) -> Vec<[u8; 8]> {
    let mut sources = Vec::new();
    store
        .visit_replay_counters(&mut |replay| {
            if let PersistentReplayCounter::Nwk(replay) = replay {
                sources.push(replay.source);
            }
        })
        .unwrap();
    sources
}

fn replay_counters(store: &mut impl SecurityStateStore) -> Vec<PersistentReplayCounter> {
    let mut replays = Vec::new();
    store
        .visit_replay_counters(&mut |replay| replays.push(replay))
        .unwrap();
    replays
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
fn distributed_frontend_fails_closed_without_a_provisioned_global_key() {
    let mut profile = profile();
    let mut device = distributed_router_device(&mut profile);
    let mut security = RamSecurityStateStore::new();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = DistributedRouterApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        &POLICY,
        RouterParts::new(
            NoStatus,
            TestSupervisor::default(),
            RecordingDiagnostics::default(),
        ),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();

    assert!(!app.node().device().is_joined());
    assert!(app.parts().diagnostics.events.iter().any(|event| {
        matches!(
            event,
            DiagnosticEvent::StartFailed {
                error: StartError::CommissioningFailed(_)
            }
        )
    }));
    drop(app);
    assert!(security.load().unwrap().is_none());
}

#[test]
fn distributed_frontend_forms_and_persists_a_router_owned_pan() {
    let mut profile = profile();
    let mut device = distributed_router_device(&mut profile);
    device.set_distributed_security_link_key(DISTRIBUTED_SECURITY_TEST_LINK_KEY);
    let mut security = RamSecurityStateStore::new();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = DistributedRouterApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        &POLICY,
        RouterParts::new(
            RecordingStatus::default(),
            TestSupervisor::default(),
            RecordingDiagnostics::default(),
        ),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();

    assert!(app.node().device().is_joined());
    assert_eq!(
        app.node().device().short_address(),
        ShortAddress::COORDINATOR.0
    );
    assert!(app.parts().status.events.iter().any(|status| {
        matches!(
            status,
            RouterStatus::Online {
                archetype: NodeArchetype::DistributedRouter,
                short_address: 0,
                ..
            }
        )
    }));
    drop(app);

    let persisted = security.load().unwrap().unwrap();
    assert_eq!(persisted.short_address, ShortAddress::COORDINATOR.0);
    assert_eq!(persisted.depth, 0);
    assert_eq!(persisted.parent_address, 0xFFFF);
    assert!(persisted.node_join_link_key_type.is_distributed());
    assert_eq!(persisted.trust_center_address, [0xFF; 8]);
}

#[test]
fn distributed_pending_join_action_cannot_fall_back_to_network_steering() {
    let mut profile = profile();
    let mut device = distributed_router_device(&mut profile);
    let mut security = RamSecurityStateStore::new();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = DistributedRouterApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert!(!app.node().device().is_joined());

    app.node_mut()
        .device_mut()
        .set_distributed_security_link_key(DISTRIBUTED_SECURITY_TEST_LINK_KEY);
    app.node_mut().device_mut().user_action(UserAction::Join);
    let events = block_on(app.step()).unwrap();

    assert!(matches!(
        events.tick,
        Some(StackEvent::Joined {
            short_address: 0,
            ..
        })
    ));
    assert!(app.node().device().is_joined());
    drop(app);
    let persisted = security.load().unwrap().unwrap();
    assert_eq!(persisted.short_address, ShortAddress::COORDINATOR.0);
    assert_eq!(persisted.depth, 0);
    assert_eq!(persisted.parent_address, 0xFFFF);
    assert!(persisted.node_join_link_key_type.is_distributed());
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
            removal_pending: false,
            removal_attempts: 0,
            reassignment_address: None,
            departure_pending: false,
        })
        .unwrap();
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
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

    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let mut security = OrderedResetSecurityStore::default();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
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
            removal_pending: false,
            removal_attempts: 0,
            reassignment_address: None,
            departure_pending: false,
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
fn parent_resumes_and_completes_a_durable_remove_device_transaction() {
    const CHILD_IEEE: [u8; 8] = [0x66; 8];
    const CHILD_SHORT: u16 = 0x5678;

    let mut child_table = PersistentChildTable::new(EXTENDED_PAN_ID);
    child_table
        .push(PersistentChild {
            ieee_address: CHILD_IEEE,
            short_address: CHILD_SHORT,
            rx_on_when_idle: true,
            security_capable: true,
            is_router: false,
            end_device_timeout: 8,
            removal_pending: true,
            removal_attempts: 0,
            reassignment_address: None,
            departure_pending: false,
        })
        .unwrap();
    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    device.mac_mut().set_tx_failures(1);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(CountingChildStore::with_table(child_table)),
        &POLICY,
        RouterParts::new(
            NoStatus,
            TestSupervisor::default(),
            RecordingDiagnostics::default(),
        ),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert_eq!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .known_child_by_ieee(&CHILD_IEEE),
        Some(ShortAddress(CHILD_SHORT))
    );

    block_on(app.step()).unwrap();
    assert!(app.parts().diagnostics.events.iter().any(|event| {
        matches!(
            event,
            DiagnosticEvent::ChildRemovalRetry {
                child_address: CHILD_IEEE,
                short_address: CHILD_SHORT,
                attempts: 1,
                ..
            }
        )
    }));
    assert!(
        app.children()
            .store()
            .table
            .as_ref()
            .unwrap()
            .child(&CHILD_IEEE)
            .unwrap()
            .removal_pending
    );

    block_on(app.step()).unwrap();
    assert!(app.parts().diagnostics.events.iter().any(|event| {
        matches!(
            event,
            DiagnosticEvent::ChildRemovalCompleted {
                child_address: CHILD_IEEE,
                short_address: CHILD_SHORT,
                attempts: 2,
                delivered: true,
            }
        )
    }));
    assert!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .known_child_by_ieee(&CHILD_IEEE)
            .is_none()
    );
    assert!(app.children().store().table.as_ref().unwrap().is_empty());
}

#[test]
fn secured_remove_device_store_failure_reboots_before_ack_or_child_leave() {
    const CHILD_IEEE: [u8; 8] = [0x61; 8];
    const CHILD_SHORT: u16 = 0x5611;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    child_store.fail_next_store();
    let mut security = security_store(false);

    {
        let mut test_profile = profile();
        let mut device = parent_device(&mut test_profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
        let mut app = ParentRouterApp::new(
            node,
            PersistentChildren::new(child_store.clone()),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        app.node_mut().device_mut().mac_mut().clear_tx_history();
        let mac = app.node_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
            lqi: 220,
            payload: remove_device_frame(CHILD_IEEE, 11, 11, 11),
            security_use: false,
        });

        assert!(matches!(
            block_on(app.step()),
            Err(RouterAppError::ChildStore(ChildStoreError::Hardware))
        ));
        assert!(
            app.node().device().mac().tx_history().is_empty(),
            "neither the APS ACK nor the child Leave may escape before the intent commit"
        );
    }

    assert!(
        !child_store
            .table()
            .unwrap()
            .child(&CHILD_IEEE)
            .unwrap()
            .removal_pending
    );
    assert_eq!(aps_replay_count(&mut security), 0);

    {
        let mut test_profile = profile();
        let mut device = parent_device(&mut test_profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
        let mut app = ParentRouterApp::new(
            node,
            PersistentChildren::new(child_store.clone()),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        app.node_mut().device_mut().mac_mut().clear_tx_history();
        let mac = app.node_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
            lqi: 220,
            payload: remove_device_frame(CHILD_IEEE, 11, 12, 12),
            security_use: false,
        });

        let events = block_on(app.step()).unwrap();
        assert!(matches!(
            events.incoming,
            Some(StackEvent::ApsSecurityIndication(
                zigbee_aps::apsme::ApsmeSecurityIndication::RemoveDevice(_)
            ))
        ));
        assert_eq!(
            outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
            0,
            "the durable APS replay/ACK completes before servicing the child transaction"
        );
        assert!(
            child_store
                .table()
                .unwrap()
                .child(&CHILD_IEEE)
                .unwrap()
                .removal_pending
        );

        app.node_mut()
            .device_mut()
            .mac_mut()
            .set_rx_delay_us(u32::MAX);
        block_on(app.step()).unwrap();
        assert_eq!(
            outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
            1
        );
        assert!(child_store.table().unwrap().is_empty());
    }

    assert_eq!(aps_replay_count(&mut security), 1);
    assert_eq!(child_store.store_attempts(), 3);
    assert_eq!(child_store.successful_stores(), 2);
}

#[test]
fn secured_remove_device_replay_failure_blocks_ack_and_child_leave_until_retry() {
    const CHILD_IEEE: [u8; 8] = [0x64; 8];
    const CHILD_SHORT: u16 = 0x5644;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = FailingApsReplayStore {
        inner: RamSecurityStateStore::new(),
        fail_next_aps_replay: true,
        ..Default::default()
    };
    security.store(&commissioned_router_state(false)).unwrap();
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: remove_device_frame(CHILD_IEEE, 41, 41, 41),
        security_use: false,
    });
    mac.clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::Node(
            zigbee_runtime::node::NodeError::Persistence(SecurityStoreError::Hardware)
        ))
    ));
    assert!(
        child_store
            .table()
            .unwrap()
            .child(&CHILD_IEEE)
            .unwrap()
            .removal_pending
    );
    assert_eq!(outbound_aps_ack_count(app.node().device().mac()), 0);
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
        0
    );

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    assert_eq!(outbound_aps_ack_count(app.node().device().mac()), 1);
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
        1
    );
    assert!(child_store.table().unwrap().is_empty());
    drop(app);
    assert_eq!(aps_replay_count(&mut security), 1);
}

#[test]
fn child_reassignment_store_failure_reboots_without_releasing_response() {
    const CHILD_IEEE: [u8; 8] = [0x62; 8];
    const CHILD_SHORT: u16 = 0x5622;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = security_store(false);

    {
        let mut test_profile = profile();
        let mut device = parent_device(&mut test_profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
        let mut app = ParentRouterApp::new(
            node,
            PersistentChildren::new(child_store.clone()),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap();

        block_on(app.initialize()).unwrap();
        child_store.fail_next_store();
        let mac = app.node_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
            lqi: 220,
            payload: child_conflict_frame(CHILD_SHORT, 21),
            security_use: false,
        });
        mac.clear_tx_history();

        assert!(matches!(
            block_on(app.step()),
            Err(RouterAppError::ChildStore(ChildStoreError::Hardware))
        ));
        assert_eq!(
            outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::RejoinResponse),
            0,
            "no Rejoin Response may precede the reassignment journal"
        );
        assert!(
            child_store
                .table()
                .unwrap()
                .pending_reassignment()
                .is_none()
        );
    }

    assert_eq!(nwk_replay_count(&mut security), 0);
    let mut rebooted_profile = profile();
    let mut rebooted_device = parent_device(&mut rebooted_profile);
    let rebooted_node = ZigbeeNode::new(&mut rebooted_device, &mut security, &mut rebooted_profile);
    let mut rebooted = ParentRouterApp::new(
        rebooted_node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(rebooted.initialize()).unwrap();
    let mac = rebooted.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: child_conflict_frame(CHILD_SHORT, 22),
        security_use: false,
    });
    mac.clear_tx_history();

    block_on(rebooted.step()).unwrap();
    assert_eq!(
        outbound_nwk_command_count(rebooted.node().device().mac(), NwkCommandId::RejoinResponse),
        1
    );
    let committed = child_store.table().unwrap();
    let child = committed.child(&CHILD_IEEE).unwrap();
    assert_ne!(child.short_address, CHILD_SHORT);
    assert!(child.reassignment_address.is_none());
    drop(rebooted);
    assert_eq!(nwk_replay_count(&mut security), 1);
}

#[test]
fn secured_reassignment_response_waits_for_replay_commit_after_durable_intent() {
    const CHILD_IEEE: [u8; 8] = [0x63; 8];
    const CHILD_SHORT: u16 = 0x5633;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = FailingApsReplayStore {
        inner: RamSecurityStateStore::new(),
        fail_next_nwk_replay: true,
        ..Default::default()
    };
    security.store(&commissioned_router_state(false)).unwrap();
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: child_conflict_frame(CHILD_SHORT, 31),
        security_use: false,
    });
    mac.clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::Node(
            zigbee_runtime::node::NodeError::Persistence(SecurityStoreError::Hardware)
        ))
    ));
    assert!(
        child_store
            .table()
            .unwrap()
            .pending_reassignment()
            .is_some(),
        "the response intent is durable before the replay floor"
    );
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::RejoinResponse),
        0,
        "the response remains blocked while replay persistence is failing"
    );
    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::RejoinResponse),
        1,
        "retry commits replay before releasing the one durable response"
    );
    assert!(
        child_store
            .table()
            .unwrap()
            .pending_reassignment()
            .is_none()
    );
    drop(app);
    assert_eq!(nwk_replay_count(&mut security), 1);
}

#[test]
fn sleepy_child_reassignment_commits_only_after_poll_delivery() {
    const CHILD_IEEE: [u8; 8] = [0x64; 8];
    const CHILD_SHORT: u16 = 0x5644;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, false))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = security_store(false);
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: child_conflict_frame(CHILD_SHORT, 32),
        security_use: false,
    });
    mac.clear_tx_history();

    block_on(app.step()).unwrap();
    let staged = child_store.table().unwrap();
    let replacement = staged
        .pending_reassignment()
        .and_then(|child| child.reassignment_address)
        .unwrap();
    assert_eq!(
        staged.child(&CHILD_IEEE).unwrap().short_address,
        CHILD_SHORT
    );
    assert_eq!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .indirect_queue()
            .pending_count(ShortAddress(CHILD_SHORT)),
        1
    );
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::RejoinResponse),
        0
    );

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    assert_eq!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .indirect_queue()
            .pending_count(ShortAddress(CHILD_SHORT)),
        1,
        "repeated service must not duplicate the queued Rejoin Response"
    );
    assert_eq!(
        child_store
            .table()
            .unwrap()
            .child(&CHILD_IEEE)
            .unwrap()
            .short_address,
        CHILD_SHORT
    );

    app.node_mut()
        .device_mut()
        .mac_mut()
        .enqueue_command_event(data_request(CHILD_SHORT));
    block_on(app.step()).unwrap();
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::RejoinResponse),
        1
    );
    let committed = child_store.table().unwrap();
    assert!(committed.pending_reassignment().is_none());
    assert_eq!(
        committed.child(&CHILD_IEEE).unwrap().short_address,
        replacement
    );
    assert_eq!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .known_child_by_ieee(&CHILD_IEEE),
        Some(ShortAddress(replacement))
    );
}

#[test]
fn remove_device_cancels_a_sleepy_child_reassignment_response() {
    const CHILD_IEEE: [u8; 8] = [0x65; 8];
    const CHILD_SHORT: u16 = 0x5655;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, false))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = security_store(false);
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: child_conflict_frame(CHILD_SHORT, 33),
        security_use: false,
    });
    mac.clear_tx_history();

    block_on(app.step()).unwrap();
    assert!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .has_pending_indirect_kind(
                ShortAddress(CHILD_SHORT),
                zigbee_nwk::IndirectFrameKind::RejoinResponse
            )
    );
    assert!(
        child_store
            .table()
            .unwrap()
            .pending_reassignment()
            .is_some()
    );

    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: remove_device_frame(CHILD_IEEE, 34, 34, 34),
        security_use: false,
    });
    block_on(app.step()).unwrap();

    assert!(
        !app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .has_pending_indirect_kind(
                ShortAddress(CHILD_SHORT),
                zigbee_nwk::IndirectFrameKind::RejoinResponse
            )
    );
    let staged = child_store.table().unwrap();
    assert!(staged.pending_reassignment().is_none());
    assert_eq!(
        staged.pending_removal().map(|child| child.ieee_address),
        Some(CHILD_IEEE)
    );
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::RejoinResponse),
        0
    );
}

#[test]
fn child_departure_store_failure_reboots_without_device_left_notification() {
    const CHILD_IEEE: [u8; 8] = [0x65; 8];
    const CHILD_SHORT: u16 = 0x5655;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = security_store(false);

    {
        let mut test_profile = profile();
        let mut device = parent_device(&mut test_profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
        let mut app = ParentRouterApp::new(
            node,
            PersistentChildren::new(child_store.clone()),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap();

        block_on(app.initialize()).unwrap();
        child_store.fail_next_store();
        let mac = app.node_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(CHILD_SHORT)),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
            lqi: 190,
            payload: child_leave_frame(CHILD_SHORT, CHILD_IEEE, 51),
            security_use: false,
        });
        mac.clear_tx_history();

        assert!(matches!(
            block_on(app.step()),
            Err(RouterAppError::ChildStore(ChildStoreError::Hardware))
        ));
        assert!(child_store.table().unwrap().pending_departure().is_none());
        assert!(
            outbound_unacked_aps_command_counters(
                app.node().device().mac(),
                ApsCommandId::UpdateDevice
            )
            .is_empty()
        );
    }

    assert_eq!(nwk_replay_count(&mut security), 0);
    let mut rebooted_profile = profile();
    let mut rebooted_device = parent_device(&mut rebooted_profile);
    let rebooted_node = ZigbeeNode::new(&mut rebooted_device, &mut security, &mut rebooted_profile);
    let mut rebooted = ParentRouterApp::new(
        rebooted_node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(rebooted.initialize()).unwrap();
    let mac = rebooted.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(CHILD_SHORT)),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 190,
        payload: child_leave_frame(CHILD_SHORT, CHILD_IEEE, 52),
        security_use: false,
    });
    mac.clear_tx_history();

    block_on(rebooted.step()).unwrap();
    let counters = outbound_unacked_aps_command_counters(
        rebooted.node().device().mac(),
        ApsCommandId::UpdateDevice,
    );
    assert_eq!(counters.len(), 1);
    assert!(
        child_store.table().unwrap().pending_departure().is_none(),
        "local submission retires the send intent without any peer ACK"
    );
    rebooted
        .node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(rebooted.step()).unwrap();
    assert!(child_store.table().unwrap().is_empty());
    assert_eq!(
        outbound_unacked_aps_command_counters(
            rebooted.node().device().mac(),
            ApsCommandId::UpdateDevice
        )
        .len(),
        1
    );
    drop(rebooted);
    assert_eq!(nwk_replay_count(&mut security), 0);
}

#[test]
fn child_departure_replay_failure_blocks_device_left_until_retry() {
    const CHILD_IEEE: [u8; 8] = [0x66; 8];
    const CHILD_SHORT: u16 = 0x5666;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = FailingApsReplayStore {
        inner: RamSecurityStateStore::new(),
        fail_next_nwk_replay: true,
        ..Default::default()
    };
    security.store(&commissioned_router_state(false)).unwrap();
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(CHILD_SHORT)),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 190,
        payload: child_leave_frame(CHILD_SHORT, CHILD_IEEE, 61),
        security_use: false,
    });
    mac.clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::Node(
            zigbee_runtime::node::NodeError::Persistence(SecurityStoreError::Hardware)
        ))
    ));
    assert!(child_store.table().unwrap().pending_departure().is_some());
    assert!(
        outbound_unacked_aps_command_counters(
            app.node().device().mac(),
            ApsCommandId::UpdateDevice
        )
        .is_empty()
    );

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    let counters = outbound_unacked_aps_command_counters(
        app.node().device().mac(),
        ApsCommandId::UpdateDevice,
    );
    assert_eq!(counters.len(), 1);
    assert!(child_store.table().unwrap().is_empty());
    block_on(app.step()).unwrap();
    assert!(child_store.table().unwrap().is_empty());
    assert_eq!(
        outbound_unacked_aps_command_counters(
            app.node().device().mac(),
            ApsCommandId::UpdateDevice
        )
        .len(),
        1
    );
    drop(app);
    assert_eq!(
        nwk_replay_count(&mut security),
        0,
        "revoking the departed child must tombstone its durable NWK replay domain"
    );
}

#[test]
fn remove_children_cascade_store_failure_reboots_before_child_or_parent_action() {
    const CHILD_IEEE: [u8; 8] = [0x67; 8];
    const CHILD_SHORT: u16 = 0x5677;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = security_store(false);

    {
        let mut test_profile = profile();
        let mut device = parent_device(&mut test_profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
        let mut app = ParentRouterApp::new(
            node,
            PersistentChildren::new(child_store.clone()),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap();

        block_on(app.initialize()).unwrap();
        child_store.fail_next_store();
        let mac = app.node_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
            lqi: 220,
            payload: remove_children_frame(true, 71),
            security_use: false,
        });
        mac.clear_tx_history();

        assert!(matches!(
            block_on(app.step()),
            Err(RouterAppError::ChildStore(ChildStoreError::Hardware))
        ));
        assert_eq!(
            outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
            0
        );
        assert!(app.node().device().is_joined());
        assert!(!child_store.table().unwrap().leave_cascade_pending());
    }

    assert_eq!(nwk_replay_count(&mut security), 0);
    let mut rebooted_profile = profile();
    let mut rebooted_device = parent_device(&mut rebooted_profile);
    let rebooted_node = ZigbeeNode::new(&mut rebooted_device, &mut security, &mut rebooted_profile);
    let mut rebooted = ParentRouterApp::new(
        rebooted_node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(
            NoStatus,
            TestSupervisor::default(),
            RecordingDiagnostics::default(),
        ),
    )
    .unwrap();

    block_on(rebooted.initialize()).unwrap();
    let mac = rebooted.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: remove_children_frame(true, 72),
        security_use: false,
    });
    mac.clear_tx_history();

    block_on(rebooted.step()).unwrap();
    assert_eq!(
        outbound_nwk_command_count(rebooted.node().device().mac(), NwkCommandId::Leave),
        1
    );
    let counters = outbound_unacked_aps_command_counters(
        rebooted.node().device().mac(),
        ApsCommandId::UpdateDevice,
    );
    assert_eq!(counters.len(), 1);
    let completed = child_store.table().unwrap();
    assert!(completed.is_empty());
    assert!(!completed.leave_cascade_pending());
    assert!(rebooted.parts().diagnostics.events.iter().any(|event| {
        matches!(
            event,
            DiagnosticEvent::SecureRejoinSucceeded { .. }
                | DiagnosticEvent::SecureRejoinFailed { .. }
                | DiagnosticEvent::SecureRejoinPending { .. }
        )
    }));
    drop(rebooted);
    assert_eq!(
        nwk_replay_sources(&mut security),
        vec![COORDINATOR_IEEE],
        "the accepted parent Leave floor remains valid while the router may rejoin the same network"
    );
}

#[test]
fn remove_children_cascade_replay_failure_blocks_child_and_parent_action_until_retry() {
    const CHILD_IEEE: [u8; 8] = [0x68; 8];
    const CHILD_SHORT: u16 = 0x5688;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = FailingApsReplayStore {
        inner: RamSecurityStateStore::new(),
        fail_next_nwk_replay: true,
        ..Default::default()
    };
    security.store(&commissioned_router_state(false)).unwrap();
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(
            NoStatus,
            TestSupervisor::default(),
            RecordingDiagnostics::default(),
        ),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: remove_children_frame(true, 81),
        security_use: false,
    });
    mac.clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::Node(
            zigbee_runtime::node::NodeError::Persistence(SecurityStoreError::Hardware)
        ))
    ));
    assert!(child_store.table().unwrap().leave_cascade_pending());
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
        0
    );
    assert!(
        outbound_unacked_aps_command_counters(
            app.node().device().mac(),
            ApsCommandId::UpdateDevice
        )
        .is_empty()
    );
    assert!(app.node().device().is_joined());

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
        1
    );
    let counters = outbound_unacked_aps_command_counters(
        app.node().device().mac(),
        ApsCommandId::UpdateDevice,
    );
    assert_eq!(counters.len(), 1);
    assert!(child_store.table().unwrap().is_empty());
    assert!(app.parts().diagnostics.events.iter().any(|event| {
        matches!(
            event,
            DiagnosticEvent::SecureRejoinSucceeded { .. }
                | DiagnosticEvent::SecureRejoinFailed { .. }
                | DiagnosticEvent::SecureRejoinPending { .. }
        )
    }));
    drop(app);
    assert_eq!(
        nwk_replay_sources(&mut security),
        vec![COORDINATOR_IEEE],
        "the accepted parent Leave floor remains valid while the router may rejoin the same network"
    );
}

#[test]
fn remove_children_cascade_waits_for_two_sleepy_children_and_device_left_submissions() {
    const CHILD1_IEEE: [u8; 8] = [0x69; 8];
    const CHILD1_SHORT: u16 = 0x5691;
    const CHILD2_IEEE: [u8; 8] = [0x6A; 8];
    const CHILD2_SHORT: u16 = 0x5692;

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD1_IEEE, CHILD1_SHORT, false))
        .unwrap();
    table
        .push(persistent_child(CHILD2_IEEE, CHILD2_SHORT, false))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let mut security = security_store(false);
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        &POLICY,
        RouterParts::new(
            NoStatus,
            TestSupervisor::default(),
            RecordingDiagnostics::default(),
        ),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: remove_children_frame(true, 91),
        security_use: false,
    });
    mac.clear_tx_history();

    block_on(app.step()).unwrap();
    let staged = child_store.table().unwrap();
    assert!(staged.leave_cascade_pending());
    assert!(staged.child(&CHILD1_IEEE).unwrap().removal_pending);
    assert!(staged.child(&CHILD2_IEEE).unwrap().removal_pending);
    assert!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .indirect_queue()
            .has_pending(ShortAddress(CHILD1_SHORT))
    );
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
        0,
        "a sleepy child's Leave is not on air until its poll"
    );
    assert!(app.node().device().is_joined());

    app.node_mut()
        .device_mut()
        .mac_mut()
        .enqueue_command_event(data_request(CHILD1_SHORT));
    assert_eq!(
        block_on(app.node_mut().device_mut().service_parent_commands()).processed,
        1
    );
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
        1
    );

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    let first_update = outbound_unacked_aps_command_counters(
        app.node().device().mac(),
        ApsCommandId::UpdateDevice,
    );
    assert_eq!(first_update.len(), 1);
    let after_first_poll = child_store.table().unwrap();
    assert!(after_first_poll.leave_cascade_pending());
    assert!(after_first_poll.child(&CHILD1_IEEE).is_none());
    assert!(
        after_first_poll
            .child(&CHILD2_IEEE)
            .unwrap()
            .removal_pending
    );
    assert!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .indirect_queue()
            .has_pending(ShortAddress(CHILD2_SHORT))
    );
    assert!(app.node().device().is_joined());

    app.node_mut()
        .device_mut()
        .mac_mut()
        .enqueue_command_event(data_request(CHILD2_SHORT));
    assert_eq!(
        block_on(app.node_mut().device_mut().service_parent_commands()).processed,
        1
    );
    assert_eq!(
        outbound_nwk_command_count(app.node().device().mac(), NwkCommandId::Leave),
        2
    );

    block_on(app.step()).unwrap();
    let updates = outbound_unacked_aps_command_counters(
        app.node().device().mac(),
        ApsCommandId::UpdateDevice,
    );
    assert_eq!(updates.len(), 2);
    let completed = child_store.table().unwrap();
    assert!(completed.is_empty());
    assert!(!completed.leave_cascade_pending());
    assert!(app.parts().diagnostics.events.iter().any(|event| {
        matches!(
            event,
            DiagnosticEvent::SecureRejoinSucceeded { .. }
                | DiagnosticEvent::SecureRejoinFailed { .. }
                | DiagnosticEvent::SecureRejoinPending { .. }
        )
    }));
}

#[test]
fn parent_restores_and_clears_durable_aps_tables_with_network_lifecycle() {
    let mut bindings = BindingTable::new();
    bindings
        .add(BindingEntry::unicast(LOCAL_IEEE, 1, 0x0006, [0x77; 8], 1))
        .unwrap();
    let mut groups = GroupTable::new();
    assert!(groups.add_group(0x1234, 1));
    let snapshot = ApsTableSnapshot::capture(EXTENDED_PAN_ID, &bindings, &groups).unwrap();
    let mut aps_store = RamApsTableStore::new();
    aps_store.store(&snapshot).unwrap();

    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        PersistentApsTables::new(aps_store),
        &POLICY,
        RouterParts::new(
            NoStatus,
            TestSupervisor::default(),
            RecordingDiagnostics::default(),
        ),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert_eq!(
        app.node().device().bdb().zdo().aps().binding_table().len(),
        1
    );
    assert!(
        app.node()
            .device()
            .bdb()
            .zdo()
            .aps()
            .group_table()
            .is_member(0x1234, 1)
    );
    assert!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::ApsTablesRestored { count: 2 }))
    );

    app.node_mut()
        .device_mut()
        .user_action(UserAction::FactoryReset);
    let events = block_on(app.step()).unwrap();
    assert!(matches!(events.tick, Some(StackEvent::Left)));
    assert!(
        app.aps_tables_mut()
            .store_mut()
            .load()
            .unwrap()
            .unwrap()
            .is_empty()
    );
    assert!(
        app.parts()
            .diagnostics
            .events
            .iter()
            .any(|event| matches!(event, DiagnosticEvent::ApsTablesCleared))
    );
}

#[test]
fn application_transport_key_is_persisted_before_replay_commit_and_ack() {
    let partner = [0x73; 8];
    let application_key = [0xA7; 16];
    let mut profile = profile();
    let mut device = parent_device(&mut profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        PersistentApsTables::new(FailingApsStore {
            inner: RamApsTableStore::new(),
            fail_next_store: true,
        }),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: application_transport_key_frame(partner, application_key, 3),
        security_use: false,
    });
    app.node_mut().device_mut().mac_mut().clear_tx_history();

    let first_step = block_on(app.step());
    assert!(
        matches!(
            first_step,
            Err(RouterAppError::ApsTables(
                zigbee_runtime::aps_table_store::ApsTableStoreError::Hardware
            ))
        ),
        "unexpected first step: {first_step:?}"
    );
    assert!(app.node().device().mac().tx_history().is_empty());
    assert!(app.node().device().application_key_persistence_pending());
    assert!(
        app.node()
            .device()
            .pending_application_key_replay()
            .is_some()
    );

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    assert!(!app.node().device().application_key_persistence_pending());
    assert!(
        app.node()
            .device()
            .pending_application_key_replay()
            .is_none()
    );
    assert_eq!(app.node().device().mac().tx_history().len(), 1);
    let snapshot = app
        .aps_tables_mut()
        .store_mut()
        .inner
        .load()
        .unwrap()
        .unwrap();
    assert_eq!(snapshot.application_keys().len(), 1);
    assert_eq!(snapshot.application_keys()[0].partner_address, partner);
    assert_eq!(snapshot.application_keys()[0].key, application_key);
}

#[test]
fn application_transport_key_retries_after_reboot_before_aps_table_commit() {
    let partner = [0x74; 8];
    let application_key = [0xA8; 16];
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let mut security = security_store(false);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        PersistentApsTables::new(FailingApsStore {
            inner: RamApsTableStore::new(),
            fail_next_store: true,
        }),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: application_transport_key_frame(partner, application_key, 3),
        security_use: false,
    });
    app.node_mut().device_mut().mac_mut().clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::ApsTables(
            zigbee_runtime::aps_table_store::ApsTableStoreError::Hardware
        ))
    ));
    assert!(app.node().device().mac().tx_history().is_empty());
    let mut aps_store = std::mem::take(&mut app.aps_tables_mut().store_mut().inner);
    assert!(aps_store.load().unwrap().is_none());
    drop(app);
    assert_eq!(aps_replay_count(&mut security), 0);

    let mut rebooted_profile = profile();
    let mut rebooted_device = parent_device(&mut rebooted_profile);
    let rebooted_node = ZigbeeNode::new(&mut rebooted_device, &mut security, &mut rebooted_profile);
    let mut rebooted = ParentRouterApp::new_with_aps_tables(
        rebooted_node,
        PersistentChildren::new(CountingChildStore::default()),
        PersistentApsTables::new(aps_store),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(rebooted.initialize()).unwrap();
    let mac = rebooted.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: application_transport_key_frame(partner, application_key, 4),
        security_use: false,
    });
    rebooted
        .node_mut()
        .device_mut()
        .mac_mut()
        .clear_tx_history();

    block_on(rebooted.step()).unwrap();
    assert_eq!(rebooted.node().device().mac().tx_history().len(), 1);
    let snapshot = rebooted
        .aps_tables_mut()
        .store_mut()
        .load()
        .unwrap()
        .unwrap();
    assert_eq!(snapshot.application_keys().len(), 1);
    assert_eq!(snapshot.application_keys()[0].partner_address, partner);
    assert_eq!(snapshot.application_keys()[0].key, application_key);
    drop(rebooted);
    assert_eq!(aps_replay_count(&mut security), 1);
}

#[test]
fn application_transport_key_retries_after_reboot_before_replay_commit() {
    let partner = [0x75; 8];
    let application_key = [0xA9; 16];
    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let mut security = FailingApsReplayStore {
        inner: RamSecurityStateStore::new(),
        fail_next_aps_replay: true,
        ..Default::default()
    };
    security.store(&commissioned_router_state(false)).unwrap();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        PersistentApsTables::new(RamApsTableStore::new()),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: application_transport_key_frame(partner, application_key, 3),
        security_use: false,
    });
    app.node_mut().device_mut().mac_mut().clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::Node(
            zigbee_runtime::node::NodeError::Persistence(SecurityStoreError::Hardware)
        ))
    ));
    assert!(app.node().device().mac().tx_history().is_empty());
    let mut aps_store = std::mem::take(app.aps_tables_mut().store_mut());
    let committed_snapshot = aps_store.load().unwrap().unwrap();
    assert_eq!(committed_snapshot.application_keys().len(), 1);
    drop(app);
    assert_eq!(aps_replay_count(&mut security), 0);

    let mut rebooted_profile = profile();
    let mut rebooted_device = parent_device(&mut rebooted_profile);
    let rebooted_node = ZigbeeNode::new(&mut rebooted_device, &mut security, &mut rebooted_profile);
    let mut rebooted = ParentRouterApp::new_with_aps_tables(
        rebooted_node,
        PersistentChildren::new(CountingChildStore::default()),
        PersistentApsTables::new(aps_store),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(rebooted.initialize()).unwrap();
    let restored_snapshot = rebooted
        .aps_tables_mut()
        .store_mut()
        .load()
        .unwrap()
        .unwrap();
    let mac = rebooted.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
        lqi: 220,
        payload: application_transport_key_frame(partner, application_key, 4),
        security_use: false,
    });
    rebooted
        .node_mut()
        .device_mut()
        .mac_mut()
        .clear_tx_history();

    block_on(rebooted.step()).unwrap();
    assert_eq!(rebooted.node().device().mac().tx_history().len(), 1);
    assert_eq!(
        rebooted
            .aps_tables_mut()
            .store_mut()
            .load()
            .unwrap()
            .unwrap(),
        restored_snapshot
    );
    drop(rebooted);
    assert_eq!(aps_replay_count(&mut security), 1);
}

#[test]
fn application_key_replacement_tombstone_failure_retries_after_reboot() {
    let partner = [0x76; 8];
    let unrelated_partner = [0x77; 8];
    let old_key = [0xB1; 16];
    let new_key = [0xB2; 16];
    let unrelated_key = old_key;

    let mut aps_security = ApsSecurity::new();
    for (partner_address, key) in [(partner, old_key), (unrelated_partner, unrelated_key)] {
        aps_security
            .add_key(ApsLinkKeyEntry {
                partner_address,
                key,
                key_type: ApsKeyType::ApplicationLinkKey,
                outgoing_frame_counter: 0,
                outgoing_frame_counter_limit: 0,
                incoming_frame_counter: 0,
                incoming_frame_counter_valid: false,
            })
            .unwrap();
    }
    let snapshot = ApsTableSnapshot::capture_with_security(
        EXTENDED_PAN_ID,
        &BindingTable::new(),
        &GroupTable::new(),
        &aps_security,
    )
    .unwrap();
    let mut aps_store = RamApsTableStore::new();
    aps_store.store(&snapshot).unwrap();

    let old_replay = PersistentReplayCounter::Aps(ApsReplayCounter::from_verified(
        ApsKeyOrigin::KeyPair {
            partner,
            key_type: ApsKeyType::ApplicationLinkKey,
        },
        partner,
        &old_key,
        23,
    ));
    let unrelated_replay = PersistentReplayCounter::Aps(ApsReplayCounter::from_verified(
        ApsKeyOrigin::KeyPair {
            partner: unrelated_partner,
            key_type: ApsKeyType::ApplicationLinkKey,
        },
        unrelated_partner,
        &unrelated_key,
        31,
    ));
    let mut security = FailingApsReplayStore::default();
    security.store(&commissioned_router_state(false)).unwrap();
    security.commit_replay_counter(old_replay).unwrap();
    security.commit_replay_counter(unrelated_replay).unwrap();

    let mut test_profile = profile();
    let mut device = parent_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        PersistentApsTables::new(aps_store),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();
    block_on(app.initialize()).unwrap();
    app.node_mut().device_mut().mac_mut().set_rx_delay_us(0);
    app.node_mut()
        .device_mut()
        .mac_mut()
        .enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT_ADDRESS)),
            lqi: 220,
            payload: application_transport_key_frame(partner, new_key, 3),
            security_use: false,
        });
    app.node_mut().device_mut().mac_mut().clear_tx_history();
    app.node_mut()
        .device_and_security_store_mut()
        .1
        .fail_next_tombstone = true;

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::Node(
            zigbee_runtime::node::NodeError::Persistence(SecurityStoreError::Hardware)
        ))
    ));
    assert!(app.node().device().mac().tx_history().is_empty());
    assert!(app.node().device().application_key_persistence_pending());
    assert_eq!(
        app.aps_tables_mut()
            .store_mut()
            .load()
            .unwrap()
            .unwrap()
            .application_keys()
            .iter()
            .find(|entry| entry.partner_address == partner)
            .unwrap()
            .key,
        new_key,
        "replacement state commits before replay retirement is attempted"
    );
    let retained = replay_counters(app.node_mut().device_and_security_store_mut().1);
    assert!(retained.contains(&old_replay));
    assert!(retained.contains(&unrelated_replay));

    let aps_store = std::mem::take(app.aps_tables_mut().store_mut());
    drop(app);

    let mut rebooted_profile = profile();
    let mut rebooted_device = parent_device(&mut rebooted_profile);
    let rebooted_node = ZigbeeNode::new(&mut rebooted_device, &mut security, &mut rebooted_profile);
    let mut rebooted = ParentRouterApp::new_with_aps_tables(
        rebooted_node,
        PersistentChildren::new(CountingChildStore::default()),
        PersistentApsTables::new(aps_store),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(rebooted.initialize()).unwrap();
    let retained = replay_counters(rebooted.node_mut().device_and_security_store_mut().1);
    assert!(!retained.contains(&old_replay));
    assert!(retained.contains(&unrelated_replay));
    assert_eq!(
        rebooted
            .node()
            .device()
            .aps()
            .security()
            .find_key(&partner, ApsKeyType::ApplicationLinkKey)
            .unwrap()
            .key,
        new_key
    );
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
            removal_pending: false,
            removal_attempts: 0,
            reassignment_address: None,
            departure_pending: false,
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
                removal_pending: false,
                removal_attempts: 0,
                reassignment_address: None,
                departure_pending: false,
            })
            .unwrap();
        let mut test_profile = profile();
        let mut device = coordinator_device(&mut test_profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
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

#[cfg(feature = "trust-center")]
#[test]
fn trust_center_factory_reset_is_cross_journal_crash_safe() {
    let mut security = RamSecurityStateStore::new();
    let mut trust_center_store = RamTrustCenterDeviceStore::new();
    let mut stale_trust_center = PersistentTrustCenterState::new([0xA5; 8]);
    stale_trust_center
        .allow_application_key_request([0x31; 8], [0x32; 8])
        .unwrap();
    trust_center_store.store(&stale_trust_center).unwrap();
    let mut profile = profile();
    let mut device = coordinator_device(&mut profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        trust_center_store,
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    assert!(app.node().device().is_joined());
    assert!(app.trust_center().is_initialized());
    let first_epid = app.node().device().bdb().zdo().nwk().nib().extended_pan_id;
    let initialized_tc = app.trust_center_mut().store_mut().load().unwrap().unwrap();
    assert!(initialized_tc.matches_network(&first_epid));
    assert!(initialized_tc.application_key_requests().is_empty());

    app.node_mut()
        .device_mut()
        .user_action(UserAction::FactoryReset);
    let events = block_on(app.step()).unwrap();
    assert!(matches!(events.tick, Some(StackEvent::Left)));
    assert!(!app.node().device().is_joined());
    assert!(!app.trust_center().is_initialized());
    assert!(
        !app.node_mut()
            .load_security_state()
            .unwrap()
            .unwrap()
            .commissioned,
        "factory-new security must commit before auxiliary stores are cleared"
    );
    assert!(app.trust_center_mut().store_mut().load().unwrap().is_none());

    block_on(app.step()).unwrap();
    assert!(app.node().device().is_joined());
    assert!(app.trust_center().is_initialized());
    let second_epid = app.node().device().bdb().zdo().nwk().nib().extended_pan_id;
    assert!(
        app.trust_center_mut()
            .store_mut()
            .load()
            .unwrap()
            .unwrap()
            .matches_network(&second_epid)
    );
}

#[cfg(feature = "trust-center")]
#[test]
fn trust_center_replay_floor_is_reapplied_after_restoring_device_keys() {
    const CHILD_IEEE: [u8; 8] = [0x80; 8];
    const CHILD_KEY: [u8; 16] = [0x90; 16];
    const REPLAY_FLOOR: u32 = 23;

    let mut trust_center_store = FaultTrustCenterStore::default();
    let mut security = RamSecurityStateStore::new();
    security.store(&commissioned_coordinator_state()).unwrap();

    {
        let mut test_profile = profile();
        let mut device = coordinator_device(&mut test_profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
        let mut app = TrustCenterCoordinatorApp::new(
            node,
            PersistentChildren::new(CountingChildStore::default()),
            trust_center_store.clone(),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        app.provision_trust_center_link_key(CHILD_IEEE, CHILD_KEY)
            .unwrap();
    }

    // Power cut after the security journal commits 23, but before the next
    // TC checkpoint: the independent TC snapshot still contains only 17.
    let mut checkpoint = trust_center_store.state().unwrap();
    let stored = checkpoint.device_mut(&CHILD_IEEE).unwrap();
    stored.incoming_frame_counter = 17;
    stored.incoming_frame_counter_valid = true;
    trust_center_store.store(&checkpoint).unwrap();
    let replay = PersistentReplayCounter::Aps(ApsReplayCounter::from_verified(
        ApsKeyOrigin::KeyPair {
            partner: CHILD_IEEE,
            key_type: ApsKeyType::TrustCenterLinkKey,
        },
        CHILD_IEEE,
        &CHILD_KEY,
        REPLAY_FLOOR,
    ));
    security.commit_replay_counter(replay).unwrap();

    // Arm the fault only when TC restore starts, not during the earlier
    // NWK/APS-table restores. Thus the failure occurs with live TC keys but
    // before their replay floors have been re-applied.
    struct ArmReplayFailure {
        inner: FaultTrustCenterStore,
        fail: Rc<Cell<bool>>,
        armed: bool,
    }
    impl TrustCenterDeviceStore for ArmReplayFailure {
        fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError> {
            if self.armed {
                self.fail.set(true);
            }
            self.inner.load()
        }
        fn store(
            &mut self,
            state: &PersistentTrustCenterState,
        ) -> Result<(), TrustCenterStoreError> {
            self.inner.store(state)
        }
        fn clear(&mut self) -> Result<(), TrustCenterStoreError> {
            self.inner.clear()
        }
    }
    let fail = Rc::new(Cell::new(false));
    let mut security = FailingApsReplayStore {
        inner: security,
        fail_replay_visit: Some(fail.clone()),
        ..Default::default()
    };
    let mut rebooted_profile = profile();
    let mut rebooted_device = coordinator_device(&mut rebooted_profile);
    let rebooted_node = ZigbeeNode::new(&mut rebooted_device, &mut security, &mut rebooted_profile);
    let mut rebooted = TrustCenterCoordinatorApp::new(
        rebooted_node,
        PersistentChildren::new(CountingChildStore::default()),
        ArmReplayFailure {
            inner: trust_center_store,
            fail: fail.clone(),
            armed: true,
        },
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();
    assert!(block_on(rebooted.initialize()).is_err());
    assert!(!rebooted.trust_center().is_initialized());
    let request_key = [ApsCommandId::RequestKey as u8, 0x04];
    let mac = rebooted.node_mut().device_mut().mac_mut();
    mac.clear_tx_history();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(0x4480)),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        lqi: 220,
        payload: secured_aps_command_frame(
            ShortAddress(0x4480),
            CHILD_IEEE,
            ShortAddress::COORDINATOR,
            CHILD_KEY,
            (1, REPLAY_FLOOR, 100),
            &request_key,
        ),
        security_use: false,
    });
    assert!(block_on(rebooted.step()).is_err());
    assert!(!rebooted.trust_center().is_initialized());
    assert!(
        rebooted.node().device().mac().tx_history().is_empty(),
        "failed replay restore must block RX/ACKs and all TC retries"
    );
    rebooted.trust_center_mut().store_mut().armed = false;
    fail.set(false);
    let events = block_on(rebooted.step()).unwrap();
    assert!(rebooted.trust_center().is_initialized());
    assert!(
        !events
            .iter()
            .any(|event| matches!(event, StackEvent::ApsSecurityIndication(_))),
        "APS counter 23 inside a fresh NWK envelope must still be rejected"
    );

    let restored = rebooted
        .node()
        .device()
        .aps()
        .security()
        .find_key(&CHILD_IEEE, ApsKeyType::TrustCenterLinkKey)
        .unwrap();
    assert!(restored.incoming_frame_counter_valid);
    assert_eq!(restored.incoming_frame_counter, REPLAY_FLOOR);
    assert!(
        !rebooted
            .node()
            .device()
            .aps()
            .security()
            .check_frame_counter(&CHILD_IEEE, ApsKeyType::TrustCenterLinkKey, REPLAY_FLOOR)
    );
    rebooted
        .node_mut()
        .device_mut()
        .mac_mut()
        .enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(0x4480)),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            lqi: 220,
            payload: secured_aps_command_frame(
                ShortAddress(0x4480),
                CHILD_IEEE,
                ShortAddress::COORDINATOR,
                CHILD_KEY,
                (2, REPLAY_FLOOR + 1, 101),
                &request_key,
            ),
            security_use: false,
        });
    let events = block_on(rebooted.step()).unwrap();
    assert!(
        events
            .iter()
            .any(|event| matches!(event, StackEvent::ApsSecurityIndication(_))),
        "the next authenticated APS counter must be accepted after recovery"
    );
    assert_eq!(
        rebooted
            .node()
            .device()
            .aps()
            .security()
            .find_key(&CHILD_IEEE, ApsKeyType::TrustCenterLinkKey)
            .unwrap()
            .incoming_frame_counter,
        REPLAY_FLOOR + 1
    );
}

#[cfg(feature = "trust-center")]
#[test]
fn trust_center_local_device_left_store_failure_requeues_before_child_cleanup() {
    const CHILD_IEEE: [u8; 8] = [0x81; 8];
    const CHILD_SHORT: u16 = 0x4181;
    const CHILD_KEY: [u8; 16] = [0x91; 16];

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let trust_center_store = FaultTrustCenterStore::default();
    let mut security = RamSecurityStateStore::new();
    security.store(&commissioned_coordinator_state()).unwrap();
    let mut test_profile = profile();
    let mut device = coordinator_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        trust_center_store.clone(),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    app.provision_trust_center_link_key(CHILD_IEEE, CHILD_KEY)
        .unwrap();
    trust_center_store.fail_next_store();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(CHILD_SHORT)),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        lqi: 190,
        payload: child_leave_frame_to(CHILD_SHORT, CHILD_IEEE, ShortAddress::COORDINATOR, 81),
        security_use: false,
    });
    mac.clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::TrustCenter(TrustCenterRuntimeError::Store(
            TrustCenterStoreError::Hardware
        )))
    ));
    assert!(
        child_store.table().unwrap().pending_departure().is_some(),
        "a failed TC commit must retain the local child departure journal"
    );
    assert!(
        trust_center_store
            .state()
            .unwrap()
            .device(&CHILD_IEEE)
            .is_some(),
        "the failed DeviceLeft store must leave the durable TC device intact"
    );
    assert!(
        outbound_unacked_aps_command_counters(
            app.node().device().mac(),
            ApsCommandId::UpdateDevice
        )
        .is_empty(),
        "the local Trust Center must never send Update-Device to itself"
    );

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    assert!(child_store.table().unwrap().pending_departure().is_none());
    assert!(
        trust_center_store
            .state()
            .unwrap()
            .device(&CHILD_IEEE)
            .is_none(),
        "the regenerated local indication must retry and durably revoke the device"
    );
    drop(app);
    assert_eq!(
        nwk_replay_count(&mut security),
        0,
        "revoking the departed child must tombstone its durable NWK replay domain"
    );
}

#[cfg(feature = "trust-center")]
#[test]
fn trust_center_device_left_replay_failure_retries_ack_after_tc_commit() {
    const PARENT_IEEE: [u8; 8] = [0x82; 8];
    const PARENT_SHORT: u16 = 0x4282;
    const PARENT_KEY: [u8; 16] = [0x92; 16];
    const DEPARTED_IEEE: [u8; 8] = [0x83; 8];
    const DEPARTED_SHORT: u16 = 0x4383;
    const DEPARTED_KEY: [u8; 16] = [0x93; 16];

    let mut children = PersistentChildTable::new(EXTENDED_PAN_ID);
    children
        .push(persistent_child(PARENT_IEEE, PARENT_SHORT, true))
        .unwrap();
    let trust_center_store = FaultTrustCenterStore::default();
    let mut security = FailingApsReplayStore {
        inner: RamSecurityStateStore::new(),
        fail_next_aps_replay: true,
        ..Default::default()
    };
    security.store(&commissioned_coordinator_state()).unwrap();
    let mut test_profile = profile();
    let mut device = coordinator_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(CountingChildStore::with_table(children)),
        trust_center_store.clone(),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    app.provision_trust_center_link_key(PARENT_IEEE, PARENT_KEY)
        .unwrap();
    app.provision_trust_center_link_key(DEPARTED_IEEE, DEPARTED_KEY)
        .unwrap();
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(PARENT_SHORT)),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        lqi: 210,
        payload: update_device_frame(
            ShortAddress(PARENT_SHORT),
            PARENT_IEEE,
            PARENT_KEY,
            DEPARTED_IEEE,
            ShortAddress(DEPARTED_SHORT),
            82,
            ApsUpdateDeviceStatus::DeviceLeft,
        ),
        security_use: false,
    });
    mac.clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::Node(
            zigbee_runtime::node::NodeError::Persistence(SecurityStoreError::Hardware)
        ))
    ));
    assert!(
        trust_center_store
            .state()
            .unwrap()
            .device(&DEPARTED_IEEE)
            .is_none(),
        "the TC DeviceLeft mutation commits before APS replay persistence"
    );
    assert_eq!(
        outbound_aps_ack_count(app.node().device().mac()),
        0,
        "the APS ACK must remain blocked while replay persistence fails"
    );
    assert!(
        app.node()
            .device()
            .security_indication_persistence_pending()
    );

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    assert_eq!(outbound_aps_ack_count(app.node().device().mac()), 1);
    assert!(
        !app.node()
            .device()
            .security_indication_persistence_pending()
    );
    drop(app);
    assert_eq!(aps_replay_count(&mut security), 1);
}

#[cfg(feature = "trust-center")]
#[test]
fn trust_center_app_handles_repeated_device_left_and_denied_unknown_secured_rejoin() {
    const PARENT: [u8; 8] = [0x82; 8];
    const PARENT_SHORT: u16 = 0x4282;
    const PARENT_KEY: [u8; 16] = [0x5A; 16];
    const DEPARTED: [u8; 8] = [0x83; 8];
    let mut tc_store = FaultTrustCenterStore::default();
    let mut security = RamSecurityStateStore::new();
    security.store(&commissioned_coordinator_state()).unwrap();
    {
        let mut profile = profile();
        let mut device = coordinator_device(&mut profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
        let mut app = TrustCenterCoordinatorApp::new(
            node,
            PersistentChildren::new(CountingChildStore::default()),
            tc_store.clone(),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        app.provision_trust_center_link_key(PARENT, PARENT_KEY)
            .unwrap();
        app.provision_trust_center_link_key(DEPARTED, [0x93; 16])
            .unwrap();
    }
    let mut snapshot = tc_store.state().unwrap();
    let parent = &mut snapshot.device_mut(&PARENT).unwrap().device;
    parent.parent_address = LOCAL_IEEE;
    parent.short_address = ShortAddress(PARENT_SHORT);
    tc_store.store(&snapshot).unwrap();
    let mut children = PersistentChildTable::new(EXTENDED_PAN_ID);
    children
        .push(persistent_child(PARENT, PARENT_SHORT, true))
        .unwrap();

    let mut profile = profile();
    let mut device = coordinator_device(&mut profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(CountingChildStore::with_table(children)),
        tc_store.clone(),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();
    block_on(app.initialize()).unwrap();
    app.node_mut().device_mut().mac_mut().clear_tx_history();
    for (index, status) in [
        ApsUpdateDeviceStatus::DeviceLeft, // revoke the existing device
        ApsUpdateDeviceStatus::DeviceLeft, // no longer in either TC table
        ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
        ApsUpdateDeviceStatus::StandardDeviceSecuredRejoin,
        ApsUpdateDeviceStatus::DeviceLeft, // parent confirms the rejection
    ]
    .into_iter()
    .enumerate()
    {
        let mac = app.node_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(PARENT_SHORT)),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            lqi: 220,
            payload: update_device_frame(
                ShortAddress(PARENT_SHORT),
                PARENT,
                PARENT_KEY,
                DEPARTED,
                ShortAddress(0x4383),
                index as u32 + 1,
                status,
            ),
            security_use: false,
        });
        let events =
            block_on(app.step()).expect("authenticated policy denial is not a fatal error");
        assert!(
            events
                .iter()
                .any(|event| matches!(event, StackEvent::ApsSecurityIndication(_)))
        );
        assert!(app.trust_center().table().device(&DEPARTED).is_none());
        assert!(tc_store.state().unwrap().device(&DEPARTED).is_none());
        assert!(
            tc_store
                .state()
                .unwrap()
                .pending_unknown_removals()
                .is_empty()
        );
    }
    assert_eq!(outbound_aps_ack_count(app.node().device().mac()), 5);
    assert_eq!(
        outbound_aps_command_counters(app.node().device().mac(), ApsCommandId::RemoveDevice, false)
            .len(),
        2,
        "each denied rejoin submits a removal without waiting for an APS ACK",
    );
}

#[cfg(feature = "trust-center")]
#[test]
fn remote_device_announce_closes_initial_key_intent_through_the_receive_path() {
    const PARENT: [u8; 8] = [0x82; 8];
    const PARENT_SHORT: u16 = 0x4282;
    const CHILD: [u8; 8] = [0x83; 8];
    const CHILD_SHORT: u16 = 0x4383;
    let mut tc_store = FaultTrustCenterStore::default();
    let mut security = RamSecurityStateStore::new();
    security.store(&commissioned_coordinator_state()).unwrap();
    {
        let mut profile = profile();
        let mut device = coordinator_device(&mut profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
        let mut app = TrustCenterCoordinatorApp::new(
            node,
            PersistentChildren::new(CountingChildStore::default()),
            tc_store.clone(),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap();
        block_on(app.initialize()).unwrap();
        app.provision_trust_center_link_key(PARENT, [0x5A; 16])
            .unwrap();
        app.provision_trust_center_link_key(CHILD, [0x6A; 16])
            .unwrap();
    }
    let mut snapshot = tc_store.state().unwrap();
    let parent = &mut snapshot.device_mut(&PARENT).unwrap().device;
    parent.parent_address = LOCAL_IEEE;
    parent.short_address = ShortAddress(PARENT_SHORT);
    let child = snapshot.device_mut(&CHILD).unwrap();
    child.device.parent_address = PARENT;
    child.device.short_address = ShortAddress(CHILD_SHORT);
    child.network_key_pending = true;
    tc_store.store(&snapshot).unwrap();
    let mut children = PersistentChildTable::new(EXTENDED_PAN_ID);
    children
        .push(persistent_child(PARENT, PARENT_SHORT, true))
        .unwrap();
    let mut profile = profile();
    let mut device = coordinator_device(&mut profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(CountingChildStore::with_table(children)),
        tc_store.clone(),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();
    block_on(app.initialize()).unwrap();

    for (index, announced_short) in [PARENT_SHORT, CHILD_SHORT].into_iter().enumerate() {
        let counter = index as u32 + 1;
        let mut aps = [0u8; 32];
        let header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Data as u8,
                delivery_mode: ApsDeliveryMode::Broadcast as u8,
                ..Default::default()
            },
            dst_endpoint: Some(0),
            cluster_id: Some(0x0013),
            profile_id: Some(0),
            src_endpoint: Some(0),
            aps_counter: counter as u8,
            ..Default::default()
        };
        let hlen = header.serialize(&mut aps);
        aps[hlen] = counter as u8;
        aps[hlen + 1..hlen + 3].copy_from_slice(&announced_short.to_le_bytes());
        aps[hlen + 3..hlen + 11].copy_from_slice(&CHILD);
        aps[hlen + 11] = 0x80;
        let frame = secured_nwk_frame(
            NwkFrameType::Data,
            ShortAddress(CHILD_SHORT),
            PARENT,
            ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            counter as u8,
            counter,
            &aps[..hlen + 12],
        );
        let mac = app.node_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(PARENT_SHORT)),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            lqi: 200,
            payload: frame,
            security_use: false,
        });
        let events = block_on(app.step()).unwrap();
        assert_eq!(
            events.iter().any(|event| matches!(event,
                StackEvent::DeviceAnnounced { address, short_address, .. }
                    if *address == CHILD && short_address.0 == CHILD_SHORT)),
            index == 1,
        );
        assert_eq!(
            tc_store
                .state()
                .unwrap()
                .device(&CHILD)
                .unwrap()
                .network_key_pending,
            index == 0
        );
    }
    assert_eq!(
        tc_store.state().unwrap().device(&CHILD).unwrap().is_router,
        Some(false)
    );
}

#[cfg(feature = "trust-center")]
#[test]
fn trust_center_local_device_left_cleanup_store_failure_retries_cleanup_only() {
    const CHILD_IEEE: [u8; 8] = [0x84; 8];
    const CHILD_SHORT: u16 = 0x4484;
    const CHILD_KEY: [u8; 16] = [0x94; 16];

    let mut table = PersistentChildTable::new(EXTENDED_PAN_ID);
    table
        .push(persistent_child(CHILD_IEEE, CHILD_SHORT, true))
        .unwrap();
    let child_store = FaultChildStore::with_table(table);
    let trust_center_store = FaultTrustCenterStore::default();
    let mut security = RamSecurityStateStore::new();
    security.store(&commissioned_coordinator_state()).unwrap();
    let mut test_profile = profile();
    let mut device = coordinator_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        trust_center_store.clone(),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    app.provision_trust_center_link_key(CHILD_IEEE, CHILD_KEY)
        .unwrap();
    child_store.fail_store_after(1);
    let mac = app.node_mut().device_mut().mac_mut();
    mac.set_rx_delay_us(0);
    mac.enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(CHILD_SHORT)),
        dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        lqi: 190,
        payload: child_leave_frame_to(CHILD_SHORT, CHILD_IEEE, ShortAddress::COORDINATOR, 84),
        security_use: false,
    });
    mac.clear_tx_history();

    assert!(matches!(
        block_on(app.step()),
        Err(RouterAppError::ChildStore(ChildStoreError::Hardware))
    ));
    assert!(
        trust_center_store
            .state()
            .unwrap()
            .device(&CHILD_IEEE)
            .is_none(),
        "child cleanup may fail only after the TC DeviceLeft state is durable"
    );
    assert!(child_store.table().unwrap().pending_departure().is_some());
    let trust_center_store_attempts = trust_center_store.store_attempts();

    app.node_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    block_on(app.step()).unwrap();
    assert!(child_store.table().unwrap().pending_departure().is_none());
    assert_eq!(
        trust_center_store.store_attempts(),
        trust_center_store_attempts,
        "cleanup retry must not re-run an already durable TC DeviceLeft mutation"
    );
    drop(app);
    assert_eq!(
        nwk_replay_count(&mut security),
        0,
        "revoking the departed child must tombstone its durable NWK replay domain"
    );
}

#[cfg(feature = "trust-center")]
#[test]
fn trust_center_app_commits_and_completes_network_key_rotation_in_order() {
    let mut security = RamSecurityStateStore::new();
    let mut profile = profile();
    let mut device = coordinator_device(&mut profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let old_sequence = app
        .node()
        .device()
        .bdb()
        .zdo()
        .nwk()
        .security()
        .active_key()
        .unwrap()
        .seq_number;
    app.node_mut().device_mut().mac_mut().clear_tx_history();

    block_on(app.request_network_key_rotation()).unwrap();
    let prepared_security = app.node_mut().load_security_state().unwrap().unwrap();
    assert!(prepared_security.staged_network_key_present);
    assert_eq!(
        prepared_security.staged_key_sequence,
        old_sequence.wrapping_add(1)
    );
    assert!(app.trust_center().network_key_rotation().is_some());
    assert!(app.node().device().mac().tx_history().is_empty());

    block_on(app.step()).unwrap();
    assert!(app.trust_center().network_key_rotation().is_none());
    assert_eq!(app.node().device().mac().tx_history().len(), 2);
    let nwk = app.node().device().bdb().zdo().nwk();
    assert_eq!(
        nwk.security().active_key().unwrap().seq_number,
        old_sequence.wrapping_add(1)
    );
    assert_eq!(
        nwk.nib().active_key_seq_number,
        old_sequence.wrapping_add(1)
    );
    let committed_security = app.node_mut().load_security_state().unwrap().unwrap();
    assert_eq!(
        committed_security.key_sequence,
        old_sequence.wrapping_add(1)
    );
    assert!(committed_security.staged_network_key_present);
    assert_eq!(committed_security.staged_key_sequence, old_sequence);
    assert!(committed_security.secondary_network_key_is_previous);
}

#[cfg(feature = "trust-center")]
#[test]
fn network_key_rotation_retains_previous_key_and_recovers_failed_replacement() {
    let mut security = FailingApsReplayStore::default();
    let mut test_profile = profile();
    let mut device = coordinator_device(&mut test_profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();

    block_on(app.initialize()).unwrap();
    let old_key = app
        .node()
        .device()
        .bdb()
        .zdo()
        .nwk()
        .security()
        .active_key()
        .unwrap()
        .clone();
    let old_replay = PersistentReplayCounter::Nwk(NwkReplayCounter::from_verified(
        [0x91; 8],
        old_key.seq_number,
        &old_key.key,
        17,
    ));
    app.node_mut()
        .device_and_security_store_mut()
        .1
        .commit_replay_counter(old_replay)
        .unwrap();

    block_on(app.request_network_key_rotation()).unwrap();
    let staged_key = app
        .node()
        .device()
        .bdb()
        .zdo()
        .nwk()
        .security()
        .staged_key()
        .unwrap()
        .clone();
    let retained_replay = PersistentReplayCounter::Nwk(NwkReplayCounter::from_verified(
        [0x92; 8],
        staged_key.seq_number,
        &staged_key.key,
        19,
    ));
    let store = app.node_mut().device_and_security_store_mut().1;
    store.commit_replay_counter(retained_replay).unwrap();
    block_on(app.step()).unwrap();
    assert!(
        app.trust_center().network_key_rotation().is_none(),
        "rotation completion retains the old key for receive compatibility"
    );
    let retained = replay_counters(app.node_mut().device_and_security_store_mut().1);
    assert!(retained.contains(&old_replay));
    assert!(retained.contains(&retained_replay));
    let durable = app.node_mut().load_security_state().unwrap().unwrap();
    assert!(
        durable.secondary_network_key_is_previous,
        "the old key remains durable until a subsequent update replaces it"
    );

    let trust_center_store = std::mem::take(app.trust_center_mut().store_mut());
    drop(app);

    let mut rebooted_profile = profile();
    let mut rebooted_device = coordinator_device(&mut rebooted_profile);
    let rebooted_node = ZigbeeNode::new(&mut rebooted_device, &mut security, &mut rebooted_profile);
    let mut rebooted = TrustCenterCoordinatorApp::new(
        rebooted_node,
        PersistentChildren::new(CountingChildStore::default()),
        trust_center_store,
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();
    block_on(rebooted.initialize()).unwrap();

    assert!(rebooted.trust_center().network_key_rotation().is_none());
    let retained = replay_counters(rebooted.node_mut().device_and_security_store_mut().1);
    assert!(retained.contains(&old_replay));
    assert!(retained.contains(&retained_replay));
    assert_eq!(
        rebooted
            .node()
            .device()
            .bdb()
            .zdo()
            .nwk()
            .security()
            .active_key()
            .unwrap()
            .seq_number,
        staged_key.seq_number
    );

    assert!(block_on(rebooted.request_network_key_rotation()).is_err());
    assert!(
        rebooted
            .node()
            .device()
            .aps()
            .nwk()
            .security()
            .staged_key()
            .is_none()
    );
    let broadcast_delay = u32::from(
        rebooted
            .node()
            .device()
            .aps()
            .nwk()
            .nib()
            .broadcast_delivery_time,
    ) * 1_000_000;
    block_on(
        rebooted
            .node_mut()
            .device_mut()
            .mac_mut()
            .delay_micros(broadcast_delay),
    );
    rebooted
        .node_mut()
        .device_mut()
        .mac_mut()
        .clear_tx_history();
    rebooted
        .node_mut()
        .device_and_security_store_mut()
        .1
        .fail_next_tombstone = true;
    assert!(block_on(rebooted.request_network_key_rotation()).is_err());
    assert!(
        !rebooted
            .node_mut()
            .device_and_security_store_mut()
            .1
            .fail_next_tombstone
    );
    assert!(rebooted.trust_center().network_key_rotation().is_none());
    assert!(rebooted.node().device().mac().tx_history().is_empty());
    let prepared = rebooted.node_mut().load_security_state().unwrap().unwrap();
    assert!(prepared.staged_network_key_present);
    assert!(!prepared.secondary_network_key_is_previous);
    let retained = replay_counters(rebooted.node_mut().device_and_security_store_mut().1);
    assert!(
        retained.contains(&old_replay),
        "failed tombstone remains recoverable"
    );
    assert!(retained.contains(&retained_replay));
    let trust_center_store = std::mem::take(rebooted.trust_center_mut().store_mut());
    drop(rebooted);

    let mut recovered_profile = profile();
    let mut recovered_device = coordinator_device(&mut recovered_profile);
    let node = ZigbeeNode::new(&mut recovered_device, &mut security, &mut recovered_profile);
    let mut recovered = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(CountingChildStore::default()),
        trust_center_store,
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();
    block_on(recovered.initialize()).unwrap();
    let retained = replay_counters(recovered.node_mut().device_and_security_store_mut().1);
    assert!(!retained.contains(&old_replay));
    assert!(retained.contains(&retained_replay));
    block_on(recovered.step()).unwrap();
    assert!(recovered.trust_center().network_key_rotation().is_none());
    assert_eq!(
        recovered
            .node()
            .device()
            .aps()
            .nwk()
            .security()
            .active_key()
            .unwrap()
            .seq_number,
        staged_key.seq_number.wrapping_add(1)
    );
}

#[cfg(feature = "trust-center")]
#[test]
fn network_key_rotation_rearms_own_sleepy_child_and_honors_profile_window() {
    use zigbee_nwk::IndirectFrameKind;
    use zigbee_runtime::trust_center_store::NetworkKeyRotationPhase;

    const CHILD: [u8; 8] = [0x76; 8];
    const SHORT: u16 = 0x4376;
    const MAX_POLL_US: u32 = 20_000_000;
    let mut children = PersistentChildTable::new(EXTENDED_PAN_ID);
    children
        .push(persistent_child(CHILD, SHORT, false))
        .unwrap();
    let child_store = FaultChildStore::with_table(children);
    let mut security = RamSecurityStateStore::new();
    security.store(&commissioned_coordinator_state()).unwrap();
    let mut test_profile = profile();
    let mut device = coordinator_device(&mut test_profile);
    device
        .set_network_key_forwarding_max_poll_interval_us(MAX_POLL_US)
        .unwrap();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(child_store.clone()),
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();
    block_on(app.initialize()).unwrap();
    app.node_mut().device_mut().mac_mut().clear_tx_history();
    block_on(app.request_network_key_rotation()).unwrap();
    let prepared = app.node_mut().load_security_state().unwrap().unwrap();
    assert!(prepared.network_key_forwarding_pending);
    let sequence = prepared.staged_key_sequence;
    let kind = IndirectFrameKind::NetworkKeyUpdate(sequence);
    assert!(app.node().device().mac().tx_history().is_empty());
    block_on(app.step()).unwrap();
    assert_eq!(
        app.trust_center().network_key_rotation().unwrap().phase,
        NetworkKeyRotationPhase::WaitingForPropagation
    );
    assert!(
        app.node()
            .device()
            .aps()
            .nwk()
            .has_pending_indirect_kind(ShortAddress(SHORT), kind)
    );
    let tc_store = std::mem::take(app.trust_center_mut().store_mut());
    drop(app);

    let mut test_profile = profile();
    let mut device = coordinator_device(&mut test_profile);
    device
        .set_network_key_forwarding_max_poll_interval_us(MAX_POLL_US)
        .unwrap();
    let node = ZigbeeNode::new(&mut device, &mut security, &mut test_profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(child_store),
        tc_store,
        &POLICY,
        RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
    )
    .unwrap();
    block_on(app.initialize()).unwrap();
    block_on(app.step()).unwrap();
    assert!(
        app.node()
            .device()
            .aps()
            .nwk()
            .has_pending_indirect_kind(ShortAddress(SHORT), kind)
    );
    let mac = app.node_mut().device_mut().mac_mut();
    mac.clear_tx_history();
    mac.enqueue_command_event(MacCommandEvent::DataRequest(MlmeDataRequestIndication {
        source_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT)),
        destination_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
        lqi: 180,
        security_use: false,
    }));
    block_on(app.step()).unwrap();
    assert!(
        !app.node()
            .device()
            .aps()
            .nwk()
            .has_pending_indirect_kind(ShortAddress(SHORT), kind)
    );
    let record = app
        .node()
        .device()
        .mac()
        .tx_history()
        .iter()
        .find(|record| record.dst == MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT)))
        .expect("the restored TC must forward the key on its own child's poll");
    let (nwk, payload) = decrypt_outbound_nwk(&record.payload).unwrap();
    assert_eq!(nwk.src_addr, ShortAddress::COORDINATOR);
    assert_eq!(nwk.dst_addr, ShortAddress(SHORT));
    let (aps, header_len) = ApsHeader::parse(&payload).unwrap();
    assert!(!aps.frame_control.ack_request);
    let command = &payload[header_len..];
    assert_eq!(command[0], ApsCommandId::TransportKey as u8);
    assert_eq!(command[1], 0x01);
    assert_eq!(command[18], sequence);
    assert_eq!(&command[19..27], &[0; 8]);
    assert_eq!(&command[27..35], &LOCAL_IEEE);

    block_on(
        app.node_mut()
            .device_mut()
            .mac_mut()
            .delay_micros(30_000_000),
    );
    block_on(app.step()).unwrap();
    assert_eq!(
        app.trust_center().network_key_rotation().unwrap().phase,
        NetworkKeyRotationPhase::WaitingForPropagation
    );
    block_on(
        app.node_mut()
            .device_mut()
            .mac_mut()
            .delay_micros(10_000_000),
    );
    block_on(app.step()).unwrap();
    assert!(app.trust_center().network_key_rotation().is_none());
    assert_eq!(
        app.node().device().aps().nwk().nib().active_key_seq_number,
        sequence
    );
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
            removal_pending: false,
            removal_attempts: 0,
            reassignment_address: None,
            departure_pending: false,
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

#[cfg(feature = "trust-center")]
mod key_power_cuts {
    use super::*;
    use zigbee_runtime::security_store::ReplayCounterTombstone;
    use zigbee_runtime::trust_center_store::NetworkKeyRotationPhase;

    const CHILD: [u8; 8] = [0x76; 8];
    const SHORT: u16 = 0x4376;
    const MAX_POLL_US: u32 = 20_000_000;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Journal {
        Security,
        Replay,
        Tombstone,
        TrustCenter,
        Children,
    }

    #[derive(Default)]
    struct Cuts {
        active: bool,
        fail_at: Option<usize>,
        failed: bool,
        trace: Vec<Journal>,
    }

    impl Cuts {
        fn arm(&mut self, fail_at: Option<usize>) {
            *self = Self {
                active: true,
                fail_at,
                ..Default::default()
            };
        }

        fn before_commit(&mut self, journal: Journal) -> bool {
            if !self.active {
                return false;
            }
            self.trace.push(journal);
            if self.fail_at == Some(self.trace.len() - 1) {
                self.failed = true;
                self.active = false;
            }
            self.failed
        }

        fn finish(&mut self, result: Result<(), RouterAppError>) -> Vec<Journal> {
            self.active = false;
            if let Some(index) = self.fail_at {
                assert!(self.failed, "cut {index} was not reached: {:?}", self.trace);
                assert!(
                    result.is_err(),
                    "cut {index} was swallowed: {:?}",
                    self.trace
                );
            } else {
                result.unwrap();
            }
            self.trace.clone()
        }
    }

    #[derive(Clone, Default)]
    struct CutStore<S> {
        inner: S,
        cuts: Rc<RefCell<Cuts>>,
    }

    impl<S: SecurityStateStore> SecurityStateStore for CutStore<S> {
        fn load(&mut self) -> Result<Option<PersistentSecurityState>, SecurityStoreError> {
            self.inner.load()
        }

        fn store(&mut self, state: &PersistentSecurityState) -> Result<(), SecurityStoreError> {
            if self.cuts.borrow_mut().before_commit(Journal::Security) {
                return Err(SecurityStoreError::Hardware);
            }
            self.inner.store(state)
        }

        fn visit_replay_counters(
            &mut self,
            visitor: &mut dyn FnMut(PersistentReplayCounter),
        ) -> Result<(), SecurityStoreError> {
            self.inner.visit_replay_counters(visitor)
        }

        fn commit_replay_counter(
            &mut self,
            replay: PersistentReplayCounter,
        ) -> Result<(), SecurityStoreError> {
            if self.cuts.borrow_mut().before_commit(Journal::Replay) {
                return Err(SecurityStoreError::Hardware);
            }
            self.inner.commit_replay_counter(replay)
        }

        fn tombstone_replay_counters(
            &mut self,
            tombstone: ReplayCounterTombstone,
        ) -> Result<(), SecurityStoreError> {
            if self.cuts.borrow_mut().before_commit(Journal::Tombstone) {
                return Err(SecurityStoreError::Hardware);
            }
            self.inner.tombstone_replay_counters(tombstone)
        }
    }

    impl<S: TrustCenterDeviceStore> TrustCenterDeviceStore for CutStore<S> {
        fn load(&mut self) -> Result<Option<PersistentTrustCenterState>, TrustCenterStoreError> {
            self.inner.load()
        }

        fn store(
            &mut self,
            state: &PersistentTrustCenterState,
        ) -> Result<(), TrustCenterStoreError> {
            if self.cuts.borrow_mut().before_commit(Journal::TrustCenter) {
                return Err(TrustCenterStoreError::Hardware);
            }
            self.inner.store(state)
        }

        fn clear(&mut self) -> Result<(), TrustCenterStoreError> {
            self.inner.clear()
        }
    }

    impl<S: ChildTableStore> ChildTableStore for CutStore<S> {
        fn load(&mut self) -> Result<Option<PersistentChildTable>, ChildStoreError> {
            self.inner.load()
        }

        fn store(&mut self, table: &PersistentChildTable) -> Result<(), ChildStoreError> {
            if self.cuts.borrow_mut().before_commit(Journal::Children) {
                return Err(ChildStoreError::Hardware);
            }
            self.inner.store(table)
        }
    }

    struct Stores {
        security: CutStore<RamSecurityStateStore>,
        children: CutStore<FaultChildStore>,
        tc: CutStore<RamTrustCenterDeviceStore>,
        cuts: Rc<RefCell<Cuts>>,
    }

    impl Stores {
        fn new() -> Self {
            let cuts = Rc::new(RefCell::new(Cuts::default()));
            let mut security = RamSecurityStateStore::new();
            security.store(&commissioned_coordinator_state()).unwrap();
            let mut children = PersistentChildTable::new(EXTENDED_PAN_ID);
            children
                .push(persistent_child(CHILD, SHORT, false))
                .unwrap();
            Self {
                security: CutStore {
                    inner: security,
                    cuts: cuts.clone(),
                },
                children: CutStore {
                    inner: FaultChildStore::with_table(children),
                    cuts: cuts.clone(),
                },
                tc: CutStore {
                    inner: RamTrustCenterDeviceStore::new(),
                    cuts: cuts.clone(),
                },
                cuts,
            }
        }
    }

    type CutApp<'a> = TrustCenterCoordinatorApp<
        'a,
        MockMac,
        CutStore<RamSecurityStateStore>,
        TestProfile,
        CutStore<FaultChildStore>,
        CutStore<RamTrustCenterDeviceStore>,
        NoStatus,
        TestSupervisor,
        NoDiagnostics,
    >;

    fn app<'a>(
        stores: &'a mut Stores,
        device: &'a mut ZigbeeDevice<MockMac, Router>,
        profile: &'a mut TestProfile,
    ) -> CutApp<'a> {
        device
            .set_network_key_forwarding_max_poll_interval_us(MAX_POLL_US)
            .unwrap();
        TrustCenterCoordinatorApp::new(
            ZigbeeNode::new(device, &mut stores.security, profile),
            PersistentChildren::new(stores.children.clone()),
            std::mem::take(&mut stores.tc),
            &POLICY,
            RouterParts::new(NoStatus, TestSupervisor::default(), NoDiagnostics),
        )
        .unwrap()
    }

    fn poll_child(app: &mut CutApp<'_>) -> Result<(), RouterAppError> {
        app.node_mut()
            .device_mut()
            .mac_mut()
            .enqueue_command_event(MacCommandEvent::DataRequest(MlmeDataRequestIndication {
                source_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT)),
                destination_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
                lqi: 180,
                security_use: false,
            }));
        block_on(app.step()).map(|_| ())
    }

    fn nwk_counters(mac: &MockMac) -> Vec<u32> {
        mac.tx_history()
            .iter()
            .filter_map(|record| {
                let (header, len) = NwkHeader::parse(record.payload.as_slice()).unwrap();
                header.frame_control.security.then(|| {
                    NwkSecurityHeader::parse(&record.payload.as_slice()[len..])
                        .unwrap()
                        .0
                        .frame_counter
                })
            })
            .collect()
    }

    fn rotation_cut(fail_at: Option<usize>) -> Vec<Journal> {
        let mut stores = Stores::new();
        let cuts = stores.cuts.clone();
        let mut test_profile = profile();
        let mut device = coordinator_device(&mut test_profile);
        let mut running = app(&mut stores, &mut device, &mut test_profile);
        block_on(running.initialize()).unwrap();
        let initial = running.node_mut().load_security_state().unwrap().unwrap();
        running.node_mut().device_mut().mac_mut().clear_tx_history();
        cuts.borrow_mut().arm(fail_at);
        let result = block_on(async {
            running.request_network_key_rotation().await?;
            running.step().await?;
            assert_eq!(
                running.trust_center().network_key_rotation().unwrap().phase,
                NetworkKeyRotationPhase::WaitingForPropagation
            );
            poll_child(&mut running)?;
            assert!(
                running
                    .node()
                    .device()
                    .mac()
                    .tx_history()
                    .iter()
                    .any(|frame| frame.indirect)
            );
            running
                .node_mut()
                .device_mut()
                .mac_mut()
                .delay_micros(2 * MAX_POLL_US)
                .await;
            running.step().await?;
            assert!(running.trust_center().network_key_rotation().is_none());
            Ok(())
        });
        let trace = cuts.borrow_mut().finish(result);
        let persisted = running.node_mut().load_security_state().unwrap().unwrap();
        let tc_state = running
            .trust_center_mut()
            .store_mut()
            .load()
            .unwrap()
            .unwrap();
        if persisted.key_sequence != initial.key_sequence {
            assert!(persisted.secondary_network_key_is_previous);
            assert_eq!(persisted.staged_network_key, initial.network_key);
            assert!(
                tc_state.rotation().is_none_or(|rotation| {
                    rotation.phase == NetworkKeyRotationPhase::Activating
                })
            );
        } else if let Some(rotation) = tc_state.rotation() {
            assert!(persisted.staged_network_key_present);
            assert_eq!(persisted.staged_key_sequence, rotation.target_sequence);
        }
        let before = nwk_counters(running.node().device().mac());
        let tc = std::mem::take(running.trust_center_mut().store_mut());
        drop(running);
        stores.tc = tc;

        let mut test_profile = profile();
        let mut device = coordinator_device(&mut test_profile);
        let mut recovered = app(&mut stores, &mut device, &mut test_profile);
        block_on(recovered.initialize()).unwrap();
        assert!(
            recovered
                .node()
                .device()
                .aps()
                .nwk()
                .nib()
                .outgoing_frame_counter
                >= persisted.global_counter_limit
        );
        if !persisted.staged_network_key_present {
            // The first preparation never committed, so no transaction existed
            // on disk. A fresh request is required, not invented by restore.
            assert!(recovered.trust_center().network_key_rotation().is_none());
            block_on(recovered.request_network_key_rotation()).unwrap();
        }
        block_on(recovered.step()).unwrap();
        poll_child(&mut recovered).unwrap();
        if let Some(rotation) = recovered.trust_center().network_key_rotation() {
            assert_eq!(
                rotation.phase,
                NetworkKeyRotationPhase::WaitingForPropagation
            );
            assert_eq!(
                recovered
                    .node()
                    .device()
                    .aps()
                    .nwk()
                    .nib()
                    .active_key_seq_number,
                initial.key_sequence
            );
            block_on(
                recovered
                    .node_mut()
                    .device_mut()
                    .mac_mut()
                    .delay_micros(2 * MAX_POLL_US),
            );
            block_on(recovered.step()).unwrap();
        }
        assert!(recovered.trust_center().network_key_rotation().is_none());
        let completed = recovered.node_mut().load_security_state().unwrap().unwrap();
        assert_eq!(completed.key_sequence, initial.key_sequence.wrapping_add(1));
        assert!(completed.secondary_network_key_is_previous);
        assert_eq!(completed.staged_network_key, initial.network_key);
        assert_eq!(completed.staged_key_sequence, initial.key_sequence);
        if persisted.staged_network_key_present && !persisted.secondary_network_key_is_previous {
            assert_eq!(completed.network_key, persisted.staged_network_key);
        }
        assert_eq!(completed.extended_pan_id, initial.extended_pan_id);
        let after = nwk_counters(recovered.node().device().mac());
        if let (Some(before), Some(after)) = (before.iter().max(), after.iter().min()) {
            assert!(
                after > before,
                "reboot reused a transmitted counter at cut {fail_at:?}"
            );
        }
        assert!(
            recovered
                .coordinator()
                .children()
                .store()
                .inner
                .table()
                .unwrap()
                .child(&CHILD)
                .is_some()
        );
        trace
    }

    #[test]
    fn sleepy_rotation_recovers_at_every_security_and_tc_commit() {
        let control = rotation_cut(None);
        assert!(control.contains(&Journal::Security));
        assert!(control.contains(&Journal::TrustCenter));
        assert!(control.len() <= 32, "keep the sweep bounded: {control:?}");
        for index in 0..control.len() {
            assert_eq!(rotation_cut(Some(index)), control[..=index], "cut {index}");
        }
    }

    fn announce(counter: u32) -> MacFrame {
        let mut aps = [0u8; 32];
        let header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Data as u8,
                delivery_mode: ApsDeliveryMode::Broadcast as u8,
                ..Default::default()
            },
            dst_endpoint: Some(0),
            cluster_id: Some(0x0013),
            profile_id: Some(0),
            src_endpoint: Some(0),
            aps_counter: counter as u8,
            ..Default::default()
        };
        let len = header.serialize(&mut aps);
        aps[len] = counter as u8;
        aps[len + 1..len + 3].copy_from_slice(&SHORT.to_le_bytes());
        aps[len + 3..len + 11].copy_from_slice(&CHILD);
        aps[len + 11] = 0x80;
        secured_nwk_frame(
            NwkFrameType::Data,
            ShortAddress(SHORT),
            CHILD,
            ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            counter as u8,
            counter,
            &aps[..len + 12],
        )
    }

    fn receive_announce(app: &mut CutApp<'_>, counter: u32) -> Result<(), RouterAppError> {
        let mac = app.node_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(McpsDataIndication {
            src_address: MacAddress::Short(PanId(PAN_ID), ShortAddress(SHORT)),
            dst_address: MacAddress::Short(PanId(PAN_ID), ShortAddress::COORDINATOR),
            lqi: 200,
            payload: announce(counter),
            security_use: false,
        });
        let result = block_on(app.step()).map(|_| ());
        app.node_mut()
            .device_mut()
            .mac_mut()
            .set_rx_delay_us(u32::MAX);
        result
    }

    fn pending(app: &mut CutApp<'_>) -> bool {
        app.trust_center_mut()
            .store_mut()
            .load()
            .unwrap()
            .unwrap()
            .device(&CHILD)
            .unwrap()
            .network_key_pending
    }

    fn initial_completion_cut(fail_at: Option<usize>) -> Vec<Journal> {
        let mut stores = Stores::new();
        let cuts = stores.cuts.clone();
        {
            let mut test_profile = profile();
            let mut device = coordinator_device(&mut test_profile);
            let mut running = app(&mut stores, &mut device, &mut test_profile);
            block_on(running.initialize()).unwrap();
            running
                .provision_trust_center_link_key(CHILD, [0x64; 16])
                .unwrap();
            let store = running.trust_center_mut().store_mut();
            let mut snapshot = store.load().unwrap().unwrap();
            let child = snapshot.device_mut(&CHILD).unwrap();
            child.device.parent_address = LOCAL_IEEE;
            child.device.short_address = ShortAddress(SHORT);
            child.network_key_pending = true;
            store.store(&snapshot).unwrap();
            let tc = std::mem::take(store);
            drop(running);
            stores.tc = tc;
        }
        let mut test_profile = profile();
        let mut device = coordinator_device(&mut test_profile);
        let mut running = app(&mut stores, &mut device, &mut test_profile);
        block_on(running.initialize()).unwrap();
        poll_child(&mut running).unwrap();
        assert!(
            running
                .node()
                .device()
                .mac()
                .tx_history()
                .iter()
                .any(|frame| frame.indirect)
        );
        assert!(
            pending(&mut running),
            "MAC delivery is not authenticated completion"
        );
        cuts.borrow_mut().arm(fail_at);
        let result = receive_announce(&mut running, 1);
        let trace = cuts.borrow_mut().finish(result);
        assert_eq!(pending(&mut running), fail_at.is_some());
        let replay_committed =
            nwk_replay_sources(running.node_mut().device_and_security_store_mut().1)
                .contains(&CHILD);
        let tc = std::mem::take(running.trust_center_mut().store_mut());
        drop(running);
        stores.tc = tc;

        let mut test_profile = profile();
        let mut device = coordinator_device(&mut test_profile);
        let mut recovered = app(&mut stores, &mut device, &mut test_profile);
        block_on(recovered.initialize()).unwrap();
        poll_child(&mut recovered).unwrap();
        assert_eq!(
            pending(&mut recovered),
            fail_at.is_some(),
            "neither restore nor a second MAC delivery may close the intent"
        );
        if fail_at.is_some() {
            receive_announce(&mut recovered, 1).unwrap();
            assert_eq!(
                pending(&mut recovered),
                replay_committed,
                "an already committed announcement must not authenticate again"
            );
            receive_announce(&mut recovered, 2).unwrap();
        }
        assert!(!pending(&mut recovered));
        assert_eq!(
            recovered
                .trust_center_mut()
                .store_mut()
                .load()
                .unwrap()
                .unwrap()
                .device(&CHILD)
                .unwrap()
                .is_router,
            Some(false)
        );
        assert!(
            recovered
                .coordinator()
                .children()
                .store()
                .inner
                .table()
                .unwrap()
                .child(&CHILD)
                .is_some()
        );
        trace
    }

    #[test]
    fn sleepy_initial_completion_recovers_at_every_replay_and_tc_commit() {
        let control = initial_completion_cut(None);
        assert!(control.contains(&Journal::Replay));
        assert!(control.contains(&Journal::TrustCenter));
        assert!(control.len() <= 16, "keep the sweep bounded: {control:?}");
        for index in 0..control.len() {
            assert_eq!(
                initial_completion_cut(Some(index)),
                control[..=index],
                "cut {index}"
            );
        }
    }
}
