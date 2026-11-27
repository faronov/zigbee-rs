//! Runtime exposures behind the NCP (`host-passthrough`): indication
//! metadata, device events, deferred APS ACK confirms and host endpoints,
//! with the real Trust Center coordinator on a mock MAC.
//!
//! The host indication queue is process-wide, so the tests run one at a time
//! (`SERIAL`).

use std::collections::VecDeque;
use std::sync::Mutex;

use ncp_coordinator_app::port::{
    CoordinatorPort, DataConfirm, DataRequest, Destination, DeviceEvent, FormParams, OpError,
    joined, key_source, left,
};
use ncp_coordinator_app::tc::{TcApp, TcPort};
use ncp_coordinator_app::transport::{HostTransport, TransportInfo};
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity};
use router_app::{
    NoDiagnostics, NoStatus, NoSupervisor, PersistentChildren, RouterParts, RouterPolicy,
    TrustCenterCoordinatorApp,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_aps::frames::{ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};
use zigbee_aps::security::{
    ApsKeyType, ApsLinkKeyEntry, ApsSecurity, ApsSecurityHeader, DEFAULT_TC_LINK_KEY,
    KEY_ID_DATA_KEY, SEC_LEVEL_ENC_MIC_32,
};
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::{
    AssociationStatus, MacFrame, McpsDataIndication, MlmeAssociateIndication,
    MlmeDataRequestIndication,
};
use zigbee_mac::{CapabilityInfo, MacCommandEvent, PlatformServices};
use zigbee_nwk::frames::{NwkCommandId, NwkFrameControl, NwkFrameType, NwkHeader};
use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::child_store::RamChildTableStore;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_runtime::security_store::RamSecurityStateStore;
use zigbee_runtime::trust_center_store::RamTrustCenterDeviceStore;
use zigbee_types::{ChannelMask, MacAddress, PanId, ShortAddress};
use zigbee_zcl::DeviceId;
use zigbee_zrhp::codec::{Bytes8, List};
use zigbee_zrhp::frame::TX_BUFFER_LEN;
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent};
use zigbee_zrhp::msg::data::{ApsDataRequest, ApsDst, EvtApsDataConfirm};
use zigbee_zrhp::msg::network::{EvtPermitJoinState, NwkAddEndpoint, NwkLeaveReset, NwkPermitJoin};
use zigbee_zrhp::msg::system::{SysNegotiateReq, SysNegotiateRsp};
use zigbee_zrhp::msg::{Batch, Kind, MsgHeader, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::types::Status;

const IEEE: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];
const PEER_IEEE: [u8; 8] = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8];

static SERIAL: Mutex<()> = Mutex::new(());

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000,
    join_retry_initial_ms: 10,
    join_retry_max_ms: 100,
    secure_rejoin_failure_limit: 2,
};

type Profile = DeviceProfile<RangeExtender>;

fn profile() -> Profile {
    DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::COMBINED_INTERFACE,
        RangeExtender,
    )
}

fn device(profile: &Profile) -> ZigbeeDevice<MockMac, Router> {
    let mut mac = MockMac::new(IEEE);
    mac.set_rx_delay_us(u32::MAX);
    mac.add_energy(zigbee_mac::EdValue {
        channel: 15,
        energy: 10,
    });
    ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .channels(ChannelMask(1 << 15))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_coordinator()
}

fn clock_ms() -> u32 {
    0
}

fn ncp_chosen() -> FormParams {
    FormParams {
        channel_mask: 1 << 15,
        pan_id: 0xFFFF,
        ext_pan_id: [0; 8],
        network_key: [0; 16],
        key_seq: 0,
        nwk_update_id: 0,
        tx_power: 0,
        nwk_manager: 0,
    }
}

/// A formed Trust Center coordinator behind a `TcPort`, receiving frames
/// queued on its mock MAC.
macro_rules! formed_port {
    ($port:ident) => {
        let mut security = RamSecurityStateStore::new();
        let mut gp_store = zigbee_runtime::gp_store::RamGpProxyStore::new();
        let mut profile = profile();
        let mut device = device(&profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
        let mut app = TrustCenterCoordinatorApp::new(
            node,
            PersistentChildren::new(RamChildTableStore::new()),
            RamTrustCenterDeviceStore::new(),
            &POLICY,
            RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
        )
        .unwrap();
        // The NCP's Green Power sink journals its table (spec 7.11).
        app.set_green_power_store(&mut gp_store);
        let mut $port = TcPort::new(app, IEEE, clock_ms);
        pollster::block_on($port.form(ncp_chosen())).unwrap();
        while $port.take_device_event().is_some() {}
    };
}

fn mac<A: TcApp<Mac = MockMac>>(port: &mut TcPort<A>) -> &mut MockMac {
    port.app_mut().device_mut().mac_mut()
}

fn steps<A: TcApp>(port: &mut TcPort<A>, count: usize) {
    for _ in 0..count {
        pollster::block_on(port.step()).unwrap();
    }
}

fn pan<A: TcApp>(port: &TcPort<A>) -> PanId {
    PanId(port.info().pan_id)
}

/// A NWK frame secured with the network key, as `source` sends it.
fn secured_nwk_frame(
    key: &[u8; 16],
    frame_type: NwkFrameType,
    source: (ShortAddress, [u8; 8]),
    destination: ShortAddress,
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
        src_addr: source.0,
        radius: 5,
        seq_number: frame_counter as u8,
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
        source_address: source.1,
        key_seq_number: 0,
    };
    let security_len = security_header.serialize(&mut bytes[header_len..]);
    let aad_len = header_len + security_len;
    let encrypted = NwkSecurity::new()
        .encrypt(&bytes[..aad_len], payload, key, &security_header)
        .unwrap();
    bytes[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    bytes[header_len] &= !0x07;
    MacFrame::from_slice(&bytes[..aad_len + encrypted.len()]).unwrap()
}

/// An APS unicast data frame (endpoint 1 → 1, On/Off), APS-secured with
/// `link_key` when given.
fn aps_data_frame(
    aps_counter: u8,
    link_key: Option<(&[u8; 16], [u8; 8], u32)>,
    payload: &[u8],
) -> Vec<u8> {
    let header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            security: link_key.is_some(),
            ..Default::default()
        },
        dst_endpoint: Some(1),
        cluster_id: Some(0x0006),
        profile_id: Some(PROFILE_HOME_AUTOMATION),
        src_endpoint: Some(1),
        aps_counter,
        ..Default::default()
    };
    let mut aps = [0u8; 96];
    let header_len = header.serialize(&mut aps);
    let Some((key, source, frame_counter)) = link_key else {
        aps[header_len..header_len + payload.len()].copy_from_slice(payload);
        return aps[..header_len + payload.len()].to_vec();
    };
    let security_header = ApsSecurityHeader {
        security_control: (KEY_ID_DATA_KEY << 3) | (1 << 5),
        frame_counter,
        source_address: Some(source),
        key_seq_number: None,
    };
    let security_len = security_header.serialize(&mut aps[header_len..]);
    let aad_len = header_len + security_len;
    let mut authenticated = [0u8; 32];
    authenticated[..aad_len].copy_from_slice(&aps[..aad_len]);
    authenticated[header_len] |= SEC_LEVEL_ENC_MIC_32;
    let encrypted = ApsSecurity::new()
        .encrypt(&authenticated[..aad_len], payload, key, &security_header)
        .unwrap();
    aps[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    aps[..aad_len + encrypted.len()].to_vec()
}

/// The APS ACK `source` sends for a data frame the NCP sent to it.
fn aps_ack_frame(aps_counter: u8) -> Vec<u8> {
    let header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Ack as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ..Default::default()
        },
        dst_endpoint: Some(1),
        cluster_id: Some(0x0006),
        profile_id: Some(PROFILE_HOME_AUTOMATION),
        src_endpoint: Some(1),
        aps_counter,
        ..Default::default()
    };
    let mut aps = [0u8; 16];
    let len = header.serialize(&mut aps);
    aps[..len].to_vec()
}

fn receive<A: TcApp<Mac = MockMac>>(
    port: &mut TcPort<A>,
    last_hop: ShortAddress,
    rssi_dbm: i8,
    frame: MacFrame,
) {
    let pan = pan(port);
    // The mock delivers a queued frame after `rx_delay`; otherwise (u32::MAX)
    // every receive slice times out.
    mac(port).set_rx_delay_us(0);
    mac(port).enqueue_rx(McpsDataIndication {
        src_address: MacAddress::Short(pan, last_hop),
        dst_address: MacAddress::Short(pan, ShortAddress::COORDINATOR),
        lqi: 200,
        payload: frame.with_rssi_dbm(rssi_dbm),
        security_use: false,
    });
    steps(port, 1);
    mac(port).set_rx_delay_us(u32::MAX);
    steps(port, 1);
}

fn network_key<A: TcApp>(port: &TcPort<A>) -> [u8; 16] {
    port.network_key(0).expect("formed").0
}

/// Associate an rx-on end device with the NCP through the MAC.
fn associate<A: TcApp<Mac = MockMac>>(port: &mut TcPort<A>, ieee: [u8; 8]) -> ShortAddress {
    pollster::block_on(port.permit_join(60, 0x0000)).unwrap();
    let pan = pan(port);
    let capability = CapabilityInfo {
        device_type_ffd: false,
        mains_powered: true,
        rx_on_when_idle: true,
        security_capable: true,
        allocate_address: true,
    };
    mac(port).enqueue_command_event(MacCommandEvent::AssociationRequest(
        MlmeAssociateIndication {
            device_address: ieee,
            coordinator_address: MacAddress::Short(pan, ShortAddress::COORDINATOR),
            capability_info: capability,
            lqi: 180,
            security_use: false,
        },
    ));
    steps(port, 2);
    let response = mac(port)
        .association_responses()
        .last()
        .expect("association response")
        .clone();
    assert_eq!(response.status, AssociationStatus::Success);
    // The mock MAC delivers the response itself (AssociationResponseDelivery).
    steps(port, 4);
    response.short_address
}

#[test]
fn indications_carry_the_last_hop_rssi_and_aps_security() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    while port.take_indication().is_some() {}
    let key = network_key(&port);
    let peer = (ShortAddress(0x1234), PEER_IEEE);

    // NWK-secured only.
    let aps = aps_data_frame(0x10, None, &[0x01, 0x02, 0x00]);
    receive(
        &mut port,
        ShortAddress(0x5678),
        -57,
        secured_nwk_frame(&key, NwkFrameType::Data, peer, ShortAddress(0), 0x100, &aps),
    );
    let indication = port.take_indication().expect("indication");
    assert_eq!(
        (
            indication.src_nwk,
            indication.last_hop_nwk,
            indication.dst_ep
        ),
        (0x1234, 0x5678, 1)
    );
    assert_eq!(indication.rssi, -57);
    assert!(indication.nwk_secured);
    assert!(!indication.aps_secured);
    assert_eq!(indication.key_source, key_source::NONE);

    // APS-secured with the device's unique Trust Center link key.
    let link_key = [0x5A; 16];
    port.app_mut()
        .device_mut()
        .aps_mut()
        .security_mut()
        .add_key(ApsLinkKeyEntry {
            partner_address: PEER_IEEE,
            key: link_key,
            key_type: ApsKeyType::TrustCenterLinkKey,
            outgoing_frame_counter: 0,
            outgoing_frame_counter_limit: 0x1000,
            incoming_frame_counter: 0,
            incoming_frame_counter_valid: false,
        })
        .unwrap();
    let aps = aps_data_frame(
        0x11,
        Some((&link_key, PEER_IEEE, 0x20)),
        &[0x01, 0x03, 0x01],
    );
    receive(
        &mut port,
        ShortAddress(0x1234),
        -71,
        secured_nwk_frame(&key, NwkFrameType::Data, peer, ShortAddress(0), 0x101, &aps),
    );
    let indication = port.take_indication().expect("APS-secured indication");
    assert_eq!(indication.asdu.as_slice(), &[0x01, 0x03, 0x01]);
    assert_eq!(indication.rssi, -71);
    assert!(indication.nwk_secured && indication.aps_secured);
    assert_eq!(indication.key_source, key_source::UNIQUE_TRUST_CENTER);
    assert_eq!(port.take_indication(), None);
}

#[test]
fn joins_departures_and_aged_children_are_device_events() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let key = network_key(&port);

    // Association to the NCP: the Trust Center admits it.
    let child = associate(&mut port, PEER_IEEE);
    assert_eq!(
        port.take_device_event(),
        Some(DeviceEvent::Joined {
            nwk: child.0,
            ieee: PEER_IEEE,
            parent_nwk: 0x0000,
            join_type: joined::JOIN_ASSOCIATION,
            capability: 0x80 | 0x40 | 0x08 | 0x04,
            decision: joined::ADMITTED,
        })
    );

    steps(&mut port, 10);
    assert_eq!(port.take_device_event(), None, "one decision per join");
    // The child announces a Leave with the rejoin bit: one event, not a
    // second one when the Trust Center processes the local Update-Device.
    let leave = [NwkCommandId::Leave as u8, 1 << 5];
    receive(
        &mut port,
        child,
        -60,
        secured_nwk_frame(
            &key,
            NwkFrameType::Command,
            (child, PEER_IEEE),
            ShortAddress(0),
            0x10,
            &leave,
        ),
    );
    steps(&mut port, 6);
    assert_eq!(
        port.take_device_event(),
        Some(DeviceEvent::Left {
            nwk: child.0,
            ieee: PEER_IEEE,
            flags: left::FLAG_REJOIN,
            reason: left::REASON_LEAVE,
        })
    );
    steps(&mut port, 10);
    assert_eq!(
        port.app().tc_ieee_by_short(child.0),
        None,
        "the Trust Center processed the departure"
    );
    assert_eq!(port.take_device_event(), None);

    // A sleepy child that stops polling ages out (End Device Timeout
    // enumeration 0: 10 s).
    let sleepy = ShortAddress(0x3344);
    let sleepy_ieee = [0xB1; 8];
    {
        let nwk = port.app_mut().device_mut().aps_mut().nwk_mut();
        assert!(nwk.restore_child(sleepy_ieee, sleepy, false, true, false, 0));
        nwk.update_address_map(sleepy, sleepy_ieee);
    }
    for _ in 0..12 {
        pollster::block_on(mac(&mut port).delay_micros(1_000_000));
        steps(&mut port, 1);
    }
    assert_eq!(
        port.take_device_event(),
        Some(DeviceEvent::Left {
            nwk: sleepy.0,
            ieee: sleepy_ieee,
            flags: 0,
            reason: left::REASON_CHILD_TIMEOUT,
        })
    );
}

/// A NWK Rejoin Request (R22 3.4.6) from `source`, secured with the
/// network key, to the parent at `0x0000`.
fn secured_rejoin_request(
    key: &[u8; 16],
    source: (ShortAddress, [u8; 8]),
    frame_counter: u32,
    capability: u8,
) -> MacFrame {
    let header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Command as u8,
            protocol_version: 0x02,
            security: true,
            src_ieee_present: true,
            ..Default::default()
        },
        dst_addr: ShortAddress(0),
        src_addr: source.0,
        radius: 1,
        seq_number: frame_counter as u8,
        dst_ieee: None,
        src_ieee: Some(source.1),
        multicast_control: None,
        source_route: None,
    };
    let mut bytes = [0u8; 128];
    let header_len = header.serialize(&mut bytes);
    let security_header = NwkSecurityHeader {
        security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
        frame_counter,
        source_address: source.1,
        key_seq_number: 0,
    };
    let security_len = security_header.serialize(&mut bytes[header_len..]);
    let aad_len = header_len + security_len;
    let payload = [NwkCommandId::RejoinRequest as u8, capability];
    let encrypted = NwkSecurity::new()
        .encrypt(&bytes[..aad_len], &payload, key, &security_header)
        .unwrap();
    bytes[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
    bytes[header_len] &= !0x07;
    MacFrame::from_slice(&bytes[..aad_len + encrypted.len()]).unwrap()
}

/// The restored-network case seen on hardware: a sleepy child restored
/// into the child table without a Trust Center record or key (an EZSP
/// backup) rejoins secured with the network key while joining is closed.
/// The Trust Center admits it (it proved the network key) and it stays a
/// child, instead of the removal that dropped it.
#[test]
fn a_restored_child_without_a_key_is_admitted_on_its_secured_rejoin() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let key = network_key(&port);
    let child = ShortAddress(0x12D2);
    {
        let nwk = port.app_mut().device_mut().aps_mut().nwk_mut();
        assert!(nwk.restore_child(PEER_IEEE, child, false, true, false, 8));
        nwk.update_address_map(child, PEER_IEEE);
    }
    assert!(port.app().tc_table().device(&PEER_IEEE).is_none());

    // Capability: security capable, allocate address; sleepy RFD.
    receive(
        &mut port,
        child,
        -48,
        secured_rejoin_request(&key, (child, PEER_IEEE), 0x2000, 0x80 | 0x40),
    );
    // It polls for the Rejoin Response (indirect: the Trust Center hears
    // of the rejoin once it is delivered).
    let pan = pan(&port);
    mac(&mut port).enqueue_command_event(MacCommandEvent::DataRequest(MlmeDataRequestIndication {
        source_address: MacAddress::Short(pan, child),
        destination_address: MacAddress::Short(pan, ShortAddress::COORDINATOR),
        lqi: 180,
        security_use: false,
    }));
    steps(&mut port, 8);
    assert_eq!(
        port.take_device_event(),
        Some(DeviceEvent::Joined {
            nwk: child.0,
            ieee: PEER_IEEE,
            parent_nwk: 0x0000,
            join_type: joined::JOIN_SECURED_REJOIN,
            capability: 0x80 | 0x40,
            decision: joined::ADMITTED,
        })
    );
    steps(&mut port, 10);
    assert_eq!(port.take_device_event(), None, "not removed afterwards");
    assert!(
        port.app()
            .device()
            .aps()
            .nwk()
            .neighbor_table()
            .children()
            .any(|entry| entry.ieee_address == PEER_IEEE && entry.network_address == child),
        "still a child at its address"
    );
    let record = *port.app().tc_table().device(&PEER_IEEE).expect("known");
    assert!(
        !record.link_key_known(),
        "its key is unknown, not the global one"
    );
    assert_eq!(port.app().tc_ieee_by_short(child.0), Some(PEER_IEEE));

    // Network-level membership only: its NWK-secured frames are served,
    // but a frame APS-secured with the default global key is not accepted
    // from it (APS does not fall back to that key for it).
    while port.take_indication().is_some() {}
    let plain = aps_data_frame(0x30, None, &[0x01, 0x02, 0x00]);
    receive(
        &mut port,
        child,
        -50,
        secured_nwk_frame(
            &key,
            NwkFrameType::Data,
            (child, PEER_IEEE),
            ShortAddress(0),
            0x2001,
            &plain,
        ),
    );
    assert!(port.take_indication().is_some_and(|i| !i.aps_secured));
    let global = aps_data_frame(
        0x31,
        Some((&DEFAULT_TC_LINK_KEY, PEER_IEEE, 0x40)),
        &[0x01, 0x03, 0x00],
    );
    receive(
        &mut port,
        child,
        -50,
        secured_nwk_frame(
            &key,
            NwkFrameType::Data,
            (child, PEER_IEEE),
            ShortAddress(0),
            0x2002,
            &global,
        ),
    );
    assert_eq!(port.take_indication(), None);
}

/// The Dongle Max capture of a sleepy child's secured rejoin (hil107):
///
/// 1. Rejoin Request (NWK-secured) from 0x12D2, MAC-ACKed;
/// 2. its Data Requests 4 and 6.5 ms later are acknowledged with Frame
///    Pending = 0: the Rejoin Response is queued only ~15 ms after the
///    request, so the child turns its receiver off;
/// 3. the NCP then served the *stale* first poll and sent the response 12 ms
///    after that ACK. The child's radio acknowledged it while idle and its
///    stack dropped it (a receiver that is off hears nothing); later polls
///    found nothing, the child timed out after ~0.45 s and retried, and the
///    Trust Center had already reported the rejoin admitted.
///
/// A poll acknowledged without Frame Pending must leave the frame queued
/// (armed) for the next poll, whose ACK carries Frame Pending = 1 and which
/// the child listens after.
#[test]
fn a_rejoin_response_queued_after_a_poll_was_acked_waits_for_the_next_poll() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let key = network_key(&port);
    let pan = pan(&port);
    let child = ShortAddress(0x12D2);
    {
        let nwk = port.app_mut().device_mut().aps_mut().nwk_mut();
        assert!(nwk.restore_child(PEER_IEEE, child, false, true, false, 8));
        nwk.update_address_map(child, PEER_IEEE);
    }
    let poll = |port: &mut TcPort<_>| {
        mac(port).enqueue_command_event(MacCommandEvent::DataRequest(MlmeDataRequestIndication {
            source_address: MacAddress::Short(pan, child),
            destination_address: MacAddress::Short(pan, ShortAddress::COORDINATOR),
            lqi: 180,
            security_use: false,
        }));
    };
    let to_child = |port: &mut TcPort<_>| {
        mac(port)
            .tx_history()
            .iter()
            .filter(|tx| tx.dst == MacAddress::Short(pan, child))
            .count()
    };
    fn queued<A: TcApp>(port: &mut TcPort<A>, child: ShortAddress) -> usize {
        port.app_mut()
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .indirect_queue()
            .pending_count(child)
    }
    mac(&mut port).clear_tx_history();

    // (1) + (2): the request; the response is queued; then the stack
    // serves a poll that the radio had acknowledged before the response
    // existed (FP = 0).
    receive(
        &mut port,
        child,
        -33,
        secured_rejoin_request(&key, (child, PEER_IEEE), 1227, 0x80),
    );
    assert_eq!(queued(&mut port, child), 1, "the Rejoin Response is queued");
    mac(&mut port).set_poll_acked_without_frame_pending(true);
    poll(&mut port);
    steps(&mut port, 4);
    assert_eq!(to_child(&mut port), 0, "nothing after an ACK without FP");
    assert_eq!(queued(&mut port, child), 1, "the Rejoin Response waits");
    assert!(
        mac(&mut port)
            .indirect_pending_history()
            .iter()
            .rev()
            .find(|(address, _)| *address == MacAddress::Short(pan, child))
            .is_some_and(|(_, armed)| *armed),
        "Frame Pending stays armed for the next poll"
    );
    assert_eq!(port.take_device_event(), None, "not delivered yet");

    // (3): the next poll is acknowledged with FP = 1 and gets the response.
    mac(&mut port).set_poll_acked_without_frame_pending(false);
    poll(&mut port);
    steps(&mut port, 4);
    assert_eq!(to_child(&mut port), 1, "delivered at the next poll");
    assert_eq!(queued(&mut port, child), 0);
    let response: Vec<u8> = mac(&mut port)
        .tx_history()
        .iter()
        .find(|tx| tx.dst == MacAddress::Short(pan, child))
        .map(|tx| tx.payload.as_slice().to_vec())
        .unwrap();
    let (header, _) = NwkHeader::parse(&response).unwrap();
    assert_eq!(header.frame_control.frame_type, NwkFrameType::Command as u8);
    assert_eq!(header.dst_ieee, Some(PEER_IEEE));
    // The Trust Center hears of the rejoin once the response is delivered.
    steps(&mut port, 6);
    assert!(matches!(
        port.take_device_event(),
        Some(DeviceEvent::Joined {
            nwk: 0x12D2,
            join_type: joined::JOIN_SECURED_REJOIN,
            decision: joined::ADMITTED,
            ..
        })
    ));
    // The child is an authenticated child (the secured rejoin proved the
    // network key): it never expires as unauthenticated, and its
    // Device_annce is served.
    assert!(
        port.app()
            .device()
            .aps()
            .nwk()
            .neighbor_table()
            .children()
            .any(|entry| entry.network_address == child
                && entry.relationship == zigbee_nwk::neighbor::Relationship::Child)
    );
}

#[test]
fn ack_request_confirms_wait_for_the_aps_ack_outcome() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let key = network_key(&port);
    let child = associate(&mut port, PEER_IEEE);
    while port.take_device_event().is_some() {}

    let request = |req_tid, tx_options| DataRequest {
        req_tid,
        dst: Destination::Nwk(child.0),
        dst_ep: 1,
        src_ep: 1,
        profile: PROFILE_HOME_AUTOMATION,
        cluster: 0x0006,
        tx_options,
        radius: 0,
        asdu: heapless::Vec::from_slice(&[0x01, 0x07, 0x02]).unwrap(),
    };

    // Without ack_request: confirmed at once.
    let confirm = pollster::block_on(port.send(&request(1, 0))).unwrap();
    assert!(!confirm.deferred());
    assert_eq!(port.take_data_confirm(), None);

    // With it: deferred until the APS ACK, while the stack keeps stepping.
    let confirm = pollster::block_on(port.send(&request(2, 0x01))).unwrap();
    assert!(confirm.deferred());
    assert_eq!(confirm.dst_nwk, child.0);
    steps(&mut port, 2);
    assert_eq!(port.take_data_confirm(), None, "no ACK yet");
    receive(
        &mut port,
        child,
        -50,
        secured_nwk_frame(
            &key,
            NwkFrameType::Data,
            (child, PEER_IEEE),
            ShortAddress(0),
            0x20,
            &aps_ack_frame(confirm.aps_counter),
        ),
    );
    let done = port.take_data_confirm().expect("ACK outcome");
    assert_eq!(done.req_tid, 2);
    assert_eq!(
        done.confirm,
        DataConfirm {
            zb_status: 0,
            aps_counter: confirm.aps_counter,
            dst_nwk: child.0,
            flags: DataConfirm::FLAG_ACK_RECEIVED,
        }
    );

    // No ACK: the APS retries run out (apscMaxFrameRetries windows).
    let confirm = pollster::block_on(port.send(&request(3, 0x01))).unwrap();
    assert!(confirm.deferred());
    for _ in 0..4 {
        assert_eq!(port.take_data_confirm(), None);
        pollster::block_on(mac(&mut port).delay_micros(zigbee_aps::APS_ACK_WAIT_DURATION_US));
        steps(&mut port, 2);
    }
    let done = port.take_data_confirm().expect("NO_ACK outcome");
    assert_eq!(done.req_tid, 3);
    assert_eq!((done.confirm.zb_status, done.confirm.flags), (0xA7, 0));
    assert_eq!(port.take_data_confirm(), None);

    // A pending send fails when the network is reset.
    let confirm = pollster::block_on(port.send(&request(4, 0x01))).unwrap();
    assert!(confirm.deferred());
    pollster::block_on(port.leave_reset()).unwrap();
    let done = port.take_data_confirm().expect("reset outcome");
    assert_eq!((done.req_tid, done.confirm.zb_status), (4, 0xA7));
}

/// `zb_status` uses the R22 values: APS Table 2-27 for APS outcomes, and the
/// NLDE-DATA.confirm status for a route failure (R22 2.2.4.1.2).
#[test]
fn data_confirm_statuses_follow_r22() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let request = |req_tid, dst| DataRequest {
        req_tid,
        dst,
        dst_ep: 1,
        src_ep: 1,
        profile: PROFILE_HOME_AUTOMATION,
        cluster: 0x0006,
        tx_options: 0,
        radius: 0,
        asdu: heapless::Vec::from_slice(&[0x01, 0x07, 0x02]).unwrap(),
    };
    let zb_status = |result| match result {
        Ok(confirm) => confirm,
        Err(OpError::Zigbee(status)) => status,
        Err(other) => panic!("not a Zigbee outcome: {other:?}"),
    };

    // An IEEE destination missing from the address map: NO_SHORT_ADDRESS.
    let unknown_ieee = pollster::block_on(port.send(&request(1, Destination::Ieee([0xC3; 8]))))
        .map(|confirm| confirm.zb_status);
    assert_eq!(zb_status(unknown_ieee), 0xA9);

    // A unicast nobody routes: NWK ROUTE_DISCOVERY_FAILED, passed through.
    let unroutable = pollster::block_on(port.send(&request(2, Destination::Nwk(0x7777))))
        .map(|confirm| confirm.zb_status);
    assert_eq!(zb_status(unroutable), 0xD0);
}

/// Spec 7.3 `tx_options` bit5 / sleepy children: a read sent right after a
/// 30 s long-polling child's poll stays queued until its next poll, and the
/// confirm waits for the APS ACK that follows — instead of NO_ACK after the
/// fixed ~6 s of APS retries while the frame expired in the 8 s queue.
#[test]
fn a_long_polling_sleepy_child_is_reached_at_its_next_poll() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let key = network_key(&port);
    let pan = pan(&port);
    let child = ShortAddress(0x3344);
    let child_ieee = [0xB1; 8];
    {
        let nwk = port.app_mut().device_mut().aps_mut().nwk_mut();
        assert!(nwk.restore_child(child_ieee, child, false, true, false, 8));
        nwk.update_address_map(child, child_ieee);
    }
    let poll = |port: &mut TcPort<_>| {
        mac(port).enqueue_command_event(MacCommandEvent::DataRequest(MlmeDataRequestIndication {
            source_address: MacAddress::Short(pan, child),
            destination_address: MacAddress::Short(pan, ShortAddress::COORDINATOR),
            lqi: 180,
            security_use: false,
        }));
        steps(port, 1);
    };
    let wait_secs = |port: &mut TcPort<_>, secs: u32| {
        for _ in 0..secs {
            pollster::block_on(mac(port).delay_micros(1_000_000));
            steps(port, 1);
        }
    };
    // Two long polls teach the NCP the child's 30 s interval.
    poll(&mut port);
    wait_secs(&mut port, 30);
    poll(&mut port);

    let request = DataRequest {
        req_tid: 7,
        dst: Destination::Nwk(child.0),
        dst_ep: 1,
        src_ep: 1,
        profile: PROFILE_HOME_AUTOMATION,
        cluster: 0x0000,
        tx_options: 0x21,
        radius: 0,
        asdu: heapless::Vec::from_slice(&[0x00, 0x01, 0x00, 0x04, 0x00]).unwrap(),
    };
    let to_child = |port: &mut TcPort<_>| {
        mac(port)
            .tx_history()
            .iter()
            .filter(|tx| tx.dst == MacAddress::Short(pan, child))
            .count()
    };
    let sent = to_child(&mut port);
    let confirm = pollster::block_on(port.send(&request)).unwrap();
    assert!(confirm.deferred());
    wait_secs(&mut port, 29);
    assert_eq!(port.take_data_confirm(), None, "still waiting for the poll");
    assert_eq!(
        to_child(&mut port),
        sent,
        "nothing went to the child before its poll"
    );

    poll(&mut port);
    let delivered_indirect = mac(&mut port).tx_history().last().unwrap().indirect;
    assert_eq!(to_child(&mut port), sent + 1, "delivered at the poll");
    assert!(delivered_indirect, "as the indirect transaction");
    assert_eq!(
        port.app_mut()
            .device_mut()
            .aps_mut()
            .nwk_mut()
            .indirect_queue()
            .pending_count(child),
        0,
        "and no APS retry was queued beside it"
    );
    receive(
        &mut port,
        child,
        -60,
        secured_nwk_frame(
            &key,
            NwkFrameType::Data,
            (child, child_ieee),
            ShortAddress(0),
            0x30,
            &aps_ack_frame(confirm.aps_counter),
        ),
    );
    let done = port.take_data_confirm().expect("ACK outcome");
    assert_eq!(done.req_tid, 7);
    assert_eq!(done.confirm.zb_status, 0);
    assert_eq!(done.confirm.flags, DataConfirm::FLAG_ACK_RECEIVED);
}

// ── The host side (ZRHP over an in-memory pipe) ────────────────────

#[derive(Default)]
struct Pipe {
    to_ncp: VecDeque<u8>,
    to_host: Vec<u8>,
}

impl HostTransport for Pipe {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        let n = buf.len().min(self.to_ncp.len());
        for slot in &mut buf[..n] {
            *slot = self.to_ncp.pop_front().unwrap();
        }
        n
    }
    fn write(&mut self, data: &[u8]) -> usize {
        self.to_host.extend_from_slice(data);
        data.len()
    }
    fn info(&self) -> TransportInfo {
        TransportInfo {
            kind: 1,
            baud: 0,
            flow_control: false,
        }
    }
}

#[derive(Debug, Clone)]
struct Received {
    kind: Kind,
    msg_id: u16,
    tid: u16,
    payload: Vec<u8>,
}

impl Received {
    fn raw(&self) -> RawMessage<'_> {
        RawMessage {
            header: MsgHeader {
                kind: self.kind,
                retry: false,
                msg_id: self.msg_id,
                tid: self.tid,
                len: self.payload.len() as u16,
            },
            payload: &self.payload,
        }
    }
}

struct Host {
    link: Link<32, 32>,
    queues: EndpointQueues<4096>,
    tid: u16,
    inbox: Vec<Received>,
}

impl Host {
    fn new(now: u64) -> Self {
        let mut link = Link::new(LinkConfig::desktop_host(), 0x1A2B_3C4D);
        link.open(now, zigbee_zrhp::frame::ResetReason::HostOpen);
        Self {
            link,
            queues: EndpointQueues::new(),
            tid: 0,
            inbox: Vec::new(),
        }
    }

    fn request<'a, T: zigbee_zrhp::codec::Wire<'a> + zigbee_zrhp::msg::MessageDef>(
        &mut self,
        body: &T,
    ) -> u16 {
        self.tid += 1;
        self.queues.request(self.tid, body).unwrap();
        self.tid
    }

    fn pump(&mut self, now: u64, pipe: &mut Pipe) {
        let bytes = std::mem::take(&mut pipe.to_host);
        let mut offset = 0;
        loop {
            offset += self.link.push_rx(&bytes[offset..], now);
            while let Some(event) = self.link.poll_event() {
                if let LinkEvent::Data { payload, .. } = event {
                    for message in Batch::parse(payload).unwrap().iter() {
                        self.inbox.push(Received {
                            kind: message.header.kind,
                            msg_id: message.header.msg_id,
                            tid: message.header.tid,
                            payload: message.payload.to_vec(),
                        });
                    }
                }
            }
            if offset >= bytes.len() {
                break;
            }
        }
        let mut tx = [0u8; TX_BUFFER_LEN];
        while let Some(n) = self.link.poll_transmit(now, &mut self.queues, &mut tx) {
            pipe.to_ncp.extend(&tx[..n]);
        }
    }

    fn status(&mut self, tid: u16) -> u8 {
        let index = self
            .inbox
            .iter()
            .position(|m| m.kind == Kind::Response && m.tid == tid)
            .unwrap_or_else(|| panic!("no response to tid {tid}: {:?}", self.inbox));
        self.inbox.remove(index).payload[0]
    }

    fn event(&mut self, msg_id: u16) -> Option<Received> {
        let index = self
            .inbox
            .iter()
            .position(|m| m.kind == Kind::Event && m.msg_id == msg_id)?;
        Some(self.inbox.remove(index))
    }
}

fn identity() -> NcpIdentity {
    NcpIdentity {
        fw_version: "test",
        build_id: [0; 8],
        chip: 1,
        chip_rev: 0,
        flash_size: 4 * 1024 * 1024,
        board: "pipe",
        product: "test",
        reset_reason: 1,
        key_export: true,
        max_children: 64,
        max_tc_devices: 256,
        stack_allowed: true,
    }
}

struct Session {
    ncp: Ncp<8>,
    host: Host,
    pipe: Pipe,
    log: LogRing<8>,
    now: u64,
}

impl Session {
    fn negotiated<A: TcApp>(port: &mut TcPort<A>) -> Self {
        let now = 1_000;
        let mut session = Self {
            ncp: Ncp::new(identity(), 0xC0FF_EE02, now),
            host: Host::new(now),
            pipe: Pipe::default(),
            log: LogRing::new(),
            now,
        };
        let negotiate = session.host.request(&SysNegotiateReq {
            major_min: 1,
            major_max: 1,
            host_caps: 0,
            name: Bytes8(b"pipe-host"),
        });
        session.exchange(port);
        let index = session
            .host
            .inbox
            .iter()
            .position(|m| m.kind == Kind::Response && m.tid == negotiate)
            .unwrap();
        let response = session.host.inbox.remove(index);
        let negotiated: SysNegotiateRsp = response
            .raw()
            .decode_response::<SysNegotiateRsp>()
            .unwrap()
            .body
            .unwrap();
        assert_eq!(negotiated.capacity(8), Some(8), "max_host_endpoints");
        session
    }

    fn exchange<A: TcApp>(&mut self, port: &mut TcPort<A>) {
        for _ in 0..8 {
            self.now += 1_000;
            self.host.pump(self.now, &mut self.pipe);
            assert!(
                self.ncp
                    .service(self.now, &mut self.pipe, port, &self.log)
                    .is_none()
            );
            while pollster::block_on(self.ncp.drive(port, || 0)) {}
            if port.running() {
                pollster::block_on(port.step()).unwrap();
            }
            self.ncp.service(self.now, &mut self.pipe, port, &self.log);
            self.host.pump(self.now, &mut self.pipe);
        }
    }
}

fn add_endpoint(ep: u8, in_clusters: &[u16]) -> NwkAddEndpoint<'_> {
    NwkAddEndpoint {
        ep,
        profile: PROFILE_HOME_AUTOMATION,
        device_id: 0x0005,
        version: 1,
        in_clusters: List::new(in_clusters),
        out_clusters: List::new(&[]),
    }
}

fn zdo_endpoints<A: TcApp>(port: &TcPort<A>) -> Vec<u8> {
    port.app()
        .device()
        .bdb()
        .zdo()
        .endpoints()
        .iter()
        .map(|descriptor| descriptor.endpoint)
        .collect()
}

#[test]
fn host_endpoints_are_registered_with_the_zdo_server_and_cleared() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let mut session = Session::negotiated(&mut port);
    assert_eq!(zdo_endpoints(&port), [1], "the product profile's endpoint");

    // Endpoint 1 replaces the profile's; 2..=8 are added; a 9th is refused.
    let clusters = [0x0000, 0x0006];
    let mut tids = Vec::new();
    for ep in 1..=9 {
        tids.push(session.host.request(&add_endpoint(ep, &clusters)));
    }
    let invalid = session.host.request(&add_endpoint(0, &clusters));
    let too_many = session.host.request(&add_endpoint(242, &[0; 17]));
    let again = session.host.request(&add_endpoint(3, &[0x0300]));
    session.exchange(&mut port);
    for (ep, tid) in (1..=9).zip(tids) {
        let expected = if ep == 9 {
            Status::NoSpace
        } else {
            Status::Success
        };
        assert_eq!(session.host.status(tid), expected.to_u8(), "endpoint {ep}");
    }
    assert_eq!(
        session.host.status(invalid),
        Status::InvalidParameter.to_u8()
    );
    assert_eq!(
        session.host.status(too_many),
        Status::InvalidParameter.to_u8()
    );
    assert_eq!(session.host.status(again), Status::Success.to_u8());
    assert_eq!(zdo_endpoints(&port), [1, 2, 3, 4, 5, 6, 7, 8]);
    let zdo = port.app().device().bdb().zdo();
    assert_eq!(zdo.find_endpoint(1).unwrap().device_id, 0x0005);
    assert_eq!(zdo.find_endpoint(3).unwrap().input_clusters, [0x0300]);

    // NWK_LEAVE_RESET keeping registrations (flags bit1), then without.
    let keep = session.host.request(&NwkLeaveReset { flags: 0x02 });
    session.exchange(&mut port);
    assert_eq!(session.host.status(keep), Status::Success.to_u8());
    assert_eq!(zdo_endpoints(&port), [1, 2, 3, 4, 5, 6, 7, 8]);
    let clear = session.host.request(&NwkLeaveReset { flags: 0 });
    session.exchange(&mut port);
    assert_eq!(session.host.status(clear), Status::Success.to_u8());
    assert!(zdo_endpoints(&port).is_empty());
}

#[test]
fn the_host_receives_the_deferred_confirm_after_the_aps_ack() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let key = network_key(&port);
    let child = associate(&mut port, PEER_IEEE);
    let mut session = Session::negotiated(&mut port);

    let mut request = ApsDataRequest {
        dst: ApsDst::Nwk(child.0),
        dst_ep: 1,
        src_ep: 1,
        profile: PROFILE_HOME_AUTOMATION,
        cluster: 0x0006,
        tx_options: 0x01,
        radius: 0,
        non_member_radius: 0,
        priority: 1,
        relays: None,
        alias: None,
        asdu: &[0x01, 0x08, 0x02],
    };
    let acked = session.host.request(&request);
    session.exchange(&mut port);
    assert_eq!(session.host.status(acked), Status::Success.to_u8());
    assert!(
        session.host.event(ids::EVT_APS_DATA_CONFIRM).is_none(),
        "the confirm waits for the APS ACK"
    );
    // The frame is NWK-encrypted on air: take its APS counter from the
    // NCP's ACK tracking.
    let aps_counter = port
        .app()
        .device()
        .aps()
        .last_ack_handle()
        .expect("the unicast awaits its ACK")
        .aps_counter();
    receive(
        &mut port,
        child,
        -50,
        secured_nwk_frame(
            &key,
            NwkFrameType::Data,
            (child, PEER_IEEE),
            ShortAddress(0),
            0x30,
            &aps_ack_frame(aps_counter),
        ),
    );
    session.exchange(&mut port);
    let confirm = session
        .host
        .event(ids::EVT_APS_DATA_CONFIRM)
        .expect("EVT_APS_DATA_CONFIRM");
    let confirm: EvtApsDataConfirm = confirm.raw().decode().unwrap();
    assert_eq!(
        (
            confirm.req_tid,
            confirm.status,
            confirm.zb_status,
            confirm.flags
        ),
        (acked, 0, 0, 0x01)
    );
    assert_eq!(confirm.aps_counter, aps_counter);

    // Without ack_request the confirm follows the transmission.
    request.tx_options = 0;
    let plain = session.host.request(&request);
    session.exchange(&mut port);
    assert_eq!(session.host.status(plain), Status::Success.to_u8());
    let confirm = session.host.event(ids::EVT_APS_DATA_CONFIRM).unwrap();
    let confirm: EvtApsDataConfirm = confirm.raw().decode().unwrap();
    assert_eq!((confirm.req_tid, confirm.flags), (plain, 0));
}

/// `EVT_PERMIT_JOIN_STATE` (7.2): opening, re-arming (here by the stack,
/// as a `Mgmt_Permit_Joining_req` from the network does), expiry and every
/// `NWK_PERMIT_JOIN` are reported; the per-second countdown is not.
#[test]
fn permit_join_state_follows_open_rearm_and_expiry() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    formed_port!(port);
    let mut session = Session::negotiated(&mut port);
    let states = |session: &mut Session| -> Vec<u8> {
        let mut out = Vec::new();
        while let Some(event) = session.host.event(ids::EVT_PERMIT_JOIN_STATE) {
            out.push(
                event
                    .raw()
                    .decode::<EvtPermitJoinState>()
                    .unwrap()
                    .remaining_s,
            );
        }
        out
    };
    assert_eq!(states(&mut session), [0u8; 0], "closed: nothing to report");

    let open = session.host.request(&NwkPermitJoin {
        duration_s: 5,
        target: 0x0000,
        flags: 0,
    });
    session.exchange(&mut port);
    assert_eq!(session.host.status(open), Status::Success.to_u8());
    assert_eq!(states(&mut session), [5]);

    // Counting down: silent.
    for _ in 0..2 {
        pollster::block_on(mac(&mut port).delay_micros(1_000_000));
        session.exchange(&mut port);
    }
    assert!(port.permit_join_remaining_s() < 5);
    assert_eq!(states(&mut session), [0u8; 0]);

    // Re-armed outside NWK_PERMIT_JOIN (a Mgmt_Permit_Joining_req).
    pollster::block_on(port.permit_join(30, 0x0000)).unwrap();
    session.exchange(&mut port);
    assert_eq!(states(&mut session), [30]);

    // Expiry.
    for _ in 0..32 {
        pollster::block_on(mac(&mut port).delay_micros(1_000_000));
        session.exchange(&mut port);
    }
    assert_eq!(port.permit_join_remaining_s(), 0);
    assert_eq!(states(&mut session), [0]);

    // A host close while closed still answers with the state.
    let close = session.host.request(&NwkPermitJoin {
        duration_s: 0,
        target: 0x0000,
        flags: 0,
    });
    session.exchange(&mut port);
    assert_eq!(session.host.status(close), Status::Success.to_u8());
    assert_eq!(states(&mut session), [0]);

    // A new session learns an open window once.
    pollster::block_on(port.permit_join(60, 0x0000)).unwrap();
    let mut again = Session::negotiated(&mut port);
    again.exchange(&mut port);
    let reported = states(&mut again);
    assert_eq!(reported.len(), 1);
    assert!(reported[0] > 50);
}
