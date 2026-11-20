//! End to end on a combo router (`green-power-translation-table`): a
//! commissioning tool pairs a GPD with GP Pairing Configuration and
//! retargets its Toggle with GP Translation Table Update; the GPD's Toggle
//! then drives another endpoint with another command, and both survive a
//! reboot through the GP journal.

use core::future::Future;
use core::task::{Context, Poll, Waker};

use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsTxOptions};
use zigbee_gp::cluster::{Pairing, client_cmd, server_cmd};
use zigbee_gp::config::{
    PairingConfiguration, PairingSecurity, TranslationRecord, TranslationTableResponse,
    TranslationTableUpdate, TtPayload, pairing_action, tt_action,
};
use zigbee_gp::frame::{FrameType, GpdfParams};
use zigbee_gp::security;
use zigbee_gp::types::{CommunicationMode, GpdId, KeyType, SecurityLevel, derive_alias};
use zigbee_mac::mock::MockMac;
use zigbee_mac::{MacDriver, MacFrame, McpsDataIndication};
use zigbee_nwk::DeviceType;
use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};
use zigbee_zcl::clusters::on_off::OnOffCluster;

use crate::role::Router;
use crate::{ClusterRef, ZigbeeDevice};

const PAN: PanId = PanId(0x1234);
const ROUTER: ShortAddress = ShortAddress(0x0001);
const ROUTER_IEEE: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];
const COORDINATOR_IEEE: IeeeAddress = [0x44; 8];
const NETWORK_KEY: [u8; 16] = [0x55; 16];
const SRC: u32 = 0x7A6B_9879;
const KEY: [u8; 16] = [0xC5; 16];

fn block_on<F: Future>(future: F) -> F::Output {
    let mut context = Context::from_waker(Waker::noop());
    let mut future = core::pin::pin!(future);
    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        std::thread::yield_now();
    }
}

fn network(device: &mut ZigbeeDevice<MockMac, Router>, nwk: ShortAddress, depth: u8) {
    device.bdb_mut().attributes_mut().node_is_on_a_network = true;
    let aps = device.bdb_mut().zdo_mut().aps_mut();
    aps.aib_mut().aps_trust_center_address = COORDINATOR_IEEE;
    let nwk_layer = aps.nwk_mut();
    nwk_layer.set_joined(true);
    nwk_layer.security_mut().set_network_key(NETWORK_KEY, 0);
    let nib = nwk_layer.nib_mut();
    nib.pan_id = PAN;
    nib.network_address = nwk;
    nib.extended_pan_id = [0xA5; 8];
    nib.depth = depth;
    nib.set_nwk_update_id(7);
    nib.security_enabled = true;
    nib.active_key_seq_number = 0;
    nib.outgoing_frame_counter_limit = 0x400;
    nib.logical_channel = 15;
}

/// The combo light: endpoint 1 (the sink's endpoint) and endpoint 2, both
/// On/Off lights.
fn combo() -> ZigbeeDevice<MockMac, Router> {
    let mut device = ZigbeeDevice::builder(MockMac::new(ROUTER_IEEE))
        .device_type(DeviceType::Router)
        .endpoint(1, 0x0104, crate::DeviceId::ON_OFF_LIGHT, |endpoint| {
            endpoint.cluster_server(crate::ClusterId::ON_OFF)
        })
        .endpoint(2, 0x0104, crate::DeviceId::ON_OFF_LIGHT, |endpoint| {
            endpoint.cluster_server(crate::ClusterId::ON_OFF)
        })
        .build_router();
    network(&mut device, ROUTER, 1);
    let nib = device.bdb_mut().zdo_mut().aps_mut().nwk_mut().nib_mut();
    nib.ieee_address = ROUTER_IEEE;
    nib.parent_address = ShortAddress::COORDINATOR;
    device
        .gp_sink_mut()
        .set_config(zigbee_gp::sink::SinkConfig {
            endpoint: Some(1),
            ..zigbee_gp::sink::SinkConfig::default()
        });
    device
}

/// The commissioning tool (the coordinator).
fn tool() -> ZigbeeDevice<MockMac, Router> {
    let mut device = ZigbeeDevice::builder(MockMac::new(COORDINATOR_IEEE)).build_coordinator();
    let mac = device.mac_mut();
    for (attribute, value) in [
        (
            zigbee_mac::PibAttribute::MacPanId,
            zigbee_mac::PibValue::PanId(PAN),
        ),
        (
            zigbee_mac::PibAttribute::MacShortAddress,
            zigbee_mac::PibValue::ShortAddress(ShortAddress::COORDINATOR),
        ),
        (
            zigbee_mac::PibAttribute::PhyCurrentChannel,
            zigbee_mac::PibValue::U8(15),
        ),
    ] {
        block_on(mac.mlme_set(attribute, value)).unwrap();
    }
    network(&mut device, ShortAddress::COORDINATOR, 0);
    let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
    nwk.nib_mut().ieee_address = COORDINATOR_IEEE;
    // The combo is a neighbor: unicasts go straight to it.
    nwk.update_neighbor_address(ROUTER, ROUTER_IEEE);
    device
}

struct Lights {
    one: OnOffCluster,
    two: OnOffCluster,
}

fn process(
    router: &mut ZigbeeDevice<MockMac, Router>,
    lights: &mut Lights,
    frame: &McpsDataIndication,
) -> Option<crate::event_loop::StackEvent> {
    let mut clusters = [
        ClusterRef {
            endpoint: 1,
            cluster: &mut lights.one,
        },
        ClusterRef {
            endpoint: 2,
            cluster: &mut lights.two,
        },
    ];
    block_on(router.process_incoming(frame, &mut clusters))
}

/// A GP cluster command from the tool's endpoint 242, as the router's MAC
/// receives it.
fn send_gp(
    tool: &mut ZigbeeDevice<MockMac, Router>,
    dst: ApsAddress,
    zcl: &[u8],
) -> McpsDataIndication {
    tool.mac_mut().clear_tx_history();
    block_on(
        tool.bdb
            .zdo_mut()
            .aps_mut()
            .apsde_data_request(&ApsdeDataRequest {
                dst_addr_mode: match dst {
                    ApsAddress::Group(_) => ApsAddressMode::Group,
                    _ => ApsAddressMode::Short,
                },
                dst_address: dst,
                dst_endpoint: 242,
                profile_id: 0xA1E0,
                cluster_id: 0x0021,
                src_endpoint: 242,
                payload: zcl,
                tx_options: ApsTxOptions {
                    use_nwk_key: true,
                    ..ApsTxOptions::default()
                },
                radius: 0,
                alias_src_addr: None,
                alias_seq: None,
            }),
    )
    .unwrap();
    let dst_mac = match dst {
        ApsAddress::Short(a) if a.0 < 0xFFF8 => a,
        _ => ShortAddress(0xFFFF),
    };
    McpsDataIndication {
        src_address: MacAddress::Short(PAN, ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PAN, dst_mac),
        lqi: 200,
        payload: MacFrame::from_slice(
            tool.mac()
                .tx_history()
                .last()
                .expect("the tool sent nothing")
                .payload
                .as_slice(),
        )
        .unwrap(),
        security_use: false,
    }
}

/// ZCL client → server frame of GP command `command` (Default Response
/// enabled).
fn zcl(tsn: u8, command: u8, payload: &[u8]) -> heapless::Vec<u8, 96> {
    let mut frame = heapless::Vec::from_slice(&[0x01, tsn, command]).unwrap();
    frame.extend_from_slice(payload).unwrap();
    frame
}

/// The APS payload of a frame the router sent, decrypted by the tool.
fn received(tool: &mut ZigbeeDevice<MockMac, Router>, nwk: &[u8]) -> heapless::Vec<u8, 128> {
    let frame: heapless::Vec<u8, 128> = heapless::Vec::from_slice(nwk).unwrap();
    match block_on(
        tool.bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .process_incoming_nwk_frame(&frame, 200),
    ) {
        Some(zigbee_nwk::nlde::NwkIndication::Owned(aps)) => {
            heapless::Vec::from_slice(&aps.payload).unwrap()
        }
        other => panic!("undecodable frame {other:?}"),
    }
}

/// ZCL frames (frame control, command, payload) the router sent on
/// endpoint 242.
fn sent_gp(
    router: &ZigbeeDevice<MockMac, Router>,
    tool: &mut ZigbeeDevice<MockMac, Router>,
) -> heapless::Vec<(u8, u8, heapless::Vec<u8, 96>), 8> {
    let mut out = heapless::Vec::new();
    for tx in router.mac().tx_history() {
        let aps = received(tool, tx.payload.as_slice());
        // APS data: FC, dst EP (unicast) or group, cluster, profile, src EP,
        // counter; only GP cluster frames from endpoint 242 are collected.
        let (header, cluster) = match aps[0] & 0x0C {
            0x0C => (9, u16::from_le_bytes([aps[3], aps[4]])),
            _ => (8, u16::from_le_bytes([aps[2], aps[3]])),
        };
        if cluster != 0x0021 {
            continue;
        }
        let z = &aps[header..];
        let _ = out.push((z[0], z[2], heapless::Vec::from_slice(&z[3..]).unwrap()));
    }
    out
}

fn gpdf(counter: u32, command: u8) -> McpsDataIndication {
    let mut raw = [0u8; 127];
    let n = security::build_gpdf(
        &mut MockMac::new([0; 8]),
        Some(&KEY),
        &GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(SRC),
            security_level: SecurityLevel::Encrypted,
            individual_key: true,
            rx_after_tx: false,
            to_gpd: false,
            frame_counter: counter,
        },
        counter as u8,
        command,
        &[],
        &mut raw,
    )
    .unwrap();
    McpsDataIndication {
        src_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
        dst_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
        lqi: 180,
        payload: MacFrame::from_slice(&raw[7..n])
            .unwrap()
            .with_mac_sequence(counter as u8),
        security_use: false,
    }
}

#[test]
fn pairing_configuration_and_translation_update_retarget_a_gpd_toggle() {
    let gpd = GpdId::SrcId(SRC);
    let alias = derive_alias(&gpd);
    let mut router = combo();
    let mut tool = tool();
    let mut lights = Lights {
        one: OnOffCluster::new(),
        two: OnOffCluster::new(),
    };

    // 1. GP Pairing Configuration (Replace, send GP Pairing), unicast.
    let pc = PairingConfiguration {
        actions: pairing_action::REPLACE | pairing_action::SEND_GP_PAIRING,
        gpd_id: gpd,
        communication_mode: CommunicationMode::DerivedGroupcast,
        mac_seq_cap: true,
        rx_on_cap: false,
        fixed_location: true,
        device_id: 0x02,
        groups: &[],
        assigned_alias: None,
        groupcast_radius: 0,
        security: Some(PairingSecurity {
            level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key: KEY,
        }),
        frame_counter: Some(10),
        number_of_paired_endpoints: 0xFE,
        paired_endpoints: &[],
        application_information: None,
    };
    let mut payload = [0u8; 64];
    let n = pc.encode(&mut payload).unwrap();
    let frame = send_gp(
        &mut tool,
        ApsAddress::Short(ROUTER),
        &zcl(0x21, server_cmd::PAIRING_CONFIGURATION, &payload[..n]),
    );
    router.mac_mut().clear_tx_history();
    process(&mut router, &mut lights, &frame);
    let entry = &router.gp_sink().entries()[0];
    assert_eq!(
        (entry.gpd_id, entry.key, entry.frame_counter),
        (gpd, Some(KEY), 10)
    );
    assert!(
        router
            .bdb
            .zdo()
            .aps()
            .group_member_endpoints(alias)
            .contains(&242)
    );
    while block_on(async {
        router.gp_poll().await;
        router.gp_next_wait_us() == Some(0)
    }) {}
    let sent = sent_gp(&router, &mut tool);
    // Default Response SUCCESS to the tool, then GP Pairing (AddSink).
    assert!(
        sent.iter()
            .any(|(fc, cmd, p)| fc & 0x03 == 0 && *cmd == 0x0B && p[..] == [0x09, 0x00]),
        "{sent:?}"
    );
    let pairing = sent
        .iter()
        .find(|(fc, cmd, _)| fc & 0x09 == 0x09 && *cmd == client_cmd::PAIRING)
        .expect("GP Pairing");
    let p = Pairing::decode(&pairing.2).unwrap();
    assert!(p.add_sink && p.key == Some(KEY));

    // 2. GP Translation Table Update: Toggle → endpoint 2, On/Off On.
    let record = TranslationRecord {
        index: 0xFF,
        gpd_command: 0x22,
        endpoint: 2,
        profile: 0x0104,
        cluster: 0x0006,
        zcl_command: 0x01,
        payload: TtPayload::Fixed(&[]),
        additional_info: None,
    };
    let n = TranslationTableUpdate::encode(&gpd, tt_action::ADD, &[record], &mut payload).unwrap();
    let frame = send_gp(
        &mut tool,
        ApsAddress::Short(ROUTER),
        &zcl(0x22, server_cmd::TRANSLATION_TABLE_UPDATE, &payload[..n]),
    );
    router.mac_mut().clear_tx_history();
    process(&mut router, &mut lights, &frame);
    assert_eq!(router.gp_sink().translations().len(), 1);

    // 3. The GPD's Toggle switches endpoint 2 on; endpoint 1 is untouched.
    let event = process(&mut router, &mut lights, &gpdf(11, 0x22));
    assert!(matches!(
        event,
        Some(crate::event_loop::StackEvent::CommandReceived {
            endpoint: 2,
            source_endpoint: 242,
            command_id: 0x01,
            ..
        })
    ));
    assert!(lights.two.is_on());
    assert!(!lights.one.is_on());
    // On has no explicit translation: the built-in one drives endpoint 1.
    process(&mut router, &mut lights, &gpdf(12, 0x21));
    assert!(lights.one.is_on());

    // 4. GP Translation Table Request → Response with the entry.
    let frame = send_gp(
        &mut tool,
        ApsAddress::Short(ROUTER),
        &zcl(0x23, server_cmd::TRANSLATION_TABLE_REQUEST, &[0x00]),
    );
    router.mac_mut().clear_tx_history();
    process(&mut router, &mut lights, &frame);
    block_on(router.gp_poll());
    let sent = sent_gp(&router, &mut tool);
    let (_, _, response) = sent
        .iter()
        .find(|(fc, cmd, _)| fc & 0x08 != 0 && *cmd == client_cmd::TRANSLATION_TABLE_RESPONSE)
        .expect("Translation Table Response");
    let response = TranslationTableResponse::decode(response).unwrap();
    assert_eq!((response.status, response.total, response.len()), (0, 1, 1));
    assert_eq!(response.entries().next(), Some((gpd, record)));

    // 5. Reboot: the Sink Table and the translation come back from the
    // journal, and the next Toggle still drives endpoint 2.
    let mut journal = crate::gp_store::GpProxyJournal::<_, 4096>::new(
        crate::gp_store::tests::MockFlash::new(),
        0,
        4096,
    );
    assert!(router.save_gp_proxy_table_if_dirty(&mut journal).unwrap());
    let mut rebooted = combo();
    assert_eq!(rebooted.restore_gp_proxy_table(&mut journal).unwrap(), 1);
    assert_eq!(
        rebooted.gp_sink().translations(),
        router.gp_sink().translations()
    );
    let mut lights = Lights {
        one: OnOffCluster::new(),
        two: OnOffCluster::new(),
    };
    process(&mut rebooted, &mut lights, &gpdf(13, 0x22));
    assert!(lights.two.is_on());
    assert!(!lights.one.is_on());
}

#[test]
fn locally_issued_configuration_retargets_a_paired_gpd_without_a_tool() {
    let gpd = GpdId::SrcId(SRC);
    let mut router = combo();
    let mut lights = Lights {
        one: OnOffCluster::new(),
        two: OnOffCluster::new(),
    };
    // Not a configuration command: nothing applied.
    assert!(!router.gp_sink_configure_locally(server_cmd::NOTIFICATION, &[]));

    // A Translation Table Update before the GPD is paired is refused
    // (NOT_FOUND, no response on the local path).
    let record = TranslationRecord {
        index: 0xFF,
        gpd_command: 0x22,
        endpoint: 2,
        profile: 0x0104,
        cluster: 0x0006,
        zcl_command: 0x01,
        payload: TtPayload::Fixed(&[]),
        additional_info: None,
    };
    let mut update = [0u8; 64];
    let update_len =
        TranslationTableUpdate::encode(&gpd, tt_action::ADD, &[record], &mut update).unwrap();
    assert!(
        router
            .gp_sink_configure_locally(server_cmd::TRANSLATION_TABLE_UPDATE, &update[..update_len])
    );
    assert!(router.gp_sink().translations().is_empty());

    // Pair (Extend creates the entry), then retarget Toggle.
    let pc = PairingConfiguration {
        actions: pairing_action::EXTEND,
        gpd_id: gpd,
        communication_mode: CommunicationMode::DerivedGroupcast,
        mac_seq_cap: true,
        rx_on_cap: false,
        fixed_location: true,
        device_id: 0x02,
        groups: &[],
        assigned_alias: None,
        groupcast_radius: 0,
        security: Some(PairingSecurity {
            level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key: KEY,
        }),
        frame_counter: Some(10),
        number_of_paired_endpoints: 0xFE,
        paired_endpoints: &[],
        application_information: None,
    };
    let mut payload = [0u8; 64];
    let n = pc.encode(&mut payload).unwrap();
    router.mac_mut().clear_tx_history();
    assert!(router.gp_sink_configure_locally(server_cmd::PAIRING_CONFIGURATION, &payload[..n]));
    assert!(
        router
            .gp_sink_configure_locally(server_cmd::TRANSLATION_TABLE_UPDATE, &update[..update_len])
    );
    assert_eq!(router.gp_sink().entries().len(), 1);
    assert_eq!(router.gp_sink().translations().len(), 1);
    assert!(router.gp_take_dirty());
    // A broadcast without "send GP Pairing": nothing goes on the air.
    block_on(router.gp_poll());
    assert!(router.mac().tx_history().is_empty());

    process(&mut router, &mut lights, &gpdf(11, 0x22));
    assert!(lights.two.is_on());
    assert!(!lights.one.is_on());

    // Remove the translation: Toggle is back on the built-in endpoint 1.
    let n =
        TranslationTableUpdate::encode(&gpd, tt_action::REMOVE, &[record], &mut payload).unwrap();
    assert!(router.gp_sink_configure_locally(server_cmd::TRANSLATION_TABLE_UPDATE, &payload[..n]));
    assert!(router.gp_sink().translations().is_empty());
    process(&mut router, &mut lights, &gpdf(12, 0x22));
    assert!(lights.one.is_on());
}
