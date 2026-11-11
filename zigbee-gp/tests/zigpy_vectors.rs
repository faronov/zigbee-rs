//! Byte-exact compatibility with zigpy (the Home Assistant ZHA GP sink).
//!
//! Vectors are produced by `tools/gp_vectors.py` from zigpy's own
//! serializers and an independent AES-CCM implementation.

#[path = "data/zigpy_vectors.rs"]
mod v;

use zigbee_crypto::SoftwareAesProvider;
use zigbee_gp::cluster::{
    CommissioningNotification, GppGpdLink, Notification, Pairing, PairingSink,
    ProxyCommissioningMode, ProxyInfo, Response, exit_mode,
};
use zigbee_gp::commissioning::{
    ApplicationInfo, AttributeReport, CommissioningPayload, CommissioningReply,
    CommissioningSecurity, MultiClusterReport, channel_configuration, channel_request,
    encode_channel_configuration,
};
use zigbee_gp::frame::{parse, parse_mac};
use zigbee_gp::security::{self, DEFAULT_TC_LINK_KEY, Direction, WrappedKey};
use zigbee_gp::types::{CommunicationMode, GpdId, KeyType, SecurityLevel, cmd};

const GPD_KEY: [u8; 16] = [
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
];
const LINK: GppGpdLink = GppGpdLink(0xE8); // rssi 40, link quality 3

fn sensor_commissioning() -> CommissioningPayload {
    let w = v::COMMISSIONING_WRAPPED_KEY;
    let mut key = [0u8; 16];
    key.copy_from_slice(&w[..16]);
    CommissioningPayload {
        device_id: 0xFE,
        mac_seq_num_capability: true,
        rx_on_capability: false,
        pan_id_request: false,
        security_key_request: false,
        fixed_location: true,
        security: Some(CommissioningSecurity {
            level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key: Some((key, Some(u32::from_le_bytes([w[16], w[17], w[18], w[19]])))),
            outgoing_counter: Some(16),
        }),
        app_info: Some(ApplicationInfo {
            manufacturer_id: Some(0x6D6B),
            model_id: Some(0xF300),
            ..ApplicationInfo::default()
        }),
    }
}

fn encode<const N: usize>(f: impl FnOnce(&mut [u8]) -> zigbee_gp::Result<usize>) -> Vec<u8> {
    let mut b = [0u8; N];
    let n = f(&mut b).unwrap();
    b[..n].to_vec()
}

#[test]
fn attribute_reports_match_zigpy() {
    let mut b = [0u8; 32];
    let n = AttributeReport::new(&mut b, 0x0402)
        .unwrap()
        .int16(0x0000, 2150)
        .unwrap()
        .finish();
    assert_eq!(&b[..n], v::ATTR_REPORT_TEMP);
    let n = AttributeReport::new_manufacturer(&mut b, 0x6D6B, 0x0405)
        .unwrap()
        .uint16(0x0000, 4875)
        .unwrap()
        .finish();
    assert_eq!(&b[..n], v::MFR_ATTR_REPORT_HUMIDITY);
}

#[test]
fn multi_cluster_report_matches_zigpy() {
    let mut b = [0u8; 32];
    let n = MultiClusterReport::new(&mut b)
        .int16(0x0402, 0x0000, -125)
        .unwrap()
        .uint16(0x0405, 0x0000, 4875)
        .unwrap()
        .uint8(0x0001, 0x0020, 30)
        .unwrap()
        .finish();
    assert_eq!(&b[..n], v::MULTI_CLUSTER_REPORT_TEMP_HUM_BATT);
}

#[test]
fn switch_commissioning_payload_matches_zigpy() {
    let mut p = sensor_commissioning();
    p.device_id = 0x02;
    let mut cmds = heapless::Vec::new();
    cmds.extend_from_slice(&[cmd::OFF, cmd::ON, cmd::TOGGLE])
        .unwrap();
    p.app_info = Some(ApplicationInfo {
        manufacturer_id: Some(0x6D6B),
        model_id: Some(0xF400),
        gpd_commands: Some(cmds),
        ..ApplicationInfo::default()
    });
    assert_eq!(encode::<64>(|b| p.encode(b)), v::COMMISSIONING_SWITCH_L3);
    assert_eq!(
        CommissioningPayload::decode(v::COMMISSIONING_SWITCH_L3).unwrap(),
        p
    );
}

#[test]
fn commissioning_payload_matches_zigpy() {
    let p = sensor_commissioning();
    assert_eq!(encode::<64>(|b| p.encode(b)), v::COMMISSIONING_SENSOR_L3);
    assert_eq!(
        CommissioningPayload::decode(v::COMMISSIONING_SENSOR_L3).unwrap(),
        p
    );
}

#[test]
fn tclk_wrapped_key_matches_zigpy_crypto() {
    let mut aes = SoftwareAesProvider;
    let gpd = GpdId::SrcId(v::SRC_ID);
    let w = security::wrap_key(
        &mut aes,
        &DEFAULT_TC_LINK_KEY,
        &gpd,
        Direction::FromGpd,
        0,
        &GPD_KEY,
    )
    .unwrap();
    assert_eq!(&w.key[..], &v::COMMISSIONING_WRAPPED_KEY[..16]);
    assert_eq!(&w.mic[..], &v::COMMISSIONING_WRAPPED_KEY[16..]);
    let k = security::unwrap_key(
        &mut aes,
        &DEFAULT_TC_LINK_KEY,
        &gpd,
        Direction::FromGpd,
        0,
        &WrappedKey {
            key: w.key,
            mic: w.mic,
        },
    )
    .unwrap();
    assert_eq!(k, GPD_KEY);
}

#[test]
fn commissioning_reply_and_channel_payloads_match_zigpy() {
    let r = CommissioningReply::decode(v::COMMISSIONING_REPLY_ZIGPY).unwrap();
    assert_eq!(r.pan_id, None);
    assert_eq!(r.key, None);
    assert_eq!(r.security_level, SecurityLevel::Encrypted);
    assert_eq!(r.key_type, KeyType::IndividualKey);
    assert_eq!(encode::<8>(|b| r.encode(b)), v::COMMISSIONING_REPLY_ZIGPY);

    assert_eq!(
        channel_request(15, 20).unwrap(),
        v::CHANNEL_REQUEST_15_20[0]
    );
    assert_eq!(
        encode_channel_configuration(25, true).unwrap(),
        v::CHANNEL_CONFIGURATION_25[0]
    );
    assert_eq!(
        channel_configuration(v::CHANNEL_CONFIGURATION_25[0]),
        (25, true)
    );
}

#[test]
fn notifications_match_zigpy() {
    let n = Notification {
        gpd_id: GpdId::SrcId(v::SRC_ID),
        also_unicast: false,
        also_derived_group: false,
        also_commissioned_group: true,
        security_level: SecurityLevel::Encrypted,
        key_type: KeyType::IndividualKey,
        rx_after_tx: false,
        tx_queue_full: false,
        bidirectional_cap: false,
        frame_counter: 0x11,
        command_id: cmd::ATTRIBUTE_REPORTING,
        payload: Some(v::ATTR_REPORT_TEMP),
        proxy_info: ProxyInfo::None,
    };
    assert_eq!(encode::<64>(|b| n.encode(b)), v::NOTIFICATION_COMM_GROUP_L3);
    assert_eq!(
        Notification::decode(v::NOTIFICATION_COMM_GROUP_L3).unwrap(),
        n
    );

    let n = Notification {
        rx_after_tx: true,
        bidirectional_cap: true,
        frame_counter: 0x12,
        command_id: cmd::TOGGLE,
        payload: Some(&[]),
        proxy_info: ProxyInfo::Link {
            gpp_short: 0x1234,
            link: LINK,
        },
        ..n
    };
    assert_eq!(
        encode::<64>(|b| n.encode(b)),
        v::NOTIFICATION_RX_AFTER_TX_PROXY_INFO
    );
    assert_eq!(
        Notification::decode(v::NOTIFICATION_RX_AFTER_TX_PROXY_INFO).unwrap(),
        n
    );
}

#[test]
fn commissioning_notifications_match_zigpy() {
    let c = CommissioningNotification {
        gpd_id: GpdId::SrcId(v::SRC_ID),
        rx_after_tx: false,
        security_level: SecurityLevel::None,
        key_type: KeyType::NoKey,
        security_failed: None,
        bidirectional_cap: false,
        frame_counter: 5,
        command_id: cmd::COMMISSIONING,
        payload: Some(v::COMMISSIONING_SENSOR_L3),
        proxy_info: ProxyInfo::Link {
            gpp_short: 0x1234,
            link: LINK,
        },
    };
    assert_eq!(
        encode::<96>(|b| c.encode(b)),
        v::COMMISSIONING_NOTIFICATION_PROXY_INFO
    );
    assert_eq!(
        CommissioningNotification::decode(v::COMMISSIONING_NOTIFICATION_PROXY_INFO).unwrap(),
        c
    );

    let c = CommissioningNotification {
        security_level: SecurityLevel::Encrypted,
        key_type: KeyType::IndividualKey,
        security_failed: Some(0xA1B2_C3D4),
        frame_counter: 0x20,
        command_id: 0x5A,
        payload: Some(&[0x01, 0x02]),
        proxy_info: ProxyInfo::Link {
            gpp_short: 0x1234,
            link: GppGpdLink(0x4A),
        },
        ..c
    };
    assert_eq!(
        encode::<64>(|b| c.encode(b)),
        v::COMMISSIONING_NOTIFICATION_SECURITY_FAILED
    );
    assert_eq!(
        CommissioningNotification::decode(v::COMMISSIONING_NOTIFICATION_SECURITY_FAILED).unwrap(),
        c
    );
}

#[test]
fn zigpy_proxy_commissioning_mode_decodes() {
    let enter = ProxyCommissioningMode::decode(v::PCM_ENTER_180).unwrap();
    assert!(enter.enter);
    assert_eq!(enter.exit_mode, exit_mode::ON_EXPLICIT_EXIT);
    assert_eq!(enter.window_s, Some(180));
    assert_eq!(enter.channel, None);
    assert!(!enter.unicast);
    assert_eq!(encode::<8>(|b| enter.encode(b)), v::PCM_ENTER_180);

    let exit = ProxyCommissioningMode::decode(v::PCM_EXIT).unwrap();
    assert!(!exit.enter);
    assert_eq!(exit.window_s, None);
    assert_eq!(encode::<8>(|b| exit.encode(b)), v::PCM_EXIT);
}

#[test]
fn zigpy_pairings_decode() {
    let add = Pairing::decode(v::PAIRING_ADD_L3).unwrap();
    assert_eq!(add.gpd_id, GpdId::SrcId(v::SRC_ID));
    assert!(add.add_sink && !add.remove_gpd);
    assert_eq!(
        add.communication_mode,
        CommunicationMode::CommissionedGroupcast
    );
    assert!(add.gpd_fixed && add.mac_seq_cap);
    assert_eq!(add.security_level, SecurityLevel::Encrypted);
    assert_eq!(add.key_type, KeyType::IndividualKey);
    assert_eq!(add.sink, PairingSink::Group(0x0B84));
    assert_eq!(add.device_id, Some(0xFE));
    assert_eq!(add.frame_counter, Some(16));
    assert_eq!(add.key, Some(GPD_KEY));
    assert_eq!(add.assigned_alias, None);
    assert_eq!(add.groupcast_radius, None);
    assert_eq!(encode::<64>(|b| add.encode(b)), v::PAIRING_ADD_L3);

    let rm = Pairing::decode(v::PAIRING_REMOVE).unwrap();
    assert!(!rm.add_sink && rm.remove_gpd);
    assert_eq!(rm.sink, PairingSink::None);
    assert_eq!(rm.device_id, None);
    assert_eq!(rm.key, None);
    assert_eq!(encode::<64>(|b| rm.encode(b)), v::PAIRING_REMOVE);
}

#[test]
fn zigpy_responses_decode() {
    let r = Response::decode(v::RESPONSE_CHANNEL_CONFIGURATION).unwrap();
    assert_eq!(r.gpd_id, GpdId::SrcId(0));
    assert_eq!(r.temp_master_short, 0x1234);
    assert_eq!(r.tx_channel(), 15);
    assert_eq!(r.command_id, cmd::CHANNEL_CONFIGURATION);
    assert_eq!(r.payload, v::CHANNEL_CONFIGURATION_25);
    assert_eq!(
        encode::<32>(|b| r.encode(b)),
        v::RESPONSE_CHANNEL_CONFIGURATION
    );

    let r = Response::decode(v::RESPONSE_COMMISSIONING_REPLY).unwrap();
    assert_eq!(r.gpd_id, GpdId::SrcId(v::SRC_ID));
    assert_eq!(r.tx_channel(), 25);
    assert_eq!(r.command_id, cmd::COMMISSIONING_REPLY);
    assert_eq!(r.payload, v::COMMISSIONING_REPLY_ZIGPY);
    assert_eq!(
        encode::<32>(|b| r.encode(b)),
        v::RESPONSE_COMMISSIONING_REPLY
    );
}

#[test]
fn independent_l3_gpdf_decrypts() {
    let mac = parse_mac(v::GPDF_L3_TEMP_REPORT).unwrap();
    let f = parse(
        &v::GPDF_L3_TEMP_REPORT[mac.payload_offset..],
        mac.seq,
        mac.src_ieee,
    )
    .unwrap();
    assert_eq!(f.header.gpd_id, Some(GpdId::SrcId(v::SRC_ID)));
    assert_eq!(f.header.frame_counter, 0x11);
    assert!(f.header.individual_key);
    let mut aes = SoftwareAesProvider;
    let clear = security::open(&mut aes, &GPD_KEY, &f, Direction::FromGpd).unwrap();
    assert_eq!(clear[0], cmd::ATTRIBUTE_REPORTING);
    assert_eq!(&clear[1..], v::ATTR_REPORT_TEMP);
}

#[cfg(feature = "gpd")]
#[test]
fn gpd_builder_matches_independent_gpdf() {
    use zigbee_gp::gpd::{CounterReservation, Gpd, GpdConfig, GpdSecurity, TxOptions};
    let mut aes = SoftwareAesProvider;
    let mut g = Gpd::new(
        GpdConfig {
            src_id: v::SRC_ID,
            security: Some(GpdSecurity {
                level: SecurityLevel::Encrypted,
                key_type: KeyType::IndividualKey,
                key: GPD_KEY,
            }),
        },
        0,
    )
    .unwrap();
    let mut c = CounterReservation::resume(0x11, 16);
    c.reserved(c.pending_reservation().unwrap().unwrap());
    let mut out = [0u8; 127];
    let n = g
        .data_frame(
            &mut aes,
            &mut c,
            TxOptions::default(),
            cmd::ATTRIBUTE_REPORTING,
            v::ATTR_REPORT_TEMP,
            &mut out,
        )
        .unwrap();
    assert_eq!(&out[..n], v::GPDF_L3_TEMP_REPORT);
    let n = g
        .data_frame(
            &mut aes,
            &mut c,
            TxOptions::default(),
            cmd::MULTI_CLUSTER_REPORTING,
            v::MULTI_CLUSTER_REPORT_TEMP_HUM_BATT,
            &mut out,
        )
        .unwrap();
    assert_eq!(&out[..n], v::GPDF_L3_MULTI_REPORT);
    let n = g
        .data_frame(
            &mut aes,
            &mut c,
            TxOptions::default(),
            cmd::TOGGLE,
            &[],
            &mut out,
        )
        .unwrap();
    assert_eq!(&out[..n], v::GPDF_L3_TOGGLE);
}

#[cfg(feature = "gpd")]
#[test]
fn gpd_commissioning_frame_payload_matches_zigpy() {
    use zigbee_gp::gpd::{CounterReservation, Gpd, GpdConfig, GpdSecurity};
    let mut aes = SoftwareAesProvider;
    let mut g = Gpd::new(
        GpdConfig {
            src_id: v::SRC_ID,
            security: Some(GpdSecurity {
                level: SecurityLevel::Encrypted,
                key_type: KeyType::IndividualKey,
                key: GPD_KEY,
            }),
        },
        4,
    )
    .unwrap();
    let mut c = CounterReservation::resume(16, 16);
    c.reserved(c.pending_reservation().unwrap().unwrap());
    let mut p = sensor_commissioning();
    p.security = None; // filled in by the builder
    let mut out = [0u8; 127];
    let n = g
        .commissioning_frame(&mut aes, &mut c, &DEFAULT_TC_LINK_KEY, p, false, &mut out)
        .unwrap();
    // zigpy stores 16 as the last counter received; data starts at 17.
    assert_eq!(c.peek(), 17);
    let mac = parse_mac(&out[..n]).unwrap();
    assert_eq!(mac.seq, 5);
    let f = parse(&out[mac.payload_offset..n], mac.seq, None).unwrap();
    assert_eq!(f.header.security_level, SecurityLevel::None);
    assert_eq!(f.payload[0], cmd::COMMISSIONING);
    assert_eq!(&f.payload[1..], v::COMMISSIONING_SENSOR_L3);
}

#[cfg(feature = "proxy")]
mod proxy {
    use super::*;
    use zigbee_gp::cluster::{client_cmd, server_cmd};
    use zigbee_gp::proxy::{
        BROADCAST_RX_ON_WHEN_IDLE, D_MIN_MS, DestAddr, GpdfOutcome, Proxy, ProxyConfig, TxFrame,
    };

    fn gp_command(tx: &zigbee_gp::proxy::GpTx) -> (u8, &[u8]) {
        match &tx.frame {
            TxFrame::GpCommand {
                command_id,
                payload,
            } => (*command_id, &payload[..]),
            other => panic!("unexpected frame {other:?}"),
        }
    }

    #[test]
    fn zha_paired_sensor_report_is_tunnelled_byte_exact() {
        let mut p: Proxy<4, 4> = Proxy::new(ProxyConfig::default());
        p.set_own_nwk(0x1234);
        let mut reply = None;
        p.on_client_command(
            0x0000,
            client_cmd::PAIRING,
            v::PAIRING_ADD_L3,
            0,
            &mut reply,
        )
        .unwrap();
        assert!(reply.is_none());

        let mac = parse_mac(v::GPDF_L3_TEMP_REPORT).unwrap();
        let f = parse(&v::GPDF_L3_TEMP_REPORT[mac.payload_offset..], mac.seq, None).unwrap();
        let mut aes = SoftwareAesProvider;
        assert_eq!(
            p.on_gpdf(&mut aes, &f, LINK, 1000),
            GpdfOutcome::Notification
        );
        let tx = p.poll(1000 + D_MIN_MS).unwrap();
        assert_eq!(tx.dest, DestAddr::Group(0x0B84));
        // Commissioned-groupcast alias: derived alias, MAC seq − 13.
        let (alias, seq) = tx.alias.unwrap();
        assert_eq!(
            alias,
            zigbee_gp::types::derive_alias(&GpdId::SrcId(v::SRC_ID))
        );
        assert_eq!(seq, 0x11u8.wrapping_sub(13));
        let (id, payload) = gp_command(&tx);
        assert_eq!(id, server_cmd::NOTIFICATION);
        assert_eq!(payload, v::NOTIFICATION_COMM_GROUP_L3);

        // Replay of the same frame is not forwarded again.
        assert!(matches!(
            p.on_gpdf(&mut aes, &f, LINK, 1100),
            GpdfOutcome::Dropped(_)
        ));
        assert!(p.poll(5000).is_none());
    }

    #[test]
    fn zha_commissioning_window_forwards_commissioning_byte_exact() {
        let mut p: Proxy<4, 4> = Proxy::new(ProxyConfig::default());
        p.set_own_nwk(0x1234);
        let mut reply = None;
        p.on_client_command(
            0x0000,
            client_cmd::PROXY_COMMISSIONING_MODE,
            v::PCM_ENTER_180,
            0,
            &mut reply,
        )
        .unwrap();
        assert!(p.in_commissioning_mode());

        // Unprotected GPD Commissioning: NWK FC 0x0C (no extended FC).
        let mut msdu = vec![0x0C];
        msdu.extend_from_slice(&v::SRC_ID.to_le_bytes());
        msdu.push(cmd::COMMISSIONING);
        msdu.extend_from_slice(v::COMMISSIONING_SENSOR_L3);
        let f = parse(&msdu, 5, None).unwrap();
        let mut aes = SoftwareAesProvider;
        assert_eq!(
            p.on_gpdf(&mut aes, &f, LINK, 1000),
            GpdfOutcome::CommissioningNotification
        );
        let tx = (1000..1200).find_map(|t| p.poll(t)).unwrap();
        assert_eq!(tx.dest, DestAddr::Broadcast(BROADCAST_RX_ON_WHEN_IDLE));
        let (id, payload) = gp_command(&tx);
        assert_eq!(id, server_cmd::COMMISSIONING_NOTIFICATION);
        assert_eq!(payload, v::COMMISSIONING_NOTIFICATION_PROXY_INFO);

        p.on_client_command(
            0x0000,
            client_cmd::PROXY_COMMISSIONING_MODE,
            v::PCM_EXIT,
            2000,
            &mut reply,
        )
        .unwrap();
        assert!(!p.in_commissioning_mode());
    }

    #[test]
    fn zha_remove_pairing_drops_entry() {
        let mut p: Proxy<4, 4> = Proxy::new(ProxyConfig::default());
        let mut reply = None;
        p.on_client_command(0, client_cmd::PAIRING, v::PAIRING_ADD_L3, 0, &mut reply)
            .unwrap();
        assert_eq!(p.entries().len(), 1);
        p.on_client_command(0, client_cmd::PAIRING, v::PAIRING_REMOVE, 10, &mut reply)
            .unwrap();
        assert_eq!(p.entries().len(), 0);
    }
}
