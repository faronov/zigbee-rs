//! Sink behaviour of GP Pairing Configuration and the Translation Table.

use super::*;
use crate::cluster::{Notification, ProxyInfo, client_cmd};
use crate::config::{TranslationRecord, TranslationTableResponse, TtPayload, tt_action};
use crate::frame::{FrameType, GpdfParams, parse, parse_mac};
use crate::proxy::{DestAddr, GpTx, TxFrame};
use crate::security;
use crate::sink::SinkConfig;
use crate::types::cmd;
use zigbee_crypto::{AesKey, SoftwareAesProvider};

const KEY: AesKey = [0x5A; 16];
const SRC: u32 = 0x7A6B_9879;
const GPD: GpdId = GpdId::SrcId(SRC);
const OWN: u16 = 0x7C87;
const CT: u16 = 0x0000;
const OWN_IEEE: crate::types::IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];
const LIGHT: u8 = 1;

type S = Sink<4, 4>;
type Command = (u8, u16, u8, heapless::Vec<u8, 8>);

fn sink(security_level: u8) -> S {
    let mut s = S::new(SinkConfig {
        endpoint: Some(LIGHT),
        security_level,
        ..SinkConfig::default()
    });
    s.set_own_address(OWN, OWN_IEEE, 0x1A62);
    s.set_operational_channel(15);
    s
}

fn command(command_id: u8, payload: &[u8], unicast: bool) -> ServerCommand<'_> {
    ServerCommand {
        src_nwk: CT,
        unicast,
        tsn: 0x33,
        command_id,
        payload,
    }
}

fn send(s: &mut S, command_id: u8, payload: &[u8]) -> (u8, Option<SinkOutcome>) {
    s.on_server_command(
        &mut SoftwareAesProvider,
        &command(command_id, payload, true),
        1_000,
    )
    .unwrap()
}

/// A GPDF from the test GPD, received directly.
fn receive(
    s: &mut S,
    level: SecurityLevel,
    counter: u32,
    command: u8,
    payload: &[u8],
) -> SinkOutcome {
    let mut raw = [0u8; 127];
    let params = GpdfParams {
        frame_type: FrameType::Data,
        auto_commissioning: false,
        src_id: Some(SRC),
        security_level: level,
        individual_key: level.is_protected(),
        rx_after_tx: false,
        to_gpd: false,
        frame_counter: counter,
    };
    let len = security::build_gpdf(
        &mut SoftwareAesProvider,
        level.is_protected().then_some(&KEY),
        &params,
        counter as u8,
        command,
        payload,
        &mut raw,
    )
    .unwrap();
    let mac = parse_mac(&raw[..len]).unwrap();
    let frame = parse(&raw[mac.payload_offset..len], mac.seq, None).unwrap();
    s.on_gpdf(&mut SoftwareAesProvider, &frame, 2_000 + counter * 10)
}

fn drain(s: &mut S) -> heapless::Vec<GpTx, 8> {
    let mut out = heapless::Vec::new();
    while let Some(tx) = s.poll(1_000) {
        out.push(tx.tx).unwrap();
    }
    out
}

fn gp_command(tx: &GpTx) -> (u8, &[u8]) {
    match &tx.frame {
        TxFrame::GpCommand {
            command_id,
            payload,
        } => (*command_id, &payload[..]),
        other => panic!("unexpected frame {other:?}"),
    }
}

/// GP Pairing Configuration from the commissioning tool.
struct Pc<'a> {
    actions: u8,
    mode: CommunicationMode,
    groups: &'a [u8],
    protected: bool,
    endpoints: (u8, &'a [u8]),
    app_info: Option<crate::config::ApplicationInformation<'a>>,
    gpd: GpdId,
}

impl Default for Pc<'_> {
    fn default() -> Self {
        Self {
            actions: pairing_action::REPLACE | pairing_action::SEND_GP_PAIRING,
            mode: CommunicationMode::DerivedGroupcast,
            groups: &[],
            protected: true,
            endpoints: (paired_endpoints::SINK_DERIVES, &[]),
            app_info: None,
            gpd: GPD,
        }
    }
}

impl Pc<'_> {
    fn encode(&self) -> heapless::Vec<u8, 96> {
        let pc = PairingConfiguration {
            actions: self.actions,
            gpd_id: self.gpd,
            communication_mode: self.mode,
            mac_seq_cap: true,
            rx_on_cap: false,
            fixed_location: true,
            device_id: 0x02,
            groups: self.groups,
            assigned_alias: None,
            groupcast_radius: 0,
            security: self.protected.then_some(crate::config::PairingSecurity {
                level: SecurityLevel::Encrypted,
                key_type: KeyType::IndividualKey,
                key: KEY,
            }),
            frame_counter: Some(10),
            number_of_paired_endpoints: self.endpoints.0,
            paired_endpoints: self.endpoints.1,
            application_information: self.app_info,
        };
        let mut b = [0u8; 96];
        let n = pc.encode(&mut b).unwrap();
        heapless::Vec::from_slice(&b[..n]).unwrap()
    }
}

fn configure(s: &mut S, pc: &Pc<'_>) -> (u8, Option<SinkOutcome>) {
    send(s, server_cmd::PAIRING_CONFIGURATION, &pc.encode())
}

fn translations(s: &mut S, action: u8, records: &[TranslationRecord<'_>]) -> u8 {
    let mut b = [0u8; 128];
    let n = TranslationTableUpdate::encode(&GPD, action, records, &mut b).unwrap();
    send(s, server_cmd::TRANSLATION_TABLE_UPDATE, &b[..n]).0
}

fn tt(
    cmd: u8,
    endpoint: u8,
    cluster: u16,
    zcl: u8,
    payload: TtPayload<'static>,
) -> TranslationRecord<'static> {
    TranslationRecord {
        index: 0xFF,
        gpd_command: cmd,
        endpoint,
        profile: 0x0104,
        cluster,
        zcl_command: zcl,
        payload,
        additional_info: None,
    }
}

fn commands(s: &mut S) -> heapless::Vec<Command, 4> {
    core::iter::from_fn(|| s.take_command())
        .map(|c| {
            let t = c.translation;
            (
                c.endpoint,
                t.cluster_id,
                t.command_id,
                heapless::Vec::from_slice(&t.payload).unwrap(),
            )
        })
        .collect()
}

fn no_payload(endpoint: u8, cluster: u16, command: u8) -> Command {
    (endpoint, cluster, command, heapless::Vec::new())
}

#[test]
fn pairing_configuration_and_translation_update_drive_endpoint_x_with_command_y() {
    let mut s = sink(0x01);
    assert_eq!(
        configure(&mut s, &Pc::default()),
        (zcl_status::SUCCESS, Some(SinkOutcome::Commissioned))
    );
    let e = &s.entries()[0];
    assert_eq!(
        (e.gpd_id, e.security_level, e.key),
        (GPD, SecurityLevel::Encrypted, Some(KEY))
    );
    assert_eq!((e.frame_counter, e.device_id), (10, 0x02));
    assert!(s.take_dirty());
    let alias = derive_alias(&GPD);
    assert_eq!(s.take_group_change(), Some(GroupChange::Join(alias)));
    assert_eq!(
        s.take_event(),
        Some(SinkEvent::Commissioned {
            gpd_id: GPD,
            device_id: 0x02
        })
    );
    // "Send GP Pairing": AddSink to the derived group with the key, and the
    // alias Device_annce.
    let tx = drain(&mut s);
    assert_eq!(tx.len(), 2);
    let (id, payload) = gp_command(&tx[0]);
    assert_eq!(id, client_cmd::PAIRING);
    let p = Pairing::decode(payload).unwrap();
    assert!(p.add_sink && !p.remove_gpd);
    assert_eq!(
        (p.sink, p.key, p.frame_counter),
        (PairingSink::Group(alias), Some(KEY), Some(10))
    );
    assert!(matches!(tx[1].frame, TxFrame::DeviceAnnce { alias: a } if a == alias));

    // Toggle → endpoint 2, On/Off Off; Step Up → endpoint 3, Level
    // Control Step (with On/Off) with the payload derived from the GPD's.
    let records = [
        tt(cmd::TOGGLE, 2, 0x0006, 0x00, TtPayload::Fixed(&[])),
        tt(0x32, 3, 0x0008, 0x06, TtPayload::Derived),
    ];
    assert_eq!(
        translations(&mut s, tt_action::ADD, &records),
        zcl_status::SUCCESS
    );
    assert!(s.take_dirty());
    assert_eq!(s.translations().len(), 2);

    assert_eq!(
        receive(&mut s, SecurityLevel::Encrypted, 11, cmd::TOGGLE, &[]),
        SinkOutcome::Translated
    );
    assert_eq!(&commands(&mut s)[..], &[no_payload(2, 0x0006, 0x00)]);
    assert_eq!(
        receive(&mut s, SecurityLevel::Encrypted, 12, 0x32, &[0x10]),
        SinkOutcome::Translated
    );
    assert_eq!(
        &commands(&mut s)[..],
        &[(
            3,
            0x0008,
            0x06,
            heapless::Vec::from_slice(&[0x00, 0x10, 0xFF, 0xFF]).unwrap()
        )]
    );
    // No explicit entry for On: the built-in translation to the light.
    assert_eq!(
        receive(&mut s, SecurityLevel::Encrypted, 13, cmd::ON, &[]),
        SinkOutcome::Translated
    );
    assert_eq!(&commands(&mut s)[..], &[no_payload(LIGHT, 0x0006, 0x01)]);
    // Freshness still applies.
    assert_eq!(
        receive(&mut s, SecurityLevel::Encrypted, 13, cmd::TOGGLE, &[]),
        SinkOutcome::Dropped(SinkDrop::Duplicate)
    );
}

#[test]
fn translation_table_request_is_answered_to_unicast_only() {
    let mut s = sink(0x00);
    configure(&mut s, &Pc::default());
    drain(&mut s);
    let records = [
        tt(cmd::TOGGLE, 2, 0x0006, 0x02, TtPayload::Fixed(&[])),
        tt(
            cmd::ON,
            2,
            0x0008,
            0x04,
            TtPayload::Fixed(&[0xFE, 0x0A, 0x00]),
        ),
    ];
    translations(&mut s, tt_action::ADD, &records);
    let req = [0x00];
    s.on_server_command(
        &mut SoftwareAesProvider,
        &command(server_cmd::TRANSLATION_TABLE_REQUEST, &req, false),
        0,
    )
    .unwrap();
    assert!(drain(&mut s).is_empty(), "no response to a broadcast");
    assert_eq!(
        send(&mut s, server_cmd::TRANSLATION_TABLE_REQUEST, &req).0,
        zcl_status::SUCCESS
    );
    let tx = s.poll(0).unwrap();
    assert_eq!(tx.tsn, Some(0x33));
    assert_eq!(
        tx.tx.dest,
        DestAddr::Unicast {
            nwk: CT,
            ieee: None
        }
    );
    let (id, payload) = gp_command(&tx.tx);
    assert_eq!(id, client_cmd::TRANSLATION_TABLE_RESPONSE);
    let r = TranslationTableResponse::decode(payload).unwrap();
    assert_eq!((r.status, r.total, r.start_index, r.len()), (0, 2, 0, 2));
    let mut entries = r.entries();
    assert_eq!(entries.next(), Some((GPD, records[0])));
    assert_eq!(entries.next(), Some((GPD, records[1])));
    // Past the end: NOT_FOUND.
    send(&mut s, server_cmd::TRANSLATION_TABLE_REQUEST, &[5]);
    let tx = s.poll(0).unwrap();
    assert_eq!(gp_command(&tx.tx).1, &[zcl_status::NOT_FOUND, 0, 2, 0, 0]);
}

#[test]
fn paired_endpoints_select_the_default_translation_targets() {
    // An unprotected GPD (gpsSecurityLevel 0) paired to endpoints 3 and 4.
    let mut s = sink(0x00);
    let pc = Pc {
        protected: false,
        endpoints: (2, &[3, 4]),
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, zcl_status::SUCCESS);
    assert_eq!(
        receive(&mut s, SecurityLevel::None, 11, cmd::TOGGLE, &[]),
        SinkOutcome::Translated
    );
    assert_eq!(
        &commands(&mut s)[..],
        &[no_payload(3, 0x0006, 0x02), no_payload(4, 0x0006, 0x02)]
    );
    // A command without a default translation is reported.
    assert_eq!(
        receive(&mut s, SecurityLevel::None, 12, 0x60, &[]),
        SinkOutcome::PassedThrough
    );
    // No paired endpoint: every command is only reported.
    let pc = Pc {
        protected: false,
        endpoints: (paired_endpoints::NONE, &[]),
        ..Pc::default()
    };
    configure(&mut s, &pc);
    while s.take_event().is_some() {}
    assert_eq!(
        receive(&mut s, SecurityLevel::None, 13, cmd::TOGGLE, &[]),
        SinkOutcome::PassedThrough
    );
    assert!(s.take_command().is_none());
    assert!(matches!(
        s.take_event(),
        Some(SinkEvent::GpdCommand {
            command_id: cmd::TOGGLE,
            ..
        })
    ));
}

#[test]
fn pass_to_application_and_copied_payloads() {
    let mut s = sink(0x01);
    configure(&mut s, &Pc::default());
    while s.take_event().is_some() {}
    let records = [
        tt(
            cmd::TOGGLE,
            crate::config::PASS_TO_APPLICATION,
            0x0006,
            0x02,
            TtPayload::Fixed(&[]),
        ),
        tt(cmd::TOGGLE, 5, 0x0006, 0x02, TtPayload::Fixed(&[])),
        tt(0x33, 5, 0x0008, 0x02, TtPayload::CopyGpd),
    ];
    translations(&mut s, tt_action::ADD, &records);
    assert_eq!(
        receive(&mut s, SecurityLevel::Encrypted, 11, cmd::TOGGLE, &[]),
        SinkOutcome::Translated
    );
    assert_eq!(commands(&mut s).len(), 1);
    assert!(matches!(s.take_event(), Some(SinkEvent::GpdCommand { .. })));
    // Copy: the GPD Step Down payload as is.
    receive(
        &mut s,
        SecurityLevel::Encrypted,
        12,
        0x33,
        &[0x20, 0x05, 0x00],
    );
    assert_eq!(
        &commands(&mut s)[..],
        &[(
            5,
            0x0008,
            0x02,
            heapless::Vec::from_slice(&[0x20, 0x05, 0x00]).unwrap()
        )]
    );
    // A copied payload longer than an entry holds is dropped and counted,
    // and the command is reported instead.
    assert_eq!(
        receive(&mut s, SecurityLevel::Encrypted, 13, 0x33, &[0; 9]),
        SinkOutcome::PassedThrough
    );
    assert_eq!(s.stats().translations_dropped, 1);
}

#[test]
fn remove_gpd_and_remove_pairing() {
    let mut s = sink(0x01);
    configure(&mut s, &Pc::default());
    translations(
        &mut s,
        tt_action::ADD,
        &[tt(cmd::TOGGLE, 2, 6, 0, TtPayload::Fixed(&[]))],
    );
    drain(&mut s);
    while s.take_group_change().is_some() {}
    // Remove GPD without GP Pairing: entry and translations go, nothing is
    // sent, the derived group is left.
    let pc = Pc {
        actions: pairing_action::REMOVE_GPD,
        ..Pc::default()
    };
    assert_eq!(
        configure(&mut s, &pc),
        (zcl_status::SUCCESS, Some(SinkOutcome::Decommissioned))
    );
    assert!(s.entries().is_empty() && s.translations().is_empty());
    assert!(drain(&mut s).is_empty());
    assert_eq!(
        s.take_group_change(),
        Some(GroupChange::Leave(derive_alias(&GPD)))
    );

    // Pre-commissioned groupcast to 0x0B84 and 0x1234.
    let groups = [0x84, 0x0B, 0xFF, 0xFF, 0x34, 0x12, 0xFF, 0xFF];
    let pc = Pc {
        mode: CommunicationMode::CommissionedGroupcast,
        groups: &groups,
        actions: pairing_action::EXTEND,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, zcl_status::SUCCESS);
    assert_eq!(s.entries()[0].groups.len(), 2);
    // A Remove Pairing for another group does not match.
    let other = [0x99, 0x99, 0xFF, 0xFF];
    let mut rp = Pc {
        actions: pairing_action::REMOVE_PAIRING | pairing_action::SEND_GP_PAIRING,
        mode: CommunicationMode::CommissionedGroupcast,
        groups: &other,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &rp).0, zcl_status::NOT_FOUND);
    rp.groups = &groups[..4];
    drain(&mut s);
    assert_eq!(
        configure(&mut s, &rp),
        (zcl_status::SUCCESS, Some(SinkOutcome::Decommissioned))
    );
    // One "remove this sink" GP Pairing per commissioned group.
    let tx = drain(&mut s);
    assert_eq!(tx.len(), 2);
    for (tx, group) in tx.iter().zip([0x0B84, 0x1234]) {
        let p = Pairing::decode(gp_command(tx).1).unwrap();
        assert!(!p.add_sink && !p.remove_gpd);
        assert_eq!(p.sink, PairingSink::Group(group));
    }
    assert!(s.entries().is_empty());
}

#[test]
fn extend_merges_groups_and_keeps_translations_replace_drops_them() {
    let mut s = sink(0x01);
    let g1 = [0x84, 0x0B, 0xFF, 0xFF];
    let g2 = [0x34, 0x12, 0x00, 0x60];
    let pc = Pc {
        mode: CommunicationMode::CommissionedGroupcast,
        groups: &g1,
        ..Pc::default()
    };
    configure(&mut s, &pc);
    translations(
        &mut s,
        tt_action::ADD,
        &[tt(cmd::TOGGLE, 2, 6, 0, TtPayload::Fixed(&[]))],
    );
    let pc = Pc {
        mode: CommunicationMode::CommissionedGroupcast,
        groups: &g2,
        actions: pairing_action::EXTEND,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, zcl_status::SUCCESS);
    let groups: heapless::Vec<(u16, Option<u16>), 2> = s.entries()[0]
        .groups
        .iter()
        .map(|g| (g.group, g.alias))
        .collect();
    assert_eq!(&groups[..], &[(0x0B84, None), (0x1234, Some(0x6000))]);
    assert_eq!(s.translations().len(), 1);
    // A third group does not fit.
    let g3 = [0x56, 0x34, 0xFF, 0xFF];
    let pc = Pc {
        mode: CommunicationMode::CommissionedGroupcast,
        groups: &g3,
        actions: pairing_action::EXTEND,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, 0x89);
    // Replace: a fresh entry, the translations are dropped.
    assert_eq!(configure(&mut s, &Pc::default()).0, zcl_status::SUCCESS);
    assert_eq!(
        s.entries()[0].communication_mode,
        CommunicationMode::DerivedGroupcast
    );
    assert!(s.entries()[0].groups.is_empty());
    assert!(s.translations().is_empty());
}

#[test]
fn refusals_change_nothing() {
    let mut s = sink(0x02);
    // Below gpsSecurityLevel (level 2 minimum): an unprotected GPD.
    let pc = Pc {
        protected: false,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc), (0x01, None));
    // Reserved GPD ID.
    let pc = Pc {
        gpd: GpdId::SrcId(0),
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, zcl_status::INVALID_FIELD);
    // Application Description and "application description follows".
    let pc = Pc {
        actions: pairing_action::APPLICATION_DESCRIPTION,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, 0x01);
    let pc = Pc {
        app_info: Some(crate::config::ApplicationInformation {
            bits: crate::config::app_info::APPLICATION_DESCRIPTION,
            application_description: Some((1, 0, &[])),
            ..Default::default()
        }),
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, 0x01);
    // Reserved and too many paired endpoints, groupcast without a group.
    let pc = Pc {
        endpoints: (paired_endpoints::RESERVED, &[]),
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, zcl_status::INVALID_FIELD);
    let pc = Pc {
        endpoints: (5, &[1, 2, 3, 4, 5]),
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, 0x89);
    let pc = Pc {
        mode: CommunicationMode::CommissionedGroupcast,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, zcl_status::INVALID_FIELD);
    // A reserved action; a resend for an unknown GPD.
    let pc = Pc {
        actions: 0x06,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, zcl_status::INVALID_FIELD);
    let pc = Pc {
        actions: pairing_action::NO_ACTION | pairing_action::SEND_GP_PAIRING,
        ..Pc::default()
    };
    assert_eq!(configure(&mut s, &pc).0, zcl_status::NOT_FOUND);
    // Its own looped-back command is ignored.
    let raw = Pc::default().encode();
    let own = ServerCommand {
        src_nwk: OWN,
        ..command(server_cmd::PAIRING_CONFIGURATION, &raw, false)
    };
    assert_eq!(
        s.on_server_command(&mut SoftwareAesProvider, &own, 0),
        Ok((zcl_status::SUCCESS, None))
    );
    assert!(s.entries().is_empty());
    assert!(!s.take_dirty());
    assert!(!s.has_tx());
    // A malformed command.
    assert_eq!(
        s.on_server_command(
            &mut SoftwareAesProvider,
            &command(server_cmd::PAIRING_CONFIGURATION, &raw[..5], true),
            0
        ),
        Err(Error::Truncated)
    );
}

#[test]
fn groupcast_notifications_are_translated_by_member_sinks() {
    let mut s = sink(0x01);
    let groups = [0x84, 0x0B, 0xFF, 0xFF];
    let pc = Pc {
        mode: CommunicationMode::CommissionedGroupcast,
        groups: &groups,
        actions: pairing_action::REPLACE,
        ..Pc::default()
    };
    configure(&mut s, &pc);
    assert_eq!(s.take_group_change(), Some(GroupChange::Join(0x0B84)));
    assert!(s.required_groups().any(|g| g == 0x0B84));
    translations(
        &mut s,
        tt_action::ADD,
        &[tt(cmd::TOGGLE, 2, 6, 0x00, TtPayload::Fixed(&[]))],
    );
    // A proxy's groupcast GP Notification (no response, not unicast).
    let n = Notification {
        gpd_id: GPD,
        also_unicast: false,
        also_derived_group: false,
        also_commissioned_group: true,
        security_level: SecurityLevel::Encrypted,
        key_type: KeyType::IndividualKey,
        rx_after_tx: false,
        tx_queue_full: false,
        bidirectional_cap: false,
        frame_counter: 20,
        command_id: cmd::TOGGLE,
        payload: Some(&[]),
        proxy_info: ProxyInfo::None,
    };
    let mut b = [0u8; 64];
    let len = n.encode(&mut b).unwrap();
    let notification = ServerCommand {
        src_nwk: 0x1111,
        ..command(server_cmd::NOTIFICATION, &b[..len], false)
    };
    assert_eq!(
        s.on_server_command(&mut SoftwareAesProvider, &notification, 5_000),
        Ok((zcl_status::SUCCESS, Some(SinkOutcome::Translated)))
    );
    assert_eq!(&commands(&mut s)[..], &[no_payload(2, 0x0006, 0x00)]);
    assert!(!s.has_tx(), "no Notification Response to a groupcast");
}

#[test]
fn functionality_advertises_ct_commissioning_and_translation_table() {
    let s = sink(0x01);
    let mut out = [0u8; 4];
    let (_, n) = s
        .read_attribute(crate::cluster::attr::GPS_FUNCTIONALITY, &mut out)
        .unwrap()
        .unwrap();
    let f = u32::from_le_bytes([out[0], out[1], out[2], 0]);
    assert_eq!(n, 3);
    assert_ne!(f & (1 << 11), 0);
    assert_ne!(f & (1 << 18), 0);
}

#[test]
fn translations_need_a_paired_gpd() {
    let mut s = sink(0x01);
    let records = [tt(cmd::TOGGLE, 2, 6, 0, TtPayload::Fixed(&[]))];
    assert_eq!(
        translations(&mut s, tt_action::ADD, &records),
        zcl_status::NOT_FOUND
    );
    assert!(s.translations().is_empty() && !s.take_dirty());
}

#[test]
fn clear_drops_translations() {
    let mut s = sink(0x01);
    configure(&mut s, &Pc::default());
    translations(
        &mut s,
        tt_action::ADD,
        &[tt(cmd::TOGGLE, 2, 6, 0, TtPayload::Fixed(&[]))],
    );
    s.clear();
    assert!(s.translations().is_empty());
}
