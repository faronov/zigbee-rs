use super::super::{
    MAINTENANCE_FUNCTIONALITY, ServerCommand, Sink, SinkConfig, SinkDrop, SinkOutcome, zcl_status,
};
use super::REPEAT_SUPPRESSION_MS;
use crate::cluster::{
    CommissioningNotification, GppGpdLink, Notification, ProxyInfo, Response, attr, client_cmd,
    functionality, server_cmd,
};
use crate::frame::{FrameType, GpdfParams, parse, parse_mac};
use crate::proxy::TxFrame;
use crate::security;
use crate::types::{GpdId, KeyType, SecurityLevel, cmd};
use zigbee_crypto::SoftwareAesProvider;

const OWN: u16 = 0x4A76;
const PROXY: u16 = 0x1111;

type S = Sink<4, 4>;
type Answer = (u16, u8, u8, heapless::Vec<u8, 8>);

fn sink() -> S {
    let mut s = S::new(SinkConfig {
        endpoint: Some(1),
        ..SinkConfig::default()
    });
    s.set_own_address(OWN, [1, 2, 3, 4, 5, 6, 7, 8], 0x1A62);
    s.set_operational_channel(15);
    s
}

fn answer(temp_master: u16, channel: u8, config: u8) -> Answer {
    (
        temp_master,
        channel,
        cmd::CHANNEL_CONFIGURATION,
        heapless::Vec::from_slice(&[config]).unwrap(),
    )
}

/// A Maintenance Channel Request GPDF (no SrcID) asking for an answer on
/// `next` (then `second`).
fn channel_request_gpdf(seq: u8, next: u8, second: u8) -> ([u8; 127], usize) {
    let mut raw = [0u8; 127];
    let params = GpdfParams {
        frame_type: FrameType::Maintenance,
        auto_commissioning: false,
        src_id: None,
        security_level: SecurityLevel::None,
        individual_key: false,
        rx_after_tx: false,
        to_gpd: false,
        frame_counter: 0,
    };
    let byte = crate::commissioning::channel_request(next, second).unwrap();
    let len = security::build_gpdf(
        &mut SoftwareAesProvider,
        None,
        &params,
        seq,
        cmd::CHANNEL_REQUEST,
        &[byte],
        &mut raw,
    )
    .unwrap();
    (raw, len)
}

fn direct(s: &mut S, seq: u8, next: u8, now: u32) -> SinkOutcome {
    let (raw, len) = channel_request_gpdf(seq, next, 26);
    let mac = parse_mac(&raw[..len]).unwrap();
    let frame = parse(&raw[mac.payload_offset..len], mac.seq, None).unwrap();
    s.on_gpdf(&mut SoftwareAesProvider, &frame, now)
}

/// The GP Responses the sink sent: (TempMaster, GPD receive channel,
/// command, payload).
fn responses(s: &mut S, now: u32) -> heapless::Vec<Answer, 4> {
    let mut out = heapless::Vec::new();
    while let Some(tx) = s.poll(now) {
        let TxFrame::GpCommand {
            command_id,
            payload,
        } = &tx.tx.frame
        else {
            panic!("unexpected frame");
        };
        assert_eq!(*command_id, client_cmd::RESPONSE);
        let r = Response::decode(payload).unwrap();
        assert_eq!(r.gpd_id, GpdId::SrcId(0));
        out.push((
            r.temp_master_short,
            r.tx_channel(),
            r.command_id,
            heapless::Vec::from_slice(r.payload).unwrap(),
        ))
        .unwrap();
    }
    out
}

#[test]
fn a_direct_channel_request_gets_the_operational_channel_through_the_own_proxy() {
    let mut s = sink();
    s.set_direct_bidirectional(true);
    // Outside commissioning mode the request is ignored.
    assert_eq!(
        direct(&mut s, 1, 20, 0),
        SinkOutcome::Dropped(SinkDrop::NotCommissioning)
    );
    s.enter_commissioning(0, false).unwrap();
    while s.poll(0).is_some() {}
    // The GPD listens next on channel 20: the own proxy delivers there
    // the operational channel 15 (0x04) with the Basic bit.
    assert_eq!(
        direct(&mut s, 2, 20, 10),
        SinkOutcome::ChannelConfigurationSent
    );
    assert_eq!(responses(&mut s, 10)[..], [answer(OWN, 20, 0x14)]);
    // The GPD repeats its request: one GP Response serves them.
    assert_eq!(
        direct(&mut s, 3, 20, 200),
        SinkOutcome::ChannelConfigurationSent
    );
    assert!(responses(&mut s, 200).is_empty());
    // The same sequence number again is a duplicate.
    assert_eq!(
        direct(&mut s, 3, 20, 210),
        SinkOutcome::Dropped(SinkDrop::Duplicate)
    );
    // After the window, or for another channel, a new answer.
    assert_eq!(
        direct(&mut s, 4, 20, 200 + REPEAT_SUPPRESSION_MS),
        SinkOutcome::ChannelConfigurationSent
    );
    assert_eq!(responses(&mut s, 1_300).len(), 1);
    assert_eq!(
        direct(&mut s, 5, 11, 1_300),
        SinkOutcome::ChannelConfigurationSent
    );
    assert_eq!(responses(&mut s, 1_300)[..], [answer(OWN, 11, 0x14)]);
}

#[test]
fn a_direct_channel_request_needs_a_local_temp_master() {
    let mut s = sink();
    s.enter_commissioning(0, false).unwrap();
    while s.poll(0).is_some() {}
    assert_eq!(
        direct(&mut s, 1, 15, 0),
        SinkOutcome::Dropped(SinkDrop::NoTempMaster)
    );
    assert!(responses(&mut s, 0).is_empty());
}

fn commissioning_notification(
    gpd: GpdId,
    rx_after_tx: bool,
    counter: u32,
) -> heapless::Vec<u8, 32> {
    let byte = crate::commissioning::channel_request(25, 26).unwrap();
    let payload = [byte];
    let n = CommissioningNotification {
        gpd_id: gpd,
        rx_after_tx,
        security_level: SecurityLevel::None,
        key_type: KeyType::NoKey,
        security_failed: None,
        bidirectional_cap: true,
        frame_counter: counter,
        command_id: cmd::CHANNEL_REQUEST,
        payload: Some(&payload),
        proxy_info: ProxyInfo::Link {
            gpp_short: PROXY,
            link: GppGpdLink(0xC0),
        },
    };
    let mut b = [0u8; 32];
    let len = n.encode(&mut b).unwrap();
    heapless::Vec::from_slice(&b[..len]).unwrap()
}

fn server(s: &mut S, command_id: u8, payload: &[u8], now: u32) -> Option<SinkOutcome> {
    let (status, outcome) = s
        .on_server_command(
            &mut SoftwareAesProvider,
            &ServerCommand {
                src_nwk: PROXY,
                unicast: false,
                tsn: 7,
                command_id,
                payload,
            },
            now,
        )
        .unwrap();
    assert_eq!(status, zcl_status::SUCCESS);
    outcome
}

#[test]
fn a_tunnelled_channel_request_is_answered_through_the_forwarding_proxy() {
    let mut s = sink();
    let cn = commissioning_notification(GpdId::SrcId(0), true, 9);
    assert_eq!(
        server(&mut s, server_cmd::COMMISSIONING_NOTIFICATION, &cn, 0),
        Some(SinkOutcome::Dropped(SinkDrop::NotCommissioning))
    );
    s.enter_commissioning(0, false).unwrap();
    while s.poll(0).is_some() {}
    assert_eq!(
        server(&mut s, server_cmd::COMMISSIONING_NOTIFICATION, &cn, 10),
        Some(SinkOutcome::ChannelConfigurationSent)
    );
    assert_eq!(responses(&mut s, 10)[..], [answer(PROXY, 25, 0x14)]);
    // Only GPD ID 0 sends Channel Requests; a proxy that saw no receive
    // window cannot be the TempMaster.
    let cn = commissioning_notification(GpdId::SrcId(0x1234_5678), true, 10);
    assert_eq!(
        server(&mut s, server_cmd::COMMISSIONING_NOTIFICATION, &cn, 20),
        Some(SinkOutcome::Dropped(SinkDrop::Invalid))
    );
    let cn = commissioning_notification(GpdId::SrcId(0), false, 11);
    assert_eq!(
        server(&mut s, server_cmd::COMMISSIONING_NOTIFICATION, &cn, 30),
        Some(SinkOutcome::Dropped(SinkDrop::NoTempMaster))
    );
    assert!(responses(&mut s, 30).is_empty());
}

#[test]
fn a_channel_request_in_a_notification_is_answered_too() {
    let mut s = sink();
    s.enter_commissioning(0, false).unwrap();
    while s.poll(0).is_some() {}
    let byte = crate::commissioning::channel_request(12, 13).unwrap();
    let payload = [byte];
    let n = Notification {
        gpd_id: GpdId::SrcId(0),
        also_unicast: false,
        also_derived_group: false,
        also_commissioned_group: false,
        security_level: SecurityLevel::None,
        key_type: KeyType::NoKey,
        rx_after_tx: true,
        tx_queue_full: false,
        bidirectional_cap: true,
        frame_counter: 3,
        command_id: cmd::CHANNEL_REQUEST,
        payload: Some(&payload),
        proxy_info: ProxyInfo::Link {
            gpp_short: PROXY,
            link: GppGpdLink(0xC0),
        },
    };
    let mut b = [0u8; 32];
    let len = n.encode(&mut b).unwrap();
    assert_eq!(
        server(&mut s, server_cmd::NOTIFICATION, &b[..len], 5),
        Some(SinkOutcome::ChannelConfigurationSent)
    );
    assert_eq!(responses(&mut s, 5)[..], [answer(PROXY, 12, 0x14)]);
}

#[test]
fn gps_functionality_advertises_gpd_maintenance() {
    assert_eq!(MAINTENANCE_FUNCTIONALITY, functionality::GPD_MAINTENANCE);
    let s = sink();
    let mut out = [0u8; 4];
    let (_, len) = s
        .read_attribute(attr::GPS_FUNCTIONALITY, &mut out)
        .unwrap()
        .unwrap();
    let value = u32::from_le_bytes([out[0], out[1], out[2], 0]);
    assert_eq!(len, 3);
    assert_ne!(value & functionality::GPD_MAINTENANCE, 0);
}
