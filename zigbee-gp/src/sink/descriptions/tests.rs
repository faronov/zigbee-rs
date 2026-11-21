//! Application Description collection on the sink.

use super::*;
use crate::app_description::tests::{HUMIDITY, TEMPERATURE, frame as adcf};
use crate::cluster::{CommissioningNotification, GppGpdLink, ProxyInfo, Response};
use crate::commissioning::{ApplicationInfo, CommissioningSecurity};
use crate::frame::{FrameType, GpdfParams, parse, parse_mac};
use crate::proxy::TxFrame;
use crate::security::{self, Direction};
use zigbee_crypto::{AesKey, SoftwareAesProvider};

const KEY: AesKey = [0xC5; 16];
const SRC: u32 = 0x0BAD_CAFE;
const GPD: GpdId = GpdId::SrcId(SRC);
const PROXY: u16 = 0x1111;

type S = Sink<4, 4>;

fn sink() -> S {
    let mut s = S::new(SinkConfig::default());
    s.set_own_address(0x4A76, [1, 2, 3, 4, 5, 6, 7, 8], 0x1A62);
    s.set_operational_channel(15);
    s.enter_commissioning(0, false).unwrap();
    s
}

/// A multi-sensor Commissioning payload: L3 individual key (TC-LK
/// wrapped), counter 10, "application description follows".
fn commissioning() -> heapless::Vec<u8, 64> {
    let w = security::wrap_key(
        &mut SoftwareAesProvider,
        &security::DEFAULT_TC_LINK_KEY,
        &GPD,
        Direction::FromGpd,
        0,
        &KEY,
    )
    .unwrap();
    let p = CommissioningPayload {
        device_id: 0x07,
        mac_seq_num_capability: true,
        rx_on_capability: false,
        pan_id_request: false,
        security_key_request: false,
        fixed_location: true,
        security: Some(CommissioningSecurity {
            level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key: Some((w.key, Some(u32::from_le_bytes(w.mic)))),
            outgoing_counter: Some(10),
        }),
        app_info: Some(ApplicationInfo {
            app_description_follows: true,
            ..ApplicationInfo::default()
        }),
    };
    let mut b = [0u8; 64];
    let n = p.encode(&mut b).unwrap();
    heapless::Vec::from_slice(&b[..n]).unwrap()
}

/// A GPDF from the test GPD as received directly.
fn direct(
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
    let gpdf = parse(&raw[mac.payload_offset..len], mac.seq, None).unwrap();
    s.on_gpdf(&mut SoftwareAesProvider, &gpdf, counter * 100)
}

/// The same command tunnelled by a proxy in commissioning mode.
fn tunnelled(
    s: &mut S,
    counter: u32,
    rx_after_tx: bool,
    command: u8,
    payload: &[u8],
) -> SinkOutcome {
    let n = CommissioningNotification {
        gpd_id: GPD,
        rx_after_tx,
        security_level: SecurityLevel::None,
        key_type: KeyType::NoKey,
        security_failed: None,
        bidirectional_cap: true,
        frame_counter: counter,
        command_id: command,
        payload: Some(payload),
        proxy_info: ProxyInfo::Link {
            gpp_short: PROXY,
            link: GppGpdLink(0xC0),
        },
    };
    let mut b = [0u8; 127];
    let len = n.encode(&mut b).unwrap();
    let command = ServerCommand {
        src_nwk: PROXY,
        unicast: false,
        tsn: 0x42,
        command_id: server_cmd::COMMISSIONING_NOTIFICATION,
        payload: &b[..len],
    };
    let (status, outcome) = s
        .on_server_command(&mut SoftwareAesProvider, &command, counter * 100)
        .unwrap();
    assert_eq!(status, zcl_status::SUCCESS);
    outcome.unwrap()
}

fn events(s: &mut S) -> heapless::Vec<SinkEvent, 4> {
    let mut v = heapless::Vec::new();
    while let Some(e) = s.take_event() {
        v.push(e).unwrap();
    }
    v
}

fn reports(total: u8, count: u8, raw: &[&[u8]]) -> SinkEvent {
    let mut reports = heapless::Vec::new();
    for r in raw {
        reports.extend_from_slice(r).unwrap();
    }
    SinkEvent::ApplicationDescription(ApplicationDescriptionReports {
        gpd_id: GPD,
        total_reports: total,
        report_count: count,
        reports,
    })
}

#[test]
fn pairing_waits_for_all_reports() {
    let mut s = sink();
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::None,
            1,
            cmd::COMMISSIONING,
            &commissioning()
        ),
        SinkOutcome::CommissioningStep
    );
    assert!(s.entries().is_empty());
    assert_eq!(s.collecting_application_description(), Some(GPD));
    assert_eq!(s.next_deadline(), Some(100 + APP_DESCRIPTION_TIMEOUT_MS));
    events(&mut s);

    // Report 1 of 2, protected with the new key: found in the pending entry.
    let one = adcf(2, &[&HUMIDITY]);
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::Encrypted,
            11,
            cmd::APPLICATION_DESCRIPTION,
            &one
        ),
        SinkOutcome::CommissioningStep
    );
    assert_eq!(events(&mut s).as_slice(), &[reports(2, 1, &[&HUMIDITY])]);
    // Other commands do not complete a collecting commissioning.
    assert_eq!(
        direct(&mut s, SecurityLevel::Encrypted, 12, cmd::SUCCESS, &[]),
        SinkOutcome::Dropped(SinkDrop::NoEntry)
    );
    // A repetition adds nothing; the remaining report completes the pairing.
    let both = adcf(2, &[&TEMPERATURE, &HUMIDITY]);
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::Encrypted,
            13,
            cmd::APPLICATION_DESCRIPTION,
            &both
        ),
        SinkOutcome::Commissioned
    );
    assert_eq!(s.entries().len(), 1);
    let e = &s.entries()[0];
    assert_eq!((e.device_id, e.key, e.frame_counter), (0x07, Some(KEY), 10));
    assert_eq!(
        events(&mut s).as_slice(),
        &[
            reports(2, 1, &[&TEMPERATURE]),
            SinkEvent::Commissioned {
                gpd_id: GPD,
                device_id: 0x07
            },
            SinkEvent::CommissioningLeft(SinkCommissioningExit::FirstPairing),
        ]
    );
    assert_eq!(s.collecting_application_description(), None);
    // Further descriptions are not collected.
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::Encrypted,
            14,
            cmd::APPLICATION_DESCRIPTION,
            &both
        ),
        SinkOutcome::Dropped(SinkDrop::NoEntry)
    );
}

#[test]
fn collection_times_out() {
    let mut s = sink();
    direct(
        &mut s,
        SecurityLevel::None,
        1,
        cmd::COMMISSIONING,
        &commissioning(),
    );
    events(&mut s);
    let deadline = 100 + APP_DESCRIPTION_TIMEOUT_MS;
    assert!(s.poll(deadline - 1).is_none());
    assert_eq!(s.collecting_application_description(), Some(GPD));
    assert!(s.poll(deadline).is_none());
    assert_eq!(s.collecting_application_description(), None);
    assert_eq!(
        events(&mut s).as_slice(),
        &[SinkEvent::CommissioningFailed {
            gpd_id: GPD,
            reason: SinkDrop::DescriptionTimeout
        }]
    );
    assert!(s.entries().is_empty());
    // The pending entry went with it: its key no longer opens frames.
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::Encrypted,
            11,
            cmd::APPLICATION_DESCRIPTION,
            &adcf(1, &[&TEMPERATURE])
        ),
        SinkOutcome::Dropped(SinkDrop::AuthFailed)
    );
}

#[test]
fn rejects_inconsistent_descriptions() {
    let mut s = sink();
    direct(
        &mut s,
        SecurityLevel::None,
        1,
        cmd::COMMISSIONING,
        &commissioning(),
    );
    // Report ID 1 of 1 total.
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::None,
            2,
            cmd::APPLICATION_DESCRIPTION,
            &adcf(1, &[&HUMIDITY])
        ),
        SinkOutcome::Dropped(SinkDrop::Invalid)
    );
    // Malformed, then a total that changes.
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::None,
            3,
            cmd::APPLICATION_DESCRIPTION,
            &[2, 1, 0]
        ),
        SinkOutcome::Dropped(SinkDrop::Invalid)
    );
    direct(
        &mut s,
        SecurityLevel::None,
        4,
        cmd::APPLICATION_DESCRIPTION,
        &adcf(2, &[&TEMPERATURE]),
    );
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::None,
            5,
            cmd::APPLICATION_DESCRIPTION,
            &adcf(3, &[&HUMIDITY])
        ),
        SinkOutcome::Dropped(SinkDrop::Invalid)
    );
    // More than MAX_REPORTS.
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::None,
            6,
            cmd::APPLICATION_DESCRIPTION,
            &adcf(33, &[&TEMPERATURE])
        ),
        SinkOutcome::Dropped(SinkDrop::Unsupported)
    );
    assert!(s.entries().is_empty());
}

#[test]
fn descriptions_outside_a_collection_are_dropped() {
    let mut s = sink();
    let one = adcf(1, &[&TEMPERATURE]);
    assert_eq!(
        direct(
            &mut s,
            SecurityLevel::None,
            1,
            cmd::APPLICATION_DESCRIPTION,
            &one
        ),
        SinkOutcome::Dropped(SinkDrop::NoEntry)
    );
    // A Channel Request is not an Application Description: unsupported, or
    // with `channel-request` invalid from a GPD other than SrcID 0.
    let expected = if cfg!(feature = "channel-request") {
        SinkDrop::Invalid
    } else {
        SinkDrop::Unsupported
    };
    assert_eq!(
        tunnelled(&mut s, 2, true, cmd::CHANNEL_REQUEST, &[0x44]),
        SinkOutcome::Dropped(expected)
    );
}

/// Through a proxy: the last report asks for a Commissioning Reply, the
/// GPD's Success completes the pairing (gecko_sdk
/// `lastReportIdHasRxAfterTx`).
#[test]
fn bidirectional_reply_after_the_last_report() {
    let mut s = sink();
    // rxAfterTx with "application description follows" is dropped.
    assert_eq!(
        tunnelled(&mut s, 1, true, cmd::COMMISSIONING, &commissioning()),
        SinkOutcome::Dropped(SinkDrop::Unsupported)
    );
    assert_eq!(
        tunnelled(&mut s, 2, false, cmd::COMMISSIONING, &commissioning()),
        SinkOutcome::CommissioningStep
    );
    let both = adcf(2, &[&TEMPERATURE, &HUMIDITY]);
    assert_eq!(
        tunnelled(&mut s, 3, true, cmd::APPLICATION_DESCRIPTION, &both),
        SinkOutcome::CommissioningReplySent
    );
    assert!(s.entries().is_empty());
    let tx = s.poll(300).unwrap().tx;
    let TxFrame::GpCommand {
        command_id,
        payload,
    } = &tx.frame
    else {
        panic!("unexpected frame {:?}", tx.frame);
    };
    assert_eq!(*command_id, crate::cluster::client_cmd::RESPONSE);
    let r = Response::decode(payload).unwrap();
    assert_eq!(
        (r.temp_master_short, r.command_id),
        (PROXY, cmd::COMMISSIONING_REPLY)
    );
    assert_eq!(r.payload, &[0x98]);
    // The GPD's Success, protected with its new key.
    assert_eq!(
        direct(&mut s, SecurityLevel::Encrypted, 11, cmd::SUCCESS, &[]),
        SinkOutcome::Commissioned
    );
    assert_eq!(s.entries()[0].key, Some(KEY));
}

#[test]
fn leaving_commissioning_ends_the_collection() {
    let mut s = sink();
    direct(
        &mut s,
        SecurityLevel::None,
        1,
        cmd::COMMISSIONING,
        &commissioning(),
    );
    s.exit_commissioning();
    assert_eq!(s.collecting_application_description(), None);
    assert_eq!(s.next_deadline(), None);
}

#[cfg(feature = "translation-table")]
mod pairing_configuration {
    use super::*;
    use crate::config::{ApplicationInformation, PairingConfiguration, pairing_action};

    fn pc(actions: u8, reports: Option<(u8, u8, &[u8])>) -> heapless::Vec<u8, 96> {
        let pc = PairingConfiguration {
            actions,
            gpd_id: GPD,
            communication_mode: crate::types::CommunicationMode::DerivedGroupcast,
            mac_seq_cap: true,
            rx_on_cap: false,
            fixed_location: true,
            device_id: 0x07,
            groups: &[],
            assigned_alias: None,
            groupcast_radius: 0,
            security: None,
            frame_counter: Some(10),
            number_of_paired_endpoints: crate::config::paired_endpoints::SINK_DERIVES,
            paired_endpoints: &[],
            application_information: reports.map(|r| ApplicationInformation {
                bits: crate::config::app_info::APPLICATION_DESCRIPTION,
                application_description: Some(r),
                ..ApplicationInformation::default()
            }),
        };
        let mut b = [0u8; 96];
        let n = pc.encode(&mut b).unwrap();
        heapless::Vec::from_slice(&b[..n]).unwrap()
    }

    fn send(s: &mut S, payload: &[u8]) -> (u8, Option<SinkOutcome>) {
        let command = ServerCommand {
            src_nwk: 0x0000,
            unicast: true,
            tsn: 0x33,
            command_id: server_cmd::PAIRING_CONFIGURATION,
            payload,
        };
        s.on_server_command(&mut SoftwareAesProvider, &command, 1_000)
            .unwrap()
    }

    #[test]
    fn extend_with_and_then_more_descriptions() {
        let mut s = sink();
        // Unprotected pairings allowed.
        s.set_config(SinkConfig {
            security_level: 0,
            ..SinkConfig::default()
        });
        s.exit_commissioning();
        events(&mut s);
        // Unknown GPD.
        assert_eq!(
            send(
                &mut s,
                &pc(
                    pairing_action::APPLICATION_DESCRIPTION,
                    Some((2, 1, &HUMIDITY))
                )
            ),
            (zcl_status::NOT_FOUND, None)
        );
        // Malformed descriptors refuse the Extend.
        assert_eq!(
            send(
                &mut s,
                &pc(pairing_action::EXTEND, Some((1, 1, &[0, 0, 0])))
            )
            .0,
            0x01
        );
        assert_eq!(
            send(
                &mut s,
                &pc(pairing_action::EXTEND, Some((2, 1, &TEMPERATURE)))
            ),
            (zcl_status::SUCCESS, Some(SinkOutcome::Commissioned))
        );
        assert_eq!(
            events(&mut s).as_slice(),
            &[
                SinkEvent::Commissioned {
                    gpd_id: GPD,
                    device_id: 0x07
                },
                reports(2, 1, &[&TEMPERATURE]),
            ]
        );
        assert_eq!(
            send(
                &mut s,
                &pc(
                    pairing_action::APPLICATION_DESCRIPTION,
                    Some((2, 1, &HUMIDITY))
                )
            ),
            (zcl_status::SUCCESS, None)
        );
        assert_eq!(events(&mut s).as_slice(), &[reports(2, 1, &[&HUMIDITY])]);
        assert_eq!(
            send(&mut s, &pc(pairing_action::APPLICATION_DESCRIPTION, None)).0,
            zcl_status::INVALID_FIELD
        );
    }

    /// Action 0b101 completes a GPD's own collection.
    #[test]
    fn completes_a_collection() {
        let mut s = sink();
        direct(
            &mut s,
            SecurityLevel::None,
            1,
            cmd::COMMISSIONING,
            &commissioning(),
        );
        direct(
            &mut s,
            SecurityLevel::None,
            2,
            cmd::APPLICATION_DESCRIPTION,
            &adcf(2, &[&TEMPERATURE]),
        );
        assert_eq!(
            send(
                &mut s,
                &pc(
                    pairing_action::APPLICATION_DESCRIPTION,
                    Some((2, 1, &HUMIDITY))
                )
            ),
            (zcl_status::SUCCESS, Some(SinkOutcome::Commissioned))
        );
        assert_eq!(s.entries()[0].key, Some(KEY));
    }
}

/// A multi-sensor asking for a key gets it in the Commissioning Reply sent
/// after its last report (`key-provisioning`).
#[cfg(feature = "key-provisioning")]
#[test]
fn the_reply_after_the_last_report_provides_the_key() {
    use crate::commissioning::CommissioningReply;
    use crate::security::WrappedKey;

    let mut s = sink();
    let shared = [0x3C; 16];
    s.set_shared_key(KeyType::GpdGroupKey, shared);
    // L3 individual key (TC-LK wrapped), asks for a key, descriptions follow.
    let mut p = CommissioningPayload::decode(&commissioning()).unwrap();
    p.security_key_request = true;
    let mut b = [0u8; 64];
    let n = p.encode(&mut b).unwrap();
    assert_eq!(
        tunnelled(&mut s, 2, false, cmd::COMMISSIONING, &b[..n]),
        SinkOutcome::CommissioningStep
    );
    let both = adcf(2, &[&TEMPERATURE, &HUMIDITY]);
    assert_eq!(
        tunnelled(&mut s, 3, true, cmd::APPLICATION_DESCRIPTION, &both),
        SinkOutcome::CommissioningReplySent
    );
    let tx = s.poll(300).unwrap().tx;
    let TxFrame::GpCommand { payload, .. } = &tx.frame else {
        panic!("unexpected frame {:?}", tx.frame);
    };
    let r = Response::decode(payload).unwrap();
    let reply = CommissioningReply::decode(r.payload).unwrap();
    assert_eq!(reply.key_type, KeyType::GpdGroupKey);
    // Protected with gpLinkKey like the GPD's own key, counter 10 + 1.
    let (key, Some((mic, fc))) = reply.key.unwrap() else {
        panic!("the key must be protected");
    };
    assert_eq!(fc, 11);
    let clear = security::unwrap_key(
        &mut SoftwareAesProvider,
        &security::DEFAULT_TC_LINK_KEY,
        &GPD,
        Direction::ToGpd,
        fc,
        &WrappedKey {
            key,
            mic: mic.to_le_bytes(),
        },
    )
    .unwrap();
    assert_eq!(clear, shared);
    // The GPD's Success under the provided key completes the pairing.
    let mut raw = [0u8; 127];
    let params = GpdfParams {
        frame_type: FrameType::Data,
        auto_commissioning: false,
        src_id: Some(SRC),
        security_level: SecurityLevel::Encrypted,
        individual_key: false,
        rx_after_tx: false,
        to_gpd: false,
        frame_counter: 11,
    };
    let len = security::build_gpdf(
        &mut SoftwareAesProvider,
        Some(&shared),
        &params,
        11,
        cmd::SUCCESS,
        &[],
        &mut raw,
    )
    .unwrap();
    let mac = parse_mac(&raw[..len]).unwrap();
    let gpdf = parse(&raw[mac.payload_offset..len], mac.seq, None).unwrap();
    assert_eq!(
        s.on_gpdf(&mut SoftwareAesProvider, &gpdf, 400),
        SinkOutcome::Commissioned
    );
    assert_eq!(
        (s.entries()[0].key, s.entries()[0].key_type),
        (Some(shared), KeyType::GpdGroupKey)
    );
}
