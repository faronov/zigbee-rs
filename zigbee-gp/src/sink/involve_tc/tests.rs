use super::super::{
    ServerCommand, Sink, SinkCommissioningMode, SinkConfig, SinkDrop, SinkOutcome,
    security_level_attr, sink_commissioning_options, zcl_status,
};
use super::JoinKind;
use crate::Error;
use crate::cluster::{attr, server_cmd};
use crate::commissioning::CommissioningPayload;
use crate::frame::{FrameType, GpdfParams, parse, parse_mac};
use crate::security;
use crate::types::{SecurityLevel, cmd};
use zigbee_crypto::SoftwareAesProvider;

const SRC: u32 = 0x1234_5678;
const BITMAP8: u8 = 0x18;

type S = Sink<4, 4>;

fn sink() -> S {
    let mut s = S::new(SinkConfig {
        endpoint: Some(1),
        security_level: 0x00,
        ..SinkConfig::default()
    });
    s.set_own_address(0x4A76, [1, 2, 3, 4, 5, 6, 7, 8], 0x1A62);
    s.set_operational_channel(15);
    s
}

fn security_level(s: &S) -> u8 {
    let mut out = [0u8; 1];
    s.read_attribute(attr::GPS_SECURITY_LEVEL, &mut out)
        .unwrap();
    out[0]
}

/// An unprotected Commissioning GPDF (no key) heard directly.
fn commission(s: &mut S, seq: u8) -> SinkOutcome {
    let p = CommissioningPayload {
        device_id: 0x02,
        mac_seq_num_capability: true,
        rx_on_capability: false,
        pan_id_request: false,
        security_key_request: false,
        fixed_location: true,
        security: None,
        app_info: None,
    };
    let mut b = [0u8; 32];
    let n = p.encode(&mut b).unwrap();
    let mut raw = [0u8; 127];
    let len = security::build_gpdf(
        &mut SoftwareAesProvider,
        None,
        &GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(SRC),
            security_level: SecurityLevel::None,
            individual_key: false,
            rx_after_tx: false,
            to_gpd: false,
            frame_counter: 0,
        },
        seq,
        cmd::COMMISSIONING,
        &b[..n],
        &mut raw,
    )
    .unwrap();
    let mac = parse_mac(&raw[..len]).unwrap();
    let frame = parse(&raw[mac.payload_offset..len], mac.seq, None).unwrap();
    s.on_gpdf(&mut SoftwareAesProvider, &frame, u32::from(seq))
}

fn sink_commissioning_mode(s: &mut S) -> u8 {
    let m = SinkCommissioningMode {
        options: sink_commissioning_options::ACTION,
        gpm_addr_for_security: 0xFFFF,
        gpm_addr_for_pairing: 0xFFFF,
        sink_endpoint: 0xFF,
    };
    let mut b = [0u8; 8];
    let n = m.encode(&mut b).unwrap();
    s.on_server_command(
        &mut SoftwareAesProvider,
        &ServerCommand {
            src_nwk: 0x0000,
            unicast: true,
            tsn: 1,
            command_id: server_cmd::SINK_COMMISSIONING_MODE,
            payload: &b[..n],
        },
        0,
    )
    .unwrap()
    .0
}

#[test]
fn involve_tc_is_writable_and_stops_commissioning() {
    let mut s = sink();
    s.enter_commissioning(0, false).unwrap();
    // A tool sets InvolveTC during the window: the GPD is refused.
    assert_eq!(
        s.write_attribute(attr::GPS_SECURITY_LEVEL, BITMAP8, &[0x08]),
        zcl_status::SUCCESS
    );
    assert_eq!(security_level(&s), security_level_attr::INVOLVE_TC);
    assert_eq!(
        commission(&mut s, 1),
        SinkOutcome::Dropped(SinkDrop::InvolveTc)
    );
    assert!(s.entries().is_empty());
    s.exit_commissioning();
    assert_eq!(s.enter_commissioning(0, false), Err(Error::Unsupported));
    assert_eq!(sink_commissioning_mode(&mut s), zcl_status::UNSUP_COMMAND);
    // Bits above InvolveTC stay reserved.
    assert_eq!(
        s.write_attribute(attr::GPS_SECURITY_LEVEL, BITMAP8, &[0x18]),
        zcl_status::INVALID_VALUE
    );
    // Cleared again, commissioning works.
    assert_eq!(
        s.write_attribute(attr::GPS_SECURITY_LEVEL, BITMAP8, &[0x00]),
        zcl_status::SUCCESS
    );
    assert_eq!(sink_commissioning_mode(&mut s), zcl_status::SUCCESS);
    assert_eq!(commission(&mut s, 2), SinkOutcome::Commissioned);
}

#[test]
fn a_trust_center_sink_commissions_with_involve_tc_set() {
    let mut s = sink();
    s.write_attribute(attr::GPS_SECURITY_LEVEL, BITMAP8, &[0x08]);
    assert!(s.derive_involve_tc(JoinKind::TrustCenter));
    assert!(s.is_trust_center());
    assert_eq!(sink_commissioning_mode(&mut s), zcl_status::SUCCESS);
    assert_eq!(commission(&mut s, 1), SinkOutcome::Commissioned);
}

#[test]
fn involve_tc_follows_the_join_as_gecko_sdk_derives_it() {
    let mut s = sink();
    // Default global key: the configured value stays.
    assert!(!s.derive_involve_tc(JoinKind::CentralizedDefaultKey));
    s.write_attribute(attr::GPS_SECURITY_LEVEL, BITMAP8, &[0x09]);
    assert!(s.derive_involve_tc(JoinKind::CentralizedDefaultKey));
    // A distributed network clears it, keeping the other bits.
    assert!(!s.derive_involve_tc(JoinKind::Distributed));
    assert_eq!(security_level(&s), 0x01);
    // An install-code join sets it.
    assert!(s.derive_involve_tc(JoinKind::CentralizedPreconfiguredKey));
    assert_eq!(security_level(&s), 0x09);
    assert!(!s.is_trust_center());
    assert_eq!(s.enter_commissioning(0, false), Err(Error::Unsupported));
}
