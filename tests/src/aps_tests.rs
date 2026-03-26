//! Tests for the APS (Application Support Sub-layer) crate.
//!
//! Covers:
//! - APS frame construction and parsing (data, ack, command, group)
//! - Serialize/parse round-trip for various delivery modes
//! - Binding table: add, lookup, remove, duplicate rejection
//! - Group table: add, membership, remove, remove-all
//! - APS counter: wrapping increment
//! - Endpoint/cluster addressing in frames
//! - APS security header parse/serialize round-trip
//! - Security key table: add, find, remove, frame counter

use zigbee_aps::binding::*;
use zigbee_aps::frames::*;
use zigbee_aps::group::*;
use zigbee_aps::security::*;
use zigbee_aps::*;

// ── Helpers ──────────────────────────────────────────────────────

/// Build a unicast data frame header with given endpoints/cluster/profile.
fn make_unicast_data_header(
    dst_ep: u8,
    src_ep: u8,
    cluster: u16,
    profile: u16,
    counter: u8,
) -> ApsHeader {
    ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: Some(dst_ep),
        group_address: None,
        cluster_id: Some(cluster),
        profile_id: Some(profile),
        src_endpoint: Some(src_ep),
        aps_counter: counter,
        extended_header: None,
    }
}

// ════════════════════════════════════════════════════════════════
// 1. APS frame construction
// ════════════════════════════════════════════════════════════════

#[test]
fn test_unicast_data_frame_construction() {
    let hdr = make_unicast_data_header(0x01, 0x0A, 0x0006, PROFILE_HOME_AUTOMATION, 42);

    assert_eq!(hdr.frame_control.frame_type, ApsFrameType::Data as u8);
    assert_eq!(
        hdr.frame_control.delivery_mode,
        ApsDeliveryMode::Unicast as u8
    );
    assert_eq!(hdr.dst_endpoint, Some(0x01));
    assert_eq!(hdr.src_endpoint, Some(0x0A));
    assert_eq!(hdr.cluster_id, Some(0x0006));
    assert_eq!(hdr.profile_id, Some(PROFILE_HOME_AUTOMATION));
    assert_eq!(hdr.aps_counter, 42);
}

#[test]
fn test_ack_frame_construction() {
    let hdr = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Ack as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: Some(0x01),
        group_address: None,
        cluster_id: Some(0x0006),
        profile_id: Some(PROFILE_HOME_AUTOMATION),
        src_endpoint: Some(0x0A),
        aps_counter: 7,
        extended_header: None,
    };

    assert_eq!(hdr.frame_control.frame_type, ApsFrameType::Ack as u8);
    assert_eq!(hdr.aps_counter, 7);
    assert_eq!(hdr.dst_endpoint, Some(0x01));
}

#[test]
fn test_group_data_frame_construction() {
    let hdr = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Group as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: None,
        group_address: Some(0x1234),
        cluster_id: Some(0x0006),
        profile_id: Some(PROFILE_HOME_AUTOMATION),
        src_endpoint: Some(0x01),
        aps_counter: 10,
        extended_header: None,
    };

    assert_eq!(hdr.group_address, Some(0x1234));
    assert!(hdr.dst_endpoint.is_none());
}

// ════════════════════════════════════════════════════════════════
// 2. APS frame serialize / parse round-trip
// ════════════════════════════════════════════════════════════════

#[test]
fn test_unicast_data_frame_round_trip() {
    let original = make_unicast_data_header(0x01, 0x0A, 0x0006, PROFILE_HOME_AUTOMATION, 42);

    let mut buf = [0u8; 64];
    let len = original.serialize(&mut buf);
    assert!(len > 0);

    let (parsed, consumed) = ApsHeader::parse(&buf[..len]).expect("parse failed");
    assert_eq!(consumed, len);
    assert_eq!(parsed.frame_control.frame_type, ApsFrameType::Data as u8);
    assert_eq!(
        parsed.frame_control.delivery_mode,
        ApsDeliveryMode::Unicast as u8
    );
    assert_eq!(parsed.dst_endpoint, Some(0x01));
    assert_eq!(parsed.src_endpoint, Some(0x0A));
    assert_eq!(parsed.cluster_id, Some(0x0006));
    assert_eq!(parsed.profile_id, Some(PROFILE_HOME_AUTOMATION));
    assert_eq!(parsed.aps_counter, 42);
}

#[test]
fn test_group_data_frame_round_trip() {
    let original = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Group as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: None,
        group_address: Some(0xABCD),
        cluster_id: Some(0x0300),
        profile_id: Some(PROFILE_HOME_AUTOMATION),
        src_endpoint: Some(0x05),
        aps_counter: 99,
        extended_header: None,
    };

    let mut buf = [0u8; 64];
    let len = original.serialize(&mut buf);

    let (parsed, consumed) = ApsHeader::parse(&buf[..len]).expect("parse failed");
    assert_eq!(consumed, len);
    assert_eq!(parsed.group_address, Some(0xABCD));
    assert!(parsed.dst_endpoint.is_none());
    assert_eq!(parsed.cluster_id, Some(0x0300));
    assert_eq!(parsed.src_endpoint, Some(0x05));
    assert_eq!(parsed.aps_counter, 99);
}

#[test]
fn test_ack_frame_round_trip() {
    let original = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Ack as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: Some(0x01),
        group_address: None,
        cluster_id: Some(0x0006),
        profile_id: Some(PROFILE_HOME_AUTOMATION),
        src_endpoint: Some(0x0A),
        aps_counter: 7,
        extended_header: None,
    };

    let mut buf = [0u8; 64];
    let len = original.serialize(&mut buf);

    let (parsed, _) = ApsHeader::parse(&buf[..len]).expect("parse failed");
    assert_eq!(parsed.frame_control.frame_type, ApsFrameType::Ack as u8);
    assert_eq!(parsed.aps_counter, 7);
    assert_eq!(parsed.cluster_id, Some(0x0006));
    assert_eq!(parsed.profile_id, Some(PROFILE_HOME_AUTOMATION));
}

#[test]
fn test_command_frame_round_trip() {
    let original = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Command as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: true,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: None,
        group_address: None,
        cluster_id: None,
        profile_id: None,
        src_endpoint: None, // Command frames don't have endpoints per Zigbee spec §2.2.5.1
        aps_counter: 200,
        extended_header: None,
    };

    let mut buf = [0u8; 64];
    let len = original.serialize(&mut buf);

    let (parsed, consumed) = ApsHeader::parse(&buf[..len]).expect("parse failed");
    assert_eq!(consumed, len);
    assert_eq!(parsed.frame_control.frame_type, ApsFrameType::Command as u8);
    assert!(parsed.frame_control.security);
    assert!(parsed.cluster_id.is_none());
    assert!(parsed.profile_id.is_none());
    assert_eq!(parsed.src_endpoint, None); // Command frames have no endpoints
    assert_eq!(parsed.aps_counter, 200);
}

// ════════════════════════════════════════════════════════════════
// 3. Frame control bitfield
// ════════════════════════════════════════════════════════════════

#[test]
fn test_frame_control_serialize_parse_round_trip() {
    let fc = ApsFrameControl {
        frame_type: ApsFrameType::Data as u8,
        delivery_mode: ApsDeliveryMode::Unicast as u8,
        ack_format: false,
        security: true,
        ack_request: true,
        extended_header: false,
    };
    let raw = fc.serialize();
    let parsed = ApsFrameControl::parse(raw);

    assert_eq!(parsed.frame_type, ApsFrameType::Data as u8);
    assert_eq!(parsed.delivery_mode, ApsDeliveryMode::Unicast as u8);
    assert!(!parsed.ack_format);
    assert!(parsed.security);
    assert!(parsed.ack_request);
    assert!(!parsed.extended_header);
}

#[test]
fn test_frame_control_all_bits_set() {
    let fc = ApsFrameControl {
        frame_type: ApsFrameType::InterPan as u8,
        delivery_mode: ApsDeliveryMode::Group as u8,
        ack_format: true,
        security: true,
        ack_request: true,
        extended_header: true,
    };
    let raw = fc.serialize();
    assert_eq!(raw, 0xFF);

    let parsed = ApsFrameControl::parse(raw);
    assert_eq!(parsed.frame_type, ApsFrameType::InterPan as u8);
    assert_eq!(parsed.delivery_mode, ApsDeliveryMode::Group as u8);
    assert!(parsed.ack_format);
    assert!(parsed.security);
    assert!(parsed.ack_request);
    assert!(parsed.extended_header);
}

// ════════════════════════════════════════════════════════════════
// 4. Binding table
// ════════════════════════════════════════════════════════════════

#[test]
fn test_binding_table_add_and_lookup() {
    let mut bt = BindingTable::new();
    let src = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    let dst = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18];

    let entry = BindingEntry::unicast(src, 1, 0x0006, dst, 1);
    assert!(bt.add(entry).is_ok());
    assert_eq!(bt.len(), 1);

    let results: Vec<_> = bt.find_by_source(&src, 1, 0x0006).collect();
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].cluster_id, 0x0006);
    assert_eq!(
        results[0].dst,
        BindingDst::Unicast {
            dst_addr: dst,
            dst_endpoint: 1,
        }
    );
}

#[test]
fn test_binding_table_duplicate_rejected() {
    let mut bt = BindingTable::new();
    let src = [0x01; 8];
    let dst = [0x02; 8];

    let e1 = BindingEntry::unicast(src, 1, 0x0006, dst, 1);
    let e2 = BindingEntry::unicast(src, 1, 0x0006, dst, 1);
    assert!(bt.add(e1).is_ok());
    assert!(bt.add(e2).is_err());
    assert_eq!(bt.len(), 1);
}

#[test]
fn test_binding_table_remove() {
    let mut bt = BindingTable::new();
    let src = [0x01; 8];
    let dst = [0x02; 8];

    let entry = BindingEntry::unicast(src, 1, 0x0006, dst, 1);
    bt.add(entry).unwrap();
    assert_eq!(bt.len(), 1);

    let removed = bt.remove(
        &src,
        1,
        0x0006,
        &BindingDst::Unicast {
            dst_addr: dst,
            dst_endpoint: 1,
        },
    );
    assert!(removed);
    assert!(bt.is_empty());
}

#[test]
fn test_binding_table_group_binding() {
    let mut bt = BindingTable::new();
    let src = [0xAA; 8];
    let group_entry = BindingEntry::group(src, 1, 0x0006, 0x1234);
    assert!(bt.add(group_entry).is_ok());

    let results: Vec<_> = bt.find_by_source(&src, 1, 0x0006).collect();
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].dst, BindingDst::Group(0x1234));
}

#[test]
fn test_binding_table_find_by_cluster() {
    let mut bt = BindingTable::new();
    let src = [0x01; 8];
    let dst = [0x02; 8];

    bt.add(BindingEntry::unicast(src, 1, 0x0006, dst, 1))
        .unwrap();
    bt.add(BindingEntry::unicast(src, 2, 0x0300, dst, 2))
        .unwrap();
    bt.add(BindingEntry::unicast(src, 3, 0x0006, dst, 3))
        .unwrap();

    let cluster_6: Vec<_> = bt.find_by_cluster(0x0006).collect();
    assert_eq!(cluster_6.len(), 2);
}

// ════════════════════════════════════════════════════════════════
// 5. Group table
// ════════════════════════════════════════════════════════════════

#[test]
fn test_group_table_add_and_membership() {
    let mut gt = GroupTable::new();
    assert!(gt.add_group(0x1234, 1));
    assert!(gt.is_member(0x1234, 1));
    assert!(!gt.is_member(0x1234, 2));
    assert!(!gt.is_member(0x5678, 1));
}

#[test]
fn test_group_table_multiple_endpoints() {
    let mut gt = GroupTable::new();
    assert!(gt.add_group(0x1234, 1));
    assert!(gt.add_group(0x1234, 2));
    assert!(gt.add_group(0x1234, 3));
    assert_eq!(gt.len(), 1); // single group

    let group = gt.find(0x1234).unwrap();
    assert_eq!(group.endpoint_list.len(), 3);
}

#[test]
fn test_group_table_remove_endpoint() {
    let mut gt = GroupTable::new();
    gt.add_group(0x1234, 1);
    gt.add_group(0x1234, 2);

    assert!(gt.remove_group(0x1234, 1));
    assert!(!gt.is_member(0x1234, 1));
    assert!(gt.is_member(0x1234, 2));
    assert_eq!(gt.len(), 1); // group still exists

    assert!(gt.remove_group(0x1234, 2));
    assert_eq!(gt.len(), 0); // group removed when empty
}

#[test]
fn test_group_table_remove_all_groups() {
    let mut gt = GroupTable::new();
    gt.add_group(0x1111, 1);
    gt.add_group(0x2222, 1);
    gt.add_group(0x3333, 2);

    gt.remove_all_groups(1);
    assert!(!gt.is_member(0x1111, 1));
    assert!(!gt.is_member(0x2222, 1));
    assert!(gt.is_member(0x3333, 2));
    assert_eq!(gt.len(), 1);
}

// ════════════════════════════════════════════════════════════════
// 6. Endpoint / cluster addressing in frames
// ════════════════════════════════════════════════════════════════

#[test]
fn test_zdo_endpoint_frame() {
    let hdr = make_unicast_data_header(ZDO_ENDPOINT, ZDO_ENDPOINT, 0x0000, PROFILE_ZDP, 0);

    let mut buf = [0u8; 64];
    let len = hdr.serialize(&mut buf);
    let (parsed, _) = ApsHeader::parse(&buf[..len]).unwrap();

    assert_eq!(parsed.dst_endpoint, Some(ZDO_ENDPOINT));
    assert_eq!(parsed.src_endpoint, Some(ZDO_ENDPOINT));
    assert_eq!(parsed.profile_id, Some(PROFILE_ZDP));
}

#[test]
fn test_broadcast_endpoint_frame() {
    let hdr =
        make_unicast_data_header(BROADCAST_ENDPOINT, 0x01, 0x0006, PROFILE_HOME_AUTOMATION, 5);

    let mut buf = [0u8; 64];
    let len = hdr.serialize(&mut buf);
    let (parsed, _) = ApsHeader::parse(&buf[..len]).unwrap();

    assert_eq!(parsed.dst_endpoint, Some(BROADCAST_ENDPOINT));
    assert_eq!(parsed.src_endpoint, Some(0x01));
}

// ════════════════════════════════════════════════════════════════
// 7. APS security header parse / serialize round-trip
// ════════════════════════════════════════════════════════════════

#[test]
fn test_security_header_round_trip_no_ext_nonce() {
    let sec_hdr = ApsSecurityHeader {
        security_control: ApsSecurityHeader::APS_DEFAULT,
        frame_counter: 0x0000_1234,
        source_address: None,
        key_seq_number: None,
    };

    let mut buf = [0u8; 32];
    let len = sec_hdr.serialize(&mut buf);
    assert_eq!(len, 5); // 1 + 4

    let (parsed, consumed) = ApsSecurityHeader::parse(&buf[..len]).unwrap();
    assert_eq!(consumed, 5);
    assert_eq!(parsed.frame_counter, 0x0000_1234);
    assert!(parsed.source_address.is_none());
    assert!(parsed.key_seq_number.is_none());
}

#[test]
fn test_security_header_round_trip_with_ext_nonce() {
    let addr = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    let sec_hdr = ApsSecurityHeader {
        security_control: ApsSecurityHeader::APS_DEFAULT_EXT_NONCE,
        frame_counter: 0xDEAD_BEEF,
        source_address: Some(addr),
        key_seq_number: None,
    };

    let mut buf = [0u8; 32];
    let len = sec_hdr.serialize(&mut buf);
    assert_eq!(len, 13); // 1 + 4 + 8

    let (parsed, consumed) = ApsSecurityHeader::parse(&buf[..len]).unwrap();
    assert_eq!(consumed, 13);
    assert_eq!(parsed.frame_counter, 0xDEAD_BEEF);
    assert_eq!(parsed.source_address, Some(addr));
}

// ════════════════════════════════════════════════════════════════
// 8. Security key table
// ════════════════════════════════════════════════════════════════

#[test]
fn test_security_add_find_remove_key() {
    let mut sec = ApsSecurity::new();
    let partner = [0xAA; 8];
    let key = [0x11; 16];

    let entry = ApsLinkKeyEntry {
        partner_address: partner,
        key,
        key_type: ApsKeyType::TrustCenterLinkKey,
        outgoing_frame_counter: 0,
        incoming_frame_counter: 0,
    };
    assert!(sec.add_key(entry).is_ok());
    assert_eq!(sec.key_count(), 1);

    let found = sec
        .find_key(&partner, ApsKeyType::TrustCenterLinkKey)
        .unwrap();
    assert_eq!(found.key, key);

    assert!(sec.remove_key(&partner, ApsKeyType::TrustCenterLinkKey));
    assert_eq!(sec.key_count(), 0);
}

#[test]
fn test_security_frame_counter_replay_protection() {
    let mut sec = ApsSecurity::new();
    let partner = [0xBB; 8];

    let entry = ApsLinkKeyEntry {
        partner_address: partner,
        key: [0x22; 16],
        key_type: ApsKeyType::ApplicationLinkKey,
        outgoing_frame_counter: 0,
        incoming_frame_counter: 0,
    };
    sec.add_key(entry).unwrap();

    // Two-phase pattern: check (read-only) then commit after MIC verify
    // Counter 10 should be accepted (> 0)
    assert!(sec.check_frame_counter(&partner, ApsKeyType::ApplicationLinkKey, 10));
    // Commit after successful MIC verification
    sec.commit_frame_counter(&partner, ApsKeyType::ApplicationLinkKey, 10);
    // Counter 5 should be rejected (replay, 5 <= 10)
    assert!(!sec.check_frame_counter(&partner, ApsKeyType::ApplicationLinkKey, 5));
    // Counter 11 should be accepted
    assert!(sec.check_frame_counter(&partner, ApsKeyType::ApplicationLinkKey, 11));
    sec.commit_frame_counter(&partner, ApsKeyType::ApplicationLinkKey, 11);
    // Counter 11 again should be rejected (equal, not greater)
    assert!(!sec.check_frame_counter(&partner, ApsKeyType::ApplicationLinkKey, 11));
}

#[test]
fn test_security_outgoing_frame_counter() {
    let mut sec = ApsSecurity::new();
    let partner = [0xCC; 8];

    let entry = ApsLinkKeyEntry {
        partner_address: partner,
        key: [0x33; 16],
        key_type: ApsKeyType::TrustCenterLinkKey,
        outgoing_frame_counter: 0,
        incoming_frame_counter: 0,
    };
    sec.add_key(entry).unwrap();

    assert_eq!(
        sec.next_frame_counter(&partner, ApsKeyType::TrustCenterLinkKey),
        Some(0)
    );
    assert_eq!(
        sec.next_frame_counter(&partner, ApsKeyType::TrustCenterLinkKey),
        Some(1)
    );
    assert_eq!(
        sec.next_frame_counter(&partner, ApsKeyType::TrustCenterLinkKey),
        Some(2)
    );
}

#[test]
fn test_default_tc_link_key() {
    let sec = ApsSecurity::new();
    assert_eq!(*sec.default_tc_link_key(), DEFAULT_TC_LINK_KEY);
    // "ZigBeeAlliance09" in ASCII
    assert_eq!(sec.default_tc_link_key()[0], b'Z');
    assert_eq!(sec.default_tc_link_key()[15], b'9');
}

// ════════════════════════════════════════════════════════════════
// 9. APS constants and enums
// ════════════════════════════════════════════════════════════════

#[test]
fn test_well_known_constants() {
    assert_eq!(ZDO_ENDPOINT, 0x00);
    assert_eq!(MIN_APP_ENDPOINT, 0x01);
    assert_eq!(MAX_APP_ENDPOINT, 0xF0);
    assert_eq!(BROADCAST_ENDPOINT, 0xFF);
    assert_eq!(PROFILE_ZDP, 0x0000);
    assert_eq!(PROFILE_HOME_AUTOMATION, 0x0104);
}

#[test]
fn test_aps_frame_type_from_u8() {
    assert_eq!(ApsFrameType::from_u8(0), Some(ApsFrameType::Data));
    assert_eq!(ApsFrameType::from_u8(1), Some(ApsFrameType::Command));
    assert_eq!(ApsFrameType::from_u8(2), Some(ApsFrameType::Ack));
    assert_eq!(ApsFrameType::from_u8(3), Some(ApsFrameType::InterPan));
    assert_eq!(ApsFrameType::from_u8(4), None);
}

#[test]
fn test_parse_empty_buffer_returns_none() {
    assert!(ApsHeader::parse(&[]).is_none());
}
