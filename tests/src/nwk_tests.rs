//! Integration tests using MockMac backend.
//!
//! These tests verify the complete stack without hardware:
//! - NWK network discovery, join, leave
//! - NWK frame construction and parsing
//! - Routing table operations
//! - Security header construction
//! - Neighbor table management

use zigbee_mac::mock::MockMac;
use zigbee_nwk::frames::*;
use zigbee_nwk::neighbor::*;
use zigbee_nwk::nib::*;
use zigbee_nwk::routing::*;
use zigbee_nwk::security::*;
use zigbee_nwk::*;
use zigbee_types::*;

fn make_mock_mac() -> MockMac {
    MockMac::new([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
}

// ── NWK Frame Tests ──────────────────────────────────

#[test]
fn test_nwk_frame_control_data() {
    let fc = NwkFrameControl {
        frame_type: NwkFrameType::Data as u8,
        protocol_version: 0x02,
        discover_route: 0,
        multicast: false,
        security: false,
        source_route: false,
        dst_ieee_present: false,
        src_ieee_present: false,
        end_device_initiator: false,
    };
    let word = fc.serialize();
    let parsed = NwkFrameControl::parse(word);
    assert_eq!(parsed.frame_type, NwkFrameType::Data as u8);
    assert_eq!(parsed.protocol_version, 0x02);
    assert!(!parsed.security);
    assert!(!parsed.multicast);
}

#[test]
fn test_nwk_frame_control_command_with_security() {
    let fc = NwkFrameControl {
        frame_type: NwkFrameType::Command as u8,
        protocol_version: 0x02,
        discover_route: 1,
        multicast: false,
        security: true,
        source_route: false,
        dst_ieee_present: false,
        src_ieee_present: true,
        end_device_initiator: false,
    };
    let word = fc.serialize();
    let parsed = NwkFrameControl::parse(word);
    assert_eq!(parsed.frame_type, NwkFrameType::Command as u8);
    assert!(parsed.security);
    assert!(parsed.src_ieee_present);
    assert!(!parsed.dst_ieee_present);
    assert_eq!(parsed.discover_route, 1);
}

#[test]
fn test_nwk_header_serialize_parse_roundtrip() {
    let header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            discover_route: 0,
            multicast: false,
            security: false,
            source_route: false,
            dst_ieee_present: false,
            src_ieee_present: false,
            end_device_initiator: false,
        },
        dst_addr: ShortAddress(0x1234),
        src_addr: ShortAddress(0x5678),
        radius: 30,
        seq_number: 42,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };

    let mut buf = [0u8; 64];
    let len = header.serialize(&mut buf);
    assert_eq!(len, 8); // Minimum NWK header: FC(2) + dst(2) + src(2) + radius(1) + seq(1)

    let (parsed, consumed) = NwkHeader::parse(&buf[..len]).unwrap();
    assert_eq!(consumed, 8);
    assert_eq!(parsed.dst_addr, ShortAddress(0x1234));
    assert_eq!(parsed.src_addr, ShortAddress(0x5678));
    assert_eq!(parsed.radius, 30);
    assert_eq!(parsed.seq_number, 42);
}

#[test]
fn test_nwk_header_with_ieee_addresses() {
    let src_ieee: IeeeAddress = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    let header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            discover_route: 0,
            multicast: false,
            security: false,
            source_route: false,
            dst_ieee_present: false,
            src_ieee_present: true,
            end_device_initiator: false,
        },
        dst_addr: ShortAddress(0x0000),
        src_addr: ShortAddress(0x1234),
        radius: 1,
        seq_number: 1,
        dst_ieee: None,
        src_ieee: Some(src_ieee),
        multicast_control: None,
        source_route: None,
    };

    let mut buf = [0u8; 64];
    let len = header.serialize(&mut buf);
    assert_eq!(len, 16); // 8 base + 8 src IEEE

    let (parsed, _) = NwkHeader::parse(&buf[..len]).unwrap();
    assert_eq!(parsed.src_ieee, Some(src_ieee));
    assert_eq!(parsed.dst_ieee, None);
}

// ── Neighbor Table Tests ─────────────────────────────

#[test]
fn test_neighbor_table_add_and_find() {
    let mut table = NeighborTable::new();
    let entry = NeighborEntry {
        ieee_address: [1, 2, 3, 4, 5, 6, 7, 8],
        network_address: ShortAddress(0x1234),
        device_type: NeighborDeviceType::Router,
        rx_on_when_idle: true,
        relationship: Relationship::Parent,
        lqi: 200,
        outgoing_cost: 1,
        depth: 0,
        permit_joining: true,
        age: 0,
        extended_pan_id: [0; 8],
        active: true,
    };
    assert!(table.add_or_update(entry).is_ok());
    assert_eq!(table.len(), 1);

    let found = table.find_by_short(ShortAddress(0x1234));
    assert!(found.is_some());
    assert_eq!(found.unwrap().lqi, 200);
}

#[test]
fn test_neighbor_table_aging() {
    let mut table = NeighborTable::new();
    let entry = NeighborEntry {
        ieee_address: [1; 8],
        network_address: ShortAddress(0x0001),
        device_type: NeighborDeviceType::EndDevice,
        rx_on_when_idle: false,
        relationship: Relationship::Child,
        lqi: 100,
        outgoing_cost: 3,
        depth: 2,
        permit_joining: false,
        age: 0,
        extended_pan_id: [0; 8],
        active: true,
    };
    table.add_or_update(entry).unwrap();
    table.age_tick();
    let found = table.find_by_short(ShortAddress(0x0001)).unwrap();
    assert_eq!(found.age, 1);
}

// ── Routing Table Tests ──────────────────────────────

#[test]
fn test_routing_table_add_and_lookup() {
    let mut rt = RoutingTable::new();
    rt.update_route(ShortAddress(0x1234), ShortAddress(0x0001), 3)
        .unwrap();

    assert_eq!(
        rt.next_hop(ShortAddress(0x1234)),
        Some(ShortAddress(0x0001))
    );
    assert_eq!(rt.next_hop(ShortAddress(0x9999)), None);
    assert_eq!(rt.len(), 1);
}

#[test]
fn test_routing_table_update_existing() {
    let mut rt = RoutingTable::new();
    rt.update_route(ShortAddress(0x1234), ShortAddress(0x0001), 5)
        .unwrap();
    rt.update_route(ShortAddress(0x1234), ShortAddress(0x0002), 3)
        .unwrap();

    // Should have updated next_hop
    assert_eq!(
        rt.next_hop(ShortAddress(0x1234)),
        Some(ShortAddress(0x0002))
    );
    assert_eq!(rt.len(), 1); // Still one entry
}

#[test]
fn test_routing_table_remove() {
    let mut rt = RoutingTable::new();
    rt.update_route(ShortAddress(0x1234), ShortAddress(0x0001), 3)
        .unwrap();
    rt.remove(ShortAddress(0x1234));
    assert_eq!(rt.next_hop(ShortAddress(0x1234)), None);
    assert!(rt.is_empty());
}

// ── Security Tests ───────────────────────────────────

#[test]
fn test_security_header_parse_serialize() {
    let hdr = NwkSecurityHeader {
        security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
        frame_counter: 0x12345678,
        source_address: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        key_seq_number: 1,
    };

    let mut buf = [0u8; 32];
    let len = hdr.serialize(&mut buf);
    assert_eq!(len, 14);

    let (parsed, consumed) = NwkSecurityHeader::parse(&buf[..len]).unwrap();
    assert_eq!(consumed, 14);
    assert_eq!(parsed.frame_counter, 0x12345678);
    assert_eq!(parsed.source_address, hdr.source_address);
    assert_eq!(parsed.key_seq_number, 1);
}

#[test]
fn test_nwk_security_key_management() {
    let mut sec = NwkSecurity::new();

    // No key initially
    assert!(sec.active_key().is_none());

    // Set a key
    let key = [0xAA; 16];
    sec.set_network_key(key, 0);
    assert!(sec.active_key().is_some());
    assert_eq!(sec.active_key().unwrap().seq_number, 0);

    // Set another key — old becomes previous
    let key2 = [0xBB; 16];
    sec.set_network_key(key2, 1);
    assert_eq!(sec.active_key().unwrap().seq_number, 1);
    assert!(sec.key_by_seq(0).is_some()); // Previous key still accessible
}

#[test]
fn test_frame_counter_replay_protection() {
    let mut sec = NwkSecurity::new();
    let source = [1u8; 8];

    assert!(sec.check_frame_counter(&source, 1)); // First frame
    assert!(sec.check_frame_counter(&source, 2)); // Newer
    assert!(!sec.check_frame_counter(&source, 2)); // Replay
    assert!(!sec.check_frame_counter(&source, 1)); // Old frame
    assert!(sec.check_frame_counter(&source, 3)); // Newer again
}

// ── NIB Tests ────────────────────────────────────────

#[test]
fn test_nib_defaults() {
    let nib = Nib::new();
    assert_eq!(nib.network_address, ShortAddress(0xFFFF));
    assert_eq!(nib.pan_id, PanId(0xFFFF));
    assert_eq!(nib.depth, 0);
    assert_eq!(nib.max_depth, 15);
    assert_eq!(nib.max_routers, 5);
}

#[test]
fn test_nib_sequence_number() {
    let mut nib = Nib::new();
    let s1 = nib.next_seq();
    let s2 = nib.next_seq();
    assert_eq!(s2, s1.wrapping_add(1));
}

// ── NWK Layer Creation ───────────────────────────────

#[test]
fn test_nwk_layer_creation() {
    let mac = make_mock_mac();
    let nwk = NwkLayer::new(mac, DeviceType::EndDevice);
    assert!(!nwk.is_joined());
    assert_eq!(nwk.device_type(), DeviceType::EndDevice);
}

// ── Leave Command Tests ──────────────────────────────

#[test]
fn test_leave_command_serialize() {
    let cmd = LeaveCommand {
        remove_children: false,
        rejoin: true,
    };
    let byte = cmd.serialize();
    assert_eq!(byte & 0x20, 0x20); // Rejoin bit set
    assert_eq!(byte & 0x40, 0x00); // Remove children bit clear
}
