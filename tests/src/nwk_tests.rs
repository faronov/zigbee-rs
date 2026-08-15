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
        incoming_cost: 1,
        outgoing_cost: 1,
        link_status_age: 0,
        depth: 0,
        permit_joining: true,
        security_capable: true,
        age: 0,
        end_device_timeout: zigbee_nwk::frames::ED_TIMEOUT_ENUM_DEFAULT,
        keepalive_remaining_secs: 0,
        keepalive_confirmed: false,
        // The `tests` crate always builds `zigbee-nwk` with `router`, so this
        // router-only field is unconditionally present here.
        parent_annce_pending: false,
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
        incoming_cost: 3,
        outgoing_cost: 3,
        link_status_age: 0,
        depth: 2,
        permit_joining: false,
        security_capable: false,
        age: 0,
        end_device_timeout: zigbee_nwk::frames::ED_TIMEOUT_ENUM_DEFAULT,
        keepalive_remaining_secs: 0,
        keepalive_confirmed: false,
        // The `tests` crate always builds `zigbee-nwk` with `router`, so this
        // router-only field is unconditionally present here.
        parent_annce_pending: false,
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

    // check_frame_counter is now check-only (no commit)
    assert!(sec.check_frame_counter(&source, 1)); // First frame — OK
    sec.commit_frame_counter(&source, 1); // Commit after successful MIC verify

    assert!(sec.check_frame_counter(&source, 2)); // Newer — OK
    sec.commit_frame_counter(&source, 2); // Commit

    assert!(!sec.check_frame_counter(&source, 2)); // Replay — reject
    assert!(!sec.check_frame_counter(&source, 1)); // Old frame — reject

    assert!(sec.check_frame_counter(&source, 3)); // Newer — OK
    sec.commit_frame_counter(&source, 3); // Commit
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
        remove_children: true,
        request: false,
        rejoin: true,
    };
    let byte = cmd.serialize();
    assert_eq!(byte & 0x20, 0x20); // Rejoin bit set
    assert_eq!(byte & 0x40, 0x00); // Request bit clear
    assert_eq!(byte & 0x80, 0x80); // Remove children bit set

    let parsed = LeaveCommand::parse(&[0xE0]).unwrap();
    assert!(parsed.remove_children);
    assert!(parsed.request);
    assert!(parsed.rejoin);
}

// ── R22 Rejoin Parent Selection (NLME path, §3.6.1.4.2) ──────────

const REJOIN_EPID: IeeeAddress = [0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44];
const FOREIGN_EPID: IeeeAddress = [0x99; 8];

/// Zigbee PRO beacon for a router that could parent a rejoining device.
///
/// `association_permit` is always `false`: rejoin must work into a closed
/// network (R22 §3.6.1.4.2).
fn rejoin_beacon(
    router: u16,
    lqi: u8,
    update_id: u8,
    depth: u8,
    extended_pan_id: IeeeAddress,
) -> zigbee_mac::primitives::PanDescriptor {
    rejoin_beacon_with_capacity(router, lqi, update_id, depth, extended_pan_id, true, true)
}

/// [`rejoin_beacon`] with explicit beacon capacity bits.
fn rejoin_beacon_with_capacity(
    router: u16,
    lqi: u8,
    update_id: u8,
    depth: u8,
    extended_pan_id: IeeeAddress,
    router_capacity: bool,
    end_device_capacity: bool,
) -> zigbee_mac::primitives::PanDescriptor {
    zigbee_mac::primitives::PanDescriptor {
        channel: 15,
        coord_address: MacAddress::Short(PanId(0x1A2B), ShortAddress(router)),
        superframe_spec: zigbee_mac::primitives::SuperframeSpec {
            beacon_order: 15,
            superframe_order: 15,
            final_cap_slot: 15,
            battery_life_ext: false,
            pan_coordinator: router == 0x0000,
            association_permit: false,
        },
        lqi,
        security_use: false,
        zigbee_beacon: zigbee_mac::primitives::ZigbeeBeaconPayload {
            protocol_id: 0,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity,
            device_depth: depth,
            end_device_capacity,
            extended_pan_id,
            tx_offset: [0xFF; 3],
            update_id,
        },
    }
}

/// Restore the NWK state of a device that was commissioned on `REJOIN_EPID`
/// at network update state `update_id`.
fn commissioned_device(update_id: u8, device_type: DeviceType) -> NwkLayer<MockMac> {
    let mut nwk = NwkLayer::new(make_mock_mac(), device_type);
    let nib = nwk.nib_mut();
    nib.extended_pan_id = REJOIN_EPID;
    nib.pan_id = PanId(0x1A2B);
    nib.network_address = ShortAddress(0x1234);
    nib.logical_channel = 15;
    // A commissioned device holds a *known-good* update state; setting the
    // raw field alone would leave it unknown and disable the staleness gate.
    nib.set_nwk_update_id(update_id);
    nib.parent_address = ShortAddress(0x0001);
    nwk
}

fn commissioned_end_device(update_id: u8) -> NwkLayer<MockMac> {
    commissioned_device(update_id, DeviceType::EndDevice)
}

async fn discover(
    nwk: &mut NwkLayer<MockMac>,
) -> heapless::Vec<zigbee_nwk::nlme::NetworkDescriptor, 16> {
    nwk.nlme_network_discovery(ChannelMask(1 << 15), 3)
        .await
        .expect("discovery")
}

#[tokio::test]
async fn rejoin_selection_retains_only_the_most_recent_update_id() {
    let mut nwk = commissioned_end_device(0xFF);
    let mac = nwk.mac_mut();
    // Stale beacon with a perfect link — must never be selected.
    mac.add_beacon(rejoin_beacon(0x0001, 250, 0xFE, 0, REJOIN_EPID));
    // Our own network at the update state we hold: not stale, but an older
    // update id than the freshest beacon below, so it is not suitable.
    mac.add_beacon(rejoin_beacon(0x0002, 250, 0xFF, 0, REJOIN_EPID));
    // Another network entirely.
    mac.add_beacon(rejoin_beacon(0x0003, 250, 0x00, 0, FOREIGN_EPID));
    // Update id wrapped forward — the most recent one discovered.
    mac.add_beacon(rejoin_beacon(0x0004, 130, 0x00, 4, REJOIN_EPID));
    mac.add_beacon(rejoin_beacon(0x0005, 160, 0x00, 2, REJOIN_EPID));
    // Most recent update id but an unusable link cost (LQI 100 -> cost 5).
    mac.add_beacon(rejoin_beacon(0x0006, 100, 0x00, 0, REJOIN_EPID));

    let mut networks = discover(&mut nwk).await;
    let suitable = nwk.select_rejoin_parents(&mut networks);

    // Only the wrapped-forward update id 0x00 survives, ordered by depth.
    assert_eq!(suitable, 2);
    assert_eq!(networks[0].router_address, ShortAddress(0x0005));
    assert_eq!(networks[1].router_address, ShortAddress(0x0004));
    for network in &networks[suitable..] {
        assert!(
            !matches!(network.router_address.0, 0x0004 | 0x0005),
            "suitable parent leaked into the rejected tail"
        );
    }
}

#[tokio::test]
async fn rejoin_selection_requires_capacity_for_the_device_type() {
    // End device: only a beacon advertising end-device capacity is usable.
    let mut nwk = commissioned_end_device(4);
    {
        let mac = nwk.mac_mut();
        mac.add_beacon(rejoin_beacon_with_capacity(
            0x0001,
            250,
            4,
            0,
            REJOIN_EPID,
            true,
            false,
        ));
        mac.add_beacon(rejoin_beacon_with_capacity(
            0x0002,
            160,
            4,
            3,
            REJOIN_EPID,
            false,
            true,
        ));
    }
    let mut networks = discover(&mut nwk).await;
    assert_eq!(nwk.select_rejoin_parents(&mut networks), 1);
    assert_eq!(networks[0].router_address, ShortAddress(0x0002));

    // Router: the capacity requirement flips to the router capacity bit.
    let mut nwk = commissioned_device(4, DeviceType::Router);
    {
        let mac = nwk.mac_mut();
        mac.add_beacon(rejoin_beacon_with_capacity(
            0x0001,
            250,
            4,
            0,
            REJOIN_EPID,
            true,
            false,
        ));
        mac.add_beacon(rejoin_beacon_with_capacity(
            0x0002,
            160,
            4,
            3,
            REJOIN_EPID,
            false,
            true,
        ));
    }
    let mut networks = discover(&mut nwk).await;
    assert_eq!(nwk.select_rejoin_parents(&mut networks), 1);
    assert_eq!(networks[0].router_address, ShortAddress(0x0001));
}

#[tokio::test]
async fn rejoin_selection_prefers_minimum_depth_over_link_cost() {
    let mut nwk = commissioned_end_device(6);
    {
        let mac = nwk.mac_mut();
        mac.add_beacon(rejoin_beacon(0x0001, 250, 6, 3, REJOIN_EPID)); // cost 1, depth 3
        mac.add_beacon(rejoin_beacon(0x0002, 130, 6, 1, REJOIN_EPID)); // cost 3, depth 1
        mac.add_beacon(rejoin_beacon(0x0003, 160, 6, 2, REJOIN_EPID)); // cost 2, depth 2
    }

    let mut networks = discover(&mut nwk).await;
    assert_eq!(nwk.select_rejoin_parents(&mut networks), 3);
    assert_eq!(
        [
            networks[0].router_address.0,
            networks[1].router_address.0,
            networks[2].router_address.0
        ],
        [0x0002, 0x0003, 0x0001]
    );
}

#[tokio::test]
async fn rejoin_is_allowed_into_a_closed_network() {
    let mut nwk = commissioned_end_device(2);
    nwk.mac_mut()
        .add_beacon(rejoin_beacon(0x0001, 250, 2, 1, REJOIN_EPID));

    let mut networks = discover(&mut nwk).await;
    assert!(
        !networks[0].permit_joining,
        "beacon must advertise joining as closed for this test to mean anything"
    );
    assert_eq!(nwk.select_rejoin_parents(&mut networks), 1);
}

#[tokio::test]
async fn rejoin_refuses_a_stale_candidate_without_transmitting() {
    let mut nwk = commissioned_end_device(0x00);
    nwk.mac_mut()
        .add_beacon(rejoin_beacon(0x0001, 250, 0xFF, 1, REJOIN_EPID));

    let networks = discover(&mut nwk).await;
    let status = nwk
        .nlme_join(&networks[0], zigbee_nwk::nlme::JoinMethod::Rejoin)
        .await;

    assert_eq!(status, Err(NwkStatus::InvalidRequest));
    assert!(
        nwk.mac().tx_history().is_empty(),
        "a stale-update-id parent must not receive a rejoin request"
    );
}

#[tokio::test]
async fn rejoin_refuses_an_unusable_link_cost_without_transmitting() {
    let mut nwk = commissioned_end_device(0x07);
    nwk.mac_mut()
        .add_beacon(rejoin_beacon(0x0001, 40, 0x07, 1, REJOIN_EPID));

    let networks = discover(&mut nwk).await;
    let status = nwk
        .nlme_join(&networks[0], zigbee_nwk::nlme::JoinMethod::Rejoin)
        .await;

    assert_eq!(status, Err(NwkStatus::InvalidRequest));
    assert!(nwk.mac().tx_history().is_empty());
}

#[tokio::test]
async fn rejoin_refuses_a_candidate_without_capacity_without_transmitting() {
    let mut nwk = commissioned_end_device(0x07);
    nwk.mac_mut().add_beacon(rejoin_beacon_with_capacity(
        0x0001,
        250,
        0x07,
        1,
        REJOIN_EPID,
        true,
        false,
    ));

    let networks = discover(&mut nwk).await;
    let status = nwk
        .nlme_join(&networks[0], zigbee_nwk::nlme::JoinMethod::Rejoin)
        .await;

    assert_eq!(status, Err(NwkStatus::InvalidRequest));
    assert!(nwk.mac().tx_history().is_empty());
}

#[tokio::test]
async fn rejoin_join_guard_accepts_a_base_eligible_but_not_freshest_candidate() {
    // The direct NLME-JOIN guard is per-candidate only: it cannot know which
    // update id was the most recent in a scan it never saw, so it must accept
    // a base-eligible candidate. Global "most recent" filtering is the
    // caller's policy, applied by `select_rejoin_parents`.
    let mut nwk = commissioned_end_device(4);
    {
        let mac = nwk.mac_mut();
        mac.add_beacon(rejoin_beacon(0x0001, 250, 4, 1, REJOIN_EPID));
        mac.add_beacon(rejoin_beacon(0x0002, 250, 5, 1, REJOIN_EPID));
    }

    let mut networks = discover(&mut nwk).await;
    // Selection keeps only the newest (update id 5).
    assert_eq!(nwk.select_rejoin_parents(&mut networks), 1);
    assert_eq!(networks[0].update_id, 5);

    // The older-but-not-stale candidate is still base eligible, so the direct
    // call transmits and fails on the missing Rejoin Response instead of being
    // rejected as invalid.
    let older = networks
        .iter()
        .find(|network| network.update_id == 4)
        .expect("older candidate retained for diagnostics")
        .clone();
    let status = nwk
        .nlme_join(&older, zigbee_nwk::nlme::JoinMethod::Rejoin)
        .await;
    assert_ne!(status, Err(NwkStatus::InvalidRequest));
    assert!(
        !nwk.mac().tx_history().is_empty(),
        "a base-eligible candidate must be attempted by the direct join path"
    );
}

/// A device whose stored record predates the `NwkUpdateId` item holds no
/// authoritative update state. It must not reject candidates as stale — with
/// a fabricated reference of `0`, every beacon in `0x81..=0xFF` would look
/// stale and the device could never get back onto its own network.
#[tokio::test]
async fn rejoin_selection_with_an_unknown_update_id_accepts_high_update_ids() {
    let mut nwk = commissioned_end_device(0);
    // Undo the "commissioned" update state: this device never learned one.
    nwk.nib_mut().clear_nwk_update_id();
    assert_eq!(nwk.nib().nwk_update_id(), None);

    {
        let mac = nwk.mac_mut();
        // Every one of these would be "stale" against a fabricated local 0.
        mac.add_beacon(rejoin_beacon(0x0001, 250, 0xFD, 2, REJOIN_EPID));
        mac.add_beacon(rejoin_beacon(0x0002, 250, 0xFE, 0, REJOIN_EPID));
        // Wraps past 0xFF: the most recent of the set.
        mac.add_beacon(rejoin_beacon(0x0003, 160, 0x02, 3, REJOIN_EPID));
        mac.add_beacon(rejoin_beacon(0x0004, 250, 0x02, 1, REJOIN_EPID));
        // Foreign network, newest-looking id: never a candidate.
        mac.add_beacon(rejoin_beacon(0x0005, 250, 0x03, 0, FOREIGN_EPID));
    }

    let mut networks = discover(&mut nwk).await;
    let suitable = nwk.select_rejoin_parents(&mut networks);

    // Selection still narrows to a single update id, chosen wrap-aware, and
    // then ranks the survivors by minimum depth.
    assert_eq!(suitable, 2);
    assert_eq!(networks[0].update_id, 0x02);
    assert_eq!(networks[0].router_address, ShortAddress(0x0004));
    assert_eq!(networks[1].router_address, ShortAddress(0x0003));
}

/// The direct per-candidate guard follows the same known-vs-unknown rule as
/// whole-scan selection: with no local update state it never refuses a
/// candidate for being "stale".
#[tokio::test]
async fn rejoin_join_guard_with_an_unknown_update_id_attempts_the_candidate() {
    let mut nwk = commissioned_end_device(0);
    nwk.nib_mut().clear_nwk_update_id();
    nwk.mac_mut()
        .add_beacon(rejoin_beacon(0x0001, 250, 0xFF, 1, REJOIN_EPID));

    let networks = discover(&mut nwk).await;
    let status = nwk
        .nlme_join(&networks[0], zigbee_nwk::nlme::JoinMethod::Rejoin)
        .await;

    assert_ne!(
        status,
        Err(NwkStatus::InvalidRequest),
        "an unknown local update state must not reject a candidate as stale"
    );
    assert!(
        !nwk.mac().tx_history().is_empty(),
        "the candidate must actually be attempted"
    );

    // With a known local state one update ahead, the same candidate is stale.
    let mut nwk = commissioned_end_device(0x00);
    nwk.mac_mut()
        .add_beacon(rejoin_beacon(0x0001, 250, 0xFF, 1, REJOIN_EPID));
    let networks = discover(&mut nwk).await;
    assert_eq!(
        nwk.nlme_join(&networks[0], zigbee_nwk::nlme::JoinMethod::Rejoin)
            .await,
        Err(NwkStatus::InvalidRequest)
    );
    assert!(nwk.mac().tx_history().is_empty());
}

/// A full leave drops the update state with the rest of the network identity,
/// so a later rejoin cannot filter candidates against a network we left.
#[tokio::test]
async fn a_full_leave_clears_the_update_id() {
    let mut nwk = commissioned_end_device(0x40);
    nwk.set_joined(true);
    assert_eq!(nwk.nib().nwk_update_id(), Some(0x40));

    nwk.nlme_leave(false).await.expect("leave");

    assert_eq!(nwk.nib().nwk_update_id(), None);
    assert!(!nwk.nib().update_id_valid);
}

#[tokio::test]
async fn orphan_recovery_ignores_stale_and_foreign_parents() {
    let mut nwk = commissioned_end_device(0x10);
    let mac = nwk.mac_mut();
    mac.add_beacon(rejoin_beacon(0x0001, 250, 0x0F, 0, REJOIN_EPID));
    mac.add_beacon(rejoin_beacon(0x0002, 250, 0x10, 0, FOREIGN_EPID));

    assert_eq!(nwk.nlme_orphan_recovery().await, Err(NwkStatus::NoNetworks));
    assert!(
        nwk.mac().tx_history().is_empty(),
        "orphan recovery must not rejoin through a stale or foreign parent"
    );
}
