//! Tests for the BDB (Base Device Behavior) commissioning crate.

use zigbee_aps::ApsLayer;
use zigbee_bdb::attributes::{
    BdbAttributes, BdbCommissioningStatus, NodeJoinLinkKeyType, TcLinkKeyExchangeMethod,
    BDB_MIN_COMMISSIONING_TIME, BDB_PRIMARY_CHANNEL_SET, BDB_SECONDARY_CHANNEL_SET,
};
use zigbee_bdb::state_machine::{BdbState, CommissioningMode};
use zigbee_bdb::{BdbLayer, BdbStatus};
use zigbee_mac::mock::MockMac;
use zigbee_nwk::{DeviceType, NwkLayer};
use zigbee_types::ChannelMask;
use zigbee_zdo::ZdoLayer;

fn make_bdb(device_type: DeviceType) -> BdbLayer<MockMac> {
    let mac = MockMac::new([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]);
    let nwk = NwkLayer::new(mac, device_type);
    let aps = ApsLayer::new(nwk);
    let zdo = ZdoLayer::new(aps);
    BdbLayer::new(zdo)
}

// ── 1. BDB layer creation ───────────────────────────────────

#[test]
fn bdb_layer_initial_state_is_idle() {
    let bdb = make_bdb(DeviceType::EndDevice);
    assert_eq!(*bdb.state(), BdbState::Idle);
}

#[test]
fn bdb_layer_not_on_network_by_default() {
    let bdb = make_bdb(DeviceType::Router);
    assert!(!bdb.is_on_network());
}

// ── 2. Commissioning modes ──────────────────────────────────

#[test]
fn commissioning_mode_individual_flags() {
    assert_eq!(CommissioningMode::STEERING.0, 1 << 0);
    assert_eq!(CommissioningMode::FORMATION.0, 1 << 1);
    assert_eq!(CommissioningMode::FINDING_BINDING.0, 1 << 2);
    assert_eq!(CommissioningMode::TOUCHLINK.0, 1 << 3);
    assert!(CommissioningMode::TOUCHLINK.contains(CommissioningMode::TOUCHLINK));
    assert!(!CommissioningMode::TOUCHLINK.contains(CommissioningMode::STEERING));
    assert!(CommissioningMode::STEERING.contains(CommissioningMode::STEERING));
    assert!(CommissioningMode::FORMATION.contains(CommissioningMode::FORMATION));
    assert!(CommissioningMode::FINDING_BINDING.contains(CommissioningMode::FINDING_BINDING));
}

#[test]
fn commissioning_mode_all_contains_every_method() {
    let all = CommissioningMode::ALL;
    assert!(all.contains(CommissioningMode::TOUCHLINK));
    assert!(all.contains(CommissioningMode::STEERING));
    assert!(all.contains(CommissioningMode::FORMATION));
    assert!(all.contains(CommissioningMode::FINDING_BINDING));
}

#[test]
fn commissioning_mode_or_combines_flags() {
    let combined = CommissioningMode::STEERING.or(CommissioningMode::FORMATION);
    assert!(combined.contains(CommissioningMode::STEERING));
    assert!(combined.contains(CommissioningMode::FORMATION));
    assert!(!combined.contains(CommissioningMode::TOUCHLINK));
}

#[test]
fn commissioning_mode_empty() {
    let empty = CommissioningMode::empty();
    assert!(empty.is_empty());
    assert!(!empty.contains(CommissioningMode::STEERING));
}

// ── 3. BDB attributes / config ─────────────────────────────

#[test]
fn bdb_attributes_defaults() {
    let attrs = BdbAttributes::default();

    assert_eq!(attrs.commissioning_group_id, 0xFFFF);
    assert_eq!(attrs.commissioning_mode, CommissioningMode::STEERING);
    assert_eq!(attrs.commissioning_status, BdbCommissioningStatus::Success);
    assert!(!attrs.node_is_on_a_network);
    assert_eq!(
        attrs.node_join_link_key_type,
        NodeJoinLinkKeyType::DefaultGlobalTrustCenterLinkKey
    );
    assert!(!attrs.join_uses_install_code_key);
    assert_eq!(attrs.scan_duration, 3);
    assert_eq!(attrs.tc_link_key_exchange_attempts, 0);
    assert_eq!(attrs.tc_link_key_exchange_attempts_max, 3);
    assert_eq!(
        attrs.tc_link_key_exchange_method,
        TcLinkKeyExchangeMethod::ApsRequestKey
    );
    assert_eq!(attrs.trust_center_node_join_timeout, 15);
    assert!(attrs.trust_center_require_key_exchange);
    assert_eq!(attrs.steering_attempts_remaining, 5);
}

#[test]
fn node_join_link_key_types_match_bdb_table_6() {
    assert_eq!(
        NodeJoinLinkKeyType::DefaultGlobalTrustCenterLinkKey as u8,
        0x00
    );
    assert_eq!(
        NodeJoinLinkKeyType::DistributedSecurityGlobalLinkKey as u8,
        0x01
    );
    assert_eq!(
        NodeJoinLinkKeyType::InstallCodeDerivedPreconfiguredLinkKey as u8,
        0x02
    );
    assert_eq!(
        NodeJoinLinkKeyType::TouchlinkPreconfiguredLinkKey as u8,
        0x03
    );
}

#[test]
fn commissioning_status_contains_every_bdb_table_4_value() {
    assert_eq!(BdbCommissioningStatus::Success as u8, 0x00);
    assert_eq!(BdbCommissioningStatus::InProgress as u8, 0x01);
    assert_eq!(BdbCommissioningStatus::NotAaCapable as u8, 0x02);
    assert_eq!(BdbCommissioningStatus::NoNetwork as u8, 0x03);
    assert_eq!(BdbCommissioningStatus::TargetFailure as u8, 0x04);
    assert_eq!(BdbCommissioningStatus::FormationFailure as u8, 0x05);
    assert_eq!(BdbCommissioningStatus::NoIdentifyQueryResponse as u8, 0x06);
    assert_eq!(BdbCommissioningStatus::BindingTableFull as u8, 0x07);
    assert_eq!(BdbCommissioningStatus::NoScanResponse as u8, 0x08);
    assert_eq!(BdbCommissioningStatus::NotPermitted as u8, 0x09);
    assert_eq!(BdbCommissioningStatus::TcLinkKeyExchangeFailure as u8, 0x0A);
    assert_eq!(BdbCommissioningStatus::NotOnANetwork as u8, 0x0B);
    assert_eq!(BdbCommissioningStatus::OnANetwork as u8, 0x0C);
}

#[test]
fn initialization_advertises_only_implemented_default_capabilities() {
    let mut end_device = make_bdb(DeviceType::EndDevice);
    end_device.initialize().unwrap();
    assert_eq!(
        end_device.attributes().node_commissioning_capability,
        CommissioningMode::STEERING
    );

    let mut coordinator = make_bdb(DeviceType::Coordinator);
    coordinator.initialize().unwrap();
    assert_eq!(
        coordinator.attributes().node_commissioning_capability,
        CommissioningMode::STEERING.or(CommissioningMode::FORMATION)
    );
}

#[test]
fn bdb_attributes_mutable_via_layer() {
    let mut bdb = make_bdb(DeviceType::EndDevice);
    bdb.attributes_mut().commissioning_mode = CommissioningMode::ALL;
    assert_eq!(bdb.attributes().commissioning_mode, CommissioningMode::ALL);

    bdb.attributes_mut().steering_attempts_remaining = 10;
    assert_eq!(bdb.attributes().steering_attempts_remaining, 10);
}

// ── 4. Channel masks ────────────────────────────────────────

#[test]
fn bdb_primary_channel_set_correct() {
    let expected = (1u32 << 11) | (1u32 << 15) | (1u32 << 20) | (1u32 << 25);
    assert_eq!(BDB_PRIMARY_CHANNEL_SET.0, expected);

    let attrs = BdbAttributes::default();
    assert_eq!(attrs.primary_channel_set, BDB_PRIMARY_CHANNEL_SET);
}

#[test]
fn bdb_secondary_channel_set_excludes_primary() {
    // Secondary = all 2.4 GHz minus primary
    assert_eq!(
        BDB_SECONDARY_CHANNEL_SET.0,
        ChannelMask::ALL_2_4GHZ.0 & !BDB_PRIMARY_CHANNEL_SET.0
    );
    // No overlap between primary and secondary
    assert_eq!(BDB_PRIMARY_CHANNEL_SET.0 & BDB_SECONDARY_CHANNEL_SET.0, 0);
}

// ── 5. BDB status codes ────────────────────────────────────

#[test]
fn bdb_status_discriminants() {
    assert_eq!(BdbStatus::Success as u8, 0x00);
    assert_eq!(BdbStatus::InProgress as u8, 0x01);
    assert_eq!(BdbStatus::NotOnNetwork as u8, 0x02);
    assert_eq!(BdbStatus::NoScanResponse as u8, 0x04);
    assert_eq!(BdbStatus::FormationFailure as u8, 0x05);
    assert_eq!(BdbStatus::TouchlinkFailure as u8, 0x09);
    assert_eq!(BdbStatus::Timeout as u8, 0x0B);
    assert_eq!(BdbStatus::TrustCenterLinkKeyExchangeFailure as u8, 0x0C);
}

// ── 6. Commissioning time constant ─────────────────────────

#[test]
fn bdb_min_commissioning_time_is_180s() {
    assert_eq!(BDB_MIN_COMMISSIONING_TIME, 180);
}

// ── 7. R22 rejoin parent selection (BDB path, §3.6.1.4.2) ──

const REJOIN_EPID: zigbee_types::IeeeAddress = [0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44];
const FOREIGN_EPID: zigbee_types::IeeeAddress = [0x99; 8];
const REJOIN_PAN: zigbee_types::PanId = zigbee_types::PanId(0x1A2B);

/// Beacon from a potential rejoin parent. Joining is always closed, because
/// R22 §3.6.1.4.2 allows rejoin into a closed network.
fn rejoin_beacon(
    router: u16,
    lqi: u8,
    update_id: u8,
    depth: u8,
    extended_pan_id: zigbee_types::IeeeAddress,
) -> zigbee_mac::primitives::PanDescriptor {
    rejoin_beacon_with_capacity(router, lqi, update_id, depth, extended_pan_id, true, true)
}

/// [`rejoin_beacon`] with explicit beacon capacity bits.
fn rejoin_beacon_with_capacity(
    router: u16,
    lqi: u8,
    update_id: u8,
    depth: u8,
    extended_pan_id: zigbee_types::IeeeAddress,
    router_capacity: bool,
    end_device_capacity: bool,
) -> zigbee_mac::primitives::PanDescriptor {
    zigbee_mac::primitives::PanDescriptor {
        channel: 15,
        coord_address: zigbee_types::MacAddress::Short(
            REJOIN_PAN,
            zigbee_types::ShortAddress(router),
        ),
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

/// BDB layer restored onto `REJOIN_EPID` at network update state `update_id`,
/// with `previous_parent` as the last known parent.
fn commissioned_bdb(
    device_type: DeviceType,
    update_id: u8,
    previous_parent: u16,
) -> BdbLayer<MockMac> {
    let mut bdb = make_bdb(device_type);
    bdb.attributes_mut().node_is_on_a_network = true;
    let nib = bdb.zdo_mut().nwk_mut().nib_mut();
    nib.extended_pan_id = REJOIN_EPID;
    nib.pan_id = REJOIN_PAN;
    nib.network_address = zigbee_types::ShortAddress(0x4321);
    nib.logical_channel = 15;
    // A commissioned device holds a *known-good* update state; setting the
    // raw field alone would leave it unknown and disable the staleness gate.
    nib.set_nwk_update_id(update_id);
    nib.parent_address = zigbee_types::ShortAddress(previous_parent);
    bdb
}

/// The parents actually addressed by transmitted rejoin attempts, in order.
fn rejoin_targets(bdb: &BdbLayer<MockMac>) -> std::vec::Vec<u16> {
    bdb.zdo()
        .nwk()
        .mac()
        .tx_history()
        .iter()
        .map(|record| match record.dst {
            zigbee_types::MacAddress::Short(_, addr) => addr.0,
            zigbee_types::MacAddress::Extended(_, _) => 0xFFFF,
        })
        .collect()
}

#[tokio::test]
async fn bdb_rejoin_attempts_only_the_most_recent_update_id_by_depth() {
    let mut bdb = commissioned_bdb(DeviceType::EndDevice, 0xFF, 0x0001);
    {
        let mac = bdb.zdo_mut().nwk_mut().mac_mut();
        // Stale network update state with the strongest link — never tried.
        mac.add_beacon(rejoin_beacon(0x0001, 250, 0xFE, 0, REJOIN_EPID));
        // Not stale, but an older update id than the freshest beacon below,
        // so R22 drops it from the suitable set despite depth 0.
        mac.add_beacon(rejoin_beacon(0x0002, 250, 0xFF, 0, REJOIN_EPID));
        // Foreign network — never tried.
        mac.add_beacon(rejoin_beacon(0x0003, 250, 0x00, 0, FOREIGN_EPID));
        // Update id wrapped forward: the most recent state discovered.
        mac.add_beacon(rejoin_beacon(0x0004, 250, 0x00, 4, REJOIN_EPID));
        mac.add_beacon(rejoin_beacon(0x0005, 130, 0x00, 2, REJOIN_EPID));
        // Most recent update id but unusable link cost (LQI 100 -> cost 5).
        mac.add_beacon(rejoin_beacon(0x0006, 100, 0x00, 0, REJOIN_EPID));
    }

    // No parent answers the Rejoin Request in the mock, so every suitable
    // candidate is tried in order and the procedure ultimately fails.
    assert_eq!(
        bdb.rejoin_previous_network().await,
        Err(BdbStatus::SteeringFailure)
    );
    // Minimum depth first: 0x0005 (depth 2) before 0x0004 (depth 4), even
    // though 0x0004 has the better link.
    assert_eq!(rejoin_targets(&bdb), std::vec![0x0005, 0x0004]);
}

#[tokio::test]
async fn bdb_rejoin_skips_parents_without_capacity_for_our_device_type() {
    let mut bdb = commissioned_bdb(DeviceType::EndDevice, 4, 0x0001);
    {
        let mac = bdb.zdo_mut().nwk_mut().mac_mut();
        // Shallowest, best link, but no end-device capacity.
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

    assert_eq!(
        bdb.rejoin_previous_network().await,
        Err(BdbStatus::SteeringFailure)
    );
    assert_eq!(rejoin_targets(&bdb), std::vec![0x0002]);
}

#[tokio::test]
async fn bdb_rejoin_breaks_equal_depth_ties_deterministically() {
    let mut bdb = commissioned_bdb(DeviceType::EndDevice, 4, 0x0002);
    {
        let mac = bdb.zdo_mut().nwk_mut().mac_mut();
        mac.add_beacon(rejoin_beacon(0x0001, 210, 4, 1, REJOIN_EPID));
        mac.add_beacon(rejoin_beacon(0x0002, 210, 4, 1, REJOIN_EPID));
        mac.add_beacon(rejoin_beacon(0x0003, 210, 4, 1, REJOIN_EPID));
    }

    assert_eq!(
        bdb.rejoin_previous_network().await,
        Err(BdbStatus::SteeringFailure)
    );
    // All three tie on update id, depth and link cost, so the implementation
    // tie-breaks apply: previous parent first, then discovery order.
    assert_eq!(rejoin_targets(&bdb), std::vec![0x0002, 0x0001, 0x0003]);
}

#[tokio::test]
async fn bdb_rejoin_orders_equal_depth_candidates_by_link_cost() {
    let mut bdb = commissioned_bdb(DeviceType::EndDevice, 4, 0xFFFF);
    {
        let mac = bdb.zdo_mut().nwk_mut().mac_mut();
        mac.add_beacon(rejoin_beacon(0x0001, 130, 4, 1, REJOIN_EPID)); // cost 3
        mac.add_beacon(rejoin_beacon(0x0002, 250, 4, 1, REJOIN_EPID)); // cost 1
        mac.add_beacon(rejoin_beacon(0x0003, 160, 4, 1, REJOIN_EPID)); // cost 2
    }

    assert_eq!(
        bdb.rejoin_previous_network().await,
        Err(BdbStatus::SteeringFailure)
    );
    assert_eq!(rejoin_targets(&bdb), std::vec![0x0002, 0x0003, 0x0001]);
}

#[tokio::test]
async fn bdb_rejoin_works_on_a_closed_network() {
    let mut bdb = commissioned_bdb(DeviceType::EndDevice, 4, 0x0001);
    {
        let mac = bdb.zdo_mut().nwk_mut().mac_mut();
        // association_permit is false on every beacon built here.
        mac.add_beacon(rejoin_beacon(0x0001, 250, 4, 1, REJOIN_EPID));
    }

    assert_eq!(
        bdb.rejoin_previous_network().await,
        Err(BdbStatus::SteeringFailure)
    );
    assert_eq!(
        rejoin_targets(&bdb),
        std::vec![0x0001],
        "a closed network must still be rejoined"
    );
}

#[tokio::test]
async fn bdb_rejoin_reports_failure_when_no_candidate_is_suitable() {
    let mut bdb = commissioned_bdb(DeviceType::EndDevice, 0x10, 0x0001);
    {
        let mac = bdb.zdo_mut().nwk_mut().mac_mut();
        // Stale, foreign, unusable-link and no-capacity candidates only.
        mac.add_beacon(rejoin_beacon(0x0001, 250, 0x0F, 0, REJOIN_EPID));
        mac.add_beacon(rejoin_beacon(0x0002, 250, 0x10, 0, FOREIGN_EPID));
        mac.add_beacon(rejoin_beacon(0x0003, 30, 0x10, 0, REJOIN_EPID));
        mac.add_beacon(rejoin_beacon_with_capacity(
            0x0004,
            250,
            0x10,
            0,
            REJOIN_EPID,
            true,
            false,
        ));
    }

    assert_eq!(
        bdb.rejoin_previous_network().await,
        Err(BdbStatus::SteeringFailure)
    );
    assert!(
        rejoin_targets(&bdb).is_empty(),
        "no rejoin request may be sent when every candidate is rejected"
    );
    assert_eq!(*bdb.state(), BdbState::Idle);
}
