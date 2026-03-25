//! Tests for the BDB (Base Device Behavior) commissioning crate.

use zigbee_aps::ApsLayer;
use zigbee_bdb::attributes::{
    BdbAttributes, BdbCommissioningStatus, NodeJoinLinkKeyType, BDB_MIN_COMMISSIONING_TIME,
    BDB_PRIMARY_CHANNEL_SET, BDB_SECONDARY_CHANNEL_SET,
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
    assert_eq!(attrs.trust_center_node_join_timeout, 10);
    assert!(attrs.trust_center_require_key_exchange);
    assert_eq!(attrs.steering_attempts_remaining, 5);
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
}

// ── 6. Commissioning time constant ─────────────────────────

#[test]
fn bdb_min_commissioning_time_is_180s() {
    assert_eq!(BDB_MIN_COMMISSIONING_TIME, 180);
}
