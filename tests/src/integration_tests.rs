//! Tests for the Trust Center and coordinator/router modules.

use zigbee::coordinator::*;
use zigbee::router::*;
use zigbee::trust_center::*;
use zigbee_types::*;

// ── Trust Center Tests ───────────────────────────────

#[test]
fn test_default_tc_link_key() {
    // "ZigBeeAlliance09" in ASCII
    assert_eq!(DEFAULT_TC_LINK_KEY[0], 0x5A); // 'Z'
    assert_eq!(DEFAULT_TC_LINK_KEY[1], 0x69); // 'i'
    assert_eq!(DEFAULT_TC_LINK_KEY[2], 0x67); // 'g'
}

#[test]
fn test_tc_key_management() {
    let nwk_key = [0xAA; 16];
    let mut tc = TrustCenter::new(nwk_key);

    assert_eq!(tc.network_key(), &nwk_key);
    assert_eq!(tc.key_seq_number(), 0);

    // Rotate key
    let new_key = [0xBB; 16];
    tc.set_network_key(new_key);
    assert_eq!(tc.network_key(), &new_key);
    assert_eq!(tc.key_seq_number(), 1);
}

#[test]
fn test_tc_link_key_per_device() {
    let mut tc = TrustCenter::new([0; 16]);
    let dev1: IeeeAddress = [1; 8];
    let dev2: IeeeAddress = [2; 8];
    let key1 = [0x11; 16];

    // Default: returns well-known key
    assert_eq!(tc.link_key_for_device(&dev1), DEFAULT_TC_LINK_KEY);

    // Set device-specific key
    tc.set_link_key(dev1, key1, TcKeyType::InstallCode).unwrap();
    assert_eq!(tc.link_key_for_device(&dev1), key1);
    assert_eq!(tc.link_key_for_device(&dev2), DEFAULT_TC_LINK_KEY);
    assert_eq!(tc.device_count(), 1);
}

#[test]
fn test_tc_join_acceptance() {
    let mut tc = TrustCenter::new([0; 16]);
    let dev: IeeeAddress = [1; 8];

    // Without install code requirement, accept all
    assert!(tc.should_accept_join(&dev));

    // With install code requirement
    tc.set_require_install_codes(true);
    assert!(!tc.should_accept_join(&dev));

    // Provision install code key
    tc.set_link_key(dev, [0x42; 16], TcKeyType::InstallCode)
        .unwrap();
    assert!(tc.should_accept_join(&dev));
}

#[test]
fn test_tc_frame_counter() {
    let mut tc = TrustCenter::new([0; 16]);
    let dev: IeeeAddress = [1; 8];
    tc.set_link_key(dev, [0; 16], TcKeyType::DefaultGlobal)
        .unwrap();

    assert!(tc.update_frame_counter(&dev, 1));
    assert!(tc.update_frame_counter(&dev, 5));
    assert!(!tc.update_frame_counter(&dev, 3)); // Replay
    assert!(!tc.update_frame_counter(&dev, 5)); // Duplicate
    assert!(tc.update_frame_counter(&dev, 6));
}

// ── Coordinator Tests ────────────────────────────────

#[test]
fn test_coordinator_address_allocation() {
    let mut coord = Coordinator::new(CoordinatorConfig::default());
    let addr1 = coord.allocate_address();
    let addr2 = coord.allocate_address();
    assert_ne!(addr1, addr2);
    assert_ne!(addr1, ShortAddress(0x0000)); // Not coordinator addr
    assert_ne!(addr1, ShortAddress(0xFFFF)); // Not broadcast
}

#[test]
fn test_coordinator_child_capacity() {
    let config = CoordinatorConfig {
        max_children: 2,
        ..Default::default()
    };
    let mut coord = Coordinator::new(config);
    assert!(coord.can_accept_child());
    coord.allocate_address();
    coord.allocate_address();
    assert!(!coord.can_accept_child());
}

// ── Router Tests ─────────────────────────────────────

#[test]
fn test_router_child_management() {
    let mut router = Router::new(RouterConfig::default());
    let ieee: IeeeAddress = [1; 8];
    let short = ShortAddress(0x1234);

    router.add_child(ieee, short, false, false).unwrap();
    assert_eq!(router.child_count(), 1);
    assert!(router.is_child(short));
    assert!(router.find_child(short).is_some());

    router.remove_child(short);
    assert_eq!(router.child_count(), 0);
    assert!(!router.is_child(short));
}

#[test]
fn test_router_child_aging() {
    let mut router = Router::new(RouterConfig::default());
    let ieee: IeeeAddress = [1; 8];
    let short = ShortAddress(0x0001);

    // Add sleepy end device with 60s timeout
    router.add_child(ieee, short, false, false).unwrap();

    // Age past timeout
    router.age_children(301); // > 300s default timeout
    assert_eq!(router.child_count(), 0); // Timed out and removed
}

#[test]
fn test_router_child_activity_resets_age() {
    let mut router = Router::new(RouterConfig::default());
    let ieee: IeeeAddress = [1; 8];
    let short = ShortAddress(0x0001);

    router.add_child(ieee, short, false, false).unwrap();
    router.age_children(200);

    // Activity resets age
    router.child_activity(short);
    let child = router.find_child(short).unwrap();
    assert_eq!(child.age, 0);
}

#[test]
fn test_router_capacity_limit() {
    let config = RouterConfig {
        max_children: 2,
        ..Default::default()
    };
    let mut router = Router::new(config);

    router
        .add_child([1; 8], ShortAddress(1), false, true)
        .unwrap();
    router
        .add_child([2; 8], ShortAddress(2), false, true)
        .unwrap();
    assert!(!router.can_accept_child());
    assert!(router
        .add_child([3; 8], ShortAddress(3), false, true)
        .is_err());
}
