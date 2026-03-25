//! Tests for the MockMac backend and MAC primitives.

use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::*;
use zigbee_mac::{AddressMode, MacDriver, TxOptions};
use zigbee_types::*;

fn make_mock() -> MockMac {
    MockMac::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11])
}

// ── MockMac Scan Tests ───────────────────────────────

#[tokio::test]
async fn test_mock_scan_empty() {
    let mut mac = make_mock();
    let result = mac
        .mlme_scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask::ALL_2_4GHZ,
            scan_duration: 3,
        })
        .await;

    // Mock returns NoBeacon when no beacons are configured
    assert!(result.is_err());
}

#[tokio::test]
async fn test_mock_scan_with_beacon() {
    let mut mac = make_mock();

    // Pre-configure a beacon response
    mac.add_beacon(PanDescriptor {
        coord_address: MacAddress::Short(PanId(0x1234), ShortAddress::COORDINATOR),
        channel: 15,
        lqi: 200,
        security_use: false,
        superframe_spec: SuperframeSpec {
            beacon_order: 15,
            superframe_order: 15,
            final_cap_slot: 0,
            battery_life_ext: false,
            pan_coordinator: true,
            association_permit: true,
        },
        zigbee_beacon: ZigbeeBeaconPayload {
            protocol_id: 0,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity: true,
            device_depth: 0,
            end_device_capacity: true,
            extended_pan_id: [0x11; 8],
            update_id: 0,
            tx_offset: [0xFF, 0xFF, 0xFF],
        },
    });

    let result = mac
        .mlme_scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask::ALL_2_4GHZ,
            scan_duration: 3,
        })
        .await
        .unwrap();

    assert_eq!(result.pan_descriptors.len(), 1);
    assert_eq!(result.pan_descriptors[0].channel, 15);
    assert_eq!(result.pan_descriptors[0].lqi, 200);
}

// ── MockMac Association Tests ────────────────────────

#[tokio::test]
async fn test_mock_associate_default() {
    let mut mac = make_mock();

    // Pre-configure a successful associate response
    mac.set_associate_response(MlmeAssociateConfirm {
        short_address: ShortAddress(0x0001),
        status: AssociationStatus::Success,
    });

    let result = mac
        .mlme_associate(MlmeAssociateRequest {
            channel: 15,
            coord_address: MacAddress::Short(PanId(0x1234), ShortAddress::COORDINATOR),
            capability_info: CapabilityInfo {
                device_type_ffd: false,
                mains_powered: false,
                rx_on_when_idle: false,
                security_capable: false,
                allocate_address: true,
            },
        })
        .await;

    assert!(result.is_ok());
    let confirm = result.unwrap();
    assert_eq!(confirm.status, AssociationStatus::Success);
    assert_ne!(confirm.short_address, ShortAddress(0xFFFF));
}

// ── MockMac Data TX/RX Tests ─────────────────────────

#[tokio::test]
async fn test_mock_data_tx() {
    let mut mac = make_mock();
    let payload = [0x01, 0x02, 0x03, 0x04];

    let result = mac
        .mcps_data(zigbee_mac::McpsDataRequest {
            src_addr_mode: AddressMode::Short,
            dst_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            payload: &payload,
            msdu_handle: 1,
            tx_options: TxOptions::default(),
        })
        .await;

    assert!(result.is_ok());

    // Check TX history
    let history = mac.tx_history();
    assert_eq!(history.len(), 1);
}

// ── MockMac Reset Test ───────────────────────────────

#[tokio::test]
async fn test_mock_reset() {
    let mut mac = make_mock();
    let result = mac.mlme_reset(true).await;
    assert!(result.is_ok());
}

// ── MockMac PIB Tests ────────────────────────────────

#[tokio::test]
async fn test_mock_pib_get_set() {
    use zigbee_mac::pib::*;

    let mut mac = make_mock();

    // Get extended address
    let result = mac.mlme_get(PibAttribute::MacExtendedAddress).await;
    assert!(result.is_ok());
    if let Ok(PibValue::ExtendedAddress(addr)) = result {
        assert_eq!(addr, [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11]);
    }

    // Set short address
    let result = mac
        .mlme_set(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x1234)),
        )
        .await;
    assert!(result.is_ok());
}

// ── Capabilities Test ────────────────────────────────

#[tokio::test]
async fn test_mock_capabilities() {
    let mac = make_mock();
    let caps = mac.capabilities();
    // Mock should report basic capabilities
    assert!(caps.coordinator || caps.router); // Mock reports basic capabilities
}

// ── CapabilityInfo Tests ─────────────────────────────

#[test]
fn test_capability_info_to_byte() {
    let cap = CapabilityInfo {
        device_type_ffd: true,
        mains_powered: true,
        rx_on_when_idle: true,
        security_capable: false,
        allocate_address: true,
    };
    let byte = cap.to_byte();
    assert_ne!(byte, 0); // Should have bits set
    assert_eq!(byte & 0x02, 0x02); // device_type_ffd
    assert_eq!(byte & 0x04, 0x04); // mains_powered
    assert_eq!(byte & 0x08, 0x08); // rx_on_when_idle
    assert_eq!(byte & 0x80, 0x80); // allocate_address
}
