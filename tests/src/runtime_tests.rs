//! Tests for the runtime: builder API, NV storage, power management, templates.

use zigbee_mac::mock::MockMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::nv_storage::*;
use zigbee_runtime::power::*;
use zigbee_runtime::templates;
use zigbee_runtime::*;
use zigbee_types::*;

fn make_mock() -> MockMac {
    MockMac::new([1, 2, 3, 4, 5, 6, 7, 8])
}

// ── Builder Tests ────────────────────────────────────

#[test]
fn test_device_builder_basic() {
    let device = ZigbeeDevice::builder(make_mock())
        .device_type(DeviceType::EndDevice)
        .manufacturer("TestCo")
        .model("TestSensor")
        .build();

    assert_eq!(device.config.device_type, DeviceType::EndDevice);
    assert_eq!(device.config.manufacturer_name, "TestCo");
    assert_eq!(device.config.model_identifier, "TestSensor");
}

#[test]
fn test_device_builder_with_endpoints() {
    let device = ZigbeeDevice::builder(make_mock())
        .endpoint(1, 0x0104, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0402) // Temperature
        })
        .endpoint(2, 0x0104, 0x0301, |ep| {
            ep.cluster_server(0x0000).cluster_server(0x0405) // Humidity
        })
        .build();

    assert_eq!(device.config.endpoints.len(), 2);
    assert_eq!(device.config.endpoints[0].endpoint, 1);
    assert_eq!(device.config.endpoints[0].server_clusters.len(), 2);
    assert_eq!(device.config.endpoints[1].endpoint, 2);
}

#[test]
fn test_device_builder_channel_mask() {
    let mask = ChannelMask(1 << 15 | 1 << 20 | 1 << 25);
    let device = ZigbeeDevice::builder(make_mock()).channels(mask).build();

    assert_eq!(device.config.channel_mask, mask);
}

// ── Template Tests ───────────────────────────────────

#[test]
fn test_temperature_sensor_template() {
    let device = templates::temperature_sensor(make_mock()).build();
    assert_eq!(device.config.device_type, DeviceType::EndDevice);
    assert_eq!(device.config.endpoints.len(), 1);
    assert_eq!(device.config.endpoints[0].endpoint, 1);
    assert_eq!(device.config.endpoints[0].profile_id, 0x0104);
    assert_eq!(device.config.endpoints[0].device_id, 0x0302);
    assert!(device.config.endpoints[0].server_clusters.contains(&0x0402));
}

#[test]
fn test_on_off_light_template() {
    let device = templates::on_off_light(make_mock()).build();
    assert_eq!(device.config.device_type, DeviceType::Router);
    assert!(device.config.endpoints[0].server_clusters.contains(&0x0006));
}

#[test]
fn test_color_temperature_light_template() {
    let device = templates::color_temperature_light(make_mock()).build();
    assert!(device.config.endpoints[0].server_clusters.contains(&0x0300));
    assert!(device.config.endpoints[0].server_clusters.contains(&0x0008));
    assert!(device.config.endpoints[0].server_clusters.contains(&0x0006));
}

#[test]
fn test_smart_plug_template() {
    let device = templates::smart_plug(make_mock()).build();
    assert!(device.config.endpoints[0].server_clusters.contains(&0x0B04));
}

// ── NV Storage Tests ─────────────────────────────────

#[test]
fn test_ram_nv_write_read() {
    let mut nv = RamNvStorage::new();

    let data = [0x12, 0x34, 0x56, 0x78];
    nv.write(NvItemId::NwkPanId, &data).unwrap();

    let mut buf = [0u8; 8];
    let len = nv.read(NvItemId::NwkPanId, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(&buf[..4], &data);
}

#[test]
fn test_ram_nv_not_found() {
    let nv = RamNvStorage::new();
    let mut buf = [0u8; 8];
    assert_eq!(
        nv.read(NvItemId::NwkChannel, &mut buf),
        Err(NvError::NotFound)
    );
}

#[test]
fn test_ram_nv_overwrite() {
    let mut nv = RamNvStorage::new();
    nv.write(NvItemId::NwkChannel, &[15]).unwrap();
    nv.write(NvItemId::NwkChannel, &[20]).unwrap();

    let mut buf = [0u8; 1];
    nv.read(NvItemId::NwkChannel, &mut buf).unwrap();
    assert_eq!(buf[0], 20);
}

#[test]
fn test_ram_nv_delete() {
    let mut nv = RamNvStorage::new();
    nv.write(NvItemId::NwkKey, &[0xAA; 16]).unwrap();
    assert!(nv.exists(NvItemId::NwkKey));

    nv.delete(NvItemId::NwkKey).unwrap();
    assert!(!nv.exists(NvItemId::NwkKey));
}

#[test]
fn test_ram_nv_item_length() {
    let mut nv = RamNvStorage::new();
    nv.write(NvItemId::NwkIeeeAddress, &[1, 2, 3, 4, 5, 6, 7, 8])
        .unwrap();
    assert_eq!(nv.item_length(NvItemId::NwkIeeeAddress), Ok(8));
}

// ── Power Management Tests ───────────────────────────

#[test]
fn test_always_on_never_sleeps() {
    let pm = PowerManager::new(PowerMode::AlwaysOn);
    match pm.decide(1000) {
        SleepDecision::StayAwake => {} // Expected
        other => panic!("Expected StayAwake, got {:?}", other),
    }
}

#[test]
fn test_sleepy_device_sleeps_after_wake() {
    let mut pm = PowerManager::new(PowerMode::Sleepy {
        poll_interval_ms: 5000,
        wake_duration_ms: 100,
    });
    pm.record_activity(0);
    pm.record_poll(0);

    // Just after activity: stay awake
    match pm.decide(50) {
        SleepDecision::StayAwake => {}
        other => panic!("Expected StayAwake, got {:?}", other),
    }

    // After wake duration: can sleep
    match pm.decide(200) {
        SleepDecision::LightSleep(ms) => {
            assert!(ms > 0);
            assert!(ms <= 5000);
        }
        other => panic!("Expected LightSleep, got {:?}", other),
    }
}

#[test]
fn test_pending_tx_prevents_sleep() {
    let mut pm = PowerManager::new(PowerMode::Sleepy {
        poll_interval_ms: 5000,
        wake_duration_ms: 100,
    });
    pm.record_activity(0);
    pm.record_poll(0);
    pm.set_pending_tx(true);

    match pm.decide(10000) {
        SleepDecision::StayAwake => {}
        other => panic!("Expected StayAwake due to pending TX, got {:?}", other),
    }
}

#[test]
fn test_should_poll_timing() {
    let mut pm = PowerManager::new(PowerMode::Sleepy {
        poll_interval_ms: 1000,
        wake_duration_ms: 100,
    });
    pm.record_poll(0);

    assert!(!pm.should_poll(500)); // Too early
    assert!(pm.should_poll(1000)); // Exactly on time
    assert!(pm.should_poll(1500)); // Overdue
}
