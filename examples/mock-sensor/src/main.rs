//! Mock Temperature + Humidity Sensor Example
//!
//! Demonstrates the zigbee-rs stack on a host machine (no hardware needed).
//!
//! This example:
//! 1. Creates a MockMac with pre-configured scan/associate responses
//! 2. Builds a temperature + humidity sensor using the DeviceBuilder
//! 3. Runs through the network join sequence (scan → associate)
//! 4. Creates ZCL clusters and updates sensor readings
//! 5. Reads attribute values back to verify
//!
//! Run with: cargo run -p mock-sensor

use zigbee_mac::MacDriver;
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::*;
use zigbee_runtime::templates;
use zigbee_types::*;
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::data_types::ZclValue;

fn main() {
    println!("╔══════════════════════════════════════════════════════╗");
    println!("║  zigbee-rs Mock Temperature + Humidity Sensor       ║");
    println!("╚══════════════════════════════════════════════════════╝");
    println!();

    // ── Step 1: Create MockMac with pre-configured responses ────────
    println!("── Step 1: Configure Mock MAC Layer ──");

    let ieee_addr: IeeeAddress = [0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44];
    let mut mac = MockMac::new(ieee_addr);
    println!(
        "  Created MockMac with IEEE address: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        ieee_addr[0],
        ieee_addr[1],
        ieee_addr[2],
        ieee_addr[3],
        ieee_addr[4],
        ieee_addr[5],
        ieee_addr[6],
        ieee_addr[7]
    );

    // Simulate a coordinator beacon on channel 15
    let coordinator_pan = PanId(0x1A62);
    let coordinator_addr = ShortAddress(0x0000);
    let extended_pan_id: IeeeAddress = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

    let beacon = PanDescriptor {
        channel: 15,
        coord_address: MacAddress::Short(coordinator_pan, coordinator_addr),
        superframe_spec: SuperframeSpec {
            beacon_order: 15,
            superframe_order: 15,
            final_cap_slot: 15,
            battery_life_ext: false,
            pan_coordinator: true,
            association_permit: true,
        },
        lqi: 220,
        security_use: false,
        zigbee_beacon: ZigbeeBeaconPayload {
            protocol_id: 0x00,
            stack_profile: 2, // ZigBee PRO
            protocol_version: 2,
            router_capacity: true,
            device_depth: 0,
            end_device_capacity: true,
            extended_pan_id,
            tx_offset: [0xFF, 0xFF, 0xFF],
            update_id: 0,
        },
    };
    mac.add_beacon(beacon);
    println!(
        "  Added coordinator beacon: PAN 0x{:04X}, channel 15, LQI 220",
        coordinator_pan.0
    );

    // Pre-configure a successful association response
    let assigned_address = ShortAddress(0x796F);
    mac.set_associate_response(MlmeAssociateConfirm {
        short_address: assigned_address,
        status: AssociationStatus::Success,
    });
    println!(
        "  Set association response: short addr 0x{:04X} (Success)",
        assigned_address.0
    );
    println!();

    // ── Step 2: Build the device using the template ─────────────────
    println!("── Step 2: Build Sensor Device ──");

    let device = templates::temperature_humidity_sensor(mac)
        .manufacturer("zigbee-rs")
        .model("MockTempHumid-01")
        .sw_build("0.1.0")
        .channels(ChannelMask::PREFERRED)
        .build();

    println!("  Built temperature + humidity sensor device");
    println!("  Device type: EndDevice");
    println!("  Profile: Home Automation (0x0104)");
    println!("  Endpoint 1 server clusters:");
    println!("    - Basic (0x0000)");
    println!("    - Power Configuration (0x0001)");
    println!("    - Identify (0x0003)");
    println!("    - Temperature Measurement (0x0402)");
    println!("    - Relative Humidity (0x0405)");
    println!();

    // ── Step 3: Network join sequence via MAC primitives ────────────
    println!("── Step 3: Network Join Sequence ──");

    // Recover the mac from the device to perform MAC-level operations.
    // In a real application the runtime event loop drives this, but here
    // we demonstrate the raw MAC primitives directly.
    let mut mac2 = MockMac::new(ieee_addr);

    // Re-add the beacon and association response for the second mac instance
    mac2.add_beacon(PanDescriptor {
        channel: 15,
        coord_address: MacAddress::Short(coordinator_pan, coordinator_addr),
        superframe_spec: SuperframeSpec {
            beacon_order: 15,
            superframe_order: 15,
            final_cap_slot: 15,
            battery_life_ext: false,
            pan_coordinator: true,
            association_permit: true,
        },
        lqi: 220,
        security_use: false,
        zigbee_beacon: ZigbeeBeaconPayload {
            protocol_id: 0x00,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity: true,
            device_depth: 0,
            end_device_capacity: true,
            extended_pan_id,
            tx_offset: [0xFF, 0xFF, 0xFF],
            update_id: 0,
        },
    });
    mac2.set_associate_response(MlmeAssociateConfirm {
        short_address: assigned_address,
        status: AssociationStatus::Success,
    });

    pollster::block_on(async {
        // 3a) Reset MAC
        mac2.mlme_reset(true).await.expect("MAC reset failed");
        println!("  [3a] MLME-RESET.request(setDefaultPIB=true) → OK");

        // 3b) Active scan on preferred channels
        let scan_req = MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask::PREFERRED,
            scan_duration: 3,
        };
        let scan_confirm = mac2.mlme_scan(scan_req).await.expect("Scan failed");
        println!(
            "  [3b] MLME-SCAN.request(Active, preferred channels) → found {} PAN(s)",
            scan_confirm.pan_descriptors.len()
        );
        for (i, pd) in scan_confirm.pan_descriptors.iter().enumerate() {
            println!(
                "       PAN[{}]: channel {}, LQI {}, association_permit={}",
                i, pd.channel, pd.lqi, pd.superframe_spec.association_permit
            );
        }

        // 3c) Associate with the best PAN
        let best = &scan_confirm.pan_descriptors[0];
        let assoc_req = MlmeAssociateRequest {
            channel: best.channel,
            coord_address: best.coord_address,
            capability_info: CapabilityInfo {
                device_type_ffd: false, // End device
                mains_powered: false,   // Battery
                rx_on_when_idle: false, // Sleepy
                security_capable: false,
                allocate_address: true,
            },
        };
        let assoc_confirm = mac2
            .mlme_associate(assoc_req)
            .await
            .expect("Association failed");
        println!(
            "  [3c] MLME-ASSOCIATE.request → status={:?}, short_addr=0x{:04X}",
            assoc_confirm.status, assoc_confirm.short_address.0
        );

        // 3d) Start with the assigned PAN parameters
        let start_req = MlmeStartRequest {
            pan_id: coordinator_pan,
            channel: best.channel,
            beacon_order: 15, // Non-beacon network
            superframe_order: 15,
            pan_coordinator: false,
            battery_life_ext: false,
        };
        mac2.mlme_start(start_req).await.expect("Start failed");
        println!(
            "  [3d] MLME-START.request → joined PAN 0x{:04X} on channel {}",
            coordinator_pan.0, best.channel
        );
    });
    println!();

    // ── Step 4: Create and configure ZCL clusters ───────────────────
    println!("── Step 4: ZCL Cluster Configuration ──");

    // Basic cluster
    let mut basic = BasicCluster::new(b"zigbee-rs", b"MockTempHumid-01", b"20250101", b"0.1.0");
    basic.set_power_source(0x03); // Battery
    println!("  Basic cluster: manufacturer='zigbee-rs', model='MockTempHumid-01'");
    println!("  Power source: Battery (0x03)");

    // Temperature cluster: range -40.00°C to +125.00°C
    let mut temp = TemperatureCluster::new(-4000, 12500);
    println!("  Temperature cluster: range [-40.00°C, +125.00°C]");

    // Humidity cluster: range 0.00% to 100.00%
    let mut humid = HumidityCluster::new(0, 10000);
    println!("  Humidity cluster: range [0.00%, 100.00%]");
    println!();

    // ── Step 5: Simulate sensor readings ────────────────────────────
    println!("── Step 5: Simulate Sensor Readings ──");

    let readings: &[(i16, u16)] = &[
        (2350, 6500), // 23.50°C, 65.00%
        (2410, 6380), // 24.10°C, 63.80%
        (2275, 7100), // 22.75°C, 71.00%
        (1890, 8250), // 18.90°C, 82.50%
    ];

    for (i, &(t, h)) in readings.iter().enumerate() {
        temp.set_temperature(t);
        humid.set_humidity(h);

        // Read back via the Cluster trait
        let temp_val = temp
            .attributes()
            .get(zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE);
        let humid_val = humid
            .attributes()
            .get(zigbee_zcl::clusters::humidity::ATTR_MEASURED_VALUE);

        let temp_display = match temp_val {
            Some(ZclValue::I16(v)) => format!("{:.2}°C", *v as f64 / 100.0),
            _ => "unknown".to_string(),
        };
        let humid_display = match humid_val {
            Some(ZclValue::U16(v)) => format!("{:.2}%", *v as f64 / 100.0),
            _ => "unknown".to_string(),
        };

        println!(
            "  Reading #{}: temperature={}, humidity={}",
            i + 1,
            temp_display,
            humid_display
        );
    }
    println!();

    // ── Step 6: Read all temperature cluster attributes ─────────────
    println!("── Step 6: Read All Attributes ──");

    println!("  Temperature Measurement cluster attributes:");
    let attrs = temp.attributes();
    for attr_id in &[
        zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE,
        zigbee_zcl::clusters::temperature::ATTR_MIN_MEASURED_VALUE,
        zigbee_zcl::clusters::temperature::ATTR_MAX_MEASURED_VALUE,
        zigbee_zcl::clusters::temperature::ATTR_TOLERANCE,
    ] {
        if let Some(val) = attrs.get(*attr_id) {
            let name = attrs.find(*attr_id).map(|d| d.name).unwrap_or("?");
            println!("    0x{:04X} ({}) = {:?}", attr_id.0, name, val);
        }
    }

    println!("  Humidity Measurement cluster attributes:");
    let hattrs = humid.attributes();
    for attr_id in &[
        zigbee_zcl::clusters::humidity::ATTR_MEASURED_VALUE,
        zigbee_zcl::clusters::humidity::ATTR_MIN_MEASURED_VALUE,
        zigbee_zcl::clusters::humidity::ATTR_MAX_MEASURED_VALUE,
        zigbee_zcl::clusters::humidity::ATTR_TOLERANCE,
    ] {
        if let Some(val) = hattrs.get(*attr_id) {
            let name = hattrs.find(*attr_id).map(|d| d.name).unwrap_or("?");
            println!("    0x{:04X} ({}) = {:?}", attr_id.0, name, val);
        }
    }

    // ── Step 7: Verify TX history ───────────────────────────────────
    println!();
    println!("── Step 7: MockMac TX History ──");
    let history = mac2.tx_history();
    if history.is_empty() {
        println!("  No frames transmitted (expected — we only did MAC-level join)");
    } else {
        for (i, rec) in history.iter().enumerate() {
            println!(
                "  TX[{}]: dst={:?}, payload_len={}, handle={}, ack={}",
                i, rec.dst, rec.payload_len, rec.handle, rec.ack_requested
            );
        }
    }

    println!();
    println!("✓ Mock sensor example completed successfully!");
    println!("  This demonstrates:");
    println!("  • MockMac configuration with beacons and association");
    println!("  • DeviceBuilder template for temp+humidity sensor");
    println!("  • MAC-level scan and association primitives");
    println!("  • ZCL cluster creation and attribute read/write");

    // Keep the built device alive to show it compiled successfully
    drop(device);
}
