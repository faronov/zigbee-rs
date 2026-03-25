//! Mock Dimmable Light Example
//!
//! Demonstrates an On/Off + Level Control light device using the zigbee-rs stack.
//!
//! This example:
//! 1. Creates a MockMac and joins a network
//! 2. Creates On/Off, Level Control, and Basic clusters
//! 3. Handles On/Off/Toggle commands
//! 4. Handles MoveToLevel commands with transition times
//! 5. Prints light state changes at each step
//!
//! Run with: cargo run -p mock-light

use zigbee_mac::MacDriver;
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::*;
use zigbee_runtime::templates;
use zigbee_types::*;
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::level_control::{self, CMD_MOVE_TO_LEVEL, CMD_STEP, LevelControlCluster};
use zigbee_zcl::clusters::on_off::{self, CMD_OFF, CMD_ON, CMD_TOGGLE, OnOffCluster};

/// Print the current light state in a visual format.
fn print_light_state(on_off: &OnOffCluster, level: &LevelControlCluster) {
    let is_on = on_off.is_on();
    let brightness = level.current_level();
    let percent = (brightness as f64 / 254.0 * 100.0).round() as u8;

    let bar_len = (brightness as usize * 20) / 254;
    let bar: String = "█".repeat(bar_len) + &"░".repeat(20 - bar_len);

    if is_on {
        println!("    💡 ON  [{bar}] {percent}% (level={brightness})");
    } else {
        println!("    ⚫ OFF [{bar}] {percent}% (level={brightness})");
    }
}

fn main() {
    println!("╔══════════════════════════════════════════════════════╗");
    println!("║  zigbee-rs Mock Dimmable Light                      ║");
    println!("╚══════════════════════════════════════════════════════╝");
    println!();

    // ── Step 1: Create MockMac and join network ─────────────────────
    println!("── Step 1: Join Network ──");

    let light_ieee: IeeeAddress = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];
    let mut mac = MockMac::new(light_ieee);

    // Configure a coordinator beacon
    let pan_id = PanId(0x1A62);
    mac.add_beacon(PanDescriptor {
        channel: 15,
        coord_address: MacAddress::Short(pan_id, ShortAddress::COORDINATOR),
        superframe_spec: SuperframeSpec {
            beacon_order: 15,
            superframe_order: 15,
            final_cap_slot: 15,
            battery_life_ext: false,
            pan_coordinator: true,
            association_permit: true,
        },
        lqi: 200,
        security_use: false,
        zigbee_beacon: ZigbeeBeaconPayload {
            protocol_id: 0x00,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity: true,
            device_depth: 0,
            end_device_capacity: true,
            extended_pan_id: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            tx_offset: [0xFF, 0xFF, 0xFF],
            update_id: 0,
        },
    });

    let assigned_short = ShortAddress(0x5E3D);
    mac.set_associate_response(MlmeAssociateConfirm {
        short_address: assigned_short,
        status: AssociationStatus::Success,
    });

    pollster::block_on(async {
        mac.mlme_reset(true).await.expect("Reset failed");
        println!("  MLME-RESET → OK");

        // Scan
        let scan = mac
            .mlme_scan(MlmeScanRequest {
                scan_type: ScanType::Active,
                channel_mask: ChannelMask::ALL_2_4GHZ,
                scan_duration: 3,
            })
            .await
            .expect("Scan failed");
        println!(
            "  MLME-SCAN(Active) → found {} PAN(s)",
            scan.pan_descriptors.len()
        );

        // Associate
        let best = &scan.pan_descriptors[0];
        let assoc = mac
            .mlme_associate(MlmeAssociateRequest {
                channel: best.channel,
                coord_address: best.coord_address,
                capability_info: CapabilityInfo {
                    device_type_ffd: true, // Lights are routers (FFD)
                    mains_powered: true,
                    rx_on_when_idle: true,
                    security_capable: false,
                    allocate_address: true,
                },
            })
            .await
            .expect("Association failed");
        println!(
            "  MLME-ASSOCIATE → short_addr=0x{:04X}, status={:?}",
            assoc.short_address.0, assoc.status
        );
    });
    println!();

    // ── Step 2: Build dimmable light via template ───────────────────
    println!("── Step 2: Build Dimmable Light Device ──");

    let mac_for_device = MockMac::new(light_ieee);
    let device = templates::dimmable_light(mac_for_device)
        .manufacturer("zigbee-rs")
        .model("MockDimLight-01")
        .sw_build("0.1.0")
        .build();

    println!("  Device type: Router (mains-powered light)");
    println!("  HA Device ID: 0x0101 (Dimmable Light)");
    println!("  Endpoint 1 clusters:");
    println!("    - Basic (0x0000)");
    println!("    - Identify (0x0003)");
    println!("    - Groups (0x0004)");
    println!("    - Scenes (0x0005)");
    println!("    - On/Off (0x0006)");
    println!("    - Level Control (0x0008)");
    println!();

    // ── Step 3: Create cluster instances ─────────────────────────────
    println!("── Step 3: Initialize Clusters ──");

    let mut basic = BasicCluster::new(b"zigbee-rs", b"MockDimLight-01", b"20250101", b"0.1.0");
    basic.set_power_source(0x01); // Mains single phase

    let mut on_off = OnOffCluster::new();
    let mut level = LevelControlCluster::new();

    println!("  Initial state:");
    print_light_state(&on_off, &level);
    println!();

    // ── Step 4: Handle On/Off commands ──────────────────────────────
    println!("── Step 4: On/Off Commands ──");

    println!("  → CMD_ON (0x01)");
    let _ = on_off.handle_command(CMD_ON, &[]);
    print_light_state(&on_off, &level);

    // Set initial brightness since light was just turned on
    let move_payload = [0xC8, 0x00, 0x00]; // level=200, transition=0
    let _ = level.handle_command(CMD_MOVE_TO_LEVEL, &move_payload);
    println!("  → MoveToLevel(200, transition=0)");
    print_light_state(&on_off, &level);

    println!("  → CMD_TOGGLE (0x02)");
    let _ = on_off.handle_command(CMD_TOGGLE, &[]);
    print_light_state(&on_off, &level);

    println!("  → CMD_TOGGLE (0x02)");
    let _ = on_off.handle_command(CMD_TOGGLE, &[]);
    print_light_state(&on_off, &level);

    println!("  → CMD_OFF (0x00)");
    let _ = on_off.handle_command(CMD_OFF, &[]);
    print_light_state(&on_off, &level);
    println!();

    // ── Step 5: Handle Level Control commands ───────────────────────
    println!("── Step 5: Level Control Commands ──");

    // Turn on first
    let _ = on_off.handle_command(CMD_ON, &[]);

    // MoveToLevel: level=50, transition=10 (1 second)
    let payload = [50u8, 0x0A, 0x00]; // level, transition_time LE
    let _ = level.handle_command(CMD_MOVE_TO_LEVEL, &payload);
    println!("  → MoveToLevel(50, transition=10)");
    print_light_state(&on_off, &level);

    // MoveToLevel: level=254, transition=20 (2 seconds)
    let payload = [254u8, 0x14, 0x00];
    let _ = level.handle_command(CMD_MOVE_TO_LEVEL, &payload);
    println!("  → MoveToLevel(254, transition=20) — full brightness");
    print_light_state(&on_off, &level);

    // Step up: mode=0(up), step=30, transition=5
    let payload = [0u8, 30, 0x05, 0x00]; // mode, step_size, transition LE
    let current_before = level.current_level();
    let _ = level.handle_command(CMD_STEP, &payload);
    println!(
        "  → Step(up, step=30, transition=5): {} → {}",
        current_before,
        level.current_level()
    );
    print_light_state(&on_off, &level);

    // Step down: mode=1(down), step=100, transition=10
    let payload = [1u8, 100, 0x0A, 0x00];
    let current_before = level.current_level();
    let _ = level.handle_command(CMD_STEP, &payload);
    println!(
        "  → Step(down, step=100, transition=10): {} → {}",
        current_before,
        level.current_level()
    );
    print_light_state(&on_off, &level);

    // MoveToLevel: dim to 10
    let payload = [10u8, 0x00, 0x00];
    let _ = level.handle_command(CMD_MOVE_TO_LEVEL, &payload);
    println!("  → MoveToLevel(10, transition=0) — very dim");
    print_light_state(&on_off, &level);
    println!();

    // ── Step 6: Read all cluster attributes ─────────────────────────
    println!("── Step 6: Read Cluster Attributes ──");

    println!("  On/Off cluster:");
    let oo_attrs = on_off.attributes();
    for attr_id in &[
        on_off::ATTR_ON_OFF,
        on_off::ATTR_GLOBAL_SCENE_CONTROL,
        on_off::ATTR_ON_TIME,
        on_off::ATTR_OFF_WAIT_TIME,
    ] {
        if let Some(val) = oo_attrs.get(*attr_id) {
            let name = oo_attrs.find(*attr_id).map(|d| d.name).unwrap_or("?");
            println!("    0x{:04X} ({}) = {:?}", attr_id.0, name, val);
        }
    }

    println!("  Level Control cluster:");
    let lc_attrs = level.attributes();
    for attr_id in &[
        level_control::ATTR_CURRENT_LEVEL,
        level_control::ATTR_MIN_LEVEL,
        level_control::ATTR_MAX_LEVEL,
        level_control::ATTR_ON_OFF_TRANSITION_TIME,
        level_control::ATTR_ON_LEVEL,
    ] {
        if let Some(val) = lc_attrs.get(*attr_id) {
            let name = lc_attrs.find(*attr_id).map(|d| d.name).unwrap_or("?");
            println!("    0x{:04X} ({}) = {:?}", attr_id.0, name, val);
        }
    }
    println!();

    // ── Step 7: Verify MockMac state ────────────────────────────────
    println!("── Step 7: MAC Verification ──");
    let caps = mac.capabilities();
    println!("  MAC capabilities:");
    println!("    coordinator:      {}", caps.coordinator);
    println!("    router:           {}", caps.router);
    println!("    hardware_security: {}", caps.hardware_security);
    println!("    max_payload:      {}", caps.max_payload);

    println!();
    println!("✓ Mock dimmable light example completed successfully!");
    println!("  This demonstrates:");
    println!("  • Network join as a router device");
    println!("  • On/Off cluster with ON, OFF, TOGGLE commands");
    println!("  • Level Control with MoveToLevel and Step commands");
    println!("  • Visual light state feedback");
    println!("  • Attribute reading via the Cluster trait");

    drop(device);
}
