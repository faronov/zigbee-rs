//! Mock Coordinator Example
//!
//! Demonstrates forming a Zigbee network using the zigbee-rs stack.
//!
//! This example:
//! 1. Creates a MockMac configured for coordinator operation
//! 2. Performs an energy detection scan to select the best channel
//! 3. Forms the network via MLME-START
//! 4. Sets up the Trust Center with default link key
//! 5. Configures address allocation and permit-joining
//! 6. Prints complete network information
//!
//! Run with: cargo run -p mock-coordinator

use zigbee::coordinator::{Coordinator, CoordinatorConfig};
use zigbee::trust_center::{DEFAULT_TC_LINK_KEY, TrustCenter};
use zigbee_mac::MacDriver;
use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::*;
use zigbee_nwk::DeviceType;
use zigbee_runtime::builder::DeviceBuilder;
use zigbee_types::*;

fn main() {
    println!("╔══════════════════════════════════════════════════════╗");
    println!("║  zigbee-rs Mock Coordinator                         ║");
    println!("╚══════════════════════════════════════════════════════╝");
    println!();

    // ── Step 1: Create MockMac for coordinator ──────────────────────
    println!("── Step 1: Initialize Coordinator MAC ──");

    let coord_ieee: IeeeAddress = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
    let mut mac = MockMac::new(coord_ieee);
    println!(
        "  IEEE address: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        coord_ieee[0],
        coord_ieee[1],
        coord_ieee[2],
        coord_ieee[3],
        coord_ieee[4],
        coord_ieee[5],
        coord_ieee[6],
        coord_ieee[7]
    );

    // Add energy scan results — lower energy = quieter channel
    let ed_results = [
        EdValue {
            channel: 11,
            energy: 180,
        }, // Noisy
        EdValue {
            channel: 15,
            energy: 45,
        }, // Quiet — best choice
        EdValue {
            channel: 20,
            energy: 90,
        }, // Moderate
        EdValue {
            channel: 25,
            energy: 60,
        }, // Fairly quiet
    ];
    for ed in &ed_results {
        mac.add_energy(*ed);
    }
    println!("  Configured energy scan results for 4 channels");
    println!();

    // ── Step 2: Energy Detection Scan ───────────────────────────────
    println!("── Step 2: Energy Detection Scan ──");

    let selected_channel = pollster::block_on(async {
        mac.mlme_reset(true).await.expect("Reset failed");
        println!("  MLME-RESET.request(setDefaultPIB=true) → OK");

        let scan_req = MlmeScanRequest {
            scan_type: ScanType::Ed,
            channel_mask: ChannelMask::ALL_2_4GHZ,
            scan_duration: 5,
        };
        let scan_result = mac.mlme_scan(scan_req).await.expect("ED scan failed");
        println!(
            "  MLME-SCAN.request(ED) → {} measurements",
            scan_result.energy_list.len()
        );

        // Select the channel with lowest energy (quietest)
        let mut best_channel = 11u8;
        let mut lowest_energy = 255u8;
        for ed in &scan_result.energy_list {
            let marker = if ed.energy < lowest_energy {
                " ← best"
            } else {
                ""
            };
            println!(
                "    Channel {}: energy level {}{}",
                ed.channel, ed.energy, marker
            );
            if ed.energy < lowest_energy {
                lowest_energy = ed.energy;
                best_channel = ed.channel;
            }
        }
        println!(
            "  Selected channel {} (energy={})",
            best_channel, lowest_energy
        );
        best_channel
    });
    println!();

    // ── Step 3: Form the Network ────────────────────────────────────
    println!("── Step 3: NLME-NETWORK-FORMATION ──");

    let pan_id = PanId(0x1A62);
    pollster::block_on(async {
        let start_req = MlmeStartRequest {
            pan_id,
            channel: selected_channel,
            beacon_order: 15, // Non-beacon network (ZigBee PRO)
            superframe_order: 15,
            pan_coordinator: true,
            battery_life_ext: false,
        };
        mac.mlme_start(start_req).await.expect("Start failed");
        println!("  MLME-START.request → Network formed!");
        println!("    PAN ID:          0x{:04X}", pan_id.0);
        println!("    Channel:         {}", selected_channel);
        println!("    Short address:   0x0000 (coordinator)");
        println!("    Beacon order:    15 (non-beacon)");

        // Set coordinator short address via PIB
        mac.mlme_set(
            zigbee_mac::pib::PibAttribute::MacShortAddress,
            zigbee_mac::pib::PibValue::ShortAddress(ShortAddress::COORDINATOR),
        )
        .await
        .expect("Set short address failed");

        mac.mlme_set(
            zigbee_mac::pib::PibAttribute::MacAssociationPermit,
            zigbee_mac::pib::PibValue::Bool(true),
        )
        .await
        .expect("Set association permit failed");

        println!("    Association:     PERMITTED");
    });
    println!();

    // ── Step 4: Trust Center Setup ──────────────────────────────────
    println!("── Step 4: Trust Center Configuration ──");

    let mut coordinator = Coordinator::new(CoordinatorConfig {
        channel_mask: ChannelMask::ALL_2_4GHZ,
        extended_pan_id: coord_ieee,
        centralized_security: true,
        require_install_codes: false,
        max_children: 20,
        max_depth: 5,
        initial_permit_join_duration: 254, // Open for joining
    });

    // Generate and set network key
    coordinator.generate_network_key();
    let nwk_key = coordinator.network_key();
    println!("  Network key: {:02X?}", nwk_key);

    // Set up Trust Center
    let mut tc = TrustCenter::new(*nwk_key);
    tc.set_require_install_codes(false);
    println!(
        "  TC link key: {:02X?} (ZigBeeAlliance09)",
        DEFAULT_TC_LINK_KEY
    );
    println!("  Install codes: NOT required");

    // Mark network as formed
    coordinator.mark_formed();
    println!("  Network formed: {}", coordinator.is_formed());
    println!();

    // ── Step 5: Simulate Device Joining ─────────────────────────────
    println!("── Step 5: Simulate Device Joining ──");

    let joining_devices = [
        (
            [0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44],
            "Temp/Humidity Sensor",
        ),
        (
            [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80],
            "Dimmable Light",
        ),
        (
            [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE],
            "Smart Plug",
        ),
    ];

    for (ieee, name) in &joining_devices {
        if coordinator.can_accept_child() {
            let short = coordinator.allocate_address();

            // TC provides the link key for the joining device
            let link_key = tc.link_key_for_device(ieee);
            let _ = tc.set_link_key(
                *ieee,
                link_key,
                zigbee::trust_center::TcKeyType::DefaultGlobal,
            );

            println!(
                "  Device '{}' joined: IEEE {:02X?} → short 0x{:04X}",
                name, ieee, short.0
            );
        }
    }
    println!();

    // ── Step 6: Build Coordinator ZigbeeDevice ──────────────────────
    println!("── Step 6: Build Coordinator Runtime Device ──");

    // Create a second MockMac for the DeviceBuilder (the first is consumed above)
    let mac_for_device = MockMac::new(coord_ieee);
    let device = DeviceBuilder::new(mac_for_device)
        .device_type(DeviceType::Coordinator)
        .manufacturer("zigbee-rs")
        .model("MockCoordinator-01")
        .sw_build("0.1.0")
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(1, 0x0104, 0x0000, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0003) // Identify
        })
        .build();

    println!("  Built coordinator device with HA profile");
    println!("  Endpoint 1: Basic + Identify clusters");
    println!();

    // ── Step 7: Network Summary ─────────────────────────────────────
    println!("── Network Summary ──");
    println!("  ┌─────────────────────────────────────────────┐");
    println!(
        "  │ PAN ID:            0x{:04X}                  │",
        pan_id.0
    );
    println!(
        "  │ Channel:           {}                      │",
        selected_channel
    );
    println!("  │ Coordinator:       0x0000                   │");
    println!("  │ Security:          Centralized (TC)         │");
    println!("  │ Joined devices:    3                        │");
    println!("  │ Association:       OPEN                     │");
    println!("  │ Network formed:    true                     │");
    println!("  └─────────────────────────────────────────────┘");
    println!();
    println!("✓ Mock coordinator example completed successfully!");

    drop(device);
}
