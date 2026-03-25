//! Zigbee-RS Sleepy End Device (SED) Demo
//!
//! Simulates a battery-powered temperature/humidity sensor running on
//! the host machine using MockMac. Demonstrates the complete SED
//! lifecycle: cold boot → join → poll → sense → report → sleep → repeat.

use zigbee_mac::mock::MockMac;
use zigbee_mac::primitives::*;
use zigbee_mac::MacDriver;
use zigbee_runtime::nv_storage::{NvItemId, NvStorage, RamNvStorage};
use zigbee_runtime::power::{PowerManager, PowerMode, SleepDecision};
use zigbee_types::*;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::poll_control::PollControlCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::clusters::Cluster;

// ── ANSI color helpers ──────────────────────────────────────────
const RESET: &str = "\x1b[0m";
const BOLD: &str = "\x1b[1m";
const DIM: &str = "\x1b[2m";
const GREEN: &str = "\x1b[32m";
const YELLOW: &str = "\x1b[33m";
const BLUE: &str = "\x1b[34m";
const MAGENTA: &str = "\x1b[35m";
const CYAN: &str = "\x1b[36m";
const RED: &str = "\x1b[31m";
const WHITE: &str = "\x1b[97m";

// ── Simulated network parameters ────────────────────────────────
const SIM_PAN_ID: u16 = 0x1A2B;
const SIM_CHANNEL: u8 = 15;
const SIM_ASSIGNED_ADDR: u16 = 0x04D2;
const _SIM_COORD_IEEE: IeeeAddress = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
const SIM_COORD_EXT_PAN: IeeeAddress = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11];
const DEVICE_IEEE: IeeeAddress = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

// ── Sensor simulation parameters ────────────────────────────────
const BASE_TEMP_HUNDREDTHS: i16 = 2300; // 23.00°C
const BASE_HUMIDITY_HUNDREDTHS: u16 = 6500; // 65.00% RH
const TEMP_REPORTABLE_CHANGE: i16 = 10; // 0.10°C threshold
const HUMIDITY_REPORTABLE_CHANGE: u16 = 50; // 0.50% RH threshold
const CHECK_IN_CYCLE_INTERVAL: u32 = 5; // send check-in every 5th cycle
const TOTAL_CYCLES: u32 = 10;
const SLEEP_DURATION_S: u32 = 30;

// ── Simulated time tracking ─────────────────────────────────────
const CYCLE_DURATION_MS: u32 = SLEEP_DURATION_S * 1000;

fn main() {
    pollster::block_on(run());
}

async fn run() {
    print_banner();

    // ── Persistent state across cycles ──────────────────────
    let mut nv = RamNvStorage::new();
    let mut temp_cluster = TemperatureCluster::new(-4000, 8500); // -40°C to 85°C
    let mut humidity_cluster = HumidityCluster::new(0, 10000); // 0% to 100% RH
    let mut poll_control = PollControlCluster::new();
    let mut power_mgr = PowerManager::new(PowerMode::DeepSleep {
        wake_interval_s: SLEEP_DURATION_S,
    });

    let mut last_reported_temp: i16 = 0;
    let mut last_reported_humidity: u16 = 0;
    let mut sim_time_ms: u32 = 0;

    for cycle in 1..=TOTAL_CYCLES {
        let is_cold_boot = cycle == 1;
        let cycle_label = if is_cold_boot {
            "COLD BOOT"
        } else if cycle % CHECK_IN_CYCLE_INTERVAL == 0 {
            "CHECK-IN"
        } else {
            "WARM BOOT"
        };

        print_cycle_header(cycle, cycle_label);

        // Create a fresh MockMac each cycle (simulates radio re-init after deep sleep)
        let mut mac = create_mock_mac(cycle);

        // ── Phase 1: Join / Restore ─────────────────────────
        if is_cold_boot {
            phase_cold_boot(&mut mac, &mut nv).await;
        } else {
            phase_warm_boot(&nv);
        }

        // ── Phase 2: Poll Parent ────────────────────────────
        let pending = phase_poll_parent(&mut mac, cycle).await;
        if pending {
            power_mgr.record_activity(sim_time_ms);
        }

        // ── Phase 3: Read Sensors ───────────────────────────
        let (temp, humidity) = phase_read_sensors(cycle, &mut temp_cluster, &mut humidity_cluster);

        // ── Phase 4: Attribute Reporting ─────────────────────
        let reported = phase_report_attributes(
            &mut mac,
            temp,
            humidity,
            &mut last_reported_temp,
            &mut last_reported_humidity,
            &nv,
        )
        .await;
        if reported {
            power_mgr.record_activity(sim_time_ms);
        }

        // ── Phase 5: Poll Control Check-In ──────────────────
        if cycle % CHECK_IN_CYCLE_INTERVAL == 0 {
            phase_checkin(
                &mut mac,
                &mut poll_control,
                &nv,
                sim_time_ms,
                &mut power_mgr,
            )
            .await;
        } else {
            let elapsed_min = (cycle as f32 * SLEEP_DURATION_S as f32) / 60.0;
            let interval_min = (CHECK_IN_CYCLE_INTERVAL as f32 * SLEEP_DURATION_S as f32) / 60.0;
            println!(
                "  {CYAN}[POLL_CTRL]{RESET} Check-in interval not reached ({:.1}/{:.0} min)",
                elapsed_min % interval_min,
                interval_min
            );
        }

        // ── Phase 6: Sleep Decision ─────────────────────────
        power_mgr.set_pending_tx(false);
        power_mgr.set_pending_reports(false);
        // Advance simulated time well past activity window so power manager allows sleep
        let decision_time = sim_time_ms + 5000;
        phase_sleep_decision(&power_mgr, decision_time, &mut nv);

        sim_time_ms += CYCLE_DURATION_MS;
        println!();
    }

    print_footer();
}

// ═══════════════════════════════════════════════════════════════
//  Phase implementations
// ═══════════════════════════════════════════════════════════════

/// Phase 1a: Cold boot — scan, associate, save to NV.
async fn phase_cold_boot(mac: &mut MockMac, nv: &mut RamNvStorage) {
    println!("  {GREEN}[NV]{RESET} No saved network state, performing fresh join");

    // Scan
    println!("  {BLUE}[MAC]{RESET} Scanning channels 11-26...");
    let scan_result = mac
        .mlme_scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask::ALL_2_4GHZ,
            scan_duration: 3,
        })
        .await;

    match scan_result {
        Ok(confirm) => {
            let pd = &confirm.pan_descriptors[0];
            println!(
                "  {BLUE}[MAC]{RESET} Found network: PAN=0x{:04X}, channel={}",
                pd.coord_address.pan_id().0,
                pd.channel
            );
        }
        Err(e) => {
            println!("  {RED}[MAC]{RESET} Scan failed: {:?}", e);
            return;
        }
    }

    // Associate
    println!(
        "  {BLUE}[MAC]{RESET} Associating... assigned address 0x{:04X}",
        SIM_ASSIGNED_ADDR
    );
    let assoc_result = mac
        .mlme_associate(MlmeAssociateRequest {
            channel: SIM_CHANNEL,
            coord_address: MacAddress::Short(PanId(SIM_PAN_ID), ShortAddress::COORDINATOR),
            capability_info: CapabilityInfo {
                device_type_ffd: false,
                mains_powered: false,
                rx_on_when_idle: false, // SED: radio off when idle
                security_capable: false,
                allocate_address: true,
            },
        })
        .await;

    match assoc_result {
        Ok(confirm) if confirm.status == AssociationStatus::Success => {
            // Save network state to NV
            nv_write_network_state(nv, SIM_PAN_ID, confirm.short_address.0, SIM_CHANNEL);
            println!(
                "  {GREEN}[NV]{RESET} Saved: PAN=0x{:04X}, addr=0x{:04X}, channel={}",
                SIM_PAN_ID, confirm.short_address.0, SIM_CHANNEL
            );
            println!("  {GREEN}{BOLD}[JOIN]{RESET} Network joined successfully");
        }
        Ok(confirm) => {
            println!(
                "  {RED}[MAC]{RESET} Association denied: {:?}",
                confirm.status
            );
        }
        Err(e) => {
            println!("  {RED}[MAC]{RESET} Association failed: {:?}", e);
        }
    }
}

/// Phase 1b: Warm boot — restore network state from NV.
fn phase_warm_boot(nv: &RamNvStorage) {
    println!("  {GREEN}[NV]{RESET} Restoring network state from NV storage");
    if let Some((pan_id, short_addr, channel)) = nv_read_network_state(nv) {
        println!(
            "  {GREEN}[NV]{RESET} PAN=0x{:04X}, addr=0x{:04X}, channel={}",
            pan_id, short_addr, channel
        );
        println!("  {GREEN}[RESTORE]{RESET} Network state restored (no rejoin needed)");
    } else {
        println!("  {RED}[NV]{RESET} ERROR: NV state corrupted!");
    }
}

/// Phase 2: Poll parent for pending indirect data.
async fn phase_poll_parent(mac: &mut MockMac, cycle: u32) -> bool {
    println!();
    println!("  {MAGENTA}[POLL]{RESET} Polling parent for pending data...");
    let poll_result = mac.mlme_poll().await;
    match poll_result {
        Ok(Some(frame)) => {
            println!(
                "  {MAGENTA}[POLL]{RESET} Received pending frame ({} bytes)",
                frame.len()
            );
            true
        }
        Ok(None) => {
            // On even cycles, simulate receiving a pending frame
            if cycle.is_multiple_of(2) {
                println!("  {MAGENTA}[POLL]{RESET} Received 1 pending frame");
                true
            } else {
                println!("  {MAGENTA}[POLL]{RESET} No pending data");
                false
            }
        }
        Err(_) => {
            println!("  {MAGENTA}[POLL]{RESET} No pending data");
            false
        }
    }
}

/// Phase 3: Simulate sensor readings with drift.
fn phase_read_sensors(
    cycle: u32,
    temp_cluster: &mut TemperatureCluster,
    humidity_cluster: &mut HumidityCluster,
) -> (i16, u16) {
    println!();
    println!("  {YELLOW}[SENSOR]{RESET} Reading sensors...");

    // Simulate drift: temperature drifts ±0.02°C per cycle, humidity ±0.1%
    let temp_drift = match cycle % 7 {
        0 => -5,
        1 => 2,
        2 => 3,
        3 => -1,
        4 => 8,
        5 => -2,
        _ => 1,
    };
    let humidity_drift: i32 = match cycle % 5 {
        0 => -20,
        1 => 10,
        2 => 15,
        3 => -5,
        _ => 8,
    };

    let temp = BASE_TEMP_HUNDREDTHS + (cycle as i16 * 2) + temp_drift;
    let humidity = (BASE_HUMIDITY_HUNDREDTHS as i32 + (cycle as i32 * 3) + humidity_drift)
        .clamp(0, 10000) as u16;

    temp_cluster.set_temperature(temp);
    humidity_cluster.set_humidity(humidity);

    println!(
        "  {YELLOW}[SENSOR]{RESET} Temperature: {BOLD}{:.2}°C{RESET}, Humidity: {BOLD}{:.1}%{RESET}",
        temp as f64 / 100.0,
        humidity as f64 / 100.0,
    );

    (temp, humidity)
}

/// Phase 4: Check reportable change and send attribute reports.
async fn phase_report_attributes(
    mac: &mut MockMac,
    temp: i16,
    humidity: u16,
    last_temp: &mut i16,
    last_humidity: &mut u16,
    nv: &RamNvStorage,
) -> bool {
    println!();

    let temp_change = (temp - *last_temp).unsigned_abs() as i16;
    let humidity_change = humidity.abs_diff(*last_humidity);

    let temp_reportable = temp_change >= TEMP_REPORTABLE_CHANGE;
    let humidity_reportable = humidity_change >= HUMIDITY_REPORTABLE_CHANGE;

    if temp_reportable || humidity_reportable {
        if temp_reportable {
            println!(
                "  {CYAN}[ZCL]{RESET} Temperature changed by {BOLD}{:.2}°C{RESET} \
                 (threshold: {:.2}°C) → {GREEN}{BOLD}REPORTING{RESET}",
                temp_change as f64 / 100.0,
                TEMP_REPORTABLE_CHANGE as f64 / 100.0,
            );
        }
        if humidity_reportable {
            println!(
                "  {CYAN}[ZCL]{RESET} Humidity changed by {BOLD}{:.1}%{RESET} \
                 (threshold: {:.1}%) → {GREEN}{BOLD}REPORTING{RESET}",
                humidity_change as f64 / 100.0,
                HUMIDITY_REPORTABLE_CHANGE as f64 / 100.0,
            );
        }

        // Build a simulated ZCL attribute report payload
        println!(
            "  {CYAN}[ZCL]{RESET} Sending attribute report: Temp={}, Humidity={}",
            temp, humidity
        );

        // Send via MAC data service to coordinator
        if let Some((pan_id, _, _)) = nv_read_network_state(nv) {
            let report_payload = build_attribute_report(temp, humidity);
            let _ = mac
                .mcps_data(McpsDataRequest {
                    src_addr_mode: AddressMode::Short,
                    dst_address: MacAddress::Short(PanId(pan_id), ShortAddress::COORDINATOR),
                    payload: &report_payload,
                    msdu_handle: 0x01,
                    tx_options: TxOptions {
                        ack_tx: true,
                        indirect: false,
                        security_enabled: false,
                    },
                })
                .await;
        }

        *last_temp = temp;
        *last_humidity = humidity;
        true
    } else {
        println!(
            "  {CYAN}[ZCL]{RESET} Temperature changed by {:.2}°C (threshold: {:.2}°C) → {DIM}SKIP{RESET}",
            temp_change as f64 / 100.0,
            TEMP_REPORTABLE_CHANGE as f64 / 100.0,
        );
        println!("  {CYAN}[ZCL]{RESET} No reportable change");
        false
    }
}

/// Phase 5: Poll Control check-in with fast-poll simulation.
async fn phase_checkin(
    mac: &mut MockMac,
    poll_control: &mut PollControlCluster,
    nv: &RamNvStorage,
    sim_time_ms: u32,
    power_mgr: &mut PowerManager,
) {
    println!();
    println!("  {CYAN}{BOLD}[POLL_CTRL]{RESET} Check-in interval reached! Sending CheckIn...");

    // Build and "send" CheckIn command
    let _checkin_payload = poll_control.trigger_checkin();
    if let Some((pan_id, _, _)) = nv_read_network_state(nv) {
        let _ = mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(PanId(pan_id), ShortAddress::COORDINATOR),
                payload: &[0x09, 0x01, 0x00], // ZCL frame: CheckIn command
                msdu_handle: 0x02,
                tx_options: TxOptions {
                    ack_tx: true,
                    indirect: false,
                    security_enabled: false,
                },
            })
            .await;
    }

    // Simulate receiving CheckInResponse: start_fast_poll=true, timeout=40 qs (10s)
    let fast_poll_timeout_qs: u16 = 40; // 10 seconds in quarter-seconds
    let response_payload = [
        0x01,
        fast_poll_timeout_qs as u8,
        (fast_poll_timeout_qs >> 8) as u8,
    ];
    let _ = poll_control.handle_command(
        zigbee_zcl::CommandId(0x00), // CMD_CHECK_IN_RESPONSE
        &response_payload,
    );

    let timeout_s = fast_poll_timeout_qs as f32 / 4.0;
    println!(
        "  {CYAN}[POLL_CTRL]{RESET} Received CheckInResponse: fast_poll={GREEN}true{RESET}, timeout={:.0}s",
        timeout_s
    );
    println!(
        "  {CYAN}[POLL_CTRL]{RESET} Entering fast poll mode (250ms intervals) for {:.0}s",
        timeout_s
    );

    // Simulate fast polling
    let fast_poll_count = fast_poll_timeout_qs; // one poll per quarter-second
    let polls_to_show = fast_poll_count.min(6); // don't spam the output
    for i in 1..=polls_to_show {
        println!(
            "  {MAGENTA}[POLL]{RESET} Fast polling... {DIM}(poll {}/{}){RESET}",
            i, fast_poll_count
        );
        power_mgr.record_activity(sim_time_ms + (i as u32) * 250);
        power_mgr.record_poll(sim_time_ms + (i as u32) * 250);
    }
    if fast_poll_count > polls_to_show {
        println!(
            "  {MAGENTA}[POLL]{RESET} {DIM}... ({} more polls){RESET}",
            fast_poll_count - polls_to_show
        );
    }

    // End fast poll
    let _ = poll_control.handle_command(
        zigbee_zcl::CommandId(0x01), // CMD_FAST_POLL_STOP
        &[],
    );
    println!("  {MAGENTA}[POLL]{RESET} Fast poll complete, returning to long poll");
}

/// Phase 6: Power manager sleep decision.
fn phase_sleep_decision(power_mgr: &PowerManager, now_ms: u32, nv: &mut RamNvStorage) {
    println!();
    let decision = power_mgr.decide(now_ms);
    match decision {
        SleepDecision::StayAwake => {
            println!(
                "  {RED}[POWER]{RESET} Sleep decision: {BOLD}STAY AWAKE{RESET} (pending work)"
            );
        }
        SleepDecision::LightSleep(ms) => {
            println!(
                "  {BLUE}[POWER]{RESET} Sleep decision: {BOLD}LIGHT SLEEP{RESET} for {:.1}s",
                ms as f64 / 1000.0
            );
        }
        SleepDecision::DeepSleep(ms) => {
            println!(
                "  {BLUE}[POWER]{RESET} Sleep decision: {BOLD}{CYAN}DEEP SLEEP{RESET} for {BOLD}{}s{RESET}",
                ms / 1000
            );
            // Save frame counters before deep sleep
            println!("  {GREEN}[POWER]{RESET} Saving frame counters to NV...");
            let frame_counter: u32 = now_ms / 1000; // simulated counter
            let _ = nv.write(NvItemId::NwkFrameCounter, &frame_counter.to_le_bytes());
        }
    }
    println!("  💤 {DIM}Sleeping...{RESET}");
}

// ═══════════════════════════════════════════════════════════════
//  NV storage helpers
// ═══════════════════════════════════════════════════════════════

fn nv_write_network_state(nv: &mut RamNvStorage, pan_id: u16, short_addr: u16, channel: u8) {
    let _ = nv.write(NvItemId::NwkPanId, &pan_id.to_le_bytes());
    let _ = nv.write(NvItemId::NwkShortAddress, &short_addr.to_le_bytes());
    let _ = nv.write(NvItemId::NwkChannel, &[channel]);
    let _ = nv.write(NvItemId::BdbNodeIsOnNetwork, &[1u8]);
}

fn nv_read_network_state(nv: &RamNvStorage) -> Option<(u16, u16, u8)> {
    let mut flag = [0u8; 1];
    if nv.read(NvItemId::BdbNodeIsOnNetwork, &mut flag).is_err() || flag[0] == 0 {
        return None;
    }
    let mut pan_buf = [0u8; 2];
    let mut addr_buf = [0u8; 2];
    let mut ch_buf = [0u8; 1];
    nv.read(NvItemId::NwkPanId, &mut pan_buf).ok()?;
    nv.read(NvItemId::NwkShortAddress, &mut addr_buf).ok()?;
    nv.read(NvItemId::NwkChannel, &mut ch_buf).ok()?;
    Some((
        u16::from_le_bytes(pan_buf),
        u16::from_le_bytes(addr_buf),
        ch_buf[0],
    ))
}

// ═══════════════════════════════════════════════════════════════
//  MockMac factory
// ═══════════════════════════════════════════════════════════════

/// Create a MockMac pre-configured with simulated network responses.
fn create_mock_mac(cycle: u32) -> MockMac {
    let mut mac = MockMac::new(DEVICE_IEEE);

    // Add a beacon for scan discovery
    mac.add_beacon(PanDescriptor {
        channel: SIM_CHANNEL,
        coord_address: MacAddress::Short(PanId(SIM_PAN_ID), ShortAddress::COORDINATOR),
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
            extended_pan_id: SIM_COORD_EXT_PAN,
            tx_offset: [0xFF, 0xFF, 0xFF],
            update_id: 0,
        },
    });

    // Set association response
    mac.set_associate_response(MlmeAssociateConfirm {
        short_address: ShortAddress(SIM_ASSIGNED_ADDR),
        status: AssociationStatus::Success,
    });

    // On some cycles, enqueue a pending RX frame to simulate indirect data
    if cycle.is_multiple_of(3) {
        if let Some(frame) = MacFrame::from_slice(&[0x08, 0x01, 0x00, 0x20, 0x00]) {
            mac.enqueue_rx(McpsDataIndication {
                src_address: MacAddress::Short(PanId(SIM_PAN_ID), ShortAddress::COORDINATOR),
                dst_address: MacAddress::Short(PanId(SIM_PAN_ID), ShortAddress(SIM_ASSIGNED_ADDR)),
                lqi: 200,
                payload: frame,
                security_use: false,
            });
        }
    }

    mac
}

// ═══════════════════════════════════════════════════════════════
//  ZCL report builder
// ═══════════════════════════════════════════════════════════════

/// Build a simplified ZCL attribute report frame (for demonstration).
fn build_attribute_report(temp: i16, humidity: u16) -> [u8; 16] {
    let mut buf = [0u8; 16];
    // ZCL header: frame control=0x18 (cluster-specific, server→client, disable default response)
    buf[0] = 0x18;
    buf[1] = 0x01; // sequence number
    buf[2] = 0x0A; // Report Attributes command (0x0A)
                   // Temperature attribute: ID=0x0000, type=0x29 (I16), value
    buf[3] = 0x00;
    buf[4] = 0x00; // attr id
    buf[5] = 0x29; // data type I16
    let temp_bytes = temp.to_le_bytes();
    buf[6] = temp_bytes[0];
    buf[7] = temp_bytes[1];
    // Humidity attribute: ID=0x0000, type=0x21 (U16), value
    buf[8] = 0x00;
    buf[9] = 0x00; // attr id
    buf[10] = 0x21; // data type U16
    let hum_bytes = humidity.to_le_bytes();
    buf[11] = hum_bytes[0];
    buf[12] = hum_bytes[1];
    // Padding
    buf[13] = 0x00;
    buf[14] = 0x00;
    buf[15] = 0x00;
    buf
}

// ═══════════════════════════════════════════════════════════════
//  Pretty-printing
// ═══════════════════════════════════════════════════════════════

fn print_banner() {
    println!();
    println!("{BOLD}{CYAN}═══════════════════════════════════════════════════{RESET}");
    println!("{BOLD}{WHITE}  Zigbee-RS Sleepy End Device Demo{RESET}");
    println!("{DIM}  Simulating battery-powered temperature sensor{RESET}");
    println!("{DIM}  Using MockMac — no hardware required{RESET}");
    println!("{BOLD}{CYAN}═══════════════════════════════════════════════════{RESET}");
    println!();
    println!(
        "{DIM}  Device IEEE:  {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}{RESET}",
        DEVICE_IEEE[0],
        DEVICE_IEEE[1],
        DEVICE_IEEE[2],
        DEVICE_IEEE[3],
        DEVICE_IEEE[4],
        DEVICE_IEEE[5],
        DEVICE_IEEE[6],
        DEVICE_IEEE[7]
    );
    println!("{DIM}  Device type:  Sleepy End Device (SED){RESET}");
    println!("{DIM}  Power mode:   Deep Sleep ({SLEEP_DURATION_S}s intervals){RESET}");
    println!("{DIM}  Clusters:     Temperature (0x0402), Humidity (0x0405),{RESET}");
    println!("{DIM}                Poll Control (0x0020){RESET}");
    println!("{DIM}  Cycles:       {TOTAL_CYCLES}{RESET}");
    println!();
}

fn print_cycle_header(cycle: u32, label: &str) {
    let color = match label {
        "COLD BOOT" => RED,
        "CHECK-IN" => MAGENTA,
        _ => GREEN,
    };
    println!(
        "{BOLD}── Cycle {cycle}: {color}{label}{RESET} {BOLD}{}──{RESET}",
        "─".repeat(40 - label.len() - format!("{cycle}").len()),
    );
}

fn print_footer() {
    println!("{BOLD}{CYAN}═══════════════════════════════════════════════════{RESET}");
    println!("{BOLD}{WHITE}  Demo complete!{RESET}");
    println!();
    println!("{DIM}  This demo showed the full SED lifecycle:{RESET}");
    println!("{DIM}  1. Cold boot: scan → associate → save NV{RESET}");
    println!("{DIM}  2. Warm boot: restore NV → skip rejoin{RESET}");
    println!("{DIM}  3. MAC polling for indirect data from parent{RESET}");
    println!("{DIM}  4. Sensor reading with reportable change check{RESET}");
    println!("{DIM}  5. ZCL attribute reporting when threshold exceeded{RESET}");
    println!("{DIM}  6. Poll Control check-in with fast-poll mode{RESET}");
    println!("{DIM}  7. Power manager deep sleep decisions{RESET}");
    println!("{DIM}  8. NV persistence of network state & frame counters{RESET}");
    println!("{BOLD}{CYAN}═══════════════════════════════════════════════════{RESET}");
    println!();
}
