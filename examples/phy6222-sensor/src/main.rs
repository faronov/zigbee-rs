//! # Zigbee-RS PHY6222 Sensor (SED)
//!
//! Full-featured Zigbee 3.0 sleepy end device for PHY6222/6252-based boards.
//! Pure-Rust radio driver — no vendor SDK, no binary blobs, no C FFI.
//!
//! # Hardware
//! - PHY6222 (512KB flash, 64KB SRAM) or PHY6252 (256KB), ARM Cortex-M0
//! - 2.4 GHz radio with IEEE 802.15.4 + BLE support
//! - Common boards: PB-03F (PHY6252), THB2/TH05F/BTH01 (PHY6222)
//!
//! # Features
//! - Auto-join on boot (no button required)
//! - Sleepy End Device: poll parent for indirect frames
//! - Fast poll (250ms) during ZHA interview, slow poll (10s) normal
//! - LED status: triple-blink boot, double-blink joining, solid joined
//! - Button: short press = toggle join/leave, long press = factory reset
//! - Device_annce retries for reliable coordinator discovery
//!
//! # Build
//! ```bash
//! cargo build --release
//! ```

#![no_std]
#![no_main]

#[cfg(feature = "stubs")]
mod stubs;

mod time_driver;
mod vectors;
mod flash_nv;

use cortex_m as _;
use panic_halt as _;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::phy6222::Phy6222Mac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 30;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 10;
const FAST_POLL_DURATION_SECS: u64 = 120;
const EXPECTED_REPORT_CLUSTERS: usize = 3; // PowerConfig + Temp + Humidity

// ── PHY6222 GPIO ────────────────────────────────────────────────

mod pins {
    pub const LED_G: u8 = 12; // Green LED (PB-03F), active LOW
    pub const BTN: u8 = 15;   // PROG button, active LOW
}

fn led_on() { phy6222_hal::gpio::write(pins::LED_G, false); }  // active LOW
fn led_off() { phy6222_hal::gpio::write(pins::LED_G, true); }

// ── Main ────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    time_driver::init();

    unsafe {
        cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::LlIrq);
    }

    log::info!("[PHY6222] Zigbee Sensor starting (pure Rust!)");

    // GPIO init
    phy6222_hal::gpio::set_output(pins::LED_G);
    phy6222_hal::gpio::set_input(pins::BTN);
    led_off();

    // Boot signal: triple blink
    for _ in 0..3u8 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;

    // Radio + MAC
    let mac = Phy6222Mac::new();
    log::info!("[PHY6222] Radio ready");

    // Flash NV storage (last 2 sectors of 512KB flash)
    let mut nv = flash_nv::create_nv();
    log::info!("[PHY6222] Flash NV storage ready");

    // ZCL clusters
    let mut basic_cluster = BasicCluster::new(
        b"Zigbee-RS",
        b"PHY6222-Sensor",
        b"20260402",
        b"0.1.0",
    );
    basic_cluster.set_power_source(0x03); // Battery
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut power_cluster = PowerConfigCluster::new();
    let mut identify_cluster = IdentifyCluster::new();
    power_cluster.set_battery_size(4);     // AAA
    power_cluster.set_battery_quantity(2); // 2× AAA
    power_cluster.set_battery_rated_voltage(15); // 1.5V

    // Simulated sensor state
    let mut hum_tick: u32 = 0;

    // Build device (SED)
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("PHY6222-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0003) // Identify
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    // Restore previous network state from flash
    let restored = device.restore_state(&nv);
    if restored {
        log::info!("[PHY6222] Restored state from flash — will rejoin");
        device.user_action(UserAction::Rejoin);
    } else {
        log::info!("[PHY6222] No saved state — auto-joining…");
        device.user_action(UserAction::Join);
    }
    let mut clusters = [
        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
    ];
    if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
        if log_event(e) {
            device.save_state(&mut nv);
            log::info!("[PHY6222] Network state saved to flash");
        }
    }

    // Default reporting so device reports even before ZHA interview
    setup_default_reporting(&mut device);

    // Set initial sensor values for ZHA interview
    {
        let temp: i16 = 2250;
        temp_cluster.set_temperature(temp);
        hum_cluster.set_humidity(5000u16);

        // Real battery voltage via ADC
        let batt_mv = phy6222_hal::adc::read_battery_mv(phy6222_hal::adc::Channel::P11);
        let batt_pct = phy6222_hal::adc::mv_to_percent(batt_mv);
        power_cluster.set_battery_voltage((batt_mv / 100) as u8);
        power_cluster.set_battery_percentage(batt_pct * 2); // ZCL 0.5% units
        log::info!("[PHY6222] Initial: T=22.50°C H=50.00% Batt={}mV ({}%)", batt_mv, batt_pct);
    }

    // ── Main loop state ──
    let mut last_report = Instant::now();
    let mut fast_poll_until = if device.is_joined() {
        log::info!("[PHY6222] Fast poll ON ({}s)", FAST_POLL_DURATION_SECS);
        led_on();
        Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS)
    } else {
        Instant::now() // expires immediately
    };
    let mut last_rejoin_attempt = Instant::now();
    let mut rejoin_count: u8 = 0;
    let mut annce_retries_left: u8 = if device.is_joined() { 5 } else { 0 };
    let mut last_annce = Instant::now();
    let mut was_fast_polling = device.is_joined();
    let mut interview_done = false;
    let mut button_was_pressed = false;
    let mut needs_save = false;

    loop {
        let now = Instant::now();
        let in_fast_poll = now < fast_poll_until;
        let poll_ms = if in_fast_poll { FAST_POLL_MS } else { SLOW_POLL_SECS * 1000 };

        // Log transition from fast→slow poll
        if was_fast_polling && !in_fast_poll {
            let cfg = device.configured_cluster_count(1);
            log::info!("[PHY6222] Fast poll OFF — {}/{} clusters configured", cfg, EXPECTED_REPORT_CLUSTERS);
            was_fast_polling = false;
            if !interview_done {
                led_off();
            }
        } else if in_fast_poll {
            was_fast_polling = true;
        }

        // ── Button check ──
        let pressed = !phy6222_hal::gpio::read(pins::BTN); // active LOW
        if pressed && !button_was_pressed {
            // Check for long press (3s = factory reset)
            let mut held_long = false;
            let press_start = Instant::now();
            while !phy6222_hal::gpio::read(pins::BTN) { // still pressed
                if press_start.elapsed().as_secs() >= 3 {
                    held_long = true;
                    break;
                }
                Timer::after(Duration::from_millis(50)).await;
            }

            if held_long {
                log::info!("[PHY6222] FACTORY RESET");
                device.factory_reset(Some(&mut nv)).await;
                log::info!("[PHY6222] NV cleared — rebooting");
                for _ in 0..5u8 {
                    led_on();
                    Timer::after(Duration::from_millis(100)).await;
                    led_off();
                    Timer::after(Duration::from_millis(100)).await;
                }
                cortex_m::peripheral::SCB::sys_reset();
            } else {
                log::info!("[PHY6222] Button → {}", if device.is_joined() { "leave" } else { "join" });
                device.user_action(UserAction::Toggle);
                let mut cls = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                ];
                if let TickResult::Event(ref e) = device.tick(0, &mut cls).await {
                    match e {
                        StackEvent::Joined { .. } => {
                            log_event(e);
                            device.save_state(&mut nv);
                            log::info!("[PHY6222] State saved to flash");
                            fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                            annce_retries_left = 5;
                            last_annce = Instant::now();
                            interview_done = false;
                        }
                        StackEvent::Left => {
                            log_event(e);
                            device.factory_reset(Some(&mut nv)).await;
                            log::info!("[PHY6222] NV cleared");
                        }
                        _ => { log_event(e); }
                    }
                }
                Timer::after(Duration::from_millis(300)).await;
            }
        }
        button_was_pressed = pressed;

        // ── Sleep until next poll (radio off to save power) ──
        device.mac_mut().radio_sleep();
        Timer::after(Duration::from_millis(poll_ms)).await;
        device.mac_mut().radio_wake();

        // ── Poll parent for indirect frames (SED core) ──
        if device.is_joined() {
            for _poll_round in 0..4u8 {
                match device.poll().await {
                    Ok(Some(ind)) => {
                        let mut cls = [
                            ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                        ];
                        if let Some(ev) = device.process_incoming(&ind, &mut cls).await {
                            match &ev {
                                StackEvent::LeaveRequested => {
                                    log::info!("[PHY6222] Leave requested — erasing NV and rejoining");
                                    device.factory_reset(Some(&mut nv)).await;
                                    device.user_action(UserAction::Join);
                                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                    interview_done = false;
                                    annce_retries_left = 5;
                                    last_annce = Instant::now();
                                    led_on();
                                    break;
                                }
                                _ => {}
                            }
                            if log_event(&ev) {
                                fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                log::info!("[PHY6222] Fast poll ON ({}s)", FAST_POLL_DURATION_SECS);
                                needs_save = true;
                            }
                        }
                        // Check if ZHA completed interview
                        if !interview_done {
                            let cfg_count = device.configured_cluster_count(1);
                            if cfg_count >= EXPECTED_REPORT_CLUSTERS {
                                log::info!("[PHY6222] Interview done! {}/{} clusters", cfg_count, EXPECTED_REPORT_CLUSTERS);
                                fast_poll_until = Instant::now() + Duration::from_secs(5);
                                interview_done = true;
                                led_off();
                            }
                        }
                        // Tick to send queued ZCL responses
                        let mut cls2 = [
                            ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                        ];
                        let _ = device.tick(0, &mut cls2).await;
                    }
                    Ok(None) => break,
                    Err(_) => break,
                }
            }

            // ── Periodic sensor readings ──
            let now2 = Instant::now();
            let elapsed_s = now2.duration_since(last_report).as_secs();

            if elapsed_s >= REPORT_INTERVAL_SECS {
                last_report = now2;

                // Simulated temp/humidity (replace with I2C sensor when available)
                let temp_hundredths: i16 = 2250 + ((hum_tick % 50) as i16 - 25);
                hum_tick = hum_tick.wrapping_add(1);
                let hum_hundredths: u16 = 5000 + ((hum_tick % 100) as u16) * 10;
                temp_cluster.set_temperature(temp_hundredths);
                hum_cluster.set_humidity(hum_hundredths);
                log::info!(
                    "[PHY6222] T={}.{:02}°C H={}.{:02}%",
                    temp_hundredths / 100,
                    (temp_hundredths % 100).unsigned_abs(),
                    hum_hundredths / 100,
                    hum_hundredths % 100,
                );

                // Real battery voltage via ADC
                let batt_mv = phy6222_hal::adc::read_battery_mv(phy6222_hal::adc::Channel::P11);
                let batt_pct = phy6222_hal::adc::mv_to_percent(batt_mv);
                power_cluster.set_battery_voltage((batt_mv / 100) as u8);
                power_cluster.set_battery_percentage(batt_pct * 2);
                log::info!("[PHY6222] Battery: {}mV ({}%)", batt_mv, batt_pct);
            }

            // Tick the runtime
            let tick_elapsed = elapsed_s.min(60) as u16;
            let mut clusters = [
                ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
            ];
            if let TickResult::Event(ref e) = device.tick(tick_elapsed, &mut clusters).await {
                if log_event(e) {
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                }
            }

            // Identify LED blink
            identify_cluster.tick(tick_elapsed);
            if identify_cluster.is_identifying() {
                let on = phy6222_hal::gpio::read(pins::LED_G);
                phy6222_hal::gpio::write(pins::LED_G, !on);
            }

            // Device_annce retry
            if annce_retries_left > 0 && now2.duration_since(last_annce).as_secs() >= 8 {
                annce_retries_left -= 1;
                last_annce = now2;
                log::info!("[PHY6222] Device_annce retry ({} left)", annce_retries_left);
                let _ = device.send_device_annce().await;
            }

            // Deferred save after join events from process_incoming
            if needs_save {
                needs_save = false;
                device.save_state(&mut nv);
                log::info!("[PHY6222] State saved to flash (deferred)");
            }
        } else {
            // ── Not joined — blink and auto-retry ──
            let now2 = Instant::now();
            if now2.duration_since(last_rejoin_attempt).as_secs() >= 1 {
                // Double blink
                led_on();
                Timer::after(Duration::from_millis(80)).await;
                led_off();
                Timer::after(Duration::from_millis(120)).await;
                led_on();
                Timer::after(Duration::from_millis(80)).await;
                led_off();
            }

            if now2.duration_since(last_rejoin_attempt).as_secs() >= 15 {
                rejoin_count = rejoin_count.wrapping_add(1);
                last_rejoin_attempt = Instant::now();
                log::info!("[PHY6222] Not joined — retrying (attempt {})…", rejoin_count);
                device.user_action(UserAction::Join);
                let mut cls = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                ];
                let _ = device.tick(0, &mut cls).await;
                if device.is_joined() {
                    log::info!("[PHY6222] Joined! addr=0x{:04X}", device.short_address());
                    led_on();
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    annce_retries_left = 5;
                    last_annce = Instant::now();
                    interview_done = false;
                    device.save_state(&mut nv);
                    log::info!("[PHY6222] State saved to flash");
                }
            }
        }
    }
}

/// Log stack events. Returns true on join event.
fn log_event(event: &StackEvent) -> bool {
    match event {
        StackEvent::Joined { short_address, channel, pan_id } => {
            led_on();
            log::info!(
                "[PHY6222] Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id
            );
            true
        }
        StackEvent::Left => {
            led_off();
            log::info!("[PHY6222] Left network");
            false
        }
        StackEvent::ReportSent => { log::info!("[PHY6222] Report sent"); false }
        StackEvent::LeaveRequested => {
            led_on();
            log::info!("[PHY6222] Leave requested by coordinator");
            false
        }
        StackEvent::CommissioningComplete { success } => {
            log::info!("[PHY6222] Commissioning: {}", if *success { "ok" } else { "failed" });
            false
        }
        _ => { log::info!("[PHY6222] Stack event"); false }
    }
}

/// Configure default reporting intervals so device reports even before ZHA interview.
fn setup_default_reporting(device: &mut ZigbeeDevice<Phy6222Mac>) {
    use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
    use zigbee_zcl::data_types::ZclDataType;

    let configs = [
        (0x0402u16, 0x0000u16, ZclDataType::I16),   // Temperature
        (0x0405, 0x0000, ZclDataType::U16),          // Humidity
        (0x0001, 0x0021, ZclDataType::U8),           // Battery %
    ];

    for (cluster_id, attr_id, data_type) in configs {
        let (min, max) = if cluster_id == 0x0001 { (300, 3600) } else { (60, 300) };
        let _ = device.reporting_mut().configure_for_cluster(
            1, cluster_id,
            ReportingConfig {
                direction: ReportDirection::Send,
                attribute_id: zigbee_zcl::AttributeId(attr_id),
                data_type,
                min_interval: min,
                max_interval: max,
                reportable_change: None,
            },
        );
    }
    log::info!("[PHY6222] Default reporting configured");
}
