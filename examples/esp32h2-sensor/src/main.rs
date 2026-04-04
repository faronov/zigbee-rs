//! # ESP32-H2 Zigbee Sensor (SED)
//!
//! Full-featured Zigbee 3.0 sleepy end device for ESP32-H2.
//! Uses the built-in IEEE 802.15.4 radio via `esp-radio`.
//!
//! # Features
//! - Auto-join on boot
//! - Sleepy End Device: poll parent for indirect frames
//! - Fast poll (250ms) during ZHA interview, slow poll (10s) normal
//! - Device_annce retries for reliable coordinator discovery
//! - NWK Leave handler: auto-rejoin when coordinator sends Leave
//! - Default reporting: temp/hum/battery reported without ZHA interview
//! - Button: BOOT (GPIO9) — short=toggle, long=factory reset
//!
//! # Build
//! ```bash
//! cargo build --release
//! espflash flash --monitor target/riscv32imac-unknown-none-elf/release/esp32h2-sensor
//! ```

#![no_std]
#![no_main]

extern crate alloc;

mod time_driver;
mod flash_nv;

use esp_backtrace as _;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};


use embassy_futures::block_on;
use embassy_time::{Duration, Instant, Timer};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

// Bridge `log` crate → esp_println
struct EspLogger;
impl log::Log for EspLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool { true }
    fn log(&self, record: &log::Record) {
        esp_println::println!("[{}] {}", record.level(), record.args());
    }
    fn flush(&self) {}
}
static LOGGER: EspLogger = EspLogger;

const REPORT_INTERVAL_SECS: u64 = 60;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 30;
const FAST_POLL_DURATION_SECS: u64 = 120;
const EXPECTED_REPORT_CLUSTERS: usize = 3;

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize heap
    esp_alloc::heap_allocator!(size: 32768);

    // Initialize log
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);

    // Start embassy time driver
    time_driver::init();

    esp_println::println!("[ESP32-H2] Zigbee Sensor starting");

    // BOOT button (GPIO9, active low with internal pull-up)
    let button = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );

    // LED on GPIO8 (active low on most ESP32-H2 boards)
    let mut led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());

    // Boot signal: triple blink
    for _ in 0..3u8 {
        led.set_low();
        // Busy-wait outside async context
        for _ in 0..100_000u32 { core::hint::spin_loop(); }
        led.set_high();
        for _ in 0..100_000u32 { core::hint::spin_loop(); }
    }

    // IEEE 802.15.4 radio
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(peripherals.IEEE802154);
    let config = esp_radio::ieee802154::Config::default();
    let mac = zigbee_mac::esp::EspMac::new(ieee802154, config);
    esp_println::println!("[ESP32-H2] Radio ready");

    // ZCL clusters
    let mut basic_cluster = BasicCluster::new(
        b"Zigbee-RS",
        b"ESP32-H2-Sensor",
        b"20260403",
        b"0.1.0",
    );
    basic_cluster.set_power_source(0x03); // Battery
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut power_cluster = PowerConfigCluster::new();
    power_cluster.set_battery_size(4);
    power_cluster.set_battery_quantity(2);
    power_cluster.set_battery_rated_voltage(15);
    let mut identify_cluster = IdentifyCluster::new();

    let mut hum_tick: u32 = 0;

    // Build device (SED)
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("ESP32-H2-Sensor")
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

    // Initial sensor values
    temp_cluster.set_temperature(2250);
    hum_cluster.set_humidity(5000u16);
    power_cluster.set_battery_voltage(33);
    power_cluster.set_battery_percentage(200);

    // Run the async SED loop synchronously via block_on
    block_on(async {
        // Flash NV storage
        let mut nv = flash_nv::create_nv();
        esp_println::println!("[ESP32-H2] Flash NV storage ready");

        // Restore previous network state or auto-join
        let restored = device.restore_state(&nv);
        if restored {
            esp_println::println!("[ESP32-H2] Restored state — will rejoin");
            device.user_action(UserAction::Rejoin);
        } else {
            esp_println::println!("[ESP32-H2] No saved state — auto-joining…");
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
                esp_println::println!("[ESP32-H2] State saved to flash");
            }
        }

        // Default reporting so device reports even before ZHA interview
        setup_default_reporting(&mut device);

        // Main loop state
        let mut last_report = Instant::now();
        let mut fast_poll_until = if device.is_joined() {
            led.set_low(); // ON
            Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS)
        } else {
            Instant::now()
        };
        let mut last_rejoin_attempt = Instant::now();
        let mut rejoin_count: u8 = 0;
        let mut annce_retries_left: u8 = if device.is_joined() { 5 } else { 0 };
        let mut last_annce = Instant::now();
        let mut interview_done = false;
        let mut button_was_pressed = false;

        loop {
            let now = Instant::now();
            let in_fast_poll = now < fast_poll_until;
            let poll_ms = if in_fast_poll { FAST_POLL_MS } else { SLOW_POLL_SECS * 1000 };

            // Button check
            let pressed = button.is_low();
            if pressed && !button_was_pressed {
                let press_start = Instant::now();
                let mut held_long = false;
                while button.is_low() {
                    if press_start.elapsed().as_secs() >= 3 {
                        held_long = true;
                        break;
                    }
                    Timer::after(Duration::from_millis(50)).await;
                }

                if held_long {
                    esp_println::println!("[ESP32-H2] FACTORY RESET");
                    for _ in 0..5u8 {
                        led.set_low();
                        Timer::after(Duration::from_millis(100)).await;
                        led.set_high();
                        Timer::after(Duration::from_millis(100)).await;
                    }
                    esp_hal::system::software_reset();
                } else {
                    esp_println::println!("[ESP32-H2] Button → {}",
                        if device.is_joined() { "leave" } else { "join" });
                    device.user_action(UserAction::Toggle);
                    let mut cls = [
                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                    ];
                    if let TickResult::Event(ref e) = device.tick(0, &mut cls).await {
                        if log_event(e) {
                            device.save_state(&mut nv);
                            fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                            annce_retries_left = 5;
                            last_annce = Instant::now();
                            interview_done = false;
                        }
                    }
                    Timer::after(Duration::from_millis(300)).await;
                }
            }
            button_was_pressed = pressed;

            // Sleep until next poll
            Timer::after(Duration::from_millis(poll_ms)).await;

            // Poll parent (SED core)
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
                                        esp_println::println!("[ESP32-H2] Leave requested — erasing NV and rejoining");
                                        device.factory_reset(Some(&mut nv)).await;
                                        device.user_action(UserAction::Join);
                                        fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                        interview_done = false;
                                        break;
                                    }
                                    _ => {}
                                }
                                if log_event(&ev) {
                                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                }
                            }
                            if !interview_done && device.configured_cluster_count(1) >= EXPECTED_REPORT_CLUSTERS {
                                interview_done = true;
                                fast_poll_until = Instant::now() + Duration::from_secs(5);
                                led.set_high(); // OFF
                                esp_println::println!("[ESP32-H2] Interview done!");
                            }
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

                // Periodic sensor update
                let now2 = Instant::now();
                let elapsed_s = now2.duration_since(last_report).as_secs();
                if elapsed_s >= REPORT_INTERVAL_SECS {
                    last_report = now2;
                    hum_tick = hum_tick.wrapping_add(1);
                    let temp: i16 = 2250 + ((hum_tick % 50) as i16 - 25);
                    let hum: u16 = 5000 + ((hum_tick % 100) as u16) * 10;
                    temp_cluster.set_temperature(temp);
                    hum_cluster.set_humidity(hum);
                    esp_println::println!("[ESP32-H2] T={}.{:02}°C H={}.{:02}%",
                        temp / 100, (temp % 100).unsigned_abs(), hum / 100, hum % 100);
                }

                // Tick runtime
                let tick_elapsed = elapsed_s.min(60) as u16;
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                ];
                let _ = device.tick(tick_elapsed, &mut clusters).await;

                // Identify LED blink
                identify_cluster.tick(tick_elapsed);
                if identify_cluster.is_identifying() {
                    led.toggle();
                }

                // Device_annce retry
                if annce_retries_left > 0 && now2.duration_since(last_annce).as_secs() >= 8 {
                    annce_retries_left -= 1;
                    last_annce = now2;
                    let _ = device.send_device_annce().await;
                }
            } else {
                // Not joined — blink and auto-retry
                let now2 = Instant::now();
                if now2.duration_since(last_rejoin_attempt).as_secs() >= 1 {
                    led.set_low();
                    Timer::after(Duration::from_millis(80)).await;
                    led.set_high();
                    Timer::after(Duration::from_millis(120)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(80)).await;
                    led.set_high();
                }

                if now2.duration_since(last_rejoin_attempt).as_secs() >= 15 {
                    rejoin_count = rejoin_count.wrapping_add(1);
                    last_rejoin_attempt = Instant::now();
                    esp_println::println!("[ESP32-H2] Retrying join (attempt {})…", rejoin_count);
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
                        esp_println::println!("[ESP32-H2] Joined! addr=0x{:04X}", device.short_address());
                        led.set_low();
                        fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                        annce_retries_left = 5;
                        last_annce = Instant::now();
                        interview_done = false;
                        device.save_state(&mut nv);
                        esp_println::println!("[ESP32-H2] State saved to flash");
                    }
                }
            }
        }
    })
}

fn log_event(event: &StackEvent) -> bool {
    match event {
        StackEvent::Joined { short_address, channel, pan_id } => {
            esp_println::println!("[ESP32-H2] Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id);
            true
        }
        StackEvent::Left => {
            esp_println::println!("[ESP32-H2] Left network");
            false
        }
        StackEvent::ReportSent => {
            esp_println::println!("[ESP32-H2] Report sent");
            false
        }
        StackEvent::LeaveRequested => {
            esp_println::println!("[ESP32-H2] Leave requested by coordinator");
            false
        }
        StackEvent::CommissioningComplete { success } => {
            esp_println::println!("[ESP32-H2] Commissioning: {}", if *success { "ok" } else { "failed" });
            false
        }
        _ => false,
    }
}

/// Configure default reporting with change thresholds to suppress unnecessary TX.
fn setup_default_reporting<M: zigbee_mac::MacDriver>(device: &mut ZigbeeDevice<M>) {
    use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
    use zigbee_zcl::data_types::{ZclDataType, ZclValue};

    let _ = device.reporting_mut().configure_for_cluster(
        1, 0x0402,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::AttributeId(0x0000),
            data_type: ZclDataType::I16,
            min_interval: 60,
            max_interval: 300,
            reportable_change: Some(ZclValue::I16(50)),
        },
    );
    let _ = device.reporting_mut().configure_for_cluster(
        1, 0x0405,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::AttributeId(0x0000),
            data_type: ZclDataType::U16,
            min_interval: 60,
            max_interval: 300,
            reportable_change: Some(ZclValue::U16(100)),
        },
    );
    let _ = device.reporting_mut().configure_for_cluster(
        1, 0x0001,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::AttributeId(0x0021),
            data_type: ZclDataType::U8,
            min_interval: 300,
            max_interval: 3600,
            reportable_change: Some(ZclValue::U8(4)),
        },
    );
    esp_println::println!("[ESP32-H2] Default reporting configured (with change thresholds)");
}
