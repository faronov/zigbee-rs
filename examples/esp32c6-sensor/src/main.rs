//! # ESP32-C6 Zigbee Sensor (SED)
//!
//! Full-featured Zigbee 3.0 sleepy end device for ESP32-C6.
//! Uses the built-in IEEE 802.15.4 radio via `esp-radio`.
//!
//! # Features
//! - Auto-join on boot
//! - Sleepy End Device: poll parent for indirect frames
//! - Fast poll (250ms) during ZHA interview, slow poll (10s) normal
//! - Device_annce retries for reliable coordinator discovery
//! - Button: BOOT (GPIO9) — short=toggle, long=factory reset
//!
//! # Build & flash
//! ```bash
//! cargo build --release
//! espflash flash --monitor target/riscv32imac-unknown-none-elf/release/esp32c6-sensor
//! ```

#![no_std]
#![no_main]

extern crate alloc;

mod time_driver;
mod flash_nv;

use esp_backtrace as _;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::tsens::{TemperatureSensor, Config as TsensConfig};

use embassy_futures::block_on;
use embassy_time::{Duration, Instant, Timer};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

// Bridge `log` crate → esp_println so stack-internal log::info! appears on serial
struct EspLogger;
impl log::Log for EspLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool { true }
    fn log(&self, record: &log::Record) {
        esp_println::println!("[{}] {}", record.level(), record.args());
    }
    fn flush(&self) {}
}
static LOGGER: EspLogger = EspLogger;

const REPORT_INTERVAL_SECS: u64 = 30;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 10;
const FAST_POLL_DURATION_SECS: u64 = 120;
const EXPECTED_REPORT_CLUSTERS: usize = 3;

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize heap — required by zigbee-mac's alloc feature  
    esp_alloc::heap_allocator!(size: 32768);

    // Initialize log → esp_println bridge
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);

    esp_println::println!("[ESP32-C6] Booting...");

    // Start embassy time driver
    time_driver::init();

    esp_println::println!("[ESP32-C6] Zigbee Sensor starting");

    // BOOT button (GPIO9, active low)
    let button = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );

    // LED on GPIO8 (active low on most devkits)
    let mut led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());

    // Boot signal: triple blink
    block_on(async {
        for _ in 0..3u8 {
            led.set_low();
            Timer::after(Duration::from_millis(100)).await;
            led.set_high();
            Timer::after(Duration::from_millis(100)).await;
        }
    });

    // IEEE 802.15.4 radio
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(peripherals.IEEE802154);
    let config = esp_radio::ieee802154::Config::default();
    let mac = zigbee_mac::esp::EspMac::new(ieee802154, config);
    esp_println::println!("[ESP32-C6] Radio ready");

    // On-chip temperature sensor
    let temp_sensor = TemperatureSensor::new(peripherals.TSENS, TsensConfig::default())
        .expect("temp sensor init failed");

    // ZCL clusters
    let mut basic_cluster = BasicCluster::new(
        b"Zigbee-RS",
        b"ESP32-C6-Sensor",
        b"20260403",
        b"0.1.0",
    );
    basic_cluster.set_power_source(0x04); // DC power (USB-powered devkit)
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut power_cluster = PowerConfigCluster::new();

    let mut hum_tick: u32 = 0;

    // Build device (SED)
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("ESP32-C6-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    // Initial sensor values
    {
        let raw_temp = temp_sensor.get_temperature();
        // Convert to centidegrees: (raw * 0.4386 - offset*27.88 - 20.52) * 100
        // Integer: (raw * 4386 - offset * 278800 - 205200) / 100
        let temp_centi = ((raw_temp.raw_value as i32) * 4386
            - (raw_temp.offset as i32) * 278800
            - 205200) / 100;
        temp_cluster.set_temperature(temp_centi as i16);
        hum_cluster.set_humidity(5000u16); // No humidity sensor — fixed 50%
        power_cluster.set_battery_voltage(33); // USB powered = 3.3V
        power_cluster.set_battery_percentage(200); // 100%
        esp_println::println!("[ESP32-C6] Temp: {}.{:02}°C (on-chip)",
            temp_centi / 100, (temp_centi % 100).unsigned_abs());
    }

    // Flash NV storage for network persistence
    let mut nv = flash_nv::create_nv();
    esp_println::println!("[ESP32-C6] Flash NV storage ready");

    // Run the async SED loop synchronously via block_on
    block_on(async {
        // Restore previous network state or auto-join
        let restored = device.restore_state(&nv);
        if restored {
            esp_println::println!("[ESP32-C6] Restored state — will rejoin");
            device.user_action(UserAction::Rejoin);
        } else {
            esp_println::println!("[ESP32-C6] No saved state — auto-joining…");
            device.user_action(UserAction::Join);
        }
        let mut clusters = [
            ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
            ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
            ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
            ClusterRef { endpoint: 1, cluster: &mut power_cluster },
        ];
        if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
            if log_event(e) {
                device.save_state(&mut nv);
                esp_println::println!("[ESP32-C6] State saved to flash");
            }
        }

        let mut last_report = Instant::now();
        let mut fast_poll_until = if device.is_joined() {
            led.set_low();
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
                    esp_println::println!("[ESP32-C6] FACTORY RESET");
                    device.factory_reset(Some(&mut nv)).await;
                    esp_println::println!("[ESP32-C6] NV cleared — rebooting");
                    for _ in 0..5u8 {
                        led.set_low();
                        Timer::after(Duration::from_millis(100)).await;
                        led.set_high();
                        Timer::after(Duration::from_millis(100)).await;
                    }
                    esp_hal::system::software_reset();
                } else {
                    esp_println::println!("[ESP32-C6] Button → {}",
                        if device.is_joined() { "leave" } else { "join" });
                    device.user_action(UserAction::Toggle);
                    let mut cls = [
                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
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

            // Sleep
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
                            ];
                            if let Some(ev) = device.process_incoming(&ind, &mut cls).await {
                                if log_event(&ev) {
                                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                }
                            }
                            if !interview_done && device.configured_cluster_count(1) >= EXPECTED_REPORT_CLUSTERS {
                                interview_done = true;
                                fast_poll_until = Instant::now() + Duration::from_secs(5);
                                led.set_high();
                                esp_println::println!("[ESP32-C6] Interview done!");
                            }
                            let mut cls2 = [
                                ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                                ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                                ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                                ClusterRef { endpoint: 1, cluster: &mut power_cluster },
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
                    let raw_temp = temp_sensor.get_temperature();
                    let temp_centi = ((raw_temp.raw_value as i32) * 4386
                        - (raw_temp.offset as i32) * 278800
                        - 205200) / 100;
                    hum_tick = hum_tick.wrapping_add(1);
                    let hum: u16 = 5000 + ((hum_tick % 100) as u16) * 10;
                    temp_cluster.set_temperature(temp_centi as i16);
                    hum_cluster.set_humidity(hum);
                    esp_println::println!("[ESP32-C6] T={}.{:02}°C H={}.{:02}%",
                        temp_centi / 100, (temp_centi % 100).unsigned_abs(), hum / 100, hum % 100);
                }

                let tick_elapsed = elapsed_s.min(60) as u16;
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                let _ = device.tick(tick_elapsed, &mut clusters).await;

                // Device_annce retry
                if annce_retries_left > 0 && now2.duration_since(last_annce).as_secs() >= 8 {
                    annce_retries_left -= 1;
                    last_annce = now2;
                    let _ = device.send_device_annce().await;
                }
            } else {
                // Not joined — blink and retry
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
                    esp_println::println!("[ESP32-C6] Retrying join ({})…", rejoin_count);
                    device.user_action(UserAction::Join);
                    let mut cls = [
                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                    ];
                    let _ = device.tick(0, &mut cls).await;
                    if device.is_joined() {
                        esp_println::println!("[ESP32-C6] Joined! addr=0x{:04X}", device.short_address());
                        led.set_low();
                        fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                        annce_retries_left = 5;
                        last_annce = Instant::now();
                        interview_done = false;
                        device.save_state(&mut nv);
                        esp_println::println!("[ESP32-C6] State saved to flash");
                    }
                }
            }
        }
    })
}

fn log_event(event: &StackEvent) -> bool {
    match event {
        StackEvent::Joined { short_address, channel, pan_id } => {
            esp_println::println!("[ESP32-C6] Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id);
            true
        }
        StackEvent::Left => {
            esp_println::println!("[ESP32-C6] Left network");
            false
        }
        StackEvent::ReportSent => {
            esp_println::println!("[ESP32-C6] Report sent");
            false
        }
        StackEvent::CommissioningComplete { success } => {
            esp_println::println!("[ESP32-C6] Commissioning: {}",
                if *success { "ok" } else { "failed" });
            false
        }
        _ => false,
    }
}
