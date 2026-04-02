//! CC2340R5 Zigbee Temperature/Humidity Sensor Example
//!
//! A complete Zigbee End Device firmware for the TI CC2340R5 LaunchPad
//! (LP-EM-CC2340R5). Reports simulated temperature and humidity readings
//! to a Zigbee coordinator every 30 seconds.
//!
//! # Hardware
//! - CC2340R5 LaunchPad (ARM Cortex-M0+, 512KB Flash, 36KB RAM)
//! - Built-in buttons: BTN1 (DIO13) = join/leave, BTN2 (DIO14) = identify
//! - Built-in LEDs: LED1 (DIO7) = network status
//!
//! # Build
//! ```bash
//! # Check only (CI — no TI SDK needed)
//! cargo check --release
//!
//! # Full build (requires TI SimpleLink SDK)
//! CC2340_SDK_DIR=/path/to/simplelink_lowpower_f3_sdk cargo build --release
//! ```

#![no_std]
#![no_main]

#[cfg(feature = "stubs")]
mod stubs;

use cortex_m as _;
use panic_halt as _;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::cc2340::Cc2340Mac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 30;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 10;
const FAST_POLL_DURATION_SECS: u64 = 120;
const EXPECTED_REPORT_CLUSTERS: usize = 3;

// ── CC2340R5 hardware constants ─────────────────────────────────

/// DIO pin assignments for LP-EM-CC2340R5
mod pins {
    pub const BTN1: u8 = 13; // Join/Leave button
    pub const BTN2: u8 = 14; // Identify button
    pub const LED1: u8 = 7; // Network status LED
    pub const LED2: u8 = 6; // Activity LED
}

/// CC2340R5 GPIO register base
const GPIO_BASE: u32 = 0x4000_6000;

// ── Minimal GPIO helpers ────────────────────────────────────────

fn gpio_set_output(pin: u8) {
    unsafe {
        let doe_reg = (GPIO_BASE + 0x0C) as *mut u32; // DOUT_EN
        let val = core::ptr::read_volatile(doe_reg);
        core::ptr::write_volatile(doe_reg, val | (1 << pin));
    }
}

fn gpio_write(pin: u8, high: bool) {
    unsafe {
        let dout_reg = (GPIO_BASE + 0x08) as *mut u32; // DOUT
        let val = core::ptr::read_volatile(dout_reg);
        if high {
            core::ptr::write_volatile(dout_reg, val | (1 << pin));
        } else {
            core::ptr::write_volatile(dout_reg, val & !(1 << pin));
        }
    }
}

fn gpio_read(pin: u8) -> bool {
    unsafe {
        let din_reg = (GPIO_BASE + 0x04) as *const u32; // DIN
        let val = core::ptr::read_volatile(din_reg);
        (val >> pin) & 1 == 1
    }
}

fn led_on() {
    gpio_write(pins::LED1, true);
}

fn led_off() {
    gpio_write(pins::LED1, false);
}

// ── Logging ─────────────────────────────────────────────────────
// On Cortex-M0+ (no native CAS atomics), log::set_logger() is unavailable.
// log::info!() etc. still compile — they become no-ops without a registered logger.
// For real debug output, use a probe-rs RTT or SWD semihosting approach.

// ── Time driver stub ────────────────────────────────────────────
// CC2340R5 has SysTick and RTC — a proper Embassy time driver would
// use one of these. For compilation purposes, we provide a minimal stub.
// A production firmware would use embassy-cc2340 (when available) or
// implement a full RTC-based time driver.

struct Cc2340TimeDriver;

impl Cc2340TimeDriver {
    const fn new() -> Self {
        Self
    }
}

impl embassy_time_driver::Driver for Cc2340TimeDriver {
    fn now(&self) -> u64 {
        // Read from SysTick or RTC in real implementation
        // This stub returns 0 — sufficient for cargo check
        0
    }

    fn schedule_wake(&self, _at: u64, _waker: &core::task::Waker) {
        // Set alarm in RTC/SysTick in real implementation
    }
}

embassy_time_driver::time_driver_impl!(static TIME_DRIVER: Cc2340TimeDriver = Cc2340TimeDriver::new());

// ── Main entry point ────────────────────────────────────────────

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Logging: no-op on Cortex-M0+ (no set_logger), use RTT probe for debug
    log::info!("[CC2340] Zigbee Sensor starting...");

    // Initialize GPIO for LEDs and buttons
    gpio_set_output(pins::LED1);
    gpio_set_output(pins::LED2);
    led_off();

    // Blink LED to show we're alive
    for _ in 0..3 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }

    // Create MAC driver
    let mac = Cc2340Mac::new();

    // ZCL cluster instances
    let mut basic_cluster = BasicCluster::new(b"Zigbee-RS", b"CC2340-Sensor", b"20260402", b"0.1.0");
    basic_cluster.set_power_source(0x03); // Battery
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut power_cluster = PowerConfigCluster::new();

    // Build Zigbee device (SED architecture)
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("CC2340-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    // Auto-join on boot
    log::info!("[CC2340] Auto-joining network…");
    device.user_action(UserAction::Join);
    let mut clusters = [
        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
    ];
    let _ = device.tick(0, &mut clusters).await;

    // Set initial sensor values
    temp_cluster.set_temperature(2250);
    hum_cluster.set_humidity(5000u16);
    power_cluster.set_battery_voltage(30);
    power_cluster.set_battery_percentage(200);

    let mut button_was_pressed = false;
    let mut last_report = Instant::now();
    let mut fast_poll_until = if device.is_joined() {
        Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS)
    } else {
        Instant::now()
    };
    let mut annce_retries_left: u8 = if device.is_joined() { 5 } else { 0 };
    let mut last_annce = Instant::now();
    let mut interview_done = false;
    let mut hum_tick: u32 = 0;
    let mut last_rejoin_attempt = Instant::now();

    loop {
        let now = Instant::now();
        let in_fast_poll = now < fast_poll_until;
        let poll_ms = if in_fast_poll { FAST_POLL_MS } else { SLOW_POLL_SECS * 1000 };

        // ── Button handling (edge detection) ─────────────────
        let pressed = !gpio_read(pins::BTN1); // Active low
        if pressed && !button_was_pressed {
            if device.is_joined() {
                log::info!("[CC2340] Button → leaving network");
            } else {
                log::info!("[CC2340] Button → joining network");
            }
            device.user_action(UserAction::Toggle);
            Timer::after(Duration::from_millis(300)).await; // debounce
        }
        button_was_pressed = pressed;

        Timer::after(Duration::from_millis(poll_ms)).await;

        if device.is_joined() {
            // Poll parent for indirect frames
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
                            if matches!(ev, StackEvent::Joined { .. }) {
                                fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                            }
                        }
                        if !interview_done && device.configured_cluster_count(1) >= EXPECTED_REPORT_CLUSTERS {
                            interview_done = true;
                            fast_poll_until = Instant::now() + Duration::from_secs(5);
                            log::info!("[CC2340] Interview done — ending fast poll");
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
                hum_tick = hum_tick.wrapping_add(1);
                let temp: i16 = 2250 + ((hum_tick % 50) as i16 - 25);
                let hum: u16 = 5000 + ((hum_tick % 100) as u16) * 10;
                temp_cluster.set_temperature(temp);
                hum_cluster.set_humidity(hum);
                log::info!(
                    "[CC2340] T={}.{:02}°C H={}.{:02}%",
                    temp / 100,
                    (temp % 100).unsigned_abs(),
                    hum / 100,
                    hum % 100,
                );
            }

            // Tick runtime
            let tick_elapsed = elapsed_s.min(60) as u16;
            let mut clusters = [
                ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                ClusterRef { endpoint: 1, cluster: &mut power_cluster },
            ];
            let _ = device.tick(tick_elapsed, &mut clusters).await;

            // Device_annce retry (5×, 8s apart)
            if annce_retries_left > 0 && now2.duration_since(last_annce).as_secs() >= 8 {
                annce_retries_left -= 1;
                last_annce = now2;
                let _ = device.send_device_annce().await;
                log::info!("[CC2340] Device_annce retry ({} left)", annce_retries_left);
            }
        } else {
            // Not joined — blink LED, auto-retry every 15s
            led_on();
            Timer::after(Duration::from_millis(80)).await;
            led_off();

            let now2 = Instant::now();
            if now2.duration_since(last_rejoin_attempt).as_secs() >= 15 {
                last_rejoin_attempt = Instant::now();
                device.user_action(UserAction::Join);
                let mut cls = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                let _ = device.tick(0, &mut cls).await;
                if device.is_joined() {
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    annce_retries_left = 5;
                    last_annce = Instant::now();
                    interview_done = false;
                    log::info!("[CC2340] Joined!");
                }
            }
        }
    }
}
