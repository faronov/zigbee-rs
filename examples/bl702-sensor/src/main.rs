//! # BL702 Zigbee Temperature Sensor
//!
//! A `no_std` firmware for the BL702 (Bouffalo Lab, RISC-V),
//! implementing a Zigbee 3.0 end device that exposes Temperature
//! Measurement (0x0402) and Relative Humidity (0x0405) clusters.
//!
//! ## Hardware
//! - BL702 module (XT-ZB1, DT-BL10, Pine64 Pinenut, or BL706 devboard)
//! - Built-in IEEE 802.15.4 + BLE 5.0 radio
//! - Boot button (GPIO8 on most modules): join/leave network
//!
//! ## Radio driver
//! The BL702 backend uses FFI bindings to Bouffalo's `lmac154` C library
//! (`liblmac154.a`) for 802.15.4 radio access, with interrupt-driven
//! TX/RX via Embassy signals.
//!
//! ## Building
//! ```bash
//! cd examples/bl702-sensor
//! LMAC154_LIB_DIR=/path/to/lib cargo build --release
//! ```
//!
//! ## Embassy Time Driver
//! Since there is no `embassy-bl702` HAL yet, this example provides a
//! minimal Embassy time driver using the BL702 TIMER_CH0 (32-bit
//! match-compare timer) running at 1 MHz (FCLK/32 prescaler from 32 MHz).

#![no_std]
#![no_main]

#[cfg(feature = "stubs")]
mod stubs;

// Always include HAL implementations for vendor library dependencies
// (delay, IRQ, GPIO, memcpy). These are needed even with real vendor libs.
mod hal;

use panic_halt as _;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::bl702::Bl702Mac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};

const REPORT_INTERVAL_SECS: u64 = 30;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 10;
const FAST_POLL_DURATION_SECS: u64 = 120;
const EXPECTED_REPORT_CLUSTERS: usize = 3;

// ── Minimal Embassy time driver using BL702 TIMER_CH0 ──────────
mod time_driver {
    use embassy_time_driver::Driver;
    use portable_atomic::{AtomicU64, Ordering};

    struct Bl702TimeDriver {
        alarm_at: AtomicU64,
    }

    // BL702 TIMER registers (TIMER_CH0)
    const TIMER_BASE: usize = 0x4000_A500;
    const TCCR: *mut u32 = (TIMER_BASE + 0x00) as *mut u32; // Timer Clock Config
    const TMSR_0: *mut u32 = (TIMER_BASE + 0x28) as *mut u32; // Match Status
    const TCR_0: *mut u32 = (TIMER_BASE + 0x2C) as *mut u32; // Counter
    const TMR_0: *mut u32 = (TIMER_BASE + 0x10) as *mut u32; // Match Register
    const TIER_0: *mut u32 = (TIMER_BASE + 0x44) as *mut u32; // Interrupt Enable
    const TICR_0: *mut u32 = (TIMER_BASE + 0x48) as *mut u32; // Interrupt Clear
    const TCER: *mut u32 = (TIMER_BASE + 0x30) as *mut u32; // Counter Enable

    /// Initialize TIMER_CH0 as a free-running 1 MHz counter.
    pub fn init() {
        unsafe {
            // Prescaler: FCLK(32 MHz) / 32 = 1 MHz tick
            core::ptr::write_volatile(TCCR, 0x1F); // div=32, FCLK source
            // Match at max value (free-running)
            core::ptr::write_volatile(TMR_0, 0xFFFF_FFFF);
            // Clear any pending interrupt
            core::ptr::write_volatile(TICR_0, 0x07);
            // Enable match0 interrupt
            core::ptr::write_volatile(TIER_0, 0x01);
            // Enable counter
            core::ptr::write_volatile(TCER, 0x01);
        }
    }

    fn now_ticks() -> u64 {
        unsafe { core::ptr::read_volatile(TCR_0) as u64 }
    }

    impl Driver for Bl702TimeDriver {
        fn now(&self) -> u64 {
            now_ticks()
        }

        fn schedule_wake(&self, at: u64, _waker: &core::task::Waker) {
            self.alarm_at.store(at, Ordering::Release);
            unsafe {
                core::ptr::write_volatile(TMR_0, at as u32);
                // Clear + re-enable
                core::ptr::write_volatile(TICR_0, 0x01);
                core::ptr::write_volatile(TIER_0, 0x01);
            }
        }
    }

    embassy_time_driver::time_driver_impl!(static DRIVER: Bl702TimeDriver = Bl702TimeDriver {
        alarm_at: AtomicU64::new(u64::MAX),
    });
}

// ── BL702 GPIO helper for button ───────────────────────────────
mod gpio {
    const GLB_BASE: usize = 0x4000_0000;

    /// Read GPIO input value (0 or 1).
    pub fn read_input(pin: u8) -> bool {
        let reg = (GLB_BASE + 0x180) as *const u32;
        let val = unsafe { core::ptr::read_volatile(reg) };
        (val >> pin) & 1 != 0
    }

    /// Configure a GPIO pin as input with pull-up.
    pub fn configure_input_pullup(pin: u8) {
        // GPIO config registers are at GLB_BASE + 0x100 + (pin * 4)
        let reg = (GLB_BASE + 0x100 + (pin as usize) * 4) as *mut u32;
        unsafe {
            // Bits: [0] input_en=1, [1] output_en=0, [4] pullup=1
            core::ptr::write_volatile(reg, 0x11);
        }
    }
}

// ── UART logger for debug output ───────────────────────────────
mod uart_log {
    use core::fmt::Write;

    const UART_BASE: usize = 0x4000_A000;
    const UART_FIFO_WDATA: *mut u32 = (UART_BASE + 0x88) as *mut u32;
    const UART_FIFO_STATUS: *const u32 = (UART_BASE + 0x84) as *const u32;

    struct UartWriter;

    impl Write for UartWriter {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            for b in s.bytes() {
                write_byte(b);
            }
            Ok(())
        }
    }

    pub struct Bl702Logger;

    impl log::Log for Bl702Logger {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            if self.enabled(record.metadata()) {
                let mut w = UartWriter;
                let _ = write!(w, "[{}] {}\r\n", record.level(), record.args());
            }
        }

        fn flush(&self) {}
    }

    fn write_byte(b: u8) {
        unsafe {
            // Wait for TX FIFO not full
            for _ in 0..10000u32 {
                let status = core::ptr::read_volatile(UART_FIFO_STATUS);
                if (status >> 8) & 0x3F < 32 {
                    break;
                }
            }
            core::ptr::write_volatile(UART_FIFO_WDATA, b as u32);
        }
    }

    static LOGGER: Bl702Logger = Bl702Logger;

    pub fn init() {
        let _ = log::set_logger(&LOGGER);
        log::set_max_level(log::LevelFilter::Info);
    }
}

// ── Entry point ────────────────────────────────────────────────
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize BL702 peripherals
    time_driver::init();
    uart_log::init();

    log::info!("BL702 Zigbee sensor starting");

    // Configure button (GPIO8 on most BL702 modules, active low)
    const BUTTON_PIN: u8 = 8;
    gpio::configure_input_pullup(BUTTON_PIN);
    let mut button_was_pressed = false;

    // Create 802.15.4 MAC driver
    let mac = Bl702Mac::new();

    log::info!("Radio ready");

    // ZCL cluster instances
    let mut basic_cluster = BasicCluster::new(b"Zigbee-RS", b"BL702-Sensor", b"20260402", b"0.1.0");
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
        .model("BL702-Sensor")
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
    log::info!("Auto-joining network…");
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
        let pressed = !gpio::read_input(BUTTON_PIN); // Active low
        if pressed && !button_was_pressed {
            if device.is_joined() {
                log::info!("Button → leaving network");
            } else {
                log::info!("Button → joining network");
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
                            log::info!("Interview done — ending fast poll");
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
                    "T={}.{:02}°C H={}.{:02}%",
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
                log::info!("Device_annce retry ({} left)", annce_retries_left);
            }
        } else {
            // Not joined — auto-retry every 15s
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
                    log::info!("Joined!");
                }
            }
        }
    }
}
