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
use zigbee_runtime::{UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

const REPORT_INTERVAL_SECS: u64 = 30;

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
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);

    // Build Zigbee device
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Zigbee-RS")
        .model("BL702-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    log::info!("Device ready — press button to join/leave");

    let mut tick: u32 = 0;

    loop {
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

        // ── Simulated sensor readings ────────────────────────
        // Replace with real I²C sensor reads for production hardware
        let temp_hundredths: i16 = 2250 + ((tick % 50) as i16 - 25);
        let hum_hundredths: u16 = 5000 + ((tick % 100) as u16) * 10;

        temp_cluster.set_temperature(temp_hundredths);
        hum_cluster.set_humidity(hum_hundredths);

        if device.is_joined() {
            log::info!(
                "T={}.{:02}°C  H={}.{:02}%",
                temp_hundredths / 100,
                (temp_hundredths % 100).unsigned_abs(),
                hum_hundredths / 100,
                hum_hundredths % 100,
            );
        }

        tick = tick.wrapping_add(1);
        Timer::after(Duration::from_secs(REPORT_INTERVAL_SECS)).await;
    }
}
