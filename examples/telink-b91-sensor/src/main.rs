//! # Telink B91 Zigbee Temperature Sensor
//!
//! A `no_std` firmware for the Telink B91 (RISC-V),
//! implementing a Zigbee 3.0 end device that exposes Temperature
//! Measurement (0x0402) and Relative Humidity (0x0405) clusters.
//!
//! ## Hardware
//! - Telink B91 module (RISC-V 32-bit, 512KB Flash, 256KB SRAM)
//! - Built-in IEEE 802.15.4 + BLE 5.0 radio
//! - Button (GPIO2 on B91 devboard): join/leave network
//!
//! ## Radio driver
//! The Telink backend uses FFI bindings to Telink's RF driver library
//! (`libdrivers_b91.a`) for 802.15.4 radio access, with interrupt-driven
//! TX/RX via Embassy signals.
//!
//! ## Building
//! ```bash
//! cd examples/telink-b91-sensor
//! TELINK_SDK_DIR=/path/to/tl_zigbee_sdk cargo build --release
//! ```

#![no_std]
#![no_main]

#[cfg(feature = "stubs")]
mod stubs;

use panic_halt as _;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::telink::TelinkMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::{UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

const REPORT_INTERVAL_SECS: u64 = 30;

// ── Minimal GPIO helpers (B91 register-based) ──────────────────

mod gpio {
    const GPIO_BASE: u32 = 0x140300;

    pub fn configure_input_pullup(pin: u8) {
        // B91 GPIO configuration — simplified for compilation
        let _ = (GPIO_BASE, pin);
    }

    pub fn set_output(pin: u8) {
        let _ = (GPIO_BASE, pin);
    }

    pub fn write(pin: u8, high: bool) {
        unsafe {
            let reg = (GPIO_BASE + 0x04) as *mut u32;
            let val = core::ptr::read_volatile(reg);
            if high {
                core::ptr::write_volatile(reg, val | (1 << pin));
            } else {
                core::ptr::write_volatile(reg, val & !(1 << pin));
            }
        }
    }

    pub fn read_input(pin: u8) -> bool {
        unsafe {
            let reg = (GPIO_BASE + 0x00) as *const u32;
            let val = core::ptr::read_volatile(reg);
            (val >> pin) & 1 == 1
        }
    }
}

// ── Minimal Embassy time driver ────────────────────────────────

mod time_driver {
    use embassy_time_driver::Driver;

    struct TelinkTimeDriver;

    impl TelinkTimeDriver {
        const fn new() -> Self {
            Self
        }
    }

    impl Driver for TelinkTimeDriver {
        fn now(&self) -> u64 {
            // Read from B91 system timer in real implementation
            0
        }

        fn schedule_wake(&self, _at: u64, _waker: &core::task::Waker) {
            // Set alarm in system timer in real implementation
        }
    }

    embassy_time_driver::time_driver_impl!(
        static TIME_DRIVER: TelinkTimeDriver = TelinkTimeDriver::new()
    );
}

// ── Logging ─────────────────────────────────────────────────────
// log::info!() etc. compile as no-ops without a registered logger.
// For real debug output, use Telink UART or BDT (Burning & Debug Tool).

// ── B91 devboard pin assignments ───────────────────────────────

mod pins {
    pub const BTN1: u8 = 2; // GPIO2 — button
    pub const LED1: u8 = 3; // GPIO3 — green LED
    pub const LED2: u8 = 4; // GPIO4 — blue LED
}

// ── Entry point ────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    log::info!("Telink B91 Zigbee sensor starting");

    // Configure GPIO
    gpio::configure_input_pullup(pins::BTN1);
    gpio::set_output(pins::LED1);
    gpio::set_output(pins::LED2);

    // Blink LED to show alive
    for _ in 0..3 {
        gpio::write(pins::LED1, true);
        Timer::after(Duration::from_millis(100)).await;
        gpio::write(pins::LED1, false);
        Timer::after(Duration::from_millis(100)).await;
    }

    // Create MAC driver
    let mac = TelinkMac::new();

    // ZCL cluster instances
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);

    // Build Zigbee device
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Zigbee-RS")
        .model("B91-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    log::info!("Device ready — press button to join/leave");

    let mut button_was_pressed = false;
    let mut tick: u32 = 0;

    loop {
        // Button handling (edge detection, active low)
        let pressed = !gpio::read_input(pins::BTN1);
        if pressed && !button_was_pressed {
            if device.is_joined() {
                log::info!("Button → leaving network");
            } else {
                log::info!("Button → joining network");
            }
            device.user_action(UserAction::Toggle);
            Timer::after(Duration::from_millis(300)).await;
        }
        button_was_pressed = pressed;

        // Simulated sensor readings
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
