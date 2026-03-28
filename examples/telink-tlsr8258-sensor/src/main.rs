//! # Telink TLSR8258 Zigbee Temperature Sensor
//!
//! A `no_std` firmware for the Telink TLSR8258 (tc32 ISA),
//! implementing a Zigbee 3.0 end device with Temperature
//! Measurement (0x0402) and Relative Humidity (0x0405) clusters.
//!
//! ## Hardware
//! - Telink TLSR8258 module (tc32 core, 512KB Flash, 64KB SRAM)
//! - Built-in IEEE 802.15.4 + BLE radio
//! - Used in many Zigbee products (Sonoff SNZB, Tuya, IKEA devices)
//!
//! ## Note on tc32 ISA
//! The TLSR8258 uses Telink's proprietary tc32 instruction set.
//! There is no official Rust target for tc32. For `cargo check`,
//! we use `thumbv6m-none-eabi` as a compilation stand-in to verify
//! the Rust code compiles. Real builds require the Telink tc32 GCC
//! toolchain and would link via C firmware that calls into the Rust
//! static library.
//!
//! ## Building (cargo check only)
//! ```bash
//! cd examples/telink-tlsr8258-sensor
//! cargo check --release
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

// ── Minimal GPIO (tc32 register-mapped) ────────────────────────

mod gpio {
    const GPIO_BASE: u32 = 0x00586; // TLSR8258 GPIO base register

    pub fn configure_input_pullup(_pin: u8) {
        // TLSR8258 GPIO configuration via register writes
    }

    pub fn set_output(_pin: u8) {
        // TLSR8258 GPIO output enable
    }

    pub fn write(pin: u8, high: bool) {
        unsafe {
            let reg = (GPIO_BASE + 0x04) as *mut u8;
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
            let reg = GPIO_BASE as *const u8;
            let val = core::ptr::read_volatile(reg);
            (val >> pin) & 1 == 1
        }
    }
}

// ── Minimal Embassy time driver ────────────────────────────────

mod time_driver {
    use embassy_time_driver::Driver;

    struct Tlsr8258TimeDriver;

    impl Tlsr8258TimeDriver {
        const fn new() -> Self {
            Self
        }
    }

    impl Driver for Tlsr8258TimeDriver {
        fn now(&self) -> u64 {
            // Read from TLSR8258 system timer
            0
        }

        fn schedule_wake(&self, _at: u64, _waker: &core::task::Waker) {
            // Set alarm in system timer
        }
    }

    embassy_time_driver::time_driver_impl!(
        static TIME_DRIVER: Tlsr8258TimeDriver = Tlsr8258TimeDriver::new()
    );
}

// ── TLSR8258 devboard pins (e.g., Sonoff SNZB-02) ─────────────

mod pins {
    pub const BTN1: u8 = 2; // Button
    pub const LED1: u8 = 3; // LED
}

// ── Entry point ────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    log::info!("TLSR8258 Zigbee sensor starting");

    gpio::configure_input_pullup(pins::BTN1);
    gpio::set_output(pins::LED1);

    // Blink LED
    for _ in 0..3 {
        gpio::write(pins::LED1, true);
        Timer::after(Duration::from_millis(100)).await;
        gpio::write(pins::LED1, false);
        Timer::after(Duration::from_millis(100)).await;
    }

    let mac = TelinkMac::new();

    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);

    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Zigbee-RS")
        .model("TLSR8258-Sensor")
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
