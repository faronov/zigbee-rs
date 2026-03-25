//! # ESP32-C6 Zigbee Temperature & Humidity Sensor
//!
//! A complete no_std firmware for the ESP32-C6, implementing a Zigbee 3.0
//! end device with simulated temperature/humidity and a join/leave button.
//!
//! ## Hardware
//! - ESP32-C6 (RISC-V, built-in IEEE 802.15.4 radio)
//! - BOOT button (GPIO9): press to join / leave the Zigbee network
//!
//! ## Operation
//! 1. Power on → device starts idle
//! 2. Press BOOT button → joins the Zigbee network
//! 3. Every 30 s: simulated sensor values are updated
//! 4. Press BOOT button again → leaves the network
//!
//! To add an external SHTC3 sensor, connect SDA→GPIO6, SCL→GPIO7 and
//! use `esp_hal::i2c::master::I2c` with the SHTC3 commands documented below.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::gpio::{Input, InputConfig, Pull};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::{UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u32 = 30;

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_println::println!("[init] ESP32-C6 Zigbee sensor starting");

    // BOOT button (GPIO9, active low)
    let button = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );
    let mut button_was_pressed = false;

    // IEEE 802.15.4 MAC driver
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(peripherals.IEEE802154);
    let config = esp_radio::ieee802154::Config::default();
    let mac = zigbee_mac::esp::EspMac::new(ieee802154, config);

    esp_println::println!("[init] Radio ready");

    // ZCL cluster instances
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);

    // Build the Zigbee device
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Zigbee-RS")
        .model("ESP32-C6-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    esp_println::println!("[init] Device ready — press BOOT button to join/leave");

    let delay = esp_hal::delay::Delay::new();
    let mut tick: u32 = 0;

    loop {
        // ── Button handling (with edge detection) ──────────
        let pressed = button.is_low();
        if pressed && !button_was_pressed {
            if device.is_joined() {
                esp_println::println!("[btn] Leaving network…");
            } else {
                esp_println::println!("[btn] Joining network…");
            }
            device.user_action(UserAction::Toggle);
            delay.delay_millis(300); // debounce
        }
        button_was_pressed = pressed;

        // ── Simulated sensor readings ──────────────────────
        // Replace with real I2C sensor reads (e.g. SHTC3 at 0x70)
        let temp_hundredths: i16 = 2250 + ((tick % 50) as i16 - 25);
        let hum_hundredths: u16 = 5000 + ((tick % 100) as u16) * 10;

        temp_cluster.set_temperature(temp_hundredths);
        hum_cluster.set_humidity(hum_hundredths);

        if device.is_joined() {
            esp_println::println!(
                "[sensor] T={}.{:02}°C  H={}.{:02}%",
                temp_hundredths / 100,
                (temp_hundredths % 100).unsigned_abs(),
                hum_hundredths / 100,
                hum_hundredths % 100,
            );
        }

        tick = tick.wrapping_add(1);
        delay.delay_millis(REPORT_INTERVAL_SECS * 1000);
    }
}
