//! # ESP32-H2 Zigbee Sensor (SED)
//!
//! Full-featured Zigbee 3.0 sleepy end device for ESP32-H2.
//! Uses the built-in IEEE 802.15.4 radio via `esp-radio`.
//!
//! # Features
//! - Auto-join on boot, secure rejoin on restart with saved state
//! - Sleepy End Device: poll parent for indirect frames
//! - Fast poll (250ms) during ZHA interview, slow poll (30s) normal
//! - Device_annce retries for reliable coordinator discovery
//! - NWK Leave handler: auto-rejoin when coordinator sends Leave
//! - Default reporting: temp/hum/battery reported without ZHA interview
//! - Button: BOOT (GPIO9) — short=toggle, long=factory reset
//!
//! # Architecture
//! `esp32-zigbee-devkit-product` (see `products/esp32-zigbee-devkit`) owns
//! the typed [`SensorProfile`](esp32_zigbee_devkit_product::profile::SensorProfile)
//! (endpoint/cluster declaration) and the durable
//! [`SecurityStore`](esp32_zigbee_devkit_product::storage::SecurityStore).
//! This build has no OTA backend — see the crate docs and
//! `docs/book/src/platform-guides/esp32.md`. This file is only the
//! composition root: platform startup, resource construction, and handing
//! both to [`zigbee_runtime::node::ZigbeeNode`] before running
//! [`app::SensorApp`]'s event loop.
//!
//! # Build
//! ```bash
//! cargo build --release
//! espflash flash --monitor target/riscv32imac-unknown-none-elf/release/esp32h2-sensor
//! ```

#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

mod app;
mod chip_temperature;
mod time_driver;

use app::SensorApp;
use chip_temperature::H2TemperatureSensor;
use esp_backtrace as _;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};

use embassy_futures::block_on;
use static_cell::StaticCell;

use esp32_zigbee_devkit_product as product;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::clusters::basic::PowerSource;

// Bridge `log` crate → esp_println
struct EspLogger;
impl log::Log for EspLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }
    fn log(&self, record: &log::Record) {
        esp_println::println!("[{}] {}", record.level(), record.args());
    }
    fn flush(&self) {}
}
static LOGGER: EspLogger = EspLogger;

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
        for _ in 0..100_000u32 {
            core::hint::spin_loop();
        }
        led.set_high();
        for _ in 0..100_000u32 {
            core::hint::spin_loop();
        }
    }

    let temp_sensor = H2TemperatureSensor::new().expect("temp sensor init failed");

    // IEEE 802.15.4 radio
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(peripherals.IEEE802154);
    let config = esp_radio::ieee802154::Config::default();
    let mac = zigbee_mac::esp::EspMac::new(ieee802154, config);
    esp_println::println!("[ESP32-H2] Radio ready");

    // Product-owned durable security store and endpoint profile (no OTA on
    // this build — see `esp32_zigbee_devkit_product::profile`).
    //
    // `open_security_store` also runs the one-time legacy persistence
    // migration (see `esp32_zigbee_devkit_product::migration`): a device
    // already joined under the old `LogStructuredNv` format keeps its network
    // and is upgraded to the crash-safe journal without reusing a frame
    // counter. A migration error means the reserved NV region may still be
    // intact, so halt rather than silently commissioning as factory-new.
    let (security, migration) = product::storage::open_security_store().unwrap_or_else(|error| {
        esp_println::println!(
            "[ESP32-H2] FATAL: persistence migration failed: {:?}",
            error
        );
        loop {
            core::hint::spin_loop();
        }
    });
    esp_println::println!("[ESP32-H2] Persistence migration: {:?}", migration);
    let profile = product::profile::sensor_profile();

    let device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .date_code(product::DATE_CODE)
        .sw_build("0.1.0")
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build();

    // `ZigbeeNode` borrows the device, security store and profile for its
    // whole lifetime; giving them `'static` storage (like the EFR32MG1
    // product) keeps that borrow trivially valid across the diverging
    // `block_on` call below, regardless of how the `#[esp_hal::main]` macro
    // structures the generated entry point.
    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<product::profile::SensorProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<zigbee_mac::esp::EspMac<'static>>> = StaticCell::new();
    static APP: StaticCell<SensorApp<'static>> = StaticCell::new();

    let security = SECURITY.init(security);
    let profile = PROFILE.init(profile);
    let device = DEVICE.init(device);

    let node = ZigbeeNode::new(device, security, profile);
    let app = APP.init(SensorApp::new(node, button, led, temp_sensor));

    esp_println::println!("[ESP32-H2] Flash NV storage ready (security-state journal)");

    block_on(app.run())
}
