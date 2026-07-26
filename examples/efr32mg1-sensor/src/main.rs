//! Production EFR32MG1P TRÅDFRI Zigbee temperature/humidity SED.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod app;
mod platform;
mod sensor;
mod time_driver;
mod vectors;

include!(concat!(env!("OUT_DIR"), "/firmware_version.rs"));

use cortex_m as _;
use efr32mg1_tradfri::resources::BoardResources;
use embassy_executor::Spawner;
use static_cell::StaticCell;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;
use zigbee_bdb::attributes::{BDB_POPULAR_CHANNEL_FALLBACK_SET, BDB_POPULAR_CHANNEL_SET};
use zigbee_mac::efr32::Efr32Mac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

const FAST_POLL_MS: u32 = 250;
const SLOW_POLL_SECS: u32 = 30;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // ── Board resource ownership ─────────────────────────────────────────
    // Take the singleton resource set. Each token is consumed exactly once,
    // enforcing mutual exclusion (PA0 → LED not PWM; flash → bootloader not SPI).
    let board = BoardResources::take().unwrap_or_else(|| platform::halt_with_led_raw());

    // Platform startup owns only physical startup resources.
    platform::init(board.pa0, board.button);
    platform::signal_boot().await;

    // Product policy selects the resident Gecko Bootloader OTA path rather
    // than direct USART0 SPI access to the same external flash.
    let ota_flash = board.external_flash.into_bootloader_managed();

    // ── Sensor I2C and supply ADC ────────────────────────────────────────
    let i2c = board
        .sensor_i2c
        .into_sensor_i2c()
        .unwrap_or_else(|_| platform::halt_with_led());
    let sht = sensor::Sensor::new(i2c);

    let battery = board.supply_adc.into_supply_monitor().ok().map(|supply| {
        efr32mg1_tradfri_product::battery::BatteryMonitor::from_supply(
            supply,
            efr32mg1_tradfri_product::battery::DEFAULT_BATTERY_CURVE,
        )
    });

    // ── Zigbee stack and product profile ─────────────────────────────────
    static SECURITY: StaticCell<efr32mg1_tradfri_product::storage::SecurityStore> =
        StaticCell::new();
    static PROFILE: StaticCell<efr32mg1_tradfri_product::profile::SensorProfile> =
        StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();
    static APP: StaticCell<app::SensorApp> = StaticCell::new();

    let profile = PROFILE.init(
        efr32mg1_tradfri_product::profile::sensor_profile(FIRMWARE_VERSION, ota_flash)
            .unwrap_or_else(|_| platform::halt_with_led()),
    );

    let device = ZigbeeDevice::builder(Efr32Mac::new())
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: SLOW_POLL_SECS * 1_000,
            wake_duration_ms: FAST_POLL_MS,
        })
        .automatic_polling(false)
        .manufacturer(efr32mg1_tradfri_product::MANUFACTURER)
        .model(efr32mg1_tradfri_product::MODEL)
        .application_version(FIRMWARE_APPLICATION_VERSION)
        .date_code(efr32mg1_tradfri_product::DATE_CODE)
        .sw_build(FIRMWARE_VERSION_STR)
        .power_source(PowerSource::Battery)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build_into(DEVICE.uninit());
    device.bdb_mut().attributes_mut().primary_channel_set = BDB_POPULAR_CHANNEL_SET;
    device.bdb_mut().attributes_mut().secondary_channel_set = BDB_POPULAR_CHANNEL_FALLBACK_SET;

    let node = ZigbeeNode::new(
        device,
        SECURITY.init(efr32mg1_tradfri_product::storage::security_store()),
        profile,
    );

    APP.init(app::SensorApp::new(node, sht, battery))
        .run()
        .await
}
