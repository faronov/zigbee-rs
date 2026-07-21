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
use embassy_executor::Spawner;
use static_cell::StaticCell;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::efr32::Efr32Mac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::ZigbeeDevice;
#[cfg(feature = "ota")]
use zigbee_runtime::ota::{OtaConfig, OtaManager};
use zigbee_runtime::power::PowerMode;
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::{ClusterId, DeviceId};

const FAST_POLL_MS: u32 = 250;
const SLOW_POLL_SECS: u32 = 30;
#[cfg(feature = "ota")]
const OTA_MANUFACTURER_CODE: u16 = 0x1049;
#[cfg(feature = "ota")]
const OTA_IMAGE_TYPE: u16 = 0x0002;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    platform::init();
    platform::signal_boot().await;

    let i2c = efr32mg1_tradfri::sensor_i2c().unwrap_or_else(|_| platform::halt_with_led());
    let sht = sensor::probe(i2c).await;
    let battery = efr32mg1_tradfri::battery_monitor().ok();
    #[cfg(feature = "ota")]
    let ota = OtaManager::new(
        efr32mg1_tradfri::ota::Efr32FirmwareWriter::new()
            .unwrap_or_else(|_| platform::halt_with_led()),
        OtaConfig {
            manufacturer_code: OTA_MANUFACTURER_CODE,
            image_type: OTA_IMAGE_TYPE,
            current_version: FIRMWARE_VERSION,
            endpoint: 1,
            block_size: 48,
            auto_accept: true,
            hardware_version: Some(1),
        },
    );

    static SECURITY: StaticCell<efr32mg1_tradfri::storage::SecurityStore> = StaticCell::new();
    static TEMP: StaticCell<TemperatureCluster> = StaticCell::new();
    static HUM: StaticCell<HumidityCluster> = StaticCell::new();
    static POWER: StaticCell<PowerConfigCluster> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();
    static APP: StaticCell<app::SensorApp> = StaticCell::new();

    let power = POWER.init(PowerConfigCluster::new());
    power.set_battery_voltage(0xFF);
    power.set_battery_percentage(0xFF);
    power.set_battery_size(4);
    power.set_battery_quantity(2);
    power.set_battery_rated_voltage(15);

    let device = ZigbeeDevice::builder(Efr32Mac::new())
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: SLOW_POLL_SECS * 1_000,
            wake_duration_ms: FAST_POLL_MS,
        })
        .manufacturer("Zigbee-RS")
        .model("EFR32MG1-Sensor")
        .date_code("20260402")
        .sw_build(FIRMWARE_VERSION_STR)
        .power_source(PowerSource::Battery)
        .channels(ChannelMask(1 << 15))
        .endpoint(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::TEMPERATURE_SENSOR,
            |ep| {
                let ep = ep
                    .cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::IDENTIFY)
                    .cluster_server(ClusterId::POWER_CONFIG)
                    .cluster_server(ClusterId::TEMPERATURE)
                    .cluster_server(ClusterId::HUMIDITY);
                #[cfg(feature = "ota")]
                let ep = ep.cluster_client(ClusterId::OTA_UPGRADE);
                ep
            },
        )
        .build_into(DEVICE.uninit());

    APP.init(app::SensorApp::new(
        device,
        SECURITY.init(efr32mg1_tradfri::storage::security_store()),
        sht,
        battery,
        TEMP.init(TemperatureCluster::new(-4000, 12500)),
        HUM.init(HumidityCluster::new(0, 10000)),
        power,
        #[cfg(feature = "ota")]
        ota,
    ))
    .run()
    .await
}
