//! Complete Zigbee profile selected by this firmware product.
//!
//! Both chips report the same clusters (Temperature, Humidity, Power
//! Configuration on the Home Automation Temperature Sensor device ID) with
//! the same reporting cadence, so the base profile is shared. Only the
//! ESP32-C6 build composes an OTA backend, and only when the checked
//! partition table on the device actually supports it — see
//! [`OptionalOta`](zigbee_runtime::profile::OptionalOta).

use crate::ENDPOINT;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{
    BatteryDescriptor, DeviceProfile, EnvironmentalReporting, TemperatureHumidityBattery,
    TemperatureRange,
};
use zigbee_zcl::DeviceId;

/// Temperature + Humidity + Power Configuration, no OTA.
pub type BaseSensorProfile = DeviceProfile<TemperatureHumidityBattery>;

fn environment() -> TemperatureHumidityBattery {
    TemperatureHumidityBattery::new(
        TemperatureRange {
            min_centi_celsius: -4_000,
            max_centi_celsius: 12_500,
        },
        BatteryDescriptor {
            size: 4,
            quantity: 2,
            rated_voltage_100mv: 15,
        },
        EnvironmentalReporting::default(),
    )
}

/// Build the shared base profile (no OTA).
pub fn base_profile() -> BaseSensorProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::TEMPERATURE_SENSOR,
        environment(),
    )
}

// `EspOtaFlash` wraps the board's ROM SPI flash driver, which only builds
// for the Espressif target (see `esp32-zigbee-devkit::flash`); this module
// is gated the same way. The chip-independent OTA writer logic underneath
// it (`EspFirmwareWriter<MockFlash>`) is still exercised on the host by
// `ota`'s own unit tests.
#[cfg(all(feature = "esp32c6", target_os = "none"))]
mod with_ota {
    use super::{BaseSensorProfile, base_profile};
    use crate::ota::{EspFirmwareWriter, EspOtaFlash};
    use crate::{ENDPOINT, OTA_HARDWARE_VERSION, OTA_IMAGE_TYPE, OTA_MANUFACTURER_CODE};
    use zigbee_runtime::ota::{OtaConfig, OtaManager};
    use zigbee_runtime::profile::OptionalOta;

    /// The ESP32-C6 profile: the shared base profile, with the OTA Upgrade
    /// client cluster composed in only when the on-device partition table
    /// supports it.
    pub type SensorProfile = OptionalOta<BaseSensorProfile, EspFirmwareWriter<EspOtaFlash>>;

    /// Build the ESP32-C6 profile.
    ///
    /// `reset` performs the software reset that hands control to the
    /// bootloader after an upgrade is staged and verified; it is provided by
    /// the composition root (`main.rs`) rather than depended on directly here,
    /// so this crate does not need an `esp-hal` dependency of its own.
    ///
    /// A missing or incompatible partition table disables OTA without
    /// preventing the sensor from joining and reporting: the endpoint simply
    /// does not advertise the OTA Upgrade client cluster. This also avoids
    /// bricking a remotely upgraded image if its layout expectations ever
    /// differ from the table installed on the device.
    pub fn sensor_profile(firmware_version: u32, reset: fn() -> !) -> SensorProfile {
        let config = OtaConfig {
            manufacturer_code: OTA_MANUFACTURER_CODE,
            image_type: OTA_IMAGE_TYPE,
            current_version: firmware_version,
            endpoint: ENDPOINT,
            block_size: 48,
            auto_accept: true,
            hardware_version: Some(OTA_HARDWARE_VERSION),
        };
        match EspFirmwareWriter::new(EspOtaFlash::new(), reset) {
            Ok(writer) => {
                log::info!(
                    "[ESP32-C6] OTA ready: running slot {}, staging slot {}, version {}",
                    writer.running_slot(),
                    writer.target_slot(),
                    firmware_version
                );
                OptionalOta::enabled(base_profile(), OtaManager::new(writer, config))
                    .expect("OTA endpoint matches the base profile endpoint")
            }
            Err(error) => {
                log::warn!(
                    "[ESP32-C6] OTA disabled: incompatible flash layout ({:?})",
                    error
                );
                OptionalOta::disabled(
                    base_profile(),
                    OTA_MANUFACTURER_CODE,
                    OTA_IMAGE_TYPE,
                    firmware_version,
                )
            }
        }
    }
}

#[cfg(all(feature = "esp32c6", target_os = "none"))]
pub use with_ota::{SensorProfile, sensor_profile};

/// The ESP32-H2 profile: the shared base profile, no OTA. The H2 build keeps
/// the default single-app partition table and has no OTA writer (see the
/// crate docs).
#[cfg(feature = "esp32h2")]
pub type SensorProfile = BaseSensorProfile;

/// Build the ESP32-H2 profile.
#[cfg(feature = "esp32h2")]
pub fn sensor_profile() -> SensorProfile {
    base_profile()
}
