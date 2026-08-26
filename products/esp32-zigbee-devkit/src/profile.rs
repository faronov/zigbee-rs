//! Complete Zigbee profile selected by this firmware product.
//!
//! Both chips report the same clusters (Temperature, Humidity, Power
//! Configuration on the Home Automation Temperature Sensor device ID) with
//! the same reporting cadence, so the base profile is shared. Only the
//! selected ESP32 build has an OTA backend. The checked partition table is a
//! startup requirement, so the OTA client descriptor never changes at runtime.

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
#[cfg(target_os = "none")]
mod with_ota {
    use super::{BaseSensorProfile, base_profile};
    use crate::ota::{EspFirmwareWriter, EspOtaFlash};
    use crate::{ENDPOINT, OTA_HARDWARE_VERSION, OTA_IMAGE_TYPE, OTA_MANUFACTURER_CODE};
    use zigbee_runtime::firmware_writer::FirmwareError;
    use zigbee_runtime::ota::{OtaConfig, OtaManager};
    use zigbee_runtime::profile::{ProfileError, WithOta};

    /// The ESP32 profile: the shared base profile plus the mandatory OTA
    /// Upgrade client cluster.
    pub type SensorProfile = WithOta<BaseSensorProfile, EspFirmwareWriter<EspOtaFlash>>;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum SensorProfileError {
        Firmware(FirmwareError),
        Profile(ProfileError),
    }

    impl From<FirmwareError> for SensorProfileError {
        fn from(error: FirmwareError) -> Self {
            Self::Firmware(error)
        }
    }

    impl From<ProfileError> for SensorProfileError {
        fn from(error: ProfileError) -> Self {
            Self::Profile(error)
        }
    }

    /// Build the selected ESP32 profile.
    ///
    /// `reset` performs the software reset that hands control to the
    /// bootloader after an upgrade is staged and verified; it is provided by
    /// the composition root (`main.rs`) rather than depended on directly here,
    /// so this crate does not need an `esp-hal` dependency of its own.
    ///
    /// A missing or incompatible partition table is an explicit startup
    /// failure. This OTA-capable image always advertises the OTA Upgrade
    /// client cluster, so it cannot safely continue with a different endpoint
    /// descriptor after partition discovery.
    pub fn sensor_profile(
        firmware_version: u32,
        reset: fn() -> !,
    ) -> Result<SensorProfile, SensorProfileError> {
        let config = OtaConfig {
            manufacturer_code: OTA_MANUFACTURER_CODE,
            image_type: OTA_IMAGE_TYPE,
            current_version: firmware_version,
            endpoint: ENDPOINT,
            block_size: 48,
            auto_accept: true,
            hardware_version: Some(OTA_HARDWARE_VERSION),
        };
        let writer = EspFirmwareWriter::new(EspOtaFlash::new(), reset)?;
        log::info!(
            "[ESP32] OTA ready: running slot {}, staging slot {}, version {}",
            writer.running_slot(),
            writer.target_slot(),
            firmware_version
        );
        Ok(WithOta::new(
            base_profile(),
            OtaManager::new(writer, config),
        )?)
    }
}

#[cfg(target_os = "none")]
pub use with_ota::{SensorProfile, SensorProfileError, sensor_profile};
