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
    use crate::ota::{EspFirmwareWriter, EspOtaFlash, OtaInitError};
    use crate::{ENDPOINT, OTA_HARDWARE_VERSION, OTA_IMAGE_TYPE, OTA_MANUFACTURER_CODE};
    use zigbee_runtime::ota::{OtaConfig, OtaManager};
    use zigbee_runtime::profile::{ProfileError, WithOta};

    /// The OTA writer: the ESP image writer, behind the signed-envelope
    /// check with feature `signed-ota`.
    #[cfg(not(feature = "signed-ota"))]
    pub type OtaWriter = EspFirmwareWriter<EspOtaFlash>;
    #[cfg(feature = "signed-ota")]
    pub type OtaWriter = crate::signed_ota::SignedEspWriter<EspOtaFlash>;

    /// The ESP32 profile: the shared base profile plus the mandatory OTA
    /// Upgrade client cluster.
    pub type SensorProfile = WithOta<BaseSensorProfile, OtaWriter>;

    /// The product's OTA writer for a build advertising `firmware_version`
    /// (with `signed-ota`, also the oldest envelope version accepted).
    #[cfg_attr(not(feature = "signed-ota"), allow(unused_variables))]
    pub fn ota_writer(firmware_version: u32, reset: fn() -> !) -> Result<OtaWriter, OtaInitError> {
        let writer = EspFirmwareWriter::new(EspOtaFlash::new(), reset)?;
        #[cfg(feature = "signed-ota")]
        let writer = crate::signed_ota::signed_writer(
            writer,
            crate::signed_ota::image_version(firmware_version),
        );
        Ok(writer)
    }

    /// The ESP image writer inside [`OtaWriter`].
    fn esp_writer(writer: &OtaWriter) -> &EspFirmwareWriter<EspOtaFlash> {
        #[cfg(feature = "signed-ota")]
        let writer = writer.inner();
        writer
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum SensorProfileError {
        OtaInitialization(OtaInitError),
        Profile(ProfileError),
    }

    impl From<OtaInitError> for SensorProfileError {
        fn from(error: OtaInitError) -> Self {
            Self::OtaInitialization(error)
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
    /// keeping reset policy in the composition root.
    ///
    /// A missing/incompatible partition table or unavailable live MMU
    /// running-image evidence is an explicit typed startup failure.
    /// This OTA-capable image always advertises the OTA Upgrade
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
        let writer = ota_writer(firmware_version, reset)?;
        log::info!(
            "[ESP32] OTA ready: running slot {}, staging slot {}, version {}",
            esp_writer(&writer).running_slot(),
            esp_writer(&writer).target_slot(),
            firmware_version
        );
        Ok(WithOta::new(
            base_profile(),
            OtaManager::new(writer, config),
        )?)
    }
}

#[cfg(target_os = "none")]
pub use with_ota::{OtaWriter, SensorProfile, SensorProfileError, ota_writer, sensor_profile};
