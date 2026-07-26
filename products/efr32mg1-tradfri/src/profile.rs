//! Complete Zigbee profile selected by this firmware product.

use crate::ota::Efr32FirmwareWriter;
use crate::{ENDPOINT, OTA_IMAGE_TYPE, OTA_MANUFACTURER_CODE};
use efr32mg1_tradfri::resources::BootloaderFlashAccess;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::firmware_writer::FirmwareError;
use zigbee_runtime::ota::{OtaConfig, OtaManager};
use zigbee_runtime::profile::{
    BatteryDescriptor, DeviceProfile, EnvironmentalReporting, ProfileError,
    TemperatureHumidityBattery, TemperatureRange, WithOta,
};
use zigbee_zcl::DeviceId;

pub type BaseSensorProfile = DeviceProfile<TemperatureHumidityBattery>;
pub type SensorProfile = WithOta<BaseSensorProfile, Efr32FirmwareWriter>;

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

pub fn sensor_profile(
    firmware_version: u32,
    flash_access: BootloaderFlashAccess,
) -> Result<SensorProfile, SensorProfileError> {
    let environment = TemperatureHumidityBattery::new(
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
    );
    let base = DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::TEMPERATURE_SENSOR,
        environment,
    );
    let ota = OtaManager::new(
        Efr32FirmwareWriter::new(flash_access)?,
        OtaConfig {
            manufacturer_code: OTA_MANUFACTURER_CODE,
            image_type: OTA_IMAGE_TYPE,
            current_version: firmware_version,
            endpoint: ENDPOINT,
            block_size: 48,
            auto_accept: true,
            hardware_version: Some(1),
        },
    );
    Ok(WithOta::new(base, ota)?)
}
