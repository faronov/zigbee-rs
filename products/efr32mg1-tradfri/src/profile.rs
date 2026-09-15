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
use zigbee_zcl::foundation::reporting::MAX_REPORT_CONFIGS;

pub type BaseSensorProfile = DeviceProfile<TemperatureHumidityBattery>;
pub type SensorProfile = WithOta<BaseSensorProfile, Efr32FirmwareWriter>;

const APPLICATION_ENDPOINT_COUNT: usize = 1;
const SERVER_CLUSTER_COUNT: usize = 5;
const CLIENT_CLUSTER_COUNT: usize = 1;
const DEFAULT_REPORT_CONFIG_COUNT: usize =
    TemperatureHumidityBattery::EXPECTED_REPORT_CLUSTER_IDS.len();

const _: () = {
    assert!(APPLICATION_ENDPOINT_COUNT <= zigbee_runtime::MAX_ENDPOINTS);
    assert!(SERVER_CLUSTER_COUNT <= zigbee_runtime::MAX_CLUSTERS_PER_ENDPOINT);
    assert!(CLIENT_CLUSTER_COUNT <= zigbee_runtime::MAX_CLUSTERS_PER_ENDPOINT);
    assert!(DEFAULT_REPORT_CONFIG_COUNT <= MAX_REPORT_CONFIGS);
};

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

fn base_sensor_profile() -> BaseSensorProfile {
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
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::TEMPERATURE_SENSOR,
        environment,
    )
}

fn ota_config(current_version: u32) -> OtaConfig {
    OtaConfig {
        manufacturer_code: OTA_MANUFACTURER_CODE,
        image_type: OTA_IMAGE_TYPE,
        current_version,
        endpoint: ENDPOINT,
        block_size: 48,
        auto_accept: true,
        hardware_version: Some(1),
    }
}

pub fn sensor_profile(
    firmware_version: u32,
    flash_access: BootloaderFlashAccess,
) -> Result<SensorProfile, SensorProfileError> {
    let base = base_sensor_profile();
    let ota = OtaManager::new(
        Efr32FirmwareWriter::new(flash_access)?,
        ota_config(firmware_version),
    );
    Ok(WithOta::new(base, ota)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_mac::mock::MockMac;
    use zigbee_runtime::ZigbeeDevice;
    use zigbee_runtime::firmware_writer::MockFirmwareWriter;
    use zigbee_runtime::profile::ApplicationProfile;
    use zigbee_zcl::ClusterId;
    use zigbee_zcl::ZclStatus;
    use zigbee_zcl::data_types::{ZclDataType, ZclValue};
    use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

    fn mock_sensor_profile() -> WithOta<BaseSensorProfile, MockFirmwareWriter> {
        WithOta::new(
            base_sensor_profile(),
            OtaManager::new(MockFirmwareWriter::new(4096), ota_config(1)),
        )
        .unwrap()
    }

    fn extra_report(index: usize) -> ReportingConfig {
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::AttributeId(0x7000 + index as u16),
            data_type: ZclDataType::U16,
            min_interval: 1,
            max_interval: 60,
            reportable_change: Some(ZclValue::U16(1)),
        }
    }

    #[test]
    fn compact_capacity_fits_the_complete_product_profile() {
        assert_eq!(zigbee_runtime::MAX_ENDPOINTS, 2);
        assert_eq!(MAX_REPORT_CONFIGS, 4);

        let profile = mock_sensor_profile();
        let mut device = ZigbeeDevice::builder(MockMac::new([0x11; 8]))
            .endpoint(
                profile.endpoint(),
                profile.profile_id(),
                profile.device_id(),
                |endpoint| profile.configure_endpoint(endpoint),
            )
            .build();

        assert_eq!(device.endpoints().len(), APPLICATION_ENDPOINT_COUNT);
        assert_eq!(
            device.endpoints()[0].server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::POWER_CONFIG,
                ClusterId::TEMPERATURE,
                ClusterId::HUMIDITY,
            ]
        );
        assert_eq!(
            device.endpoints()[0].client_clusters.as_slice(),
            &[ClusterId::OTA_UPGRADE]
        );
        assert_eq!(
            device.bdb().zdo().endpoints().len(),
            APPLICATION_ENDPOINT_COUNT
        );

        profile.configure_default_reporting(&mut device).unwrap();
        assert_eq!(
            device.reporting().configured_cluster_count(ENDPOINT),
            DEFAULT_REPORT_CONFIG_COUNT
        );

        for index in DEFAULT_REPORT_CONFIG_COUNT..MAX_REPORT_CONFIGS {
            assert_eq!(
                device.reporting_mut().configure_for_cluster(
                    ENDPOINT,
                    0x7000 + index as u16,
                    extra_report(index),
                ),
                Ok(())
            );
        }
        assert_eq!(
            device.reporting_mut().configure_for_cluster(
                ENDPOINT,
                0x7FFF,
                extra_report(MAX_REPORT_CONFIGS),
            ),
            Err(ZclStatus::InsufficientSpace)
        );
    }
}
