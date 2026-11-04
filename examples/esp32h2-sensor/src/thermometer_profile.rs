//! Thermometer build profile: endpoint, reporting defaults, sleepy policy
//! and the die-temperature measurement.
//!
//! Every number comes from the build profile (`thermometer.toml`, emitted by
//! `build.rs` as [`config`]); this module only maps it onto the shared
//! runtime and sleepy-sensor contracts.

use esp32_zigbee_devkit_product as product;
use sensor_sed_app::{
    ButtonPolicy, DiagnosticEvent, Diagnostics, EnvironmentSink, SampleTrigger, SensorMeasurement,
    SensorPolicy, SleepDepth,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::MacDriver;
use zigbee_runtime::builder::EndpointBuilder;
use zigbee_runtime::profile::{
    ApplicationClusters, BatteryMeasurement, DeviceProfile, ExpectedReportClusters,
    ProfileComponent, ProfileError, TemperatureHumidityMeasurement, WithOta,
};
use zigbee_runtime::role::DeviceRole;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_zcl::clusters::temperature::{TemperatureCluster, ATTR_MEASURED_VALUE};
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
use zigbee_zcl::{ClusterId, DeviceId};

use crate::chip_temperature::H2TemperatureSensor;

/// Constants generated from the thermometer build profile.
pub mod config {
    include!(concat!(env!("OUT_DIR"), "/thermometer_config.rs"));
}

/// Sleepy policy: the product's commissioning/button/announce behavior with
/// the build profile's poll and sample cadence.
pub static POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: config::SLOW_POLL_MS * config::SAMPLE_EVERY_POLLS,
    fast_poll_ms: config::FAST_POLL_MS,
    slow_poll_ms: config::SLOW_POLL_MS,
    fresh_join_fast_ms: config::FRESH_JOIN_FAST_MS,
    restored_fast_ms: config::RESTORED_FAST_MS,
    interview_complete_grace_ms: config::BUTTON_FAST_POLL_MS,
    button: ButtonPolicy {
        long_press_ms: product::policy::SENSOR_POLICY.button.long_press_ms,
        debounce_ms: product::policy::SENSOR_POLICY.button.debounce_ms,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: if cfg!(feature = "light-sleep") {
        SleepDepth::Idle
    } else {
        SleepDepth::Active
    },
    ..product::policy::SENSOR_POLICY
};

// No status LED is fitted (`NoStatus`), and the slow poll must fit the
// shared application's bounded wait.
const _: () = assert!(POLICY.is_valid_for_status(false));
const _: () = assert!(config::SLOW_POLL_MS <= sensor_sed_app::policy::MAX_WAIT_MS);

/// Basic + Identify + Temperature Measurement server.
pub struct Thermometer {
    temperature: TemperatureCluster,
}

impl Thermometer {
    pub const EXPECTED_REPORT_CLUSTER_IDS: [u16; 1] = [ClusterId::TEMPERATURE.0];

    pub fn new() -> Self {
        Self {
            temperature: TemperatureCluster::new(
                config::MIN_MEASURED_CENTI_C,
                config::MAX_MEASURED_CENTI_C,
            ),
        }
    }

    pub fn set_temperature(&mut self, centi_celsius: i16) {
        self.temperature.set_temperature(centi_celsius);
    }
}

impl ProfileComponent for Thermometer {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY)
            .cluster_server(self.temperature.cluster_id())
    }

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.temperature,
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    fn expected_report_cluster_ids(&self, out: &mut ExpectedReportClusters) {
        let _ = out.push(Self::EXPECTED_REPORT_CLUSTER_IDS[0]);
    }

    /// Boot-time default only: a remote Configure Reporting (e.g. the ZHA
    /// interview) replaces it for the rest of the power cycle.
    fn configure_default_reporting<M: MacDriver, R: DeviceRole>(
        &self,
        endpoint: u8,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        device
            .reporting_mut()
            .configure_for_cluster(
                endpoint,
                ClusterId::TEMPERATURE.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_MEASURED_VALUE,
                    data_type: ZclDataType::I16,
                    min_interval: config::REPORT_MIN_S,
                    max_interval: config::REPORT_MAX_S,
                    reportable_change: Some(ZclValue::I16(config::REPORT_CHANGE_CENTI_C)),
                },
            )
            .map_err(ProfileError::Reporting)
    }
}

/// Makes the profile a shared-application `SensorProfile`. There is no
/// humidity or Power Configuration cluster: those inputs are discarded.
impl EnvironmentSink for Thermometer {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.set_temperature(measurement.temperature_centi_celsius);
    }

    fn update_battery(&mut self, _measurement: BatteryMeasurement) {}
}

pub type ThermoProfile =
    WithOta<DeviceProfile<Thermometer>, product::ota::EspFirmwareWriter<product::ota::EspOtaFlash>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThermoProfileError {
    OtaInitialization(product::ota::OtaInitError),
    Profile(ProfileError),
}

/// Temperature Sensor endpoint plus the mandatory OTA Upgrade client, using
/// the product's OTA partition layout and this build's image type.
pub fn thermo_profile(
    firmware_version: u32,
    reset: fn() -> !,
) -> Result<ThermoProfile, ThermoProfileError> {
    let writer = product::ota::EspFirmwareWriter::new(product::ota::EspOtaFlash::new(), reset)
        .map_err(ThermoProfileError::OtaInitialization)?;
    let config = zigbee_runtime::ota::OtaConfig {
        manufacturer_code: product::OTA_MANUFACTURER_CODE,
        image_type: config::OTA_IMAGE_TYPE,
        current_version: firmware_version,
        endpoint: product::ENDPOINT,
        block_size: 48,
        auto_accept: true,
        hardware_version: Some(product::OTA_HARDWARE_VERSION),
    };
    WithOta::new(
        DeviceProfile::new(
            product::ENDPOINT,
            PROFILE_HOME_AUTOMATION,
            DeviceId::TEMPERATURE_SENSOR,
            Thermometer::new(),
        ),
        zigbee_runtime::ota::OtaManager::new(writer, config),
    )
    .map_err(ThermoProfileError::Profile)
}

/// eFuse-calibrated die temperature, sampled only when the shared lifecycle
/// asks (cold start, every `sample_every_polls` slow polls, user request).
pub struct DieThermometer {
    sensor: H2TemperatureSensor,
}

impl DieThermometer {
    pub const fn new(sensor: H2TemperatureSensor) -> Self {
        Self { sensor }
    }
}

impl SensorMeasurement<ThermoProfile> for DieThermometer {
    async fn measure<D: Diagnostics>(
        &mut self,
        profile: &mut ThermoProfile,
        _trigger: SampleTrigger,
        diagnostics: &mut D,
    ) {
        let cluster_id = ClusterId::TEMPERATURE.0;
        match self.sensor.read_gated_centi_celsius() {
            Ok(value) => {
                profile.inner_mut().component_mut().set_temperature(value);
                diagnostics.record(DiagnosticEvent::Measurement {
                    cluster_id,
                    value: i32::from(value),
                });
            }
            Err(_) => diagnostics.record(DiagnosticEvent::MeasurementFailed { cluster_id }),
        }
    }
}
