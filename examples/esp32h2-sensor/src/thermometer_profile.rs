//! Thermometer build profile: endpoint, reporting defaults, sleepy policy
//! and the die-temperature measurement.
//!
//! Every number comes from the build profile (`thermometer.toml`, emitted by
//! `build.rs` as [`config`]); this module only maps it onto the shared
//! runtime and sleepy-sensor contracts.

use esp32_zigbee_devkit_product as product;
use sensor_sed_app::{
    ButtonPolicy, DiagnosticEvent, Diagnostics, EnvironmentSink, RecoveryPolicy, SampleTrigger,
    SensorMeasurement, SensorPolicy, SleepDepth,
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
use zigbee_zcl::clusters::poll_control::{PollControlCluster, PollControlConfig};
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
    fast_sleep_depth: if cfg!(feature = "light-sleep") && config::FAST_POLL_LIGHT_SLEEP {
        SleepDepth::Idle
    } else {
        SleepDepth::Active
    },
    slow_sleep_depth: if cfg!(feature = "light-sleep") {
        SleepDepth::Idle
    } else {
        SleepDepth::Active
    },
    join_retry_ms: config::JOIN_RETRY_MS,
    secure_rejoin_failure_limit: 0,
    recovery: RecoveryPolicy {
        join_retry_max_ms: config::JOIN_RETRY_MAX_MS,
        join_retry_jitter_pct: config::JOIN_RETRY_JITTER_PCT,
        parent_loss_strikes: config::PARENT_LOSS_STRIKES,
        max_active_ms: config::MAX_ACTIVE_MS,
        report_aps_ack: config::REPORT_APS_ACK,
        max_ota_awake_ms: config::MAX_OTA_AWAKE_MS,
        max_identify_awake_ms: config::MAX_IDENTIFY_AWAKE_MS,
        max_fast_poll_ms: config::MAX_FAST_POLL_MS,
        missed_report_grace_ms: config::MISSED_REPORT_GRACE_MS,
        sleep_failure_limit: config::SLEEP_FAILURE_LIMIT,
    },
    ..product::policy::SENSOR_POLICY
};

/// Poll Control server values and limits from the build profile.
pub const POLL_CONTROL: PollControlConfig = PollControlConfig {
    check_in_interval: config::CHECK_IN_INTERVAL_QS,
    long_poll_interval: config::LONG_POLL_INTERVAL_QS,
    short_poll_interval: config::SHORT_POLL_INTERVAL_QS,
    fast_poll_timeout: config::FAST_POLL_TIMEOUT_QS,
    check_in_interval_min: config::CHECK_IN_INTERVAL_MIN_QS,
    long_poll_interval_min: config::LONG_POLL_MIN_QS,
    fast_poll_timeout_max: config::FAST_POLL_TIMEOUT_MAX_QS,
    long_poll_interval_max: config::LONG_POLL_MAX_QS,
    check_in_response_timeout: config::CHECK_IN_RESPONSE_QS,
};

const _: () = assert!(POLL_CONTROL.is_valid());
const _: () = assert!(config::LONG_POLL_MAX_QS * 250 <= sensor_sed_app::policy::MAX_WAIT_MS);

// No status LED is fitted (`NoStatus`), and the slow poll must fit the
// shared application's bounded wait.
const _: () = assert!(POLICY.is_valid_for_status(false));
const _: () = assert!(config::SLOW_POLL_MS <= sensor_sed_app::policy::MAX_WAIT_MS);

/// Basic + Identify + Poll Control + Temperature Measurement server, plus
/// the anomaly-record Diagnostics server in `anomaly-record` builds.
pub struct Thermometer {
    temperature: TemperatureCluster,
    poll_control: PollControlCluster,
    #[cfg(feature = "anomaly-record")]
    anomalies: crate::thermometer_anomaly::AnomalyDiagnostics,
}

impl Thermometer {
    pub const EXPECTED_REPORT_CLUSTER_IDS: [u16; 1] = [ClusterId::TEMPERATURE.0];

    pub fn new() -> Self {
        Self {
            temperature: TemperatureCluster::new(
                config::MIN_MEASURED_CENTI_C,
                config::MAX_MEASURED_CENTI_C,
            ),
            poll_control: PollControlCluster::with_config(POLL_CONTROL),
            #[cfg(feature = "anomaly-record")]
            anomalies: crate::thermometer_anomaly::AnomalyDiagnostics::new(),
        }
    }

    #[cfg(feature = "anomaly-record")]
    pub fn anomalies_mut(&mut self) -> &mut crate::thermometer_anomaly::AnomalyDiagnostics {
        &mut self.anomalies
    }

    pub fn set_temperature(&mut self, centi_celsius: i16) {
        self.temperature.set_temperature(centi_celsius);
    }
}

impl ProfileComponent for Thermometer {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        let endpoint = endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY)
            .cluster_server(ClusterId::POLL_CONTROL)
            .cluster_server(self.temperature.cluster_id());
        #[cfg(feature = "anomaly-record")]
        let endpoint = endpoint.cluster_server(ClusterId::DIAGNOSTICS);
        endpoint
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
            .map_err(|_| ProfileError::TooManyClusters)?;
        #[cfg(feature = "anomaly-record")]
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.anomalies,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.poll_control,
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
        #[cfg(feature = "anomaly-record")]
        crate::thermometer_anomaly::configure_reporting(endpoint, device)?;
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

    fn poll_control_mut(&mut self) -> Option<&mut PollControlCluster> {
        Some(&mut self.poll_control)
    }
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
