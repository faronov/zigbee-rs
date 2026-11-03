//! TB-04 sleepy temperature/humidity sensor: identity, policy, endpoint,
//! data source and status LEDs.
//!
//! TB-04 has no environmental sensor fitted, so the data source is the
//! deterministic synthetic sequence of the original bring-up image. The
//! TLSR8258 sleep/retention machinery lives in `tlsr8258-sensor-app`.

use core::convert::Infallible;

use sensor_sed_app::{
    ButtonPolicy, EnvironmentReading, EnvironmentSink, EnvironmentSource, FixedBattery,
    NoUserAction, NonOtaComponent, SensorPolicy, SleepDepth, StatusPolicy,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::MacDriver;
use zigbee_runtime::profile::{
    ApplicationClusters, BatteryMeasurement, DeviceProfile, ExpectedReportClusters,
    ProfileComponent, ProfileError, TemperatureHumidityMeasurement,
};
use zigbee_runtime::synthetic_sensor::SyntheticSensor;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{MAX_REPORT_CONFIGS, ReportDirection, ReportingConfig};
use zigbee_zcl::{ClusterId, DeviceId};

#[cfg(target_arch = "tc32")]
use sensor_sed_app::{SensorStatus, StatusSink};
#[cfg(target_arch = "tc32")]
use tlsr8258_tb04::leds::StatusLeds;

pub const ENDPOINT: u8 = 1;
pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "TLSR8258-Runtime";
pub const DATE_CODE: &str = "20260718";
pub const SW_BUILD: &str = "0.1.0";
/// First-byte offset applied to the factory EUI-64. Preserves the IEEE
/// address (and so the journal and ZHA identity) of the proven sensor image.
pub const DEVICE_EUI_OFFSET: u8 = 0x33;

const APPLICATION_ENDPOINT_COUNT: usize = 1;
const SERVER_CLUSTER_COUNT: usize = 5;
const DEFAULT_REPORT_CONFIG_COUNT: usize = 2;

const _: () = {
    assert!(APPLICATION_ENDPOINT_COUNT <= zigbee_runtime::MAX_ENDPOINTS);
    assert!(SERVER_CLUSTER_COUNT <= zigbee_runtime::MAX_CLUSTERS_PER_ENDPOINT);
    assert!(DEFAULT_REPORT_CONFIG_COUNT <= MAX_REPORT_CONFIGS);
};

pub const USER_ACTIONS: NoUserAction = NoUserAction;

/// 250 ms polling and a 30-second synthetic sample cadence. Joining and
/// interview waits stay Active; joined steady-state waits use full-SRAM
/// SUSPEND (`Idle`), or LOW32K reset-on-wake with `retention-proof`.
/// `retention-proof-10s` lengthens only the joined slow poll to ten seconds.
pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 30_000,
    fast_poll_ms: 250,
    slow_poll_ms: if cfg!(feature = "retention-proof-10s") {
        10_000
    } else {
        250
    },
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 120_000,
    wake_duration_ms: 500,
    join_retry_ms: 5_000,
    announce_retry_ms: 8_000,
    // BDB steering already owns the bounded Device_annce retry sequence.
    announce_retries: 0,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    button: ButtonPolicy {
        long_press_ms: None,
        debounce_ms: 300,
    },
    status: StatusPolicy {
        // Red is held steadily while unjoined; the shared blink deadline is
        // effectively disabled.
        unjoined_blink_period_ms: u32::MAX,
        blink_on_ms: 1,
        blink_gap_ms: 1,
        reset_blinks: 1,
        reset_phase_ms: 1,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: if cfg!(feature = "retention-proof") {
        SleepDepth::Retention
    } else {
        SleepDepth::Idle
    },
};

const SYNTHETIC_SENSOR: SyntheticSensor = SyntheticSensor::new(2_150, 100, 5_000, 400);

/// Deterministic source matching the previous `SyntheticSensor` sequence.
pub struct SyntheticEnvironment {
    sample_index: u32,
}

impl SyntheticEnvironment {
    pub const fn new() -> Self {
        Self { sample_index: 0 }
    }

    fn next_reading(&mut self) -> EnvironmentReading {
        let reading = SYNTHETIC_SENSOR.sample(self.sample_index);
        self.sample_index = self.sample_index.wrapping_add(1);
        EnvironmentReading {
            temperature_centi_celsius: reading.temperature_centidegrees,
            humidity_centi_percent: reading.humidity_centipercent,
            pressure_tenth_kpa: None,
        }
    }
}

impl Default for SyntheticEnvironment {
    fn default() -> Self {
        Self::new()
    }
}

impl EnvironmentSource for SyntheticEnvironment {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        Ok(self.next_reading())
    }
}

/// Fixed 3.0 V / 100% battery state used by the existing synthetic image.
pub const fn fixed_battery() -> FixedBattery {
    FixedBattery::new(
        3_000,
        BatteryMeasurement {
            voltage_100mv: 30,
            percentage_remaining: 200,
        },
    )
}

/// Exact cluster storage and local reporting defaults of the pre-migration
/// TLSR8258 sensor.
pub struct SensorClusters {
    power: PowerConfigCluster,
    temperature: TemperatureCluster,
    humidity: HumidityCluster,
}

impl SensorClusters {
    fn new() -> Self {
        let mut power = PowerConfigCluster::new();
        power.set_battery_voltage(30);
        power.set_battery_percentage(200);
        power.set_battery_size(0x04);
        power.set_battery_quantity(2);
        power.set_battery_rated_voltage(15);

        let mut temperature = TemperatureCluster::new(-4_000, 12_500);
        temperature.set_temperature(2_150);
        let mut humidity = HumidityCluster::new(0, 10_000);
        humidity.set_humidity(5_000);

        Self {
            power,
            temperature,
            humidity,
        }
    }
}

impl ProfileComponent for SensorClusters {
    fn configure_endpoint(
        &self,
        endpoint: zigbee_runtime::builder::EndpointBuilder,
    ) -> zigbee_runtime::builder::EndpointBuilder {
        endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::POWER_CONFIG)
            .cluster_server(ClusterId::IDENTIFY)
            .cluster_server(ClusterId::TEMPERATURE)
            .cluster_server(ClusterId::HUMIDITY)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.power,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.temperature,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.humidity,
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    fn expected_report_cluster_ids(&self, out: &mut ExpectedReportClusters) {
        let _ = out.push(ClusterId::TEMPERATURE.0);
        let _ = out.push(ClusterId::HUMIDITY.0);
    }

    fn configure_default_reporting<M: MacDriver, R: zigbee_runtime::role::DeviceRole>(
        &self,
        endpoint: u8,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        let reporting = device.reporting_mut();
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::TEMPERATURE.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE,
                    data_type: ZclDataType::I16,
                    min_interval: 1,
                    max_interval: 60,
                    reportable_change: Some(ZclValue::I16(1)),
                },
            )
            .map_err(ProfileError::Reporting)?;
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::HUMIDITY.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: zigbee_zcl::clusters::humidity::ATTR_MEASURED_VALUE,
                    data_type: ZclDataType::U16,
                    min_interval: 1,
                    max_interval: 60,
                    reportable_change: Some(ZclValue::U16(1)),
                },
            )
            .map_err(ProfileError::Reporting)
    }
}

impl EnvironmentSink for SensorClusters {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.temperature
            .set_temperature(measurement.temperature_centi_celsius);
        self.humidity
            .set_humidity(measurement.humidity_centi_percent);
    }

    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.power.set_battery_voltage(measurement.voltage_100mv);
        self.power
            .set_battery_percentage(measurement.percentage_remaining);
    }
}

impl NonOtaComponent for SensorClusters {}

pub type SensorProfile = DeviceProfile<SensorClusters>;

pub fn sensor_profile() -> SensorProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::TEMPERATURE_SENSOR,
        SensorClusters::new(),
    )
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
const RGB_RED: u8 = 1 << 0;
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
const RGB_GREEN: u8 = 1 << 1;
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
const RGB_BLUE: u8 = 1 << 2;

/// Semantic mapping for the three fitted LEDs.
#[cfg(target_arch = "tc32")]
pub struct SensorRgbStatus {
    leds: StatusLeds,
}

#[cfg(target_arch = "tc32")]
impl SensorRgbStatus {
    pub const fn new(leds: StatusLeds) -> Self {
        Self { leds }
    }

    fn set_rgb(&self, red: bool, green: bool, blue: bool) {
        self.leds.red.write(red);
        self.leds.green.write(green);
        self.leds.blue.write(blue);
        #[cfg(feature = "retention-proof")]
        tlsr8258_sensor_app::retention::record_indicator(rgb_bits(red, green, blue));
    }
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
const fn rgb_bits(red: bool, green: bool, blue: bool) -> u8 {
    red as u8 * RGB_RED | green as u8 * RGB_GREEN | blue as u8 * RGB_BLUE
}

#[cfg(target_arch = "tc32")]
impl StatusSink for SensorRgbStatus {
    fn set(&mut self, status: SensorStatus) {
        match status {
            SensorStatus::Off
            | SensorStatus::Joining { .. }
            | SensorStatus::Resetting { .. }
            | SensorStatus::Fault => self.set_rgb(true, false, false),
            SensorStatus::Joined { .. } => self.set_rgb(false, true, false),
            SensorStatus::Identifying { on } | SensorStatus::Reporting { on } => {
                self.set_rgb(false, true, on)
            }
            SensorStatus::Ota => self.set_rgb(false, false, true),
        }
    }
}

/// After a LOW32K wake the LEDs show the last recorded state; a fail-closed
/// reset shows red.
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
impl tlsr8258_sensor_app::retention::RetainedIndicator for SensorRgbStatus {
    const RESET_STATE: u8 = RGB_RED;

    fn restore(state: u8) -> Result<(), tlsr8258_hal::gpio::GpioError> {
        let state = tlsr8258_tb04::leds::StatusLedState::new(
            state & RGB_RED != 0,
            state & RGB_GREEN != 0,
            state & RGB_BLUE != 0,
        );
        // SAFETY: reset discarded the only executing future that can access
        // the retained StatusLeds owner. This runs with IRQs disabled and
        // completes before the fresh root borrows it.
        unsafe { StatusLeds::restore_after_reset(state) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_mac::mock::MockMac;
    use zigbee_runtime::profile::ApplicationProfile;
    use zigbee_zcl::ZclStatus;

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

        let profile = sensor_profile();
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.profile_id(), PROFILE_HOME_AUTOMATION);
        assert_eq!(profile.device_id(), DeviceId::TEMPERATURE_SENSOR);
        let mut expected = ExpectedReportClusters::new();
        profile.expected_report_cluster_ids(&mut expected);
        assert_eq!(
            expected.as_slice(),
            [ClusterId::TEMPERATURE.0, ClusterId::HUMIDITY.0]
        );

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
                ClusterId::POWER_CONFIG,
                ClusterId::IDENTIFY,
                ClusterId::TEMPERATURE,
                ClusterId::HUMIDITY,
            ]
        );
        assert!(device.endpoints()[0].client_clusters.is_empty());
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

    #[test]
    fn policy_keeps_fast_active_and_selects_explicit_slow_proof() {
        assert!(SENSOR_POLICY.is_valid());
        assert_eq!(SENSOR_POLICY.sample_interval_ms, 30_000);
        assert_eq!(SENSOR_POLICY.fast_poll_ms, 250);
        #[cfg(not(feature = "retention-proof-10s"))]
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 250);
        #[cfg(feature = "retention-proof-10s")]
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 10_000);
        assert_eq!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Active);
        #[cfg(not(feature = "retention-proof"))]
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Idle);
        #[cfg(feature = "retention-proof")]
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Retention);
    }

    #[test]
    fn synthetic_sequence_starts_with_the_existing_sample_zero() {
        let mut source = SyntheticEnvironment::new();
        assert_eq!(
            source.next_reading(),
            EnvironmentReading {
                temperature_centi_celsius: 2_050,
                humidity_centi_percent: 5_100,
                pressure_tenth_kpa: None,
            }
        );
    }

    #[test]
    fn fixed_battery_identity_is_three_volts_and_full() {
        let reading = sensor_sed_app::BatteryReading {
            millivolts: 3_000,
            measurement: BatteryMeasurement {
                voltage_100mv: 30,
                percentage_remaining: 200,
            },
        };
        assert_eq!(reading.millivolts, 3_000);
        assert_eq!(reading.measurement.voltage_100mv, 30);
        assert_eq!(reading.measurement.percentage_remaining, 200);
    }
}
