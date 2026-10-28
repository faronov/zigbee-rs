//! Endpoint profile derived at runtime from the detected sensor set.
//!
//! [`flower_profile`] is the only place the endpoint shape is decided:
//!
//! | detected            | Simple Descriptor server clusters                   |
//! |---------------------|-----------------------------------------------------|
//! | no light sensor     | Basic, Identify, PowerConfig, SoilMoisture, Temp    |
//! | BH1750 and/or OPT3001 | ... plus Illuminance (0x0400)                     |
//!
//! The Temperature cluster is always present; its *source* (TMP102 or the
//! BATMON die sensor) is a measurement detail, see
//! [`crate::measurement::FlowerMeasurement`]. The same
//! [`PlantSensor`](zigbee_runtime::profile::PlantSensor) component then
//! drives the endpoint registration (and therefore the ZDO Simple
//! Descriptor), the ZCL cluster dispatch list, the expected-report set
//! used by the interview tracker, and the default reporting table, so
//! none of these can disagree about Illuminance.

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, PlantSensor, PlantSensorReporting, TemperatureRange};
use zigbee_zcl::DeviceId;
use zigbee_zcl::clusters::illuminance::measured_value_from_centi_lux;

use crate::ENDPOINT;
use crate::battery::BATTERY;
use crate::detection::{DetectedSensors, LightSensor};

pub type FlowerProfile = DeviceProfile<PlantSensor>;

/// TMP102 operating range; also covers the CC2340 die sensor.
pub const TEMPERATURE_RANGE: TemperatureRange = TemperatureRange {
    min_centi_celsius: -4_000,
    max_centi_celsius: 12_500,
};

/// OPT3001 full scale: 83 865.60 lx.
pub const OPT3001_MAX_CENTI_LUX: u32 = 8_386_560;
/// BH1750 high-resolution full scale: 65 535 / 1.2 = 54 612.5 lx.
pub const BH1750_MAX_CENTI_LUX: u32 = 5_461_250;

impl LightSensor {
    /// ZCL `MaxMeasuredValue` for this sensor's full scale.
    pub fn max_measured_value(self) -> u16 {
        measured_value_from_centi_lux(match self {
            LightSensor::Opt3001 => OPT3001_MAX_CENTI_LUX,
            LightSensor::Bh1750 => BH1750_MAX_CENTI_LUX,
        })
    }
}

/// Build the product endpoint for `sensors`.
pub fn flower_profile(sensors: DetectedSensors) -> FlowerProfile {
    let light = sensors.light_sensor();
    // Without a light sensor the maximum is irrelevant: the cluster is
    // removed below.
    let illuminance_max = light.map_or(0, LightSensor::max_measured_value);
    let mut component = PlantSensor::new(
        TEMPERATURE_RANGE,
        illuminance_max,
        PlantSensorReporting::default(),
    )
    .with_battery(BATTERY);
    if light.is_none() {
        component = component.without_illuminance();
    }
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::SIMPLE_SENSOR,
        component,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detection::TemperatureSource;
    use zigbee_mac::mock::MockMac;
    use zigbee_runtime::ZigbeeDevice;
    use zigbee_runtime::profile::{
        ApplicationClusters, ApplicationProfile, ExpectedReportClusters,
    };
    use zigbee_zcl::ClusterId;
    use zigbee_zcl::clusters::Cluster;
    use zigbee_zcl::data_types::ZclValue;
    use zigbee_zcl::foundation::reporting::ReportDirection;

    const ALWAYS: [ClusterId; 5] = [
        ClusterId::BASIC,
        ClusterId::IDENTIFY,
        ClusterId::POWER_CONFIG,
        ClusterId::SOIL_MOISTURE,
        ClusterId::TEMPERATURE,
    ];

    struct Expected {
        illuminance: bool,
        temperature: TemperatureSource,
        light: Option<LightSensor>,
    }

    /// Build the device exactly as the firmware does and check every place
    /// the cluster set is visible.
    fn check(sensors: DetectedSensors, expected: Expected) {
        let mut profile = flower_profile(sensors);
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.device_id(), DeviceId::SIMPLE_SENSOR);
        assert_eq!(sensors.temperature_source(), expected.temperature);
        assert_eq!(sensors.light_sensor(), expected.light);

        // Simple Descriptor (the builder registers these server clusters
        // with ZDO verbatim).
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(
                profile.endpoint(),
                profile.profile_id(),
                profile.device_id(),
                |endpoint| profile.configure_endpoint(endpoint),
            )
            .build();
        let endpoints = device.endpoints();
        assert_eq!(endpoints.len(), 1);
        let mut clusters = heapless::Vec::<ClusterId, 8>::new();
        clusters.extend_from_slice(&ALWAYS).unwrap();
        if expected.illuminance {
            clusters.push(ClusterId::ILLUMINANCE).unwrap();
        }
        assert_eq!(endpoints[0].server_clusters.as_slice(), clusters.as_slice());
        assert!(endpoints[0].client_clusters.is_empty());

        // ZCL dispatch: Read/Configure Reporting for 0x0400 can only reach
        // a cluster instance that exists.
        {
            let mut dispatch = ApplicationClusters::new();
            profile.collect_clusters(&mut dispatch).unwrap();
            let has_illuminance_instance = dispatch
                .iter()
                .any(|cluster| cluster.cluster.cluster_id() == ClusterId::ILLUMINANCE);
            assert_eq!(has_illuminance_instance, expected.illuminance);
            // Soil, temperature, power configuration (+ illuminance); Basic
            // and Identify are runtime-owned.
            assert_eq!(dispatch.len(), if expected.illuminance { 4 } else { 3 });
        }

        // Interview tracking: reports the coordinator is expected to bind
        // and configure.
        let mut reports = ExpectedReportClusters::new();
        profile.expected_report_cluster_ids(&mut reports);
        let mut want = heapless::Vec::<u16, 4>::new();
        want.extend_from_slice(&[ClusterId::SOIL_MOISTURE.0, ClusterId::TEMPERATURE.0])
            .unwrap();
        if expected.illuminance {
            want.push(ClusterId::ILLUMINANCE.0).unwrap();
        }
        want.push(ClusterId::POWER_CONFIG.0).unwrap();
        assert_eq!(reports.as_slice(), want.as_slice());

        // Default reporting table.
        profile.configure_default_reporting(&mut device).unwrap();
        assert_eq!(
            device.configured_cluster_count(ENDPOINT),
            if expected.illuminance { 4 } else { 3 }
        );
        let illuminance_config = device.reporting().get_config(
            ENDPOINT,
            ClusterId::ILLUMINANCE.0,
            ReportDirection::Send,
            zigbee_zcl::clusters::illuminance::ATTR_MEASURED_VALUE,
        );
        assert_eq!(illuminance_config.is_some(), expected.illuminance);
        for (cluster, attribute) in [
            (
                ClusterId::SOIL_MOISTURE,
                zigbee_zcl::clusters::soil_moisture::ATTR_MEASURED_VALUE,
            ),
            (
                ClusterId::TEMPERATURE,
                zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE,
            ),
        ] {
            assert!(
                device
                    .reporting()
                    .get_config(ENDPOINT, cluster.0, ReportDirection::Send, attribute)
                    .is_some()
            );
        }

        // Illuminance limits follow the detected sensor.
        let component = profile.component();
        assert_eq!(component.has_illuminance(), expected.illuminance);
        assert!(component.has_battery());
        if let Some(light) = expected.light {
            assert_eq!(
                component
                    .illuminance()
                    .unwrap()
                    .attributes()
                    .get(zigbee_zcl::clusters::illuminance::ATTR_MAX_MEASURED_VALUE),
                Some(&ZclValue::U16(light.max_measured_value()))
            );
        }
    }

    #[test]
    fn no_optional_sensors() {
        check(
            DetectedSensors::NONE,
            Expected {
                illuminance: false,
                temperature: TemperatureSource::DieTemperature,
                light: None,
            },
        );
    }

    #[test]
    fn tmp102_only() {
        check(
            DetectedSensors {
                tmp102: true,
                ..DetectedSensors::NONE
            },
            Expected {
                illuminance: false,
                temperature: TemperatureSource::Tmp102,
                light: None,
            },
        );
    }

    #[test]
    fn bh1750_only() {
        check(
            DetectedSensors {
                bh1750: true,
                ..DetectedSensors::NONE
            },
            Expected {
                illuminance: true,
                temperature: TemperatureSource::DieTemperature,
                light: Some(LightSensor::Bh1750),
            },
        );
    }

    #[test]
    fn opt3001_only() {
        check(
            DetectedSensors {
                opt3001: true,
                ..DetectedSensors::NONE
            },
            Expected {
                illuminance: true,
                temperature: TemperatureSource::DieTemperature,
                light: Some(LightSensor::Opt3001),
            },
        );
    }

    #[test]
    fn tmp102_and_light() {
        check(
            DetectedSensors {
                tmp102: true,
                bh1750: true,
                ..DetectedSensors::NONE
            },
            Expected {
                illuminance: true,
                temperature: TemperatureSource::Tmp102,
                light: Some(LightSensor::Bh1750),
            },
        );
        check(
            DetectedSensors {
                tmp102: true,
                opt3001: true,
                ..DetectedSensors::NONE
            },
            Expected {
                illuminance: true,
                temperature: TemperatureSource::Tmp102,
                light: Some(LightSensor::Opt3001),
            },
        );
    }

    #[test]
    fn both_light_sensors_advertise_one_illuminance_cluster() {
        check(
            DetectedSensors {
                tmp102: true,
                bh1750: true,
                opt3001: true,
            },
            Expected {
                illuminance: true,
                temperature: TemperatureSource::Tmp102,
                light: Some(LightSensor::Opt3001),
            },
        );
    }

    #[test]
    fn persisted_set_rebuilds_the_identical_endpoint() {
        // The descriptor is a pure function of the persisted bits.
        for bits in 0..8u8 {
            let sensors = DetectedSensors::from_bits(bits).unwrap();
            let decoded =
                crate::detection::decode_record(&crate::detection::encode_record(sensors)).unwrap();
            let a = flower_profile(sensors);
            let b = flower_profile(decoded);
            let mut ra = ExpectedReportClusters::new();
            let mut rb = ExpectedReportClusters::new();
            a.expected_report_cluster_ids(&mut ra);
            b.expected_report_cluster_ids(&mut rb);
            assert_eq!(ra, rb);
        }
    }

    #[test]
    fn illuminance_maxima_use_the_log_encoding() {
        // 10000*log10(83865.6)+1 = 49237; 10000*log10(54612.5)+1 = 47373.
        assert!(LightSensor::Opt3001.max_measured_value().abs_diff(49_237) <= 1);
        assert!(LightSensor::Bh1750.max_measured_value().abs_diff(47_373) <= 1);
    }
}
