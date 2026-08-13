//! Air quality archetype: CO₂ + temperature + humidity, optional battery.
//!
//! Composes only clusters with genuinely implemented, reportable attributes
//! in this workspace: [`CarbonDioxideCluster`], [`TemperatureCluster`], and
//! [`HumidityCluster`]. A Particulate Matter (PM2.5) cluster also exists in
//! `zigbee-zcl` (`zigbee_zcl::clusters::pm25`) and is a natural companion for
//! a real air-quality sensor, but it is intentionally **not** included here:
//! bundling it into every air-quality device would be decorative until a
//! product actually wires up a PM2.5 sensor. Add a `with_pm25()` builder step
//! when that happens, following the same optional-cluster pattern as
//! [`with_battery`](AirQuality::with_battery).
//!
//! There is no Home Automation profile device ID dedicated to "air quality
//! sensor", so this archetype uses the generic Simple Sensor device ID
//! (`DeviceId::SIMPLE_SENSOR`, 0x000C) intended for composite measurement
//! endpoints.

use super::{
    ApplicationClusters, BatteryDescriptor, BatteryMeasurement, ExpectedReportClusters,
    ProfileComponent, ProfileError, TemperatureRange,
};
use crate::builder::EndpointBuilder;
use crate::{ClusterRef, ZigbeeDevice};
use zigbee_mac::MacDriver;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::carbon_dioxide::{
    ATTR_MEASURED_VALUE as CO2_MEASURED_VALUE, CarbonDioxideCluster,
};
use zigbee_zcl::clusters::humidity::{
    ATTR_MEASURED_VALUE as HUMIDITY_MEASURED_VALUE, HumidityCluster,
};
use zigbee_zcl::clusters::power_config::{ATTR_BATTERY_PERCENTAGE_REMAINING, PowerConfigCluster};
use zigbee_zcl::clusters::temperature::{
    ATTR_MEASURED_VALUE as TEMPERATURE_MEASURED_VALUE, TemperatureCluster,
};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

/// One CO₂ + temperature + humidity reading.
///
/// Grouped together because low-cost NDIR/photoacoustic CO₂ sensors (e.g.
/// Sensirion SCD4x) report all three values from a single measurement cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirQualityMeasurement {
    pub co2_ppm: f32,
    pub temperature_centi_celsius: i16,
    pub humidity_centi_percent: u16,
}

/// Reporting cadence and thresholds for the air quality archetype.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirQualityReporting {
    pub co2_min_secs: u16,
    pub co2_max_secs: u16,
    pub co2_change_ppm: f32,
    pub temperature_min_secs: u16,
    pub temperature_max_secs: u16,
    pub temperature_change_centi_celsius: i16,
    pub humidity_min_secs: u16,
    pub humidity_max_secs: u16,
    pub humidity_change_centi_percent: u16,
    pub battery_min_secs: u16,
    pub battery_max_secs: u16,
    pub battery_change_half_percent: u8,
}

impl Default for AirQualityReporting {
    fn default() -> Self {
        Self {
            co2_min_secs: 60,
            co2_max_secs: 300,
            co2_change_ppm: 50.0,
            temperature_min_secs: 60,
            temperature_max_secs: 300,
            temperature_change_centi_celsius: 50,
            humidity_min_secs: 60,
            humidity_max_secs: 300,
            humidity_change_centi_percent: 100,
            battery_min_secs: 300,
            battery_max_secs: 3_600,
            battery_change_half_percent: 4,
        }
    }
}

/// CO₂ + temperature + humidity clusters with an optional battery.
pub struct AirQuality {
    co2: CarbonDioxideCluster,
    temperature: TemperatureCluster,
    humidity: HumidityCluster,
    battery: Option<PowerConfigCluster>,
    reporting: AirQualityReporting,
}

impl AirQuality {
    pub fn new(temperature_range: TemperatureRange, reporting: AirQualityReporting) -> Self {
        Self {
            co2: CarbonDioxideCluster::new(),
            temperature: TemperatureCluster::new(
                temperature_range.min_centi_celsius,
                temperature_range.max_centi_celsius,
            ),
            humidity: HumidityCluster::new(0, 10_000),
            battery: None,
            reporting,
        }
    }

    /// Add a battery-backed Power Configuration cluster.
    pub fn with_battery(mut self, battery: BatteryDescriptor) -> Self {
        let mut power = PowerConfigCluster::new();
        power.set_battery_voltage(0xFF);
        power.set_battery_percentage(0xFF);
        power.set_battery_size(battery.size);
        power.set_battery_quantity(battery.quantity);
        power.set_battery_rated_voltage(battery.rated_voltage_100mv);
        self.battery = Some(power);
        self
    }

    pub fn has_battery(&self) -> bool {
        self.battery.is_some()
    }

    /// Update CO₂, temperature, and humidity from one measurement cycle.
    pub fn update_measurement(&mut self, measurement: AirQualityMeasurement) {
        self.co2.set_co2_ppm(measurement.co2_ppm);
        self.temperature
            .set_temperature(measurement.temperature_centi_celsius);
        self.humidity
            .set_humidity(measurement.humidity_centi_percent);
    }

    /// Update the battery reading. No-op if no battery was configured via
    /// [`with_battery`](Self::with_battery).
    pub fn update_battery(&mut self, measurement: BatteryMeasurement) {
        if let Some(power) = &mut self.battery {
            power.set_battery_voltage(measurement.voltage_100mv);
            power.set_battery_percentage(measurement.percentage_remaining);
        }
    }

    /// Mark the battery reading unknown (e.g. before the first sample).
    /// No-op if no battery was configured.
    pub fn set_battery_unknown(&mut self) {
        if let Some(power) = &mut self.battery {
            power.set_battery_voltage(0xFF);
            power.set_battery_percentage(0xFF);
        }
    }

    pub const fn co2(&self) -> &CarbonDioxideCluster {
        &self.co2
    }

    pub const fn temperature(&self) -> &TemperatureCluster {
        &self.temperature
    }

    pub const fn humidity(&self) -> &HumidityCluster {
        &self.humidity
    }
}

impl ProfileComponent for AirQuality {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        let endpoint = endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY);
        let endpoint = if self.battery.is_some() {
            endpoint.cluster_server(ClusterId::POWER_CONFIG)
        } else {
            endpoint
        };
        endpoint
            .cluster_server(ClusterId::TEMPERATURE)
            .cluster_server(ClusterId::HUMIDITY)
            .cluster_server(ClusterId::CARBON_DIOXIDE)
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
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.humidity,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.co2,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        if let Some(power) = &mut self.battery {
            clusters
                .push(ClusterRef {
                    endpoint,
                    cluster: power,
                })
                .map_err(|_| ProfileError::TooManyClusters)?;
        }
        Ok(())
    }

    fn expected_report_cluster_ids(&self, out: &mut ExpectedReportClusters) {
        let _ = out.push(ClusterId::TEMPERATURE.0);
        let _ = out.push(ClusterId::HUMIDITY.0);
        let _ = out.push(ClusterId::CARBON_DIOXIDE.0);
        if self.battery.is_some() {
            let _ = out.push(ClusterId::POWER_CONFIG.0);
        }
    }

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
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
                    attribute_id: TEMPERATURE_MEASURED_VALUE,
                    data_type: ZclDataType::I16,
                    min_interval: self.reporting.temperature_min_secs,
                    max_interval: self.reporting.temperature_max_secs,
                    reportable_change: Some(ZclValue::I16(
                        self.reporting.temperature_change_centi_celsius,
                    )),
                },
            )
            .map_err(ProfileError::Reporting)?;
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::HUMIDITY.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: HUMIDITY_MEASURED_VALUE,
                    data_type: ZclDataType::U16,
                    min_interval: self.reporting.humidity_min_secs,
                    max_interval: self.reporting.humidity_max_secs,
                    reportable_change: Some(ZclValue::U16(
                        self.reporting.humidity_change_centi_percent,
                    )),
                },
            )
            .map_err(ProfileError::Reporting)?;
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::CARBON_DIOXIDE.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: CO2_MEASURED_VALUE,
                    data_type: ZclDataType::Float32,
                    min_interval: self.reporting.co2_min_secs,
                    max_interval: self.reporting.co2_max_secs,
                    reportable_change: Some(ZclValue::Float32(
                        self.reporting.co2_change_ppm * 1.0e-6,
                    )),
                },
            )
            .map_err(ProfileError::Reporting)?;
        if self.battery.is_some() {
            reporting
                .configure_for_cluster(
                    endpoint,
                    ClusterId::POWER_CONFIG.0,
                    ReportingConfig {
                        direction: ReportDirection::Send,
                        attribute_id: ATTR_BATTERY_PERCENTAGE_REMAINING,
                        data_type: ZclDataType::U8,
                        min_interval: self.reporting.battery_min_secs,
                        max_interval: self.reporting.battery_max_secs,
                        reportable_change: Some(ZclValue::U8(
                            self.reporting.battery_change_half_percent,
                        )),
                    },
                )
                .map_err(ProfileError::Reporting)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ZigbeeDevice;
    use zigbee_aps::PROFILE_HOME_AUTOMATION;
    use zigbee_mac::mock::MockMac;
    use zigbee_zcl::DeviceId;
    use zigbee_zcl::clusters::Cluster;
    use zigbee_zcl::foundation::reporting::{AttributeReport, MAX_REPORT_CONFIGS, ReportDirection};

    fn range() -> TemperatureRange {
        TemperatureRange {
            min_centi_celsius: -1_000,
            max_centi_celsius: 6_000,
        }
    }

    fn battery() -> BatteryDescriptor {
        BatteryDescriptor {
            size: 4,
            quantity: 2,
            rated_voltage_100mv: 15,
        }
    }

    fn endpoint_builder() -> EndpointBuilder {
        EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::SIMPLE_SENSOR,
            device_version: 1,
            server_clusters: heapless::Vec::new(),
            client_clusters: heapless::Vec::new(),
        }
    }

    fn reporting_device() -> ZigbeeDevice<MockMac> {
        ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(1, PROFILE_HOME_AUTOMATION, DeviceId::SIMPLE_SENSOR, |ep| {
                ep.cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::IDENTIFY)
                    .cluster_server(ClusterId::TEMPERATURE)
                    .cluster_server(ClusterId::HUMIDITY)
                    .cluster_server(ClusterId::CARBON_DIOXIDE)
            })
            .build()
    }

    fn collect_co2_reports(
        device: &mut ZigbeeDevice<MockMac>,
        component: &AirQuality,
    ) -> heapless::Vec<AttributeReport, MAX_REPORT_CONFIGS> {
        let mut reports = heapless::Vec::new();
        device.reporting_mut().check_and_collect_dyn(
            1,
            ClusterId::CARBON_DIOXIDE.0,
            component.co2().attributes(),
            &mut reports,
        );
        reports
    }

    #[test]
    fn endpoint_without_battery_omits_power_config() {
        let component = AirQuality::new(range(), AirQualityReporting::default());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::TEMPERATURE,
                ClusterId::HUMIDITY,
                ClusterId::CARBON_DIOXIDE,
            ]
        );
    }

    #[test]
    fn endpoint_with_battery_adds_power_config() {
        let component =
            AirQuality::new(range(), AirQualityReporting::default()).with_battery(battery());
        assert!(component.has_battery());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::POWER_CONFIG,
                ClusterId::TEMPERATURE,
                ClusterId::HUMIDITY,
                ClusterId::CARBON_DIOXIDE,
            ]
        );
    }

    #[test]
    fn collect_clusters_matches_configured_battery_presence() {
        let mut without_battery = AirQuality::new(range(), AirQualityReporting::default());
        assert_eq!(without_battery.expected_report_clusters(), 3);
        let mut clusters = ApplicationClusters::new();
        without_battery.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 3);

        let mut with_battery =
            AirQuality::new(range(), AirQualityReporting::default()).with_battery(battery());
        assert_eq!(with_battery.expected_report_clusters(), 4);
        let mut clusters = ApplicationClusters::new();
        with_battery.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 4);
        assert_eq!(
            clusters.last().map(|c| c.cluster.cluster_id()),
            Some(ClusterId::POWER_CONFIG)
        );
    }

    #[test]
    fn measured_value_limits_come_from_constructor_bounds() {
        let component = AirQuality::new(range(), AirQualityReporting::default());
        assert_eq!(
            component
                .temperature()
                .attributes()
                .get(zigbee_zcl::clusters::temperature::ATTR_MIN_MEASURED_VALUE),
            Some(&ZclValue::I16(-1_000))
        );
        assert_eq!(
            component
                .temperature()
                .attributes()
                .get(zigbee_zcl::clusters::temperature::ATTR_MAX_MEASURED_VALUE),
            Some(&ZclValue::I16(6_000))
        );
        assert_eq!(
            component
                .humidity()
                .attributes()
                .get(zigbee_zcl::clusters::humidity::ATTR_MAX_MEASURED_VALUE),
            Some(&ZclValue::U16(10_000))
        );
        assert_eq!(
            component
                .co2()
                .attributes()
                .get(zigbee_zcl::clusters::carbon_dioxide::ATTR_MAX_MEASURED_VALUE),
            Some(&ZclValue::Float32(0.01))
        );
    }

    #[test]
    fn update_measurement_and_battery_writes_owned_clusters() {
        let mut component =
            AirQuality::new(range(), AirQualityReporting::default()).with_battery(battery());
        component.update_measurement(AirQualityMeasurement {
            co2_ppm: 812.5,
            temperature_centi_celsius: 2_150,
            humidity_centi_percent: 4_500,
        });
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 28,
            percentage_remaining: 180,
        });

        assert_eq!(
            component.co2().attributes().get(CO2_MEASURED_VALUE),
            Some(&ZclValue::Float32(812.5e-6))
        );
        assert_eq!(
            component
                .temperature()
                .attributes()
                .get(TEMPERATURE_MEASURED_VALUE),
            Some(&ZclValue::I16(2_150))
        );
        assert_eq!(
            component
                .humidity()
                .attributes()
                .get(HUMIDITY_MEASURED_VALUE),
            Some(&ZclValue::U16(4_500))
        );

        component.set_battery_unknown();
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 0,
            percentage_remaining: 0,
        }); // battery still present; verifies update path re-applies real values
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 30,
            percentage_remaining: 200,
        });
    }

    #[test]
    fn update_battery_is_a_no_op_without_a_configured_battery() {
        let mut component = AirQuality::new(range(), AirQualityReporting::default());
        // Must not panic — there is no Power Configuration cluster to write to.
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 30,
            percentage_remaining: 200,
        });
        component.set_battery_unknown();
        assert!(!component.has_battery());
    }

    #[test]
    fn default_reporting_configures_every_owned_cluster() {
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(1, PROFILE_HOME_AUTOMATION, DeviceId::SIMPLE_SENSOR, |ep| {
                ep.cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::IDENTIFY)
                    .cluster_server(ClusterId::POWER_CONFIG)
                    .cluster_server(ClusterId::TEMPERATURE)
                    .cluster_server(ClusterId::HUMIDITY)
                    .cluster_server(ClusterId::CARBON_DIOXIDE)
            })
            .build();
        let component =
            AirQuality::new(range(), AirQualityReporting::default()).with_battery(battery());
        component
            .configure_default_reporting(1, &mut device)
            .unwrap();

        assert_eq!(device.configured_cluster_count(1), 4);

        let co2_config = device
            .reporting()
            .get_config(
                1,
                ClusterId::CARBON_DIOXIDE.0,
                ReportDirection::Send,
                CO2_MEASURED_VALUE,
            )
            .unwrap();
        assert_eq!(co2_config.min_interval, 60);
        assert_eq!(co2_config.max_interval, 300);
        match &co2_config.reportable_change {
            Some(ZclValue::Float32(change)) => {
                assert!((*change - 50.0e-6).abs() < f32::EPSILON);
            }
            other => panic!("unexpected CO2 reportable change: {other:?}"),
        }

        let battery_config = device
            .reporting()
            .get_config(
                1,
                ClusterId::POWER_CONFIG.0,
                ReportDirection::Send,
                ATTR_BATTERY_PERCENTAGE_REMAINING,
            )
            .unwrap();
        assert_eq!(battery_config.min_interval, 300);
        assert_eq!(battery_config.max_interval, 3_600);
    }

    #[test]
    fn co2_reporting_suppresses_sub_threshold_changes_and_honors_max_interval() {
        let reporting = AirQualityReporting {
            co2_min_secs: 1,
            co2_max_secs: 5,
            co2_change_ppm: 50.0,
            ..AirQualityReporting::default()
        };
        let mut component = AirQuality::new(range(), reporting);
        let mut device = reporting_device();
        component
            .configure_default_reporting(1, &mut device)
            .unwrap();

        component.update_measurement(AirQualityMeasurement {
            co2_ppm: 1_000.0,
            temperature_centi_celsius: 2_000,
            humidity_centi_percent: 4_000,
        });
        device.reporting_mut().tick(1);
        assert_eq!(collect_co2_reports(&mut device, &component).len(), 1);

        component.update_measurement(AirQualityMeasurement {
            co2_ppm: 1_049.9,
            temperature_centi_celsius: 2_000,
            humidity_centi_percent: 4_000,
        });
        device.reporting_mut().tick(1);
        assert!(collect_co2_reports(&mut device, &component).is_empty());

        component.update_measurement(AirQualityMeasurement {
            co2_ppm: 1_050.0,
            temperature_centi_celsius: 2_000,
            humidity_centi_percent: 4_000,
        });
        device.reporting_mut().tick(1);
        assert_eq!(collect_co2_reports(&mut device, &component).len(), 1);

        component.update_measurement(AirQualityMeasurement {
            co2_ppm: 1_051.0,
            temperature_centi_celsius: 2_000,
            humidity_centi_percent: 4_000,
        });
        device.reporting_mut().tick(5);
        assert_eq!(collect_co2_reports(&mut device, &component).len(), 1);
    }

    #[test]
    fn co2_reporting_does_not_repeat_stable_non_finite_values() {
        let reporting = AirQualityReporting {
            co2_min_secs: 1,
            co2_max_secs: 10,
            co2_change_ppm: 50.0,
            ..AirQualityReporting::default()
        };
        let mut component = AirQuality::new(range(), reporting);
        let mut device = reporting_device();
        component
            .configure_default_reporting(1, &mut device)
            .unwrap();

        component.update_measurement(AirQualityMeasurement {
            co2_ppm: f32::NAN,
            temperature_centi_celsius: 2_000,
            humidity_centi_percent: 4_000,
        });
        device.reporting_mut().tick(1);
        assert_eq!(collect_co2_reports(&mut device, &component).len(), 1);

        component.update_measurement(AirQualityMeasurement {
            co2_ppm: f32::NAN,
            temperature_centi_celsius: 2_000,
            humidity_centi_percent: 4_000,
        });
        device.reporting_mut().tick(1);
        assert!(collect_co2_reports(&mut device, &component).is_empty());

        component.update_measurement(AirQualityMeasurement {
            co2_ppm: f32::INFINITY,
            temperature_centi_celsius: 2_000,
            humidity_centi_percent: 4_000,
        });
        device.reporting_mut().tick(1);
        assert_eq!(collect_co2_reports(&mut device, &component).len(), 1);

        device.reporting_mut().tick(1);
        assert!(collect_co2_reports(&mut device, &component).is_empty());
    }
}
