//! Plant/soil sensor archetype: soil moisture + temperature + illuminance,
//! optional battery.
//!
//! Composes clusters with genuinely implemented attributes in this
//! workspace: Soil Moisture Measurement (0x0408), Temperature Measurement
//! (0x0402), and Illuminance Measurement (0x0400). There is no Home
//! Automation device ID dedicated to "plant sensor" (real products such as
//! the Xiaomi/Aqara Mi Flora use a manufacturer-specific profile rather than
//! HA 0x0104), so this archetype uses the same generic Simple Sensor device
//! ID (`DeviceId::SIMPLE_SENSOR`, 0x000C) as the optional `AirQuality`
//! profile.

use super::{
    ApplicationClusters, BatteryDescriptor, BatteryMeasurement, ExpectedReportClusters,
    ProfileComponent, ProfileError, TemperatureRange,
};
use crate::builder::EndpointBuilder;
use crate::{ClusterRef, ZigbeeDevice};
use zigbee_mac::MacDriver;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::illuminance::{
    ATTR_MEASURED_VALUE as ILLUMINANCE_MEASURED_VALUE, IlluminanceCluster,
};
use zigbee_zcl::clusters::power_config::{ATTR_BATTERY_PERCENTAGE_REMAINING, PowerConfigCluster};
use zigbee_zcl::clusters::soil_moisture::{
    ATTR_MEASURED_VALUE as SOIL_MOISTURE_MEASURED_VALUE, SoilMoistureCluster,
};
use zigbee_zcl::clusters::temperature::{
    ATTR_MEASURED_VALUE as TEMPERATURE_MEASURED_VALUE, TemperatureCluster,
};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

/// One soil moisture + temperature reading (illuminance updates separately,
/// see [`PlantSensor::update_illuminance`], since ambient light is often
/// sampled on its own cadence).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlantSensorMeasurement {
    pub soil_moisture_hundredths_pct: u16,
    pub temperature_centi_celsius: i16,
}

/// Reporting cadence and thresholds for the plant sensor archetype.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlantSensorReporting {
    pub soil_moisture_min_secs: u16,
    pub soil_moisture_max_secs: u16,
    pub soil_moisture_change_hundredths_pct: u16,
    pub temperature_min_secs: u16,
    pub temperature_max_secs: u16,
    pub temperature_change_centi_celsius: i16,
    pub illuminance_min_secs: u16,
    pub illuminance_max_secs: u16,
    pub illuminance_change: u16,
    pub battery_min_secs: u16,
    pub battery_max_secs: u16,
    pub battery_change_half_percent: u8,
}

impl Default for PlantSensorReporting {
    fn default() -> Self {
        Self {
            // Soil moisture changes slowly; poll infrequently to save battery.
            soil_moisture_min_secs: 300,
            soil_moisture_max_secs: 1_800,
            soil_moisture_change_hundredths_pct: 200, // 2.00%
            temperature_min_secs: 60,
            temperature_max_secs: 300,
            temperature_change_centi_celsius: 50,
            illuminance_min_secs: 300,
            illuminance_max_secs: 1_800,
            illuminance_change: 10,
            battery_min_secs: 300,
            battery_max_secs: 3_600,
            battery_change_half_percent: 4,
        }
    }
}

/// Soil Moisture + Temperature + Illuminance clusters, optional battery.
pub struct PlantSensor {
    soil_moisture: SoilMoistureCluster,
    temperature: TemperatureCluster,
    illuminance: IlluminanceCluster,
    battery: Option<PowerConfigCluster>,
    reporting: PlantSensorReporting,
}

impl PlantSensor {
    pub fn new(
        temperature_range: TemperatureRange,
        illuminance_max: u16,
        reporting: PlantSensorReporting,
    ) -> Self {
        Self {
            soil_moisture: SoilMoistureCluster::new(),
            temperature: TemperatureCluster::new(
                temperature_range.min_centi_celsius,
                temperature_range.max_centi_celsius,
            ),
            illuminance: IlluminanceCluster::new(0, illuminance_max),
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

    /// Update soil moisture and temperature from one measurement cycle.
    pub fn update_measurement(&mut self, measurement: PlantSensorMeasurement) {
        self.soil_moisture
            .set_moisture(measurement.soil_moisture_hundredths_pct);
        self.temperature
            .set_temperature(measurement.temperature_centi_celsius);
    }

    /// Update the illuminance reading. `value` must already be encoded per
    /// ZCL §4.2.2.2.1: `MeasuredValue = 10000 x log10(lux) + 1`. This runtime
    /// does not depend on `libm`, so the log10 conversion is left to the
    /// platform/product side where a math library is available.
    pub fn update_illuminance(&mut self, value: u16) {
        self.illuminance.set_illuminance(value);
    }

    /// Update the battery reading. No-op if no battery was configured.
    pub fn update_battery(&mut self, measurement: BatteryMeasurement) {
        if let Some(power) = &mut self.battery {
            power.set_battery_voltage(measurement.voltage_100mv);
            power.set_battery_percentage(measurement.percentage_remaining);
        }
    }

    /// Mark the battery reading unknown. No-op if no battery was configured.
    pub fn set_battery_unknown(&mut self) {
        if let Some(power) = &mut self.battery {
            power.set_battery_voltage(0xFF);
            power.set_battery_percentage(0xFF);
        }
    }

    pub const fn soil_moisture(&self) -> &SoilMoistureCluster {
        &self.soil_moisture
    }

    pub const fn temperature(&self) -> &TemperatureCluster {
        &self.temperature
    }

    pub const fn illuminance(&self) -> &IlluminanceCluster {
        &self.illuminance
    }
}

impl ProfileComponent for PlantSensor {
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
            .cluster_server(ClusterId::SOIL_MOISTURE)
            .cluster_server(ClusterId::TEMPERATURE)
            .cluster_server(ClusterId::ILLUMINANCE)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.soil_moisture,
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
                cluster: &mut self.illuminance,
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
        let _ = out.push(ClusterId::SOIL_MOISTURE.0);
        let _ = out.push(ClusterId::TEMPERATURE.0);
        let _ = out.push(ClusterId::ILLUMINANCE.0);
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
                ClusterId::SOIL_MOISTURE.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: SOIL_MOISTURE_MEASURED_VALUE,
                    data_type: ZclDataType::U16,
                    min_interval: self.reporting.soil_moisture_min_secs,
                    max_interval: self.reporting.soil_moisture_max_secs,
                    reportable_change: Some(ZclValue::U16(
                        self.reporting.soil_moisture_change_hundredths_pct,
                    )),
                },
            )
            .map_err(ProfileError::Reporting)?;
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
                ClusterId::ILLUMINANCE.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ILLUMINANCE_MEASURED_VALUE,
                    data_type: ZclDataType::U16,
                    min_interval: self.reporting.illuminance_min_secs,
                    max_interval: self.reporting.illuminance_max_secs,
                    reportable_change: Some(ZclValue::U16(self.reporting.illuminance_change)),
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
    use zigbee_zcl::foundation::reporting::ReportDirection;

    fn range() -> TemperatureRange {
        TemperatureRange {
            min_centi_celsius: 0,
            max_centi_celsius: 5_000,
        }
    }

    fn battery() -> BatteryDescriptor {
        BatteryDescriptor {
            size: 3, // AA
            quantity: 1,
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

    #[test]
    fn endpoint_without_battery() {
        let component = PlantSensor::new(range(), 0xFFFE, PlantSensorReporting::default());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::SOIL_MOISTURE,
                ClusterId::TEMPERATURE,
                ClusterId::ILLUMINANCE,
            ]
        );
    }

    #[test]
    fn endpoint_with_battery() {
        let component = PlantSensor::new(range(), 0xFFFE, PlantSensorReporting::default())
            .with_battery(battery());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::POWER_CONFIG,
                ClusterId::SOIL_MOISTURE,
                ClusterId::TEMPERATURE,
                ClusterId::ILLUMINANCE,
            ]
        );
    }

    #[test]
    fn collect_clusters_and_expected_report_count_track_battery() {
        let mut without_battery =
            PlantSensor::new(range(), 0xFFFE, PlantSensorReporting::default());
        assert_eq!(without_battery.expected_report_clusters(), 3);
        let mut clusters = ApplicationClusters::new();
        without_battery.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 3);

        let mut with_battery = PlantSensor::new(range(), 0xFFFE, PlantSensorReporting::default())
            .with_battery(battery());
        assert_eq!(with_battery.expected_report_clusters(), 4);
        let mut clusters = ApplicationClusters::new();
        with_battery.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 4);
    }

    #[test]
    fn measured_value_limits_come_from_constructor_bounds() {
        let component = PlantSensor::new(range(), 20_000, PlantSensorReporting::default());
        assert_eq!(
            component
                .temperature()
                .attributes()
                .get(zigbee_zcl::clusters::temperature::ATTR_MIN_MEASURED_VALUE),
            Some(&ZclValue::I16(0))
        );
        assert_eq!(
            component
                .temperature()
                .attributes()
                .get(zigbee_zcl::clusters::temperature::ATTR_MAX_MEASURED_VALUE),
            Some(&ZclValue::I16(5_000))
        );
        assert_eq!(
            component
                .illuminance()
                .attributes()
                .get(zigbee_zcl::clusters::illuminance::ATTR_MAX_MEASURED_VALUE),
            Some(&ZclValue::U16(20_000))
        );
        assert_eq!(
            component
                .soil_moisture()
                .attributes()
                .get(zigbee_zcl::clusters::soil_moisture::ATTR_MAX_MEASURED_VALUE),
            Some(&ZclValue::U16(10_000))
        );
    }

    #[test]
    fn update_measurement_illuminance_and_battery_write_owned_clusters() {
        let mut component = PlantSensor::new(range(), 0xFFFE, PlantSensorReporting::default())
            .with_battery(battery());
        component.update_measurement(PlantSensorMeasurement {
            soil_moisture_hundredths_pct: 3_250,
            temperature_centi_celsius: 2_200,
        });
        component.update_illuminance(8_500);
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 15,
            percentage_remaining: 100,
        });

        assert_eq!(
            component
                .soil_moisture()
                .attributes()
                .get(SOIL_MOISTURE_MEASURED_VALUE),
            Some(&ZclValue::U16(3_250))
        );
        assert_eq!(
            component
                .temperature()
                .attributes()
                .get(TEMPERATURE_MEASURED_VALUE),
            Some(&ZclValue::I16(2_200))
        );
        assert_eq!(
            component
                .illuminance()
                .attributes()
                .get(ILLUMINANCE_MEASURED_VALUE),
            Some(&ZclValue::U16(8_500))
        );
    }

    #[test]
    fn update_battery_is_a_no_op_without_a_configured_battery() {
        let mut component = PlantSensor::new(range(), 0xFFFE, PlantSensorReporting::default());
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 15,
            percentage_remaining: 100,
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
                    .cluster_server(ClusterId::SOIL_MOISTURE)
                    .cluster_server(ClusterId::TEMPERATURE)
                    .cluster_server(ClusterId::ILLUMINANCE)
            })
            .build();
        let component = PlantSensor::new(range(), 0xFFFE, PlantSensorReporting::default())
            .with_battery(battery());
        component
            .configure_default_reporting(1, &mut device)
            .unwrap();

        assert_eq!(device.configured_cluster_count(1), 4);
        // Local defaults are not a coordinator interview: the remote record
        // stays empty until a remote client actually configures reporting.
        assert_eq!(device.remote_reporting_cluster_count(1), 0);
        assert!(!device.is_cluster_remotely_configured(1, ClusterId::TEMPERATURE.0));

        let soil_config = device
            .reporting()
            .get_config(
                1,
                ClusterId::SOIL_MOISTURE.0,
                ReportDirection::Send,
                SOIL_MOISTURE_MEASURED_VALUE,
            )
            .unwrap();
        assert_eq!(soil_config.min_interval, 300);
        assert_eq!(soil_config.max_interval, 1_800);
        assert_eq!(soil_config.reportable_change, Some(ZclValue::U16(200)));
    }
}
