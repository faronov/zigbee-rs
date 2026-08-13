//! Occupancy + illuminance sensor archetype ("occupancy/light" sensing).
//!
//! Composes the Occupancy Sensing (0x0406) and Illuminance Measurement
//! (0x0400) clusters, matching real combined PIR + ambient-light sensors
//! (e.g. Hue motion sensor, IKEA TRÅDFRI motion sensor). This is a **sensor**
//! archetype, not a luminaire: it does not include On/Off, Level Control, or
//! Color Control. A light fixture that reacts to occupancy is a product/
//! application concern (binding an occupancy sensor endpoint to a light
//! endpoint), not a single combined device profile.
//!
//! IAS Zone (0x0500, `zigbee_zcl::clusters::ias_zone`) is an alternative way
//! some vendors report occupancy, as a security "motion" zone with its own
//! CIE enrollment/notification handshake. It is intentionally **not** used
//! here: mixing zone-alarm semantics into a plain occupancy sensor archetype
//! would misrepresent the device as security equipment when it is not.

use super::{
    ApplicationClusters, BatteryDescriptor, BatteryMeasurement, ExpectedReportClusters,
    ProfileComponent, ProfileError,
};
use crate::builder::EndpointBuilder;
use crate::{ClusterRef, ZigbeeDevice};
use zigbee_mac::MacDriver;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::illuminance::{
    ATTR_MEASURED_VALUE as ILLUMINANCE_MEASURED_VALUE, IlluminanceCluster,
};
use zigbee_zcl::clusters::occupancy::{ATTR_OCCUPANCY, OccupancyCluster};
use zigbee_zcl::clusters::power_config::{ATTR_BATTERY_PERCENTAGE_REMAINING, PowerConfigCluster};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

/// A single occupancy + illuminance reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OccupancyLightMeasurement {
    pub occupied: bool,
    /// Pre-encoded per ZCL §4.2.2.2.1: `MeasuredValue = 10000 x log10(lux) + 1`
    /// (0 = too low to measure, 0xFFFF = invalid/unknown). This runtime does
    /// not depend on `libm`, so it does not compute the log10 conversion
    /// itself — compute it on the platform/product side where a math library
    /// is available, then pass the encoded value here.
    pub illuminance_measured_value: u16,
}

/// Reporting cadence and thresholds for the occupancy/light archetype.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OccupancyLightReporting {
    pub illuminance_min_secs: u16,
    pub illuminance_max_secs: u16,
    pub illuminance_change: u16,
    pub battery_min_secs: u16,
    pub battery_max_secs: u16,
    pub battery_change_half_percent: u8,
}

impl Default for OccupancyLightReporting {
    fn default() -> Self {
        Self {
            illuminance_min_secs: 60,
            illuminance_max_secs: 300,
            illuminance_change: 10,
            battery_min_secs: 300,
            battery_max_secs: 3_600,
            battery_change_half_percent: 4,
        }
    }
}

/// Occupancy Sensing + Illuminance Measurement clusters, optional battery.
pub struct OccupancyLight {
    occupancy: OccupancyCluster,
    illuminance: IlluminanceCluster,
    battery: Option<PowerConfigCluster>,
    reporting: OccupancyLightReporting,
}

impl OccupancyLight {
    /// `sensor_type` is one of the `SENSOR_TYPE_*` constants in
    /// `zigbee_zcl::clusters::occupancy` (PIR, ultrasonic, both, or physical
    /// contact). `illuminance_max` is the sensor's maximum encoded
    /// `MeasuredValue` (see [`OccupancyLightMeasurement::illuminance_measured_value`]).
    pub fn new(sensor_type: u8, illuminance_max: u16, reporting: OccupancyLightReporting) -> Self {
        Self {
            occupancy: OccupancyCluster::new(sensor_type),
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

    /// Update occupancy and illuminance from one measurement cycle.
    pub fn update_measurement(&mut self, measurement: OccupancyLightMeasurement) {
        self.occupancy.set_occupied(measurement.occupied);
        self.illuminance
            .set_illuminance(measurement.illuminance_measured_value);
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

    pub const fn occupancy(&self) -> &OccupancyCluster {
        &self.occupancy
    }

    pub const fn illuminance(&self) -> &IlluminanceCluster {
        &self.illuminance
    }
}

impl ProfileComponent for OccupancyLight {
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
            .cluster_server(ClusterId::OCCUPANCY)
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
                cluster: &mut self.occupancy,
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
        let _ = out.push(ClusterId::OCCUPANCY.0);
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
        // Bitmap8 is a discrete type: no reportable-change threshold, any
        // change is reported (see `zigbee_zcl::data_types::is_analog_type`).
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::OCCUPANCY.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_OCCUPANCY,
                    data_type: ZclDataType::Bitmap8,
                    min_interval: 0,
                    max_interval: 3_600,
                    reportable_change: None,
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
    use zigbee_zcl::clusters::occupancy::SENSOR_TYPE_PIR;
    use zigbee_zcl::foundation::reporting::ReportDirection;

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
            device_id: DeviceId::OCCUPANCY_SENSOR,
            device_version: 1,
            server_clusters: heapless::Vec::new(),
            client_clusters: heapless::Vec::new(),
        }
    }

    #[test]
    fn endpoint_without_battery() {
        let component =
            OccupancyLight::new(SENSOR_TYPE_PIR, 0xFFFE, OccupancyLightReporting::default());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::OCCUPANCY,
                ClusterId::ILLUMINANCE,
            ]
        );
    }

    #[test]
    fn endpoint_with_battery() {
        let component =
            OccupancyLight::new(SENSOR_TYPE_PIR, 0xFFFE, OccupancyLightReporting::default())
                .with_battery(battery());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::POWER_CONFIG,
                ClusterId::OCCUPANCY,
                ClusterId::ILLUMINANCE,
            ]
        );
    }

    #[test]
    fn collect_clusters_and_expected_report_count_track_battery() {
        let mut without_battery =
            OccupancyLight::new(SENSOR_TYPE_PIR, 0xFFFE, OccupancyLightReporting::default());
        assert_eq!(without_battery.expected_report_clusters(), 2);
        let mut clusters = ApplicationClusters::new();
        without_battery.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 2);

        let mut with_battery =
            OccupancyLight::new(SENSOR_TYPE_PIR, 0xFFFE, OccupancyLightReporting::default())
                .with_battery(battery());
        assert_eq!(with_battery.expected_report_clusters(), 3);
        let mut clusters = ApplicationClusters::new();
        with_battery.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 3);
    }

    #[test]
    fn illuminance_limits_come_from_constructor_bounds() {
        let component =
            OccupancyLight::new(SENSOR_TYPE_PIR, 15_000, OccupancyLightReporting::default());
        assert_eq!(
            component
                .illuminance()
                .attributes()
                .get(zigbee_zcl::clusters::illuminance::ATTR_MAX_MEASURED_VALUE),
            Some(&ZclValue::U16(15_000))
        );
        assert_eq!(
            component
                .illuminance()
                .attributes()
                .get(zigbee_zcl::clusters::illuminance::ATTR_MIN_MEASURED_VALUE),
            Some(&ZclValue::U16(0))
        );
    }

    #[test]
    fn update_measurement_and_battery_writes_owned_clusters() {
        let mut component =
            OccupancyLight::new(SENSOR_TYPE_PIR, 0xFFFE, OccupancyLightReporting::default())
                .with_battery(battery());
        component.update_measurement(OccupancyLightMeasurement {
            occupied: true,
            illuminance_measured_value: 12_345,
        });
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 29,
            percentage_remaining: 170,
        });

        assert_eq!(
            component.occupancy().attributes().get(ATTR_OCCUPANCY),
            Some(&ZclValue::Bitmap8(1))
        );
        assert_eq!(
            component
                .illuminance()
                .attributes()
                .get(ILLUMINANCE_MEASURED_VALUE),
            Some(&ZclValue::U16(12_345))
        );

        component.update_measurement(OccupancyLightMeasurement {
            occupied: false,
            illuminance_measured_value: 0,
        });
        assert_eq!(
            component.occupancy().attributes().get(ATTR_OCCUPANCY),
            Some(&ZclValue::Bitmap8(0))
        );
    }

    #[test]
    fn update_battery_is_a_no_op_without_a_configured_battery() {
        let mut component =
            OccupancyLight::new(SENSOR_TYPE_PIR, 0xFFFE, OccupancyLightReporting::default());
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 30,
            percentage_remaining: 200,
        });
        component.set_battery_unknown();
        assert!(!component.has_battery());
    }

    #[test]
    fn default_reporting_configures_occupancy_and_illuminance() {
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(
                1,
                PROFILE_HOME_AUTOMATION,
                DeviceId::OCCUPANCY_SENSOR,
                |ep| {
                    ep.cluster_server(ClusterId::BASIC)
                        .cluster_server(ClusterId::IDENTIFY)
                        .cluster_server(ClusterId::OCCUPANCY)
                        .cluster_server(ClusterId::ILLUMINANCE)
                },
            )
            .build();
        let component =
            OccupancyLight::new(SENSOR_TYPE_PIR, 0xFFFE, OccupancyLightReporting::default());
        component
            .configure_default_reporting(1, &mut device)
            .unwrap();

        assert_eq!(device.configured_cluster_count(1), 2);

        let occupancy_config = device
            .reporting()
            .get_config(
                1,
                ClusterId::OCCUPANCY.0,
                ReportDirection::Send,
                ATTR_OCCUPANCY,
            )
            .unwrap();
        assert_eq!(occupancy_config.reportable_change, None);

        let illuminance_config = device
            .reporting()
            .get_config(
                1,
                ClusterId::ILLUMINANCE.0,
                ReportDirection::Send,
                ILLUMINANCE_MEASURED_VALUE,
            )
            .unwrap();
        assert_eq!(illuminance_config.min_interval, 60);
        assert_eq!(illuminance_config.max_interval, 300);
    }
}
