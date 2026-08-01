//! Thermostat archetype: local temperature + full thermostat controls.
//!
//! The Thermostat cluster (0x0201) already owns and reports its own
//! `LocalTemperature` attribute, so this archetype does not add a second
//! Temperature Measurement server that would just duplicate the same value.
//! [`ThermostatCluster`] already implements a genuinely complete control
//! surface for this archetype: setpoint raise/lower, weekly schedule
//! set/get/clear, and a `tick()` state machine that derives
//! `ThermostatRunningMode` from `SystemMode` and the current setpoints — so
//! "thermostat controls" here means real commanded behavior, not just
//! attribute storage.
//!
//! Optional add-ons, each independently toggled:
//! - Relative Humidity Measurement, for thermostats with a humidity sensor.
//! - Power Configuration, for battery-backed thermostats (e.g. radiator
//!   valves).
//!
//! Deliberately **not** included: Thermostat User Interface Configuration
//! (0x0204, `zigbee_zcl::clusters::thermostat_ui`) and Fan Control (0x0202,
//! `zigbee_zcl::clusters::fan_control`). Both exist in `zigbee-zcl`, but they
//! are separate concerns (client display/lockout configuration, and fan
//! actuator speed) outside "local temperature + thermostat controls". Adding
//! them here before a product actually implements a UI or a fan output would
//! be decorative.

use super::{
    ApplicationClusters, BatteryDescriptor, BatteryMeasurement, ProfileComponent, ProfileError,
};
use crate::builder::EndpointBuilder;
use crate::{ClusterRef, ZigbeeDevice};
use zigbee_mac::MacDriver;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::humidity::{
    ATTR_MEASURED_VALUE as HUMIDITY_MEASURED_VALUE, HumidityCluster,
};
use zigbee_zcl::clusters::power_config::{ATTR_BATTERY_PERCENTAGE_REMAINING, PowerConfigCluster};
use zigbee_zcl::clusters::thermostat::{ATTR_LOCAL_TEMPERATURE, ThermostatCluster};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

/// Reporting cadence and thresholds for the thermostat archetype.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThermostatReporting {
    pub local_temperature_min_secs: u16,
    pub local_temperature_max_secs: u16,
    pub local_temperature_change_centi_celsius: i16,
    pub humidity_min_secs: u16,
    pub humidity_max_secs: u16,
    pub humidity_change_centi_percent: u16,
    pub battery_min_secs: u16,
    pub battery_max_secs: u16,
    pub battery_change_half_percent: u8,
}

impl Default for ThermostatReporting {
    fn default() -> Self {
        Self {
            local_temperature_min_secs: 60,
            local_temperature_max_secs: 300,
            local_temperature_change_centi_celsius: 50,
            humidity_min_secs: 60,
            humidity_max_secs: 300,
            humidity_change_centi_percent: 100,
            battery_min_secs: 300,
            battery_max_secs: 3_600,
            battery_change_half_percent: 4,
        }
    }
}

/// Thermostat cluster with optional humidity and battery clusters.
pub struct Thermostat {
    thermostat: ThermostatCluster,
    humidity: Option<HumidityCluster>,
    battery: Option<PowerConfigCluster>,
    reporting: ThermostatReporting,
}

impl Thermostat {
    pub fn new(reporting: ThermostatReporting) -> Self {
        Self {
            thermostat: ThermostatCluster::new(),
            humidity: None,
            battery: None,
            reporting,
        }
    }

    /// Add a Relative Humidity Measurement cluster (0–100.00% RH range).
    pub fn with_humidity(mut self) -> Self {
        self.humidity = Some(HumidityCluster::new(0, 10_000));
        self
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

    pub fn has_humidity(&self) -> bool {
        self.humidity.is_some()
    }

    pub fn has_battery(&self) -> bool {
        self.battery.is_some()
    }

    /// Update the local temperature reading (in 0.01°C units).
    pub fn update_local_temperature(&mut self, hundredths: i16) {
        self.thermostat.set_local_temperature(hundredths);
    }

    /// Update the humidity reading. No-op if humidity was not configured via
    /// [`with_humidity`](Self::with_humidity).
    pub fn update_humidity(&mut self, hundredths: u16) {
        if let Some(humidity) = &mut self.humidity {
            humidity.set_humidity(hundredths);
        }
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

    /// Advance the weekly schedule and derive `ThermostatRunningMode` from
    /// `SystemMode` and the current setpoints. See [`ThermostatCluster::tick`].
    pub fn tick(&mut self, day_of_week: u8, minutes_since_midnight: u16) {
        self.thermostat.tick(day_of_week, minutes_since_midnight);
    }

    pub const fn thermostat(&self) -> &ThermostatCluster {
        &self.thermostat
    }

    pub fn thermostat_mut(&mut self) -> &mut ThermostatCluster {
        &mut self.thermostat
    }
}

impl ProfileComponent for Thermostat {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        let endpoint = endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY);
        let endpoint = if self.battery.is_some() {
            endpoint.cluster_server(ClusterId::POWER_CONFIG)
        } else {
            endpoint
        };
        let endpoint = endpoint.cluster_server(ClusterId::THERMOSTAT);
        if self.humidity.is_some() {
            endpoint.cluster_server(ClusterId::HUMIDITY)
        } else {
            endpoint
        }
    }

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.thermostat,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        if let Some(humidity) = &mut self.humidity {
            clusters
                .push(ClusterRef {
                    endpoint,
                    cluster: humidity,
                })
                .map_err(|_| ProfileError::TooManyClusters)?;
        }
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

    fn expected_report_clusters(&self) -> usize {
        1 + usize::from(self.humidity.is_some()) + usize::from(self.battery.is_some())
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
                ClusterId::THERMOSTAT.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_LOCAL_TEMPERATURE,
                    data_type: ZclDataType::I16,
                    min_interval: self.reporting.local_temperature_min_secs,
                    max_interval: self.reporting.local_temperature_max_secs,
                    reportable_change: Some(ZclValue::I16(
                        self.reporting.local_temperature_change_centi_celsius,
                    )),
                },
            )
            .map_err(ProfileError::Reporting)?;
        if self.humidity.is_some() {
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
        }
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
    use zigbee_zcl::clusters::thermostat::{
        ATTR_OCCUPIED_COOLING_SETPOINT, ATTR_OCCUPIED_HEATING_SETPOINT, CMD_SETPOINT_RAISE_LOWER,
    };
    use zigbee_zcl::foundation::reporting::ReportDirection;
    use zigbee_zcl::{CommandId, ZclStatus};

    fn battery() -> BatteryDescriptor {
        BatteryDescriptor {
            size: 10, // AAA
            quantity: 2,
            rated_voltage_100mv: 15,
        }
    }

    fn endpoint_builder() -> EndpointBuilder {
        EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::THERMOSTAT,
            device_version: 1,
            server_clusters: heapless::Vec::new(),
            client_clusters: heapless::Vec::new(),
        }
    }

    #[test]
    fn minimal_endpoint_has_only_thermostat() {
        let component = Thermostat::new(ThermostatReporting::default());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[ClusterId::BASIC, ClusterId::IDENTIFY, ClusterId::THERMOSTAT]
        );
    }

    #[test]
    fn full_endpoint_adds_battery_then_humidity() {
        let component = Thermostat::new(ThermostatReporting::default())
            .with_battery(battery())
            .with_humidity();
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::POWER_CONFIG,
                ClusterId::THERMOSTAT,
                ClusterId::HUMIDITY,
            ]
        );
    }

    #[test]
    fn collect_clusters_and_expected_report_count_track_options() {
        let mut minimal = Thermostat::new(ThermostatReporting::default());
        assert_eq!(minimal.expected_report_clusters(), 1);
        let mut clusters = ApplicationClusters::new();
        minimal.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 1);

        let mut full = Thermostat::new(ThermostatReporting::default())
            .with_humidity()
            .with_battery(battery());
        assert_eq!(full.expected_report_clusters(), 3);
        let mut clusters = ApplicationClusters::new();
        full.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 3);
    }

    #[test]
    fn update_local_temperature_and_optional_sensors() {
        let mut component = Thermostat::new(ThermostatReporting::default())
            .with_humidity()
            .with_battery(battery());
        component.update_local_temperature(2_150);
        component.update_humidity(4_800);
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 29,
            percentage_remaining: 190,
        });

        assert_eq!(
            component
                .thermostat()
                .attributes()
                .get(ATTR_LOCAL_TEMPERATURE),
            Some(&ZclValue::I16(2_150))
        );
    }

    #[test]
    fn optional_sensor_updates_are_no_ops_when_not_configured() {
        let mut component = Thermostat::new(ThermostatReporting::default());
        // Must not panic: no humidity or battery cluster was configured.
        component.update_humidity(5_000);
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 30,
            percentage_remaining: 200,
        });
        component.set_battery_unknown();
        assert!(!component.has_humidity());
        assert!(!component.has_battery());
    }

    #[test]
    fn setpoint_raise_lower_command_is_genuinely_handled() {
        // Demonstrates real "thermostat controls", not just attribute storage:
        // the underlying ThermostatCluster parses and applies the command.
        let mut component = Thermostat::new(ThermostatReporting::default());
        let heat_before = component
            .thermostat()
            .attributes()
            .get(ATTR_OCCUPIED_HEATING_SETPOINT)
            .cloned();
        assert_eq!(heat_before, Some(ZclValue::I16(2_000)));

        // mode=0 (heat), amount=+5 (raise by 0.5 degC = 50 in 0.01degC units)
        let result = component
            .thermostat_mut()
            .handle_command(CMD_SETPOINT_RAISE_LOWER, &[0x00, 0x05]);
        assert!(result.is_ok());
        assert_eq!(
            component
                .thermostat()
                .attributes()
                .get(ATTR_OCCUPIED_HEATING_SETPOINT),
            Some(&ZclValue::I16(2_050))
        );
        // Cooling setpoint must be untouched (mode == heat only).
        assert_eq!(
            component
                .thermostat()
                .attributes()
                .get(ATTR_OCCUPIED_COOLING_SETPOINT),
            Some(&ZclValue::I16(2_600))
        );

        let unsupported = component
            .thermostat_mut()
            .handle_command(CommandId(0xFE), &[]);
        assert_eq!(unsupported, Err(ZclStatus::UnsupClusterCommand));
    }

    #[test]
    fn default_reporting_configures_local_temperature_and_options() {
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(1, PROFILE_HOME_AUTOMATION, DeviceId::THERMOSTAT, |ep| {
                ep.cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::IDENTIFY)
                    .cluster_server(ClusterId::POWER_CONFIG)
                    .cluster_server(ClusterId::THERMOSTAT)
                    .cluster_server(ClusterId::HUMIDITY)
            })
            .build();
        let component = Thermostat::new(ThermostatReporting::default())
            .with_humidity()
            .with_battery(battery());
        component
            .configure_default_reporting(1, &mut device)
            .unwrap();

        assert_eq!(device.configured_cluster_count(1), 3);

        let local_temp_config = device
            .reporting()
            .get_config(
                1,
                ClusterId::THERMOSTAT.0,
                ReportDirection::Send,
                ATTR_LOCAL_TEMPERATURE,
            )
            .unwrap();
        assert_eq!(local_temp_config.min_interval, 60);
        assert_eq!(local_temp_config.max_interval, 300);
        assert_eq!(local_temp_config.reportable_change, Some(ZclValue::I16(50)));
    }
}
