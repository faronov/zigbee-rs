//! Smart plug archetype: On/Off + electrical measurement, optional metering.
//!
//! Only clusters with genuinely complete, load-bearing APIs in this
//! workspace are composed here:
//!
//! - On/Off (0x0006): the full HA command set — On/Off/Toggle,
//!   OffWithEffect, OnWithRecallGlobalScene, OnWithTimedOff — plus
//!   OnTime/OffWaitTime countdown and StartUpOnOff power-on-restore
//!   behavior. This is real commanded control, not attribute storage.
//! - Electrical Measurement (0x0B04): basic single-phase AC RMS voltage/
//!   current/active power reporting. Multi-phase, harmonics, and
//!   power-quality attributes are not implemented in `zigbee-zcl` and are
//!   not claimed here.
//! - Simple Metering (0x0702), optional: cumulative energy delivered and
//!   instantaneous demand. This is report-only telemetry — Simple
//!   Metering's optional command set (GetProfile, RequestFastPollMode,
//!   GetSnapshot, ...) is not implemented in `zigbee-zcl`, so this archetype
//!   does not claim full AMI metering behavior. `CurrentSummationReceived`
//!   (energy exported back to the grid) is also not reported: a plug-load
//!   device only consumes, so a permanently-zero "received" report would be
//!   decorative.
//!
//! Groups (0x0004) and Scenes (0x0005) are commonly bundled with On/Off on
//! real outlets, but they are a general binding/scene concern rather than
//! smart-plug-specific measurement behavior. They are left to the product's
//! own endpoint composition — see [`crate::templates::smart_plug`] for a
//! preset that includes them alongside the lower-level `DeviceBuilder` API.

use super::{ApplicationClusters, ProfileComponent, ProfileError};
use crate::builder::EndpointBuilder;
use crate::{ClusterRef, ZigbeeDevice};
use zigbee_mac::MacDriver;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::electrical::{
    ATTR_ACTIVE_POWER, ATTR_RMS_CURRENT, ATTR_RMS_VOLTAGE, ElectricalMeasurementCluster,
};
use zigbee_zcl::clusters::metering::{
    ATTR_CURRENT_SUMMATION_DELIVERED, ATTR_INSTANTANEOUS_DEMAND, MeteringCluster,
};
use zigbee_zcl::clusters::on_off::{ATTR_ON_OFF, OnOffCluster};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

/// One RMS voltage/current/active-power reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ElectricalReading {
    pub rms_voltage: u16,
    pub rms_current: u16,
    pub active_power_watts: i16,
}

/// Reporting cadence and thresholds for the smart plug archetype.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SmartPlugReporting {
    pub on_off_max_secs: u16,
    pub voltage_min_secs: u16,
    pub voltage_max_secs: u16,
    pub voltage_change: u16,
    pub current_min_secs: u16,
    pub current_max_secs: u16,
    pub current_change: u16,
    pub power_min_secs: u16,
    pub power_max_secs: u16,
    pub power_change_watts: i16,
    pub energy_min_secs: u16,
    pub energy_max_secs: u16,
    pub demand_change_watts: i32,
}

impl Default for SmartPlugReporting {
    fn default() -> Self {
        Self {
            on_off_max_secs: 3_600,
            voltage_min_secs: 30,
            voltage_max_secs: 300,
            voltage_change: 2,
            current_min_secs: 30,
            current_max_secs: 300,
            current_change: 100,
            power_min_secs: 10,
            power_max_secs: 60,
            power_change_watts: 5,
            energy_min_secs: 60,
            energy_max_secs: 900,
            demand_change_watts: 10,
        }
    }
}

/// On/Off + Electrical Measurement clusters, optional Simple Metering.
pub struct SmartPlug {
    on_off: OnOffCluster,
    electrical: ElectricalMeasurementCluster,
    metering: Option<MeteringCluster>,
    reporting: SmartPlugReporting,
}

impl SmartPlug {
    pub fn new(reporting: SmartPlugReporting) -> Self {
        Self {
            on_off: OnOffCluster::new(),
            electrical: ElectricalMeasurementCluster::new(),
            metering: None,
            reporting,
        }
    }

    /// Add a Simple Metering cluster. `unit`/`multiplier`/`divisor` are the
    /// `UNIT_*`/`Multiplier`/`Divisor` values documented in
    /// `zigbee_zcl::clusters::metering`.
    pub fn with_metering(mut self, unit: u8, multiplier: u32, divisor: u32) -> Self {
        self.metering = Some(MeteringCluster::new(unit, multiplier, divisor));
        self
    }

    pub fn has_metering(&self) -> bool {
        self.metering.is_some()
    }

    pub fn is_on(&self) -> bool {
        self.on_off.is_on()
    }

    /// Advance the On/Off OnTime/OffWaitTime timers. Call at the ZCL-mandated
    /// 1/10-second (100 ms) cadence. See [`OnOffCluster::tick`].
    pub fn tick_on_off(&mut self) {
        self.on_off.tick();
    }

    /// Apply `StartUpOnOff` on device power-on. See
    /// [`OnOffCluster::apply_startup`].
    pub fn apply_startup_on_off(&mut self, previous_on: bool) {
        self.on_off.apply_startup(previous_on);
    }

    /// Update RMS voltage/current/active power from one measurement cycle.
    pub fn update_electrical(&mut self, reading: ElectricalReading) {
        self.electrical.set_measurements(
            reading.rms_voltage,
            reading.rms_current,
            reading.active_power_watts,
        );
    }

    /// Add delivered energy (Wh) to the cumulative summation counter. No-op
    /// if metering was not configured via
    /// [`with_metering`](Self::with_metering).
    pub fn add_energy_delivered_wh(&mut self, wh: u64) {
        if let Some(metering) = &mut self.metering {
            metering.add_energy_delivered(wh);
        }
    }

    /// Update the instantaneous demand (W). No-op if metering was not
    /// configured.
    pub fn set_instantaneous_demand_watts(&mut self, watts: i32) {
        if let Some(metering) = &mut self.metering {
            metering.set_instantaneous_demand(watts);
        }
    }

    /// Total delivered energy (Wh), if metering is configured.
    pub fn total_energy_delivered_wh(&self) -> Option<u64> {
        self.metering
            .as_ref()
            .map(MeteringCluster::get_total_delivered)
    }

    pub const fn on_off(&self) -> &OnOffCluster {
        &self.on_off
    }

    pub fn on_off_mut(&mut self) -> &mut OnOffCluster {
        &mut self.on_off
    }

    pub const fn electrical(&self) -> &ElectricalMeasurementCluster {
        &self.electrical
    }
}

impl ProfileComponent for SmartPlug {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        let endpoint = endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY)
            .cluster_server(ClusterId::ON_OFF)
            .cluster_server(ClusterId::ELECTRICAL_MEASUREMENT);
        if self.metering.is_some() {
            endpoint.cluster_server(ClusterId::METERING)
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
                cluster: &mut self.on_off,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.electrical,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        if let Some(metering) = &mut self.metering {
            clusters
                .push(ClusterRef {
                    endpoint,
                    cluster: metering,
                })
                .map_err(|_| ProfileError::TooManyClusters)?;
        }
        Ok(())
    }

    fn expected_report_clusters(&self) -> usize {
        2 + usize::from(self.metering.is_some())
    }

    fn configure_default_reporting<M: MacDriver>(
        &self,
        endpoint: u8,
        device: &mut ZigbeeDevice<M>,
    ) -> Result<(), ProfileError> {
        let reporting = device.reporting_mut();
        // Bool is a discrete type: no reportable-change threshold, any
        // change is reported immediately; max_interval is a heartbeat only.
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::ON_OFF.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_ON_OFF,
                    data_type: ZclDataType::Bool,
                    min_interval: 0,
                    max_interval: self.reporting.on_off_max_secs,
                    reportable_change: None,
                },
            )
            .map_err(ProfileError::Reporting)?;
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::ELECTRICAL_MEASUREMENT.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_RMS_VOLTAGE,
                    data_type: ZclDataType::U16,
                    min_interval: self.reporting.voltage_min_secs,
                    max_interval: self.reporting.voltage_max_secs,
                    reportable_change: Some(ZclValue::U16(self.reporting.voltage_change)),
                },
            )
            .map_err(ProfileError::Reporting)?;
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::ELECTRICAL_MEASUREMENT.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_RMS_CURRENT,
                    data_type: ZclDataType::U16,
                    min_interval: self.reporting.current_min_secs,
                    max_interval: self.reporting.current_max_secs,
                    reportable_change: Some(ZclValue::U16(self.reporting.current_change)),
                },
            )
            .map_err(ProfileError::Reporting)?;
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::ELECTRICAL_MEASUREMENT.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_ACTIVE_POWER,
                    data_type: ZclDataType::I16,
                    min_interval: self.reporting.power_min_secs,
                    max_interval: self.reporting.power_max_secs,
                    reportable_change: Some(ZclValue::I16(self.reporting.power_change_watts)),
                },
            )
            .map_err(ProfileError::Reporting)?;
        if self.metering.is_some() {
            reporting
                .configure_for_cluster(
                    endpoint,
                    ClusterId::METERING.0,
                    ReportingConfig {
                        direction: ReportDirection::Send,
                        attribute_id: ATTR_CURRENT_SUMMATION_DELIVERED,
                        data_type: ZclDataType::U48,
                        min_interval: self.reporting.energy_min_secs,
                        max_interval: self.reporting.energy_max_secs,
                        reportable_change: Some(ZclValue::U48(1)),
                    },
                )
                .map_err(ProfileError::Reporting)?;
            reporting
                .configure_for_cluster(
                    endpoint,
                    ClusterId::METERING.0,
                    ReportingConfig {
                        direction: ReportDirection::Send,
                        attribute_id: ATTR_INSTANTANEOUS_DEMAND,
                        data_type: ZclDataType::I32,
                        min_interval: self.reporting.energy_min_secs,
                        max_interval: self.reporting.energy_max_secs,
                        reportable_change: Some(ZclValue::I32(self.reporting.demand_change_watts)),
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
    use zigbee_zcl::clusters::metering::{DEVICE_TYPE_ELECTRIC, UNIT_KWH};
    use zigbee_zcl::clusters::on_off::CMD_ON;
    use zigbee_zcl::foundation::reporting::ReportDirection;

    fn endpoint_builder() -> EndpointBuilder {
        EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::MAINS_POWER_OUTLET,
            device_version: 1,
            server_clusters: heapless::Vec::new(),
            client_clusters: heapless::Vec::new(),
        }
    }

    #[test]
    fn endpoint_without_metering() {
        let component = SmartPlug::new(SmartPlugReporting::default());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::ON_OFF,
                ClusterId::ELECTRICAL_MEASUREMENT,
            ]
        );
    }

    #[test]
    fn endpoint_with_metering() {
        let component =
            SmartPlug::new(SmartPlugReporting::default()).with_metering(UNIT_KWH, 1, 1_000);
        assert!(component.has_metering());
        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::ON_OFF,
                ClusterId::ELECTRICAL_MEASUREMENT,
                ClusterId::METERING,
            ]
        );
        let _ = DEVICE_TYPE_ELECTRIC; // documents intended metering device type
    }

    #[test]
    fn collect_clusters_and_expected_report_count_track_metering() {
        let mut without_metering = SmartPlug::new(SmartPlugReporting::default());
        assert_eq!(without_metering.expected_report_clusters(), 2);
        let mut clusters = ApplicationClusters::new();
        without_metering.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 2);

        let mut with_metering =
            SmartPlug::new(SmartPlugReporting::default()).with_metering(UNIT_KWH, 1, 1_000);
        assert_eq!(with_metering.expected_report_clusters(), 3);
        let mut clusters = ApplicationClusters::new();
        with_metering.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 3);
    }

    #[test]
    fn on_off_command_is_genuinely_handled() {
        let mut component = SmartPlug::new(SmartPlugReporting::default());
        assert!(!component.is_on());
        let result = component.on_off_mut().handle_command(CMD_ON, &[]);
        assert!(result.is_ok());
        assert!(component.is_on());
    }

    #[test]
    fn update_electrical_and_metering_write_owned_clusters() {
        let mut component =
            SmartPlug::new(SmartPlugReporting::default()).with_metering(UNIT_KWH, 1, 1_000);
        component.update_electrical(ElectricalReading {
            rms_voltage: 230,
            rms_current: 500,
            active_power_watts: 115,
        });
        component.add_energy_delivered_wh(1_500);
        component.set_instantaneous_demand_watts(115);

        assert_eq!(
            component.electrical().attributes().get(ATTR_RMS_VOLTAGE),
            Some(&ZclValue::U16(230))
        );
        assert_eq!(
            component.electrical().attributes().get(ATTR_ACTIVE_POWER),
            Some(&ZclValue::I16(115))
        );
        assert_eq!(component.total_energy_delivered_wh(), Some(1_500));

        component.add_energy_delivered_wh(500);
        assert_eq!(component.total_energy_delivered_wh(), Some(2_000));
    }

    #[test]
    fn metering_updates_are_no_ops_without_configured_metering() {
        let mut component = SmartPlug::new(SmartPlugReporting::default());
        component.add_energy_delivered_wh(1_000);
        component.set_instantaneous_demand_watts(50);
        assert_eq!(component.total_energy_delivered_wh(), None);
        assert!(!component.has_metering());
    }

    #[cfg(feature = "ota")]
    #[test]
    fn with_ota_wraps_a_new_archetype_generically() {
        // Proves WithOta (defined once in `profile::mod`) composes with any
        // ProfileComponent, not just the original TemperatureHumidityBattery.
        use crate::firmware_writer::MockFirmwareWriter;
        use crate::ota::{OtaConfig, OtaManager};
        use crate::profile::{ApplicationProfile, DeviceProfile, WithOta};

        let base = DeviceProfile::new(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::MAINS_POWER_OUTLET,
            SmartPlug::new(SmartPlugReporting::default()),
        );
        let ota = OtaManager::new(
            MockFirmwareWriter::new(1024),
            OtaConfig {
                endpoint: 1,
                ..OtaConfig::default()
            },
        );
        let mut profile = WithOta::new(base, ota).unwrap();
        let endpoint = profile.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.client_clusters.as_slice(),
            &[ClusterId::OTA_UPGRADE]
        );

        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters).unwrap();
        assert_eq!(clusters.len(), 3); // on_off + electrical + ota
        assert_eq!(
            clusters.last().map(|c| c.cluster.cluster_id()),
            Some(ClusterId::OTA_UPGRADE)
        );
    }

    #[test]
    fn default_reporting_configures_on_off_electrical_and_metering() {
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(
                1,
                PROFILE_HOME_AUTOMATION,
                DeviceId::MAINS_POWER_OUTLET,
                |ep| {
                    ep.cluster_server(ClusterId::BASIC)
                        .cluster_server(ClusterId::IDENTIFY)
                        .cluster_server(ClusterId::ON_OFF)
                        .cluster_server(ClusterId::ELECTRICAL_MEASUREMENT)
                        .cluster_server(ClusterId::METERING)
                },
            )
            .build();
        let component =
            SmartPlug::new(SmartPlugReporting::default()).with_metering(UNIT_KWH, 1, 1_000);
        component
            .configure_default_reporting(1, &mut device)
            .unwrap();

        assert_eq!(device.configured_cluster_count(1), 3);

        let on_off_config = device
            .reporting()
            .get_config(1, ClusterId::ON_OFF.0, ReportDirection::Send, ATTR_ON_OFF)
            .unwrap();
        assert_eq!(on_off_config.reportable_change, None);
        assert_eq!(on_off_config.max_interval, 3_600);

        let power_config = device
            .reporting()
            .get_config(
                1,
                ClusterId::ELECTRICAL_MEASUREMENT.0,
                ReportDirection::Send,
                ATTR_ACTIVE_POWER,
            )
            .unwrap();
        assert_eq!(power_config.reportable_change, Some(ZclValue::I16(5)));

        let demand_config = device
            .reporting()
            .get_config(
                1,
                ClusterId::METERING.0,
                ReportDirection::Send,
                ATTR_INSTANTANEOUS_DEMAND,
            )
            .unwrap();
        assert_eq!(demand_config.reportable_change, Some(ZclValue::I32(10)));
    }
}
