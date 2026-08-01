//! Typed application profiles that own their ZCL cluster instances.
//!
//! Besides the shared [`ProfileComponent`] / [`ApplicationProfile`] traits,
//! [`DeviceProfile`], the optional `WithOta` / `OptionalOta` decorators, the original
//! [`TemperatureHumidityBattery`] sensor archetype, and its
//! [`TemperatureHumidityPressureBattery`] pressure-composed sibling (built
//! via [`TemperatureHumidityBattery::with_pressure`]) defined in this
//! module, the following ready-made
//! archetypes live in their own submodules. Each composes only cluster
//! implementations from `zigbee-zcl` that have genuinely complete, honest
//! attribute/command/reporting support — see each submodule's doc comment
//! for what it deliberately leaves out and why:
//!
//! - `AirQuality` — CO₂ + temperature + humidity, optional battery; requires
//!   the `float32` feature.
//! - [`thermostat::Thermostat`] — local temperature + full thermostat controls,
//!   optional humidity/battery.
//! - [`occupancy_light::OccupancyLight`] — occupancy + illuminance sensing,
//!   optional battery.
//! - [`plant_sensor::PlantSensor`] — soil moisture + temperature + illuminance,
//!   optional battery.
//! - [`range_extender::RangeExtender`] — Basic + Identify for a router-only
//!   Home Automation range extender.
//! - [`smart_plug::SmartPlug`] — On/Off + electrical measurement, optional
//!   metering.
//!
//! `WithOta` always adds the OTA Upgrade client cluster; `OptionalOta` is
//! for platforms where the firmware backend may fail to construct (a checked
//! partition/bootloader layout that does not match this device) and OTA must
//! be cleanly omitted rather than block commissioning.

#[cfg(feature = "float32")]
pub mod air_quality;
pub mod occupancy_light;
pub mod plant_sensor;
pub mod range_extender;
pub mod smart_plug;
pub mod thermostat;

#[cfg(feature = "float32")]
pub use air_quality::{AirQuality, AirQualityMeasurement, AirQualityReporting};
pub use occupancy_light::{OccupancyLight, OccupancyLightMeasurement, OccupancyLightReporting};
pub use plant_sensor::{PlantSensor, PlantSensorMeasurement, PlantSensorReporting};
pub use range_extender::RangeExtender;
pub use smart_plug::{ElectricalReading, SmartPlug, SmartPlugReporting};
pub use thermostat::{Thermostat, ThermostatReporting};

use crate::builder::EndpointBuilder;
use crate::{ClusterRef, MAX_CLUSTERS_PER_ENDPOINT, ZigbeeDevice};
use heapless::Vec;
use zigbee_mac::MacDriver;
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::humidity::{
    ATTR_MEASURED_VALUE as HUMIDITY_MEASURED_VALUE, HumidityCluster,
};
use zigbee_zcl::clusters::power_config::{ATTR_BATTERY_PERCENTAGE_REMAINING, PowerConfigCluster};
use zigbee_zcl::clusters::pressure::PressureCluster;
use zigbee_zcl::clusters::temperature::{
    ATTR_MEASURED_VALUE as TEMPERATURE_MEASURED_VALUE, TemperatureCluster,
};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
use zigbee_zcl::{ClusterId, DeviceId, ZclStatus};

pub const MAX_APPLICATION_CLUSTERS: usize = MAX_CLUSTERS_PER_ENDPOINT;

pub type ApplicationClusters<'a> = Vec<ClusterRef<'a>, MAX_APPLICATION_CLUSTERS>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProfileError {
    TooManyClusters,
    EndpointMismatch {
        profile_endpoint: u8,
        component_endpoint: u8,
    },
    Reporting(ZclStatus),
}

/// Application-owned clusters and their measurement/reporting policy.
pub trait ProfileComponent {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder;

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError>;

    fn expected_report_clusters(&self) -> usize {
        0
    }

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
        &self,
        _endpoint: u8,
        _device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        Ok(())
    }
}

/// Complete endpoint profile used by [`crate::node::ZigbeeNode`].
pub trait ApplicationProfile {
    fn endpoint(&self) -> u8;
    fn profile_id(&self) -> u16;
    fn device_id(&self) -> DeviceId;
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder;

    fn collect_clusters<'a>(
        &'a mut self,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError>;

    fn expected_report_clusters(&self) -> usize;

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
        &self,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError>;
}

/// Endpoint identity plus a typed cluster component.
pub struct DeviceProfile<C> {
    endpoint: u8,
    profile_id: u16,
    device_id: DeviceId,
    component: C,
}

impl<C> DeviceProfile<C> {
    pub const fn new(endpoint: u8, profile_id: u16, device_id: DeviceId, component: C) -> Self {
        Self {
            endpoint,
            profile_id,
            device_id,
            component,
        }
    }

    pub const fn component(&self) -> &C {
        &self.component
    }

    pub fn component_mut(&mut self) -> &mut C {
        &mut self.component
    }
}

impl<C: ProfileComponent> ApplicationProfile for DeviceProfile<C> {
    fn endpoint(&self) -> u8 {
        self.endpoint
    }

    fn profile_id(&self) -> u16 {
        self.profile_id
    }

    fn device_id(&self) -> DeviceId {
        self.device_id
    }

    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        self.component.configure_endpoint(endpoint)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        self.component.collect_clusters(self.endpoint, clusters)
    }

    fn expected_report_clusters(&self) -> usize {
        self.component.expected_report_clusters()
    }

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
        &self,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        self.component
            .configure_default_reporting(self.endpoint, device)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TemperatureRange {
    pub min_centi_celsius: i16,
    pub max_centi_celsius: i16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryDescriptor {
    pub size: u8,
    pub quantity: u8,
    pub rated_voltage_100mv: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TemperatureHumidityMeasurement {
    pub temperature_centi_celsius: i16,
    pub humidity_centi_percent: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryMeasurement {
    pub voltage_100mv: u8,
    pub percentage_remaining: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnvironmentalReporting {
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

impl Default for EnvironmentalReporting {
    fn default() -> Self {
        Self {
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

/// Measurement range for an optional Pressure Measurement cluster, in
/// tenths of a kPa (1 hPa = 10 units), matching [`PressureCluster::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PressureRange {
    pub min_tenth_kpa: i16,
    pub max_tenth_kpa: i16,
}

/// Temperature, humidity, and battery clusters with a shared update API.
///
/// Deliberately has no pressure field or branch of any kind: this type is
/// reused by every EFR32/ESP32/nRF product, and `dyn Cluster` trait objects
/// mean any `Option<PressureCluster>` branch here — even one that is always
/// `None` at runtime — pulls [`PressureCluster`]'s full `Cluster` vtable and
/// attribute storage into *every* firmware image built from this archetype,
/// whether or not it ever calls [`Self::with_pressure`]. Pressure support
/// instead lives in the separate [`TemperatureHumidityPressureBattery`]
/// wrapper returned by [`Self::with_pressure`], so only the one product that
/// actually composes it (the nRF52840 BME280 variant) pays for it.
pub struct TemperatureHumidityBattery {
    temperature: TemperatureCluster,
    humidity: HumidityCluster,
    power: PowerConfigCluster,
    reporting: EnvironmentalReporting,
}

impl TemperatureHumidityBattery {
    pub fn new(
        temperature_range: TemperatureRange,
        battery: BatteryDescriptor,
        reporting: EnvironmentalReporting,
    ) -> Self {
        let mut power = PowerConfigCluster::new();
        power.set_battery_voltage(0xFF);
        power.set_battery_percentage(0xFF);
        power.set_battery_size(battery.size);
        power.set_battery_quantity(battery.quantity);
        power.set_battery_rated_voltage(battery.rated_voltage_100mv);

        Self {
            temperature: TemperatureCluster::new(
                temperature_range.min_centi_celsius,
                temperature_range.max_centi_celsius,
            ),
            humidity: HumidityCluster::new(0, 10_000),
            power,
            reporting,
        }
    }

    /// Compose with a Pressure Measurement cluster (e.g. for a BME280 that
    /// also reports pressure alongside temperature and humidity).
    ///
    /// Returns a distinct [`TemperatureHumidityPressureBattery`] type rather
    /// than adding an optional field to `Self`: see the type-level doc
    /// comment above for why that matters for flash size on platforms that
    /// never call this.
    ///
    /// The pressure cluster's reporting is left to the coordinator's own
    /// ConfigureReporting during interview rather than a default reporting
    /// policy, matching the reference nRF52840 BME280 firmware this
    /// archetype was extracted from.
    pub fn with_pressure(self, range: PressureRange) -> TemperatureHumidityPressureBattery {
        TemperatureHumidityPressureBattery {
            inner: self,
            pressure: PressureCluster::new(range.min_tenth_kpa, range.max_tenth_kpa),
        }
    }

    pub fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.temperature
            .set_temperature(measurement.temperature_centi_celsius);
        self.humidity
            .set_humidity(measurement.humidity_centi_percent);
    }

    pub fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.power.set_battery_voltage(measurement.voltage_100mv);
        self.power
            .set_battery_percentage(measurement.percentage_remaining);
    }

    pub fn set_battery_unknown(&mut self) {
        self.power.set_battery_voltage(0xFF);
        self.power.set_battery_percentage(0xFF);
    }
}

impl ProfileComponent for TemperatureHumidityBattery {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY)
            .cluster_server(self.power.cluster_id())
            .cluster_server(self.temperature.cluster_id())
            .cluster_server(self.humidity.cluster_id())
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
                cluster: &mut self.power,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        Ok(())
    }

    fn expected_report_clusters(&self) -> usize {
        3
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
            .map_err(ProfileError::Reporting)
    }
}

/// [`TemperatureHumidityBattery`] plus a mandatory Pressure Measurement
/// cluster, built via [`TemperatureHumidityBattery::with_pressure`].
///
/// This is a separate, statically-composed type (the same decorator shape
/// as `WithOta` / `OptionalOta` below) rather than an `Option` field on
/// `TemperatureHumidityBattery`, so that products which never call
/// `with_pressure` — every current EFR32 and ESP32 product — never
/// monomorphize or link [`PressureCluster`]'s `Cluster` vtable and
/// attribute storage at all.
pub struct TemperatureHumidityPressureBattery {
    inner: TemperatureHumidityBattery,
    pressure: PressureCluster,
}

impl TemperatureHumidityPressureBattery {
    /// Update the pressure reading, in tenths of a kPa.
    pub fn update_pressure(&mut self, tenth_kpa: i16) {
        self.pressure.set_pressure(tenth_kpa);
    }

    pub fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.inner.update_environment(measurement);
    }

    pub fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.inner.update_battery(measurement);
    }

    pub fn set_battery_unknown(&mut self) {
        self.inner.set_battery_unknown();
    }

    pub const fn inner(&self) -> &TemperatureHumidityBattery {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut TemperatureHumidityBattery {
        &mut self.inner
    }
}

impl ProfileComponent for TemperatureHumidityPressureBattery {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        self.inner
            .configure_endpoint(endpoint)
            .cluster_server(self.pressure.cluster_id())
    }

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        self.inner.collect_clusters(endpoint, clusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.pressure,
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    fn expected_report_clusters(&self) -> usize {
        self.inner.expected_report_clusters() + 1
    }

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
        &self,
        endpoint: u8,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        // Pressure reporting is intentionally left to the coordinator's own
        // ConfigureReporting during interview; see `with_pressure`'s doc
        // comment. Only delegate to the wrapped temperature/humidity/battery
        // defaults.
        self.inner.configure_default_reporting(endpoint, device)
    }
}

#[cfg(feature = "ota")]
pub struct WithOta<P, F: crate::firmware_writer::FirmwareWriter> {
    inner: P,
    ota: crate::ota::OtaManager<F>,
}

#[cfg(feature = "ota")]
impl<P: ApplicationProfile, F: crate::firmware_writer::FirmwareWriter> WithOta<P, F> {
    pub fn new(inner: P, ota: crate::ota::OtaManager<F>) -> Result<Self, ProfileError> {
        if inner.endpoint() != ota.endpoint() {
            return Err(ProfileError::EndpointMismatch {
                profile_endpoint: inner.endpoint(),
                component_endpoint: ota.endpoint(),
            });
        }
        Ok(Self { inner, ota })
    }

    pub const fn inner(&self) -> &P {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut P {
        &mut self.inner
    }

    pub const fn ota(&self) -> &crate::ota::OtaManager<F> {
        &self.ota
    }

    pub fn ota_mut(&mut self) -> &mut crate::ota::OtaManager<F> {
        &mut self.ota
    }
}

#[cfg(feature = "ota")]
impl<P: ApplicationProfile, F: crate::firmware_writer::FirmwareWriter> ApplicationProfile
    for WithOta<P, F>
{
    fn endpoint(&self) -> u8 {
        self.inner.endpoint()
    }

    fn profile_id(&self) -> u16 {
        self.inner.profile_id()
    }

    fn device_id(&self) -> DeviceId {
        self.inner.device_id()
    }

    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        self.inner
            .configure_endpoint(endpoint)
            .cluster_client(ClusterId::OTA_UPGRADE)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        let Self { inner, ota } = self;
        let endpoint = inner.endpoint();
        inner.collect_clusters(clusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: ota.cluster_mut(),
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    fn expected_report_clusters(&self) -> usize {
        self.inner.expected_report_clusters()
    }

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
        &self,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        self.inner.configure_default_reporting(device)
    }
}

/// The OTA cluster instance behind [`OptionalOta`]: either a live
/// [`crate::ota::OtaManager`] or an inert [`zigbee_zcl::clusters::ota::OtaCluster`]
/// that reports the current firmware version but never accepts an upgrade.
///
/// The variants are intentionally not boxed: this crate is heap-free, and a
/// platform that cannot construct a firmware writer still needs to hold the
/// disabled state inline in the same statically sized profile.
#[cfg(feature = "ota")]
#[allow(clippy::large_enum_variant)]
pub enum OtaBackend<F: crate::firmware_writer::FirmwareWriter> {
    Enabled(crate::ota::OtaManager<F>),
    Disabled(zigbee_zcl::clusters::ota::OtaCluster),
}

#[cfg(feature = "ota")]
impl<F: crate::firmware_writer::FirmwareWriter> OtaBackend<F> {
    /// Whether the endpoint should advertise the OTA Upgrade client cluster.
    pub fn is_enabled(&self) -> bool {
        matches!(self, Self::Enabled(_))
    }

    /// The ZCL cluster instance, whichever backend is active.
    pub fn cluster_mut(&mut self) -> &mut zigbee_zcl::clusters::ota::OtaCluster {
        match self {
            Self::Enabled(manager) => manager.cluster_mut(),
            Self::Disabled(cluster) => cluster,
        }
    }

    /// The live manager, if the firmware backend was constructed successfully.
    pub fn manager(&self) -> Option<&crate::ota::OtaManager<F>> {
        match self {
            Self::Enabled(manager) => Some(manager),
            Self::Disabled(_) => None,
        }
    }

    /// The live manager, if the firmware backend was constructed successfully.
    pub fn manager_mut(&mut self) -> Option<&mut crate::ota::OtaManager<F>> {
        match self {
            Self::Enabled(manager) => Some(manager),
            Self::Disabled(_) => None,
        }
    }
}

/// Like [`WithOta`], but tolerates a firmware backend that could not be
/// constructed — for example a checked partition or bootloader layout that
/// does not match what the writer requires on this particular device.
///
/// The endpoint then omits the OTA Upgrade client cluster and the profile
/// behaves like a normal Zigbee device: commissioning, reporting and every
/// other application cluster keep working. Nothing panics or halts startup
/// just because OTA hardware is unavailable or unrecognised.
#[cfg(feature = "ota")]
pub struct OptionalOta<P, F: crate::firmware_writer::FirmwareWriter> {
    inner: P,
    backend: OtaBackend<F>,
}

#[cfg(feature = "ota")]
impl<P: ApplicationProfile, F: crate::firmware_writer::FirmwareWriter> OptionalOta<P, F> {
    /// Compose with a successfully constructed firmware writer.
    pub fn enabled(inner: P, ota: crate::ota::OtaManager<F>) -> Result<Self, ProfileError> {
        if inner.endpoint() != ota.endpoint() {
            return Err(ProfileError::EndpointMismatch {
                profile_endpoint: inner.endpoint(),
                component_endpoint: ota.endpoint(),
            });
        }
        Ok(Self {
            inner,
            backend: OtaBackend::Enabled(ota),
        })
    }

    /// Compose without a firmware writer. The OTA Upgrade client cluster is
    /// not advertised on the endpoint; a plain [`zigbee_zcl::clusters::ota::OtaCluster`]
    /// still reports the running version to anything that reads it directly.
    pub fn disabled(
        inner: P,
        manufacturer_code: u16,
        image_type: u16,
        current_version: u32,
    ) -> Self {
        let cluster = zigbee_zcl::clusters::ota::OtaCluster::new(
            manufacturer_code,
            image_type,
            current_version,
        );
        Self {
            inner,
            backend: OtaBackend::Disabled(cluster),
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.backend.is_enabled()
    }

    pub const fn inner(&self) -> &P {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut P {
        &mut self.inner
    }

    pub const fn backend(&self) -> &OtaBackend<F> {
        &self.backend
    }

    pub fn backend_mut(&mut self) -> &mut OtaBackend<F> {
        &mut self.backend
    }
}

#[cfg(feature = "ota")]
impl<P: ApplicationProfile, F: crate::firmware_writer::FirmwareWriter> ApplicationProfile
    for OptionalOta<P, F>
{
    fn endpoint(&self) -> u8 {
        self.inner.endpoint()
    }

    fn profile_id(&self) -> u16 {
        self.inner.profile_id()
    }

    fn device_id(&self) -> DeviceId {
        self.inner.device_id()
    }

    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        let endpoint = self.inner.configure_endpoint(endpoint);
        if self.is_enabled() {
            endpoint.cluster_client(ClusterId::OTA_UPGRADE)
        } else {
            endpoint
        }
    }

    fn collect_clusters<'a>(
        &'a mut self,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        let Self { inner, backend } = self;
        let endpoint = inner.endpoint();
        inner.collect_clusters(clusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: backend.cluster_mut(),
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    fn expected_report_clusters(&self) -> usize {
        self.inner.expected_report_clusters()
    }

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
        &self,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        self.inner.configure_default_reporting(device)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_aps::PROFILE_HOME_AUTOMATION;

    fn profile() -> DeviceProfile<TemperatureHumidityBattery> {
        DeviceProfile::new(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::TEMPERATURE_SENSOR,
            TemperatureHumidityBattery::new(
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
            ),
        )
    }

    #[test]
    fn endpoint_and_cluster_refs_come_from_the_same_profile() {
        let mut profile = profile();
        let endpoint = EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::TEMPERATURE_SENSOR,
            device_version: 1,
            server_clusters: Vec::new(),
            client_clusters: Vec::new(),
        };
        let endpoint = profile.configure_endpoint(endpoint);
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::POWER_CONFIG,
                ClusterId::TEMPERATURE,
                ClusterId::HUMIDITY,
            ]
        );

        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters).unwrap();
        let ids: heapless::Vec<ClusterId, MAX_APPLICATION_CLUSTERS> = clusters
            .iter()
            .map(|cluster| cluster.cluster.cluster_id())
            .collect();
        assert_eq!(
            ids.as_slice(),
            &[
                ClusterId::TEMPERATURE,
                ClusterId::HUMIDITY,
                ClusterId::POWER_CONFIG,
            ]
        );
    }

    #[test]
    fn measurements_update_owned_clusters() {
        let mut component = TemperatureHumidityBattery::new(
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
        component.update_environment(TemperatureHumidityMeasurement {
            temperature_centi_celsius: 2_345,
            humidity_centi_percent: 5_678,
        });
        component.update_battery(BatteryMeasurement {
            voltage_100mv: 29,
            percentage_remaining: 150,
        });

        assert_eq!(
            component
                .temperature
                .attributes()
                .get(TEMPERATURE_MEASURED_VALUE),
            Some(&ZclValue::I16(2_345))
        );
        assert_eq!(
            component.humidity.attributes().get(HUMIDITY_MEASURED_VALUE),
            Some(&ZclValue::U16(5_678))
        );
        assert_eq!(
            component
                .power
                .attributes()
                .get(ATTR_BATTERY_PERCENTAGE_REMAINING),
            Some(&ZclValue::U8(150))
        );
    }

    #[test]
    fn bare_archetype_has_no_pressure() {
        // Without `with_pressure`, behavior is unchanged from the base
        // archetype: no pressure cluster, 3 expected report clusters. This
        // is a compile-time property now (there is no `has_pressure()` to
        // assert `false` on): `TemperatureHumidityBattery` simply has no
        // pressure field or cluster reference at all.
        let bare = TemperatureHumidityBattery::new(
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
        assert_eq!(bare.expected_report_clusters(), 3);
    }

    #[test]
    fn with_pressure_composes_a_distinct_type_additively() {
        use zigbee_zcl::clusters::pressure::ATTR_MEASURED_VALUE as PRESSURE_MEASURED_VALUE;

        let mut with_pressure = TemperatureHumidityBattery::new(
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
        )
        .with_pressure(PressureRange {
            min_tenth_kpa: 3_000,
            max_tenth_kpa: 11_000,
        });
        assert_eq!(with_pressure.expected_report_clusters(), 4);

        let endpoint = EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::TEMPERATURE_SENSOR,
            device_version: 1,
            server_clusters: Vec::new(),
            client_clusters: Vec::new(),
        };
        let endpoint = with_pressure.configure_endpoint(endpoint);
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::POWER_CONFIG,
                ClusterId::TEMPERATURE,
                ClusterId::HUMIDITY,
                ClusterId::PRESSURE,
            ]
        );

        with_pressure.update_pressure(9_812);
        let mut clusters = ApplicationClusters::new();
        with_pressure.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(clusters.len(), 4);
        assert_eq!(
            clusters[3]
                .cluster
                .attributes()
                .get(PRESSURE_MEASURED_VALUE),
            Some(&ZclValue::I16(9_812))
        );
    }

    #[cfg(feature = "ota")]
    #[test]
    fn ota_decorator_adds_client_and_runtime_cluster() {
        use crate::firmware_writer::MockFirmwareWriter;
        use crate::ota::{OtaConfig, OtaManager};

        let ota = OtaManager::new(
            MockFirmwareWriter::new(1024),
            OtaConfig {
                endpoint: 1,
                ..OtaConfig::default()
            },
        );
        let mut profile = WithOta::new(profile(), ota).unwrap();
        let endpoint = EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::TEMPERATURE_SENSOR,
            device_version: 1,
            server_clusters: Vec::new(),
            client_clusters: Vec::new(),
        };
        let endpoint = profile.configure_endpoint(endpoint);
        assert_eq!(
            endpoint.client_clusters.as_slice(),
            &[ClusterId::OTA_UPGRADE]
        );

        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters).unwrap();
        assert_eq!(clusters.len(), 4);
        assert_eq!(
            clusters.last().map(|cluster| cluster.cluster.cluster_id()),
            Some(ClusterId::OTA_UPGRADE)
        );
    }

    #[cfg(feature = "ota")]
    #[test]
    fn ota_decorator_rejects_endpoint_mismatch() {
        use crate::firmware_writer::MockFirmwareWriter;
        use crate::ota::{OtaConfig, OtaManager};

        let ota = OtaManager::new(
            MockFirmwareWriter::new(1024),
            OtaConfig {
                endpoint: 2,
                ..OtaConfig::default()
            },
        );
        assert!(matches!(
            WithOta::new(profile(), ota),
            Err(ProfileError::EndpointMismatch {
                profile_endpoint: 1,
                component_endpoint: 2,
            })
        ));
    }

    /// Zero-sized [`crate::firmware_writer::FirmwareWriter`] stub. Unlike
    /// `MockFirmwareWriter` (a 256 KB RAM buffer, sized for exercising real
    /// staged-image byte layout), `OptionalOta` composition only needs *some*
    /// writer to exist, so a stub avoids inflating every test stack frame
    /// that touches it.
    #[cfg(feature = "ota")]
    struct StubWriter;

    #[cfg(feature = "ota")]
    impl crate::firmware_writer::FirmwareWriter for StubWriter {
        fn erase_slot(&mut self) -> Result<(), crate::firmware_writer::FirmwareError> {
            Ok(())
        }
        fn write_block(
            &mut self,
            _offset: u32,
            _data: &[u8],
        ) -> Result<(), crate::firmware_writer::FirmwareError> {
            Ok(())
        }
        fn verify(
            &mut self,
            _expected_size: u32,
            _expected_hash: Option<&[u8]>,
        ) -> Result<(), crate::firmware_writer::FirmwareError> {
            Ok(())
        }
        fn activate(&mut self) -> Result<(), crate::firmware_writer::FirmwareError> {
            Ok(())
        }
        fn slot_size(&self) -> u32 {
            1024
        }
        fn abort(&mut self) -> Result<(), crate::firmware_writer::FirmwareError> {
            Ok(())
        }
    }

    #[cfg(feature = "ota")]
    #[test]
    fn optional_ota_enabled_adds_client_and_runtime_cluster() {
        use crate::ota::{OtaConfig, OtaManager};

        let ota = OtaManager::new(
            StubWriter,
            OtaConfig {
                endpoint: 1,
                ..OtaConfig::default()
            },
        );
        let mut profile = OptionalOta::enabled(profile(), ota).unwrap();
        assert!(profile.is_enabled());
        let endpoint = EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::TEMPERATURE_SENSOR,
            device_version: 1,
            server_clusters: Vec::new(),
            client_clusters: Vec::new(),
        };
        let endpoint = profile.configure_endpoint(endpoint);
        assert_eq!(
            endpoint.client_clusters.as_slice(),
            &[ClusterId::OTA_UPGRADE]
        );

        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters).unwrap();
        assert_eq!(clusters.len(), 4);
        assert_eq!(
            clusters.last().map(|cluster| cluster.cluster.cluster_id()),
            Some(ClusterId::OTA_UPGRADE)
        );
    }

    #[cfg(feature = "ota")]
    #[test]
    fn optional_ota_disabled_omits_client_cluster_but_keeps_endpoint_working() {
        let mut profile = OptionalOta::<_, StubWriter>::disabled(profile(), 0x1234, 0x0001, 1);
        assert!(!profile.is_enabled());
        let endpoint = EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::TEMPERATURE_SENSOR,
            device_version: 1,
            server_clusters: Vec::new(),
            client_clusters: Vec::new(),
        };
        let endpoint = profile.configure_endpoint(endpoint);
        assert!(
            endpoint.client_clusters.is_empty(),
            "a disabled backend must not advertise the OTA client cluster"
        );
        assert_eq!(profile.expected_report_clusters(), 3);

        // The inert cluster is still collected, so a client that addresses
        // cluster 0x0019 directly gets a well-formed (if unsupported) reply
        // instead of nothing being dispatched at all.
        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters).unwrap();
        assert_eq!(clusters.len(), 4);
        assert_eq!(
            clusters.last().map(|cluster| cluster.cluster.cluster_id()),
            Some(ClusterId::OTA_UPGRADE)
        );
    }

    #[cfg(feature = "ota")]
    #[test]
    fn optional_ota_rejects_endpoint_mismatch_when_enabled() {
        use crate::ota::{OtaConfig, OtaManager};

        let ota = OtaManager::new(
            StubWriter,
            OtaConfig {
                endpoint: 2,
                ..OtaConfig::default()
            },
        );
        assert!(matches!(
            OptionalOta::enabled(profile(), ota),
            Err(ProfileError::EndpointMismatch {
                profile_endpoint: 1,
                component_endpoint: 2,
            })
        ));
    }
}
