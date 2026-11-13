//! Multi-endpoint profile of the DK router with its on-board sensors.
//!
//! | EP | Device | Server clusters | Source |
//! |----|--------|-----------------|--------|
//! | 1 | Range Extender (0x0008) | Basic, Identify, Temperature Measurement (0x0402)¹, Occupancy Sensing (0x0406)¹ | STTS22H; ISM330DHCX motion |
//! | 2 | Simple Sensor (0x000C) | Analog Input (0x000C)¹: distance, mm (BACnet units 30) | VL53L0X |
//! | 3 | Simple Sensor (0x000C) | Analog Input¹: inclination from vertical, degrees (units 90) | ISM330DHCX gravity |
//! | 4 | Simple Sensor (0x000C) | Analog Input²: sound level, dB (units 199), uncalibrated | MP23DB01HP PDM microphone |
//!
//! ¹ feature `sensors`; ² feature `sound`.
//!
//! Occupancy uses sensor type *physical contact* (3): the IMU detects the
//! board being moved or touched, not a person in the room. Its writable
//! `PIROccupiedToUnoccupiedDelay` is the motion hold; see [`crate::motion`].
//!
//! Analog Input reports `Reliability` (0x0067) and the `FAULT` bit of
//! `StatusFlags` when a reading is unavailable: *over range* (2) when the
//! ToF sensor sees no target within [`MAX_DISTANCE_MM`], *no sensor* (1)
//! when a sensor has not produced a value. `EngineeringUnits`, `Min`/`Max`
//! and `Resolution` are re-asserted on every update because the generic
//! cluster lets clients write them and resets them to "no units" on a
//! Basic *reset to factory defaults*.

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::MacDriver;
use zigbee_runtime::builder::{DeviceBuilder, EndpointBuilder};
use zigbee_runtime::profile::{
    ApplicationClusters, ApplicationProfile, ExpectedReportClusters, ProfileComponent,
    ProfileError, RangeExtender,
};
use zigbee_runtime::role::DeviceRole;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::analog_input::{
    ATTR_ENGINEERING_UNITS, ATTR_MAX_PRESENT_VALUE, ATTR_MIN_PRESENT_VALUE, ATTR_PRESENT_VALUE,
    ATTR_RELIABILITY, ATTR_RESOLUTION, ATTR_STATUS_FLAGS, AnalogInputCluster,
};
#[cfg(feature = "sensors")]
use zigbee_zcl::clusters::occupancy::ATTR_OCCUPANCY;
#[cfg(feature = "sensors")]
use zigbee_zcl::clusters::temperature::{
    ATTR_MEASURED_VALUE as TEMPERATURE_MEASURED_VALUE, TemperatureCluster,
};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
use zigbee_zcl::{ClusterId, DeviceId};

use crate::ENDPOINT;
#[cfg(feature = "sensors")]
use crate::measure::MAX_DISTANCE_MM;
#[cfg(feature = "sensors")]
use crate::motion::MotionOccupancyCluster;
use crate::screen::SensorSnapshot;

pub const DISTANCE_ENDPOINT: u8 = 2;
pub const INCLINATION_ENDPOINT: u8 = 3;
pub const SOUND_ENDPOINT: u8 = 4;

/// STTS22H operating range, 0.01 °C.
pub const TEMPERATURE_RANGE_CENTI_C: (i16, i16) = (-4_000, 12_500);
/// ZCL Temperature Measurement "invalid" value.
pub const TEMPERATURE_INVALID: i16 = i16::MIN;

/// BACnet engineering units (ASHRAE 135 `BACnetEngineeringUnits`).
pub mod units {
    pub const MILLIMETERS: u16 = 30;
    pub const DEGREES_ANGULAR: u16 = 90;
    pub const DECIBELS: u16 = 199;
}

/// BACnet `Reliability` values used here.
pub mod reliability {
    pub const NO_FAULT_DETECTED: u8 = 0;
    pub const NO_SENSOR: u8 = 1;
    pub const OVER_RANGE: u8 = 2;
}

const STATUS_FLAG_FAULT: u8 = 0x02;

/// Static description of one Analog Input endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalogChannel {
    pub endpoint: u8,
    pub units: u16,
    pub min: f32,
    pub max: f32,
    pub resolution: f32,
    /// Default reporting.
    pub min_interval_secs: u16,
    pub max_interval_secs: u16,
    pub reportable_change: f32,
}

pub const DISTANCE: AnalogChannel = AnalogChannel {
    endpoint: DISTANCE_ENDPOINT,
    units: units::MILLIMETERS,
    min: 0.0,
    max: 2_000.0,
    resolution: 1.0,
    min_interval_secs: 5,
    max_interval_secs: 300,
    reportable_change: 20.0,
};

pub const INCLINATION: AnalogChannel = AnalogChannel {
    endpoint: INCLINATION_ENDPOINT,
    units: units::DEGREES_ANGULAR,
    min: 0.0,
    max: 180.0,
    resolution: 0.01,
    min_interval_secs: 5,
    max_interval_secs: 300,
    reportable_change: 2.0,
};

pub const SOUND: AnalogChannel = AnalogChannel {
    endpoint: SOUND_ENDPOINT,
    units: units::DECIBELS,
    min: 0.0,
    max: 130.0,
    resolution: 0.01,
    min_interval_secs: 10,
    max_interval_secs: 300,
    reportable_change: 3.0,
};

/// One Analog Input endpoint plus its descriptor.
pub struct AnalogEndpoint {
    channel: AnalogChannel,
    cluster: AnalogInputCluster,
}

impl AnalogEndpoint {
    pub fn new(channel: AnalogChannel) -> Self {
        let mut this = Self {
            channel,
            cluster: AnalogInputCluster::new(),
        };
        this.set(None, reliability::NO_SENSOR);
        this
    }

    pub const fn channel(&self) -> &AnalogChannel {
        &self.channel
    }

    pub fn cluster(&self) -> &AnalogInputCluster {
        &self.cluster
    }

    /// Publish `value` (or keep the last one when `None`) with the given
    /// reliability.
    pub fn set(&mut self, value: Option<f32>, reliability: u8) {
        let c = &self.channel;
        let attributes = self.cluster.attributes_mut();
        let _ = attributes.set_raw(ATTR_ENGINEERING_UNITS, ZclValue::U16(c.units));
        let _ = attributes.set_raw(ATTR_MIN_PRESENT_VALUE, ZclValue::Float32(c.min));
        let _ = attributes.set_raw(ATTR_MAX_PRESENT_VALUE, ZclValue::Float32(c.max));
        let _ = attributes.set_raw(ATTR_RESOLUTION, ZclValue::Float32(c.resolution));
        let _ = attributes.set_raw(ATTR_RELIABILITY, ZclValue::U8(reliability));
        let flags = if reliability == reliability::NO_FAULT_DETECTED {
            0
        } else {
            STATUS_FLAG_FAULT
        };
        let _ = attributes.set_raw(ATTR_STATUS_FLAGS, ZclValue::U8(flags));
        if let Some(value) = value {
            self.cluster.set_present_value(value.clamp(c.min, c.max));
        }
    }

    pub fn present_value(&self) -> Option<f32> {
        match self.cluster.attributes().get(ATTR_PRESENT_VALUE) {
            Some(ZclValue::Float32(v)) => Some(*v),
            _ => None,
        }
    }

    pub fn reliability(&self) -> Option<u8> {
        match self.cluster.attributes().get(ATTR_RELIABILITY) {
            Some(ZclValue::U8(v)) => Some(*v),
            _ => None,
        }
    }

    fn declare(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        endpoint.cluster_server(ClusterId::ANALOG_INPUT)
    }

    fn configure_reporting<M: MacDriver, R: DeviceRole>(
        &self,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        let c = &self.channel;
        device
            .reporting_mut()
            .configure_for_cluster(
                c.endpoint,
                ClusterId::ANALOG_INPUT.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_PRESENT_VALUE,
                    data_type: ZclDataType::Float32,
                    min_interval: c.min_interval_secs,
                    max_interval: c.max_interval_secs,
                    reportable_change: Some(ZclValue::Float32(c.reportable_change)),
                },
            )
            .map_err(ProfileError::Reporting)
    }
}

/// The DK's endpoints. Construct with [`DkProfile::new`].
pub struct DkProfile {
    #[cfg(feature = "sensors")]
    temperature: TemperatureCluster,
    #[cfg(feature = "sensors")]
    occupancy: MotionOccupancyCluster,
    #[cfg(feature = "sensors")]
    distance: AnalogEndpoint,
    #[cfg(feature = "sensors")]
    inclination: AnalogEndpoint,
    #[cfg(feature = "sound")]
    sound: AnalogEndpoint,
}

impl Default for DkProfile {
    fn default() -> Self {
        Self::new()
    }
}

impl DkProfile {
    pub fn new() -> Self {
        #[cfg(feature = "sensors")]
        let temperature = {
            let (min, max) = TEMPERATURE_RANGE_CENTI_C;
            let mut temperature = TemperatureCluster::new(min, max);
            temperature.set_temperature(TEMPERATURE_INVALID);
            temperature
        };
        Self {
            #[cfg(feature = "sensors")]
            temperature,
            #[cfg(feature = "sensors")]
            occupancy: MotionOccupancyCluster::new(),
            #[cfg(feature = "sensors")]
            distance: AnalogEndpoint::new(DISTANCE),
            #[cfg(feature = "sensors")]
            inclination: AnalogEndpoint::new(INCLINATION),
            #[cfg(feature = "sound")]
            sound: AnalogEndpoint::new(SOUND),
        }
    }

    /// Declare every endpoint of this profile on the device builder.
    pub fn declare_endpoints<M: MacDriver>(&self, builder: DeviceBuilder<M>) -> DeviceBuilder<M> {
        let builder = builder.endpoint(
            self.endpoint(),
            self.profile_id(),
            self.device_id(),
            |endpoint| self.configure_endpoint(endpoint),
        );
        #[cfg(feature = "sensors")]
        let builder = {
            let builder = self.analog_endpoint(builder, &self.distance);
            self.analog_endpoint(builder, &self.inclination)
        };
        #[cfg(feature = "sound")]
        let builder = self.analog_endpoint(builder, &self.sound);
        builder
    }

    #[cfg(any(feature = "sensors", feature = "sound"))]
    fn analog_endpoint<M: MacDriver>(
        &self,
        builder: DeviceBuilder<M>,
        analog: &AnalogEndpoint,
    ) -> DeviceBuilder<M> {
        builder.endpoint(
            analog.channel.endpoint,
            PROFILE_HOME_AUTOMATION,
            DeviceId::SIMPLE_SENSOR,
            |endpoint| analog.declare(endpoint),
        )
    }

    /// STTS22H reading in 0.01 °C; `None` marks the value invalid.
    #[cfg(feature = "sensors")]
    pub fn set_temperature(&mut self, centi_c: Option<i16>) {
        let (min, max) = TEMPERATURE_RANGE_CENTI_C;
        self.temperature
            .set_temperature(centi_c.map_or(TEMPERATURE_INVALID, |t| t.clamp(min, max)));
    }

    #[cfg(feature = "sensors")]
    pub fn set_occupied(&mut self, occupied: bool) {
        self.occupancy.set_occupied(occupied);
    }

    /// The Occupancy Sensing cluster, for the motion hold and its
    /// persistence.
    #[cfg(feature = "sensors")]
    pub fn occupancy_mut(&mut self) -> &mut MotionOccupancyCluster {
        &mut self.occupancy
    }

    /// ToF distance: `Some(mm)` for a valid range, `None` when no target is
    /// within [`MAX_DISTANCE_MM`] (reported as the maximum, over range).
    #[cfg(feature = "sensors")]
    pub fn set_distance(&mut self, distance_mm: Option<u16>) {
        match distance_mm {
            Some(mm) => self
                .distance
                .set(Some(f32::from(mm)), reliability::NO_FAULT_DETECTED),
            None => self
                .distance
                .set(Some(f32::from(MAX_DISTANCE_MM)), reliability::OVER_RANGE),
        }
    }

    /// Inclination from vertical in 0.01°; `None` while the board is
    /// accelerating (no clean gravity vector), keeping the last value.
    #[cfg(feature = "sensors")]
    pub fn set_inclination(&mut self, centi_deg: Option<u16>) {
        if let Some(centi) = centi_deg {
            self.inclination.set(
                Some(f32::from(centi) / 100.0),
                reliability::NO_FAULT_DETECTED,
            );
        }
    }

    /// Mark the I²C sensors unavailable (bus or probe failure).
    #[cfg(feature = "sensors")]
    pub fn set_sensors_missing(&mut self) {
        self.set_temperature(None);
        self.distance.set(None, reliability::NO_SENSOR);
        self.inclination.set(None, reliability::NO_SENSOR);
    }

    /// Sound level in 0.01 dB SPL (nominal sensitivity, uncalibrated);
    /// `None` = microphone not delivering.
    #[cfg(feature = "sound")]
    pub fn set_sound(&mut self, centi_db: Option<i16>) {
        match centi_db {
            Some(centi) => self.sound.set(
                Some(f32::from(centi) / 100.0),
                reliability::NO_FAULT_DETECTED,
            ),
            None => self.sound.set(None, reliability::NO_SENSOR),
        }
    }

    /// Apply the latest readings of the sensor tasks.
    pub fn apply(&mut self, readings: &SensorSnapshot) {
        #[cfg(feature = "sensors")]
        {
            self.set_temperature(readings.temperature_centi_c);
            match readings.distance_mm {
                Some(distance) => self.set_distance(distance),
                None => self.distance.set(None, reliability::NO_SENSOR),
            }
            match readings.occupied {
                Some(occupied) => {
                    self.set_occupied(occupied);
                    self.set_inclination(readings.inclination_centi_deg);
                }
                None => {
                    self.set_occupied(false);
                    self.inclination.set(None, reliability::NO_SENSOR);
                }
            }
        }
        #[cfg(feature = "sound")]
        self.set_sound(readings.sound_centi_db);
        let _ = readings;
    }

    #[cfg(feature = "sensors")]
    pub fn distance(&self) -> &AnalogEndpoint {
        &self.distance
    }

    #[cfg(feature = "sensors")]
    pub fn inclination(&self) -> &AnalogEndpoint {
        &self.inclination
    }

    #[cfg(feature = "sound")]
    pub fn sound(&self) -> &AnalogEndpoint {
        &self.sound
    }
}

impl ApplicationProfile for DkProfile {
    fn endpoint(&self) -> u8 {
        ENDPOINT
    }

    fn profile_id(&self) -> u16 {
        PROFILE_HOME_AUTOMATION
    }

    fn device_id(&self) -> DeviceId {
        DeviceId::RANGE_EXTENDER
    }

    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        let endpoint = RangeExtender.configure_endpoint(endpoint);
        #[cfg(feature = "sensors")]
        let endpoint = endpoint
            .cluster_server(ClusterId::TEMPERATURE)
            .cluster_server(ClusterId::OCCUPANCY);
        endpoint
    }

    fn collect_clusters<'a>(
        &'a mut self,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        #[cfg(feature = "sensors")]
        {
            push(clusters, ENDPOINT, &mut self.temperature)?;
            push(clusters, ENDPOINT, &mut self.occupancy)?;
            push(clusters, DISTANCE_ENDPOINT, &mut self.distance.cluster)?;
            push(
                clusters,
                INCLINATION_ENDPOINT,
                &mut self.inclination.cluster,
            )?;
        }
        #[cfg(feature = "sound")]
        push(clusters, SOUND_ENDPOINT, &mut self.sound.cluster)?;
        let _ = clusters;
        Ok(())
    }

    /// Interview completion is judged on endpoint 1 only (a runtime
    /// contract); the Analog Input endpoints carry device-side default
    /// reporting instead.
    fn expected_report_cluster_ids(&self, out: &mut ExpectedReportClusters) {
        #[cfg(feature = "sensors")]
        {
            let _ = out.push(ClusterId::TEMPERATURE.0);
            let _ = out.push(ClusterId::OCCUPANCY.0);
        }
        let _ = out;
    }

    fn configure_default_reporting<M: MacDriver, R: DeviceRole>(
        &self,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        #[cfg(feature = "sensors")]
        {
            let reporting = device.reporting_mut();
            reporting
                .configure_for_cluster(
                    ENDPOINT,
                    ClusterId::TEMPERATURE.0,
                    ReportingConfig {
                        direction: ReportDirection::Send,
                        attribute_id: TEMPERATURE_MEASURED_VALUE,
                        data_type: ZclDataType::I16,
                        min_interval: 10,
                        max_interval: 300,
                        reportable_change: Some(ZclValue::I16(20)),
                    },
                )
                .map_err(ProfileError::Reporting)?;
            reporting
                .configure_for_cluster(
                    ENDPOINT,
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
            self.distance.configure_reporting(device)?;
            self.inclination.configure_reporting(device)?;
        }
        #[cfg(feature = "sound")]
        self.sound.configure_reporting(device)?;
        let _ = device;
        Ok(())
    }
}

#[cfg(any(feature = "sensors", feature = "sound"))]
fn push<'a>(
    clusters: &mut ApplicationClusters<'a>,
    endpoint: u8,
    cluster: &'a mut dyn Cluster,
) -> Result<(), ProfileError> {
    clusters
        .push(ClusterRef { endpoint, cluster })
        .map_err(|_| ProfileError::TooManyClusters)
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_mac::mock::MockMac;
    use zigbee_runtime::profile::ApplicationProfile;
    use zigbee_runtime::role::Router;

    fn device(profile: &DkProfile) -> ZigbeeDevice<MockMac, Router> {
        profile
            .declare_endpoints(ZigbeeDevice::builder(MockMac::new([
                1, 2, 3, 4, 5, 6, 7, 8,
            ])))
            .build_router()
    }

    #[test]
    fn endpoints_and_simple_descriptors() {
        let profile = DkProfile::new();
        let device = device(&profile);
        let zdo = device.bdb().zdo();
        let ep1 = zdo.find_endpoint(ENDPOINT).unwrap();
        assert_eq!(ep1.device_id, DeviceId::RANGE_EXTENDER.0);
        #[cfg(feature = "sensors")]
        {
            assert_eq!(
                ep1.input_clusters.as_slice(),
                &[0x0000, 0x0003, 0x0402, 0x0406]
            );
            for ep in [DISTANCE_ENDPOINT, INCLINATION_ENDPOINT] {
                let d = zdo.find_endpoint(ep).unwrap();
                assert_eq!(d.device_id, DeviceId::SIMPLE_SENSOR.0);
                assert_eq!(d.profile_id, PROFILE_HOME_AUTOMATION);
                assert_eq!(d.input_clusters.as_slice(), &[0x000C]);
            }
        }
        #[cfg(not(feature = "sensors"))]
        assert_eq!(ep1.input_clusters.as_slice(), &[0x0000, 0x0003]);
        #[cfg(feature = "sound")]
        assert_eq!(
            zdo.find_endpoint(SOUND_ENDPOINT)
                .unwrap()
                .input_clusters
                .as_slice(),
            &[0x000C]
        );
        #[cfg(not(feature = "sound"))]
        assert!(zdo.find_endpoint(SOUND_ENDPOINT).is_none());
    }

    #[test]
    fn clusters_are_collected_on_their_endpoints() {
        let mut profile = DkProfile::new();
        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters).unwrap();
        let mut seen: heapless::Vec<(u8, u16), 8> = heapless::Vec::new();
        for c in clusters.iter() {
            seen.push((c.endpoint, c.cluster.cluster_id().0)).unwrap();
        }
        let mut expected: heapless::Vec<(u8, u16), 8> = heapless::Vec::new();
        #[cfg(feature = "sensors")]
        expected
            .extend_from_slice(&[(1, 0x0402), (1, 0x0406), (2, 0x000C), (3, 0x000C)])
            .unwrap();
        #[cfg(feature = "sound")]
        expected.push((4, 0x000C)).unwrap();
        assert_eq!(seen, expected);
    }

    #[test]
    fn default_reporting_is_accepted() {
        let profile = DkProfile::new();
        let mut device = device(&profile);
        profile.configure_default_reporting(&mut device).unwrap();
        let mut expected = ExpectedReportClusters::new();
        profile.expected_report_cluster_ids(&mut expected);
        #[cfg(feature = "sensors")]
        assert_eq!(expected.as_slice(), &[0x0402, 0x0406]);
        #[cfg(not(feature = "sensors"))]
        assert!(expected.is_empty());
    }

    #[cfg(feature = "sensors")]
    #[test]
    fn analog_inputs_carry_units_ranges_and_reliability() {
        let mut profile = DkProfile::new();
        assert_eq!(
            profile.distance().reliability(),
            Some(reliability::NO_SENSOR)
        );

        profile.set_distance(Some(123));
        let attrs = profile.distance().cluster().attributes();
        assert_eq!(attrs.get(ATTR_ENGINEERING_UNITS), Some(&ZclValue::U16(30)));
        assert_eq!(profile.distance().present_value(), Some(123.0));
        assert_eq!(profile.distance().reliability(), Some(0));
        assert_eq!(attrs.get(ATTR_STATUS_FLAGS), Some(&ZclValue::U8(0)));

        profile.set_distance(None);
        assert_eq!(profile.distance().present_value(), Some(2_000.0));
        assert_eq!(
            profile.distance().reliability(),
            Some(reliability::OVER_RANGE)
        );
        assert_eq!(
            profile
                .distance()
                .cluster()
                .attributes()
                .get(ATTR_STATUS_FLAGS),
            Some(&ZclValue::U8(STATUS_FLAG_FAULT))
        );

        profile.set_inclination(Some(4_550));
        assert_eq!(profile.inclination().present_value(), Some(45.5));
        profile.set_inclination(None);
        assert_eq!(profile.inclination().present_value(), Some(45.5), "kept");
        assert_eq!(
            profile
                .inclination()
                .cluster()
                .attributes()
                .get(ATTR_ENGINEERING_UNITS),
            Some(&ZclValue::U16(90))
        );
    }

    #[cfg(feature = "sensors")]
    #[test]
    fn readings_map_to_reliability() {
        let mut profile = DkProfile::new();
        profile.apply(&SensorSnapshot {
            temperature_centi_c: Some(2_100),
            distance_mm: Some(Some(321)),
            inclination_centi_deg: Some(9_000),
            occupied: Some(true),
            sound_centi_db: Some(4_000),
        });
        assert_eq!(profile.distance().present_value(), Some(321.0));
        assert_eq!(profile.inclination().present_value(), Some(90.0));
        assert_eq!(
            profile.occupancy.attributes().get(ATTR_OCCUPANCY),
            Some(&ZclValue::Bitmap8(1))
        );

        profile.apply(&SensorSnapshot {
            distance_mm: Some(None),
            ..SensorSnapshot::EMPTY
        });
        assert_eq!(
            profile.distance().reliability(),
            Some(reliability::OVER_RANGE)
        );
        assert_eq!(
            profile.inclination().reliability(),
            Some(reliability::NO_SENSOR),
            "IMU gone"
        );
        assert_eq!(
            profile.occupancy.attributes().get(ATTR_OCCUPANCY),
            Some(&ZclValue::Bitmap8(0))
        );

        profile.apply(&SensorSnapshot::EMPTY);
        assert_eq!(
            profile.distance().reliability(),
            Some(reliability::NO_SENSOR)
        );
    }

    #[cfg(feature = "sensors")]
    #[test]
    fn units_survive_a_factory_defaults_reset() {
        let mut profile = DkProfile::new();
        profile.distance.cluster.reset_to_factory_defaults();
        profile.set_distance(Some(5));
        assert_eq!(
            profile
                .distance()
                .cluster()
                .attributes()
                .get(ATTR_ENGINEERING_UNITS),
            Some(&ZclValue::U16(30))
        );
    }

    #[cfg(feature = "sensors")]
    #[test]
    fn temperature_starts_invalid_and_clamps() {
        let mut profile = DkProfile::new();
        let value = |p: &DkProfile| {
            p.temperature
                .attributes()
                .get(TEMPERATURE_MEASURED_VALUE)
                .cloned()
        };
        assert_eq!(value(&profile), Some(ZclValue::I16(TEMPERATURE_INVALID)));
        profile.set_temperature(Some(2_345));
        assert_eq!(value(&profile), Some(ZclValue::I16(2_345)));
        profile.set_temperature(Some(20_000));
        assert_eq!(value(&profile), Some(ZclValue::I16(12_500)));
        profile.set_sensors_missing();
        assert_eq!(value(&profile), Some(ZclValue::I16(TEMPERATURE_INVALID)));
    }

    #[cfg(feature = "sound")]
    #[test]
    fn sound_level_is_published_in_decibels() {
        let mut profile = DkProfile::new();
        profile.set_sound(Some(6_214));
        assert_eq!(profile.sound().present_value(), Some(62.14));
        assert_eq!(
            profile
                .sound()
                .cluster()
                .attributes()
                .get(ATTR_ENGINEERING_UNITS),
            Some(&ZclValue::U16(199))
        );
        profile.set_sound(None);
        assert_eq!(profile.sound().reliability(), Some(reliability::NO_SENSOR));
    }
}
