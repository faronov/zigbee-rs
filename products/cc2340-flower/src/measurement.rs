//! Product measurement: soil probe, supply/battery, and the detected
//! optional I2C sensors, mapped onto the [`FlowerProfile`] clusters.
//!
//! Power discipline per sample:
//!
//! 1. trigger the one-shot conversions of the fitted I2C sensors (TMP102
//!    one-shot, OPT3001 single-shot, BH1750 one-time: each powers itself
//!    down when done);
//! 2. while they convert, read the supply and run the soil burst (the board
//!    gates the PWM and ADC clocks around it);
//! 3. wait for the longest pending conversion with the async timer, which
//!    lets the CPU idle;
//! 4. read the results.
//!
//! The I2C controller is clocked only for the individual transactions.
//!
//! Battery attributes are refreshed here rather than through the
//! lifecycle's battery hook because the supply reading is also the soil
//! calibration input; [`crate::battery::SampledWithSensors`] is the paired
//! no-op hook.

use embedded_hal::i2c::I2c;
use embedded_hal_async::delay::DelayNs;
use sensor_sed_app::{DiagnosticEvent, Diagnostics, SampleTrigger, SensorMeasurement};
use zigbee_bh1750::{Bh1750, Mode as Bh1750Mode};
use zigbee_opt3001::{ConversionTime, Opt3001};
use zigbee_tmp102::Tmp102;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::illuminance::measured_value_from_centi_lux;

use crate::battery::{battery_measurement, battery_percent};
use crate::calibration::SoilCalibration;
use crate::detection::{DetectedSensors, LightSensor, TemperatureSource};
use crate::profile::FlowerProfile;

/// Conversions averaged per soil sample (reference `soil_moisture(10, …)`).
pub const SOIL_SAMPLES: u8 = 10;
const OPT3001_CONVERSION: ConversionTime = ConversionTime::Ms100;
const BH1750_MODE: Bh1750Mode = Bh1750Mode::OneTimeHighRes;

/// Supply (VDDS) and die-temperature monitor.
pub trait SupplyMonitor {
    /// Uncompensated supply voltage, or `None` if no valid sample exists.
    fn supply_mv(&mut self) -> Option<u16>;
    fn die_temperature_centi_c(&mut self) -> Option<i16>;
}

/// Capacitive soil probe.
pub trait SoilProbe {
    /// Excite the probe and return the mean detector voltage of `samples`
    /// conversions, or `None` if every conversion failed.
    fn average_microvolts(&mut self, samples: u8) -> Option<u32>;
}

#[cfg(target_os = "none")]
impl SupplyMonitor for flower_cc2340r5::batmon::Batmon {
    fn supply_mv(&mut self) -> Option<u16> {
        self.voltage_mv()
    }

    fn die_temperature_centi_c(&mut self) -> Option<i16> {
        self.temperature_centi_c()
    }
}

#[cfg(target_os = "none")]
impl SoilProbe for flower_cc2340r5::SoilProbe {
    fn average_microvolts(&mut self, samples: u8) -> Option<u32> {
        flower_cc2340r5::SoilProbe::average_microvolts(self, samples)
    }
}

/// How often each quantity is refreshed, in periodic lifecycle samples.
/// Cold start and user requests refresh everything.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cadence {
    pub soil_every: u8,
    pub battery_every: u8,
    pub ambient_every: u8,
}

impl Cadence {
    pub const fn is_valid(self) -> bool {
        self.soil_every != 0 && self.battery_every != 0 && self.ambient_every != 0
    }
}

/// With the 5-minute policy sample interval: soil every 10 min, battery
/// hourly, temperature and illuminance every 5 min.
pub const CADENCE: Cadence = Cadence {
    soil_every: 2,
    battery_every: 12,
    ambient_every: 1,
};

const _: () = assert!(CADENCE.is_valid());

/// What one `measure` call refreshes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Due {
    pub soil: bool,
    pub battery: bool,
    pub ambient: bool,
}

pub struct FlowerMeasurement<I2C, S, M, D> {
    bus: I2C,
    soil: S,
    monitor: M,
    delay: D,
    sensors: DetectedSensors,
    calibration: SoilCalibration,
    cadence: Cadence,
    periodic_ticks: u32,
}

impl<I2C, S, M, D> FlowerMeasurement<I2C, S, M, D> {
    /// `sensors` must be the same set the profile was built from.
    pub const fn new(
        bus: I2C,
        soil: S,
        monitor: M,
        delay: D,
        sensors: DetectedSensors,
        calibration: SoilCalibration,
        cadence: Cadence,
    ) -> Self {
        Self {
            bus,
            soil,
            monitor,
            delay,
            sensors,
            calibration,
            cadence,
            periodic_ticks: 0,
        }
    }

    pub const fn sensors(&self) -> DetectedSensors {
        self.sensors
    }

    /// Advance the schedule for `trigger`.
    pub fn schedule(&mut self, trigger: SampleTrigger) -> Due {
        if trigger.refresh_all() {
            self.periodic_ticks = 0;
            return Due {
                soil: true,
                battery: true,
                ambient: true,
            };
        }
        self.periodic_ticks = self.periodic_ticks.wrapping_add(1);
        let tick = self.periodic_ticks;
        Due {
            soil: tick.is_multiple_of(u32::from(self.cadence.soil_every)),
            battery: tick.is_multiple_of(u32::from(self.cadence.battery_every)),
            ambient: tick.is_multiple_of(u32::from(self.cadence.ambient_every)),
        }
    }
}

impl<I2C, S, M, D> SensorMeasurement<FlowerProfile> for FlowerMeasurement<I2C, S, M, D>
where
    I2C: I2c,
    S: SoilProbe,
    M: SupplyMonitor,
    D: DelayNs,
{
    async fn measure<G: Diagnostics>(
        &mut self,
        profile: &mut FlowerProfile,
        trigger: SampleTrigger,
        diagnostics: &mut G,
    ) {
        let due = self.schedule(trigger);
        let plant = profile.component_mut();

        // 1. Start the I2C conversions.
        let temperature_source = self.sensors.temperature_source();
        let light = self.sensors.light_sensor();
        let mut tmp102_started = false;
        let mut light_started = false;
        let mut wait_ms = 0;
        if due.ambient {
            if temperature_source == TemperatureSource::Tmp102 {
                tmp102_started = Tmp102::new(&mut self.bus, zigbee_tmp102::ADDRESS_GND)
                    .start_one_shot()
                    .is_ok();
                if tmp102_started {
                    wait_ms = wait_ms.max(zigbee_tmp102::CONVERSION_TIME_MS);
                }
            }
            match light {
                Some(LightSensor::Opt3001) => {
                    light_started = Opt3001::new(&mut self.bus, zigbee_opt3001::ADDRESS_GND)
                        .start_single_shot(OPT3001_CONVERSION)
                        .is_ok();
                    if light_started {
                        wait_ms = wait_ms.max(OPT3001_CONVERSION.max_delay_ms());
                    }
                }
                Some(LightSensor::Bh1750) => {
                    let mut sensor = Bh1750::new(&mut self.bus, zigbee_bh1750::ADDRESS_LOW);
                    light_started = sensor.power_on().is_ok() && sensor.start(BH1750_MODE).is_ok();
                    if light_started {
                        wait_ms = wait_ms.max(BH1750_MODE.max_delay_ms());
                    }
                }
                None => {}
            }
        }

        // 2. Supply, soil and battery while the sensors convert.
        if due.soil || due.battery {
            let supply_mv = self
                .monitor
                .supply_mv()
                .map(|mv| self.calibration.compensated_supply_mv(mv));
            if due.soil {
                // Without a supply reading the calibration lines are
                // unknown, so the probe is not even excited.
                let soil = supply_mv.and_then(|supply| {
                    self.soil
                        .average_microvolts(SOIL_SAMPLES)
                        .map(|uv| self.calibration.zcl_value(uv, supply))
                });
                match soil {
                    Some(value) => {
                        plant.update_soil_moisture(value);
                        diagnostics.record(DiagnosticEvent::Measurement {
                            cluster_id: ClusterId::SOIL_MOISTURE.0,
                            value: i32::from(value),
                        });
                    }
                    None => diagnostics.record(DiagnosticEvent::MeasurementFailed {
                        cluster_id: ClusterId::SOIL_MOISTURE.0,
                    }),
                }
            }
            if due.battery {
                match supply_mv {
                    Some(supply) => {
                        plant.update_battery(battery_measurement(supply));
                        diagnostics.record(DiagnosticEvent::Battery {
                            millivolts: u32::from(supply),
                            percentage: battery_percent(supply),
                        });
                    }
                    None => diagnostics.record(DiagnosticEvent::BatteryReadFailed),
                }
            }
        }

        if !due.ambient {
            return;
        }

        // 3. Wait for the slowest started conversion.
        if wait_ms != 0 {
            self.delay.delay_ms(wait_ms).await;
        }

        // 4. Temperature: TMP102 if fitted, else the die sensor. A failed
        //    TMP102 read keeps the previous value rather than mixing sources.
        let temperature = match temperature_source {
            TemperatureSource::Tmp102 => tmp102_started
                .then(|| {
                    Tmp102::new(&mut self.bus, zigbee_tmp102::ADDRESS_GND)
                        .read_temperature_centi()
                        .ok()
                })
                .flatten(),
            TemperatureSource::DieTemperature => self.monitor.die_temperature_centi_c(),
        };
        match temperature {
            Some(centi_celsius) => {
                plant.update_temperature(centi_celsius);
                diagnostics.record(DiagnosticEvent::Measurement {
                    cluster_id: ClusterId::TEMPERATURE.0,
                    value: i32::from(centi_celsius),
                });
            }
            None => diagnostics.record(DiagnosticEvent::MeasurementFailed {
                cluster_id: ClusterId::TEMPERATURE.0,
            }),
        }

        // Illuminance exists only with a detected light sensor.
        let Some(light) = light else {
            return;
        };
        let centi_lux = if !light_started {
            None
        } else {
            match light {
                LightSensor::Opt3001 => {
                    let mut sensor = Opt3001::new(&mut self.bus, zigbee_opt3001::ADDRESS_GND);
                    let lux = sensor.read_lux_centi().ok();
                    if lux.is_none() {
                        // Make sure a failed read cannot leave it converting.
                        let _ = sensor.shutdown();
                    }
                    lux
                }
                // One-time mode has already powered the BH1750 down.
                LightSensor::Bh1750 => Bh1750::new(&mut self.bus, zigbee_bh1750::ADDRESS_LOW)
                    .read_lux_centi(BH1750_MODE)
                    .ok(),
            }
        };
        match centi_lux {
            Some(centi_lux) => {
                let value = measured_value_from_centi_lux(centi_lux);
                plant.update_illuminance(value);
                diagnostics.record(DiagnosticEvent::Measurement {
                    cluster_id: ClusterId::ILLUMINANCE.0,
                    value: i32::from(value),
                });
            }
            None => diagnostics.record(DiagnosticEvent::MeasurementFailed {
                cluster_id: ClusterId::ILLUMINANCE.0,
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::calibration::KOPTSERG;
    use crate::detection::fake_bus::FakeBus;
    use crate::profile::flower_profile;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    use std::vec::Vec;
    use zigbee_zcl::clusters::Cluster;
    use zigbee_zcl::data_types::ZclValue;

    fn block_on<F: Future>(future: F) -> F::Output {
        const VTABLE: RawWakerVTable = RawWakerVTable::new(
            |_| RawWaker::new(core::ptr::null(), &VTABLE),
            |_| {},
            |_| {},
            |_| {},
        );
        // SAFETY: the vtable functions ignore the null data pointer.
        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut context = Context::from_waker(&waker);
        let mut future = pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
        }
    }

    struct FakeProbe {
        microvolts: Option<u32>,
        bursts: usize,
    }

    impl SoilProbe for FakeProbe {
        fn average_microvolts(&mut self, samples: u8) -> Option<u32> {
            assert_eq!(samples, SOIL_SAMPLES);
            self.bursts += 1;
            self.microvolts
        }
    }

    struct FakeMonitor {
        supply_mv: Option<u16>,
        die: Option<i16>,
    }

    impl SupplyMonitor for FakeMonitor {
        fn supply_mv(&mut self) -> Option<u16> {
            self.supply_mv
        }

        fn die_temperature_centi_c(&mut self) -> Option<i16> {
            self.die
        }
    }

    #[derive(Default)]
    struct FakeDelay {
        waits_ms: Vec<u32>,
    }

    impl DelayNs for FakeDelay {
        async fn delay_ns(&mut self, ns: u32) {
            self.waits_ms.push(ns / 1_000_000);
        }

        async fn delay_ms(&mut self, ms: u32) {
            self.waits_ms.push(ms);
        }
    }

    #[derive(Default)]
    struct Recorder(Vec<DiagnosticEvent>);

    impl Diagnostics for Recorder {
        fn record(&mut self, event: DiagnosticEvent) {
            self.0.push(event);
        }
    }

    type Measurement = FlowerMeasurement<FakeBus, FakeProbe, FakeMonitor, FakeDelay>;

    fn measurement(sensors: DetectedSensors, bus: FakeBus) -> Measurement {
        FlowerMeasurement::new(
            bus,
            FakeProbe {
                microvolts: Some(1_798_000),
                bursts: 0,
            },
            FakeMonitor {
                supply_mv: Some(2_980),
                die: Some(2_150),
            },
            FakeDelay::default(),
            sensors,
            KOPTSERG,
            CADENCE,
        )
    }

    fn soil(profile: &FlowerProfile) -> Option<ZclValue> {
        profile
            .component()
            .soil_moisture()
            .attributes()
            .get(zigbee_zcl::clusters::soil_moisture::ATTR_MEASURED_VALUE)
            .cloned()
    }

    fn temperature(profile: &FlowerProfile) -> Option<ZclValue> {
        profile
            .component()
            .temperature()
            .attributes()
            .get(zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE)
            .cloned()
    }

    fn illuminance(profile: &FlowerProfile) -> Option<ZclValue> {
        profile
            .component()
            .illuminance()?
            .attributes()
            .get(zigbee_zcl::clusters::illuminance::ATTR_MEASURED_VALUE)
            .cloned()
    }

    #[test]
    fn cold_start_without_optional_sensors_uses_soil_battery_and_die_temperature() {
        let sensors = DetectedSensors::NONE;
        let mut profile = flower_profile(sensors);
        let mut m = measurement(sensors, FakeBus::with(false, false, false));
        let mut diagnostics = Recorder::default();
        block_on(m.measure(&mut profile, SampleTrigger::ColdStart, &mut diagnostics));

        // 2980 mV + 20 mV compensation = 3.0 V; 1.798 V is half way.
        assert_eq!(soil(&profile), Some(ZclValue::U16(5_000)));
        assert_eq!(temperature(&profile), Some(ZclValue::I16(2_150)));
        assert_eq!(illuminance(&profile), None);
        assert!(diagnostics.0.contains(&DiagnosticEvent::Battery {
            millivolts: 3_000,
            percentage: 100,
        }));
        // Nothing to wait for and the bus was never addressed.
        assert!(m.delay.waits_ms.is_empty());
        assert!(m.bus.log.is_empty());
    }

    #[test]
    fn tmp102_and_opt3001_are_triggered_then_read_after_the_longest_conversion() {
        let sensors = DetectedSensors {
            tmp102: true,
            opt3001: true,
            ..DetectedSensors::NONE
        };
        let mut bus = FakeBus::with(true, false, true);
        bus.tmp102_raw = [0x19, 0x00]; // 25.0 C
        bus.opt3001_result = 0x3_000 | 1_000; // exponent 3: 1000 * 0.08 lx = 80 lx
        let mut profile = flower_profile(sensors);
        let mut m = measurement(sensors, bus);
        let mut diagnostics = Recorder::default();
        block_on(m.measure(&mut profile, SampleTrigger::UserRequest, &mut diagnostics));

        assert_eq!(m.delay.waits_ms, [OPT3001_CONVERSION.max_delay_ms()]);
        assert_eq!(temperature(&profile), Some(ZclValue::I16(2_500)));
        // 10000 * log10(80) + 1 = 19032.
        let Some(ZclValue::U16(lux)) = illuminance(&profile) else {
            panic!("illuminance not updated")
        };
        assert!(lux.abs_diff(19_032) <= 1, "{lux}");
        // TMP102 one-shot keeps shutdown (OS | SD).
        assert!(
            m.bus
                .writes_to(0x48)
                .iter()
                .any(|w| w.len() == 3 && w[1] & 0x81 == 0x81)
        );
        // OPT3001 single-shot mode (M = 01).
        assert!(
            m.bus
                .writes_to(0x44)
                .iter()
                .any(|w| w.len() == 3 && u16::from_be_bytes([w[1], w[2]]) & 0x0600 == 0x0200)
        );
    }

    #[test]
    fn bh1750_uses_one_time_mode_and_its_conversion_time() {
        let sensors = DetectedSensors {
            bh1750: true,
            ..DetectedSensors::NONE
        };
        let mut bus = FakeBus::with(false, true, false);
        bus.bh1750_raw = 120; // 100 lx
        let mut profile = flower_profile(sensors);
        let mut m = measurement(sensors, bus);
        let mut diagnostics = Recorder::default();
        block_on(m.measure(&mut profile, SampleTrigger::ColdStart, &mut diagnostics));

        assert_eq!(m.delay.waits_ms, [BH1750_MODE.max_delay_ms()]);
        // Power on, then one-time high resolution; no continuous mode.
        assert_eq!(m.bus.writes_to(0x23), [[0x01], [0x20]]);
        assert_eq!(illuminance(&profile), Some(ZclValue::U16(20_001)));
        // Die temperature fallback.
        assert_eq!(temperature(&profile), Some(ZclValue::I16(2_150)));
    }

    #[test]
    fn periodic_cadence_spreads_soil_and_battery() {
        let sensors = DetectedSensors::NONE;
        let mut profile = flower_profile(sensors);
        let mut m = measurement(sensors, FakeBus::with(false, false, false));
        let mut diagnostics = Recorder::default();
        block_on(m.measure(&mut profile, SampleTrigger::ColdStart, &mut diagnostics));
        assert_eq!(m.soil.bursts, 1);

        let mut battery_ticks = Vec::new();
        for tick in 1..=24u32 {
            let before = diagnostics.0.len();
            block_on(m.measure(&mut profile, SampleTrigger::Periodic, &mut diagnostics));
            let events = &diagnostics.0[before..];
            if events
                .iter()
                .any(|e| matches!(e, DiagnosticEvent::Battery { .. }))
            {
                battery_ticks.push(tick);
            }
            // Ambient (die temperature) every sample.
            assert!(events.iter().any(|e| matches!(
                e,
                DiagnosticEvent::Measurement { cluster_id, .. }
                    if *cluster_id == ClusterId::TEMPERATURE.0
            )));
        }
        // Soil every second 5-minute sample, battery every twelfth.
        assert_eq!(m.soil.bursts, 1 + 12);
        assert_eq!(battery_ticks, [12, 24]);

        // A user request refreshes everything and restarts the schedule.
        block_on(m.measure(&mut profile, SampleTrigger::UserRequest, &mut diagnostics));
        assert_eq!(m.soil.bursts, 14);
        assert_eq!(
            m.schedule(SampleTrigger::Periodic),
            Due {
                soil: false,
                battery: false,
                ambient: true,
            }
        );
    }

    #[test]
    fn failures_keep_previous_values_and_are_reported() {
        let sensors = DetectedSensors {
            tmp102: true,
            bh1750: true,
            ..DetectedSensors::NONE
        };
        let mut bus = FakeBus::with(true, true, false);
        bus.tmp102_raw = [0x19, 0x00];
        bus.bh1750_raw = 120;
        let mut profile = flower_profile(sensors);
        let mut m = measurement(sensors, bus);
        let mut diagnostics = Recorder::default();
        block_on(m.measure(&mut profile, SampleTrigger::ColdStart, &mut diagnostics));
        let good_soil = soil(&profile);
        let good_temperature = temperature(&profile);
        let good_lux = illuminance(&profile);

        // The bus stops returning data, the probe and BATMON fail.
        m.bus.fail_reads = true;
        m.soil.microvolts = None;
        m.monitor.supply_mv = None;
        let mut diagnostics = Recorder::default();
        block_on(m.measure(&mut profile, SampleTrigger::UserRequest, &mut diagnostics));

        assert_eq!(soil(&profile), good_soil);
        assert_eq!(temperature(&profile), good_temperature);
        assert_eq!(illuminance(&profile), good_lux);
        for cluster in [
            ClusterId::SOIL_MOISTURE,
            ClusterId::TEMPERATURE,
            ClusterId::ILLUMINANCE,
        ] {
            assert!(
                diagnostics.0.contains(&DiagnosticEvent::MeasurementFailed {
                    cluster_id: cluster.0
                }),
                "{cluster:?}"
            );
        }
        assert!(diagnostics.0.contains(&DiagnosticEvent::BatteryReadFailed));
        // No supply reading: the probe is not excited at all.
        assert_eq!(m.soil.bursts, 1);
    }
}
