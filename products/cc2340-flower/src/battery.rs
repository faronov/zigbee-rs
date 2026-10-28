//! CR2032 battery conversion (reference `getBatteryRemainingPercentageZCL`).
//!
//! The coin cell is VDDS, measured by the always-on BATMON. Battery
//! attributes are refreshed by [`crate::measurement::FlowerMeasurement`] on
//! the product's hourly cadence; the lifecycle's separate battery hook is
//! therefore [`SampledWithSensors`], which never publishes on its own.

use core::convert::Infallible;

use sensor_sed_app::{BatteryReading, BatterySource};
use zigbee_runtime::profile::{BatteryDescriptor, BatteryMeasurement};

use crate::calibration::map_clamped;

pub const EMPTY_MV: u16 = 2_000;
pub const FULL_MV: u16 = 3_000;

/// ZCL `BatterySize`: CR2032 has no enumerated value, so 0x02 "other".
pub const BATTERY: BatteryDescriptor = BatteryDescriptor {
    size: 0x02,
    quantity: 1,
    rated_voltage_100mv: 30,
};

/// Whole percent over the reference's 2.0-V empty to 3.0-V full line.
pub const fn battery_percent(supply_mv: u16) -> u8 {
    if supply_mv >= FULL_MV {
        100
    } else {
        map_clamped(supply_mv as i64, EMPTY_MV as i64, FULL_MV as i64, 0, 100) as u8
    }
}

/// ZCL `BatteryVoltage` (100 mV) and `BatteryPercentageRemaining` (0.5 %).
pub const fn battery_measurement(supply_mv: u16) -> BatteryMeasurement {
    let voltage = supply_mv / 100;
    BatteryMeasurement {
        voltage_100mv: if voltage > u8::MAX as u16 {
            u8::MAX
        } else {
            voltage as u8
        },
        percentage_remaining: battery_percent(supply_mv) * 2,
    }
}

/// Battery hook for the lifecycle: the battery is sampled together with
/// the soil probe (it is the probe's compensation input), so this source
/// never has a separate sample to publish.
#[derive(Debug, Clone, Copy, Default)]
pub struct SampledWithSensors;

impl BatterySource for SampledWithSensors {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error> {
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentage_follows_the_reference_line_and_clamps() {
        assert_eq!(battery_percent(1_800), 0);
        assert_eq!(battery_percent(EMPTY_MV), 0);
        assert_eq!(battery_percent(2_019), 1);
        assert_eq!(battery_percent(2_500), 50);
        assert_eq!(battery_percent(2_999), 99);
        assert_eq!(battery_percent(FULL_MV), 100);
        assert_eq!(battery_percent(3_300), 100);
    }

    #[test]
    fn measurement_uses_zcl_units() {
        assert_eq!(
            battery_measurement(2_750),
            BatteryMeasurement {
                voltage_100mv: 27,
                percentage_remaining: 150,
            }
        );
        assert_eq!(battery_measurement(3_100).percentage_remaining, 200);
        assert_eq!(battery_measurement(u16::MAX).voltage_100mv, u8::MAX);
    }
}
