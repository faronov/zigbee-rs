//! Product-owned two-cell battery chemistry and blocking ADC source.

#[cfg(target_os = "none")]
use sensor_sed_app::{BatteryReading, BlockingBatterySource};
use zigbee_runtime::profile::BatteryMeasurement;

pub const EMPTY_MV: u32 = 2_000;
pub const FULL_MV: u32 = 3_000;

pub const fn zcl_battery_voltage(voltage_mv: u32) -> u8 {
    let value = voltage_mv / 100;
    if value > u8::MAX as u32 {
        u8::MAX
    } else {
        value as u8
    }
}

pub const fn battery_percent(voltage_mv: u32) -> u8 {
    if voltage_mv <= EMPTY_MV {
        0
    } else if voltage_mv >= FULL_MV {
        100
    } else {
        ((voltage_mv - EMPTY_MV) * 100 / (FULL_MV - EMPTY_MV)) as u8
    }
}

pub const fn battery_measurement(voltage_mv: u32) -> BatteryMeasurement {
    BatteryMeasurement {
        voltage_100mv: zcl_battery_voltage(voltage_mv),
        percentage_remaining: battery_percent(voltage_mv) * 2,
    }
}

#[cfg(target_os = "none")]
pub struct SupplyBattery {
    monitor: phy62x2_evk::SupplyMonitor,
}

#[cfg(target_os = "none")]
impl SupplyBattery {
    pub fn new(token: phy62x2_evk::SupplyMonitorToken) -> Result<Self, phy62x2_evk::AdcError> {
        Ok(Self {
            monitor: token.into_monitor()?,
        })
    }
}

#[cfg(target_os = "none")]
impl BlockingBatterySource for SupplyBattery {
    type Error = phy62x2_evk::AdcError;

    fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error> {
        let millivolts = self.monitor.read_millivolts()?;
        Ok(Some(BatteryReading {
            millivolts,
            measurement: battery_measurement(millivolts),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentage_curve_preserves_the_existing_two_to_three_volt_policy() {
        assert_eq!(battery_percent(1_900), 0);
        assert_eq!(battery_percent(EMPTY_MV), 0);
        assert_eq!(battery_percent(2_500), 50);
        assert_eq!(battery_percent(FULL_MV), 100);
        assert_eq!(battery_percent(3_100), 100);
    }

    #[test]
    fn measurement_uses_zcl_units() {
        assert_eq!(
            battery_measurement(2_500),
            BatteryMeasurement {
                voltage_100mv: 25,
                percentage_remaining: 100,
            }
        );
    }
}
