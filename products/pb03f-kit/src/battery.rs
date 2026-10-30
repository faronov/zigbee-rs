//! Product-owned supply curve and blocking ADC battery source.
//!
//! The kit is USB powered (3.3 V LDO), but the PB-03F module is the same one a
//! battery product would run directly from two alkaline cells. The product
//! therefore publishes the measured module supply with the same linear
//! 2.0 V (empty) to 3.0 V (full) curve as the PHY62x2 EVK and pvvx THB2
//! devices; a USB-powered kit reads about 3.3 V and reports 100 %.

#[cfg(target_os = "none")]
use sensor_sed_app::{BatteryReading, BlockingBatterySource};
use zigbee_runtime::profile::BatteryMeasurement;

pub const EMPTY_MV: u32 = 2_000;
pub const FULL_MV: u32 = 3_000;

/// ZCL BatteryVoltage, in 100 mV units.
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
        // ZCL BatteryPercentageRemaining is in 0.5 % units.
        percentage_remaining: battery_percent(voltage_mv) * 2,
    }
}

/// Module supply measured by the board's ADC monitor.
#[cfg(target_os = "none")]
pub struct SupplyBattery {
    monitor: pb03f_kit::SupplyMonitor,
}

#[cfg(target_os = "none")]
impl SupplyBattery {
    pub fn new(token: pb03f_kit::SupplyMonitorToken) -> Self {
        Self {
            monitor: token.into_monitor(),
        }
    }
}

#[cfg(target_os = "none")]
impl BlockingBatterySource for SupplyBattery {
    type Error = pb03f_kit::AdcError;

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
    fn percentage_curve_is_linear_from_two_to_three_volts() {
        assert_eq!(battery_percent(1_900), 0);
        assert_eq!(battery_percent(EMPTY_MV), 0);
        assert_eq!(battery_percent(2_500), 50);
        assert_eq!(battery_percent(FULL_MV), 100);
        assert_eq!(battery_percent(3_300), 100);
    }

    #[test]
    fn usb_supply_maps_to_full_in_zcl_units() {
        assert_eq!(
            battery_measurement(3_300),
            BatteryMeasurement {
                voltage_100mv: 33,
                percentage_remaining: 200,
            }
        );
    }
}
