//! Product battery conversion and bring-up source policy.
//!
//! The LP-EM board has no verified pure-Rust supply-voltage adapter yet.
//! The selected [`FixedBattery`](sensor_sed_app::FixedBattery) is therefore
//! explicitly synthetic and must be replaced when an ADC/BATMON adapter is
//! available.

use sensor_sed_app::FixedBattery;
use zigbee_runtime::profile::BatteryMeasurement;

pub const EMPTY_MV: u16 = 1_800;
pub const FULL_MV: u16 = 3_000;
pub const SYNTHETIC_FIXED_MV: u16 = 3_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatterySourceDisposition {
    SyntheticFixedUntilAdc,
}

pub const fn source_disposition() -> BatterySourceDisposition {
    BatterySourceDisposition::SyntheticFixedUntilAdc
}

/// ZCL BatteryVoltage in 100-mV units.
pub const fn zcl_battery_voltage(voltage_mv: u16) -> u8 {
    let value = voltage_mv / 100;
    if value > u8::MAX as u16 {
        u8::MAX
    } else {
        value as u8
    }
}

/// Linear percentage over the product's 1.8-V empty to 3.0-V full curve.
pub const fn battery_percent(voltage_mv: u16) -> u8 {
    if voltage_mv <= EMPTY_MV {
        0
    } else if voltage_mv >= FULL_MV {
        100
    } else {
        ((voltage_mv - EMPTY_MV) as u32 * 100 / (FULL_MV - EMPTY_MV) as u32) as u8
    }
}

/// Convert a supplied voltage to the ZCL battery attributes.
pub const fn battery_measurement(voltage_mv: u16) -> BatteryMeasurement {
    BatteryMeasurement {
        voltage_100mv: zcl_battery_voltage(voltage_mv),
        percentage_remaining: battery_percent(voltage_mv) * 2,
    }
}

/// Construct the explicitly synthetic source selected by this bring-up
/// product. It is not a hardware battery measurement.
pub const fn fixed_battery() -> FixedBattery {
    FixedBattery::new(
        SYNTHETIC_FIXED_MV as u32,
        battery_measurement(SYNTHETIC_FIXED_MV),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentage_curve_clamps_and_interpolates() {
        assert_eq!(battery_percent(1_700), 0);
        assert_eq!(battery_percent(EMPTY_MV), 0);
        assert_eq!(battery_percent(2_400), 50);
        assert_eq!(battery_percent(FULL_MV), 100);
        assert_eq!(battery_percent(3_300), 100);
    }

    #[test]
    fn measurement_uses_zcl_units() {
        assert_eq!(
            battery_measurement(2_400),
            BatteryMeasurement {
                voltage_100mv: 24,
                percentage_remaining: 100,
            }
        );
    }

    #[test]
    fn absent_hardware_measurement_is_explicit() {
        assert_eq!(
            source_disposition(),
            BatterySourceDisposition::SyntheticFixedUntilAdc
        );
        assert_eq!(SYNTHETIC_FIXED_MV, FULL_MV);
    }
}
