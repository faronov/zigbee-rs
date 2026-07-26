//! Battery voltage/percentage conversion for the on-board SAADC VDD
//! measurement.
//!
//! Extracted verbatim from the original `examples/nrf52840-sensor` inline
//! calculation (no formula change): the SAADC samples the internal VDD
//! divider with the crate's default `saadc::Config` (12-bit, ~3.6 V full
//! scale), and percentage is a linear 1.8 V (empty) .. 3.0 V (full) curve
//! matching a CR2032-style coin cell, as documented in the example README.

use zigbee_runtime::profile::BatteryMeasurement;

const EMPTY_MV: u32 = 1_800;
const FULL_MV: u32 = 3_000;

/// Convert a raw single-ended SAADC sample to millivolts.
///
/// Negative raw samples (possible from SAADC noise near 0 V) are clamped to
/// 0, matching the original `raw.max(0)` behavior.
pub fn millivolts(raw_sample: i16) -> u32 {
    let raw = raw_sample.max(0) as u32;
    raw * 3_600 / 4_096
}

/// ZCL BatteryVoltage, in 100 mV units.
pub const fn zcl_battery_voltage(voltage_mv: u32) -> u8 {
    (voltage_mv / 100) as u8
}

/// Linear percentage (0..=100) between `EMPTY_MV` and `FULL_MV`.
pub const fn battery_percent(voltage_mv: u32) -> u8 {
    if voltage_mv >= FULL_MV {
        100
    } else if voltage_mv <= EMPTY_MV {
        0
    } else {
        ((voltage_mv - EMPTY_MV) * 100 / (FULL_MV - EMPTY_MV)) as u8
    }
}

/// Convert one raw SAADC sample directly into a profile [`BatteryMeasurement`]
/// (ZCL half-percent units, i.e. `battery_percent() * 2`).
pub fn battery_measurement(raw_sample: i16) -> BatteryMeasurement {
    let voltage_mv = millivolts(raw_sample);
    BatteryMeasurement {
        voltage_100mv: zcl_battery_voltage(voltage_mv),
        percentage_remaining: battery_percent(voltage_mv) * 2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn millivolts_matches_original_saadc_scaling() {
        assert_eq!(millivolts(0), 0);
        assert_eq!(millivolts(-1), 0);
        assert_eq!(millivolts(4_096), 3_600);
    }

    #[test]
    fn percentage_curve_clamps_and_interpolates() {
        assert_eq!(battery_percent(1_700), 0);
        assert_eq!(battery_percent(1_800), 0);
        assert_eq!(battery_percent(2_400), 50);
        assert_eq!(battery_percent(3_000), 100);
        assert_eq!(battery_percent(3_300), 100);
    }

    #[test]
    fn battery_measurement_uses_half_percent_units() {
        let measurement = battery_measurement(2_731); // ~2400 mV
        assert_eq!(measurement.voltage_100mv, 24);
        assert_eq!(measurement.percentage_remaining, 100);
    }
}
