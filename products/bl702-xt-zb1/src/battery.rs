//! XT-ZB1 supply-voltage reporting policy.
//!
//! The GPADC mechanism lives in `bl702-hal`; this module owns the product's
//! plausible-input window, fallback value, and voltage/percentage curve.

use zigbee_runtime::profile::BatteryMeasurement;

pub const PLAUSIBLE_MIN_MV: u16 = 1_800;
pub const PLAUSIBLE_MAX_MV: u16 = 3_800;
pub const EMPTY_MV: u16 = 2_000;
pub const FULL_MV: u16 = 3_000;
pub const SYNTHETIC_FALLBACK_MV: u16 = 3_000;

pub const fn is_plausible_supply_mv(millivolts: u16) -> bool {
    millivolts >= PLAUSIBLE_MIN_MV && millivolts <= PLAUSIBLE_MAX_MV
}

/// Convert millivolts to ZCL BatteryVoltage in rounded 100 mV units.
pub const fn voltage_100mv(millivolts: u16) -> u8 {
    let rounded = (millivolts as u32 + 50) / 100;
    if rounded > u8::MAX as u32 {
        u8::MAX
    } else {
        rounded as u8
    }
}

/// Preserve the existing linear 2.0 V empty .. 3.0 V full curve.
///
/// The result is in ZCL half-percent units (`0..=200`).
pub const fn percentage_remaining(millivolts: u16) -> u8 {
    if millivolts <= EMPTY_MV {
        0
    } else if millivolts >= FULL_MV {
        200
    } else {
        (((millivolts - EMPTY_MV) as u32 * 200) / (FULL_MV - EMPTY_MV) as u32) as u8
    }
}

pub const fn measurement(millivolts: u16) -> BatteryMeasurement {
    BatteryMeasurement {
        voltage_100mv: voltage_100mv(millivolts),
        percentage_remaining: percentage_remaining(millivolts),
    }
}

pub const fn synthetic_fallback() -> BatteryMeasurement {
    measurement(SYNTHETIC_FALLBACK_MV)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plausibility_window_matches_the_existing_adc_guard() {
        assert!(!is_plausible_supply_mv(1_799));
        assert!(is_plausible_supply_mv(1_800));
        assert!(is_plausible_supply_mv(3_800));
        assert!(!is_plausible_supply_mv(3_801));
    }

    #[test]
    fn voltage_rounding_matches_the_existing_inline_conversion() {
        assert_eq!(voltage_100mv(2_949), 29);
        assert_eq!(voltage_100mv(2_950), 30);
        assert_eq!(voltage_100mv(3_000), 30);
        assert_eq!(voltage_100mv(u16::MAX), u8::MAX);
    }

    #[test]
    fn percentage_curve_clamps_and_interpolates_in_half_percent_units() {
        assert_eq!(percentage_remaining(1_800), 0);
        assert_eq!(percentage_remaining(2_000), 0);
        assert_eq!(percentage_remaining(2_500), 100);
        assert_eq!(percentage_remaining(3_000), 200);
        assert_eq!(percentage_remaining(3_800), 200);
    }

    #[test]
    fn synthetic_fallback_remains_three_volts_and_full() {
        let fallback = synthetic_fallback();
        assert_eq!(fallback.voltage_100mv, 30);
        assert_eq!(fallback.percentage_remaining, 200);
    }
}
