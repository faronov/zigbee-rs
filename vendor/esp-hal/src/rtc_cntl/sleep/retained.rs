//! Pure arithmetic shared by the C6/H2 retained-sleep backport.

/// Admission and preparation failures of the retained-domain sleep backport.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RetainedSleepError {
    /// The current CPU/bus configuration cannot be restored by this backport.
    UnsupportedClock,
    /// An RTC calibration failed or produced an invalid period.
    InvalidCalibration,
    /// The wake duration is zero or exceeds the unambiguous 48-bit interval.
    InvalidDuration,
    /// Only timer and digital GPIO wake are supported by the retained entry.
    UnsupportedWakeSource,
    /// This entry does not power down CPU, memory, flash, peripherals or XTAL.
    UnsupportedPowerConfiguration,
}

pub(crate) fn timer_ticks(us: u128, period: u32) -> Result<u64, RetainedSleepError> {
    if period == 0 {
        return Err(RetainedSleepError::InvalidCalibration);
    }
    let scaled = us
        .checked_mul(1 << 19)
        .ok_or(RetainedSleepError::InvalidDuration)?;
    let ticks = scaled.div_ceil(u128::from(period));
    if ticks == 0 || ticks >= (1 << 47) {
        return Err(RetainedSleepError::InvalidDuration);
    }
    Ok(ticks as u64)
}

pub(crate) fn calibrated_period(
    xtal_cycles: u32,
    xtal_mhz: u32,
    cycles: u32,
) -> Result<u32, RetainedSleepError> {
    if xtal_cycles == 0 || xtal_mhz == 0 || cycles == 0 {
        return Err(RetainedSleepError::InvalidCalibration);
    }

    let divider = u64::from(xtal_mhz) * u64::from(cycles);
    let period = ((u64::from(xtal_cycles) << 19) + divider / 2 - 1) / divider;
    if period == 0 || period > u64::from(u32::MAX) {
        return Err(RetainedSleepError::InvalidCalibration);
    }
    Ok(period as u32)
}

pub(crate) const fn divided_fast_clock(h2: bool, revision: u16) -> bool {
    revision >= if h2 { 2 } else { 1 }
}

pub(crate) fn validate_retained(
    deep: bool,
    power_down: u32,
    wake: u16,
) -> Result<(), RetainedSleepError> {
    if deep || power_down != 0 {
        return Err(RetainedSleepError::UnsupportedPowerConfiguration);
    }
    if wake == 0 || wake & !((1 << 4) | (1 << 2)) != 0 {
        return Err(RetainedSleepError::UnsupportedWakeSource);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timer_rounds_up_and_checks_the_actual_48_bit_limit() {
        assert_eq!(timer_ticks(21, 10 << 19), Ok(3));
        assert_eq!(timer_ticks((1 << 47) - 1, 1 << 19), Ok((1 << 47) - 1));
        assert_eq!(
            timer_ticks(1 << 47, 1 << 19),
            Err(RetainedSleepError::InvalidDuration)
        );
        assert_eq!(
            timer_ticks(u128::MAX, 1),
            Err(RetainedSleepError::InvalidDuration)
        );
        assert_eq!(timer_ticks(0, 1), Err(RetainedSleepError::InvalidDuration));
        assert_eq!(
            timer_ticks(1, 0),
            Err(RetainedSleepError::InvalidCalibration)
        );
    }

    #[test]
    fn calibration_failure_is_not_a_plausible_period() {
        for args in [
            (0, 32, 2048),
            (1, 0, 2048),
            (1, 32, 0),
            (1, u32::MAX, u32::MAX),
        ] {
            assert_eq!(
                calibrated_period(args.0, args.1, args.2),
                Err(RetainedSleepError::InvalidCalibration)
            );
        }
        assert_eq!(calibrated_period(8192, 32, 2048), Ok(1 << 16));
        assert_eq!(calibrated_period(256, 32, 64), Ok(1 << 16));
    }

    #[test]
    fn calibration_division_starts_at_the_correct_chip_revision() {
        assert!(!divided_fast_clock(false, 0));
        assert!(divided_fast_clock(false, 1));
        assert!(!divided_fast_clock(true, 0));
        assert!(!divided_fast_clock(true, 1));
        assert!(divided_fast_clock(true, 2));
    }

    #[test]
    fn entry_rejects_every_power_down_bit_and_unhandled_wake_source() {
        for bit in 0..32 {
            assert_eq!(
                validate_retained(false, 1 << bit, 1 << 4),
                Err(RetainedSleepError::UnsupportedPowerConfiguration)
            );
        }
        assert_eq!(
            validate_retained(true, 0, 1 << 4),
            Err(RetainedSleepError::UnsupportedPowerConfiguration)
        );
        for wake in [0, 1, 1 << 1, 1 << 6, (1 << 4) | (1 << 10)] {
            assert_eq!(
                validate_retained(false, 0, wake),
                Err(RetainedSleepError::UnsupportedWakeSource)
            );
        }
        for wake in [1 << 4, 1 << 2, (1 << 4) | (1 << 2)] {
            assert_eq!(validate_retained(false, 0, wake), Ok(()));
        }
    }
}
