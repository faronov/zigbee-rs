//! Pure arithmetic shared by the C6/H2 retained-sleep backport.

/// Admission and preparation failures of the retained-domain sleep backport.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RetainedSleepError {
    /// The current CPU/bus configuration cannot be restored by this backport.
    UnsupportedClock,
    /// An RTC calibration failed or produced an invalid period.
    InvalidCalibration(CalibrationFailure),
    /// The wake duration is zero or exceeds the unambiguous 48-bit interval.
    InvalidDuration,
    /// Only timer and digital GPIO wake are supported by the retained entry.
    UnsupportedWakeSource,
    /// This entry does not power down CPU, memory, flash or peripherals. Only
    /// the oscillators accepted by [`RetainedPowerDown`] for the chip may stop.
    UnsupportedPowerConfiguration,
}

/// Which RTC calibration step rejected a retained sleep (local extension).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CalibrationFailure {
    /// The boot-time RTC slow-clock period (`LP_AON.store1`) is zero.
    SlowClockPeriodZero,
    /// The TIMG0 RC_FAST calibration timed out: no divided RC_FAST ticks
    /// were counted (`rtc_cali_timeout`, result 0).
    FastClockTimeout,
    /// The RC_FAST calibration completed with an implausible period.
    FastClockOutOfRange,
}

/// Oscillators a retained light sleep may stop (local backport extension).
///
/// CPU, RAM, top/peripheral, modem and flash domains stay powered in every
/// combination, so execution resumes in place. Stopping XTAL also stops
/// SYSTIMER; the caller's calibrated RTC accounting must cover that.
/// Only ESP32-H2 accepts a non-empty set; C6 rejects it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RetainedPowerDown {
    /// Stop the 32 MHz main crystal (`PMU_SLEEP_PD_XTAL`).
    pub xtal: bool,
    /// Stop the RC_FAST oscillator (`RTC_SLEEP_PD_INT_8M`).
    pub rc_fast: bool,
}

impl RetainedPowerDown {
    /// Keep every oscillator running (the original backport behaviour).
    pub const NONE: Self = Self {
        xtal: false,
        rc_fast: false,
    };
    /// ESP-IDF's default light-sleep oscillator set: XTAL and RC_FAST off.
    pub const OSCILLATORS: Self = Self {
        xtal: true,
        rc_fast: true,
    };

    /// `PowerDownFlags` bits (pd_xtal = bit 4, pd_rc_fast = bit 5).
    pub const fn flags(self) -> u32 {
        (if self.xtal { PD_XTAL } else { 0 }) | (if self.rc_fast { PD_RC_FAST } else { 0 })
    }
}

pub(crate) const PD_XTAL: u32 = 1 << 4;
pub(crate) const PD_RC_FAST: u32 = 1 << 5;

/// HP/LP sleep regulator and bias settings (`pmu_sleep_analog_config_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SleepAnalog {
    pub hp_xpd_trx: bool,
    pub hp_pd_cur: bool,
    pub hp_bias_sleep: bool,
    pub hp_xpd: bool,
    pub hp_dbias: u8,
    pub hp_drv_b: u32,
    pub lp_pd_cur: bool,
    pub lp_bias_sleep: bool,
    pub lp_slp_xpd: bool,
    pub lp_slp_dbias: u8,
    pub lp_xpd: bool,
    pub lp_dbias: u8,
    pub lp_drv_b: u8,
}

/// Raw H2 eFuse inputs of the PMU voltage selection (`pmu_param.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct DbiasFuses {
    pub blk_major: u8,
    pub blk_minor: u8,
    pub active_hp: u8,
    pub active_lp: u8,
    pub deep_sleep: u8,
}

impl DbiasFuses {
    const HP_CALI_DBIAS_DEFAULT: u8 = 17;
    const LP_CALI_DBIAS_DEFAULT: u8 = 18;
    const LP_DBIAS_SLEEP_0V7_DEFAULT: u8 = 6;

    fn calibrated(&self) -> bool {
        u32::from(self.blk_major) * 100 + u32::from(self.blk_minor) >= 3
    }

    fn active(&self, fuse: u8, default: u8) -> u8 {
        if self.calibrated() && fuse != 0 {
            (fuse + 2).min(31)
        } else {
            default
        }
    }

    /// `get_act_hp_dbias`.
    pub fn act_hp(&self) -> u8 {
        self.active(self.active_hp, Self::HP_CALI_DBIAS_DEFAULT)
    }

    /// `get_act_lp_dbias`.
    pub fn act_lp(&self) -> u8 {
        self.active(self.active_lp, Self::LP_CALI_DBIAS_DEFAULT)
    }

    /// `get_slp_lp_dbias`.
    pub fn slp_lp(&self) -> u8 {
        if self.calibrated() && self.deep_sleep != 0 {
            self.deep_sleep
        } else {
            Self::LP_DBIAS_SLEEP_0V7_DEFAULT
        }
    }
}

/// ESP-IDF v5.5.1 H2 `pmu_sleep_config_default` light-sleep analog branch.
pub(crate) fn h2_light_sleep_analog(power_down: u32, fuses: &DbiasFuses) -> SleepAnalog {
    // PMU_SLEEP_ANALOG_LSLP_CONFIG_DEFAULT
    let mut analog = SleepAnalog {
        hp_xpd_trx: false,
        hp_pd_cur: true,
        hp_bias_sleep: true,
        hp_xpd: true,
        hp_dbias: 1, // PMU_HP_DBIAS_LIGHTSLEEP_0V6_DEFAULT
        hp_drv_b: 0x00FF_FFF8,
        lp_pd_cur: true,
        lp_bias_sleep: true,
        lp_slp_xpd: false,
        lp_slp_dbias: 0,
        lp_xpd: true,
        lp_dbias: fuses.slp_lp(),
        lp_drv_b: 0,
    };
    if power_down & PD_XTAL == 0 {
        analog.hp_xpd_trx = true;
        analog.hp_dbias = fuses.act_hp();
        analog.hp_pd_cur = false;
        analog.hp_bias_sleep = false;
        analog.lp_pd_cur = false;
        analog.lp_bias_sleep = false;
        analog.lp_dbias = fuses.act_lp();
    } else if power_down & PD_RC_FAST == 0 {
        analog.hp_dbias = fuses.act_hp();
        analog.lp_dbias = fuses.act_lp();
    }
    analog
}

pub(crate) fn timer_ticks(us: u128, period: u32) -> Result<u64, RetainedSleepError> {
    if period == 0 {
        return Err(RetainedSleepError::InvalidCalibration(
            CalibrationFailure::SlowClockPeriodZero,
        ));
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
    const OUT_OF_RANGE: RetainedSleepError =
        RetainedSleepError::InvalidCalibration(CalibrationFailure::FastClockOutOfRange);
    if xtal_cycles == 0 {
        // `calibrate_internal` returns 0 only on `rtc_cali_timeout`.
        return Err(RetainedSleepError::InvalidCalibration(
            CalibrationFailure::FastClockTimeout,
        ));
    }
    if xtal_mhz == 0 || cycles == 0 {
        return Err(OUT_OF_RANGE);
    }

    let divider = u64::from(xtal_mhz) * u64::from(cycles);
    let period = ((u64::from(xtal_cycles) << 19) + divider / 2 - 1) / divider;
    if period == 0 || period > u64::from(u32::MAX) {
        return Err(OUT_OF_RANGE);
    }
    Ok(period as u32)
}

/// RC_FAST period (µs << 19) bounds: 1..40 MHz. The nominal H2 (8 MHz,
/// period 65536) and C6 (17.5 MHz, ~29959) clocks sit well inside; a wrong
/// REF_TICK divider or a stuck counter does not.
#[cfg_attr(not(esp32h2), allow(dead_code))]
pub(crate) const FAST_CLOCK_PERIOD_RANGE: core::ops::RangeInclusive<u32> =
    ((1 << 19) / 40)..=(1 << 19);

/// [`calibrated_period`] for RC_FAST, rejecting implausible frequencies.
#[cfg_attr(not(esp32h2), allow(dead_code))]
pub(crate) fn fast_clock_period(
    xtal_cycles: u32,
    xtal_mhz: u32,
    cycles: u32,
) -> Result<u32, RetainedSleepError> {
    let period = calibrated_period(xtal_cycles, xtal_mhz, cycles)?;
    if !FAST_CLOCK_PERIOD_RANGE.contains(&period) {
        return Err(RetainedSleepError::InvalidCalibration(
            CalibrationFailure::FastClockOutOfRange,
        ));
    }
    Ok(period)
}

pub(crate) const fn divided_fast_clock(h2: bool, revision: u16) -> bool {
    revision >= if h2 { 2 } else { 1 }
}

pub(crate) fn validate_retained(
    deep: bool,
    power_down: u32,
    allowed_power_down: u32,
    wake: u16,
) -> Result<(), RetainedSleepError> {
    if deep || power_down & !allowed_power_down != 0 {
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
            Err(RetainedSleepError::InvalidCalibration(
                CalibrationFailure::SlowClockPeriodZero
            ))
        );
    }

    const TIMEOUT: RetainedSleepError =
        RetainedSleepError::InvalidCalibration(CalibrationFailure::FastClockTimeout);
    const OUT_OF_RANGE: RetainedSleepError =
        RetainedSleepError::InvalidCalibration(CalibrationFailure::FastClockOutOfRange);

    #[test]
    fn calibration_failure_is_not_a_plausible_period() {
        assert_eq!(calibrated_period(0, 32, 2048), Err(TIMEOUT));
        for args in [(1, 0, 2048), (1, 32, 0), (1, u32::MAX, u32::MAX)] {
            assert_eq!(calibrated_period(args.0, args.1, args.2), Err(OUT_OF_RANGE));
        }
        assert_eq!(calibrated_period(8192, 32, 2048), Ok(1 << 16));
        assert_eq!(calibrated_period(256, 32, 64), Ok(1 << 16));
    }

    #[test]
    fn fast_clock_period_distinguishes_timeout_and_implausible_results() {
        // H2 ECO2+: 64 divided ticks = 2048 RC_FAST cycles at 8 MHz is 8192
        // XTAL cycles, `>> 5` = 256 -> period 65536 (0.125 us).
        assert_eq!(fast_clock_period(256, 32, 64), Ok(1 << 16));
        // C6 at 17.5 MHz, 64 ticks: 3657 XTAL(40 MHz) cycles >> 5 = 114.
        assert!(fast_clock_period(117, 40, 64).is_ok());
        assert_eq!(fast_clock_period(0, 32, 64), Err(TIMEOUT));
        // A stuck or undivided counter: below 1 MHz or above 40 MHz.
        assert_eq!(fast_clock_period(1, 32, 64), Err(OUT_OF_RANGE));
        assert_eq!(fast_clock_period(256 * 40, 32, 64), Err(OUT_OF_RANGE));
        assert_eq!(*FAST_CLOCK_PERIOD_RANGE.end(), 1 << 19);
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
                validate_retained(false, 1 << bit, 0, 1 << 4),
                Err(RetainedSleepError::UnsupportedPowerConfiguration)
            );
        }
        assert_eq!(
            validate_retained(true, 0, 0, 1 << 4),
            Err(RetainedSleepError::UnsupportedPowerConfiguration)
        );
        for wake in [0, 1, 1 << 1, 1 << 6, (1 << 4) | (1 << 10)] {
            assert_eq!(
                validate_retained(false, 0, 0, wake),
                Err(RetainedSleepError::UnsupportedWakeSource)
            );
        }
        for wake in [1 << 4, 1 << 2, (1 << 4) | (1 << 2)] {
            assert_eq!(validate_retained(false, 0, 0, wake), Ok(()));
        }
    }

    #[test]
    fn only_the_allowed_oscillators_may_stop() {
        let allowed = RetainedPowerDown::OSCILLATORS.flags();
        assert_eq!(allowed, (1 << 4) | (1 << 5));
        assert_eq!(RetainedPowerDown::NONE.flags(), 0);
        for flags in [PD_XTAL, PD_RC_FAST, allowed] {
            assert_eq!(validate_retained(false, flags, allowed, 1 << 4), Ok(()));
        }
        // Top, VDD_SDIO (flash), modem, CPU and XTAL32K stay powered.
        for bit in [0, 1, 2, 3, 6, 7, 31] {
            assert_eq!(
                validate_retained(false, (1 << bit) | allowed, allowed, 1 << 4),
                Err(RetainedSleepError::UnsupportedPowerConfiguration)
            );
        }
        assert_eq!(
            validate_retained(true, PD_XTAL, allowed, 1 << 4),
            Err(RetainedSleepError::UnsupportedPowerConfiguration)
        );
    }

    const UNFUSED: DbiasFuses = DbiasFuses {
        blk_major: 0,
        blk_minor: 2,
        active_hp: 20,
        active_lp: 21,
        deep_sleep: 9,
    };

    #[test]
    fn dbias_follows_the_efuse_block_version_and_defaults() {
        assert_eq!(
            (UNFUSED.act_hp(), UNFUSED.act_lp(), UNFUSED.slp_lp()),
            (17, 18, 6)
        );
        let fused = DbiasFuses {
            blk_minor: 3,
            ..UNFUSED
        };
        assert_eq!(
            (fused.act_hp(), fused.act_lp(), fused.slp_lp()),
            (22, 23, 9)
        );
        let major = DbiasFuses {
            blk_major: 1,
            blk_minor: 0,
            ..UNFUSED
        };
        assert_eq!(major.act_hp(), 22);
        let blank = DbiasFuses {
            active_hp: 0,
            active_lp: 30,
            deep_sleep: 0,
            ..fused
        };
        assert_eq!(
            (blank.act_hp(), blank.act_lp(), blank.slp_lp()),
            (17, 31, 6)
        );
    }

    #[test]
    fn light_sleep_analog_matches_the_idf_branches() {
        let fused = DbiasFuses {
            blk_minor: 3,
            ..UNFUSED
        };
        // XTAL and RC_FAST off: 0.6 V HP retention, biases off, TRX off.
        let low = h2_light_sleep_analog(PD_XTAL | PD_RC_FAST, &fused);
        assert!(!low.hp_xpd_trx && low.hp_pd_cur && low.hp_bias_sleep && low.hp_xpd);
        assert_eq!((low.hp_dbias, low.hp_drv_b), (1, 0x00FF_FFF8));
        assert!(low.lp_pd_cur && low.lp_bias_sleep && !low.lp_slp_xpd && low.lp_xpd);
        assert_eq!((low.lp_slp_dbias, low.lp_dbias, low.lp_drv_b), (0, 9, 0));
        // RC_FAST kept: active regulator voltages, biases still off.
        let fosc = h2_light_sleep_analog(PD_XTAL, &fused);
        assert_eq!((fosc.hp_dbias, fosc.lp_dbias), (22, 23));
        assert!(fosc.hp_pd_cur && !fosc.hp_xpd_trx);
        // XTAL kept: TRX, current and bias stay on at active voltages.
        for flags in [0, PD_RC_FAST] {
            let on = h2_light_sleep_analog(flags, &fused);
            assert!(on.hp_xpd_trx && !on.hp_pd_cur && !on.hp_bias_sleep);
            assert!(!on.lp_pd_cur && !on.lp_bias_sleep);
            assert_eq!((on.hp_dbias, on.lp_dbias), (22, 23));
        }
    }
}
