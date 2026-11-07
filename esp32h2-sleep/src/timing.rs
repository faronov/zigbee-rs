//! Calibration arithmetic, LP-timer ticks and PMU wait parameters
//! (ESP-IDF v5.5.1 H2 `pmu_sleep.c` `pmu_sleep_param_config_default`,
//! `pmu_sleep_calculate_hw_wait_time`, `PMU_SLEEP_MC_DEFAULT`).

use crate::error::{CalibrationFailure, SleepError};
use crate::power::{PD_TOP, PD_XTAL};

/// Fixed-point fraction of RTC calibration values (`RTC_CLK_CAL_FRACT`).
pub(crate) const CAL_FRACT: u32 = 19;

/// LP-timer ticks for `us`, rounded up. The 48-bit counter wraps, so a target
/// must stay below half of it to compare unambiguously.
pub(crate) fn timer_ticks(us: u128, period: u32) -> Result<u64, SleepError> {
    if period == 0 {
        return Err(SleepError::InvalidCalibration(
            CalibrationFailure::SlowClockPeriodZero,
        ));
    }
    let scaled = us
        .checked_mul(1 << CAL_FRACT)
        .ok_or(SleepError::InvalidDuration)?;
    let ticks = scaled.div_ceil(u128::from(period));
    if ticks == 0 || ticks >= (1 << 47) {
        return Err(SleepError::InvalidDuration);
    }
    Ok(ticks as u64)
}

/// `rtc_clk_cal` result: period in µs << 19 from counted XTAL cycles.
pub(crate) fn calibrated_period(
    xtal_cycles: u32,
    xtal_mhz: u32,
    cycles: u32,
) -> Result<u32, SleepError> {
    const OUT_OF_RANGE: SleepError =
        SleepError::InvalidCalibration(CalibrationFailure::FastClockOutOfRange);
    if xtal_cycles == 0 {
        // The calibration returns 0 only on `rtc_cali_timeout`.
        return Err(SleepError::InvalidCalibration(
            CalibrationFailure::FastClockTimeout,
        ));
    }
    if xtal_mhz == 0 || cycles == 0 {
        return Err(OUT_OF_RANGE);
    }
    let divider = u64::from(xtal_mhz) * u64::from(cycles);
    let period = ((u64::from(xtal_cycles) << CAL_FRACT) + divider / 2 - 1) / divider;
    if period == 0 || period > u64::from(u32::MAX) {
        return Err(OUT_OF_RANGE);
    }
    Ok(period as u32)
}

/// RC_FAST period (µs << 19) bounds: 1..40 MHz. The nominal 8 MHz period is
/// 65536; a wrong REF_TICK divider or a stuck counter falls outside.
pub(crate) const FAST_CLOCK_PERIOD_RANGE: core::ops::RangeInclusive<u32> =
    ((1 << CAL_FRACT) / 40)..=(1 << CAL_FRACT);

/// [`calibrated_period`] for RC_FAST, rejecting implausible frequencies.
pub(crate) fn fast_clock_period(
    xtal_cycles: u32,
    xtal_mhz: u32,
    cycles: u32,
) -> Result<u32, SleepError> {
    let period = calibrated_period(xtal_cycles, xtal_mhz, cycles)?;
    if !FAST_CLOCK_PERIOD_RANGE.contains(&period) {
        return Err(SleepError::InvalidCalibration(
            CalibrationFailure::FastClockOutOfRange,
        ));
    }
    Ok(period)
}

/// H2 ECO2+ (revision >= 2, i.e. v0.2+) divides the calibrated RC_FAST by 32.
pub(crate) const fn divided_fast_clock(revision: u16) -> bool {
    revision >= 2
}

/// RC_FAST cycles counted by the sleep-entry calibration (IDF
/// `FAST_CLK_SRC_CAL_CYCLES`), divided by 32 on ECO2+.
pub(crate) const fn fast_calibration_cycles(revision: u16) -> u32 {
    const FAST_CLK_SRC_CAL_CYCLES: u32 = 2048;
    if divided_fast_clock(revision) {
        FAST_CLK_SRC_CAL_CYCLES / 32
    } else {
        FAST_CLK_SRC_CAL_CYCLES
    }
}

/// `PMU_SLEEP_MC_DEFAULT` (H2). Some constants only feed the informational
/// [`SleepTime::hw_wait_us`] (host-tested; the LP timer target is not
/// adjusted on chip, as in the proven vendor path).
#[allow(dead_code, reason = "informational IDF machine constants")]
pub(crate) mod mc {
    pub const LP_MIN_SLP_TIME_US: u32 = 450;
    pub const LP_ANALOG_WAIT_TIME_US: u32 = 154;
    pub const LP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    pub const LP_CLK_SWITCH_CYCLE: u32 = 1;
    pub const LP_CLK_POWER_ON_WAIT_CYCLE: u32 = 1;
    pub const LP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    pub const LP_POWER_UP_WAIT_TIME_US: u32 = 2;

    pub const HP_MIN_SLP_TIME_US: u32 = 450;
    pub const HP_ANALOG_WAIT_TIME_US: u32 = 154;
    pub const HP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    pub const HP_POWER_UP_WAIT_TIME_US: u32 = 2;
    /// IDF `PMU_REGDMA_S2A_WORK_TIME_PD_TOP_US`: H2 restores by software
    /// (`SOC_PM_RETENTION_SW_TRIGGER_REGDMA`), so the PMU waits for nothing.
    pub const HP_REGDMA_S2A_WORK_TIME_PD_TOP_US: u32 = 0;
    pub const HP_REGDMA_S2A_WORK_TIME_PU_TOP_US: u32 = 390;
    pub const HP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    pub const HP_PLL_WAIT_STABLE_TIME_US: u32 = 50;

    /// `LIGHT_SLEEP_TIME_OVERHEAD_US` (software entry/exit).
    pub const LIGHT_SLEEP_TIME_OVERHEAD_US: u32 = 9;
}

/// Calibrated slow (RTC) and fast (RC_FAST) clock periods, µs << 19.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SleepTime {
    pub slow_period: u32,
    pub fast_period: u32,
}

impl SleepTime {
    pub(crate) fn us_to_slow(&self, us: u32) -> u32 {
        (us << CAL_FRACT) / self.slow_period
    }

    #[allow(dead_code, reason = "used by the informational hw_wait_us")]
    pub(crate) fn slow_to_us(&self, cycles: u32) -> u32 {
        (cycles * self.slow_period) >> CAL_FRACT
    }

    pub(crate) fn us_to_fast(&self, us: u32) -> u32 {
        (us << CAL_FRACT) / self.fast_period
    }

    /// `pmu_sleep_calculate_hw_wait_time`: PMU wake latency before software
    /// runs (informational; the LP timer target is not adjusted, as before).
    #[allow(dead_code, reason = "informational, host-tested")]
    pub(crate) fn hw_wait_us(&self, pd: u32) -> u32 {
        let lp = mc::LP_MIN_SLP_TIME_US
            + mc::LP_ANALOG_WAIT_TIME_US
            + self.slow_to_us(mc::LP_CLK_POWER_ON_WAIT_CYCLE)
            + self.slow_to_us(mc::LP_CLK_SWITCH_CYCLE)
            + mc::LP_POWER_SUPPLY_WAIT_TIME_US
            + mc::LP_POWER_UP_WAIT_TIME_US;
        let regdma = if pd & PD_TOP != 0 {
            mc::HP_REGDMA_S2A_WORK_TIME_PD_TOP_US
        } else {
            mc::HP_REGDMA_S2A_WORK_TIME_PU_TOP_US
        };
        let clock = if pd & PD_XTAL != 0 {
            mc::HP_XTAL_WAIT_STABLE_TIME_US + mc::HP_PLL_WAIT_STABLE_TIME_US
        } else {
            mc::HP_PLL_WAIT_STABLE_TIME_US
        };
        let hp = mc::HP_ANALOG_WAIT_TIME_US
            + mc::HP_POWER_SUPPLY_WAIT_TIME_US
            + mc::HP_POWER_UP_WAIT_TIME_US
            + regdma
            + clock;
        lp + hp
    }

    /// `pmu_sleep_param_config_default`, truncated to the register widths
    /// exactly as IDF's assignments to the narrower struct fields do.
    pub(crate) fn params(&self, xpd_xtal: bool) -> SleepParams {
        SleepParams {
            hp_min_slp: self.us_to_slow(mc::HP_MIN_SLP_TIME_US) as u8,
            lp_min_slp: self.us_to_slow(mc::LP_MIN_SLP_TIME_US) as u8,
            hp_analog_wait: self.us_to_fast(mc::HP_ANALOG_WAIT_TIME_US) as u16,
            lp_analog_wait: self.us_to_slow(mc::LP_ANALOG_WAIT_TIME_US) as u8,
            hp_supply_wait: self.us_to_fast(mc::HP_POWER_SUPPLY_WAIT_TIME_US) as u16,
            hp_power_up_wait: self.us_to_fast(mc::HP_POWER_UP_WAIT_TIME_US) as u16,
            lp_supply_wait: self.us_to_fast(mc::LP_POWER_SUPPLY_WAIT_TIME_US) as u16,
            lp_power_up_wait: self.us_to_fast(mc::LP_POWER_UP_WAIT_TIME_US) as u8,
            // A union in IDF: the slow-clock count when XTAL stays powered.
            xtal_wait: if xpd_xtal {
                self.us_to_slow(mc::LP_XTAL_WAIT_STABLE_TIME_US) as u16
            } else {
                self.us_to_fast(mc::HP_XTAL_WAIT_STABLE_TIME_US) as u16
            },
            pll_wait: self.us_to_fast(mc::HP_PLL_WAIT_STABLE_TIME_US) as u16,
        }
    }
}

/// `pmu_sleep_param_config_t` as written by `pmu_sleep_param_init`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SleepParams {
    pub hp_min_slp: u8,
    pub lp_min_slp: u8,
    pub hp_analog_wait: u16,
    pub lp_analog_wait: u8,
    pub hp_supply_wait: u16,
    pub hp_power_up_wait: u16,
    pub lp_supply_wait: u16,
    pub lp_power_up_wait: u8,
    pub xtal_wait: u16,
    pub pll_wait: u16,
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
            Err(SleepError::InvalidDuration)
        );
        assert_eq!(timer_ticks(u128::MAX, 1), Err(SleepError::InvalidDuration));
        assert_eq!(timer_ticks(0, 1), Err(SleepError::InvalidDuration));
        assert_eq!(
            timer_ticks(1, 0),
            Err(SleepError::InvalidCalibration(
                CalibrationFailure::SlowClockPeriodZero
            ))
        );
    }

    const TIMEOUT: SleepError =
        SleepError::InvalidCalibration(CalibrationFailure::FastClockTimeout);
    const OUT_OF_RANGE: SleepError =
        SleepError::InvalidCalibration(CalibrationFailure::FastClockOutOfRange);

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
        // ECO2+: 64 divided ticks = 2048 RC_FAST cycles at 8 MHz is 8192
        // XTAL cycles, `>> 5` = 256 -> period 65536 (0.125 us).
        assert_eq!(fast_clock_period(256, 32, 64), Ok(1 << 16));
        assert_eq!(fast_clock_period(0, 32, 64), Err(TIMEOUT));
        assert_eq!(fast_clock_period(1, 32, 64), Err(OUT_OF_RANGE));
        assert_eq!(fast_clock_period(256 * 40, 32, 64), Err(OUT_OF_RANGE));
        assert_eq!(*FAST_CLOCK_PERIOD_RANGE.end(), 1 << 19);
    }

    #[test]
    fn calibration_division_starts_at_eco2() {
        assert!(!divided_fast_clock(0));
        assert!(!divided_fast_clock(1));
        assert!(divided_fast_clock(2));
        assert!(divided_fast_clock(102));
        assert_eq!(fast_calibration_cycles(1), 2048);
        assert_eq!(fast_calibration_cycles(2), 64);
    }

    /// RC_SLOW at ~136 kHz (7.35 µs) and RC_FAST at 8 MHz.
    const NOMINAL: SleepTime = SleepTime {
        slow_period: 3_853_516,
        fast_period: 1 << 16,
    };

    #[test]
    fn parameters_match_the_idf_conversions_at_nominal_clocks() {
        let off = NOMINAL.params(false);
        assert_eq!((off.hp_min_slp, off.lp_min_slp), (61, 61));
        assert_eq!((off.hp_analog_wait, off.lp_analog_wait), (1232, 20));
        assert_eq!((off.hp_supply_wait, off.hp_power_up_wait), (16, 16));
        assert_eq!((off.lp_supply_wait, off.lp_power_up_wait), (16, 16));
        assert_eq!((off.xtal_wait, off.pll_wait), (2000, 400));
        // XTAL kept powered: the slow-clock variant of the union.
        assert_eq!(NOMINAL.params(true).xtal_wait, 34);
    }

    #[test]
    fn hardware_wait_uses_the_idf_regdma_times() {
        let pu = NOMINAL.hw_wait_us(PD_XTAL);
        let pd = NOMINAL.hw_wait_us(PD_XTAL | PD_TOP);
        assert_eq!(pu - pd, 390);
        // 450 + 154 + 7 + 7 + 2 + 2 (LP) + 154 + 2 + 2 + 390 + 250 + 50 (HP).
        assert_eq!(pu, 1470);
        assert_eq!(NOMINAL.hw_wait_us(0), 1470 - 250);
    }
}
