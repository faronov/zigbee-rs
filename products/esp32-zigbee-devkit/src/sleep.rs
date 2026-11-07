//! Calibrated accounting and a retained ESP32-C6/H2 SoC light-sleep boundary.
//!
//! ESP32-C6 uses the repository's esp-hal 1.0.0 retained-sleep backport;
//! ESP32-H2 uses the product's own RAM-resident `esp32h2-sleep` crate (a port
//! of the same ESP-IDF v5.5.1 sequence, see `esp32h2-sleep/README.md`). Neither
//! substitutes WFI or deep sleep. The default mode keeps the CPU/RAM/flash/
//! peripheral domains retained; the H2-only, experimental
//! `SleepMode::TopPowerDown` (feature `h2-top-pd`) powers TOP/CPU/modem off
//! and restores their registers from RAM.
//! C6 MSPI follows SOC_ROOT_CLK (ESP-IDF's C6 `hal/clk_tree_ll.h`); the HAL
//! switches that root to XTAL before disabling BBPLL. Keeping XTAL and flash
//! power enabled is essential to this configuration's flash-resident code.
//!
//! The caller owns radio/peripheral quiescence and must apply the returned
//! SYSTIMER correction and rearm overdue alarms before releasing its critical
//! section. Neither the requested duration nor nominal RTC frequency measures
//! elapsed sleep. The vendored HAL fixes the high-word shift in
//! `SystemTimer::set_unit_value`; upstream 1.0.0 must not be used for correction.
//!
//! Register semantics and Q19 calibration follow ESP-IDF v5.5.1:
//! `components/hal/lp_timer_hal.c`, the C6/H2 `hal/lp_timer_ll.h`,
//! `components/esp_hw_support/port/esp32c6/rtc_time.c`, and `hal/pmu_ll.h`.

/// Product admission margin for the pinned HAL's post-alarm clock calibration
/// and PMU preparation. This is a policy floor, not a silicon timing guarantee.
pub const MIN_SLEEP_US: u64 = 20_000;

#[cfg(any(test, target_os = "none"))]
use accounting::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepError {
    UnsupportedClock,
    UnsupportedConfiguration,
    InvalidCalibration(CalibrationFault),
    InvalidDuration,
    InvalidCounter,
}

/// Which RTC calibration step failed (mirrors the HAL's `CalibrationFailure`;
/// kept local so this module stays host-testable).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationFault {
    /// The RTC slow-clock period is zero.
    SlowPeriodZero,
    /// The RC_FAST calibration timed out.
    FastTimeout,
    /// The RC_FAST calibration result is implausible.
    FastOutOfRange,
}

/// Multiple sources can assert simultaneously; do not discard either bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeCause {
    Timer,
    Gpio,
    TimerAndGpio,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepStatus {
    Woke(WakeCause),
    Rejected { cause: u32 },
    Unexpected { cause: u32 },
}

#[derive(Debug, PartialEq, Eq)]
#[must_use = "Account for stopped time and handle rejection before resuming the executor"]
pub struct SleepReport {
    /// Actual calibrated LP-timer interval, including sleep entry and exit.
    pub elapsed_us: u64,
    /// Add this once to SYSTIMER unit 0, then service/rearm overdue alarms.
    /// Time already counted by SYSTIMER has been subtracted.
    pub missing_systimer_ticks: u64,
    pub status: SleepStatus,
    /// ESP32-H2 TOP/CPU/modem power-down happened (registers restored by
    /// `esp32h2-sleep`); always `false` in the retained mode and on C6.
    pub top_powered_down: bool,
}

#[cfg(any(test, target_os = "none"))]
mod accounting {
    use super::*;

    pub(super) const RTC_MASK: u64 = (1 << 48) - 1;
    pub(super) const SYSTIMER_MASK: u64 = (1 << 52) - 1;
    const CAL_SCALE: u128 = 1 << 19;
    pub(super) const TIMER_WAKE: u32 = 1 << 4;
    pub(super) const GPIO_WAKE: u32 = 1 << 2;
    /// LP IO (EXT1) pad wake, the H2 TOP power-down's GPIO wake.
    pub(super) const EXT1_WAKE: u32 = 1 << 1;

    #[derive(Clone, Copy)]
    pub(super) struct Calibration(u32);

    impl Calibration {
        pub(super) fn new(period: u32) -> Result<Self, SleepError> {
            if period == 0 {
                return Err(SleepError::InvalidCalibration(
                    CalibrationFault::SlowPeriodZero,
                ));
            }
            Ok(Self(period))
        }

        pub(super) fn duration_ticks(self, duration_us: u64) -> Result<u64, SleepError> {
            if duration_us < MIN_SLEEP_US {
                return Err(SleepError::InvalidDuration);
            }
            let ticks = (u128::from(duration_us) * CAL_SCALE).div_ceil(u128::from(self.0));
            // Keep modular deadline comparisons unambiguous, and require at least
            // two LP clock edges.
            if ticks < 2 || ticks > u128::from(RTC_MASK / 2) {
                return Err(SleepError::InvalidDuration);
            }
            Ok(ticks as u64)
        }

        pub(super) fn report(
            self,
            rtc_before: u64,
            rtc_after: u64,
            systimer_before: u64,
            systimer_after: u64,
            systimer_hz: u64,
            status: SleepStatus,
        ) -> Result<SleepReport, SleepError> {
            if rtc_before > RTC_MASK
                || rtc_after > RTC_MASK
                || systimer_before > SYSTIMER_MASK
                || systimer_after > SYSTIMER_MASK
                || systimer_hz == 0
            {
                return Err(SleepError::InvalidCounter);
            }
            let rtc_delta = rtc_after.wrapping_sub(rtc_before) & RTC_MASK;
            let systimer_delta = systimer_after.wrapping_sub(systimer_before) & SYSTIMER_MASK;
            if rtc_delta > RTC_MASK / 2 || systimer_delta > SYSTIMER_MASK / 2 {
                return Err(SleepError::InvalidCounter);
            }
            let elapsed_q19 = u128::from(rtc_delta) * u128::from(self.0);
            let elapsed_us = elapsed_q19 / CAL_SCALE;
            let elapsed_ticks = elapsed_q19
                .checked_mul(u128::from(systimer_hz))
                .ok_or(SleepError::InvalidCounter)?
                / (CAL_SCALE * 1_000_000);
            if elapsed_us > u128::from(u64::MAX) || elapsed_ticks > u128::from(SYSTIMER_MASK / 2) {
                return Err(SleepError::InvalidCounter);
            }
            // A rejected sleep never stopped the counter. RTC quantization must not
            // turn a rejected request into a spurious positive clock correction.
            let missing = if matches!(status, SleepStatus::Rejected { .. }) {
                0
            } else {
                (elapsed_ticks as u64).saturating_sub(systimer_delta)
            };
            Ok(SleepReport {
                elapsed_us: elapsed_us as u64,
                missing_systimer_ticks: missing,
                status,
                top_powered_down: false,
            })
        }
    }

    pub(super) fn sleep_status(
        rejected: bool,
        reject_cause: u32,
        wake_cause: u32,
        gpio: bool,
    ) -> SleepStatus {
        if rejected {
            return SleepStatus::Rejected {
                cause: reject_cause,
            };
        }
        let pad = if gpio { GPIO_WAKE | EXT1_WAKE } else { 0 };
        if wake_cause & !(TIMER_WAKE | pad) != 0 {
            return SleepStatus::Unexpected { cause: wake_cause };
        }
        match (wake_cause & TIMER_WAKE != 0, wake_cause & pad != 0) {
            (true, false) => SleepStatus::Woke(WakeCause::Timer),
            (false, true) => SleepStatus::Woke(WakeCause::Gpio),
            (true, true) => SleepStatus::Woke(WakeCause::TimerAndGpio),
            (false, false) => SleepStatus::Unexpected { cause: wake_cause },
        }
    }
}

#[cfg(all(target_os = "none", feature = "esp32c6"))]
pub use esp_hal::rtc_cntl::sleep::RetainedPowerDown;
/// ESP32-H2: the product's own RAM-resident sleep crate replaces the vendor
/// esp-hal retained-sleep backport (`esp32h2-sleep/README.md`).
#[cfg(all(target_os = "none", feature = "esp32h2"))]
pub use esp32h2_sleep::{Ext1Wake, PowerDown as RetainedPowerDown, SleepMode, TOP_PD_SUPPORTED};
#[cfg(target_os = "none")]
pub use hardware::LightSleep;

#[cfg(target_os = "none")]
mod hardware {
    use super::*;
    use esp_hal::{
        peripherals::{LP_AON, LP_TIMER, PMU},
        rtc_cntl::Rtc,
    };

    /// Exclusive ownership of the sleep controller and its always-on timer.
    pub struct LightSleep<'d> {
        _rtc: Rtc<'d>,
        _timer: LP_TIMER<'d>,
        _pmu: PMU<'d>,
        _aon: LP_AON<'d>,
        power_down: RetainedPowerDown,
        #[cfg(feature = "esp32h2")]
        chip: esp32h2_sleep::ChipInfo,
        #[cfg(feature = "esp32h2")]
        mode: SleepMode,
        #[cfg(feature = "esp32h2")]
        ext1: Option<Ext1Wake>,
    }

    impl<'d> LightSleep<'d> {
        pub fn new(rtc: Rtc<'d>, timer: LP_TIMER<'d>, pmu: PMU<'d>, aon: LP_AON<'d>) -> Self {
            Self {
                _rtc: rtc,
                _timer: timer,
                _pmu: pmu,
                _aon: aon,
                power_down: RetainedPowerDown::NONE,
                #[cfg(feature = "esp32h2")]
                chip: chip_info(),
                #[cfg(feature = "esp32h2")]
                mode: SleepMode::Retained,
                #[cfg(feature = "esp32h2")]
                ext1: None,
            }
        }

        /// Also stop the selected oscillators during each sleep (H2 only; the
        /// C6 HAL rejects anything but `NONE` on the first sleep).
        ///
        /// Stopping XTAL stops SYSTIMER: [`SleepReport::missing_systimer_ticks`]
        /// then carries the whole interval, which the caller already applies.
        pub fn with_power_down(mut self, power_down: RetainedPowerDown) -> Self {
            self.power_down = power_down;
            self
        }

        /// ESP32-H2: select how much of the digital system each sleep powers
        /// down. [`SleepMode::TopPowerDown`] needs the `h2-top-pd` feature
        /// (otherwise every sleep fails with `UnsupportedConfiguration`), the
        /// XTAL power-down, and [`Self::with_ext1_wake`] for pad wake: the
        /// digital GPIO wake lives in the powered-down TOP domain.
        #[cfg(feature = "esp32h2")]
        pub fn with_mode(mut self, mode: SleepMode) -> Self {
            self.mode = mode;
            self
        }

        /// ESP32-H2 TOP power-down: the LP IO pads (GPIO7..=14) that replace
        /// the digital GPIO wake when `sleep(.., gpio_wakeup = true)`.
        #[cfg(feature = "esp32h2")]
        pub fn with_ext1_wake(mut self, ext1: Ext1Wake) -> Self {
            self.ext1 = Some(ext1);
            self
        }

        /// Enter actual PMU light sleep, retaining digital state.
        ///
        /// `gpio_wakeup` enables the digital GPIO wake source (RTC EXT1 with
        /// the H2 TOP power-down). GPIO9 is supported by this source when its
        /// digital input, pull-up and low-level wake are configured by its
        /// owner. The report identifies pad wake, not an individual pin.
        /// Durations below [`MIN_SLEEP_US`] are rejected without entering sleep.
        ///
        /// # Safety
        ///
        /// The caller must stop the radio and all PLL-dependent activity, finish
        /// flash/DMA/UART/USB operations, and disable or budget watchdogs. No other
        /// RTC/PMU/LP-timer user may run. For GPIO wake, the caller must own the
        /// armed pins, retain their input/pull configuration during sleep, and
        /// restore interrupt configuration afterwards (`Input::wakeup_enable`
        /// unlistens the pin). An asserted wake level can reject sleep.
        ///
        /// Hold `cs` across the idle-policy check, this call, clock correction,
        /// and alarm servicing. Apply `missing_systimer_ticks` exactly once,
        /// without moving time backwards. The caller must explicitly handle
        /// `Rejected`/`Unexpected`; a returned report is not proof of sleep.
        /// After [`SleepReport::top_powered_down`], peripherals outside the
        /// retained set (UART, TSENS analog trim, AES data) were reset.
        pub unsafe fn sleep(
            &mut self,
            _cs: critical_section::CriticalSection<'_>,
            duration_us: u64,
            gpio_wakeup: bool,
        ) -> Result<SleepReport, SleepError> {
            use esp_hal::timer::systimer::{SystemTimer, Unit};

            let calibration = Calibration::new(LP_AON::regs().store1().read().bits())?;
            calibration.duration_ticks(duration_us)?;
            let systimer_before = SystemTimer::unit_value(Unit::Unit0);
            let rtc_before = rtc_ticks();
            let outcome = unsafe { self.enter(duration_us, gpio_wakeup) };
            let rtc_after = rtc_ticks();
            let systimer_after = SystemTimer::unit_value(Unit::Unit0);
            let wake = outcome?;
            let mut report = calibration.report(
                rtc_before,
                rtc_after,
                systimer_before,
                systimer_after,
                SystemTimer::ticks_per_second(),
                sleep_status(wake.rejected, wake.reject_cause, wake.wake_cause, gpio_wakeup),
            )?;
            report.top_powered_down = wake.top_powered_down;
            Ok(report)
        }

        #[cfg(feature = "esp32h2")]
        unsafe fn enter(&mut self, duration_us: u64, gpio_wakeup: bool) -> Result<Wake, SleepError> {
            let top_pd = self.mode == SleepMode::TopPowerDown;
            let ext1 = match (gpio_wakeup && top_pd, self.ext1) {
                (false, _) => None,
                (true, Some(ext1)) => Some(ext1),
                (true, None) => return Err(SleepError::UnsupportedConfiguration),
            };
            let request = esp32h2_sleep::Request {
                duration_us,
                power_down: self.power_down,
                gpio_wake: gpio_wakeup && !top_pd,
                ext1,
                mode: self.mode,
            };
            // SAFETY: forwarded from `sleep`; this struct owns the PMU, LP
            // timer and LP_AON, and TIMG0 calibration runs only inside it.
            let wake = unsafe { esp32h2_sleep::light_sleep(&self.chip, &request) }
                .map_err(map_crate_error)?;
            Ok(Wake {
                rejected: wake.rejected,
                wake_cause: wake.wake_cause,
                reject_cause: wake.reject_cause,
                top_powered_down: wake.top_powered_down,
            })
        }

        #[cfg(feature = "esp32c6")]
        unsafe fn enter(&mut self, duration_us: u64, gpio_wakeup: bool) -> Result<Wake, SleepError> {
            use esp_hal::rtc_cntl::sleep::{GpioWakeupSource, TimerWakeupSource};

            let wake_timer = TimerWakeupSource::new(core::time::Duration::from_micros(duration_us));
            let power_down = self.power_down;
            let result = if gpio_wakeup {
                unsafe {
                    self._rtc.sleep_light_retained_power_down(
                        power_down,
                        &[&wake_timer, &GpioWakeupSource::new()],
                    )
                }
            } else {
                unsafe {
                    self._rtc
                        .sleep_light_retained_power_down(power_down, &[&wake_timer])
                }
            };
            let pmu = PMU::regs();
            let wake = Wake {
                rejected: pmu.int_raw().read().soc_sleep_reject().bit_is_set(),
                reject_cause: pmu.slp_wakeup_status1().read().bits(),
                wake_cause: pmu.slp_wakeup_status0().read().bits(),
                top_powered_down: false,
            };
            LP_TIMER::regs()
                .tar0_high()
                .modify(|_, w| w.main_timer_tar_en0().clear_bit());
            LP_TIMER::regs()
                .int_clr()
                .write(|w| w.soc_wakeup().clear_bit_by_one());
            result.map_err(map_hal_error)?;
            Ok(wake)
        }
    }

    /// PMU status of one sleep, chip-independent.
    struct Wake {
        rejected: bool,
        wake_cause: u32,
        reject_cause: u32,
        top_powered_down: bool,
    }

    fn rtc_ticks() -> u64 {
        let timer = LP_TIMER::regs();
        timer.update().write(|w| w.main_timer_update().set_bit());
        let low = timer.main_buf0_low().read().main_timer_buf0_low().bits();
        let high = timer.main_buf0_high().read().main_timer_buf0_high().bits();
        (u64::from(high) << 32) | u64::from(low)
    }

    /// PMU voltage eFuses and chip revision, read once outside any sleep
    /// sequence (eFuse access is flash-resident).
    #[cfg(feature = "esp32h2")]
    fn chip_info() -> esp32h2_sleep::ChipInfo {
        use esp_hal::efuse::{ACTIVE_HP_DBIAS, ACTIVE_LP_DBIAS, DSLP_DBIAS, Efuse};
        let (blk_major, blk_minor) = Efuse::block_version();
        esp32h2_sleep::ChipInfo {
            fuses: esp32h2_sleep::DbiasFuses {
                blk_major,
                blk_minor,
                active_hp: Efuse::read_field_le::<u8>(ACTIVE_HP_DBIAS),
                active_lp: Efuse::read_field_le::<u8>(ACTIVE_LP_DBIAS),
                deep_sleep: Efuse::read_field_le::<u8>(DSLP_DBIAS),
            },
            revision: Efuse::chip_revision(),
        }
    }

    #[cfg(feature = "esp32h2")]
    fn map_crate_error(error: esp32h2_sleep::SleepError) -> SleepError {
        use esp32h2_sleep::{CalibrationFailure, SleepError as E};
        match error {
            E::UnsupportedClock => SleepError::UnsupportedClock,
            E::InvalidCalibration(failure) => SleepError::InvalidCalibration(match failure {
                CalibrationFailure::SlowClockPeriodZero => CalibrationFault::SlowPeriodZero,
                CalibrationFailure::FastClockTimeout => CalibrationFault::FastTimeout,
                CalibrationFailure::FastClockOutOfRange => CalibrationFault::FastOutOfRange,
            }),
            E::InvalidDuration => SleepError::InvalidDuration,
            E::UnsupportedWakeSource | E::UnsupportedPowerConfiguration => {
                SleepError::UnsupportedConfiguration
            }
        }
    }

    #[cfg(feature = "esp32c6")]
    fn map_hal_error(error: esp_hal::rtc_cntl::sleep::RetainedSleepError) -> SleepError {
        use esp_hal::rtc_cntl::sleep::RetainedSleepError;
        match error {
            RetainedSleepError::UnsupportedClock => SleepError::UnsupportedClock,
            RetainedSleepError::InvalidCalibration(failure) => {
                use esp_hal::rtc_cntl::sleep::CalibrationFailure;
                SleepError::InvalidCalibration(match failure {
                    CalibrationFailure::SlowClockPeriodZero => CalibrationFault::SlowPeriodZero,
                    CalibrationFailure::FastClockTimeout => CalibrationFault::FastTimeout,
                    CalibrationFailure::FastClockOutOfRange => CalibrationFault::FastOutOfRange,
                })
            }
            RetainedSleepError::InvalidDuration => SleepError::InvalidDuration,
            RetainedSleepError::UnsupportedWakeSource
            | RetainedSleepError::UnsupportedPowerConfiguration => {
                SleepError::UnsupportedConfiguration
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMER: SleepStatus = SleepStatus::Woke(WakeCause::Timer);

    #[test]
    fn calibration_and_duration_fail_closed() {
        assert!(matches!(
            Calibration::new(0),
            Err(SleepError::InvalidCalibration(
                CalibrationFault::SlowPeriodZero
            ))
        ));
        let cal = Calibration::new(10 << 19).unwrap();
        assert_eq!(cal.duration_ticks(0), Err(SleepError::InvalidDuration));
        assert_eq!(cal.duration_ticks(10), Err(SleepError::InvalidDuration));
        assert_eq!(
            cal.duration_ticks(MIN_SLEEP_US - 1),
            Err(SleepError::InvalidDuration)
        );
        assert_eq!(cal.duration_ticks(MIN_SLEEP_US), Ok(2_000));
        assert_eq!(
            cal.duration_ticks(u64::MAX),
            Err(SleepError::InvalidDuration)
        );
        assert_eq!(cal.duration_ticks(20_001), Ok(2_001));
    }

    #[test]
    fn measured_time_not_requested_or_nominal_time() {
        let cal = Calibration::new(10 << 19).unwrap();
        let report = cal.report(200, 300, 900, 1_060, 16_000_000, TIMER).unwrap();
        assert_eq!(report.elapsed_us, 1_000);
        assert_eq!(report.missing_systimer_ticks, 15_840);
    }

    #[test]
    fn entry_and_exit_time_is_not_counted_twice() {
        let cal = Calibration::new(10 << 19).unwrap();
        let report = cal.report(0, 100, 0, 3_200, 16_000_000, TIMER).unwrap();
        assert_eq!(report.missing_systimer_ticks, 12_800);
        assert_eq!(3_200 + report.missing_systimer_ticks, 16_000);
    }

    #[test]
    fn running_counter_and_rtc_quantization_cannot_rewind_time() {
        let cal = Calibration::new(10 << 19).unwrap();
        for counter in [16_000, 16_100] {
            let report = cal.report(0, 100, 0, counter, 16_000_000, TIMER).unwrap();
            assert_eq!(report.missing_systimer_ticks, 0);
        }
    }

    #[test]
    fn counter_wraps_and_high_words_are_preserved() {
        let cal = Calibration::new(1 << 19).unwrap();
        let report = cal
            .report(RTC_MASK - 9, 10, SYSTIMER_MASK - 15, 16, 16_000_000, TIMER)
            .unwrap();
        assert_eq!(report.elapsed_us, 20);
        assert_eq!(report.missing_systimer_ticks, 288);
        let report = cal.report(0, 1 << 32, 0, 0, 16_000_000, TIMER).unwrap();
        assert_eq!(report.elapsed_us, 1 << 32);
        assert_eq!(report.missing_systimer_ticks, 1 << 36);
    }

    #[test]
    fn fractional_calibration_is_not_rounded_to_whole_microseconds_first() {
        let cal = Calibration::new(1 << 18).unwrap();
        let report = cal.report(0, 3, 0, 0, 16_000_000, TIMER).unwrap();
        assert_eq!(report.elapsed_us, 1);
        assert_eq!(report.missing_systimer_ticks, 24);
    }

    #[test]
    fn invalid_counter_intervals_and_arithmetic_overflow_are_errors() {
        let cal = Calibration::new(u32::MAX).unwrap();
        for (before, after, hz) in [
            (RTC_MASK + 1, 0, 16_000_000),
            (1, 0, 16_000_000),
            (0, RTC_MASK / 2, u64::MAX),
            (0, 1, 0),
        ] {
            assert_eq!(
                cal.report(before, after, 0, 0, hz, TIMER),
                Err(SleepError::InvalidCounter)
            );
        }
        assert_eq!(
            cal.report(0, 1, 1, 0, 16_000_000, TIMER),
            Err(SleepError::InvalidCounter)
        );
    }

    #[test]
    fn reject_is_not_a_wakeup_or_a_clock_correction() {
        let status = sleep_status(true, GPIO_WAKE, TIMER_WAKE, true);
        assert_eq!(status, SleepStatus::Rejected { cause: GPIO_WAKE });
        let report = Calibration::new(10 << 19)
            .unwrap()
            .report(0, 2, 0, 1, 16_000_000, status)
            .unwrap();
        assert_eq!(report.missing_systimer_ticks, 0);
    }

    #[test]
    fn wake_sources_are_exact_and_simultaneous_sources_survive() {
        assert_eq!(sleep_status(false, 0, TIMER_WAKE, false), TIMER);
        assert_eq!(
            sleep_status(false, 0, GPIO_WAKE, true),
            SleepStatus::Woke(WakeCause::Gpio)
        );
        assert_eq!(
            sleep_status(false, 0, TIMER_WAKE | GPIO_WAKE, true),
            SleepStatus::Woke(WakeCause::TimerAndGpio)
        );
        for cause in [0, GPIO_WAKE, EXT1_WAKE, TIMER_WAKE | 1, 1 << 31] {
            assert_eq!(
                sleep_status(false, 0, cause, false),
                SleepStatus::Unexpected { cause }
            );
        }
        for cause in [0, TIMER_WAKE | 1, GPIO_WAKE | (1 << 6)] {
            assert_eq!(
                sleep_status(false, 0, cause, true),
                SleepStatus::Unexpected { cause }
            );
        }
    }

    #[test]
    fn top_power_down_ext1_wake_is_a_pad_wake() {
        assert_eq!(
            sleep_status(false, 0, EXT1_WAKE, true),
            SleepStatus::Woke(WakeCause::Gpio)
        );
        assert_eq!(
            sleep_status(false, 0, EXT1_WAKE | TIMER_WAKE, true),
            SleepStatus::Woke(WakeCause::TimerAndGpio)
        );
        let report = Calibration::new(10 << 19)
            .unwrap()
            .report(0, 2, 0, 1, 16_000_000, TIMER)
            .unwrap();
        assert!(!report.top_powered_down);
    }
}
