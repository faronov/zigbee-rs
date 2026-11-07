//! Admission and preparation failures.

/// Why a sleep request was refused. Every refusal happens before the chip
/// enters sleep; clocks and PMU state are restored before returning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepError {
    /// The current CPU/bus clock configuration cannot be restored.
    UnsupportedClock,
    /// An RTC calibration failed or produced an implausible period.
    InvalidCalibration(CalibrationFailure),
    /// The wake duration is zero or exceeds the unambiguous 47-bit interval.
    InvalidDuration,
    /// The wake-source combination is not supported by the requested mode.
    UnsupportedWakeSource,
    /// The power-down set is not supported by the requested mode (or the
    /// mode is not compiled in).
    UnsupportedPowerConfiguration,
}

/// Which calibration step rejected a sleep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationFailure {
    /// The boot-time RTC slow-clock period (`LP_AON.store1`) is zero.
    SlowClockPeriodZero,
    /// The TIMG0 RC_FAST calibration timed out (result 0).
    FastClockTimeout,
    /// The RC_FAST calibration completed with an implausible period.
    FastClockOutOfRange,
}
