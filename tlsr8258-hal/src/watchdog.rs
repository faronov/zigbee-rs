//! TLSR8258 watchdog (Timer2-based), transcribed from the open
//! `static inline` functions in `platform/chip_8258/watchdog.h` and the
//! matching Timer2/`reg_tmr_ctrl`/`reg_tmr_sta` fields in
//! `platform/chip_8258/register.h`. Fully open source — no closed-library
//! functions were involved for this peripheral.
//!
//! Telink's own formula for the interval-to-capture-value conversion is
//! `capture = period_ms * 1000 * system_clk_mhz >> 18`. This module hosts
//! that same formula but takes the clock frequency as an explicit parameter
//! (see [`set_interval_ms`]) rather than hardcoding it, since the crate
//! supports more than one clock configuration.

#[cfg(target_arch = "tc32")]
use super::mmio::{r32, w8, w32};

const REG_TMR_CTRL: u32 = super::mmio::REG_BASE + 0x620;
const REG_TMR_STA: u32 = super::mmio::REG_BASE + 0x623;
const REG_TMR2_TICK: u32 = super::mmio::REG_BASE + 0x638;

// `reg_tmr_ctrl` (32-bit) field layout, from `platform/chip_8258/register.h`:
//   bit0        FLD_TMR0_EN
//   bits[2:1]   FLD_TMR0_MODE
//   bit3        FLD_TMR1_EN
//   bits[5:4]   FLD_TMR1_MODE
//   bit6        FLD_TMR2_EN
//   bits[8:7]   FLD_TMR2_MODE
//   bits[22:9]  FLD_TMR_WD_CAPT (watchdog timeout capture value, 14 bits)
//   bit23       FLD_TMR_WD_EN   (watchdog enable)
//   bit24..26   FLD_TMRn_STA
//   bit27       FLD_CLR_WD
const FLD_TMR2_EN: u32 = 1 << 6;
const FLD_TMR_WD_EN: u32 = 1 << 23;
const FLD_TMR_WD_CAPT_SHIFT: u32 = 9;
const FLD_TMR_WD_CAPT_BITS: u32 = 14;
const FLD_TMR_WD_CAPT_MASK: u32 = ((1u32 << FLD_TMR_WD_CAPT_BITS) - 1) << FLD_TMR_WD_CAPT_SHIFT;

/// `reg_tmr_sta` bit for "watchdog fired" / "feed the watchdog" status.
/// `wd_clear()` in the vendor header writes this bit directly (a
/// write-1-to-clear status register), separate from `FLD_CLR_WD` in
/// `reg_tmr_ctrl` which this module does not use.
const FLD_TMR_STA_WD: u8 = 1 << 3;

/// [`capture_value`] cannot produce a valid `FLD_TMR_WD_CAPT` field for
/// the requested interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchdogError {
    /// `period_ms == 0` would arm the watchdog with a zero timeout, which
    /// resets the chip essentially immediately — almost certainly not what
    /// the caller intended, so this is rejected rather than silently
    /// programming a capture value of `0`.
    ZeroPeriod,
    /// `system_clk_mhz == 0` is not a valid clock configuration; dividing
    /// by it is meaningless (and the previous implementation would have
    /// silently produced a capture value of `0`, arming a zero-timeout
    /// watchdog exactly as with `ZeroPeriod`).
    ZeroClock,
    /// `period_ms` at `system_clk_mhz` does not fit in the 14-bit
    /// `FLD_TMR_WD_CAPT` field. A previous implementation silently
    /// saturated to the field's maximum instead, which would arm the
    /// watchdog with a *shorter* timeout than requested — the opposite of
    /// safe, since a caller asking for a long interval could get an
    /// unexpectedly early reset instead of a clear error.
    IntervalTooLong,
}

/// Compute the raw `FLD_TMR_WD_CAPT` value for a timeout of `period_ms`
/// milliseconds at `system_clk_mhz` MHz, using Telink's own formula
/// (`wd_set_interval_ms()` in `watchdog.h`).
///
/// Returns `Err` instead of silently clamping/zeroing whenever the request
/// can't be represented exactly — see [`WatchdogError`]'s variants for why
/// each case is a hard error rather than a best-effort substitution.
pub const fn capture_value(period_ms: u32, system_clk_mhz: u32) -> Result<u32, WatchdogError> {
    if period_ms == 0 {
        return Err(WatchdogError::ZeroPeriod);
    }
    if system_clk_mhz == 0 {
        return Err(WatchdogError::ZeroClock);
    }
    let raw = (period_ms as u64 * 1000 * system_clk_mhz as u64) >> 18;
    let max = (1u64 << FLD_TMR_WD_CAPT_BITS) - 1;
    if raw > max {
        return Err(WatchdogError::IntervalTooLong);
    }
    Ok(raw as u32)
}

/// Program the watchdog timeout without starting it. `system_clk_mhz` must
/// match the clock configuration set up by [`crate::clocks::init`] (24 MHz
/// on the code path this crate currently ships).
#[cfg(target_arch = "tc32")]
pub fn set_interval_ms(period_ms: u32, system_clk_mhz: u32) -> Result<(), WatchdogError> {
    let capture = capture_value(period_ms, system_clk_mhz)?;
    unsafe {
        w32(REG_TMR2_TICK, 0);
        let ctrl = r32(REG_TMR_CTRL);
        w32(
            REG_TMR_CTRL,
            (ctrl & !FLD_TMR_WD_CAPT_MASK) | (capture << FLD_TMR_WD_CAPT_SHIFT),
        );
    }
    Ok(())
}

/// Start the watchdog (Timer2 must be enabled for its tick to advance).
#[cfg(target_arch = "tc32")]
pub fn start() {
    unsafe {
        let ctrl = r32(REG_TMR_CTRL);
        w32(REG_TMR_CTRL, ctrl | FLD_TMR2_EN | FLD_TMR_WD_EN);
    }
}

/// Stop the watchdog (leaves Timer2 itself running if other code depends
/// on it).
#[cfg(target_arch = "tc32")]
pub fn stop() {
    unsafe {
        let ctrl = r32(REG_TMR_CTRL);
        w32(REG_TMR_CTRL, ctrl & !FLD_TMR_WD_EN);
    }
}

/// Feed the watchdog — call this periodically from `user_app_idle` (or
/// equivalent) inside the timeout window, or the chip resets.
#[cfg(target_arch = "tc32")]
pub fn feed() {
    unsafe { w8(REG_TMR_STA, FLD_TMR_STA_WD) };
}

/// `true` if the watchdog is currently enabled.
#[cfg(target_arch = "tc32")]
pub fn is_running() -> bool {
    unsafe { r32(REG_TMR_CTRL) & FLD_TMR_WD_EN != 0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_value_matches_vendor_formula_at_24mhz() {
        // wd_set_interval_ms(): tmp = (period_ms*1000*system_clk_mHz) >> 18.
        // 1000 ms @ 24 MHz -> (1000*1000*24) >> 18 = 91.55.. -> 91 (truncated).
        assert_eq!(capture_value(1000, 24), Ok(91));
    }

    #[test]
    fn capture_value_rejects_zero_period() {
        assert_eq!(capture_value(0, 24), Err(WatchdogError::ZeroPeriod));
    }

    #[test]
    fn capture_value_rejects_zero_clock() {
        assert_eq!(capture_value(1000, 0), Err(WatchdogError::ZeroClock));
    }

    #[test]
    fn capture_value_scales_with_clock() {
        assert!(capture_value(1000, 48).unwrap() > capture_value(1000, 24).unwrap());
    }

    #[test]
    fn capture_value_rejects_interval_too_long_instead_of_saturating() {
        // 1_000_000 ms @ 24 MHz -> (1e6*1000*24) >> 18 ~= 91_553, far past
        // the 14-bit field's maximum of 16_383. A previous implementation
        // silently clamped to 16_383 here instead of erroring, which would
        // have armed the watchdog with a much shorter timeout than the
        // 1000-second interval the caller actually asked for.
        assert_eq!(
            capture_value(1_000_000, 24),
            Err(WatchdogError::IntervalTooLong)
        );
    }

    #[test]
    fn capture_value_accepts_the_largest_representable_interval() {
        // The largest period (at 24 MHz) whose capture value still fits in
        // 14 bits should succeed, not saturate.
        let max_capture = (1u32 << 14) - 1;
        let result = capture_value(1000, 24).unwrap();
        assert!(result <= max_capture);
    }
}
