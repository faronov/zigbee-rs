//! CC2340R52 watchdog (`CKMD.WDTCNT`), clocked by LFCLK at 32 768 Hz.
//!
//! Writing `WDTCNT` (after unlocking `WDTLOCK` with `0x1ACCE551`) starts or
//! restarts a countdown that resets the chip when it reaches zero; the
//! registers re-lock after the write. The sequence mirrors
//! `WatchdogLPF3_initHw()`/`WatchdogLPF3_clear()` (`WatchdogLPF3.c`, LPF3
//! SDK), including `WDTTEST.STALLEN` so a halted debugger does not trigger a
//! reset.
//!
//! **Once started the watchdog cannot be stopped** (TI's driver has no
//! close); it keeps counting in Active, Idle and Standby and only SHUTDOWN or
//! a reset stops it. A loader that arms it hands the obligation to service
//! it on to the application.
//!
//! Hardware status: sequence taken from the SDK; not yet run on silicon.
//! After reset `CKMD.LFCLKSEL` is `FAKE` (LF ticks derived from the HF
//! clock, see [`crate::power`]), which ticks while the CPU runs.

/// LFCLK ticks per second.
pub const TICK_HZ: u32 = 32_768;

/// Longest period the 32-bit counter allows (as `WatchdogLPF3`).
pub const MAX_PERIOD_MS: u32 = (u32::MAX / TICK_HZ) * 1000;

const CKMD_BASE: u32 = 0x4000_1000;
const WDTCNT: u32 = CKMD_BASE + 0x300;
const WDTTEST: u32 = CKMD_BASE + 0x304;
const WDTLOCK: u32 = CKMD_BASE + 0x308;
const UNLOCK: u32 = 0x1ACC_E551;
const STALLEN: u32 = 1;

/// A validated watchdog period.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Period {
    ticks: u32,
}

impl Period {
    /// `period_ms` milliseconds, or `None` outside `1..=MAX_PERIOD_MS`.
    pub const fn from_ms(period_ms: u32) -> Option<Self> {
        if period_ms == 0 || period_ms > MAX_PERIOD_MS {
            return None;
        }
        // As `WatchdogLPF3_convertMsToTicks`, without intermediate overflow.
        let ticks = (period_ms / 1000) * TICK_HZ + (period_ms % 1000) * TICK_HZ / 1000;
        Some(Self { ticks })
    }

    /// The countdown start value.
    pub const fn ticks(self) -> u32 {
        self.ticks
    }
}

/// Exclusive access to the watchdog.
#[derive(Debug)]
pub struct Wdt {
    _private: (),
}

impl Wdt {
    #[cfg(target_os = "none")]
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

/// A running watchdog: the chip resets unless [`Watchdog::kick`] runs at
/// least once per period. There is no way back to [`Wdt`].
#[derive(Debug)]
pub struct Watchdog {
    _token: Wdt,
    period: Period,
}

#[cfg(target_os = "none")]
impl Watchdog {
    /// Start the watchdog with `period` (stalled while a debugger halts the
    /// CPU).
    pub fn start(token: Wdt, period: Period) -> Self {
        cortex_m::interrupt::free(|_| {
            unlock();
            write(WDTTEST, STALLEN);
            write(WDTCNT, period.ticks());
            write(WDTLOCK, 0);
        });
        Self {
            _token: token,
            period,
        }
    }

    /// Restart the countdown from the full period.
    pub fn kick(&mut self) {
        cortex_m::interrupt::free(|_| {
            unlock();
            write(WDTCNT, self.period.ticks());
            write(WDTLOCK, 0);
        });
    }
}

#[cfg(target_os = "none")]
fn unlock() {
    write(WDTLOCK, UNLOCK);
    // SAFETY: CKMD.WDTLOCK is an always-readable status register.
    while unsafe { core::ptr::read_volatile(WDTLOCK as *const u32) } == 1 {}
}

#[cfg(target_os = "none")]
fn write(address: u32, value: u32) {
    // SAFETY: CKMD watchdog registers; the caller owns the watchdog token.
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn period_bounds_and_conversion() {
        assert_eq!(Period::from_ms(0), None);
        assert_eq!(Period::from_ms(MAX_PERIOD_MS + 1), None);
        assert_eq!(Period::from_ms(1000).unwrap().ticks(), 32_768);
        assert_eq!(Period::from_ms(1).unwrap().ticks(), 32);
        assert_eq!(Period::from_ms(1500).unwrap().ticks(), 49_152);
        assert_eq!(
            Period::from_ms(MAX_PERIOD_MS).unwrap().ticks(),
            (u32::MAX / TICK_HZ) * TICK_HZ
        );
    }
}
