//! WDOG0 in reset mode on the ULFRCO.
//!
//! Register map and sequences from GSDK 4.5.0 (`efr32mg2{1,2}_wdog.h`,
//! `efr32mg2{1,2}_cmu.h`, `em_wdog.c`); identical on xG21 and xG22 apart
//! from the xG22 bus-clock gate (`CMU_CLKEN0.WDOG0`):
//!
//! * `WDOG0` at `0x4A01_8000` (secure alias): `EN` `0x04`, `CFG` `0x08`,
//!   `CMD` `0x0C`, `STATUS` `0x14` (bit 31 `LOCK`), `SYNCBUSY` `0x24`
//!   (bit 0 `CMD`); set/clear aliases at `+0x1000`/`+0x2000`.
//! * `CMU_WDOG0CLKCTRL` at `CMU + 0x200`, `CLKSEL` `ULFRCO = 3` (reset
//!   value `LFRCO = 1`).
//! * `CFG` may only change while `EN = 0`; the timeout is
//!   `2^(3 + PERSEL) + 1` watchdog clock cycles. A `CMD.CLEAR` written while
//!   a previous clear is still synchronizing is a bus fault, so
//!   [`Watchdog::kick`] skips a kick while `SYNCBUSY.CMD` is set (as
//!   `WDOGn_Feed` does): the previous clear is still on its way.
//!
//! The ULFRCO (nominally 1 kHz) runs in every energy mode down to EM3, so
//! the longest period is about 262 s. Its frequency is uncalibrated;
//! [`Period::from_ms`] picks the shortest period that is still at least the
//! requested time at [`ULFRCO_MAX_HZ`], the assumed upper bound (10 % above
//! nominal). The watchdog is paused during a debug halt (`DEBUGRUN = 0`) and
//! in EM2/EM3 (`EM2RUN = EM3RUN = 0`): it bounds CPU-active work only.
//!
//! A locked watchdog (`STATUS.LOCK`, set by earlier code with
//! `LOCK = 0`) cannot be reconfigured until reset; [`Watchdog::arm`] then
//! refuses rather than waiting on a `SYNCBUSY` that may never clear.
//!
//! Hardware status: register sequence from the SDK; host-tested period
//! arithmetic only, not yet measured on silicon.

use crate::chip::{BusClock, base};

const WDOG0: u32 = 0x4A01_8000;
const WDOG_EN: u32 = WDOG0 + 0x04;
const WDOG_CFG: u32 = WDOG0 + 0x08;
const WDOG_CMD: u32 = WDOG0 + 0x0C;
const WDOG_STATUS: u32 = WDOG0 + 0x14;
const WDOG_SYNCBUSY: u32 = WDOG0 + 0x24;
const SET: u32 = 0x1000;
const CLR: u32 = 0x2000;

const WDOG_EN_EN: u32 = 1 << 0;
const WDOG_CMD_CLEAR: u32 = 1 << 0;
const WDOG_STATUS_LOCK: u32 = 1 << 31;
const WDOG_SYNCBUSY_CMD: u32 = 1 << 0;
const WDOG_CFG_PERSEL_SHIFT: u32 = 16;

const CMU_LOCK: u32 = base::CMU + 0x010;
const CMU_LOCK_UNLOCK: u32 = 0x93F7;
const CMU_WDOG0CLKCTRL: u32 = base::CMU + 0x200;
const CMU_WDOG0CLKCTRL_ULFRCO: u32 = 3;

/// Bound on every `SYNCBUSY` wait: the ULFRCO synchronizer needs a few
/// 1 kHz cycles (a few ms); this is tens of ms even at 80 MHz.
const SYNC_TIMEOUT_ITERATIONS: u32 = 1_000_000;

/// Assumed upper bound of the uncalibrated ULFRCO (nominal 1000 Hz + 10 %).
pub const ULFRCO_MAX_HZ: u32 = 1_100;

/// Largest `CFG.PERSEL`.
pub const PERSEL_MAX: u8 = 15;

/// Longest period [`Period::from_ms`] accepts: `2^18 + 1` cycles at
/// [`ULFRCO_MAX_HZ`].
pub const MAX_PERIOD_MS: u32 = Period { persel: PERSEL_MAX }.min_ms();

/// A watchdog timeout (`CFG.PERSEL`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Period {
    persel: u8,
}

impl Period {
    /// The shortest period that lasts at least `period_ms` even at
    /// [`ULFRCO_MAX_HZ`], or `None` for `0` or above [`MAX_PERIOD_MS`].
    pub const fn from_ms(period_ms: u32) -> Option<Self> {
        if period_ms == 0 {
            return None;
        }
        let mut persel = 0;
        while persel <= PERSEL_MAX {
            let period = Self { persel };
            if period.min_ms() >= period_ms {
                return Some(period);
            }
            persel += 1;
        }
        None
    }

    /// `CFG.PERSEL`.
    pub const fn persel(self) -> u8 {
        self.persel
    }

    /// Watchdog clock cycles to a reset: `2^(3 + PERSEL) + 1`.
    pub const fn cycles(self) -> u32 {
        (1 << (3 + self.persel as u32)) + 1
    }

    /// Shortest timeout, at [`ULFRCO_MAX_HZ`].
    pub const fn min_ms(self) -> u32 {
        self.cycles() * 1000 / ULFRCO_MAX_HZ
    }

    /// Nominal timeout at 1 kHz.
    pub const fn nominal_ms(self) -> u32 {
        self.cycles()
    }
}

/// Exclusive ownership of WDOG0.
pub struct WdogToken {
    _private: (),
}

impl WdogToken {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

/// Why [`Watchdog::arm`] refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchdogError {
    /// `STATUS.LOCK`: configured and locked by earlier code until reset.
    Locked,
    /// A `SYNCBUSY` wait did not finish.
    SyncTimeout,
}

/// An armed WDOG0; the chip resets unless [`Watchdog::kick`] runs at least
/// once per period of CPU-active time.
pub struct Watchdog {
    token: WdogToken,
}

impl Watchdog {
    /// Clock WDOG0 from the ULFRCO and start it in reset mode with
    /// `period`, counting from zero. A running, unlocked watchdog is
    /// stopped and reconfigured first.
    pub fn arm(token: WdogToken, period: Period) -> Result<Self, (WdogToken, WatchdogError)> {
        crate::clock::enable_bus_clock(BusClock::WDOG0);
        if hw::read(WDOG_STATUS) & WDOG_STATUS_LOCK != 0 {
            return Err((token, WatchdogError::Locked));
        }
        if let Err(error) = stop() {
            return Err((token, error));
        }
        hw::write(CMU_LOCK, CMU_LOCK_UNLOCK);
        hw::write(CMU_WDOG0CLKCTRL, CMU_WDOG0CLKCTRL_ULFRCO);
        // CLRSRC = software, EM2RUN = EM3RUN = 0, EM4BLOCK = 0,
        // DEBUGRUN = 0, no warning or window, reset enabled.
        hw::write(
            WDOG_CFG,
            u32::from(period.persel()) << WDOG_CFG_PERSEL_SHIFT,
        );
        hw::write(WDOG_EN + SET, WDOG_EN_EN);
        Ok(Self { token })
    }

    /// Restart the count (skipped while the previous clear synchronizes).
    pub fn kick(&mut self) {
        hw::critical(|| {
            if hw::read(WDOG_SYNCBUSY) & WDOG_SYNCBUSY_CMD == 0 {
                hw::write(WDOG_CMD, WDOG_CMD_CLEAR);
            }
        });
    }

    /// Stop the watchdog. On a synchronizer timeout the watchdog may still
    /// run; the token is returned either way.
    pub fn disarm(self) -> (WdogToken, Result<(), WatchdogError>) {
        let result = stop();
        (self.token, result)
    }
}

/// Clear `EN` once pending synchronization has finished.
fn stop() -> Result<(), WatchdogError> {
    if hw::read(WDOG_EN) & WDOG_EN_EN == 0 {
        return Ok(());
    }
    let mut remaining = SYNC_TIMEOUT_ITERATIONS;
    while hw::read(WDOG_SYNCBUSY) != 0 {
        if remaining == 0 {
            return Err(WatchdogError::SyncTimeout);
        }
        remaining -= 1;
        core::hint::spin_loop();
    }
    hw::write(WDOG_EN + CLR, WDOG_EN_EN);
    Ok(())
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod hw {
    #[inline]
    pub fn read(address: u32) -> u32 {
        // SAFETY: WDOG0/CMU register addresses from the device headers.
        unsafe { core::ptr::read_volatile(address as *const u32) }
    }

    #[inline]
    pub fn write(address: u32, value: u32) {
        // SAFETY: as `read`; the caller owns the WDOG token.
        unsafe { core::ptr::write_volatile(address as *mut u32, value) }
    }

    pub fn critical(f: impl FnOnce()) {
        cortex_m::interrupt::free(|_| f());
    }
}

/// Host builds: no registers (an idle, unlocked, disabled watchdog).
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
mod hw {
    pub fn read(_address: u32) -> u32 {
        0
    }

    pub fn write(_address: u32, _value: u32) {}

    pub fn critical(f: impl FnOnce()) {
        f();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn period_cycles_follow_persel() {
        assert_eq!(Period { persel: 0 }.cycles(), 9);
        assert_eq!(Period { persel: 7 }.cycles(), 1025);
        assert_eq!(Period { persel: 15 }.cycles(), 262_145);
        assert_eq!(MAX_PERIOD_MS, 238_313);
    }

    #[test]
    fn from_ms_picks_the_shortest_period_that_is_long_enough() {
        assert_eq!(Period::from_ms(0), None);
        assert_eq!(Period::from_ms(1).map(Period::persel), Some(0));
        // 8 s needs 8800 cycles at 1.1 kHz: 2^14 + 1 (PERSEL 11).
        let eight = Period::from_ms(8_000).unwrap();
        assert_eq!(eight.persel(), 11);
        assert!(eight.min_ms() >= 8_000);
        assert!(Period { persel: 10 }.min_ms() < 8_000);
        assert_eq!(eight.nominal_ms(), 16_385);
        assert_eq!(
            Period::from_ms(MAX_PERIOD_MS).map(Period::persel),
            Some(PERSEL_MAX)
        );
        assert_eq!(Period::from_ms(MAX_PERIOD_MS + 1), None);
        for ms in [1, 9, 10, 100, 999, 1_000, 30_000, 60_000, MAX_PERIOD_MS] {
            let period = Period::from_ms(ms).unwrap();
            assert!(period.min_ms() >= ms, "{ms}");
            if period.persel() > 0 {
                assert!(
                    Period {
                        persel: period.persel() - 1
                    }
                    .min_ms()
                        < ms,
                    "{ms}"
                );
            }
        }
    }

    #[test]
    fn host_arm_kick_disarm_round_trip() {
        let watchdog = Watchdog::arm(WdogToken::new(), Period::from_ms(1_000).unwrap())
            .unwrap_or_else(|_| panic!("host watchdog is unlocked"));
        let mut watchdog = watchdog;
        watchdog.kick();
        let (_token, result) = watchdog.disarm();
        assert_eq!(result, Ok(()));
    }
}
