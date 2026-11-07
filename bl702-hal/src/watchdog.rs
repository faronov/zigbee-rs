//! BL702 watchdog (the WDT channel of the TIMER block) in reset mode.
//!
//! The counter runs from the 32 kHz clock (`TCCR.cs_wdt = 1`) divided by 32
//! (`TCDR.wcdr = 31`), so one tick is 1/1024 s and the 16-bit match register
//! allows periods up to [`MAX_PERIOD_MS`]. When the counter reaches the match
//! value the chip resets (`WMER.WRIE = 1`); [`Watchdog::kick`] restarts the
//! count. Every WDT register write must be preceded by the access keys
//! (`WFAR = 0xBABA`, `WSAR = 0xEB10`), as `WDT_ENABLE_ACCESS()` does in the
//! BL702 SDK (`bl702_timer.c`).
//!
//! The WDT shares `TCCR`/`TCDR` with the timer channels; [`Watchdog::arm`]
//! saves both and [`Watchdog::disarm`] restores them, so a later owner of
//! the TIMER block (for example [`crate::timer::Monotonic`], which resets the
//! block anyway) starts from the state it would have seen without the
//! watchdog.
//!
//! Hardware status: register sequence taken from the SDK; not yet measured
//! on silicon. If the 32 kHz clock does not run, the watchdog never fires
//! (it fails open, never with a spurious reset).

use crate::clock::{Peripheral, enable_clock};
use crate::mmio::{read32, rmw, write32};
use crate::peripherals::Wdt;

const TIMER_BASE: u32 = 0x4000_a500;
const TCCR: u32 = TIMER_BASE;
const WMER: u32 = TIMER_BASE + 0x064;
const WMR: u32 = TIMER_BASE + 0x068;
const WSR: u32 = TIMER_BASE + 0x070;
const WCR: u32 = TIMER_BASE + 0x098;
const WFAR: u32 = TIMER_BASE + 0x09c;
const WSAR: u32 = TIMER_BASE + 0x0a0;
const TCDR: u32 = TIMER_BASE + 0x0bc;

const CS_WDT_MASK: u32 = 0x3 << 8;
const CS_WDT_32K: u32 = 1 << 8;
const WCDR_MASK: u32 = 0xff << 24;
const WCDR_DIV_32: u32 = 31 << 24;
const WMER_WE: u32 = 1 << 0;
const WMER_WRIE: u32 = 1 << 1;
const WSR_WTS: u32 = 1 << 0;

/// Watchdog ticks per second (32 768 Hz / 32).
pub const TICK_HZ: u32 = 1024;

/// Longest period the 16-bit match register allows at [`TICK_HZ`].
pub const MAX_PERIOD_MS: u32 = 63_999;

/// A validated watchdog period.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Period {
    ticks: u16,
}

impl Period {
    /// `period_ms` milliseconds, or `None` outside `1..=MAX_PERIOD_MS`.
    pub const fn from_ms(period_ms: u32) -> Option<Self> {
        if period_ms == 0 || period_ms > MAX_PERIOD_MS {
            return None;
        }
        // At most 63 999 * 1024 < 2^26: no overflow. At least one tick.
        let ticks = period_ms * TICK_HZ / 1000;
        Some(Self {
            ticks: if ticks == 0 { 1 } else { ticks as u16 },
        })
    }

    /// The match-register value.
    pub const fn ticks(self) -> u16 {
        self.ticks
    }
}

/// An armed watchdog; the chip resets unless [`Watchdog::kick`] runs at
/// least once per period.
#[derive(Debug)]
pub struct Watchdog {
    token: Wdt,
    saved_tccr: u32,
    saved_tcdr: u32,
}

impl Watchdog {
    /// Gate the TIMER clock on, select the 1024 Hz watchdog clock and arm
    /// the watchdog in reset mode with `period`, counting from zero.
    pub fn arm(token: Wdt, period: Period) -> Self {
        enable_clock(Peripheral::Timer);
        let saved_tccr = read32(TCCR);
        let saved_tcdr = read32(TCDR);
        protected_write(WMER, 0);
        rmw(TCCR, CS_WDT_MASK, CS_WDT_32K);
        rmw(TCDR, WCDR_MASK, WCDR_DIV_32);
        protected_write(WMR, u32::from(period.ticks()));
        protected_write(WCR, 1);
        protected_write(WMER, WMER_WRIE | WMER_WE);
        Self {
            token,
            saved_tccr,
            saved_tcdr,
        }
    }

    /// Restart the count.
    pub fn kick(&mut self) {
        protected_write(WCR, 1);
    }

    /// Stop the watchdog and restore the TIMER clock configuration it
    /// changed.
    pub fn disarm(self) -> Wdt {
        protected_write(WMER, 0);
        write32(TCCR, self.saved_tccr);
        write32(TCDR, self.saved_tcdr);
        self.token
    }
}

/// Whether the last reset was a watchdog reset; clears the flag.
pub fn take_reset_flag(_token: &mut Wdt) -> bool {
    enable_clock(Peripheral::Timer);
    let fired = read32(WSR) & WSR_WTS != 0;
    if fired {
        protected_write(WSR, 0);
    }
    fired
}

fn protected_write(address: u32, value: u32) {
    write32(WFAR, 0xbaba);
    write32(WSAR, 0xeb10);
    write32(address, value);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn period_bounds_and_rounding() {
        assert_eq!(Period::from_ms(0), None);
        assert_eq!(Period::from_ms(MAX_PERIOD_MS + 1), None);
        assert_eq!(Period::from_ms(1).unwrap().ticks(), 1);
        assert_eq!(Period::from_ms(1000).unwrap().ticks(), 1024);
        assert_eq!(Period::from_ms(16_000).unwrap().ticks(), 16_384);
        assert_eq!(Period::from_ms(MAX_PERIOD_MS).unwrap().ticks(), 65_534);
    }

    #[test]
    fn clock_fields_do_not_touch_the_timer_channels() {
        // Channel clock selects live in TCCR[7:0], channel dividers in
        // TCDR[23:0] (`timer_reg.h`).
        assert_eq!(CS_WDT_MASK & 0xff, 0);
        assert_eq!(WCDR_MASK & 0x00ff_ffff, 0);
        assert_eq!(CS_WDT_32K & !CS_WDT_MASK, 0);
        assert_eq!(WCDR_DIV_32 & !WCDR_MASK, 0);
    }
}
