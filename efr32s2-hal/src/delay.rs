//! Free-running 38.4 MHz time base on TIMER0: busy-wait delays and a
//! cycle timestamp.
//!
//! The Cortex-M33 DWT cycle counter is not usable on Series 2: with the
//! core in Secure state and no debugger attached it does not advance
//! (secure non-invasive debug is gated), so a firmware time base built on it
//! only works under a debugger. TIMER0 (32-bit on xG21 and xG22) is clocked
//! from EM01GRPACLK = HFXO with no prescaler, i.e. one tick per HCLK cycle.
//! It stops in EM2/EM3 like the DWT would: use it for the radio's
//! sub-millisecond timing (ACK turnaround, CSMA backoff, ACK wait), not as a
//! wall clock across deep sleep. This module owns TIMER0.

const CMU_EM01GRPACLKCTRL: u32 = 0x4000_8120;
const EM01GRPACLK_HFXO: u32 = 2;
const TIMER0: u32 = 0x4004_8000;
const TIMER_CFG: u32 = TIMER0 + 0x04;
const TIMER_CMD: u32 = TIMER0 + 0x0C;
const TIMER_STATUS: u32 = TIMER0 + 0x10;
const TIMER_TOP: u32 = TIMER0 + 0x1C;
const TIMER_CNT: u32 = TIMER0 + 0x24;
const TIMER_EN: u32 = TIMER0 + 0x30;
const TIMER_CMD_START: u32 = 1;
const TIMER_STATUS_RUNNING: u32 = 1;
const TIMER_STATUS_SYNCBUSY: u32 = 1 << 6;

/// HCLK after [`crate::clock::ClockControl::configure_hfxo`].
pub const HCLK_HZ: u32 = 38_400_000;
/// Time-base cycles per ten microseconds at [`HCLK_HZ`] (38.4 cycles/us exactly).
pub const CYCLES_PER_10_US: u32 = HCLK_HZ / 100_000;

/// Start the TIMER0 time base (idempotent). Requires the HFXO to be
/// running (see [`crate::clock::ClockControl::configure_hfxo`]).
pub fn enable_cycle_counter() {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        use core::ptr::{read_volatile as rd, write_volatile as wr};
        // Unclocked peripheral access bus-faults on Series 2: clock first.
        crate::clock::enable_bus_clock(crate::chip::BusClock::TIMER0);
        if rd(TIMER_EN as *const u32) & 1 != 0
            && rd(TIMER_STATUS as *const u32) & TIMER_STATUS_RUNNING != 0
        {
            return;
        }
        wr(CMU_EM01GRPACLKCTRL as *mut u32, EM01GRPACLK_HFXO);
        wr(TIMER_EN as *mut u32, 0);
        wr(TIMER_CFG as *mut u32, 0); // up-count, DIV1, HFXO-derived
        wr(TIMER_EN as *mut u32, 1);
        wr(TIMER_TOP as *mut u32, u32::MAX);
        while rd(TIMER_STATUS as *const u32) & TIMER_STATUS_SYNCBUSY != 0 {}
        wr(TIMER_CMD as *mut u32, TIMER_CMD_START);
        while rd(TIMER_STATUS as *const u32) & TIMER_STATUS_RUNNING == 0 {}
    }
}

/// Raw 32-bit cycle count (wraps every ~111.8 s at 38.4 MHz).
#[inline(always)]
pub fn cycles() -> u32 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::read_volatile(TIMER_CNT as *const u32)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        0
    }
}

/// Microseconds to time-base cycles, saturating.
#[inline]
pub const fn us_to_cycles(us: u32) -> u32 {
    let cycles = (us as u64 * CYCLES_PER_10_US as u64).div_ceil(10);
    if cycles > u32::MAX as u64 {
        u32::MAX
    } else {
        cycles as u32
    }
}

/// Time-base cycles to whole microseconds.
#[inline]
pub const fn cycles_to_us(cycles: u32) -> u32 {
    ((cycles as u64 * 10) / CYCLES_PER_10_US as u64) as u32
}

/// Busy-wait for at least `cycles_to_wait` cycles.
#[inline]
pub fn spin_cycles(cycles_to_wait: u32) {
    let start = cycles();
    while cycles().wrapping_sub(start) < cycles_to_wait {
        core::hint::spin_loop();
    }
}

/// Bounded deadline on the time base (at most ~55 s ahead).
#[derive(Debug, Clone, Copy)]
pub struct Deadline {
    start: u32,
    length: u32,
}

impl Deadline {
    #[inline]
    pub fn after_us(us: u32) -> Self {
        Self {
            start: cycles(),
            length: us_to_cycles(us).min(i32::MAX as u32),
        }
    }

    #[inline]
    pub fn expired(&self) -> bool {
        cycles().wrapping_sub(self.start) >= self.length
    }

    /// Cycles elapsed since the deadline was armed.
    #[inline]
    pub fn elapsed_cycles(&self) -> u32 {
        cycles().wrapping_sub(self.start)
    }
}

/// `embedded-hal` delay provider on the TIMER0 time base.
#[derive(Debug, Default, Clone, Copy)]
pub struct Delay;

impl Delay {
    /// Enable the counter and return a delay provider.
    pub fn new() -> Self {
        enable_cycle_counter();
        Self
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // ceil(ns * 38.4 / 1000) without overflow for any u32 ns.
        let cycles_to_wait = ((ns as u64 * CYCLES_PER_10_US as u64).div_ceil(10_000)) as u32;
        // Chunk below i32::MAX so wrapping_sub stays monotone.
        let mut remaining = cycles_to_wait;
        while remaining > 0 {
            let step = remaining.min(1 << 30);
            spin_cycles(step);
            remaining -= step;
        }
    }

    fn delay_us(&mut self, us: u32) {
        let mut remaining = us;
        while remaining > 0 {
            let step = remaining.min(10_000_000);
            spin_cycles(us_to_cycles(step));
            remaining -= step;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversions() {
        assert_eq!(CYCLES_PER_10_US, 384);
        assert_eq!(us_to_cycles(192), 7373);
        assert_eq!(cycles_to_us(7373), 192);
        assert_eq!(us_to_cycles(u32::MAX), u32::MAX);
    }
}
