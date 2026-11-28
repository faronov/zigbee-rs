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
//!
//! With `isr-ack`, compare channel 2 is the radio's step [`alarm`]: the
//! `TIMER0` interrupt at a cycle stamp, which times the interrupt-driven
//! Imm-Ack (`radio::isr_ack`).

const CMU_EM01GRPACLKCTRL: u32 = 0x4000_8120;
const EM01GRPACLK_HFXO: u32 = 2;
const TIMER0: u32 = 0x4004_8000;
const TIMER_CFG: u32 = TIMER0 + 0x04;
const TIMER_CMD: u32 = TIMER0 + 0x0C;
const TIMER_STATUS: u32 = TIMER0 + 0x10;
const TIMER_TOP: u32 = TIMER0 + 0x1C;
const TIMER_CNT: u32 = TIMER0 + 0x24;
const TIMER_EN: u32 = TIMER0 + 0x30;
/// `CC[2].CFG` (`CC[n]` at `0x60 + 0x20 * n`, the same on xG21/xG22/xG24).
#[cfg(feature = "isr-ack")]
const TIMER_CC2_CFG: u32 = TIMER0 + 0xA0;
/// `CC_CFG.MODE` = `OUTPUTCOMPARE` (no pin action: `CC_CTRL` stays at reset).
#[cfg(feature = "isr-ack")]
const CC_CFG_OUTPUTCOMPARE: u32 = 2;
const TIMER_CMD_START: u32 = 1;
const TIMER_STATUS_RUNNING: u32 = 1;
const TIMER_STATUS_SYNCBUSY: u32 = 1 << 6;

/// HCLK after [`crate::clock::ClockControl::configure_hfxo`] (38.4 MHz, or
/// 39 MHz with `hfxo-39mhz`).
pub const HCLK_HZ: u32 = crate::clock::HFXO_HZ;
/// Time-base cycles per ten microseconds at [`HCLK_HZ`] (38.4 or 39
/// cycles/us exactly).
pub const CYCLES_PER_10_US: u32 = HCLK_HZ / 100_000;

/// Start the TIMER0 time base (idempotent). Requires the HFXO to be
/// running (see [`crate::clock::ClockControl::configure_hfxo`]).
pub fn enable_cycle_counter() {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        use core::ptr::{read_volatile as rd, write_volatile as wr};
        // Unclocked peripheral access bus-faults on Series 2: clock first.
        crate::clock::enable_bus_clock(crate::chip::BusClock::TIMER0);
        // The channel configuration is only writable with the timer
        // disabled, so a running timer without the alarm channel restarts.
        #[cfg(feature = "isr-ack")]
        let configured = rd(TIMER_CC2_CFG as *const u32) == CC_CFG_OUTPUTCOMPARE;
        #[cfg(not(feature = "isr-ack"))]
        let configured = true;
        if configured
            && rd(TIMER_EN as *const u32) & 1 != 0
            && rd(TIMER_STATUS as *const u32) & TIMER_STATUS_RUNNING != 0
        {
            return;
        }
        wr(CMU_EM01GRPACLKCTRL as *mut u32, EM01GRPACLK_HFXO);
        wr(TIMER_EN as *mut u32, 0);
        wr(TIMER_CFG as *mut u32, 0); // up-count, DIV1, HFXO-derived
        #[cfg(feature = "isr-ack")]
        wr(TIMER_CC2_CFG as *mut u32, CC_CFG_OUTPUTCOMPARE);
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

/// Compare channel 2 of the time base as a one-shot alarm: the `TIMER0`
/// interrupt (handled by `radio::isr_ack`) once the counter reaches a
/// cycle stamp. Shared with that handler only, which runs with the radio's
/// interrupt lines masked by the same lock.
#[cfg(feature = "isr-ack")]
pub(crate) mod alarm {
    use super::TIMER0;

    const TIMER_IF: u32 = TIMER0 + 0x14;
    const TIMER_IEN: u32 = TIMER0 + 0x18;
    const TIMER_CC2_OC: u32 = TIMER0 + 0xA8;
    const IF_CC2: u32 = 1 << 6;
    const SET: u32 = 0x1000;
    const CLR: u32 = 0x2000;

    /// Raise the interrupt at cycle `at`, or at once if `at` is (about to
    /// be) past: the channel matches `CNT == OC` exactly, so a stamp the
    /// counter passes before the write lands would otherwise wait a full
    /// counter wrap.
    pub(crate) fn set(at: u32) {
        #[cfg(all(target_arch = "arm", target_os = "none"))]
        unsafe {
            use core::ptr::write_volatile as wr;
            wr(TIMER_CC2_OC as *mut u32, at);
            wr((TIMER_IF + CLR) as *mut u32, IF_CC2);
            wr((TIMER_IEN + SET) as *mut u32, IF_CC2);
            if at.wrapping_sub(super::cycles()) as i32 <= LEAD as i32 {
                wr((TIMER_IF + SET) as *mut u32, IF_CC2);
            }
        }
        #[cfg(not(all(target_arch = "arm", target_os = "none")))]
        let _ = at;
    }

    /// Cycles ahead of a stamp below which [`set`] raises the interrupt at
    /// once (the handler then waits out the rest).
    pub(crate) const LEAD: u32 = super::CYCLES_PER_10_US / 5;

    /// Disarm and drop a pending match.
    pub(crate) fn clear() {
        #[cfg(all(target_arch = "arm", target_os = "none"))]
        unsafe {
            use core::ptr::write_volatile as wr;
            wr((TIMER_IEN + CLR) as *mut u32, IF_CC2);
            wr((TIMER_IF + CLR) as *mut u32, IF_CC2);
        }
    }

    /// In the handler: acknowledge the match.
    pub(crate) fn acknowledge() {
        #[cfg(all(target_arch = "arm", target_os = "none"))]
        unsafe {
            core::ptr::write_volatile((TIMER_IF + CLR) as *mut u32, IF_CC2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(feature = "hfxo-39mhz")]
    fn conversions_39mhz() {
        assert_eq!(CYCLES_PER_10_US, 390);
        assert_eq!(us_to_cycles(192), 7488);
        assert_eq!(cycles_to_us(7488), 192);
    }

    #[test]
    #[cfg(not(feature = "hfxo-39mhz"))]
    fn conversions() {
        assert_eq!(CYCLES_PER_10_US, 384);
        assert_eq!(us_to_cycles(192), 7373);
        assert_eq!(cycles_to_us(7373), 192);
        assert_eq!(us_to_cycles(u32::MAX), u32::MAX);
    }
}
