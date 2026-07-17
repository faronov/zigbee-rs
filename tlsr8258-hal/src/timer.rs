//! Timer0 as a free-running 24 MHz tick counter with a bounded,
//! deadline-based wait helper. Radio waits should go through
//! [`wait_until`] so "fixed time bounds; no infinite wait for radio status"
//! is enforced in one place instead of per call site.

/// CPU clock used for Timer0 SYS_CLK mode on TLSR8258.
pub const TICKS_PER_MS: u32 = 24_000;
pub const TICKS_PER_US: u32 = 24;

#[cfg(target_arch = "tc32")]
pub fn init() {
    use super::mmio::*;
    unsafe {
        let ctrl = r8(REG_TMR_CTRL);
        w8(REG_TMR_CTRL, ctrl & !0x31); // stop timer0, SYS_CLK mode
        w32(REG_TMR0_TICK, 0);
        w32(REG_TMR0_CAPT, 0xFFFF_FFFF); // park compare far in the future
        w8(REG_TMR_STA, 0x01); // clear latched status
        w32(REG_IRQ_SRC, 1 << 0);
        let ctrl = r8(REG_TMR_CTRL);
        w8(REG_TMR_CTRL, (ctrl & !0x30) | 0x01); // enable, SYS_CLK mode
    }
}

#[cfg(target_arch = "tc32")]
pub fn now_ticks() -> u32 {
    unsafe { super::mmio::r32(super::mmio::REG_TMR0_TICK) }
}

/// Poll `condition` until it returns `true` or `timeout_ticks` elapse
/// (measured against the free-running Timer0 counter, which wraps at
/// `u32::MAX` — fine for the millisecond-scale windows used here). Returns
/// `true` if `condition` became true, `false` on timeout.
///
/// This is a *bounded busy-wait*: IRQs are intentionally left disabled (see
/// `platform::vectors`), so there is nothing to block on except polling,
/// and every call site must pass a finite, documented `timeout_ticks`.
#[cfg(target_arch = "tc32")]
pub fn wait_until(timeout_ticks: u32, mut condition: impl FnMut() -> bool) -> bool {
    let start = now_ticks();
    loop {
        if condition() {
            return true;
        }
        if now_ticks().wrapping_sub(start) >= timeout_ticks {
            return condition();
        }
        unsafe { core::arch::asm!("nop") };
    }
}

/// Fixed-duration busy-wait (no condition to poll for) — used for hardware
/// settle delays. Implemented on top of [`wait_until`] with an
/// always-false condition so there is exactly one bounded-wait primitive in
/// the codebase.
#[cfg(target_arch = "tc32")]
pub fn sleep_ticks(ticks: u32) {
    wait_until(ticks, || false);
}

/// `ms` milliseconds expressed in Timer0 ticks, saturating rather than
/// overflowing for accidental large inputs.
pub const fn ms(ms: u32) -> u32 {
    ms.saturating_mul(TICKS_PER_MS)
}

/// `us` microseconds expressed in Timer0 ticks, saturating rather than
/// overflowing for accidental large inputs.
pub const fn us(us: u32) -> u32 {
    us.saturating_mul(TICKS_PER_US)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ms_conversion_is_exact_for_small_values() {
        assert_eq!(ms(1), 24_000);
        assert_eq!(ms(10), 240_000);
    }

    #[test]
    fn ms_conversion_saturates_instead_of_overflowing() {
        assert_eq!(ms(u32::MAX), u32::MAX);
    }

    #[test]
    fn us_conversion_is_exact_for_ack_turnaround() {
        assert_eq!(us(120), 2_880);
    }

    #[test]
    fn us_conversion_saturates_instead_of_overflowing() {
        assert_eq!(us(u32::MAX), u32::MAX);
    }
}
