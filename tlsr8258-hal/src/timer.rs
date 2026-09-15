//! Timer0 as a free-running 24 MHz tick counter with a bounded,
//! deadline-based wait helper. Radio waits should go through
//! [`wait_until`] so "fixed time bounds; no infinite wait for radio status"
//! is enforced in one place instead of per call site.
//!
//! Also exposes Timer1 as a second, independent free-running SYS_CLK tick
//! counter ([`start_timer1`]/[`now_ticks1`]) for callers that need a
//! separate time base from Timer0 (e.g. so a future feature's periodic
//! polling cannot be confused with the radio's own bounded waits). Timer2
//! is not exposed here: [`crate::watchdog`] already owns it exclusively as
//! the hardware watchdog's tick source, sharing this module's same
//! `reg_tmr_ctrl` register (see that module's own `FLD_TMR2_EN` comment).
//!
//! # `reg_tmr_ctrl` bit layout (register evidence)
//!
//! `platform/chip_8258/register.h`'s `reg_tmr_ctrl` (32-bit, `REG_TMR_CTRL`)
//! low byte: `FLD_TMR0_EN = BIT(0)`, `FLD_TMR0_MODE = BIT_RNG(1,2)`,
//! `FLD_TMR1_EN = BIT(3)`, `FLD_TMR1_MODE = BIT_RNG(4,5)`, `FLD_TMR2_EN =
//! BIT(6)`. [`FLD_TMR0_EN`]/[`FLD_TMR0_MODE_MASK`]/[`FLD_TMR1_EN`]/
//! [`FLD_TMR1_MODE_MASK`] below transcribe these bit positions exactly
//! (`BIT_RNG(s, e)` is `((1 << (e-s+1)) - 1) << s`, per
//! `proj/common/bit.h`, so `BIT_RNG(1,2) = 0x06` and `BIT_RNG(4,5) =
//! 0x30`).
//!
//! An earlier revision of [`init`] cleared mask `0x31` (`FLD_TMR0_EN |
//! FLD_TMR1_MODE`) instead of `0x07` (`FLD_TMR0_EN | FLD_TMR0_MODE`) when
//! resetting Timer0's own mode field, and cleared `0x30` (`FLD_TMR1_MODE`
//! again) instead of `0x06` (`FLD_TMR0_MODE`) when re-enabling it — i.e. it
//! never actually cleared Timer0's own 2-bit mode field, and instead
//! clobbered Timer1's unrelated mode field on every call. This was inert
//! under the configuration this crate has actually shipped (Timer1 is
//! reset-default `0` and untouched by any other code before or after
//! `init` runs, and Timer0's own mode field is also reset-default `0`, so
//! clearing the wrong bits happened to leave every bit this function cares
//! about at the same value clearing the *right* bits would have), but it
//! was wrong per the vendor's own bit-field layout and would have silently
//! corrupted Timer1's mode the moment anything used Timer1 (e.g.
//! [`start_timer1`], added in this same change). Fixed to use the correctly
//! named `FLD_TMR0_*` constants; behavior is unchanged for every
//! configuration this crate has shipped so far.

/// CPU clock used for Timer0/Timer1 SYS_CLK mode on TLSR8258.
pub const TICKS_PER_MS: u32 = 24_000;
pub const TICKS_PER_US: u32 = 24;

/// Calculate the Timer0 value that preserves application monotonic time over
/// a full-SRAM SUSPEND interval measured by the separate 16 MHz PM timer.
///
/// Splitting the exact 3/2 ratio into quotient and remainder avoids an
/// overflowing intermediate and preserves Timer0's native wrapping semantics.
pub const fn rebased_ticks_after_suspend(timer0_before: u32, elapsed_system_ticks: u32) -> u32 {
    let half = elapsed_system_ticks / 2;
    let elapsed_timer0_ticks = half
        .wrapping_mul(TICKS_PER_MS / (crate::pm::SYSTEM_TIMER_TICKS_PER_MS / 2))
        .wrapping_add(elapsed_system_ticks & 1);
    timer0_before.wrapping_add(elapsed_timer0_ticks)
}

/// `reg_tmr_ctrl` bit 0 (`FLD_TMR0_EN`, `register.h`).
const FLD_TMR0_EN: u8 = 1 << 0;
/// `reg_tmr_ctrl` bits 1..2 (`FLD_TMR0_MODE`, `register.h`). `0` selects
/// `TIMER_MODE_SYSCLK`, the only mode this module programs.
const FLD_TMR0_MODE_MASK: u8 = 0b0000_0110;
/// `reg_tmr_ctrl` bit 3 (`FLD_TMR1_EN`, `register.h`).
const FLD_TMR1_EN: u8 = 1 << 3;
/// `reg_tmr_ctrl` bits 4..5 (`FLD_TMR1_MODE`, `register.h`).
const FLD_TMR1_MODE_MASK: u8 = 0b0011_0000;

/// `reg_tmr_sta` bit 0 (`FLD_TMR_STA_TMR0`, `register.h`). Write-1-to-clear
/// latched Timer0 IRQ status, distinct from `reg_irq_src`'s
/// `FLD_IRQ_TMR0_EN` bit — the vendor's own
/// `platform/services/b85m/irq_handler.c` (`irq_handler`, lines ~82-90)
/// clears *both* registers on every Timer0/Timer1 IRQ:
/// ```c
/// if ((src & FLD_IRQ_TMR0_EN)) {
///     reg_irq_src = FLD_IRQ_TMR0_EN;
///     reg_tmr_sta = FLD_TMR_STA_TMR0;
///     ...
/// }
/// ```
/// Clearing only `reg_irq_src` (as an earlier revision of this module did)
/// leaves `reg_tmr_sta`'s bit latched, which the vendor's own handler
/// treats as a distinct piece of state to acknowledge — on real hardware
/// this can make the source reassert on the next check. See
/// [`clear_timer0_irq_pending`]/[`clear_timer1_irq_pending`].
const FLD_TMR_STA_TMR0: u8 = 1 << 0;
/// `reg_tmr_sta` bit 1 (`FLD_TMR_STA_TMR1`, `register.h`). See
/// [`FLD_TMR_STA_TMR0`]'s doc.
const FLD_TMR_STA_TMR1: u8 = 1 << 1;

#[cfg(target_arch = "tc32")]
pub fn init() {
    use super::mmio::*;
    with_irqs_disabled(|| unsafe {
        let ctrl = r8(REG_TMR_CTRL);
        w8(REG_TMR_CTRL, ctrl & !(FLD_TMR0_EN | FLD_TMR0_MODE_MASK)); // stop timer0, SYS_CLK mode
        w32(REG_TMR0_TICK, 0);
        // Park compare at the latest possible tick. This avoids an
        // immediate match, but it is not a permanent "never" sentinel:
        // the 32-bit counter reaches this value once per wrap.
        w32(REG_TMR0_CAPT, 0xFFFF_FFFF);
        w8(REG_TMR_STA, FLD_TMR_STA_TMR0); // clear latched status
        crate::irq::clear_pending(crate::irq::IrqSource::Timer0);
        let ctrl = r8(REG_TMR_CTRL);
        w8(
            REG_TMR_CTRL,
            (ctrl & !FLD_TMR0_MODE_MASK) | FLD_TMR0_EN, // enable, SYS_CLK mode
        );
    });
}

#[cfg(target_arch = "tc32")]
pub fn now_ticks() -> u32 {
    unsafe { super::mmio::r32(super::mmio::REG_TMR0_TICK) }
}

/// Rebase Timer0 after full-SRAM SUSPEND and radio reinitialization.
///
/// Call this from the PM restore phase, after radio/MAC restoration (which
/// reinitializes Timer0) and before global interrupts are restored. Returning
/// the programmed value lets an on-device diagnostic verify the transaction.
#[cfg(target_arch = "tc32")]
#[inline(never)]
pub fn rebase_after_suspend(timer0_before: u32, elapsed_system_ticks: u32) -> u32 {
    let rebased = rebased_ticks_after_suspend(timer0_before, elapsed_system_ticks);
    unsafe {
        super::mmio::w32(super::mmio::REG_TMR0_TICK, rebased);
    }
    rebased
}

/// Whether Timer0 is currently enabled. HAL operations whose bounded waits
/// depend on [`now_ticks`] use this to fail instead of hanging if
/// [`init`] has not run.
#[cfg(target_arch = "tc32")]
pub fn is_timer0_running() -> bool {
    (unsafe { super::mmio::r8(super::mmio::REG_TMR_CTRL) } & FLD_TMR0_EN) != 0
}

/// Start Timer1 as a second, independent free-running SYS_CLK tick counter
/// (mirrors [`init`]'s Timer0 sequence, using [`FLD_TMR1_EN`]/
/// [`FLD_TMR1_MODE_MASK`] instead). Does not touch Timer0's or Timer2's
/// (the watchdog's) enable/mode bits in the shared `reg_tmr_ctrl` register.
/// Also clears any stale Timer1 IRQ status/source latched from a previous
/// run, mirroring [`init`]'s Timer0 clear (see [`FLD_TMR_STA_TMR0`]'s doc
/// for why both registers must be cleared).
#[cfg(target_arch = "tc32")]
pub fn start_timer1() {
    use super::mmio::*;
    with_irqs_disabled(|| unsafe {
        let ctrl = r8(REG_TMR_CTRL);
        w8(REG_TMR_CTRL, ctrl & !(FLD_TMR1_EN | FLD_TMR1_MODE_MASK));
        w32(REG_TMR1_TICK, 0);
        // Same finite wrap-boundary parking value as Timer0; callers must
        // program the intended deadline before unmasking Timer1's IRQ.
        w32(REG_TMR1_CAPT, 0xFFFF_FFFF);
        w8(REG_TMR_STA, FLD_TMR_STA_TMR1); // clear latched status
        crate::irq::clear_pending(crate::irq::IrqSource::Timer1);
        let ctrl = r8(REG_TMR_CTRL);
        w8(REG_TMR_CTRL, (ctrl & !FLD_TMR1_MODE_MASK) | FLD_TMR1_EN);
    });
}

/// Stop Timer1 (leaves Timer0/Timer2 untouched).
#[cfg(target_arch = "tc32")]
pub fn stop_timer1() {
    use super::mmio::*;
    with_irqs_disabled(|| unsafe {
        let ctrl = r8(REG_TMR_CTRL);
        w8(REG_TMR_CTRL, ctrl & !FLD_TMR1_EN);
    });
}

/// Timer1's free-running tick counter, independent of [`now_ticks`]'s
/// Timer0 counter.
#[cfg(target_arch = "tc32")]
pub fn now_ticks1() -> u32 {
    unsafe { super::mmio::r32(super::mmio::REG_TMR1_TICK) }
}

/// Enable or disable Timer0's IRQ source in the global `reg_irq_mask`
/// (`FLD_IRQ_TMR0_EN`, bit 0 — see [`crate::irq::IrqSource::Timer0`]).
/// This crate's shipped applications poll [`now_ticks`]/[`wait_until`]
/// instead of using this IRQ — provided for callers that need a genuine
/// periodic interrupt (e.g. a future scheduler tick) rather than
/// busy-polling. Program [`set_timer0_capture_ticks`] before enabling it.
/// Delegates the `reg_irq_mask` read-modify-write to
/// [`crate::irq::set_enabled`], which does not touch [`crate::gpio`]'s or
/// [`crate::radio`]'s own bits (each source is a disjoint bit, see
/// `crate::irq`'s module docs for the full `reg_irq_mask` layout).
#[cfg(target_arch = "tc32")]
pub fn set_timer0_irq_enable(enable: bool) {
    crate::irq::set_enabled(crate::irq::IrqSource::Timer0, enable);
}

/// Enable or disable Timer1's IRQ source in the global `reg_irq_mask`
/// (`FLD_IRQ_TMR1_EN`, bit 1). See [`set_timer0_irq_enable`]'s docs.
#[cfg(target_arch = "tc32")]
pub fn set_timer1_irq_enable(enable: bool) {
    crate::irq::set_enabled(crate::irq::IrqSource::Timer1, enable);
}

/// `true` if Timer0's IRQ source is currently latched in `reg_irq_src`.
/// Compare-and-wrap: Timer0's own capture register ([`REG_TMR0_CAPT`])
/// must be programmed by the caller before enabling the source. [`init`]
/// leaves it at `0xFFFF_FFFF`, which avoids an immediate match but is still
/// reached once per 32-bit counter wrap (roughly 179 seconds at 24 MHz);
/// it is not a permanent "never fires" sentinel.
#[cfg(target_arch = "tc32")]
pub fn timer0_irq_pending() -> bool {
    crate::irq::pending(crate::irq::IrqSource::Timer0)
}

/// `true` if Timer1's IRQ source is currently latched in `reg_irq_src`.
/// See [`timer0_irq_pending`]'s finite wrap-boundary parking note (applies
/// identically via [`REG_TMR1_CAPT`], initialized by [`start_timer1`]).
#[cfg(target_arch = "tc32")]
pub fn timer1_irq_pending() -> bool {
    crate::irq::pending(crate::irq::IrqSource::Timer1)
}

/// Acknowledge (write-1-to-clear) Timer0's latched IRQ source in
/// `reg_irq_src` **and** its latched status bit in `reg_tmr_sta`. The
/// vendor's own `irq_handler()` (`platform/services/b85m/irq_handler.c`)
/// clears both registers for every Timer0 IRQ — see [`FLD_TMR_STA_TMR0`]'s
/// doc for the exact evidence and why clearing only `reg_irq_src` is
/// insufficient (the status bit can otherwise leave the source able to
/// reassert). [`crate::irq::clear_pending`] handles the `reg_irq_src` half
/// (and, being a single write of exactly one bit, cannot disturb any other
/// source's pending bit in that register); `reg_tmr_sta` is Timer-specific
/// state outside `crate::irq`'s four-register scope, so it stays here.
#[cfg(target_arch = "tc32")]
pub fn clear_timer0_irq_pending() {
    crate::irq::clear_pending(crate::irq::IrqSource::Timer0);
    unsafe {
        super::mmio::w8(super::mmio::REG_TMR_STA, FLD_TMR_STA_TMR0);
    }
}

/// Acknowledge (write-1-to-clear) Timer1's latched IRQ source in
/// `reg_irq_src` **and** its latched status bit in `reg_tmr_sta`. See
/// [`clear_timer0_irq_pending`]'s doc.
#[cfg(target_arch = "tc32")]
pub fn clear_timer1_irq_pending() {
    crate::irq::clear_pending(crate::irq::IrqSource::Timer1);
    unsafe {
        super::mmio::w8(super::mmio::REG_TMR_STA, FLD_TMR_STA_TMR1);
    }
}

/// Program Timer0's compare/capture value (`reg_tmr0_capt`), the tick count
/// at which [`timer0_irq_pending`] latches. Mirrors the vendor's
/// `timer_set_cap_tick()` (`platform/chip_8258/timer.h`): `reg_tmr_capt(type)
/// = cap_tick`. Without this, [`set_timer0_irq_enable`] has nothing to
/// compare against ([`init`] parks the capture register at `0xFFFF_FFFF`,
/// which avoids an immediate match but is reached at the wrap boundary).
/// Program the intended capture before enabling the IRQ.
#[cfg(target_arch = "tc32")]
pub fn set_timer0_capture_ticks(cap_tick: u32) {
    unsafe { super::mmio::w32(super::mmio::REG_TMR0_CAPT, cap_tick) };
}

/// Program Timer1's compare/capture value (`reg_tmr1_capt`). See
/// [`set_timer0_capture_ticks`]'s doc.
#[cfg(target_arch = "tc32")]
pub fn set_timer1_capture_ticks(cap_tick: u32) {
    unsafe { super::mmio::w32(super::mmio::REG_TMR1_CAPT, cap_tick) };
}

/// Poll `condition` until it returns `true` or `timeout_ticks` elapse
/// (measured against the free-running Timer0 counter, which wraps at
/// `u32::MAX` — fine for the millisecond-scale windows used here). Returns
/// `true` if `condition` became true, `false` on timeout.
///
/// This is a *bounded busy-wait*. The always-on router may service RF RX from
/// an interrupt between condition polls; every call site must still pass a
/// finite, documented `timeout_ticks`.
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
    fn timer_ctrl_bit_fields_match_register_h() {
        // `BIT_RNG(s, e) = ((1 << (e-s+1)) - 1) << s` (`proj/common/bit.h`).
        assert_eq!(FLD_TMR0_EN, 0x01);
        assert_eq!(FLD_TMR0_MODE_MASK, 0x06); // BIT_RNG(1,2)
        assert_eq!(FLD_TMR1_EN, 0x08);
        assert_eq!(FLD_TMR1_MODE_MASK, 0x30); // BIT_RNG(4,5)
        // Timer0's and Timer1's fields must not overlap: this is exactly
        // the property the fixed `init`/`start_timer1` masks depend on to
        // avoid disturbing each other's timer.
        assert_eq!(
            (FLD_TMR0_EN | FLD_TMR0_MODE_MASK) & (FLD_TMR1_EN | FLD_TMR1_MODE_MASK),
            0
        );
    }

    #[test]
    fn suspend_rebase_converts_16mhz_elapsed_time_to_timer0_24mhz() {
        assert_eq!(rebased_ticks_after_suspend(10, 16_000), 24_010);
        assert_eq!(rebased_ticks_after_suspend(10, 4_000_000), 6_000_010);
        assert_eq!(rebased_ticks_after_suspend(10, 1), 11);
        assert_eq!(rebased_ticks_after_suspend(10, 2), 13);
    }

    #[test]
    fn suspend_rebase_preserves_timer0_wrapping_semantics() {
        assert_eq!(rebased_ticks_after_suspend(u32::MAX - 5, 16), 18);
    }

    #[test]
    fn timer_irq_mask_bits_match_register_h_and_are_disjoint() {
        // The `reg_irq_mask` bit constants used to live here as local
        // `FLD_IRQ_TMR0_EN`/`FLD_IRQ_TMR1_EN` consts; they now come from
        // `crate::irq::IrqSource`, the single place that owns this
        // register's bit table (see that module's own exhaustive
        // disjointness test for every source, not just these two).
        assert_eq!(crate::irq::IrqSource::Timer0.mask(), 0x01);
        assert_eq!(crate::irq::IrqSource::Timer1.mask(), 0x02);
        assert_eq!(
            crate::irq::IrqSource::Timer0.mask() & crate::irq::IrqSource::Timer1.mask(),
            0
        );
    }

    #[test]
    fn timer_sta_bits_match_register_h_and_are_disjoint() {
        // `reg_tmr_sta` (register.h): FLD_TMR_STA_TMR0=BIT(0),
        // FLD_TMR_STA_TMR1=BIT(1), FLD_TMR_STA_TMR2=BIT(2),
        // FLD_TMR_STA_WD=BIT(3) (the last already named in watchdog.rs as
        // `FLD_TMR_STA_WD`, verified consistent there).
        assert_eq!(FLD_TMR_STA_TMR0, 0x01);
        assert_eq!(FLD_TMR_STA_TMR1, 0x02);
        assert_eq!(FLD_TMR_STA_TMR0 & FLD_TMR_STA_TMR1, 0);
    }

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
