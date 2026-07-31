//! Analog/clock bring-up, transcribed from
//! Telink's TLSR8258 startup sequence. Runs from `.ram_code`
//! (preloaded into RAM by the boot ROM) because it touches clock-enable
//! registers before the flash cache/XIP path is known-stable.
//!
//! # Fail-closed on analog-bus timeout
//!
//! [`analog_write`] polls the same bounded budget as
//! [`crate::mmio::analog_write`] ([`crate::mmio::ANALOG_POLL_ITERATIONS`],
//! reused directly rather than re-derived, so the two bounds cannot
//! silently drift apart) before giving up. An earlier revision of this
//! module treated a timeout as "clear the trigger and keep going" —
//! silently leaving whichever analog register that call was supposed to
//! program in an unknown state, then proceeding to bring up every
//! remaining clock-enable register on top of that unknown foundation.
//! [`init`] now instead stops at the first timeout and reports
//! [`ClockError::AnalogTimeout`] to its caller, which must not proceed
//! into normal application code on `Err` — see this module's four call
//! sites (`examples/telink-tlsr8258-{sensor,router}`,
//! `examples/telink-tlsr8258-radio` via `platform::clocks::init`, and
//! `tools/telink-tlsr8258-lab`'s `diag-pm` build) for the fail-closed halt
//! each one performs instead of calling into its own `app`/diagnostic
//! logic with a clock tree in an unknown state. None of this adds an
//! unbounded wait: the retry budget inside [`analog_write`] is unchanged,
//! and the fail-closed halt on `Err` is a deliberate terminal stop (the
//! same "there is no safe fallback action, so refuse to proceed" reasoning
//! [`crate::reset::reboot`]'s own doc already establishes for this crate),
//! not a wait for something to complete.

/// Early-boot analog-bus bring-up in [`init`] could not complete within
/// [`analog_write`]'s bounded retry budget.
///
/// This is a hard failure, not a "assume it worked and carry on" situation
/// — see this module's doc for why, and [`crate::mmio::AnalogError`] for
/// the equivalent, independently-bounded failure mode of the *runtime*
/// (non-early-boot) analog-bus path every other caller (gpio/adc/pm) uses.
/// The two error types are intentionally not unified: this one is specific
/// to the fixed, `.ram_code`-resident bring-up sequence in [`init`], which
/// (unlike [`crate::mmio::analog_write`]) never disables/restores
/// `reg_irq_en` around the bus access, because the CPU interrupt
/// controller has no meaningful "previous state" to restore this early in
/// boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockError {
    AnalogTimeout,
}

#[inline(never)]
#[cfg(target_arch = "tc32")]
#[unsafe(link_section = ".ram_code")]
fn analog_write(addr: u8, value: u8) -> Result<(), ClockError> {
    use super::mmio::{
        ANALOG_POLL_ITERATIONS, ANALOG_TRIGGER_WRITE, REG_ANALOG_ADDR, REG_ANALOG_DATA,
        REG_ANALOG_TRIGGER, r8, w8,
    };
    unsafe {
        w8(REG_ANALOG_ADDR, addr);
        w8(REG_ANALOG_DATA, value);
        w8(REG_ANALOG_TRIGGER, ANALOG_TRIGGER_WRITE);
        for _ in 0..ANALOG_POLL_ITERATIONS {
            if r8(REG_ANALOG_TRIGGER) & 1 == 0 {
                w8(REG_ANALOG_TRIGGER, 0);
                return Ok(());
            }
            core::arch::asm!("nop");
        }
        // Fail closed: clear the stuck trigger (leaving it set would wedge
        // every later analog-bus access, early-boot or runtime) but report
        // the timeout instead of silently continuing as if this write had
        // taken effect.
        w8(REG_ANALOG_TRIGGER, 0);
        Err(ClockError::AnalogTimeout)
    }
}

#[inline(never)]
#[cfg(target_arch = "tc32")]
#[unsafe(link_section = ".ram_code")]
pub fn init() -> Result<(), ClockError> {
    // Deliberately *not* a glob import: `super::mmio::analog_write` (also
    // `Result`-returning) would otherwise shadow this module's own
    // early-boot `analog_write` above within this function's scope (a
    // local `use` shadows an outer-module item of the same name), silently
    // swapping in the wrong implementation — including its IRQ-disable/
    // restore path, which has no business running this early in boot.
    use super::mmio::{REG_BASE, REG_CLK_EN0, REG_CLK_EN1, REG_CLK_EN2, w8};
    analog_write(0x82, 0x64)?;
    analog_write(0x34, 0x80)?;
    analog_write(0x06, 0x00)?;
    analog_write(0x0a, 0x44)?;
    analog_write(0x0b, 0x38)?;
    analog_write(0x05, 0x02)?;
    analog_write(0x8c, 0x02)?;
    analog_write(0x02, 0xa2)?;
    analog_write(0x27, 0x00)?;
    analog_write(0x28, 0x00)?;
    analog_write(0x29, 0x00)?;
    analog_write(0x2a, 0x00)?;
    analog_write(0x01, 0x4c)?;
    unsafe {
        for _ in 0..20_000u32 {
            core::arch::asm!("nop");
        }
        w8(REG_BASE + 0x066, 0x42);
        for _ in 0..5_000u32 {
            core::arch::asm!("nop");
        }
        w8(REG_CLK_EN0, 0xFF);
        w8(REG_CLK_EN1, 0xFF);
        w8(REG_CLK_EN2, 0xFF);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_error_is_a_plain_equatable_marker() {
        assert_eq!(ClockError::AnalogTimeout, ClockError::AnalogTimeout);
    }

    /// `clocks::analog_write` (early boot) and `mmio::analog_write`
    /// (runtime) must keep using the exact same trigger byte and
    /// addr/data/trigger register offsets, even though they are separate
    /// functions for separate reasons (see this module's doc) — otherwise
    /// this module's early-boot path would silently diverge from the one
    /// every other peripheral module trusts.
    #[test]
    fn early_boot_analog_write_reuses_mmios_canonical_trigger_constants() {
        assert_eq!(super::super::mmio::ANALOG_TRIGGER_WRITE, 0x60);
        assert_eq!(
            super::super::mmio::REG_ANALOG_DATA,
            super::super::mmio::REG_ANALOG_ADDR + 1
        );
        assert_eq!(
            super::super::mmio::REG_ANALOG_TRIGGER,
            super::super::mmio::REG_ANALOG_ADDR + 2
        );
    }
}
