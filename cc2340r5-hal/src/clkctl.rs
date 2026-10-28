//! `CLKCTL` peripheral clock gating (`Power_setDependency()` /
//! `Power_releaseDependency()` in TI's `PowerCC23X0.c`).
//!
//! Peripheral drivers in this HAL gate their clock on only for the duration
//! of an operation, so a battery device never leaves I2C/ADC/LGPT clocked
//! across standby.

/// `CLKCTL_BASE` (`hw_memmap.h`).
pub const CLKCTL_BASE: u32 = 0x4002_0000;
/// `CLKCTL_O_CLKCFG0`: current clock-enable status.
pub const CLKCTL_CLKCFG0: u32 = CLKCTL_BASE + 0x0C;
/// `CLKCTL_O_CLKENSET0`: write-one-to-enable.
pub const CLKCTL_CLKENSET0: u32 = CLKCTL_BASE + 0x14;
/// `CLKCTL_O_CLKENCLR0`: write-one-to-disable.
pub const CLKCTL_CLKENCLR0: u32 = CLKCTL_BASE + 0x20;

/// Gated peripheral clocks used by this HAL (`CLKCTL_CLKENSET0_*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum PeripheralClock {
    I2c0 = 0x0000_0040,
    Adc0 = 0x0000_4000,
    Lgpt1 = 0x1000_0000,
}

impl PeripheralClock {
    pub const fn mask(self) -> u32 {
        self as u32
    }
}

/// Enable a peripheral clock and wait until `CLKCFG0` reports it running.
#[cfg(target_os = "none")]
pub(crate) fn enable(clock: PeripheralClock) {
    // SAFETY: fixed CLKCTL registers; set/clear registers are atomic.
    unsafe {
        (CLKCTL_CLKENSET0 as *mut u32).write_volatile(clock.mask());
        while (CLKCTL_CLKCFG0 as *const u32).read_volatile() & clock.mask() == 0 {}
    }
}

/// Disable a peripheral clock.
#[cfg(target_os = "none")]
pub(crate) fn disable(clock: PeripheralClock) {
    // SAFETY: fixed CLKCTL register; write-one-to-clear.
    unsafe { (CLKCTL_CLKENCLR0 as *mut u32).write_volatile(clock.mask()) };
}

/// Whether a peripheral clock is currently enabled (diagnostics).
#[cfg(target_os = "none")]
pub fn is_enabled(clock: PeripheralClock) -> bool {
    // SAFETY: read-only status register.
    unsafe { (CLKCTL_CLKCFG0 as *const u32).read_volatile() & clock.mask() != 0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gate_bits_match_hw_clkctl() {
        assert_eq!(CLKCTL_CLKCFG0, 0x4002_000C);
        assert_eq!(CLKCTL_CLKENSET0, 0x4002_0014);
        assert_eq!(CLKCTL_CLKENCLR0, 0x4002_0020);
        assert_eq!(PeripheralClock::I2c0.mask(), 1 << 6);
        assert_eq!(PeripheralClock::Adc0.mask(), 1 << 14);
        assert_eq!(PeripheralClock::Lgpt1.mask(), 1 << 28);
    }
}
