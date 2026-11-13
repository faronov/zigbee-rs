//! Busy-wait delay for bit-banged timing.

use embedded_hal::delay::DelayNs;

/// CPU1 core clock of [`crate::rcc_config`].
pub const CPU_HZ: u32 = 64_000_000;

/// [`DelayNs`] by counting CPU cycles at [`CPU_HZ`]. Waits at least the
/// requested time; interrupts lengthen it.
#[derive(Debug, Clone, Copy, Default)]
pub struct CycleDelay;

impl DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        // 64 cycles per µs: ns * 64 / 1000, rounded up, without overflow.
        let cycles = (u64::from(ns) * u64::from(CPU_HZ / 1_000_000)).div_ceil(1_000);
        cortex_m::asm::delay(cycles.min(u64::from(u32::MAX)) as u32);
    }
}
