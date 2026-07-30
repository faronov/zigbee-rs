//! BL702 free-running timer and cycle-count delays.

use core::hint::spin_loop;

use crate::clock::{Clocks, Peripheral, enable_and_reset};
use crate::mmio::{read32, rmw, write32};
use crate::peripherals::Timer0;

const TIMER_BASE: u32 = 0x4000_a500;
const TIMER_CLOCK_CONFIG: u32 = TIMER_BASE;
const TIMER_COMPARE_0: u32 = TIMER_BASE + 0x010;
const TIMER_COMPARE_1: u32 = TIMER_BASE + 0x014;
const TIMER_COMPARE_2: u32 = TIMER_BASE + 0x018;
const TIMER_COUNTER: u32 = TIMER_BASE + 0x02c;
const TIMER_INTERRUPT_ENABLE: u32 = TIMER_BASE + 0x044;
const TIMER_PRELOAD_CONTROL: u32 = TIMER_BASE + 0x05c;
const TIMER_INTERRUPT_CLEAR: u32 = TIMER_BASE + 0x078;
const TIMER_ENABLE: u32 = TIMER_BASE + 0x084;
const TIMER_COUNT_MODE: u32 = TIMER_BASE + 0x088;
const TIMER_CLOCK_DIVIDER: u32 = TIMER_BASE + 0x0bc;

/// Free-running one-megahertz TIMER channel used as a monotonic clock.
pub struct Monotonic {
    _token: Timer0,
    last_ticks: u32,
    epoch: u64,
}

impl Monotonic {
    /// Configure the same 1 MHz free-running timer used by the hardware-proven
    /// BL702 radio path.
    pub fn new_1mhz(token: Timer0, clocks: Clocks) -> Self {
        assert_eq!(clocks.fclk_hz(), 32_000_000);
        enable_and_reset(Peripheral::Timer);

        rmw(TIMER_ENABLE, 1 << 1, 0);
        rmw(TIMER_CLOCK_CONFIG, 0x3 << 2, 0);
        rmw(TIMER_CLOCK_DIVIDER, 0xff << 8, 31 << 8);
        rmw(TIMER_COUNT_MODE, 1 << 1, 1 << 1);
        write32(TIMER_PRELOAD_CONTROL, 0);
        write32(TIMER_COMPARE_0, u32::MAX - 2);
        write32(TIMER_COMPARE_1, u32::MAX - 2);
        write32(TIMER_COMPARE_2, u32::MAX - 2);
        write32(TIMER_INTERRUPT_ENABLE, 0);
        write32(TIMER_INTERRUPT_CLEAR, 0x7);
        rmw(TIMER_ENABLE, 1 << 1, 1 << 1);
        Self {
            _token: token,
            last_ticks: read32(TIMER_COUNTER),
            epoch: 0,
        }
    }

    /// Return active-time microseconds as an extended 64-bit monotonic value.
    ///
    /// The current polling executor calls this well within the raw counter's
    /// approximately 71-minute wrap period. Deep sleep remains unsupported;
    /// a future sleep path must use a wake-capable counter or overflow
    /// interrupt rather than relying on polling across a stopped clock.
    pub fn ticks(&mut self) -> u64 {
        let current = read32(TIMER_COUNTER);
        extend_ticks(&mut self.last_ticks, &mut self.epoch, current)
    }

    /// Read the raw wrapping one-megahertz hardware counter.
    pub fn raw_ticks(&self) -> u32 {
        read32(TIMER_COUNTER)
    }
}

fn extend_ticks(last: &mut u32, epoch: &mut u64, current: u32) -> u64 {
    if current < *last {
        *epoch += 1u64 << 32;
    }
    *last = current;
    *epoch | u64::from(current)
}

/// Busy-wait for microseconds using the 32 MHz RISC-V cycle counter.
///
/// This function intentionally retains a plain function-pointer ABI for the
/// existing pure-Rust radio calibration path.
pub fn delay_us(duration_us: u32) {
    let target_cycles = duration_us.saturating_mul(32);
    let started = cycle_count();
    while cycle_count().wrapping_sub(started) < target_cycles {
        spin_loop();
    }
}

#[inline(always)]
fn cycle_count() -> u32 {
    #[cfg(target_arch = "riscv32")]
    {
        let value: u32;
        // SAFETY: Reading `mcycle` has no memory-safety side effects.
        unsafe {
            core::arch::asm!("csrr {}, mcycle", out(reg) value);
        }
        value
    }

    #[cfg(not(target_arch = "riscv32"))]
    {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::extend_ticks;

    #[test]
    fn active_polling_extends_one_mhz_counter_rollover() {
        let mut last = u32::MAX - 10;
        let mut epoch = 0;
        assert_eq!(
            extend_ticks(&mut last, &mut epoch, u32::MAX - 1),
            u64::from(u32::MAX - 1)
        );
        assert_eq!(extend_ticks(&mut last, &mut epoch, 5), (1u64 << 32) | 5);
    }
}
