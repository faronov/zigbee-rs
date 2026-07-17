//! Mandatory platform services used by protocol layers.

use core::cell::Cell;

use crate::MacError;

/// Clock, delay, and entropy services required by the portable stack.
pub trait PlatformServices {
    /// Monotonic time in microseconds, wrapping at `u32::MAX`.
    fn monotonic_micros(&self) -> u32;

    /// Delay protocol progress for at least `duration_us` microseconds.
    async fn delay_micros(&mut self, duration_us: u32);

    /// Fill `output` with cryptographically secure random bytes.
    ///
    /// Backends without a hardware entropy source must return
    /// `MacError::Unsupported` rather than generating predictable key
    /// material.
    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError>;
}

/// Extends a wrapping 32-bit hardware tick counter to 64 bits.
///
/// Callers must sample at least once per raw-counter wrap. Absolute wrap count
/// does not need to survive reset; protocol timeouts only use differences.
pub struct WrappingTickExtender {
    last_ticks: Cell<u32>,
    wraps: Cell<u32>,
}

impl WrappingTickExtender {
    pub const fn new(initial_ticks: u32) -> Self {
        Self {
            last_ticks: Cell::new(initial_ticks),
            wraps: Cell::new(0),
        }
    }

    pub fn extend(&self, ticks: u32) -> u64 {
        let previous = self.last_ticks.replace(ticks);
        let mut wraps = self.wraps.get();
        if ticks < previous {
            wraps = wraps.wrapping_add(1);
            self.wraps.set(wraps);
        }
        (u64::from(wraps) << 32) | u64::from(ticks)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tick_extender_preserves_order_across_wrap() {
        let extender = WrappingTickExtender::new(u32::MAX - 5);
        assert_eq!(extender.extend(u32::MAX - 1), u64::from(u32::MAX - 1));
        assert_eq!(extender.extend(3), (1u64 << 32) | 3);
        assert_eq!(extender.extend(10), (1u64 << 32) | 10);
    }
}
