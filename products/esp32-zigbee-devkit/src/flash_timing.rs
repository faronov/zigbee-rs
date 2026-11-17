//! Duration of the product journals' flash erases and writes (GP Combo lab
//! image, `combo-light`).
//!
//! esp-storage runs every flash operation with interrupts masked (feature
//! `critical-section`), so an erase stalls the event loop and the radio
//! interrupts alike. The GP Combo light logs these numbers to find what
//! delayed GPDF handling by 68–173 ms during commissioning (2026-11-17).

use core::sync::atomic::{AtomicU32, Ordering};

static ERASES: AtomicU32 = AtomicU32::new(0);
static ERASE_MAX_US: AtomicU32 = AtomicU32::new(0);
static WRITES: AtomicU32 = AtomicU32::new(0);
static WRITE_MAX_US: AtomicU32 = AtomicU32::new(0);

/// Counts and longest durations since the last [`take`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FlashTiming {
    pub erases: u32,
    pub erase_max_us: u32,
    pub writes: u32,
    pub write_max_us: u32,
}

/// Read and reset the counters.
pub fn take() -> FlashTiming {
    FlashTiming {
        erases: ERASES.swap(0, Ordering::Relaxed),
        erase_max_us: ERASE_MAX_US.swap(0, Ordering::Relaxed),
        writes: WRITES.swap(0, Ordering::Relaxed),
        write_max_us: WRITE_MAX_US.swap(0, Ordering::Relaxed),
    }
}

/// Run and time a flash erase.
pub(crate) fn erase<T>(op: impl FnOnce() -> T) -> T {
    timed(&ERASES, &ERASE_MAX_US, op)
}

/// Run and time a flash write.
pub(crate) fn write<T>(op: impl FnOnce() -> T) -> T {
    timed(&WRITES, &WRITE_MAX_US, op)
}

fn timed<T>(count: &AtomicU32, max_us: &AtomicU32, op: impl FnOnce() -> T) -> T {
    let started = esp_hal::time::Instant::now();
    let result = op();
    let us = started.elapsed().as_micros() as u32;
    count.fetch_add(1, Ordering::Relaxed);
    max_us.fetch_max(us, Ordering::Relaxed);
    result
}
