//! Embassy time driver for ESP32-C6/H2 using SYSTIMER peripheral.
//!
//! The SYSTIMER is a 52-bit counter running at 16 MHz on ESP32-C6.
//! We use it as a monotonic clock with ~1µs resolution.
//!
//! # Architecture
//! - SYSTIMER UNIT0 runs as a free counter at 16 MHz
//! - Alarm0 is used for embassy wake scheduling
//! - The alarm ISR signals the executor to poll futures

use embassy_time_driver::Driver;
use portable_atomic::{AtomicU64, Ordering};

// SYSTIMER register base (ESP32-C6)
const SYSTIMER_BASE: u32 = 0x6000_1000;

// Register offsets
const CONF: u32 = SYSTIMER_BASE + 0x00;
const UNIT0_OP: u32 = SYSTIMER_BASE + 0x04;
const UNIT0_VALUE_HI: u32 = SYSTIMER_BASE + 0x40;
const UNIT0_VALUE_LO: u32 = SYSTIMER_BASE + 0x44;
const TARGET0_HI: u32 = SYSTIMER_BASE + 0x18;
const TARGET0_LO: u32 = SYSTIMER_BASE + 0x1C;
const TARGET0_CONF: u32 = SYSTIMER_BASE + 0x20;
const COMP0_LOAD: u32 = SYSTIMER_BASE + 0x34;
const INT_ENA: u32 = SYSTIMER_BASE + 0x60;
const INT_CLR: u32 = SYSTIMER_BASE + 0x6C;

// SYSTIMER runs at 16 MHz on ESP32-C6
const SYSTIMER_HZ: u64 = 16_000_000;
// Embassy expects TICK_HZ = 1_000_000
const TICKS_PER_COUNT: u64 = SYSTIMER_HZ / 1_000_000; // 16

struct EspTimeDriver {
    alarm_at: AtomicU64,
}

fn reg_write(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) };
}

fn reg_read(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Read the 52-bit SYSTIMER UNIT0 counter value.
fn read_systimer_count() -> u64 {
    // Trigger a snapshot of UNIT0
    reg_write(UNIT0_OP, 1 << 30); // TIMER_UNIT0_UPDATE
    // Brief spin for value to be latched
    for _ in 0..10 { core::hint::spin_loop(); }
    let lo = reg_read(UNIT0_VALUE_LO) as u64;
    let hi = (reg_read(UNIT0_VALUE_HI) & 0xFFFFF) as u64; // 20 bits
    (hi << 32) | lo
}

/// Convert SYSTIMER counts to embassy ticks (µs).
fn counts_to_ticks(counts: u64) -> u64 {
    counts / TICKS_PER_COUNT
}

/// Convert embassy ticks to SYSTIMER counts.
fn ticks_to_counts(ticks: u64) -> u64 {
    ticks * TICKS_PER_COUNT
}

/// Initialize the SYSTIMER for embassy use.
pub fn init() {
    // Enable SYSTIMER clock and UNIT0
    let conf = reg_read(CONF);
    reg_write(CONF, conf | (1 << 31) | (1 << 30)); // CLK_EN + UNIT0 work enable

    // Disable alarm0 target initially
    reg_write(TARGET0_CONF, 0); // period mode off
    // Clear pending interrupt
    reg_write(INT_CLR, 1 << 0);
    // Enable alarm0 interrupt
    reg_write(INT_ENA, reg_read(INT_ENA) | (1 << 0));
}

impl Driver for EspTimeDriver {
    fn now(&self) -> u64 {
        counts_to_ticks(read_systimer_count())
    }

    fn schedule_wake(&self, at: u64, _waker: &core::task::Waker) {
        self.alarm_at.store(at, Ordering::Release);

        let target_counts = ticks_to_counts(at);
        // Set alarm0 target value
        reg_write(TARGET0_HI, ((target_counts >> 32) & 0xFFFFF) as u32);
        reg_write(TARGET0_LO, (target_counts & 0xFFFFFFFF) as u32);
        // Load the comparator value and enable
        reg_write(COMP0_LOAD, 1);
        // Enable one-shot alarm
        reg_write(TARGET0_CONF, 1 << 31); // TIMER_TARGET0_TIMER_UNIT_SEL=0 (UNIT0)
        // Clear + enable interrupt
        reg_write(INT_CLR, 1 << 0);
        reg_write(INT_ENA, reg_read(INT_ENA) | (1 << 0));
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: EspTimeDriver = EspTimeDriver {
    alarm_at: AtomicU64::new(u64::MAX),
});
