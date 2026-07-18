//! PHY6222 power management and AON system sleep.
//!
//! The PHY6222 has three low-power modes:
//!
//! | Mode | CPU | SRAM | 32K RC | Radio | Wake |
//! |------|-----|------|--------|-------|------|
//! | WFI  | Halt| On   | On     | Off*  | Any IRQ |
//! | System Sleep | Off | Retention | On | Off | RTC, GPIO |
//! | System Off | Off | Off | On | Off | GPIO only |
//!
//! System sleep is a warm reboot — the ROM bootloader runs on wake and
//! jumps to the application entry point. State must be saved to retention
//! SRAM or flash before entering sleep.
//!
//! # SRAM banks
//! - Bank 0: 32 KB (0x1FFF0000–0x1FFF7FFF) — stack, globals, heap
//! - Bank 1: 16 KB (0x1FFF8000–0x1FFFBFFF)
//! - Bank 2: 16 KB (0x1FFFC000–0x1FFFFFFF)
//!
//! # RTC
//! The AON domain has a 24-bit counter clocked at ~32 kHz (RC oscillator).
//! A compare channel (RTCCC0) can wake the CPU from system sleep.

use crate::regs::*;
use crate::flash::FlashError;
use core::convert::Infallible;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepError {
    Flash(FlashError),
}

/// Approximate RC32K frequency (varies ±10% per chip/temperature).
pub const RC32K_HZ: u32 = 32_768;

/// Convert milliseconds to RC32K ticks.
pub const fn ms_to_rtc_ticks(ms: u32) -> u32 {
    (ms * RC32K_HZ) / 1000
}

/// Read the current AON RTC counter value (24-bit, wraps at 0xFFFFFF).
pub fn rtc_count() -> u32 {
    reg_read(AON_RTCCNT) & 0x00FF_FFFF
}

/// Configure the RTC compare channel 0 to fire after `ticks` RC32K cycles.
///
/// This sets up the wake source for system sleep.
pub fn config_rtc_wakeup(ticks: u32) {
    // Read current counter (must sample twice for stability)
    let mut cnt = reg_read(AON_RTCCNT);
    while cnt == reg_read(AON_RTCCNT) {} // wait for counter edge
    cnt = reg_read(AON_RTCCNT);

    // Set compare value
    reg_write(AON_RTCCC0, cnt.wrapping_add(ticks));

    // Enable: comparator0 event (bit20) + overflow IRQ (bit18) + comparator0 IRQ (bit15)
    let ctl = reg_read(AON_RTCCTL);
    reg_write(AON_RTCCTL, ctl | (1 << 20) | (1 << 18) | (1 << 15));
}

/// Set SRAM retention during system sleep.
///
/// `banks` is a bitmask of `RET_SRAM0 | RET_SRAM1 | RET_SRAM2`.
/// Only retained banks keep their contents during system sleep.
///
/// For a minimal Zigbee SED, retain bank 0 (32KB with stack + globals).
pub fn set_ram_retention(banks: u32) {
    reg_set_bits(AON_PMCTL2_0, 21, 17, banks & 0x1F);
}

/// Clear all SRAM retention (for system off mode).
pub fn clear_ram_retention() {
    reg_set_bits(AON_PMCTL2_0, 21, 17, 0);
}

/// Enter system sleep mode.
///
/// The CPU and all peripherals are powered off. Only the AON domain
/// (32 kHz RC, RTC, GPIO wake detect) remains active.
///
/// On wake, the chip does a warm reboot through the ROM bootloader.
/// The application must check `was_sleep_reset()` and restore state.
///
/// # Prerequisites
/// - Call `config_rtc_wakeup()` to set the wake timer
/// - Call `set_ram_retention()` to select which SRAM banks to keep
/// - Save any critical state to retention SRAM or flash
///
/// On success this function does not return. Before triggering sleep it sends
/// the external flash into deep power-down, so the complete transition runs
/// from SRAM with interrupts disabled. The ROM wake path is responsible for
/// making XIP flash available before it starts the application again.
#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
pub fn enter_system_sleep() -> Result<Infallible, SleepError> {
    cortex_m::interrupt::free(|_| {
        crate::flash::prepare_deep_power_down().map_err(SleepError::Flash)?;

        reg_set_bits(AON_PMCTL2_0, 6, 6, 0);
        reg_set_bits(AON_PMCTL0, 29, 27, 0x07);
        reg_write(AON_SLEEP_R0, 4);
        reg_write(AON_PWRSLP, SYSTEM_SLEEP_MAGIC);

        loop {
            unsafe {
                core::arch::asm!("wfi", options(nomem, nostack, preserves_flags));
            }
        }
    })
}

/// Check if the current boot was a wake from system sleep.
///
/// Returns `true` if `SLEEP_R[0]` indicates a warm wake.
/// Call this early in main() to decide whether to do a full init
/// or a fast restore from retention SRAM.
pub fn was_sleep_reset() -> bool {
    let r0 = reg_read(AON_SLEEP_R0);
    // RSTC_WARM_NDWC = 4, set by enter_system_sleep() before sleep
    r0 == 4
}

/// Clear the sleep reset flag after handling it.
pub fn clear_sleep_flag() {
    reg_write(AON_SLEEP_R0, 0);
}

/// Use SLEEP_R[2] and SLEEP_R[3] to persist a 64-bit timestamp across sleep.
/// These registers survive system sleep (AON domain).
pub fn save_sleep_timestamp(ticks: u64) {
    reg_write(AON_SLEEP_R2, ticks as u32);
    reg_write(AON_SLEEP_R3, (ticks >> 32) as u32);
}

/// Read the 64-bit timestamp saved before sleep.
pub fn load_sleep_timestamp() -> u64 {
    let lo = reg_read(AON_SLEEP_R2) as u64;
    let hi = reg_read(AON_SLEEP_R3) as u64;
    (hi << 32) | lo
}
