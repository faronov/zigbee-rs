//! Reset cause and rapid-reset guard storage in RTC fast (LP) memory.
//!
//! The shared [`sensor_sed_app::RetainedBoot`] logic is kept in four words of
//! `.rtc_fast.persistent`, which esp-hal does not initialise at boot. The
//! words survive software, watchdog and USB/JTAG resets and light sleep, but
//! not a power-on. They are validated by magic and check word, so power-on
//! contents never read as a cause.
//!
//! The LP_AON `store*` registers are deliberately not used: `store1` holds
//! the RTC slow-clock calibration and `store4`/`store9` carry ROM flags.

use sensor_sed_app::ResetKind;

/// Classify the raw SoC reset reason (identical codes on ESP32-C6 and
/// ESP32-H2, ESP-IDF `soc/reset_reasons.h`).
pub const fn classify(raw: u32) -> ResetKind {
    match raw {
        // RESET_REASON_CHIP_POWER_ON: retained memory is undefined.
        0x01 => ResetKind::PowerOn,
        // CORE_SW, CORE_DEEP_SLEEP, CPU0_SW, CORE_USB_UART, CORE_USB_JTAG,
        // CPU0_JTAG (C6): firmware, flasher or debugger initiated. A cause
        // is carried only if the firmware recorded one first.
        0x03 | 0x05 | 0x0C | 0x15 | 0x16 | 0x18 => ResetKind::Software,
        // Watchdogs, brownout, clock/power glitch, eFuse CRC, unknown.
        other => ResetKind::Unplanned(other),
    }
}

#[cfg(target_os = "none")]
mod hardware {
    use super::*;
    use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};
    use sensor_sed_app::{BootRecord, RETAINED_WORDS, RapidResetPolicy, ResetCause, RetainedBoot};

    #[esp_hal::ram(unstable(rtc_fast, persistent))]
    static mut RETAINED: [u32; RETAINED_WORDS] = [0; RETAINED_WORDS];

    fn load() -> RetainedBoot {
        let base = addr_of!(RETAINED).cast::<u32>();
        let mut words = [0; RETAINED_WORDS];
        for (index, word) in words.iter_mut().enumerate() {
            // SAFETY: in-bounds word reads of a static only this module
            // touches; volatile so a reset between stores is honoured.
            *word = unsafe { read_volatile(base.add(index)) };
        }
        RetainedBoot::from_words(words)
    }

    fn store(state: &RetainedBoot) {
        let base = addr_of_mut!(RETAINED).cast::<u32>();
        critical_section::with(|_| {
            for (index, word) in state.words.iter().enumerate() {
                // SAFETY: as in `load`; the critical section keeps a panic
                // handler from interleaving with a task-level update.
                unsafe { write_volatile(base.add(index), *word) };
            }
        });
    }

    /// The raw SoC reset reason of this boot, exactly as the ROM reports
    /// it.
    ///
    /// esp-hal's `SocResetReason` has no variant for every code (it lacks
    /// CPU0_JTAG, 0x18, which a debugger or flasher reset produces), and
    /// mapping an unknown code to 0 would misclassify a planned reset as an
    /// unplanned one and lose the code. The ROM call is read directly.
    pub fn raw_reset_reason() -> u32 {
        esp_hal::rom::rtc_get_reset_reason(esp_hal::system::Cpu::current() as u32)
    }

    /// Consume the previous boot's retained state. Call once, early.
    pub fn boot(policy: RapidResetPolicy) -> BootRecord {
        let mut state = load();
        let record = state.boot(classify(raw_reset_reason()), policy);
        store(&state);
        record
    }

    /// Record why this boot is about to reset (just before
    /// `software_reset`; also safe from the panic handler).
    pub fn record(cause: ResetCause) {
        let mut state = load();
        state.record(cause);
        store(&state);
    }

    /// This boot outlived the rapid-reset window.
    pub fn mark_stable() {
        let mut state = load();
        state.mark_stable();
        store(&state);
    }

    /// Resets since the last power-on (saturating).
    pub fn resets_since_power_on() -> u16 {
        load().resets_since_power_on()
    }
}

#[cfg(target_os = "none")]
pub use hardware::{boot, mark_stable, raw_reset_reason, record, resets_since_power_on};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_power_on_forgets_and_only_hardware_faults_are_unplanned() {
        assert_eq!(classify(0x01), ResetKind::PowerOn);
        for software in [0x03, 0x05, 0x0C, 0x15, 0x16, 0x18] {
            assert_eq!(classify(software), ResetKind::Software);
        }
        for unplanned in [0x00, 0x07, 0x09, 0x0F, 0x10, 0x12, 0x13, 0x14, 0x17] {
            assert_eq!(classify(unplanned), ResetKind::Unplanned(unplanned));
        }
    }
}
