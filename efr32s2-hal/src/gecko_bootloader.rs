//! Gecko Bootloader re-entry from the application (Series 2).
//!
//! The resident Gecko Bootloader (GSDK 4.5.0 `platform/bootloader/core/
//! btl_main.c` `enterBootloader`) stays in bootload mode after a reset only
//! if `EMU_RSTCAUSE.SYSREQ` is set and the `BootloaderResetCause_t` word at
//! `SRAM_BASE` (`btl_reset.c`, application side) holds a valid signature and
//! one of the bootload reasons. That is the mechanism EmberZNet uses for
//! `launchStandaloneBootloader`; it is the only software path into a
//! bootloader built without GPIO activation (for example the Home Assistant
//! Connect ZBT-1 UART/XMODEM bootloader).
//!
//! This is mechanism only. The product decides whether a resident Gecko
//! Bootloader exists (the MG22 WSTK product has none: there the reset simply
//! restarts the application) and what may trigger re-entry.

/// `BOOTLOADER_RESET_REASON_BOOTLOAD` (`btl_reset_info.h`).
pub const RESET_REASON_BOOTLOAD: u16 = 0x0202;
/// `BOOTLOADER_RESET_SIGNATURE_VALID` (`btl_reset_info.h`).
pub const RESET_SIGNATURE_VALID: u16 = 0xF00F;
/// `BOOTLOADER_RESET_SIGNATURE_INVALID` (`btl_reset_info.h`).
pub const RESET_SIGNATURE_INVALID: u16 = 0xC33C;
/// `SRAM_BASE`: the application-side `BootloaderResetCause_t` location.
pub const RESET_CAUSE_ADDRESS: u32 = 0x2000_0000;

/// `BootloaderResetCause_t { uint16_t reason; uint16_t signature; }` as one
/// little-endian word.
pub const fn reset_cause_word(reason: u16) -> u32 {
    reason as u32 | (RESET_SIGNATURE_VALID as u32) << 16
}

/// Ask the resident Gecko Bootloader to stay in bootload mode, then reset.
///
/// Overwrites the first SRAM word; nothing after this call runs.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn reboot_to_bootloader() -> ! {
    cortex_m::interrupt::disable();
    // SAFETY: the system is about to reset; the first SRAM word is the
    // bootloader's documented reset-cause mailbox and no code that could
    // observe the clobbered word runs afterwards.
    unsafe {
        core::ptr::write_volatile(
            RESET_CAUSE_ADDRESS as *mut u32,
            reset_cause_word(RESET_REASON_BOOTLOAD),
        );
    }
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset()
}

/// Reasons for which the resident bootloader stays in bootload mode after a
/// software reset instead of starting the application (GSDK 4.5.0
/// `btl_main.c` `enterBootloader`: `BOOTLOAD`, `FORCE`, `BADAPP`,
/// `UPGRADE`).
pub const fn reason_enters_bootloader(reason: u16) -> bool {
    matches!(reason, 0x0202 | 0x0205 | 0x0208 | 0x0209)
}

/// The reason of a `BootloaderResetCause_t` word, if its signature is
/// valid.
pub const fn reset_cause_reason(word: u32) -> Option<u16> {
    if (word >> 16) as u16 == RESET_SIGNATURE_VALID {
        Some(word as u16)
    } else {
        None
    }
}

/// Software reset that leaves `reason` in the `BootloaderResetCause_t`
/// word for the next boot of the application ([`take_reset_reason`]).
///
/// The resident bootloader starts the application for every reason except
/// the bootload ones ([`reason_enters_bootloader`]); without a bootloader
/// the SRAM word simply survives the `SYSRESETREQ`. Mechanism only: the
/// product owns the reason codes and when to reset.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn reset_with_reason(reason: u16) -> ! {
    cortex_m::interrupt::disable();
    // SAFETY: as `reboot_to_bootloader`: the system is about to reset and
    // the first SRAM word is the documented reset-cause mailbox.
    unsafe {
        core::ptr::write_volatile(RESET_CAUSE_ADDRESS as *mut u32, reset_cause_word(reason));
    }
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset()
}

/// Take the reason the previous software reset left in the
/// `BootloaderResetCause_t` word (by [`reset_with_reason`] or by the
/// bootloader itself, e.g. `GO`/`BADIMAGE`) and invalidate the word, so a
/// later reset without a reason is not mistaken for it.
///
/// The word is only meaningful after a `SYSRESETREQ` reset: SRAM is
/// undefined after power-on, so the caller checks the reset cause.
///
/// # Safety
///
/// The image must not place any RAM data in the first SRAM word (its
/// linker script starts RAM at `RESET_CAUSE_ADDRESS + 4`); otherwise
/// start-up has already overwritten the word and this clobbers live data.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub unsafe fn take_reset_reason() -> Option<u16> {
    let word = RESET_CAUSE_ADDRESS as *mut u32;
    // SAFETY: per the contract, the word is reserved for the reset cause.
    unsafe {
        let value = core::ptr::read_volatile(word);
        core::ptr::write_volatile(word, u32::from(RESET_SIGNATURE_INVALID) << 16);
        reset_cause_reason(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_cause_layout_matches_gsdk() {
        // reason at offset 0, signature at offset 2, little-endian.
        let word = reset_cause_word(RESET_REASON_BOOTLOAD);
        assert_eq!(word.to_le_bytes(), [0x02, 0x02, 0x0F, 0xF0]);
    }

    #[test]
    fn only_a_validly_signed_word_carries_a_reason() {
        assert_eq!(reset_cause_reason(reset_cause_word(0x0980)), Some(0x0980));
        assert_eq!(
            reset_cause_reason(u32::from(RESET_SIGNATURE_INVALID) << 16 | 0x0980),
            None
        );
        assert_eq!(reset_cause_reason(0), None);
        assert_eq!(reset_cause_reason(0xFFFF_FFFF), None);
    }

    #[test]
    fn bootload_reasons_match_btl_main() {
        // BOOTLOAD, FORCE, BADAPP, UPGRADE keep the bootloader running.
        for reason in [0x0202, 0x0205, 0x0208, 0x0209] {
            assert!(reason_enters_bootloader(reason));
        }
        // GO, BADIMAGE, FATAL, OTAVALID and application reasons do not.
        for reason in [0x0201, 0x0203, 0x0204, 0x0206, 0x0601, 0x0980] {
            assert!(!reason_enters_bootloader(reason));
        }
        assert!(reason_enters_bootloader(RESET_REASON_BOOTLOAD));
    }
}
