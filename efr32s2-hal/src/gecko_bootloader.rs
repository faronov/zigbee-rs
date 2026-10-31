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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_cause_layout_matches_gsdk() {
        // reason at offset 0, signature at offset 2, little-endian.
        let word = reset_cause_word(RESET_REASON_BOOTLOAD);
        assert_eq!(word.to_le_bytes(), [0x02, 0x02, 0x0F, 0xF0]);
    }
}
