//! CC2340R5 shell of the zigbee-rs bootloader core ([`zrs_boot_core`]).
//!
//! A product's bootloader binary (linked at `0x0`, at most 16 KiB) provides
//! the slot [`Layout`], the trusted public key, the SRAM bounds and a status
//! [`Indicator`], places [`loader_info`] at a fixed address, and calls
//! `run`. Each boot then runs the core sequence ([`zrs_boot_core::run`]):
//!
//! 1. installs a pending, valid, non-downgrade image from the secondary slot
//!    ([`mcuboot_image::boot`], power-fail safe and resumable);
//! 2. validates the primary image (SHA-256 and ECDSA P-256 signature);
//! 3. checks the application vector table and jumps to it with `VTOR` and
//!    `MSP` set, leaving clocks, power and interrupts in the ROM boot state.
//!
//! With nothing bootable it shows a fault pattern and enters SHUTDOWN; a
//! configured button (or a power cycle / debugger reset) restarts the
//! decision. Recovery is reflashing over SWD.
//!
//! This crate keeps only what is CC2340R5-specific: the SRAM bounds, the
//! fault/SHUTDOWN policy and the Cortex-M hand-over in `target`.

#![cfg_attr(not(test), no_std)]

pub use zrs_boot_core::entry::{VectorError, check_vector_table};
pub use zrs_boot_core::indicator::{Indicator, indicate};
pub use zrs_boot_core::{
    BootError, BootEvent, Bootable, Layout, LoaderInfo, TrustedKey, Update, loader_info,
    mcuboot_image,
};

/// Fixed application header size (`imgtool --header-size`): the vector
/// table sits this far into the primary slot.
pub const HEADER_SIZE: u32 = 0x100;

/// Loader configuration supplied by the product.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub layout: Layout,
    pub key: TrustedKey,
    /// SRAM `[start, end)` for the initial stack pointer check.
    pub ram: (u32, u32),
}

#[cfg(target_os = "none")]
mod target;
#[cfg(target_os = "none")]
pub use target::run;

#[cfg(test)]
mod tests {
    use super::*;

    /// The vector-table and blink rules live in the core and are tested
    /// there; this checks the shell still exposes them unchanged.
    #[test]
    fn shell_reexports_core_checks() {
        let ram = (0x2000_0000, 0x2000_9000);
        assert_eq!(
            check_vector_table(0x2000_9000, 0x4201, 0x4100, 0x5100, ram),
            Ok(())
        );
        assert_eq!(
            check_vector_table(0x2000_9008, 0x4201, 0x4100, 0x5100, ram),
            Err(VectorError::StackPointer)
        );
        let info = LoaderInfo::decode(&loader_info(3, &[0x42; 64])).unwrap();
        assert_eq!(info.loader_version, 3);
    }
}
