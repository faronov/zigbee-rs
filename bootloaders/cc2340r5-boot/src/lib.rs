//! CC2340R5 shell of the zigbee-rs bootloader core ([`zrs_boot_core`]).
//!
//! A product's bootloader binary (linked at `0x0`, at most 16 KiB) provides
//! the [`BootConfig`] (slot [`Layout`], trusted public keys, trial boots,
//! install mode), the SRAM bounds and a status [`Indicator`], places
//! [`loader_block`] (or the single-key [`loader_info`]) at a fixed address,
//! and calls `run`. Each boot then runs the core sequence
//! ([`zrs_boot_core::run`]):
//!
//! 1. installs a pending, valid, non-downgrade image from the secondary slot
//!    ([`mcuboot_image::boot`], power-fail safe and resumable; overwrite, or
//!    swap-move with revert of an unconfirmed image under feature
//!    `swap-move`);
//! 2. validates the primary image (SHA-256 and an ECDSA P-256 signature by
//!    a trusted key);
//! 3. checks the application vector table and jumps to it with `VTOR` and
//!    `MSP` set, leaving clocks, power and interrupts in the ROM boot state.
//!
//! All flash access goes through [`BootFlash`], which refuses to erase or
//! program anything outside the two slots (and, in swap-move mode, the
//! status sectors), so the loader, `LoaderInfo` and the product's
//! persistence and CCFG pages cannot be touched by an install.
//!
//! With nothing bootable it shows a fault pattern and enters SHUTDOWN; a
//! configured button (or a power cycle / debugger reset) restarts the
//! decision. Recovery is reflashing over SWD.
//!
//! **Watchdog.** With [`Config::watchdog`] set, the CKMD watchdog is
//! started for the boot decision and kicked at every engine progress step
//! and fault blink. It cannot be stopped again (SHUTDOWN stops it), so it is
//! still running when the application starts, kicked once just before the
//! jump: the application must service it within the period from its reset
//! handler on. No CC2340 application does that yet, so products leave it
//! `None` until theirs does.
//!
//! **Signatures.** ECDSA P-256 only: the Ed25519 verifier (about 19 KB on
//! Cortex-M0+) does not fit the 16 KiB loader region, and feature `ed25519`
//! fails the build.
//!
//! This crate keeps only what is CC2340R5-specific: the SRAM bounds, the
//! fault/SHUTDOWN policy and the Cortex-M hand-over in `target`.

#![cfg_attr(not(test), no_std)]

#[cfg(feature = "ed25519")]
compile_error!(
    "cc2340r5-boot: Ed25519 is not supported on CC2340R5. The Ed25519 verifier (~19 KB on \
     Cortex-M0+) does not fit the 16 KiB loader region; CC2340 loaders verify ECDSA P-256 \
     signatures only. Build without feature `ed25519`."
);

pub use zrs_boot_core::entry::{VectorError, check_vector_table};
pub use zrs_boot_core::flash::{BootFlash, BootFlashError};
pub use zrs_boot_core::indicator::{Indicator, indicate};
pub use zrs_boot_core::{
    BootConfig, BootError, BootEvent, Bootable, Layout, LoaderInfo, Mode, Trial, TrustedKey,
    Update, loader_block, loader_info, mcuboot_image,
};

/// Fixed application header size (`imgtool --header-size`): the vector
/// table sits this far into the primary slot.
pub const HEADER_SIZE: u32 = 0x100;

/// Longest watchdog period (32-bit count at 32 768 Hz).
pub const WATCHDOG_MAX_MS: u32 = (u32::MAX / 32_768) * 1000;

/// A loader watchdog period in milliseconds, `1..=`[`WATCHDOG_MAX_MS`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WatchdogMs(u32);

impl WatchdogMs {
    pub const fn get(self) -> u32 {
        self.0
    }
}

/// The loader watchdog setting for `period_ms`: `0` leaves the watchdog
/// off. Evaluated in a `static`/`const`, an out-of-range period fails the
/// build. A non-zero period obliges the application to service the
/// watchdog (see the crate documentation).
pub const fn watchdog_ms(period_ms: u32) -> Option<WatchdogMs> {
    assert!(
        period_ms <= WATCHDOG_MAX_MS,
        "loader watchdog period exceeds the 32-bit LFCLK count"
    );
    if period_ms == 0 {
        None
    } else {
        Some(WatchdogMs(period_ms))
    }
}

/// Loader configuration supplied by the product.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Slots, trusted keys, trial boots and install mode.
    pub boot: BootConfig<'static>,
    /// SRAM `[start, end)` for the initial stack pointer check.
    pub ram: (u32, u32),
    /// Loader watchdog period, or `None` to leave the watchdog stopped.
    /// Started, it keeps running in the application.
    pub watchdog: Option<WatchdogMs>,
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

    #[test]
    fn watchdog_setting() {
        assert_eq!(watchdog_ms(0), None);
        assert_eq!(watchdog_ms(16_000).map(WatchdogMs::get), Some(16_000));
        assert_eq!(WATCHDOG_MAX_MS, 131_071_000);
        assert!(std::panic::catch_unwind(|| watchdog_ms(WATCHDOG_MAX_MS + 1)).is_err());
    }
}
