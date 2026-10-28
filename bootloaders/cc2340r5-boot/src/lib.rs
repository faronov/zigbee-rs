//! Overwrite-only, MCUboot-image-compatible bootloader core for the
//! CC2340R5.
//!
//! A product's bootloader binary (linked at `0x0`, at most 16 KiB) provides
//! the slot [`Layout`], the trusted public key, the SRAM bounds and a status
//! [`Indicator`], places [`loader_info`] at a fixed address, and calls
//! `run`. Each boot then:
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

#![cfg_attr(not(test), no_std)]

pub use mcuboot_image::{
    self, BootError, BootEvent, Bootable, Layout, LoaderInfo, TrustedKey, Update,
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

/// Status output (typically LEDs) while the loader runs.
pub trait Indicator {
    /// An upgrade is being installed; called repeatedly with alternating
    /// `on` while copying.
    fn installing(&mut self, on: bool);
    /// Nothing can be booted.
    fn fault(&mut self, on: bool);
    /// Everything off before jumping to the application.
    fn off(&mut self);
}

/// Encode the record applications read to detect this loader. Place the
/// result with `#[link_section]` at the address the product's application
/// expects (by convention the last 32 bytes of the loader region).
pub const fn loader_info(loader_version: u32, key_xy: &[u8; 64]) -> [u8; 32] {
    LoaderInfo::new(0, loader_version, key_xy).encode()
}

/// Why a validated image was not started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VectorError {
    StackPointer,
    ResetVector,
}

/// Sanity-check an application vector table: the initial stack pointer must
/// be 8-byte aligned within `ram` (an empty full-descending stack may point
/// at its end), and the reset handler a Thumb address inside the image.
pub fn check_vector_table(
    initial_sp: u32,
    reset: u32,
    vector_table: u32,
    image_end: u32,
    ram: (u32, u32),
) -> Result<(), VectorError> {
    if !initial_sp.is_multiple_of(8) || initial_sp <= ram.0 || initial_sp > ram.1 {
        return Err(VectorError::StackPointer);
    }
    if reset & 1 == 0 || reset & !1 < vector_table || reset & !1 >= image_end {
        return Err(VectorError::ResetVector);
    }
    Ok(())
}

/// Copy progress granularity for the installing indication.
const BLINK_EVERY: u32 = 8 * 1024;

/// Map boot progress onto the indicator.
pub fn indicate(indicator: &mut impl Indicator, event: BootEvent) {
    if let BootEvent::Copying { done, .. } = event {
        indicator.installing((done / BLINK_EVERY).is_multiple_of(2));
    }
}

#[cfg(target_os = "none")]
mod target;
#[cfg(target_os = "none")]
pub use target::run;

#[cfg(test)]
mod tests {
    use super::*;

    const RAM: (u32, u32) = (0x2000_0000, 0x2000_9000);

    #[test]
    fn vector_table_checks() {
        let vt = 0x4100;
        let end = 0x4100 + 0x1000;
        assert_eq!(
            check_vector_table(0x2000_9000, 0x4201, vt, end, RAM),
            Ok(())
        );
        assert_eq!(
            check_vector_table(0x2000_1000, 0x4101 + 0xEFE, vt, end, RAM),
            Ok(())
        );
        for sp in [0x2000_0000, 0x2000_9008, 0x2000_8FFC, 0xFFFF_FFFF, 0] {
            assert_eq!(
                check_vector_table(sp, 0x4201, vt, end, RAM),
                Err(VectorError::StackPointer),
                "{sp:#x}"
            );
        }
        for reset in [0x4200, 0x40FF, 0x5101, 0x0000_0101, 0xFFFF_FFFF] {
            assert_eq!(
                check_vector_table(0x2000_9000, reset, vt, end, RAM),
                Err(VectorError::ResetVector),
                "{reset:#x}"
            );
        }
    }

    #[derive(Default)]
    struct Recorder {
        installing: Vec<bool>,
    }

    impl Indicator for Recorder {
        fn installing(&mut self, on: bool) {
            self.installing.push(on);
        }
        fn fault(&mut self, _on: bool) {}
        fn off(&mut self) {}
    }

    #[test]
    fn copy_progress_blinks_per_8k() {
        let mut recorder = Recorder::default();
        for done in (0..32 * 1024).step_by(4096) {
            indicate(
                &mut recorder,
                BootEvent::Copying {
                    done,
                    total: 32 * 1024,
                },
            );
        }
        indicate(&mut recorder, BootEvent::Validating);
        assert_eq!(
            recorder.installing,
            [true, true, false, false, true, true, false, false]
        );
    }

    #[test]
    fn loader_info_identifies_the_key() {
        let key = [0x42; 64];
        let info = LoaderInfo::decode(&loader_info(3, &key)).unwrap();
        assert_eq!(info.loader_version, 3);
        assert!(info.trusts(&key));
    }
}
