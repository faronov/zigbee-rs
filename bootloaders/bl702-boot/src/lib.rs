//! BL702 shell of the zigbee-rs bootloader core ([`zrs_boot_core`]).
//!
//! A product's bootloader binary (booted by the mask ROM from the boot
//! header's `img_start`, executing at the start of the XIP window) provides
//! the [`BootConfig`] (slot [`Layout`], trusted public keys, trial boots)
//! and the XIP mapping, places [`loader_block`] (or the single-key
//! [`loader_info`]) at a fixed address, and calls `run`. Each boot then runs
//! the core sequence ([`zrs_boot_core::run`]):
//!
//! 1. installs a pending, valid, non-downgrade image from the secondary slot
//!    by overwrite (power-fail safe and resumable: the request stays armed
//!    until the copied primary validates);
//! 2. validates the primary image (SHA-256 and an ECDSA P-256 — or, with
//!    feature `ed25519`, Ed25519 — signature by a trusted key) and accounts
//!    a trial boot while an installed image is unconfirmed;
//! 3. checks the entry point, invalidates the L1C cache that fronts XIP
//!    (the primary was possibly reprogrammed behind it) and jumps to the
//!    application's riscv-rt `_start` with interrupts disabled, leaving
//!    clocks in the ROM boot state.
//!
//! All flash access goes through [`BootFlash`], which refuses to erase or
//! program anything outside the two slots, so the boot header, the loader
//! and the product's persistence partitions cannot be touched by the copy.
//!
//! With nothing bootable it reports on the console and halts; a power cycle
//! retries. Recovery is reflashing over the ROM's UART/USB ISP.
//!
//! This crate keeps only what is BL702-specific: the XIP mapping, the
//! fixed `riscv-rt` entry offset, the console prefix and the target
//! hand-over in `target`.

#![cfg_attr(not(test), no_std)]

pub use zrs_boot_core::console::{
    Console, NoConsole, Reporter, image_error_name, write_decimal, write_hex, write_version,
};
pub use zrs_boot_core::entry::EntryError;
pub use zrs_boot_core::flash::{BootFlash, BootFlashError};
pub use zrs_boot_core::{
    BootConfig, BootError, BootEvent, Bootable, ImageError, Layout, LoaderInfo, Mode, Trial,
    TrustedKey, Update, loader_block, loader_info, mcuboot_image,
};

/// Fixed application header size (`imgtool --header-size`): the
/// application entry sits this far into the primary slot.
pub const HEADER_SIZE: u32 = 0x100;

/// Prefix of every console line.
pub const LOG_PREFIX: &[u8] = b"bl702-boot: ";

/// Loader configuration supplied by the product.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Slots, trusted keys, trial boots (install mode is always overwrite
    /// on this chip).
    pub boot: BootConfig<'static>,
    /// Physical flash offset (boot header `img_start`) that the ROM maps at
    /// [`Config::xip_base`].
    pub img_start: u32,
    /// Start of the flash XIP window.
    pub xip_base: u32,
}

impl Config {
    /// XIP address of physical flash offset `physical`, if mapped.
    pub const fn xip_address(&self, physical: u32) -> Option<u32> {
        if physical < self.img_start {
            return None;
        }
        self.xip_base.checked_add(physical - self.img_start)
    }
}

/// The status reporter writing [`LOG_PREFIX`] lines.
pub const fn reporter() -> Reporter {
    Reporter::new(LOG_PREFIX)
}

/// Check the application entry derived from a validated image: it must be
/// the fixed header offset into the primary slot (the address the product
/// links `_start` at), 4-byte aligned, and inside the image payload.
pub fn check_entry(primary: u32, entry: u32, image_end: u32) -> Result<(), EntryError> {
    zrs_boot_core::entry::check_fixed_entry(primary, HEADER_SIZE, entry, image_end)
}

#[cfg(target_arch = "riscv32")]
mod target;
#[cfg(target_arch = "riscv32")]
pub use target::run;

#[cfg(test)]
mod tests {
    use super::*;
    use zrs_boot_core::embedded_storage::nor_flash::NorFlash;
    use zrs_boot_core::sim::SimFlash;

    const LAYOUT: Layout = Layout {
        primary: 0x10000,
        secondary: 0x87000,
        slot_size: 0x77000,
        sector_size: 0x1000,
    };

    #[test]
    fn entry_checks() {
        assert_eq!(check_entry(0x10000, 0x10100, 0x40000), Ok(()));
        assert_eq!(
            check_entry(0x10000, 0x10200, 0x40000),
            Err(EntryError::Offset)
        );
        assert_eq!(
            check_entry(0x10000, 0x10100, 0x10100),
            Err(EntryError::Range)
        );
        assert_eq!(
            check_entry(0x10002, 0x10102, 0x40000),
            Err(EntryError::Range)
        );
    }

    static KEYS: [TrustedKey; 1] = [TrustedKey::new([1; 64])];

    #[test]
    fn xip_mapping() {
        let config = Config {
            boot: BootConfig::overwrite(LAYOUT, &KEYS),
            img_start: 0x2000,
            xip_base: 0x2300_0000,
        };
        assert_eq!(config.xip_address(0x2000), Some(0x2300_0000));
        assert_eq!(config.xip_address(0x10100), Some(0x2300_E100));
        assert_eq!(config.xip_address(0x1000), None);
    }

    #[test]
    fn boot_flash_protects_loader_and_journal() {
        let mut ram = SimFlash::<0x1000, 1>::new(0x10_0000);
        ram.load(0, &[0; 0x10_0000]);
        let mut flash = BootFlash::new(ram, &LAYOUT);
        // Boot header, loader, LoaderInfo and the journal are refused.
        for (from, to) in [
            (0x0, 0x1000),
            (0xF000, 0x10000),
            (0xF000, 0x11000),
            (0xFE000, 0xFF000),
            (0xFD000, 0xFF000),
        ] {
            assert_eq!(flash.erase(from, to), Err(BootFlashError::Protected));
        }
        assert_eq!(flash.write(0xFFE0, &[0]), Err(BootFlashError::Protected));
        assert_eq!(
            flash.write(0xFDFFF, &[0, 0]),
            Err(BootFlashError::Protected)
        );
        assert_eq!(flash.write(u32::MAX, &[0]), Err(BootFlashError::Protected));
        // Both slots are writable up to their ends.
        assert_eq!(flash.erase(0x10000, 0x11000), Ok(()));
        assert_eq!(flash.erase(0xFD000, 0xFE000), Ok(()));
        assert_eq!(flash.write(0xFDFFF, &[0]), Ok(()));
        let ram = flash.into_inner();
        assert!(ram.bytes()[..0x10000].iter().all(|byte| *byte == 0));
        assert!(ram.bytes()[0xFE000..].iter().all(|byte| *byte == 0));
    }

    struct Text(Vec<u8>);

    impl Console for Text {
        fn write(&mut self, bytes: &[u8]) {
            self.0.extend_from_slice(bytes);
        }
    }

    /// The console lines are unchanged from the pre-core loader.
    #[test]
    fn copy_progress_lines() {
        let mut text = Text(Vec::new());
        let mut reporter = reporter();
        let total = 0x30000;
        reporter.event(&mut text, BootEvent::Validating);
        reporter.event(&mut text, BootEvent::Erasing);
        let mut done = 0;
        while done < total {
            reporter.event(&mut text, BootEvent::Copying { done, total });
            done += 256;
        }
        reporter.event(&mut text, BootEvent::Validating);
        reporter.event(&mut text, BootEvent::Dropping);
        reporter.update(&mut text, Update::Installed);
        reporter.update(&mut text, Update::Rejected(ImageError::BadSignature));
        reporter.update(&mut text, Update::Downgrade);
        assert_eq!(
            text.0,
            b"bl702-boot: installing update ...\r\nbl702-boot: update installed\r\n\
              bl702-boot: update rejected: bad signature\r\n\
              bl702-boot: update rejected: downgrade\r\n"
        );
    }

    #[test]
    fn trial_lines() {
        let mut text = Text(Vec::new());
        let mut reporter = reporter();
        reporter.trial(&mut text, Trial::Permanent);
        reporter.trial(&mut text, Trial::Confirmed);
        reporter.trial(&mut text, Trial::Testing { boot: 2 });
        reporter.trial(&mut text, Trial::Unconfirmed);
        assert_eq!(
            text.0,
            b"bl702-boot: trial boot 2\r\nbl702-boot: image unconfirmed\r\n"
        );
    }

    #[test]
    fn loader_info_identifies_the_key() {
        let key = [0x42; 64];
        let info = LoaderInfo::decode(&loader_info(3, &key)).unwrap();
        assert_eq!(info.loader_version, 3);
        assert!(info.trusts(&key));
        // The multi-key block ends in the same v1 record.
        let block = loader_block(3, &[TrustedKey::new(key)]);
        assert_eq!(block[32..], loader_info(3, &key));
        assert!(block[..32].iter().all(|byte| *byte == 0xFF));
    }
}
