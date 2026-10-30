//! Overwrite-only, MCUboot-image-compatible bootloader core for the BL702.
//!
//! A product's bootloader binary (booted by the mask ROM from the boot
//! header's `img_start`, executing at the start of the XIP window) provides
//! the slot [`Layout`], the trusted public key and the XIP mapping, places
//! [`loader_info`] at a fixed address, and calls `run`. Each boot then:
//!
//! 1. installs a pending, valid, non-downgrade image from the secondary slot
//!    ([`mcuboot_image::boot`], power-fail safe and resumable: the request
//!    stays armed until the copied primary validates);
//! 2. validates the primary image (SHA-256 and ECDSA P-256 signature);
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

#![cfg_attr(not(test), no_std)]

use embedded_storage::nor_flash::ReadNorFlash;
use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind};
pub use mcuboot_image::{
    self, BootError, BootEvent, Bootable, ImageError, Layout, LoaderInfo, TrustedKey, Update,
};

/// Fixed application header size (`imgtool --header-size`): the
/// application entry sits this far into the primary slot.
pub const HEADER_SIZE: u32 = 0x100;

/// Loader configuration supplied by the product.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub layout: Layout,
    pub key: TrustedKey,
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

/// Encode the record applications read to detect this loader. Place the
/// result with `#[link_section]` at the address the product's application
/// expects (by convention the last 32 bytes of the loader region).
pub const fn loader_info(loader_version: u32, key_xy: &[u8; 64]) -> [u8; 32] {
    LoaderInfo::new(0, loader_version, key_xy).encode()
}

/// Why a validated image was not started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntryError {
    /// The header does not place the entry at the slot's fixed
    /// [`HEADER_SIZE`] offset the application is linked for.
    Offset,
    /// The entry is outside the image or misaligned.
    Range,
}

/// Check the application entry derived from a validated image: it must be
/// the fixed header offset into the primary slot (the address the product
/// links `_start` at), 4-byte aligned, and inside the image payload.
pub fn check_entry(primary: u32, entry: u32, image_end: u32) -> Result<(), EntryError> {
    if entry != primary + HEADER_SIZE {
        return Err(EntryError::Offset);
    }
    if !entry.is_multiple_of(4) || entry >= image_end {
        return Err(EntryError::Range);
    }
    Ok(())
}

/// Flash view used by the loader: reads anywhere, but erases and programs
/// only inside `[primary, secondary + slot_size)`.
pub struct BootFlash<F> {
    flash: F,
    start: u32,
    end: u32,
}

impl<F> BootFlash<F> {
    pub fn new(flash: F, layout: &Layout) -> Self {
        let start = core::cmp::min(layout.primary, layout.secondary);
        let end = core::cmp::max(layout.primary, layout.secondary) + layout.slot_size;
        Self { flash, start, end }
    }

    fn writable(&self, from: u32, len: usize) -> bool {
        from >= self.start
            && u32::try_from(len)
                .ok()
                .and_then(|len| from.checked_add(len))
                .is_some_and(|to| to <= self.end)
    }

    pub fn into_inner(self) -> F {
        self.flash
    }
}

/// [`BootFlash`] error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootFlashError<E> {
    /// An erase or program outside the slots was refused.
    Protected,
    Flash(E),
}

impl<E: NorFlashError> NorFlashError for BootFlashError<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Protected => NorFlashErrorKind::OutOfBounds,
            Self::Flash(error) => error.kind(),
        }
    }
}

impl<F: ErrorType> ErrorType for BootFlash<F> {
    type Error = BootFlashError<F::Error>;
}

impl<F: ReadNorFlash> ReadNorFlash for BootFlash<F> {
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash
            .read(offset, bytes)
            .map_err(BootFlashError::Flash)
    }

    fn capacity(&self) -> usize {
        self.flash.capacity()
    }
}

impl<F: NorFlash> NorFlash for BootFlash<F> {
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if to < from || !self.writable(from, (to - from) as usize) {
            return Err(BootFlashError::Protected);
        }
        self.flash.erase(from, to).map_err(BootFlashError::Flash)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if !self.writable(offset, bytes.len()) {
            return Err(BootFlashError::Protected);
        }
        self.flash
            .write(offset, bytes)
            .map_err(BootFlashError::Flash)
    }
}

/// Byte console for loader status lines (the product's UART).
pub trait Console {
    fn write(&mut self, bytes: &[u8]);
}

/// Console that discards everything.
pub struct NoConsole;

impl Console for NoConsole {
    fn write(&mut self, _bytes: &[u8]) {}
}

/// Write `value` as `0x` and eight hex digits.
pub fn write_hex(console: &mut impl Console, value: u32) {
    let mut text = *b"0x00000000";
    for (index, digit) in text[2..].iter_mut().enumerate() {
        let nibble = (value >> (28 - 4 * index)) & 0xF;
        *digit = b"0123456789abcdef"[nibble as usize];
    }
    console.write(&text);
}

/// Write `v<major>.<minor>.<revision>+<build>`.
pub fn write_version(console: &mut impl Console, version: &mcuboot_image::ImageVersion) {
    console.write(b"v");
    write_decimal(console, u32::from(version.major));
    console.write(b".");
    write_decimal(console, u32::from(version.minor));
    console.write(b".");
    write_decimal(console, u32::from(version.revision));
    console.write(b"+");
    write_decimal(console, version.build);
}

/// Write `value` in decimal.
pub fn write_decimal(console: &mut impl Console, mut value: u32) {
    let mut text = [0u8; 10];
    let mut start = text.len();
    loop {
        start -= 1;
        text[start] = b'0' + (value % 10) as u8;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    console.write(&text[start..]);
}

/// Short name of an image validation failure.
pub fn image_error_name(error: ImageError) -> &'static [u8] {
    match error {
        ImageError::Flash => b"flash",
        ImageError::NoImage => b"no image",
        ImageError::Malformed => b"malformed",
        ImageError::Unsupported => b"unsupported",
        ImageError::HashMismatch => b"hash mismatch",
        ImageError::BadSignature => b"bad signature",
    }
}

/// Turns boot progress and outcomes into console lines.
#[derive(Debug, Default)]
pub struct Reporter {
    copying: bool,
}

impl Reporter {
    /// Boot progress: one line when an install starts and a dot per 64 KiB
    /// copied.
    pub fn event(&mut self, console: &mut impl Console, event: BootEvent) {
        match event {
            BootEvent::Erasing => console.write(b"bl702-boot: installing update "),
            BootEvent::Copying { done, .. } => {
                self.copying = true;
                if done.is_multiple_of(64 * 1024) {
                    console.write(b".");
                }
            }
            BootEvent::Validating | BootEvent::Dropping => self.end_line(console),
        }
    }

    /// Outcome of the upgrade-request step.
    pub fn update(&mut self, console: &mut impl Console, update: Update) {
        self.end_line(console);
        match update {
            Update::None => {}
            Update::Installed => console.write(b"bl702-boot: update installed\r\n"),
            Update::Downgrade => console.write(b"bl702-boot: update rejected: downgrade\r\n"),
            Update::Rejected(error) => {
                console.write(b"bl702-boot: update rejected: ");
                console.write(image_error_name(error));
                console.write(b"\r\n");
            }
        }
    }

    /// Terminate a progress line, if one is open.
    pub fn end_line(&mut self, console: &mut impl Console) {
        if core::mem::take(&mut self.copying) {
            console.write(b"\r\n");
        }
    }
}

#[cfg(target_arch = "riscv32")]
mod target;
#[cfg(target_arch = "riscv32")]
pub use target::run;

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn xip_mapping() {
        let config = Config {
            layout: LAYOUT,
            key: TrustedKey::new([1; 64]),
            img_start: 0x2000,
            xip_base: 0x2300_0000,
        };
        assert_eq!(config.xip_address(0x2000), Some(0x2300_0000));
        assert_eq!(config.xip_address(0x10100), Some(0x2300_E100));
        assert_eq!(config.xip_address(0x1000), None);
    }

    struct Ram {
        bytes: Vec<u8>,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct RamError;

    impl NorFlashError for RamError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    impl ErrorType for Ram {
        type Error = RamError;
    }

    impl ReadNorFlash for Ram {
        const READ_SIZE: usize = 1;
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), RamError> {
            let start = offset as usize;
            bytes.copy_from_slice(&self.bytes[start..start + bytes.len()]);
            Ok(())
        }
        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for Ram {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = 0x1000;
        fn erase(&mut self, from: u32, to: u32) -> Result<(), RamError> {
            self.bytes[from as usize..to as usize].fill(0xFF);
            Ok(())
        }
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), RamError> {
            let start = offset as usize;
            for (cell, byte) in self.bytes[start..start + bytes.len()].iter_mut().zip(bytes) {
                *cell &= *byte;
            }
            Ok(())
        }
    }

    #[test]
    fn boot_flash_protects_loader_and_journal() {
        let mut flash = BootFlash::new(
            Ram {
                bytes: vec![0; 0x10_0000],
            },
            &LAYOUT,
        );
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
        assert!(ram.bytes[..0x10000].iter().all(|byte| *byte == 0));
        assert!(ram.bytes[0xFE000..].iter().all(|byte| *byte == 0));
    }

    struct Text(Vec<u8>);

    impl Console for Text {
        fn write(&mut self, bytes: &[u8]) {
            self.0.extend_from_slice(bytes);
        }
    }

    #[test]
    fn console_formatting() {
        let mut text = Text(Vec::new());
        write_hex(&mut text, 0x2300_E100);
        text.write(b" ");
        write_version(
            &mut text,
            &mcuboot_image::ImageVersion {
                major: 0,
                minor: 1,
                revision: 2,
                build: 0,
            },
        );
        assert_eq!(text.0, b"0x2300e100 v0.1.2+0");
    }

    #[test]
    fn copy_progress_lines() {
        let mut text = Text(Vec::new());
        let mut reporter = Reporter::default();
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
        assert_eq!(
            text.0,
            b"bl702-boot: installing update ...\r\nbl702-boot: update installed\r\n\
              bl702-boot: update rejected: bad signature\r\n"
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
