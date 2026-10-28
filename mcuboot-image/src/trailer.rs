//! The MCUboot image trailer magic that requests an upgrade.
//!
//! In overwrite-only mode the only trailer field either bootloader acts on
//! is the 16-byte magic at the very end of the secondary slot
//! (`BOOT_MAX_ALIGN = 8`, the default for single-image builds). `imgtool
//! sign --pad` already places it there; an unpadded image gets it from the
//! application after the download has been verified.

use embedded_storage::nor_flash::ReadNorFlash;

/// `boot_img_magic` for `BOOT_MAX_ALIGN == 8`.
pub const BOOT_MAGIC: [u8; 16] = [
    0x77, 0xC2, 0x95, 0xF3, 0x60, 0xD2, 0xEF, 0x7F, 0x35, 0x52, 0x50, 0x0F, 0x2C, 0xB6, 0x79, 0x80,
];
pub const BOOT_MAGIC_LEN: u32 = BOOT_MAGIC.len() as u32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagicState {
    /// The upgrade request magic is present.
    Good,
    /// All 16 bytes read as erased flash.
    Erased,
    /// Anything else, including an interrupted write or erase.
    Other,
}

impl MagicState {
    pub fn classify(bytes: &[u8; 16]) -> Self {
        if *bytes == BOOT_MAGIC {
            Self::Good
        } else if bytes.iter().all(|byte| *byte == 0xFF) {
            Self::Erased
        } else {
            Self::Other
        }
    }
}

/// Absolute address of the magic of the slot at `slot`.
pub const fn magic_address(slot: u32, slot_size: u32) -> u32 {
    slot + slot_size - BOOT_MAGIC_LEN
}

pub fn read_magic<F: ReadNorFlash>(
    flash: &mut F,
    slot: u32,
    slot_size: u32,
) -> Result<MagicState, F::Error> {
    let mut bytes = [0u8; 16];
    flash.read(magic_address(slot, slot_size), &mut bytes)?;
    Ok(MagicState::classify(&bytes))
}
