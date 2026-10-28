//! Overwrite-only boot decision and copy, compatible with MCUboot's
//! `MCUBOOT_OVERWRITE_ONLY` single-image mode.
//!
//! Every boot:
//!
//! 1. If the secondary slot ends with [`BOOT_MAGIC`](crate::trailer::BOOT_MAGIC),
//!    validate the secondary image (hash, signature, geometry) and apply
//!    MCUboot's downgrade rule: refuse an image whose release version is
//!    lower than the primary's while the primary still has a header.
//! 2. An accepted image is installed: erase the primary sectors it needs,
//!    copy it, re-validate the primary, then erase the secondary trailer
//!    sector (dropping the request) and the secondary header sector.
//!    A rejected image is dropped the same way without touching the primary.
//! 3. Validate the primary and hand back its vector table address.
//!
//! **Power-fail safety.** The request (the magic) survives until the primary
//! holds a complete, validated copy, so an interrupted install simply runs
//! again on the next boot: the secondary is never modified before that, the
//! downgrade rule allows equal versions, and a half-erased primary has no
//! header and so no version to compare against. Interrupting the final
//! erases leaves a valid primary and a secondary without a request.

use embedded_storage::nor_flash::NorFlash;

use crate::image::{ImageError, ImageHeader, TrustedKey, ValidImage, validate};
use crate::trailer::{MagicState, read_magic};

/// Copy unit; bounded stack use.
const COPY_CHUNK: usize = 256;

/// Two equally sized slots in one flash device, by absolute address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layout {
    pub primary: u32,
    pub secondary: u32,
    pub slot_size: u32,
    pub sector_size: u32,
}

impl Layout {
    /// The longest image (header through TLVs) a slot may hold: the last
    /// sector is reserved for the trailer.
    pub const fn max_image_len(&self) -> u32 {
        self.slot_size - self.sector_size
    }

    pub const fn trailer_sector(&self, slot: u32) -> u32 {
        slot + self.slot_size - self.sector_size
    }

    const fn round_up(&self, len: u32) -> u32 {
        len.div_ceil(self.sector_size) * self.sector_size
    }
}

/// What happened to a pending upgrade request on this boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Update {
    /// No request magic in the secondary slot.
    None,
    /// The secondary image was copied into the primary slot.
    Installed,
    /// The request was dropped because the image failed validation.
    Rejected(ImageError),
    /// The request was dropped by downgrade prevention.
    Downgrade,
}

/// Progress for status indication; the loader may blink an LED.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootEvent {
    Validating,
    Erasing,
    Copying { done: u32, total: u32 },
    Dropping,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bootable {
    pub image: ValidImage,
    /// Absolute address of the application's vector table.
    pub vector_table: u32,
    pub update: Update,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootError {
    /// A flash erase, write or read failed.
    Flash,
    /// The installed copy does not validate although the source did. The
    /// request is kept; retrying needs a reset.
    CopyCorrupt,
    /// Nothing valid to boot.
    NoBootableImage { primary: ImageError, update: Update },
}

/// Run one boot decision. `progress` is called before each long step.
pub fn boot<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    key: &TrustedKey,
    mut progress: impl FnMut(BootEvent),
) -> Result<Bootable, BootError> {
    let update = process_request(flash, layout, key, &mut progress)?;
    progress(BootEvent::Validating);
    match validate(flash, layout.primary, layout.max_image_len(), key) {
        Ok(image) => Ok(Bootable {
            image,
            vector_table: layout.primary + image.header.hdr_size as u32,
            update,
        }),
        Err(ImageError::Flash) => Err(BootError::Flash),
        Err(primary) => Err(BootError::NoBootableImage { primary, update }),
    }
}

fn process_request<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    key: &TrustedKey,
    progress: &mut impl FnMut(BootEvent),
) -> Result<Update, BootError> {
    let magic =
        read_magic(flash, layout.secondary, layout.slot_size).map_err(|_| BootError::Flash)?;
    if magic != MagicState::Good {
        return Ok(Update::None);
    }
    progress(BootEvent::Validating);
    let update = match validate(flash, layout.secondary, layout.max_image_len(), key) {
        Err(ImageError::Flash) => return Err(BootError::Flash),
        Err(error) => Update::Rejected(error),
        Ok(candidate) => {
            let primary = ImageHeader::read(flash, layout.primary).map_err(|_| BootError::Flash)?;
            let downgrade = primary.is_some_and(|primary| {
                candidate
                    .header
                    .version
                    .cmp_release(&primary.version)
                    .is_lt()
            });
            if downgrade {
                Update::Downgrade
            } else {
                install(flash, layout, key, &candidate, progress)?;
                Update::Installed
            }
        }
    };
    progress(BootEvent::Dropping);
    erase_sector(
        flash,
        layout.trailer_sector(layout.secondary),
        layout.sector_size,
    )?;
    erase_sector(flash, layout.secondary, layout.sector_size)?;
    Ok(update)
}

fn install<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    key: &TrustedKey,
    image: &ValidImage,
    progress: &mut impl FnMut(BootEvent),
) -> Result<(), BootError> {
    let total = image.end.div_ceil(F::WRITE_SIZE as u32) * F::WRITE_SIZE as u32;
    progress(BootEvent::Erasing);
    let erase_end = layout.primary + layout.round_up(total);
    flash
        .erase(layout.primary, erase_end)
        .map_err(|_| BootError::Flash)?;

    let mut buffer = [0u8; COPY_CHUNK];
    let mut offset = 0;
    while offset < total {
        progress(BootEvent::Copying {
            done: offset,
            total,
        });
        let chunk = core::cmp::min(COPY_CHUNK as u32, total - offset) as usize;
        let data = &mut buffer[..chunk];
        flash
            .read(layout.secondary + offset, data)
            .map_err(|_| BootError::Flash)?;
        if data.iter().any(|byte| *byte != 0xFF) {
            flash
                .write(layout.primary + offset, data)
                .map_err(|_| BootError::Flash)?;
        }
        offset += chunk as u32;
    }

    progress(BootEvent::Validating);
    match validate(flash, layout.primary, layout.max_image_len(), key) {
        Ok(copy) if copy.digest == image.digest => Ok(()),
        Err(ImageError::Flash) => Err(BootError::Flash),
        _ => Err(BootError::CopyCorrupt),
    }
}

fn erase_sector<F: NorFlash>(flash: &mut F, address: u32, size: u32) -> Result<(), BootError> {
    flash
        .erase(address, address + size)
        .map_err(|_| BootError::Flash)
}
