//! [`FirmwareWriter`] that stages a downloaded MCUboot image in the
//! secondary slot of an overwrite-only layout.
//!
//! The writer accepts either an unpadded `imgtool sign` image or a
//! `--pad`ded full-slot image (koptserg-style OTA files). In both cases the
//! upgrade request magic is *withheld* while downloading and written only by
//! [`activate`](FirmwareWriter::activate), after the complete image has been
//! validated with the trusted key and checked against downgrade prevention.
//! A reset at any time before activation therefore leaves no request behind.
//!
//! Sectors are erased lazily as sequential writes reach them, so the radio
//! is never blocked for a whole-slot erase. [`erase_slot`] erases the
//! trailer sector first (dropping any stale request) and then the header
//! sector.

use embedded_storage::nor_flash::NorFlash;
use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

use crate::boot::Layout;
use crate::image::{ImageHeader, ImageVersion, TrustedKey, validate};
use crate::trailer::{BOOT_MAGIC, MagicState, magic_address};

/// Staging buffer; must be a multiple of the flash write size.
const STAGE: usize = 64;

/// Installation policy checked before activation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlotPolicy {
    /// The trusted signing key of the bootloader that will install the image.
    pub key: TrustedKey,
    /// Required `hdr_size` (the application vector-table offset).
    pub header_size: u16,
}

pub struct SlotWriter<F> {
    flash: F,
    layout: Layout,
    policy: SlotPolicy,
    /// Next expected slot offset.
    written: u32,
    /// Slot offset below which every sector has been erased.
    erased_until: u32,
    stage: [u8; STAGE],
    stage_len: usize,
    verified: bool,
    /// Set by `erase_slot`: this writer may have left data in the slot.
    staging: bool,
}

impl<F: NorFlash> SlotWriter<F> {
    pub fn new(flash: F, layout: Layout, policy: SlotPolicy) -> Self {
        assert!(
            STAGE.is_multiple_of(F::WRITE_SIZE)
                && layout.sector_size.is_multiple_of(F::ERASE_SIZE as u32)
        );
        Self {
            flash,
            layout,
            policy,
            written: 0,
            erased_until: 0,
            stage: [0xFF; STAGE],
            stage_len: 0,
            verified: false,
            staging: false,
        }
    }

    pub fn flash_mut(&mut self) -> &mut F {
        &mut self.flash
    }

    /// The release version of the running (primary) image, if it has a
    /// header.
    pub fn running_version(&mut self) -> Option<ImageVersion> {
        ImageHeader::read(&mut self.flash, self.layout.primary)
            .ok()
            .flatten()
            .map(|header| header.version)
    }

    fn trailer_offset(&self) -> u32 {
        self.layout.slot_size - self.layout.sector_size
    }

    fn erase_at(&mut self, offset: u32) -> Result<(), FirmwareError> {
        let start = self.layout.secondary + offset;
        self.flash
            .erase(start, start + self.layout.sector_size)
            .map_err(|_| FirmwareError::EraseFailed)
    }

    /// Erase every sector below `end` that has not been erased yet.
    fn ensure_erased(&mut self, end: u32) -> Result<(), FirmwareError> {
        while self.erased_until < end {
            let sector = self.erased_until;
            // Sector 0 and the trailer sector were erased by `erase_slot`.
            if sector != 0 && sector != self.trailer_offset() {
                self.erase_at(sector)?;
            }
            self.erased_until += self.layout.sector_size;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), FirmwareError> {
        if self.stage_len == 0 {
            return Ok(());
        }
        let len = self.stage_len.div_ceil(F::WRITE_SIZE) * F::WRITE_SIZE;
        self.stage[self.stage_len..len].fill(0xFF);
        let offset = self.written - self.stage_len as u32;
        // Withhold the request magic until activation.
        let magic = magic_address(0, self.layout.slot_size);
        for (index, byte) in self.stage[..len].iter_mut().enumerate() {
            if offset + index as u32 >= magic {
                *byte = 0xFF;
            }
        }
        self.ensure_erased(offset + len as u32)?;
        if self.stage[..len].iter().any(|byte| *byte != 0xFF) {
            self.flash
                .write(self.layout.secondary + offset, &self.stage[..len])
                .map_err(|_| FirmwareError::WriteFailed)?;
        }
        self.stage_len = 0;
        Ok(())
    }

    fn reset(&mut self) {
        self.written = 0;
        self.erased_until = 0;
        self.stage_len = 0;
        self.verified = false;
    }
}

impl<F: NorFlash> FirmwareWriter for SlotWriter<F> {
    fn erase_slot(&mut self) -> Result<(), FirmwareError> {
        self.reset();
        self.staging = true;
        self.erase_at(self.trailer_offset())?;
        self.erase_at(0)
    }

    fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError> {
        if offset != self.written {
            return Err(FirmwareError::OutOfRange);
        }
        if offset
            .checked_add(data.len() as u32)
            .is_none_or(|end| end > self.layout.slot_size)
        {
            return Err(FirmwareError::ImageTooLarge);
        }
        self.verified = false;
        let mut rest = data;
        while !rest.is_empty() {
            let take = core::cmp::min(STAGE - self.stage_len, rest.len());
            self.stage[self.stage_len..self.stage_len + take].copy_from_slice(&rest[..take]);
            self.stage_len += take;
            self.written += take as u32;
            rest = &rest[take..];
            if self.stage_len == STAGE {
                self.flush()?;
            }
        }
        Ok(())
    }

    fn verify(
        &mut self,
        expected_size: u32,
        _expected_hash: Option<&[u8]>,
    ) -> Result<(), FirmwareError> {
        self.verified = false;
        self.flush()?;
        if self.written != expected_size {
            return Err(FirmwareError::VerifyFailed);
        }
        let image = validate(
            &mut self.flash,
            self.layout.secondary,
            self.layout.max_image_len(),
            &self.policy.key,
        )
        .map_err(|_| FirmwareError::VerifyFailed)?;
        if image.header.hdr_size != self.policy.header_size || image.end > self.written {
            return Err(FirmwareError::VerifyFailed);
        }
        if let Some(running) = self.running_version()
            && image.header.version.cmp_release(&running).is_lt()
        {
            return Err(FirmwareError::VerifyFailed);
        }
        self.verified = true;
        Ok(())
    }

    fn activate(&mut self) -> Result<(), FirmwareError> {
        if !self.verified {
            return Err(FirmwareError::ActivateFailed);
        }
        let address = magic_address(self.layout.secondary, self.layout.slot_size);
        let mut current = [0u8; 16];
        self.flash
            .read(address, &mut current)
            .map_err(|_| FirmwareError::HardwareError)?;
        match MagicState::classify(&current) {
            MagicState::Good => Ok(()),
            MagicState::Erased => {
                self.flash
                    .write(address, &BOOT_MAGIC)
                    .map_err(|_| FirmwareError::ActivateFailed)?;
                self.flash
                    .read(address, &mut current)
                    .map_err(|_| FirmwareError::HardwareError)?;
                if current == BOOT_MAGIC {
                    Ok(())
                } else {
                    Err(FirmwareError::ActivateFailed)
                }
            }
            MagicState::Other => Err(FirmwareError::ActivateFailed),
        }
    }

    fn slot_size(&self) -> u32 {
        self.layout.slot_size
    }

    /// Drop a staged download. An abort before any `erase_slot` (for
    /// example a declined offer) leaves the flash untouched: there is
    /// nothing to drop, and a battery device should not spend a sector
    /// erase on it.
    fn abort(&mut self) -> Result<(), FirmwareError> {
        self.reset();
        if !self.staging {
            return Ok(());
        }
        self.erase_at(self.trailer_offset())?;
        self.staging = false;
        Ok(())
    }
}
