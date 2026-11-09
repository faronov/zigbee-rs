//! [`FirmwareWriter`] that stages a downloaded MCUboot image in the
//! secondary slot of an overwrite-only layout.
//!
//! The writer accepts either an unpadded `imgtool sign` image or a
//! `--pad`ded full-slot image (koptserg-style OTA files). In both cases the
//! upgrade request magic is *withheld* while downloading and written only by
//! [`activate`](FirmwareWriter::activate), after the complete image has been
//! validated with a trusted key and checked against downgrade prevention.
//! A reset at any time before activation therefore leaves no request behind.
//!
//! Sectors are erased lazily as sequential writes reach them, so the radio
//! is never blocked for a whole-slot erase. [`erase_slot`] erases the
//! trailer sector first (dropping any stale request) and then the header
//! sector.
//!
//! # Resume after a reset
//!
//! A writer built with [`SlotWriter::with_resume_checkpoints`] in
//! [`Mode::Overwrite`] keeps download progress
//! ([`checkpoint`](FirmwareWriter::checkpoint)) at the start of the
//! secondary slot's trailer sector, which never holds image bytes (the image
//! ends before it and download bytes that fall into it are not written).
//! The loader reads only the request magic at the end of that sector and
//! erases the whole sector after an install or a dropped request;
//! [`erase_slot`] and [`abort`](FirmwareWriter::abort) erase it too. The
//! records therefore live and die with the staged download and cost no extra
//! erase: one 32-byte session header (image identity, CRC-32) followed by
//! append-only 8-byte progress entries (`staged`, `!staged`). A torn entry
//! fails its complement and is skipped. [`resume`](FirmwareWriter::resume)
//! continues at the last recorded offset rounded down to a sector, and
//! erases that sector again before writing it, so no byte is programmed
//! twice. The final signature check still covers the whole image.
//!
//! [`confirm_image`](FirmwareWriter::confirm_image) confirms the running
//! image to a loader with trial boots. In [`Mode::SwapMove`] the secondary
//! slot holds the previous image while the running one is on trial, so
//! staging a new download is refused until the running image is confirmed.

use embedded_storage::nor_flash::NorFlash;
use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter, ResumeRecord};

use crate::boot::{Layout, Mode, TrialState, confirm, trial_state};
use crate::image::{ImageHeader, ImageVersion, KeySet, validate_with_set};
use crate::loader_info::crc32;
use crate::trailer::{BOOT_MAGIC, MagicState, magic_address};

/// Staging buffer; must be a multiple of the flash write size.
const STAGE: usize = 64;

/// Resume session header at the start of the secondary trailer sector.
const RESUME_MAGIC: [u8; 8] = *b"ZRSOTAR1";
const RESUME_HEADER_LEN: u32 = 32;
/// One progress entry: `staged` and its complement.
const RESUME_ENTRY_LEN: u32 = 8;
/// End of the progress area, before the trailer-sector tail that
/// MCUboot-format tools use (magic, flags).
const RESUME_TAIL_RESERVE: u32 = 128;

/// Installation policy checked before activation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlotPolicy {
    /// Keys the bootloader that will install the image trusts.
    pub keys: KeySet,
    /// Required `hdr_size` (the application vector-table offset).
    pub header_size: u16,
    /// The loader's install mode (where its trial log lives).
    pub mode: Mode,
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
    /// Trailer-sector offset of the next free resume entry; 0 before the
    /// session header is written, `u32::MAX` once checkpoints are off.
    resume_next: u32,
    /// Staged bytes between resume checkpoints; 0 keeps none.
    checkpoint_every: u32,
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
            resume_next: 0,
            checkpoint_every: 0,
        }
    }

    /// Keep download progress every `every` staged bytes so a download
    /// continues after a reset (see the module documentation). `0` keeps
    /// none. Ignored, with no checkpoints, in [`Mode::SwapMove`] or when the
    /// flash cannot program an 8-byte entry on its own.
    #[must_use]
    pub fn with_resume_checkpoints(mut self, every: u32) -> Self {
        self.checkpoint_every = every;
        self
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
        // Nothing reaches the trailer sector: the request magic is withheld
        // until activation and the resume records live there.
        let trailer = self.trailer_offset();
        for (index, byte) in self.stage[..len].iter_mut().enumerate() {
            if offset + index as u32 >= trailer {
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
        self.resume_next = 0;
    }

    /// Resume records need the overwrite trailer and 8-byte programming.
    fn resume_supported(&self) -> bool {
        matches!(self.policy.mode, Mode::Overwrite)
            && RESUME_ENTRY_LEN.is_multiple_of(F::WRITE_SIZE as u32)
            && self.layout.sector_size >= RESUME_HEADER_LEN + RESUME_TAIL_RESERVE + RESUME_ENTRY_LEN
    }

    fn resume_base(&self) -> u32 {
        self.layout.secondary + self.trailer_offset()
    }

    fn resume_end(&self) -> u32 {
        self.layout.sector_size - RESUME_TAIL_RESERVE
    }

    fn resume_header(record: &ResumeRecord) -> [u8; RESUME_HEADER_LEN as usize] {
        let mut out = [0xFF; RESUME_HEADER_LEN as usize];
        out[..8].copy_from_slice(&RESUME_MAGIC);
        out[8..10].copy_from_slice(&record.manufacturer_code.to_le_bytes());
        out[10..12].copy_from_slice(&record.image_type.to_le_bytes());
        out[12..16].copy_from_slice(&record.file_version.to_le_bytes());
        out[16..20].copy_from_slice(&record.image_size.to_le_bytes());
        out[20..24].copy_from_slice(&record.payload_start.to_le_bytes());
        out[24..28].copy_from_slice(&record.firmware_size.to_le_bytes());
        let crc = crc32(&out, 28);
        out[28..].copy_from_slice(&crc.to_le_bytes());
        out
    }

    fn read_resume(&mut self) -> Option<(ResumeRecord, u32)> {
        let word = |bytes: &[u8], at: usize| {
            u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
        };
        let base = self.resume_base();
        let mut header = [0u8; RESUME_HEADER_LEN as usize];
        self.flash.read(base, &mut header).ok()?;
        if header[..8] != RESUME_MAGIC || word(&header, 28) != crc32(&header, 28) {
            return None;
        }
        let mut record = ResumeRecord {
            manufacturer_code: u16::from_le_bytes([header[8], header[9]]),
            image_type: u16::from_le_bytes([header[10], header[11]]),
            file_version: word(&header, 12),
            image_size: word(&header, 16),
            payload_start: word(&header, 20),
            firmware_size: word(&header, 24),
            staged: 0,
        };
        let mut next = RESUME_HEADER_LEN;
        while next + RESUME_ENTRY_LEN <= self.resume_end() {
            let mut entry = [0u8; RESUME_ENTRY_LEN as usize];
            self.flash.read(base + next, &mut entry).ok()?;
            if entry == [0xFF; RESUME_ENTRY_LEN as usize] {
                break;
            }
            next += RESUME_ENTRY_LEN;
            let staged = word(&entry, 0);
            // A torn entry fails the complement; progress only grows.
            if word(&entry, 4) == !staged
                && staged > record.staged
                && staged <= record.firmware_size
            {
                record.staged = staged;
            }
        }
        Some((record, next))
    }
}

impl<F: NorFlash> FirmwareWriter for SlotWriter<F> {
    fn erase_slot(&mut self) -> Result<(), FirmwareError> {
        #[cfg(feature = "swap-move")]
        if matches!(self.policy.mode, Mode::SwapMove { .. })
            && matches!(
                trial_state(&mut self.flash, &self.layout, self.policy.mode),
                Ok(TrialState::Testing { .. }) | Err(_)
            )
        {
            // The secondary slot is the revert image.
            return Err(FirmwareError::EraseFailed);
        }
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
        let image = validate_with_set(
            &mut self.flash,
            self.layout.secondary,
            self.layout.max_image_len(),
            &self.policy.keys,
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

    fn checkpoint_interval(&self) -> u32 {
        if self.resume_supported() {
            self.checkpoint_every
        } else {
            0
        }
    }

    /// Append one progress entry (and the session header before the first).
    /// Only bytes already programmed count; the unflushed staging buffer
    /// does not.
    fn checkpoint(&mut self, record: &ResumeRecord) -> bool {
        if !self.staging || self.resume_next == u32::MAX || self.checkpoint_interval() == 0 {
            return false;
        }
        let base = self.resume_base();
        if self.resume_next == 0 {
            // A failed write may have programmed part of the header: never
            // program it again in this session.
            self.resume_next = u32::MAX;
            if self
                .flash
                .write(base, &Self::resume_header(record))
                .is_err()
            {
                return false;
            }
            self.resume_next = RESUME_HEADER_LEN;
        }
        if self.resume_next + RESUME_ENTRY_LEN > self.resume_end() {
            return false;
        }
        let staged = record.staged.min(self.written - self.stage_len as u32);
        let mut entry = [0u8; RESUME_ENTRY_LEN as usize];
        entry[..4].copy_from_slice(&staged.to_le_bytes());
        entry[4..].copy_from_slice(&(!staged).to_le_bytes());
        let address = base + self.resume_next;
        self.resume_next += RESUME_ENTRY_LEN;
        self.flash.write(address, &entry).is_ok()
    }

    fn resume(&mut self) -> Option<ResumeRecord> {
        if self.checkpoint_interval() == 0 {
            return None;
        }
        let (mut record, next) = self.read_resume()?;
        // Continue at the start of the sector holding the last recorded
        // byte; that sector is erased again before it is rewritten.
        let sector = self.layout.sector_size;
        let aligned = record.staged / sector * sector;
        if aligned == 0 || aligned > self.trailer_offset() || aligned >= record.firmware_size {
            return None;
        }
        self.reset();
        self.written = aligned;
        self.erased_until = aligned;
        self.staging = true;
        self.resume_next = next;
        record.staged = aligned;
        Some(record)
    }

    /// Append the confirm record if the running image is on trial; no
    /// flash write otherwise.
    fn confirm_image(&mut self) -> Result<(), FirmwareError> {
        confirm(&mut self.flash, &self.layout, self.policy.mode)
            .map(|_| ())
            .map_err(|_| FirmwareError::WriteFailed)
    }
}

impl<F: NorFlash> SlotWriter<F> {
    /// The trial state of the running image.
    pub fn trial_state(&mut self) -> Option<TrialState> {
        trial_state(&mut self.flash, &self.layout, self.policy.mode).ok()
    }
}
