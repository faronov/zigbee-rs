//! Firmware writer abstraction for OTA upgrades.
//!
//! Each platform implements `FirmwareWriter` for its specific flash hardware.
//! The OTA engine calls these methods to write downloaded firmware blocks.

/// Errors from firmware write operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirmwareError {
    /// Flash erase failed.
    EraseFailed,
    /// Flash write failed.
    WriteFailed,
    /// Verification failed (hash mismatch or size mismatch).
    VerifyFailed,
    /// Offset is out of range for the firmware slot.
    OutOfRange,
    /// The firmware slot is not large enough for the image.
    ImageTooLarge,
    /// Activation failed (e.g., boot flag not set).
    ActivateFailed,
    /// Hardware error.
    HardwareError,
}

/// Download progress a writer can keep across a reset, so an interrupted OTA
/// download continues where the staged data ends (see
/// [`FirmwareWriter::checkpoint`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResumeRecord {
    /// Offered image identity (Query Next Image Response).
    pub manufacturer_code: u16,
    pub image_type: u16,
    pub file_version: u32,
    /// Total OTA file size.
    pub image_size: u32,
    /// OTA file offset of the Upgrade Image payload.
    pub payload_start: u32,
    /// Upgrade Image payload length.
    pub firmware_size: u32,
    /// Payload bytes durably staged from slot offset 0.
    pub staged: u32,
}

/// Platform-independent firmware writer for OTA upgrades.
///
/// # Platform implementations
/// - **nRF52840**: Secondary flash bank via NVMC
/// - **ESP32**: OTA partition via esp-storage
/// - **BL702**: XIP flash via bl702-pac
/// - **Mock**: RAM buffer for host testing
pub trait FirmwareWriter {
    /// Erase the firmware update slot, preparing it for writes.
    fn erase_slot(&mut self) -> Result<(), FirmwareError>;

    /// Write a block of data at the given offset within the update slot.
    ///
    /// Blocks must be written sequentially (offset = 0, then offset = len1, etc.)
    /// but the OTA engine handles the sequencing.
    fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError>;

    /// Verify the written image integrity.
    ///
    /// Checks that `expected_size` bytes were written. If `expected_hash` is
    /// provided, verifies the image hash matches.
    fn verify(
        &mut self,
        expected_size: u32,
        expected_hash: Option<&[u8]>,
    ) -> Result<(), FirmwareError>;

    /// Mark the new image as pending activation.
    ///
    /// On next reboot, the bootloader will swap to the new image.
    fn activate(&mut self) -> Result<(), FirmwareError>;

    /// Return the maximum image size this slot can hold.
    fn slot_size(&self) -> u32;

    /// Abort an in-progress update and revert to the current image.
    fn abort(&mut self) -> Result<(), FirmwareError>;

    /// Confirm that the running image works, so a bootloader with trial
    /// boots keeps it instead of treating it as failed. Called by the
    /// application once it has end-to-end proof (for example an
    /// acknowledged report after joining). Idempotent; the default, for
    /// bootloaders without a trial, does nothing.
    fn confirm_image(&mut self) -> Result<(), FirmwareError> {
        Ok(())
    }

    /// Staged payload bytes between progress checkpoints kept across a
    /// reset; `0` (the default) keeps no progress, and a reset then restarts
    /// the download at offset 0. The product selects the interval when it
    /// builds the writer; each checkpoint costs the writer one small flash
    /// write, so it bounds wear (never one write per block).
    fn checkpoint_interval(&self) -> u32 {
        0
    }

    /// Record download progress so that [`resume`](Self::resume) can
    /// continue it after a reset. Called each time `record.staged` crosses a
    /// multiple of [`checkpoint_interval`](Self::checkpoint_interval), with a
    /// growing value for one staged image (since the last
    /// [`erase_slot`](Self::erase_slot)). Returns whether the progress was
    /// recorded.
    ///
    /// The record must live with the staged data: erasing or aborting the
    /// slot, and installing the image, must also drop it.
    fn checkpoint(&mut self, record: &ResumeRecord) -> bool {
        let _ = record;
        false
    }

    /// The download recorded by [`checkpoint`](Self::checkpoint), if the
    /// slot still holds it. On `Some`, the writer is positioned to continue
    /// with `write_block(record.staged, ..)`; `staged` may be lower than the
    /// last checkpoint (for example rounded down to an erase sector) and is
    /// never 0. Called once at start-up, before any other write.
    fn resume(&mut self) -> Option<ResumeRecord> {
        None
    }
}

/// Mock firmware writer for host testing — stores data in a RAM buffer.
#[cfg(any(test, feature = "ota"))]
pub struct MockFirmwareWriter {
    /// RAM buffer simulating flash.
    buffer: heapless::Vec<u8, 262144>, // 256KB max
    /// Number of bytes written so far.
    written: u32,
    /// Whether the slot has been erased.
    erased: bool,
    /// Whether the image has been activated.
    activated: bool,
    /// Maximum slot size.
    max_size: u32,
    /// Checkpoint interval; `0` keeps no progress. Checkpoints survive a
    /// manager rebuild, as on flash.
    checkpoint_every: u32,
    /// Last recorded progress.
    checkpoint: Option<ResumeRecord>,
    /// Checkpoints recorded since creation.
    checkpoints: u32,
}

#[cfg(any(test, feature = "ota"))]
impl MockFirmwareWriter {
    /// Create a new mock writer with the given slot size.
    pub fn new(slot_size: u32) -> Self {
        Self {
            buffer: heapless::Vec::new(),
            written: 0,
            erased: false,
            activated: false,
            max_size: slot_size.min(262144),
            checkpoint_every: 0,
            checkpoint: None,
            checkpoints: 0,
        }
    }

    /// A mock writer that keeps download progress every `checkpoint_every`
    /// staged bytes for [`FirmwareWriter::resume`].
    pub fn with_resume(slot_size: u32, checkpoint_every: u32) -> Self {
        Self {
            checkpoint_every,
            ..Self::new(slot_size)
        }
    }

    /// Last recorded progress.
    pub fn last_checkpoint(&self) -> Option<ResumeRecord> {
        self.checkpoint
    }

    /// Checkpoints recorded since creation.
    pub fn checkpoint_count(&self) -> u32 {
        self.checkpoints
    }

    /// Get the written data (for verification in tests).
    pub fn data(&self) -> &[u8] {
        &self.buffer
    }

    /// Check if the image was activated.
    pub fn is_activated(&self) -> bool {
        self.activated
    }

    /// Total bytes written.
    pub fn bytes_written(&self) -> u32 {
        self.written
    }
}

#[cfg(any(test, feature = "ota"))]
impl FirmwareWriter for MockFirmwareWriter {
    fn erase_slot(&mut self) -> Result<(), FirmwareError> {
        self.buffer.clear();
        self.written = 0;
        self.erased = true;
        self.activated = false;
        self.checkpoint = None;
        log::debug!("[MockFW] Slot erased");
        Ok(())
    }

    fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError> {
        if !self.erased {
            return Err(FirmwareError::WriteFailed);
        }
        if offset + data.len() as u32 > self.max_size {
            return Err(FirmwareError::OutOfRange);
        }
        // Ensure we're writing sequentially
        if offset != self.written {
            return Err(FirmwareError::WriteFailed);
        }
        for &b in data {
            self.buffer
                .push(b)
                .map_err(|_| FirmwareError::ImageTooLarge)?;
        }
        self.written += data.len() as u32;
        log::debug!(
            "[MockFW] Write {} bytes at offset {}, total={}",
            data.len(),
            offset,
            self.written
        );
        Ok(())
    }

    fn verify(
        &mut self,
        expected_size: u32,
        _expected_hash: Option<&[u8]>,
    ) -> Result<(), FirmwareError> {
        if self.written != expected_size {
            log::warn!(
                "[MockFW] Size mismatch: written={} expected={}",
                self.written,
                expected_size
            );
            return Err(FirmwareError::VerifyFailed);
        }
        log::debug!("[MockFW] Verify OK: {} bytes", expected_size);
        Ok(())
    }

    fn activate(&mut self) -> Result<(), FirmwareError> {
        self.activated = true;
        log::info!("[MockFW] Image activated — reboot to apply");
        Ok(())
    }

    fn slot_size(&self) -> u32 {
        self.max_size
    }

    fn abort(&mut self) -> Result<(), FirmwareError> {
        self.buffer.clear();
        self.written = 0;
        self.erased = false;
        self.activated = false;
        self.checkpoint = None;
        log::info!("[MockFW] OTA aborted");
        Ok(())
    }

    fn checkpoint_interval(&self) -> u32 {
        self.checkpoint_every
    }

    fn checkpoint(&mut self, record: &ResumeRecord) -> bool {
        if self.checkpoint_every == 0 || !self.erased || record.staged > self.written {
            return false;
        }
        self.checkpoint = Some(*record);
        self.checkpoints += 1;
        true
    }

    /// Continue exactly at the last checkpoint; staged bytes after it are
    /// dropped.
    fn resume(&mut self) -> Option<ResumeRecord> {
        let record = self.checkpoint.filter(|record| record.staged > 0)?;
        self.buffer.truncate(record.staged as usize);
        self.written = record.staged;
        self.erased = true;
        self.activated = false;
        Some(record)
    }
}
