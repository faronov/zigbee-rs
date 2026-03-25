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
    fn verify(&self, expected_size: u32, expected_hash: Option<&[u8]>)
    -> Result<(), FirmwareError>;

    /// Mark the new image as pending activation.
    ///
    /// On next reboot, the bootloader will swap to the new image.
    fn activate(&mut self) -> Result<(), FirmwareError>;

    /// Return the maximum image size this slot can hold.
    fn slot_size(&self) -> u32;

    /// Abort an in-progress update and revert to the current image.
    fn abort(&mut self) -> Result<(), FirmwareError>;
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
        }
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
        &self,
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
        log::info!("[MockFW] OTA aborted");
        Ok(())
    }
}
