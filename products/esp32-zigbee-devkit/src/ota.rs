//! [`FirmwareWriter`] implementation that stages a Zigbee OTA payload into the
//! inactive ESP-IDF application slot.
//!
//! # How an upgrade runs
//!
//! 1. Live MMU evidence identifies the executing application's slot. The other
//!    slot becomes the staging target. `otadata` is only a boot preference:
//!    the bootloader can fall back without rewriting it. Missing evidence is
//!    an explicit error, even when metadata is erased.
//! 2. [`FirmwareWriter::erase_slot`] is bookkeeping only. Erasing 1.9 MiB up
//!    front would hold a critical section for many seconds — `esp-storage`
//!    masks interrupts around every ROM flash call — and the radio would miss
//!    every parent poll in the meantime. Instead each 4 KiB sector is erased
//!    lazily, immediately before the first byte lands in it, which spreads the
//!    erase cost across the download at ~1 sector per 4 KiB of payload.
//! 3. Zigbee may deliver ragged blocks, but ESP flash programs 4-byte words.
//!    Sub-word tails are buffered in
//!    RAM until the next block completes the word; the final partial word is
//!    padded with `0xFF` — the erased value — so no byte of the image is ever
//!    altered and the padding lives past the end of the image.
//! 4. [`FirmwareWriter::verify`] re-reads and validates the complete ESP image
//!    structure, chip compatibility, XOR checksum and appended SHA-256.
//!    This is integrity checking, NOT secure-boot authentication.
//! 5. [`FirmwareWriter::activate`] writes one 32-byte `otadata` entry into the
//!    sector that does not hold the active entry and resets the chip. A power
//!    failure leaves the previous entry intact and never erases the executing
//!    firmware. An old preference may already refer to a failed image; it is
//!    not proof of which fallback the bootloader will try on the next reset.

use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

use crate::esp_image::{DIGEST_SIZE, ImageCompatibility, ImageReadError, verify_image};
use crate::layout::{
    EXPECTED_PARTITIONS, OTA_SLOT_SIZE, OTADATA_OFFSET, PARTITION_ENTRY_SIZE,
    PARTITION_TABLE_OFFSET, SECTOR_SIZE, WORD_SIZE, ota_slot_offset, otadata_sector_offset,
};
use crate::otadata::{ENTRY_SIZE, OtaData, OtaSelectEntry};

#[path = "running_image.rs"]
mod running_image;
pub use running_image::RunningImageError;

/// Initialization failures are distinguishable before any erase/write occurs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaInitError {
    Flash(FirmwareError),
    RunningImage(RunningImageError),
}

impl From<FirmwareError> for OtaInitError {
    fn from(error: FirmwareError) -> Self {
        Self::Flash(error)
    }
}

impl From<RunningImageError> for OtaInitError {
    fn from(error: RunningImageError) -> Self {
        Self::RunningImage(error)
    }
}

/// Raw flash access needed to stage an image.
///
/// Deliberately narrower than `embedded-storage`: addresses are absolute flash
/// offsets, writes are word aligned and erases are whole sectors, which is what
/// both the ROM routines and the host mock implement.
pub trait OtaFlash {
    /// Actual executing-image evidence (e.g. live MMU translation), never an
    /// otadata preference, image-version comparison, or an assumed slot zero.
    /// The default deliberately disables OTA on backends without evidence.
    fn running_slot(&mut self) -> Result<u8, RunningImageError> {
        Err(RunningImageError::Unavailable)
    }

    /// Read actual silicon/eFuse revisions for image compatibility checks.
    fn image_compatibility(&mut self) -> Result<ImageCompatibility, FirmwareError> {
        Err(FirmwareError::HardwareError)
    }

    /// Read `buffer.len()` bytes starting at `address`.
    fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FirmwareError>;

    /// Program `data` at `address`. Both must be [`WORD_SIZE`] aligned.
    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FirmwareError>;

    /// Erase the 4 KiB sector starting at `address`.
    fn erase_sector(&mut self, address: u32) -> Result<(), FirmwareError>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// No transfer in progress.
    Idle,
    /// Accepting blocks.
    Staging,
    /// Image checked against its own SHA-256, ready to activate.
    Verified,
}

/// Stages OTA payloads into the inactive `ota_0`/`ota_1` slot.
pub struct EspFirmwareWriter<F: OtaFlash> {
    flash: F,
    reset: fn() -> !,
    running_slot: u8,
    target_slot: u8,
    state: State,
    /// Payload bytes accepted from the OTA engine.
    written: u32,
    /// Payload bytes already programmed into flash.
    flushed: u32,
    /// Sub-word remainder waiting for the next block.
    tail: [u8; WORD_SIZE as usize],
    tail_len: usize,
    /// Number of leading sectors of the slot that have been erased.
    sectors_erased: u32,
    /// Set once the padded final word has been programmed.
    finalized: bool,
}

impl<F: OtaFlash> EspFirmwareWriter<F> {
    /// Create a writer using actual executing-image evidence, not `otadata`.
    ///
    /// `reset` performs the software reset that hands control back to the
    /// bootloader; it is only called from [`FirmwareWriter::activate`], after
    /// the new `otadata` entry has been programmed and read back.
    pub fn new(mut flash: F, reset: fn() -> !) -> Result<Self, OtaInitError> {
        let running_slot = flash.running_slot()?;
        if running_slot >= crate::layout::OTA_SLOT_COUNT {
            return Err(RunningImageError::OutsideOtaSlots.into());
        }
        let mut writer = Self {
            flash,
            reset,
            running_slot,
            target_slot: (running_slot + 1) % crate::layout::OTA_SLOT_COUNT,
            state: State::Idle,
            written: 0,
            flushed: 0,
            tail: [0xFF; WORD_SIZE as usize],
            tail_len: 0,
            sectors_erased: 0,
            finalized: false,
        };
        writer.validate_partition_table()?;
        Ok(writer)
    }

    /// Slot the bootloader started this firmware from.
    pub fn running_slot(&self) -> u8 {
        self.running_slot
    }

    /// Slot the next update is staged into.
    pub fn target_slot(&self) -> u8 {
        self.target_slot
    }

    /// Payload bytes accepted so far.
    pub fn bytes_written(&self) -> u32 {
        self.written
    }

    /// Sectors erased so far in the staging slot.
    pub fn sectors_erased(&self) -> u32 {
        self.sectors_erased
    }

    /// Program the `otadata` entry that selects the staged slot.
    ///
    /// [`FirmwareWriter::activate`] is this plus a software reset; keeping them
    /// apart is what makes the activation path testable on the host.
    pub fn stage_activation(&mut self) -> Result<(), FirmwareError> {
        if self.state != State::Verified {
            return Err(FirmwareError::ActivateFailed);
        }

        self.check_running_image()
            .map_err(|_| FirmwareError::ActivateFailed)?;
        // Revalidate before committing boot preference, including flash errors
        // or corruption occurring after verify().
        self.verify_staged_image(None)?;
        let data = self.read_otadata()?;

        let activation = data
            .activation_for(self.target_slot)
            .map_err(|_| FirmwareError::ActivateFailed)?;
        let address = otadata_sector_offset(activation.sector);
        let encoded = activation.entry.encode();

        self.flash.erase_sector(address)?;
        self.flash.write(address, &encoded)?;

        let mut readback = [0u8; ENTRY_SIZE];
        self.flash.read(address, &mut readback)?;
        if OtaSelectEntry::decode(&readback) != activation.entry {
            return Err(FirmwareError::ActivateFailed);
        }

        let confirmed = self.read_otadata()?;
        if confirmed.active_slot() != Some(self.target_slot) {
            return Err(FirmwareError::ActivateFailed);
        }

        log::info!(
            "[ESP OTA] otadata sector {} -> seq {} (slot {})",
            activation.sector,
            activation.entry.seq,
            self.target_slot
        );
        Ok(())
    }

    fn read_otadata(&mut self) -> Result<OtaData, FirmwareError> {
        let mut first = [0u8; ENTRY_SIZE];
        let mut second = [0u8; ENTRY_SIZE];
        self.flash.read(OTADATA_OFFSET, &mut first)?;
        self.flash.read(OTADATA_OFFSET + SECTOR_SIZE, &mut second)?;
        Ok(OtaData::decode([&first, &second]))
    }

    fn validate_partition_table(&mut self) -> Result<(), FirmwareError> {
        let mut entry = [0u8; PARTITION_ENTRY_SIZE];
        for (index, expected) in EXPECTED_PARTITIONS.iter().copied().enumerate() {
            let address = PARTITION_TABLE_OFFSET + (index * PARTITION_ENTRY_SIZE) as u32;
            self.flash.read(address, &mut entry)?;
            if !expected.matches(&entry) {
                log::error!(
                    "[ESP OTA] partition table entry {} does not match the required OTA layout",
                    index
                );
                return Err(FirmwareError::HardwareError);
            }
        }
        Ok(())
    }

    fn check_running_image(&mut self) -> Result<(), RunningImageError> {
        if self.flash.running_slot()? != self.running_slot {
            return Err(RunningImageError::Changed);
        }
        Ok(())
    }

    fn slot_base(&self) -> u32 {
        ota_slot_offset(self.target_slot)
    }

    fn reset_staging(&mut self) {
        self.state = State::Idle;
        self.written = 0;
        self.flushed = 0;
        self.tail = [0xFF; WORD_SIZE as usize];
        self.tail_len = 0;
        self.sectors_erased = 0;
        self.finalized = false;
    }

    fn ensure_sector_erased(&mut self, sector: u32) -> Result<(), FirmwareError> {
        while self.sectors_erased <= sector {
            let address = self.slot_base() + self.sectors_erased * SECTOR_SIZE;
            self.flash.erase_sector(address)?;
            self.sectors_erased += 1;
        }
        Ok(())
    }

    /// Program word-aligned data at the current flush cursor, erasing sectors
    /// as they are reached.
    fn commit(&mut self, data: &[u8]) -> Result<(), FirmwareError> {
        debug_assert_eq!(data.len() % WORD_SIZE as usize, 0);
        debug_assert_eq!(self.flushed % WORD_SIZE, 0);

        let mut cursor = self.flushed;
        let mut rest = data;
        while !rest.is_empty() {
            let sector = cursor / SECTOR_SIZE;
            self.ensure_sector_erased(sector)?;
            let room = ((sector + 1) * SECTOR_SIZE - cursor) as usize;
            let take = room.min(rest.len());
            self.flash.write(self.slot_base() + cursor, &rest[..take])?;
            cursor += take as u32;
            rest = &rest[take..];
        }
        self.flushed = cursor;
        Ok(())
    }

    /// Program the last, partially filled word, padded with erased bytes.
    fn flush_tail(&mut self) -> Result<(), FirmwareError> {
        if self.tail_len > 0 {
            let mut word = [0xFFu8; WORD_SIZE as usize];
            word[..self.tail_len].copy_from_slice(&self.tail[..self.tail_len]);
            self.commit(&word)?;
            self.tail_len = 0;
        }
        self.finalized = true;
        Ok(())
    }

    fn verify_staged_image(&mut self, expected_hash: Option<&[u8]>) -> Result<(), FirmwareError> {
        let compatibility = self.flash.image_compatibility()?;
        let base = self.slot_base();
        let digest = verify_image(self.written, base, compatibility, |offset, buffer| {
            self.flash.read(base + offset, buffer)
        })
        .map_err(|error| match error {
            ImageReadError::Read(error) => error,
            ImageReadError::Image(error) => {
                log::warn!("[ESP OTA] staged image rejected: {:?}", error);
                FirmwareError::VerifyFailed
            }
        })?;

        if let Some(expected) = expected_hash
            && (expected.len() != DIGEST_SIZE || expected != digest)
        {
            log::warn!("[ESP OTA] staged image does not match the expected hash");
            return Err(FirmwareError::VerifyFailed);
        }

        Ok(())
    }
}

impl<F: OtaFlash> FirmwareWriter for EspFirmwareWriter<F> {
    /// Prepare for a download. No flash is erased here; see the module docs.
    fn erase_slot(&mut self) -> Result<(), FirmwareError> {
        self.reset_staging();
        self.check_running_image().map_err(|error| {
            log::error!("[ESP OTA] running-image evidence lost: {:?}", error);
            FirmwareError::HardwareError
        })?;
        self.state = State::Staging;
        log::info!(
            "[ESP OTA] staging into slot {} (running slot {})",
            self.target_slot,
            self.running_slot
        );
        Ok(())
    }

    fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError> {
        if self.state != State::Staging || self.finalized {
            return Err(FirmwareError::WriteFailed);
        }
        if offset != self.written {
            // The OTA engine writes strictly sequentially; anything else means
            // a lost or reordered block and the staged image cannot be trusted.
            return Err(FirmwareError::WriteFailed);
        }
        let length = u32::try_from(data.len()).map_err(|_| FirmwareError::OutOfRange)?;
        let end = offset
            .checked_add(length)
            .ok_or(FirmwareError::OutOfRange)?;
        if end > OTA_SLOT_SIZE {
            return Err(FirmwareError::OutOfRange);
        }

        let mut rest = data;

        if self.tail_len > 0 {
            let take = (WORD_SIZE as usize - self.tail_len).min(rest.len());
            self.tail[self.tail_len..self.tail_len + take].copy_from_slice(&rest[..take]);
            self.tail_len += take;
            rest = &rest[take..];
            if self.tail_len == WORD_SIZE as usize {
                let word = self.tail;
                self.commit(&word)?;
                self.tail_len = 0;
            }
            // Otherwise the block ended inside the word; keep buffering.
        }

        if !rest.is_empty() {
            debug_assert_eq!(self.tail_len, 0);
            let whole = rest.len() - rest.len() % WORD_SIZE as usize;
            if whole > 0 {
                self.commit(&rest[..whole])?;
            }
            let remainder = &rest[whole..];
            self.tail[..remainder.len()].copy_from_slice(remainder);
            self.tail_len = remainder.len();
        }

        self.written = end;
        Ok(())
    }

    fn verify(
        &mut self,
        expected_size: u32,
        expected_hash: Option<&[u8]>,
    ) -> Result<(), FirmwareError> {
        if self.state != State::Staging {
            return Err(FirmwareError::VerifyFailed);
        }
        self.flush_tail()?;
        if self.written != expected_size {
            log::warn!(
                "[ESP OTA] size mismatch: staged {} bytes, expected {}",
                self.written,
                expected_size
            );
            return Err(FirmwareError::VerifyFailed);
        }
        self.verify_staged_image(expected_hash)?;
        self.state = State::Verified;
        log::info!(
            "[ESP OTA] slot {} verified ({} bytes)",
            self.target_slot,
            self.written
        );
        Ok(())
    }

    /// Select the staged slot and reboot into it. Does not return on hardware.
    fn activate(&mut self) -> Result<(), FirmwareError> {
        self.stage_activation()?;
        (self.reset)()
    }

    fn slot_size(&self) -> u32 {
        OTA_SLOT_SIZE
    }

    /// Drop the partially staged image. `otadata` is untouched, so the running
    /// slot stays selected.
    fn abort(&mut self) -> Result<(), FirmwareError> {
        if self.state != State::Idle {
            log::info!("[ESP OTA] aborted after {} bytes", self.written);
        }
        self.reset_staging();
        Ok(())
    }
}

// ── Hardware backing ────────────────────────────────────────────────────────

/// [`OtaFlash`] backed by the board's raw whole-chip flash access.
#[cfg(target_os = "none")]
pub struct EspOtaFlash {
    flash: esp32_zigbee_devkit::flash::RawFlash,
}

#[cfg(target_os = "none")]
impl EspOtaFlash {
    /// Open the on-board SPI flash.
    pub fn new() -> Self {
        Self {
            flash: esp32_zigbee_devkit::flash::RawFlash::new(),
        }
    }
}

#[cfg(target_os = "none")]
impl Default for EspOtaFlash {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(target_os = "none")]
impl OtaFlash for EspOtaFlash {
    fn running_slot(&mut self) -> Result<u8, RunningImageError> {
        running_image::detect()
    }

    fn image_compatibility(&mut self) -> Result<ImageCompatibility, FirmwareError> {
        use esp_hal::efuse::Efuse;
        let (major, minor) = Efuse::block_version();
        Ok(ImageCompatibility {
            chip_id: crate::esp_image::EXPECTED_CHIP_ID,
            chip_revision: Efuse::chip_revision(),
            efuse_block_revision: major as u16 * 100 + minor as u16,
        })
    }

    fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FirmwareError> {
        use embedded_storage::nor_flash::ReadNorFlash;
        self.flash
            .read(address, buffer)
            .map_err(|_| FirmwareError::HardwareError)
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FirmwareError> {
        use embedded_storage::nor_flash::NorFlash;

        use crate::layout::is_ota_writable;
        let length = u32::try_from(data.len()).map_err(|_| FirmwareError::OutOfRange)?;
        if !is_ota_writable(address, length) {
            return Err(FirmwareError::OutOfRange);
        }
        self.flash
            .write(address, data)
            .map_err(|_| FirmwareError::WriteFailed)
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), FirmwareError> {
        use embedded_storage::nor_flash::NorFlash;

        use crate::layout::is_ota_writable;
        if !is_ota_writable(address, SECTOR_SIZE) {
            return Err(FirmwareError::OutOfRange);
        }
        self.flash
            .erase(address, address + SECTOR_SIZE)
            .map_err(|_| FirmwareError::EraseFailed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::esp_image::tests::{application as esp_image, compatibility, rehash};
    use crate::esp_image::{EXPECTED_CHIP_ID, HEADER_SIZE, IMAGE_MAGIC};
    use crate::layout::{FLASH_SIZE, OTA_0_OFFSET, OTA_1_OFFSET, is_ota_writable};
    use crate::otadata::STATE_VALID;
    use crate::sha256::sha256;

    /// NOR-like flash mock: bits only go 1 -> 0, writes are word aligned and
    /// every access is bounds checked against the partition table.
    struct MockFlash {
        data: Vec<u8>,
        erased: Vec<u32>,
        writes: Vec<(u32, usize)>,
        fail_write_at: Option<u32>,
        running: Result<u8, RunningImageError>,
    }

    impl MockFlash {
        fn erased() -> Self {
            Self {
                data: vec![0xFF; FLASH_SIZE as usize],
                erased: Vec::new(),
                writes: Vec::new(),
                fail_write_at: None,
                running: Ok(0),
            }
        }

        fn new() -> Self {
            let mut flash = Self::erased();
            for (index, partition) in EXPECTED_PARTITIONS.iter().copied().enumerate() {
                let start = PARTITION_TABLE_OFFSET as usize + index * PARTITION_ENTRY_SIZE;
                flash.data[start..start + PARTITION_ENTRY_SIZE]
                    .copy_from_slice(&partition.encode());
            }
            flash
        }

        fn with_otadata(seq: u32, sector: u8) -> Self {
            let mut flash = Self::new();
            let entry = OtaSelectEntry::new(seq, STATE_VALID).encode();
            let base = otadata_sector_offset(sector) as usize;
            flash.data[base..base + ENTRY_SIZE].copy_from_slice(&entry);
            flash
        }

        fn slice(&self, address: u32, len: usize) -> &[u8] {
            &self.data[address as usize..address as usize + len]
        }
    }

    impl OtaFlash for MockFlash {
        fn running_slot(&mut self) -> Result<u8, RunningImageError> {
            self.running
        }

        fn image_compatibility(&mut self) -> Result<ImageCompatibility, FirmwareError> {
            Ok(compatibility())
        }

        fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FirmwareError> {
            let end = address as usize + buffer.len();
            assert!(end <= self.data.len(), "read out of flash");
            buffer.copy_from_slice(&self.data[address as usize..end]);
            Ok(())
        }

        fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FirmwareError> {
            assert_eq!(
                address % WORD_SIZE,
                0,
                "unaligned write offset {address:#X}"
            );
            assert_eq!(data.len() % WORD_SIZE as usize, 0, "unaligned write length");
            assert!(
                is_ota_writable(address, data.len() as u32),
                "write outside the OTA partitions at {address:#X}"
            );
            if self.fail_write_at == Some(address) {
                return Err(FirmwareError::WriteFailed);
            }
            for (index, byte) in data.iter().enumerate() {
                let slot = &mut self.data[address as usize + index];
                assert_eq!(*slot, 0xFF, "write to un-erased byte at {address:#X}");
                *slot = *byte;
            }
            self.writes.push((address, data.len()));
            Ok(())
        }

        fn erase_sector(&mut self, address: u32) -> Result<(), FirmwareError> {
            assert_eq!(address % SECTOR_SIZE, 0, "unaligned erase");
            assert!(
                is_ota_writable(address, SECTOR_SIZE),
                "erase outside the OTA partitions at {address:#X}"
            );
            let start = address as usize;
            self.data[start..start + SECTOR_SIZE as usize].fill(0xFF);
            self.erased.push(address);
            Ok(())
        }
    }

    fn never_resets() -> ! {
        panic!("reset must not be called from a host test");
    }

    fn new_writer(flash: MockFlash) -> EspFirmwareWriter<MockFlash> {
        EspFirmwareWriter::new(flash, never_resets).expect("writer")
    }

    fn stage(
        writer: &mut EspFirmwareWriter<MockFlash>,
        image: &[u8],
        block: usize,
    ) -> Result<(), FirmwareError> {
        writer.erase_slot()?;
        let mut offset = 0u32;
        for chunk in image.chunks(block) {
            writer.write_block(offset, chunk)?;
            offset += chunk.len() as u32;
        }
        writer.verify(image.len() as u32, None)
    }

    #[test]
    fn actual_slot_zero_stages_into_slot_one() {
        let writer = new_writer(MockFlash::new());
        assert_eq!(writer.running_slot(), 0);
        assert_eq!(writer.target_slot(), 1);
        assert_eq!(writer.slot_size(), OTA_SLOT_SIZE);
    }

    #[test]
    fn rejects_flash_without_the_required_partition_table() {
        assert!(matches!(
            EspFirmwareWriter::new(MockFlash::erased(), never_resets),
            Err(OtaInitError::Flash(FirmwareError::HardwareError))
        ));
    }

    #[test]
    fn running_slot_one_stages_into_slot_zero() {
        let mut flash = MockFlash::new();
        flash.running = Ok(1); // evidence, even with ERASED metadata
        let writer = new_writer(flash);
        assert_eq!(writer.running_slot(), 1);
        assert_eq!(writer.target_slot(), 0);
    }

    #[test]
    fn writes_are_rejected_before_erase_slot() {
        let mut writer = new_writer(MockFlash::new());
        assert_eq!(
            writer.write_block(0, &[1, 2, 3, 4]),
            Err(FirmwareError::WriteFailed)
        );
    }

    #[test]
    fn out_of_order_and_oversized_blocks_are_rejected() {
        let mut writer = new_writer(MockFlash::new());
        writer.erase_slot().unwrap();
        writer.write_block(0, &[0u8; 48]).unwrap();

        assert_eq!(
            writer.write_block(0, &[0u8; 48]),
            Err(FirmwareError::WriteFailed),
            "replayed offset"
        );
        assert_eq!(
            writer.write_block(96, &[0u8; 48]),
            Err(FirmwareError::WriteFailed),
            "gap in the stream"
        );

        let mut writer = new_writer(MockFlash::new());
        writer.erase_slot().unwrap();
        writer.write_block(0, &[0u8; 4]).unwrap();
        assert_eq!(
            writer.write_block(4, &vec![0u8; OTA_SLOT_SIZE as usize]),
            Err(FirmwareError::OutOfRange)
        );
    }

    #[test]
    fn ragged_blocks_are_reassembled_byte_exactly() {
        // ESP images are 16-byte aligned, but transport blocks need not be.
        let image = esp_image(EXPECTED_CHIP_ID, 501);
        let mut writer = new_writer(MockFlash::new());
        stage(&mut writer, &image, 47).expect("staged");

        let staged = writer.flash.slice(OTA_1_OFFSET, image.len());
        assert_eq!(staged, image.as_slice());

        // The padding of the final word never reaches beyond the image.
        let padding = writer.flash.slice(OTA_1_OFFSET + image.len() as u32, 4);
        assert!(padding.iter().all(|byte| *byte == 0xFF));
    }

    #[test]
    fn every_block_size_reproduces_the_image() {
        let image = esp_image(EXPECTED_CHIP_ID, 4096 + 7);
        for block in [1usize, 3, 4, 5, 48, 64, 1023, 4096] {
            let mut writer = new_writer(MockFlash::new());
            stage(&mut writer, &image, block).unwrap_or_else(|e| panic!("block {block}: {e:?}"));
            assert_eq!(
                writer.flash.slice(OTA_1_OFFSET, image.len()),
                image.as_slice(),
                "block size {block}"
            );
        }
    }

    #[test]
    fn sectors_are_erased_lazily_and_only_where_data_lands() {
        let image = esp_image(EXPECTED_CHIP_ID, 4096); // spans two sectors
        let mut writer = new_writer(MockFlash::new());

        writer.erase_slot().unwrap();
        assert!(
            writer.flash.erased.is_empty(),
            "erase_slot() must not touch flash"
        );

        writer.write_block(0, &image[..48]).unwrap();
        assert_eq!(writer.flash.erased, vec![OTA_1_OFFSET], "first sector only");

        let mut offset = 48u32;
        for chunk in image[48..].chunks(48) {
            writer.write_block(offset, chunk).unwrap();
            offset += chunk.len() as u32;
        }
        writer.verify(image.len() as u32, None).unwrap();

        let expected: Vec<u32> = (0..2).map(|i| OTA_1_OFFSET + i * SECTOR_SIZE).collect();
        assert_eq!(writer.flash.erased, expected);
        assert_eq!(writer.sectors_erased(), 2);
    }

    #[test]
    fn verify_rejects_size_mismatch_and_foreign_images() {
        let image = esp_image(EXPECTED_CHIP_ID, 200);

        let mut writer = new_writer(MockFlash::new());
        writer.erase_slot().unwrap();
        writer.write_block(0, &image).unwrap();
        assert_eq!(
            writer.verify(image.len() as u32 + 1, None),
            Err(FirmwareError::VerifyFailed)
        );

        let foreign_chip = if EXPECTED_CHIP_ID == crate::esp_image::CHIP_ID_ESP32C6 {
            crate::esp_image::CHIP_ID_ESP32H2
        } else {
            crate::esp_image::CHIP_ID_ESP32C6
        };
        let foreign = esp_image(foreign_chip, 200);
        let mut writer = new_writer(MockFlash::new());
        assert_eq!(
            stage(&mut writer, &foreign, 48),
            Err(FirmwareError::VerifyFailed)
        );

        let mut truncated = esp_image(EXPECTED_CHIP_ID, 200);
        truncated.truncate(HEADER_SIZE + DIGEST_SIZE - 1);
        let mut writer = new_writer(MockFlash::new());
        assert_eq!(
            stage(&mut writer, &truncated, 48),
            Err(FirmwareError::VerifyFailed)
        );
    }

    #[test]
    fn verify_rejects_a_corrupted_payload() {
        let mut image = esp_image(EXPECTED_CHIP_ID, 300);
        let last = image.len() - DIGEST_SIZE - 1;
        image[last] ^= 0x01; // flip a payload bit, keep the stored digest

        let mut writer = new_writer(MockFlash::new());
        assert_eq!(
            stage(&mut writer, &image, 48),
            Err(FirmwareError::VerifyFailed)
        );
    }

    #[test]
    fn verify_honours_an_externally_supplied_hash() {
        let image = esp_image(EXPECTED_CHIP_ID, 128);
        let digest = sha256(&image[..image.len() - DIGEST_SIZE]);

        let mut writer = new_writer(MockFlash::new());
        writer.erase_slot().unwrap();
        writer.write_block(0, &image).unwrap();
        assert!(writer.verify(image.len() as u32, Some(&digest)).is_ok());

        let mut writer = new_writer(MockFlash::new());
        writer.erase_slot().unwrap();
        writer.write_block(0, &image).unwrap();
        assert_eq!(
            writer.verify(image.len() as u32, Some(&[0u8; 32])),
            Err(FirmwareError::VerifyFailed)
        );
    }

    #[test]
    fn activation_requires_a_verified_image() {
        let mut writer = new_writer(MockFlash::new());
        assert_eq!(
            writer.stage_activation(),
            Err(FirmwareError::ActivateFailed)
        );

        let image = esp_image(EXPECTED_CHIP_ID, 64);
        writer.erase_slot().unwrap();
        writer.write_block(0, &image).unwrap();
        assert_eq!(
            writer.stage_activation(),
            Err(FirmwareError::ActivateFailed),
            "verify() has not run yet"
        );
    }

    #[test]
    fn activation_selects_the_staged_slot_without_touching_the_active_entry() {
        // Sector 0 holds the entry that boots slot 0.
        let flash = MockFlash::with_otadata(1, 0);
        let active_before = flash.slice(otadata_sector_offset(0), ENTRY_SIZE).to_vec();

        let image = esp_image(EXPECTED_CHIP_ID, 512);
        let mut writer = new_writer(flash);
        stage(&mut writer, &image, 48).unwrap();
        writer.stage_activation().unwrap();

        assert_eq!(
            writer.flash.slice(otadata_sector_offset(0), ENTRY_SIZE),
            active_before.as_slice(),
            "the active otadata entry must survive the activation"
        );

        let mut first = [0u8; ENTRY_SIZE];
        let mut second = [0u8; ENTRY_SIZE];
        first.copy_from_slice(writer.flash.slice(otadata_sector_offset(0), ENTRY_SIZE));
        second.copy_from_slice(writer.flash.slice(otadata_sector_offset(1), ENTRY_SIZE));
        let data = OtaData::decode([&first, &second]);
        assert_eq!(data.active_slot(), Some(1));
        assert_eq!(data.active_sector(), Some(1));
    }

    #[test]
    fn abort_leaves_the_active_slot_selected() {
        let flash = MockFlash::with_otadata(1, 0);
        let otadata_before = flash.slice(OTADATA_OFFSET, ENTRY_SIZE * 2).to_vec();

        let image = esp_image(EXPECTED_CHIP_ID, 4096);
        let mut writer = new_writer(flash);
        writer.erase_slot().unwrap();
        writer.write_block(0, &image[..2048]).unwrap();
        writer.abort().unwrap();

        assert_eq!(
            writer.flash.slice(OTADATA_OFFSET, ENTRY_SIZE * 2),
            otadata_before.as_slice(),
            "abort must not change the boot selection"
        );
        assert_eq!(writer.bytes_written(), 0);
        assert_eq!(
            writer.stage_activation(),
            Err(FirmwareError::ActivateFailed)
        );

        // A fresh transfer works after an abort.
        stage(&mut writer, &image, 48).unwrap();
        assert_eq!(writer.flash.slice(OTA_1_OFFSET, image.len()), image);
    }

    #[test]
    fn the_running_slot_is_never_written() {
        let image = esp_image(EXPECTED_CHIP_ID, 8192);
        let mut writer = new_writer(MockFlash::with_otadata(1, 0)); // running slot 0
        stage(&mut writer, &image, 48).unwrap();
        writer.stage_activation().unwrap();

        for (address, length) in &writer.flash.writes {
            let within_running_slot =
                *address >= OTA_0_OFFSET && *address < OTA_0_OFFSET + OTA_SLOT_SIZE;
            assert!(
                !within_running_slot,
                "wrote {length} bytes into the running slot at {address:#X}"
            );
        }
        for address in &writer.flash.erased {
            assert!(
                !(*address >= OTA_0_OFFSET && *address < OTA_0_OFFSET + OTA_SLOT_SIZE),
                "erased the running slot at {address:#X}"
            );
        }
    }

    #[test]
    fn a_failing_flash_write_is_reported() {
        let mut flash = MockFlash::new();
        flash.fail_write_at = Some(OTA_1_OFFSET);
        let mut writer = new_writer(flash);
        writer.erase_slot().unwrap();
        assert_eq!(
            writer.write_block(0, &[0u8; 64]),
            Err(FirmwareError::WriteFailed)
        );
    }

    #[test]
    fn no_writes_are_accepted_after_verification() {
        let image = esp_image(EXPECTED_CHIP_ID, 100);
        let mut writer = new_writer(MockFlash::new());
        stage(&mut writer, &image, 48).unwrap();
        assert_eq!(
            writer.write_block(image.len() as u32, &[0u8; 4]),
            Err(FirmwareError::WriteFailed)
        );
    }

    #[test]
    fn bootloader_fallback_never_erases_the_executing_slot() {
        // IDF 5.5.1 can fail the preferred image and fall back WITHOUT
        // rewriting valid, non-erased otadata. Exercise both directions,
        // erased metadata, and a restart after activation without a reset.
        for actual in 0..2 {
            for preferred_seq in [None, Some(1), Some(2)] {
                let mut flash = match preferred_seq {
                    Some(seq) => MockFlash::with_otadata(seq, 0),
                    None => MockFlash::new(),
                };
                flash.running = Ok(actual);
                let base = ota_slot_offset(actual) as usize;
                flash.data[base..base + OTA_SLOT_SIZE as usize].fill(0xA5);
                let mut writer = new_writer(flash);
                assert_eq!(writer.running_slot(), actual);
                assert_eq!(writer.target_slot(), 1 - actual);
                let image = esp_image(EXPECTED_CHIP_ID, 8192);
                for _ in 0..2 {
                    stage(&mut writer, &image, 47).unwrap();
                    writer.stage_activation().unwrap();
                }
                assert!(
                    writer.flash.data[base..base + OTA_SLOT_SIZE as usize]
                        .iter()
                        .all(|byte| *byte == 0xA5)
                );
                for address in &writer.flash.erased {
                    assert!(!(*address >= base as u32 && *address < base as u32 + OTA_SLOT_SIZE));
                }
                for (address, _) in &writer.flash.writes {
                    assert!(!(*address >= base as u32 && *address < base as u32 + OTA_SLOT_SIZE));
                }
                assert_eq!(
                    writer.read_otadata().unwrap().active_slot(),
                    Some(1 - actual)
                );
            }
        }
    }

    #[test]
    fn missing_evidence_defaults_to_typed_failure_before_any_flash_access() {
        struct NoEvidence;
        impl OtaFlash for NoEvidence {
            fn read(&mut self, _: u32, _: &mut [u8]) -> Result<(), FirmwareError> {
                panic!("must fail before flash access");
            }
            fn write(&mut self, _: u32, _: &[u8]) -> Result<(), FirmwareError> {
                panic!("must never write");
            }
            fn erase_sector(&mut self, _: u32) -> Result<(), FirmwareError> {
                panic!("must never erase");
            }
        }
        assert!(matches!(
            EspFirmwareWriter::new(NoEvidence, never_resets),
            Err(OtaInitError::RunningImage(RunningImageError::Unavailable))
        ));
        let mut flash = MockFlash::new();
        flash.running = Ok(2);
        assert!(matches!(
            EspFirmwareWriter::new(flash, never_resets),
            Err(OtaInitError::RunningImage(
                RunningImageError::OutsideOtaSlots
            ))
        ));
    }

    #[test]
    fn lost_or_changed_running_evidence_fails_closed() {
        for evidence in [Err(RunningImageError::Unavailable), Ok(1)] {
            let mut writer = new_writer(MockFlash::new());
            writer.flash.running = evidence;
            assert_eq!(writer.erase_slot(), Err(FirmwareError::HardwareError));
            assert_eq!(
                writer.write_block(0, &[0; 4]),
                Err(FirmwareError::WriteFailed)
            );
            assert!(writer.flash.erased.is_empty());
            assert!(writer.flash.writes.is_empty());

            let mut writer = new_writer(MockFlash::new());
            stage(&mut writer, &esp_image(EXPECTED_CHIP_ID, 4), 48).unwrap();
            let erases = writer.flash.erased.len();
            let writes = writer.flash.writes.len();
            writer.flash.running = evidence;
            assert_eq!(
                writer.stage_activation(),
                Err(FirmwareError::ActivateFailed)
            );
            assert_eq!(writer.flash.erased.len(), erases);
            assert_eq!(writer.flash.writes.len(), writes);
        }
    }

    #[test]
    fn original_public_repro_cannot_verify_or_activate() {
        let mut image = vec![0; HEADER_SIZE + DIGEST_SIZE];
        image[0] = IMAGE_MAGIC;
        image[1] = 17;
        image[12..14].copy_from_slice(&EXPECTED_CHIP_ID.to_le_bytes());
        image[23] = 1;
        rehash(&mut image);
        let mut writer = new_writer(MockFlash::new());
        assert_eq!(
            stage(&mut writer, &image, 48),
            Err(FirmwareError::VerifyFailed)
        );
        assert_eq!(
            writer.stage_activation(),
            Err(FirmwareError::ActivateFailed)
        );
        assert_eq!(writer.read_otadata().unwrap().active_slot(), None);
        assert!(
            writer
                .flash
                .erased
                .iter()
                .all(|address| *address >= OTA_1_OFFSET)
        );
    }

    #[test]
    fn hash_correct_but_checksum_invalid_cannot_activate() {
        let mut image = esp_image(EXPECTED_CHIP_ID, 64);
        let checksum = image.len() - DIGEST_SIZE - 1;
        image[checksum] ^= 1;
        rehash(&mut image);
        let mut writer = new_writer(MockFlash::new());
        assert_eq!(
            stage(&mut writer, &image, 47),
            Err(FirmwareError::VerifyFailed)
        );
        assert_eq!(
            writer.stage_activation(),
            Err(FirmwareError::ActivateFailed)
        );
        assert_eq!(writer.read_otadata().unwrap().active_slot(), None);
    }

    #[test]
    fn activation_rechecks_flash_integrity_after_verification() {
        let image = esp_image(EXPECTED_CHIP_ID, 64);
        let mut writer = new_writer(MockFlash::new());
        stage(&mut writer, &image, 48).unwrap();
        writer.flash.data[OTA_1_OFFSET as usize + 300] ^= 1;
        assert_eq!(writer.stage_activation(), Err(FirmwareError::VerifyFailed));
        assert_eq!(writer.read_otadata().unwrap().active_slot(), None);
    }

    #[test]
    fn partial_final_word_is_padded_but_not_accepted_as_an_image() {
        let mut writer = new_writer(MockFlash::new());
        writer.erase_slot().unwrap();
        writer.write_block(0, &[1, 2, 3]).unwrap();
        assert!(writer.flash.writes.is_empty());
        assert_eq!(writer.verify(3, None), Err(FirmwareError::VerifyFailed));
        assert_eq!(writer.flash.slice(OTA_1_OFFSET, 4), &[1, 2, 3, 0xFF]);
        assert_eq!(
            writer.stage_activation(),
            Err(FirmwareError::ActivateFailed)
        );
    }

    /// Explicit opt-in, never silently passes without a real generated file:
    /// ESP_OTA_TEST_IMAGE=/path/to/espflash-save-image.app.bin cargo test ... \
    ///   generated_application_image -- --ignored --nocapture
    #[test]
    #[ignore = "requires ESP_OTA_TEST_IMAGE: real espflash-generated application binary"]
    fn generated_application_image() {
        let path = std::env::var("ESP_OTA_TEST_IMAGE").expect("set ESP_OTA_TEST_IMAGE");
        let image = std::fs::read(&path).expect("read actual application image");
        for actual in 0..2 {
            let mut flash = MockFlash::with_otadata((2 - actual) as u32, 0);
            flash.running = Ok(actual);
            let mut writer = new_writer(flash);
            stage(&mut writer, &image, 48).expect("real image verifies");
            writer.stage_activation().expect("real image activates");
            assert_eq!(
                writer.flash.slice(ota_slot_offset(1 - actual), image.len()),
                image
            );
        }
        println!(
            "verified and activated {} bytes in both slots: {path}",
            image.len()
        );
    }
}
