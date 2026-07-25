//! [`FirmwareWriter`] implementation that stages a Zigbee OTA payload into the
//! inactive ESP-IDF application slot.
//!
//! # How an upgrade runs
//!
//! 1. `otadata` is read to learn which slot the bootloader is currently booting
//!    (`ota_0` when `otadata` is erased) and the *other* slot becomes the
//!    staging target. The running image is never written to.
//! 2. [`FirmwareWriter::erase_slot`] is bookkeeping only. Erasing 1.9 MiB up
//!    front would hold a critical section for many seconds — `esp-storage`
//!    masks interrupts around every ROM flash call — and the radio would miss
//!    every parent poll in the meantime. Instead each 4 KiB sector is erased
//!    lazily, immediately before the first byte lands in it, which spreads the
//!    erase cost across the download at ~1 sector per 4 KiB of payload.
//! 3. Zigbee delivers ragged blocks (the last one is almost never a multiple of
//!    4), but ESP flash programs 4-byte words. Sub-word tails are buffered in
//!    RAM until the next block completes the word; the final partial word is
//!    padded with `0xFF` — the erased value — so no byte of the image is ever
//!    altered and the padding lives past the end of the image.
//! 4. [`FirmwareWriter::verify`] re-reads the staged slot: the ESP image magic,
//!    the chip ID and the appended SHA-256 all have to check out. Nothing about
//!    the transfer is trusted.
//! 5. [`FirmwareWriter::activate`] writes one 32-byte `otadata` entry into the
//!    sector that does not hold the active entry and resets the chip. A power
//!    failure at any point before that write leaves the old entry — and the old
//!    firmware — in charge.

use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

use crate::esp_image::{
    DIGEST_SIZE, EXPECTED_CHIP_ID, EspImageHeader, HEADER_SIZE, ImageError, hashed_range,
};
use crate::layout::{
    EXPECTED_PARTITIONS, OTA_SLOT_SIZE, OTADATA_OFFSET, PARTITION_ENTRY_SIZE,
    PARTITION_TABLE_OFFSET, SECTOR_SIZE, WORD_SIZE, ota_slot_offset, otadata_sector_offset,
};
use crate::otadata::{ENTRY_SIZE, OtaData, OtaSelectEntry};

/// Chunk size used when re-reading the staged image for hashing.
const VERIFY_CHUNK: usize = 256;

/// Raw flash access needed to stage an image.
///
/// Deliberately narrower than `embedded-storage`: addresses are absolute flash
/// offsets, writes are word aligned and erases are whole sectors, which is what
/// both the ROM routines and the host mock implement.
pub trait OtaFlash {
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
    /// Create a writer, choosing the staging slot from `otadata`.
    ///
    /// `reset` performs the software reset that hands control back to the
    /// bootloader; it is only called from [`FirmwareWriter::activate`], after
    /// the new `otadata` entry has been programmed and read back.
    pub fn new(flash: F, reset: fn() -> !) -> Result<Self, FirmwareError> {
        let mut writer = Self {
            flash,
            reset,
            running_slot: 0,
            target_slot: 1,
            state: State::Idle,
            written: 0,
            flushed: 0,
            tail: [0xFF; WORD_SIZE as usize],
            tail_len: 0,
            sectors_erased: 0,
            finalized: false,
        };
        writer.validate_partition_table()?;
        writer.select_target_slot()?;
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

        let data = self.read_otadata()?;
        if data.running_slot() == self.target_slot {
            // The staging slot became the running slot behind our back.
            return Err(FirmwareError::ActivateFailed);
        }

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

    fn select_target_slot(&mut self) -> Result<(), FirmwareError> {
        let data = self.read_otadata()?;
        self.running_slot = data.running_slot();
        self.target_slot = data.target_slot();
        debug_assert_ne!(self.running_slot, self.target_slot);
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
        let size = self.written;
        let (start, end) = hashed_range(size).map_err(image_rejected)?;

        let mut header = [0u8; HEADER_SIZE];
        self.flash.read(self.slot_base(), &mut header)?;
        EspImageHeader::parse(&header, EXPECTED_CHIP_ID).map_err(image_rejected)?;

        let mut hasher = crate::sha256::Sha256::new();
        let mut cursor = start;
        let mut buffer = [0u8; VERIFY_CHUNK];
        while cursor < end {
            let take = (end - cursor).min(VERIFY_CHUNK as u32) as usize;
            self.flash
                .read(self.slot_base() + cursor, &mut buffer[..take])?;
            hasher.update(&buffer[..take]);
            cursor += take as u32;
        }
        let digest = hasher.finalize();

        let mut stored = [0u8; DIGEST_SIZE];
        self.flash.read(self.slot_base() + end, &mut stored)?;
        if digest != stored {
            log::warn!("[ESP OTA] staged image SHA-256 mismatch");
            return Err(FirmwareError::VerifyFailed);
        }

        if let Some(expected) = expected_hash
            && (expected.len() != DIGEST_SIZE || expected != digest)
        {
            log::warn!("[ESP OTA] staged image does not match the expected hash");
            return Err(FirmwareError::VerifyFailed);
        }

        Ok(())
    }
}

fn image_rejected(error: ImageError) -> FirmwareError {
    log::warn!("[ESP OTA] staged image rejected: {:?}", error);
    match error {
        ImageError::TooSmall => FirmwareError::VerifyFailed,
        _ => FirmwareError::VerifyFailed,
    }
}

impl<F: OtaFlash> FirmwareWriter for EspFirmwareWriter<F> {
    /// Prepare for a download. No flash is erased here; see the module docs.
    fn erase_slot(&mut self) -> Result<(), FirmwareError> {
        self.reset_staging();
        self.select_target_slot()?;
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

/// [`OtaFlash`] backed by the ROM SPI flash routines.
#[cfg(target_os = "none")]
pub struct EspOtaFlash {
    flash: esp_storage::FlashStorage,
}

#[cfg(target_os = "none")]
impl EspOtaFlash {
    /// Open the on-board SPI flash.
    pub fn new() -> Self {
        Self {
            flash: esp_storage::FlashStorage::new(),
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
    use crate::esp_image::IMAGE_MAGIC;
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
    }

    impl MockFlash {
        fn erased() -> Self {
            Self {
                data: vec![0xFF; FLASH_SIZE as usize],
                erased: Vec::new(),
                writes: Vec::new(),
                fail_write_at: None,
            }
        }

        fn new() -> Self {
            let mut flash = Self::erased();
            for (index, partition) in EXPECTED_PARTITIONS.iter().copied().enumerate() {
                let start =
                    PARTITION_TABLE_OFFSET as usize + index * PARTITION_ENTRY_SIZE;
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

    /// Build a minimal but structurally valid ESP application image.
    fn esp_image(chip_id: u16, payload_len: usize) -> Vec<u8> {
        let mut image = vec![0u8; HEADER_SIZE + payload_len];
        image[0] = IMAGE_MAGIC;
        image[1] = 1; // one segment
        image[12..14].copy_from_slice(&chip_id.to_le_bytes());
        image[23] = 1; // hash appended
        for (index, byte) in image[HEADER_SIZE..].iter_mut().enumerate() {
            *byte = (index % 251) as u8;
        }
        let digest = sha256(&image);
        image.extend_from_slice(&digest);
        image
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
    fn erased_otadata_stages_into_slot_one() {
        let writer = new_writer(MockFlash::new());
        assert_eq!(writer.running_slot(), 0);
        assert_eq!(writer.target_slot(), 1);
        assert_eq!(writer.slot_size(), OTA_SLOT_SIZE);
    }

    #[test]
    fn rejects_flash_without_the_required_partition_table() {
        assert!(matches!(
            EspFirmwareWriter::new(MockFlash::erased(), never_resets),
            Err(FirmwareError::HardwareError)
        ));
    }

    #[test]
    fn running_slot_one_stages_into_slot_zero() {
        // seq 2 -> slot (2 - 1) % 2 == 1
        let writer = new_writer(MockFlash::with_otadata(2, 0));
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
        // 48-byte Zigbee blocks over a payload whose length is not a multiple
        // of four: every intermediate write must still be word aligned.
        let image = esp_image(EXPECTED_CHIP_ID, 501);
        assert_ne!(image.len() % 4, 0, "test needs a ragged tail");

        let mut writer = new_writer(MockFlash::new());
        stage(&mut writer, &image, 48).expect("staged");

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
}
