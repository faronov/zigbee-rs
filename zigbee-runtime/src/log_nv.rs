//! Generic log-structured NV storage over raw flash.
//!
//! Platform-independent implementation over the standard
//! [`embedded_storage::nor_flash::NorFlash`] interface.
//!
//! # Design
//! Uses 2 flash sectors (pages): one active, one scratch. Items are appended
//! sequentially; the latest entry for each ID wins. When the active page fills,
//! live items are compacted to the scratch page and roles swap.
//!
//! # Item format (4-byte aligned)
//! ```text
//! [magic:2][id:2][len:2][pad:2][data:len][pad to 4B]
//! ```
//!
//! Compacted pages begin with a generation header whose commit word is
//! programmed last. Legacy pages without this header remain readable.

use crate::nv_storage::{NvError, NvItemId, NvStorage};
use embedded_storage::nor_flash::NorFlash;

/// Magic bytes indicating a valid item header.
const ITEM_MAGIC: u16 = 0xA55A;

/// Header size: magic(2) + id(2) + len(2) + pad(2) = 8 bytes.
const HEADER_SIZE: usize = 8;
const MAX_ITEM_SIZE: usize = 128;
const PAGE_HEADER_SIZE: usize = 16;
const PAGE_MAGIC: [u8; 4] = *b"LNV1";
const PAGE_COMMIT: [u8; 4] = *b"CMIT";

#[derive(Clone, Copy, PartialEq, Eq)]
enum PageState {
    Empty,
    Legacy,
    Committed(u32),
    Invalid,
}

/// Generic log-structured NV storage backed by raw flash.
pub struct LogStructuredNv<F: NorFlash> {
    flash: F,
    /// Flash offset of page A.
    page_a: u32,
    /// Flash offset of page B.
    page_b: u32,
    /// Which page is currently active.
    active_page: u32,
    /// Current write cursor within the active page.
    write_offset: usize,
    /// Generation of the active committed page. Legacy pages use generation 0.
    generation: u32,
}

impl<F: NorFlash> LogStructuredNv<F> {
    /// Create and initialize log-structured NV storage.
    ///
    /// `page_a` and `page_b` are flash offsets for the two NV sectors.
    pub fn new(flash: F, page_a: u32, page_b: u32) -> Result<Self, NvError> {
        if F::READ_SIZE == 0
            || F::WRITE_SIZE == 0
            || F::ERASE_SIZE == 0
            || !HEADER_SIZE.is_multiple_of(F::READ_SIZE)
            || !HEADER_SIZE.is_multiple_of(F::WRITE_SIZE)
            || !PAGE_HEADER_SIZE.is_multiple_of(F::READ_SIZE)
            || !PAGE_HEADER_SIZE.is_multiple_of(F::WRITE_SIZE)
            || !4usize.is_multiple_of(F::READ_SIZE)
            || !4usize.is_multiple_of(F::WRITE_SIZE)
            || page_a == page_b
            || !(page_a as usize).is_multiple_of(F::ERASE_SIZE)
            || !(page_b as usize).is_multiple_of(F::ERASE_SIZE)
            || [page_a, page_b].iter().any(|page| {
                (*page as usize)
                    .checked_add(F::ERASE_SIZE)
                    .is_none_or(|end| end > flash.capacity())
            })
        {
            return Err(NvError::HardwareError);
        }
        let mut s = Self {
            flash,
            page_a,
            page_b,
            active_page: page_a,
            write_offset: 0,
            generation: 0,
        };
        s.init()?;
        Ok(s)
    }

    fn sector_size(&self) -> usize {
        F::ERASE_SIZE
    }

    fn init(&mut self) -> Result<(), NvError> {
        let a_state = self.page_state(self.page_a)?;
        let b_state = self.page_state(self.page_b)?;
        let a_generation = Self::valid_generation(a_state);
        let b_generation = Self::valid_generation(b_state);

        match (a_generation, b_generation) {
            (Some(a), Some(b)) => {
                if Self::is_newer_generation(b, a) {
                    self.active_page = self.page_b;
                    self.generation = b;
                    self.erase_page(self.page_a)?;
                } else {
                    self.active_page = self.page_a;
                    self.generation = a;
                    self.erase_page(self.page_b)?;
                }
            }
            (Some(a), None) => {
                self.active_page = self.page_a;
                self.generation = a;
                if b_state == PageState::Invalid {
                    self.erase_page(self.page_b)?;
                }
            }
            (None, Some(b)) => {
                self.active_page = self.page_b;
                self.generation = b;
                if a_state == PageState::Invalid {
                    self.erase_page(self.page_a)?;
                }
            }
            (None, None) if a_state == PageState::Empty && b_state == PageState::Empty => {
                self.active_page = self.page_a;
                self.generation = 0;
            }
            (None, None) => return Err(NvError::Corrupt),
        }

        self.write_offset = self.find_write_offset(self.active_page)?;

        log::debug!(
            "[LogNV] Active=0x{:05X}, offset={}",
            self.active_page,
            self.write_offset
        );
        Ok(())
    }

    fn page_state(&mut self, page: u32) -> Result<PageState, NvError> {
        let mut header = [0u8; PAGE_HEADER_SIZE];
        self.read_flash(page, &mut header)?;
        if header.iter().all(|byte| *byte == 0xFF) {
            return Ok(PageState::Empty);
        }
        if header[..4] == PAGE_MAGIC && header[12..] == PAGE_COMMIT {
            let generation = u32::from_le_bytes([header[4], header[5], header[6], header[7]]);
            let inverse = u32::from_le_bytes([header[8], header[9], header[10], header[11]]);
            return Ok(if generation ^ inverse == u32::MAX {
                PageState::Committed(generation)
            } else {
                PageState::Invalid
            });
        }
        if u16::from_le_bytes([header[0], header[1]]) == ITEM_MAGIC {
            return Ok(PageState::Legacy);
        }
        Ok(PageState::Invalid)
    }

    fn valid_generation(state: PageState) -> Option<u32> {
        match state {
            PageState::Legacy => Some(0),
            PageState::Committed(generation) => Some(generation),
            PageState::Empty | PageState::Invalid => None,
        }
    }

    fn is_newer_generation(candidate: u32, current: u32) -> bool {
        candidate != current && candidate.wrapping_sub(current) < 0x8000_0000
    }

    fn page_data_start(&mut self, page: u32) -> Result<usize, NvError> {
        match self.page_state(page)? {
            PageState::Committed(_) => Ok(PAGE_HEADER_SIZE),
            PageState::Empty | PageState::Legacy => Ok(0),
            PageState::Invalid => Err(NvError::Corrupt),
        }
    }

    fn commit_page(&mut self, page: u32, generation: u32) -> Result<(), NvError> {
        let mut header = [0xFFu8; PAGE_HEADER_SIZE];
        header[..4].copy_from_slice(&PAGE_MAGIC);
        header[4..8].copy_from_slice(&generation.to_le_bytes());
        header[8..12].copy_from_slice(&(!generation).to_le_bytes());
        self.flash
            .write(page, &header[..12])
            .map_err(|_| NvError::HardwareError)?;
        self.flash
            .write(page + 12, &PAGE_COMMIT)
            .map_err(|_| NvError::HardwareError)
    }

    fn find_write_offset(&mut self, page: u32) -> Result<usize, NvError> {
        let page_size = self.sector_size();
        let mut offset = self.page_data_start(page)?;
        let mut hdr = [0u8; HEADER_SIZE];

        while offset + HEADER_SIZE <= page_size {
            self.read_flash(page + offset as u32, &mut hdr)?;
            if u16::from_le_bytes([hdr[0], hdr[1]]) != ITEM_MAGIC {
                return Ok(offset);
            }
            let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
            let record_size = HEADER_SIZE + align4(len);
            if offset + record_size > page_size {
                return Err(NvError::Corrupt);
            }
            offset += record_size;
        }
        Ok(offset)
    }

    fn find_latest(&mut self, id: NvItemId, buf: &mut [u8]) -> Result<Option<usize>, NvError> {
        let Some((data_offset, len)) = self.find_latest_entry(id)? else {
            return Ok(None);
        };
        if len == 0 {
            return Ok(None);
        }
        if len > MAX_ITEM_SIZE {
            return Err(NvError::Corrupt);
        }
        if len > buf.len() {
            return Err(NvError::BufferTooSmall);
        }

        let aligned_len = align4(len);
        let mut data = [0u8; MAX_ITEM_SIZE];
        self.read_flash(data_offset, &mut data[..aligned_len])?;
        buf[..len].copy_from_slice(&data[..len]);
        Ok(Some(len))
    }

    fn find_latest_entry(&mut self, id: NvItemId) -> Result<Option<(u32, usize)>, NvError> {
        let page_size = self.sector_size();
        let mut offset = self.page_data_start(self.active_page)?;
        let mut hdr = [0u8; HEADER_SIZE];
        let mut found = None;

        while offset + HEADER_SIZE <= page_size {
            self.read_flash(self.active_page + offset as u32, &mut hdr)?;
            if u16::from_le_bytes([hdr[0], hdr[1]]) != ITEM_MAGIC {
                break;
            }
            let item_id = u16::from_le_bytes([hdr[2], hdr[3]]);
            let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
            let record_size = HEADER_SIZE + align4(len);
            if offset + record_size > page_size {
                return Err(NvError::Corrupt);
            }

            if item_id == id as u16 {
                found = Some((self.active_page + (offset + HEADER_SIZE) as u32, len));
            }

            offset += record_size;
        }

        Ok(found)
    }

    fn append_item(&mut self, id: NvItemId, data: &[u8]) -> Result<(), NvError> {
        if data.len() > MAX_ITEM_SIZE {
            return Err(NvError::BufferTooSmall);
        }
        let aligned_len = align4(data.len());
        let total = HEADER_SIZE + aligned_len;

        if self.write_offset + total > self.sector_size() {
            self.compact()?;
        }
        self.append_item_to_active(id, data)
    }

    fn append_item_to_active(&mut self, id: NvItemId, data: &[u8]) -> Result<(), NvError> {
        let aligned_len = align4(data.len());
        let total = HEADER_SIZE + aligned_len;
        if self.write_offset + total > self.sector_size() {
            return Err(NvError::Full);
        }
        let mut write_buf = [0xFFu8; 128 + HEADER_SIZE];
        write_buf[0] = (ITEM_MAGIC & 0xFF) as u8;
        write_buf[1] = (ITEM_MAGIC >> 8) as u8;
        write_buf[2] = (id as u16 & 0xFF) as u8;
        write_buf[3] = (id as u16 >> 8) as u8;
        write_buf[4] = (data.len() as u16 & 0xFF) as u8;
        write_buf[5] = (data.len() as u16 >> 8) as u8;
        write_buf[6] = 0x00;
        write_buf[7] = 0x00;
        if !data.is_empty() {
            write_buf[HEADER_SIZE..HEADER_SIZE + data.len()].copy_from_slice(data);
        }

        self.flash
            .write(
                self.active_page + self.write_offset as u32,
                &write_buf[..total],
            )
            .map_err(|_| NvError::HardwareError)?;
        self.write_offset += total;
        Ok(())
    }

    fn scratch_page(&self) -> u32 {
        if self.active_page == self.page_a {
            self.page_b
        } else {
            self.page_a
        }
    }

    fn read_flash(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), NvError> {
        self.flash
            .read(offset, bytes)
            .map_err(|_| NvError::HardwareError)
    }

    fn erase_page(&mut self, page: u32) -> Result<(), NvError> {
        self.flash
            .erase(page, page + F::ERASE_SIZE as u32)
            .map_err(|_| NvError::HardwareError)
    }
}

impl<F: NorFlash> NvStorage for LogStructuredNv<F> {
    fn read(&mut self, id: NvItemId, buf: &mut [u8]) -> Result<usize, NvError> {
        self.find_latest(id, buf)?.ok_or(NvError::NotFound)
    }

    fn write(&mut self, id: NvItemId, data: &[u8]) -> Result<(), NvError> {
        self.append_item(id, data)
    }

    fn delete(&mut self, id: NvItemId) -> Result<(), NvError> {
        self.append_item(id, &[])
    }

    fn exists(&mut self, id: NvItemId) -> Result<bool, NvError> {
        let mut buf = [0u8; 128];
        Ok(self.find_latest(id, &mut buf)?.is_some())
    }

    fn item_length(&mut self, id: NvItemId) -> Result<usize, NvError> {
        self.find_latest_entry(id)?
            .and_then(|(_, len)| (len != 0).then_some(len))
            .ok_or(NvError::NotFound)
    }

    fn compact(&mut self) -> Result<(), NvError> {
        let scratch = self.scratch_page();
        self.erase_page(scratch)?;

        // Collect unique item IDs
        let page_size = self.sector_size();
        let mut seen_ids: heapless::Vec<u16, 32> = heapless::Vec::new();
        let mut offset = self.page_data_start(self.active_page)?;
        let mut hdr = [0u8; HEADER_SIZE];

        while offset + HEADER_SIZE <= page_size {
            self.read_flash(self.active_page + offset as u32, &mut hdr)?;
            if u16::from_le_bytes([hdr[0], hdr[1]]) != ITEM_MAGIC {
                break;
            }
            let item_id = u16::from_le_bytes([hdr[2], hdr[3]]);
            if !seen_ids.contains(&item_id) {
                seen_ids.push(item_id).map_err(|_| NvError::Full)?;
            }
            let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
            let record_size = HEADER_SIZE + align4(len);
            if offset + record_size > page_size {
                return Err(NvError::Corrupt);
            }
            offset += record_size;
        }

        // Copy latest of each item to scratch
        let old_active = self.active_page;
        let old_write_offset = self.write_offset;
        let old_generation = self.generation;
        let mut next_generation = old_generation.wrapping_add(1);
        if next_generation == 0 {
            next_generation = 1;
        }
        self.active_page = scratch;
        self.write_offset = PAGE_HEADER_SIZE;

        let mut data_buf = [0u8; 128];
        let copy_result = (|| {
            for &item_id in seen_ids.iter() {
                self.active_page = old_active;
                if let Some(nv_id) = raw_to_nv_item_id(item_id)
                    && let Some(len) = self.find_latest(nv_id, &mut data_buf)?
                {
                    self.active_page = scratch;
                    self.append_item_to_active(nv_id, &data_buf[..len])?;
                    continue;
                }
                self.active_page = scratch;
            }
            Ok(())
        })();
        if let Err(error) = copy_result {
            self.active_page = old_active;
            self.write_offset = old_write_offset;
            self.generation = old_generation;
            return Err(error);
        }

        self.active_page = scratch;
        let scratch_write_offset = self.write_offset;
        if let Err(error) = self.commit_page(scratch, next_generation) {
            self.active_page = old_active;
            self.write_offset = old_write_offset;
            self.generation = old_generation;
            return Err(error);
        }
        self.generation = next_generation;
        if let Err(error) = self.erase_page(old_active) {
            self.active_page = scratch;
            self.write_offset = scratch_write_offset;
            return Err(error);
        }

        log::debug!(
            "[LogNV] Compacted: {} items, offset={}",
            seen_ids.len(),
            self.write_offset
        );

        Ok(())
    }
}

const fn align4(n: usize) -> usize {
    (n + 3) & !3
}

fn raw_to_nv_item_id(raw: u16) -> Option<NvItemId> {
    match raw {
        0x0001 => Some(NvItemId::NwkPanId),
        0x0002 => Some(NvItemId::NwkChannel),
        0x0003 => Some(NvItemId::NwkShortAddress),
        0x0004 => Some(NvItemId::NwkExtendedPanId),
        0x0005 => Some(NvItemId::NwkIeeeAddress),
        0x0006 => Some(NvItemId::NwkKey),
        0x0007 => Some(NvItemId::NwkKeySeqNum),
        0x0008 => Some(NvItemId::NwkFrameCounter),
        0x0009 => Some(NvItemId::NwkDepth),
        0x000A => Some(NvItemId::NwkParentAddress),
        0x000B => Some(NvItemId::NwkUpdateId),
        0x0020 => Some(NvItemId::ApsTrustCenterAddress),
        0x0021 => Some(NvItemId::ApsLinkKey),
        0x0022 => Some(NvItemId::ApsBindingTable),
        0x0023 => Some(NvItemId::ApsGroupTable),
        0x0040 => Some(NvItemId::BdbNodeIsOnNetwork),
        0x0041 => Some(NvItemId::BdbCommissioningMode),
        0x0042 => Some(NvItemId::BdbPrimaryChannelSet),
        0x0043 => Some(NvItemId::BdbSecondaryChannelSet),
        0x0044 => Some(NvItemId::BdbCommissioningGroupId),
        0x0100 => Some(NvItemId::AppEndpoint1),
        0x0101 => Some(NvItemId::AppEndpoint2),
        0x0102 => Some(NvItemId::AppEndpoint3),
        _ if raw >= 0x0200 => Some(NvItemId::AppCustomBase),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};

    const SECTOR_SIZE: usize = 256;

    struct MockFlash {
        data: [u8; SECTOR_SIZE * 2],
        fail_reads: bool,
        fail_writes: bool,
        fail_erases: bool,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                data: [0xFF; SECTOR_SIZE * 2],
                fail_reads: false,
                fail_writes: false,
                fail_erases: false,
            }
        }

        fn range(
            &self,
            offset: u32,
            length: usize,
        ) -> Result<core::ops::Range<usize>, NorFlashErrorKind> {
            let start = offset as usize;
            let end = start
                .checked_add(length)
                .filter(|end| *end <= self.data.len())
                .ok_or(NorFlashErrorKind::OutOfBounds)?;
            Ok(start..end)
        }
    }

    impl ErrorType for MockFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 4;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            if self.fail_reads {
                return Err(NorFlashErrorKind::Other);
            }
            if offset as usize % Self::READ_SIZE != 0 || bytes.len() % Self::READ_SIZE != 0 {
                return Err(NorFlashErrorKind::NotAligned);
            }
            let range = self.range(offset, bytes.len())?;
            bytes.copy_from_slice(&self.data[range]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = SECTOR_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if self.fail_erases {
                return Err(NorFlashErrorKind::Other);
            }
            let range = self.range(from, (to - from) as usize)?;
            if range.start % SECTOR_SIZE != 0 || range.end % SECTOR_SIZE != 0 {
                return Err(NorFlashErrorKind::NotAligned);
            }
            self.data[range].fill(0xFF);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if self.fail_writes {
                return Err(NorFlashErrorKind::Other);
            }
            let range = self.range(offset, bytes.len())?;
            for (old, new) in self.data[range].iter_mut().zip(bytes) {
                if (*old & *new) != *new {
                    return Err(NorFlashErrorKind::Other);
                }
                *old &= *new;
            }
            Ok(())
        }
    }

    #[test]
    fn latest_value_and_delete_round_trip() {
        let mut nv = LogStructuredNv::new(MockFlash::new(), 0, SECTOR_SIZE as u32).unwrap();
        nv.write(NvItemId::NwkChannel, &[15]).unwrap();
        nv.write(NvItemId::NwkChannel, &[20]).unwrap();

        let mut channel = [0u8; 1];
        assert_eq!(nv.read(NvItemId::NwkChannel, &mut channel), Ok(1));
        assert_eq!(channel, [20]);
        assert_eq!(nv.exists(NvItemId::NwkChannel), Ok(true));

        nv.delete(NvItemId::NwkChannel).unwrap();
        assert_eq!(
            nv.read(NvItemId::NwkChannel, &mut channel),
            Err(NvError::NotFound)
        );
        assert_eq!(nv.exists(NvItemId::NwkChannel), Ok(false));
    }

    #[test]
    fn compaction_preserves_latest_values() {
        let mut nv = LogStructuredNv::new(MockFlash::new(), 0, SECTOR_SIZE as u32).unwrap();
        for value in 0..24u8 {
            nv.write(NvItemId::NwkChannel, &[value]).unwrap();
        }

        let mut channel = [0u8; 1];
        assert_eq!(nv.read(NvItemId::NwkChannel, &mut channel), Ok(1));
        assert_eq!(channel, [23]);
    }

    #[test]
    fn historical_large_value_does_not_block_small_replacement() {
        let mut nv = LogStructuredNv::new(MockFlash::new(), 0, SECTOR_SIZE as u32).unwrap();
        nv.write(NvItemId::ApsBindingTable, &[0xAA; MAX_ITEM_SIZE])
            .unwrap();
        nv.write(NvItemId::ApsBindingTable, &[0x42]).unwrap();

        let mut value = [0u8; 1];
        assert_eq!(nv.read(NvItemId::ApsBindingTable, &mut value), Ok(1));
        assert_eq!(value, [0x42]);
    }

    #[test]
    fn incomplete_compaction_page_is_ignored_after_restart() {
        let mut nv = LogStructuredNv::new(MockFlash::new(), 0, SECTOR_SIZE as u32).unwrap();
        nv.write(NvItemId::NwkChannel, &[20]).unwrap();
        nv.compact().unwrap();
        assert_eq!(nv.active_page, SECTOR_SIZE as u32);

        let mut flash = nv.flash;
        flash.data[..4].copy_from_slice(&PAGE_MAGIC);
        flash.data[4..6].copy_from_slice(b"CM");
        flash.data[PAGE_HEADER_SIZE..PAGE_HEADER_SIZE + 2]
            .copy_from_slice(&ITEM_MAGIC.to_le_bytes());

        let mut restored = LogStructuredNv::new(flash, 0, SECTOR_SIZE as u32).unwrap();
        assert_eq!(restored.active_page, SECTOR_SIZE as u32);
        let mut channel = [0u8; 1];
        assert_eq!(restored.read(NvItemId::NwkChannel, &mut channel), Ok(1));
        assert_eq!(channel, [20]);
    }

    #[test]
    fn committed_generation_wins_over_unerased_legacy_page() {
        let mut nv = LogStructuredNv::new(MockFlash::new(), 0, SECTOR_SIZE as u32).unwrap();
        nv.write(NvItemId::NwkChannel, &[20]).unwrap();
        let mut legacy_page = [0u8; SECTOR_SIZE];
        legacy_page.copy_from_slice(&nv.flash.data[..SECTOR_SIZE]);

        nv.compact().unwrap();
        nv.flash.data[..SECTOR_SIZE].copy_from_slice(&legacy_page);

        let mut restored = LogStructuredNv::new(nv.flash, 0, SECTOR_SIZE as u32).unwrap();
        assert_eq!(restored.active_page, SECTOR_SIZE as u32);
        let mut channel = [0u8; 1];
        assert_eq!(restored.read(NvItemId::NwkChannel, &mut channel), Ok(1));
        assert_eq!(channel, [20]);
    }

    #[test]
    fn compaction_that_cannot_fit_header_preserves_source_page() {
        let mut nv = LogStructuredNv::new(MockFlash::new(), 0, SECTOR_SIZE as u32).unwrap();
        nv.write(NvItemId::ApsBindingTable, &[0xAA; 128]).unwrap();
        nv.write(NvItemId::ApsGroupTable, &[0x55; 100]).unwrap();
        nv.write(NvItemId::NwkChannel, &[20]).unwrap();

        assert_eq!(
            nv.write(NvItemId::NwkPanId, &[0x34, 0x12]),
            Err(NvError::Full)
        );
        let mut binding = [0u8; 128];
        let mut group = [0u8; 100];
        assert_eq!(nv.read(NvItemId::ApsBindingTable, &mut binding), Ok(128));
        assert_eq!(nv.read(NvItemId::ApsGroupTable, &mut group), Ok(100));
        assert_eq!(binding, [0xAA; 128]);
        assert_eq!(group, [0x55; 100]);
    }

    #[test]
    fn hardware_read_errors_are_reported() {
        let mut nv = LogStructuredNv::new(MockFlash::new(), 0, SECTOR_SIZE as u32).unwrap();
        nv.flash.fail_reads = true;
        let mut value = [0u8; 1];
        assert_eq!(
            nv.read(NvItemId::NwkChannel, &mut value),
            Err(NvError::HardwareError)
        );
    }

    #[test]
    fn invalid_layout_and_mutation_errors_are_reported() {
        assert!(matches!(
            LogStructuredNv::new(MockFlash::new(), 0, 0),
            Err(NvError::HardwareError)
        ));

        let mut write_flash = MockFlash::new();
        write_flash.fail_writes = true;
        let mut nv = LogStructuredNv::new(write_flash, 0, SECTOR_SIZE as u32).unwrap();
        assert_eq!(
            nv.write(NvItemId::NwkChannel, &[15]),
            Err(NvError::HardwareError)
        );

        let mut erase_flash = MockFlash::new();
        erase_flash.fail_erases = true;
        let mut nv = LogStructuredNv::new(erase_flash, 0, SECTOR_SIZE as u32).unwrap();
        assert_eq!(nv.compact(), Err(NvError::HardwareError));
    }
}
