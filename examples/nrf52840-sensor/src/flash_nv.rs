//! Flash-backed NV storage for nRF52840 using NVMC.
//!
//! Uses the last 2 flash pages (8 KB) for persistent Zigbee state.
//! Log-structured: items are appended sequentially, latest wins.
//! When the active page fills, compact to the scratch page and swap.
//!
//! # Flash layout (nRF52840: 1 MB, page = 4 KB)
//! ```text
//! Page 254 (0xFE000): Active NV page
//! Page 255 (0xFF000): Scratch page (for compaction)
//! ```
//!
//! # Item format (4-byte aligned)
//! ```text
//! [magic:2][id:2][len:2][pad:2][data:len][pad to 4B]
//! ```

use core::cell::RefCell;

use embassy_nrf::nvmc::{Nvmc, PAGE_SIZE};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use zigbee_runtime::nv_storage::{NvError, NvItemId, NvStorage};

/// Page addresses for NV storage (last 2 pages of 1 MB flash).
const NV_PAGE_A: u32 = 0x000F_E000; // Page 254
const NV_PAGE_B: u32 = 0x000F_F000; // Page 255

/// Magic bytes indicating a valid item header.
const ITEM_MAGIC: u16 = 0xA55A;

/// Header size: magic(2) + id(2) + len(2) + pad(2) = 8 bytes.
const HEADER_SIZE: usize = 8;

/// Flash-backed NV storage.
/// Uses RefCell for interior mutability since NVMC reads require &mut.
pub struct FlashNvStorage<'d> {
    nvmc: RefCell<Nvmc<'d>>,
    /// Which page is currently active (A or B).
    active_page: u32,
    /// Current write offset within the active page.
    write_offset: usize,
}

impl<'d> FlashNvStorage<'d> {
    /// Create and initialize flash NV storage.
    ///
    /// Scans both pages to determine which is active and where the
    /// write cursor is. If neither page has data, initializes page A.
    pub fn new(nvmc: Nvmc<'d>) -> Self {
        let mut s = Self {
            nvmc: RefCell::new(nvmc),
            active_page: NV_PAGE_A,
            write_offset: 0,
        };
        s.init();
        s
    }

    /// Initialize: find active page and write offset.
    fn init(&mut self) {
        let a_valid = self.page_has_data(NV_PAGE_A);
        let b_valid = self.page_has_data(NV_PAGE_B);

        match (a_valid, b_valid) {
            (true, false) => {
                self.active_page = NV_PAGE_A;
            }
            (false, true) => {
                self.active_page = NV_PAGE_B;
            }
            (true, true) => {
                // Both have data — shouldn't happen, use A and erase B
                self.active_page = NV_PAGE_A;
                let _ = self.erase_page(NV_PAGE_B);
            }
            (false, false) => {
                // Fresh flash — use A
                self.active_page = NV_PAGE_A;
            }
        }

        // Find write offset by scanning for first empty slot
        self.write_offset = self.find_write_offset(self.active_page);

        log::debug!(
            "[FlashNV] Active page=0x{:05X}, write_offset={}",
            self.active_page,
            self.write_offset
        );
    }

    /// Check if a page has any valid NV data.
    fn page_has_data(&self, page: u32) -> bool {
        let mut hdr = [0u8; HEADER_SIZE];
        if self.nvmc.borrow_mut().read(page, &mut hdr).is_err() {
            return false;
        }
        let magic = u16::from_le_bytes([hdr[0], hdr[1]]);
        magic == ITEM_MAGIC
    }

    /// Scan a page to find where the next write should go.
    fn find_write_offset(&self, page: u32) -> usize {
        let mut offset = 0;
        let mut hdr = [0u8; HEADER_SIZE];

        while offset + HEADER_SIZE <= PAGE_SIZE {
            if self.nvmc.borrow_mut().read(page + offset as u32, &mut hdr).is_err() {
                return offset;
            }
            let magic = u16::from_le_bytes([hdr[0], hdr[1]]);
            if magic != ITEM_MAGIC {
                return offset;
            }
            let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
            let item_size = HEADER_SIZE + align4(len);
            offset += item_size;
        }
        offset
    }

    /// Read the latest value of an item by scanning the entire active page.
    /// Returns data_len or None.
    fn find_latest(&self, id: NvItemId, buf: &mut [u8]) -> Option<usize> {
        let mut offset = 0;
        let mut hdr = [0u8; HEADER_SIZE];
        let mut found_len = None;
        let mut nvmc = self.nvmc.borrow_mut();

        while offset + HEADER_SIZE <= PAGE_SIZE {
            if nvmc.read(self.active_page + offset as u32, &mut hdr).is_err() {
                break;
            }
            let magic = u16::from_le_bytes([hdr[0], hdr[1]]);
            if magic != ITEM_MAGIC {
                break;
            }
            let item_id = u16::from_le_bytes([hdr[2], hdr[3]]);
            let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;

            if item_id == id as u16 {
                if len == 0 {
                    // Deletion marker
                    found_len = None;
                } else if len <= buf.len() {
                    let data_addr = self.active_page + (offset + HEADER_SIZE) as u32;
                    if nvmc.read(data_addr, &mut buf[..len]).is_ok() {
                        found_len = Some(len);
                    }
                }
            }

            let item_size = HEADER_SIZE + align4(len);
            offset += item_size;
        }

        found_len
    }

    /// Append an item to the active page. Returns Err if page is full.
    fn append_item(&mut self, id: NvItemId, data: &[u8]) -> Result<(), NvError> {
        let aligned_len = align4(data.len());
        let total = HEADER_SIZE + aligned_len;

        if self.write_offset + total > PAGE_SIZE {
            // Try compaction first
            self.compact()?;
            if self.write_offset + total > PAGE_SIZE {
                return Err(NvError::Full);
            }
        }

        // Build header
        let hdr = [
            (ITEM_MAGIC & 0xFF) as u8,
            (ITEM_MAGIC >> 8) as u8,
            (id as u16 & 0xFF) as u8,
            (id as u16 >> 8) as u8,
            (data.len() as u16 & 0xFF) as u8,
            (data.len() as u16 >> 8) as u8,
            0x00, // padding
            0x00,
        ];

        let addr = self.active_page + self.write_offset as u32;

        {
            let mut nvmc = self.nvmc.borrow_mut();

            // Write header
            nvmc.write(addr, &hdr)
                .map_err(|_| NvError::HardwareError)?;

            // Write data (must be 4-byte aligned for NVMC)
            if !data.is_empty() {
                let mut aligned_buf = [0xFFu8; 128];
                aligned_buf[..data.len()].copy_from_slice(data);
                nvmc.write(addr + HEADER_SIZE as u32, &aligned_buf[..aligned_len])
                    .map_err(|_| NvError::HardwareError)?;
            }
        }

        self.write_offset += total;
        Ok(())
    }

    /// Erase a flash page.
    fn erase_page(&self, page: u32) -> Result<(), NvError> {
        self.nvmc.borrow_mut()
            .erase(page, page + PAGE_SIZE as u32)
            .map_err(|_| NvError::HardwareError)
    }

    /// Scratch page address (the one NOT active).
    fn scratch_page(&self) -> u32 {
        if self.active_page == NV_PAGE_A {
            NV_PAGE_B
        } else {
            NV_PAGE_A
        }
    }
}

impl NvStorage for FlashNvStorage<'_> {
    fn read(&self, id: NvItemId, buf: &mut [u8]) -> Result<usize, NvError> {
        self.find_latest(id, buf).ok_or(NvError::NotFound)
    }

    fn write(&mut self, id: NvItemId, data: &[u8]) -> Result<(), NvError> {
        self.append_item(id, data)
    }

    fn delete(&mut self, id: NvItemId) -> Result<(), NvError> {
        // Write a zero-length entry as deletion marker
        self.append_item(id, &[])
    }

    fn exists(&self, id: NvItemId) -> bool {
        let mut buf = [0u8; 128];
        self.read(id, &mut buf).is_ok()
    }

    fn item_length(&self, id: NvItemId) -> Result<usize, NvError> {
        let mut buf = [0u8; 128];
        self.read(id, &mut buf)
    }

    fn compact(&mut self) -> Result<(), NvError> {
        let scratch = self.scratch_page();

        // Erase scratch page
        self.erase_page(scratch)?;

        // Collect all unique item IDs and their latest data
        let mut seen_ids: heapless::Vec<u16, 32> = heapless::Vec::new();
        let mut offset = 0;
        let mut hdr = [0u8; HEADER_SIZE];

        // First pass: find all unique IDs
        while offset + HEADER_SIZE <= PAGE_SIZE {
            if self.nvmc.borrow_mut()
                .read(self.active_page + offset as u32, &mut hdr)
                .is_err()
            {
                break;
            }
            let magic = u16::from_le_bytes([hdr[0], hdr[1]]);
            if magic != ITEM_MAGIC {
                break;
            }
            let item_id = u16::from_le_bytes([hdr[2], hdr[3]]);
            if !seen_ids.contains(&item_id) {
                let _ = seen_ids.push(item_id);
            }
            let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
            offset += HEADER_SIZE + align4(len);
        }

        // Second pass: for each ID, find latest value and write to scratch
        let old_active = self.active_page;
        // Temporarily switch active to write to scratch
        self.active_page = scratch;
        self.write_offset = 0;

        let mut data_buf = [0u8; 128];
        for &item_id in seen_ids.iter() {
            // Read latest from old active page
            self.active_page = old_active;
            // Re-create a fake NvItemId from raw u16
            if let Some(nv_id) = raw_to_nv_item_id(item_id) {
                if let Some(len) = self.find_latest(nv_id, &mut data_buf) {
                    // Write to scratch
                    self.active_page = scratch;
                    let _ = self.append_item(nv_id, &data_buf[..len]);
                    continue;
                }
            }
            self.active_page = scratch;
        }

        // Erase old active page
        self.active_page = scratch;
        let _ = self.erase_page(old_active);

        log::debug!(
            "[FlashNV] Compacted: {} items, write_offset={}",
            seen_ids.len(),
            self.write_offset
        );

        Ok(())
    }
}

/// Round up to next 4-byte boundary.
const fn align4(n: usize) -> usize {
    (n + 3) & !3
}

/// Convert raw u16 back to NvItemId (for compaction).
fn raw_to_nv_item_id(raw: u16) -> Option<NvItemId> {
    // Match all known IDs
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
