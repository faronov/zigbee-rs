//! Non-volatile storage abstraction for persistent Zigbee state.
//!
//! Zigbee devices need to persist:
//! - Network parameters (PAN ID, channel, addresses, keys)
//! - Binding table
//! - Group table
//! - Scene table
//! - OTA upgrade state
//! - Application attributes (e.g., on/off state, setpoints)
//!
//! This module provides a trait that platform backends implement
//! using their specific flash/EEPROM hardware.

/// NV storage item IDs — identifies what's being stored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum NvItemId {
    // Network parameters
    NwkPanId = 0x0001,
    NwkChannel = 0x0002,
    NwkShortAddress = 0x0003,
    NwkExtendedPanId = 0x0004,
    NwkIeeeAddress = 0x0005,
    NwkKey = 0x0006,
    NwkKeySeqNum = 0x0007,
    NwkFrameCounter = 0x0008,
    NwkDepth = 0x0009,
    NwkParentAddress = 0x000A,
    NwkUpdateId = 0x000B,

    // APS parameters
    ApsTrustCenterAddress = 0x0020,
    ApsLinkKey = 0x0021,
    ApsBindingTable = 0x0022,
    ApsGroupTable = 0x0023,

    // BDB parameters
    BdbNodeIsOnNetwork = 0x0040,
    BdbCommissioningMode = 0x0041,

    // Application data (0x0100+)
    AppEndpoint1 = 0x0100,
    AppEndpoint2 = 0x0101,
    AppEndpoint3 = 0x0102,
    AppCustomBase = 0x0200,
}

/// NV storage error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NvError {
    /// Item not found.
    NotFound,
    /// Storage is full.
    Full,
    /// Item too large for buffer.
    BufferTooSmall,
    /// Hardware error during read/write.
    HardwareError,
    /// Data corruption detected.
    Corrupt,
}

/// Non-volatile storage trait — implement per platform.
///
/// # Platform implementations
/// - **ESP32**: `nvs_flash` partition
/// - **nRF52840**: `nrf-softdevice` flash or littlefs
/// - **STM32WB**: internal flash with wear leveling
/// - **Generic**: `embedded-storage` trait bridge
pub trait NvStorage {
    /// Read an item from NV storage.
    ///
    /// Returns the number of bytes read into `buf`.
    fn read(&self, id: NvItemId, buf: &mut [u8]) -> Result<usize, NvError>;

    /// Write an item to NV storage.
    fn write(&mut self, id: NvItemId, data: &[u8]) -> Result<(), NvError>;

    /// Delete an item from NV storage.
    fn delete(&mut self, id: NvItemId) -> Result<(), NvError>;

    /// Check if an item exists.
    fn exists(&self, id: NvItemId) -> bool;

    /// Get the length of a stored item.
    fn item_length(&self, id: NvItemId) -> Result<usize, NvError>;

    /// Compact/defragment the storage (if applicable).
    fn compact(&mut self) -> Result<(), NvError>;
}

/// In-memory NV storage for testing (volatile — lost on reset).
pub struct RamNvStorage {
    items: heapless::Vec<NvItem, 64>,
}

struct NvItem {
    id: NvItemId,
    data: heapless::Vec<u8, 128>,
}

impl RamNvStorage {
    pub fn new() -> Self {
        Self {
            items: heapless::Vec::new(),
        }
    }
}

impl Default for RamNvStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl NvStorage for RamNvStorage {
    fn read(&self, id: NvItemId, buf: &mut [u8]) -> Result<usize, NvError> {
        let item = self
            .items
            .iter()
            .find(|i| i.id == id)
            .ok_or(NvError::NotFound)?;
        if buf.len() < item.data.len() {
            return Err(NvError::BufferTooSmall);
        }
        buf[..item.data.len()].copy_from_slice(&item.data);
        Ok(item.data.len())
    }

    fn write(&mut self, id: NvItemId, data: &[u8]) -> Result<(), NvError> {
        // Update existing
        if let Some(item) = self.items.iter_mut().find(|i| i.id == id) {
            item.data.clear();
            item.data
                .extend_from_slice(data)
                .map_err(|_| NvError::Full)?;
            return Ok(());
        }
        // New item
        let mut nv_data = heapless::Vec::new();
        nv_data.extend_from_slice(data).map_err(|_| NvError::Full)?;
        self.items
            .push(NvItem { id, data: nv_data })
            .map_err(|_| NvError::Full)?;
        Ok(())
    }

    fn delete(&mut self, id: NvItemId) -> Result<(), NvError> {
        if let Some(pos) = self.items.iter().position(|i| i.id == id) {
            self.items.swap_remove(pos);
            Ok(())
        } else {
            Err(NvError::NotFound)
        }
    }

    fn exists(&self, id: NvItemId) -> bool {
        self.items.iter().any(|i| i.id == id)
    }

    fn item_length(&self, id: NvItemId) -> Result<usize, NvError> {
        self.items
            .iter()
            .find(|i| i.id == id)
            .map(|i| i.data.len())
            .ok_or(NvError::NotFound)
    }

    fn compact(&mut self) -> Result<(), NvError> {
        Ok(()) // RAM storage doesn't need compaction
    }
}
