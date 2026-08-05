//! Product-owned Zigbee persistence: a crash-safe security-state journal at
//! the last 8 KiB of the physical flash chip.
//!
//! Mirrors the EFR32MG1 TRADFRI product's `PartitionFlash` pattern: a
//! generic const-bounded window over the board's whole-chip raw flash
//! access, translating relative offsets to physical addresses.
//!
//! This replaces the legacy flat `LogStructuredNv`-based `save_state`/
//! `restore_state` this product used before: that format persists the *live*
//! NWK frame counter directly and is documented as "not suitable for
//! production secured restore" (see `ZigbeeDevice::restore_state`).
//! `SecurityStateJournal` reserves a bounded block of frame-counter values on
//! every commit, so a power loss can never replay a previously used counter
//! value — the durability guarantee OTA and battery-powered operation both
//! depend on.

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use esp_storage::FlashStorageError;
use esp32_zigbee_devkit::flash::RawFlash;
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

use crate::migration::{MigrationError, MigrationOutcome};

/// Start of the reserved Zigbee NV window: the last 8 KiB of the physical
/// flash chip. Deliberately the same addresses used before any partition
/// table existed, so introducing one does not move
/// already-joined network state.
pub const NV_OFFSET: u32 = 0x003F_E000;
/// Size of the reserved Zigbee NV window (two 4 KiB sectors).
pub const NV_SIZE: usize = 0x0000_2000;

const SECTOR_A: u32 = 0;
const SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;

const _: () = assert!(NV_SIZE == 2 * SECURITY_JOURNAL_SECTOR_SIZE);
const _: () = assert!(NV_OFFSET as usize + NV_SIZE == 0x0040_0000);

// On both OTA-capable builds `layout` independently mirrors these addresses
// (it must stay host-testable, so it cannot import this `target_os = "none"`
// module) — cross-check them here so the two cannot silently drift apart.
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
const _: () = assert!(NV_OFFSET == crate::layout::NV_OFFSET);
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
const _: () = assert!(NV_SIZE as u32 == crate::layout::NV_SIZE);

/// The reserved 8 KiB Zigbee NV window, bounded within the physical chip.
pub struct SecurityFlash {
    flash: RawFlash,
}

impl SecurityFlash {
    fn new() -> Self {
        Self {
            flash: RawFlash::new(),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashStorageError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= NV_SIZE)
            .ok_or(FlashStorageError::OutOfBounds)?;
        NV_OFFSET
            .checked_add(offset)
            .ok_or(FlashStorageError::OutOfBounds)
    }
}

impl ErrorType for SecurityFlash {
    type Error = FlashStorageError;
}

impl ReadNorFlash for SecurityFlash {
    const READ_SIZE: usize = esp_storage::FlashStorage::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        NV_SIZE
    }
}

impl NorFlash for SecurityFlash {
    const WRITE_SIZE: usize = esp_storage::FlashStorage::WRITE_SIZE;
    const ERASE_SIZE: usize = esp_storage::FlashStorage::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashStorageError::OutOfBounds);
        }
        let length = usize::try_from(to - from).map_err(|_| FlashStorageError::OutOfBounds)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(FlashStorageError::OutOfBounds)?;
        self.flash.erase(physical_from, physical_to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.write(physical, bytes)
    }
}

/// Durable Zigbee security/network state for this product.
pub type SecurityStore = SecurityStateJournal<SecurityFlash>;

/// Build the security store over the reserved NV window.
pub fn security_store() -> SecurityStore {
    SecurityStateJournal::new(SecurityFlash::new(), SECTOR_A, SECTOR_B)
}

/// Run the one-time legacy persistence migration, then build the security
/// store over the reserved NV window.
///
/// This is the composition-root entry point: it upgrades an already-joined
/// device's `LogStructuredNv` state to the crash-safe journal exactly once (see
/// [`crate::migration`]), keeping the device on its existing network and
/// flooring both counter reservations above the legacy live counter so none can
/// be reused across the format switch. On any migration error the caller must
/// halt rather than treat the device as factory-new — a hardware or
/// corrupt-legacy result means the reserved region may still be intact.
pub fn open_security_store() -> Result<(SecurityStore, MigrationOutcome), MigrationError> {
    let outcome = crate::migration::migrate(&mut SecurityFlash::new(), SECTOR_A, SECTOR_B)?;
    Ok((security_store(), outcome))
}
