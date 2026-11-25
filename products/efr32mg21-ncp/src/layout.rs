//! Memory map of the EFR32MG21 NCP (mirrors `link/memory-*.x`).
//!
//! Offsets are bus addresses (the xG21 main flash starts at 0); the HAL's
//! flash partitions use the same convention. 8 KiB pages:
//!
//! ```text
//! 0x00000..0x04000      Gecko bootloader (kept, never written)    16 KiB
//! 0x04000..NV_START     application (GBL target)   656 KiB (F768) / 400 KiB (F512)
//! NV_START..FLASH_SIZE  NV journals (outside every GBL)            96 KiB
//! ```
//!
//! The journals follow the ESP32-H2 NCP (ZRHP spec 8.9) at the tiny tier's
//! sizes (`ncp-coordinator-app/tiny`): every journal is two 8 KiB pages,
//! the smallest independently erasable halves. They sit at the top of
//! flash so that a GBL, which only erases the pages it programs, leaves
//! them in place.

use efr32s2_hal::flash::FLASH_PAGE_SIZE;

/// Fitted flash: 768 KiB (EFR32MG21A020F768, Tuya ZS3L, SONOFF Dongle-E)
/// or 512 KiB (F512 parts).
#[cfg(feature = "flash-768k")]
pub const FLASH_SIZE: u32 = 768 * 1024;
#[cfg(feature = "flash-512k")]
pub const FLASH_SIZE: u32 = 512 * 1024;
/// Usable SRAM of both parts (64 KiB; the first word is the Gecko
/// reset-cause record, outside the image).
pub const SRAM_SIZE: u32 = 64 * 1024;
/// Resident Gecko bootloader (`0x0..0x4000`, as on the ZBT-1).
pub const BOOTLOADER_SIZE: u32 = 0x4000;
/// Application start.
pub const APPLICATION_START: u32 = BOOTLOADER_SIZE;

/// One journal: start offset, total size (two halves) and the logical
/// sector (half) size.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    pub start: u32,
    pub size: usize,
    pub sector: usize,
}

impl Window {
    pub const fn end(&self) -> u32 {
        self.start + self.size as u32
    }
}

const fn below(end: u32, sector: usize) -> Window {
    Window {
        start: end - 2 * sector as u32,
        size: 2 * sector,
        sector,
    }
}

/// NWK/APS reservations, keys and replay floors: 2 x 8 KiB (the tiny
/// tier's 48 APS keys and 64 NWK floors fit an 8 KiB rollover snapshot,
/// `SecurityStore::NCP_SECTOR_FITS`).
pub const SECURITY: Window = below(FLASH_SIZE, FLASH_PAGE_SIZE);
/// Trust Center devices (48): one 4 KiB chunk per snapshot, 2 x 8 KiB.
pub const TRUST_CENTER: Window = below(SECURITY.start, FLASH_PAGE_SIZE);
/// 32 children in 512 B slots: 2 x 8 KiB.
pub const CHILDREN: Window = below(TRUST_CENTER.start, FLASH_PAGE_SIZE);
/// GP combo (Proxy 8 + Sink 8): 2 x 8 KiB.
pub const GREEN_POWER: Window = below(CHILDREN.start, FLASH_PAGE_SIZE);
/// APS bindings, groups and application keys: 2 x 8 KiB.
pub const APS_TABLES: Window = below(GREEN_POWER.start, FLASH_PAGE_SIZE);
/// Identity and settings (IEEE override, UART default; reserved): 2 x 8 KiB.
pub const IDENTITY: Window = below(APS_TABLES.start, FLASH_PAGE_SIZE);

/// Start of the NV journals = end of the application window.
pub const NV_START: u32 = IDENTITY.start;
/// Total NV size.
pub const NV_SIZE: usize = (FLASH_SIZE - NV_START) as usize;
pub const APPLICATION_END: u32 = NV_START;
/// Largest application image (and GBL program payload).
pub const APPLICATION_SIZE: u32 = APPLICATION_END - APPLICATION_START;

const _: () = {
    assert!(NV_SIZE == 96 * 1024);
    assert!(SECURITY.end() == FLASH_SIZE);
    assert!((NV_START as usize).is_multiple_of(FLASH_PAGE_SIZE));
    assert!((APPLICATION_START as usize).is_multiple_of(FLASH_PAGE_SIZE));
    assert!(FLASH_SIZE as usize <= efr32s2_hal::flash::MAX_FLASH_CAPACITY);
    assert!(FLASH_SIZE as usize <= tuya_zs3l::INTERNAL_FLASH_CAPACITY);
    // link/memory-768k.x and link/memory-512k.x
    assert!(APPLICATION_START == 0x4000);
    #[cfg(feature = "flash-768k")]
    assert!(APPLICATION_END == 0xA_8000);
    #[cfg(feature = "flash-512k")]
    assert!(APPLICATION_END == 0x6_8000);
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn windows_tile_the_nv_region_without_overlap() {
        let order = [
            IDENTITY,
            APS_TABLES,
            GREEN_POWER,
            CHILDREN,
            TRUST_CENTER,
            SECURITY,
        ];
        let mut at = NV_START;
        for w in order {
            assert_eq!(w.start, at);
            assert!((w.start as usize).is_multiple_of(FLASH_PAGE_SIZE));
            assert_eq!(w.size, 2 * w.sector);
            at = w.end();
        }
        assert_eq!(at, FLASH_SIZE);
    }

    #[test]
    fn bootloader_and_journals_are_outside_the_application() {
        const { assert!(APPLICATION_START >= BOOTLOADER_SIZE) };
        const { assert!(APPLICATION_END <= NV_START) };
        #[cfg(feature = "flash-768k")]
        assert_eq!(APPLICATION_SIZE, 656 * 1024);
        #[cfg(feature = "flash-512k")]
        assert_eq!(APPLICATION_SIZE, 400 * 1024);
    }
}
