//! Memory map of the EFR32MG24 NCP (mirrors `link/memory.x`).
//!
//! Offsets are relative to the main-flash base `0x0800_0000`
//! (`efr32s2_hal::chip::FLASH_BASE`); the HAL's flash partitions use the
//! same convention. The map is identical on the RF-BM-MG24B1 module and the
//! Dongle-M (both 1536 KiB, 8 KiB pages):
//!
//! ```text
//! 0x000000..0x006000  Gecko bootloader (kept, never written)      24 KiB
//! 0x006000..0x158000  application (GBL target)                   1352 KiB
//! 0x158000..0x180000  NV journals (outside every GBL)             160 KiB
//! ```
//!
//! The journals follow the ESP32-H2 NCP (ZRHP spec 8.9). Logical sectors
//! are whole multiples of the 8 KiB page, so every journal half erases
//! independently. They sit at the top of flash so that a GBL, which only
//! erases the pages it programs, leaves them in place, and so that a later
//! A/B layout below them would not move them.

use efr32s2_hal::flash::FLASH_PAGE_SIZE;

/// Fitted flash of both boards.
pub const FLASH_SIZE: u32 = 1536 * 1024;
/// Resident Gecko bootloader (`0x0800_0000..0x0800_6000`); the stock
/// Dongle-M GBLs program from `0x0800_6000`.
pub const BOOTLOADER_SIZE: u32 = 0x6000;
/// Application start (offset); bus address `0x0800_6000`.
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

/// NWK/APS reservations, keys and replay floors: 2 x 16 KiB (the full
/// `ncp-capacity` replay snapshot needs 16 KiB sectors).
pub const SECURITY: Window = below(FLASH_SIZE, 16 * 1024);
/// Trust Center devices (256), streamed snapshots: 2 x 32 KiB.
pub const TRUST_CENTER: Window = below(SECURITY.start, 32 * 1024);
/// 64 children in 1 KiB slots: 2 x 8 KiB.
pub const CHILDREN: Window = below(TRUST_CENTER.start, FLASH_PAGE_SIZE);
/// GP combo (Proxy 8 + Sink 16): 2 x 8 KiB.
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
    assert!(NV_SIZE == 160 * 1024);
    assert!(SECURITY.sector.is_multiple_of(FLASH_PAGE_SIZE));
    assert!(TRUST_CENTER.sector.is_multiple_of(FLASH_PAGE_SIZE));
    assert!((NV_START as usize).is_multiple_of(FLASH_PAGE_SIZE));
    assert!((APPLICATION_START as usize).is_multiple_of(FLASH_PAGE_SIZE));
    assert!(FLASH_SIZE as usize <= efr32s2_hal::flash::MAX_FLASH_CAPACITY);
    // link/memory.x
    assert!(APPLICATION_START == 0x6000);
    assert!(APPLICATION_END == 0x15_8000);
    assert!(IDENTITY.start == 0x15_8000);
    assert!(APS_TABLES.start == 0x15_C000);
    assert!(GREEN_POWER.start == 0x16_0000);
    assert!(CHILDREN.start == 0x16_4000);
    assert!(TRUST_CENTER.start == 0x16_8000);
    assert!(SECURITY.start == 0x17_8000);
    assert!(SECURITY.end() == FLASH_SIZE);
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
        assert_eq!(APPLICATION_SIZE, 0x15_2000);
    }
}
