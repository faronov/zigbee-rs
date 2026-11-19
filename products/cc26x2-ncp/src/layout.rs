//! NV journal placement of the NCP (mirrors `link/memory-<chip>.x`).
//!
//! The journals and their logical sector sizes follow the ESP32-H2 NCP
//! (ZRHP spec 8.9) on the CC26x2's 8 KiB erase sectors: logical sectors are
//! whole multiples of a physical sector, so every journal half erases
//! independently.

use cc26x2_hal::chip::{FLASH_SIZE, SECTOR_SIZE};
use cc26x2_hal::flash::CCFG_SECTOR;

/// One journal: start address, total size (two halves) and the logical
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
pub const SECURITY: Window = below(CCFG_SECTOR, 16 * 1024);
/// Trust Center devices (256), streamed snapshots: 2 x 32 KiB.
pub const TRUST_CENTER: Window = below(SECURITY.start, 32 * 1024);
/// 64 children in 1 KiB slots: 2 x 8 KiB.
pub const CHILDREN: Window = below(TRUST_CENTER.start, SECTOR_SIZE);
/// GP combo (Proxy 8 + Sink 16): 2 x 8 KiB.
pub const GREEN_POWER: Window = below(CHILDREN.start, SECTOR_SIZE);
/// APS bindings, groups and application keys: 2 x 8 KiB.
pub const APS_TABLES: Window = below(GREEN_POWER.start, SECTOR_SIZE);
/// Identity and settings (IEEE override, UART default): 2 x 8 KiB.
pub const IDENTITY: Window = below(APS_TABLES.start, SECTOR_SIZE);

/// Start of the NV journals = end of the application window.
pub const NV_START: u32 = IDENTITY.start;
/// Total NV size.
pub const NV_SIZE: usize = (CCFG_SECTOR - NV_START) as usize;
pub const APPLICATION_END: u32 = NV_START;

const _: () = {
    assert!(CCFG_SECTOR as usize == FLASH_SIZE - SECTOR_SIZE);
    assert!(NV_SIZE == 160 * 1024);
    assert!(SECURITY.sector.is_multiple_of(SECTOR_SIZE));
    assert!(TRUST_CENTER.sector.is_multiple_of(SECTOR_SIZE));
    assert!((NV_START as usize).is_multiple_of(SECTOR_SIZE));
};

#[cfg(feature = "cc2652p7-dongle")]
const _: () = {
    assert!(FLASH_SIZE == 704 * 1024);
    // link/memory-cc2652p7.x
    assert!(APPLICATION_END == 0x86000);
    assert!(IDENTITY.start == 0x86000);
    assert!(APS_TABLES.start == 0x8A000);
    assert!(GREEN_POWER.start == 0x8E000);
    assert!(CHILDREN.start == 0x92000);
    assert!(TRUST_CENTER.start == 0x96000);
    assert!(SECURITY.start == 0xA6000);
    assert!(SECURITY.end() == 0xAE000);
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
            assert!((w.start as usize).is_multiple_of(SECTOR_SIZE));
            assert_eq!(w.size, 2 * w.sector);
            at = w.end();
        }
        assert_eq!(at, CCFG_SECTOR);
    }
}
