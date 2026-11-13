//! Internal-flash layout of the router image. Keep in sync with
//! `link/memory.x`, whose assertions check the same boundaries at link time.

/// Main flash base.
pub const FLASH_BASE: u32 = 0x0800_0000;
/// STM32WB55 erase page.
pub const PAGE_SIZE: usize = 4096;
/// Physical bytes per journal: two 4 KiB logical sectors, each stored on two
/// physical pages by the word-per-double-word adapter.
pub const JOURNAL_PHYSICAL_SIZE: usize = 4 * PAGE_SIZE;
/// Logical sector size the journals are built with.
pub const JOURNAL_SECTOR_SIZE: usize = 4096;

/// Physical bytes of the settings journal: two 2 KiB logical sectors, one
/// physical page each through the word-per-double-word adapter.
pub const SETTINGS_PHYSICAL_SIZE: usize = 2 * PAGE_SIZE;
/// Logical sector size of the settings journal.
pub const SETTINGS_SECTOR_SIZE: usize = 2048;

/// End of the application image (`FLASH` in `link/memory.x`).
pub const APPLICATION_END: u32 = 0x080D_3000;
/// Small application settings (the Occupancy Sensing motion hold).
pub const SETTINGS_START: u32 = APPLICATION_END;
/// The router journals keep the addresses of the first commissioned
/// images, so an upgrade retains the network state.
pub const CHILD_TABLE_START: u32 = SETTINGS_START + SETTINGS_PHYSICAL_SIZE as u32;
pub const APS_TABLE_START: u32 = CHILD_TABLE_START + JOURNAL_PHYSICAL_SIZE as u32;
pub const SECURITY_START: u32 = APS_TABLE_START + JOURNAL_PHYSICAL_SIZE as u32;
/// End of product-owned non-volatile storage. The CPU2 secure area must
/// start at or above this address.
pub const NV_END: u32 = SECURITY_START + JOURNAL_PHYSICAL_SIZE as u32;

/// Secure-area start with FUS 1.2.0 and the 802.15.4 MAC 1.24.0 stack on a
/// 1 MiB part (SFSA = 0xE1), measured on the DK.
pub const EXPECTED_SECURE_START: u32 = 0x080E_1000;

const _: () = assert!(APPLICATION_END.is_multiple_of(PAGE_SIZE as u32));
const _: () = assert!(NV_END == EXPECTED_SECURE_START);
const _: () = assert!(JOURNAL_PHYSICAL_SIZE == 2 * 2 * JOURNAL_SECTOR_SIZE);
const _: () = assert!(SETTINGS_PHYSICAL_SIZE == 2 * 2 * SETTINGS_SECTOR_SIZE);
const _: () = assert!(CHILD_TABLE_START == 0x080D_5000);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn journals_are_contiguous_below_the_secure_area() {
        assert_eq!(SETTINGS_START, 0x080D_3000);
        assert_eq!(CHILD_TABLE_START, 0x080D_5000);
        assert_eq!(APS_TABLE_START, 0x080D_9000);
        assert_eq!(SECURITY_START, 0x080D_D000);
        assert_eq!(NV_END, 0x080E_1000);
        assert_eq!((APPLICATION_END - FLASH_BASE) / 1024, 844);
    }
}
