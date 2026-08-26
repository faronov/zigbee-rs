//! Product-owned flash layout and persistence policy for TLSR8258 TB-04 firmware.

#![no_std]

#[cfg(all(target_arch = "tc32", feature = "router"))]
pub mod router;
#[cfg(feature = "sensor")]
pub mod sensor;
#[cfg(target_arch = "tc32")]
pub mod storage;

pub const FLASH_CAPACITY: usize = tlsr8258_tb04::ONBOARD_FLASH_CAPACITY;
pub const SECURITY_PARTITION_START: u32 = 0x0007_4000;
pub const SECURITY_PARTITION_SIZE: usize = 8 * 1024;

/// Product-owned child-table journal partition (two 4 KiB erase sectors).
///
/// Kept strictly before the security partition and below Telink's factory
/// EUI/config sectors at `0x76000..0x78000`.
pub const CHILD_TABLE_PARTITION_START: u32 = 0x0007_2000;
pub const CHILD_TABLE_PARTITION_SIZE: usize = 8 * 1024;
const FACTORY_EUI_SECTOR_START: u32 = 0x0007_6000;

const _: () =
    assert!(SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE <= FLASH_CAPACITY);
const _: () = assert!(
    CHILD_TABLE_PARTITION_START + CHILD_TABLE_PARTITION_SIZE as u32 <= SECURITY_PARTITION_START
);
const _: () =
    assert!(SECURITY_PARTITION_START + SECURITY_PARTITION_SIZE as u32 <= FACTORY_EUI_SECTOR_START);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn security_partition_matches_the_product_linker_layout() {
        assert_eq!(SECURITY_PARTITION_START, 0x74000);
        assert_eq!(SECURITY_PARTITION_SIZE, 0x2000);
        assert_eq!(
            SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE,
            0x76000
        );
    }

    #[test]
    fn child_table_partition_precedes_security_and_factory_data() {
        assert_eq!(CHILD_TABLE_PARTITION_START, 0x72000);
        assert_eq!(CHILD_TABLE_PARTITION_SIZE, 0x2000);
        assert_eq!(
            CHILD_TABLE_PARTITION_START as usize + CHILD_TABLE_PARTITION_SIZE,
            0x74000
        );
        assert!(
            CHILD_TABLE_PARTITION_START + CHILD_TABLE_PARTITION_SIZE as u32
                <= SECURITY_PARTITION_START,
            "the two journals must never share an erase sector"
        );
        assert_eq!(
            SECURITY_PARTITION_START + SECURITY_PARTITION_SIZE as u32,
            FACTORY_EUI_SECTOR_START,
            "NV journals must stop before Telink's factory EUI-64 sector"
        );
    }
}
