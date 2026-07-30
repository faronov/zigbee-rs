//! Product-owned flash layout and persistence policy for TLSR8258 TB-04 firmware.

#![no_std]

#[cfg(target_arch = "tc32")]
pub mod storage;

pub const FLASH_CAPACITY: usize = tlsr8258_tb04::ONBOARD_FLASH_CAPACITY;
pub const SECURITY_PARTITION_START: u32 = 0x0007_4000;
pub const SECURITY_PARTITION_SIZE: usize = 8 * 1024;

const _: () =
    assert!(SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE <= FLASH_CAPACITY);

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
}
