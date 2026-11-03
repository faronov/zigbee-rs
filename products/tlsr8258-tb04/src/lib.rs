//! Product-owned flash layout and persistence policy for TLSR8258 TB-04 firmware.
//!
//! One product, three node roles, each its own image: the polling sleepy
//! sensor (`sensor`), the always-on End Device (`end-device`) and the
//! parent router (`router`). All three link the same partition map, so a
//! reflash between roles never moves a journal.

#![no_std]

#[cfg(target_arch = "tc32")]
pub mod boot;
#[cfg(feature = "end-device")]
pub mod end_device;
#[cfg(all(target_arch = "tc32", feature = "router"))]
pub mod router;
#[cfg(feature = "sensor")]
pub mod sensor;
#[cfg(all(target_arch = "tc32", feature = "router"))]
pub mod stack_guard;
#[cfg(all(target_arch = "tc32", any(feature = "router", feature = "end-device")))]
pub mod status_leds;
#[cfg(target_arch = "tc32")]
pub mod storage;

/// Security-journal role tags (`SecurityStateJournal::select_role_tag`).
///
/// All TB-04 role images share the security journal at
/// [`SECURITY_PARTITION_START`], so outgoing frame-counter limits survive a
/// reflash between roles. The sensor and router images predate the tags and
/// write none; each role image also derives its own IEEE address from the
/// factory EUI-64, so `reset_security_state_if_identity_changed` already
/// keeps the counter limits and clears foreign network state when they
/// meet. The End Device, the first tagged TB-04 role, additionally selects
/// [`role::END_DEVICE`]; values follow the repository-wide role template.
pub mod role {
    pub const END_DEVICE: u16 = 0x5202;
}

pub const FLASH_CAPACITY: usize = tlsr8258_tb04::ONBOARD_FLASH_CAPACITY;
pub const SECURITY_PARTITION_START: u32 = 0x0007_4000;
pub const SECURITY_PARTITION_SIZE: usize = 8 * 1024;
pub const APS_TABLE_PARTITION_START: u32 = 0x0007_0000;
pub const APS_TABLE_PARTITION_SIZE: usize = 8 * 1024;

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
    APS_TABLE_PARTITION_START + APS_TABLE_PARTITION_SIZE as u32 <= CHILD_TABLE_PARTITION_START
);
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
        assert_eq!(APS_TABLE_PARTITION_START, 0x70000);
        assert_eq!(APS_TABLE_PARTITION_SIZE, 0x2000);
        assert_eq!(
            APS_TABLE_PARTITION_START as usize + APS_TABLE_PARTITION_SIZE,
            0x72000
        );
        assert_eq!(CHILD_TABLE_PARTITION_START, 0x72000);
        assert_eq!(CHILD_TABLE_PARTITION_SIZE, 0x2000);
        assert_eq!(
            CHILD_TABLE_PARTITION_START as usize + CHILD_TABLE_PARTITION_SIZE,
            0x74000
        );
        assert!(
            APS_TABLE_PARTITION_START + APS_TABLE_PARTITION_SIZE as u32
                <= CHILD_TABLE_PARTITION_START,
            "the APS and child-table journals must never share an erase sector"
        );
        assert!(
            CHILD_TABLE_PARTITION_START + CHILD_TABLE_PARTITION_SIZE as u32
                <= SECURITY_PARTITION_START,
            "the child-table and security journals must never share an erase sector"
        );
        assert_eq!(
            SECURITY_PARTITION_START + SECURITY_PARTITION_SIZE as u32,
            FACTORY_EUI_SECTOR_START,
            "NV journals must stop before Telink's factory EUI-64 sector"
        );
    }

    #[cfg(feature = "router")]
    #[test]
    fn range_extender_uses_the_compact_single_endpoint_capacity() {
        assert_eq!(zigbee_runtime::MAX_ENDPOINTS, 2);
        assert_eq!(zigbee_zcl::foundation::reporting::MAX_REPORT_CONFIGS, 4);
    }
}
