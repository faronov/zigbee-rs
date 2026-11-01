//! Router-role persistence, policy and profile (size probe).
//!
//! **The router does not fit the PB-03F XIP application slot** (see
//! `examples/pb03f-kit/src/bin/router.rs` for the measured numbers); this
//! module keeps the role template complete so the linker, not a silent
//! table shrink, rejects it.
//!
//! ```text
//! 0x7E000..0x80000 security journal
//! 0x7C000..0x7E000 APS journal
//! 0x7A000..0x7C000 child journal
//! 0x41000..0x7A000 reserved OTA staging (228 KiB, still > the 200,448 B slot)
//! 0x10100..0x41000 XIP application slot
//! ```
//!
//! The router journals take the top 16 KiB of the reserved (unused) OTA
//! staging area; the XIP slot is unchanged. The polling end device and the
//! End Device never touch them. The board's one `InternalFlash` token is
//! shared by the three journals through [`SharedFlash`].

use core::cell::RefCell;

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, ReadNorFlash, check_erase, check_read, check_write,
};
use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::aps_table_store::{APS_TABLE_JOURNAL_SECTOR_SIZE, ApsTableJournal};
use zigbee_runtime::child_store::{CHILD_JOURNAL_SECTOR_SIZE, ChildTableJournal};
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;
use crate::storage::{PartitionError, SECURITY_PARTITION_START, SecurityStore, security_journal};

/// Physical erase sector of the PHY62x2 SPIF flash.
pub const PHYSICAL_ERASE_SIZE: usize = 4096;

/// APS binding/group journal: two 4 KiB sectors directly below the
/// security journal.
pub const APS_TABLE_PARTITION_SIZE: usize = APS_TABLE_JOURNAL_SECTOR_SIZE * 2;
pub const APS_TABLE_PARTITION_START: u32 =
    SECURITY_PARTITION_START - APS_TABLE_PARTITION_SIZE as u32;
/// Child-table journal: two 4 KiB sectors below the APS journal.
pub const CHILD_TABLE_PARTITION_SIZE: usize = CHILD_JOURNAL_SECTOR_SIZE * 2;
pub const CHILD_TABLE_PARTITION_START: u32 =
    APS_TABLE_PARTITION_START - CHILD_TABLE_PARTITION_SIZE as u32;

const _: () = assert!(
    CHILD_TABLE_PARTITION_START as usize + CHILD_TABLE_PARTITION_SIZE
        == APS_TABLE_PARTITION_START as usize
);
const _: () = assert!(
    APS_TABLE_PARTITION_START as usize + APS_TABLE_PARTITION_SIZE
        == SECURITY_PARTITION_START as usize
);
const _: () = assert!((CHILD_TABLE_PARTITION_START as usize).is_multiple_of(PHYSICAL_ERASE_SIZE));
const _: () = assert!(CHILD_JOURNAL_SECTOR_SIZE.is_multiple_of(PHYSICAL_ERASE_SIZE));
const _: () = assert!(APS_TABLE_JOURNAL_SECTOR_SIZE.is_multiple_of(PHYSICAL_ERASE_SIZE));
// The journals stay above the XIP slot (0x41000) and leave a staging area
// larger than the slot below them.
const _: () = assert!(CHILD_TABLE_PARTITION_START == 0x0007_A000);
const _: () = assert!(CHILD_TABLE_PARTITION_START - 0x0004_1000 > 0x0003_0F00);

/// Same bounded receive slice as the end device; 5..60 s commissioning
/// retry; three failed secured rejoins fall back to fresh commissioning.
pub static ROUTER_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 3,
};

pub type RouterProfile = DeviceProfile<RangeExtender>;

/// HA Range Extender: Basic + Identify on a routing, child-capable node.
pub const fn router_profile() -> RouterProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::RANGE_EXTENDER,
        RangeExtender,
    )
}

/// A [`NorFlash`] shared through a [`RefCell`]: every operation borrows the
/// flash only for its own duration, so the three journals can use the one
/// flash token in the single-threaded executor (no ISR touches flash).
pub struct SharedFlash<'a, F> {
    flash: &'a RefCell<F>,
}

impl<'a, F> SharedFlash<'a, F> {
    pub const fn new(flash: &'a RefCell<F>) -> Self {
        Self { flash }
    }
}

impl<F: ErrorType> ErrorType for SharedFlash<'_, F> {
    type Error = F::Error;
}

impl<F: ReadNorFlash> ReadNorFlash for SharedFlash<'_, F> {
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash.borrow_mut().read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.flash.borrow().capacity()
    }
}

impl<F: NorFlash> NorFlash for SharedFlash<'_, F> {
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.flash.borrow_mut().erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.flash.borrow_mut().write(offset, bytes)
    }
}

/// Bounded view `[START, START + SIZE)` of the main flash, offsets relative
/// to `START`.
pub struct FlashWindow<F, const START: u32, const SIZE: usize> {
    flash: F,
}

impl<F, const START: u32, const SIZE: usize> FlashWindow<F, START, SIZE> {
    pub const fn new(flash: F) -> Self {
        Self { flash }
    }

    pub fn into_inner(self) -> F {
        self.flash
    }

    /// Translate a window range to its physical start address.
    pub const fn physical_offset(offset: u32, length: usize) -> Option<u32> {
        let Some(end) = (offset as usize).checked_add(length) else {
            return None;
        };
        if end > SIZE {
            return None;
        }
        START.checked_add(offset)
    }
}

impl<F: ErrorType, const START: u32, const SIZE: usize> ErrorType for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError,
{
    type Error = PartitionError<F::Error>;
}

impl<F: ReadNorFlash, const START: u32, const SIZE: usize> ReadNorFlash
    for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError,
{
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_read(self, offset, bytes.len()).map_err(layout_error)?;
        let physical =
            Self::physical_offset(offset, bytes.len()).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .read(physical, bytes)
            .map_err(PartitionError::Backend)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<F: NorFlash, const START: u32, const SIZE: usize> NorFlash for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError,
{
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(layout_error)?;
        let length = (to - from) as usize;
        let physical_from =
            Self::physical_offset(from, length).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .erase(physical_from, physical_from + (to - from))
            .map_err(PartitionError::Backend)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_write(self, offset, bytes.len()).map_err(layout_error)?;
        let physical =
            Self::physical_offset(offset, bytes.len()).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .write(physical, bytes)
            .map_err(PartitionError::Backend)
    }
}

const fn layout_error<E>(
    error: embedded_storage::nor_flash::NorFlashErrorKind,
) -> PartitionError<E> {
    match error {
        embedded_storage::nor_flash::NorFlashErrorKind::NotAligned => PartitionError::NotAligned,
        _ => PartitionError::OutOfBounds,
    }
}

pub type ChildTableFlash<'a, F> =
    FlashWindow<SharedFlash<'a, F>, CHILD_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE>;
pub type ApsTableFlash<'a, F> =
    FlashWindow<SharedFlash<'a, F>, APS_TABLE_PARTITION_START, APS_TABLE_PARTITION_SIZE>;
pub type ChildStore<'a, F> = ChildTableJournal<ChildTableFlash<'a, F>>;
pub type ApsTableStore<'a, F> = ApsTableJournal<ApsTableFlash<'a, F>>;
pub type RouterSecurityStore<'a, F> = SecurityStore<SharedFlash<'a, F>>;

/// The router's three journals over the one shared flash.
pub struct RouterStores<'a, F> {
    pub security: RouterSecurityStore<'a, F>,
    pub children: ChildStore<'a, F>,
    pub aps: ApsTableStore<'a, F>,
}

/// Split the shared main flash into the security, child and APS journals.
/// The windows are disjoint from each other and from the router image
/// (const assertions above, `link/router-tables-*.x`).
pub fn router_stores<F: NorFlash>(flash: &RefCell<F>) -> RouterStores<'_, F>
where
    F::Error: NorFlashError,
{
    RouterStores {
        security: security_journal(SharedFlash::new(flash)),
        children: ChildStore::new(
            FlashWindow::new(SharedFlash::new(flash)),
            0,
            CHILD_JOURNAL_SECTOR_SIZE as u32,
        ),
        aps: ApsTableStore::new(
            FlashWindow::new(SharedFlash::new(flash)),
            0,
            APS_TABLE_JOURNAL_SECTOR_SIZE as u32,
        ),
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_storage::nor_flash::NorFlashErrorKind;
    use std::vec;
    use std::vec::Vec;
    use zigbee_runtime::profile::ApplicationProfile;

    struct MockMainFlash {
        bytes: Vec<u8>,
    }

    impl ErrorType for MockMainFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for MockMainFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            check_read(self, offset, bytes.len())?;
            let start = offset as usize;
            bytes.copy_from_slice(&self.bytes[start..start + bytes.len()]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for MockMainFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = PHYSICAL_ERASE_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            check_erase(self, from, to)?;
            self.bytes[from as usize..to as usize].fill(0xFF);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            check_write(self, offset, bytes.len())?;
            let start = offset as usize;
            for (stored, new) in self.bytes[start..start + bytes.len()].iter_mut().zip(bytes) {
                *stored &= *new;
            }
            Ok(())
        }
    }

    #[test]
    fn router_policy_is_valid_and_bounded() {
        assert!(ROUTER_POLICY.is_valid());
        assert!(ROUTER_POLICY.max_receive_slice_us <= 20_000);
    }

    #[test]
    fn router_advertises_a_range_extender() {
        let profile = router_profile();
        assert_eq!(profile.device_id(), DeviceId::RANGE_EXTENDER);
        assert_eq!(profile.expected_report_clusters(), 0);
    }

    #[test]
    fn router_journals_sit_below_the_security_journal() {
        assert_eq!(
            APS_TABLE_PARTITION_START as usize + APS_TABLE_PARTITION_SIZE,
            SECURITY_PARTITION_START as usize
        );
        assert_eq!(CHILD_TABLE_PARTITION_START, 0x0007_A000);
        let script = include_str!("../link/router-tables.x");
        assert!(script.contains("_router_tables_start = 0x0007A000;"));
    }

    #[test]
    fn windows_confine_writes_to_their_partition() {
        let flash = RefCell::new(MockMainFlash {
            bytes: vec![0xFF; crate::storage::FLASH_CAPACITY],
        });
        let stores = router_stores(&flash);
        let mut children = stores.children.into_storage();
        children.erase(0, PHYSICAL_ERASE_SIZE as u32).unwrap();
        children.write(0, &[0xA5; 4]).unwrap();
        assert_eq!(
            children.write(CHILD_TABLE_PARTITION_SIZE as u32, &[0; 4]),
            Err(PartitionError::OutOfBounds)
        );
        let bytes = &flash.borrow().bytes;
        let start = CHILD_TABLE_PARTITION_START as usize;
        assert_eq!(&bytes[start..start + 4], &[0xA5; 4]);
        assert!(bytes[..start].iter().all(|byte| *byte == 0xFF));
        assert!(bytes[start + 4..].iter().all(|byte| *byte == 0xFF));
    }
}
