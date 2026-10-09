//! Product-owned TLSR8258 flash partitions and Zigbee durable journals.
//!
//! Three independent two-sector journals live here, each behind its own
//! partition-bounded [`FlashRegion`] view so neither can address the other's
//! sectors:
//!
//! - the **security journal** (network state, keys, frame counters), rewritten
//!   on every frame-counter reservation, and
//! - the **APS-table journal** (bindings and groups), rewritten only when
//!   application addressing state changes, and
//! - the **child-table journal** (router/coordinator child records), rewritten
//!   only on a child lifecycle transition.

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use tlsr8258_hal::flash::{FlashError, FlashRegion};
use tlsr8258_tb04::resources::OnboardFlash;
use zigbee_runtime::aps_table_store::{APS_TABLE_JOURNAL_SECTOR_SIZE, ApsTableJournal};
use zigbee_runtime::child_store::{CHILD_JOURNAL_SECTOR_SIZE, ChildTableJournal};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

use crate::{
    APS_TABLE_PARTITION_SIZE, APS_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE,
    CHILD_TABLE_PARTITION_START, SECURITY_PARTITION_SIZE, SECURITY_PARTITION_START,
};

const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;
const CHILD_SECTOR_A: u32 = 0;
const CHILD_SECTOR_B: u32 = CHILD_JOURNAL_SECTOR_SIZE as u32;
const APS_SECTOR_A: u32 = 0;
const APS_SECTOR_B: u32 = APS_TABLE_JOURNAL_SECTOR_SIZE as u32;

const _: () = assert!(SECURITY_PARTITION_SIZE == SECURITY_JOURNAL_SECTOR_SIZE * 2);
const _: () = assert!(CHILD_TABLE_PARTITION_SIZE == CHILD_JOURNAL_SECTOR_SIZE * 2);
const _: () = assert!(APS_TABLE_PARTITION_SIZE == APS_TABLE_JOURNAL_SECTOR_SIZE * 2);

/// Exclusive ownership of the security journal partition.
pub struct SecurityPartition(());
/// Exclusive ownership of the child-table journal partition.
pub struct ChildTablePartition(());
/// Exclusive ownership of the APS binding/group journal partition.
pub struct ApsTablePartition(());

/// Split the board's single onboard-flash token into the product's disjoint
/// NV partitions.
///
/// The board crate rightly owns *one* physical flash device; how that device
/// is divided is product policy. Consuming the board token here and handing
/// back one zero-sized token per partition means the three journals cannot be
/// constructed twice or aliased, without the board knowing anything about
/// Zigbee persistence.
pub const fn split_flash(
    _token: OnboardFlash,
) -> (SecurityPartition, ChildTablePartition, ApsTablePartition) {
    (
        SecurityPartition(()),
        ChildTablePartition(()),
        ApsTablePartition(()),
    )
}

/// Product flash partition. In the router image it re-checks the SVC stack
/// guard before every erase or program, so a stack overflow can never reach
/// persistent state. The sensor retention image owns a separate LOW32K guard.
pub struct GuardedFlash(FlashRegion);

impl ErrorType for GuardedFlash {
    type Error = FlashError;
}

impl ReadNorFlash for GuardedFlash {
    const READ_SIZE: usize = FlashRegion::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl NorFlash for GuardedFlash {
    const WRITE_SIZE: usize = FlashRegion::WRITE_SIZE;
    const ERASE_SIZE: usize = FlashRegion::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        #[cfg(feature = "router")]
        crate::stack_guard::check();
        self.0.erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        #[cfg(feature = "router")]
        crate::stack_guard::check();
        self.0.write(offset, bytes)
    }
}

/// Partition-bounded view of the security journal region.
pub type SecurityFlash = GuardedFlash;

const fn security_flash(_token: SecurityPartition) -> SecurityFlash {
    // SAFETY: `SecurityPartition` is minted once by `split_flash`, which
    // consumed the board's unique `OnboardFlash` token, so this is the only
    // handle over the security sectors. The crate-level const asserts keep
    // the window inside the fitted flash, after the firmware image limit and
    // below the factory EUI sector.
    GuardedFlash(unsafe { FlashRegion::new(SECURITY_PARTITION_START, SECURITY_PARTITION_SIZE) })
}

pub type SecurityStore = SecurityStateJournal<SecurityFlash>;

pub const fn security_store(token: SecurityPartition) -> SecurityStore {
    SecurityStateJournal::new(security_flash(token), SECURITY_SECTOR_A, SECURITY_SECTOR_B)
}

/// Partition-bounded view of the child-table journal region, disjoint from
/// the security and APS partitions.
pub type ChildTableFlash = GuardedFlash;

const fn child_table_flash(_token: ChildTablePartition) -> ChildTableFlash {
    // SAFETY: as for `security_flash`; `ChildTablePartition` is unique and
    // the const asserts keep the window disjoint from the other journals.
    GuardedFlash(unsafe {
        FlashRegion::new(CHILD_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE)
    })
}

pub type ChildStore = ChildTableJournal<ChildTableFlash>;

/// Durable child-table store for a router/coordinator build.
///
/// A sensor product never constructs this, so the child-table journal code is
/// dead-code-eliminated from the sensor image.
pub const fn child_table_store(token: ChildTablePartition) -> ChildStore {
    ChildTableJournal::new(child_table_flash(token), CHILD_SECTOR_A, CHILD_SECTOR_B)
}

/// Partition-bounded view of the APS binding/group journal region.
pub type ApsTableFlash = GuardedFlash;

const fn aps_table_flash(_token: ApsTablePartition) -> ApsTableFlash {
    // SAFETY: as for `security_flash`; `ApsTablePartition` is unique and the
    // const asserts keep the window disjoint from the other journals.
    GuardedFlash(unsafe { FlashRegion::new(APS_TABLE_PARTITION_START, APS_TABLE_PARTITION_SIZE) })
}

pub type ApsTableStore = ApsTableJournal<ApsTableFlash>;

pub const fn aps_table_store(token: ApsTablePartition) -> ApsTableStore {
    ApsTableJournal::new(aps_table_flash(token), APS_SECTOR_A, APS_SECTOR_B)
}
