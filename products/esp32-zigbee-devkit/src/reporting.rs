//! Sleepy-sensor reporting-configuration journal.
//!
//! The ZCL reporting configuration a coordinator installs with Configure
//! Reporting lives in two product-owned sectors of `zbnv`
//! ([`crate::layout::REPORTING_OFFSET`]), disjoint from the security journal,
//! the router journals and the OTA slots. Only images built with the
//! `reporting-persist` feature open it.

use zigbee_runtime::reporting_store::{ReportingJournal, ReportingPersistence};

use crate::flash_window::FlashWindow;
use crate::layout::{REPORTING_OFFSET, REPORTING_SIZE, SECTOR_SIZE};

const SECTOR: usize = SECTOR_SIZE as usize;

pub type ReportingFlash = FlashWindow<REPORTING_OFFSET, { REPORTING_SIZE as usize }>;
/// Reporting-configuration journal on two 4 KiB flash sectors.
pub type ReportingStore = ReportingJournal<ReportingFlash, SECTOR>;

/// Restore-once / write-on-change persistence over the product window.
pub fn reporting_persistence() -> ReportingPersistence<ReportingStore> {
    ReportingPersistence::new(ReportingStore::new(ReportingFlash::new(), 0, SECTOR_SIZE))
}
