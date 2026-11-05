//! Sleepy-sensor anomaly record journal.
//!
//! The shared lifecycle's [`sensor_sed_app::AnomalyRecord`] (which
//! keep-awake limit fired, and how often) lives in two product-owned
//! sectors of `zbnv` ([`crate::layout::ANOMALY_OFFSET`]), disjoint from the
//! security, router and reporting journals and the OTA slots. Only images
//! built with the `anomaly-record` feature open it.

use sensor_sed_app::AnomalyJournal;

use crate::flash_window::FlashWindow;
use crate::layout::{ANOMALY_OFFSET, ANOMALY_SIZE};

pub type AnomalyFlash = FlashWindow<ANOMALY_OFFSET, { ANOMALY_SIZE as usize }>;
pub type AnomalyStore = AnomalyJournal<AnomalyFlash>;

/// The journal over the product window, allowing `max_writes_per_boot`
/// appends before it stops writing until the next reset.
pub fn anomaly_store(max_writes_per_boot: u8) -> AnomalyStore {
    AnomalyJournal::new(AnomalyFlash::new(), max_writes_per_boot)
}
