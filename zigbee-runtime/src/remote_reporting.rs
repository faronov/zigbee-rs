//! Remote (client-configured) attribute-reporting interview state.
//!
//! A Zigbee coordinator or any other ZCL *client* completes its interview of
//! this device by sending global Configure Reporting (0x06, client→server)
//! commands, one per server cluster it intends to receive reports from. A
//! product's application layer wants to know when that step is finished so it
//! can leave its post-join fast-poll window, park the commissioning LED and
//! return to bounded long polling.
//!
//! That question cannot be answered from
//! [`ReportingEngine`](zigbee_zcl::foundation::reporting::ReportingEngine):
//! the engine holds *every* configuration, including the local defaults a
//! product installs itself through
//! [`ApplicationProfile::configure_default_reporting`](crate::profile::ApplicationProfile::configure_default_reporting)
//! (and through an interview-timeout fallback). Counting engine entries
//! therefore reports "the interview finished" for a device that has merely
//! configured itself, which is exactly the failure mode this module exists to
//! remove.
//!
//! [`RemoteReportingState`] is that separate, heap-free record. It tracks the
//! distinct `(endpoint, cluster_id)` pairs for which a **remote client**
//! outbound-reporting command was accepted in full, and nothing else:
//!
//! - a cluster is recorded only after a non-empty, well-formed global
//!   Configure Reporting command made entirely of Send-direction records has
//!   been fully parsed *and* every one of its status records returned
//!   [`Success`](zigbee_zcl::ZclStatus::Success);
//! - receive-only and mixed Send/Receive commands never advance outbound
//!   reporting progress, even when each record is individually accepted;
//! - an empty or malformed command, an unsupported attribute, an
//!   unreportable attribute, an invalid or disabled data type, a
//!   reporting-table capacity failure, or any other unsuccessful record
//!   leaves the state untouched;
//! - repeated commands for the same `(endpoint, cluster_id)` do not
//!   double-count;
//! - local defaults never appear here, because nothing but the incoming
//!   command path writes to it.
//!
//! The record counts *any* server cluster on the endpoint that a remote
//! client configured in full — it does not filter against the product's
//! expected cluster set, because the runtime does not own the product's
//! interview expectations. A profile-aware application must therefore count
//! only its expected IDs (via `ZigbeeNode::remote_reporting_cluster_count` or
//! `ZigbeeDevice::remote_reporting_coverage`) and check exact coverage rather
//! than comparing this generic state's
//! [`cluster_count`](RemoteReportingState::cluster_count) with a target count.
//! An application that must know a *specific* cluster was configured can ask
//! [`contains`](RemoteReportingState::contains) (exposed as
//! `ZigbeeDevice::is_cluster_remotely_configured` /
//! `ZigbeeNode::is_cluster_remotely_configured`).
//!
//! The record is live network state, not durable state: it is cleared on a
//! fresh commissioning/rejoin lifecycle (see
//! [`ZigbeeDevice::reset_remote_reporting`](crate::ZigbeeDevice::reset_remote_reporting)),
//! and is deliberately never persisted — a rejoining coordinator re-runs the
//! interview.

/// Maximum number of distinct `(endpoint, cluster)` pairs remembered.
///
/// A newly accepted distinct cluster necessarily occupies at least one entry
/// in the reporting engine, so matching that engine's capacity guarantees the
/// remote tracker cannot fill before the command itself fails with
/// `InsufficientSpace`. This also follows the `constrained-memory` feature
/// automatically instead of tying reporting capacity to the Zigbee role.
pub const MAX_REMOTE_REPORTING_CLUSTERS: usize =
    zigbee_zcl::foundation::reporting::MAX_REPORT_CONFIGS;

/// Outcome of recording one fully successful outbound Configure Reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RecordOutcome {
    /// This `(endpoint, cluster)` pair had not been configured before.
    Added,
    /// The pair was already recorded — the count is unchanged.
    AlreadyRecorded,
    /// The table is full, so this pair could not be recorded.
    ///
    /// Surfaced rather than swallowed: an application that keys interview
    /// completion off the count would otherwise wait forever with no
    /// indication of why, and the caller can log it.
    Full,
}

/// Distinct `(endpoint, cluster_id)` pairs configured by a remote ZCL client.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RemoteReportingState {
    clusters: heapless::Vec<(u8, u16), MAX_REMOTE_REPORTING_CLUSTERS>,
}

impl RemoteReportingState {
    /// Create an empty record — no remote client has configured anything.
    pub const fn new() -> Self {
        Self {
            clusters: heapless::Vec::new(),
        }
    }

    /// Record a fully successful outbound Configure Reporting for one cluster.
    ///
    /// The caller is responsible for only invoking this after a non-empty,
    /// well-formed command made entirely of Send-direction records whose every
    /// status record was `Success`.
    pub(crate) fn record(&mut self, endpoint: u8, cluster_id: u16) -> RecordOutcome {
        if self.contains(endpoint, cluster_id) {
            return RecordOutcome::AlreadyRecorded;
        }
        match self.clusters.push((endpoint, cluster_id)) {
            Ok(()) => RecordOutcome::Added,
            Err(_) => RecordOutcome::Full,
        }
    }

    /// Whether a remote client has configured reporting for this cluster.
    pub fn contains(&self, endpoint: u8, cluster_id: u16) -> bool {
        self.clusters
            .iter()
            .any(|&(ep, cluster)| ep == endpoint && cluster == cluster_id)
    }

    /// Number of distinct clusters a remote client configured on `endpoint`.
    pub fn cluster_count(&self, endpoint: u8) -> usize {
        self.clusters
            .iter()
            .filter(|&&(ep, _)| ep == endpoint)
            .count()
    }

    /// Number of distinct `(endpoint, cluster)` pairs across all endpoints.
    pub fn total_cluster_count(&self) -> usize {
        self.clusters.len()
    }

    /// Whether no remote client has configured any reporting yet.
    pub fn is_empty(&self) -> bool {
        self.clusters.is_empty()
    }

    /// Forget every recorded cluster for a new commissioning/rejoin cycle.
    pub fn clear(&mut self) {
        self.clusters.clear();
    }

    /// Iterate the recorded `(endpoint, cluster_id)` pairs.
    pub fn iter(&self) -> impl Iterator<Item = (u8, u16)> + '_ {
        self.clusters.iter().copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_record_is_empty() {
        let state = RemoteReportingState::new();
        assert!(state.is_empty());
        assert_eq!(state.cluster_count(1), 0);
        assert_eq!(state.total_cluster_count(), 0);
        assert!(!state.contains(1, 0x0402));
    }

    #[test]
    fn duplicate_clusters_do_not_double_count() {
        let mut state = RemoteReportingState::new();
        assert_eq!(state.record(1, 0x0402), RecordOutcome::Added);
        assert_eq!(state.record(1, 0x0402), RecordOutcome::AlreadyRecorded);
        assert_eq!(state.cluster_count(1), 1);
    }

    #[test]
    fn endpoints_are_tracked_separately() {
        let mut state = RemoteReportingState::new();
        assert_eq!(state.record(1, 0x0402), RecordOutcome::Added);
        assert_eq!(state.record(2, 0x0402), RecordOutcome::Added);
        assert_eq!(state.cluster_count(1), 1);
        assert_eq!(state.cluster_count(2), 1);
        assert_eq!(state.total_cluster_count(), 2);
        assert!(state.contains(2, 0x0402));
        assert!(!state.contains(3, 0x0402));
    }

    #[test]
    fn clear_resets_the_interview_record() {
        let mut state = RemoteReportingState::new();
        state.record(1, 0x0001);
        state.record(1, 0x0402);
        assert_eq!(state.cluster_count(1), 2);
        state.clear();
        assert!(state.is_empty());
        assert_eq!(state.cluster_count(1), 0);
    }

    /// Overflow is reported, not silently treated as recorded.
    #[test]
    fn a_full_table_reports_capacity_failure() {
        let mut state = RemoteReportingState::new();
        for n in 0..MAX_REMOTE_REPORTING_CLUSTERS {
            assert_eq!(state.record(1, n as u16), RecordOutcome::Added);
        }
        assert_eq!(
            state.record(1, MAX_REMOTE_REPORTING_CLUSTERS as u16),
            RecordOutcome::Full
        );
        assert_eq!(state.cluster_count(1), MAX_REMOTE_REPORTING_CLUSTERS);
        assert!(!state.contains(1, MAX_REMOTE_REPORTING_CLUSTERS as u16));
    }
}
