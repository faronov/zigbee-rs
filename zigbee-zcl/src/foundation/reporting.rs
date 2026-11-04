//! ZCL attribute reporting — configuration, engine, and wire formats.

use crate::attribute::AttributeStore;
use crate::clusters::AttributeStoreAccess;
use crate::data_types::{self, ZclDataType, ZclValue};
use crate::{AttributeId, ZclStatus};

/// Compact reporting capacity for a product with one application endpoint
/// and a small, fixed report set.
#[cfg(feature = "compact-single-endpoint")]
pub const MAX_REPORT_CONFIGS: usize = 4;
/// Reduced reporting capacity for devices with tightly constrained SRAM.
#[cfg(all(
    not(feature = "compact-single-endpoint"),
    feature = "constrained-memory"
))]
pub const MAX_REPORT_CONFIGS: usize = 8;
/// Default reporting capacity for general-purpose products.
#[cfg(all(
    not(feature = "compact-single-endpoint"),
    not(feature = "constrained-memory")
))]
pub const MAX_REPORT_CONFIGS: usize = 16;

/// Direction field in a reporting configuration record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReportDirection {
    /// Send reports (direction = 0x00).
    Send = 0x00,
    /// Receive reports (direction = 0x01).
    Receive = 0x01,
}

/// Reporting configuration for a single attribute.
#[derive(Debug, Clone)]
pub struct ReportingConfig {
    pub direction: ReportDirection,
    pub attribute_id: AttributeId,
    pub data_type: ZclDataType,
    /// Minimum reporting interval in seconds.
    pub min_interval: u16,
    /// Maximum interval: 0 disables periodic reports; 0xFFFF disables automatic reports.
    pub max_interval: u16,
    /// Minimum change to trigger a report (for analog types).
    pub reportable_change: Option<ZclValue>,
}

/// Configure Reporting request (command 0x06).
#[derive(Debug, Clone)]
pub struct ConfigureReportingRequest {
    pub configs: heapless::Vec<ReportingConfig, MAX_REPORT_CONFIGS>,
}

/// A single status record in the Configure Reporting Response.
#[derive(Debug, Clone)]
pub struct ConfigureReportingStatusRecord {
    pub status: ZclStatus,
    pub direction: ReportDirection,
    pub attribute_id: AttributeId,
}

/// Configure Reporting response (command 0x07).
#[derive(Debug, Clone)]
pub struct ConfigureReportingResponse {
    pub records: heapless::Vec<ConfigureReportingStatusRecord, MAX_REPORT_CONFIGS>,
}

impl ConfigureReportingResponse {
    /// Serialize to ZCL payload bytes.
    ///
    /// Per ZCL spec: if all statuses are Success, send a single record with
    /// status=Success only. Otherwise, send individual status records.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        // Check if all succeeded
        let all_success = self.records.iter().all(|r| r.status == ZclStatus::Success);
        if all_success {
            if buf.is_empty() {
                return 0;
            }
            buf[0] = ZclStatus::Success as u8;
            return 1;
        }
        let mut pos = 0;
        for rec in &self.records {
            // Need 1 (status) + 1 (direction) + 2 (attr_id) = 4 bytes
            if pos + 4 > buf.len() {
                break;
            }
            buf[pos] = rec.status as u8;
            pos += 1;
            buf[pos] = rec.direction as u8;
            pos += 1;
            let b = rec.attribute_id.0.to_le_bytes();
            buf[pos] = b[0];
            buf[pos + 1] = b[1];
            pos += 2;
        }
        pos
    }
}

/// An attribute report record (used in Report Attributes command 0x0A).
#[derive(Debug, Clone)]
pub struct AttributeReport {
    pub id: AttributeId,
    pub data_type: ZclDataType,
    pub value: ZclValue,
}

/// Report Attributes payload (command 0x0A).
#[derive(Debug, Clone)]
pub struct ReportAttributes {
    pub reports: heapless::Vec<AttributeReport, MAX_REPORT_CONFIGS>,
}

impl ReportAttributes {
    /// Serialize Report Attributes payload to ZCL wire format.
    ///
    /// Only complete records are written; serialization stops at the first
    /// record that does not fit. Never panics.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for rpt in &self.reports {
            let rest = &mut buf[pos..];
            let Some(hdr) = rest.get_mut(..3) else { break };
            hdr[..2].copy_from_slice(&rpt.id.0.to_le_bytes());
            hdr[2] = rpt.data_type as u8;
            let Some(n) = rpt.value.try_serialize(&mut rest[3..]) else {
                break;
            };
            pos += 3 + n;
        }
        pos
    }

    /// Number of leading reports that [`serialize`](Self::serialize) can fit
    /// in `capacity` bytes. Callers can use this to avoid marking reports
    /// as sent that were not transmitted.
    pub fn records_fitting(&self, capacity: usize) -> usize {
        let mut used = 0;
        let mut count = 0;
        for rpt in &self.reports {
            used += 3 + rpt.value.wire_len();
            if used > capacity {
                break;
            }
            count += 1;
        }
        count
    }
}

impl ConfigureReportingRequest {
    /// Parse a non-empty sequence of complete reporting records.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let mut configs = heapless::Vec::new();
        let mut i = 0;
        while i < data.len() {
            let (config, consumed) = ReportingConfig::parse_record(&data[i..])?;
            i += consumed;
            configs.push(config).ok()?;
        }
        Some(Self { configs })
    }
}

impl ReportingConfig {
    /// Parse one Configure Reporting attribute record (ZCL wire format).
    ///
    /// Returns the configuration and the number of bytes consumed.
    pub fn parse_record(data: &[u8]) -> Option<(Self, usize)> {
        let direction = match *data.first()? {
            0x00 => ReportDirection::Send,
            0x01 => ReportDirection::Receive,
            _ => return None,
        };
        let mut i = 1;
        if i + 2 > data.len() {
            return None;
        }
        let attr_id = AttributeId(u16::from_le_bytes([data[i], data[i + 1]]));
        i += 2;

        if direction == ReportDirection::Send {
            if i + 5 > data.len() {
                return None;
            }
            let dt = ZclDataType::from_u8(data[i])?;
            i += 1;
            let min_interval = u16::from_le_bytes([data[i], data[i + 1]]);
            i += 2;
            let max_interval = u16::from_le_bytes([data[i], data[i + 1]]);
            i += 2;
            // Reportable change only for analog types
            let reportable_change = if data_types::is_analog_type(dt) {
                let (val, consumed) = ZclValue::deserialize(dt, &data[i..])?;
                i += consumed;
                Some(val)
            } else {
                None
            };
            Some((
                ReportingConfig {
                    direction,
                    attribute_id: attr_id,
                    data_type: dt,
                    min_interval,
                    max_interval,
                    reportable_change,
                },
                i,
            ))
        } else {
            // Receive direction: timeout period
            if i + 2 > data.len() {
                return None;
            }
            let timeout = u16::from_le_bytes([data[i], data[i + 1]]);
            i += 2;
            Some((
                ReportingConfig {
                    direction,
                    attribute_id: attr_id,
                    data_type: ZclDataType::NoData,
                    min_interval: 0,
                    max_interval: timeout,
                    reportable_change: None,
                },
                i,
            ))
        }
    }

    /// Serialize this configuration as one Configure Reporting attribute
    /// record (the exact inverse of [`parse_record`](Self::parse_record)).
    ///
    /// Returns `None` without a usable result when `buf` is too short or an
    /// analog Send record has no representable reportable change.
    pub fn encode_record(&self, buf: &mut [u8]) -> Option<usize> {
        let head = buf.get_mut(..3)?;
        head[0] = self.direction as u8;
        head[1..3].copy_from_slice(&self.attribute_id.0.to_le_bytes());
        if self.direction == ReportDirection::Receive {
            buf.get_mut(3..5)?
                .copy_from_slice(&self.max_interval.to_le_bytes());
            return Some(5);
        }
        let body = buf.get_mut(3..8)?;
        body[0] = self.data_type as u8;
        body[1..3].copy_from_slice(&self.min_interval.to_le_bytes());
        body[3..5].copy_from_slice(&self.max_interval.to_le_bytes());
        if !data_types::is_analog_type(self.data_type) {
            return Some(8);
        }
        let change = self.reportable_change.as_ref()?;
        let n = change.try_serialize(buf.get_mut(8..)?)?;
        Some(8 + n)
    }
}

/// Internal state for a single configured report.
#[derive(Debug, Clone)]
struct ReportState {
    endpoint: u8,
    cluster_id: u16,
    config: ReportingConfig,
    /// Seconds elapsed since the last report was sent.
    elapsed: u16,
    /// Last reported value (for change detection).
    last_value: Option<ZclValue>,
    forced: bool,
}

impl ReportState {
    fn report_due(&self, current: &ZclValue) -> bool {
        if self.forced {
            return true;
        }
        if self.config.max_interval == 0xFFFF {
            return false;
        }
        if self.config.max_interval != 0 && self.elapsed >= self.config.max_interval {
            return true;
        }
        if self.elapsed < self.config.min_interval {
            return false;
        }
        match (&self.last_value, &self.config.reportable_change) {
            (Some(last), Some(change)) => current.exceeds_threshold(last, change),
            (Some(last), None) => last != current,
            (None, _) => true,
        }
    }

    fn reported(&mut self, current: &ZclValue) {
        self.elapsed = 0;
        self.last_value = Some(current.clone());
        self.forced = false;
    }
}

/// Engine that tracks configured reports and decides when to generate them.
#[derive(Debug)]
pub struct ReportingEngine {
    states: heapless::Vec<ReportState, MAX_REPORT_CONFIGS>,
}

impl Default for ReportingEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl ReportingEngine {
    pub const fn new() -> Self {
        Self {
            states: heapless::Vec::new(),
        }
    }

    /// Add or replace a reporting configuration (legacy, no cluster tracking).
    pub fn configure(&mut self, config: ReportingConfig) -> Result<(), ZclStatus> {
        self.configure_for_cluster(0, 0, config)
    }

    /// Add or replace a reporting configuration for a specific cluster.
    pub fn configure_for_cluster(
        &mut self,
        endpoint: u8,
        cluster_id: u16,
        config: ReportingConfig,
    ) -> Result<(), ZclStatus> {
        // Replace existing config for the same attribute on the same cluster
        for state in self.states.iter_mut() {
            if state.endpoint == endpoint
                && state.cluster_id == cluster_id
                && state.config.attribute_id == config.attribute_id
                && state.config.direction == config.direction
            {
                state.config = config;
                state.elapsed = 0;
                state.last_value = None;
                state.forced = false;
                return Ok(());
            }
        }
        self.states
            .push(ReportState {
                endpoint,
                cluster_id,
                config,
                elapsed: 0,
                last_value: None,
                forced: false,
            })
            .map_err(|_| ZclStatus::InsufficientSpace)
    }

    /// Advance all timers by `elapsed_secs`.
    pub fn tick(&mut self, elapsed_secs: u16) {
        for state in self.states.iter_mut() {
            state.elapsed = state.elapsed.saturating_add(elapsed_secs);
        }
    }

    /// Make every locally-sent report due on the next reporting pass.
    ///
    /// This bypasses the configured minimum interval and value-change
    /// threshold exactly once, including when automatic reporting is disabled.
    /// It is intended for explicit operator actions such as a device button
    /// requesting an immediate telemetry snapshot.
    pub fn force_all_due(&mut self) {
        for state in self.states.iter_mut() {
            if state.config.direction == ReportDirection::Send {
                state.forced = true;
            }
        }
    }

    /// Check all configured reports and generate a `ReportAttributes` payload
    /// if any reports are due.
    pub fn check_and_report<const N: usize>(
        &mut self,
        store: &AttributeStore<N>,
    ) -> Option<ReportAttributes> {
        self.check_and_report_filtered(None, None, store)
    }

    /// Check reports for a specific cluster and generate a `ReportAttributes`
    /// payload if any are due.
    pub fn check_and_report_cluster<const N: usize>(
        &mut self,
        endpoint: u8,
        cluster_id: u16,
        store: &AttributeStore<N>,
    ) -> Option<ReportAttributes> {
        self.check_and_report_filtered(Some(endpoint), Some(cluster_id), store)
    }

    fn check_and_report_filtered<const N: usize>(
        &mut self,
        filter_ep: Option<u8>,
        filter_cluster: Option<u16>,
        store: &AttributeStore<N>,
    ) -> Option<ReportAttributes> {
        let mut reports: heapless::Vec<AttributeReport, MAX_REPORT_CONFIGS> = heapless::Vec::new();

        for state in self.states.iter_mut() {
            if state.config.direction != ReportDirection::Send {
                continue;
            }

            // Filter by endpoint/cluster if specified
            if let Some(ep) = filter_ep
                && state.endpoint != ep
            {
                continue;
            }
            if let Some(cl) = filter_cluster
                && state.cluster_id != cl
            {
                continue;
            }

            let current = store.get(state.config.attribute_id);
            let current = match current {
                Some(v) => v,
                None => continue,
            };

            if state.report_due(current) {
                state.reported(current);
                if let Some(def) = store.find(state.config.attribute_id) {
                    let _ = reports.push(AttributeReport {
                        id: state.config.attribute_id,
                        data_type: def.data_type,
                        value: current.clone(),
                    });
                }
            }
        }

        if reports.is_empty() {
            None
        } else {
            Some(ReportAttributes { reports })
        }
    }

    /// Check if any reporting has been configured for a specific cluster.
    ///
    /// Returns `true` if at least one attribute on this cluster/endpoint has
    /// a reporting configuration. Useful for detecting when ZHA has completed
    /// its Configure Reporting step for a given cluster (interview detection).
    pub fn has_cluster_configured(&self, endpoint: u8, cluster_id: u16) -> bool {
        self.states
            .iter()
            .any(|s| s.endpoint == endpoint && s.cluster_id == cluster_id)
    }

    /// Count how many distinct clusters have reporting configured on an endpoint.
    pub fn configured_cluster_count(&self, endpoint: u8) -> usize {
        let mut count = 0u16;
        let mut seen: heapless::Vec<u16, MAX_REPORT_CONFIGS> = heapless::Vec::new();
        for s in self.states.iter() {
            if s.endpoint == endpoint && !seen.contains(&s.cluster_id) {
                let _ = seen.push(s.cluster_id);
                count += 1;
            }
        }
        count as usize
    }

    /// Look up the reporting configuration for a specific attribute.
    pub fn get_config(
        &self,
        endpoint: u8,
        cluster_id: u16,
        direction: ReportDirection,
        attr_id: AttributeId,
    ) -> Option<&ReportingConfig> {
        self.states
            .iter()
            .find(|s| {
                s.endpoint == endpoint
                    && s.cluster_id == cluster_id
                    && s.config.direction == direction
                    && s.config.attribute_id == attr_id
            })
            .map(|s| &s.config)
    }

    /// Every stored configuration as `(endpoint, cluster_id, config)`, in
    /// insertion order. Used to snapshot the engine for persistence.
    pub fn configs(&self) -> impl Iterator<Item = (u8, u16, &ReportingConfig)> {
        self.states
            .iter()
            .map(|s| (s.endpoint, s.cluster_id, &s.config))
    }

    /// Check reports for a cluster using a type-erased attribute store (trait object).
    ///
    /// Appends due reports to the provided `out` vec. Used by the runtime to
    /// work with `dyn AttributeStoreAccess` without knowing the concrete `N`.
    pub fn check_and_collect_dyn(
        &mut self,
        endpoint: u8,
        cluster_id: u16,
        store: &dyn AttributeStoreAccess,
        out: &mut heapless::Vec<AttributeReport, MAX_REPORT_CONFIGS>,
    ) {
        for state in self.states.iter_mut() {
            if state.config.direction != ReportDirection::Send {
                continue;
            }
            if state.endpoint != endpoint || state.cluster_id != cluster_id {
                continue;
            }

            let current = match store.get(state.config.attribute_id) {
                Some(v) => v,
                None => continue,
            };

            if state.report_due(current) {
                state.reported(current);
                if let Some(def) = store.find(state.config.attribute_id) {
                    let _ = out.push(AttributeReport {
                        id: state.config.attribute_id,
                        data_type: def.data_type,
                        value: current.clone(),
                    });
                }
            }
        }
    }
}

/// Whether a data type is "analog" (supports reportable change).
fn is_analog_type(dt: ZclDataType) -> bool {
    data_types::is_analog_type(dt)
}

/// Read Reporting Configuration request (command 0x08).
#[derive(Debug, Clone)]
pub struct ReadReportingConfigRequest {
    pub records: heapless::Vec<ReadReportingConfigRecord, MAX_REPORT_CONFIGS>,
}

/// A single record in the Read Reporting Configuration request.
#[derive(Debug, Clone)]
pub struct ReadReportingConfigRecord {
    pub direction: ReportDirection,
    pub attribute_id: AttributeId,
}

impl ReadReportingConfigRequest {
    /// Parse a non-empty sequence of complete reporting records.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.is_empty() || !data.len().is_multiple_of(3) {
            return None;
        }
        let mut records = heapless::Vec::new();
        let mut i = 0;
        while i + 2 < data.len() {
            let direction = match data[i] {
                0x00 => ReportDirection::Send,
                0x01 => ReportDirection::Receive,
                _ => return None,
            };
            i += 1;
            let attr_id = AttributeId(u16::from_le_bytes([data[i], data[i + 1]]));
            i += 2;
            records
                .push(ReadReportingConfigRecord {
                    direction,
                    attribute_id: attr_id,
                })
                .ok()?;
        }
        Some(Self { records })
    }
}

/// Read Reporting Configuration Response (command 0x09).
#[derive(Debug, Clone)]
pub struct ReadReportingConfigResponse {
    pub records: heapless::Vec<ReadReportingConfigResponseRecord, MAX_REPORT_CONFIGS>,
}

/// A single record in the Read Reporting Configuration Response.
#[derive(Debug, Clone)]
pub struct ReadReportingConfigResponseRecord {
    pub status: ZclStatus,
    pub direction: ReportDirection,
    pub attribute_id: AttributeId,
    /// Present only when status == Success and direction == Send.
    pub config: Option<ReportingConfig>,
    /// Timeout period, present when status == Success and direction == Receive.
    pub timeout: Option<u16>,
}

impl ReadReportingConfigResponse {
    /// Serialize the response to ZCL payload bytes.
    /// Direction-aware: Send records get data_type/min/max/change,
    /// Receive records get timeout only (per ZCL spec §2.5.7.1.6).
    ///
    /// Only complete records are written: serialization stops at the first
    /// record that does not fit, so the payload carries as many records as
    /// fit and stays well-formed. Never panics.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for rec in &self.records {
            match rec.serialize_into(&mut buf[pos..]) {
                Some(n) => pos += n,
                None => break,
            }
        }
        pos
    }
}

impl ReadReportingConfigResponseRecord {
    /// Serialize one complete record, or `None` if it does not fit.
    fn serialize_into(&self, buf: &mut [u8]) -> Option<usize> {
        let hdr = buf.get_mut(..4)?;
        hdr[1] = self.direction as u8;
        hdr[2..4].copy_from_slice(&self.attribute_id.0.to_le_bytes());
        let mut status = self.status;
        let mut pos = 4;
        if status == ZclStatus::Success {
            match (self.direction, &self.config, self.timeout) {
                (ReportDirection::Send, Some(cfg), _) => {
                    let b = buf.get_mut(4..9)?;
                    b[0] = cfg.data_type as u8;
                    b[1..3].copy_from_slice(&cfg.min_interval.to_le_bytes());
                    b[3..5].copy_from_slice(&cfg.max_interval.to_le_bytes());
                    pos = 9;
                    // The reportable change field is present for analog
                    // types only, and then it is mandatory.
                    if is_analog_type(cfg.data_type) {
                        match &cfg.reportable_change {
                            Some(change) => pos += change.try_serialize(&mut buf[9..])?,
                            None => {
                                status = ZclStatus::Failure;
                                pos = 4;
                            }
                        }
                    }
                }
                (ReportDirection::Receive, _, Some(timeout)) => {
                    buf.get_mut(4..6)?.copy_from_slice(&timeout.to_le_bytes());
                    pos = 6;
                }
                // A success record without its body cannot be encoded.
                _ => status = ZclStatus::Failure,
            }
        }
        buf[0] = status as u8;
        Some(pos)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::attribute::{AttributeAccess, AttributeDefinition};

    fn config(attribute_id: u16) -> ReportingConfig {
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(attribute_id),
            data_type: ZclDataType::U16,
            min_interval: 1,
            max_interval: 60,
            reportable_change: Some(ZclValue::U16(1)),
        }
    }

    #[test]
    fn configure_reporting_rejects_every_incomplete_record_prefix() {
        for complete in [
            &[0, 0, 0, 0x21, 30, 0, 60, 0, 1, 0][..],
            &[1, 0, 0, 30, 0][..],
        ] {
            for len in 0..complete.len() {
                assert!(ConfigureReportingRequest::parse(&complete[..len]).is_none());
            }
            assert_eq!(
                ConfigureReportingRequest::parse(complete)
                    .unwrap()
                    .configs
                    .len(),
                1
            );
        }
    }

    #[test]
    fn configure_reporting_does_not_accept_a_valid_prefix_with_a_truncated_tail() {
        let payload = [0, 0, 0, 0x21, 30, 0, 60, 0, 1, 0, 1, 1, 0, 30, 0];
        for len in 11..payload.len() {
            assert!(ConfigureReportingRequest::parse(&payload[..len]).is_none());
        }
        assert_eq!(
            ConfigureReportingRequest::parse(&payload)
                .unwrap()
                .configs
                .len(),
            2
        );
    }

    #[test]
    fn read_reporting_rejects_empty_payload_and_trailing_record_bytes() {
        let payload = [0, 0, 0, 1, 1, 0];
        for len in [0, 1, 2, 4, 5] {
            assert!(ReadReportingConfigRequest::parse(&payload[..len]).is_none());
        }
        assert_eq!(
            ReadReportingConfigRequest::parse(&payload)
                .unwrap()
                .records
                .len(),
            2
        );
    }

    fn fixture(min: u16, max: u16) -> (ReportingEngine, AttributeStore<1>) {
        let mut cfg = config(0);
        cfg.min_interval = min;
        cfg.max_interval = max;
        let mut engine = ReportingEngine::new();
        engine.configure_for_cluster(1, 0x0402, cfg).unwrap();
        let mut store = AttributeStore::new();
        store
            .register(
                AttributeDefinition {
                    id: AttributeId(0),
                    data_type: ZclDataType::U16,
                    access: AttributeAccess::Reportable,
                    name: "measured",
                },
                ZclValue::U16(100),
            )
            .unwrap();
        (engine, store)
    }

    fn reports_due(engine: &mut ReportingEngine, store: &AttributeStore<1>, dynamic: bool) -> bool {
        if dynamic {
            let mut out = heapless::Vec::new();
            engine.check_and_collect_dyn(1, 0x0402, store, &mut out);
            !out.is_empty()
        } else {
            engine.check_and_report_cluster(1, 0x0402, store).is_some()
        }
    }

    #[test]
    fn zero_max_reports_changes_only_after_the_minimum_interval() {
        for dynamic in [false, true] {
            let (mut engine, mut store) = fixture(30, 0);
            assert!(!reports_due(&mut engine, &store, dynamic));
            engine.tick(29);
            assert!(!reports_due(&mut engine, &store, dynamic));
            engine.tick(1);
            assert!(reports_due(&mut engine, &store, dynamic));
            assert!(!reports_due(&mut engine, &store, dynamic));
            store.set_raw(AttributeId(0), ZclValue::U16(101)).unwrap();
            engine.tick(29);
            assert!(!reports_due(&mut engine, &store, dynamic));
            engine.tick(1);
            assert!(reports_due(&mut engine, &store, dynamic));
            engine.tick(u16::MAX);
            assert!(!reports_due(&mut engine, &store, dynamic));
        }
    }

    #[test]
    fn disabled_reporting_ignores_initial_value_changes_and_elapsed_saturation() {
        for dynamic in [false, true] {
            let (mut engine, mut store) = fixture(0, 0xFFFF);
            assert!(!reports_due(&mut engine, &store, dynamic));
            store.set_raw(AttributeId(0), ZclValue::U16(101)).unwrap();
            engine.tick(u16::MAX);
            assert!(!reports_due(&mut engine, &store, dynamic));
            engine.tick(1);
            assert!(!reports_due(&mut engine, &store, dynamic));
        }
    }

    #[test]
    fn explicit_force_reports_once_even_when_automatic_reporting_is_disabled() {
        for dynamic in [false, true] {
            for max in [0, 600, 0xFFFF] {
                let (mut engine, store) = fixture(30, max);
                engine.force_all_due();
                assert!(reports_due(&mut engine, &store, dynamic));
                assert!(!reports_due(&mut engine, &store, dynamic));
                engine.tick(29);
                assert!(!reports_due(&mut engine, &store, dynamic));
            }
        }
    }

    #[test]
    fn reconfiguring_clears_a_pending_explicit_force() {
        for dynamic in [false, true] {
            let (mut engine, store) = fixture(30, 60);
            engine.force_all_due();
            let mut cfg = config(0);
            cfg.max_interval = 0xFFFF;
            engine.configure_for_cluster(1, 0x0402, cfg).unwrap();
            assert!(!reports_due(&mut engine, &store, dynamic));
        }
    }

    #[test]
    fn ordinary_periodic_deadlines_still_report_unchanged_values() {
        for dynamic in [false, true] {
            let (mut engine, store) = fixture(30, 60);
            engine.tick(30);
            assert!(reports_due(&mut engine, &store, dynamic));
            engine.tick(59);
            assert!(!reports_due(&mut engine, &store, dynamic));
            engine.tick(1);
            assert!(reports_due(&mut engine, &store, dynamic));
        }
    }

    #[test]
    fn capacity_features_select_the_smallest_requested_table() {
        let expected = if cfg!(feature = "compact-single-endpoint") {
            4
        } else if cfg!(feature = "constrained-memory") {
            8
        } else {
            16
        };
        assert_eq!(MAX_REPORT_CONFIGS, expected);
    }

    #[test]
    fn reporting_overflow_is_explicit() {
        let mut engine = ReportingEngine::new();
        for index in 0..MAX_REPORT_CONFIGS {
            assert_eq!(
                engine.configure_for_cluster(1, index as u16, config(index as u16)),
                Ok(())
            );
        }
        assert_eq!(
            engine.configure_for_cluster(
                1,
                MAX_REPORT_CONFIGS as u16,
                config(MAX_REPORT_CONFIGS as u16),
            ),
            Err(ZclStatus::InsufficientSpace)
        );
    }

    #[test]
    fn report_attributes_serializes_complete_records_only() {
        let s = ZclValue::CharString(heapless::Vec::from_slice(&[b'r'; 32]).unwrap());
        let mut reports = heapless::Vec::new();
        for id in 0..3u16 {
            reports
                .push(AttributeReport {
                    id: AttributeId(id),
                    data_type: ZclDataType::CharString,
                    value: s.clone(),
                })
                .unwrap();
        }
        let rpt = ReportAttributes { reports };
        for cap in 0..128 {
            let mut buf = [0u8; 128];
            let n = rpt.serialize(&mut buf[..cap]);
            // 36 bytes per record (2 id + 1 type + 1 len + 32)
            assert_eq!(n, (cap / 36).min(3) * 36, "cap {cap}");
            assert_eq!(rpt.records_fitting(cap), n / 36);
            for rec in buf[..n].chunks(36) {
                assert_eq!(rec[2], ZclDataType::CharString as u8);
                assert_eq!(rec[3], 32);
            }
        }
    }

    #[test]
    fn read_reporting_config_response_never_panics_and_stays_well_formed() {
        let mut records = heapless::Vec::new();
        let mut cfg64 = config(1);
        cfg64.data_type = ZclDataType::U64;
        cfg64.reportable_change = Some(ZclValue::U64(5));
        for (i, (dir, cfg, timeout)) in [
            (ReportDirection::Send, Some(cfg64), None),
            (ReportDirection::Receive, None, Some(30u16)),
            (ReportDirection::Send, Some(config(2)), None),
        ]
        .into_iter()
        .enumerate()
        {
            records
                .push(ReadReportingConfigResponseRecord {
                    status: ZclStatus::Success,
                    direction: dir,
                    attribute_id: AttributeId(i as u16),
                    config: cfg,
                    timeout,
                })
                .unwrap();
        }
        let resp = ReadReportingConfigResponse { records };
        // Record sizes: 4+5+8, 4+2, 4+5+2.
        let ends = [17usize, 23, 34];
        for cap in 0..48 {
            let mut buf = [0u8; 48];
            let n = resp.serialize(&mut buf[..cap]);
            let expected = ends
                .iter()
                .copied()
                .filter(|&e| e <= cap)
                .max()
                .unwrap_or(0);
            assert_eq!(n, expected, "cap {cap}");
        }
    }

    #[test]
    fn read_reporting_config_success_without_body_is_failure_and_digital_has_no_change() {
        let mut records = heapless::Vec::new();
        records
            .push(ReadReportingConfigResponseRecord {
                status: ZclStatus::Success,
                direction: ReportDirection::Send,
                attribute_id: AttributeId(7),
                config: None,
                timeout: None,
            })
            .unwrap();
        let mut digital = config(8);
        digital.data_type = ZclDataType::Bool;
        records
            .push(ReadReportingConfigResponseRecord {
                status: ZclStatus::Success,
                direction: ReportDirection::Send,
                attribute_id: AttributeId(8),
                config: Some(digital),
                timeout: None,
            })
            .unwrap();
        // Analog type with no stored reportable change: cannot be encoded.
        let mut analog = config(9);
        analog.reportable_change = None;
        records
            .push(ReadReportingConfigResponseRecord {
                status: ZclStatus::Success,
                direction: ReportDirection::Send,
                attribute_id: AttributeId(9),
                config: Some(analog),
                timeout: None,
            })
            .unwrap();
        let mut buf = [0u8; 32];
        let n = ReadReportingConfigResponse { records }.serialize(&mut buf);
        assert_eq!(
            &buf[..n],
            &[
                ZclStatus::Failure as u8,
                0,
                7,
                0,
                ZclStatus::Success as u8,
                0,
                8,
                0,
                ZclDataType::Bool as u8,
                1,
                0,
                60,
                0,
                ZclStatus::Failure as u8,
                0,
                9,
                0
            ]
        );
    }

    #[test]
    fn reporting_record_encode_is_the_inverse_of_parse() {
        let send = ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0),
            data_type: ZclDataType::I16,
            min_interval: 30,
            max_interval: 900,
            reportable_change: Some(ZclValue::I16(50)),
        };
        let digital = ReportingConfig {
            data_type: ZclDataType::Bool,
            reportable_change: None,
            ..send.clone()
        };
        let receive = ReportingConfig {
            direction: ReportDirection::Receive,
            attribute_id: AttributeId(7),
            data_type: ZclDataType::NoData,
            min_interval: 0,
            max_interval: 120,
            reportable_change: None,
        };
        for (config, expected_len) in [(send, 10), (digital, 8), (receive, 5)] {
            let mut buf = [0u8; 16];
            let n = config.encode_record(&mut buf).unwrap();
            assert_eq!(n, expected_len);
            let (parsed, consumed) = ReportingConfig::parse_record(&buf[..n]).unwrap();
            assert_eq!(consumed, n);
            assert_eq!(parsed.direction, config.direction);
            assert_eq!(parsed.attribute_id, config.attribute_id);
            assert_eq!(parsed.data_type, config.data_type);
            assert_eq!(parsed.min_interval, config.min_interval);
            assert_eq!(parsed.max_interval, config.max_interval);
            assert_eq!(parsed.reportable_change, config.reportable_change);
            for short in 0..n {
                assert!(config.encode_record(&mut buf[..short]).is_none());
            }
        }
    }

    #[test]
    fn analog_record_without_change_cannot_be_encoded() {
        let mut config = config(0);
        config.reportable_change = None;
        assert!(config.encode_record(&mut [0u8; 16]).is_none());
    }

    #[test]
    fn engine_lists_every_configuration_in_insertion_order() {
        let mut engine = ReportingEngine::new();
        engine.configure_for_cluster(1, 0x0402, config(0)).unwrap();
        engine.configure_for_cluster(2, 0x0405, config(1)).unwrap();
        let mut replacement = config(0);
        replacement.max_interval = 300;
        engine
            .configure_for_cluster(1, 0x0402, replacement)
            .unwrap();
        let listed: heapless::Vec<_, 4> = engine
            .configs()
            .map(|(ep, cluster, cfg)| (ep, cluster, cfg.attribute_id.0, cfg.max_interval))
            .collect();
        assert_eq!(
            listed.as_slice(),
            &[(1, 0x0402, 0, 300), (2, 0x0405, 1, 60)]
        );
    }
}
