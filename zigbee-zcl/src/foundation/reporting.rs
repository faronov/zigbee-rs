//! ZCL attribute reporting — configuration, engine, and wire formats.

use crate::attribute::AttributeStore;
use crate::data_types::{self, ZclDataType, ZclValue};
use crate::{AttributeId, ZclStatus};

/// Maximum number of reporting configurations tracked simultaneously.
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
    /// Maximum reporting interval in seconds (0xFFFF = no periodic reporting).
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
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for rpt in &self.reports {
            let b = rpt.id.0.to_le_bytes();
            buf[pos] = b[0];
            buf[pos + 1] = b[1];
            pos += 2;
            buf[pos] = rpt.data_type as u8;
            pos += 1;
            pos += rpt.value.serialize(&mut buf[pos..]);
        }
        pos
    }
}

impl ConfigureReportingRequest {
    /// Parse from ZCL payload bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
        let mut configs = heapless::Vec::new();
        let mut i = 0;
        while i < data.len() {
            let direction = match data[i] {
                0x00 => ReportDirection::Send,
                0x01 => ReportDirection::Receive,
                _ => return None,
            };
            i += 1;
            if i + 2 > data.len() {
                break;
            }
            let attr_id = AttributeId(u16::from_le_bytes([data[i], data[i + 1]]));
            i += 2;

            if direction == ReportDirection::Send {
                if i + 5 > data.len() {
                    break;
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
                configs
                    .push(ReportingConfig {
                        direction,
                        attribute_id: attr_id,
                        data_type: dt,
                        min_interval,
                        max_interval,
                        reportable_change,
                    })
                    .ok()?;
            } else {
                // Receive direction: timeout period
                if i + 2 > data.len() {
                    break;
                }
                let timeout = u16::from_le_bytes([data[i], data[i + 1]]);
                i += 2;
                configs
                    .push(ReportingConfig {
                        direction,
                        attribute_id: attr_id,
                        data_type: ZclDataType::NoData,
                        min_interval: 0,
                        max_interval: timeout,
                        reportable_change: None,
                    })
                    .ok()?;
            }
        }
        Some(Self { configs })
    }
}

/// Internal state for a single configured report.
#[derive(Debug, Clone)]
struct ReportState {
    config: ReportingConfig,
    /// Seconds elapsed since the last report was sent.
    elapsed: u16,
    /// Last reported value (for change detection).
    last_value: Option<ZclValue>,
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

    /// Add or replace a reporting configuration.
    pub fn configure(&mut self, config: ReportingConfig) -> Result<(), ZclStatus> {
        // Replace existing config for the same attribute
        for state in self.states.iter_mut() {
            if state.config.attribute_id == config.attribute_id
                && state.config.direction == config.direction
            {
                state.config = config;
                state.elapsed = 0;
                state.last_value = None;
                return Ok(());
            }
        }
        self.states
            .push(ReportState {
                config,
                elapsed: 0,
                last_value: None,
            })
            .map_err(|_| ZclStatus::InsufficientSpace)
    }

    /// Advance all timers by `elapsed_secs`.
    pub fn tick(&mut self, elapsed_secs: u16) {
        for state in self.states.iter_mut() {
            state.elapsed = state.elapsed.saturating_add(elapsed_secs);
        }
    }

    /// Check all configured reports and generate a `ReportAttributes` payload
    /// if any reports are due.
    pub fn check_and_report<const N: usize>(
        &mut self,
        store: &AttributeStore<N>,
    ) -> Option<ReportAttributes> {
        let mut reports: heapless::Vec<AttributeReport, MAX_REPORT_CONFIGS> = heapless::Vec::new();

        for state in self.states.iter_mut() {
            if state.config.direction != ReportDirection::Send {
                continue;
            }

            let current = store.get(state.config.attribute_id);
            let current = match current {
                Some(v) => v,
                None => continue,
            };

            let mut should_report = false;

            // Max interval expired?
            if state.config.max_interval != 0xFFFF && state.elapsed >= state.config.max_interval {
                should_report = true;
            }

            // Value changed beyond threshold?
            if state.elapsed >= state.config.min_interval {
                if let Some(ref last) = state.last_value {
                    if last != current {
                        if let Some(ref _change) = state.config.reportable_change {
                            // Simplified: any change triggers report.
                            should_report = true;
                        } else {
                            should_report = true;
                        }
                    }
                } else {
                    // No previous value — first report.
                    should_report = true;
                }
            }

            if should_report {
                state.elapsed = 0;
                state.last_value = Some(current.clone());
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
}

/// Whether a data type is "analog" (supports reportable change).
#[allow(dead_code)]
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
    /// Parse from ZCL payload bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
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
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for rec in &self.records {
            buf[pos] = rec.status as u8;
            pos += 1;
            buf[pos] = rec.direction as u8;
            pos += 1;
            let b = rec.attribute_id.0.to_le_bytes();
            buf[pos] = b[0];
            buf[pos + 1] = b[1];
            pos += 2;

            if rec.status == ZclStatus::Success {
                if let Some(ref cfg) = rec.config {
                    buf[pos] = cfg.data_type as u8;
                    pos += 1;
                    let b = cfg.min_interval.to_le_bytes();
                    buf[pos] = b[0];
                    buf[pos + 1] = b[1];
                    pos += 2;
                    let b = cfg.max_interval.to_le_bytes();
                    buf[pos] = b[0];
                    buf[pos + 1] = b[1];
                    pos += 2;
                    if let Some(ref change) = cfg.reportable_change {
                        pos += change.serialize(&mut buf[pos..]);
                    }
                }
                if let Some(timeout) = rec.timeout {
                    let b = timeout.to_le_bytes();
                    buf[pos] = b[0];
                    buf[pos + 1] = b[1];
                    pos += 2;
                }
            }
        }
        pos
    }
}
