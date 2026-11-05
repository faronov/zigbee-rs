//! Anomaly record publication: the shared lifecycle's
//! [`sensor_sed_app::AnomalyRecord`] as a minimal Diagnostics (0x0B05)
//! server, so a field device that hit a keep-awake limit can be inspected
//! from the coordinator without a UART.
//!
//! Standard Diagnostics attribute IDs carry the record, each with the
//! standard's data type (ZHA/zigpy decode them by that type, so the generic
//! "read attribute" tooling shows them):
//!
//! | Attribute | Standard name                     | Meaning here                          |
//! |-----------|-----------------------------------|---------------------------------------|
//! | `0x0000`  | `NumberOfResets` (uint16)         | total anomalies recorded              |
//! | `0x0001`  | `PersistentMemoryWrites` (uint16) | [`AnomalyRecord::summary_word`]: bits 0–7 last reason, 8–15 reasons seen |
//! | `0x0100`  | `MacRxBcast` (uint32)             | [`AnomalyRecord::last_cause`]: reset/sleep-failure cause word |
//! | `0x0101`  | `MacTxBcast` (uint32)             | per-reason counts, reasons 1–4 (one byte each, LE) |
//! | `0x0102`  | `MacRxUcast` (uint32)             | per-reason counts, reasons 5–8 (one byte each, LE) |
//!
//! `0x0000` reports on change only (no periodic report), so a recorded
//! anomaly reaches the coordinator without extra traffic in normal
//! operation.

use sensor_sed_app::AnomalyRecord;
use zigbee_mac::MacDriver;
use zigbee_runtime::profile::ProfileError;
use zigbee_runtime::role::DeviceRole;
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use zigbee_zcl::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
use zigbee_zcl::{AttributeId, ClusterId, CommandId, ZclStatus};

pub const ATTR_ANOMALY_TOTAL: AttributeId = AttributeId(0x0000);
pub const ATTR_ANOMALY_SUMMARY: AttributeId = AttributeId(0x0001);
pub const ATTR_ANOMALY_LAST_CAUSE: AttributeId = AttributeId(0x0100);
pub const ATTR_ANOMALY_COUNTS_LOW: AttributeId = AttributeId(0x0101);
pub const ATTR_ANOMALY_COUNTS_HIGH: AttributeId = AttributeId(0x0102);

const U32_ATTRIBUTES: [AttributeId; 3] = [
    ATTR_ANOMALY_LAST_CAUSE,
    ATTR_ANOMALY_COUNTS_LOW,
    ATTR_ANOMALY_COUNTS_HIGH,
];

/// Read-only Diagnostics server holding the anomaly record.
pub struct AnomalyDiagnostics {
    store: AttributeStore<5>,
}

impl AnomalyDiagnostics {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ANOMALY_TOTAL,
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ANOMALY_SUMMARY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "",
            },
            ZclValue::U16(0),
        );
        for id in U32_ATTRIBUTES {
            let _ = store.register(
                AttributeDefinition {
                    id,
                    data_type: ZclDataType::U32,
                    access: AttributeAccess::ReadOnly,
                    name: "",
                },
                ZclValue::U32(0),
            );
        }
        Self { store }
    }

    /// Publish `record`; the reporting engine sends the changed total.
    pub fn publish(&mut self, record: &AnomalyRecord) {
        let _ = self
            .store
            .set_raw(ATTR_ANOMALY_TOTAL, ZclValue::U16(record.total));
        let _ = self
            .store
            .set_raw(ATTR_ANOMALY_SUMMARY, ZclValue::U16(record.summary_word()));
        let values = [
            record.last_cause,
            record.counts_word(0),
            record.counts_word(1),
        ];
        for (id, value) in U32_ATTRIBUTES.into_iter().zip(values) {
            let _ = self.store.set_raw(id, ZclValue::U32(value));
        }
    }
}

impl Cluster for AnomalyDiagnostics {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::DIAGNOSTICS
    }
    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        Err(ZclStatus::UnsupClusterCommand)
    }
    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
    /// The record is device history, not configuration: a factory reset of
    /// the cluster keeps it (the journal survives too).
    fn reset_to_factory_defaults(&mut self) {}
}

/// Report the anomaly total on change only (`max_interval` 0: never
/// periodic, so it adds no traffic and no missed-report deadline).
pub fn configure_reporting<M: MacDriver, R: DeviceRole>(
    endpoint: u8,
    device: &mut ZigbeeDevice<M, R>,
) -> Result<(), ProfileError> {
    device
        .reporting_mut()
        .configure_for_cluster(
            endpoint,
            ClusterId::DIAGNOSTICS.0,
            ReportingConfig {
                direction: ReportDirection::Send,
                attribute_id: ATTR_ANOMALY_TOTAL,
                data_type: ZclDataType::U16,
                min_interval: 0,
                max_interval: 0,
                reportable_change: Some(ZclValue::U16(1)),
            },
        )
        .map_err(ProfileError::Reporting)
}
