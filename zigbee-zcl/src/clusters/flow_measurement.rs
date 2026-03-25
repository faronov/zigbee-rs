//! Flow Measurement cluster (0x0404).
//!
//! Reports fluid flow rate in units of 0.1 m³/h. Same attribute
//! pattern as Temperature / Pressure / Humidity measurement clusters.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_MEASURED_VALUE: AttributeId = AttributeId(0x0000);
pub const ATTR_MIN_MEASURED_VALUE: AttributeId = AttributeId(0x0001);
pub const ATTR_MAX_MEASURED_VALUE: AttributeId = AttributeId(0x0002);
pub const ATTR_TOLERANCE: AttributeId = AttributeId(0x0003);

/// Flow Measurement cluster implementation.
pub struct FlowMeasurementCluster {
    store: AttributeStore<4>,
}

impl FlowMeasurementCluster {
    pub fn new(min: u16, max: u16) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MEASURED_VALUE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "MeasuredValue",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_MEASURED_VALUE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MinMeasuredValue",
            },
            ZclValue::U16(min),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_MEASURED_VALUE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MaxMeasuredValue",
            },
            ZclValue::U16(max),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_TOLERANCE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "Tolerance",
            },
            ZclValue::U16(0),
        );
        Self { store }
    }

    /// Update the measured flow value (in units of 0.1 m³/h).
    pub fn set_flow(&mut self, value: u16) {
        let _ = self
            .store
            .set_raw(ATTR_MEASURED_VALUE, ZclValue::U16(value));
    }
}

impl Cluster for FlowMeasurementCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0404)
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
}
