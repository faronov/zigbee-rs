//! Relative Humidity Measurement cluster (0x0405).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs — values in 0.01% RH units.
pub const ATTR_MEASURED_VALUE: AttributeId = AttributeId(0x0000);
pub const ATTR_MIN_MEASURED_VALUE: AttributeId = AttributeId(0x0001);
pub const ATTR_MAX_MEASURED_VALUE: AttributeId = AttributeId(0x0002);
pub const ATTR_TOLERANCE: AttributeId = AttributeId(0x0003);

/// Relative Humidity Measurement cluster.
pub struct HumidityCluster {
    store: AttributeStore<4>,
}

impl HumidityCluster {
    pub fn new(min_hundredths: u16, max_hundredths: u16) -> Self {
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
            ZclValue::U16(min_hundredths),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_MEASURED_VALUE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MaxMeasuredValue",
            },
            ZclValue::U16(max_hundredths),
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

    /// Update the measured humidity (in 0.01% RH units).
    pub fn set_humidity(&mut self, hundredths: u16) {
        let _ = self
            .store
            .set_raw(ATTR_MEASURED_VALUE, ZclValue::U16(hundredths));
    }
}

impl Cluster for HumidityCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::HUMIDITY
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
