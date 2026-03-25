//! Illuminance Measurement cluster (0x0400).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs — MeasuredValue = 10000 × log10(lux) + 1.
pub const ATTR_MEASURED_VALUE: AttributeId = AttributeId(0x0000);
pub const ATTR_MIN_MEASURED_VALUE: AttributeId = AttributeId(0x0001);
pub const ATTR_MAX_MEASURED_VALUE: AttributeId = AttributeId(0x0002);
pub const ATTR_TOLERANCE: AttributeId = AttributeId(0x0003);
pub const ATTR_LIGHT_SENSOR_TYPE: AttributeId = AttributeId(0x0004);

/// Illuminance Measurement cluster.
pub struct IlluminanceCluster {
    store: AttributeStore<8>,
}

impl IlluminanceCluster {
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
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LIGHT_SENSOR_TYPE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "LightSensorType",
            },
            ZclValue::Enum8(0xFF), // Unknown
        );
        Self { store }
    }

    /// Update the measured illuminance (in 10000 × log10(lux) + 1 units).
    pub fn set_illuminance(&mut self, value: u16) {
        let _ = self
            .store
            .set_raw(ATTR_MEASURED_VALUE, ZclValue::U16(value));
    }
}

impl Cluster for IlluminanceCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::ILLUMINANCE
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
