//! Pressure Measurement cluster (0x0403).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs — values in 0.1 kPa (i.e. 1 hPa = 10 units).
pub const ATTR_MEASURED_VALUE: AttributeId = AttributeId(0x0000);
pub const ATTR_MIN_MEASURED_VALUE: AttributeId = AttributeId(0x0001);
pub const ATTR_MAX_MEASURED_VALUE: AttributeId = AttributeId(0x0002);
pub const ATTR_TOLERANCE: AttributeId = AttributeId(0x0003);
pub const ATTR_SCALED_VALUE: AttributeId = AttributeId(0x0010);
pub const ATTR_MIN_SCALED_VALUE: AttributeId = AttributeId(0x0011);
pub const ATTR_MAX_SCALED_VALUE: AttributeId = AttributeId(0x0012);
pub const ATTR_SCALED_TOLERANCE: AttributeId = AttributeId(0x0013);
pub const ATTR_SCALE: AttributeId = AttributeId(0x0014);

/// Pressure Measurement cluster.
pub struct PressureCluster {
    store: AttributeStore<10>,
}

impl PressureCluster {
    pub fn new(min_tenth_kpa: i16, max_tenth_kpa: i16) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MEASURED_VALUE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::Reportable,
                name: "MeasuredValue",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_MEASURED_VALUE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "MinMeasuredValue",
            },
            ZclValue::I16(min_tenth_kpa),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_MEASURED_VALUE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "MaxMeasuredValue",
            },
            ZclValue::I16(max_tenth_kpa),
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
                id: ATTR_SCALED_VALUE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::Reportable,
                name: "ScaledValue",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_SCALED_VALUE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "MinScaledValue",
            },
            ZclValue::I16(min_tenth_kpa),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_SCALED_VALUE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "MaxScaledValue",
            },
            ZclValue::I16(max_tenth_kpa),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SCALED_TOLERANCE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ScaledTolerance",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SCALE,
                data_type: ZclDataType::I8,
                access: AttributeAccess::ReadOnly,
                name: "Scale",
            },
            ZclValue::I8(0),
        );
        Self { store }
    }

    /// Update the measured pressure (in 0.1 kPa units, e.g. 10132 = 1013.2 hPa).
    pub fn set_pressure(&mut self, tenth_kpa: i16) {
        let _ = self
            .store
            .set_raw(ATTR_MEASURED_VALUE, ZclValue::I16(tenth_kpa));
    }
}

impl Cluster for PressureCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::PRESSURE
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
