//! Analog Input cluster (0x000C).
//!
//! Used heavily by Aqara/Xiaomi devices for reporting custom analog sensor
//! values (power, energy, etc.).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_DESCRIPTION: AttributeId = AttributeId(0x001C);
pub const ATTR_MAX_PRESENT_VALUE: AttributeId = AttributeId(0x0041);
pub const ATTR_MIN_PRESENT_VALUE: AttributeId = AttributeId(0x0045);
pub const ATTR_OUT_OF_SERVICE: AttributeId = AttributeId(0x0051);
pub const ATTR_PRESENT_VALUE: AttributeId = AttributeId(0x0055);
pub const ATTR_RELIABILITY: AttributeId = AttributeId(0x0067);
pub const ATTR_RESOLUTION: AttributeId = AttributeId(0x006A);
pub const ATTR_STATUS_FLAGS: AttributeId = AttributeId(0x006F);
pub const ATTR_ENGINEERING_UNITS: AttributeId = AttributeId(0x0075);
pub const ATTR_APPLICATION_TYPE: AttributeId = AttributeId(0x0100);

/// Analog Input cluster.
pub struct AnalogInputCluster {
    store: AttributeStore<10>,
}

impl Default for AnalogInputCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl AnalogInputCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OUT_OF_SERVICE,
                data_type: ZclDataType::Bool,
                access: AttributeAccess::ReadWrite,
                name: "OutOfService",
            },
            ZclValue::Bool(false),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_PRESENT_VALUE,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::Reportable,
                name: "PresentValue",
            },
            ZclValue::Float32(0.0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_STATUS_FLAGS,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "StatusFlags",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_PRESENT_VALUE,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::ReadWrite,
                name: "MaxPresentValue",
            },
            ZclValue::Float32(f32::MAX),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_PRESENT_VALUE,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::ReadWrite,
                name: "MinPresentValue",
            },
            ZclValue::Float32(f32::MIN),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_RELIABILITY,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "Reliability",
            },
            ZclValue::U8(0), // NO_FAULT_DETECTED
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_RESOLUTION,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::ReadOnly,
                name: "Resolution",
            },
            ZclValue::Float32(0.1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ENGINEERING_UNITS,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "EngineeringUnits",
            },
            ZclValue::U16(95), // no-units
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APPLICATION_TYPE,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "ApplicationType",
            },
            ZclValue::U32(0),
        );
        Self { store }
    }

    /// Set the current present value.
    pub fn set_present_value(&mut self, val: f32) {
        let _ = self
            .store
            .set_raw(ATTR_PRESENT_VALUE, ZclValue::Float32(val));
    }
}

impl Cluster for AnalogInputCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::ANALOG_INPUT
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
