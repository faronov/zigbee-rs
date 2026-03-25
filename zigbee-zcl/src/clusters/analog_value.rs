//! Analog Value cluster (0x000E).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

pub const ATTR_DESCRIPTION: AttributeId = AttributeId(0x001C);
pub const ATTR_OUT_OF_SERVICE: AttributeId = AttributeId(0x0051);
pub const ATTR_PRESENT_VALUE: AttributeId = AttributeId(0x0055);
pub const ATTR_RELIABILITY: AttributeId = AttributeId(0x0067);
pub const ATTR_RELINQUISH_DEFAULT: AttributeId = AttributeId(0x0068);
pub const ATTR_STATUS_FLAGS: AttributeId = AttributeId(0x006F);
pub const ATTR_ENGINEERING_UNITS: AttributeId = AttributeId(0x0075);
pub const ATTR_APPLICATION_TYPE: AttributeId = AttributeId(0x0100);

/// Analog Value cluster.
pub struct AnalogValueCluster {
    store: AttributeStore<8>,
}

impl Default for AnalogValueCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl AnalogValueCluster {
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
                access: AttributeAccess::ReadWrite,
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
                id: ATTR_RELIABILITY,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "Reliability",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_RELINQUISH_DEFAULT,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::ReadWrite,
                name: "RelinquishDefault",
            },
            ZclValue::Float32(0.0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ENGINEERING_UNITS,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "EngineeringUnits",
            },
            ZclValue::U16(95),
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

    /// Set the analog value.
    pub fn set_present_value(&mut self, val: f32) {
        let _ = self
            .store
            .set_raw(ATTR_PRESENT_VALUE, ZclValue::Float32(val));
    }
}

impl Cluster for AnalogValueCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::ANALOG_VALUE
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
