//! Binary Input cluster (0x000F).
//!
//! Commonly used for contact sensors, motion detectors, and other boolean
//! state devices.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

pub const ATTR_ACTIVE_TEXT: AttributeId = AttributeId(0x0004);
pub const ATTR_DESCRIPTION: AttributeId = AttributeId(0x001C);
pub const ATTR_INACTIVE_TEXT: AttributeId = AttributeId(0x002E);
pub const ATTR_OUT_OF_SERVICE: AttributeId = AttributeId(0x0051);
pub const ATTR_POLARITY: AttributeId = AttributeId(0x0054);
pub const ATTR_PRESENT_VALUE: AttributeId = AttributeId(0x0055);
pub const ATTR_RELIABILITY: AttributeId = AttributeId(0x0067);
pub const ATTR_STATUS_FLAGS: AttributeId = AttributeId(0x006F);
pub const ATTR_APPLICATION_TYPE: AttributeId = AttributeId(0x0100);

/// Binary Input cluster.
pub struct BinaryInputCluster {
    store: AttributeStore<9>,
}

impl Default for BinaryInputCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl BinaryInputCluster {
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
                data_type: ZclDataType::Bool,
                access: AttributeAccess::Reportable,
                name: "PresentValue",
            },
            ZclValue::Bool(false),
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
                id: ATTR_POLARITY,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "Polarity",
            },
            ZclValue::U8(0), // NORMAL
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
                id: ATTR_APPLICATION_TYPE,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "ApplicationType",
            },
            ZclValue::U32(0),
        );
        Self { store }
    }

    /// Set the binary state.
    pub fn set_present_value(&mut self, active: bool) {
        let _ = self
            .store
            .set_raw(ATTR_PRESENT_VALUE, ZclValue::Bool(active));
    }
}

impl Cluster for BinaryInputCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::BINARY_INPUT
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
