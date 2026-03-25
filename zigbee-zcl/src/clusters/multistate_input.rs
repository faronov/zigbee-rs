//! Multistate Input Basic cluster (0x0012).
//!
//! Generic multi-state input object, commonly used for rotary switches,
//! multi-position selectors, or enumerated sensor readings.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_DESCRIPTION: AttributeId = AttributeId(0x001C);
pub const ATTR_NUMBER_OF_STATES: AttributeId = AttributeId(0x004A);
pub const ATTR_OUT_OF_SERVICE: AttributeId = AttributeId(0x0051);
pub const ATTR_PRESENT_VALUE: AttributeId = AttributeId(0x0055);
pub const ATTR_RELIABILITY: AttributeId = AttributeId(0x0067);
pub const ATTR_STATUS_FLAGS: AttributeId = AttributeId(0x006F);
pub const ATTR_APPLICATION_TYPE: AttributeId = AttributeId(0x0100);

// StatusFlags bitmap bits
pub const STATUS_FLAG_IN_ALARM: u8 = 0x01;
pub const STATUS_FLAG_FAULT: u8 = 0x02;
pub const STATUS_FLAG_OVERRIDDEN: u8 = 0x04;
pub const STATUS_FLAG_OUT_OF_SERVICE: u8 = 0x08;

/// Multistate Input Basic cluster implementation.
pub struct MultistateInputCluster {
    store: AttributeStore<7>,
}

impl MultistateInputCluster {
    pub fn new(number_of_states: u16) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DESCRIPTION,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadWrite,
                name: "Description",
            },
            ZclValue::CharString(heapless::Vec::new()),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NUMBER_OF_STATES,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "NumberOfStates",
            },
            ZclValue::U16(number_of_states),
        );
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
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "PresentValue",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_RELIABILITY,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "Reliability",
            },
            ZclValue::Enum8(0x00),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_STATUS_FLAGS,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::Reportable,
                name: "StatusFlags",
            },
            ZclValue::Bitmap8(0x00),
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

    /// Set the present value of the multistate input.
    pub fn set_value(&mut self, value: u16) {
        let _ = self.store.set_raw(ATTR_PRESENT_VALUE, ZclValue::U16(value));
    }

    /// Get the current present value.
    pub fn get_value(&self) -> u16 {
        match self.store.get(ATTR_PRESENT_VALUE) {
            Some(ZclValue::U16(v)) => *v,
            _ => 0,
        }
    }
}

impl Cluster for MultistateInputCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0012)
    }

    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        // No cluster-specific commands defined for Multistate Input Basic.
        Err(ZclStatus::UnsupClusterCommand)
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
