//! Basic cluster (0x0000) — mandatory for all Zigbee devices.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_ZCL_VERSION: AttributeId = AttributeId(0x0000);
pub const ATTR_APPLICATION_VERSION: AttributeId = AttributeId(0x0001);
pub const ATTR_STACK_VERSION: AttributeId = AttributeId(0x0002);
pub const ATTR_HW_VERSION: AttributeId = AttributeId(0x0003);
pub const ATTR_MANUFACTURER_NAME: AttributeId = AttributeId(0x0004);
pub const ATTR_MODEL_IDENTIFIER: AttributeId = AttributeId(0x0005);
pub const ATTR_DATE_CODE: AttributeId = AttributeId(0x0006);
pub const ATTR_POWER_SOURCE: AttributeId = AttributeId(0x0007);
pub const ATTR_GENERIC_DEVICE_CLASS: AttributeId = AttributeId(0x0008);
pub const ATTR_GENERIC_DEVICE_TYPE: AttributeId = AttributeId(0x0009);
pub const ATTR_PRODUCT_CODE: AttributeId = AttributeId(0x000A);
pub const ATTR_PRODUCT_URL: AttributeId = AttributeId(0x000B);
pub const ATTR_LOCATION_DESCRIPTION: AttributeId = AttributeId(0x0010);
pub const ATTR_SW_BUILD_ID: AttributeId = AttributeId(0x4000);

// Command IDs
pub const CMD_RESET_TO_FACTORY_DEFAULTS: CommandId = CommandId(0x00);

/// Basic cluster implementation.
pub struct BasicCluster {
    store: AttributeStore<16>,
}

impl BasicCluster {
    pub fn new(manufacturer: &[u8], model: &[u8], date_code: &[u8], sw_build: &[u8]) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ZCL_VERSION,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ZCLVersion",
            },
            ZclValue::U8(8), // ZCL Rev 8
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APPLICATION_VERSION,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ApplicationVersion",
            },
            ZclValue::U8(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_STACK_VERSION,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "StackVersion",
            },
            ZclValue::U8(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_HW_VERSION,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "HWVersion",
            },
            ZclValue::U8(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MANUFACTURER_NAME,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadOnly,
                name: "ManufacturerName",
            },
            ZclValue::CharString(heapless::Vec::from_slice(manufacturer).unwrap_or_default()),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MODEL_IDENTIFIER,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadOnly,
                name: "ModelIdentifier",
            },
            ZclValue::CharString(heapless::Vec::from_slice(model).unwrap_or_default()),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DATE_CODE,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadOnly,
                name: "DateCode",
            },
            ZclValue::CharString(heapless::Vec::from_slice(date_code).unwrap_or_default()),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_POWER_SOURCE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "PowerSource",
            },
            ZclValue::Enum8(0x01), // Mains (single phase)
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LOCATION_DESCRIPTION,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadWrite,
                name: "LocationDescription",
            },
            ZclValue::CharString(heapless::Vec::new()),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SW_BUILD_ID,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadOnly,
                name: "SWBuildID",
            },
            ZclValue::CharString(heapless::Vec::from_slice(sw_build).unwrap_or_default()),
        );
        Self { store }
    }

    /// Set the power source enum value.
    pub fn set_power_source(&mut self, source: u8) {
        let _ = self
            .store
            .set_raw(ATTR_POWER_SOURCE, ZclValue::Enum8(source));
    }
}

impl Cluster for BasicCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::BASIC
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_RESET_TO_FACTORY_DEFAULTS => {
                // Application should handle actual reset; we just acknowledge.
                Ok(heapless::Vec::new())
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
