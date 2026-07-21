//! Basic cluster (0x0000) — mandatory for all Zigbee devices.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{MAX_STRING_LEN, ZclDataType, ZclValue};
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

/// Basic-cluster PowerSource values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerSource {
    Unknown = 0x00,
    MainsSinglePhase = 0x01,
    MainsThreePhase = 0x02,
    Battery = 0x03,
    DcSource = 0x04,
    EmergencyMainsConstantlyPowered = 0x05,
    EmergencyMainsTransferSwitch = 0x06,
    UnknownWithBatteryBackup = 0x80,
    MainsSinglePhaseWithBatteryBackup = 0x81,
    MainsThreePhaseWithBatteryBackup = 0x82,
    BatteryWithBatteryBackup = 0x83,
    DcSourceWithBatteryBackup = 0x84,
    EmergencyMainsConstantlyPoweredWithBatteryBackup = 0x85,
    EmergencyMainsTransferSwitchWithBatteryBackup = 0x86,
}

/// Basic cluster implementation.
pub struct BasicCluster {
    store: AttributeStore<10>,
}

impl BasicCluster {
    /// Create a Basic cluster.
    ///
    /// ZCL strings are limited to [`MAX_STRING_LEN`] bytes. Longer values are
    /// truncated at a UTF-8 character boundary.
    pub fn new(
        manufacturer: &str,
        model: &str,
        date_code: &str,
        sw_build: &str,
        power_source: PowerSource,
    ) -> Self {
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
            ZclValue::CharString(zcl_string(manufacturer)),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MODEL_IDENTIFIER,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadOnly,
                name: "ModelIdentifier",
            },
            ZclValue::CharString(zcl_string(model)),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DATE_CODE,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadOnly,
                name: "DateCode",
            },
            ZclValue::CharString(zcl_string(date_code)),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_POWER_SOURCE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "PowerSource",
            },
            ZclValue::Enum8(power_source as u8),
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
            ZclValue::CharString(zcl_string(sw_build)),
        );
        Self { store }
    }

    /// Set the power source enum value.
    pub fn set_power_source(&mut self, source: PowerSource) {
        let _ = self
            .store
            .set_raw(ATTR_POWER_SOURCE, ZclValue::Enum8(source as u8));
    }

    /// Reset writable Basic attributes while preserving configured identity.
    pub fn reset_to_factory_defaults(&mut self) {
        let _ = self.store.set_raw(
            ATTR_LOCATION_DESCRIPTION,
            ZclValue::CharString(heapless::Vec::new()),
        );
    }

    pub fn manufacturer_name(&self) -> &str {
        self.string_attribute(ATTR_MANUFACTURER_NAME)
    }

    pub fn model_identifier(&self) -> &str {
        self.string_attribute(ATTR_MODEL_IDENTIFIER)
    }

    pub fn date_code(&self) -> &str {
        self.string_attribute(ATTR_DATE_CODE)
    }

    pub fn sw_build_id(&self) -> &str {
        self.string_attribute(ATTR_SW_BUILD_ID)
    }

    fn string_attribute(&self, id: AttributeId) -> &str {
        let Some(ZclValue::CharString(value)) = self.store.get(id) else {
            unreachable!("Basic cluster string attribute is always registered");
        };
        core::str::from_utf8(value.as_slice())
            .expect("Basic cluster strings originate from valid UTF-8")
    }
}

fn zcl_string(value: &str) -> heapless::Vec<u8, MAX_STRING_LEN> {
    let mut end = value.len().min(MAX_STRING_LEN);
    while !value.is_char_boundary(end) {
        end -= 1;
    }
    heapless::Vec::from_slice(&value.as_bytes()[..end])
        .expect("slice is bounded by the ZCL string capacity")
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
                self.reset_to_factory_defaults();
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

    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::from_slice(&[0x00]).unwrap_or_default()
    }
}
