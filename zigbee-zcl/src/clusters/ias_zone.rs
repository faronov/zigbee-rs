//! IAS Zone cluster (0x0500).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_ZONE_STATE: AttributeId = AttributeId(0x0000);
pub const ATTR_ZONE_TYPE: AttributeId = AttributeId(0x0001);
pub const ATTR_ZONE_STATUS: AttributeId = AttributeId(0x0002);
pub const ATTR_IAS_CIE_ADDRESS: AttributeId = AttributeId(0x0010);
pub const ATTR_ZONE_ID: AttributeId = AttributeId(0x0011);
pub const ATTR_NUM_ZONE_SENSITIVITY_LEVELS: AttributeId = AttributeId(0x0012);
pub const ATTR_CURRENT_ZONE_SENSITIVITY_LEVEL: AttributeId = AttributeId(0x0013);

// Zone state values
pub const ZONE_STATE_NOT_ENROLLED: u8 = 0x00;
pub const ZONE_STATE_ENROLLED: u8 = 0x01;

// Zone type values
pub const ZONE_TYPE_STANDARD_CIE: u16 = 0x0000;
pub const ZONE_TYPE_MOTION_SENSOR: u16 = 0x000D;
pub const ZONE_TYPE_CONTACT_SWITCH: u16 = 0x0015;
pub const ZONE_TYPE_FIRE_SENSOR: u16 = 0x0028;
pub const ZONE_TYPE_WATER_SENSOR: u16 = 0x002A;
pub const ZONE_TYPE_CO_SENSOR: u16 = 0x002B;
pub const ZONE_TYPE_PERSONAL_EMERGENCY: u16 = 0x002D;
pub const ZONE_TYPE_REMOTE_CONTROL: u16 = 0x010F;
pub const ZONE_TYPE_KEY_FOB: u16 = 0x0115;
pub const ZONE_TYPE_KEYPAD: u16 = 0x021D;
pub const ZONE_TYPE_STANDARD_WARNING: u16 = 0x0225;

// Command IDs (client to server)
pub const CMD_ZONE_ENROLL_RESPONSE: CommandId = CommandId(0x00);
pub const CMD_INITIATE_NORMAL_OP_MODE: CommandId = CommandId(0x01);
pub const CMD_INITIATE_TEST_MODE: CommandId = CommandId(0x02);

// Command IDs (server to client)
pub const CMD_ZONE_STATUS_CHANGE_NOTIFICATION: CommandId = CommandId(0x00);
pub const CMD_ZONE_ENROLL_REQUEST: CommandId = CommandId(0x01);

/// IAS Zone cluster implementation.
pub struct IasZoneCluster {
    store: AttributeStore<10>,
}

impl IasZoneCluster {
    pub fn new(zone_type: u16) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ZONE_STATE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "ZoneState",
            },
            ZclValue::Enum8(ZONE_STATE_NOT_ENROLLED),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ZONE_TYPE,
                data_type: ZclDataType::Enum16,
                access: AttributeAccess::ReadOnly,
                name: "ZoneType",
            },
            ZclValue::Enum16(zone_type),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ZONE_STATUS,
                data_type: ZclDataType::Bitmap16,
                access: AttributeAccess::ReadOnly,
                name: "ZoneStatus",
            },
            ZclValue::Bitmap16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_IAS_CIE_ADDRESS,
                data_type: ZclDataType::IeeeAddr,
                access: AttributeAccess::ReadWrite,
                name: "IAS_CIE_Address",
            },
            ZclValue::IeeeAddr(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ZONE_ID,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ZoneID",
            },
            ZclValue::U8(0xFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NUM_ZONE_SENSITIVITY_LEVELS,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "NumberOfZoneSensitivityLevelsSupported",
            },
            ZclValue::U8(2),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_ZONE_SENSITIVITY_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "CurrentZoneSensitivityLevel",
            },
            ZclValue::U8(0),
        );
        Self { store }
    }

    /// Update zone status bits.
    pub fn set_zone_status(&mut self, status: u16) {
        let _ = self
            .store
            .set_raw(ATTR_ZONE_STATUS, ZclValue::Bitmap16(status));
    }
}

impl Cluster for IasZoneCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::IAS_ZONE
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_ZONE_ENROLL_RESPONSE => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let enroll_response_code = payload[0];
                let zone_id = payload[1];
                if enroll_response_code == 0x00 {
                    // Success
                    let _ = self
                        .store
                        .set_raw(ATTR_ZONE_STATE, ZclValue::Enum8(ZONE_STATE_ENROLLED));
                    let _ = self.store.set_raw(ATTR_ZONE_ID, ZclValue::U8(zone_id));
                }
                Ok(heapless::Vec::new())
            }
            CMD_INITIATE_NORMAL_OP_MODE => Ok(heapless::Vec::new()),
            CMD_INITIATE_TEST_MODE => {
                // Payload: test mode duration (u8) + current zone sensitivity level (u8)
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
