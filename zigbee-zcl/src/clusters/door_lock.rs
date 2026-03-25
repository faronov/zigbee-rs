//! Door Lock cluster (0x0101).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_LOCK_STATE: AttributeId = AttributeId(0x0000);
pub const ATTR_LOCK_TYPE: AttributeId = AttributeId(0x0001);
pub const ATTR_ACTUATOR_ENABLED: AttributeId = AttributeId(0x0002);
pub const ATTR_DOOR_STATE: AttributeId = AttributeId(0x0003);
pub const ATTR_DOOR_OPEN_EVENTS: AttributeId = AttributeId(0x0004);
pub const ATTR_DOOR_CLOSED_EVENTS: AttributeId = AttributeId(0x0005);
pub const ATTR_OPEN_PERIOD: AttributeId = AttributeId(0x0006);
pub const ATTR_NUM_LOG_RECORDS_SUPPORTED: AttributeId = AttributeId(0x0010);
pub const ATTR_NUM_TOTAL_USERS_SUPPORTED: AttributeId = AttributeId(0x0011);
pub const ATTR_NUM_PIN_USERS_SUPPORTED: AttributeId = AttributeId(0x0012);
pub const ATTR_NUM_RFID_USERS_SUPPORTED: AttributeId = AttributeId(0x0013);
pub const ATTR_MAX_PIN_CODE_LENGTH: AttributeId = AttributeId(0x0017);
pub const ATTR_MIN_PIN_CODE_LENGTH: AttributeId = AttributeId(0x0018);
pub const ATTR_LANGUAGE: AttributeId = AttributeId(0x0021);
pub const ATTR_AUTO_RELOCK_TIME: AttributeId = AttributeId(0x0023);
pub const ATTR_OPERATING_MODE: AttributeId = AttributeId(0x0025);

// Client-to-server command IDs
pub const CMD_LOCK_DOOR: CommandId = CommandId(0x00);
pub const CMD_UNLOCK_DOOR: CommandId = CommandId(0x01);
pub const CMD_TOGGLE: CommandId = CommandId(0x02);
pub const CMD_UNLOCK_WITH_TIMEOUT: CommandId = CommandId(0x03);
pub const CMD_SET_PIN_CODE: CommandId = CommandId(0x05);
pub const CMD_GET_PIN_CODE: CommandId = CommandId(0x06);
pub const CMD_CLEAR_PIN_CODE: CommandId = CommandId(0x07);
pub const CMD_CLEAR_ALL_PIN_CODES: CommandId = CommandId(0x08);
pub const CMD_SET_USER_STATUS: CommandId = CommandId(0x09);
pub const CMD_GET_USER_STATUS: CommandId = CommandId(0x0A);

// Server-to-client command IDs
pub const CMD_LOCK_DOOR_RSP: CommandId = CommandId(0x00);
pub const CMD_UNLOCK_DOOR_RSP: CommandId = CommandId(0x01);
pub const CMD_TOGGLE_RSP: CommandId = CommandId(0x02);
pub const CMD_OPERATING_EVENT_NOTIFICATION: CommandId = CommandId(0x20);
pub const CMD_PROGRAMMING_EVENT_NOTIFICATION: CommandId = CommandId(0x21);

// LockState values
pub const LOCK_STATE_NOT_FULLY_LOCKED: u8 = 0x00;
pub const LOCK_STATE_LOCKED: u8 = 0x01;
pub const LOCK_STATE_UNLOCKED: u8 = 0x02;
pub const LOCK_STATE_UNDEFINED: u8 = 0xFF;

// LockType values
pub const LOCK_TYPE_DEAD_BOLT: u8 = 0x00;
pub const LOCK_TYPE_MAGNETIC: u8 = 0x01;
pub const LOCK_TYPE_OTHER: u8 = 0x02;
pub const LOCK_TYPE_MORTISE: u8 = 0x03;
pub const LOCK_TYPE_RIM: u8 = 0x04;
pub const LOCK_TYPE_LATCH_BOLT: u8 = 0x05;

// DoorState values
pub const DOOR_STATE_OPEN: u8 = 0x00;
pub const DOOR_STATE_CLOSED: u8 = 0x01;
pub const DOOR_STATE_ERROR_JAMMED: u8 = 0x02;
pub const DOOR_STATE_ERROR_FORCED_OPEN: u8 = 0x03;
pub const DOOR_STATE_ERROR_UNSPECIFIED: u8 = 0x04;
pub const DOOR_STATE_UNDEFINED: u8 = 0xFF;

// OperatingMode values
pub const OPERATING_MODE_NORMAL: u8 = 0x00;
pub const OPERATING_MODE_VACATION: u8 = 0x01;
pub const OPERATING_MODE_PRIVACY: u8 = 0x02;
pub const OPERATING_MODE_NO_RF_LOCK: u8 = 0x03;
pub const OPERATING_MODE_PASSAGE: u8 = 0x04;

/// Door Lock cluster.
pub struct DoorLockCluster {
    store: AttributeStore<16>,
}

impl DoorLockCluster {
    pub fn new(lock_type: u8) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LOCK_STATE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::Reportable,
                name: "LockState",
            },
            ZclValue::Enum8(LOCK_STATE_UNDEFINED),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LOCK_TYPE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "LockType",
            },
            ZclValue::Enum8(lock_type),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ACTUATOR_ENABLED,
                data_type: ZclDataType::Bool,
                access: AttributeAccess::ReadOnly,
                name: "ActuatorEnabled",
            },
            ZclValue::Bool(true),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DOOR_STATE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::Reportable,
                name: "DoorState",
            },
            ZclValue::Enum8(DOOR_STATE_UNDEFINED),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DOOR_OPEN_EVENTS,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadWrite,
                name: "DoorOpenEvents",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DOOR_CLOSED_EVENTS,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadWrite,
                name: "DoorClosedEvents",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OPEN_PERIOD,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "OpenPeriod",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NUM_LOG_RECORDS_SUPPORTED,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "NumberOfLogRecordsSupported",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NUM_TOTAL_USERS_SUPPORTED,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "NumberOfTotalUsersSupported",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NUM_PIN_USERS_SUPPORTED,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "NumberOfPINUsersSupported",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NUM_RFID_USERS_SUPPORTED,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "NumberOfRFIDUsersSupported",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_PIN_CODE_LENGTH,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "MaxPINCodeLength",
            },
            ZclValue::U8(8),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_PIN_CODE_LENGTH,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "MinPINCodeLength",
            },
            ZclValue::U8(4),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LANGUAGE,
                data_type: ZclDataType::CharString,
                access: AttributeAccess::ReadWrite,
                name: "Language",
            },
            ZclValue::CharString(heapless::Vec::from_slice(b"en").unwrap_or_default()),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_AUTO_RELOCK_TIME,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadWrite,
                name: "AutoRelockTime",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OPERATING_MODE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "OperatingMode",
            },
            ZclValue::Enum8(OPERATING_MODE_NORMAL),
        );
        Self { store }
    }

    /// Get current lock state.
    pub fn lock_state(&self) -> u8 {
        match self.store.get(ATTR_LOCK_STATE) {
            Some(ZclValue::Enum8(v)) => *v,
            _ => LOCK_STATE_UNDEFINED,
        }
    }

    /// Set lock state directly.
    pub fn set_lock_state(&mut self, state: u8) {
        let _ = self.store.set_raw(ATTR_LOCK_STATE, ZclValue::Enum8(state));
    }

    /// Set door state directly.
    pub fn set_door_state(&mut self, state: u8) {
        let _ = self.store.set_raw(ATTR_DOOR_STATE, ZclValue::Enum8(state));
    }

    fn build_status_response(status: u8) -> heapless::Vec<u8, 64> {
        let mut resp = heapless::Vec::new();
        let _ = resp.push(status);
        resp
    }
}

impl Cluster for DoorLockCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::DOOR_LOCK
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_LOCK_DOOR => {
                self.set_lock_state(LOCK_STATE_LOCKED);
                Ok(Self::build_status_response(0x00))
            }
            CMD_UNLOCK_DOOR => {
                self.set_lock_state(LOCK_STATE_UNLOCKED);
                Ok(Self::build_status_response(0x00))
            }
            CMD_TOGGLE => {
                let new_state = if self.lock_state() == LOCK_STATE_LOCKED {
                    LOCK_STATE_UNLOCKED
                } else {
                    LOCK_STATE_LOCKED
                };
                self.set_lock_state(new_state);
                Ok(Self::build_status_response(0x00))
            }
            CMD_UNLOCK_WITH_TIMEOUT => {
                if _payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                // timeout is u16 LE in payload; we unlock and leave timeout to the application layer
                self.set_lock_state(LOCK_STATE_UNLOCKED);
                Ok(Self::build_status_response(0x00))
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
