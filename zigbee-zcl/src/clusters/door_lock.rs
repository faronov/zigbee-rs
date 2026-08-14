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

/// A PIN code entry for a user.
#[derive(Debug, Clone)]
pub struct PinEntry {
    pub status: u8,    // 0=available, 1=occupied_enabled, 3=occupied_disabled
    pub user_type: u8, // 0=unrestricted, 1=year_day, 2=week_day, 3=master
    pub pin: heapless::Vec<u8, 8>,
}

/// Door Lock cluster.
pub struct DoorLockCluster {
    store: AttributeStore<16>,
    pins: heapless::Vec<(u16, PinEntry), 8>,
    /// Seconds remaining until automatic re-lock (0 = inactive).
    pub auto_relock_remaining: u32,
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
        Self {
            store,
            pins: heapless::Vec::new(),
            auto_relock_remaining: 0,
        }
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

    /// Start the auto-relock countdown from the ATTR_AUTO_RELOCK_TIME value.
    fn start_auto_relock_timer(&mut self) {
        let timeout = match self.store.get(ATTR_AUTO_RELOCK_TIME) {
            Some(ZclValue::U32(v)) => *v,
            _ => 0,
        };
        self.auto_relock_remaining = timeout;
    }

    /// Call every 1 second. Decrements the auto-relock timer and locks
    /// the door when it expires.
    pub fn tick(&mut self) {
        if self.auto_relock_remaining > 0 && self.lock_state() == LOCK_STATE_UNLOCKED {
            self.auto_relock_remaining -= 1;
            if self.auto_relock_remaining == 0 {
                self.set_lock_state(LOCK_STATE_LOCKED);
            }
        }
    }
}

impl Cluster for DoorLockCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::DOOR_LOCK
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_LOCK_DOOR => {
                self.set_lock_state(LOCK_STATE_LOCKED);
                Ok(Self::build_status_response(0x00))
            }
            CMD_UNLOCK_DOOR => {
                self.set_lock_state(LOCK_STATE_UNLOCKED);
                self.start_auto_relock_timer();
                Ok(Self::build_status_response(0x00))
            }
            CMD_TOGGLE => {
                let new_state = if self.lock_state() == LOCK_STATE_LOCKED {
                    LOCK_STATE_UNLOCKED
                } else {
                    LOCK_STATE_LOCKED
                };
                self.set_lock_state(new_state);
                if new_state == LOCK_STATE_UNLOCKED {
                    self.start_auto_relock_timer();
                }
                Ok(Self::build_status_response(0x00))
            }
            CMD_UNLOCK_WITH_TIMEOUT => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                self.set_lock_state(LOCK_STATE_UNLOCKED);
                self.start_auto_relock_timer();
                Ok(Self::build_status_response(0x00))
            }
            CMD_SET_PIN_CODE => {
                // Payload: user_id(2) + user_status(1) + user_type(1) + pin_len(1) + pin[]
                if payload.len() < 5 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let user_id = u16::from_le_bytes([payload[0], payload[1]]);
                let user_status = payload[2];
                let user_type = payload[3];
                let pin_len = payload[4] as usize;
                if payload.len() < 5 + pin_len {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mut pin = heapless::Vec::new();
                let _ = pin.extend_from_slice(&payload[5..5 + pin_len]);
                let entry = PinEntry {
                    status: user_status,
                    user_type,
                    pin,
                };
                // Update existing or insert new
                let mut found = false;
                for (id, existing) in self.pins.iter_mut() {
                    if *id == user_id {
                        *existing = entry.clone();
                        found = true;
                        break;
                    }
                }
                if !found {
                    let _ = self.pins.push((user_id, entry));
                }
                Ok(Self::build_status_response(0x00)) // success
            }
            CMD_GET_PIN_CODE => {
                // Payload: user_id(2)
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let user_id = u16::from_le_bytes([payload[0], payload[1]]);
                let mut resp = heapless::Vec::new();
                let _ = resp.extend_from_slice(&user_id.to_le_bytes());
                if let Some((_, entry)) = self.pins.iter().find(|(id, _)| *id == user_id) {
                    let _ = resp.push(entry.status);
                    let _ = resp.push(entry.user_type);
                    let _ = resp.push(entry.pin.len() as u8);
                    let _ = resp.extend_from_slice(&entry.pin);
                } else {
                    let _ = resp.push(0x00); // available
                    let _ = resp.push(0x00); // unrestricted
                    let _ = resp.push(0x00); // no pin
                }
                Ok(resp)
            }
            CMD_CLEAR_PIN_CODE => {
                // Payload: user_id(2)
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let user_id = u16::from_le_bytes([payload[0], payload[1]]);
                self.pins.retain(|(id, _)| *id != user_id);
                Ok(Self::build_status_response(0x00))
            }
            CMD_CLEAR_ALL_PIN_CODES => {
                self.pins.clear();
                Ok(Self::build_status_response(0x00))
            }
            CMD_SET_USER_STATUS => {
                // Payload: user_id(2) + status(1)
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let user_id = u16::from_le_bytes([payload[0], payload[1]]);
                let new_status = payload[2];
                for (id, entry) in self.pins.iter_mut() {
                    if *id == user_id {
                        entry.status = new_status;
                        break;
                    }
                }
                Ok(Self::build_status_response(0x00))
            }
            CMD_GET_USER_STATUS => {
                // Payload: user_id(2)
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let user_id = u16::from_le_bytes([payload[0], payload[1]]);
                let mut resp = heapless::Vec::new();
                let _ = resp.extend_from_slice(&user_id.to_le_bytes());
                let status = self
                    .pins
                    .iter()
                    .find(|(id, _)| *id == user_id)
                    .map(|(_, e)| e.status)
                    .unwrap_or(0x00); // available if not found
                let _ = resp.push(status);
                Ok(resp)
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::from_slice(&[
            CMD_LOCK_DOOR.0,
            CMD_UNLOCK_DOOR.0,
            CMD_TOGGLE.0,
            CMD_UNLOCK_WITH_TIMEOUT.0,
            CMD_SET_PIN_CODE.0,
            CMD_GET_PIN_CODE.0,
            CMD_CLEAR_PIN_CODE.0,
            CMD_CLEAR_ALL_PIN_CODES.0,
            CMD_SET_USER_STATUS.0,
            CMD_GET_USER_STATUS.0,
        ])
        .unwrap_or_default()
    }

    fn generated_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::from_slice(&[
            CMD_LOCK_DOOR_RSP.0,
            CMD_UNLOCK_DOOR_RSP.0,
            CMD_TOGGLE_RSP.0,
            CMD_OPERATING_EVENT_NOTIFICATION.0,
            CMD_PROGRAMMING_EVENT_NOTIFICATION.0,
        ])
        .unwrap_or_default()
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }

    /// PIN/RFID user codes (`pins`) are a security credential store and
    /// MUST NOT be cleared by a Basic cluster reset (only an explicit,
    /// authorized `ClearAllPINCodes` command may do that). `LockState` and
    /// `DoorState` reflect real physical security state — the former only
    /// ever changes via an explicit Lock/Unlock/Toggle command and the
    /// latter via the door sensor driver — so neither is overwritten here
    /// to avoid ever falsely reporting an unlocked/open state. Only the
    /// writable configuration attributes and the transient auto-relock
    /// timer are reset.
    fn reset_to_factory_defaults(&mut self) {
        self.auto_relock_remaining = 0;
        let _ = self.store.set_raw(ATTR_DOOR_OPEN_EVENTS, ZclValue::U32(0));
        let _ = self
            .store
            .set_raw(ATTR_DOOR_CLOSED_EVENTS, ZclValue::U32(0));
        let _ = self.store.set_raw(ATTR_OPEN_PERIOD, ZclValue::U16(0));
        let _ = self.store.set_raw(
            ATTR_LANGUAGE,
            ZclValue::CharString(heapless::Vec::from_slice(b"en").unwrap_or_default()),
        );
        let _ = self.store.set_raw(ATTR_AUTO_RELOCK_TIME, ZclValue::U32(0));
        let _ = self
            .store
            .set_raw(ATTR_OPERATING_MODE, ZclValue::Enum8(OPERATING_MODE_NORMAL));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_preserves_pin_codes_and_lock_state_but_clears_config_and_timer() {
        let mut cluster = DoorLockCluster::new(LOCK_TYPE_DEAD_BOLT);
        cluster
            .handle_command(
                CMD_SET_PIN_CODE,
                &[0x01, 0x00, 0x01, 0x00, 0x04, 1, 2, 3, 4],
            )
            .unwrap();
        cluster
            .attributes_mut()
            .set(ATTR_AUTO_RELOCK_TIME, ZclValue::U32(60))
            .unwrap();
        cluster.handle_command(CMD_UNLOCK_DOOR, &[]).unwrap();
        cluster
            .attributes_mut()
            .set(ATTR_OPEN_PERIOD, ZclValue::U16(30))
            .unwrap();
        assert_eq!(cluster.lock_state(), LOCK_STATE_UNLOCKED);
        assert_eq!(cluster.auto_relock_remaining, 60);

        Cluster::reset_to_factory_defaults(&mut cluster);

        // Security-sensitive state survives the reset.
        assert_eq!(cluster.lock_state(), LOCK_STATE_UNLOCKED);
        let mut resp = cluster
            .handle_command(CMD_GET_PIN_CODE, &[0x01, 0x00])
            .unwrap();
        // user_id(2) + status(1) + user_type(1) + pin_len(1) + pin(4)
        assert_eq!(resp.len(), 9);
        assert_eq!(resp.remove(4), 4); // pin_len preserved (code not cleared)

        // Transient/config state is reset.
        assert_eq!(cluster.auto_relock_remaining, 0);
        assert_eq!(
            cluster.attributes().get(ATTR_OPEN_PERIOD),
            Some(&ZclValue::U16(0))
        );
        assert_eq!(
            cluster.attributes().get(ATTR_AUTO_RELOCK_TIME),
            Some(&ZclValue::U32(0))
        );
    }
}
