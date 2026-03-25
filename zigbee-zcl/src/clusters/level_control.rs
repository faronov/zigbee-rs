//! Level Control cluster (0x0008).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_CURRENT_LEVEL: AttributeId = AttributeId(0x0000);
pub const ATTR_REMAINING_TIME: AttributeId = AttributeId(0x0001);
pub const ATTR_MIN_LEVEL: AttributeId = AttributeId(0x0002);
pub const ATTR_MAX_LEVEL: AttributeId = AttributeId(0x0003);
pub const ATTR_ON_OFF_TRANSITION_TIME: AttributeId = AttributeId(0x0010);
pub const ATTR_ON_LEVEL: AttributeId = AttributeId(0x0011);
pub const ATTR_STARTUP_CURRENT_LEVEL: AttributeId = AttributeId(0x4000);
pub const ATTR_OPTIONS: AttributeId = AttributeId(0x000F);

// Command IDs
pub const CMD_MOVE_TO_LEVEL: CommandId = CommandId(0x00);
pub const CMD_MOVE: CommandId = CommandId(0x01);
pub const CMD_STEP: CommandId = CommandId(0x02);
pub const CMD_STOP: CommandId = CommandId(0x03);
pub const CMD_MOVE_TO_LEVEL_WITH_ON_OFF: CommandId = CommandId(0x04);
pub const CMD_MOVE_WITH_ON_OFF: CommandId = CommandId(0x05);
pub const CMD_STEP_WITH_ON_OFF: CommandId = CommandId(0x06);
pub const CMD_STOP_WITH_ON_OFF: CommandId = CommandId(0x07);

/// Level Control cluster implementation.
pub struct LevelControlCluster {
    store: AttributeStore<10>,
}

impl Default for LevelControlCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl LevelControlCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "CurrentLevel",
            },
            ZclValue::U8(0x00),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_REMAINING_TIME,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "RemainingTime",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "MinLevel",
            },
            ZclValue::U8(0x01),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "MaxLevel",
            },
            ZclValue::U8(0xFE),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ON_OFF_TRANSITION_TIME,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "OnOffTransitionTime",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ON_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "OnLevel",
            },
            ZclValue::U8(0xFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OPTIONS,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadWrite,
                name: "Options",
            },
            ZclValue::Bitmap8(0x00),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_STARTUP_CURRENT_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "StartUpCurrentLevel",
            },
            ZclValue::U8(0xFF), // Previous
        );
        Self { store }
    }

    /// Get the current level.
    pub fn current_level(&self) -> u8 {
        match self.store.get(ATTR_CURRENT_LEVEL) {
            Some(ZclValue::U8(v)) => *v,
            _ => 0,
        }
    }

    fn set_level(&mut self, level: u8) {
        let _ = self.store.set_raw(ATTR_CURRENT_LEVEL, ZclValue::U8(level));
    }
}

impl Cluster for LevelControlCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::LEVEL_CONTROL
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_MOVE_TO_LEVEL | CMD_MOVE_TO_LEVEL_WITH_ON_OFF => {
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let level = payload[0];
                let _transition_time = u16::from_le_bytes([payload[1], payload[2]]);
                // Instant move (transition would be ticked externally).
                self.set_level(level);
                Ok(heapless::Vec::new())
            }
            CMD_MOVE | CMD_MOVE_WITH_ON_OFF => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0]; // 0=up, 1=down
                let _rate = payload[1];
                // Simplified: jump to min or max.
                let level = if mode == 0 { 0xFE } else { 0x01 };
                self.set_level(level);
                Ok(heapless::Vec::new())
            }
            CMD_STEP | CMD_STEP_WITH_ON_OFF => {
                if payload.len() < 4 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0];
                let step_size = payload[1];
                let _transition_time = u16::from_le_bytes([payload[2], payload[3]]);
                let current = self.current_level();
                let new_level = if mode == 0 {
                    current.saturating_add(step_size).min(0xFE)
                } else {
                    current.saturating_sub(step_size).max(0x01)
                };
                self.set_level(new_level);
                Ok(heapless::Vec::new())
            }
            CMD_STOP | CMD_STOP_WITH_ON_OFF => {
                // Stop any ongoing transition.
                let _ = self.store.set_raw(ATTR_REMAINING_TIME, ZclValue::U16(0));
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
