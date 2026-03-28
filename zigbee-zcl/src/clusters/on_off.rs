//! On/Off cluster (0x0006).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_ON_OFF: AttributeId = AttributeId(0x0000);
pub const ATTR_GLOBAL_SCENE_CONTROL: AttributeId = AttributeId(0x4000);
pub const ATTR_ON_TIME: AttributeId = AttributeId(0x4001);
pub const ATTR_OFF_WAIT_TIME: AttributeId = AttributeId(0x4002);
pub const ATTR_START_UP_ON_OFF: AttributeId = AttributeId(0x4003);

// Command IDs (client to server)
pub const CMD_OFF: CommandId = CommandId(0x00);
pub const CMD_ON: CommandId = CommandId(0x01);
pub const CMD_TOGGLE: CommandId = CommandId(0x02);
pub const CMD_OFF_WITH_EFFECT: CommandId = CommandId(0x40);
pub const CMD_ON_WITH_RECALL_GLOBAL_SCENE: CommandId = CommandId(0x41);
pub const CMD_ON_WITH_TIMED_OFF: CommandId = CommandId(0x42);

/// On/Off cluster implementation.
pub struct OnOffCluster {
    store: AttributeStore<8>,
}

impl Default for OnOffCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl OnOffCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ON_OFF,
                data_type: ZclDataType::Bool,
                access: AttributeAccess::Reportable,
                name: "OnOff",
            },
            ZclValue::Bool(false),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_GLOBAL_SCENE_CONTROL,
                data_type: ZclDataType::Bool,
                access: AttributeAccess::ReadOnly,
                name: "GlobalSceneControl",
            },
            ZclValue::Bool(true),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ON_TIME,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "OnTime",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OFF_WAIT_TIME,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "OffWaitTime",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_START_UP_ON_OFF,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "StartUpOnOff",
            },
            ZclValue::Enum8(0xFF), // Previous
        );
        Self { store }
    }

    /// Current on/off state.
    pub fn is_on(&self) -> bool {
        matches!(self.store.get(ATTR_ON_OFF), Some(ZclValue::Bool(true)))
    }

    fn set_on_off(&mut self, on: bool) {
        let _ = self.store.set_raw(ATTR_ON_OFF, ZclValue::Bool(on));
    }

    /// Tick the On/Off cluster timers (call every 100ms = 1/10th second).
    ///
    /// OnTime and OffWaitTime are in 1/10th seconds per ZCL spec.
    /// When OnTime reaches 0 while the device is on, it turns off and
    /// OffWaitTime begins counting down.
    pub fn tick(&mut self) {
        let on_time = match self.store.get(ATTR_ON_TIME) {
            Some(ZclValue::U16(v)) => *v,
            _ => 0,
        };
        let off_wait = match self.store.get(ATTR_OFF_WAIT_TIME) {
            Some(ZclValue::U16(v)) => *v,
            _ => 0,
        };

        if self.is_on() && on_time > 0 {
            let new_on = on_time.saturating_sub(1);
            let _ = self.store.set_raw(ATTR_ON_TIME, ZclValue::U16(new_on));
            if new_on == 0 {
                // OnTime expired — turn off, start off-wait countdown
                self.set_on_off(false);
            }
        } else if !self.is_on() && off_wait > 0 {
            let new_wait = off_wait.saturating_sub(1);
            let _ = self
                .store
                .set_raw(ATTR_OFF_WAIT_TIME, ZclValue::U16(new_wait));
        }
    }

    /// Apply StartUpOnOff on device power-on (ZCL spec §3.8.2.2.5).
    ///
    /// 0x00 = Off, 0x01 = On, 0x02 = Toggle, 0xFF = Previous (no change).
    pub fn apply_startup(&mut self, previous_on: bool) {
        let startup = match self.store.get(ATTR_START_UP_ON_OFF) {
            Some(ZclValue::Enum8(v)) => *v,
            _ => 0xFF,
        };
        match startup {
            0x00 => self.set_on_off(false),
            0x01 => self.set_on_off(true),
            0x02 => self.set_on_off(!previous_on),
            _ => self.set_on_off(previous_on), // 0xFF = previous
        }
    }
}

impl Cluster for OnOffCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::ON_OFF
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_OFF => {
                self.set_on_off(false);
                Ok(heapless::Vec::new())
            }
            CMD_ON => {
                self.set_on_off(true);
                Ok(heapless::Vec::new())
            }
            CMD_TOGGLE => {
                let new_state = !self.is_on();
                self.set_on_off(new_state);
                Ok(heapless::Vec::new())
            }
            CMD_OFF_WITH_EFFECT => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                // Effect ID (u8) + Effect Variant (u8) — we just turn off.
                self.set_on_off(false);
                let _ = self
                    .store
                    .set_raw(ATTR_GLOBAL_SCENE_CONTROL, ZclValue::Bool(false));
                Ok(heapless::Vec::new())
            }
            CMD_ON_WITH_RECALL_GLOBAL_SCENE => {
                self.set_on_off(true);
                let _ = self
                    .store
                    .set_raw(ATTR_GLOBAL_SCENE_CONTROL, ZclValue::Bool(true));
                Ok(heapless::Vec::new())
            }
            CMD_ON_WITH_TIMED_OFF => {
                if payload.len() < 5 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let on_off_control = payload[0];
                let on_time = u16::from_le_bytes([payload[1], payload[2]]);
                let off_wait = u16::from_le_bytes([payload[3], payload[4]]);
                // Bit 0 of OnOffControl: "Accept Only When On"
                if on_off_control & 0x01 != 0 && !self.is_on() {
                    return Ok(heapless::Vec::new());
                }
                self.set_on_off(true);
                let _ = self.store.set_raw(ATTR_ON_TIME, ZclValue::U16(on_time));
                let _ = self
                    .store
                    .set_raw(ATTR_OFF_WAIT_TIME, ZclValue::U16(off_wait));
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
        heapless::Vec::from_slice(&[0x00, 0x01, 0x02, 0x40, 0x41, 0x42]).unwrap_or_default()
    }
}
