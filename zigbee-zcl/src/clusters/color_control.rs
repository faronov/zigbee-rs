//! Color Control cluster (0x0300).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::transition::TransitionManager;
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_CURRENT_HUE: AttributeId = AttributeId(0x0000);
pub const ATTR_CURRENT_SATURATION: AttributeId = AttributeId(0x0001);
pub const ATTR_REMAINING_TIME: AttributeId = AttributeId(0x0002);
pub const ATTR_CURRENT_X: AttributeId = AttributeId(0x0003);
pub const ATTR_CURRENT_Y: AttributeId = AttributeId(0x0004);
pub const ATTR_COLOR_TEMPERATURE_MIREDS: AttributeId = AttributeId(0x0007);
pub const ATTR_COLOR_MODE: AttributeId = AttributeId(0x0008);
pub const ATTR_OPTIONS: AttributeId = AttributeId(0x000F);
pub const ATTR_ENHANCED_CURRENT_HUE: AttributeId = AttributeId(0x4000);
pub const ATTR_ENHANCED_COLOR_MODE: AttributeId = AttributeId(0x4001);
pub const ATTR_COLOR_LOOP_ACTIVE: AttributeId = AttributeId(0x4002);
pub const ATTR_COLOR_LOOP_DIRECTION: AttributeId = AttributeId(0x4003);
pub const ATTR_COLOR_LOOP_TIME: AttributeId = AttributeId(0x4004);
pub const ATTR_COLOR_CAPABILITIES: AttributeId = AttributeId(0x400A);
pub const ATTR_COLOR_TEMP_PHYSICAL_MIN: AttributeId = AttributeId(0x400B);
pub const ATTR_COLOR_TEMP_PHYSICAL_MAX: AttributeId = AttributeId(0x400C);

// Command IDs
pub const CMD_MOVE_TO_HUE: CommandId = CommandId(0x00);
pub const CMD_MOVE_HUE: CommandId = CommandId(0x01);
pub const CMD_STEP_HUE: CommandId = CommandId(0x02);
pub const CMD_MOVE_TO_SATURATION: CommandId = CommandId(0x03);
pub const CMD_MOVE_SATURATION: CommandId = CommandId(0x04);
pub const CMD_STEP_SATURATION: CommandId = CommandId(0x05);
pub const CMD_MOVE_TO_HUE_AND_SATURATION: CommandId = CommandId(0x06);
pub const CMD_MOVE_TO_COLOR: CommandId = CommandId(0x07);
pub const CMD_MOVE_COLOR: CommandId = CommandId(0x08);
pub const CMD_STEP_COLOR: CommandId = CommandId(0x09);
pub const CMD_MOVE_TO_COLOR_TEMPERATURE: CommandId = CommandId(0x0A);
pub const CMD_ENHANCED_MOVE_TO_HUE: CommandId = CommandId(0x40);
pub const CMD_ENHANCED_MOVE_HUE: CommandId = CommandId(0x41);
pub const CMD_ENHANCED_STEP_HUE: CommandId = CommandId(0x42);
pub const CMD_ENHANCED_MOVE_TO_HUE_AND_SATURATION: CommandId = CommandId(0x43);
pub const CMD_COLOR_LOOP_SET: CommandId = CommandId(0x44);
pub const CMD_STOP_MOVE_STEP: CommandId = CommandId(0x47);
pub const CMD_MOVE_COLOR_TEMPERATURE: CommandId = CommandId(0x4B);
pub const CMD_STEP_COLOR_TEMPERATURE: CommandId = CommandId(0x4C);

/// Color mode values.
pub const COLOR_MODE_HUE_SAT: u8 = 0x00;
pub const COLOR_MODE_XY: u8 = 0x01;
pub const COLOR_MODE_TEMPERATURE: u8 = 0x02;

/// Color Control cluster implementation.
pub struct ColorControlCluster {
    store: AttributeStore<20>,
    pub transitions: TransitionManager<4>,
}

impl Default for ColorControlCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl ColorControlCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_HUE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "CurrentHue",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_SATURATION,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "CurrentSaturation",
            },
            ZclValue::U8(0),
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
                id: ATTR_CURRENT_X,
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "CurrentX",
            },
            ZclValue::U16(0x616B), // Default per spec
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_Y,
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "CurrentY",
            },
            ZclValue::U16(0x607D), // Default per spec
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_COLOR_TEMPERATURE_MIREDS,
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "ColorTemperatureMireds",
            },
            ZclValue::U16(250),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_COLOR_MODE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "ColorMode",
            },
            ZclValue::Enum8(COLOR_MODE_XY),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OPTIONS,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadWrite,
                name: "Options",
            },
            ZclValue::Bitmap8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ENHANCED_CURRENT_HUE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "EnhancedCurrentHue",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ENHANCED_COLOR_MODE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "EnhancedColorMode",
            },
            ZclValue::Enum8(COLOR_MODE_XY),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_COLOR_LOOP_ACTIVE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ColorLoopActive",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_COLOR_LOOP_DIRECTION,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "ColorLoopDirection",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_COLOR_LOOP_TIME,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ColorLoopTime",
            },
            ZclValue::U16(25),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_COLOR_CAPABILITIES,
                data_type: ZclDataType::Bitmap16,
                access: AttributeAccess::ReadOnly,
                name: "ColorCapabilities",
            },
            ZclValue::Bitmap16(0x001F), // All capabilities
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_COLOR_TEMP_PHYSICAL_MIN,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ColorTempPhysicalMinMireds",
            },
            ZclValue::U16(153), // ~6500K
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_COLOR_TEMP_PHYSICAL_MAX,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ColorTempPhysicalMaxMireds",
            },
            ZclValue::U16(500), // ~2000K
        );
        Self {
            store,
            transitions: TransitionManager::new(),
        }
    }

    fn get_u8_attr(&self, attr: AttributeId) -> u8 {
        match self.store.get(attr) {
            Some(ZclValue::U8(v)) => *v,
            _ => 0,
        }
    }

    fn get_u16_attr(&self, attr: AttributeId) -> u16 {
        match self.store.get(attr) {
            Some(ZclValue::U16(v)) => *v,
            _ => 0,
        }
    }

    /// Advance all active transitions by `elapsed_ds` deciseconds.
    pub fn tick(&mut self, elapsed_ds: u16) {
        let updates = self.transitions.tick(elapsed_ds);
        for (attr_id, value) in &updates {
            match *attr_id {
                0x0000 => {
                    let v = (*value).clamp(0, 254) as u8;
                    let _ = self.store.set_raw(ATTR_CURRENT_HUE, ZclValue::U8(v));
                }
                0x0001 => {
                    let v = (*value).clamp(0, 254) as u8;
                    let _ = self.store.set_raw(ATTR_CURRENT_SATURATION, ZclValue::U8(v));
                }
                0x0003 => {
                    let v = (*value).clamp(0, 0xFEFF) as u16;
                    let _ = self.store.set_raw(ATTR_CURRENT_X, ZclValue::U16(v));
                }
                0x0004 => {
                    let v = (*value).clamp(0, 0xFEFF) as u16;
                    let _ = self.store.set_raw(ATTR_CURRENT_Y, ZclValue::U16(v));
                }
                0x4000 => {
                    let v = (*value).clamp(0, 0xFFFF) as u16;
                    let _ = self
                        .store
                        .set_raw(ATTR_ENHANCED_CURRENT_HUE, ZclValue::U16(v));
                }
                _ => {}
            }
        }
        let remaining = self.transitions.max_remaining_ds();
        let _ = self
            .store
            .set_raw(ATTR_REMAINING_TIME, ZclValue::U16(remaining));

        // Color loop engine: cycle enhanced hue when active
        let loop_active = self.get_u8_attr(ATTR_COLOR_LOOP_ACTIVE);
        if loop_active != 0 {
            let loop_time = self.get_u16_attr(ATTR_COLOR_LOOP_TIME);
            let direction = self.get_u8_attr(ATTR_COLOR_LOOP_DIRECTION);
            if loop_time > 0 {
                let current = self.get_u16_attr(ATTR_ENHANCED_CURRENT_HUE);
                // loop_time is in seconds; elapsed_ds is in 1/10th seconds
                // Full hue cycle (0..65535) in loop_time seconds
                // Step per tick = 65536 / (loop_time * 10 / elapsed_ds)
                let steps_per_cycle = (loop_time as u32) * 10;
                let hue_step = (65536u32 * elapsed_ds as u32)
                    .checked_div(steps_per_cycle)
                    .unwrap_or(1) as u16;
                let new_hue = if direction == 0x01 {
                    // Increment
                    current.wrapping_add(hue_step)
                } else {
                    // Decrement
                    current.wrapping_sub(hue_step)
                };
                let _ = self
                    .store
                    .set_raw(ATTR_ENHANCED_CURRENT_HUE, ZclValue::U16(new_hue));
                // Update 8-bit hue too (high byte of enhanced)
                let _ = self
                    .store
                    .set_raw(ATTR_CURRENT_HUE, ZclValue::U8((new_hue >> 8) as u8));
            }
        }
    }
}

impl Cluster for ColorControlCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::COLOR_CONTROL
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_MOVE_TO_HUE => {
                if payload.len() < 4 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let hue = payload[0];
                let _direction = payload[1];
                let transition = u16::from_le_bytes([payload[2], payload[3]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_HUE_SAT));
                if transition == 0 {
                    let _ = self.store.set_raw(ATTR_CURRENT_HUE, ZclValue::U8(hue));
                    self.transitions.stop(ATTR_CURRENT_HUE.0);
                } else {
                    let current = self.get_u8_attr(ATTR_CURRENT_HUE) as i32;
                    self.transitions
                        .start(ATTR_CURRENT_HUE.0, current, hue as i32, transition);
                }
                Ok(heapless::Vec::new())
            }
            CMD_MOVE_HUE => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0]; // 0=stop, 1=up, 3=down
                let rate = payload[1];
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_HUE_SAT));
                if mode == 0 {
                    self.transitions.stop(ATTR_CURRENT_HUE.0);
                } else {
                    let current = self.get_u8_attr(ATTR_CURRENT_HUE) as i32;
                    let target: i32 = if mode == 1 { 254 } else { 0 };
                    let diff = (target - current).unsigned_abs();
                    let time_ds = if rate > 0 {
                        ((diff as u16) * 10 / (rate as u16)).max(1)
                    } else {
                        1
                    };
                    self.transitions
                        .start(ATTR_CURRENT_HUE.0, current, target, time_ds);
                }
                Ok(heapless::Vec::new())
            }
            CMD_STEP_HUE => {
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0]; // 1=up, 3=down
                let step_size = payload[1];
                let transition_time = payload[2] as u16;
                let current = self.get_u8_attr(ATTR_CURRENT_HUE);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_HUE_SAT));
                let target = if mode == 1 {
                    current.saturating_add(step_size).min(254)
                } else {
                    current.saturating_sub(step_size)
                };
                if transition_time == 0 {
                    let _ = self.store.set_raw(ATTR_CURRENT_HUE, ZclValue::U8(target));
                    self.transitions.stop(ATTR_CURRENT_HUE.0);
                } else {
                    self.transitions.start(
                        ATTR_CURRENT_HUE.0,
                        current as i32,
                        target as i32,
                        transition_time,
                    );
                }
                Ok(heapless::Vec::new())
            }
            CMD_MOVE_TO_SATURATION => {
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let sat = payload[0];
                let transition = u16::from_le_bytes([payload[1], payload[2]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_HUE_SAT));
                if transition == 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_CURRENT_SATURATION, ZclValue::U8(sat));
                    self.transitions.stop(ATTR_CURRENT_SATURATION.0);
                } else {
                    let current = self.get_u8_attr(ATTR_CURRENT_SATURATION) as i32;
                    self.transitions.start(
                        ATTR_CURRENT_SATURATION.0,
                        current,
                        sat as i32,
                        transition,
                    );
                }
                Ok(heapless::Vec::new())
            }
            CMD_MOVE_SATURATION => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0]; // 0=stop, 1=up, 3=down
                let rate = payload[1];
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_HUE_SAT));
                if mode == 0 {
                    self.transitions.stop(ATTR_CURRENT_SATURATION.0);
                } else {
                    let current = self.get_u8_attr(ATTR_CURRENT_SATURATION) as i32;
                    let target: i32 = if mode == 1 { 254 } else { 0 };
                    let diff = (target - current).unsigned_abs();
                    let time_ds = if rate > 0 {
                        ((diff as u16) * 10 / (rate as u16)).max(1)
                    } else {
                        1
                    };
                    self.transitions
                        .start(ATTR_CURRENT_SATURATION.0, current, target, time_ds);
                }
                Ok(heapless::Vec::new())
            }
            CMD_STEP_SATURATION => {
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0]; // 1=up, 3=down
                let step_size = payload[1];
                let transition_time = payload[2] as u16;
                let current = self.get_u8_attr(ATTR_CURRENT_SATURATION);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_HUE_SAT));
                let target = if mode == 1 {
                    current.saturating_add(step_size).min(254)
                } else {
                    current.saturating_sub(step_size)
                };
                if transition_time == 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_CURRENT_SATURATION, ZclValue::U8(target));
                    self.transitions.stop(ATTR_CURRENT_SATURATION.0);
                } else {
                    self.transitions.start(
                        ATTR_CURRENT_SATURATION.0,
                        current as i32,
                        target as i32,
                        transition_time,
                    );
                }
                Ok(heapless::Vec::new())
            }
            CMD_MOVE_TO_HUE_AND_SATURATION => {
                if payload.len() < 4 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let hue = payload[0];
                let sat = payload[1];
                let transition = u16::from_le_bytes([payload[2], payload[3]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_HUE_SAT));
                if transition == 0 {
                    let _ = self.store.set_raw(ATTR_CURRENT_HUE, ZclValue::U8(hue));
                    let _ = self
                        .store
                        .set_raw(ATTR_CURRENT_SATURATION, ZclValue::U8(sat));
                    self.transitions.stop(ATTR_CURRENT_HUE.0);
                    self.transitions.stop(ATTR_CURRENT_SATURATION.0);
                } else {
                    let cur_hue = self.get_u8_attr(ATTR_CURRENT_HUE) as i32;
                    let cur_sat = self.get_u8_attr(ATTR_CURRENT_SATURATION) as i32;
                    self.transitions
                        .start(ATTR_CURRENT_HUE.0, cur_hue, hue as i32, transition);
                    self.transitions.start(
                        ATTR_CURRENT_SATURATION.0,
                        cur_sat,
                        sat as i32,
                        transition,
                    );
                }
                Ok(heapless::Vec::new())
            }
            CMD_MOVE_TO_COLOR => {
                if payload.len() < 6 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let x = u16::from_le_bytes([payload[0], payload[1]]);
                let y = u16::from_le_bytes([payload[2], payload[3]]);
                let transition = u16::from_le_bytes([payload[4], payload[5]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_XY));
                if transition == 0 {
                    let _ = self.store.set_raw(ATTR_CURRENT_X, ZclValue::U16(x));
                    let _ = self.store.set_raw(ATTR_CURRENT_Y, ZclValue::U16(y));
                    self.transitions.stop(ATTR_CURRENT_X.0);
                    self.transitions.stop(ATTR_CURRENT_Y.0);
                } else {
                    let cur_x = self.get_u16_attr(ATTR_CURRENT_X) as i32;
                    let cur_y = self.get_u16_attr(ATTR_CURRENT_Y) as i32;
                    self.transitions
                        .start(ATTR_CURRENT_X.0, cur_x, x as i32, transition);
                    self.transitions
                        .start(ATTR_CURRENT_Y.0, cur_y, y as i32, transition);
                }
                Ok(heapless::Vec::new())
            }
            CMD_MOVE_COLOR => {
                if payload.len() < 4 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let rate_x = i16::from_le_bytes([payload[0], payload[1]]);
                let rate_y = i16::from_le_bytes([payload[2], payload[3]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_XY));
                // Continuous move: calculate target based on rate direction
                if rate_x == 0 && rate_y == 0 {
                    self.transitions.stop(ATTR_CURRENT_X.0);
                    self.transitions.stop(ATTR_CURRENT_Y.0);
                } else {
                    let cur_x = self.get_u16_attr(ATTR_CURRENT_X) as i32;
                    let cur_y = self.get_u16_attr(ATTR_CURRENT_Y) as i32;
                    // Move for ~25 seconds (250 ds) toward limit
                    let time_ds: u16 = 250;
                    let target_x =
                        (cur_x + (rate_x as i32) * (time_ds as i32) / 10).clamp(0, 0xFEFF);
                    let target_y =
                        (cur_y + (rate_y as i32) * (time_ds as i32) / 10).clamp(0, 0xFEFF);
                    self.transitions
                        .start(ATTR_CURRENT_X.0, cur_x, target_x, time_ds);
                    self.transitions
                        .start(ATTR_CURRENT_Y.0, cur_y, target_y, time_ds);
                }
                Ok(heapless::Vec::new())
            }
            CMD_STEP_COLOR => {
                if payload.len() < 6 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let step_x = i16::from_le_bytes([payload[0], payload[1]]);
                let step_y = i16::from_le_bytes([payload[2], payload[3]]);
                let transition = u16::from_le_bytes([payload[4], payload[5]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_XY));
                let cur_x = self.get_u16_attr(ATTR_CURRENT_X) as i32;
                let cur_y = self.get_u16_attr(ATTR_CURRENT_Y) as i32;
                let target_x = (cur_x + step_x as i32).clamp(0, 0xFEFF);
                let target_y = (cur_y + step_y as i32).clamp(0, 0xFEFF);
                if transition == 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_CURRENT_X, ZclValue::U16(target_x as u16));
                    let _ = self
                        .store
                        .set_raw(ATTR_CURRENT_Y, ZclValue::U16(target_y as u16));
                    self.transitions.stop(ATTR_CURRENT_X.0);
                    self.transitions.stop(ATTR_CURRENT_Y.0);
                } else {
                    self.transitions
                        .start(ATTR_CURRENT_X.0, cur_x, target_x, transition);
                    self.transitions
                        .start(ATTR_CURRENT_Y.0, cur_y, target_y, transition);
                }
                Ok(heapless::Vec::new())
            }
            CMD_MOVE_TO_COLOR_TEMPERATURE => {
                if payload.len() < 4 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mireds = u16::from_le_bytes([payload[0], payload[1]]);
                let _transition = u16::from_le_bytes([payload[2], payload[3]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_TEMPERATURE_MIREDS, ZclValue::U16(mireds));
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_TEMPERATURE));
                Ok(heapless::Vec::new())
            }
            CMD_ENHANCED_MOVE_TO_HUE => {
                if payload.len() < 5 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let ehue = u16::from_le_bytes([payload[0], payload[1]]);
                let _direction = payload[2];
                let transition = u16::from_le_bytes([payload[3], payload[4]]);
                let _ = self
                    .store
                    .set_raw(ATTR_ENHANCED_COLOR_MODE, ZclValue::Enum8(0x03));
                if transition == 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_ENHANCED_CURRENT_HUE, ZclValue::U16(ehue));
                    self.transitions.stop(ATTR_ENHANCED_CURRENT_HUE.0);
                } else {
                    let current = self.get_u16_attr(ATTR_ENHANCED_CURRENT_HUE) as i32;
                    self.transitions.start(
                        ATTR_ENHANCED_CURRENT_HUE.0,
                        current,
                        ehue as i32,
                        transition,
                    );
                }
                Ok(heapless::Vec::new())
            }
            CMD_ENHANCED_MOVE_HUE => {
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0]; // 0=stop, 1=up, 3=down
                let rate = u16::from_le_bytes([payload[1], payload[2]]);
                let _ = self
                    .store
                    .set_raw(ATTR_ENHANCED_COLOR_MODE, ZclValue::Enum8(0x03));
                if mode == 0 {
                    self.transitions.stop(ATTR_ENHANCED_CURRENT_HUE.0);
                } else {
                    let current = self.get_u16_attr(ATTR_ENHANCED_CURRENT_HUE) as i32;
                    let target: i32 = if mode == 1 { 0xFFFE } else { 0 };
                    let diff = (target - current).unsigned_abs();
                    let time_ds = if rate > 0 {
                        (diff * 10 / (rate as u32)).min(0xFFFF) as u16
                    } else {
                        1
                    };
                    let time_ds = time_ds.max(1);
                    self.transitions
                        .start(ATTR_ENHANCED_CURRENT_HUE.0, current, target, time_ds);
                }
                Ok(heapless::Vec::new())
            }
            CMD_ENHANCED_STEP_HUE => {
                if payload.len() < 5 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0]; // 1=up, 3=down
                let step_size = u16::from_le_bytes([payload[1], payload[2]]);
                let transition_time = u16::from_le_bytes([payload[3], payload[4]]);
                let current = self.get_u16_attr(ATTR_ENHANCED_CURRENT_HUE);
                let _ = self
                    .store
                    .set_raw(ATTR_ENHANCED_COLOR_MODE, ZclValue::Enum8(0x03));
                let target = if mode == 1 {
                    current.saturating_add(step_size)
                } else {
                    current.saturating_sub(step_size)
                };
                if transition_time == 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_ENHANCED_CURRENT_HUE, ZclValue::U16(target));
                    self.transitions.stop(ATTR_ENHANCED_CURRENT_HUE.0);
                } else {
                    self.transitions.start(
                        ATTR_ENHANCED_CURRENT_HUE.0,
                        current as i32,
                        target as i32,
                        transition_time,
                    );
                }
                Ok(heapless::Vec::new())
            }
            CMD_ENHANCED_MOVE_TO_HUE_AND_SATURATION => {
                if payload.len() < 5 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let ehue = u16::from_le_bytes([payload[0], payload[1]]);
                let sat = payload[2];
                let transition = u16::from_le_bytes([payload[3], payload[4]]);
                let _ = self
                    .store
                    .set_raw(ATTR_ENHANCED_COLOR_MODE, ZclValue::Enum8(0x03));
                if transition == 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_ENHANCED_CURRENT_HUE, ZclValue::U16(ehue));
                    let _ = self
                        .store
                        .set_raw(ATTR_CURRENT_SATURATION, ZclValue::U8(sat));
                } else {
                    let cur_ehue = self.get_u16_attr(ATTR_ENHANCED_CURRENT_HUE) as i32;
                    let cur_sat = self.get_u8_attr(ATTR_CURRENT_SATURATION) as i32;
                    self.transitions.start(
                        ATTR_ENHANCED_CURRENT_HUE.0,
                        cur_ehue,
                        ehue as i32,
                        transition,
                    );
                    self.transitions.start(
                        ATTR_CURRENT_SATURATION.0,
                        cur_sat,
                        sat as i32,
                        transition,
                    );
                }
                Ok(heapless::Vec::new())
            }
            CMD_STOP_MOVE_STEP => {
                self.transitions.stop_all();
                let _ = self.store.set_raw(ATTR_REMAINING_TIME, ZclValue::U16(0));
                Ok(heapless::Vec::new())
            }
            CMD_COLOR_LOOP_SET => {
                if payload.len() < 7 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let update_flags = payload[0];
                let action = payload[1];
                let direction = payload[2];
                let time = u16::from_le_bytes([payload[3], payload[4]]);
                let _start_hue = u16::from_le_bytes([payload[5], payload[6]]);
                if update_flags & 0x01 != 0 {
                    let _ = self.store.set_raw(
                        ATTR_COLOR_LOOP_ACTIVE,
                        ZclValue::U8(if action > 0 { 1 } else { 0 }),
                    );
                }
                if update_flags & 0x02 != 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_COLOR_LOOP_DIRECTION, ZclValue::U8(direction));
                }
                if update_flags & 0x04 != 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_COLOR_LOOP_TIME, ZclValue::U16(time));
                }
                Ok(heapless::Vec::new())
            }
            CMD_MOVE_COLOR_TEMPERATURE => {
                if payload.len() < 7 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let move_mode = payload[0];
                let rate = u16::from_le_bytes([payload[1], payload[2]]);
                let min_ct = u16::from_le_bytes([payload[3], payload[4]]);
                let max_ct = u16::from_le_bytes([payload[5], payload[6]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_TEMPERATURE));
                let current = self.get_u16_attr(ATTR_COLOR_TEMPERATURE_MIREDS);
                match move_mode {
                    0x01 => {
                        // Move up
                        let target = if max_ct > 0 { max_ct } else { 65279 };
                        let distance = target.saturating_sub(current) as i32;
                        let time_ds = if rate > 0 {
                            (distance * 10 / rate as i32).max(1) as u16
                        } else {
                            1
                        };
                        self.transitions.start(
                            ATTR_COLOR_TEMPERATURE_MIREDS.0,
                            current as i32,
                            target as i32,
                            time_ds,
                        );
                    }
                    0x03 => {
                        // Move down
                        let target = if min_ct > 0 { min_ct } else { 0 };
                        let distance = current.saturating_sub(target) as i32;
                        let time_ds = if rate > 0 {
                            (distance * 10 / rate as i32).max(1) as u16
                        } else {
                            1
                        };
                        self.transitions.start(
                            ATTR_COLOR_TEMPERATURE_MIREDS.0,
                            current as i32,
                            target as i32,
                            time_ds,
                        );
                    }
                    _ => {
                        // Stop
                        self.transitions.stop(ATTR_COLOR_TEMPERATURE_MIREDS.0);
                    }
                }
                Ok(heapless::Vec::new())
            }
            CMD_STEP_COLOR_TEMPERATURE => {
                if payload.len() < 9 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let step_mode = payload[0];
                let step_size = u16::from_le_bytes([payload[1], payload[2]]);
                let transition = u16::from_le_bytes([payload[3], payload[4]]);
                let min_ct = u16::from_le_bytes([payload[5], payload[6]]);
                let max_ct = u16::from_le_bytes([payload[7], payload[8]]);
                let _ = self
                    .store
                    .set_raw(ATTR_COLOR_MODE, ZclValue::Enum8(COLOR_MODE_TEMPERATURE));
                let current = self.get_u16_attr(ATTR_COLOR_TEMPERATURE_MIREDS) as i32;
                let target = match step_mode {
                    0x01 => {
                        let t = (current + step_size as i32).min(65279);
                        if max_ct > 0 { t.min(max_ct as i32) } else { t }
                    }
                    0x03 => {
                        let t = (current - step_size as i32).max(0);
                        if min_ct > 0 { t.max(min_ct as i32) } else { t }
                    }
                    _ => current,
                };
                if transition == 0 {
                    let _ = self
                        .store
                        .set_raw(ATTR_COLOR_TEMPERATURE_MIREDS, ZclValue::U16(target as u16));
                } else {
                    self.transitions.start(
                        ATTR_COLOR_TEMPERATURE_MIREDS.0,
                        current,
                        target,
                        transition,
                    );
                }
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
        heapless::Vec::from_slice(&[
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x40, 0x41, 0x42,
            0x43, 0x44, 0x47, 0x4B, 0x4C,
        ])
        .unwrap_or_default()
    }
}
