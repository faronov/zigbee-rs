//! Thermostat cluster (0x0201).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs (temperature values in 0.01 deg C).
pub const ATTR_LOCAL_TEMPERATURE: AttributeId = AttributeId(0x0000);
pub const ATTR_OUTDOOR_TEMPERATURE: AttributeId = AttributeId(0x0001);
pub const ATTR_OCCUPANCY: AttributeId = AttributeId(0x0002);
pub const ATTR_ABS_MIN_HEAT_SETPOINT_LIMIT: AttributeId = AttributeId(0x0003);
pub const ATTR_ABS_MAX_HEAT_SETPOINT_LIMIT: AttributeId = AttributeId(0x0004);
pub const ATTR_ABS_MIN_COOL_SETPOINT_LIMIT: AttributeId = AttributeId(0x0005);
pub const ATTR_ABS_MAX_COOL_SETPOINT_LIMIT: AttributeId = AttributeId(0x0006);
pub const ATTR_OCCUPIED_COOLING_SETPOINT: AttributeId = AttributeId(0x0011);
pub const ATTR_OCCUPIED_HEATING_SETPOINT: AttributeId = AttributeId(0x0012);
pub const ATTR_MIN_HEAT_SETPOINT_LIMIT: AttributeId = AttributeId(0x0015);
pub const ATTR_MAX_HEAT_SETPOINT_LIMIT: AttributeId = AttributeId(0x0016);
pub const ATTR_MIN_COOL_SETPOINT_LIMIT: AttributeId = AttributeId(0x0017);
pub const ATTR_MAX_COOL_SETPOINT_LIMIT: AttributeId = AttributeId(0x0018);
pub const ATTR_CONTROL_SEQUENCE_OF_OPERATION: AttributeId = AttributeId(0x001B);
pub const ATTR_SYSTEM_MODE: AttributeId = AttributeId(0x001C);
pub const ATTR_THERMOSTAT_RUNNING_MODE: AttributeId = AttributeId(0x001E);

// System mode values
pub const SYSTEM_MODE_OFF: u8 = 0x00;
pub const SYSTEM_MODE_AUTO: u8 = 0x01;
pub const SYSTEM_MODE_COOL: u8 = 0x03;
pub const SYSTEM_MODE_HEAT: u8 = 0x04;
pub const SYSTEM_MODE_EMERGENCY_HEAT: u8 = 0x05;
pub const SYSTEM_MODE_FAN_ONLY: u8 = 0x07;

// Command IDs (client to server)
pub const CMD_SETPOINT_RAISE_LOWER: CommandId = CommandId(0x00);
pub const CMD_SET_WEEKLY_SCHEDULE: CommandId = CommandId(0x01);
pub const CMD_GET_WEEKLY_SCHEDULE: CommandId = CommandId(0x02);
pub const CMD_CLEAR_WEEKLY_SCHEDULE: CommandId = CommandId(0x03);

// Command IDs (server to client)
pub const CMD_GET_WEEKLY_SCHEDULE_RESPONSE: CommandId = CommandId(0x00);

/// A single schedule transition (time + setpoints).
#[derive(Debug, Clone)]
pub struct ScheduleTransition {
    pub transition_time: u16, // minutes since midnight
    pub heat_setpoint: Option<i16>,
    pub cool_setpoint: Option<i16>,
}

/// A weekly schedule entry for one or more days.
#[derive(Debug, Clone)]
pub struct WeeklyScheduleEntry {
    pub days_of_week: u8, // bitmask: bit0=Sunday .. bit6=Saturday
    pub mode: u8,         // 0x01=heat, 0x02=cool, 0x03=both
    pub transitions: heapless::Vec<ScheduleTransition, 10>,
}

/// Thermostat cluster implementation.
pub struct ThermostatCluster {
    store: AttributeStore<18>,
    schedule: heapless::Vec<WeeklyScheduleEntry, 16>,
}

impl Default for ThermostatCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl ThermostatCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LOCAL_TEMPERATURE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::Reportable,
                name: "LocalTemperature",
            },
            ZclValue::I16(2200), // 22.00 deg C
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OUTDOOR_TEMPERATURE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "OutdoorTemperature",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPANCY,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "Occupancy",
            },
            ZclValue::U8(0x01), // Occupied
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ABS_MIN_HEAT_SETPOINT_LIMIT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "AbsMinHeatSetpointLimit",
            },
            ZclValue::I16(700), // 7.00 deg C
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ABS_MAX_HEAT_SETPOINT_LIMIT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "AbsMaxHeatSetpointLimit",
            },
            ZclValue::I16(3000), // 30.00 deg C
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ABS_MIN_COOL_SETPOINT_LIMIT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "AbsMinCoolSetpointLimit",
            },
            ZclValue::I16(1600), // 16.00 deg C
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ABS_MAX_COOL_SETPOINT_LIMIT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "AbsMaxCoolSetpointLimit",
            },
            ZclValue::I16(3200), // 32.00 deg C
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPIED_COOLING_SETPOINT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "OccupiedCoolingSetpoint",
            },
            ZclValue::I16(2600), // 26.00 deg C
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPIED_HEATING_SETPOINT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "OccupiedHeatingSetpoint",
            },
            ZclValue::I16(2000), // 20.00 deg C
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_HEAT_SETPOINT_LIMIT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "MinHeatSetpointLimit",
            },
            ZclValue::I16(700),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_HEAT_SETPOINT_LIMIT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "MaxHeatSetpointLimit",
            },
            ZclValue::I16(3000),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_COOL_SETPOINT_LIMIT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "MinCoolSetpointLimit",
            },
            ZclValue::I16(1600),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_COOL_SETPOINT_LIMIT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "MaxCoolSetpointLimit",
            },
            ZclValue::I16(3200),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CONTROL_SEQUENCE_OF_OPERATION,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "ControlSequenceOfOperation",
            },
            ZclValue::Enum8(0x04), // Cooling and heating
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SYSTEM_MODE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "SystemMode",
            },
            ZclValue::Enum8(SYSTEM_MODE_AUTO),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_THERMOSTAT_RUNNING_MODE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "ThermostatRunningMode",
            },
            ZclValue::Enum8(0x00), // Off
        );
        Self {
            store,
            schedule: heapless::Vec::new(),
        }
    }

    /// Update the local temperature reading (in 0.01 deg C).
    pub fn set_local_temperature(&mut self, hundredths: i16) {
        let _ = self
            .store
            .set_raw(ATTR_LOCAL_TEMPERATURE, ZclValue::I16(hundredths));
    }

    /// Advance schedule execution.
    ///
    /// `day_of_week` is a bitmask (bit 0 = Sunday … bit 6 = Saturday).
    /// `minutes_since_midnight` is the current time of day in minutes.
    ///
    /// Finds the latest schedule transition that has already passed for the
    /// given day and applies its setpoints, then updates running mode.
    pub fn tick(&mut self, day_of_week: u8, minutes_since_midnight: u16) {
        let mut best_time: Option<u16> = None;
        let mut best_heat: Option<i16> = None;
        let mut best_cool: Option<i16> = None;

        for entry in self.schedule.iter() {
            if entry.days_of_week & day_of_week == 0 {
                continue;
            }
            for t in entry.transitions.iter() {
                if t.transition_time <= minutes_since_midnight
                    && (best_time.is_none() || t.transition_time >= best_time.unwrap())
                {
                    best_time = Some(t.transition_time);
                    best_heat = t.heat_setpoint;
                    best_cool = t.cool_setpoint;
                }
            }
        }

        if best_time.is_some() {
            if let Some(heat) = best_heat {
                let _ = self
                    .store
                    .set(ATTR_OCCUPIED_HEATING_SETPOINT, ZclValue::I16(heat));
            }
            if let Some(cool) = best_cool {
                let _ = self
                    .store
                    .set(ATTR_OCCUPIED_COOLING_SETPOINT, ZclValue::I16(cool));
            }
        }

        // Update running mode based on system mode and current temperature vs setpoints
        let system_mode = match self.store.get(ATTR_SYSTEM_MODE) {
            Some(ZclValue::Enum8(v)) => *v,
            _ => SYSTEM_MODE_OFF,
        };
        let local_temp = match self.store.get(ATTR_LOCAL_TEMPERATURE) {
            Some(ZclValue::I16(v)) => *v,
            _ => 0,
        };
        let heat_sp = match self.store.get(ATTR_OCCUPIED_HEATING_SETPOINT) {
            Some(ZclValue::I16(v)) => *v,
            _ => 2000,
        };
        let cool_sp = match self.store.get(ATTR_OCCUPIED_COOLING_SETPOINT) {
            Some(ZclValue::I16(v)) => *v,
            _ => 2600,
        };

        let running_mode = match system_mode {
            SYSTEM_MODE_OFF | SYSTEM_MODE_FAN_ONLY => SYSTEM_MODE_OFF,
            SYSTEM_MODE_HEAT | SYSTEM_MODE_EMERGENCY_HEAT => {
                if local_temp < heat_sp {
                    SYSTEM_MODE_HEAT
                } else {
                    SYSTEM_MODE_OFF
                }
            }
            SYSTEM_MODE_COOL => {
                if local_temp > cool_sp {
                    SYSTEM_MODE_COOL
                } else {
                    SYSTEM_MODE_OFF
                }
            }
            SYSTEM_MODE_AUTO => {
                if local_temp < heat_sp {
                    SYSTEM_MODE_HEAT
                } else if local_temp > cool_sp {
                    SYSTEM_MODE_COOL
                } else {
                    SYSTEM_MODE_OFF
                }
            }
            _ => SYSTEM_MODE_OFF,
        };
        let _ = self
            .store
            .set_raw(ATTR_THERMOSTAT_RUNNING_MODE, ZclValue::Enum8(running_mode));
    }
}

impl Cluster for ThermostatCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::THERMOSTAT
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_SETPOINT_RAISE_LOWER => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let mode = payload[0]; // 0=heat, 1=cool, 2=both
                let amount = payload[1] as i8 as i16 * 10; // Convert to 0.01 deg C

                if (mode == 0 || mode == 2)
                    && let Some(ZclValue::I16(cur)) = self.store.get(ATTR_OCCUPIED_HEATING_SETPOINT)
                {
                    let new_val = cur.saturating_add(amount);
                    let _ = self
                        .store
                        .set(ATTR_OCCUPIED_HEATING_SETPOINT, ZclValue::I16(new_val));
                }
                if (mode == 1 || mode == 2)
                    && let Some(ZclValue::I16(cur)) = self.store.get(ATTR_OCCUPIED_COOLING_SETPOINT)
                {
                    let new_val = cur.saturating_add(amount);
                    let _ = self
                        .store
                        .set(ATTR_OCCUPIED_COOLING_SETPOINT, ZclValue::I16(new_val));
                }
                Ok(heapless::Vec::new())
            }
            CMD_SET_WEEKLY_SCHEDULE => {
                // Payload: num_transitions(u8) + day_of_week(u8) + mode(u8) + transitions
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let num_transitions = payload[0] as usize;
                let day_of_week = payload[1];
                let mode = payload[2];
                let mut offset = 3;
                let mut transitions: heapless::Vec<ScheduleTransition, 10> = heapless::Vec::new();
                for _ in 0..num_transitions {
                    if offset + 2 > payload.len() {
                        return Err(ZclStatus::MalformedCommand);
                    }
                    let transition_time =
                        u16::from_le_bytes([payload[offset], payload[offset + 1]]);
                    offset += 2;
                    let heat_setpoint = if mode & 0x01 != 0 {
                        if offset + 2 > payload.len() {
                            return Err(ZclStatus::MalformedCommand);
                        }
                        let v = i16::from_le_bytes([payload[offset], payload[offset + 1]]);
                        offset += 2;
                        Some(v)
                    } else {
                        None
                    };
                    let cool_setpoint = if mode & 0x02 != 0 {
                        if offset + 2 > payload.len() {
                            return Err(ZclStatus::MalformedCommand);
                        }
                        let v = i16::from_le_bytes([payload[offset], payload[offset + 1]]);
                        offset += 2;
                        Some(v)
                    } else {
                        None
                    };
                    let _ = transitions.push(ScheduleTransition {
                        transition_time,
                        heat_setpoint,
                        cool_setpoint,
                    });
                }
                let entry = WeeklyScheduleEntry {
                    days_of_week: day_of_week,
                    mode,
                    transitions,
                };
                // Replace any existing entry for same days+mode, or append
                let mut replaced = false;
                for existing in self.schedule.iter_mut() {
                    if existing.days_of_week == day_of_week && existing.mode == mode {
                        *existing = entry.clone();
                        replaced = true;
                        break;
                    }
                }
                if !replaced {
                    let _ = self.schedule.push(entry);
                }
                Ok(heapless::Vec::new())
            }
            CMD_GET_WEEKLY_SCHEDULE => {
                // Payload: days_to_return(u8) + mode_to_return(u8)
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let days_to_return = payload[0];
                let mode_to_return = payload[1];
                // Find first matching entry
                let mut resp = heapless::Vec::new();
                let mut found = false;
                for entry in self.schedule.iter() {
                    if entry.days_of_week & days_to_return != 0 && entry.mode & mode_to_return != 0
                    {
                        let _ = resp.push(entry.transitions.len() as u8);
                        let _ = resp.push(entry.days_of_week);
                        let _ = resp.push(entry.mode);
                        for t in entry.transitions.iter() {
                            let _ = resp.extend_from_slice(&t.transition_time.to_le_bytes());
                            if entry.mode & 0x01 != 0 {
                                let sp = t.heat_setpoint.unwrap_or(0);
                                let _ = resp.extend_from_slice(&sp.to_le_bytes());
                            }
                            if entry.mode & 0x02 != 0 {
                                let sp = t.cool_setpoint.unwrap_or(0);
                                let _ = resp.extend_from_slice(&sp.to_le_bytes());
                            }
                        }
                        found = true;
                        break;
                    }
                }
                if !found {
                    // Return empty schedule for requested days/mode
                    let _ = resp.push(0);
                    let _ = resp.push(days_to_return);
                    let _ = resp.push(mode_to_return);
                }
                Ok(resp)
            }
            CMD_CLEAR_WEEKLY_SCHEDULE => {
                self.schedule.clear();
                Ok(heapless::Vec::new())
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::from_slice(&[
            CMD_SETPOINT_RAISE_LOWER.0,
            CMD_SET_WEEKLY_SCHEDULE.0,
            CMD_GET_WEEKLY_SCHEDULE.0,
            CMD_CLEAR_WEEKLY_SCHEDULE.0,
        ])
        .unwrap_or_default()
    }

    fn generated_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::from_slice(&[CMD_GET_WEEKLY_SCHEDULE_RESPONSE.0]).unwrap_or_default()
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
