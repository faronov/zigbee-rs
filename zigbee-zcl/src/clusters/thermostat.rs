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

/// Thermostat cluster implementation.
pub struct ThermostatCluster {
    store: AttributeStore<18>,
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
        Self { store }
    }

    /// Update the local temperature reading (in 0.01 deg C).
    pub fn set_local_temperature(&mut self, hundredths: i16) {
        let _ = self
            .store
            .set_raw(ATTR_LOCAL_TEMPERATURE, ZclValue::I16(hundredths));
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
                // Stub: accept but do nothing beyond acknowledging.
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                Ok(heapless::Vec::new())
            }
            CMD_GET_WEEKLY_SCHEDULE => {
                // Payload: days_to_return(u8) + mode_to_return(u8)
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                // Return empty schedule
                let mut resp = heapless::Vec::new();
                let _ = resp.push(0); // number of transitions
                let _ = resp.push(payload[0]); // days
                let _ = resp.push(payload[1]); // mode
                Ok(resp)
            }
            CMD_CLEAR_WEEKLY_SCHEDULE => Ok(heapless::Vec::new()),
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
