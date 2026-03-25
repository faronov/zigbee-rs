//! Alarms cluster (0x0009).
//!
//! Provides a mechanism for alerting the network about device-specific alarms.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_ALARM_COUNT: AttributeId = AttributeId(0x0000);

// Client→Server command IDs
pub const CMD_RESET_ALARM: CommandId = CommandId(0x00);
pub const CMD_RESET_ALL_ALARMS: CommandId = CommandId(0x01);
pub const CMD_GET_ALARM: CommandId = CommandId(0x02);
pub const CMD_RESET_ALARM_LOG: CommandId = CommandId(0x03);

// Server→Client command IDs
pub const CMD_ALARM: CommandId = CommandId(0x00);
pub const CMD_GET_ALARM_RESPONSE: CommandId = CommandId(0x01);

/// Alarms cluster implementation.
pub struct AlarmsCluster {
    store: AttributeStore<1>,
}

impl Default for AlarmsCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl AlarmsCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ALARM_COUNT,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "AlarmCount",
            },
            ZclValue::U16(0),
        );
        Self { store }
    }

    /// Build an Alarm notification payload (server→client).
    pub fn build_alarm_payload(alarm_code: u8, cluster_id: u16) -> heapless::Vec<u8, 64> {
        let mut v = heapless::Vec::new();
        let _ = v.push(alarm_code);
        let _ = v.extend_from_slice(&cluster_id.to_le_bytes());
        v
    }

    /// Build a GetAlarmResponse payload (server→client).
    pub fn build_get_alarm_response(
        status: u8,
        alarm_code: u8,
        cluster_id: u16,
        timestamp: u32,
    ) -> heapless::Vec<u8, 64> {
        let mut v = heapless::Vec::new();
        let _ = v.push(status);
        let _ = v.push(alarm_code);
        let _ = v.extend_from_slice(&cluster_id.to_le_bytes());
        let _ = v.extend_from_slice(&timestamp.to_le_bytes());
        v
    }
}

impl Cluster for AlarmsCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0009)
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_RESET_ALARM => {
                if payload.len() < 3 {
                    return Err(ZclStatus::MalformedCommand);
                }
                // alarm_code (u8) + cluster_id (u16 LE) — acknowledged
                Ok(heapless::Vec::new())
            }
            CMD_RESET_ALL_ALARMS => {
                let _ = self.store.set_raw(ATTR_ALARM_COUNT, ZclValue::U16(0));
                Ok(heapless::Vec::new())
            }
            CMD_GET_ALARM => {
                // Respond with GetAlarmResponse: no pending alarms → status 0x01 (NOT_FOUND)
                Ok(Self::build_get_alarm_response(0x01, 0x00, 0x0000, 0))
            }
            CMD_RESET_ALARM_LOG => {
                let _ = self.store.set_raw(ATTR_ALARM_COUNT, ZclValue::U16(0));
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
