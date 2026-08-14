//! IAS ACE (Ancillary Control Equipment) cluster (0x0501).
//!
//! Command-driven cluster for security keypads and control panels.
//! Provides arm/disarm, emergency/fire/panic, zone information,
//! and panel status exchange between ACE devices and the IAS CIE.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Synthetic attribute for internal panel state tracking
pub const ATTR_PANEL_STATUS: AttributeId = AttributeId(0xFF00);

// Client → Server command IDs (from keypad to panel)
pub const CMD_ARM: CommandId = CommandId(0x00);
pub const CMD_BYPASS: CommandId = CommandId(0x01);
pub const CMD_EMERGENCY: CommandId = CommandId(0x02);
pub const CMD_FIRE: CommandId = CommandId(0x03);
pub const CMD_PANIC: CommandId = CommandId(0x04);
pub const CMD_GET_ZONE_ID_MAP: CommandId = CommandId(0x05);
pub const CMD_GET_ZONE_INFORMATION: CommandId = CommandId(0x06);
pub const CMD_GET_PANEL_STATUS: CommandId = CommandId(0x07);
pub const CMD_GET_BYPASSED_ZONE_LIST: CommandId = CommandId(0x08);
pub const CMD_GET_ZONE_STATUS: CommandId = CommandId(0x09);

// Server → Client command IDs (from panel to keypad)
pub const CMD_ARM_RESPONSE: CommandId = CommandId(0x00);
pub const CMD_GET_ZONE_ID_MAP_RESPONSE: CommandId = CommandId(0x01);
pub const CMD_GET_ZONE_INFORMATION_RESPONSE: CommandId = CommandId(0x02);
pub const CMD_ZONE_STATUS_CHANGED: CommandId = CommandId(0x03);
pub const CMD_PANEL_STATUS_CHANGED: CommandId = CommandId(0x04);
pub const CMD_GET_PANEL_STATUS_RESPONSE: CommandId = CommandId(0x05);
pub const CMD_SET_BYPASSED_ZONE_LIST: CommandId = CommandId(0x06);
pub const CMD_BYPASS_RESPONSE: CommandId = CommandId(0x07);
pub const CMD_GET_ZONE_STATUS_RESPONSE: CommandId = CommandId(0x08);

// Arm mode values
pub const ARM_MODE_DISARM: u8 = 0x00;
pub const ARM_MODE_ARM_DAY_ZONES_ONLY: u8 = 0x01;
pub const ARM_MODE_ARM_NIGHT_ZONES_ONLY: u8 = 0x02;
pub const ARM_MODE_ARM_ALL_ZONES: u8 = 0x03;

// Arm notification values
pub const ARM_NOTIF_ALL_ZONES_DISARMED: u8 = 0x00;
pub const ARM_NOTIF_ONLY_DAY_ZONES_ARMED: u8 = 0x01;
pub const ARM_NOTIF_ONLY_NIGHT_ZONES_ARMED: u8 = 0x02;
pub const ARM_NOTIF_ALL_ZONES_ARMED: u8 = 0x03;
pub const ARM_NOTIF_INVALID_ARM_CODE: u8 = 0x04;
pub const ARM_NOTIF_NOT_READY_TO_ARM: u8 = 0x05;
pub const ARM_NOTIF_ALREADY_DISARMED: u8 = 0x06;

// Panel status values
pub const PANEL_STATUS_DISARMED: u8 = 0x00;
pub const PANEL_STATUS_ARMED_STAY: u8 = 0x01;
pub const PANEL_STATUS_ARMED_NIGHT: u8 = 0x02;
pub const PANEL_STATUS_ARMED_AWAY: u8 = 0x03;
pub const PANEL_STATUS_EXIT_DELAY: u8 = 0x04;
pub const PANEL_STATUS_ENTRY_DELAY: u8 = 0x05;
pub const PANEL_STATUS_NOT_READY_TO_ARM: u8 = 0x06;
pub const PANEL_STATUS_IN_ALARM: u8 = 0x07;

// Alarm status values
pub const ALARM_STATUS_NO_ALARM: u8 = 0x00;
pub const ALARM_STATUS_BURGLAR: u8 = 0x01;
pub const ALARM_STATUS_FIRE: u8 = 0x02;
pub const ALARM_STATUS_EMERGENCY: u8 = 0x03;
pub const ALARM_STATUS_POLICE_PANIC: u8 = 0x04;
pub const ALARM_STATUS_FIRE_PANIC: u8 = 0x05;
pub const ALARM_STATUS_EMERGENCY_PANIC: u8 = 0x06;

/// IAS ACE cluster implementation.
pub struct IasAceCluster {
    store: AttributeStore<1>,
}

impl Default for IasAceCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl IasAceCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_PANEL_STATUS,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "PanelStatus",
            },
            ZclValue::Enum8(PANEL_STATUS_DISARMED),
        );
        Self { store }
    }

    /// Build a PanelStatusChanged / GetPanelStatusResponse payload.
    fn build_panel_status_payload(&self, alarm_status: u8) -> heapless::Vec<u8, 64> {
        let panel_status = match self.store.get(ATTR_PANEL_STATUS) {
            Some(ZclValue::Enum8(v)) => *v,
            _ => PANEL_STATUS_DISARMED,
        };
        let mut resp = heapless::Vec::new();
        let _ = resp.push(panel_status); // panel_status
        let _ = resp.push(0x00); // seconds_remaining
        let _ = resp.push(0x00); // audible_notification (default mute)
        let _ = resp.push(alarm_status); // alarm_status
        resp
    }
}

impl Cluster for IasAceCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0501)
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_ARM => {
                if payload.is_empty() {
                    return Err(ZclStatus::MalformedCommand);
                }
                let arm_mode = payload[0];
                let notification = match arm_mode {
                    ARM_MODE_DISARM => {
                        let _ = self
                            .store
                            .set_raw(ATTR_PANEL_STATUS, ZclValue::Enum8(PANEL_STATUS_DISARMED));
                        ARM_NOTIF_ALL_ZONES_DISARMED
                    }
                    ARM_MODE_ARM_DAY_ZONES_ONLY => {
                        let _ = self
                            .store
                            .set_raw(ATTR_PANEL_STATUS, ZclValue::Enum8(PANEL_STATUS_ARMED_STAY));
                        ARM_NOTIF_ONLY_DAY_ZONES_ARMED
                    }
                    ARM_MODE_ARM_NIGHT_ZONES_ONLY => {
                        let _ = self
                            .store
                            .set_raw(ATTR_PANEL_STATUS, ZclValue::Enum8(PANEL_STATUS_ARMED_NIGHT));
                        ARM_NOTIF_ONLY_NIGHT_ZONES_ARMED
                    }
                    ARM_MODE_ARM_ALL_ZONES => {
                        let _ = self
                            .store
                            .set_raw(ATTR_PANEL_STATUS, ZclValue::Enum8(PANEL_STATUS_ARMED_AWAY));
                        ARM_NOTIF_ALL_ZONES_ARMED
                    }
                    _ => ARM_NOTIF_INVALID_ARM_CODE,
                };
                // ArmResponse: command 0x00 server→client, payload = notification byte
                let mut resp = heapless::Vec::new();
                let _ = resp.push(notification);
                Ok(resp)
            }
            CMD_EMERGENCY => {
                let _ = self
                    .store
                    .set_raw(ATTR_PANEL_STATUS, ZclValue::Enum8(PANEL_STATUS_IN_ALARM));
                Ok(self.build_panel_status_payload(ALARM_STATUS_EMERGENCY))
            }
            CMD_FIRE => {
                let _ = self
                    .store
                    .set_raw(ATTR_PANEL_STATUS, ZclValue::Enum8(PANEL_STATUS_IN_ALARM));
                Ok(self.build_panel_status_payload(ALARM_STATUS_FIRE))
            }
            CMD_PANIC => {
                let _ = self
                    .store
                    .set_raw(ATTR_PANEL_STATUS, ZclValue::Enum8(PANEL_STATUS_IN_ALARM));
                Ok(self.build_panel_status_payload(ALARM_STATUS_POLICE_PANIC))
            }
            CMD_GET_PANEL_STATUS => Ok(self.build_panel_status_payload(ALARM_STATUS_NO_ALARM)),
            CMD_GET_ZONE_ID_MAP => {
                // Return 16 zero u16 values (no zones mapped)
                let mut resp = heapless::Vec::new();
                for _ in 0..16 {
                    let _ = resp.extend_from_slice(&0u16.to_le_bytes());
                }
                Ok(resp)
            }
            CMD_GET_ZONE_INFORMATION => {
                // Payload: zone_id(u8)
                let zone_id = if payload.is_empty() { 0u8 } else { payload[0] };
                let mut resp = heapless::Vec::new();
                let _ = resp.push(zone_id); // zone_id
                let _ = resp.extend_from_slice(&0u16.to_le_bytes()); // zone_type = 0
                let _ = resp.extend_from_slice(&[0u8; 8]); // IEEE address = 0
                // zone_label: ZCL string (length-prefixed), empty
                let _ = resp.push(0x00); // label length = 0
                Ok(resp)
            }
            CMD_BYPASS => {
                // Accept but no-op (no bypass table)
                Ok(heapless::Vec::new())
            }
            CMD_GET_BYPASSED_ZONE_LIST => {
                // Return empty bypassed zone list
                let mut resp = heapless::Vec::new();
                let _ = resp.push(0x00); // number of zones = 0
                Ok(resp)
            }
            CMD_GET_ZONE_STATUS => {
                // Return zone status response with empty zone list
                let mut resp = heapless::Vec::new();
                let _ = resp.push(0x01); // zone_status_complete = true
                let _ = resp.push(0x00); // number_of_zones = 0
                Ok(resp)
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::from_slice(&[
            CMD_ARM.0,
            CMD_BYPASS.0,
            CMD_EMERGENCY.0,
            CMD_FIRE.0,
            CMD_PANIC.0,
            CMD_GET_ZONE_ID_MAP.0,
            CMD_GET_ZONE_INFORMATION.0,
            CMD_GET_PANEL_STATUS.0,
            CMD_GET_BYPASSED_ZONE_LIST.0,
            CMD_GET_ZONE_STATUS.0,
        ])
        .unwrap_or_default()
    }

    fn generated_commands(&self) -> heapless::Vec<u8, 32> {
        heapless::Vec::from_slice(&[
            CMD_ARM_RESPONSE.0,
            CMD_GET_ZONE_ID_MAP_RESPONSE.0,
            CMD_GET_ZONE_INFORMATION_RESPONSE.0,
            CMD_ZONE_STATUS_CHANGED.0,
            CMD_PANEL_STATUS_CHANGED.0,
            CMD_GET_PANEL_STATUS_RESPONSE.0,
            CMD_SET_BYPASSED_ZONE_LIST.0,
            CMD_BYPASS_RESPONSE.0,
            CMD_GET_ZONE_STATUS_RESPONSE.0,
        ])
        .unwrap_or_default()
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }

    /// `PanelStatus` is driven exclusively by this cluster's own Arm/
    /// Emergency/Fire/Panic commands (no external hardware feed), so a
    /// Basic cluster reset returns it to the idle, disarmed state.
    fn reset_to_factory_defaults(&mut self) {
        let _ = self
            .store
            .set_raw(ATTR_PANEL_STATUS, ZclValue::Enum8(PANEL_STATUS_DISARMED));
    }
}
