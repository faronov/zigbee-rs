//! Touchlink Commissioning cluster (0x1000).
//!
//! Implements proximity-based commissioning via inter-PAN frames.
//! An initiator scans for nearby targets, identifies them, and can form
//! or join a network without traditional Trust-Centre commissioning.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// ---------------------------------------------------------------------------
// Attribute IDs (synthetic — Touchlink is mostly command-driven)
// ---------------------------------------------------------------------------
pub const ATTR_TOUCHLINK_STATE: AttributeId = AttributeId(0xFF00);

// ---------------------------------------------------------------------------
// Initiator → Target commands
// ---------------------------------------------------------------------------
pub const CMD_SCAN_REQUEST: CommandId = CommandId(0x00);
pub const CMD_DEVICE_INFORMATION_REQUEST: CommandId = CommandId(0x02);
pub const CMD_IDENTIFY_REQUEST: CommandId = CommandId(0x06);
pub const CMD_RESET_TO_FACTORY_NEW_REQUEST: CommandId = CommandId(0x07);
pub const CMD_NETWORK_START_REQUEST: CommandId = CommandId(0x10);
pub const CMD_NETWORK_JOIN_ROUTER_REQUEST: CommandId = CommandId(0x12);
pub const CMD_NETWORK_JOIN_END_DEVICE_REQUEST: CommandId = CommandId(0x14);
pub const CMD_NETWORK_UPDATE_REQUEST: CommandId = CommandId(0x16);

// ---------------------------------------------------------------------------
// Target → Initiator commands
// ---------------------------------------------------------------------------
pub const CMD_SCAN_RESPONSE: CommandId = CommandId(0x01);
pub const CMD_DEVICE_INFORMATION_RESPONSE: CommandId = CommandId(0x03);
pub const CMD_NETWORK_START_RESPONSE: CommandId = CommandId(0x11);
pub const CMD_NETWORK_JOIN_ROUTER_RESPONSE: CommandId = CommandId(0x13);
pub const CMD_NETWORK_JOIN_END_DEVICE_RESPONSE: CommandId = CommandId(0x15);

// ---------------------------------------------------------------------------
// Touchlink information bitmap bits
// ---------------------------------------------------------------------------
pub const TL_INFO_FACTORY_NEW: u8 = 0x01;
pub const TL_INFO_ADDRESS_ASSIGNMENT: u8 = 0x02;
pub const TL_INFO_LINK_INITIATOR: u8 = 0x10;
pub const TL_INFO_TOUCHLINK_PRIORITY_REQUEST: u8 = 0x20;

// ---------------------------------------------------------------------------
// RSSI threshold for proximity detection
// ---------------------------------------------------------------------------
pub const TOUCHLINK_RSSI_THRESHOLD: i8 = -70;

// ---------------------------------------------------------------------------
// Touchlink state values for ATTR_TOUCHLINK_STATE
// ---------------------------------------------------------------------------
pub const STATE_IDLE: u8 = 0x00;
pub const STATE_SCANNING: u8 = 0x01;
pub const STATE_IDENTIFYING: u8 = 0x02;
pub const STATE_COMMISSIONING: u8 = 0x03;

// ---------------------------------------------------------------------------
// Cluster struct
// ---------------------------------------------------------------------------

/// Touchlink Commissioning cluster implementation.
pub struct TouchlinkCluster {
    store: AttributeStore<4>,
}

impl Default for TouchlinkCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl TouchlinkCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_TOUCHLINK_STATE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "TouchlinkState",
            },
            ZclValue::Enum8(STATE_IDLE),
        );
        Self { store }
    }
}

impl Cluster for TouchlinkCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x1000)
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_SCAN_REQUEST => {
                // inter_pan_transaction_id (u32) + zigbee_information (u8) + touchlink_information (u8)
                if payload.len() < 6 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let _inter_pan_transaction_id =
                    u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                let _zigbee_information = payload[4];
                let _touchlink_information = payload[5];
                let _ = self
                    .store
                    .set_raw(ATTR_TOUCHLINK_STATE, ZclValue::Enum8(STATE_SCANNING));
                Ok(heapless::Vec::new())
            }
            CMD_IDENTIFY_REQUEST => {
                // inter_pan_transaction_id (u32) + identify_duration (u16)
                if payload.len() < 6 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let _inter_pan_transaction_id =
                    u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                let _identify_duration = u16::from_le_bytes([payload[4], payload[5]]);
                let _ = self
                    .store
                    .set_raw(ATTR_TOUCHLINK_STATE, ZclValue::Enum8(STATE_IDENTIFYING));
                Ok(heapless::Vec::new())
            }
            // Remaining commands require inter-PAN transport not available through
            // normal ZCL cluster command processing.
            CMD_DEVICE_INFORMATION_REQUEST
            | CMD_RESET_TO_FACTORY_NEW_REQUEST
            | CMD_NETWORK_START_REQUEST
            | CMD_NETWORK_JOIN_ROUTER_REQUEST
            | CMD_NETWORK_JOIN_END_DEVICE_REQUEST
            | CMD_NETWORK_UPDATE_REQUEST => Err(ZclStatus::UnsupClusterCommand),
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
