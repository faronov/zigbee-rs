//! Window Covering cluster (0x0102).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_WINDOW_COVERING_TYPE: AttributeId = AttributeId(0x0000);
pub const ATTR_CONFIG_STATUS: AttributeId = AttributeId(0x0007);
pub const ATTR_CURRENT_POSITION_LIFT_PERCENTAGE: AttributeId = AttributeId(0x0008);
pub const ATTR_CURRENT_POSITION_TILT_PERCENTAGE: AttributeId = AttributeId(0x0009);
pub const ATTR_INSTALLED_OPEN_LIMIT_LIFT: AttributeId = AttributeId(0x0010);
pub const ATTR_INSTALLED_CLOSED_LIMIT_LIFT: AttributeId = AttributeId(0x0011);
pub const ATTR_INSTALLED_OPEN_LIMIT_TILT: AttributeId = AttributeId(0x0012);
pub const ATTR_INSTALLED_CLOSED_LIMIT_TILT: AttributeId = AttributeId(0x0013);
pub const ATTR_MODE: AttributeId = AttributeId(0x0017);

// Command IDs (client to server)
pub const CMD_UP_OPEN: CommandId = CommandId(0x00);
pub const CMD_DOWN_CLOSE: CommandId = CommandId(0x01);
pub const CMD_STOP: CommandId = CommandId(0x02);
pub const CMD_GO_TO_LIFT_VALUE: CommandId = CommandId(0x04);
pub const CMD_GO_TO_LIFT_PERCENTAGE: CommandId = CommandId(0x05);
pub const CMD_GO_TO_TILT_VALUE: CommandId = CommandId(0x07);
pub const CMD_GO_TO_TILT_PERCENTAGE: CommandId = CommandId(0x08);

// WindowCoveringType values
pub const COVERING_ROLLERSHADE: u8 = 0x00;
pub const COVERING_ROLLERSHADE_2_MOTOR: u8 = 0x01;
pub const COVERING_ROLLERSHADE_EXTERIOR: u8 = 0x02;
pub const COVERING_ROLLERSHADE_EXTERIOR_2_MOTOR: u8 = 0x03;
pub const COVERING_DRAPERY: u8 = 0x04;
pub const COVERING_AWNING: u8 = 0x05;
pub const COVERING_SHUTTER: u8 = 0x06;
pub const COVERING_TILT_BLIND_TILT_ONLY: u8 = 0x07;
pub const COVERING_TILT_BLIND_LIFT_AND_TILT: u8 = 0x08;
pub const COVERING_PROJECTOR_SCREEN: u8 = 0x09;

/// Window Covering cluster.
pub struct WindowCoveringCluster {
    store: AttributeStore<9>,
}

impl WindowCoveringCluster {
    pub fn new(covering_type: u8) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_WINDOW_COVERING_TYPE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "WindowCoveringType",
            },
            ZclValue::Enum8(covering_type),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CONFIG_STATUS,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadOnly,
                name: "ConfigStatus",
            },
            ZclValue::Bitmap8(0x03),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_POSITION_LIFT_PERCENTAGE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "CurrentPositionLiftPercentage",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_POSITION_TILT_PERCENTAGE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "CurrentPositionTiltPercentage",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_INSTALLED_OPEN_LIMIT_LIFT,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "InstalledOpenLimitLift",
            },
            ZclValue::U16(0x0000),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_INSTALLED_CLOSED_LIMIT_LIFT,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "InstalledClosedLimitLift",
            },
            ZclValue::U16(0xFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_INSTALLED_OPEN_LIMIT_TILT,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "InstalledOpenLimitTilt",
            },
            ZclValue::U16(0x0000),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_INSTALLED_CLOSED_LIMIT_TILT,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "InstalledClosedLimitTilt",
            },
            ZclValue::U16(0xFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MODE,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadWrite,
                name: "Mode",
            },
            ZclValue::Bitmap8(0x00),
        );
        Self { store }
    }

    /// Get current lift position percentage (0 = fully open, 100 = fully closed).
    pub fn lift_percentage(&self) -> u8 {
        match self.store.get(ATTR_CURRENT_POSITION_LIFT_PERCENTAGE) {
            Some(ZclValue::U8(v)) => *v,
            _ => 0,
        }
    }

    /// Get current tilt position percentage.
    pub fn tilt_percentage(&self) -> u8 {
        match self.store.get(ATTR_CURRENT_POSITION_TILT_PERCENTAGE) {
            Some(ZclValue::U8(v)) => *v,
            _ => 0,
        }
    }

    fn set_lift_percentage(&mut self, pct: u8) {
        let _ = self
            .store
            .set_raw(ATTR_CURRENT_POSITION_LIFT_PERCENTAGE, ZclValue::U8(pct));
    }

    fn set_tilt_percentage(&mut self, pct: u8) {
        let _ = self
            .store
            .set_raw(ATTR_CURRENT_POSITION_TILT_PERCENTAGE, ZclValue::U8(pct));
    }
}

impl Cluster for WindowCoveringCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::WINDOW_COVERING
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_UP_OPEN => {
                self.set_lift_percentage(0);
                Ok(heapless::Vec::new())
            }
            CMD_DOWN_CLOSE => {
                self.set_lift_percentage(100);
                Ok(heapless::Vec::new())
            }
            CMD_STOP => {
                // No-op: covering stops at current position.
                Ok(heapless::Vec::new())
            }
            CMD_GO_TO_LIFT_VALUE => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let _lift_value = u16::from_le_bytes([payload[0], payload[1]]);
                // Application layer should translate absolute value to percentage.
                Ok(heapless::Vec::new())
            }
            CMD_GO_TO_LIFT_PERCENTAGE => {
                if payload.is_empty() {
                    return Err(ZclStatus::MalformedCommand);
                }
                let pct = payload[0];
                if pct > 100 {
                    return Err(ZclStatus::InvalidValue);
                }
                self.set_lift_percentage(pct);
                Ok(heapless::Vec::new())
            }
            CMD_GO_TO_TILT_VALUE => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let _tilt_value = u16::from_le_bytes([payload[0], payload[1]]);
                Ok(heapless::Vec::new())
            }
            CMD_GO_TO_TILT_PERCENTAGE => {
                if payload.is_empty() {
                    return Err(ZclStatus::MalformedCommand);
                }
                let pct = payload[0];
                if pct > 100 {
                    return Err(ZclStatus::InvalidValue);
                }
                self.set_tilt_percentage(pct);
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
