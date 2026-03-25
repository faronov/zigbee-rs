//! Fan Control cluster (0x0202).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_FAN_MODE: AttributeId = AttributeId(0x0000);
pub const ATTR_FAN_MODE_SEQUENCE: AttributeId = AttributeId(0x0001);

// FanMode values
pub const FAN_MODE_OFF: u8 = 0x00;
pub const FAN_MODE_LOW: u8 = 0x01;
pub const FAN_MODE_MEDIUM: u8 = 0x02;
pub const FAN_MODE_HIGH: u8 = 0x03;
pub const FAN_MODE_ON: u8 = 0x04;
pub const FAN_MODE_AUTO: u8 = 0x05;
pub const FAN_MODE_SMART: u8 = 0x06;

// FanModeSequence values
pub const FAN_SEQ_LOW_MED_HIGH: u8 = 0x00;
pub const FAN_SEQ_LOW_HIGH: u8 = 0x01;
pub const FAN_SEQ_LOW_MED_HIGH_AUTO: u8 = 0x02;
pub const FAN_SEQ_LOW_HIGH_AUTO: u8 = 0x03;
pub const FAN_SEQ_ON_AUTO: u8 = 0x04;

/// Fan Control cluster.
pub struct FanControlCluster {
    store: AttributeStore<2>,
}

impl Default for FanControlCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl FanControlCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_FAN_MODE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "FanMode",
            },
            ZclValue::Enum8(FAN_MODE_AUTO),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_FAN_MODE_SEQUENCE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "FanModeSequence",
            },
            ZclValue::Enum8(FAN_SEQ_LOW_MED_HIGH_AUTO),
        );
        Self { store }
    }

    /// Get the current fan mode.
    pub fn fan_mode(&self) -> u8 {
        match self.store.get(ATTR_FAN_MODE) {
            Some(ZclValue::Enum8(v)) => *v,
            _ => FAN_MODE_AUTO,
        }
    }

    /// Set the fan mode.
    pub fn set_fan_mode(&mut self, mode: u8) {
        let _ = self.store.set_raw(ATTR_FAN_MODE, ZclValue::Enum8(mode));
    }
}

impl Cluster for FanControlCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0202)
    }

    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        Err(ZclStatus::UnsupClusterCommand)
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
