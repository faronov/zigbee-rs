//! Illuminance Level Sensing cluster (0x0401).
//!
//! Provides threshold-based illuminance level (dark/bright/etc.) as opposed to
//! the raw lux value from the Illuminance Measurement cluster (0x0400).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

pub const ATTR_LEVEL_STATUS: AttributeId = AttributeId(0x0000);
pub const ATTR_LIGHT_SENSOR_TYPE: AttributeId = AttributeId(0x0001);
pub const ATTR_ILLUMINANCE_TARGET_LEVEL: AttributeId = AttributeId(0x0010);

/// Level status values.
pub const LEVEL_ON_TARGET: u8 = 0x00;
pub const LEVEL_BELOW_TARGET: u8 = 0x01;
pub const LEVEL_ABOVE_TARGET: u8 = 0x02;

/// Illuminance Level Sensing cluster.
pub struct IlluminanceLevelCluster {
    store: AttributeStore<3>,
}

impl Default for IlluminanceLevelCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl IlluminanceLevelCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LEVEL_STATUS,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "LevelStatus",
            },
            ZclValue::U8(LEVEL_ON_TARGET),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LIGHT_SENSOR_TYPE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "LightSensorType",
            },
            ZclValue::U8(0xFF), // unknown
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ILLUMINANCE_TARGET_LEVEL,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "IlluminanceTargetLevel",
            },
            ZclValue::U16(0),
        );
        Self { store }
    }

    /// Set the level status.
    pub fn set_level_status(&mut self, status: u8) {
        let _ = self
            .store
            .set_raw(ATTR_LEVEL_STATUS, ZclValue::U8(status));
    }
}

impl Cluster for IlluminanceLevelCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::ILLUMINANCE_LEVEL_SENSING
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
