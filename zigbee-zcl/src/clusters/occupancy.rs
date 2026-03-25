//! Occupancy Sensing cluster (0x0406).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_OCCUPANCY: AttributeId = AttributeId(0x0000);
pub const ATTR_OCCUPANCY_SENSOR_TYPE: AttributeId = AttributeId(0x0001);
pub const ATTR_OCCUPANCY_SENSOR_TYPE_BITMAP: AttributeId = AttributeId(0x0002);
pub const ATTR_PIR_O_TO_U_DELAY: AttributeId = AttributeId(0x0010);
pub const ATTR_PIR_U_TO_O_DELAY: AttributeId = AttributeId(0x0011);
pub const ATTR_PIR_U_TO_O_THRESHOLD: AttributeId = AttributeId(0x0012);
pub const ATTR_ULTRASONIC_O_TO_U_DELAY: AttributeId = AttributeId(0x0020);
pub const ATTR_ULTRASONIC_U_TO_O_DELAY: AttributeId = AttributeId(0x0021);

// Sensor type values
pub const SENSOR_TYPE_PIR: u8 = 0x00;
pub const SENSOR_TYPE_ULTRASONIC: u8 = 0x01;
pub const SENSOR_TYPE_PIR_AND_ULTRASONIC: u8 = 0x02;
pub const SENSOR_TYPE_PHYSICAL_CONTACT: u8 = 0x03;

/// Occupancy Sensing cluster.
pub struct OccupancyCluster {
    store: AttributeStore<10>,
}

impl OccupancyCluster {
    pub fn new(sensor_type: u8) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPANCY,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::Reportable,
                name: "Occupancy",
            },
            ZclValue::Bitmap8(0x00),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPANCY_SENSOR_TYPE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "OccupancySensorType",
            },
            ZclValue::Enum8(sensor_type),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OCCUPANCY_SENSOR_TYPE_BITMAP,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadOnly,
                name: "OccupancySensorTypeBitmap",
            },
            ZclValue::Bitmap8(1 << sensor_type),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_PIR_O_TO_U_DELAY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "PIROccupiedToUnoccupiedDelay",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_PIR_U_TO_O_DELAY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "PIRUnoccupiedToOccupiedDelay",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_PIR_U_TO_O_THRESHOLD,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "PIRUnoccupiedToOccupiedThreshold",
            },
            ZclValue::U8(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ULTRASONIC_O_TO_U_DELAY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "UltrasonicOccupiedToUnoccupiedDelay",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ULTRASONIC_U_TO_O_DELAY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "UltrasonicUnoccupiedToOccupiedDelay",
            },
            ZclValue::U16(0),
        );
        Self { store }
    }

    /// Set the occupancy state (bit 0 = occupied).
    pub fn set_occupied(&mut self, occupied: bool) {
        let _ = self.store.set_raw(
            ATTR_OCCUPANCY,
            ZclValue::Bitmap8(if occupied { 1 } else { 0 }),
        );
    }
}

impl Cluster for OccupancyCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::OCCUPANCY
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
