//! Device Temperature Configuration cluster (0x0002).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

pub const ATTR_CURRENT_TEMPERATURE: AttributeId = AttributeId(0x0000);
pub const ATTR_MIN_TEMP_EXPERIENCED: AttributeId = AttributeId(0x0001);
pub const ATTR_MAX_TEMP_EXPERIENCED: AttributeId = AttributeId(0x0002);
pub const ATTR_OVER_TEMP_TOTAL_DWELL: AttributeId = AttributeId(0x0003);
pub const ATTR_DEVICE_TEMP_ALARM_MASK: AttributeId = AttributeId(0x0010);
pub const ATTR_LOW_TEMP_THRESHOLD: AttributeId = AttributeId(0x0011);
pub const ATTR_HIGH_TEMP_THRESHOLD: AttributeId = AttributeId(0x0012);
pub const ATTR_LOW_TEMP_DWELL_TRIP_POINT: AttributeId = AttributeId(0x0013);
pub const ATTR_HIGH_TEMP_DWELL_TRIP_POINT: AttributeId = AttributeId(0x0014);

/// Device Temperature Configuration cluster.
pub struct DeviceTempConfigCluster {
    store: AttributeStore<9>,
}

impl Default for DeviceTempConfigCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceTempConfigCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_TEMPERATURE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "CurrentTemperature",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_TEMP_EXPERIENCED,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "MinTempExperienced",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_TEMP_EXPERIENCED,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "MaxTempExperienced",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_OVER_TEMP_TOTAL_DWELL,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "OverTempTotalDwell",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DEVICE_TEMP_ALARM_MASK,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "DeviceTempAlarmMask",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LOW_TEMP_THRESHOLD,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "LowTempThreshold",
            },
            ZclValue::I16(-40),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_HIGH_TEMP_THRESHOLD,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "HighTempThreshold",
            },
            ZclValue::I16(85),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LOW_TEMP_DWELL_TRIP_POINT,
                data_type: ZclDataType::U24,
                access: AttributeAccess::ReadWrite,
                name: "LowTempDwellTripPoint",
            },
            ZclValue::U32(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_HIGH_TEMP_DWELL_TRIP_POINT,
                data_type: ZclDataType::U24,
                access: AttributeAccess::ReadWrite,
                name: "HighTempDwellTripPoint",
            },
            ZclValue::U32(1),
        );
        Self { store }
    }

    /// Update the device temperature (in degrees Celsius).
    pub fn set_temperature(&mut self, temp_c: i16) {
        let _ = self
            .store
            .set_raw(ATTR_CURRENT_TEMPERATURE, ZclValue::I16(temp_c));
    }
}

impl Cluster for DeviceTempConfigCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::DEVICE_TEMP_CONFIG
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
