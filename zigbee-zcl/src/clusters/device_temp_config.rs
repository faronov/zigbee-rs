//! Device Temperature Configuration cluster (0x0002).
//!
//! The ZCL specification defines `CurrentTemperature` (0x0000) in whole
//! degrees Celsius. ZHA exposes this cluster as a diagnostic temperature
//! entity with a divisor of 100, so this implementation intentionally stores
//! current/min/max values and thresholds in centi-degrees for ZHA
//! interoperability (2500 = 25.00 °C).

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
pub const TEMPERATURE_UNAVAILABLE: i16 = i16::MIN;

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
            ZclValue::I16(TEMPERATURE_UNAVAILABLE),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_TEMP_EXPERIENCED,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "MinTempExperienced",
            },
            ZclValue::I16(TEMPERATURE_UNAVAILABLE),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_TEMP_EXPERIENCED,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "MaxTempExperienced",
            },
            ZclValue::I16(TEMPERATURE_UNAVAILABLE),
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
            ZclValue::I16(-4000),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_HIGH_TEMP_THRESHOLD,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadWrite,
                name: "HighTempThreshold",
            },
            ZclValue::I16(8500),
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

    /// Update the current device temperature in **centi-degrees Celsius**
    /// (e.g., pass 2500 for 25.00 °C).  Also updates `MinTempExperienced`
    /// and `MaxTempExperienced` when the new value extends the observed range.
    pub fn set_temperature(&mut self, temp_centi_c: i16) {
        let initialized = !matches!(
            self.store.get(ATTR_CURRENT_TEMPERATURE),
            Some(ZclValue::I16(TEMPERATURE_UNAVAILABLE))
        );

        let _ = self
            .store
            .set_raw(ATTR_CURRENT_TEMPERATURE, ZclValue::I16(temp_centi_c));

        if !initialized {
            let _ = self
                .store
                .set_raw(ATTR_MIN_TEMP_EXPERIENCED, ZclValue::I16(temp_centi_c));
            let _ = self
                .store
                .set_raw(ATTR_MAX_TEMP_EXPERIENCED, ZclValue::I16(temp_centi_c));
            return;
        }

        let current_min = match self.store.get(ATTR_MIN_TEMP_EXPERIENCED) {
            Some(ZclValue::I16(v)) => *v,
            _ => i16::MAX,
        };
        if temp_centi_c < current_min {
            let _ = self
                .store
                .set_raw(ATTR_MIN_TEMP_EXPERIENCED, ZclValue::I16(temp_centi_c));
        }

        let current_max = match self.store.get(ATTR_MAX_TEMP_EXPERIENCED) {
            Some(ZclValue::I16(v)) => *v,
            _ => i16::MIN,
        };
        if temp_centi_c > current_max {
            let _ = self
                .store
                .set_raw(ATTR_MAX_TEMP_EXPERIENCED, ZclValue::I16(temp_centi_c));
        }
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
