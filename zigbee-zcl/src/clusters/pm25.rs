//! PM2.5 Measurement cluster (0x042A).
//!
//! Reports particulate matter (≤2.5µm) concentration in µg/m³.
//! Used by air quality sensors.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

pub const ATTR_MEASURED_VALUE: AttributeId = AttributeId(0x0000);
pub const ATTR_MIN_MEASURED_VALUE: AttributeId = AttributeId(0x0001);
pub const ATTR_MAX_MEASURED_VALUE: AttributeId = AttributeId(0x0002);
pub const ATTR_TOLERANCE: AttributeId = AttributeId(0x0003);

/// PM2.5 Measurement cluster.
pub struct Pm25Cluster {
    store: AttributeStore<4>,
}

impl Default for Pm25Cluster {
    fn default() -> Self {
        Self::new()
    }
}

impl Pm25Cluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MEASURED_VALUE,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::Reportable,
                name: "MeasuredValue",
            },
            ZclValue::Float32(0.0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_MEASURED_VALUE,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::ReadOnly,
                name: "MinMeasuredValue",
            },
            ZclValue::Float32(0.0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_MEASURED_VALUE,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::ReadOnly,
                name: "MaxMeasuredValue",
            },
            ZclValue::Float32(999.0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_TOLERANCE,
                data_type: ZclDataType::Float32,
                access: AttributeAccess::ReadOnly,
                name: "Tolerance",
            },
            ZclValue::Float32(0.0),
        );
        Self { store }
    }

    /// Set the PM2.5 concentration in µg/m³.
    pub fn set_pm25(&mut self, ug_per_m3: f32) {
        let _ = self
            .store
            .set_raw(ATTR_MEASURED_VALUE, ZclValue::Float32(ug_per_m3));
    }
}

impl Cluster for Pm25Cluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::PM25_MEASUREMENT
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
