//! Carbon Dioxide (CO₂) Measurement cluster (0x040D).
//!
//! Reports CO₂ concentration as a fraction of one, as required by ZCL.
//! [`CarbonDioxideCluster::set_co2_ppm`] accepts the application-friendly
//! parts-per-million representation and performs the wire-unit conversion.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

pub const ATTR_MEASURED_VALUE: AttributeId = AttributeId(0x0000);
pub const ATTR_MIN_MEASURED_VALUE: AttributeId = AttributeId(0x0001);
pub const ATTR_MAX_MEASURED_VALUE: AttributeId = AttributeId(0x0002);
pub const ATTR_TOLERANCE: AttributeId = AttributeId(0x0003);

const PPM_TO_FRACTION: f32 = 1.0e-6;

/// Carbon Dioxide Measurement cluster.
pub struct CarbonDioxideCluster {
    store: AttributeStore<4>,
}

impl Default for CarbonDioxideCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl CarbonDioxideCluster {
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
            ZclValue::Float32(10_000.0 * PPM_TO_FRACTION),
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

    /// Set the CO₂ concentration in ppm.
    pub fn set_co2_ppm(&mut self, ppm: f32) {
        let _ = self.store.set_raw(
            ATTR_MEASURED_VALUE,
            ZclValue::Float32(ppm * PPM_TO_FRACTION),
        );
    }
}

impl Cluster for CarbonDioxideCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::CARBON_DIOXIDE
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

    /// No writable attributes: `MeasuredValue` is a live sensor reading fed
    /// by the driver, and `MinMeasuredValue`/`MaxMeasuredValue`/`Tolerance`
    /// are fixed physical-range configuration supplied at construction.
    fn reset_to_factory_defaults(&mut self) {}
}
