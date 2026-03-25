//! Electrical Measurement cluster (0x0B04).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_MEASUREMENT_TYPE: AttributeId = AttributeId(0x0000);
pub const ATTR_DC_VOLTAGE: AttributeId = AttributeId(0x0100);
pub const ATTR_DC_CURRENT: AttributeId = AttributeId(0x0103);
pub const ATTR_DC_POWER: AttributeId = AttributeId(0x0106);
pub const ATTR_AC_FREQUENCY: AttributeId = AttributeId(0x0300);
pub const ATTR_RMS_VOLTAGE: AttributeId = AttributeId(0x0505);
pub const ATTR_RMS_CURRENT: AttributeId = AttributeId(0x0508);
pub const ATTR_ACTIVE_POWER: AttributeId = AttributeId(0x050B);
pub const ATTR_REACTIVE_POWER: AttributeId = AttributeId(0x050E);
pub const ATTR_APPARENT_POWER: AttributeId = AttributeId(0x050F);
pub const ATTR_POWER_FACTOR: AttributeId = AttributeId(0x0510);
pub const ATTR_AC_VOLTAGE_MULTIPLIER: AttributeId = AttributeId(0x0600);
pub const ATTR_AC_VOLTAGE_DIVISOR: AttributeId = AttributeId(0x0601);
pub const ATTR_AC_CURRENT_MULTIPLIER: AttributeId = AttributeId(0x0602);
pub const ATTR_AC_CURRENT_DIVISOR: AttributeId = AttributeId(0x0603);
pub const ATTR_AC_POWER_MULTIPLIER: AttributeId = AttributeId(0x0604);
pub const ATTR_AC_POWER_DIVISOR: AttributeId = AttributeId(0x0605);

/// Electrical Measurement cluster.
pub struct ElectricalMeasurementCluster {
    store: AttributeStore<20>,
}

impl Default for ElectricalMeasurementCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl ElectricalMeasurementCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MEASUREMENT_TYPE,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "MeasurementType",
            },
            ZclValue::U32(0x00000008),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DC_VOLTAGE,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "DCVoltage",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DC_CURRENT,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "DCCurrent",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DC_POWER,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "DCPower",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_AC_FREQUENCY,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ACFrequency",
            },
            ZclValue::U16(50),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_RMS_VOLTAGE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "RMSVoltage",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_RMS_CURRENT,
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "RMSCurrent",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_ACTIVE_POWER,
                data_type: ZclDataType::I16,
                access: AttributeAccess::Reportable,
                name: "ActivePower",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_REACTIVE_POWER,
                data_type: ZclDataType::I16,
                access: AttributeAccess::ReadOnly,
                name: "ReactivePower",
            },
            ZclValue::I16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_APPARENT_POWER,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ApparentPower",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_POWER_FACTOR,
                data_type: ZclDataType::I8,
                access: AttributeAccess::ReadOnly,
                name: "PowerFactor",
            },
            ZclValue::I8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_AC_VOLTAGE_MULTIPLIER,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ACVoltageMultiplier",
            },
            ZclValue::U16(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_AC_VOLTAGE_DIVISOR,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ACVoltageDivisor",
            },
            ZclValue::U16(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_AC_CURRENT_MULTIPLIER,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ACCurrentMultiplier",
            },
            ZclValue::U16(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_AC_CURRENT_DIVISOR,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ACCurrentDivisor",
            },
            ZclValue::U16(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_AC_POWER_MULTIPLIER,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ACPowerMultiplier",
            },
            ZclValue::U16(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_AC_POWER_DIVISOR,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ACPowerDivisor",
            },
            ZclValue::U16(1),
        );
        Self { store }
    }

    /// Update electrical measurements.
    pub fn set_measurements(&mut self, voltage: u16, current: u16, power: i16) {
        let _ = self.store.set_raw(ATTR_RMS_VOLTAGE, ZclValue::U16(voltage));
        let _ = self.store.set_raw(ATTR_RMS_CURRENT, ZclValue::U16(current));
        let _ = self.store.set_raw(ATTR_ACTIVE_POWER, ZclValue::I16(power));
    }
}

impl Cluster for ElectricalMeasurementCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::ELECTRICAL_MEASUREMENT
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
