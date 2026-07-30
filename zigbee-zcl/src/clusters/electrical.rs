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

/// Scaling applied by clients to the raw AC measurement attributes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AcScaling {
    pub voltage_multiplier: u16,
    pub voltage_divisor: u16,
    pub current_multiplier: u16,
    pub current_divisor: u16,
    pub power_multiplier: u16,
    pub power_divisor: u16,
}

impl Default for AcScaling {
    fn default() -> Self {
        Self {
            voltage_multiplier: 1,
            voltage_divisor: 1,
            current_multiplier: 1,
            current_divisor: 1,
            power_multiplier: 1,
            power_divisor: 1,
        }
    }
}

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

    /// Configure the multiplier/divisor pairs used to interpret AC readings.
    ///
    /// Divisors must be non-zero. A common smart-plug representation is
    /// decivolts (`1/10`), milliamps (`1/1000`), and whole watts (`1/1`).
    pub fn set_ac_scaling(&mut self, scaling: AcScaling) -> Result<(), ZclStatus> {
        if scaling.voltage_divisor == 0
            || scaling.current_divisor == 0
            || scaling.power_divisor == 0
        {
            return Err(ZclStatus::InvalidValue);
        }

        let values = [
            (ATTR_AC_VOLTAGE_MULTIPLIER, scaling.voltage_multiplier),
            (ATTR_AC_VOLTAGE_DIVISOR, scaling.voltage_divisor),
            (ATTR_AC_CURRENT_MULTIPLIER, scaling.current_multiplier),
            (ATTR_AC_CURRENT_DIVISOR, scaling.current_divisor),
            (ATTR_AC_POWER_MULTIPLIER, scaling.power_multiplier),
            (ATTR_AC_POWER_DIVISOR, scaling.power_divisor),
        ];
        for (attribute, value) in values {
            let _ = self.store.set_raw(attribute, ZclValue::U16(value));
        }
        Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ac_scaling_updates_all_multiplier_divisor_pairs() {
        let mut cluster = ElectricalMeasurementCluster::new();
        cluster
            .set_ac_scaling(AcScaling {
                voltage_multiplier: 1,
                voltage_divisor: 10,
                current_multiplier: 1,
                current_divisor: 1_000,
                power_multiplier: 1,
                power_divisor: 1,
            })
            .unwrap();

        assert_eq!(
            cluster.attributes().get(ATTR_AC_VOLTAGE_DIVISOR),
            Some(&ZclValue::U16(10))
        );
        assert_eq!(
            cluster.attributes().get(ATTR_AC_CURRENT_DIVISOR),
            Some(&ZclValue::U16(1_000))
        );
        assert_eq!(
            cluster.attributes().get(ATTR_AC_POWER_MULTIPLIER),
            Some(&ZclValue::U16(1))
        );
    }

    #[test]
    fn ac_scaling_rejects_zero_divisors_without_partial_update() {
        let mut cluster = ElectricalMeasurementCluster::new();
        assert_eq!(
            cluster.set_ac_scaling(AcScaling {
                voltage_divisor: 10,
                current_divisor: 0,
                ..AcScaling::default()
            }),
            Err(ZclStatus::InvalidValue)
        );
        assert_eq!(
            cluster.attributes().get(ATTR_AC_VOLTAGE_DIVISOR),
            Some(&ZclValue::U16(1))
        );
    }
}
