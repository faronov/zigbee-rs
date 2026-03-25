//! Simple Metering cluster (0x0702).
//!
//! Smart Energy metering for electricity, gas, water, etc. Provides
//! cumulative summation counters, instantaneous demand readings, and
//! formatting/unit metadata. Read/report only — no cluster-specific commands.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_CURRENT_SUMMATION_DELIVERED: AttributeId = AttributeId(0x0000);
pub const ATTR_CURRENT_SUMMATION_RECEIVED: AttributeId = AttributeId(0x0001);
pub const ATTR_UNIT_OF_MEASURE: AttributeId = AttributeId(0x0300);
pub const ATTR_MULTIPLIER: AttributeId = AttributeId(0x0301);
pub const ATTR_DIVISOR: AttributeId = AttributeId(0x0302);
pub const ATTR_SUMMATION_FORMATTING: AttributeId = AttributeId(0x0303);
pub const ATTR_DEMAND_FORMATTING: AttributeId = AttributeId(0x0304);
pub const ATTR_METERING_DEVICE_TYPE: AttributeId = AttributeId(0x0308);
pub const ATTR_INSTANTANEOUS_DEMAND: AttributeId = AttributeId(0x0400);
pub const ATTR_POWER_FACTOR: AttributeId = AttributeId(0x0510);

// Unit of measure values
pub const UNIT_KWH: u8 = 0x00;
pub const UNIT_M3: u8 = 0x01;
pub const UNIT_FT3: u8 = 0x02;
pub const UNIT_CCF: u8 = 0x03;
pub const UNIT_US_GAL: u8 = 0x04;
pub const UNIT_IMP_GAL: u8 = 0x05;
pub const UNIT_BTU: u8 = 0x06;
pub const UNIT_LITERS: u8 = 0x07;
pub const UNIT_KPA: u8 = 0x08;

// Metering device type values
pub const DEVICE_TYPE_ELECTRIC: u8 = 0x00;
pub const DEVICE_TYPE_GAS: u8 = 0x01;
pub const DEVICE_TYPE_WATER: u8 = 0x02;

/// Simple Metering cluster implementation.
pub struct MeteringCluster {
    store: AttributeStore<10>,
}

impl MeteringCluster {
    pub fn new(unit: u8, multiplier: u32, divisor: u32) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_SUMMATION_DELIVERED,
                data_type: ZclDataType::U48,
                access: AttributeAccess::Reportable,
                name: "CurrentSummationDelivered",
            },
            ZclValue::U48(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_SUMMATION_RECEIVED,
                data_type: ZclDataType::U48,
                access: AttributeAccess::Reportable,
                name: "CurrentSummationReceived",
            },
            ZclValue::U48(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_UNIT_OF_MEASURE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "UnitOfMeasure",
            },
            ZclValue::Enum8(unit),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MULTIPLIER,
                data_type: ZclDataType::U24,
                access: AttributeAccess::ReadOnly,
                name: "Multiplier",
            },
            ZclValue::U24(multiplier),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DIVISOR,
                data_type: ZclDataType::U24,
                access: AttributeAccess::ReadOnly,
                name: "Divisor",
            },
            ZclValue::U24(divisor),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SUMMATION_FORMATTING,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadOnly,
                name: "SummationFormatting",
            },
            ZclValue::Bitmap8(0x00),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DEMAND_FORMATTING,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadOnly,
                name: "DemandFormatting",
            },
            ZclValue::Bitmap8(0x00),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_METERING_DEVICE_TYPE,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadOnly,
                name: "MeteringDeviceType",
            },
            ZclValue::Bitmap8(0x00),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_INSTANTANEOUS_DEMAND,
                data_type: ZclDataType::I32,
                access: AttributeAccess::Reportable,
                name: "InstantaneousDemand",
            },
            ZclValue::I32(0),
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
        Self { store }
    }

    /// Add energy to the delivered summation counter.
    pub fn add_energy_delivered(&mut self, wh: u64) {
        let current = match self.store.get(ATTR_CURRENT_SUMMATION_DELIVERED) {
            Some(ZclValue::U48(v)) => *v,
            _ => 0,
        };
        let _ = self.store.set_raw(
            ATTR_CURRENT_SUMMATION_DELIVERED,
            ZclValue::U48(current.saturating_add(wh)),
        );
    }

    /// Set the instantaneous demand in watts (signed).
    pub fn set_instantaneous_demand(&mut self, watts: i32) {
        let _ = self
            .store
            .set_raw(ATTR_INSTANTANEOUS_DEMAND, ZclValue::I32(watts));
    }

    /// Get the total energy delivered counter.
    pub fn get_total_delivered(&self) -> u64 {
        match self.store.get(ATTR_CURRENT_SUMMATION_DELIVERED) {
            Some(ZclValue::U48(v)) => *v,
            _ => 0,
        }
    }
}

impl Cluster for MeteringCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::METERING
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
