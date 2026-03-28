//! Power Configuration cluster (0x0001).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs (battery information set)
pub const ATTR_BATTERY_VOLTAGE: AttributeId = AttributeId(0x0020);
pub const ATTR_BATTERY_PERCENTAGE_REMAINING: AttributeId = AttributeId(0x0021);
pub const ATTR_BATTERY_ALARM_MASK: AttributeId = AttributeId(0x0035);
pub const ATTR_BATTERY_VOLTAGE_MIN_THRESHOLD: AttributeId = AttributeId(0x0036);
pub const ATTR_BATTERY_SIZE: AttributeId = AttributeId(0x0031);
pub const ATTR_BATTERY_QUANTITY: AttributeId = AttributeId(0x0033);
pub const ATTR_BATTERY_RATED_VOLTAGE: AttributeId = AttributeId(0x0034);
pub const ATTR_BATTERY_ALARM_STATE: AttributeId = AttributeId(0x003E);

/// Power Configuration cluster (battery subset).
pub struct PowerConfigCluster {
    store: AttributeStore<12>,
}

impl Default for PowerConfigCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl PowerConfigCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BATTERY_VOLTAGE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "BatteryVoltage",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BATTERY_PERCENTAGE_REMAINING,
                data_type: ZclDataType::U8,
                access: AttributeAccess::Reportable,
                name: "BatteryPercentageRemaining",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BATTERY_SIZE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "BatterySize",
            },
            ZclValue::Enum8(0xFF), // Unknown
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BATTERY_QUANTITY,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "BatteryQuantity",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BATTERY_RATED_VOLTAGE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "BatteryRatedVoltage",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BATTERY_ALARM_MASK,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadWrite,
                name: "BatteryAlarmMask",
            },
            ZclValue::Bitmap8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BATTERY_VOLTAGE_MIN_THRESHOLD,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "BatteryVoltageMinThreshold",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BATTERY_ALARM_STATE,
                data_type: ZclDataType::Bitmap32,
                access: AttributeAccess::ReadOnly,
                name: "BatteryAlarmState",
            },
            ZclValue::Bitmap32(0),
        );
        Self { store }
    }

    /// Update battery voltage (in 100 mV units, e.g. 33 = 3.3 V).
    pub fn set_battery_voltage(&mut self, voltage_100mv: u8) {
        let _ = self
            .store
            .set_raw(ATTR_BATTERY_VOLTAGE, ZclValue::U8(voltage_100mv));
    }

    /// Update battery percentage (in 0.5% units, e.g. 200 = 100%).
    pub fn set_battery_percentage(&mut self, half_percent: u8) {
        let _ = self.store.set_raw(
            ATTR_BATTERY_PERCENTAGE_REMAINING,
            ZclValue::U8(half_percent),
        );
    }

    /// Set battery size (ZCL Enum8: 0=NoBattery, 1=Built-in, 2=Other, 3=AA, 4=AAA, …, 0xFF=Unknown).
    pub fn set_battery_size(&mut self, size: u8) {
        let _ = self.store.set_raw(ATTR_BATTERY_SIZE, ZclValue::Enum8(size));
    }

    /// Set number of battery cells.
    pub fn set_battery_quantity(&mut self, qty: u8) {
        let _ = self.store.set_raw(ATTR_BATTERY_QUANTITY, ZclValue::U8(qty));
    }

    /// Set battery rated voltage (in 100 mV units, e.g. 12 = 1.2 V for NiMH).
    pub fn set_battery_rated_voltage(&mut self, voltage_100mv: u8) {
        let _ = self
            .store
            .set_raw(ATTR_BATTERY_RATED_VOLTAGE, ZclValue::U8(voltage_100mv));
    }

    /// Set minimum battery voltage threshold (100 mV units) for alarm.
    pub fn set_battery_voltage_min_threshold(&mut self, voltage_100mv: u8) {
        let _ = self.store.set_raw(
            ATTR_BATTERY_VOLTAGE_MIN_THRESHOLD,
            ZclValue::U8(voltage_100mv),
        );
    }

    /// Recalculate BatteryAlarmState from current voltage vs threshold.
    ///
    /// Call this after updating the battery voltage. Sets bit 0 of
    /// BatteryAlarmState if voltage < min threshold (and alarm mask allows it).
    pub fn update_alarm_state(&mut self) {
        let voltage = match self.store.get(ATTR_BATTERY_VOLTAGE) {
            Some(ZclValue::U8(v)) => *v,
            _ => 0,
        };
        let threshold = match self.store.get(ATTR_BATTERY_VOLTAGE_MIN_THRESHOLD) {
            Some(ZclValue::U8(v)) => *v,
            _ => 0,
        };
        let alarm_mask = match self.store.get(ATTR_BATTERY_ALARM_MASK) {
            Some(ZclValue::Bitmap8(v)) => *v,
            _ => 0,
        };

        let mut alarm_state: u32 = 0;
        // Bit 0: battery voltage below min threshold
        if threshold > 0 && voltage < threshold && (alarm_mask & 0x01) != 0 {
            alarm_state |= 0x01;
        }
        let _ = self
            .store
            .set_raw(ATTR_BATTERY_ALARM_STATE, ZclValue::Bitmap32(alarm_state));
    }
}

impl Cluster for PowerConfigCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::POWER_CONFIG
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
