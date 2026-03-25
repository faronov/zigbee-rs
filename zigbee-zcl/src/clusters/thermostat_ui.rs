//! Thermostat User Interface Configuration cluster (0x0204).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_TEMPERATURE_DISPLAY_MODE: AttributeId = AttributeId(0x0000);
pub const ATTR_KEYPAD_LOCKOUT: AttributeId = AttributeId(0x0001);
pub const ATTR_SCHEDULE_PROGRAMMING_VISIBILITY: AttributeId = AttributeId(0x0002);

// TemperatureDisplayMode values
pub const DISPLAY_CELSIUS: u8 = 0x00;
pub const DISPLAY_FAHRENHEIT: u8 = 0x01;

// KeypadLockout values
pub const KEYPAD_NO_LOCKOUT: u8 = 0x00;
pub const KEYPAD_LEVEL1: u8 = 0x01;
pub const KEYPAD_LEVEL2: u8 = 0x02;
pub const KEYPAD_LEVEL3: u8 = 0x03;
pub const KEYPAD_LEVEL4: u8 = 0x04;
pub const KEYPAD_LEVEL5: u8 = 0x05;

// ScheduleProgrammingVisibility values
pub const SCHEDULE_ENABLED: u8 = 0x00;
pub const SCHEDULE_DISABLED: u8 = 0x01;

/// Thermostat User Interface Configuration cluster.
pub struct ThermostatUiCluster {
    store: AttributeStore<3>,
}

impl Default for ThermostatUiCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl ThermostatUiCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_TEMPERATURE_DISPLAY_MODE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "TemperatureDisplayMode",
            },
            ZclValue::Enum8(DISPLAY_CELSIUS),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_KEYPAD_LOCKOUT,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "KeypadLockout",
            },
            ZclValue::Enum8(KEYPAD_NO_LOCKOUT),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SCHEDULE_PROGRAMMING_VISIBILITY,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadWrite,
                name: "ScheduleProgrammingVisibility",
            },
            ZclValue::Enum8(SCHEDULE_ENABLED),
        );
        Self { store }
    }

    /// Get current temperature display mode.
    pub fn display_mode(&self) -> u8 {
        match self.store.get(ATTR_TEMPERATURE_DISPLAY_MODE) {
            Some(ZclValue::Enum8(v)) => *v,
            _ => DISPLAY_CELSIUS,
        }
    }

    /// Set temperature display mode.
    pub fn set_display_mode(&mut self, mode: u8) {
        let _ = self
            .store
            .set_raw(ATTR_TEMPERATURE_DISPLAY_MODE, ZclValue::Enum8(mode));
    }

    /// Get current keypad lockout level.
    pub fn keypad_lockout(&self) -> u8 {
        match self.store.get(ATTR_KEYPAD_LOCKOUT) {
            Some(ZclValue::Enum8(v)) => *v,
            _ => KEYPAD_NO_LOCKOUT,
        }
    }

    /// Set keypad lockout level.
    pub fn set_keypad_lockout(&mut self, level: u8) {
        let _ = self
            .store
            .set_raw(ATTR_KEYPAD_LOCKOUT, ZclValue::Enum8(level));
    }
}

impl Cluster for ThermostatUiCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0204)
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
