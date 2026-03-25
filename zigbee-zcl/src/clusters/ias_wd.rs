//! IAS WD (Warning Devices) cluster (0x0502).
//!
//! Controls sirens and strobes. Supports StartWarning (with duration
//! clamped to MaxDuration) and Squawk commands.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_MAX_DURATION: AttributeId = AttributeId(0x0000);

// Command IDs (client → server)
pub const CMD_START_WARNING: CommandId = CommandId(0x00);
pub const CMD_SQUAWK: CommandId = CommandId(0x01);

// Warning mode values (bits 7-4 of warning_info)
pub const WARNING_MODE_STOP: u8 = 0;
pub const WARNING_MODE_BURGLAR: u8 = 1;
pub const WARNING_MODE_FIRE: u8 = 2;
pub const WARNING_MODE_EMERGENCY: u8 = 3;
pub const WARNING_MODE_POLICE_PANIC: u8 = 4;
pub const WARNING_MODE_FIRE_PANIC: u8 = 5;
pub const WARNING_MODE_EMERGENCY_PANIC: u8 = 6;

// Strobe values (bits 3-2 of warning_info)
pub const STROBE_NO_STROBE: u8 = 0;
pub const STROBE_USE_STROBE: u8 = 1;

// Siren level values (bits 1-0 of warning_info)
pub const SIREN_LEVEL_LOW: u8 = 0;
pub const SIREN_LEVEL_MEDIUM: u8 = 1;
pub const SIREN_LEVEL_HIGH: u8 = 2;
pub const SIREN_LEVEL_VERY_HIGH: u8 = 3;

// Squawk mode values (bits 7-4 of squawk_info)
pub const SQUAWK_MODE_SYSTEM_ARMED: u8 = 0;
pub const SQUAWK_MODE_SYSTEM_DISARMED: u8 = 1;

/// IAS WD cluster implementation.
pub struct IasWdCluster {
    store: AttributeStore<1>,
    /// Current warning mode (0 = stopped).
    pub warning_mode: u8,
    /// Current warning duration in seconds.
    pub warning_duration: u16,
}

impl Default for IasWdCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl IasWdCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_DURATION,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "MaxDuration",
            },
            ZclValue::U16(240),
        );
        Self {
            store,
            warning_mode: 0,
            warning_duration: 0,
        }
    }

    fn get_max_duration(&self) -> u16 {
        match self.store.get(ATTR_MAX_DURATION) {
            Some(ZclValue::U16(v)) => *v,
            _ => 240,
        }
    }

    /// Programmatically start a warning.
    pub fn start_warning(&mut self, mode: u8, strobe: bool, level: u8, duration: u16) {
        let max = self.get_max_duration();
        self.warning_mode = mode;
        self.warning_duration = duration.min(max);
        let _ = strobe;
        let _ = level;
    }

    /// Programmatically trigger a squawk.
    pub fn squawk(&mut self, mode: u8, strobe: bool, level: u8) {
        let _ = mode;
        let _ = strobe;
        let _ = level;
    }
}

impl Cluster for IasWdCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x0502)
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_START_WARNING => {
                if payload.len() < 5 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let warning_info = payload[0];
                let mode = (warning_info >> 4) & 0x0F;
                let strobe = ((warning_info >> 2) & 0x03) != 0;
                let level = warning_info & 0x03;
                let duration = u16::from_le_bytes([payload[1], payload[2]]);
                // payload[3] = strobe_duty_cycle, payload[4] = strobe_level (acknowledged)
                self.start_warning(mode, strobe, level, duration);
                Ok(heapless::Vec::new())
            }
            CMD_SQUAWK => {
                if payload.is_empty() {
                    return Err(ZclStatus::MalformedCommand);
                }
                let squawk_info = payload[0];
                let mode = (squawk_info >> 4) & 0x0F;
                let strobe = ((squawk_info >> 3) & 0x01) != 0;
                let level = squawk_info & 0x03;
                self.squawk(mode, strobe, level);
                Ok(heapless::Vec::new())
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
