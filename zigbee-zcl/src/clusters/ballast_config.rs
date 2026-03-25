//! Ballast Configuration cluster (0x0301).
//!
//! Used by lighting ballast devices to expose configuration of the physical
//! ballast (min/max levels, lamp burn hours, etc.).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Ballast Information
pub const ATTR_PHYSICAL_MIN_LEVEL: AttributeId = AttributeId(0x0000);
pub const ATTR_PHYSICAL_MAX_LEVEL: AttributeId = AttributeId(0x0001);
pub const ATTR_BALLAST_STATUS: AttributeId = AttributeId(0x0002);

// Ballast Settings
pub const ATTR_MIN_LEVEL: AttributeId = AttributeId(0x0010);
pub const ATTR_MAX_LEVEL: AttributeId = AttributeId(0x0011);
pub const ATTR_POWER_ON_LEVEL: AttributeId = AttributeId(0x0012);
pub const ATTR_POWER_ON_FADE_TIME: AttributeId = AttributeId(0x0013);
pub const ATTR_INTRINSIC_BALLAST_FACTOR: AttributeId = AttributeId(0x0014);
pub const ATTR_BALLAST_FACTOR_ADJUSTMENT: AttributeId = AttributeId(0x0015);

// Lamp Information
pub const ATTR_LAMP_QUANTITY: AttributeId = AttributeId(0x0020);

// Lamp Settings
pub const ATTR_LAMP_TYPE: AttributeId = AttributeId(0x0030);
pub const ATTR_LAMP_MANUFACTURER: AttributeId = AttributeId(0x0031);
pub const ATTR_LAMP_RATED_HOURS: AttributeId = AttributeId(0x0032);
pub const ATTR_LAMP_BURN_HOURS: AttributeId = AttributeId(0x0033);
pub const ATTR_LAMP_ALARM_MODE: AttributeId = AttributeId(0x0034);
pub const ATTR_LAMP_BURN_HOURS_TRIP_POINT: AttributeId = AttributeId(0x0035);

/// Ballast Configuration cluster.
pub struct BallastConfigCluster {
    store: AttributeStore<16>,
}

impl Default for BallastConfigCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl BallastConfigCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_PHYSICAL_MIN_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "PhysicalMinLevel",
            },
            ZclValue::U8(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_PHYSICAL_MAX_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "PhysicalMaxLevel",
            },
            ZclValue::U8(254),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BALLAST_STATUS,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "BallastStatus",
            },
            ZclValue::U8(0x01), // ballast is operational
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "MinLevel",
            },
            ZclValue::U8(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "MaxLevel",
            },
            ZclValue::U8(254),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_POWER_ON_LEVEL,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "PowerOnLevel",
            },
            ZclValue::U8(254),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_POWER_ON_FADE_TIME,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadWrite,
                name: "PowerOnFadeTime",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_INTRINSIC_BALLAST_FACTOR,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "IntrinsicBallastFactor",
            },
            ZclValue::U8(0xFF), // unknown
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_BALLAST_FACTOR_ADJUSTMENT,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "BallastFactorAdjustment",
            },
            ZclValue::U8(0xFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAMP_QUANTITY,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "LampQuantity",
            },
            ZclValue::U8(1),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAMP_RATED_HOURS,
                data_type: ZclDataType::U24,
                access: AttributeAccess::ReadWrite,
                name: "LampRatedHours",
            },
            ZclValue::U32(0x00FFFFFF), // unknown
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAMP_BURN_HOURS,
                data_type: ZclDataType::U24,
                access: AttributeAccess::ReadWrite,
                name: "LampBurnHours",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAMP_ALARM_MODE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadWrite,
                name: "LampAlarmMode",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAMP_BURN_HOURS_TRIP_POINT,
                data_type: ZclDataType::U24,
                access: AttributeAccess::ReadWrite,
                name: "LampBurnHoursTripPoint",
            },
            ZclValue::U32(0x00FFFFFF),
        );
        Self { store }
    }

    /// Update lamp burn hours.
    pub fn set_lamp_burn_hours(&mut self, hours: u32) {
        let _ = self
            .store
            .set_raw(ATTR_LAMP_BURN_HOURS, ZclValue::U32(hours & 0x00FFFFFF));
    }
}

impl Cluster for BallastConfigCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::BALLAST_CONFIG
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
