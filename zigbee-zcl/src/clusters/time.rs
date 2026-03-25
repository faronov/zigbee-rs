//! Time cluster (0x000A).
//!
//! Provides a common time reference for the Zigbee network.
//! ZCL epoch = Jan 1, 2000 00:00:00 UTC.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_TIME: AttributeId = AttributeId(0x0000);
pub const ATTR_TIME_STATUS: AttributeId = AttributeId(0x0001);
pub const ATTR_TIME_ZONE: AttributeId = AttributeId(0x0002);
pub const ATTR_DST_START: AttributeId = AttributeId(0x0003);
pub const ATTR_DST_END: AttributeId = AttributeId(0x0004);
pub const ATTR_DST_SHIFT: AttributeId = AttributeId(0x0005);
pub const ATTR_STANDARD_TIME: AttributeId = AttributeId(0x0006);
pub const ATTR_LOCAL_TIME: AttributeId = AttributeId(0x0007);
pub const ATTR_LAST_SET_TIME: AttributeId = AttributeId(0x0008);
pub const ATTR_VALID_UNTIL_TIME: AttributeId = AttributeId(0x0009);

/// Offset in seconds from Unix epoch (1970-01-01) to ZCL epoch (2000-01-01).
pub const ZCL_EPOCH_OFFSET: u32 = 946684800;

// TimeStatus bitmap bits
pub const TIME_STATUS_MASTER: u8 = 0x01;
pub const TIME_STATUS_SYNCHRONIZED: u8 = 0x02;
pub const TIME_STATUS_MASTER_ZONE_DST: u8 = 0x04;
pub const TIME_STATUS_SUPERSEDING: u8 = 0x08;

/// Time cluster implementation.
pub struct TimeCluster {
    store: AttributeStore<10>,
}

impl Default for TimeCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_TIME,
                data_type: ZclDataType::UtcTime,
                access: AttributeAccess::ReadWrite,
                name: "Time",
            },
            ZclValue::UtcTime(0xFFFFFFFF), // invalid until set
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_TIME_STATUS,
                data_type: ZclDataType::Bitmap8,
                access: AttributeAccess::ReadWrite,
                name: "TimeStatus",
            },
            ZclValue::Bitmap8(0x00),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_TIME_ZONE,
                data_type: ZclDataType::I32,
                access: AttributeAccess::ReadWrite,
                name: "TimeZone",
            },
            ZclValue::I32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DST_START,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadWrite,
                name: "DstStart",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DST_END,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadWrite,
                name: "DstEnd",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DST_SHIFT,
                data_type: ZclDataType::I32,
                access: AttributeAccess::ReadWrite,
                name: "DstShift",
            },
            ZclValue::I32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_STANDARD_TIME,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "StandardTime",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LOCAL_TIME,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "LocalTime",
            },
            ZclValue::U32(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAST_SET_TIME,
                data_type: ZclDataType::UtcTime,
                access: AttributeAccess::ReadOnly,
                name: "LastSetTime",
            },
            ZclValue::UtcTime(0xFFFFFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_VALID_UNTIL_TIME,
                data_type: ZclDataType::UtcTime,
                access: AttributeAccess::ReadWrite,
                name: "ValidUntilTime",
            },
            ZclValue::UtcTime(0xFFFFFFFF),
        );
        Self { store }
    }

    /// Set the current ZCL time and update LastSetTime.
    pub fn set_time(&mut self, zcl_utc: u32) {
        let _ = self.store.set_raw(ATTR_TIME, ZclValue::UtcTime(zcl_utc));
        let _ = self
            .store
            .set_raw(ATTR_LAST_SET_TIME, ZclValue::UtcTime(zcl_utc));
    }

    /// Get the current ZCL time value.
    pub fn get_time(&self) -> u32 {
        match self.store.get(ATTR_TIME) {
            Some(ZclValue::UtcTime(v)) => *v,
            _ => 0xFFFFFFFF,
        }
    }
}

impl Cluster for TimeCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId(0x000A)
    }

    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        // No cluster-specific commands defined for Time cluster.
        Err(ZclStatus::UnsupClusterCommand)
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
