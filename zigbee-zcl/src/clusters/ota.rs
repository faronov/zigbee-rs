//! OTA Upgrade cluster (0x0019) — stub.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs (client-side, OTA Upgrade cluster is client-initiated)
pub const ATTR_UPGRADE_SERVER_ID: AttributeId = AttributeId(0x0000);
pub const ATTR_FILE_OFFSET: AttributeId = AttributeId(0x0001);
pub const ATTR_CURRENT_FILE_VERSION: AttributeId = AttributeId(0x0002);
pub const ATTR_CURRENT_STACK_VERSION: AttributeId = AttributeId(0x0003);
pub const ATTR_DOWNLOADED_FILE_VERSION: AttributeId = AttributeId(0x0004);
pub const ATTR_DOWNLOADED_STACK_VERSION: AttributeId = AttributeId(0x0005);
pub const ATTR_IMAGE_UPGRADE_STATUS: AttributeId = AttributeId(0x0006);
pub const ATTR_MANUFACTURER_ID: AttributeId = AttributeId(0x0007);
pub const ATTR_IMAGE_TYPE_ID: AttributeId = AttributeId(0x0008);
pub const ATTR_MIN_BLOCK_PERIOD: AttributeId = AttributeId(0x0009);

// Command IDs (client to server)
pub const CMD_QUERY_NEXT_IMAGE_REQUEST: CommandId = CommandId(0x01);
pub const CMD_IMAGE_BLOCK_REQUEST: CommandId = CommandId(0x03);
pub const CMD_UPGRADE_END_REQUEST: CommandId = CommandId(0x06);

// Command IDs (server to client)
pub const CMD_QUERY_NEXT_IMAGE_RESPONSE: CommandId = CommandId(0x02);
pub const CMD_IMAGE_BLOCK_RESPONSE: CommandId = CommandId(0x05);
pub const CMD_UPGRADE_END_RESPONSE: CommandId = CommandId(0x07);
pub const CMD_IMAGE_NOTIFY: CommandId = CommandId(0x00);

/// OTA Upgrade cluster stub.
pub struct OtaCluster {
    store: AttributeStore<12>,
}

impl OtaCluster {
    pub fn new(current_version: u32) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_UPGRADE_SERVER_ID,
                data_type: ZclDataType::IeeeAddr,
                access: AttributeAccess::ReadOnly,
                name: "UpgradeServerID",
            },
            ZclValue::IeeeAddr(0xFFFFFFFFFFFFFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_FILE_OFFSET,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "FileOffset",
            },
            ZclValue::U32(0xFFFFFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_FILE_VERSION,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "CurrentFileVersion",
            },
            ZclValue::U32(current_version),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_STACK_VERSION,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "CurrentZigbeeStackVersion",
            },
            ZclValue::U16(0x0002), // Zigbee PRO
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DOWNLOADED_FILE_VERSION,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "DownloadedFileVersion",
            },
            ZclValue::U32(0xFFFFFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DOWNLOADED_STACK_VERSION,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "DownloadedZigbeeStackVersion",
            },
            ZclValue::U16(0xFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_IMAGE_UPGRADE_STATUS,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "ImageUpgradeStatus",
            },
            ZclValue::Enum8(0x00), // Normal
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MANUFACTURER_ID,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ManufacturerID",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_IMAGE_TYPE_ID,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ImageTypeID",
            },
            ZclValue::U16(0xFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_BLOCK_PERIOD,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MinimumBlockPeriod",
            },
            ZclValue::U16(0),
        );
        Self { store }
    }
}

impl Cluster for OtaCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::OTA_UPGRADE
    }
    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        // OTA is complex; real implementation would be in the application layer.
        Err(ZclStatus::UnsupClusterCommand)
    }
    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
