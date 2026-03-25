//! Scenes cluster (0x0005) — stub implementation.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

pub const ATTR_SCENE_COUNT: AttributeId = AttributeId(0x0000);
pub const ATTR_CURRENT_SCENE: AttributeId = AttributeId(0x0001);
pub const ATTR_CURRENT_GROUP: AttributeId = AttributeId(0x0002);
pub const ATTR_SCENE_VALID: AttributeId = AttributeId(0x0003);
pub const ATTR_NAME_SUPPORT: AttributeId = AttributeId(0x0004);
pub const ATTR_LAST_CONFIGURED_BY: AttributeId = AttributeId(0x0005);

/// Scenes cluster (stub — attribute definitions only).
pub struct ScenesCluster {
    store: AttributeStore<8>,
}

impl Default for ScenesCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl ScenesCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SCENE_COUNT,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "SceneCount",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_SCENE,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "CurrentScene",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_GROUP,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "CurrentGroup",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_SCENE_VALID,
                data_type: ZclDataType::Bool,
                access: AttributeAccess::ReadOnly,
                name: "SceneValid",
            },
            ZclValue::Bool(false),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NAME_SUPPORT,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "NameSupport",
            },
            ZclValue::U8(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LAST_CONFIGURED_BY,
                data_type: ZclDataType::IeeeAddr,
                access: AttributeAccess::ReadOnly,
                name: "LastConfiguredBy",
            },
            ZclValue::IeeeAddr(0),
        );
        Self { store }
    }
}

impl Cluster for ScenesCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::SCENES
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
