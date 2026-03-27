//! Scenes cluster (0x0005).
//!
//! Implements the ZCL Scenes cluster with a fixed-capacity scene table.
//! Supports Add Scene, View Scene, Remove Scene, Remove All Scenes,
//! Store Scene, Recall Scene, and Get Scene Membership commands.

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

// Command IDs (client → server)
pub const CMD_ADD_SCENE: CommandId = CommandId(0x00);
pub const CMD_VIEW_SCENE: CommandId = CommandId(0x01);
pub const CMD_REMOVE_SCENE: CommandId = CommandId(0x02);
pub const CMD_REMOVE_ALL_SCENES: CommandId = CommandId(0x03);
pub const CMD_STORE_SCENE: CommandId = CommandId(0x04);
pub const CMD_RECALL_SCENE: CommandId = CommandId(0x05);
pub const CMD_GET_SCENE_MEMBERSHIP: CommandId = CommandId(0x06);

/// Maximum number of scenes the table can hold.
const MAX_SCENES: usize = 16;
/// Maximum extension data per scene (cluster attribute snapshots).
const MAX_EXTENSION_DATA: usize = 32;

/// A single scene table entry.
#[derive(Debug, Clone)]
struct SceneEntry {
    group_id: u16,
    scene_id: u8,
    transition_time: u16,
    extension_data: heapless::Vec<u8, MAX_EXTENSION_DATA>,
    active: bool,
}

impl SceneEntry {
    const fn empty() -> Self {
        Self {
            group_id: 0,
            scene_id: 0,
            transition_time: 0,
            extension_data: heapless::Vec::new(),
            active: false,
        }
    }
}

/// Scenes cluster — full implementation with scene table.
pub struct ScenesCluster {
    store: AttributeStore<8>,
    scenes: [SceneEntry; MAX_SCENES],
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
        Self {
            store,
            scenes: core::array::from_fn(|_| SceneEntry::empty()),
        }
    }

    /// Number of active scenes.
    pub fn scene_count(&self) -> u8 {
        self.scenes.iter().filter(|s| s.active).count() as u8
    }

    fn update_scene_count(&mut self) {
        let count = self.scene_count();
        let _ = self.store.set_raw(ATTR_SCENE_COUNT, ZclValue::U8(count));
    }

    fn find_scene(&self, group_id: u16, scene_id: u8) -> Option<usize> {
        self.scenes
            .iter()
            .position(|s| s.active && s.group_id == group_id && s.scene_id == scene_id)
    }

    fn find_empty_slot(&self) -> Option<usize> {
        self.scenes.iter().position(|s| !s.active)
    }

    /// Add Scene (0x00): group_id(2) + scene_id(1) + transition_time(2) + name_len(1) + name + ext
    fn handle_add_scene(&mut self, payload: &[u8]) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        if payload.len() < 5 {
            return Err(ZclStatus::MalformedCommand);
        }
        let group_id = u16::from_le_bytes([payload[0], payload[1]]);
        let scene_id = payload[2];
        let transition_time = u16::from_le_bytes([payload[3], payload[4]]);

        // Skip scene name (length-prefixed string)
        let name_len = if payload.len() > 5 {
            payload[5] as usize
        } else {
            0
        };
        let ext_start = 6 + name_len;
        let ext_data = if payload.len() > ext_start {
            &payload[ext_start..]
        } else {
            &[]
        };

        let status = if let Some(idx) = self.find_scene(group_id, scene_id) {
            // Update existing
            self.scenes[idx].transition_time = transition_time;
            self.scenes[idx].extension_data.clear();
            for &b in ext_data {
                let _ = self.scenes[idx].extension_data.push(b);
            }
            ZclStatus::Success
        } else if let Some(idx) = self.find_empty_slot() {
            self.scenes[idx] = SceneEntry {
                group_id,
                scene_id,
                transition_time,
                extension_data: {
                    let mut v = heapless::Vec::new();
                    for &b in ext_data {
                        let _ = v.push(b);
                    }
                    v
                },
                active: true,
            };
            ZclStatus::Success
        } else {
            ZclStatus::InsufficientSpace
        };

        self.update_scene_count();
        // Response: status(1) + group_id(2) + scene_id(1)
        let mut resp = heapless::Vec::new();
        let _ = resp.push(status as u8);
        let _ = resp.extend_from_slice(&group_id.to_le_bytes());
        let _ = resp.push(scene_id);
        Ok(resp)
    }

    /// View Scene (0x01): group_id(2) + scene_id(1)
    fn handle_view_scene(&self, payload: &[u8]) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        if payload.len() < 3 {
            return Err(ZclStatus::MalformedCommand);
        }
        let group_id = u16::from_le_bytes([payload[0], payload[1]]);
        let scene_id = payload[2];
        let mut resp = heapless::Vec::new();

        if let Some(idx) = self.find_scene(group_id, scene_id) {
            let s = &self.scenes[idx];
            let _ = resp.push(ZclStatus::Success as u8);
            let _ = resp.extend_from_slice(&group_id.to_le_bytes());
            let _ = resp.push(scene_id);
            let _ = resp.extend_from_slice(&s.transition_time.to_le_bytes());
            let _ = resp.push(0); // name length = 0 (no name support)
            let _ = resp.extend_from_slice(&s.extension_data);
        } else {
            let _ = resp.push(ZclStatus::NotFound as u8);
            let _ = resp.extend_from_slice(&group_id.to_le_bytes());
            let _ = resp.push(scene_id);
        }
        Ok(resp)
    }

    /// Remove Scene (0x02): group_id(2) + scene_id(1)
    fn handle_remove_scene(&mut self, payload: &[u8]) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        if payload.len() < 3 {
            return Err(ZclStatus::MalformedCommand);
        }
        let group_id = u16::from_le_bytes([payload[0], payload[1]]);
        let scene_id = payload[2];

        let status = if let Some(idx) = self.find_scene(group_id, scene_id) {
            self.scenes[idx].active = false;
            ZclStatus::Success
        } else {
            ZclStatus::NotFound
        };

        self.update_scene_count();
        let mut resp = heapless::Vec::new();
        let _ = resp.push(status as u8);
        let _ = resp.extend_from_slice(&group_id.to_le_bytes());
        let _ = resp.push(scene_id);
        Ok(resp)
    }

    /// Remove All Scenes (0x03): group_id(2)
    fn handle_remove_all_scenes(
        &mut self,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        if payload.len() < 2 {
            return Err(ZclStatus::MalformedCommand);
        }
        let group_id = u16::from_le_bytes([payload[0], payload[1]]);
        for scene in &mut self.scenes {
            if scene.active && scene.group_id == group_id {
                scene.active = false;
            }
        }
        self.update_scene_count();
        let mut resp = heapless::Vec::new();
        let _ = resp.push(ZclStatus::Success as u8);
        let _ = resp.extend_from_slice(&group_id.to_le_bytes());
        Ok(resp)
    }

    /// Store Scene (0x04): group_id(2) + scene_id(1)
    fn handle_store_scene(&mut self, payload: &[u8]) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        if payload.len() < 3 {
            return Err(ZclStatus::MalformedCommand);
        }
        let group_id = u16::from_le_bytes([payload[0], payload[1]]);
        let scene_id = payload[2];

        // Store or update with empty extension data (runtime should snapshot attrs)
        let status = if let Some(idx) = self.find_scene(group_id, scene_id) {
            self.scenes[idx].extension_data.clear();
            ZclStatus::Success
        } else if let Some(idx) = self.find_empty_slot() {
            self.scenes[idx] = SceneEntry {
                group_id,
                scene_id,
                transition_time: 0,
                extension_data: heapless::Vec::new(),
                active: true,
            };
            ZclStatus::Success
        } else {
            ZclStatus::InsufficientSpace
        };

        if status == ZclStatus::Success {
            let _ = self
                .store
                .set_raw(ATTR_CURRENT_SCENE, ZclValue::U8(scene_id));
            let _ = self
                .store
                .set_raw(ATTR_CURRENT_GROUP, ZclValue::U16(group_id));
            let _ = self.store.set_raw(ATTR_SCENE_VALID, ZclValue::Bool(true));
        }

        self.update_scene_count();
        let mut resp = heapless::Vec::new();
        let _ = resp.push(status as u8);
        let _ = resp.extend_from_slice(&group_id.to_le_bytes());
        let _ = resp.push(scene_id);
        Ok(resp)
    }

    /// Recall Scene (0x05): group_id(2) + scene_id(1) [+ transition_time(2)]
    fn handle_recall_scene(&mut self, payload: &[u8]) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        if payload.len() < 3 {
            return Err(ZclStatus::MalformedCommand);
        }
        let group_id = u16::from_le_bytes([payload[0], payload[1]]);
        let scene_id = payload[2];

        if self.find_scene(group_id, scene_id).is_some() {
            let _ = self
                .store
                .set_raw(ATTR_CURRENT_SCENE, ZclValue::U8(scene_id));
            let _ = self
                .store
                .set_raw(ATTR_CURRENT_GROUP, ZclValue::U16(group_id));
            let _ = self.store.set_raw(ATTR_SCENE_VALID, ZclValue::Bool(true));
            // No response payload for Recall Scene — it's a no-response command
            Ok(heapless::Vec::new())
        } else {
            Err(ZclStatus::NotFound)
        }
    }

    /// Get Scene Membership (0x06): group_id(2)
    fn handle_get_scene_membership(
        &self,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        if payload.len() < 2 {
            return Err(ZclStatus::MalformedCommand);
        }
        let group_id = u16::from_le_bytes([payload[0], payload[1]]);
        let capacity = (MAX_SCENES - self.scenes.iter().filter(|s| s.active).count()) as u8;

        let mut scene_list: heapless::Vec<u8, 16> = heapless::Vec::new();
        for scene in &self.scenes {
            if scene.active && scene.group_id == group_id {
                let _ = scene_list.push(scene.scene_id);
            }
        }

        let mut resp = heapless::Vec::new();
        let _ = resp.push(ZclStatus::Success as u8);
        let _ = resp.push(capacity);
        let _ = resp.extend_from_slice(&group_id.to_le_bytes());
        let _ = resp.push(scene_list.len() as u8);
        let _ = resp.extend_from_slice(&scene_list);
        Ok(resp)
    }
}

impl Cluster for ScenesCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::SCENES
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_ADD_SCENE => self.handle_add_scene(payload),
            CMD_VIEW_SCENE => self.handle_view_scene(payload),
            CMD_REMOVE_SCENE => self.handle_remove_scene(payload),
            CMD_REMOVE_ALL_SCENES => self.handle_remove_all_scenes(payload),
            CMD_STORE_SCENE => self.handle_store_scene(payload),
            CMD_RECALL_SCENE => self.handle_recall_scene(payload),
            CMD_GET_SCENE_MEMBERSHIP => self.handle_get_scene_membership(payload),
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
