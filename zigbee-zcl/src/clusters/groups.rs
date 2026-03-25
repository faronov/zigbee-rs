//! Groups cluster (0x0004).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs
pub const ATTR_NAME_SUPPORT: AttributeId = AttributeId(0x0000);

// Command IDs (client to server)
pub const CMD_ADD_GROUP: CommandId = CommandId(0x00);
pub const CMD_VIEW_GROUP: CommandId = CommandId(0x01);
pub const CMD_GET_GROUP_MEMBERSHIP: CommandId = CommandId(0x02);
pub const CMD_REMOVE_GROUP: CommandId = CommandId(0x03);
pub const CMD_REMOVE_ALL_GROUPS: CommandId = CommandId(0x04);
pub const CMD_ADD_GROUP_IF_IDENTIFYING: CommandId = CommandId(0x05);

// Response command IDs (server to client)
pub const CMD_ADD_GROUP_RESPONSE: CommandId = CommandId(0x00);
pub const CMD_VIEW_GROUP_RESPONSE: CommandId = CommandId(0x01);
pub const CMD_GET_GROUP_MEMBERSHIP_RESPONSE: CommandId = CommandId(0x02);
pub const CMD_REMOVE_GROUP_RESPONSE: CommandId = CommandId(0x03);

/// Maximum number of groups a device can belong to.
pub const MAX_GROUPS: usize = 16;

/// Groups cluster implementation.
pub struct GroupsCluster {
    store: AttributeStore<4>,
    /// List of group IDs this endpoint belongs to.
    groups: heapless::Vec<u16, MAX_GROUPS>,
}

impl Default for GroupsCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl GroupsCluster {
    pub fn new() -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_NAME_SUPPORT,
                data_type: ZclDataType::U8,
                access: AttributeAccess::ReadOnly,
                name: "NameSupport",
            },
            ZclValue::U8(0x00), // Group names not supported
        );
        Self {
            store,
            groups: heapless::Vec::new(),
        }
    }

    fn add_group(&mut self, group_id: u16) -> u8 {
        if self.groups.contains(&group_id) {
            return 0x8A; // DUPLICATE_EXISTS
        }
        match self.groups.push(group_id) {
            Ok(()) => ZclStatus::Success as u8,
            Err(_) => ZclStatus::InsufficientSpace as u8,
        }
    }

    fn remove_group(&mut self, group_id: u16) -> u8 {
        if let Some(pos) = self.groups.iter().position(|&g| g == group_id) {
            self.groups.swap_remove(pos);
            ZclStatus::Success as u8
        } else {
            ZclStatus::NotFound as u8
        }
    }
}

impl Cluster for GroupsCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::GROUPS
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        match cmd_id {
            CMD_ADD_GROUP => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let group_id = u16::from_le_bytes([payload[0], payload[1]]);
                let status = self.add_group(group_id);
                let mut resp = heapless::Vec::new();
                let _ = resp.push(status);
                let b = group_id.to_le_bytes();
                let _ = resp.push(b[0]);
                let _ = resp.push(b[1]);
                Ok(resp)
            }
            CMD_VIEW_GROUP => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let group_id = u16::from_le_bytes([payload[0], payload[1]]);
                let status = if self.groups.contains(&group_id) {
                    ZclStatus::Success as u8
                } else {
                    ZclStatus::NotFound as u8
                };
                let mut resp = heapless::Vec::new();
                let _ = resp.push(status);
                let b = group_id.to_le_bytes();
                let _ = resp.push(b[0]);
                let _ = resp.push(b[1]);
                let _ = resp.push(0); // empty group name
                Ok(resp)
            }
            CMD_GET_GROUP_MEMBERSHIP => {
                let mut resp = heapless::Vec::new();
                // Capacity
                let _ = resp.push((MAX_GROUPS - self.groups.len()) as u8);
                if payload.is_empty() || payload[0] == 0 {
                    // Return all groups
                    let _ = resp.push(self.groups.len() as u8);
                    for &gid in &self.groups {
                        let b = gid.to_le_bytes();
                        let _ = resp.push(b[0]);
                        let _ = resp.push(b[1]);
                    }
                } else {
                    let count = payload[0] as usize;
                    let mut matched: heapless::Vec<u16, MAX_GROUPS> = heapless::Vec::new();
                    let mut i = 1;
                    for _ in 0..count {
                        if i + 1 < payload.len() {
                            let gid = u16::from_le_bytes([payload[i], payload[i + 1]]);
                            if self.groups.contains(&gid) {
                                let _ = matched.push(gid);
                            }
                            i += 2;
                        }
                    }
                    let _ = resp.push(matched.len() as u8);
                    for &gid in &matched {
                        let b = gid.to_le_bytes();
                        let _ = resp.push(b[0]);
                        let _ = resp.push(b[1]);
                    }
                }
                Ok(resp)
            }
            CMD_REMOVE_GROUP => {
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let group_id = u16::from_le_bytes([payload[0], payload[1]]);
                let status = self.remove_group(group_id);
                let mut resp = heapless::Vec::new();
                let _ = resp.push(status);
                let b = group_id.to_le_bytes();
                let _ = resp.push(b[0]);
                let _ = resp.push(b[1]);
                Ok(resp)
            }
            CMD_REMOVE_ALL_GROUPS => {
                self.groups.clear();
                Ok(heapless::Vec::new())
            }
            CMD_ADD_GROUP_IF_IDENTIFYING => {
                // Only add if the identify cluster has IdentifyTime > 0.
                // Since we don't have cross-cluster access here, just accept.
                if payload.len() < 2 {
                    return Err(ZclStatus::MalformedCommand);
                }
                let group_id = u16::from_le_bytes([payload[0], payload[1]]);
                let _ = self.add_group(group_id);
                Ok(heapless::Vec::new()) // No response for this command
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
