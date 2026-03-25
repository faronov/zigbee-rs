//! APS group table (Zigbee spec 2.2.8.5).
//!
//! The group table maps group addresses to local endpoints. When a frame
//! arrives addressed to a group, the APS layer delivers it to each endpoint
//! that is a member of that group.

/// Maximum number of groups in the table.
pub const MAX_GROUPS: usize = 16;

/// Maximum number of endpoints per group.
pub const MAX_ENDPOINTS_PER_GROUP: usize = 8;

// ── Group entry ─────────────────────────────────────────────────

/// A single group table entry.
///
/// Each entry maps a 16-bit group address to a list of local endpoints
/// that are members of the group.
#[derive(Debug, Clone)]
pub struct GroupEntry {
    /// 16-bit group address (0x0000-0xFFFF)
    pub group_address: u16,
    /// Endpoints that belong to this group
    pub endpoint_list: heapless::Vec<u8, MAX_ENDPOINTS_PER_GROUP>,
}

// ── Group table ─────────────────────────────────────────────────

/// Fixed-capacity APS group table.
pub struct GroupTable {
    groups: heapless::Vec<GroupEntry, MAX_GROUPS>,
}

impl GroupTable {
    /// Create an empty group table.
    pub fn new() -> Self {
        Self {
            groups: heapless::Vec::new(),
        }
    }

    /// Number of groups currently in the table.
    pub fn len(&self) -> usize {
        self.groups.len()
    }

    /// Whether the table is empty.
    pub fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }

    /// Add an endpoint to a group. Creates the group if it doesn't exist.
    ///
    /// Returns `true` if the endpoint was added (or already present).
    /// Returns `false` if the group table is full or the endpoint list is full.
    pub fn add_group(&mut self, group_address: u16, endpoint: u8) -> bool {
        // Find existing group
        if let Some(group) = self
            .groups
            .iter_mut()
            .find(|g| g.group_address == group_address)
        {
            // Already a member?
            if group.endpoint_list.contains(&endpoint) {
                return true;
            }
            // Add endpoint to existing group
            return group.endpoint_list.push(endpoint).is_ok();
        }

        // Create new group
        let mut ep_list = heapless::Vec::new();
        if ep_list.push(endpoint).is_err() {
            return false;
        }
        self.groups
            .push(GroupEntry {
                group_address,
                endpoint_list: ep_list,
            })
            .is_ok()
    }

    /// Remove an endpoint from a group. If the group becomes empty, it is
    /// removed from the table.
    ///
    /// Returns `true` if the endpoint was removed.
    pub fn remove_group(&mut self, group_address: u16, endpoint: u8) -> bool {
        if let Some(idx) = self
            .groups
            .iter()
            .position(|g| g.group_address == group_address)
        {
            let group = &mut self.groups[idx];
            if let Some(ep_idx) = group.endpoint_list.iter().position(|&e| e == endpoint) {
                group.endpoint_list.swap_remove(ep_idx);
                // Remove the group entirely if no endpoints remain
                if group.endpoint_list.is_empty() {
                    self.groups.swap_remove(idx);
                }
                return true;
            }
        }
        false
    }

    /// Remove all groups for a given endpoint.
    pub fn remove_all_groups(&mut self, endpoint: u8) {
        // Remove the endpoint from every group; drop groups that become empty.
        let mut i = 0;
        while i < self.groups.len() {
            let group = &mut self.groups[i];
            if let Some(ep_idx) = group.endpoint_list.iter().position(|&e| e == endpoint) {
                group.endpoint_list.swap_remove(ep_idx);
            }
            if group.endpoint_list.is_empty() {
                self.groups.swap_remove(i);
                // Don't increment — the swapped element is now at position i
            } else {
                i += 1;
            }
        }
    }

    /// Find a group by address. Returns the endpoint list.
    pub fn find(&self, group_address: u16) -> Option<&GroupEntry> {
        self.groups
            .iter()
            .find(|g| g.group_address == group_address)
    }

    /// Check whether an endpoint is a member of a group.
    pub fn is_member(&self, group_address: u16, endpoint: u8) -> bool {
        self.groups
            .iter()
            .any(|g| g.group_address == group_address && g.endpoint_list.contains(&endpoint))
    }

    /// Get all groups as a slice.
    pub fn groups(&self) -> &[GroupEntry] {
        &self.groups
    }
}

impl Default for GroupTable {
    fn default() -> Self {
        Self::new()
    }
}
