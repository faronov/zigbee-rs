//! APS binding table (Zigbee spec 2.2.8.2).
//!
//! The binding table maps (source address, source endpoint, cluster) to
//! one or more destinations. It is used for indirect addressing: when
//! the APS layer receives a data request with address mode = Indirect,
//! it looks up matching binding entries to determine where to send the frame.

use zigbee_types::IeeeAddress;

/// Maximum number of entries in the binding table.
pub const MAX_BINDING_ENTRIES: usize = 32;

// ── Destination address in a binding entry ──────────────────────

/// Binding destination — either a group or a specific device+endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BindingDst {
    /// Group address (16-bit)
    Group(u16),
    /// Unicast: IEEE address + endpoint
    Unicast {
        dst_addr: IeeeAddress,
        dst_endpoint: u8,
    },
}

/// Destination address mode in a binding entry (Zigbee spec Table 2-13).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BindingDstMode {
    /// 16-bit group address
    Group = 0x01,
    /// 64-bit IEEE address + endpoint
    Extended = 0x03,
}

// ── Binding entry ───────────────────────────────────────────────

/// A single binding table entry (Zigbee spec Table 2-13).
///
/// Each entry maps:
///   (src_addr, src_endpoint, cluster_id) → destination
#[derive(Debug, Clone)]
pub struct BindingEntry {
    /// Source IEEE address (this device, typically)
    pub src_addr: IeeeAddress,
    /// Source endpoint (1-240)
    pub src_endpoint: u8,
    /// Cluster identifier
    pub cluster_id: u16,
    /// Destination address mode
    pub dst_addr_mode: BindingDstMode,
    /// Destination
    pub dst: BindingDst,
}

impl BindingEntry {
    /// Create a unicast binding entry.
    pub fn unicast(
        src_addr: IeeeAddress,
        src_endpoint: u8,
        cluster_id: u16,
        dst_addr: IeeeAddress,
        dst_endpoint: u8,
    ) -> Self {
        Self {
            src_addr,
            src_endpoint,
            cluster_id,
            dst_addr_mode: BindingDstMode::Extended,
            dst: BindingDst::Unicast {
                dst_addr,
                dst_endpoint,
            },
        }
    }

    /// Create a group binding entry.
    pub fn group(
        src_addr: IeeeAddress,
        src_endpoint: u8,
        cluster_id: u16,
        group_address: u16,
    ) -> Self {
        Self {
            src_addr,
            src_endpoint,
            cluster_id,
            dst_addr_mode: BindingDstMode::Group,
            dst: BindingDst::Group(group_address),
        }
    }
}

// ── Binding table ───────────────────────────────────────────────

/// Fixed-capacity binding table.
pub struct BindingTable {
    entries: heapless::Vec<BindingEntry, MAX_BINDING_ENTRIES>,
}

impl BindingTable {
    /// Create an empty binding table.
    pub fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
        }
    }

    /// Number of entries currently in the table.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the table is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Whether the table is full.
    pub fn is_full(&self) -> bool {
        self.entries.is_full()
    }

    /// Remove all binding entries.
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// Add a binding entry. Returns `Err(entry)` if the table is full
    /// or the entry already exists.
    pub fn add(&mut self, entry: BindingEntry) -> Result<(), BindingEntry> {
        // Check for duplicate
        if self.find_exact(&entry).is_some() {
            return Err(entry);
        }
        self.entries.push(entry)
    }

    /// Remove a binding entry matching all fields. Returns true if found.
    pub fn remove(
        &mut self,
        src_addr: &IeeeAddress,
        src_endpoint: u8,
        cluster_id: u16,
        dst: &BindingDst,
    ) -> bool {
        if let Some(idx) = self.entries.iter().position(|e| {
            e.src_addr == *src_addr
                && e.src_endpoint == src_endpoint
                && e.cluster_id == cluster_id
                && e.dst == *dst
        }) {
            self.entries.swap_remove(idx);
            true
        } else {
            false
        }
    }

    /// Find all entries matching (src_addr, src_endpoint, cluster_id).
    ///
    /// Returns an iterator over matching entries — used for indirect addressing.
    pub fn find_by_source(
        &self,
        src_addr: &IeeeAddress,
        src_endpoint: u8,
        cluster_id: u16,
    ) -> impl Iterator<Item = &BindingEntry> {
        self.entries.iter().filter(move |e| {
            e.src_addr == *src_addr && e.src_endpoint == src_endpoint && e.cluster_id == cluster_id
        })
    }

    /// Find all entries matching a given cluster ID (across all endpoints).
    pub fn find_by_cluster(&self, cluster_id: u16) -> impl Iterator<Item = &BindingEntry> {
        self.entries
            .iter()
            .filter(move |e| e.cluster_id == cluster_id)
    }

    /// Find all entries for a given source endpoint.
    pub fn find_by_endpoint(&self, src_endpoint: u8) -> impl Iterator<Item = &BindingEntry> {
        self.entries
            .iter()
            .filter(move |e| e.src_endpoint == src_endpoint)
    }

    /// Get all entries as a slice.
    pub fn entries(&self) -> &[BindingEntry] {
        &self.entries
    }

    /// Internal: check if an exact duplicate exists.
    fn find_exact(&self, entry: &BindingEntry) -> Option<usize> {
        self.entries.iter().position(|e| {
            e.src_addr == entry.src_addr
                && e.src_endpoint == entry.src_endpoint
                && e.cluster_id == entry.cluster_id
                && e.dst == entry.dst
        })
    }
}

impl Default for BindingTable {
    fn default() -> Self {
        Self::new()
    }
}
