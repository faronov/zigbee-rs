//! APS fragment reassembly for incoming fragmented frames.
//!
//! Manages up to `MAX_ENTRIES` concurrent reassembly sessions, each tracking
//! fragments via a bitmask (up to 8 blocks). When all blocks for a given
//! (src_addr, aps_counter) pair are received, the complete payload is returned.

/// Maximum concurrent reassembly sessions.
#[cfg(feature = "router")]
const MAX_ENTRIES: usize = 4;
#[cfg(not(feature = "router"))]
const MAX_ENTRIES: usize = 1;

/// A single fragment reassembly slot.
struct ReassemblyEntry {
    active: bool,
    src_addr: u16,
    aps_counter: u8,
    total_blocks: u8,
    received_mask: u8,
    data: [u8; 256],
    data_len: usize,
    /// Ticks since last fragment received (incremented by age_entries)
    age: u8,
}

impl ReassemblyEntry {
    const fn empty() -> Self {
        Self {
            active: false,
            src_addr: 0,
            aps_counter: 0,
            total_blocks: 0,
            received_mask: 0,
            data: [0u8; 256],
            data_len: 0,
            age: 0,
        }
    }

    fn matches(&self, src_addr: u16, aps_counter: u8) -> bool {
        self.active && self.src_addr == src_addr && self.aps_counter == aps_counter
    }

    fn is_complete(&self) -> bool {
        if !self.active || self.total_blocks == 0 || self.total_blocks > 8 {
            return false;
        }
        let expected = (1u16 << self.total_blocks) - 1;
        (self.received_mask as u16) & expected == expected
    }
}

/// APS fragment reassembly context.
///
/// Used by `ApsLayer` to reassemble incoming fragmented data frames.
pub struct FragmentReassembly {
    entries: [ReassemblyEntry; MAX_ENTRIES],
}

impl Default for FragmentReassembly {
    fn default() -> Self {
        Self::new()
    }
}

impl FragmentReassembly {
    #[cfg(feature = "router")]
    pub const fn new() -> Self {
        Self {
            entries: [
                ReassemblyEntry::empty(),
                ReassemblyEntry::empty(),
                ReassemblyEntry::empty(),
                ReassemblyEntry::empty(),
            ],
        }
    }
    #[cfg(not(feature = "router"))]
    pub const fn new() -> Self {
        Self {
            entries: [ReassemblyEntry::empty()],
        }
    }

    /// Insert a fragment (first or subsequent).
    ///
    /// * `block_num == 0` and `total_blocks > 0` → first fragment (creates entry)
    /// * `block_num > 0` and `total_blocks == 0` → subsequent fragment
    ///
    /// Returns `Some(&[u8])` with the reassembled payload when all blocks
    /// have been received, or `None` if more fragments are needed.
    pub fn insert_fragment(
        &mut self,
        src_addr: u16,
        aps_counter: u8,
        block_num: u8,
        total_blocks: u8,
        payload: &[u8],
    ) -> Option<&[u8]> {
        if block_num == 0 && total_blocks > 0 {
            // First fragment — allocate or reuse a slot
            let idx = self.find_or_alloc(src_addr, aps_counter);
            let entry = &mut self.entries[idx];
            entry.active = true;
            entry.src_addr = src_addr;
            entry.aps_counter = aps_counter;
            entry.total_blocks = total_blocks;
            entry.received_mask = 1; // block 0
            entry.age = 0;
            let copy_len = payload.len().min(entry.data.len());
            entry.data[..copy_len].copy_from_slice(&payload[..copy_len]);
            entry.data_len = copy_len;
        } else {
            // Subsequent fragment
            let idx = self
                .entries
                .iter()
                .position(|e| e.matches(src_addr, aps_counter))?;
            let entry = &mut self.entries[idx];
            if block_num >= entry.total_blocks {
                return None;
            }
            let bit = 1u8 << block_num;
            if entry.received_mask & bit != 0 {
                return None; // duplicate
            }
            entry.received_mask |= bit;
            entry.age = 0; // reset age on new fragment
            let copy_len = payload.len().min(entry.data.len() - entry.data_len);
            entry.data[entry.data_len..entry.data_len + copy_len]
                .copy_from_slice(&payload[..copy_len]);
            entry.data_len += copy_len;
        }

        // Check completeness
        let idx = self
            .entries
            .iter()
            .position(|e| e.matches(src_addr, aps_counter))?;
        if self.entries[idx].is_complete() {
            let e = &self.entries[idx];
            Some(&e.data[..e.data_len])
        } else {
            None
        }
    }

    /// Mark a completed entry as inactive so the slot can be reused.
    pub fn complete_entry(&mut self, src_addr: u16, aps_counter: u8) {
        if let Some(entry) = self
            .entries
            .iter_mut()
            .find(|e| e.matches(src_addr, aps_counter))
        {
            entry.active = false;
        }
    }

    /// Find an existing entry or allocate an inactive slot. Returns index.
    fn find_or_alloc(&mut self, src_addr: u16, aps_counter: u8) -> usize {
        // Reuse existing entry for the same source
        if let Some(idx) = self
            .entries
            .iter()
            .position(|e| e.matches(src_addr, aps_counter))
        {
            return idx;
        }
        // Find first inactive slot
        if let Some(idx) = self.entries.iter().position(|e| !e.active) {
            return idx;
        }
        // Evict slot 0 as last resort
        self.entries[0].active = false;
        0
    }

    /// Age reassembly entries — expire stale incomplete reassemblies.
    ///
    /// Should be called periodically (e.g., every 1 second from the runtime tick).
    /// Entries that have not received a new fragment within `MAX_AGE_TICKS`
    /// ticks are expired and their slots freed.
    pub fn age_entries(&mut self) {
        // ~10 ticks at 1s per tick = 10 seconds timeout
        const MAX_AGE_TICKS: u8 = 10;
        for entry in self.entries.iter_mut() {
            if entry.active && !entry.is_complete() {
                entry.age = entry.age.saturating_add(1);
                if entry.age >= MAX_AGE_TICKS {
                    log::debug!(
                        "[APS frag] Expiring stale reassembly: src=0x{:04X} counter={} ({}/{} blocks)",
                        entry.src_addr,
                        entry.aps_counter,
                        entry.received_mask.count_ones(),
                        entry.total_blocks,
                    );
                    entry.active = false;
                }
            }
        }
    }
}
