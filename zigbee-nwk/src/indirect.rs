//! Indirect frame queue for sleeping end device children.

use zigbee_types::ShortAddress;

#[cfg(feature = "router")]
pub const MAX_INDIRECT: usize = 8;
#[cfg(not(feature = "router"))]
pub const MAX_INDIRECT: usize = 0;

const MAX_FRAME_SIZE: usize = 128;

/// A buffered frame waiting for a sleeping child to poll.
pub struct IndirectEntry {
    pub dst_addr: ShortAddress,
    pub frame: [u8; MAX_FRAME_SIZE],
    pub len: usize,
    /// Remaining lifetime in seconds (spec: 7.68s ≈ nwkIndirectPollTimeout)
    pub ttl: u8,
    pub active: bool,
}

impl IndirectEntry {
    fn empty() -> Self {
        Self {
            dst_addr: ShortAddress(0xFFFF),
            frame: [0; MAX_FRAME_SIZE],
            len: 0,
            ttl: 0,
            active: false,
        }
    }
}

pub struct IndirectQueue {
    entries: [IndirectEntry; MAX_INDIRECT],
}

impl IndirectQueue {
    pub fn new() -> Self {
        Self {
            entries: core::array::from_fn(|_| IndirectEntry::empty()),
        }
    }

    /// Buffer a frame for a sleeping child.
    pub fn enqueue(&mut self, dst: ShortAddress, frame: &[u8]) -> bool {
        if frame.len() > MAX_FRAME_SIZE {
            return false;
        }
        let slot = self.entries.iter_mut().find(|e| !e.active);
        if let Some(entry) = slot {
            entry.dst_addr = dst;
            entry.frame[..frame.len()].copy_from_slice(frame);
            entry.len = frame.len();
            entry.ttl = 8; // ~7.68s
            entry.active = true;
            true
        } else {
            false
        }
    }

    /// Dequeue a pending frame for a child (called when child sends Data Request).
    /// Returns (frame_slice, has_more_pending).
    pub fn dequeue(&mut self, child: ShortAddress) -> Option<(&[u8], bool)> {
        let idx = self
            .entries
            .iter()
            .position(|e| e.active && e.dst_addr == child)?;
        self.entries[idx].active = false;
        let has_more = self.entries.iter().any(|e| e.active && e.dst_addr == child);
        Some((&self.entries[idx].frame[..self.entries[idx].len], has_more))
    }

    /// Check if there are pending frames for a child.
    pub fn has_pending(&self, child: ShortAddress) -> bool {
        self.entries.iter().any(|e| e.active && e.dst_addr == child)
    }

    /// Age entries. Call every second.
    pub fn age(&mut self) {
        for e in self.entries.iter_mut() {
            if e.active {
                e.ttl = e.ttl.saturating_sub(1);
                if e.ttl == 0 {
                    e.active = false;
                }
            }
        }
    }
}

impl Default for IndirectQueue {
    fn default() -> Self {
        Self::new()
    }
}
