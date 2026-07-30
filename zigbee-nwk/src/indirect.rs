//! Indirect frame queue for sleeping end device children.
//!
//! Frames are retained until the matching child polls or the bounded
//! transaction lifetime expires. The parent runtime synchronizes this queue
//! with the MAC ACK Frame Pending table.

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
    order: u32,
    pub active: bool,
}

impl IndirectEntry {
    fn empty() -> Self {
        Self {
            dst_addr: ShortAddress(0xFFFF),
            frame: [0; MAX_FRAME_SIZE],
            len: 0,
            ttl: 0,
            order: 0,
            active: false,
        }
    }
}

pub struct IndirectQueue {
    entries: [IndirectEntry; MAX_INDIRECT],
    next_order: u32,
}

/// Owned snapshot of the oldest frame waiting for one child.
pub struct PendingIndirectFrame {
    frame: [u8; MAX_FRAME_SIZE],
    len: usize,
}

impl PendingIndirectFrame {
    pub fn as_slice(&self) -> &[u8] {
        &self.frame[..self.len]
    }
}

impl IndirectQueue {
    pub fn new() -> Self {
        Self {
            entries: core::array::from_fn(|_| IndirectEntry::empty()),
            next_order: 0,
        }
    }

    /// Buffer a frame for a sleeping child.
    pub fn enqueue(&mut self, dst: ShortAddress, frame: &[u8]) -> bool {
        self.enqueue_with_slot(dst, frame).is_some()
    }

    pub(crate) fn enqueue_with_slot(&mut self, dst: ShortAddress, frame: &[u8]) -> Option<usize> {
        if frame.len() > MAX_FRAME_SIZE {
            return None;
        }
        let index = self.entries.iter().position(|entry| !entry.active)?;
        let entry = &mut self.entries[index];
        entry.dst_addr = dst;
        entry.frame[..frame.len()].copy_from_slice(frame);
        entry.len = frame.len();
        entry.ttl = 8; // ~7.68s
        entry.order = self.next_order;
        self.next_order = self.next_order.wrapping_add(1);
        entry.active = true;
        Some(index)
    }

    fn oldest_index(&self, child: ShortAddress) -> Option<usize> {
        self.entries
            .iter()
            .enumerate()
            .filter(|(_, entry)| entry.active && entry.dst_addr == child)
            .max_by_key(|(_, entry)| self.next_order.wrapping_sub(entry.order))
            .map(|(index, _)| index)
    }

    pub(crate) fn remove_slot(&mut self, index: usize) {
        if let Some(entry) = self.entries.get_mut(index) {
            entry.active = false;
        }
    }

    /// Dequeue a pending frame for a child (called when child sends Data Request).
    /// Returns (frame_slice, has_more_pending).
    pub fn dequeue(&mut self, child: ShortAddress) -> Option<(&[u8], bool)> {
        let idx = self.oldest_index(child)?;
        self.entries[idx].active = false;
        let has_more = self.entries.iter().any(|e| e.active && e.dst_addr == child);
        Some((&self.entries[idx].frame[..self.entries[idx].len], has_more))
    }

    /// Copy the oldest pending frame without removing it.
    ///
    /// Delivery removes the entry only after the MAC confirms transmission;
    /// failed sends therefore remain available for the child's next poll.
    pub fn peek(&self, child: ShortAddress) -> Option<PendingIndirectFrame> {
        let entry = &self.entries[self.oldest_index(child)?];
        let mut frame = [0u8; MAX_FRAME_SIZE];
        frame[..entry.len].copy_from_slice(&entry.frame[..entry.len]);
        Some(PendingIndirectFrame {
            frame,
            len: entry.len,
        })
    }

    /// Complete one successfully transmitted transaction.
    pub fn complete_one(&mut self, child: ShortAddress) -> Option<bool> {
        let index = self.oldest_index(child)?;
        self.entries[index].active = false;
        Some(self.has_pending(child))
    }

    /// Check if there are pending frames for a child.
    pub fn has_pending(&self, child: ShortAddress) -> bool {
        self.entries.iter().any(|e| e.active && e.dst_addr == child)
    }

    /// Count pending frames for one child without modifying FIFO order.
    pub fn pending_count(&self, child: ShortAddress) -> usize {
        self.entries
            .iter()
            .filter(|entry| entry.active && entry.dst_addr == child)
            .count()
    }

    /// Remove every queued transaction for one child.
    pub fn remove_all(&mut self, child: ShortAddress) {
        for entry in self
            .entries
            .iter_mut()
            .filter(|entry| entry.active && entry.dst_addr == child)
        {
            entry.active = false;
        }
    }

    /// Iterate child addresses that currently have queued transactions.
    ///
    /// A child may appear more than once; beacon construction de-duplicates
    /// into its smaller bounded Pending Address list.
    pub fn pending_children(&self) -> impl Iterator<Item = ShortAddress> + '_ {
        self.entries
            .iter()
            .filter(|entry| entry.active)
            .map(|entry| entry.dst_addr)
    }

    /// Age entries. Call every second.
    pub fn age(&mut self) -> heapless::Vec<ShortAddress, MAX_INDIRECT> {
        let mut expired_children = heapless::Vec::new();
        for e in self.entries.iter_mut() {
            if e.active {
                e.ttl = e.ttl.saturating_sub(1);
                if e.ttl == 0 {
                    e.active = false;
                    if !expired_children.contains(&e.dst_addr) {
                        let _ = expired_children.push(e.dst_addr);
                    }
                }
            }
        }
        expired_children
    }
}

impl Default for IndirectQueue {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, feature = "router"))]
mod tests {
    use super::*;

    #[test]
    fn slot_reuse_does_not_reorder_frames_for_one_child() {
        let child = ShortAddress(0x1234);
        let mut queue = IndirectQueue::new();
        assert!(queue.enqueue(child, &[1]));
        assert!(queue.enqueue(child, &[2]));
        assert_eq!(queue.peek(child).unwrap().as_slice(), &[1]);
        assert_eq!(queue.complete_one(child), Some(true));

        // Reuses the lower-numbered array slot freed by frame 1.
        assert!(queue.enqueue(child, &[3]));
        assert_eq!(
            queue.peek(child).unwrap().as_slice(),
            &[2],
            "array slot order must not overtake FIFO order"
        );
        assert_eq!(queue.complete_one(child), Some(true));
        assert_eq!(queue.peek(child).unwrap().as_slice(), &[3]);
    }

    #[test]
    fn pending_count_is_scoped_to_one_child() {
        let child = ShortAddress(0x1234);
        let other = ShortAddress(0x5678);
        let mut queue = IndirectQueue::new();
        assert!(queue.enqueue(child, &[1]));
        assert!(queue.enqueue(other, &[2]));
        assert!(queue.enqueue(child, &[3]));

        assert_eq!(queue.pending_count(child), 2);
        assert_eq!(queue.pending_count(other), 1);
        assert_eq!(queue.pending_count(ShortAddress(0x9ABC)), 0);
    }
}
