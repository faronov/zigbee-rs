//! NWK neighbor table.
//!
//! Tracks all known neighbors (parents, children, siblings) with
//! their addresses, relationship, LQI, and aging information.

use zigbee_types::*;

/// Maximum number of neighbors we track
#[cfg(feature = "router")]
pub const MAX_NEIGHBORS: usize = 32;
#[cfg(not(feature = "router"))]
pub const MAX_NEIGHBORS: usize = 8;

/// Relationship with a neighbor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Relationship {
    /// Our parent (coordinator or router we joined through)
    Parent,
    /// Our child (device that joined through us)
    Child,
    /// Sibling (same parent, used for routing)
    Sibling,
    /// Previous child (was our child, now re-joined elsewhere)
    PreviousChild,
    /// Unauthenticated child (joined but not yet authenticated)
    UnauthenticatedChild,
}

/// Device type of a neighbor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NeighborDeviceType {
    Coordinator,
    Router,
    EndDevice,
    Unknown,
}

/// A single neighbor entry
#[derive(Debug, Clone)]
pub struct NeighborEntry {
    /// Extended IEEE address
    pub ieee_address: IeeeAddress,
    /// Short network address
    pub network_address: ShortAddress,
    /// Device type
    pub device_type: NeighborDeviceType,
    /// Rx on when idle (false = sleepy end device)
    pub rx_on_when_idle: bool,
    /// Relationship to us
    pub relationship: Relationship,
    /// Link Quality Indicator (rolling average)
    pub lqi: u8,
    /// Outgoing cost (1-7, derived from LQI)
    pub outgoing_cost: u8,
    /// Network depth of the neighbor
    pub depth: u8,
    /// Permit joining (for routers/coordinator)
    pub permit_joining: bool,
    /// Age counter — incremented on each aging tick, reset on frame receipt
    pub age: u16,
    /// Extended PAN ID of neighbor's network
    pub extended_pan_id: IeeeAddress,
    /// Whether this entry is occupied
    pub active: bool,
}

impl NeighborEntry {
    fn empty() -> Self {
        Self {
            ieee_address: [0; 8],
            network_address: ShortAddress(0xFFFF),
            device_type: NeighborDeviceType::Unknown,
            rx_on_when_idle: false,
            relationship: Relationship::Sibling,
            lqi: 0,
            outgoing_cost: 7,
            depth: 0,
            permit_joining: false,
            age: 0,
            extended_pan_id: [0; 8],
            active: false,
        }
    }

    /// Create a minimal neighbor entry from a Device_annce.
    pub fn new_from_annce(nwk_addr: ShortAddress, ieee_addr: IeeeAddress) -> Self {
        let mut e = Self::empty();
        e.network_address = nwk_addr;
        e.ieee_address = ieee_addr;
        e.active = true;
        e
    }

    /// Calculate outgoing cost from LQI (Zigbee spec Section 3.6.3.1)
    pub fn update_cost_from_lqi(&mut self) {
        self.outgoing_cost = match self.lqi {
            0..=50 => 7,
            51..=100 => 5,
            101..=150 => 3,
            151..=200 => 2,
            201..=255 => 1,
        };
    }
}

/// NWK neighbor table
pub struct NeighborTable {
    entries: [NeighborEntry; MAX_NEIGHBORS],
    count: usize,
}

impl NeighborTable {
    pub fn new() -> Self {
        Self {
            entries: core::array::from_fn(|_| NeighborEntry::empty()),
            count: 0,
        }
    }

    /// Find neighbor by short address.
    pub fn find_by_short(&self, addr: ShortAddress) -> Option<&NeighborEntry> {
        self.entries[..self.count]
            .iter()
            .find(|e| e.active && e.network_address == addr)
    }

    /// Find neighbor by IEEE address.
    pub fn find_by_ieee(&self, addr: &IeeeAddress) -> Option<&NeighborEntry> {
        self.entries[..self.count]
            .iter()
            .find(|e| e.active && e.ieee_address == *addr)
    }

    /// Find neighbor by short address (mutable).
    pub fn find_by_short_mut(&mut self, addr: ShortAddress) -> Option<&mut NeighborEntry> {
        self.entries[..self.count]
            .iter_mut()
            .find(|e| e.active && e.network_address == addr)
    }

    /// Get our parent entry.
    pub fn parent(&self) -> Option<&NeighborEntry> {
        self.entries[..self.count]
            .iter()
            .find(|e| e.active && e.relationship == Relationship::Parent)
    }

    /// Get all children.
    pub fn children(&self) -> impl Iterator<Item = &NeighborEntry> {
        self.entries[..self.count]
            .iter()
            .filter(|e| e.active && e.relationship == Relationship::Child)
    }

    /// Add or update a neighbor entry. Returns Ok if added, Err if table full.
    #[allow(clippy::result_unit_err)]
    pub fn add_or_update(&mut self, entry: NeighborEntry) -> Result<(), ()> {
        // Check if already present
        if let Some(existing) = self.entries[..self.count]
            .iter_mut()
            .find(|e| e.active && e.network_address == entry.network_address)
        {
            *existing = entry;
            existing.active = true;
            return Ok(());
        }

        // Find empty slot
        if let Some(slot) = self.entries.iter_mut().find(|e| !e.active) {
            *slot = entry;
            slot.active = true;
            if self.count < MAX_NEIGHBORS {
                self.count += 1;
            }
            Ok(())
        } else {
            // Table full — try to evict oldest non-parent, non-child
            if let Some(victim) = self
                .entries
                .iter_mut()
                .filter(|e| {
                    e.active
                        && !matches!(e.relationship, Relationship::Parent | Relationship::Child)
                })
                .max_by_key(|e| e.age)
            {
                *victim = entry;
                victim.active = true;
                Ok(())
            } else {
                Err(())
            }
        }
    }

    /// Remove a neighbor by short address.
    pub fn remove(&mut self, addr: ShortAddress) {
        if let Some(entry) = self
            .entries
            .iter_mut()
            .find(|e| e.active && e.network_address == addr)
        {
            entry.active = false;
        }
    }

    /// Age all entries. Called periodically to expire stale neighbors.
    pub fn age_tick(&mut self) {
        for entry in self.entries.iter_mut().filter(|e| e.active) {
            entry.age = entry.age.saturating_add(1);
        }
    }

    /// Number of active entries.
    pub fn len(&self) -> usize {
        self.entries.iter().filter(|e| e.active).count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Iterate all active neighbors.
    pub fn iter(&self) -> impl Iterator<Item = &NeighborEntry> {
        self.entries.iter().filter(|e| e.active)
    }

    /// Iterate all active neighbors (mutable).
    pub fn iter_mut_all(&mut self) -> impl Iterator<Item = &mut NeighborEntry> {
        self.entries.iter_mut().filter(|e| e.active)
    }
}

impl Default for NeighborTable {
    fn default() -> Self {
        Self::new()
    }
}
