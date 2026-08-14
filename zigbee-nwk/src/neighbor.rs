//! NWK neighbor table.
//!
//! Tracks all known neighbors (parents, children, siblings) with
//! their addresses, relationship, LQI, and aging information.

use crate::frames::{ED_TIMEOUT_ENUM_DEFAULT, ed_timeout_enum_to_seconds};
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
    /// Security-capable bit supplied by the device during admission.
    pub security_capable: bool,
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
    /// R22 accepted End Device Timeout enumeration (0..=14) for an end-device
    /// child. Only meaningful when this entry is an authenticated end-device
    /// [`Relationship::Child`]; other entries leave it at the default.
    ///
    /// A child that never negotiates keeps [`ED_TIMEOUT_ENUM_DEFAULT`] (8), the
    /// value a R22 parent applies by default, so a missing negotiation can
    /// never grant a longer child lifetime than the parent actually intended.
    pub end_device_timeout: u8,
    /// R22 End Device Timeout deadline countdown, in whole seconds.
    ///
    /// Armed to `ed_timeout_enum_to_seconds(end_device_timeout)` when an
    /// end-device child becomes authenticated and reset on every keepalive the
    /// parent advertises (MAC Data Poll, End Device Timeout Request) plus valid
    /// secured traffic. Aged down each tick with saturating subtraction; when
    /// it reaches `0` the child is evicted. `u32` because enum 14 is
    /// ≈ 11 days (983040 s), which overflows the `u16` [`Self::age`] counter.
    /// `0` on a non-child / unauthenticated entry means "not armed" and is
    /// never aged.
    pub keepalive_remaining_secs: u32,
    /// Whether this child has proven liveness in the current power cycle.
    ///
    /// `true` for a child admitted or kept alive at runtime (association,
    /// rejoin, key proof, MAC poll, End Device Timeout Request, valid secured
    /// traffic); `false` for a child re-installed from durable persistence
    /// that has not yet been heard from. Used by Parent Announce so a router
    /// yields an unconfirmed restored child to whichever parent it actually
    /// keeps alive with, and defends a child it is actively parenting.
    pub keepalive_confirmed: bool,
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
            security_capable: false,
            relationship: Relationship::Sibling,
            lqi: 0,
            outgoing_cost: 7,
            depth: 0,
            permit_joining: false,
            age: 0,
            end_device_timeout: ED_TIMEOUT_ENUM_DEFAULT,
            keepalive_remaining_secs: 0,
            keepalive_confirmed: false,
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

    /// Calculate outgoing cost from LQI (Zigbee spec Section 3.6.3.1).
    pub fn update_cost_from_lqi(&mut self) {
        self.outgoing_cost = link_cost_from_lqi(self.lqi);
    }

    /// Re-arm the R22 End Device Timeout deadline from the stored enumeration.
    ///
    /// Called when an end-device child is authenticated and on every keepalive
    /// it performs. Falls back to the default enumeration's window if the
    /// stored enumeration is somehow undefined, so an armed child always has a
    /// finite, positive deadline rather than an immediate eviction.
    pub fn refresh_end_device_timeout(&mut self) {
        self.keepalive_remaining_secs = ed_timeout_enum_to_seconds(self.end_device_timeout)
            .or_else(|| ed_timeout_enum_to_seconds(ED_TIMEOUT_ENUM_DEFAULT))
            .unwrap_or(1);
    }
}

/// Convert a received-frame LQI into the bounded Zigbee link cost used by
/// route discovery and by parent selection (R22 §3.6.3.1).
///
/// The result is always in `1..=7`; `1` is the best link and `7` the worst.
pub const fn link_cost_from_lqi(lqi: u8) -> u8 {
    match lqi {
        0..=50 => 7,
        51..=100 => 5,
        101..=150 => 3,
        151..=200 => 2,
        201..=255 => 1,
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

    /// Find neighbor by IEEE address (mutable).
    pub fn find_by_ieee_mut(&mut self, addr: &IeeeAddress) -> Option<&mut NeighborEntry> {
        self.entries[..self.count]
            .iter_mut()
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
        self.entries[..self.count].iter().filter(|e| {
            e.active
                && matches!(
                    e.relationship,
                    Relationship::Child | Relationship::UnauthenticatedChild
                )
        })
    }

    /// Whether a child can be inserted without evicting a parent or child.
    pub fn has_child_slot(&self) -> bool {
        self.entries.iter().any(|entry| !entry.active)
            || self.entries.iter().any(|entry| {
                entry.active
                    && !matches!(
                        entry.relationship,
                        Relationship::Parent
                            | Relationship::Child
                            | Relationship::UnauthenticatedChild
                    )
            })
    }

    /// Add or update a neighbor entry. Returns Ok if added, Err if table full.
    #[allow(clippy::result_unit_err)]
    pub fn add_or_update(&mut self, entry: NeighborEntry) -> Result<(), ()> {
        // Both addresses are unique identifiers. Merge any stale entries that
        // match either one so IEEE and short-address lookups cannot disagree.
        let mut existing_index = None;
        for index in 0..self.count {
            if self.entries[index].active
                && (self.entries[index].network_address == entry.network_address
                    || self.entries[index].ieee_address == entry.ieee_address)
            {
                if existing_index.is_none() {
                    existing_index = Some(index);
                } else {
                    self.entries[index].active = false;
                }
            }
        }
        if let Some(index) = existing_index {
            self.entries[index] = entry;
            self.entries[index].active = true;
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
            // Table full — evict only entries outside an active
            // parent/child authorization relationship.
            if let Some(victim) = self
                .entries
                .iter_mut()
                .filter(|e| {
                    e.active
                        && !matches!(
                            e.relationship,
                            Relationship::Parent
                                | Relationship::Child
                                | Relationship::UnauthenticatedChild
                        )
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

    /// Remove provisional children that did not prove network-key possession.
    pub fn expire_unauthenticated(
        &mut self,
        max_age: u16,
    ) -> heapless::Vec<ShortAddress, MAX_NEIGHBORS> {
        let mut expired = heapless::Vec::new();
        for entry in self.entries.iter_mut().filter(|entry| {
            entry.active
                && entry.relationship == Relationship::UnauthenticatedChild
                && entry.age >= max_age
        }) {
            let _ = expired.push(entry.network_address);
            entry.active = false;
        }
        expired
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

#[cfg(all(test, feature = "router"))]
mod tests {
    use super::*;

    fn entry(index: usize, relationship: Relationship) -> NeighborEntry {
        let mut entry =
            NeighborEntry::new_from_annce(ShortAddress((index + 1) as u16), [(index + 1) as u8; 8]);
        entry.relationship = relationship;
        entry.age = index as u16;
        entry
    }

    #[test]
    fn provisional_child_is_not_evicted_from_a_full_table() {
        let mut table = NeighborTable::new();
        table
            .add_or_update(entry(0, Relationship::UnauthenticatedChild))
            .unwrap();
        for index in 1..MAX_NEIGHBORS {
            table
                .add_or_update(entry(index, Relationship::Child))
                .unwrap();
        }

        assert!(
            table
                .add_or_update(entry(MAX_NEIGHBORS, Relationship::Sibling))
                .is_err()
        );
        assert_eq!(
            table.find_by_short(ShortAddress(1)).unwrap().relationship,
            Relationship::UnauthenticatedChild
        );
    }

    #[test]
    fn ieee_address_update_reuses_the_existing_neighbor_slot() {
        let mut table = NeighborTable::new();
        let announced = entry(0, Relationship::Sibling);
        let ieee = announced.ieee_address;
        table.add_or_update(announced).unwrap();

        let mut child = entry(1, Relationship::UnauthenticatedChild);
        child.ieee_address = ieee;
        table.add_or_update(child).unwrap();

        assert_eq!(table.len(), 1);
        assert!(table.find_by_short(ShortAddress(1)).is_none());
        let found = table.find_by_ieee(&ieee).unwrap();
        assert_eq!(found.network_address, ShortAddress(2));
        assert_eq!(found.relationship, Relationship::UnauthenticatedChild);
    }
}
