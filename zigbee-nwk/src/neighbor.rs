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
    /// Incoming cost (1-7) — this device's own estimate of the cost of the
    /// link *from* this neighbor, derived from [`Self::lqi`] (R22 §3.6.3.1).
    ///
    /// This is the value advertised in the incoming-cost field of a Link
    /// Status entry naming this neighbor (R22 §3.4.8.3.2).
    pub incoming_cost: u8,
    /// Outgoing cost (0-7) — the cost of the link *to* this neighbor, as
    /// measured by the neighbor itself (R22 §3.6.1.5).
    ///
    /// Only ever written from a received Link Status command that names this
    /// device, or reset to `0` by router aging. `0` means "no Link Status
    /// listing this device has been received", which R22 §3.6.3.5.2 treats as
    /// "this link may not be used for many-to-one or symmetric-link routing".
    pub outgoing_cost: u8,
    /// Number of `nwkLinkStatusPeriod` intervals since the last Link Status
    /// command frame was received from this neighbor (R22 §3.6.1.5), saturated
    /// at [`ROUTER_AGE_LIMIT`] + 1.
    ///
    /// Separate from [`Self::age`], which counts seconds and drives neighbor
    /// eviction ordering and provisional-child expiry.
    pub link_status_age: u8,
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
    /// This end-device child still has to be named in a R22 Parent Announce
    /// (§2.4.3.1.12).
    ///
    /// Set for every end-device child when the `apsParentAnnounceTimer` first
    /// expires after a reboot ("construct the message"; the spec explicitly
    /// ignores Keepalive Received at that point), cleared as each child is
    /// packed into a broadcast chunk. A keepalive received *after* that
    /// construction also clears it, which is R22's "if the device must send
    /// multiple Parent_annce messages but receives a keepalive from an end
    /// device before it has sent the message, it shall not include that device
    /// in the message".
    ///
    /// Keeping the pending set as one bit per neighbour avoids a second
    /// 32×8-byte IEEE snapshot buffer in a router build, and gating it on the
    /// `router` feature keeps an end-device neighbour table exactly the size
    /// it was: only a parent ever sends a Parent Announce.
    #[cfg(feature = "router")]
    pub parent_annce_pending: bool,
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
            incoming_cost: link_cost_from_lqi(0),
            outgoing_cost: 0,
            link_status_age: 0,
            depth: 0,
            permit_joining: false,
            age: 0,
            end_device_timeout: ED_TIMEOUT_ENUM_DEFAULT,
            keepalive_remaining_secs: 0,
            keepalive_confirmed: false,
            #[cfg(feature = "router")]
            parent_annce_pending: false,
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

    /// Calculate the incoming link cost from LQI (R22 §3.6.3.1).
    ///
    /// Only the *incoming* cost is a local measurement. The outgoing cost is
    /// the neighbor's own measurement and may only be written by Link Status
    /// receive processing (R22 §3.6.3.4.2) or cleared by router aging
    /// (§3.6.3.4.3).
    pub fn update_incoming_cost_from_lqi(&mut self) {
        self.incoming_cost = link_cost_from_lqi(self.lqi);
    }

    /// Fold a freshly received frame's LQI into the rolling average and
    /// refresh the incoming link cost.
    ///
    /// R22 §3.6.1.5 requires a neighbor table entry to be updated on every
    /// frame received from that neighbor; §3.6.3.1 requires the initial cost
    /// estimate to be based on *average* LQI rather than a single sample.
    pub fn observe_frame(&mut self, lqi: u8) {
        self.lqi = if self.lqi == 0 {
            lqi
        } else {
            (((self.lqi as u16) * 3 + lqi as u16) / 4) as u8
        };
        self.update_incoming_cost_from_lqi();
    }

    /// Whether this neighbor is a router or the coordinator, i.e. a device
    /// that participates in routing and exchanges Link Status frames.
    pub fn is_router(&self) -> bool {
        matches!(
            self.device_type,
            NeighborDeviceType::Router | NeighborDeviceType::Coordinator
        )
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

/// `nwkLinkStatusPeriod` — seconds between Link Status command frames
/// (R22 Table 3-58 default `0x0f`).
pub const LINK_STATUS_PERIOD_SECS: u16 = 15;

/// `nwkRouterAgeLimit` — missed Link Status frames after which a router
/// neighbor's outgoing cost is discarded (R22 Table 3-58 default `3`).
pub const ROUTER_AGE_LIMIT: u8 = 3;

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

    /// Age router neighbors by one `nwkLinkStatusPeriod` (R22 §3.6.3.4.3).
    ///
    /// A router that fails to send [`ROUTER_AGE_LIMIT`] Link Status frames in
    /// a row has its outgoing cost discarded: the reverse-direction cost we
    /// hold for it is no longer evidence of a working link, so it must not
    /// keep feeding many-to-one or symmetric-link route decisions. End devices
    /// never issue Link Status frames and are therefore never aged this way.
    pub fn age_router_link_status(&mut self) {
        for entry in self
            .entries
            .iter_mut()
            .filter(|entry| entry.active && entry.is_router())
        {
            entry.link_status_age = entry
                .link_status_age
                .saturating_add(1)
                .min(ROUTER_AGE_LIMIT.saturating_add(1));
            if entry.link_status_age > ROUTER_AGE_LIMIT && entry.outgoing_cost != 0 {
                log::debug!(
                    "[NWK] Router neighbor 0x{:04X} stale after {} missed link status frames",
                    entry.network_address.0,
                    entry.link_status_age,
                );
                entry.outgoing_cost = 0;
            }
        }
    }

    /// Collect this device's router neighbors, sorted ascending by network
    /// address as R22 §3.4.8.3.2 requires of the link status list.
    ///
    /// The list is built by insertion rather than by sorting afterwards:
    /// `[T]::sort_unstable_by_key` instantiates the generic pattern-defeating
    /// quicksort, which costs several kilobytes of flash — far more than the
    /// whole link status path — for a list bounded by
    /// [`MAX_NEIGHBORS`] entries.
    pub fn router_link_status_entries(
        &self,
    ) -> heapless::Vec<crate::frames::LinkStatusEntry, MAX_NEIGHBORS> {
        let mut entries: heapless::Vec<crate::frames::LinkStatusEntry, MAX_NEIGHBORS> =
            heapless::Vec::new();
        for neighbor in self
            .iter()
            .filter(|neighbor| neighbor.is_router() && neighbor.network_address.0 < 0xFFF8)
        {
            let entry = crate::frames::LinkStatusEntry {
                address: neighbor.network_address,
                // R22 §3.4.8.3.2: our own estimate of the link from them, and
                // the outgoing cost field straight out of the neighbor table.
                incoming_cost: neighbor.incoming_cost.clamp(1, 7),
                outgoing_cost: neighbor.outgoing_cost,
            };
            let position = entries
                .iter()
                .position(|existing| existing.address.0 > entry.address.0)
                .unwrap_or(entries.len());
            if entries.push(entry).is_err() {
                break;
            }
            // Shift the tail up by one to open `position`.
            let mut index = entries.len() - 1;
            while index > position {
                entries.swap(index, index - 1);
                index -= 1;
            }
        }
        entries
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
