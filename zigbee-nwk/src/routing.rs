//! NWK routing table and route discovery (AODV mesh routing).

use zigbee_types::ShortAddress;

/// Maximum routing table entries
#[cfg(feature = "router")]
pub const MAX_ROUTES: usize = 32;
#[cfg(not(feature = "router"))]
pub const MAX_ROUTES: usize = 0;

/// Maximum pending route discoveries
#[cfg(feature = "router")]
pub const MAX_ROUTE_DISCOVERIES: usize = 8;
#[cfg(not(feature = "router"))]
pub const MAX_ROUTE_DISCOVERIES: usize = 0;

/// Route table entry status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteStatus {
    Active,
    DiscoveryUnderway,
    DiscoveryFailed,
    Inactive,
    ValidationUnderway,
}

/// A single routing table entry (Zigbee spec Table 3-62)
#[derive(Debug, Clone)]
pub struct RouteEntry {
    pub destination: ShortAddress,
    pub next_hop: ShortAddress,
    pub status: RouteStatus,
    /// Whether the destination is a concentrator (many-to-one)
    pub many_to_one: bool,
    /// Whether a route record is required
    pub route_record_required: bool,
    /// Whether the destination is a group address
    pub group_id: bool,
    /// Path cost (sum of link costs along the route)
    pub path_cost: u8,
    /// Age counter for expiration
    pub age: u16,
    pub active: bool,
}

impl RouteEntry {
    fn empty() -> Self {
        Self {
            destination: ShortAddress(0xFFFF),
            next_hop: ShortAddress(0xFFFF),
            status: RouteStatus::Inactive,
            many_to_one: false,
            route_record_required: false,
            group_id: false,
            path_cost: 0xFF,
            age: 0,
            active: false,
        }
    }
}

/// Pending route discovery
#[derive(Debug, Clone)]
pub struct RouteDiscovery {
    pub request_id: u8,
    pub destination: ShortAddress,
    pub sender: ShortAddress,
    pub forward_cost: u8,
    pub residual_cost: u8,
    pub timestamp: u32,
    pub active: bool,
}

impl RouteDiscovery {
    fn empty() -> Self {
        Self {
            request_id: 0,
            destination: ShortAddress(0xFFFF),
            sender: ShortAddress(0xFFFF),
            forward_cost: 0xFF,
            residual_cost: 0xFF,
            timestamp: 0,
            active: false,
        }
    }
}

/// NWK routing table — supports both tree and AODV mesh routing.
pub struct RoutingTable {
    routes: [RouteEntry; MAX_ROUTES],
    discoveries: [RouteDiscovery; MAX_ROUTE_DISCOVERIES],
}

impl RoutingTable {
    pub fn new() -> Self {
        Self {
            routes: core::array::from_fn(|_| RouteEntry::empty()),
            discoveries: core::array::from_fn(|_| RouteDiscovery::empty()),
        }
    }

    /// Look up the next hop for a destination.
    pub fn next_hop(&self, destination: ShortAddress) -> Option<ShortAddress> {
        self.routes
            .iter()
            .find(|r| r.active && r.destination == destination && r.status == RouteStatus::Active)
            .map(|r| r.next_hop)
    }

    /// Add or update a route entry.
    #[allow(clippy::result_unit_err)]
    pub fn update_route(
        &mut self,
        destination: ShortAddress,
        next_hop: ShortAddress,
        cost: u8,
    ) -> Result<(), ()> {
        // Update existing
        if let Some(entry) = self
            .routes
            .iter_mut()
            .find(|r| r.active && r.destination == destination)
        {
            entry.next_hop = next_hop;
            entry.path_cost = cost;
            entry.status = RouteStatus::Active;
            entry.age = 0;
            return Ok(());
        }

        // Find empty slot
        if let Some(slot) = self.routes.iter_mut().find(|r| !r.active) {
            *slot = RouteEntry {
                destination,
                next_hop,
                status: RouteStatus::Active,
                many_to_one: false,
                route_record_required: false,
                group_id: false,
                path_cost: cost,
                age: 0,
                active: true,
            };
            Ok(())
        } else {
            // Evict oldest inactive or highest-cost route
            if let Some(victim) = self
                .routes
                .iter_mut()
                .filter(|r| r.active && r.status != RouteStatus::Active)
                .max_by_key(|r| r.age)
            {
                *victim = RouteEntry {
                    destination,
                    next_hop,
                    status: RouteStatus::Active,
                    many_to_one: false,
                    route_record_required: false,
                    group_id: false,
                    path_cost: cost,
                    age: 0,
                    active: true,
                };
                Ok(())
            } else {
                Err(())
            }
        }
    }

    /// Mark a route as discovery underway.
    pub fn mark_discovery(&mut self, destination: ShortAddress) {
        if let Some(entry) = self
            .routes
            .iter_mut()
            .find(|r| r.active && r.destination == destination)
        {
            entry.status = RouteStatus::DiscoveryUnderway;
        }
    }

    /// Remove a route.
    pub fn remove(&mut self, destination: ShortAddress) {
        if let Some(entry) = self
            .routes
            .iter_mut()
            .find(|r| r.active && r.destination == destination)
        {
            entry.active = false;
            entry.status = RouteStatus::Inactive;
        }
    }

    /// Add a pending route discovery.
    #[allow(clippy::result_unit_err)]
    pub fn add_discovery(&mut self, discovery: RouteDiscovery) -> Result<(), ()> {
        if let Some(slot) = self.discoveries.iter_mut().find(|d| !d.active) {
            *slot = discovery;
            slot.active = true;
            Ok(())
        } else {
            Err(())
        }
    }

    /// Find a pending route discovery by request ID.
    pub fn find_discovery(&self, request_id: u8) -> Option<&RouteDiscovery> {
        self.discoveries
            .iter()
            .find(|d| d.active && d.request_id == request_id)
    }

    /// Complete a route discovery (remove from pending).
    pub fn complete_discovery(&mut self, request_id: u8) {
        if let Some(d) = self
            .discoveries
            .iter_mut()
            .find(|d| d.active && d.request_id == request_id)
        {
            d.active = false;
        }
    }

    /// Tree routing: calculate next hop using CSkip algorithm.
    ///
    /// For tree topology, the next hop is either:
    /// - Parent (if destination is outside our subtree)
    /// - One of our children (if destination is in their subtree)
    pub fn tree_route(
        &self,
        our_addr: ShortAddress,
        dst_addr: ShortAddress,
        depth: u8,
        max_routers: u8,
        max_depth: u8,
    ) -> Option<ShortAddress> {
        let cskip = cskip_value(depth, max_routers, max_depth);
        if cskip == 0 {
            return None; // Leaf node, must route to parent
        }

        let dst = dst_addr.0 as u32;
        let our = our_addr.0 as u32;

        // Is destination in our address space?
        if dst > our && dst < our + cskip as u32 {
            // Destination is one of our children
            // Find which child subtree it belongs to
            for i in 1..=max_routers as u32 {
                let child_addr = our + (i - 1) * cskip as u32 + 1;
                if dst >= child_addr && dst < child_addr + cskip as u32 {
                    return Some(ShortAddress(child_addr as u16));
                }
            }
            // Must be an end device child (address = our + Rm*Cskip + n)
            Some(ShortAddress(dst as u16))
        } else {
            None // Not in our subtree, route to parent
        }
    }

    /// Age all route entries.
    pub fn age_tick(&mut self) {
        for route in self.routes.iter_mut().filter(|r| r.active) {
            route.age = route.age.saturating_add(1);
        }
    }

    /// Number of active routes.
    pub fn len(&self) -> usize {
        self.routes.iter().filter(|r| r.active).count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Look up a route entry for a destination (read-only).
    pub fn get_entry(&self, destination: ShortAddress) -> Option<&RouteEntry> {
        self.routes
            .iter()
            .find(|r| r.active && r.destination == destination && r.status == RouteStatus::Active)
    }

    /// Add or update a route to a many-to-one concentrator.
    #[allow(clippy::result_unit_err)]
    pub fn update_route_many_to_one(
        &mut self,
        destination: ShortAddress,
        next_hop: ShortAddress,
        cost: u8,
    ) -> Result<(), ()> {
        // Update existing entry
        if let Some(entry) = self
            .routes
            .iter_mut()
            .find(|r| r.active && r.destination == destination)
        {
            entry.next_hop = next_hop;
            entry.path_cost = cost;
            entry.status = RouteStatus::Active;
            entry.many_to_one = true;
            entry.route_record_required = true;
            entry.age = 0;
            return Ok(());
        }

        // Find empty slot
        if let Some(slot) = self.routes.iter_mut().find(|r| !r.active) {
            *slot = RouteEntry {
                destination,
                next_hop,
                status: RouteStatus::Active,
                many_to_one: true,
                route_record_required: true,
                group_id: false,
                path_cost: cost,
                age: 0,
                active: true,
            };
            Ok(())
        } else {
            // Evict oldest inactive
            if let Some(victim) = self
                .routes
                .iter_mut()
                .filter(|r| r.active && r.status != RouteStatus::Active)
                .max_by_key(|r| r.age)
            {
                *victim = RouteEntry {
                    destination,
                    next_hop,
                    status: RouteStatus::Active,
                    many_to_one: true,
                    route_record_required: true,
                    group_id: false,
                    path_cost: cost,
                    age: 0,
                    active: true,
                };
                Ok(())
            } else {
                Err(())
            }
        }
    }

    /// Clear the route_record_required flag for a destination.
    pub fn clear_route_record_required(&mut self, destination: ShortAddress) {
        if let Some(entry) = self
            .routes
            .iter_mut()
            .find(|r| r.active && r.destination == destination)
        {
            entry.route_record_required = false;
        }
    }

    /// Iterate over active route entries.
    pub fn iter(&self) -> impl Iterator<Item = &RouteEntry> {
        self.routes.iter().filter(|r| r.active)
    }

    /// Mark routes with age exceeding `max_age` as [`RouteStatus::Inactive`].
    pub fn expire_stale(&mut self, max_age: u16) {
        for route in self.routes.iter_mut().filter(|r| r.active) {
            if route.age > max_age {
                route.status = RouteStatus::Inactive;
                route.active = false;
                log::debug!(
                    "[NWK] Route to 0x{:04X} expired (age={})",
                    route.destination.0,
                    route.age,
                );
            }
        }
    }

    /// Fail discoveries that have been active longer than `max_age` ticks.
    pub fn expire_discoveries(&mut self, max_age: u32, current_time: u32) {
        for disc in self.discoveries.iter_mut().filter(|d| d.active) {
            let elapsed = current_time.wrapping_sub(disc.timestamp);
            if elapsed > max_age {
                disc.active = false;
                // Update the corresponding route entry to DiscoveryFailed
                if let Some(route) = self.routes.iter_mut().find(|r| {
                    r.destination == disc.destination && r.status == RouteStatus::DiscoveryUnderway
                }) {
                    route.status = RouteStatus::DiscoveryFailed;
                }
                log::debug!(
                    "[NWK] Route discovery for 0x{:04X} timed out (elapsed={})",
                    disc.destination.0,
                    elapsed,
                );
            }
        }
    }
}

/// CSkip value calculation for tree addressing.
/// CSkip(d) = (1 + Cm - Rm - Cm * Rm^(Lm-d-1)) / (1 - Rm) for Rm != 1
fn cskip_value(depth: u8, max_routers: u8, max_depth: u8) -> u16 {
    if depth >= max_depth {
        return 0;
    }
    let rm = max_routers as u32;
    if rm == 1 {
        return 1;
    }
    let cm = 20u32; // max_children default
    let remaining = max_depth as u32 - depth as u32 - 1;
    let rm_pow = rm.pow(remaining);
    let numerator = 1 + cm - rm - cm * rm_pow;
    let denominator = 1u32.wrapping_sub(rm);
    if denominator == 0 {
        return 0;
    }
    (numerator / denominator) as u16
}

impl Default for RoutingTable {
    fn default() -> Self {
        Self::new()
    }
}

// ── Broadcast Transaction Record ──────────────────────────────

/// Maximum BTR entries for broadcast deduplication
#[cfg(feature = "router")]
pub const MAX_BTR: usize = 16;
#[cfg(not(feature = "router"))]
pub const MAX_BTR: usize = 0;

/// Broadcast Transaction Record — tracks seen broadcasts to prevent storms.
/// Spec: Table 3-68.
#[derive(Debug, Clone)]
pub struct BtrEntry {
    pub src_addr: ShortAddress,
    pub seq_number: u8,
    /// Remaining lifetime in seconds (spec default: 9s = nwkNetworkBroadcastDeliveryTime)
    pub expiry: u8,
    pub active: bool,
}

impl BtrEntry {
    fn empty() -> Self {
        Self {
            src_addr: ShortAddress(0xFFFF),
            seq_number: 0,
            expiry: 0,
            active: false,
        }
    }
}

/// BTR table for broadcast relay deduplication.
pub struct BtrTable {
    entries: [BtrEntry; MAX_BTR],
}

impl BtrTable {
    pub fn new() -> Self {
        Self {
            entries: core::array::from_fn(|_| BtrEntry::empty()),
        }
    }

    /// Check if this broadcast was already seen. Returns true if duplicate.
    pub fn is_duplicate(&self, src: ShortAddress, seq: u8) -> bool {
        self.entries
            .iter()
            .any(|e| e.active && e.src_addr == src && e.seq_number == seq)
    }

    /// Record a broadcast (must call after is_duplicate check).
    pub fn record(&mut self, src: ShortAddress, seq: u8) {
        // Find empty slot first
        let mut target_idx = None;
        for (i, e) in self.entries.iter().enumerate() {
            if !e.active {
                target_idx = Some(i);
                break;
            }
        }
        // No empty slot — evict oldest (lowest expiry)
        if target_idx.is_none() {
            let mut min_expiry = u8::MAX;
            for (i, e) in self.entries.iter().enumerate() {
                if e.expiry < min_expiry {
                    min_expiry = e.expiry;
                    target_idx = Some(i);
                }
            }
        }
        if let Some(idx) = target_idx {
            self.entries[idx] = BtrEntry {
                src_addr: src,
                seq_number: seq,
                expiry: 9,
                active: true,
            };
        }
    }

    /// Age entries. Call every second.
    pub fn age(&mut self) {
        for e in self.entries.iter_mut() {
            if e.active {
                e.expiry = e.expiry.saturating_sub(1);
                if e.expiry == 0 {
                    e.active = false;
                }
            }
        }
    }
}

impl Default for BtrTable {
    fn default() -> Self {
        Self::new()
    }
}
