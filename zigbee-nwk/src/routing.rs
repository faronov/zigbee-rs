//! NWK routing table and route discovery (AODV mesh routing).

use zigbee_types::ShortAddress;

/// Maximum routing table entries
pub const MAX_ROUTES: usize = 32;
/// Maximum pending route discoveries
pub const MAX_ROUTE_DISCOVERIES: usize = 8;

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

    /// Iterate over active route entries.
    pub fn iter(&self) -> impl Iterator<Item = &RouteEntry> {
        self.routes.iter().filter(|r| r.active)
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
