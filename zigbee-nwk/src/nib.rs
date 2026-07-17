//! Network Information Base (NIB).
//!
//! The NIB stores all NWK-layer configuration and state.
//! It's the NWK equivalent of the MAC PIB.

use zigbee_types::*;

/// NWK Information Base — all NWK layer state.
#[derive(Debug)]
pub struct Nib {
    // ── Network identity ────────────────────────────────
    /// Extended PAN ID of the network (64-bit)
    pub extended_pan_id: IeeeAddress,
    /// Short (16-bit) PAN ID
    pub pan_id: PanId,
    /// Own network (short) address
    pub network_address: ShortAddress,
    /// Operating channel (11-26)
    pub logical_channel: u8,

    // ── Network parameters ──────────────────────────────
    /// Stack profile: 0x02 = Zigbee PRO
    pub stack_profile: u8,
    /// Network depth of this device
    pub depth: u8,
    /// Maximum depth for the network
    pub max_depth: u8,
    /// Maximum number of child routers
    pub max_routers: u8,
    /// Maximum number of child end devices
    pub max_children: u8,
    /// Network update ID
    pub update_id: u8,
    /// NWK manager address (for frequency agility)
    pub nwk_manager_addr: ShortAddress,

    // ── Addressing ──────────────────────────────────────
    /// Own IEEE (extended) address
    pub ieee_address: IeeeAddress,
    /// Parent's short address
    pub parent_address: ShortAddress,
    /// Short address assignment method
    pub address_assign: AddressAssignMethod,

    // ── Timing ──────────────────────────────────────────
    /// Network broadcast delivery time (in half-seconds)
    pub broadcast_delivery_time: u8,
    /// Passive ack timeout (ms)
    pub passive_ack_timeout: u16,
    /// Max broadcast retries
    pub max_broadcast_retries: u8,
    /// Transaction persistence time (ms)
    pub transaction_persistence_time: u16,

    // ── Routing ─────────────────────────────────────────
    /// Use tree routing (vs mesh-only)
    pub use_tree_routing: bool,
    /// Use source routing
    pub source_routing: bool,
    /// Route discovery retries
    pub route_discovery_retries: u8,

    // ── Security ────────────────────────────────────────
    /// Security level (0=none, 5=ENC-MIC-32, typical for Zigbee)
    pub security_level: u8,
    /// Whether NWK security is enabled
    pub security_enabled: bool,
    /// Active network key index
    pub active_key_seq_number: u8,
    /// NWK frame counter (outgoing)
    pub outgoing_frame_counter: u32,
    /// Exclusive upper bound of the durably reserved outgoing-counter range.
    pub outgoing_frame_counter_limit: u32,

    // ── Sequences ───────────────────────────────────────
    /// NWK sequence number
    pub sequence_number: u8,
    /// Route request ID counter
    pub route_request_id: u8,

    // ── Permit joining ──────────────────────────────────
    /// Whether new devices can join through this device
    pub permit_joining: bool,
    /// Time remaining for permit joining (0 = permanent, 0xFF = permanent)
    pub permit_joining_duration: u8,
}

/// How short addresses are assigned
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressAssignMethod {
    /// Tree-based (CSkip algorithm)
    TreeBased,
    /// Stochastic (random, check for conflicts)
    Stochastic,
}

impl Nib {
    /// Create a new NIB with default values.
    pub fn new() -> Self {
        Self {
            extended_pan_id: [0u8; 8],
            pan_id: PanId(0xFFFF),
            network_address: ShortAddress(0xFFFF),
            logical_channel: 0,
            stack_profile: 0x02, // Zigbee PRO
            depth: 0,
            max_depth: 15,
            max_routers: 5,
            max_children: 20,
            update_id: 0,
            nwk_manager_addr: ShortAddress::COORDINATOR,
            ieee_address: [0u8; 8],
            parent_address: ShortAddress(0xFFFF),
            address_assign: AddressAssignMethod::Stochastic,
            broadcast_delivery_time: 9,
            passive_ack_timeout: 500,
            max_broadcast_retries: 3,
            transaction_persistence_time: 500,
            use_tree_routing: false,
            source_routing: false,
            route_discovery_retries: 3,
            security_level: 5, // ENC-MIC-32 (standard Zigbee)
            // A factory-new device has no active network key. NWK security
            // becomes active only after network formation, restore, or a
            // successful APS Transport-Key exchange.
            security_enabled: false,
            active_key_seq_number: 0,
            outgoing_frame_counter: 0,
            outgoing_frame_counter_limit: u32::MAX,
            sequence_number: 0,
            route_request_id: 0,
            permit_joining: false,
            permit_joining_duration: 0,
        }
    }

    /// Get the next NWK sequence number (wrapping).
    pub fn next_seq(&mut self) -> u8 {
        let seq = self.sequence_number;
        self.sequence_number = self.sequence_number.wrapping_add(1);
        seq
    }

    /// Get the next route request ID.
    pub fn next_route_request_id(&mut self) -> u8 {
        let id = self.route_request_id;
        self.route_request_id = self.route_request_id.wrapping_add(1);
        id
    }

    /// Increment outgoing frame counter. Returns the pre-increment value.
    /// Returns None if the counter has reached the durably reserved limit.
    pub fn next_frame_counter(&mut self) -> Option<u32> {
        if self.outgoing_frame_counter >= self.outgoing_frame_counter_limit {
            log::error!("[NWK] Frame counter reservation exhausted");
            return None;
        }
        let fc = self.outgoing_frame_counter;
        self.outgoing_frame_counter += 1;
        Some(fc)
    }

    /// Install a durably persisted outgoing-counter reservation.
    pub fn set_frame_counter_reservation(&mut self, current: u32, limit: u32) -> bool {
        if current > limit {
            return false;
        }
        self.outgoing_frame_counter = current;
        self.outgoing_frame_counter_limit = limit;
        true
    }
}

impl Default for Nib {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::Nib;

    #[test]
    fn factory_new_nib_starts_without_nwk_security() {
        assert!(!Nib::new().security_enabled);
    }

    #[test]
    fn outgoing_counter_stops_at_reserved_limit() {
        let mut nib = Nib::new();
        assert!(nib.set_frame_counter_reservation(7, 9));
        assert_eq!(nib.next_frame_counter(), Some(7));
        assert_eq!(nib.next_frame_counter(), Some(8));
        assert_eq!(nib.next_frame_counter(), None);
        assert_eq!(nib.outgoing_frame_counter, 9);
    }

    #[test]
    fn outgoing_counter_rejects_invalid_reservation() {
        let mut nib = Nib::new();
        assert!(!nib.set_frame_counter_reservation(10, 9));
        assert_eq!(nib.outgoing_frame_counter, 0);
        assert_eq!(nib.outgoing_frame_counter_limit, u32::MAX);
    }
}
