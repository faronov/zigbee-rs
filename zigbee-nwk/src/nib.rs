//! Network Information Base (NIB).
//!
//! The NIB stores all NWK-layer configuration and state.
//! It's the NWK equivalent of the MAC PIB.

use crate::frames::{
    ED_TIMEOUT_ENUM_DEFAULT, ED_TIMEOUT_ENUM_MAX, ED_TIMEOUT_ENUM_REQUESTED, PARENT_INFO_MASK,
};
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
    /// Time remaining for permit joining (0 = closed, 0xFF = indefinite)
    pub permit_joining_duration: u8,

    // ── R22 End Device Timeout (client side) ────────────
    /// `nwkParentInformation` — keepalive methods the current parent
    /// advertised in its End Device Timeout Response.
    ///
    /// Only [`PARENT_INFO_MASK`] bits are ever stored. The value is
    /// meaningful only while [`Self::parent_information_valid`] is set; a
    /// valid value of 0 means the parent answered but advertised no keepalive
    /// method (pre-R22 behaviour), which implies MAC Data Poll keepalive.
    pub parent_information: u8,
    /// Whether [`Self::parent_information`] describes the current parent.
    ///
    /// Cleared at every authoritative parent (re)assignment and parent loss,
    /// so a stale advertisement from a previous parent can never select the
    /// keepalive method for a new one.
    pub parent_information_valid: bool,
    /// `nwkEndDeviceTimeout` — the timeout enumeration currently in effect.
    ///
    /// Starts at [`ED_TIMEOUT_ENUM_DEFAULT`] (the value a R22 parent applies
    /// to a child that never negotiated) and only moves to the requested
    /// enumeration once the parent answered SUCCESS.
    pub end_device_timeout: u8,
    /// Timeout enumeration carried by the next End Device Timeout Request.
    ///
    /// Set to [`ED_TIMEOUT_ENUM_REQUESTED`] at every fresh join or secured
    /// rejoin and walked down towards [`ED_TIMEOUT_ENUM_DEFAULT`] — never
    /// below — when the parent answers `INCORRECT_VALUE`.
    pub requested_end_device_timeout: u8,
    /// Wrapping count of accepted End Device Timeout Responses.
    ///
    /// Not a spec attribute: it exists so the layer above can tell "the parent
    /// accepted the request again" from "no response arrived" by comparing the
    /// negotiation fields around `process_incoming_nwk_frame`. A recurring
    /// keepalive to an already-negotiated parent changes nothing else, so
    /// without this the response wait could never be cancelled and every
    /// keepalive would be retransmitted needlessly.
    pub end_device_timeout_accepts: u8,
    /// Whether a successfully transmitted End Device Timeout Request is still
    /// awaiting one valid response from the current parent.
    ///
    /// This is runtime-only state. It is cleared on parent changes, restore,
    /// cancellation, and after the first SUCCESS or INCORRECT_VALUE response,
    /// so an unsolicited or delayed duplicate response cannot mutate the NIB.
    pub(crate) end_device_timeout_response_pending: bool,
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
            parent_information: 0,
            parent_information_valid: false,
            end_device_timeout: ED_TIMEOUT_ENUM_DEFAULT,
            requested_end_device_timeout: ED_TIMEOUT_ENUM_DEFAULT,
            end_device_timeout_accepts: 0,
            end_device_timeout_response_pending: false,
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

    // ── R22 End Device Timeout (client side) ────────────────

    /// Start a fresh End Device Timeout negotiation with a new parent.
    ///
    /// Called from the authoritative NWK parent-assignment and parent-loss
    /// points (association success, secured rejoin success, and a full leave)
    /// rather than from a runtime wrapper, so a parent change can never leave
    /// the previous parent's advertised keepalive method in place.
    ///
    /// The requested enumeration goes back up to [`ED_TIMEOUT_ENUM_REQUESTED`]
    /// and the in-effect timeout drops to [`ED_TIMEOUT_ENUM_DEFAULT`]: until
    /// the new parent answers, only the value a R22 parent applies by default
    /// is safe to keep alive against.
    pub fn reset_end_device_timeout_negotiation(&mut self) {
        self.parent_information = 0;
        self.parent_information_valid = false;
        self.end_device_timeout = ED_TIMEOUT_ENUM_DEFAULT;
        self.requested_end_device_timeout = ED_TIMEOUT_ENUM_REQUESTED;
        self.end_device_timeout_response_pending = false;
    }

    /// Install a persisted End Device Timeout negotiation result.
    ///
    /// Rejects an out-of-range enumeration, reserved `nwkParentInformation`
    /// bits, and information that claims to be valid while it is not, so a
    /// corrupt durable record cannot install an impossible keepalive policy.
    ///
    /// A record that already carries an accepted timeout re-requests the same
    /// enumeration, so a keepalive request cannot silently renegotiate the
    /// parent's child lifetime downwards. A record whose negotiation never
    /// succeeded — including one migrated from a persistence format that
    /// predates these fields — asks for [`ED_TIMEOUT_ENUM_REQUESTED`] again.
    pub fn restore_end_device_timeout(
        &mut self,
        parent_information: u8,
        parent_information_valid: bool,
        end_device_timeout: u8,
    ) -> bool {
        if end_device_timeout > ED_TIMEOUT_ENUM_MAX
            || parent_information & !PARENT_INFO_MASK != 0
            || (!parent_information_valid && parent_information != 0)
        {
            return false;
        }
        self.parent_information = parent_information;
        self.parent_information_valid = parent_information_valid;
        self.end_device_timeout = end_device_timeout;
        self.requested_end_device_timeout = if parent_information_valid {
            end_device_timeout
        } else {
            ED_TIMEOUT_ENUM_REQUESTED
        };
        self.end_device_timeout_response_pending = false;
        true
    }

    pub(crate) fn mark_end_device_timeout_response_pending(&mut self) {
        self.end_device_timeout_response_pending = true;
    }

    pub(crate) fn take_end_device_timeout_response_pending(&mut self) -> bool {
        core::mem::take(&mut self.end_device_timeout_response_pending)
    }

    pub(crate) fn clear_end_device_timeout_response_pending(&mut self) {
        self.end_device_timeout_response_pending = false;
    }

    /// Apply a SUCCESS End Device Timeout Response from the current parent.
    ///
    /// The parent accepted [`Self::requested_end_device_timeout`], so that
    /// enumeration becomes the timeout in effect and the advertised keepalive
    /// methods become authoritative.
    pub(crate) fn accept_end_device_timeout(&mut self, parent_information: u8) {
        self.parent_information = parent_information & PARENT_INFO_MASK;
        self.parent_information_valid = true;
        self.end_device_timeout = self.requested_end_device_timeout;
        // Observable even when nothing else moved, which is the normal case
        // for a recurring keepalive to an already-negotiated parent.
        self.end_device_timeout_accepts = self.end_device_timeout_accepts.wrapping_add(1);
    }

    /// Apply an INCORRECT_VALUE End Device Timeout Response.
    ///
    /// Walks the requested enumeration one step down, never below
    /// [`ED_TIMEOUT_ENUM_DEFAULT`], and never touches previously validated
    /// parent information — a refusal says nothing about the parent's
    /// keepalive capabilities. Returns whether a lower value is now pending.
    pub(crate) fn lower_requested_end_device_timeout(&mut self) -> bool {
        if self.requested_end_device_timeout <= ED_TIMEOUT_ENUM_DEFAULT {
            return false;
        }
        self.requested_end_device_timeout -= 1;
        true
    }

    /// Seconds of the timeout enumeration currently in effect.
    ///
    /// Falls back to [`ED_TIMEOUT_ENUM_DEFAULT`] if the stored enumeration is
    /// somehow undefined, so a keepalive deadline always exists.
    pub fn end_device_timeout_seconds(&self) -> u32 {
        crate::frames::ed_timeout_enum_to_seconds(self.end_device_timeout).unwrap_or_else(|| {
            crate::frames::ed_timeout_enum_to_seconds(ED_TIMEOUT_ENUM_DEFAULT).unwrap_or(10)
        })
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
    use crate::frames::{ED_TIMEOUT_ENUM_DEFAULT, ED_TIMEOUT_ENUM_REQUESTED};

    #[test]
    fn factory_new_nib_starts_without_nwk_security() {
        assert!(!Nib::new().security_enabled);
    }

    #[test]
    fn factory_new_nib_uses_the_default_end_device_timeout() {
        let nib = Nib::new();
        assert_eq!(nib.end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT);
        assert_eq!(nib.requested_end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT);
        assert_eq!(nib.parent_information, 0);
        assert!(!nib.parent_information_valid);
        assert_eq!(nib.end_device_timeout_seconds(), 256 * 60);
    }

    #[test]
    fn negotiation_reset_requests_the_long_timeout_and_clears_validity() {
        let mut nib = Nib::new();
        nib.accept_end_device_timeout(0x03);
        assert!(nib.parent_information_valid);

        nib.reset_end_device_timeout_negotiation();
        assert!(!nib.parent_information_valid);
        assert_eq!(nib.parent_information, 0);
        assert_eq!(nib.end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT);
        assert_eq!(nib.requested_end_device_timeout, ED_TIMEOUT_ENUM_REQUESTED);
    }

    #[test]
    fn accepting_a_response_masks_reserved_parent_information_bits() {
        let mut nib = Nib::new();
        nib.reset_end_device_timeout_negotiation();
        nib.accept_end_device_timeout(0xFD);
        assert_eq!(nib.parent_information, 0x01);
        assert!(nib.parent_information_valid);
        assert_eq!(nib.end_device_timeout, ED_TIMEOUT_ENUM_REQUESTED);
        assert_eq!(nib.end_device_timeout_seconds(), 16384 * 60);
    }

    #[test]
    fn a_repeat_acceptance_is_observable_even_when_nothing_else_moves() {
        let mut nib = Nib::new();
        nib.reset_end_device_timeout_negotiation();
        nib.accept_end_device_timeout(0x02);
        let first = nib.end_device_timeout_accepts;
        let information = nib.parent_information;
        let timeout = nib.end_device_timeout;

        nib.accept_end_device_timeout(0x02);

        assert_eq!(nib.parent_information, information);
        assert_eq!(nib.end_device_timeout, timeout);
        assert_ne!(
            nib.end_device_timeout_accepts, first,
            "a recurring keepalive acceptance must remain detectable"
        );
    }

    #[test]
    fn refusals_walk_down_to_the_default_and_stop() {
        let mut nib = Nib::new();
        nib.reset_end_device_timeout_negotiation();
        let mut steps = 0;
        while nib.lower_requested_end_device_timeout() {
            steps += 1;
            assert!(steps <= 16, "refusal walk must terminate");
        }
        assert_eq!(steps, ED_TIMEOUT_ENUM_REQUESTED - ED_TIMEOUT_ENUM_DEFAULT);
        assert_eq!(nib.requested_end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT);
        // A refusal must never invent parent information.
        assert!(!nib.parent_information_valid);
        assert_eq!(nib.parent_information, 0);
    }

    #[test]
    fn restoring_rejects_impossible_persisted_negotiation_state() {
        let mut nib = Nib::new();
        assert!(!nib.restore_end_device_timeout(0x00, true, 15));
        assert!(!nib.restore_end_device_timeout(0x04, true, 8));
        assert!(!nib.restore_end_device_timeout(0x01, false, 8));
        assert_eq!(nib.end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT);

        assert!(nib.restore_end_device_timeout(0x02, true, 14));
        assert_eq!(nib.parent_information, 0x02);
        assert!(nib.parent_information_valid);
        assert_eq!(nib.end_device_timeout, 14);
        assert_eq!(
            nib.requested_end_device_timeout, 14,
            "an accepted timeout is re-requested unchanged"
        );
    }

    #[test]
    fn restoring_an_unnegotiated_record_asks_for_the_long_timeout_again() {
        let mut nib = Nib::new();
        assert!(nib.restore_end_device_timeout(0x00, false, ED_TIMEOUT_ENUM_DEFAULT));
        assert!(!nib.parent_information_valid);
        assert_eq!(nib.end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT);
        assert_eq!(nib.requested_end_device_timeout, ED_TIMEOUT_ENUM_REQUESTED);
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
