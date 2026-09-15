//! Crash-safe persistence hooks for commissioning security state.

use zigbee_aps::security::{AesKey, ApsKeyType};
use zigbee_types::IeeeAddress;

use crate::attributes::NodeJoinLinkKeyType;

/// Official Telink outgoing-security-counter reservation size.
pub const FRAME_COUNTER_RESERVATION_SIZE: u32 = 0x400;

/// A durably persisted counter range `[current, limit)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CounterReservation {
    pub current: u32,
    pub limit: u32,
}

impl CounterReservation {
    pub const fn is_valid(self) -> bool {
        self.current < self.limit
    }
}

/// Network state available immediately after the Network-Key Transport-Key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetworkSecurityState {
    pub extended_pan_id: IeeeAddress,
    pub pan_id: u16,
    pub short_address: u16,
    pub ieee_address: IeeeAddress,
    pub channel: u8,
    pub depth: u8,
    pub parent_address: u16,
    /// `nwkUpdateId` as the NIB holds it, or `0` when it is not known-good.
    pub update_id: u8,
    /// Whether [`Self::update_id`] is an authoritative network update state.
    ///
    /// Carried explicitly so persistence can never promote the placeholder of
    /// an unknown state into a known `0` (see
    /// [`Nib::nwk_update_id`](zigbee_nwk::nib::Nib::nwk_update_id)).
    pub update_id_valid: bool,
    pub network_key: AesKey,
    pub key_sequence: u8,
    pub outgoing_frame_counter: u32,
    /// Security model learned from the initial Network-Key Transport-Key.
    pub trust_center_address: IeeeAddress,
    /// Link-key regime that decrypted the initial network key.
    pub node_join_link_key_type: NodeJoinLinkKeyType,
}

/// Unique Trust Center link-key state installed during commissioning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrustCenterLinkKeyState {
    pub partner_address: IeeeAddress,
    pub key: AesKey,
    pub key_type: ApsKeyType,
    pub outgoing_frame_counter: u32,
    pub incoming_frame_counter: u32,
    pub incoming_frame_counter_valid: bool,
}

/// Persistence failure reported synchronously to commissioning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SecurityPersistenceError {
    Storage,
    CounterExhausted,
    InvalidState,
}

/// Synchronous persistence required before commissioning may use security keys.
pub trait SecurityPersistence {
    /// Persist the network identity/key and reserve the global outgoing range.
    fn reserve_network_security(
        &mut self,
        state: &NetworkSecurityState,
    ) -> Result<CounterReservation, SecurityPersistenceError>;

    /// Persist the unique TCLK and reserve its per-key outgoing range.
    fn reserve_trust_center_link_key(
        &mut self,
        state: &TrustCenterLinkKeyState,
    ) -> Result<CounterReservation, SecurityPersistenceError>;

    /// Persist final TCLK counters and mark the network valid after Confirm-Key.
    fn commit_network(
        &mut self,
        trust_center_link_key: &TrustCenterLinkKeyState,
    ) -> Result<(), SecurityPersistenceError>;

    /// Mark a distributed-security network commissioned after `Device_annce`.
    ///
    /// Distributed networks have no Trust Center and therefore no unique-TCLK
    /// exchange to provide the centralized commit point.
    fn commit_distributed_network(&mut self) -> Result<(), SecurityPersistenceError>;
}
