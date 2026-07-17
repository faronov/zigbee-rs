//! Crash-safe persistence hooks for commissioning security state.

use zigbee_aps::security::{AesKey, ApsKeyType};
use zigbee_types::IeeeAddress;

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
    pub update_id: u8,
    pub network_key: AesKey,
    pub key_sequence: u8,
    pub outgoing_frame_counter: u32,
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
}
