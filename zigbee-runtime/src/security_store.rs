//! Durable Zigbee network and security-counter state.

use core::cmp::max;

use zigbee_aps::security::ApsKeyType;
use zigbee_bdb::{
    CounterReservation, FRAME_COUNTER_RESERVATION_SIZE, NetworkSecurityState, SecurityPersistence,
    SecurityPersistenceError, TrustCenterLinkKeyState,
};
use zigbee_types::IeeeAddress;

pub const ENCODED_SECURITY_STATE_LEN: usize = 80;

const FLAG_COMMISSIONED: u8 = 1 << 0;
const FLAG_TCLK_PRESENT: u8 = 1 << 1;
const FLAG_TCLK_INCOMING_VALID: u8 = 1 << 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SecurityStoreError {
    NotFound,
    Corrupt,
    Full,
    Hardware,
    CounterExhausted,
    GenerationExhausted,
}

/// Complete crash-safe state needed for secured rejoin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistentSecurityState {
    pub commissioned: bool,
    pub extended_pan_id: IeeeAddress,
    pub pan_id: u16,
    pub short_address: u16,
    pub ieee_address: IeeeAddress,
    pub channel: u8,
    pub depth: u8,
    pub parent_address: u16,
    pub update_id: u8,
    pub network_key: [u8; 16],
    pub key_sequence: u8,
    /// Persisted exclusive upper bound, never the live counter.
    pub global_counter_limit: u32,
    pub tclk_present: bool,
    pub trust_center_address: IeeeAddress,
    pub trust_center_link_key: [u8; 16],
    /// Persisted exclusive upper bound, never the live counter.
    pub tclk_counter_limit: u32,
    pub tclk_incoming_counter: u32,
    pub tclk_incoming_counter_valid: bool,
}

impl PersistentSecurityState {
    pub const fn empty() -> Self {
        Self {
            commissioned: false,
            extended_pan_id: [0; 8],
            pan_id: 0,
            short_address: 0,
            ieee_address: [0; 8],
            channel: 0,
            depth: 0,
            parent_address: 0,
            update_id: 0,
            network_key: [0; 16],
            key_sequence: 0,
            global_counter_limit: 0,
            tclk_present: false,
            trust_center_address: [0; 8],
            trust_center_link_key: [0; 16],
            tclk_counter_limit: 0,
            tclk_incoming_counter: 0,
            tclk_incoming_counter_valid: false,
        }
    }

    pub fn encode(&self, output: &mut [u8; ENCODED_SECURITY_STATE_LEN]) {
        output.fill(0);
        output[0] = (if self.commissioned {
            FLAG_COMMISSIONED
        } else {
            0
        }) | (if self.tclk_present {
            FLAG_TCLK_PRESENT
        } else {
            0
        }) | (if self.tclk_incoming_counter_valid {
            FLAG_TCLK_INCOMING_VALID
        } else {
            0
        });
        output[1] = self.channel;
        output[2] = self.depth;
        output[3] = self.update_id;
        output[4..6].copy_from_slice(&self.pan_id.to_le_bytes());
        output[6..8].copy_from_slice(&self.short_address.to_le_bytes());
        output[8..10].copy_from_slice(&self.parent_address.to_le_bytes());
        output[10] = self.key_sequence;
        output[12..16].copy_from_slice(&self.global_counter_limit.to_le_bytes());
        output[16..24].copy_from_slice(&self.extended_pan_id);
        output[24..32].copy_from_slice(&self.ieee_address);
        output[32..48].copy_from_slice(&self.network_key);
        output[48..56].copy_from_slice(&self.trust_center_address);
        output[56..72].copy_from_slice(&self.trust_center_link_key);
        output[72..76].copy_from_slice(&self.tclk_counter_limit.to_le_bytes());
        output[76..80].copy_from_slice(&self.tclk_incoming_counter.to_le_bytes());
    }

    pub fn decode(input: &[u8; ENCODED_SECURITY_STATE_LEN]) -> Result<Self, SecurityStoreError> {
        let flags = input[0];
        if flags & !(FLAG_COMMISSIONED | FLAG_TCLK_PRESENT | FLAG_TCLK_INCOMING_VALID) != 0 {
            return Err(SecurityStoreError::Corrupt);
        }
        let mut state = Self::empty();
        state.commissioned = flags & FLAG_COMMISSIONED != 0;
        state.tclk_present = flags & FLAG_TCLK_PRESENT != 0;
        state.tclk_incoming_counter_valid = flags & FLAG_TCLK_INCOMING_VALID != 0;
        state.channel = input[1];
        state.depth = input[2];
        state.update_id = input[3];
        state.pan_id = u16::from_le_bytes([input[4], input[5]]);
        state.short_address = u16::from_le_bytes([input[6], input[7]]);
        state.parent_address = u16::from_le_bytes([input[8], input[9]]);
        state.key_sequence = input[10];
        state.global_counter_limit =
            u32::from_le_bytes([input[12], input[13], input[14], input[15]]);
        state.extended_pan_id.copy_from_slice(&input[16..24]);
        state.ieee_address.copy_from_slice(&input[24..32]);
        state.network_key.copy_from_slice(&input[32..48]);
        state.trust_center_address.copy_from_slice(&input[48..56]);
        state.trust_center_link_key.copy_from_slice(&input[56..72]);
        state.tclk_counter_limit = u32::from_le_bytes([input[72], input[73], input[74], input[75]]);
        state.tclk_incoming_counter =
            u32::from_le_bytes([input[76], input[77], input[78], input[79]]);

        state.validate()?;
        Ok(state)
    }

    pub fn validate(&self) -> Result<(), SecurityStoreError> {
        if self.commissioned
            && (!(11..=26).contains(&self.channel)
                || self.pan_id == 0xFFFF
                || self.short_address == 0xFFFF
                || self.ieee_address == [0; 8]
                || self.global_counter_limit == 0
                || !self.tclk_present)
        {
            return Err(SecurityStoreError::Corrupt);
        }
        if self.tclk_present
            && (self.trust_center_address == [0; 8] || self.tclk_counter_limit == 0)
        {
            return Err(SecurityStoreError::Corrupt);
        }
        Ok(())
    }
}

impl Default for PersistentSecurityState {
    fn default() -> Self {
        Self::empty()
    }
}

/// Atomic storage for complete security-state snapshots.
pub trait SecurityStateStore {
    fn load(&mut self) -> Result<Option<PersistentSecurityState>, SecurityStoreError>;
    fn store(&mut self, state: &PersistentSecurityState) -> Result<(), SecurityStoreError>;
}

pub(crate) struct CommissioningSecurityPersistence<'a, S: SecurityStateStore> {
    store: &'a mut S,
    state: PersistentSecurityState,
    last_error: Option<SecurityStoreError>,
}

impl<'a, S: SecurityStateStore> CommissioningSecurityPersistence<'a, S> {
    pub(crate) fn new(store: &'a mut S) -> Result<Self, SecurityStoreError> {
        let state = store.load()?.unwrap_or_default();
        Ok(Self {
            store,
            state,
            last_error: None,
        })
    }

    pub(crate) fn take_error(&mut self) -> Option<SecurityStoreError> {
        self.last_error.take()
    }

    fn reserve_from(
        &mut self,
        current: u32,
    ) -> Result<CounterReservation, SecurityPersistenceError> {
        let limit = current
            .checked_add(FRAME_COUNTER_RESERVATION_SIZE)
            .ok_or(SecurityPersistenceError::CounterExhausted)?;
        Ok(CounterReservation { current, limit })
    }

    fn persist(&mut self) -> Result<(), SecurityPersistenceError> {
        self.store.store(&self.state).map_err(|error| {
            self.last_error = Some(error);
            SecurityPersistenceError::Storage
        })
    }
}

impl<S: SecurityStateStore> SecurityPersistence for CommissioningSecurityPersistence<'_, S> {
    fn reserve_network_security(
        &mut self,
        state: &NetworkSecurityState,
    ) -> Result<CounterReservation, SecurityPersistenceError> {
        let current = max(
            state.outgoing_frame_counter,
            self.state.global_counter_limit,
        );
        let reservation = self.reserve_from(current)?;

        self.state.commissioned = false;
        self.state.extended_pan_id = state.extended_pan_id;
        self.state.pan_id = state.pan_id;
        self.state.short_address = state.short_address;
        self.state.ieee_address = state.ieee_address;
        self.state.channel = state.channel;
        self.state.depth = state.depth;
        self.state.parent_address = state.parent_address;
        self.state.update_id = state.update_id;
        self.state.network_key = state.network_key;
        self.state.key_sequence = state.key_sequence;
        self.state.global_counter_limit = reservation.limit;
        self.state.tclk_present = false;
        self.state.trust_center_address = [0; 8];
        self.state.trust_center_link_key = [0; 16];
        self.state.tclk_incoming_counter = 0;
        self.state.tclk_incoming_counter_valid = false;
        self.persist()?;
        Ok(reservation)
    }

    fn reserve_trust_center_link_key(
        &mut self,
        state: &TrustCenterLinkKeyState,
    ) -> Result<CounterReservation, SecurityPersistenceError> {
        if state.key_type != ApsKeyType::TrustCenterLinkKey {
            return Err(SecurityPersistenceError::InvalidState);
        }
        // Keep one monotonic reservation space across replacement TCLKs. This
        // avoids nonce reuse if commissioning is interrupted or a factory-new
        // join receives the same per-device key again.
        let current = max(state.outgoing_frame_counter, self.state.tclk_counter_limit);
        let reservation = self.reserve_from(current)?;

        self.state.tclk_present = true;
        self.state.trust_center_address = state.partner_address;
        self.state.trust_center_link_key = state.key;
        self.state.tclk_counter_limit = reservation.limit;
        self.state.tclk_incoming_counter = state.incoming_frame_counter;
        self.state.tclk_incoming_counter_valid = state.incoming_frame_counter_valid;
        self.persist()?;
        Ok(reservation)
    }

    fn commit_network(
        &mut self,
        trust_center_link_key: &TrustCenterLinkKeyState,
    ) -> Result<(), SecurityPersistenceError> {
        if !self.state.tclk_present
            || self.state.trust_center_address != trust_center_link_key.partner_address
            || self.state.trust_center_link_key != trust_center_link_key.key
            || trust_center_link_key.outgoing_frame_counter > self.state.tclk_counter_limit
        {
            return Err(SecurityPersistenceError::InvalidState);
        }
        self.state.tclk_incoming_counter = trust_center_link_key.incoming_frame_counter;
        self.state.tclk_incoming_counter_valid = trust_center_link_key.incoming_frame_counter_valid;
        self.state.commissioned = true;
        self.persist()
    }
}

/// In-memory store for tests.
pub struct RamSecurityStateStore {
    state: Option<PersistentSecurityState>,
}

impl RamSecurityStateStore {
    pub const fn new() -> Self {
        Self { state: None }
    }
}

impl Default for RamSecurityStateStore {
    fn default() -> Self {
        Self::new()
    }
}

impl SecurityStateStore for RamSecurityStateStore {
    fn load(&mut self) -> Result<Option<PersistentSecurityState>, SecurityStoreError> {
        Ok(self.state)
    }

    fn store(&mut self, state: &PersistentSecurityState) -> Result<(), SecurityStoreError> {
        self.state = Some(*state);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_bdb::SecurityPersistence;

    fn network_state(counter: u32) -> NetworkSecurityState {
        NetworkSecurityState {
            extended_pan_id: [1; 8],
            pan_id: 0x1234,
            short_address: 0x5678,
            ieee_address: [2; 8],
            channel: 15,
            depth: 1,
            parent_address: 0,
            update_id: 3,
            network_key: [4; 16],
            key_sequence: 5,
            outgoing_frame_counter: counter,
        }
    }

    fn tclk_state(counter: u32, incoming: u32) -> TrustCenterLinkKeyState {
        TrustCenterLinkKeyState {
            partner_address: [6; 8],
            key: [7; 16],
            key_type: ApsKeyType::TrustCenterLinkKey,
            outgoing_frame_counter: counter,
            incoming_frame_counter: incoming,
            incoming_frame_counter_valid: true,
        }
    }

    #[test]
    fn state_encoding_round_trips() {
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [1; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = [2; 8];
        state.channel = 15;
        state.depth = 1;
        state.parent_address = 0x1111;
        state.update_id = 9;
        state.network_key = [3; 16];
        state.key_sequence = 4;
        state.global_counter_limit = 0x400;
        state.tclk_present = true;
        state.trust_center_address = [5; 8];
        state.trust_center_link_key = [6; 16];
        state.tclk_counter_limit = 0x800;
        state.tclk_incoming_counter = 17;
        state.tclk_incoming_counter_valid = true;
        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(state));
    }

    #[test]
    fn commissioning_reserves_before_commit() {
        let mut store = RamSecurityStateStore::new();
        {
            let mut persistence = CommissioningSecurityPersistence::new(&mut store).unwrap();
            assert_eq!(
                persistence.reserve_network_security(&network_state(2)),
                Ok(CounterReservation {
                    current: 2,
                    limit: 0x402
                })
            );
            assert_eq!(
                persistence.reserve_trust_center_link_key(&tclk_state(0, 0)),
                Ok(CounterReservation {
                    current: 0,
                    limit: 0x400
                })
            );
            persistence.commit_network(&tclk_state(1, 9)).unwrap();
        }
        let saved = store.load().unwrap().unwrap();
        assert!(saved.commissioned);
        assert_eq!(saved.global_counter_limit, 0x402);
        assert_eq!(saved.tclk_counter_limit, 0x400);
        assert_eq!(saved.tclk_incoming_counter, 9);
    }

    #[test]
    fn preserved_global_limit_is_next_boot_start() {
        let mut store = RamSecurityStateStore::new();
        let mut old = PersistentSecurityState::empty();
        old.global_counter_limit = 0x800;
        store.store(&old).unwrap();
        let mut persistence = CommissioningSecurityPersistence::new(&mut store).unwrap();
        assert_eq!(
            persistence.reserve_network_security(&network_state(0)),
            Ok(CounterReservation {
                current: 0x800,
                limit: 0xC00
            })
        );
    }

    #[test]
    fn preserved_tclk_limit_survives_interrupted_commissioning() {
        let mut store = RamSecurityStateStore::new();
        let mut old = PersistentSecurityState::empty();
        old.tclk_counter_limit = 0x800;
        store.store(&old).unwrap();

        {
            let mut persistence = CommissioningSecurityPersistence::new(&mut store).unwrap();
            persistence
                .reserve_network_security(&network_state(0))
                .unwrap();
        }

        let interrupted = store.load().unwrap().unwrap();
        assert!(!interrupted.tclk_present);
        assert_eq!(interrupted.tclk_counter_limit, 0x800);

        let mut persistence = CommissioningSecurityPersistence::new(&mut store).unwrap();
        persistence
            .reserve_network_security(&network_state(0))
            .unwrap();
        let mut replacement_tclk = tclk_state(0, 0);
        replacement_tclk.key = [9; 16];
        assert_eq!(
            persistence.reserve_trust_center_link_key(&replacement_tclk),
            Ok(CounterReservation {
                current: 0x800,
                limit: 0xC00
            })
        );
    }
}
