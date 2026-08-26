//! Durable Zigbee network and security-counter state.

use core::cmp::max;

use zigbee_aps::security::ApsKeyType;
use zigbee_bdb::{
    CounterReservation, FRAME_COUNTER_RESERVATION_SIZE, NetworkSecurityState, SecurityPersistence,
    SecurityPersistenceError, TrustCenterLinkKeyState,
};
use zigbee_nwk::frames::{ED_TIMEOUT_ENUM_DEFAULT, ED_TIMEOUT_ENUM_MAX, PARENT_INFO_MASK};
use zigbee_types::IeeeAddress;

/// Encoded length of the current (version 4) record.
///
/// Version 3 appended the R22 End Device Timeout negotiation result to the
/// version 2 layout: flags bit 6 carries `parent_information_valid`, the
/// previously unused encoded byte 11 carries `parent_information`, and the new
/// byte 97 carries `end_device_timeout`.
///
/// Version 4 adds *no* byte at all: it claims the last free flags bit, bit 7,
/// for `update_id_valid`, so a record can finally say "this device holds no
/// authoritative `nwkUpdateId`" instead of implying a known `0`. The encoded
/// length is therefore identical to version 3, and the journal slot geometry
/// (slot size, CRC offset, prefix length and commit offset) is unchanged for
/// every version since 2.
pub const ENCODED_SECURITY_STATE_LEN: usize = 98;
/// Encoded length of a version 2 record (staged network key, no ED timeout).
pub(crate) const V2_ENCODED_SECURITY_STATE_LEN: usize = 97;
/// Encoded length of a version 1 record (no staged network key).
pub(crate) const LEGACY_ENCODED_SECURITY_STATE_LEN: usize = 80;

const FLAG_COMMISSIONED: u8 = 1 << 0;
const FLAG_TCLK_PRESENT: u8 = 1 << 1;
const FLAG_TCLK_INCOMING_VALID: u8 = 1 << 2;
const FLAG_REJOIN_PENDING: u8 = 1 << 3;
const FLAG_LEGACY_DEFAULT_TCLK: u8 = 1 << 4;
const FLAG_STAGED_NETWORK_KEY: u8 = 1 << 5;
const FLAG_PARENT_INFORMATION_VALID: u8 = 1 << 6;
const FLAG_UPDATE_ID_VALID: u8 = 1 << 7;

/// Encoded record layout revision.
///
/// Each variant lists exactly which bytes and flags it may touch, so an older
/// record can never be read with the newer field offsets and a newer flag bit
/// can never be silently accepted by an older layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StateFormat {
    /// 80 bytes: no staged network key, no End Device Timeout fields.
    V1,
    /// 97 bytes: staged network key, no End Device Timeout fields.
    V2,
    /// 98 bytes: staged network key and End Device Timeout fields, but no
    /// `nwkUpdateId` validity bit — the stored update ID is authoritative by
    /// construction.
    V3,
    /// 98 bytes: as version 3, plus flags bit 7 carrying `update_id_valid`.
    V4,
}

impl StateFormat {
    const fn allowed_flags(self) -> u8 {
        let common = FLAG_COMMISSIONED
            | FLAG_TCLK_PRESENT
            | FLAG_TCLK_INCOMING_VALID
            | FLAG_REJOIN_PENDING
            | FLAG_LEGACY_DEFAULT_TCLK;
        match self {
            Self::V1 => common,
            Self::V2 => common | FLAG_STAGED_NETWORK_KEY,
            Self::V3 => common | FLAG_STAGED_NETWORK_KEY | FLAG_PARENT_INFORMATION_VALID,
            Self::V4 => {
                common
                    | FLAG_STAGED_NETWORK_KEY
                    | FLAG_PARENT_INFORMATION_VALID
                    | FLAG_UPDATE_ID_VALID
            }
        }
    }

    const fn has_staged_key(self) -> bool {
        !matches!(self, Self::V1)
    }

    const fn has_end_device_timeout(self) -> bool {
        matches!(self, Self::V3 | Self::V4)
    }

    /// Whether the format encodes `nwkUpdateId` validity explicitly.
    ///
    /// Versions 1..=3 predate the bit. Their stored `update_id` was written by
    /// firmware whose NIB had no unknown state at all and whose restore path
    /// installed the byte unconditionally, so it stays authoritative on
    /// migration — anything else would silently drop live update state.
    const fn has_update_id_valid(self) -> bool {
        matches!(self, Self::V4)
    }
}

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
    /// `nwkUpdateId` as this device last knew it.
    ///
    /// Only meaningful while [`Self::update_id_valid`] is set; it is `0` in
    /// every other case, exactly as
    /// [`Nib::clear_nwk_update_id`](zigbee_nwk::nib::Nib::clear_nwk_update_id)
    /// leaves the live field.
    pub update_id: u8,
    /// Whether [`Self::update_id`] is a known-good network update state.
    ///
    /// `nwkUpdateId` is a serial number, so `0` is an ordinary live value and
    /// never an "unset" marker. A record migrated from a persistence format
    /// that never stored the item — such as a legacy ESP32 log-structured NV
    /// region — genuinely knows nothing about the network's update state, and
    /// restoring that silence as an authoritative `0` would make every beacon
    /// advertising `0x81..=0xFF` look stale and strand the device off its own
    /// network. Encoded in flags bit 7 from record version 4 onwards.
    pub update_id_valid: bool,
    pub network_key: [u8; 16],
    pub key_sequence: u8,
    pub staged_network_key_present: bool,
    pub staged_network_key: [u8; 16],
    pub staged_key_sequence: u8,
    /// Persisted exclusive upper bound, never the live counter.
    pub global_counter_limit: u32,
    pub tclk_present: bool,
    /// Commissioned network recovered from a persistence format that never
    /// stored a unique Trust Center link key.
    ///
    /// The node keeps its NWK identity, network key and counter reservation,
    /// but has no unique TCLK, so APS link-key traffic uses the well-known
    /// default global Trust Center link key. That key's outgoing counter space
    /// *is* the NWK frame counter (see
    /// `zigbee_aps::Apsde::next_default_tc_link_key_frame_counter`), which the
    /// durable `global_counter_limit` reservation already covers — no Trust
    /// Center address or key is ever invented. Mutually exclusive with
    /// [`Self::tclk_present`]; cleared as soon as a real unique TCLK is
    /// reserved.
    pub legacy_default_tclk: bool,
    pub trust_center_address: IeeeAddress,
    pub trust_center_link_key: [u8; 16],
    /// Persisted exclusive upper bound, never the live counter.
    pub tclk_counter_limit: u32,
    pub tclk_incoming_counter: u32,
    pub tclk_incoming_counter_valid: bool,
    pub rejoin_pending: bool,
    /// `nwkParentInformation` advertised by the parent in its End Device
    /// Timeout Response, masked to the two defined bits.
    ///
    /// Only meaningful while [`Self::parent_information_valid`] is set. Kept
    /// durable so a silent persisted resume can pick the right keepalive
    /// method — a MAC data poll or a fresh End Device Timeout Request —
    /// without re-running the negotiation on every reboot.
    ///
    /// The stored relationship is keyed by [`Self::parent_address`] only; the
    /// parent's IEEE address is not persisted yet. That is safe because the
    /// NWK layer clears validity at every real parent (re)assignment and
    /// parent loss, so a stored advertisement can only ever be replayed by the
    /// silent resume path, which keeps the same parent by construction.
    pub parent_information: u8,
    /// Whether [`Self::parent_information`] describes the stored parent.
    pub parent_information_valid: bool,
    /// `nwkEndDeviceTimeout` enumeration currently in effect (0..=14).
    ///
    /// Defaults to 8, the value a R22 parent applies to a child that never
    /// negotiated, so a migrated or freshly commissioned record can never
    /// claim a longer child lifetime than the parent actually granted.
    pub end_device_timeout: u8,
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
            update_id_valid: false,
            network_key: [0; 16],
            key_sequence: 0,
            staged_network_key_present: false,
            staged_network_key: [0; 16],
            staged_key_sequence: 0,
            global_counter_limit: 0,
            tclk_present: false,
            legacy_default_tclk: false,
            trust_center_address: [0; 8],
            trust_center_link_key: [0; 16],
            tclk_counter_limit: 0,
            tclk_incoming_counter: 0,
            tclk_incoming_counter_valid: false,
            rejoin_pending: false,
            parent_information: 0,
            parent_information_valid: false,
            end_device_timeout: ED_TIMEOUT_ENUM_DEFAULT,
        }
    }

    /// Whether this record describes the PAN coordinator itself.
    ///
    /// A coordinator owns short address `0x0000`, has depth zero, and has no
    /// parent. This shape is unambiguous in a Zigbee network and lets the
    /// existing record format represent a coordinator without inventing a
    /// self-referential Trust Center link key or consuming another format bit.
    pub(crate) const fn is_coordinator_network(&self) -> bool {
        self.short_address == 0x0000 && self.depth == 0 && self.parent_address == 0xFFFF
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
        }) | (if self.rejoin_pending {
            FLAG_REJOIN_PENDING
        } else {
            0
        }) | (if self.legacy_default_tclk {
            FLAG_LEGACY_DEFAULT_TCLK
        } else {
            0
        }) | (if self.staged_network_key_present {
            FLAG_STAGED_NETWORK_KEY
        } else {
            0
        }) | (if self.parent_information_valid {
            FLAG_PARENT_INFORMATION_VALID
        } else {
            0
        }) | (if self.update_id_valid {
            FLAG_UPDATE_ID_VALID
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
        output[11] = self.parent_information;
        output[12..16].copy_from_slice(&self.global_counter_limit.to_le_bytes());
        output[16..24].copy_from_slice(&self.extended_pan_id);
        output[24..32].copy_from_slice(&self.ieee_address);
        output[32..48].copy_from_slice(&self.network_key);
        output[48..56].copy_from_slice(&self.trust_center_address);
        output[56..72].copy_from_slice(&self.trust_center_link_key);
        output[72..76].copy_from_slice(&self.tclk_counter_limit.to_le_bytes());
        output[76..80].copy_from_slice(&self.tclk_incoming_counter.to_le_bytes());
        output[80] = self.staged_key_sequence;
        output[81..97].copy_from_slice(&self.staged_network_key);
        output[97] = self.end_device_timeout;
    }

    pub fn decode(input: &[u8; ENCODED_SECURITY_STATE_LEN]) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V4)
    }

    /// Decode a version 3 record: same length as the current format, but
    /// without the `update_id_valid` bit.
    pub(crate) fn decode_v3(
        input: &[u8; ENCODED_SECURITY_STATE_LEN],
    ) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V3)
    }

    pub(crate) fn decode_v2(
        input: &[u8; V2_ENCODED_SECURITY_STATE_LEN],
    ) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V2)
    }

    pub(crate) fn decode_legacy(
        input: &[u8; LEGACY_ENCODED_SECURITY_STATE_LEN],
    ) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V1)
    }

    fn decode_bytes(input: &[u8], format: StateFormat) -> Result<Self, SecurityStoreError> {
        let flags = input[0];
        if flags & !format.allowed_flags() != 0 {
            return Err(SecurityStoreError::Corrupt);
        }
        // `empty()` supplies the migration defaults for every field a format
        // predates — notably `end_device_timeout = 8` and invalid parent
        // information — so a v1/v2 record never reads byte 11 or byte 97.
        let mut state = Self::empty();
        state.commissioned = flags & FLAG_COMMISSIONED != 0;
        state.tclk_present = flags & FLAG_TCLK_PRESENT != 0;
        state.tclk_incoming_counter_valid = flags & FLAG_TCLK_INCOMING_VALID != 0;
        state.rejoin_pending = flags & FLAG_REJOIN_PENDING != 0;
        state.legacy_default_tclk = flags & FLAG_LEGACY_DEFAULT_TCLK != 0;
        state.channel = input[1];
        state.depth = input[2];
        state.update_id = input[3];
        // Versions 1..=3 have no validity bit: the byte they stored was
        // authoritative in the firmware that wrote it, so it stays
        // authoritative here. Only version 4 can express "unknown".
        state.update_id_valid = if format.has_update_id_valid() {
            flags & FLAG_UPDATE_ID_VALID != 0
        } else {
            true
        };
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
        if format.has_staged_key() {
            state.staged_network_key_present = flags & FLAG_STAGED_NETWORK_KEY != 0;
            state.staged_key_sequence = input[80];
            state.staged_network_key.copy_from_slice(&input[81..97]);
        }
        if format.has_end_device_timeout() {
            state.parent_information_valid = flags & FLAG_PARENT_INFORMATION_VALID != 0;
            state.parent_information = input[11];
            state.end_device_timeout = input[97];
        }

        state.validate()?;
        Ok(state)
    }

    pub fn validate(&self) -> Result<(), SecurityStoreError> {
        let coordinator = self.is_coordinator_network();
        if self.commissioned
            && (!(11..=26).contains(&self.channel)
                || self.pan_id == 0xFFFF
                || self.short_address == 0xFFFF
                || self.ieee_address == [0; 8]
                || self.global_counter_limit == 0
                || !(coordinator || self.tclk_present || self.legacy_default_tclk))
        {
            return Err(SecurityStoreError::Corrupt);
        }
        // The coordinator is the Trust Center; it never holds a unique TCLK
        // with itself and never has an end-device parent relationship. Keep
        // those representations disjoint so a corrupt leaf record cannot be
        // reinterpreted as a coordinator merely because it carries address 0.
        if coordinator
            && (self.tclk_present
                || self.legacy_default_tclk
                || self.trust_center_address != [0; 8]
                || self.trust_center_link_key != [0; 16]
                || self.tclk_incoming_counter != 0
                || self.tclk_incoming_counter_valid
                || self.rejoin_pending
                || self.parent_information != 0
                || self.parent_information_valid
                || self.end_device_timeout != ED_TIMEOUT_ENUM_DEFAULT)
        {
            return Err(SecurityStoreError::Corrupt);
        }
        // A legacy default-TCLK network is a commissioned network *without* a
        // unique key; the two representations must never be combined.
        if self.legacy_default_tclk
            && (!self.commissioned
                || self.tclk_present
                || self.trust_center_address != [0; 8]
                || self.trust_center_link_key != [0; 16]
                || self.tclk_counter_limit == 0
                || self.tclk_incoming_counter != 0
                || self.tclk_incoming_counter_valid)
        {
            return Err(SecurityStoreError::Corrupt);
        }
        if self.staged_network_key_present {
            if !self.commissioned || self.staged_key_sequence == self.key_sequence {
                return Err(SecurityStoreError::Corrupt);
            }
        } else if self.staged_key_sequence != 0 || self.staged_network_key != [0; 16] {
            return Err(SecurityStoreError::Corrupt);
        }
        if self.tclk_present
            && (self.trust_center_address == [0; 8] || self.tclk_counter_limit == 0)
        {
            return Err(SecurityStoreError::Corrupt);
        }
        if self.rejoin_pending && !self.commissioned {
            return Err(SecurityStoreError::Corrupt);
        }
        // R22 End Device Timeout negotiation result. An undefined enumeration
        // would produce an undefined keepalive deadline, a reserved
        // `nwkParentInformation` bit would claim a keepalive method that does
        // not exist, and information that is not valid must carry no bits at
        // all — otherwise a corrupt record could select a keepalive method
        // that silently ages the device out of its parent's child table.
        if self.end_device_timeout > ED_TIMEOUT_ENUM_MAX
            || self.parent_information & !PARENT_INFO_MASK != 0
            || (!self.parent_information_valid && self.parent_information != 0)
        {
            return Err(SecurityStoreError::Corrupt);
        }
        // An unknown `nwkUpdateId` carries no value at all, exactly as the NIB
        // holds it. Allowing a residual byte here would let a later revision
        // (or a corrupt record) resurrect it as authoritative update state.
        if !self.update_id_valid && self.update_id != 0 {
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

    #[cfg(any(feature = "router", test))]
    pub(crate) fn reserve_coordinator_network_security(
        &mut self,
        state: &NetworkSecurityState,
    ) -> Result<CounterReservation, SecurityStoreError> {
        match <Self as SecurityPersistence>::reserve_network_security(self, state) {
            Ok(reservation) => Ok(reservation),
            Err(SecurityPersistenceError::Storage) => {
                Err(self.take_error().unwrap_or(SecurityStoreError::Hardware))
            }
            Err(SecurityPersistenceError::CounterExhausted) => {
                Err(SecurityStoreError::CounterExhausted)
            }
            Err(SecurityPersistenceError::InvalidState) => Err(SecurityStoreError::Corrupt),
        }
    }

    /// Mark a freshly formed coordinator network as committed.
    ///
    /// Network formation has no unique-TCLK exchange: the coordinator is the
    /// Trust Center. The network reservation must already be durable and
    /// installed in the live NIB before this transition is committed.
    #[cfg(any(feature = "router", test))]
    pub(crate) fn commit_coordinator_network(&mut self) -> Result<(), SecurityStoreError> {
        if !self.state.is_coordinator_network()
            || self.state.tclk_present
            || self.state.legacy_default_tclk
        {
            return Err(SecurityStoreError::Corrupt);
        }
        self.state.commissioned = true;
        self.state.rejoin_pending = false;
        self.state.validate()?;
        self.store.store(&self.state)
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
        self.state.rejoin_pending = false;
        self.state.extended_pan_id = state.extended_pan_id;
        self.state.pan_id = state.pan_id;
        self.state.short_address = state.short_address;
        self.state.ieee_address = state.ieee_address;
        self.state.channel = state.channel;
        self.state.depth = state.depth;
        self.state.parent_address = state.parent_address;
        self.state.update_id = state.update_id;
        self.state.update_id_valid = state.update_id_valid;
        self.state.network_key = state.network_key;
        self.state.key_sequence = state.key_sequence;
        self.state.staged_network_key_present = false;
        self.state.staged_network_key = [0; 16];
        self.state.staged_key_sequence = 0;
        self.state.global_counter_limit = reservation.limit;
        self.state.tclk_present = false;
        self.state.legacy_default_tclk = false;
        self.state.trust_center_address = [0; 8];
        self.state.trust_center_link_key = [0; 16];
        self.state.tclk_incoming_counter = 0;
        self.state.tclk_incoming_counter_valid = false;
        // A fresh commissioning selects a new parent, so any keepalive method
        // the previous parent advertised is void and the child lifetime falls
        // back to the R22 default until the new parent answers.
        self.state.parent_information = 0;
        self.state.parent_information_valid = false;
        self.state.end_device_timeout = ED_TIMEOUT_ENUM_DEFAULT;
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
        self.state.legacy_default_tclk = false;
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
        self.state.rejoin_pending = false;
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
            update_id_valid: true,
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
        state.update_id_valid = true;
        state.network_key = [3; 16];
        state.key_sequence = 4;
        state.staged_network_key_present = true;
        state.staged_network_key = [8; 16];
        state.staged_key_sequence = 5;
        state.global_counter_limit = 0x400;
        state.tclk_present = true;
        state.trust_center_address = [5; 8];
        state.trust_center_link_key = [6; 16];
        state.tclk_counter_limit = 0x800;
        state.tclk_incoming_counter = 17;
        state.tclk_incoming_counter_valid = true;
        state.rejoin_pending = true;
        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(state));
    }

    #[test]
    fn end_device_timeout_fields_round_trip_and_use_the_new_byte() {
        let mut state = PersistentSecurityState::empty();
        state.parent_information = 0x02;
        state.parent_information_valid = true;
        state.end_device_timeout = 14;

        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);
        assert_eq!(
            encoded[0] & (1 << 6),
            1 << 6,
            "flags bit 6 carries validity"
        );
        assert_eq!(encoded[11], 0x02, "byte 11 carries parent information");
        assert_eq!(encoded[97], 14, "byte 97 carries the timeout enumeration");
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(state));
    }

    #[test]
    fn an_empty_state_defaults_to_the_r22_default_timeout() {
        let state = PersistentSecurityState::empty();
        assert_eq!(state.end_device_timeout, 8);
        assert_eq!(state.parent_information, 0);
        assert!(!state.parent_information_valid);
        assert_eq!(state.validate(), Ok(()));
    }

    #[test]
    fn impossible_end_device_timeout_state_is_rejected() {
        let mut state = PersistentSecurityState::empty();
        state.end_device_timeout = 15;
        assert_eq!(state.validate(), Err(SecurityStoreError::Corrupt));

        let mut state = PersistentSecurityState::empty();
        state.parent_information_valid = true;
        state.parent_information = 0x04;
        assert_eq!(state.validate(), Err(SecurityStoreError::Corrupt));

        let mut state = PersistentSecurityState::empty();
        state.parent_information = 0x01;
        assert_eq!(
            state.validate(),
            Err(SecurityStoreError::Corrupt),
            "advertised bits without validity are impossible"
        );
    }

    /// The `update_id_valid` bit is the whole point of record version 4: a
    /// state that knows its update ID and one that does not must survive a
    /// round trip as *different* states.
    #[test]
    fn update_id_validity_round_trips_through_the_current_format() {
        let mut known = PersistentSecurityState::empty();
        known.update_id = 0x2A;
        known.update_id_valid = true;
        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        known.encode(&mut encoded);
        assert_eq!(
            encoded[0] & (1 << 7),
            1 << 7,
            "flags bit 7 carries validity"
        );
        assert_eq!(encoded[3], 0x2A, "byte 3 still carries the update ID");
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(known));

        // A genuine, authoritative 0 is not the same state as "unknown", and
        // only the flag bit tells them apart.
        let mut known_zero = PersistentSecurityState::empty();
        known_zero.update_id_valid = true;
        let mut encoded_zero = [0u8; ENCODED_SECURITY_STATE_LEN];
        known_zero.encode(&mut encoded_zero);
        assert_eq!(encoded_zero[0] & (1 << 7), 1 << 7);
        assert_eq!(
            PersistentSecurityState::decode(&encoded_zero),
            Ok(known_zero)
        );

        let unknown = PersistentSecurityState::empty();
        let mut encoded_unknown = [0u8; ENCODED_SECURITY_STATE_LEN];
        unknown.encode(&mut encoded_unknown);
        assert_eq!(encoded_unknown[0] & (1 << 7), 0);
        assert_eq!(encoded_unknown[3], 0);
        let decoded = PersistentSecurityState::decode(&encoded_unknown).unwrap();
        assert_eq!(decoded, unknown);
        assert!(!decoded.update_id_valid);
        assert_ne!(decoded, known_zero, "unknown is not an authoritative 0");
    }

    #[test]
    fn an_empty_state_holds_no_authoritative_update_state() {
        let state = PersistentSecurityState::empty();
        assert!(!state.update_id_valid);
        assert_eq!(state.update_id, 0);
        assert_eq!(state.validate(), Ok(()));
    }

    #[test]
    fn coordinator_state_needs_no_self_tclk_and_round_trips() {
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [1; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x0000;
        state.ieee_address = [2; 8];
        state.channel = 15;
        state.depth = 0;
        state.parent_address = 0xFFFF;
        state.update_id = 3;
        state.update_id_valid = true;
        state.network_key = [4; 16];
        state.global_counter_limit = 0x400;
        assert!(state.is_coordinator_network());
        assert_eq!(state.validate(), Ok(()));

        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(state));

        // An interrupted coordinator reservation is intentionally still
        // readable: the next boot forms a new PAN while preserving the
        // abandoned counter floor.
        state.commissioned = false;
        assert_eq!(state.validate(), Ok(()));
        state.encode(&mut encoded);
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(state));
    }

    #[test]
    fn an_unknown_update_id_may_not_carry_a_value() {
        let mut state = PersistentSecurityState::empty();
        state.update_id = 7;
        assert_eq!(
            state.validate(),
            Err(SecurityStoreError::Corrupt),
            "an update ID without validity is impossible"
        );
    }

    /// A version 3 record has the *same* length as the current one, so the
    /// version byte is the only thing separating them.
    #[test]
    fn a_v3_record_has_no_validity_bit_and_stays_authoritative() {
        let mut state = PersistentSecurityState::empty();
        state.update_id = 0x2A;
        state.update_id_valid = true;
        state.parent_information = 0x02;
        state.parent_information_valid = true;
        state.end_device_timeout = 14;
        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);

        // Firmware that predates version 4 never set bit 7 …
        encoded[0] &= !(1 << 7);
        let migrated = PersistentSecurityState::decode_v3(&encoded).unwrap();
        assert_eq!(migrated, state, "the v3 update ID stays authoritative");
        // … including for the value 0, which such a record still meant
        // literally.
        let mut zero = encoded;
        zero[3] = 0;
        let migrated_zero = PersistentSecurityState::decode_v3(&zero).unwrap();
        assert_eq!(migrated_zero.update_id, 0);
        assert!(migrated_zero.update_id_valid);
        // The version 3 fields it does own are decoded normally.
        assert_eq!(migrated_zero.parent_information, 0x02);
        assert!(migrated_zero.parent_information_valid);
        assert_eq!(migrated_zero.end_device_timeout, 14);

        // The version 4 flag bit does not exist in the v3 layout.
        encoded[0] |= 1 << 7;
        assert_eq!(
            PersistentSecurityState::decode_v3(&encoded),
            Err(SecurityStoreError::Corrupt)
        );
        // The very same bytes are a valid version 4 record.
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(state));
    }

    #[test]
    fn a_v2_record_never_decodes_the_version_three_fields() {
        let mut state = PersistentSecurityState::empty();
        state.parent_information = 0x03;
        state.parent_information_valid = true;
        state.end_device_timeout = 14;
        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);

        // A real v2 record carries neither the flag bit nor byte 11.
        let mut v2 = [0u8; V2_ENCODED_SECURITY_STATE_LEN];
        v2.copy_from_slice(&encoded[..V2_ENCODED_SECURITY_STATE_LEN]);
        v2[0] &= !(1 << 6);
        v2[11] = 0;
        let migrated = PersistentSecurityState::decode_v2(&v2).unwrap();
        assert_eq!(migrated.parent_information, 0);
        assert!(!migrated.parent_information_valid);
        assert_eq!(migrated.end_device_timeout, 8);

        // The version 3 flag bit does not exist in the v2 layout.
        v2[0] |= 1 << 6;
        assert_eq!(
            PersistentSecurityState::decode_v2(&v2),
            Err(SecurityStoreError::Corrupt)
        );
    }

    #[test]
    fn legacy_default_tclk_state_round_trips_and_is_validated() {
        // A network recovered from a persistence format that never stored a
        // unique TCLK: commissioned, but explicitly without one.
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.legacy_default_tclk = true;
        state.extended_pan_id = [1; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = [2; 8];
        state.channel = 15;
        state.network_key = [3; 16];
        state.key_sequence = 4;
        state.global_counter_limit = 0x400;
        state.tclk_counter_limit = 0x400;
        assert_eq!(state.validate(), Ok(()));

        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);
        let decoded = PersistentSecurityState::decode(&encoded).unwrap();
        assert_eq!(decoded, state);
        assert!(decoded.legacy_default_tclk);
        assert!(!decoded.tclk_present);
        assert_eq!(decoded.trust_center_address, [0; 8]);

        // The flag is only meaningful for a commissioned network without a
        // unique key; every other combination is corruption.
        let mut both = state;
        both.tclk_present = true;
        both.trust_center_address = [5; 8];
        assert_eq!(both.validate(), Err(SecurityStoreError::Corrupt));
        let mut uncommissioned = state;
        uncommissioned.commissioned = false;
        assert_eq!(uncommissioned.validate(), Err(SecurityStoreError::Corrupt));
        let mut neither = state;
        neither.legacy_default_tclk = false;
        assert_eq!(neither.validate(), Err(SecurityStoreError::Corrupt));
        let mut invented_trust_center = state;
        invented_trust_center.trust_center_address = [5; 8];
        assert_eq!(
            invented_trust_center.validate(),
            Err(SecurityStoreError::Corrupt)
        );
        let mut no_tclk_floor = state;
        no_tclk_floor.tclk_counter_limit = 0;
        assert_eq!(no_tclk_floor.validate(), Err(SecurityStoreError::Corrupt));
    }

    #[test]
    fn a_real_tclk_replaces_the_legacy_default_key_marker() {
        let mut store = RamSecurityStateStore::new();
        let mut legacy = PersistentSecurityState::empty();
        legacy.commissioned = true;
        legacy.legacy_default_tclk = true;
        legacy.extended_pan_id = [1; 8];
        legacy.pan_id = 0x1234;
        legacy.short_address = 0x5678;
        legacy.ieee_address = [2; 8];
        legacy.channel = 15;
        legacy.network_key = [3; 16];
        legacy.global_counter_limit = 0x800;
        legacy.tclk_counter_limit = 0x800;
        store.store(&legacy).unwrap();

        {
            let mut persistence = CommissioningSecurityPersistence::new(&mut store).unwrap();
            // A unique key delivered later continues above the migrated floor …
            assert_eq!(
                persistence.reserve_trust_center_link_key(&tclk_state(0, 0)),
                Ok(CounterReservation {
                    current: 0x800,
                    limit: 0xC00
                })
            );
            persistence.commit_network(&tclk_state(1, 9)).unwrap();
        }

        // … and the transitional marker is gone once it exists.
        let saved = store.load().unwrap().unwrap();
        assert!(saved.commissioned);
        assert!(saved.tclk_present);
        assert!(!saved.legacy_default_tclk);
        assert_eq!(saved.tclk_counter_limit, 0xC00);
        assert_eq!(saved.validate(), Ok(()));
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
    fn fresh_commissioning_discards_a_previously_staged_network_key() {
        let mut store = RamSecurityStateStore::new();
        let mut old = PersistentSecurityState::empty();
        old.commissioned = true;
        old.staged_network_key_present = true;
        old.staged_network_key = [0x55; 16];
        old.staged_key_sequence = 7;
        store.store(&old).unwrap();

        {
            let mut persistence = CommissioningSecurityPersistence::new(&mut store).unwrap();
            persistence
                .reserve_network_security(&network_state(0))
                .unwrap();
        }

        let saved = store.load().unwrap().unwrap();
        assert!(!saved.staged_network_key_present);
        assert_eq!(saved.staged_network_key, [0; 16]);
        assert_eq!(saved.staged_key_sequence, 0);
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
