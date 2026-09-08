//! Durable Zigbee network and security-counter state.

use core::cmp::max;

use zigbee_aps::security::ApsKeyType;
#[cfg(any(feature = "router", test))]
use zigbee_bdb::formation::FormationPersistence;
use zigbee_bdb::{
    CounterReservation, FRAME_COUNTER_RESERVATION_SIZE, NetworkSecurityState, NodeJoinLinkKeyType,
    SecurityPersistence, SecurityPersistenceError, TrustCenterLinkKeyState,
};
use zigbee_nwk::frames::{ED_TIMEOUT_ENUM_DEFAULT, ED_TIMEOUT_ENUM_MAX, PARENT_INFO_MASK};
use zigbee_types::IeeeAddress;

pub const MAX_PERSISTENT_REPLAY_COUNTERS: usize = zigbee_nwk::security::MAX_FRAME_COUNTER_ENTRIES
    + zigbee_aps::security::MAX_KEY_TABLE_ENTRIES
    + zigbee_aps::security::MAX_GLOBAL_REPLAY_ENTRIES;

/// Encoded length of the current (version 8) record.
///
/// Version 3 appended the R22 End Device Timeout negotiation result to the
/// version 2 layout: flags bit 6 carries `parent_information_valid`, the
/// previously unused encoded byte 11 carries `parent_information`, and the new
/// byte 97 carries `end_device_timeout`.
///
/// Version 4 added `update_id_valid` in flags bit 7 without changing length.
/// Version 5 appends the BDB Table 6 join-link-key type so the centralized or
/// distributed security model survives reboot.
///
/// Version 6 appends the inactive NWK key role, distinguishing a future staged
/// key from the previous key retained after activation.
///
/// Version 7 appends crash-resumable NWK lifecycle state: a pending short-PAN
/// transition, a pending `Device_annce`, and a provisional parent link.
///
/// Version 8 uses lifecycle flags bit 4 for explicit parent Network-Key fanout
/// intent. Length and flash slot geometry are unchanged. Older records do not
/// infer fanout from an inactive key: its descriptor destination is unknown.
pub const ENCODED_SECURITY_STATE_LEN: usize = 103;
/// Encoded length of a version 6 record.
pub(crate) const V6_ENCODED_SECURITY_STATE_LEN: usize = 100;
/// Encoded length of a version 5 record.
pub(crate) const V5_ENCODED_SECURITY_STATE_LEN: usize = 99;
/// Encoded length of version 3 and version 4 records.
pub(crate) const V4_ENCODED_SECURITY_STATE_LEN: usize = 98;
/// Encoded length of a version 2 record (secondary network key, no ED timeout).
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

const LIFECYCLE_PENDING_PAN_ID: u8 = 1 << 0;
const LIFECYCLE_PENDING_PAN_ID_BROADCAST: u8 = 1 << 1;
const LIFECYCLE_DEVICE_ANNOUNCE_PENDING: u8 = 1 << 2;
const LIFECYCLE_PARENT_LINK_PROVISIONAL: u8 = 1 << 3;
const LIFECYCLE_NETWORK_KEY_FORWARDING: u8 = 1 << 4;
const LIFECYCLE_ALLOWED_FLAGS: u8 = LIFECYCLE_PENDING_PAN_ID
    | LIFECYCLE_PENDING_PAN_ID_BROADCAST
    | LIFECYCLE_DEVICE_ANNOUNCE_PENDING
    | LIFECYCLE_PARENT_LINK_PROVISIONAL;

/// Encoded record layout revision.
///
/// Each variant lists exactly which bytes and flags it may touch, so an older
/// record can never be read with the newer field offsets and a newer flag bit
/// can never be silently accepted by an older layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StateFormat {
    /// 80 bytes: no staged network key, no End Device Timeout fields.
    V1,
    /// 97 bytes: secondary network key, no End Device Timeout fields.
    V2,
    /// 98 bytes: staged network key and End Device Timeout fields, but no
    /// `nwkUpdateId` validity bit — the stored update ID is authoritative by
    /// construction.
    V3,
    /// 98 bytes: as version 3, plus flags bit 7 carrying `update_id_valid`.
    V4,
    /// 99 bytes: as version 4, plus `bdbNodeJoinLinkKeyType`.
    V5,
    /// 100 bytes: as version 5, plus the inactive NWK key role.
    V6,
    /// 103 bytes: as version 6, plus crash-resumable NWK lifecycle state.
    V7,
    /// 103 bytes: as version 7, plus explicit Network-Key fanout intent.
    V8,
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
            Self::V4 | Self::V5 | Self::V6 | Self::V7 | Self::V8 => {
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
        matches!(
            self,
            Self::V3 | Self::V4 | Self::V5 | Self::V6 | Self::V7 | Self::V8
        )
    }

    /// Whether the format encodes `nwkUpdateId` validity explicitly.
    ///
    /// Versions 1..=3 predate the bit. Their stored `update_id` was written by
    /// firmware whose NIB had no unknown state at all and whose restore path
    /// installed the byte unconditionally, so it stays authoritative on
    /// migration — anything else would silently drop live update state.
    const fn has_update_id_valid(self) -> bool {
        matches!(self, Self::V4 | Self::V5 | Self::V6 | Self::V7 | Self::V8)
    }

    const fn has_node_join_link_key_type(self) -> bool {
        matches!(self, Self::V5 | Self::V6 | Self::V7 | Self::V8)
    }

    const fn has_secondary_key_role(self) -> bool {
        matches!(self, Self::V6 | Self::V7 | Self::V8)
    }

    const fn has_nwk_lifecycle(self) -> bool {
        matches!(self, Self::V7 | Self::V8)
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

/// One durably committed incoming replay floor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentReplayCounter {
    Nwk(zigbee_nwk::security::NwkReplayCounter),
    Aps(zigbee_aps::security::ApsReplayCounter),
}

impl PersistentReplayCounter {
    pub(crate) fn same_domain(&self, other: &Self) -> bool {
        match (*self, *other) {
            (Self::Nwk(left), Self::Nwk(right)) => {
                left.source == right.source
                    && left.key_sequence == right.key_sequence
                    && left.key_fingerprint == right.key_fingerprint
            }
            (Self::Aps(left), Self::Aps(right)) => {
                left.origin == right.origin && left.key_fingerprint == right.key_fingerprint
            }
            _ => false,
        }
    }

    pub(crate) const fn counter(self) -> u32 {
        match self {
            Self::Nwk(replay) => replay.counter,
            Self::Aps(replay) => replay.counter,
        }
    }

    pub(crate) fn matches_tombstone(self, tombstone: ReplayCounterTombstone) -> bool {
        match tombstone {
            ReplayCounterTombstone::Device(address) => match self {
                Self::Nwk(replay) => replay.source == address,
                Self::Aps(replay) => match replay.origin {
                    zigbee_aps::security::ApsReplayOrigin::KeyPair { partner, .. } => {
                        partner == address
                    }
                    zigbee_aps::security::ApsReplayOrigin::PreconfiguredGlobal { .. }
                    | zigbee_aps::security::ApsReplayOrigin::DistributedGlobal { .. } => false,
                },
            },
            ReplayCounterTombstone::KeyFingerprint(key_fingerprint) => match self {
                Self::Nwk(replay) => replay.key_fingerprint == key_fingerprint,
                Self::Aps(replay) => replay.key_fingerprint == key_fingerprint,
            },
            ReplayCounterTombstone::ReplayDomain(replay) => self.same_domain(&replay),
        }
    }
}

/// Durable replay domains that must be removed when their device or key is
/// revoked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayCounterTombstone {
    Device(IeeeAddress),
    KeyFingerprint(u32),
    /// Remove one exact replay domain while preserving another live domain
    /// that intentionally shares the same key material.
    ReplayDomain(PersistentReplayCounter),
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
    /// Whether the inactive NWK key slot is present.
    ///
    /// Before Switch-Key this is the future key; after activation it is the
    /// previous key retained for dual-key receive compatibility across reboot.
    pub staged_network_key_present: bool,
    pub staged_network_key: [u8; 16],
    pub staged_key_sequence: u8,
    /// The inactive key is the previous key retained after Switch-Key rather
    /// than a future key awaiting activation.
    pub secondary_network_key_is_previous: bool,
    /// A zero-destination standard Network-Key update requires parent fanout.
    ///
    /// Targets the staged key, or the active key after activation (never the
    /// retained previous RX key). Reset conservatively restarts the bounded
    /// profile-policy window; an addressed update must clear this intent.
    pub network_key_forwarding_pending: bool,
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
    /// BDB Table 6 link-key regime used to authenticate the initial network
    /// key. This also identifies centralized versus distributed security.
    pub node_join_link_key_type: NodeJoinLinkKeyType,
    /// A short PAN identifier accepted from a Network Update but not yet
    /// applied after `nwkNetworkBroadcastDeliveryTime`.
    pub pending_pan_id: Option<u16>,
    /// The local network manager still owes the network the corresponding
    /// Network Update broadcast.
    pub pending_pan_id_broadcast: bool,
    /// The current short address/parent checkpoint is durable, but its
    /// `Device_annce` has not yet completed successfully.
    pub device_announce_pending: bool,
    /// An unsecured centralized rejoin selected the current parent. Normal
    /// operation remains gated until that parent proves the active NWK key.
    pub parent_link_provisional: bool,
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
            secondary_network_key_is_previous: false,
            network_key_forwarding_pending: false,
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
            node_join_link_key_type: NodeJoinLinkKeyType::DefaultGlobalTrustCenterLinkKey,
            pending_pan_id: None,
            pending_pan_id_broadcast: false,
            device_announce_pending: false,
            parent_link_provisional: false,
        }
    }

    /// Whether this record describes a node that formed and owns the PAN.
    ///
    /// A coordinator owns short address `0x0000`, has depth zero, and has no
    /// parent. This shape is unambiguous in a Zigbee network and lets the
    /// existing record format represent a coordinator without inventing a
    /// self-referential Trust Center link key or consuming another format bit.
    pub(crate) const fn is_formed_network(&self) -> bool {
        self.short_address == 0x0000 && self.depth == 0 && self.parent_address == 0xFFFF
    }

    /// Whether this is a centralized network formed by its coordinator.
    pub(crate) const fn is_coordinator_network(&self) -> bool {
        self.is_formed_network() && !self.node_join_link_key_type.is_distributed()
    }

    /// Whether this is a distributed network formed by a router.
    #[cfg(any(feature = "router", test))]
    pub(crate) const fn is_distributed_network_owner(&self) -> bool {
        self.is_formed_network() && self.node_join_link_key_type.is_distributed()
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
        output[98] = self.node_join_link_key_type as u8;
        output[99] = u8::from(self.secondary_network_key_is_previous);
        output[100] = (if self.pending_pan_id.is_some() {
            LIFECYCLE_PENDING_PAN_ID
        } else {
            0
        }) | (if self.pending_pan_id_broadcast {
            LIFECYCLE_PENDING_PAN_ID_BROADCAST
        } else {
            0
        }) | (if self.device_announce_pending {
            LIFECYCLE_DEVICE_ANNOUNCE_PENDING
        } else {
            0
        }) | (if self.parent_link_provisional {
            LIFECYCLE_PARENT_LINK_PROVISIONAL
        } else {
            0
        }) | (if self.network_key_forwarding_pending {
            LIFECYCLE_NETWORK_KEY_FORWARDING
        } else {
            0
        });
        output[101..103].copy_from_slice(&self.pending_pan_id.unwrap_or(0).to_le_bytes());
    }

    pub fn decode(input: &[u8; ENCODED_SECURITY_STATE_LEN]) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V8)
    }

    pub(crate) fn decode_v7(
        input: &[u8; ENCODED_SECURITY_STATE_LEN],
    ) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V7)
    }

    pub(crate) fn decode_v6(
        input: &[u8; V6_ENCODED_SECURITY_STATE_LEN],
    ) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V6)
    }

    pub(crate) fn decode_v5(
        input: &[u8; V5_ENCODED_SECURITY_STATE_LEN],
    ) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V5)
    }

    /// Decode a version 4 record with explicit `nwkUpdateId` validity but no
    /// persisted BDB join-link-key type.
    pub(crate) fn decode_v4(
        input: &[u8; V4_ENCODED_SECURITY_STATE_LEN],
    ) -> Result<Self, SecurityStoreError> {
        Self::decode_bytes(input, StateFormat::V4)
    }

    /// Decode a version 3 record: same length as the current format, but
    /// without the `update_id_valid` bit.
    pub(crate) fn decode_v3(
        input: &[u8; V4_ENCODED_SECURITY_STATE_LEN],
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
        state.node_join_link_key_type = if format.has_node_join_link_key_type() {
            NodeJoinLinkKeyType::from_u8(input[98]).ok_or(SecurityStoreError::Corrupt)?
        } else if state.trust_center_address == [0xFF; 8] {
            NodeJoinLinkKeyType::DistributedSecurityGlobalLinkKey
        } else {
            NodeJoinLinkKeyType::DefaultGlobalTrustCenterLinkKey
        };
        state.secondary_network_key_is_previous = if format.has_secondary_key_role() {
            match input[99] {
                0 => false,
                1 => true,
                _ => return Err(SecurityStoreError::Corrupt),
            }
        } else {
            false
        };
        if format.has_nwk_lifecycle() {
            let lifecycle = input[100];
            let allowed = LIFECYCLE_ALLOWED_FLAGS
                | if format == StateFormat::V8 {
                    LIFECYCLE_NETWORK_KEY_FORWARDING
                } else {
                    0
                };
            if lifecycle & !allowed != 0 {
                return Err(SecurityStoreError::Corrupt);
            }
            state.pending_pan_id = if lifecycle & LIFECYCLE_PENDING_PAN_ID != 0 {
                Some(u16::from_le_bytes([input[101], input[102]]))
            } else {
                None
            };
            state.pending_pan_id_broadcast = lifecycle & LIFECYCLE_PENDING_PAN_ID_BROADCAST != 0;
            state.device_announce_pending = lifecycle & LIFECYCLE_DEVICE_ANNOUNCE_PENDING != 0;
            state.parent_link_provisional = lifecycle & LIFECYCLE_PARENT_LINK_PROVISIONAL != 0;
            state.network_key_forwarding_pending =
                lifecycle & LIFECYCLE_NETWORK_KEY_FORWARDING != 0;
        }

        state.validate()?;
        Ok(state)
    }

    pub fn validate(&self) -> Result<(), SecurityStoreError> {
        let formed_network = self.is_formed_network();
        let distributed = self.node_join_link_key_type.is_distributed();
        if self.commissioned
            && (!(11..=26).contains(&self.channel)
                || self.pan_id == 0xFFFF
                || self.short_address == 0xFFFF
                || self.ieee_address == [0; 8]
                || self.global_counter_limit == 0
                || !(formed_network
                    || distributed
                    || self.tclk_present
                    || self.legacy_default_tclk))
        {
            return Err(SecurityStoreError::Corrupt);
        }
        // The coordinator is the Trust Center; it never holds a unique TCLK
        // with itself and never has an end-device parent relationship. Keep
        // those representations disjoint so a corrupt leaf record cannot be
        // reinterpreted as a coordinator merely because it carries address 0.
        if formed_network
            && !distributed
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
        if distributed
            && (self.trust_center_address != [0xFF; 8]
                || self.tclk_present
                || self.legacy_default_tclk
                || self.trust_center_link_key != [0; 16]
                || self.tclk_counter_limit != 0
                || self.tclk_incoming_counter != 0
                || self.tclk_incoming_counter_valid)
        {
            return Err(SecurityStoreError::Corrupt);
        }
        if !distributed && self.trust_center_address == [0xFF; 8] {
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
            if !self.commissioned
                || self.staged_key_sequence == self.key_sequence
                || (self.secondary_network_key_is_previous
                    && self.staged_key_sequence != self.key_sequence.wrapping_sub(1))
            {
                return Err(SecurityStoreError::Corrupt);
            }
        } else if self.staged_key_sequence != 0
            || self.staged_network_key != [0; 16]
            || self.secondary_network_key_is_previous
        {
            return Err(SecurityStoreError::Corrupt);
        }
        if self.network_key_forwarding_pending
            && (!self.commissioned || distributed || !self.staged_network_key_present)
        {
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
        if let Some(pending_pan_id) = self.pending_pan_id {
            if !self.commissioned
                || !self.update_id_valid
                || pending_pan_id == 0
                || pending_pan_id == 0xFFFF
                || pending_pan_id == self.pan_id
            {
                return Err(SecurityStoreError::Corrupt);
            }
        } else if self.pending_pan_id_broadcast {
            return Err(SecurityStoreError::Corrupt);
        }
        if self.device_announce_pending && (!self.commissioned || formed_network) {
            return Err(SecurityStoreError::Corrupt);
        }
        if self.parent_link_provisional && (!self.commissioned || formed_network || distributed) {
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
    fn visit_replay_counters(
        &mut self,
        _visitor: &mut dyn FnMut(PersistentReplayCounter),
    ) -> Result<(), SecurityStoreError> {
        Ok(())
    }
    fn commit_replay_counter(
        &mut self,
        _replay: PersistentReplayCounter,
    ) -> Result<(), SecurityStoreError> {
        // A legacy snapshot-only backend cannot safely acknowledge or act on
        // an authenticated frame whose replay floor is still volatile.
        Err(SecurityStoreError::Hardware)
    }
    fn tombstone_replay_counters(
        &mut self,
        _tombstone: ReplayCounterTombstone,
    ) -> Result<(), SecurityStoreError> {
        // Replay cleanup is part of key/device revocation. A backend that
        // cannot make it durable must fail closed rather than retain an
        // unbounded stale replay domain.
        Err(SecurityStoreError::Hardware)
    }
    fn retain_replay_counters(
        &mut self,
        retain: &dyn Fn(PersistentReplayCounter) -> bool,
    ) -> Result<usize, SecurityStoreError> {
        let mut removed = 0usize;
        loop {
            let mut stale = None;
            self.visit_replay_counters(&mut |replay| {
                if stale.is_none() && !retain(replay) {
                    stale = Some(replay);
                }
            })?;
            let Some(stale) = stale else {
                return Ok(removed);
            };
            self.tombstone_replay_counters(ReplayCounterTombstone::ReplayDomain(stale))?;
            removed = removed.saturating_add(1);
        }
    }
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

#[cfg(any(feature = "router", test))]
impl<S: SecurityStateStore> FormationPersistence for CommissioningSecurityPersistence<'_, S> {
    fn commit_formed_network(
        &mut self,
        state: &NetworkSecurityState,
    ) -> Result<CounterReservation, SecurityPersistenceError> {
        let reservation = <Self as SecurityPersistence>::reserve_network_security(self, state)?;

        if state.node_join_link_key_type.is_distributed() {
            <Self as SecurityPersistence>::commit_distributed_network(self)?;
        } else {
            if !self.state.is_coordinator_network()
                || self.state.tclk_present
                || self.state.legacy_default_tclk
            {
                return Err(SecurityPersistenceError::InvalidState);
            }
            self.state.commissioned = true;
            self.state.rejoin_pending = false;
            self.state
                .validate()
                .map_err(|_| SecurityPersistenceError::InvalidState)?;
            self.persist()?;
        }

        Ok(reservation)
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
        self.state.secondary_network_key_is_previous = false;
        self.state.network_key_forwarding_pending = false;
        self.state.global_counter_limit = reservation.limit;
        self.state.node_join_link_key_type = state.node_join_link_key_type;
        let formed_network =
            state.short_address == 0x0000 && state.depth == 0 && state.parent_address == 0xFFFF;
        self.state.trust_center_address =
            if formed_network && !state.node_join_link_key_type.is_distributed() {
                [0; 8]
            } else {
                state.trust_center_address
            };
        self.state.tclk_present = false;
        self.state.legacy_default_tclk = false;
        self.state.trust_center_link_key = [0; 16];
        if state.node_join_link_key_type.is_distributed() {
            self.state.tclk_counter_limit = 0;
        }
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
        if self.state.node_join_link_key_type.is_distributed() {
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
        if self.state.node_join_link_key_type.is_distributed()
            || !self.state.tclk_present
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

    fn commit_distributed_network(&mut self) -> Result<(), SecurityPersistenceError> {
        if !self.state.node_join_link_key_type.is_distributed()
            || self.state.trust_center_address != [0xFF; 8]
            || self.state.tclk_present
            || self.state.legacy_default_tclk
        {
            return Err(SecurityPersistenceError::InvalidState);
        }
        self.state.commissioned = true;
        self.state.rejoin_pending = false;
        self.state
            .validate()
            .map_err(|_| SecurityPersistenceError::InvalidState)?;
        self.persist()
    }
}

/// In-memory store for tests.
pub struct RamSecurityStateStore {
    state: Option<PersistentSecurityState>,
    replay: heapless::Vec<PersistentReplayCounter, MAX_PERSISTENT_REPLAY_COUNTERS>,
}

impl RamSecurityStateStore {
    pub const fn new() -> Self {
        Self {
            state: None,
            replay: heapless::Vec::new(),
        }
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
        let preserve_replay = self.state.is_some_and(|current| {
            current.commissioned
                && state.commissioned
                && current.extended_pan_id == state.extended_pan_id
                && current.ieee_address == state.ieee_address
        });
        if !preserve_replay {
            self.replay.clear();
        }
        self.state = Some(*state);
        Ok(())
    }

    fn visit_replay_counters(
        &mut self,
        visitor: &mut dyn FnMut(PersistentReplayCounter),
    ) -> Result<(), SecurityStoreError> {
        for replay in self.replay.iter().copied() {
            visitor(replay);
        }
        Ok(())
    }

    fn commit_replay_counter(
        &mut self,
        replay: PersistentReplayCounter,
    ) -> Result<(), SecurityStoreError> {
        if !self.state.is_some_and(|state| state.commissioned) {
            return Err(SecurityStoreError::Corrupt);
        }
        if let Some(stored) = self
            .replay
            .iter_mut()
            .find(|stored| stored.same_domain(&replay))
        {
            if replay.counter() > stored.counter() {
                *stored = replay;
            }
            return Ok(());
        }
        self.replay
            .push(replay)
            .map_err(|_| SecurityStoreError::Full)
    }

    fn tombstone_replay_counters(
        &mut self,
        tombstone: ReplayCounterTombstone,
    ) -> Result<(), SecurityStoreError> {
        self.replay
            .retain(|replay| !replay.matches_tombstone(tombstone));
        Ok(())
    }

    fn retain_replay_counters(
        &mut self,
        retain: &dyn Fn(PersistentReplayCounter) -> bool,
    ) -> Result<usize, SecurityStoreError> {
        let previous_len = self.replay.len();
        self.replay.retain(|replay| retain(*replay));
        Ok(previous_len - self.replay.len())
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
            trust_center_address: [6; 8],
            node_join_link_key_type: NodeJoinLinkKeyType::DefaultGlobalTrustCenterLinkKey,
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
        state.node_join_link_key_type = NodeJoinLinkKeyType::InstallCodeDerivedPreconfiguredLinkKey;
        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);
        assert_eq!(encoded[98], 0x02);
        assert_eq!(encoded[99], 0);
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(state));
    }

    #[test]
    fn previous_network_key_role_round_trips_and_rejects_reserved_values() {
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [1; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = [2; 8];
        state.channel = 15;
        state.network_key = [3; 16];
        state.key_sequence = 0;
        state.staged_network_key_present = true;
        state.staged_network_key = [8; 16];
        state.staged_key_sequence = 0xFF;
        state.secondary_network_key_is_previous = true;
        state.global_counter_limit = 0x400;
        state.legacy_default_tclk = true;
        state.tclk_counter_limit = 0x400;
        let mut encoded = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded);
        assert_eq!(encoded[99], 1);
        assert_eq!(PersistentSecurityState::decode(&encoded), Ok(state));

        encoded[99] = 2;
        assert_eq!(
            PersistentSecurityState::decode(&encoded),
            Err(SecurityStoreError::Corrupt)
        );
    }

    #[test]
    fn distributed_network_commits_without_a_trust_center_key() {
        let mut store = RamSecurityStateStore::new();
        let mut persistence = CommissioningSecurityPersistence::new(&mut store).unwrap();
        let mut network = network_state(0);
        network.trust_center_address = [0xFF; 8];
        network.node_join_link_key_type = NodeJoinLinkKeyType::DistributedSecurityGlobalLinkKey;

        assert!(
            persistence
                .reserve_network_security(&network)
                .unwrap()
                .is_valid()
        );
        assert_eq!(persistence.commit_distributed_network(), Ok(()));

        let saved = store.load().unwrap().unwrap();
        assert!(saved.commissioned);
        assert_eq!(saved.trust_center_address, [0xFF; 8]);
        assert_eq!(
            saved.node_join_link_key_type,
            NodeJoinLinkKeyType::DistributedSecurityGlobalLinkKey
        );
        assert!(!saved.tclk_present);
        assert_eq!(saved.tclk_counter_limit, 0);
        assert_eq!(saved.validate(), Ok(()));
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

    /// Version 3 and version 4 share the old 98-byte layout.
    #[test]
    fn a_v3_record_has_no_validity_bit_and_stays_authoritative() {
        let mut state = PersistentSecurityState::empty();
        state.update_id = 0x2A;
        state.update_id_valid = true;
        state.parent_information = 0x02;
        state.parent_information_valid = true;
        state.end_device_timeout = 14;
        let mut current = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut current);
        let mut encoded = [0u8; V4_ENCODED_SECURITY_STATE_LEN];
        encoded.copy_from_slice(&current[..V4_ENCODED_SECURITY_STATE_LEN]);

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
        assert_eq!(PersistentSecurityState::decode_v4(&encoded), Ok(state));
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
