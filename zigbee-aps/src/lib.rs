//! Zigbee PRO R22 Application Support Sub-layer (APS).
//!
//! This crate implements the APS layer of the Zigbee stack, providing:
//! - APS frame construction and parsing
//! - APS Data Entity (APSDE-DATA) service
//! - APS Management Entity (APSME) — binding, group, key management
//! - APS Information Base (AIB)
//! - APS-level security (link key encryption)
//!
//! # Architecture
//! ```text
//! ┌──────────────────────────────────────┐
//! │  ZDO / ZCL / Application             │
//! └──────────────┬───────────────────────┘
//!                │ APSDE-DATA / APSME-*
//! ┌──────────────┴───────────────────────┐
//! │  APS Layer (this crate)              │
//! │  ├── apsde: data service             │
//! │  ├── apsme: management entity        │
//! │  ├── aib: APS information base       │
//! │  ├── frames: APS frame codec         │
//! │  ├── binding: binding table          │
//! │  ├── group: group table              │
//! │  └── security: APS encryption        │
//! └──────────────┬───────────────────────┘
//!                │ NLDE-DATA / NLME-*
//! ┌──────────────┴───────────────────────┐
//! │  NWK Layer (zigbee-nwk)              │
//! └──────────────────────────────────────┘
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

#[cfg(test)]
extern crate std;

pub mod aib;
pub mod apsde;
pub mod apsme;
pub mod binding;
pub mod fragment;
#[cfg(feature = "fragmentation")]
mod fragment_tx;
pub mod frames;
pub mod group;
pub mod security;

use zigbee_mac::MacDriver;
use zigbee_nwk::NwkLayer;

/// Volatile wipe of secret material (re-exported from `zigbee-crypto`).
pub(crate) use zigbee_crypto::zeroize;

/// Stack buffer holding secret material (plaintext keys, derived keys,
/// key-bearing command payloads) that is wiped on drop, on every return path.
pub(crate) struct Secret<const N: usize>(pub(crate) [u8; N]);

impl<const N: usize> Secret<N> {
    pub(crate) const fn zeroed() -> Self {
        Self([0u8; N])
    }
}

impl<const N: usize> Drop for Secret<N> {
    fn drop(&mut self) {
        zeroize(&mut self.0);
    }
}

impl<const N: usize> core::ops::Deref for Secret<N> {
    type Target = [u8; N];
    fn deref(&self) -> &[u8; N] {
        &self.0
    }
}

impl<const N: usize> core::ops::DerefMut for Secret<N> {
    fn deref_mut(&mut self) -> &mut [u8; N] {
        &mut self.0
    }
}

// ── Well-known endpoints ────────────────────────────────────────

/// ZDO endpoint (Zigbee Device Object)
pub const ZDO_ENDPOINT: u8 = 0x00;

/// Minimum application endpoint
pub const MIN_APP_ENDPOINT: u8 = 0x01;

/// Maximum application endpoint
pub const MAX_APP_ENDPOINT: u8 = 0xF0;

/// Broadcast endpoint — delivers to all active endpoints on a device
pub const BROADCAST_ENDPOINT: u8 = 0xFF;

// ── Well-known profile IDs ──────────────────────────────────────

/// Zigbee Device Profile (ZDP)
pub const PROFILE_ZDP: u16 = 0x0000;

/// Home Automation profile
pub const PROFILE_HOME_AUTOMATION: u16 = 0x0104;

/// Smart Energy profile
pub const PROFILE_SMART_ENERGY: u16 = 0x0109;

/// Zigbee Light Link (ZLL) profile
pub const PROFILE_ZLL: u16 = 0xC05E;

/// Wildcard profile — matches any profile
pub const PROFILE_WILDCARD: u16 = 0xFFFF;

// ── APS Status Codes (Zigbee spec Table 2-27) ──────────────────

/// APS layer status codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ApsStatus {
    /// Request executed successfully
    Success = 0x00,
    /// A transmit request failed since the ASDU is too large and fragmentation
    /// is not supported
    AsduTooLong = 0xA0,
    /// A received fragmented frame could not be defragmented
    DefragDeferred = 0xA1,
    /// A received fragmented frame could not be defragmented because the device
    /// does not support fragmentation
    DefragUnsupported = 0xA2,
    /// A parameter value was out of range
    IllegalRequest = 0xA3,
    /// An APSME-UNBIND.request failed because the requested binding table
    /// entry was not found
    InvalidBinding = 0xA4,
    /// An APSME-GET/SET request was issued with an unknown attribute identifier
    InvalidParameter = 0xA5,
    /// An APSDE-DATA.request requesting acknowledged transmission failed due
    /// to no acknowledgement being received
    NoAck = 0xA6,
    /// An APSDE-DATA.request with a destination addressing mode set to 0x00
    /// failed due to there being no devices bound to this device
    NoBoundDevice = 0xA7,
    /// An APSDE-DATA.request with a destination addressing mode set to 0x03
    /// failed because no matching group table entry could be found
    NoShortAddress = 0xA8,
    /// An APSME-BIND.request or APSME-ADD-GROUP.request issued when the
    /// binding/group table is full
    TableFull = 0xA9,
    /// An ASDU was received that was secured using a link key but a link key
    /// was not found in the key table
    UnsecuredKey = 0xAA,
    /// An APSME-GET.request or APSME-SET.request has been issued with an
    /// unsupported attribute identifier
    UnsupportedAttribute = 0xAB,
    /// An unsecured frame was received
    SecurityFail = 0xAD,
    /// Decryption or authentication of the APS frame failed
    DecryptionError = 0xAE,
    /// Not enough buffers for the requested operation
    InsufficientSpace = 0xAF,
    /// No matching entry in binding table
    NotFound = 0xB0,
}

// ── APS address modes ───────────────────────────────────────────

/// APS addressing modes (Zigbee spec Table 2-3)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ApsAddressMode {
    /// Indirect (via binding table)
    Indirect = 0x00,
    /// Group addressing (16-bit group address)
    Group = 0x01,
    /// Direct short (16-bit NWK address + endpoint)
    Short = 0x02,
    /// Direct extended (64-bit IEEE address + endpoint)
    Extended = 0x03,
}

// ── APS address ─────────────────────────────────────────────────

/// Destination/source address used in APS primitives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApsAddress {
    /// 16-bit NWK short address
    Short(zigbee_types::ShortAddress),
    /// 64-bit IEEE extended address
    Extended(zigbee_types::IeeeAddress),
    /// 16-bit group address
    Group(u16),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApsSecurityHandshakeStats {
    pub verify_key_sent: u32,
    /// Legacy APS security frame-counter diagnostic.
    ///
    /// R22 requires Verify-Key to be sent without APS encryption, so compliant
    /// transmissions leave this at zero and do not consume the Trust Center
    /// link key's outgoing security counter.
    pub last_verify_key_frame_counter: u32,
    pub last_verify_key_trust_center: zigbee_types::IeeeAddress,
    /// APS counter carried by the most recent Verify-Key transmission.
    ///
    /// This is the ordinary one-octet APS sequence counter, not an APS security
    /// frame counter. It is retained for wire diagnostics only; an
    /// acknowledgement of Verify-Key is never proof of link-key possession.
    pub last_verify_key_aps_counter: u8,
    pub confirm_key_received: u32,
    pub confirm_key_successes: u32,
    pub confirm_key_rejections: u32,
    /// Confirm-Key frames that did not authenticate as coming from the
    /// configured Trust Center under the unique Trust Center link key, or that
    /// were malformed or addressed elsewhere.
    ///
    /// Kept separate on purpose: the three counters above are read by the BDB
    /// key-exchange state machine as security predicates (a rejection leaves
    /// the network), so an unauthenticated frame must never move them. This
    /// one is pure diagnostics — a forged or stray Confirm-Key is visible
    /// without being able to influence commissioning.
    pub confirm_key_ignored: u32,
    pub last_confirm_key_status: u8,
    pub last_confirm_key_type: u8,
    pub last_confirm_key_key_identifier: u8,
    pub last_confirm_key_aps_secured: bool,
    pub last_confirm_key_nwk_secured: bool,
    pub last_confirm_key_source: u16,
    pub last_confirm_key_source_ieee: zigbee_types::IeeeAddress,
    pub last_confirm_key_destination: zigbee_types::IeeeAddress,
    /// Malformed or unauthenticated Trust Center command indications dropped
    /// before reaching the runtime.
    pub security_commands_ignored: u32,
    /// Valid indications dropped because the single upper-layer handoff slot
    /// was still occupied.
    pub security_indications_dropped: u32,
}

impl Default for ApsSecurityHandshakeStats {
    fn default() -> Self {
        Self {
            verify_key_sent: 0,
            last_verify_key_frame_counter: 0,
            last_verify_key_trust_center: [0u8; 8],
            last_verify_key_aps_counter: 0,
            confirm_key_received: 0,
            confirm_key_successes: 0,
            confirm_key_rejections: 0,
            confirm_key_ignored: 0,
            last_confirm_key_status: 0xFF,
            last_confirm_key_type: 0xFF,
            last_confirm_key_key_identifier: 0xFF,
            last_confirm_key_aps_secured: false,
            last_confirm_key_nwk_secured: false,
            last_confirm_key_source: 0xFFFF,
            last_confirm_key_source_ieee: [0u8; 8],
            last_confirm_key_destination: [0u8; 8],
            security_commands_ignored: 0,
            security_indications_dropped: 0,
        }
    }
}

/// Link-key regime that authenticated the initial Network-Key Transport-Key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkKeyJoinMethod {
    /// Centralized network using the configured precommissioned global key.
    CentralizedPreconfiguredGlobal,
    /// Centralized network using a partner-specific preconfigured key entry.
    CentralizedKeyPair,
    /// Distributed network using the distributed-security global key.
    DistributedSecurityGlobal,
}

// ── TX Options ──────────────────────────────────────────────────

/// APSDE-DATA.request TX options bitfield.
#[derive(Debug, Clone, Copy, Default)]
pub struct ApsTxOptions {
    /// Use APS-level security (link key encryption)
    pub security_enabled: bool,
    /// Use NWK key (standard NWK encryption)
    pub use_nwk_key: bool,
    /// Request APS acknowledgement
    pub ack_request: bool,
    /// Enable fragmentation
    pub fragmentation_permitted: bool,
    /// Include extended nonce in APS security frame
    pub include_extended_nonce: bool,
}

// ── The APS Layer ───────────────────────────────────────────────

/// Pending APS ACK to be sent (queued during receive processing).
#[derive(Debug, Clone)]
pub struct PendingApsAck {
    pub dst_addr: zigbee_types::ShortAddress,
    pub dst_endpoint: u8,
    pub src_endpoint: u8,
    pub cluster_id: u16,
    pub profile_id: u16,
    pub aps_counter: u8,
    /// Acknowledgement-format sub-field (R22 §2.2.5.1.1.5).
    ///
    /// `true` acknowledges an APS *command* frame: the ACK carries neither
    /// endpoints nor cluster/profile identifiers. `false` acknowledges a data
    /// frame and echoes the addressing fields.
    pub command: bool,
    /// Windowed acknowledgement of a fragmented transaction
    /// (R22 §2.2.5.2.3): when `Some`, the ACK carries the extended header with
    /// the block number of the receive window and the ACK bitfield.
    pub fragment: Option<fragment::FragmentAck>,
    /// APS security of the acknowledged data frame. When `Some`, the ACK is
    /// APS-secured with the same key, so the sender can authenticate that the
    /// acknowledgement came from the key holder.
    pub aps_security: Option<security::ApsKeyOrigin>,
}

/// An APS unicast awaiting retransmission after its acknowledgement window.
///
/// Carries the destination alongside the frame so the retry repeats the
/// original *unicast* transmission (R22 §2.2.5.2.2).
#[derive(Debug, Clone)]
pub struct ApsRetransmission {
    pub dst_addr: zigbee_types::ShortAddress,
    pub frame: heapless::Vec<u8, 128>,
}

/// Stable identity of one acknowledged APS unicast.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApsAckHandle {
    dst_addr: u16,
    generation: u16,
    aps_counter: u8,
}

impl ApsAckHandle {
    const fn new(dst_addr: u16, aps_counter: u8, generation: u16) -> Self {
        Self {
            dst_addr,
            generation,
            aps_counter,
        }
    }

    pub const fn destination(self) -> zigbee_types::ShortAddress {
        zigbee_types::ShortAddress(self.dst_addr)
    }

    pub const fn aps_counter(self) -> u8 {
        self.aps_counter
    }
}

/// Current delivery state of an acknowledged APS unicast.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApsAckStatus {
    Pending,
    Confirmed,
}

/// APS Tunnel command awaiting hop-by-hop delivery to a joining child.
#[cfg(any(feature = "router", test))]
#[derive(Debug, Clone)]
pub struct PendingApsTunnel {
    pub destination: zigbee_types::IeeeAddress,
    frame: heapless::Vec<u8, 128>,
}

#[cfg(any(feature = "router", test))]
impl PendingApsTunnel {
    pub fn frame(&self) -> &[u8] {
        self.frame.as_slice()
    }
}

/// Maximum entries in APS duplicate rejection table
#[cfg(feature = "router")]
const APS_DUP_TABLE_SIZE: usize = 16;
#[cfg(not(feature = "router"))]
const APS_DUP_TABLE_SIZE: usize = 4;

/// Maximum entries in outbound APS ACK tracking table
#[cfg(feature = "router")]
const APS_ACK_TABLE_SIZE: usize = 8;
#[cfg(not(feature = "router"))]
const APS_ACK_TABLE_SIZE: usize = 4;

/// `apscAckWaitDuration` — how long a transmitter waits for an APS
/// acknowledgement before retransmitting (R22 Table 2-24).
///
/// The normative value is `0.05 * 2 * nwkcMaxDepth` seconds. This stack's
/// `nwkcMaxDepth` is 15 (the `max_depth` default in [`zigbee_nwk::nib`]), so
/// the wait is 1.5 s.
///
/// It is expressed in microseconds because that is the unit of the stack's
/// only monotonic clock ([`zigbee_mac::PlatformServices::monotonic_micros`]).
/// Deriving the wait from that clock — rather than from how often the
/// application happens to call [`ApsLayer::age_ack_table`] — is what keeps the
/// retry interval identical on a 1 s router tick and on a sleepy device that
/// runs maintenance far more often.
pub const APS_ACK_WAIT_DURATION_US: u32 = 1_500_000;

/// APS duplicate rejection entry (R22 §2.2.8.4.2).
///
/// Holds the source address, the APS counter and *timing information*: the
/// monotonic timestamp at which the frame was accepted. Expiry compares that
/// timestamp with `apsDuplicateRejectionTimeout` (milliseconds) on the
/// platform's monotonic clock, so it no longer depends on how often the
/// runtime happens to call [`ApsLayer::age_dup_table`].
#[derive(Debug, Clone, Copy)]
struct ApsDuplicateEntry {
    src_addr: u16,
    aps_counter: u8,
    /// `Some(window)` for a *completed fragmented* transaction: every later
    /// block with this (source, APS counter) is a retransmission caused by a
    /// lost final acknowledgement, and `window` is the first block of the
    /// final receive window that acknowledgement must repeat
    /// (R22 §2.2.8.4.5.2). `None` for an unfragmented frame.
    fragment_window: Option<u8>,
    accepted_at_us: u32,
    active: bool,
}

impl ApsDuplicateEntry {
    const fn empty() -> Self {
        Self {
            src_addr: 0,
            aps_counter: 0,
            fragment_window: None,
            accepted_at_us: 0,
            active: false,
        }
    }
}

/// Reception state recorded for the frame currently awaiting an upper-layer
/// durable commit, so an abort can undo it (see
/// [`ApsLayer::abort_data_persistence`]).
#[derive(Debug, Clone, Copy)]
enum ApsRxRollback {
    /// Duplicate-rejection entry recorded for an accepted frame.
    Duplicate { src_addr: u16, aps_counter: u8 },
    /// A fragment block recorded in an in-progress reassembly session.
    #[cfg(feature = "fragmentation")]
    FragmentBlock {
        src_addr: u16,
        aps_counter: u8,
        block: u8,
    },
}

/// Tracks an outbound APS frame that requested an ACK.
#[derive(Debug, Clone)]
struct PendingApsAckEntry {
    /// Whether this slot is in use
    active: bool,
    /// Monotonic in-RAM identity that prevents a stale handle from matching a
    /// later transaction which reused the same destination and APS counter.
    generation: u16,
    /// APS counter of the sent frame
    aps_counter: u8,
    /// Destination short address
    dst_addr: u16,
    /// Whether an ACK has been received
    confirmed: bool,
    /// Remaining retries (decremented each timeout tick)
    retries: u8,
    /// Monotonic microsecond timestamp of the transmission this entry is
    /// currently waiting on — the original send, or the most recent retry.
    ///
    /// A single `u32` per entry (16 bytes for the whole 4-entry sensor table)
    /// is what turns [`ApsLayer::age_ack_table`] from "retransmit on every
    /// call" into a real [`APS_ACK_WAIT_DURATION_US`] timer. Compared with the
    /// monotonic clock by wrapping subtraction, so a `u32` microsecond counter
    /// rolling over (every ~71.6 min) is handled without any extra state.
    waiting_since_us: u32,
    /// Frame bytes held for retransmission.
    ///
    /// For an unsecured frame these are the exact bytes that went out. For an
    /// APS-secured frame they are the *plaintext* form — APS header followed
    /// by the unencrypted payload — because every retry has to be secured
    /// again with a fresh frame counter (see [`PendingApsSecurity`]).
    original_frame: heapless::Vec<u8, 128>,
    /// APS security context of the original transmission, when it was secured.
    security: Option<PendingApsSecurity>,
}

/// Everything needed to re-secure an APS data retransmission.
///
/// Only APSDE data registers this context, using `APS_DEFAULT_EXT_NONCE`.
/// R22 security commands never enter the retransmission table.
///
/// R22 §4.4.1.1 derives the CCM* nonce from (source address, frame counter,
/// security control) and requires a frame counter never to be reused with the
/// same key. Re-sending the stored ciphertext therefore repeats a nonce, and
/// any receiver that implements APS replay protection correctly (including
/// this stack, see `ApsSecurity::check_frame_counter_for`) drops every retry —
/// so a secured unicast that lost its first copy could never be acknowledged.
/// Keeping the security context instead lets the retry be encrypted again with
/// the next counter of the *same key-pair entry*, while the APS header — and
/// with it the APS counter the acknowledgement and duplicate rejection are
/// keyed on (R22 §2.2.5.1.1.5) — is byte-identical.
#[derive(Debug, Clone, Copy)]
struct PendingApsSecurity {
    /// Key-pair entry (or preconfigured global key) that secured the frame.
    origin: security::ApsKeyOrigin,
    /// Local IEEE address that forms the CCM* nonce source address.
    src_ieee: zigbee_types::IeeeAddress,
    /// Length of the APS header prefix inside `original_frame`.
    header_len: u8,
}

/// Accepted parent Network-Key forwarding change awaiting a durable snapshot.
#[cfg(feature = "router")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetworkKeyForwardingIntent {
    /// A later standard/high-security Network-Key transport superseded fanout.
    Stop,
    /// A standard Network-Key descriptor with an all-zero destination.
    Forward(u8),
}

/// The APS layer — owns the NWK layer and all APS state.
///
/// Generic over `M: MacDriver` (the hardware abstraction).
pub struct ApsLayer<M: MacDriver> {
    /// Underlying NWK layer
    nwk: NwkLayer<M>,
    /// APS Information Base
    aib: aib::Aib,
    /// Binding table
    binding_table: binding::BindingTable,
    /// Group table
    group_table: group::GroupTable,
    /// APS security material
    security: security::ApsSecurity,
    /// APS frame counter (outgoing)
    aps_counter: u8,
    security_handshake_stats: ApsSecurityHandshakeStats,
    /// Pending APS ACK to send after processing incoming frame
    pending_aps_ack: Option<PendingApsAck>,
    /// Pending APS Tunnel payload to forward without NWK security.
    #[cfg(any(feature = "router", test))]
    pending_tunnel: Option<PendingApsTunnel>,
    /// Protection that authenticated the most recently installed initial NWK key.
    network_key_join_method: Option<NetworkKeyJoinMethod>,
    /// Parsed Trust Center / parent security command for the runtime.
    #[cfg(feature = "router")]
    pending_security_indication: Option<apsme::ApsmeSecurityIndication>,
    /// Its durable upper-layer mutation has not completed yet.
    #[cfg(feature = "router")]
    pending_security_indication_persistence: bool,
    /// APS replay floor held until that upper-layer mutation is durable.
    #[cfg(feature = "router")]
    pending_security_indication_replay: Option<security::ApsReplayCounter>,
    /// Authenticated Trust Center Remove-Device targeted at this device.
    pending_local_remove_device: bool,
    /// Whether Transport-Key may install an application link key.
    application_link_key_installation_enabled: bool,
    /// Whether a Network Transport-Key must wait for an external security
    /// snapshot before its replay floor becomes live.
    network_key_persistence_enabled: bool,
    /// A newly installed network key awaits that durable snapshot.
    pending_network_key_persistence: bool,
    /// Replay floor held until the network key snapshot is durable.
    pending_network_key_replay: Option<security::ApsReplayCounter>,
    #[cfg(feature = "router")]
    network_key_forwarding_intent: Option<NetworkKeyForwardingIntent>,
    /// A newly installed application link key awaits durable APS-table storage.
    pending_application_key_persistence: bool,
    /// Replay floor held until the corresponding application key is durable.
    pending_application_key_replay: Option<security::ApsReplayCounter>,
    /// Verified data-frame replay floor held for an upper-layer durable mutation.
    pending_data_replay: Option<security::ApsReplayCounter>,
    /// The held replay floor (at most one is outstanding) was admitted into a
    /// new durable replay domain and so reserves one unit of its capacity.
    pending_replay_new_domain: bool,
    /// APS duplicate rejection table
    dup_table: [ApsDuplicateEntry; APS_DUP_TABLE_SIZE],
    /// Reception state to undo if the current frame's durable commit aborts.
    pending_rx_rollback: Option<ApsRxRollback>,
    /// Outgoing fragmented transaction (R22 §2.2.8.4.5.1).
    #[cfg(feature = "fragmentation")]
    fragment_tx: Option<fragment_tx::FragmentTxSession>,
    /// Final APSDE-DATA.confirm of the last fragmented transaction.
    fragment_tx_confirm: Option<apsde::ApsdeDataConfirm>,
    /// Outbound APS ACK tracking (frames awaiting ACK confirmation)
    ack_table: heapless::Vec<PendingApsAckEntry, APS_ACK_TABLE_SIZE>,
    /// Recently confirmed handles retained for durable upper-layer delivery.
    ack_completions: heapless::Vec<ApsAckHandle, APS_ACK_TABLE_SIZE>,
    /// Identity assigned to the next acknowledged outbound transaction.
    next_ack_generation: u16,
    /// Fragment reassembly buffer for incoming fragmented frames
    fragment_rx: fragment::FragmentReassembly,
}

impl<M: MacDriver> ApsLayer<M> {
    /// Create a new APS layer wrapping the given NWK layer.
    #[inline(never)]
    pub fn new(nwk: NwkLayer<M>) -> Self {
        Self {
            nwk,
            aib: aib::Aib::new(),
            binding_table: binding::BindingTable::new(),
            group_table: group::GroupTable::new(),
            security: security::ApsSecurity::new(),
            aps_counter: 0,
            security_handshake_stats: ApsSecurityHandshakeStats::default(),
            pending_aps_ack: None,
            #[cfg(any(feature = "router", test))]
            pending_tunnel: None,
            network_key_join_method: None,
            #[cfg(feature = "router")]
            pending_security_indication: None,
            #[cfg(feature = "router")]
            pending_security_indication_persistence: false,
            #[cfg(feature = "router")]
            pending_security_indication_replay: None,
            pending_local_remove_device: false,
            application_link_key_installation_enabled: false,
            network_key_persistence_enabled: false,
            pending_network_key_persistence: false,
            pending_network_key_replay: None,
            #[cfg(feature = "router")]
            network_key_forwarding_intent: None,
            pending_application_key_persistence: false,
            pending_application_key_replay: None,
            pending_data_replay: None,
            pending_replay_new_domain: false,
            dup_table: [ApsDuplicateEntry::empty(); APS_DUP_TABLE_SIZE],
            pending_rx_rollback: None,
            #[cfg(feature = "fragmentation")]
            fragment_tx: None,
            fragment_tx_confirm: None,
            ack_table: heapless::Vec::new(),
            ack_completions: heapless::Vec::new(),
            next_ack_generation: 1,
            fragment_rx: fragment::FragmentReassembly::new(),
        }
    }

    /// Construct an APS layer directly into caller-provided storage.
    ///
    /// # Safety
    /// `slot` must point to valid, properly aligned, uninitialized storage for `Self`.
    #[inline(never)]
    pub unsafe fn write_into(slot: *mut Self, mac: M, device_type: zigbee_nwk::DeviceType) {
        unsafe {
            NwkLayer::write_into(core::ptr::addr_of_mut!((*slot).nwk), mac, device_type);
            core::ptr::addr_of_mut!((*slot).aib).write(aib::Aib::new());
            core::ptr::addr_of_mut!((*slot).binding_table).write(binding::BindingTable::new());
            core::ptr::addr_of_mut!((*slot).group_table).write(group::GroupTable::new());
            core::ptr::addr_of_mut!((*slot).security).write(security::ApsSecurity::new());
            core::ptr::addr_of_mut!((*slot).aps_counter).write(0);
            core::ptr::addr_of_mut!((*slot).security_handshake_stats)
                .write(ApsSecurityHandshakeStats::default());
            core::ptr::addr_of_mut!((*slot).pending_aps_ack).write(None);
            #[cfg(any(feature = "router", test))]
            core::ptr::addr_of_mut!((*slot).pending_tunnel).write(None);
            core::ptr::addr_of_mut!((*slot).network_key_join_method).write(None);
            #[cfg(feature = "router")]
            core::ptr::addr_of_mut!((*slot).pending_security_indication).write(None);
            #[cfg(feature = "router")]
            core::ptr::addr_of_mut!((*slot).pending_security_indication_persistence).write(false);
            #[cfg(feature = "router")]
            core::ptr::addr_of_mut!((*slot).pending_security_indication_replay).write(None);
            core::ptr::addr_of_mut!((*slot).pending_local_remove_device).write(false);
            core::ptr::addr_of_mut!((*slot).application_link_key_installation_enabled).write(false);
            core::ptr::addr_of_mut!((*slot).network_key_persistence_enabled).write(false);
            core::ptr::addr_of_mut!((*slot).pending_network_key_persistence).write(false);
            core::ptr::addr_of_mut!((*slot).pending_network_key_replay).write(None);
            #[cfg(feature = "router")]
            core::ptr::addr_of_mut!((*slot).network_key_forwarding_intent).write(None);
            core::ptr::addr_of_mut!((*slot).pending_application_key_persistence).write(false);
            core::ptr::addr_of_mut!((*slot).pending_application_key_replay).write(None);
            core::ptr::addr_of_mut!((*slot).pending_data_replay).write(None);
            core::ptr::addr_of_mut!((*slot).pending_replay_new_domain).write(false);
            core::ptr::addr_of_mut!((*slot).dup_table)
                .write([ApsDuplicateEntry::empty(); APS_DUP_TABLE_SIZE]);
            core::ptr::addr_of_mut!((*slot).pending_rx_rollback).write(None);
            #[cfg(feature = "fragmentation")]
            core::ptr::addr_of_mut!((*slot).fragment_tx).write(None);
            core::ptr::addr_of_mut!((*slot).fragment_tx_confirm).write(None);
            core::ptr::addr_of_mut!((*slot).ack_table).write(heapless::Vec::new());
            core::ptr::addr_of_mut!((*slot).ack_completions).write(heapless::Vec::new());
            core::ptr::addr_of_mut!((*slot).next_ack_generation).write(1);
            core::ptr::addr_of_mut!((*slot).fragment_rx).write(fragment::FragmentReassembly::new());
        }
    }

    /// Get the next APS counter value (wrapping).
    pub fn next_aps_counter(&mut self) -> u8 {
        let c = self.aps_counter;
        self.aps_counter = self.aps_counter.wrapping_add(1);
        c
    }

    pub fn security_handshake_stats(&self) -> ApsSecurityHandshakeStats {
        self.security_handshake_stats
    }

    /// Take the link-key regime recorded by the initial Network-Key transport.
    pub fn take_network_key_join_method(&mut self) -> Option<NetworkKeyJoinMethod> {
        self.network_key_join_method.take()
    }

    /// Take an authenticated Remove-Device command addressed to this node.
    pub fn take_local_remove_device(&mut self) -> bool {
        core::mem::take(&mut self.pending_local_remove_device)
    }

    /// Enable or disable application-link-key installation by Transport-Key.
    ///
    /// It is disabled by default. A composition root enables it only when it
    /// owns durable APS key-table storage.
    pub fn set_application_link_key_installation_enabled(&mut self, enabled: bool) {
        self.application_link_key_installation_enabled = enabled;
    }

    pub fn set_network_key_persistence_enabled(&mut self, enabled: bool) {
        self.network_key_persistence_enabled = enabled;
        if !enabled {
            self.pending_network_key_persistence = false;
            self.pending_network_key_replay = None;
        }
    }

    pub const fn network_key_persistence_pending(&self) -> bool {
        self.pending_network_key_persistence
    }

    pub const fn pending_network_key_replay(&self) -> Option<security::ApsReplayCounter> {
        self.pending_network_key_replay
    }

    /// Inspect a received forwarding change without authorizing transmission.
    #[cfg(feature = "router")]
    pub const fn network_key_forwarding_intent(&self) -> Option<NetworkKeyForwardingIntent> {
        self.network_key_forwarding_intent
    }

    /// Request TC-local fanout of its staged zero-destination update.
    ///
    /// The runtime must checkpoint this intent with the key before sending.
    #[cfg(feature = "router")]
    pub fn request_network_key_forwarding(&mut self, sequence: u8) -> Result<(), ApsStatus> {
        if self.nwk.device_type() != zigbee_nwk::DeviceType::Coordinator
            || self.aib.aps_trust_center_address != self.nwk.nib().ieee_address
            || self
                .nwk
                .security()
                .staged_key()
                .is_none_or(|key| key.seq_number != sequence)
        {
            return Err(ApsStatus::InvalidParameter);
        }
        self.network_key_forwarding_intent = Some(NetworkKeyForwardingIntent::Forward(sequence));
        Ok(())
    }

    /// Complete only after the key and forwarding intent are durably stored.
    #[cfg(feature = "router")]
    pub fn complete_network_key_forwarding_checkpoint(&mut self) {
        self.network_key_forwarding_intent = None;
    }

    pub fn complete_network_key_persistence(&mut self) {
        if let Some(replay) = self.pending_network_key_replay.take() {
            self.security.commit_replay_counter(replay);
        }
        self.pending_network_key_persistence = false;
        self.network_key_persistence_enabled = false;
    }

    pub fn abort_network_key_persistence(&mut self) {
        self.pending_network_key_persistence = false;
        self.pending_network_key_replay = None;
        self.network_key_persistence_enabled = false;
    }

    pub const fn application_key_persistence_pending(&self) -> bool {
        self.pending_application_key_persistence
    }

    pub const fn pending_application_key_replay(&self) -> Option<security::ApsReplayCounter> {
        self.pending_application_key_replay
    }

    pub fn complete_application_key_persistence(&mut self) {
        if let Some(replay) = self.pending_application_key_replay.take() {
            self.security.commit_replay_counter(replay);
        }
        self.pending_application_key_persistence = false;
    }

    pub const fn pending_data_replay(&self) -> Option<security::ApsReplayCounter> {
        self.pending_data_replay
    }

    /// New durable replay domains reserved by the held APS replay floor.
    ///
    /// The reservation is owned by the pending slot: completing, aborting or
    /// disabling it releases the reservation with it.
    pub const fn pending_replay_reservations(&self) -> usize {
        #[cfg(feature = "router")]
        let security_indication = self.pending_security_indication_replay.is_some();
        #[cfg(not(feature = "router"))]
        let security_indication = false;
        let held = self.pending_data_replay.is_some()
            || self.pending_application_key_replay.is_some()
            || self.pending_network_key_replay.is_some()
            || security_indication;
        (held && self.pending_replay_new_domain) as usize
    }

    /// Apply the live floor only after the caller has committed the data-frame
    /// side effect and this replay counter. Does not release the pending ACK.
    pub fn complete_data_persistence(&mut self) {
        if let Some(replay) = self.pending_data_replay.take() {
            self.security.commit_replay_counter(replay);
        }
        self.pending_rx_rollback = None;
    }

    /// Abandon the current data frame's durable commit.
    ///
    /// The frame was not durably accepted, so everything reception recorded
    /// for it is undone as well: its duplicate-rejection entry (or its block in
    /// a reassembly session) is removed together with the deferred replay
    /// floor. Otherwise the sender's retransmission — the only way the frame
    /// can still be delivered — would be acknowledged and discarded as a
    /// duplicate.
    pub fn abort_data_persistence(&mut self) {
        self.pending_data_replay = None;
        match self.pending_rx_rollback.take() {
            Some(ApsRxRollback::Duplicate {
                src_addr,
                aps_counter,
            }) => {
                for entry in self.dup_table.iter_mut() {
                    if entry.active
                        && entry.src_addr == src_addr
                        && entry.aps_counter == aps_counter
                    {
                        entry.active = false;
                    }
                }
            }
            #[cfg(feature = "fragmentation")]
            Some(ApsRxRollback::FragmentBlock {
                src_addr,
                aps_counter,
                block,
            }) => {
                for entry in self.dup_table.iter_mut() {
                    if entry.active
                        && entry.fragment_window.is_some()
                        && entry.src_addr == src_addr
                        && entry.aps_counter == aps_counter
                    {
                        entry.active = false;
                    }
                }
                self.fragment_rx
                    .rollback_block(src_addr, aps_counter, block);
            }
            None => {}
        }
    }

    /// `apsDuplicateRejectionTimeout` in microseconds of the monotonic clock.
    fn duplicate_rejection_timeout_us(&self) -> u32 {
        u32::from(self.aib.aps_duplicate_rejection_timeout).saturating_mul(1000)
    }

    fn dup_entry_live(entry: &ApsDuplicateEntry, now: u32, timeout_us: u32) -> bool {
        entry.active && now.wrapping_sub(entry.accepted_at_us) < timeout_us
    }

    /// Look up a live duplicate-rejection entry.
    ///
    /// `fragmented` selects the transaction kind: an unfragmented frame and a
    /// fragmented transaction never match each other's entries.
    fn find_duplicate(&self, src_addr: u16, aps_counter: u8, fragmented: bool) -> Option<usize> {
        let now = self.nwk.mac().monotonic_micros();
        let timeout = self.duplicate_rejection_timeout_us();
        self.dup_table.iter().position(|entry| {
            Self::dup_entry_live(entry, now, timeout)
                && entry.src_addr == src_addr
                && entry.aps_counter == aps_counter
                && entry.fragment_window.is_some() == fragmented
        })
    }

    /// Record an accepted transaction in the duplicate-rejection table,
    /// reusing an expired slot first and otherwise the oldest one.
    fn record_duplicate(&mut self, src_addr: u16, aps_counter: u8, fragment_window: Option<u8>) {
        let now = self.nwk.mac().monotonic_micros();
        let timeout = self.duplicate_rejection_timeout_us();
        let mut best: Option<(usize, u32)> = None;
        for (i, entry) in self.dup_table.iter().enumerate() {
            if !Self::dup_entry_live(entry, now, timeout) {
                best = Some((i, u32::MAX));
                break;
            }
            let waited = now.wrapping_sub(entry.accepted_at_us);
            if best.is_none_or(|(_, oldest)| waited >= oldest) {
                best = Some((i, waited));
            }
        }
        if let Some((idx, _)) = best {
            self.dup_table[idx] = ApsDuplicateEntry {
                src_addr,
                aps_counter,
                fragment_window,
                accepted_at_us: now,
                active: true,
            };
        }
    }

    /// Check if an unfragmented APS frame is a duplicate. Returns true if
    /// duplicate. If not a duplicate, records it in the table.
    ///
    /// An entry stays live for `apsDuplicateRejectionTimeout` milliseconds of
    /// the platform monotonic clock (R22 §2.2.8.4.2).
    pub fn is_aps_duplicate(&mut self, src_addr: u16, aps_counter: u8) -> bool {
        if self.find_duplicate(src_addr, aps_counter, false).is_some() {
            return true;
        }
        self.record_duplicate(src_addr, aps_counter, None);
        false
    }

    /// Expire duplicate-rejection entries — and stale fragment reassembly
    /// sessions — by elapsed time.
    ///
    /// Timing is read from the platform monotonic clock
    /// ([`zigbee_mac::PlatformServices::monotonic_micros`], the same base as
    /// the ACK table), so the call frequency no longer matters: entries expire
    /// after `apsDuplicateRejectionTimeout` milliseconds whether this runs every
    /// 10 ms or every few seconds. Lookups apply the same deadline, so calling
    /// this only reclaims slots early.
    pub fn age_dup_table(&mut self) {
        let now = self.nwk.mac().monotonic_micros();
        let timeout = self.duplicate_rejection_timeout_us();
        for entry in self.dup_table.iter_mut() {
            if entry.active && !Self::dup_entry_live(entry, now, timeout) {
                entry.active = false;
            }
        }
        #[cfg(feature = "fragmentation")]
        self.fragment_rx.expire(now);
    }

    /// Register an outbound frame for ACK tracking.
    ///
    /// The acknowledgement wait starts now: the entry records the monotonic
    /// timestamp of this transmission, and [`Self::age_ack_table`] refuses to
    /// retransmit until a full [`APS_ACK_WAIT_DURATION_US`] has elapsed since
    /// it.
    ///
    /// Returns the slot index. When every slot is occupied by a transmission
    /// still inside its acknowledgement window, the longest-waiting entry is
    /// reused (and logged) so the newest transmission is always the one that
    /// keeps its retries.
    pub fn register_ack_pending(
        &mut self,
        aps_counter: u8,
        dst_addr: u16,
        frame_bytes: &[u8],
    ) -> Option<usize> {
        self.register_ack_pending_inner(aps_counter, dst_addr, frame_bytes, None)
    }

    /// Register an APS-secured outbound frame for ACK tracking.
    ///
    /// `plaintext_frame` is the APS header followed by the *unencrypted*
    /// payload; `security` records the key-pair entry, nonce source and
    /// security-control byte the frame went out with, so
    /// [`Self::age_ack_table`] can encrypt each retry again under a fresh
    /// frame counter instead of replaying the original ciphertext.
    pub(crate) fn register_secured_ack_pending(
        &mut self,
        aps_counter: u8,
        dst_addr: u16,
        plaintext_frame: &[u8],
        security: PendingApsSecurity,
    ) -> Option<usize> {
        self.register_ack_pending_inner(aps_counter, dst_addr, plaintext_frame, Some(security))
    }

    fn register_ack_pending_inner(
        &mut self,
        aps_counter: u8,
        dst_addr: u16,
        frame_bytes: &[u8],
        security: Option<PendingApsSecurity>,
    ) -> Option<usize> {
        let generation = self.next_ack_generation;
        self.next_ack_generation = self.next_ack_generation.wrapping_add(1);
        if self.next_ack_generation == 0 {
            self.next_ack_generation = 1;
        }
        let now = self.nwk.mac().monotonic_micros();
        // Try to find an inactive slot to reuse
        for (i, entry) in self.ack_table.iter_mut().enumerate() {
            if !entry.active {
                *entry = PendingApsAckEntry {
                    active: true,
                    generation,
                    aps_counter,
                    dst_addr,
                    confirmed: false,
                    retries: 3,
                    waiting_since_us: now,
                    original_frame: heapless::Vec::new(),
                    security,
                };
                let _ = entry.original_frame.extend_from_slice(frame_bytes);
                return Some(i);
            }
        }
        // No inactive slot — try to push a new entry
        let idx = self.ack_table.len();
        let mut new_entry = PendingApsAckEntry {
            active: true,
            generation,
            aps_counter,
            dst_addr,
            confirmed: false,
            retries: 3,
            waiting_since_us: now,
            original_frame: heapless::Vec::new(),
            security,
        };
        let _ = new_entry.original_frame.extend_from_slice(frame_bytes);
        if self.ack_table.push(new_entry).is_ok() {
            return Some(idx);
        }

        // Every slot is occupied by a transmission still inside its
        // acknowledgement window. Entries now live for up to four
        // `apscAckWaitDuration` windows instead of a handful of maintenance
        // calls, so a burst — a ZDO interview answers several unicast requests
        // in a row, and each one asks for an acknowledgement — can legitimately
        // fill a 4-slot sensor table. Reuse the entry that has been waiting
        // longest: it is the closest to timing out anyway, and dropping
        // tracking for the *newest* frame would silently disable retries for
        // exactly the traffic in flight.
        let mut victim: Option<(usize, u32)> = None;
        for (i, entry) in self.ack_table.iter().enumerate() {
            let waited = now.wrapping_sub(entry.waiting_since_us);
            if victim.is_none_or(|(_, longest)| waited > longest) {
                victim = Some((i, waited));
            }
        }
        let Some((idx, waited)) = victim else {
            log::warn!("[APS] ACK tracking table full, cannot track counter={aps_counter}");
            return None;
        };
        log::warn!(
            "[APS] ACK table full — evicting counter={} (waited {} us) for counter={}",
            self.ack_table[idx].aps_counter,
            waited,
            aps_counter,
        );
        let entry = &mut self.ack_table[idx];
        *entry = PendingApsAckEntry {
            active: true,
            generation,
            aps_counter,
            dst_addr,
            confirmed: false,
            retries: 3,
            waiting_since_us: now,
            original_frame: heapless::Vec::new(),
            security,
        };
        let _ = entry.original_frame.extend_from_slice(frame_bytes);
        Some(idx)
    }

    #[cfg(all(test, feature = "router"))]
    pub(crate) fn ack_handle(&self, dst_addr: u16, aps_counter: u8) -> Option<ApsAckHandle> {
        self.ack_table.iter().find_map(|entry| {
            (entry.active && entry.aps_counter == aps_counter && entry.dst_addr == dst_addr)
                .then_some(ApsAckHandle::new(dst_addr, aps_counter, entry.generation))
        })
    }

    /// Whether an outbound unicast still needs its peer's acknowledgement.
    pub fn has_pending_ack(&self) -> bool {
        #[cfg(feature = "fragmentation")]
        if self.fragment_tx.is_some() {
            return true;
        }
        self.ack_table
            .iter()
            .any(|entry| entry.active && !entry.confirmed)
    }

    /// Final APSDE-DATA.confirm of a fragmented transaction (R22
    /// §2.2.8.4.5.1): `SUCCESS` once every block was acknowledged, `NO_ACK`
    /// after `apscMaxFrameRetries` unanswered retries, `SECURITY_FAIL` if a
    /// block could not be secured. `None` while the transaction is in flight
    /// or when none was started. Taking the result clears it.
    pub fn take_fragmented_tx_confirm(&mut self) -> Option<apsde::ApsdeDataConfirm> {
        self.fragment_tx_confirm.take()
    }

    /// Deliver an incoming APS ACK. Returns true if matched a pending request.
    ///
    /// This entry point carries no security information, so it only
    /// completes transmissions that were not APS-secured; the receive path
    /// uses [`Self::confirm_ack_with_security`].
    pub fn confirm_ack(&mut self, src_addr: u16, aps_counter: u8) -> bool {
        self.confirm_ack_with_security(src_addr, aps_counter, false)
    }

    /// Deliver an incoming APS ACK whose APS security state is known.
    ///
    /// An APS-secured transmission is only completed by an APS-secured ACK:
    /// a device holding just the network key could otherwise forge the
    /// acknowledgement and suppress the retries of a link-key-protected
    /// frame.
    pub(crate) fn confirm_ack_with_security(
        &mut self,
        src_addr: u16,
        aps_counter: u8,
        aps_secured_ack: bool,
    ) -> bool {
        for entry in self.ack_table.iter_mut() {
            if entry.active
                && entry.aps_counter == aps_counter
                && entry.dst_addr == src_addr
                && !entry.confirmed
            {
                if entry.security.is_some() && !aps_secured_ack {
                    log::warn!(
                        "[APS] ignoring unsecured ACK for APS-secured counter={} from 0x{:04X}",
                        aps_counter,
                        src_addr,
                    );
                    return false;
                }
                entry.confirmed = true;
                let handle = ApsAckHandle::new(src_addr, aps_counter, entry.generation);
                if !self.ack_completions.contains(&handle) {
                    if self.ack_completions.is_full() {
                        self.ack_completions.remove(0);
                    }
                    self.ack_completions
                        .push(handle)
                        .expect("completion capacity checked above");
                }
                log::debug!(
                    "[APS] ACK confirmed counter={} from 0x{:04X}",
                    aps_counter,
                    src_addr,
                );
                return true;
            }
        }
        false
    }

    /// Observe an acknowledged unicast without consuming its delivery result.
    pub fn ack_status(&self, handle: ApsAckHandle) -> Option<ApsAckStatus> {
        if self.ack_completions.contains(&handle) {
            return Some(ApsAckStatus::Confirmed);
        }
        self.ack_table.iter().find_map(|entry| {
            (entry.active
                && entry.generation == handle.generation
                && entry.aps_counter == handle.aps_counter
                && entry.dst_addr == handle.dst_addr)
                .then_some(if entry.confirmed {
                    ApsAckStatus::Confirmed
                } else {
                    ApsAckStatus::Pending
                })
        })
    }

    /// Release ACK tracking after an upper-layer durable transaction records
    /// the delivery outcome.
    pub fn clear_ack_status(&mut self, handle: ApsAckHandle) {
        if let Some(index) = self
            .ack_completions
            .iter()
            .position(|completed| *completed == handle)
        {
            self.ack_completions.swap_remove(index);
        }
        if let Some(entry) = self.ack_table.iter_mut().find(|entry| {
            entry.active
                && entry.generation == handle.generation
                && entry.aps_counter == handle.aps_counter
                && entry.dst_addr == handle.dst_addr
        }) {
            entry.active = false;
            entry.original_frame.clear();
        }
    }

    /// Cancel every outbound ACK transaction and invalidate all outstanding
    /// handles. The generation counter deliberately continues across resets,
    /// so a later reuse of the same APS counter cannot match an old handle.
    pub fn cancel_all_ack_tracking(&mut self) {
        for entry in &mut self.ack_table {
            entry.active = false;
            entry.original_frame.clear();
        }
        self.ack_completions.clear();
    }

    /// Check if a specific APS counter has been ACK'd. Clears the slot if confirmed.
    pub fn take_ack_status(&mut self, aps_counter: u8) -> Option<bool> {
        for entry in self.ack_table.iter_mut() {
            if entry.active && entry.aps_counter == aps_counter {
                let confirmed = entry.confirmed;
                entry.active = false;
                return Some(confirmed);
            }
        }
        None
    }

    /// Age the ACK table. Returns frames that need retransmission.
    ///
    /// Retransmission is driven by *elapsed time*, not by call frequency: an
    /// unconfirmed entry is left alone until a full
    /// [`APS_ACK_WAIT_DURATION_US`] has passed since the transmission it is
    /// waiting on (R22 §2.2.5.2.2, `apscAckWaitDuration`). Only then does it
    /// consume one retry and return the original frame bytes together with the
    /// short address the frame was originally addressed to; the wait then
    /// restarts, so every successive retry gets its own full window. When the
    /// retries are exhausted, the entry is deactivated one further window
    /// later.
    ///
    /// This makes the behaviour identical whether the caller runs maintenance
    /// once a second or every few milliseconds. Without it, each maintenance
    /// call burned one retry, so a unicast that requested an acknowledgement
    /// put four copies of the same frame on the air back to back and then gave
    /// up long before any acknowledgement could arrive.
    ///
    /// The destination travels with the frame because an APS retransmission is
    /// a *unicast* repeat of the original transmission (R22 §2.2.5.2.2):
    /// re-sending it to a broadcast address would flood the network with a
    /// frame only one device is expecting, and the acknowledgement it is
    /// waiting for would still never arrive.
    ///
    /// An APS-secured frame is *re-secured*, never replayed: the retry is
    /// encrypted again under the next frame counter of the key-pair entry that
    /// secured the original (R22 §4.4.1.1 forbids reusing a counter, and a
    /// receiver enforcing replay protection drops a repeated one). The APS
    /// header — including the APS counter the acknowledgement is keyed on — is
    /// unchanged, so the transaction semantics are identical. If that key-pair
    /// entry is gone, or its durable counter reservation is exhausted, the
    /// retry is dropped and the entry released rather than sent with stale
    /// security.
    pub fn age_ack_table(&mut self) -> heapless::Vec<ApsRetransmission, APS_ACK_TABLE_SIZE> {
        let now = self.nwk.mac().monotonic_micros();
        let mut due = heapless::Vec::<usize, APS_ACK_TABLE_SIZE>::new();
        for (index, entry) in self.ack_table.iter_mut().enumerate() {
            if !entry.active {
                continue;
            }
            if entry.confirmed {
                // The transmission is complete. Free the slot here: the table
                // is small (4 entries on a sensor build), and leaving confirmed
                // entries in place would permanently exhaust it after the first
                // handful of acknowledged unicasts, silently dropping ACK
                // tracking for everything sent afterwards.
                entry.active = false;
                entry.original_frame.clear();
                continue;
            }
            // Wrapping subtraction: the monotonic microsecond clock rolls over
            // every ~71.6 minutes, and only the difference matters.
            if now.wrapping_sub(entry.waiting_since_us) < APS_ACK_WAIT_DURATION_US {
                continue;
            }
            if entry.retries == 0 {
                log::warn!(
                    "[APS] ACK timeout counter={} dst=0x{:04X}",
                    entry.aps_counter,
                    entry.dst_addr,
                );
                entry.active = false;
                entry.original_frame.clear();
            } else {
                entry.retries = entry.retries.saturating_sub(1);
                entry.waiting_since_us = now;
                if !entry.original_frame.is_empty() {
                    log::debug!(
                        "[APS] Retransmit counter={} dst=0x{:04X} retries_left={}",
                        entry.aps_counter,
                        entry.dst_addr,
                        entry.retries,
                    );
                    due.push(index)
                        .expect("one due index per ACK table entry at most");
                }
            }
        }

        // Second pass: build the frames. Re-securing needs `&mut self`
        // (a fresh outgoing frame counter and the MAC's AES backend), so it
        // cannot run while the ACK table is borrowed above.
        let mut retransmit = heapless::Vec::<ApsRetransmission, APS_ACK_TABLE_SIZE>::new();
        for index in due {
            let dst_addr = zigbee_types::ShortAddress(self.ack_table[index].dst_addr);
            let Some(security) = self.ack_table[index].security else {
                let frame = self.ack_table[index].original_frame.clone();
                retransmit
                    .push(ApsRetransmission { dst_addr, frame })
                    .expect("retransmission capacity matches the ACK table");
                continue;
            };
            let plaintext = self.ack_table[index].original_frame.clone();
            match self.resecure_retransmission(&security, &plaintext) {
                Some(frame) => {
                    retransmit
                        .push(ApsRetransmission { dst_addr, frame })
                        .expect("retransmission capacity matches the ACK table");
                }
                None => {
                    log::error!(
                        "[APS] Cannot re-secure retransmission counter={} dst=0x{:04X}; dropping",
                        self.ack_table[index].aps_counter,
                        self.ack_table[index].dst_addr,
                    );
                    let entry = &mut self.ack_table[index];
                    entry.active = false;
                    entry.original_frame.clear();
                }
            }
        }

        // Fragmented transaction: run its acknowledgement timer and hand out
        // due blocks in the remaining capacity, so a maintenance loop that
        // only sends what this returns also drives fragmentation.
        #[cfg(feature = "fragmentation")]
        {
            self.poll_fragment_tx_timeout();
            while !retransmit.is_full() {
                let Some((dst_addr, _, _, frame)) = self.next_due_fragment() else {
                    break;
                };
                retransmit
                    .push(ApsRetransmission { dst_addr, frame })
                    .expect("capacity checked above");
            }
        }
        retransmit
    }

    /// Local endpoints that are members of `group_address`
    /// (R22 §2.2.4.1.2), in group-table order; empty when none.
    ///
    /// A group-addressed [`apsde::ApsdeDataIndication`] reports destination
    /// endpoint `0xFF`; the receiver must hand the ASDU to each endpoint
    /// returned here (and only those).
    pub fn group_member_endpoints(&self, group_address: u16) -> &[u8] {
        self.group_table
            .find(group_address)
            .map(|group| group.endpoint_list.as_slice())
            .unwrap_or(&[])
    }

    /// Reference to the underlying NWK layer.
    pub fn nwk(&self) -> &NwkLayer<M> {
        &self.nwk
    }

    /// Mutable reference to the underlying NWK layer.
    pub fn nwk_mut(&mut self) -> &mut NwkLayer<M> {
        &mut self.nwk
    }

    /// Reference to the APS Information Base.
    pub fn aib(&self) -> &aib::Aib {
        &self.aib
    }

    /// Mutable reference to the APS Information Base.
    pub fn aib_mut(&mut self) -> &mut aib::Aib {
        &mut self.aib
    }

    /// Reference to the binding table.
    pub fn binding_table(&self) -> &binding::BindingTable {
        &self.binding_table
    }

    /// Mutable reference to the binding table.
    pub fn binding_table_mut(&mut self) -> &mut binding::BindingTable {
        &mut self.binding_table
    }

    /// Reference to the group table.
    pub fn group_table(&self) -> &group::GroupTable {
        &self.group_table
    }

    /// Mutable reference to the group table.
    pub fn group_table_mut(&mut self) -> &mut group::GroupTable {
        &mut self.group_table
    }

    /// Reference to APS security state.
    pub fn security(&self) -> &security::ApsSecurity {
        &self.security
    }

    /// Mutable reference to APS security state.
    pub fn security_mut(&mut self) -> &mut security::ApsSecurity {
        &mut self.security
    }

    /// Reference to the fragment reassembly buffer.
    pub fn fragment_rx(&self) -> &fragment::FragmentReassembly {
        &self.fragment_rx
    }

    /// Mutable reference to the fragment reassembly buffer.
    pub fn fragment_rx_mut(&mut self) -> &mut fragment::FragmentReassembly {
        &mut self.fragment_rx
    }
}
