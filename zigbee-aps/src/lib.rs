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
pub mod frames;
pub mod group;
pub mod security;

use zigbee_mac::MacDriver;
use zigbee_nwk::NwkLayer;

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
        }
    }
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

/// APS Tunnel command awaiting hop-by-hop delivery to a joining child.
#[derive(Debug, Clone)]
pub struct PendingApsTunnel {
    pub destination: zigbee_types::IeeeAddress,
    frame: heapless::Vec<u8, 128>,
}

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

/// APS duplicate rejection entry
#[derive(Debug, Clone, Copy)]
struct ApsDuplicateEntry {
    src_addr: u16,
    aps_counter: u8,
    age: u16,
    active: bool,
}

impl ApsDuplicateEntry {
    const fn empty() -> Self {
        Self {
            src_addr: 0,
            aps_counter: 0,
            age: 0,
            active: false,
        }
    }
}

/// Tracks an outbound APS frame that requested an ACK.
#[derive(Debug, Clone)]
struct PendingApsAckEntry {
    /// Whether this slot is in use
    active: bool,
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

/// Everything needed to re-secure an APS retransmission.
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
    /// Security control byte of the original auxiliary header.
    security_control: u8,
    /// Length of the APS header prefix inside `original_frame`.
    header_len: u8,
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
    pending_tunnel: Option<PendingApsTunnel>,
    /// APS duplicate rejection table
    dup_table: [ApsDuplicateEntry; APS_DUP_TABLE_SIZE],
    /// Outbound APS ACK tracking (frames awaiting ACK confirmation)
    ack_table: heapless::Vec<PendingApsAckEntry, APS_ACK_TABLE_SIZE>,
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
            pending_tunnel: None,
            dup_table: [ApsDuplicateEntry::empty(); APS_DUP_TABLE_SIZE],
            ack_table: heapless::Vec::new(),
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
            core::ptr::addr_of_mut!((*slot).pending_tunnel).write(None);
            core::ptr::addr_of_mut!((*slot).dup_table)
                .write([ApsDuplicateEntry::empty(); APS_DUP_TABLE_SIZE]);
            core::ptr::addr_of_mut!((*slot).ack_table).write(heapless::Vec::new());
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

    /// Check if an APS frame is a duplicate. Returns true if duplicate.
    /// If not a duplicate, records it in the table.
    pub fn is_aps_duplicate(&mut self, src_addr: u16, aps_counter: u8) -> bool {
        // Check existing entries
        for entry in self.dup_table.iter() {
            if entry.active && entry.src_addr == src_addr && entry.aps_counter == aps_counter {
                return true; // Duplicate
            }
        }
        // Not a duplicate — record it
        // Find inactive slot first, else evict oldest
        let mut best_idx: Option<usize> = None;
        let mut best_age: u16 = 0;
        for (i, entry) in self.dup_table.iter().enumerate() {
            if !entry.active {
                best_idx = Some(i);
                break;
            }
            if entry.age >= best_age {
                best_age = entry.age;
                best_idx = Some(i);
            }
        }
        if let Some(idx) = best_idx {
            self.dup_table[idx] = ApsDuplicateEntry {
                src_addr,
                aps_counter,
                age: 0,
                active: true,
            };
        }
        false
    }

    /// Age the APS duplicate rejection table. Call periodically (e.g. every second).
    pub fn age_dup_table(&mut self) {
        let timeout = self.aib.aps_duplicate_rejection_timeout;
        for entry in self.dup_table.iter_mut() {
            if entry.active {
                entry.age = entry.age.saturating_add(1);
                if entry.age >= timeout {
                    entry.active = false;
                }
            }
        }
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
        let now = self.nwk.mac().monotonic_micros();
        // Try to find an inactive slot to reuse
        for (i, entry) in self.ack_table.iter_mut().enumerate() {
            if !entry.active {
                *entry = PendingApsAckEntry {
                    active: true,
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

    /// Deliver an incoming APS ACK. Returns true if matched a pending request.
    pub fn confirm_ack(&mut self, src_addr: u16, aps_counter: u8) -> bool {
        for entry in self.ack_table.iter_mut() {
            if entry.active
                && entry.aps_counter == aps_counter
                && entry.dst_addr == src_addr
                && !entry.confirmed
            {
                entry.confirmed = true;
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
        retransmit
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
