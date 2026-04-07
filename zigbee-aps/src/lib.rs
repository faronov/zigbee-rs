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
    /// Original serialized frame bytes for retransmission
    original_frame: heapless::Vec<u8, 128>,
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
    /// Pending APS ACK to send after processing incoming frame
    pending_aps_ack: Option<PendingApsAck>,
    /// APS duplicate rejection table
    dup_table: [ApsDuplicateEntry; APS_DUP_TABLE_SIZE],
    /// Outbound APS ACK tracking (frames awaiting ACK confirmation)
    ack_table: heapless::Vec<PendingApsAckEntry, APS_ACK_TABLE_SIZE>,
    /// Fragment reassembly buffer for incoming fragmented frames
    fragment_rx: fragment::FragmentReassembly,
}

impl<M: MacDriver> ApsLayer<M> {
    /// Create a new APS layer wrapping the given NWK layer.
    pub fn new(nwk: NwkLayer<M>) -> Self {
        Self {
            nwk,
            aib: aib::Aib::new(),
            binding_table: binding::BindingTable::new(),
            group_table: group::GroupTable::new(),
            security: security::ApsSecurity::new(),
            aps_counter: 0,
            pending_aps_ack: None,
            dup_table: [ApsDuplicateEntry::empty(); APS_DUP_TABLE_SIZE],
            ack_table: heapless::Vec::new(),
            fragment_rx: fragment::FragmentReassembly::new(),
        }
    }

    /// Get the next APS counter value (wrapping).
    pub fn next_aps_counter(&mut self) -> u8 {
        let c = self.aps_counter;
        self.aps_counter = self.aps_counter.wrapping_add(1);
        c
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
    /// Returns the slot index, or None if the table is full.
    pub fn register_ack_pending(
        &mut self,
        aps_counter: u8,
        dst_addr: u16,
        frame_bytes: &[u8],
    ) -> Option<usize> {
        // Try to find an inactive slot to reuse
        for (i, entry) in self.ack_table.iter_mut().enumerate() {
            if !entry.active {
                *entry = PendingApsAckEntry {
                    active: true,
                    aps_counter,
                    dst_addr,
                    confirmed: false,
                    retries: 3,
                    original_frame: heapless::Vec::new(),
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
            original_frame: heapless::Vec::new(),
        };
        let _ = new_entry.original_frame.extend_from_slice(frame_bytes);
        if self.ack_table.push(new_entry).is_ok() {
            return Some(idx);
        }
        log::warn!("[APS] ACK tracking table full, cannot track counter={aps_counter}");
        None
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
    /// When an unconfirmed entry still has retries, it decrements the retry
    /// count and returns the original frame bytes for retransmission.
    /// When retries are exhausted, the entry is deactivated.
    pub fn age_ack_table(&mut self) -> heapless::Vec<heapless::Vec<u8, 128>, 4> {
        let mut retransmit = heapless::Vec::<heapless::Vec<u8, 128>, 4>::new();
        for entry in self.ack_table.iter_mut() {
            if entry.active && !entry.confirmed {
                if entry.retries == 0 {
                    log::warn!(
                        "[APS] ACK timeout counter={} dst=0x{:04X}",
                        entry.aps_counter,
                        entry.dst_addr,
                    );
                    entry.active = false;
                } else {
                    entry.retries = entry.retries.saturating_sub(1);
                    if !entry.original_frame.is_empty() {
                        log::debug!(
                            "[APS] Retransmit counter={} dst=0x{:04X} retries_left={}",
                            entry.aps_counter,
                            entry.dst_addr,
                            entry.retries,
                        );
                        let _ = retransmit.push(entry.original_frame.clone());
                    }
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
