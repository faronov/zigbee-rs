//! Link layer (spec 5): sessions, selective-repeat ARQ, credits, keepalive.
//!
//! [`Link`] is a sans-IO, poll-style state machine. It never reads a clock,
//! sleeps or allocates; the caller supplies a monotonic time in microseconds
//! ([`Instant`]) to every call and moves bytes between the link and the
//! transport:
//!
//! ```text
//!   transport RX bytes ──> Link::push_rx ──> Link::poll_event ──> app (Data, session events)
//!   app queues (TxSource) <── Link::poll_transmit ──> transport TX bytes
//!   sleep until Link::next_deadline()
//! ```
//!
//! The same code runs on both sides; [`LinkConfig::role`] selects host or
//! NCP behaviour (cookie roles in the epoch and `ResetBody.role`). Memory is
//! fixed by two const generics: `TXW` transmit slots (frames stored for
//! byte-identical retransmission) and `RXW` receive slots (reorder buffer,
//! ≥ the advertised `rx_window`). Each slot holds one 640-byte payload.
//!
//! | Profile | `TXW` | `RXW` | `size_of::<Link>()` (x86-64) |
//! |---|---:|---:|---:|
//! | ESP32-H2 NCP | 16 | 16 | 22 440 B |
//! | ESP32-S3 embedded host | 8 (or 4) | 8 (or 4) | 11 848 B (6 552 B) |
//! | Desktop host | 32 | 32 | 43 624 B |
//!
//! (`tests/size.rs` keeps these within `slots × (640 + 48) + 656 + 1024`.)
//!
//! ```
//! use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent};
//! use zigbee_zrhp::frame::{ResetReason, TX_BUFFER_LEN};
//! use zigbee_zrhp::queue::NoSource;
//!
//! let mut host = Link::<8, 8>::new(LinkConfig::embedded_host(), 0x1A2B_3C4D);
//! let mut ncp = Link::<16, 16>::new(LinkConfig::ncp(), 0xC0FF_EE01);
//! host.open(0, ResetReason::HostOpen);
//! ncp.open(0, ResetReason::PowerOn);
//! let mut buf = [0u8; TX_BUFFER_LEN];
//! for _ in 0..4 {
//!     while let Some(n) = host.poll_transmit(0, &mut NoSource, &mut buf) {
//!         let mut off = 0;
//!         while off < n { off += ncp.push_rx(&buf[off..n], 0); while ncp.poll_event().is_some() {} }
//!     }
//!     while let Some(n) = ncp.poll_transmit(0, &mut NoSource, &mut buf) {
//!         let mut off = 0;
//!         while off < n { off += host.push_rx(&buf[off..n], 0); while host.poll_event().is_some() {} }
//!     }
//! }
//! assert_eq!(host.session().unwrap().epoch, 33);
//! assert_eq!(ncp.session().unwrap().epoch, 33);
//! ```

mod engine;

pub use engine::Link;

use crate::frame::{CRC_LEN, HEADER_LEN, Role, TX_BUFFER_LEN};
use crate::types::Endpoint;

/// Monotonic time in microseconds, supplied by the caller.
pub type Instant = u64;

/// Bits on the line per byte (UART 8N1: start + 8 data + stop).
pub const BITS_PER_BYTE: u64 = 10;

/// ACK-first rule (spec 5.6): a new or retransmitted DATA frame is only
/// produced while fewer than this many bytes are waiting ahead on our
/// transmit side (modelled from the line rate, or reported by the
/// transport). ACK, NAK, REJECT and RESET frames are never held back, so a
/// control frame waits behind fewer than [`PEER_ACK_AHEAD`] bytes of DATA.
pub const ACK_FIRST_BACKLOG: usize = 256;

/// Bound on what an ACK-first peer can have queued ahead of the ACK it
/// owes us: less than [`ACK_FIRST_BACKLOG`] plus one maximum wire frame.
pub const PEER_ACK_AHEAD: usize = ACK_FIRST_BACKLOG + TX_BUFFER_LEN;

/// Longest ACK/NAK on the wire: header, `sack`, `nbad`, five credit
/// entries and CRC, COBS-encoded, with a leading and a trailing `0x00`.
pub const ACK_WIRE_MAX: usize = crate::cobs::max_encoded_len(HEADER_LEN + 6 + 2 * 5 + CRC_LEN) + 2;

/// Time (µs, rounded up) to serialise `bytes` at `line_rate_bps` (8N1);
/// 0 when the line rate is unknown (0).
pub const fn wire_time_us(bytes: usize, line_rate_bps: u32) -> u64 {
    if line_rate_bps == 0 {
        return 0;
    }
    (bytes as u64 * BITS_PER_BYTE * 1_000_000).div_ceil(line_rate_bps as u64)
}

/// Lower bound of the RTO (µs), measured from the moment a frame has left
/// our wire (spec 5.6 S3): the peer may first have to send up to
/// [`PEER_ACK_AHEAD`] bytes it had queued, then the ACK itself
/// ([`ACK_WIRE_MAX`]); it may hold the ACK for its `ack_delay`; and
/// `rto_min_us` stays as the margin for scheduling and USB latency. With an
/// unknown line rate this is `rto_min_us`, the fixed floor.
///
/// 115 200 baud, `ack_delay` 1 ms, `RTO_min` 25 ms:
/// `T(913 + 31 B) + 1 ms + 25 ms = 81.9 + 1 + 25 ≈ 108 ms`.
pub const fn rto_floor_us(line_rate_bps: u32, peer_ack_delay_us: u32, rto_min_us: u32) -> u64 {
    if line_rate_bps == 0 {
        return rto_min_us as u64;
    }
    wire_time_us(PEER_ACK_AHEAD + ACK_WIRE_MAX, line_rate_bps)
        + peer_ack_delay_us as u64
        + rto_min_us as u64
}

/// Link configuration (local side). See spec 5.8 for the defaults.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkConfig {
    /// Host or NCP.
    pub role: Role,
    /// Advertised receive window (4..=32, and ≤ the `RXW` slots).
    pub rx_window: u8,
    /// Delayed-ack timer of our receiver (µs).
    pub ack_delay_us: u16,
    /// Proposed keepalive interval (ms).
    pub keepalive_ms: u16,
    /// Proposed link timeout (ms).
    pub link_timeout_ms: u16,
    /// Initial RTO (µs).
    pub rto_initial_us: u32,
    /// RTO lower bound for the adaptive estimator (µs).
    pub rto_min_us: u32,
    /// RTO upper bound (µs).
    pub rto_max_us: u32,
    /// Use RFC 6298 RTT samples (Karn) to adapt the RTO.
    pub adaptive_rto: bool,
    /// Retransmissions of one frame before the link is declared failed.
    pub n_retx_max: u8,
    /// First `RESET_*` retry interval (ms), doubling up to the maximum.
    pub reset_retry_ms: u16,
    /// Maximum `RESET_*` retry interval (ms).
    pub reset_retry_max_ms: u16,
    /// Initial credit we grant the peer per endpoint ID (index 0 unused).
    pub initial_credits: [u8; Endpoint::COUNT],
    /// Return a delivered frame's credit automatically when the next event
    /// is polled. Disable to call [`Link::release_credit`] explicitly when
    /// the consumer frees its buffer.
    pub auto_credit: bool,
    /// Transport line rate in baud (8N1 UART), or 0 when unknown (USB).
    /// When known, the link models its own wire: every frame's RTO runs
    /// from the moment its last byte has left the line (bytes queued ahead
    /// of it included), the RTO never falls below [`rto_floor_us`], and DATA
    /// is held back while [`ACK_FIRST_BACKLOG`] bytes or more are still
    /// waiting to go out, so ACKs never queue behind a deep transmit buffer
    /// (spec 5.6). Change it at runtime with [`Link::set_line_rate`].
    pub line_rate_bps: u32,
}

impl LinkConfig {
    /// NCP defaults: window 16, grants SYSTEM 4, ZIGBEE 8, TABLES 4,
    /// BULK 2, LOG 0.
    pub const fn ncp() -> Self {
        Self {
            role: Role::Ncp,
            rx_window: 16,
            ack_delay_us: 1000,
            keepalive_ms: 500,
            link_timeout_ms: 3000,
            rto_initial_us: 100_000,
            rto_min_us: 25_000,
            rto_max_us: 1_000_000,
            adaptive_rto: true,
            n_retx_max: 10,
            reset_retry_ms: 200,
            reset_retry_max_ms: 2000,
            initial_credits: [0, 4, 8, 4, 2, 0],
            auto_credit: true,
            line_rate_bps: 0,
        }
    }

    /// Desktop host defaults: window 32, grants SYSTEM 4, ZIGBEE 8,
    /// TABLES 4, BULK 4, LOG 2.
    pub const fn desktop_host() -> Self {
        Self {
            role: Role::Host,
            rx_window: 32,
            initial_credits: [0, 4, 8, 4, 4, 2],
            ..Self::ncp()
        }
    }

    /// Embedded host defaults (3.3): window 8, LOG credit 0, BULK 2.
    pub const fn embedded_host() -> Self {
        Self {
            role: Role::Host,
            rx_window: 8,
            initial_credits: [0, 4, 8, 4, 2, 0],
            ..Self::ncp()
        }
    }
}

/// Session state (spec 5.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkState {
    /// Not opened.
    Unsynced,
    /// Handshake in progress.
    Resetting,
    /// Session established.
    Established,
}

/// How a received `RESET_*` relates to the last known peer cookie.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeerChange {
    /// No peer cookie was known before.
    New,
    /// Different cookie: the peer rebooted or reopened (`PEER_RESTARTED`).
    Restarted,
    /// Same cookie: the peer lost synchronisation only (`PEER_RESYNC`).
    Resync,
}

/// Negotiated session parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SessionInfo {
    /// Session epoch (1..=255).
    pub epoch: u8,
    /// Our cookie.
    pub own_cookie: u32,
    /// Peer cookie.
    pub peer_cookie: u32,
    /// Send window (`min(peer rx_window, TXW)`).
    pub tx_window: u8,
    /// Peer `rx_mtu` (capped at 640).
    pub peer_rx_mtu: u16,
    /// Negotiated keepalive (ms).
    pub keepalive_ms: u16,
    /// Negotiated link timeout (ms).
    pub link_timeout_ms: u32,
}

/// Why a transmitted DATA frame failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxFailure {
    /// The session reset before the frame was acknowledged; every message in
    /// it fails with `LINK_RESET` (outcome unknown, 6.5).
    LinkReset,
    /// The peer REJECTed the frame (raw [`RejectReason`](crate::frame::RejectReason));
    /// every message in it fails with `LINK_REJECTED`.
    Rejected(u8),
}

/// Events for the upper layer, drained with [`Link::poll_event`].
#[derive(Debug, PartialEq, Eq)]
pub enum LinkEvent<'a> {
    /// The peer sent `RESET_*` with a cookie that is new, different
    /// (`PEER_RESTARTED`) or the same (`PEER_RESYNC`). All ARQ state was
    /// reset.
    PeerReset {
        /// Peer cookie.
        cookie: u32,
        /// Relation to the previous cookie.
        change: PeerChange,
    },
    /// No valid frame for `link_timeout_ms` (`LINK_DOWN`); a reset with
    /// reason 1 has started.
    LinkDown,
    /// A frame exceeded `n_retx_max` retransmissions (`LINK_FAILED`), or the
    /// peer reported a window violation; a reset has started.
    LinkFailed,
    /// Session established (after a handshake). The NCP must now send
    /// `EVT_NCP_READY` first; the host must send `SYS_NEGOTIATE` first.
    Established(SessionInfo),
    /// A DATA payload delivered in per-endpoint order. Parse it with
    /// [`Batch::parse`](crate::msg::Batch::parse).
    Data {
        /// Endpoint.
        endpoint: Endpoint,
        /// Batched messages.
        payload: &'a [u8],
    },
    /// A DATA frame we sent failed; report each message in `payload`.
    TxFailed {
        /// Endpoint.
        endpoint: Endpoint,
        /// The frame's batched messages.
        payload: &'a [u8],
        /// Reason.
        reason: TxFailure,
    },
}

/// Link counters (spec 5.12), saturating.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct LinkCounters {
    pub tx_frames: u32,
    pub tx_bytes: u32,
    pub tx_retransmissions: u32,
    pub tx_rto_expiries: u32,
    pub tx_rejected: u32,
    pub tx_blocked_credit_ms: u32,
    pub tx_blocked_window_ms: u32,
    pub tx_discarded_no_host: u32,
    pub naks_sent: u32,
    pub naks_received: u32,
    pub sessions_established: u32,
    pub link_down_events: u32,
    pub rx_frames: u32,
    pub rx_bytes: u32,
    pub rx_bad_frame: u32,
    pub rx_duplicate: u32,
    pub rx_out_of_order: u32,
    pub rx_epoch_mismatch: u32,
    pub rx_not_in_session: u32,
    pub rx_rejected_sent: u32,
    pub rx_overflow: u32,
    pub rx_stale_reset: u32,
    pub peer_restarted: u32,
    pub peer_resync: u32,
    /// Retransmissions acknowledged before their copy could have left our
    /// line, i.e. the peer had acknowledged an earlier copy (spurious). A
    /// lower bound; 0 while the line rate is unknown.
    pub tx_spurious_retx: u32,
}

impl LinkCounters {
    /// `(counter ID, value)` pairs for `DIAG_GET_COUNTERS` group `0x01`.
    pub fn entries(&self) -> [(u16, u32); 25] {
        [
            (0x0101, self.tx_frames),
            (0x0102, self.tx_bytes),
            (0x0103, self.tx_retransmissions),
            (0x0104, self.tx_rto_expiries),
            (0x0105, self.tx_rejected),
            (0x0106, self.tx_blocked_credit_ms),
            (0x0107, self.tx_blocked_window_ms),
            (0x0108, self.tx_discarded_no_host),
            (0x0109, self.naks_sent),
            (0x010A, self.naks_received),
            (0x010B, self.sessions_established),
            (0x010C, self.link_down_events),
            (0x010D, self.tx_spurious_retx),
            (0x0111, self.rx_frames),
            (0x0112, self.rx_bytes),
            (0x0113, self.rx_bad_frame),
            (0x0114, self.rx_duplicate),
            (0x0115, self.rx_out_of_order),
            (0x0116, self.rx_epoch_mismatch),
            (0x0117, self.rx_not_in_session),
            (0x0118, self.rx_rejected_sent),
            (0x0119, self.rx_overflow),
            (0x011A, self.rx_stale_reset),
            (0x011B, self.peer_restarted),
            (0x011C, self.peer_resync),
        ]
    }
}

#[inline]
pub(crate) fn inc(c: &mut u32) {
    *c = c.saturating_add(1);
}
