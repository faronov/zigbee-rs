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
//! | Profile | `TXW` | `RXW` | RAM (approx.) |
//! |---|---:|---:|---:|
//! | ESP32-H2 NCP | 16 | 16 | 21.6 KB |
//! | ESP32-S3 embedded host | 8 (or 4) | 8 (or 4) | 11.1 KB (5.9 KB) |
//! | Desktop host | 32 | 32 | 42 KB |
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

use crate::frame::Role;
use crate::types::Endpoint;

/// Monotonic time in microseconds, supplied by the caller.
pub type Instant = u64;

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
}

impl LinkCounters {
    /// `(counter ID, value)` pairs for `DIAG_GET_COUNTERS` group `0x01`.
    pub fn entries(&self) -> [(u16, u32); 24] {
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
