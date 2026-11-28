//! The link state machine. See the module docs of [`crate::link`].

use super::{
    ACK_FIRST_BACKLOG, BITS_PER_BYTE, Instant, LinkConfig, LinkCounters, LinkEvent, LinkState,
    PeerChange, SessionInfo, TxFailure, inc, rto_floor_us, wire_time_us,
};
use crate::cobs::{Accumulator, Feed};
use crate::frame::{
    self, AckBody, CreditEntry, FrameError, FrameType, Header, MAX_ENCODED_FRAME, MAX_PAYLOAD,
    RejectBody, RejectReason, ResetBody, ResetReason, Role, TX_BUFFER_LEN,
};
use crate::queue::TxSource;
use crate::types::Endpoint;

const NEP: usize = Endpoint::COUNT;
/// Received-bitmap span (`ack+1 .. ack+32`) and duplicate window.
const SACK_SPAN: u8 = 32;
/// Idle time after which a `0x00` precedes the next frame (5.1).
const RESYNC_IDLE_US: u64 = 100_000;
/// Rate limit of REJECT `STALE_EPOCH` / `UNSUPPORTED` (5.5).
const REJECT_RATE_US: u64 = 100_000;
/// Anti-starvation threshold (5.10).
const STARVATION_FRAMES: u8 = 8;
/// Pending REJECT queue.
const REJECT_QUEUE: usize = 4;
/// Pending control-event queue.
const CTL_QUEUE: usize = 4;
/// Re-check interval while DATA waits behind a transport backlog of
/// unknown drain rate.
const GATE_RETRY_US: u64 = 1_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TxState {
    Free,
    InFlight,
    Failed(TxFailure),
}

#[derive(Debug, Clone)]
struct TxSlot {
    state: TxState,
    seq: u8,
    endpoint: u8,
    ep_seq: u8,
    len: u16,
    sacked: bool,
    needs_retx: bool,
    retx_poll: bool,
    retx_count: u8,
    /// When the last byte of the latest copy leaves our line (the time it
    /// was produced while the line rate is unknown); its RTO runs from here.
    wire_done: Instant,
    last_tx_order: u64,
    payload: [u8; MAX_PAYLOAD],
}

impl TxSlot {
    const EMPTY: Self = Self {
        state: TxState::Free,
        seq: 0,
        endpoint: 0,
        ep_seq: 0,
        len: 0,
        sacked: false,
        needs_retx: false,
        retx_poll: false,
        retx_count: 0,
        wire_done: 0,
        last_tx_order: 0,
        payload: [0; MAX_PAYLOAD],
    };
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RxState {
    Free,
    /// Received, waiting for earlier `ep_seq`s of its endpoint.
    Buffered,
    /// Rejected frame that still consumes its `ep_seq`.
    Tombstone,
    /// Deliverable; `u32` is the delivery order.
    Ready(u32),
}

#[derive(Debug, Clone)]
struct RxSlot {
    state: RxState,
    endpoint: u8,
    ep_seq: u8,
    len: u16,
    payload: [u8; MAX_PAYLOAD],
}

impl RxSlot {
    const EMPTY: Self = Self {
        state: RxState::Free,
        endpoint: 0,
        ep_seq: 0,
        len: 0,
        payload: [0; MAX_PAYLOAD],
    };
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AckReq {
    None,
    Ack,
    Nak,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ResetTx {
    Req,
    Ack,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ctl {
    PeerReset(u32, PeerChange),
    LinkDown,
    LinkFailed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Lent {
    None,
    Rx(usize),
    Tx(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Peer {
    rx_window: u8,
    rx_mtu: u16,
    ack_delay_us: u16,
    keepalive_ms: u16,
    link_timeout_ms: u16,
    credits: [u8; NEP],
}

impl Peer {
    const NONE: Self = Self {
        rx_window: 4,
        rx_mtu: MAX_PAYLOAD as u16,
        ack_delay_us: 1000,
        keepalive_ms: 500,
        link_timeout_ms: 3000,
        credits: [0; NEP],
    };
}

/// The ZRHP link state machine with `TXW` transmit and `RXW` receive slots.
///
/// See the [module documentation](crate::link) for the call pattern.
pub struct Link<const TXW: usize, const RXW: usize> {
    dec: Accumulator<MAX_ENCODED_FRAME>,
    core: Core<TXW, RXW>,
}

impl<const TXW: usize, const RXW: usize> core::fmt::Debug for Link<TXW, RXW> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Link")
            .field("state", &self.core.state)
            .field("epoch", &self.core.epoch)
            .field("next_seq", &self.core.next_seq)
            .field("base", &self.core.base)
            .field("rx_ack", &self.core.rx_ack)
            .finish_non_exhaustive()
    }
}

struct Core<const TXW: usize, const RXW: usize> {
    cfg: LinkConfig,
    cookie: u32,
    state: LinkState,
    peer_known: Option<u32>,
    answered: Option<u32>,
    session_peer: u32,
    epoch: u8,
    peer: Peer,
    keepalive_us: u64,
    link_timeout_us: u64,
    tx_window: u8,
    reset_reason: u8,
    reset_tx: Option<ResetTx>,
    reset_retry_at: Instant,
    reset_retry_ms: u32,
    dup_ack_at: Option<Instant>,
    rx_in_session: bool,
    any_tx: bool,
    last_tx: Instant,
    last_rx_valid: Instant,
    // Transmit ARQ.
    tx: [TxSlot; TXW],
    next_seq: u8,
    base: u8,
    tx_order: u64,
    d_max: Option<u64>,
    nbad_handled: u8,
    tx_credit_edge: [u8; NEP],
    next_ep_seq: [u8; NEP],
    srtt: Option<u64>,
    rttvar: u64,
    rto: u64,
    /// RTO lower bound: `rto_min` raised by the line-rate terms.
    rto_floor: u64,
    /// Line model: when our transmit line drains (line rate known).
    wire_free_at: Instant,
    /// Bytes the transport reported still queued at this poll.
    reported_backlog: usize,
    /// DATA was ready but held back by the ACK-first rule ...
    data_gated: bool,
    /// ... until about this time.
    gate_open_at: Instant,
    blocked_since: Option<Instant>,
    blocked_by_window: bool,
    consecutive_high: u8,
    last_served: [u64; NEP],
    served_counter: u64,
    // Receive ARQ.
    rx: [RxSlot; RXW],
    rx_ack: u8,
    rx_bitmap: u32,
    expected_ep_seq: [u8; NEP],
    rx_credit_edge: [u8; NEP],
    last_rx_ep_seq: [Option<u8>; NEP],
    credit_dirty: u8,
    nbad: u8,
    nbad_reported: u8,
    frames_since_ack: u8,
    delayed_ack_at: Option<Instant>,
    ack_req: AckReq,
    poll_reply: bool,
    keepalive_due: bool,
    crc_holdoff_until: Option<Instant>,
    crc_nak_after_holdoff: bool,
    rejects: [Option<RejectBody>; REJECT_QUEUE],
    last_stale_reject: Option<Instant>,
    ready_counter: u32,
    // Upper-layer events.
    ctl: [Option<Ctl>; CTL_QUEUE],
    established_pending: bool,
    lent: Lent,
    counters: LinkCounters,
}

impl<const TXW: usize, const RXW: usize> Link<TXW, RXW> {
    /// A link in [`LinkState::Unsynced`]. `cookie` must be a fresh random
    /// non-zero value per boot (NCP) or per port open (host).
    ///
    /// `const`, so the large slot arrays can live in a `static`.
    pub const fn new(cfg: LinkConfig, cookie: u32) -> Self {
        assert!(TXW >= 1 && TXW <= 128, "TXW must be 1..=128");
        assert!(RXW >= 4 && RXW <= 32, "RXW must be 4..=32");
        Self {
            dec: Accumulator::new(),
            core: Core::new(cfg, cookie),
        }
    }

    /// Start (or restart) the session handshake: drop all ARQ state, fail
    /// unacknowledged frames with `LINK_RESET`, and send `0x00` +
    /// `RESET_REQ`, repeated every `reset_retry` until a peer answers.
    ///
    /// A new `cookie` may be supplied when the host reopens its port.
    pub fn open(&mut self, now: Instant, reason: ResetReason) {
        self.dec.reset();
        self.core.begin_reset(now, reason as u8);
    }

    /// Like [`open`](Self::open) with a new cookie (host port reopen).
    pub fn reopen(&mut self, now: Instant, cookie: u32, reason: ResetReason) {
        self.core.cookie = cookie;
        self.open(now, reason);
    }

    /// Current state.
    pub fn state(&self) -> LinkState {
        self.core.state
    }

    /// Our cookie.
    pub fn cookie(&self) -> u32 {
        self.core.cookie
    }

    /// The last known peer cookie.
    pub fn peer_cookie(&self) -> Option<u32> {
        self.core.peer_known
    }

    /// Session parameters while established.
    pub fn session(&self) -> Option<SessionInfo> {
        (self.core.state == LinkState::Established).then(|| self.core.session_info())
    }

    /// Link counters.
    pub fn counters(&self) -> &LinkCounters {
        &self.core.counters
    }

    /// Reset all counters to zero (`DIAG_RESET_COUNTERS`).
    pub fn reset_counters(&mut self) {
        self.core.counters = LinkCounters::default();
    }

    /// Current retransmission timeout (µs), counted from the moment a frame
    /// has left our line.
    pub fn rto_us(&self) -> u64 {
        self.core.rto
    }

    /// Lower bound of the RTO (µs): `rto_min_us`, raised by the line-rate
    /// terms of [`rto_floor_us`](super::rto_floor_us) once the line rate and
    /// the peer's `ack_delay` are known.
    pub fn rto_floor_us(&self) -> u64 {
        self.core.rto_floor
    }

    /// Transport line rate (baud), 0 when unknown.
    pub fn line_rate(&self) -> u32 {
        self.core.cfg.line_rate_bps
    }

    /// Set the transport line rate (baud; 0 = unknown, e.g. USB) after a
    /// `SYS_SET_UART` switch or a host line change. See
    /// [`LinkConfig::line_rate_bps`].
    pub fn set_line_rate(&mut self, bps: u32) {
        self.core.set_line_rate(bps);
    }

    /// Bytes still waiting ahead on our transmit side at `now`: the line
    /// model's estimate, or the transport figure passed to the last
    /// [`poll_transmit_with_backlog`](Self::poll_transmit_with_backlog) if
    /// larger.
    pub fn tx_backlog(&self, now: Instant) -> usize {
        self.core.tx_backlog(now)
    }

    /// Oldest unacknowledged sequence number.
    pub fn tx_base(&self) -> u8 {
        self.core.base
    }

    /// Next DATA sequence number.
    pub fn tx_next_seq(&self) -> u8 {
        self.core.next_seq
    }

    /// Sequence numbers of frames acknowledged neither cumulatively nor by
    /// SACK, unordered (the "unacked" set of the ARQ traces).
    pub fn tx_unacked(&self) -> impl Iterator<Item = u8> + '_ {
        self.core
            .tx
            .iter()
            .filter(|s| s.state == TxState::InFlight && !s.sacked)
            .map(|s| s.seq)
    }

    /// Sequence numbers of every frame still occupying a window slot
    /// (including SACKed frames above the cumulative ack), unordered.
    pub fn tx_in_flight(&self) -> impl Iterator<Item = u8> + '_ {
        self.core
            .tx
            .iter()
            .filter(|s| s.state == TxState::InFlight)
            .map(|s| s.seq)
    }

    /// Next expected DATA sequence number on receive (`ack`).
    pub fn rx_ack(&self) -> u8 {
        self.core.rx_ack
    }

    /// `true` if a new DATA frame on `endpoint` would be admitted now
    /// (window, credit and BULK/LOG quota, rule S0).
    pub fn can_send(&self, endpoint: Endpoint) -> bool {
        self.core.state == LinkState::Established
            && self.core.admissible(endpoint)
            && self.core.free_tx_slot().is_some()
    }

    /// Feed received bytes. Returns how many bytes were consumed; it stops
    /// early while delivered frames are waiting in
    /// [`poll_event`](Self::poll_event), so call it again with the remainder after
    /// draining events.
    pub fn push_rx(&mut self, data: &[u8], now: Instant) -> usize {
        self.core.release_lent();
        let mut i = 0;
        while i < data.len() {
            if self.core.has_ready() {
                break;
            }
            let b = data[i];
            i += 1;
            match self.dec.push(b) {
                Feed::Pending => {}
                Feed::Frame(n) => {
                    let frame = self.dec.frame(n);
                    self.core.on_frame(frame, now);
                }
                Feed::Bad(_) => self.core.on_bad_frame(now),
            }
        }
        self.core.counters.rx_bytes = self.core.counters.rx_bytes.saturating_add(i as u32);
        i
    }

    /// Report a transport line error (UART framing/parity/overrun): the
    /// current frame is aborted and counted as one `nbad` (4.1).
    pub fn on_line_error(&mut self, now: Instant, overrun: bool) {
        self.dec.line_error();
        if overrun {
            inc(&mut self.core.counters.rx_overflow);
        }
        self.core.on_bad_frame(now);
    }

    /// Count a frame the transport had to discard because no host was
    /// reading (USB-Serial-JTAG FIFO full, section 11).
    pub fn note_discarded_no_host(&mut self) {
        inc(&mut self.core.counters.tx_discarded_no_host);
    }

    /// Next upper-layer event. A borrowed payload stays valid until the next
    /// call on the link.
    pub fn poll_event(&mut self) -> Option<LinkEvent<'_>> {
        self.core.poll_event()
    }

    /// Return `frames` receive credits on `endpoint` after the consumer
    /// freed its buffers (rule R8). Only needed with `auto_credit = false`.
    pub fn release_credit(&mut self, endpoint: Endpoint, frames: u8) {
        self.core.release_credit(endpoint as usize, frames);
    }

    /// Produce the next wire frame (COBS + delimiter, optionally preceded by
    /// `0x00`) into `out`, processing timers due at `now` first. Returns the
    /// number of bytes to write, or `None` when nothing is due. Call it until
    /// it returns `None`, then sleep until [`next_deadline`](Self::next_deadline)
    /// or new input.
    ///
    /// ACK-first (spec 5.6): ACK, NAK, REJECT and RESET frames come first
    /// and are always produced; DATA (new or retransmitted) only while less
    /// than [`ACK_FIRST_BACKLOG`](super::ACK_FIRST_BACKLOG) bytes are
    /// waiting on our transmit side. Without a line rate and a reported
    /// backlog nothing is held back.
    pub fn poll_transmit(
        &mut self,
        now: Instant,
        src: &mut impl TxSource,
        out: &mut [u8; TX_BUFFER_LEN],
    ) -> Option<usize> {
        self.poll_transmit_with_backlog(now, src, out, 0)
    }

    /// [`poll_transmit`](Self::poll_transmit) for a transport that knows how
    /// many bytes it has accepted but not yet sent (`tx_backlog`: its
    /// transmit ring or DMA buffer). The link holds DATA back while that
    /// figure, or its own line model, reaches
    /// [`ACK_FIRST_BACKLOG`](super::ACK_FIRST_BACKLOG), and starts the
    /// frame's RTO only after those bytes and the frame itself are out.
    pub fn poll_transmit_with_backlog(
        &mut self,
        now: Instant,
        src: &mut impl TxSource,
        out: &mut [u8; TX_BUFFER_LEN],
        tx_backlog: usize,
    ) -> Option<usize> {
        self.core.release_lent();
        self.core.reported_backlog = tx_backlog;
        self.core.handle_timers(now);
        self.core.transmit(now, src, out)
    }

    /// Earliest time a timer needs [`poll_transmit`](Self::poll_transmit).
    pub fn next_deadline(&self) -> Option<Instant> {
        self.core.next_deadline()
    }
}

fn min_opt(a: Option<Instant>, b: Option<Instant>) -> Option<Instant> {
    match (a, b) {
        (Some(x), Some(y)) => Some(x.min(y)),
        (x, None) => x,
        (None, y) => y,
    }
}

impl<const TXW: usize, const RXW: usize> Core<TXW, RXW> {
    const fn new(cfg: LinkConfig, cookie: u32) -> Self {
        Self {
            cfg,
            cookie,
            state: LinkState::Unsynced,
            peer_known: None,
            answered: None,
            session_peer: 0,
            epoch: 0,
            peer: Peer::NONE,
            keepalive_us: 500_000,
            link_timeout_us: 3_000_000,
            tx_window: 4,
            reset_reason: 0,
            reset_tx: None,
            reset_retry_at: 0,
            reset_retry_ms: 200,
            dup_ack_at: None,
            rx_in_session: false,
            any_tx: false,
            last_tx: 0,
            last_rx_valid: 0,
            tx: [TxSlot::EMPTY; TXW],
            next_seq: 0,
            base: 0,
            tx_order: 0,
            d_max: None,
            nbad_handled: 0,
            tx_credit_edge: [0; NEP],
            next_ep_seq: [0; NEP],
            srtt: None,
            rttvar: 0,
            rto: cfg.rto_initial_us as u64,
            rto_floor: rto_floor_us(
                cfg.line_rate_bps,
                Peer::NONE.ack_delay_us as u32,
                cfg.rto_min_us,
            ),
            wire_free_at: 0,
            reported_backlog: 0,
            data_gated: false,
            gate_open_at: 0,
            blocked_since: None,
            blocked_by_window: false,
            consecutive_high: 0,
            last_served: [0; NEP],
            served_counter: 0,
            rx: [RxSlot::EMPTY; RXW],
            rx_ack: 0,
            rx_bitmap: 0,
            expected_ep_seq: [0; NEP],
            rx_credit_edge: [0; NEP],
            last_rx_ep_seq: [None; NEP],
            credit_dirty: 0,
            nbad: 0,
            nbad_reported: 0,
            frames_since_ack: 0,
            delayed_ack_at: None,
            ack_req: AckReq::None,
            poll_reply: false,
            keepalive_due: false,
            crc_holdoff_until: None,
            crc_nak_after_holdoff: false,
            rejects: [None; REJECT_QUEUE],
            last_stale_reject: None,
            ready_counter: 0,
            ctl: [None; CTL_QUEUE],
            established_pending: false,
            lent: Lent::None,
            counters: LinkCounters {
                tx_frames: 0,
                tx_bytes: 0,
                tx_retransmissions: 0,
                tx_rto_expiries: 0,
                tx_rejected: 0,
                tx_blocked_credit_ms: 0,
                tx_blocked_window_ms: 0,
                tx_discarded_no_host: 0,
                naks_sent: 0,
                naks_received: 0,
                sessions_established: 0,
                link_down_events: 0,
                rx_frames: 0,
                rx_bytes: 0,
                rx_bad_frame: 0,
                rx_duplicate: 0,
                rx_out_of_order: 0,
                rx_epoch_mismatch: 0,
                rx_not_in_session: 0,
                rx_rejected_sent: 0,
                rx_overflow: 0,
                rx_stale_reset: 0,
                peer_restarted: 0,
                peer_resync: 0,
                tx_spurious_retx: 0,
            },
        }
    }

    fn rx_window(&self) -> u8 {
        self.cfg.rx_window.clamp(4, RXW as u8)
    }

    fn session_info(&self) -> SessionInfo {
        SessionInfo {
            epoch: self.epoch,
            own_cookie: self.cookie,
            peer_cookie: self.session_peer,
            tx_window: self.tx_window,
            peer_rx_mtu: self.peer.rx_mtu,
            keepalive_ms: (self.keepalive_us / 1000) as u16,
            link_timeout_ms: (self.link_timeout_us / 1000) as u32,
        }
    }

    // ── Session ─────────────────────────────────────────────────────────

    fn begin_reset(&mut self, now: Instant, reason: u8) {
        self.fail_all_inflight();
        self.clear_rx();
        self.state = LinkState::Resetting;
        self.answered = None;
        self.reset_reason = reason;
        self.reset_tx = Some(ResetTx::Req);
        self.reset_retry_ms = self.cfg.reset_retry_ms as u32;
        self.reset_retry_at = now + self.reset_retry_ms as u64 * 1000;
        self.any_tx = false;
    }

    fn push_ctl(&mut self, c: Ctl) {
        if let Some(slot) = self.ctl.iter_mut().find(|s| s.is_none()) {
            *slot = Some(c);
        } else {
            // Keep the newest: shift out the oldest.
            self.ctl.rotate_left(1);
            self.ctl[CTL_QUEUE - 1] = Some(c);
        }
    }

    fn store_peer(&mut self, body: &ResetBody<'_>) {
        let mut credits = [0u8; NEP];
        for c in body.credits.clone() {
            if let Some(ep) = Endpoint::from_u8(c.endpoint).filter(|e| *e != Endpoint::Link) {
                credits[ep as usize] = c.value;
            }
        }
        self.peer = Peer {
            rx_window: body.rx_window.clamp(4, 32),
            rx_mtu: body.rx_mtu.min(MAX_PAYLOAD as u16),
            ack_delay_us: body.ack_delay_us,
            keepalive_ms: body.keepalive_ms.max(1),
            link_timeout_ms: body.link_timeout_ms,
            credits,
        };
    }

    fn on_reset(&mut self, ftype: FrameType, payload: &[u8], now: Instant) {
        let Ok(body) = ResetBody::parse(payload) else {
            inc(&mut self.counters.rx_stale_reset);
            return;
        };
        let peer_role = if self.cfg.role == Role::Host {
            Role::Ncp
        } else {
            Role::Host
        };
        if body.role != peer_role as u8 || body.cookie == 0 || body.link_version < 1 {
            inc(&mut self.counters.rx_stale_reset);
            return;
        }
        let c = body.cookie;
        match ftype {
            FrameType::ResetReq => {
                if self.state == LinkState::Resetting && self.answered == Some(c) {
                    // Retried request: our RESET_ACK was lost.
                    self.store_peer(&body);
                    self.reset_tx = Some(ResetTx::Ack);
                    return;
                }
                let change = self.note_peer_cookie(c);
                self.store_peer(&body);
                self.fail_all_inflight();
                self.clear_rx();
                self.push_ctl(Ctl::PeerReset(c, change));
                self.state = LinkState::Resetting;
                self.answered = Some(c);
                self.reset_tx = Some(ResetTx::Ack);
                self.reset_retry_ms = self.cfg.reset_retry_ms as u32;
                self.reset_retry_at = now + self.reset_retry_ms as u64 * 1000;
            }
            _ => {
                if body.peer_cookie != self.cookie {
                    inc(&mut self.counters.rx_stale_reset);
                    return;
                }
                if self.state == LinkState::Established && self.session_peer == c {
                    // Duplicate RESET_ACK: the peer is still RESETTING because
                    // our RESET_ACK was lost. Answer once more (rate-limited)
                    // unless the peer has already spoken in-session.
                    if !self.rx_in_session
                        && self
                            .dup_ack_at
                            .is_none_or(|t| now >= t + self.cfg.reset_retry_ms as u64 * 1000)
                    {
                        self.dup_ack_at = Some(now);
                        self.reset_tx = Some(ResetTx::Ack);
                    }
                    return;
                }
                if self.peer_known != Some(c) {
                    let change = self.note_peer_cookie(c);
                    self.fail_all_inflight();
                    self.clear_rx();
                    self.push_ctl(Ctl::PeerReset(c, change));
                }
                self.store_peer(&body);
                if self.answered != Some(c) {
                    self.answered = Some(c);
                    self.reset_tx = Some(ResetTx::Ack);
                } else {
                    self.reset_tx = None;
                }
                self.establish(c, now);
            }
        }
    }

    fn note_peer_cookie(&mut self, c: u32) -> PeerChange {
        let change = match self.peer_known {
            None => PeerChange::New,
            Some(k) if k == c => PeerChange::Resync,
            Some(_) => PeerChange::Restarted,
        };
        match change {
            PeerChange::Resync => inc(&mut self.counters.peer_resync),
            _ => inc(&mut self.counters.peer_restarted),
        }
        self.peer_known = Some(c);
        change
    }

    fn establish(&mut self, peer_cookie: u32, now: Instant) {
        self.state = LinkState::Established;
        self.session_peer = peer_cookie;
        let (h, n) = match self.cfg.role {
            Role::Host => (self.cookie, peer_cookie),
            Role::Ncp => (peer_cookie, self.cookie),
        };
        self.epoch = frame::session_epoch(h, n);
        // Transmit ARQ.
        self.next_seq = 0;
        self.base = 0;
        self.d_max = None;
        self.nbad_handled = 0;
        self.tx_credit_edge = self.peer.credits;
        self.next_ep_seq = [0; NEP];
        self.srtt = None;
        self.rttvar = 0;
        self.update_rto_floor();
        self.rto = if self.cfg.line_rate_bps == 0 {
            self.cfg.rto_initial_us as u64
        } else {
            let (lo, hi) = self.rto_bounds();
            (self.cfg.rto_initial_us as u64).clamp(lo, hi)
        };
        self.data_gated = false;
        self.blocked_since = None;
        self.consecutive_high = 0;
        // Receive ARQ.
        self.rx_ack = 0;
        self.rx_bitmap = 0;
        self.expected_ep_seq = [0; NEP];
        self.rx_credit_edge = self.cfg.initial_credits;
        self.rx_credit_edge[0] = 0;
        self.last_rx_ep_seq = [None; NEP];
        self.credit_dirty = 0;
        self.nbad = 0;
        self.nbad_reported = 0;
        self.frames_since_ack = 0;
        self.delayed_ack_at = None;
        self.ack_req = AckReq::None;
        self.poll_reply = false;
        self.keepalive_due = false;
        self.crc_holdoff_until = None;
        self.crc_nak_after_holdoff = false;
        self.rejects = [None; REJECT_QUEUE];
        // Negotiation (5.5).
        let ka = self.cfg.keepalive_ms.min(self.peer.keepalive_ms).max(1) as u64;
        self.keepalive_us = ka * 1000;
        let lt = (self.cfg.link_timeout_ms as u64)
            .max(self.peer.link_timeout_ms as u64)
            .max(3 * ka);
        self.link_timeout_us = lt * 1000;
        self.tx_window = self.peer.rx_window.min(TXW.min(32) as u8);
        self.last_rx_valid = now;
        self.last_tx = now;
        self.rx_in_session = false;
        self.dup_ack_at = None;
        self.established_pending = true;
        inc(&mut self.counters.sessions_established);
    }

    fn fail_all_inflight(&mut self) {
        for s in &mut self.tx {
            if s.state == TxState::InFlight {
                s.state = TxState::Failed(TxFailure::LinkReset);
                s.needs_retx = false;
            }
        }
    }

    fn clear_rx(&mut self) {
        // Ready frames were accepted (and possibly acknowledged); they are
        // still delivered. Buffered frames were at most sacked: the peer
        // fails them with LINK_RESET.
        for s in &mut self.rx {
            if matches!(s.state, RxState::Buffered | RxState::Tombstone) {
                s.state = RxState::Free;
            }
        }
        self.rx_bitmap = 0;
    }

    fn link_failed(&mut self, now: Instant, reason: ResetReason) {
        self.push_ctl(Ctl::LinkFailed);
        self.begin_reset(now, reason as u8);
    }

    // ── Receive ─────────────────────────────────────────────────────────

    fn has_ready(&self) -> bool {
        self.rx.iter().any(|s| matches!(s.state, RxState::Ready(_)))
    }

    fn on_bad_frame(&mut self, now: Instant) {
        inc(&mut self.counters.rx_bad_frame);
        self.nbad = self.nbad.saturating_add(1);
        if self.state != LinkState::Established {
            return;
        }
        match self.crc_holdoff_until {
            Some(t) if now < t => self.crc_nak_after_holdoff = true,
            _ => {
                self.ack_req = AckReq::Nak;
                self.crc_holdoff_until = Some(now + self.cfg.ack_delay_us as u64);
                self.crc_nak_after_holdoff = false;
            }
        }
    }

    fn queue_reject(&mut self, body: RejectBody) {
        if let Some(slot) = self.rejects.iter_mut().find(|s| s.is_none()) {
            *slot = Some(body);
            inc(&mut self.counters.rx_rejected_sent);
        }
    }

    fn on_frame(&mut self, decoded: &[u8], now: Instant) {
        let f = match frame::parse(decoded) {
            Ok(f) => f,
            Err(e) if e.is_noise() => return self.on_bad_frame(now),
            Err(FrameError::Version(_) | FrameError::Type(_)) => {
                // Valid CRC: a peer bug or version mismatch, not noise.
                self.nbad = 0;
                if self.state == LinkState::Established && self.stale_reject_allowed(now) {
                    let h = frame::raw_header(decoded).unwrap_or_default();
                    self.queue_reject(RejectBody {
                        seq: h[3],
                        endpoint: h[2],
                        ep_seq: h[5],
                        reason: RejectReason::Unsupported as u8,
                    });
                }
                return;
            }
            Err(_) => return,
        };
        inc(&mut self.counters.rx_frames);
        self.nbad = 0;
        let h = f.header;
        match h.ftype {
            FrameType::ResetReq | FrameType::ResetAck => {
                return self.on_reset(h.ftype, f.payload, now);
            }
            _ => {}
        }
        if self.state != LinkState::Established {
            inc(&mut self.counters.rx_not_in_session);
            return;
        }
        if h.epoch != self.epoch {
            inc(&mut self.counters.rx_epoch_mismatch);
            if h.ftype == FrameType::Data && self.stale_reject_allowed(now) {
                self.queue_reject(RejectBody {
                    seq: h.seq,
                    endpoint: h.endpoint,
                    ep_seq: h.ep_seq,
                    reason: RejectReason::StaleEpoch as u8,
                });
            }
            return;
        }
        self.last_rx_valid = now;
        self.rx_in_session = true;
        match h.ftype {
            FrameType::Data => {
                self.process_ack(h.ack, None, now);
                self.on_data(&h, f.payload, now);
            }
            FrameType::Ack | FrameType::Nak => {
                let Ok(body) = AckBody::parse(f.payload) else {
                    return;
                };
                for c in body.credits.clone() {
                    if let Some(ep) = Endpoint::from_u8(c.endpoint).filter(|e| *e != Endpoint::Link)
                    {
                        self.tx_credit_edge[ep as usize] = c.value;
                    }
                }
                self.process_ack(h.ack, Some(body.sack), now);
                if h.ftype == FrameType::Nak {
                    inc(&mut self.counters.naks_received);
                    self.process_nbad(body.nbad);
                }
            }
            FrameType::Reject => {
                if let Ok(body) = RejectBody::parse(f.payload) {
                    self.on_reject(body, now);
                }
                if self.state == LinkState::Established {
                    self.process_ack(h.ack, None, now);
                }
            }
            FrameType::ResetReq | FrameType::ResetAck => {}
        }
        if h.poll && self.state == LinkState::Established {
            self.poll_reply = true;
            if self.ack_req == AckReq::None {
                self.ack_req = AckReq::Ack;
            }
        }
    }

    fn stale_reject_allowed(&mut self, now: Instant) -> bool {
        if self
            .last_stale_reject
            .is_none_or(|t| now >= t + REJECT_RATE_US)
        {
            self.last_stale_reject = Some(now);
            true
        } else {
            false
        }
    }

    fn request_ack(&mut self, kind: AckReq) {
        if kind == AckReq::Nak || self.ack_req == AckReq::None {
            self.ack_req = kind;
        }
    }

    fn free_rx_slot(&self) -> Option<usize> {
        self.rx.iter().position(|s| s.state == RxState::Free)
    }

    fn on_data(&mut self, h: &Header, payload: &[u8], now: Instant) {
        let d = h.seq.wrapping_sub(self.rx_ack);
        let rx_window = self.rx_window();
        let already = d > 0 && d <= SACK_SPAN && self.rx_bitmap & (1 << (d - 1)) != 0;
        if d >= 0u8.wrapping_sub(SACK_SPAN) || already {
            // R4 duplicate.
            inc(&mut self.counters.rx_duplicate);
            self.request_ack(AckReq::Ack);
            return;
        }
        if d >= rx_window {
            // R5 outside the window.
            self.queue_reject(RejectBody {
                seq: h.seq,
                endpoint: h.endpoint,
                ep_seq: h.ep_seq,
                reason: RejectReason::WindowViolation as u8,
            });
            return;
        }
        let ep = Endpoint::from_u8(h.endpoint).filter(|e| *e != Endpoint::Link);
        let reject = match ep {
            None => Some(RejectReason::UnknownEndpoint),
            Some(_) if payload.len() > MAX_PAYLOAD => Some(RejectReason::TooLong),
            Some(e) => {
                let avail = self.rx_credit_edge[e as usize].wrapping_sub(h.ep_seq);
                if (1..=128).contains(&avail) {
                    None
                } else {
                    Some(RejectReason::NoCredit)
                }
            }
        };
        // A frame needs a slot unless it is rejected for an unknown endpoint.
        let slot = match (ep, reject) {
            (None, _) => None,
            _ => match self.free_rx_slot() {
                Some(i) => Some(i),
                None => {
                    // Cannot happen while RXW >= rx_window and events are
                    // drained; drop unrecorded so the peer retransmits.
                    inc(&mut self.counters.rx_overflow);
                    return;
                }
            },
        };
        // Record the seq as received.
        let hole_filled = d == 0 && self.rx_bitmap != 0;
        if d == 0 {
            self.rx_ack = self.rx_ack.wrapping_add(1);
            loop {
                let b0 = self.rx_bitmap & 1;
                self.rx_bitmap >>= 1;
                if b0 == 0 {
                    break;
                }
                self.rx_ack = self.rx_ack.wrapping_add(1);
            }
        } else {
            self.rx_bitmap |= 1 << (d - 1);
        }
        if let (Some(e), Some(i)) = (ep, slot) {
            let s = &mut self.rx[i];
            s.endpoint = e as u8;
            s.ep_seq = h.ep_seq;
            if reject.is_some() {
                s.state = RxState::Tombstone;
                s.len = 0;
            } else {
                s.state = RxState::Buffered;
                s.len = payload.len() as u16;
                s.payload[..payload.len()].copy_from_slice(payload);
                self.last_rx_ep_seq[e as usize] = Some(h.ep_seq);
            }
            self.deliver_ready(e as usize);
        }
        if let Some(r) = reject {
            self.queue_reject(RejectBody {
                seq: h.seq,
                endpoint: h.endpoint,
                ep_seq: h.ep_seq,
                reason: r as u8,
            });
        }
        if d == 0 {
            if hole_filled {
                // R3.
                self.request_ack(AckReq::Ack);
            } else {
                // R1.
                self.frames_since_ack = self.frames_since_ack.saturating_add(1);
                if self.delayed_ack_at.is_none() {
                    self.delayed_ack_at = Some(now + self.cfg.ack_delay_us as u64);
                }
                if self.frames_since_ack >= (rx_window / 2).max(1) {
                    self.request_ack(AckReq::Ack);
                }
            }
        } else {
            // R2.
            inc(&mut self.counters.rx_out_of_order);
            self.request_ack(AckReq::Nak);
        }
    }

    fn deliver_ready(&mut self, e: usize) {
        loop {
            let exp = self.expected_ep_seq[e];
            let Some(i) = self.rx.iter().position(|s| {
                matches!(s.state, RxState::Buffered | RxState::Tombstone)
                    && s.endpoint as usize == e
                    && s.ep_seq == exp
            }) else {
                return;
            };
            self.expected_ep_seq[e] = exp.wrapping_add(1);
            if self.rx[i].state == RxState::Tombstone {
                // A rejected frame consumed its ep_seq without holding a
                // buffer: return that credit so the edge keeps tracking the
                // free buffers.
                self.rx[i].state = RxState::Free;
                self.rx_credit_edge[e] = self.rx_credit_edge[e].wrapping_add(1);
                self.credit_dirty |= 1 << e;
            } else {
                self.rx[i].state = RxState::Ready(self.ready_counter);
                self.ready_counter = self.ready_counter.wrapping_add(1);
            }
        }
    }

    fn release_credit(&mut self, e: usize, frames: u8) {
        if e == 0 || e >= NEP || frames == 0 || self.state != LinkState::Established {
            return;
        }
        let old = self.rx_credit_edge[e];
        self.rx_credit_edge[e] = old.wrapping_add(frames);
        self.credit_dirty |= 1 << e;
        // R8: the peer is blocked on this endpoint when its next `ep_seq`
        // is the old edge, including an endpoint granted no credit at all
        // (an embedded host's LOG endpoint, granted later).
        if self.last_rx_ep_seq[e].map_or(0, |seq| seq.wrapping_add(1)) == old {
            self.request_ack(AckReq::Ack);
        }
    }

    // ── Transmit-side acknowledgement processing ────────────────────────

    fn process_ack(&mut self, ack: u8, sack: Option<u32>, now: Instant) {
        let inflight = self.next_seq.wrapping_sub(self.base);
        let adv = ack.wrapping_sub(self.base);
        if adv > inflight {
            return; // stale or bogus
        }
        let mut d: Option<u64> = None;
        let mut sample: Option<u64> = None;
        let mut spurious = 0u32;
        let base = self.base;
        for s in &mut self.tx {
            if s.state != TxState::InFlight {
                continue;
            }
            let off = s.seq.wrapping_sub(base);
            let sacked_now = off >= adv
                && sack.is_some_and(|bits| {
                    let i = s.seq.wrapping_sub(ack).wrapping_sub(1);
                    i < SACK_SPAN && bits & (1 << i) != 0
                });
            if (off < adv || sacked_now) && !s.sacked {
                d = Some(d.map_or(s.last_tx_order, |x| x.max(s.last_tx_order)));
                if s.retx_count == 0 {
                    // From the moment the frame left our line (Karn: only
                    // frames sent once).
                    sample = Some(now.saturating_sub(s.wire_done));
                } else if now < s.wire_done {
                    // The latest copy has not even left our line: the peer
                    // acknowledged an earlier one.
                    spurious += 1;
                }
            }
            if off < adv {
                s.state = TxState::Free;
                s.needs_retx = false;
            } else if sacked_now {
                s.sacked = true;
                s.needs_retx = false;
            }
        }
        self.counters.tx_spurious_retx = self.counters.tx_spurious_retx.saturating_add(spurious);
        self.base = ack;
        if let Some(r) = sample.filter(|_| self.cfg.adaptive_rto) {
            self.rtt_sample(r);
        }
        if let Some(dv) = d {
            // S1: FIFO loss inference.
            for s in &mut self.tx {
                if s.state == TxState::InFlight && !s.sacked && s.last_tx_order < dv {
                    s.needs_retx = true;
                }
            }
            if self.d_max.is_none_or(|m| dv > m) {
                self.d_max = Some(dv);
                self.nbad_handled = 0;
            }
            self.blocked_since = None;
        }
    }

    fn process_nbad(&mut self, k: u8) {
        // S2.
        if k <= self.nbad_handled {
            return;
        }
        let d_max = self.d_max;
        let mut idx: [usize; TXW] = [usize::MAX; TXW];
        let mut n = 0;
        for (i, s) in self.tx.iter().enumerate() {
            if s.state == TxState::InFlight
                && !s.sacked
                && d_max.is_none_or(|m| s.last_tx_order > m)
            {
                idx[n] = i;
                n += 1;
            }
        }
        let list = &mut idx[..n];
        list.sort_unstable_by_key(|&i| self.tx[i].last_tx_order);
        let from = self.nbad_handled as usize;
        let to = (k as usize).min(n);
        for &i in list.iter().take(to).skip(from) {
            self.tx[i].needs_retx = true;
        }
        self.nbad_handled = k;
    }

    fn on_reject(&mut self, body: RejectBody, now: Instant) {
        match RejectReason::from_u8(body.reason) {
            Some(
                RejectReason::NoCredit | RejectReason::TooLong | RejectReason::UnknownEndpoint,
            ) => {
                if let Some(s) = self
                    .tx
                    .iter_mut()
                    .find(|s| s.state == TxState::InFlight && s.seq == body.seq)
                {
                    s.state = TxState::Failed(TxFailure::Rejected(body.reason));
                    s.needs_retx = false;
                    inc(&mut self.counters.tx_rejected);
                }
            }
            Some(RejectReason::WindowViolation) => {
                // The receiver could not record the seq: the session cannot
                // make progress. Report the frame and reset (protocol error).
                if let Some(s) = self
                    .tx
                    .iter_mut()
                    .find(|s| s.state == TxState::InFlight && s.seq == body.seq)
                {
                    s.state = TxState::Failed(TxFailure::Rejected(body.reason));
                    inc(&mut self.counters.tx_rejected);
                }
                self.link_failed(now, ResetReason::ProtocolError);
            }
            // STALE_EPOCH / NOT_IN_SESSION / UNSUPPORTED do not concern a
            // frame of this session.
            _ => {}
        }
    }

    fn rtt_sample(&mut self, r: u64) {
        // RFC 6298.
        match self.srtt {
            None => {
                self.srtt = Some(r);
                self.rttvar = r / 2;
            }
            Some(srtt) => {
                let diff = srtt.abs_diff(r);
                self.rttvar = (3 * self.rttvar + diff) / 4;
                self.srtt = Some((7 * srtt + r) / 8);
            }
        }
        let rto = self.srtt.unwrap_or(r) + 4 * self.rttvar;
        let (lo, hi) = self.rto_bounds();
        self.rto = rto.clamp(lo, hi);
    }

    /// `(RTO lower bound, RTO_max)`; the line-rate floor wins over `RTO_max`
    /// on a line too slow for it.
    fn rto_bounds(&self) -> (u64, u64) {
        let lo = self.rto_floor;
        (lo, (self.cfg.rto_max_us as u64).max(lo))
    }

    fn update_rto_floor(&mut self) {
        self.rto_floor = rto_floor_us(
            self.cfg.line_rate_bps,
            self.peer.ack_delay_us as u32,
            self.cfg.rto_min_us,
        );
    }

    fn set_line_rate(&mut self, bps: u32) {
        if bps == self.cfg.line_rate_bps {
            return;
        }
        self.cfg.line_rate_bps = bps;
        self.update_rto_floor();
        if self.state == LinkState::Established {
            let (lo, _) = self.rto_bounds();
            self.rto = self.rto.max(lo);
        }
    }

    /// Bytes still waiting ahead on our transmit side at `now`: the line
    /// model's or the transport's figure, whichever is larger.
    fn tx_backlog(&self, now: Instant) -> usize {
        let bps = self.cfg.line_rate_bps as u64;
        let modelled = if bps == 0 || self.wire_free_at <= now {
            0
        } else {
            ((self.wire_free_at - now) * bps).div_ceil(BITS_PER_BYTE * 1_000_000) as usize
        };
        modelled.max(self.reported_backlog)
    }

    /// ACK-first rule: `true` (and the time to look again recorded) while
    /// DATA must wait for the transmit backlog to drain.
    fn data_held_back(&mut self, now: Instant) -> bool {
        let backlog = self.tx_backlog(now);
        if backlog < ACK_FIRST_BACKLOG {
            return false;
        }
        let bps = self.cfg.line_rate_bps;
        self.gate_open_at = if bps == 0 {
            now + GATE_RETRY_US
        } else {
            now + wire_time_us(backlog + 1 - ACK_FIRST_BACKLOG, bps).max(1)
        };
        true
    }

    // ── Timers ──────────────────────────────────────────────────────────

    fn oldest_unsacked(&self) -> Option<usize> {
        self.tx
            .iter()
            .enumerate()
            .filter(|(_, s)| s.state == TxState::InFlight && !s.sacked)
            .min_by_key(|(_, s)| s.last_tx_order)
            .map(|(i, _)| i)
    }

    fn handle_timers(&mut self, now: Instant) {
        match self.state {
            LinkState::Unsynced => {}
            LinkState::Resetting => {
                if now >= self.reset_retry_at {
                    self.reset_tx = Some(if self.answered.is_some() {
                        ResetTx::Ack
                    } else {
                        ResetTx::Req
                    });
                    self.any_tx = false;
                    self.reset_retry_ms =
                        (self.reset_retry_ms * 2).min(self.cfg.reset_retry_max_ms as u32);
                    self.reset_retry_at = now + self.reset_retry_ms as u64 * 1000;
                }
            }
            LinkState::Established => {
                if now >= self.last_rx_valid + self.link_timeout_us {
                    inc(&mut self.counters.link_down_events);
                    self.push_ctl(Ctl::LinkDown);
                    self.begin_reset(now, ResetReason::PeerTimeout as u8);
                    return;
                }
                if self.delayed_ack_at.is_some_and(|t| now >= t) {
                    self.request_ack(AckReq::Ack);
                }
                if let Some(t) = self.crc_holdoff_until.filter(|t| now >= *t) {
                    let _ = t;
                    self.crc_holdoff_until = None;
                    if self.crc_nak_after_holdoff && self.nbad != self.nbad_reported {
                        self.ack_req = AckReq::Nak;
                        self.crc_holdoff_until = Some(now + self.cfg.ack_delay_us as u64);
                    }
                    self.crc_nak_after_holdoff = false;
                }
                // S3: RTO on the oldest unacked, unsacked frame.
                if let Some(i) = self.oldest_unsacked() {
                    let s = &self.tx[i];
                    if !s.needs_retx && now >= s.wire_done + self.rto {
                        inc(&mut self.counters.tx_rto_expiries);
                        self.tx[i].needs_retx = true;
                        self.tx[i].retx_poll = true;
                        self.rto = (self.rto * 2).min(self.cfg.rto_max_us as u64);
                    }
                }
                if now >= self.last_tx + self.keepalive_us {
                    self.keepalive_due = true;
                }
                // S5: blocked sender.
                if let Some(t) = self.blocked_since.filter(|t| now >= *t + self.keepalive_us) {
                    let ms = ((now - t) / 1000) as u32;
                    if self.blocked_by_window {
                        self.counters.tx_blocked_window_ms =
                            self.counters.tx_blocked_window_ms.saturating_add(ms);
                    } else {
                        self.counters.tx_blocked_credit_ms =
                            self.counters.tx_blocked_credit_ms.saturating_add(ms);
                    }
                    self.keepalive_due = true;
                    self.blocked_since = Some(now);
                }
            }
        }
    }

    fn next_deadline(&self) -> Option<Instant> {
        match self.state {
            LinkState::Unsynced => None,
            LinkState::Resetting => {
                if self.reset_tx.is_some() {
                    Some(0)
                } else {
                    Some(self.reset_retry_at)
                }
            }
            LinkState::Established => {
                let immediate = self.reset_tx.is_some()
                    || self.ack_req != AckReq::None
                    || self.keepalive_due
                    || self.rejects.iter().any(Option::is_some)
                    || (!self.data_gated
                        && self
                            .tx
                            .iter()
                            .any(|s| s.state == TxState::InFlight && s.needs_retx));
                if immediate {
                    return Some(0);
                }
                let mut t = Some(self.last_rx_valid + self.link_timeout_us);
                t = min_opt(t, Some(self.last_tx + self.keepalive_us));
                t = min_opt(t, self.delayed_ack_at);
                if self.crc_nak_after_holdoff {
                    t = min_opt(t, self.crc_holdoff_until);
                }
                if let Some(i) = self.oldest_unsacked() {
                    t = min_opt(t, Some(self.tx[i].wire_done + self.rto));
                }
                if let Some(b) = self.blocked_since {
                    t = min_opt(t, Some(b + self.keepalive_us));
                }
                if self.data_gated {
                    t = min_opt(t, Some(self.gate_open_at));
                }
                t
            }
        }
    }

    // ── Transmit ────────────────────────────────────────────────────────

    fn lead_zero(&mut self, now: Instant) -> bool {
        let lead = !self.any_tx || now >= self.last_tx + RESYNC_IDLE_US;
        self.any_tx = true;
        lead
    }

    /// Account a frame of `n` wire bytes produced at `now`; returns when its
    /// last byte leaves our line (`now` while the line rate is unknown).
    fn sent(&mut self, now: Instant, n: usize) -> Instant {
        self.last_tx = now;
        inc(&mut self.counters.tx_frames);
        self.counters.tx_bytes = self.counters.tx_bytes.saturating_add(n as u32);
        let bps = self.cfg.line_rate_bps;
        if bps == 0 {
            return now;
        }
        let ahead = now + wire_time_us(self.reported_backlog, bps);
        self.wire_free_at = self.wire_free_at.max(ahead) + wire_time_us(n, bps);
        self.wire_free_at
    }

    fn transmit(
        &mut self,
        now: Instant,
        src: &mut impl TxSource,
        out: &mut [u8; TX_BUFFER_LEN],
    ) -> Option<usize> {
        if let Some(kind) = self.reset_tx.take() {
            return Some(self.send_reset(kind, now, out));
        }
        if self.state != LinkState::Established {
            return None;
        }
        if let Some(i) = self.rejects.iter().position(Option::is_some) {
            let body = self.rejects[i].take().unwrap_or(RejectBody {
                seq: 0,
                endpoint: 0,
                ep_seq: 0,
                reason: 0,
            });
            let mut h = Header::new(FrameType::Reject);
            h.epoch = self.epoch;
            h.ack = self.rx_ack;
            let lead = self.lead_zero(now);
            let n = frame::encode_wire(&h, &[&body.to_bytes()], lead, out).ok()?;
            self.sent(now, n);
            return Some(n);
        }
        if self.ack_req != AckReq::None {
            let kind = self.ack_req;
            let poll = false;
            return Some(self.send_ack(kind, poll, false, now, out));
        }
        // ACK-first: control frames above never wait; DATA only joins a
        // short transmit backlog.
        if self.data_held_back(now) {
            self.data_gated = self
                .tx
                .iter()
                .any(|s| s.state == TxState::InFlight && s.needs_retx)
                || Endpoint::DATA.iter().any(|e| src.ready(*e));
        } else {
            self.data_gated = false;
            if let Some(n) = self.send_retransmission(now, out) {
                return Some(n);
            }
            if let Some(n) = self.send_new(now, src, out) {
                return Some(n);
            }
        }
        if self.keepalive_due {
            self.keepalive_due = false;
            return Some(self.send_ack(AckReq::Ack, true, true, now, out));
        }
        None
    }

    fn send_reset(&mut self, kind: ResetTx, now: Instant, out: &mut [u8; TX_BUFFER_LEN]) -> usize {
        let (ftype, peer_cookie) = match kind {
            ResetTx::Req => (FrameType::ResetReq, self.peer_known.unwrap_or(0)),
            ResetTx::Ack => (
                FrameType::ResetAck,
                self.answered.or(self.peer_known).unwrap_or(0),
            ),
        };
        let body = ResetBody {
            cookie: self.cookie,
            peer_cookie,
            role: self.cfg.role as u8,
            link_version: frame::LINK_VERSION,
            rx_window: self.rx_window(),
            reason: self.reset_reason,
            rx_mtu: MAX_PAYLOAD as u16,
            ack_delay_us: self.cfg.ack_delay_us,
            keepalive_ms: self.cfg.keepalive_ms,
            link_timeout_ms: self.cfg.link_timeout_ms,
            credits: frame::Credits::default(),
        };
        let mut credits = [CreditEntry {
            endpoint: 0,
            value: 0,
        }; 5];
        for (i, ep) in Endpoint::DATA.iter().enumerate() {
            credits[i] = CreditEntry {
                endpoint: *ep as u8,
                value: self.cfg.initial_credits[*ep as usize],
            };
        }
        let mut payload = [0u8; ResetBody::FIXED_LEN + 10];
        let len = body.write(&credits, &mut payload).unwrap_or(0);
        let lead = self.lead_zero(now);
        let n = frame::encode_wire(&Header::new(ftype), &[&payload[..len]], lead, out).unwrap_or(0);
        self.sent(now, n);
        n
    }

    fn send_ack(
        &mut self,
        kind: AckReq,
        poll: bool,
        all_credits: bool,
        now: Instant,
        out: &mut [u8; TX_BUFFER_LEN],
    ) -> usize {
        let mut credits = [CreditEntry {
            endpoint: 0,
            value: 0,
        }; 5];
        let mut nc = 0;
        for ep in Endpoint::DATA {
            let e = ep as usize;
            if all_credits || self.credit_dirty & (1 << e) != 0 {
                credits[nc] = CreditEntry {
                    endpoint: e as u8,
                    value: self.rx_credit_edge[e],
                };
                nc += 1;
            }
        }
        self.credit_dirty = 0;
        let mut payload = [0u8; 16];
        let len =
            AckBody::write(self.rx_bitmap, self.nbad, &credits[..nc], &mut payload).unwrap_or(0);
        let mut h = Header::new(if kind == AckReq::Nak {
            FrameType::Nak
        } else {
            FrameType::Ack
        });
        h.epoch = self.epoch;
        h.ack = self.rx_ack;
        h.poll = poll;
        let lead = self.lead_zero(now);
        let n = frame::encode_wire(&h, &[&payload[..len]], lead, out).unwrap_or(0);
        if kind == AckReq::Nak {
            inc(&mut self.counters.naks_sent);
        }
        self.nbad_reported = self.nbad;
        self.frames_since_ack = 0;
        self.delayed_ack_at = None;
        self.ack_req = AckReq::None;
        self.poll_reply = false;
        self.sent(now, n);
        n
    }

    fn send_retransmission(
        &mut self,
        now: Instant,
        out: &mut [u8; TX_BUFFER_LEN],
    ) -> Option<usize> {
        let i = self
            .tx
            .iter()
            .enumerate()
            .filter(|(_, s)| s.state == TxState::InFlight && s.needs_retx)
            .min_by_key(|(_, s)| s.last_tx_order)
            .map(|(i, _)| i)?;
        if self.tx[i].retx_count >= self.cfg.n_retx_max {
            // S4.
            self.link_failed(now, ResetReason::RetryExhausted);
            return self.reset_tx.take().map(|k| self.send_reset(k, now, out));
        }
        self.tx_order += 1;
        let order = self.tx_order;
        let lead = self.lead_zero(now);
        let epoch = self.epoch;
        let ack = self.rx_ack;
        let s = &mut self.tx[i];
        s.needs_retx = false;
        s.retx_count += 1;
        s.last_tx_order = order;
        let mut h = Header::new(FrameType::Data);
        h.epoch = epoch;
        h.endpoint = s.endpoint;
        h.seq = s.seq;
        h.ack = ack;
        h.ep_seq = s.ep_seq;
        h.retx = true;
        h.poll = core::mem::take(&mut s.retx_poll);
        let n = frame::encode_wire(&h, &[&s.payload[..s.len as usize]], lead, out).ok()?;
        inc(&mut self.counters.tx_retransmissions);
        self.tx[i].wire_done = self.sent(now, n);
        Some(n)
    }

    fn free_tx_slot(&self) -> Option<usize> {
        self.tx.iter().position(|s| s.state == TxState::Free)
    }

    fn window_open(&self) -> bool {
        self.next_seq.wrapping_sub(self.base) < self.tx_window
    }

    fn credit_ok(&self, e: usize) -> bool {
        (1..=128).contains(&self.tx_credit_edge[e].wrapping_sub(self.next_ep_seq[e]))
    }

    fn bulk_quota_ok(&self) -> bool {
        let used = self
            .tx
            .iter()
            .filter(|s| {
                s.state == TxState::InFlight
                    && Endpoint::from_u8(s.endpoint).is_some_and(Endpoint::is_bulk_class)
            })
            .count();
        used < (self.tx_window as usize / 2).max(1)
    }

    fn admissible(&self, ep: Endpoint) -> bool {
        ep != Endpoint::Link
            && self.window_open()
            && self.credit_ok(ep as usize)
            && (!ep.is_bulk_class() || self.bulk_quota_ok())
    }

    fn send_new(
        &mut self,
        now: Instant,
        src: &mut impl TxSource,
        out: &mut [u8; TX_BUFFER_LEN],
    ) -> Option<usize> {
        // Ready endpoints in priority order.
        let mut by_prio = Endpoint::DATA;
        by_prio.sort_unstable_by_key(|e| e.priority());
        let any_ready = by_prio.iter().any(|e| src.ready(*e));
        if !any_ready {
            self.blocked_since = None;
            return None;
        }
        let slot = self.free_tx_slot();
        let mut cands = [Endpoint::Link; 5];
        let mut nc = 0;
        if slot.is_some() {
            for e in by_prio {
                if src.ready(e) && self.admissible(e) {
                    cands[nc] = e;
                    nc += 1;
                }
            }
        }
        if nc == 0 {
            if self.blocked_since.is_none() {
                self.blocked_since = Some(now);
                self.blocked_by_window = !self.window_open() || slot.is_none();
            }
            return None;
        }
        let cands = &cands[..nc];
        let mut chosen = cands[0];
        if nc > 1 {
            if self.consecutive_high >= STARVATION_FRAMES {
                // Anti-starvation: the longest-waiting lower-priority endpoint.
                chosen = *cands[1..]
                    .iter()
                    .min_by_key(|e| self.last_served[**e as usize])
                    .unwrap_or(&chosen);
                self.consecutive_high = 0;
            } else {
                self.consecutive_high += 1;
            }
        } else {
            self.consecutive_high = 0;
        }
        let i = slot?;
        let e = chosen as usize;
        let mtu = (self.peer.rx_mtu as usize).min(MAX_PAYLOAD);
        let n = src.fill(chosen, &mut self.tx[i].payload[..mtu]);
        if n == 0 || n > mtu {
            return None;
        }
        self.blocked_since = None;
        self.served_counter += 1;
        self.last_served[e] = self.served_counter;
        self.tx_order += 1;
        let seq = self.next_seq;
        let ep_seq = self.next_ep_seq[e];
        self.next_seq = seq.wrapping_add(1);
        self.next_ep_seq[e] = ep_seq.wrapping_add(1);
        let order = self.tx_order;
        let lead = self.lead_zero(now);
        let s = &mut self.tx[i];
        s.state = TxState::InFlight;
        s.seq = seq;
        s.endpoint = e as u8;
        s.ep_seq = ep_seq;
        s.len = n as u16;
        s.sacked = false;
        s.needs_retx = false;
        s.retx_poll = false;
        s.retx_count = 0;
        s.wire_done = now;
        s.last_tx_order = order;
        let mut h = Header::new(FrameType::Data);
        h.epoch = self.epoch;
        h.endpoint = e as u8;
        h.seq = seq;
        h.ack = self.rx_ack;
        h.ep_seq = ep_seq;
        let w = frame::encode_wire(&h, &[&self.tx[i].payload[..n]], lead, out).ok()?;
        self.tx[i].wire_done = self.sent(now, w);
        Some(w)
    }

    // ── Events ──────────────────────────────────────────────────────────

    fn release_lent(&mut self) {
        match core::mem::replace(&mut self.lent, Lent::None) {
            Lent::None => {}
            Lent::Rx(i) => {
                let e = self.rx[i].endpoint as usize;
                self.rx[i].state = RxState::Free;
                if self.cfg.auto_credit {
                    self.release_credit(e, 1);
                }
            }
            Lent::Tx(i) => self.tx[i].state = TxState::Free,
        }
    }

    fn poll_event(&mut self) -> Option<LinkEvent<'_>> {
        self.release_lent();
        if let Some(c) = self.ctl[0].take() {
            self.ctl.rotate_left(1);
            return Some(match c {
                Ctl::PeerReset(cookie, change) => LinkEvent::PeerReset { cookie, change },
                Ctl::LinkDown => LinkEvent::LinkDown,
                Ctl::LinkFailed => LinkEvent::LinkFailed,
            });
        }
        if let Some(i) = self
            .tx
            .iter()
            .enumerate()
            .filter(|(_, s)| matches!(s.state, TxState::Failed(_)))
            .min_by_key(|(_, s)| s.last_tx_order)
            .map(|(i, _)| i)
        {
            self.lent = Lent::Tx(i);
            let s = &self.tx[i];
            let TxState::Failed(reason) = s.state else {
                return None;
            };
            return Some(LinkEvent::TxFailed {
                endpoint: Endpoint::from_u8(s.endpoint).unwrap_or(Endpoint::Link),
                payload: &s.payload[..s.len as usize],
                reason,
            });
        }
        if self.established_pending {
            self.established_pending = false;
            if self.state == LinkState::Established {
                return Some(LinkEvent::Established(self.session_info()));
            }
        }
        let i = self
            .rx
            .iter()
            .enumerate()
            .filter_map(|(i, s)| {
                if let RxState::Ready(o) = s.state {
                    Some((i, o))
                } else {
                    None
                }
            })
            .min_by_key(|(_, o)| *o)
            .map(|(i, _)| i)?;
        self.lent = Lent::Rx(i);
        let s = &self.rx[i];
        Some(LinkEvent::Data {
            endpoint: Endpoint::from_u8(s.endpoint).unwrap_or(Endpoint::Link),
            payload: &s.payload[..s.len as usize],
        })
    }
}
