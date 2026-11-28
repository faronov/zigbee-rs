//! ACK-first transmission and the line-rate RTO floor (spec 5.6): two real
//! links over a simulated 8N1 UART without flow control, each side with a
//! bounded transmit buffer drained at the line rate, both streaming (ZCL
//! read bursts from the host, responses and indications from the NCP).

mod common;

use std::collections::{BTreeMap, VecDeque};

use common::Rng;
use common::harness::{OnePerFrame, Rig, decode_wire, tagged_payload, wire};
use zigbee_zrhp::Endpoint;
use zigbee_zrhp::frame::{FrameType, ResetReason, TX_BUFFER_LEN};
use zigbee_zrhp::link::{
    ACK_FIRST_BACKLOG, ACK_WIRE_MAX, Link, LinkConfig, LinkEvent, LinkState, PEER_ACK_AHEAD,
    rto_floor_us, wire_time_us,
};
use zigbee_zrhp::msg::{Batch, BatchWriter, Kind};
use zigbee_zrhp::queue::{NoSource, TxSource};

const BAUD: u32 = 115_200;
/// USB-UART bridge latency (WSTK VCOM) in each direction.
const BRIDGE_NS: u64 = 1_000_000;
/// The EFR32MG24 NCP's interrupt-driven host transmit ring.
const NCP_TX_RING: usize = 2048;
/// OS serial buffers on a desktop host: effectively unbounded.
const HOST_TX_BUFFER: usize = 64 * 1024;
const STEP_US: u64 = 100;

// ── Wire ────────────────────────────────────────────────────────────────

/// One direction: the sender's transmit buffer, drained at the line rate.
struct Uart {
    /// One byte on the 8N1 line, rounded up (the simulated line is a few
    /// ppm slower than the links' model).
    byte_ns: u64,
    cap: usize,
    /// Bytes not yet delivered: (time the last bit left the line, byte,
    /// lost on the wire).
    bytes: VecDeque<(u64, u8, bool)>,
    free_at_ns: u64,
    max_backlog: usize,
}

impl Uart {
    fn new(baud: u32, cap: usize) -> Self {
        Self {
            byte_ns: 10_000_000_000u64.div_ceil(baud as u64),
            cap,
            bytes: VecDeque::new(),
            free_at_ns: 0,
            max_backlog: 0,
        }
    }

    /// Accepted but not yet on the line.
    fn backlog(&self, now_ns: u64) -> usize {
        self.bytes.iter().filter(|b| b.0 > now_ns).count()
    }

    fn write(&mut self, now_ns: u64, data: &[u8], lost: bool) -> usize {
        let n = data
            .len()
            .min(self.cap.saturating_sub(self.backlog(now_ns)));
        for &b in &data[..n] {
            self.free_at_ns = self.free_at_ns.max(now_ns) + self.byte_ns;
            self.bytes.push_back((self.free_at_ns, b, lost));
        }
        self.max_backlog = self.max_backlog.max(self.backlog(now_ns));
        n
    }

    fn deliver(&mut self, now_ns: u64) -> Vec<u8> {
        let mut out = Vec::new();
        while self
            .bytes
            .front()
            .is_some_and(|b| b.0 + BRIDGE_NS <= now_ns)
        {
            let (_, b, lost) = self.bytes.pop_front().unwrap();
            if !lost {
                out.push(b);
            }
        }
        out
    }
}

// ── Applications ────────────────────────────────────────────────────────

/// Queued messages of one endpoint, batched into frames like the NCP's and
/// the hub's queues.
#[derive(Default)]
struct Queue {
    msgs: VecDeque<(Kind, u16, usize)>,
}

impl TxSource for Queue {
    fn ready(&self, endpoint: Endpoint) -> bool {
        endpoint == Endpoint::Zigbee && !self.msgs.is_empty()
    }
    fn fill(&mut self, endpoint: Endpoint, buf: &mut [u8]) -> usize {
        if endpoint != Endpoint::Zigbee {
            return 0;
        }
        let body = [0x5Au8; 640];
        let mut w = BatchWriter::new(buf);
        while let Some(&(kind, tid, len)) = self.msgs.front() {
            if w.push_raw(kind, false, 0x0201, tid, &body[..len]).is_err() {
                break;
            }
            self.msgs.pop_front();
        }
        w.len()
    }
}

struct Side<const T: usize, const R: usize> {
    link: Link<T, R>,
    queue: Queue,
    uart: Uart,
    /// Pass the transmit buffer's fill level to the link.
    report_backlog: bool,
    staging: Vec<u8>,
    staged_off: usize,
    staged_lost: bool,
    /// tid → time it was queued / delivered (µs).
    queued: BTreeMap<u16, u64>,
    delivered: Vec<(u16, u64)>,
    requests_in: Vec<u16>,
    data_frames_out: u32,
}

impl<const T: usize, const R: usize> Side<T, R> {
    fn new(cfg: LinkConfig, cookie: u32, uart: Uart, report_backlog: bool) -> Self {
        Self {
            link: Link::new(cfg, cookie),
            queue: Queue::default(),
            uart,
            report_backlog,
            staging: Vec::new(),
            staged_off: 0,
            staged_lost: false,
            queued: BTreeMap::new(),
            delivered: Vec::new(),
            requests_in: Vec::new(),
            data_frames_out: 0,
        }
    }

    fn push(&mut self, kind: Kind, tid: u16, len: usize, now: u64) {
        self.queue.msgs.push_back((kind, tid, len));
        self.queued.insert(tid, now);
    }

    fn feed(&mut self, bytes: &[u8], now: u64) {
        let mut off = 0;
        while off < bytes.len() {
            off += self.link.push_rx(&bytes[off..], now);
            while let Some(ev) = self.link.poll_event() {
                match ev {
                    LinkEvent::Data { payload, .. } => {
                        for m in Batch::parse(payload).unwrap().iter() {
                            self.delivered.push((m.header.tid, now));
                            if m.header.kind == Kind::Request {
                                self.requests_in.push(m.header.tid);
                            }
                        }
                    }
                    LinkEvent::TxFailed { .. } | LinkEvent::LinkFailed | LinkEvent::LinkDown => {
                        panic!("unexpected {ev:?}")
                    }
                    _ => {}
                }
            }
        }
    }

    /// The platform glue: write the staged frame, then take the next one
    /// while the transmit buffer accepts it. `lose` decides whether a DATA
    /// frame is lost on the wire (it still occupies the line).
    fn pump(&mut self, now: u64, lose: &mut Loss<'_>) {
        let now_ns = now * 1000;
        let mut out = [0u8; TX_BUFFER_LEN];
        loop {
            if self.staged_off < self.staging.len() {
                self.staged_off +=
                    self.uart
                        .write(now_ns, &self.staging[self.staged_off..], self.staged_lost);
                if self.staged_off < self.staging.len() {
                    return;
                }
            }
            let backlog = if self.report_backlog {
                self.uart.backlog(now_ns)
            } else {
                0
            };
            let Some(n) =
                self.link
                    .poll_transmit_with_backlog(now, &mut self.queue, &mut out, backlog)
            else {
                return;
            };
            self.staging.clear();
            self.staging.extend_from_slice(&out[..n]);
            self.staged_off = 0;
            self.staged_lost = false;
            if decode_wire(&out[..n])
                .iter()
                .any(|(h, _)| h.ftype == FrameType::Data)
            {
                self.staged_lost = lose(self.data_frames_out);
                self.data_frames_out += 1;
            }
        }
    }

    /// Every queued message arrived exactly once, in order, at `peer`.
    fn check_delivered_by<const T2: usize, const R2: usize>(&self, peer: &Side<T2, R2>) {
        let tids: Vec<u16> = peer.delivered.iter().map(|d| d.0).collect();
        assert!(tids.windows(2).all(|w| w[0] < w[1]), "order/duplicates");
        assert_eq!(tids, self.queued.keys().copied().collect::<Vec<_>>());
    }

    /// Longest queue-to-delivery latency of our messages at `peer` (µs).
    fn max_latency<const T2: usize, const R2: usize>(&self, peer: &Side<T2, R2>) -> u64 {
        peer.delivered
            .iter()
            .map(|(tid, t)| t - self.queued[tid])
            .max()
            .unwrap_or(0)
    }
}

#[derive(Debug, Default, PartialEq, Eq)]
struct Outcome {
    /// Duplicates seen by the receivers (each one a spurious or needed
    /// retransmission the peer had already received).
    rx_duplicates: u32,
    retransmissions: u32,
    rto_expiries: u32,
    spurious_detected: u32,
    host_max_latency_us: u64,
    ncp_max_latency_us: u64,
    ncp_max_tx_backlog: usize,
    host_max_tx_backlog: usize,
}

struct Sim {
    host: Side<8, 8>,
    ncp: Side<16, 16>,
    now: u64,
    rng: Rng,
    next_tid_host: u16,
    next_tid_ncp: u16,
}

/// A 115 200 baud line; `line_rate` given to both links (0 = today's
/// behaviour without it).
fn sim(line_rate: u32) -> Sim {
    sim2(BAUD, line_rate, line_rate, 0x5EED_F00D)
}

fn sim2(baud: u32, host_rate: u32, ncp_rate: u32, seed: u64) -> Sim {
    let hcfg = LinkConfig {
        line_rate_bps: host_rate,
        ..LinkConfig::embedded_host()
    };
    let ncfg = LinkConfig {
        line_rate_bps: ncp_rate,
        ..LinkConfig::ncp()
    };
    let line_rate = ncp_rate;
    let mut s = Sim {
        // The hub cannot see its OS buffers; the NCP reports its ring.
        host: Side::new(hcfg, 0x1A2B_3C4D, Uart::new(baud, HOST_TX_BUFFER), false),
        ncp: Side::new(
            ncfg,
            0xC0FF_EE01,
            Uart::new(baud, NCP_TX_RING),
            line_rate != 0,
        ),
        now: 0,
        rng: Rng(seed),
        next_tid_host: 1,
        next_tid_ncp: 1,
    };
    s.ncp.link.open(0, ResetReason::PowerOn);
    s.host.link.open(0, ResetReason::HostOpen);
    s.run_until(200_000, &mut |_| false, &mut |_| false);
    assert_eq!(s.host.link.state(), LinkState::Established);
    assert_eq!(s.ncp.link.state(), LinkState::Established);
    s
}

/// Decides, by DATA frame index, whether the frame is lost on the wire.
type Loss<'a> = dyn FnMut(u32) -> bool + 'a;

impl Sim {
    fn step(&mut self, host_drop: &mut Loss<'_>, ncp_drop: &mut Loss<'_>) {
        let now = self.now;
        let now_ns = now * 1000;
        let to_ncp = self.host.uart.deliver(now_ns);
        self.ncp.feed(&to_ncp, now);
        let to_host = self.ncp.uart.deliver(now_ns);
        self.host.feed(&to_host, now);
        // The NCP answers each request with a status response and, like a
        // ZCL read, an indication carrying the attribute data.
        for _ in std::mem::take(&mut self.ncp.requests_in) {
            self.ncp_message(Kind::Response, 4);
            self.ncp_message(Kind::Event, 90);
        }
        self.host.pump(now, host_drop);
        self.ncp.pump(now, ncp_drop);
        self.now += STEP_US;
    }

    fn run_until(&mut self, t: u64, host_drop: &mut Loss<'_>, ncp_drop: &mut Loss<'_>) {
        while self.now < t {
            self.step(host_drop, ncp_drop);
        }
    }

    fn ncp_message(&mut self, kind: Kind, len: usize) {
        let tid = self.next_tid_ncp;
        self.next_tid_ncp += 1;
        self.ncp.push(kind, tid, len, self.now);
    }

    /// `n` ZCL read requests (APS data requests of 40 bytes).
    fn burst(&mut self, n: usize) {
        for _ in 0..n {
            let tid = self.next_tid_host;
            self.next_tid_host += 1;
            self.host.push(Kind::Request, tid, 40, self.now);
        }
    }

    /// Read bursts of 8–24 requests every 0.4–1 s over light background
    /// traffic (a single poll every 50–100 ms, a device report every
    /// 80–160 ms): the adaptive RTO falls to its minimum on the quick
    /// exchanges, then a burst queues responses behind each other.
    fn bursts(&mut self, seconds: u64, host_drop: &mut Loss<'_>, ncp_drop: &mut Loss<'_>) {
        let end = self.now + seconds * 1_000_000;
        let (mut burst, mut poll, mut report) = (self.now, self.now, self.now + 20_000);
        while self.now < end {
            if self.now >= burst {
                let n = 8 + self.rng.below(17) as usize;
                self.burst(n);
                burst += 400_000 + self.rng.below(600_000);
            }
            if self.now >= poll {
                self.burst(1);
                poll += 50_000 + self.rng.below(50_000);
            }
            if self.now >= report {
                self.ncp_message(Kind::Event, 50);
                report += 80_000 + self.rng.below(80_000);
            }
            self.step(host_drop, ncp_drop);
        }
        let t = self.now + 3_000_000;
        self.run_until(t, host_drop, ncp_drop);
    }

    fn outcome(&self) -> Outcome {
        let (h, n) = (self.host.link.counters(), self.ncp.link.counters());
        self.host.check_delivered_by(&self.ncp);
        self.ncp.check_delivered_by(&self.host);
        Outcome {
            rx_duplicates: h.rx_duplicate + n.rx_duplicate,
            retransmissions: h.tx_retransmissions + n.tx_retransmissions,
            rto_expiries: h.tx_rto_expiries + n.tx_rto_expiries,
            spurious_detected: h.tx_spurious_retx + n.tx_spurious_retx,
            host_max_latency_us: self.host.max_latency(&self.ncp),
            ncp_max_latency_us: self.ncp.max_latency(&self.host),
            ncp_max_tx_backlog: self.ncp.uart.max_backlog,
            host_max_tx_backlog: self.host.uart.max_backlog,
        }
    }
}

fn never(_: u32) -> bool {
    false
}

// ── Streaming at 115 200 without flow control ──────────────────────────

#[test]
fn without_line_rate_acks_queue_behind_data_and_frames_are_resent() {
    // Today's behaviour (the 35-minute BRD4187 soak): the NCP fills its
    // 2 KiB ring (~180 ms at 115 200), each queued frame's RTO runs from
    // when it entered the ring, and the RTO has fallen to 25 ms in the
    // quiet gaps.
    let mut s = sim(0);
    s.bursts(20, &mut never, &mut never);
    let o = s.outcome();
    let (h, n) = (s.host.link.counters(), s.ncp.link.counters());
    println!(
        "without line rate: {o:?} host retx {} ncp retx {}",
        h.tx_retransmissions, n.tx_retransmissions
    );
    assert!(o.retransmissions >= 50, "{o:?}");
    assert!(o.rx_duplicates >= 50, "{o:?}");
}

#[test]
fn with_line_rate_no_spurious_retransmissions() {
    let mut s = sim(BAUD);
    s.bursts(20, &mut never, &mut never);
    let o = s.outcome();
    println!("with line rate: {o:?}");
    assert_eq!(o.rx_duplicates, 0, "{o:?}");
    assert_eq!(o.retransmissions, 0, "{o:?}");
    // ACK-first: DATA joins the NCP ring only below the threshold.
    assert!(
        o.ncp_max_tx_backlog < PEER_ACK_AHEAD + ACK_WIRE_MAX,
        "{o:?}"
    );
    assert_eq!(s.ncp.link.rto_floor_us(), rto_floor_us(BAUD, 1000, 25_000));
}

#[test]
fn with_line_rate_no_spurious_retransmissions_other_rates_and_seeds() {
    for (baud, seed) in [(115_200, 1), (115_200, 2), (460_800, 3), (921_600, 4)] {
        let mut s = sim2(baud, baud, baud, seed);
        s.bursts(10, &mut never, &mut never);
        let o = s.outcome();
        println!("{baud} seed {seed}: {o:?}");
        assert_eq!(o.rx_duplicates, 0, "{baud}: {o:?}");
        assert_eq!(o.retransmissions, 0, "{baud}: {o:?}");
    }
}

#[test]
fn ncp_alone_with_line_rate_cuts_spurious_retransmissions() {
    // NCP firmware updated, host not yet (no line rate on its side).
    let mut s = sim2(BAUD, 0, BAUD, 0x5EED_F00D);
    s.bursts(20, &mut never, &mut never);
    let o = s.outcome();
    let (h, n) = (s.host.link.counters(), s.ncp.link.counters());
    println!(
        "NCP only: {o:?} host retx {} ncp retx {}",
        h.tx_retransmissions, n.tx_retransmissions
    );
    assert_eq!(n.tx_retransmissions, 0, "{o:?}");
    // The host still queues its frames in its OS buffer and times them
    // from there with the 25 ms floor: it must pass its line rate too.
    assert!(h.tx_retransmissions > 0, "{o:?}");
}

#[test]
fn with_line_rate_lost_frames_recover_promptly() {
    // Lose every 25th DATA frame in each direction (mid-burst losses are
    // repaired by FIFO inference or NAK, tail losses by the RTO).
    let mut s = sim(BAUD);
    let mut host_drop = |i: u32| i % 25 == 7;
    let mut ncp_drop = |i: u32| i % 25 == 11;
    s.bursts(20, &mut host_drop, &mut ncp_drop);
    let o = s.outcome();
    println!("lossy with line rate: {o:?}");
    println!(
        "data frames: host {} ncp {}",
        s.host.data_frames_out, s.ncp.data_frames_out
    );
    assert!(s.host.data_frames_out >= 50 && s.ncp.data_frames_out >= 50);
    let lost = s.host.data_frames_out / 25 + s.ncp.data_frames_out / 25;
    assert!(o.retransmissions >= lost, "{o:?}");
    // Every loss is repaired once: at most one duplicate per lost frame.
    assert!(o.rx_duplicates <= lost, "{o:?} lost {lost}");
    assert_eq!(o.spurious_detected, 0, "{o:?}");
    // Bounded: the worst message waited for its burst plus one recovery.
    assert!(o.host_max_latency_us < 400_000, "{o:?}");
    assert!(o.ncp_max_latency_us < 500_000, "{o:?}");
}

#[test]
fn with_line_rate_tail_loss_is_resent_after_the_floor() {
    let mut s = sim(BAUD);
    s.run_until(s.now + 1_000_000, &mut never, &mut never);
    let floor = s.host.link.rto_floor_us();
    assert_eq!(floor, rto_floor_us(BAUD, 1000, 25_000));
    // A lone request, lost: only the RTO can repair it.
    s.burst(1);
    let sent = s.now;
    let mut host_drop = |_: u32| true;
    s.step(&mut host_drop, &mut never);
    let deadline = sent + floor + 50_000;
    s.run_until(deadline, &mut never, &mut never);
    assert_eq!(s.ncp.delivered.len(), 1, "not recovered by {deadline}");
    let at = s.ncp.delivered[0].1;
    let frame_us = wire_time_us(40 + 7 + 8 + 4 + 3, BAUD);
    assert!(at >= sent + floor, "resent before the floor: {at}");
    assert!(
        at <= sent + frame_us + floor + 2 * frame_us + 2_000,
        "{at} vs {sent} + {floor}"
    );
    assert_eq!(s.host.link.counters().tx_rto_expiries, 1);
}

// ── Engine rules ───────────────────────────────────────────────────────

#[test]
fn rto_floor_numbers() {
    assert_eq!(ACK_WIRE_MAX, 31);
    assert_eq!(PEER_ACK_AHEAD, 256 + 657);
    // 657 bytes at 115 200 8N1: 57.03 ms.
    assert_eq!(wire_time_us(TX_BUFFER_LEN, 115_200), 57_032);
    assert_eq!(wire_time_us(1, 921_600), 11);
    assert_eq!(wire_time_us(1000, 0), 0);
    // 944 bytes at 115 200 (81.94 ms) + ack_delay 1 ms + RTO_min 25 ms.
    assert_eq!(rto_floor_us(115_200, 1000, 25_000), 81_945 + 1000 + 25_000);
    assert_eq!(rto_floor_us(460_800, 1000, 25_000), 20_487 + 1000 + 25_000);
    assert_eq!(rto_floor_us(921_600, 1000, 25_000), 10_244 + 1000 + 25_000);
    // Unknown line rate: the fixed RTO_min.
    assert_eq!(rto_floor_us(0, 1000, 25_000), 25_000);
}

fn rig(line_rate: u32) -> Rig<16, 16> {
    let cfg = LinkConfig {
        line_rate_bps: line_rate,
        ..LinkConfig::ncp()
    };
    Rig::established(cfg, 16, &[(1, 4), (2, 64), (3, 4), (4, 2), (5, 0)])
}

/// A source of `n` one-message frames of 200 bytes on ZIGBEE.
struct Big(u16);

impl TxSource for Big {
    fn ready(&self, endpoint: Endpoint) -> bool {
        endpoint == Endpoint::Zigbee && self.0 > 0
    }
    fn fill(&mut self, _: Endpoint, buf: &mut [u8]) -> usize {
        self.0 -= 1;
        let mut w = BatchWriter::new(buf);
        w.push_raw(Kind::Event, false, 0x0201, self.0, &[0x33; 200])
            .unwrap();
        w.len()
    }
}

#[test]
fn rto_floor_follows_peer_and_line_rate() {
    let mut r = rig(0);
    assert_eq!(r.link.rto_floor_us(), 25_000);
    assert_eq!(r.link.rto_us(), 100_000);
    r.link.set_line_rate(BAUD);
    assert_eq!(r.link.line_rate(), BAUD);
    assert_eq!(r.link.rto_floor_us(), rto_floor_us(BAUD, 1000, 25_000));
    assert_eq!(r.link.rto_us(), r.link.rto_floor_us());
    r.link.set_line_rate(921_600);
    assert_eq!(r.link.rto_floor_us(), rto_floor_us(921_600, 1000, 25_000));
    // Established at 115 200: RTO_initial (100 ms) is raised to the floor.
    let r = rig(BAUD);
    assert_eq!(r.link.rto_us(), rto_floor_us(BAUD, 1000, 25_000));
}

#[test]
fn data_is_held_back_but_acks_are_not() {
    let mut r = rig(BAUD);
    r.now = 200_000;
    let mut src = Big(40);
    r.drain_tx(&mut src);
    // DATA stops once the modelled backlog reaches ACK_FIRST_BACKLOG: two
    // 222-byte frames.
    assert_eq!(r.sent.len(), 2);
    let backlog = r.link.tx_backlog(r.now);
    assert!(backlog >= ACK_FIRST_BACKLOG, "{backlog}");
    // The link sleeps until the backlog has drained below the threshold
    // instead of spinning.
    let gate = r.link.next_deadline().unwrap();
    let expect = wire_time_us(backlog + 1 - ACK_FIRST_BACKLOG, BAUD);
    assert_eq!(gate, r.now + expect);
    // A POLL from the peer is answered at once, ahead of the queued DATA.
    r.now += 1_000;
    let mut h = r.data_header(0, Endpoint::Zigbee as u8, 0);
    h.poll = true;
    r.feed(&wire(&h, &tagged_payload(1)));
    let before = r.sent.len();
    r.drain_tx(&mut src);
    assert_eq!(r.sent.len(), before + 1);
    assert_eq!(r.sent[before].header.ftype, FrameType::Ack);
    // DATA resumes when the backlog has drained.
    r.advance(gate + 10_000, &mut src);
    assert!(r.sent.len() > before + 1);
    assert_eq!(r.sent.last().unwrap().header.ftype, FrameType::Data);
}

#[test]
fn reported_backlog_holds_data_without_a_line_rate() {
    let mut r = rig(0);
    let mut src = OnePerFrame::default();
    src.push(Endpoint::Zigbee, 1);
    let mut out = [0u8; TX_BUFFER_LEN];
    assert!(
        r.link
            .poll_transmit_with_backlog(0, &mut src, &mut out, ACK_FIRST_BACKLOG)
            .is_none()
    );
    // Unknown drain rate: look again in a millisecond.
    assert_eq!(r.link.next_deadline(), Some(1_000));
    assert!(
        r.link
            .poll_transmit_with_backlog(0, &mut src, &mut out, ACK_FIRST_BACKLOG - 1)
            .is_some()
    );
    assert!(r.link.poll_transmit(0, &mut NoSource, &mut out).is_none());
}

#[test]
fn rto_runs_from_when_the_frame_left_the_line() {
    let mut r = rig(BAUD);
    let mut src = OnePerFrame::default();
    src.push(Endpoint::Zigbee, 1);
    let mut out = [0u8; TX_BUFFER_LEN];
    // 200 bytes of earlier output still wait in the transport.
    let n = r
        .link
        .poll_transmit_with_backlog(0, &mut src, &mut out, 200)
        .unwrap();
    let left = wire_time_us(200, BAUD) + wire_time_us(n, BAUD);
    let floor = r.link.rto_floor_us();
    assert_eq!(r.link.next_deadline(), Some(left + floor));
    // Not before.
    assert!(
        r.link
            .poll_transmit(left + floor - 1, &mut NoSource, &mut out)
            .is_none()
    );
    let n2 = r
        .link
        .poll_transmit(left + floor, &mut NoSource, &mut out)
        .unwrap();
    let (h, _) = &decode_wire(&out[..n2])[0];
    assert!(h.retx && h.poll);
    // Backoff doubles the effective RTO.
    assert_eq!(r.link.rto_us(), 2 * floor);
}

#[test]
fn backoff_never_drops_below_the_floor_when_rto_max_is_lower() {
    // RTO_max (50 ms) below the 115 200 floor (~108 ms): the cap of the
    // backoff is max(RTO_max, floor), as for every other RTO update.
    let cfg = LinkConfig {
        line_rate_bps: BAUD,
        rto_max_us: 50_000,
        ..LinkConfig::ncp()
    };
    let mut r: Rig<16, 16> = Rig::established(cfg, 16, &[(1, 4), (2, 64), (3, 4), (4, 2), (5, 0)]);
    let floor = r.link.rto_floor_us();
    assert!(floor > 50_000);
    assert_eq!(r.link.rto_us(), floor);
    let mut src = OnePerFrame::default();
    src.push(Endpoint::Zigbee, 1);
    let mut out = [0u8; TX_BUFFER_LEN];
    r.link.poll_transmit(0, &mut src, &mut out).unwrap();
    let mut due = r.link.next_deadline().unwrap();
    for expiry in 1..=3 {
        r.link.poll_transmit(due, &mut NoSource, &mut out).unwrap();
        assert_eq!(r.link.counters().tx_rto_expiries, expiry);
        assert_eq!(r.link.rto_us(), floor, "expiry {expiry}");
        let next = r.link.next_deadline().unwrap();
        assert!(
            next >= due + floor,
            "expiry {expiry}: {next} < {due} + {floor}"
        );
        due = next;
    }
}

#[test]
fn ack_of_an_earlier_copy_counts_a_spurious_retransmission() {
    let mut r = rig(BAUD);
    let mut src = OnePerFrame::default();
    src.push(Endpoint::Zigbee, 1);
    let mut out = [0u8; TX_BUFFER_LEN];
    r.link.poll_transmit(0, &mut src, &mut out).unwrap();
    let rto = r.link.next_deadline().unwrap();
    // Retransmitted behind 200 bytes the transport still holds...
    r.link
        .poll_transmit_with_backlog(rto, &mut NoSource, &mut out, 200)
        .unwrap();
    // ...and the ACK of the first copy arrives meanwhile.
    r.now = rto + 1_000;
    let ack = r.ack_frame(FrameType::Ack, 1, 0, 0, &[]);
    r.feed(&ack);
    assert_eq!(r.link.counters().tx_spurious_retx, 1);
    assert_eq!(r.link.counters().tx_retransmissions, 1);
}
