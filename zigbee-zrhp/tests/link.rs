//! Link-layer conformance and property tests (spec 14.2): two real `Link`s
//! connected by a simulated byte stream with fault injection and a virtual
//! clock.

mod common;

use std::collections::{BTreeMap, VecDeque};

use common::Rng;
use common::harness::{
    OnePerFrame, Rig, decode_wire, payload_tag, reset_payload, tagged_payload, wire,
};
use zigbee_zrhp::Endpoint;
use zigbee_zrhp::frame::{self, FrameType, Header, RejectReason, ResetReason, Role, TX_BUFFER_LEN};
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent, LinkState, PeerChange, TxFailure};
use zigbee_zrhp::msg::Batch;
use zigbee_zrhp::queue::{NoSource, TxSource};

const HOST_COOKIE: u32 = 0x1A2B_3C4D;
const NCP_COOKIE: u32 = 0xC0FF_EE01;

/// Per-direction fault model (probabilities per million).
#[derive(Debug, Clone, Copy, Default)]
struct Faults {
    flip_per_byte: u64,
    drop_per_byte: u64,
    insert_zero_per_byte: u64,
    drop_frame: u64,
    dup_frame: u64,
    swap_frame: u64,
}

/// One side of the simulation: a link, its outgoing source and what it
/// received / was told.
struct Side<const T: usize, const R: usize> {
    link: Link<T, R>,
    src: OnePerFrame,
    delivered: BTreeMap<u8, Vec<u16>>,
    failed: Vec<(u16, TxFailure)>,
    events: Vec<String>,
}

impl<const T: usize, const R: usize> Side<T, R> {
    fn new(cfg: LinkConfig, cookie: u32) -> Self {
        Self {
            link: Link::new(cfg, cookie),
            src: OnePerFrame::default(),
            delivered: BTreeMap::new(),
            failed: Vec::new(),
            events: Vec::new(),
        }
    }

    fn feed(&mut self, bytes: &[u8], now: u64) {
        let mut off = 0;
        while off < bytes.len() {
            off += self.link.push_rx(&bytes[off..], now);
            self.drain_events();
        }
    }

    fn drain_events(&mut self) {
        while let Some(ev) = self.link.poll_event() {
            match ev {
                LinkEvent::Data { endpoint, payload } => {
                    let batch = Batch::parse(payload).expect("delivered payload is a valid batch");
                    for m in batch.iter() {
                        self.delivered
                            .entry(endpoint as u8)
                            .or_default()
                            .push(m.header.tid);
                    }
                }
                LinkEvent::TxFailed {
                    payload, reason, ..
                } => {
                    for m in Batch::parse(payload).unwrap().iter() {
                        self.failed.push((m.header.tid, reason));
                    }
                }
                other => self.events.push(format!("{other:?}")),
            }
        }
    }

    fn all_delivered(&self) -> Vec<u16> {
        let mut v: Vec<u16> = self.delivered.values().flatten().copied().collect();
        v.sort_unstable();
        v
    }
}

/// A source that batches as many tagged messages as fit (exercises batching).
#[derive(Debug, Default)]
struct Batching {
    queue: VecDeque<(Endpoint, u16)>,
}

impl TxSource for Batching {
    fn ready(&self, endpoint: Endpoint) -> bool {
        self.queue.iter().any(|(e, _)| *e == endpoint)
    }
    fn fill(&mut self, endpoint: Endpoint, buf: &mut [u8]) -> usize {
        let mut n = 0;
        while let Some(i) = self.queue.iter().position(|(e, _)| *e == endpoint) {
            let p = tagged_payload(self.queue[i].1);
            if n + p.len() > buf.len() {
                break;
            }
            buf[n..n + p.len()].copy_from_slice(&p);
            n += p.len();
            self.queue.remove(i);
        }
        n
    }
}

struct Channel {
    queue: VecDeque<(u64, Vec<u8>)>,
    faults: Faults,
    latency: u64,
    blackhole: bool,
}

impl Channel {
    fn new(faults: Faults) -> Self {
        Self {
            queue: VecDeque::new(),
            faults,
            latency: 300,
            blackhole: false,
        }
    }

    fn send(&mut self, now: u64, chunk: &[u8], rng: &mut Rng) {
        if self.blackhole {
            return;
        }
        let f = self.faults;
        if rng.chance(f.drop_frame) {
            return;
        }
        let mut out = Vec::with_capacity(chunk.len() + 4);
        for &b in chunk {
            if rng.chance(f.drop_per_byte) {
                continue;
            }
            if rng.chance(f.insert_zero_per_byte) {
                out.push(0);
            }
            let mut b = b;
            if rng.chance(f.flip_per_byte) {
                b ^= 1 << rng.below(8);
            }
            out.push(b);
        }
        // Byte time at ~1 Mbaud: 10 µs per byte.
        let at = now + self.latency + out.len() as u64 * 10;
        let at = self.queue.back().map_or(at, |(t, _)| at.max(*t));
        if rng.chance(f.dup_frame) {
            self.queue.push_back((at, out.clone()));
        }
        self.queue.push_back((at, out));
        if rng.chance(f.swap_frame) && self.queue.len() >= 2 {
            let n = self.queue.len();
            self.queue.swap(n - 1, n - 2);
            let t = self.queue[n - 1].0.max(self.queue[n - 2].0);
            self.queue[n - 1].0 = t;
            self.queue[n - 2].0 = t;
        }
    }

    fn due(&mut self, now: u64) -> Option<Vec<u8>> {
        if self.queue.front().is_some_and(|(t, _)| *t <= now) {
            self.queue.pop_front().map(|(_, b)| b)
        } else {
            None
        }
    }
}

struct Sim<const HT: usize, const HR: usize, const NT: usize, const NR: usize> {
    host: Side<HT, HR>,
    ncp: Side<NT, NR>,
    h2n: Channel,
    n2h: Channel,
    now: u64,
    rng: Rng,
}

impl<const HT: usize, const HR: usize, const NT: usize, const NR: usize> Sim<HT, HR, NT, NR> {
    fn new(hcfg: LinkConfig, ncfg: LinkConfig, faults: Faults, seed: u64) -> Self {
        let mut s = Self {
            host: Side::new(hcfg, HOST_COOKIE),
            ncp: Side::new(ncfg, NCP_COOKIE),
            h2n: Channel::new(faults),
            n2h: Channel::new(faults),
            now: 0,
            rng: Rng(seed | 1),
        };
        s.ncp.link.open(0, ResetReason::PowerOn);
        s.host.link.open(0, ResetReason::HostOpen);
        s
    }

    fn step(&mut self, dt: u64) {
        let now = self.now;
        while let Some(b) = self.h2n.due(now) {
            self.ncp.feed(&b, now);
        }
        while let Some(b) = self.n2h.due(now) {
            self.host.feed(&b, now);
        }
        let mut out = [0u8; TX_BUFFER_LEN];
        let mut guard = 0;
        while let Some(n) = self
            .host
            .link
            .poll_transmit(now, &mut self.host.src, &mut out)
        {
            self.h2n.send(now, &out[..n], &mut self.rng);
            guard += 1;
            assert!(guard < 1000);
        }
        self.host.drain_events();
        while let Some(n) = self
            .ncp
            .link
            .poll_transmit(now, &mut self.ncp.src, &mut out)
        {
            self.n2h.send(now, &out[..n], &mut self.rng);
            guard += 1;
            assert!(guard < 2000);
        }
        self.ncp.drain_events();
        self.now += dt;
    }

    fn run_until(&mut self, t: u64, dt: u64) {
        while self.now < t {
            self.step(dt);
        }
    }

    fn established(&self) -> bool {
        self.host.link.state() == LinkState::Established
            && self.ncp.link.state() == LinkState::Established
    }

    fn settle(&mut self, max: u64) {
        let end = self.now + max;
        while self.now < end {
            self.step(100);
            let idle = self.h2n.queue.is_empty()
                && self.n2h.queue.is_empty()
                && self.host.src.queue.is_empty()
                && self.ncp.src.queue.is_empty()
                && self.host.link.tx_in_flight().count() == 0
                && self.ncp.link.tx_in_flight().count() == 0;
            if idle && self.established() {
                return;
            }
        }
    }
}

type Desktop = Sim<32, 32, 16, 16>;
type Embedded = Sim<8, 8, 16, 16>;
type Minimal = Sim<4, 4, 16, 16>;

fn quiet() -> Faults {
    Faults::default()
}

/// Every message is delivered exactly once in per-endpoint order, or
/// reported as failed. Returns (delivered, failed) counts.
fn check_accounting<const T: usize, const R: usize, const T2: usize, const R2: usize>(
    sender: &Side<T, R>,
    receiver: &Side<T2, R2>,
    sent: &BTreeMap<u8, Vec<u16>>,
) -> (usize, usize) {
    let mut delivered = 0;
    for (ep, tags) in sent {
        let got = receiver.delivered.get(ep).cloned().unwrap_or_default();
        // Strictly increasing ⇒ in order and no duplicates.
        assert!(
            got.windows(2).all(|w| w[0] < w[1]),
            "ep {ep}: out of order or duplicate: {got:?}"
        );
        for t in &got {
            assert!(tags.contains(t), "ep {ep}: unknown tag {t}");
        }
        delivered += got.len();
        for t in tags {
            let ok = got.contains(t)
                || sender.failed.iter().any(|(f, _)| f == t)
                || sender.src.queue.iter().any(|(_, q)| q == t);
            assert!(ok, "ep {ep}: tag {t} lost silently");
        }
    }
    (delivered, sender.failed.len())
}

// ── Handshake ──────────────────────────────────────────────────────────

#[test]
fn handshake_both_open_epoch_33() {
    let mut s = Desktop::new(LinkConfig::desktop_host(), LinkConfig::ncp(), quiet(), 1);
    s.run_until(20_000, 100);
    assert!(s.established());
    assert_eq!(s.host.link.session().unwrap().epoch, 33);
    assert_eq!(s.ncp.link.session().unwrap().epoch, 33);
    assert_eq!(s.host.link.session().unwrap().tx_window, 16);
    assert_eq!(s.ncp.link.session().unwrap().tx_window, 16); // min(32, NCP TXW 16)
    assert!(s.host.events.iter().any(|e| e.starts_with("Established")));
    assert!(s.ncp.events.iter().any(|e| e.starts_with("Established")));
    assert_eq!(s.host.link.counters().sessions_established, 1);
    assert_eq!(s.ncp.link.counters().sessions_established, 1);
}

#[test]
fn handshake_host_alone_then_ncp_reply() {
    // NCP never calls open(): it answers the host's RESET_REQ.
    let mut host: Rig<8, 8> = Rig {
        link: Link::new(LinkConfig::embedded_host(), HOST_COOKIE),
        now: 0,
        sent: vec![],
        delivered: vec![],
        events: vec![],
        epoch: 0,
        peer_role: Role::Ncp,
    };
    let mut ncp = Link::<16, 16>::new(LinkConfig::ncp(), NCP_COOKIE);
    host.link.open(0, ResetReason::HostOpen);
    let mut buf = [0u8; TX_BUFFER_LEN];
    for round in 0..4 {
        let mut bytes = Vec::new();
        while let Some(n) = host.link.poll_transmit(0, &mut NoSource, &mut buf) {
            if round == 0 && bytes.is_empty() {
                assert_eq!(buf[0], 0, "first frame after open starts with 0x00");
            }
            bytes.extend_from_slice(&buf[..n]);
        }
        let mut off = 0;
        while off < bytes.len() {
            off += ncp.push_rx(&bytes[off..], 0);
            while ncp.poll_event().is_some() {}
        }
        while let Some(n) = ncp.poll_transmit(0, &mut NoSource, &mut buf) {
            host.feed(&buf[..n]);
        }
    }
    assert_eq!(host.link.state(), LinkState::Established);
    assert_eq!(ncp.state(), LinkState::Established);
}

#[test]
fn simultaneous_reset_req() {
    // Both sides open at the same instant with zero latency: each answers the
    // other's RESET_REQ and both establish on the matching RESET_ACK.
    let mut s = Embedded::new(LinkConfig::embedded_host(), LinkConfig::ncp(), quiet(), 3);
    s.h2n.latency = 0;
    s.n2h.latency = 0;
    s.run_until(5_000, 50);
    assert!(s.established());
    assert_eq!(
        s.host.link.session().unwrap().epoch,
        s.ncp.link.session().unwrap().epoch
    );
}

#[test]
fn lost_reset_ack_is_repaired() {
    // Host establishes on the NCP's RESET_ACK; the host's answering RESET_ACK
    // is lost. The NCP retries its RESET_ACK; the established host answers
    // the duplicate once more and both converge.
    let mut s = Embedded::new(LinkConfig::embedded_host(), LinkConfig::ncp(), quiet(), 5);
    // Phase 1: let the NCP's boot RESET_REQ and the host's RESET_REQ cross.
    s.step(100);
    s.run_until(400, 100);
    // Drop everything host → NCP for a while.
    s.h2n.blackhole = true;
    s.run_until(5_000, 100);
    assert_eq!(s.host.link.state(), LinkState::Established);
    assert_eq!(s.ncp.link.state(), LinkState::Resetting);
    s.h2n.blackhole = false;
    s.run_until(800_000, 200);
    assert!(
        s.established(),
        "host {:?} ncp {:?}",
        s.host.link.state(),
        s.ncp.link.state()
    );
}

// ── Basic transfer, batching, wrap ─────────────────────────────────────

#[test]
fn bulk_transfer_both_directions_with_seq_wrap() {
    let mut s = Desktop::new(LinkConfig::desktop_host(), LinkConfig::ncp(), quiet(), 7);
    s.run_until(5_000, 100);
    let mut sent_h: BTreeMap<u8, Vec<u16>> = BTreeMap::new();
    let mut sent_n: BTreeMap<u8, Vec<u16>> = BTreeMap::new();
    let mut tag = 1u16;
    for i in 0..700u16 {
        let ep = [
            Endpoint::System,
            Endpoint::Zigbee,
            Endpoint::Tables,
            Endpoint::Bulk,
        ][i as usize % 4];
        s.host.src.push(ep, tag);
        sent_h.entry(ep as u8).or_default().push(tag);
        tag += 1;
        let ep = [
            Endpoint::Zigbee,
            Endpoint::Zigbee,
            Endpoint::System,
            Endpoint::Log,
        ][i as usize % 4];
        s.ncp.src.push(ep, tag);
        sent_n.entry(ep as u8).or_default().push(tag);
        tag += 1;
    }
    s.settle(20_000_000);
    let (d1, f1) = check_accounting(&s.host, &s.ncp, &sent_h);
    let (d2, f2) = check_accounting(&s.ncp, &s.host, &sent_n);
    assert_eq!((d1, f1, d2, f2), (700, 0, 700, 0));
    // 700 frames per direction: link seq and ep_seq both wrapped.
    assert!(s.host.link.counters().tx_frames > 700);
    assert_eq!(s.host.link.counters().tx_retransmissions, 0);
    assert_eq!(s.ncp.link.counters().rx_duplicate, 0);
}

#[test]
fn batching_packs_queued_messages() {
    let mut s = Embedded::new(LinkConfig::embedded_host(), LinkConfig::ncp(), quiet(), 9);
    s.run_until(5_000, 100);
    let before = s.ncp.link.counters().tx_frames;
    let mut src = Batching::default();
    for t in 1..=300u16 {
        src.queue.push_back((Endpoint::Zigbee, t));
    }
    let mut out = [0u8; TX_BUFFER_LEN];
    let now = s.now;
    let mut frames = 0;
    while let Some(n) = s.ncp.link.poll_transmit(now, &mut src, &mut out) {
        s.n2h.send(now, &out[..n], &mut s.rng);
        frames += 1;
    }
    // 7-byte messages, 91 per 640-byte frame: 300 messages need 4 frames.
    assert_eq!(frames, 4);
    assert!(s.ncp.link.counters().tx_frames - before == 4);
    s.settle(1_000_000);
    assert_eq!(s.host.all_delivered(), (1..=300).collect::<Vec<_>>());
}

// ── Loss, duplicates, reordering, CRC errors ───────────────────────────

#[test]
fn mid_stream_loss_recovered_by_fifo_inference() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::embedded_host(), 16, &[(2, 8)]);
    let mut src = OnePerFrame::default();
    for t in 0..4 {
        src.push(Endpoint::Zigbee, t);
    }
    rig.drain_tx(&mut src);
    assert_eq!(rig.sent.len(), 4);
    // Peer got 0, lost 1, got 2 and 3: NAK ack=1 sack=0b11.
    rig.now = 1_000;
    let nak = rig.ack_frame(FrameType::Nak, 1, 0b11, 0, &[]);
    rig.feed(&nak);
    rig.drain_tx(&mut src);
    let retx: Vec<_> = rig.sent.iter().filter(|s| s.header.retx).collect();
    assert_eq!(retx.len(), 1);
    assert_eq!(retx[0].header.seq, 1);
    assert!(!retx[0].header.poll);
    // An identical NAK must not retransmit again.
    let n = rig.sent.len();
    rig.feed(&nak);
    rig.drain_tx(&mut src);
    assert_eq!(rig.sent.len(), n);
}

#[test]
fn duplicate_frames_are_never_delivered_twice() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::ncp(), 8, &[(2, 8)]);
    for seq in 0..3u8 {
        let w = wire(&rig.data_header(seq, 2, seq), &tagged_payload(seq as u16));
        rig.feed(&w);
        rig.feed(&w);
    }
    // Duplicate far behind the window.
    let old = wire(&rig.data_header(0, 2, 0), &tagged_payload(0));
    rig.feed(&old);
    let tags: Vec<u16> = rig.delivered.iter().map(|(_, p)| payload_tag(p)).collect();
    assert_eq!(tags, [0, 1, 2]);
    assert_eq!(rig.link.counters().rx_duplicate, 4);
}

#[test]
fn crc_error_naks_with_nbad_and_rate_limit() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::ncp(), 8, &[(2, 8)]);
    let mut bad = wire(&rig.data_header(0, 2, 0), &tagged_payload(0));
    let n = bad.len();
    bad[n - 2] ^= 0x20;
    rig.feed(&bad);
    rig.drain_tx(&mut NoSource);
    rig.feed(&bad);
    rig.drain_tx(&mut NoSource);
    rig.feed(&bad);
    rig.drain_tx(&mut NoSource);
    // Only one NAK inside the hold-off (ack_delay = 1 ms) …
    let naks: Vec<_> = rig
        .sent
        .iter()
        .filter(|s| s.header.ftype == FrameType::Nak)
        .collect();
    assert_eq!(naks.len(), 1);
    assert_eq!(naks[0].ack_body().1, 1);
    // … and one more with the updated nbad when it ends.
    rig.advance(1_000, &mut NoSource);
    let naks: Vec<_> = rig
        .sent
        .iter()
        .filter(|s| s.header.ftype == FrameType::Nak)
        .collect();
    assert_eq!(naks.len(), 2);
    assert_eq!(naks[1].ack_body().1, 3);
    assert_eq!(naks[1].t, 1_000);
    assert_eq!(rig.link.counters().rx_bad_frame, 3);
}

#[test]
fn oversize_and_garbage_input_is_bounded_and_resyncs() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::ncp(), 8, &[(2, 8)]);
    // 5000 non-zero bytes without a delimiter: one bad frame, bounded buffer.
    rig.feed(&[0x55; 5000]);
    assert_eq!(rig.link.counters().rx_bad_frame, 1);
    // The next delimiter resynchronises; a valid frame follows.
    rig.feed(&[0x00]);
    let w = wire(&rig.data_header(0, 2, 0), &tagged_payload(42));
    rig.feed(&w);
    assert_eq!(rig.delivered.len(), 1);
    // A frame whose decoded length disagrees with `len`.
    let mut h = rig.data_header(1, 2, 1);
    h.seq = 1;
    let mut dec = [0u8; 64];
    let n = frame::encode_decoded(&h, &[&tagged_payload(1)], &mut dec).unwrap();
    dec[6] = dec[6].wrapping_add(1); // len field off by one
    let mut enc = [0u8; 80];
    let m = zigbee_zrhp::cobs::encode(&dec[..n], &mut enc).unwrap();
    let mut wirebytes = enc[..m].to_vec();
    wirebytes.push(0);
    rig.feed(&wirebytes);
    assert_eq!(rig.link.counters().rx_bad_frame, 2);
    assert_eq!(rig.delivered.len(), 1);
}

#[test]
fn tail_loss_rto_then_retry_exhausted_resets() {
    let cfg = LinkConfig {
        adaptive_rto: false,
        n_retx_max: 3,
        ..LinkConfig::embedded_host()
    };
    let mut rig: Rig<8, 8> = Rig::established(cfg, 8, &[(2, 8)]);
    let mut src = OnePerFrame::default();
    src.push(Endpoint::Zigbee, 1);
    rig.drain_tx(&mut src);
    // No answer at all: 100, 200, 400 ms RTO retransmissions with POLL.
    rig.advance(1_000_000, &mut src);
    let retx: Vec<_> = rig.sent.iter().filter(|s| s.header.retx).collect();
    assert_eq!(
        retx.iter().map(|s| s.t).collect::<Vec<_>>(),
        [100_000, 300_000, 700_000]
    );
    assert!(retx.iter().all(|s| s.header.poll));
    assert_eq!(rig.link.rto_us(), 800_000);
    // The 4th retransmission attempt fails the link: LINK_FAILED, a reset with
    // reason 3, and the message reported with LINK_RESET.
    rig.advance(1_600_000, &mut src);
    assert!(
        rig.events.iter().any(|e| e == "LinkFailed"),
        "{:?}",
        rig.events
    );
    assert!(
        rig.events
            .iter()
            .any(|e| e.contains("TxFailed") || e.contains("LinkReset"))
    );
    assert_eq!(rig.link.state(), LinkState::Resetting);
    let req = rig
        .sent
        .iter()
        .rfind(|s| s.header.ftype == FrameType::ResetReq)
        .unwrap();
    assert_eq!(req.reset().reason, ResetReason::RetryExhausted as u8);
}

#[test]
fn property_random_faults_exactly_once_in_order() {
    let faults = Faults {
        flip_per_byte: 300,
        drop_per_byte: 100,
        insert_zero_per_byte: 50,
        drop_frame: 5_000,
        dup_frame: 5_000,
        swap_frame: 2_000,
    };
    let mut total_retx = 0;
    for seed in 1..=12u64 {
        let mut s = Embedded::new(
            LinkConfig::embedded_host(),
            LinkConfig::ncp(),
            faults,
            seed * 7919,
        );
        let mut sent_h: BTreeMap<u8, Vec<u16>> = BTreeMap::new();
        let mut sent_n: BTreeMap<u8, Vec<u16>> = BTreeMap::new();
        let mut tag = 1u16;
        let mut rng = Rng(seed);
        // Traffic arrives over 2 s while faults happen.
        while s.now < 2_000_000 {
            if rng.chance(300_000) {
                let ep = Endpoint::DATA[rng.below(4) as usize];
                s.host.src.push(ep, tag);
                sent_h.entry(ep as u8).or_default().push(tag);
                tag += 1;
            }
            if rng.chance(400_000) {
                let ep = Endpoint::DATA[rng.below(4) as usize];
                s.ncp.src.push(ep, tag);
                sent_n.entry(ep as u8).or_default().push(tag);
                tag += 1;
            }
            s.step(200);
        }
        // Stop the noise and let everything drain.
        s.h2n.faults = Faults::default();
        s.n2h.faults = Faults::default();
        s.settle(30_000_000);
        let (d1, f1) = check_accounting(&s.host, &s.ncp, &sent_h);
        let (d2, f2) = check_accounting(&s.ncp, &s.host, &sent_n);
        let total = sent_h.values().map(Vec::len).sum::<usize>()
            + sent_n.values().map(Vec::len).sum::<usize>();
        assert!(d1 + d2 + f1 + f2 >= total, "seed {seed}");
        assert!(
            d1 + d2 >= total * 9 / 10,
            "seed {seed}: only {} of {total} delivered",
            d1 + d2
        );
        // With no resets nothing may fail.
        if s.host.link.counters().sessions_established == 1
            && s.ncp.link.counters().sessions_established == 1
        {
            assert_eq!((f1, f2), (0, 0), "seed {seed}");
        }
        assert!(
            s.ncp.link.counters().rx_bad_frame > 0,
            "seed {seed}: noise must have hit"
        );
        total_retx +=
            s.host.link.counters().tx_retransmissions + s.ncp.link.counters().tx_retransmissions;
    }
    assert!(total_retx > 0);
}

// ── Window, credits, HOL, scheduler ────────────────────────────────────

#[test]
fn window_full_blocks_then_poll_after_keepalive() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::embedded_host(), 4, &[(2, 100)]);
    let mut src = OnePerFrame::default();
    for t in 0..10 {
        src.push(Endpoint::Zigbee, t);
    }
    rig.drain_tx(&mut src);
    // Peer window 4: exactly 4 frames, then blocked.
    assert_eq!(
        rig.sent
            .iter()
            .filter(|s| s.header.ftype == FrameType::Data)
            .count(),
        4
    );
    assert!(!rig.link.can_send(Endpoint::Zigbee));
    // Acknowledge 2 → 2 more go out.
    rig.now = 10_000;
    let ack = rig.ack_frame(FrameType::Ack, 2, 0, 0, &[]);
    rig.feed(&ack);
    rig.drain_tx(&mut src);
    assert_eq!(
        rig.sent
            .iter()
            .filter(|s| s.header.ftype == FrameType::Data && !s.header.retx)
            .count(),
        6
    );
}

#[test]
fn credits_block_sender_and_keepalive_repairs_lost_update() {
    // Peer grants BULK 2 frames only.
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::embedded_host(), 16, &[(4, 2)]);
    let mut src = OnePerFrame::default();
    for t in 0..5 {
        src.push(Endpoint::Bulk, t);
    }
    rig.drain_tx(&mut src);
    let data = |r: &Rig<8, 8>| {
        r.sent
            .iter()
            .filter(|s| s.header.ftype == FrameType::Data && !s.header.retx)
            .count()
    };
    // BULK quota max(1, W/2) = 8 and credit 2: two frames.
    assert_eq!(data(&rig), 2);
    // The peer acks both but its credit update is "lost": no credit entries.
    rig.now = 2_000;
    let ack = rig.ack_frame(FrameType::Ack, 2, 0, 0, &[]);
    rig.feed(&ack);
    rig.drain_tx(&mut src);
    assert_eq!(data(&rig), 2);
    // S5: blocked for keepalive_ms → ACK|POLL.
    rig.advance(600_000, &mut src);
    assert!(
        rig.sent
            .iter()
            .any(|s| s.header.ftype == FrameType::Ack && s.header.poll)
    );
    // The peer's keepalive ACK carries all edges: BULK edge 5 → 3 more frames.
    let ka = rig.ack_frame(
        FrameType::Ack,
        2,
        0,
        0,
        &[(1, 4), (2, 8), (3, 4), (4, 5), (5, 0)],
    );
    rig.feed(&ka);
    rig.drain_tx(&mut src);
    assert_eq!(data(&rig), 5);
}

#[test]
fn receiver_rejects_frames_beyond_credit() {
    let mut cfg = LinkConfig {
        auto_credit: false,
        ..LinkConfig::ncp()
    };
    cfg.initial_credits[Endpoint::Bulk as usize] = 1;
    let mut rig: Rig<8, 8> = Rig::established(cfg, 8, &[(4, 4)]);
    rig.feed(&wire(&rig.data_header(0, 4, 0), &tagged_payload(0)));
    rig.feed(&wire(&rig.data_header(1, 4, 1), &tagged_payload(1)));
    rig.drain_tx(&mut NoSource);
    let rej: Vec<_> = rig
        .sent
        .iter()
        .filter(|s| s.header.ftype == FrameType::Reject)
        .collect();
    assert_eq!(rej.len(), 1);
    assert_eq!(rej[0].reject().reason, RejectReason::NoCredit as u8);
    assert_eq!((rej[0].reject().seq, rej[0].reject().ep_seq), (1, 1));
    // The rejected frame consumed its seq: ack is 2.
    assert_eq!(rej[0].header.ack, 2);
    assert_eq!(rig.link.rx_ack(), 2);
    assert_eq!(rig.delivered.len(), 1);
    // The rejected frame costs no credit: after one consumer release the
    // next ACK advertises BULK edge 3 (1 initial + 1 returned by the
    // rejection + 1 released) and ep_seq 2 is accepted.
    rig.link.release_credit(Endpoint::Bulk, 1);
    rig.advance(2_000, &mut NoSource); // delayed ACK
    let ack = rig
        .sent
        .iter()
        .rfind(|s| s.header.ftype == FrameType::Ack)
        .unwrap();
    assert_eq!(ack.ack_body().2, [(4, 3)]);
    rig.feed(&wire(&rig.data_header(2, 4, 2), &tagged_payload(2)));
    assert_eq!(rig.delivered.len(), 2);
    // Unknown endpoint 9 is rejected too and also consumes its seq.
    rig.feed(&wire(&rig.data_header(3, 9, 0), &tagged_payload(3)));
    rig.drain_tx(&mut NoSource);
    let last = rig
        .sent
        .iter()
        .rfind(|s| s.header.ftype == FrameType::Reject)
        .unwrap();
    assert_eq!(last.reject().reason, RejectReason::UnknownEndpoint as u8);
    assert_eq!(rig.link.rx_ack(), 4);
}

#[test]
fn granting_an_endpoint_its_first_credit_is_sent_at_once() {
    // An embedded host grants LOG no credit at session start, so the NCP
    // is blocked there from the start; a later grant (R8) goes out at
    // once, not with the next ACK of other traffic.
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::embedded_host(), 16, &[(2, 8)]);
    rig.drain_tx(&mut NoSource);
    let sent = rig.sent.len();
    rig.link.release_credit(Endpoint::Log, 4);
    rig.drain_tx(&mut NoSource);
    let ack = rig.sent[sent..]
        .iter()
        .find(|s| s.header.ftype == FrameType::Ack)
        .expect("immediate ACK");
    assert_eq!(ack.ack_body().2, [(5, 4)]);
    // An endpoint that still has credit is not blocked: its edge rides on
    // the next ACK.
    let sent = rig.sent.len();
    rig.link.release_credit(Endpoint::Bulk, 1);
    rig.drain_tx(&mut NoSource);
    assert_eq!(rig.sent.len(), sent);
}

#[test]
fn sender_reports_rejected_frames() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::embedded_host(), 8, &[(2, 8)]);
    let mut src = OnePerFrame::default();
    src.push(Endpoint::Zigbee, 77);
    rig.drain_tx(&mut src);
    let mut h = Header::new(FrameType::Reject);
    h.epoch = rig.epoch;
    h.ack = 1;
    let body = frame::RejectBody {
        seq: 0,
        endpoint: 2,
        ep_seq: 0,
        reason: RejectReason::NoCredit as u8,
    };
    rig.feed(&wire(&h, &body.to_bytes()));
    assert!(
        rig.events
            .iter()
            .any(|e| e.contains("TxFailed") && e.contains("Rejected(1)")),
        "{:?}",
        rig.events
    );
    assert_eq!(rig.link.counters().tx_rejected, 1);
    assert_eq!(rig.link.tx_in_flight().count(), 0);
}

#[test]
fn bulk_loss_does_not_delay_zigbee_delivery() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::ncp(), 8, &[]);
    // BULK seq 0 is lost; ZIGBEE seqs 1..=3 arrive.
    for seq in 1..=3u8 {
        rig.feed(&wire(
            &rig.data_header(seq, 2, seq - 1),
            &tagged_payload(seq as u16),
        ));
    }
    let eps: Vec<_> = rig
        .delivered
        .iter()
        .map(|(e, p)| (*e, payload_tag(p)))
        .collect();
    assert_eq!(
        eps,
        [
            (Endpoint::Zigbee, 1),
            (Endpoint::Zigbee, 2),
            (Endpoint::Zigbee, 3)
        ]
    );
    assert_eq!(rig.link.rx_ack(), 0);
    // BULK arrives later and is delivered then; ack jumps to 4.
    rig.feed(&wire(&rig.data_header(0, 4, 0), &tagged_payload(100)));
    assert_eq!(
        rig.delivered.last().map(|(e, p)| (*e, payload_tag(p))),
        Some((Endpoint::Bulk, 100))
    );
    assert_eq!(rig.link.rx_ack(), 4);
}

#[test]
fn per_endpoint_order_waits_for_missing_ep_seq() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::ncp(), 8, &[]);
    // ZIGBEE ep_seq 1 (seq 1) arrives before ep_seq 0 (seq 0).
    rig.feed(&wire(&rig.data_header(1, 2, 1), &tagged_payload(11)));
    assert!(rig.delivered.is_empty());
    rig.feed(&wire(&rig.data_header(0, 2, 0), &tagged_payload(10)));
    let tags: Vec<u16> = rig.delivered.iter().map(|(_, p)| payload_tag(p)).collect();
    assert_eq!(tags, [10, 11]);
}

#[test]
fn anti_starvation_serves_low_priority_after_eight_frames() {
    let mut rig: Rig<32, 32> =
        Rig::established(LinkConfig::desktop_host(), 32, &[(1, 100), (5, 100)]);
    let mut src = OnePerFrame::default();
    for t in 0..20 {
        src.push(Endpoint::System, t);
    }
    src.push(Endpoint::Log, 1000);
    rig.drain_tx(&mut src);
    let eps: Vec<u8> = rig
        .sent
        .iter()
        .filter(|s| s.header.ftype == FrameType::Data)
        .map(|s| s.header.endpoint)
        .collect();
    let log_pos = eps.iter().position(|&e| e == Endpoint::Log as u8).unwrap();
    // Eight SYSTEM frames, then the starving LOG frame.
    assert_eq!(log_pos, 8, "{eps:?}");
}

#[test]
fn scheduler_priority_order() {
    let mut rig: Rig<16, 16> = Rig::established(
        LinkConfig::desktop_host(),
        16,
        &[(1, 8), (2, 8), (3, 8), (4, 8)],
    );
    let mut src = OnePerFrame::default();
    src.push(Endpoint::Bulk, 4);
    src.push(Endpoint::Zigbee, 2);
    src.push(Endpoint::Tables, 3);
    src.push(Endpoint::System, 1);
    rig.drain_tx(&mut src);
    let eps: Vec<u8> = rig.sent.iter().map(|s| s.header.endpoint).collect();
    assert_eq!(eps, [1, 3, 2, 4]);
}

#[test]
fn minimal_host_absorbs_indication_storm() {
    // rx_window 4, LOG credit 0, BULK credit 1 (3.3).
    let mut hcfg = LinkConfig::embedded_host();
    hcfg.rx_window = 4;
    hcfg.initial_credits = [0, 4, 8, 4, 1, 0];
    let mut s = Minimal::new(hcfg, LinkConfig::ncp(), quiet(), 11);
    s.run_until(5_000, 100);
    assert!(s.established());
    let mut sent: BTreeMap<u8, Vec<u16>> = BTreeMap::new();
    for t in 1..=1000u16 {
        s.ncp.src.push(Endpoint::Zigbee, t);
        sent.entry(Endpoint::Zigbee as u8).or_default().push(t);
    }
    s.ncp.src.push(Endpoint::Log, 5000);
    s.settle(30_000_000);
    let (d, f) = check_accounting(&s.ncp, &s.host, &sent);
    assert_eq!((d, f), (1000, 0));
    // LOG has no credit: it is never sent and stays queued (the app counts it).
    assert!(!s.host.delivered.contains_key(&(Endpoint::Log as u8)));
    assert!(s.ncp.src.ready(Endpoint::Log));
    assert_eq!(s.ncp.link.session().unwrap().tx_window, 4);
}

// ── Resets, epochs, peer restart vs resync ─────────────────────────────

#[test]
fn ncp_reboot_fails_pending_and_resumes_with_new_epoch() {
    let mut s = Embedded::new(LinkConfig::embedded_host(), LinkConfig::ncp(), quiet(), 13);
    s.run_until(5_000, 100);
    let old_epoch = s.host.link.session().unwrap().epoch;
    // Host sends; the NCP reboots before acknowledging.
    s.n2h.blackhole = true;
    for t in 1..=3 {
        s.host.src.push(Endpoint::Zigbee, t);
    }
    s.step(100);
    assert_eq!(s.host.link.tx_in_flight().count(), 3);
    // Stale bytes of the old session are still in the NCP→host pipe.
    let mut stale_h = Header::new(FrameType::Data);
    stale_h.epoch = old_epoch;
    stale_h.endpoint = 2;
    let stale = wire(&stale_h, &tagged_payload(9));
    s.ncp = Side::new(LinkConfig::ncp(), 0xBEEF_0002);
    s.ncp.link.open(s.now, ResetReason::Watchdog);
    s.n2h.blackhole = false;
    s.n2h.queue.clear();
    s.run_until(s.now + 50_000, 100);
    assert!(s.established());
    let new_epoch = s.host.link.session().unwrap().epoch;
    assert_ne!(new_epoch, old_epoch);
    assert!(
        s.host
            .events
            .iter()
            .any(|e| e.contains("PeerReset") && e.contains("Restarted")),
        "{:?}",
        s.host.events
    );
    assert_eq!(s.host.failed.len(), 3);
    assert!(
        s.host
            .failed
            .iter()
            .all(|(_, r)| *r == TxFailure::LinkReset)
    );
    assert_eq!(s.host.link.counters().peer_restarted, 2); // first contact + reboot
    // Old-epoch DATA arriving now is discarded, counted and rejected.
    let before = s.host.link.counters().rx_epoch_mismatch;
    s.host.feed(&stale, s.now);
    assert_eq!(s.host.link.counters().rx_epoch_mismatch, before + 1);
    let mut out = [0u8; TX_BUFFER_LEN];
    let n = s
        .host
        .link
        .poll_transmit(s.now, &mut NoSource, &mut out)
        .unwrap();
    let frames = decode_wire(&out[..n]);
    assert_eq!(frames[0].0.ftype, FrameType::Reject);
    assert_eq!(frames[0].1[3], RejectReason::StaleEpoch as u8);
    // Traffic resumes.
    s.host.src.push(Endpoint::Zigbee, 50);
    s.settle(1_000_000);
    assert_eq!(s.ncp.all_delivered(), [50]);
}

#[test]
fn same_cookie_reset_is_reported_as_resync() {
    let mut s = Embedded::new(LinkConfig::embedded_host(), LinkConfig::ncp(), quiet(), 17);
    s.run_until(5_000, 100);
    // The host loses sync without reopening (e.g. after SYS_SET_UART) and
    // restarts the handshake with the same cookie.
    s.host.link.open(s.now, ResetReason::PeerRequested);
    s.run_until(s.now + 50_000, 100);
    assert!(s.established());
    assert!(
        s.ncp.events.iter().any(|e| e.contains("Resync")),
        "{:?}",
        s.ncp.events
    );
    assert_eq!(s.ncp.link.counters().peer_resync, 1);
}

#[test]
fn link_timeout_reports_link_down_and_resets() {
    let mut s = Embedded::new(LinkConfig::embedded_host(), LinkConfig::ncp(), quiet(), 19);
    s.run_until(5_000, 100);
    s.n2h.blackhole = true;
    s.h2n.blackhole = true;
    s.run_until(4_000_000, 1000);
    assert!(s.host.events.iter().any(|e| e == "LinkDown"));
    assert!(s.ncp.events.iter().any(|e| e == "LinkDown"));
    assert_eq!(s.host.link.state(), LinkState::Resetting);
    s.n2h.blackhole = false;
    s.h2n.blackhole = false;
    s.run_until(s.now + 5_000_000, 1000);
    assert!(s.established());
    // Both kept their cookies: the re-handshake is a resync on both sides.
    assert!(s.host.events.iter().any(|e| e.contains("Resync")));
}

#[test]
fn not_in_session_frames_are_discarded() {
    let mut l = Link::<8, 8>::new(LinkConfig::ncp(), NCP_COOKIE);
    l.open(0, ResetReason::PowerOn);
    let mut h = Header::new(FrameType::Data);
    h.epoch = 33;
    h.endpoint = 2;
    let w = wire(&h, &tagged_payload(1));
    let mut off = 0;
    while off < w.len() {
        off += l.push_rx(&w[off..], 0);
    }
    assert!(l.poll_event().is_none());
    assert_eq!(l.counters().rx_not_in_session, 1);
}

#[test]
fn unknown_frame_type_with_valid_crc_is_rejected() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::ncp(), 8, &[]);
    let mut dec = [0u8; 32];
    let n = frame::encode_decoded(&Header::new(FrameType::Ack), &[], &mut dec).unwrap();
    dec[0] = (1 << 6) | 9; // type 9
    dec[1] = rig.epoch;
    let crc = zigbee_zrhp::crc::crc32c(&dec[..n - 4]);
    dec[n - 4..n].copy_from_slice(&crc.to_le_bytes());
    let mut enc = [0u8; 40];
    let m = zigbee_zrhp::cobs::encode(&dec[..n], &mut enc).unwrap();
    let mut w = enc[..m].to_vec();
    w.push(0);
    rig.feed(&w);
    rig.drain_tx(&mut NoSource);
    let rej = rig
        .sent
        .iter()
        .find(|s| s.header.ftype == FrameType::Reject)
        .unwrap();
    assert_eq!(rej.reject().reason, RejectReason::Unsupported as u8);
    assert_eq!(rig.link.counters().rx_bad_frame, 0);
}

#[test]
fn wrong_role_and_stale_reset_ack_are_ignored() {
    let mut l = Link::<8, 8>::new(LinkConfig::ncp(), NCP_COOKIE);
    l.open(0, ResetReason::PowerOn);
    // RESET_REQ claiming to be an NCP (loopback / misconfiguration).
    let p = reset_payload(HOST_COOKIE, 0, Role::Ncp, 8, &[]);
    let w = wire(&Header::new(FrameType::ResetReq), &p);
    let mut off = 0;
    while off < w.len() {
        off += l.push_rx(&w[off..], 0);
    }
    // RESET_ACK acknowledging somebody else's cookie.
    let p = reset_payload(HOST_COOKIE, 0x1234_5678, Role::Host, 8, &[]);
    let w = wire(&Header::new(FrameType::ResetAck), &p);
    let mut off = 0;
    while off < w.len() {
        off += l.push_rx(&w[off..], 0);
    }
    assert_eq!(l.state(), LinkState::Resetting);
    assert_eq!(l.counters().rx_stale_reset, 2);
    assert_eq!(l.peer_cookie(), None);
    let _ = PeerChange::New;
}

#[test]
fn reset_retry_doubles_up_to_two_seconds() {
    let mut rig: Rig<8, 8> = Rig {
        link: Link::new(LinkConfig::ncp(), NCP_COOKIE),
        now: 0,
        sent: vec![],
        delivered: vec![],
        events: vec![],
        epoch: 0,
        peer_role: Role::Host,
    };
    rig.link.open(0, ResetReason::PowerOn);
    rig.advance(8_000_000, &mut NoSource);
    let times: Vec<u64> = rig.sent.iter().map(|s| s.t / 1000).collect();
    assert_eq!(times, [0, 200, 600, 1400, 3000, 5000, 7000]);
    assert!(
        rig.sent
            .iter()
            .all(|s| s.header.ftype == FrameType::ResetReq)
    );
    assert_eq!(rig.sent[0].reset().reason, ResetReason::PowerOn as u8);
}

#[test]
fn established_link_answers_poll_immediately() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::ncp(), 8, &[]);
    rig.now = 1000;
    let mut h = Header::new(FrameType::Ack);
    h.epoch = rig.epoch;
    h.poll = true;
    let mut p = [0u8; 8];
    let n = frame::AckBody::write(0, 0, &[], &mut p).unwrap();
    rig.feed(&wire(&h, &p[..n]));
    rig.drain_tx(&mut NoSource);
    assert_eq!(rig.sent.len(), 1);
    assert_eq!(rig.sent[0].header.ftype, FrameType::Ack);
    assert_eq!(rig.sent[0].t, 1000);
}

#[test]
fn keepalive_carries_all_credit_edges() {
    let mut rig: Rig<8, 8> = Rig::established(LinkConfig::ncp(), 8, &[]);
    rig.advance(600_000, &mut NoSource);
    let ka = rig
        .sent
        .iter()
        .find(|s| s.header.ftype == FrameType::Ack && s.header.poll)
        .unwrap();
    assert_eq!(ka.t, 500_000);
    assert_eq!(ka.ack_body().2, [(1, 4), (2, 8), (3, 4), (4, 2), (5, 0)]);
}
