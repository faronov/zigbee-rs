//! Link test harness: a scripted fake peer and helpers to run a `Link`
//! against a virtual clock.

#![allow(dead_code)]

use zigbee_zrhp::Endpoint;
use zigbee_zrhp::cobs::{Accumulator, Feed};
use zigbee_zrhp::frame::{
    self, AckBody, CreditEntry, FrameType, Header, MAX_ENCODED_FRAME, RejectBody, ResetBody, Role,
    TX_BUFFER_LEN,
};
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent};
use zigbee_zrhp::msg::{BatchWriter, Kind};
use zigbee_zrhp::queue::{NoSource, TxSource};

pub const PEER_COOKIE: u32 = 0x1A2B_3C4D;
pub const OWN_COOKIE: u32 = 0xC0FF_EE01;

/// A decoded frame captured from the link under test.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sent {
    pub t: u64,
    pub header: Header,
    pub payload: Vec<u8>,
}

impl Sent {
    pub fn ack_body(&self) -> (u32, u8, Vec<(u8, u8)>) {
        let b = AckBody::parse(&self.payload).unwrap();
        (
            b.sack,
            b.nbad,
            b.credits.map(|c| (c.endpoint, c.value)).collect(),
        )
    }
    pub fn reject(&self) -> RejectBody {
        RejectBody::parse(&self.payload).unwrap()
    }
    pub fn reset(&self) -> ResetBody<'_> {
        ResetBody::parse(&self.payload).unwrap()
    }
}

/// Decode every frame in a wire chunk.
pub fn decode_wire(bytes: &[u8]) -> Vec<(Header, Vec<u8>)> {
    let mut acc = Accumulator::<MAX_ENCODED_FRAME>::new();
    let mut out = Vec::new();
    for &b in bytes {
        if let Feed::Frame(n) = acc.push(b) {
            let f = frame::parse(acc.frame(n)).unwrap();
            out.push((f.header, f.payload.to_vec()));
        }
    }
    out
}

pub fn wire(h: &Header, payload: &[u8]) -> Vec<u8> {
    let mut out = [0u8; TX_BUFFER_LEN];
    let n = frame::encode_wire(h, &[payload], false, &mut out).unwrap();
    out[..n].to_vec()
}

pub fn reset_payload(
    cookie: u32,
    peer_cookie: u32,
    role: Role,
    rx_window: u8,
    credits: &[(u8, u8)],
) -> Vec<u8> {
    let body = ResetBody {
        cookie,
        peer_cookie,
        role: role as u8,
        link_version: 1,
        rx_window,
        reason: 4,
        rx_mtu: 640,
        ack_delay_us: 1000,
        keepalive_ms: 500,
        link_timeout_ms: 3000,
        credits: frame::Credits::default(),
    };
    let c: Vec<CreditEntry> = credits
        .iter()
        .map(|&(endpoint, value)| CreditEntry { endpoint, value })
        .collect();
    let mut out = [0u8; 64];
    let n = body.write(&c, &mut out).unwrap();
    out[..n].to_vec()
}

/// A one-message DATA payload whose TID identifies it.
pub fn tagged_payload(tag: u16) -> Vec<u8> {
    let mut buf = [0u8; 16];
    let mut w = BatchWriter::new(&mut buf);
    w.push_raw(Kind::Request, false, 0x0201, tag, &[]).unwrap();
    w.written().to_vec()
}

pub fn payload_tag(p: &[u8]) -> u16 {
    u16::from_le_bytes([p[3], p[4]])
}

/// Drives one `Link` against a scripted peer.
pub struct Rig<const T: usize, const R: usize> {
    pub link: Link<T, R>,
    pub now: u64,
    pub sent: Vec<Sent>,
    pub delivered: Vec<(Endpoint, Vec<u8>)>,
    pub events: Vec<String>,
    pub epoch: u8,
    pub peer_role: Role,
}

impl<const T: usize, const R: usize> Rig<T, R> {
    /// Establish a session with a fake peer at t = 0. The link under test
    /// uses `cfg`; the fake peer advertises `peer_window` and `peer_credits`.
    pub fn established(cfg: LinkConfig, peer_window: u8, peer_credits: &[(u8, u8)]) -> Self {
        let peer_role = if cfg.role == Role::Host {
            Role::Ncp
        } else {
            Role::Host
        };
        let mut rig = Self {
            link: Link::new(cfg, OWN_COOKIE),
            now: 0,
            sent: Vec::new(),
            delivered: Vec::new(),
            events: Vec::new(),
            epoch: 0,
            peer_role,
        };
        rig.link.open(0, frame::ResetReason::PowerOn);
        rig.drain_tx(&mut NoSource);
        let p = reset_payload(
            PEER_COOKIE,
            OWN_COOKIE,
            peer_role,
            peer_window,
            peer_credits,
        );
        rig.feed(&wire(&Header::new(FrameType::ResetAck), &p));
        rig.drain_tx(&mut NoSource);
        let s = rig.link.session().expect("established");
        rig.epoch = s.epoch;
        let (h, n) = if cfg.role == Role::Host {
            (OWN_COOKIE, PEER_COOKIE)
        } else {
            (PEER_COOKIE, OWN_COOKIE)
        };
        assert_eq!(s.epoch, frame::session_epoch(h, n));
        rig.sent.clear();
        rig.events.clear();
        rig
    }

    pub fn feed(&mut self, bytes: &[u8]) {
        let mut off = 0;
        while off < bytes.len() {
            off += self.link.push_rx(&bytes[off..], self.now);
            self.drain_events();
        }
    }

    pub fn drain_events(&mut self) {
        while let Some(ev) = self.link.poll_event() {
            match ev {
                LinkEvent::Data { endpoint, payload } => {
                    self.delivered.push((endpoint, payload.to_vec()))
                }
                other => self.events.push(format!("{other:?}")),
            }
        }
    }

    pub fn drain_tx(&mut self, src: &mut impl TxSource) -> usize {
        let mut out = [0u8; TX_BUFFER_LEN];
        let mut count = 0;
        while let Some(n) = self.link.poll_transmit(self.now, src, &mut out) {
            for (header, payload) in decode_wire(&out[..n]) {
                self.sent.push(Sent {
                    t: self.now,
                    header,
                    payload,
                });
            }
            count += 1;
            assert!(count < 10_000, "transmit loop");
        }
        self.drain_events();
        count
    }

    /// Advance the virtual clock to `to`, servicing every timer on the way.
    pub fn advance(&mut self, to: u64, src: &mut impl TxSource) {
        let mut guard = 0;
        while let Some(d) = self.link.next_deadline() {
            if d > to {
                break;
            }
            self.now = self.now.max(d);
            if self.drain_tx(src) == 0 && self.link.next_deadline() == Some(d) {
                break;
            }
            guard += 1;
            assert!(guard < 10_000, "timer loop");
        }
        self.now = self.now.max(to);
        self.drain_tx(src);
    }

    pub fn data_header(&self, seq: u8, endpoint: u8, ep_seq: u8) -> Header {
        let mut h = Header::new(FrameType::Data);
        h.epoch = self.epoch;
        h.endpoint = endpoint;
        h.seq = seq;
        h.ep_seq = ep_seq;
        h
    }

    pub fn ack_frame(
        &self,
        ftype: FrameType,
        ack: u8,
        sack: u32,
        nbad: u8,
        credits: &[(u8, u8)],
    ) -> Vec<u8> {
        let mut h = Header::new(ftype);
        h.epoch = self.epoch;
        h.ack = ack;
        let c: Vec<CreditEntry> = credits
            .iter()
            .map(|&(endpoint, value)| CreditEntry { endpoint, value })
            .collect();
        let mut p = [0u8; 64];
        let n = AckBody::write(sack, nbad, &c, &mut p).unwrap();
        wire(&h, &p[..n])
    }
}

/// A source that hands out one tagged message per frame.
#[derive(Debug, Default)]
pub struct OnePerFrame {
    pub queue: std::collections::VecDeque<(Endpoint, u16)>,
}

impl OnePerFrame {
    pub fn push(&mut self, ep: Endpoint, tag: u16) {
        self.queue.push_back((ep, tag));
    }
}

impl TxSource for OnePerFrame {
    fn ready(&self, endpoint: Endpoint) -> bool {
        self.queue.iter().any(|(e, _)| *e == endpoint)
    }
    fn fill(&mut self, endpoint: Endpoint, buf: &mut [u8]) -> usize {
        let Some(i) = self.queue.iter().position(|(e, _)| *e == endpoint) else {
            return 0;
        };
        let (_, tag) = self.queue.remove(i).unwrap();
        let p = tagged_payload(tag);
        buf[..p.len()].copy_from_slice(&p);
        p.len()
    }
}
