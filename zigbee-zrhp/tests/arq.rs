//! Normative ARQ traces (`arq_scenarios.json`, spec 5.9): a virtual clock
//! drives the link and it must emit exactly the listed frames at exactly the
//! listed times.

mod common;

use common::harness::{OnePerFrame, Rig, payload_tag, tagged_payload, wire};
use common::{Json, load};
use zigbee_zrhp::Endpoint;
use zigbee_zrhp::frame::{FrameType, Role};
use zigbee_zrhp::link::LinkConfig;
use zigbee_zrhp::queue::NoSource;

fn config(a: &Json, role: Role) -> LinkConfig {
    let base = if role == Role::Ncp {
        LinkConfig::ncp()
    } else {
        LinkConfig::embedded_host()
    };
    LinkConfig {
        role,
        rx_window: a.at("rx_window").u64() as u8,
        ack_delay_us: a.at("ack_delay_us").u64() as u16,
        rto_initial_us: a.at("rto_initial_us").u64() as u32,
        adaptive_rto: a.at("rto_adaptive").bool(),
        auto_credit: false,
        ..base
    }
}

fn run_receiver(sc: &Json, a: &Json) {
    let name = sc.at("name").str();
    let mut cfg = config(a, Role::Ncp);
    if let Some(edges) = sc.get("initial_credit_edge") {
        for (ep, v) in edges.obj() {
            cfg.initial_credits[ep.parse::<usize>().unwrap()] = v.u64() as u8;
        }
    }
    assert_eq!((cfg.rx_window / 2) as u64, a.at("ack_every_frames").u64());
    let mut rig: Rig<8, 8> = Rig::established(cfg, 8, &[(1, 4), (2, 8), (3, 4), (4, 4), (5, 2)]);
    let mut end = 0;
    for input in sc.at("inputs").arr() {
        let t = input.at("t").u64();
        end = end.max(t);
        rig.advance(t, &mut NoSource);
        if let Some(seq) = input.get("data") {
            let seq = seq.u64() as u8;
            let ep = input
                .get("endpoint")
                .map(|e| e.u64() as u8)
                .unwrap_or(Endpoint::Zigbee as u8);
            let ep_seq = input.get("ep_seq").map(|e| e.u64() as u8).unwrap_or(seq);
            let mut h = rig.data_header(seq, ep, ep_seq);
            h.retx = input.get("retx").is_some_and(Json::bool);
            h.poll = input.get("poll").is_some_and(Json::bool);
            let w = wire(&h, &tagged_payload(seq as u16));
            rig.feed(&w);
        } else if input.get("crc_error").is_some_and(Json::bool) {
            let h = rig.data_header(9, Endpoint::Zigbee as u8, 9);
            let mut w = wire(&h, &tagged_payload(999));
            let n = w.len();
            w[n - 3] ^= 0x55; // corrupt the CRC (never produces 0x00 from non-zero COBS here)
            assert!(w[..n - 1].iter().all(|&b| b != 0));
            rig.feed(&w);
        } else if let Some(f) = input.get("consumer_frees") {
            let ep = Endpoint::from_u8(f.at("endpoint").u64() as u8).unwrap();
            rig.link.release_credit(ep, f.at("frames").u64() as u8);
        } else {
            panic!("{name}: unknown input {input:?}");
        }
        rig.drain_tx(&mut NoSource);
    }
    let expect = sc.at("expect").arr();
    for e in expect {
        end = end.max(e.at("t").u64());
    }
    rig.advance(end + 5_000, &mut NoSource);

    let got: Vec<_> = rig
        .sent
        .iter()
        .filter(|s| s.header.ftype != FrameType::Data)
        .collect();
    assert_eq!(got.len(), expect.len(), "{name}: sent {:#?}", rig.sent);
    for (s, e) in got.iter().zip(expect) {
        assert_eq!(s.t, e.at("t").u64(), "{name}: time of {s:?}");
        let want_type = match e.at("send").str() {
            "ACK" => FrameType::Ack,
            "NAK" => FrameType::Nak,
            other => panic!("{other}"),
        };
        assert_eq!(s.header.ftype, want_type, "{name} at {}", s.t);
        assert_eq!(
            s.header.ack as u64,
            e.at("ack").u64(),
            "{name} ack at {}",
            s.t
        );
        assert_eq!(s.header.epoch, rig.epoch);
        let (sack, nbad, credits) = s.ack_body();
        assert_eq!(sack as u64, e.at("sack").u64(), "{name} sack at {}", s.t);
        assert_eq!(
            nbad as u64,
            e.get("nbad").map(Json::u64).unwrap_or(0),
            "{name} nbad at {}",
            s.t
        );
        let want_credits: Vec<(u8, u8)> = e
            .get("credits")
            .map(|c| {
                c.obj()
                    .iter()
                    .map(|(k, v)| (k.parse().unwrap(), v.u64() as u8))
                    .collect()
            })
            .unwrap_or_default();
        assert_eq!(credits, want_credits, "{name} credits at {}", s.t);
    }
    if let Some(d) = sc.get("delivered") {
        let got: Vec<u64> = rig
            .delivered
            .iter()
            .map(|(_, p)| payload_tag(p) as u64)
            .collect();
        let want: Vec<u64> = d.arr().iter().map(Json::u64).collect();
        assert_eq!(got, want, "{name}: delivered");
    }
    if let Some(c) = sc.get("counters") {
        for (k, v) in c.obj() {
            let got = match k.as_str() {
                "rx_duplicate" => rig.link.counters().rx_duplicate,
                other => panic!("{other}"),
            };
            assert_eq!(got as u64, v.u64(), "{name}: {k}");
        }
    }
}

fn run_sender(sc: &Json, a: &Json) {
    let name = sc.at("name").str();
    let cfg = config(a, Role::Host);
    let mut rig: Rig<8, 8> = Rig::established(
        cfg,
        a.at("rx_window").u64() as u8,
        &[(1, 4), (2, 8), (3, 4), (4, 2), (5, 0)],
    );
    let mut src = OnePerFrame::default();
    let mut end = 0;
    for input in sc.at("inputs").arr() {
        let t = input.at("t").u64();
        end = end.max(t);
        rig.advance(t, &mut src);
        if let Some(list) = input.get("send_new") {
            let before = rig.sent.len();
            for s in list.arr() {
                src.push(Endpoint::Zigbee, s.u64() as u16);
            }
            rig.drain_tx(&mut src);
            let new: Vec<u64> = rig.sent[before..]
                .iter()
                .map(|s| s.header.seq as u64)
                .collect();
            let want: Vec<u64> = list.arr().iter().map(Json::u64).collect();
            assert_eq!(new, want, "{name}: new DATA seqs");
            assert!(
                rig.sent[before..]
                    .iter()
                    .all(|s| !s.header.retx && s.header.ftype == FrameType::Data)
            );
        } else if let Some(kind) = input.get("recv") {
            let ftype = match kind.str() {
                "ACK" => FrameType::Ack,
                "NAK" => FrameType::Nak,
                other => panic!("{other}"),
            };
            let nbad = input.get("nbad").map(Json::u64).unwrap_or(0) as u8;
            let w = rig.ack_frame(
                ftype,
                input.at("ack").u64() as u8,
                input.at("sack").u64() as u32,
                nbad,
                &[],
            );
            rig.feed(&w);
        } else {
            panic!("{name}: unknown input");
        }
        rig.drain_tx(&mut src);
    }
    let expect = sc.at("expect").arr();
    for e in expect {
        end = end.max(e.at("t").u64());
    }
    rig.advance(end + 5_000, &mut src);
    let retx: Vec<_> = rig
        .sent
        .iter()
        .filter(|s| s.header.retx || s.header.ftype != FrameType::Data)
        .collect();
    assert_eq!(retx.len(), expect.len(), "{name}: {:#?}", rig.sent);
    for (s, e) in retx.iter().zip(expect) {
        assert_eq!(s.header.ftype, FrameType::Data, "{name}");
        assert_eq!(s.t, e.at("t").u64(), "{name}: retransmission time");
        assert_eq!(s.header.seq as u64, e.at("retransmit").u64(), "{name}");
        let flags: Vec<&str> = e.at("flags").arr().iter().map(Json::str).collect();
        assert_eq!(s.header.retx, flags.contains(&"RETX"), "{name}");
        assert_eq!(s.header.poll, flags.contains(&"POLL"), "{name}");
        assert_eq!(
            payload_tag(&s.payload),
            s.header.seq as u16,
            "{name}: byte-identical payload"
        );
    }
    let fin = sc.at("final");
    assert_eq!(
        rig.link.tx_base() as u64,
        fin.at("base").u64(),
        "{name}: base"
    );
    let mut unacked: Vec<u64> = rig.link.tx_unacked().map(u64::from).collect();
    unacked.sort();
    let want: Vec<u64> = fin.at("unacked").arr().iter().map(Json::u64).collect();
    assert_eq!(unacked, want, "{name}: unacked");
    if let Some(rto) = fin.get("rto_us") {
        assert_eq!(rig.link.rto_us(), rto.u64(), "{name}: rto");
    }
}

#[test]
fn arq_scenarios() {
    let v = load("arq_scenarios.json");
    let a = v.at("assumptions");
    let mut n = 0;
    for sc in v.at("scenarios").arr() {
        match sc.at("role").str() {
            "receiver" => run_receiver(sc, a),
            "sender" => run_sender(sc, a),
            other => panic!("role {other}"),
        }
        n += 1;
    }
    assert_eq!(n, 8);
}
