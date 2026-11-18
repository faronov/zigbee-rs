//! Cheap fuzz-style robustness tests (spec 14.2 item 4) and message-layer
//! edge cases: random input never panics, work stays bounded, and malformed
//! batches are rejected atomically.

mod common;

use common::Rng;
use zigbee_zrhp::cobs::{self, Accumulator, Feed};
use zigbee_zrhp::codec::{Reader, Wire};
use zigbee_zrhp::frame::{self, MAX_ENCODED_FRAME, MAX_FRAME};
use zigbee_zrhp::link::{Link, LinkConfig};
use zigbee_zrhp::msg::{self, Batch, BatchError, BatchWriter, Kind, MsgHeader};
use zigbee_zrhp::records::{self, Record};
use zigbee_zrhp::{Status, frame::ResetReason};

fn random_bytes(rng: &mut Rng, max: usize) -> Vec<u8> {
    let n = rng.below(max as u64 + 1) as usize;
    (0..n).map(|_| rng.next() as u8).collect()
}

#[test]
fn cobs_roundtrip_random() {
    let mut rng = Rng(0x1234_5678);
    for _ in 0..5_000 {
        let mut raw = random_bytes(&mut rng, MAX_FRAME);
        // Bias towards zeros and long non-zero runs.
        if rng.chance(300_000) {
            raw.iter_mut()
                .for_each(|b| *b = if *b < 40 { 0 } else { *b });
        }
        let mut enc = vec![0u8; cobs::max_encoded_len(raw.len())];
        let n = cobs::encode(&raw, &mut enc).unwrap();
        assert!(n <= cobs::max_encoded_len(raw.len()));
        assert!(!enc[..n].contains(&0));
        let mut dec = enc[..n].to_vec();
        let m = cobs::decode_in_place(&mut dec).unwrap();
        assert_eq!(&dec[..m], &raw[..]);
    }
}

#[test]
fn decoders_never_panic_on_garbage() {
    let mut rng = Rng(0xDEAD_BEEF);
    let mut acc = Accumulator::<MAX_ENCODED_FRAME>::new();
    for _ in 0..20_000 {
        let data = random_bytes(&mut rng, 900);
        let mut buf = data.clone();
        let _ = cobs::decode_in_place(&mut buf);
        let _ = frame::parse(&data);
        let _ = frame::AckBody::parse(&data);
        let _ = frame::ResetBody::parse(&data);
        if let Ok(b) = Batch::parse(&data) {
            for m in b.iter() {
                let _ = m.decode::<msg::data::ApsDataRequest>();
                let _ = m.decode::<msg::data::EvtApsDataIndication>();
                let _ = m.decode::<msg::gp::EvtGpFrame>();
                let _ = m.decode_response::<msg::system::SysNegotiateRsp>();
                let _ = m.decode_response::<msg::tables::TblReadRsp>();
            }
        }
        let mut r = Reader::new(&data);
        while !r.is_empty() {
            if Record::read(&mut r).is_err() {
                break;
            }
        }
        let _ = records::verify_stream(&data);
        let _ = msg::backup::BkpRestorePut::read(&mut Reader::new(&data));
        let _ = msg::diag::DiagGetSourceRoutesRsp::read(&mut Reader::new(&data));
        for &b in &data {
            if let Feed::Frame(n) = acc.push(b) {
                assert!(n <= MAX_ENCODED_FRAME);
            }
        }
    }
}

#[test]
fn link_survives_garbage_storm() {
    let mut rng = Rng(42);
    let mut l = Link::<8, 8>::new(LinkConfig::ncp(), 0xC0FF_EE01);
    l.open(0, ResetReason::PowerOn);
    let mut out = [0u8; frame::TX_BUFFER_LEN];
    let mut now = 0;
    for _ in 0..2_000 {
        let data = random_bytes(&mut rng, 700);
        let mut off = 0;
        while off < data.len() {
            off += l.push_rx(&data[off..], now);
            while l.poll_event().is_some() {}
        }
        now += 1_000;
        let mut guard = 0;
        while l
            .poll_transmit(now, &mut zigbee_zrhp::queue::NoSource, &mut out)
            .is_some()
        {
            guard += 1;
            assert!(guard < 100);
        }
    }
    assert!(l.counters().rx_bad_frame > 0);
}

#[test]
fn batch_must_tile_exactly() {
    let mut buf = [0u8; 64];
    let mut w = BatchWriter::new(&mut buf);
    w.push_raw(Kind::Request, false, msg::ids::SYS_ECHO, 1, b"ab")
        .unwrap();
    w.push_raw(
        Kind::Event,
        false,
        msg::ids::EVT_FATAL,
        2,
        &[0, 0, 0, 0, 0, 0],
    )
    .unwrap();
    let good = w.written().to_vec();
    assert_eq!(Batch::parse(&good).unwrap().iter().count(), 2);
    // One byte short: the whole frame is rejected, nothing is dispatched.
    assert_eq!(
        Batch::parse(&good[..good.len() - 1]).unwrap_err(),
        BatchError::Truncated
    );
    // One extra byte.
    let mut long = good.clone();
    long.push(0);
    assert_eq!(Batch::parse(&long).unwrap_err(), BatchError::Truncated);
    // Reserved kind 3 and reserved flag bits.
    let mut k3 = good.clone();
    k3[0] = 0xC0;
    assert_eq!(Batch::parse(&k3).unwrap_err(), BatchError::ReservedKind);
    let mut rf = good.clone();
    rf[0] = 0x01;
    assert_eq!(Batch::parse(&rf).unwrap_err(), BatchError::ReservedFlags);
    assert_eq!(Batch::parse(&[]).unwrap_err(), BatchError::Empty);
}

#[test]
fn retry_flag_and_header_roundtrip() {
    let h = MsgHeader {
        kind: Kind::Request,
        retry: true,
        msg_id: 0x0201,
        tid: 0xBEEF,
        len: 3,
    };
    assert_eq!(MsgHeader::parse(&h.to_bytes()).unwrap(), h);
    assert_eq!(h.to_bytes()[0], 0x20);
}

#[test]
fn trailing_bytes_are_ignored_and_missing_fields_fail() {
    use msg::network::NwkPermitJoin;
    let mut buf = [0u8; 32];
    let mut w = BatchWriter::new(&mut buf);
    // Payload with one extra trailing byte (a field from a later minor).
    w.push_raw(
        Kind::Request,
        false,
        msg::ids::NWK_PERMIT_JOIN,
        7,
        &[0xFE, 0xFC, 0xFF, 0x01, 0x99],
    )
    .unwrap();
    let bytes = w.written().to_vec();
    let m = Batch::parse(&bytes).unwrap().iter().next().unwrap();
    let r: NwkPermitJoin = m.decode().unwrap();
    assert_eq!(
        r,
        NwkPermitJoin {
            duration_s: 254,
            target: 0xFFFC,
            flags: 1
        }
    );
    // Truncated payload → codec error (the NCP answers INVALID_PARAMETER).
    let mut buf = [0u8; 32];
    let mut w = BatchWriter::new(&mut buf);
    w.push_raw(
        Kind::Request,
        false,
        msg::ids::NWK_PERMIT_JOIN,
        8,
        &[0xFE, 0xFC],
    )
    .unwrap();
    let bytes = w.written().to_vec();
    let m = Batch::parse(&bytes).unwrap().iter().next().unwrap();
    assert!(m.decode::<NwkPermitJoin>().is_err());
}

#[test]
fn status_only_response_has_no_body() {
    use msg::security::SecGetNetworkKeyRsp;
    let mut buf = [0u8; 16];
    let mut w = BatchWriter::new(&mut buf);
    w.response_status(msg::ids::SEC_GET_NETWORK_KEY, 3, Status::AccessDenied)
        .unwrap();
    let bytes = w.written().to_vec();
    let m = Batch::parse(&bytes).unwrap().iter().next().unwrap();
    let r = m.decode_response::<SecGetNetworkKeyRsp>().unwrap();
    assert_eq!(r.status, Status::AccessDenied);
    assert!(r.body.is_none());
    for v in 0..=255u8 {
        assert_eq!(Status::from_u8(v).to_u8(), v);
    }
}

#[test]
fn message_size_limit_is_enforced() {
    let mut buf = [0u8; 700];
    let mut w = BatchWriter::new(&mut buf);
    assert!(
        w.push_raw(
            Kind::Event,
            false,
            0x0980,
            1,
            &[b'x'; msg::MAX_MESSAGE_PAYLOAD]
        )
        .is_ok()
    );
    assert_eq!(w.len(), frame::MAX_PAYLOAD);
    let mut w = BatchWriter::new(&mut buf);
    assert!(
        w.push_raw(
            Kind::Event,
            false,
            0x0980,
            1,
            &[b'x'; msg::MAX_MESSAGE_PAYLOAD + 1]
        )
        .is_err()
    );
    assert!(
        w.is_empty(),
        "failed push leaves the writer at the message boundary"
    );
}

#[test]
fn tid_allocator_skips_zero() {
    let mut t = msg::TidAllocator::new();
    let mut last = 0;
    for _ in 0..70_000 {
        let v = t.next_tid();
        assert_ne!(v, 0);
        last = v;
    }
    assert_eq!(last, (70_000u32 % 65_535) as u16);
}
