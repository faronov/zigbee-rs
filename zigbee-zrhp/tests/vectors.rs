//! Drives every shared JSON vector through the crate's codecs, byte-exact
//! in both directions (spec 14.1).

mod common;

use common::{Json, hex, load, unhex};
use zigbee_zrhp::cobs::{self, Accumulator, Feed};
use zigbee_zrhp::codec::{List, Reader, Wire, Writer};
use zigbee_zrhp::crc::crc32c;
use zigbee_zrhp::frame::{
    self, AckBody, CreditEntry, FrameType, Header, MAX_ENCODED_FRAME, RejectBody, ResetBody,
    TX_BUFFER_LEN,
};
use zigbee_zrhp::msg::{self, Batch, BatchWriter, Kind, MessageDef, RawMessage, Response};
use zigbee_zrhp::records::{self, Record, RestoreCounters};
use zigbee_zrhp::{Endpoint, Ieee, Status};

// ── CRC-32C and COBS ───────────────────────────────────────────────────

#[test]
fn crc32c_vectors() {
    let v = load("crc32c.json");
    for case in v.at("vectors").arr() {
        let input = match case.get("input_ascii") {
            Some(a) => a.str().as_bytes().to_vec(),
            None => unhex(case.at("input_hex").str()),
        };
        let want = u32::from_str_radix(case.at("crc32c").str(), 16).unwrap();
        assert_eq!(crc32c(&input), want, "{case:?}");
    }
}

#[test]
fn cobs_vectors() {
    let v = load("cobs.json");
    for case in v.at("vectors").arr() {
        let name = case.at("name").str();
        let raw = unhex(case.at("raw_hex").str());
        let enc = unhex(case.at("encoded_hex").str());
        assert_eq!(enc.len() as u64, case.at("encoded_len").u64());
        let mut out = vec![0u8; cobs::max_encoded_len(raw.len())];
        let n = cobs::encode(&raw, &mut out).unwrap();
        assert_eq!(&out[..n], &enc[..], "encode {name}");
        let mut dec = vec![0u8; raw.len() + 1];
        let m = cobs::decode(&enc, &mut dec).unwrap();
        assert_eq!(&dec[..m], &raw[..], "decode {name}");
        let mut inplace = enc.clone();
        let k = cobs::decode_in_place(&mut inplace).unwrap();
        assert_eq!(&inplace[..k], &raw[..], "decode_in_place {name}");
        // Streaming accumulator with the wire delimiter.
        let mut acc = Accumulator::<MAX_ENCODED_FRAME>::new();
        let mut got = None;
        for &b in enc.iter().chain([0u8].iter()) {
            if let Feed::Frame(len) = acc.push(b) {
                got = Some(acc.frame(len).to_vec());
            }
        }
        if raw.is_empty() {
            // `01 00`: an encoded empty frame is a valid (zero-length) frame.
            assert_eq!(got.as_deref(), Some(&[][..]), "{name}");
        } else {
            assert_eq!(got.as_deref(), Some(&raw[..]), "accumulator {name}");
        }
    }
    for case in v.at("decode_only").arr() {
        let raw = unhex(case.at("raw_hex").str());
        let enc = unhex(case.at("encoded_hex").str());
        let mut dec = vec![0u8; raw.len() + 4];
        let n = cobs::decode(&enc, &mut dec).unwrap();
        assert_eq!(&dec[..n], &raw[..]);
        let mut inplace = enc.clone();
        let k = cobs::decode_in_place(&mut inplace).unwrap();
        assert_eq!(&inplace[..k], &raw[..]);
    }
}

// ── Link frames ────────────────────────────────────────────────────────

fn header_from(fields: &Json, ftype: FrameType) -> Header {
    let mut h = Header::new(ftype);
    let f = |k: &str| fields.get(k).map(|v| v.u64() as u8).unwrap_or(0);
    h.epoch = f("epoch");
    h.endpoint = f("endpoint");
    h.seq = f("seq");
    h.ack = f("ack");
    h.ep_seq = f("ep_seq");
    let flags = f("flags");
    h.poll = flags & frame::CTRL_POLL != 0;
    h.retx = flags & frame::CTRL_RETX != 0;
    h
}

fn credits_of(c: frame::Credits<'_>) -> Vec<CreditEntry> {
    c.collect()
}

#[test]
fn link_frame_vectors() {
    let v = load("link_frames.json");
    let session = v.at("session");
    let hc = u32::from_str_radix(session.at("host_cookie").str(), 16).unwrap();
    let nc = u32::from_str_radix(session.at("ncp_cookie").str(), 16).unwrap();
    assert_eq!(
        frame::session_epoch(hc, nc) as u64,
        session.at("epoch").u64()
    );

    let mut max_wire = 0;
    for case in v.at("vectors").arr() {
        let name = case.at("name").str();
        let ftype = FrameType::from_u8(case.at("type").u64() as u8).unwrap();
        let payload = unhex(case.at("payload_hex").str());
        let decoded = unhex(case.at("decoded_hex").str());
        let wire = unhex(case.at("wire_hex").str());
        let crc = u32::from_str_radix(case.at("crc32c").str(), 16).unwrap();
        max_wire = max_wire.max(wire.len());

        // Re-build the payload from its parsed body (field-level round trip).
        let rebuilt = match ftype {
            FrameType::ResetReq | FrameType::ResetAck => {
                let body = ResetBody::parse(&payload).unwrap();
                assert_eq!(body.rx_mtu, 640, "{name}");
                let credits = credits_of(body.credits.clone());
                let mut out = [0u8; 64];
                let n = body.write(&credits, &mut out).unwrap();
                out[..n].to_vec()
            }
            FrameType::Ack | FrameType::Nak => {
                let body = AckBody::parse(&payload).unwrap();
                let credits = credits_of(body.credits.clone());
                let mut out = [0u8; 64];
                let n = AckBody::write(body.sack, body.nbad, &credits, &mut out).unwrap();
                out[..n].to_vec()
            }
            FrameType::Reject => RejectBody::parse(&payload).unwrap().to_bytes().to_vec(),
            FrameType::Data => payload.clone(),
        };
        assert_eq!(rebuilt, payload, "body {name}");

        let h = header_from(case.at("fields"), ftype);
        let mut dec = [0u8; 700];
        let n = frame::encode_decoded(&h, &[&payload], &mut dec).unwrap();
        assert_eq!(hex(&dec[..n]), hex(&decoded), "decoded {name}");
        assert_eq!(crc32c(&decoded[..decoded.len() - 4]), crc, "crc {name}");
        let mut out = [0u8; TX_BUFFER_LEN];
        let w = frame::encode_wire(&h, &[&payload], false, &mut out).unwrap();
        assert_eq!(hex(&out[..w]), hex(&wire), "wire {name}");
        // Leading resync zero variant.
        let w0 = frame::encode_wire(&h, &[&payload], true, &mut out).unwrap();
        assert_eq!(out[0], 0);
        assert_eq!(&out[1..w0], &wire[..], "wire+lead {name}");

        // Parse back from the wire through the streaming accumulator.
        let mut acc = Accumulator::<MAX_ENCODED_FRAME>::new();
        let mut parsed = None;
        for &b in &wire {
            if let Feed::Frame(len) = acc.push(b) {
                let f = frame::parse(acc.frame(len)).unwrap();
                parsed = Some((f.header, f.payload.to_vec()));
            }
        }
        let (ph, pp) = parsed.unwrap_or_else(|| panic!("no frame for {name}"));
        assert_eq!(ph, h, "header {name}");
        assert_eq!(pp, payload);
    }
    assert_eq!(max_wire as u64, v.at("max_wire_len_in_set").u64());
    assert_eq!(max_wire, frame::MAX_WIRE_FRAME);
}

#[test]
fn link_frame_field_values() {
    let v = load("link_frames.json");
    let by_name = |n: &str| {
        v.at("vectors")
            .arr()
            .iter()
            .find(|c| c.at("name").str() == n)
            .unwrap()
            .clone()
    };
    let req = by_name("reset_req_host");
    let p = unhex(req.at("payload_hex").str());
    let b = ResetBody::parse(&p).unwrap();
    assert_eq!(
        (b.cookie, b.peer_cookie, b.role, b.link_version),
        (0x1A2B_3C4D, 0, 0, 1)
    );
    assert_eq!(
        (
            b.rx_window,
            b.reason,
            b.ack_delay_us,
            b.keepalive_ms,
            b.link_timeout_ms
        ),
        (32, 4, 1000, 500, 3000)
    );
    let cr: Vec<(u8, u8)> = b.credits.map(|c| (c.endpoint, c.value)).collect();
    assert_eq!(cr, [(1, 4), (2, 8), (3, 4), (4, 4), (5, 2)]);

    let ack = by_name("reset_ack_ncp");
    let p = unhex(ack.at("payload_hex").str());
    let b = ResetBody::parse(&p).unwrap();
    assert_eq!(
        (b.cookie, b.peer_cookie, b.role, b.rx_window),
        (0xC0FF_EE01, 0x1A2B_3C4D, 1, 16)
    );
    let cr: Vec<(u8, u8)> = b.credits.map(|c| (c.endpoint, c.value)).collect();
    assert_eq!(cr, [(1, 4), (2, 8), (3, 4), (4, 2), (5, 0)]);

    let nak = by_name("nak_gap");
    let b = AckBody::parse(&unhex(nak.at("payload_hex").str()))
        .unwrap()
        .sack;
    assert_eq!(b, 0b11);
    let rj = by_name("reject_no_credit");
    let r = RejectBody::parse(&unhex(rj.at("payload_hex").str())).unwrap();
    assert_eq!(
        r,
        RejectBody {
            seq: 7,
            endpoint: 4,
            ep_seq: 6,
            reason: frame::RejectReason::NoCredit as u8
        }
    );
}

// ── Messages ───────────────────────────────────────────────────────────

/// Decode a body, re-encode it into a full message and compare bytes.
fn rt<'a, T>(m: &RawMessage<'a>, whole: &[u8]) -> T
where
    T: Wire<'a> + MessageDef + core::fmt::Debug + Clone,
{
    assert_eq!(T::ID, m.header.msg_id);
    assert_eq!(T::KIND, m.header.kind);
    let v: T = m.decode().unwrap();
    let mut buf = [0u8; 700];
    let mut w = BatchWriter::new(&mut buf);
    match T::KIND {
        Kind::Request => w.request(m.header.tid, &v).unwrap(),
        Kind::Event => w.event(m.header.tid, &v).unwrap(),
        Kind::Response => unreachable!(),
    }
    assert_eq!(hex(w.written()), hex(whole), "{v:?}");
    v
}

fn rt_rsp<'a, T>(m: &RawMessage<'a>, whole: &[u8]) -> Response<T>
where
    T: Wire<'a> + MessageDef + core::fmt::Debug + Clone,
{
    assert_eq!(T::ID, m.header.msg_id);
    assert_eq!(m.header.kind, Kind::Response);
    let r: Response<T> = m.decode_response().unwrap();
    let mut buf = [0u8; 700];
    let mut w = BatchWriter::new(&mut buf);
    w.response(m.header.tid, r.status, r.body.as_ref()).unwrap();
    assert_eq!(hex(w.written()), hex(whole), "{r:?}");
    r
}

fn status_only(m: &RawMessage<'_>, whole: &[u8]) -> Status {
    let s = m.status().unwrap();
    assert_eq!(m.payload.len(), 1);
    let mut buf = [0u8; 16];
    let mut w = BatchWriter::new(&mut buf);
    w.response_status(m.header.msg_id, m.header.tid, s).unwrap();
    assert_eq!(w.written(), whole);
    s
}

fn ieee_be(s: &str) -> Ieee {
    Ieee::from_u64(u64::from_str_radix(s, 16).unwrap())
}

#[test]
fn message_vectors() {
    use zigbee_zrhp::msg::{data::*, gp::*, log::*, network::*, security::*, system::*, tables::*};
    let v = load("messages.json");
    let mut seen = 0;
    for case in v.arr() {
        let name = case.at("name").str();
        let whole = unhex(case.at("message_hex").str());
        let payload = unhex(case.at("payload_hex").str());
        let fields = case.at("fields");
        let batch = Batch::parse(&whole).unwrap();
        let mut it = batch.iter();
        let m = it.next().unwrap();
        assert!(it.next().is_none());
        assert_eq!(m.payload, &payload[..], "{name}");
        assert_eq!(m.header.kind as u64, case.at("kind").u64(), "{name}");
        assert_eq!(m.header.msg_id as u64, case.at("msg_id").int(), "{name}");
        assert_eq!(m.header.tid as u64, case.at("tid").u64(), "{name}");
        assert_eq!(
            msg::home_endpoint(m.header.msg_id).unwrap() as u64,
            case.at("endpoint").u64(),
            "{name}"
        );
        assert!(msg::ids::is_known(m.header.msg_id), "{name}");
        seen += 1;
        match name {
            "sys_negotiate_req" => {
                let r: SysNegotiateReq = rt(&m, &whole);
                assert_eq!(r.major_min as u64, fields.at("major_min").u64());
                assert_eq!(r.major_max as u64, fields.at("major_max").u64());
                assert_eq!(r.host_caps, fields.at("host_caps").u64());
                assert_eq!(r.name.as_str(), Some(fields.at("name").str()));
                // Build from fields.
                let built = SysNegotiateReq {
                    major_min: 1,
                    major_max: 1,
                    host_caps: 0,
                    name: zigbee_zrhp::codec::Bytes8(b"zigpy-zrs/0.1"),
                };
                let mut buf = [0u8; 64];
                let n =
                    msg::encode_message(Kind::Request, SysNegotiateReq::ID, 1, &built, &mut buf)
                        .unwrap();
                assert_eq!(&buf[..n], &whole[..]);
            }
            "sys_negotiate_rsp" => {
                let r = rt_rsp::<SysNegotiateRsp>(&m, &whole);
                assert_eq!(r.status, Status::Success);
                let b = r.body.unwrap();
                assert_eq!((b.major, b.minor, b.chip), (1, 0, 1));
                assert_eq!(b.ncp_caps, fields.at("ncp_caps").int());
                assert_eq!(b.fw.as_str(), Some(fields.at("fw").str()));
                let caps = fields.at("capacities").obj();
                assert_eq!(b.capacities.len(), caps.len());
                for (k, val) in caps {
                    assert_eq!(
                        b.capacity(k.parse().unwrap()),
                        Some(val.u64() as u32),
                        "capacity {k}"
                    );
                }
                // Build from fields.
                let list: Vec<Capacity> = caps
                    .iter()
                    .map(|(k, val)| Capacity {
                        id: k.parse().unwrap(),
                        value: val.u64() as u32,
                    })
                    .collect();
                let built = SysNegotiateRsp {
                    major: 1,
                    minor: 0,
                    ncp_caps: fields.at("ncp_caps").int(),
                    capacities: List::new(&list),
                    fw: zigbee_zrhp::codec::Bytes8(b"0.1.0"),
                    build_id: b.build_id,
                    chip: 1,
                };
                let mut buf = [0u8; 200];
                let mut w = BatchWriter::new(&mut buf);
                w.response(1, Status::Success, Some(&built)).unwrap();
                assert_eq!(w.written(), &whole[..]);
            }
            "evt_ncp_ready" => {
                let e: EvtNcpReady = rt(&m, &whole);
                let built = EvtNcpReady {
                    ncp_cookie: u32::from_str_radix(fields.at("ncp_cookie").str(), 16).unwrap(),
                    reset_reason: fields.at("reset_reason").u64() as u8,
                    network_state: fields.at("network_state").u64() as u8,
                    table_epoch: fields.at("table_epoch").u64() as u32,
                    nwk_fc_limit: fields.at("nwk_fc_limit").u64() as u32,
                    aps_fc_limit: fields.at("aps_fc_limit").u64() as u32,
                    dirty_writebacks: fields.at("dirty_writebacks").u64() as u16,
                    dropped_indications: fields.at("dropped_indications").u64() as u32,
                };
                assert_eq!(e, built);
            }
            "aps_data_request_unicast" => {
                let r: ApsDataRequest = rt(&m, &whole);
                assert_eq!(r.dst, ApsDst::Nwk(fields.at("dst").int() as u16));
                assert_eq!((r.dst_ep, r.src_ep), (1, 1));
                assert_eq!((r.profile, r.cluster), (0x0104, 0x0000));
                assert_eq!((r.tx_options, r.radius, r.priority), (1, 0, 1));
                assert!(r.relays.is_none() && r.alias.is_none());
                let built = ApsDataRequest {
                    dst: ApsDst::Nwk(0x1A2B),
                    dst_ep: 1,
                    src_ep: 1,
                    profile: 0x0104,
                    cluster: 0,
                    tx_options: tx_options::APS_ACK,
                    radius: 0,
                    non_member_radius: 0,
                    priority: 1,
                    relays: None,
                    alias: None,
                    asdu: r.asdu,
                };
                assert_eq!(r, built);
            }
            "aps_data_request_source_routed" => {
                let r: ApsDataRequest = rt(&m, &whole);
                let relays: Vec<u16> = r.relays.as_ref().unwrap().iter().collect();
                let want: Vec<u16> = fields
                    .at("relays")
                    .arr()
                    .iter()
                    .map(|x| x.int() as u16)
                    .collect();
                assert_eq!(relays, want);
                // Encoding derives bit 6 from `relays`.
                let mut built = r.clone();
                built.tx_options = tx_options::APS_ACK;
                let mut buf = [0u8; 64];
                let n = msg::encode_message(
                    Kind::Request,
                    msg::ids::APS_DATA_REQUEST,
                    259,
                    &built,
                    &mut buf,
                )
                .unwrap();
                assert_eq!(&buf[..n], &whole[..]);
            }
            "aps_data_request_rsp_accepted" => assert_eq!(status_only(&m, &whole), Status::Success),
            "aps_data_request_rsp_busy" => assert_eq!(status_only(&m, &whole), Status::Busy),
            "evt_aps_data_confirm" => {
                let e: EvtApsDataConfirm = rt(&m, &whole);
                assert_eq!(e.req_tid as u64, fields.at("req_tid").int());
                assert_eq!(e.zb_status as u64, fields.at("zb_status").u64());
                assert_eq!(e.aps_counter as u64, fields.at("aps_counter").u64());
                assert_eq!((e.dst_nwk, e.flags, e.elapsed_ms), (0x1A2B, 1, 23));
            }
            "evt_aps_data_indication" => {
                let e: EvtApsDataIndication = rt(&m, &whole);
                assert_eq!(e.src_nwk as u64, fields.at("src_nwk").int());
                assert_eq!(e.src_ieee, ieee_be(fields.at("src_ieee").str()));
                assert_eq!(e.lqi as u64, fields.at("lqi").u64());
                assert_eq!(e.rssi as f64, fields.at("rssi").num());
                assert_eq!(e.last_hop_nwk as u64, fields.at("last_hop").int());
                assert_eq!(e.flags as u64, fields.at("flags").u64());
                assert_eq!(e.key_source as u64, fields.at("key_source").u64());
                assert_eq!(e.asdu.0.len(), 8);
            }
            "nwk_permit_join" => {
                let r: NwkPermitJoin = rt(&m, &whole);
                assert_eq!(
                    r,
                    NwkPermitJoin {
                        duration_s: 254,
                        target: 0xFFFC,
                        flags: 1
                    }
                );
            }
            "evt_device_joined" => {
                let e: EvtDeviceJoined = rt(&m, &whole);
                assert_eq!(e.ieee, ieee_be("842e14fffe0a0b0c"));
                assert_eq!(
                    (e.nwk, e.parent_nwk, e.join_type, e.capability, e.decision),
                    (0x4F21, 0x1A2B, 0, 0x80, 0)
                );
            }
            "evt_route_record" => {
                let e: EvtRouteRecord = rt(&m, &whole);
                assert_eq!(e.relays.iter().collect::<Vec<_>>(), [0x3C4D, 0x1A2B]);
            }
            "sec_add_install_code" => {
                let r: SecAddInstallCode = rt(&m, &whole);
                assert_eq!(r.code.0.len(), 18);
            }
            "sec_add_install_code_rsp" => {
                let r = rt_rsp::<SecAddInstallCodeRsp>(&m, &whole);
                assert_eq!(
                    hex(&r.body.unwrap().derived_key),
                    fields.at("derived_key").str()
                );
            }
            "evt_fc_checkpoint" => {
                let e: EvtFcCheckpoint = rt(&m, &whole);
                assert_eq!((e.domain, e.key_seq, e.generation), (1, 1, 7));
            }
            "tbl_lookup_req" => {
                let r: TblLookup = rt(&m, &whole);
                assert_eq!(
                    r,
                    TblLookup {
                        table: records::record_type::DEVICE,
                        key: TableKey::Nwk(0x4F21)
                    }
                );
            }
            "tbl_lookup_rsp" => {
                let r = rt_rsp::<TblLookupRsp>(&m, &whole);
                let b = r.body.unwrap();
                assert_eq!(b.rev, 41);
                let Record::Device(d) = b.record else {
                    panic!("not a DEVICE")
                };
                let l = d.link.unwrap();
                assert_eq!(
                    (l.tx_counter, l.rx_floor, l.attrs, l.origin),
                    (14024, 3, 2, 3)
                );
            }
            "tbl_read_aps_group" => {
                let r: TblRead = rt(&m, &whole);
                assert_eq!(
                    r,
                    TblRead {
                        table: records::record_type::APS_GROUP,
                        cursor: 0,
                        max: 8
                    }
                );
            }
            "tbl_read_aps_group_rsp" => {
                let r = rt_rsp::<TblReadRsp>(&m, &whole);
                let b = r.body.unwrap();
                assert_eq!(b.next_cursor, 0xFFFF);
                let groups: Vec<(u16, Vec<u8>)> = b
                    .entries
                    .iter()
                    .map(|entry| match entry.record {
                        Record::ApsGroup(g) => (g.group, g.endpoints.to_vec()),
                        other => panic!("not an APS_GROUP: {other:?}"),
                    })
                    .collect();
                assert_eq!(groups, [(0x0B84, vec![242]), (0x1234, vec![1, 3])]);
            }
            "tbl_writeback_evict" => {
                let r: TblWriteback = rt(&m, &whole);
                assert_eq!((r.table, r.reason, r.rev), (5, tables_reason_evict(), 0x39));
            }
            "sec_lock_key_export" => {
                let r: SecLockKeyExport = rt(&m, &whole);
                assert_eq!(r.confirm, LOCK_CONFIRM);
            }
            "sec_lock_key_export_rsp" => assert_eq!(status_only(&m, &whole), Status::Success),
            "sys_set_uart" => {
                let r: SysSetUart = rt(&m, &whole);
                assert_eq!(
                    r,
                    SysSetUart {
                        baud: 921_600,
                        flow: 0,
                        apply_delay_ms: 50
                    }
                );
            }
            "evt_log_record" => {
                let e: EvtLogRecord = rt(&m, &whole);
                assert_eq!(e.time_ms as u64, fields.at("time_ms").u64());
                assert_eq!(e.level as u64, fields.at("level").u64());
                assert_eq!(e.module as u64, fields.at("module").u64());
                assert_eq!(e.seq as u64, fields.at("seq").u64());
                assert_eq!(e.text.as_str(), Some(fields.at("text").str()));
            }
            "gp_sink_config_zigpy" => {
                let r: GpSinkConfig = rt(&m, &whole);
                let built = GpSinkConfig {
                    mode: fields.at("mode").u64() as u8,
                    communication_mode: fields.at("communication_mode").u64() as u8,
                    group: fields.at("group").int() as u16,
                    exit_mode: fields.at("exit_mode").u64() as u8,
                    window_s: fields.at("window_s").u64() as u16,
                    security_level: fields.at("security_level").u64() as u8,
                    groupcast_radius: fields.at("groupcast_radius").u64() as u8,
                };
                assert_eq!(r, built);
            }
            "gp_sink_commissioning_enter" => {
                let r: GpSinkCommissioning = rt(&m, &whole);
                assert_eq!(
                    r,
                    GpSinkCommissioning {
                        action: 1,
                        flags: 1,
                        window_s: 254
                    }
                );
            }
            "evt_gp_commissioned" => {
                let e: EvtGpCommissioned = rt(&m, &whole);
                assert_eq!(e.gpd_id, GpdId::SrcId(0x293C_9879));
                let mut buf = [0u8; 64];
                let n = zigbee_zrhp::codec::encode_into(&e.record, &mut buf).unwrap();
                assert_eq!(hex(&buf[..n]), fields.at("record_hex").str());
                let Record::GpSink(s) = e.record else {
                    panic!("not R_GP_SINK")
                };
                assert_eq!(s.gpd_id, e.gpd_id);
                assert_eq!(
                    (s.communication_mode, s.group, s.security_level, s.key_type),
                    (2, 0x0B84, 3, 4)
                );
                assert!(s.key.is_some());
                assert_eq!(s.frame_counter, 0x1160);
            }
            "evt_gp_frame_toggle" => {
                let e: EvtGpFrame = rt(&m, &whole);
                assert_eq!(e.gpd_id, GpdId::SrcId(0x293C_9879));
                assert_eq!(
                    (e.origin, e.result, e.command_id, e.proxy_nwk, e.rssi),
                    (1, 1, 0x22, 0xFFFF, -48)
                );
                assert!(e.payload.0.is_empty());
            }
            "gp_pairing_configuration" => {
                let r: GpPairingConfiguration = rt(&m, &whole);
                assert_eq!(r.dst, fields.at("dst").int() as u16);
                assert_eq!(r.actions as u64, fields.at("actions").u64());
                assert_eq!(r.gpd_id, GpdId::SrcId(0x7A6B_9879));
                assert_eq!(
                    r.communication_mode as u64,
                    fields.at("communication_mode").u64()
                );
                assert_eq!(r.flags as u64, fields.at("flags").u64());
                assert_eq!((r.security_level, r.key_type), (3, 4));
                assert_eq!(r.frame_counter, 10);
                assert!(r.groups.is_empty() && r.endpoints.is_empty());
                assert_eq!(r.paired as u64, fields.at("paired").u64());
                assert_eq!(r.app_info, 0);
            }
            "gp_remote_rsp" => {
                let r: Response<GpRemoteRsp> = m.decode_response().unwrap();
                assert!(r.status.is_success());
                assert_eq!(r.body.unwrap().zcl_tsn as u64, fields.at("zcl_tsn").u64());
            }
            "gp_translation_update" => {
                let r: GpTranslationUpdate = rt(&m, &whole);
                assert_eq!(r.dst, fields.at("dst").int() as u16);
                assert_eq!(r.action as u64, fields.at("action").u64());
                assert_eq!(r.translations.len() as u64, fields.at("translations").u64());
                let t: Vec<GpTranslation> = r.translations.iter().collect();
                assert_eq!(
                    (
                        t[0].gpd_command,
                        t[0].endpoint,
                        t[0].cluster,
                        t[0].zcl_command
                    ),
                    (0x22, 1, 0x0006, 0x00)
                );
                assert_eq!(t[1].payload_len, gp_translation_payload::DERIVED);
                assert!(t[1].payload.is_empty());
                assert_eq!(t[2].payload, &[0x80, 0x0A, 0x00]);
            }
            "gp_send_pairing" => {
                let r: GpSendPairing = rt(&m, &whole);
                assert_eq!(r.dst, fields.at("dst").int() as u16);
                assert_eq!(r.flags as u64, fields.at("flags").u64());
                assert_eq!(
                    r.communication_mode as u64,
                    fields.at("communication_mode").u64()
                );
                assert_eq!(r.sink_ieee, ieee_be("0102030405060708"));
                assert_eq!((r.sink_nwk, r.frame_counter), (0x7C87, 10));
            }
            "gp_translation_request" => {
                let r: GpTranslationRequest = rt(&m, &whole);
                assert_eq!((r.dst, r.start_index), (0x7C87, 0));
            }
            "gp_remote_sink_table_request" => {
                let r: GpRemoteSinkTableRequest = rt(&m, &whole);
                assert_eq!((r.dst, r.index), (0x7C87, 0xFF));
                assert_eq!(r.gpd_id, GpdId::SrcId(0x7A6B_9879));
            }
            "evt_gp_remote_status" => {
                let e: EvtGpRemoteStatus = rt(&m, &whole);
                assert_eq!(
                    (e.src_nwk, e.zcl_tsn, e.command_id, e.zcl_status),
                    (0x7C87, 5, 0x09, 0)
                );
            }
            "evt_gp_remote_translation_table" => {
                let e: EvtGpRemoteTranslationTable = rt(&m, &whole);
                assert_eq!((e.status, e.total, e.start_index, e.flags), (0, 2, 0, 0));
                let expected = fields.at("records_hex").arr();
                assert_eq!(e.records.len(), expected.len());
                for (record, hexstr) in e.records.iter().zip(expected) {
                    let mut buf = [0u8; 64];
                    let n = zigbee_zrhp::codec::encode_into(&record, &mut buf).unwrap();
                    assert_eq!(hex(&buf[..n]), hexstr.str());
                    let Record::GpTranslation(t) = record else {
                        panic!("not R_GP_TRANSLATION")
                    };
                    assert_eq!(t.gpd_id, GpdId::SrcId(0x7A6B_9879));
                }
            }
            "evt_gp_app_description" => {
                let e: EvtGpAppDescription = rt(&m, &whole);
                assert_eq!(e.gpd_id, GpdId::SrcId(0x0BAD_CAFE));
                assert_eq!((e.total_reports, e.report_count), (2, 1));
                assert_eq!(hex(e.reports.0), fields.at("report_hex").str());
            }
            other => panic!("vector {other} has no codec test"),
        }
    }
    assert_eq!(seen, v.arr().len());
}

fn tables_reason_evict() -> u8 {
    zigbee_zrhp::msg::tables::writeback_reason::EVICT
}

// ── Records and OCB ────────────────────────────────────────────────────

fn record_rt(hexstr: &str) -> Vec<u8> {
    let raw = unhex(hexstr);
    let mut r = Reader::new(&raw);
    let rec: Record = r.read().unwrap();
    assert!(r.is_empty());
    assert!(!matches!(rec, Record::Unknown { .. }), "{hexstr}");
    let mut buf = [0u8; 128];
    let n = zigbee_zrhp::codec::encode_into(&rec, &mut buf).unwrap();
    assert_eq!(hex(&buf[..n]), hexstr);
    raw
}

fn stream_of(recs: &[Json]) -> Vec<u8> {
    let mut out = Vec::new();
    for r in recs {
        out.extend(record_rt(r.at("hex").str()));
    }
    out
}

/// Host-side OCB → records conversion (spec 9.2), built with the crate's
/// record encoders, to check the record layouts against the vectors.
fn ocb_to_records(ocb: &Json) -> Vec<u8> {
    use zigbee_zrhp::records::*;
    let internal = ocb.at("metadata").get("internal");
    let network = internal.and_then(|i| i.get("network"));
    let key16 = |s: &str| -> [u8; 16] { unhex(s).try_into().unwrap() };
    let be16 = |s: &str| u16::from_str_radix(s, 16).unwrap();
    let coordinator = ieee_be(ocb.at("coordinator_ieee").str());
    let mut recs: Vec<Record> = Vec::new();
    recs.push(Record::Node(NodeRecord {
        ieee: coordinator,
        nwk: 0,
        logical_type: 0,
        flags: 1,
    }));
    let mut mask = 0u32;
    for c in ocb.at("channel_mask").arr() {
        mask |= 1 << c.u64();
    }
    let tx_power = internal
        .and_then(|i| i.get("tx_power"))
        .map(|t| t.num() as i8)
        .unwrap_or(0x7F);
    recs.push(Record::Network(NetworkRecord {
        pan_id: be16(ocb.at("pan_id").str()),
        ext_pan_id: ieee_be(ocb.at("extended_pan_id").str()).0,
        channel: ocb.at("channel").u64() as u8,
        channel_mask: mask,
        nwk_update_id: ocb.at("nwk_update_id").u64() as u8,
        nwk_manager: network
            .and_then(|n| n.get("nwk_manager"))
            .map(|m| be16(m.str()))
            .unwrap_or(0),
        security_level: ocb.at("security_level").u64() as u8,
        tx_power,
    }));
    let nk = ocb.at("network_key");
    recs.push(Record::NetworkKey(NetworkKeyRecord {
        key: key16(nk.at("key").str()),
        seq: nk.at("sequence_number").u64() as u8,
        tx_counter: nk.at("frame_counter").u64() as u32,
        flags: 1,
    }));
    let hashed = ocb
        .get("stack_specific")
        .and_then(|s| s.get("ezsp"))
        .and_then(|e| e.get("hashed_tclk"));
    let tclk = network.and_then(|n| n.get("tc_link_key"));
    recs.push(Record::TcLinkKey(TcLinkKeyRecord {
        key: tclk
            .map(|t| key16(t.at("key").str()))
            .unwrap_or(*b"ZigBeeAlliance09"),
        tc_ieee: network
            .and_then(|n| n.get("tc_address"))
            .map(|a| ieee_be(a.str()))
            .unwrap_or(coordinator),
        tx_counter: tclk
            .and_then(|t| t.get("frame_counter"))
            .map(|c| c.u64() as u32)
            .unwrap_or(0),
        flags: u8::from(hashed.is_some()),
    }));
    if let Some(h) = hashed {
        recs.push(Record::EzspHashedTclk(key16(h.str())));
    }
    let seqs = internal.and_then(|i| i.get("link_key_seqs"));
    let mut devices: Vec<&Json> = ocb.at("devices").arr().iter().collect();
    devices.sort_by_key(|d| d.at("ieee_address").str().to_lowercase());
    for d in devices {
        let ieee_s = d.at("ieee_address").str().to_lowercase();
        let nwk = d.at("nwk_address");
        let is_child = d.get("is_child").map(Json::bool).unwrap_or(true);
        let link = d.get("link_key").map(|l| LinkBlock {
            key: key16(l.at("key").str()),
            key_seq: seqs
                .and_then(|s| s.get(&ieee_s))
                .map(|x| x.u64() as u8)
                .unwrap_or(0),
            attrs: 2,
            origin: 3,
            tx_counter: l.at("tx_counter").u64() as u32,
            rx_floor: l.at("rx_counter").u64() as u32,
        });
        let mut flags = 0;
        if is_child {
            flags |= device_flags::IS_CHILD;
        }
        if !nwk.is_null() {
            flags |= device_flags::NWK_VALID;
        }
        recs.push(Record::Device(DeviceRecord {
            ieee: ieee_be(&ieee_s),
            // Z2M writes NWK/PAN without leading zeros: pad (9.1).
            nwk: if nwk.is_null() {
                0xFFFF
            } else {
                be16(nwk.str())
            },
            flags,
            child: is_child.then_some(ChildInfo {
                capability: 0x80,
                timeout_exp: 8,
            }),
            link,
        }));
    }
    let mut buf = vec![0u8; 2048];
    let mut w = Writer::new(&mut buf);
    let mut crc = StreamCrc::new();
    for r in &recs {
        crc.write_record(r, &mut w).unwrap();
    }
    w.write(&Record::End(crc.end())).unwrap();
    let n = w.len();
    buf.truncate(n);
    buf
}

#[test]
fn ocb_roundtrip_vector() {
    use zigbee_zrhp::msg::backup::*;
    let v = load("ocb_roundtrip.json");
    let restore = v.at("restore");
    let recs = restore.at("records").arr();
    let stream = stream_of(recs);
    assert_eq!(records::verify_stream(&stream), Ok(recs.len() as u32 - 1));
    assert_eq!(
        hex(&ocb_to_records(v.at("input_ocb"))),
        hex(&stream),
        "OCB -> records"
    );

    // Restore messages.
    let msgs = restore.at("messages");
    let get = |k: &str| unhex(msgs.at(k).str());
    let begin = get("begin");
    let m = Batch::parse(&begin).unwrap().iter().next().unwrap();
    let b: BkpRestoreBegin = rt(&m, &begin);
    assert_eq!(b.flags & 1, 1);
    assert_eq!((b.nwk_margin, b.aps_margin), (10_000, 10_000));
    assert_eq!(b.expected_records as usize, recs.len());
    let rsp = get("begin_rsp");
    let m = Batch::parse(&rsp).unwrap().iter().next().unwrap();
    let r = rt_rsp::<BkpRestoreBeginRsp>(&m, &rsp).body.unwrap();
    assert_eq!(r.txn, 1);
    let put = get("put");
    let m = Batch::parse(&put).unwrap().iter().next().unwrap();
    let p: BkpRestorePut = rt(&m, &put);
    assert_eq!(p.txn, 1);
    assert_eq!(p.records.len(), recs.len());
    let List::Raw { bytes, .. } = p.records else {
        panic!()
    };
    assert_eq!(bytes, &stream[..]);
    let rsp = get("put_rsp");
    let m = Batch::parse(&rsp).unwrap().iter().next().unwrap();
    let r = rt_rsp::<BkpRestorePutRsp>(&m, &rsp).body.unwrap();
    assert_eq!(r.accepted as usize, recs.len());
    assert!(r.notes.is_empty());
    let commit = get("commit");
    let m = Batch::parse(&commit).unwrap().iter().next().unwrap();
    let c: BkpRestoreCommit = rt(&m, &commit);
    assert_eq!(c.txn, 1);
    let rsp = get("commit_rsp");
    let m = Batch::parse(&rsp).unwrap().iter().next().unwrap();
    let cr = rt_rsp::<BkpRestoreCommitRsp>(&m, &rsp).body.unwrap();

    // Counter rule of BKP_RESTORE_COMMIT.
    let mut rc = RestoreCounters::new();
    for item in records::RawRecords::new(&stream) {
        let (_, tlv) = item.unwrap();
        rc.add(&Reader::new(tlv).read().unwrap());
    }
    let st = restore.at("ncp_state");
    assert_eq!(rc.nwk_start(10_000) as u64, st.at("nwk_fc_start").u64());
    assert_eq!(rc.aps_start(10_000) as u64, st.at("aps_fc_start").u64());
    assert_eq!(cr.nwk_fc_start, rc.nwk_start(10_000));
    assert_eq!(cr.aps_fc_start, rc.aps_start(10_000));
    assert_eq!(cr.children as u64, st.at("children").u64());
    assert_eq!(cr.link_keys_on_ncp as u64, st.at("link_keys_on_ncp").u64());
    assert_eq!(
        cr.link_keys_host_backed as u64,
        st.at("link_keys_host_backed").u64()
    );
    let stride = 1024;
    assert_eq!(
        (rc.nwk_start(10_000) + stride) as u64,
        st.at("nwk_fc_limit").u64()
    );
    assert_eq!(
        (rc.aps_start(10_000) + stride) as u64,
        st.at("aps_fc_limit").u64()
    );

    // Export: round trip, END, durable limits everywhere, zero-margin restore.
    let export = v.at("export");
    let erecs = export.at("records").arr();
    let estream = stream_of(erecs);
    assert_eq!(records::verify_stream(&estream), Ok(erecs.len() as u32 - 1));
    let mut rc2 = RestoreCounters::new();
    for item in records::RawRecords::new(&estream) {
        let (_, tlv) = item.unwrap();
        let rec: Record = Reader::new(tlv).read().unwrap();
        match &rec {
            Record::Device(d) => {
                if let Some(l) = &d.link {
                    assert_eq!(l.tx_counter as u64, st.at("aps_fc_limit").u64());
                }
            }
            Record::NetworkKey(k) => assert_eq!(k.tx_counter as u64, st.at("nwk_fc_limit").u64()),
            Record::TcLinkKey(k) => assert_eq!(k.tx_counter as u64, st.at("aps_fc_limit").u64()),
            Record::RsParams(p) => {
                assert_eq!(p.table_epoch as u64, export.at("table_epoch").u64());
                assert_eq!(p.fc_stride, stride);
            }
            _ => {}
        }
        rc2.add(&rec);
    }
    assert_eq!(rc2.nwk_start(0) as u64, st.at("nwk_fc_limit").u64());
    assert_eq!(rc2.aps_start(0) as u64, st.at("aps_fc_limit").u64());
    // Output OCB converted back gives byte-identical DEVICE records.
    let out = ocb_to_records(v.at("output_ocb"));
    let devs = |s: &[u8]| -> Vec<Vec<u8>> {
        records::RawRecords::new(s)
            .map(Result::unwrap)
            .filter(|(t, _)| *t == records::record_type::DEVICE)
            .map(|(_, tlv)| tlv.to_vec())
            .collect()
    };
    assert_eq!(devs(&out), devs(&estream));
}

#[test]
fn ocb_z2m_import_vector() {
    let v = load("ocb_z2m_import.json");
    let recs = v.at("records").arr();
    let stream = stream_of(recs);
    assert_eq!(records::verify_stream(&stream), Ok(recs.len() as u32 - 1));
    assert_eq!(hex(&ocb_to_records(v.at("input_ocb"))), hex(&stream));
    let mut rc = RestoreCounters::new();
    for item in records::RawRecords::new(&stream) {
        rc.add(&Reader::new(item.unwrap().1).read().unwrap());
    }
    assert_eq!(
        v.at("aps_margin").u64(),
        records::APS_UNKNOWN_TCLK_MARGIN as u64
    );
    let st = v.at("ncp_state");
    assert_eq!(
        rc.nwk_start(v.at("nwk_margin").u64() as u32) as u64,
        st.at("nwk_fc_start").u64()
    );
    assert_eq!(
        rc.aps_start(records::APS_UNKNOWN_TCLK_MARGIN) as u64,
        st.at("aps_fc_start").u64()
    );
}

#[test]
fn every_message_id_has_a_home_endpoint() {
    for &id in msg::ids::REQUESTS.iter().chain(msg::ids::EVENTS) {
        let ep = msg::home_endpoint(id).unwrap();
        assert_ne!(ep, Endpoint::Link);
        assert_eq!(
            msg::is_event_id(id),
            msg::ids::EVENTS.contains(&id),
            "{id:#06x}"
        );
    }
}
