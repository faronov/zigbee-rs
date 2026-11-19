//! Minimal ZRHP v1 host for NCP bring-up.
//!
//! The byte stream goes through hwlab (`hwlab invoke uart/<port>
//! SerialTransport.read|write`), so every hardware access stays under the
//! lab's control; the link runs on the shared `zigbee-zrhp` crate with the
//! desktop-host profile and the wall clock.
//!
//! ```text
//! zrhp-host --hwlab uart/cu.usbmodem14601 smoke          # handshake + read-only commands
//! zrhp-host --hwlab uart/cu.usbmodem14601 smoke --form   # also NWK_FORM / NWK_INIT
//! zrhp-host --serial /dev/cu.usbmodem7CDFA1FE67F01 --baud 460800 smoke
//! ```
//!
//! `--serial` opens the port directly with DTR and RTS asserted (the Thread
//! BR bridge maps the esptool DTR/RTS reset; both asserted keeps the H2
//! running).
//!
//! `smoke` never opens the network for joining and never transmits user
//! frames on air except what formation itself sends. Optional steps after
//! the smoke sequence:
//!
//! ```text
//! --backup FILE    BKP_EXPORT (keys + devices) into FILE (OCB record stream)
//! --restore FILE   BKP_RESTORE of FILE (network must not be running:
//!                  after a reset, without --form); margins 10 000/10 000
//! --tables         TBL_STATS / TBL_READ / TBL_SYNC of the DEVICE table
//! --fw-slots       FW_GET_SLOTS
//! --fw-update FILE --fw-version 0xMMmmrrrr
//!                  FW_BEGIN/WRITE/FINISH/ACTIVATE of FILE (format 1); the
//!                  NCP reboots into the trial image
//! --fw-confirm     FW_CONFIRM (after the trial image resumed: use --form)
//! --join SECS      open NWK_PERMIT_JOIN on the NCP only for at most SECS,
//!                  close it at the first admitted join, interview the
//!                  device (Active_EP, Basic attributes), bind its
//!                  measurement clusters to the NCP and print every event
//!                  and indication until SECS elapsed
//! --listen SECS --device 0xNWK [--ep N]
//!                  read the device's Basic attributes and print every
//!                  event and indication for SECS (no join window)
//! ```

use std::process::Command;
use std::time::{Duration, Instant};

use zigbee_zrhp::codec::{Bytes8, Rest, Wire};
use zigbee_zrhp::frame::{ResetReason, TX_BUFFER_LEN};
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent, LinkState};
use zigbee_zrhp::msg::backup::{
    BkpExportBegin, BkpExportBeginRsp, BkpExportEnd, BkpExportRead, BkpRestoreBegin,
    BkpRestoreBeginRsp, BkpRestoreCommit, BkpRestoreCommitRsp, BkpRestorePutRsp,
};
use zigbee_zrhp::msg::data::{ApsDataRequest, ApsDst};
use zigbee_zrhp::msg::diag::{DiagGetCounters, DiagGetCountersRsp};
use zigbee_zrhp::msg::firmware::{
    FwActivate, FwBegin, FwBeginRsp, FwFinishRsp, FwGetSlotsRsp, FwWrite, FwWriteRsp,
};
use zigbee_zrhp::msg::network::{
    EvtNetworkState, NwkForm, NwkGetInfoRsp, NwkInit, NwkInitRsp, NwkPermitJoin,
};
use zigbee_zrhp::msg::security::SecGetFrameCountersRsp;
use zigbee_zrhp::msg::system::{
    EvtNcpReady, SysEcho, SysGetInfoRsp, SysGetStatusRsp, SysNegotiateReq, SysNegotiateRsp,
};
use zigbee_zrhp::msg::tables::{TblRead, TblReadRsp, TblStats, TblStatsRsp, TblSync, TblSyncRsp};
use zigbee_zrhp::msg::{Batch, Kind, MessageDef, MsgHeader, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::records::{RawRecords, Record, record_type, verify_stream};
use zigbee_zrhp::types::{Endpoint, Status};

/// The byte stream to the NCP.
trait ByteStream {
    fn write(&mut self, data: &[u8]) -> Result<(), String>;
    /// Bytes received within `timeout`.
    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, String>;
}

/// Serial port through `hwlab invoke` (one process per call).
struct Hwlab {
    device: String,
}

impl Hwlab {
    fn invoke(
        &self,
        method: &str,
        args: &[String],
        confirm: bool,
    ) -> Result<serde_json::Value, String> {
        let hwlab = std::env::var("HWLAB").unwrap_or_else(|_| {
            format!(
                "{}/hwlab/bin/hwlab",
                std::env::var("HOME").unwrap_or_default()
            )
        });
        let mut command = Command::new(hwlab);
        command.args(["--json", "invoke", &self.device, method]);
        for arg in args {
            command.args(["-a", arg]);
        }
        if confirm {
            command.arg("--yes");
        }
        let output = command.output().map_err(|e| e.to_string())?;
        if !output.status.success() {
            return Err(String::from_utf8_lossy(&output.stderr).into_owned());
        }
        serde_json::from_slice(&output.stdout).map_err(|e| e.to_string())
    }
}

impl ByteStream for Hwlab {
    fn write(&mut self, data: &[u8]) -> Result<(), String> {
        for chunk in data.chunks(4096) {
            self.invoke(
                "SerialTransport.write",
                &[format!("hex=\"{}\"", hex(chunk))],
                true,
            )?;
        }
        Ok(())
    }

    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, String> {
        let mut out = Vec::new();
        loop {
            let value = self.invoke(
                "SerialTransport.read",
                &[
                    format!("timeout_ms={}", timeout.as_millis()),
                    "idle_gap_ms=30".into(),
                    "max_bytes=256".into(),
                ],
                false,
            )?;
            let n = value["n"].as_u64().unwrap_or(0) as usize;
            let bytes = unhex(value["hex"].as_str().unwrap_or(""));
            out.extend_from_slice(&bytes);
            if n < 256 || bytes.len() < n {
                return Ok(out);
            }
        }
    }
}

/// A serial port opened directly (DTR = RTS = 1, no reset).
struct Serial {
    port: Box<dyn serialport::SerialPort>,
}

impl Serial {
    fn open(path: &str, baud: u32) -> Result<Self, String> {
        let mut port = serialport::new(path, baud)
            .timeout(Duration::from_millis(20))
            .open()
            .map_err(|e| e.to_string())?;
        port.write_data_terminal_ready(true)
            .map_err(|e| e.to_string())?;
        port.write_request_to_send(true)
            .map_err(|e| e.to_string())?;
        Ok(Self { port })
    }
}

impl ByteStream for Serial {
    fn write(&mut self, data: &[u8]) -> Result<(), String> {
        use std::io::Write;
        self.port.write_all(data).map_err(|e| e.to_string())
    }

    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, String> {
        use std::io::Read;
        let deadline = Instant::now() + timeout;
        let mut out = Vec::new();
        let mut buf = [0u8; 1024];
        loop {
            match self.port.read(&mut buf) {
                Ok(n) => out.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(e.to_string()),
            }
            if !out.is_empty() && self.port.bytes_to_read().unwrap_or(0) == 0 {
                return Ok(out);
            }
            if Instant::now() >= deadline {
                return Ok(out);
            }
        }
    }
}

impl ByteStream for Box<dyn ByteStream> {
    fn write(&mut self, data: &[u8]) -> Result<(), String> {
        if std::env::var_os("ZRHP_TRACE").is_some() {
            eprintln!(
                "tx {:>4} B {}",
                data.len(),
                hex(&data[..data.len().min(24)])
            );
        }
        (**self).write(data)
    }
    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, String> {
        let data = (**self).read(timeout)?;
        if std::env::var_os("ZRHP_TRACE").is_some() && !data.is_empty() {
            eprintln!(
                "rx {:>4} B {}",
                data.len(),
                hex(&data[..data.len().min(24)])
            );
        }
        Ok(data)
    }
}

fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(text: &str) -> Vec<u8> {
    (0..text.len() / 2)
        .filter_map(|i| u8::from_str_radix(&text[2 * i..2 * i + 2], 16).ok())
        .collect()
}

#[derive(Debug, Clone)]
struct Received {
    kind: Kind,
    msg_id: u16,
    tid: u16,
    payload: Vec<u8>,
}

impl Received {
    fn raw(&self) -> RawMessage<'_> {
        RawMessage {
            header: MsgHeader {
                kind: self.kind,
                retry: false,
                msg_id: self.msg_id,
                tid: self.tid,
                len: self.payload.len() as u16,
            },
            payload: &self.payload,
        }
    }

    fn status(&self) -> Status {
        Status::from_u8(self.payload.first().copied().unwrap_or(0xFF))
    }
}

struct Host<S: ByteStream> {
    stream: S,
    link: Link<32, 32>,
    queues: EndpointQueues<4096>,
    tid: u16,
    inbox: Vec<Received>,
    epoch: Instant,
    junk: usize,
}

impl<S: ByteStream> Host<S> {
    fn new(stream: S) -> Self {
        let epoch = Instant::now();
        let cookie = (std::process::id() ^ 0x5A5A_0000) | 1;
        let mut link = Link::new(LinkConfig::desktop_host(), cookie);
        link.open(0, ResetReason::HostOpen);
        Self {
            stream,
            link,
            queues: EndpointQueues::new(),
            tid: 0,
            inbox: Vec::new(),
            epoch,
            junk: 0,
        }
    }

    fn now(&self) -> u64 {
        self.epoch.elapsed().as_micros() as u64
    }

    /// One round: transmit what the link has, then read for `wait`.
    fn round(&mut self, wait: Duration) -> Result<(), String> {
        let mut tx = [0u8; TX_BUFFER_LEN];
        let mut out = Vec::new();
        let now = self.now();
        while let Some(n) = self.link.poll_transmit(now, &mut self.queues, &mut tx) {
            out.extend_from_slice(&tx[..n]);
        }
        if !out.is_empty() {
            self.stream.write(&out)?;
        }
        let bytes = self.stream.read(wait)?;
        let now = self.now();
        let mut offset = 0;
        while offset < bytes.len() {
            offset += self.link.push_rx(&bytes[offset..], now);
            while let Some(event) = self.link.poll_event() {
                match event {
                    LinkEvent::Data { payload, .. } => match Batch::parse(payload) {
                        Ok(batch) => {
                            for m in batch.iter() {
                                self.inbox.push(Received {
                                    kind: m.header.kind,
                                    msg_id: m.header.msg_id,
                                    tid: m.header.tid,
                                    payload: m.payload.to_vec(),
                                });
                            }
                        }
                        Err(error) => eprintln!("bad batch: {error:?}"),
                    },
                    LinkEvent::Established(info) => {
                        println!(
                            "link: established epoch {} NCP cookie 0x{:08X}",
                            info.epoch, info.peer_cookie
                        )
                    }
                    LinkEvent::PeerReset { cookie, change } => {
                        println!("link: peer reset 0x{cookie:08X} {change:?}")
                    }
                    LinkEvent::LinkDown => println!("link: down"),
                    LinkEvent::LinkFailed => println!("link: failed"),
                    LinkEvent::TxFailed { reason, .. } => println!("link: tx failed {reason:?}"),
                }
            }
        }
        self.junk = self.link.counters().rx_bad_frame as usize;
        Ok(())
    }

    fn request<'a, T: Wire<'a> + MessageDef>(&mut self, body: &T) -> u16 {
        self.tid = self.tid.wrapping_add(1).max(1);
        self.queues.request(self.tid, body).expect("queue");
        self.tid
    }

    fn request_empty(&mut self, msg_id: u16) -> u16 {
        self.tid = self.tid.wrapping_add(1).max(1);
        let endpoint = zigbee_zrhp::msg::home_endpoint(msg_id).unwrap_or(Endpoint::System);
        self.queues
            .queue(endpoint)
            .raw(Kind::Request, msg_id, self.tid, &[])
            .expect("queue");
        self.tid
    }

    fn wait_for(
        &mut self,
        what: &str,
        timeout: Duration,
        pred: impl Fn(&Received) -> bool,
    ) -> Result<Received, String> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(i) = self.inbox.iter().position(&pred) {
                return Ok(self.inbox.remove(i));
            }
            if Instant::now() >= deadline {
                return Err(format!("timeout waiting for {what}"));
            }
            self.round(Duration::from_millis(150))?;
        }
    }

    fn response(&mut self, tid: u16) -> Result<Received, String> {
        self.response_within(tid, Duration::from_secs(10))
    }

    fn response_within(&mut self, tid: u16, timeout: Duration) -> Result<Received, String> {
        self.wait_for(&format!("response tid {tid}"), timeout, |m| {
            m.kind == Kind::Response && m.tid == tid
        })
    }

    fn request_raw(&mut self, msg_id: u16, payload: &[u8]) -> u16 {
        self.tid = self.tid.wrapping_add(1).max(1);
        let endpoint = zigbee_zrhp::msg::home_endpoint(msg_id).unwrap_or(Endpoint::System);
        self.queues
            .queue(endpoint)
            .raw(Kind::Request, msg_id, self.tid, payload)
            .expect("queue");
        self.tid
    }

    fn event(&mut self, msg_id: u16, timeout: Duration) -> Result<Received, String> {
        self.wait_for(&format!("event 0x{msg_id:04X}"), timeout, |m| {
            m.kind == Kind::Event && m.msg_id == msg_id
        })
    }
}

fn body<'a, T: Wire<'a>>(message: &'a Received) -> Result<T, String> {
    let response = message
        .raw()
        .decode_response::<T>()
        .map_err(|e| format!("decode: {e:?}"))?;
    if !response.status.is_success() {
        return Err(format!("status {:?}", response.status));
    }
    response.body.ok_or_else(|| "empty body".into())
}

/// `floors`: `NWK_INIT` counter floors `(nwk, aps)` for a resume.
fn smoke<S: ByteStream>(host: &mut Host<S>, form: bool, floors: (u32, u32)) -> Result<(), String> {
    let started = Instant::now();
    let ready = host.event(ids::EVT_NCP_READY, Duration::from_secs(15))?;
    let ready: EvtNcpReady = ready.raw().decode().map_err(|e| format!("{e:?}"))?;
    println!(
        "EVT_NCP_READY after {:?}: cookie 0x{:08X} reset_reason {} network_state {} nwk_fc_limit {} aps_fc_limit {}",
        started.elapsed(),
        ready.ncp_cookie,
        ready.reset_reason,
        ready.network_state,
        ready.nwk_fc_limit,
        ready.aps_fc_limit
    );
    let tid = host.request(&SysNegotiateReq {
        major_min: 1,
        major_max: 1,
        host_caps: 0,
        name: Bytes8(b"zrhp-host"),
    });
    let rsp = host.response(tid)?;
    let negotiated: SysNegotiateRsp = body(&rsp)?;
    println!(
        "SYS_NEGOTIATE: v{}.{} caps 0x{:016X} fw {:?} chip {} capacities {:?}",
        negotiated.major,
        negotiated.minor,
        negotiated.ncp_caps,
        negotiated.fw.as_str(),
        negotiated.chip,
        negotiated
            .capacities
            .iter()
            .map(|c| (c.id, c.value))
            .collect::<Vec<_>>()
    );
    let tid = host.request_empty(ids::SYS_GET_INFO);
    let rsp = host.response(tid)?;
    let info: SysGetInfoRsp = body(&rsp)?;
    println!(
        "SYS_GET_INFO: ieee {:02X?} flash {} transport {} board {:?} product {:?}",
        info.ieee_active.0,
        info.flash_size,
        info.transport,
        info.board.as_str(),
        info.product.as_str()
    );
    // Echo round-trip latency over 10 requests.
    let mut worst = Duration::ZERO;
    for i in 0..10u8 {
        let sent = Instant::now();
        let tid = host.request(&SysEcho {
            data: Rest(&[i; 32]),
        });
        let rsp = host.response(tid)?;
        if rsp.payload.get(1..) != Some(&[i; 32][..]) {
            return Err("echo mismatch".into());
        }
        worst = worst.max(sent.elapsed());
    }
    println!("SYS_ECHO x10 ok, worst round trip {worst:?} (includes hwlab process spawn)");
    let tid = host.request_empty(ids::SYS_GET_STATUS);
    let rsp = host.response(tid)?;
    let status: SysGetStatusRsp = body(&rsp)?;
    println!(
        "SYS_GET_STATUS: network {} uptime {} s free_aps {} flags 0x{:02X}",
        status.network_state, status.uptime_s, status.free_aps_slots, status.flags
    );
    if form && ready.network_state == 0 {
        let tid = host.request(&NwkForm {
            channel_mask: 0x07FF_F800,
            pan_id: 0xFFFF,
            ext_pan_id: [0; 8],
            network_key: [0; 16],
            key_seq: 0,
            nwk_update_id: 0,
            tx_power: 0,
            nwk_manager: 0,
        });
        let rsp = host.response(tid)?;
        println!("NWK_FORM: {:?}", rsp.status());
        let state = host.event(ids::EVT_NETWORK_STATE, Duration::from_secs(30))?;
        let state: EvtNetworkState = state.raw().decode().map_err(|e| format!("{e:?}"))?;
        println!(
            "EVT_NETWORK_STATE: state {} zb_status 0x{:02X}",
            state.state, state.zb_status
        );
    } else if form {
        let tid = host.request(&NwkInit {
            nwk_fc_floor: floors.0,
            aps_fc_floor: floors.1,
        });
        let rsp = host.response(tid)?;
        let init: NwkInitRsp = body(&rsp)?;
        println!("NWK_INIT: network_state {}", init.network_state);
        let state = host.event(ids::EVT_NETWORK_STATE, Duration::from_secs(30))?;
        let state: EvtNetworkState = state.raw().decode().map_err(|e| format!("{e:?}"))?;
        println!(
            "EVT_NETWORK_STATE: state {} zb_status 0x{:02X}",
            state.state, state.zb_status
        );
    }
    let tid = host.request_empty(ids::NWK_GET_INFO);
    let rsp = host.response(tid)?;
    let nwk: NwkGetInfoRsp = body(&rsp)?;
    println!(
        "NWK_GET_INFO: state {} pan 0x{:04X} epid {:02X?} channel {} short 0x{:04X} key_seq {}",
        nwk.state, nwk.pan_id, nwk.ext_pan_id, nwk.channel, nwk.short, nwk.key_seq
    );
    let tid = host.request_empty(ids::SEC_GET_FRAME_COUNTERS);
    let rsp = host.response(tid)?;
    let fc: SecGetFrameCountersRsp = body(&rsp)?;
    println!(
        "SEC_GET_FRAME_COUNTERS: nwk {}/{} aps {}/{} stride {}",
        fc.nwk_current, fc.nwk_limit, fc.aps_current, fc.aps_limit, fc.stride
    );
    while let Some(i) = host
        .inbox
        .iter()
        .position(|m| m.kind == Kind::Event && m.msg_id == ids::EVT_LOG_RECORD)
    {
        let record = host.inbox.remove(i);
        if let Ok(log) = record.raw().decode::<zigbee_zrhp::msg::log::EvtLogRecord>() {
            println!(
                "LOG [{} ms L{}] {}",
                log.time_ms,
                log.level,
                log.text.as_str().unwrap_or("<invalid UTF-8>")
            );
        }
    }
    let tid = host.request(&DiagGetCounters { group_mask: 0x41 });
    let rsp = host.response(tid)?;
    let counters: DiagGetCountersRsp = body(&rsp)?;
    let nonzero: Vec<_> = counters
        .counters
        .iter()
        .filter(|c| c.value != 0)
        .map(|c| format!("0x{:04X}={}", c.id, c.value))
        .collect();
    println!("NCP counters (non-zero): {}", nonzero.join(" "));
    let own = host.link.counters();
    println!(
        "host link: tx {} retx {} rx {} bad {} dup {} state {:?}",
        own.tx_frames,
        own.tx_retransmissions,
        own.rx_frames,
        own.rx_bad_frame,
        own.rx_duplicate,
        host.link.state()
    );
    if host.link.state() != LinkState::Established {
        return Err("link not established at the end".into());
    }
    Ok(())
}

fn record_name(kind: u8) -> &'static str {
    match kind {
        record_type::NODE => "NODE",
        record_type::NETWORK => "NETWORK",
        record_type::NETWORK_KEY => "NETWORK_KEY",
        record_type::TC_LINK_KEY => "TC_LINK_KEY",
        record_type::DEVICE => "DEVICE",
        record_type::RS_PARAMS => "RS_PARAMS",
        record_type::END => "END",
        _ => "other",
    }
}

/// `BKP_EXPORT` with keys and devices into `path`.
fn backup_export<S: ByteStream>(host: &mut Host<S>, path: &str) -> Result<(), String> {
    let tid = host.request(&BkpExportBegin { flags: 0x03 });
    let rsp = host.response(tid)?;
    let begin: BkpExportBeginRsp = body(&rsp)?;
    let mut stream = Vec::new();
    loop {
        let tid = host.request(&BkpExportRead {
            txn: begin.txn,
            max_bytes: 400,
        });
        let rsp = host.response(tid)?;
        if !rsp.status().is_success() || rsp.payload.len() < 2 {
            return Err(format!("BKP_EXPORT_READ status {:?}", rsp.status()));
        }
        stream.extend_from_slice(&rsp.payload[2..]);
        if rsp.payload[1] == 0 {
            break;
        }
    }
    let tid = host.request(&BkpExportEnd { txn: begin.txn });
    host.response(tid)?;
    let count = verify_stream(&stream).map_err(|e| format!("export stream: {e:?}"))?;
    let kinds: Vec<&str> = RawRecords::new(&stream)
        .filter_map(Result::ok)
        .map(|(kind, _)| record_name(kind))
        .collect();
    std::fs::write(path, &stream).map_err(|e| format!("{path}: {e}"))?;
    println!(
        "BKP_EXPORT: {} records announced, {count} + END verified, {} bytes -> {path}: {}",
        begin.record_count,
        stream.len(),
        kinds.join(" ")
    );
    Ok(())
}

/// `BKP_RESTORE` of the record stream in `path`.
fn backup_restore<S: ByteStream>(host: &mut Host<S>, path: &str) -> Result<(), String> {
    let stream = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    verify_stream(&stream).map_err(|e| format!("{path}: {e:?}"))?;
    let records: Vec<&[u8]> = RawRecords::new(&stream)
        .map(|r| r.map(|(_, tlv)| tlv))
        .collect::<Result<_, _>>()
        .map_err(|e| format!("{e:?}"))?;
    let tid = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 10_000,
        aps_margin: 10_000,
        expected_records: records.len() as u32,
    });
    let rsp = host.response(tid)?;
    let begin: BkpRestoreBeginRsp = body(&rsp)?;
    for chunk in records.chunks(usize::from(begin.max_records_per_put.max(1))) {
        let mut payload = begin.txn.to_le_bytes().to_vec();
        for record in chunk {
            payload.extend_from_slice(record);
        }
        let tid = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
        let rsp = host.response(tid)?;
        let put: BkpRestorePutRsp = body(&rsp)?;
        let notes: Vec<_> = put.notes.iter().map(|n| (n.index, n.note)).collect();
        println!("BKP_RESTORE_PUT: accepted {} notes {notes:?}", put.accepted);
    }
    let tid = host.request(&BkpRestoreCommit { txn: begin.txn });
    let rsp = host.response_within(tid, Duration::from_secs(30))?;
    let commit: BkpRestoreCommitRsp = body(&rsp)?;
    println!(
        "BKP_RESTORE_COMMIT: nwk_fc_start {} aps_fc_start {} keys on NCP {} host-backed {}",
        commit.nwk_fc_start,
        commit.aps_fc_start,
        commit.link_keys_on_ncp,
        commit.link_keys_host_backed
    );
    let tid = host.request_empty(ids::NWK_GET_INFO);
    let rsp = host.response(tid)?;
    let nwk: NwkGetInfoRsp = body(&rsp)?;
    println!(
        "NWK_GET_INFO: state {} pan 0x{:04X} epid {:02X?} channel {} key_seq {}",
        nwk.state, nwk.pan_id, nwk.ext_pan_id, nwk.channel, nwk.key_seq
    );
    Ok(())
}

/// `TBL_STATS`, `TBL_SYNC` and every `TBL_READ` page of the DEVICE table.
fn tables<S: ByteStream>(host: &mut Host<S>) -> Result<(), String> {
    let tid = host.request(&TblStats {
        table: record_type::DEVICE,
    });
    let rsp = host.response(tid)?;
    let stats: TblStatsRsp = body(&rsp)?;
    println!(
        "TBL_STATS: capacity {} used {} pinned {}",
        stats.capacity, stats.used, stats.pinned
    );
    let tid = host.request(&TblSync {
        host_db_epoch: 0,
        last_ncp_table_epoch: 0,
        host_max_rev: 0,
        flags: 0,
    });
    let rsp = host.response(tid)?;
    let sync: TblSyncRsp = body(&rsp)?;
    println!(
        "TBL_SYNC: epoch 0x{:08X} cached devices {}",
        sync.ncp_table_epoch, sync.cached_devices
    );
    let mut cursor = 0u16;
    let mut total = 0usize;
    while cursor != 0xFFFF {
        let tid = host.request(&TblRead {
            table: record_type::DEVICE,
            cursor,
            max: 8,
        });
        let rsp = host.response(tid)?;
        let read: TblReadRsp = body(&rsp)?;
        for entry in read.entries.iter() {
            if let Record::Device(device) = &entry.record {
                println!(
                    "  DEVICE {:02X?} nwk 0x{:04X} child {} key {}",
                    device.ieee.0,
                    device.nwk,
                    device.child.is_some(),
                    device.link.is_some()
                );
            }
            total += 1;
        }
        cursor = read.next_cursor;
    }
    println!("TBL_READ: {total} entries");
    Ok(())
}

fn fw_slots<S: ByteStream>(host: &mut Host<S>) -> Result<(), String> {
    let tid = host.request_empty(ids::FW_GET_SLOTS);
    let rsp = host.response(tid)?;
    let slots: FwGetSlotsRsp = body(&rsp)?;
    for slot in slots.slots.iter() {
        println!(
            "FW slot {}: state {} version 0x{:08X} sha256 {:02X?}..",
            slot.slot,
            slot.state,
            slot.version,
            &slot.sha256[..4]
        );
    }
    Ok(())
}

/// Stage `path` in the inactive slot and activate it.
fn fw_update<S: ByteStream>(host: &mut Host<S>, path: &str, version: u32) -> Result<(), String> {
    let image = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let digest = Command::new("shasum")
        .args(["-a", "256", path])
        .output()
        .map_err(|e| format!("shasum: {e}"))?;
    let hex = String::from_utf8_lossy(&digest.stdout);
    let hex = hex.split_whitespace().next().unwrap_or_default();
    let mut sha256 = [0u8; 32];
    for (index, byte) in sha256.iter_mut().enumerate() {
        *byte = u8::from_str_radix(hex.get(index * 2..index * 2 + 2).unwrap_or("zz"), 16)
            .map_err(|_| "bad shasum output".to_string())?;
    }
    let started = Instant::now();
    let tid = host.request(&FwBegin {
        image_size: image.len() as u32,
        format: 1,
        sha256,
        version,
        flags: 0,
    });
    let rsp = host.response(tid)?;
    let begin: FwBeginRsp = body(&rsp)?;
    println!(
        "FW_BEGIN: {} bytes version 0x{version:08X} -> slot {} (max_chunk {})",
        image.len(),
        begin.slot,
        begin.max_chunk
    );
    let mut offset = 0u32;
    for chunk in image.chunks(usize::from(begin.max_chunk.max(1))) {
        let tid = host.request(&FwWrite {
            offset,
            data: Rest(chunk),
        });
        let rsp = host.response(tid)?;
        let write: FwWriteRsp = body(&rsp)?;
        offset = write.next_offset;
    }
    println!("FW_WRITE: {offset} bytes in {:?}", started.elapsed());
    let tid = host.request_empty(ids::FW_FINISH);
    let rsp = host.response_within(tid, Duration::from_secs(30))?;
    let finish: FwFinishRsp = body(&rsp)?;
    println!("FW_FINISH: verify_result {}", finish.verify_result);
    fw_slots(host)?;
    let tid = host.request(&FwActivate { delay_ms: 200 });
    let rsp = host.response(tid)?;
    if !rsp.status().is_success() {
        return Err(format!("FW_ACTIVATE {:?}", rsp.status()));
    }
    println!("FW_ACTIVATE: rebooting into the trial image");
    Ok(())
}

/// A ZCL/ZDO conversation with one joined device.
struct Peer {
    nwk: u16,
    ieee: [u8; 8],
    ep: Option<u8>,
    seq: u8,
}

fn data_request<S: ByteStream>(
    host: &mut Host<S>,
    nwk: u16,
    dst_ep: u8,
    src_ep: u8,
    profile: u16,
    cluster: u16,
    asdu: &[u8],
) -> u16 {
    host.request(&ApsDataRequest {
        dst: ApsDst::Nwk(nwk),
        dst_ep,
        src_ep,
        profile,
        cluster,
        // APS ACK, extended timeout (sleepy destination).
        tx_options: 0x21,
        radius: 0,
        non_member_radius: 0,
        priority: 1,
        relays: None,
        alias: None,
        asdu,
    })
}

/// ZCL Read Attributes of Basic: manufacturer, model, date code, SW build.
fn read_basic<S: ByteStream>(host: &mut Host<S>, peer: &mut Peer) {
    let Some(ep) = peer.ep else { return };
    peer.seq = peer.seq.wrapping_add(1);
    let mut zcl = vec![0x00, peer.seq, 0x00];
    for attr in [0x0004u16, 0x0005, 0x0006, 0x4000] {
        zcl.extend_from_slice(&attr.to_le_bytes());
    }
    data_request(host, peer.nwk, ep, 1, 0x0104, 0x0000, &zcl);
    println!("-> ZCL Read Attributes Basic to 0x{:04X} ep {ep}", peer.nwk);
}

/// ZDO Bind_req: `cluster` of the device to the NCP (IEEE, endpoint 1).
fn bind_to_ncp<S: ByteStream>(host: &mut Host<S>, peer: &mut Peer, own: [u8; 8], cluster: u16) {
    let Some(ep) = peer.ep else { return };
    peer.seq = peer.seq.wrapping_add(1);
    let mut zdo = vec![peer.seq];
    zdo.extend_from_slice(&peer.ieee);
    zdo.push(ep);
    zdo.extend_from_slice(&cluster.to_le_bytes());
    zdo.push(0x03);
    zdo.extend_from_slice(&own);
    zdo.push(1);
    data_request(host, peer.nwk, 0, 0, 0x0000, 0x0021, &zdo);
    println!("-> ZDO Bind_req 0x{cluster:04X} to the NCP");
}

/// ZCL Configure Reporting of the measurement attributes (10 s .. 60 s).
fn configure_reporting<S: ByteStream>(host: &mut Host<S>, peer: &mut Peer) {
    let Some(ep) = peer.ep else { return };
    // (cluster, attribute, type, reportable change)
    let configs: [(u16, u16, u8, &[u8]); 3] = [
        (0x0402, 0x0000, 0x29, &10i16.to_le_bytes()),
        (0x0405, 0x0000, 0x21, &10u16.to_le_bytes()),
        (0x0001, 0x0021, 0x20, &[1]),
    ];
    for (cluster, attribute, data_type, change) in configs {
        peer.seq = peer.seq.wrapping_add(1);
        let mut zcl = vec![0x00, peer.seq, 0x06, 0x00];
        zcl.extend_from_slice(&attribute.to_le_bytes());
        zcl.push(data_type);
        zcl.extend_from_slice(&10u16.to_le_bytes());
        zcl.extend_from_slice(&60u16.to_le_bytes());
        zcl.extend_from_slice(change);
        data_request(host, peer.nwk, ep, 1, 0x0104, cluster, &zcl);
        println!("-> ZCL Configure Reporting 0x{cluster:04X}/0x{attribute:04X} 10..60 s");
    }
}

/// Print one event; returns a newly admitted device.
fn report_event(message: &Received) -> Option<(u16, [u8; 8])> {
    use zigbee_zrhp::msg::data::{EvtApsDataConfirm, EvtApsDataIndication};
    use zigbee_zrhp::msg::network::{EvtDeviceAuthorized, EvtDeviceJoined, EvtDeviceLeft};
    let raw = message.raw();
    match message.msg_id {
        ids::EVT_DEVICE_JOINED => {
            let joined: EvtDeviceJoined = raw.decode().ok()?;
            println!(
                "EVT_DEVICE_JOINED nwk 0x{:04X} ieee {:02X?} parent 0x{:04X} type {} cap 0x{:02X} decision {}",
                joined.nwk,
                joined.ieee.0,
                joined.parent_nwk,
                joined.join_type,
                joined.capability,
                joined.decision
            );
            return (joined.decision == 0).then_some((joined.nwk, joined.ieee.0));
        }
        ids::EVT_DEVICE_AUTHORIZED => {
            let event: EvtDeviceAuthorized = raw.decode().ok()?;
            println!(
                "EVT_DEVICE_AUTHORIZED nwk 0x{:04X} ieee {:02X?} result {} link_key_type {}",
                event.nwk, event.ieee.0, event.result, event.link_key_type
            );
        }
        ids::EVT_DEVICE_LEFT => {
            let event: EvtDeviceLeft = raw.decode().ok()?;
            println!(
                "EVT_DEVICE_LEFT nwk 0x{:04X} ieee {:02X?} flags 0x{:02X} reason {}",
                event.nwk, event.ieee.0, event.flags, event.reason
            );
        }
        ids::EVT_APS_DATA_CONFIRM => {
            let event: EvtApsDataConfirm = raw.decode().ok()?;
            println!(
                "EVT_APS_DATA_CONFIRM tid {} status {} zb 0x{:02X} flags 0x{:02X} {} ms",
                event.req_tid, event.status, event.zb_status, event.flags, event.elapsed_ms
            );
        }
        ids::EVT_APS_DATA_INDICATION => {
            let event: EvtApsDataIndication = raw.decode().ok()?;
            println!(
                "IND from 0x{:04X} ep {} -> ep {} profile 0x{:04X} cluster 0x{:04X} lqi {} rssi {} dBm flags 0x{:02X} key {} asdu {:02X?}",
                event.src_nwk,
                event.src_ep,
                event.dst_ep,
                event.profile,
                event.cluster,
                event.lqi,
                event.rssi,
                event.flags,
                event.key_source,
                event.asdu.0
            );
        }
        ids::EVT_LOG_RECORD => {
            if let Ok(log) = raw.decode::<zigbee_zrhp::msg::log::EvtLogRecord>() {
                println!(
                    "LOG [{} ms L{}] {}",
                    log.time_ms,
                    log.level,
                    log.text.as_str().unwrap_or("<invalid UTF-8>")
                );
            }
        }
        _ => {}
    }
    None
}

/// Drive the interview from the indications of `peer`.
fn interview_step<S: ByteStream>(
    host: &mut Host<S>,
    peer: &mut Peer,
    own: [u8; 8],
    message: &Received,
) {
    use zigbee_zrhp::msg::data::EvtApsDataIndication;
    if message.msg_id != ids::EVT_APS_DATA_INDICATION {
        return;
    }
    let Ok(event) = message.raw().decode::<EvtApsDataIndication>() else {
        return;
    };
    if event.src_nwk != peer.nwk || event.profile != 0x0000 {
        return;
    }
    let asdu = event.asdu.0;
    match event.cluster {
        // Device_annce: ask for the endpoints.
        0x0013 => {
            peer.seq = peer.seq.wrapping_add(1);
            let mut zdo = vec![peer.seq];
            zdo.extend_from_slice(&peer.nwk.to_le_bytes());
            data_request(host, peer.nwk, 0, 0, 0x0000, 0x0005, &zdo);
            println!("-> ZDO Active_EP_req");
        }
        // Active_EP_rsp: seq, status, nwk, n, eps.
        0x8005 if asdu.len() >= 5 && asdu[1] == 0 && asdu[4] > 0 && peer.ep.is_none() => {
            peer.ep = Some(asdu[5]);
            println!("Active_EP_rsp: endpoints {:?}", &asdu[5..]);
            read_basic(host, peer);
            for cluster in [0x0402, 0x0405, 0x0001] {
                bind_to_ncp(host, peer, own, cluster);
            }
            configure_reporting(host, peer);
        }
        0x8021 => println!(
            "Bind_rsp status 0x{:02X}",
            asdu.get(1).copied().unwrap_or(0xFF)
        ),
        _ => {}
    }
}

fn own_ieee<S: ByteStream>(host: &mut Host<S>) -> Result<[u8; 8], String> {
    let tid = host.request_empty(ids::SYS_GET_INFO);
    let rsp = host.response(tid)?;
    let info: SysGetInfoRsp = body(&rsp)?;
    Ok(info.ieee_active.0)
}

/// Open joining on the NCP only, admit one device and interview it.
fn join<S: ByteStream>(host: &mut Host<S>, secs: u64) -> Result<(), String> {
    let own = own_ieee(host)?;
    let tid = host.request(&NwkPermitJoin {
        duration_s: secs.min(254) as u8,
        target: 0x0000,
        flags: 0,
    });
    let rsp = host.response(tid)?;
    println!("NWK_PERMIT_JOIN {secs} s (NCP only): {:?}", rsp.status());
    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut peer: Option<Peer> = None;
    let mut closed = false;
    while Instant::now() < deadline {
        host.round(Duration::from_millis(100))?;
        let events: Vec<Received> = host
            .inbox
            .drain(..)
            .filter(|m| m.kind == Kind::Event)
            .collect();
        for message in &events {
            if let Some((nwk, ieee)) = report_event(message)
                && peer.is_none()
            {
                peer = Some(Peer {
                    nwk,
                    ieee,
                    ep: None,
                    seq: 0x40,
                });
                if !closed {
                    host.request(&NwkPermitJoin {
                        duration_s: 0,
                        target: 0x0000,
                        flags: 0,
                    });
                    closed = true;
                    println!("-> NWK_PERMIT_JOIN 0 (closed after the first join)");
                }
            }
            if let Some(peer) = peer.as_mut() {
                interview_step(host, peer, own, message);
            }
        }
    }
    if !closed {
        let tid = host.request(&NwkPermitJoin {
            duration_s: 0,
            target: 0x0000,
            flags: 0,
        });
        host.response(tid)?;
        println!("NWK_PERMIT_JOIN 0 (window over)");
    }
    match peer {
        Some(peer) => {
            println!(
                "joined: nwk 0x{:04X} ieee {:02X?} ep {:?}",
                peer.nwk, peer.ieee, peer.ep
            );
            Ok(())
        }
        None => Err("no device joined".into()),
    }
}

/// ZDO Mgmt_Leave_req (no rejoin) to a joined device, then report what
/// arrives for `secs` (a sleepy device fetches it at its next poll).
fn leave<S: ByteStream>(
    host: &mut Host<S>,
    nwk: u16,
    ieee: [u8; 8],
    secs: u64,
) -> Result<(), String> {
    let mut zdo = vec![0x5A];
    zdo.extend_from_slice(&ieee);
    zdo.push(0x00);
    data_request(host, nwk, 0, 0, 0x0000, 0x0034, &zdo);
    println!("-> ZDO Mgmt_Leave_req (no rejoin) to 0x{nwk:04X}");
    let deadline = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < deadline {
        host.round(Duration::from_millis(100))?;
        let events: Vec<Received> = host
            .inbox
            .drain(..)
            .filter(|m| m.kind == Kind::Event)
            .collect();
        for message in &events {
            report_event(message);
        }
    }
    Ok(())
}

/// Talk to an already joined device and print what arrives.
fn listen<S: ByteStream>(
    host: &mut Host<S>,
    secs: u64,
    nwk: u16,
    ep: u8,
    bind_ieee: Option<[u8; 8]>,
) -> Result<(), String> {
    let mut peer = Peer {
        nwk,
        ieee: bind_ieee.unwrap_or([0; 8]),
        ep: Some(ep),
        seq: 0x80,
    };
    read_basic(host, &mut peer);
    if bind_ieee.is_some() {
        let own = own_ieee(host)?;
        for cluster in [0x0402, 0x0405, 0x0001] {
            bind_to_ncp(host, &mut peer, own, cluster);
        }
        configure_reporting(host, &mut peer);
    }
    let deadline = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < deadline {
        host.round(Duration::from_millis(100))?;
        let events: Vec<Received> = host
            .inbox
            .drain(..)
            .filter(|m| m.kind == Kind::Event)
            .collect();
        for message in &events {
            report_event(message);
        }
    }
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let value = |flag: &str| {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let stream: Box<dyn ByteStream> = if let Some(device) = value("--hwlab") {
        Box::new(Hwlab { device })
    } else if let Some(path) = value("--serial") {
        let baud = value("--baud").map_or(460_800, |b| b.parse().unwrap_or(460_800));
        match Serial::open(&path, baud) {
            Ok(serial) => Box::new(serial),
            Err(error) => {
                eprintln!("{path}: {error}");
                std::process::exit(2)
            }
        }
    } else {
        eprintln!(
            "usage: zrhp-host (--hwlab uart/<port> | --serial <path> [--baud N]) smoke [--form]"
        );
        std::process::exit(2)
    };
    let form = args.iter().any(|a| a == "--form");
    let mut host = Host::new(stream);
    let floor = |flag: &str| match value(flag).map(|v| v.parse::<u32>()) {
        None => 0,
        Some(Ok(n)) => n,
        Some(Err(_)) => {
            eprintln!("{flag} N: decimal counter floor");
            std::process::exit(2);
        }
    };
    let floors = (floor("--nwk-floor"), floor("--aps-floor"));
    let result = smoke(&mut host, form, floors).and_then(|()| {
        if let Some(path) = value("--restore") {
            backup_restore(&mut host, &path)?;
        }
        if let Some(path) = value("--backup") {
            backup_export(&mut host, &path)?;
        }
        if args.iter().any(|a| a == "--tables") {
            tables(&mut host)?;
        }
        if args.iter().any(|a| a == "--fw-slots") {
            fw_slots(&mut host)?;
        }
        if args.iter().any(|a| a == "--fw-confirm") {
            let tid = host.request_empty(ids::FW_CONFIRM);
            let rsp = host.response(tid)?;
            println!("FW_CONFIRM: {:?}", rsp.status());
            fw_slots(&mut host)?;
        }
        if let Some(secs) = value("--join") {
            join(&mut host, secs.parse().map_err(|_| "--join SECS")?)?;
        }
        if let Some(secs) = value("--listen") {
            let nwk = value("--device")
                .and_then(|v| u16::from_str_radix(v.trim_start_matches("0x"), 16).ok())
                .ok_or("--device 0xNWK required")?;
            let ep = value("--ep")
                .map_or(Ok(1), |v| v.parse())
                .map_err(|_| "--ep N")?;
            // `--bind IEEE` (16 hex digits in the byte order EVT_DEVICE_JOINED
            // prints) re-binds and configures reporting.
            let bind = value("--bind").map(|hex| {
                let mut ieee = [0u8; 8];
                for (index, byte) in ieee.iter_mut().enumerate() {
                    *byte = hex
                        .get(index * 2..index * 2 + 2)
                        .and_then(|pair| u8::from_str_radix(pair, 16).ok())
                        .unwrap_or(0);
                }
                ieee
            });
            listen(
                &mut host,
                secs.parse().map_err(|_| "--listen SECS")?,
                nwk,
                ep,
                bind,
            )?;
        }
        if let Some(hex) = value("--leave") {
            let nwk = value("--device")
                .and_then(|v| u16::from_str_radix(v.trim_start_matches("0x"), 16).ok())
                .ok_or("--device 0xNWK required")?;
            let mut ieee = [0u8; 8];
            for (index, byte) in ieee.iter_mut().enumerate() {
                *byte = hex
                    .get(index * 2..index * 2 + 2)
                    .and_then(|pair| u8::from_str_radix(pair, 16).ok())
                    .ok_or("--leave IEEE: 16 hex digits")?;
            }
            leave(&mut host, nwk, ieee, 45)?;
        }
        if let Some(path) = value("--fw-update") {
            let version = value("--fw-version")
                .and_then(|v| u32::from_str_radix(v.trim_start_matches("0x"), 16).ok())
                .ok_or("--fw-version 0xMMmmrrrr required")?;
            fw_update(&mut host, &path, version)?;
        }
        Ok(())
    });
    match result {
        Ok(()) => println!("smoke: PASS"),
        Err(error) => {
            println!("smoke: FAIL: {error}");
            std::process::exit(1)
        }
    }
}
