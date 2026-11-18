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
//! ```
//!
//! `smoke` never opens the network for joining and never transmits user
//! frames on air except what formation itself sends.

use std::process::Command;
use std::time::{Duration, Instant};

use zigbee_zrhp::codec::{Bytes8, Rest, Wire};
use zigbee_zrhp::frame::{ResetReason, TX_BUFFER_LEN};
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent, LinkState};
use zigbee_zrhp::msg::diag::{DiagGetCounters, DiagGetCountersRsp};
use zigbee_zrhp::msg::network::{EvtNetworkState, NwkForm, NwkGetInfoRsp, NwkInit, NwkInitRsp};
use zigbee_zrhp::msg::security::SecGetFrameCountersRsp;
use zigbee_zrhp::msg::system::{
    EvtNcpReady, SysEcho, SysGetInfoRsp, SysGetStatusRsp, SysNegotiateReq, SysNegotiateRsp,
};
use zigbee_zrhp::msg::{Batch, Kind, MessageDef, MsgHeader, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
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
        self.wait_for(
            &format!("response tid {tid}"),
            Duration::from_secs(10),
            |m| m.kind == Kind::Response && m.tid == tid,
        )
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

fn smoke<S: ByteStream>(host: &mut Host<S>, form: bool) -> Result<(), String> {
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
            nwk_fc_floor: 0,
            aps_fc_floor: 0,
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

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let device = args
        .iter()
        .position(|a| a == "--hwlab")
        .and_then(|i| args.get(i + 1))
        .cloned()
        .unwrap_or_else(|| {
            eprintln!("usage: zrhp-host --hwlab uart/<port> smoke [--form]");
            std::process::exit(2)
        });
    let form = args.iter().any(|a| a == "--form");
    let mut host = Host::new(Hwlab { device });
    match smoke(&mut host, form) {
        Ok(()) => println!("smoke: PASS"),
        Err(error) => {
            println!("smoke: FAIL: {error}");
            std::process::exit(1)
        }
    }
}
