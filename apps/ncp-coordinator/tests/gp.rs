//! GREEN POWER (spec 7.11) end to end: the NCP's GP combo behind the
//! host link, on the real Trust Center coordinator over a mock MAC. GPDFs
//! are built and injected as the radio would hand them up; GP cluster
//! frames from a proxy are sent by a second (router) stack that shares the
//! network key, so they arrive NWK-secured as on air.
//!
//! The GP mode, report queue and counters are a process-wide singleton of
//! the runtime (one Zigbee device per NCP image): the tests take `SERIAL`.

use std::collections::VecDeque;

use ncp_coordinator_app::port::CoordinatorPort;
use ncp_coordinator_app::tc::{TcApp, TcPort};
use ncp_coordinator_app::transport::{HostTransport, TransportInfo};
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity};
use router_app::{
    NoDiagnostics, NoStatus, NoSupervisor, PersistentChildren, RouterParts, RouterPolicy,
    TrustCenterCoordinatorApp,
};
use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsTxOptions, PROFILE_HOME_AUTOMATION};
use zigbee_gp::cluster::{
    Notification, ProxyCommissioningMode, ProxyInfo, client_cmd, exit_mode, server_cmd,
};
use zigbee_gp::commissioning::{CommissioningPayload, CommissioningSecurity};
use zigbee_gp::frame::{FrameType, GpdfParams};
use zigbee_gp::security::{self, DEFAULT_TC_LINK_KEY, Direction};
use zigbee_gp::types::{GpdId, KeyType, SecurityLevel};
use zigbee_mac::mock::MockMac;
use zigbee_mac::{MacFrame, McpsDataIndication};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::child_store::RamChildTableStore;
use zigbee_runtime::gp_store::RamGpProxyStore;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_runtime::security_store::RamSecurityStateStore;
use zigbee_runtime::trust_center_store::RamTrustCenterDeviceStore;
use zigbee_types::{ChannelMask, MacAddress, PanId, ShortAddress};
use zigbee_zcl::DeviceId;
use zigbee_zrhp::codec::{Bytes8, List};
use zigbee_zrhp::frame::TX_BUFFER_LEN;
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent};
use zigbee_zrhp::msg::data::{ApsDataRequest, ApsDst, EvtApsDataIndication};
use zigbee_zrhp::msg::diag::{DiagGetCounters, DiagGetCountersRsp};
use zigbee_zrhp::msg::gp::{
    EvtGpCommissioned, EvtGpCommissioning, EvtGpDecommissioned, EvtGpFrame, EvtGpRemoteProxyTable,
    EvtGpRemoteSinkTable, EvtGpRemoteStatus, GpGroup, GpPairingConfiguration, GpProxyTableRead,
    GpProxyTableReadRsp, GpRemoteProxyTableRequest, GpRemoteRsp, GpRemoteSinkTableRequest,
    GpSendPairing, GpSetSharedKey, GpSinkCommissioning, GpSinkConfig, GpSinkRemove, GpSinkTablePut,
    GpSinkTablePutRsp, GpSinkTableRead, GpSinkTableReadRsp, GpTranslationRequest, frame_origin,
    frame_result, paired_endpoints, pairing_config_flags, send_pairing_flags, sink_mode,
};
use zigbee_zrhp::msg::network::{EvtNetworkState, NwkForm};
use zigbee_zrhp::msg::security::SecLockKeyExport;
use zigbee_zrhp::msg::system::{SysNegotiateReq, SysNegotiateRsp, capacity, caps};
use zigbee_zrhp::msg::{Batch, Kind, MsgHeader, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::records::{GpSinkRecord, Record};
use zigbee_zrhp::types::Status;

static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

const IEEE: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];
const PROXY: u16 = 0x4A76;
const PROXY_IEEE: [u8; 8] = [0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28];
const SRC: u32 = 0x7A6B_9879;
const GPD_KEY: [u8; 16] = [
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
];

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000,
    join_retry_initial_ms: 10,
    join_retry_max_ms: 100,
    secure_rejoin_failure_limit: 2,
};

// ── Host link harness (as in host_link.rs) ──────────────────────────

#[derive(Default)]
struct Pipe {
    to_ncp: VecDeque<u8>,
    to_host: Vec<u8>,
}

impl HostTransport for Pipe {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        let n = buf.len().min(self.to_ncp.len());
        for slot in &mut buf[..n] {
            *slot = self.to_ncp.pop_front().unwrap();
        }
        n
    }
    fn write(&mut self, data: &[u8]) -> usize {
        self.to_host.extend_from_slice(data);
        data.len()
    }
    fn info(&self) -> TransportInfo {
        TransportInfo {
            kind: 1,
            baud: 0,
            flow_control: false,
        }
    }
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

    fn status(&self) -> u8 {
        self.payload[0]
    }

    fn body<'a, T: zigbee_zrhp::codec::Wire<'a>>(&'a self) -> T {
        let response = self.raw().decode_response::<T>().unwrap();
        assert!(response.status.is_success(), "{:?}", response.status);
        response.body.unwrap()
    }

    fn event<'a, T: zigbee_zrhp::codec::Wire<'a>>(&'a self) -> T {
        self.raw().decode::<T>().unwrap()
    }
}

struct Host {
    link: Link<32, 32>,
    queues: EndpointQueues<4096>,
    tid: u16,
    inbox: Vec<Received>,
}

impl Host {
    fn new(now: u64) -> Self {
        let mut link = Link::new(LinkConfig::desktop_host(), 0x1A2B_3C4D);
        link.open(now, zigbee_zrhp::frame::ResetReason::HostOpen);
        Self {
            link,
            queues: EndpointQueues::new(),
            tid: 0,
            inbox: Vec::new(),
        }
    }

    fn request<'a, T: zigbee_zrhp::codec::Wire<'a> + zigbee_zrhp::msg::MessageDef>(
        &mut self,
        body: &T,
    ) -> u16 {
        self.tid += 1;
        self.queues.request(self.tid, body).unwrap();
        self.tid
    }

    fn pump(&mut self, now: u64, pipe: &mut Pipe) {
        let bytes = std::mem::take(&mut pipe.to_host);
        let mut offset = 0;
        loop {
            offset += self.link.push_rx(&bytes[offset..], now);
            while let Some(event) = self.link.poll_event() {
                if let LinkEvent::Data { payload, .. } = event {
                    for message in Batch::parse(payload).unwrap().iter() {
                        self.inbox.push(Received {
                            kind: message.header.kind,
                            msg_id: message.header.msg_id,
                            tid: message.header.tid,
                            payload: message.payload.to_vec(),
                        });
                    }
                }
            }
            if offset >= bytes.len() {
                break;
            }
        }
        let mut tx = [0u8; TX_BUFFER_LEN];
        while let Some(n) = self.link.poll_transmit(now, &mut self.queues, &mut tx) {
            pipe.to_ncp.extend(&tx[..n]);
        }
    }

    fn response(&mut self, tid: u16) -> Received {
        let index = self
            .inbox
            .iter()
            .position(|m| m.kind == Kind::Response && m.tid == tid)
            .unwrap_or_else(|| panic!("no response to tid {tid}: {:?}", self.inbox));
        self.inbox.remove(index)
    }

    fn event(&mut self, msg_id: u16) -> Option<Received> {
        let index = self
            .inbox
            .iter()
            .position(|m| m.kind == Kind::Event && m.msg_id == msg_id)?;
        Some(self.inbox.remove(index))
    }

    fn events(&mut self, msg_id: u16) -> Vec<Received> {
        let mut out = Vec::new();
        while let Some(event) = self.event(msg_id) {
            out.push(event);
        }
        out
    }
}

type Profile = DeviceProfile<RangeExtender>;

fn profile() -> Profile {
    DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::COMBINED_INTERFACE,
        RangeExtender,
    )
}

fn device(profile: &Profile) -> ZigbeeDevice<MockMac, Router> {
    let mut mac = MockMac::new(IEEE);
    mac.set_rx_delay_us(u32::MAX);
    mac.add_energy(zigbee_mac::EdValue {
        channel: 15,
        energy: 10,
    });
    ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .channels(ChannelMask(1 << 15))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_coordinator()
}

fn identity() -> NcpIdentity {
    NcpIdentity {
        fw_version: "test",
        build_id: [0; 8],
        chip: 1,
        chip_rev: 0,
        flash_size: 4 * 1024 * 1024,
        board: "pipe",
        product: "test",
        reset_reason: 1,
        key_export: true,
        max_children: 64,
        max_tc_devices: 256,
        stack_allowed: true,
    }
}

fn clock_ms() -> u32 {
    0
}

fn exchange<A: ncp_coordinator_app::tc::TcApp>(
    ncp: &mut Ncp<8>,
    host: &mut Host,
    port: &mut TcPort<A>,
    pipe: &mut Pipe,
    log: &LogRing<8>,
    now: &mut u64,
) {
    for _ in 0..8 {
        *now += 1_000;
        host.pump(*now, pipe);
        assert!(ncp.service(*now, pipe, port, log).is_none());
        while pollster::block_on(ncp.drive(port, || 0)) {}
        ncp.service(*now, pipe, port, log);
        host.pump(*now, pipe);
    }
}

/// Hand `frame` to the coordinator's radio and run stack steps (receive,
/// then the GP transmissions it caused).
fn receive<A: ncp_coordinator_app::tc::TcApp<Mac = MockMac>>(
    port: &mut TcPort<A>,
    frame: McpsDataIndication,
) {
    {
        let mac = port.app_mut().device_mut().mac_mut();
        mac.set_rx_delay_us(0);
        mac.enqueue_rx(frame);
    }
    let _ = pollster::block_on(port.step());
    port.app_mut()
        .device_mut()
        .mac_mut()
        .set_rx_delay_us(u32::MAX);
    steps(port, 3);
}

fn steps<A: ncp_coordinator_app::tc::TcApp<Mac = MockMac>>(port: &mut TcPort<A>, n: usize) {
    for _ in 0..n {
        pollster::block_on(port.step()).unwrap();
    }
}

fn tx_count<A: ncp_coordinator_app::tc::TcApp<Mac = MockMac>>(port: &TcPort<A>) -> usize {
    port.app().device().mac().tx_history().len()
}

/// A GPDF of the test GPD as the MAC hands it up (NWK bytes, MAC sequence
/// number), protected with the GPD key at `level`.
fn gpdf(level: SecurityLevel, counter: u32, command: u8, payload: &[u8]) -> McpsDataIndication {
    gpdf_of(SRC, level, counter, command, payload)
}

/// [`gpdf`] of another SrcID (same key).
fn gpdf_of(
    src: u32,
    level: SecurityLevel,
    counter: u32,
    command: u8,
    payload: &[u8],
) -> McpsDataIndication {
    let mut aes = MockMac::new([0; 8]);
    let mut raw = [0u8; 127];
    let n = security::build_gpdf(
        &mut aes,
        level.is_protected().then_some(&GPD_KEY),
        &GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(src),
            security_level: level,
            individual_key: level.is_protected(),
            rx_after_tx: false,
            to_gpd: false,
            frame_counter: counter,
        },
        counter as u8,
        command,
        payload,
        &mut raw,
    )
    .unwrap();
    McpsDataIndication {
        src_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
        dst_address: MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF)),
        lqi: 180,
        payload: MacFrame::from_slice(&raw[7..n])
            .unwrap()
            .with_mac_sequence(counter as u8),
        security_use: false,
    }
}

/// The GPD's Commissioning GPDF: level 3, individual key wrapped with the
/// default TC-LK, outgoing counter 10.
fn commissioning_gpdf() -> (McpsDataIndication, Vec<u8>) {
    let gpd = GpdId::SrcId(SRC);
    let wrapped = security::wrap_key(
        &mut MockMac::new([0; 8]),
        &DEFAULT_TC_LINK_KEY,
        &gpd,
        Direction::FromGpd,
        0,
        &GPD_KEY,
    )
    .unwrap();
    let p = CommissioningPayload {
        device_id: 0x02,
        mac_seq_num_capability: true,
        rx_on_capability: false,
        pan_id_request: false,
        security_key_request: false,
        fixed_location: true,
        security: Some(CommissioningSecurity {
            level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            key: Some((wrapped.key, Some(u32::from_le_bytes(wrapped.mic)))),
            outgoing_counter: Some(10),
        }),
        app_info: None,
    };
    let mut b = [0u8; 64];
    let n = p.encode(&mut b).unwrap();
    (gpdf(SecurityLevel::None, 1, 0xE0, &b[..n]), b[..n].to_vec())
}

/// A router on the coordinator's network that acts as another GP proxy:
/// its GP cluster frames are NWK-secured with the network key.
fn proxy_router(network_key: [u8; 16], key_seq: u8, pan_id: u16) -> ZigbeeDevice<MockMac, Router> {
    let mut device = ZigbeeDevice::builder(MockMac::new(PROXY_IEEE))
        .device_type(zigbee_nwk::DeviceType::Router)
        .build_router();
    device.bdb_mut().attributes_mut().node_is_on_a_network = true;
    let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
    nwk.set_joined(true);
    nwk.security_mut().set_network_key(network_key, key_seq);
    let nib = nwk.nib_mut();
    nib.pan_id = PanId(pan_id);
    nib.network_address = ShortAddress(PROXY);
    nib.ieee_address = PROXY_IEEE;
    nib.parent_address = ShortAddress::COORDINATOR;
    nib.security_enabled = true;
    nib.active_key_seq_number = key_seq;
    nib.outgoing_frame_counter_limit = 0x400;
    nib.logical_channel = 15;
    device
}

/// A GP cluster command from the proxy router to the coordinator's GP
/// endpoint, as the coordinator's MAC hands it up.
fn from_proxy(
    proxy: &mut ZigbeeDevice<MockMac, Router>,
    pan_id: u16,
    zcl: &[u8],
) -> McpsDataIndication {
    proxy.mac_mut().clear_tx_history();
    pollster::block_on(proxy.aps_mut().apsde_data_request(&ApsdeDataRequest {
        dst_addr_mode: ApsAddressMode::Short,
        dst_address: ApsAddress::Short(ShortAddress::COORDINATOR),
        dst_endpoint: 242,
        profile_id: 0xA1E0,
        cluster_id: 0x0021,
        src_endpoint: 242,
        payload: zcl,
        tx_options: ApsTxOptions {
            use_nwk_key: true,
            ..ApsTxOptions::default()
        },
        radius: 0,
        alias_src_addr: None,
        alias_seq: None,
    }))
    .unwrap();
    McpsDataIndication {
        src_address: MacAddress::Short(PanId(pan_id), ShortAddress(PROXY)),
        dst_address: MacAddress::Short(PanId(pan_id), ShortAddress::COORDINATOR),
        lqi: 150,
        payload: MacFrame::from_slice(proxy.mac().tx_history().last().unwrap().payload.as_slice())
            .unwrap(),
        security_use: false,
    }
}

/// A GP Notification (client → server) of the test GPD, unprotected.
fn notification_zcl(mac_seq: u8, command: u8, payload: &[u8]) -> Vec<u8> {
    let n = Notification {
        gpd_id: GpdId::SrcId(SRC),
        also_unicast: true,
        also_derived_group: false,
        also_commissioned_group: false,
        security_level: SecurityLevel::None,
        key_type: KeyType::NoKey,
        rx_after_tx: false,
        tx_queue_full: false,
        bidirectional_cap: false,
        frame_counter: u32::from(mac_seq),
        command_id: command,
        payload: Some(payload),
        proxy_info: ProxyInfo::None,
    };
    let mut buf = [0u8; 96];
    let len = n.encode(&mut buf).unwrap();
    let mut zcl = vec![0x11, mac_seq, server_cmd::NOTIFICATION];
    zcl.extend_from_slice(&buf[..len]);
    zcl
}

fn aps(dst: ApsDst, src_ep: u8, profile: u16, cluster: u16, asdu: &[u8]) -> ApsDataRequest<'_> {
    ApsDataRequest {
        dst,
        dst_ep: 242,
        src_ep,
        profile,
        cluster,
        tx_options: 0,
        radius: 0,
        non_member_radius: 0,
        priority: 1,
        relays: None,
        alias: None,
        asdu,
    }
}

fn gp_counter(counters: &DiagGetCountersRsp<'_>, id: u16) -> u32 {
    counters
        .counters
        .iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("counter 0x{id:04X}"))
        .value
}

fn level0_record(src_id: u32, counter: u32) -> GpSinkRecord {
    GpSinkRecord {
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(src_id),
        device_id: 0x07,
        communication_mode: 2,
        group: 0x0B84,
        assigned_alias: 0xFFFF,
        flags: 0x01,
        security_level: 0,
        key_type: 0,
        key: None,
        frame_counter: counter,
        groupcast_radius: 0,
    }
}

#[test]
fn ncp_sink_commissions_reports_and_hands_over_to_a_host_sink() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let mut security = RamSecurityStateStore::new();
    let mut gp_store = RamGpProxyStore::new();
    let mut profile = profile();
    let mut device = device(&profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(RamChildTableStore::new()),
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    )
    .unwrap();
    app.set_green_power_store(&mut gp_store);
    let mut port = TcPort::new(app, IEEE, clock_ms);
    let log = LogRing::<8>::new();
    let mut pipe = Pipe::default();
    let mut now = 1_000u64;
    let mut ncp = Ncp::<8>::new(identity(), 0xC0FF_EE01, now);
    let mut host = Host::new(now);
    macro_rules! exchange {
        () => {
            exchange(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now)
        };
    }

    // ── Negotiation: GP capabilities and table sizes (spec 6.6, 8.9).
    exchange!();
    let negotiate = host.request(&SysNegotiateReq {
        major_min: 1,
        major_max: 1,
        host_caps: 0,
        name: Bytes8(b"gp-host"),
    });
    exchange!();
    let response = host.response(negotiate);
    let negotiated: SysNegotiateRsp = response.body();
    assert_ne!(negotiated.ncp_caps & caps::GREEN_POWER_PROXY, 0);
    assert_ne!(negotiated.ncp_caps & caps::GREEN_POWER_SINK, 0);
    assert_eq!(negotiated.capacity(capacity::GP_SINK_TABLE), Some(16));
    assert_eq!(negotiated.capacity(capacity::GP_PROXY_TABLE), Some(8));

    // Commissioning and table changes need a network (the table is
    // restored from NV when it resumes).
    let early = host.request(&GpSinkCommissioning {
        action: 1,
        flags: 0x03,
        window_s: 60,
    });
    let early_records = [Record::GpSink(level0_record(0x0055_0009, 1))];
    let early_put = host.request(&GpSinkTablePut {
        flags: 0,
        records: List::new(&early_records),
    });
    exchange!();
    assert_eq!(host.response(early).status(), Status::InvalidState.to_u8());
    assert_eq!(
        host.response(early_put).status(),
        Status::InvalidState.to_u8()
    );

    let form = host.request(&NwkForm {
        channel_mask: 1 << 15,
        pan_id: 0xFFFF,
        ext_pan_id: [0; 8],
        network_key: [0; 16],
        key_seq: 0,
        nwk_update_id: 0,
        tx_power: 0,
        nwk_manager: 0,
    });
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let state: EvtNetworkState = host.event(ids::EVT_NETWORK_STATE).unwrap().event();
    assert_eq!(state.state, 2);
    let pan_id = port.info().pan_id;
    let (network_key, key_seq) = port.network_key(0).unwrap();

    // ── NCP_SINK (boot default): empty tables, then commissioning.
    let read = host.request(&GpSinkTableRead { start: 0 });
    let proxies = host.request(&GpProxyTableRead { start: 0 });
    let shared = host.request(&GpSetSharedKey {
        key_type: 3,
        key: [1; 16],
    });
    exchange!();
    let read_rsp = host.response(read);
    let empty: GpSinkTableReadRsp = read_rsp.body();
    assert_eq!((empty.total, empty.records.len()), (0, 0));
    let proxies_rsp = host.response(proxies);
    let empty: GpProxyTableReadRsp = proxies_rsp.body();
    assert_eq!((empty.total, empty.records.len()), (0, 0));
    assert_eq!(
        host.response(shared).status(),
        Status::UnsupportedCommand.to_u8(),
        "no shared GPD key in this sink"
    );

    let before = tx_count(&port);
    let enter = host.request(&GpSinkCommissioning {
        action: 1,
        flags: 0x03,
        window_s: 60,
    });
    exchange!();
    assert_eq!(host.response(enter).status(), 0);
    let entered: EvtGpCommissioning = host.event(ids::EVT_GP_COMMISSIONING).unwrap().event();
    assert_eq!(
        (entered.state, entered.window_s, entered.involve_proxies),
        (1, 60, 1)
    );
    steps(&mut port, 2);
    assert!(
        tx_count(&port) > before,
        "GP Proxy Commissioning Mode broadcast"
    );

    // The GPD's Commissioning GPDF heard directly: level 3 with a TC-LK
    // wrapped key is accepted (gpsSecurityLevel 0x00 admits 0, 2 and 3).
    let (commissioning, commissioning_payload) = commissioning_gpdf();
    receive(&mut port, commissioning);
    exchange!();
    let frame = host.event(ids::EVT_GP_FRAME).expect("EVT_GP_FRAME");
    let frame: EvtGpFrame = frame.event();
    assert_eq!(
        (
            frame.origin,
            frame.result,
            frame.command_id,
            frame.device_id
        ),
        (frame_origin::DIRECT, frame_result::COMMISSIONED, 0xE0, 0x02)
    );
    assert_eq!(frame.gpd_id, zigbee_zrhp::records::GpdId::SrcId(SRC));
    assert_eq!(frame.payload.0, commissioning_payload.as_slice());
    assert_eq!(frame.flags & 0x04, 0x04, "TC-LK wrapped key");
    assert_eq!(
        (frame.proxy_nwk, frame.lqi, frame.rssi),
        (0xFFFF, 180, -128)
    );
    let commissioned = host.event(ids::EVT_GP_COMMISSIONED).expect("commissioned");
    let commissioned: EvtGpCommissioned = commissioned.event();
    assert_eq!(commissioned.device_id, 0x02);
    assert_eq!(commissioned.flags & 0x02, 0x02, "key present");
    let Record::GpSink(record) = commissioned.record else {
        panic!("not R_GP_SINK");
    };
    assert_eq!(record.key, Some(GPD_KEY));
    assert_eq!((record.security_level, record.key_type), (3, 0b100));
    assert_eq!(record.frame_counter, 10);
    let left: EvtGpCommissioning = host.event(ids::EVT_GP_COMMISSIONING).unwrap().event();
    assert_eq!((left.state, left.exit_reason), (0, 2), "first pairing");
    assert_eq!(port.app().device().gp_sink().entries().len(), 1);

    // An operational, encrypted Toggle: reported decrypted, exactly once.
    receive(&mut port, gpdf(SecurityLevel::Encrypted, 11, 0x22, &[]));
    receive(&mut port, gpdf(SecurityLevel::Encrypted, 11, 0x22, &[]));
    exchange!();
    let frames = host.events(ids::EVT_GP_FRAME);
    assert_eq!(frames.len(), 1, "replay not forwarded");
    let toggle: EvtGpFrame = frames[0].event();
    assert_eq!(
        (toggle.result, toggle.command_id, toggle.frame_counter),
        (frame_result::PASSED_THROUGH, 0x22, 11)
    );
    assert_eq!((toggle.security_level, toggle.key_type), (3, 0b100));
    assert_eq!(toggle.flags & 0x02, 0x02, "decrypted");

    // The same GPD through another proxy's GP Notification (the sink
    // dedups by counter, so a fresh counter): origin 2 with the proxy.
    let mut proxy = proxy_router(network_key, key_seq, pan_id);
    let attributes = [0x05, 0x04, 0x00, 0x00, 0x29, 0x10, 0x00];
    let mut notified = notification_zcl(0, 0xA0, &attributes);
    // Protected GPD: the Notification carries level 3 and counter 12.
    {
        let n = Notification {
            gpd_id: GpdId::SrcId(SRC),
            also_unicast: true,
            also_derived_group: false,
            also_commissioned_group: false,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            rx_after_tx: false,
            tx_queue_full: false,
            bidirectional_cap: false,
            frame_counter: 12,
            command_id: 0xA0,
            payload: Some(&attributes),
            proxy_info: ProxyInfo::None,
        };
        let mut buf = [0u8; 96];
        let len = n.encode(&mut buf).unwrap();
        notified.truncate(3);
        notified.extend_from_slice(&buf[..len]);
    }
    receive(&mut port, from_proxy(&mut proxy, pan_id, &notified));
    exchange!();
    let via_proxy = host.event(ids::EVT_GP_FRAME).expect("notified");
    let via_proxy: EvtGpFrame = via_proxy.event();
    assert_eq!(
        (via_proxy.origin, via_proxy.result, via_proxy.command_id),
        (
            frame_origin::NOTIFICATION,
            frame_result::PASSED_THROUGH,
            0xA0
        )
    );
    assert_eq!((via_proxy.proxy_nwk, via_proxy.frame_counter), (PROXY, 12));
    assert_eq!(via_proxy.payload.0, &attributes);
    assert_eq!(via_proxy.device_id, 0x02);
    assert_eq!(via_proxy.lqi, 150);

    // ── Sink Table read, with and without key export.
    let read = host.request(&GpSinkTableRead { start: 0 });
    exchange!();
    let read_rsp = host.response(read);
    let table: GpSinkTableReadRsp = read_rsp.body();
    assert_eq!((table.total, table.records.len()), (1, 1));
    let Some(Record::GpSink(row)) = table.records.get(0) else {
        panic!("not R_GP_SINK");
    };
    assert_eq!((row.key, row.frame_counter), (Some(GPD_KEY), 12));

    // ── Import (no Pairing on air unless asked), removal.
    let before = tx_count(&port);
    let mut level1 = level0_record(0x0101_0101, 1);
    level1.security_level = 1;
    let records = [
        Record::GpSink(level0_record(0x0055_0001, 33)),
        Record::GpSink(record.clone()),
        Record::GpSink(level1),
    ];
    let put = host.request(&GpSinkTablePut {
        flags: 0,
        records: List::new(&records),
    });
    exchange!();
    steps(&mut port, 2);
    let put_rsp = host.response(put);
    let statuses: GpSinkTablePutRsp = put_rsp.body();
    let statuses: Vec<Status> = statuses.statuses.iter().collect();
    assert_eq!(
        statuses,
        [
            Status::Success,
            Status::AlreadyExists,
            Status::InvalidParameter
        ]
    );
    assert_eq!(tx_count(&port), before, "no GP Pairing without flags bit0");
    let imported = port
        .app()
        .device()
        .gp_sink()
        .entries()
        .iter()
        .find(|e| e.gpd_id == GpdId::SrcId(0x0055_0001))
        .unwrap()
        .clone();
    assert_eq!((imported.frame_counter, imported.device_id), (33, 0x07));
    assert!(
        port.app()
            .device()
            .aps()
            .group_member_endpoints(0x0B84)
            .contains(&242),
        "pre-commissioned group joined on the GP endpoint"
    );

    let announced = [Record::GpSink(level0_record(0x0055_0002, 1))];
    let put = host.request(&GpSinkTablePut {
        flags: 1,
        records: List::new(&announced),
    });
    exchange!();
    assert_eq!(host.response(put).status(), 0);
    steps(&mut port, 2);
    assert!(tx_count(&port) > before, "GP Pairing with flags bit0");

    let before = tx_count(&port);
    let quiet = host.request(&GpSinkRemove {
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(0x0055_0001),
        flags: 0,
    });
    let missing = host.request(&GpSinkRemove {
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(0x0BAD_0BAD),
        flags: 1,
    });
    exchange!();
    steps(&mut port, 2);
    assert_eq!(host.response(quiet).status(), 0);
    assert_eq!(host.response(missing).status(), Status::NotFound.to_u8());
    let gone: EvtGpDecommissioned = host.event(ids::EVT_GP_DECOMMISSIONED).unwrap().event();
    assert_eq!(gone.gpd_id, zigbee_zrhp::records::GpdId::SrcId(0x0055_0001));
    assert_eq!(tx_count(&port), before, "no RemoveGPD without flags bit0");
    let loud = host.request(&GpSinkRemove {
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(0x0055_0002),
        flags: 1,
    });
    exchange!();
    steps(&mut port, 2);
    assert_eq!(host.response(loud).status(), 0);
    assert!(tx_count(&port) > before, "GP Pairing (RemoveGPD)");
    let _ = host.events(ids::EVT_GP_DECOMMISSIONED);

    // ── A host sink transmission in NCP_SINK mode: CONFLICT, counted.
    let pairing = [0x19, 0x01, client_cmd::PAIRING, 0x00, 0x00, 0x00];
    let conflict = host.request(&aps(
        ApsDst::Broadcast(0xFFFD),
        242,
        0xA1E0,
        0x0021,
        &pairing,
    ));
    let annce = [
        0, 0x34, 0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0,
    ];
    let alias = host.request(&aps(ApsDst::Broadcast(0xFFFD), 0, 0, 0x0013, &annce));
    exchange!();
    assert_eq!(host.response(conflict).status(), Status::Conflict.to_u8());
    assert_eq!(host.response(alias).status(), Status::Conflict.to_u8());
    // DIAG is on the SYSTEM endpoint: asked once the conflicts happened.
    let counters = host.request(&DiagGetCounters { group_mask: 0x80 });
    exchange!();
    let counters_rsp = host.response(counters);
    let diag: DiagGetCountersRsp = counters_rsp.body();
    assert_eq!(gp_counter(&diag, 0x0813), 2, "conflicts");
    assert!(
        gp_counter(&diag, 0x0806) + gp_counter(&diag, 0x0807) >= 1,
        "the replay was counted"
    );

    // ── Key export locked: records lose their keys.
    let lock = host.request(&SecLockKeyExport {
        confirm: 0x4B43_4F4C,
    });
    let read = host.request(&GpSinkTableRead { start: 0 });
    exchange!();
    assert_eq!(host.response(lock).status(), 0);
    let read_rsp = host.response(read);
    let table: GpSinkTableReadRsp = read_rsp.body();
    let Some(Record::GpSink(row)) = table.records.get(0) else {
        panic!("not R_GP_SINK");
    };
    assert_eq!(row.key, None);
    assert_eq!(row.flags & 0x08, 0);

    // ── HOST_SINK: the NCP sink idles, GP server frames and the local
    // proxy's output go to the host.
    let config = host.request(&GpSinkConfig {
        mode: sink_mode::HOST_SINK,
        communication_mode: 1,
        group: 0xFFFF,
        exit_mode: 0x02,
        window_s: 180,
        security_level: 0,
        groupcast_radius: 0,
    });
    exchange!();
    assert_eq!(host.response(config).status(), 0);
    let refused = host.request(&GpSinkCommissioning {
        action: 1,
        flags: 0,
        window_s: 0,
    });
    exchange!();
    assert_eq!(
        host.response(refused).status(),
        Status::InvalidState.to_u8()
    );

    // A paired GPD's GPDF heard directly: no sink report any more.
    receive(&mut port, gpdf(SecurityLevel::Encrypted, 20, 0x22, &[]));
    exchange!();
    assert!(host.event(ids::EVT_GP_FRAME).is_none());

    // Another proxy's GP Notification reaches the host as a raw frame.
    let notified = notification_zcl(7, 0x20, &[]);
    receive(&mut port, from_proxy(&mut proxy, pan_id, &notified));
    exchange!();
    let raw = host
        .event(ids::EVT_APS_DATA_INDICATION)
        .expect("GP Notification for the host sink");
    let raw: EvtApsDataIndication = raw.event();
    assert_eq!((raw.dst_ep, raw.src_ep, raw.src_nwk), (242, 242, PROXY));
    assert_eq!((raw.profile, raw.cluster), (0xA1E0, 0x0021));
    assert_eq!(raw.flags & 0x80, 0, "not local");
    assert_eq!(raw.asdu.0, notified.as_slice());
    assert!(host.event(ids::EVT_GP_FRAME).is_none());

    // The host sink opens proxy commissioning: its broadcast GP Proxy
    // Commissioning Mode is accepted (no CONFLICT) and handed to the local
    // proxy, which then reports a Commissioning GPDF it hears as a local
    // GP Commissioning Notification (flag bit 7).
    let mut pcm = vec![0x19, 0x02, client_cmd::PROXY_COMMISSIONING_MODE];
    {
        let mut buf = [0u8; 16];
        let len = ProxyCommissioningMode {
            enter: true,
            exit_mode: exit_mode::ON_EXPLICIT_EXIT,
            window_s: Some(60),
            channel: None,
            unicast: false,
        }
        .encode(&mut buf)
        .unwrap();
        pcm.extend_from_slice(&buf[..len]);
    }
    let enter = host.request(&aps(ApsDst::Broadcast(0xFFFD), 242, 0xA1E0, 0x0021, &pcm));
    exchange!();
    assert_eq!(host.response(enter).status(), 0);
    let _ = host.events(ids::EVT_APS_DATA_CONFIRM);
    let (commissioning, _) = commissioning_gpdf();
    let before = tx_count(&port);
    receive(&mut port, commissioning);
    steps(&mut port, 40);
    exchange!();
    let local = host
        .event(ids::EVT_APS_DATA_INDICATION)
        .expect("local proxy output");
    let local: EvtApsDataIndication = local.event();
    assert_eq!(local.flags & 0x80, 0x80, "local");
    assert_eq!(
        (local.src_ep, local.profile, local.cluster),
        (242, 0xA1E0, 0x0021)
    );
    assert_eq!(local.asdu.0[2], server_cmd::COMMISSIONING_NOTIFICATION);
    assert!(tx_count(&port) > before, "also broadcast on air");
    assert!(host.event(ids::EVT_GP_FRAME).is_none());
    assert!(host.event(ids::EVT_GP_COMMISSIONED).is_none());

    // ── Back to NCP_SINK: the table survived the mode switches.
    let config = host.request(&GpSinkConfig {
        mode: sink_mode::NCP_SINK,
        communication_mode: 1,
        group: 0xFFFF,
        exit_mode: 0x02,
        window_s: 180,
        security_level: 0,
        groupcast_radius: 0,
    });
    exchange!();
    assert_eq!(host.response(config).status(), 0);
    receive(&mut port, gpdf(SecurityLevel::Encrypted, 30, 0x20, &[]));
    exchange!();
    let off = host.event(ids::EVT_GP_FRAME).expect("reported again");
    let off: EvtGpFrame = off.event();
    assert_eq!((off.command_id, off.frame_counter), (0x20, 30));
    assert!(gp_store.writes > 0, "the Sink Table was journaled");
}

/// Negotiate on a fresh link.
fn negotiate<A: TcApp>(
    ncp: &mut Ncp<8>,
    host: &mut Host,
    port: &mut TcPort<A>,
    pipe: &mut Pipe,
    log: &LogRing<8>,
    now: &mut u64,
) {
    exchange(ncp, host, port, pipe, log, now);
    let negotiate = host.request(&SysNegotiateReq {
        major_min: 1,
        major_max: 1,
        host_caps: 0,
        name: Bytes8(b"gp-host"),
    });
    exchange(ncp, host, port, pipe, log, now);
    assert_eq!(host.response(negotiate).status(), 0);
}

#[test]
fn the_sink_table_resumes_with_the_network_after_a_reboot() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let mut security = RamSecurityStateStore::new();
    let mut gp_store = RamGpProxyStore::new();
    let log = LogRing::<8>::new();
    let mut now = 1_000u64;
    {
        let mut profile = profile();
        let mut device = device(&profile);
        let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
        let mut app = TrustCenterCoordinatorApp::new(
            node,
            PersistentChildren::new(RamChildTableStore::new()),
            RamTrustCenterDeviceStore::new(),
            &POLICY,
            RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
        )
        .unwrap();
        app.set_green_power_store(&mut gp_store);
        let mut port = TcPort::new(app, IEEE, clock_ms);
        let mut pipe = Pipe::default();
        let mut ncp = Ncp::<8>::new(identity(), 1, now);
        let mut host = Host::new(now);
        negotiate(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
        let form = host.request(&NwkForm {
            channel_mask: 1 << 15,
            pan_id: 0xFFFF,
            ext_pan_id: [0; 8],
            network_key: [0; 16],
            key_seq: 0,
            nwk_update_id: 0,
            tx_power: 0,
            nwk_manager: 0,
        });
        exchange(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
        assert_eq!(host.response(form).status(), 0);
        let mut record = level0_record(0x0055_0003, 77);
        record.communication_mode = 1;
        record.group = 0xFFFF;
        record.security_level = 3;
        record.key_type = 0b100;
        record.key = Some(GPD_KEY);
        let records = [Record::GpSink(record)];
        let put = host.request(&GpSinkTablePut {
            flags: 0,
            records: List::new(&records),
        });
        exchange(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
        assert_eq!(host.response(put).status(), 0);
        // The next step journals the table.
        steps(&mut port, 2);
    }
    assert!(gp_store.writes > 0, "Sink Table journaled");

    // Reboot: same NV (security and GP stores), a new stack and link.
    let mut profile = profile();
    let mut device = device(&profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(RamChildTableStore::new()),
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    )
    .unwrap();
    app.set_green_power_store(&mut gp_store);
    let mut port = TcPort::new(app, IEEE, clock_ms);
    let mut pipe = Pipe::default();
    let mut ncp = Ncp::<8>::new(identity(), 2, now);
    let mut host = Host::new(now);
    negotiate(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
    let init = host.request(&zigbee_zrhp::msg::network::NwkInit {
        nwk_fc_floor: 0,
        aps_fc_floor: 0,
    });
    exchange(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
    assert_eq!(host.response(init).status(), 0);
    let state: EvtNetworkState = host.event(ids::EVT_NETWORK_STATE).unwrap().event();
    assert_eq!(state.state, 2, "resumed");
    let read = host.request(&GpSinkTableRead { start: 0 });
    exchange(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
    let read_rsp = host.response(read);
    let table: GpSinkTableReadRsp = read_rsp.body();
    assert_eq!(table.total, 1);
    let Some(Record::GpSink(row)) = table.records.get(0) else {
        panic!("not R_GP_SINK");
    };
    assert_eq!(row.gpd_id, zigbee_zrhp::records::GpdId::SrcId(0x0055_0003));
    assert_eq!((row.key, row.frame_counter), (Some(GPD_KEY), 77));
    // A replay of the last accepted counter stays rejected; the next one
    // is reported.
    receive(
        &mut port,
        gpdf_of(0x0055_0003, SecurityLevel::Encrypted, 77, 0x22, &[]),
    );
    exchange(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
    assert!(
        host.event(ids::EVT_GP_FRAME).is_none(),
        "counter floor kept"
    );
    receive(
        &mut port,
        gpdf_of(0x0055_0003, SecurityLevel::Encrypted, 78, 0x22, &[]),
    );
    exchange(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
    let next = host
        .event(ids::EVT_GP_FRAME)
        .expect("fresh frame after the reboot");
    let next: EvtGpFrame = next.event();
    assert_eq!((next.frame_counter, next.command_id), (78, 0x22));
}

/// The NCP's frame `index` (from the end of its MAC history) as `proxy`'s
/// MAC hands it up.
fn to_proxy<A: TcApp<Mac = MockMac>>(port: &TcPort<A>, pan_id: u16) -> McpsDataIndication {
    let tx = port.app().device().mac().tx_history().last().unwrap();
    McpsDataIndication {
        src_address: MacAddress::Short(PanId(pan_id), ShortAddress::COORDINATOR),
        dst_address: MacAddress::Short(PanId(pan_id), ShortAddress(PROXY)),
        lqi: 150,
        payload: MacFrame::from_slice(tx.payload.as_slice()).unwrap(),
        security_use: false,
    }
}

/// `proxy`'s newest frame as the coordinator's MAC hands it up.
fn from_proxy_tx(proxy: &ZigbeeDevice<MockMac, Router>, pan_id: u16) -> McpsDataIndication {
    McpsDataIndication {
        src_address: MacAddress::Short(PanId(pan_id), ShortAddress(PROXY)),
        dst_address: MacAddress::Short(PanId(pan_id), ShortAddress::COORDINATOR),
        lqi: 150,
        payload: MacFrame::from_slice(proxy.mac().tx_history().last().unwrap().payload.as_slice())
            .unwrap(),
        security_use: false,
    }
}

/// One remote GP round trip: the NCP's newest frame to `proxy`, the
/// proxy's answer back to the NCP.
fn round_trip<A: TcApp<Mac = MockMac>>(
    port: &mut TcPort<A>,
    proxy: &mut ZigbeeDevice<MockMac, Router>,
    pan_id: u16,
) {
    steps(port, 2);
    let frame = to_proxy(port, pan_id);
    proxy.mac_mut().clear_tx_history();
    pollster::block_on(proxy.process_incoming(&frame, &mut []));
    pollster::block_on(proxy.gp_poll());
    assert!(!proxy.mac().tx_history().is_empty(), "the proxy answered");
    // Its APS ACK (when one was requested), then its answer.
    let sent: Vec<Vec<u8>> = proxy
        .mac()
        .tx_history()
        .iter()
        .map(|tx| tx.payload.as_slice().to_vec())
        .collect();
    for payload in sent {
        receive(
            port,
            McpsDataIndication {
                src_address: MacAddress::Short(PanId(pan_id), ShortAddress(PROXY)),
                dst_address: MacAddress::Short(PanId(pan_id), ShortAddress::COORDINATOR),
                lqi: 150,
                payload: MacFrame::from_slice(&payload).unwrap(),
                security_use: false,
            },
        );
    }
}

/// Remote GP provisioning (spec 7.11, `0x0A08`-`0x0A0D`): the host reads
/// another router's Proxy and Sink Tables, configures its sink and pairs
/// proxies through the NCP; the answers come back as `EVT_GP_REMOTE_*`
/// with the TSN of the request.
#[test]
fn remote_provisioning_reaches_another_proxy_and_sink() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let mut security = RamSecurityStateStore::new();
    let mut gp_store = RamGpProxyStore::new();
    let log = LogRing::<8>::new();
    let mut now = 1_000u64;
    let mut profile = profile();
    let mut device = device(&profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(RamChildTableStore::new()),
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    )
    .unwrap();
    app.set_green_power_store(&mut gp_store);
    let mut port = TcPort::new(app, IEEE, clock_ms);
    let mut pipe = Pipe::default();
    let mut ncp = Ncp::<8>::new(identity(), 3, now);
    let mut host = Host::new(now);
    negotiate(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now);
    macro_rules! exchange {
        () => {
            exchange(&mut ncp, &mut host, &mut port, &mut pipe, &log, &mut now)
        };
    }

    // Before the network is up the request is refused.
    let early = host.request(&GpRemoteProxyTableRequest {
        dst: PROXY,
        index: 0,
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(SRC),
    });
    exchange!();
    assert_eq!(host.response(early).status(), Status::InvalidState.to_u8());

    let form = host.request(&NwkForm {
        channel_mask: 1 << 15,
        pan_id: 0xFFFF,
        ext_pan_id: [0; 8],
        network_key: [0; 16],
        key_seq: 0,
        nwk_update_id: 0,
        tx_power: 0,
        nwk_manager: 0,
    });
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let pan_id = port.info().pan_id;
    let (network_key, key_seq) = port.network_key(0).unwrap();
    let mut proxy = proxy_router(network_key, key_seq, pan_id);
    // The proxy router is the coordinator's neighbour.
    port.app_mut()
        .device_mut()
        .bdb_mut()
        .zdo_mut()
        .aps_mut()
        .nwk_mut()
        .update_neighbor_address(ShortAddress(PROXY), PROXY_IEEE);

    // ── Proxy Table Request → EVT_GP_REMOTE_PROXY_TABLE (empty table).
    let request = host.request(&GpRemoteProxyTableRequest {
        dst: PROXY,
        index: 0,
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(SRC),
    });
    exchange!();
    let response = host.response(request);
    let tsn = response.body::<GpRemoteRsp>().zcl_tsn;
    round_trip(&mut port, &mut proxy, pan_id);
    exchange!();
    let table = host
        .event(ids::EVT_GP_REMOTE_PROXY_TABLE)
        .expect("proxy table event");
    let table: EvtGpRemoteProxyTable = table.event();
    assert_eq!((table.src_nwk, table.zcl_tsn), (PROXY, tsn));
    assert_eq!(
        (table.status, table.total, table.records.len()),
        (0x8B, 0, 0)
    );
    assert!(
        host.event(ids::EVT_APS_DATA_CONFIRM).is_some(),
        "two-phase: the delivery is confirmed"
    );

    // ── GP Pairing to the proxies (broadcast, the local proxy learns it
    // too): unicast sink = the proxy router.
    let pairing = host.request(&GpSendPairing {
        dst: 0xFFFD,
        flags: send_pairing_flags::ADD_SINK
            | send_pairing_flags::KEY
            | send_pairing_flags::FRAME_COUNTER
            | send_pairing_flags::MAC_SEQ_CAPABLE,
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(SRC),
        communication_mode: 3,
        security_level: 3,
        key_type: 4,
        sink_ieee: zigbee_zrhp::types::Ieee(PROXY_IEEE),
        sink_nwk: PROXY,
        sink_group: 0,
        device_id: 0x02,
        frame_counter: 10,
        key: GPD_KEY,
        assigned_alias: 0,
        groupcast_radius: 0,
    });
    exchange!();
    assert_eq!(host.response(pairing).status(), 0);
    let local = port.app().device().gp_proxy().entries();
    assert_eq!(local.len(), 1, "the NCP's own proxy is paired");
    assert_eq!(local[0].gpd_id, GpdId::SrcId(SRC));
    steps(&mut port, 2);
    let broadcast = to_proxy(&port, pan_id);
    pollster::block_on(proxy.process_incoming(&broadcast, &mut []));
    assert_eq!(
        proxy.gp_proxy().entries().len(),
        1,
        "the remote proxy is paired"
    );
    // Naming the NCP itself as the unicast sink is the NCP sink's job.
    let conflict = host.request(&GpSendPairing {
        dst: 0xFFFD,
        flags: send_pairing_flags::ADD_SINK,
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(SRC),
        communication_mode: 3,
        security_level: 0,
        key_type: 0,
        sink_ieee: zigbee_zrhp::types::Ieee(IEEE),
        sink_nwk: 0x0000,
        sink_group: 0,
        device_id: 0x02,
        frame_counter: 0,
        key: [0; 16],
        assigned_alias: 0,
        groupcast_radius: 0,
    });
    exchange!();
    assert_eq!(host.response(conflict).status(), Status::Conflict.to_u8());

    // ── Proxy Table Request by GPD ID now finds the entry.
    let request = host.request(&GpRemoteProxyTableRequest {
        dst: PROXY,
        index: 0xFF,
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(SRC),
    });
    exchange!();
    let tsn = host.response(request).body::<GpRemoteRsp>().zcl_tsn;
    round_trip(&mut port, &mut proxy, pan_id);
    exchange!();
    let event = host.event(ids::EVT_GP_REMOTE_PROXY_TABLE).unwrap();
    let table: EvtGpRemoteProxyTable = event.event();
    assert_eq!(
        (table.zcl_tsn, table.status, table.records.len()),
        (tsn, 0, 1)
    );
    let Some(Record::GpProxy(row)) = table.records.get(0) else {
        panic!("not R_GP_PROXY");
    };
    let (entry, _) = zigbee_gp::proxy::ProxyEntry::decode(&row.entries[1..]).unwrap();
    assert_eq!(entry.gpd_id, GpdId::SrcId(SRC));
    assert_eq!(entry.key, Some(GPD_KEY), "keys exported: export not locked");

    // ── Sink Table Request → EVT_GP_REMOTE_SINK_TABLE.
    let request = host.request(&GpRemoteSinkTableRequest {
        dst: PROXY,
        index: 0,
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(SRC),
    });
    exchange!();
    let tsn = host.response(request).body::<GpRemoteRsp>().zcl_tsn;
    round_trip(&mut port, &mut proxy, pan_id);
    exchange!();
    let event = host.event(ids::EVT_GP_REMOTE_SINK_TABLE).unwrap();
    let table: EvtGpRemoteSinkTable = event.event();
    assert_eq!(
        (table.src_nwk, table.zcl_tsn, table.status),
        (PROXY, tsn, 0x8B)
    );

    // ── Pairing Configuration and Translation Table Request to a sink
    // without the translation table: Default Response UNSUP_CLUSTER_COMMAND
    // (0x81) as EVT_GP_REMOTE_STATUS.
    let groups = [GpGroup {
        group: 0x0B84,
        alias: 0xFFFF,
    }];
    let request = host.request(&GpPairingConfiguration {
        dst: PROXY,
        actions: 0x0A,
        gpd_id: zigbee_zrhp::records::GpdId::SrcId(SRC),
        device_id: 0x02,
        communication_mode: 2,
        flags: pairing_config_flags::SECURITY | pairing_config_flags::MAC_SEQ_CAPABLE,
        assigned_alias: 0,
        groupcast_radius: 0,
        security_level: 3,
        key_type: 4,
        key: GPD_KEY,
        frame_counter: 10,
        groups: List::new(&groups),
        paired: paired_endpoints::SINK_DERIVES,
        endpoints: List::new(&[]),
        app_info: 0,
        manufacturer_id: 0,
        model_id: 0,
        gpd_commands: List::new(&[]),
        server_clusters: List::new(&[]),
        client_clusters: List::new(&[]),
    });
    exchange!();
    let tsn = host.response(request).body::<GpRemoteRsp>().zcl_tsn;
    round_trip(&mut port, &mut proxy, pan_id);
    exchange!();
    let status: EvtGpRemoteStatus = host.event(ids::EVT_GP_REMOTE_STATUS).unwrap().event();
    assert_eq!(
        (status.src_nwk, status.zcl_tsn, status.command_id),
        (PROXY, tsn, 0x09)
    );
    assert_eq!(status.zcl_status, 0x81);
    let request = host.request(&GpTranslationRequest {
        dst: PROXY,
        start_index: 0,
    });
    exchange!();
    let tsn = host.response(request).body::<GpRemoteRsp>().zcl_tsn;
    round_trip(&mut port, &mut proxy, pan_id);
    exchange!();
    let status: EvtGpRemoteStatus = host.event(ids::EVT_GP_REMOTE_STATUS).unwrap().event();
    assert_eq!(
        (status.zcl_tsn, status.command_id, status.zcl_status),
        (tsn, 0x08, 0x81)
    );

    // Unicast-only requests refuse broadcasts; responses nobody asked for
    // stay with the stack.
    let broadcast = host.request(&GpTranslationRequest {
        dst: 0xFFFD,
        start_index: 0,
    });
    exchange!();
    assert_eq!(
        host.response(broadcast).status(),
        Status::InvalidParameter.to_u8()
    );
    receive(&mut port, from_proxy_tx(&proxy, pan_id));
    exchange!();
    assert!(host.event(ids::EVT_GP_REMOTE_STATUS).is_none());
}
