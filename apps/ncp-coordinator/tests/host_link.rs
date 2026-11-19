//! The NCP against the zigbee-zrhp host link over an in-memory pipe, with
//! the real Trust Center coordinator on a mock MAC.

use std::collections::VecDeque;

use ncp_coordinator_app::port::CoordinatorPort;
use ncp_coordinator_app::tc::TcPort;
use ncp_coordinator_app::transport::{HostTransport, TransportInfo};
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity};
use router_app::{
    NoDiagnostics, NoStatus, NoSupervisor, PersistentChildren, RouterParts, RouterPolicy,
    TrustCenterCoordinatorApp,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::mock::MockMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::child_store::RamChildTableStore;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_runtime::security_store::RamSecurityStateStore;
use zigbee_runtime::trust_center_store::RamTrustCenterDeviceStore;
use zigbee_types::ChannelMask;
use zigbee_zcl::DeviceId;
use zigbee_zrhp::codec::{Bytes8, Rest};
use zigbee_zrhp::frame::TX_BUFFER_LEN;
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent};
use zigbee_zrhp::msg::data::{ApsDataRequest, ApsDst, EvtApsDataConfirm};
use zigbee_zrhp::msg::network::{EvtNetworkState, NwkForm, NwkGetInfoRsp, NwkInit, NwkInitRsp};
use zigbee_zrhp::msg::security::{
    EvtFcCheckpoint, SecGetFrameCountersRsp, SecGetNetworkKey, SecLockKeyExport,
};
use zigbee_zrhp::msg::system::{EvtNcpReady, SysEcho, SysNegotiateReq, SysNegotiateRsp, caps};
use zigbee_zrhp::msg::{Batch, Kind, MsgHeader, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::types::{Endpoint, Status};

const IEEE: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000,
    join_retry_initial_ms: 10,
    join_retry_max_ms: 100,
    secure_rejoin_failure_limit: 2,
};

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

/// One message the host received.
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

    fn request_empty(&mut self, msg_id: u16, endpoint: Endpoint) -> u16 {
        self.tid += 1;
        let tid = self.tid;
        self.queues
            .queue(endpoint)
            .raw(Kind::Request, msg_id, tid, &[])
            .unwrap();
        tid
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

/// A few service/drive rounds with the host pumping in between.
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

fn aps_request(
    dst: ApsDst,
    dst_ep: u8,
    src_ep: u8,
    profile: u16,
    cluster: u16,
) -> ApsDataRequest<'static> {
    ApsDataRequest {
        dst,
        dst_ep,
        src_ep,
        profile,
        cluster,
        tx_options: 0,
        radius: 0,
        non_member_radius: 0,
        priority: 1,
        relays: None,
        alias: None,
        asdu: &[0x01, 0x00, 0x00],
    }
}

#[test]
fn host_negotiates_forms_reads_counters_and_sends_data() {
    let mut security = RamSecurityStateStore::new();
    let mut gp_store = zigbee_runtime::gp_store::RamGpProxyStore::new();
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
    // The NCP runs the GP combo; its tables need a store.
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

    // Handshake: the NCP announces itself first.
    exchange!();
    let ready: EvtNcpReady = host
        .event(ids::EVT_NCP_READY)
        .expect("EVT_NCP_READY")
        .event();
    assert_eq!(ready.ncp_cookie, 0xC0FF_EE01);
    assert_eq!(ready.network_state, 0);

    // Requests before SYS_NEGOTIATE are refused.
    let early = host.request_empty(ids::NWK_GET_INFO, Endpoint::Zigbee);
    exchange!();
    assert_eq!(host.response(early).status(), Status::InvalidState.to_u8());

    let negotiate = host.request(&SysNegotiateReq {
        major_min: 1,
        major_max: 1,
        host_caps: 0,
        name: Bytes8(b"pipe-host"),
    });
    exchange!();
    let response = host.response(negotiate);
    let negotiated: SysNegotiateRsp = response.body();
    assert_eq!(negotiated.major, 1);
    assert_ne!(negotiated.ncp_caps & caps::KEY_EXPORT, 0);
    assert_ne!(negotiated.ncp_caps & caps::BACKUP_RESTORE, 0);
    assert_eq!(negotiated.ncp_caps & caps::NCP_FW_UPDATE, 0, "NoPlatform");
    assert_eq!(negotiated.capacity(1), Some(16), "max_aps_inflight");
    assert_eq!(negotiated.capacity(7), Some(0x400), "fc_stride");
    // `ncp-capacity` table sizes (spec 8.9), halved by `compact`.
    let (keys, address_map, source_routes, neighbors) = if cfg!(feature = "compact") {
        (128, 256, 64, 64)
    } else {
        (256, 512, 128, 96)
    };
    assert_eq!(negotiated.capacity(3), Some(keys), "link_key_cache");
    assert_eq!(
        negotiated.capacity(4),
        Some(address_map),
        "address_map_cache"
    );
    assert_eq!(
        negotiated.capacity(5),
        Some(source_routes),
        "source_route_cache"
    );
    assert_eq!(negotiated.capacity(11), Some(neighbors), "neighbor_table");

    // NWK_INIT on a factory-new NCP: NOT_FORMED.
    let init = host.request(&NwkInit {
        nwk_fc_floor: 0,
        aps_fc_floor: 0,
    });
    exchange!();
    assert_eq!(host.response(init).status(), Status::NotFormed.to_u8());

    // Formation with NCP-chosen identity, then EVT_NETWORK_STATE(up).
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
    assert_eq!((state.state, state.req_tid), (2, form));
    assert_eq!(port.state() as u8, 2);
    assert_eq!(
        port.permit_join_remaining_s(),
        0,
        "formation leaves joining closed"
    );

    // Forming again while a network exists: INVALID_STATE.
    let again = host.request(&NwkForm {
        channel_mask: 1 << 15,
        pan_id: 0x1A62,
        ext_pan_id: [1; 8],
        network_key: [2; 16],
        key_seq: 0,
        nwk_update_id: 0,
        tx_power: 0,
        nwk_manager: 0,
    });
    exchange!();
    assert_eq!(host.response(again).status(), Status::InvalidState.to_u8());

    let info = host.request_empty(ids::NWK_GET_INFO, Endpoint::Zigbee);
    exchange!();
    let info_rsp = host.response(info);
    let info: NwkGetInfoRsp = info_rsp.body();
    assert_eq!((info.state, info.short, info.channel), (2, 0x0000, 15));
    assert_eq!(info.ieee.0, IEEE);

    // Frame counters: both domains reserved, and checkpoints announced.
    let counters = host.request_empty(ids::SEC_GET_FRAME_COUNTERS, Endpoint::Zigbee);
    exchange!();
    let counters_rsp = host.response(counters);
    let counters: SecGetFrameCountersRsp = counters_rsp.body();
    assert!(counters.nwk_limit > counters.nwk_current);
    assert!(counters.aps_limit > counters.aps_current);
    assert_eq!(counters.stride, 0x400);
    let mut domains = Vec::new();
    while let Some(checkpoint) = host.event(ids::EVT_FC_CHECKPOINT) {
        let checkpoint: EvtFcCheckpoint = checkpoint.event();
        domains.push((checkpoint.domain, checkpoint.durable_limit));
    }
    assert!(domains.contains(&(1, counters.nwk_limit)), "{domains:?}");
    assert!(domains.contains(&(2, counters.aps_limit)), "{domains:?}");

    // NWK_INIT on a running network only applies the floors.
    let floor = counters.aps_limit + 0x10_0000;
    let init = host.request(&NwkInit {
        nwk_fc_floor: counters.nwk_limit + 0x5000,
        aps_fc_floor: floor,
    });
    exchange!();
    let init_rsp = host.response(init);
    assert_eq!(init_rsp.body::<NwkInitRsp>().network_state, 2);
    let after = port.frame_counters();
    assert!(after.aps_current >= floor);
    assert!(after.nwk_current >= counters.nwk_limit + 0x5000);

    // Key export, then the lock.
    let key = host.request(&SecGetNetworkKey { which: 0 });
    exchange!();
    assert_eq!(host.response(key).status(), 0);
    let bad_lock = host.request(&SecLockKeyExport { confirm: 1 });
    let lock = host.request(&SecLockKeyExport {
        confirm: 0x4B43_4F4C,
    });
    let key = host.request(&SecGetNetworkKey { which: 0 });
    exchange!();
    assert_eq!(
        host.response(bad_lock).status(),
        Status::InvalidParameter.to_u8()
    );
    assert_eq!(host.response(lock).status(), 0);
    assert_eq!(host.response(key).status(), Status::AccessDenied.to_u8());

    // A broadcast ZCL frame: accepted, then confirmed; ZDO to the NCP
    // itself is refused (no loopback).
    let data = host.request(&aps_request(ApsDst::Broadcast(0xFFFD), 0xFF, 1, 0x0104, 6));
    let loopback = host.request(&aps_request(ApsDst::Nwk(0x0000), 0, 0, 0, 5));
    exchange!();
    assert_eq!(host.response(data).status(), 0);
    assert_eq!(
        host.response(loopback).status(),
        Status::InvalidParameter.to_u8()
    );
    let confirm: EvtApsDataConfirm = host.event(ids::EVT_APS_DATA_CONFIRM).unwrap().event();
    assert_eq!((confirm.req_tid, confirm.status), (data, 0));
    // The host frame went out NWK-secured (frame control bit 9).
    let sent = ncp_coordinator_app::tc::TcApp::device(port.app())
        .aps()
        .nwk()
        .mac()
        .tx_history();
    let nwk_frame = sent.last().expect("broadcast sent").payload.as_slice();
    assert_ne!(nwk_frame[1] & 0x02, 0, "NWK security on host data");

    // Echo, an unknown ID, a wrong endpoint, and an unsupported command.
    let echo = host.request(&SysEcho {
        data: Rest(b"ping"),
    });
    let unknown = host.request_empty(0x00F0, Endpoint::System);
    let wrong = host.request_empty(ids::NWK_GET_INFO, Endpoint::System);
    // FW_* without platform firmware services (`NoPlatform`).
    let unsupported = host.request_empty(ids::FW_GET_SLOTS, Endpoint::Bulk);
    exchange!();
    assert_eq!(host.response(echo).payload, b"\x00ping");
    assert_eq!(
        host.response(unknown).status(),
        Status::UnsupportedCommand.to_u8()
    );
    assert_eq!(host.response(wrong).status(), Status::WrongEndpoint.to_u8());
    assert_eq!(
        host.response(unsupported).status(),
        Status::UnsupportedCommand.to_u8()
    );
}
