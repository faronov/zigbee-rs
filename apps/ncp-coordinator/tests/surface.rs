//! The rest of the NCP surface over the zigbee-zrhp host link: host-supplied
//! formation, TABLES, BACKUP export → restore, FW_* and SYS_SET_IEEE/PTA on
//! a fake platform, and the retry cache.

use std::collections::VecDeque;

use ncp_coordinator_app::platform::{
    Counter, Counters, FwBegin, FwImage, FwSlot, NcpPlatform, PlatformError, slot_state,
};
use ncp_coordinator_app::port::CoordinatorPort;
use ncp_coordinator_app::tc::{TcApp, TcPort};
use ncp_coordinator_app::transport::{HostTransport, TransportInfo};
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity, PlatformRequest};
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
use zigbee_zrhp::codec::{Bytes8, List, Rest};
use zigbee_zrhp::codec::{Wire, Writer};
use zigbee_zrhp::frame::TX_BUFFER_LEN;
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent};
use zigbee_zrhp::msg::backup::{
    BkpExportBegin, BkpExportBeginRsp, BkpExportEnd, BkpExportRead, BkpRestoreAbort,
    BkpRestoreBegin, BkpRestoreBeginRsp, BkpRestoreCommit, BkpRestoreCommitRsp, BkpRestorePutRsp,
};
use zigbee_zrhp::msg::diag::{
    DiagGetChildren, DiagGetChildrenRsp, DiagGetCounters, DiagGetCountersRsp, DiagGetNeighbors,
    DiagGetNeighborsRsp, DiagGetRoutes, DiagGetRoutesRsp, DiagResetCounters,
};
use zigbee_zrhp::msg::firmware::{
    FwActivate, FwBegin as FwBeginReq, FwBeginRsp, FwFinishRsp, FwGetSlotsRsp, FwWrite, FwWriteRsp,
};
use zigbee_zrhp::msg::network::{
    EvtChannelChanged, EvtEnergyScanResult, NwkChangeChannel, NwkEnergyScan, NwkForm,
    NwkGetInfoRsp, NwkGroupMembership, NwkLeaveReset, NwkSetTxPower, NwkSetTxPowerRsp,
};
use zigbee_zrhp::msg::security::{
    SecAddInstallCode, SecAddInstallCodeRsp, SecAddLinkKey, SecGetFrameCountersRsp,
    SecGetNetworkKey, SecGetNetworkKeyRsp,
};
use zigbee_zrhp::msg::system::{
    ConfigValue, SysGetConfig, SysGetConfigRsp, SysGetInfoRsp, SysGetStatusRsp, SysNegotiateReq,
    SysNegotiateRsp, SysReset, SysSetConfig, SysSetConfigRsp, SysSetIeee, caps, config,
};
use zigbee_zrhp::msg::tables::{
    EvtTblEpochChanged, TableKey, TblInvalidate, TblPush, TblPushRsp, TblRead, TblReadRsp,
    TblStats, TblStatsRsp, TblSync, TblSyncRsp,
};
use zigbee_zrhp::msg::{Batch, Kind, MsgHeader, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::records::{DeviceRecord, LinkBlock, RawRecords, Record, StreamCrc, record_type};
use zigbee_zrhp::types::{Endpoint, Ieee, Status};

const IEEE: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];
const OTHER_IEEE: [u8; 8] = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8];
const NETWORK_KEY: [u8; 16] = [
    0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0D,
];
const EPID: [u8; 8] = [0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD];
const DEVICE_IEEE: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
const DEVICE_KEY: [u8; 16] = [0x5A; 16];

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000,
    join_retry_initial_ms: 10,
    join_retry_max_ms: 100,
    secure_rejoin_failure_limit: 2,
};

// ── Host side ───────────────────────────────────────────────────────

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
            kind: 2,
            baud: 460_800,
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
        assert!(
            response.status.is_success(),
            "{:#06x}: {:?}",
            self.msg_id,
            response.status
        );
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

    /// The same request again with the RETRY flag (spec 6.5).
    fn retry<'a, T: zigbee_zrhp::codec::Wire<'a> + zigbee_zrhp::msg::MessageDef>(
        &mut self,
        tid: u16,
        body: &T,
    ) {
        let endpoint = zigbee_zrhp::msg::home_endpoint(T::ID).unwrap();
        self.queues
            .queue(endpoint)
            .push_with(|w| w.request_retry(tid, body))
            .unwrap();
    }

    fn request_raw(&mut self, msg_id: u16, payload: &[u8]) -> u16 {
        self.tid += 1;
        let tid = self.tid;
        let endpoint = zigbee_zrhp::msg::home_endpoint(msg_id).unwrap();
        self.queues
            .queue(endpoint)
            .raw(Kind::Request, msg_id, tid, payload)
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

    fn responses(&self, tid: u16) -> usize {
        self.inbox
            .iter()
            .filter(|m| m.kind == Kind::Response && m.tid == tid)
            .count()
    }

    fn event(&mut self, msg_id: u16) -> Option<Received> {
        let index = self
            .inbox
            .iter()
            .position(|m| m.kind == Kind::Event && m.msg_id == msg_id)?;
        Some(self.inbox.remove(index))
    }
}

// ── Fake product platform ──────────────────────────────────────────

#[derive(Debug, Default)]
struct FakePlatform {
    image: Option<FwImage>,
    staged: Vec<u8>,
    finished: bool,
    activated: bool,
    confirmed: bool,
    ieee_override: Option<[u8; 8]>,
    pta: Option<(bool, u8)>,
    mac_tx: u32,
}

impl NcpPlatform for FakePlatform {
    fn fw_begin(&mut self, image: &FwImage) -> Result<FwBegin, PlatformError> {
        if image.image_size > 4096 {
            return Err(PlatformError::TooLarge);
        }
        self.image = Some(*image);
        self.staged.clear();
        self.finished = false;
        // The product engine's chunk (1 KiB) exceeds a frame.
        Ok(FwBegin {
            max_chunk: 1024,
            slot: 1,
        })
    }
    fn fw_write(&mut self, offset: u32, data: &[u8]) -> Result<u32, PlatformError> {
        let image = self.image.ok_or(PlatformError::InvalidState)?;
        if offset as usize != self.staged.len() {
            return Err(PlatformError::InvalidParameter);
        }
        if self.staged.len() + data.len() > image.image_size as usize {
            return Err(PlatformError::TooLarge);
        }
        self.staged.extend_from_slice(data);
        Ok(self.staged.len() as u32)
    }
    fn fw_finish(&mut self) -> Result<u8, PlatformError> {
        let image = self.image.ok_or(PlatformError::InvalidState)?;
        if self.staged.len() != image.image_size as usize {
            return Err(PlatformError::VerifyFailed);
        }
        self.finished = true;
        Ok(0)
    }
    fn fw_activate(&mut self) -> Result<(), PlatformError> {
        if !self.finished {
            return Err(PlatformError::InvalidState);
        }
        self.activated = true;
        Ok(())
    }
    fn fw_confirm(&mut self) -> Result<(), PlatformError> {
        self.confirmed = true;
        Ok(())
    }
    fn fw_abort(&mut self) -> Result<(), PlatformError> {
        self.image = None;
        self.staged.clear();
        Ok(())
    }
    fn fw_slots(&mut self, out: &mut [FwSlot]) -> Result<usize, PlatformError> {
        out[0] = FwSlot {
            slot: 0,
            state: slot_state::RUNNING,
            version: 1,
            sha256: [0; 32],
        };
        out[1] = FwSlot {
            slot: 1,
            state: if self.finished {
                slot_state::STAGED
            } else {
                slot_state::EMPTY
            },
            version: self.image.map_or(0, |image| image.version),
            sha256: self.image.map_or([0; 32], |image| image.sha256),
        };
        Ok(2)
    }
    fn fw_supported(&self) -> bool {
        true
    }
    fn ieee_rewritable(&self) -> bool {
        true
    }
    fn factory_ieee(&self) -> Option<[u8; 8]> {
        Some(IEEE)
    }
    fn set_ieee_override(&mut self, ieee: Option<[u8; 8]>) -> Result<(), PlatformError> {
        self.ieee_override = ieee;
        Ok(())
    }
    fn pta_supported(&self) -> bool {
        true
    }
    fn set_pta(&mut self, enable: bool, priority_mask: u8) -> Result<(), PlatformError> {
        self.pta = Some((enable, priority_mask));
        Ok(())
    }
    /// A radio with 2 dB steps from -20 to +10 dBm.
    fn tx_power_step(&self, dbm: i8) -> Option<i8> {
        Some(dbm.clamp(-20, 10) & !1)
    }
    fn energy_to_dbm(&self, energy: u8) -> i8 {
        energy as i8 - 100
    }
    fn append_counters(&self, group_mask: u8, out: &mut Counters) {
        if group_mask & 0x02 != 0 {
            let _ = out.push(Counter {
                id: 0x0201,
                value: self.mac_tx,
            });
        }
    }
    fn reset_counters(&mut self, group_mask: u8) {
        if group_mask & 0x02 != 0 {
            self.mac_tx = 0;
        }
    }
}

// ── NCP side ────────────────────────────────────────────────────────

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
    for (channel, energy) in [(15, 60), (20, 5), (25, 90)] {
        mac.add_energy(zigbee_mac::EdValue { channel, energy });
    }
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

/// A few service/drive rounds with the host pumping in between; returns
/// the platform requests the NCP raised.
fn exchange<A: ncp_coordinator_app::tc::TcApp>(
    ncp: &mut Ncp<8>,
    host: &mut Host,
    port: &mut TcPort<A>,
    platform: &mut FakePlatform,
    pipe: &mut Pipe,
    log: &LogRing<8>,
    now: &mut u64,
) -> Vec<PlatformRequest> {
    let mut requests = Vec::new();
    for _ in 0..10 {
        *now += 1_000;
        host.pump(*now, pipe);
        requests.extend(ncp.service_with(*now, pipe, port, platform, log));
        while pollster::block_on(ncp.drive_with(port, platform, || 0)) {}
        requests.extend(ncp.service_with(*now, pipe, port, platform, log));
        host.pump(*now, pipe);
    }
    requests
}

macro_rules! setup {
    ($ncp:ident, $host:ident, $port:ident, $platform:ident, $pipe:ident, $log:ident, $now:ident,
     $exchange:ident) => {
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
        app.set_green_power_store(&mut gp_store);
        let mut $port = TcPort::new(app, IEEE, clock_ms);
        let $log = LogRing::<8>::new();
        let mut $pipe = Pipe::default();
        let mut $platform = FakePlatform::default();
        let mut $now = 1_000u64;
        let mut $ncp = Ncp::<8>::new(identity(), 0xC0FF_EE02, $now);
        let mut $host = Host::new($now);
        macro_rules! $exchange {
            () => {
                exchange(
                    &mut $ncp,
                    &mut $host,
                    &mut $port,
                    &mut $platform,
                    &mut $pipe,
                    &$log,
                    &mut $now,
                )
            };
        }
        $exchange!();
        assert!($host.event(ids::EVT_NCP_READY).is_some());
        let negotiate = $host.request(&SysNegotiateReq {
            major_min: 1,
            major_max: 1,
            host_caps: 0,
            name: Bytes8(b"surface"),
        });
        $exchange!();
        let negotiated = $host.response(negotiate);
        let negotiated: SysNegotiateRsp = negotiated.body();
        for bit in [
            caps::BACKUP_RESTORE,
            caps::NCP_FW_UPDATE,
            caps::IEEE_REWRITABLE,
            caps::PTA_COEX,
            caps::GROUP_MEMBERSHIP,
            caps::ENERGY_SCAN,
            caps::TX_POWER,
        ] {
            assert_ne!(negotiated.ncp_caps & bit, 0, "capability {bit:#x}");
        }
    };
}

fn form_request(pan_id: u16, channel_mask: u32) -> NwkForm {
    NwkForm {
        channel_mask,
        pan_id,
        ext_pan_id: EPID,
        network_key: NETWORK_KEY,
        key_seq: 3,
        nwk_update_id: 2,
        tx_power: 0,
        nwk_manager: 0x0000,
    }
}

fn device_push(ieee: [u8; 8], key: [u8; 16]) -> Record<'static> {
    Record::Device(DeviceRecord {
        ieee: Ieee(ieee),
        nwk: 0xFFFE,
        flags: 0,
        child: None,
        link: Some(LinkBlock {
            key,
            key_seq: 0,
            attrs: 0,
            origin: 0,
            tx_counter: 0,
            rx_floor: 0,
        }),
    })
}

#[test]
fn host_identity_formation_tables_and_backup_restore() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);

    // Host-chosen PAN ID / EPID / key; the quietest channel of the mask.
    let form = host.request(&form_request(0x1A62, (1 << 15) | (1 << 20) | (1 << 25)));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let info = host.request_raw(ids::NWK_GET_INFO, &[]);
    exchange!();
    let info = host.response(info);
    let info: NwkGetInfoRsp = info.body();
    assert_eq!(
        (info.state, info.pan_id, info.ext_pan_id, info.channel),
        (2, 0x1A62, EPID, 20)
    );
    assert_eq!((info.key_seq, info.nwk_update_id), (3, 2));
    let key = host.request(&SecGetNetworkKey { which: 0 });
    exchange!();
    let key = host.response(key);
    let key: SecGetNetworkKeyRsp = key.body();
    assert_eq!((key.key, key.seq), (NETWORK_KEY, 3));

    // TABLES: provision a unique TCLK, read it back, invalidate another.
    let push = host.request(&TblPush {
        table: record_type::DEVICE,
        flags: 0,
        records: List::new(&[device_push(DEVICE_IEEE, DEVICE_KEY)]),
    });
    let stats = host.request(&TblStats {
        table: record_type::DEVICE,
    });
    let read = host.request(&TblRead {
        table: record_type::DEVICE,
        cursor: 0,
        max: 8,
    });
    let unknown = host.request(&TblInvalidate {
        table: record_type::DEVICE,
        key: TableKey::Ieee(Ieee(OTHER_IEEE)),
    });
    let sync = host.request(&TblSync {
        host_db_epoch: 1,
        last_ncp_table_epoch: 0,
        host_max_rev: 0,
        flags: 0,
    });
    exchange!();
    let push = host.response(push);
    let push: TblPushRsp = push.body();
    assert_eq!(
        push.results.iter().map(|r| r.status).collect::<Vec<_>>(),
        [Status::Success]
    );
    let stats = host.response(stats);
    let stats: TblStatsRsp = stats.body();
    assert_eq!((stats.used, stats.pinned), (1, 0));
    assert!(stats.capacity >= 1);
    let read = host.response(read);
    let read: TblReadRsp = read.body();
    assert_eq!(read.next_cursor, 0xFFFF);
    let entries: Vec<_> = read.entries.iter().collect();
    assert_eq!(entries.len(), 1);
    let Record::Device(record) = &entries[0].record else {
        panic!("{:?}", entries[0].record)
    };
    assert_eq!(record.ieee.0, DEVICE_IEEE);
    assert_eq!(record.link.as_ref().map(|link| link.key), Some(DEVICE_KEY));
    assert_eq!(host.response(unknown).status(), 0, "unknown device: no-op");
    let sync = host.response(sync);
    let sync: TblSyncRsp = sync.body();
    assert_ne!(sync.ncp_table_epoch, 0);
    assert_eq!(sync.cached_devices, 1);
    let epoch = sync.ncp_table_epoch;

    // Export: keys and devices.
    let counters = host.request_raw(ids::SEC_GET_FRAME_COUNTERS, &[]);
    let begin = host.request(&BkpExportBegin { flags: 0x03 });
    exchange!();
    let counters = host.response(counters);
    let counters: SecGetFrameCountersRsp = counters.body();
    let begin = host.response(begin);
    let begin: BkpExportBeginRsp = begin.body();
    assert_eq!(begin.record_count, 7, "NODE NETWORK KEY TCLK DEVICE RS END");
    let mut stream = Vec::new();
    loop {
        let read = host.request(&BkpExportRead {
            txn: begin.txn,
            max_bytes: 64,
        });
        exchange!();
        let read = host.response(read);
        assert_eq!(read.status(), 0);
        let more = read.payload[1];
        stream.extend_from_slice(&read.payload[2..]);
        if more == 0 {
            break;
        }
    }
    let end = host.request(&BkpExportEnd { txn: begin.txn });
    exchange!();
    assert_eq!(host.response(end).status(), 0);
    let records: Vec<&[u8]> = RawRecords::new(&stream).map(|r| r.unwrap().1).collect();
    assert_eq!(records.len(), 7);
    assert_eq!(records[0][0], record_type::NODE);
    assert_eq!(*records.last().unwrap().first().unwrap(), record_type::END);

    // A restore begun with the network up stages while it keeps running;
    // ABORT leaves it as it was (9.4 rule 1).
    let early = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 10_000,
        aps_margin: 10_000,
        expected_records: 7,
    });
    exchange!();
    let early = host.response(early);
    let early: BkpRestoreBeginRsp = early.body();
    let abort = host.request(&BkpRestoreAbort { txn: early.txn });
    exchange!();
    assert_eq!(host.response(abort).status(), 0);
    assert_eq!(port.state() as u8, 2, "still up after ABORT");

    // Factory reset of the network, then the restore.
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);
    let wiped: EvtTblEpochChanged = host.event(ids::EVT_TBL_EPOCH_CHANGED).unwrap().event();
    assert_eq!((wiped.new_epoch, wiped.reason), (0, 1));
    assert_eq!(port.state() as u8, 0);

    let begin = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 10_000,
        aps_margin: 10_000,
        expected_records: 7,
    });
    exchange!();
    let begin = host.response(begin);
    let begin: BkpRestoreBeginRsp = begin.body();
    let status = host.request_raw(ids::SYS_GET_STATUS, &[]);
    exchange!();
    let status = host.response(status);
    assert_eq!(status.body::<SysGetStatusRsp>().restore_in_progress, 1);
    for chunk in records.chunks(usize::from(begin.max_records_per_put).min(3)) {
        let mut payload = begin.txn.to_le_bytes().to_vec();
        for record in chunk {
            payload.extend_from_slice(record);
        }
        let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
        exchange!();
        let put = host.response(put);
        let put: BkpRestorePutRsp = put.body();
        assert_eq!(usize::from(put.accepted), chunk.len());
        assert_eq!(
            put.notes.iter().count(),
            0,
            "a zigbee-rs backup restores as is"
        );
    }
    let commit = host.request(&BkpRestoreCommit { txn: begin.txn });
    exchange!();
    let commit = host.response(commit);
    let commit: BkpRestoreCommitRsp = commit.body();
    assert!(commit.nwk_fc_start >= counters.nwk_limit + 10_000);
    assert!(commit.aps_fc_start >= counters.aps_limit + 10_000);
    assert_eq!(
        (commit.link_keys_on_ncp, commit.link_keys_host_backed),
        (1, 0)
    );
    let restored: EvtTblEpochChanged = host.event(ids::EVT_TBL_EPOCH_CHANGED).unwrap().event();
    assert_eq!((restored.new_epoch, restored.reason), (epoch, 2));

    // The same network, key and device link key, counters forward only.
    let info = host.request_raw(ids::NWK_GET_INFO, &[]);
    let key = host.request(&SecGetNetworkKey { which: 0 });
    let read = host.request(&TblRead {
        table: record_type::DEVICE,
        cursor: 0,
        max: 8,
    });
    exchange!();
    let info = host.response(info);
    let info: NwkGetInfoRsp = info.body();
    assert_eq!(
        (info.state, info.pan_id, info.ext_pan_id, info.channel),
        (2, 0x1A62, EPID, 20)
    );
    let key = host.response(key);
    let key: SecGetNetworkKeyRsp = key.body();
    assert_eq!((key.key, key.seq), (NETWORK_KEY, 3));
    let read = host.response(read);
    let read: TblReadRsp = read.body();
    let entries: Vec<_> = read.entries.iter().collect();
    assert_eq!(entries.len(), 1);
    let after = port.frame_counters();
    assert!(after.nwk_current >= commit.nwk_fc_start);
    assert!(after.aps_current >= commit.aps_fc_start);

    // Deleting the device (I5) removes its key.
    let invalidate = host.request(&TblInvalidate {
        table: record_type::DEVICE,
        key: TableKey::Ieee(Ieee(DEVICE_IEEE)),
    });
    exchange!();
    assert_eq!(host.response(invalidate).status(), 0);
    assert!(port.device(0).is_none());

    // A restore committed while the network is up replaces it in place:
    // the device key returns, counters only move forward.
    let before = port.frame_counters();
    let begin = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 0,
        aps_margin: 0,
        expected_records: 7,
    });
    exchange!();
    let begin = host.response(begin);
    let begin: BkpRestoreBeginRsp = begin.body();
    let mut payload = begin.txn.to_le_bytes().to_vec();
    for record in &records {
        payload.extend_from_slice(record);
    }
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    exchange!();
    let put = host.response(put);
    assert_eq!(usize::from(put.body::<BkpRestorePutRsp>().accepted), 7);
    let commit = host.request(&BkpRestoreCommit { txn: begin.txn });
    exchange!();
    let commit = host.response(commit);
    let commit: BkpRestoreCommitRsp = commit.body();
    assert!(commit.nwk_fc_start > before.nwk_current);
    assert!(commit.aps_fc_start > before.aps_current);
    assert_eq!(commit.link_keys_on_ncp, 1);
    assert_eq!(port.state() as u8, 2);
    let after = port.frame_counters();
    assert!(after.nwk_current > before.nwk_current);
    assert!(after.aps_current > before.aps_current);
    assert_eq!(port.device(0).map(|view| view.ieee), Some(DEVICE_IEEE));
}

#[test]
fn restore_rejects_bad_records_and_crc() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x2B73, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let begin = host.request(&BkpExportBegin { flags: 0x01 });
    exchange!();
    let begin = host.response(begin);
    let begin: BkpExportBeginRsp = begin.body();
    let read = host.request(&BkpExportRead {
        txn: begin.txn,
        max_bytes: 480,
    });
    exchange!();
    let read = host.response(read);
    assert_eq!(read.payload[1], 0, "one chunk");
    let stream = read.payload[2..].to_vec();
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);

    // A flipped byte inside the network key: END's CRC fails at commit.
    let mut corrupt = stream.clone();
    let records: Vec<(usize, u8)> = {
        let mut offset = 0;
        RawRecords::new(&stream)
            .map(|r| {
                let (kind, tlv) = r.unwrap();
                let at = offset;
                offset += tlv.len();
                (at, kind)
            })
            .collect()
    };
    let (key_at, _) = records
        .iter()
        .find(|(_, kind)| *kind == record_type::NETWORK_KEY)
        .unwrap();
    corrupt[key_at + 3] ^= 0x01;
    let restore = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 0,
        aps_margin: 0,
        expected_records: 6,
    });
    exchange!();
    let restore = host.response(restore);
    let txn = restore.body::<BkpRestoreBeginRsp>().txn;
    let mut payload = txn.to_le_bytes().to_vec();
    payload.extend_from_slice(&corrupt);
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    exchange!();
    assert_eq!(host.response(put).status(), 0);
    let commit = host.request(&BkpRestoreCommit { txn });
    exchange!();
    assert_eq!(
        host.response(commit).status(),
        Status::SecurityError.to_u8()
    );
    assert_eq!(port.state() as u8, 0, "nothing installed");

    // A PAN ID of 0xFFFF violates 9.4 rule 2: the PUT fails and aborts.
    let restore = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 0,
        aps_margin: 0,
        expected_records: 6,
    });
    exchange!();
    let restore = host.response(restore);
    let txn = restore.body::<BkpRestoreBeginRsp>().txn;
    let mut bad = stream.clone();
    let (network_at, _) = records
        .iter()
        .find(|(_, kind)| *kind == record_type::NETWORK)
        .unwrap();
    bad[network_at + 3] = 0xFF;
    bad[network_at + 4] = 0xFF;
    let mut payload = txn.to_le_bytes().to_vec();
    payload.extend_from_slice(&bad);
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    let commit = host.request(&BkpRestoreCommit { txn });
    exchange!();
    assert_eq!(
        host.response(put).status(),
        Status::InvalidParameter.to_u8()
    );
    assert_eq!(host.response(commit).status(), Status::NotFound.to_u8());

    // A foreign NODE IEEE without the allow-IEEE flag is refused; with it
    // the override is stored and the radio takes the restored IEEE.
    let mut foreign = stream.clone();
    foreign[3..11].copy_from_slice(&OTHER_IEEE);
    let foreign = resign(&foreign);
    for flags in [0u8, 1] {
        let restore = host.request(&BkpRestoreBegin {
            flags,
            nwk_margin: 0,
            aps_margin: 0,
            expected_records: 6,
        });
        exchange!();
        let restore = host.response(restore);
        let txn = restore.body::<BkpRestoreBeginRsp>().txn;
        let mut payload = txn.to_le_bytes().to_vec();
        payload.extend_from_slice(&foreign);
        let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
        let commit = host.request(&BkpRestoreCommit { txn });
        exchange!();
        if flags == 0 {
            assert_eq!(
                host.response(put).status(),
                Status::InvalidParameter.to_u8()
            );
            assert_eq!(host.response(commit).status(), Status::NotFound.to_u8());
        } else {
            assert_eq!(host.response(put).status(), 0);
            let commit = host.response(commit);
            let commit: BkpRestoreCommitRsp = commit.body();
            assert_eq!(commit.link_keys_host_backed, 0);
        }
    }
    assert_eq!(platform.ieee_override, Some(OTHER_IEEE));
    assert_eq!(port.ieee(), OTHER_IEEE);
    let info = host.request_raw(ids::NWK_GET_INFO, &[]);
    exchange!();
    let info = host.response(info);
    let info: NwkGetInfoRsp = info.body();
    assert_eq!(
        (info.state, info.pan_id, info.ieee.0),
        (2, 0x2B73, OTHER_IEEE)
    );
}

/// Recompute `END` over the other records of `stream`.
fn resign(stream: &[u8]) -> Vec<u8> {
    let mut crc = StreamCrc::new();
    let mut out = Vec::new();
    for item in RawRecords::new(stream) {
        let (kind, tlv) = item.unwrap();
        if kind == record_type::END {
            break;
        }
        crc.add(tlv);
        out.extend_from_slice(tlv);
    }
    let mut end = [0u8; 16];
    let mut w = Writer::new(&mut end);
    Record::End(crc.end()).write(&mut w).unwrap();
    let len = w.len();
    out.extend_from_slice(&end[..len]);
    out
}

#[test]
fn firmware_ieee_pta_and_retry_cache() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);

    // FW_*: stage, verify, activate (reboot after the delay), confirm.
    let image: Vec<u8> = (0..1300u32).map(|i| i as u8).collect();
    let begin = host.request(&FwBeginReq {
        image_size: image.len() as u32,
        format: 1,
        sha256: [0x11; 32],
        version: 2,
        flags: 0,
    });
    exchange!();
    let begin = host.response(begin);
    let begin: FwBeginRsp = begin.body();
    assert_eq!((begin.max_chunk, begin.slot), (512, 1), "capped to a frame");
    let mut offset = 0;
    for chunk in image.chunks(usize::from(begin.max_chunk)) {
        let write = host.request(&FwWrite {
            offset,
            data: Rest(chunk),
        });
        exchange!();
        let write = host.response(write);
        offset = write.body::<FwWriteRsp>().next_offset;
    }
    assert_eq!(offset, 1300);
    let bad = host.request(&FwWrite {
        offset: 7,
        data: Rest(&[0]),
    });
    let finish = host.request_raw(ids::FW_FINISH, &[]);
    let slots = host.request_raw(ids::FW_GET_SLOTS, &[]);
    exchange!();
    assert_eq!(
        host.response(bad).status(),
        Status::InvalidParameter.to_u8()
    );
    let finish = host.response(finish);
    assert_eq!(finish.body::<FwFinishRsp>().verify_result, 0);
    let slots = host.response(slots);
    let slots: FwGetSlotsRsp = slots.body();
    let states: Vec<_> = slots
        .slots
        .iter()
        .map(|s| (s.slot, s.state, s.version))
        .collect();
    assert_eq!(states, [(0, 1, 1), (1, 3, 2)]);
    let activate = host.request(&FwActivate { delay_ms: 5 });
    let requests = exchange!();
    assert_eq!(host.response(activate).status(), 0);
    assert!(platform.activated);
    assert!(
        requests.contains(&PlatformRequest::Reset { mode: 0 }),
        "{requests:?}"
    );
    let confirm = host.request_raw(ids::FW_CONFIRM, &[]);
    exchange!();
    assert_eq!(host.response(confirm).status(), 0);
    assert!(platform.confirmed);

    // SYS_SET_IEEE before formation: stored and applied to the MAC; all-FF
    // reverts to the factory IEEE.
    let set = host.request(&SysSetIeee {
        ieee: Ieee(OTHER_IEEE),
    });
    let zero = host.request(&SysSetIeee { ieee: Ieee([0; 8]) });
    exchange!();
    assert_eq!(host.response(set).status(), 0);
    assert_eq!(
        host.response(zero).status(),
        Status::InvalidParameter.to_u8()
    );
    assert_eq!(platform.ieee_override, Some(OTHER_IEEE));
    assert_eq!(port.ieee(), OTHER_IEEE);
    let info = host.request_raw(ids::SYS_GET_INFO, &[]);
    exchange!();
    let info = host.response(info);
    let info: SysGetInfoRsp = info.body();
    assert_eq!(
        (info.ieee_factory.0, info.ieee_active.0, info.ieee_source),
        (IEEE, OTHER_IEEE, 1)
    );
    let revert = host.request(&SysSetIeee {
        ieee: Ieee([0xFF; 8]),
    });
    exchange!();
    assert_eq!(host.response(revert).status(), 0);
    assert_eq!((platform.ieee_override, port.ieee()), (None, IEEE));

    // PTA: off by default, enabled with a priority mask.
    let get = host.request(&SysGetConfig {
        ids: List::new(&[config::COEX_ENABLE, config::COEX_TX_PRIORITY_MASK]),
    });
    exchange!();
    let get = host.response(get);
    let get: SysGetConfigRsp = get.body();
    let values: Vec<_> = get.items.iter().map(|i| i.value.0.to_vec()).collect();
    assert_eq!(values, [vec![0], vec![0]]);
    let set = host.request(&SysSetConfig {
        values: List::new(&[
            ConfigValue {
                id: config::COEX_TX_PRIORITY_MASK,
                value: Bytes8(&[0x05]),
            },
            ConfigValue {
                id: config::COEX_ENABLE,
                value: Bytes8(&[1]),
            },
            ConfigValue {
                id: config::COEX_ENABLE,
                value: Bytes8(&[2]),
            },
        ]),
    });
    let status = host.request_raw(ids::SYS_GET_STATUS, &[]);
    exchange!();
    let set = host.response(set);
    let set: SysSetConfigRsp = set.body();
    let results: Vec<_> = set.results.iter().map(|r| r.status).collect();
    assert_eq!(
        results,
        [Status::Success, Status::Success, Status::InvalidParameter]
    );
    assert_eq!(platform.pta, Some((true, 0x05)));
    let status = host.response(status);
    assert_eq!(status.body::<SysGetStatusRsp>().flags & 0x04, 0x04);

    // Retry cache: a repeated NWK_FORM is answered from the cache and the
    // network is formed once.
    let request = form_request(0x3C84, 1 << 15);
    let form = host.request(&request);
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    host.retry(form, &request);
    exchange!();
    assert_eq!(host.responses(form), 1);
    assert_eq!(
        host.response(form).status(),
        0,
        "cached success, not INVALID_STATE"
    );
    // A new request with the same TID but no RETRY flag runs again.
    host.queues
        .queue(Endpoint::Zigbee)
        .push_with(|w| w.request(form, &request))
        .unwrap();
    exchange!();
    assert_eq!(host.response(form).status(), Status::InvalidState.to_u8());
    // FW_CONFIRM with a formed network still answers (network is up).
    let confirm = host.request_raw(ids::FW_CONFIRM, &[]);
    exchange!();
    assert_eq!(host.response(confirm).status(), 0);
    // NWK_LEAVE_RESET, then a new network in the same session (no reboot).
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);
    let form = host.request(&form_request(0x4D95, 1 << 25));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let info = host.request_raw(ids::NWK_GET_INFO, &[]);
    exchange!();
    let info = host.response(info);
    let info: NwkGetInfoRsp = info.body();
    assert_eq!((info.state, info.pan_id, info.channel), (2, 0x4D95, 25));
}

#[test]
fn restore_keeps_ncp_children_and_their_keys() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x5EA7, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let begin = host.request(&BkpExportBegin { flags: 0x03 });
    exchange!();
    let begin = host.response(begin);
    let begin: BkpExportBeginRsp = begin.body();
    let read = host.request(&BkpExportRead {
        txn: begin.txn,
        max_bytes: 480,
    });
    exchange!();
    let read = host.response(read);
    let stream = read.payload[2..].to_vec();
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);

    // The backup of an NCP with one sleepy child holding a unique TCLK.
    let child = Record::Device(DeviceRecord {
        ieee: Ieee(DEVICE_IEEE),
        nwk: 0x4C21,
        flags: 0,
        child: Some(zigbee_zrhp::records::ChildInfo {
            capability: 0x80,
            timeout_exp: 9,
        }),
        link: Some(LinkBlock {
            key: DEVICE_KEY,
            key_seq: 0,
            attrs: 2,
            origin: 2,
            tx_counter: 0,
            rx_floor: 0,
        }),
    });
    let mut encoded = [0u8; 64];
    let mut w = Writer::new(&mut encoded);
    child.write(&mut w).unwrap();
    let len = w.len();
    let mut with_child = Vec::new();
    for item in RawRecords::new(&stream) {
        let (kind, tlv) = item.unwrap();
        if kind == record_type::RS_PARAMS {
            with_child.extend_from_slice(&encoded[..len]);
        }
        with_child.extend_from_slice(tlv);
    }
    let with_child = resign(&with_child);
    let restore = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 0,
        aps_margin: 0,
        expected_records: 7,
    });
    exchange!();
    let restore = host.response(restore);
    let txn = restore.body::<BkpRestoreBeginRsp>().txn;
    let mut payload = txn.to_le_bytes().to_vec();
    payload.extend_from_slice(&with_child);
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    let commit = host.request(&BkpRestoreCommit { txn });
    exchange!();
    let put = host.response(put);
    let put: BkpRestorePutRsp = put.body();
    assert_eq!(put.notes.iter().count(), 0, "the child is not demoted");
    let commit = host.response(commit);
    let commit: BkpRestoreCommitRsp = commit.body();
    assert_eq!((commit.children, commit.link_keys_on_ncp), (1, 1));

    // Restored as a child at its old address, with its key.
    let view = port.device(0).expect("restored device");
    assert_eq!(view.ieee, DEVICE_IEEE);
    assert_eq!(view.nwk, Some(0x4C21));
    assert_eq!(view.child, Some((0x80, 9)));
    let link = view.link.expect("restored key");
    assert_eq!(link.key, DEVICE_KEY);
    // A verified generated key stays one (not "provisioned"): a
    // factory-reset device re-pairing gets the network key under the
    // global key, and the device may secure-rejoin.
    assert_eq!((link.origin, link.attrs), (2, 2));

    // The coordinator's tables for hosts (spec 7.8; zigpy Z13, herdsman
    // H6): the restored child appears as a child neighbour.
    let neighbors = host.request(&DiagGetNeighbors { start: 0 });
    let children = host.request(&DiagGetChildren { start: 0 });
    let routes = host.request(&DiagGetRoutes { start: 0 });
    let past_end = host.request(&DiagGetChildren { start: 5 });
    exchange!();
    let neighbors = host.response(neighbors);
    let neighbors: DiagGetNeighborsRsp = neighbors.body();
    assert_eq!(neighbors.total, 1);
    let neighbor = neighbors.entries.iter().next().unwrap();
    assert_eq!(
        (
            neighbor.ieee.0,
            neighbor.nwk,
            neighbor.relationship,
            neighbor.device_type
        ),
        (DEVICE_IEEE, 0x4C21, 1, 2)
    );
    let children = host.response(children);
    let children: DiagGetChildrenRsp = children.body();
    let child = children.entries.iter().next().unwrap();
    assert_eq!(children.total, 1);
    assert_eq!(
        (child.ieee.0, child.nwk, child.timeout_exp),
        (DEVICE_IEEE, 0x4C21, 9)
    );
    assert_eq!(child.capability & 0x88, 0x80, "a sleepy end device");
    assert_eq!(child.last_poll_ms, u32::MAX, "not heard since the restore");
    let routes = host.response(routes);
    let routes: DiagGetRoutesRsp = routes.body();
    assert_eq!(routes.entries.iter().count(), usize::from(routes.total));
    let past_end = host.response(past_end);
    let past_end: DiagGetChildrenRsp = past_end.body();
    assert_eq!((past_end.total, past_end.entries.iter().count()), (1, 0));
}

#[test]
fn restoring_an_old_backup_never_lowers_the_outgoing_counters() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x6B11, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let begin = host.request(&BkpExportBegin { flags: 0x03 });
    exchange!();
    let begin = host.response(begin);
    let begin: BkpExportBeginRsp = begin.body();
    let read = host.request(&BkpExportRead {
        txn: begin.txn,
        max_bytes: 480,
    });
    exchange!();
    let read = host.response(read);
    let stream = read.payload[2..].to_vec();

    // The NCP keeps running well past the backup (host floors stand in
    // for weeks of traffic).
    let init = host.request(&zigbee_zrhp::msg::network::NwkInit {
        nwk_fc_floor: 0x20_0000,
        aps_fc_floor: 0x30_0000,
    });
    exchange!();
    assert_eq!(host.response(init).status(), 0);
    let used = port.frame_counters();
    assert!(used.nwk_current >= 0x20_0000 && used.aps_current >= 0x30_0000);

    // Wipe, then restore the old backup without margins: the counters
    // must not go back below what this NCP already used with these keys.
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);
    let restore = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 0,
        aps_margin: 0,
        expected_records: 6,
    });
    exchange!();
    let restore = host.response(restore);
    let txn = restore.body::<BkpRestoreBeginRsp>().txn;
    let mut payload = txn.to_le_bytes().to_vec();
    payload.extend_from_slice(&stream);
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    let commit = host.request(&BkpRestoreCommit { txn });
    exchange!();
    assert_eq!(host.response(put).status(), 0);
    let commit = host.response(commit);
    let commit: BkpRestoreCommitRsp = commit.body();
    assert!(commit.nwk_fc_start >= used.nwk_limit, "{commit:?} {used:?}");
    assert!(commit.aps_fc_start >= used.aps_limit, "{commit:?} {used:?}");
    let after = port.frame_counters();
    assert!(after.nwk_current >= used.nwk_limit);
    assert!(after.aps_current >= used.aps_limit);
}

/// One NCP boot over `security` (the persistent store): negotiates and
/// runs `body` with the NCP, host and an `exchange` closure-like macro.
macro_rules! boot {
    ($security:ident, |$ncp:ident, $host:ident, $port:ident, $platform:ident, $pipe:ident,
      $log:ident, $now:ident| $body:block) => {{
        let mut gp_store = zigbee_runtime::gp_store::RamGpProxyStore::new();
        let mut profile = profile();
        let mut device = device(&profile);
        let node = ZigbeeNode::new(&mut device, &mut $security, &mut profile);
        let mut app = TrustCenterCoordinatorApp::new(
            node,
            PersistentChildren::new(RamChildTableStore::new()),
            RamTrustCenterDeviceStore::new(),
            &POLICY,
            RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
        )
        .unwrap();
        app.set_green_power_store(&mut gp_store);
        let mut $port = TcPort::new(app, IEEE, clock_ms);
        let $log = LogRing::<8>::new();
        let mut $pipe = Pipe::default();
        let mut $platform = FakePlatform::default();
        let mut $now = 1_000u64;
        let mut $ncp = Ncp::<8>::new(identity(), 0xC0FF_EE05, $now);
        let mut $host = Host::new($now);
        exchange(
            &mut $ncp,
            &mut $host,
            &mut $port,
            &mut $platform,
            &mut $pipe,
            &$log,
            &mut $now,
        );
        assert!($host.event(ids::EVT_NCP_READY).is_some());
        let negotiate = $host.request(&SysNegotiateReq {
            major_min: 1,
            major_max: 1,
            host_caps: 0,
            name: Bytes8(b"surface"),
        });
        exchange(
            &mut $ncp,
            &mut $host,
            &mut $port,
            &mut $platform,
            &mut $pipe,
            &$log,
            &mut $now,
        );
        let _: SysNegotiateRsp = $host.response(negotiate).body();
        $body
    }};
}

/// Spec 7.2 / 7.11 step 4 (zigpy Z11): after a reboot and before
/// `NWK_INIT`, `NWK_GET_INFO` reports the committed network's identity
/// with `state` 1 (formed, not up) instead of an empty NIB.
#[test]
fn network_info_after_a_reboot_reports_the_stored_identity() {
    let mut security = RamSecurityStateStore::new();
    boot!(security, |ncp, host, port, platform, pipe, log, now| {
        let form = host.request(&form_request(0x1A62, 1 << 20));
        exchange(
            &mut ncp,
            &mut host,
            &mut port,
            &mut platform,
            &mut pipe,
            &log,
            &mut now,
        );
        assert_eq!(host.response(form).status(), 0);
    });
    boot!(security, |ncp, host, port, platform, pipe, log, now| {
        let info = host.request_raw(ids::NWK_GET_INFO, &[]);
        exchange(
            &mut ncp,
            &mut host,
            &mut port,
            &mut platform,
            &mut pipe,
            &log,
            &mut now,
        );
        let info = host.response(info);
        let info: NwkGetInfoRsp = info.body();
        assert_eq!(
            (info.state, info.pan_id, info.ext_pan_id, info.channel),
            (1, 0x1A62, EPID, 20)
        );
        assert_eq!((info.key_seq, info.nwk_update_id), (3, 2));
        assert_eq!(info.ieee.0, IEEE);
    });
    // A factory-new NCP still reports nothing.
    let mut empty = RamSecurityStateStore::new();
    boot!(empty, |ncp, host, port, platform, pipe, log, now| {
        let info = host.request_raw(ids::NWK_GET_INFO, &[]);
        exchange(
            &mut ncp,
            &mut host,
            &mut port,
            &mut platform,
            &mut pipe,
            &log,
            &mut now,
        );
        let info = host.response(info);
        let info: NwkGetInfoRsp = info.body();
        assert_eq!(info.state, 0);
    });
}

/// `SEC_ADD_INSTALL_CODE` / `SEC_ADD_LINK_KEY` (spec 7.5; herdsman H6):
/// the derived key (Silicon Labs vector) is returned and provisioned for
/// the device; a bad CRC is refused; a unique TCLK is provisioned with its
/// attributes; unsupported variants are refused, not silently accepted.
#[test]
fn install_codes_and_link_keys_are_provisioned() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    const CODE: [u8; 18] = [
        0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0xD4, 0x90,
    ];
    const DERIVED: [u8; 16] = [
        0xFA, 0x80, 0x81, 0xCA, 0xAA, 0x41, 0xD5, 0xAD, 0xE9, 0xB5, 0x65, 0x87, 0x99, 0x26, 0x8B,
        0x88,
    ];
    const CODE_DEVICE: [u8; 8] = [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38];
    const KEY_DEVICE: [u8; 8] = [0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48];

    // Not before the network runs.
    let early = host.request(&SecAddInstallCode {
        ieee: Ieee(CODE_DEVICE),
        code: Bytes8(&CODE),
    });
    exchange!();
    assert_eq!(host.response(early).status(), Status::InvalidState.to_u8());

    let form = host.request(&form_request(0x2C11, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let mut bad = CODE;
    bad[0] ^= 1;
    let rejected = host.request(&SecAddInstallCode {
        ieee: Ieee(CODE_DEVICE),
        code: Bytes8(&bad),
    });
    let added = host.request(&SecAddInstallCode {
        ieee: Ieee(CODE_DEVICE),
        code: Bytes8(&CODE),
    });
    let key = host.request(&SecAddLinkKey {
        ieee: Ieee(KEY_DEVICE),
        key: DEVICE_KEY,
        key_type: 1,
        attrs: 1,
        rx_floor: 0,
        flags: 1,
    });
    let application = host.request(&SecAddLinkKey {
        ieee: Ieee(KEY_DEVICE),
        key: DEVICE_KEY,
        key_type: 3,
        attrs: 0,
        rx_floor: 0,
        flags: 0,
    });
    exchange!();
    assert_eq!(
        host.response(rejected).status(),
        Status::InvalidParameter.to_u8()
    );
    let added = host.response(added);
    let added: SecAddInstallCodeRsp = added.body();
    assert_eq!(added.derived_key, DERIVED);
    assert_eq!(host.response(key).status(), 0);
    assert_eq!(
        host.response(application).status(),
        Status::UnsupportedCommand.to_u8()
    );

    let keys: Vec<_> = (0..port.device_capacity())
        .filter_map(|index| port.device(index))
        .filter_map(|view| view.link.map(|link| (view.ieee, link.key, link.origin)))
        .collect();
    assert!(keys.contains(&(CODE_DEVICE, DERIVED, 1)), "{keys:?}");
    assert!(keys.contains(&(KEY_DEVICE, DEVICE_KEY, 3)), "{keys:?}");
}

/// A reboot that falls due while the product drives a queued operation is
/// not lost: the composition roots ignore the `service` result inside their
/// drive loop, so the request stays armed until the outer `service` takes
/// it (a lost one left `SYS_RESET` unanswered until the host reset the NCP
/// by its lines).
#[test]
fn a_reset_due_during_a_queued_operation_is_kept() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let reset = host.request(&SysReset {
        mode: 0,
        confirm: 0,
    });
    let set = host.request(&SysSetIeee {
        ieee: Ieee(OTHER_IEEE),
    });
    now += 1_000;
    host.pump(now, &mut pipe);
    assert_eq!(
        ncp.service_with(now, &mut pipe, &mut port, &mut platform, &log),
        None
    );
    // The operation runs after the reset delay; the product loop drops the
    // in-loop `service` result.
    now += 100_000;
    while pollster::block_on(ncp.drive_with(&mut port, &mut platform, || 0)) {
        let _ = ncp.service_with(now, &mut pipe, &mut port, &mut platform, &log);
    }
    host.pump(now, &mut pipe);
    now += 1_000;
    assert_eq!(
        ncp.service_with(now, &mut pipe, &mut port, &mut platform, &log),
        Some(PlatformRequest::Reset { mode: 0 })
    );
    host.pump(now, &mut pipe);
    assert_eq!(host.response(reset).status(), 0);
    assert_eq!(host.response(set).status(), 0);
}

#[test]
fn group_membership_edits_the_aps_group_table() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let early = host.request(&NwkGroupMembership {
        op: 1,
        group: 0x0B84,
        ep: 242,
    });
    exchange!();
    assert_eq!(host.response(early).status(), Status::InvalidState.to_u8());
    let form = host.request(&form_request(0x1A62, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);

    macro_rules! run {
        ($op:expr, $group:expr, $ep:expr) => {{
            let request = host.request(&NwkGroupMembership {
                op: $op,
                group: $group,
                ep: $ep,
            });
            exchange!();
            host.response(request).status()
        }};
    }
    assert_eq!(
        run!(1, 0x0B84, 242),
        0,
        "the GP endpoint joins the sink group"
    );
    assert_eq!(run!(1, 0x1234, 1), 0);
    assert_eq!(run!(1, 0x1234, 1), 0, "idempotent");
    assert_eq!(run!(1, 0x1234, 2), 0);
    assert_eq!(run!(1, 0x1234, 241), Status::InvalidParameter.to_u8());
    assert_eq!(run!(1, 0xFFF8, 1), Status::InvalidParameter.to_u8());
    assert_eq!(run!(9, 0x1234, 1), Status::InvalidParameter.to_u8());
    {
        let groups = TcApp::device(port.app()).aps().group_table();
        assert!(groups.is_member(0x0B84, 242));
        assert!(groups.is_member(0x1234, 1) && groups.is_member(0x1234, 2));
    }
    assert_eq!(run!(2, 0x1234, 2), 0);
    assert_eq!(run!(2, 0x1234, 2), 0, "removing a non-member is a no-op");
    assert_eq!(run!(3, 0, 1), 0, "endpoint 1 leaves every group");
    {
        let groups = TcApp::device(port.app()).aps().group_table();
        assert!(!groups.is_member(0x1234, 1) && !groups.is_member(0x1234, 2));
        assert!(groups.is_member(0x0B84, 242));
    }
    let mut status = 0;
    for group in 1..=32u16 {
        status = run!(1, group, 1);
        if status != 0 {
            break;
        }
    }
    assert_eq!(status, Status::NoSpace.to_u8(), "a full table says so");
}

/// `NWK_CHANGE_CHANNEL` (R22 2.4.3.3.9): the Mgmt_NWK_Update_req broadcast
/// and the NV commit come first, the radio follows after
/// `nwkNetworkBroadcastDeliveryTime`, then `EVT_CHANNEL_CHANGED`.
#[test]
fn channel_change_announces_commits_then_switches() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let early = host.request(&NwkChangeChannel {
        channel: 25,
        nwk_update_id: 0,
    });
    exchange!();
    assert_eq!(host.response(early).status(), Status::InvalidState.to_u8());
    let form = host.request(&form_request(0x1A62, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    assert_eq!(port.info().channel, 20);
    let update_id = port.info().nwk_update_id;
    let sent = TcApp::device(port.app())
        .aps()
        .nwk()
        .mac()
        .tx_history()
        .len();

    let change = host.request(&NwkChangeChannel {
        channel: 25,
        nwk_update_id: 0,
    });
    let bad = host.request(&NwkChangeChannel {
        channel: 27,
        nwk_update_id: 0,
    });
    exchange!();
    assert_eq!(host.response(change).status(), 0);
    assert_eq!(
        host.response(bad).status(),
        Status::InvalidParameter.to_u8()
    );
    let history = TcApp::device(port.app()).aps().nwk().mac().tx_history();
    assert!(
        history[sent..].iter().any(|tx| matches!(
            tx.dst,
            zigbee_types::MacAddress::Short(_, zigbee_types::ShortAddress(0xFFFF))
        )),
        "Mgmt_NWK_Update_req broadcast"
    );
    // Announced: NV and the update ID moved, the radio not yet.
    let stored = {
        let (_, store) = TcApp::device_and_store_mut(port.app_mut());
        zigbee_runtime::security_store::SecurityStateStore::load(store)
            .unwrap()
            .unwrap()
    };
    assert_eq!(
        (stored.channel, stored.update_id),
        (25, update_id.wrapping_add(1))
    );
    // A security checkpoint inside the delivery interval keeps the new
    // channel (the NWK holds it as pending).
    let stored = {
        let (device, store) = TcApp::device_and_store_mut(port.app_mut());
        device.refresh_security_state(store).unwrap();
        zigbee_runtime::security_store::SecurityStateStore::load(store)
            .unwrap()
            .unwrap()
    };
    assert_eq!(stored.channel, 25);
    assert_eq!(port.info().channel, 20);
    assert_eq!(port.info().nwk_update_id, update_id.wrapping_add(1));
    assert!(host.event(ids::EVT_CHANNEL_CHANGED).is_none());
    let again = host.request(&NwkChangeChannel {
        channel: 15,
        nwk_update_id: 0,
    });
    exchange!();
    assert_eq!(host.response(again).status(), Status::Busy.to_u8());

    // nwkNetworkBroadcastDeliveryTime later. The test loop's drive clock is
    // 0 (the switch is due at 9 s on it); run the product loop with the
    // real one around that point.
    macro_rules! run_until {
        ($at:expr) => {
            let at: u64 = $at;
            while now < at + 3_000 {
                // Small steps keep the link's keepalives flowing.
                now = (now + 50_000).min(at + 3_000);
                host.pump(now, &mut pipe);
                let _ = ncp.service_with(now, &mut pipe, &mut port, &mut platform, &log);
                while pollster::block_on(ncp.drive_with(&mut port, &mut platform, || now)) {}
                let _ = ncp.service_with(now, &mut pipe, &mut port, &mut platform, &log);
                host.pump(now, &mut pipe);
            }
        };
    }
    run_until!(8_900_000);
    assert!(host.event(ids::EVT_CHANNEL_CHANGED).is_none());
    assert_eq!(port.info().channel, 20);
    run_until!(9_000_000);
    let changed: EvtChannelChanged = host.event(ids::EVT_CHANNEL_CHANGED).unwrap().event();
    assert_eq!(
        (changed.channel, changed.nwk_update_id, changed.req_tid),
        (25, update_id.wrapping_add(1), change)
    );
    assert_eq!(port.info().channel, 25);

    // The current channel: done at once, no broadcast.
    let same = host.request(&NwkChangeChannel {
        channel: 25,
        nwk_update_id: 0,
    });
    exchange!();
    assert_eq!(host.response(same).status(), 0);
    let changed: EvtChannelChanged = host.event(ids::EVT_CHANNEL_CHANGED).unwrap().event();
    assert_eq!((changed.channel, changed.req_tid), (25, same));
}

fn counter_values(received: Received) -> Vec<(u16, u32)> {
    let diag: DiagGetCountersRsp = received.body();
    diag.counters.iter().map(|c| (c.id, c.value)).collect()
}

#[test]
fn energy_scan_tx_power_and_stack_counters() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let channels = (1 << 15) | (1 << 20) | (1 << 25);

    // Before a network exists; a second scan while one runs is refused.
    let scan = host.request(&NwkEnergyScan {
        channel_mask: channels,
        duration_exp: 2,
        count: 2,
    });
    let again = host.request(&NwkEnergyScan {
        channel_mask: channels,
        duration_exp: 2,
        count: 1,
    });
    let bad = host.request(&NwkEnergyScan {
        channel_mask: 1 << 10,
        duration_exp: 2,
        count: 1,
    });
    exchange!();
    exchange!();
    let response = host
        .inbox
        .iter()
        .position(|m| m.kind == Kind::Response && m.tid == scan)
        .unwrap();
    let event = host
        .inbox
        .iter()
        .position(|m| m.kind == Kind::Event && m.msg_id == ids::EVT_ENERGY_SCAN_RESULT)
        .expect("EVT_ENERGY_SCAN_RESULT");
    assert!(response < event, "the result follows the response");
    assert_eq!(host.response(scan).status(), 0);
    assert_eq!(host.response(again).status(), Status::InProgress.to_u8());
    assert_eq!(
        host.response(bad).status(),
        Status::InvalidParameter.to_u8()
    );
    let result = host.event(ids::EVT_ENERGY_SCAN_RESULT).unwrap();
    let result: EvtEnergyScanResult = result.event();
    assert_eq!((result.req_tid, result.status), (scan, 0));
    let results: Vec<_> = result
        .results
        .iter()
        .map(|r| (r.channel, r.energy, r.max_rssi_dbm))
        .collect();
    // MockMac energies of `device()`.
    assert_eq!(results, [(15, 60, -40), (20, 5, -95), (25, 90, -10)]);

    // With the network up, on the operating channel afterwards.
    let form = host.request(&form_request(0x2B33, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let scan = host.request(&NwkEnergyScan {
        channel_mask: channels,
        duration_exp: 4,
        count: 1,
    });
    exchange!();
    exchange!();
    assert_eq!(host.response(scan).status(), 0);
    let result = host.event(ids::EVT_ENERGY_SCAN_RESULT).unwrap();
    let result: EvtEnergyScanResult = result.event();
    assert_eq!((result.status, result.results.len()), (0, 3));
    assert_eq!(port.info().channel, 20);
    assert_eq!(
        port.info().state,
        ncp_coordinator_app::port::NetworkState::Up
    );

    // Transmit power: stepped and clamped by the platform, reported by
    // NWK_GET_INFO, kept across a formation with tx_power 0.
    let set = host.request(&NwkSetTxPower { dbm: 7 });
    let high = host.request(&NwkSetTxPower { dbm: 20 });
    exchange!();
    let applied: NwkSetTxPowerRsp = host.response(set).body();
    assert_eq!(applied.applied_dbm, 6);
    let applied: NwkSetTxPowerRsp = host.response(high).body();
    assert_eq!(applied.applied_dbm, 10);
    let info = host.request_raw(ids::NWK_GET_INFO, &[]);
    exchange!();
    let info: NwkGetInfoRsp = host.response(info).body();
    assert_eq!(info.tx_power, 10);
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);
    let form = host.request(&form_request(0x2B34, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let info = host.request_raw(ids::NWK_GET_INFO, &[]);
    exchange!();
    let info: NwkGetInfoRsp = host.response(info).body();
    assert_eq!(info.tx_power, 10, "NWK_FORM tx_power 0 keeps the power");

    // Counter groups: MAC from the platform, NWK from the stack, APS and
    // security from the NCP; each reset by its own group bit.
    platform.mac_tx = 9;
    let counters = host.request(&DiagGetCounters { group_mask: 0x1E });
    exchange!();
    let values = counter_values(host.response(counters));
    let ids: Vec<u16> = values.iter().map(|(id, _)| *id).collect();
    assert_eq!(
        ids,
        [
            0x0201, 0x0301, 0x0302, 0x0303, 0x0304, 0x0305, 0x0306, 0x0401, 0x0402, 0x0403, 0x0501,
            0x0502, 0x0503, 0x0504, 0x0505
        ]
    );
    assert_eq!(values[0], (0x0201, 9));
    let reset = host.request(&DiagResetCounters { group_mask: 0x02 });
    exchange!();
    assert_eq!(host.response(reset).status(), 0);
    assert_eq!(platform.mac_tx, 0);
    // Every counter of every group fits one response.
    let counters = host.request(&DiagGetCounters { group_mask: 0xFF });
    exchange!();
    let all = counter_values(host.response(counters));
    assert!(
        all.len() < ncp_coordinator_app::ncp::MAX_COUNTERS,
        "{}",
        all.len()
    );
}
