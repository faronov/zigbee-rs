//! The rest of the NCP surface over the zigbee-zrhp host link: host-supplied
//! formation, TABLES, BACKUP export → restore, FW_* and SYS_SET_IEEE/PTA on
//! a fake platform, and the retry cache.

use std::collections::VecDeque;

use ncp_coordinator_app::platform::{
    FwBegin, FwImage, FwSlot, NcpPlatform, PlatformError, slot_state,
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
    export_flags, note,
};
use zigbee_zrhp::msg::diag::{
    DiagGetChildren, DiagGetChildrenRsp, DiagGetCounters, DiagGetCountersRsp, DiagGetNeighbors,
    DiagGetNeighborsRsp, DiagGetRoutes, DiagGetRoutesRsp, DiagResetCounters,
};
use zigbee_zrhp::msg::firmware::{
    FwActivate, FwBegin as FwBeginReq, FwBeginRsp, FwFinishRsp, FwGetSlotsRsp, FwWrite, FwWriteRsp,
};
use zigbee_zrhp::msg::network::{
    EvtChannelChanged, EvtEnergyScanResult, EvtScanProgress, NwkActiveScan, NwkActiveScanRsp,
    NwkChangeChannel, NwkEnergyScan, NwkForm, NwkGetInfoRsp, NwkGroupMembership, NwkLeaveReset,
    NwkSetTxPower, NwkSetTxPowerRsp, leave_reset_flags,
};
use zigbee_zrhp::msg::security::{
    SecAddInstallCode, SecAddInstallCodeRsp, SecAddLinkKey, SecDeriveEzspKey, SecDeriveEzspKeyRsp,
    SecGetFrameCountersRsp, SecGetNetworkKey, SecGetNetworkKeyRsp,
};
use zigbee_zrhp::msg::system::capacity;
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
use zigbee_zrhp::records::{
    ApsGroupRecord, DeviceRecord, LinkBlock, RawRecords, Record, StreamCrc, record_type,
};
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
    /// PTA turned off by the platform itself (fail closed after a fault).
    pta_failed: bool,
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
        if enable && self.pta_failed {
            return Err(PlatformError::Hardware);
        }
        self.pta = Some((enable, priority_mask));
        Ok(())
    }
    fn pta_active(&self) -> Option<bool> {
        Some(self.pta.is_some_and(|(enable, _)| enable) && !self.pta_failed)
    }
    /// A radio with 2 dB steps from -20 to +10 dBm.
    fn tx_power_step(&self, dbm: i8) -> Option<i8> {
        Some(dbm.clamp(-20, 10) & !1)
    }
    fn energy_to_dbm(&self, energy: u8) -> i8 {
        energy as i8 - 100
    }
    fn lqi_to_dbm(&self, lqi: u8) -> i8 {
        (lqi / 4) as i8 - 100
    }
    fn visit_counters(&self, group_mask: u8, sink: &mut dyn FnMut(u16, u32)) {
        if group_mask & 0x02 != 0 {
            // As the ESP32 product: eight MAC counters.
            sink(0x0201, self.mac_tx);
            for id in 0x0202..=0x0208 {
                sink(id, 0);
            }
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
            caps::ACTIVE_SCAN,
            caps::APS_GROUP_TABLE,
            caps::EZSP_HASHED_TCLK,
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

/// The NWK command ID of a NWK-secured command frame the NCP sent,
/// decrypted with `key` (the command ID is in the secured payload).
fn nwk_command(key: &[u8; 16], frame: &[u8]) -> Option<u8> {
    use zigbee_nwk::frames::{NwkFrameType, NwkHeader};
    use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};
    let (header, header_len) = NwkHeader::parse(frame)?;
    if header.frame_control.frame_type != NwkFrameType::Command as u8 {
        return None;
    }
    let (mut security, security_len) = NwkSecurityHeader::parse(&frame[header_len..])?;
    // The level is sent as 0 and restored for the nonce and the MIC.
    security.security_control = (security.security_control & !0x07) | 0x05;
    let mut aad = frame[..header_len + security_len].to_vec();
    aad[header_len] = security.security_control;
    let plain =
        NwkSecurity::new().decrypt(&aad, &frame[header_len + security_len..], key, &security)?;
    plain.first().copied()
}

/// NWK Leave commands (`0x04`) among `frames`.
fn leaves(frames: &[zigbee_mac::mock::TxRecord]) -> usize {
    frames
        .iter()
        .filter(|tx| nwk_command(&NETWORK_KEY, tx.payload.as_slice()) == Some(0x04))
        .count()
}

fn mock_mac<A: TcApp<Mac = MockMac>>(port: &mut TcPort<A>) -> &mut MockMac {
    port.app_mut().device_mut().mac_mut()
}

/// NWK Leave commands sent since the history was last cleared, including
/// the ones a MAC reset (the wipe's MLME-RESET) took out of the history.
fn leaves_sent<A: TcApp<Mac = MockMac>>(port: &mut TcPort<A>) -> usize {
    let mac = mock_mac(port);
    leaves(mac.tx_history_before_reset()) + leaves(mac.tx_history())
}

/// Forget every recorded transmission.
fn forget_sent<A: TcApp<Mac = MockMac>>(port: &mut TcPort<A>) {
    mock_mac(port).clear_tx_history();
}

/// A host wipe transmits nothing: before a restore (the coordinator's
/// Leave broadcast made the routers of a restored network drop it, hub HIL
/// on a Dongle Max), the 9.4 rule 1 wipe of a restore committed with the
/// network up, `NWK_LEAVE_RESET` without flags bit2 (also the path of
/// `SYS_RESET` mode 2). Only bit2, an explicit "leave network", broadcasts
/// the NWK Leave.
#[test]
fn host_wipes_transmit_no_leave_unless_it_is_announced() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x5EA7, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let begin = host.request(&BkpExportBegin { flags: 0x03 });
    exchange!();
    let begin: BkpExportBeginRsp = host.response(begin).body();
    let mut stream = Vec::new();
    loop {
        let read = host.request(&BkpExportRead {
            txn: begin.txn,
            max_bytes: 64,
        });
        exchange!();
        let read = host.response(read);
        stream.extend_from_slice(&read.payload[2..]);
        if read.payload[1] == 0 {
            break;
        }
    }
    let end = host.request(&BkpExportEnd { txn: begin.txn });
    exchange!();
    assert_eq!(host.response(end).status(), 0);
    let records: Vec<&[u8]> = RawRecords::new(&stream).map(|r| r.unwrap().1).collect();

    // A restore committed with the network up (rule 1), started again.
    forget_sent(&mut port);
    let begin = host.request(&BkpRestoreBegin {
        flags: 0x02,
        nwk_margin: 10_000,
        aps_margin: 10_000,
        expected_records: records.len() as u32,
    });
    exchange!();
    let begin: BkpRestoreBeginRsp = host.response(begin).body();
    let mut payload = begin.txn.to_le_bytes().to_vec();
    for record in records.iter().take(usize::from(begin.max_records_per_put)) {
        payload.extend_from_slice(record);
    }
    assert!(usize::from(begin.max_records_per_put) >= records.len());
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    let commit = host.request(&BkpRestoreCommit { txn: begin.txn });
    exchange!();
    assert_eq!(host.response(put).status(), 0);
    assert_eq!(host.response(commit).status(), 0);
    assert_eq!(port.state() as u8, 2, "restored and up");
    assert_eq!(leaves_sent(&mut port), 0, "rule 1 wipe");

    // NWK_LEAVE_RESET as hosts send it before a restore (flags 0x03), and
    // without flags.
    for flags in [
        leave_reset_flags::KEEP_IEEE | leave_reset_flags::KEEP_ENDPOINTS,
        0,
    ] {
        if port.state() as u8 != 2 {
            let form = host.request(&form_request(0x5EA7, 1 << 15));
            exchange!();
            assert_eq!(host.response(form).status(), 0);
        }
        forget_sent(&mut port);
        let wipe = host.request(&NwkLeaveReset { flags });
        exchange!();
        assert_eq!(host.response(wipe).status(), 0);
        assert_eq!(port.state() as u8, 0);
        let mac = mock_mac(&mut port);
        assert!(
            mac.tx_history_before_reset().is_empty() && mac.tx_history().is_empty(),
            "flags {flags:#04x}: nothing on air"
        );
    }

    // An explicit leave announces it once, to every rx-on device.
    let form = host.request(&form_request(0x5EA7, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    forget_sent(&mut port);
    let leave = host.request(&NwkLeaveReset {
        flags: leave_reset_flags::ANNOUNCE_LEAVE,
    });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);
    assert_eq!(leaves_sent(&mut port), 1);
    let frames = mock_mac(&mut port).tx_history_before_reset();
    assert_eq!(frames.len(), 1, "only the Leave");
    let (header, _) =
        zigbee_nwk::frames::NwkHeader::parse(frames[0].payload.as_slice()).expect("a NWK frame");
    assert_eq!(header.dst_addr, zigbee_types::ShortAddress(0xFFFD));
    assert_eq!(port.state() as u8, 0);
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

    // The platform turns PTA off on its own (fail closed after a fault):
    // reported off, re-enabling fails, a new mask is passed on as disabled.
    platform.pta_failed = true;
    let status = host.request_raw(ids::SYS_GET_STATUS, &[]);
    let get = host.request(&SysGetConfig {
        ids: List::new(&[config::COEX_ENABLE]),
    });
    let set = host.request(&SysSetConfig {
        values: List::new(&[
            ConfigValue {
                id: config::COEX_TX_PRIORITY_MASK,
                value: Bytes8(&[0x02]),
            },
            ConfigValue {
                id: config::COEX_ENABLE,
                value: Bytes8(&[1]),
            },
        ]),
    });
    exchange!();
    let status = host.response(status);
    assert_eq!(status.body::<SysGetStatusRsp>().flags & 0x04, 0);
    let get = host.response(get);
    let get: SysGetConfigRsp = get.body();
    let values: Vec<_> = get.items.iter().map(|i| i.value.0.to_vec()).collect();
    assert_eq!(values, [vec![0]]);
    let set = host.response(set);
    let set: SysSetConfigRsp = set.body();
    let results: Vec<_> = set.results.iter().map(|r| r.status).collect();
    assert_eq!(results, [Status::Success, Status::Failure]);
    assert_eq!(platform.pta, Some((false, 0x02)));

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

/// An EmberZNet backup (zigpy `stack_specific.ezsp.hashed_tclk`) lists its
/// devices without link keys: the TCLK of each is derived from the root.
const EZSP_ROOT: [u8; 16] = [
    0x0F, 0x1E, 0x2D, 0x3C, 0x4B, 0x5A, 0x69, 0x78, 0x87, 0x96, 0xA5, 0xB4, 0xC3, 0xD2, 0xE1, 0xF0,
];

/// TLV bytes of `record`.
fn tlv(record: &Record<'_>) -> Vec<u8> {
    let mut buffer = [0u8; 64];
    let mut w = Writer::new(&mut buffer);
    record.write(&mut w).unwrap();
    let len = w.len();
    buffer[..len].to_vec()
}

/// TLV of a device record without a link block (an EZSP backup's device).
fn keyless(ieee: [u8; 8], nwk: u16, child: bool) -> Vec<u8> {
    tlv(&Record::Device(DeviceRecord {
        ieee: Ieee(ieee),
        nwk,
        flags: 0,
        child: child.then_some(zigbee_zrhp::records::ChildInfo {
            capability: 0x80,
            timeout_exp: 8,
        }),
        link: None,
    }))
}

#[test]
fn restoring_an_ezsp_backup_derives_the_hashed_link_keys() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x9BBF, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let begin = host.request(&BkpExportBegin { flags: 0x03 });
    exchange!();
    let begin: BkpExportBeginRsp = host.response(begin).body();
    // NODE, NETWORK, NWK key, TCLK, RS_PARAMS, END: no root yet.
    assert_eq!(begin.record_count, 6);
    let read = host.request(&BkpExportRead {
        txn: begin.txn,
        max_bytes: 480,
    });
    exchange!();
    let stream = host.response(read).payload[2..].to_vec();
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);

    // A sleepy child and a router, both without a key in the backup.
    const ROUTER_IEEE: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    let ezsp = with_records(
        &stream,
        &[
            tlv(&Record::EzspHashedTclk(EZSP_ROOT)),
            keyless(DEVICE_IEEE, 0x12D2, true),
            keyless(ROUTER_IEEE, 0x5D01, false),
        ],
    );
    let restore = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 0,
        aps_margin: 0,
        expected_records: 9,
    });
    exchange!();
    let txn = host.response(restore).body::<BkpRestoreBeginRsp>().txn;
    let mut payload = txn.to_le_bytes().to_vec();
    payload.extend_from_slice(&ezsp);
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    let commit = host.request(&BkpRestoreCommit { txn });
    exchange!();
    let put = host.response(put);
    let put: BkpRestorePutRsp = put.body();
    assert_eq!(put.notes.iter().count(), 0, "the root is kept");
    let commit: BkpRestoreCommitRsp = host.response(commit).body();
    assert_eq!((commit.children, commit.link_keys_on_ncp), (1, 2));

    // Both devices are known with the key their EmberZNet Trust Center
    // derived for them; the child keeps its parent and address.
    for (ieee, nwk, child) in [(DEVICE_IEEE, 0x12D2, true), (ROUTER_IEEE, 0x5D01, false)] {
        let view = (0..)
            .map_while(|index| port.device(index))
            .find(|view| view.ieee == ieee)
            .expect("restored device");
        assert_eq!(view.child.is_some(), child);
        if child {
            assert_eq!(view.nwk, Some(nwk));
        }
        let link = view.link.expect("derived key");
        assert_eq!(
            link.key,
            zigbee_aps::security::derive_ezsp_hashed_link_key(&EZSP_ROOT, &ieee)
        );
        assert_eq!((link.origin, link.attrs), (4, 2));
    }

    // Any device's key on request (export-gated like the other keys).
    let derive = host.request(&SecDeriveEzspKey {
        ieee: Ieee(OTHER_IEEE),
    });
    exchange!();
    let derived: SecDeriveEzspKeyRsp = host.response(derive).body();
    assert_eq!(
        derived.key,
        zigbee_aps::security::derive_ezsp_hashed_link_key(&EZSP_ROOT, &OTHER_IEEE)
    );

    // The next backup carries the root again, and the TCLK flags it.
    let begin = host.request(&BkpExportBegin { flags: 0x03 });
    exchange!();
    let begin: BkpExportBeginRsp = host.response(begin).body();
    assert_eq!(begin.record_count, 9);
    let mut records = Vec::new();
    loop {
        let read = host.request(&BkpExportRead {
            txn: begin.txn,
            max_bytes: 480,
        });
        exchange!();
        let read = host.response(read);
        records.extend_from_slice(&read.payload[2..]);
        if read.payload[1] == 0 {
            break;
        }
    }
    let mut root = None;
    let mut tclk_flags = None;
    let mut count = 0;
    for item in RawRecords::new(&records) {
        let (kind, tlv) = item.unwrap();
        count += 1;
        match zigbee_zrhp::codec::Reader::new(tlv)
            .read::<Record<'_>>()
            .unwrap()
        {
            Record::EzspHashedTclk(seed) => root = Some(seed),
            Record::TcLinkKey(tclk) => tclk_flags = Some(tclk.flags),
            Record::End(end) => assert_eq!(end.record_count, 8),
            _ => assert_ne!(kind, record_type::EZSP_HASHED_TCLK),
        }
    }
    assert_eq!(count, 9);
    assert_eq!(root, Some(EZSP_ROOT));
    assert_eq!(tclk_flags, Some(0x01));
}

#[test]
fn a_keyless_restored_child_is_known_without_a_root() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x9BC0, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let begin = host.request(&BkpExportBegin { flags: 0x03 });
    exchange!();
    let begin: BkpExportBeginRsp = host.response(begin).body();
    let read = host.request(&BkpExportRead {
        txn: begin.txn,
        max_bytes: 480,
    });
    exchange!();
    let stream = host.response(read).payload[2..].to_vec();
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);

    let backup = with_records(&stream, &[keyless(DEVICE_IEEE, 0x12D2, true)]);
    let restore = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 0,
        aps_margin: 0,
        expected_records: 7,
    });
    exchange!();
    let txn = host.response(restore).body::<BkpRestoreBeginRsp>().txn;
    let mut payload = txn.to_le_bytes().to_vec();
    payload.extend_from_slice(&backup);
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    let commit = host.request(&BkpRestoreCommit { txn });
    exchange!();
    assert_eq!(host.response(put).status(), 0);
    let commit: BkpRestoreCommitRsp = host.response(commit).body();
    assert_eq!((commit.children, commit.link_keys_on_ncp), (1, 0));

    // Known to the Trust Center (a secured rejoin is admitted), still a
    // child, and exported without a link block (its key is unknown).
    let view = port.device(0).expect("restored child");
    assert_eq!(
        (view.ieee, view.nwk, view.child),
        (DEVICE_IEEE, Some(0x12D2), Some((0x80, 8)))
    );
    assert_eq!(view.link, None);
    assert!(port.device(1).is_none(), "listed once");
    assert!(
        port.app()
            .tc_table()
            .device(&DEVICE_IEEE)
            .is_some_and(|device| !device.link_key_known())
    );
    let derive = host.request(&SecDeriveEzspKey {
        ieee: Ieee(DEVICE_IEEE),
    });
    exchange!();
    assert_eq!(host.response(derive).status(), Status::NotFound.to_u8());
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

/// TLV bytes of an `APS_GROUP` record.
fn group_tlv(group: u16, endpoints: &[u8]) -> Vec<u8> {
    let mut buf = vec![0u8; 3 + 3 + endpoints.len()];
    let mut w = Writer::new(&mut buf);
    Record::ApsGroup(ApsGroupRecord { group, endpoints })
        .write(&mut w)
        .unwrap();
    buf
}

/// `stream` with `extra` TLVs before `RS_PARAMS` and `END` recomputed.
fn with_records(stream: &[u8], extra: &[Vec<u8>]) -> Vec<u8> {
    let mut out = Vec::new();
    for item in RawRecords::new(stream) {
        let (kind, tlv) = item.unwrap();
        if kind == record_type::RS_PARAMS {
            for record in extra {
                out.extend_from_slice(record);
            }
        }
        out.extend_from_slice(tlv);
    }
    resign(&out)
}

/// `(group, endpoints)` of every `APS_GROUP` in a record stream.
fn stream_groups(stream: &[u8]) -> Vec<(u16, Vec<u8>)> {
    RawRecords::new(stream)
        .filter_map(|item| {
            let (_, tlv) = item.unwrap();
            match zigbee_zrhp::codec::Reader::new(tlv)
                .read::<Record<'_>>()
                .unwrap()
            {
                Record::ApsGroup(group) => Some((group.group, group.endpoints.to_vec())),
                _ => None,
            }
        })
        .collect()
}

/// Sorted groups with sorted endpoints (the table reorders on removal).
fn canonical(mut groups: Vec<(u16, Vec<u8>)>) -> Vec<(u16, Vec<u8>)> {
    for (_, endpoints) in &mut groups {
        endpoints.sort_unstable();
    }
    groups.sort_unstable();
    groups
}

/// `TBL_READ` of the APS group table (`0x0D`): empty, one group, a full
/// table over several pages, cursor and `max` bounds, read-only, and wiped
/// with the network.
#[test]
fn group_table_reads_page_by_page() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    macro_rules! read {
        ($cursor:expr, $max:expr) => {{
            let request = host.request(&TblRead {
                table: record_type::APS_GROUP,
                cursor: $cursor,
                max: $max,
            });
            exchange!();
            host.response(request)
        }};
    }
    macro_rules! page {
        ($cursor:expr, $max:expr) => {{
            let response = read!($cursor, $max);
            let read: TblReadRsp = response.body();
            let entries: Vec<(u32, u8, u16, Vec<u8>)> = read
                .entries
                .iter()
                .map(|entry| match entry.record {
                    Record::ApsGroup(group) => (
                        entry.rev,
                        entry.state,
                        group.group,
                        group.endpoints.to_vec(),
                    ),
                    other => panic!("not an APS_GROUP: {other:?}"),
                })
                .collect();
            (read.next_cursor, entries)
        }};
    }
    macro_rules! stats {
        () => {{
            let request = host.request(&TblStats {
                table: record_type::APS_GROUP,
            });
            exchange!();
            let response = host.response(request);
            let stats: TblStatsRsp = response.body();
            (stats.capacity, stats.used, stats.dirty, stats.pinned)
        }};
    }
    macro_rules! member {
        ($op:expr, $group:expr, $ep:expr) => {{
            let request = host.request(&NwkGroupMembership {
                op: $op,
                group: $group,
                ep: $ep,
            });
            exchange!();
            assert_eq!(host.response(request).status(), 0);
        }};
    }

    // Advertised: capability bit 24 and capacity `group_table`.
    let negotiate = host.request(&SysNegotiateReq {
        major_min: 1,
        major_max: 1,
        host_caps: 0,
        name: Bytes8(b"surface"),
    });
    exchange!();
    let negotiated = host.response(negotiate);
    let negotiated: SysNegotiateRsp = negotiated.body();
    assert_ne!(negotiated.ncp_caps & caps::APS_GROUP_TABLE, 0);
    let group_table = negotiated
        .capacities
        .iter()
        .find(|c| c.id == capacity::GROUP_TABLE)
        .map(|c| c.value);
    assert_eq!(group_table, Some(16));

    // The table of the running network only, like DEVICE.
    assert_eq!(read!(0, 8).status(), Status::InvalidState.to_u8());
    assert_eq!(stats!(), (16, 0, 0, 0));
    let form = host.request(&form_request(0x1A62, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let epoch = port.table_epoch();
    assert_eq!(page!(0, 8), (0xFFFF, vec![]), "empty");

    member!(1, 0x0B84, 242);
    assert_eq!(page!(0, 8), (0xFFFF, vec![(0, 0, 0x0B84, vec![242])]));
    assert_eq!(stats!(), (16, 1, 0, 0));

    // A full table: 0x0B84, 0x1234 with the most endpoints (8), 14 more.
    for ep in 1..=8 {
        member!(1, 0x1234, ep);
    }
    for group in 1..=14u16 {
        member!(1, group, 1);
    }
    assert_eq!(stats!(), (16, 16, 0, 0));
    let table: Vec<(u16, Vec<u8>)> = (0..16)
        .map(|index| {
            let view = port.group(index).unwrap();
            (view.group, view.endpoints.to_vec())
        })
        .collect();
    assert_eq!(table[1], (0x1234, (1..=8).collect::<Vec<u8>>()));
    let (next, first) = page!(0, 8);
    assert_eq!((next, first.len()), (8, 8));
    let (next, second) = page!(8, 8);
    assert_eq!((next, second.len()), (0xFFFF, 8));
    let read: Vec<(u16, Vec<u8>)> = first
        .into_iter()
        .chain(second)
        .map(|(rev, state, group, endpoints)| {
            assert_eq!(
                (rev, state),
                (0, 0),
                "standalone: rev 0, neither dirty nor pinned"
            );
            (group, endpoints)
        })
        .collect();
    assert_eq!(read, table, "table order, every endpoint");
    // Smaller pages walk the same table.
    let mut cursor = 0u16;
    let mut walked = Vec::new();
    while cursor != 0xFFFF {
        let (next, entries) = page!(cursor, 3);
        assert!(entries.len() <= 3);
        walked.extend(entries.into_iter().map(|(_, _, g, e)| (g, e)));
        cursor = next;
    }
    assert_eq!(walked, table);
    // `max` 0 reads one entry, more than 8 reads 8; past the end is empty.
    assert_eq!(page!(0, 0).0, 1);
    assert_eq!(page!(0, 0).1.len(), 1);
    assert_eq!(page!(0, 255).1.len(), 8);
    assert_eq!(
        page!(15, 8),
        (0xFFFF, vec![(0, 0, table[15].0, table[15].1.clone())])
    );
    assert_eq!(page!(16, 8), (0xFFFF, vec![]));
    assert_eq!(page!(0xFFFF, 8), (0xFFFF, vec![]));

    // Read-only: written with NWK_GROUP_MEMBERSHIP.
    let push = host.request(&TblPush {
        table: record_type::APS_GROUP,
        flags: 0,
        records: List::new(&[]),
    });
    let invalidate = host.request(&TblInvalidate {
        table: record_type::APS_GROUP,
        key: TableKey::Nwk(0x0B84),
    });
    exchange!();
    assert_eq!(
        host.response(push).status(),
        Status::UnsupportedCommand.to_u8()
    );
    assert_eq!(
        host.response(invalidate).status(),
        Status::UnsupportedCommand.to_u8()
    );
    // Membership edits do not change the table epoch.
    member!(3, 0, 1);
    assert_eq!(stats!(), (16, 2, 0, 0));
    assert_eq!(port.table_epoch(), epoch);

    // A network leave wipes the groups with the other tables (new epoch).
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);
    assert!(host.event(ids::EVT_TBL_EPOCH_CHANGED).is_some());
    assert_eq!(read!(0, 8).status(), Status::InvalidState.to_u8());
    let form = host.request(&form_request(0x1A63, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    assert_eq!(page!(0, 8), (0xFFFF, vec![]), "a new network has no groups");
}

/// `BKP_EXPORT_BEGIN` flags bit3 exports the group table as `APS_GROUP`
/// records; a restore puts it back on the restored network.
#[test]
fn backup_exports_and_restores_the_group_table() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x7C01, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let groups = [
        (0x0B84u16, vec![242u8]),
        (0x1234, vec![1, 3]),
        (0x0001, vec![2]),
    ];
    for (group, endpoints) in &groups {
        for &ep in endpoints {
            let request = host.request(&NwkGroupMembership {
                op: 1,
                group: *group,
                ep,
            });
            exchange!();
            assert_eq!(host.response(request).status(), 0);
        }
    }
    macro_rules! export {
        ($flags:expr) => {{
            let begin = host.request(&BkpExportBegin { flags: $flags });
            exchange!();
            let begin = host.response(begin);
            let begin: BkpExportBeginRsp = begin.body();
            let mut stream = Vec::new();
            loop {
                let read = host.request(&BkpExportRead {
                    txn: begin.txn,
                    max_bytes: 64,
                });
                exchange!();
                let read = host.response(read);
                assert_eq!(read.status(), 0);
                stream.extend_from_slice(&read.payload[2..]);
                if read.payload[1] == 0 {
                    break;
                }
            }
            let end = host.request(&BkpExportEnd { txn: begin.txn });
            exchange!();
            assert_eq!(host.response(end).status(), 0);
            assert_eq!(
                zigbee_zrhp::records::verify_stream(&stream).unwrap(),
                begin.record_count - 1,
                "announced count"
            );
            stream
        }};
    }
    let without = export!(export_flags::KEYS | export_flags::DEVICES);
    assert!(stream_groups(&without).is_empty(), "groups only on request");
    let stream = export!(export_flags::KEYS | export_flags::DEVICES | export_flags::APS_GROUPS);
    assert_eq!(
        RawRecords::new(&stream).count(),
        RawRecords::new(&without).count() + 3
    );
    assert_eq!(
        canonical(stream_groups(&stream)),
        canonical(groups.to_vec())
    );

    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange!();
    assert_eq!(host.response(leave).status(), 0);
    let begin = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 10_000,
        aps_margin: 10_000,
        expected_records: RawRecords::new(&stream).count() as u32,
    });
    exchange!();
    let begin = host.response(begin);
    let begin: BkpRestoreBeginRsp = begin.body();
    let mut payload = begin.txn.to_le_bytes().to_vec();
    payload.extend_from_slice(&stream);
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    let commit = host.request(&BkpRestoreCommit { txn: begin.txn });
    exchange!();
    let put = host.response(put);
    let put: BkpRestorePutRsp = put.body();
    assert_eq!(
        put.notes.iter().count(),
        0,
        "a zigbee-rs backup restores as is"
    );
    let commit = host.response(commit);
    let _: BkpRestoreCommitRsp = commit.body();
    assert_eq!(port.state() as u8, 2);
    let restored: Vec<(u16, Vec<u8>)> = (0..)
        .map_while(|index| port.group(index))
        .map(|view| (view.group, view.endpoints.to_vec()))
        .collect();
    assert_eq!(canonical(restored), canonical(groups.to_vec()));
    let tables = TcApp::device(port.app()).aps().group_table();
    assert!(tables.is_member(0x0B84, 242) && tables.is_member(0x1234, 3));
    // The restored network exports them again.
    let again = export!(export_flags::APS_GROUPS);
    assert_eq!(canonical(stream_groups(&again)), canonical(groups.to_vec()));
}

/// A restore never overflows the group table: groups beyond its capacity
/// and groups with more endpoints than it holds are `IGNORED` (nothing of
/// the record applied), records of one group merge; malformed records
/// abort the transaction (9.4 rule 2).
#[test]
fn restore_bounds_the_group_table() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x7C02, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let begin = host.request(&BkpExportBegin {
        flags: export_flags::KEYS,
    });
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

    /// Restore `stream` in PUTs of at most `max_records_per_put` records
    /// and 512 bytes; the notes and the statuses of the last PUT and the
    /// COMMIT.
    macro_rules! restore {
        ($stream:expr) => {{
            let stream: Vec<u8> = $stream;
            let records: Vec<&[u8]> = RawRecords::new(&stream).map(|r| r.unwrap().1).collect();
            let begin = host.request(&BkpRestoreBegin {
                flags: 0,
                nwk_margin: 0,
                aps_margin: 0,
                expected_records: records.len() as u32,
            });
            exchange!();
            let begin = host.response(begin);
            let begin: BkpRestoreBeginRsp = begin.body();
            let mut notes = Vec::new();
            let mut put_status = 0;
            let mut chunk: Vec<&[u8]> = Vec::new();
            let mut chunks = Vec::new();
            for record in records {
                let bytes: usize = chunk.iter().map(|r| r.len()).sum();
                if chunk.len() == usize::from(begin.max_records_per_put)
                    || bytes + record.len() > 512
                {
                    chunks.push(std::mem::take(&mut chunk));
                }
                chunk.push(record);
            }
            chunks.push(chunk);
            for chunk in chunks {
                let mut payload = begin.txn.to_le_bytes().to_vec();
                for record in chunk {
                    payload.extend_from_slice(record);
                }
                let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
                exchange!();
                let put = host.response(put);
                put_status = put.status();
                if put_status != 0 {
                    break;
                }
                let put: BkpRestorePutRsp = put.body();
                notes.extend(put.notes.iter().map(|n| (n.index, n.note)));
            }
            let commit = host.request(&BkpRestoreCommit { txn: begin.txn });
            exchange!();
            (notes, put_status, host.response(commit).status())
        }};
    }

    // Index of the first inserted record: NODE NETWORK NETWORK_KEY
    // TC_LINK_KEY come first.
    let first = 4u16;
    let codec_max: Vec<u8> = (0..255).map(|i| (i % 240) as u8 + 1).collect();
    let mut extra = vec![
        group_tlv(0x2000, &[1, 2]),                      // 0: staged
        group_tlv(0x2000, &[3]),                         // 1: merged into 0x2000
        group_tlv(0x2001, &[1, 2, 3, 4, 5, 6, 7, 8, 9]), // 2: 9 endpoints
        group_tlv(0x2000, &[4, 5, 6, 7, 8, 9]),          // 3: 0x2000 would hold 9
        group_tlv(0x2002, &[5, 5, 5]),                   // 4: staged as [5]
        group_tlv(0x2003, &codec_max),                   // 5: 255 endpoints (codec limit)
    ];
    for group in 0x3000..0x300E {
        extra.push(group_tlv(group, &[1])); // 6..=19: 16 groups staged
    }
    extra.push(group_tlv(0x4000, &[1])); // 20: the table is full
    extra.push(group_tlv(0x2002, &[6])); // 21: an existing group still merges
    let (notes, put, commit) = restore!(with_records(&stream, &extra));
    assert_eq!((put, commit), (0, 0));
    assert_eq!(
        notes,
        [2u16, 3, 5, 20]
            .iter()
            .map(|i| (first + i, note::IGNORED))
            .collect::<Vec<_>>()
    );
    let restored: Vec<(u16, Vec<u8>)> = (0..)
        .map_while(|index| port.group(index))
        .map(|view| (view.group, view.endpoints.to_vec()))
        .collect();
    let mut expected = vec![(0x2000u16, vec![1u8, 2, 3]), (0x2002, vec![5, 6])];
    expected.extend((0x3000..0x300E).map(|group| (group, vec![1])));
    assert_eq!(restored.len(), 16, "the table is exactly full");
    assert_eq!(canonical(restored), canonical(expected));

    // Malformed: the PUT fails and the transaction is gone.
    for bad in [
        group_tlv(0xFFF8, &[1]),
        group_tlv(0x2000, &[]),
        group_tlv(0x2000, &[0]),
        group_tlv(0x2000, &[1, 241]),
        group_tlv(0x2000, &[243]),
    ] {
        let (_, put, commit) = restore!(with_records(&stream, &[bad]));
        assert_eq!(put, Status::InvalidParameter.to_u8());
        assert_eq!(commit, Status::NotFound.to_u8());
    }
    // The last committed restore is untouched.
    assert_eq!(port.group_capacity(), 16);
    assert_eq!((0..).map_while(|index| port.group(index)).count(), 16);
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
    // One EVT_SCAN_PROGRESS per measurement (3 channels × 2 passes), all
    // before the result.
    let progress: Vec<(usize, EvtScanProgress)> = host
        .inbox
        .iter()
        .enumerate()
        .filter(|(_, m)| m.kind == Kind::Event && m.msg_id == ids::EVT_SCAN_PROGRESS)
        .map(|(i, m)| (i, m.event()))
        .collect();
    assert_eq!(progress.len(), 6);
    assert!(
        progress
            .iter()
            .all(|(i, p)| *i < event && p.req_tid == scan && p.scan == 0)
    );
    assert_eq!(
        progress
            .iter()
            .map(|(_, p)| (p.channel, p.channels_done, p.channels_total))
            .collect::<Vec<_>>(),
        [
            (15, 1, 6),
            (20, 2, 6),
            (25, 3, 6),
            (15, 4, 6),
            (20, 5, 6),
            (25, 6, 6)
        ]
    );
    while host.event(ids::EVT_SCAN_PROGRESS).is_some() {}
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
            0x0201, 0x0202, 0x0203, 0x0204, 0x0205, 0x0206, 0x0207, 0x0208, 0x0301, 0x0302, 0x0303,
            0x0304, 0x0305, 0x0306, 0x0401, 0x0402, 0x0403, 0x0501, 0x0502, 0x0503, 0x0504, 0x0505
        ]
    );
    assert_eq!(values[0], (0x0201, 9));
    let reset = host.request(&DiagResetCounters { group_mask: 0x02 });
    exchange!();
    assert_eq!(host.response(reset).status(), 0);
    assert_eq!(platform.mac_tx, 0);
    // Every counter of every group fits one response: the last group's
    // last counter (GP 0x0813) is present.
    let counters = host.request(&DiagGetCounters { group_mask: 0xFF });
    exchange!();
    let all = counter_values(host.response(counters));
    assert!(all.len() > 64, "{}", all.len());
    assert!(all.len() < ncp_coordinator_app::ncp::MAX_COUNTERS);
    assert_eq!(all.last().map(|(id, _)| *id), Some(0x0813));
    let mut ids: Vec<u16> = all.iter().map(|(id, _)| *id).collect();
    ids.dedup();
    assert_eq!(ids.len(), all.len());
}

fn neighbour_beacon(channel: u8, src: u16) -> zigbee_mac::primitives::PanDescriptor {
    use zigbee_mac::primitives::{PanDescriptor, SuperframeSpec, ZigbeeBeaconPayload};
    PanDescriptor {
        channel,
        coord_address: zigbee_types::MacAddress::Short(
            zigbee_types::PanId(0x4D2A),
            zigbee_types::ShortAddress(src),
        ),
        superframe_spec: SuperframeSpec {
            association_permit: true,
            ..Default::default()
        },
        lqi: 160,
        security_use: false,
        zigbee_beacon: ZigbeeBeaconPayload {
            protocol_id: 0,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity: true,
            device_depth: 1,
            end_device_capacity: true,
            extended_pan_id: [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
            tx_offset: [0xFF; 3],
            update_id: 7,
        },
    }
}

#[test]
fn active_scan_answers_with_every_beacon_and_keeps_the_network() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    {
        let mac = TcApp::device_mut(port.app_mut())
            .aps_mut()
            .nwk_mut()
            .mac_mut();
        mac.add_beacon(neighbour_beacon(25, 0x0000));
        mac.add_beacon(neighbour_beacon(25, 0x1234));
        // Not reported: a Thread beacon and one without a Zigbee payload.
        let mut thread = neighbour_beacon(20, 0x0042);
        thread.zigbee_beacon.protocol_id = 3;
        mac.add_beacon(thread);
        let mut bare = neighbour_beacon(20, 0x0043);
        bare.zigbee_beacon.extended_pan_id = [0; 8];
        bare.zigbee_beacon.stack_profile = 0;
        mac.add_beacon(bare);
    }
    let channels = (1 << 15) | (1 << 20) | (1 << 25);
    let bad = host.request(&NwkActiveScan {
        channel_mask: channels,
        duration_exp: 7,
    });
    let scan = host.request(&NwkActiveScan {
        channel_mask: channels,
        duration_exp: 3,
    });
    let busy = host.request(&NwkEnergyScan {
        channel_mask: channels,
        duration_exp: 2,
        count: 1,
    });
    exchange!();
    exchange!();
    assert_eq!(
        host.response(bad).status(),
        Status::InvalidParameter.to_u8()
    );
    assert_eq!(host.response(busy).status(), Status::InProgress.to_u8());
    let answered = host
        .inbox
        .iter()
        .position(|m| m.kind == Kind::Response && m.tid == scan)
        .unwrap();
    let progress: Vec<(usize, EvtScanProgress)> = host
        .inbox
        .iter()
        .enumerate()
        .filter(|(_, m)| m.kind == Kind::Event && m.msg_id == ids::EVT_SCAN_PROGRESS)
        .map(|(i, m)| (i, m.event()))
        .collect();
    assert!(
        progress
            .iter()
            .all(|(i, p)| *i < answered && p.scan == 1 && p.req_tid == scan)
    );
    assert_eq!(
        progress
            .iter()
            .map(|(_, p)| (
                p.channel,
                p.channels_done,
                p.channels_total,
                p.networks_found
            ))
            .collect::<Vec<_>>(),
        [(15, 1, 3, 0), (20, 2, 3, 0), (25, 3, 3, 2)]
    );
    while host.event(ids::EVT_SCAN_PROGRESS).is_some() {}
    let response = host.response(scan);
    let beacons: NwkActiveScanRsp = response.body();
    let found: Vec<_> = beacons
        .results
        .iter()
        .map(|b| {
            (
                b.channel,
                b.pan,
                b.epid,
                b.src,
                b.lqi,
                b.rssi,
                b.permit,
                b.stack_profile,
                b.update_id,
            )
        })
        .collect();
    let epid = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    assert_eq!(
        found,
        [
            (25, 0x4D2A, epid, 0x0000, 160, -60, 1, 2, 7),
            (25, 0x4D2A, epid, 0x1234, 160, -60, 1, 2, 7),
        ]
    );

    // With the network up: the radio is back on the operating channel.
    let form = host.request(&form_request(0x2B35, 1 << 20));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let scan = host.request(&NwkActiveScan {
        channel_mask: channels,
        duration_exp: 3,
    });
    exchange!();
    exchange!();
    let response = host.response(scan);
    let beacons: NwkActiveScanRsp = response.body();
    assert_eq!(beacons.results.len(), 2);
    assert_eq!(port.info().channel, 20);
    assert_eq!(
        port.info().state,
        ncp_coordinator_app::port::NetworkState::Up
    );
}

// ── Restored and reloaded children age out ──────────────────────────

/// A second keyless child of the restored backups below.
const OTHER_CHILD: [u8; 8] = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18];

/// TLV of a keyless sleepy end-device child of the NCP (`DEVICE` with a
/// child block, capability `0x80`).
fn child_tlv(ieee: [u8; 8], nwk: u16, timeout_exp: u8) -> Vec<u8> {
    tlv(&Record::Device(DeviceRecord {
        ieee: Ieee(ieee),
        nwk,
        flags: 0,
        child: Some(zigbee_zrhp::records::ChildInfo {
            capability: 0x80,
            timeout_exp,
        }),
        link: None,
    }))
}

/// The complete `BKP_EXPORT` stream (keys and devices).
fn export_stream<A: TcApp>(
    ncp: &mut Ncp<8>,
    host: &mut Host,
    port: &mut TcPort<A>,
    platform: &mut FakePlatform,
    pipe: &mut Pipe,
    log: &LogRing<8>,
    now: &mut u64,
) -> Vec<u8> {
    let begin = host.request(&BkpExportBegin { flags: 0x03 });
    exchange(ncp, host, port, platform, pipe, log, now);
    let begin: BkpExportBeginRsp = host.response(begin).body();
    let mut stream = Vec::new();
    loop {
        let read = host.request(&BkpExportRead {
            txn: begin.txn,
            max_bytes: 480,
        });
        exchange(ncp, host, port, platform, pipe, log, now);
        let read = host.response(read);
        stream.extend_from_slice(&read.payload[2..]);
        if read.payload[1] == 0 {
            break;
        }
    }
    let end = host.request(&BkpExportEnd { txn: begin.txn });
    exchange(ncp, host, port, platform, pipe, log, now);
    assert_eq!(host.response(end).status(), 0);
    stream
}

/// Wipe the running network and restore `stream` with `extra` records.
#[allow(clippy::too_many_arguments)]
fn restore_stream<A: TcApp>(
    ncp: &mut Ncp<8>,
    host: &mut Host,
    port: &mut TcPort<A>,
    platform: &mut FakePlatform,
    pipe: &mut Pipe,
    log: &LogRing<8>,
    now: &mut u64,
    stream: &[u8],
    extra: &[Vec<u8>],
) -> BkpRestoreCommitRsp {
    let leave = host.request(&NwkLeaveReset { flags: 0 });
    exchange(ncp, host, port, platform, pipe, log, now);
    assert_eq!(host.response(leave).status(), 0);
    let backup = with_records(stream, extra);
    let restore = host.request(&BkpRestoreBegin {
        flags: 0,
        nwk_margin: 0,
        aps_margin: 0,
        expected_records: 0,
    });
    exchange(ncp, host, port, platform, pipe, log, now);
    let txn = host.response(restore).body::<BkpRestoreBeginRsp>().txn;
    let mut payload = txn.to_le_bytes().to_vec();
    payload.extend_from_slice(&backup);
    let put = host.request_raw(ids::BKP_RESTORE_PUT, &payload);
    let commit = host.request(&BkpRestoreCommit { txn });
    exchange(ncp, host, port, platform, pipe, log, now);
    let put = host.response(put);
    let put: BkpRestorePutRsp = put.body();
    assert_eq!(put.notes.iter().count(), 0, "nothing demoted or ignored");
    host.response(commit).body()
}

/// The child block of `ieee`'s `DEVICE` record in an export stream:
/// `None` when it is not listed, `Some(None)` when it is listed as a
/// device that is not a child of the NCP.
fn exported_child(stream: &[u8], ieee: [u8; 8]) -> Option<Option<(u8, u8)>> {
    RawRecords::new(stream).find_map(|item| {
        let (_, tlv) = item.unwrap();
        match zigbee_zrhp::codec::Reader::new(tlv)
            .read::<Record<'_>>()
            .unwrap()
        {
            Record::Device(device) if device.ieee.0 == ieee => Some(
                device
                    .child
                    .map(|child| (child.capability, child.timeout_exp)),
            ),
            _ => None,
        }
    })
}

/// `(capability, timeout_exp)` of `ieee` as the NCP's device view (backup
/// export, `TBL_READ`) reports it; `None` when it is not a child.
fn child_view<A: TcApp>(port: &TcPort<A>, ieee: [u8; 8]) -> Option<(u8, u8)> {
    (0..)
        .map_while(|index| port.device(index))
        .find(|view| view.ieee == ieee)
        .and_then(|view| view.child)
}

/// Whether `ieee` is in the live neighbour table as an authenticated child.
fn in_child_table<A: TcApp>(port: &TcPort<A>, ieee: [u8; 8]) -> bool {
    port.app()
        .device()
        .aps()
        .nwk()
        .neighbor_table()
        .find_by_ieee(&ieee)
        .is_some_and(|entry| entry.relationship == zigbee_nwk::neighbor::Relationship::Child)
}

/// Move the mock radio clock `secs` forward without running the stack.
fn advance_clock<A: TcApp<Mac = MockMac>>(port: &mut TcPort<A>, secs: u32) {
    use zigbee_mac::PlatformServices as _;
    let mac = mock_mac(port);
    let now = mac.monotonic_micros();
    mac.set_time_micros(now.wrapping_add(secs * 1_000_000));
}

/// Run the stack (the platform loop's `TcPort::step`) for `secs` seconds of
/// radio time, a minute per step at most.
fn run_for<A: TcApp<Mac = MockMac>>(port: &mut TcPort<A>, secs: u32) {
    let mut left = secs;
    while left > 0 {
        let step = left.min(60);
        advance_clock(port, step);
        pollster::block_on(port.step()).unwrap();
        left -= step;
    }
}

/// Whether the NCP reported `ieee` as an aged-out child
/// (`EVT_DEVICE_LEFT` reason 4); drains the device events.
fn reported_aged_out<A: TcApp>(port: &mut TcPort<A>, ieee: [u8; 8]) -> bool {
    use ncp_coordinator_app::port::{DeviceEvent, left};
    let mut reported = false;
    while let Some(event) = port.take_device_event() {
        if let DeviceEvent::Left {
            ieee: departed,
            reason,
            ..
        } = event
            && departed == ieee
            && reason == left::REASON_CHILD_TIMEOUT
        {
            reported = true;
        }
    }
    reported
}

/// A MAC Data Request (keepalive poll) of the child at `nwk`.
fn child_polls<A: TcApp<Mac = MockMac>>(port: &mut TcPort<A>, nwk: u16) {
    use zigbee_mac::MacCommandEvent;
    use zigbee_mac::primitives::MlmeDataRequestIndication;
    use zigbee_types::{MacAddress, PanId, ShortAddress};
    let pan = PanId(port.info().pan_id);
    mock_mac(port).enqueue_command_event(MacCommandEvent::DataRequest(MlmeDataRequestIndication {
        source_address: MacAddress::Short(pan, ShortAddress(nwk)),
        destination_address: MacAddress::Short(pan, ShortAddress::COORDINATOR),
        lqi: 180,
        security_use: false,
        acked_without_frame_pending: true,
    }));
    for _ in 0..2 {
        pollster::block_on(port.step()).unwrap();
    }
}

/// R22 default End Device Timeout enumeration (256 min).
const DEFAULT_TIMEOUT: u8 = zigbee_nwk::frames::ED_TIMEOUT_ENUM_DEFAULT;
/// Its window in seconds.
const DEFAULT_WINDOW_SECS: u32 = 256 * 60;

/// A backup whose child block carries an undefined End Device Timeout
/// (`0xFF`: the host did not know it) restores that child with the R22
/// default (enumeration 8, 256 min), not the longest window (14, ≈ 11
/// days). Its clock starts at the restore, and a child that never polls
/// ages out like a joined one: reported as `EVT_DEVICE_LEFT` reason 4 and
/// exported afterwards as a known device, no longer a child.
#[test]
fn a_restored_child_with_an_undefined_timeout_ages_out_at_the_default() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x5EA7, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let stream = export_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
    );
    let commit = restore_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
        &stream,
        &[child_tlv(DEVICE_IEEE, 0x4C21, 0xFF)],
    );
    assert_eq!(commit.children, 1);
    assert_eq!(
        child_view(&port, DEVICE_IEEE),
        Some((0x80, DEFAULT_TIMEOUT)),
        "an undefined timeout is restored as the R22 default"
    );

    // The window runs from the restore: still a child a minute before it
    // ends, aged out once it has elapsed without a poll.
    run_for(&mut port, DEFAULT_WINDOW_SECS - 60);
    assert!(in_child_table(&port, DEVICE_IEEE), "within its window");
    assert!(!reported_aged_out(&mut port, DEVICE_IEEE));
    run_for(&mut port, 120);
    assert!(
        !in_child_table(&port, DEVICE_IEEE),
        "a restored child that never polls ages out"
    );
    assert!(reported_aged_out(&mut port, DEVICE_IEEE));
    assert_eq!(child_view(&port, DEVICE_IEEE), None);
    let stream = export_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
    );
    assert_eq!(
        exported_child(&stream, DEVICE_IEEE),
        Some(None),
        "still known to the Trust Center, no longer exported as a child"
    );
}

/// A backup vouches only for children that are still alive. A restored
/// child whose own End Device Timeout (2 min) has elapsed without a
/// keepalive is exported as a plain device even before the stack's aging
/// pass has run, while a child within its window keeps its child block;
/// the next aging pass then removes the expired one through the normal
/// path (`EVT_DEVICE_LEFT` reason 4).
#[test]
fn a_backup_never_exports_an_expired_child_before_the_aging_pass() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x5EA7, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let stream = export_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
    );
    let commit = restore_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
        &stream,
        &[
            child_tlv(DEVICE_IEEE, 0x4C21, 1),
            child_tlv(OTHER_CHILD, 0x4C22, 3),
        ],
    );
    assert_eq!(commit.children, 2);
    let stream = export_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
    );
    assert_eq!(exported_child(&stream, DEVICE_IEEE), Some(Some((0x80, 1))));
    assert_eq!(exported_child(&stream, OTHER_CHILD), Some(Some((0x80, 3))));

    // Three minutes of radio time, no poll and no stack step (no aging
    // pass yet).
    advance_clock(&mut port, 180);
    let stream = export_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
    );
    assert_eq!(
        exported_child(&stream, DEVICE_IEEE),
        Some(None),
        "an expired child is not vouched for, even before the aging pass"
    );
    assert_eq!(
        exported_child(&stream, OTHER_CHILD),
        Some(Some((0x80, 3))),
        "a child within its window still is"
    );
    let stats = host.request(&TblStats {
        table: record_type::DEVICE,
    });
    exchange!();
    let stats: TblStatsRsp = host.response(stats).body();
    assert_eq!(stats.pinned, 1, "only the live child is pinned");

    // The aging pass removes it through the usual path.
    assert!(in_child_table(&port, DEVICE_IEEE), "not swept yet");
    pollster::block_on(port.step()).unwrap();
    assert!(!in_child_table(&port, DEVICE_IEEE));
    assert!(reported_aged_out(&mut port, DEVICE_IEEE));
    assert!(in_child_table(&port, OTHER_CHILD));
}

/// The happy path is unchanged: a restored child that keeps polling stays a
/// child well past its window and is exported with its own timeout; once
/// it stops, it ages out after that window.
#[test]
fn a_restored_child_that_keeps_alive_stays_a_child() {
    setup!(ncp, host, port, platform, pipe, log, now, exchange);
    let form = host.request(&form_request(0x5EA7, 1 << 15));
    exchange!();
    assert_eq!(host.response(form).status(), 0);
    let stream = export_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
    );
    restore_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
        &stream,
        &[child_tlv(DEVICE_IEEE, 0x4C21, 1)],
    );
    // Ten minutes, a poll every minute (its window is 2 min).
    for _ in 0..10 {
        run_for(&mut port, 60);
        child_polls(&mut port, 0x4C21);
    }
    assert!(in_child_table(&port, DEVICE_IEEE));
    assert!(!reported_aged_out(&mut port, DEVICE_IEEE));
    let stream = export_stream(
        &mut ncp,
        &mut host,
        &mut port,
        &mut platform,
        &mut pipe,
        &log,
        &mut now,
    );
    assert_eq!(exported_child(&stream, DEVICE_IEEE), Some(Some((0x80, 1))));
    let children = host.request(&DiagGetChildren { start: 0 });
    exchange!();
    let children = host.response(children);
    let children: DiagGetChildrenRsp = children.body();
    let child = children.entries.iter().next().unwrap();
    assert!(child.last_poll_ms < 60_000, "heard since the restore");

    // It stops polling: gone once its own window has elapsed.
    run_for(&mut port, 60);
    assert!(in_child_table(&port, DEVICE_IEEE));
    run_for(&mut port, 120);
    assert!(!in_child_table(&port, DEVICE_IEEE));
    assert!(reported_aged_out(&mut port, DEVICE_IEEE));
}

/// A child table store shared across simulated reboots (the flash journal
/// outlives the application).
struct SharedChildren<'s>(&'s mut RamChildTableStore);

impl zigbee_runtime::child_store::ChildTableStore for SharedChildren<'_> {
    fn load(
        &mut self,
    ) -> Result<
        Option<zigbee_runtime::child_store::PersistentChildTable>,
        zigbee_runtime::child_store::ChildStoreError,
    > {
        self.0.load()
    }

    fn store(
        &mut self,
        table: &zigbee_runtime::child_store::PersistentChildTable,
    ) -> Result<(), zigbee_runtime::child_store::ChildStoreError> {
        self.0.store(table)
    }
}

/// A Trust Center store shared across simulated reboots.
struct SharedTrustCenter<'s>(&'s mut RamTrustCenterDeviceStore);

impl zigbee_runtime::trust_center_store::TrustCenterDeviceStore for SharedTrustCenter<'_> {
    fn load_into(
        &mut self,
        state: &mut zigbee_runtime::trust_center_store::PersistentTrustCenterState,
    ) -> Result<bool, zigbee_runtime::trust_center_store::TrustCenterStoreError> {
        self.0.load_into(state)
    }

    fn store(
        &mut self,
        state: &zigbee_runtime::trust_center_store::PersistentTrustCenterState,
    ) -> Result<(), zigbee_runtime::trust_center_store::TrustCenterStoreError> {
        self.0.store(state)
    }

    fn clear(&mut self) -> Result<(), zigbee_runtime::trust_center_store::TrustCenterStoreError> {
        self.0.clear()
    }
}

/// One NCP boot over durable `security`, `children` and `trust_center`
/// stores (see [`boot!`]).
macro_rules! reboot {
    ($security:ident, $children:ident, $trust_center:ident,
     |$ncp:ident, $host:ident, $port:ident, $platform:ident, $pipe:ident, $log:ident,
      $now:ident| $body:block) => {{
        let mut gp_store = zigbee_runtime::gp_store::RamGpProxyStore::new();
        let mut profile = profile();
        let mut device = device(&profile);
        let node = ZigbeeNode::new(&mut device, &mut $security, &mut profile);
        let mut app = TrustCenterCoordinatorApp::new(
            node,
            PersistentChildren::new(SharedChildren(&mut $children)),
            SharedTrustCenter(&mut $trust_center),
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
        let mut $ncp = Ncp::<8>::new(identity(), 0xC0FF_EE07, $now);
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

/// After a reboot the NCP reloads its children from the durable child
/// table. A child restored with an undefined timeout was persisted with
/// the default and ages out with it, a child with a known timeout ages out
/// with that one, and both clocks start at the reload. Once aged out they
/// are neither exported as children nor kept in the durable table.
#[test]
fn a_reloaded_child_ages_out_at_its_persisted_timeout() {
    use zigbee_runtime::child_store::ChildTableStore as _;
    let mut security = RamSecurityStateStore::new();
    let mut children = RamChildTableStore::new();
    let mut trust_center = RamTrustCenterDeviceStore::new();
    reboot!(
        security,
        children,
        trust_center,
        |ncp, host, port, platform, pipe, log, now| {
            let form = host.request(&form_request(0x5EA7, 1 << 15));
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
            let stream = export_stream(
                &mut ncp,
                &mut host,
                &mut port,
                &mut platform,
                &mut pipe,
                &log,
                &mut now,
            );
            let commit = restore_stream(
                &mut ncp,
                &mut host,
                &mut port,
                &mut platform,
                &mut pipe,
                &log,
                &mut now,
                &stream,
                &[
                    child_tlv(DEVICE_IEEE, 0x4C21, 0xFF),
                    child_tlv(OTHER_CHILD, 0x4C22, 3),
                ],
            );
            assert_eq!(commit.children, 2);
        }
    );
    let stored = children.load().unwrap().expect("a durable child table");
    let timeouts: Vec<([u8; 8], u8)> = stored
        .children()
        .map(|child| (child.ieee_address, child.end_device_timeout))
        .collect();
    assert_eq!(
        timeouts,
        [(DEVICE_IEEE, DEFAULT_TIMEOUT), (OTHER_CHILD, 3)],
        "persisted with the default, never the longest window"
    );

    reboot!(
        security,
        children,
        trust_center,
        |ncp, host, port, platform, pipe, log, now| {
            let init = host.request(&zigbee_zrhp::msg::network::NwkInit {
                nwk_fc_floor: 0,
                aps_fc_floor: 0,
            });
            exchange(
                &mut ncp,
                &mut host,
                &mut port,
                &mut platform,
                &mut pipe,
                &log,
                &mut now,
            );
            assert_eq!(host.response(init).status(), 0);
            assert_eq!(
                child_view(&port, DEVICE_IEEE),
                Some((0x80, DEFAULT_TIMEOUT))
            );
            assert_eq!(child_view(&port, OTHER_CHILD), Some((0x80, 3)));

            // The known 8-minute timeout.
            run_for(&mut port, 7 * 60);
            assert!(in_child_table(&port, OTHER_CHILD));
            run_for(&mut port, 2 * 60);
            assert!(!in_child_table(&port, OTHER_CHILD));
            assert!(reported_aged_out(&mut port, OTHER_CHILD));

            // The default 256-minute one, counted from the reload.
            run_for(&mut port, DEFAULT_WINDOW_SECS - 10 * 60);
            assert!(in_child_table(&port, DEVICE_IEEE), "within its window");
            run_for(&mut port, 2 * 60);
            assert!(
                !in_child_table(&port, DEVICE_IEEE),
                "a reloaded child that never polls ages out"
            );
            assert!(reported_aged_out(&mut port, DEVICE_IEEE));
            let stream = export_stream(
                &mut ncp,
                &mut host,
                &mut port,
                &mut platform,
                &mut pipe,
                &log,
                &mut now,
            );
            assert_eq!(exported_child(&stream, DEVICE_IEEE), Some(None));
            assert_eq!(exported_child(&stream, OTHER_CHILD), Some(None));
        }
    );
    assert!(
        children
            .load()
            .unwrap()
            .is_some_and(|table| table.is_empty()),
        "aged-out children leave the durable table"
    );
}
