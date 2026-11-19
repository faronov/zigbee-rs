//! GREEN POWER group (`0x0Axx`, endpoint ZIGBEE) of the NCP, spec 7.11.
//!
//! The coordinator runs the zigbee-rs GP combo: Proxy Basic, the sink
//! (16-entry Sink Table) and, with `green-power-bidirectional`, the
//! TempMaster. Exactly one sink authority exists, chosen at runtime with
//! `GP_SINK_CONFIG.mode`:
//!
//! * `NCP_SINK` (boot default): the NCP sink commissions, pairs, checks
//!   security and counters and removes duplicates; the host sees
//!   `EVT_GP_*` events. Host transmissions that are sink behaviour (GP
//!   Pairing, Proxy Commissioning Mode, Response, Translation Table
//!   Update/Request from endpoint 242, an alias `Device_annce`) are refused
//!   with `CONFLICT` and counted.
//! * `HOST_SINK`: the NCP sink is idle. GP server frames from proxies reach
//!   the host as `EVT_APS_DATA_INDICATION` on endpoint 242, and so do the
//!   local proxy's GP Notifications and Commissioning Notifications (flag
//!   bit 7). Host GP Pairing, Proxy Commissioning Mode and Response frames
//!   are also handed to the local proxy (a broadcast never loops back).
//!
//! The NCP sink defaults to `gpsSecurityLevel` 0x00 (decision N1) and has
//! no translation endpoint: every accepted GPD command reaches the host as
//! `EVT_GP_FRAME`. `GP_SET_SHARED_KEY` is answered `UNSUPPORTED_COMMAND`:
//! the sink provisions no shared GPD key (`zigbee_gp::sink`).
//!
//! [`ncp.rs`](crate::ncp) calls [`dispatch_gp`] for the GREEN POWER IDs,
//! [`screen_data_request`] before accepting an `APS_DATA_REQUEST`, and
//! [`emit_gp_events`] while the host has credit.

use heapless::Vec;
use zigbee_gp::cluster::{client_cmd, server_cmd};
use zigbee_gp::proxy::{MAX_SINK_GROUPS, ProxyEntry, SinkGroup};
use zigbee_gp::sink::{
    ImportError, SinkCommissioningExit, SinkConfig, SinkEntry, SinkEvent, SinkStats,
    exit_mode_attr, security_level_attr,
};
use zigbee_gp::types::{CommunicationMode, GpdId, KeyType, SecurityLevel};
use zigbee_gp::{GP_CLUSTER_ID, GP_ENDPOINT, GP_PROFILE_ID};
use zigbee_runtime::green_power::ncp::{self as rt, GpFrameReport, GpLocalFrame, GpNcpEvent};
use zigbee_zrhp::codec::{Bytes8, Bytes16, CodecError, List, Reader, Wire, Writer};
use zigbee_zrhp::msg::data::{ApsDataRequest, ApsDst, EvtApsDataIndication};
use zigbee_zrhp::msg::diag::Counter;
use zigbee_zrhp::msg::gp::{
    EvtGpCommissioned, EvtGpCommissioning, EvtGpCommissioningFailed, EvtGpDecommissioned,
    EvtGpFrame, GpProxyTableRead, GpSetSharedKey, GpSinkCommissioning, GpSinkConfig, GpSinkRemove,
    GpSinkTablePut, GpSinkTablePutRsp, GpSinkTableRead, sink_mode,
};
use zigbee_zrhp::msg::system::caps;
use zigbee_zrhp::msg::{Kind, MessageDef, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::records::{GpProxyRecord, GpSinkRecord, Record, gp_sink_flags};
use zigbee_zrhp::types::{Endpoint, Ieee, Status};

use crate::ncp::QUEUE_BYTES;
use crate::port::{CoordinatorPort, NetworkState};
use crate::tc::{TcApp, TcPort};

/// `R_GP_PROXY.format_version`: entries in the `gp_store` encoding (a
/// length byte, then the GP 1.1 Proxy Table attribute entry).
pub const GP_PROXY_FORMAT_VERSION: u8 = 1;
/// Record bytes of one `GP_*_TABLE_READ` response; the host continues at
/// `start + n`.
pub const TABLE_READ_BUDGET: usize = 400;
/// Records of one `GP_SINK_TABLE_PUT`.
pub const MAX_PUT_RECORDS: usize = 16;
/// DIAG group bit of the GP counters (`0x08xx`).
pub const DIAG_GROUP_GP: u8 = 0x80;
/// `EVT_APS_DATA_INDICATION.flags` bit 7: local proxy output.
pub const INDICATION_LOCAL: u8 = 0x80;

/// What the GREEN POWER group needs from the stack.
pub trait GpPort {
    /// `HOST_SINK` mode.
    fn host_sink(&self) -> bool;
    /// The NCP sink's configuration.
    fn sink_config(&self) -> SinkConfig;
    /// Select the sink authority and the NCP sink's configuration.
    fn configure(&mut self, host_sink: bool, config: SinkConfig);
    /// Enter NCP sink commissioning mode.
    fn enter_commissioning(&mut self, involve_proxies: bool) -> Result<(), zigbee_gp::Error>;
    /// Leave NCP sink commissioning mode.
    fn exit_commissioning(&mut self);
    fn sink_entries(&self) -> &[SinkEntry];
    fn proxy_entries(&self) -> &[ProxyEntry];
    fn sink_stats(&self) -> SinkStats;
    /// Add a Sink Table entry; GP Pairing only with `announce`.
    fn import(&mut self, entry: SinkEntry, announce: bool) -> Result<(), ImportError>;
    /// Remove a Sink Table entry; GP Pairing (RemoveGPD) only with
    /// `announce`.
    fn remove(&mut self, gpd: &GpdId, announce: bool) -> bool;
    /// Next report, sink event or local proxy copy.
    fn take_event(&mut self) -> Option<GpNcpEvent>;
    /// A host GP client command for the local proxy.
    fn loopback(&mut self, command_id: u8, payload: &[u8]);
}

impl<A: TcApp> GpPort for TcPort<A> {
    fn host_sink(&self) -> bool {
        self.app().device().gp_ncp_host_sink()
    }

    fn sink_config(&self) -> SinkConfig {
        *self.app().device().gp_sink().config()
    }

    fn configure(&mut self, host_sink: bool, config: SinkConfig) {
        self.app_mut()
            .device_mut()
            .gp_ncp_configure(host_sink, config);
    }

    fn enter_commissioning(&mut self, involve_proxies: bool) -> Result<(), zigbee_gp::Error> {
        self.app_mut()
            .device_mut()
            .gp_sink_enter_commissioning(involve_proxies)
    }

    fn exit_commissioning(&mut self) {
        self.app_mut().device_mut().gp_sink_exit_commissioning();
    }

    fn sink_entries(&self) -> &[SinkEntry] {
        self.app().device().gp_sink().entries()
    }

    fn proxy_entries(&self) -> &[ProxyEntry] {
        self.app().device().gp_proxy().entries()
    }

    fn sink_stats(&self) -> SinkStats {
        self.app().device().gp_sink().stats()
    }

    fn import(&mut self, entry: SinkEntry, announce: bool) -> Result<(), ImportError> {
        self.app_mut().device_mut().gp_ncp_import(entry, announce)
    }

    fn remove(&mut self, gpd: &GpdId, announce: bool) -> bool {
        self.app_mut().device_mut().gp_ncp_remove(gpd, announce)
    }

    fn take_event(&mut self) -> Option<GpNcpEvent> {
        self.app_mut().device_mut().gp_ncp_take_event()
    }

    fn loopback(&mut self, command_id: u8, payload: &[u8]) {
        self.app_mut()
            .device_mut()
            .gp_ncp_loopback(command_id, payload);
    }
}

/// Apply the NCP GP defaults to a freshly composed coordinator.
pub(crate) fn reset_defaults<A: TcApp>(app: &mut A) {
    app.device_mut().gp_ncp_reset();
}

// ── Record mapping (spec 9.3, `R_GP_SINK`) ──────────────────────────

/// The ZRHP GPD identifier.
pub fn zrhp_gpd_id(gpd: &GpdId) -> zigbee_zrhp::records::GpdId {
    match *gpd {
        GpdId::SrcId(id) => zigbee_zrhp::records::GpdId::SrcId(id),
        GpdId::Ieee { ieee, endpoint } => zigbee_zrhp::records::GpdId::Ieee {
            ieee: Ieee(ieee),
            endpoint,
        },
    }
}

/// The GP GPD identifier.
pub fn gp_gpd_id(gpd: &zigbee_zrhp::records::GpdId) -> GpdId {
    match *gpd {
        zigbee_zrhp::records::GpdId::SrcId(id) => GpdId::SrcId(id),
        zigbee_zrhp::records::GpdId::Ieee { ieee, endpoint } => GpdId::Ieee {
            ieee: ieee.0,
            endpoint,
        },
    }
}

/// `R_GP_SINK` of a Sink Table entry; the key only with `with_key`. A
/// commissioned groupcast entry exports its first group (the record has
/// one).
pub fn sink_record(entry: &SinkEntry, with_key: bool) -> GpSinkRecord {
    let mut flags = 0;
    if entry.mac_seq_cap {
        flags |= gp_sink_flags::MAC_SEQ_CAPABLE;
    }
    if entry.rx_on_cap {
        flags |= gp_sink_flags::RX_ON_CAPABLE;
    }
    if entry.fixed_location {
        flags |= gp_sink_flags::FIXED_LOCATION;
    }
    GpSinkRecord {
        gpd_id: zrhp_gpd_id(&entry.gpd_id),
        device_id: entry.device_id,
        communication_mode: entry.communication_mode as u8,
        group: entry.groups.first().map_or(0xFFFF, |g| g.group),
        assigned_alias: entry.assigned_alias.unwrap_or(0xFFFF),
        flags,
        security_level: entry.security_level as u8,
        key_type: entry.key_type as u8,
        key: if with_key { entry.key } else { None },
        frame_counter: entry.frame_counter,
        groupcast_radius: entry.groupcast_radius,
    }
}

/// The Sink Table entry an `R_GP_SINK` describes; `INVALID_PARAMETER` for
/// a reserved communication mode, security level or key type.
pub fn sink_entry(record: &GpSinkRecord) -> Result<SinkEntry, Status> {
    if record.communication_mode > 3 || record.security_level > 3 {
        return Err(Status::InvalidParameter);
    }
    let level = SecurityLevel::from_bits(record.security_level);
    if level == SecurityLevel::Reserved {
        return Err(Status::InvalidParameter);
    }
    let mut entry = SinkEntry::new(gp_gpd_id(&record.gpd_id));
    entry.device_id = record.device_id;
    entry.communication_mode = CommunicationMode::from_bits(record.communication_mode);
    if entry.communication_mode == CommunicationMode::CommissionedGroupcast {
        if record.group == 0xFFFF {
            return Err(Status::InvalidParameter);
        }
        let _ = entry.groups.push(SinkGroup {
            group: record.group,
            alias: None,
        });
    }
    entry.assigned_alias = (record.assigned_alias != 0xFFFF).then_some(record.assigned_alias);
    entry.mac_seq_cap = record.flags & gp_sink_flags::MAC_SEQ_CAPABLE != 0;
    entry.rx_on_cap = record.flags & gp_sink_flags::RX_ON_CAPABLE != 0;
    entry.fixed_location = record.flags & gp_sink_flags::FIXED_LOCATION != 0;
    entry.security_level = level;
    entry.key_type = KeyType::from_bits(record.key_type).map_err(|_| Status::InvalidParameter)?;
    entry.key = record.key;
    entry.frame_counter = record.frame_counter;
    entry.groupcast_radius = record.groupcast_radius;
    Ok(entry)
}

const _: () = assert!(MAX_SINK_GROUPS >= 1);

/// The TLV length of a record.
fn record_len(record: &Record<'_>) -> usize {
    let mut scratch = [0u8; 128];
    let mut w = Writer::new(&mut scratch);
    match w.write(record) {
        Ok(()) => w.len(),
        Err(_) => usize::MAX,
    }
}

/// `GP_SINK_TABLE_READ` response body, encoded straight into the queue.
struct SinkTableBody<'s> {
    entries: &'s [SinkEntry],
    start: usize,
    with_keys: bool,
}

impl MessageDef for SinkTableBody<'_> {
    const ID: u16 = ids::GP_SINK_TABLE_READ;
    const KIND: Kind = Kind::Response;
}

impl<'a> Wire<'a> for SinkTableBody<'_> {
    fn read(_: &mut Reader<'a>) -> Result<Self, CodecError> {
        Err(CodecError::Invalid)
    }

    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.u8(self.entries.len() as u8)?;
        let at = w.reserve(1)?;
        let mut n = 0u8;
        let mut budget = TABLE_READ_BUDGET;
        for entry in self.entries.iter().skip(self.start) {
            let record = Record::GpSink(sink_record(entry, self.with_keys));
            let len = record_len(&record);
            if len > budget {
                break;
            }
            w.write(&record)?;
            budget -= len;
            n += 1;
        }
        w.patch(at, &[n]);
        Ok(())
    }
}

/// `GP_PROXY_TABLE_READ` response body: one `R_GP_PROXY` per entry.
struct ProxyTableBody<'s> {
    entries: &'s [ProxyEntry],
    start: usize,
    with_keys: bool,
}

impl MessageDef for ProxyTableBody<'_> {
    const ID: u16 = ids::GP_PROXY_TABLE_READ;
    const KIND: Kind = Kind::Response;
}

impl<'a> Wire<'a> for ProxyTableBody<'_> {
    fn read(_: &mut Reader<'a>) -> Result<Self, CodecError> {
        Err(CodecError::Invalid)
    }

    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.u8(self.entries.len() as u8)?;
        let at = w.reserve(1)?;
        let mut n = 0u8;
        let mut budget = TABLE_READ_BUDGET;
        let mut encoded = [0u8; 1 + zigbee_gp::proxy::MAX_ENTRY_ENCODED_LEN];
        for entry in self.entries.iter().skip(self.start) {
            let Ok(len) = proxy_entry_bytes(entry, self.with_keys, &mut encoded) else {
                return Err(CodecError::Invalid);
            };
            let record = Record::GpProxy(GpProxyRecord {
                format_version: GP_PROXY_FORMAT_VERSION,
                entries: &encoded[..len],
            });
            // Type, length, format version, entries.
            let record_len = 3 + 1 + len;
            if record_len > budget {
                break;
            }
            w.write(&record)?;
            budget -= record_len;
            n += 1;
        }
        w.patch(at, &[n]);
        Ok(())
    }
}

/// One Proxy Table entry in the `gp_store` encoding (length byte, then the
/// attribute entry); without `with_key` the key bytes are zero.
pub fn proxy_entry_bytes(
    entry: &ProxyEntry,
    with_key: bool,
    out: &mut [u8; 1 + zigbee_gp::proxy::MAX_ENTRY_ENCODED_LEN],
) -> Result<usize, zigbee_gp::Error> {
    let n = if with_key || entry.key.is_none() {
        entry.encode(&mut out[1..])?
    } else {
        let mut keyless = entry.clone();
        keyless.key = None;
        keyless.encode(&mut out[1..])?
    };
    out[0] = n as u8;
    Ok(1 + n)
}

// ── Dispatch ────────────────────────────────────────────────────────

/// Facts for `SYS_NEGOTIATE`: capability bits, `gp_sink_table`,
/// `gp_proxy_table` (all zero without a GP port).
pub(crate) fn negotiate_facts<C: CoordinatorPort>(port: &mut C) -> (u64, u32, u32) {
    match port.gp() {
        Some(_) => (
            caps::GREEN_POWER_PROXY | caps::GREEN_POWER_SINK,
            zigbee_runtime::green_power::GP_SINK_TABLE_ENTRIES as u32,
            zigbee_runtime::green_power::GP_PROXY_TABLE_ENTRIES as u32,
        ),
        None => (0, 0, 0),
    }
}

/// Answer a GREEN POWER request (or `UNSUPPORTED_COMMAND` for any other
/// ID). `keys_exportable`: key export is built in and not locked.
// Own frame: its buffers stay out of the root future's poll frame.
#[inline(never)]
pub(crate) fn dispatch_gp<C: CoordinatorPort>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    endpoint: Endpoint,
    message: &RawMessage<'_>,
    keys_exportable: bool,
    port: &mut C,
) {
    let id = message.header.msg_id;
    let tid = message.header.tid;
    let status = |queues: &mut EndpointQueues<QUEUE_BYTES>, status: Status| {
        let _ = queues
            .queue(endpoint)
            .push_with(|w| w.response_status(id, tid, status));
    };
    if !is_gp_request(id) {
        status(queues, Status::UnsupportedCommand);
        return;
    }
    let up = port.state() == NetworkState::Up;
    let Some(gp) = port.gp() else {
        status(queues, Status::UnsupportedCommand);
        return;
    };
    macro_rules! decode {
        ($ty:ty) => {
            match message.decode::<$ty>() {
                Ok(value) => value,
                Err(_) => {
                    status(queues, Status::InvalidParameter);
                    return;
                }
            }
        };
    }
    match id {
        ids::GP_SINK_CONFIG => {
            let request = decode!(GpSinkConfig);
            match sink_config(&request, gp.sink_config()) {
                Ok(config) => {
                    gp.configure(request.mode == sink_mode::HOST_SINK, config);
                    status(queues, Status::Success);
                }
                Err(error) => status(queues, error),
            }
        }
        ids::GP_SET_SHARED_KEY => {
            let _ = decode!(GpSetSharedKey);
            status(queues, Status::UnsupportedCommand);
        }
        ids::GP_SINK_COMMISSIONING => {
            let request = decode!(GpSinkCommissioning);
            if gp.host_sink() {
                status(queues, Status::InvalidState);
                return;
            }
            match request.action {
                1 => {
                    if !up {
                        status(queues, Status::InvalidState);
                        return;
                    }
                    let mut config = gp.sink_config();
                    if request.window_s != 0 {
                        config.commissioning_window_s = request.window_s;
                    }
                    if request.flags & 0x02 != 0 {
                        config.exit_mode |= exit_mode_attr::ON_FIRST_PAIRING;
                    } else {
                        config.exit_mode &= !exit_mode_attr::ON_FIRST_PAIRING;
                    }
                    gp.configure(false, config);
                    match gp.enter_commissioning(request.flags & 0x01 != 0) {
                        Ok(()) => status(queues, Status::Success),
                        Err(_) => status(queues, Status::InvalidState),
                    }
                }
                0 => {
                    gp.exit_commissioning();
                    status(queues, Status::Success);
                }
                _ => status(queues, Status::InvalidParameter),
            }
        }
        ids::GP_SINK_TABLE_READ => {
            let request = decode!(GpSinkTableRead);
            let entries = gp.sink_entries();
            if usize::from(request.start) > entries.len() {
                status(queues, Status::InvalidParameter);
                return;
            }
            respond_table(
                queues,
                endpoint,
                tid,
                &SinkTableBody {
                    entries,
                    start: usize::from(request.start),
                    with_keys: keys_exportable,
                },
            );
        }
        ids::GP_PROXY_TABLE_READ => {
            let request = decode!(GpProxyTableRead);
            let entries = gp.proxy_entries();
            if usize::from(request.start) > entries.len() {
                status(queues, Status::InvalidParameter);
                return;
            }
            respond_table(
                queues,
                endpoint,
                tid,
                &ProxyTableBody {
                    entries,
                    start: usize::from(request.start),
                    with_keys: keys_exportable,
                },
            );
        }
        ids::GP_SINK_TABLE_PUT => {
            let request = decode!(GpSinkTablePut);
            // The table is restored from NV when the network resumes; an
            // import must not race that restore.
            if !up {
                status(queues, Status::InvalidState);
                return;
            }
            if request.records.len() > MAX_PUT_RECORDS {
                status(queues, Status::TooLarge);
                return;
            }
            let announce = request.flags & 0x01 != 0;
            let mut statuses: Vec<Status, MAX_PUT_RECORDS> = Vec::new();
            for record in request.records.iter() {
                let result = match record {
                    Record::GpSink(record) => match sink_entry(&record) {
                        Ok(entry) => match gp.import(entry, announce) {
                            Ok(()) => Status::Success,
                            Err(ImportError::AlreadyExists) => Status::AlreadyExists,
                            Err(ImportError::TableFull) => Status::NoSpace,
                            Err(ImportError::Invalid) => Status::InvalidParameter,
                        },
                        Err(error) => error,
                    },
                    _ => Status::InvalidParameter,
                };
                let _ = statuses.push(result);
            }
            let response = GpSinkTablePutRsp {
                statuses: List::new(&statuses),
            };
            let _ = queues.response(tid, Status::Success, Some(&response));
        }
        ids::GP_SINK_REMOVE => {
            let request = decode!(GpSinkRemove);
            if !up {
                status(queues, Status::InvalidState);
                return;
            }
            let gpd = gp_gpd_id(&request.gpd_id);
            if gp.remove(&gpd, request.flags & 0x01 != 0) {
                status(queues, Status::Success);
            } else {
                status(queues, Status::NotFound);
            }
        }
        _ => status(queues, Status::UnsupportedCommand),
    }
}

fn is_gp_request(id: u16) -> bool {
    matches!(
        id,
        ids::GP_SINK_CONFIG
            | ids::GP_SET_SHARED_KEY
            | ids::GP_SINK_COMMISSIONING
            | ids::GP_SINK_TABLE_READ
            | ids::GP_SINK_TABLE_PUT
            | ids::GP_SINK_REMOVE
            | ids::GP_PROXY_TABLE_READ
    )
}

/// Queue a table response when the queue can take the largest one.
fn respond_table<'a, T: Wire<'a> + MessageDef>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    endpoint: Endpoint,
    tid: u16,
    body: &T,
) {
    let queue = queues.queue(endpoint);
    let result = if queue.free() >= TABLE_READ_BUDGET + 32 {
        queue.response(tid, Status::Success, Some(body))
    } else {
        Err(CodecError::BufferFull)
    };
    if result.is_err() {
        let _ = queues
            .queue(endpoint)
            .push_with(|w| w.response_status(T::ID, tid, Status::Busy));
    }
}

/// The NCP sink configuration a `GP_SINK_CONFIG` asks for.
fn sink_config(request: &GpSinkConfig, current: SinkConfig) -> Result<SinkConfig, Status> {
    if request.mode > sink_mode::NCP_SINK
        || request.communication_mode > 3
        || request.exit_mode & !0x07 != 0
        || request.security_level & !0x0F != 0
    {
        return Err(Status::InvalidParameter);
    }
    if request.security_level & security_level_attr::INVOLVE_TC != 0 {
        // Commissioning through the Trust Center is not implemented.
        return Err(Status::UnsupportedCommand);
    }
    let communication_mode = CommunicationMode::from_bits(request.communication_mode);
    let group = (request.group != 0xFFFF).then_some(request.group);
    if communication_mode == CommunicationMode::CommissionedGroupcast && group.is_none() {
        return Err(Status::InvalidParameter);
    }
    Ok(SinkConfig {
        communication_mode,
        commissioned_group: group,
        exit_mode: request.exit_mode,
        commissioning_window_s: if request.window_s == 0 {
            zigbee_gp::sink::DEFAULT_SINK_COMMISSIONING_WINDOW_S
        } else {
            request.window_s
        },
        security_level: request.security_level,
        groupcast_radius: (request.groupcast_radius != 0).then_some(request.groupcast_radius),
        endpoint: None,
        ..current
    })
}

// ── APS_DATA_REQUEST screening ─────────────────────────────────────

/// A GP cluster-specific command from endpoint 242: (server → client,
/// CommandID, payload).
fn gp_command<'a>(request: &ApsDataRequest<'a>) -> Option<(bool, u8, &'a [u8])> {
    if request.profile != GP_PROFILE_ID
        || request.cluster != GP_CLUSTER_ID
        || request.src_ep != GP_ENDPOINT
    {
        return None;
    }
    let fc = *request.asdu.first()?;
    if fc & 0x03 != 0x01 || fc & 0x04 != 0 || request.asdu.len() < 3 {
        return None;
    }
    Some((fc & 0x08 != 0, request.asdu[2], &request.asdu[3..]))
}

/// Whether a host transmission is GP sink behaviour (spec 7.11 step 4).
pub fn is_sink_behaviour(request: &ApsDataRequest<'_>) -> bool {
    if request.profile == 0x0000 && request.cluster == 0x0013 {
        // Device_annce for a GPD alias: IEEE all ones.
        return request.asdu.get(3..11) == Some(&[0xFF; 8]);
    }
    match gp_command(request) {
        Some((true, command, _)) => matches!(
            command,
            client_cmd::PAIRING | client_cmd::PROXY_COMMISSIONING_MODE | client_cmd::RESPONSE
        ),
        Some((false, command, _)) => matches!(
            command,
            server_cmd::TRANSLATION_TABLE_UPDATE | server_cmd::TRANSLATION_TABLE_REQUEST
        ),
        None => false,
    }
}

/// Before an `APS_DATA_REQUEST` is accepted: `CONFLICT` for sink behaviour
/// in `NCP_SINK` mode; in `HOST_SINK` mode a GP Pairing, Proxy
/// Commissioning Mode or Response this node also receives is handed to its
/// own proxy, and the request continues.
pub(crate) fn screen_data_request<C: CoordinatorPort>(
    port: &mut C,
    request: &ApsDataRequest<'_>,
) -> Option<Status> {
    let own_nwk = port.info().short;
    let gp = port.gp()?;
    if !gp.host_sink() {
        if is_sink_behaviour(request) {
            rt::note_conflict();
            return Some(Status::Conflict);
        }
        return None;
    }
    let to_self = match request.dst {
        ApsDst::Broadcast(_) | ApsDst::Group(_) => true,
        ApsDst::Nwk(nwk) => nwk == own_nwk,
        ApsDst::Ieee(_) => false,
    };
    if let Some((true, command, payload)) = gp_command(request)
        && to_self
        && matches!(
            command,
            client_cmd::PAIRING | client_cmd::PROXY_COMMISSIONING_MODE | client_cmd::RESPONSE
        )
    {
        gp.loopback(command, payload);
    }
    None
}

// ── Events ──────────────────────────────────────────────────────────

/// Queue GP events while the ZIGBEE endpoint can take the largest one.
/// `keys_exportable`: records in `EVT_GP_COMMISSIONED` carry the key.
pub(crate) fn emit_gp_events<C: CoordinatorPort>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    seqs: &mut [u16; Endpoint::COUNT],
    keys_exportable: bool,
    port: &mut C,
) {
    let ieee = port.ieee();
    let Some(gp) = port.gp() else {
        return;
    };
    while queues.queue(Endpoint::Zigbee).free() >= 7 + 40 + crate::port::MAX_INDICATION_ASDU {
        let Some(event) = gp.take_event() else {
            break;
        };
        match event {
            GpNcpEvent::Frame(report) => {
                crate::ncp::push_event(queues, seqs, &frame_event(&report));
            }
            GpNcpEvent::LocalProxy(frame) => {
                crate::ncp::push_event(queues, seqs, &local_indication(&frame, ieee));
            }
            GpNcpEvent::Sink(SinkEvent::CommissioningEntered {
                window_s,
                involve_proxies,
            }) => {
                let event = EvtGpCommissioning {
                    state: 1,
                    window_s: window_s.unwrap_or(0xFFFF),
                    involve_proxies: u8::from(involve_proxies),
                    exit_reason: 0,
                };
                crate::ncp::push_event(queues, seqs, &event);
            }
            GpNcpEvent::Sink(SinkEvent::CommissioningLeft(reason)) => {
                let event = EvtGpCommissioning {
                    state: 0,
                    window_s: 0xFFFF,
                    involve_proxies: 0,
                    exit_reason: match reason {
                        SinkCommissioningExit::Explicit => 1,
                        SinkCommissioningExit::FirstPairing => 2,
                        SinkCommissioningExit::Timeout => 3,
                    },
                };
                crate::ncp::push_event(queues, seqs, &event);
            }
            GpNcpEvent::Sink(SinkEvent::Commissioned { gpd_id, device_id }) => {
                let entry = gp
                    .sink_entries()
                    .iter()
                    .find(|e| e.gpd_id.matches(&gpd_id))
                    .cloned()
                    .unwrap_or_else(|| {
                        let mut entry = SinkEntry::new(gpd_id);
                        entry.device_id = device_id;
                        entry
                    });
                let mut flags = 0;
                if entry.rx_on_cap {
                    flags |= 0x01;
                }
                if entry.key.is_some() {
                    flags |= 0x02;
                }
                let event = EvtGpCommissioned {
                    gpd_id: zrhp_gpd_id(&gpd_id),
                    device_id,
                    flags,
                    record: Record::GpSink(sink_record(&entry, keys_exportable)),
                };
                crate::ncp::push_event(queues, seqs, &event);
            }
            GpNcpEvent::Sink(SinkEvent::CommissioningFailed { gpd_id, reason }) => {
                let event = EvtGpCommissioningFailed {
                    gpd_id: zrhp_gpd_id(&gpd_id),
                    reason: rt::sink_drop_code(reason),
                };
                crate::ncp::push_event(queues, seqs, &event);
            }
            GpNcpEvent::Sink(SinkEvent::Decommissioned { gpd_id }) => {
                let event = EvtGpDecommissioned {
                    gpd_id: zrhp_gpd_id(&gpd_id),
                };
                crate::ncp::push_event(queues, seqs, &event);
            }
            // Reported as `EVT_GP_FRAME` when accepted.
            GpNcpEvent::Sink(SinkEvent::GpdCommand { .. }) => {}
        }
    }
}

/// `EVT_GP_FRAME` of a report.
pub fn frame_event(report: &GpFrameReport) -> EvtGpFrame<'_> {
    EvtGpFrame {
        origin: report.origin,
        result: report.result,
        gpd_id: zrhp_gpd_id(&report.gpd_id),
        device_id: report.device_id,
        command_id: report.command_id,
        frame_counter: report.frame_counter,
        security_level: report.security_level,
        key_type: report.key_type,
        flags: report.flags,
        proxy_nwk: report.proxy_nwk,
        gpp_gpd_link: report.gpp_gpd_link,
        rssi: report.rssi,
        lqi: report.lqi,
        payload: Bytes8(&report.payload),
    }
}

/// `EVT_APS_DATA_INDICATION` (flag bit 7) of a local proxy frame.
pub fn local_indication(frame: &GpLocalFrame, ieee: [u8; 8]) -> EvtApsDataIndication<'_> {
    let mut flags = INDICATION_LOCAL | 0x02 | 0x04;
    match frame.dst_mode {
        0x0F => flags |= 0x08,
        0x01 => flags |= 0x10,
        _ => {}
    }
    EvtApsDataIndication {
        dst_mode: frame.dst_mode,
        dst: frame.dst,
        dst_ep: GP_ENDPOINT,
        src_nwk: frame.src_nwk,
        src_ieee: Ieee(ieee),
        src_ep: GP_ENDPOINT,
        profile: GP_PROFILE_ID,
        cluster: GP_CLUSTER_ID,
        flags,
        lqi: 0xFF,
        rssi: -128,
        last_hop_nwk: frame.src_nwk,
        aps_counter: 0,
        key_source: 0,
        rx_time_ms: 0,
        asdu: Bytes16(&frame.asdu),
    }
}

// ── DIAG ────────────────────────────────────────────────────────────

/// GP counters (group bit 7, IDs `0x08xx`): `0x0801..=0x080D` sink drops
/// by reason (1 Invalid … 13 QueueFull, duplicates are `0x0807`), `0x0810`
/// reports lost, `0x0811` local proxy copies, `0x0812` frames handed to a
/// host sink, `0x0813` host transmissions refused with `CONFLICT`.
pub(crate) fn append_counters<const N: usize>(group_mask: u8, out: &mut Vec<Counter, N>) {
    if group_mask & DIAG_GROUP_GP == 0 {
        return;
    }
    let counters = rt::counters();
    for (index, value) in counters.sink_drops.iter().enumerate() {
        let _ = out.push(Counter {
            id: 0x0801 + index as u16,
            value: *value,
        });
    }
    for (id, value) in [
        (0x0810, counters.reports_dropped),
        (0x0811, counters.local_copies),
        (0x0812, counters.host_sink_frames),
        (0x0813, counters.conflicts),
    ] {
        let _ = out.push(Counter { id, value });
    }
}

/// `DIAG_RESET_COUNTERS` for the GP group.
pub(crate) fn reset_counters(group_mask: u8) {
    if group_mask & DIAG_GROUP_GP != 0 {
        rt::reset_counters();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: zigbee_zrhp::types::Key = [0xA5; 16];

    fn protected_entry() -> SinkEntry {
        let mut entry = SinkEntry::new(GpdId::SrcId(0x0042_1337));
        entry.device_id = 0x02;
        entry.communication_mode = CommunicationMode::CommissionedGroupcast;
        entry
            .groups
            .push(SinkGroup {
                group: 0x0B84,
                alias: None,
            })
            .unwrap();
        entry.assigned_alias = Some(0x4321);
        entry.mac_seq_cap = true;
        entry.fixed_location = true;
        entry.security_level = SecurityLevel::Encrypted;
        entry.key_type = KeyType::IndividualKey;
        entry.key = Some(KEY);
        entry.frame_counter = 0x0102_0304;
        entry.groupcast_radius = 7;
        entry
    }

    #[test]
    fn sink_record_round_trips_through_the_wire() {
        let entry = protected_entry();
        let record = sink_record(&entry, true);
        assert_eq!(
            record.gpd_id,
            zigbee_zrhp::records::GpdId::SrcId(0x0042_1337)
        );
        assert_eq!(
            (
                record.communication_mode,
                record.group,
                record.assigned_alias
            ),
            (2, 0x0B84, 0x4321)
        );
        assert_eq!(
            record.flags,
            gp_sink_flags::MAC_SEQ_CAPABLE | gp_sink_flags::FIXED_LOCATION
        );
        assert_eq!((record.security_level, record.key_type), (3, 0b100));
        let mut buf = [0u8; 64];
        let mut w = Writer::new(&mut buf);
        w.write(&Record::GpSink(record.clone())).unwrap();
        let len = w.len();
        // TLV header, SrcID GPD ID (5), fixed fields (9), key, counter, radius.
        assert_eq!(len, 3 + 5 + 9 + 16 + 4 + 1);
        assert_eq!(buf[0], 0x0B);
        assert_ne!(buf[3 + 5 + 6] & gp_sink_flags::KEY_PRESENT, 0);
        let decoded: Record<'_> = Reader::new(&buf[..len]).read().unwrap();
        let Record::GpSink(decoded) = decoded else {
            panic!("not R_GP_SINK");
        };
        let back = sink_entry(&decoded).unwrap();
        assert_eq!(back.gpd_id, entry.gpd_id);
        assert_eq!(back.groups.as_slice(), entry.groups.as_slice());
        assert_eq!(
            (back.key, back.frame_counter, back.assigned_alias),
            (Some(KEY), 0x0102_0304, Some(0x4321))
        );
        assert_eq!(
            (back.mac_seq_cap, back.rx_on_cap, back.fixed_location),
            (true, false, true)
        );
        assert_eq!(back.groupcast_radius, 7);
    }

    #[test]
    fn locked_export_omits_the_key() {
        let record = sink_record(&protected_entry(), false);
        assert_eq!(record.key, None);
        let mut buf = [0u8; 64];
        let mut w = Writer::new(&mut buf);
        w.write(&Record::GpSink(record)).unwrap();
        assert_eq!(w.len(), 3 + 5 + 9 + 4 + 1);
        assert_eq!(buf[3 + 5 + 6] & gp_sink_flags::KEY_PRESENT, 0);

        let mut proxy = ProxyEntry::new(GpdId::SrcId(7));
        proxy.security_level = SecurityLevel::Encrypted;
        proxy.key_type = KeyType::IndividualKey;
        proxy.key = Some(KEY);
        let mut with = [0u8; 1 + zigbee_gp::proxy::MAX_ENTRY_ENCODED_LEN];
        let mut without = with;
        let n = proxy_entry_bytes(&proxy, true, &mut with).unwrap();
        assert_eq!(proxy_entry_bytes(&proxy, false, &mut without).unwrap(), n);
        assert!(with[..n].windows(16).any(|w| w == KEY));
        assert!(!without[..n].windows(16).any(|w| w == KEY));
        assert_eq!(usize::from(with[0]), n - 1);
    }

    #[test]
    fn ieee_gpds_and_reserved_values_map_explicitly() {
        let gpd = GpdId::Ieee {
            ieee: [1, 2, 3, 4, 5, 6, 7, 8],
            endpoint: 3,
        };
        assert_eq!(gp_gpd_id(&zrhp_gpd_id(&gpd)), gpd);
        let mut record = sink_record(&SinkEntry::new(gpd), true);
        assert_eq!((record.group, record.assigned_alias), (0xFFFF, 0xFFFF));
        assert_eq!(record.key, None);
        assert!(sink_entry(&record).is_ok());
        record.security_level = 1;
        assert_eq!(sink_entry(&record).err(), Some(Status::InvalidParameter));
        record.security_level = 0;
        record.key_type = 0b101;
        assert_eq!(sink_entry(&record).err(), Some(Status::InvalidParameter));
        record.key_type = 0;
        record.communication_mode = 2;
        assert_eq!(
            sink_entry(&record).err(),
            Some(Status::InvalidParameter),
            "pre-commissioned groupcast needs a group"
        );
    }

    #[test]
    fn sink_config_requests_are_validated() {
        let request = GpSinkConfig {
            mode: sink_mode::NCP_SINK,
            communication_mode: 2,
            group: 0x0B84,
            exit_mode: 0x03,
            window_s: 0,
            security_level: 0,
            groupcast_radius: 0,
        };
        let config = sink_config(&request, rt::ncp_sink_config()).unwrap();
        assert_eq!(
            (config.communication_mode, config.commissioned_group),
            (CommunicationMode::CommissionedGroupcast, Some(0x0B84))
        );
        assert_eq!(config.commissioning_window_s, 180);
        assert_eq!((config.security_level, config.endpoint), (0, None));
        assert_eq!(config.groupcast_radius, None);
        let bad = |change: fn(&mut GpSinkConfig)| {
            let mut r = request.clone();
            change(&mut r);
            sink_config(&r, rt::ncp_sink_config()).err()
        };
        assert_eq!(bad(|r| r.mode = 2), Some(Status::InvalidParameter));
        assert_eq!(bad(|r| r.group = 0xFFFF), Some(Status::InvalidParameter));
        assert_eq!(bad(|r| r.exit_mode = 0x08), Some(Status::InvalidParameter));
        assert_eq!(
            bad(|r| r.security_level = 0x08),
            Some(Status::UnsupportedCommand)
        );
    }

    #[test]
    fn sink_behaviour_from_the_host_is_recognised() {
        let request = |profile, cluster, src_ep, asdu: &'static [u8]| ApsDataRequest {
            dst: ApsDst::Broadcast(0xFFFD),
            dst_ep: GP_ENDPOINT,
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
        };
        // GP Pairing, server → client, from endpoint 242.
        assert!(is_sink_behaviour(&request(
            0xA1E0,
            0x0021,
            242,
            &[0x19, 1, 0x01, 0]
        )));
        assert!(is_sink_behaviour(&request(
            0xA1E0,
            0x0021,
            242,
            &[0x19, 1, 0x06]
        )));
        // Translation Table Update (client → server).
        assert!(is_sink_behaviour(&request(
            0xA1E0,
            0x0021,
            242,
            &[0x11, 1, 0x07]
        )));
        // A GP Notification Response is not.
        assert!(!is_sink_behaviour(&request(
            0xA1E0,
            0x0021,
            242,
            &[0x19, 1, 0x00]
        )));
        // Another endpoint or profile is not.
        assert!(!is_sink_behaviour(&request(
            0x0104,
            0x0021,
            242,
            &[0x19, 1, 0x01]
        )));
        assert!(!is_sink_behaviour(&request(
            0xA1E0,
            0x0021,
            1,
            &[0x19, 1, 0x01]
        )));
        // An alias Device_annce is; a real one is not.
        let annce: &'static [u8] = &[
            0, 0x34, 0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0,
        ];
        assert!(is_sink_behaviour(&request(0, 0x0013, 0, annce)));
        let real: &'static [u8] = &[0, 0x34, 0x12, 1, 2, 3, 4, 5, 6, 7, 8, 0x8E];
        assert!(!is_sink_behaviour(&request(0, 0x0013, 0, real)));
    }
}
