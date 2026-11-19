//! BACKUP group (`0x06xx`, endpoint BULK), spec 9.4–9.5.
//!
//! **Export** streams a record set built from the running network:
//! `NODE`, `NETWORK`, `NETWORK_KEY`, `TC_LINK_KEY` (keys only with export
//! flag bit0 and an unlocked key export), one `DEVICE` per Trust Center
//! record or child (flag bit1), `RS_PARAMS`, then `END` with the count and
//! CRC-32C. Counters are exported as durable limits; every link-key
//! `tx_counter` is the global durable APS limit (8.4). The records are
//! generated on demand per `BKP_EXPORT_READ` (no RAM copy): a device that
//! joins or leaves while a transfer runs can appear or vanish between reads.
//! GP tables (flag bit2) are not part of the stream: hosts read them with
//! `GP_*_TABLE_READ`.
//!
//! **Restore** stages the network records and up to [`MAX_STAGED_KEYS`]
//! device link keys in RAM, validates every record as it arrives (9.4 rule
//! 2) and applies them at `BKP_RESTORE_COMMIT` after checking `END`:
//! counters by the 9.4 rule 4, the IEEE override when allowed (rule 7), the
//! network (the previous one is wiped only at commit), then the link keys.
//! Children of the NCP (`DEVICE` with a child block, up to
//! [`MAX_STAGED_CHILDREN`]) are written to the durable child table before
//! the resume, so they keep their parent and short address. Deviations,
//! reported as notes: link keys beyond the staging capacity
//! (`HOST_BACKED`), children beyond it (`CHILD_DEMOTED`: they rejoin), GP
//! records, the EZSP hashed TCLK and unknown records (`IGNORED`). The network is started at commit (flags bit1 is implied);
//! a later `NWK_INIT` only applies floors. Restore needs the stack stopped
//! (after a reboot, before `NWK_INIT`).

use heapless::Vec;
use zigbee_zrhp::codec::List;
use zigbee_zrhp::codec::{Reader, Writer};
use zigbee_zrhp::msg::backup::{
    BkpExportBegin, BkpExportBeginRsp, BkpExportEnd, BkpExportRead, BkpRestoreAbort,
    BkpRestoreBegin, BkpRestoreBeginRsp, BkpRestoreCommit, BkpRestoreCommitRsp, RestoreNote, note,
};
use zigbee_zrhp::msg::{Kind, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::records::{
    EndRecord, NetworkKeyRecord, NetworkRecord, NodeRecord, RawRecords, Record, RestoreCounters,
    RsParamsRecord, StreamCrc, TcLinkKeyRecord, record_type,
};
use zigbee_zrhp::types::{Endpoint, Ieee, Status};

use crate::ncp::{FC_STRIDE, QUEUE_BYTES};
use crate::platform::NcpPlatform;
use crate::port::{CoordinatorPort, NetworkImage, NetworkState, OpError, RestoredChild};
use crate::tables::device_record;

/// NCP children staged by a restore; more are `CHILD_DEMOTED` (they
/// rejoin).
pub const MAX_STAGED_CHILDREN: usize = 64;
/// Device link keys staged by a restore; more are `HOST_BACKED`.
pub const MAX_STAGED_KEYS: usize = 64;
/// `max_records_per_put` advertised by `BKP_RESTORE_BEGIN`.
pub const MAX_RECORDS_PER_PUT: u16 = 12;
/// Largest export chunk (record bytes per `BKP_EXPORT_READ`).
pub const MAX_EXPORT_CHUNK: usize = 480;

/// Whether `id` belongs to the BACKUP group.
pub(crate) const fn is_backup_request(id: u16) -> bool {
    matches!(
        id,
        ids::BKP_EXPORT_BEGIN
            | ids::BKP_EXPORT_READ
            | ids::BKP_EXPORT_END
            | ids::BKP_RESTORE_BEGIN
            | ids::BKP_RESTORE_PUT
            | ids::BKP_RESTORE_COMMIT
            | ids::BKP_RESTORE_ABORT
    )
}

/// A running export.
#[derive(Debug, Clone, Copy)]
pub struct Export {
    txn: u16,
    keys: bool,
    devices: bool,
    /// Next record position (see [`Export::record`]).
    next: usize,
    total: usize,
    crc: StreamCrc,
    table_epoch: u32,
}

/// A running restore.
#[derive(Debug, Clone)]
pub struct Restore {
    txn: u16,
    allow_ieee: bool,
    nwk_margin: u32,
    aps_margin: u32,
    crc: StreamCrc,
    counters: RestoreCounters,
    node: Option<NodeRecord>,
    network: Option<NetworkRecord>,
    network_key: Option<NetworkKeyRecord>,
    tclk_counter_known: bool,
    keys: Vec<([u8; 8], [u8; 16]), MAX_STAGED_KEYS>,
    host_backed: u16,
    children: Vec<RestoredChild, MAX_STAGED_CHILDREN>,
    end: Option<EndRecord>,
    put_index: u16,
}

/// Transactions of the BULK endpoint.
#[derive(Debug, Default)]
pub struct BackupState {
    export: Option<Export>,
    restore: Option<Restore>,
    next_txn: u16,
}

impl BackupState {
    pub const fn new() -> Self {
        Self {
            export: None,
            restore: None,
            next_txn: 0,
        }
    }

    fn txn(&mut self) -> u16 {
        self.next_txn = self.next_txn.wrapping_add(1).max(1);
        self.next_txn
    }

    /// A restore is staging (`SYS_GET_STATUS.restore_in_progress`).
    pub const fn restore_in_progress(&self) -> bool {
        self.restore.is_some()
    }

    /// Session lost: staging is discarded (9.4 rule 1).
    pub fn abort_all(&mut self) {
        self.export = None;
        self.restore = None;
    }
}

impl Export {
    /// Position layout: 0 NODE, 1 NETWORK, 2 NETWORK_KEY, 3 TC_LINK_KEY,
    /// 4.. devices, then RS_PARAMS, then END.
    fn record<C: CoordinatorPort>(&self, port: &C, position: usize) -> Option<Record<'static>> {
        let info = port.info();
        let counters = port.frame_counters();
        match position {
            0 => Some(Record::Node(NodeRecord {
                ieee: Ieee(port.ieee()),
                nwk: 0x0000,
                logical_type: 0,
                flags: 0,
            })),
            1 => Some(Record::Network(NetworkRecord {
                pan_id: info.pan_id,
                ext_pan_id: info.ext_pan_id,
                channel: info.channel,
                channel_mask: info.channel_mask,
                nwk_update_id: info.nwk_update_id,
                nwk_manager: info.nwk_manager,
                security_level: 5,
                tx_power: info.tx_power,
            })),
            2 if self.keys => port.network_key(0).map(|(key, seq)| {
                Record::NetworkKey(NetworkKeyRecord {
                    key,
                    seq,
                    tx_counter: counters.nwk_limit,
                    flags: 0x01,
                })
            }),
            3 if self.keys => port.tc_link_key().map(|key| {
                Record::TcLinkKey(TcLinkKeyRecord {
                    key,
                    tc_ieee: Ieee(port.ieee()),
                    tx_counter: counters.aps_limit,
                    flags: 0,
                })
            }),
            2 | 3 => None,
            _ => {
                let devices = if self.devices { self.total - 6 } else { 0 };
                let index = position - 4;
                if index < devices {
                    port.device(index).map(|view| {
                        Record::Device(device_record(&view, self.keys, counters.aps_limit))
                    })
                } else if index == devices {
                    Some(Record::RsParams(RsParamsRecord {
                        fc_stride: FC_STRIDE,
                        table_epoch: self.table_epoch,
                        nwk_fc_limit: counters.nwk_limit,
                        aps_fc_limit: counters.aps_limit,
                        flags: 0,
                    }))
                } else {
                    None
                }
            }
        }
    }
}

/// Handle one BACKUP request. `BKP_RESTORE_COMMIT` awaits the stack, so
/// it returns `true` and the caller queues it for [`commit_staged`].
// Own frame: its buffers stay out of the root future's poll frame.
#[inline(never)]
pub(crate) fn dispatch_backup<C: CoordinatorPort>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    endpoint: Endpoint,
    message: &RawMessage<'_>,
    keys_exportable: bool,
    state: &mut BackupState,
    port: &mut C,
) -> bool {
    let id = message.header.msg_id;
    let tid = message.header.tid;
    let payload = message.payload;
    let status = |queues: &mut EndpointQueues<QUEUE_BYTES>, status: Status| {
        let _ = queues
            .queue(endpoint)
            .push_with(|w| w.response_status(id, tid, status));
    };
    macro_rules! decode {
        ($ty:ty) => {
            match message.decode::<$ty>() {
                Ok(value) => value,
                Err(_) => {
                    status(queues, Status::InvalidParameter);
                    return false;
                }
            }
        };
    }
    match id {
        ids::BKP_EXPORT_BEGIN => {
            let request = decode!(BkpExportBegin);
            if port.state() != NetworkState::Up {
                status(queues, Status::InvalidState);
                return false;
            }
            let keys = request.flags & 0x01 != 0 && keys_exportable;
            let devices = request.flags & 0x02 != 0;
            let device_count = if devices {
                crate::tables::device_count(port)
            } else {
                0
            };
            // NODE, NETWORK, [NWK key, TCLK], devices, RS_PARAMS, END.
            let total = 6 + device_count;
            let txn = state.txn();
            state.export = Some(Export {
                txn,
                keys,
                devices,
                next: 0,
                total,
                crc: StreamCrc::new(),
                table_epoch: port.table_epoch(),
            });
            let records = total - if keys { 0 } else { 2 };
            let response = BkpExportBeginRsp {
                txn,
                record_count: records as u32,
            };
            let _ = queues.response(tid, Status::Success, Some(&response));
        }
        ids::BKP_EXPORT_READ => {
            let request = decode!(BkpExportRead);
            let Some(export) = state.export.as_mut().filter(|e| e.txn == request.txn) else {
                status(queues, Status::NotFound);
                return false;
            };
            let limit = usize::from(request.max_bytes).clamp(32, MAX_EXPORT_CHUNK);
            let mut payload = [0u8; 2 + MAX_EXPORT_CHUNK];
            payload[0] = Status::Success.to_u8();
            let mut w = Writer::new(&mut payload[2..2 + limit]);
            let end_position = export.total - 1;
            let mut done = false;
            while export.next <= end_position {
                let mark = w.len();
                let mut crc = export.crc;
                let result = if export.next == end_position {
                    crc_end(&crc, &mut w)
                } else {
                    match export.record(port, export.next) {
                        Some(record) => crc.write_record(&record, &mut w),
                        None => Ok(()),
                    }
                };
                if result.is_err() {
                    w.truncate(mark);
                    break;
                }
                export.crc = crc;
                export.next += 1;
                if export.next > end_position {
                    done = true;
                }
            }
            let written = w.len();
            if written == 0 && !done {
                status(queues, Status::InvalidParameter);
                return false;
            }
            payload[1] = u8::from(!done);
            let _ = queues
                .queue(endpoint)
                .raw(Kind::Response, id, tid, &payload[..2 + written]);
        }
        ids::BKP_EXPORT_END => {
            let request = decode!(BkpExportEnd);
            if state.export.as_ref().is_some_and(|e| e.txn == request.txn) {
                state.export = None;
                status(queues, Status::Success);
            } else {
                status(queues, Status::NotFound);
            }
        }
        ids::BKP_RESTORE_BEGIN => {
            let request = decode!(BkpRestoreBegin);
            if state.restore.is_some() {
                status(queues, Status::InProgress);
                return false;
            }
            if port.state() == NetworkState::Up {
                // The stack must be stopped: reboot, then restore before
                // NWK_INIT.
                status(queues, Status::InvalidState);
                return false;
            }
            let txn = state.txn();
            state.restore = Some(Restore {
                txn,
                allow_ieee: request.flags & 0x01 != 0,
                nwk_margin: request.nwk_margin,
                aps_margin: request.aps_margin,
                crc: StreamCrc::new(),
                counters: RestoreCounters::new(),
                node: None,
                network: None,
                network_key: None,
                tclk_counter_known: false,
                keys: Vec::new(),
                host_backed: 0,
                children: Vec::new(),
                end: None,
                put_index: 0,
            });
            let response = BkpRestoreBeginRsp {
                txn,
                max_records_per_put: MAX_RECORDS_PER_PUT,
            };
            let _ = queues.response(tid, Status::Success, Some(&response));
        }
        ids::BKP_RESTORE_PUT => {
            if payload.len() < 2 {
                status(queues, Status::InvalidParameter);
                return false;
            }
            let txn = u16::from_le_bytes([payload[0], payload[1]]);
            let Some(restore) = state.restore.as_mut().filter(|r| r.txn == txn) else {
                status(queues, Status::NotFound);
                return false;
            };
            let mut notes: Vec<RestoreNote, 16> = Vec::new();
            let mut accepted = 0u16;
            let mut failed = false;
            for item in RawRecords::new(&payload[2..]) {
                let Ok((rtype, tlv)) = item else {
                    failed = true;
                    break;
                };
                let index = restore.put_index;
                restore.put_index = restore.put_index.wrapping_add(1);
                if restore.end.is_some() {
                    failed = true;
                    break;
                }
                let Ok(record) = Reader::new(tlv).read::<Record<'_>>() else {
                    failed = true;
                    break;
                };
                if rtype != record_type::END {
                    restore.crc.add(tlv);
                }
                match stage(restore, &record, port.ieee()) {
                    Ok(Some(n)) => {
                        let _ = notes.push(RestoreNote { index, note: n });
                    }
                    Ok(None) => {}
                    Err(()) => {
                        failed = true;
                        break;
                    }
                }
                accepted += 1;
            }
            if failed {
                // 9.4 rule 2: a violation aborts the transaction.
                state.restore = None;
                status(queues, Status::InvalidParameter);
                return false;
            }
            let response = zigbee_zrhp::msg::backup::BkpRestorePutRsp {
                accepted,
                notes: List::new(&notes),
            };
            let _ = queues.response(tid, Status::Success, Some(&response));
        }
        ids::BKP_RESTORE_ABORT => {
            let request = decode!(BkpRestoreAbort);
            if state.restore.as_ref().is_some_and(|r| r.txn == request.txn) {
                state.restore = None;
                status(queues, Status::Success);
            } else {
                status(queues, Status::NotFound);
            }
        }
        ids::BKP_RESTORE_COMMIT => {
            let request = decode!(BkpRestoreCommit);
            if state.restore.as_ref().is_none_or(|r| r.txn != request.txn) {
                status(queues, Status::NotFound);
                return false;
            }
            // Answered by `commit_staged` once the stack ran it.
            return true;
        }
        _ => status(queues, Status::UnsupportedCommand),
    }
    false
}

/// Run a queued `BKP_RESTORE_COMMIT` and queue its response; `true` when
/// a network was installed.
pub(crate) async fn commit_staged<C: CoordinatorPort, P: NcpPlatform>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    tid: u16,
    state: &mut BackupState,
    port: &mut C,
    platform: &mut P,
) -> bool {
    // Committed in place: the staged keys (1.5 KiB) are never moved onto
    // the stack or into the future.
    let result = match state.restore.as_ref() {
        Some(restore) => commit(restore, port, platform).await,
        None => Err(Status::NotFound),
    };
    state.restore = None;
    let id = ids::BKP_RESTORE_COMMIT;
    match result {
        Ok(response) => {
            let _ = queues.response(tid, Status::Success, Some(&response));
            true
        }
        Err(status) => {
            if let Some(endpoint) = zigbee_zrhp::msg::home_endpoint(id) {
                let _ = queues
                    .queue(endpoint)
                    .push_with(|w| w.response_status(id, tid, status));
            }
            false
        }
    }
}

fn crc_end(crc: &StreamCrc, w: &mut Writer<'_>) -> Result<(), zigbee_zrhp::codec::CodecError> {
    use zigbee_zrhp::codec::Wire;
    Record::End(crc.end()).write(w)
}

/// Validate and stage one record; `Ok(Some(note))` reports a deviation,
/// `Err` violates 9.4 rule 2.
fn stage(restore: &mut Restore, record: &Record<'_>, own_ieee: [u8; 8]) -> Result<Option<u8>, ()> {
    restore.counters.add(record);
    match record {
        Record::Node(node) => {
            if node.nwk != 0x0000 || node.logical_type != 0 {
                return Err(());
            }
            if node.ieee.0 != own_ieee && !restore.allow_ieee {
                return Err(());
            }
            restore.node = Some(node.clone());
            Ok(None)
        }
        Record::Network(network) => {
            if network.security_level != 5
                || !(11..=26).contains(&network.channel)
                || matches!(network.pan_id, 0x0000 | 0xFFFF)
                || network.ext_pan_id == [0; 8]
                || network.ext_pan_id == [0xFF; 8]
            {
                return Err(());
            }
            restore.network = Some(network.clone());
            Ok(None)
        }
        Record::NetworkKey(key) => {
            if key.key == [0; 16] {
                return Err(());
            }
            restore.network_key = Some(key.clone());
            Ok(None)
        }
        Record::TcLinkKey(tclk) => {
            restore.tclk_counter_known = true;
            // The NCP keeps the standard global TCLK; another one cannot be
            // installed (its counter still raises the APS start).
            Ok((tclk.key != ZIGBEE_ALLIANCE_09).then_some(note::IGNORED))
        }
        Record::Device(device) => {
            let mut result = None;
            match &device.child {
                // A child of the NCP keeps its short address and parent.
                Some(child)
                    if (0x0001..=0xFFF7).contains(&device.nwk)
                        && restore
                            .children
                            .push(RestoredChild {
                                ieee: device.ieee.0,
                                nwk: device.nwk,
                                capability: child.capability,
                                timeout: child.timeout_exp.min(14),
                            })
                            .is_ok() => {}
                Some(_) => result = Some(note::CHILD_DEMOTED),
                None => {}
            }
            if let Some(link) = &device.link
                && restore.keys.push((device.ieee.0, link.key)).is_err()
            {
                restore.host_backed = restore.host_backed.saturating_add(1);
                result = Some(note::HOST_BACKED);
            }
            Ok(result)
        }
        Record::RsParams(_) => Ok(None),
        Record::End(end) => {
            restore.end = Some(end.clone());
            Ok(None)
        }
        _ => Ok(Some(note::IGNORED)),
    }
}

/// "ZigBeeAlliance09".
const ZIGBEE_ALLIANCE_09: [u8; 16] = *b"ZigBeeAlliance09";

async fn commit<C: CoordinatorPort, P: NcpPlatform>(
    restore: &Restore,
    port: &mut C,
    platform: &mut P,
) -> Result<BkpRestoreCommitRsp, Status> {
    let end = restore.end.clone().ok_or(Status::InvalidParameter)?;
    let expected = restore.crc.end();
    if end.record_count != expected.record_count || end.crc32c != expected.crc32c {
        return Err(Status::SecurityError);
    }
    let network = restore.network.clone().ok_or(Status::InvalidParameter)?;
    let key = restore
        .network_key
        .clone()
        .ok_or(Status::InvalidParameter)?;
    // 9.4 rule 4, with the 2^20 margin when the APS counter is unknown.
    let aps_margin = if restore.tclk_counter_known {
        restore.aps_margin
    } else {
        restore
            .aps_margin
            .max(zigbee_zrhp::records::APS_UNKNOWN_TCLK_MARGIN)
    };
    let nwk_fc_start = restore.counters.nwk_start(restore.nwk_margin);
    let aps_fc_start = restore.counters.aps_start(aps_margin);
    // Rule 7: the IEEE override first, so the network is stored under the
    // EUI-64 the radio will use.
    if let Some(node) = &restore.node
        && node.ieee.0 != port.ieee()
    {
        if !platform.ieee_rewritable() {
            return Err(Status::SecurityError);
        }
        platform
            .set_ieee_override(Some(node.ieee.0))
            .map_err(crate::firmware::status_of)?;
        port.set_ieee(node.ieee.0).await.map_err(op_status)?;
    }
    let image = NetworkImage {
        pan_id: network.pan_id,
        ext_pan_id: network.ext_pan_id,
        channel: network.channel,
        channel_mask: network.channel_mask,
        nwk_update_id: network.nwk_update_id,
        nwk_manager: network.nwk_manager,
        tx_power: (network.tx_power != 0x7F).then_some(network.tx_power),
        network_key: key.key,
        key_seq: key.seq,
        nwk_fc_start,
        aps_fc_start,
    };
    port.install_network(&image, &restore.children)
        .await
        .map_err(op_status)?;
    let mut on_ncp = 0u16;
    let mut host_backed = restore.host_backed;
    for (ieee, link_key) in &restore.keys {
        match port.put_link_key(*ieee, *link_key) {
            Ok(()) => on_ncp += 1,
            Err(_) => host_backed = host_backed.saturating_add(1),
        }
    }
    let counters = port.frame_counters();
    Ok(BkpRestoreCommitRsp {
        nwk_fc_start: counters.nwk_current.max(nwk_fc_start),
        aps_fc_start: counters.aps_current.max(aps_fc_start),
        link_keys_on_ncp: on_ncp,
        link_keys_host_backed: host_backed,
        children: restore.children.len() as u16,
    })
}

fn op_status(error: OpError) -> Status {
    match error {
        OpError::InvalidState => Status::InvalidState,
        OpError::NotFormed => Status::NotFormed,
        OpError::Storage => Status::StorageError,
        OpError::Unsupported => Status::UnsupportedCommand,
        OpError::InvalidParameter => Status::InvalidParameter,
        OpError::Zigbee(_) => Status::Failure,
    }
}
