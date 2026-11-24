//! TABLES group (`0x05xx`, endpoint TABLES), spec 8.5, standalone mode.
//!
//! The NCP keeps its Trust Center devices (unique link keys) and children
//! itself (`max_tc_devices` = 256 with `ncp-capacity`), which is the spec's
//! **standalone** mode (8.1): no lookups, no evictions. Implemented:
//!
//! * `TBL_READ` / `TBL_STATS` of the DEVICE table (record type `0x05`);
//! * `TBL_READ` / `TBL_STATS` of the APS group table (`APS_GROUP`, `0x0D`,
//!   capability `APS_GROUP_TABLE`), read-only: it is written with
//!   `NWK_GROUP_MEMBERSHIP`;
//! * `TBL_PUSH` of DEVICE records with a link block (unique TCLK provisioned
//!   before join), `TBL_INVALIDATE` (host deletion, I5);
//! * `TBL_SYNC`: epoch and counts.
//!
//! `TBL_READ` pages by entry index: `next_cursor` is the first entry not
//! sent. An entry added or removed between pages can shift the others by
//! one (a host that needs a consistent view reads again from cursor 0 after
//! its own changes); `table_epoch` changes when the network's tables are
//! wiped or replaced, both tables included.
//!
//! Not implemented (answered explicitly): paging (`TBL_LOOKUP` misses,
//! eviction writebacks), NCP-originated `TBL_WRITEBACK`, per-entry `rev`
//! (always 0) and dirty tracking, and the SOURCE_ROUTE / GP tables (GP
//! tables are read with `GP_*_TABLE_READ`). A host that sets `HOST_TABLES`
//! therefore never sees paging; a full table refuses new keyed joiners
//! (8.1 "full without host tables").

use heapless::Vec;
use zigbee_zrhp::codec::{CodecError, List, Writer};
use zigbee_zrhp::msg::tables::{
    PushResult, TableKey, TblInvalidate, TblPush, TblPushRsp, TblRead, TblStats, TblStatsRsp,
    TblSync, TblSyncRsp,
};
use zigbee_zrhp::msg::{Kind, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::records::{
    ApsGroupRecord, ChildInfo, DeviceRecord, LinkBlock, Record, device_flags, record_type,
};
use zigbee_zrhp::types::{Endpoint, Ieee, Status};

use crate::ncp::QUEUE_BYTES;
use crate::port::{CoordinatorPort, DeviceView, GroupView, NetworkState, OpError};

/// Most entries one `TBL_READ` response carries (DEVICE ≤ 46 B each,
/// APS_GROUP ≤ 19 B).
pub const MAX_READ_ENTRIES: usize = 8;

/// Whether `id` belongs to the TABLES group.
pub(crate) const fn is_tables_request(id: u16) -> bool {
    matches!(
        id,
        ids::TBL_LOOKUP
            | ids::TBL_WRITEBACK
            | ids::TBL_PUSH
            | ids::TBL_INVALIDATE
            | ids::TBL_SYNC
            | ids::TBL_READ
            | ids::TBL_STATS
    )
}

/// A DEVICE record from a [`DeviceView`]; the link key only when keys may
/// be exported. `tx_counter` is the global durable APS limit (9.3).
pub(crate) fn device_record(view: &DeviceView, keys: bool, aps_limit: u32) -> DeviceRecord {
    let mut flags = 0;
    if view.child.is_some() {
        flags |= device_flags::IS_CHILD;
    }
    if view.nwk.is_some() {
        flags |= device_flags::NWK_VALID;
    }
    DeviceRecord {
        ieee: Ieee(view.ieee),
        nwk: view.nwk.unwrap_or(0xFFFF),
        flags,
        child: view.child.map(|(capability, timeout_exp)| ChildInfo {
            capability,
            timeout_exp,
        }),
        link: view.link.filter(|_| keys).map(|link| LinkBlock {
            key: link.key,
            key_seq: 0,
            attrs: link.attrs,
            origin: link.origin,
            tx_counter: aps_limit,
            rx_floor: link.rx_floor,
        }),
    }
}

/// Number of known devices.
pub(crate) fn device_count<C: CoordinatorPort>(port: &C) -> usize {
    let mut count = 0;
    while port.device(count).is_some() {
        count += 1;
    }
    count
}

/// Handle one TABLES request.
// Own frame: its buffers stay out of the root future's poll frame.
#[inline(never)]
pub(crate) fn dispatch_tables<C: CoordinatorPort>(
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
        ids::TBL_READ => {
            let request = decode!(TblRead);
            let table = request.table;
            if table != record_type::DEVICE
                && !(table == record_type::APS_GROUP && port.group_capacity() != 0)
            {
                status(queues, Status::UnsupportedCommand);
                return;
            }
            if port.state() != NetworkState::Up {
                status(queues, Status::InvalidState);
                return;
            }
            if table == record_type::DEVICE {
                let aps_limit = port.frame_counters().aps_limit;
                read_page(
                    queues,
                    endpoint,
                    tid,
                    &request,
                    port,
                    |port, index| port.device(index).is_some(),
                    |port, index, w| {
                        let view = port.device(index)?;
                        let record =
                            Record::Device(device_record(&view, keys_exportable, aps_limit));
                        // Children are pinned (bit1).
                        Some(write_entry(w, u8::from(view.child.is_some()) << 1, &record))
                    },
                );
            } else {
                read_page(
                    queues,
                    endpoint,
                    tid,
                    &request,
                    port,
                    |port, index| port.group(index).is_some(),
                    |port, index, w| {
                        let view = port.group(index)?;
                        Some(write_entry(w, 0, &group_record(&view)))
                    },
                );
            }
        }
        ids::TBL_STATS => {
            let request = decode!(TblStats);
            let (capacity, used, pinned) = match request.table {
                record_type::DEVICE => {
                    let used = device_count(port);
                    let pinned = (0..used)
                        .filter(|&index| {
                            port.device(index).is_some_and(|view| view.child.is_some())
                        })
                        .count();
                    (port.device_capacity(), used, pinned)
                }
                record_type::APS_GROUP if port.group_capacity() != 0 => {
                    (port.group_capacity(), group_count(port), 0)
                }
                _ => {
                    status(queues, Status::UnsupportedCommand);
                    return;
                }
            };
            let response = TblStatsRsp {
                capacity: capacity.min(usize::from(u16::MAX)) as u16,
                used: used as u16,
                dirty: 0,
                pinned: pinned as u16,
                hits: 0,
                misses: 0,
                evictions: 0,
                lookup_timeouts: 0,
            };
            let _ = queues.response(tid, Status::Success, Some(&response));
        }
        ids::TBL_SYNC => {
            let _ = decode!(TblSync);
            let response = TblSyncRsp {
                ncp_table_epoch: port.table_epoch(),
                host_acked_rev: 0,
                dirty: 0,
                cached_devices: device_count(port) as u16,
                cached_routes: 0,
                paging_active: 0,
            };
            let _ = queues.response(tid, Status::Success, Some(&response));
        }
        ids::TBL_PUSH => {
            let request = decode!(TblPush);
            if request.table != record_type::DEVICE {
                status(queues, Status::UnsupportedCommand);
                return;
            }
            let mut results: Vec<PushResult, 16> = Vec::new();
            for record in request.records.iter() {
                let result = match record {
                    Record::Device(DeviceRecord {
                        ieee,
                        link: Some(link),
                        ..
                    }) => match port.put_link_key(ieee.0, link.key) {
                        Ok(()) => Status::Success,
                        Err(OpError::InvalidState) => Status::InvalidState,
                        Err(OpError::Unsupported) => Status::UnsupportedCommand,
                        Err(_) => Status::NoSpace,
                    },
                    // Identity only: the NCP learns addresses from traffic.
                    Record::Device(_) => Status::Success,
                    _ => Status::InvalidParameter,
                };
                if results
                    .push(PushResult {
                        status: result,
                        rev: 0,
                    })
                    .is_err()
                {
                    break;
                }
            }
            let response = TblPushRsp {
                results: List::new(&results),
            };
            let _ = queues.response(tid, Status::Success, Some(&response));
        }
        ids::TBL_INVALIDATE => {
            let request = decode!(TblInvalidate);
            if request.table != record_type::DEVICE {
                status(queues, Status::UnsupportedCommand);
                return;
            }
            let ieee = match request.key {
                TableKey::Ieee(ieee) => Some(ieee.0),
                TableKey::Nwk(nwk) => {
                    let mut found = None;
                    let mut index = 0;
                    while let Some(view) = port.device(index) {
                        if view.nwk == Some(nwk) {
                            found = Some(view.ieee);
                            break;
                        }
                        index += 1;
                    }
                    found
                }
            };
            // An unknown device is not an error (8.5).
            let result = match ieee {
                Some(ieee) => match port.forget_device(ieee) {
                    Ok(_) => Status::Success,
                    Err(OpError::InvalidState) => Status::InvalidState,
                    Err(_) => Status::StorageError,
                },
                None => Status::Success,
            };
            status(queues, result);
        }
        // NCP-originated in v1 (paging); not served by this NCP.
        _ => status(queues, Status::UnsupportedCommand),
    }
}

/// One `TBL_READ` page from `request.cursor` (the entry index): at most
/// `request.max` (1..=[`MAX_READ_ENTRIES`]) entries written by `entry`
/// (`None` past the end, `Err` when the entry does not fit the page), and
/// `next_cursor` the first entry not sent (`FFFF` once `exists` says the
/// table ends there). Every entry has `rev` 0 (standalone, 8.1).
// Own frame per table: one page buffer on the stack at a time.
#[inline(never)]
fn read_page<C: CoordinatorPort>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    endpoint: Endpoint,
    tid: u16,
    request: &TblRead,
    port: &C,
    exists: impl Fn(&C, usize) -> bool,
    entry: impl Fn(&C, usize, &mut Writer<'_>) -> Option<Result<(), CodecError>>,
) {
    let max = usize::from(request.max).clamp(1, MAX_READ_ENTRIES);
    // status, u16 next_cursor, u8 n, n × {u32 rev, u8 state, record}
    let mut payload = [0u8; 4 + MAX_READ_ENTRIES * 48];
    payload[0] = Status::Success.to_u8();
    let mut w = Writer::new(&mut payload[4..]);
    let mut index = usize::from(request.cursor);
    let mut n = 0u8;
    while usize::from(n) < max {
        let mark = w.len();
        match entry(port, index, &mut w) {
            Some(Ok(())) => {}
            Some(Err(_)) => {
                w.truncate(mark);
                break;
            }
            None => break,
        }
        n += 1;
        index += 1;
    }
    let written = w.len();
    let next = if exists(port, index) {
        index as u16
    } else {
        0xFFFF
    };
    payload[1..3].copy_from_slice(&next.to_le_bytes());
    payload[3] = n;
    let _ = queues
        .queue(endpoint)
        .raw(Kind::Response, ids::TBL_READ, tid, &payload[..4 + written]);
}

/// An `APS_GROUP` record of `view` (table `0x0D` and the backup).
pub(crate) fn group_record(view: &GroupView) -> Record<'_> {
    Record::ApsGroup(ApsGroupRecord {
        group: view.group,
        endpoints: &view.endpoints,
    })
}

/// Number of groups in the APS group table.
pub(crate) fn group_count<C: CoordinatorPort>(port: &C) -> usize {
    let mut count = 0;
    while port.group(count).is_some() {
        count += 1;
    }
    count
}

/// `u32 rev, u8 state, record` of one `TBL_READ` entry.
fn write_entry(w: &mut Writer<'_>, state: u8, record: &Record<'_>) -> Result<(), CodecError> {
    use zigbee_zrhp::codec::Wire;
    w.u32(0)?;
    w.u8(state)?;
    record.write(w)
}
