//! TABLES group (`0x05xx`, endpoint TABLES), spec 8.5, standalone mode.
//!
//! The NCP keeps its Trust Center devices (unique link keys) and children
//! itself (`max_tc_devices` = 256 with `ncp-capacity`), which is the spec's
//! **standalone** mode (8.1): no lookups, no evictions. Implemented:
//!
//! * `TBL_READ` / `TBL_STATS` of the DEVICE table (record type `0x05`);
//! * `TBL_PUSH` of DEVICE records with a link block (unique TCLK provisioned
//!   before join), `TBL_INVALIDATE` (host deletion, I5);
//! * `TBL_SYNC`: epoch and counts.
//!
//! Not implemented (answered explicitly): paging (`TBL_LOOKUP` misses,
//! eviction writebacks), NCP-originated `TBL_WRITEBACK`, per-entry `rev`
//! (always 0) and dirty tracking, and the SOURCE_ROUTE / GP tables (GP
//! tables are read with `GP_*_TABLE_READ`). A host that sets `HOST_TABLES`
//! therefore never sees paging; a full table refuses new keyed joiners
//! (8.1 "full without host tables").

use heapless::Vec;
use zigbee_zrhp::codec::{List, Writer};
use zigbee_zrhp::msg::tables::{
    PushResult, TableKey, TblInvalidate, TblPush, TblPushRsp, TblRead, TblStats, TblStatsRsp,
    TblSync, TblSyncRsp,
};
use zigbee_zrhp::msg::{Kind, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::records::{ChildInfo, DeviceRecord, LinkBlock, Record, device_flags, record_type};
use zigbee_zrhp::types::{Endpoint, Ieee, Status};

use crate::ncp::QUEUE_BYTES;
use crate::port::{CoordinatorPort, DeviceView, NetworkState, OpError};

/// Most DEVICE entries one `TBL_READ` response carries (≤ 46 B each).
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
            if request.table != record_type::DEVICE {
                status(queues, Status::UnsupportedCommand);
                return;
            }
            if port.state() != NetworkState::Up {
                status(queues, Status::InvalidState);
                return;
            }
            let aps_limit = port.frame_counters().aps_limit;
            let max = usize::from(request.max).clamp(1, MAX_READ_ENTRIES);
            // status, u16 next_cursor, u8 n, n × {u32 rev, u8 state, record}
            let mut payload = [0u8; 4 + MAX_READ_ENTRIES * 48];
            payload[0] = Status::Success.to_u8();
            let mut w = Writer::new(&mut payload[4..]);
            let mut index = usize::from(request.cursor);
            let mut n = 0u8;
            while usize::from(n) < max {
                let Some(view) = port.device(index) else {
                    break;
                };
                let record = Record::Device(device_record(&view, keys_exportable, aps_limit));
                let state = u8::from(view.child.is_some()) << 1; // pinned
                let mark = w.len();
                if write_entry(&mut w, state, &record).is_err() {
                    w.truncate(mark);
                    break;
                }
                n += 1;
                index += 1;
            }
            let written = w.len();
            let next = if port.device(index).is_some() {
                index as u16
            } else {
                0xFFFF
            };
            payload[1..3].copy_from_slice(&next.to_le_bytes());
            payload[3] = n;
            let _ = queues
                .queue(endpoint)
                .raw(Kind::Response, id, tid, &payload[..4 + written]);
        }
        ids::TBL_STATS => {
            let request = decode!(TblStats);
            if request.table != record_type::DEVICE {
                status(queues, Status::UnsupportedCommand);
                return;
            }
            let used = device_count(port);
            let pinned = (0..used)
                .filter(|&index| port.device(index).is_some_and(|view| view.child.is_some()))
                .count();
            let response = TblStatsRsp {
                capacity: port.device_capacity().min(usize::from(u16::MAX)) as u16,
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

/// `u32 rev, u8 state, record` of one `TBL_READ` entry.
fn write_entry(
    w: &mut Writer<'_>,
    state: u8,
    record: &Record<'_>,
) -> Result<(), zigbee_zrhp::codec::CodecError> {
    use zigbee_zrhp::codec::Wire;
    w.u32(0)?;
    w.u8(state)?;
    record.write(w)
}
