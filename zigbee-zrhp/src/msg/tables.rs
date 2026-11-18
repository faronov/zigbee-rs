//! TABLES group (`0x05xx`, endpoint TABLES), spec 8.5.
//!
//! Table IDs equal record types: `0x05` DEVICE, `0x08` SOURCE_ROUTE, `0x09`
//! GP_PROXY, `0x0B` GP_SINK.

use crate::codec::{CodecError, List, Reader, Wire, Writer, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::records::Record;
use crate::types::{Ieee, Status};

/// Lookup key: `u8 key_type (1 IEEE, 2 NWK), key[8|2]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TableKey {
    /// By IEEE.
    Ieee(Ieee),
    /// By short address.
    Nwk(u16),
}

impl<'a> Wire<'a> for TableKey {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        Ok(match r.u8()? {
            1 => Self::Ieee(r.read()?),
            2 => Self::Nwk(r.u16()?),
            _ => return Err(CodecError::Invalid),
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        match self {
            Self::Ieee(i) => {
                w.u8(1)?;
                w.write(i)
            }
            Self::Nwk(n) => {
                w.u8(2)?;
                w.u16(*n)
            }
        }
    }
}

wire_struct! {
    /// `TBL_LOOKUP` request (NCP → host).
    pub struct TblLookup {
        /// Table ID.
        pub table: u8,
        /// Key.
        pub key: TableKey,
    }
}

wire_struct! {
    /// `TBL_LOOKUP` response body (host → NCP).
    pub struct TblLookupRsp<'a> {
        /// Host revision of the entry.
        pub rev: u32,
        /// Record.
        pub record: Record<'a>,
    }
}

/// `TBL_WRITEBACK.reason`.
pub mod writeback_reason {
    /// New entry.
    pub const NEW: u8 = 1;
    /// Updated entry.
    pub const UPDATE: u8 = 2;
    /// Evicted entry.
    pub const EVICT: u8 = 3;
    /// Deleted entry.
    pub const DELETE: u8 = 4;
}

wire_struct! {
    /// `TBL_WRITEBACK` request (NCP → host). The host answers `SUCCESS`
    /// only after a durable commit.
    pub struct TblWriteback<'a> {
        /// Table ID.
        pub table: u8,
        /// [`writeback_reason`].
        pub reason: u8,
        /// NCP revision.
        pub rev: u32,
        /// Record.
        pub record: Record<'a>,
    }
}

wire_struct! {
    /// `TBL_PUSH` request.
    pub struct TblPush<'a> {
        /// Table ID.
        pub table: u8,
        /// Bit0 pin, bit1 force, bit2 only if free space.
        pub flags: u8,
        /// Records.
        pub records: List<'a, Record<'a>>,
    }
}

wire_struct! {
    /// `{u8 status, u32 rev}` push result.
    pub struct PushResult {
        /// Per-record status.
        pub status: Status,
        /// Resulting revision.
        pub rev: u32,
    }
}

wire_struct! {
    /// `TBL_PUSH` response body.
    pub struct TblPushRsp<'a> {
        /// Per-record results.
        pub results: List<'a, PushResult>,
    }
}

wire_struct! {
    /// `TBL_INVALIDATE` request.
    pub struct TblInvalidate {
        /// Table ID.
        pub table: u8,
        /// Key.
        pub key: TableKey,
    }
}

wire_struct! {
    /// `TBL_SYNC` request.
    pub struct TblSync {
        /// Host DB epoch.
        pub host_db_epoch: u32,
        /// Last NCP table epoch the host knows.
        pub last_ncp_table_epoch: u32,
        /// Highest revision in the host store.
        pub host_max_rev: u32,
        /// Bit0 host store rolled back.
        pub flags: u8,
    }
}

wire_struct! {
    /// `TBL_SYNC` response body.
    pub struct TblSyncRsp {
        /// NCP table epoch.
        pub ncp_table_epoch: u32,
        /// Highest revision the host ever acknowledged.
        pub host_acked_rev: u32,
        /// Dirty entries.
        pub dirty: u16,
        /// Cached devices.
        pub cached_devices: u16,
        /// Cached routes.
        pub cached_routes: u16,
        /// Paging active.
        pub paging_active: u8,
    }
}

wire_struct! {
    /// `TBL_READ` request.
    pub struct TblRead {
        /// Table ID.
        pub table: u8,
        /// Cursor.
        pub cursor: u16,
        /// Maximum entries.
        pub max: u8,
    }
}

wire_struct! {
    /// `{u32 rev, u8 state, record}` read entry.
    pub struct TblReadEntry<'a> {
        /// Revision.
        pub rev: u32,
        /// Bit0 dirty, bit1 pinned.
        pub state: u8,
        /// Record.
        pub record: Record<'a>,
    }
}

wire_struct! {
    /// `TBL_READ` response body.
    pub struct TblReadRsp<'a> {
        /// Next cursor (`FFFF` = end).
        pub next_cursor: u16,
        /// Entries.
        pub entries: List<'a, TblReadEntry<'a>>,
    }
}

wire_struct! {
    /// `TBL_STATS` request.
    pub struct TblStats {
        /// Table ID.
        pub table: u8,
    }
}

wire_struct! {
    /// `TBL_STATS` response body.
    pub struct TblStatsRsp {
        /// Capacity.
        pub capacity: u16,
        /// Used.
        pub used: u16,
        /// Dirty.
        pub dirty: u16,
        /// Pinned.
        pub pinned: u16,
        /// Hits.
        pub hits: u32,
        /// Misses.
        pub misses: u32,
        /// Evictions.
        pub evictions: u32,
        /// Lookup timeouts.
        pub lookup_timeouts: u32,
    }
}

wire_struct! {
    /// `EVT_TBL_EPOCH_CHANGED`.
    pub struct EvtTblEpochChanged {
        /// New epoch.
        pub new_epoch: u32,
        /// 1 factory reset, 2 restore, 3 NV re-initialised.
        pub reason: u8,
    }
}

wire_struct! {
    /// `EVT_TBL_PAGING`.
    pub struct EvtTblPaging {
        /// Paging active.
        pub active: u8,
        /// Cached entries.
        pub cached: u16,
        /// Capacity.
        pub capacity: u16,
    }
}

message_def! {
    TblLookup = ids::TBL_LOOKUP, Request;
    TblLookupRsp<'a> = ids::TBL_LOOKUP, Response;
    TblWriteback<'a> = ids::TBL_WRITEBACK, Request;
    TblPush<'a> = ids::TBL_PUSH, Request;
    TblPushRsp<'a> = ids::TBL_PUSH, Response;
    TblInvalidate = ids::TBL_INVALIDATE, Request;
    TblSync = ids::TBL_SYNC, Request;
    TblSyncRsp = ids::TBL_SYNC, Response;
    TblRead = ids::TBL_READ, Request;
    TblReadRsp<'a> = ids::TBL_READ, Response;
    TblStats = ids::TBL_STATS, Request;
    TblStatsRsp = ids::TBL_STATS, Response;
    EvtTblEpochChanged = ids::EVT_TBL_EPOCH_CHANGED, Event;
    EvtTblPaging = ids::EVT_TBL_PAGING, Event;
}
