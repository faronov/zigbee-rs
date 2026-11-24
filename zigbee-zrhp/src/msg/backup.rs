//! BACKUP group (`0x06xx`, endpoint BULK), spec 9.4–9.5.

use crate::codec::{List, ToEnd, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::records::Record;

/// A record stream running to the end of the payload.
pub type RecordStream<'a> = List<'a, Record<'a>, ToEnd>;

/// `BKP_RESTORE_PUT` note codes.
pub mod note {
    /// Key not cached on the NCP; the host must hold it.
    pub const HOST_BACKED: u8 = 1;
    /// Child table full; kept as a non-child.
    pub const CHILD_DEMOTED: u8 = 2;
    /// Record ignored: unknown, or not held by this NCP (for example an
    /// `APS_GROUP` beyond its group table).
    pub const IGNORED: u8 = 3;
    /// A frame-counter floor was raised.
    pub const FLOOR_RAISED: u8 = 4;
}

/// `BKP_EXPORT_BEGIN` flags.
pub mod export_flags {
    /// Network and link keys (when key export is unlocked).
    pub const KEYS: u8 = 0x01;
    /// `DEVICE` records.
    pub const DEVICES: u8 = 0x02;
    /// GP tables.
    pub const GP_TABLES: u8 = 0x04;
    /// `APS_GROUP` records: the coordinator's APS group table
    /// (`APS_GROUP_TABLE` capability).
    pub const APS_GROUPS: u8 = 0x08;
}

wire_struct! {
    /// `BKP_EXPORT_BEGIN` request.
    pub struct BkpExportBegin {
        /// [`export_flags`]: bit0 keys, bit1 devices, bit2 GP tables, bit3
        /// APS groups.
        pub flags: u8,
    }
}

wire_struct! {
    /// `BKP_EXPORT_BEGIN` response body.
    pub struct BkpExportBeginRsp {
        /// Transaction.
        pub txn: u16,
        /// Records in the snapshot.
        pub record_count: u32,
    }
}

wire_struct! {
    /// `BKP_EXPORT_READ` request.
    pub struct BkpExportRead {
        /// Transaction.
        pub txn: u16,
        /// Maximum bytes of records.
        pub max_bytes: u16,
    }
}

wire_struct! {
    /// `BKP_EXPORT_READ` response body.
    pub struct BkpExportReadRsp<'a> {
        /// More data follows.
        pub more: u8,
        /// Records (the stream ends with `END`).
        pub records: RecordStream<'a>,
    }
}

wire_struct! {
    /// `BKP_EXPORT_END` request.
    pub struct BkpExportEnd {
        /// Transaction.
        pub txn: u16,
    }
}

wire_struct! {
    /// `BKP_RESTORE_COMMIT` request.
    pub struct BkpRestoreCommit {
        /// Transaction.
        pub txn: u16,
    }
}

wire_struct! {
    /// `BKP_RESTORE_ABORT` request.
    pub struct BkpRestoreAbort {
        /// Transaction.
        pub txn: u16,
    }
}

wire_struct! {
    /// `BKP_RESTORE_BEGIN` request.
    pub struct BkpRestoreBegin {
        /// Bit0 allow IEEE change, bit1 start network after commit.
        pub flags: u8,
        /// NWK margin.
        pub nwk_margin: u32,
        /// APS margin.
        pub aps_margin: u32,
        /// Expected records.
        pub expected_records: u32,
    }
}

wire_struct! {
    /// `BKP_RESTORE_BEGIN` response body.
    pub struct BkpRestoreBeginRsp {
        /// Transaction.
        pub txn: u16,
        /// Maximum records per PUT.
        pub max_records_per_put: u16,
    }
}

wire_struct! {
    /// `BKP_RESTORE_PUT` request.
    pub struct BkpRestorePut<'a> {
        /// Transaction.
        pub txn: u16,
        /// Records (the last PUT ends with `END`).
        pub records: RecordStream<'a>,
    }
}

wire_struct! {
    /// `{u16 index, u8 note}`.
    pub struct RestoreNote {
        /// Record index in the stream.
        pub index: u16,
        /// [`note`] code.
        pub note: u8,
    }
}

wire_struct! {
    /// `BKP_RESTORE_PUT` response body.
    pub struct BkpRestorePutRsp<'a> {
        /// Records accepted so far.
        pub accepted: u16,
        /// Notes.
        pub notes: List<'a, RestoreNote>,
    }
}

wire_struct! {
    /// `BKP_RESTORE_COMMIT` response body.
    pub struct BkpRestoreCommitRsp {
        /// NWK start.
        pub nwk_fc_start: u32,
        /// APS start.
        pub aps_fc_start: u32,
        /// Link keys cached on the NCP.
        pub link_keys_on_ncp: u16,
        /// Link keys only on the host.
        pub link_keys_host_backed: u16,
        /// Children restored.
        pub children: u16,
    }
}

message_def! {
    BkpExportBegin = ids::BKP_EXPORT_BEGIN, Request;
    BkpExportBeginRsp = ids::BKP_EXPORT_BEGIN, Response;
    BkpExportRead = ids::BKP_EXPORT_READ, Request;
    BkpExportReadRsp<'a> = ids::BKP_EXPORT_READ, Response;
    BkpRestoreBegin = ids::BKP_RESTORE_BEGIN, Request;
    BkpRestoreBeginRsp = ids::BKP_RESTORE_BEGIN, Response;
    BkpRestorePut<'a> = ids::BKP_RESTORE_PUT, Request;
    BkpRestorePutRsp<'a> = ids::BKP_RESTORE_PUT, Response;
    BkpRestoreCommitRsp = ids::BKP_RESTORE_COMMIT, Response;
    BkpExportEnd = ids::BKP_EXPORT_END, Request;
    BkpRestoreCommit = ids::BKP_RESTORE_COMMIT, Request;
    BkpRestoreAbort = ids::BKP_RESTORE_ABORT, Request;
}
