//! Backup and table records (spec 9.3), shared by BACKUP, TABLES and GP
//! commands.
//!
//! Every record is a TLV: `u8 type, u16 len, value[len]`. [`Record`] decodes
//! the value zero-copy; unknown types are kept as [`Record::Unknown`] so hosts
//! can preserve them. Known record values may grow by appending fields in a
//! later minor version: extra value bytes are ignored on decode.
//!
//! A record stream (backup export/restore) ends with an [`EndRecord`] whose
//! CRC-32C covers the concatenated TLV bytes of all preceding records;
//! [`StreamCrc`] computes it incrementally and [`verify_stream`] checks it.
//! [`RestoreCounters`] implements the frame-counter start rule of
//! `BKP_RESTORE_COMMIT` (9.4).

use crate::codec::{CodecError, List, Reader, Wire, Writer, wire_struct};
use crate::crc::Crc32c;
use crate::types::{Ieee, Key};

/// Record type codes.
pub mod record_type {
    /// `NODE`
    pub const NODE: u8 = 0x01;
    /// `NETWORK`
    pub const NETWORK: u8 = 0x02;
    /// `NETWORK_KEY`
    pub const NETWORK_KEY: u8 = 0x03;
    /// `TC_LINK_KEY`
    pub const TC_LINK_KEY: u8 = 0x04;
    /// `DEVICE` (also the TABLES table ID)
    pub const DEVICE: u8 = 0x05;
    /// `EZSP_HASHED_TCLK`
    pub const EZSP_HASHED_TCLK: u8 = 0x06;
    /// `SOURCE_ROUTE` (also a table ID)
    pub const SOURCE_ROUTE: u8 = 0x08;
    /// `GP_PROXY` (also a table ID)
    pub const GP_PROXY: u8 = 0x09;
    /// `RS_PARAMS`
    pub const RS_PARAMS: u8 = 0x0A;
    /// `GP_SINK` (also a table ID)
    pub const GP_SINK: u8 = 0x0B;
    /// `GP_TRANSLATION`
    pub const GP_TRANSLATION: u8 = 0x0C;
    /// `END`
    pub const END: u8 = 0xFF;
}

/// Green Power device identifier: `u8 app_id`, then `u32 src_id` (app 0)
/// or `[8] ieee, u8 endpoint` (app 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpdId {
    /// Application ID 0.
    SrcId(u32),
    /// Application ID 2.
    Ieee {
        /// GPD IEEE.
        ieee: Ieee,
        /// GPD endpoint.
        endpoint: u8,
    },
}

impl<'a> Wire<'a> for GpdId {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        Ok(match r.u8()? {
            0 => Self::SrcId(r.u32()?),
            2 => Self::Ieee {
                ieee: r.read()?,
                endpoint: r.u8()?,
            },
            _ => return Err(CodecError::Invalid),
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        match self {
            Self::SrcId(id) => {
                w.u8(0)?;
                w.u32(*id)
            }
            Self::Ieee { ieee, endpoint } => {
                w.u8(2)?;
                w.write(ieee)?;
                w.u8(*endpoint)
            }
        }
    }
}

wire_struct! {
    /// `NODE` (`0x01`).
    pub struct NodeRecord {
        /// Coordinator IEEE.
        pub ieee: Ieee,
        /// Short address (must be `0x0000`).
        pub nwk: u16,
        /// 0 coordinator.
        pub logical_type: u8,
        /// Bit0 IEEE is/should be an override.
        pub flags: u8,
    }
}

wire_struct! {
    /// `NETWORK` (`0x02`).
    pub struct NetworkRecord {
        /// PAN ID.
        pub pan_id: u16,
        /// Extended PAN ID (on-air order).
        pub ext_pan_id: [u8; 8],
        /// Channel.
        pub channel: u8,
        /// Channel mask (bit n = channel n).
        pub channel_mask: u32,
        /// NWK update ID.
        pub nwk_update_id: u8,
        /// Network manager.
        pub nwk_manager: u16,
        /// Security level (5).
        pub security_level: u8,
        /// TX power (`0x7F` unknown).
        pub tx_power: i8,
    }
}

wire_struct! {
    /// `NETWORK_KEY` (`0x03`).
    pub struct NetworkKeyRecord {
        /// Key.
        pub key: Key,
        /// Sequence.
        pub seq: u8,
        /// Outgoing counter (durable limit on export).
        pub tx_counter: u32,
        /// Bit0 active.
        pub flags: u8,
    }
}

wire_struct! {
    /// `TC_LINK_KEY` (`0x04`).
    pub struct TcLinkKeyRecord {
        /// Global TCLK.
        pub key: Key,
        /// Trust Center IEEE.
        pub tc_ieee: Ieee,
        /// Global APS counter (durable limit on export).
        pub tx_counter: u32,
        /// Bit0 EZSP hashed mode active.
        pub flags: u8,
    }
}

wire_struct! {
    /// `DEVICE` child info: `u8 capability, u8 timeout_exp`.
    pub struct ChildInfo {
        /// MAC capability.
        pub capability: u8,
        /// Timeout index.
        pub timeout_exp: u8,
    }
}

wire_struct! {
    /// `DEVICE` link block.
    pub struct LinkBlock {
        /// Link key.
        pub key: Key,
        /// Key sequence.
        pub key_seq: u8,
        /// 0 provisional, 1 unverified, 2 verified.
        pub attrs: u8,
        /// 0 global, 1 install code, 2 TC generated, 3 imported/host, 4 EZSP-hashed.
        pub origin: u8,
        /// Informational: the global APS limit (8.4).
        pub tx_counter: u32,
        /// Incoming frame-counter floor.
        pub rx_floor: u32,
    }
}

/// `DEVICE.flags` bits.
pub mod device_flags {
    /// `is_child`
    pub const IS_CHILD: u8 = 0x01;
    /// Link block present.
    pub const LINK: u8 = 0x02;
    /// Child info present.
    pub const CHILD_INFO: u8 = 0x04;
    /// `nwk` valid.
    pub const NWK_VALID: u8 = 0x08;
}

/// `DEVICE` (`0x05`). Flag bits 1 (link) and 2 (child info) are derived
/// from `link` / `child` when encoding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceRecord {
    /// IEEE.
    pub ieee: Ieee,
    /// Short address (`FFFF` unknown).
    pub nwk: u16,
    /// [`device_flags`].
    pub flags: u8,
    /// Child info.
    pub child: Option<ChildInfo>,
    /// Link block.
    pub link: Option<LinkBlock>,
}

impl<'a> Wire<'a> for DeviceRecord {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let ieee = r.read()?;
        let nwk = r.u16()?;
        let flags = r.u8()?;
        let child = if flags & device_flags::CHILD_INFO != 0 {
            Some(r.read()?)
        } else {
            None
        };
        let link = if flags & device_flags::LINK != 0 {
            Some(r.read()?)
        } else {
            None
        };
        Ok(Self {
            ieee,
            nwk,
            flags,
            child,
            link,
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        let mut flags = self.flags & !(device_flags::LINK | device_flags::CHILD_INFO);
        if self.child.is_some() {
            flags |= device_flags::CHILD_INFO;
        }
        if self.link.is_some() {
            flags |= device_flags::LINK;
        }
        w.write(&self.ieee)?;
        w.u16(self.nwk)?;
        w.u8(flags)?;
        if let Some(c) = &self.child {
            w.write(c)?;
        }
        if let Some(l) = &self.link {
            w.write(l)?;
        }
        Ok(())
    }
}

wire_struct! {
    /// `SOURCE_ROUTE` (`0x08`), relays in path order from the coordinator.
    pub struct SourceRouteRecord<'a> {
        /// Destination.
        pub dst: u16,
        /// Relays.
        pub relays: List<'a, u16>,
    }
}

/// `GP_PROXY` (`0x09`): `u8 format_version, entry[]` in the zigbee-rs
/// `gp_store` entry encoding (opaque here).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpProxyRecord<'a> {
    /// Entry encoding version.
    pub format_version: u8,
    /// Encoded entries.
    pub entries: &'a [u8],
}

impl<'a> Wire<'a> for GpProxyRecord<'a> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        Ok(Self {
            format_version: r.u8()?,
            entries: r.rest(),
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.u8(self.format_version)?;
        w.bytes(self.entries)
    }
}

wire_struct! {
    /// `RS_PARAMS` (`0x0A`).
    pub struct RsParamsRecord {
        /// Reservation stride.
        pub fc_stride: u32,
        /// Table epoch.
        pub table_epoch: u32,
        /// NWK durable limit.
        pub nwk_fc_limit: u32,
        /// APS durable limit.
        pub aps_fc_limit: u32,
        /// Bit0 host-backed tables.
        pub flags: u8,
    }
}

/// `GP_SINK.flags` bits.
pub mod gp_sink_flags {
    /// MAC sequence-number capable.
    pub const MAC_SEQ_CAPABLE: u8 = 0x01;
    /// RX-on capable.
    pub const RX_ON_CAPABLE: u8 = 0x02;
    /// Fixed location.
    pub const FIXED_LOCATION: u8 = 0x04;
    /// Key present.
    pub const KEY_PRESENT: u8 = 0x08;
}

/// `GP_SINK` (`0x0B`). Flag bit 3 (key present) is derived from `key` when
/// encoding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpSinkRecord {
    /// GPD identifier.
    pub gpd_id: GpdId,
    /// GPD device ID.
    pub device_id: u8,
    /// Communication mode.
    pub communication_mode: u8,
    /// Group (`FFFF` none).
    pub group: u16,
    /// Assigned alias (`FFFF` derived).
    pub assigned_alias: u16,
    /// [`gp_sink_flags`].
    pub flags: u8,
    /// Security level.
    pub security_level: u8,
    /// Key type.
    pub key_type: u8,
    /// Key (present iff flags bit3).
    pub key: Option<Key>,
    /// GPD frame counter.
    pub frame_counter: u32,
    /// Groupcast radius.
    pub groupcast_radius: u8,
}

impl<'a> Wire<'a> for GpSinkRecord {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let gpd_id = r.read()?;
        let device_id = r.u8()?;
        let communication_mode = r.u8()?;
        let group = r.u16()?;
        let assigned_alias = r.u16()?;
        let flags = r.u8()?;
        let security_level = r.u8()?;
        let key_type = r.u8()?;
        let key = if flags & gp_sink_flags::KEY_PRESENT != 0 {
            Some(r.array()?)
        } else {
            None
        };
        let frame_counter = r.u32()?;
        let groupcast_radius = r.u8()?;
        Ok(Self {
            gpd_id,
            device_id,
            communication_mode,
            group,
            assigned_alias,
            flags,
            security_level,
            key_type,
            key,
            frame_counter,
            groupcast_radius,
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        let mut flags = self.flags & !gp_sink_flags::KEY_PRESENT;
        if self.key.is_some() {
            flags |= gp_sink_flags::KEY_PRESENT;
        }
        w.write(&self.gpd_id)?;
        w.u8(self.device_id)?;
        w.u8(self.communication_mode)?;
        w.u16(self.group)?;
        w.u16(self.assigned_alias)?;
        w.u8(flags)?;
        w.u8(self.security_level)?;
        w.u8(self.key_type)?;
        if let Some(k) = &self.key {
            w.bytes(k)?;
        }
        w.u32(self.frame_counter)?;
        w.u8(self.groupcast_radius)
    }
}

/// `GP_TRANSLATION.payload_len` values that carry no payload bytes.
pub mod gp_translation_payload {
    /// Derived from the GPD command as by the sink's default translation.
    pub const DERIVED: u8 = 0xFE;
    /// The GPD command payload, copied.
    pub const COPY_GPD: u8 = 0xFF;
}

/// One GPD command → ZCL command translation: `u8 gpd_command, u8
/// endpoint, u16 profile, u16 cluster, u8 zcl_command, u8 payload_len`,
/// then `payload_len` payload bytes when it is `0x00..=0xFD`
/// ([`gp_translation_payload`] for `0xFE` and `0xFF`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpTranslation<'a> {
    /// GPD CommandID.
    pub gpd_command: u8,
    /// Endpoint on the sink (`0xFC`: hand the command to the application).
    pub endpoint: u8,
    /// Zigbee profile.
    pub profile: u16,
    /// ZCL cluster.
    pub cluster: u16,
    /// ZCL cluster-specific command.
    pub zcl_command: u8,
    /// Payload length or [`gp_translation_payload`].
    pub payload_len: u8,
    /// Fixed payload (empty for `0xFE`/`0xFF`).
    pub payload: &'a [u8],
}

impl<'a> Wire<'a> for GpTranslation<'a> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let gpd_command = r.u8()?;
        let endpoint = r.u8()?;
        let profile = r.u16()?;
        let cluster = r.u16()?;
        let zcl_command = r.u8()?;
        let payload_len = r.u8()?;
        let payload = if payload_len < gp_translation_payload::DERIVED {
            r.bytes(usize::from(payload_len))?
        } else {
            &[]
        };
        Ok(Self {
            gpd_command,
            endpoint,
            profile,
            cluster,
            zcl_command,
            payload_len,
            payload,
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        let fixed = self.payload_len < gp_translation_payload::DERIVED;
        if (fixed && usize::from(self.payload_len) != self.payload.len())
            || (!fixed && !self.payload.is_empty())
        {
            return Err(CodecError::Invalid);
        }
        w.u8(self.gpd_command)?;
        w.u8(self.endpoint)?;
        w.u16(self.profile)?;
        w.u16(self.cluster)?;
        w.u8(self.zcl_command)?;
        w.u8(self.payload_len)?;
        w.bytes(self.payload)
    }
}

wire_struct! {
    /// `GP_TRANSLATION` (`0x0C`): one Translation Table entry of a sink.
    pub struct GpTranslationRecord<'a> {
        /// GPD identifier.
        pub gpd_id: GpdId,
        /// The translation.
        pub translation: GpTranslation<'a>,
    }
}

wire_struct! {
    /// `END` (`0xFF`).
    pub struct EndRecord {
        /// Number of preceding records.
        pub record_count: u32,
        /// CRC-32C over their concatenated TLV bytes.
        pub crc32c: u32,
    }
}

/// One record (TLV).
#[derive(Debug, Clone, PartialEq)]
pub enum Record<'a> {
    /// `0x01`
    Node(NodeRecord),
    /// `0x02`
    Network(NetworkRecord),
    /// `0x03`
    NetworkKey(NetworkKeyRecord),
    /// `0x04`
    TcLinkKey(TcLinkKeyRecord),
    /// `0x05`
    Device(DeviceRecord),
    /// `0x06` seed
    EzspHashedTclk(Key),
    /// `0x08`
    SourceRoute(SourceRouteRecord<'a>),
    /// `0x09`
    GpProxy(GpProxyRecord<'a>),
    /// `0x0A`
    RsParams(RsParamsRecord),
    /// `0x0B`
    GpSink(GpSinkRecord),
    /// `0x0C`
    GpTranslation(GpTranslationRecord<'a>),
    /// `0xFF`
    End(EndRecord),
    /// Any other type (skipped by the NCP, preserved by hosts).
    Unknown {
        /// Type.
        rtype: u8,
        /// Value.
        value: &'a [u8],
    },
}

impl Record<'_> {
    /// The type code.
    pub const fn rtype(&self) -> u8 {
        use record_type::*;
        match self {
            Self::Node(_) => NODE,
            Self::Network(_) => NETWORK,
            Self::NetworkKey(_) => NETWORK_KEY,
            Self::TcLinkKey(_) => TC_LINK_KEY,
            Self::Device(_) => DEVICE,
            Self::EzspHashedTclk(_) => EZSP_HASHED_TCLK,
            Self::SourceRoute(_) => SOURCE_ROUTE,
            Self::GpProxy(_) => GP_PROXY,
            Self::RsParams(_) => RS_PARAMS,
            Self::GpSink(_) => GP_SINK,
            Self::GpTranslation(_) => GP_TRANSLATION,
            Self::End(_) => END,
            Self::Unknown { rtype, .. } => *rtype,
        }
    }
}

impl<'a> Wire<'a> for Record<'a> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        use record_type::*;
        let rtype = r.u8()?;
        let len = r.u16()? as usize;
        let value = r.bytes(len)?;
        let mut v = Reader::new(value);
        Ok(match rtype {
            NODE => Self::Node(v.read()?),
            NETWORK => Self::Network(v.read()?),
            NETWORK_KEY => Self::NetworkKey(v.read()?),
            TC_LINK_KEY => Self::TcLinkKey(v.read()?),
            DEVICE => Self::Device(v.read()?),
            EZSP_HASHED_TCLK => Self::EzspHashedTclk(v.read()?),
            SOURCE_ROUTE => Self::SourceRoute(v.read()?),
            GP_PROXY => Self::GpProxy(v.read()?),
            RS_PARAMS => Self::RsParams(v.read()?),
            GP_SINK => Self::GpSink(v.read()?),
            GP_TRANSLATION => Self::GpTranslation(v.read()?),
            END => Self::End(v.read()?),
            _ => Self::Unknown { rtype, value },
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.u8(self.rtype())?;
        let at = w.reserve(2)?;
        let start = w.len();
        match self {
            Self::Node(v) => w.write(v)?,
            Self::Network(v) => w.write(v)?,
            Self::NetworkKey(v) => w.write(v)?,
            Self::TcLinkKey(v) => w.write(v)?,
            Self::Device(v) => w.write(v)?,
            Self::EzspHashedTclk(v) => w.bytes(v)?,
            Self::SourceRoute(v) => w.write(v)?,
            Self::GpProxy(v) => w.write(v)?,
            Self::RsParams(v) => w.write(v)?,
            Self::GpSink(v) => w.write(v)?,
            Self::GpTranslation(v) => w.write(v)?,
            Self::End(v) => w.write(v)?,
            Self::Unknown { value, .. } => w.bytes(value)?,
        }
        let len = u16::try_from(w.len() - start).map_err(|_| CodecError::Invalid)?;
        w.patch(at, &len.to_le_bytes());
        Ok(())
    }
}

/// Split a record stream into raw TLVs without decoding the values.
#[derive(Debug, Clone)]
pub struct RawRecords<'a> {
    rest: &'a [u8],
}

impl<'a> RawRecords<'a> {
    /// Iterate the TLVs of `stream`.
    pub const fn new(stream: &'a [u8]) -> Self {
        Self { rest: stream }
    }
}

impl<'a> Iterator for RawRecords<'a> {
    /// `(type, full TLV bytes)`, or `Err` on a truncated TLV.
    type Item = Result<(u8, &'a [u8]), CodecError>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.rest.is_empty() {
            return None;
        }
        if self.rest.len() < 3 {
            self.rest = &[];
            return Some(Err(CodecError::Truncated));
        }
        let len = 3 + u16::from_le_bytes([self.rest[1], self.rest[2]]) as usize;
        if self.rest.len() < len {
            self.rest = &[];
            return Some(Err(CodecError::Truncated));
        }
        let (tlv, rest) = self.rest.split_at(len);
        self.rest = rest;
        Some(Ok((tlv[0], tlv)))
    }
}

/// Incremental `END` record computation.
#[derive(Debug, Clone, Copy, Default)]
pub struct StreamCrc {
    count: u32,
    crc: Crc32c,
}

impl StreamCrc {
    /// Empty stream.
    pub const fn new() -> Self {
        Self {
            count: 0,
            crc: Crc32c::new(),
        }
    }

    /// Account for one complete TLV (type, length and value bytes).
    pub fn add(&mut self, tlv: &[u8]) {
        self.count = self.count.saturating_add(1);
        self.crc.update(tlv);
    }

    /// Encode `record` into `w` and account for it.
    pub fn write_record(
        &mut self,
        record: &Record<'_>,
        w: &mut Writer<'_>,
    ) -> Result<(), CodecError> {
        let start = w.len();
        record.write(w)?;
        self.add(&w.written()[start..]);
        Ok(())
    }

    /// The `END` record for everything added so far.
    pub const fn end(&self) -> EndRecord {
        EndRecord {
            record_count: self.count,
            crc32c: self.crc.finish(),
        }
    }
}

/// `END` verification errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamError {
    /// TLV structure error.
    Codec(CodecError),
    /// No `END` record.
    MissingEnd,
    /// Bytes after `END`.
    TrailingData,
    /// `END.record_count` mismatch.
    CountMismatch,
    /// `END.crc32c` mismatch.
    CrcMismatch,
}

/// Check that `stream` is a sequence of TLVs closed by a matching `END`.
/// Returns the number of records before `END`.
pub fn verify_stream(stream: &[u8]) -> Result<u32, StreamError> {
    let mut acc = StreamCrc::new();
    let mut it = RawRecords::new(stream);
    while let Some(item) = it.next() {
        let (rtype, tlv) = item.map_err(StreamError::Codec)?;
        if rtype == record_type::END {
            if it.next().is_some() {
                return Err(StreamError::TrailingData);
            }
            let end: EndRecord = Reader::new(&tlv[3..]).read().map_err(StreamError::Codec)?;
            let want = acc.end();
            if end.record_count != want.record_count {
                return Err(StreamError::CountMismatch);
            }
            if end.crc32c != want.crc32c {
                return Err(StreamError::CrcMismatch);
            }
            return Ok(end.record_count);
        }
        acc.add(tlv);
    }
    Err(StreamError::MissingEnd)
}

/// Frame-counter start rule of `BKP_RESTORE_COMMIT` (9.4):
///
/// * `nwk_fc_start = max(NETWORK_KEY.tx_counter, RS_PARAMS.nwk_fc_limit) + nwk_margin`
/// * `aps_fc_start = max(TC_LINK_KEY.tx_counter, max(DEVICE.link.tx), RS_PARAMS.aps_fc_limit) + aps_margin`
///
/// Additions saturate at `u32::MAX`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RestoreCounters {
    /// Largest NWK input seen.
    pub nwk_max: u32,
    /// Largest APS input seen.
    pub aps_max: u32,
}

impl RestoreCounters {
    /// No inputs.
    pub const fn new() -> Self {
        Self {
            nwk_max: 0,
            aps_max: 0,
        }
    }

    /// Account for one record.
    pub fn add(&mut self, record: &Record<'_>) {
        match record {
            Record::NetworkKey(k) => self.nwk_max = self.nwk_max.max(k.tx_counter),
            Record::RsParams(p) => {
                self.nwk_max = self.nwk_max.max(p.nwk_fc_limit);
                self.aps_max = self.aps_max.max(p.aps_fc_limit);
            }
            Record::TcLinkKey(k) => self.aps_max = self.aps_max.max(k.tx_counter),
            Record::Device(DeviceRecord { link: Some(l), .. }) => {
                self.aps_max = self.aps_max.max(l.tx_counter)
            }
            _ => {}
        }
    }

    /// Raise both inputs to host floors (`NWK_INIT`-style).
    pub fn apply_floors(&mut self, nwk_floor: u32, aps_floor: u32) {
        self.nwk_max = self.nwk_max.max(nwk_floor);
        self.aps_max = self.aps_max.max(aps_floor);
    }

    /// NWK start.
    pub const fn nwk_start(&self, nwk_margin: u32) -> u32 {
        self.nwk_max.saturating_add(nwk_margin)
    }

    /// APS start.
    pub const fn aps_start(&self, aps_margin: u32) -> u32 {
        self.aps_max.saturating_add(aps_margin)
    }
}

/// Margin hosts use when a backup carries no TCLK frame counter (9.4).
pub const APS_UNKNOWN_TCLK_MARGIN: u32 = 1 << 20;
