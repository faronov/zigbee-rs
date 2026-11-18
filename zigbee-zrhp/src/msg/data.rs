//! DATA group (`0x02xx`) and ZDO group (`0x03xx`), endpoint ZIGBEE,
//! spec 7.3–7.4.

use crate::codec::{Bytes16, CodecError, List, Reader, Wire, Writer, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::types::Ieee;

/// `tx_options` bits of `APS_DATA_REQUEST`.
pub mod tx_options {
    /// Request an APS ACK.
    pub const APS_ACK: u8 = 0x01;
    /// APS link-key security.
    pub const APS_SECURITY: u8 = 0x02;
    /// Force route discovery.
    pub const FORCE_ROUTE_DISCOVERY: u8 = 0x04;
    /// Fragmentation permitted.
    pub const FRAGMENTATION: u8 = 0x08;
    /// Alias fields present (GP proxy).
    pub const USE_ALIAS: u8 = 0x10;
    /// Extended timeout (sleepy destination).
    pub const EXTENDED_TIMEOUT: u8 = 0x20;
    /// Source-route relays present.
    pub const SOURCE_ROUTE: u8 = 0x40;
}

/// `APS_DATA_REQUEST` destination (`dst_mode` + `dst`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApsDst {
    /// `0x01` group.
    Group(u16),
    /// `0x02` NWK short address.
    Nwk(u16),
    /// `0x03` IEEE.
    Ieee(Ieee),
    /// `0x0F` broadcast (`0xFFFF`, `0xFFFD`, `0xFFFC`, `0xFFFB`).
    Broadcast(u16),
}

impl ApsDst {
    /// The `dst_mode` byte.
    pub const fn mode(&self) -> u8 {
        match self {
            Self::Group(_) => 0x01,
            Self::Nwk(_) => 0x02,
            Self::Ieee(_) => 0x03,
            Self::Broadcast(_) => 0x0F,
        }
    }
}

impl<'a> Wire<'a> for ApsDst {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        Ok(match r.u8()? {
            0x01 => Self::Group(r.u16()?),
            0x02 => Self::Nwk(r.u16()?),
            0x03 => Self::Ieee(r.read()?),
            0x0F => Self::Broadcast(r.u16()?),
            _ => return Err(CodecError::Invalid),
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.u8(self.mode())?;
        match self {
            Self::Group(v) | Self::Nwk(v) | Self::Broadcast(v) => w.u16(*v),
            Self::Ieee(i) => w.write(i),
        }
    }
}

/// `APS_DATA_REQUEST` (`0x0201`).
///
/// `tx_options` bits 4 (alias) and 6 (source route) are derived from
/// `alias` / `relays` when encoding, so the two can never disagree.
#[derive(Debug, Clone, PartialEq)]
pub struct ApsDataRequest<'a> {
    /// Destination.
    pub dst: ApsDst,
    /// Destination Zigbee endpoint (ignored for groups).
    pub dst_ep: u8,
    /// Source endpoint (host endpoint, 0 ZDO or 242 GP).
    pub src_ep: u8,
    /// Profile.
    pub profile: u16,
    /// Cluster.
    pub cluster: u16,
    /// [`tx_options`] bits.
    pub tx_options: u8,
    /// Radius (0 = default).
    pub radius: u8,
    /// Non-member radius (groups only).
    pub non_member_radius: u8,
    /// 0 low, 1 normal, 2 high, 3 critical.
    pub priority: u8,
    /// Source-route relays, path order from the coordinator (bit 6).
    pub relays: Option<List<'a, u16>>,
    /// `(alias_src, alias_seq)` (bit 4).
    pub alias: Option<(u16, u8)>,
    /// ASDU.
    pub asdu: &'a [u8],
}

impl<'a> Wire<'a> for ApsDataRequest<'a> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let dst = r.read()?;
        let dst_ep = r.u8()?;
        let src_ep = r.u8()?;
        let profile = r.u16()?;
        let cluster = r.u16()?;
        let tx_options = r.u8()?;
        let radius = r.u8()?;
        let non_member_radius = r.u8()?;
        let priority = r.u8()?;
        let relays = if tx_options & tx_options::SOURCE_ROUTE != 0 {
            Some(r.read()?)
        } else {
            None
        };
        let alias = if tx_options & tx_options::USE_ALIAS != 0 {
            Some((r.u16()?, r.u8()?))
        } else {
            None
        };
        let asdu = Bytes16::read(r)?.0;
        Ok(Self {
            dst,
            dst_ep,
            src_ep,
            profile,
            cluster,
            tx_options,
            radius,
            non_member_radius,
            priority,
            relays,
            alias,
            asdu,
        })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        let mut opts = self.tx_options & !(tx_options::SOURCE_ROUTE | tx_options::USE_ALIAS);
        if self.relays.is_some() {
            opts |= tx_options::SOURCE_ROUTE;
        }
        if self.alias.is_some() {
            opts |= tx_options::USE_ALIAS;
        }
        w.write(&self.dst)?;
        w.u8(self.dst_ep)?;
        w.u8(self.src_ep)?;
        w.u16(self.profile)?;
        w.u16(self.cluster)?;
        w.u8(opts)?;
        w.u8(self.radius)?;
        w.u8(self.non_member_radius)?;
        w.u8(self.priority)?;
        if let Some(relays) = &self.relays {
            w.write(relays)?;
        }
        if let Some((src, seq)) = self.alias {
            w.u16(src)?;
            w.u8(seq)?;
        }
        w.write(&Bytes16(self.asdu))
    }
}

wire_struct! {
    /// `EVT_APS_DATA_CONFIRM` (`0x0281`).
    pub struct EvtApsDataConfirm {
        /// TID of the `APS_DATA_REQUEST`.
        pub req_tid: u16,
        /// `SUCCESS` or `FAILURE` (raw status byte).
        pub status: u8,
        /// Zigbee status.
        pub zb_status: u8,
        /// APS counter.
        pub aps_counter: u8,
        /// Destination short address.
        pub dst_nwk: u16,
        /// Bit0 APS ACK received, bit1 via source route, bit2 route discovery done.
        pub flags: u8,
        /// Elapsed time.
        pub elapsed_ms: u16,
    }
}

/// `EVT_APS_DATA_INDICATION` flag bits.
pub mod indication_flags {
    /// APS secured.
    pub const APS_SECURED: u8 = 0x01;
    /// NWK secured.
    pub const NWK_SECURED: u8 = 0x02;
    /// `src_ieee` valid.
    pub const SRC_IEEE_VALID: u8 = 0x04;
    /// Broadcast.
    pub const BROADCAST: u8 = 0x08;
    /// Groupcast.
    pub const GROUPCAST: u8 = 0x10;
    /// Reassembled.
    pub const REASSEMBLED: u8 = 0x20;
    /// Already answered by the NCP.
    pub const HANDLED_BY_NCP: u8 = 0x40;
    /// Local loopback (GP proxy).
    pub const LOCAL_LOOPBACK: u8 = 0x80;
}

wire_struct! {
    /// `EVT_APS_DATA_INDICATION` (`0x0280`).
    pub struct EvtApsDataIndication<'a> {
        /// `0x01` group, `0x02` unicast to NCP, `0x0F` broadcast.
        pub dst_mode: u8,
        /// Group, `0x0000`, or broadcast address.
        pub dst: u16,
        /// Destination endpoint.
        pub dst_ep: u8,
        /// Source short address.
        pub src_nwk: u16,
        /// Source IEEE (valid if flags bit2).
        pub src_ieee: Ieee,
        /// Source endpoint.
        pub src_ep: u8,
        /// Profile.
        pub profile: u16,
        /// Cluster.
        pub cluster: u16,
        /// [`indication_flags`].
        pub flags: u8,
        /// LQI.
        pub lqi: u8,
        /// RSSI (dBm, last hop).
        pub rssi: i8,
        /// Last hop short address.
        pub last_hop_nwk: u16,
        /// APS counter.
        pub aps_counter: u8,
        /// 0 n/a, 1 global TCLK, 2 unique TCLK, 3 application link key.
        pub key_source: u8,
        /// NCP monotonic receive time.
        pub rx_time_ms: u32,
        /// ASDU.
        pub asdu: Bytes16<'a>,
    }
}

wire_struct! {
    /// `ZDO_CONFIG` (`0x0301`).
    pub struct ZdoConfig {
        /// Bit0 forward handled requests, bit1 forward Match_Desc.
        pub flags: u8,
    }
}

message_def! {
    ApsDataRequest<'a> = ids::APS_DATA_REQUEST, Request;
    EvtApsDataConfirm = ids::EVT_APS_DATA_CONFIRM, Event;
    EvtApsDataIndication<'a> = ids::EVT_APS_DATA_INDICATION, Event;
    ZdoConfig = ids::ZDO_CONFIG, Request;
}
