//! Green Power sink (GP server, combo device) – sans-IO core.
//!
//! [`Sink`] owns the Sink Table, the sink commissioning mode and the
//! commissioning of GPDs, and translates GPD commands for one local
//! application endpoint ([`crate::translation`]). The caller (normally
//! `zigbee-runtime`) feeds:
//!
//! * GPDFs it received directly ([`Sink::on_gpdf`]) – a combo device
//!   processes them locally instead of tunnelling them to itself;
//! * Green Power cluster commands addressed to the GP server
//!   ([`Sink::on_server_command`]): GP Notification and GP Commissioning
//!   Notification from proxies, GP Sink Commissioning Mode and GP Sink
//!   Table Request;
//! * GP server attribute reads and writes ([`Sink::read_attribute`],
//!   [`Sink::write_attribute`]);
//! * the product's local commissioning button
//!   ([`Sink::enter_commissioning`]).
//!
//! and drains the network frames ([`Sink::poll`]: GP Pairing, GP Proxy
//! Commissioning Mode, GP Response, GP Notification Response, GP Sink Table
//! Response, alias `Device_annce`), the translated ZCL commands for the
//! application endpoint ([`Sink::take_command`]), GPEP group membership
//! changes ([`Sink::take_group_change`]) and application events
//! ([`Sink::take_event`]).
//!
//! No GP specification text is available to this project. Formats and
//! behaviour follow the certified Silicon Labs GP 1.1 sink (gecko_sdk
//! `app/zcl/green-power.xml`, `protocol/zigbee/app/framework/plugin/
//! green-power-server/green-power-server.c`, `green-power-security.c` and
//! `green-power-server-gpd-cmd-maps.c`) and the zigpy sink used by Home
//! Assistant; the functions below name the Silicon Labs routine they mirror.
//!
//! Not implemented (rejected explicitly, not silently accepted): GP
//! Translation Table Update/Request, GP Pairing Configuration, Application
//! Description (multi-sensor report descriptors), GPD Channel Request
//! handling, key provisioning to a GPD (shared-key attributes, key in the
//! Commissioning Reply), commissioning through a GP commissioning tool
//! (GPM, involve-TC) and GP Pairing Search.

use zigbee_crypto::{AesKey, ForwardAesProvider};

use crate::cluster::{
    CommissioningNotification, GP_CLUSTER_REVISION, Notification, NotificationResponse, Pairing,
    PairingSink, ProxyCommissioningMode, ProxyInfo, Response, attr, client_cmd, exit_mode,
    functionality, server_cmd, status,
};
use crate::codec::{Reader, Writer};
use crate::commissioning::{CommissioningPayload, CommissioningReply};
use crate::dup::DuplicateFilter;
use crate::frame::{FrameType, Gpdf, GpdfHeader, GpdfParams, MIC_LEN, write_header};
use crate::proxy::{
    BROADCAST_RX_ON_WHEN_IDLE, DestAddr, GpTx, GroupChange, MAX_GP_COMMAND_LEN, MAX_SINK_GROUPS,
    MAX_TX_PAYLOAD, SinkGroup, TxFrame,
};
use crate::security::{self, DEFAULT_TC_LINK_KEY, Direction, GpdPayload, WrappedKey};
use crate::translation::{self, Translation};
use crate::types::{
    ApplicationId, CommunicationMode, GpdId, IeeeAddress, KeyType, SecurityLevel, cmd, derive_alias,
};
use crate::{Error, Result};

/// Maximum encoded length of one Sink Table entry (options, IEEE GPD ID,
/// DeviceID, two commissioned groups, alias, radius, security options,
/// counter, key).
pub const MAX_SINK_ENTRY_ENCODED_LEN: usize =
    2 + 9 + 1 + 1 + MAX_SINK_GROUPS * 4 + 2 + 1 + 1 + 4 + 16;
/// gpsCommissioningWindow default, s (gecko_sdk `green-power.xml`, 0x00B4).
pub const DEFAULT_SINK_COMMISSIONING_WINDOW_S: u16 = 180;
/// Default [`SinkConfig::counter_persist_stride`] (as the proxy).
pub const DEFAULT_SINK_COUNTER_PERSIST_STRIDE: u32 = 64;
/// GPDs waiting for their Success after a Commissioning Reply.
pub const MAX_PENDING_COMMISSIONING: usize = 2;
/// Application events kept until [`Sink::take_event`].
pub const SINK_EVENT_QUEUE_LEN: usize = 3;
/// Translated commands kept until [`Sink::take_command`].
pub const SINK_COMMAND_QUEUE_LEN: usize = 2;

/// gpsCommissioningExitMode bits (`GpSinkCommissioningModeExitMode`).
pub mod exit_mode_attr {
    /// Leave sink commissioning mode when the commissioning window expires.
    pub const ON_WINDOW_EXPIRATION: u8 = 0x01;
    /// Leave on the first successful pairing.
    pub const ON_FIRST_PAIRING: u8 = 0x02;
    /// Leave on an explicit exit (GP Proxy Commissioning Mode exit).
    pub const ON_PROXY_COMMISSIONING_MODE_EXIT: u8 = 0x04;
}

/// gpsSecurityLevel bits (GP 1.1 sink attribute 0x0005, as evaluated by
/// gecko_sdk `gpCommissioningNotificationCommissioningGpdf`).
pub mod security_level_attr {
    /// Minimal GPD security level accepted at commissioning.
    pub const MINIMAL_LEVEL_MASK: u8 = 0x03;
    /// A GPD key sent at commissioning must be TC-LK protected.
    pub const PROTECTION_WITH_GP_LINK_KEY: u8 = 0x04;
    /// Involve the Trust Center (not supported: commissioning is refused).
    pub const INVOLVE_TC: u8 = 0x08;
}

/// GP Sink Commissioning Mode options (`GpSinkCommissioningModeOptions`).
pub mod sink_commissioning_options {
    /// Enter (1) or exit (0).
    pub const ACTION: u8 = 0x01;
    /// Involve a GP commissioning tool in security.
    pub const INVOLVE_GPM_IN_SECURITY: u8 = 0x02;
    /// Involve a GP commissioning tool in pairing.
    pub const INVOLVE_GPM_IN_PAIRING: u8 = 0x04;
    /// Broadcast GP Proxy Commissioning Mode to the proxies.
    pub const INVOLVE_PROXIES: u8 = 0x08;
}

/// ZCL status codes returned by [`Sink::on_server_command`] and
/// [`Sink::write_attribute`].
pub mod zcl_status {
    /// SUCCESS.
    pub const SUCCESS: u8 = 0x00;
    /// UNSUP_CLUSTER_COMMAND.
    pub const UNSUP_COMMAND: u8 = 0x81;
    /// INVALID_FIELD.
    pub const INVALID_FIELD: u8 = 0x85;
    /// UNSUPPORTED_ATTRIBUTE.
    pub const UNSUPPORTED_ATTRIBUTE: u8 = 0x86;
    /// INVALID_VALUE.
    pub const INVALID_VALUE: u8 = 0x87;
    /// READ_ONLY.
    pub const READ_ONLY: u8 = 0x88;
    /// NOT_FOUND.
    pub const NOT_FOUND: u8 = 0x8B;
    /// INVALID_DATA_TYPE.
    pub const INVALID_DATA_TYPE: u8 = 0x8D;
}

const BITMAP8: u8 = 0x18;
const BITMAP24: u8 = 0x1A;
const UINT8: u8 = 0x20;
const UINT16: u8 = 0x21;
const LONG_OCTET_STRING: u8 = 0x43;

/// gpsFunctionality of this sink (GP 1.1 bits, names from gecko_sdk
/// `GpGpsFunctionality`): GP feature, direct reception, derived and
/// pre-commissioned groupcast, full and lightweight unicast, multi-hop
/// bidirectional commissioning (GP Response through a TempMaster),
/// proximity and multi-hop commissioning, security levels 0, 2 and 3 and
/// IEEE-addressed GPDs. Proximity bidirectional operation (bit 6) is added
/// while the local proxy can act as TempMaster
/// ([`Sink::set_direct_bidirectional`]). Not advertised: Proxy Table
/// maintenance, CT-based commissioning, GPD maintenance (Channel Request),
/// sink-table-based groupcast forwarding, Translation Table commands and
/// compact attribute reporting.
pub const SINK_FUNCTIONALITY: u32 = functionality::GP_FEATURE
    | functionality::DIRECT_COMMUNICATION
    | functionality::DERIVED_GROUPCAST
    | functionality::PRECOMMISSIONED_GROUPCAST
    | functionality::FULL_UNICAST
    | functionality::LIGHTWEIGHT_UNICAST
    | functionality::BIDIRECTIONAL
    | PROXIMITY_COMMISSIONING
    | functionality::GP_COMMISSIONING
    | functionality::SECURITY_LEVEL_0
    | functionality::SECURITY_LEVEL_2
    | functionality::SECURITY_LEVEL_3
    | functionality::IEEE_ADDRESS;
/// gpsFunctionality bit 6, proximity bidirectional operation.
pub const PROXIMITY_BIDIRECTIONAL: u32 = 1 << 6;
/// gpsFunctionality bit 9, proximity commissioning.
pub const PROXIMITY_COMMISSIONING: u32 = 1 << 9;

/// Sink configuration selected by the product (initial values of the
/// writable GP server attributes and local policy).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SinkConfig {
    /// gpsCommunicationMode: the mode new pairings use.
    pub communication_mode: CommunicationMode,
    /// Group of [`CommunicationMode::CommissionedGroupcast`] pairings. The
    /// mode cannot be used without one.
    pub commissioned_group: Option<u16>,
    /// gpsCommissioningExitMode ([`exit_mode_attr`]).
    pub exit_mode: u8,
    /// gpsCommissioningWindow, s.
    pub commissioning_window_s: u16,
    /// gpsSecurityLevel ([`security_level_attr`]).
    pub security_level: u8,
    /// Groupcast radius sent in GP Pairing (`None` = not sent, stack
    /// default).
    pub groupcast_radius: Option<u8>,
    /// Local application endpoint that receives translated commands
    /// (`None`: GPD commands are only reported as events).
    pub endpoint: Option<u8>,
    /// Persist a protected GPD's frame counter after it advanced by this
    /// many values; restore skips ahead by `stride - 1` (as the proxy, see
    /// [`crate::proxy::ProxyConfig::counter_persist_stride`]). Batched
    /// persistence applies to reports; event commands are written exactly
    /// within the budget below ([`crate::counter`]).
    pub counter_persist_stride: u32,
    /// Exact counter writes allowed in a burst ([`crate::counter`]); 0 keeps
    /// every entry batched.
    pub exact_counter_burst: u8,
    /// One exact counter write is refilled per this many milliseconds.
    pub exact_counter_refill_ms: u32,
}

impl Default for SinkConfig {
    /// gecko_sdk `green-power.xml` attribute defaults: derived groupcast
    /// (0x01), exit on first pairing (0x02), 180 s window, minimal security
    /// level 0b01 (0x01: only levels 2 and 3 are commissioned).
    fn default() -> Self {
        Self {
            communication_mode: CommunicationMode::DerivedGroupcast,
            commissioned_group: None,
            exit_mode: exit_mode_attr::ON_FIRST_PAIRING,
            commissioning_window_s: DEFAULT_SINK_COMMISSIONING_WINDOW_S,
            security_level: 0x01,
            groupcast_radius: None,
            endpoint: None,
            counter_persist_stride: DEFAULT_SINK_COUNTER_PERSIST_STRIDE,
            exact_counter_burst: crate::counter::DEFAULT_EXACT_COUNTER_BURST,
            exact_counter_refill_ms: crate::counter::DEFAULT_EXACT_COUNTER_REFILL_MS,
        }
    }
}

/// One Sink Table entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SinkEntry {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// Communication mode of the pairing.
    pub communication_mode: CommunicationMode,
    /// GPD MAC sequence number is incremental.
    pub mac_seq_cap: bool,
    /// GPD can receive (RxOnCapability).
    pub rx_on_cap: bool,
    /// GPD has a fixed location.
    pub fixed_location: bool,
    /// Assigned alias (`None` = derived alias).
    pub assigned_alias: Option<u16>,
    /// Security level.
    pub security_level: SecurityLevel,
    /// Key type.
    pub key_type: KeyType,
    /// GPD key.
    pub key: Option<AesKey>,
    /// Last accepted security frame counter.
    pub frame_counter: u32,
    /// GPD DeviceID.
    pub device_id: u8,
    /// Commissioned groups ([`CommunicationMode::CommissionedGroupcast`]).
    pub groups: heapless::Vec<SinkGroup, MAX_SINK_GROUPS>,
    /// Groupcast radius (0 = stack default).
    pub groupcast_radius: u8,
    persisted_counter: u32,
    /// The last write stored the counter exactly ([`crate::counter`]).
    counter_exact: bool,
}

impl SinkEntry {
    /// A new, unprotected entry for `gpd_id` in derived groupcast.
    pub fn new(gpd_id: GpdId) -> Self {
        Self {
            gpd_id,
            communication_mode: CommunicationMode::DerivedGroupcast,
            mac_seq_cap: false,
            rx_on_cap: false,
            fixed_location: false,
            assigned_alias: None,
            security_level: SecurityLevel::None,
            key_type: KeyType::NoKey,
            key: None,
            frame_counter: 0,
            device_id: 0xFF,
            groups: heapless::Vec::new(),
            groupcast_radius: 0,
            persisted_counter: 0,
            counter_exact: false,
        }
    }

    /// Whether the frame counter is stored exactly (see
    /// [`crate::proxy::ProxyEntry::counter_exact`]).
    pub fn counter_exact(&self) -> bool {
        self.counter_exact
    }

    /// The entry as restored from storage that recorded the counter mode
    /// `exact` (storage without the mode restores batched: `false`).
    pub fn with_counter_exact(mut self, exact: bool) -> Self {
        self.counter_exact = exact;
        self
    }

    /// NWK alias of the GPD.
    pub fn alias(&self) -> u16 {
        self.assigned_alias
            .unwrap_or_else(|| derive_alias(&self.gpd_id))
    }

    /// GPEP groups this entry needs.
    fn group_ids(&self) -> heapless::Vec<u16, MAX_SINK_GROUPS> {
        let mut groups = heapless::Vec::new();
        match self.communication_mode {
            CommunicationMode::DerivedGroupcast => {
                let _ = groups.push(derive_alias(&self.gpd_id));
            }
            CommunicationMode::CommissionedGroupcast => {
                for g in &self.groups {
                    let _ = groups.push(g.group);
                }
            }
            _ => {}
        }
        groups
    }

    /// Sink Table entry options (`GpSinkTableEntryOptions`).
    pub fn options(&self) -> u16 {
        (self.gpd_id.application_id() as u16)
            | (self.communication_mode as u16) << 3
            | (self.mac_seq_cap as u16) << 5
            | (self.rx_on_cap as u16) << 6
            | (self.fixed_location as u16) << 7
            | (self.assigned_alias.is_some() as u16) << 8
            | (self.security_level.is_protected() as u16) << 9
    }

    /// Encode in the Sink Table attribute / GP Sink Table Response entry
    /// format (gecko_sdk `storeSinkTableEntryInBuffer`): options, GPD ID,
    /// DeviceID, group list (commissioned groupcast), alias, groupcast
    /// radius, then security options, frame counter and key (security use),
    /// or only the frame counter (sequence number capability).
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u16(self.options())?;
        match self.gpd_id {
            GpdId::SrcId(s) => w.u32(s)?,
            GpdId::Ieee { ieee, endpoint } => {
                w.bytes(&ieee)?;
                w.u8(endpoint)?;
            }
        }
        w.u8(self.device_id)?;
        if self.communication_mode == CommunicationMode::CommissionedGroupcast {
            w.u8(self.groups.len() as u8)?;
            for g in &self.groups {
                w.u16(g.group)?;
                w.u16(g.alias.unwrap_or(0xFFFF))?;
            }
        }
        if let Some(a) = self.assigned_alias {
            w.u16(a)?;
        }
        w.u8(self.groupcast_radius)?;
        if self.security_level.is_protected() {
            w.u8(self.security_level as u8 | (self.key_type as u8) << 2)?;
            w.u32(self.frame_counter)?;
            w.bytes(&self.key.unwrap_or([0; 16]))?;
        } else if self.mac_seq_cap {
            w.u32(self.frame_counter)?;
        }
        Ok(w.len())
    }

    /// Decode one entry, returning it and the consumed length.
    ///
    /// SecurityUse with security level 0 (written by gecko_sdk for GPDs
    /// that sent extended options) is accepted: the entry is unprotected.
    pub fn decode(buf: &[u8]) -> Result<(Self, usize)> {
        let mut r = Reader::new(buf);
        let o = r.u16()?;
        let gpd_id = match ApplicationId::from_bits(o as u8)? {
            ApplicationId::SrcId => GpdId::SrcId(r.u32()?),
            ApplicationId::Ieee => {
                let ieee = r.array::<8>()?;
                GpdId::Ieee {
                    ieee,
                    endpoint: r.u8()?,
                }
            }
            ApplicationId::Lped => return Err(Error::Unsupported),
        };
        let mut e = Self::new(gpd_id);
        e.communication_mode = CommunicationMode::from_bits((o >> 3) as u8);
        e.mac_seq_cap = o & (1 << 5) != 0;
        e.rx_on_cap = o & (1 << 6) != 0;
        e.fixed_location = o & (1 << 7) != 0;
        e.device_id = r.u8()?;
        if e.communication_mode == CommunicationMode::CommissionedGroupcast {
            let n = r.u8()?;
            for _ in 0..n {
                let group = r.u16()?;
                let alias = r.u16()?;
                e.groups
                    .push(SinkGroup {
                        group,
                        alias: (alias != 0xFFFF).then_some(alias),
                    })
                    .map_err(|_| Error::BufferFull)?;
            }
        }
        if o & (1 << 8) != 0 {
            e.assigned_alias = Some(r.u16()?);
        }
        e.groupcast_radius = r.u8()?;
        if o & (1 << 9) != 0 {
            let so = r.u8()?;
            let level = SecurityLevel::from_bits(so);
            if level == SecurityLevel::Reserved {
                return Err(Error::Unsupported);
            }
            if level.is_protected() || e.mac_seq_cap {
                e.frame_counter = r.u32()?;
            }
            if level.is_protected() {
                e.security_level = level;
                e.key_type = KeyType::from_bits(so >> 2)?;
                e.key = Some(r.array::<16>()?);
            }
        } else if e.mac_seq_cap {
            e.frame_counter = r.u32()?;
        }
        e.persisted_counter = e.frame_counter;
        Ok((e, buf.len() - r.remaining()))
    }

    /// Locate the GPD ID bytes and the 4-byte frame counter of a protected
    /// entry produced by [`encode`](Self::encode), so storage can journal a
    /// counter change without a new snapshot. Unprotected entries report no
    /// counter: their sequence number is not persisted.
    pub fn encoded_layout(buf: &[u8]) -> Result<(core::ops::Range<usize>, Option<usize>)> {
        let mut r = Reader::new(buf);
        let o = r.u16()?;
        let id_len = match ApplicationId::from_bits(o as u8)? {
            ApplicationId::SrcId => 4,
            ApplicationId::Ieee => 9,
            ApplicationId::Lped => return Err(Error::Unsupported),
        };
        let id = 2..2 + id_len;
        r.bytes(id_len + 1)?;
        if CommunicationMode::from_bits((o >> 3) as u8) == CommunicationMode::CommissionedGroupcast
        {
            let n = r.u8()? as usize;
            r.bytes(n * 4)?;
        }
        if o & (1 << 8) != 0 {
            r.bytes(2)?;
        }
        r.u8()?;
        if o & (1 << 9) == 0 {
            return Ok((id, None));
        }
        let level = SecurityLevel::from_bits(r.u8()?);
        if !level.is_protected() {
            return Ok((id, None));
        }
        let at = buf.len() - r.remaining();
        r.bytes(4 + 16)?;
        Ok((id, Some(at)))
    }

    fn same_except_counter(&mut self, other: &Self) -> bool {
        let counters = (
            self.frame_counter,
            self.persisted_counter,
            self.counter_exact,
        );
        self.frame_counter = other.frame_counter;
        self.persisted_counter = other.persisted_counter;
        self.counter_exact = other.counter_exact;
        let same = *self == *other;
        (
            self.frame_counter,
            self.persisted_counter,
            self.counter_exact,
        ) = counters;
        same
    }
}

/// GP Sink Commissioning Mode (client → server 0x05).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SinkCommissioningMode {
    /// Options ([`sink_commissioning_options`]).
    pub options: u8,
    /// GPM address for security (0xFFFF = none).
    pub gpm_addr_for_security: u16,
    /// GPM address for pairing (0xFFFF = none).
    pub gpm_addr_for_pairing: u16,
    /// Sink endpoint (0xFF = all).
    pub sink_endpoint: u8,
}

impl SinkCommissioningMode {
    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.options)?;
        w.u16(self.gpm_addr_for_security)?;
        w.u16(self.gpm_addr_for_pairing)?;
        w.u8(self.sink_endpoint)?;
        Ok(w.len())
    }

    /// Strictly decode.
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let s = Self {
            options: r.u8()?,
            gpm_addr_for_security: r.u16()?,
            gpm_addr_for_pairing: r.u16()?,
            sink_endpoint: r.u8()?,
        };
        r.finish()?;
        Ok(s)
    }
}

/// GP Sink Table Request (client → server 0x0A).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SinkTableRequest {
    /// Request type 0b00: the entry of one GPD.
    ByGpdId(GpdId),
    /// Request type 0b01: entries from an index.
    ByIndex(u8),
}

impl SinkTableRequest {
    /// Strictly decode (`GpSinkTableRequestOptions`: ApplicationID bits
    /// 0-2, request type bits 3-4).
    pub fn decode(buf: &[u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        let o = r.u8()?;
        let req = match (o >> 3) & 0x03 {
            0 => Self::ByGpdId(match ApplicationId::from_bits(o)? {
                ApplicationId::SrcId => GpdId::SrcId(r.u32()?),
                ApplicationId::Ieee => {
                    let ieee = r.array::<8>()?;
                    GpdId::Ieee {
                        ieee,
                        endpoint: r.u8()?,
                    }
                }
                ApplicationId::Lped => return Err(Error::Unsupported),
            }),
            1 => Self::ByIndex(r.u8()?),
            _ => return Err(Error::Unsupported),
        };
        r.finish()?;
        Ok(req)
    }

    /// Encode into `out`, returning the length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        match self {
            Self::ByGpdId(id) => {
                w.u8(id.application_id() as u8)?;
                match *id {
                    GpdId::SrcId(s) => w.u32(s)?,
                    GpdId::Ieee { ieee, endpoint } => {
                        w.bytes(&ieee)?;
                        w.u8(endpoint)?;
                    }
                }
            }
            Self::ByIndex(i) => {
                w.u8(0x08)?;
                w.u8(*i)?;
            }
        }
        Ok(w.len())
    }
}

/// A Green Power cluster command received for the GP server.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServerCommand<'a> {
    /// NWK source.
    pub src_nwk: u16,
    /// APS unicast to this node (not group or broadcast).
    pub unicast: bool,
    /// ZCL transaction sequence number.
    pub tsn: u8,
    /// ZCL command identifier ([`server_cmd`]).
    pub command_id: u8,
    /// ZCL payload.
    pub payload: &'a [u8],
}

/// Why a frame did not change the sink.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SinkDrop {
    /// Frame towards a GPD, reserved GPD ID or malformed payload.
    Invalid,
    /// Commissioning frame outside sink commissioning mode (or tunnelled
    /// in a GP Notification).
    NotCommissioning,
    /// No Sink Table entry for the GPD.
    NoEntry,
    /// Security level or key type differ from the Sink Table entry.
    SecurityMismatch,
    /// MIC verification or key unwrapping failed, or no key is known.
    AuthFailed,
    /// Frame counter not above the last accepted one.
    Stale,
    /// Duplicate within gpDuplicateTimeout.
    Duplicate,
    /// Commissioning refused by gpsSecurityLevel.
    SecurityPolicy,
    /// A protected GPD sent no key (this sink provisions none).
    NoKey,
    /// Bidirectional commissioning without a usable TempMaster.
    NoTempMaster,
    /// Sink Table full.
    TableFull,
    /// Not implemented (see the module documentation).
    Unsupported,
    /// Network transmission queue full.
    QueueFull,
}

/// Result of feeding a frame to the sink.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SinkOutcome {
    /// A translated ZCL command was queued for the application endpoint.
    Translated,
    /// The GPD command was queued as a [`SinkEvent::GpdCommand`].
    PassedThrough,
    /// A GPD was commissioned (Sink Table entry stored, GP Pairing queued).
    Commissioned,
    /// A Commissioning Reply was sent; the pairing completes on Success.
    CommissioningReplySent,
    /// A GPD was decommissioned.
    Decommissioned,
    /// The frame was dropped.
    Dropped(SinkDrop),
}

/// Why sink commissioning mode ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SinkCommissioningExit {
    /// Local API or GP Sink Commissioning Mode (exit).
    Explicit,
    /// First pairing with gpsCommissioningExitMode "on first pairing".
    FirstPairing,
    /// The commissioning window elapsed.
    Timeout,
}

/// Event for the application.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SinkEvent {
    /// Sink commissioning mode entered.
    CommissioningEntered {
        /// Window, `None` when it ends only on pairing or explicit exit.
        window_s: Option<u16>,
        /// GP Proxy Commissioning Mode was broadcast to the proxies.
        involve_proxies: bool,
    },
    /// Sink commissioning mode left.
    CommissioningLeft(SinkCommissioningExit),
    /// A GPD was commissioned.
    Commissioned {
        /// GPD identity.
        gpd_id: GpdId,
        /// GPD DeviceID.
        device_id: u8,
    },
    /// A GPD's commissioning failed after its frame was accepted.
    CommissioningFailed {
        /// GPD identity.
        gpd_id: GpdId,
        /// Reason.
        reason: SinkDrop,
    },
    /// A GPD was removed from the Sink Table.
    Decommissioned {
        /// GPD identity.
        gpd_id: GpdId,
    },
    /// A GPD command without a built-in translation (attribute reports,
    /// scenes, generic switch, ...), already authenticated and fresh.
    GpdCommand {
        /// GPD identity.
        gpd_id: GpdId,
        /// GPD DeviceID from the Sink Table.
        device_id: u8,
        /// GPD CommandID.
        command_id: u8,
        /// GPD command payload.
        payload: heapless::Vec<u8, { crate::frame::MAX_GPD_PAYLOAD }>,
    },
}

/// A translated command for the local application endpoint.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SinkCommand {
    /// GPD that sent it.
    pub gpd_id: GpdId,
    /// Local application endpoint.
    pub endpoint: u8,
    /// ZCL command.
    pub translation: Translation,
}

/// A network frame of the sink. All GP cluster commands it sends are
/// server → client; `tsn` is the ZCL transaction sequence number of the
/// request a response answers (`None`: the caller picks one).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SinkTx {
    /// The frame.
    pub tx: GpTx,
    /// ZCL TSN to reuse.
    pub tsn: Option<u8>,
}

/// Sink counters (wrapping), for diagnostics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SinkStats {
    /// Frames that could not be queued for transmission.
    pub tx_dropped: u32,
    /// Events dropped because the application did not take them.
    pub events_dropped: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CommissioningState {
    deadline_ms: Option<u32>,
    involve_proxies: bool,
    /// The GP Proxy Commissioning Mode (enter) carried the window.
    proxies_have_window: bool,
}

/// A queued network frame; [`Sink::poll`] encodes it (a GP Pairing from
/// the GPD's Sink Table entry at that time), which keeps the queue small.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Out {
    ProxyCommissioningMode(ProxyCommissioningMode),
    /// GP Pairing (AddSink) for the GPD's Sink Table entry.
    PairingAdd(GpdId),
    /// GP Pairing with RemoveGPD.
    PairingRemove {
        gpd_id: GpdId,
        mode: CommunicationMode,
        level: SecurityLevel,
        key_type: KeyType,
    },
    /// `Device_annce` for an alias.
    DeviceAnnce(u16),
    /// GP Response carrying a Commissioning Reply.
    Response {
        gpd_id: GpdId,
        temp_master: u16,
        tx_channel: u8,
        reply: [u8; MAX_REPLY_LEN],
        reply_len: u8,
    },
    NotificationResponse {
        dst: u16,
        tsn: u8,
        response: NotificationResponse,
    },
    SinkTableResponse {
        dst: u16,
        tsn: u8,
        request: SinkTableRequest,
    },
}

/// Commissioning Reply without a key: options and the optional PAN ID.
const MAX_REPLY_LEN: usize = 3;

#[derive(Debug, Clone, PartialEq, Eq)]
struct Pending {
    entry: SinkEntry,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    /// GPDF received by this node.
    Direct,
    /// GP Notification from a proxy.
    Notification,
    /// GP Commissioning Notification from a proxy.
    Commissioning,
}

/// A received GPD command after security processing.
struct Received<'a> {
    gpd: GpdId,
    level: SecurityLevel,
    key_type: Option<KeyType>,
    counter: u32,
    rx_after_tx: bool,
    temp_master: Option<u16>,
    command_id: u8,
    payload: &'a [u8],
    source: Source,
    now_ms: u32,
}

fn due(now: u32, deadline: u32) -> bool {
    now.wrapping_sub(deadline) < 0x8000_0000
}

/// The Green Power sink with `N` Sink Table entries and `Q` queued network
/// transmissions.
pub struct Sink<const N: usize, const Q: usize> {
    config: SinkConfig,
    own_nwk: u16,
    own_ieee: IeeeAddress,
    pan_id: u16,
    operational_channel: u8,
    direct_bidirectional: bool,
    table: heapless::Vec<SinkEntry, N>,
    dup: DuplicateFilter<8>,
    commissioning: Option<CommissioningState>,
    pending: heapless::Vec<Pending, MAX_PENDING_COMMISSIONING>,
    /// First GP Notification sender per (GPD, counter), for FirstToForward.
    first_forwarders: heapless::Deque<(GpdId, u32, u16), 4>,
    tx: heapless::Deque<Out, Q>,
    commands: heapless::Deque<SinkCommand, SINK_COMMAND_QUEUE_LEN>,
    events: heapless::Deque<SinkEvent, SINK_EVENT_QUEUE_LEN>,
    group_changes: heapless::Deque<GroupChange, 4>,
    dirty: bool,
    /// Bounds exact frame-counter writes ([`crate::counter`]).
    exact_budget: crate::counter::ExactBudget,
    stats: SinkStats,
}

impl<const N: usize, const Q: usize> Sink<N, Q> {
    /// Create an empty sink.
    pub const fn new(config: SinkConfig) -> Self {
        Self {
            config,
            own_nwk: 0xFFFE,
            own_ieee: [0; 8],
            pan_id: 0xFFFF,
            operational_channel: 0,
            direct_bidirectional: false,
            table: heapless::Vec::new(),
            dup: DuplicateFilter::new(crate::dup::DUPLICATE_TIMEOUT_MS),
            commissioning: None,
            pending: heapless::Vec::new(),
            first_forwarders: heapless::Deque::new(),
            tx: heapless::Deque::new(),
            commands: heapless::Deque::new(),
            events: heapless::Deque::new(),
            group_changes: heapless::Deque::new(),
            dirty: false,
            exact_budget: crate::counter::ExactBudget::new(),
            stats: SinkStats {
                tx_dropped: 0,
                events_dropped: 0,
            },
        }
    }

    /// Active configuration.
    pub fn config(&self) -> &SinkConfig {
        &self.config
    }

    /// Replace the configuration (product policy).
    pub fn set_config(&mut self, config: SinkConfig) {
        self.config = config;
    }

    /// Choose the local application endpoint for translated commands.
    pub fn set_endpoint(&mut self, endpoint: Option<u8>) {
        self.config.endpoint = endpoint;
    }

    /// Update this node's addresses (after join / rejoin / conflict).
    pub fn set_own_address(&mut self, nwk: u16, ieee: IeeeAddress, pan_id: u16) {
        self.own_nwk = nwk;
        self.own_ieee = ieee;
        self.pan_id = pan_id;
    }

    /// Update the network's IEEE 802.15.4 channel (11..=26), used as the
    /// TempMaster transmit channel of GP Responses.
    pub fn set_operational_channel(&mut self, channel: u8) {
        self.operational_channel = channel;
    }

    /// Declare whether this node's own proxy can act as TempMaster for a GPD
    /// it hears directly (bidirectional proxy with timed transmission).
    pub fn set_direct_bidirectional(&mut self, capable: bool) {
        self.direct_bidirectional = capable;
    }

    /// Diagnostic counters.
    pub fn stats(&self) -> SinkStats {
        self.stats
    }

    /// Drop the Sink Table and all volatile state (leave / factory reset).
    /// The emptied table is reported by [`take_dirty`](Self::take_dirty).
    pub fn clear(&mut self) {
        self.table.clear();
        self.dup = DuplicateFilter::new(crate::dup::DUPLICATE_TIMEOUT_MS);
        self.commissioning = None;
        self.pending.clear();
        self.first_forwarders.clear();
        self.tx.clear();
        self.commands.clear();
        self.events.clear();
        self.group_changes.clear();
        self.dirty = true;
    }

    fn counter_policy(&self) -> crate::counter::CounterPolicy {
        crate::counter::CounterPolicy {
            stride: self.config.counter_persist_stride.max(1),
            exact_burst: self.config.exact_counter_burst,
            exact_refill_ms: self.config.exact_counter_refill_ms,
        }
    }

    /// Sink Table entries.
    pub fn entries(&self) -> &[SinkEntry] {
        &self.table
    }

    /// Restore an entry from persistent storage: a protected entry's counter
    /// moves to the highest value accepted before the reboot (the stored
    /// counter when written exactly, otherwise skipped ahead by
    /// `counter_persist_stride - 1`; see [`crate::proxy::Proxy::restore`]).
    pub fn restore(&mut self, mut entry: SinkEntry) -> Result<()> {
        if entry.security_level.is_protected() {
            entry.frame_counter = self
                .counter_policy()
                .restore_floor(entry.persisted_counter, entry.counter_exact);
        }
        if let Some(i) = self.find(&entry.gpd_id) {
            self.table[i] = entry;
            return Ok(());
        }
        self.table.push(entry).map_err(|_| Error::BufferFull)
    }

    /// GPEP groups required by the Sink Table.
    pub fn required_groups(&self) -> impl Iterator<Item = u16> + '_ {
        self.table.iter().flat_map(|e| e.group_ids())
    }

    /// Returns and clears the "table changed, persist it" flag.
    pub fn take_dirty(&mut self) -> bool {
        core::mem::take(&mut self.dirty)
    }

    /// Set the "table changed" flag again (failed persistence).
    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    /// Whether the sink is in commissioning mode.
    pub fn in_commissioning_mode(&self) -> bool {
        self.commissioning.is_some()
    }

    /// Next translated command for the application endpoint.
    pub fn take_command(&mut self) -> Option<SinkCommand> {
        self.commands.pop_front()
    }

    /// Next application event.
    pub fn take_event(&mut self) -> Option<SinkEvent> {
        self.events.pop_front()
    }

    /// Next GPEP group membership change.
    pub fn take_group_change(&mut self) -> Option<GroupChange> {
        self.group_changes.pop_front()
    }

    /// Whether [`poll`](Self::poll) has a frame to send now.
    pub fn has_tx(&self) -> bool {
        !self.tx.is_empty()
    }

    /// Next timer deadline (commissioning window), if any.
    pub fn next_deadline(&self) -> Option<u32> {
        self.commissioning.and_then(|c| c.deadline_ms)
    }

    /// Process timers and return the next network frame, if any. Call
    /// repeatedly until it returns `None`.
    pub fn poll(&mut self, now_ms: u32) -> Option<SinkTx> {
        if let Some(c) = self.commissioning
            && c.deadline_ms.is_some_and(|d| due(now_ms, d))
        {
            self.leave_commissioning(SinkCommissioningExit::Timeout);
        }
        while let Some(out) = self.tx.pop_front() {
            if let Some(tx) = self.encode(&out) {
                return Some(tx);
            }
        }
        None
    }

    /// Encode a queued frame; `None` when it no longer applies (the GPD of
    /// a queued GP Pairing was removed) or does not encode.
    #[inline(never)]
    fn encode(&self, out: &Out) -> Option<SinkTx> {
        let broadcast = DestAddr::Broadcast(BROADCAST_RX_ON_WHEN_IDLE);
        let unicast = |nwk| DestAddr::Unicast { nwk, ieee: None };
        let mut payload = heapless::Vec::<u8, MAX_GP_COMMAND_LEN>::new();
        payload.resize_default(MAX_TX_PAYLOAD).ok()?;
        let (dest, command_id, tsn, len) = match *out {
            Out::ProxyCommissioningMode(m) => (
                broadcast,
                client_cmd::PROXY_COMMISSIONING_MODE,
                None,
                m.encode(&mut payload),
            ),
            Out::PairingAdd(gpd_id) => {
                let e = &self.table[self.find(&gpd_id)?];
                (
                    broadcast,
                    client_cmd::PAIRING,
                    None,
                    self.pairing(e).encode(&mut payload),
                )
            }
            Out::PairingRemove {
                gpd_id,
                mode,
                level,
                key_type,
            } => {
                // gecko_sdk `decommissionGpd`: mode, level and key type in
                // the options, no sink fields.
                let p = Pairing {
                    gpd_id,
                    add_sink: false,
                    remove_gpd: true,
                    communication_mode: mode,
                    gpd_fixed: false,
                    mac_seq_cap: false,
                    security_level: level,
                    key_type,
                    sink: PairingSink::None,
                    device_id: None,
                    frame_counter: None,
                    key: None,
                    assigned_alias: None,
                    groupcast_radius: None,
                };
                (broadcast, client_cmd::PAIRING, None, p.encode(&mut payload))
            }
            Out::DeviceAnnce(alias) => {
                return Some(SinkTx {
                    tx: GpTx {
                        dest: broadcast,
                        alias: Some((alias, 0)),
                        radius: 0,
                        frame: TxFrame::DeviceAnnce { alias },
                    },
                    tsn: None,
                });
            }
            Out::Response {
                gpd_id,
                temp_master,
                tx_channel,
                reply,
                reply_len,
            } => {
                let r = Response {
                    gpd_id,
                    transmit_on_endpoint_match: false,
                    temp_master_short: temp_master,
                    temp_master_tx_channel: tx_channel,
                    command_id: cmd::COMMISSIONING_REPLY,
                    payload: &reply[..usize::from(reply_len)],
                };
                (
                    broadcast,
                    client_cmd::RESPONSE,
                    None,
                    r.encode(&mut payload),
                )
            }
            Out::NotificationResponse { dst, tsn, response } => (
                unicast(dst),
                client_cmd::NOTIFICATION_RESPONSE,
                Some(tsn),
                response.encode(&mut payload),
            ),
            Out::SinkTableResponse { dst, tsn, request } => (
                unicast(dst),
                client_cmd::SINK_TABLE_RESPONSE,
                Some(tsn),
                self.sink_table_response(&request, &mut payload),
            ),
        };
        payload.truncate(len.ok()?);
        Some(SinkTx {
            tx: GpTx {
                dest,
                alias: None,
                radius: 0,
                frame: TxFrame::GpCommand {
                    command_id,
                    payload,
                },
            },
            tsn,
        })
    }

    fn find(&self, gpd: &GpdId) -> Option<usize> {
        self.table.iter().position(|e| e.gpd_id.matches(gpd))
    }

    fn pending_index(&self, gpd: &GpdId) -> Option<usize> {
        self.pending
            .iter()
            .position(|p| p.entry.gpd_id.matches(gpd))
    }

    fn event(&mut self, event: SinkEvent) {
        if self.events.is_full() {
            self.events.pop_front();
            self.stats.events_dropped = self.stats.events_dropped.wrapping_add(1);
        }
        let _ = self.events.push_back(event);
    }

    fn queue(&mut self, out: Out) -> bool {
        let ok = self.tx.push_back(out).is_ok();
        if !ok {
            self.stats.tx_dropped = self.stats.tx_dropped.wrapping_add(1);
        }
        ok
    }

    fn proxy_commissioning_mode(&self, enter: bool) -> ProxyCommissioningMode {
        let em = self.config.exit_mode;
        // gecko_sdk `..._gp_sink_commissioning_mode_command_handler`: only
        // one of the two exit-mode flags is forwarded to the proxies.
        let exit_bits = match (
            em & exit_mode_attr::ON_FIRST_PAIRING != 0,
            em & exit_mode_attr::ON_PROXY_COMMISSIONING_MODE_EXIT != 0,
        ) {
            (true, false) => exit_mode::ON_FIRST_PAIRING,
            (false, true) => exit_mode::ON_EXPLICIT_EXIT,
            _ => 0,
        };
        ProxyCommissioningMode {
            enter,
            exit_mode: exit_bits,
            window_s: (enter && em & exit_mode_attr::ON_WINDOW_EXPIRATION != 0)
                .then_some(self.config.commissioning_window_s),
            channel: None,
            unicast: false,
        }
    }

    /// Enter sink commissioning mode (local API, e.g. a button), optionally
    /// broadcasting GP Proxy Commissioning Mode (enter) to the proxies.
    ///
    /// A locally opened window always ends after gpsCommissioningWindow
    /// (default 180 s), also when gpsCommissioningExitMode only asks to exit
    /// on the first pairing: a button press must never leave the sink, and
    /// the proxies it involved, in commissioning mode indefinitely. A second
    /// call restarts the window. Refused with [`Error::Unsupported`] while
    /// gpsSecurityLevel asks to involve the Trust Center.
    pub fn enter_commissioning(&mut self, now_ms: u32, involve_proxies: bool) -> Result<()> {
        self.enter(now_ms, involve_proxies, true)
    }

    /// Enter commissioning mode; `local` arms gpsCommissioningWindow even
    /// without "on window expiration" in gpsCommissioningExitMode. GP Sink
    /// Commissioning Mode from the network follows the exit mode exactly
    /// (the window is armed with "on window expiration" or no exit
    /// condition at all).
    fn enter(&mut self, now_ms: u32, involve_proxies: bool, local: bool) -> Result<()> {
        if self.config.security_level & security_level_attr::INVOLVE_TC != 0 {
            return Err(Error::Unsupported);
        }
        let em = self.config.exit_mode & 0x07;
        let window = (local || em & exit_mode_attr::ON_WINDOW_EXPIRATION != 0 || em == 0)
            .then_some(self.config.commissioning_window_s);
        let pcm = self.proxy_commissioning_mode(true);
        self.commissioning = Some(CommissioningState {
            deadline_ms: window.map(|w| now_ms.wrapping_add(u32::from(w) * 1000)),
            involve_proxies,
            proxies_have_window: pcm.window_s.is_some(),
        });
        if involve_proxies {
            self.queue(Out::ProxyCommissioningMode(pcm));
        }
        self.event(SinkEvent::CommissioningEntered {
            window_s: window,
            involve_proxies,
        });
        Ok(())
    }

    /// Leave sink commissioning mode (local API).
    pub fn exit_commissioning(&mut self) {
        if self.commissioning.is_some() {
            self.leave_commissioning(SinkCommissioningExit::Explicit);
        }
    }

    fn leave_commissioning(&mut self, reason: SinkCommissioningExit) {
        let Some(c) = self.commissioning.take() else {
            return;
        };
        self.pending.clear();
        // Proxies time out on their own when they were given the window.
        if c.involve_proxies && !(reason == SinkCommissioningExit::Timeout && c.proxies_have_window)
        {
            let pcm = self.proxy_commissioning_mode(false);
            self.queue(Out::ProxyCommissioningMode(pcm));
        }
        self.event(SinkEvent::CommissioningLeft(reason));
    }

    /// Encode a GP *server* attribute (GP 1.1 A.3.3.2, sink attributes
    /// 0x0000-0x0007 and the cluster revision) into `out` as ZCL data.
    /// Returns the ZCL data type and the length, or `None` for an
    /// unsupported attribute. The Sink Table attribute fails with
    /// [`Error::BufferFull`] when it does not fit `out`; sinks are read with
    /// GP Sink Table Request instead.
    pub fn read_attribute(&self, id: u16, out: &mut [u8]) -> Result<Option<(u8, usize)>> {
        let mut w = Writer::new(out);
        let data_type = match id {
            attr::GPS_MAX_SINK_TABLE_ENTRIES => {
                w.u8(N.min(u8::MAX as usize) as u8)?;
                UINT8
            }
            attr::SINK_TABLE => {
                let mut tmp = [0u8; MAX_SINK_ENTRY_ENCODED_LEN];
                let mut total = 0usize;
                for e in &self.table {
                    total += e.encode(&mut tmp)?;
                }
                w.u16(u16::try_from(total).map_err(|_| Error::BufferFull)?)?;
                for e in &self.table {
                    let n = e.encode(&mut tmp)?;
                    w.bytes(&tmp[..n])?;
                }
                LONG_OCTET_STRING
            }
            attr::GPS_COMMUNICATION_MODE => {
                w.u8(self.config.communication_mode as u8)?;
                BITMAP8
            }
            attr::GPS_COMMISSIONING_EXIT_MODE => {
                w.u8(self.config.exit_mode)?;
                BITMAP8
            }
            attr::GPS_COMMISSIONING_WINDOW => {
                w.u16(self.config.commissioning_window_s)?;
                UINT16
            }
            attr::GPS_SECURITY_LEVEL => {
                w.u8(self.config.security_level)?;
                BITMAP8
            }
            attr::GPS_FUNCTIONALITY => {
                let proximity_bidirectional = if self.direct_bidirectional {
                    PROXIMITY_BIDIRECTIONAL
                } else {
                    0
                };
                w.u24(SINK_FUNCTIONALITY | proximity_bidirectional)?;
                BITMAP24
            }
            attr::GPS_ACTIVE_FUNCTIONALITY => {
                // gecko_sdk default 0xFFFFFF: every supported function on.
                w.u24(0x00FF_FFFF)?;
                BITMAP24
            }
            attr::CLUSTER_REVISION => {
                w.u16(GP_CLUSTER_REVISION)?;
                UINT16
            }
            _ => return Ok(None),
        };
        Ok(Some((data_type, w.len())))
    }

    /// Write a GP server attribute from ZCL data (`data_type`, `value`).
    /// Returns the ZCL status. Writable: gpsCommunicationMode,
    /// gpsCommissioningExitMode, gpsCommissioningWindow and gpsSecurityLevel
    /// (gecko_sdk `green-power.xml`); commissioned groupcast needs
    /// [`SinkConfig::commissioned_group`] and involve-TC is refused. Written
    /// values are volatile: the product configuration applies after a
    /// reboot.
    pub fn write_attribute(&mut self, id: u16, data_type: u8, value: &[u8]) -> u8 {
        use zcl_status::*;
        let expected = match id {
            attr::GPS_COMMUNICATION_MODE
            | attr::GPS_COMMISSIONING_EXIT_MODE
            | attr::GPS_SECURITY_LEVEL => BITMAP8,
            attr::GPS_COMMISSIONING_WINDOW => UINT16,
            attr::GPS_MAX_SINK_TABLE_ENTRIES
            | attr::SINK_TABLE
            | attr::GPS_FUNCTIONALITY
            | attr::GPS_ACTIVE_FUNCTIONALITY
            | attr::CLUSTER_REVISION => return READ_ONLY,
            _ => return UNSUPPORTED_ATTRIBUTE,
        };
        let len = if expected == UINT16 { 2 } else { 1 };
        if data_type != expected || value.len() != len {
            return INVALID_DATA_TYPE;
        }
        let byte = value[0];
        match id {
            attr::GPS_COMMUNICATION_MODE => {
                let mode = CommunicationMode::from_bits(byte);
                if byte & !0x03 != 0
                    || (mode == CommunicationMode::CommissionedGroupcast
                        && self.config.commissioned_group.is_none())
                {
                    return INVALID_VALUE;
                }
                self.config.communication_mode = mode;
            }
            attr::GPS_COMMISSIONING_EXIT_MODE => {
                if byte & !0x07 != 0 {
                    return INVALID_VALUE;
                }
                self.config.exit_mode = byte;
            }
            attr::GPS_SECURITY_LEVEL => {
                if byte & !0x07 != 0 {
                    return INVALID_VALUE;
                }
                self.config.security_level = byte;
            }
            _ => self.config.commissioning_window_s = u16::from_le_bytes([value[0], value[1]]),
        }
        SUCCESS
    }

    /// Key for a protected GPDF of `gpd`: the Sink Table entry's or a
    /// pending commissioning's, when level and key kind match.
    fn key_for(&self, gpd: &GpdId, level: SecurityLevel, individual: bool) -> Option<AesKey> {
        let matching = |e: &SinkEntry| {
            (e.security_level == level && e.key_type.is_individual() == individual)
                .then_some(e.key)
                .flatten()
        };
        self.find(gpd)
            .and_then(|i| matching(&self.table[i]))
            .or_else(|| {
                self.pending_index(gpd)
                    .and_then(|i| matching(&self.pending[i].entry))
            })
    }

    /// Process a GPDF this node received directly (combo device). The GPDF
    /// is authenticated with the Sink Table (or pending commissioning) key
    /// and is never tunnelled to this node.
    pub fn on_gpdf<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        frame: &Gpdf<'_>,
        now_ms: u32,
    ) -> SinkOutcome {
        use SinkDrop::*;
        let h = &frame.header;
        if h.to_gpd {
            return SinkOutcome::Dropped(Invalid);
        }
        // Maintenance frames without a SrcID carry only a Channel Request.
        let Some(gpd) = h.gpd_id else {
            return SinkOutcome::Dropped(Unsupported);
        };
        if gpd.is_reserved() {
            return SinkOutcome::Dropped(Invalid);
        }
        let protected = h.security_level.is_protected();
        let clear = if protected {
            let Some(key) = self.key_for(&gpd, h.security_level, h.individual_key) else {
                return SinkOutcome::Dropped(AuthFailed);
            };
            match security::open(aes, &key, frame, Direction::FromGpd) {
                Ok(clear) => clear,
                Err(_) => return SinkOutcome::Dropped(AuthFailed),
            }
        } else {
            let mut clear = GpdPayload::new();
            if clear.extend_from_slice(frame.payload).is_err() {
                return SinkOutcome::Dropped(Invalid);
            }
            clear
        };
        let Some((&command_id, payload)) = clear.split_first() else {
            return SinkOutcome::Dropped(Invalid);
        };
        let counter = if protected {
            h.frame_counter
        } else {
            u32::from(frame.mac_seq)
        };
        if self.dup.check_and_record(&gpd, counter, now_ms) {
            return SinkOutcome::Dropped(Duplicate);
        }
        let rx = Received {
            gpd,
            level: h.security_level,
            key_type: None,
            counter,
            rx_after_tx: h.rx_after_tx,
            temp_master: (h.rx_after_tx && self.direct_bidirectional).then_some(self.own_nwk),
            command_id,
            payload,
            source: Source::Direct,
            now_ms,
        };
        self.process(aes, &rx)
    }

    /// Process a Green Power cluster command sent to the GP server
    /// (client → server direction).
    ///
    /// Returns the ZCL status of commands that report one (GP Sink
    /// Commissioning Mode), otherwise `SUCCESS`, together with what the
    /// frame did. Commands this sink does not implement fail with
    /// [`Error::Unsupported`].
    pub fn on_server_command<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        command: &ServerCommand<'_>,
        now_ms: u32,
    ) -> Result<(u8, Option<SinkOutcome>)> {
        let ServerCommand {
            src_nwk,
            unicast,
            tsn,
            command_id,
            payload,
        } = *command;
        match command_id {
            server_cmd::NOTIFICATION => {
                let n = Notification::decode(payload)?;
                Ok((
                    zcl_status::SUCCESS,
                    Some(self.on_notification(aes, src_nwk, unicast, tsn, &n, now_ms)),
                ))
            }
            server_cmd::COMMISSIONING_NOTIFICATION => {
                let n = CommissioningNotification::decode(payload)?;
                Ok((
                    zcl_status::SUCCESS,
                    Some(self.on_commissioning_notification(aes, &n, now_ms)),
                ))
            }
            server_cmd::SINK_COMMISSIONING_MODE => {
                let m = SinkCommissioningMode::decode(payload)?;
                Ok((self.on_sink_commissioning_mode(&m, now_ms), None))
            }
            server_cmd::SINK_TABLE_REQUEST => {
                let req = SinkTableRequest::decode(payload)?;
                // gecko_sdk answers unicast requests only.
                if unicast {
                    self.queue(Out::SinkTableResponse {
                        dst: src_nwk,
                        tsn,
                        request: req,
                    });
                }
                Ok((zcl_status::SUCCESS, None))
            }
            // Proxy → sink commands with nothing for a sink to do (gecko_sdk
            // handles neither) and the not implemented configuration
            // commands.
            _ => Err(Error::Unsupported),
        }
    }

    /// GP Sink Commissioning Mode (gecko_sdk
    /// `sli_zigbee_af_green_power_server_gp_sink_commissioning_mode_command_handler`).
    fn on_sink_commissioning_mode(&mut self, m: &SinkCommissioningMode, now_ms: u32) -> u8 {
        use sink_commissioning_options::*;
        if m.sink_endpoint != 0xFF && Some(m.sink_endpoint) != self.config.endpoint {
            return zcl_status::NOT_FOUND;
        }
        if m.options & (INVOLVE_GPM_IN_SECURITY | INVOLVE_GPM_IN_PAIRING) != 0
            || self.config.security_level & security_level_attr::INVOLVE_TC != 0
        {
            return zcl_status::UNSUP_COMMAND;
        }
        if m.options & ACTION != 0 {
            if m.gpm_addr_for_security != 0xFFFF || m.gpm_addr_for_pairing != 0xFFFF {
                return zcl_status::INVALID_FIELD;
            }
            if self
                .enter(now_ms, m.options & INVOLVE_PROXIES != 0, false)
                .is_err()
            {
                return zcl_status::UNSUP_COMMAND;
            }
        } else {
            self.exit_commissioning();
        }
        zcl_status::SUCCESS
    }

    /// Whether `src_nwk` was the first proxy to forward (`gpd`, `counter`).
    fn first_forwarder(&mut self, gpd: &GpdId, counter: u32, src_nwk: u16) -> bool {
        if let Some(&(_, _, first)) = self
            .first_forwarders
            .iter()
            .find(|(g, c, _)| g.matches(gpd) && *c == counter)
        {
            return first == src_nwk;
        }
        if self.first_forwarders.is_full() {
            self.first_forwarders.pop_front();
        }
        let _ = self.first_forwarders.push_back((*gpd, counter, src_nwk));
        true
    }

    fn on_notification<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        src_nwk: u16,
        unicast: bool,
        tsn: u8,
        n: &Notification<'_>,
        now_ms: u32,
    ) -> SinkOutcome {
        use SinkDrop::*;
        if n.gpd_id.is_reserved() {
            return SinkOutcome::Dropped(Invalid);
        }
        let idx = self.find(&n.gpd_id);
        // GP Notification Response to a unicast GP Notification: in full
        // unicast, and with NoPairing when this sink has no pairing, so the
        // proxy drops this sink. FirstToForward is set for the proxy whose
        // copy of the GPDF arrived first.
        let full =
            idx.is_some_and(|i| self.table[i].communication_mode == CommunicationMode::FullUnicast);
        if unicast && (full || idx.is_none()) {
            let first = self.first_forwarder(&n.gpd_id, n.frame_counter, src_nwk);
            let r = NotificationResponse {
                gpd_id: n.gpd_id,
                first_to_forward: full && first,
                no_pairing: idx.is_none(),
                frame_counter: n.frame_counter,
            };
            self.queue(Out::NotificationResponse {
                dst: src_nwk,
                tsn,
                response: r,
            });
        }
        let Some(payload) = n.payload else {
            return SinkOutcome::Dropped(Invalid);
        };
        if self
            .dup
            .check_and_record(&n.gpd_id, n.frame_counter, now_ms)
        {
            return SinkOutcome::Dropped(Duplicate);
        }
        let rx = Received {
            gpd: n.gpd_id,
            level: n.security_level,
            key_type: Some(n.key_type),
            counter: n.frame_counter,
            rx_after_tx: n.rx_after_tx,
            temp_master: None,
            command_id: n.command_id,
            payload,
            source: Source::Notification,
            now_ms,
        };
        self.process(aes, &rx)
    }

    fn on_commissioning_notification<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        n: &CommissioningNotification<'_>,
        now_ms: u32,
    ) -> SinkOutcome {
        use SinkDrop::*;
        if self.commissioning.is_none() {
            return SinkOutcome::Dropped(NotCommissioning);
        }
        if n.gpd_id.is_reserved() {
            return SinkOutcome::Dropped(Invalid);
        }
        let Some(raw) = n.payload else {
            return SinkOutcome::Dropped(Invalid);
        };
        let mut clear = GpdPayload::new();
        if let Some(mic) = n.security_failed {
            // The proxy had no key: verify (and decrypt) with the key of the
            // pending commissioning, rebuilding the GPDF header as gecko_sdk
            // `processCommNotificationsWithSecurityProcessingFailedFlag`
            // does.
            let Some(i) = self.pending_index(&n.gpd_id) else {
                return SinkOutcome::Dropped(AuthFailed);
            };
            let pending = &self.pending[i].entry;
            let (Some(key), GpdId::SrcId(src_id)) = (pending.key, n.gpd_id) else {
                return SinkOutcome::Dropped(AuthFailed);
            };
            match open_tunnelled(
                aes,
                &key,
                src_id,
                n.security_level,
                pending.key_type.is_individual(),
                n.rx_after_tx,
                n.frame_counter,
                n.command_id,
                raw,
                mic,
            ) {
                Ok(pt) => clear = pt,
                Err(_) => return SinkOutcome::Dropped(AuthFailed),
            }
        } else if clear.push(n.command_id).is_err() || clear.extend_from_slice(raw).is_err() {
            return SinkOutcome::Dropped(Invalid);
        }
        let Some((&command_id, payload)) = clear.split_first() else {
            return SinkOutcome::Dropped(Invalid);
        };
        if self
            .dup
            .check_and_record(&n.gpd_id, n.frame_counter, now_ms)
        {
            return SinkOutcome::Dropped(Duplicate);
        }
        let temp_master = match n.proxy_info {
            ProxyInfo::Link { gpp_short, .. } | ProxyInfo::Distance { gpp_short, .. } => {
                n.rx_after_tx.then_some(gpp_short)
            }
            ProxyInfo::None => None,
        };
        let rx = Received {
            gpd: n.gpd_id,
            level: n.security_level,
            key_type: Some(n.key_type),
            counter: n.frame_counter,
            rx_after_tx: n.rx_after_tx,
            temp_master,
            command_id,
            payload,
            source: Source::Commissioning,
            now_ms,
        };
        self.process(aes, &rx)
    }

    fn process<P: ForwardAesProvider>(&mut self, aes: &mut P, rx: &Received<'_>) -> SinkOutcome {
        use SinkDrop::*;
        match rx.command_id {
            cmd::COMMISSIONING => {
                // gecko_sdk drops Commissioning tunnelled in a Notification.
                if rx.source == Source::Notification || self.commissioning.is_none() {
                    return SinkOutcome::Dropped(NotCommissioning);
                }
                self.commission(aes, rx)
            }
            cmd::DECOMMISSIONING => {
                let Some(i) = self.find(&rx.gpd) else {
                    return SinkOutcome::Dropped(NoEntry);
                };
                if let Err(drop) = self.check_operational(i, rx) {
                    return SinkOutcome::Dropped(drop);
                }
                self.remove(&rx.gpd);
                SinkOutcome::Decommissioned
            }
            cmd::CHANNEL_REQUEST | cmd::APPLICATION_DESCRIPTION => {
                SinkOutcome::Dropped(Unsupported)
            }
            _ if self.commissioning.is_some() && self.pending_index(&rx.gpd).is_some() => {
                self.complete_pending(rx)
            }
            cmd::SUCCESS => SinkOutcome::Dropped(NoEntry),
            _ => self.operational(rx),
        }
    }

    /// gecko_sdk `gpCommissioningNotificationSuccessGpdf`: Success, or any
    /// other command protected with the new key, completes a bidirectional
    /// commissioning.
    fn complete_pending(&mut self, rx: &Received<'_>) -> SinkOutcome {
        let Some(i) = self.pending_index(&rx.gpd) else {
            return SinkOutcome::Dropped(SinkDrop::NoEntry);
        };
        let pending = &self.pending[i].entry;
        if rx.level != pending.security_level {
            return SinkOutcome::Dropped(SinkDrop::SecurityMismatch);
        }
        if pending.frame_counter > rx.counter {
            return SinkOutcome::Dropped(SinkDrop::Stale);
        }
        let mut entry = self.pending.swap_remove(i).entry;
        entry.frame_counter = rx.counter;
        self.finalize(entry)
    }

    /// Security and freshness of an operational frame against entry `i`
    /// (gecko_sdk `emberAfGreenPowerClusterGpNotificationCallback`). A
    /// directly received frame was already authenticated with the entry's
    /// key; a tunnelled one by the proxy.
    fn check_operational(&self, i: usize, rx: &Received<'_>) -> core::result::Result<(), SinkDrop> {
        let e = &self.table[i];
        if e.security_level.is_protected() {
            let key_mismatch = rx.key_type.is_some_and(|k| k != e.key_type);
            if rx.level < e.security_level || key_mismatch {
                return Err(SinkDrop::SecurityMismatch);
            }
            if rx.counter <= e.frame_counter {
                return Err(SinkDrop::Stale);
            }
        } else if rx.source == Source::Direct && rx.level.is_protected() {
            return Err(SinkDrop::SecurityMismatch);
        }
        Ok(())
    }

    fn operational(&mut self, rx: &Received<'_>) -> SinkOutcome {
        let Some(i) = self.find(&rx.gpd) else {
            return SinkOutcome::Dropped(SinkDrop::NoEntry);
        };
        if let Err(drop) = self.check_operational(i, rx) {
            return SinkOutcome::Dropped(drop);
        }
        let policy = self.counter_policy();
        let e = &mut self.table[i];
        e.frame_counter = rx.counter;
        if e.security_level.is_protected()
            && crate::counter::on_accepted(
                &mut e.persisted_counter,
                &mut e.counter_exact,
                rx.counter,
                crate::counter::is_event_command(rx.command_id),
                &mut self.exact_budget,
                &policy,
                rx.now_ms,
            )
        {
            self.dirty = true;
        }
        let device_id = e.device_id;
        if let (Some(endpoint), Some(translation)) = (
            self.config.endpoint,
            translation::translate(rx.command_id, rx.payload),
        ) {
            if self.commands.is_full() {
                self.commands.pop_front();
            }
            let _ = self.commands.push_back(SinkCommand {
                gpd_id: rx.gpd,
                endpoint,
                translation,
            });
            return SinkOutcome::Translated;
        }
        let Ok(payload) = heapless::Vec::from_slice(rx.payload) else {
            return SinkOutcome::Dropped(SinkDrop::Invalid);
        };
        self.event(SinkEvent::GpdCommand {
            gpd_id: rx.gpd,
            device_id,
            command_id: rx.command_id,
            payload,
        });
        SinkOutcome::PassedThrough
    }

    /// GPD Commissioning (gecko_sdk
    /// `gpCommissioningNotificationCommissioningGpdf`).
    fn commission<P: ForwardAesProvider>(&mut self, aes: &mut P, rx: &Received<'_>) -> SinkOutcome {
        use SinkDrop::*;
        let Ok(p) = CommissioningPayload::decode(rx.payload) else {
            return SinkOutcome::Dropped(Invalid);
        };
        let policy = self.config.security_level;
        if policy & security_level_attr::INVOLVE_TC != 0 {
            return SinkOutcome::Dropped(SecurityPolicy);
        }
        let (level, key_type, wrapped, outgoing) = match &p.security {
            Some(s) => (s.level, s.key_type, s.key, s.outgoing_counter),
            None => (SecurityLevel::None, KeyType::NoKey, None, None),
        };
        if policy & security_level_attr::MINIMAL_LEVEL_MASK > level as u8 {
            return SinkOutcome::Dropped(SecurityPolicy);
        }
        if policy & security_level_attr::PROTECTION_WITH_GP_LINK_KEY != 0
            && wrapped.is_some_and(|(_, mic)| mic.is_none())
        {
            return SinkOutcome::Dropped(SecurityPolicy);
        }
        let key = match wrapped {
            Some((key, Some(mic))) => {
                let wrapped = WrappedKey {
                    key,
                    mic: mic.to_le_bytes(),
                };
                match security::unwrap_key(
                    aes,
                    &DEFAULT_TC_LINK_KEY,
                    &rx.gpd,
                    Direction::FromGpd,
                    0,
                    &wrapped,
                ) {
                    Ok(key) => Some(key),
                    Err(_) => return SinkOutcome::Dropped(AuthFailed),
                }
            }
            Some((key, None)) => Some(key),
            None => None,
        };
        let protected = level.is_protected();
        if protected && key.is_none() {
            return SinkOutcome::Dropped(NoKey);
        }
        if rx.rx_after_tx
            && p.app_info
                .as_ref()
                .is_some_and(|a| a.app_description_follows)
        {
            return SinkOutcome::Dropped(Unsupported);
        }
        let mode = self.config.communication_mode;
        let mut entry = SinkEntry::new(rx.gpd);
        entry.communication_mode = mode;
        entry.mac_seq_cap = p.mac_seq_num_capability;
        entry.rx_on_cap = p.rx_on_capability;
        entry.fixed_location = p.fixed_location;
        entry.device_id = p.device_id;
        entry.groupcast_radius = self.config.groupcast_radius.unwrap_or(0);
        if protected {
            entry.security_level = level;
            entry.key_type = key_type;
            entry.key = key;
        }
        // The GPD's outgoing counter, not the commissioning frame's.
        entry.frame_counter = match outgoing {
            Some(c) if protected => c,
            _ => rx.counter,
        };
        if mode == CommunicationMode::CommissionedGroupcast {
            let Some(group) = self.config.commissioned_group else {
                return SinkOutcome::Dropped(Unsupported);
            };
            let _ = entry.groups.push(SinkGroup { group, alias: None });
        }
        if !rx.rx_after_tx {
            return self.finalize(entry);
        }
        // Bidirectional: GP Response with a Commissioning Reply through the
        // TempMaster (gecko_sdk `sendCommissioningReply`); the pairing
        // completes on the GPD's Success.
        let Some(temp_master) = rx.temp_master else {
            return SinkOutcome::Dropped(NoTempMaster);
        };
        if !(11..=26).contains(&self.operational_channel) {
            return SinkOutcome::Dropped(NoTempMaster);
        }
        let reply = CommissioningReply {
            pan_id: p.pan_id_request.then_some(self.pan_id),
            security_level: level,
            key_type,
            key: None,
        };
        let mut reply_buf = [0u8; MAX_REPLY_LEN];
        let Ok(reply_len) = reply.encode(&mut reply_buf) else {
            return SinkOutcome::Dropped(Invalid);
        };
        if let Some(i) = self.pending_index(&rx.gpd) {
            self.pending.swap_remove(i);
        }
        if self.pending.is_full() {
            self.pending.remove(0);
        }
        if !self.queue(Out::Response {
            gpd_id: rx.gpd,
            temp_master,
            tx_channel: self.operational_channel - 11,
            reply: reply_buf,
            reply_len: reply_len as u8,
        }) {
            return SinkOutcome::Dropped(QueueFull);
        }
        let _ = self.pending.push(Pending { entry });
        SinkOutcome::CommissioningReplySent
    }

    /// GP Pairing (AddSink) for a Sink Table entry (gecko_sdk
    /// `sendGpPairingFromSinkEntry`).
    fn pairing(&self, e: &SinkEntry) -> Pairing {
        let protected = e.security_level.is_protected();
        let sink = match e.communication_mode {
            CommunicationMode::DerivedGroupcast => PairingSink::Group(derive_alias(&e.gpd_id)),
            CommunicationMode::CommissionedGroupcast => {
                PairingSink::Group(e.groups.first().map_or(0xFFFF, |g| g.group))
            }
            _ => PairingSink::Unicast {
                ieee: self.own_ieee,
                nwk: self.own_nwk,
            },
        };
        let groupcast = matches!(sink, PairingSink::Group(_));
        Pairing {
            gpd_id: e.gpd_id,
            add_sink: true,
            remove_gpd: false,
            communication_mode: e.communication_mode,
            gpd_fixed: e.fixed_location,
            mac_seq_cap: e.mac_seq_cap,
            security_level: e.security_level,
            key_type: e.key_type,
            sink,
            device_id: Some(e.device_id),
            frame_counter: (e.mac_seq_cap || protected).then_some(e.frame_counter),
            key: (protected && e.key_type != KeyType::NoKey)
                .then_some(e.key)
                .flatten(),
            assigned_alias: e.assigned_alias,
            groupcast_radius: (groupcast && e.groupcast_radius != 0).then_some(e.groupcast_radius),
        }
    }

    fn release_groups(&mut self, groups: &[u16]) {
        for &g in groups {
            if !self.table.iter().any(|e| e.group_ids().contains(&g)) {
                let _ = self.group_changes.push_back(GroupChange::Leave(g));
            }
        }
    }

    /// Store a commissioned GPD, join its groups, announce the pairing
    /// (gecko_sdk `handleSinkEntryAndPairing`, `sendGpPairingFromSinkEntry`,
    /// `handleClosingCommissioningSessionOnFirstPairing`).
    fn finalize(&mut self, mut entry: SinkEntry) -> SinkOutcome {
        let gpd_id = entry.gpd_id;
        let device_id = entry.device_id;
        let new_groups = entry.group_ids();
        let announce = (entry.communication_mode != CommunicationMode::LightweightUnicast)
            .then(|| entry.alias());
        entry.persisted_counter = entry.frame_counter;
        // A new entry has accepted nothing yet: its counter is exact.
        entry.counter_exact = self.config.exact_counter_burst != 0;
        let old_groups = match self.find(&gpd_id) {
            Some(i) => {
                // A repeated commissioning that changes nothing but the
                // counter follows the persistence stride; a lower counter
                // (a reset GPD) is always written.
                let stride = self.config.counter_persist_stride.max(1);
                let old = &mut self.table[i];
                let structural = !old.same_except_counter(&entry);
                let counter_due = entry.frame_counter < old.persisted_counter
                    || entry.frame_counter.wrapping_sub(old.persisted_counter) >= stride;
                if structural || counter_due {
                    self.dirty = true;
                } else {
                    entry.persisted_counter = old.persisted_counter;
                    entry.counter_exact = old.counter_exact;
                }
                let old_groups = old.group_ids();
                *old = entry;
                old_groups
            }
            None => {
                if self.table.push(entry).is_err() {
                    self.event(SinkEvent::CommissioningFailed {
                        gpd_id,
                        reason: SinkDrop::TableFull,
                    });
                    return SinkOutcome::Dropped(SinkDrop::TableFull);
                }
                self.dirty = true;
                heapless::Vec::new()
            }
        };
        for g in &new_groups {
            if !old_groups.contains(g) {
                let _ = self.group_changes.push_back(GroupChange::Join(*g));
            }
        }
        for &g in &old_groups {
            if !new_groups.contains(&g) {
                self.release_groups(&[g]);
            }
        }
        self.queue(Out::PairingAdd(gpd_id));
        // A sink that added a pairing announces the alias (except in
        // lightweight unicast).
        if let Some(alias) = announce {
            self.queue(Out::DeviceAnnce(alias));
        }
        self.event(SinkEvent::Commissioned { gpd_id, device_id });
        if self.config.exit_mode & exit_mode_attr::ON_FIRST_PAIRING != 0 {
            self.leave_commissioning(SinkCommissioningExit::FirstPairing);
        }
        SinkOutcome::Commissioned
    }

    /// Remove a GPD from the Sink Table and broadcast GP Pairing with
    /// RemoveGPD (gecko_sdk `decommissionGpd`). Returns whether it was
    /// paired.
    pub fn remove(&mut self, gpd: &GpdId) -> bool {
        let Some(i) = self.find(gpd) else {
            return false;
        };
        let e = self.table.swap_remove(i);
        self.dup.forget(&e.gpd_id);
        self.release_groups(&e.group_ids());
        self.queue(Out::PairingRemove {
            gpd_id: e.gpd_id,
            mode: e.communication_mode,
            level: e.security_level,
            key_type: e.key_type,
        });
        self.dirty = true;
        self.event(SinkEvent::Decommissioned { gpd_id: e.gpd_id });
        true
    }

    /// GP Sink Table Response payload (gecko_sdk
    /// `emberAfGreenPowerClusterGpSinkTableRequestCallback`): status, total,
    /// start index, count, then the entries that fit one frame.
    fn sink_table_response(&self, req: &SinkTableRequest, out: &mut [u8]) -> Result<usize> {
        let (status, start, slice): (u8, u8, &[SinkEntry]) = match *req {
            SinkTableRequest::ByGpdId(id) => match self.find(&id) {
                Some(i) if !id.is_reserved() => (status::SUCCESS, 0xFF, &self.table[i..=i]),
                _ => (status::NOT_FOUND, 0xFF, &[]),
            },
            SinkTableRequest::ByIndex(i) if (i as usize) < self.table.len() => {
                (status::SUCCESS, i, &self.table[i as usize..])
            }
            SinkTableRequest::ByIndex(i) => (status::NOT_FOUND, i, &[]),
        };
        let mut w = Writer::new(out);
        w.bytes(&[status, self.table.len() as u8, start, 0])?;
        let mut count = 0u8;
        for e in slice {
            let mut tmp = [0u8; MAX_SINK_ENTRY_ENCODED_LEN];
            let n = e.encode(&mut tmp)?;
            if w.len() + n > MAX_TX_PAYLOAD {
                break;
            }
            w.bytes(&tmp[..n])?;
            count += 1;
        }
        let len = w.len();
        out[3] = count;
        Ok(len)
    }
}

/// Verify and decrypt a GPD command a proxy tunnelled with "security
/// processing failed": rebuild the GPDF NWK header (data frame, Extended
/// NWK FC, SrcID, counter; gecko_sdk `green-power-security.c`
/// `prepareHeader`) and open it like a received GPDF.
#[allow(clippy::too_many_arguments)]
fn open_tunnelled<P: ForwardAesProvider>(
    aes: &mut P,
    key: &AesKey,
    src_id: u32,
    level: SecurityLevel,
    individual_key: bool,
    rx_after_tx: bool,
    frame_counter: u32,
    command_id: u8,
    payload: &[u8],
    mic: u32,
) -> Result<GpdPayload> {
    if !level.is_protected() {
        return Err(Error::Unsupported);
    }
    let params = GpdfParams {
        frame_type: FrameType::Data,
        auto_commissioning: false,
        src_id: Some(src_id),
        security_level: level,
        individual_key,
        rx_after_tx,
        to_gpd: false,
        frame_counter,
    };
    let mut header = [0u8; 10];
    let header_len = write_header(&mut header, &params)?;
    let mut body = GpdPayload::new();
    body.push(command_id).map_err(|_| Error::BufferFull)?;
    body.extend_from_slice(payload)
        .map_err(|_| Error::BufferFull)?;
    let frame = Gpdf {
        header: GpdfHeader {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            ext_present: true,
            app_id: ApplicationId::SrcId,
            security_level: level,
            individual_key,
            rx_after_tx,
            to_gpd: false,
            gpd_id: Some(GpdId::SrcId(src_id)),
            frame_counter,
        },
        mac_seq: 0,
        raw_header: &header[..header_len],
        payload: &body,
        mic: Some(mic.to_le_bytes()),
    };
    const _: () = assert!(MIC_LEN == 4);
    security::open(aes, key, &frame, Direction::FromGpd)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cluster::GppGpdLink;
    use crate::commissioning::{ApplicationInfo, CommissioningSecurity};
    use crate::frame::{parse, parse_mac};
    use zigbee_crypto::SoftwareAesProvider;

    const KEY: AesKey = [
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE,
        0xCF,
    ];
    const SRC: u32 = 0x1234_5678;
    const GPD: GpdId = GpdId::SrcId(SRC);
    const OWN: u16 = 0x4A76;
    const OWN_IEEE: IeeeAddress = [1, 2, 3, 4, 5, 6, 7, 8];
    const ENDPOINT: u8 = 1;

    type S = Sink<4, 4>;

    fn sink(config: SinkConfig) -> S {
        let mut s = S::new(config);
        s.set_own_address(OWN, OWN_IEEE, 0x1A62);
        s.set_operational_channel(15);
        s
    }

    fn default_sink() -> S {
        sink(SinkConfig {
            endpoint: Some(ENDPOINT),
            ..SinkConfig::default()
        })
    }

    /// A GPDF as received: MAC frame built, then parsed like the runtime.
    struct Frame {
        raw: [u8; 127],
        len: usize,
    }

    impl Frame {
        fn gpdf(&self) -> Gpdf<'_> {
            let mac = parse_mac(&self.raw[..self.len]).unwrap();
            parse(&self.raw[mac.payload_offset..self.len], mac.seq, None).unwrap()
        }
    }

    fn frame(
        level: SecurityLevel,
        counter: u32,
        rx_after_tx: bool,
        command: u8,
        payload: &[u8],
    ) -> Frame {
        let mut raw = [0u8; 127];
        let params = GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(SRC),
            security_level: level,
            individual_key: level.is_protected(),
            rx_after_tx,
            to_gpd: false,
            frame_counter: counter,
        };
        let len = security::build_gpdf(
            &mut SoftwareAesProvider,
            level.is_protected().then_some(&KEY),
            &params,
            counter as u8,
            command,
            payload,
            &mut raw,
        )
        .unwrap();
        Frame { raw, len }
    }

    fn commissioning_payload(counter: u32, wrapped: bool) -> heapless::Vec<u8, 64> {
        let key = if wrapped {
            let w = security::wrap_key(
                &mut SoftwareAesProvider,
                &DEFAULT_TC_LINK_KEY,
                &GPD,
                Direction::FromGpd,
                0,
                &KEY,
            )
            .unwrap();
            (w.key, Some(u32::from_le_bytes(w.mic)))
        } else {
            (KEY, None)
        };
        let p = CommissioningPayload {
            device_id: 0x02,
            mac_seq_num_capability: true,
            rx_on_capability: false,
            pan_id_request: false,
            security_key_request: false,
            fixed_location: true,
            security: Some(CommissioningSecurity {
                level: SecurityLevel::Encrypted,
                key_type: KeyType::IndividualKey,
                key: Some(key),
                outgoing_counter: Some(counter),
            }),
            app_info: Some(ApplicationInfo {
                manufacturer_id: Some(0x6D6B),
                model_id: Some(0xF400),
                ..ApplicationInfo::default()
            }),
        };
        let mut b = [0u8; 64];
        let n = p.encode(&mut b).unwrap();
        heapless::Vec::from_slice(&b[..n]).unwrap()
    }

    fn command(tx: &GpTx) -> (u8, &[u8]) {
        match &tx.frame {
            TxFrame::GpCommand {
                command_id,
                payload,
            } => (*command_id, &payload[..]),
            other => panic!("unexpected frame {other:?}"),
        }
    }

    fn drain(s: &mut S, now: u32) -> heapless::Vec<GpTx, 8> {
        let mut out = heapless::Vec::new();
        while let Some(tx) = s.poll(now) {
            out.push(tx.tx).unwrap();
        }
        out
    }

    /// Commission the test GPD directly (unprotected Commissioning GPDF
    /// carrying a TC-LK wrapped key and outgoing counter 10).
    fn commissioned() -> S {
        let mut s = default_sink();
        s.enter_commissioning(0, true).unwrap();
        drain(&mut s, 0);
        let f = frame(
            SecurityLevel::None,
            1,
            false,
            cmd::COMMISSIONING,
            &commissioning_payload(10, true),
        );
        assert_eq!(
            s.on_gpdf(&mut SoftwareAesProvider, &f.gpdf(), 100),
            SinkOutcome::Commissioned
        );
        s
    }

    #[test]
    fn sink_entry_round_trips_and_locates_its_counter() {
        let mut protected = SinkEntry::new(GPD);
        protected.security_level = SecurityLevel::Encrypted;
        protected.key_type = KeyType::IndividualKey;
        protected.key = Some(KEY);
        protected.frame_counter = 0x0102_0304;
        protected.persisted_counter = protected.frame_counter;
        protected.device_id = 0x02;
        protected.mac_seq_cap = true;
        let mut grouped = SinkEntry::new(GpdId::Ieee {
            ieee: [9; 8],
            endpoint: 3,
        });
        grouped.communication_mode = CommunicationMode::CommissionedGroupcast;
        grouped
            .groups
            .push(SinkGroup {
                group: 0x0B84,
                alias: None,
            })
            .unwrap();
        grouped
            .groups
            .push(SinkGroup {
                group: 0x1000,
                alias: Some(0x2222),
            })
            .unwrap();
        grouped.assigned_alias = Some(0x3333);
        grouped.groupcast_radius = 7;
        grouped.mac_seq_cap = true;
        grouped.frame_counter = 0x55;
        grouped.persisted_counter = 0x55;
        for e in [protected, grouped, SinkEntry::new(GpdId::SrcId(7))] {
            let mut b = [0u8; MAX_SINK_ENTRY_ENCODED_LEN];
            let n = e.encode(&mut b).unwrap();
            assert_eq!(SinkEntry::decode(&b[..n]).unwrap(), (e.clone(), n));
            let (id, counter) = SinkEntry::encoded_layout(&b[..n]).unwrap();
            assert_eq!(id.len(), e.gpd_id.wire_len());
            match counter {
                Some(at) => {
                    assert!(e.security_level.is_protected());
                    assert_eq!(&b[at..at + 4], &e.frame_counter.to_le_bytes());
                }
                None => assert!(!e.security_level.is_protected()),
            }
        }
    }

    /// The gecko_sdk `storeSinkTableEntryInBuffer` layout, byte by byte.
    #[test]
    fn sink_entry_matches_the_silabs_layout() {
        let mut e = SinkEntry::new(GPD);
        e.communication_mode = CommunicationMode::CommissionedGroupcast;
        e.groups
            .push(SinkGroup {
                group: 0x0B84,
                alias: None,
            })
            .unwrap();
        e.security_level = SecurityLevel::Encrypted;
        e.key_type = KeyType::IndividualKey;
        e.key = Some(KEY);
        e.frame_counter = 16;
        e.device_id = 0xFE;
        e.mac_seq_cap = true;
        e.fixed_location = true;
        let mut b = [0u8; MAX_SINK_ENTRY_ENCODED_LEN];
        let n = e.encode(&mut b).unwrap();
        let mut expected = heapless::Vec::<u8, 64>::new();
        // Options: ApplicationID 0, commissioned groupcast (0b10 << 3),
        // sequence number capability, fixed location, security use.
        expected.extend_from_slice(&[0xB0, 0x02]).unwrap();
        expected.extend_from_slice(&SRC.to_le_bytes()).unwrap();
        expected.push(0xFE).unwrap(); // DeviceID
        expected
            .extend_from_slice(&[1, 0x84, 0x0B, 0xFF, 0xFF])
            .unwrap();
        expected.push(0).unwrap(); // groupcast radius
        expected.push(0x13).unwrap(); // level 3, key type 0b100
        expected.extend_from_slice(&16u32.to_le_bytes()).unwrap();
        expected.extend_from_slice(&KEY).unwrap();
        assert_eq!(&b[..n], &expected[..]);
        // gecko_sdk sets SecurityUse for level 0 GPDs that sent extended
        // options: the frame counter (sequence number capability) follows,
        // no key.
        let silabs_level0 = [
            0x28, 0x02, 0x78, 0x56, 0x34, 0x12, 0x02, 0x00, 0x00, 9, 0, 0, 0,
        ];
        let (d, used) = SinkEntry::decode(&silabs_level0).unwrap();
        assert_eq!(used, silabs_level0.len());
        assert_eq!(d.security_level, SecurityLevel::None);
        assert_eq!(d.frame_counter, 9);
        assert_eq!(d.key, None);
    }

    #[test]
    fn a_local_window_times_out_even_when_exiting_on_first_pairing() {
        // The default exit mode is "on first pairing" only (gecko_sdk).
        let mut s = default_sink();
        assert_eq!(s.config().exit_mode, exit_mode_attr::ON_FIRST_PAIRING);
        s.enter_commissioning(1000, true).unwrap();
        let tx = drain(&mut s, 1000);
        // The proxies are told to exit on the first pairing, no window.
        assert_eq!(command(&tx[0]).0, client_cmd::PROXY_COMMISSIONING_MODE);
        assert_eq!(
            s.take_event(),
            Some(SinkEvent::CommissioningEntered {
                window_s: Some(180),
                involve_proxies: true
            })
        );
        assert_eq!(s.next_deadline(), Some(181_000));
        drain(&mut s, 180_999);
        assert!(s.in_commissioning_mode());
        // The window elapses: the sink leaves and tells the proxies, which
        // had no window of their own.
        let tx = drain(&mut s, 181_000);
        assert!(!s.in_commissioning_mode());
        assert_eq!(
            s.take_event(),
            Some(SinkEvent::CommissioningLeft(SinkCommissioningExit::Timeout))
        );
        assert_eq!(tx.len(), 1);
        let (id, payload) = command(&tx[0]);
        assert_eq!(id, client_cmd::PROXY_COMMISSIONING_MODE);
        assert_eq!(payload[0] & 0x01, 0, "exit");
        // GP Sink Commissioning Mode from the network keeps the exit mode:
        // no window.
        let mut aes = SoftwareAesProvider;
        let enter = SinkCommissioningMode {
            options: sink_commissioning_options::ACTION
                | sink_commissioning_options::INVOLVE_PROXIES,
            gpm_addr_for_security: 0xFFFF,
            gpm_addr_for_pairing: 0xFFFF,
            sink_endpoint: 0xFF,
        };
        let mut b = [0u8; 8];
        let n = enter.encode(&mut b).unwrap();
        s.on_server_command(
            &mut aes,
            &ServerCommand {
                src_nwk: 0,
                unicast: true,
                tsn: 1,
                command_id: server_cmd::SINK_COMMISSIONING_MODE,
                payload: &b[..n],
            },
            200_000,
        )
        .unwrap();
        assert!(s.in_commissioning_mode());
        assert_eq!(s.next_deadline(), None);
    }

    #[test]
    fn commissioning_mode_drives_the_proxies() {
        let mut s = sink(SinkConfig {
            exit_mode: exit_mode_attr::ON_WINDOW_EXPIRATION
                | exit_mode_attr::ON_PROXY_COMMISSIONING_MODE_EXIT,
            ..SinkConfig::default()
        });
        s.enter_commissioning(1000, true).unwrap();
        let tx = drain(&mut s, 1000);
        assert_eq!(tx.len(), 1);
        assert_eq!(tx[0].dest, DestAddr::Broadcast(BROADCAST_RX_ON_WHEN_IDLE));
        assert_eq!(
            command(&tx[0]),
            (
                client_cmd::PROXY_COMMISSIONING_MODE,
                &[0x0B, 0xB4, 0x00][..]
            )
        );
        assert_eq!(
            s.take_event(),
            Some(SinkEvent::CommissioningEntered {
                window_s: Some(180),
                involve_proxies: true
            })
        );
        assert_eq!(s.next_deadline(), Some(181_000));
        // The window elapses; the proxies time out on their own window.
        assert!(s.poll(180_999).is_none());
        assert!(s.in_commissioning_mode());
        assert!(s.poll(181_000).is_none());
        assert!(!s.in_commissioning_mode());
        assert_eq!(
            s.take_event(),
            Some(SinkEvent::CommissioningLeft(SinkCommissioningExit::Timeout))
        );
        // An explicit exit tells the proxies.
        s.enter_commissioning(0, true).unwrap();
        drain(&mut s, 0);
        s.exit_commissioning();
        let tx = drain(&mut s, 1);
        assert_eq!(
            command(&tx[0]),
            (client_cmd::PROXY_COMMISSIONING_MODE, &[0x08][..])
        );
        // Without involving proxies nothing is broadcast.
        s.enter_commissioning(0, false).unwrap();
        assert!(drain(&mut s, 0).is_empty());
    }

    #[test]
    fn direct_commissioning_stores_entry_pairs_and_announces_the_alias() {
        let mut s = commissioned();
        let e = &s.entries()[0];
        assert_eq!(e.gpd_id, GPD);
        assert_eq!(e.key, Some(KEY));
        assert_eq!(e.frame_counter, 10);
        assert_eq!(e.device_id, 0x02);
        assert_eq!(e.communication_mode, CommunicationMode::DerivedGroupcast);
        assert!(s.take_dirty());
        assert_eq!(
            s.take_group_change(),
            Some(GroupChange::Join(derive_alias(&GPD)))
        );
        // Exit on first pairing (default exit mode): PCM exit follows the
        // Pairing and the alias Device_annce.
        assert!(!s.in_commissioning_mode());
        let tx = drain(&mut s, 100);
        assert_eq!(tx.len(), 3);
        let (id, payload) = command(&tx[0]);
        assert_eq!(id, client_cmd::PAIRING);
        let p = Pairing::decode(payload).unwrap();
        assert!(p.add_sink);
        assert_eq!(p.communication_mode, CommunicationMode::DerivedGroupcast);
        assert_eq!(p.sink, PairingSink::Group(derive_alias(&GPD)));
        assert_eq!(p.key, Some(KEY));
        assert_eq!(p.frame_counter, Some(10));
        assert_eq!(p.security_level, SecurityLevel::Encrypted);
        assert_eq!(p.key_type, KeyType::IndividualKey);
        assert!(p.gpd_fixed && p.mac_seq_cap);
        assert_eq!(
            tx[1].frame,
            TxFrame::DeviceAnnce {
                alias: derive_alias(&GPD)
            }
        );
        assert_eq!(tx[1].alias, Some((derive_alias(&GPD), 0)));
        assert_eq!(command(&tx[2]).0, client_cmd::PROXY_COMMISSIONING_MODE);
        assert!(matches!(
            s.take_event(),
            Some(SinkEvent::CommissioningEntered { .. })
        ));
        assert_eq!(
            s.take_event(),
            Some(SinkEvent::Commissioned {
                gpd_id: GPD,
                device_id: 0x02
            })
        );
    }

    #[test]
    fn secured_toggle_is_translated_and_replays_are_rejected() {
        let mut s = commissioned();
        let mut aes = SoftwareAesProvider;
        let toggle = frame(SecurityLevel::Encrypted, 11, false, 0x22, &[]);
        assert_eq!(
            s.on_gpdf(&mut aes, &toggle.gpdf(), 200),
            SinkOutcome::Translated
        );
        let c = s.take_command().unwrap();
        assert_eq!((c.gpd_id, c.endpoint), (GPD, ENDPOINT));
        assert_eq!(c.translation.cluster_id, translation::ON_OFF_CLUSTER);
        assert_eq!(c.translation.command_id, 0x02);
        // The second copy (GPDs send each frame twice) is a duplicate.
        assert_eq!(
            s.on_gpdf(&mut aes, &toggle.gpdf(), 300),
            SinkOutcome::Dropped(SinkDrop::Duplicate)
        );
        // After the duplicate window it is stale.
        assert_eq!(
            s.on_gpdf(&mut aes, &toggle.gpdf(), 5000),
            SinkOutcome::Dropped(SinkDrop::Stale)
        );
        // A tampered frame fails the MIC.
        let mut bad = frame(SecurityLevel::Encrypted, 12, false, 0x22, &[]);
        bad.raw[bad.len - 1] ^= 1;
        assert_eq!(
            s.on_gpdf(&mut aes, &bad.gpdf(), 6000),
            SinkOutcome::Dropped(SinkDrop::AuthFailed)
        );
        // An unprotected frame for a protected entry is refused.
        let clear = frame(SecurityLevel::None, 13, false, 0x22, &[]);
        assert_eq!(
            s.on_gpdf(&mut aes, &clear.gpdf(), 7000),
            SinkOutcome::Dropped(SinkDrop::SecurityMismatch)
        );
        assert!(s.take_command().is_none());
        // Reports have no translation and reach the application.
        let report = frame(
            SecurityLevel::Encrypted,
            14,
            false,
            0xA0,
            &[0x02, 0x04, 0x00, 0x00, 0x29, 0x66, 0x08],
        );
        assert_eq!(
            s.on_gpdf(&mut aes, &report.gpdf(), 8000),
            SinkOutcome::PassedThrough
        );
        let Some(SinkEvent::GpdCommand {
            command_id,
            payload,
            ..
        }) = core::iter::from_fn(|| s.take_event()).last()
        else {
            panic!()
        };
        assert_eq!(command_id, 0xA0);
        assert_eq!(payload.len(), 7);
    }

    fn notification(counter: u32, command: u8, proxy_info: ProxyInfo) -> heapless::Vec<u8, 64> {
        let n = Notification {
            gpd_id: GPD,
            also_unicast: false,
            also_derived_group: true,
            also_commissioned_group: false,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::IndividualKey,
            rx_after_tx: false,
            tx_queue_full: false,
            bidirectional_cap: false,
            frame_counter: counter,
            command_id: command,
            payload: Some(&[]),
            proxy_info,
        };
        let mut b = [0u8; 64];
        let len = n.encode(&mut b).unwrap();
        heapless::Vec::from_slice(&b[..len]).unwrap()
    }

    #[test]
    fn notifications_from_proxies_are_translated_once() {
        let mut s = commissioned();
        drain(&mut s, 100);
        let mut aes = SoftwareAesProvider;
        let on = notification(20, 0x21, ProxyInfo::None);
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x1111,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::NOTIFICATION,
                    payload: &on
                },
                1000
            ),
            Ok((zcl_status::SUCCESS, Some(SinkOutcome::Translated)))
        );
        assert_eq!(s.take_command().unwrap().translation.command_id, 0x01);
        // A second proxy's copy, and the GPDF heard directly, are dropped.
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x2222,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::NOTIFICATION,
                    payload: &on
                },
                1010
            ),
            Ok((
                zcl_status::SUCCESS,
                Some(SinkOutcome::Dropped(SinkDrop::Duplicate))
            ))
        );
        let direct = frame(SecurityLevel::Encrypted, 20, false, 0x21, &[]);
        assert_eq!(
            s.on_gpdf(&mut aes, &direct.gpdf(), 1020),
            SinkOutcome::Dropped(SinkDrop::Duplicate)
        );
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x2222,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::NOTIFICATION,
                    payload: &on
                },
                9000
            ),
            Ok((
                zcl_status::SUCCESS,
                Some(SinkOutcome::Dropped(SinkDrop::Stale))
            ))
        );
        assert!(s.take_command().is_none());
        // Unknown GPDs and the counter floor.
        assert_eq!(s.entries()[0].frame_counter, 20);
        // Commissioning tunnelled in a Notification is dropped.
        let c = notification(21, cmd::COMMISSIONING, ProxyInfo::None);
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x1111,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::NOTIFICATION,
                    payload: &c
                },
                9100
            ),
            Ok((
                zcl_status::SUCCESS,
                Some(SinkOutcome::Dropped(SinkDrop::NotCommissioning))
            ))
        );
        assert!(drain(&mut s, 9100).is_empty());
    }

    #[test]
    fn unicast_notifications_get_a_notification_response() {
        let mut s = commissioned();
        drain(&mut s, 100);
        let mut aes = SoftwareAesProvider;
        // Not paired in full unicast: no response.
        let n = notification(30, 0x21, ProxyInfo::None);
        s.on_server_command(
            &mut aes,
            &ServerCommand {
                src_nwk: 0x1111,
                unicast: true,
                tsn: 0x42,
                command_id: server_cmd::NOTIFICATION,
                payload: &n,
            },
            1000,
        )
        .unwrap();
        assert!(drain(&mut s, 1000).is_empty());
        s.table[0].communication_mode = CommunicationMode::FullUnicast;
        let n = notification(31, 0x21, ProxyInfo::None);
        for (proxy, first) in [(0x1111, true), (0x2222, false), (0x1111, true)] {
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: proxy,
                    unicast: true,
                    tsn: 0x42,
                    command_id: server_cmd::NOTIFICATION,
                    payload: &n,
                },
                2000,
            )
            .unwrap();
            let tx = drain(&mut s, 2000);
            assert_eq!(
                tx[0].dest,
                DestAddr::Unicast {
                    nwk: proxy,
                    ieee: None
                }
            );
            let (id, payload) = command(&tx[0]);
            assert_eq!(id, client_cmd::NOTIFICATION_RESPONSE);
            let r = NotificationResponse::decode(payload).unwrap();
            assert_eq!(
                (r.first_to_forward, r.no_pairing, r.frame_counter),
                (first, false, 31)
            );
        }
        // A GPD this sink has no pairing for: NoPairing.
        s.remove(&GPD);
        drain(&mut s, 3000);
        let n = notification(32, 0x21, ProxyInfo::None);
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x1111,
                    unicast: true,
                    tsn: 0x42,
                    command_id: server_cmd::NOTIFICATION,
                    payload: &n
                },
                3000
            ),
            Ok((
                zcl_status::SUCCESS,
                Some(SinkOutcome::Dropped(SinkDrop::NoEntry))
            ))
        );
        let tx = drain(&mut s, 3000);
        let r = NotificationResponse::decode(command(&tx[0]).1).unwrap();
        assert!(r.no_pairing && !r.first_to_forward);
    }

    fn commissioning_notification(
        counter: u32,
        rx_after_tx: bool,
        command: u8,
        payload: &[u8],
        security_failed: Option<u32>,
    ) -> heapless::Vec<u8, 96> {
        let n = CommissioningNotification {
            gpd_id: GPD,
            rx_after_tx,
            security_level: if security_failed.is_some() {
                SecurityLevel::Encrypted
            } else {
                SecurityLevel::None
            },
            key_type: if security_failed.is_some() {
                KeyType::IndividualKey
            } else {
                KeyType::NoKey
            },
            security_failed,
            bidirectional_cap: true,
            frame_counter: counter,
            command_id: command,
            payload: Some(payload),
            proxy_info: ProxyInfo::Link {
                gpp_short: 0x1111,
                link: GppGpdLink(0xC0),
            },
        };
        let mut b = [0u8; 96];
        let len = n.encode(&mut b).unwrap();
        heapless::Vec::from_slice(&b[..len]).unwrap()
    }

    #[test]
    fn commissioning_notification_commissions_outside_radio_range() {
        let mut s = default_sink();
        let mut aes = SoftwareAesProvider;
        let cn = commissioning_notification(
            3,
            false,
            cmd::COMMISSIONING,
            &commissioning_payload(10, false),
            None,
        );
        // Outside commissioning mode nothing happens.
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x1111,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::COMMISSIONING_NOTIFICATION,
                    payload: &cn
                },
                0
            ),
            Ok((
                zcl_status::SUCCESS,
                Some(SinkOutcome::Dropped(SinkDrop::NotCommissioning))
            ))
        );
        s.enter_commissioning(0, false).unwrap();
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x1111,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::COMMISSIONING_NOTIFICATION,
                    payload: &cn
                },
                10
            ),
            Ok((zcl_status::SUCCESS, Some(SinkOutcome::Commissioned)))
        );
        assert_eq!(s.entries()[0].key, Some(KEY));
    }

    /// Bidirectional commissioning through a TempMaster: GP Response with a
    /// Commissioning Reply, then the GPD's Success, tunnelled with
    /// "security processing failed" because the proxy has no key yet.
    #[test]
    fn bidirectional_commissioning_completes_on_tunnelled_success() {
        let mut s = default_sink();
        let mut aes = SoftwareAesProvider;
        s.enter_commissioning(0, false).unwrap();
        let cn = commissioning_notification(
            3,
            true,
            cmd::COMMISSIONING,
            &commissioning_payload(10, true),
            None,
        );
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x1111,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::COMMISSIONING_NOTIFICATION,
                    payload: &cn
                },
                10
            ),
            Ok((
                zcl_status::SUCCESS,
                Some(SinkOutcome::CommissioningReplySent)
            ))
        );
        assert!(s.entries().is_empty());
        let tx = drain(&mut s, 10);
        assert_eq!(tx[0].dest, DestAddr::Broadcast(BROADCAST_RX_ON_WHEN_IDLE));
        let (id, payload) = command(&tx[0]);
        assert_eq!(id, client_cmd::RESPONSE);
        let r = Response::decode(payload).unwrap();
        assert_eq!(r.temp_master_short, 0x1111);
        assert_eq!(r.tx_channel(), 15);
        assert_eq!(r.command_id, cmd::COMMISSIONING_REPLY);
        // zigpy's reply for an L3 individual-key GPD (no key, no PAN ID).
        assert_eq!(r.payload, &[0x98]);

        // Success (0xE2), protected with the GPD's key, counter 11; the
        // proxy forwards it encrypted with its MIC.
        let success = frame(SecurityLevel::Encrypted, 11, false, cmd::SUCCESS, &[]);
        let f = success.gpdf();
        let mic = u32::from_le_bytes(f.mic.unwrap());
        let mut bad =
            commissioning_notification(11, false, f.payload[0], &f.payload[1..], Some(mic ^ 1));
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x1111,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::COMMISSIONING_NOTIFICATION,
                    payload: &bad
                },
                20
            ),
            Ok((
                zcl_status::SUCCESS,
                Some(SinkOutcome::Dropped(SinkDrop::AuthFailed))
            ))
        );
        bad = commissioning_notification(11, false, f.payload[0], &f.payload[1..], Some(mic));
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x1111,
                    unicast: false,
                    tsn: 0x42,
                    command_id: server_cmd::COMMISSIONING_NOTIFICATION,
                    payload: &bad
                },
                30
            ),
            Ok((zcl_status::SUCCESS, Some(SinkOutcome::Commissioned)))
        );
        assert_eq!(s.entries()[0].frame_counter, 11);
        let tx = drain(&mut s, 30);
        assert_eq!(command(&tx[0]).0, client_cmd::PAIRING);
    }

    #[test]
    fn direct_bidirectional_commissioning_needs_a_local_temp_master() {
        let mut s = default_sink();
        let mut aes = SoftwareAesProvider;
        s.enter_commissioning(0, false).unwrap();
        let payload = commissioning_payload(10, true);
        let f = frame(SecurityLevel::None, 1, true, cmd::COMMISSIONING, &payload);
        assert_eq!(
            s.on_gpdf(&mut aes, &f.gpdf(), 10),
            SinkOutcome::Dropped(SinkDrop::NoTempMaster)
        );
        s.set_direct_bidirectional(true);
        let f = frame(SecurityLevel::None, 2, true, cmd::COMMISSIONING, &payload);
        assert_eq!(
            s.on_gpdf(&mut aes, &f.gpdf(), 20),
            SinkOutcome::CommissioningReplySent
        );
        let tx = drain(&mut s, 20);
        assert_eq!(
            Response::decode(command(&tx[0]).1)
                .unwrap()
                .temp_master_short,
            OWN
        );
        // The GPD's Success, heard directly, is opened with the pending key.
        let success = frame(SecurityLevel::Encrypted, 11, false, cmd::SUCCESS, &[]);
        assert_eq!(
            s.on_gpdf(&mut aes, &success.gpdf(), 30),
            SinkOutcome::Commissioned
        );
    }

    #[test]
    fn security_policy_is_enforced_at_commissioning() {
        let mut aes = SoftwareAesProvider;
        let unprotected = {
            let p = CommissioningPayload {
                device_id: 0x02,
                mac_seq_num_capability: true,
                rx_on_capability: false,
                pan_id_request: false,
                security_key_request: false,
                fixed_location: false,
                security: None,
                app_info: None,
            };
            let mut b = [0u8; 8];
            let n = p.encode(&mut b).unwrap();
            heapless::Vec::<u8, 8>::from_slice(&b[..n]).unwrap()
        };
        // Default gpsSecurityLevel: minimal level 0b01, level 0 refused.
        let mut s = default_sink();
        s.enter_commissioning(0, false).unwrap();
        let f = frame(
            SecurityLevel::None,
            1,
            false,
            cmd::COMMISSIONING,
            &unprotected,
        );
        assert_eq!(
            s.on_gpdf(&mut aes, &f.gpdf(), 0),
            SinkOutcome::Dropped(SinkDrop::SecurityPolicy)
        );
        // Minimal level 0 accepts it.
        assert_eq!(
            s.write_attribute(attr::GPS_SECURITY_LEVEL, BITMAP8, &[0x00]),
            zcl_status::SUCCESS
        );
        let f = frame(
            SecurityLevel::None,
            2,
            false,
            cmd::COMMISSIONING,
            &unprotected,
        );
        assert_eq!(s.on_gpdf(&mut aes, &f.gpdf(), 0), SinkOutcome::Commissioned);
        // "Protection with gpLinkKey" refuses a key sent in the clear.
        let mut s = default_sink();
        s.write_attribute(attr::GPS_SECURITY_LEVEL, BITMAP8, &[0x04]);
        s.enter_commissioning(0, false).unwrap();
        let f = frame(
            SecurityLevel::None,
            1,
            false,
            cmd::COMMISSIONING,
            &commissioning_payload(10, false),
        );
        assert_eq!(
            s.on_gpdf(&mut aes, &f.gpdf(), 0),
            SinkOutcome::Dropped(SinkDrop::SecurityPolicy)
        );
        // A wrong key MIC is refused.
        let mut payload = commissioning_payload(10, true);
        payload[10] ^= 1;
        let f = frame(SecurityLevel::None, 2, false, cmd::COMMISSIONING, &payload);
        assert_eq!(
            s.on_gpdf(&mut aes, &f.gpdf(), 0),
            SinkOutcome::Dropped(SinkDrop::AuthFailed)
        );
        // Involve-TC is not supported.
        assert_eq!(
            s.write_attribute(attr::GPS_SECURITY_LEVEL, BITMAP8, &[0x08]),
            zcl_status::INVALID_VALUE
        );
        s.set_config(SinkConfig {
            security_level: security_level_attr::INVOLVE_TC,
            ..*s.config()
        });
        assert_eq!(s.enter_commissioning(0, false), Err(Error::Unsupported));
    }

    #[test]
    fn decommissioning_removes_the_gpd_everywhere() {
        let mut s = commissioned();
        drain(&mut s, 100);
        while s.take_group_change().is_some() {}
        s.take_dirty();
        let mut aes = SoftwareAesProvider;
        let f = frame(
            SecurityLevel::Encrypted,
            11,
            false,
            cmd::DECOMMISSIONING,
            &[],
        );
        assert_eq!(
            s.on_gpdf(&mut aes, &f.gpdf(), 200),
            SinkOutcome::Decommissioned
        );
        assert!(s.entries().is_empty());
        assert!(s.take_dirty());
        assert_eq!(
            s.take_group_change(),
            Some(GroupChange::Leave(derive_alias(&GPD)))
        );
        let tx = drain(&mut s, 200);
        let p = Pairing::decode(command(&tx[0]).1).unwrap();
        assert!(p.remove_gpd && !p.add_sink);
        assert_eq!(p.gpd_id, GPD);
        // A replayed (stale) Decommissioning of another commissioning is
        // refused.
        let mut s = commissioned();
        let f = frame(
            SecurityLevel::Encrypted,
            9,
            false,
            cmd::DECOMMISSIONING,
            &[],
        );
        assert_eq!(
            s.on_gpdf(&mut aes, &f.gpdf(), 200),
            SinkOutcome::Dropped(SinkDrop::Stale)
        );
        assert_eq!(s.entries().len(), 1);
    }

    #[test]
    fn sink_table_request_follows_the_silabs_rules() {
        let mut s = default_sink();
        let mut aes = SoftwareAesProvider;
        let mut request = |s: &mut S, req: SinkTableRequest, unicast: bool| {
            let mut b = [0u8; 16];
            let n = req.encode(&mut b).unwrap();
            assert_eq!(SinkTableRequest::decode(&b[..n]), Ok(req));
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0x0000,
                    unicast,
                    tsn: 0x42,
                    command_id: server_cmd::SINK_TABLE_REQUEST,
                    payload: &b[..n],
                },
                0,
            )
            .unwrap();
            s.poll(0).map(|reply| {
                // The response reuses the request's ZCL TSN.
                assert_eq!(reply.tsn, Some(0x42));
                let tx = &reply.tx;
                assert_eq!(tx.dest, DestAddr::Unicast { nwk: 0, ieee: None });
                let (id, p) = command(tx);
                assert_eq!(id, client_cmd::SINK_TABLE_RESPONSE);
                heapless::Vec::<u8, 128>::from_slice(p).unwrap()
            })
        };
        // Empty table.
        assert_eq!(
            request(&mut s, SinkTableRequest::ByIndex(0), true).as_deref(),
            Some(&[status::NOT_FOUND, 0, 0, 0][..])
        );
        for i in 0..4u32 {
            let mut e = SinkEntry::new(GpdId::SrcId(0x100 + i));
            e.security_level = SecurityLevel::Encrypted;
            e.key_type = KeyType::IndividualKey;
            e.key = Some(KEY);
            s.restore(e).unwrap();
        }
        // Broadcast requests are not answered.
        assert_eq!(request(&mut s, SinkTableRequest::ByIndex(0), false), None);
        let r = request(&mut s, SinkTableRequest::ByIndex(1), true).unwrap();
        assert_eq!(&r[..3], &[status::SUCCESS, 4, 1]);
        assert!(r.len() <= MAX_TX_PAYLOAD);
        let (first, _) = SinkEntry::decode(&r[4..]).unwrap();
        assert_eq!(first.gpd_id, GpdId::SrcId(0x101));
        assert!(r[3] >= 1 && r[3] < 3);
        let r = request(&mut s, SinkTableRequest::ByIndex(4), true).unwrap();
        assert_eq!(&r[..], &[status::NOT_FOUND, 4, 4, 0]);
        let r = request(&mut s, SinkTableRequest::ByGpdId(GpdId::SrcId(0x102)), true).unwrap();
        assert_eq!(&r[..4], &[status::SUCCESS, 4, 0xFF, 1]);
        let r = request(&mut s, SinkTableRequest::ByGpdId(GpdId::SrcId(0x999)), true).unwrap();
        assert_eq!(&r[..], &[status::NOT_FOUND, 4, 0xFF, 0]);
    }

    #[test]
    fn sink_attributes_read_and_write() {
        let mut s = commissioned();
        let mut out = [0u8; 128];
        let read = |s: &S, id, out: &mut [u8]| s.read_attribute(id, out).unwrap().unwrap();
        assert_eq!(
            read(&s, attr::GPS_MAX_SINK_TABLE_ENTRIES, &mut out),
            (UINT8, 1)
        );
        assert_eq!(out[0], 4);
        assert_eq!(
            read(&s, attr::GPS_COMMUNICATION_MODE, &mut out),
            (BITMAP8, 1)
        );
        assert_eq!(out[0], 0x01);
        assert_eq!(
            read(&s, attr::GPS_COMMISSIONING_EXIT_MODE, &mut out),
            (BITMAP8, 1)
        );
        assert_eq!(out[0], 0x02);
        assert_eq!(
            read(&s, attr::GPS_COMMISSIONING_WINDOW, &mut out),
            (UINT16, 2)
        );
        assert_eq!(&out[..2], &[0xB4, 0x00]);
        assert_eq!(read(&s, attr::GPS_SECURITY_LEVEL, &mut out), (BITMAP8, 1));
        assert_eq!(read(&s, attr::GPS_FUNCTIONALITY, &mut out), (BITMAP24, 3));
        let f = u32::from_le_bytes([out[0], out[1], out[2], 0]);
        assert_eq!(f, SINK_FUNCTIONALITY);
        assert_eq!(f & PROXIMITY_BIDIRECTIONAL, 0);
        s.set_direct_bidirectional(true);
        read(&s, attr::GPS_FUNCTIONALITY, &mut out);
        assert_ne!(out[0] & PROXIMITY_BIDIRECTIONAL as u8, 0);
        assert_eq!(
            read(&s, attr::GPS_ACTIVE_FUNCTIONALITY, &mut out),
            (BITMAP24, 3)
        );
        let (t, n) = read(&s, attr::SINK_TABLE, &mut out);
        assert_eq!(t, LONG_OCTET_STRING);
        assert_eq!(u16::from_le_bytes([out[0], out[1]]) as usize, n - 2);
        assert_eq!(SinkEntry::decode(&out[2..n]).unwrap().0.gpd_id, GPD);
        assert_eq!(s.read_attribute(attr::PROXY_TABLE, &mut out), Ok(None));

        use zcl_status::*;
        assert_eq!(
            s.write_attribute(attr::GPS_FUNCTIONALITY, BITMAP24, &[0, 0, 0]),
            READ_ONLY
        );
        assert_eq!(
            s.write_attribute(0x0042, UINT8, &[0]),
            UNSUPPORTED_ATTRIBUTE
        );
        assert_eq!(
            s.write_attribute(attr::GPS_COMMISSIONING_WINDOW, UINT8, &[5]),
            INVALID_DATA_TYPE
        );
        assert_eq!(
            s.write_attribute(attr::GPS_COMMISSIONING_WINDOW, UINT16, &[0x3C, 0x00]),
            SUCCESS
        );
        assert_eq!(s.config().commissioning_window_s, 60);
        // Commissioned groupcast needs a configured group.
        assert_eq!(
            s.write_attribute(attr::GPS_COMMUNICATION_MODE, BITMAP8, &[0x02]),
            INVALID_VALUE
        );
        assert_eq!(
            s.write_attribute(attr::GPS_COMMUNICATION_MODE, BITMAP8, &[0x00]),
            SUCCESS
        );
        assert_eq!(
            s.config().communication_mode,
            CommunicationMode::FullUnicast
        );
        assert_eq!(
            s.write_attribute(attr::GPS_COMMISSIONING_EXIT_MODE, BITMAP8, &[0x08]),
            INVALID_VALUE
        );
    }

    #[test]
    fn sink_commissioning_mode_command() {
        use sink_commissioning_options::*;
        let mut s = default_sink();
        let mut aes = SoftwareAesProvider;
        let mut send = |s: &mut S, options: u8, gpm: u16, endpoint: u8| {
            let m = SinkCommissioningMode {
                options,
                gpm_addr_for_security: gpm,
                gpm_addr_for_pairing: 0xFFFF,
                sink_endpoint: endpoint,
            };
            let mut b = [0u8; 8];
            let n = m.encode(&mut b).unwrap();
            assert_eq!(SinkCommissioningMode::decode(&b[..n]), Ok(m));
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0,
                    unicast: true,
                    tsn: 0x42,
                    command_id: server_cmd::SINK_COMMISSIONING_MODE,
                    payload: &b[..n],
                },
                0,
            )
            .unwrap()
            .0
        };
        assert_eq!(send(&mut s, ACTION, 0xFFFF, 7), zcl_status::NOT_FOUND);
        assert_eq!(
            send(&mut s, ACTION | INVOLVE_GPM_IN_SECURITY, 0xFFFF, 0xFF),
            zcl_status::UNSUP_COMMAND
        );
        assert_eq!(
            send(&mut s, ACTION, 0x1234, ENDPOINT),
            zcl_status::INVALID_FIELD
        );
        assert!(!s.in_commissioning_mode());
        assert_eq!(
            send(&mut s, ACTION | INVOLVE_PROXIES, 0xFFFF, ENDPOINT),
            zcl_status::SUCCESS
        );
        assert!(s.in_commissioning_mode());
        assert_eq!(
            command(&drain(&mut s, 0)[0]).0,
            client_cmd::PROXY_COMMISSIONING_MODE
        );
        assert_eq!(send(&mut s, 0, 0xFFFF, 0xFF), zcl_status::SUCCESS);
        assert!(!s.in_commissioning_mode());
        // Translation Table and Pairing Configuration are not implemented.
        assert_eq!(
            s.on_server_command(
                &mut aes,
                &ServerCommand {
                    src_nwk: 0,
                    unicast: true,
                    tsn: 0x42,
                    command_id: server_cmd::TRANSLATION_TABLE_REQUEST,
                    payload: &[0]
                },
                0
            ),
            Err(Error::Unsupported)
        );
    }

    #[test]
    fn restore_skips_protected_counters_ahead() {
        let s = commissioned();
        let entry = s.entries()[0].clone();
        let mut b = [0u8; MAX_SINK_ENTRY_ENCODED_LEN];
        let n = entry.encode(&mut b).unwrap();
        let (stored, _) = SinkEntry::decode(&b[..n]).unwrap();
        let mut rebooted = default_sink();
        rebooted.restore(stored).unwrap();
        assert_eq!(
            rebooted.entries()[0].frame_counter,
            10 + DEFAULT_SINK_COUNTER_PERSIST_STRIDE - 1
        );
        let mut aes = SoftwareAesProvider;
        let replay = frame(SecurityLevel::Encrypted, 11, false, 0x22, &[]);
        assert_eq!(
            rebooted.on_gpdf(&mut aes, &replay.gpdf(), 0),
            SinkOutcome::Dropped(SinkDrop::Stale)
        );
        let next = frame(
            SecurityLevel::Encrypted,
            10 + DEFAULT_SINK_COUNTER_PERSIST_STRIDE,
            false,
            0x22,
            &[],
        );
        assert_eq!(
            rebooted.on_gpdf(&mut aes, &next.gpdf(), 0),
            SinkOutcome::Translated
        );
        // The first frame above the floor is persisted.
        assert!(rebooted.take_dirty());
        assert_eq!(rebooted.required_groups().next(), Some(derive_alias(&GPD)));
    }

    /// The entry as the runtime journals it: encoding plus counter mode.
    fn stored(s: &S) -> (heapless::Vec<u8, MAX_SINK_ENTRY_ENCODED_LEN>, bool) {
        let entry = &s.entries()[0];
        let mut b = [0u8; MAX_SINK_ENTRY_ENCODED_LEN];
        let n = entry.encode(&mut b).unwrap();
        (
            heapless::Vec::from_slice(&b[..n]).unwrap(),
            entry.counter_exact(),
        )
    }

    fn reboot_from(stored: &(heapless::Vec<u8, MAX_SINK_ENTRY_ENCODED_LEN>, bool)) -> S {
        let (entry, _) = SinkEntry::decode(&stored.0).unwrap();
        let mut s = default_sink();
        s.restore(entry.with_counter_exact(stored.1)).unwrap();
        s
    }

    /// Reboot after `n` Toggles (exact) or reports (batched): no counter is
    /// accepted twice and the first Toggle after the reboot works.
    #[test]
    fn reboot_after_1_2_64_65_frames_in_both_modes() {
        const S_: u32 = DEFAULT_SINK_COUNTER_PERSIST_STRIDE;
        let mut aes = SoftwareAesProvider;
        let report = [0x02, 0x04, 0x00, 0x00, 0x29, 0x10, 0x09];
        for n in [1u32, 2, 64, 65] {
            for switch in [true, false] {
                let mut s = commissioned();
                assert!(s.entries()[0].counter_exact(), "new entries are exact");
                s.take_dirty();
                let mut flash = stored(&s);
                let mut now = 100;
                let rx = |s: &mut S, fc: u32, now: u32, aes: &mut SoftwareAesProvider| {
                    let f = if switch {
                        frame(SecurityLevel::Encrypted, fc, false, 0x22, &[])
                    } else {
                        frame(SecurityLevel::Encrypted, fc, false, 0xA0, &report)
                    };
                    let outcome = s.on_gpdf(aes, &f.gpdf(), now);
                    while s.take_command().is_some() {}
                    while s.take_event().is_some() {}
                    outcome
                };
                for fc in 11..=10 + n {
                    now += 60_000;
                    let outcome = rx(&mut s, fc, now, &mut aes);
                    assert!(
                        matches!(
                            outcome,
                            SinkOutcome::Translated | SinkOutcome::PassedThrough
                        ),
                        "n {n} fc {fc}: {outcome:?}"
                    );
                    if s.take_dirty() {
                        flash = stored(&s);
                    }
                }
                assert_eq!(s.entries()[0].counter_exact(), switch);
                let mut r = reboot_from(&flash);
                for fc in 10..=10 + n {
                    now += 1_000;
                    assert_eq!(
                        rx(&mut r, fc, now, &mut aes),
                        SinkOutcome::Dropped(SinkDrop::Stale),
                        "n {n} switch {switch}: replayed {fc}"
                    );
                }
                let floor = r.entries()[0].frame_counter;
                if switch {
                    assert_eq!(floor, 10 + n);
                    now += 60_000;
                    assert_eq!(
                        rx(&mut r, 11 + n, now, &mut aes),
                        SinkOutcome::Translated,
                        "n {n}: the first Toggle after the reboot"
                    );
                } else {
                    assert!(floor >= 10 + n && floor < 10 + n + S_, "n {n}: {floor}");
                }
            }
        }
    }

    #[test]
    fn repeated_commissioning_follows_the_counter_stride() {
        let mut s = sink(SinkConfig {
            exit_mode: exit_mode_attr::ON_WINDOW_EXPIRATION,
            ..SinkConfig::default()
        });
        let mut aes = SoftwareAesProvider;
        s.enter_commissioning(0, false).unwrap();
        for (i, (fc, dirty)) in [(10, true), (11, false), (12, false), (5, true)]
            .into_iter()
            .enumerate()
        {
            let f = frame(
                SecurityLevel::None,
                i as u32 + 1,
                false,
                cmd::COMMISSIONING,
                &commissioning_payload(fc, true),
            );
            assert_eq!(
                s.on_gpdf(&mut aes, &f.gpdf(), 10 * i as u32),
                SinkOutcome::Commissioned
            );
            assert_eq!(s.take_dirty(), dirty, "counter {fc}");
            // Every commissioning re-announces the pairing.
            assert_eq!(command(&drain(&mut s, 0)[0]).0, client_cmd::PAIRING);
        }
        assert!(s.in_commissioning_mode());
        assert_eq!(s.entries().len(), 1);
    }

    #[test]
    fn full_table_reports_a_failed_commissioning() {
        let mut s = default_sink();
        for i in 0..4u32 {
            s.restore(SinkEntry::new(GpdId::SrcId(0x100 + i))).unwrap();
        }
        s.enter_commissioning(0, false).unwrap();
        let f = frame(
            SecurityLevel::None,
            1,
            false,
            cmd::COMMISSIONING,
            &commissioning_payload(10, true),
        );
        assert_eq!(
            s.on_gpdf(&mut SoftwareAesProvider, &f.gpdf(), 0),
            SinkOutcome::Dropped(SinkDrop::TableFull)
        );
        assert_eq!(
            core::iter::from_fn(|| s.take_event()).last(),
            Some(SinkEvent::CommissioningFailed {
                gpd_id: GPD,
                reason: SinkDrop::TableFull
            })
        );
    }
}

/// Why [`Sink::import`] refused an entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImportError {
    /// The Sink Table already has an entry for the GPD.
    AlreadyExists,
    /// The Sink Table is full.
    TableFull,
    /// The entry cannot be used: reserved GPD ID or security level, a
    /// protected entry without a key, or commissioned groupcast without a
    /// group.
    Invalid,
}

/// Host-managed Sink Table: a network co-processor whose host mirrors,
/// backs up and restores the table (ZRHP spec 7.11). Kept apart from the
/// commissioning paths above; nothing here runs unless a product calls it.
impl<const N: usize, const Q: usize> Sink<N, Q> {
    /// Add an entry that was commissioned elsewhere (a backup, another
    /// sink, the host's own records). The frame counter is taken as the last
    /// accepted one and is persisted exactly. GPEP group membership follows
    /// the entry; with `announce` a GP Pairing (AddSink) is broadcast, as
    /// after a commissioning, otherwise nothing is sent (the proxies are
    /// assumed to be paired already).
    pub fn import(
        &mut self,
        mut entry: SinkEntry,
        announce: bool,
    ) -> core::result::Result<(), ImportError> {
        let protected = entry.security_level.is_protected();
        if entry.gpd_id.is_reserved()
            || entry.security_level == SecurityLevel::Reserved
            || (protected && (entry.key.is_none() || entry.key_type == KeyType::NoKey))
            || (entry.communication_mode == CommunicationMode::CommissionedGroupcast
                && entry.groups.is_empty())
        {
            return Err(ImportError::Invalid);
        }
        if !protected {
            entry.key = None;
            entry.key_type = KeyType::NoKey;
        }
        if self.find(&entry.gpd_id).is_some() {
            return Err(ImportError::AlreadyExists);
        }
        if self.table.is_full() {
            return Err(ImportError::TableFull);
        }
        entry.persisted_counter = entry.frame_counter;
        entry.counter_exact = true;
        let gpd_id = entry.gpd_id;
        let groups = entry.group_ids();
        for &g in &groups {
            if !self.table.iter().any(|e| e.group_ids().contains(&g)) {
                let _ = self.group_changes.push_back(GroupChange::Join(g));
            }
        }
        let _ = self.table.push(entry);
        self.dirty = true;
        if announce {
            self.queue(Out::PairingAdd(gpd_id));
        }
        Ok(())
    }

    /// Remove a GPD from the Sink Table without GP Pairing (RemoveGPD): the
    /// proxies keep forwarding for it (e.g. another sink takes over). Reports
    /// [`SinkEvent::Decommissioned`] like [`remove`](Self::remove).
    pub fn remove_quietly(&mut self, gpd: &GpdId) -> bool {
        let Some(i) = self.find(gpd) else {
            return false;
        };
        let e = self.table.swap_remove(i);
        self.dup.forget(&e.gpd_id);
        self.release_groups(&e.group_ids());
        self.dirty = true;
        self.event(SinkEvent::Decommissioned { gpd_id: e.gpd_id });
        true
    }

    /// Queued application events, oldest first, without taking them (the
    /// newest is the one the last processed frame produced).
    pub fn events(&self) -> impl Iterator<Item = &SinkEvent> {
        self.events.iter()
    }
}

#[cfg(test)]
mod import_tests {
    use super::*;

    const OWN: u16 = 0x0000;
    const KEY: AesKey = [0x5A; 16];

    fn sink() -> Sink<2, 4> {
        let mut s = Sink::<2, 4>::new(SinkConfig::default());
        s.set_own_address(OWN, [1, 2, 3, 4, 5, 6, 7, 8], 0x1A62);
        s
    }

    fn protected(src: u32, counter: u32) -> SinkEntry {
        let mut e = SinkEntry::new(GpdId::SrcId(src));
        e.security_level = SecurityLevel::Encrypted;
        e.key_type = KeyType::IndividualKey;
        e.key = Some(KEY);
        e.frame_counter = counter;
        e.device_id = 0x02;
        e
    }

    #[test]
    fn import_keeps_the_counter_joins_groups_and_pairs_only_on_request() {
        let mut s = sink();
        assert_eq!(s.import(protected(0x1111_2222, 500), false), Ok(()));
        assert!(s.take_dirty());
        let alias = derive_alias(&GpdId::SrcId(0x1111_2222));
        assert_eq!(s.take_group_change(), Some(GroupChange::Join(alias)));
        assert!(!s.has_tx(), "no GP Pairing without announce");
        let e = &s.entries()[0];
        assert_eq!(
            (e.frame_counter, e.key, e.counter_exact()),
            (500, Some(KEY), true)
        );
        // A restored copy keeps the exact counter: nothing at or below it
        // is ever accepted again.
        let mut copy = sink();
        copy.restore(e.clone()).unwrap();
        assert_eq!(copy.entries()[0].frame_counter, 500);

        assert_eq!(
            s.import(protected(0x1111_2222, 9), false),
            Err(ImportError::AlreadyExists)
        );
        assert_eq!(s.import(protected(0x3333_4444, 1), true), Ok(()));
        assert!(s.has_tx(), "announce queues GP Pairing");
        let tx = s.poll(0).unwrap();
        assert!(matches!(
            tx.tx.frame,
            TxFrame::GpCommand {
                command_id: client_cmd::PAIRING,
                ..
            }
        ));
        assert_eq!(
            s.import(protected(0x5555_6666, 1), false),
            Err(ImportError::TableFull)
        );
    }

    #[test]
    fn import_rejects_unusable_entries() {
        let mut s = sink();
        let mut no_key = protected(0x1111_2222, 1);
        no_key.key = None;
        assert_eq!(s.import(no_key, false), Err(ImportError::Invalid));
        let mut reserved = protected(0x1111_2222, 1);
        reserved.security_level = SecurityLevel::Reserved;
        assert_eq!(s.import(reserved, false), Err(ImportError::Invalid));
        assert_eq!(
            s.import(protected(0, 1), false),
            Err(ImportError::Invalid),
            "SrcID 0 is reserved"
        );
        let mut group = protected(0x1111_2222, 1);
        group.communication_mode = CommunicationMode::CommissionedGroupcast;
        assert_eq!(s.import(group, false), Err(ImportError::Invalid));
        assert!(s.entries().is_empty());
    }

    #[test]
    fn quiet_removal_sends_nothing_but_reports_and_releases_groups() {
        let mut s = sink();
        s.import(protected(0x1111_2222, 5), false).unwrap();
        let _ = s.take_group_change();
        assert!(s.remove_quietly(&GpdId::SrcId(0x1111_2222)));
        assert!(!s.remove_quietly(&GpdId::SrcId(0x1111_2222)));
        assert!(!s.has_tx());
        assert!(matches!(s.take_group_change(), Some(GroupChange::Leave(_))));
        assert_eq!(
            s.events().last(),
            Some(&SinkEvent::Decommissioned {
                gpd_id: GpdId::SrcId(0x1111_2222)
            })
        );
        assert!(s.entries().is_empty());
    }
}
