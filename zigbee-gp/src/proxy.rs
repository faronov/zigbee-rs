//! Green Power proxy (GP spec A.3.5.2, A.3.6.3, A.3.9.1) – sans-IO core.
//!
//! [`Proxy`] owns the Proxy Table, the duplicate filter, the commissioning
//! mode state and a small queue of scheduled transmissions. The caller
//! (normally `zigbee-runtime`) feeds:
//!
//! * received GPDFs ([`Proxy::on_gpdf`]),
//! * received Green Power cluster commands addressed to the GP client
//!   ([`Proxy::on_client_command`]) and overheard proxy → sink commands
//!   ([`Proxy::on_server_command`]),
//! * `Device_annce` frames ([`Proxy::on_device_annce`]),
//!
//! and drains [`GpTx`] transmissions with [`Proxy::poll`]. Group membership
//! changes for the GP endpoint and persistence are reported back to the
//! caller; this module never touches the radio, flash or a clock.

use zigbee_crypto::{AesKey, ForwardAesProvider};

use crate::cluster::{
    self, CommissioningNotification, GP_CLUSTER_REVISION, GppGpdLink, Notification,
    NotificationResponse, Pairing, PairingSink, ProxyCommissioningMode, ProxyInfo,
    ProxyTableRequest, Response, attr, client_cmd, exit_mode, functionality, server_cmd,
};
use crate::codec::{Reader, Writer};
use crate::dup::DuplicateFilter;
#[cfg(feature = "proxy-bidirectional")]
use crate::frame::GpdfParams;
use crate::frame::{FrameType, Gpdf};
use crate::security::{self, Direction};
use crate::types::{
    ApplicationId, CommunicationMode, GpdId, IeeeAddress, KeyType, SecurityLevel, alias_seq, cmd,
    derive_alias,
};
use crate::{Error, Result};

/// Maximum unicast sinks per Proxy Table entry.
pub const MAX_UNICAST_SINKS: usize = 2;
/// Maximum commissioned groups per Proxy Table entry.
pub const MAX_SINK_GROUPS: usize = 2;
/// Maximum destinations of one scheduled transmission.
pub const MAX_DESTINATIONS: usize = MAX_UNICAST_SINKS + MAX_SINK_GROUPS + 1;
/// Maximum Green Power cluster command payload produced by the proxy.
pub const MAX_GP_COMMAND_LEN: usize = 128;
/// Largest Green Power cluster command payload that fits one unfragmented,
/// NWK-secured APS frame: 127-byte PSDU − 9 MAC header − 2 FCS − 8 NWK header
/// − 14 NWK auxiliary header − 4 MIC − 8 APS header − 3 ZCL header.
pub const MAX_TX_PAYLOAD: usize = 79;
/// Maximum encoded length of one Proxy Table entry.
pub const MAX_ENTRY_ENCODED_LEN: usize = 2
    + 9
    + 2
    + 1
    + 16
    + 4
    + 1
    + MAX_UNICAST_SINKS * 10
    + 1
    + MAX_SINK_GROUPS * 4
    + 1
    + 1
    + 1
    + 1
    + MAX_UNICAST_SINKS * 10;
/// Dmin of the tunnelling delay (A.3.6.3.1), ms.
pub const D_MIN_MS: u32 = 15;
/// Dmax of the tunnelling delay (A.3.6.3.1), ms.
pub const D_MAX_MS: u32 = 100;
/// Default commissioning window, s (gpsCommissioningWindow default).
pub const DEFAULT_COMMISSIONING_WINDOW_S: u16 = 180;
/// NWK broadcast address for all RxOnWhenIdle devices.
pub const BROADCAST_RX_ON_WHEN_IDLE: u16 = 0xFFFD;
/// gpTxQueue length: the GP 1.0a A.1.5.2.1.1 minimum.
#[cfg(feature = "proxy-bidirectional")]
pub const GP_TX_QUEUE_LEN: usize = 5;
/// Largest GPD command payload a GP Response can queue for a GPD. A
/// Commissioning Reply with PAN ID, TC-LK protected key, key MIC and frame
/// counter is 27 bytes.
#[cfg(feature = "proxy-bidirectional")]
pub const MAX_DOWNLINK_PAYLOAD: usize = 32;
/// Default gpTxQueueEntryLifetime, ms.
pub const DEFAULT_TX_QUEUE_LIFETIME_MS: u32 = 10_000;
/// gpTxOffset: a queued GPDF starts (first preamble symbol) this long after
/// the *end* of the triggering GPDF, in µs. GP 1.1 sets gpdRxOffset to
/// 20 ms (GP 1.0: 5 ms); a GPD opens its receive window at that offset for
/// at least gpdMinRxWindow (0.576 ms). Reference stacks agree: ZBOSS
/// `ZB_GPD_RX_OFFSET_MS` 20, EmberZNet `GP_RX_OFFSET_USEC` 20500 from the RX
/// time stamp. Default of [`ProxyConfig::tx_offset_us`].
pub const GP_TX_OFFSET_US: u32 = 20_000;

/// Default [`ProxyConfig::counter_persist_stride`].
pub const DEFAULT_COUNTER_PERSIST_STRIDE: u32 = 64;

/// Proxy configuration selected by the product.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProxyConfig {
    /// Commissioning window used when a Proxy Commissioning Mode command
    /// requests neither a window nor an exit mode.
    pub default_commissioning_window_s: u16,
    /// Persist a protected GPD's frame counter only after it advanced by
    /// this many values since the last persisted value, and on
    /// [`Proxy::restore`] skip it ahead by `stride - 1`: every counter
    /// accepted before a reboot was below `persisted + stride`, so none is
    /// accepted again after it. 1 persists every update and skips nothing.
    ///
    /// The trade-off is flash wear against frames lost after a reboot: up to
    /// `stride - 1` genuinely new frames per GPD are rejected as stale until
    /// its counter passes the skipped-ahead floor. The default 64 keeps one
    /// GPD reporting every 60 s near ten years of a 10k-cycle, 4 KiB-sector
    /// two-sector journal (one write per ~64 min; see the GP design doc).
    pub counter_persist_stride: u32,
    /// gpTxQueueEntryLifetime of a GPDF queued by a GP Response, ms
    /// (bidirectional proxies only).
    pub tx_queue_lifetime_ms: u32,
    /// Start of a GPDF sent from the gpTxQueue, in µs after the end of the
    /// GPDF that opened the GPD's receive window ([`GP_TX_OFFSET_US`]).
    #[cfg(feature = "proxy-bidirectional")]
    pub tx_offset_us: u32,
}

impl Default for ProxyConfig {
    fn default() -> Self {
        Self {
            default_commissioning_window_s: DEFAULT_COMMISSIONING_WINDOW_S,
            counter_persist_stride: DEFAULT_COUNTER_PERSIST_STRIDE,
            tx_queue_lifetime_ms: DEFAULT_TX_QUEUE_LIFETIME_MS,
            #[cfg(feature = "proxy-bidirectional")]
            tx_offset_us: GP_TX_OFFSET_US,
        }
    }
}

/// A unicast sink of a Proxy Table entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnicastSink {
    /// Sink IEEE address.
    pub ieee: IeeeAddress,
    /// Sink NWK address.
    pub nwk: u16,
    /// Full unicast (with GP Notification Response) instead of lightweight.
    pub full: bool,
}

/// A commissioned group of a Proxy Table entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SinkGroup {
    /// Group address.
    pub group: u16,
    /// Group-specific alias (`None` = use the entry / derived alias).
    pub alias: Option<u16>,
}

/// One Proxy Table entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProxyEntry {
    /// GPD identity.
    pub gpd_id: GpdId,
    /// EntryActive.
    pub active: bool,
    /// EntryValid.
    pub valid: bool,
    /// GPD MAC sequence number is incremental.
    pub mac_seq_cap: bool,
    /// Derived groupcast sink present.
    pub derived_group: bool,
    /// FirstToForward.
    pub first_to_forward: bool,
    /// InRange.
    pub in_range: bool,
    /// GPDfixed.
    pub gpd_fixed: bool,
    /// HasAllUnicastRoutes.
    pub has_all_unicast_routes: bool,
    /// Assigned alias.
    pub assigned_alias: Option<u16>,
    /// Security level.
    pub security_level: SecurityLevel,
    /// Key type.
    pub key_type: KeyType,
    /// GPD key (`None` for key types the proxy has no key material for).
    pub key: Option<AesKey>,
    /// Last accepted security frame counter (or MAC sequence number).
    pub frame_counter: u32,
    /// Unicast sinks.
    pub unicast_sinks: heapless::Vec<UnicastSink, MAX_UNICAST_SINKS>,
    /// Commissioned groups.
    pub groups: heapless::Vec<SinkGroup, MAX_SINK_GROUPS>,
    /// Groupcast radius (0 = stack default).
    pub groupcast_radius: u8,
    /// Search counter (inactive / invalid entries).
    pub search_counter: u8,
    persisted_counter: u32,
}

impl ProxyEntry {
    /// A new, empty, active and valid entry.
    pub fn new(gpd_id: GpdId) -> Self {
        Self {
            gpd_id,
            active: true,
            valid: true,
            mac_seq_cap: false,
            derived_group: false,
            first_to_forward: false,
            in_range: false,
            gpd_fixed: false,
            has_all_unicast_routes: false,
            assigned_alias: None,
            security_level: SecurityLevel::None,
            key_type: KeyType::NoKey,
            key: None,
            frame_counter: 0,
            unicast_sinks: heapless::Vec::new(),
            groups: heapless::Vec::new(),
            groupcast_radius: 0,
            search_counter: 0,
            persisted_counter: 0,
        }
    }

    /// Whether `self` and `other` differ at most in the frame counter.
    fn same_except_counter(&mut self, other: &Self) -> bool {
        let counters = (self.frame_counter, self.persisted_counter);
        self.frame_counter = other.frame_counter;
        self.persisted_counter = other.persisted_counter;
        let same = *self == *other;
        (self.frame_counter, self.persisted_counter) = counters;
        same
    }

    /// Whether any sink communication mode remains.
    pub fn has_sinks(&self) -> bool {
        self.derived_group || !self.unicast_sinks.is_empty() || !self.groups.is_empty()
    }

    /// The alias used for groupcast forwarding of this GPD.
    pub fn alias(&self) -> u16 {
        self.assigned_alias
            .unwrap_or_else(|| derive_alias(&self.gpd_id))
    }

    fn uses_group(&self, group: u16) -> bool {
        (self.derived_group && derive_alias(&self.gpd_id) == group)
            || self.groups.iter().any(|g| g.group == group)
    }

    /// Encode in the Proxy Table attribute format (GP 1.1 Table 39).
    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let light = self.unicast_sinks.iter().filter(|s| !s.full).count();
        let full = self.unicast_sinks.len() - light;
        let sec_use = self.security_level != SecurityLevel::None;
        let opts: u16 = (self.gpd_id.application_id() as u16)
            | (self.active as u16) << 3
            | (self.valid as u16) << 4
            | (self.mac_seq_cap as u16) << 5
            | ((light > 0) as u16) << 6
            | (self.derived_group as u16) << 7
            | (!self.groups.is_empty() as u16) << 8
            | (self.first_to_forward as u16) << 9
            | (self.in_range as u16) << 10
            | (self.gpd_fixed as u16) << 11
            | (self.has_all_unicast_routes as u16) << 12
            | (self.assigned_alias.is_some() as u16) << 13
            | (sec_use as u16) << 14
            | ((full > 0) as u16) << 15;
        let mut w = Writer::new(out);
        w.u16(opts)?;
        match self.gpd_id {
            GpdId::SrcId(s) => w.u32(s)?,
            GpdId::Ieee { ieee, endpoint } => {
                w.bytes(&ieee)?;
                w.u8(endpoint)?;
            }
        }
        if let Some(a) = self.assigned_alias {
            w.u16(a)?;
        }
        if sec_use {
            w.u8(self.security_level as u8 | (self.key_type as u8) << 2)?;
            w.bytes(&self.key.unwrap_or([0; 16]))?;
        }
        if sec_use || self.mac_seq_cap {
            w.u32(self.frame_counter)?;
        }
        let write_sinks = |w: &mut Writer<'_>, want_full: bool| -> Result<()> {
            let n = self
                .unicast_sinks
                .iter()
                .filter(|s| s.full == want_full)
                .count();
            w.u8(n as u8)?;
            for s in self.unicast_sinks.iter().filter(|s| s.full == want_full) {
                w.bytes(&s.ieee)?;
                w.u16(s.nwk)?;
            }
            Ok(())
        };
        if light > 0 {
            write_sinks(&mut w, false)?;
        }
        if !self.groups.is_empty() {
            w.u8(self.groups.len() as u8)?;
            for g in &self.groups {
                w.u16(g.group)?;
                w.u16(g.alias.unwrap_or(0xFFFF))?;
            }
        }
        w.u8(self.groupcast_radius)?;
        if !self.active || !self.valid {
            w.u8(self.search_counter)?;
        }
        if full > 0 {
            // Extended options: bit 0 = Full unicast GPS.
            w.u8(0x01)?;
            write_sinks(&mut w, true)?;
        }
        Ok(w.len())
    }

    /// Decode one entry from the Proxy Table attribute format, returning
    /// the entry and the consumed length.
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
        e.active = o & (1 << 3) != 0;
        e.valid = o & (1 << 4) != 0;
        e.mac_seq_cap = o & (1 << 5) != 0;
        e.derived_group = o & (1 << 7) != 0;
        e.first_to_forward = o & (1 << 9) != 0;
        e.in_range = o & (1 << 10) != 0;
        e.gpd_fixed = o & (1 << 11) != 0;
        e.has_all_unicast_routes = o & (1 << 12) != 0;
        if o & (1 << 13) != 0 {
            e.assigned_alias = Some(r.u16()?);
        }
        let sec_use = o & (1 << 14) != 0;
        if sec_use {
            let so = r.u8()?;
            e.security_level = SecurityLevel::from_bits(so);
            if !e.security_level.is_protected() {
                return Err(Error::Unsupported);
            }
            e.key_type = KeyType::from_bits(so >> 2)?;
            e.key = Some(r.array::<16>()?);
        }
        if sec_use || e.mac_seq_cap {
            e.frame_counter = r.u32()?;
            e.persisted_counter = e.frame_counter;
        }
        let read_sinks = |r: &mut Reader<'_>, e: &mut Self, full: bool| -> Result<()> {
            let n = r.u8()?;
            for _ in 0..n {
                let ieee = r.array::<8>()?;
                let nwk = r.u16()?;
                e.unicast_sinks
                    .push(UnicastSink { ieee, nwk, full })
                    .map_err(|_| Error::BufferFull)?;
            }
            Ok(())
        };
        if o & (1 << 6) != 0 {
            read_sinks(&mut r, &mut e, false)?;
        }
        if o & (1 << 8) != 0 {
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
        e.groupcast_radius = r.u8()?;
        if !e.active || !e.valid {
            e.search_counter = r.u8()?;
        }
        if o & (1 << 15) != 0 {
            let ext = r.u8()?;
            if ext & 0x01 != 0 {
                read_sinks(&mut r, &mut e, true)?;
            }
        }
        Ok((e, buf.len() - r.remaining()))
    }

    /// Locate the GPD ID bytes and the 4-byte frame counter (when present)
    /// in an entry produced by [`encode`](Self::encode), without decoding
    /// the rest, so storage can update a counter in place.
    pub fn encoded_layout(buf: &[u8]) -> Result<(core::ops::Range<usize>, Option<usize>)> {
        let mut r = Reader::new(buf);
        let o = r.u16()?;
        let id_len = match ApplicationId::from_bits(o as u8)? {
            ApplicationId::SrcId => 4,
            ApplicationId::Ieee => 9,
            ApplicationId::Lped => return Err(Error::Unsupported),
        };
        let id = 2..2 + id_len;
        let sec_use = o & (1 << 14) != 0;
        let at = id.end + if o & (1 << 13) != 0 { 2 } else { 0 } + if sec_use { 17 } else { 0 };
        let counter = (sec_use || o & (1 << 5) != 0).then_some(at);
        if buf.len() < counter.map_or(at, |c| c + 4) {
            return Err(Error::Truncated);
        }
        Ok((id, counter))
    }
}

/// Destination address of a proxy transmission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DestAddr {
    /// APS group addressing (NWK broadcast 0xFFFD).
    Group(u16),
    /// NWK broadcast with APS unicast delivery to the GP endpoint.
    Broadcast(u16),
    /// APS unicast.
    Unicast {
        /// Sink NWK address.
        nwk: u16,
        /// Sink IEEE address, when known.
        ieee: Option<IeeeAddress>,
    },
}

/// Payload of a proxy transmission.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TxFrame {
    /// Green Power cluster command from the GP client to the GP server
    /// (APS profile 0xA1E0, endpoints 242 → 242, no APS ACK).
    GpCommand {
        /// ZCL command identifier ([`server_cmd`]).
        command_id: u8,
        /// ZCL payload.
        payload: heapless::Vec<u8, MAX_GP_COMMAND_LEN>,
    },
    /// ZDO `Device_annce` for an alias (A.3.6.3.4.2): NWK address = alias,
    /// IEEE 0xFFFFFFFFFFFFFFFF, ZDO TSN 0.
    DeviceAnnce {
        /// The alias being defended.
        alias: u16,
    },
}

/// One transmission requested by the proxy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpTx {
    /// Destination.
    pub dest: DestAddr,
    /// NWK alias source address and sequence number, if aliasing is used.
    pub alias: Option<(u16, u8)>,
    /// NWK radius (0 = stack default).
    pub radius: u8,
    /// Frame to send.
    pub frame: TxFrame,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Dest {
    addr: DestAddr,
    alias: Option<(u16, u8)>,
    radius: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Pending {
    due_ms: u32,
    /// GPDF identity (GPD, counter) for cancellation by other proxies.
    origin: Option<(GpdId, u32)>,
    rx_after_tx: bool,
    link: GppGpdLink,
    frame: TxFrame,
    dests: heapless::Vec<Dest, MAX_DESTINATIONS>,
    /// At least one destination was handed out by `poll`.
    started: bool,
}

/// One gpTxQueue entry: a GPDF waiting for the GPD's next `rxAfterTx`
/// window.
#[cfg(feature = "proxy-bidirectional")]
#[derive(Debug, Clone, PartialEq, Eq)]
struct TxQueueEntry {
    /// GPD the frame is for; SrcID 0 for a Maintenance frame (Channel
    /// Configuration).
    gpd_id: GpdId,
    command_id: u8,
    payload: heapless::Vec<u8, MAX_DOWNLINK_PAYLOAD>,
    expires_ms: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CommissioningState {
    originator: u16,
    deadline_ms: Option<u32>,
    exit_on_first_pairing: bool,
    unicast: bool,
}

/// Why a GPDF did not produce a transmission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DropReason {
    /// Frame direction is towards a GPD.
    ToGpd,
    /// Commissioning-class command outside commissioning mode.
    NotCommissioning,
    /// No active and valid Proxy Table entry.
    NoEntry,
    /// Security level / key type do not match the entry.
    SecurityMismatch,
    /// MIC verification failed or no key is available.
    AuthFailed,
    /// Frame counter / sequence number is not fresh.
    Stale,
    /// Duplicate within gpDuplicateTimeout.
    Duplicate,
    /// Reserved SrcID or malformed payload.
    Invalid,
    /// Transmission queue full.
    QueueFull,
}

/// Result of feeding a GPDF to the proxy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpdfOutcome {
    /// A GP Notification was scheduled.
    Notification,
    /// A GP Commissioning Notification was scheduled.
    CommissioningNotification,
    /// The frame was dropped.
    Dropped(DropReason),
}

/// Group membership change for the GP endpoint requested by the proxy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GroupChange {
    /// Add GPEP (242) to the group.
    Join(u16),
    /// Remove GPEP (242) from the group.
    Leave(u16),
}

/// Why the proxy left commissioning mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommissioningExit {
    /// The originating sink sent Proxy Commissioning Mode (exit).
    Explicit,
    /// A GP Pairing arrived and the window requested exit on first pairing.
    FirstPairing,
    /// The commissioning window elapsed (reported by
    /// [`Proxy::take_commissioning_timeout`]).
    Timeout,
}

/// Commissioning mode transition caused by a Green Power cluster command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommissioningEvent {
    /// Commissioning mode entered. `window_s` is `None` when the window
    /// ends only on an explicit exit or the first pairing.
    Entered {
        originator: u16,
        window_s: Option<u16>,
    },
    /// The originator sent a new enter command while the proxy was in
    /// commissioning mode; the previous window ends and this one starts.
    Replaced { window_s: Option<u16> },
    /// Commissioning mode left.
    Left(CommissioningExit),
}

/// Effects of a received Green Power cluster command.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CommandEffects {
    /// GPEP group membership changes.
    pub groups: heapless::Vec<GroupChange, 4>,
    /// Commissioning mode transition, if any.
    pub commissioning: Option<CommissioningEvent>,
}

/// GPDF forwarding counters (wrapping), for diagnostics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ForwardStats {
    /// GPDF forwards (GP Notification / Commissioning Notification) whose
    /// first transmission [`Proxy::poll`] handed out.
    pub sent: u32,
    /// GPDF forwards cancelled before any transmission because another
    /// proxy tunnelled the same GPDF first (A.3.5.2.1).
    pub cancelled: u32,
}

/// Our pending forward of a GPDF that another proxy's GP Notification
/// cancelled (returned by [`Proxy::on_server_command`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CancelledForward {
    /// GPD of the cancelled forward.
    pub gpd_id: GpdId,
    /// Security frame counter / sequence number of the GPDF.
    pub frame_counter: u32,
}

/// The Green Power proxy state machine with `N` Proxy Table entries and a
/// transmission queue of `Q` scheduled GPDF forwards.
pub struct Proxy<const N: usize, const Q: usize> {
    config: ProxyConfig,
    own_nwk: u16,
    table: heapless::Vec<ProxyEntry, N>,
    dup: DuplicateFilter<8>,
    commissioning: Option<CommissioningState>,
    pending: heapless::Vec<Pending, Q>,
    dirty: bool,
    /// The commissioning window elapsed and the owner was not told yet.
    commissioning_timed_out: bool,
    stats: ForwardStats,
    #[cfg(feature = "proxy-bidirectional")]
    bidi: Bidirectional,
}

/// TempMaster state of a bidirectional proxy.
#[cfg(feature = "proxy-bidirectional")]
struct Bidirectional {
    /// The platform transmits a GPDF at gpTxOffset after a received GPDF.
    capable: bool,
    /// IEEE 802.15.4 channel of the network (0 = unknown).
    operational_channel: u8,
    tx_queue: heapless::Vec<TxQueueEntry, GP_TX_QUEUE_LEN>,
    /// GPD this proxy was last appointed TempMaster for; it uses the
    /// shortest tunnelling delay for that GPD (A.3.9.1 step 12).
    temp_master_for: Option<GpdId>,
}

fn due(now: u32, deadline: u32) -> bool {
    now.wrapping_sub(deadline) < 0x8000_0000
}

/// gppTunnelingDelay (A.3.6.3.1) for a GPDF received with `link`.
///
/// The distance `d` (0 = adjacent .. Dmax = edge of range) is
/// approximated from the RSSI part of the GPP-GPD link byte.
pub fn tunneling_delay_ms(first_to_forward: bool, no_route: bool, link: GppGpdLink) -> u32 {
    if no_route {
        return D_MIN_MS + D_MAX_MS;
    }
    if first_to_forward {
        return D_MIN_MS;
    }
    let rssi = (link.0 & 0x3F) as u64; // 0 (-110 dBm) .. 59 (+8 dBm)
    let d = D_MAX_MS as u64 - (rssi.min(59) * D_MAX_MS as u64) / 59;
    let x = D_MAX_MS as u64 - d;
    let x6 = x.pow(6);
    let full = (D_MAX_MS as u64).pow(6);
    (85 * (full - x6) / full) as u32 + D_MIN_MS
}

impl<const N: usize, const Q: usize> Proxy<N, Q> {
    /// Create an empty proxy.
    pub const fn new(config: ProxyConfig) -> Self {
        Self {
            config,
            own_nwk: 0xFFFE,
            table: heapless::Vec::new(),
            dup: DuplicateFilter::new(crate::dup::DUPLICATE_TIMEOUT_MS),
            commissioning: None,
            pending: heapless::Vec::new(),
            dirty: false,
            commissioning_timed_out: false,
            stats: ForwardStats {
                sent: 0,
                cancelled: 0,
            },
            #[cfg(feature = "proxy-bidirectional")]
            bidi: Bidirectional {
                capable: false,
                operational_channel: 0,
                tx_queue: heapless::Vec::new(),
                temp_master_for: None,
            },
        }
    }

    /// Active configuration.
    pub fn config(&self) -> &ProxyConfig {
        &self.config
    }

    /// Replace the configuration (product policy).
    pub fn set_config(&mut self, config: ProxyConfig) {
        self.config = config;
    }

    /// Drop the Proxy Table and all volatile state (leave / factory reset).
    /// The emptied table is reported by [`take_dirty`](Self::take_dirty) so
    /// the owner persists the reset.
    pub fn clear(&mut self) {
        self.table.clear();
        self.pending.clear();
        self.commissioning = None;
        self.commissioning_timed_out = false;
        self.dup = DuplicateFilter::new(crate::dup::DUPLICATE_TIMEOUT_MS);
        self.dirty = true;
        #[cfg(feature = "proxy-bidirectional")]
        {
            self.bidi.tx_queue.clear();
            self.bidi.temp_master_for = None;
        }
    }

    /// Whether this proxy acts as TempMaster: built with
    /// `proxy-bidirectional` and the platform declared, through
    /// [`set_bidirectional_capable`](Self::set_bidirectional_capable), that it
    /// can transmit a GPDF at gpTxOffset after a received one.
    pub fn bidirectional(&self) -> bool {
        #[cfg(feature = "proxy-bidirectional")]
        {
            self.bidi.capable
        }
        #[cfg(not(feature = "proxy-bidirectional"))]
        {
            false
        }
    }

    /// Declare whether the platform can transmit a queued GPDF at
    /// [`ProxyConfig::tx_offset_us`] after the end of a received GPDF,
    /// without CSMA/CA. Without it the proxy stays unidirectional: it does not
    /// advertise bidirectional operation and rejects GP Responses that
    /// appoint it TempMaster.
    #[cfg(feature = "proxy-bidirectional")]
    pub fn set_bidirectional_capable(&mut self, capable: bool) {
        self.bidi.capable = capable;
        if !capable {
            self.bidi.tx_queue.clear();
        }
    }

    /// Update the network's IEEE 802.15.4 channel (11..=26).
    #[cfg(feature = "proxy-bidirectional")]
    pub fn set_operational_channel(&mut self, channel: u8) {
        if self.bidi.operational_channel != channel {
            self.bidi.operational_channel = channel;
            self.bidi.tx_queue.clear();
        }
    }

    /// Whether this proxy has no gpTxQueue room for `gpd` (Notification
    /// option gppTxQueueFull, A.3.5.2.1). A proxy that cannot act as
    /// TempMaster has no room for any GPD.
    fn tx_queue_full_for(&self, _gpd: &GpdId, _now_ms: u32) -> bool {
        #[cfg(feature = "proxy-bidirectional")]
        {
            let q = &self.bidi.tx_queue;
            !self.bidi.capable
                || (!q.iter().any(|e| e.gpd_id.matches(_gpd))
                    && q.iter().filter(|e| !due(_now_ms, e.expires_ms)).count() >= GP_TX_QUEUE_LEN)
        }
        #[cfg(not(feature = "proxy-bidirectional"))]
        {
            true
        }
    }

    /// Whether this proxy is the GPD's current TempMaster.
    fn is_temp_master_for(&self, _gpd: &GpdId) -> bool {
        #[cfg(feature = "proxy-bidirectional")]
        {
            self.bidi.temp_master_for.is_some_and(|g| g.matches(_gpd))
        }
        #[cfg(not(feature = "proxy-bidirectional"))]
        {
            false
        }
    }

    /// Update this node's NWK address (after join / rejoin / conflict).
    pub fn set_own_nwk(&mut self, nwk: u16) {
        self.own_nwk = nwk;
    }

    /// Whether the proxy is in commissioning mode.
    pub fn in_commissioning_mode(&self) -> bool {
        self.commissioning.is_some()
    }

    /// Returns and clears the "commissioning window elapsed" event. The
    /// window is closed by [`poll`](Self::poll) or by the next Green Power
    /// cluster command once its deadline passed; check this after both.
    pub fn take_commissioning_timeout(&mut self) -> bool {
        core::mem::take(&mut self.commissioning_timed_out)
    }

    /// GPDF forwarding counters since start-up.
    pub fn forward_stats(&self) -> ForwardStats {
        self.stats
    }

    fn expire_commissioning(&mut self, now_ms: u32) {
        if let Some(c) = self.commissioning
            && c.deadline_ms.is_some_and(|d| due(now_ms, d))
        {
            self.commissioning = None;
            self.commissioning_timed_out = true;
        }
    }

    /// Proxy Table entries.
    pub fn entries(&self) -> &[ProxyEntry] {
        &self.table
    }

    /// Restore an entry from persistent storage.
    ///
    /// A protected entry's frame counter is skipped ahead to the highest
    /// value the proxy may have accepted before the reboot (`stored +
    /// counter_persist_stride - 1`), so no frame it saw is accepted again.
    /// The persisted value stays the stored one: the first frame above the
    /// floor is persisted.
    pub fn restore(&mut self, mut entry: ProxyEntry) -> Result<()> {
        if entry.security_level.is_protected() {
            let skip = self.config.counter_persist_stride.max(1) - 1;
            entry.frame_counter = entry.persisted_counter.saturating_add(skip);
        }
        if let Some(i) = self.find(&entry.gpd_id) {
            self.table[i] = entry;
            return Ok(());
        }
        self.table.push(entry).map_err(|_| Error::BufferFull)
    }

    /// GPEP groups required by the current table (for restoring group
    /// membership after reboot).
    pub fn required_groups(&self) -> impl Iterator<Item = u16> + '_ {
        self.table.iter().flat_map(|e| {
            let derived = e.derived_group.then(|| derive_alias(&e.gpd_id));
            derived.into_iter().chain(e.groups.iter().map(|g| g.group))
        })
    }

    /// Encode a GP *client* cluster attribute served by this proxy (GP 1.1
    /// A.3.4.2) into `out` as ZCL data. Returns the ZCL data type and the
    /// encoded length, or `None` for an unsupported attribute.
    ///
    /// The Proxy Table attribute is a long octet string of the encoded
    /// entries; when it does not fit `out` the result is
    /// [`Error::BufferFull`] and sinks use Proxy Table Request instead.
    pub fn read_attribute(&self, id: u16, out: &mut [u8]) -> Result<Option<(u8, usize)>> {
        const UINT8: u8 = 0x20;
        const UINT16: u8 = 0x21;
        const BITMAP24: u8 = 0x1A;
        const LONG_OCTET_STRING: u8 = 0x43;
        let mut w = Writer::new(out);
        let data_type = match id {
            attr::GPP_MAX_PROXY_TABLE_ENTRIES => {
                w.u8(N.min(u8::MAX as usize) as u8)?;
                UINT8
            }
            attr::PROXY_TABLE => {
                let mut total = 0usize;
                let mut tmp = [0u8; MAX_ENTRY_ENCODED_LEN];
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
            attr::GPP_FUNCTIONALITY => {
                let bidirectional = if self.bidirectional() {
                    functionality::BIDIRECTIONAL
                } else {
                    0
                };
                w.u24(functionality::BASIC_PROXY | bidirectional)?;
                BITMAP24
            }
            attr::GPP_ACTIVE_FUNCTIONALITY => {
                // A.3.4.2.8: GP feature set, remaining sub-fields reserved 1.
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

    /// Returns and clears the "table changed, persist it" flag.
    pub fn take_dirty(&mut self) -> bool {
        core::mem::take(&mut self.dirty)
    }

    /// Set the "table changed, persist it" flag again, e.g. after the owner
    /// failed to persist a snapshot.
    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    fn find(&self, gpd: &GpdId) -> Option<usize> {
        self.table.iter().position(|e| e.gpd_id.matches(gpd))
    }

    /// Next time `poll` has work, if any.
    pub fn next_deadline(&self) -> Option<u32> {
        let mut best: Option<u32> = self.commissioning.and_then(|c| c.deadline_ms);
        for p in &self.pending {
            best = Some(match best {
                Some(b) if due(p.due_ms, b) => b,
                _ => p.due_ms,
            });
        }
        best
    }

    /// Process timers and return the next due transmission, if any. Call
    /// repeatedly until it returns `None`.
    pub fn poll(&mut self, now_ms: u32) -> Option<GpTx> {
        self.expire_commissioning(now_ms);
        let idx = self.pending.iter().position(|p| due(now_ms, p.due_ms))?;
        let p = &mut self.pending[idx];
        if p.origin.is_some() && !p.started {
            self.stats.sent = self.stats.sent.wrapping_add(1);
        }
        p.started = true;
        let d = p.dests.remove(0);
        let tx = GpTx {
            dest: d.addr,
            alias: d.alias,
            radius: d.radius,
            frame: p.frame.clone(),
        };
        let forwarded_group = matches!(d.addr, DestAddr::Group(_));
        let origin = p.origin;
        if p.dests.is_empty() {
            self.pending.swap_remove(idx);
        }
        if forwarded_group
            && let Some((gpd, _)) = origin
            && let Some(i) = self.find(&gpd)
        {
            self.table[i].first_to_forward = true;
        }
        Some(tx)
    }

    fn schedule(&mut self, p: Pending) -> Result<()> {
        self.pending.push(p).map_err(|_| Error::BufferFull)
    }

    /// Drop pending forwards of GPDF (`gpd`, `counter`) for which `drop`
    /// holds; counts those not yet transmitted as cancelled.
    fn cancel_where(
        &mut self,
        gpd: &GpdId,
        counter: u32,
        mut drop: impl FnMut(&Pending) -> bool,
    ) -> Option<CancelledForward> {
        let mut cancelled = None;
        let stats = &mut self.stats;
        self.pending.retain(|p| {
            let same = p
                .origin
                .is_some_and(|(g, c)| g.matches(gpd) && c == counter);
            if !same || !drop(p) {
                return true;
            }
            if !p.started {
                stats.cancelled = stats.cancelled.wrapping_add(1);
                cancelled = Some(CancelledForward {
                    gpd_id: *gpd,
                    frame_counter: counter,
                });
            }
            false
        });
        cancelled
    }

    /// Process a received GPDF.
    ///
    /// `link` is the GPP-GPD link computed from the received RSSI/LQI.
    pub fn on_gpdf<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        frame: &Gpdf<'_>,
        link: GppGpdLink,
        now_ms: u32,
    ) -> GpdfOutcome {
        use DropReason::*;
        let h = &frame.header;
        if h.to_gpd {
            return GpdfOutcome::Dropped(ToGpd);
        }
        let gpd = h.gpd_id.unwrap_or(GpdId::SrcId(0));
        let entry_idx = self.find(&gpd);
        let protected = h.security_level.is_protected();
        let counter = if protected {
            h.frame_counter
        } else {
            frame.mac_seq as u32
        };

        // Security processing (GP-SEC, A.3.7.3).
        let mut clear = security::GpdPayload::new();
        let sec_ok = if protected {
            match entry_idx.and_then(|i| {
                let e = &self.table[i];
                let key_ok = e.security_level == h.security_level
                    && e.key_type.is_individual() == h.individual_key;
                key_ok.then_some(e.key).flatten()
            }) {
                Some(key) => match security::open(aes, &key, frame, Direction::FromGpd) {
                    Ok(pt) => {
                        clear = pt;
                        true
                    }
                    Err(_) => false,
                },
                None => false,
            }
        } else {
            let _ = clear.extend_from_slice(frame.payload);
            true
        };

        let command_id = if sec_ok { clear.first().copied() } else { None };
        let commissioning_cmd =
            command_id.is_some_and(cmd::is_commissioning) || h.frame_type == FrameType::Maintenance;

        if self.commissioning.is_some() && (!sec_ok || commissioning_cmd || h.auto_commissioning) {
            if gpd.is_reserved() && command_id != Some(cmd::CHANNEL_REQUEST) {
                return GpdfOutcome::Dropped(Invalid);
            }
            if self.dup.check_and_record(&gpd, counter, now_ms) {
                return GpdfOutcome::Dropped(Duplicate);
            }
            return self.commissioning_notification(frame, &gpd, sec_ok, &clear, link, now_ms);
        }

        if commissioning_cmd {
            return GpdfOutcome::Dropped(NotCommissioning);
        }
        let Some(i) = entry_idx else {
            return GpdfOutcome::Dropped(NoEntry);
        };
        {
            let e = &mut self.table[i];
            if e.active && !e.in_range {
                e.in_range = true;
            }
            if !e.active || !e.valid {
                return GpdfOutcome::Dropped(NoEntry);
            }
            if e.security_level != h.security_level
                || (protected && e.key_type.is_individual() != h.individual_key)
            {
                return GpdfOutcome::Dropped(SecurityMismatch);
            }
        }
        if !sec_ok {
            return GpdfOutcome::Dropped(AuthFailed);
        }
        if clear.is_empty() {
            return GpdfOutcome::Dropped(Invalid);
        }
        // Freshness (A.3.6.1.2.1).
        {
            let e = &self.table[i];
            let fresh = if protected {
                h.frame_counter > e.frame_counter
            } else if e.mac_seq_cap {
                let diff = (frame.mac_seq as u32).wrapping_sub(e.frame_counter) & 0xFF;
                (1..128).contains(&diff)
            } else {
                true
            };
            if !fresh {
                return GpdfOutcome::Dropped(Stale);
            }
        }
        if self.dup.check_and_record(&gpd, counter, now_ms) {
            return GpdfOutcome::Dropped(Duplicate);
        }
        let stride = self.config.counter_persist_stride.max(1);
        let e = &mut self.table[i];
        if protected || e.mac_seq_cap {
            e.frame_counter = counter;
            if counter.wrapping_sub(e.persisted_counter) >= stride {
                e.persisted_counter = counter;
                self.dirty = true;
            }
        }
        match self.notification(i, frame, &clear, link, now_ms) {
            Ok(()) => GpdfOutcome::Notification,
            Err(_) => GpdfOutcome::Dropped(QueueFull),
        }
    }

    fn notification(
        &mut self,
        i: usize,
        frame: &Gpdf<'_>,
        clear: &[u8],
        link: GppGpdLink,
        now_ms: u32,
    ) -> Result<()> {
        let h = &frame.header;
        let e = &self.table[i];
        let light_unicast = e.unicast_sinks.iter().any(|s| !s.full);
        let full_unicast = e.unicast_sinks.iter().any(|s| s.full);
        let n = Notification {
            gpd_id: e.gpd_id,
            also_unicast: light_unicast || full_unicast,
            also_derived_group: e.derived_group,
            also_commissioned_group: !e.groups.is_empty(),
            security_level: h.security_level,
            key_type: if h.security_level.is_protected() {
                e.key_type
            } else {
                KeyType::NoKey
            },
            rx_after_tx: h.rx_after_tx,
            tx_queue_full: h.rx_after_tx && self.tx_queue_full_for(&e.gpd_id, now_ms),
            bidirectional_cap: self.bidirectional(),
            frame_counter: if h.security_level.is_protected() {
                h.frame_counter
            } else {
                frame.mac_seq as u32
            },
            command_id: clear[0],
            payload: Some(&clear[1..]),
            proxy_info: if h.rx_after_tx {
                ProxyInfo::Link {
                    gpp_short: self.own_nwk,
                    link,
                }
            } else {
                ProxyInfo::None
            },
        };
        let mut payload = heapless::Vec::<u8, MAX_GP_COMMAND_LEN>::new();
        payload
            .resize_default(MAX_GP_COMMAND_LEN)
            .map_err(|_| Error::BufferFull)?;
        let len = n.encode(&mut payload)?;
        payload.truncate(len);

        let mut dests = heapless::Vec::<Dest, MAX_DESTINATIONS>::new();
        for s in &e.unicast_sinks {
            let _ = dests.push(Dest {
                addr: DestAddr::Unicast {
                    nwk: s.nwk,
                    ieee: Some(s.ieee),
                },
                alias: None,
                radius: 0,
            });
        }
        if e.derived_group {
            let _ = dests.push(Dest {
                addr: DestAddr::Group(derive_alias(&e.gpd_id)),
                alias: Some((
                    e.alias(),
                    alias_seq::derive(frame.mac_seq, alias_seq::DERIVED_GROUPCAST),
                )),
                radius: e.groupcast_radius,
            });
        }
        for g in &e.groups {
            let _ = dests.push(Dest {
                addr: DestAddr::Group(g.group),
                alias: Some((
                    g.alias.unwrap_or_else(|| e.alias()),
                    alias_seq::derive(frame.mac_seq, alias_seq::COMMISSIONED_GROUPCAST),
                )),
                radius: e.groupcast_radius,
            });
        }
        if dests.is_empty() {
            return Ok(());
        }
        let delay = if full_unicast || h.rx_after_tx {
            tunneling_delay_ms(e.first_to_forward, false, link)
        } else {
            D_MIN_MS
        };
        let origin = Some((e.gpd_id, n.frame_counter));
        self.schedule(Pending {
            due_ms: now_ms.wrapping_add(delay),
            origin,
            rx_after_tx: h.rx_after_tx,
            link,
            frame: TxFrame::GpCommand {
                command_id: server_cmd::NOTIFICATION,
                payload,
            },
            dests,
            started: false,
        })
    }

    fn commissioning_notification(
        &mut self,
        frame: &Gpdf<'_>,
        gpd: &GpdId,
        sec_ok: bool,
        clear: &[u8],
        link: GppGpdLink,
        now_ms: u32,
    ) -> GpdfOutcome {
        let h = &frame.header;
        let Some(c) = self.commissioning else {
            return GpdfOutcome::Dropped(DropReason::NotCommissioning);
        };
        let protected = h.security_level.is_protected();
        let (command_id, payload, security_failed) = if sec_ok {
            if clear.is_empty() {
                return GpdfOutcome::Dropped(DropReason::Invalid);
            }
            (clear[0], &clear[1..], None)
        } else {
            let mic = frame.mic.map(u32::from_le_bytes).unwrap_or(0);
            (frame.payload[0], &frame.payload[1..], Some(mic))
        };
        let key_type = if !protected {
            KeyType::NoKey
        } else if let Some(i) = self.find(gpd) {
            self.table[i].key_type
        } else if h.individual_key {
            KeyType::IndividualKey
        } else {
            KeyType::GpdGroupKey
        };
        let n = CommissioningNotification {
            gpd_id: *gpd,
            rx_after_tx: h.rx_after_tx,
            security_level: h.security_level,
            key_type,
            security_failed,
            bidirectional_cap: self.bidirectional(),
            frame_counter: if protected {
                h.frame_counter
            } else {
                frame.mac_seq as u32
            },
            command_id,
            payload: Some(payload),
            proxy_info: ProxyInfo::Link {
                gpp_short: self.own_nwk,
                link,
            },
        };
        let mut buf = heapless::Vec::<u8, MAX_GP_COMMAND_LEN>::new();
        let _ = buf.resize_default(MAX_GP_COMMAND_LEN);
        let Ok(len) = n.encode(&mut buf) else {
            return GpdfOutcome::Dropped(DropReason::Invalid);
        };
        buf.truncate(len);

        let mut dests = heapless::Vec::<Dest, MAX_DESTINATIONS>::new();
        let (dest_addr, alias) = if c.unicast {
            (
                DestAddr::Unicast {
                    nwk: c.originator,
                    ieee: None,
                },
                None,
            )
        } else if h.rx_after_tx {
            (DestAddr::Broadcast(BROADCAST_RX_ON_WHEN_IDLE), None)
        } else {
            let alias = self
                .find(gpd)
                .map(|i| self.table[i].alias())
                .unwrap_or_else(|| derive_alias(gpd));
            (
                DestAddr::Broadcast(BROADCAST_RX_ON_WHEN_IDLE),
                Some((
                    alias,
                    alias_seq::derive(frame.mac_seq, alias_seq::COMMISSIONING_NOTIFICATION),
                )),
            )
        };
        let _ = dests.push(Dest {
            addr: dest_addr,
            alias,
            radius: 0,
        });
        let delay = if h.rx_after_tx {
            tunneling_delay_ms(self.is_temp_master_for(gpd), false, link)
        } else {
            D_MIN_MS
        };
        match self.schedule(Pending {
            due_ms: now_ms.wrapping_add(delay),
            origin: Some((*gpd, n.frame_counter)),
            rx_after_tx: h.rx_after_tx,
            link,
            frame: TxFrame::GpCommand {
                command_id: server_cmd::COMMISSIONING_NOTIFICATION,
                payload: buf,
            },
            dests,
            started: false,
        }) {
            Ok(()) => GpdfOutcome::CommissioningNotification,
            Err(_) => GpdfOutcome::Dropped(DropReason::QueueFull),
        }
    }

    /// Process a Green Power cluster command sent to the GP client
    /// (server → client direction) from `src_nwk`.
    ///
    /// Commands that need a reply (Proxy Table Request) return it through
    /// `reply`.
    pub fn on_client_command(
        &mut self,
        src_nwk: u16,
        command_id: u8,
        payload: &[u8],
        now_ms: u32,
        reply: &mut Option<GpTx>,
    ) -> Result<CommandEffects> {
        let mut fx = CommandEffects::default();
        match command_id {
            client_cmd::PROXY_COMMISSIONING_MODE => {
                let m = ProxyCommissioningMode::decode(payload)?;
                self.on_proxy_commissioning_mode(src_nwk, &m, now_ms, &mut fx);
            }
            client_cmd::PAIRING => {
                let p = Pairing::decode(payload)?;
                self.on_pairing(&p, now_ms, &mut fx)?;
            }
            client_cmd::RESPONSE => {
                let r = Response::decode(payload)?;
                self.on_response(&r, now_ms)?;
            }
            client_cmd::NOTIFICATION_RESPONSE => {
                let r = NotificationResponse::decode(payload)?;
                if let Some(i) = self.find(&r.gpd_id) {
                    let e = &mut self.table[i];
                    e.first_to_forward |= r.first_to_forward;
                    if r.no_pairing {
                        e.unicast_sinks.retain(|s| s.nwk != src_nwk);
                        self.dirty = true;
                    }
                }
            }
            client_cmd::PROXY_TABLE_REQUEST => {
                let req = ProxyTableRequest::decode(payload)?;
                *reply = Some(self.proxy_table_response(src_nwk, &req)?);
            }
            _ => return Err(Error::Unsupported),
        }
        Ok(fx)
    }

    fn on_proxy_commissioning_mode(
        &mut self,
        src_nwk: u16,
        m: &ProxyCommissioningMode,
        now_ms: u32,
        fx: &mut CommandEffects,
    ) {
        self.expire_commissioning(now_ms);
        if let Some(c) = self.commissioning
            && c.originator != src_nwk
        {
            // Only the sink that put us in commissioning mode controls it.
            return;
        }
        if !m.enter {
            if self.commissioning.take().is_some() {
                fx.commissioning = Some(CommissioningEvent::Left(CommissioningExit::Explicit));
            }
            return;
        }
        let window = match (m.window_s, m.exit_mode) {
            (Some(w), _) => Some(w),
            (None, 0) => Some(self.config.default_commissioning_window_s),
            (None, _) => None,
        };
        let replaced = self
            .commissioning
            .replace(CommissioningState {
                originator: src_nwk,
                deadline_ms: window.map(|w| now_ms.wrapping_add(w as u32 * 1000)),
                exit_on_first_pairing: m.exit_mode & exit_mode::ON_FIRST_PAIRING != 0,
                unicast: m.unicast,
            })
            .is_some();
        fx.commissioning = Some(if replaced {
            CommissioningEvent::Replaced { window_s: window }
        } else {
            CommissioningEvent::Entered {
                originator: src_nwk,
                window_s: window,
            }
        });
    }

    fn mode_supported(mode: CommunicationMode) -> bool {
        cfg!(feature = "proxy-full") || !matches!(mode, CommunicationMode::FullUnicast)
    }

    fn release_groups(&self, groups: &[u16], fx: &mut CommandEffects) {
        for &g in groups {
            if !self.table.iter().any(|e| e.uses_group(g)) {
                let _ = fx.groups.push(GroupChange::Leave(g));
            }
        }
    }

    fn on_pairing(&mut self, p: &Pairing, now_ms: u32, fx: &mut CommandEffects) -> Result<()> {
        self.expire_commissioning(now_ms);
        if let Some(c) = self.commissioning
            && c.exit_on_first_pairing
        {
            self.commissioning = None;
            fx.commissioning = Some(CommissioningEvent::Left(CommissioningExit::FirstPairing));
        }
        let idx = self.find(&p.gpd_id);
        if p.remove_gpd {
            if let Some(i) = idx {
                let e = self.table.swap_remove(i);
                self.dup.forget(&e.gpd_id);
                let mut gs = heapless::Vec::<u16, { MAX_SINK_GROUPS + 1 }>::new();
                if e.derived_group {
                    let _ = gs.push(derive_alias(&e.gpd_id));
                }
                for g in &e.groups {
                    let _ = gs.push(g.group);
                }
                self.release_groups(&gs, fx);
                self.dirty = true;
            }
            return Ok(());
        }
        if !Self::mode_supported(p.communication_mode) {
            return Err(Error::Unsupported);
        }
        if !p.add_sink {
            let Some(i) = idx else { return Ok(()) };
            let e = &mut self.table[i];
            let mut released = None;
            match (p.communication_mode, p.sink) {
                (CommunicationMode::DerivedGroupcast, _) => {
                    e.derived_group = false;
                    released = Some(derive_alias(&e.gpd_id));
                }
                (CommunicationMode::CommissionedGroupcast, PairingSink::Group(g)) => {
                    e.groups.retain(|x| x.group != g);
                    released = Some(g);
                }
                (_, PairingSink::Unicast { ieee, .. }) => {
                    e.unicast_sinks.retain(|s| s.ieee != ieee);
                }
                _ => {}
            }
            if !e.has_sinks() {
                let e = self.table.swap_remove(i);
                self.dup.forget(&e.gpd_id);
            }
            if let Some(g) = released {
                self.release_groups(&[g], fx);
            }
            self.dirty = true;
            return Ok(());
        }

        // An unchanged entry is not written again; a counter-only change
        // follows the frame-counter persistence stride (as for GPDFs).
        let before = idx.map(|i| self.table[i].clone());
        let i = match idx {
            Some(i) => i,
            None => {
                self.table
                    .push(ProxyEntry::new(p.gpd_id))
                    .map_err(|_| Error::BufferFull)?;
                self.table.len() - 1
            }
        };
        let stride = self.config.counter_persist_stride.max(1);
        let e = &mut self.table[i];
        e.active = true;
        e.valid = true;
        e.search_counter = 0;
        e.gpd_fixed = p.gpd_fixed;
        e.mac_seq_cap = p.mac_seq_cap;
        e.security_level = p.security_level;
        e.key_type = if p.security_level.is_protected() {
            p.key_type
        } else {
            KeyType::NoKey
        };
        if let Some(k) = p.key {
            e.key = Some(k);
        }
        if !p.security_level.is_protected() {
            e.key = None;
        }
        if let Some(fc) = p.frame_counter {
            e.frame_counter = fc;
        }
        if let Some(a) = p.assigned_alias {
            e.assigned_alias = Some(a);
        }
        if let Some(r) = p.groupcast_radius {
            e.groupcast_radius = r;
        }
        match (p.communication_mode, p.sink) {
            (CommunicationMode::DerivedGroupcast, PairingSink::Group(_)) => {
                if !e.derived_group {
                    e.derived_group = true;
                    let _ = fx.groups.push(GroupChange::Join(derive_alias(&e.gpd_id)));
                }
            }
            (CommunicationMode::CommissionedGroupcast, PairingSink::Group(g)) => {
                if !e.groups.iter().any(|x| x.group == g) {
                    e.groups
                        .push(SinkGroup {
                            group: g,
                            alias: None,
                        })
                        .map_err(|_| Error::BufferFull)?;
                    let _ = fx.groups.push(GroupChange::Join(g));
                }
            }
            (mode, PairingSink::Unicast { ieee, nwk }) => {
                let full = mode == CommunicationMode::FullUnicast;
                if let Some(s) = e.unicast_sinks.iter_mut().find(|s| s.ieee == ieee) {
                    s.nwk = nwk;
                    s.full = full;
                } else {
                    e.unicast_sinks
                        .push(UnicastSink { ieee, nwk, full })
                        .map_err(|_| Error::BufferFull)?;
                }
            }
            _ => return Err(Error::Unsupported),
        }
        if before.is_none_or(|b| !e.same_except_counter(&b))
            || e.frame_counter.wrapping_sub(e.persisted_counter) >= stride
        {
            e.persisted_counter = e.frame_counter;
            self.dirty = true;
        }
        Ok(())
    }

    /// GP Response (A.3.5.2.1 / A.3.9.1 steps 8 and 14).
    ///
    /// A Response appointing this proxy TempMaster queues the GPDF for the
    /// GPD's next `rxAfterTx` window and sets FirstToForward. It fails with
    /// [`Error::Unsupported`] when this proxy cannot act as TempMaster
    /// (unidirectional build or platform), or when the TempMaster would have
    /// to switch to another channel (Channel Configuration delivery on a
    /// non-operational channel). A full gpTxQueue returns
    /// [`Error::BufferFull`]. A Response appointing another proxy drops our
    /// queued GPDF for that GPD and clears FirstToForward.
    fn on_response(&mut self, r: &Response<'_>, _now_ms: u32) -> Result<()> {
        if r.temp_master_short == self.own_nwk {
            #[cfg(feature = "proxy-bidirectional")]
            return self.queue_response(r, _now_ms);
            #[cfg(not(feature = "proxy-bidirectional"))]
            return Err(Error::Unsupported);
        }
        #[cfg(feature = "proxy-bidirectional")]
        {
            self.bidi.tx_queue.retain(|e| !e.gpd_id.matches(&r.gpd_id));
            if self.is_temp_master_for(&r.gpd_id) {
                self.bidi.temp_master_for = None;
            }
        }
        if let Some(i) = self.find(&r.gpd_id) {
            self.table[i].first_to_forward = false;
        }
        Ok(())
    }

    #[cfg(feature = "proxy-bidirectional")]
    fn queue_response(&mut self, r: &Response<'_>, now_ms: u32) -> Result<()> {
        if !self.bidi.capable || r.transmit_on_endpoint_match {
            return Err(Error::Unsupported);
        }
        // Delivery on another channel needs the TempMaster channel switch
        // of A.3.9.1 step 8 (proxy-full).
        if self.bidi.operational_channel == 0 || r.tx_channel() != self.bidi.operational_channel {
            return Err(Error::Unsupported);
        }
        let payload = heapless::Vec::from_slice(r.payload).map_err(|_| Error::BufferFull)?;
        let entry = TxQueueEntry {
            gpd_id: r.gpd_id,
            command_id: r.command_id,
            payload,
            expires_ms: now_ms.wrapping_add(self.config.tx_queue_lifetime_ms),
        };
        let q = &mut self.bidi.tx_queue;
        q.retain(|e| !due(now_ms, e.expires_ms));
        if let Some(e) = q.iter_mut().find(|e| e.gpd_id.matches(&r.gpd_id)) {
            *e = entry; // ENTRY_REPLACED
        } else {
            q.push(entry).map_err(|_| Error::BufferFull)?; // QUEUE_FULL
        }
        self.bidi.temp_master_for = Some(r.gpd_id);
        if let Some(i) = self.find(&r.gpd_id) {
            self.table[i].first_to_forward = true;
        }
        Ok(())
    }

    /// TempMaster transmission (A.1.5.2.2, A.3.9.1 steps 8 and 14).
    ///
    /// Call for every received GPDF *before* [`on_gpdf`](Self::on_gpdf).
    /// When the GPDF opens a receive window (`rxAfterTx`, or a Channel
    /// Request) and the gpTxQueue holds a GPDF for its GPD, the queued GPDF
    /// is built into `out` (MAC frame without FCS) and removed from the
    /// queue; the caller starts it [`ProxyConfig::tx_offset_us`] after the
    /// end of the received GPDF, without CSMA/CA and without a MAC ACK
    /// request.
    ///
    /// The reply uses the security parameters of the triggering GPDF: a
    /// protected trigger must pass the Proxy Table entry's MIC and
    /// freshness checks and the reply is protected with the same key and
    /// frame counter (direction GPP → GPD). A queued GPDF for SrcID 0 is
    /// sent as an unprotected Maintenance frame.
    #[cfg(feature = "proxy-bidirectional")]
    pub fn take_downlink<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        frame: &Gpdf<'_>,
        now_ms: u32,
        mac_seq: u8,
        out: &mut [u8],
    ) -> Option<usize> {
        if !self.bidi.capable || self.bidi.tx_queue.is_empty() {
            return None;
        }
        let h = &frame.header;
        if h.to_gpd {
            return None;
        }
        let channel_request = frame.clear_command_id() == Some(cmd::CHANNEL_REQUEST);
        if !h.rx_after_tx && !channel_request {
            return None;
        }
        self.bidi.tx_queue.retain(|e| !due(now_ms, e.expires_ms));
        let gpd = h.gpd_id.unwrap_or(GpdId::SrcId(0));
        let q = &self.bidi.tx_queue;
        let idx = q.iter().position(|e| e.gpd_id.matches(&gpd)).or_else(|| {
            channel_request
                .then(|| q.iter().position(|e| e.gpd_id == GpdId::SrcId(0)))
                .flatten()
        })?;
        let maintenance = q[idx].gpd_id == GpdId::SrcId(0);
        let GpdId::SrcId(src_id) = gpd else {
            // ApplicationID 0b010 GPDs are not served by the TempMaster.
            return None;
        };
        let key = if h.security_level.is_protected() && !maintenance {
            let i = self.find(&gpd)?;
            let e = &self.table[i];
            if !e.active
                || !e.valid
                || e.security_level != h.security_level
                || e.key_type.is_individual() != h.individual_key
                || h.frame_counter <= e.frame_counter
            {
                return None;
            }
            let key = e.key?;
            security::open(aes, &key, frame, Direction::FromGpd).ok()?;
            Some(key)
        } else {
            None
        };
        let params = if maintenance {
            // A.1.4.1.2: the Extended NWK FC may be omitted towards the GPD;
            // with it, a SrcID would have to follow.
            GpdfParams {
                frame_type: FrameType::Maintenance,
                auto_commissioning: false,
                src_id: None,
                security_level: SecurityLevel::None,
                individual_key: false,
                rx_after_tx: false,
                to_gpd: false,
                frame_counter: 0,
            }
        } else {
            GpdfParams {
                frame_type: FrameType::Data,
                auto_commissioning: false,
                src_id: Some(src_id),
                security_level: if key.is_some() {
                    h.security_level
                } else {
                    SecurityLevel::None
                },
                individual_key: key.is_some() && h.individual_key,
                rx_after_tx: false,
                to_gpd: true,
                frame_counter: if key.is_some() { h.frame_counter } else { 0 },
            }
        };
        let e = &self.bidi.tx_queue[idx];
        let n = security::build_gpdf(
            aes,
            key.as_ref(),
            &params,
            mac_seq,
            e.command_id,
            &e.payload,
            out,
        )
        .ok()?;
        self.bidi.tx_queue.swap_remove(idx);
        Some(n)
    }

    fn proxy_table_response(&self, dst: u16, req: &ProxyTableRequest) -> Result<GpTx> {
        let mut payload = heapless::Vec::<u8, MAX_GP_COMMAND_LEN>::new();
        let _ = payload.resize_default(MAX_GP_COMMAND_LEN);
        let total = self.table.len() as u8;
        let mut w = Writer::new(&mut payload);
        let (status, start, slice): (u8, u8, &[ProxyEntry]) = match *req {
            ProxyTableRequest::ByGpdId(id) => match self.find(&id) {
                Some(i) => (cluster::status::SUCCESS, 0xFF, &self.table[i..=i]),
                None => (cluster::status::NOT_FOUND, 0xFF, &[]),
            },
            ProxyTableRequest::ByIndex(i) if (i as usize) < self.table.len() => {
                (cluster::status::SUCCESS, i, &self.table[i as usize..])
            }
            ProxyTableRequest::ByIndex(i) => (cluster::status::NOT_FOUND, i, &[]),
        };
        w.u8(status)?;
        w.u8(total)?;
        w.u8(start)?;
        let mut encoded = [0u8; MAX_GP_COMMAND_LEN];
        let mut used = 0usize;
        let mut count = 0u8;
        // Status, total, start index and entry count precede the entries.
        let budget = MAX_TX_PAYLOAD - 4;
        for e in slice {
            let mut tmp = [0u8; MAX_ENTRY_ENCODED_LEN];
            let n = e.encode(&mut tmp)?;
            if used + n > budget {
                break;
            }
            encoded[used..used + n].copy_from_slice(&tmp[..n]);
            used += n;
            count += 1;
        }
        w.u8(count)?;
        w.bytes(&encoded[..used])?;
        let len = w.len();
        payload.truncate(len);
        Ok(GpTx {
            dest: DestAddr::Unicast {
                nwk: dst,
                ieee: None,
            },
            alias: None,
            radius: 0,
            frame: TxFrame::GpCommand {
                command_id: server_cmd::PROXY_TABLE_RESPONSE,
                payload,
            },
        })
    }

    /// Process an overheard client → server command (GP Notification,
    /// GP Commissioning Notification, GP Tunneling Stop) sent by another
    /// proxy, cancelling our own pending forward of the same GPDF
    /// (A.3.5.2.1). Returns the cancelled forward, if one had not been
    /// transmitted yet.
    pub fn on_server_command(
        &mut self,
        src_nwk: u16,
        command_id: u8,
        payload: &[u8],
    ) -> Option<CancelledForward> {
        if src_nwk == self.own_nwk {
            return None;
        }
        let (gpd, counter, rx_after_tx, other) = match command_id {
            server_cmd::NOTIFICATION => match Notification::decode(payload) {
                Ok(n) => (n.gpd_id, n.frame_counter, n.rx_after_tx, n.proxy_info),
                Err(_) => return None,
            },
            server_cmd::COMMISSIONING_NOTIFICATION => {
                match CommissioningNotification::decode(payload) {
                    Ok(n) => (n.gpd_id, n.frame_counter, n.rx_after_tx, n.proxy_info),
                    Err(_) => return None,
                }
            }
            _ => return None,
        };
        if !rx_after_tx {
            return self.cancel_where(&gpd, counter, |_| true);
        }
        let own = self.own_nwk;
        self.cancel_where(&gpd, counter, |p| {
            if !p.rx_after_tx {
                return true;
            }
            let (other_short, other_link) = match other {
                ProxyInfo::Link { gpp_short, link } => (gpp_short, link),
                ProxyInfo::Distance {
                    gpp_short,
                    distance,
                } => (gpp_short, GppGpdLink(distance)),
                ProxyInfo::None => return false,
            };
            let ours = (p.link.link_quality(), p.link.0 & 0x3F);
            let theirs = (other_link.link_quality(), other_link.0 & 0x3F);
            // Keep ours only if we are strictly better, or equal with the
            // lower short address.
            !(ours > theirs || (ours == theirs && own < other_short))
        })
    }

    /// Process a received `Device_annce` / Update Device: refresh unicast
    /// sink addresses and defend aliases (A.3.5.2.1). `random` selects the
    /// Dmin..Dmax defence delay.
    pub fn on_device_annce(&mut self, nwk: u16, ieee: IeeeAddress, now_ms: u32, random: u32) {
        if ieee == [0xFF; 8] {
            return;
        }
        let mut changed = false;
        let mut conflict = false;
        for e in self.table.iter_mut() {
            for s in e.unicast_sinks.iter_mut() {
                if s.ieee == ieee && s.nwk != nwk {
                    s.nwk = nwk;
                    changed = true;
                }
            }
            if e.alias() == nwk
                || (e.derived_group && derive_alias(&e.gpd_id) == nwk)
                || e.groups.iter().any(|g| g.alias == Some(nwk))
            {
                conflict = true;
            }
        }
        if changed {
            self.dirty = true;
        }
        let already = self
            .pending
            .iter()
            .any(|p| p.frame == TxFrame::DeviceAnnce { alias: nwk });
        if conflict && !already {
            let mut dests = heapless::Vec::new();
            let _ = dests.push(Dest {
                addr: DestAddr::Broadcast(BROADCAST_RX_ON_WHEN_IDLE),
                alias: Some((nwk, 0)),
                radius: 0,
            });
            let delay = D_MIN_MS + random % (D_MAX_MS - D_MIN_MS + 1);
            let _ = self.schedule(Pending {
                due_ms: now_ms.wrapping_add(delay),
                origin: None,
                rx_after_tx: false,
                link: GppGpdLink(0),
                frame: TxFrame::DeviceAnnce { alias: nwk },
                dests,
                started: false,
            });
        }
    }

    /// An identical alias `Device_annce` was overheard: drop our pending one.
    pub fn on_alias_device_annce(&mut self, alias: u16) {
        self.pending
            .retain(|p| p.frame != TxFrame::DeviceAnnce { alias });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::parse;
    use zigbee_crypto::SoftwareAesProvider;

    const KEY_C0: AesKey = [
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE,
        0xCF,
    ];
    const SRC: u32 = 0x8765_4321;
    // A.1.5.5.4 L3 shared-key frame, FC 2, OFF.
    const L3_SHARED: [u8; 15] = [
        0x8C, 0x18, 0x21, 0x43, 0x65, 0x87, 0x02, 0x00, 0x00, 0x00, 0x83, 0xCA, 0x43, 0x24, 0xDD,
    ];

    type P = Proxy<4, 4>;

    fn zha_pcm(enter: bool) -> heapless::Vec<u8, 8> {
        let mut b = [0u8; 8];
        let n = ProxyCommissioningMode {
            enter,
            exit_mode: exit_mode::ON_EXPLICIT_EXIT,
            window_s: enter.then_some(180),
            channel: None,
            unicast: false,
        }
        .encode(&mut b)
        .unwrap();
        heapless::Vec::from_slice(&b[..n]).unwrap()
    }

    fn zha_pairing(fc: u32) -> heapless::Vec<u8, 64> {
        let p = Pairing {
            gpd_id: GpdId::SrcId(SRC),
            add_sink: true,
            remove_gpd: false,
            communication_mode: CommunicationMode::CommissionedGroupcast,
            gpd_fixed: false,
            mac_seq_cap: true,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::GpdGroupKey,
            sink: PairingSink::Group(0x0B84),
            device_id: Some(0x02),
            frame_counter: Some(fc),
            key: Some(KEY_C0),
            assigned_alias: None,
            groupcast_radius: None,
        };
        let mut b = [0u8; 64];
        let n = p.encode(&mut b).unwrap();
        heapless::Vec::from_slice(&b[..n]).unwrap()
    }

    fn paired_proxy(fc: u32) -> P {
        let mut p = P::new(ProxyConfig::default());
        p.set_own_nwk(0x1234);
        let mut reply = None;
        let fx = p
            .on_client_command(0x0000, client_cmd::PAIRING, &zha_pairing(fc), 0, &mut reply)
            .unwrap();
        assert_eq!(&fx.groups[..], &[GroupChange::Join(0x0B84)]);
        assert!(p.take_dirty());
        p
    }

    #[test]
    fn serves_mandatory_client_attributes() {
        let p = paired_proxy(1);
        let mut out = [0u8; 128];
        assert_eq!(
            p.read_attribute(attr::GPP_MAX_PROXY_TABLE_ENTRIES, &mut out),
            Ok(Some((0x20, 1)))
        );
        assert_eq!(out[0], 4);
        assert_eq!(
            p.read_attribute(attr::GPP_FUNCTIONALITY, &mut out),
            Ok(Some((0x1A, 3)))
        );
        assert_eq!(
            u32::from_le_bytes([out[0], out[1], out[2], 0]),
            functionality::BASIC_PROXY
        );
        assert_eq!(
            p.read_attribute(attr::GPP_ACTIVE_FUNCTIONALITY, &mut out),
            Ok(Some((0x1A, 3)))
        );
        assert_eq!(&out[..3], &[0xFF, 0xFF, 0xFF]);
        let Ok(Some((0x43, n))) = p.read_attribute(attr::PROXY_TABLE, &mut out) else {
            panic!()
        };
        let body = u16::from_le_bytes([out[0], out[1]]) as usize;
        assert_eq!(n, body + 2);
        let (entry, used) = ProxyEntry::decode(&out[2..n]).unwrap();
        assert_eq!(used, body);
        assert_eq!(entry.gpd_id, GpdId::SrcId(SRC));
        assert_eq!(p.read_attribute(attr::GP_LINK_KEY, &mut out), Ok(None));
        let mut tiny = [0u8; 4];
        assert_eq!(
            p.read_attribute(attr::PROXY_TABLE, &mut tiny),
            Err(Error::BufferFull)
        );
    }

    #[test]
    fn proxy_table_response_fits_one_unfragmented_frame() {
        let mut p = P::new(ProxyConfig::default());
        for i in 0..4u32 {
            let mut e = ProxyEntry::new(GpdId::SrcId(0x1000 + i));
            e.security_level = SecurityLevel::Encrypted;
            e.key_type = KeyType::IndividualKey;
            e.key = Some(KEY_C0);
            e.groups
                .push(SinkGroup {
                    group: 0x0B84,
                    alias: None,
                })
                .unwrap();
            p.restore(e).unwrap();
        }
        let mut reply = None;
        p.on_client_command(
            0x0000,
            client_cmd::PROXY_TABLE_REQUEST,
            &[0x08, 0x00],
            0,
            &mut reply,
        )
        .unwrap();
        let TxFrame::GpCommand { payload, .. } = reply.unwrap().frame else {
            panic!()
        };
        assert!(payload.len() <= MAX_TX_PAYLOAD);
        // Status, total entries, start index, then at least one entry.
        assert_eq!(&payload[..3], &[0x00, 4, 0]);
        assert!(payload[3] >= 1 && payload[3] < 4);
    }

    #[test]
    fn forwards_secured_gpdf_as_commissioned_groupcast_notification() {
        let mut p = paired_proxy(1);
        let mut aes = SoftwareAesProvider;
        let f = parse(&L3_SHARED, 0x20, None).unwrap();
        let out = p.on_gpdf(&mut aes, &f, GppGpdLink(0xC0 | 40), 1000);
        assert_eq!(out, GpdfOutcome::Notification);
        assert!(p.poll(1000 + D_MIN_MS - 1).is_none());
        let tx = p.poll(1000 + D_MIN_MS).unwrap();
        assert_eq!(tx.dest, DestAddr::Group(0x0B84));
        assert_eq!(tx.alias, Some((0x4321, 0x20u8.wrapping_sub(9))));
        let TxFrame::GpCommand {
            command_id,
            payload,
        } = tx.frame
        else {
            panic!()
        };
        assert_eq!(command_id, server_cmd::NOTIFICATION);
        let n = Notification::decode(&payload).unwrap();
        assert_eq!(n.command_id, cmd::OFF);
        assert_eq!(n.frame_counter, 2);
        assert_eq!(n.security_level, SecurityLevel::Encrypted);
        assert_eq!(n.key_type, KeyType::GpdGroupKey);
        assert!(n.also_commissioned_group);
        assert!(p.poll(5000).is_none());

        // Replay is rejected (stale), even after the duplicate timeout.
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0), 10_000),
            GpdfOutcome::Dropped(DropReason::Stale)
        );
    }

    #[test]
    fn rejects_frame_counter_not_above_paired_value() {
        let mut p = paired_proxy(2);
        let mut aes = SoftwareAesProvider;
        let f = parse(&L3_SHARED, 0x20, None).unwrap();
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0), 0),
            GpdfOutcome::Dropped(DropReason::Stale)
        );
    }

    #[test]
    fn wrong_key_and_unpaired_are_dropped() {
        let mut aes = SoftwareAesProvider;
        let f = parse(&L3_SHARED, 0x20, None).unwrap();
        let mut p = P::new(ProxyConfig::default());
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0), 0),
            GpdfOutcome::Dropped(DropReason::NoEntry)
        );
        let mut p = paired_proxy(0);
        p.table[0].key = Some([0; 16]);
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0), 0),
            GpdfOutcome::Dropped(DropReason::AuthFailed)
        );
        let mut p = paired_proxy(0);
        p.table[0].key_type = KeyType::IndividualKey;
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0), 0),
            GpdfOutcome::Dropped(DropReason::SecurityMismatch)
        );
    }

    #[test]
    fn commissioning_gpdf_needs_commissioning_mode() {
        let mut aes = SoftwareAesProvider;
        // Unprotected Commissioning GPDF: FC 0x0C, SrcID, 0xE0, dev 0x02, opts 0x00.
        let msdu = [0x0C, 0x78, 0x56, 0x34, 0x12, 0xE0, 0x02, 0x00];
        let f = parse(&msdu, 7, None).unwrap();
        let mut p = P::new(ProxyConfig::default());
        p.set_own_nwk(0x1234);
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0), 0),
            GpdfOutcome::Dropped(DropReason::NotCommissioning)
        );
        let mut reply = None;
        let fx = p
            .on_client_command(
                0,
                client_cmd::PROXY_COMMISSIONING_MODE,
                &zha_pcm(true),
                0,
                &mut reply,
            )
            .unwrap();
        assert_eq!(
            fx.commissioning,
            Some(CommissioningEvent::Entered {
                originator: 0,
                window_s: Some(180)
            })
        );
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0xC0 | 30), 10),
            GpdfOutcome::CommissioningNotification
        );
        let tx = p.poll(10 + D_MIN_MS).unwrap();
        assert_eq!(tx.dest, DestAddr::Broadcast(0xFFFD));
        assert_eq!(tx.alias, Some((0x5678, 7u8.wrapping_sub(12))));
        let TxFrame::GpCommand { payload, .. } = tx.frame else {
            panic!()
        };
        let n = CommissioningNotification::decode(&payload).unwrap();
        assert_eq!(n.gpd_id, GpdId::SrcId(0x1234_5678));
        assert_eq!(n.command_id, cmd::COMMISSIONING);
        assert_eq!(n.payload, Some(&[0x02, 0x00][..]));
        assert_eq!(n.frame_counter, 7);
        // Duplicate suppressed.
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0), 20),
            GpdfOutcome::Dropped(DropReason::Duplicate)
        );
        // Another sink cannot close the window; the originator can.
        let fx = p
            .on_client_command(
                9,
                client_cmd::PROXY_COMMISSIONING_MODE,
                &zha_pcm(false),
                30,
                &mut reply,
            )
            .unwrap();
        assert_eq!(fx.commissioning, None);
        assert!(p.in_commissioning_mode());
        let fx = p
            .on_client_command(
                0,
                client_cmd::PROXY_COMMISSIONING_MODE,
                &zha_pcm(false),
                30,
                &mut reply,
            )
            .unwrap();
        assert_eq!(
            fx.commissioning,
            Some(CommissioningEvent::Left(CommissioningExit::Explicit))
        );
        assert!(!p.take_commissioning_timeout());
    }

    #[test]
    fn commissioning_window_expires() {
        let mut p = P::new(ProxyConfig::default());
        let mut reply = None;
        p.on_client_command(
            0,
            client_cmd::PROXY_COMMISSIONING_MODE,
            &zha_pcm(true),
            100,
            &mut reply,
        )
        .unwrap();
        assert_eq!(p.next_deadline(), Some(100 + 180_000));
        p.poll(100 + 179_999);
        assert!(p.in_commissioning_mode());
        assert!(!p.take_commissioning_timeout());
        p.poll(100 + 180_000);
        assert!(!p.in_commissioning_mode());
        // Reported exactly once.
        assert!(p.take_commissioning_timeout());
        p.poll(100 + 180_001);
        assert!(!p.take_commissioning_timeout());
    }

    fn pcm(enter: bool, exit: u8, window_s: Option<u16>) -> heapless::Vec<u8, 8> {
        let mut b = [0u8; 8];
        let n = ProxyCommissioningMode {
            enter,
            exit_mode: exit,
            window_s,
            channel: None,
            unicast: false,
        }
        .encode(&mut b)
        .unwrap();
        heapless::Vec::from_slice(&b[..n]).unwrap()
    }

    #[test]
    fn commissioning_window_replaced_then_left_on_first_pairing() {
        let mut p = P::new(ProxyConfig::default());
        p.set_own_nwk(0x1234);
        let mut reply = None;
        let pcm_cmd = client_cmd::PROXY_COMMISSIONING_MODE;
        let fx = p
            .on_client_command(0, pcm_cmd, &zha_pcm(true), 0, &mut reply)
            .unwrap();
        assert!(matches!(
            fx.commissioning,
            Some(CommissioningEvent::Entered { .. })
        ));
        // ZHA re-opening permit-join replaces the running window.
        let first_pairing = exit_mode::ON_FIRST_PAIRING;
        let fx = p
            .on_client_command(
                0,
                pcm_cmd,
                &pcm(true, first_pairing, None),
                1_000,
                &mut reply,
            )
            .unwrap();
        assert_eq!(
            fx.commissioning,
            Some(CommissioningEvent::Replaced { window_s: None })
        );
        // The replacement has no deadline: the old 180 s one is gone.
        assert_eq!(p.next_deadline(), None);
        p.poll(1_000 + 180_000);
        assert!(p.in_commissioning_mode());
        assert!(!p.take_commissioning_timeout());
        let fx = p
            .on_client_command(0, client_cmd::PAIRING, &zha_pairing(1), 2_000, &mut reply)
            .unwrap();
        assert_eq!(
            fx.commissioning,
            Some(CommissioningEvent::Left(CommissioningExit::FirstPairing))
        );
        assert!(!p.in_commissioning_mode());
        // Leaving twice reports nothing.
        let fx = p
            .on_client_command(0, pcm_cmd, &zha_pcm(false), 3_000, &mut reply)
            .unwrap();
        assert_eq!(fx.commissioning, None);
    }

    #[test]
    fn elapsed_window_is_a_timeout_not_a_replacement() {
        let mut p = P::new(ProxyConfig::default());
        let mut reply = None;
        let pcm_cmd = client_cmd::PROXY_COMMISSIONING_MODE;
        p.on_client_command(0, pcm_cmd, &pcm(true, 0, Some(10)), 0, &mut reply)
            .unwrap();
        // No poll ran since the deadline; the next enter starts a new window.
        let fx = p
            .on_client_command(0, pcm_cmd, &zha_pcm(true), 10_000, &mut reply)
            .unwrap();
        assert!(p.take_commissioning_timeout());
        assert_eq!(
            fx.commissioning,
            Some(CommissioningEvent::Entered {
                originator: 0,
                window_s: Some(180)
            })
        );
        // An explicit exit after expiry is not reported twice.
        p.on_client_command(0, pcm_cmd, &pcm(true, 0, Some(10)), 20_000, &mut reply)
            .unwrap();
        assert!(!p.take_commissioning_timeout());
        let fx = p
            .on_client_command(0, pcm_cmd, &zha_pcm(false), 30_000, &mut reply)
            .unwrap();
        assert!(p.take_commissioning_timeout());
        assert_eq!(fx.commissioning, None);
        // Leave / factory reset drops a pending report.
        p.on_client_command(0, pcm_cmd, &pcm(true, 0, Some(1)), 40_000, &mut reply)
            .unwrap();
        p.poll(41_000);
        p.clear();
        assert!(!p.take_commissioning_timeout());
    }

    #[test]
    fn other_proxy_notification_cancels_pending() {
        let mut p = paired_proxy(1);
        let mut aes = SoftwareAesProvider;
        let f = parse(&L3_SHARED, 0x20, None).unwrap();
        p.on_gpdf(&mut aes, &f, GppGpdLink(0), 0);
        let n = Notification {
            gpd_id: GpdId::SrcId(SRC),
            also_unicast: false,
            also_derived_group: false,
            also_commissioned_group: true,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::GpdGroupKey,
            rx_after_tx: false,
            tx_queue_full: false,
            bidirectional_cap: false,
            frame_counter: 2,
            command_id: cmd::OFF,
            payload: Some(&[]),
            proxy_info: ProxyInfo::None,
        };
        let mut b = [0u8; 32];
        let len = n.encode(&mut b).unwrap();
        p.on_server_command(0x4321, server_cmd::NOTIFICATION, &b[..len]);
        assert!(p.poll(1000).is_none());
    }

    /// Encrypted OFF GPDF of SRC with KEY_C0, NWK part (no MAC header).
    fn secure_off(fc: u32, rx_after_tx: bool) -> heapless::Vec<u8, 64> {
        let params = crate::frame::GpdfParams {
            frame_type: FrameType::Data,
            auto_commissioning: false,
            src_id: Some(SRC),
            security_level: SecurityLevel::Encrypted,
            individual_key: false,
            rx_after_tx,
            to_gpd: false,
            frame_counter: fc,
        };
        let mut out = [0u8; 64];
        let n = security::build_gpdf(
            &mut SoftwareAesProvider,
            Some(&KEY_C0),
            &params,
            fc as u8,
            cmd::OFF,
            &[],
            &mut out,
        )
        .unwrap();
        let mac = crate::frame::parse_mac(&out[..n]).unwrap();
        heapless::Vec::from_slice(&out[mac.payload_offset..n]).unwrap()
    }

    /// Another proxy's GP Notification of SRC's frame `fc`.
    fn others_notification(fc: u32, rx_after_tx: bool, info: ProxyInfo) -> heapless::Vec<u8, 48> {
        let n = Notification {
            gpd_id: GpdId::SrcId(SRC),
            also_unicast: false,
            also_derived_group: false,
            also_commissioned_group: true,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::GpdGroupKey,
            rx_after_tx,
            tx_queue_full: false,
            bidirectional_cap: false,
            frame_counter: fc,
            command_id: cmd::OFF,
            payload: Some(&[]),
            proxy_info: info,
        };
        let mut b = [0u8; 48];
        let len = n.encode(&mut b).unwrap();
        heapless::Vec::from_slice(&b[..len]).unwrap()
    }

    #[test]
    fn forward_stats_count_sent_and_cancelled_forwards() {
        let mut p = paired_proxy(1);
        let mut aes = SoftwareAesProvider;
        let notify = server_cmd::NOTIFICATION;
        assert_eq!(p.forward_stats(), ForwardStats::default());

        // fc 2: we tunnel first.
        let raw = secure_off(2, false);
        let f = parse(&raw, 2, None).unwrap();
        assert_eq!(
            p.on_gpdf(&mut aes, &f, GppGpdLink(0), 0),
            GpdfOutcome::Notification
        );
        assert!(p.poll(1_000).is_some());
        assert!(p.poll(1_000).is_none());
        // A late duplicate from another proxy cancels nothing.
        let late = others_notification(2, false, ProxyInfo::None);
        assert_eq!(p.on_server_command(0x4321, notify, &late), None);

        // fc 3: another proxy tunnels first; ours is cancelled.
        let raw = secure_off(3, false);
        let f = parse(&raw, 3, None).unwrap();
        p.on_gpdf(&mut aes, &f, GppGpdLink(0), 2_000);
        // Our own Notification (heard back) cancels nothing.
        let own = others_notification(3, false, ProxyInfo::None);
        assert_eq!(p.on_server_command(0x1234, notify, &own), None);
        // Another GPDF's Notification cancels nothing.
        let other_fc = others_notification(4, false, ProxyInfo::None);
        assert_eq!(p.on_server_command(0x4321, notify, &other_fc), None);
        assert_eq!(
            p.on_server_command(0x4321, notify, &own),
            Some(CancelledForward {
                gpd_id: GpdId::SrcId(SRC),
                frame_counter: 3
            })
        );
        assert!(p.poll(3_000).is_none());
        assert_eq!(
            p.forward_stats(),
            ForwardStats {
                sent: 1,
                cancelled: 1
            }
        );
    }

    #[test]
    fn rx_after_tx_forward_cancelled_only_by_a_better_proxy() {
        let mut p = paired_proxy(1);
        let mut aes = SoftwareAesProvider;
        let notify = server_cmd::NOTIFICATION;
        let link = GppGpdLink(0x80 | 30);
        let raw = secure_off(2, true);
        let f = parse(&raw, 2, None).unwrap();
        assert_eq!(p.on_gpdf(&mut aes, &f, link, 0), GpdfOutcome::Notification);
        // A worse proxy does not cancel ours.
        let worse = ProxyInfo::Link {
            gpp_short: 0x0001,
            link: GppGpdLink(0x80 | 10),
        };
        assert_eq!(
            p.on_server_command(0x0001, notify, &others_notification(2, true, worse)),
            None
        );
        // A better one does.
        let better = ProxyInfo::Link {
            gpp_short: 0x4321,
            link: GppGpdLink(0xC0 | 40),
        };
        assert!(
            p.on_server_command(0x4321, notify, &others_notification(2, true, better))
                .is_some()
        );
        assert!(p.poll(1_000).is_none());
        assert_eq!(
            p.forward_stats(),
            ForwardStats {
                sent: 0,
                cancelled: 1
            }
        );
    }

    fn pair(p: &mut P, pairing: &[u8]) -> CommandEffects {
        let mut reply = None;
        p.on_client_command(0, client_cmd::PAIRING, pairing, 0, &mut reply)
            .unwrap()
    }

    #[test]
    fn repeated_pairing_does_not_rewrite_the_table() {
        const S: u32 = DEFAULT_COUNTER_PERSIST_STRIDE;
        // ZHA sends the same GP Pairing for every commissioning GPDF.
        let mut p = paired_proxy(1176);
        let fx = pair(&mut p, &zha_pairing(1176));
        assert!(fx.groups.is_empty());
        assert!(!p.take_dirty(), "identical pairing");
        // Counter-only changes follow the persistence stride.
        pair(&mut p, &zha_pairing(1177));
        pair(&mut p, &zha_pairing(1178));
        assert!(!p.take_dirty(), "counter +2");
        assert_eq!(p.entries()[0].frame_counter, 1178);
        pair(&mut p, &zha_pairing(1176 + S - 1));
        assert!(!p.take_dirty(), "counter +stride-1");
        pair(&mut p, &zha_pairing(1176 + S));
        assert!(p.take_dirty(), "counter +stride since the last write");
        pair(&mut p, &zha_pairing(1176 + S + 1));
        assert!(!p.take_dirty(), "stride restarts at the written value");
        // GPDFs and Pairings share the stride.
        let mut aes = SoftwareAesProvider;
        let raw = secure_off(1176 + 2 * S - 1, false);
        let f = parse(&raw, 0, None).unwrap();
        p.on_gpdf(&mut aes, &f, GppGpdLink(0), 0);
        assert!(!p.take_dirty(), "GPDF +stride-1");
        pair(&mut p, &zha_pairing(1176 + 2 * S));
        assert!(p.take_dirty(), "pairing completes the stride");
    }

    /// The Proxy Table snapshot the runtime journals when the proxy
    /// reports it dirty.
    type Flash = heapless::Vec<heapless::Vec<u8, MAX_ENTRY_ENCODED_LEN>, 4>;

    fn journal(p: &P) -> Flash {
        let mut out = [0u8; MAX_ENTRY_ENCODED_LEN];
        p.entries()
            .iter()
            .map(|e| {
                let n = e.encode(&mut out).unwrap();
                heapless::Vec::from_slice(&out[..n]).unwrap()
            })
            .collect()
    }

    /// A fresh (rebooted) proxy restored from `flash`.
    fn reboot(config: ProxyConfig, flash: &Flash) -> P {
        let mut rebooted = P::new(config);
        rebooted.set_own_nwk(0x1234);
        for bytes in flash {
            let (entry, used) = ProxyEntry::decode(bytes).unwrap();
            assert_eq!(used, bytes.len());
            rebooted.restore(entry).unwrap();
        }
        rebooted
    }

    fn gpdf(p: &mut P, fc: u32, now_ms: u32) -> GpdfOutcome {
        let raw = secure_off(fc, false);
        let f = parse(&raw, fc as u8, None).unwrap();
        p.on_gpdf(&mut SoftwareAesProvider, &f, GppGpdLink(0), now_ms)
    }

    #[test]
    fn reboot_skips_the_counter_ahead_by_the_stride() {
        const S: u32 = DEFAULT_COUNTER_PERSIST_STRIDE;
        let mut p = paired_proxy(1000);
        let flash = journal(&p);
        // Accept frames up to the last one before the next write.
        let mut now = 0;
        for fc in [1001, 1000 + S / 2, 1000 + S - 1] {
            now += 1_000;
            assert_eq!(gpdf(&mut p, fc, now), GpdfOutcome::Notification);
            assert!(!p.take_dirty(), "fc {fc} is inside the stride");
        }
        // Power loss: the flash still holds 1000.
        let mut p = reboot(p.config, &flash);
        let floor = 1000 + S - 1;
        assert_eq!(p.entries()[0].frame_counter, floor);
        // Replays of everything accepted before the reboot are stale.
        for fc in [1000, 1001, 1000 + S / 2, floor] {
            now += 1_000;
            assert_eq!(
                gpdf(&mut p, fc, now),
                GpdfOutcome::Dropped(DropReason::Stale),
                "replayed fc {fc}"
            );
        }
        assert!(!p.take_dirty());
        // The first frame above the floor is forwarded and persisted.
        now += 1_000;
        assert_eq!(gpdf(&mut p, floor + 1, now), GpdfOutcome::Notification);
        assert!(p.take_dirty(), "first frame after the reboot is written");
        // ...after which the stride batches again.
        now += 1_000;
        assert_eq!(gpdf(&mut p, floor + S, now), GpdfOutcome::Notification);
        assert!(!p.take_dirty());
        now += 1_000;
        assert_eq!(gpdf(&mut p, floor + S + 1, now), GpdfOutcome::Notification);
        assert!(p.take_dirty());
    }

    #[test]
    fn reboot_after_a_write_rejects_replays_below_the_new_floor() {
        const S: u32 = DEFAULT_COUNTER_PERSIST_STRIDE;
        let mut p = paired_proxy(1000);
        assert_eq!(gpdf(&mut p, 1000 + S, 1_000), GpdfOutcome::Notification);
        assert!(p.take_dirty(), "written at 1000+stride");
        let flash = journal(&p);
        assert_eq!(
            gpdf(&mut p, 1000 + 2 * S - 1, 2_000),
            GpdfOutcome::Notification
        );
        assert!(!p.take_dirty());
        let mut p = reboot(p.config, &flash);
        // The snapshot written at 1000+stride bounds every accepted counter.
        assert_eq!(p.entries()[0].frame_counter, 1000 + 2 * S - 1);
        assert_eq!(
            gpdf(&mut p, 1000 + 2 * S - 1, 3_000),
            GpdfOutcome::Dropped(DropReason::Stale)
        );
        assert_eq!(gpdf(&mut p, 1000 + 2 * S, 4_000), GpdfOutcome::Notification);
    }

    #[test]
    fn stride_one_restores_the_exact_counter() {
        let mut p = P::new(ProxyConfig {
            counter_persist_stride: 1,
            ..ProxyConfig::default()
        });
        p.set_own_nwk(0x1234);
        pair(&mut p, &zha_pairing(1000));
        assert!(p.take_dirty());
        assert_eq!(gpdf(&mut p, 1001, 1_000), GpdfOutcome::Notification);
        assert!(p.take_dirty(), "stride 1 writes every frame");
        let mut p = reboot(p.config, &journal(&p));
        assert_eq!(p.entries()[0].frame_counter, 1001);
        assert_eq!(
            gpdf(&mut p, 1001, 2_000),
            GpdfOutcome::Dropped(DropReason::Stale)
        );
        assert_eq!(gpdf(&mut p, 1002, 3_000), GpdfOutcome::Notification);
    }

    #[test]
    fn skip_ahead_saturates_and_leaves_unprotected_entries_alone() {
        let mut p = paired_proxy(u32::MAX - 3);
        let p2 = reboot(p.config, &journal(&p));
        assert_eq!(p2.entries()[0].frame_counter, u32::MAX, "exhausted");
        // An unsecured MAC-sequence entry keeps its 8-bit window.
        let mut pr = Pairing::decode(&zha_pairing(7)).unwrap();
        pr.security_level = SecurityLevel::None;
        pr.key_type = KeyType::NoKey;
        pr.key = None;
        let mut b = [0u8; 64];
        let n = pr.encode(&mut b).unwrap();
        pair(&mut p, &b[..n]);
        let p3 = reboot(p.config, &journal(&p));
        assert_eq!(p3.entries()[0].frame_counter, 7);
    }

    #[test]
    fn pairing_counter_decrease_is_persisted() {
        let mut p = paired_proxy(1000);
        // A re-commissioned GPD restarted its counter: a stale persisted
        // value above it would reject its frames after a reboot.
        pair(&mut p, &zha_pairing(5));
        assert!(p.take_dirty());
        assert_eq!(p.entries()[0].frame_counter, 5);
        pair(&mut p, &zha_pairing(5));
        assert!(!p.take_dirty());
    }

    #[test]
    fn pairing_changing_the_entry_is_persisted() {
        let mut p = paired_proxy(1);
        // New key.
        let mut b = [0u8; 64];
        let mut pr = Pairing::decode(&zha_pairing(1)).unwrap();
        pr.key = Some([0x11; 16]);
        let n = pr.encode(&mut b).unwrap();
        pair(&mut p, &b[..n]);
        assert!(p.take_dirty(), "key");
        // Another sink group.
        pr.sink = PairingSink::Group(0x1234);
        let n = pr.encode(&mut b).unwrap();
        let fx = pair(&mut p, &b[..n]);
        assert_eq!(&fx.groups[..], &[GroupChange::Join(0x1234)]);
        assert!(p.take_dirty(), "group");
        // Inactive entry re-activated.
        p.table[0].active = false;
        pair(&mut p, &b[..n]);
        assert!(p.take_dirty(), "activation");
        // Removal is always written.
        let rm = Pairing {
            gpd_id: GpdId::SrcId(SRC),
            add_sink: false,
            remove_gpd: true,
            communication_mode: CommunicationMode::CommissionedGroupcast,
            gpd_fixed: false,
            mac_seq_cap: false,
            security_level: SecurityLevel::None,
            key_type: KeyType::NoKey,
            sink: PairingSink::None,
            device_id: None,
            frame_counter: None,
            key: None,
            assigned_alias: None,
            groupcast_radius: None,
        };
        let n = rm.encode(&mut b).unwrap();
        pair(&mut p, &b[..n]);
        assert!(p.take_dirty(), "removal");
        assert!(p.entries().is_empty());
    }

    #[test]
    fn encoded_layout_locates_id_and_counter() {
        let mut buf = [0u8; MAX_ENTRY_ENCODED_LEN];
        let mut e = ProxyEntry::new(GpdId::Ieee {
            ieee: [1, 2, 3, 4, 5, 6, 7, 8],
            endpoint: 9,
        });
        e.assigned_alias = Some(0x1234);
        e.security_level = SecurityLevel::Encrypted;
        e.key_type = KeyType::IndividualKey;
        e.key = Some([0xA5; 16]);
        e.frame_counter = 0xDEAD_BEEF;
        e.derived_group = true;
        let n = e.encode(&mut buf).unwrap();
        let (id, counter) = ProxyEntry::encoded_layout(&buf[..n]).unwrap();
        assert_eq!(&buf[id], &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
        let c = counter.unwrap();
        assert_eq!(buf[c..c + 4], 0xDEAD_BEEFu32.to_le_bytes());
        assert_eq!(
            ProxyEntry::encoded_layout(&buf[..c + 3]),
            Err(Error::Truncated)
        );

        // Unsecured, no MAC sequence capability: no counter.
        let mut plain = ProxyEntry::new(GpdId::SrcId(0x0102_0304));
        plain.derived_group = true;
        let n = plain.encode(&mut buf).unwrap();
        let (id, counter) = ProxyEntry::encoded_layout(&buf[..n]).unwrap();
        assert_eq!(buf[id], 0x0102_0304u32.to_le_bytes());
        assert_eq!(counter, None);

        // Unsecured with incremental MAC sequence: counter present.
        plain.mac_seq_cap = true;
        plain.frame_counter = 77;
        let n = plain.encode(&mut buf).unwrap();
        let c = ProxyEntry::encoded_layout(&buf[..n]).unwrap().1.unwrap();
        assert_eq!(buf[c..c + 4], 77u32.to_le_bytes());
    }

    #[test]
    fn remove_gpd_leaves_group_and_entry_round_trips() {
        let mut p = paired_proxy(5);
        let mut buf = [0u8; MAX_ENTRY_ENCODED_LEN];
        let n = p.entries()[0].encode(&mut buf).unwrap();
        let (e, used) = ProxyEntry::decode(&buf[..n]).unwrap();
        assert_eq!(used, n);
        assert_eq!(e, p.entries()[0]);

        let rm = Pairing {
            gpd_id: GpdId::SrcId(SRC),
            add_sink: false,
            remove_gpd: true,
            communication_mode: CommunicationMode::CommissionedGroupcast,
            gpd_fixed: false,
            mac_seq_cap: false,
            security_level: SecurityLevel::None,
            key_type: KeyType::NoKey,
            sink: PairingSink::None,
            device_id: None,
            frame_counter: None,
            key: None,
            assigned_alias: None,
            groupcast_radius: None,
        };
        let mut b = [0u8; 16];
        let len = rm.encode(&mut b).unwrap();
        let mut reply = None;
        let fx = p
            .on_client_command(0, client_cmd::PAIRING, &b[..len], 0, &mut reply)
            .unwrap();
        assert_eq!(&fx.groups[..], &[GroupChange::Leave(0x0B84)]);
        assert!(p.entries().is_empty());
    }

    #[test]
    fn alias_conflict_triggers_device_annce() {
        let mut p = paired_proxy(0);
        p.on_device_annce(0x4321, [1, 2, 3, 4, 5, 6, 7, 8], 0, 1000);
        let deadline = p.next_deadline().unwrap();
        assert!((D_MIN_MS..=D_MAX_MS).contains(&deadline));
        let tx = p.poll(deadline).unwrap();
        assert_eq!(tx.frame, TxFrame::DeviceAnnce { alias: 0x4321 });
        assert_eq!(tx.alias, Some((0x4321, 0)));
    }

    #[test]
    fn proxy_table_request_by_index() {
        let mut p = paired_proxy(0);
        let mut reply = None;
        p.on_client_command(
            0x0000,
            client_cmd::PROXY_TABLE_REQUEST,
            &[0x08, 0x00],
            0,
            &mut reply,
        )
        .unwrap();
        let tx = reply.unwrap();
        let TxFrame::GpCommand {
            command_id,
            payload,
        } = tx.frame
        else {
            panic!()
        };
        assert_eq!(command_id, server_cmd::PROXY_TABLE_RESPONSE);
        assert_eq!(&payload[..4], &[0x00, 1, 0, 1]);
        let (e, _) = ProxyEntry::decode(&payload[4..]).unwrap();
        assert_eq!(e.gpd_id, GpdId::SrcId(SRC));
    }

    #[test]
    fn tunneling_delay_bounds() {
        assert_eq!(tunneling_delay_ms(true, false, GppGpdLink(0)), D_MIN_MS);
        assert_eq!(
            tunneling_delay_ms(false, true, GppGpdLink(0)),
            D_MIN_MS + D_MAX_MS
        );
        assert_eq!(tunneling_delay_ms(false, false, GppGpdLink(59)), D_MIN_MS);
        assert_eq!(
            tunneling_delay_ms(false, false, GppGpdLink(0)),
            D_MIN_MS + 85
        );
    }

    #[cfg(feature = "proxy-bidirectional")]
    mod bidirectional {
        use super::*;
        use crate::frame::{FrameType, GpdfParams, parse_mac};
        use crate::security::{self, Direction};

        const CHANNEL: u8 = 25;

        fn bidi_proxy(fc: u32) -> P {
            let mut p = paired_proxy(fc);
            p.set_bidirectional_capable(true);
            p.set_operational_channel(CHANNEL);
            p
        }

        fn response(
            p: &mut P,
            gpd: GpdId,
            temp_master: u16,
            channel: u8,
            command_id: u8,
            payload: &[u8],
            now_ms: u32,
        ) -> Result<()> {
            let r = Response {
                gpd_id: gpd,
                transmit_on_endpoint_match: false,
                temp_master_short: temp_master,
                temp_master_tx_channel: channel - 11,
                command_id,
                payload,
            };
            let mut b = [0u8; 64];
            let n = r.encode(&mut b).unwrap();
            let mut reply = None;
            p.on_client_command(0x0000, client_cmd::RESPONSE, &b[..n], now_ms, &mut reply)
                .map(|_| ())
        }

        /// GPD → GPP frame (MAC header included, no FCS).
        fn uplink(
            src: Option<u32>,
            key: Option<&AesKey>,
            fc: u32,
            rx_after_tx: bool,
            command_id: u8,
            payload: &[u8],
        ) -> heapless::Vec<u8, 64> {
            let params = GpdfParams {
                frame_type: if src.is_some() {
                    FrameType::Data
                } else {
                    FrameType::Maintenance
                },
                auto_commissioning: false,
                src_id: src,
                security_level: if key.is_some() {
                    SecurityLevel::Encrypted
                } else {
                    SecurityLevel::None
                },
                individual_key: false,
                rx_after_tx,
                to_gpd: false,
                frame_counter: fc,
            };
            let mut out = [0u8; 64];
            let n = security::build_gpdf(
                &mut SoftwareAesProvider,
                key,
                &params,
                fc as u8,
                command_id,
                payload,
                &mut out,
            )
            .unwrap();
            heapless::Vec::from_slice(&out[..n]).unwrap()
        }

        fn gpdf(raw: &[u8]) -> Gpdf<'_> {
            let mac = parse_mac(raw).unwrap();
            parse(&raw[mac.payload_offset..], mac.seq, None).unwrap()
        }

        fn take(p: &mut P, raw: &[u8], now_ms: u32) -> Option<heapless::Vec<u8, 64>> {
            let mut out = [0u8; 64];
            let n =
                p.take_downlink(&mut SoftwareAesProvider, &gpdf(raw), now_ms, 0x42, &mut out)?;
            Some(heapless::Vec::from_slice(&out[..n]).unwrap())
        }

        #[test]
        fn functionality_and_notification_follow_capability() {
            let mut p = paired_proxy(1);
            let mut out = [0u8; 4];
            p.read_attribute(attr::GPP_FUNCTIONALITY, &mut out).unwrap();
            assert_eq!(u32::from_le_bytes(out) & functionality::BIDIRECTIONAL, 0);
            // A unidirectional proxy has no gpTxQueue room for anyone.
            let raw = uplink(Some(SRC), Some(&KEY_C0), 2, true, cmd::TOGGLE, &[]);
            p.on_gpdf(&mut SoftwareAesProvider, &gpdf(&raw), GppGpdLink(0), 0);
            let tx = (0..1000).find_map(|t| p.poll(t)).unwrap();
            let TxFrame::GpCommand { payload, .. } = &tx.frame else {
                panic!()
            };
            let n = Notification::decode(payload).unwrap();
            assert!(n.rx_after_tx && n.tx_queue_full && !n.bidirectional_cap);

            p.set_bidirectional_capable(true);
            p.read_attribute(attr::GPP_FUNCTIONALITY, &mut out).unwrap();
            assert_ne!(u32::from_le_bytes(out) & functionality::BIDIRECTIONAL, 0);
            let raw = uplink(Some(SRC), Some(&KEY_C0), 3, true, cmd::TOGGLE, &[]);
            p.on_gpdf(&mut SoftwareAesProvider, &gpdf(&raw), GppGpdLink(0), 2000);
            let tx = (2000..3000).find_map(|t| p.poll(t)).unwrap();
            let TxFrame::GpCommand { payload, .. } = &tx.frame else {
                panic!()
            };
            let n = Notification::decode(payload).unwrap();
            assert!(n.rx_after_tx && !n.tx_queue_full && n.bidirectional_cap);
        }

        #[test]
        fn unsupported_temp_master_appointments_fail_explicitly() {
            let gpd = GpdId::SrcId(SRC);
            let mut p = paired_proxy(1);
            assert_eq!(
                response(&mut p, gpd, 0x1234, CHANNEL, 0xF0, &[0x98], 0),
                Err(Error::Unsupported)
            );
            p.set_bidirectional_capable(true);
            // No operational channel known yet.
            assert_eq!(
                response(&mut p, gpd, 0x1234, CHANNEL, 0xF0, &[0x98], 0),
                Err(Error::Unsupported)
            );
            p.set_operational_channel(CHANNEL);
            // Delivery on another channel needs a channel switch (proxy-full).
            assert_eq!(
                response(&mut p, gpd, 0x1234, 15, 0xF0, &[0x98], 0),
                Err(Error::Unsupported)
            );
            assert_eq!(
                response(&mut p, gpd, 0x1234, CHANNEL, 0xF0, &[0x98], 0),
                Ok(())
            );
        }

        #[test]
        fn protected_trigger_gets_sealed_reply_with_trigger_counter() {
            let mut p = bidi_proxy(1);
            let gpd = GpdId::SrcId(SRC);
            response(&mut p, gpd, 0x1234, CHANNEL, 0xF0, &[0x98], 0).unwrap();
            assert!(p.entries()[0].first_to_forward);

            // Without rxAfterTx the GPD is not listening.
            let plain = uplink(Some(SRC), Some(&KEY_C0), 2, false, cmd::TOGGLE, &[]);
            assert!(take(&mut p, &plain, 10).is_none());
            // A forged MIC does not release the queued GPDF.
            let mut forged = uplink(Some(SRC), Some(&KEY_C0), 3, true, cmd::TOGGLE, &[]);
            let last = forged.len() - 1;
            forged[last] ^= 1;
            assert!(take(&mut p, &forged, 10).is_none());
            // A stale frame counter neither.
            let stale = uplink(Some(SRC), Some(&KEY_C0), 1, true, cmd::TOGGLE, &[]);
            assert!(take(&mut p, &stale, 10).is_none());

            let trigger = uplink(Some(SRC), Some(&KEY_C0), 7, true, cmd::TOGGLE, &[]);
            let down = take(&mut p, &trigger, 20).unwrap();
            assert_eq!(&down[..7], &[0x01, 0x08, 0x42, 0xFF, 0xFF, 0xFF, 0xFF]);
            let f = gpdf(&down);
            assert_eq!(f.header.frame_type, FrameType::Data);
            assert!(f.header.to_gpd && !f.header.rx_after_tx);
            assert_eq!(f.header.gpd_id, Some(gpd));
            assert_eq!(f.header.security_level, SecurityLevel::Encrypted);
            assert_eq!(f.header.frame_counter, 7);
            let clear =
                security::open(&mut SoftwareAesProvider, &KEY_C0, &f, Direction::ToGpd).unwrap();
            assert_eq!(&clear[..], &[0xF0, 0x98]);
            // Sent once; the trigger is still forwarded to the sinks.
            assert!(take(&mut p, &trigger, 20).is_none());
            assert_eq!(
                p.on_gpdf(&mut SoftwareAesProvider, &gpdf(&trigger), GppGpdLink(0), 20),
                GpdfOutcome::Notification
            );
        }

        #[test]
        fn unprotected_commissioning_gets_clear_reply() {
            let mut p = bidi_proxy(1);
            let gpd = GpdId::SrcId(0x6D6B_F300);
            response(&mut p, gpd, 0x1234, CHANNEL, 0xF0, &[0x98], 0).unwrap();
            let trigger = uplink(
                Some(0x6D6B_F300),
                None,
                0,
                true,
                cmd::COMMISSIONING,
                &[0x02],
            );
            let down = take(&mut p, &trigger, 5).unwrap();
            let f = gpdf(&down);
            assert!(f.header.to_gpd);
            assert_eq!(f.header.security_level, SecurityLevel::None);
            assert_eq!(f.header.gpd_id, Some(gpd));
            assert_eq!(f.payload, &[0xF0, 0x98]);
        }

        #[test]
        fn channel_request_gets_maintenance_channel_configuration() {
            let mut p = bidi_proxy(1);
            // zigpy: GP Response for GPD SrcID 0, transmitted on the
            // requested channel (here the operational one).
            response(&mut p, GpdId::SrcId(0), 0x1234, CHANNEL, 0xF3, &[0x1E], 0).unwrap();
            let req = uplink(None, None, 0, false, cmd::CHANNEL_REQUEST, &[0xEE]);
            let down = take(&mut p, &req, 5).unwrap();
            assert_eq!(&down[7..], &[0x0D, 0xF3, 0x1E]);
        }

        #[test]
        fn queue_replaces_fills_and_expires() {
            let mut p = bidi_proxy(1);
            for i in 0..GP_TX_QUEUE_LEN as u32 {
                response(
                    &mut p,
                    GpdId::SrcId(0x100 + i),
                    0x1234,
                    CHANNEL,
                    0xF0,
                    &[0],
                    0,
                )
                .unwrap();
            }
            // ENTRY_REPLACED for a GPD already queued.
            response(&mut p, GpdId::SrcId(0x100), 0x1234, CHANNEL, 0xF0, &[1], 0).unwrap();
            assert_eq!(
                response(&mut p, GpdId::SrcId(0x200), 0x1234, CHANNEL, 0xF0, &[0], 0),
                Err(Error::BufferFull)
            );
            let t = uplink(Some(0x100), None, 0, true, cmd::COMMISSIONING, &[0x02]);
            let down = take(&mut p, &t, 1).unwrap();
            assert_eq!(gpdf(&down).payload, &[0xF0, 1]);
            // Lifetime elapsed: queued GPDFs are dropped, room again.
            let late = DEFAULT_TX_QUEUE_LIFETIME_MS;
            let t = uplink(Some(0x101), None, 0, true, cmd::COMMISSIONING, &[0x02]);
            assert!(take(&mut p, &t, late).is_none());
            response(
                &mut p,
                GpdId::SrcId(0x200),
                0x1234,
                CHANNEL,
                0xF0,
                &[0],
                late,
            )
            .unwrap();
        }

        #[test]
        fn response_for_other_temp_master_drops_our_entry() {
            let mut p = bidi_proxy(1);
            let gpd = GpdId::SrcId(SRC);
            response(&mut p, gpd, 0x1234, CHANNEL, 0xF0, &[0x98], 0).unwrap();
            assert!(p.entries()[0].first_to_forward);
            response(&mut p, gpd, 0x5678, CHANNEL, 0xF0, &[0x98], 1).unwrap();
            assert!(!p.entries()[0].first_to_forward);
            let trigger = uplink(Some(SRC), Some(&KEY_C0), 7, true, cmd::TOGGLE, &[]);
            assert!(take(&mut p, &trigger, 2).is_none());
        }
    }
}
