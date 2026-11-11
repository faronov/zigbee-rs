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
    self, CommissioningNotification, GppGpdLink, Notification, NotificationResponse, Pairing,
    PairingSink, ProxyCommissioningMode, ProxyInfo, ProxyTableRequest, Response, client_cmd,
    exit_mode, server_cmd,
};
use crate::codec::{Reader, Writer};
use crate::dup::DuplicateFilter;
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

/// Proxy configuration selected by the product.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProxyConfig {
    /// Commissioning window used when a Proxy Commissioning Mode command
    /// requests neither a window nor an exit mode.
    pub default_commissioning_window_s: u16,
    /// Persist a forwarded frame counter only after it advanced by this many
    /// values since the last persisted value (flash wear trade-off; sinks run
    /// their own freshness check). 1 persists every update.
    pub counter_persist_stride: u32,
}

impl Default for ProxyConfig {
    fn default() -> Self {
        Self {
            default_commissioning_window_s: DEFAULT_COMMISSIONING_WINDOW_S,
            counter_persist_stride: 16,
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

/// Effects of a received Green Power cluster command.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CommandEffects {
    /// GPEP group membership changes.
    pub groups: heapless::Vec<GroupChange, 4>,
    /// Commissioning mode was entered (true) or left (false).
    pub commissioning: Option<bool>,
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

    /// Proxy Table entries.
    pub fn entries(&self) -> &[ProxyEntry] {
        &self.table
    }

    /// Restore an entry from persistent storage.
    pub fn restore(&mut self, entry: ProxyEntry) -> Result<()> {
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

    /// Returns and clears the "table changed, persist it" flag.
    pub fn take_dirty(&mut self) -> bool {
        core::mem::take(&mut self.dirty)
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
        if let Some(c) = self.commissioning
            && c.deadline_ms.is_some_and(|d| due(now_ms, d))
        {
            self.commissioning = None;
        }
        let idx = self.pending.iter().position(|p| due(now_ms, p.due_ms))?;
        let p = &mut self.pending[idx];
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

    fn cancel(&mut self, gpd: &GpdId, counter: u32) {
        self.pending.retain(|p| {
            !p.origin
                .is_some_and(|(g, c)| g.matches(gpd) && c == counter)
        });
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
            tx_queue_full: false,
            bidirectional_cap: false,
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
            bidirectional_cap: false,
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
            tunneling_delay_ms(false, false, link)
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
                self.on_pairing(&p, &mut fx)?;
            }
            client_cmd::RESPONSE => {
                let r = Response::decode(payload)?;
                self.on_response(&r);
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
        if let Some(c) = self.commissioning
            && c.originator != src_nwk
        {
            // Only the sink that put us in commissioning mode controls it.
            return;
        }
        if !m.enter {
            if self.commissioning.take().is_some() {
                fx.commissioning = Some(false);
            }
            return;
        }
        let window = match (m.window_s, m.exit_mode) {
            (Some(w), _) => Some(w),
            (None, 0) => Some(self.config.default_commissioning_window_s),
            (None, _) => None,
        };
        self.commissioning = Some(CommissioningState {
            originator: src_nwk,
            deadline_ms: window.map(|w| now_ms.wrapping_add(w as u32 * 1000)),
            exit_on_first_pairing: m.exit_mode & exit_mode::ON_FIRST_PAIRING != 0,
            unicast: m.unicast,
        });
        fx.commissioning = Some(true);
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

    fn on_pairing(&mut self, p: &Pairing, fx: &mut CommandEffects) -> Result<()> {
        if let Some(c) = self.commissioning
            && c.exit_on_first_pairing
        {
            self.commissioning = None;
            fx.commissioning = Some(false);
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

        let i = match idx {
            Some(i) => i,
            None => {
                self.table
                    .push(ProxyEntry::new(p.gpd_id))
                    .map_err(|_| Error::BufferFull)?;
                self.table.len() - 1
            }
        };
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
            e.persisted_counter = fc;
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
        self.dirty = true;
        Ok(())
    }

    fn on_response(&mut self, r: &Response<'_>) {
        // Proxy Basic is unidirectional: it never acts as TempMaster. A
        // response appointing another proxy clears FirstToForward.
        if r.temp_master_short != self.own_nwk
            && let Some(i) = self.find(&r.gpd_id)
        {
            self.table[i].first_to_forward = false;
        }
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
        let budget = MAX_GP_COMMAND_LEN - 4;
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
    /// (A.3.5.2.1).
    pub fn on_server_command(&mut self, src_nwk: u16, command_id: u8, payload: &[u8]) {
        if src_nwk == self.own_nwk {
            return;
        }
        let (gpd, counter, rx_after_tx, other) = match command_id {
            server_cmd::NOTIFICATION => match Notification::decode(payload) {
                Ok(n) => (n.gpd_id, n.frame_counter, n.rx_after_tx, n.proxy_info),
                Err(_) => return,
            },
            server_cmd::COMMISSIONING_NOTIFICATION => {
                match CommissioningNotification::decode(payload) {
                    Ok(n) => (n.gpd_id, n.frame_counter, n.rx_after_tx, n.proxy_info),
                    Err(_) => return,
                }
            }
            _ => return,
        };
        if !rx_after_tx {
            self.cancel(&gpd, counter);
            return;
        }
        let own = self.own_nwk;
        self.pending.retain(|p| {
            let same = p
                .origin
                .is_some_and(|(g, c)| g.matches(&gpd) && c == counter);
            if !same || !p.rx_after_tx {
                return !same;
            }
            let (other_short, other_link) = match other {
                ProxyInfo::Link { gpp_short, link } => (gpp_short, link),
                ProxyInfo::Distance {
                    gpp_short,
                    distance,
                } => (gpp_short, GppGpdLink(distance)),
                ProxyInfo::None => return true,
            };
            let ours = (p.link.link_quality(), p.link.0 & 0x3F);
            let theirs = (other_link.link_quality(), other_link.0 & 0x3F);
            // Keep ours only if we are strictly better, or equal with the
            // lower short address.
            ours > theirs || (ours == theirs && own < other_short)
        });
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
    fn forwards_secured_gpdf_as_commissioned_groupcast_notification() {
        let mut p = paired_proxy(1);
        let mut aes = SoftwareAesProvider;
        let f = parse(&L3_SHARED, 0x20, None).unwrap();
        let out = p.on_gpdf(&mut aes, &f, GppGpdLink(0xC0 | 40), 1000);
        assert_eq!(out, GpdfOutcome::Notification);
        assert!(p.poll(1000 + D_MIN_MS - 1).is_none());
        let tx = p.poll(1000 + D_MIN_MS).unwrap();
        assert_eq!(tx.dest, DestAddr::Group(0x0B84));
        assert_eq!(tx.alias, Some((0x4321, 0x20u8.wrapping_sub(13))));
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
        assert_eq!(fx.commissioning, Some(true));
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
        assert_eq!(fx.commissioning, Some(false));
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
        p.poll(100 + 180_000);
        assert!(!p.in_commissioning_mode());
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
}
