//! Green Power combo on the ZRHP network co-processor (`green-power-ncp`,
//! spec `docs/book/src/advanced/host-protocol.md` 7.11).
//!
//! The coordinator runs the same proxy and sink as the router combo; this
//! module only adds what a host-driven coordinator needs:
//!
//! * **One sink authority.** [`ZigbeeDevice::gp_ncp_configure`] selects
//!   `NCP_SINK` (the runtime's sink commissions, pairs and filters) or
//!   `HOST_SINK` (the host runs its own sink). In `HOST_SINK` mode the
//!   runtime's sink is idle: GPDFs heard directly go only to the proxy, GP
//!   server frames on endpoint 242 (client → server: GP Notification, GP
//!   Commissioning Notification, sink attribute access, ...) go to the host
//!   queue of [`crate::host_passthrough`], and the local proxy's GP
//!   Notifications and Commissioning Notifications are copied to the host
//!   ([`GpNcpEvent::LocalProxy`]); one addressed to this node itself is not
//!   transmitted.
//! * **Frame reports.** Every frame the sink accepted (authenticated, fresh,
//!   not a duplicate) becomes a [`GpFrameReport`] with its security
//!   metadata and the decrypted command; drops are only counted
//!   ([`GpNcpCounters`]).
//! * **Host-managed Sink Table.** Import without GP Pairing, removal with or
//!   without RemoveGPD, and host-originated GP client commands (Pairing,
//!   Proxy Commissioning Mode, Response) looped back to the local proxy,
//!   because a broadcast never comes back to its sender.
//!
//! The NCP composes exactly one Zigbee device, so the mode, the report queue
//! and the counters are a process-wide singleton, as the host queue of
//! [`crate::host_passthrough`] is. [`ZigbeeDevice::gp_ncp_reset`] applies
//! the defaults: `NCP_SINK`, `gpsSecurityLevel` 0x00 (ZRHP decision N1,
//! unlike the router combo's 0x01) and no local translation endpoint, so
//! every accepted GPD command reaches the host.

use core::cell::RefCell;

use critical_section::Mutex;
use heapless::{Deque, Vec};
use zigbee_aps::apsde::ApsdeDataIndication;
use zigbee_aps::{ApsAddress, ApsAddressMode};
use zigbee_gp::cluster::{CommissioningNotification, Notification, ProxyInfo, server_cmd};
use zigbee_gp::commissioning::CommissioningPayload;
use zigbee_gp::frame::{Gpdf, MAX_GPD_PAYLOAD};
use zigbee_gp::proxy::{DestAddr, GpTx, MAX_TX_PAYLOAD, TxFrame};
use zigbee_gp::security::{self, Direction};
use zigbee_gp::sink::{ImportError, SinkConfig, SinkDrop, SinkEntry, SinkEvent, SinkOutcome};
use zigbee_gp::types::{GpdId, KeyType, SecurityLevel, cmd};
use zigbee_gp::{GP_CLUSTER_ID, GP_ENDPOINT, GP_PROFILE_ID};
use zigbee_mac::MacDriver;
use zigbee_types::{IeeeAddress, MacAddress};

use super::{
    ZCL_CLUSTER_SPECIFIC, ZCL_DISABLE_DEFAULT_RESPONSE, ZCL_SERVER_TO_CLIENT, gp_log_sink_command,
    link_from_lqi, parse_zcl,
};
use crate::ZigbeeDevice;
use crate::role::DeviceRole;

/// Queued reports, sink events and local proxy copies for the host.
pub const REPORT_QUEUE_LEN: usize = 6;
/// Largest local proxy frame copied to the host: ZCL header and the GP
/// command.
pub const MAX_LOCAL_FRAME: usize = 3 + MAX_TX_PAYLOAD;
/// [`SinkDrop`] reasons, numbered 1..=13 as in `EVT_GP_COMMISSIONING_FAILED`.
pub const SINK_DROP_REASONS: usize = 13;

/// [`GpFrameReport::origin`].
pub mod origin {
    /// GPDF heard directly.
    pub const DIRECT: u8 = 1;
    /// GP Notification from a proxy.
    pub const NOTIFICATION: u8 = 2;
    /// GP Commissioning Notification from a proxy.
    pub const COMMISSIONING_NOTIFICATION: u8 = 3;
}

/// [`GpFrameReport::flags`].
pub mod frame_flags {
    /// The GPD opened a receive window after the frame.
    pub const RX_AFTER_TX: u8 = 0x01;
    /// The payload was encrypted on air and is reported decrypted.
    pub const DECRYPTED: u8 = 0x02;
    /// The Commissioning payload carries a TC-LK wrapped key.
    pub const KEY_WRAPPED: u8 = 0x04;
}

/// A frame the sink accepted (`EVT_GP_FRAME`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpFrameReport {
    /// [`origin`].
    pub origin: u8,
    /// [`sink_result`] of the sink outcome.
    pub result: u8,
    pub gpd_id: GpdId,
    /// GPD DeviceID (`0xFF` unknown).
    pub device_id: u8,
    /// GPD CommandID (`0xFF` when encrypted and not decrypted).
    pub command_id: u8,
    /// Security frame counter (the MAC sequence number when unprotected).
    pub frame_counter: u32,
    /// 0..=3.
    pub security_level: u8,
    /// [`KeyType`] bits.
    pub key_type: u8,
    /// [`frame_flags`].
    pub flags: u8,
    /// Forwarding proxy, `0xFFFF` for a GPDF heard directly.
    pub proxy_nwk: u16,
    /// GPP-GPD link byte (`0xFF` unknown).
    pub gpp_gpd_link: u8,
    /// Not provided by the MAC: always -128 (unknown).
    pub rssi: i8,
    /// LQI of the GPDF, or of the proxy's APS frame.
    pub lqi: u8,
    /// Command payload (without the CommandID).
    pub payload: Vec<u8, MAX_GPD_PAYLOAD>,
}

/// A GP Notification or Commissioning Notification of the local proxy, for
/// a host sink (`EVT_APS_DATA_INDICATION` with the local flag).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpLocalFrame {
    /// `0x01` group, `0x02` unicast, `0x0F` broadcast.
    pub dst_mode: u8,
    pub dst: u16,
    /// This node.
    pub src_nwk: u16,
    /// ZCL frame (client → server, cluster-specific).
    pub asdu: Vec<u8, MAX_LOCAL_FRAME>,
}

/// Something for the host.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GpNcpEvent {
    /// A frame the sink accepted.
    Frame(GpFrameReport),
    /// A sink event other than a GPD command (those are reported as frames).
    Sink(SinkEvent),
    /// Local proxy output in `HOST_SINK` mode.
    LocalProxy(GpLocalFrame),
}

/// Diagnostic counters (ZRHP DIAG group `0x08xx`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GpNcpCounters {
    /// Frames the sink dropped, at index [`sink_drop_code`] - 1.
    pub sink_drops: [u32; SINK_DROP_REASONS],
    /// Reports and copies lost because the host did not drain the queue.
    pub reports_dropped: u32,
    /// Local proxy frames copied to a host sink.
    pub local_copies: u32,
    /// GP server frames handed to a host sink.
    pub host_sink_frames: u32,
    /// Host transmissions refused because they belong to the NCP sink.
    pub conflicts: u32,
}

/// `EVT_GP_FRAME.result` of an accepted frame; `None` for a drop.
pub const fn sink_result(outcome: SinkOutcome) -> Option<u8> {
    match outcome {
        SinkOutcome::Translated => Some(0),
        SinkOutcome::PassedThrough => Some(1),
        SinkOutcome::Commissioned => Some(2),
        SinkOutcome::CommissioningReplySent => Some(3),
        SinkOutcome::Decommissioned => Some(4),
        SinkOutcome::Dropped(_) => None,
    }
}

/// `EVT_GP_COMMISSIONING_FAILED.reason` (1 Invalid … 13 QueueFull).
pub const fn sink_drop_code(reason: SinkDrop) -> u8 {
    match reason {
        SinkDrop::Invalid => 1,
        SinkDrop::NotCommissioning => 2,
        SinkDrop::NoEntry => 3,
        SinkDrop::SecurityMismatch => 4,
        SinkDrop::AuthFailed => 5,
        SinkDrop::Stale => 6,
        SinkDrop::Duplicate => 7,
        SinkDrop::SecurityPolicy => 8,
        SinkDrop::NoKey => 9,
        SinkDrop::NoTempMaster => 10,
        SinkDrop::TableFull => 11,
        SinkDrop::Unsupported => 12,
        SinkDrop::QueueFull => 13,
    }
}

/// The NCP sink's defaults: derived groupcast, exit on first pairing, the
/// 180 s window, `gpsSecurityLevel` 0x00 (levels 0, 2 and 3 are
/// commissioned, ZRHP decision N1) and no translation endpoint.
pub fn ncp_sink_config() -> SinkConfig {
    SinkConfig {
        security_level: 0x00,
        endpoint: None,
        ..SinkConfig::default()
    }
}

struct State {
    host_sink: bool,
    queue: Deque<GpNcpEvent, REPORT_QUEUE_LEN>,
    counters: GpNcpCounters,
    /// LQI of the last APS frame (the proxy hop of a GP Notification).
    last_aps_lqi: u8,
    /// (CommandID, CRC) of the last local proxy frame copied: one GPDF is
    /// tunnelled to every destination, the host needs it once.
    last_local: Option<(u8, u32)>,
}

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State {
    host_sink: false,
    queue: Deque::new(),
    counters: GpNcpCounters {
        sink_drops: [0; SINK_DROP_REASONS],
        reports_dropped: 0,
        local_copies: 0,
        host_sink_frames: 0,
        conflicts: 0,
    },
    last_aps_lqi: 0,
    last_local: None,
}));

fn with<T>(f: impl FnOnce(&mut State) -> T) -> T {
    critical_section::with(|cs| f(&mut STATE.borrow_ref_mut(cs)))
}

/// Whether the host is the sink (`HOST_SINK`).
pub fn host_sink() -> bool {
    with(|s| s.host_sink)
}

/// Diagnostic counters.
pub fn counters() -> GpNcpCounters {
    with(|s| s.counters)
}

/// Zero the diagnostic counters.
pub fn reset_counters() {
    with(|s| s.counters = GpNcpCounters::default());
}

/// Count a host transmission refused in `NCP_SINK` mode.
pub fn note_conflict() {
    with(|s| s.counters.conflicts = s.counters.conflicts.saturating_add(1));
}

fn count_drop(reason: SinkDrop) {
    let index = usize::from(sink_drop_code(reason)) - 1;
    with(|s| s.counters.sink_drops[index] = s.counters.sink_drops[index].saturating_add(1));
}

fn push(event: GpNcpEvent) {
    with(|s| {
        if s.queue.push_back(event).is_err() {
            s.counters.reports_dropped = s.counters.reports_dropped.saturating_add(1);
        }
    });
}

/// Whether a GPD Commissioning payload carries a TC-LK wrapped key.
fn key_wrapped(command_id: u8, payload: &[u8]) -> bool {
    command_id == cmd::COMMISSIONING
        && CommissioningPayload::decode(payload).is_ok_and(|p| {
            p.security
                .and_then(|s| s.key)
                .is_some_and(|(_, mic)| mic.is_some())
        })
}

impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// NCP defaults: `NCP_SINK`, [`ncp_sink_config`], no queued reports and
    /// zeroed counters. The Sink and Proxy Tables are kept.
    pub fn gp_ncp_reset(&mut self) {
        with(|s| {
            s.host_sink = false;
            s.queue.clear();
            s.counters = GpNcpCounters::default();
            s.last_local = None;
        });
        self.green_power.sink.set_config(ncp_sink_config());
    }

    /// Select the sink authority and the NCP sink's configuration (its
    /// endpoint is always `None`: no local translation). Entering
    /// `HOST_SINK` leaves the NCP sink's commissioning mode.
    pub fn gp_ncp_configure(&mut self, host_sink: bool, mut config: SinkConfig) {
        config.endpoint = None;
        if host_sink {
            self.green_power.sink.exit_commissioning();
        }
        self.green_power.sink.set_config(config);
        with(|s| {
            s.host_sink = host_sink;
            s.last_local = None;
        });
    }

    /// Whether the host is the sink.
    pub fn gp_ncp_host_sink(&self) -> bool {
        host_sink()
    }

    /// Add a Sink Table entry from the host or a backup
    /// ([`zigbee_gp::sink::Sink::import`]); GP Pairing only with `announce`.
    pub fn gp_ncp_import(&mut self, entry: SinkEntry, announce: bool) -> Result<(), ImportError> {
        self.gp_sink_sync();
        let result = self.green_power.sink.import(entry, announce);
        self.gp_sink_apply_groups();
        result
    }

    /// Remove a GPD from the Sink Table, broadcasting GP Pairing with
    /// RemoveGPD only with `announce`. Returns whether it was paired.
    pub fn gp_ncp_remove(&mut self, gpd: &GpdId, announce: bool) -> bool {
        if announce {
            self.gp_sink_sync();
            return self.gp_sink_remove(gpd);
        }
        let removed = self.green_power.sink.remove_quietly(gpd);
        self.gp_sink_apply_groups();
        removed
    }

    /// Next report, local proxy copy or sink event for the host. GPD
    /// commands were already reported as frames when they were accepted.
    pub fn gp_ncp_take_event(&mut self) -> Option<GpNcpEvent> {
        if let Some(event) = with(|s| s.queue.pop_front()) {
            return Some(event);
        }
        while let Some(event) = self.green_power.sink.take_event() {
            if !matches!(event, SinkEvent::GpdCommand { .. }) {
                return Some(GpNcpEvent::Sink(event));
            }
        }
        None
    }

    /// Hand a GP client command (server → client) the host transmits to
    /// this node's own proxy: GP Pairing, Proxy Commissioning Mode or GP
    /// Response (a Response appointing this node TempMaster queues the
    /// GPDF in its gpTxQueue). Nothing happens while not on a network.
    pub fn gp_ncp_loopback(&mut self, command_id: u8, payload: &[u8]) {
        if !self.gp_active() {
            return;
        }
        let now_ms = self.gp_now_ms();
        let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
        self.green_power.proxy.set_own_nwk(own_nwk);
        #[cfg(feature = "green-power-bidirectional")]
        self.gp_sync_bidirectional();
        let mut reply = None;
        match self
            .green_power
            .proxy
            .on_client_command(own_nwk, command_id, payload, now_ms, &mut reply)
        {
            Ok(effects) => self.gp_apply_effects(&effects),
            Err(error) => log::warn!(
                "[GP] host GP command 0x{:02X} refused by the local proxy: {:?}",
                command_id,
                error
            ),
        }
    }

    /// GP server frames for a host sink: `Some(true)` when the frame was
    /// handed to the host (or dropped as unauthenticated) and must not
    /// reach the runtime's sink; `None` to continue as usual.
    pub(crate) fn gp_ncp_capture(
        &mut self,
        indication: &ApsdeDataIndication<'_>,
        mac_source: &MacAddress,
        nwk_secured: bool,
        src_ieee: Option<IeeeAddress>,
    ) -> Option<bool> {
        with(|s| s.last_aps_lqi = indication.lqi);
        if indication.profile_id != GP_PROFILE_ID
            || indication.cluster_id != GP_CLUSTER_ID
            || !host_sink()
        {
            return None;
        }
        let addressed = match (indication.dst_addr_mode, indication.dst_address) {
            (ApsAddressMode::Group, ApsAddress::Group(group)) => self
                .bdb
                .zdo()
                .aps()
                .group_member_endpoints(group)
                .contains(&GP_ENDPOINT),
            _ => matches!(indication.dst_endpoint, GP_ENDPOINT | 0xFF),
        };
        let fc = *indication.payload.first()?;
        if !addressed || fc & ZCL_SERVER_TO_CLIENT != 0 {
            // Server → client frames are the local proxy's.
            return None;
        }
        if self.bdb.zdo().nwk().nib().security_enabled && !nwk_secured {
            return Some(true);
        }
        // Another proxy's GP Notification still cancels our own pending
        // forward of the same GPDF.
        if let (ApsAddress::Short(src), Some((fc, _, command, payload))) =
            (indication.src_address, parse_zcl(indication.payload))
            && fc & 0x03 == ZCL_CLUSTER_SPECIFIC
        {
            let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
            self.green_power.proxy.set_own_nwk(own_nwk);
            let _ = self
                .green_power
                .proxy
                .on_server_command(src.0, command, payload);
        }
        crate::host_passthrough::copy(indication, mac_source, nwk_secured, src_ieee);
        with(|s| s.counters.host_sink_frames = s.counters.host_sink_frames.saturating_add(1));
        Some(true)
    }

    // ── Seams of `green_power` ──────────────────────────────────────

    /// The NCP sink is idle while the host is the sink.
    pub(super) fn gp_sink_enabled(&self) -> bool {
        !host_sink()
    }

    /// The proxy's next due frame; in `HOST_SINK` mode its GP
    /// Notifications and Commissioning Notifications are also copied to the
    /// host, and one addressed to this node is only copied.
    pub(super) fn gp_proxy_poll(&mut self, now_ms: u32) -> Option<GpTx> {
        loop {
            let tx = self.green_power.proxy.poll(now_ms)?;
            if !host_sink() {
                return Some(tx);
            }
            let TxFrame::GpCommand {
                command_id,
                payload,
            } = &tx.frame
            else {
                return Some(tx);
            };
            if !matches!(
                *command_id,
                server_cmd::NOTIFICATION | server_cmd::COMMISSIONING_NOTIFICATION
            ) {
                return Some(tx);
            }
            let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
            let key = (*command_id, crate::security_journal::crc32(payload));
            if with(|s| s.last_local.replace(key)) != Some(key) {
                let (dst_mode, dst) = match tx.dest {
                    DestAddr::Group(group) => (0x01, group),
                    DestAddr::Broadcast(address) => (0x0F, address),
                    DestAddr::Unicast { nwk, .. } => (0x02, nwk),
                };
                let tsn = self.green_power.next_tsn();
                let mut asdu = Vec::new();
                if asdu
                    .extend_from_slice(&[
                        ZCL_CLUSTER_SPECIFIC | ZCL_DISABLE_DEFAULT_RESPONSE,
                        tsn,
                        *command_id,
                    ])
                    .is_ok()
                    && asdu.extend_from_slice(payload).is_ok()
                {
                    with(|s| s.counters.local_copies = s.counters.local_copies.saturating_add(1));
                    push(GpNcpEvent::LocalProxy(GpLocalFrame {
                        dst_mode,
                        dst,
                        src_nwk: own_nwk,
                        asdu,
                    }));
                }
            }
            if matches!(tx.dest, DestAddr::Unicast { nwk, .. } if nwk == own_nwk) {
                continue;
            }
            return Some(tx);
        }
    }

    /// Report a GPDF heard directly that the sink accepted.
    pub(super) fn gp_sink_report_gpdf(&mut self, frame: &Gpdf<'_>, outcome: SinkOutcome, lqi: u8) {
        let result = match outcome {
            SinkOutcome::Dropped(reason) => {
                count_drop(reason);
                return;
            }
            accepted => sink_result(accepted).unwrap_or(1),
        };
        let h = &frame.header;
        let Some(gpd_id) = h.gpd_id else {
            return;
        };
        let level = h.security_level;
        let entry = self
            .green_power
            .sink
            .entries()
            .iter()
            .find(|e| e.gpd_id.matches(&gpd_id));
        let key_type = match entry {
            Some(e) if level.is_protected() => e.key_type,
            _ if !level.is_protected() => KeyType::NoKey,
            _ if h.individual_key => KeyType::IndividualKey,
            _ => KeyType::GpdGroupKey,
        };
        let key = entry.and_then(|e| e.key);
        let mut report = GpFrameReport {
            origin: origin::DIRECT,
            result,
            gpd_id,
            device_id: entry.map_or(0xFF, |e| e.device_id),
            command_id: 0xFF,
            frame_counter: if level.is_protected() {
                h.frame_counter
            } else {
                u32::from(frame.mac_seq)
            },
            security_level: level as u8,
            key_type: key_type as u8,
            flags: if h.rx_after_tx {
                frame_flags::RX_AFTER_TX
            } else {
                0
            },
            proxy_nwk: 0xFFFF,
            gpp_gpd_link: link_from_lqi(lqi).0,
            rssi: -128,
            lqi,
            payload: Vec::new(),
        };
        // The command as the sink saw it: from its GPD command event, or the
        // frame itself (decrypted with the paired key when encrypted).
        let from_event = matches!(
            outcome,
            SinkOutcome::PassedThrough | SinkOutcome::Translated
        ) && self.gp_ncp_fill_from_event(&mut report);
        if !from_event {
            if level == SecurityLevel::Encrypted {
                let aes = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
                if let Some(clear) =
                    key.and_then(|k| security::open(aes, &k, frame, Direction::FromGpd).ok())
                {
                    report.flags |= frame_flags::DECRYPTED;
                    fill_command(&mut report, &clear);
                } else if outcome == SinkOutcome::Decommissioned {
                    // Only Decommissioning decommissions; it has no payload.
                    report.command_id = cmd::DECOMMISSIONING;
                } else {
                    let _ = report.payload.extend_from_slice(frame.payload);
                }
            } else {
                fill_command(&mut report, frame.payload);
            }
        }
        push(GpNcpEvent::Frame(report));
    }

    /// Log, and report a GP Notification or Commissioning Notification the
    /// sink accepted.
    pub(super) fn gp_sink_command_outcome(
        &mut self,
        src: u16,
        command: u8,
        payload: &[u8],
        outcome: SinkOutcome,
    ) {
        gp_log_sink_command(src, command, outcome);
        let result = match outcome {
            SinkOutcome::Dropped(reason) => {
                count_drop(reason);
                return;
            }
            accepted => sink_result(accepted).unwrap_or(1),
        };
        let lqi = with(|s| s.last_aps_lqi);
        let link_byte = |info: ProxyInfo| match info {
            ProxyInfo::Link { link, .. } => link.0,
            ProxyInfo::Distance { distance, .. } => distance,
            ProxyInfo::None => 0xFF,
        };
        let mut report = match command {
            server_cmd::NOTIFICATION => {
                let Ok(n) = Notification::decode(payload) else {
                    return;
                };
                let mut report = notification_report(
                    origin::NOTIFICATION,
                    result,
                    (n.gpd_id, n.security_level, n.key_type, n.frame_counter),
                    n.rx_after_tx,
                    (src, link_byte(n.proxy_info), lqi),
                );
                fill_command_parts(&mut report, n.command_id, n.payload.unwrap_or_default());
                report
            }
            server_cmd::COMMISSIONING_NOTIFICATION => {
                let Ok(n) = CommissioningNotification::decode(payload) else {
                    return;
                };
                let mut report = notification_report(
                    origin::COMMISSIONING_NOTIFICATION,
                    result,
                    (n.gpd_id, n.security_level, n.key_type, n.frame_counter),
                    n.rx_after_tx,
                    (src, link_byte(n.proxy_info), lqi),
                );
                let raw = n.payload.unwrap_or_default();
                if n.security_failed.is_none() {
                    fill_command_parts(&mut report, n.command_id, raw);
                } else {
                    // A proxy without the key tunnels the frame encrypted.
                    let _ = report.payload.extend_from_slice(raw);
                }
                report
            }
            _ => return,
        };
        if let Some(e) = self
            .green_power
            .sink
            .entries()
            .iter()
            .find(|e| e.gpd_id.matches(&report.gpd_id))
        {
            report.device_id = e.device_id;
        }
        if matches!(
            outcome,
            SinkOutcome::PassedThrough | SinkOutcome::Translated
        ) {
            self.gp_ncp_fill_from_event(&mut report);
        }
        push(GpNcpEvent::Frame(report));
    }

    /// Take the command, payload and DeviceID from the sink's newest event
    /// when it is the GPD command of `report`'s GPD (GPD command events are
    /// not reported separately).
    fn gp_ncp_fill_from_event(&self, report: &mut GpFrameReport) -> bool {
        let Some(SinkEvent::GpdCommand {
            gpd_id,
            device_id,
            command_id,
            payload,
        }) = self.green_power.sink.events().last()
        else {
            return false;
        };
        if !gpd_id.matches(&report.gpd_id) {
            return false;
        }
        report.device_id = *device_id;
        report.command_id = *command_id;
        report.payload.clear();
        let _ = report.payload.extend_from_slice(payload);
        if report.security_level == SecurityLevel::Encrypted as u8 {
            report.flags |= frame_flags::DECRYPTED;
        }
        true
    }
}

/// Split a clear GPD command (CommandID first) into the report.
fn fill_command(report: &mut GpFrameReport, clear: &[u8]) {
    if let Some((&command_id, payload)) = clear.split_first() {
        fill_command_parts(report, command_id, payload);
    }
}

fn fill_command_parts(report: &mut GpFrameReport, command_id: u8, payload: &[u8]) {
    report.command_id = command_id;
    report.payload.clear();
    let _ = report.payload.extend_from_slice(payload);
    if command_id == cmd::COMMISSIONING {
        if report.device_id == 0xFF {
            report.device_id = payload.first().copied().unwrap_or(0xFF);
        }
        if key_wrapped(command_id, payload) {
            report.flags |= frame_flags::KEY_WRAPPED;
        }
    }
}

/// A report of a proxy-forwarded frame: `security` is (GPD, level, key
/// type, counter), `hop` is (proxy, GPP-GPD link byte, APS LQI).
fn notification_report(
    origin: u8,
    result: u8,
    security: (GpdId, SecurityLevel, KeyType, u32),
    rx_after_tx: bool,
    hop: (u16, u8, u8),
) -> GpFrameReport {
    let (gpd_id, level, key_type, frame_counter) = security;
    let (proxy_nwk, gpp_gpd_link, lqi) = hop;
    GpFrameReport {
        origin,
        result,
        gpd_id,
        device_id: 0xFF,
        command_id: 0xFF,
        frame_counter,
        security_level: level as u8,
        key_type: if level.is_protected() {
            key_type as u8
        } else {
            KeyType::NoKey as u8
        },
        flags: if rx_after_tx {
            frame_flags::RX_AFTER_TX
        } else {
            0
        },
        proxy_nwk,
        gpp_gpd_link,
        rssi: -128,
        lqi,
        payload: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn outcome_and_drop_codes_follow_the_spec_numbering() {
        assert_eq!(sink_result(SinkOutcome::Translated), Some(0));
        assert_eq!(sink_result(SinkOutcome::Decommissioned), Some(4));
        assert_eq!(sink_result(SinkOutcome::Dropped(SinkDrop::Stale)), None);
        assert_eq!(sink_drop_code(SinkDrop::Invalid), 1);
        assert_eq!(sink_drop_code(SinkDrop::Duplicate), 7);
        assert_eq!(
            usize::from(sink_drop_code(SinkDrop::QueueFull)),
            SINK_DROP_REASONS
        );
    }

    #[test]
    fn ncp_defaults_commission_level_zero_without_translation() {
        let config = ncp_sink_config();
        assert_eq!(config.security_level, 0x00);
        assert_eq!(config.endpoint, None);
        assert_eq!(SinkConfig::default().security_level, 0x01, "router combo");
    }

    #[test]
    fn commissioning_payloads_report_a_wrapped_key() {
        use zigbee_gp::commissioning::CommissioningSecurity;
        let encode = |mic: Option<u32>| {
            let p = CommissioningPayload {
                device_id: 0x02,
                mac_seq_num_capability: true,
                rx_on_capability: false,
                pan_id_request: false,
                security_key_request: false,
                fixed_location: false,
                security: Some(CommissioningSecurity {
                    level: SecurityLevel::Encrypted,
                    key_type: KeyType::IndividualKey,
                    key: Some(([7; 16], mic)),
                    outgoing_counter: Some(1),
                }),
                app_info: None,
            };
            let mut b = [0u8; 64];
            let n = p.encode(&mut b).unwrap();
            Vec::<u8, 64>::from_slice(&b[..n]).unwrap()
        };
        assert!(key_wrapped(cmd::COMMISSIONING, &encode(Some(0x1234))));
        assert!(!key_wrapped(cmd::COMMISSIONING, &encode(None)));
        assert!(!key_wrapped(cmd::SUCCESS, &encode(Some(0x1234))));
        let mut report = notification_report(
            origin::NOTIFICATION,
            2,
            (GpdId::SrcId(1), SecurityLevel::None, KeyType::NoKey, 3),
            true,
            (0x1234, 0x55, 200),
        );
        fill_command(&mut report, &[cmd::COMMISSIONING, 0x02, 0x81]);
        assert_eq!(
            (report.command_id, report.device_id, report.flags),
            (cmd::COMMISSIONING, 0x02, frame_flags::RX_AFTER_TX)
        );
        assert_eq!(report.payload.as_slice(), &[0x02, 0x81]);
    }
}
