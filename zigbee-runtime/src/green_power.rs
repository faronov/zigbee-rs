//! Green Power Proxy Basic integration (GP 1.1, docs-14-0563 "Green Power
//! Basic").
//!
//! The protocol state machine lives in [`zigbee_gp::proxy`]; this module only
//! connects it to the router runtime:
//!
//! * the receive demultiplexer hands NWK protocol-version-3 frames (GPDFs) to
//!   [`Proxy::on_gpdf`] instead of the Zigbee NWK layer;
//! * Green Power cluster frames for the GP endpoint (242, profile 0xA1E0) are
//!   routed to the proxy (GP Pairing, Proxy Commissioning Mode, Response,
//!   Notification Response, Proxy Table Request, client attribute reads), and
//!   overheard GP Notifications of other proxies cancel our own duplicate
//!   tunnelling;
//! * `Device_annce` feeds sink address updates and alias conflict detection;
//! * [`ZigbeeDevice::gp_poll`] transmits due GP Notifications / Commissioning
//!   Notifications / replies through APS, with the NWK alias the proxy
//!   selected (GP 1.1 A.3.6.3.3).
//!
//! The application owns the event loop: it calls [`ZigbeeDevice::gp_poll`]
//! every pass and bounds its receive timeout with
//! [`ZigbeeDevice::gp_next_wait_us`], because the tunnelling delay
//! (Dmin..Dmin+Dmax = 15..115 ms) is far below the runtime's one-second tick.
//!
//! Proxy Table persistence is the product's responsibility through
//! [`crate::gp_store`]; [`ZigbeeDevice::gp_take_dirty`] reports when the table
//! changed.
//!
//! With `green-power-sink` the router is also a GP sink (combo device,
//! [`zigbee_gp::sink`]): GP server commands on the GP endpoint (GP
//! Notification, GP Commissioning Notification, GP Sink Commissioning Mode,
//! GP Sink Table Request, server attribute reads and writes) go to the sink;
//! a GPDF heard directly is processed by the sink as well as by the proxy and
//! is never tunnelled to this node; translated GPD commands are delivered to
//! the configured local application endpoint through the ZCL dispatcher as
//! locally originated, non-unicast commands; the Sink Table shares the GP
//! journal of [`crate::gp_store`].

use zigbee_aps::ApsTxOptions;
use zigbee_aps::apsde::{ApsdeDataIndication, ApsdeDataRequest};
use zigbee_aps::{ApsAddress, ApsAddressMode};
use zigbee_gp::cluster::GppGpdLink;
use zigbee_gp::proxy::{
    CommandEffects, CommissioningEvent, CommissioningExit, DestAddr, GpTx, GpdfOutcome,
    GroupChange, MAX_TX_PAYLOAD, Proxy, ProxyConfig, TxFrame,
};
use zigbee_gp::{GP_CLUSTER_ID, GP_ENDPOINT, GP_PROFILE_ID};
use zigbee_mac::{MacDriver, McpsDataIndication};
use zigbee_nwk::DeviceType;
use zigbee_types::{MacAddress, ShortAddress};

use crate::ZigbeeDevice;
use crate::role::DeviceRole;

/// Proxy Table capacity of a router built with `green-power`.
pub const GP_PROXY_TABLE_ENTRIES: usize = 8;
/// Scheduled GP Notification / Commissioning Notification transmissions.
pub const GP_TX_QUEUE_DEPTH: usize = 4;

/// The router's Green Power proxy.
pub type GpProxy = Proxy<GP_PROXY_TABLE_ENTRIES, GP_TX_QUEUE_DEPTH>;

/// Sink Table capacity of a combo (`green-power-sink`): 8, or 16 with `ncp-capacity`.
#[cfg(feature = "green-power-sink")]
pub const GP_SINK_TABLE_ENTRIES: usize = crate::ncp_capacity(8, 16);
/// Queued sink transmissions (GP Pairing, Proxy Commissioning Mode, GP
/// Response, responses, alias `Device_annce`).
#[cfg(feature = "green-power-sink")]
pub const GP_SINK_TX_QUEUE_DEPTH: usize = 4;

/// The combo's Green Power sink.
#[cfg(feature = "green-power-sink")]
pub type GpSink = zigbee_gp::sink::Sink<GP_SINK_TABLE_ENTRIES, GP_SINK_TX_QUEUE_DEPTH>;

/// Largest GPDF the TempMaster sends to a GPD: 7-octet MAC header, NWK
/// header with SrcID and frame counter (10), command ID plus
/// [`zigbee_gp::proxy::MAX_DOWNLINK_PAYLOAD`], MIC (4).
#[cfg(feature = "green-power-bidirectional")]
const GP_DOWNLINK_FRAME_LEN: usize = 7 + 10 + 1 + zigbee_gp::proxy::MAX_DOWNLINK_PAYLOAD + 4;

/// A GPDF from the gpTxQueue, due in a GPD's receive window.
#[cfg(feature = "green-power-bidirectional")]
pub(crate) struct GpDownlink {
    frame: [u8; GP_DOWNLINK_FRAME_LEN],
    len: u8,
    /// Start instant ([`zigbee_mac::PlatformServices::monotonic_micros`]).
    at_us: u32,
    /// When the reply was built (`green-power-sink`: logged after the TX
    /// with the instant, to show the latency budget).
    #[cfg(feature = "green-power-sink")]
    built_us: u32,
}

/// The gpTxQueue GPDF armed in the radio ([`MacDriver::arm_gp_reply`]):
/// the radio's interrupts send it gpTxOffset after the GPD's next
/// unprotected `rxAfterTx` GPDF, however late the task handles that GPDF.
/// On the ESP32-H2 combo the task saw triggers 68–173 ms after they ended
/// during a commissioning burst (2026-11-17), far past the 20 ms instant.
#[cfg(feature = "green-power-bidirectional")]
pub(crate) struct GpRadioReply {
    /// The MAC cannot arm replies: only the task path is used.
    unsupported: bool,
    /// Bytes of the armed frame (MAC sequence number 0), 0 = none armed.
    len: u8,
    src_id: u32,
    frame: [u8; GP_DOWNLINK_FRAME_LEN],
    /// [`zigbee_mac::PlatformServices::monotonic_micros`] when armed: only
    /// GPDFs that ended later can have triggered it.
    armed_at_us: u32,
}

/// Bound on transmissions per [`ZigbeeDevice::gp_poll`] call, so one call
/// cannot monopolise the event loop.
const MAX_TX_PER_POLL: usize = 4;

/// ZCL frame control bits.
const ZCL_CLUSTER_SPECIFIC: u8 = 0x01;
const ZCL_MANUFACTURER_SPECIFIC: u8 = 0x04;
const ZCL_SERVER_TO_CLIENT: u8 = 0x08;
const ZCL_DISABLE_DEFAULT_RESPONSE: u8 = 0x10;
const ZCL_READ_ATTRIBUTES: u8 = 0x00;
const ZCL_READ_ATTRIBUTES_RESPONSE: u8 = 0x01;
const ZCL_WRITE_ATTRIBUTES: u8 = 0x02;
const ZCL_WRITE_ATTRIBUTES_RESPONSE: u8 = 0x04;
const ZCL_DEFAULT_RESPONSE: u8 = 0x0B;
const ZCL_UNSUPPORTED_ATTRIBUTE: u8 = 0x86;
const ZCL_INSUFFICIENT_SPACE: u8 = 0x89;

/// ZDO `Device_annce` cluster.
const DEVICE_ANNCE: u16 = 0x0013;

/// Extends the wrapping microsecond MAC clock to a millisecond clock that
/// wraps only after 49.7 days, as the proxy's deadline arithmetic needs.
/// Must be sampled at least once per 71-minute microsecond wrap.
#[derive(Debug, Clone, Copy)]
struct GpClock {
    last_us: u32,
    remainder_us: u32,
    now_ms: u32,
}

impl GpClock {
    const fn new() -> Self {
        Self {
            last_us: 0,
            remainder_us: 0,
            now_ms: 0,
        }
    }

    fn advance(&mut self, now_us: u32) -> u32 {
        let elapsed = u64::from(now_us.wrapping_sub(self.last_us)) + u64::from(self.remainder_us);
        self.last_us = now_us;
        self.now_ms = self.now_ms.wrapping_add((elapsed / 1_000) as u32);
        self.remainder_us = (elapsed % 1_000) as u32;
        self.now_ms
    }
}

/// One frame waiting for [`ZigbeeDevice::gp_poll`].
struct OutFrame {
    tx: GpTx,
    zcl_frame_control: u8,
    tsn: Option<u8>,
}

/// Green Power runtime state of a router (`green-power` feature).
pub struct GreenPowerState {
    proxy: GpProxy,
    clock: GpClock,
    /// A reply (Proxy Table / Read Attributes Response) for the next [`ZigbeeDevice::gp_poll`].
    reply: Option<OutFrame>,
    tsn: u8,
    #[cfg(feature = "green-power-pairing-search")]
    search: pairing_search::SearchState,
    /// Channel tuned to for a Channel Configuration delivery (`None`: operational).
    #[cfg(feature = "green-power-full-proxy")]
    off_channel: Option<u8>,
    #[cfg(feature = "green-power-bidirectional")]
    radio_reply: GpRadioReply,
    #[cfg(feature = "green-power-sink")]
    sink: GpSink,
    /// The GP frame being handled was APS-unicast to this node.
    #[cfg(feature = "green-power-sink")]
    rx_unicast: bool,
}

impl GreenPowerState {
    pub(crate) fn new() -> Self {
        Self {
            proxy: Proxy::new(ProxyConfig::default()),
            clock: GpClock::new(),
            reply: None,
            tsn: 0,
            #[cfg(feature = "green-power-pairing-search")]
            search: pairing_search::SearchState::new(),
            #[cfg(feature = "green-power-full-proxy")]
            off_channel: None,
            #[cfg(feature = "green-power-bidirectional")]
            radio_reply: GpRadioReply {
                unsupported: false,
                len: 0,
                src_id: 0,
                frame: [0; GP_DOWNLINK_FRAME_LEN],
                armed_at_us: 0,
            },
            #[cfg(feature = "green-power-sink")]
            sink: GpSink::new(zigbee_gp::sink::SinkConfig::default()),
            #[cfg(feature = "green-power-sink")]
            rx_unicast: false,
        }
    }

    /// Initialize `*dst` in place.
    ///
    /// Out of line on purpose: the device builder writes `ZigbeeDevice` in place, and
    /// constructing this value inside the builder kept a temporary in the builder's frame,
    /// on the deepest boot stack chain (`compose -> assemble_into -> NwkLayer::write_into`).
    ///
    /// # Safety
    ///
    /// `dst` must be valid for writes and properly aligned.
    #[inline(never)]
    pub(crate) unsafe fn write_into(dst: *mut Self) {
        // SAFETY: the caller guarantees `dst` is writable and aligned.
        unsafe { dst.write(Self::new()) }
    }

    /// Drop all network-bound GP state (leave / factory reset). The emptied
    /// table is reported dirty so the product persists the reset.
    pub(crate) fn clear(&mut self) {
        self.proxy.clear();
        self.reply = None;
        // Leave and rejoin retune the radio themselves.
        #[cfg(feature = "green-power-full-proxy")]
        {
            self.off_channel = None;
        }
        #[cfg(feature = "green-power-sink")]
        self.sink.clear();
    }

    /// Record whether the APS frame about to be handled was unicast to this
    /// node (GP Notification Response, Sink Table Response, Default
    /// Response only answer unicast requests).
    #[cfg(feature = "green-power-sink")]
    pub(crate) fn set_rx_unicast(&mut self, unicast: bool) {
        self.rx_unicast = unicast;
    }

    fn next_tsn(&mut self) -> u8 {
        self.tsn = self.tsn.wrapping_add(1);
        self.tsn
    }
}

/// Approximate the received RSSI from the 802.15.4 LQI for the GPP-GPD link byte (the
/// MAC indication carries no RSSI). It inverts the stack-wide RSSI-derived LQI policy
/// (`zigbee_mac::lqi::from_rssi_dbm`, linear over −100..−20 dBm), and the Link quality
/// follows the RSSI bands of [`GppGpdLink::from_rssi`].
fn link_from_lqi(lqi: u8) -> GppGpdLink {
    GppGpdLink::from_rssi(-100 + (i16::from(lqi) * 80 / 255))
}

/// Split a ZCL frame into (frame control, TSN, command, payload).
fn parse_zcl(frame: &[u8]) -> Option<(u8, u8, u8, &[u8])> {
    let fc = *frame.first()?;
    let header_len = if fc & ZCL_MANUFACTURER_SPECIFIC != 0 {
        5
    } else {
        3
    };
    if frame.len() < header_len {
        return None;
    }
    Some((
        fc,
        frame[header_len - 2],
        frame[header_len - 1],
        &frame[header_len..],
    ))
}

impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// The Green Power proxy (Proxy Table inspection and persistence).
    pub fn gp_proxy(&self) -> &GpProxy {
        &self.green_power.proxy
    }

    /// Mutable access to the Green Power proxy, e.g. to restore the Proxy
    /// Table from [`crate::gp_store`] after boot.
    pub fn gp_proxy_mut(&mut self) -> &mut GpProxy {
        &mut self.green_power.proxy
    }

    /// Whether the Proxy Table (or, with `green-power-sink`, the Sink Table)
    /// changed since the last call (persist it).
    pub fn gp_take_dirty(&mut self) -> bool {
        #[cfg(feature = "green-power-sink")]
        if self.green_power.sink.take_dirty() {
            self.green_power.proxy.take_dirty();
            return true;
        }
        self.green_power.proxy.take_dirty()
    }

    /// Request the Proxy Table be persisted again (a failed write).
    pub(crate) fn gp_mark_dirty(&mut self) {
        self.green_power.proxy.mark_dirty();
    }

    /// Re-add the GP endpoint to every group the restored Proxy Table needs,
    /// in case the APS group table snapshot is older than the Proxy Table.
    pub fn gp_restore_groups(&mut self) {
        let aps = self.bdb.zdo_mut().aps_mut();
        for group in self.green_power.proxy.required_groups() {
            let _ = aps.apsme_add_group(&zigbee_aps::apsme::ApsmeAddGroupRequest {
                group_address: group,
                endpoint: GP_ENDPOINT,
            });
        }
        #[cfg(feature = "green-power-sink")]
        for group in self.green_power.sink.required_groups() {
            let _ = aps.apsme_add_group(&zigbee_aps::apsme::ApsmeAddGroupRequest {
                group_address: group,
                endpoint: GP_ENDPOINT,
            });
        }
    }

    /// Whether this node currently acts as a GP proxy: a joined router.
    fn gp_active(&self) -> bool {
        let nwk = self.bdb.zdo().nwk();
        nwk.is_joined() && nwk.device_type() != DeviceType::EndDevice
    }

    fn gp_now_ms(&mut self) -> u32 {
        let now_us = self.bdb.zdo().nwk().mac().monotonic_micros();
        self.green_power.clock.advance(now_us)
    }

    /// Tell the proxy whether it can act as TempMaster (the MAC transmits
    /// at an instant) and on which channel it operates.
    #[cfg(feature = "green-power-bidirectional")]
    fn gp_sync_bidirectional(&mut self) {
        let capable = self.bdb.zdo().nwk().mac().timed_tx_supported();
        let channel = self.bdb.zdo().nwk().nib().logical_channel;
        let proxy = &mut self.green_power.proxy;
        if proxy.bidirectional() != capable {
            proxy.set_bidirectional_capable(capable);
        }
        proxy.set_operational_channel(channel);
    }

    /// Combo TempMaster (`green-power-sink`): only the gpTxQueue GPDF a
    /// received GPDF triggers, built and scheduled for
    /// [`Self::gp_send_downlink`]. The caller transmits it *before* the proxy
    /// and the sink process the GPDF with [`Self::gp_receive_gpdf`]: the
    /// sink's commissioning work (key unwrap, logging) on the same frame
    /// pushed the reply past its instant on hardware (2026-11-17). The
    /// reply is built from the Proxy Table entry before `on_gpdf` advances
    /// the GPD's counter, as without the sink.
    #[cfg(all(feature = "green-power-bidirectional", feature = "green-power-sink"))]
    #[inline(never)]
    pub(crate) fn gp_downlink_first(
        &mut self,
        indication: &McpsDataIndication,
    ) -> Option<GpDownlink> {
        if !self.gp_active() {
            return None;
        }
        self.gp_sync_bidirectional();
        self.gp_take_downlink(indication)
    }

    /// Handle a received GPDF (NWK protocol version 3) as TempMaster: the
    /// proxy processes it as [`Self::gp_receive_gpdf`] does, and the gpTxQueue
    /// GPDF it triggers (GP 1.1 A.3.9.1) is returned, already built and
    /// scheduled [`ProxyConfig::tx_offset_us`] after the end of
    /// `indication`, for [`Self::gp_send_downlink`].
    #[cfg(all(
        feature = "green-power-bidirectional",
        not(feature = "green-power-sink")
    ))]
    #[inline(never)]
    pub(crate) fn gp_receive_gpdf_bidirectional(
        &mut self,
        indication: &McpsDataIndication,
    ) -> Option<GpDownlink> {
        if !self.gp_active() {
            return None;
        }
        self.gp_sync_bidirectional();
        let downlink = self.gp_take_downlink(indication);
        self.gp_receive_gpdf(indication);
        downlink
    }

    /// Build the queued GPDF a received GPDF triggers, before
    /// [`Proxy::on_gpdf`] advances the GPD's frame counter: a protected
    /// trigger is verified against, and the reply protected with, the
    /// Proxy Table entry.
    #[cfg(feature = "green-power-bidirectional")]
    fn gp_take_downlink(&mut self, indication: &McpsDataIndication) -> Option<GpDownlink> {
        if !self.green_power.proxy.bidirectional() {
            return None;
        }
        let rx_end_us = indication.payload.rx_end_us();
        self.gp_radio_reply_outcome();
        let now_ms = self.gp_now_ms();
        let mac_src_ieee = match indication.src_address {
            MacAddress::Extended(_, ieee) => Some(ieee),
            _ => None,
        };
        let frame = zigbee_gp::frame::parse(
            indication.payload.as_slice(),
            indication.payload.mac_sequence(),
            mac_src_ieee,
        )
        .ok()?;
        // A trigger of the reply armed in the radio (the criteria of
        // `zigbee_mac::gp_reply::gpdf_triggers_reply`) is the radio's to
        // answer, and already answered when the task is late: no second,
        // timed transmission from the task.
        let armed = &self.green_power.radio_reply;
        let h = &frame.header;
        if armed.len != 0
            && rx_end_us.wrapping_sub(armed.armed_at_us) as i32 > 0
            && h.frame_type == zigbee_gp::frame::FrameType::Data
            && h.gpd_id == Some(zigbee_gp::types::GpdId::SrcId(armed.src_id))
            && !h.security_level.is_protected()
            && h.rx_after_tx
            && !h.to_gpd
        {
            return None;
        }
        let mut downlink = GpDownlink {
            frame: [0; GP_DOWNLINK_FRAME_LEN],
            len: 0,
            at_us: rx_end_us.wrapping_add(self.green_power.proxy.config().tx_offset_us),
            #[cfg(feature = "green-power-sink")]
            built_us: 0,
        };
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        // The MAC sequence number is set from macDSN when sending.
        let len =
            self.green_power
                .proxy
                .take_downlink(mac, &frame, now_ms, 0, &mut downlink.frame)?;
        downlink.len = len as u8;
        #[cfg(feature = "green-power-sink")]
        {
            downlink.built_us = self.bdb.zdo().nwk().mac().monotonic_micros();
        }
        Some(downlink)
    }

    /// Send a TempMaster GPDF at its instant, with the next macDSN, no
    /// CSMA/CA and no ACK request (GP 1.1 A.1.5.2.2).
    #[cfg(feature = "green-power-bidirectional")]
    #[inline(never)]
    pub(crate) async fn gp_send_downlink(&mut self, mut downlink: GpDownlink) {
        use zigbee_mac::pib::{PibAttribute, PibValue};
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        if let Ok(PibValue::U8(dsn)) = mac.mlme_get(PibAttribute::MacDsn).await {
            downlink.frame[2] = dsn;
            let _ = mac
                .mlme_set(PibAttribute::MacDsn, PibValue::U8(dsn.wrapping_add(1)))
                .await;
        }
        let frame = &downlink.frame[..usize::from(downlink.len)];
        let result = mac.transmit_raw_at(frame, downlink.at_us).await;
        #[cfg(not(feature = "green-power-sink"))]
        match result {
            Ok(()) => log::info!("[GP] TempMaster GPDF sent ({} B)", downlink.len),
            Err(error) => log::warn!("[GP] TempMaster GPDF not sent: {:?}", error),
        }
        // After the TX: the reply was built `built` µs after the end of the
        // triggering GPDF, was due at `due` and the MAC returned at `done`.
        #[cfg(feature = "green-power-sink")]
        {
            let tx_offset = self.green_power.proxy.config().tx_offset_us;
            let rx_end = downlink.at_us.wrapping_sub(tx_offset);
            let done = self
                .bdb
                .zdo()
                .nwk()
                .mac()
                .monotonic_micros()
                .wrapping_sub(rx_end);
            let built = downlink.built_us.wrapping_sub(rx_end) as i32;
            match result {
                Ok(()) => log::info!(
                    "[GP] TempMaster GPDF sent ({} B): built +{} us, due +{} us, done +{} us after the GPDF",
                    downlink.len,
                    built,
                    tx_offset,
                    done as i32
                ),
                Err(error) => log::warn!(
                    "[GP] TempMaster GPDF not sent: {:?}: built +{} us, due +{} us, done +{} us after the GPDF",
                    error,
                    built,
                    tx_offset,
                    done as i32
                ),
            }
        }
        // A delivered Channel Configuration ends a temporary channel.
        #[cfg(feature = "green-power-full-proxy")]
        self.gp_sync_radio_channel().await;
    }

    /// Tune the radio to the channel the proxy needs: a GPD's transmit
    /// channel while a Channel Configuration waits for delivery (GP 1.1
    /// A.3.9.1 step 8), otherwise back to the operational channel.
    #[cfg(feature = "green-power-full-proxy")]
    async fn gp_sync_radio_channel(&mut self) {
        let now_ms = self.gp_now_ms();
        let wanted = self.green_power.proxy.transmit_channel(now_ms);
        if wanted == self.green_power.off_channel {
            return;
        }
        let channel = wanted.unwrap_or(self.bdb.zdo().nwk().nib().logical_channel);
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        match mac
            .mlme_set(
                zigbee_mac::PibAttribute::PhyCurrentChannel,
                zigbee_mac::PibValue::U8(channel),
            )
            .await
        {
            Ok(()) => {
                log::info!("[GP] radio on channel {}", channel);
                self.green_power.off_channel = wanted;
            }
            Err(error) => {
                log::warn!("[GP] channel {} switch failed: {:?}", channel, error);
                if wanted.is_some() {
                    // Give up the delivery rather than retrying every pass.
                    self.green_power.proxy.abandon_transmit_channel();
                }
            }
        }
    }

    /// Handle a received GPDF (NWK protocol version 3). Never reaches the
    /// Zigbee NWK layer.
    #[inline(never)]
    pub(crate) fn gp_receive_gpdf(&mut self, indication: &McpsDataIndication) {
        if !self.gp_active() {
            return;
        }
        let now_ms = self.gp_now_ms();
        let mac_src_ieee = match indication.src_address {
            MacAddress::Extended(_, ieee) => Some(ieee),
            _ => None,
        };
        let frame = match zigbee_gp::frame::parse(
            indication.payload.as_slice(),
            indication.payload.mac_sequence(),
            mac_src_ieee,
        ) {
            Ok(frame) => frame,
            Err(error) => {
                log::debug!("[GP] GPDF rejected: {:?}", error);
                return;
            }
        };
        let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
        self.green_power.proxy.set_own_nwk(own_nwk);
        let link = link_from_lqi(indication.lqi);
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        let outcome = pairing_search::on_gpdf(&mut self.green_power, mac, &frame, link, now_ms);
        match outcome {
            GpdfOutcome::Dropped(reason) => {
                log::debug!("[GP] GPDF {:?} dropped: {:?}", frame.header.gpd_id, reason)
            }
            _ => log::info!("[GP] GPDF {:?}: {:?}", frame.header.gpd_id, outcome),
        }
        // Combo: the sink processes the GPDF itself (never tunnelled to us).
        #[cfg(feature = "green-power-sink")]
        if self.gp_sink_enabled() {
            self.gp_sink_sync();
            let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
            let outcome = self.green_power.sink.on_gpdf(mac, &frame, now_ms);
            gp_log_sink_outcome(frame.header.gpd_id, outcome);
            self.gp_sink_report_gpdf(&frame, outcome, indication.lqi);
            self.gp_sink_apply_groups();
        }
    }

    /// Route a Green Power cluster frame addressed to the GP endpoint.
    /// Returns `true` when the frame was for the GP endpoint (handled or
    /// deliberately ignored) and must not reach the application dispatcher.
    #[inline(never)]
    pub(crate) fn gp_handle_aps(
        &mut self,
        indication: &ApsdeDataIndication<'_>,
        src_addr: u16,
        nwk_authenticated: bool,
    ) -> bool {
        if indication.profile_id != GP_PROFILE_ID {
            return false;
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
        if !addressed {
            return false;
        }
        // Nothing but the GP cluster lives on the GP endpoint; every frame
        // addressed to it stops here.
        if indication.cluster_id != GP_CLUSTER_ID
            || !self.gp_active()
            || (self.bdb.zdo().nwk().nib().security_enabled && !nwk_authenticated)
        {
            return true;
        }
        let Some((fc, tsn, command, payload)) = parse_zcl(indication.payload) else {
            return true;
        };
        if fc & ZCL_MANUFACTURER_SPECIFIC != 0 {
            return true;
        }
        let now_ms = self.gp_now_ms();
        let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
        self.green_power.proxy.set_own_nwk(own_nwk);
        #[cfg(feature = "green-power-bidirectional")]
        self.gp_sync_bidirectional();
        let to_client = fc & ZCL_SERVER_TO_CLIENT != 0;
        if fc & 0x03 == ZCL_CLUSTER_SPECIFIC {
            if to_client {
                let mut reply = None;
                let result = self
                    .green_power
                    .proxy
                    .on_client_command(src_addr, command, payload, now_ms, &mut reply);
                // A window that elapsed before this command is reported
                // ahead of what the command did.
                self.gp_log_commissioning_timeout();
                match result {
                    Ok(effects) => self.gp_apply_effects(&effects),
                    Err(error) => log::warn!("[GP] client cmd 0x{:02X}: {:?}", command, error),
                }
                if let Some(tx) = reply {
                    self.green_power.reply = Some(OutFrame {
                        tx,
                        zcl_frame_control: ZCL_CLUSTER_SPECIFIC | ZCL_DISABLE_DEFAULT_RESPONSE,
                        tsn: Some(tsn),
                    });
                }
            } else {
                // Another proxy's GP Notification: lets us drop a duplicate
                // of the same GPDF still waiting out its tunnelling delay.
                if let Some(cancelled) = self
                    .green_power
                    .proxy
                    .on_server_command(src_addr, command, payload)
                {
                    log::debug!(
                        "[GP] forward {:?} fc {} cancelled: 0x{:04X} tunnelled it first",
                        cancelled.gpd_id,
                        cancelled.frame_counter,
                        src_addr
                    );
                }
                #[cfg(feature = "green-power-sink")]
                self.gp_sink_server_command(src_addr, fc, tsn, command, payload, now_ms);
            }
        } else if fc & 0x03 == 0 && to_client {
            match command {
                ZCL_READ_ATTRIBUTES => self.gp_read_attributes(src_addr, tsn, payload),
                ZCL_WRITE_ATTRIBUTES => self.gp_write_attributes(src_addr, tsn, payload),
                _ => {}
            }
        }
        // GP server attributes (sink, 0x0000-0x0007).
        #[cfg(feature = "green-power-sink")]
        if fc & 0x03 == 0 && !to_client {
            match command {
                ZCL_READ_ATTRIBUTES => self.gp_sink_read_attributes(src_addr, tsn, payload),
                ZCL_WRITE_ATTRIBUTES => self.gp_sink_write_attributes(src_addr, tsn, payload),
                _ => {}
            }
        }
        true
    }

    fn gp_apply_effects(&mut self, effects: &CommandEffects) {
        let aps = self.bdb.zdo_mut().aps_mut();
        for change in &effects.groups {
            match *change {
                GroupChange::Join(group) => {
                    let _ = aps.apsme_add_group(&zigbee_aps::apsme::ApsmeAddGroupRequest {
                        group_address: group,
                        endpoint: GP_ENDPOINT,
                    });
                }
                GroupChange::Leave(group) => {
                    // The combo's sink may still need the group.
                    #[cfg(feature = "green-power-sink")]
                    if self.green_power.sink.required_groups().any(|g| g == group) {
                        continue;
                    }
                    let _ = aps.apsme_remove_group(&zigbee_aps::apsme::ApsmeRemoveGroupRequest {
                        group_address: group,
                        endpoint: GP_ENDPOINT,
                    });
                }
            }
        }
        match effects.commissioning {
            Some(CommissioningEvent::Entered {
                originator,
                window_s,
            }) => log::info!(
                "[GP] proxy commissioning mode entered by 0x{:04X}, window {:?} s",
                originator,
                window_s
            ),
            Some(CommissioningEvent::Replaced { window_s }) => log::info!(
                "[GP] proxy commissioning window replaced, window {:?} s",
                window_s
            ),
            Some(CommissioningEvent::Left(reason)) => {
                log::info!("[GP] proxy commissioning mode left: {:?}", reason)
            }
            None => {}
        }
    }

    fn gp_log_commissioning_timeout(&mut self) {
        if self.green_power.proxy.take_commissioning_timeout() {
            log::info!(
                "[GP] proxy commissioning mode left: {:?}",
                CommissioningExit::Timeout
            );
        }
    }

    /// Answer a Read Attributes request for the GP client attributes.
    fn gp_read_attributes(&mut self, src_addr: u16, tsn: u8, request: &[u8]) {
        let mut payload = heapless::Vec::<u8, { zigbee_gp::proxy::MAX_GP_COMMAND_LEN }>::new();
        let mut value = [0u8; MAX_TX_PAYLOAD];
        for id in request.chunks_exact(2) {
            let id_bytes = [id[0], id[1]];
            let record_start = payload.len();
            let ok = match self
                .green_power
                .proxy
                .read_attribute(u16::from_le_bytes(id_bytes), &mut value)
            {
                Ok(Some((data_type, len))) => {
                    payload.extend_from_slice(&id_bytes).is_ok()
                        && payload.push(0x00).is_ok()
                        && payload.push(data_type).is_ok()
                        && payload.extend_from_slice(&value[..len]).is_ok()
                }
                Ok(None) => {
                    payload.extend_from_slice(&id_bytes).is_ok()
                        && payload.push(ZCL_UNSUPPORTED_ATTRIBUTE).is_ok()
                }
                Err(_) => {
                    payload.extend_from_slice(&id_bytes).is_ok()
                        && payload.push(ZCL_INSUFFICIENT_SPACE).is_ok()
                }
            };
            if !ok || payload.len() > MAX_TX_PAYLOAD {
                // ZCL 2.5.2.3: stop at the last record that fits.
                payload.truncate(record_start);
                break;
            }
        }
        self.green_power.reply = Some(OutFrame {
            tx: GpTx {
                dest: DestAddr::Unicast {
                    nwk: src_addr,
                    ieee: None,
                },
                alias: None,
                radius: 0,
                frame: TxFrame::GpCommand {
                    command_id: ZCL_READ_ATTRIBUTES_RESPONSE,
                    payload,
                },
            },
            zcl_frame_control: ZCL_SERVER_TO_CLIENT | ZCL_DISABLE_DEFAULT_RESPONSE,
            tsn: Some(tsn),
        });
    }

    /// Apply Write Attributes to the Full Proxy retry controls and answer
    /// unsupported/read-only attributes with their ZCL status.
    fn gp_write_attributes(&mut self, src_addr: u16, tsn: u8, request: &[u8]) {
        let (command_id, payload) =
            gp_write_attributes_response(&mut self.green_power.proxy, request);
        self.green_power.reply = Some(OutFrame {
            tx: GpTx {
                dest: DestAddr::Unicast {
                    nwk: src_addr,
                    ieee: None,
                },
                alias: None,
                radius: 0,
                frame: TxFrame::GpCommand {
                    command_id,
                    payload,
                },
            },
            zcl_frame_control: ZCL_SERVER_TO_CLIENT | ZCL_DISABLE_DEFAULT_RESPONSE,
            tsn: Some(tsn),
        });
    }

    /// Feed a NWK-authenticated `Device_annce` to the proxy (sink address
    /// update, alias conflict detection, alias Device_annce suppression).
    #[inline(never)]
    pub(crate) fn gp_device_annce(&mut self, payload: &[u8]) {
        if payload.len() != 12 || !self.gp_active() {
            return;
        }
        let nwk = u16::from_le_bytes([payload[1], payload[2]]);
        let mut ieee = [0u8; 8];
        ieee.copy_from_slice(&payload[3..11]);
        if ieee == [0xFF; 8] {
            self.green_power.proxy.on_alias_device_annce(nwk);
            return;
        }
        // The Dmin..Dmax alias-defence delay (A.3.5.2.1) only has to
        // differ between proxies, not be secret: draw it from the NWK
        // jitter mixer like the broadcast jitter and Parent Announce timer,
        // so no platform entropy harvester or DRBG (software AES on
        // TLSR8258) is linked into router images for it.
        let now_us = self.bdb.zdo().nwk().mac().monotonic_micros();
        let ieee_mix = ieee.iter().fold(u32::from(nwk), |acc, &b| {
            acc.rotate_left(5) ^ u32::from(b).wrapping_mul(0x9E37_79B9)
        });
        let random = zigbee_nwk::routing::routing_random_sample(now_us ^ ieee_mix);
        let now_ms = self.gp_now_ms();
        self.green_power
            .proxy
            .on_device_annce(nwk, ieee, now_ms, random);
    }

    /// Microseconds until [`gp_poll`](Self::gp_poll) has work, `None` when
    /// idle. The event loop bounds its receive timeout with this.
    pub fn gp_next_wait_us(&mut self) -> Option<u32> {
        if self.green_power.reply.is_some() {
            return Some(0);
        }
        #[cfg(feature = "green-power-full-proxy")]
        {
            let now_ms = self.gp_now_ms();
            if self.green_power.proxy.transmit_channel(now_ms) != self.green_power.off_channel {
                return Some(0);
            }
        }
        #[cfg(feature = "green-power-sink")]
        if self.green_power.sink.has_tx() {
            return Some(0);
        }
        #[cfg(not(feature = "green-power-sink"))]
        let deadline = self.green_power.proxy.next_deadline()?;
        #[cfg(feature = "green-power-sink")]
        let deadline = match (
            self.green_power.proxy.next_deadline(),
            self.green_power.sink.next_deadline(),
        ) {
            (Some(a), Some(b)) => {
                if b.wrapping_sub(a) < 0x8000_0000 {
                    a
                } else {
                    b
                }
            }
            (a, b) => a.or(b)?,
        };
        let now_ms = self.gp_now_ms();
        let remaining_ms = deadline.wrapping_sub(now_ms);
        Some(if remaining_ms >= 0x8000_0000 {
            0
        } else {
            remaining_ms.saturating_mul(1_000)
        })
    }

    /// Transmit due Green Power frames (tunnelled GP Notifications and
    /// Commissioning Notifications, replies, alias `Device_annce`). Call
    /// every event-loop pass while joined.
    pub async fn gp_poll(&mut self) {
        if !self.gp_active() {
            self.green_power.reply = None;
            #[cfg(feature = "green-power-bidirectional")]
            self.gp_disarm_radio_reply();
            return;
        }
        // Retune first: due network frames are sent only on the operational
        // channel, a Channel Configuration only on the GPD's.
        #[cfg(feature = "green-power-full-proxy")]
        self.gp_sync_radio_channel().await;
        let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
        self.green_power.proxy.set_own_nwk(own_nwk);
        // Before the sink queues new replies: a reply the radio sent leaves
        // the gpTxQueue, a newer one for the same GPD stays.
        #[cfg(feature = "green-power-bidirectional")]
        self.gp_radio_reply_outcome();
        for _ in 0..MAX_TX_PER_POLL {
            // The sink's frames share the reply slot (one transmit path).
            #[cfg(feature = "green-power-sink")]
            if self.green_power.reply.is_none() {
                self.green_power.reply = self.gp_sink_next_frame();
            }
            let frame = match self.green_power.reply.take() {
                Some(frame) => frame,
                None => {
                    let now_ms = self.gp_now_ms();
                    let tx = self.gp_proxy_poll(now_ms);
                    self.gp_log_commissioning_timeout();
                    let Some(tx) = tx else {
                        break;
                    };
                    OutFrame {
                        tx,
                        zcl_frame_control: ZCL_CLUSTER_SPECIFIC | ZCL_DISABLE_DEFAULT_RESPONSE,
                        tsn: None,
                    }
                }
            };
            self.gp_transmit(frame).await;
        }
        #[cfg(feature = "green-power-bidirectional")]
        self.gp_sync_radio_reply().await;
    }

    /// Keep the radio armed with the gpTxQueue GPDF it can send by itself
    /// ([`Proxy::radio_downlink`]), after reporting what became of the last
    /// armed one. The MAC sequence number is taken from macDSN when arming.
    #[cfg(feature = "green-power-bidirectional")]
    #[inline(never)]
    async fn gp_sync_radio_reply(&mut self) {
        use zigbee_mac::pib::{PibAttribute, PibValue};
        if self.green_power.radio_reply.unsupported {
            return;
        }
        self.gp_radio_reply_outcome();
        let now_ms = self.gp_now_ms();
        let mut frame = [0u8; GP_DOWNLINK_FRAME_LEN];
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        let candidate = self
            .green_power
            .proxy
            .radio_downlink(mac, now_ms, 0, &mut frame);
        let armed = &self.green_power.radio_reply;
        let current =
            (armed.len != 0).then(|| (armed.src_id, &armed.frame[..usize::from(armed.len)]));
        if candidate.map(|(src_id, len)| (src_id, &frame[..len])) == current {
            return;
        }
        let Some((src_id, len)) = candidate else {
            self.gp_disarm_radio_reply();
            return;
        };
        let mut on_air = frame;
        if let Ok(PibValue::U8(dsn)) = mac.mlme_get(PibAttribute::MacDsn).await {
            on_air[2] = dsn;
            let _ = mac
                .mlme_set(PibAttribute::MacDsn, PibValue::U8(dsn.wrapping_add(1)))
                .await;
        }
        let offset_us = self.green_power.proxy.config().tx_offset_us;
        let armed_at_us = mac.monotonic_micros();
        let reply = &mut self.green_power.radio_reply;
        match mac.arm_gp_reply(src_id, &on_air[..len], offset_us) {
            Ok(()) => {
                reply.len = len as u8;
                reply.src_id = src_id;
                reply.frame = frame;
                reply.armed_at_us = armed_at_us;
            }
            Err(zigbee_mac::MacError::Unsupported) => {
                reply.unsupported = true;
                reply.len = 0;
            }
            // E.g. a reply counting down: retried on the next poll.
            Err(_) => reply.len = 0,
        }
    }

    #[cfg(feature = "green-power-bidirectional")]
    fn gp_disarm_radio_reply(&mut self) {
        if self.green_power.radio_reply.len != 0 {
            self.green_power.radio_reply.len = 0;
            self.bdb
                .zdo_mut()
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .disarm_gp_reply();
        }
    }

    /// Report the radio's last armed reply, once; a sent GPDF leaves the
    /// gpTxQueue. Any outcome disarms the radio.
    #[cfg(feature = "green-power-bidirectional")]
    fn gp_radio_reply_outcome(&mut self) {
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        let Some(outcome) = mac.take_gp_reply_outcome() else {
            return;
        };
        let reply = &mut self.green_power.radio_reply;
        let len = reply.len;
        reply.len = 0;
        match outcome.result {
            Ok(()) => {
                self.green_power.proxy.radio_downlink_sent(outcome.src_id);
                log::info!(
                    "[GP] TempMaster GPDF sent by the radio ({} B): on air +{} us after the GPDF",
                    len,
                    outcome.after_us
                );
            }
            Err(error) => log::warn!(
                "[GP] TempMaster GPDF not sent by the radio: {:?} at +{} us after the GPDF",
                error,
                outcome.after_us
            ),
        }
    }

    async fn gp_transmit(&mut self, frame: OutFrame) {
        let (dst_addr_mode, dst_address) = match frame.tx.dest {
            DestAddr::Group(group) => (ApsAddressMode::Group, ApsAddress::Group(group)),
            DestAddr::Broadcast(addr) | DestAddr::Unicast { nwk: addr, .. } => {
                (ApsAddressMode::Short, ApsAddress::Short(ShortAddress(addr)))
            }
        };
        let mut buffer = [0u8; 3 + MAX_TX_PAYLOAD];
        let (len, endpoint, profile_id, cluster_id) = match &frame.tx.frame {
            TxFrame::GpCommand {
                command_id,
                payload,
            } => {
                if payload.len() > MAX_TX_PAYLOAD {
                    log::warn!(
                        "[GP] {}-byte GP command does not fit a frame",
                        payload.len()
                    );
                    return;
                }
                let tsn = match frame.tsn {
                    Some(tsn) => tsn,
                    None => self.green_power.next_tsn(),
                };
                buffer[0] = frame.zcl_frame_control;
                buffer[1] = tsn;
                buffer[2] = *command_id;
                buffer[3..3 + payload.len()].copy_from_slice(payload);
                (3 + payload.len(), GP_ENDPOINT, GP_PROFILE_ID, GP_CLUSTER_ID)
            }
            TxFrame::DeviceAnnce { alias } => {
                // A.3.6.3.4.2: ZDO TSN 0, alias, IEEE all-ones, capability
                // all zero (security capability inherited: not set).
                buffer[0] = 0;
                buffer[1..3].copy_from_slice(&alias.to_le_bytes());
                buffer[3..11].copy_from_slice(&[0xFF; 8]);
                buffer[11] = 0;
                (12, 0, 0x0000, DEVICE_ANNCE)
            }
        };
        let request = ApsdeDataRequest {
            dst_addr_mode,
            dst_address,
            dst_endpoint: endpoint,
            profile_id,
            cluster_id,
            src_endpoint: endpoint,
            payload: &buffer[..len],
            tx_options: ApsTxOptions {
                use_nwk_key: true,
                ..ApsTxOptions::default()
            },
            radius: frame.tx.radius,
            alias_src_addr: frame.tx.alias.map(|(addr, _)| ShortAddress(addr)),
            alias_seq: frame.tx.alias.map(|(_, seq)| seq),
        };
        if let Err(status) = self
            .bdb
            .zdo_mut()
            .aps_mut()
            .apsde_data_request(&request)
            .await
        {
            log::warn!("[GP] TX to {:?} failed: {:?}", frame.tx.dest, status);
        }
    }
}

#[cfg(feature = "green-power-sink")]
fn gp_log_sink_outcome(
    gpd: Option<zigbee_gp::types::GpdId>,
    outcome: zigbee_gp::sink::SinkOutcome,
) {
    match outcome {
        zigbee_gp::sink::SinkOutcome::Dropped(reason) => {
            log::debug!("[GP] sink {:?} dropped: {:?}", gpd, reason)
        }
        _ => log::info!("[GP] sink {:?}: {:?}", gpd, outcome),
    }
}

#[cfg(feature = "green-power-sink")]
fn gp_log_sink_command(src: u16, command: u8, outcome: zigbee_gp::sink::SinkOutcome) {
    match outcome {
        zigbee_gp::sink::SinkOutcome::Dropped(reason) => {
            log::debug!(
                "[GP] sink cmd 0x{:02X} from 0x{:04X} dropped: {:?}",
                command,
                src,
                reason
            )
        }
        _ => log::info!(
            "[GP] sink cmd 0x{:02X} from 0x{:04X}: {:?}",
            command,
            src,
            outcome
        ),
    }
}

/// ZCL Read Attributes Response records for `request`, served by `read`;
/// stops at the last record that fits one frame (ZCL 2.5.2.3).
#[cfg(feature = "green-power-sink")]
fn gp_read_attributes_payload(
    request: &[u8],
    mut read: impl FnMut(u16, &mut [u8]) -> zigbee_gp::Result<Option<(u8, usize)>>,
) -> heapless::Vec<u8, { zigbee_gp::proxy::MAX_GP_COMMAND_LEN }> {
    let mut payload = heapless::Vec::new();
    let mut value = [0u8; MAX_TX_PAYLOAD];
    for id in request.chunks_exact(2) {
        let id_bytes = [id[0], id[1]];
        let record_start = payload.len();
        let ok = match read(u16::from_le_bytes(id_bytes), &mut value) {
            Ok(Some((data_type, len))) => {
                payload.extend_from_slice(&id_bytes).is_ok()
                    && payload.push(0x00).is_ok()
                    && payload.push(data_type).is_ok()
                    && payload.extend_from_slice(&value[..len]).is_ok()
            }
            Ok(None) => {
                payload.extend_from_slice(&id_bytes).is_ok()
                    && payload.push(ZCL_UNSUPPORTED_ATTRIBUTE).is_ok()
            }
            Err(_) => {
                payload.extend_from_slice(&id_bytes).is_ok()
                    && payload.push(ZCL_INSUFFICIENT_SPACE).is_ok()
            }
        };
        if !ok || payload.len() > MAX_TX_PAYLOAD {
            payload.truncate(record_start);
            break;
        }
    }
    payload
}

/// Write Attributes to the GP server attributes of the sink: the response
/// command and payload.
#[cfg(feature = "green-power-sink")]
fn gp_sink_write_attributes_response(
    sink: &mut GpSink,
    request: &[u8],
) -> (
    u8,
    heapless::Vec<u8, { zigbee_gp::proxy::MAX_GP_COMMAND_LEN }>,
) {
    use zigbee_zcl::ZclStatus;
    use zigbee_zcl::data_types::ZclValue;
    use zigbee_zcl::foundation::write_attributes::{
        WriteAttributeStatusRecord, WriteAttributesRequest, WriteAttributesResponse,
    };
    let mut payload = heapless::Vec::new();
    let Ok(parsed) = WriteAttributesRequest::parse_checked(request) else {
        let _ =
            payload.extend_from_slice(&[ZCL_WRITE_ATTRIBUTES, ZclStatus::MalformedCommand as u8]);
        return (ZCL_DEFAULT_RESPONSE, payload);
    };
    let mut applied = WriteAttributesResponse {
        records: heapless::Vec::new(),
    };
    for record in &parsed.request.records {
        let mut raw = [0u8; 2];
        let value: &[u8] = match record.value {
            ZclValue::Bitmap8(v) | ZclValue::U8(v) => {
                raw[0] = v;
                &raw[..1]
            }
            ZclValue::U16(v) => {
                raw = v.to_le_bytes();
                &raw[..2]
            }
            _ => &[],
        };
        let status = sink.write_attribute(record.id.0, record.data_type as u8, value);
        let _ = applied.records.push(WriteAttributeStatusRecord {
            status: ZclStatus::from_u8(status),
            id: record.id,
        });
    }
    let response = parsed.merge_in_request_order(&applied);
    let mut encoded = [0u8; zigbee_gp::proxy::MAX_GP_COMMAND_LEN];
    let len = response.serialize(&mut encoded);
    let _ = payload.extend_from_slice(&encoded[..len]);
    (ZCL_WRITE_ATTRIBUTES_RESPONSE, payload)
}

#[cfg(feature = "green-power-sink")]
impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// The combo's Green Power sink (Sink Table inspection).
    pub fn gp_sink(&self) -> &GpSink {
        &self.green_power.sink
    }

    /// Mutable access to the sink, e.g. to apply the product's
    /// [`zigbee_gp::sink::SinkConfig`] at boot.
    pub fn gp_sink_mut(&mut self) -> &mut GpSink {
        &mut self.green_power.sink
    }

    /// Choose the local application endpoint that receives translated GPD
    /// commands (`None`: GPD commands are only reported as events).
    pub fn gp_sink_set_endpoint(&mut self, endpoint: Option<u8>) {
        self.green_power.sink.set_endpoint(endpoint);
    }

    /// Enter sink commissioning mode (e.g. on a button press), optionally
    /// putting the network's proxies into commissioning mode too. Fails with
    /// [`zigbee_gp::Error::Unsupported`] while this node is not a joined
    /// router or when gpsSecurityLevel asks to involve the Trust Center.
    pub fn gp_sink_enter_commissioning(
        &mut self,
        involve_proxies: bool,
    ) -> Result<(), zigbee_gp::Error> {
        if !self.gp_active() {
            return Err(zigbee_gp::Error::Unsupported);
        }
        self.gp_sink_sync();
        let now_ms = self.gp_now_ms();
        self.green_power
            .sink
            .enter_commissioning(now_ms, involve_proxies)?;
        log::info!(
            "[GP] sink commissioning mode entered (proxies {})",
            involve_proxies
        );
        Ok(())
    }

    /// Leave sink commissioning mode.
    pub fn gp_sink_exit_commissioning(&mut self) {
        self.green_power.sink.exit_commissioning();
    }

    /// Remove a GPD from the Sink Table (GP Pairing with RemoveGPD is
    /// broadcast by the next [`gp_poll`](Self::gp_poll)).
    pub fn gp_sink_remove(&mut self, gpd: &zigbee_gp::types::GpdId) -> bool {
        let removed = self.green_power.sink.remove(gpd);
        self.gp_sink_apply_groups();
        removed
    }

    /// Next sink event (commissioning, decommissioning, untranslated GPD
    /// commands such as attribute reports).
    pub fn gp_sink_take_event(&mut self) -> Option<zigbee_gp::sink::SinkEvent> {
        self.green_power.sink.take_event()
    }

    /// Give the sink this node's current addresses and channel.
    pub(crate) fn gp_sink_sync(&mut self) {
        let nib = self.bdb.zdo().nwk().nib();
        let (nwk, ieee, pan_id, channel) = (
            nib.network_address.0,
            nib.ieee_address,
            nib.pan_id.0,
            nib.logical_channel,
        );
        let bidirectional = self.green_power.proxy.bidirectional();
        let sink = &mut self.green_power.sink;
        crate::gp_sink_keys::sync(sink, &self.bdb, nwk, ieee, pan_id);
        sink.set_operational_channel(channel);
        sink.set_direct_bidirectional(bidirectional);
    }

    /// Apply the sink's GPEP group changes; a group the proxy still needs is
    /// kept.
    fn gp_sink_apply_groups(&mut self) {
        while let Some(change) = self.green_power.sink.take_group_change() {
            let aps = self.bdb.zdo_mut().aps_mut();
            match change {
                GroupChange::Join(group) => {
                    let _ = aps.apsme_add_group(&zigbee_aps::apsme::ApsmeAddGroupRequest {
                        group_address: group,
                        endpoint: GP_ENDPOINT,
                    });
                }
                GroupChange::Leave(group) => {
                    if self.green_power.proxy.required_groups().any(|g| g == group) {
                        continue;
                    }
                    let _ = aps.apsme_remove_group(&zigbee_aps::apsme::ApsmeRemoveGroupRequest {
                        group_address: group,
                        endpoint: GP_ENDPOINT,
                    });
                }
            }
        }
    }

    /// A GP server command (client → server) for the sink. Unicast requests
    /// get a ZCL Default Response on error or when it was not disabled
    /// (ZCL 2.5.12).
    fn gp_sink_server_command(
        &mut self,
        src_addr: u16,
        fc: u8,
        tsn: u8,
        command: u8,
        payload: &[u8],
        now_ms: u32,
    ) {
        const ZCL_MALFORMED_COMMAND: u8 = 0x80;
        const ZCL_UNSUP_CLUSTER_COMMAND: u8 = 0x81;
        self.gp_sink_sync();
        let unicast = self.green_power.rx_unicast;
        let command_frame = zigbee_gp::sink::ServerCommand {
            src_nwk: src_addr,
            unicast,
            tsn,
            command_id: command,
            payload,
        };
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        let status = match self
            .green_power
            .sink
            .on_server_command(mac, &command_frame, now_ms)
        {
            Ok((status, outcome)) => {
                if let Some(outcome) = outcome {
                    self.gp_sink_command_outcome(src_addr, command, payload, outcome);
                }
                status
            }
            Err(zigbee_gp::Error::Unsupported) => {
                log::debug!("[GP] sink cmd 0x{:02X} not supported", command);
                ZCL_UNSUP_CLUSTER_COMMAND
            }
            Err(error) => {
                log::debug!("[GP] sink cmd 0x{:02X}: {:?}", command, error);
                ZCL_MALFORMED_COMMAND
            }
        };
        self.gp_sink_apply_groups();
        if unicast && (status != 0 || fc & ZCL_DISABLE_DEFAULT_RESPONSE == 0) {
            let mut response = heapless::Vec::new();
            let _ = response.extend_from_slice(&[command, status]);
            self.green_power.reply = Some(OutFrame {
                tx: GpTx {
                    dest: DestAddr::Unicast {
                        nwk: src_addr,
                        ieee: None,
                    },
                    alias: None,
                    radius: 0,
                    frame: TxFrame::GpCommand {
                        command_id: ZCL_DEFAULT_RESPONSE,
                        payload: response,
                    },
                },
                zcl_frame_control: ZCL_SERVER_TO_CLIENT | ZCL_DISABLE_DEFAULT_RESPONSE,
                tsn: Some(tsn),
            });
        }
    }

    /// Answer a Read Attributes request for the GP server attributes.
    fn gp_sink_read_attributes(&mut self, src_addr: u16, tsn: u8, request: &[u8]) {
        let sink = &self.green_power.sink;
        let payload = gp_read_attributes_payload(request, |id, out| sink.read_attribute(id, out));
        self.gp_sink_reply(src_addr, tsn, ZCL_READ_ATTRIBUTES_RESPONSE, payload);
    }

    /// Apply Write Attributes to the writable GP server attributes.
    fn gp_sink_write_attributes(&mut self, src_addr: u16, tsn: u8, request: &[u8]) {
        let (command_id, payload) =
            gp_sink_write_attributes_response(&mut self.green_power.sink, request);
        self.gp_sink_reply(src_addr, tsn, command_id, payload);
    }

    fn gp_sink_reply(
        &mut self,
        src_addr: u16,
        tsn: u8,
        command_id: u8,
        payload: heapless::Vec<u8, { zigbee_gp::proxy::MAX_GP_COMMAND_LEN }>,
    ) {
        self.green_power.reply = Some(OutFrame {
            tx: GpTx {
                dest: DestAddr::Unicast {
                    nwk: src_addr,
                    ieee: None,
                },
                alias: None,
                radius: 0,
                frame: TxFrame::GpCommand {
                    command_id,
                    payload,
                },
            },
            zcl_frame_control: ZCL_SERVER_TO_CLIENT | ZCL_DISABLE_DEFAULT_RESPONSE,
            tsn: Some(tsn),
        });
    }

    /// Deliver the sink's translated GPD commands to the local application
    /// endpoint through the ZCL dispatcher, as locally originated (source:
    /// this node, GP endpoint) non-unicast commands, so no ZCL response goes
    /// on air. Returns the first resulting event.
    #[inline(never)]
    pub(crate) fn gp_sink_deliver(
        &mut self,
        clusters: &mut [crate::ClusterRef<'_>],
    ) -> Option<crate::event_loop::StackEvent> {
        let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
        let mut event = None;
        while let Some(command) = self.green_power.sink.take_command() {
            let translation = &command.translation;
            let mut zcl = [0u8; 3 + zigbee_gp::translation::MAX_TRANSLATED_PAYLOAD];
            zcl[0] = ZCL_CLUSTER_SPECIFIC | ZCL_DISABLE_DEFAULT_RESPONSE;
            zcl[1] = self.green_power.next_tsn();
            zcl[2] = translation.command_id;
            let len = 3 + translation.payload.len();
            zcl[3..len].copy_from_slice(&translation.payload);
            log::info!(
                "[GP] sink {:?}: cluster 0x{:04X} cmd 0x{:02X} to endpoint {}",
                command.gpd_id,
                translation.cluster_id,
                translation.command_id,
                command.endpoint
            );
            // SAFETY: the ZCL scratch is only borrowed by the synchronous
            // dispatcher, as in `dispatch_application_zcl`.
            let zcl_scratch = unsafe { &mut *self.scratch.zcl.get() };
            let ctx = crate::zcl_dispatch::LocalZclCtx::new(
                &self.endpoints,
                &mut self.basic_cluster,
                &mut self.identify_clusters,
                &mut self.reporting,
                &mut self.remote_reporting,
                &mut self.pending_responses,
                clusters,
                zcl_scratch,
            )
            .with_unicast(false);
            #[cfg(any(feature = "groups", test))]
            let ctx = ctx.with_group_table(self.bdb.zdo().aps().group_table());
            let outcome = ctx.dispatch(
                command.endpoint,
                GP_ENDPOINT,
                translation.cluster_id,
                own_nwk,
                &zcl[..len],
            );
            if event.is_none() {
                event = outcome.event;
            }
        }
        event
    }

    /// The sink's next frame. A GP Response that appoints this node
    /// TempMaster is also handed to its own proxy: the broadcast does not
    /// come back to the sender.
    fn gp_sink_next_frame(&mut self) -> Option<OutFrame> {
        self.gp_sink_sync();
        let now_ms = self.gp_now_ms();
        let out = self.green_power.sink.poll(now_ms)?;
        #[cfg(feature = "green-power-bidirectional")]
        if let TxFrame::GpCommand {
            command_id: zigbee_gp::cluster::client_cmd::RESPONSE,
            payload,
        } = &out.tx.frame
        {
            self.gp_sync_bidirectional();
            let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
            let proxy = &mut self.green_power.proxy;
            proxy.set_own_nwk(own_nwk);
            let mut reply = None;
            if let Err(error) = proxy.on_client_command(
                own_nwk,
                zigbee_gp::cluster::client_cmd::RESPONSE,
                payload,
                now_ms,
                &mut reply,
            ) {
                log::warn!("[GP] own proxy refused the sink's GP Response: {:?}", error);
            }
        }
        Some(OutFrame {
            tx: out.tx,
            zcl_frame_control: ZCL_CLUSTER_SPECIFIC
                | ZCL_SERVER_TO_CLIENT
                | ZCL_DISABLE_DEFAULT_RESPONSE,
            tsn: out.tsn,
        })
    }
}

/// Build the response to a GP client-attribute Write Attributes request.
/// Malformed records are rejected as a whole before the proxy configuration
/// is changed.
fn gp_write_attributes_response(
    proxy: &mut GpProxy,
    request: &[u8],
) -> (
    u8,
    heapless::Vec<u8, { zigbee_gp::proxy::MAX_GP_COMMAND_LEN }>,
) {
    use zigbee_zcl::ZclStatus;
    #[cfg(feature = "green-power-full-proxy")]
    use zigbee_zcl::data_types::ZclValue;
    use zigbee_zcl::foundation::write_attributes::{
        WriteAttributeStatusRecord, WriteAttributesRequest, WriteAttributesResponse,
    };

    let Ok(parsed) = WriteAttributesRequest::parse_checked(request) else {
        let mut payload = heapless::Vec::new();
        let _ =
            payload.extend_from_slice(&[ZCL_WRITE_ATTRIBUTES, ZclStatus::MalformedCommand as u8]);
        return (ZCL_DEFAULT_RESPONSE, payload);
    };

    #[cfg(feature = "green-power-full-proxy")]
    let mut config = *proxy.config();
    #[cfg(feature = "green-power-full-proxy")]
    let mut config_changed = false;
    let mut response = WriteAttributesResponse {
        records: heapless::Vec::new(),
    };
    let mut valid_records = parsed.request.records.iter();
    let mut invalid_records = parsed.invalid_data_types.iter().peekable();
    let record_count = parsed.request.records.len() + parsed.invalid_data_types.len();

    for ordinal in 0..record_count {
        let record_status = if invalid_records
            .peek()
            .is_some_and(|failure| usize::from(failure.ordinal) == ordinal)
        {
            invalid_records.next().map(|failure| failure.record)
        } else {
            valid_records.next().map(|record| {
                let id = record.id.0;
                let status = match id {
                    #[cfg(feature = "green-power-full-proxy")]
                    zigbee_gp::cluster::attr::GPP_NOTIFICATION_RETRY_NUMBER => {
                        match (record.data_type, &record.value) {
                            (zigbee_zcl::data_types::ZclDataType::U8, ZclValue::U8(value)) => {
                                config.notification_retry_number = *value;
                                config_changed = true;
                                ZclStatus::Success
                            }
                            _ => ZclStatus::InvalidDataType,
                        }
                    }
                    #[cfg(feature = "green-power-full-proxy")]
                    zigbee_gp::cluster::attr::GPP_NOTIFICATION_RETRY_TIMER => {
                        match (record.data_type, &record.value) {
                            (zigbee_zcl::data_types::ZclDataType::U8, ZclValue::U8(value)) => {
                                config.notification_retry_timer_ms = *value;
                                config_changed = true;
                                ZclStatus::Success
                            }
                            _ => ZclStatus::InvalidDataType,
                        }
                    }
                    _ => {
                        let mut value = [0u8; zigbee_gp::proxy::MAX_GP_COMMAND_LEN];
                        match proxy.read_attribute(id, &mut value) {
                            Ok(Some(_)) | Err(zigbee_gp::Error::BufferFull) => ZclStatus::ReadOnly,
                            Ok(None) => ZclStatus::UnsupportedAttribute,
                            Err(error) => {
                                log::warn!("[GP] attribute 0x{:04X} read failed: {:?}", id, error);
                                ZclStatus::Failure
                            }
                        }
                    }
                };
                WriteAttributeStatusRecord {
                    status,
                    id: record.id,
                }
            })
        };
        if let Some(status) = record_status {
            let _ = response.records.push(status);
        }
    }

    #[cfg(feature = "green-power-full-proxy")]
    if config_changed {
        proxy.set_config(config);
    }
    let mut payload = heapless::Vec::new();
    let mut encoded = [0u8; zigbee_gp::proxy::MAX_GP_COMMAND_LEN];
    let len = response.serialize(&mut encoded);
    let _ = payload.extend_from_slice(&encoded[..len]);
    (ZCL_WRITE_ATTRIBUTES_RESPONSE, payload)
}

// Network co-processor seams, declared after all other non-test code so
// that every line above keeps its number. Without `green-power-ncp` each one is the plain
// behaviour it replaced; `ncp` provides them for the ZRHP NCP (sink mode
// switch, sink frame reports, local proxy output for a host sink).

#[cfg(not(feature = "green-power-ncp"))]
impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// The proxy's next due frame.
    #[inline(always)]
    fn gp_proxy_poll(&mut self, now_ms: u32) -> Option<GpTx> {
        pairing_search::poll(&mut self.green_power, now_ms)
    }
}

#[cfg(all(feature = "green-power-sink", not(feature = "green-power-ncp")))]
impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// Whether the combo's sink processes GP traffic (always).
    #[inline(always)]
    fn gp_sink_enabled(&self) -> bool {
        true
    }

    /// What the sink made of a GPDF heard directly (logged only).
    #[inline(always)]
    fn gp_sink_report_gpdf(
        &mut self,
        _frame: &zigbee_gp::frame::Gpdf<'_>,
        _outcome: zigbee_gp::sink::SinkOutcome,
        _lqi: u8,
    ) {
    }

    /// What the sink made of a GP server command.
    #[inline(always)]
    fn gp_sink_command_outcome(
        &mut self,
        src: u16,
        command: u8,
        _payload: &[u8],
        outcome: zigbee_gp::sink::SinkOutcome,
    ) {
        gp_log_sink_command(src, command, outcome);
    }
}

#[cfg(feature = "green-power-ncp")]
pub mod ncp;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_extends_microsecond_wrap_without_loss() {
        let mut clock = GpClock::new();
        assert_eq!(clock.advance(1_500), 1);
        assert_eq!(clock.advance(2_000), 2);
        // Whole milliseconds up to just below the wrap: no remainder.
        let before = clock.advance(4_294_967_000);
        assert_eq!(before, 4_294_967);
        // 1 001 µs across the u32 wrap.
        assert_eq!(clock.advance(705), before + 1);
        assert_eq!(clock.remainder_us, 1);
    }

    #[test]
    fn zcl_header_split() {
        assert_eq!(
            parse_zcl(&[0x19, 0x07, 0x02, 0xAA]),
            Some((0x19, 0x07, 0x02, &[0xAA][..]))
        );
        assert_eq!(
            parse_zcl(&[0x1D, 0x34, 0x12, 0x07, 0x02]),
            Some((0x1D, 0x07, 0x02, &[][..]))
        );
        assert_eq!(parse_zcl(&[0x19, 0x07]), None);
    }

    #[cfg(feature = "green-power-full-proxy")]
    #[test]
    fn write_attributes_updates_full_proxy_retry_controls() {
        let mut proxy = GpProxy::new(ProxyConfig::default());
        let (command, response) = gp_write_attributes_response(
            &mut proxy,
            &[
                0x12, 0x00, 0x20, 0x03, // retry count
                0x13, 0x00, 0x20, 0x32, // retry timer, 50 ms
            ],
        );
        assert_eq!(command, ZCL_WRITE_ATTRIBUTES_RESPONSE);
        assert_eq!(response.as_slice(), &[0]);
        assert_eq!(proxy.config().notification_retry_number, 3);
        assert_eq!(proxy.config().notification_retry_timer_ms, 50);
    }

    #[cfg(feature = "green-power-full-proxy")]
    #[test]
    fn malformed_write_attributes_does_not_change_proxy_config() {
        let mut proxy = GpProxy::new(ProxyConfig::default());
        let (command, response) = gp_write_attributes_response(&mut proxy, &[0x12, 0x00, 0x20]);
        assert_eq!(command, ZCL_DEFAULT_RESPONSE);
        assert_eq!(
            response.as_slice(),
            &[
                ZCL_WRITE_ATTRIBUTES,
                zigbee_zcl::ZclStatus::MalformedCommand as u8
            ]
        );
        assert_eq!(
            proxy.config().notification_retry_number,
            zigbee_gp::proxy::DEFAULT_NOTIFICATION_RETRY_NUMBER
        );
    }

    #[cfg(feature = "green-power-full-proxy")]
    #[test]
    fn write_attributes_reports_type_read_only_and_unknown_errors() {
        let mut proxy = GpProxy::new(ProxyConfig::default());
        let (command, response) = gp_write_attributes_response(
            &mut proxy,
            &[
                0x12, 0x00, 0x21, 0x03, 0x00, // retry count with wrong type
                0x10, 0x00, 0x20, 0x03, // read-only max proxy table entries
                0x34, 0x12, 0x20, 0x03, // unknown attribute
            ],
        );
        assert_eq!(command, ZCL_WRITE_ATTRIBUTES_RESPONSE);
        assert_eq!(
            response.as_slice(),
            &[
                zigbee_zcl::ZclStatus::InvalidDataType as u8,
                0x12,
                0x00,
                zigbee_zcl::ZclStatus::ReadOnly as u8,
                0x10,
                0x00,
                ZCL_UNSUPPORTED_ATTRIBUTE,
                0x34,
                0x12,
            ]
        );
        assert_eq!(
            proxy.config().notification_retry_number,
            zigbee_gp::proxy::DEFAULT_NOTIFICATION_RETRY_NUMBER
        );
    }

    #[test]
    fn lqi_link_estimate_is_monotonic_and_in_range() {
        let low = link_from_lqi(0);
        let high = link_from_lqi(255);
        assert_eq!(low.rssi_dbm(), -100);
        assert_eq!(low.link_quality(), 0);
        assert_eq!(high.rssi_dbm(), -20);
        assert_eq!(high.link_quality(), 3);
        let mut last = low;
        for lqi in 1..=255 {
            let l = link_from_lqi(lqi);
            assert!(l.link_quality() >= last.link_quality());
            assert!(l.rssi_dbm() >= last.rssi_dbm());
            last = l;
        }
    }
}

mod pairing_search;
