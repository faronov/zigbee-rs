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

use zigbee_aps::ApsTxOptions;
use zigbee_aps::apsde::{ApsdeDataIndication, ApsdeDataRequest};
use zigbee_aps::{ApsAddress, ApsAddressMode};
use zigbee_gp::cluster::GppGpdLink;
use zigbee_gp::proxy::{
    CommandEffects, DestAddr, GpTx, GpdfOutcome, GroupChange, MAX_TX_PAYLOAD, Proxy, ProxyConfig,
    TxFrame,
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
    /// A reply (Proxy Table Response, Read Attributes Response) waiting for
    /// the next [`ZigbeeDevice::gp_poll`].
    reply: Option<OutFrame>,
    tsn: u8,
}

impl GreenPowerState {
    pub(crate) fn new() -> Self {
        Self {
            proxy: Proxy::new(ProxyConfig::default()),
            clock: GpClock::new(),
            reply: None,
            tsn: 0,
        }
    }

    /// Drop all network-bound GP state (leave / factory reset). The emptied
    /// table is reported dirty so the product persists the reset.
    pub(crate) fn clear(&mut self) {
        self.proxy.clear();
        self.reply = None;
    }

    fn next_tsn(&mut self) -> u8 {
        self.tsn = self.tsn.wrapping_add(1);
        self.tsn
    }
}

/// Approximate the received RSSI from the 802.15.4 LQI for the GPP-GPD link
/// byte (the MAC indication carries no RSSI). Linear over the usual
/// −100..−20 dBm LQI range; it only orders proxies for the tunnelling delay
/// and is reported to the sink as an estimate.
fn link_from_lqi(lqi: u8) -> GppGpdLink {
    let rssi = -100 + (i16::from(lqi) * 80 / 255);
    GppGpdLink::from_rssi_lqi(rssi as i8, lqi)
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

    /// Whether the Proxy Table changed since the last call (persist it).
    pub fn gp_take_dirty(&mut self) -> bool {
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
        let outcome = self.green_power.proxy.on_gpdf(mac, &frame, link, now_ms);
        match outcome {
            GpdfOutcome::Dropped(reason) => {
                log::debug!("[GP] GPDF {:?} dropped: {:?}", frame.header.gpd_id, reason)
            }
            _ => log::info!("[GP] GPDF {:?}: {:?}", frame.header.gpd_id, outcome),
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
        let to_client = fc & ZCL_SERVER_TO_CLIENT != 0;
        if fc & 0x03 == ZCL_CLUSTER_SPECIFIC {
            if to_client {
                let mut reply = None;
                match self
                    .green_power
                    .proxy
                    .on_client_command(src_addr, command, payload, now_ms, &mut reply)
                {
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
                self.green_power
                    .proxy
                    .on_server_command(src_addr, command, payload);
            }
        } else if fc & 0x03 == 0 && to_client && command == ZCL_READ_ATTRIBUTES {
            self.gp_read_attributes(src_addr, tsn, payload);
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
                    let _ = aps.apsme_remove_group(&zigbee_aps::apsme::ApsmeRemoveGroupRequest {
                        group_address: group,
                        endpoint: GP_ENDPOINT,
                    });
                }
            }
        }
        if let Some(enter) = effects.commissioning {
            log::info!(
                "[GP] proxy commissioning mode {}",
                if enter { "entered" } else { "left" }
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
            zcl_frame_control: ZCL_DISABLE_DEFAULT_RESPONSE,
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
        let now_ms = self.gp_now_ms();
        let mut random = [0u8; 4];
        let mac = self.bdb.zdo_mut().aps_mut().nwk_mut().mac_mut();
        if mac.fill_random(&mut random).is_err() {
            // Without entropy the conflict announcement still goes out, at
            // the deterministic end of the Dmin..Dmax window.
            random = [0xFF; 4];
        }
        self.green_power
            .proxy
            .on_device_annce(nwk, ieee, now_ms, u32::from_le_bytes(random));
    }

    /// Microseconds until [`gp_poll`](Self::gp_poll) has work, `None` when
    /// idle. The event loop bounds its receive timeout with this.
    pub fn gp_next_wait_us(&mut self) -> Option<u32> {
        if self.green_power.reply.is_some() {
            return Some(0);
        }
        let deadline = self.green_power.proxy.next_deadline()?;
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
            return;
        }
        let own_nwk = self.bdb.zdo().nwk().nib().network_address.0;
        self.green_power.proxy.set_own_nwk(own_nwk);
        for _ in 0..MAX_TX_PER_POLL {
            let frame = match self.green_power.reply.take() {
                Some(frame) => frame,
                None => {
                    let now_ms = self.gp_now_ms();
                    let Some(tx) = self.green_power.proxy.poll(now_ms) else {
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

    #[test]
    fn lqi_link_estimate_is_monotonic_and_in_range() {
        let low = link_from_lqi(0);
        let high = link_from_lqi(255);
        assert_eq!(low.rssi_dbm(), -100);
        assert_eq!(high.rssi_dbm(), -20);
        assert_eq!(high.link_quality(), 3);
    }
}
