//! ZDP request / response dispatcher.
//!
//! Routes incoming APS frames on endpoint 0 to the appropriate ZDP handler,
//! builds the response, and sends it back through the APS layer.

use zigbee_aps::apsde::ApsdeDataIndication;
use zigbee_aps::binding::{BindingDst, BindingDstMode, BindingEntry};
use zigbee_aps::{ApsAddress, ApsAddressMode};
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;

use crate::binding_mgmt::{BindReq, BindTarget};
use crate::discovery::*;
use crate::network_mgmt::*;
use crate::{ZDO_ENDPOINT, ZdoError, ZdoLayer, ZdpStatus};

/// Bit 15 of a ZDP cluster identifier: set on responses, clear on requests.
///
/// A ZDP response cluster is always `request | ZDP_RESPONSE_BIT` (R22 2.4.4).
const ZDP_RESPONSE_BIT: u16 = 0x8000;

/// Whether `addr` is one of the four NWK broadcast destinations.
///
/// Zigbee PRO R22 3.6.5 defines exactly `0xFFFF` (all devices), `0xFFFD`
/// (receiver on when idle), `0xFFFC` (routers and coordinator) and `0xFFFB`
/// (low-power routers). `0xFFF8..=0xFFFA` are reserved and `0xFFFE` is the
/// unassigned marker, so none of them name a real unicast destination either.
const fn is_broadcast_short(addr: ShortAddress) -> bool {
    matches!(addr.0, 0xFFFF | 0xFFFD | 0xFFFC | 0xFFFB)
}

/// Whether `addr` can name an individual device (`0x0000..=0xFFF7`).
const fn is_unicast_short(addr: ShortAddress) -> bool {
    addr.0 < 0xFFF8
}

/// Single formatting site for every ZDP frame the dispatcher does not answer
/// normally. Kept out of line so all exceptional paths share one log site
/// instead of embedding a format-argument table each.
#[inline(never)]
fn log_zdp_exception(cluster: u16, reason: &str) {
    log::debug!("ZDP: cluster 0x{cluster:04X}: {reason}");
}

#[inline(always)]
fn zdp_cluster_name(cluster: u16) -> &'static str {
    match cluster {
        crate::NWK_ADDR_REQ => "NWK_ADDR_REQ",
        crate::IEEE_ADDR_REQ => "IEEE_ADDR_REQ",
        crate::NODE_DESC_REQ => "NODE_DESC_REQ",
        crate::POWER_DESC_REQ => "POWER_DESC_REQ",
        crate::SIMPLE_DESC_REQ => "SIMPLE_DESC_REQ",
        crate::ACTIVE_EP_REQ => "ACTIVE_EP_REQ",
        crate::MATCH_DESC_REQ => "MATCH_DESC_REQ",
        crate::BIND_REQ => "BIND_REQ",
        crate::UNBIND_REQ => "UNBIND_REQ",
        crate::MGMT_LQI_REQ => "MGMT_LQI_REQ",
        crate::MGMT_RTG_REQ => "MGMT_RTG_REQ",
        crate::MGMT_BIND_REQ => "MGMT_BIND_REQ",
        crate::MGMT_LEAVE_REQ => "MGMT_LEAVE_REQ",
        crate::MGMT_PERMIT_JOINING_REQ => "MGMT_PERMIT_JOINING_REQ",
        crate::MGMT_NWK_UPDATE_REQ => "MGMT_NWK_UPDATE_REQ",
        _ => "UNKNOWN_ZDP",
    }
}

// ── Main dispatcher ─────────────────────────────────────────────

impl<M: MacDriver> ZdoLayer<M> {
    /// Process an incoming APS indication addressed to the ZDO endpoint.
    ///
    /// Returns `Ok(())` if the frame was handled (or silently ignored).
    pub async fn handle_indication(
        &mut self,
        ind: &ApsdeDataIndication<'_>,
    ) -> Result<(), ZdoError> {
        // Only handle ZDO endpoint
        if ind.dst_endpoint != ZDO_ENDPOINT {
            return Ok(());
        }
        if ind.payload.is_empty() {
            return Err(ZdoError::InvalidLength);
        }

        let tsn = ind.payload[0];
        let payload = &ind.payload[1..];
        let cluster = ind.cluster_id;

        // Extract source short address for the reply
        let src_short = match ind.src_address {
            ApsAddress::Short(a) => a,
            _ => ShortAddress(0x0000),
        };

        self.diagnostics.indications = self.diagnostics.indications.wrapping_add(1);
        self.diagnostics.last_cluster = cluster;
        match cluster {
            crate::NODE_DESC_REQ => {
                self.diagnostics.node_desc_requests =
                    self.diagnostics.node_desc_requests.wrapping_add(1);
            }
            crate::ACTIVE_EP_REQ => {
                self.diagnostics.active_ep_requests =
                    self.diagnostics.active_ep_requests.wrapping_add(1);
            }
            crate::SIMPLE_DESC_REQ => {
                self.diagnostics.simple_desc_requests =
                    self.diagnostics.simple_desc_requests.wrapping_add(1);
            }
            _ => {}
        }

        log::info!(
            "[ZDO RX] {} cluster=0x{:04X} tsn={} from=0x{:04X} ep={}",
            zdp_cluster_name(cluster),
            cluster,
            tsn,
            src_short.0,
            ind.src_endpoint
        );

        // --- Device_annce is fire-and-forget (no response) ---
        if cluster == crate::DEVICE_ANNCE {
            let _ = self.process_device_annce(payload);
            return Ok(());
        }

        // --- R22 Parent Announce (router/coordinator child reconciliation) ---
        if cluster == crate::PARENT_ANNCE {
            self.process_parent_annce(src_short, tsn, payload).await?;
            return Ok(());
        }
        if cluster == crate::PARENT_ANNCE_RSP {
            self.process_parent_annce_rsp(tsn, payload);
            return Ok(());
        }

        // --- Check if this is a response to a pending client request ---
        if self.deliver_response(cluster, tsn, payload) {
            log::info!("[ZDO] Consumed as client response: cluster=0x{cluster:04X} tsn={tsn}");
            return Ok(());
        }

        // --- Never answer a response cluster ---
        //
        // Bit 15 marks a ZDP response. Anything that reaches this point is an
        // unsolicited response (no pending request matched it): it must be
        // dropped, never turned into a request/response of our own.
        if cluster & ZDP_RESPONSE_BIT != 0 {
            log_zdp_exception(cluster, "unsolicited response — dropped");
            return Ok(());
        }

        // Whether this request was addressed to this node individually.
        // Broadcast and group requests must only be answered when this node
        // actually has something to say (R22 2.4.3): a "not supported" or
        // "no match" reply from every receiver is a broadcast storm.
        let unicast = self.indication_is_unicast(ind);

        // --- Build response in a stack buffer ---
        let mut rsp_buf = [0u8; 256];
        rsp_buf[0] = tsn; // echo TSN

        let (rsp_cluster, result) = match cluster {
            // ── Discovery ───────────────────────────────────────
            crate::NWK_ADDR_REQ => {
                let result = self.handle_nwk_addr_req(payload, &mut rsp_buf[1..]);
                if !unicast && result.is_ok() && rsp_buf[1] != ZdpStatus::Success as u8 {
                    log_zdp_exception(cluster, "broadcast address miss — silent");
                    return Ok(());
                }
                (crate::NWK_ADDR_RSP, result)
            }
            crate::IEEE_ADDR_REQ => {
                let result = self.handle_ieee_addr_req(payload, &mut rsp_buf[1..]);
                if !unicast && result.is_ok() && rsp_buf[1] != ZdpStatus::Success as u8 {
                    log_zdp_exception(cluster, "broadcast address miss — silent");
                    return Ok(());
                }
                (crate::IEEE_ADDR_RSP, result)
            }
            crate::NODE_DESC_REQ => (
                crate::NODE_DESC_RSP,
                self.handle_node_desc_req(payload, &mut rsp_buf[1..]),
            ),
            crate::POWER_DESC_REQ => (
                crate::POWER_DESC_RSP,
                self.handle_power_desc_req(payload, &mut rsp_buf[1..]),
            ),
            crate::SIMPLE_DESC_REQ => (
                crate::SIMPLE_DESC_RSP,
                self.handle_simple_desc_req(payload, &mut rsp_buf[1..]),
            ),
            crate::ACTIVE_EP_REQ => (
                crate::ACTIVE_EP_RSP,
                self.handle_active_ep_req(payload, &mut rsp_buf[1..]),
            ),
            crate::MATCH_DESC_REQ => {
                match self.handle_match_desc_req(payload, unicast, &mut rsp_buf[1..]) {
                    // A broadcast Match_Desc_req that this node cannot satisfy
                    // is answered with silence, not with NO_MATCH.
                    Ok(None) => {
                        log_zdp_exception(cluster, "broadcast without match — silent");
                        return Ok(());
                    }
                    Ok(Some(n)) => (crate::MATCH_DESC_RSP, Ok(n)),
                    Err(e) => (crate::MATCH_DESC_RSP, Err(e)),
                }
            }

            // ── Binding management ──────────────────────────────
            crate::BIND_REQ => (
                crate::BIND_RSP,
                self.handle_bind_req(payload, &mut rsp_buf[1..]),
            ),
            crate::UNBIND_REQ => (
                crate::UNBIND_RSP,
                self.handle_unbind_req(payload, &mut rsp_buf[1..]),
            ),

            // ── Network management ──────────────────────────────
            crate::MGMT_LQI_REQ => (
                crate::MGMT_LQI_RSP,
                self.handle_mgmt_lqi_req(payload, &mut rsp_buf[1..]),
            ),
            crate::MGMT_RTG_REQ => (
                crate::MGMT_RTG_RSP,
                self.handle_mgmt_rtg_req(payload, &mut rsp_buf[1..]),
            ),
            crate::MGMT_BIND_REQ => (
                crate::MGMT_BIND_RSP,
                self.handle_mgmt_bind_req(payload, &mut rsp_buf[1..]),
            ),
            crate::MGMT_LEAVE_REQ => (
                crate::MGMT_LEAVE_RSP,
                self.handle_mgmt_leave_req(src_short, payload, &mut rsp_buf[1..]),
            ),
            crate::MGMT_PERMIT_JOINING_REQ => {
                let result = self
                    .handle_mgmt_permit_joining_req(payload, &mut rsp_buf[1..])
                    .await;
                if !unicast {
                    return match result {
                        Ok(_) | Err(ZdoError::InvalidLength | ZdoError::InvalidData) => Ok(()),
                        Err(err) => Err(err),
                    };
                }
                (crate::MGMT_PERMIT_JOINING_RSP, result)
            }
            crate::MGMT_NWK_UPDATE_REQ => (
                crate::MGMT_NWK_UPDATE_RSP,
                self.handle_mgmt_nwk_update_req(payload, &mut rsp_buf[1..])
                    .await,
            ),

            // ── Everything else ─────────────────────────────────
            //
            // Any other request cluster is one this stack does not implement.
            // R22 2.4.5 requires the matching response cluster carrying
            // NOT_SUPPORTED for a unicast request; broadcast and group
            // requests are dropped.
            _ => {
                if !unicast {
                    log_zdp_exception(cluster, "unsupported broadcast — dropped");
                    return Ok(());
                }
                log_zdp_exception(cluster, "unsupported unicast — NOT_SUPPORTED");
                rsp_buf[1] = ZdpStatus::NotSupported as u8;
                (cluster | ZDP_RESPONSE_BIT, Ok(1))
            }
        };

        let rsp_len = match result {
            Ok(n) => 1 + n,
            // Malformed broadcasts stay silent. A malformed unicast remains
            // an explicit dispatcher error because most ZDP responses require
            // mandatory fields after the status byte; a status-only frame
            // would itself be malformed.
            Err(err @ (ZdoError::InvalidLength | ZdoError::InvalidData)) => {
                if !unicast {
                    log_zdp_exception(cluster, "malformed broadcast — dropped");
                    return Ok(());
                }
                return Err(err);
            }
            Err(err) => return Err(err),
        };

        // --- Send response ---
        log::info!(
            "[ZDO TX] rsp cluster=0x{:04X} to 0x{:04X} len={}",
            rsp_cluster,
            src_short.0,
            rsp_len
        );
        self.diagnostics.response_attempts = self.diagnostics.response_attempts.wrapping_add(1);
        self.diagnostics.last_response_cluster = rsp_cluster;
        let tx_result = self
            .send_zdp_unicast(src_short, rsp_cluster, &rsp_buf[..rsp_len])
            .await;
        if tx_result.is_ok() {
            self.diagnostics.response_successes =
                self.diagnostics.response_successes.wrapping_add(1);
        } else {
            self.diagnostics.response_failures = self.diagnostics.response_failures.wrapping_add(1);
        }
        tx_result
    }

    /// This node's short address, preferring the live NIB value.
    ///
    /// The ZDO copy is refreshed at join/restore; the NIB is authoritative and
    /// also tracks address changes, so it wins whenever it names a real device.
    fn local_short_address(&self) -> ShortAddress {
        let nib_addr = self.nwk().nib().network_address;
        if is_unicast_short(nib_addr) {
            nib_addr
        } else {
            self.local_nwk_addr()
        }
    }

    /// Whether `ind` was delivered to this node individually.
    ///
    /// Decided from the indication's own destination address mode and address
    /// — the APS layer reports the NWK destination for unicast and broadcast
    /// alike — checked against this node's short/extended address. Group
    /// deliveries and the four NWK broadcast addresses are never unicast.
    fn indication_is_unicast(&self, ind: &ApsdeDataIndication<'_>) -> bool {
        if matches!(ind.dst_addr_mode, ApsAddressMode::Group) {
            return false;
        }
        match ind.dst_address {
            ApsAddress::Group(_) => false,
            ApsAddress::Extended(ieee) => ieee == self.local_ieee_addr(),
            ApsAddress::Short(dst) => {
                if is_broadcast_short(dst) || !is_unicast_short(dst) {
                    return false;
                }
                let local = self.local_short_address();
                // Before a short address is assigned there is nothing to
                // compare against, and the lower layers only deliver frames
                // addressed to this node, so treat it as an individual frame.
                !is_unicast_short(local) || dst == local
            }
        }
    }
}

// ── Individual handlers ─────────────────────────────────────────
//
// Each method writes the response payload (after TSN) into `rsp` and
// returns the number of bytes written.

impl<M: MacDriver> ZdoLayer<M> {
    // ── Discovery ───────────────────────────────────────────────

    fn handle_nwk_addr_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = NwkAddrReq::parse(payload)?;
        let rsp_data = if req.ieee_addr == self.local_ieee_addr() {
            NwkAddrRsp {
                status: ZdpStatus::Success,
                ieee_addr: self.local_ieee_addr(),
                nwk_addr: self.local_nwk_addr(),
                num_assoc_dev: 0,
                start_index: 0,
                assoc_dev_list: heapless::Vec::new(),
            }
        } else {
            NwkAddrRsp {
                status: ZdpStatus::DeviceNotFound,
                ieee_addr: req.ieee_addr,
                nwk_addr: ShortAddress(0x0000),
                num_assoc_dev: 0,
                start_index: 0,
                assoc_dev_list: heapless::Vec::new(),
            }
        };
        rsp_data.serialize(rsp)
    }

    fn handle_ieee_addr_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = IeeeAddrReq::parse(payload)?;
        let rsp_data = if req.nwk_addr_of_interest == self.local_nwk_addr() {
            NwkAddrRsp {
                status: ZdpStatus::Success,
                ieee_addr: self.local_ieee_addr(),
                nwk_addr: self.local_nwk_addr(),
                num_assoc_dev: 0,
                start_index: 0,
                assoc_dev_list: heapless::Vec::new(),
            }
        } else {
            NwkAddrRsp {
                status: ZdpStatus::DeviceNotFound,
                ieee_addr: [0u8; 8],
                nwk_addr: req.nwk_addr_of_interest,
                num_assoc_dev: 0,
                start_index: 0,
                assoc_dev_list: heapless::Vec::new(),
            }
        };
        rsp_data.serialize(rsp)
    }

    fn handle_node_desc_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = NodeDescReq::parse(payload)?;
        let rsp_data = if req.nwk_addr_of_interest == self.local_nwk_addr() {
            NodeDescRsp {
                status: ZdpStatus::Success,
                nwk_addr_of_interest: self.local_nwk_addr(),
                node_descriptor: Some(*self.node_descriptor()),
            }
        } else {
            NodeDescRsp {
                status: ZdpStatus::DeviceNotFound,
                nwk_addr_of_interest: req.nwk_addr_of_interest,
                node_descriptor: None,
            }
        };
        rsp_data.serialize(rsp)
    }

    fn handle_power_desc_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = NodeDescReq::parse(payload)?; // same layout as PowerDescReq
        let rsp_data = if req.nwk_addr_of_interest == self.local_nwk_addr() {
            PowerDescRsp {
                status: ZdpStatus::Success,
                nwk_addr_of_interest: self.local_nwk_addr(),
                power_descriptor: Some(*self.power_descriptor()),
            }
        } else {
            PowerDescRsp {
                status: ZdpStatus::DeviceNotFound,
                nwk_addr_of_interest: req.nwk_addr_of_interest,
                power_descriptor: None,
            }
        };
        rsp_data.serialize(rsp)
    }

    fn handle_simple_desc_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = SimpleDescReq::parse(payload)?;
        if req.nwk_addr_of_interest != self.local_nwk_addr() {
            let rsp_data = SimpleDescRsp {
                status: ZdpStatus::DeviceNotFound,
                nwk_addr_of_interest: req.nwk_addr_of_interest,
                simple_descriptor: None,
            };
            return rsp_data.serialize(rsp);
        }
        match self.find_endpoint(req.endpoint) {
            Some(sd) => {
                let rsp_data = SimpleDescRsp {
                    status: ZdpStatus::Success,
                    nwk_addr_of_interest: self.local_nwk_addr(),
                    simple_descriptor: Some(sd.clone()),
                };
                rsp_data.serialize(rsp)
            }
            None => {
                let status = if req.endpoint == 0 || req.endpoint > 240 {
                    ZdpStatus::InvalidEp
                } else {
                    ZdpStatus::NotActive
                };
                let rsp_data = SimpleDescRsp {
                    status,
                    nwk_addr_of_interest: self.local_nwk_addr(),
                    simple_descriptor: None,
                };
                rsp_data.serialize(rsp)
            }
        }
    }

    fn handle_active_ep_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = NodeDescReq::parse(payload)?; // same layout
        if req.nwk_addr_of_interest != self.local_nwk_addr() {
            let rsp_data = ActiveEpRsp {
                status: ZdpStatus::DeviceNotFound,
                nwk_addr_of_interest: req.nwk_addr_of_interest,
                active_ep_list: heapless::Vec::new(),
            };
            return rsp_data.serialize(rsp);
        }
        let mut ep_list: heapless::Vec<u8, 32> = heapless::Vec::new();
        for sd in self.endpoints() {
            let _ = ep_list.push(sd.endpoint);
        }
        let rsp_data = ActiveEpRsp {
            status: ZdpStatus::Success,
            nwk_addr_of_interest: self.local_nwk_addr(),
            active_ep_list: ep_list,
        };
        rsp_data.serialize(rsp)
    }

    /// Handle Match_Desc_req.
    ///
    /// Returns `Ok(None)` when the request must be answered with silence: a
    /// broadcast Match_Desc_req is only answered by devices that actually
    /// match (R22 2.4.3.1.7), so neither NO_MATCH nor DEVICE_NOT_FOUND may be
    /// unicast back to the requester in that case. Unicast requests keep their
    /// explicit status response.
    fn handle_match_desc_req(
        &self,
        payload: &[u8],
        unicast: bool,
        rsp: &mut [u8],
    ) -> Result<Option<usize>, ZdoError> {
        let req = MatchDescReq::parse(payload)?;
        if req.nwk_addr_of_interest != self.local_short_address()
            && req.nwk_addr_of_interest != self.local_nwk_addr()
            && !is_broadcast_short(req.nwk_addr_of_interest)
        {
            if !unicast {
                return Ok(None);
            }
            let rsp_data = MatchDescRsp {
                status: ZdpStatus::DeviceNotFound,
                nwk_addr_of_interest: req.nwk_addr_of_interest,
                match_list: heapless::Vec::new(),
            };
            return rsp_data.serialize(rsp).map(Some);
        }
        let mut matches: heapless::Vec<u8, 32> = heapless::Vec::new();
        for sd in self.endpoints() {
            if sd.profile_id != req.profile_id {
                continue;
            }
            let mut matched = false;
            // Check input clusters
            for &req_cluster in req.input_clusters.iter() {
                if sd.input_clusters.contains(&req_cluster) {
                    matched = true;
                    break;
                }
            }
            // Check output clusters
            if !matched {
                for &req_cluster in req.output_clusters.iter() {
                    if sd.output_clusters.contains(&req_cluster) {
                        matched = true;
                        break;
                    }
                }
            }
            if matched {
                let _ = matches.push(sd.endpoint);
            }
        }
        let status = if matches.is_empty() {
            // Broadcast probes that this node cannot satisfy stay silent.
            if !unicast {
                return Ok(None);
            }
            ZdpStatus::NoMatch
        } else {
            ZdpStatus::Success
        };
        let rsp_data = MatchDescRsp {
            status,
            nwk_addr_of_interest: self.local_short_address(),
            match_list: matches,
        };
        rsp_data.serialize(rsp).map(Some)
    }

    // ── Binding management ──────────────────────────────────────

    fn handle_bind_req(&mut self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = BindReq::parse(payload)?;
        let entry = bind_req_to_entry(&req);
        let status = match self.aps_mut().binding_table_mut().add(entry) {
            Ok(()) => ZdpStatus::Success,
            Err(_) => ZdpStatus::TableFull,
        };
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        rsp[0] = status as u8;
        Ok(1)
    }

    fn handle_unbind_req(&mut self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = BindReq::parse(payload)?;
        let dst = bind_target_to_dst(&req.dst);
        let removed = self.aps_mut().binding_table_mut().remove(
            &req.src_addr,
            req.src_endpoint,
            req.cluster_id,
            &dst,
        );
        let status = if removed {
            ZdpStatus::Success
        } else {
            ZdpStatus::NoEntry
        };
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        rsp[0] = status as u8;
        Ok(1)
    }

    // ── Network management ──────────────────────────────────────

    fn handle_mgmt_lqi_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = MgmtLqiReq::parse(payload)?;
        let neighbor_table = self.nwk().neighbor_table();
        let total = neighbor_table.len() as u8;
        let start = req.start_index as usize;
        let mut list: heapless::Vec<NeighborTableRecord, 16> = heapless::Vec::new();
        for entry in neighbor_table.iter().skip(start) {
            if list.is_full() {
                break;
            }
            use zigbee_nwk::neighbor::{NeighborDeviceType, Relationship};
            let device_type = match entry.device_type {
                NeighborDeviceType::Coordinator => 0,
                NeighborDeviceType::Router => 1,
                NeighborDeviceType::EndDevice => 2,
                NeighborDeviceType::Unknown => 3,
            };
            let rx_on = u8::from(entry.rx_on_when_idle);
            let relationship = match entry.relationship {
                Relationship::Parent => 0,
                Relationship::Child => 1,
                Relationship::Sibling => 2,
                Relationship::PreviousChild => 4,
                Relationship::UnauthenticatedChild => 3,
            };
            let permit = if entry.permit_joining { 1 } else { 0 };
            let _ = list.push(NeighborTableRecord {
                extended_pan_id: entry.extended_pan_id,
                extended_addr: entry.ieee_address,
                network_addr: entry.network_address,
                device_type,
                rx_on_when_idle: rx_on,
                relationship,
                permit_joining: permit,
                depth: entry.depth,
                lqi: entry.lqi,
            });
        }
        let rsp_data = MgmtLqiRsp {
            status: ZdpStatus::Success,
            neighbor_table_entries: total,
            start_index: req.start_index,
            neighbor_table_list: list,
        };
        rsp_data.serialize(rsp)
    }

    fn handle_mgmt_rtg_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = MgmtRtgReq::parse(payload)?;
        let routing_table = self.nwk().routing_table();
        let total = routing_table.len() as u8;
        let start = req.start_index as usize;
        let mut list: heapless::Vec<RoutingTableRecord, 16> = heapless::Vec::new();
        for entry in routing_table.iter().skip(start) {
            if list.is_full() {
                break;
            }
            use zigbee_nwk::routing::RouteStatus;
            let status = match entry.status {
                RouteStatus::Active => 0,
                RouteStatus::DiscoveryUnderway => 1,
                RouteStatus::DiscoveryFailed => 2,
                RouteStatus::Inactive => 3,
                RouteStatus::ValidationUnderway => 4,
            };
            let _ = list.push(RoutingTableRecord {
                dst_addr: entry.destination,
                status,
                memory_constrained: false,
                many_to_one: entry.many_to_one,
                route_record_required: entry.route_record_required,
                next_hop: entry.next_hop,
            });
        }
        let rsp_data = MgmtRtgRsp {
            status: ZdpStatus::Success,
            routing_table_entries: total,
            start_index: req.start_index,
            routing_table_list: list,
        };
        rsp_data.serialize(rsp)
    }

    fn handle_mgmt_bind_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = MgmtBindReq::parse(payload)?;
        let entries = self.aps().binding_table().entries();
        let total = entries.len() as u8;
        let start = req.start_index as usize;
        let mut list: heapless::Vec<BindingTableRecord, 16> = heapless::Vec::new();
        for entry in entries.iter().skip(start) {
            if list.is_full() {
                break;
            }
            let _ = list.push(aps_binding_to_record(entry));
        }
        let rsp_data = MgmtBindRsp {
            status: ZdpStatus::Success,
            binding_table_entries: total,
            start_index: req.start_index,
            binding_table_list: list,
        };
        rsp_data.serialize(rsp)
    }

    fn handle_mgmt_leave_req(
        &self,
        src: ShortAddress,
        payload: &[u8],
        rsp: &mut [u8],
    ) -> Result<usize, ZdoError> {
        let req = MgmtLeaveReq::parse(payload)?;
        // Note: actual leave is triggered by setting a flag that the runtime polls.
        // We can't call async nlme_leave from a sync context, and the leave needs
        // to happen AFTER we've sent the response. Set a flag and return success.
        log::info!("[ZDO] Mgmt_Leave_req received — leave will be executed after response");
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        if self.nwk().device_type() == zigbee_nwk::DeviceType::EndDevice
            && src != self.nwk().nib().parent_address
            && src != ShortAddress::COORDINATOR
        {
            log::warn!(
                "[ZDO] Ignoring Mgmt_Leave_req from unauthorized source 0x{:04X}",
                src.0
            );
            return Err(ZdoError::InvalidData);
        }
        let local_ieee = self.nwk().nib().ieee_address;
        let targets_local_device = req.device_address == [0; 8] || req.device_address == local_ieee;
        rsp[0] = if targets_local_device {
            ZdpStatus::Success
        } else {
            ZdpStatus::NotSupported
        } as u8;
        Ok(1)
    }

    async fn handle_mgmt_permit_joining_req(
        &mut self,
        payload: &[u8],
        rsp: &mut [u8],
    ) -> Result<usize, ZdoError> {
        let req = MgmtPermitJoiningReq::parse(payload)?;
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        match self
            .nwk_mut()
            .nlme_permit_joining(req.permit_duration)
            .await
        {
            Ok(()) => {
                log::info!(
                    "[ZDO] Mgmt_Permit_Joining_req: duration={} tc_significance={}",
                    req.permit_duration,
                    req.tc_significance,
                );
                rsp[0] = ZdpStatus::Success as u8;
            }
            Err(e) => {
                log::warn!("[ZDO] Mgmt_Permit_Joining_req failed: {:?}", e,);
                rsp[0] = ZdpStatus::NotSupported as u8;
            }
        }
        Ok(1)
    }

    async fn handle_mgmt_nwk_update_req(
        &mut self,
        payload: &[u8],
        rsp: &mut [u8],
    ) -> Result<usize, ZdoError> {
        let req = MgmtNwkUpdateReq::parse(payload)?;
        match req {
            MgmtNwkUpdateReq::EdScan {
                scan_channels,
                scan_duration,
                scan_count,
            } => {
                log::info!(
                    "[ZDO] Mgmt_NWK_Update: ED scan channels=0x{scan_channels:08X} duration={scan_duration} count={scan_count}"
                );
                // Perform ED scan (use first scan_count iteration, repeat is optional)
                match self
                    .nwk_mut()
                    .nlme_ed_scan(zigbee_types::ChannelMask(scan_channels), scan_duration)
                    .await
                {
                    Ok(result) => {
                        let mut energy_values: heapless::Vec<u8, 16> = heapless::Vec::new();
                        for ed in &result.energy_list {
                            let _ = energy_values.push(ed.energy);
                        }
                        let rsp_data = MgmtNwkUpdateRsp {
                            status: ZdpStatus::Success,
                            scanned_channels: scan_channels,
                            total_transmissions: 0,
                            transmission_failures: 0,
                            energy_values,
                        };
                        rsp_data.serialize(rsp)
                    }
                    Err(e) => {
                        log::warn!("[ZDO] ED scan failed: {e:?}");
                        let rsp_data = MgmtNwkUpdateRsp {
                            status: ZdpStatus::NotSupported,
                            scanned_channels: scan_channels,
                            total_transmissions: 0,
                            transmission_failures: 0,
                            energy_values: heapless::Vec::new(),
                        };
                        rsp_data.serialize(rsp)
                    }
                }
            }
            MgmtNwkUpdateReq::ChannelChange {
                scan_channels,
                nwk_update_id,
            } => {
                // Find the single channel bit set in scan_channels
                let channel = (0u8..=26).find(|&ch| scan_channels & (1 << ch) != 0);
                if let Some(ch) = channel {
                    log::info!(
                        "[ZDO] Mgmt_NWK_Update: channel change to {ch} (update_id={nwk_update_id})"
                    );
                    match self.nwk_mut().nlme_set_channel(ch).await {
                        Ok(()) => {
                            self.nwk_mut().nib_mut().update_id = nwk_update_id;
                            if rsp.is_empty() {
                                return Err(ZdoError::BufferTooSmall);
                            }
                            rsp[0] = ZdpStatus::Success as u8;
                            Ok(1)
                        }
                        Err(_) => {
                            if rsp.is_empty() {
                                return Err(ZdoError::BufferTooSmall);
                            }
                            rsp[0] = ZdpStatus::InvRequestType as u8;
                            Ok(1)
                        }
                    }
                } else {
                    if rsp.is_empty() {
                        return Err(ZdoError::BufferTooSmall);
                    }
                    rsp[0] = ZdpStatus::InvRequestType as u8;
                    Ok(1)
                }
            }
            MgmtNwkUpdateReq::ManagerChange {
                nwk_update_id,
                nwk_manager_addr,
                ..
            } => {
                log::info!(
                    "[ZDO] Mgmt_NWK_Update: manager change to 0x{:04X} (update_id={nwk_update_id})",
                    nwk_manager_addr.0,
                );
                self.nwk_mut().nib_mut().nwk_manager_addr = nwk_manager_addr;
                self.nwk_mut().nib_mut().update_id = nwk_update_id;
                if rsp.is_empty() {
                    return Err(ZdoError::BufferTooSmall);
                }
                rsp[0] = ZdpStatus::Success as u8;
                Ok(1)
            }
        }
    }
}

// ── Conversion helpers ──────────────────────────────────────────

/// Convert a ZDP [`BindReq`] into an APS [`BindingEntry`].
fn bind_req_to_entry(req: &BindReq) -> BindingEntry {
    match req.dst {
        BindTarget::Group(group) => {
            BindingEntry::group(req.src_addr, req.src_endpoint, req.cluster_id, group)
        }
        BindTarget::Unicast {
            dst_addr,
            dst_endpoint,
        } => BindingEntry::unicast(
            req.src_addr,
            req.src_endpoint,
            req.cluster_id,
            dst_addr,
            dst_endpoint,
        ),
    }
}

/// Convert a ZDP [`BindTarget`] to an APS [`BindingDst`].
fn bind_target_to_dst(target: &BindTarget) -> BindingDst {
    match *target {
        BindTarget::Group(g) => BindingDst::Group(g),
        BindTarget::Unicast {
            dst_addr,
            dst_endpoint,
        } => BindingDst::Unicast {
            dst_addr,
            dst_endpoint,
        },
    }
}

/// Convert an APS [`BindingEntry`] into a ZDP [`BindingTableRecord`].
fn aps_binding_to_record(entry: &BindingEntry) -> BindingTableRecord {
    let (dst_addr_mode, dst) = match entry.dst {
        BindingDst::Group(g) => (BindingDstMode::Group as u8, BindTarget::Group(g)),
        BindingDst::Unicast {
            dst_addr,
            dst_endpoint,
        } => (
            BindingDstMode::Extended as u8,
            BindTarget::Unicast {
                dst_addr,
                dst_endpoint,
            },
        ),
    };
    BindingTableRecord {
        src_addr: entry.src_addr,
        src_endpoint: entry.src_endpoint,
        cluster_id: entry.cluster_id,
        dst_addr_mode,
        dst,
    }
}

// ── ZDP dispatcher tests ────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use zigbee_aps::{ApsAddress, ApsAddressMode, ApsLayer};
    use zigbee_mac::mock::MockMac;
    #[cfg(feature = "router")]
    use zigbee_nwk::ChildPollOutcome;
    use zigbee_nwk::{DeviceType, NwkLayer};
    use zigbee_types::PanId;
    #[cfg(feature = "router")]
    use zigbee_types::{IeeeAddress, MacAddress};

    const LOCAL_SHORT: ShortAddress = ShortAddress(0x1234);
    const LOCAL_IEEE: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    const PARENT: ShortAddress = ShortAddress(0x0000);
    #[cfg(feature = "router")]
    const CHILD_SHORT: ShortAddress = ShortAddress(0x4567);
    #[cfg(feature = "router")]
    const CHILD_IEEE: [u8; 8] = [0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97];
    #[cfg(feature = "router")]
    const CHILD_2_SHORT: ShortAddress = ShortAddress(0x4568);
    #[cfg(feature = "router")]
    const CHILD_2_IEEE: [u8; 8] = [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7];

    fn block_on<F: Future>(future: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
        fn noop(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut context = Context::from_waker(&waker);
        let mut future = core::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
        }
    }

    /// A joined end device with one Home Automation endpoint, so ZDP responses
    /// have a next hop (the parent) and Match_Desc_req has something to match.
    fn test_zdo() -> ZdoLayer<MockMac> {
        test_zdo_for(DeviceType::EndDevice)
    }

    fn test_zdo_for(device_type: DeviceType) -> ZdoLayer<MockMac> {
        let mac = MockMac::new(LOCAL_IEEE);
        let nwk = NwkLayer::new(mac, device_type);
        let aps = ApsLayer::new(nwk);
        let mut zdo = ZdoLayer::new(aps);
        {
            let nwk = zdo.nwk_mut();
            nwk.set_joined(true);
            let nib = nwk.nib_mut();
            nib.pan_id = PanId(0xABCD);
            nib.network_address = LOCAL_SHORT;
            nib.parent_address = PARENT;
            nib.ieee_address = LOCAL_IEEE;
        }
        zdo.set_local_nwk_addr(LOCAL_SHORT);
        zdo.set_local_ieee_addr(LOCAL_IEEE);

        let mut input_clusters = heapless::Vec::new();
        let _ = input_clusters.push(0x0000u16); // Basic
        let _ = input_clusters.push(0x0402u16); // Temperature measurement
        let desc = crate::descriptors::SimpleDescriptor {
            endpoint: 1,
            profile_id: 0x0104,
            device_id: 0x0302,
            device_version: 1,
            input_clusters,
            output_clusters: heapless::Vec::new(),
        };
        zdo.register_endpoint(desc).unwrap();
        zdo
    }

    fn indication<'a>(
        cluster: u16,
        dst_addr_mode: ApsAddressMode,
        dst_address: ApsAddress,
        payload: &'a [u8],
    ) -> ApsdeDataIndication<'a> {
        ApsdeDataIndication {
            dst_addr_mode,
            dst_address,
            dst_endpoint: ZDO_ENDPOINT,
            src_addr_mode: ApsAddressMode::Short,
            src_address: ApsAddress::Short(PARENT),
            src_endpoint: ZDO_ENDPOINT,
            profile_id: crate::ZDP_PROFILE_ID,
            cluster_id: cluster,
            payload,
            aps_counter: 1,
            security_status: true,
            lqi: 200,
        }
    }

    fn unicast(cluster: u16, payload: &[u8]) -> ApsdeDataIndication<'_> {
        indication(
            cluster,
            ApsAddressMode::Short,
            ApsAddress::Short(LOCAL_SHORT),
            payload,
        )
    }

    fn broadcast(cluster: u16, payload: &[u8]) -> ApsdeDataIndication<'_> {
        indication(
            cluster,
            ApsAddressMode::Short,
            ApsAddress::Short(ShortAddress::BROADCAST_RX_ON_WHEN_IDLE),
            payload,
        )
    }

    /// Decode the ZDP cluster and payload of the last frame the MAC sent.
    fn last_zdp_tx(zdo: &ZdoLayer<MockMac>) -> Option<(u16, heapless::Vec<u8, 64>)> {
        let record = zdo.nwk().mac().tx_history().last()?;
        let frame = record.payload.as_slice();
        let (_nwk, nwk_len) = zigbee_nwk::frames::NwkHeader::parse(frame)?;
        let aps_frame = frame.get(nwk_len..)?;
        let (aps, aps_len) = zigbee_aps::frames::ApsHeader::parse(aps_frame)?;
        let payload = aps_frame.get(aps_len..)?;
        let mut out = heapless::Vec::new();
        for &b in payload {
            out.push(b).ok()?;
        }
        Some((aps.cluster_id?, out))
    }

    fn tx_count(zdo: &ZdoLayer<MockMac>) -> usize {
        zdo.nwk().mac().tx_history().len()
    }

    #[cfg(feature = "router")]
    fn add_confirmed_child(zdo: &mut ZdoLayer<MockMac>, short: ShortAddress, ieee: IeeeAddress) {
        let nwk = zdo.nwk_mut();
        assert!(nwk.restore_child(ieee, short, false, true, false, 8));
        assert_eq!(
            block_on(nwk.service_child_data_request(MacAddress::Short(PanId(0xABCD), short)))
                .unwrap(),
            ChildPollOutcome::NoData
        );
    }

    #[cfg(feature = "router")]
    fn parent_annce_rsp(tsn: u8, status: u8, child: IeeeAddress) -> [u8; 11] {
        let mut payload = [0u8; 11];
        payload[0] = tsn;
        payload[1] = status;
        payload[2] = 1;
        payload[3..].copy_from_slice(&child);
        payload
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_annce_response_echoes_the_request_tsn() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        add_confirmed_child(&mut zdo, CHILD_SHORT, CHILD_IEEE);
        let mut payload = [0u8; 10];
        payload[0] = 0xA5;
        payload[1] = 1;
        payload[2..].copy_from_slice(&CHILD_IEEE);

        block_on(zdo.handle_indication(&broadcast(crate::PARENT_ANNCE, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("Parent_annce_rsp");
        assert_eq!(cluster, crate::PARENT_ANNCE_RSP);
        assert_eq!(body[0], 0xA5);
        assert_eq!(body[1], ZdpStatus::Success as u8);
        assert_eq!(body[2], 1);
        assert_eq!(&body[3..], &CHILD_IEEE);
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_annce_responses_require_success_and_an_open_transaction() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        add_confirmed_child(&mut zdo, CHILD_SHORT, CHILD_IEEE);
        add_confirmed_child(&mut zdo, CHILD_2_SHORT, CHILD_2_IEEE);

        let unsolicited = parent_annce_rsp(0xEE, ZdpStatus::Success as u8, CHILD_IEEE);
        block_on(zdo.handle_indication(&unicast(crate::PARENT_ANNCE_RSP, &unsolicited))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_some()
        );

        block_on(zdo.send_parent_annce()).unwrap();
        let (_, announcement) = last_zdp_tx(&zdo).expect("Parent_annce");
        let tsn = announcement[0];
        zdo.nwk_mut().mac_mut().clear_tx_history();

        let failed = parent_annce_rsp(tsn, ZdpStatus::NotSupported as u8, CHILD_IEEE);
        block_on(zdo.handle_indication(&unicast(crate::PARENT_ANNCE_RSP, &failed))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_some()
        );

        let wrong_tsn = parent_annce_rsp(tsn.wrapping_add(1), ZdpStatus::Success as u8, CHILD_IEEE);
        block_on(zdo.handle_indication(&unicast(crate::PARENT_ANNCE_RSP, &wrong_tsn))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_some()
        );

        let first = parent_annce_rsp(tsn, ZdpStatus::Success as u8, CHILD_IEEE);
        block_on(zdo.handle_indication(&unicast(crate::PARENT_ANNCE_RSP, &first))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_none()
        );

        let second = parent_annce_rsp(tsn, ZdpStatus::Success as u8, CHILD_2_IEEE);
        block_on(zdo.handle_indication(&unicast(crate::PARENT_ANNCE_RSP, &second))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_2_IEEE)
                .is_none(),
            "the transaction stays open for responses from multiple parents"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn expired_parent_annce_transactions_reject_late_responses() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        add_confirmed_child(&mut zdo, CHILD_SHORT, CHILD_IEEE);
        block_on(zdo.send_parent_annce()).unwrap();
        let (_, announcement) = last_zdp_tx(&zdo).expect("Parent_annce");
        let tsn = announcement[0];

        zdo.tick_parent_annce_transactions(crate::parent_annce::PARENT_ANNCE_RESPONSE_WINDOW_SECS);
        let late = parent_annce_rsp(tsn, ZdpStatus::Success as u8, CHILD_IEEE);
        block_on(zdo.handle_indication(&unicast(crate::PARENT_ANNCE_RSP, &late))).unwrap();

        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_some()
        );
    }

    #[test]
    fn mgmt_leave_self_target_accepts_remove_children() {
        let mut zdo = test_zdo();
        let mut payload = [0u8; 10];
        payload[0] = 0x42;
        payload[9] = 0x40;

        block_on(zdo.handle_indication(&unicast(crate::MGMT_LEAVE_REQ, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("Mgmt_Leave_rsp");
        assert_eq!(cluster, crate::MGMT_LEAVE_RSP);
        assert_eq!(body.as_slice(), &[0x42, ZdpStatus::Success as u8]);
    }

    // ── Unsupported / undefined clusters ────────────────────────

    #[test]
    fn undefined_unicast_request_answers_not_supported() {
        let mut zdo = test_zdo();
        // 0x0037 is not a defined ZDP request in this stack.
        let payload = [0x5Au8];
        block_on(zdo.handle_indication(&unicast(0x0037, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("response frame");
        assert_eq!(cluster, 0x8037);
        assert_eq!(body.as_slice(), &[0x5A, ZdpStatus::NotSupported as u8]);
        assert_eq!(zdo.diagnostics().last_response_cluster, 0x8037);
    }

    #[test]
    fn undefined_broadcast_request_is_dropped() {
        let mut zdo = test_zdo();
        let payload = [0x5Au8];
        block_on(zdo.handle_indication(&broadcast(0x0037, &payload))).unwrap();

        assert_eq!(tx_count(&zdo), 0);
        assert_eq!(zdo.diagnostics().response_attempts, 0);
    }

    #[test]
    fn undefined_group_request_is_dropped() {
        let mut zdo = test_zdo();
        let payload = [0x5Au8];
        let ind = indication(
            0x0037,
            ApsAddressMode::Group,
            ApsAddress::Group(0x0007),
            &payload,
        );
        block_on(zdo.handle_indication(&ind)).unwrap();

        assert_eq!(tx_count(&zdo), 0);
        assert_eq!(zdo.diagnostics().response_attempts, 0);
    }

    #[test]
    fn broadcast_nwk_address_miss_stays_silent() {
        let mut zdo = test_zdo();
        let mut payload = [0u8; 11];
        payload[0] = 0x5B;
        payload[1..9].copy_from_slice(&[0xAA; 8]);
        block_on(zdo.handle_indication(&broadcast(crate::NWK_ADDR_REQ, &payload))).unwrap();

        assert_eq!(tx_count(&zdo), 0);
    }

    #[test]
    fn broadcast_ieee_address_miss_stays_silent() {
        let mut zdo = test_zdo();
        let payload = [0x5C, 0x21, 0x43, 0x00, 0x00];
        block_on(zdo.handle_indication(&broadcast(crate::IEEE_ADDR_REQ, &payload))).unwrap();

        assert_eq!(tx_count(&zdo), 0);
    }

    #[test]
    fn broadcast_permit_joining_is_applied_without_a_response() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        let payload = [0x5D, 60, 1];
        block_on(zdo.handle_indication(&broadcast(crate::MGMT_PERMIT_JOINING_REQ, &payload)))
            .unwrap();

        assert!(zdo.nwk().nib().permit_joining);
        assert_eq!(zdo.nwk().nib().permit_joining_duration, 60);
        assert_eq!(tx_count(&zdo), 0);
    }

    #[test]
    fn unsolicited_response_cluster_is_never_answered() {
        let mut zdo = test_zdo();
        // A Node_Desc_rsp nobody asked for, and an undefined response cluster.
        for cluster in [crate::NODE_DESC_RSP, 0x80B7] {
            let payload = [0x11u8, 0x00];
            block_on(zdo.handle_indication(&unicast(cluster, &payload))).unwrap();
        }

        assert_eq!(tx_count(&zdo), 0);
        assert_eq!(zdo.diagnostics().response_attempts, 0);
    }

    // ── Match_Desc_req ──────────────────────────────────────────

    fn match_desc_payload(
        tsn: u8,
        addr_of_interest: ShortAddress,
        profile: u16,
        cluster: u16,
    ) -> [u8; 9] {
        let mut payload = [0u8; 9];
        payload[0] = tsn;
        payload[1..3].copy_from_slice(&addr_of_interest.0.to_le_bytes());
        payload[3..5].copy_from_slice(&profile.to_le_bytes());
        payload[5] = 1; // input cluster count
        payload[6..8].copy_from_slice(&cluster.to_le_bytes());
        payload[8] = 0; // output cluster count
        payload
    }

    #[test]
    fn broadcast_match_desc_without_match_stays_silent() {
        let mut zdo = test_zdo();
        // Profile matches, cluster does not.
        let payload = match_desc_payload(0x21, ShortAddress::BROADCAST, 0x0104, 0x0006);
        block_on(zdo.handle_indication(&broadcast(crate::MATCH_DESC_REQ, &payload))).unwrap();

        assert_eq!(tx_count(&zdo), 0);
        assert_eq!(zdo.diagnostics().response_attempts, 0);
    }

    #[test]
    fn broadcast_match_desc_with_match_responds() {
        let mut zdo = test_zdo();
        let payload = match_desc_payload(0x22, ShortAddress::BROADCAST, 0x0104, 0x0402);
        block_on(zdo.handle_indication(&broadcast(crate::MATCH_DESC_REQ, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("response frame");
        assert_eq!(cluster, crate::MATCH_DESC_RSP);
        assert_eq!(body[0], 0x22);
        assert_eq!(body[1], ZdpStatus::Success as u8);
        assert_eq!(
            u16::from_le_bytes([body[2], body[3]]),
            LOCAL_SHORT.0,
            "response reports this node's address"
        );
        assert_eq!(body[4], 1, "one matching endpoint");
        assert_eq!(body[5], 1, "endpoint 1 matched");
    }

    #[test]
    fn unicast_match_desc_without_match_answers_no_match() {
        let mut zdo = test_zdo();
        let payload = match_desc_payload(0x23, LOCAL_SHORT, 0x0104, 0x0006);
        block_on(zdo.handle_indication(&unicast(crate::MATCH_DESC_REQ, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("response frame");
        assert_eq!(cluster, crate::MATCH_DESC_RSP);
        assert_eq!(body[0], 0x23);
        assert_eq!(body[1], ZdpStatus::NoMatch as u8);
    }

    #[test]
    fn broadcast_match_desc_for_another_device_stays_silent() {
        let mut zdo = test_zdo();
        let payload = match_desc_payload(0x24, ShortAddress(0x4321), 0x0104, 0x0402);
        block_on(zdo.handle_indication(&broadcast(crate::MATCH_DESC_REQ, &payload))).unwrap();

        assert_eq!(tx_count(&zdo), 0);
    }

    #[test]
    fn unicast_match_desc_for_another_device_answers_device_not_found() {
        let mut zdo = test_zdo();
        let payload = match_desc_payload(0x25, ShortAddress(0x4321), 0x0104, 0x0402);
        block_on(zdo.handle_indication(&unicast(crate::MATCH_DESC_REQ, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("response frame");
        assert_eq!(cluster, crate::MATCH_DESC_RSP);
        assert_eq!(body[1], ZdpStatus::DeviceNotFound as u8);
    }

    // ── Malformed known requests ────────────────────────────────

    #[test]
    fn malformed_unicast_request_is_rejected_without_a_malformed_response() {
        let mut zdo = test_zdo();
        // Simple_Desc_req truncated: TSN only, no address or endpoint.
        let payload = [0x31u8];
        let result = block_on(zdo.handle_indication(&unicast(crate::SIMPLE_DESC_REQ, &payload)));

        assert_eq!(result, Err(ZdoError::InvalidLength));
        assert_eq!(tx_count(&zdo), 0);
    }

    #[test]
    fn malformed_broadcast_request_stays_silent() {
        let mut zdo = test_zdo();
        let payload = [0x32u8];
        block_on(zdo.handle_indication(&broadcast(crate::MATCH_DESC_REQ, &payload))).unwrap();

        assert_eq!(tx_count(&zdo), 0);
    }

    // ── Supported requests keep working ─────────────────────────

    #[test]
    fn unicast_active_ep_request_still_answers() {
        let mut zdo = test_zdo();
        let mut payload = [0u8; 3];
        payload[0] = 0x41;
        payload[1..3].copy_from_slice(&LOCAL_SHORT.0.to_le_bytes());
        block_on(zdo.handle_indication(&unicast(crate::ACTIVE_EP_REQ, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("response frame");
        assert_eq!(cluster, crate::ACTIVE_EP_RSP);
        assert_eq!(body[0], 0x41);
        assert_eq!(body[1], ZdpStatus::Success as u8);
    }
}
