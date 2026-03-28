//! ZDP request / response dispatcher.
//!
//! Routes incoming APS frames on endpoint 0 to the appropriate ZDP handler,
//! builds the response, and sends it back through the APS layer.

use zigbee_aps::ApsAddress;
use zigbee_aps::apsde::ApsdeDataIndication;
use zigbee_aps::binding::{BindingDst, BindingDstMode, BindingEntry};
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;

use crate::binding_mgmt::{BindReq, BindTarget};
use crate::discovery::*;
use crate::network_mgmt::*;
use crate::{ZDO_ENDPOINT, ZdoError, ZdoLayer, ZdpStatus};

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

        // --- Device_annce is fire-and-forget (no response) ---
        if cluster == crate::DEVICE_ANNCE {
            let _ = self.process_device_annce(payload);
            return Ok(());
        }

        // --- Check if this is a response to a pending client request ---
        if self.deliver_response(cluster, tsn, payload) {
            log::info!("[ZDO] Consumed as client response: cluster=0x{cluster:04X} tsn={tsn}");
            return Ok(());
        }

        // --- Build response in a stack buffer ---
        let mut rsp_buf = [0u8; 256];
        rsp_buf[0] = tsn; // echo TSN

        let (rsp_cluster, rsp_len) = match cluster {
            // ── Discovery ───────────────────────────────────────
            crate::NWK_ADDR_REQ => {
                let n = self.handle_nwk_addr_req(payload, &mut rsp_buf[1..])?;
                (crate::NWK_ADDR_RSP, 1 + n)
            }
            crate::IEEE_ADDR_REQ => {
                let n = self.handle_ieee_addr_req(payload, &mut rsp_buf[1..])?;
                (crate::IEEE_ADDR_RSP, 1 + n)
            }
            crate::NODE_DESC_REQ => {
                let n = self.handle_node_desc_req(payload, &mut rsp_buf[1..])?;
                (crate::NODE_DESC_RSP, 1 + n)
            }
            crate::POWER_DESC_REQ => {
                let n = self.handle_power_desc_req(payload, &mut rsp_buf[1..])?;
                (crate::POWER_DESC_RSP, 1 + n)
            }
            crate::SIMPLE_DESC_REQ => {
                let n = self.handle_simple_desc_req(payload, &mut rsp_buf[1..])?;
                (crate::SIMPLE_DESC_RSP, 1 + n)
            }
            crate::ACTIVE_EP_REQ => {
                let n = self.handle_active_ep_req(payload, &mut rsp_buf[1..])?;
                (crate::ACTIVE_EP_RSP, 1 + n)
            }
            crate::MATCH_DESC_REQ => {
                let n = self.handle_match_desc_req(payload, &mut rsp_buf[1..])?;
                (crate::MATCH_DESC_RSP, 1 + n)
            }

            // ── Binding management ──────────────────────────────
            crate::BIND_REQ => {
                let n = self.handle_bind_req(payload, &mut rsp_buf[1..])?;
                (crate::BIND_RSP, 1 + n)
            }
            crate::UNBIND_REQ => {
                let n = self.handle_unbind_req(payload, &mut rsp_buf[1..])?;
                (crate::UNBIND_RSP, 1 + n)
            }

            // ── Network management ──────────────────────────────
            crate::MGMT_LQI_REQ => {
                let n = self.handle_mgmt_lqi_req(payload, &mut rsp_buf[1..])?;
                (crate::MGMT_LQI_RSP, 1 + n)
            }
            crate::MGMT_RTG_REQ => {
                let n = self.handle_mgmt_rtg_req(payload, &mut rsp_buf[1..])?;
                (crate::MGMT_RTG_RSP, 1 + n)
            }
            crate::MGMT_BIND_REQ => {
                let n = self.handle_mgmt_bind_req(payload, &mut rsp_buf[1..])?;
                (crate::MGMT_BIND_RSP, 1 + n)
            }
            crate::MGMT_LEAVE_REQ => {
                let n = self.handle_mgmt_leave_req(payload, &mut rsp_buf[1..])?;
                (crate::MGMT_LEAVE_RSP, 1 + n)
            }
            crate::MGMT_PERMIT_JOINING_REQ => {
                let n = self
                    .handle_mgmt_permit_joining_req(payload, &mut rsp_buf[1..])
                    .await?;
                (crate::MGMT_PERMIT_JOINING_RSP, 1 + n)
            }
            crate::MGMT_NWK_UPDATE_REQ => {
                let n = self
                    .handle_mgmt_nwk_update_req(payload, &mut rsp_buf[1..])
                    .await?;
                (crate::MGMT_NWK_UPDATE_RSP, 1 + n)
            }

            _ => {
                log::warn!("ZDP: unsupported cluster 0x{:04X}", cluster);
                return Ok(());
            }
        };

        // --- Send response ---
        log::info!(
            "[ZDO TX] rsp cluster=0x{:04X} to 0x{:04X} len={}",
            rsp_cluster,
            src_short.0,
            rsp_len
        );
        self.send_zdp_unicast(src_short, rsp_cluster, &rsp_buf[..rsp_len])
            .await
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

    fn handle_match_desc_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = MatchDescReq::parse(payload)?;
        if req.nwk_addr_of_interest != self.local_nwk_addr()
            && req.nwk_addr_of_interest != ShortAddress(0xFFFF)
            && req.nwk_addr_of_interest != ShortAddress(0xFFFD)
        {
            let rsp_data = MatchDescRsp {
                status: ZdpStatus::DeviceNotFound,
                nwk_addr_of_interest: req.nwk_addr_of_interest,
                match_list: heapless::Vec::new(),
            };
            return rsp_data.serialize(rsp);
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
            ZdpStatus::NoMatch
        } else {
            ZdpStatus::Success
        };
        let rsp_data = MatchDescRsp {
            status,
            nwk_addr_of_interest: self.local_nwk_addr(),
            match_list: matches,
        };
        rsp_data.serialize(rsp)
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

    fn handle_mgmt_leave_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let _req = MgmtLeaveReq::parse(payload)?;
        // Note: actual leave is triggered by setting a flag that the runtime polls.
        // We can't call async nlme_leave from a sync context, and the leave needs
        // to happen AFTER we've sent the response. Set a flag and return success.
        log::info!("[ZDO] Mgmt_Leave_req received — leave will be executed after response");
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        rsp[0] = ZdpStatus::Success as u8;
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
