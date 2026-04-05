//! NWK command frame send helpers.
//!
//! Provides methods on NwkLayer to build and transmit NWK command frames
//! (Route Request, Route Reply, Link Status, Route Record).

use crate::frames::{
    EdTimeoutRequest, LinkStatusCommand, LinkStatusEntry, NetworkStatusCommand, NwkCommandId,
    NwkFrameControl, NwkFrameType, NwkHeader, RouteReply, RouteRequest,
};
use crate::{NwkLayer, NwkStatus};
use zigbee_mac::{AddressMode, MacDriver, McpsDataRequest, TxOptions};
use zigbee_types::*;

impl<M: MacDriver> NwkLayer<M> {
    /// Build and send a NWK command frame.
    async fn send_nwk_command(
        &mut self,
        dst_addr: ShortAddress,
        cmd_id: NwkCommandId,
        cmd_payload: &[u8],
    ) -> Result<(), NwkStatus> {
        if !self.joined {
            return Err(NwkStatus::InvalidRequest);
        }

        let seq = self.nib.next_seq();
        let is_broadcast = dst_addr.0 >= 0xFFF8;

        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: self.nib.security_enabled,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: true,
                end_device_initiator: false,
            },
            dst_addr,
            src_addr: self.nib.network_address,
            radius: if is_broadcast { 30 } else { 10 },
            seq_number: seq,
            dst_ieee: None,
            src_ieee: Some(self.nib.ieee_address),
            multicast_control: None,
            source_route: None,
        };

        // Assemble full command: [cmd_id, ...cmd_payload]
        let mut full_cmd = [0u8; 80];
        full_cmd[0] = cmd_id as u8;
        let cmd_len = 1 + cmd_payload.len();
        if cmd_len > full_cmd.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        full_cmd[1..cmd_len].copy_from_slice(cmd_payload);

        let mut buf = [0u8; 128];
        let hdr_len = header.serialize(&mut buf);

        let total_len;
        if self.nib.security_enabled {
            let sec_hdr = crate::security::NwkSecurityHeader {
                security_control: crate::security::NwkSecurityHeader::ZIGBEE_DEFAULT,
                frame_counter: self
                    .nib
                    .next_frame_counter()
                    .ok_or(NwkStatus::InvalidRequest)?,
                source_address: self.nib.ieee_address,
                key_seq_number: self.nib.active_key_seq_number,
            };
            let sec_hdr_len = sec_hdr.serialize(&mut buf[hdr_len..]);
            let aad_len = hdr_len + sec_hdr_len;

            if let Some(key_entry) = self.security.active_key() {
                if let Some(encrypted) = self.security.encrypt(
                    &buf[..aad_len],
                    &full_cmd[..cmd_len],
                    &key_entry.key,
                    &sec_hdr,
                ) {
                    if aad_len + encrypted.len() > buf.len() {
                        return Err(NwkStatus::FrameTooLong);
                    }
                    buf[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
                    total_len = aad_len + encrypted.len();
                } else {
                    return Err(NwkStatus::InvalidRequest);
                }
            } else {
                return Err(NwkStatus::NoKey);
            }
        } else {
            if hdr_len + cmd_len > buf.len() {
                return Err(NwkStatus::FrameTooLong);
            }
            buf[hdr_len..hdr_len + cmd_len].copy_from_slice(&full_cmd[..cmd_len]);
            total_len = hdr_len + cmd_len;
        }

        let next_hop = self.resolve_next_hop(dst_addr)?;

        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &buf[..total_len],
                msdu_handle: seq,
                tx_options: TxOptions {
                    ack_tx: next_hop.0 != 0xFFFF,
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::RouteError)?;

        Ok(())
    }

    /// Send a Route Request (RREQ) broadcast.
    pub async fn send_route_request(
        &mut self,
        dest: ShortAddress,
        path_cost: u8,
    ) -> Result<(), NwkStatus> {
        let rreq_id = self.nib.next_route_request_id();
        let rreq = RouteRequest {
            command_options: 0x00,
            route_request_id: rreq_id,
            dst_addr: dest,
            path_cost,
            dst_ieee: None,
        };
        let mut payload = [0u8; 16];
        let len = rreq.serialize(&mut payload);

        // Mark discovery in routing table
        self.routing.mark_discovery(dest);

        self.send_nwk_command(
            ShortAddress::BROADCAST,
            NwkCommandId::RouteRequest,
            &payload[..len],
        )
        .await
    }

    /// Send a Route Reply (RREP) unicast toward the originator.
    pub async fn send_route_reply(
        &mut self,
        dest: ShortAddress,
        originator: ShortAddress,
        path_cost: u8,
    ) -> Result<(), NwkStatus> {
        let rrep = RouteReply {
            command_options: 0x00,
            route_request_id: 0,
            originator,
            responder: self.nib.network_address,
            path_cost,
            originator_ieee: None,
            responder_ieee: None,
        };
        let mut payload = [0u8; 32];
        let len = rrep.serialize(&mut payload);

        self.send_nwk_command(dest, NwkCommandId::RouteReply, &payload[..len])
            .await
    }

    /// Send Link Status to all neighbors (broadcast).
    pub async fn send_link_status(&mut self) -> Result<(), NwkStatus> {
        let mut entries = heapless::Vec::<LinkStatusEntry, 16>::new();
        for neighbor in self.neighbors.iter() {
            let _ = entries.push(LinkStatusEntry {
                address: neighbor.network_address,
                incoming_cost: neighbor.outgoing_cost,
                outgoing_cost: neighbor.outgoing_cost,
            });
        }
        let ls = LinkStatusCommand { entries };
        let mut payload = [0u8; 64];
        let len = ls.serialize(&mut payload);

        self.send_nwk_command(
            ShortAddress::BROADCAST,
            NwkCommandId::LinkStatus,
            &payload[..len],
        )
        .await
    }

    /// Send a Route Record unicast to the destination.
    pub async fn send_route_record(
        &mut self,
        dest: ShortAddress,
        relay_list: &[ShortAddress],
    ) -> Result<(), NwkStatus> {
        let mut payload = [0u8; 40];
        let count = relay_list.len().min(16);
        payload[0] = count as u8;
        let mut offset = 1;
        for relay in &relay_list[..count] {
            payload[offset] = (relay.0 & 0xFF) as u8;
            payload[offset + 1] = ((relay.0 >> 8) & 0xFF) as u8;
            offset += 2;
        }

        self.send_nwk_command(dest, NwkCommandId::RouteRecord, &payload[..offset])
            .await
    }

    /// Send a Network Status command (NWK command 0x03) for route errors.
    pub async fn send_network_status(
        &mut self,
        dest: ShortAddress,
        status_code: u8,
        failed_destination: ShortAddress,
    ) -> Result<(), NwkStatus> {
        let ns = NetworkStatusCommand {
            status_code,
            destination: failed_destination,
        };
        let mut payload = [0u8; 4];
        let len = ns.serialize(&mut payload);

        self.send_nwk_command(dest, NwkCommandId::NetworkStatus, &payload[..len])
            .await
    }

    /// Send a Many-to-One Route Request (RREQ) broadcast.
    ///
    /// Used by concentrators (coordinators) to establish reverse routes
    /// from all routers back to the concentrator.
    pub async fn send_many_to_one_rreq(&mut self) -> Result<(), NwkStatus> {
        let rreq_id = self.nib.next_route_request_id();
        let rreq = RouteRequest {
            command_options: 0x08, // Bit 3 = many-to-one
            route_request_id: rreq_id,
            dst_addr: self.nib.network_address, // Concentrator is both source and dest
            path_cost: 0,
            dst_ieee: None,
        };
        let mut payload = [0u8; 16];
        let len = rreq.serialize(&mut payload);

        log::info!(
            "[NWK] Sending many-to-one RREQ (id={}, addr=0x{:04X})",
            rreq_id,
            self.nib.network_address.0,
        );

        self.send_nwk_command(
            ShortAddress::BROADCAST,
            NwkCommandId::RouteRequest,
            &payload[..len],
        )
        .await
    }

    /// Drain and send all queued route replies and RREQ rebroadcasts.
    ///
    /// Call this after `process_incoming_nwk_frame` returns so that
    /// deferred RREPs and RREQs (generated in sync command handlers) get
    /// transmitted asynchronously.
    pub async fn process_pending_routing(&mut self) {
        while let Some(pending) = self.pending_route_replies.pop() {
            let rrep = RouteReply {
                command_options: 0x00,
                route_request_id: pending.route_request_id,
                originator: pending.originator,
                responder: pending.responder,
                path_cost: pending.path_cost,
                originator_ieee: None,
                responder_ieee: None,
            };
            let mut payload = [0u8; 32];
            let len = rrep.serialize(&mut payload);

            if let Err(e) = self
                .send_nwk_command(pending.next_hop, NwkCommandId::RouteReply, &payload[..len])
                .await
            {
                log::warn!(
                    "[NWK] Failed to send queued RREP to 0x{:04X}: {:?}",
                    pending.next_hop.0,
                    e
                );
            }
        }

        // Drain RREQ rebroadcasts
        while let Some(pending) = self.pending_rreq_rebroadcasts.pop() {
            let rreq = RouteRequest {
                command_options: pending.command_options,
                route_request_id: pending.route_request_id,
                dst_addr: pending.dst_addr,
                path_cost: pending.path_cost,
                dst_ieee: None,
            };
            let mut payload = [0u8; 16];
            let len = rreq.serialize(&mut payload);

            if let Err(e) = self
                .send_nwk_command(
                    ShortAddress::BROADCAST,
                    NwkCommandId::RouteRequest,
                    &payload[..len],
                )
                .await
            {
                log::warn!(
                    "[NWK] Failed to rebroadcast RREQ for 0x{:04X}: {:?}",
                    pending.dst_addr.0,
                    e
                );
            }
        }

        // Send link status if due
        if self.link_status_due {
            self.link_status_due = false;
            if let Err(e) = self.send_link_status().await {
                log::warn!("[NWK] Failed to send periodic link status: {:?}", e);
            }
        }

        // Drain pending Network Status (route error) notifications
        while let Some(pending) = self.pending_route_errors.pop() {
            if let Err(e) = self
                .send_network_status(
                    pending.destination,
                    pending.status_code,
                    pending.failed_destination,
                )
                .await
            {
                log::warn!(
                    "[NWK] Failed to send NetworkStatus to 0x{:04X}: {:?}",
                    pending.destination.0,
                    e
                );
            }
        }

        // Send concentrator many-to-one RREQ if due
        if self.concentrator_rreq_due {
            self.concentrator_rreq_due = false;
            if let Err(e) = self.send_many_to_one_rreq().await {
                log::warn!("[NWK] Failed to send concentrator RREQ: {:?}", e);
            }
        }
    }

    /// Send End Device Timeout Request to parent after joining.
    ///
    /// Requests the maximum timeout (index 14 = ~11 days) so the parent
    /// keeps our entry in its neighbor table even during extended sleep.
    pub async fn send_ed_timeout_request(&mut self) -> Result<(), NwkStatus> {
        if self.device_type != crate::DeviceType::EndDevice {
            return Ok(()); // Only end devices send this
        }
        let req = EdTimeoutRequest::max_timeout();
        let mut payload = [0u8; 2];
        let len = req.serialize(&mut payload);
        log::info!(
            "[NWK] Sending ED Timeout Request (index={}, ~11 days) to parent 0x{:04X}",
            req.requested_timeout,
            self.nib.parent_address.0
        );
        self.send_nwk_command(
            self.nib.parent_address,
            NwkCommandId::EdTimeoutRequest,
            &payload[..len],
        )
        .await
    }
}
