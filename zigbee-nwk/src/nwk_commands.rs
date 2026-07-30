//! NWK command frame send helpers.
//!
//! Provides methods on NwkLayer to build and transmit NWK command frames
//! (Route Request, Route Reply, Link Status, Route Record).

use crate::frames::{
    EdTimeoutRequest, LinkStatusCommand, LinkStatusEntry, NetworkStatusCommand, NwkCommandId,
    NwkFrameControl, NwkFrameType, NwkHeader, RouteReply, RouteRequest,
};
use crate::nlde::{is_nwk_broadcast, is_unicast_address};
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
        let is_broadcast = is_nwk_broadcast(dst_addr);
        let radius = if is_broadcast { 30 } else { 10 };
        self.send_nwk_command_with_radius(dst_addr, cmd_id, cmd_payload, radius)
            .await
    }

    /// Build and send a NWK command frame with explicit radius.
    async fn send_nwk_command_with_radius(
        &mut self,
        dst_addr: ShortAddress,
        cmd_id: NwkCommandId,
        cmd_payload: &[u8],
        radius: u8,
    ) -> Result<(), NwkStatus> {
        if !self.joined {
            return Err(NwkStatus::InvalidRequest);
        }

        let seq = self.nib.next_seq();

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
            radius,
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

        // The shared builder owns header serialization, key selection, the
        // durable frame-counter reservation and CCM*, so commands, data and
        // relayed frames are all secured the same way.
        let mut buf = [0u8; crate::nlde::MAX_NWK_FRAME];
        let total_len = self.build_nwk_frame(&header, &full_cmd[..cmd_len], &mut buf)?;

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
        self.send_route_request_with_id(rreq_id, dest, path_cost)
            .await
    }

    /// Send a Route Request (RREQ) broadcast with a caller-chosen request ID.
    async fn send_route_request_with_id(
        &mut self,
        rreq_id: u8,
        dest: ShortAddress,
        path_cost: u8,
    ) -> Result<(), NwkStatus> {
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

    /// Start an AODV route discovery for `dest`.
    ///
    /// Records the pending discovery before transmitting so repeated data
    /// requests for the same destination do not flood the network with Route
    /// Requests; the record is failed again by
    /// [`NwkLayer::tick_router_maintenance`] after
    /// [`crate::routing::ROUTE_DISCOVERY_TIMEOUT_US`].
    ///
    /// If the Route Request never reaches the air — no network key, an
    /// exhausted frame-counter reservation, a MAC transmit error — that record
    /// is withdrawn again and the original error is returned, so the caller
    /// may retry immediately instead of waiting out a discovery that was never
    /// started.
    ///
    /// Returns the allocated route request ID.
    pub async fn discover_route(&mut self, dest: ShortAddress) -> Result<u8, NwkStatus> {
        if !self.can_route() {
            // Non-routing builds and end devices have no routing table to
            // install the answer into — fail instead of broadcasting a RREQ
            // whose Route Reply would be discarded.
            return Err(NwkStatus::InvalidRequest);
        }
        if !is_unicast_address(dest) || dest == self.nib.network_address {
            // Broadcast, reserved and unassigned destinations name no device
            // that a Route Reply could ever come back from.
            return Err(NwkStatus::InvalidParameter);
        }

        let rreq_id = self.nib.next_route_request_id();
        let now = self.mac.monotonic_micros();
        if self
            .routing
            .add_discovery(crate::routing::RouteDiscovery {
                request_id: rreq_id,
                destination: dest,
                sender: self.nib.network_address,
                forward_cost: 0,
                residual_cost: 0xFF,
                timestamp: now,
                active: true,
            })
            .is_err()
        {
            log::warn!("[NWK] Route discovery table full for 0x{:04X}", dest.0);
            return Err(NwkStatus::RouteDiscoveryFailed);
        }

        if let Err(e) = self.send_route_request_with_id(rreq_id, dest, 0).await {
            // Nothing was broadcast, so nothing may be waited for: withdraw
            // the record this call installed and report why the discovery
            // could not be started.
            self.routing.fail_discovery(rreq_id);
            log::warn!(
                "[NWK] Route Request for 0x{:04X} not sent ({:?}); discovery withdrawn",
                dest.0,
                e,
            );
            return Err(e);
        }
        Ok(rreq_id)
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
    ///
    /// The originator normally passes an empty relay list: every router the
    /// record passes through appends itself (see
    /// [`NwkLayer::process_incoming_nwk_frame`]), and the concentrator stores
    /// the path it finally receives.
    pub async fn send_route_record(
        &mut self,
        dest: ShortAddress,
        relay_list: &[ShortAddress],
    ) -> Result<(), NwkStatus> {
        // The path is bounded by what a concentrator can store and
        // source-route over. A longer one is refused rather than silently
        // truncated into a record that names the wrong hops.
        if relay_list.len() > crate::routing::MAX_SOURCE_ROUTE_RELAYS {
            return Err(NwkStatus::FrameTooLong);
        }
        let mut payload = [0u8; 1 + 2 * crate::routing::MAX_SOURCE_ROUTE_RELAYS];
        payload[0] = relay_list.len() as u8;
        let mut offset = 1;
        for relay in relay_list {
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
            command_options: self.concentrator_type.rreq_options(),
            route_request_id: rreq_id,
            dst_addr: self.nib.network_address, // Concentrator is both source and dest
            path_cost: 0,
            dst_ieee: None,
        };
        let mut payload = [0u8; 16];
        let len = rreq.serialize(&mut payload);

        log::info!(
            "[NWK] Sending many-to-one RREQ (id={}, addr=0x{:04X}, {:?})",
            rreq_id,
            self.nib.network_address.0,
            self.concentrator_type,
        );

        // Use concentrator_radius instead of default broadcast radius
        let radius = self.concentrator_radius;
        self.send_nwk_command_with_radius(
            ShortAddress::BROADCAST,
            NwkCommandId::RouteRequest,
            &payload[..len],
            radius,
        )
        .await
    }

    /// Forward an accepted Route Request one hop further.
    ///
    /// The frame that goes on air is the *originator's* broadcast, not a new
    /// one of ours:
    ///
    /// - the NWK source address stays the originator's, so every receiver's
    ///   broadcast transaction record recognises the copies that come back
    ///   through other neighbours and suppresses them;
    /// - the NWK sequence number and the end-device-initiator bit are
    ///   preserved for the same reason, and because they belong to the
    ///   originator's broadcast rather than to this hop;
    /// - the radius is the received one minus one, so the flood stays inside
    ///   the bound the originator set;
    /// - only the RREQ path cost changes, carrying the cost of the path this
    ///   request travelled to reach the next hop.
    ///
    /// NWK security is hop by hop, so the frame is re-secured through the
    /// shared builder with *this* device's IEEE address and a fresh durable
    /// frame counter over the header that actually goes on air.
    async fn forward_route_request(
        &mut self,
        pending: &crate::QueuedRreqForward,
    ) -> Result<(), NwkStatus> {
        if !self.can_route() {
            // Membership can end between accepting the request and this
            // maintenance pass (a Leave, or a failed rejoin).
            return Err(NwkStatus::InvalidRequest);
        }

        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: pending.frame_control.protocol_version,
                discover_route: pending.frame_control.discover_route,
                // A Route Request is a plain broadcast: neither subframe is
                // carried, so their flags are cleared rather than preserved
                // over a header that no longer has them.
                multicast: false,
                security: pending.frame_control.security,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: pending.src_ieee.is_some(),
                end_device_initiator: pending.frame_control.end_device_initiator,
            },
            dst_addr: pending.dst_addr,
            src_addr: pending.originator,
            radius: pending.radius,
            seq_number: pending.seq_number,
            dst_ieee: None,
            src_ieee: pending.src_ieee,
            multicast_control: None,
            source_route: None,
        };

        let rreq = RouteRequest {
            command_options: pending.command_options,
            route_request_id: pending.route_request_id,
            dst_addr: pending.rreq_dst,
            path_cost: pending.path_cost,
            dst_ieee: pending.rreq_dst_ieee,
        };
        let mut cmd = [0u8; 16];
        cmd[0] = NwkCommandId::RouteRequest as u8;
        let cmd_len = 1 + rreq.serialize(&mut cmd[1..]);

        let mut buf = [0u8; crate::nlde::MAX_NWK_FRAME];
        let total_len = self.build_nwk_frame(&header, &cmd[..cmd_len], &mut buf)?;

        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, ShortAddress::BROADCAST),
                payload: &buf[..total_len],
                // MAC transaction handle only — the NWK sequence number in the
                // header above stays the originator's.
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: false,
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::RouteError)?;

        Ok(())
    }

    /// Drain and send all queued route replies and Route Request forwards.
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

        // Drain accepted Route Request forwards
        while let Some(pending) = self.pending_rreq_forwards.pop() {
            if let Err(e) = self.forward_route_request(&pending).await {
                log::warn!(
                    "[NWK] Failed to forward RREQ for 0x{:04X}: {:?}",
                    pending.rreq_dst.0,
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
