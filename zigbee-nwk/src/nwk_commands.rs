//! NWK command frame send helpers.
//!
//! Provides methods on NwkLayer to build and transmit NWK command frames
//! (Route Request, Route Reply, Link Status, Route Record).

use crate::frames::{
    EdTimeoutRequest, LINK_STATUS_ENTRIES_PER_FRAME, LinkStatusCommand, NetworkStatusCommand,
    NwkCommandId, NwkFrameControl, NwkFrameType, NwkHeader, PanIdConflictReport, PanIdUpdate,
    RouteReply, RouteRequest,
};
use crate::nlde::{is_nwk_broadcast, is_unicast_address};
use crate::routing::routing_random_sample;
use crate::{NwkLayer, NwkStatus};
use zigbee_mac::{AddressMode, MacDriver, McpsDataRequest, TxOptions};
use zigbee_types::*;

const ALL_ROUTERS_AND_COORDINATOR: ShortAddress = ShortAddress(0xFFFC);
/// R22 §3.4.8.2 — a Link Status frame is a one-hop broadcast.
const LINK_STATUS_RADIUS: u8 = 1;
const RREQ_RELAY_RETRIES: u8 = 2;
const RREQ_RETRY_INTERVAL_US: u32 = 254_000;
const MIN_RREQ_JITTER_MS: u32 = 2;
const MAX_RREQ_JITTER_MS: u32 = 128;

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
        self.send_nwk_command_from_with_radius(
            dst_addr,
            self.nib.network_address,
            Some(self.nib.ieee_address),
            None,
            cmd_id,
            cmd_payload,
            radius,
            None,
        )
        .await
    }

    /// Build and send a NWK command with an explicit NWK source and optional
    /// MAC next hop.
    ///
    /// The explicit source is required for a parent-generated Route Record on
    /// behalf of its end-device child. The next-hop override is required for a
    /// many-to-one route failure, which R22 injects through a random router
    /// neighbor instead of consulting the failed route.
    #[allow(clippy::too_many_arguments)]
    async fn send_nwk_command_from_with_radius(
        &mut self,
        dst_addr: ShortAddress,
        src_addr: ShortAddress,
        src_ieee: Option<IeeeAddress>,
        dst_ieee: Option<IeeeAddress>,
        cmd_id: NwkCommandId,
        cmd_payload: &[u8],
        radius: u8,
        explicit_next_hop: Option<ShortAddress>,
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
                dst_ieee_present: dst_ieee.is_some(),
                src_ieee_present: src_ieee.is_some(),
                end_device_initiator: false,
            },
            dst_addr,
            src_addr,
            radius,
            seq_number: seq,
            dst_ieee,
            src_ieee,
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

        let next_hop = match explicit_next_hop {
            Some(next_hop) => next_hop,
            None => self.resolve_next_hop(dst_addr)?,
        };

        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &buf[..total_len],
                msdu_handle: seq,
                tx_options: TxOptions {
                    ack_tx: next_hop != ShortAddress::BROADCAST,
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

    /// Send this device's Link Status (R22 §3.4.8, §3.6.3.4.1).
    ///
    /// The link status list holds one entry per **router** neighbor, sorted
    /// ascending by network address. Each entry advertises this device's own
    /// estimate of the incoming link cost and the outgoing cost currently held
    /// in the neighbor table, so the receiving router learns the reverse cost
    /// of the link it shares with us.
    ///
    /// The list is fragmented when it does not fit in one frame. R22
    /// §3.4.8.3.2 requires consecutive frames to overlap by one entry — the
    /// last address of frame N is the first address of frame N+1 — so the
    /// address ranges the receiver derives from the first/last flags are
    /// contiguous. A single frame sets both the first-frame and last-frame
    /// bits.
    ///
    /// Each frame is a one-hop broadcast to the router-only broadcast address
    /// with radius 1 and no MAC retries. End devices do not send Link Status
    /// frames (R22 §3.6.3.4.1), so this is a no-op for them.
    pub async fn send_link_status(&mut self) -> Result<(), NwkStatus> {
        if self.device_type == crate::DeviceType::EndDevice {
            return Ok(());
        }

        let entries = self.neighbors.router_link_status_entries();
        let mut payload = [0u8; 1 + 3 * LINK_STATUS_ENTRIES_PER_FRAME];
        let mut sent = 0usize;
        let mut first_entry = 0usize;
        loop {
            let mut frame = LinkStatusCommand {
                first_frame: first_entry == 0,
                last_frame: true,
                entries: heapless::Vec::new(),
            };
            for entry in entries
                .iter()
                .skip(first_entry)
                .take(LINK_STATUS_ENTRIES_PER_FRAME)
            {
                let _ = frame.entries.push(*entry);
            }
            // The next fragment repeats this fragment's last entry, so a
            // fragment is final only when it reached the end of the list.
            let next_entry = first_entry + frame.entries.len();
            frame.last_frame = next_entry >= entries.len();

            let Some(len) = frame.serialize(&mut payload) else {
                return Err(NwkStatus::FrameTooLong);
            };
            self.send_nwk_command_with_radius(
                ALL_ROUTERS_AND_COORDINATOR,
                NwkCommandId::LinkStatus,
                &payload[..len],
                LINK_STATUS_RADIUS,
            )
            .await?;
            sent += 1;

            if frame.last_frame {
                break;
            }
            // R22 §3.4.8.3.2 one-entry overlap between consecutive frames. The
            // `max` keeps the walk strictly forward: with a one-entry frame the
            // overlap alone would never advance and the loop would not end.
            first_entry = (next_entry - 1).max(first_entry + 1);
        }

        log::debug!("[NWK] Link Status sent in {} frame(s)", sent);
        Ok(())
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
        self.send_route_record_from(
            dest,
            self.nib.network_address,
            self.nib.ieee_address,
            relay_list,
        )
        .await
    }

    /// Send a Route Record whose NWK source is another local device.
    ///
    /// R22 requires a parent forwarding an end-device child's traffic over a
    /// many-to-one route to originate a Route Record with the child's short
    /// and IEEE addresses and with the parent as the first relay.
    pub(crate) async fn send_route_record_from(
        &mut self,
        dest: ShortAddress,
        source: ShortAddress,
        source_ieee: IeeeAddress,
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

        self.send_nwk_command_from_with_radius(
            dest,
            source,
            Some(source_ieee),
            None,
            NwkCommandId::RouteRecord,
            &payload[..offset],
            10,
            None,
        )
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
        let dst_ieee = self.find_ieee_by_short(dest);

        self.send_nwk_command_from_with_radius(
            dest,
            self.nib.network_address,
            Some(self.nib.ieee_address),
            dst_ieee,
            NwkCommandId::NetworkStatus,
            &payload[..len],
            10,
            None,
        )
        .await
    }

    /// Broadcast a Network Status command reporting an address conflict.
    ///
    /// R22 §3.6.1.9.3: the offending 16-bit address goes in the command's
    /// destination address field and the frame is broadcast to `0xFFFD`, i.e.
    /// every device with `macRxOnWhenIdle = TRUE`. Only a router or the
    /// coordinator originates this broadcast.
    pub(crate) async fn send_address_conflict_status(
        &mut self,
        offending_address: ShortAddress,
    ) -> Result<(), NwkStatus> {
        if !self.can_route() {
            return Err(NwkStatus::InvalidRequest);
        }
        let ns = NetworkStatusCommand {
            status_code: NetworkStatusCommand::ADDRESS_CONFLICT,
            destination: offending_address,
        };
        let mut payload = [0u8; 4];
        let len = ns.serialize(&mut payload);

        log::warn!(
            "[NWK] Broadcasting address conflict for 0x{:04X}",
            offending_address.0,
        );
        self.send_nwk_command(
            ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            NwkCommandId::NetworkStatus,
            &payload[..len],
        )
        .await
    }

    /// Unicast a Network Report of type PAN identifier conflict to the network
    /// manager (R22 §3.4.9, §3.6.1.13.1).
    ///
    /// `observed` lists the 16-bit PAN identifiers seen in this device's
    /// neighborhood; the manager picks a replacement that appears in neither
    /// that list nor its own neighborhood. The report carries this network's
    /// EPID and, when known, the manager's IEEE address in the NWK header so a
    /// device that has taken over the manager address cannot absorb it.
    pub async fn send_pan_id_conflict_report(
        &mut self,
        observed: &[PanId],
    ) -> Result<(), NwkStatus> {
        if !self.can_route() {
            return Err(NwkStatus::InvalidRequest);
        }
        let manager = self.nib.nwk_manager_addr;
        if !is_unicast_address(manager) || manager == self.nib.network_address {
            // Nothing to report to: either the manager address is unusable or
            // this device *is* the manager and resolves the conflict itself.
            return Err(NwkStatus::InvalidRequest);
        }

        let mut report = PanIdConflictReport {
            epid: self.nib.extended_pan_id,
            pan_ids: heapless::Vec::new(),
        };
        for pan_id in observed {
            if report.pan_ids.contains(pan_id) {
                continue;
            }
            if report.pan_ids.push(*pan_id).is_err() {
                break;
            }
        }
        let mut payload = [0u8; 9 + 2 * crate::frames::MAX_PAN_ID_CONFLICT_REPORT];
        let Some(len) = report.serialize(&mut payload) else {
            return Err(NwkStatus::FrameTooLong);
        };
        let manager_ieee = self.find_ieee_by_short(manager);

        log::info!(
            "[NWK] Reporting PAN ID conflict ({} PAN IDs) to manager 0x{:04X}",
            report.pan_ids.len(),
            manager.0,
        );
        self.send_nwk_command_from_with_radius(
            manager,
            self.nib.network_address,
            Some(self.nib.ieee_address),
            manager_ieee,
            NwkCommandId::NetworkReport,
            &payload[..len],
            10,
            None,
        )
        .await
    }

    /// Broadcast a Network Update of type PAN identifier update
    /// (R22 §3.4.10, §3.6.1.13.2).
    ///
    /// Sent by the network manager only. The NWK destination is the `0xFFFF`
    /// broadcast address so the coordinator and every router receive it, and
    /// the payload carries the manager's freshly incremented `nwkUpdateId`.
    pub(crate) async fn send_pan_id_update(
        &mut self,
        update: PanIdUpdate,
    ) -> Result<(), NwkStatus> {
        if !self.can_route() {
            return Err(NwkStatus::InvalidRequest);
        }
        let mut payload = [0u8; PanIdUpdate::WIRE_SIZE];
        let Some(len) = update.serialize(&mut payload) else {
            return Err(NwkStatus::FrameTooLong);
        };
        log::info!(
            "[NWK] Broadcasting PAN ID update to 0x{:04X} (update_id={})",
            update.new_pan_id.0,
            update.update_id,
        );
        self.send_nwk_command(
            ShortAddress::BROADCAST,
            NwkCommandId::NetworkUpdate,
            &payload[..len],
        )
        .await
    }

    /// Send a Network Status command through an explicit MAC next hop.
    pub(crate) async fn send_network_status_via(
        &mut self,
        dest: ShortAddress,
        status_code: u8,
        failed_destination: ShortAddress,
        next_hop: ShortAddress,
    ) -> Result<(), NwkStatus> {
        let ns = NetworkStatusCommand {
            status_code,
            destination: failed_destination,
        };
        let mut payload = [0u8; 4];
        let len = ns.serialize(&mut payload);
        let dst_ieee = self.find_ieee_by_short(dest);

        self.send_nwk_command_from_with_radius(
            dest,
            self.nib.network_address,
            Some(self.nib.ieee_address),
            dst_ieee,
            NwkCommandId::NetworkStatus,
            &payload[..len],
            10,
            Some(next_hop),
        )
        .await
    }

    /// Send a Many-to-One Route Request (RREQ) broadcast.
    ///
    /// Used by concentrators (coordinators) to establish reverse routes
    /// from all routers back to the concentrator.
    pub async fn send_many_to_one_rreq(&mut self) -> Result<(), NwkStatus> {
        self.concentrator_rreq_due = false;
        self.concentrator_counter = 0;
        let rreq_id = self.nib.next_route_request_id();
        let rreq = RouteRequest {
            command_options: self.concentrator_type.rreq_options(),
            route_request_id: rreq_id,
            dst_addr: ALL_ROUTERS_AND_COORDINATOR,
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
            ALL_ROUTERS_AND_COORDINATOR,
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

        // R22 delays the original relay by 2*R[2,128] ms and retransmits it
        // `nwkcRREQRetries` (2) times at the 254 ms retry interval. Reuse the
        // same NWK frame for all three MAC broadcasts: they are retries of one
        // discovery, not new NWK/security transactions.
        let sample = routing_random_sample(
            self.mac.monotonic_micros()
                ^ (u32::from(self.nib.network_address.0) << 16)
                ^ u32::from(pending.originator.0)
                ^ (u32::from(pending.route_request_id) << 8)
                ^ (u32::from(pending.path_cost) << 24)
                ^ u32::from(pending.seq_number),
        );
        let jitter_span = MAX_RREQ_JITTER_MS - MIN_RREQ_JITTER_MS + 1;
        let jitter_ms = 2 * (MIN_RREQ_JITTER_MS + sample % jitter_span);
        self.mac.delay_micros(jitter_ms * 1_000).await;

        let mut sent = false;
        let msdu_handle = self.nib.next_seq();
        for attempt in 0..=RREQ_RELAY_RETRIES {
            if attempt != 0 {
                self.mac.delay_micros(RREQ_RETRY_INTERVAL_US).await;
            }
            let result = self
                .mac
                .mcps_data(McpsDataRequest {
                    src_addr_mode: AddressMode::Short,
                    dst_address: MacAddress::Short(self.nib.pan_id, ShortAddress::BROADCAST),
                    payload: &buf[..total_len],
                    // MAC transaction handle only — the NWK sequence number in
                    // the header above stays the originator's.
                    msdu_handle,
                    tx_options: TxOptions {
                        ack_tx: false,
                        ..Default::default()
                    },
                })
                .await;
            sent |= result.is_ok();
        }

        if sent {
            Ok(())
        } else {
            Err(NwkStatus::RouteError)
        }
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
            let result = match pending.next_hop {
                Some(next_hop) => {
                    self.send_network_status_via(
                        pending.destination,
                        pending.status_code,
                        pending.failed_destination,
                        next_hop,
                    )
                    .await
                }
                None => {
                    self.send_network_status(
                        pending.destination,
                        pending.status_code,
                        pending.failed_destination,
                    )
                    .await
                }
            };
            if let Err(e) = result {
                log::warn!(
                    "[NWK] Failed to send NetworkStatus to 0x{:04X}: {:?}",
                    pending.destination.0,
                    e
                );
            }
        }

        // Send concentrator many-to-one RREQ if due
        if self.concentrator_rreq_due && self.joined {
            self.concentrator_rreq_due = false;
            if let Err(e) = self.send_many_to_one_rreq().await {
                log::warn!("[NWK] Failed to send concentrator RREQ: {:?}", e);
            }
        }

        // R22 §3.6.1.9.3 address-conflict broadcasts whose jitter has elapsed,
        // and §3.6.1.13.2/§3.6.1.13.3 PAN identifier switches whose
        // nwkNetworkBroadcastDeliveryTime timer has expired.
        self.process_pending_conflicts().await;
    }

    /// Transmit address-conflict broadcasts and apply PAN identifier updates
    /// whose deadlines have passed.
    ///
    /// Both are deferred by design: R22 §3.6.1.9.3 requires a random jitter
    /// before an address-conflict broadcast (cancelled if an identical one is
    /// heard first), and §3.6.1.13.3 requires the new PAN identifier to be
    /// adopted only after `nwkNetworkBroadcastDeliveryTime`, so that the
    /// broadcast that announced it can still reach the whole network on the
    /// old PAN identifier.
    async fn process_pending_conflicts(&mut self) {
        if !self.joined {
            return;
        }
        let now = self.mac.monotonic_micros();

        // The network manager owes the network a Network Update after it
        // accepted a PAN identifier conflict report (R22 §3.6.1.13.2).
        if let Some(update) = self.pending_pan_id_broadcast.take()
            && let Err(e) = self.send_pan_id_update(update).await
        {
            log::warn!("[NWK] Failed to broadcast PAN ID update: {:?}", e);
        }

        while let Some(index) = self
            .pending_conflicts
            .iter()
            .position(|pending| now.wrapping_sub(pending.send_after_us) < u32::MAX / 2)
        {
            let pending = self.pending_conflicts.swap_remove(index);
            if let Err(e) = self.send_address_conflict_status(pending.address).await {
                log::warn!(
                    "[NWK] Address conflict broadcast for 0x{:04X} failed: {:?}",
                    pending.address.0,
                    e,
                );
            }
        }

        if let Some(pending) = self.pending_pan_id_update
            && now.wrapping_sub(pending.apply_at_us) < u32::MAX / 2
        {
            self.pending_pan_id_update = None;
            if let Err(e) = self.apply_pan_id(pending.new_pan_id).await {
                log::error!(
                    "[NWK] PAN ID switch to 0x{:04X} failed: {:?}",
                    pending.new_pan_id.0,
                    e,
                );
            }
        }
    }

    /// Adopt a new short PAN identifier at the MAC and in the NIB.
    ///
    /// R22 §3.6.1.13.2 has the coordinator reissue `MLME-START.request` with
    /// the new PAN identifier; a router only has to retune the MAC PIB, since
    /// it is not the beacon-issuing PAN coordinator.
    async fn apply_pan_id(&mut self, new_pan_id: PanId) -> Result<(), NwkStatus> {
        if self.device_type == crate::DeviceType::Coordinator {
            self.mac
                .mlme_start(zigbee_mac::MlmeStartRequest {
                    pan_id: new_pan_id,
                    channel: self.nib.logical_channel,
                    beacon_order: 15,
                    superframe_order: 15,
                    pan_coordinator: true,
                    battery_life_ext: false,
                })
                .await
                .map_err(|_| NwkStatus::StartupFailure)?;
        } else {
            self.mac
                .mlme_set(
                    zigbee_mac::PibAttribute::MacPanId,
                    zigbee_mac::PibValue::PanId(new_pan_id),
                )
                .await
                .map_err(|_| NwkStatus::InvalidRequest)?;
        }
        log::warn!(
            "[NWK] PAN ID changed 0x{:04X} -> 0x{:04X}",
            self.nib.pan_id.0,
            new_pan_id.0,
        );
        self.nib.pan_id = new_pan_id;
        Ok(())
    }

    /// Send an End Device Timeout Request (0x0B) to the current parent.
    ///
    /// The requested enumeration comes from
    /// [`Nib::requested_end_device_timeout`](crate::nib::Nib::requested_end_device_timeout):
    /// a fresh join or secured rejoin asks for the long enumeration 14
    /// (≈ 11 days) and a parent that answers `INCORRECT_VALUE` walks it down
    /// towards the default enumeration 8.
    ///
    /// Only end devices negotiate a timeout for themselves; a router or
    /// coordinator returns `Ok(())` without transmitting.
    pub async fn send_ed_timeout_request(&mut self) -> Result<(), NwkStatus> {
        if self.device_type != crate::DeviceType::EndDevice {
            return Ok(()); // Only end devices send this
        }
        let requested = self.nib.requested_end_device_timeout;
        // An out-of-range NIB value would otherwise put an undefined
        // enumeration on the wire; refuse instead of clamping so the caller's
        // retry/fallback path runs.
        let Some(req) = EdTimeoutRequest::new(requested) else {
            log::error!("[NWK] Refusing undefined ED Timeout enum {}", requested);
            return Err(NwkStatus::InvalidRequest);
        };
        let mut payload = [0u8; 2];
        let len = req.serialize(&mut payload);
        log::info!(
            "[NWK] Sending ED Timeout Request (enum={}) to parent 0x{:04X}",
            req.requested_timeout,
            self.nib.parent_address.0
        );
        let result = self
            .send_nwk_command(
                self.nib.parent_address,
                NwkCommandId::EdTimeoutRequest,
                &payload[..len],
            )
            .await;
        if result.is_ok() {
            self.nib.mark_end_device_timeout_response_pending();
        }
        result
    }

    /// Stop accepting a response for the current End Device Timeout Request.
    ///
    /// Used when the runtime exhausts its bounded response retries. A late
    /// response after that point belongs to an abandoned negotiation round and
    /// must not change the selected timeout or keepalive method.
    pub fn cancel_ed_timeout_response_wait(&mut self) {
        self.nib.clear_end_device_timeout_response_pending();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DeviceType;
    use crate::frames::{LinkStatusReport, NwkHeader};
    use crate::neighbor::{NeighborDeviceType, NeighborEntry, Relationship};
    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use std::sync::Arc;
    use std::task::Wake;
    use zigbee_mac::mock::MockMac;

    const OUR_ADDR: ShortAddress = ShortAddress(0x1111);
    const PAN: PanId = PanId(0x1234);

    struct NoopWake;
    impl Wake for NoopWake {
        fn wake(self: Arc<Self>) {}
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(NoopWake));
        let mut context = Context::from_waker(&waker);
        let mut future = std::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    fn node(device_type: DeviceType) -> NwkLayer<MockMac> {
        let mut nwk = NwkLayer::new(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]), device_type);
        nwk.set_joined(true);
        nwk.nib_mut().network_address = OUR_ADDR;
        nwk.nib_mut().pan_id = PAN;
        nwk.nib_mut().security_enabled = false;
        nwk
    }

    /// Add a neighbour of the given type with a known incoming cost and an
    /// outgoing cost as if a Link Status had already reported it.
    fn neighbour(
        nwk: &mut NwkLayer<MockMac>,
        address: ShortAddress,
        device_type: NeighborDeviceType,
        incoming_cost: u8,
        outgoing_cost: u8,
    ) {
        let mut entry = NeighborEntry::new_from_annce(address, [address.0 as u8; 8]);
        entry.device_type = device_type;
        entry.relationship = Relationship::Sibling;
        entry.incoming_cost = incoming_cost;
        entry.outgoing_cost = outgoing_cost;
        nwk.neighbors
            .add_or_update(entry)
            .expect("the neighbour table has room");
    }

    /// The Link Status payloads a device put on air, in transmission order.
    fn link_status_frames(nwk: &NwkLayer<MockMac>) -> std::vec::Vec<std::vec::Vec<u8>> {
        nwk.mac()
            .tx_history()
            .iter()
            .filter_map(|record| {
                let bytes = record.payload.as_slice();
                let (header, consumed) = NwkHeader::parse(bytes)?;
                if bytes.get(consumed).copied()? != NwkCommandId::LinkStatus as u8 {
                    return None;
                }
                assert_eq!(
                    header.dst_addr, ALL_ROUTERS_AND_COORDINATOR,
                    "R22 §3.4.8.2: the destination is the router-only broadcast address",
                );
                assert_eq!(header.radius, 1, "R22 §3.4.8.2: the radius is 1");
                assert!(
                    header.src_ieee.is_some(),
                    "R22 §3.4.8.2: the source IEEE address is present",
                );
                Some(bytes[consumed + 1..].to_vec())
            })
            .collect()
    }

    #[test]
    fn link_status_lists_only_router_neighbours_sorted_by_address() {
        let mut router = node(DeviceType::Router);
        neighbour(
            &mut router,
            ShortAddress(0x4444),
            NeighborDeviceType::Router,
            3,
            2,
        );
        neighbour(
            &mut router,
            ShortAddress(0x2222),
            NeighborDeviceType::Coordinator,
            1,
            0,
        );
        // An end device neither routes nor exchanges link status frames.
        neighbour(
            &mut router,
            ShortAddress(0x3333),
            NeighborDeviceType::EndDevice,
            1,
            1,
        );
        // A device only known from a Device_annce is not a proven neighbour.
        neighbour(
            &mut router,
            ShortAddress(0x5555),
            NeighborDeviceType::Unknown,
            1,
            1,
        );

        block_on(router.send_link_status()).expect("the link status is sent");
        let frames = link_status_frames(&router);
        assert_eq!(frames.len(), 1, "three entries fit in one frame");

        let report = LinkStatusReport::parse(&frames[0]).expect("the frame parses");
        assert!(
            report.first_frame() && report.last_frame(),
            "R22 §3.4.8.3.1: a single-frame list sets both flags",
        );
        let entries: std::vec::Vec<_> = report.iter().collect();
        assert_eq!(entries.len(), 2);
        assert_eq!(entries[0].address, ShortAddress(0x2222));
        assert_eq!(entries[1].address, ShortAddress(0x4444));
        // R22 §3.4.8.3.2: our own estimate outbound, the neighbour table's
        // outgoing cost verbatim — including 0 for "never heard from".
        assert_eq!(entries[0].incoming_cost, 1);
        assert_eq!(entries[0].outgoing_cost, 0);
        assert_eq!(entries[1].incoming_cost, 3);
        assert_eq!(entries[1].outgoing_cost, 2);
    }

    #[test]
    #[cfg(feature = "router")]
    fn link_status_fragments_overlap_by_one_entry() {
        let mut router = node(DeviceType::Router);
        let count = LINK_STATUS_ENTRIES_PER_FRAME + 3;
        for index in 0..count {
            neighbour(
                &mut router,
                ShortAddress(0x0100 + index as u16),
                NeighborDeviceType::Router,
                2,
                3,
            );
        }

        block_on(router.send_link_status()).expect("the link status is sent");
        let frames = link_status_frames(&router);
        assert_eq!(frames.len(), 2, "17+ entries need two frames");

        let first = LinkStatusReport::parse(&frames[0]).expect("the first frame parses");
        let second = LinkStatusReport::parse(&frames[1]).expect("the second frame parses");
        assert!(first.first_frame() && !first.last_frame());
        assert!(!second.first_frame() && second.last_frame());
        assert_eq!(first.len(), LINK_STATUS_ENTRIES_PER_FRAME);

        let first_entries: std::vec::Vec<_> = first.iter().collect();
        let second_entries: std::vec::Vec<_> = second.iter().collect();
        assert_eq!(
            first_entries.last().unwrap().address,
            second_entries[0].address,
            "R22 §3.4.8.3.2: consecutive frames share one address",
        );
        // Every neighbour is covered exactly once across the pair, and the two
        // ranges join without a gap.
        assert_eq!(
            first_entries.len() + second_entries.len() - 1,
            count,
            "every router neighbour is listed",
        );
        assert!(first.covers(ShortAddress(0x0100)));
        assert!(second.covers(ShortAddress(0x0100 + count as u16 - 1)));
    }

    #[test]
    fn an_end_device_never_sends_link_status() {
        // R22 §3.6.3.4.1: "End devices do not send link status command frames."
        let mut end_device = node(DeviceType::EndDevice);
        neighbour(
            &mut end_device,
            ShortAddress(0x0000),
            NeighborDeviceType::Coordinator,
            1,
            1,
        );
        block_on(end_device.send_link_status()).expect("the call is a no-op");
        assert!(end_device.mac().tx_history().is_empty());
    }
}
