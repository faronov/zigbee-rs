//! NLDE — NWK Layer Data Entity.
//!
//! Handles sending and receiving NWK data frames via the MAC layer.
//! - NLDE-DATA.request: send NWK data to a destination
//! - NLDE-DATA.indication: receive NWK data from the network
//! - Frame relay for routers/coordinators

use crate::frames::{NwkCommandId, NwkFrameControl, NwkFrameType, NwkHeader};
use crate::{DeviceType, NwkLayer, NwkStatus, RejoinResponseDelivery};
use zigbee_mac::{AddressMode, MacDriver, McpsDataRequest, TxOptions};
use zigbee_types::*;

/// NWK data indication — received NWK-level data.
#[derive(Debug)]
pub struct NldeDataIndication<'a> {
    pub dst_addr: ShortAddress,
    pub src_addr: ShortAddress,
    pub payload: &'a [u8],
    pub lqi: u8,
    pub security_use: bool,
    pub security_source: Option<IeeeAddress>,
}

/// Owned NWK data indication — for decrypted frames where payload is owned.
#[derive(Debug)]
pub struct NldeDataIndicationOwned {
    pub dst_addr: ShortAddress,
    pub src_addr: ShortAddress,
    pub payload: heapless::Vec<u8, 128>,
    pub lqi: u8,
    pub security_use: bool,
    pub security_source: Option<IeeeAddress>,
}

/// Result of processing an incoming NWK frame.
///
/// NWK commands never appear here: they are handled inside the NWK layer, and
/// the few that additionally change this device's network lifecycle are
/// reported out of band through
/// [`NwkLayer::take_command_outcome`](crate::NwkLayer::take_command_outcome).
#[derive(Debug)]
pub enum NwkIndication<'a> {
    /// Unsecured frame — payload borrows from MAC buffer
    Borrowed(NldeDataIndication<'a>),
    /// Decrypted frame — payload is owned
    Owned(NldeDataIndicationOwned),
}

/// NWK command outcomes that the layer above must act on.
///
/// The NWK layer applies the network-level effect itself (clearing the joined
/// flag, dropping the neighbour); the caller owns the policy that follows,
/// such as scheduling a secured rejoin or clearing persisted credentials.
///
/// Retrieved with
/// [`NwkLayer::take_command_outcome`](crate::NwkLayer::take_command_outcome)
/// immediately after `process_incoming_nwk_frame`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NwkCommandOutcome {
    /// The parent sent a Leave *request* addressed to this device.
    LeaveRequested {
        /// Sender of the Leave command (always our parent — enforced on RX).
        src: ShortAddress,
        /// Rejoin bit: the device is expected to rejoin the same network.
        rejoin: bool,
        /// Remove-children bit from the Leave command.
        remove_children: bool,
    },
    /// Our parent announced that it is leaving the network, so the current
    /// parent relationship is gone and connectivity must be re-established.
    ParentLeft {
        /// Address of the parent that announced the leave.
        src: ShortAddress,
    },
    /// A locally addressed Rejoin Request needs an asynchronous response and
    /// Trust Center notification from the runtime.
    ChildRejoinRequest {
        src: ShortAddress,
        ieee: IeeeAddress,
        capability_info: u8,
        secured: bool,
    },
    /// A directly-attached authenticated child sent an End Device Timeout
    /// Request (0x0B). The runtime must apply the timeout policy and transmit
    /// the End Device Timeout Response (0x0C), which may be delivered
    /// indirectly to a sleepy child. The NWK layer has already checked that
    /// `src`/`ieee` name an authenticated child of this parent and that the
    /// frame was addressed to and authenticated for this device.
    EndDeviceTimeoutRequest {
        /// Short address of the requesting child (validated as our child).
        src: ShortAddress,
        /// Authenticated IEEE identity of the requesting child.
        ieee: IeeeAddress,
        /// Requested Timeout Enumeration (already parsed to 0..=14).
        requested_timeout: u8,
    },
}

/// NWK data confirm — result of NLDE-DATA.request.
#[derive(Debug)]
pub struct NldeDataConfirm {
    pub status: NwkStatus,
    pub nsdu_handle: u8,
}

/// Largest serialized NWK frame this stack builds, relays or accepts.
///
/// Bounds every heap-free frame buffer on the transmit and relay paths.
pub(crate) const MAX_NWK_FRAME: usize = 128;

/// Largest CCM* additional authenticated data block (NWK header || aux header).
///
/// A NWK header is at most 8 fixed bytes + two IEEE addresses + the multicast
/// control byte + a 16-relay source-route subframe (59 bytes), followed by the
/// 14-byte security auxiliary header. Sizing this buffer any smaller would
/// silently truncate the AAD of a source-routed frame and fail every MIC.
const MAX_NWK_AAD: usize = 80;

/// NWK payload of a received frame, after authentication.
///
/// Secured frames own their decrypted plaintext; unsecured frames borrow the
/// MAC buffer. Both are handed to the relay path and to local delivery as a
/// plain slice, so a secured frame is authenticated exactly once no matter how
/// many of those consumers run.
enum NwkPayload<'a> {
    Plain(&'a [u8]),
    Decrypted {
        payload: heapless::Vec<u8, MAX_NWK_FRAME>,
        security_source: IeeeAddress,
    },
}

impl NwkPayload<'_> {
    fn as_slice(&self) -> &[u8] {
        match self {
            Self::Plain(payload) => payload,
            Self::Decrypted { payload, .. } => payload.as_slice(),
        }
    }

    fn security_source(&self) -> Option<IeeeAddress> {
        match self {
            Self::Plain(_) => None,
            Self::Decrypted {
                security_source, ..
            } => Some(*security_source),
        }
    }
}

/// Whether `addr` is one of the NWK broadcast addresses this stack supports.
///
/// Zigbee PRO R22 3.6.5 defines exactly four broadcast destinations:
///
/// - `0xFFFF` — all devices,
/// - `0xFFFD` — all devices with the receiver on when idle,
/// - `0xFFFC` — all routers and the coordinator,
/// - `0xFFFB` — low-power routers.
///
/// `0xFFF8`–`0xFFFA` are reserved and `0xFFFE` is unassigned. Classifying the
/// whole `0xFFF8..=0xFFFF` range as broadcast — as this stack used to — hands
/// those undefined destinations to the upper layer, records them in the
/// broadcast transaction record (where they suppress a genuine broadcast that
/// shares a source and sequence number) and re-emits them as MAC broadcasts on
/// this device's behalf. Every broadcast decision therefore goes through this
/// one predicate.
pub(crate) const fn is_nwk_broadcast(addr: ShortAddress) -> bool {
    matches!(addr.0, 0xFFFF | 0xFFFD | 0xFFFC | 0xFFFB)
}

/// Whether `addr` can name an individual device.
///
/// `0x0000..=0xFFF7` is the unicast address space. Everything above it is
/// either a defined broadcast address or reserved, so it can never be a route
/// destination, a route discovery target or a previous hop.
pub(crate) const fn is_unicast_address(addr: ShortAddress) -> bool {
    addr.0 < 0xFFF8
}

impl<M: MacDriver> NwkLayer<M> {
    /// Serialize `header` and `payload` into `buf`, applying NWK security when
    /// `header.frame_control.security` is set. Returns the frame length.
    ///
    /// This is the single frame builder behind the data service, the NWK
    /// command senders and the relay path. Every secured frame therefore gets:
    ///
    /// - the caller's header as CCM* additional authenticated data, so any
    ///   mutation (radius, source-route relay index) must already be applied —
    ///   security is computed over the header that actually goes on air;
    /// - an auxiliary header carrying *this* device's IEEE address and active
    ///   key sequence number, matching the hop-by-hop NWK security model;
    /// - a frame counter taken from the durable outgoing reservation, never a
    ///   counter reused from another device or another frame;
    /// - zeroed over-the-air security-level bits (spec §4.3.1.2), while CCM*
    ///   itself uses the real ENC-MIC-32 level.
    ///
    /// The key is resolved before a counter is drawn so a frame that cannot be
    /// encrypted never burns durably reserved counter space.
    pub(crate) fn build_nwk_frame(
        &mut self,
        header: &NwkHeader,
        payload: &[u8],
        buf: &mut [u8; MAX_NWK_FRAME],
    ) -> Result<usize, NwkStatus> {
        let hdr_len = header.serialize(buf);

        if !header.frame_control.security {
            let end = hdr_len + payload.len();
            if end > buf.len() {
                return Err(NwkStatus::FrameTooLong);
            }
            buf[hdr_len..end].copy_from_slice(payload);
            return Ok(end);
        }

        let Some(key) = self.security.active_key().map(|entry| entry.key) else {
            log::warn!("[NWK] No active network key for encryption");
            return Err(NwkStatus::NoKey);
        };
        let sec_hdr = crate::security::NwkSecurityHeader {
            security_control: crate::security::NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: self
                .nib
                .next_frame_counter()
                .ok_or(NwkStatus::MaxFrmCounterReached)?,
            source_address: self.nib.ieee_address,
            key_seq_number: self.nib.active_key_seq_number,
        };
        if hdr_len + crate::security::NWK_AUX_HEADER_LEN > buf.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        let aad_len = hdr_len + sec_hdr.serialize(&mut buf[hdr_len..]);

        let Some(encrypted) =
            self.security
                .encrypt_with(&mut self.mac, &buf[..aad_len], payload, &key, &sec_hdr)
        else {
            log::warn!("[NWK] Encryption failed");
            return Err(NwkStatus::BadCcmOutput);
        };
        let end = aad_len + encrypted.len();
        if end > buf.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        buf[aad_len..end].copy_from_slice(&encrypted);
        // Zigbee transmits zero in the over-the-air security-level bits.
        buf[hdr_len] &= !0x07;

        log::debug!(
            "[NWK TX] sec: fc={} key_seq={} ieee={:02X?}",
            sec_hdr.frame_counter,
            sec_hdr.key_seq_number,
            &sec_hdr.source_address[..4],
        );
        Ok(end)
    }

    /// Send NWK data to a destination address.
    ///
    /// This is the primary data service used by the APS layer above.
    /// It builds a NWK frame, determines the MAC next-hop, and sends via MAC.
    pub async fn nlde_data_request(
        &mut self,
        dst_addr: ShortAddress,
        radius: u8,
        payload: &[u8],
        security_enable: bool,
        discover_route: bool,
    ) -> Result<NldeDataConfirm, NwkStatus> {
        if !self.joined {
            log::warn!(
                "[NWK] nlde_data_request called but not joined! dst=0x{:04X}",
                dst_addr.0
            );
            return Err(NwkStatus::InvalidRequest);
        }
        if self.routing.has_failed_many_to_one(dst_addr) {
            log::warn!(
                "[NWK] Failed many-to-one route to 0x{:04X} awaits a new MTOR discovery",
                dst_addr.0,
            );
            return Err(NwkStatus::RouteError);
        }

        // A concentrator sends over the path a Route Record established, when
        // it has one. Zigbee stores the relay closest to the destination
        // first and the relay closest to this originator last. Therefore the
        // source route starts at `relay_count - 1` and each relay decrements
        // the index as the frame approaches the destination
        // (see [`process_source_route`]).
        //
        // A destination with no relays between us and it is sent to directly:
        // attaching an empty subframe would put a relay count of zero on air
        // and force every receiver to fall back to its routing table anyway.
        let source_route_subframe = if self.concentrator_active {
            self.source_route_table
                .lookup(dst_addr)
                .filter(|relays| !relays.is_empty())
                .map(|relays| {
                    let mut relay_list = heapless::Vec::new();
                    for addr in relays {
                        let _ = relay_list.push(*addr);
                    }
                    crate::frames::SourceRoute {
                        relay_count: relay_list.len() as u8,
                        relay_index: relay_list.len().saturating_sub(1) as u8,
                        relay_list,
                    }
                })
        } else {
            None
        };
        let has_source_route = source_route_subframe.is_some();

        // Resolve the MAC next hop before allocating a sequence number or a
        // durably reserved security frame counter: a frame that cannot be
        // routed must not consume counter space that can never be reused.
        //
        // A source-routed frame goes to the relay named by its initial index;
        // only a frame without one consults the routing table, and only that
        // frame can therefore be unroutable.
        let next_hop = match source_route_subframe
            .as_ref()
            .and_then(|sr| sr.relay_list.get(sr.relay_index as usize).copied())
        {
            Some(first_relay) => first_relay,
            None => match self.resolve_next_hop(dst_addr) {
                Ok(hop) => hop,
                Err(status) => {
                    return self
                        .handle_unroutable_destination(dst_addr, discover_route, status)
                        .await;
                }
            },
        };

        // Auto Route Record: if the destination route requires one, transmit
        // it before reserving the data frame's security counter. Receivers
        // commit replay counters in transmission order, so building data
        // first would assign it N, send the Route Record as N+1, then have the
        // next hop reject the later data frame N as a replay.
        if is_unicast_address(dst_addr) {
            let needs_rr = self
                .routing
                .get_entry(dst_addr)
                .map(|e| e.route_record_required)
                .unwrap_or(false);
            if needs_rr {
                log::debug!(
                    "[NWK] Sending Route Record to concentrator 0x{:04X}",
                    dst_addr.0
                );
                if let Err(status) = self.send_route_record(dst_addr, &[]).await {
                    self.routing.remove(dst_addr);
                    return Err(status);
                }
                self.routing.clear_route_record_required(dst_addr);
            }
        }
        let seq = self.nib.next_seq();

        // Build NWK header
        // Note: multicast flag is ONLY for group-addressed frames (via APS group delivery).
        // The defined broadcast addresses must NOT set the multicast flag.
        // End devices suppress route discovery (parent handles routing).

        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Data as u8,
                protocol_version: 0x02,
                discover_route: if discover_route && self.device_type != DeviceType::EndDevice {
                    1
                } else {
                    0
                },
                multicast: false,
                security: security_enable && self.nib.security_enabled,
                source_route: has_source_route,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: false, // Maximise compatibility with older stacks
            },
            dst_addr,
            src_addr: self.nib.network_address,
            radius,
            seq_number: seq,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: source_route_subframe,
        };

        // Serialize the frame, applying NWK security when the header asks for
        // it. The shared builder owns key selection, the durable frame-counter
        // reservation and CCM* so this path cannot drift from the command and
        // relay paths.
        let mut nwk_buf = [0u8; MAX_NWK_FRAME];
        let total_len = self.build_nwk_frame(&header, payload, &mut nwk_buf)?;

        // Locally originated traffic uses the same bounded indirect queue as
        // relayed traffic when its final destination is our sleepy child.
        if !has_source_route && next_hop == dst_addr && self.is_sleepy_child(next_hop) {
            self.enqueue_indirect_for_child(next_hop, &nwk_buf[..total_len])?;
            return Ok(NldeDataConfirm {
                status: NwkStatus::Success,
                nsdu_handle: seq,
            });
        }

        log::info!(
            "[NWK TX] dst=0x{:04X} next_hop=0x{:04X} sec={} len={} hdr={:02X?}",
            dst_addr.0,
            next_hop.0,
            security_enable && self.nib.security_enabled,
            total_len,
            &nwk_buf[..core::cmp::min(8, total_len)],
        );
        // Send via MAC
        let mac_result = self
            .mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &nwk_buf[..total_len],
                msdu_handle: seq,
                tx_options: TxOptions {
                    // Fix 9: No MAC ACK for broadcast
                    ack_tx: next_hop != ShortAddress::BROADCAST,
                    ..Default::default()
                },
            })
            .await;

        if let Err(ref e) = mac_result {
            log::warn!("[NWK TX] MAC send failed: {:?}", e);
            if has_source_route {
                self.source_route_table.remove(dst_addr);
                self.routing.remove(dst_addr);
            } else if self
                .routing
                .get_entry(dst_addr)
                .is_some_and(|route| route.many_to_one)
            {
                self.routing.remove(dst_addr);
            }
        }

        mac_result.map_err(|_| NwkStatus::RouteError)?;
        Ok(NldeDataConfirm {
            status: NwkStatus::Success,
            nsdu_handle: seq,
        })
    }

    /// Send a one-hop Rejoin Response using exactly the security state of the
    /// corresponding request.
    ///
    /// `request_address` remains the MAC/NWK destination even when the
    /// response assigns a different short address. Sleepy requesters poll
    /// using that old address until they receive the response.
    pub async fn send_rejoin_response(
        &mut self,
        request_address: ShortAddress,
        child_ieee: IeeeAddress,
        assigned_address: ShortAddress,
        status: u8,
        secured: bool,
        rx_on_when_idle: bool,
    ) -> Result<RejoinResponseDelivery, NwkStatus> {
        if !self.can_route() || !is_unicast_address(request_address) {
            return Err(NwkStatus::InvalidRequest);
        }

        let sequence = self.nib.next_seq();
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: secured,
                source_route: false,
                dst_ieee_present: true,
                src_ieee_present: false,
                end_device_initiator: false,
            },
            dst_addr: request_address,
            src_addr: self.nib.network_address,
            radius: 1,
            seq_number: sequence,
            dst_ieee: Some(child_ieee),
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };
        let command = [
            NwkCommandId::RejoinResponse as u8,
            assigned_address.0 as u8,
            (assigned_address.0 >> 8) as u8,
            status,
        ];
        let mut frame = [0u8; MAX_NWK_FRAME];
        let frame_len = self.build_nwk_frame(&header, &command, &mut frame)?;

        let request_address_conflicts = self
            .neighbors
            .find_by_short(request_address)
            .is_some_and(|entry| entry.ieee_address != child_ieee);
        if !rx_on_when_idle && status == 0x00 && !request_address_conflicts {
            // A rejoining child may still poll with its previous address.
            // Discard stale transactions and key this response by that
            // address rather than by the newly assigned one in the payload.
            self.indirect.remove_all(request_address);
            let Some(slot) = self
                .indirect
                .enqueue_with_slot(request_address, &frame[..frame_len])
            else {
                return Err(NwkStatus::FrameNotBuffered);
            };
            if self
                .mac
                .set_indirect_data_pending(
                    MacAddress::Short(self.nib.pan_id, request_address),
                    true,
                )
                .is_err()
            {
                self.indirect.remove_slot(slot);
                return Err(NwkStatus::FrameNotBuffered);
            }
            return Ok(RejoinResponseDelivery::Indirect);
        }

        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, request_address),
                payload: &frame[..frame_len],
                msdu_handle: sequence,
                tx_options: TxOptions {
                    ack_tx: true,
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::RouteError)?;
        Ok(RejoinResponseDelivery::Direct)
    }

    /// Apply the R22 End Device Timeout policy for a validated child request
    /// and transmit the End Device Timeout Response (0x0C).
    ///
    /// Called by the layer above for a
    /// [`NwkCommandOutcome::EndDeviceTimeoutRequest`]. The identity and
    /// authentication checks were already made by `handle_ed_timeout_request`;
    /// this re-validates that `child`/`child_ieee` still name an authenticated
    /// end-device child (the child table could have changed between receive and
    /// this async step) and then:
    /// - accepts every defined enumeration 0..=14 — this parent imposes no
    ///   tighter bound — recording it as the child's accepted timeout and
    ///   answering `SUCCESS`;
    /// - answers `INCORRECT_VALUE` and falls back to
    ///   [`ED_TIMEOUT_ENUM_DEFAULT`] for any other value (unreachable through
    ///   the strict wire parser, but kept as an explicit deterministic branch);
    /// - refreshes the child's deadline, because the request itself is an End
    ///   Device Timeout Request keepalive;
    /// - advertises both supported keepalive methods in `nwkParentInformation`.
    ///
    /// A sleepy child receives the response indirectly through the parent's
    /// queue on its next MAC Data Request; an rx-on child receives it directly.
    pub async fn respond_to_end_device_timeout_request(
        &mut self,
        child: ShortAddress,
        child_ieee: IeeeAddress,
        requested_timeout: u8,
    ) -> Result<(), NwkStatus> {
        use crate::frames::{
            ED_TIMEOUT_ENUM_DEFAULT, ED_TIMEOUT_STATUS_INCORRECT_VALUE, ED_TIMEOUT_STATUS_SUCCESS,
            PARENT_INFO_MASK, ed_timeout_enum_to_seconds,
        };

        if !self.can_route() {
            return Err(NwkStatus::InvalidRequest);
        }
        let rx_on_when_idle = match self.neighbors.find_by_short(child) {
            Some(entry)
                if entry.relationship == crate::neighbor::Relationship::Child
                    && entry.ieee_address == child_ieee =>
            {
                entry.rx_on_when_idle
            }
            _ => return Err(NwkStatus::UnknownDevice),
        };

        // Explicit supported-enumeration policy: accept all R22 values.
        let (status, accepted) = match ed_timeout_enum_to_seconds(requested_timeout) {
            Some(_) => (ED_TIMEOUT_STATUS_SUCCESS, requested_timeout),
            None => (ED_TIMEOUT_STATUS_INCORRECT_VALUE, ED_TIMEOUT_ENUM_DEFAULT),
        };

        if let Some(entry) = self.neighbors.find_by_short_mut(child) {
            entry.end_device_timeout = accepted;
            // The request is itself a keepalive, so re-arm the deadline from
            // the (possibly new) accepted enumeration.
            entry.refresh_end_device_timeout();
            entry.age = 0;
            entry.keepalive_confirmed = true;
        }

        // This parent implements both keepalive methods.
        let parent_info = PARENT_INFO_MASK;
        log::info!(
            "[NWK] ED Timeout {} for child 0x{:04X}: enum={} parent_info=0x{:02X}",
            if status == ED_TIMEOUT_STATUS_SUCCESS {
                "accepted"
            } else {
                "refused"
            },
            child.0,
            accepted,
            parent_info,
        );
        self.send_end_device_timeout_response(
            child,
            child_ieee,
            status,
            parent_info,
            rx_on_when_idle,
        )
        .await
    }

    /// Send a one-hop End Device Timeout Response (0x0C) to a child.
    ///
    /// Mirrors [`Self::send_rejoin_response`] delivery: a sleepy child's
    /// response is queued indirectly and armed with MAC Frame Pending so it is
    /// retrieved on the child's next Data Request without bypassing ACK/Frame
    /// Pending semantics; an rx-on child is sent the response directly.
    async fn send_end_device_timeout_response(
        &mut self,
        child: ShortAddress,
        child_ieee: IeeeAddress,
        status: u8,
        parent_info: u8,
        rx_on_when_idle: bool,
    ) -> Result<(), NwkStatus> {
        if !self.can_route() || !is_unicast_address(child) {
            return Err(NwkStatus::InvalidRequest);
        }

        let sequence = self.nib.next_seq();
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: self.nib.security_enabled,
                source_route: false,
                dst_ieee_present: true,
                src_ieee_present: false,
                end_device_initiator: false,
            },
            dst_addr: child,
            src_addr: self.nib.network_address,
            radius: 1,
            seq_number: sequence,
            dst_ieee: Some(child_ieee),
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };
        let command = [NwkCommandId::EdTimeoutResponse as u8, status, parent_info];
        let mut frame = [0u8; MAX_NWK_FRAME];
        let frame_len = self.build_nwk_frame(&header, &command, &mut frame)?;

        if !rx_on_when_idle {
            // The response is keyed on the child's own short address so its
            // next poll retrieves it, exactly like the Rejoin Response path.
            let Some(slot) = self.indirect.enqueue_with_slot(child, &frame[..frame_len]) else {
                return Err(NwkStatus::FrameNotBuffered);
            };
            if self
                .mac
                .set_indirect_data_pending(MacAddress::Short(self.nib.pan_id, child), true)
                .is_err()
            {
                self.indirect.remove_slot(slot);
                return Err(NwkStatus::FrameNotBuffered);
            }
            return Ok(());
        }

        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, child),
                payload: &frame[..frame_len],
                msdu_handle: sequence,
                tx_options: TxOptions {
                    ack_tx: true,
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::RouteError)?;
        Ok(())
    }

    /// Decide what to do with a unicast that currently has no next hop.
    ///
    /// Routing devices start an AODV route discovery so a retry (APS
    /// retransmission or the next report) can succeed, and report
    /// [`NwkStatus::RouteDiscoveryFailed`] for this attempt — the frame is not
    /// buffered, so the request still fails rather than silently succeeding.
    /// End devices and non-routing builds keep the original error.
    async fn handle_unroutable_destination(
        &mut self,
        dst_addr: ShortAddress,
        discover_route: bool,
        status: NwkStatus,
    ) -> Result<NldeDataConfirm, NwkStatus> {
        if !discover_route || !is_unicast_address(dst_addr) || !self.can_route() {
            log::warn!("[NWK] No route to 0x{:04X}: {:?}", dst_addr.0, status);
            return Err(status);
        }

        if self.routing.has_active_discovery(dst_addr) {
            log::debug!(
                "[NWK] Route discovery already underway for 0x{:04X}",
                dst_addr.0
            );
            return Err(NwkStatus::RouteDiscoveryFailed);
        }

        match self.discover_route(dst_addr).await {
            Ok(request_id) => {
                log::info!(
                    "[NWK] No route to 0x{:04X}; started discovery (id={})",
                    dst_addr.0,
                    request_id
                );
            }
            Err(e) => {
                log::warn!(
                    "[NWK] Route discovery for 0x{:04X} could not be started: {:?}",
                    dst_addr.0,
                    e
                );
            }
        }
        Err(NwkStatus::RouteDiscoveryFailed)
    }

    /// Process incoming MAC data indication as a NWK frame.
    ///
    /// Parses the NWK header, authenticates the frame, and then either:
    /// - Delivers to the upper layer (if destined for us)
    /// - Handles it internally (NWK commands, optionally recording a
    ///   lifecycle outcome for
    ///   [`take_command_outcome`](crate::NwkLayer::take_command_outcome))
    /// - Relays the frame — a unicast, or an NWK *Data* broadcast, when we
    ///   are a joined routing device. NWK command propagation is never
    ///   generic: it is owned by the individual command handlers.
    ///
    /// The pending command outcome is cleared on entry and set only by the
    /// frame being processed, so a stale outcome can never be observed after a
    /// later, unrelated frame.
    ///
    /// A secured frame is decrypted and MIC-checked *before* it is recorded in
    /// the broadcast transaction record, before any routing command takes
    /// effect and before it is relayed, and its incoming frame counter is
    /// committed exactly once. A broadcast that is both rebroadcast and
    /// delivered locally is therefore authenticated a single time.
    ///
    /// Two admission checks run ahead of all of that, on the header alone:
    /// a frame whose type is neither Data nor Command (Inter-PAN, reserved)
    /// is dropped, and on a secured network so is any unsecured NWK command
    /// or unsecured NWK broadcast. Neither can reach the BTR, the relay path
    /// or a command handler. Unsecured unicasts remain accepted so that
    /// pre-key APS commissioning traffic still arrives.
    pub async fn process_incoming_nwk_frame<'a>(
        &mut self,
        mac_payload: &'a [u8],
        lqi: u8,
    ) -> Option<NwkIndication<'a>> {
        self.process_incoming_nwk_frame_from(mac_payload, lqi, None)
            .await
    }

    /// Process an incoming NWK frame, naming the MAC device it arrived from.
    ///
    /// Identical to [`process_incoming_nwk_frame`](Self::process_incoming_nwk_frame)
    /// except that `prev_hop` carries the short address of the *immediate*
    /// transmitter (`McpsDataIndication::src_address`). Routing state is
    /// hop-by-hop: a Route Request keeps the originator in its NWK header all
    /// the way across the mesh, so the only address that may be installed as a
    /// next hop is the neighbour the frame was actually received from. A
    /// caller that cannot name it passes `None`, and the NWK source address is
    /// used instead — correct for a single-hop frame, and the behaviour of the
    /// two-argument entry point.
    pub async fn process_incoming_nwk_frame_from<'a>(
        &mut self,
        mac_payload: &'a [u8],
        lqi: u8,
        prev_hop: Option<ShortAddress>,
    ) -> Option<NwkIndication<'a>> {
        // A command outcome describes exactly one frame. Clearing it before
        // any early return keeps an outcome from an earlier Leave from being
        // picked up after some later frame that reported nothing.
        self.pending_command_outcome = None;

        // Parse NWK header
        let (header, consumed) = NwkHeader::parse(mac_payload)?;

        let dst = header.dst_addr;
        let src = header.src_addr;

        // Never accept or relay a frame we originated: a neighbour's
        // rebroadcast of our own broadcast would otherwise be delivered to the
        // upper layer and rebroadcast again by us.
        if src == self.nib.network_address {
            log::debug!(
                "[NWK] Dropping self-originated frame seq={}",
                header.seq_number
            );
            return None;
        }

        let frame_type = header.frame_control.frame_type;
        let is_command = frame_type == NwkFrameType::Command as u8;
        let is_data = frame_type == NwkFrameType::Data as u8;
        let secured = header.frame_control.security;

        // ── Frame type admission ──
        // Only NWK Data and NWK Command frames exist on this network. An
        // Inter-PAN or reserved frame type may still parse as a complete NWK
        // header, so it has to be refused *here* — before the BTR records it,
        // before it is relayed or rebroadcast on our behalf and before any
        // command dispatch — rather than at the local-delivery gate below,
        // which a frame addressed elsewhere never reaches.
        if !is_data && !is_command {
            log::debug!(
                "[NWK] Dropping unsupported NWK frame type {} from 0x{:04X}",
                frame_type,
                src.0
            );
            return None;
        }

        let is_broadcast = is_nwk_broadcast(dst);

        // ── Undefined destination addresses ──
        // `0xFFF8..=0xFFFA` are reserved and `0xFFFE` is unassigned: they name
        // neither a device nor a broadcast group, so such a frame can be
        // neither delivered nor routed. It is dropped here — before the
        // broadcast transaction record, before the MAC-broadcast relay and
        // before the unicast relay — rather than being treated as a broadcast
        // because its value happens to sit above 0xFFF7.
        if !is_broadcast && !is_unicast_address(dst) {
            log::debug!(
                "[NWK] Dropping frame for undefined destination 0x{:04X} from 0x{:04X}",
                dst.0,
                src.0,
            );
            return None;
        }

        // ── Unsecured traffic on a secured network ──
        // Once the network key is in use every NWK command and every NWK
        // broadcast is NWK-encrypted, so an unsecured one is forged by
        // construction. Drop it before it can poison the BTR (which would
        // suppress the genuine secured broadcast carrying the same
        // source/sequence pair), before it is rebroadcast on our behalf — the
        // rebroadcast would be re-secured with *our* key material, laundering
        // an attacker's frame into the network and burning a durable outgoing
        // frame counter — and before it is acted upon.
        //
        // Unsecured *unicasts* are deliberately left alone: pre-key
        // commissioning traffic (APS Transport-Key) arrives that way, and the
        // APS layer applies its own security policy to it.
        let unsecured_local_rejoin = !secured
            && is_command
            && !is_broadcast
            && dst == self.nib.network_address
            && self.can_route()
            && header.radius == 1
            && header.src_ieee.is_some()
            && header
                .dst_ieee
                .is_none_or(|address| address == self.nib.ieee_address)
            && mac_payload.get(consumed) == Some(&(NwkCommandId::RejoinRequest as u8));
        if !secured
            && self.nib.security_enabled
            && (is_command || is_broadcast)
            && !unsecured_local_rejoin
        {
            log::warn!(
                "[NWK] Dropping unsecured NWK {} from 0x{:04X}",
                if is_command { "command" } else { "broadcast" },
                src.0
            );
            return None;
        }
        if !secured
            && self.nib.security_enabled
            && self.child_is_unauthenticated(src)
            && !unsecured_local_rejoin
        {
            log::warn!(
                "[NWK] Dropping unsecured traffic from provisional child 0x{:04X}",
                src.0
            );
            return None;
        }

        let can_route = self.can_route();
        let is_for_us = if is_broadcast {
            self.broadcast_is_for_us(dst)
        } else {
            dst == self.nib.network_address
        };
        // Forwarding is bounded by the radius. Deciding this from the header
        // alone costs nothing and keeps sleepy devices from spending energy on
        // CCM* for a frame they would drop either way.
        let may_forward = can_route && header.radius > 1;
        // A broadcast NWK command is never carried further by the generic
        // relay: its propagation belongs to the command handler (see the
        // broadcast relay below). One that is not addressed to us therefore
        // has nothing left to do and is dropped before CCM* runs on it.
        let may_relay = if is_broadcast {
            may_forward && !is_command
        } else {
            may_forward
        };
        if !is_for_us && !may_relay {
            return None;
        }

        // ── Authentication first ──
        // The NWK header is CCM* additional authenticated data. Nothing may
        // record, mutate, relay or act on a secured frame before its MIC has
        // been verified, and the relay path below re-secures the frame with
        // our own key material rather than replaying the original ciphertext.
        let payload = if secured {
            let (payload, security_source) =
                self.authenticate_incoming(mac_payload, consumed, src)?;
            NwkPayload::Decrypted {
                payload,
                security_source,
            }
        } else {
            NwkPayload::Plain(&mac_payload[consumed..])
        };
        if let Some(security_source) = payload.security_source()
            && self.find_ieee_by_short(src) == Some(security_source)
            && self.authorize_child(src)
        {
            log::info!(
                "[NWK] Child 0x{:04X} proved possession of the network key",
                src.0
            );
        }
        // R22 secured-traffic keepalive: an authenticated frame from an
        // attached end-device child refreshes its End Device Timeout deadline.
        // Runs after `authorize_child` so a frame that both authenticates and
        // keeps alive is credited once, and before the relay branch so a
        // child's data relayed on to the coordinator still counts.
        if let Some(security_source) = payload.security_source() {
            self.refresh_child_keepalive_secured(src, security_source);
        }

        // Two commands are not handled by the generic broadcast and relay
        // paths, so their identity is needed before either runs. The payload
        // is the authenticated plaintext, so this reads a command byte that
        // has already been proven to come from a holder of the network key.
        let command_id = if is_command {
            payload.as_slice().first().copied()
        } else {
            None
        };
        let is_route_request = command_id == Some(NwkCommandId::RouteRequest as u8);
        let is_route_record = command_id == Some(NwkCommandId::RouteRecord as u8);
        let is_rejoin_request = command_id == Some(NwkCommandId::RejoinRequest as u8);

        // ── Broadcast deduplication (BTR) ──
        //
        // Route Requests are deliberately exempt. Every copy of one discovery
        // carries the *originator's* source address and sequence number by
        // design, so the transaction record cannot tell an alternate path from
        // a duplicate: it would drop the copy that arrived over a cheaper route
        // before `handle_route_request` ever compared its cost, and a route
        // discovery would only ever learn the path that happened to arrive
        // first. `RreqRecordTable` owns this decision: it admits better paths
        // and equal-cost copies from another neighbor, but suppresses repeated
        // MAC transmissions from the same neighbor.
        if is_broadcast && can_route && !is_route_request {
            if self.btr.is_duplicate(src, header.seq_number) {
                log::debug!(
                    "[NWK] BTR dup: src=0x{:04X} seq={}",
                    src.0,
                    header.seq_number
                );
                return None;
            }
            self.btr.record(src, header.seq_number);
        }

        // ── Broadcast relay (routers/coordinators rebroadcast) ──
        //
        // Only NWK *Data* broadcasts are propagated by this generic relay.
        // NWK commands are not verbatim-floodable: a Route Request must be
        // rebroadcast by `handle_route_request` with *our* path cost added to
        // it (a verbatim copy would advertise the originator's cost and, being
        // a distinct frame from a new source, defeat the receiver's BTR
        // suppression), a Route Reply travels hop by hop toward the
        // originator, and a Link Status describes this device's own links to
        // its immediate neighbours only — flooding a neighbour's Link Status
        // would attribute its link costs to us. Command propagation is
        // therefore owned by the handlers below; local dispatch is unaffected.
        if is_broadcast && may_forward && !is_command {
            let rebroadcast = self.relay_broadcast(&header, payload.as_slice()).await;
            if let Err(e) = rebroadcast {
                log::warn!(
                    "[NWK] Rebroadcast of frame from 0x{:04X} failed: {:?}",
                    src.0,
                    e
                );
            }
        }

        if !is_for_us {
            // Rejoin Requests are one-hop parent-selection commands and are
            // never routed beyond the prospective parent.
            if is_rejoin_request {
                return None;
            }
            // Not for us — relay unicast if we are a routing device. A
            // broadcast has already been rebroadcast above and must not be
            // sent a second time through the unicast relay.
            if !is_broadcast && may_forward {
                // A Route Record is the one command that must not be carried
                // on unchanged: it exists to record the path it travels, so
                // this router appends itself to it (and re-secures it) instead
                // of relaying the originator's list verbatim.
                let relayed = if is_route_record {
                    self.relay_route_record(&header, payload.as_slice()).await
                } else {
                    self.relay_frame(
                        &header,
                        payload.as_slice(),
                        prev_hop.unwrap_or(header.src_addr),
                    )
                    .await
                };
                if let Err(e) = relayed {
                    log::debug!("[NWK] Relay to 0x{:04X} failed: {:?}", header.dst_addr.0, e);
                }
            }
            return None;
        }

        // NWK command frames are handled internally, not passed to APS. A
        // command never produces an NLDE-DATA indication; a lifecycle outcome
        // is parked for the caller to collect with `take_command_outcome`.
        if is_command {
            // Hop-by-hop routing state is installed against the neighbour the
            // frame was received from, not against the NWK source, which for a
            // propagated Route Request is still the original originator many
            // hops away. Without a MAC source the frame can only have come
            // from its NWK source.
            let previous_hop = prev_hop
                .filter(|hop| is_unicast_address(*hop) && *hop != self.nib.network_address)
                .unwrap_or(src);
            self.pending_command_outcome = self.dispatch_nwk_command(
                &header,
                previous_hop,
                lqi,
                secured,
                payload.security_source(),
                payload.as_slice(),
            );
            return None;
        }

        match payload {
            NwkPayload::Plain(payload) => Some(NwkIndication::Borrowed(NldeDataIndication {
                dst_addr: dst,
                src_addr: src,
                payload,
                lqi,
                security_use: false,
                security_source: None,
            })),
            NwkPayload::Decrypted {
                payload,
                security_source,
            } => Some(NwkIndication::Owned(NldeDataIndicationOwned {
                dst_addr: dst,
                src_addr: src,
                payload,
                lqi,
                security_use: true,
                security_source: Some(security_source),
            })),
        }
    }

    /// Verify and decrypt a secured incoming NWK frame.
    ///
    /// Returns the plaintext NWK payload, or `None` when the frame must be
    /// dropped. The incoming replay counter is committed exactly once, and
    /// only after the MIC verifies, so a forged or replayed frame can neither
    /// advance the replay window nor reach the relay path.
    fn authenticate_incoming(
        &mut self,
        mac_payload: &[u8],
        header_len: usize,
        src: ShortAddress,
    ) -> Option<(heapless::Vec<u8, MAX_NWK_FRAME>, IeeeAddress)> {
        self.rx_security_stats.secured_frames =
            self.rx_security_stats.secured_frames.wrapping_add(1);

        let after_header = mac_payload.get(header_len..)?;
        let Some((sec_hdr, sec_consumed)) = crate::security::NwkSecurityHeader::parse(after_header)
        else {
            self.rx_security_stats.security_header_parse_failures = self
                .rx_security_stats
                .security_header_parse_failures
                .wrapping_add(1);
            return None;
        };

        let Some(key) = self
            .security
            .key_by_seq(sec_hdr.key_seq_number)
            .map(|entry| entry.key)
        else {
            self.rx_security_stats.missing_keys =
                self.rx_security_stats.missing_keys.wrapping_add(1);
            return None;
        };

        log::debug!(
            "[NWK SEC] sc=0x{:02X} fc={} key_seq={}",
            sec_hdr.security_control,
            sec_hdr.frame_counter,
            sec_hdr.key_seq_number,
        );

        // Step 1: check the replay window WITHOUT committing.
        if !self.security.check_frame_counter_for_key(
            &sec_hdr.source_address,
            sec_hdr.key_seq_number,
            sec_hdr.frame_counter,
        ) {
            self.rx_security_stats.replay_rejections =
                self.rx_security_stats.replay_rejections.wrapping_add(1);
            log::warn!("[NWK] Frame counter replay from 0x{:04X}", src.0);
            return None;
        }

        // Step 2: rebuild the AAD (NWK header || auxiliary header) with the
        // ACTUAL security level (5), not the zero carried over the air.
        let aad_len = header_len + sec_consumed;
        if aad_len > MAX_NWK_AAD {
            // Truncating the AAD would silently fail every MIC check instead
            // of reporting that the header cannot be authenticated.
            log::warn!(
                "[NWK] NWK header too long to authenticate ({} bytes)",
                aad_len
            );
            self.rx_security_stats.security_header_parse_failures = self
                .rx_security_stats
                .security_header_parse_failures
                .wrapping_add(1);
            return None;
        }
        let mut aad_buf = [0u8; MAX_NWK_AAD];
        aad_buf[..aad_len].copy_from_slice(&mac_payload[..aad_len]);
        aad_buf[header_len] = (aad_buf[header_len] & !0x07) | 0x05;

        match self.security.decrypt_with(
            &mut self.mac,
            &aad_buf[..aad_len],
            &after_header[sec_consumed..],
            &key,
            &sec_hdr,
        ) {
            Some(plaintext) => {
                self.rx_security_stats.decrypt_successes =
                    self.rx_security_stats.decrypt_successes.wrapping_add(1);
                // Step 3: MIC verified — NOW commit the replay counter, once.
                self.security.commit_frame_counter_for_key(
                    &sec_hdr.source_address,
                    sec_hdr.key_seq_number,
                    sec_hdr.frame_counter,
                );
                log::debug!(
                    "[NWK] Decrypted frame from 0x{:04X} ({} bytes)",
                    src.0,
                    plaintext.len()
                );
                Some((plaintext, sec_hdr.source_address))
            }
            None => {
                self.rx_security_stats.decrypt_failures =
                    self.rx_security_stats.decrypt_failures.wrapping_add(1);
                log::warn!("[NWK] Decrypt/MIC failed from 0x{:04X}", src.0);
                // Do NOT commit the frame counter — the frame is forged.
                None
            }
        }
    }

    /// Whether a broadcast destination is addressed to this device.
    ///
    /// Zigbee PRO R22 §3.6.5 broadcast addresses:
    /// - `0xFFFF` — all devices
    /// - `0xFFFD` — all devices with receiver on when idle
    /// - `0xFFFC` — all routers and the coordinator
    /// - `0xFFFB` — low-power routers
    ///
    /// `0xFFFD` is accepted regardless of `rx_on_when_idle`: a frame that
    /// reached this device was either received with the radio on or handed
    /// over by the parent from its indirect queue, and sleepy sensors depend
    /// on seeing those ZDO broadcasts. `0xFFFB` and the reserved values are
    /// not claimed by this stack — they are still relayed by routers, but are
    /// not delivered locally.
    fn broadcast_is_for_us(&self, dst: ShortAddress) -> bool {
        match dst.0 {
            0xFFFF | 0xFFFD => true,
            0xFFFC => self.device_type != DeviceType::EndDevice,
            _ => false,
        }
    }

    /// Relay a NWK frame (router/coordinator duty).
    ///
    /// `payload` is the authenticated plaintext NWK payload produced by
    /// [`NwkLayer::authenticate_incoming`] (or the frame's own plaintext when
    /// it was unsecured). Secured frames are rebuilt and re-secured with this
    /// device's key material by [`NwkLayer::build_nwk_frame`]: the NWK header
    /// is CCM* additional authenticated data, so forwarding the original
    /// ciphertext under a mutated header would fail the MIC at the next hop.
    async fn relay_frame(
        &mut self,
        header: &NwkHeader,
        payload: &[u8],
        previous_hop: ShortAddress,
    ) -> Result<(), NwkStatus> {
        // Decrement radius
        let new_radius = header.radius.saturating_sub(1);
        if new_radius == 0 {
            return Ok(()); // TTL expired
        }

        // ── Source routing: use relay list instead of routing table ──
        if header.source_route.is_some() {
            return self
                .relay_frame_source_routed(header, payload, new_radius)
                .await;
        }

        let is_many_to_one_status = header.frame_control.frame_type == NwkFrameType::Command as u8
            && payload.first() == Some(&(NwkCommandId::NetworkStatus as u8))
            && crate::frames::NetworkStatusCommand::parse(payload.get(1..).unwrap_or_default())
                .is_some_and(|status| {
                    status.status_code
                        == crate::frames::NetworkStatusCommand::MANY_TO_ONE_ROUTE_FAILURE
                });
        let route_is_many_to_one = self
            .routing
            .get_entry(header.dst_addr)
            .is_some_and(|route| route.many_to_one);
        if self.routing.has_failed_many_to_one(header.dst_addr) && !is_many_to_one_status {
            return Err(NwkStatus::RouteError);
        }

        // Determine next hop for the final destination. A many-to-one failure
        // notification is the one exception: if the failed route is already
        // gone, R22 sends it through a random router neighbor.
        let next_hop = if is_many_to_one_status {
            if self.neighbors.find_by_short(header.dst_addr).is_some() {
                header.dst_addr
            } else if let Some(next_hop) = self.routing.next_hop(header.dst_addr) {
                next_hop
            } else {
                self.random_router_neighbor(Some(previous_hop), None)
                    .ok_or(NwkStatus::RouteError)?
            }
        } else {
            self.resolve_next_hop(header.dst_addr)?
        };

        // A parent forwarding its direct end-device child's data over a
        // many-to-one route originates a Route Record on the child's behalf
        // before the data frame. The record starts with this parent as relay
        // and carries the child's short and IEEE identities in the NWK header.
        if route_is_many_to_one
            && header.frame_control.frame_type == NwkFrameType::Data as u8
            && previous_hop == header.src_addr
        {
            let child_ieee = self
                .neighbors
                .find_by_short(header.src_addr)
                .filter(|neighbor| neighbor.relationship == crate::neighbor::Relationship::Child)
                .filter(|neighbor| {
                    neighbor.device_type == crate::neighbor::NeighborDeviceType::EndDevice
                })
                .map(|neighbor| neighbor.ieee_address);
            if let Some(child_ieee) = child_ieee
                && let Err(error) = self
                    .send_route_record_from(
                        header.dst_addr,
                        header.src_addr,
                        child_ieee,
                        &[self.nib.network_address],
                    )
                    .await
            {
                // A failed Route Record is discarded silently at the
                // protocol level. The child's data remains an independent
                // relay attempt and must not be lost with it.
                log::warn!(
                    "[NWK] Child Route Record to 0x{:04X} failed: {:?}",
                    header.dst_addr.0,
                    error,
                );
            }
        }

        // Rebuild the frame with the decremented radius
        let mut relay_header = header.clone();
        relay_header.radius = new_radius;
        let mut relay_buf = [0u8; MAX_NWK_FRAME];
        let total = self.build_nwk_frame(&relay_header, payload, &mut relay_buf)?;

        // Sleeping children receive one queued NWK frame after each matching
        // MAC Data Request. Queue first, then arm ACK Frame Pending; rollback
        // the exact slot if the MAC cannot remember this child.
        if self.is_sleepy_child(next_hop) {
            return self.enqueue_indirect_for_child(next_hop, &relay_buf[..total]);
        }

        let mac_result = self
            .mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &relay_buf[..total],
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: next_hop.0 != 0xFFFF,
                    ..Default::default()
                },
            })
            .await;

        // ── Route repair: if MAC TX fails, handle relay failure ──
        if mac_result.is_err() && is_many_to_one_status {
            // A failed status relay is retried through another random router
            // neighbor. Never answer a failed Network Status with another
            // Network Status.
            if let Some(alternate) = self.random_router_neighbor(Some(previous_hop), Some(next_hop))
                && self
                    .mac
                    .mcps_data(McpsDataRequest {
                        src_addr_mode: AddressMode::Short,
                        dst_address: MacAddress::Short(self.nib.pan_id, alternate),
                        payload: &relay_buf[..total],
                        msdu_handle: self.nib.next_seq(),
                        tx_options: TxOptions {
                            ack_tx: true,
                            ..Default::default()
                        },
                    })
                    .await
                    .is_ok()
            {
                return Ok(());
            }
            return Err(NwkStatus::RouteError);
        }

        if mac_result.is_err() {
            self.handle_relay_failure(header, next_hop, false);
            return Err(NwkStatus::RouteError);
        }

        Ok(())
    }

    /// Whether `addr` is a child of ours that sleeps, and therefore can only
    /// be reached through indirect (polled) delivery.
    ///
    /// [`crate::neighbor::NeighborEntry::new_from_annce`] leaves
    /// `rx_on_when_idle` false for every device that merely announced itself,
    /// so that flag alone would classify arbitrary siblings and unknown
    /// announced neighbours as sleepy children. Only a device that joined
    /// through us — where the capability information came from its own
    /// Association Request — is treated as one; everything else is relayed
    /// directly.
    fn is_sleepy_child(&self, addr: ShortAddress) -> bool {
        self.neighbors.find_by_short(addr).is_some_and(|neighbor| {
            !neighbor.rx_on_when_idle
                && matches!(
                    neighbor.relationship,
                    crate::neighbor::Relationship::Child
                        | crate::neighbor::Relationship::UnauthenticatedChild
                )
        })
    }

    /// Relay a frame using source routing (relay list in NWK header).
    async fn relay_frame_source_routed(
        &mut self,
        header: &NwkHeader,
        payload: &[u8],
        new_radius: u8,
    ) -> Result<(), NwkStatus> {
        let our_addr = self.nib.network_address;
        let sr = header
            .source_route
            .as_ref()
            .ok_or(NwkStatus::InvalidParameter)?;
        let previous_index = sr.relay_index;

        // Find next hop from source route relay list.
        // The relay list is destination-nearest first and `relay_index` names
        // this relay. Decrement it to advance toward the destination.
        let (next_hop, new_index) = process_source_route(sr, our_addr, header.dst_addr)?;

        // A source-routed response proves that a route-record-capable
        // concentrator has the reverse path. Clear the one-shot Route Record
        // requirement on the route back to that concentrator; low-RAM
        // concentrators keep it armed through `route_record_every_frame`.
        self.routing.clear_route_record_required(header.src_addr);

        // Build new header with updated source route. Both mutable fields are
        // applied before the frame is (re-)secured, because the header is part
        // of the CCM* authenticated data.
        let mut relay_header = header.clone();
        relay_header.radius = new_radius;
        if let Some(ref mut relay_sr) = relay_header.source_route {
            relay_sr.relay_index = new_index;
        }

        let mut relay_buf = [0u8; MAX_NWK_FRAME];
        let total = self.build_nwk_frame(&relay_header, payload, &mut relay_buf)?;

        if self.is_sleepy_child(next_hop) {
            return self.enqueue_indirect_for_child(next_hop, &relay_buf[..total]);
        }

        log::debug!(
            "[NWK] Source-route relay: next_hop=0x{:04X} index={}→{}",
            next_hop.0,
            previous_index,
            new_index,
        );

        let mac_result = self
            .mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &relay_buf[..total],
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: true,
                    ..Default::default()
                },
            })
            .await;

        if mac_result.is_err() {
            self.handle_relay_failure(header, next_hop, true);
            return Err(NwkStatus::RouteError);
        }

        Ok(())
    }

    /// Relay a Route Record, appending this device to its relay list.
    ///
    /// A Route Record exists to record the path a device's traffic takes to a
    /// concentrator, which then source-routes back over it. Relaying it
    /// unchanged would hand the concentrator a path missing every router
    /// between the originator and itself, so each authenticated intermediate
    /// router appends its own short address and increments the relay count.
    ///
    /// `payload` is the authenticated plaintext NWK command payload —
    /// `[command id, relay count, relay addresses...]`. The rebuilt frame
    /// keeps the originator's NWK source address, destination and sequence
    /// number, decrements the radius and is re-secured hop by hop by
    /// [`NwkLayer::build_nwk_frame`] with this device's own IEEE address and a
    /// fresh durable frame counter, exactly like any other relay.
    async fn relay_route_record(
        &mut self,
        header: &NwkHeader,
        payload: &[u8],
    ) -> Result<(), NwkStatus> {
        let new_radius = header.radius.saturating_sub(1);
        if new_radius == 0 {
            return Ok(()); // TTL expired
        }

        // Skip the command identifier the caller matched on.
        let body = payload.get(1..).ok_or(NwkStatus::InvalidParameter)?;
        let relay_count = *body.first().ok_or(NwkStatus::InvalidParameter)? as usize;
        let listed_len = 1 + relay_count * 2;
        if body.len() < listed_len {
            log::warn!(
                "[NWK] RouteRecord from 0x{:04X} too short to extend: need {}, have {}",
                header.src_addr.0,
                listed_len,
                body.len(),
            );
            return Err(NwkStatus::InvalidParameter);
        }
        // A concentrator can only store, and only source-route over,
        // `MAX_SOURCE_ROUTE_RELAYS` hops. Extending a record beyond that would
        // deliver a path that has to be truncated — a source route over the
        // wrong hops — so the record is refused explicitly instead.
        if relay_count >= crate::routing::MAX_SOURCE_ROUTE_RELAYS {
            log::warn!(
                "[NWK] RouteRecord from 0x{:04X} already lists {} relays (max {}) — not extended",
                header.src_addr.0,
                relay_count,
                crate::routing::MAX_SOURCE_ROUTE_RELAYS,
            );
            return Err(NwkStatus::FrameTooLong);
        }

        // [command id, relay count, relays..., this device]
        let mut cmd = [0u8; 2 + 2 * crate::routing::MAX_SOURCE_ROUTE_RELAYS];
        let cmd_len = 2 + (relay_count + 1) * 2;
        if cmd_len > cmd.len() {
            return Err(NwkStatus::FrameTooLong);
        }
        cmd[0] = NwkCommandId::RouteRecord as u8;
        cmd[1] = (relay_count + 1) as u8;
        cmd[2..2 + relay_count * 2].copy_from_slice(&body[1..listed_len]);
        let our_addr = self.nib.network_address;
        cmd[cmd_len - 2] = (our_addr.0 & 0xFF) as u8;
        cmd[cmd_len - 1] = ((our_addr.0 >> 8) & 0xFF) as u8;

        let next_hop = self.resolve_next_hop(header.dst_addr)?;

        let mut relay_header = header.clone();
        relay_header.radius = new_radius;
        let mut relay_buf = [0u8; MAX_NWK_FRAME];
        let total = self.build_nwk_frame(&relay_header, &cmd[..cmd_len], &mut relay_buf)?;

        log::debug!(
            "[NWK] Route Record from 0x{:04X} extended to {} relays, on to 0x{:04X}",
            header.src_addr.0,
            relay_count + 1,
            next_hop.0,
        );

        let mac_result = self
            .mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, next_hop),
                payload: &relay_buf[..total],
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: true,
                    ..Default::default()
                },
            })
            .await;

        if mac_result.is_err() {
            // R22 requires Route Record relay failures to be discarded
            // silently: no Network Status is generated for this command.
            self.routing.remove(header.dst_addr);
            return Err(NwkStatus::RouteError);
        }

        Ok(())
    }

    /// Select a router neighbor for many-to-one failure recovery.
    fn random_router_neighbor(
        &mut self,
        exclude_a: Option<ShortAddress>,
        exclude_b: Option<ShortAddress>,
    ) -> Option<ShortAddress> {
        let eligible = |neighbor: &&crate::neighbor::NeighborEntry| {
            neighbor.device_type == crate::neighbor::NeighborDeviceType::Router
                && neighbor.outgoing_cost != 0
                && Some(neighbor.network_address) != exclude_a
                && Some(neighbor.network_address) != exclude_b
        };
        let count = self.neighbors.iter().filter(eligible).count();
        if count == 0 {
            return None;
        }

        let sample = crate::routing::routing_random_sample(
            self.mac.monotonic_micros()
                ^ (u32::from(self.nib.network_address.0) << 16)
                ^ (u32::from(self.nib.sequence_number) << 8)
                ^ u32::from(exclude_a.unwrap_or(ShortAddress::BROADCAST).0)
                ^ u32::from(exclude_b.unwrap_or(ShortAddress::BROADCAST).0).rotate_left(11),
        ) as usize;
        self.neighbors
            .iter()
            .filter(eligible)
            .nth(sample % count)
            .map(|neighbor| neighbor.network_address)
    }

    /// Handle relay failure: retire the broken path and queue the R22 status.
    fn handle_relay_failure(
        &mut self,
        header: &NwkHeader,
        failed_next_hop: ShortAddress,
        source_routed: bool,
    ) {
        let failed_dest = header.dst_addr;
        let frame_source = header.src_addr;
        let many_to_one = !source_routed
            && self
                .routing
                .get_entry(failed_dest)
                .is_some_and(|route| route.many_to_one);
        log::warn!(
            "[NWK] Relay failure for dst=0x{:04X}, removing route",
            failed_dest.0,
        );

        // Remove the broken route
        self.routing.remove(failed_dest);

        if frame_source == self.nib.network_address {
            return;
        }

        let pending = if source_routed {
            crate::PendingNetworkStatus {
                destination: frame_source,
                status_code: crate::frames::NetworkStatusCommand::SOURCE_ROUTE_FAILURE,
                failed_destination: failed_dest,
                next_hop: None,
            }
        } else if many_to_one {
            let Some(next_hop) =
                self.random_router_neighbor(Some(failed_next_hop), Some(frame_source))
            else {
                log::warn!(
                    "[NWK] No alternate router neighbor for MTOR failure to 0x{:04X}",
                    failed_dest.0,
                );
                return;
            };
            crate::PendingNetworkStatus {
                // R22 sends the command toward the concentrator named by the
                // failed data frame's NWK destination.
                destination: failed_dest,
                status_code: crate::frames::NetworkStatusCommand::MANY_TO_ONE_ROUTE_FAILURE,
                // The payload names the source whose upstream path failed.
                failed_destination: frame_source,
                next_hop: Some(next_hop),
            }
        } else {
            crate::PendingNetworkStatus {
                destination: frame_source,
                status_code: crate::frames::NetworkStatusCommand::NO_ROUTE_AVAILABLE,
                failed_destination: failed_dest,
                next_hop: None,
            }
        };
        if self.pending_route_errors.push(pending).is_err() {
            log::warn!("[NWK] Network Status queue full");
        }
    }

    /// Relay a broadcast NWK frame via MAC broadcast with decremented radius.
    ///
    /// `payload` is the authenticated plaintext; a secured broadcast is
    /// re-secured with this device's own key material, exactly like a unicast
    /// relay, because the decremented radius is authenticated header data.
    async fn relay_broadcast(
        &mut self,
        header: &NwkHeader,
        payload: &[u8],
    ) -> Result<(), NwkStatus> {
        let new_radius = header.radius.saturating_sub(1);
        if new_radius == 0 {
            return Ok(());
        }

        let mut relay_header = header.clone();
        relay_header.radius = new_radius;
        let mut relay_buf = [0u8; MAX_NWK_FRAME];
        let total = self.build_nwk_frame(&relay_header, payload, &mut relay_buf)?;

        log::debug!(
            "[NWK] Relaying broadcast from 0x{:04X} (radius {} → {})",
            header.src_addr.0,
            header.radius,
            new_radius
        );

        self.mac
            .mcps_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(self.nib.pan_id, ShortAddress::BROADCAST),
                payload: &relay_buf[..total],
                msdu_handle: self.nib.next_seq(),
                tx_options: TxOptions {
                    ack_tx: false, // No ACK for broadcast
                    ..Default::default()
                },
            })
            .await
            .map_err(|_| NwkStatus::RouteError)?;

        Ok(())
    }

    /// Resolve the MAC next hop for a given NWK destination.
    ///
    /// Strategy:
    /// 1. If destination is a neighbor → send directly
    /// 2. If destination is in routing table → use next_hop
    /// 3. If we're an end device → send to parent
    /// 4. For broadcast → send to all neighbors (simplified: send to parent)
    pub(crate) fn resolve_next_hop(
        &self,
        destination: ShortAddress,
    ) -> Result<ShortAddress, NwkStatus> {
        // Broadcast: send to parent (end device) or all neighbors (router)
        if is_nwk_broadcast(destination) {
            if self.device_type == DeviceType::EndDevice {
                return Ok(self.nib.parent_address);
            }
            // Routers broadcast via MAC broadcast
            return Ok(ShortAddress::BROADCAST);
        }

        // Reserved (0xFFF8..=0xFFFA) or unassigned (0xFFFE): neither a device
        // nor a broadcast group. Refuse explicitly instead of flooding an
        // undefined address as a MAC broadcast.
        if !is_unicast_address(destination) {
            return Err(NwkStatus::InvalidParameter);
        }

        // Direct neighbor?
        if self.neighbors.find_by_short(destination).is_some() {
            return Ok(destination);
        }

        // Routing table lookup
        if let Some(next) = self.routing.next_hop(destination) {
            return Ok(next);
        }

        // End device fallback: always route through parent
        if self.device_type == DeviceType::EndDevice {
            return Ok(self.nib.parent_address);
        }

        // Tree routing fallback
        if let Some(next) = self.routing.tree_route(
            self.nib.network_address,
            destination,
            self.nib.depth,
            self.nib.max_routers,
            self.nib.max_depth,
        ) {
            return Ok(next);
        }

        // Route to parent as last resort
        if self.nib.parent_address.0 != 0xFFFF {
            Ok(self.nib.parent_address)
        } else {
            Err(NwkStatus::RouteError)
        }
    }

    // ── NWK Command Dispatch ─────────────────────────────────

    /// Dispatch an incoming NWK command frame to the appropriate handler.
    ///
    /// Routing and neighbour maintenance commands are handled entirely here.
    /// Commands that change the device's network lifecycle are reported back
    /// so the layer above can schedule a rejoin or update persistence.
    fn dispatch_nwk_command(
        &mut self,
        header: &NwkHeader,
        prev_hop: ShortAddress,
        lqi: u8,
        secured: bool,
        security_source: Option<IeeeAddress>,
        payload: &[u8],
    ) -> Option<NwkCommandOutcome> {
        let src = header.src_addr;
        let dst = header.dst_addr;
        if payload.is_empty() {
            log::warn!("[NWK] Empty NWK command payload from 0x{:04X}", src.0);
            return None;
        }

        let cmd_id_byte = payload[0];
        let cmd_payload = &payload[1..];

        // On a secured network every NWK command is NWK-encrypted. Accepting
        // an unsecured one would let any nearby device forge a Leave, a route
        // error or a link-status update.
        let is_local_rejoin = cmd_id_byte == NwkCommandId::RejoinRequest as u8
            && dst == self.nib.network_address
            && self.can_route()
            && header.src_ieee.is_some();
        if self.nib.security_enabled && !secured && !is_local_rejoin {
            log::warn!(
                "[NWK] Dropping unsecured NWK command 0x{:02X} from 0x{:04X}",
                payload[0],
                src.0
            );
            return None;
        }

        match NwkCommandId::from_u8(cmd_id_byte) {
            Some(NwkCommandId::Leave) => return self.handle_nwk_leave(src, dst, cmd_payload),
            Some(NwkCommandId::RouteRequest) => {
                self.handle_route_request(header, prev_hop, lqi, cmd_payload)
            }
            // A Route Reply installs a next hop, so it uses the neighbour the
            // frame came from. A Route Record names the *originating* device
            // whose path is being recorded, and a Link Status describes the
            // sender's own links: both stay keyed on the NWK source.
            Some(NwkCommandId::RouteReply) => self.handle_route_reply(prev_hop, cmd_payload),
            Some(NwkCommandId::RouteRecord) => self.handle_route_record(src, cmd_payload),
            Some(NwkCommandId::LinkStatus) => self.handle_link_status(src, cmd_payload),
            Some(NwkCommandId::NetworkStatus) => self.handle_network_status(src, cmd_payload),
            Some(NwkCommandId::EdTimeoutResponse) => {
                self.handle_ed_timeout_response(src, dst, cmd_payload)
            }
            Some(NwkCommandId::EdTimeoutRequest) => {
                return self.handle_ed_timeout_request(
                    header,
                    secured,
                    security_source,
                    cmd_payload,
                );
            }
            Some(NwkCommandId::RejoinRequest) => {
                return self.handle_rejoin_request(header, secured, security_source, cmd_payload);
            }
            Some(other) => {
                log::debug!(
                    "[NWK] Ignoring NWK command {:?} from 0x{:04X}",
                    other,
                    src.0
                );
            }
            None => {
                log::warn!(
                    "[NWK] Unknown NWK command ID 0x{:02X} from 0x{:04X}",
                    cmd_id_byte,
                    src.0
                );
            }
        }
        None
    }

    // ── NWK Command Handlers ─────────────────────────────────

    fn handle_rejoin_request(
        &self,
        header: &NwkHeader,
        secured: bool,
        security_source: Option<IeeeAddress>,
        payload: &[u8],
    ) -> Option<NwkCommandOutcome> {
        if !self.can_route()
            || header.dst_addr != self.nib.network_address
            || header.radius != 1
            || !is_unicast_address(header.src_addr)
            || payload.len() != 1
        {
            log::warn!(
                "[NWK] Rejecting malformed Rejoin Request from 0x{:04X}",
                header.src_addr.0
            );
            return None;
        }
        if header
            .dst_ieee
            .is_some_and(|address| address != self.nib.ieee_address)
        {
            return None;
        }
        let ieee = header.src_ieee?;
        if secured && security_source != Some(ieee) {
            log::warn!("[NWK] Rejoin Request IEEE does not match its security source");
            return None;
        }

        Some(NwkCommandOutcome::ChildRejoinRequest {
            src: header.src_addr,
            ieee,
            capability_info: payload[0],
            secured,
        })
    }

    /// Handle incoming NWK End Device Timeout Request (0x0B) on a parent.
    ///
    /// The R22 server side. Accepted only when every one of these holds, so an
    /// arbitrary network member can never rewrite a child's timeout state:
    /// - this device can route (only a parent answers requests);
    /// - the frame was addressed directly to this parent's own short address
    ///   (and matching IEEE, if the sender included one);
    /// - the source is a real unicast address;
    /// - the reserved End Device Configuration byte is zero and the requested
    ///   enumeration is defined (0..=14) — enforced by the wire parser;
    /// - on a secured network the frame was NWK-authenticated (the shared
    ///   dispatcher guard) and its auxiliary-header source IEEE matches the
    ///   IEEE the sender advertised;
    /// - the sender is a directly-attached, **authenticated** child of this
    ///   parent whose stored IEEE matches that authenticated identity.
    ///
    /// The actual policy decision and the 0x0C transmission (which may be
    /// indirect for a sleepy child) are performed asynchronously by the layer
    /// above via [`NwkLayer::respond_to_end_device_timeout_request`], so this
    /// synchronous handler only validates and reports.
    fn handle_ed_timeout_request(
        &self,
        header: &NwkHeader,
        secured: bool,
        security_source: Option<IeeeAddress>,
        payload: &[u8],
    ) -> Option<NwkCommandOutcome> {
        if !self.can_route() {
            return None;
        }
        if header.dst_addr != self.nib.network_address {
            return None;
        }
        if header
            .dst_ieee
            .is_some_and(|address| address != self.nib.ieee_address)
        {
            return None;
        }
        if !is_unicast_address(header.src_addr) {
            return None;
        }
        // On a secured network the request must be authenticated; the shared
        // dispatcher guard already enforced this, so a false `secured` here can
        // only occur on an unsecured network, where no authenticated child can
        // exist and the relationship check below fails anyway.
        if self.nib.security_enabled && !secured {
            return None;
        }
        let request = crate::frames::EdTimeoutRequest::parse(payload)?;

        // Authenticated identity of the sender: the NWK security source on a
        // secured network, otherwise the IEEE the sender embedded in the
        // header. Any embedded IEEE must agree with the authenticated one.
        let ieee = if secured {
            security_source?
        } else {
            header.src_ieee?
        };
        if header.src_ieee.is_some_and(|address| address != ieee) {
            log::warn!(
                "[NWK] ED Timeout Request IEEE does not match its security source (0x{:04X})",
                header.src_addr.0
            );
            return None;
        }

        // The sender must be a directly-attached authenticated child, matched
        // by both its short address and the authenticated IEEE. A provisional
        // (unauthenticated) child, a router, a sibling or an unknown device is
        // refused silently — no response is sent.
        let is_attached_child =
            self.neighbors
                .find_by_short(header.src_addr)
                .is_some_and(|entry| {
                    entry.relationship == crate::neighbor::Relationship::Child
                        && entry.ieee_address == ieee
                });
        if !is_attached_child {
            log::warn!(
                "[NWK] Ignoring ED Timeout Request from non-child 0x{:04X}",
                header.src_addr.0
            );
            return None;
        }

        Some(NwkCommandOutcome::EndDeviceTimeoutRequest {
            src: header.src_addr,
            ieee,
            requested_timeout: request.requested_timeout,
        })
    }

    /// Handle incoming NWK End Device Timeout Response (0x0C).
    ///
    /// Accepted only when every one of these holds:
    /// - this device is an end device (a router never negotiates a timeout
    ///   for itself);
    /// - the frame came from the current parent, and that parent address is a
    ///   real unicast address rather than the unassigned/broadcast range;
    /// - the frame was addressed to this device's own network address;
    /// - the shared secured-NWK-command guard in the dispatcher already
    ///   passed, so on a secured network the frame was NWK-decrypted.
    ///
    /// Without the source and destination checks any neighbour could shorten
    /// this device's keepalive interval or advertise a keepalive method the
    /// real parent does not implement, which would silently age the device out
    /// of its parent's child table.
    ///
    /// No [`NwkCommandOutcome`] is produced: the effect is entirely NIB state,
    /// which the layer above detects by comparing the negotiation fields
    /// around `process_incoming_nwk_frame`.
    fn handle_ed_timeout_response(&mut self, src: ShortAddress, dst: ShortAddress, payload: &[u8]) {
        if self.device_type != crate::DeviceType::EndDevice {
            return;
        }
        let parent = self.nib.parent_address;
        if !is_unicast_address(parent) || src != parent || dst != self.nib.network_address {
            log::warn!(
                "[NWK] Ignoring ED Timeout Response src=0x{:04X} dst=0x{:04X} (parent=0x{:04X})",
                src.0,
                dst.0,
                parent.0,
            );
            return;
        }
        let Some(response) = crate::frames::EdTimeoutResponse::parse(payload) else {
            log::warn!("[NWK] Malformed ED Timeout Response from 0x{:04X}", src.0);
            return;
        };

        match response.status {
            crate::frames::ED_TIMEOUT_STATUS_SUCCESS => {
                if !self.nib.take_end_device_timeout_response_pending() {
                    log::warn!(
                        "[NWK] Ignoring unsolicited ED Timeout Response from 0x{:04X}",
                        src.0
                    );
                    return;
                }
                self.nib.accept_end_device_timeout(response.parent_info);
                log::info!(
                    "[NWK] ED Timeout accepted: enum={} parent_info=0x{:02X}",
                    self.nib.end_device_timeout,
                    self.nib.parent_information,
                );
            }
            crate::frames::ED_TIMEOUT_STATUS_INCORRECT_VALUE => {
                if !self.nib.take_end_device_timeout_response_pending() {
                    log::warn!(
                        "[NWK] Ignoring unsolicited ED Timeout Response from 0x{:04X}",
                        src.0
                    );
                    return;
                }
                // A refusal carries no usable parent information, so any
                // previously validated advertisement stays untouched.
                let lowered = self.nib.lower_requested_end_device_timeout();
                log::warn!(
                    "[NWK] ED Timeout refused; requested enum now {} (lowered={})",
                    self.nib.requested_end_device_timeout,
                    lowered,
                );
            }
            other => {
                log::warn!("[NWK] Unknown ED Timeout Response status 0x{:02X}", other);
            }
        }
    }

    /// Handle incoming NWK Leave command.
    ///
    /// A Leave *request* is only honoured from the current parent and only
    /// when addressed to this device. A Leave *indication* (a device
    /// announcing that it left) is honoured when addressed to this device or
    /// to the rx-on-when-idle broadcast address.
    fn handle_nwk_leave(
        &mut self,
        src: ShortAddress,
        dst: ShortAddress,
        payload: &[u8],
    ) -> Option<NwkCommandOutcome> {
        let Some(leave) = crate::frames::LeaveCommand::parse(payload) else {
            log::warn!("[NWK] Malformed Leave command from 0x{:04X}", src.0);
            return None;
        };

        log::info!(
            "[NWK] Leave from 0x{:04X} (remove_children={}, request={}, rejoin={})",
            src.0,
            leave.remove_children,
            leave.request,
            leave.rejoin
        );

        if leave.request {
            if dst != self.nib.network_address || src != self.nib.parent_address {
                log::warn!(
                    "[NWK] Ignoring unauthorized leave request src=0x{:04X} dst=0x{:04X}",
                    src.0,
                    dst.0
                );
                return None;
            }
            // Stop using the network until the caller either honours the
            // requested rejoin or clears its persisted network state.
            self.joined = false;
            // The parent relationship is over either way, so the R22 End
            // Device Timeout has to be renegotiated with whichever parent
            // accepts the device next.
            self.nib.reset_end_device_timeout_negotiation();
            return Some(NwkCommandOutcome::LeaveRequested {
                src,
                rejoin: leave.rejoin,
                remove_children: leave.remove_children,
            });
        }

        if dst != self.nib.network_address && dst != ShortAddress::BROADCAST_RX_ON_WHEN_IDLE {
            return None;
        }

        self.neighbors.remove(src);
        if src == self.nib.parent_address {
            self.joined = false;
            self.nib.reset_end_device_timeout_negotiation();
            return Some(NwkCommandOutcome::ParentLeft { src });
        }
        None
    }

    /// Handle incoming Route Request (RREQ).
    ///
    /// `header` is the NWK header the request arrived with — for a request
    /// that has already crossed the mesh it still names the *originator*, not
    /// the neighbour that transmitted it. `prev_hop` is that neighbour, and it
    /// is the only address that may be installed as a next hop.
    ///
    /// Propagation forwards the originator's own broadcast: source address,
    /// sequence number and end-device-initiator bit are preserved, the radius
    /// is decremented, and only the RREQ path cost is updated. Re-originating
    /// the request with our address and a fresh sequence number would make it
    /// a new broadcast for every receiver's transaction record, so two routers
    /// would hand the same discovery back and forth without bound and the
    /// many-to-one next hops would form a cycle.
    fn handle_route_request(
        &mut self,
        header: &NwkHeader,
        prev_hop: ShortAddress,
        lqi: u8,
        payload: &[u8],
    ) {
        let originator = header.src_addr;
        let Some(rreq) = crate::frames::RouteRequest::parse(payload) else {
            log::warn!("[NWK] Malformed RREQ from 0x{:04X}", prev_hop.0);
            return;
        };

        let many_to_one_value = (rreq.command_options >> 3) & 0x03;
        if many_to_one_value == 3 {
            log::warn!("[NWK] Reserved MTOR option 3 from 0x{:04X}", originator.0,);
            return;
        }
        let concentrator_type =
            crate::routing::ConcentratorType::from_rreq_options(rreq.command_options);
        let is_many_to_one = concentrator_type.is_some();

        if is_many_to_one
            && (header.dst_addr != ShortAddress(0xFFFC)
                || rreq.dst_addr != ShortAddress(0xFFFC)
                || !is_unicast_address(originator))
        {
            log::warn!(
                "[NWK] Malformed MTOR RREQ from 0x{:04X}: nwk_dst=0x{:04X} payload_dst=0x{:04X}",
                originator.0,
                header.dst_addr.0,
                rreq.dst_addr.0,
            );
            return;
        }

        // Cost of the path the request travelled to reach us: what the
        // previous hop advertised plus our link from it.
        let link_cost = if is_many_to_one {
            let Some(neighbor) = self.neighbors.find_by_short(prev_hop) else {
                log::debug!(
                    "[NWK] MTOR RREQ from unknown previous hop 0x{:04X} dropped",
                    prev_hop.0,
                );
                return;
            };
            if neighbor.outgoing_cost == 0 {
                log::debug!(
                    "[NWK] MTOR RREQ via 0x{:04X} has no reciprocal link cost",
                    prev_hop.0,
                );
                return;
            }
            let incoming_cost = crate::neighbor::link_cost_from_lqi(lqi);
            core::cmp::max(incoming_cost, neighbor.outgoing_cost)
        } else {
            self.neighbors
                .find_by_short(prev_hop)
                .map(|neighbor| {
                    let incoming_cost = crate::neighbor::link_cost_from_lqi(lqi);
                    if neighbor.outgoing_cost == 0 {
                        incoming_cost
                    } else {
                        core::cmp::max(incoming_cost, neighbor.outgoing_cost)
                    }
                })
                .unwrap_or(7)
        };
        let forward_cost = rreq.path_cost.saturating_add(link_cost);

        log::debug!(
            "[NWK] RREQ orig=0x{:04X} via 0x{:04X}: id={}, dst=0x{:04X}, cost={}→{}, m2o={}",
            originator.0,
            prev_hop.0,
            rreq.route_request_id,
            rreq.dst_addr.0,
            rreq.path_cost,
            forward_cost,
            is_many_to_one,
        );

        // One route discovery is identified by (originator, request ID) on
        // every hop it reaches. The broadcast transaction record cannot
        // distinguish alternate paths because all copies preserve the
        // originator's NWK sequence number. This record compares path cost and
        // immediate sender: R22 equal-cost alternate paths remain admissible,
        // while repeated MAC transmissions from one neighbor are suppressed.
        if !self
            .rreq_records
            .admit(originator, rreq.route_request_id, prev_hop, forward_cost)
        {
            log::debug!(
                "[NWK] RREQ id={} from 0x{:04X} via 0x{:04X} is not admissible (cost={})",
                rreq.route_request_id,
                originator.0,
                prev_hop.0,
                forward_cost,
            );
            return;
        }

        let our_addr = self.nib.network_address;

        // ── Many-to-one RREQ: install route to concentrator, forward, no RREP ──
        if is_many_to_one {
            let conc_type = concentrator_type.expect("validated MTOR option");

            // The route to the concentrator goes through the neighbour this
            // request was received from. Installing the RREQ's originator as
            // the next hop instead would name a device several hops away —
            // and, once the request is forwarded, our own upstream neighbour's
            // next hop would point back at us.
            if self
                .routing
                .update_route_many_to_one(originator, prev_hop, forward_cost, conc_type)
                .is_err()
            {
                log::warn!(
                    "[NWK] No routing-table capacity for MTOR concentrator 0x{:04X}",
                    originator.0,
                );
                self.rreq_records.remove(originator, rreq.route_request_id);
                return;
            }

            log::info!(
                "[NWK] Many-to-one route installed: concentrator=0x{:04X} via 0x{:04X} (cost={})",
                originator.0,
                prev_hop.0,
                forward_cost,
            );

            self.queue_rreq_forward(header, &rreq, forward_cost);
            return;
        }

        // ── Standard RREQ handling ──
        // If destination is us, or we have a route, we can reply. Decided
        // before the reverse route below is installed, so a request naming its
        // own originator as the destination cannot be "answered" out of the
        // entry this hop just created for it.
        let have_route =
            rreq.dst_addr == our_addr || self.routing.next_hop(rreq.dst_addr).is_some();

        // Reverse route toward the originator, used by the Route Reply on its
        // way back and by any later traffic to the originator.
        let _ = self
            .routing
            .update_route(originator, prev_hop, forward_cost);

        if have_route {
            // Record route discovery and complete it
            let _ = self.routing.add_discovery(crate::routing::RouteDiscovery {
                request_id: rreq.route_request_id,
                destination: rreq.dst_addr,
                sender: prev_hop,
                forward_cost,
                residual_cost: 0,
                timestamp: self.mac.monotonic_micros(),
                active: true,
            });
            self.routing.complete_discovery(rreq.route_request_id);

            // Queue RREP to be sent asynchronously back toward the RREQ
            // originator. It travels to the neighbour we heard the request
            // from, but it names the originator that started the discovery so
            // every hop on the way back can forward it further.
            let responder = if rreq.dst_addr == our_addr {
                our_addr
            } else {
                rreq.dst_addr
            };
            let _ = self.pending_route_replies.push(crate::PendingRouteReply {
                next_hop: prev_hop,
                originator,
                responder,
                path_cost: rreq.path_cost,
                route_request_id: rreq.route_request_id,
            });
            log::info!(
                "[NWK] RREQ destination 0x{:04X} reachable — RREP queued toward 0x{:04X}",
                rreq.dst_addr.0,
                prev_hop.0,
            );
        } else {
            // Router: record the discovery and carry the originator's request
            // one hop further.
            let _ = self.routing.add_discovery(crate::routing::RouteDiscovery {
                request_id: rreq.route_request_id,
                destination: rreq.dst_addr,
                sender: prev_hop,
                forward_cost,
                residual_cost: 0xFF,
                timestamp: self.mac.monotonic_micros(),
                active: true,
            });

            self.queue_rreq_forward(header, &rreq, forward_cost);
        }
    }

    /// Queue the originator's Route Request for one more hop.
    ///
    /// Refused for a device that may not forward at all, and for a request
    /// whose radius is exhausted: a frame received with radius 1 has reached
    /// the last hop the originator allowed, so forwarding it with radius 0
    /// would extend the flood past its bound.
    fn queue_rreq_forward(
        &mut self,
        header: &NwkHeader,
        rreq: &crate::frames::RouteRequest,
        path_cost: u8,
    ) {
        if !self.can_route() {
            return;
        }
        if is_unicast_address(header.dst_addr) {
            // Route discovery is flooded, never carried on as a unicast. A
            // request addressed to this device alone stops here rather than
            // being re-emitted as a broadcast still naming us as destination.
            log::debug!(
                "[NWK] RREQ addressed to 0x{:04X} is not propagated",
                header.dst_addr.0,
            );
            return;
        }
        if header.radius <= 1 {
            log::debug!(
                "[NWK] RREQ from 0x{:04X} not forwarded: radius {} exhausted",
                header.src_addr.0,
                header.radius,
            );
            return;
        }

        let queued = crate::QueuedRreqForward {
            frame_control: header.frame_control,
            dst_addr: header.dst_addr,
            originator: header.src_addr,
            src_ieee: header.src_ieee,
            seq_number: header.seq_number,
            radius: header.radius - 1,
            command_options: rreq.command_options,
            route_request_id: rreq.route_request_id,
            rreq_dst: rreq.dst_addr,
            rreq_dst_ieee: rreq.dst_ieee,
            path_cost,
        };

        log::debug!(
            "[NWK] Forwarding RREQ id={} for 0x{:04X} (src=0x{:04X} seq={} radius={} cost={})",
            queued.route_request_id,
            queued.rreq_dst.0,
            queued.originator.0,
            queued.seq_number,
            queued.radius,
            queued.path_cost,
        );

        if self.pending_rreq_forwards.push(queued).is_err() {
            log::warn!(
                "[NWK] RREQ forward queue full — dropping request id={} from 0x{:04X}",
                rreq.route_request_id,
                header.src_addr.0,
            );
        }
    }

    /// Handle incoming Route Reply (RREP).
    ///
    /// `prev_hop` is the neighbour that transmitted the reply — the next hop
    /// of the route being installed. The RREP names the originator that
    /// started the discovery, which is where the reply is forwarded on to.
    fn handle_route_reply(&mut self, prev_hop: ShortAddress, payload: &[u8]) {
        let Some(rrep) = crate::frames::RouteReply::parse(payload) else {
            log::warn!("[NWK] Malformed RREP from 0x{:04X}", prev_hop.0);
            return;
        };

        log::debug!(
            "[NWK] RREP from 0x{:04X}: id={}, orig=0x{:04X}, resp=0x{:04X}, cost={}",
            prev_hop.0,
            rrep.route_request_id,
            rrep.originator.0,
            rrep.responder.0,
            rrep.path_cost
        );

        // Update routing table: route to responder via the sender
        let _ = self
            .routing
            .update_route(rrep.responder, prev_hop, rrep.path_cost);

        // Complete the route discovery
        self.routing.complete_discovery(rrep.route_request_id);

        let our_addr = self.nib.network_address;

        if rrep.originator != our_addr {
            // Not the originator — forward RREP toward originator via routing
            let forward_hop = self
                .routing
                .next_hop(rrep.originator)
                .unwrap_or(self.nib.parent_address);
            let _ = self.pending_route_replies.push(crate::PendingRouteReply {
                next_hop: forward_hop,
                originator: rrep.originator,
                responder: rrep.responder,
                path_cost: rrep.path_cost,
                route_request_id: rrep.route_request_id,
            });
            log::debug!(
                "[NWK] Forwarding RREP toward originator 0x{:04X} via 0x{:04X}",
                rrep.originator.0,
                forward_hop.0,
            );
        } else {
            log::info!(
                "[NWK] Route discovered to 0x{:04X} via 0x{:04X} (cost={})",
                rrep.responder.0,
                prev_hop.0,
                rrep.path_cost
            );
        }
    }

    /// Handle incoming Route Record.
    ///
    /// This is the concentrator end of the path every intermediate router
    /// appended itself to (see [`NwkLayer::relay_route_record`]). The record
    /// lists the relays in the order the frame travelled — closest to the
    /// originating device first — and this device sends in the opposite
    /// direction. That is already the source-route wire order: the relay
    /// closest to the destination is first and the relay closest to this
    /// concentrator is last. The source-route index starts at that last entry,
    /// while the regular routing-table next hop is the same last relay.
    fn handle_route_record(&mut self, src: ShortAddress, payload: &[u8]) {
        if payload.is_empty() {
            log::warn!("[NWK] Malformed RouteRecord from 0x{:04X}", src.0);
            return;
        }

        let relay_count = payload[0] as usize;
        let expected_len = 1 + relay_count * 2;
        if payload.len() < expected_len {
            log::warn!(
                "[NWK] RouteRecord too short from 0x{:04X}: need {}, have {}",
                src.0,
                expected_len,
                payload.len()
            );
            return;
        }
        // A path longer than this device can store cannot be source-routed
        // over: keeping the first hops of it would send over a path whose tail
        // is missing. Refuse the record rather than install a truncated one.
        if relay_count > crate::routing::MAX_SOURCE_ROUTE_RELAYS {
            log::warn!(
                "[NWK] RouteRecord from 0x{:04X} lists {} relays, more than the {} \
                 this device can source-route over",
                src.0,
                relay_count,
                crate::routing::MAX_SOURCE_ROUTE_RELAYS,
            );
            return;
        }

        // Parse the full relay list from the payload
        let mut relay_list: heapless::Vec<
            ShortAddress,
            { crate::routing::MAX_SOURCE_ROUTE_RELAYS },
        > = heapless::Vec::new();
        for i in 0..relay_count {
            let offset = 1 + i * 2;
            let addr = u16::from_le_bytes([payload[offset], payload[offset + 1]]);
            let _ = relay_list.push(ShortAddress(addr));
        }

        log::debug!(
            "[NWK] RouteRecord from 0x{:04X}: {} relays travelled {:?}",
            src.0,
            relay_count,
            relay_list.as_slice(),
        );

        // Store the full relay path in the source route table (for concentrator TX)
        self.source_route_table.insert(src, relay_list.as_slice());

        // Also update the regular routing table with the relay closest to us.
        if let Some(first_hop) = relay_list.last().copied() {
            let _ = self.routing.update_route(src, first_hop, relay_count as u8);
        } else {
            // Direct neighbor, no relays
            let _ = self.routing.update_route(src, src, 0);
        }

        // R22 replaces source routes not only for the message source but also
        // for every intermediary relay. For relay `i`, the return path is the
        // suffix after that relay; an empty suffix means it is our direct
        // neighbor.
        for (index, relay) in relay_list.iter().copied().enumerate() {
            let suffix = &relay_list.as_slice()[index + 1..];
            self.source_route_table.insert(relay, suffix);
            let next_hop = suffix.last().copied().unwrap_or(relay);
            let _ = self
                .routing
                .update_route(relay, next_hop, suffix.len() as u8);
        }
    }

    /// Handle incoming Link Status command.
    fn handle_link_status(&mut self, src: ShortAddress, payload: &[u8]) {
        let Some(ls) = crate::frames::LinkStatusCommand::parse(payload) else {
            log::warn!("[NWK] Malformed LinkStatus from 0x{:04X}", src.0);
            return;
        };

        log::debug!(
            "[NWK] LinkStatus from 0x{:04X}: {} entries",
            src.0,
            ls.entries.len()
        );

        // Check if any entry references us, and update the neighbor's cost
        let our_addr = self.nib.network_address;
        for entry in &ls.entries {
            if entry.address == our_addr {
                // This neighbor reports its cost to/from us
                if let Some(neighbor) = self.neighbors.find_by_short_mut(src) {
                    neighbor.outgoing_cost = entry.incoming_cost.clamp(1, 7);
                    log::debug!(
                        "[NWK] Updated link cost to 0x{:04X}: outgoing={}",
                        src.0,
                        neighbor.outgoing_cost
                    );
                }
                break;
            }
        }
    }

    /// Handle incoming Network Status command (route error notification).
    fn handle_network_status(&mut self, src: ShortAddress, payload: &[u8]) {
        let Some(ns) = crate::frames::NetworkStatusCommand::parse(payload) else {
            log::warn!("[NWK] Malformed NetworkStatus from 0x{:04X}", src.0);
            return;
        };

        log::info!(
            "[NWK] NetworkStatus from 0x{:04X}: code={} dst=0x{:04X}",
            src.0,
            ns.status_code,
            ns.destination.0,
        );

        match ns.status_code {
            crate::frames::NetworkStatusCommand::NO_ROUTE_AVAILABLE
            | crate::frames::NetworkStatusCommand::TREE_LINK_FAILURE
            | crate::frames::NetworkStatusCommand::NON_TREE_LINK_FAILURE => {
                self.routing.remove(ns.destination);
            }
            crate::frames::NetworkStatusCommand::SOURCE_ROUTE_FAILURE
            | crate::frames::NetworkStatusCommand::MANY_TO_ONE_ROUTE_FAILURE => {
                self.source_route_table.remove(ns.destination);
                self.routing.remove(ns.destination);
            }
            _ => {}
        }
    }
}

/// Process a source route relay list to determine the next hop.
///
/// The relay closest to the destination is `relay_list[0]` and the relay
/// closest to the originator is the last entry. A Route Record arrives in
/// exactly that order for traffic sent back toward its originating device.
///
/// `relay_index` names the next relay. The originator initializes it to
/// `relay_count - 1`; each receiving relay decrements it before forwarding.
/// A relay that receives index zero forwards directly to the destination.
///
/// Returns `(next_hop, new_relay_index)`.
fn process_source_route(
    sr: &crate::frames::SourceRoute,
    our_addr: ShortAddress,
    dst_addr: ShortAddress,
) -> Result<(ShortAddress, u8), NwkStatus> {
    let idx = sr.relay_index as usize;

    // An index past the end cannot name a relay in this subframe.
    if idx >= sr.relay_list.len() {
        log::warn!(
            "[NWK] Source route relay_index {} out of bounds (len={})",
            idx,
            sr.relay_list.len(),
        );
        return Err(NwkStatus::InvalidParameter);
    }

    // Index zero is the final relay operation: the destination is next and
    // the index remains zero on that last hop.
    if idx == 0 {
        return Ok((dst_addr, 0));
    }

    // At every earlier hop the indexed relay must be this device. Searching
    // the list and resuming from another position is not a Zigbee operation
    // and can skip or repeat relays.
    if sr.relay_list[idx] != our_addr {
        log::warn!(
            "[NWK] Source route index {} names 0x{:04X}, not us 0x{:04X}",
            idx,
            sr.relay_list[idx].0,
            our_addr.0,
        );
        return Err(NwkStatus::InvalidParameter);
    }

    let next_index = idx - 1;
    Ok((sr.relay_list[next_index], next_index as u8))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{
        ED_TIMEOUT_ENUM_DEFAULT, ED_TIMEOUT_ENUM_REQUESTED, NwkFrameControl, NwkFrameType,
        NwkHeader,
    };
    use crate::{DeviceType, NwkLayer};
    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use std::sync::Arc;
    use std::task::Wake;
    #[cfg(feature = "router")]
    use zigbee_mac::CapabilityInfo;
    #[cfg(feature = "router")]
    use zigbee_mac::PlatformServices;
    use zigbee_mac::mock::MockMac;
    #[cfg(feature = "router")]
    use zigbee_types::IeeeAddress;
    use zigbee_types::{MacAddress, PanId, ShortAddress};

    const OUR_ADDR: ShortAddress = ShortAddress(0x1111);
    const PEER: ShortAddress = ShortAddress(0x3333);
    const FAR: ShortAddress = ShortAddress(0x2222);
    const NEXT_HOP: ShortAddress = ShortAddress(0x4444);
    const PAN: PanId = PanId(0x1234);

    struct NoopWake;

    impl Wake for NoopWake {
        fn wake(self: Arc<Self>) {}
    }

    /// Minimal executor: every MockMac primitive completes without yielding.
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

    fn node(device_type: DeviceType, addr: ShortAddress) -> NwkLayer<MockMac> {
        let mut nwk = NwkLayer::new(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]), device_type);
        nwk.joined = true;
        nwk.nib.network_address = addr;
        nwk.nib.pan_id = PAN;
        nwk.nib.parent_address = ShortAddress(0x0000);
        nwk.nib.security_enabled = false;
        nwk
    }

    fn frame(frame_type: NwkFrameType, src: ShortAddress, dst: ShortAddress) -> NwkHeader {
        NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: frame_type as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: false,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: false,
            },
            dst_addr: dst,
            src_addr: src,
            radius: 5,
            seq_number: 7,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        }
    }

    fn encode(header: &NwkHeader, payload: &[u8], buf: &mut [u8; 128]) -> usize {
        let hdr_len = header.serialize(buf);
        buf[hdr_len..hdr_len + payload.len()].copy_from_slice(payload);
        hdr_len + payload.len()
    }

    fn command_payload(id: NwkCommandId, body: &[u8], out: &mut [u8; 32]) -> usize {
        out[0] = id as u8;
        out[1..1 + body.len()].copy_from_slice(body);
        1 + body.len()
    }

    fn tx_short_dst(record: &zigbee_mac::mock::TxRecord) -> Option<ShortAddress> {
        match record.dst {
            MacAddress::Short(_, addr) => Some(addr),
            _ => None,
        }
    }

    /// Returns `(nwk_header, first payload byte)` of a recorded transmission.
    #[cfg(feature = "router")]
    fn tx_frame(record: &zigbee_mac::mock::TxRecord) -> (NwkHeader, u8) {
        let bytes = record.payload.as_slice();
        let (header, consumed) = NwkHeader::parse(bytes).expect("recorded frame parses");
        (header, bytes[consumed])
    }

    // ── Secured multi-hop helpers ────────────────────────────

    #[cfg(feature = "router")]
    const NETWORK_KEY: crate::security::AesKey = [0x21; 16];
    #[cfg(feature = "router")]
    const KEY_SEQ: u8 = 3;
    #[cfg(feature = "router")]
    const ORIGIN: ShortAddress = ShortAddress(0x7777);
    #[cfg(feature = "router")]
    const ORIGIN_IEEE: IeeeAddress = [0xA0; 8];
    #[cfg(feature = "router")]
    const RELAY_IEEE: IeeeAddress = [0xB0; 8];
    #[cfg(feature = "router")]
    const DEST_IEEE: IeeeAddress = [0xC0; 8];
    /// Floor of the relaying device's durable outgoing-counter reservation.
    #[cfg(feature = "router")]
    const RESERVED_FLOOR: u32 = 0x0100;

    /// A joined device on a NWK-secured network holding the shared key.
    #[cfg(feature = "router")]
    fn secured_node(
        device_type: DeviceType,
        addr: ShortAddress,
        ieee: IeeeAddress,
    ) -> NwkLayer<MockMac> {
        let mut nwk = node(device_type, addr);
        nwk.nib.ieee_address = ieee;
        nwk.nib.security_enabled = true;
        nwk.nib.active_key_seq_number = KEY_SEQ;
        nwk.security.set_network_key(NETWORK_KEY, KEY_SEQ);
        nwk
    }

    /// Copy a recorded transmission out of the mock MAC history.
    #[cfg(feature = "router")]
    fn recorded_frame(nwk: &NwkLayer<MockMac>, index: usize) -> heapless::Vec<u8, 128> {
        heapless::Vec::from_slice(nwk.mac.tx_history()[index].payload.as_slice())
            .expect("recorded frame fits")
    }

    /// The exact bytes an originator puts on air for `dst`, produced by the
    /// real transmit path rather than by a hand-rolled test encoder.
    #[cfg(feature = "router")]
    fn secured_frame_on_air(dst: ShortAddress, payload: &[u8]) -> heapless::Vec<u8, 128> {
        let mut origin = secured_node(DeviceType::Router, ORIGIN, ORIGIN_IEEE);
        if dst.0 < 0xFFF8 {
            // Reach the destination through the device under test.
            origin.routing.update_route(dst, OUR_ADDR, 1).unwrap();
        }
        block_on(origin.nlde_data_request(dst, 5, payload, true, false))
            .expect("the originator secures and sends the frame");
        recorded_frame(&origin, 0)
    }

    // ── Route Request propagation helpers ────────────────────

    /// The concentrator that originates many-to-one Route Requests.
    #[cfg(feature = "router")]
    const CONCENTRATOR: ShortAddress = ShortAddress(0x0000);

    #[cfg(feature = "router")]
    fn standard_rreq(id: u8, dst: ShortAddress, path_cost: u8) -> crate::frames::RouteRequest {
        crate::frames::RouteRequest {
            command_options: 0x00,
            route_request_id: id,
            dst_addr: dst,
            path_cost,
            dst_ieee: None,
        }
    }

    /// A many-to-one Route Request from a low-RAM concentrator (bit 3 set).
    #[cfg(feature = "router")]
    fn many_to_one_rreq(id: u8, path_cost: u8) -> crate::frames::RouteRequest {
        crate::frames::RouteRequest {
            command_options: crate::routing::ConcentratorType::LowRam.rreq_options(),
            route_request_id: id,
            dst_addr: ShortAddress(0xFFFC),
            path_cost,
            dst_ieee: None,
        }
    }

    /// The bytes a Route Request travels in: an unsecured NWK command
    /// broadcast whose header names the *originator*, not the transmitter.
    #[cfg(feature = "router")]
    fn rreq_on_air(
        originator: ShortAddress,
        seq: u8,
        radius: u8,
        rreq: &crate::frames::RouteRequest,
    ) -> heapless::Vec<u8, 128> {
        let destination = if ((rreq.command_options >> 3) & 0x03) != 0 {
            ShortAddress(0xFFFC)
        } else {
            ShortAddress::BROADCAST
        };
        let mut header = frame(NwkFrameType::Command, originator, destination);
        header.seq_number = seq;
        header.radius = radius;
        let mut body = [0u8; 16];
        let body_len = rreq.serialize(&mut body);
        let mut payload = [0u8; 32];
        let payload_len =
            command_payload(NwkCommandId::RouteRequest, &body[..body_len], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(&header, &payload[..payload_len], &mut buf);
        heapless::Vec::from_slice(&buf[..len]).expect("the frame fits")
    }

    /// Header and Route Request carried by an unsecured recorded transmission.
    #[cfg(feature = "router")]
    fn parse_rreq(bytes: &[u8]) -> (NwkHeader, crate::frames::RouteRequest) {
        let (header, consumed) = NwkHeader::parse(bytes).expect("the forwarded frame parses");
        assert_eq!(
            bytes[consumed],
            NwkCommandId::RouteRequest as u8,
            "the forwarded frame is a Route Request"
        );
        let rreq =
            crate::frames::RouteRequest::parse(&bytes[consumed + 1..]).expect("the RREQ parses");
        (header, rreq)
    }

    /// Decrypt a recorded secured NWK frame with the shared network key.
    ///
    /// Returns `(header, auxiliary header, plaintext NWK payload)`.
    #[cfg(feature = "router")]
    fn decrypt_recorded(
        bytes: &[u8],
    ) -> (
        NwkHeader,
        crate::security::NwkSecurityHeader,
        heapless::Vec<u8, MAX_NWK_FRAME>,
    ) {
        let (header, consumed) = NwkHeader::parse(bytes).expect("the frame parses");
        let (aux, aux_len) = crate::security::NwkSecurityHeader::parse(&bytes[consumed..])
            .expect("the frame carries an auxiliary header");
        let aad_len = consumed + aux_len;
        let mut aad = [0u8; MAX_NWK_AAD];
        aad[..aad_len].copy_from_slice(&bytes[..aad_len]);
        aad[consumed] = (aad[consumed] & !0x07) | 0x05;
        let plaintext = crate::security::NwkSecurity::new()
            .decrypt(&aad[..aad_len], &bytes[aad_len..], &NETWORK_KEY, &aux)
            .expect("the recorded frame decrypts under the shared network key");
        (header, aux, plaintext)
    }

    /// Relay list carried by a Route Record command payload.
    #[cfg(feature = "router")]
    fn route_record_relays(payload: &[u8]) -> heapless::Vec<ShortAddress, 8> {
        assert_eq!(payload[0], NwkCommandId::RouteRecord as u8);
        let count = payload[1] as usize;
        assert_eq!(payload.len(), 2 + count * 2);
        let mut relays = heapless::Vec::new();
        for i in 0..count {
            let offset = 2 + i * 2;
            relays
                .push(ShortAddress(u16::from_le_bytes([
                    payload[offset],
                    payload[offset + 1],
                ])))
                .expect("the recorded path fits");
        }
        relays
    }

    /// Copy the next unread transmission out of a router's MAC history.
    #[cfg(feature = "router")]
    fn next_transmission(
        nwk: &NwkLayer<MockMac>,
        seen: &mut usize,
    ) -> Option<heapless::Vec<u8, 128>> {
        let history = nwk.mac.tx_history();
        if history.len() <= *seen {
            return None;
        }
        let bytes =
            heapless::Vec::from_slice(history[*seen].payload.as_slice()).expect("the frame fits");
        *seen += 1;
        Some(bytes)
    }

    /// Register a neighbour with an explicit link cost.
    #[cfg(feature = "router")]
    fn neighbour_with_cost(nwk: &mut NwkLayer<MockMac>, addr: ShortAddress, cost: u8) {
        nwk.update_neighbor_address(addr, [addr.0 as u8; 8]);
        let neighbor = nwk
            .neighbors
            .find_by_short_mut(addr)
            .expect("the neighbour was just added");
        neighbor.device_type = crate::neighbor::NeighborDeviceType::Router;
        neighbor.outgoing_cost = cost;
        neighbor.lqi = match cost {
            1 => 220,
            2 => 180,
            3 => 125,
            5 => 75,
            _ => 25,
        };
    }

    #[test]
    #[cfg(feature = "router")]
    fn concentrator_options_and_mtor_destinations_match_r22() {
        assert_eq!(
            crate::routing::ConcentratorType::HighRam.rreq_options(),
            0x08
        );
        assert_eq!(
            crate::routing::ConcentratorType::LowRam.rreq_options(),
            0x10
        );
        assert_eq!(
            crate::routing::ConcentratorType::from_rreq_options(0x08),
            Some(crate::routing::ConcentratorType::HighRam)
        );
        assert_eq!(
            crate::routing::ConcentratorType::from_rreq_options(0x10),
            Some(crate::routing::ConcentratorType::LowRam)
        );
        assert_eq!(
            crate::routing::ConcentratorType::from_rreq_options(0x18),
            None,
            "many-to-one value three is reserved"
        );

        for (concentrator_type, expected_options) in [
            (crate::routing::ConcentratorType::HighRam, 0x08),
            (crate::routing::ConcentratorType::LowRam, 0x10),
        ] {
            let mut concentrator = node(DeviceType::Coordinator, CONCENTRATOR);
            concentrator.start_concentrator(concentrator_type, 0, 5);
            block_on(concentrator.process_pending_routing());

            assert_eq!(
                concentrator.mac.tx_history().len(),
                1,
                "an originated MTOR request is transmitted once"
            );
            assert_eq!(
                tx_short_dst(&concentrator.mac.tx_history()[0]),
                Some(ShortAddress::BROADCAST)
            );
            let (header, rreq) = parse_rreq(concentrator.mac.tx_history()[0].payload.as_slice());
            assert_eq!(header.dst_addr, ShortAddress(0xFFFC));
            assert_eq!(rreq.dst_addr, ShortAddress(0xFFFC));
            assert_eq!(rreq.command_options, expected_options);

            concentrator.tick_router_maintenance(1);
            block_on(concentrator.process_pending_routing());
            assert_eq!(
                concentrator.mac.tx_history().len(),
                1,
                "discovery time zero is startup/explicit only"
            );
        }
    }

    #[test]
    #[cfg(feature = "router")]
    fn startup_mtor_remains_pending_until_the_concentrator_is_joined() {
        let mut concentrator = node(DeviceType::Coordinator, CONCENTRATOR);
        concentrator.joined = false;
        concentrator.start_concentrator(crate::routing::ConcentratorType::HighRam, 0, 5);

        block_on(concentrator.process_pending_routing());
        assert!(concentrator.mac.tx_history().is_empty());

        concentrator.joined = true;
        block_on(concentrator.process_pending_routing());
        assert_eq!(concentrator.mac.tx_history().len(), 1);
    }

    #[test]
    #[cfg(feature = "router")]
    fn mtor_uses_the_worse_of_incoming_and_outgoing_link_costs() {
        for (outgoing_cost, lqi, expected_link_cost) in [(5, 180, 5), (2, 75, 5), (1, 220, 1)] {
            let mut router = node(DeviceType::Router, OUR_ADDR);
            neighbour_with_cost(&mut router, CONCENTRATOR, outgoing_cost);
            let on_air = rreq_on_air(CONCENTRATOR, 0x31, 5, &many_to_one_rreq(4, 3));

            assert!(
                block_on(router.process_incoming_nwk_frame_from(&on_air, lqi, Some(CONCENTRATOR),))
                    .is_none()
            );
            assert_eq!(
                router
                    .routing
                    .get_entry(CONCENTRATOR)
                    .expect("the MTOR route is installed")
                    .path_cost,
                3 + expected_link_cost
            );
        }
    }

    #[test]
    #[cfg(feature = "router")]
    fn mtor_requires_a_known_nonzero_reciprocal_link_cost() {
        let mut router = node(DeviceType::Router, OUR_ADDR);
        router.update_neighbor_address(CONCENTRATOR, [0xC0; 8]);
        let neighbor = router.neighbors.find_by_short_mut(CONCENTRATOR).unwrap();
        neighbor.device_type = crate::neighbor::NeighborDeviceType::Router;
        neighbor.outgoing_cost = 0;
        let on_air = rreq_on_air(CONCENTRATOR, 0x32, 5, &many_to_one_rreq(5, 0));

        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(CONCENTRATOR),))
                .is_none()
        );
        assert!(router.routing.get_entry(CONCENTRATOR).is_none());
        assert!(router.pending_rreq_forwards.is_empty());
    }

    #[test]
    #[cfg(feature = "router")]
    fn reserved_or_malformed_mtor_requests_are_not_installed_or_forwarded() {
        let mut router = node(DeviceType::Router, OUR_ADDR);
        neighbour_with_cost(&mut router, CONCENTRATOR, 1);

        let mut reserved = many_to_one_rreq(6, 0);
        reserved.command_options = 0x18;
        let reserved = rreq_on_air(CONCENTRATOR, 0x33, 5, &reserved);
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&reserved, 255, Some(CONCENTRATOR),))
                .is_none()
        );

        let mut malformed = many_to_one_rreq(7, 0);
        malformed.dst_addr = CONCENTRATOR;
        let malformed = rreq_on_air(CONCENTRATOR, 0x34, 5, &malformed);
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&malformed, 255, Some(CONCENTRATOR),))
                .is_none()
        );

        assert!(router.routing.get_entry(CONCENTRATOR).is_none());
        assert!(router.pending_rreq_forwards.is_empty());
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_full_routing_table_prevents_mtor_propagation() {
        let mut router = node(DeviceType::Router, OUR_ADDR);
        for index in 0..crate::routing::MAX_ROUTES {
            router
                .routing
                .update_route(ShortAddress(0x1000 + index as u16), NEXT_HOP, index as u8)
                .unwrap();
        }
        neighbour_with_cost(&mut router, CONCENTRATOR, 1);
        let on_air = rreq_on_air(CONCENTRATOR, 0x35, 5, &many_to_one_rreq(8, 0));

        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(CONCENTRATOR),))
                .is_none()
        );
        assert!(router.routing.get_entry(CONCENTRATOR).is_none());
        assert!(
            router.pending_rreq_forwards.is_empty(),
            "a request without routing-table state must not be propagated"
        );
        assert_eq!(
            router.rreq_records.recorded_cost(CONCENTRATOR, 8),
            None,
            "routing capacity requires both the discovery record and route entry"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_full_route_request_table_refuses_another_discovery() {
        let mut router = node(DeviceType::Router, OUR_ADDR);
        for index in 0..crate::routing::MAX_RREQ_RECORDS {
            assert!(router.rreq_records.admit(
                ShortAddress(0x1000 + index as u16),
                index as u8,
                PEER,
                1,
            ));
        }

        assert!(
            !router.rreq_records.admit(CONCENTRATOR, 0xA5, NEXT_HOP, 1),
            "a live discovery record must not be evicted to admit a ninth request"
        );
        assert_eq!(router.rreq_records.recorded_cost(CONCENTRATOR, 0xA5), None);
    }

    #[test]
    #[cfg(feature = "router")]
    fn route_and_source_route_ages_use_elapsed_seconds() {
        let mut router = node(DeviceType::Router, OUR_ADDR);
        router.routing.update_route(FAR, NEXT_HOP, 1).unwrap();
        router.source_route_table.insert(PEER, &[NEXT_HOP]);

        router.tick_router_maintenance(7);

        assert_eq!(router.routing.get_entry(FAR).unwrap().age, 7);
        assert_eq!(router.source_route_table.entry_age(PEER), Some(7));
    }

    #[test]
    #[cfg(feature = "router")]
    fn high_ram_routes_rearm_route_record_only_when_the_path_changes() {
        let mut routing = crate::routing::RoutingTable::new();
        routing
            .update_route_many_to_one(
                CONCENTRATOR,
                PEER,
                2,
                crate::routing::ConcentratorType::HighRam,
            )
            .unwrap();
        routing.clear_route_record_required(CONCENTRATOR);
        assert!(
            !routing
                .get_entry(CONCENTRATOR)
                .unwrap()
                .route_record_required
        );

        routing
            .update_route_many_to_one(
                CONCENTRATOR,
                PEER,
                2,
                crate::routing::ConcentratorType::HighRam,
            )
            .unwrap();
        assert!(
            !routing
                .get_entry(CONCENTRATOR)
                .unwrap()
                .route_record_required
        );

        routing
            .update_route_many_to_one(
                CONCENTRATOR,
                NEXT_HOP,
                1,
                crate::routing::ConcentratorType::HighRam,
            )
            .unwrap();
        assert!(
            routing
                .get_entry(CONCENTRATOR)
                .unwrap()
                .route_record_required
        );
    }

    // ── Local delivery versus relay ──────────────────────────

    #[test]
    fn tick_end_device_maintenance_ages_the_neighbour_cache() {
        // The role-independent common maintenance path ages the neighbour cache
        // once per elapsed second — the LRU input a non-routing end device
        // still needs — without touching any router maintenance state.
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        nwk.update_neighbor_address(PEER, [0xAB; 8]);
        assert_eq!(
            nwk.neighbor_table().find_by_ieee(&[0xAB; 8]).unwrap().age,
            0
        );

        nwk.tick_end_device_maintenance(7);

        assert_eq!(
            nwk.neighbor_table().find_by_ieee(&[0xAB; 8]).unwrap().age,
            7,
            "each elapsed second ages the neighbour cache exactly once"
        );
    }

    #[test]
    fn unicast_addressed_to_us_is_delivered_locally_and_not_relayed() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Data, PEER, OUR_ADDR),
            &[0xAB, 0xCD],
            &mut buf,
        );

        let indication = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));

        match indication {
            Some(NwkIndication::Borrowed(data)) => {
                assert_eq!(data.src_addr, PEER);
                assert_eq!(data.dst_addr, OUR_ADDR);
                assert_eq!(data.payload, &[0xAB, 0xCD]);
                assert_eq!(data.lqi, 42);
                assert!(!data.security_use);
            }
            other => panic!("expected local delivery, got {other:?}"),
        }
        assert!(
            nwk.mac.tx_history().is_empty(),
            "a frame addressed to us must never be relayed"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn unicast_for_another_device_is_relayed_with_decremented_radius() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.routing.update_route(FAR, NEXT_HOP, 1).unwrap();
        let mut buf = [0u8; 128];
        let len = encode(&frame(NwkFrameType::Data, PEER, FAR), &[0x01], &mut buf);

        let indication = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));

        assert!(
            indication.is_none(),
            "relayed frames are not delivered locally"
        );
        assert_eq!(nwk.mac.tx_history().len(), 1);
        let record = &nwk.mac.tx_history()[0];
        assert_eq!(tx_short_dst(record), Some(NEXT_HOP));
        let (relayed, _) = tx_frame(record);
        assert_eq!(relayed.dst_addr, FAR);
        assert_eq!(relayed.src_addr, PEER, "relay must preserve the originator");
        assert_eq!(relayed.radius, 4);
    }

    #[test]
    fn end_devices_never_relay_traffic_for_other_devices() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        nwk.routing.update_route(FAR, NEXT_HOP, 1).ok();
        let mut buf = [0u8; 128];
        let len = encode(&frame(NwkFrameType::Data, PEER, FAR), &[0x01], &mut buf);

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert!(nwk.mac.tx_history().is_empty());
        assert!(!nwk.can_route());
    }

    #[test]
    fn self_originated_frames_are_dropped_before_relay() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Data, OUR_ADDR, ShortAddress::BROADCAST),
            &[0x01],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert!(
            nwk.mac.tx_history().is_empty(),
            "our own broadcast must not be echoed back into the network"
        );
    }

    #[test]
    fn inter_pan_frames_are_not_handed_to_the_upper_layer() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::InterPan, PEER, OUR_ADDR),
            &[0x01],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
    }

    /// The 2-bit frame type field has one value with no meaning in this
    /// stack (0b10). It parses like any other header, so it has to be
    /// refused explicitly.
    #[cfg(feature = "router")]
    const RESERVED_FRAME_TYPE: u8 = 0b10;

    #[test]
    #[cfg(feature = "router")]
    fn inter_pan_and_reserved_unicasts_are_dropped_before_the_relay() {
        for frame_type in [NwkFrameType::InterPan as u8, RESERVED_FRAME_TYPE] {
            let mut nwk = node(DeviceType::Router, OUR_ADDR);
            nwk.routing.update_route(FAR, NEXT_HOP, 1).unwrap();
            let mut header = frame(NwkFrameType::Data, PEER, FAR);
            header.frame_control.frame_type = frame_type;
            let mut buf = [0u8; 128];
            let len = encode(&header, &[0x42], &mut buf);

            assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
            assert!(
                nwk.mac.tx_history().is_empty(),
                "frame type {frame_type} must never be relayed"
            );

            // Control: the very same setup does relay a NWK Data frame, so
            // the silence above is the frame type check and not the route.
            let len = encode(&frame(NwkFrameType::Data, PEER, FAR), &[0x42], &mut buf);
            assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
            assert_eq!(nwk.mac.tx_history().len(), 1);
        }
    }

    #[test]
    #[cfg(feature = "router")]
    fn inter_pan_and_reserved_broadcasts_are_neither_recorded_nor_rebroadcast() {
        for frame_type in [NwkFrameType::InterPan as u8, RESERVED_FRAME_TYPE] {
            let mut nwk = node(DeviceType::Router, OUR_ADDR);
            let mut header = frame(NwkFrameType::Data, PEER, ShortAddress::BROADCAST);
            header.frame_control.frame_type = frame_type;
            let mut buf = [0u8; 128];
            let len = encode(&header, &[0x55], &mut buf);

            assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
            assert!(
                nwk.mac.tx_history().is_empty(),
                "frame type {frame_type} must never be rebroadcast"
            );

            // The BTR was not poisoned: a genuine Data broadcast carrying the
            // same source and sequence number is still accepted and relayed.
            let genuine = frame(NwkFrameType::Data, PEER, ShortAddress::BROADCAST);
            assert_eq!(genuine.seq_number, header.seq_number);
            let len = encode(&genuine, &[0x55], &mut buf);
            assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_some());
            assert_eq!(nwk.mac.tx_history().len(), 1);
        }
    }

    // ── Broadcast eligibility and BTR ────────────────────────

    #[test]
    #[cfg(feature = "router")]
    fn source_routed_frames_follow_the_relay_list() {
        const NEXT_RELAY: ShortAddress = ShortAddress(0x5555);

        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let mut header = frame(NwkFrameType::Data, PEER, FAR);
        header.frame_control.source_route = true;
        // Ordered destination-nearest first. The index points at this device,
        // so forwarding decrements it to the next relay toward the destination.
        let mut relay_list = heapless::Vec::new();
        relay_list.push(NEXT_RELAY).unwrap();
        relay_list.push(OUR_ADDR).unwrap();
        header.source_route = Some(crate::frames::SourceRoute {
            relay_count: 2,
            relay_index: 1,
            relay_list,
        });
        let mut buf = [0u8; 128];
        let len = encode(&header, &[0x42], &mut buf);

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert_eq!(nwk.mac.tx_history().len(), 1);
        let record = &nwk.mac.tx_history()[0];
        assert_eq!(tx_short_dst(record), Some(NEXT_RELAY));
        let (relayed, _) = tx_frame(record);
        assert_eq!(relayed.radius, 4);
        assert_eq!(
            relayed
                .source_route
                .expect("relay list is preserved")
                .relay_index,
            0,
            "the relay index decrements one hop toward the destination"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn broadcast_is_rebroadcast_once_and_duplicates_are_suppressed() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Data, PEER, ShortAddress::BROADCAST),
            &[0x55],
            &mut buf,
        );

        let first = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));
        assert!(matches!(first, Some(NwkIndication::Borrowed(_))));
        assert_eq!(
            nwk.mac.tx_history().len(),
            1,
            "broadcast is rebroadcast once"
        );
        let (rebroadcast, _) = tx_frame(&nwk.mac.tx_history()[0]);
        assert_eq!(rebroadcast.radius, 4);
        assert_eq!(
            tx_short_dst(&nwk.mac.tx_history()[0]),
            Some(ShortAddress::BROADCAST)
        );

        let second = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));
        assert!(
            second.is_none(),
            "BTR must suppress the duplicate broadcast"
        );
        assert_eq!(
            nwk.mac.tx_history().len(),
            1,
            "a duplicate broadcast must not be rebroadcast again"
        );
    }

    #[test]
    fn broadcast_with_expired_radius_is_delivered_but_not_rebroadcast() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let mut header = frame(NwkFrameType::Data, PEER, ShortAddress::BROADCAST);
        header.radius = 1;
        let mut buf = [0u8; 128];
        let len = encode(&header, &[0x55], &mut buf);

        let indication = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));

        assert!(matches!(indication, Some(NwkIndication::Borrowed(_))));
        assert!(nwk.mac.tx_history().is_empty());
    }

    #[test]
    fn router_broadcast_reaches_routers_but_not_end_devices() {
        const ALL_ROUTERS: ShortAddress = ShortAddress(0xFFFC);

        let mut router = node(DeviceType::Router, OUR_ADDR);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Data, PEER, ALL_ROUTERS),
            &[0x77],
            &mut buf,
        );
        assert!(matches!(
            block_on(router.process_incoming_nwk_frame(&buf[..len], 42)),
            Some(NwkIndication::Borrowed(_))
        ));

        let mut coordinator = node(DeviceType::Coordinator, ShortAddress(0x0000));
        assert!(matches!(
            block_on(coordinator.process_incoming_nwk_frame(&buf[..len], 42)),
            Some(NwkIndication::Borrowed(_))
        ));

        let mut end_device = node(DeviceType::EndDevice, OUR_ADDR);
        assert!(
            block_on(end_device.process_incoming_nwk_frame(&buf[..len], 42)).is_none(),
            "0xFFFC is addressed to routers and the coordinator only"
        );
    }

    #[test]
    fn rx_on_when_idle_broadcast_is_delivered_to_end_devices() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        nwk.set_rx_on_when_idle(false);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(
                NwkFrameType::Data,
                PEER,
                ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            ),
            &[0x99],
            &mut buf,
        );

        assert!(
            matches!(
                block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)),
                Some(NwkIndication::Borrowed(_))
            ),
            "sleepy sensors receive 0xFFFD broadcasts from the parent's indirect queue"
        );
    }

    // ── NWK command dispatch ─────────────────────────────────

    #[test]
    #[cfg(feature = "router")]
    fn route_request_for_us_queues_a_route_reply_that_is_sent_on_maintenance() {
        use crate::frames::RouteRequest;

        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let rreq = RouteRequest {
            command_options: 0x00,
            route_request_id: 9,
            dst_addr: OUR_ADDR,
            path_cost: 3,
            dst_ieee: None,
        };
        let mut body = [0u8; 16];
        let body_len = rreq.serialize(&mut body);
        let mut payload = [0u8; 32];
        let payload_len =
            command_payload(NwkCommandId::RouteRequest, &body[..body_len], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, ShortAddress::BROADCAST),
            &payload[..payload_len],
            &mut buf,
        );

        let indication = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));
        assert!(
            indication.is_none(),
            "routing commands are consumed by the NWK layer"
        );
        assert_eq!(nwk.pending_route_replies.len(), 1);
        assert_eq!(
            nwk.routing.next_hop(PEER),
            Some(PEER),
            "the reverse route to the originator is installed"
        );

        nwk.mac.clear_tx_history();
        block_on(nwk.process_pending_routing());

        let replies: std::vec::Vec<_> = nwk
            .mac
            .tx_history()
            .iter()
            .filter(|r| tx_short_dst(r) == Some(PEER))
            .map(tx_frame)
            .filter(|(header, cmd)| {
                header.frame_control.frame_type == NwkFrameType::Command as u8
                    && *cmd == NwkCommandId::RouteReply as u8
            })
            .collect();
        assert_eq!(
            replies.len(),
            1,
            "exactly one RREP is unicast to 0x{:04X}",
            PEER.0
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn route_reply_installs_the_discovered_route() {
        use crate::frames::RouteReply;

        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let rrep = RouteReply {
            command_options: 0x00,
            route_request_id: 9,
            originator: OUR_ADDR,
            responder: FAR,
            path_cost: 4,
            originator_ieee: None,
            responder_ieee: None,
        };
        let mut body = [0u8; 32];
        let body_len = rrep.serialize(&mut body);
        let mut payload = [0u8; 32];
        let payload_len =
            command_payload(NwkCommandId::RouteReply, &body[..body_len], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, OUR_ADDR),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert_eq!(nwk.routing.next_hop(FAR), Some(PEER));
        assert!(
            nwk.pending_route_replies.is_empty(),
            "a reply addressed to us is not forwarded further"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn network_status_removes_the_failed_route() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.routing.update_route(FAR, NEXT_HOP, 1).unwrap();
        let ns = crate::frames::NetworkStatusCommand {
            status_code: crate::frames::NetworkStatusCommand::NO_ROUTE_AVAILABLE,
            destination: FAR,
        };
        let mut body = [0u8; 4];
        let body_len = ns.serialize(&mut body);
        let mut payload = [0u8; 32];
        let payload_len =
            command_payload(NwkCommandId::NetworkStatus, &body[..body_len], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, OUR_ADDR),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert_eq!(nwk.routing.next_hop(FAR), None);
        assert_eq!(
            nwk.take_command_outcome(),
            None,
            "routing maintenance stays inside the NWK layer"
        );
    }

    #[test]
    fn leave_request_from_the_parent_is_reported_to_the_caller() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        let mut payload = [0u8; 32];
        let leave = crate::frames::LeaveCommand {
            remove_children: false,
            request: true,
            rejoin: true,
        };
        let payload_len = command_payload(NwkCommandId::Leave, &[leave.serialize()], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, ShortAddress(0x0000), OUR_ADDR),
            &payload[..payload_len],
            &mut buf,
        );

        let indication = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));

        assert!(
            indication.is_none(),
            "a NWK command never surfaces as an NLDE-DATA indication"
        );
        assert_eq!(
            nwk.take_command_outcome(),
            Some(NwkCommandOutcome::LeaveRequested {
                src: ShortAddress(0x0000),
                rejoin: true,
                remove_children: false,
            })
        );
        assert!(!nwk.is_joined());
    }

    #[test]
    fn leave_request_from_a_non_parent_is_ignored() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        let leave = crate::frames::LeaveCommand {
            remove_children: false,
            request: true,
            rejoin: true,
        };
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::Leave, &[leave.serialize()], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, OUR_ADDR),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert_eq!(
            nwk.take_command_outcome(),
            None,
            "an unauthorized leave reports no lifecycle outcome"
        );
        assert!(nwk.is_joined());
    }

    #[test]
    fn unsecured_commands_are_dropped_on_a_secured_network() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        nwk.nib.security_enabled = true;
        let leave = crate::frames::LeaveCommand {
            remove_children: false,
            request: true,
            rejoin: false,
        };
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::Leave, &[leave.serialize()], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, ShortAddress(0x0000), OUR_ADDR),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert_eq!(
            nwk.take_command_outcome(),
            None,
            "a forged unsecured leave reports no lifecycle outcome"
        );
        assert!(
            nwk.is_joined(),
            "a forged unsecured leave must not take the device off the network"
        );
    }

    /// Deliver a NWK End Device Timeout Response (0x0C) from `src` to `dst`.
    fn deliver_ed_timeout_response(
        nwk: &mut NwkLayer<MockMac>,
        src: ShortAddress,
        dst: ShortAddress,
        body: &[u8],
    ) {
        deliver_ed_timeout_response_with_seq(nwk, src, dst, body, 7);
    }

    fn deliver_ed_timeout_response_with_seq(
        nwk: &mut NwkLayer<MockMac>,
        src: ShortAddress,
        dst: ShortAddress,
        body: &[u8],
        sequence: u8,
    ) {
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::EdTimeoutResponse, body, &mut payload);
        let mut buf = [0u8; 128];
        let mut header = frame(NwkFrameType::Command, src, dst);
        header.seq_number = sequence;
        let len = encode(&header, &payload[..payload_len], &mut buf);
        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert_eq!(
            nwk.take_command_outcome(),
            None,
            "the timeout negotiation is NIB state, not a lifecycle outcome"
        );
    }

    fn negotiating_end_device() -> NwkLayer<MockMac> {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        nwk.nib.reset_end_device_timeout_negotiation();
        block_on(nwk.send_ed_timeout_request()).unwrap();
        nwk
    }

    #[test]
    fn ed_timeout_success_stores_masked_parent_information() {
        let mut nwk = negotiating_end_device();
        // Trailing byte from a future revision must be ignored, not rejected.
        deliver_ed_timeout_response(
            &mut nwk,
            ShortAddress(0x0000),
            OUR_ADDR,
            &[0x00, 0xF3, 0x5A],
        );

        assert!(nwk.nib.parent_information_valid);
        assert_eq!(nwk.nib.parent_information, 0x03);
        assert_eq!(nwk.nib.end_device_timeout, ED_TIMEOUT_ENUM_REQUESTED);
        assert_eq!(
            nwk.nib.requested_end_device_timeout,
            ED_TIMEOUT_ENUM_REQUESTED
        );
    }

    #[test]
    fn ed_timeout_incorrect_value_lowers_only_to_the_default_floor() {
        let mut nwk = negotiating_end_device();
        // Establish prior valid parent information, then refuse a request.
        deliver_ed_timeout_response(&mut nwk, ShortAddress(0x0000), OUR_ADDR, &[0x00, 0x01]);
        assert!(nwk.nib.parent_information_valid);

        for _ in 0..(ED_TIMEOUT_ENUM_REQUESTED - ED_TIMEOUT_ENUM_DEFAULT + 3) {
            block_on(nwk.send_ed_timeout_request()).unwrap();
            deliver_ed_timeout_response(&mut nwk, ShortAddress(0x0000), OUR_ADDR, &[0x01, 0x02]);
        }

        assert_eq!(
            nwk.nib.requested_end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT,
            "refusals must never walk below the default enumeration"
        );
        assert!(
            nwk.nib.parent_information_valid,
            "a refusal must not invalidate previously accepted parent information"
        );
        assert_eq!(
            nwk.nib.parent_information, 0x01,
            "a refusal must not overwrite prior valid parent information"
        );
    }

    #[test]
    fn ed_timeout_unknown_status_changes_no_state() {
        let mut nwk = negotiating_end_device();
        deliver_ed_timeout_response(&mut nwk, ShortAddress(0x0000), OUR_ADDR, &[0x7F, 0x03]);

        assert!(!nwk.nib.parent_information_valid);
        assert_eq!(nwk.nib.parent_information, 0);
        assert_eq!(nwk.nib.end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT);
        assert_eq!(
            nwk.nib.requested_end_device_timeout,
            ED_TIMEOUT_ENUM_REQUESTED
        );
    }

    #[test]
    fn unsolicited_and_duplicate_ed_timeout_responses_are_ignored() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        nwk.nib.reset_end_device_timeout_negotiation();

        deliver_ed_timeout_response_with_seq(
            &mut nwk,
            ShortAddress(0x0000),
            OUR_ADDR,
            &[0x00, 0x03],
            1,
        );
        assert!(!nwk.nib.parent_information_valid);
        assert_eq!(nwk.nib.end_device_timeout_accepts, 0);

        block_on(nwk.send_ed_timeout_request()).unwrap();
        deliver_ed_timeout_response_with_seq(
            &mut nwk,
            ShortAddress(0x0000),
            OUR_ADDR,
            &[0x00, 0x01],
            2,
        );
        assert!(nwk.nib.parent_information_valid);
        assert_eq!(nwk.nib.parent_information, 0x01);
        assert_eq!(nwk.nib.end_device_timeout_accepts, 1);

        deliver_ed_timeout_response_with_seq(
            &mut nwk,
            ShortAddress(0x0000),
            OUR_ADDR,
            &[0x01, 0x02],
            3,
        );
        assert_eq!(nwk.nib.parent_information, 0x01);
        assert_eq!(nwk.nib.end_device_timeout_accepts, 1);
        assert_eq!(
            nwk.nib.requested_end_device_timeout, ED_TIMEOUT_ENUM_REQUESTED,
            "a delayed duplicate must not ratchet the requested timeout"
        );
    }

    #[test]
    fn ed_timeout_response_is_only_accepted_from_the_parent_to_this_node() {
        // Wrong source: a neighbour that is not the current parent.
        let mut nwk = negotiating_end_device();
        deliver_ed_timeout_response(&mut nwk, PEER, OUR_ADDR, &[0x00, 0x03]);
        assert!(!nwk.nib.parent_information_valid);

        // Wrong destination: a frame addressed elsewhere.
        let mut nwk = negotiating_end_device();
        deliver_ed_timeout_response(&mut nwk, ShortAddress(0x0000), FAR, &[0x00, 0x03]);
        assert!(!nwk.nib.parent_information_valid);

        // No valid parent yet.
        let mut nwk = negotiating_end_device();
        nwk.nib.parent_address = ShortAddress(0xFFFF);
        deliver_ed_timeout_response(&mut nwk, ShortAddress(0xFFFF), OUR_ADDR, &[0x00, 0x03]);
        assert!(!nwk.nib.parent_information_valid);

        // Truncated payload.
        let mut nwk = negotiating_end_device();
        deliver_ed_timeout_response(&mut nwk, ShortAddress(0x0000), OUR_ADDR, &[0x00]);
        assert!(!nwk.nib.parent_information_valid);
    }

    #[test]
    fn ed_timeout_response_is_ignored_by_non_end_devices() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.nib.reset_end_device_timeout_negotiation();
        deliver_ed_timeout_response(&mut nwk, ShortAddress(0x0000), OUR_ADDR, &[0x00, 0x03]);
        assert!(!nwk.nib.parent_information_valid);
    }

    #[test]
    fn unsecured_ed_timeout_response_is_dropped_on_a_secured_network() {
        let mut nwk = negotiating_end_device();
        nwk.nib.security_enabled = true;
        deliver_ed_timeout_response(&mut nwk, ShortAddress(0x0000), OUR_ADDR, &[0x00, 0x03]);
        assert!(
            !nwk.nib.parent_information_valid,
            "the shared secured-command guard must run before the timeout state changes"
        );
    }

    #[test]
    fn parent_leave_clears_the_end_device_timeout_negotiation() {
        let mut nwk = negotiating_end_device();
        deliver_ed_timeout_response(&mut nwk, ShortAddress(0x0000), OUR_ADDR, &[0x00, 0x03]);
        assert!(nwk.nib.parent_information_valid);

        let leave = crate::frames::LeaveCommand {
            remove_children: false,
            request: false,
            rejoin: false,
        };
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::Leave, &[leave.serialize()], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(
                NwkFrameType::Command,
                ShortAddress(0x0000),
                ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            ),
            &payload[..payload_len],
            &mut buf,
        );
        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert!(!nwk.nib.parent_information_valid);
        assert_eq!(nwk.nib.parent_information, 0);
        assert_eq!(nwk.nib.end_device_timeout, ED_TIMEOUT_ENUM_DEFAULT);
    }

    #[test]
    fn parent_leave_indication_is_reported_and_clears_the_joined_state() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        let leave = crate::frames::LeaveCommand {
            remove_children: false,
            request: false,
            rejoin: false,
        };
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::Leave, &[leave.serialize()], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(
                NwkFrameType::Command,
                ShortAddress(0x0000),
                ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            ),
            &payload[..payload_len],
            &mut buf,
        );

        let indication = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));

        assert!(indication.is_none());
        assert_eq!(
            nwk.take_command_outcome(),
            Some(NwkCommandOutcome::ParentLeft {
                src: ShortAddress(0x0000)
            })
        );
        assert!(!nwk.is_joined());
    }

    #[test]
    fn link_status_is_handled_internally_without_a_lifecycle_outcome() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        nwk.neighbors
            .add_or_update(crate::neighbor::NeighborEntry::new_from_annce(
                PEER, [0x0B; 8],
            ))
            .unwrap();
        // One entry naming us, with incoming cost 3 (low nibble bits 0..2).
        let body = [
            0x01u8,
            (OUR_ADDR.0 & 0xFF) as u8,
            (OUR_ADDR.0 >> 8) as u8,
            0x03,
        ];
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::LinkStatus, &body, &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(
                NwkFrameType::Command,
                PEER,
                ShortAddress::BROADCAST_RX_ON_WHEN_IDLE,
            ),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert_eq!(
            nwk.neighbors
                .find_by_short(PEER)
                .expect("neighbour is present")
                .outgoing_cost,
            3,
            "the link cost update is applied inside the NWK layer"
        );
        assert_eq!(
            nwk.take_command_outcome(),
            None,
            "link maintenance is not a lifecycle outcome"
        );
        assert!(nwk.is_joined());
    }

    #[test]
    fn a_command_outcome_never_survives_the_next_frame() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);
        let leave = crate::frames::LeaveCommand {
            remove_children: false,
            request: true,
            rejoin: true,
        };
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::Leave, &[leave.serialize()], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, ShortAddress(0x0000), OUR_ADDR),
            &payload[..payload_len],
            &mut buf,
        );

        // The caller deliberately does not collect the outcome here.
        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        // An unrelated data frame must clear it rather than let a stale Leave
        // be attributed to this frame.
        let mut data = [0u8; 128];
        let data_len = encode(
            &frame(NwkFrameType::Data, PEER, OUR_ADDR),
            &[0x42],
            &mut data,
        );
        let indication = block_on(nwk.process_incoming_nwk_frame(&data[..data_len], 42));

        assert!(matches!(indication, Some(NwkIndication::Borrowed(_))));
        assert_eq!(
            nwk.take_command_outcome(),
            None,
            "an uncollected outcome must not leak into a later frame"
        );
    }

    // ── Route failure and discovery fallback ─────────────────

    #[test]
    #[cfg(feature = "router")]
    fn unroutable_unicast_starts_one_route_discovery() {
        let mut nwk = node(DeviceType::Coordinator, ShortAddress(0x0000));
        nwk.nib.parent_address = ShortAddress(0xFFFF);

        let first = block_on(nwk.nlde_data_request(FAR, 30, &[0x01, 0x02], false, true));
        assert_eq!(first.err(), Some(NwkStatus::RouteDiscoveryFailed));

        let requests: std::vec::Vec<_> = nwk
            .mac
            .tx_history()
            .iter()
            .map(tx_frame)
            .filter(|(header, cmd)| {
                header.frame_control.frame_type == NwkFrameType::Command as u8
                    && *cmd == NwkCommandId::RouteRequest as u8
            })
            .collect();
        assert_eq!(
            requests.len(),
            1,
            "one RREQ is broadcast for the missing route"
        );
        assert!(nwk.routing.has_active_discovery(FAR));

        let second = block_on(nwk.nlde_data_request(FAR, 30, &[0x03], false, true));
        assert_eq!(second.err(), Some(NwkStatus::RouteDiscoveryFailed));
        assert_eq!(
            nwk.mac.tx_history().len(),
            1,
            "an outstanding discovery must not be restarted per frame"
        );
    }

    #[test]
    fn unroutable_unicast_without_discovery_permission_reports_the_route_error() {
        let mut nwk = node(DeviceType::Coordinator, ShortAddress(0x0000));
        nwk.nib.parent_address = ShortAddress(0xFFFF);

        let result = block_on(nwk.nlde_data_request(FAR, 30, &[0x01], false, false));

        assert_eq!(result.err(), Some(NwkStatus::RouteError));
        assert!(nwk.mac.tx_history().is_empty());
        assert!(!nwk.routing.has_active_discovery(FAR));
    }

    #[test]
    fn unroutable_unicast_does_not_consume_a_reserved_frame_counter() {
        let mut nwk = node(DeviceType::Coordinator, ShortAddress(0x0000));
        nwk.nib.parent_address = ShortAddress(0xFFFF);
        nwk.nib.security_enabled = true;
        nwk.security.set_network_key([0xAA; 16], 0);
        assert!(nwk.nib.set_frame_counter_reservation(10, 20));

        let result = block_on(nwk.nlde_data_request(FAR, 30, &[0x01], true, false));

        assert_eq!(result.err(), Some(NwkStatus::RouteError));
        assert_eq!(
            nwk.nib.outgoing_frame_counter, 10,
            "a frame that is never transmitted must not burn counter space"
        );
    }

    #[test]
    fn end_devices_fall_back_to_the_parent_instead_of_discovering_routes() {
        let mut nwk = node(DeviceType::EndDevice, OUR_ADDR);

        let confirm = block_on(nwk.nlde_data_request(FAR, 30, &[0x01], false, true))
            .expect("end devices always route through the parent");
        assert_eq!(confirm.status, NwkStatus::Success);
        assert_eq!(
            tx_short_dst(&nwk.mac.tx_history()[0]),
            Some(ShortAddress(0x0000))
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn relay_failure_removes_the_route_and_queues_a_network_status() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.routing.update_route(FAR, NEXT_HOP, 1).unwrap();

        let header = frame(NwkFrameType::Data, PEER, FAR);
        nwk.handle_relay_failure(&header, NEXT_HOP, false);

        assert_eq!(nwk.routing.next_hop(FAR), None);
        assert_eq!(nwk.pending_route_errors.len(), 1);
        assert_eq!(nwk.pending_route_errors[0].destination, PEER);
        assert_eq!(nwk.pending_route_errors[0].failed_destination, FAR);
        assert_eq!(nwk.pending_route_errors[0].next_hop, None);
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_mtor_link_failure_is_reported_to_the_concentrator_via_another_router() {
        let mut relay = node(DeviceType::Router, OUR_ADDR);
        relay
            .routing
            .update_route_many_to_one(
                CONCENTRATOR,
                NEXT_HOP,
                2,
                crate::routing::ConcentratorType::HighRam,
            )
            .unwrap();
        neighbour_with_cost(&mut relay, PEER, 1);
        relay.mac.set_tx_failures(1);
        let header = frame(NwkFrameType::Data, ORIGIN, CONCENTRATOR);

        assert_eq!(
            block_on(relay.relay_frame(&header, &[0xAA], ORIGIN)).unwrap_err(),
            NwkStatus::RouteError
        );
        assert!(relay.routing.has_failed_many_to_one(CONCENTRATOR));
        assert_eq!(relay.pending_route_errors.len(), 1);
        let pending = &relay.pending_route_errors[0];
        assert_eq!(
            pending.status_code,
            crate::frames::NetworkStatusCommand::MANY_TO_ONE_ROUTE_FAILURE
        );
        assert_eq!(pending.destination, CONCENTRATOR);
        assert_eq!(pending.failed_destination, ORIGIN);
        assert_eq!(pending.next_hop, Some(PEER));

        block_on(relay.process_pending_routing());
        assert_eq!(relay.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&relay.mac.tx_history()[0]), Some(PEER));
        let bytes = relay.mac.tx_history()[0].payload.as_slice();
        let (status_header, consumed) = NwkHeader::parse(bytes).unwrap();
        assert_eq!(status_header.dst_addr, CONCENTRATOR);
        assert_eq!(bytes[consumed], NwkCommandId::NetworkStatus as u8);
        let status = crate::frames::NetworkStatusCommand::parse(&bytes[consumed + 1..]).unwrap();
        assert_eq!(
            status.status_code,
            crate::frames::NetworkStatusCommand::MANY_TO_ONE_ROUTE_FAILURE
        );
        assert_eq!(status.destination, ORIGIN);

        assert_eq!(
            block_on(relay.nlde_data_request(CONCENTRATOR, 5, &[0x01], false, true,)).unwrap_err(),
            NwkStatus::RouteError,
            "a failed MTOR route is not automatically rediscovered"
        );
        assert_eq!(
            relay.mac.tx_history().len(),
            1,
            "no ordinary RREQ is emitted for the failed MTOR route"
        );

        relay
            .routing
            .update_route_many_to_one(
                CONCENTRATOR,
                NEXT_HOP,
                1,
                crate::routing::ConcentratorType::HighRam,
            )
            .unwrap();
        assert!(
            !relay.routing.has_failed_many_to_one(CONCENTRATOR),
            "a new MTOR request replaces the failed-route tombstone"
        );
        block_on(relay.nlde_data_request(CONCENTRATOR, 5, &[0x02], false, true))
            .expect("the replacement MTOR route is usable");
        assert_eq!(
            relay.mac.tx_history().len(),
            3,
            "the replacement path emits its required Route Record, then the data"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_source_route_failure_queues_status_0x0b_back_to_the_originator() {
        let mut relay = node(DeviceType::Router, OUR_ADDR);
        relay.update_neighbor_address(CONCENTRATOR, [0xA5; 8]);
        relay.routing.update_route(CONCENTRATOR, PEER, 1).unwrap();
        relay.mac.set_tx_failures(1);

        let mut header = frame(NwkFrameType::Data, CONCENTRATOR, FAR);
        header.frame_control.source_route = true;
        let mut relay_list = heapless::Vec::new();
        relay_list.push(OUR_ADDR).unwrap();
        header.source_route = Some(crate::frames::SourceRoute {
            relay_count: 1,
            relay_index: 0,
            relay_list,
        });

        assert_eq!(
            block_on(relay.relay_frame_source_routed(&header, &[0x02], 4)).unwrap_err(),
            NwkStatus::RouteError
        );
        assert_eq!(relay.pending_route_errors.len(), 1);
        assert_eq!(
            relay.pending_route_errors[0].status_code,
            crate::frames::NetworkStatusCommand::SOURCE_ROUTE_FAILURE
        );
        assert_eq!(relay.pending_route_errors[0].destination, CONCENTRATOR);
        assert_eq!(relay.pending_route_errors[0].failed_destination, FAR);
        assert_eq!(relay.pending_route_errors[0].next_hop, None);

        block_on(relay.process_pending_routing());
        let bytes = relay.mac.tx_history()[0].payload.as_slice();
        let (status_header, consumed) = NwkHeader::parse(bytes).unwrap();
        assert_eq!(status_header.dst_addr, CONCENTRATOR);
        assert_eq!(status_header.dst_ieee, Some([0xA5; 8]));
        let status = crate::frames::NetworkStatusCommand::parse(&bytes[consumed + 1..]).unwrap();
        assert_eq!(
            status.status_code,
            crate::frames::NetworkStatusCommand::SOURCE_ROUTE_FAILURE
        );
        assert_eq!(status.destination, FAR);
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_mtor_failure_status_without_a_route_uses_a_random_router_neighbor() {
        let mut relay = node(DeviceType::Router, OUR_ADDR);
        relay.nib.parent_address = ShortAddress::BROADCAST;
        neighbour_with_cost(&mut relay, PEER, 1);
        let header = frame(NwkFrameType::Command, ORIGIN, CONCENTRATOR);
        let status = crate::frames::NetworkStatusCommand {
            status_code: crate::frames::NetworkStatusCommand::MANY_TO_ONE_ROUTE_FAILURE,
            destination: FAR,
        };
        let mut body = [0u8; 4];
        let body_len = status.serialize(&mut body);
        let mut payload = [0u8; 5];
        payload[0] = NwkCommandId::NetworkStatus as u8;
        payload[1..1 + body_len].copy_from_slice(&body[..body_len]);

        block_on(relay.relay_frame(&header, &payload[..1 + body_len], NEXT_HOP)).unwrap();

        assert_eq!(relay.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&relay.mac.tx_history()[0]), Some(PEER));
    }

    #[test]
    #[cfg(feature = "router")]
    fn source_route_statuses_invalidate_cached_return_paths() {
        let mut concentrator = node(DeviceType::Coordinator, CONCENTRATOR);

        for status_code in [
            crate::frames::NetworkStatusCommand::SOURCE_ROUTE_FAILURE,
            crate::frames::NetworkStatusCommand::MANY_TO_ONE_ROUTE_FAILURE,
        ] {
            concentrator.source_route_table.insert(FAR, &[PEER]);
            concentrator.routing.update_route(FAR, PEER, 1).unwrap();
            let status = crate::frames::NetworkStatusCommand {
                status_code,
                destination: FAR,
            };
            let mut payload = [0u8; 4];
            let len = status.serialize(&mut payload);

            concentrator.handle_network_status(PEER, &payload[..len]);

            assert!(concentrator.source_route_table.lookup(FAR).is_none());
            assert!(concentrator.routing.get_entry(FAR).is_none());
        }
    }

    #[test]
    #[cfg(feature = "router")]
    fn expired_route_discoveries_allow_a_retry() {
        let mut nwk = node(DeviceType::Coordinator, ShortAddress(0x0000));
        nwk.nib.parent_address = ShortAddress(0xFFFF);
        assert!(block_on(nwk.discover_route(FAR)).is_ok());
        assert!(nwk.routing.has_active_discovery(FAR));

        block_on(
            nwk.mac
                .delay_micros(crate::routing::ROUTE_DISCOVERY_TIMEOUT_US + 1),
        );
        nwk.tick_router_maintenance(1);

        assert!(
            !nwk.routing.has_active_discovery(FAR),
            "a discovery that never produced a reply must be retryable"
        );
    }

    #[test]
    #[cfg(not(feature = "router"))]
    fn builds_without_the_router_feature_do_not_forward_traffic() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let mut buf = [0u8; 128];
        let len = encode(&frame(NwkFrameType::Data, PEER, FAR), &[0x01], &mut buf);

        assert!(!nwk.can_route());
        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert!(
            nwk.mac.tx_history().is_empty(),
            "routing tables have zero capacity without the router feature, so \
             forwarding must be refused instead of storming the network"
        );
    }

    #[test]
    #[cfg(not(feature = "router"))]
    fn builds_without_the_router_feature_report_route_errors_instead_of_discovering() {
        let mut nwk = node(DeviceType::Coordinator, ShortAddress(0x0000));
        nwk.nib.parent_address = ShortAddress(0xFFFF);

        let result = block_on(nwk.nlde_data_request(FAR, 30, &[0x01], false, true));

        assert_eq!(result.err(), Some(NwkStatus::RouteError));
        assert_eq!(
            block_on(nwk.discover_route(FAR)).err(),
            Some(NwkStatus::InvalidRequest)
        );
        assert!(nwk.mac.tx_history().is_empty());
    }

    // ── Secured relay (hop-by-hop NWK security) ──────────────

    #[test]
    #[cfg(feature = "router")]
    fn a_secured_unicast_relay_is_rebuilt_and_re_encrypted_for_the_next_hop() {
        let on_air = secured_frame_on_air(FAR, &[0xDE, 0xAD, 0xBE, 0xEF]);
        let (origin_header, _) = NwkHeader::parse(&on_air).expect("originator frame parses");

        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        assert!(
            relay
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );
        relay.routing.update_route(FAR, NEXT_HOP, 1).unwrap();

        assert!(block_on(relay.process_incoming_nwk_frame(&on_air, 42)).is_none());

        assert_eq!(
            relay.rx_security_stats().decrypt_successes,
            1,
            "a relay authenticates the frame before forwarding it"
        );
        assert_eq!(relay.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&relay.mac.tx_history()[0]), Some(NEXT_HOP));
        let relayed = recorded_frame(&relay, 0);
        let (relayed_header, consumed) = NwkHeader::parse(&relayed).expect("relayed frame parses");

        // Everything but the radius is preserved end to end.
        assert_eq!(relayed_header.dst_addr, FAR);
        assert_eq!(
            relayed_header.src_addr, ORIGIN,
            "the relay preserves the originator"
        );
        assert_eq!(
            relayed_header.seq_number, origin_header.seq_number,
            "the relay preserves the NWK sequence number"
        );
        assert_eq!(relayed_header.radius, origin_header.radius - 1);
        assert!(relayed_header.frame_control.security);

        // The relay owns the security applied to its own transmission.
        let (aux, _) = crate::security::NwkSecurityHeader::parse(&relayed[consumed..])
            .expect("the relay writes its own auxiliary header");
        assert_eq!(
            aux.source_address, RELAY_IEEE,
            "the auxiliary header carries the relaying device's IEEE address"
        );
        assert_eq!(
            aux.frame_counter, RESERVED_FLOOR,
            "the relay allocates a fresh counter from its durable reservation"
        );
        assert_eq!(relay.nib.outgoing_frame_counter, RESERVED_FLOOR + 1);
        assert_ne!(
            &relayed[consumed..],
            &on_air[consumed..],
            "the original ciphertext and MIC must never be copied through"
        );

        // The next hop authenticates and decrypts what the relay produced.
        let mut destination = secured_node(DeviceType::Router, FAR, DEST_IEEE);
        match block_on(destination.process_incoming_nwk_frame(&relayed, 42)) {
            Some(NwkIndication::Owned(data)) => {
                assert_eq!(data.payload.as_slice(), &[0xDE, 0xAD, 0xBE, 0xEF]);
                assert_eq!(data.src_addr, ORIGIN);
                assert!(data.security_use);
            }
            other => panic!("the relayed frame must authenticate at the next hop, got {other:?}"),
        }
        assert_eq!(destination.rx_security_stats().decrypt_failures, 0);
    }

    #[test]
    #[cfg(feature = "router")]
    fn mutating_a_secured_header_without_re_encrypting_fails_at_the_next_hop() {
        // This is the copy-through relay the shared frame builder replaces:
        // the NWK header is CCM* additional authenticated data, so editing the
        // radius in place while keeping the original ciphertext and MIC breaks
        // authentication at the very next hop.
        let mut on_air = secured_frame_on_air(FAR, &[0x33, 0x44]);
        const RADIUS_OFFSET: usize = 6;
        on_air[RADIUS_OFFSET] -= 1;

        let mut destination = secured_node(DeviceType::Router, FAR, DEST_IEEE);
        assert!(block_on(destination.process_incoming_nwk_frame(&on_air, 42)).is_none());
        assert_eq!(destination.rx_security_stats().decrypt_failures, 1);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_secured_broadcast_is_authenticated_once_then_rebroadcast_and_delivered() {
        let on_air = secured_frame_on_air(ShortAddress::BROADCAST, &[0x5A, 0xA5]);

        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        assert!(
            relay
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );

        match block_on(relay.process_incoming_nwk_frame(&on_air, 42)) {
            Some(NwkIndication::Owned(data)) => {
                assert_eq!(
                    data.payload.as_slice(),
                    &[0x5A, 0xA5],
                    "the local copy is plaintext"
                );
                assert!(data.security_use);
            }
            other => panic!("expected local delivery of the broadcast, got {other:?}"),
        }
        assert_eq!(
            relay.rx_security_stats().decrypt_successes,
            1,
            "one authentication covers both the rebroadcast and the local copy"
        );
        assert_eq!(
            relay.mac.tx_history().len(),
            1,
            "the broadcast is rebroadcast exactly once"
        );
        assert_eq!(
            tx_short_dst(&relay.mac.tx_history()[0]),
            Some(ShortAddress::BROADCAST)
        );

        let rebroadcast = recorded_frame(&relay, 0);
        let (header, consumed) = NwkHeader::parse(&rebroadcast).expect("rebroadcast parses");
        assert_eq!(header.radius, 4);
        assert_eq!(header.src_addr, ORIGIN);
        let (aux, _) = crate::security::NwkSecurityHeader::parse(&rebroadcast[consumed..])
            .expect("the rebroadcast carries a fresh auxiliary header");
        assert_eq!(aux.source_address, RELAY_IEEE);
        assert_eq!(aux.frame_counter, RESERVED_FLOOR);

        // A neighbour of the relay authenticates the rebroadcast.
        let mut listener = secured_node(DeviceType::Router, ShortAddress(0x6666), DEST_IEEE);
        match block_on(listener.process_incoming_nwk_frame(&rebroadcast, 42)) {
            Some(NwkIndication::Owned(data)) => {
                assert_eq!(data.payload.as_slice(), &[0x5A, 0xA5]);
                assert_eq!(data.src_addr, ORIGIN);
            }
            other => panic!("the rebroadcast must authenticate at the next hop, got {other:?}"),
        }

        // The incoming counter was committed, so the very same frame is now a
        // replay — and nothing further goes on air.
        assert!(block_on(relay.process_incoming_nwk_frame(&on_air, 42)).is_none());
        assert_eq!(relay.rx_security_stats().replay_rejections, 1);
        assert_eq!(relay.mac.tx_history().len(), 1);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_forged_relay_neither_commits_the_replay_counter_nor_reaches_the_next_hop() {
        let mut on_air = secured_frame_on_air(FAR, &[0x11, 0x22]);
        let mic_byte = on_air.len() - 1;
        on_air[mic_byte] ^= 0xFF;

        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        relay.routing.update_route(FAR, NEXT_HOP, 1).unwrap();

        assert!(block_on(relay.process_incoming_nwk_frame(&on_air, 42)).is_none());
        assert_eq!(relay.rx_security_stats().decrypt_failures, 1);
        assert!(
            relay.mac.tx_history().is_empty(),
            "a frame that fails its MIC must never be relayed"
        );

        // The rejected frame must not have advanced the replay window: the
        // genuine frame carrying the same counter still authenticates.
        on_air[mic_byte] ^= 0xFF;
        assert!(block_on(relay.process_incoming_nwk_frame(&on_air, 42)).is_none());
        assert_eq!(relay.rx_security_stats().decrypt_successes, 1);
        assert_eq!(relay.rx_security_stats().replay_rejections, 0);
        assert_eq!(
            relay.mac.tx_history().len(),
            1,
            "the genuine frame is relayed"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_secured_relay_without_counter_space_is_refused_instead_of_malformed() {
        let on_air = secured_frame_on_air(FAR, &[0x01, 0x02]);
        let (header, _) = NwkHeader::parse(&on_air).expect("originator frame parses");

        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        // The durable reservation is already spent: no counter may be issued.
        assert!(
            relay
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR)
        );
        relay.routing.update_route(FAR, NEXT_HOP, 1).unwrap();

        assert!(block_on(relay.process_incoming_nwk_frame(&on_air, 42)).is_none());
        assert!(
            relay.mac.tx_history().is_empty(),
            "a relay that cannot be secured must not go on air"
        );
        assert_eq!(
            block_on(relay.relay_frame(&header, &[0x01, 0x02], header.src_addr)).unwrap_err(),
            NwkStatus::MaxFrmCounterReached,
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_router_without_the_network_key_cannot_relay_secured_traffic() {
        let on_air = secured_frame_on_air(FAR, &[0x01, 0x02]);

        let mut relay = node(DeviceType::Router, OUR_ADDR);
        relay.nib.ieee_address = RELAY_IEEE;
        relay.nib.security_enabled = true;
        relay.routing.update_route(FAR, NEXT_HOP, 1).unwrap();

        assert!(block_on(relay.process_incoming_nwk_frame(&on_air, 42)).is_none());
        assert_eq!(relay.rx_security_stats().missing_keys, 1);
        assert!(
            relay.mac.tx_history().is_empty(),
            "an unauthenticated frame must not be forwarded"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn unsecured_commands_are_not_relayed_on_a_secured_network() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.nib.security_enabled = true;
        nwk.routing.update_route(FAR, NEXT_HOP, 1).unwrap();
        let leave = crate::frames::LeaveCommand {
            remove_children: true,
            request: true,
            rejoin: false,
        };
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::Leave, &[leave.serialize()], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, FAR),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());
        assert!(
            nwk.mac.tx_history().is_empty(),
            "a forged unsecured command must not be carried further into the network"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_unsecured_broadcast_is_dropped_on_a_secured_network_without_poisoning_the_btr() {
        let genuine = secured_frame_on_air(ShortAddress::BROADCAST, &[0x5A, 0xA5]);
        let (genuine_header, _) = NwkHeader::parse(&genuine).expect("originator frame parses");

        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        assert!(
            relay
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );

        // A forged, unsecured copy of the same broadcast: same originator,
        // same sequence number, so accepting it would both launder it into
        // the network and suppress the genuine frame that follows.
        let mut forged = frame(NwkFrameType::Data, ORIGIN, ShortAddress::BROADCAST);
        forged.seq_number = genuine_header.seq_number;
        forged.radius = genuine_header.radius;
        let mut buf = [0u8; 128];
        let len = encode(&forged, &[0xDE, 0xAD], &mut buf);

        assert!(
            block_on(relay.process_incoming_nwk_frame(&buf[..len], 42)).is_none(),
            "an unsecured broadcast is not delivered on a secured network"
        );
        assert!(
            relay.mac.tx_history().is_empty(),
            "an unsecured broadcast must never be rebroadcast"
        );
        assert_eq!(
            relay.rx_security_stats().secured_frames,
            0,
            "the forged frame never reached the CCM* path"
        );

        // The genuine secured broadcast still works: it is delivered locally,
        // rebroadcast once, and it is the first frame to spend the durable
        // outgoing counter reservation — the forged one burned nothing.
        match block_on(relay.process_incoming_nwk_frame(&genuine, 42)) {
            Some(NwkIndication::Owned(data)) => {
                assert_eq!(data.payload.as_slice(), &[0x5A, 0xA5]);
                assert!(data.security_use);
            }
            other => panic!("expected local delivery of the secured broadcast, got {other:?}"),
        }
        assert_eq!(
            relay.mac.tx_history().len(),
            1,
            "the genuine broadcast is rebroadcast exactly once"
        );
        let rebroadcast = recorded_frame(&relay, 0);
        let (header, consumed) = NwkHeader::parse(&rebroadcast).expect("rebroadcast parses");
        assert_eq!(header.src_addr, ORIGIN);
        assert_eq!(header.seq_number, genuine_header.seq_number);
        let (aux, _) = crate::security::NwkSecurityHeader::parse(&rebroadcast[consumed..])
            .expect("the rebroadcast carries a fresh auxiliary header");
        assert_eq!(
            aux.frame_counter, RESERVED_FLOOR,
            "no outgoing frame counter was spent on the forged broadcast"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_unsecured_unicast_is_still_accepted_on_a_secured_network() {
        // Pre-key APS commissioning traffic (Transport-Key) arrives as an
        // unsecured NWK unicast; the broadcast/command drop above must not
        // take it with it. APS applies its own policy to the payload.
        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Data, PEER, OUR_ADDR),
            &[0x01, 0x02],
            &mut buf,
        );

        match block_on(relay.process_incoming_nwk_frame(&buf[..len], 42)) {
            Some(NwkIndication::Borrowed(data)) => {
                assert_eq!(data.payload, &[0x01, 0x02]);
                assert!(!data.security_use);
            }
            other => panic!("expected local delivery of the unsecured unicast, got {other:?}"),
        }
    }

    // ── Sleepy children versus announced neighbours ──────────

    #[test]
    #[cfg(feature = "router")]
    fn an_announced_neighbour_is_relayed_directly_and_never_queued() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        // A Device_annce only carries the address pair, so the entry keeps the
        // default `rx_on_when_idle == false` — it is not a sleepy child of ours.
        nwk.update_neighbor_address(FAR, [9u8; 8]);
        let mut buf = [0u8; 128];
        let len = encode(&frame(NwkFrameType::Data, PEER, FAR), &[0x01], &mut buf);

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert_eq!(nwk.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&nwk.mac.tx_history()[0]), Some(FAR));
        assert!(
            !nwk.indirect_queue().has_pending(FAR),
            "an announced neighbour must not be parked in the indirect queue"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_last_hop_source_route_to_a_sleeping_child_is_buffered_indirectly() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.nib.permit_joining = true;
        // Capability information without the rx-on-when-idle bit: a real
        // sleeping end device that joined through us.
        let child = nwk
            .handle_child_association([7u8; 8], 0x80)
            .expect("the child associates");

        // A source route whose last relay is this device: the next hop is the
        // destination itself, i.e. the sleeping child.
        let mut header = frame(NwkFrameType::Data, PEER, child);
        header.frame_control.source_route = true;
        let mut relay_list = heapless::Vec::new();
        relay_list.push(OUR_ADDR).unwrap();
        header.source_route = Some(crate::frames::SourceRoute {
            relay_count: 1,
            relay_index: 0,
            relay_list,
        });
        let mut buf = [0u8; 128];
        let len = encode(&header, &[0x01], &mut buf);

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert!(
            nwk.mac.tx_history().is_empty(),
            "a sleeping child cannot receive a direct source-routed relay either"
        );
        assert!(
            nwk.indirect_queue().has_pending(child),
            "the source-routed frame must wait for the child's poll"
        );
        assert_eq!(
            nwk.mac.indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, child), true))
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_buffered_secured_source_route_reserves_one_frame_counter() {
        let mut nwk = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        assert!(
            nwk.nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );
        nwk.nib.permit_joining = true;
        let child = nwk
            .handle_child_association([7u8; 8], 0x80)
            .expect("the child associates");

        let mut header = frame(NwkFrameType::Data, PEER, child);
        header.frame_control.security = true;
        header.frame_control.source_route = true;
        let mut relay_list = heapless::Vec::new();
        relay_list.push(OUR_ADDR).unwrap();
        header.source_route = Some(crate::frames::SourceRoute {
            relay_count: 1,
            relay_index: 0,
            relay_list,
        });

        assert!(block_on(nwk.relay_frame(&header, &[0x01], header.src_addr)).is_ok());
        assert!(nwk.mac.tx_history().is_empty());
        assert!(nwk.indirect_queue().has_pending(child));
        assert_eq!(
            nwk.nib.outgoing_frame_counter,
            RESERVED_FLOOR + 1,
            "the buffered secured frame owns its reserved outgoing counter"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_source_routed_relay_to_an_announced_neighbour_still_goes_out_directly() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        // A Device_annce only carries the address pair, so the entry keeps the
        // default `rx_on_when_idle == false`. It is a sibling, not our child,
        // and must still be relayed to directly.
        nwk.update_neighbor_address(FAR, [9u8; 8]);

        let mut header = frame(NwkFrameType::Data, PEER, FAR);
        header.frame_control.source_route = true;
        let mut relay_list = heapless::Vec::new();
        relay_list.push(OUR_ADDR).unwrap();
        header.source_route = Some(crate::frames::SourceRoute {
            relay_count: 1,
            relay_index: 0,
            relay_list,
        });
        let mut buf = [0u8; 128];
        let len = encode(&header, &[0x01], &mut buf);

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert_eq!(nwk.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&nwk.mac.tx_history()[0]), Some(FAR));
        assert!(!nwk.indirect_queue().has_pending(FAR));
    }

    // ── NWK command propagation is owned by the handlers ─────

    #[test]
    #[cfg(feature = "router")]
    fn a_broadcast_route_request_is_never_relayed_verbatim() {
        use crate::frames::RouteRequest;

        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        // Nothing known about the RREQ target, so this router must propagate
        // the discovery rather than answer it.
        let rreq = RouteRequest {
            command_options: 0x00,
            route_request_id: 9,
            dst_addr: FAR,
            path_cost: 3,
            dst_ieee: None,
        };
        let mut body = [0u8; 16];
        let body_len = rreq.serialize(&mut body);
        let mut payload = [0u8; 32];
        let payload_len =
            command_payload(NwkCommandId::RouteRequest, &body[..body_len], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, ShortAddress::BROADCAST),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert!(
            nwk.mac.tx_history().is_empty(),
            "the generic broadcast relay must not flood a verbatim copy of the RREQ"
        );
        assert_eq!(
            nwk.pending_rreq_forwards.len(),
            1,
            "propagation is owned by the RREQ handler"
        );
        // Unknown neighbour → the default link cost of 7 is added to the
        // originator's cost.
        assert_eq!(nwk.pending_rreq_forwards[0].path_cost, 3 + 7);

        block_on(nwk.process_pending_routing());

        assert_eq!(
            nwk.mac.tx_history().len(),
            3,
            "the handler emits the original relay plus two R22 retries"
        );
        assert!(
            (512_000..=764_000).contains(&nwk.mac.monotonic_micros()),
            "2*R[2,128] ms jitter plus two 254 ms retry intervals"
        );
        assert_eq!(
            nwk.mac.tx_history()[0].payload.as_slice(),
            nwk.mac.tx_history()[1].payload.as_slice(),
            "a retry reuses the same NWK/security transaction"
        );
        assert_eq!(
            nwk.mac.tx_history()[1].payload.as_slice(),
            nwk.mac.tx_history()[2].payload.as_slice()
        );
        assert_eq!(
            nwk.mac.tx_history()[0].handle,
            nwk.mac.tx_history()[2].handle,
            "all MAC retries belong to one request"
        );
        assert_eq!(
            tx_short_dst(&nwk.mac.tx_history()[0]),
            Some(ShortAddress::BROADCAST)
        );
        let bytes = nwk.mac.tx_history()[0].payload.as_slice();
        let (forwarded_header, consumed) = NwkHeader::parse(bytes).expect("the forward parses");
        assert_eq!(bytes[consumed], NwkCommandId::RouteRequest as u8);
        assert_eq!(
            forwarded_header.src_addr, PEER,
            "a forwarded RREQ stays the originator's broadcast"
        );
        assert_eq!(
            forwarded_header.seq_number, 7,
            "the originator's NWK sequence number is preserved for the BTR"
        );
        assert_eq!(
            forwarded_header.radius, 4,
            "the received radius is decremented, never reset"
        );
        let propagated =
            RouteRequest::parse(&bytes[consumed + 1..]).expect("the forwarded RREQ parses");
        assert_eq!(propagated.route_request_id, 9);
        assert_eq!(propagated.dst_addr, FAR);
        assert_eq!(
            propagated.path_cost,
            3 + 7,
            "the forward carries our accumulated path cost, not the originator's"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_broadcast_link_status_is_handled_without_any_relay_transmission() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.update_neighbor_address(PEER, [0x0B; 8]);
        // One entry naming us, with incoming cost 3 (low nibble bits 0..2).
        let body = [
            0x01u8,
            (OUR_ADDR.0 & 0xFF) as u8,
            (OUR_ADDR.0 >> 8) as u8,
            0x03,
        ];
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::LinkStatus, &body, &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, ShortAddress::BROADCAST),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert_eq!(
            nwk.neighbors
                .find_by_short(PEER)
                .expect("neighbour is present")
                .outgoing_cost,
            3,
            "local dispatch of the command is preserved"
        );
        assert!(
            nwk.mac.tx_history().is_empty(),
            "Link Status is link-local: a router must never flood a neighbour's"
        );

        block_on(nwk.process_pending_routing());
        assert!(
            nwk.mac.tx_history().is_empty(),
            "and nothing is queued for later transmission either"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_broadcast_data_frame_is_still_rebroadcast_after_commands_stopped_being() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Data, PEER, ShortAddress::BROADCAST),
            &[0x55],
            &mut buf,
        );

        assert!(matches!(
            block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)),
            Some(NwkIndication::Borrowed(_))
        ));
        assert_eq!(nwk.mac.tx_history().len(), 1);
        let (rebroadcast, _) = tx_frame(&nwk.mac.tx_history()[0]);
        assert_eq!(
            rebroadcast.frame_control.frame_type,
            NwkFrameType::Data as u8
        );
        assert_eq!(rebroadcast.radius, 4);
    }

    // ── Network membership gates forwarding ──────────────────

    #[test]
    #[cfg(feature = "router")]
    fn an_unjoined_router_neither_relays_nor_burns_a_reserved_frame_counter() {
        let on_air = secured_frame_on_air(FAR, &[0xDE, 0xAD]);

        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        assert!(
            relay
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );
        relay.routing.update_route(FAR, NEXT_HOP, 1).unwrap();
        // Never joined, or told to leave: the routing state is still in place
        // but this device is no longer a member of the network.
        relay.set_joined(false);
        assert!(!relay.can_route());

        assert!(block_on(relay.process_incoming_nwk_frame(&on_air, 42)).is_none());

        assert!(
            relay.mac.tx_history().is_empty(),
            "an unjoined router must not transmit on behalf of the network"
        );
        assert_eq!(
            relay.nib.outgoing_frame_counter, RESERVED_FLOOR,
            "a relay that never happens must not burn durable counter space"
        );
        assert_eq!(
            relay.rx_security_stats().decrypt_successes,
            0,
            "the frame is dropped before CCM* runs on it"
        );
        assert_eq!(relay.nib.outgoing_frame_counter, RESERVED_FLOOR);

        // Rejoining restores forwarding without any other change.
        relay.set_joined(true);
        assert!(relay.can_route());
        assert!(block_on(relay.process_incoming_nwk_frame(&on_air, 42)).is_none());
        assert_eq!(relay.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&relay.mac.tx_history()[0]), Some(NEXT_HOP));
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_unjoined_router_neither_rebroadcasts_broadcasts_nor_propagates_route_requests() {
        use crate::frames::RouteRequest;

        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.set_joined(false);

        let mut data = [0u8; 128];
        let data_len = encode(
            &frame(NwkFrameType::Data, PEER, ShortAddress::BROADCAST),
            &[0x55],
            &mut data,
        );
        // Reception still works — a rejoining device has to hear the network.
        // What it must not do is carry the frame any further.
        assert!(matches!(
            block_on(nwk.process_incoming_nwk_frame(&data[..data_len], 42)),
            Some(NwkIndication::Borrowed(_))
        ));
        assert!(
            nwk.mac.tx_history().is_empty(),
            "an unjoined router must not rebroadcast"
        );

        let rreq = RouteRequest {
            command_options: 0x00,
            route_request_id: 9,
            dst_addr: FAR,
            path_cost: 3,
            dst_ieee: None,
        };
        let mut body = [0u8; 16];
        let body_len = rreq.serialize(&mut body);
        let mut payload = [0u8; 32];
        let payload_len =
            command_payload(NwkCommandId::RouteRequest, &body[..body_len], &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, ShortAddress::BROADCAST),
            &payload[..payload_len],
            &mut buf,
        );
        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert!(
            nwk.pending_rreq_forwards.is_empty(),
            "an unjoined router must not queue a RREQ forward"
        );
        block_on(nwk.process_pending_routing());
        assert!(
            nwk.mac.tx_history().is_empty(),
            "and nothing reaches the air on the next maintenance pass"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn locally_originated_data_for_a_sleeping_child_is_buffered() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.nib.permit_joining = true;
        let child = nwk
            .handle_child_association([7u8; 8], 0x80)
            .expect("the child associates");

        assert!(block_on(nwk.nlde_data_request(child, 5, &[0x01], false, false)).is_ok());
        assert!(nwk.mac.tx_history().is_empty());
        assert!(nwk.indirect_queue().has_pending(child));
        assert_eq!(
            nwk.mac.indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, child), true))
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn relaying_to_a_sleeping_child_buffers_and_arms_frame_pending() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        nwk.nib.permit_joining = true;
        // Capability information without the rx-on-when-idle bit: a real
        // sleeping end device that joined through us.
        let child = nwk
            .handle_child_association([7u8; 8], 0x80)
            .expect("the child associates");
        let header = frame(NwkFrameType::Data, PEER, child);
        let mut buf = [0u8; 128];
        let len = encode(&header, &[0x01], &mut buf);

        assert!(block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert!(
            nwk.mac.tx_history().is_empty(),
            "a sleeping child cannot receive a direct relay"
        );
        assert!(
            nwk.indirect_queue().has_pending(child),
            "the frame must remain queued until the child polls"
        );
        assert_eq!(
            nwk.mac.indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, child), true))
        );
    }
    // ── Route Request propagation across routers ─────────────

    #[test]
    #[cfg(feature = "router")]
    fn a_forwarded_route_request_keeps_the_originators_header_and_decrements_the_radius() {
        // A originated the discovery; B transmitted the copy we hear. Only B
        // is our neighbour — A may be several hops away.
        let mut router = node(DeviceType::Router, OUR_ADDR);
        neighbour_with_cost(&mut router, PEER, 2);
        let on_air = rreq_on_air(ORIGIN, 0x5A, 5, &standard_rreq(9, NEXT_HOP, 3));

        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(PEER))).is_none()
        );
        block_on(router.process_pending_routing());

        assert_eq!(
            router.mac.tx_history().len(),
            3,
            "the request is relayed once and retried twice"
        );
        let (header, forwarded) = parse_rreq(router.mac.tx_history()[0].payload.as_slice());
        assert_eq!(
            header.src_addr, ORIGIN,
            "the forward stays A's broadcast, not a new one of ours"
        );
        assert_eq!(
            header.seq_number, 0x5A,
            "A's NWK sequence number is preserved so receivers can suppress duplicates"
        );
        assert_eq!(
            header.radius, 4,
            "the received radius is decremented, never reset to the broadcast default"
        );
        assert_eq!(header.frame_control.frame_type, NwkFrameType::Command as u8);
        assert_eq!(forwarded.route_request_id, 9);
        assert_eq!(forwarded.dst_addr, NEXT_HOP);
        assert_eq!(
            forwarded.path_cost,
            3 + 2,
            "only the path cost changes: the previous hop's cost plus our link"
        );
        assert_eq!(
            router.routing.next_hop(ORIGIN),
            Some(PEER),
            "the reverse route to the originator points at the previous hop"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_equal_cost_route_request_copy_is_admitted_but_a_costlier_return_is_suppressed() {
        let mut first = node(DeviceType::Router, OUR_ADDR);
        neighbour_with_cost(&mut first, PEER, 1);
        let on_air = rreq_on_air(ORIGIN, 0x5A, 5, &standard_rreq(9, NEXT_HOP, 0));
        assert!(block_on(first.process_incoming_nwk_frame_from(&on_air, 42, Some(PEER))).is_none());
        block_on(first.process_pending_routing());
        let forwarded = next_transmission(&first, &mut 0).expect("the first router forwards");

        // The next router hears an equal-cost retry through another neighbour.
        // R22 drops only a strictly greater path cost, so both copies relay.
        let mut second = node(DeviceType::Router, FAR);
        neighbour_with_cost(&mut second, OUR_ADDR, 1);
        neighbour_with_cost(&mut second, NEXT_HOP, 1);
        assert!(
            block_on(second.process_incoming_nwk_frame_from(&forwarded, 42, Some(OUR_ADDR)))
                .is_none()
        );
        block_on(second.process_pending_routing());
        assert_eq!(second.mac.tx_history().len(), 3);
        assert!(
            !second.btr.is_duplicate(ORIGIN, 0x5A),
            "Route Requests use their discovery record so a better alternate path remains admissible"
        );
        assert!(
            block_on(second.process_incoming_nwk_frame_from(&forwarded, 42, Some(NEXT_HOP)))
                .is_none()
        );
        block_on(second.process_pending_routing());

        assert_eq!(
            second.mac.tx_history().len(),
            6,
            "an equal-cost alternate copy is relayed with its two retries"
        );

        // Once that forward returns to the first router its accumulated cost
        // is greater, so it cannot restart the flood.
        let costlier_return = recorded_frame(&second, 3);
        assert!(
            block_on(first.process_incoming_nwk_frame_from(
                costlier_return.as_slice(),
                42,
                Some(FAR),
            ))
            .is_none()
        );
        block_on(first.process_pending_routing());
        assert_eq!(first.mac.tx_history().len(), 3);
    }

    #[test]
    #[cfg(feature = "router")]
    fn two_routers_do_not_ping_pong_a_costlier_route_request() {
        let mut b = node(DeviceType::Router, OUR_ADDR);
        let mut c = node(DeviceType::Router, FAR);
        neighbour_with_cost(&mut b, ORIGIN, 1);
        neighbour_with_cost(&mut b, FAR, 1);
        neighbour_with_cost(&mut c, OUR_ADDR, 1);

        // A's request reaches B first.
        let a_request = rreq_on_air(ORIGIN, 0x11, 8, &standard_rreq(4, NEXT_HOP, 0));
        assert!(
            block_on(b.process_incoming_nwk_frame_from(&a_request, 42, Some(ORIGIN))).is_none()
        );
        block_on(b.process_pending_routing());
        assert_eq!(b.mac.tx_history().len(), 3);

        // One of B's relays reaches C and is forwarded with a strictly larger
        // path cost. When that copy returns to B it is rejected.
        let from_b = recorded_frame(&b, 0);
        let _ = block_on(c.process_incoming_nwk_frame_from(from_b.as_slice(), 42, Some(OUR_ADDR)));
        block_on(c.process_pending_routing());
        assert_eq!(c.mac.tx_history().len(), 3);
        let from_c = recorded_frame(&c, 0);
        let _ = block_on(b.process_incoming_nwk_frame_from(from_c.as_slice(), 42, Some(FAR)));
        block_on(b.process_pending_routing());

        assert_eq!(
            b.mac.tx_history().len(),
            3,
            "the costlier returning copy does not restart B's relay"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn many_to_one_routes_point_at_the_previous_hop_on_every_hop() {
        // First hop: the concentrator's own broadcast.
        let mut first = node(DeviceType::Router, OUR_ADDR);
        neighbour_with_cost(&mut first, CONCENTRATOR, 1);
        let on_air = rreq_on_air(CONCENTRATOR, 0x21, 5, &many_to_one_rreq(3, 0));
        assert!(
            block_on(first.process_incoming_nwk_frame_from(&on_air, 42, Some(CONCENTRATOR)))
                .is_none()
        );
        block_on(first.process_pending_routing());

        let entry = first
            .routing
            .get_entry(CONCENTRATOR)
            .expect("the concentrator route is installed");
        assert_eq!(entry.next_hop, CONCENTRATOR, "one hop away: direct");
        assert!(entry.many_to_one);
        assert!(entry.route_record_required, "low-RAM concentrator");

        // Second hop: the same request, heard from the first router.
        let forwarded = next_transmission(&first, &mut 0).expect("the first router forwards");
        let (header, _) = parse_rreq(&forwarded);
        assert_eq!(
            header.src_addr, CONCENTRATOR,
            "the forward still names the concentrator as NWK source"
        );

        let mut second = node(DeviceType::Router, FAR);
        neighbour_with_cost(&mut second, OUR_ADDR, 1);
        assert!(
            block_on(second.process_incoming_nwk_frame_from(&forwarded, 42, Some(OUR_ADDR)))
                .is_none()
        );

        let entry = second
            .routing
            .get_entry(CONCENTRATOR)
            .expect("the second hop installs a concentrator route too");
        assert_eq!(
            entry.next_hop, OUR_ADDR,
            "traffic to the concentrator goes through the router we heard it from, \
             not straight at the concentrator's address"
        );
        assert!(entry.many_to_one);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_route_reply_goes_to_the_previous_hop_and_names_the_original_originator() {
        use crate::frames::RouteReply;

        // We are the destination of a discovery that A started and B relayed.
        let mut router = node(DeviceType::Router, OUR_ADDR);
        neighbour_with_cost(&mut router, PEER, 1);
        let on_air = rreq_on_air(ORIGIN, 0x33, 5, &standard_rreq(6, OUR_ADDR, 4));

        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 42, Some(PEER))).is_none()
        );

        assert_eq!(router.pending_route_replies.len(), 1);
        assert_eq!(router.pending_route_replies[0].next_hop, PEER);
        assert_eq!(router.pending_route_replies[0].originator, ORIGIN);
        assert_eq!(
            router.routing.next_hop(ORIGIN),
            Some(PEER),
            "the reverse route the reply follows points at the previous hop"
        );
        assert!(
            router.pending_rreq_forwards.is_empty(),
            "a request we can answer is not propagated further"
        );

        block_on(router.process_pending_routing());

        let replies: std::vec::Vec<_> = router
            .mac
            .tx_history()
            .iter()
            .filter(|record| tx_short_dst(record) == Some(PEER))
            .collect();
        assert_eq!(
            replies.len(),
            1,
            "exactly one RREP is unicast to the previous hop"
        );
        let bytes = replies[0].payload.as_slice();
        let (header, consumed) = NwkHeader::parse(bytes).expect("the reply parses");
        assert_eq!(
            header.dst_addr, PEER,
            "the reply is addressed to the previous hop"
        );
        assert_eq!(bytes[consumed], NwkCommandId::RouteReply as u8);
        let reply = RouteReply::parse(&bytes[consumed + 1..]).expect("the RREP parses");
        assert_eq!(
            reply.originator, ORIGIN,
            "the reply still names the device that started the discovery"
        );
        assert_eq!(reply.responder, OUR_ADDR);
        assert_eq!(reply.route_request_id, 6);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_route_request_received_with_radius_one_is_handled_but_never_forwarded() {
        let mut router = node(DeviceType::Router, OUR_ADDR);
        neighbour_with_cost(&mut router, CONCENTRATOR, 1);
        let on_air = rreq_on_air(CONCENTRATOR, 0x44, 1, &many_to_one_rreq(5, 0));

        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 42, Some(CONCENTRATOR)))
                .is_none()
        );

        assert!(
            router.routing.get_entry(CONCENTRATOR).is_some(),
            "the last hop still installs its own route to the concentrator"
        );
        assert!(
            router.pending_rreq_forwards.is_empty(),
            "a request received with radius 1 has reached the bound the originator set"
        );
        block_on(router.process_pending_routing());
        assert!(
            router.mac.tx_history().is_empty(),
            "and nothing reaches the air on the next maintenance pass"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_repeated_route_request_cannot_replace_a_better_route_or_restart_propagation() {
        let mut router = node(DeviceType::Router, OUR_ADDR);
        neighbour_with_cost(&mut router, PEER, 2);
        neighbour_with_cost(&mut router, NEXT_HOP, 1);

        let first = rreq_on_air(CONCENTRATOR, 0x51, 5, &many_to_one_rreq(7, 0));
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&first, 255, Some(PEER))).is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.routing.get_entry(CONCENTRATOR).unwrap().next_hop,
            PEER
        );
        assert_eq!(router.mac.tx_history().len(), 3);
        assert_eq!(
            router.rreq_records.recorded_cost(CONCENTRATOR, 7),
            Some(2),
            "the best cost this discovery arrived with is remembered"
        );

        // The originator retried with a fresh NWK sequence number, so the
        // broadcast transaction record cannot suppress it — but the path it
        // travelled is worse, so it must change nothing.
        let worse = rreq_on_air(CONCENTRATOR, 0x52, 5, &many_to_one_rreq(7, 5));
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&worse, 255, Some(NEXT_HOP))).is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.routing.get_entry(CONCENTRATOR).unwrap().next_hop,
            PEER,
            "a worse copy of the same discovery must not overwrite the installed route"
        );
        assert_eq!(
            router.mac.tx_history().len(),
            3,
            "and it must not be propagated a second time"
        );

        // A strictly better path is adopted and relayed with two retries.
        let better = rreq_on_air(CONCENTRATOR, 0x53, 5, &many_to_one_rreq(7, 0));
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&better, 255, Some(NEXT_HOP)))
                .is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.routing.get_entry(CONCENTRATOR).unwrap().next_hop,
            NEXT_HOP,
            "a strictly cheaper path replaces the installed next hop"
        );
        assert_eq!(router.mac.tx_history().len(), 6);
        assert_eq!(router.rreq_records.recorded_cost(CONCENTRATOR, 7), Some(1));
    }

    // ── Route Record path recording ──────────────────────────

    #[test]
    #[cfg(feature = "router")]
    fn an_automatic_route_record_uses_the_counter_before_its_data_frame() {
        let mut origin = secured_node(DeviceType::Router, ORIGIN, ORIGIN_IEEE);
        assert!(
            origin
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );
        origin
            .routing
            .update_route_many_to_one(
                CONCENTRATOR,
                CONCENTRATOR,
                1,
                crate::routing::ConcentratorType::HighRam,
            )
            .unwrap();

        block_on(origin.nlde_data_request(CONCENTRATOR, 5, &[0xAB], true, false))
            .expect("Route Record and data are transmitted");

        assert_eq!(origin.mac.tx_history().len(), 2);
        let (_, route_record_aux, route_record) = decrypt_recorded(&recorded_frame(&origin, 0));
        let (_, data_aux, data) = decrypt_recorded(&recorded_frame(&origin, 1));
        assert_eq!(route_record[0], NwkCommandId::RouteRecord as u8);
        assert_eq!(data.as_slice(), &[0xAB]);
        assert_eq!(route_record_aux.frame_counter, RESERVED_FLOOR);
        assert_eq!(data_aux.frame_counter, RESERVED_FLOOR + 1);
        assert!(
            !origin
                .routing
                .get_entry(CONCENTRATOR)
                .unwrap()
                .route_record_required,
            "a high-RAM concentrator caches the successfully sent record"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_low_ram_concentrator_gets_a_route_record_before_every_data_frame() {
        let mut origin = secured_node(DeviceType::Router, ORIGIN, ORIGIN_IEEE);
        assert!(
            origin
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );
        origin
            .routing
            .update_route_many_to_one(
                CONCENTRATOR,
                CONCENTRATOR,
                1,
                crate::routing::ConcentratorType::LowRam,
            )
            .unwrap();

        for payload in [[0xA1], [0xA2]] {
            block_on(origin.nlde_data_request(CONCENTRATOR, 5, &payload, true, false))
                .expect("each data frame follows its Route Record");
        }

        assert_eq!(origin.mac.tx_history().len(), 4);
        for (index, counter) in (RESERVED_FLOOR..RESERVED_FLOOR + 4).enumerate() {
            let (_, aux, payload) = decrypt_recorded(&recorded_frame(&origin, index));
            assert_eq!(aux.frame_counter, counter);
            if index % 2 == 0 {
                assert_eq!(payload[0], NwkCommandId::RouteRecord as u8);
            }
        }
        assert!(
            origin
                .routing
                .get_entry(CONCENTRATOR)
                .unwrap()
                .route_record_required,
            "low-RAM concentrators require a record for the next frame too"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_parent_originates_a_child_route_record_before_relaying_mtor_data() {
        let child_ieee = [0xA5; 8];
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.nib.permit_joining = true;
        let child = parent
            .handle_child_association(child_ieee, 0x80)
            .expect("the child associates");
        parent
            .neighbors
            .find_by_short_mut(child)
            .expect("the child is present")
            .relationship = crate::neighbor::Relationship::Child;
        parent
            .routing
            .update_route_many_to_one(
                CONCENTRATOR,
                NEXT_HOP,
                2,
                crate::routing::ConcentratorType::HighRam,
            )
            .unwrap();

        let header = frame(NwkFrameType::Data, child, CONCENTRATOR);
        let mut bytes = [0u8; 128];
        let len = encode(&header, &[0xAA], &mut bytes);
        assert!(
            block_on(parent.process_incoming_nwk_frame_from(&bytes[..len], 255, Some(child),))
                .is_none()
        );

        assert_eq!(
            parent.mac.tx_history().len(),
            2,
            "the child Route Record precedes the relayed data"
        );
        let route_record = parent.mac.tx_history()[0].payload.as_slice();
        let (route_record_header, consumed) =
            NwkHeader::parse(route_record).expect("the Route Record parses");
        assert_eq!(route_record_header.src_addr, child);
        assert_eq!(route_record_header.src_ieee, Some(child_ieee));
        assert_eq!(route_record_header.dst_addr, CONCENTRATOR);
        assert_eq!(
            route_record_relays(&route_record[consumed..]).as_slice(),
            [OUR_ADDR].as_slice(),
            "the parent is the first relay in the child's record"
        );

        let data = parent.mac.tx_history()[1].payload.as_slice();
        let (data_header, data_consumed) = NwkHeader::parse(data).expect("the data parses");
        assert_eq!(data_header.src_addr, child);
        assert_eq!(&data[data_consumed..], &[0xAA]);

        parent.mac.clear_tx_history();
        parent
            .neighbors
            .find_by_short_mut(child)
            .expect("the child is present")
            .device_type = crate::neighbor::NeighborDeviceType::Router;
        assert!(
            block_on(parent.process_incoming_nwk_frame_from(&bytes[..len], 255, Some(child),))
                .is_none()
        );
        assert_eq!(
            parent.mac.tx_history().len(),
            1,
            "a router child originates its own Route Record"
        );

        parent.mac.clear_tx_history();
        parent.mac.set_tx_failures(1);
        parent
            .neighbors
            .find_by_short_mut(child)
            .expect("the child is present")
            .device_type = crate::neighbor::NeighborDeviceType::EndDevice;
        assert!(
            block_on(parent.process_incoming_nwk_frame_from(&bytes[..len], 255, Some(child),))
                .is_none()
        );
        assert_eq!(
            parent.mac.tx_history().len(),
            1,
            "a failed child Route Record does not discard the relayed data"
        );
        let relayed_data = parent.mac.tx_history()[0].payload.as_slice();
        let (relayed_header, relayed_consumed) =
            NwkHeader::parse(relayed_data).expect("the relayed data parses");
        assert_eq!(relayed_header.src_addr, child);
        assert_eq!(&relayed_data[relayed_consumed..], &[0xAA]);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_relayed_route_record_appends_each_router_exactly_once() {
        const RELAY_B: ShortAddress = FAR;
        const CONCENTRATOR_IEEE: IeeeAddress = [0xD0; 8];

        // The device whose path is being recorded starts with an empty list.
        let mut origin = secured_node(DeviceType::Router, ORIGIN, ORIGIN_IEEE);
        origin
            .routing
            .update_route(CONCENTRATOR, OUR_ADDR, 2)
            .unwrap();
        block_on(origin.send_route_record(CONCENTRATOR, &[]))
            .expect("the originator sends its Route Record");
        let originated = recorded_frame(&origin, 0);
        let (_, _, payload) = decrypt_recorded(&originated);
        assert!(
            route_record_relays(&payload).is_empty(),
            "the originator records no relays of its own"
        );

        // First intermediate router.
        let mut relay_a = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        assert!(
            relay_a
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );
        relay_a
            .routing
            .update_route(CONCENTRATOR, RELAY_B, 1)
            .unwrap();
        assert!(
            block_on(relay_a.process_incoming_nwk_frame_from(&originated, 42, Some(ORIGIN)))
                .is_none()
        );
        assert_eq!(
            relay_a.rx_security_stats().decrypt_successes,
            1,
            "the record is authenticated before this router extends it"
        );
        assert_eq!(relay_a.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&relay_a.mac.tx_history()[0]), Some(RELAY_B));

        let from_a = recorded_frame(&relay_a, 0);
        let (header_a, aux_a, payload_a) = decrypt_recorded(&from_a);
        assert_eq!(
            header_a.src_addr, ORIGIN,
            "the record still names the device whose path it describes"
        );
        assert_eq!(header_a.dst_addr, CONCENTRATOR);
        assert_eq!(header_a.radius, 9, "the relay decrements the radius");
        assert_eq!(
            aux_a.source_address, RELAY_IEEE,
            "NWK security is hop by hop: the relay re-encrypts with its own IEEE address"
        );
        assert_eq!(
            aux_a.frame_counter, RESERVED_FLOOR,
            "and spends its own durably reserved counter"
        );
        assert_eq!(
            route_record_relays(&payload_a).as_slice(),
            [OUR_ADDR].as_slice(),
            "the first router appended itself exactly once"
        );

        // Second intermediate router.
        let mut relay_b = secured_node(DeviceType::Router, RELAY_B, DEST_IEEE);
        relay_b
            .routing
            .update_route(CONCENTRATOR, CONCENTRATOR, 1)
            .unwrap();
        assert!(
            block_on(relay_b.process_incoming_nwk_frame_from(&from_a, 42, Some(OUR_ADDR)))
                .is_none()
        );
        assert_eq!(
            relay_b.rx_security_stats().decrypt_successes,
            1,
            "the next hop authenticates the re-encrypted command"
        );
        assert_eq!(relay_b.mac.tx_history().len(), 1);
        assert_eq!(
            tx_short_dst(&relay_b.mac.tx_history()[0]),
            Some(CONCENTRATOR)
        );

        let from_b = recorded_frame(&relay_b, 0);
        let (_, _, payload_b) = decrypt_recorded(&from_b);
        assert_eq!(
            route_record_relays(&payload_b).as_slice(),
            [OUR_ADDR, RELAY_B].as_slice(),
            "every router appears exactly once, in the order the record travelled"
        );

        // The concentrator consumes the final record into source-route state.
        let mut concentrator =
            secured_node(DeviceType::Coordinator, CONCENTRATOR, CONCENTRATOR_IEEE);
        assert!(
            block_on(concentrator.process_incoming_nwk_frame_from(&from_b, 42, Some(RELAY_B)))
                .is_none()
        );
        assert_eq!(concentrator.rx_security_stats().decrypt_successes, 1);
        assert!(
            concentrator.mac.tx_history().is_empty(),
            "the record stops at the device it was addressed to"
        );
        assert_eq!(
            concentrator.source_route_table.lookup(ORIGIN),
            Some([OUR_ADDR, RELAY_B].as_slice()),
            "the stored path is destination-nearest first"
        );
        assert_eq!(
            concentrator.routing.next_hop(ORIGIN),
            Some(RELAY_B),
            "and the routing table names that same first hop"
        );
        assert_eq!(
            concentrator.source_route_table.lookup(OUR_ADDR),
            Some([RELAY_B].as_slice()),
            "the destination-nearest intermediary gets the remaining suffix"
        );
        assert_eq!(
            concentrator.source_route_table.lookup(RELAY_B),
            Some([].as_slice()),
            "the concentrator-nearest relay is stored as a direct route"
        );
        assert_eq!(concentrator.routing.next_hop(OUR_ADDR), Some(RELAY_B));
        assert_eq!(concentrator.routing.next_hop(RELAY_B), Some(RELAY_B));
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_route_record_relay_failure_is_silent() {
        let mut relay = node(DeviceType::Router, OUR_ADDR);
        relay
            .routing
            .update_route(CONCENTRATOR, NEXT_HOP, 1)
            .unwrap();
        relay.mac.set_tx_failures(1);
        let header = frame(NwkFrameType::Command, ORIGIN, CONCENTRATOR);
        let payload = [NwkCommandId::RouteRecord as u8, 0];

        assert_eq!(
            block_on(relay.relay_route_record(&header, &payload)).unwrap_err(),
            NwkStatus::RouteError
        );
        assert!(
            relay.pending_route_errors.is_empty(),
            "R22 forbids a Network Status response to a failed Route Record"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_source_routed_return_clears_only_a_cached_route_record_requirement() {
        for (concentrator_type, remains_required) in [
            (crate::routing::ConcentratorType::HighRam, false),
            (crate::routing::ConcentratorType::LowRam, true),
        ] {
            let mut relay = node(DeviceType::Router, OUR_ADDR);
            relay
                .routing
                .update_route_many_to_one(CONCENTRATOR, PEER, 1, concentrator_type)
                .unwrap();

            let mut header = frame(NwkFrameType::Data, CONCENTRATOR, FAR);
            header.frame_control.source_route = true;
            let mut relay_list = heapless::Vec::new();
            relay_list.push(OUR_ADDR).unwrap();
            header.source_route = Some(crate::frames::SourceRoute {
                relay_count: 1,
                relay_index: 0,
                relay_list,
            });

            block_on(relay.relay_frame_source_routed(&header, &[0x01], 4)).unwrap();
            assert_eq!(
                relay
                    .routing
                    .get_entry(CONCENTRATOR)
                    .unwrap()
                    .route_record_required,
                remains_required
            );
        }
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_route_record_longer_than_this_device_can_store_is_refused_not_truncated() {
        let mut concentrator = node(DeviceType::Coordinator, CONCENTRATOR);
        let relays = crate::routing::MAX_SOURCE_ROUTE_RELAYS + 1;
        let mut body = [0u8; 1 + 2 * (crate::routing::MAX_SOURCE_ROUTE_RELAYS + 1)];
        body[0] = relays as u8;
        for i in 0..relays {
            body[1 + i * 2] = (i + 1) as u8;
        }
        let mut payload = [0u8; 32];
        let payload_len = command_payload(NwkCommandId::RouteRecord, &body, &mut payload);
        let mut buf = [0u8; 128];
        let len = encode(
            &frame(NwkFrameType::Command, PEER, CONCENTRATOR),
            &payload[..payload_len],
            &mut buf,
        );

        assert!(block_on(concentrator.process_incoming_nwk_frame(&buf[..len], 42)).is_none());

        assert!(
            concentrator.source_route_table.lookup(PEER).is_none(),
            "a path this device cannot hold must not be stored with its tail cut off"
        );
        assert_eq!(concentrator.routing.next_hop(PEER), None);
    }

    // ── Source routing from a concentrator ───────────────────

    #[test]
    #[cfg(feature = "router")]
    fn a_source_routed_frame_walks_the_relay_list_from_its_last_entry() {
        // The path a Route Record established: destination-nearest first,
        // concentrator-nearest last.
        let mut concentrator = node(DeviceType::Coordinator, CONCENTRATOR);
        concentrator.start_concentrator(crate::routing::ConcentratorType::LowRam, 60, 5);
        concentrator
            .source_route_table
            .insert(FAR, &[NEXT_HOP, OUR_ADDR]);
        concentrator.routing.update_route(FAR, OUR_ADDR, 2).unwrap();

        block_on(concentrator.nlde_data_request(FAR, 5, &[0xAB], false, false))
            .expect("the concentrator sends over its stored path");

        assert_eq!(concentrator.mac.tx_history().len(), 1);
        assert_eq!(
            tx_short_dst(&concentrator.mac.tx_history()[0]),
            Some(OUR_ADDR),
            "the first hop is the last entry, closest to the concentrator"
        );
        let originated = recorded_frame(&concentrator, 0);
        let (header, _) = NwkHeader::parse(&originated).expect("the originated frame parses");
        let sr = header
            .source_route
            .as_ref()
            .expect("the frame carries its path");
        assert_eq!(sr.relay_count, 2);
        assert_eq!(
            sr.relay_index, 1,
            "the index names the relay that carries the frame next"
        );

        // First relay: hands it on, decrementing the index.
        let mut first = node(DeviceType::Router, OUR_ADDR);
        assert!(block_on(first.process_incoming_nwk_frame(&originated, 42)).is_none());
        assert_eq!(first.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&first.mac.tx_history()[0]), Some(NEXT_HOP));
        let at_second = recorded_frame(&first, 0);
        let (header, _) = NwkHeader::parse(&at_second).expect("the relayed frame parses");
        assert_eq!(header.radius, 4);
        assert_eq!(header.source_route.as_ref().unwrap().relay_index, 0);

        // Last relay: the list is exhausted, so the destination is next.
        let mut second = node(DeviceType::Router, NEXT_HOP);
        assert!(block_on(second.process_incoming_nwk_frame(&at_second, 42)).is_none());
        assert_eq!(second.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&second.mac.tx_history()[0]), Some(FAR));
        let at_destination = recorded_frame(&second, 0);
        let (header, _) = NwkHeader::parse(&at_destination).expect("the last hop parses");
        assert_eq!(
            header.source_route.as_ref().unwrap().relay_index,
            0,
            "the last relay leaves the index at zero for the destination hop"
        );

        // The destination delivers it instead of relaying it again.
        let mut destination = node(DeviceType::Router, FAR);
        match block_on(destination.process_incoming_nwk_frame(&at_destination, 42)) {
            Some(NwkIndication::Borrowed(data)) => {
                assert_eq!(data.src_addr, CONCENTRATOR);
                assert_eq!(data.payload, &[0xAB]);
            }
            other => panic!("expected local delivery, got {other:?}"),
        }
        assert!(destination.mac.tx_history().is_empty());
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_stored_path_with_no_relays_is_sent_straight_to_the_destination() {
        let mut concentrator = node(DeviceType::Coordinator, CONCENTRATOR);
        concentrator.start_concentrator(crate::routing::ConcentratorType::LowRam, 60, 5);
        // A Route Record from a direct neighbour lists no relays at all.
        concentrator.source_route_table.insert(FAR, &[]);
        concentrator.routing.update_route(FAR, FAR, 1).unwrap();

        block_on(concentrator.nlde_data_request(FAR, 5, &[0xAB], false, false))
            .expect("a neighbour needs no source route");

        assert_eq!(concentrator.mac.tx_history().len(), 1);
        assert_eq!(tx_short_dst(&concentrator.mac.tx_history()[0]), Some(FAR));
        let originated = recorded_frame(&concentrator, 0);
        let (header, _) = NwkHeader::parse(&originated).expect("the originated frame parses");
        assert!(
            !header.frame_control.source_route,
            "an empty path attaches no source route subframe"
        );
        assert!(header.source_route.is_none());
    }

    // ── Broadcast address classification ─────────────────────

    #[test]
    #[cfg(feature = "router")]
    fn reserved_and_unassigned_destinations_are_neither_delivered_nor_relayed() {
        for dst in [0xFFF8u16, 0xFFF9, 0xFFFA, 0xFFFE] {
            let mut nwk = node(DeviceType::Router, OUR_ADDR);
            let mut buf = [0u8; 128];
            let len = encode(
                &frame(NwkFrameType::Data, PEER, ShortAddress(dst)),
                &[0x55],
                &mut buf,
            );

            assert!(
                block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)).is_none(),
                "0x{dst:04X} names no device and no broadcast group"
            );
            assert!(
                nwk.mac.tx_history().is_empty(),
                "0x{dst:04X} must be neither rebroadcast nor relayed"
            );
            assert!(
                !nwk.btr.is_duplicate(PEER, 7),
                "0x{dst:04X} must not enter the broadcast transaction record"
            );

            // Control: the genuine broadcast carrying the same source and
            // sequence number is still delivered and rebroadcast.
            let len = encode(
                &frame(NwkFrameType::Data, PEER, ShortAddress::BROADCAST),
                &[0x55],
                &mut buf,
            );
            assert!(matches!(
                block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42)),
                Some(NwkIndication::Borrowed(_))
            ));
            assert_eq!(nwk.mac.tx_history().len(), 1);
        }
    }

    #[test]
    #[cfg(feature = "router")]
    fn every_defined_broadcast_address_is_still_rebroadcast() {
        for (dst, delivered) in [
            (0xFFFFu16, true),
            (0xFFFD, true),
            (0xFFFC, true),
            (0xFFFB, false),
        ] {
            let mut nwk = node(DeviceType::Router, OUR_ADDR);
            let mut buf = [0u8; 128];
            let len = encode(
                &frame(NwkFrameType::Data, PEER, ShortAddress(dst)),
                &[0x55],
                &mut buf,
            );

            let indication = block_on(nwk.process_incoming_nwk_frame(&buf[..len], 42));
            assert_eq!(
                indication.is_some(),
                delivered,
                "0x{dst:04X} local delivery follows the addressed device set"
            );
            assert_eq!(
                nwk.mac.tx_history().len(),
                1,
                "0x{dst:04X} is a defined broadcast and is carried further"
            );
            assert!(
                nwk.btr.is_duplicate(PEER, 7),
                "0x{dst:04X} is recorded so its duplicates are suppressed"
            );
        }
    }

    #[test]
    fn sending_to_a_reserved_address_is_refused_instead_of_broadcast() {
        for dst in [0xFFF8u16, 0xFFF9, 0xFFFA, 0xFFFE] {
            let mut nwk = node(DeviceType::Router, OUR_ADDR);
            let result =
                block_on(nwk.nlde_data_request(ShortAddress(dst), 5, &[0x01], false, true));

            assert_eq!(
                result.err(),
                Some(NwkStatus::InvalidParameter),
                "0x{dst:04X} is not a destination this stack can send to"
            );
            assert!(nwk.mac.tx_history().is_empty());
            assert!(
                !nwk.routing.has_active_discovery(ShortAddress(dst)),
                "and it must not start a route discovery either"
            );
        }
    }

    // ── Route discovery startup failures ─────────────────────

    #[test]
    #[cfg(feature = "router")]
    fn a_route_request_that_cannot_be_secured_leaves_no_discovery_behind() {
        let mut nwk = node(DeviceType::Router, OUR_ADDR);
        // A secured network, but this device holds no network key yet.
        nwk.nib.security_enabled = true;

        assert_eq!(
            block_on(nwk.discover_route(FAR)).err(),
            Some(NwkStatus::NoKey)
        );
        assert!(nwk.mac.tx_history().is_empty());
        assert!(
            !nwk.routing.has_active_discovery(FAR),
            "a discovery that never reached the air must not suppress the retry"
        );

        // The immediate retry is allowed, and reaches the air once the key is
        // installed.
        nwk.security.set_network_key(NETWORK_KEY, KEY_SEQ);
        nwk.nib.active_key_seq_number = KEY_SEQ;
        assert!(block_on(nwk.discover_route(FAR)).is_ok());
        assert_eq!(nwk.mac.tx_history().len(), 1);
        assert!(nwk.routing.has_active_discovery(FAR));
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_route_request_without_counter_space_leaves_no_discovery_behind() {
        let mut nwk = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        // A reservation with nothing left in it.
        assert!(
            nwk.nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR)
        );

        assert_eq!(
            block_on(nwk.discover_route(FAR)).err(),
            Some(NwkStatus::MaxFrmCounterReached)
        );
        assert!(nwk.mac.tx_history().is_empty());
        assert_eq!(
            nwk.nib.outgoing_frame_counter, RESERVED_FLOOR,
            "a request that is never transmitted must not burn counter space"
        );
        assert!(
            !nwk.routing.has_active_discovery(FAR),
            "and it must not leave a discovery record blocking the retry"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_transmitted_route_request_keeps_its_discovery_and_suppresses_a_retry() {
        let mut nwk = node(DeviceType::Coordinator, CONCENTRATOR);
        nwk.nib.parent_address = ShortAddress(0xFFFF);

        assert!(block_on(nwk.discover_route(FAR)).is_ok());
        assert_eq!(nwk.mac.tx_history().len(), 1);
        assert!(
            nwk.routing.has_active_discovery(FAR),
            "a request that went out is waited for"
        );

        // A data request for the same destination rides the outstanding
        // discovery instead of flooding a second one.
        assert_eq!(
            block_on(nwk.nlde_data_request(FAR, 30, &[0x01], false, true)).err(),
            Some(NwkStatus::RouteDiscoveryFailed)
        );
        assert_eq!(
            nwk.mac.tx_history().len(),
            1,
            "an outstanding discovery must not be restarted per frame"
        );
    }

    // ── Alternate Route Request copies ───────────────────────

    #[test]
    #[cfg(feature = "router")]
    fn an_alternate_route_request_copy_is_judged_on_cost_not_on_the_broadcast_record() {
        const THIRD_HOP: ShortAddress = ShortAddress(0x6666);

        let mut router = node(DeviceType::Router, OUR_ADDR);
        neighbour_with_cost(&mut router, PEER, 5);
        neighbour_with_cost(&mut router, THIRD_HOP, 5);
        neighbour_with_cost(&mut router, NEXT_HOP, 1);

        // One discovery, heard through three different neighbours: same
        // originator, same NWK sequence number, same request ID. The broadcast
        // transaction record cannot tell these copies apart at all.
        let on_air = rreq_on_air(CONCENTRATOR, 0x60, 5, &many_to_one_rreq(3, 0));

        // The first copy installs the route it arrived over.
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(PEER))).is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.routing.get_entry(CONCENTRATOR).unwrap().next_hop,
            PEER
        );
        assert_eq!(router.mac.tx_history().len(), 3);
        assert_eq!(router.rreq_records.recorded_cost(CONCENTRATOR, 3), Some(5));

        // The same neighbor's MAC retry is not a new equal-cost path and must
        // not start another three-transmission propagation round.
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(PEER))).is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.mac.tx_history().len(),
            3,
            "same-neighbor retries are suppressed by the discovery record"
        );

        // R22 admits an equal-cost copy and updates the sender address.
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(THIRD_HOP)))
                .is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.routing.get_entry(CONCENTRATOR).unwrap().next_hop,
            THIRD_HOP,
            "an equal-cost copy updates the recorded sender"
        );
        assert_eq!(
            router.mac.tx_history().len(),
            6,
            "and is relayed with the required retries"
        );

        // Interleaved retries from the first equal-cost neighbor remain
        // suppressed after the alternate neighbor was accepted.
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(PEER))).is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.mac.tx_history().len(),
            6,
            "all previously accepted equal-cost senders remain deduplicated"
        );

        // A strictly cheaper copy is adopted and propagated once more — the
        // case the broadcast transaction record used to hide.
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(NEXT_HOP)))
                .is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.routing.get_entry(CONCENTRATOR).unwrap().next_hop,
            NEXT_HOP,
            "a strictly better path reaches the cost comparison and wins"
        );
        assert_eq!(router.mac.tx_history().len(), 9);
        assert_eq!(router.rreq_records.recorded_cost(CONCENTRATOR, 3), Some(1));

        // And a worse copy after that is still refused.
        assert!(
            block_on(router.process_incoming_nwk_frame_from(&on_air, 255, Some(PEER))).is_none()
        );
        block_on(router.process_pending_routing());
        assert_eq!(
            router.routing.get_entry(CONCENTRATOR).unwrap().next_hop,
            NEXT_HOP
        );
        assert_eq!(router.mac.tx_history().len(), 9);

        assert!(
            !router.btr.is_duplicate(CONCENTRATOR, 0x60),
            "route requests are deduplicated by their own record, not by the BTR"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_secured_route_request_forward_authenticates_at_the_next_router() {
        // The concentrator's own secured many-to-one request, produced by the
        // real transmit path.
        let mut concentrator = secured_node(DeviceType::Router, ORIGIN, ORIGIN_IEEE);
        concentrator.start_concentrator(crate::routing::ConcentratorType::LowRam, 60, 5);
        block_on(concentrator.send_many_to_one_rreq()).expect("the concentrator sends its RREQ");
        let originated = recorded_frame(&concentrator, 0);
        let (originated_header, _) = NwkHeader::parse(&originated).expect("the RREQ parses");
        assert!(originated_header.frame_control.security);

        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        neighbour_with_cost(&mut relay, ORIGIN, 1);
        assert!(
            relay
                .nib
                .set_frame_counter_reservation(RESERVED_FLOOR, RESERVED_FLOOR + 8)
        );
        assert!(
            block_on(relay.process_incoming_nwk_frame_from(&originated, 42, Some(ORIGIN)))
                .is_none()
        );
        block_on(relay.process_pending_routing());
        assert_eq!(
            relay.rx_security_stats().decrypt_successes,
            1,
            "the request is authenticated before it is acted upon"
        );
        assert_eq!(relay.mac.tx_history().len(), 3);

        let forwarded = recorded_frame(&relay, 0);
        let (header, consumed) = NwkHeader::parse(&forwarded).expect("the forward parses");
        assert_eq!(
            header.src_addr, ORIGIN,
            "a secured forward is still the originator's broadcast"
        );
        assert_eq!(
            header.seq_number, originated_header.seq_number,
            "the originator's sequence number survives re-encryption"
        );
        assert_eq!(header.radius, originated_header.radius - 1);
        let (aux, _) = crate::security::NwkSecurityHeader::parse(&forwarded[consumed..])
            .expect("the forward carries a fresh auxiliary header");
        assert_eq!(
            aux.source_address, RELAY_IEEE,
            "NWK security is hop by hop: the relay signs with its own IEEE address"
        );
        assert_eq!(
            aux.frame_counter, RESERVED_FLOOR,
            "and spends its own durably reserved counter"
        );

        // The next router downstream authenticates and installs its own route.
        let mut next = secured_node(DeviceType::Router, FAR, DEST_IEEE);
        neighbour_with_cost(&mut next, OUR_ADDR, 1);
        assert!(
            block_on(next.process_incoming_nwk_frame_from(&forwarded, 42, Some(OUR_ADDR)))
                .is_none()
        );
        assert_eq!(
            next.rx_security_stats().decrypt_successes,
            1,
            "the re-secured forward decrypts at the next hop"
        );
        assert_eq!(
            next.routing
                .get_entry(ORIGIN)
                .expect("the concentrator route is installed downstream")
                .next_hop,
            OUR_ADDR,
            "the downstream route points at the relay, not at the concentrator"
        );
    }

    #[cfg(feature = "router")]
    fn rejoin_request_on_air(
        dst: ShortAddress,
        radius: u8,
        secured: bool,
        header_ieee: IeeeAddress,
        security_ieee: IeeeAddress,
        capability_info: u8,
    ) -> heapless::Vec<u8, 128> {
        let mut sender = secured_node(DeviceType::EndDevice, ORIGIN, security_ieee);
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: secured,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: true,
                end_device_initiator: true,
            },
            dst_addr: dst,
            src_addr: ORIGIN,
            radius,
            seq_number: 0x5A,
            dst_ieee: None,
            src_ieee: Some(header_ieee),
            multicast_control: None,
            source_route: None,
        };
        let mut frame = [0u8; 128];
        let len = sender
            .build_nwk_frame(
                &header,
                &[NwkCommandId::RejoinRequest as u8, capability_info],
                &mut frame,
            )
            .expect("the request frame builds");
        heapless::Vec::from_slice(&frame[..len]).unwrap()
    }

    #[test]
    #[cfg(feature = "router")]
    fn unsecured_rejoin_is_the_only_local_unsecured_command_exception() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        }
        .to_byte();
        let request =
            rejoin_request_on_air(OUR_ADDR, 1, false, ORIGIN_IEEE, ORIGIN_IEEE, capability);
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);

        assert!(
            block_on(parent.process_incoming_nwk_frame_from(&request, 42, Some(ORIGIN))).is_none()
        );
        assert_eq!(
            parent.take_command_outcome(),
            Some(NwkCommandOutcome::ChildRejoinRequest {
                src: ORIGIN,
                ieee: ORIGIN_IEEE,
                capability_info: capability,
                secured: false,
            })
        );

        let wrong_radius =
            rejoin_request_on_air(OUR_ADDR, 2, false, ORIGIN_IEEE, ORIGIN_IEEE, capability);
        assert!(
            block_on(parent.process_incoming_nwk_frame_from(&wrong_radius, 42, Some(ORIGIN)))
                .is_none()
        );
        assert_eq!(parent.take_command_outcome(), None);

        let mut unsecured_leave = [0u8; 128];
        let mut leave_header = frame(NwkFrameType::Command, ORIGIN, OUR_ADDR);
        leave_header.radius = 1;
        leave_header.src_ieee = Some(ORIGIN_IEEE);
        leave_header.frame_control.src_ieee_present = true;
        let leave_len = encode(
            &leave_header,
            &[NwkCommandId::Leave as u8, 0],
            &mut unsecured_leave,
        );
        assert!(
            block_on(parent.process_incoming_nwk_frame_from(
                &unsecured_leave[..leave_len],
                42,
                Some(ORIGIN),
            ))
            .is_none()
        );
        assert_eq!(parent.take_command_outcome(), None);
    }

    #[test]
    #[cfg(feature = "router")]
    fn secured_rejoin_requires_the_header_and_security_source_ieees_to_match() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: true,
            security_capable: true,
            allocate_address: true,
        }
        .to_byte();
        let request = rejoin_request_on_air(OUR_ADDR, 1, true, ORIGIN_IEEE, RELAY_IEEE, capability);
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, DEST_IEEE);

        assert!(
            block_on(parent.process_incoming_nwk_frame_from(&request, 42, Some(ORIGIN))).is_none()
        );
        assert_eq!(parent.rx_security_stats().decrypt_successes, 1);
        assert_eq!(parent.take_command_outcome(), None);
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_rejoin_request_is_never_relayed() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: true,
            security_capable: true,
            allocate_address: true,
        }
        .to_byte();
        let request = rejoin_request_on_air(FAR, 5, true, ORIGIN_IEEE, ORIGIN_IEEE, capability);
        let mut relay = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        relay.routing.update_route(FAR, NEXT_HOP, 1).unwrap();

        assert!(
            block_on(relay.process_incoming_nwk_frame_from(&request, 42, Some(ORIGIN))).is_none()
        );
        assert!(relay.mac.tx_history().is_empty());
        assert_eq!(relay.take_command_outcome(), None);
    }

    #[test]
    #[cfg(feature = "router")]
    fn conflicting_sleepy_rejoin_response_is_sent_directly() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.nib.permit_joining = true;
        parent.update_neighbor_address(ORIGIN, [0xEE; 8]);
        let assigned = parent
            .handle_child_rejoin(ORIGIN, ORIGIN_IEEE, capability.to_byte(), false)
            .expect("the child is admitted");
        assert_ne!(assigned, ORIGIN, "the old address was already occupied");

        assert_eq!(
            block_on(parent.send_rejoin_response(ORIGIN, ORIGIN_IEEE, assigned, 0, false, false,)),
            Ok(RejoinResponseDelivery::Direct)
        );
        assert!(!parent.indirect_queue().has_pending(ORIGIN));
        let record = &parent.mac.tx_history()[0];
        assert!(!record.indirect);
        assert_eq!(tx_short_dst(record), Some(ORIGIN));
        let bytes = record.payload.as_slice();
        let (header, consumed) = NwkHeader::parse(bytes).expect("the response parses");
        assert_eq!(header.dst_addr, ORIGIN);
        assert_eq!(header.dst_ieee, Some(ORIGIN_IEEE));
        assert_eq!(header.radius, 1);
        assert!(!header.frame_control.security);
        assert_eq!(
            &bytes[consumed..],
            &[
                NwkCommandId::RejoinResponse as u8,
                assigned.0 as u8,
                (assigned.0 >> 8) as u8,
                0,
            ]
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn rejected_rejoin_type_change_preserves_the_existing_child() {
        let end_device = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        };
        let router = CapabilityInfo {
            device_type_ffd: true,
            mains_powered: true,
            rx_on_when_idle: true,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.nib.permit_joining = true;
        let child = parent
            .handle_child_association(ORIGIN_IEEE, end_device.to_byte())
            .unwrap();
        assert!(parent.authorize_child(child));
        parent.nib.permit_joining = false;

        assert_eq!(
            parent.handle_child_rejoin(child, ORIGIN_IEEE, router.to_byte(), false),
            Err(NwkStatus::NotPermitted)
        );
        let entry = parent.neighbors.find_by_ieee(&ORIGIN_IEEE).unwrap();
        assert_eq!(entry.network_address, child);
        assert_eq!(
            entry.device_type,
            crate::neighbor::NeighborDeviceType::EndDevice
        );
        assert_eq!(entry.relationship, crate::neighbor::Relationship::Child);
    }

    #[test]
    #[cfg(feature = "router")]
    fn announced_sibling_association_reuses_one_neighbor_entry() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.nib.permit_joining = true;
        parent.update_neighbor_address(ORIGIN, ORIGIN_IEEE);
        assert_eq!(parent.neighbors.len(), 1);

        let child = parent
            .handle_child_association(ORIGIN_IEEE, capability.to_byte())
            .unwrap();
        assert_eq!(child, ORIGIN);
        assert_eq!(parent.neighbors.len(), 1);
        let entry = parent.neighbors.find_by_ieee(&ORIGIN_IEEE).unwrap();
        assert_eq!(
            entry.relationship,
            crate::neighbor::Relationship::UnauthenticatedChild
        );
        assert_eq!(parent.known_child_by_ieee(&ORIGIN_IEEE), Some(ORIGIN));
    }

    #[test]
    #[cfg(feature = "router")]
    fn repeated_association_requires_fresh_network_key_proof() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.nib.permit_joining = true;
        let child = parent
            .handle_child_association(ORIGIN_IEEE, capability.to_byte())
            .unwrap();
        assert!(parent.authorize_child(child));
        assert!(parent.indirect.enqueue(child, &[0xAA]));
        parent.routing.update_route(child, child, 1).unwrap();
        parent
            .security_mut()
            .commit_frame_counter_for_key(&ORIGIN_IEEE, 0, 100);
        assert!(
            !parent
                .security()
                .check_frame_counter_for_key(&ORIGIN_IEEE, 0, 0)
        );

        assert_eq!(
            parent.handle_child_association(ORIGIN_IEEE, capability.to_byte()),
            Ok(child)
        );
        assert!(!parent.child_is_authorized(&ORIGIN_IEEE));
        assert!(!parent.indirect.has_pending(child));
        assert!(parent.routing.get_entry(child).is_none());
        assert!(
            parent
                .security()
                .check_frame_counter_for_key(&ORIGIN_IEEE, 0, 0)
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn announced_sibling_can_perform_a_secured_rejoin() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.update_neighbor_address(ORIGIN, ORIGIN_IEEE);

        assert_eq!(
            parent.handle_child_rejoin(ORIGIN, ORIGIN_IEEE, capability.to_byte(), true),
            Ok(ORIGIN)
        );
        assert_eq!(parent.neighbors.len(), 1);
        let entry = parent.neighbors.find_by_ieee(&ORIGIN_IEEE).unwrap();
        assert_eq!(entry.relationship, crate::neighbor::Relationship::Child);
    }

    #[test]
    #[cfg(feature = "router")]
    fn unsecured_child_rejoin_clears_stale_replay_state() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.nib.permit_joining = true;
        let child = parent
            .handle_child_association(ORIGIN_IEEE, capability.to_byte())
            .unwrap();
        parent
            .security_mut()
            .commit_frame_counter_for_key(&ORIGIN_IEEE, 0, 100);

        assert_eq!(
            parent.handle_child_rejoin(child, ORIGIN_IEEE, capability.to_byte(), false),
            Ok(child)
        );
        assert!(
            parent
                .security()
                .check_frame_counter_for_key(&ORIGIN_IEEE, 0, 0)
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn secured_child_rejoin_preserves_authenticated_replay_state() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.update_neighbor_address(ORIGIN, ORIGIN_IEEE);
        parent
            .security_mut()
            .commit_frame_counter_for_key(&ORIGIN_IEEE, 0, 100);

        assert_eq!(
            parent.handle_child_rejoin(ORIGIN, ORIGIN_IEEE, capability.to_byte(), true),
            Ok(ORIGIN)
        );
        assert!(
            !parent
                .security()
                .check_frame_counter_for_key(&ORIGIN_IEEE, 0, 1)
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn rejected_sleepy_rejoin_cannot_clear_another_childs_queue() {
        const VICTIM_IEEE: [u8; 8] = [0xEE; 8];
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.update_neighbor_address(ORIGIN, VICTIM_IEEE);
        let victim = parent.neighbors.find_by_short_mut(ORIGIN).unwrap();
        victim.relationship = crate::neighbor::Relationship::Child;
        victim.rx_on_when_idle = false;
        assert!(parent.indirect.enqueue(ORIGIN, &[0xAA, 0xBB]));

        assert_eq!(
            block_on(parent.send_rejoin_response(
                ORIGIN,
                ORIGIN_IEEE,
                ShortAddress(0xFFFF),
                0x02,
                false,
                false,
            )),
            Ok(RejoinResponseDelivery::Direct)
        );
        assert_eq!(
            parent.indirect.peek(ORIGIN).unwrap().as_slice(),
            &[0xAA, 0xBB]
        );

        let outcome = block_on(parent.service_child_data_request(MacAddress::Short(PAN, ORIGIN)))
            .expect("the victim's queued frame is still deliverable");
        assert_eq!(
            outcome,
            crate::ChildPollOutcome::Delivered {
                child: ORIGIN,
                more_pending: false,
            }
        );
        assert_eq!(parent.mac.tx_history()[1].payload.as_slice(), &[0xAA, 0xBB]);
    }

    #[test]
    #[cfg(feature = "router")]
    fn indirect_delivery_advertises_each_additional_queued_frame() {
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.update_neighbor_address(ORIGIN, ORIGIN_IEEE);
        let child = parent.neighbors.find_by_short_mut(ORIGIN).unwrap();
        child.relationship = crate::neighbor::Relationship::Child;
        child.rx_on_when_idle = false;
        parent.enqueue_indirect_for_child(ORIGIN, &[0xAA]).unwrap();
        parent.enqueue_indirect_for_child(ORIGIN, &[0xBB]).unwrap();

        assert_eq!(
            block_on(parent.service_child_data_request(MacAddress::Short(PAN, ORIGIN))).unwrap(),
            crate::ChildPollOutcome::Delivered {
                child: ORIGIN,
                more_pending: true,
            }
        );
        assert_eq!(parent.mac.tx_history()[0].payload.as_slice(), &[0xAA]);
        assert!(parent.mac.tx_history()[0].frame_pending);

        assert_eq!(
            block_on(parent.service_child_data_request(MacAddress::Short(PAN, ORIGIN))).unwrap(),
            crate::ChildPollOutcome::Delivered {
                child: ORIGIN,
                more_pending: false,
            }
        );
        assert_eq!(parent.mac.tx_history()[1].payload.as_slice(), &[0xBB]);
        assert!(!parent.mac.tx_history()[1].frame_pending);
    }

    #[test]
    #[cfg(feature = "router")]
    fn secured_rejoin_response_matches_the_requests_security_state() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: true,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        let assigned = parent
            .handle_child_rejoin(ORIGIN, ORIGIN_IEEE, capability.to_byte(), true)
            .unwrap();

        assert_eq!(
            block_on(parent.send_rejoin_response(ORIGIN, ORIGIN_IEEE, assigned, 0, true, true,)),
            Ok(RejoinResponseDelivery::Direct)
        );
        let recorded = recorded_frame(&parent, 0);
        let (header, _, payload) = decrypt_recorded(&recorded);
        assert!(header.frame_control.security);
        assert_eq!(header.dst_addr, ORIGIN);
        assert_eq!(header.radius, 1);
        assert_eq!(
            payload.as_slice(),
            &[
                NwkCommandId::RejoinResponse as u8,
                assigned.0 as u8,
                (assigned.0 >> 8) as u8,
                0,
            ]
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn matching_nwk_security_proof_authorizes_a_provisional_child() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: true,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        parent.nib.permit_joining = true;
        let child = parent
            .handle_child_association(ORIGIN_IEEE, capability.to_byte())
            .unwrap();
        assert!(!parent.child_is_authorized(&ORIGIN_IEEE));

        let mut unsecured = [0u8; 128];
        let unsecured_len = encode(
            &frame(NwkFrameType::Data, child, OUR_ADDR),
            &[0x55],
            &mut unsecured,
        );
        assert!(
            block_on(parent.process_incoming_nwk_frame_from(
                &unsecured[..unsecured_len],
                42,
                Some(child),
            ))
            .is_none()
        );
        assert!(!parent.child_is_authorized(&ORIGIN_IEEE));

        let mut sender = secured_node(DeviceType::EndDevice, child, ORIGIN_IEEE);
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Data as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: true,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: true,
            },
            dst_addr: OUR_ADDR,
            src_addr: child,
            radius: 1,
            seq_number: 1,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };
        let mut frame = [0u8; 128];
        let len = sender
            .build_nwk_frame(&header, &[0xAA], &mut frame)
            .unwrap();

        assert!(matches!(
            block_on(parent.process_incoming_nwk_frame_from(&frame[..len], 42, Some(child),)),
            Some(NwkIndication::Owned(_))
        ));
        assert!(parent.child_is_authorized(&ORIGIN_IEEE));
    }

    #[test]
    #[cfg(feature = "router")]
    fn unauthenticated_child_timeout_clears_routes_and_pending_data() {
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        };
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        parent.nib.permit_joining = true;
        let child = parent
            .handle_child_association(ORIGIN_IEEE, capability.to_byte())
            .unwrap();
        parent.routing.update_route(child, child, 1).unwrap();
        parent.enqueue_indirect_for_child(child, &[0xAA]).unwrap();

        parent.tick_router_maintenance(10);

        assert_eq!(parent.known_child_by_ieee(&ORIGIN_IEEE), None);
        assert!(!parent.indirect_queue().has_pending(child));
        assert!(parent.routing.get_entry(child).is_none());
        assert_eq!(
            parent.mac.indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, child), false))
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn stochastic_child_addresses_are_always_allocated_unicast_values() {
        let mut parent = node(DeviceType::Router, OUR_ADDR);
        for seed in 0..=u16::MAX {
            let ieee = [seed as u8, (seed >> 8) as u8, 1, 2, 3, 4, 5, 6];
            let address = parent.assign_child_address(&ieee);
            assert!(
                (0x0001..=0xFFF7).contains(&address.0),
                "assigned reserved address 0x{:04X}",
                address.0
            );
        }
    }

    // ── R22 End Device Timeout Request server ────────────────

    /// The exact bytes a child puts on air for an End Device Timeout Request
    /// (0x0B), produced by the real transmit path of a secured node standing
    /// in for the child rather than a hand-rolled encoder.
    #[cfg(feature = "router")]
    fn ed_timeout_request_on_air(
        dst: ShortAddress,
        secured: bool,
        header_ieee: IeeeAddress,
        security_ieee: IeeeAddress,
        requested_timeout: u8,
        ed_config: u8,
    ) -> heapless::Vec<u8, 128> {
        let mut child = secured_node(DeviceType::EndDevice, ORIGIN, security_ieee);
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Command as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: secured,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: true,
                end_device_initiator: false,
            },
            dst_addr: dst,
            src_addr: ORIGIN,
            radius: 1,
            seq_number: 0x33,
            dst_ieee: None,
            src_ieee: Some(header_ieee),
            multicast_control: None,
            source_route: None,
        };
        let mut frame = [0u8; 128];
        let len = child
            .build_nwk_frame(
                &header,
                &[
                    NwkCommandId::EdTimeoutRequest as u8,
                    requested_timeout,
                    ed_config,
                ],
                &mut frame,
            )
            .expect("the request frame builds");
        heapless::Vec::from_slice(&frame[..len]).unwrap()
    }

    #[cfg(feature = "router")]
    fn sleepy_end_device_capability() -> CapabilityInfo {
        CapabilityInfo {
            device_type_ffd: false,
            mains_powered: false,
            rx_on_when_idle: false,
            security_capable: true,
            allocate_address: true,
        }
    }

    /// A router parent that already holds `ORIGIN_IEEE` at `ORIGIN` as a
    /// secured, authenticated sleepy end-device child.
    #[cfg(feature = "router")]
    fn parent_with_sleepy_child() -> NwkLayer<MockMac> {
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        parent.nib.permit_joining = true;
        let assigned = parent
            .handle_child_rejoin(
                ORIGIN,
                ORIGIN_IEEE,
                sleepy_end_device_capability().to_byte(),
                true,
            )
            .expect("the child is admitted");
        assert_eq!(assigned, ORIGIN, "the requested address is free");
        parent
    }

    /// Decrypt a recorded secured NWK command back to its plaintext body.
    #[cfg(feature = "router")]
    fn decrypted_command_body(frame: &[u8]) -> heapless::Vec<u8, MAX_NWK_FRAME> {
        decrypt_recorded(frame).2
    }

    #[cfg(feature = "router")]
    fn assert_ed_request_refused(parent: &mut NwkLayer<MockMac>, frame: &[u8]) {
        assert!(
            block_on(parent.process_incoming_nwk_frame_from(frame, 42, Some(ORIGIN))).is_none()
        );
        assert_eq!(parent.take_command_outcome(), None);
        assert!(
            parent.mac.tx_history().is_empty(),
            "a refused request must never draw a response"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn ed_timeout_request_from_an_authenticated_child_is_accepted_and_answered_indirectly() {
        let mut parent = parent_with_sleepy_child();
        let request = ed_timeout_request_on_air(OUR_ADDR, true, ORIGIN_IEEE, ORIGIN_IEEE, 14, 0);

        assert!(
            block_on(parent.process_incoming_nwk_frame_from(&request, 42, Some(ORIGIN))).is_none()
        );
        assert_eq!(
            parent.take_command_outcome(),
            Some(NwkCommandOutcome::EndDeviceTimeoutRequest {
                src: ORIGIN,
                ieee: ORIGIN_IEEE,
                requested_timeout: 14,
            })
        );

        block_on(parent.respond_to_end_device_timeout_request(ORIGIN, ORIGIN_IEEE, 14))
            .expect("the response is queued");

        // The accepted timeout is recorded and the deadline re-armed to it.
        let entry = parent.neighbor_table().find_by_short(ORIGIN).unwrap();
        assert_eq!(entry.end_device_timeout, 14);
        assert_eq!(
            entry.keepalive_remaining_secs,
            crate::frames::ed_timeout_enum_to_seconds(14).unwrap()
        );

        // A sleepy child receives the response indirectly on its next poll,
        // through the parent queue with Frame Pending armed.
        assert!(parent.indirect_queue().has_pending(ORIGIN));
        assert_eq!(
            parent.mac.indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, ORIGIN), true))
        );
        parent.mac.clear_tx_history();
        let outcome = block_on(parent.service_child_data_request(MacAddress::Short(PAN, ORIGIN)))
            .expect("the poll delivers the response");
        assert!(matches!(outcome, crate::ChildPollOutcome::Delivered { .. }));

        let record = &parent.mac.tx_history()[0];
        assert!(record.indirect, "a sleepy child response is indirect");
        assert_eq!(
            decrypted_command_body(record.payload.as_slice()).as_slice(),
            &[
                NwkCommandId::EdTimeoutResponse as u8,
                crate::frames::ED_TIMEOUT_STATUS_SUCCESS,
                crate::frames::PARENT_INFO_MASK,
            ]
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn ed_timeout_request_rejects_reserved_byte_and_undefined_enum() {
        // A fresh parent per frame keeps each request at frame counter zero
        // without tripping the replay filter.
        let mut reserved = parent_with_sleepy_child();
        assert_ed_request_refused(
            &mut reserved,
            &ed_timeout_request_on_air(OUR_ADDR, true, ORIGIN_IEEE, ORIGIN_IEEE, 8, 0x01),
        );

        let mut undefined = parent_with_sleepy_child();
        assert_ed_request_refused(
            &mut undefined,
            &ed_timeout_request_on_air(OUR_ADDR, true, ORIGIN_IEEE, ORIGIN_IEEE, 15, 0x00),
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn ed_timeout_request_rejects_a_spoofed_source_identity() {
        // The auxiliary-header (authenticated) IEEE is the child's, but the
        // NWK header claims a different one.
        let mut parent = parent_with_sleepy_child();
        assert_ed_request_refused(
            &mut parent,
            &ed_timeout_request_on_air(OUR_ADDR, true, DEST_IEEE, ORIGIN_IEEE, 8, 0x00),
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn ed_timeout_request_from_an_unknown_device_is_ignored() {
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        assert_ed_request_refused(
            &mut parent,
            &ed_timeout_request_on_air(OUR_ADDR, true, ORIGIN_IEEE, ORIGIN_IEEE, 8, 0x00),
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn ed_timeout_request_from_a_non_child_network_member_is_ignored() {
        // ORIGIN is a known, authenticated network member — but a sibling, not
        // a child of this parent. It must not be able to rewrite child state.
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        parent.update_neighbor_address(ORIGIN, ORIGIN_IEEE);
        assert_ed_request_refused(
            &mut parent,
            &ed_timeout_request_on_air(OUR_ADDR, true, ORIGIN_IEEE, ORIGIN_IEEE, 8, 0x00),
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_unsupported_enum_is_refused_and_an_rx_on_child_is_answered_directly() {
        // rx-on end-device child at ORIGIN.
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        parent.nib.permit_joining = true;
        let capability = CapabilityInfo {
            device_type_ffd: false,
            mains_powered: true,
            rx_on_when_idle: true,
            security_capable: true,
            allocate_address: true,
        };
        let assigned = parent
            .handle_child_rejoin(ORIGIN, ORIGIN_IEEE, capability.to_byte(), true)
            .expect("the child is admitted");
        assert_eq!(assigned, ORIGIN);

        // A value outside 0..=14 exercises the deterministic INCORRECT_VALUE
        // fallback to the default enumeration.
        block_on(parent.respond_to_end_device_timeout_request(ORIGIN, ORIGIN_IEEE, 15))
            .expect("the refusal is transmitted");

        let entry = parent.neighbor_table().find_by_short(ORIGIN).unwrap();
        assert_eq!(
            entry.end_device_timeout,
            crate::frames::ED_TIMEOUT_ENUM_DEFAULT
        );

        // rx-on child → a direct response, nothing queued indirectly.
        assert!(!parent.indirect_queue().has_pending(ORIGIN));
        let record = &parent.mac.tx_history()[0];
        assert!(!record.indirect, "an rx-on child response is direct");
        assert_eq!(
            decrypted_command_body(record.payload.as_slice()).as_slice(),
            &[
                NwkCommandId::EdTimeoutResponse as u8,
                crate::frames::ED_TIMEOUT_STATUS_INCORRECT_VALUE,
                crate::frames::PARENT_INFO_MASK,
            ]
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn respond_to_ed_timeout_request_rejects_an_evicted_child() {
        // If the child is gone by the async response step, no frame is sent.
        let mut parent = parent_with_sleepy_child();
        parent.remove_neighbor(ORIGIN);
        assert_eq!(
            block_on(parent.respond_to_end_device_timeout_request(ORIGIN, ORIGIN_IEEE, 8)),
            Err(NwkStatus::UnknownDevice)
        );
        assert!(parent.mac.tx_history().is_empty());
    }

    #[test]
    #[cfg(feature = "router")]
    fn a_mac_poll_refreshes_an_end_device_child_deadline() {
        let mut parent = parent_with_sleepy_child();
        let window =
            crate::frames::ed_timeout_enum_to_seconds(crate::frames::ED_TIMEOUT_ENUM_DEFAULT)
                .unwrap() as u16;
        // Age almost to the deadline without evicting.
        assert!(parent.age_end_device_children(window - 2).is_empty());
        // A MAC Data Poll is an advertised keepalive and re-arms the deadline.
        let _ = block_on(parent.service_child_data_request(MacAddress::Short(PAN, ORIGIN)));
        assert_eq!(
            parent
                .neighbor_table()
                .find_by_short(ORIGIN)
                .unwrap()
                .keepalive_remaining_secs,
            crate::frames::ed_timeout_enum_to_seconds(crate::frames::ED_TIMEOUT_ENUM_DEFAULT)
                .unwrap(),
            "a MAC Data Poll re-arms the deadline"
        );
        // Without the poll this much aging would have evicted the child; it
        // survives because the poll refreshed it.
        assert!(parent.age_end_device_children(window - 1).is_empty());
        assert!(parent.neighbor_table().find_by_short(ORIGIN).is_some());
    }

    #[test]
    #[cfg(feature = "router")]
    fn end_device_children_age_out_and_are_cleaned_up() {
        let mut parent = parent_with_sleepy_child();
        // Queue an indirect frame so eviction has coupled state to clean.
        parent.enqueue_indirect_for_child(ORIGIN, &[0xAB]).unwrap();
        assert!(parent.indirect_queue().has_pending(ORIGIN));

        // Age just short of the default deadline: still present.
        let window =
            crate::frames::ed_timeout_enum_to_seconds(crate::frames::ED_TIMEOUT_ENUM_DEFAULT)
                .unwrap() as u16;
        // (The default enumeration's window fits in u16.)
        let evicted = parent.age_end_device_children(window - 1);
        assert!(evicted.is_empty());
        assert!(parent.neighbor_table().find_by_short(ORIGIN).is_some());

        // One more second past the deadline evicts it and cleans coupled state.
        let evicted = parent.age_end_device_children(1);
        assert_eq!(evicted.as_slice(), &[ORIGIN]);
        assert!(parent.neighbor_table().find_by_short(ORIGIN).is_none());
        assert!(!parent.indirect_queue().has_pending(ORIGIN));
        assert_eq!(
            parent.mac.indirect_pending_history().last(),
            Some(&(MacAddress::Short(PAN, ORIGIN), false)),
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn an_end_device_resume_never_ages_children() {
        // Aging is a parent-only concern; an end device returns nothing.
        let mut end_device = secured_node(DeviceType::EndDevice, OUR_ADDR, RELAY_IEEE);
        assert!(end_device.age_end_device_children(u16::MAX).is_empty());
    }

    // ── R22 child restore + Parent Announce ──────────────────

    const RESTORED_CHILD: ShortAddress = FAR;
    const RESTORED_CHILD_IEEE: IeeeAddress = [0xC7; 8];

    #[test]
    #[cfg(feature = "router")]
    fn restore_child_reinstalls_authenticated_children_unconfirmed() {
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        // An end-device child and a router child.
        assert!(parent.restore_child(ORIGIN_IEEE, ORIGIN, false, true, false, 8));
        assert!(parent.restore_child(RESTORED_CHILD_IEEE, RESTORED_CHILD, true, true, true, 8));

        let end_device = parent.neighbor_table().find_by_short(ORIGIN).unwrap();
        assert_eq!(
            end_device.relationship,
            crate::neighbor::Relationship::Child
        );
        assert!(
            !end_device.keepalive_confirmed,
            "a restored child has not been heard from yet"
        );
        assert_eq!(
            end_device.keepalive_remaining_secs,
            crate::frames::ed_timeout_enum_to_seconds(8).unwrap(),
            "an end-device child's deadline is re-armed to a full window"
        );
        let router_child = parent
            .neighbor_table()
            .find_by_short(RESTORED_CHILD)
            .unwrap();
        assert_eq!(
            router_child.keepalive_remaining_secs, 0,
            "a router child does not use the End Device Timeout deadline"
        );

        let ieees: heapless::Vec<IeeeAddress, 8> = parent.authenticated_child_ieees();
        assert_eq!(ieees.len(), 2);
        assert!(ieees.contains(&ORIGIN_IEEE));
        assert!(ieees.contains(&RESTORED_CHILD_IEEE));

        // An end device never restores children or announces them.
        let mut end = secured_node(DeviceType::EndDevice, OUR_ADDR, RELAY_IEEE);
        assert!(!end.restore_child(ORIGIN_IEEE, ORIGIN, false, true, false, 8));
        assert!(end.authenticated_child_ieees::<8>().is_empty());
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_annce_keeps_confirmed_children_and_yields_unconfirmed_ones() {
        let mut parent = parent_with_sleepy_child(); // ORIGIN is confirmed (live rejoin)
        assert!(parent.restore_child(RESTORED_CHILD_IEEE, RESTORED_CHILD, false, true, false, 14));

        let outcome = parent.apply_parent_annce(&[ORIGIN_IEEE, RESTORED_CHILD_IEEE]);
        assert_eq!(
            outcome.kept.as_slice(),
            &[ORIGIN_IEEE],
            "a child we actively parent is defended and reported"
        );
        assert_eq!(
            outcome.dropped.as_slice(),
            &[RESTORED_CHILD],
            "an unconfirmed restored child yields to the announcer"
        );
        assert!(parent.neighbor_table().find_by_short(ORIGIN).is_some());
        assert!(
            parent
                .neighbor_table()
                .find_by_short(RESTORED_CHILD)
                .is_none()
        );

        // An unrelated IEEE that we do not parent is neither kept nor dropped.
        let outcome = parent.apply_parent_annce(&[[0x99; 8]]);
        assert!(outcome.kept.is_empty() && outcome.dropped.is_empty());
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_annce_confirms_a_child_after_a_keepalive() {
        // A restored (unconfirmed) child that then polls becomes confirmed and
        // is thereafter defended rather than yielded.
        let mut parent = secured_node(DeviceType::Router, OUR_ADDR, RELAY_IEEE);
        assert!(parent.restore_child(ORIGIN_IEEE, ORIGIN, false, true, false, 8));
        let _ = block_on(parent.service_child_data_request(MacAddress::Short(PAN, ORIGIN)));

        let outcome = parent.apply_parent_annce(&[ORIGIN_IEEE]);
        assert_eq!(outcome.kept.as_slice(), &[ORIGIN_IEEE]);
        assert!(outcome.dropped.is_empty());
        assert!(parent.neighbor_table().find_by_short(ORIGIN).is_some());
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_annce_rsp_relinquishes_moved_children() {
        let mut parent = parent_with_sleepy_child();
        let dropped = parent.remove_children_by_ieee(&[ORIGIN_IEEE]);
        assert_eq!(dropped.as_slice(), &[ORIGIN]);
        assert!(parent.neighbor_table().find_by_short(ORIGIN).is_none());
        // An IEEE we do not parent is ignored.
        assert!(parent.remove_children_by_ieee(&[[0x99; 8]]).is_empty());
    }
}
