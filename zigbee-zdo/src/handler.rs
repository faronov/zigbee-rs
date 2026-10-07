//! ZDP request / response dispatcher.
//!
//! Routes incoming APS frames on endpoint 0 to the appropriate ZDP handler,
//! builds the response, and sends it back through the APS layer.

use zigbee_aps::apsde::ApsdeDataIndication;
use zigbee_aps::binding::{BindingDst, BindingDstMode, BindingEntry};
use zigbee_aps::{ApsAddress, ApsAddressMode};
use zigbee_mac::MacDriver;
#[cfg(any(not(feature = "end-device"), feature = "router", test))]
use zigbee_nwk::nlme::nwk_update_id_is_newer;
use zigbee_types::ShortAddress;

use crate::binding_mgmt::{BindReq, BindTarget};
use crate::discovery::*;
use crate::network_mgmt::*;
use crate::{ZDO_ENDPOINT, ZdoError, ZdoLayer, ZdpStatus};

/// Bit 15 of a ZDP cluster identifier: set on responses, clear on requests.
///
/// A ZDP response cluster is always `request | ZDP_RESPONSE_BIT` (R22 2.4.4).
const ZDP_RESPONSE_BIT: u16 = 0x8000;

/// An already-applied Bind/Unbind result whose transmission is owned by the
/// caller. Persist the binding table before releasing this response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreparedBindingResponse {
    source: ShortAddress,
    cluster: u16,
    payload: [u8; 2],
}

/// R22 Table 2-145 `NOT_AUTHORIZED`.
///
/// Trust Center policy procedures return APS-internal status values to the
/// ZDO implementation. Those values are not valid ZDP status encodings, so a
/// policy denial is exposed on the wire as the standard ZDP security denial.
#[cfg(any(not(feature = "end-device"), feature = "router"))]
const ZDP_STATUS_NOT_AUTHORIZED: u8 = 0x8D;

/// `Status | TotalEntries | StartIndex | ListCount` prefix of the Mgmt
/// LQI/Rtg/Bind responses.
const MGMT_LIST_HEADER_LEN: usize = 4;

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
pub(crate) const fn is_unicast_short(addr: ShortAddress) -> bool {
    addr.0 < 0xFFF8
}

/// Outcome of validating an incoming `nwkUpdateId` against the local one.
#[cfg(any(not(feature = "end-device"), feature = "router", test))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum UpdateIdAdoption {
    /// Adopt the incoming update state and apply the requested change.
    Adopt,
    /// Same update state and the requested change is already in effect: a
    /// retransmission. Answer SUCCESS but change nothing.
    AlreadyApplied,
    /// Stale, ambiguous, or an equal update ID demanding a *different*
    /// configuration. Answer with the invalid-request status and change
    /// nothing.
    Reject,
}

/// R22 §3.4.12 `Mgmt_NWK_Update_req` update-state validation.
///
/// `local` is the locally held `nwkUpdateId` from
/// [`Nib::nwk_update_id`](zigbee_nwk::nib::Nib::nwk_update_id) — `None` when
/// this device holds no authoritative update state. `already_matches` says
/// whether the configuration the request asks for (channel, or NWK manager
/// address) is the one already in effect.
///
/// The rules, in order:
///
/// * unknown local state — nothing to order the request against and nothing to
///   protect, so the incoming ID is adopted;
/// * strictly newer (wrap-aware, [`nwk_update_id_is_newer`]) — adopted;
/// * equal and the requested configuration already matches — idempotent
///   retransmission, accepted without mutating anything;
/// * equal but demanding a *different* configuration — two different network
///   states claim the same update ID; the request is refused rather than
///   silently splitting the network;
/// * older or exactly half a window away (unorderable) — refused.
///
/// This never mutates anything, so the caller can validate before touching the
/// NIB or PIB.
#[cfg(any(not(feature = "end-device"), feature = "router", test))]
pub(crate) const fn nwk_update_id_adoption(
    local: Option<u8>,
    incoming: u8,
    already_matches: bool,
) -> UpdateIdAdoption {
    let Some(local) = local else {
        return UpdateIdAdoption::Adopt;
    };
    if nwk_update_id_is_newer(incoming, local) {
        return UpdateIdAdoption::Adopt;
    }
    if incoming == local && already_matches {
        return UpdateIdAdoption::AlreadyApplied;
    }
    UpdateIdAdoption::Reject
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
    /// Send an accepted local Mgmt_Leave response after the caller has made
    /// the corresponding leave or rejoin intent durable.
    pub async fn send_deferred_mgmt_leave_response(
        &mut self,
        destination: ShortAddress,
        transaction_sequence: u8,
    ) -> Result<(), ZdoError> {
        let payload = [transaction_sequence, ZdpStatus::Success as u8];
        self.diagnostics.response_attempts = self.diagnostics.response_attempts.wrapping_add(1);
        self.diagnostics.last_response_cluster = crate::MGMT_LEAVE_RSP;
        let result = self
            .send_zdp_unicast(destination, crate::MGMT_LEAVE_RSP, &payload)
            .await;
        if result.is_ok() {
            self.diagnostics.response_successes =
                self.diagnostics.response_successes.wrapping_add(1);
        } else {
            self.diagnostics.response_failures = self.diagnostics.response_failures.wrapping_add(1);
        }
        result
    }

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
        #[cfg(feature = "router")]
        if cluster == crate::PARENT_ANNCE {
            let exact_destination = matches!(
                (ind.dst_addr_mode, ind.dst_address),
                (
                    ApsAddressMode::Short,
                    ApsAddress::Short(ShortAddress(crate::BROADCAST_ROUTERS))
                )
            );
            let announcer = match ind.src_address {
                ApsAddress::Short(source) if is_unicast_short(source) => source,
                _ => return Ok(()),
            };
            if !exact_destination
                || (self.nwk().nib().security_enabled && !ind.security_status)
                || self
                    .nwk()
                    .neighbor_table()
                    .find_by_short(announcer)
                    .is_some_and(|entry| {
                        entry.device_type == zigbee_nwk::neighbor::NeighborDeviceType::EndDevice
                    })
            {
                log_zdp_exception(cluster, "non-normative or unauthenticated Parent_annce");
                return Ok(());
            }
            self.process_parent_annce(announcer, tsn, payload).await?;
            return Ok(());
        }
        #[cfg(feature = "router")]
        if cluster == crate::PARENT_ANNCE_RSP {
            let responder = match ind.src_address {
                ApsAddress::Short(source) if is_unicast_short(source) => source,
                _ => return Ok(()),
            };
            if !self.indication_is_unicast(ind)
                || (self.nwk().nib().security_enabled && !ind.security_status)
                || self
                    .nwk()
                    .neighbor_table()
                    .find_by_short(responder)
                    .is_some_and(|entry| {
                        entry.device_type == zigbee_nwk::neighbor::NeighborDeviceType::EndDevice
                    })
            {
                log_zdp_exception(
                    cluster,
                    "non-unicast, unauthenticated or end-device Parent_annce_rsp",
                );
                return Ok(());
            }
            self.process_parent_annce_rsp(tsn, payload);
            return Ok(());
        }

        // --- Check if this is a response to a pending client request ---
        let response_source = match ind.src_address {
            ApsAddress::Short(source) => Some(source),
            _ => None,
        };
        if self.deliver_response_from(response_source, cluster, tsn, payload) {
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
            crate::NWK_ADDR_REQ | crate::IEEE_ADDR_REQ => {
                let capacity = self.zdp_response_capacity();
                let result = self.handle_addr_req(
                    cluster == crate::IEEE_ADDR_REQ,
                    unicast,
                    payload,
                    &mut rsp_buf[1..1 + capacity],
                );
                // Only the device that owns (or parents) the address of
                // interest answers a broadcast (R22 2.4.3.1.1/2).
                if !unicast && result.is_ok() && rsp_buf[1] == ZdpStatus::DeviceNotFound as u8 {
                    log_zdp_exception(cluster, "broadcast address miss — silent");
                    return Ok(());
                }
                (cluster | ZDP_RESPONSE_BIT, result)
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
            #[cfg(any(not(feature = "end-device"), feature = "router"))]
            crate::MGMT_LQI_REQ => {
                let capacity = self.zdp_response_capacity();
                (
                    crate::MGMT_LQI_RSP,
                    self.handle_mgmt_lqi_req(payload, &mut rsp_buf[1..1 + capacity]),
                )
            }
            #[cfg(any(not(feature = "end-device"), feature = "router"))]
            crate::MGMT_RTG_REQ => {
                let capacity = self.zdp_response_capacity();
                (
                    crate::MGMT_RTG_RSP,
                    self.handle_mgmt_rtg_req(payload, &mut rsp_buf[1..1 + capacity]),
                )
            }
            crate::MGMT_BIND_REQ => {
                let capacity = self.zdp_response_capacity();
                (
                    crate::MGMT_BIND_RSP,
                    self.handle_mgmt_bind_req(payload, &mut rsp_buf[1..1 + capacity]),
                )
            }
            crate::MGMT_LEAVE_REQ => (
                crate::MGMT_LEAVE_RSP,
                self.handle_mgmt_leave_req(src_short, payload, &mut rsp_buf[1..]),
            ),
            #[cfg(any(not(feature = "end-device"), feature = "router"))]
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
            #[cfg(any(not(feature = "end-device"), feature = "router"))]
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

        self.send_response(src_short, rsp_cluster, &rsp_buf[..rsp_len])
            .await
    }

    /// Apply a Bind/Unbind request without sending its response. Parsing,
    /// mutation and status selection are shared with the ordinary dispatcher.
    /// A malformed broadcast is dropped, as in `handle_indication`.
    pub fn prepare_binding_response(
        &mut self,
        ind: &ApsdeDataIndication<'_>,
    ) -> Result<Option<PreparedBindingResponse>, ZdoError> {
        if ind.dst_endpoint != ZDO_ENDPOINT {
            return Err(ZdoError::InvalidData);
        }
        let (&tsn, payload) = ind.payload.split_first().ok_or(ZdoError::InvalidLength)?;
        let mut response = [tsn, 0];
        let (cluster, result) = match ind.cluster_id {
            crate::BIND_REQ => (
                crate::BIND_RSP,
                self.handle_bind_req(payload, &mut response[1..]),
            ),
            crate::UNBIND_REQ => (
                crate::UNBIND_RSP,
                self.handle_unbind_req(payload, &mut response[1..]),
            ),
            _ => return Err(ZdoError::InvalidData),
        };
        self.diagnostics.indications = self.diagnostics.indications.wrapping_add(1);
        self.diagnostics.last_cluster = ind.cluster_id;
        match result {
            Ok(_) => {}
            Err(ZdoError::InvalidLength | ZdoError::InvalidData)
                if !self.indication_is_unicast(ind) =>
            {
                log_zdp_exception(ind.cluster_id, "malformed broadcast — dropped");
                return Ok(None);
            }
            Err(error) => return Err(error),
        }
        let source = match ind.src_address {
            ApsAddress::Short(address) => address,
            _ => ShortAddress::COORDINATOR,
        };
        Ok(Some(PreparedBindingResponse {
            source,
            cluster,
            payload: response,
        }))
    }

    /// Send an already-applied binding response without applying the request
    /// again. On failure the caller retains the response and may retry it.
    pub async fn send_prepared_binding_response(
        &mut self,
        response: &PreparedBindingResponse,
    ) -> Result<(), ZdoError> {
        self.send_response(response.source, response.cluster, &response.payload)
            .await
    }

    async fn send_response(
        &mut self,
        src_short: ShortAddress,
        rsp_cluster: u16,
        payload: &[u8],
    ) -> Result<(), ZdoError> {
        log::info!(
            "[ZDO TX] rsp cluster=0x{:04X} to 0x{:04X} len={}",
            rsp_cluster,
            src_short.0,
            payload.len()
        );
        self.diagnostics.response_attempts = self.diagnostics.response_attempts.wrapping_add(1);
        self.diagnostics.last_response_cluster = rsp_cluster;
        let tx_result = self.send_zdp_unicast(src_short, rsp_cluster, payload).await;
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

    /// Bytes available for a ZDP response after the echoed TSN.
    ///
    /// R22 2.4.4.3.2-4: list responses carry "as many entries as fit" in one
    /// frame; the budget is [`crate::ZDP_MAX_PAYLOAD`] minus the source-route
    /// subframe a concentrator may add to the unicast response.
    fn zdp_response_capacity(&self) -> usize {
        let reserve = if self.nwk().is_concentrator() {
            crate::ZDP_SOURCE_ROUTE_RESERVE
        } else {
            0
        };
        crate::ZDP_MAX_PAYLOAD - reserve - 1
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

    /// NWK_addr_req / IEEE_addr_req (R22 2.4.3.1.1-2, 2.4.4.2.1-2).
    ///
    /// The node answers for itself and, on a router, for its end-device
    /// children. An unknown `RequestType` is answered with `INV_REQUESTTYPE`
    /// when the request was unicast or names an address this node answers
    /// for; an Extended request appends the associated-device list (children
    /// of this router) starting at `StartIndex`, as many as fit the frame.
    fn handle_addr_req(
        &self,
        by_nwk_addr: bool,
        unicast: bool,
        payload: &[u8],
        rsp: &mut [u8],
    ) -> Result<usize, ZdoError> {
        // IEEE_addr_req: NWKAddrOfInterest(2) | RequestType(1) | StartIndex(1)
        // NWK_addr_req:  IEEEAddr(8)          | RequestType(1) | StartIndex(1)
        let key_len = if by_nwk_addr { 2 } else { 8 };
        if payload.len() < key_len + 2 {
            return Err(ZdoError::InvalidLength);
        }
        if rsp.len() < NwkAddrRsp::MIN_SIZE + 2 {
            return Err(ZdoError::BufferTooSmall);
        }
        let request_type = payload[key_len];
        let start_index = payload[key_len + 1];
        let local_short = self.local_short_address();
        let local_ieee = self.local_ieee_addr();
        let (requested_ieee, requested_short) = if by_nwk_addr {
            (
                [0u8; 8],
                ShortAddress(u16::from_le_bytes([payload[0], payload[1]])),
            )
        } else {
            let mut ieee = [0u8; 8];
            ieee.copy_from_slice(&payload[..8]);
            (ieee, ShortAddress(0x0000))
        };
        let local_match = if by_nwk_addr {
            requested_short == local_short
        } else {
            requested_ieee == local_ieee
        };
        let matched = if local_match {
            Some((local_ieee, local_short))
        } else {
            self.end_device_child_address(by_nwk_addr, requested_ieee, requested_short)
        };

        let (status, ieee, short) = match matched {
            Some((ieee, short)) if request_type <= RequestType::Extended as u8 => {
                (ZdpStatus::Success, ieee, short)
            }
            Some((ieee, short)) => (ZdpStatus::InvRequestType, ieee, short),
            None if request_type > RequestType::Extended as u8 && unicast => {
                (ZdpStatus::InvRequestType, requested_ieee, requested_short)
            }
            None => (ZdpStatus::DeviceNotFound, requested_ieee, requested_short),
        };
        rsp[0] = status as u8;
        rsp[1..9].copy_from_slice(&ieee);
        rsp[9..11].copy_from_slice(&short.0.to_le_bytes());
        if status != ZdpStatus::Success || request_type != RequestType::Extended as u8 {
            return Ok(NwkAddrRsp::MIN_SIZE);
        }

        // Extended response. Only a router/coordinator answering for itself
        // has associated devices; an end device, or a parent answering for
        // a child, reports NumAssocDev = 0 without StartIndex or list.
        #[cfg(feature = "router")]
        if local_match {
            return Ok(self.write_associated_devices(start_index, rsp));
        }
        let _ = start_index;
        rsp[NwkAddrRsp::MIN_SIZE] = 0;
        Ok(NwkAddrRsp::MIN_SIZE + 1)
    }

    /// Append `NumAssocDev | StartIndex | NWKAddrAssocDevList` (children of
    /// this router from `start_index`, as many as fit `rsp`) to an Extended
    /// address response. NumAssocDev counts the entries in this frame's list
    /// (R22 2.4.3.1.1), not every child.
    #[cfg(feature = "router")]
    fn write_associated_devices(&self, start_index: u8, rsp: &mut [u8]) -> usize {
        let mut children = self
            .nwk()
            .neighbor_table()
            .iter()
            .filter(|entry| entry.relationship == zigbee_nwk::neighbor::Relationship::Child)
            .peekable();
        if children.peek().is_none() {
            rsp[NwkAddrRsp::MIN_SIZE] = 0;
            return NwkAddrRsp::MIN_SIZE + 1;
        }
        let mut off = NwkAddrRsp::MIN_SIZE + 2;
        let mut listed = 0u8;
        for child in children.skip(usize::from(start_index)) {
            if rsp.len() - off < 2 {
                break;
            }
            rsp[off..off + 2].copy_from_slice(&child.network_address.0.to_le_bytes());
            off += 2;
            listed += 1;
        }
        rsp[NwkAddrRsp::MIN_SIZE] = listed;
        rsp[NwkAddrRsp::MIN_SIZE + 1] = start_index;
        off
    }

    /// Address pair of an end-device child this router answers for.
    #[cfg(feature = "router")]
    fn end_device_child_address(
        &self,
        by_nwk_addr: bool,
        ieee: zigbee_types::IeeeAddress,
        short: ShortAddress,
    ) -> Option<(zigbee_types::IeeeAddress, ShortAddress)> {
        let table = self.nwk().neighbor_table();
        let entry = if by_nwk_addr {
            table.find_by_short(short)
        } else {
            table.find_by_ieee(&ieee)
        }?;
        (entry.relationship == zigbee_nwk::neighbor::Relationship::Child
            && entry.device_type == zigbee_nwk::neighbor::NeighborDeviceType::EndDevice)
            .then_some((entry.ieee_address, entry.network_address))
    }

    #[cfg(not(feature = "router"))]
    fn end_device_child_address(
        &self,
        _by_nwk_addr: bool,
        _ieee: zigbee_types::IeeeAddress,
        _short: ShortAddress,
    ) -> Option<(zigbee_types::IeeeAddress, ShortAddress)> {
        None
    }

    fn handle_node_desc_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = NodeDescReq::parse(payload)?;
        let rsp_data = if req.nwk_addr_of_interest == self.local_short_address() {
            NodeDescRsp {
                status: ZdpStatus::Success,
                nwk_addr_of_interest: self.local_short_address(),
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
        let rsp_data = if req.nwk_addr_of_interest == self.local_short_address() {
            PowerDescRsp {
                status: ZdpStatus::Success,
                nwk_addr_of_interest: self.local_short_address(),
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
        if req.nwk_addr_of_interest != self.local_short_address() {
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
                    nwk_addr_of_interest: self.local_short_address(),
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
                    nwk_addr_of_interest: self.local_short_address(),
                    simple_descriptor: None,
                };
                rsp_data.serialize(rsp)
            }
        }
    }

    fn handle_active_ep_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = NodeDescReq::parse(payload)?; // same layout
        if req.nwk_addr_of_interest != self.local_short_address() {
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
            nwk_addr_of_interest: self.local_short_address(),
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

    /// Validate a Bind_req/Unbind_req (R22 2.4.3.2.2-3).
    ///
    /// `Ok(Err(status))` is a well-formed request this node refuses with the
    /// given Bind_rsp/Unbind_rsp status:
    ///
    /// * a reserved `DstAddrMode` (only 0x01 group and 0x03 extended are
    ///   defined) — `NOT_SUPPORTED`;
    /// * a `SrcAddress` other than this node — this node only holds source
    ///   bindings for itself (no primary binding cache) — `NOT_SUPPORTED`;
    /// * a source or destination endpoint outside 0x01..=0xFE — `INVALID_EP`.
    fn parse_binding_request(
        &self,
        payload: &[u8],
    ) -> Result<Result<BindReq, ZdpStatus>, ZdoError> {
        if payload.len() >= 12 && !matches!(payload[11], 0x01 | 0x03) {
            return Ok(Err(ZdpStatus::NotSupported));
        }
        let req = BindReq::parse(payload)?;
        if req.src_addr != self.local_ieee_addr() {
            return Ok(Err(ZdpStatus::NotSupported));
        }
        let valid_ep = |endpoint: u8| (0x01..=0xFE).contains(&endpoint);
        let dst_ep_valid = match req.dst {
            BindTarget::Group(_) => true,
            BindTarget::Unicast { dst_endpoint, .. } => valid_ep(dst_endpoint),
        };
        if !valid_ep(req.src_endpoint) || !dst_ep_valid {
            return Ok(Err(ZdpStatus::InvalidEp));
        }
        Ok(Ok(req))
    }

    fn handle_bind_req(&mut self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let status = match self.parse_binding_request(payload)? {
            Ok(req) => match self
                .aps_mut()
                .binding_table_mut()
                .add(bind_req_to_entry(&req))
            {
                Ok(()) => ZdpStatus::Success,
                Err(_) => ZdpStatus::TableFull,
            },
            Err(status) => status,
        };
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        rsp[0] = status as u8;
        Ok(1)
    }

    fn handle_unbind_req(&mut self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let status = match self.parse_binding_request(payload)? {
            Ok(req) => {
                let dst = bind_target_to_dst(&req.dst);
                if self.aps_mut().binding_table_mut().remove(
                    &req.src_addr,
                    req.src_endpoint,
                    req.cluster_id,
                    &dst,
                ) {
                    ZdpStatus::Success
                } else {
                    ZdpStatus::NoEntry
                }
            }
            Err(status) => status,
        };
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        rsp[0] = status as u8;
        Ok(1)
    }

    // ── Network management ──────────────────────────────────────

    /// Write the common `Status | TotalEntries | StartIndex | ListCount`
    /// header of a Mgmt list response (R22 2.4.4.3.2-4).
    fn write_mgmt_list_header(rsp: &mut [u8], total: usize, start: u8, count: u8) {
        rsp[0] = ZdpStatus::Success as u8;
        rsp[1] = total.min(u8::MAX as usize) as u8;
        rsp[2] = start;
        rsp[3] = count;
    }

    /// Mgmt_Lqi_req: as many 22-byte neighbor records as fit `rsp`.
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn handle_mgmt_lqi_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = MgmtLqiReq::parse(payload)?;
        if rsp.len() < MGMT_LIST_HEADER_LEN {
            return Err(ZdoError::BufferTooSmall);
        }
        let neighbor_table = self.nwk().neighbor_table();
        let mut off = MGMT_LIST_HEADER_LEN;
        let mut count = 0u8;
        for entry in neighbor_table.iter().skip(req.start_index as usize) {
            if rsp.len() - off < NeighborTableRecord::WIRE_SIZE {
                break;
            }
            use zigbee_nwk::neighbor::{NeighborDeviceType, Relationship};
            let device_type = match entry.device_type {
                NeighborDeviceType::Coordinator => 0,
                NeighborDeviceType::Router => 1,
                NeighborDeviceType::EndDevice => 2,
                NeighborDeviceType::Unknown => 3,
            };
            let relationship = match entry.relationship {
                Relationship::Parent => 0,
                Relationship::Child => 1,
                Relationship::Sibling => 2,
                Relationship::PreviousChild => 4,
                Relationship::UnauthenticatedChild => 3,
            };
            off += NeighborTableRecord {
                extended_pan_id: entry.extended_pan_id,
                extended_addr: entry.ieee_address,
                network_addr: entry.network_address,
                device_type,
                rx_on_when_idle: u8::from(entry.rx_on_when_idle),
                relationship,
                permit_joining: u8::from(entry.permit_joining),
                depth: entry.depth,
                lqi: entry.lqi,
            }
            .serialize(&mut rsp[off..])?;
            count += 1;
        }
        Self::write_mgmt_list_header(rsp, neighbor_table.len(), req.start_index, count);
        Ok(off)
    }

    /// Mgmt_Rtg_req: as many 5-byte routing records as fit `rsp`.
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn handle_mgmt_rtg_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = MgmtRtgReq::parse(payload)?;
        if rsp.len() < MGMT_LIST_HEADER_LEN {
            return Err(ZdoError::BufferTooSmall);
        }
        let routing_table = self.nwk().routing_table();
        let mut off = MGMT_LIST_HEADER_LEN;
        let mut count = 0u8;
        for entry in routing_table.iter().skip(req.start_index as usize) {
            if rsp.len() - off < RoutingTableRecord::WIRE_SIZE {
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
            off += RoutingTableRecord {
                dst_addr: entry.destination,
                status,
                memory_constrained: false,
                many_to_one: entry.many_to_one,
                route_record_required: entry.route_record_required,
                next_hop: entry.next_hop,
            }
            .serialize(&mut rsp[off..])?;
            count += 1;
        }
        Self::write_mgmt_list_header(rsp, routing_table.len(), req.start_index, count);
        Ok(off)
    }

    /// Mgmt_Bind_req: as many 14/21-byte binding records as fit `rsp`.
    ///
    /// Records are emitted contiguously from `StartIndex`; the first record
    /// that does not fit ends the list so the requester can continue from
    /// `StartIndex + ListCount`.
    fn handle_mgmt_bind_req(&self, payload: &[u8], rsp: &mut [u8]) -> Result<usize, ZdoError> {
        let req = MgmtBindReq::parse(payload)?;
        if rsp.len() < MGMT_LIST_HEADER_LEN {
            return Err(ZdoError::BufferTooSmall);
        }
        let entries = self.aps().binding_table().entries();
        let mut off = MGMT_LIST_HEADER_LEN;
        let mut count = 0u8;
        for entry in entries.iter().skip(req.start_index as usize) {
            let record = aps_binding_to_record(entry);
            if rsp.len() - off < record.wire_size() {
                break;
            }
            off += record.serialize(&mut rsp[off..])?;
            count += 1;
        }
        Self::write_mgmt_list_header(rsp, entries.len(), req.start_index, count);
        Ok(off)
    }

    fn handle_mgmt_leave_req(
        &self,
        src: ShortAddress,
        payload: &[u8],
        rsp: &mut [u8],
    ) -> Result<usize, ZdoError> {
        // Note: actual leave is triggered by setting a flag that the runtime polls.
        // We can't call async nlme_leave from a sync context, and the leave needs
        // to happen AFTER we've attempted the response. Validate here with the
        // same classifier the runtime uses to decide whether the request was
        // accepted independently of response delivery.
        let targets_local_device = match self.classify_mgmt_leave_request(src, payload) {
            Ok(request) => request.is_some(),
            Err(error @ ZdoError::InvalidData) => {
                log::warn!(
                    "[ZDO] Ignoring Mgmt_Leave_req from unauthorized source 0x{:04X}",
                    src.0
                );
                return Err(error);
            }
            Err(error) => return Err(error),
        };
        log::info!("[ZDO] Mgmt_Leave_req received — leave will be executed after response attempt");
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        rsp[0] = if targets_local_device {
            ZdpStatus::Success
        } else {
            ZdpStatus::NotSupported
        } as u8;
        Ok(1)
    }

    /// Classify a Mgmt_Leave request using ZDO-owned source and target policy.
    ///
    /// `Ok(Some(request))` means the request is well formed, authorized, and
    /// targets this device, so the runtime must execute it even if transmitting
    /// `Mgmt_Leave_rsp` fails. `Ok(None)` is a valid authorized request for a
    /// different device; `Err` is malformed or unauthorized.
    pub fn classify_mgmt_leave_request(
        &self,
        src: ShortAddress,
        payload: &[u8],
    ) -> Result<Option<MgmtLeaveReq>, ZdoError> {
        let request = MgmtLeaveReq::parse(payload)?;
        if self.nwk().device_type() == zigbee_nwk::DeviceType::EndDevice
            && src != self.nwk().nib().parent_address
            && src != ShortAddress::COORDINATOR
        {
            return Err(ZdoError::InvalidData);
        }
        let local_ieee = self.nwk().nib().ieee_address;
        if request.device_address == [0; 8] || request.device_address == local_ieee {
            Ok(Some(request))
        } else {
            Ok(None)
        }
    }

    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    async fn handle_mgmt_permit_joining_req(
        &mut self,
        payload: &[u8],
        rsp: &mut [u8],
    ) -> Result<usize, ZdoError> {
        let req = MgmtPermitJoiningReq::parse(payload)?;
        if rsp.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }

        // R21+ deprecates TC_Significance=0 and requires every received
        // Mgmt_Permit_Joining_req to be treated as policy-significant. Use
        // the APS Trust Center identity rather than logical device type: a
        // router/coordinator that is not the configured Trust Center still
        // performs the local NLME action required by BDB 3.0.1, while child
        // admission remains subject to the configured Trust Center through
        // the existing Update-Device path.
        let is_trust_center = self.aps().is_trust_center();
        if is_trust_center {
            if !self.trust_center_allow_remote_policy_change {
                rsp[0] = ZDP_STATUS_NOT_AUTHORIZED;
                return Ok(1);
            }
            if self.trust_center_use_whitelist {
                rsp[0] = ZDP_STATUS_NOT_AUTHORIZED;
                return Ok(1);
            }
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
                if is_trust_center {
                    self.pending_trust_center_allow_joins = Some(req.permit_duration != 0);
                }
            }
            Err(e) => {
                log::warn!("[ZDO] Mgmt_Permit_Joining_req failed: {:?}", e,);
                rsp[0] = ZdpStatus::NotSupported as u8;
            }
        }
        Ok(1)
    }

    #[cfg(any(not(feature = "end-device"), feature = "router"))]
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
                let Some(ch) = channel else {
                    if rsp.is_empty() {
                        return Err(ZdoError::BufferTooSmall);
                    }
                    rsp[0] = ZdpStatus::InvRequestType as u8;
                    return Ok(1);
                };

                // R22 §3.4.12 — the update ID orders network update state.
                // Validate *before* touching the NIB/PIB so a stale or
                // ambiguous request can never move the radio off-channel.
                let current_channel = self.nwk().nib().logical_channel;
                match nwk_update_id_adoption(
                    self.nwk().nib().nwk_update_id(),
                    nwk_update_id,
                    ch == current_channel,
                ) {
                    UpdateIdAdoption::Reject => {
                        log::warn!(
                            "[ZDO] Mgmt_NWK_Update: rejected channel change to {ch} \
                             (update_id={nwk_update_id}, local {:?}, current channel {current_channel})",
                            self.nwk().nib().nwk_update_id(),
                        );
                        if rsp.is_empty() {
                            return Err(ZdoError::BufferTooSmall);
                        }
                        rsp[0] = ZdpStatus::InvRequestType as u8;
                        Ok(1)
                    }
                    UpdateIdAdoption::AlreadyApplied => {
                        // Same update state, already on the requested channel:
                        // a retransmission. Confirm without re-tuning.
                        log::debug!(
                            "[ZDO] Mgmt_NWK_Update: channel change to {ch} already applied \
                             (update_id={nwk_update_id})"
                        );
                        if rsp.is_empty() {
                            return Err(ZdoError::BufferTooSmall);
                        }
                        rsp[0] = ZdpStatus::Success as u8;
                        Ok(1)
                    }
                    UpdateIdAdoption::Adopt => {
                        log::info!(
                            "[ZDO] Mgmt_NWK_Update: channel change to {ch} (update_id={nwk_update_id})"
                        );
                        match self.nwk_mut().nlme_set_channel(ch).await {
                            Ok(()) => {
                                // Only a channel change that actually took
                                // effect may advance the update state.
                                self.nwk_mut().nib_mut().set_nwk_update_id(nwk_update_id);
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
                    }
                }
            }
            MgmtNwkUpdateReq::ManagerChange {
                nwk_update_id,
                nwk_manager_addr,
                ..
            } => {
                let current_manager = self.nwk().nib().nwk_manager_addr;
                match nwk_update_id_adoption(
                    self.nwk().nib().nwk_update_id(),
                    nwk_update_id,
                    nwk_manager_addr == current_manager,
                ) {
                    UpdateIdAdoption::Reject => {
                        log::warn!(
                            "[ZDO] Mgmt_NWK_Update: rejected manager change to 0x{:04X} \
                             (update_id={nwk_update_id}, local {:?}, current manager 0x{:04X})",
                            nwk_manager_addr.0,
                            self.nwk().nib().nwk_update_id(),
                            current_manager.0,
                        );
                        if rsp.is_empty() {
                            return Err(ZdoError::BufferTooSmall);
                        }
                        rsp[0] = ZdpStatus::InvRequestType as u8;
                        Ok(1)
                    }
                    UpdateIdAdoption::AlreadyApplied => {
                        log::debug!(
                            "[ZDO] Mgmt_NWK_Update: manager change to 0x{:04X} already applied \
                             (update_id={nwk_update_id})",
                            nwk_manager_addr.0,
                        );
                        if rsp.is_empty() {
                            return Err(ZdoError::BufferTooSmall);
                        }
                        rsp[0] = ZdpStatus::Success as u8;
                        Ok(1)
                    }
                    UpdateIdAdoption::Adopt => {
                        log::info!(
                            "[ZDO] Mgmt_NWK_Update: manager change to 0x{:04X} (update_id={nwk_update_id})",
                            nwk_manager_addr.0,
                        );
                        let nib = self.nwk_mut().nib_mut();
                        nib.nwk_manager_addr = nwk_manager_addr;
                        nib.set_nwk_update_id(nwk_update_id);
                        if rsp.is_empty() {
                            return Err(ZdoError::BufferTooSmall);
                        }
                        rsp[0] = ZdpStatus::Success as u8;
                        Ok(1)
                    }
                }
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

    #[cfg(feature = "router")]
    fn router_broadcast(cluster: u16, payload: &[u8]) -> ApsdeDataIndication<'_> {
        indication(
            cluster,
            ApsAddressMode::Short,
            ApsAddress::Short(ShortAddress(crate::BROADCAST_ROUTERS)),
            payload,
        )
    }

    /// Decode the ZDP cluster and payload of the last frame the MAC sent.
    fn last_zdp_tx(zdo: &ZdoLayer<MockMac>) -> Option<(u16, heapless::Vec<u8, 128>)> {
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

    #[test]
    fn prepared_binding_response_defers_transmit_and_preserves_unbind_status_on_retry() {
        for dst in [
            BindTarget::Group(0x1234),
            BindTarget::Unicast {
                dst_addr: [0x42; 8],
                dst_endpoint: 1,
            },
        ] {
            let mut zdo = test_zdo();
            let request = BindReq {
                src_addr: LOCAL_IEEE,
                src_endpoint: 1,
                cluster_id: 0x0006,
                dst,
            };
            let mut payload = [0u8; 22];
            payload[0] = 0x79;
            let len = 1 + request.serialize(&mut payload[1..]).unwrap();
            let prepared = zdo
                .prepare_binding_response(&unicast(crate::BIND_REQ, &payload[..len]))
                .unwrap()
                .unwrap();
            assert_eq!(zdo.aps().binding_table().len(), 1);
            assert_eq!(tx_count(&zdo), 0);
            block_on(zdo.send_prepared_binding_response(&prepared)).unwrap();
            let (cluster, response) = last_zdp_tx(&zdo).unwrap();
            assert_eq!(cluster, crate::BIND_RSP);
            assert_eq!(response.as_slice(), &[0x79, ZdpStatus::Success as u8]);

            let prepared = zdo
                .prepare_binding_response(&unicast(crate::UNBIND_REQ, &payload[..len]))
                .unwrap()
                .unwrap();
            assert!(zdo.aps().binding_table().is_empty());
            assert_eq!(tx_count(&zdo), 1);
            zdo.nwk_mut().mac_mut().set_tx_failures(100);
            assert!(matches!(
                block_on(zdo.send_prepared_binding_response(&prepared)),
                Err(ZdoError::ApsError(_))
            ));
            zdo.nwk_mut().mac_mut().set_tx_failures(0);
            block_on(zdo.send_prepared_binding_response(&prepared)).unwrap();
            let (cluster, response) = last_zdp_tx(&zdo).unwrap();
            assert_eq!(cluster, crate::UNBIND_RSP);
            assert_eq!(response.as_slice(), &[0x79, ZdpStatus::Success as u8]);
        }
    }

    #[test]
    fn prepared_binding_rejects_truncation_without_mutation_or_transmit() {
        let mut zdo = test_zdo();
        for cluster in [crate::BIND_REQ, crate::UNBIND_REQ] {
            assert_eq!(
                zdo.prepare_binding_response(&unicast(cluster, &[0x79, 0])),
                Err(ZdoError::InvalidLength)
            );
            assert_eq!(
                zdo.prepare_binding_response(&broadcast(cluster, &[0x79, 0])),
                Ok(None)
            );
        }
        assert!(zdo.aps().binding_table().is_empty());
        assert_eq!(tx_count(&zdo), 0);
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

        block_on(zdo.handle_indication(&router_broadcast(crate::PARENT_ANNCE, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("Parent_annce_rsp");
        assert_eq!(cluster, crate::PARENT_ANNCE_RSP);
        assert_eq!(body[0], 0xA5);
        assert_eq!(body[1], ZdpStatus::Success as u8);
        assert_eq!(body[2], 1);
        assert_eq!(&body[3..], &CHILD_IEEE);
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_annce_requires_the_secured_all_routers_broadcast() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        assert!(zdo.nwk_mut().restore_child(
            CHILD_IEEE,
            CHILD_SHORT,
            false,
            true,
            false,
            zigbee_nwk::frames::ED_TIMEOUT_ENUM_DEFAULT,
        ));
        zdo.nwk_mut().nib_mut().security_enabled = true;
        let mut payload = [0u8; 10];
        payload[0] = 0xA5;
        payload[1] = 1;
        payload[2..].copy_from_slice(&CHILD_IEEE);

        let mut unsecured = router_broadcast(crate::PARENT_ANNCE, &payload);
        unsecured.security_status = false;
        block_on(zdo.handle_indication(&unsecured)).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_some(),
            "an unsecured frame cannot evict a restored child"
        );

        block_on(zdo.handle_indication(&broadcast(crate::PARENT_ANNCE, &payload))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_some(),
            "0xFFFD is not the normative Parent_annce destination"
        );

        block_on(zdo.handle_indication(&unicast(crate::PARENT_ANNCE, &payload))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_some(),
            "a unicast Parent_annce must be dropped"
        );

        block_on(zdo.handle_indication(&router_broadcast(crate::PARENT_ANNCE, &payload))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_none(),
            "the normative secured broadcast still reconciles an unconfirmed child"
        );
    }

    #[test]
    #[cfg(feature = "router")]
    fn parent_annce_rsp_requires_security_on_a_secured_network() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        add_confirmed_child(&mut zdo, CHILD_SHORT, CHILD_IEEE);
        block_on(zdo.send_parent_annce()).unwrap();
        let (_, announcement) = last_zdp_tx(&zdo).expect("Parent_annce");
        let tsn = announcement[0];
        zdo.nwk_mut().nib_mut().security_enabled = true;

        let response = parent_annce_rsp(tsn, ZdpStatus::Success as u8, CHILD_IEEE);
        let mut unsecured = unicast(crate::PARENT_ANNCE_RSP, &response);
        unsecured.security_status = false;
        block_on(zdo.handle_indication(&unsecured)).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_some(),
            "an unsecured response cannot relinquish an active child"
        );

        block_on(zdo.handle_indication(&unicast(crate::PARENT_ANNCE_RSP, &response))).unwrap();
        assert!(
            zdo.nwk()
                .neighbor_table()
                .find_by_ieee(&CHILD_IEEE)
                .is_none(),
            "a secured correlated response still relinquishes the child"
        );
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

    #[test]
    #[cfg(feature = "router")]
    fn trust_center_rejects_unauthorized_permit_joining_request() {
        let mut zdo = test_zdo_for(DeviceType::Coordinator);
        zdo.aps_mut().aib_mut().aps_trust_center_address = LOCAL_IEEE;
        zdo.set_trust_center_permit_joining_policy(false, false);

        // R21+ treats the deprecated zero value as policy-significant too.
        let request = [0x42, 30, 0];
        block_on(zdo.handle_indication(&unicast(crate::MGMT_PERMIT_JOINING_REQ, &request)))
            .unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(cluster, crate::MGMT_PERMIT_JOINING_RSP);
        assert_eq!(body.as_slice(), &[0x42, ZDP_STATUS_NOT_AUTHORIZED]);
        assert!(!zdo.nwk().nib().permit_joining);
        assert_eq!(zdo.take_trust_center_allow_joins_update(), None);
    }

    #[test]
    #[cfg(feature = "router")]
    fn trust_center_applies_authorized_permit_joining_request() {
        let mut zdo = test_zdo_for(DeviceType::Coordinator);
        zdo.aps_mut().aib_mut().aps_trust_center_address = LOCAL_IEEE;
        zdo.set_trust_center_permit_joining_policy(true, false);

        let request = [0x43, 30, 1];
        block_on(zdo.handle_indication(&unicast(crate::MGMT_PERMIT_JOINING_REQ, &request)))
            .unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(cluster, crate::MGMT_PERMIT_JOINING_RSP);
        assert_eq!(body.as_slice(), &[0x43, ZdpStatus::Success as u8]);
        assert!(zdo.nwk().nib().permit_joining);
        assert_eq!(zdo.nwk().nib().permit_joining_duration, 30);
        assert_eq!(zdo.take_trust_center_allow_joins_update(), Some(true));
    }

    #[test]
    #[cfg(feature = "router")]
    fn non_trust_center_parent_does_not_apply_trust_center_policy() {
        const REMOTE_TRUST_CENTER: [u8; 8] = [0xA5; 8];

        for device_type in [DeviceType::Router, DeviceType::Coordinator] {
            let mut zdo = test_zdo_for(device_type);
            zdo.aps_mut().aib_mut().aps_trust_center_address = REMOTE_TRUST_CENTER;
            // These settings would deny the request if logical type were
            // incorrectly used as proof that this node owns TC policy.
            zdo.set_trust_center_permit_joining_policy(false, true);

            let request = [0x44, 30, 1];
            block_on(zdo.handle_indication(&unicast(crate::MGMT_PERMIT_JOINING_REQ, &request)))
                .unwrap();

            let (cluster, body) = last_zdp_tx(&zdo).unwrap();
            assert_eq!(cluster, crate::MGMT_PERMIT_JOINING_RSP);
            assert_eq!(body.as_slice(), &[0x44, ZdpStatus::Success as u8]);
            assert!(zdo.nwk().nib().permit_joining);
            assert_eq!(zdo.take_trust_center_allow_joins_update(), None);
        }
    }

    #[test]
    #[cfg(feature = "router")]
    fn permit_joining_policy_denial_uses_valid_zdp_status_encoding() {
        let mut zdo = test_zdo_for(DeviceType::Coordinator);
        zdo.aps_mut().aib_mut().aps_trust_center_address = LOCAL_IEEE;
        zdo.set_trust_center_permit_joining_policy(true, true);

        let request = [0x45, 30, 1];
        block_on(zdo.handle_indication(&unicast(crate::MGMT_PERMIT_JOINING_REQ, &request)))
            .unwrap();

        let (_, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(body.as_slice(), &[0x45, ZDP_STATUS_NOT_AUTHORIZED]);
        assert!(matches!(
            body[1],
            0x00 | 0x80..=0x86 | 0x88..=0x8F
        ));
        assert!(!matches!(body[1], 0xA3 | 0xAA));
        assert!(!zdo.nwk().nib().permit_joining);
        assert_eq!(zdo.take_trust_center_allow_joins_update(), None);
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
    #[cfg(all(feature = "end-device", not(feature = "router")))]
    fn end_device_optional_management_unicasts_answer_not_supported() {
        for cluster in [
            crate::MGMT_LQI_REQ,
            crate::MGMT_RTG_REQ,
            crate::MGMT_PERMIT_JOINING_REQ,
            crate::MGMT_NWK_UPDATE_REQ,
        ] {
            let mut zdo = test_zdo();
            let payload = [0x5Eu8];
            block_on(zdo.handle_indication(&unicast(cluster, &payload))).unwrap();

            let (response_cluster, body) = last_zdp_tx(&zdo).expect("response frame");
            assert_eq!(response_cluster, cluster | ZDP_RESPONSE_BIT);
            assert_eq!(body.as_slice(), &[0x5E, ZdpStatus::NotSupported as u8]);
        }
    }

    #[test]
    #[cfg(all(feature = "end-device", not(feature = "router")))]
    fn end_device_optional_management_broadcasts_are_dropped() {
        for cluster in [
            crate::MGMT_LQI_REQ,
            crate::MGMT_RTG_REQ,
            crate::MGMT_PERMIT_JOINING_REQ,
            crate::MGMT_NWK_UPDATE_REQ,
        ] {
            let mut zdo = test_zdo();
            let payload = [0x5Fu8];
            block_on(zdo.handle_indication(&broadcast(cluster, &payload))).unwrap();

            assert_eq!(tx_count(&zdo), 0);
            assert_eq!(zdo.diagnostics().response_attempts, 0);
        }
    }

    /// The leaf fallback deliberately compiles out optional table and
    /// network-management *servers*, but it must not turn the mandatory
    /// binding-table query or full Mgmt_Leave processing into generic
    /// NOT_SUPPORTED replies.
    #[test]
    #[cfg(all(feature = "end-device", not(feature = "router")))]
    fn end_device_fallback_retains_mgmt_bind_and_mgmt_leave() {
        let mut zdo = test_zdo();

        // Mgmt_Bind_req: TSN followed by the first binding-table index. The
        // empty local table still has the four mandatory response fields.
        block_on(zdo.handle_indication(&unicast(crate::MGMT_BIND_REQ, &[0x60, 0x00]))).unwrap();
        let (cluster, body) = last_zdp_tx(&zdo).expect("Mgmt_Bind_rsp");
        assert_eq!(cluster, crate::MGMT_BIND_RSP);
        assert_eq!(
            body.as_slice(),
            &[0x60, ZdpStatus::Success as u8, 0x00, 0x00, 0x00]
        );

        zdo.nwk_mut().mac_mut().clear_tx_history();
        let mut leave = [0u8; 10];
        leave[0] = 0x61;
        leave[1..9].copy_from_slice(&LOCAL_IEEE);
        // Rejoin=0, removeChildren=0. The parent is the authorized source
        // in `unicast`, so this must reach the real Mgmt_Leave classifier.
        leave[9] = 0;
        block_on(zdo.handle_indication(&unicast(crate::MGMT_LEAVE_REQ, &leave))).unwrap();
        let (cluster, body) = last_zdp_tx(&zdo).expect("Mgmt_Leave_rsp");
        assert_eq!(cluster, crate::MGMT_LEAVE_RSP);
        assert_eq!(body.as_slice(), &[0x61, ZdpStatus::Success as u8]);
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
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
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

    /// Decode the APS header of the last frame the MAC sent.
    fn last_aps_header(zdo: &ZdoLayer<MockMac>) -> Option<zigbee_aps::frames::ApsHeader> {
        let record = zdo.nwk().mac().tx_history().last()?;
        let frame = record.payload.as_slice();
        let (_nwk, nwk_len) = zigbee_nwk::frames::NwkHeader::parse(frame)?;
        let (aps, _) = zigbee_aps::frames::ApsHeader::parse(frame.get(nwk_len..)?)?;
        Some(aps)
    }

    /// Reproduction cover for the ZiGate interview stall (capture 2026-08-09,
    /// frame 1001): a unicast `Simple_Desc_req` addressed to this node for a
    /// registered endpoint must produce a `Simple_Desc_rsp` (0x8004) carrying
    /// the descriptor. z2m times out on cluster 32772 when this frame never
    /// reaches the coordinator.
    #[test]
    fn unicast_simple_desc_request_answers_with_the_descriptor() {
        let mut zdo = test_zdo();
        let mut payload = [0u8; 4];
        payload[0] = 0x51;
        payload[1..3].copy_from_slice(&LOCAL_SHORT.0.to_le_bytes());
        payload[3] = 1;
        block_on(zdo.handle_indication(&unicast(crate::SIMPLE_DESC_REQ, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("Simple_Desc_rsp must be transmitted");
        assert_eq!(cluster, crate::SIMPLE_DESC_RSP);
        assert_eq!(body[0], 0x51, "the response echoes the request TSN");
        assert_eq!(body[1], ZdpStatus::Success as u8);
        assert_eq!(u16::from_le_bytes([body[2], body[3]]), LOCAL_SHORT.0);
        assert!(body[4] > 0, "a descriptor length must be reported");
        assert_eq!(body[5], 1, "endpoint 1");
        assert_eq!(u16::from_le_bytes([body[6], body[7]]), 0x0104);
    }

    /// A `Simple_Desc_req` for an endpoint this node does not have is still
    /// answered — with `NOT_ACTIVE`, never with silence.
    #[test]
    fn unicast_simple_desc_request_for_an_unknown_endpoint_still_answers() {
        let mut zdo = test_zdo();
        let mut payload = [0u8; 4];
        payload[0] = 0x52;
        payload[1..3].copy_from_slice(&LOCAL_SHORT.0.to_le_bytes());
        payload[3] = 9;
        block_on(zdo.handle_indication(&unicast(crate::SIMPLE_DESC_REQ, &payload))).unwrap();

        let (cluster, body) = last_zdp_tx(&zdo).expect("response frame");
        assert_eq!(cluster, crate::SIMPLE_DESC_RSP);
        assert_eq!(body[1], ZdpStatus::NotActive as u8);
    }

    /// R22 §2.4.1.2: a unicast ZDP frame is transmitted with an APS
    /// acknowledgement requested. ZDP has no retry of its own, so the APS
    /// retry is what carries a descriptor response through a transient route
    /// failure — and the acknowledgement is what tells us it did not.
    #[test]
    fn a_unicast_zdp_response_requests_an_aps_acknowledgement() {
        let mut zdo = test_zdo();
        let mut payload = [0u8; 4];
        payload[0] = 0x53;
        payload[1..3].copy_from_slice(&LOCAL_SHORT.0.to_le_bytes());
        payload[3] = 1;
        block_on(zdo.handle_indication(&unicast(crate::SIMPLE_DESC_REQ, &payload))).unwrap();

        let aps = last_aps_header(&zdo).expect("response frame");
        assert!(
            aps.frame_control.ack_request,
            "a unicast ZDP response must request an APS acknowledgement"
        );
        assert_eq!(aps.cluster_id, Some(crate::SIMPLE_DESC_RSP));
    }

    /// A broadcast ZDP frame must never request an acknowledgement: there is
    /// no single peer to answer, and every receiver answering would be a
    /// broadcast storm.
    #[test]
    fn a_broadcast_zdp_frame_never_requests_an_aps_acknowledgement() {
        let mut zdo = test_zdo();
        block_on(zdo.device_annce(LOCAL_SHORT, LOCAL_IEEE)).unwrap();

        let aps = last_aps_header(&zdo).expect("Device_annce frame");
        assert_eq!(aps.cluster_id, Some(crate::DEVICE_ANNCE));
        assert!(
            !aps.frame_control.ack_request,
            "a broadcast must not request an APS acknowledgement"
        );
    }

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

    // ── Mgmt_NWK_Update adoption (R22 §3.4.12) ──────────────

    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    const START_CHANNEL: u8 = 15;
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    const NEW_CHANNEL: u8 = 20;
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    const NEW_MANAGER: ShortAddress = ShortAddress(0x1A2B);

    /// A commissioned device holding a known-good `nwkUpdateId`.
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn test_zdo_on_channel(update_id: Option<u8>) -> ZdoLayer<MockMac> {
        let mut zdo = test_zdo();
        {
            let nib = zdo.nwk_mut().nib_mut();
            nib.logical_channel = START_CHANNEL;
            nib.restore_nwk_update_id(update_id);
        }
        zdo
    }

    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn channel_change(tsn: u8, channel: u8, nwk_update_id: u8) -> [u8; 7] {
        let mut payload = [0u8; 7];
        payload[0] = tsn;
        payload[1..5].copy_from_slice(&(1u32 << channel).to_le_bytes());
        payload[5] = 0xFE;
        payload[6] = nwk_update_id;
        payload
    }

    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn manager_change(tsn: u8, manager: ShortAddress, nwk_update_id: u8) -> [u8; 9] {
        let mut payload = [0u8; 9];
        payload[0] = tsn;
        payload[1..5].copy_from_slice(&0u32.to_le_bytes());
        payload[5] = 0xFF;
        payload[6] = nwk_update_id;
        payload[7..9].copy_from_slice(&manager.0.to_le_bytes());
        payload
    }

    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn zdp_status(zdo: &ZdoLayer<MockMac>) -> u8 {
        let (cluster, body) = last_zdp_tx(zdo).expect("Mgmt_NWK_Update_rsp must be transmitted");
        assert_eq!(cluster, crate::MGMT_NWK_UPDATE_RSP);
        body[1]
    }

    /// The pure classification, independent of any NIB.
    #[test]
    fn update_id_adoption_classification_is_wrap_aware() {
        // Unknown local state accepts anything, and never reports "already
        // applied" — there is no known state to be idempotent against.
        assert_eq!(
            nwk_update_id_adoption(None, 0x00, false),
            UpdateIdAdoption::Adopt
        );
        assert_eq!(
            nwk_update_id_adoption(None, 0xF0, true),
            UpdateIdAdoption::Adopt
        );

        // Strictly newer, including across the wrap.
        assert_eq!(
            nwk_update_id_adoption(Some(5), 6, false),
            UpdateIdAdoption::Adopt
        );
        assert_eq!(
            nwk_update_id_adoption(Some(0xFF), 0x00, false),
            UpdateIdAdoption::Adopt
        );

        // Equal: idempotent only when the requested configuration is already
        // in effect; equal-but-different is a conflict.
        assert_eq!(
            nwk_update_id_adoption(Some(7), 7, true),
            UpdateIdAdoption::AlreadyApplied
        );
        assert_eq!(
            nwk_update_id_adoption(Some(7), 7, false),
            UpdateIdAdoption::Reject
        );

        // Older, and the unorderable half-window, are refused in both
        // directions even when the configuration happens to match.
        assert_eq!(
            nwk_update_id_adoption(Some(7), 6, true),
            UpdateIdAdoption::Reject
        );
        assert_eq!(
            nwk_update_id_adoption(Some(0x00), 0xFF, true),
            UpdateIdAdoption::Reject
        );
        assert_eq!(
            nwk_update_id_adoption(Some(0x00), 0x80, false),
            UpdateIdAdoption::Reject
        );
        assert_eq!(
            nwk_update_id_adoption(Some(0x80), 0x00, false),
            UpdateIdAdoption::Reject
        );
    }

    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_adopts_a_newer_channel_change() {
        let mut zdo = test_zdo_on_channel(Some(4));
        let payload = channel_change(0x61, NEW_CHANNEL, 5);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();

        assert_eq!(zdp_status(&zdo), ZdpStatus::Success as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, NEW_CHANNEL);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(5));
    }

    /// Wrap-aware: 0x00 is newer than 0xFF, not eight generations older.
    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_adopts_a_channel_change_across_the_wrap() {
        let mut zdo = test_zdo_on_channel(Some(0xFF));
        let payload = channel_change(0x62, NEW_CHANNEL, 0x00);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();

        assert_eq!(zdp_status(&zdo), ZdpStatus::Success as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, NEW_CHANNEL);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(0x00));
    }

    /// A stale request must not move the radio off-channel: a device that
    /// followed it would be deaf on a channel the network has left behind.
    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_rejects_a_stale_channel_change_without_retuning() {
        let mut zdo = test_zdo_on_channel(Some(9));
        let payload = channel_change(0x63, NEW_CHANNEL, 8);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();

        assert_eq!(zdp_status(&zdo), ZdpStatus::InvRequestType as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, START_CHANNEL);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(9));
    }

    /// The unorderable half-window distance is refused, not guessed at.
    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_rejects_an_ambiguous_channel_change() {
        let mut zdo = test_zdo_on_channel(Some(0x00));
        let payload = channel_change(0x64, NEW_CHANNEL, 0x80);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();

        assert_eq!(zdp_status(&zdo), ZdpStatus::InvRequestType as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, START_CHANNEL);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(0x00));
    }

    /// Equal update ID, same channel: a retransmission. Confirm, change
    /// nothing.
    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_treats_an_equal_matching_channel_change_as_idempotent() {
        let mut zdo = test_zdo_on_channel(Some(3));
        let payload = channel_change(0x65, START_CHANNEL, 3);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();

        assert_eq!(zdp_status(&zdo), ZdpStatus::Success as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, START_CHANNEL);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(3));
    }

    /// Equal update ID but a *different* channel: two network states claiming
    /// the same update ID. Refuse rather than split the network.
    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_rejects_an_equal_but_conflicting_channel_change() {
        let mut zdo = test_zdo_on_channel(Some(3));
        let payload = channel_change(0x66, NEW_CHANNEL, 3);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();

        assert_eq!(zdp_status(&zdo), ZdpStatus::InvRequestType as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, START_CHANNEL);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(3));
    }

    /// Unknown local state has nothing to order the request against, so the
    /// incoming ID is adopted — and becomes known-good from then on.
    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_adopts_a_channel_change_when_local_state_is_unknown() {
        let mut zdo = test_zdo_on_channel(None);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), None);

        // An update ID that a fabricated local `0` would have called stale.
        let payload = channel_change(0x67, NEW_CHANNEL, 0xF0);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();

        assert_eq!(zdp_status(&zdo), ZdpStatus::Success as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, NEW_CHANNEL);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(0xF0));

        // Now that the state is known, the same request is a conflict-free
        // retransmission, and an older one is refused.
        let repeat = channel_change(0x68, NEW_CHANNEL, 0xF0);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &repeat))).unwrap();
        assert_eq!(zdp_status(&zdo), ZdpStatus::Success as u8);

        let stale = channel_change(0x69, START_CHANNEL, 0xEF);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &stale))).unwrap();
        assert_eq!(zdp_status(&zdo), ZdpStatus::InvRequestType as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, NEW_CHANNEL);
    }

    /// A channel-change request that names no channel is refused before the
    /// update state is even consulted.
    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_rejects_a_channel_change_without_a_channel() {
        let mut zdo = test_zdo_on_channel(Some(3));
        let mut payload = [0u8; 7];
        payload[0] = 0x6A;
        payload[1..5].copy_from_slice(&0u32.to_le_bytes());
        payload[5] = 0xFE;
        payload[6] = 9;
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();

        assert_eq!(zdp_status(&zdo), ZdpStatus::InvRequestType as u8);
        assert_eq!(zdo.nwk().nib().logical_channel, START_CHANNEL);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(3));
    }

    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_nwk_update_manager_change_follows_the_same_rules() {
        // Newer: adopted.
        let mut zdo = test_zdo_on_channel(Some(4));
        let original_manager = zdo.nwk().nib().nwk_manager_addr;
        let payload = manager_change(0x71, NEW_MANAGER, 5);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();
        assert_eq!(zdp_status(&zdo), ZdpStatus::Success as u8);
        assert_eq!(zdo.nwk().nib().nwk_manager_addr, NEW_MANAGER);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(5));

        // Stale: refused, manager untouched.
        let stale = manager_change(0x72, ShortAddress(0x4444), 4);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &stale))).unwrap();
        assert_eq!(zdp_status(&zdo), ZdpStatus::InvRequestType as u8);
        assert_eq!(zdo.nwk().nib().nwk_manager_addr, NEW_MANAGER);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(5));

        // Equal and already applied: idempotent.
        let repeat = manager_change(0x73, NEW_MANAGER, 5);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &repeat))).unwrap();
        assert_eq!(zdp_status(&zdo), ZdpStatus::Success as u8);
        assert_eq!(zdo.nwk().nib().nwk_manager_addr, NEW_MANAGER);

        // Equal but naming a different manager: refused.
        let conflicting = manager_change(0x74, ShortAddress(0x5555), 5);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &conflicting)))
            .unwrap();
        assert_eq!(zdp_status(&zdo), ZdpStatus::InvRequestType as u8);
        assert_eq!(zdo.nwk().nib().nwk_manager_addr, NEW_MANAGER);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(5));

        // Unknown local state adopts, from the original manager.
        let mut zdo = test_zdo_on_channel(None);
        assert_eq!(zdo.nwk().nib().nwk_manager_addr, original_manager);
        let payload = manager_change(0x75, NEW_MANAGER, 0xC0);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_NWK_UPDATE_REQ, &payload))).unwrap();
        assert_eq!(zdp_status(&zdo), ZdpStatus::Success as u8);
        assert_eq!(zdo.nwk().nib().nwk_manager_addr, NEW_MANAGER);
        assert_eq!(zdo.nwk().nib().nwk_update_id(), Some(0xC0));
    }

    // ── Frame-size budget for Mgmt list responses (R22 2.4.4.3.2-4) ──

    fn neighbor_ieee(index: u8) -> [u8; 8] {
        [0xC0, index, 0, 0, 0, 0, 0, 0x01]
    }

    #[test]
    #[cfg(any(not(feature = "end-device"), feature = "router"))]
    fn mgmt_lqi_response_carries_only_the_records_that_fit_one_frame() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        let neighbors = zigbee_nwk::neighbor::MAX_NEIGHBORS.min(16) as u8;
        for index in 0..neighbors {
            zdo.nwk_mut()
                .update_neighbor_address(ShortAddress(0x2000 + index as u16), neighbor_ieee(index));
        }
        let total = zdo.nwk().neighbor_table().len() as u8;
        assert!(total >= 4, "need more neighbors than one frame holds");
        let per_frame = (crate::ZDP_MAX_PAYLOAD - 1 - 4) / NeighborTableRecord::WIRE_SIZE;
        assert_eq!(per_frame, 3);

        for start in [0u8, 3] {
            zdo.nwk_mut().mac_mut().clear_tx_history();
            block_on(zdo.handle_indication(&unicast(crate::MGMT_LQI_REQ, &[0x70, start]))).unwrap();
            let (cluster, body) = last_zdp_tx(&zdo).expect("Mgmt_Lqi_rsp");
            assert_eq!(cluster, crate::MGMT_LQI_RSP);
            assert!(body.len() <= crate::ZDP_MAX_PAYLOAD);
            assert_eq!(&body[..4], &[0x70, ZdpStatus::Success as u8, total, start]);
            assert_eq!(body[4] as usize, per_frame);
            assert_eq!(body.len(), 5 + per_frame * NeighborTableRecord::WIRE_SIZE);
            let first = NeighborTableRecord::parse(&body[5..]).unwrap();
            let expected = zdo
                .nwk()
                .neighbor_table()
                .iter()
                .nth(start as usize)
                .unwrap()
                .network_address;
            assert_eq!(first.network_addr, expected);
        }

        // A start index past the end is an empty, well-formed list.
        block_on(zdo.handle_indication(&unicast(crate::MGMT_LQI_REQ, &[0x71, 0xF0]))).unwrap();
        let (_, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(
            body.as_slice(),
            &[0x71, ZdpStatus::Success as u8, total, 0xF0, 0]
        );
    }

    #[test]
    fn mgmt_bind_response_carries_only_the_records_that_fit_one_frame() {
        let mut zdo = test_zdo();
        for index in 0..6u8 {
            zdo.aps_mut()
                .binding_table_mut()
                .add(BindingEntry::unicast(
                    LOCAL_IEEE,
                    1,
                    0x0006,
                    neighbor_ieee(index),
                    1,
                ))
                .unwrap();
        }
        // Two short group records after the unicast ones.
        for group in [0x0101u16, 0x0102] {
            zdo.aps_mut()
                .binding_table_mut()
                .add(BindingEntry::group(LOCAL_IEEE, 1, 0x0006, group))
                .unwrap();
        }

        block_on(zdo.handle_indication(&unicast(crate::MGMT_BIND_REQ, &[0x72, 0]))).unwrap();
        let (cluster, body) = last_zdp_tx(&zdo).expect("Mgmt_Bind_rsp");
        assert_eq!(cluster, crate::MGMT_BIND_RSP);
        assert!(body.len() <= crate::ZDP_MAX_PAYLOAD);
        // 4 header bytes + 3 × 21-byte unicast records; a 4th would overflow.
        assert_eq!(&body[..5], &[0x72, ZdpStatus::Success as u8, 8, 0, 3]);
        assert_eq!(body.len(), 5 + 3 * 21);

        // Paging continues contiguously: three unicast records plus one
        // 14-byte group record exactly fill the 82-byte budget.
        block_on(zdo.handle_indication(&unicast(crate::MGMT_BIND_REQ, &[0x73, 3]))).unwrap();
        let (_, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(&body[..5], &[0x73, ZdpStatus::Success as u8, 8, 3, 4]);
        assert_eq!(body.len(), crate::ZDP_MAX_PAYLOAD);
        let rsp = MgmtBindRsp::parse(&body[1..]).unwrap();
        assert_eq!(rsp.binding_table_list.len(), 4);
        block_on(zdo.handle_indication(&unicast(crate::MGMT_BIND_REQ, &[0x74, 6]))).unwrap();
        let (_, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(&body[..5], &[0x74, ZdpStatus::Success as u8, 8, 6, 2]);
        assert_eq!(body.len(), 5 + 2 * 14);
    }

    // ── Address requests (R22 2.4.3.1.1-2 / 2.4.4.2.1-2) ──────────

    #[test]
    fn unicast_address_request_with_reserved_request_type_answers_inv_requesttype() {
        let mut zdo = test_zdo();
        let mut nwk_req = [0u8; 11];
        nwk_req[0] = 0x75;
        nwk_req[1..9].copy_from_slice(&LOCAL_IEEE);
        nwk_req[9] = 0x02;
        block_on(zdo.handle_indication(&unicast(crate::NWK_ADDR_REQ, &nwk_req))).unwrap();
        let (cluster, body) = last_zdp_tx(&zdo).expect("NWK_addr_rsp");
        assert_eq!(cluster, crate::NWK_ADDR_RSP);
        assert_eq!(body[1], ZdpStatus::InvRequestType as u8);
        assert_eq!(body.len(), 1 + NwkAddrRsp::MIN_SIZE);

        let short = LOCAL_SHORT.0.to_le_bytes();
        let ieee_req = [0x76, short[0], short[1], 0x7F, 0];
        block_on(zdo.handle_indication(&unicast(crate::IEEE_ADDR_REQ, &ieee_req))).unwrap();
        let (cluster, body) = last_zdp_tx(&zdo).expect("IEEE_addr_rsp");
        assert_eq!(cluster, crate::IEEE_ADDR_RSP);
        assert_eq!(body[1], ZdpStatus::InvRequestType as u8);
        assert_eq!(&body[2..10], &LOCAL_IEEE);

        // Reserved type for an address this node does not own: a broadcast
        // stays silent, a unicast still gets INV_REQUESTTYPE.
        zdo.nwk_mut().mac_mut().clear_tx_history();
        let other = [0x77, 0x21, 0x43, 0x02, 0];
        block_on(zdo.handle_indication(&broadcast(crate::IEEE_ADDR_REQ, &other))).unwrap();
        assert_eq!(tx_count(&zdo), 0);
        block_on(zdo.handle_indication(&unicast(crate::IEEE_ADDR_REQ, &other))).unwrap();
        assert_eq!(
            last_zdp_tx(&zdo).unwrap().1[1],
            ZdpStatus::InvRequestType as u8
        );
    }

    #[test]
    fn extended_address_request_on_an_end_device_reports_zero_associated_devices() {
        let mut zdo = test_zdo();
        let short = LOCAL_SHORT.0.to_le_bytes();
        let req = [0x78, short[0], short[1], 0x01, 0];
        block_on(zdo.handle_indication(&unicast(crate::IEEE_ADDR_REQ, &req))).unwrap();
        let (_, body) = last_zdp_tx(&zdo).unwrap();
        // NumAssocDev = 0 present; StartIndex and list omitted.
        assert_eq!(body.len(), 1 + NwkAddrRsp::MIN_SIZE + 1);
        assert_eq!(body[1], ZdpStatus::Success as u8);
        assert_eq!(body[12], 0);
    }

    /// PROTO-01: every address response this node emits must be accepted by
    /// its own parser (R22 Table 2-92). The Extended/no-devices form is 12
    /// octets: `NumAssocDev = 0` without `StartIndex`.
    #[test]
    fn extended_address_responses_round_trip_through_the_parser() {
        let mut zdo = test_zdo();
        let short = LOCAL_SHORT.0.to_le_bytes();
        let req = [0x7D, short[0], short[1], 0x01, 0];
        block_on(zdo.handle_indication(&unicast(crate::IEEE_ADDR_REQ, &req))).unwrap();
        let (_, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(body.len(), 1 + NwkAddrRsp::MIN_SIZE + 1);
        let rsp = crate::discovery::IeeeAddrRsp::parse(&body[1..])
            .expect("our own Extended response must parse");
        assert_eq!(rsp.status, ZdpStatus::Success);
        assert_eq!(rsp.nwk_addr, LOCAL_SHORT);
        assert_eq!(rsp.num_assoc_dev, 0);
        assert!(rsp.assoc_dev_list.is_empty());
    }

    /// R22 §2.4.3.1.1: NumAssocDev is "the number of entries in the
    /// NWKAddrAssocDevList field" of this frame, not the router's total child
    /// count — otherwise a StartIndex > 0 fragment is self-inconsistent.
    #[test]
    #[cfg(feature = "router")]
    fn router_extended_address_response_counts_only_the_listed_entries() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        add_confirmed_child(&mut zdo, CHILD_SHORT, CHILD_IEEE);
        add_confirmed_child(&mut zdo, CHILD_2_SHORT, CHILD_2_IEEE);
        let short = LOCAL_SHORT.0.to_le_bytes();
        let children = [CHILD_SHORT, CHILD_2_SHORT];
        for start in 0..=2u8 {
            let req = [0x7E, short[0], short[1], 0x01, start];
            block_on(zdo.handle_indication(&unicast(crate::IEEE_ADDR_REQ, &req))).unwrap();
            let (_, body) = last_zdp_tx(&zdo).unwrap();
            let listed = body.len().saturating_sub(1 + NwkAddrRsp::MIN_SIZE + 2) / 2;
            assert_eq!(
                body[12] as usize, listed,
                "StartIndex={start}: NumAssocDev must equal the {listed} listed entries (frame {body:02X?})"
            );
            assert_eq!(body[13], start, "StartIndex echoes the requested offset");
            let rsp = crate::discovery::IeeeAddrRsp::parse(&body[1..])
                .expect("our own Extended response must parse");
            assert_eq!(rsp.start_index, start);
            assert_eq!(&rsp.assoc_dev_list[..], &children[usize::from(start)..]);
        }
    }

    /// A concentrator reserves room for a source route, so a full child table
    /// does not fit: NumAssocDev must advertise only the serialized entries.
    #[test]
    #[cfg(feature = "router")]
    fn concentrator_extended_address_response_never_advertises_unserialized_children() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        let children = zigbee_nwk::neighbor::MAX_NEIGHBORS as u16;
        zdo.nwk_mut().nib_mut().max_children = children as u8;
        for i in 0..children {
            let mut ieee = CHILD_IEEE;
            ieee[7] = i as u8;
            // Restored children are already `Relationship::Child`; skipping the
            // keepalive poll keeps the mock MAC's 16-entry history in bounds.
            assert!(zdo.nwk_mut().restore_child(
                ieee,
                ShortAddress(0x5000 + i),
                false,
                true,
                false,
                8
            ));
        }
        zdo.nwk_mut()
            .start_concentrator(zigbee_nwk::routing::ConcentratorType::LowRam, 0, 0);
        let fits = (crate::ZDP_MAX_PAYLOAD
            - crate::ZDP_SOURCE_ROUTE_RESERVE
            - 1
            - NwkAddrRsp::MIN_SIZE
            - 2)
            / 2;
        assert!(
            fits < usize::from(children),
            "the table must overflow one frame"
        );

        let short = LOCAL_SHORT.0.to_le_bytes();
        let mut start = 0u16;
        while start < children {
            let req = [0x7F, short[0], short[1], 0x01, start as u8];
            block_on(zdo.handle_indication(&unicast(crate::IEEE_ADDR_REQ, &req))).unwrap();
            let (_, body) = last_zdp_tx(&zdo).unwrap();
            let rsp = crate::discovery::IeeeAddrRsp::parse(&body[1..])
                .expect("our own Extended response must parse");
            let expected = fits.min(usize::from(children - start));
            assert_eq!(usize::from(rsp.num_assoc_dev), expected);
            assert_eq!(rsp.assoc_dev_list.len(), expected);
            assert_eq!(body.len(), 1 + NwkAddrRsp::MIN_SIZE + 2 + 2 * expected);
            assert_eq!(rsp.assoc_dev_list[0], ShortAddress(0x5000 + start));
            start += expected as u16;
        }

        // Past the end of a non-empty table: NumAssocDev = 0, StartIndex kept.
        let req = [0x80, short[0], short[1], 0x01, children as u8];
        block_on(zdo.handle_indication(&unicast(crate::IEEE_ADDR_REQ, &req))).unwrap();
        let (_, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(&body[12..], &[0, children as u8]);
        let rsp = crate::discovery::IeeeAddrRsp::parse(&body[1..]).unwrap();
        assert!(rsp.assoc_dev_list.is_empty());
    }

    #[test]
    fn descriptor_requests_follow_the_live_nib_address() {
        let mut zdo = test_zdo();
        // Address-conflict resolution changed the NIB address; the cached
        // ZDO copy is stale.
        zdo.nwk_mut().nib_mut().network_address = ShortAddress(0x2468);
        block_on(zdo.handle_indication(&unicast(crate::NODE_DESC_REQ, &[0x79, 0x68, 0x24])))
            .unwrap();
        let (_, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(&body[..4], &[0x79, ZdpStatus::Success as u8, 0x68, 0x24]);
    }

    #[test]
    #[cfg(feature = "router")]
    fn router_answers_address_requests_for_its_end_device_children() {
        let mut zdo = test_zdo_for(DeviceType::Router);
        add_confirmed_child(&mut zdo, CHILD_SHORT, CHILD_IEEE);
        add_confirmed_child(&mut zdo, CHILD_2_SHORT, CHILD_2_IEEE);

        let mut req = [0u8; 11];
        req[0] = 0x7A;
        req[1..9].copy_from_slice(&CHILD_IEEE);
        block_on(zdo.handle_indication(&broadcast(crate::NWK_ADDR_REQ, &req))).unwrap();
        let (cluster, body) = last_zdp_tx(&zdo).expect("answered for the child");
        assert_eq!(cluster, crate::NWK_ADDR_RSP);
        assert_eq!(body[1], ZdpStatus::Success as u8);
        assert_eq!(&body[2..10], &CHILD_IEEE);
        assert_eq!(&body[10..12], &CHILD_SHORT.0.to_le_bytes());

        // Extended request for the router itself lists its children.
        let short = LOCAL_SHORT.0.to_le_bytes();
        let req = [0x7B, short[0], short[1], 0x01, 1];
        block_on(zdo.handle_indication(&unicast(crate::IEEE_ADDR_REQ, &req))).unwrap();
        let (_, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(body[1], ZdpStatus::Success as u8);
        assert_eq!(body[12], 1, "NumAssocDev counts the one listed entry");
        assert_eq!(body[13], 1, "StartIndex");
        assert_eq!(body.len(), 1 + NwkAddrRsp::MIN_SIZE + 2 + 2);
    }

    // ── Bind/Unbind validation (R22 2.4.3.2.2-3) ──────────────────

    fn bind_status(zdo: &mut ZdoLayer<MockMac>, cluster: u16, payload: &[u8]) -> u8 {
        block_on(zdo.handle_indication(&unicast(cluster, payload))).unwrap();
        let (rsp_cluster, body) = last_zdp_tx(zdo).expect("binding response");
        assert_eq!(rsp_cluster, cluster | ZDP_RESPONSE_BIT);
        assert_eq!(body.len(), 2);
        body[1]
    }

    fn bind_payload(src: [u8; 8], src_ep: u8, dst: BindTarget) -> heapless::Vec<u8, 22> {
        let mut buf = [0u8; 22];
        buf[0] = 0x7C;
        let len = BindReq {
            src_addr: src,
            src_endpoint: src_ep,
            cluster_id: 0x0006,
            dst,
        }
        .serialize(&mut buf[1..])
        .unwrap();
        heapless::Vec::from_slice(&buf[..1 + len]).unwrap()
    }

    #[test]
    fn bind_and_unbind_with_a_reserved_dst_addr_mode_answer_not_supported() {
        let mut zdo = test_zdo();
        let mut payload = [0u8; 22];
        payload[0] = 0x7D;
        payload[1..9].copy_from_slice(&LOCAL_IEEE);
        payload[9] = 1;
        payload[12] = 0x02; // reserved DstAddrMode
        for cluster in [crate::BIND_REQ, crate::UNBIND_REQ] {
            assert_eq!(
                bind_status(&mut zdo, cluster, &payload),
                ZdpStatus::NotSupported as u8
            );
        }
        assert!(zdo.aps().binding_table().is_empty());
    }

    #[test]
    fn bind_rejects_foreign_source_and_invalid_endpoints() {
        let mut zdo = test_zdo();
        let unicast_dst = |dst_endpoint| BindTarget::Unicast {
            dst_addr: [0x42; 8],
            dst_endpoint,
        };
        for cluster in [crate::BIND_REQ, crate::UNBIND_REQ] {
            let foreign = bind_payload([0x99; 8], 1, unicast_dst(1));
            assert_eq!(
                bind_status(&mut zdo, cluster, &foreign),
                ZdpStatus::NotSupported as u8
            );
            for (src_ep, dst) in [
                (0x00, unicast_dst(1)),
                (0xFF, BindTarget::Group(0x0001)),
                (1, unicast_dst(0x00)),
                (1, unicast_dst(0xFF)),
            ] {
                let payload = bind_payload(LOCAL_IEEE, src_ep, dst);
                assert_eq!(
                    bind_status(&mut zdo, cluster, &payload),
                    ZdpStatus::InvalidEp as u8
                );
            }
        }
        assert!(zdo.aps().binding_table().is_empty());
    }

    #[test]
    fn bind_table_full_uses_the_r22_table_full_status() {
        assert_eq!(ZdpStatus::TableFull as u8, 0x8C);
        assert_eq!(ZdpStatus::from_u8(0x8C), Some(ZdpStatus::TableFull));
        assert_eq!(ZdpStatus::from_u8(0x87), None);
        let mut zdo = test_zdo();
        let mut group = 0u16;
        while zdo
            .aps_mut()
            .binding_table_mut()
            .add(BindingEntry::group(LOCAL_IEEE, 1, 0x0006, group))
            .is_ok()
        {
            group += 1;
        }
        let payload = bind_payload(LOCAL_IEEE, 1, BindTarget::Group(0xFFF0));
        assert_eq!(
            bind_status(&mut zdo, crate::BIND_REQ, &payload),
            ZdpStatus::TableFull as u8
        );
    }

    // ── Client request slots (finding: legacy requests leaked slots) ──

    #[test]
    #[allow(deprecated)]
    fn legacy_client_requests_never_leak_pending_slots() {
        let mut zdo = test_zdo();
        let entry = BindingEntry::unicast(LOCAL_IEEE, 1, 0x0006, [0x42; 8], 2);
        for _ in 0..(crate::MAX_PENDING_ZDP * 2) {
            block_on(zdo.bind_req(PARENT, &entry)).unwrap();
            block_on(zdo.unbind_req(PARENT, &entry)).unwrap();
            assert_eq!(block_on(zdo.active_ep_req(PARENT)), Err(ZdpStatus::Timeout));
            assert_eq!(
                block_on(zdo.match_desc_req(PARENT, 0x0104, &[0x0006], &[])),
                Err(ZdpStatus::Timeout)
            );
        }
        assert_eq!(zdo.pending_count(), 0);

        // A send failure also releases the slot.
        zdo.nwk_mut().mac_mut().set_tx_failures(100);
        assert!(block_on(zdo.start_active_ep_req(PARENT)).is_err());
        zdo.nwk_mut().mac_mut().set_tx_failures(0);
        assert_eq!(zdo.pending_count(), 0);
    }

    #[test]
    fn bind_req_sends_the_complete_r22_payload() {
        let mut zdo = test_zdo();
        let entry = BindingEntry::unicast(LOCAL_IEEE, 1, 0x0006, [0x42; 8], 2);
        block_on(zdo.bind_req(PARENT, &entry)).unwrap();
        let (cluster, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(cluster, crate::BIND_REQ);
        assert_eq!(body.len(), 1 + 21);
        let parsed = BindReq::parse(&body[1..]).unwrap();
        assert_eq!(parsed.src_addr, LOCAL_IEEE);
        assert_eq!(
            parsed.dst,
            BindTarget::Unicast {
                dst_addr: [0x42; 8],
                dst_endpoint: 2
            }
        );

        let (slot, tsn) = block_on(zdo.start_unbind_req(PARENT, &parsed)).unwrap();
        let (cluster, body) = last_zdp_tx(&zdo).unwrap();
        assert_eq!(cluster, crate::UNBIND_REQ);
        assert_eq!(body[0], tsn);
        block_on(zdo.handle_indication(&unicast(
            crate::UNBIND_RSP,
            &[tsn, ZdpStatus::NoEntry as u8],
        )))
        .unwrap();
        assert_eq!(
            zdo.take_response(slot).unwrap().as_slice(),
            &[ZdpStatus::NoEntry as u8]
        );
        assert_eq!(zdo.pending_count(), 0);
    }

    #[test]
    fn unicast_request_only_accepts_a_response_from_its_destination() {
        let mut zdo = test_zdo();
        let slot = block_on(zdo.start_node_desc_req(PARENT)).unwrap();
        let tsn = zdo.pending_tsn(slot).unwrap();
        let payload = [tsn, 0x81, 0, 0];
        let mut spoofed = unicast(crate::NODE_DESC_RSP, &payload);
        spoofed.src_address = ApsAddress::Short(ShortAddress(0x5555));
        block_on(zdo.handle_indication(&spoofed)).unwrap();
        assert!(zdo.take_response(slot).is_none());
        assert!(!zdo.deliver_client_response(&spoofed));

        let genuine = unicast(crate::NODE_DESC_RSP, &payload);
        assert!(zdo.deliver_client_response(&genuine));
        assert!(zdo.take_response(slot).is_some());

        // Broadcast discovery accepts any responder; resets drop everything.
        let (slot, _) = block_on(zdo.start_ieee_addr_req(ShortAddress::BROADCAST)).unwrap();
        assert_eq!(zdo.pending_count(), 1);
        zdo.cancel_all_pending();
        assert_eq!(zdo.pending_count(), 0);
        assert_eq!(zdo.pending_tsn(slot), None);
    }

    #[test]
    fn initial_transaction_sequence_number_is_seeded_per_device() {
        let seq_for = |ieee: [u8; 8]| {
            let nwk = NwkLayer::new(MockMac::new(ieee), DeviceType::EndDevice);
            let mut zdo = ZdoLayer::new(ApsLayer::new(nwk));
            zdo.nwk_mut().nib_mut().ieee_address = ieee;
            zdo.next_seq()
        };
        let seeds: heapless::Vec<u8, 4> = [[1u8; 8], [2; 8], [3; 8], [4; 8]]
            .into_iter()
            .map(seq_for)
            .collect();
        assert!(seeds.iter().any(|&seq| seq != seeds[0]));
    }
}
