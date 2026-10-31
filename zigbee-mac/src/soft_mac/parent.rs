//! Parent (router) role of [`SoftMacCore`].
//!
//! Everything in this module is reachable only from a
//! `SoftMacCore<P, Parent<..>, RX>`, so an end-device image never links it.
//!
//! Model (shared with the Telink parent backend):
//!
//! * received Beacon Request, Association Request, Data Request and Orphan
//!   Notification commands are strictly parsed
//!   ([`crate::frames::parse_parent_command`]) and queued as
//!   [`MacCommandEvent`]s; a queued Data Request ends the current receive so
//!   the runtime services the child poll at once;
//! * MLME-ASSOCIATE.response is an indirect transaction keyed by the child's
//!   EUI-64 and kept for `macTransactionPersistenceTime`; the extended-source
//!   Data Request that retrieves it triggers the acknowledged Association
//!   Response, reported as [`MacCommandEvent::AssociationResponseDelivery`];
//! * Frame Pending in the ACK to a Data Request comes from
//!   [`MacDriver::set_indirect_data_pending`] and the association queue —
//!   in the PHY's hardware/ISR table when `hardware_auto_ack` is set
//!   ([`RadioPhy::set_ack_frame_pending`]), otherwise as the `frame_pending`
//!   argument of the software [`RadioPhy::send_ack`] fallback.

use zigbee_types::{IeeeAddress, MacAddress, PanId};

use super::{
    DATA_INDICATION_WAIT_US, MAX_SCAN_FRAMES_PER_CHANNEL, MacRole, RoleOf, SoftMacCore,
    role_sealed, soft_mac_driver_common,
};
use crate::frames::{
    FrameBuildError, build_association_response, build_coordinator_realignment_orphan_response,
    build_nonbeacon_beacon, parse_parent_command,
};
use crate::{
    MacCapabilities, MacCommandEvent, MacDriver, MacError, MacFrame, McpsDataConfirm,
    McpsDataIndication, McpsDataRequest, MlmeAssociateConfirm, MlmeAssociateRequest,
    MlmeAssociateResponse, MlmeAssociateResponseDelivery, MlmeBeaconResponse,
    MlmeDisassociateRequest, MlmeOrphanResponse, MlmeScanConfirm, MlmeScanRequest,
    MlmeStartRequest, PhyError, PibAttribute, PibValue, PlatformServices, RadioPhy,
};
use zigbee_types::TxPower;

/// aBaseSuperframeDuration (960 symbols × 16 µs): unit of
/// `macTransactionPersistenceTime` in a non-beacon PAN.
const BASE_SUPERFRAME_DURATION_US: u32 = 960 * 16;

struct PendingAssociation {
    response: MlmeAssociateResponse,
    expires_at_us: u32,
}

/// Router/parent role state of a [`SoftMacCore`].
///
/// * `EVENTS` — queued MAC command events awaiting the runtime;
/// * `ASSOC` — concurrent Association Response transactions;
/// * `PENDING` — software Frame Pending table (children with queued indirect
///   NWK data). Unused by hardware auto-ACK PHYs, which keep their own table.
pub struct Parent<const EVENTS: usize = 8, const ASSOC: usize = 8, const PENDING: usize = 8> {
    events: heapless::Deque<MacCommandEvent, EVENTS>,
    associations: heapless::Vec<PendingAssociation, ASSOC>,
    pending: heapless::Vec<MacAddress, PENDING>,
    service_request: bool,
    dropped_events: u16,
}

impl<const EVENTS: usize, const ASSOC: usize, const PENDING: usize> role_sealed::Sealed
    for Parent<EVENTS, ASSOC, PENDING>
{
}

impl<const EVENTS: usize, const ASSOC: usize, const PENDING: usize> MacRole
    for Parent<EVENTS, ASSOC, PENDING>
{
    const PARENT: bool = true;

    fn new_role() -> Self {
        Self {
            events: heapless::Deque::new(),
            associations: heapless::Vec::new(),
            pending: heapless::Vec::new(),
            service_request: false,
            dropped_events: 0,
        }
    }

    fn frame_pending_for(&self, source: &MacAddress) -> bool {
        self.pending.contains(source)
            || matches!(source, MacAddress::Extended(_, address)
                if self.associations.iter().any(|entry| entry.response.device_address == *address))
    }

    fn queue_command(&mut self, data: &[u8], lqi: u8) {
        let Some(event) = parse_parent_command(data, lqi) else {
            return;
        };
        let data_request = matches!(event, MacCommandEvent::DataRequest(_));
        if self.events.push_back(event).is_err() {
            self.dropped_events = self.dropped_events.saturating_add(1);
            return;
        }
        self.service_request |= data_request;
    }

    fn take_service_request(&mut self) -> bool {
        core::mem::take(&mut self.service_request)
    }

    fn clear_role(&mut self) {
        self.events.clear();
        self.associations.clear();
        self.pending.clear();
        self.service_request = false;
    }
}

fn deadline_reached(now_us: u32, deadline_us: u32) -> bool {
    (now_us.wrapping_sub(deadline_us) as i32) >= 0
}

fn map_build_error(error: FrameBuildError) -> MacError {
    match error {
        FrameBuildError::FrameTooLong => MacError::FrameTooLong,
        FrameBuildError::InvalidParameter => MacError::InvalidParameter,
    }
}

fn is_unicast(address: &MacAddress) -> bool {
    match address {
        MacAddress::Short(_, address) => address.0 < 0xFFF8,
        MacAddress::Extended(_, address) => *address != [0xFF; 8],
    }
}

fn delivery(response: MlmeAssociateResponse, result: Result<(), MacError>) -> MacCommandEvent {
    MacCommandEvent::AssociationResponseDelivery(MlmeAssociateResponseDelivery {
        device_address: response.device_address,
        short_address: response.short_address,
        status: response.status,
        result,
    })
}

impl<
    P: RadioPhy + PlatformServices,
    const EVENTS: usize,
    const ASSOC: usize,
    const PENDING: usize,
    const RX: usize,
> SoftMacCore<P, Parent<EVENTS, ASSOC, PENDING>, RX>
{
    /// Command events dropped because the event queue was full.
    pub fn dropped_command_events(&self) -> u16 {
        self.role.dropped_events
    }

    /// MLME-START for a Zigbee non-beacon router.
    pub fn start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        crate::validate_router_start(&req)?;
        let mut next = self.pib.clone();
        next.set(PibAttribute::PhyCurrentChannel, PibValue::U8(req.channel))
            .map_err(Self::map_pib_error)?;
        next.set(PibAttribute::MacPanId, PibValue::PanId(req.pan_id))
            .map_err(Self::map_pib_error)?;
        next.set(PibAttribute::MacRxOnWhenIdle, PibValue::Bool(true))
            .map_err(Self::map_pib_error)?;
        Self::apply_full_config(&mut self.phy, &next)?;
        self.pib = next;
        Ok(())
    }

    /// MLME-ASSOCIATE.response: queue an indirect Association Response for
    /// the child's next extended-source Data Request.
    pub fn associate_response(&mut self, rsp: MlmeAssociateResponse) -> Result<(), MacError> {
        let pan_id = self.pib.pan_id();
        if pan_id == PanId::BROADCAST {
            return Err(MacError::InvalidParameter);
        }
        // Validate the eventual wire frame now.
        build_association_response(self.pib.dsn(), pan_id, &self.pib.extended_address(), &rsp)
            .map_err(map_build_error)?;

        let child = rsp.device_address;
        let expires_at_us = self.monotonic_micros().wrapping_add(
            u32::from(self.pib.transaction_persistence_time()) * BASE_SUPERFRAME_DURATION_US,
        );
        let existing = self
            .role
            .associations
            .iter()
            .position(|entry| entry.response.device_address == child);
        if existing.is_none() && self.role.associations.is_full() {
            return Err(MacError::TransactionOverflow);
        }
        self.set_hardware_frame_pending(&MacAddress::Extended(pan_id, child), true)?;
        let entry = PendingAssociation {
            response: rsp,
            expires_at_us,
        };
        match existing {
            Some(index) => self.role.associations[index] = entry,
            None => {
                let _ = self.role.associations.push(entry);
            }
        }
        Ok(())
    }

    /// Arm or clear Frame Pending for a child with queued indirect data.
    pub fn set_frame_pending(&mut self, child: MacAddress, pending: bool) -> Result<(), MacError> {
        let pan_id = self.pib.pan_id();
        if child.pan_id() != pan_id || pan_id == PanId::BROADCAST || !is_unicast(&child) {
            return Err(MacError::InvalidParameter);
        }
        if self.phy.capabilities().hardware_auto_ack {
            return self.set_hardware_frame_pending(&child, pending);
        }
        let table = &mut self.role.pending;
        let index = table.iter().position(|entry| *entry == child);
        match (index, pending) {
            (None, true) => table.push(child).map_err(|_| MacError::TransactionOverflow),
            (Some(index), false) => {
                table.swap_remove(index);
                Ok(())
            }
            _ => Ok(()),
        }
    }

    /// Transmit an on-demand beacon in reply to a Beacon Request.
    pub async fn beacon_response(&mut self, rsp: MlmeBeaconResponse) -> Result<(), MacError> {
        let source = if self.pib.short_address().0 < 0xFFF8 {
            MacAddress::Short(self.pib.pan_id(), self.pib.short_address())
        } else {
            MacAddress::Extended(self.pib.pan_id(), self.pib.extended_address())
        };
        let frame = build_nonbeacon_beacon(
            self.pib.next_bsn(),
            &source,
            rsp.pan_coordinator,
            rsp.association_permit,
            &rsp.pending_short_addresses,
            &rsp.pending_extended_addresses,
            rsp.beacon_payload.as_slice(),
        )
        .map_err(map_build_error)?;
        self.transmit_csma(&frame).await
    }

    /// Reply to an orphan child with a Coordinator Realignment command.
    pub async fn orphan_response(&mut self, rsp: MlmeOrphanResponse) -> Result<(), MacError> {
        // R22 §3.6.1.4.3.2: a device that is not our child terminates the
        // procedure without any transmission.
        if !rsp.associated_member {
            return Ok(());
        }
        let frame = build_coordinator_realignment_orphan_response(
            self.pib.next_dsn(),
            self.pib.pan_id(),
            self.pib.short_address(),
            &self.pib.extended_address(),
            self.pib.current_channel(),
            &rsp.orphan_address,
            rsp.short_address,
        )
        .map_err(map_build_error)?;
        self.transmit_acknowledged(&frame).await.map(|_| ())
    }

    /// Transmit NWK indirect data to a child that has just polled.
    pub async fn indirect_data(
        &mut self,
        mut req: McpsDataRequest<'_>,
    ) -> Result<McpsDataConfirm, MacError> {
        if !req.tx_options.indirect
            || req.dst_address.pan_id() != self.pib.pan_id()
            || !is_unicast(&req.dst_address)
        {
            return Err(MacError::InvalidParameter);
        }
        // The child is awake right after its Data Request: send directly.
        req.tx_options.indirect = false;
        self.transmit_data(req).await
    }

    /// Next MAC command event, listening up to `timeout_us` for one.
    ///
    /// Expired Association Response transactions and deliveries come first.
    /// Data frames heard while listening are retained for
    /// `mcps_data_indication`. A zero timeout never starts the receiver.
    pub async fn command_event(&mut self, timeout_us: u32) -> Result<MacCommandEvent, MacError> {
        let started_at = self.monotonic_micros();
        for _ in 0..MAX_SCAN_FRAMES_PER_CHANNEL {
            if let Some(event) = self.next_command_event().await {
                return Ok(event);
            }
            let elapsed = self.monotonic_micros().wrapping_sub(started_at);
            let remaining = timeout_us.saturating_sub(elapsed);
            if remaining == 0 {
                return Err(MacError::NoData);
            }
            match self.phy.receive(remaining).await {
                Ok(Some(frame)) => {
                    if self.is_retained(&frame) {
                        self.acknowledge_received_frame(&frame).await?;
                        self.store_retained(frame);
                    }
                }
                Ok(None) => return Err(MacError::NoData),
                Err(PhyError::CrcFailed) => {}
                Err(error) => return Err(Self::map_phy_error(error)),
            }
        }
        Err(MacError::NoData)
    }

    async fn next_command_event(&mut self) -> Option<MacCommandEvent> {
        let now_us = self.monotonic_micros();
        if let Some(index) = self
            .role
            .associations
            .iter()
            .position(|entry| deadline_reached(now_us, entry.expires_at_us))
        {
            let entry = self.role.associations.swap_remove(index);
            self.clear_association_frame_pending(&entry.response.device_address);
            return Some(delivery(entry.response, Err(MacError::TransactionExpired)));
        }

        let Some(event) = self.role.events.pop_front() else {
            self.role.service_request = false;
            return None;
        };
        let MacCommandEvent::DataRequest(indication) = &event else {
            return Some(event);
        };
        match indication.source_address {
            MacAddress::Extended(_, child) => {
                let Some(response) = self
                    .role
                    .associations
                    .iter()
                    .find(|entry| entry.response.device_address == child)
                    .map(|entry| entry.response.clone())
                else {
                    return Some(event);
                };
                // A failed attempt keeps the transaction queued, so a later
                // poll can retry it until macTransactionPersistenceTime.
                match self.transmit_association_response(&response).await {
                    Ok(()) => Some(self.finish_association_delivery(&child, Ok(()))),
                    Err(error @ (MacError::FrameTooLong | MacError::InvalidParameter)) => {
                        Some(self.finish_association_delivery(&child, Err(error)))
                    }
                    Err(_) => Some(event),
                }
            }
            MacAddress::Short(_, short_address) => {
                // The child already uses its new address, so the response
                // reached it even if its ACK was lost: confirm the delivery
                // first and keep the poll for NWK indirect service.
                let Some(child) = self
                    .role
                    .associations
                    .iter()
                    .find(|entry| {
                        entry.response.status == crate::AssociationStatus::Success
                            && entry.response.short_address == short_address
                    })
                    .map(|entry| entry.response.device_address)
                else {
                    return Some(event);
                };
                let _ = self.role.events.push_front(event);
                self.role.service_request = true;
                Some(self.finish_association_delivery(&child, Ok(())))
            }
        }
    }

    async fn transmit_association_response(
        &mut self,
        response: &MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        let frame = build_association_response(
            self.pib.next_dsn(),
            self.pib.pan_id(),
            &self.pib.extended_address(),
            response,
        )
        .map_err(map_build_error)?;
        self.transmit_acknowledged(&frame).await.map(|_| ())
    }

    fn finish_association_delivery(
        &mut self,
        child: &IeeeAddress,
        result: Result<(), MacError>,
    ) -> MacCommandEvent {
        let index = self
            .role
            .associations
            .iter()
            .position(|entry| entry.response.device_address == *child);
        let response = match index {
            Some(index) => self.role.associations.swap_remove(index).response,
            None => MlmeAssociateResponse {
                device_address: *child,
                short_address: zigbee_types::ShortAddress(0xFFFF),
                status: crate::AssociationStatus::PanAccessDenied,
            },
        };
        self.clear_association_frame_pending(child);
        delivery(response, result)
    }

    fn clear_association_frame_pending(&mut self, child: &IeeeAddress) {
        let address = MacAddress::Extended(self.pib.pan_id(), *child);
        let _ = self.set_hardware_frame_pending(&address, false);
    }

    fn set_hardware_frame_pending(
        &mut self,
        address: &MacAddress,
        pending: bool,
    ) -> Result<(), MacError> {
        if !self.phy.capabilities().hardware_auto_ack {
            return Ok(());
        }
        self.phy
            .set_ack_frame_pending(address, pending)
            .map_err(|error| match error {
                PhyError::Hardware => MacError::TransactionOverflow,
                other => Self::map_phy_error(other),
            })
    }
}

impl<
    P: RadioPhy + PlatformServices,
    const EVENTS: usize,
    const ASSOC: usize,
    const PENDING: usize,
    const RX: usize,
> MacDriver for SoftMacCore<P, Parent<EVENTS, ASSOC, PENDING>, RX>
{
    soft_mac_driver_common!();

    async fn mlme_associate_response(
        &mut self,
        rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        self.associate_response(rsp)
    }

    async fn mlme_beacon_response(&mut self, rsp: MlmeBeaconResponse) -> Result<(), MacError> {
        self.beacon_response(rsp).await
    }

    async fn mlme_orphan_response(&mut self, rsp: MlmeOrphanResponse) -> Result<(), MacError> {
        self.orphan_response(rsp).await
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.start(req)
    }

    fn set_indirect_data_pending(
        &mut self,
        child: MacAddress,
        pending: bool,
    ) -> Result<(), MacError> {
        self.set_frame_pending(child, pending)
    }

    async fn mcps_indirect_data(
        &mut self,
        req: McpsDataRequest<'_>,
    ) -> Result<McpsDataConfirm, MacError> {
        self.indirect_data(req).await
    }

    async fn mac_command_event(&mut self) -> Result<MacCommandEvent, MacError> {
        self.command_event(DATA_INDICATION_WAIT_US).await
    }

    async fn mac_command_event_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<MacCommandEvent, MacError> {
        self.command_event(timeout_us).await
    }
}

impl<
    P: RadioPhy + PlatformServices,
    const EVENTS: usize,
    const ASSOC: usize,
    const PENDING: usize,
    const RX: usize,
> crate::sealed::SealedParent for SoftMacCore<P, Parent<EVENTS, ASSOC, PENDING>, RX>
{
}

impl<
    P: RadioPhy + PlatformServices,
    const EVENTS: usize,
    const ASSOC: usize,
    const PENDING: usize,
    const RX: usize,
> crate::ParentMacDriver for SoftMacCore<P, Parent<EVENTS, ASSOC, PENDING>, RX>
{
}
