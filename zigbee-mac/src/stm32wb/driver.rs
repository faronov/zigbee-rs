//! [`crate::MacDriver`] over the STM32WB coprocessor MAC SAP.
//!
//! See the [module documentation](super) for the capability mapping and its
//! documented deviations from a host-controlled software MAC.

use heapless::{Deque, Vec};
use zigbee_types::{ChannelMask, IeeeAddress, MacAddress, PanId, ShortAddress, TxPower};

use super::sap::{
    BeaconNotify, Event, Request, SapAddressMode, SapScanType, WbMacTransport, pib_id, status,
};
use crate::frames::parse_zigbee_beacon;
use crate::pib::{MacPib, PibAttribute, PibError, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacError, PlatformServices};

/// Received data frames buffered while waiting for a confirm or command.
pub const RX_QUEUE_DEPTH: usize = 4;
/// Parent command events buffered between runtime steps.
pub const COMMAND_QUEUE_DEPTH: usize = 6;
/// Children for which the NWK layer holds undelivered indirect frames.
pub const MAX_PENDING_CHILDREN: usize = 16;
/// Indirect MSDUs handed to the coprocessor and not yet confirmed. A
/// transport must be able to hold this many indirect MSDUs plus one direct
/// MSDU at the same time.
pub const MAX_INDIRECT_IN_FLIGHT: usize = 6;
/// Association responses awaiting their COMM-STATUS.indication.
pub const MAX_PENDING_ASSOCIATIONS: usize = 4;

/// Beacon content refresh cadence. The coprocessor answers Beacon Requests
/// autonomously from its PIB, so the runtime's beacon decision (permit join,
/// capacity, update id) is pushed into that PIB at this period.
pub const BEACON_REFRESH_US: u32 = 250_000;

/// Upper bound for a confirm of a short command (SET, RESET, START, direct
/// DATA, PURGE, ORPHAN). The coprocessor answers these in milliseconds; this
/// only guards against a wedged mailbox.
const COMMAND_CONFIRM_TIMEOUT_US: u32 = 1_000_000;
/// MLME-ASSOCIATE/POLL: macResponseWaitTime (32 × 15.36 ms) plus margin.
const ASSOCIATE_CONFIRM_TIMEOUT_US: u32 = 3_000_000;
/// Indirect disassociation waits for the child's poll:
/// macTransactionPersistenceTime (7.68 s) plus margin.
const INDIRECT_CONFIRM_TIMEOUT_US: u32 = 9_000_000;
/// Events drained per non-blocking service call.
const MAX_DRAIN_PER_CALL: usize = 8;

/// IEEE 802.15.4 2.4 GHz page 0 aBaseSuperframeDuration in microseconds.
const BASE_SUPERFRAME_US: u64 = 15_360;

/// Diagnostic counters of the coprocessor adapter.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Stm32wbMacStats {
    pub indirect_handed_off: u32,
    pub indirect_delivered: u32,
    pub indirect_expired: u32,
    pub indirect_failed: u32,
    pub rx_dropped: u32,
    pub commands_dropped: u32,
    pub stray_confirms: u32,
    pub beacon_refreshes: u32,
    pub other_events: u32,
    pub sync_loss: u32,
    /// Direct MCPS-DATA outcomes reported by the coprocessor.
    pub tx_ok: u32,
    pub tx_no_ack: u32,
    pub tx_channel_access_failures: u32,
    pub tx_other_failures: u32,
    /// Direct transmissions that failed CSMA-CA back to back; reset by any
    /// other outcome. A persistent run means the coprocessor sees the
    /// channel busy (local interference or a wedged radio).
    pub tx_channel_access_streak: u32,
}

#[derive(Debug, Clone, Copy)]
struct PendingAssociation {
    device: IeeeAddress,
    short_address: ShortAddress,
    status: AssociationStatus,
}

/// Beacon content currently programmed into the coprocessor PIB.
#[derive(Debug, Clone, PartialEq, Eq)]
struct BeaconState {
    payload: PibPayload,
    association_permit: bool,
}

/// Coprocessor-MAC backend for STM32WB.
pub struct Stm32wbMac<T: WbMacTransport> {
    transport: T,
    pib: MacPib,
    /// `mlme_reset` is synchronous in [`crate::MacDriver`]; the coprocessor
    /// RESET is issued before the next asynchronous primitive.
    reset_pending: Option<bool>,
    /// Active/passive scan collector; beacons outside a scan are dropped.
    scanning: Option<PanDescriptorList>,
    rx: Deque<McpsDataIndication, RX_QUEUE_DEPTH>,
    commands: Deque<MacCommandEvent, COMMAND_QUEUE_DEPTH>,
    pending_children: Vec<ShortAddress, MAX_PENDING_CHILDREN>,
    /// Handles of indirect MSDUs owned by the coprocessor.
    in_flight: Vec<u8, MAX_INDIRECT_IN_FLIGHT>,
    associations: Vec<PendingAssociation, MAX_PENDING_ASSOCIATIONS>,
    beacon: Option<BeaconState>,
    /// `Some` once MLME-START succeeded: next synthetic Beacon Request due.
    next_beacon_refresh_us: Option<u32>,
    next_handle: u8,
    stats: Stm32wbMacStats,
}

impl<T: WbMacTransport> Stm32wbMac<T> {
    /// Wrap a ready coprocessor transport. `extended_address` is the
    /// device's EUI-64 (from UID64), in over-the-air byte order. The
    /// coprocessor is reset and programmed before the first primitive.
    pub fn new(transport: T, extended_address: IeeeAddress) -> Self {
        Self {
            transport,
            pib: MacPib::new(extended_address, 0, 0),
            reset_pending: Some(true),
            scanning: None,
            rx: Deque::new(),
            commands: Deque::new(),
            pending_children: Vec::new(),
            in_flight: Vec::new(),
            associations: Vec::new(),
            beacon: None,
            next_beacon_refresh_us: None,
            next_handle: 0,
            stats: Stm32wbMacStats::default(),
        }
    }

    pub fn stats(&self) -> Stm32wbMacStats {
        self.stats
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    fn record_tx_outcome(&mut self, code: u8) {
        let stats = &mut self.stats;
        let counter = match code {
            status::SUCCESS => &mut stats.tx_ok,
            status::NO_ACK => &mut stats.tx_no_ack,
            status::CHANNEL_ACCESS_FAILURE => &mut stats.tx_channel_access_failures,
            _ => &mut stats.tx_other_failures,
        };
        *counter = counter.wrapping_add(1);
        stats.tx_channel_access_streak = if code == status::CHANNEL_ACCESS_FAILURE {
            stats.tx_channel_access_streak.saturating_add(1)
        } else {
            0
        };
    }

    /// Whether MLME-START has succeeded since the last reset.
    pub fn started(&self) -> bool {
        self.next_beacon_refresh_us.is_some()
    }

    // ── Coprocessor plumbing ─────────────────────────────────

    async fn send(&mut self, request: &Request<'_>) -> Result<(), MacError> {
        self.transport.request(request).await.map_err(map_status)
    }

    /// Store an indication in the bounded queues, handle internal confirms
    /// and return any confirm the caller may be waiting for.
    fn absorb(&mut self, event: Event) -> Option<Event> {
        match event {
            Event::DataIndication(indication) => {
                let indication = McpsDataIndication {
                    src_address: indication.source,
                    dst_address: indication.destination,
                    lqi: indication.lqi,
                    payload: indication.msdu,
                    security_use: indication.security_use,
                };
                if self.rx.push_back(indication).is_err() {
                    self.stats.rx_dropped = self.stats.rx_dropped.wrapping_add(1);
                }
                None
            }
            Event::BeaconNotify(beacon) => {
                if let Some(list) = self.scanning.as_mut() {
                    collect_beacon(list, &beacon);
                }
                None
            }
            Event::AssociateIndication {
                device,
                capability,
                security_use,
            } => {
                let indication = MlmeAssociateIndication {
                    device_address: device,
                    coordinator_address: MacAddress::Short(
                        self.pib.pan_id(),
                        self.pib.short_address(),
                    ),
                    capability_info: CapabilityInfo::from_byte(capability),
                    lqi: 0,
                    security_use,
                };
                self.queue_command(MacCommandEvent::AssociationRequest(indication));
                None
            }
            Event::OrphanIndication { orphan } => {
                let indication = MlmeOrphanIndication {
                    orphan_address: orphan,
                    destination_address: MacAddress::Short(self.pib.pan_id(), ShortAddress(0xFFFF)),
                    lqi: 0,
                    security_use: false,
                };
                self.queue_command(MacCommandEvent::OrphanNotification(indication));
                None
            }
            Event::PollIndication { requester } => {
                if let MacAddress::Short(_, child) = requester {
                    // The NWK layer services this child now and re-arms the
                    // pending flag if frames remain.
                    self.pending_children.retain(|pending| *pending != child);
                }
                let indication = MlmeDataRequestIndication {
                    source_address: requester,
                    destination_address: MacAddress::Short(
                        self.pib.pan_id(),
                        self.pib.short_address(),
                    ),
                    lqi: 0,
                    security_use: false,
                };
                self.queue_command(MacCommandEvent::DataRequest(indication));
                None
            }
            Event::CommStatus {
                destination: Some(MacAddress::Extended(_, device)),
                status,
                ..
            } => {
                if let Some(index) = self
                    .associations
                    .iter()
                    .position(|pending| pending.device == device)
                {
                    let pending = self.associations.swap_remove(index);
                    let result = if status == status::SUCCESS {
                        Ok(())
                    } else {
                        Err(map_status(status))
                    };
                    self.queue_command(MacCommandEvent::AssociationResponseDelivery(
                        MlmeAssociateResponseDelivery {
                            device_address: pending.device,
                            short_address: pending.short_address,
                            status: pending.status,
                            result,
                        },
                    ));
                }
                None
            }
            Event::CommStatus { .. } => None,
            Event::DataConfirm { handle, status, .. } if self.in_flight.contains(&handle) => {
                self.in_flight.retain(|entry| *entry != handle);
                let counter = match status {
                    status::SUCCESS => &mut self.stats.indirect_delivered,
                    status::TRANSACTION_EXPIRED => &mut self.stats.indirect_expired,
                    _ => &mut self.stats.indirect_failed,
                };
                *counter = counter.wrapping_add(1);
                None
            }
            Event::SyncLoss { .. } => {
                self.stats.sync_loss = self.stats.sync_loss.wrapping_add(1);
                None
            }
            Event::DisassociateIndication { .. } | Event::Other { .. } => {
                self.stats.other_events = self.stats.other_events.wrapping_add(1);
                None
            }
            confirm => Some(confirm),
        }
    }

    fn queue_command(&mut self, event: MacCommandEvent) {
        if self.commands.push_back(event).is_err() {
            self.stats.commands_dropped = self.stats.commands_dropped.wrapping_add(1);
        }
    }

    /// Absorb already-pending coprocessor events without waiting.
    async fn drain(&mut self) {
        for _ in 0..MAX_DRAIN_PER_CALL {
            let Some(event) = self.transport.event(0).await else {
                return;
            };
            if self.absorb(event).is_some() {
                self.stats.stray_confirms = self.stats.stray_confirms.wrapping_add(1);
            }
        }
    }

    /// Wait for the confirm selected by `wanted`, absorbing everything else.
    async fn confirm(
        &mut self,
        timeout_us: u32,
        wanted: impl Fn(&Event) -> bool,
    ) -> Result<Event, MacError> {
        let started = self.transport.monotonic_micros();
        loop {
            let elapsed = self.transport.monotonic_micros().wrapping_sub(started);
            let Some(remaining) = timeout_us.checked_sub(elapsed) else {
                return Err(MacError::RadioError);
            };
            let Some(event) = self.transport.event(remaining).await else {
                return Err(MacError::RadioError);
            };
            if let Some(confirm) = self.absorb(event) {
                if wanted(&confirm) {
                    return Ok(confirm);
                }
                self.stats.stray_confirms = self.stats.stray_confirms.wrapping_add(1);
            }
        }
    }

    async fn set_raw(&mut self, attribute: u8, value: &[u8]) -> Result<(), MacError> {
        self.send(&Request::Set { attribute, value }).await?;
        match self
            .confirm(
                COMMAND_CONFIRM_TIMEOUT_US,
                |event| matches!(event, Event::SetConfirm { attribute: a, .. } if *a == attribute),
            )
            .await?
        {
            Event::SetConfirm { status, .. } => status_result(status),
            _ => Err(MacError::Other),
        }
    }

    /// Issue a deferred MLME-RESET and restore the identity the coprocessor
    /// does not own (its EUI-64).
    async fn flush_reset(&mut self) -> Result<(), MacError> {
        let Some(set_default_pib) = self.reset_pending else {
            return Ok(());
        };
        self.send(&Request::Reset { set_default_pib }).await?;
        match self
            .confirm(COMMAND_CONFIRM_TIMEOUT_US, |event| {
                matches!(event, Event::ResetConfirm { .. })
            })
            .await?
        {
            Event::ResetConfirm { status } => status_result(status)?,
            _ => return Err(MacError::Other),
        }
        let extended = self.pib.extended_address();
        self.set_raw(pib_id::EXTENDED_ADDRESS, &extended).await?;
        self.reset_pending = None;
        Ok(())
    }

    fn allocate_handle(&mut self) -> u8 {
        loop {
            let handle = self.next_handle;
            self.next_handle = self.next_handle.wrapping_add(1);
            if !self.in_flight.contains(&handle) {
                return handle;
            }
        }
    }

    /// Next child whose pending NWK frame can be handed to the coprocessor.
    fn virtual_data_request(&mut self) -> Option<MacCommandEvent> {
        if self.in_flight.is_full() || self.pending_children.is_empty() {
            return None;
        }
        let child = self.pending_children.remove(0);
        let pan_id = self.pib.pan_id();
        Some(MacCommandEvent::DataRequest(MlmeDataRequestIndication {
            source_address: MacAddress::Short(pan_id, child),
            destination_address: MacAddress::Short(pan_id, self.pib.short_address()),
            lqi: 0,
            security_use: false,
        }))
    }

    fn beacon_refresh_due(&mut self) -> bool {
        let Some(due) = self.next_beacon_refresh_us else {
            return false;
        };
        let now = self.transport.monotonic_micros();
        if (now.wrapping_sub(due) as i32) < 0 {
            return false;
        }
        self.next_beacon_refresh_us = Some(now.wrapping_add(BEACON_REFRESH_US));
        true
    }

    fn take_command(&mut self) -> Option<MacCommandEvent> {
        if let Some(event) = self.commands.pop_front() {
            return Some(event);
        }
        if let Some(event) = self.virtual_data_request() {
            return Some(event);
        }
        if self.beacon_refresh_due() {
            let pan_id = self.pib.pan_id();
            return Some(MacCommandEvent::BeaconRequest(
                MlmeBeaconRequestIndication {
                    destination_address: MacAddress::Short(pan_id, ShortAddress(0xFFFF)),
                    lqi: 0,
                    security_use: false,
                },
            ));
        }
        None
    }

    async fn data_request(
        &mut self,
        req: &McpsDataRequest<'_>,
        handle: u8,
        indirect: bool,
    ) -> Result<(), MacError> {
        if req.tx_options.security_enabled {
            // Zigbee secures at NWK/APS; MAC security is not used.
            return Err(MacError::Unsupported);
        }
        if req.payload.len() > MAX_MSDU {
            return Err(MacError::FrameTooLong);
        }
        let src_addr_mode = match req.src_addr_mode {
            AddressMode::None => SapAddressMode::None,
            AddressMode::Short => SapAddressMode::Short,
            AddressMode::Extended => SapAddressMode::Extended,
        };
        self.send(&Request::Data {
            src_addr_mode,
            destination: req.dst_address,
            handle,
            ack: req.tx_options.ack_tx,
            indirect,
            msdu: req.payload,
        })
        .await
    }

    fn mirror_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        self.pib.set(attr, value).map_err(|error| match error {
            PibError::InvalidValue => MacError::InvalidParameter,
            PibError::ReadOnly => MacError::Unsupported,
        })
    }
}

/// aMaxMACSafePayloadSize: an MSDU that fits whatever addressing is used.
const MAX_MSDU: usize = 118;

impl<T: WbMacTransport> zigbee_crypto::ForwardAesProvider for Stm32wbMac<T> {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        self.transport.forward_cipher(key)
    }
}

impl<T: WbMacTransport> PlatformServices for Stm32wbMac<T> {
    fn monotonic_micros(&self) -> u32 {
        self.transport.monotonic_micros()
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        self.transport.delay_micros(duration_us).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.transport.fill_random(output)
    }
}

impl<T: WbMacTransport> crate::MacDriver for Stm32wbMac<T> {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        self.flush_reset().await?;
        if req.scan_duration > 14 {
            return Err(MacError::InvalidParameter);
        }
        let scan_type = match req.scan_type {
            ScanType::Ed => SapScanType::Ed,
            ScanType::Active => SapScanType::Active,
            ScanType::Passive => SapScanType::Passive,
            // Orphan scan is an end-device rejoin path the NWK layer does
            // not use; the soft MAC rejects it too.
            ScanType::Orphan => return Err(MacError::Unsupported),
        };
        let channels = req.channel_mask.0 & ChannelMask::ALL_2_4GHZ.0;
        if channels == 0 {
            return Err(MacError::InvalidParameter);
        }
        let per_channel_us = BASE_SUPERFRAME_US * ((1u64 << req.scan_duration) + 1);
        let timeout_us = per_channel_us
            .saturating_mul(u64::from(channels.count_ones()))
            .saturating_add(u64::from(COMMAND_CONFIRM_TIMEOUT_US))
            .min(u64::from(u32::MAX)) as u32;

        self.scanning = Some(PanDescriptorList::new());
        let result = async {
            self.send(&Request::Scan {
                scan_type,
                channels,
                duration: req.scan_duration,
            })
            .await?;
            self.confirm(timeout_us, |event| {
                matches!(event, Event::ScanConfirm { .. })
            })
            .await
        }
        .await;
        let pan_descriptors = self.scanning.take().unwrap_or_default();
        let Event::ScanConfirm {
            status: scan_status,
            energy,
            ..
        } = result?
        else {
            return Err(MacError::Other);
        };

        let mut confirm = MlmeScanConfirm {
            scan_type: req.scan_type,
            pan_descriptors,
            energy_list: EdList::new(),
        };
        if req.scan_type == ScanType::Ed {
            status_result(scan_status)?;
            let scanned = (11u8..=26).filter(|channel| channels & (1 << channel) != 0);
            for (channel, energy) in scanned.zip(energy) {
                let _ = confirm.energy_list.push(EdValue { channel, energy });
            }
            return Ok(confirm);
        }
        // With macAutoRequest = FALSE the coprocessor reports beacons only
        // through BEACON-NOTIFY and returns NO_BEACON in the confirm when its
        // own PAN descriptor list stays empty; the collected list decides.
        if !matches!(scan_status, status::SUCCESS | status::NO_BEACON) {
            return Err(map_status(scan_status));
        }
        if confirm.pan_descriptors.is_empty() {
            return Err(MacError::NoBeacon);
        }
        Ok(confirm)
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        self.flush_reset().await?;
        self.send(&Request::Associate {
            channel: req.channel,
            coordinator: req.coord_address,
            capability: req.capability_info.to_byte(),
        })
        .await?;
        let Event::AssociateConfirm {
            status: assoc_status,
            short_address,
        } = self
            .confirm(ASSOCIATE_CONFIRM_TIMEOUT_US, |event| {
                matches!(event, Event::AssociateConfirm { .. })
            })
            .await?
        else {
            return Err(MacError::Other);
        };
        let status = match assoc_status {
            0x00 => AssociationStatus::Success,
            0x01 => AssociationStatus::PanAtCapacity,
            0x02 => AssociationStatus::PanAccessDenied,
            other => return Err(map_status(other)),
        };
        if status == AssociationStatus::Success {
            if short_address.0 >= 0xFFF8 {
                return Err(MacError::AssociationDenied);
            }
            let pan_id = req.coord_address.pan_id();
            self.mirror_set(PibAttribute::MacPanId, PibValue::PanId(pan_id))?;
            self.mirror_set(
                PibAttribute::MacShortAddress,
                PibValue::ShortAddress(short_address),
            )?;
            self.mirror_set(PibAttribute::PhyCurrentChannel, PibValue::U8(req.channel))?;
            match req.coord_address {
                MacAddress::Short(_, coordinator) => self.mirror_set(
                    PibAttribute::MacCoordShortAddress,
                    PibValue::ShortAddress(coordinator),
                )?,
                MacAddress::Extended(_, coordinator) => self.mirror_set(
                    PibAttribute::MacCoordExtendedAddress,
                    PibValue::ExtendedAddress(coordinator),
                )?,
            }
        }
        Ok(MlmeAssociateConfirm {
            short_address,
            status,
        })
    }

    async fn mlme_associate_response(
        &mut self,
        rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        self.flush_reset().await?;
        // Replace a stale entry for a device that re-sends its request.
        self.associations
            .retain(|pending| pending.device != rsp.device_address);
        if self.associations.is_full() {
            return Err(MacError::TransactionOverflow);
        }
        // The coprocessor queues the response as an indirect transaction,
        // sets Frame Pending for the joiner's EUI-64 and reports delivery
        // through MLME-COMM-STATUS.indication.
        self.send(&Request::AssociateResponse {
            device: rsp.device_address,
            short_address: rsp.short_address,
            status: rsp.status as u8,
        })
        .await?;
        let _ = self.associations.push(PendingAssociation {
            device: rsp.device_address,
            short_address: rsp.short_address,
            status: rsp.status,
        });
        Ok(())
    }

    async fn mlme_beacon_response(&mut self, rsp: MlmeBeaconResponse) -> Result<(), MacError> {
        if rsp.pan_coordinator {
            return Err(MacError::Unsupported);
        }
        if !self.started() {
            return Err(MacError::InvalidParameter);
        }
        self.flush_reset().await?;
        let wanted = BeaconState {
            payload: rsp.beacon_payload,
            association_permit: rsp.association_permit,
        };
        if self.beacon.as_ref() == Some(&wanted) {
            return Ok(());
        }
        // Forget the programmed state first: a partial update must be
        // retried in full on the next refresh.
        self.beacon = None;
        let payload = wanted.payload.as_slice();
        self.set_raw(pib_id::BEACON_PAYLOAD, payload).await?;
        self.set_raw(pib_id::BEACON_PAYLOAD_LENGTH, &[payload.len() as u8])
            .await?;
        self.set_raw(
            pib_id::ASSOCIATION_PERMIT,
            &[u8::from(wanted.association_permit)],
        )
        .await?;
        self.mirror_set(
            PibAttribute::MacBeaconPayload,
            PibValue::Payload(wanted.payload.clone()),
        )?;
        self.mirror_set(
            PibAttribute::MacAssociationPermit,
            PibValue::Bool(wanted.association_permit),
        )?;
        self.beacon = Some(wanted);
        self.stats.beacon_refreshes = self.stats.beacon_refreshes.wrapping_add(1);
        Ok(())
    }

    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError> {
        self.flush_reset().await?;
        self.send(&Request::Disassociate {
            device: req.device_address,
            reason: req.reason as u8,
            tx_indirect: req.tx_indirect,
        })
        .await?;
        let timeout = if req.tx_indirect {
            INDIRECT_CONFIRM_TIMEOUT_US
        } else {
            ASSOCIATE_CONFIRM_TIMEOUT_US
        };
        match self
            .confirm(timeout, |event| {
                matches!(event, Event::DisassociateConfirm { .. })
            })
            .await?
        {
            Event::DisassociateConfirm { status } => status_result(status),
            _ => Err(MacError::Other),
        }
    }

    fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        // A pending full reset already includes a PIB reset.
        let set_default_pib = set_default_pib || self.reset_pending == Some(true);
        self.reset_pending = Some(set_default_pib);
        if set_default_pib {
            self.pib = MacPib::new(self.pib.extended_address(), 0, 0);
        }
        self.scanning = None;
        self.rx.clear();
        self.commands.clear();
        self.pending_children.clear();
        // The coprocessor drops its transactions; their confirms may still
        // arrive and are counted as stray.
        self.in_flight.clear();
        self.associations.clear();
        self.beacon = None;
        self.next_beacon_refresh_us = None;
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        validate_router_start(&req)?;
        self.flush_reset().await?;
        self.send(&Request::Start {
            pan_id: req.pan_id,
            channel: req.channel,
            beacon_order: req.beacon_order,
            superframe_order: req.superframe_order,
            pan_coordinator: req.pan_coordinator,
            battery_life_ext: req.battery_life_ext,
        })
        .await?;
        match self
            .confirm(COMMAND_CONFIRM_TIMEOUT_US, |event| {
                matches!(event, Event::StartConfirm { .. })
            })
            .await?
        {
            Event::StartConfirm { status } => status_result(status)?,
            _ => return Err(MacError::Other),
        }
        self.mirror_set(PibAttribute::MacPanId, PibValue::PanId(req.pan_id))?;
        self.mirror_set(PibAttribute::PhyCurrentChannel, PibValue::U8(req.channel))?;
        self.beacon = None;
        self.next_beacon_refresh_us = Some(self.transport.monotonic_micros());
        Ok(())
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        Ok(self.pib.get(attr))
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        self.flush_reset().await?;
        // Validate against the portable PIB rules before touching CPU2.
        let mut candidate = self.pib.clone();
        candidate
            .set(attr, value.clone())
            .map_err(|error| match error {
                PibError::InvalidValue => MacError::InvalidParameter,
                PibError::ReadOnly => MacError::Unsupported,
            })?;
        match attr {
            // Fixed or host-only attributes: nothing to program.
            PibAttribute::MacBeaconOrder
            | PibAttribute::MacSuperframeOrder
            | PibAttribute::MacBeaconPayloadLength
            | PibAttribute::MacMaxFrameTotalWaitTime
            | PibAttribute::PhyCcaMode
            | PibAttribute::PhyCurrentPage
            | PibAttribute::PhyChannelsSupported => {}
            PibAttribute::MacBeaconPayload => {
                let PibValue::Payload(payload) = &value else {
                    return Err(MacError::InvalidParameter);
                };
                let payload = payload.clone();
                self.set_raw(pib_id::BEACON_PAYLOAD, payload.as_slice())
                    .await?;
                self.set_raw(
                    pib_id::BEACON_PAYLOAD_LENGTH,
                    &[payload.as_slice().len() as u8],
                )
                .await?;
                self.beacon = None;
            }
            _ => {
                let mut wire = [0u8; 8];
                let len = encode_value(&value, &mut wire).ok_or(MacError::InvalidParameter)?;
                self.set_raw(attr as u8, &wire[..len]).await?;
                if attr == PibAttribute::MacAssociationPermit {
                    self.beacon = None;
                }
            }
        }
        self.pib = candidate;
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        self.flush_reset().await?;
        let pan_id = self.pib.pan_id();
        let coordinator = match self.pib.get(PibAttribute::MacCoordShortAddress) {
            PibValue::ShortAddress(short) if short.0 < 0xFFFE => MacAddress::Short(pan_id, short),
            _ => MacAddress::Extended(
                pan_id,
                self.pib
                    .get(PibAttribute::MacCoordExtendedAddress)
                    .as_extended_address()
                    .unwrap_or([0; 8]),
            ),
        };
        self.send(&Request::Poll { coordinator }).await?;
        let Event::PollConfirm {
            status: poll_status,
        } = self
            .confirm(ASSOCIATE_CONFIRM_TIMEOUT_US, |event| {
                matches!(event, Event::PollConfirm { .. })
            })
            .await?
        else {
            return Err(MacError::Other);
        };
        match poll_status {
            status::NO_DATA => Ok(None),
            status::SUCCESS => {
                let index = self
                    .rx
                    .iter()
                    .position(|frame| same_device(&frame.src_address, &coordinator));
                Ok(index.and_then(|index| take_at(&mut self.rx, index).map(|f| f.payload)))
            }
            other => Err(map_status(other)),
        }
    }

    async fn mlme_orphan_response(&mut self, rsp: MlmeOrphanResponse) -> Result<(), MacError> {
        if !rsp.associated_member {
            return Ok(());
        }
        self.flush_reset().await?;
        // Delivery is reported by COMM-STATUS; the runtime treats a sent
        // realignment as complete, so only acceptance is checked here.
        self.send(&Request::OrphanResponse {
            orphan: rsp.orphan_address,
            short_address: rsp.short_address,
            associated_member: true,
        })
        .await
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        if req.tx_options.indirect {
            return Err(MacError::InvalidParameter);
        }
        self.flush_reset().await?;
        let handle = self.allocate_handle();
        self.data_request(&req, handle, false).await?;
        match self
            .confirm(
                COMMAND_CONFIRM_TIMEOUT_US,
                |event| matches!(event, Event::DataConfirm { handle: h, .. } if *h == handle),
            )
            .await?
        {
            Event::DataConfirm {
                status, timestamp, ..
            } => {
                self.record_tx_outcome(status);
                status_result(status)?;
                Ok(McpsDataConfirm {
                    msdu_handle: req.msdu_handle,
                    timestamp: Some(timestamp),
                })
            }
            _ => Err(MacError::Other),
        }
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        loop {
            match self.mcps_data_indication_timeout(u32::MAX / 2).await {
                Err(MacError::NoData) => continue,
                other => return other,
            }
        }
    }

    async fn mcps_data_indication_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<McpsDataIndication, MacError> {
        self.flush_reset().await?;
        let started = self.transport.monotonic_micros();
        loop {
            if let Some(frame) = self.rx.pop_front() {
                return Ok(frame);
            }
            let elapsed = self.transport.monotonic_micros().wrapping_sub(started);
            let Some(remaining) = timeout_us.checked_sub(elapsed) else {
                return Err(MacError::NoData);
            };
            let Some(event) = self.transport.event(remaining).await else {
                return Err(MacError::NoData);
            };
            if self.absorb(event).is_some() {
                self.stats.stray_confirms = self.stats.stray_confirms.wrapping_add(1);
            }
        }
    }

    fn set_indirect_data_pending(
        &mut self,
        child: MacAddress,
        pending: bool,
    ) -> Result<(), MacError> {
        let MacAddress::Short(_, child) = child else {
            return Err(MacError::InvalidParameter);
        };
        if !pending {
            self.pending_children.retain(|entry| *entry != child);
            return Ok(());
        }
        if self.pending_children.contains(&child) {
            return Ok(());
        }
        self.pending_children
            .push(child)
            .map_err(|_| MacError::TransactionOverflow)
    }

    async fn mcps_indirect_data(
        &mut self,
        req: McpsDataRequest<'_>,
    ) -> Result<McpsDataConfirm, MacError> {
        if !matches!(req.dst_address, MacAddress::Short(..)) {
            return Err(MacError::InvalidParameter);
        }
        if self.in_flight.is_full() {
            return Err(MacError::TransactionOverflow);
        }
        self.flush_reset().await?;
        let handle = self.allocate_handle();
        self.data_request(&req, handle, true).await?;
        // Ownership moves to the coprocessor's indirect queue: it holds the
        // frame for macTransactionPersistenceTime, sets Frame Pending in the
        // child's poll ACK and reports the outcome in a later DATA.confirm.
        let _ = self.in_flight.push(handle);
        self.stats.indirect_handed_off = self.stats.indirect_handed_off.wrapping_add(1);
        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    async fn mac_command_event(&mut self) -> Result<MacCommandEvent, MacError> {
        loop {
            match self.mac_command_event_timeout(u32::MAX / 2).await {
                Err(MacError::NoData) => continue,
                other => return other,
            }
        }
    }

    async fn mac_command_event_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<MacCommandEvent, MacError> {
        self.flush_reset().await?;
        self.drain().await;
        if let Some(event) = self.take_command() {
            return Ok(event);
        }
        let started = self.transport.monotonic_micros();
        loop {
            let elapsed = self.transport.monotonic_micros().wrapping_sub(started);
            let Some(remaining) = timeout_us.checked_sub(elapsed).filter(|r| *r > 0) else {
                return Err(MacError::NoData);
            };
            let Some(event) = self.transport.event(remaining).await else {
                return Err(MacError::NoData);
            };
            if self.absorb(event).is_some() {
                self.stats.stray_confirms = self.stats.stray_confirms.wrapping_add(1);
            }
            if let Some(event) = self.take_command() {
                return Ok(event);
            }
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            // A PAN-coordinator start is rejected by `validate_router_start`.
            coordinator: false,
            router: true,
            // Zigbee CCM* runs in the portable Rust stack.
            hardware_security: false,
            max_payload: 102,
            // STM32WB55 radio output range.
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(6),
        }
    }
}

impl<T: WbMacTransport> crate::sealed::SealedParent for Stm32wbMac<T> {}

/// Audited parent capability: the coprocessor MAC answers Beacon Requests
/// from a host-refreshed beacon PIB, queues association responses and
/// indirect data with Frame Pending, and reports Data Request polls and
/// orphan notifications. The deviations are listed in the module docs.
impl<T: WbMacTransport> crate::ParentMacDriver for Stm32wbMac<T> {}

// ── Helpers ─────────────────────────────────────────────────────

/// Map an ST MAC status to the portable error.
pub fn map_status(code: u8) -> MacError {
    match code {
        status::PAN_AT_CAPACITY => MacError::PanAtCapacity,
        status::PAN_ACCESS_DENIED | status::DENIED => MacError::AssociationDenied,
        status::CHANNEL_ACCESS_FAILURE => MacError::ChannelAccessFailure,
        status::NO_ACK => MacError::NoAck,
        status::NO_BEACON => MacError::NoBeacon,
        status::NO_DATA => MacError::NoData,
        status::FRAME_TOO_LONG => MacError::FrameTooLong,
        status::TRANSACTION_EXPIRED => MacError::TransactionExpired,
        status::TRANSACTION_OVERFLOW => MacError::TransactionOverflow,
        status::SCAN_IN_PROGRESS => MacError::ScanInProgress,
        status::TRACKING_OFF => MacError::TrackingOff,
        status::INVALID_PARAMETER
        | status::INVALID_ADDRESS
        | status::INVALID_HANDLE
        | status::INVALID_REQUEST => MacError::InvalidParameter,
        status::UNSUPPORTED_ATTRIBUTE | status::READ_ONLY => MacError::Unsupported,
        status::COUNTER_ERROR | status::SECURITY_ERROR => MacError::SecurityError,
        _ => MacError::Other,
    }
}

fn status_result(code: u8) -> Result<(), MacError> {
    if code == status::SUCCESS {
        Ok(())
    } else {
        Err(map_status(code))
    }
}

/// Little-endian wire form of a scalar PIB value.
fn encode_value(value: &PibValue, out: &mut [u8; 8]) -> Option<usize> {
    let len = match value {
        PibValue::Bool(value) => {
            out[0] = u8::from(*value);
            1
        }
        PibValue::U8(value) => {
            out[0] = *value;
            1
        }
        PibValue::I8(value) => {
            out[0] = *value as u8;
            1
        }
        PibValue::U16(value) => {
            out[..2].copy_from_slice(&value.to_le_bytes());
            2
        }
        PibValue::U32(value) => {
            out[..4].copy_from_slice(&value.to_le_bytes());
            4
        }
        PibValue::ShortAddress(ShortAddress(value)) | PibValue::PanId(PanId(value)) => {
            out[..2].copy_from_slice(&value.to_le_bytes());
            2
        }
        PibValue::ExtendedAddress(value) => {
            out.copy_from_slice(value);
            8
        }
        PibValue::Payload(_) => return None,
    };
    Some(len)
}

fn same_device(a: &MacAddress, b: &MacAddress) -> bool {
    match (a, b) {
        (MacAddress::Short(_, x), MacAddress::Short(_, y)) => x == y,
        (MacAddress::Extended(_, x), MacAddress::Extended(_, y)) => x == y,
        _ => false,
    }
}

fn take_at<V, const N: usize>(queue: &mut Deque<V, N>, index: usize) -> Option<V> {
    for _ in 0..index {
        let value = queue.pop_front()?;
        let _ = queue.push_back(value);
    }
    let taken = queue.pop_front();
    // Restore the original order of the remaining entries.
    let remaining = queue.len();
    for _ in 0..remaining.saturating_sub(index) {
        if let Some(value) = queue.pop_front() {
            let _ = queue.push_back(value);
        }
    }
    taken
}

/// Add a Zigbee beacon to the scan list, keeping the best LQI per
/// coordinator and channel.
fn collect_beacon(list: &mut PanDescriptorList, beacon: &BeaconNotify) {
    if beacon.payload.len() < 15 {
        return; // Not a Zigbee beacon payload.
    }
    let descriptor = PanDescriptor {
        channel: beacon.channel,
        coord_address: beacon.coordinator,
        superframe_spec: SuperframeSpec::from_raw(beacon.superframe_spec),
        lqi: beacon.lqi,
        security_use: beacon.security_use,
        zigbee_beacon: parse_zigbee_beacon(&beacon.payload),
    };
    if let Some(existing) = list.iter_mut().find(|existing| {
        existing.channel == descriptor.channel && existing.coord_address == descriptor.coord_address
    }) {
        if descriptor.lqi > existing.lqi {
            *existing = descriptor;
        }
        return;
    }
    let _ = list.push(descriptor);
}
