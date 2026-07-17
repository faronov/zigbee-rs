//! Portable MAC state layered over a hardware radio PHY.

use zigbee_types::MacAddress;

use crate::frames::{
    build_association_request, build_beacon_request, build_data_frame, build_data_request,
    build_data_request_short, build_disassociation_notification, parse_association_response,
    parse_beacon, parse_mac_addresses,
};
use crate::pib::scan_duration_us;
use crate::{
    AssociationStatus, EdValue, MacCapabilities, MacDriver, MacError, MacFrame, MacPib,
    McpsDataConfirm, McpsDataIndication, McpsDataRequest, MlmeAssociateConfirm,
    MlmeAssociateRequest, MlmeAssociateResponse, MlmeDisassociateRequest, MlmeScanConfirm,
    MlmeScanRequest, MlmeStartRequest, PanDescriptor, PhyAddressFilter, PhyError, PhyRxFrame,
    PibAttribute, PibError, PibValue, PlatformServices, RadioPhy, ScanType,
};
use zigbee_types::TxPower;

const UNIT_BACKOFF_PERIOD_US: u32 = 320;
const ACK_WAIT_US: u32 = 1_200;
const ASSOCIATION_DIRECT_WAIT_US: u32 = 500_000;
const POLL_RESPONSE_WAIT_US: u32 = 30_000;
const POST_ASSOCIATION_RX_US: u32 = 250_000;
const ASSOCIATION_POLL_DELAY_US: u32 = 100_000;
const DATA_INDICATION_WAIT_US: u32 = 1_000_000;
const MAX_ACK_WINDOW_FRAMES: u8 = 16;
const MAX_ASSOCIATION_POLLS: u8 = 32;
const MAX_SCAN_FRAMES_PER_CHANNEL: u16 = 256;
const PENDING_RX_CAPACITY: usize = 4;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AckResult {
    pub frame_pending: bool,
}

struct PendingRxFrame {
    frame: PhyRxFrame,
    software_ack_sent: bool,
}

/// Shared software-MAC state for one radio.
///
/// Protocol operations are added incrementally; existing hardware backends
/// continue implementing [`crate::MacDriver`] directly during migration.
pub struct SoftMacCore<P: RadioPhy> {
    phy: P,
    pib: MacPib,
    pending_rx: heapless::Deque<PendingRxFrame, PENDING_RX_CAPACITY>,
    pending_error: Option<MacError>,
    random_state: u32,
}

impl<P: RadioPhy> SoftMacCore<P> {
    pub fn new(mut phy: P, pib: MacPib) -> Result<Self, MacError> {
        Self::apply_full_config(&mut phy, &pib)?;
        let random_state = Self::random_seed(&pib);
        Ok(Self {
            phy,
            pib,
            pending_rx: heapless::Deque::new(),
            pending_error: None,
            random_state,
        })
    }

    pub fn phy(&self) -> &P {
        &self.phy
    }

    pub fn phy_mut(&mut self) -> &mut P {
        &mut self.phy
    }

    pub fn into_phy(self) -> P {
        self.phy
    }

    pub fn pib(&self) -> &MacPib {
        &self.pib
    }

    pub fn get_pib(&self, attr: PibAttribute) -> PibValue {
        self.pib.get(attr)
    }

    pub fn set_pib(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        let mut next = self.pib.clone();
        next.set(attr, value).map_err(Self::map_pib_error)?;
        self.apply_changed_config(attr, &next)?;
        self.pib = next;
        Ok(())
    }

    pub fn reset_pib(&mut self, dsn: u8, bsn: u8) -> Result<(), MacError> {
        let mut next = self.pib.clone();
        next.reset(dsn, bsn);
        Self::apply_full_config(&mut self.phy, &next)?;
        self.pib = next;
        self.pending_rx.clear();
        self.pending_error = None;
        self.random_state = Self::random_seed(&self.pib);
        Ok(())
    }

    pub fn take_pending_rx(&mut self) -> Option<PhyRxFrame> {
        self.pending_rx.pop_front().map(|pending| pending.frame)
    }

    pub fn accepts_destination(&self, destination: &MacAddress) -> bool {
        if self.pib.promiscuous() {
            return true;
        }

        match destination {
            MacAddress::Short(pan_id, address) => {
                (*pan_id == self.pib.pan_id() || pan_id.0 == 0xFFFF)
                    && (*address == self.pib.short_address() || address.0 == 0xFFFF)
            }
            MacAddress::Extended(pan_id, address) => {
                (*pan_id == self.pib.pan_id() || pan_id.0 == 0xFFFF)
                    && *address == self.pib.extended_address()
            }
        }
    }

    pub fn is_exact_destination(&self, destination: &MacAddress) -> bool {
        match destination {
            MacAddress::Short(pan_id, address) => {
                address.0 != 0xFFFF
                    && *pan_id == self.pib.pan_id()
                    && *address == self.pib.short_address()
            }
            MacAddress::Extended(pan_id, address) => {
                *pan_id == self.pib.pan_id() && *address == self.pib.extended_address()
            }
        }
    }

    fn apply_changed_config(&mut self, attr: PibAttribute, next: &MacPib) -> Result<(), MacError> {
        use PibAttribute::*;
        match attr {
            PhyCurrentChannel => self
                .phy
                .set_channel(next.current_channel())
                .map_err(Self::map_phy_error),
            PhyTransmitPower => self
                .phy
                .set_tx_power(next.transmit_power())
                .map_err(Self::map_phy_error),
            MacShortAddress | MacPanId | MacExtendedAddress | MacPromiscuousMode => self
                .phy
                .set_address_filter(Self::address_filter(next))
                .map_err(Self::map_phy_error),
            _ => Ok(()),
        }
    }

    fn apply_full_config(phy: &mut P, pib: &MacPib) -> Result<(), MacError> {
        phy.set_channel(pib.current_channel())
            .map_err(Self::map_phy_error)?;
        phy.set_tx_power(pib.transmit_power())
            .map_err(Self::map_phy_error)?;
        phy.set_address_filter(Self::address_filter(pib))
            .map_err(Self::map_phy_error)
    }

    fn address_filter(pib: &MacPib) -> Option<PhyAddressFilter> {
        (!pib.promiscuous()).then_some(PhyAddressFilter {
            pan_id: pib.pan_id(),
            short_address: pib.short_address(),
            extended_address: pib.extended_address(),
        })
    }

    fn map_pib_error(error: PibError) -> MacError {
        match error {
            PibError::InvalidValue | PibError::ReadOnly => MacError::InvalidParameter,
        }
    }

    fn map_phy_error(error: PhyError) -> MacError {
        match error {
            PhyError::ChannelBusy => MacError::ChannelAccessFailure,
            PhyError::InvalidChannel => MacError::InvalidParameter,
            PhyError::FrameTooLong => MacError::FrameTooLong,
            PhyError::CrcFailed | PhyError::Hardware => MacError::RadioError,
            PhyError::Unsupported => MacError::Unsupported,
        }
    }

    fn random_seed(pib: &MacPib) -> u32 {
        let address = pib.extended_address();
        let seed = u32::from_le_bytes([address[0], address[1], address[2], address[3]])
            ^ u32::from_le_bytes([address[4], address[5], address[6], address[7]])
            ^ u32::from(pib.dsn())
            ^ (u32::from(pib.bsn()) << 8)
            ^ 0x9E37_79B9;
        seed.max(1)
    }
}

impl<P: RadioPhy + PlatformServices> PlatformServices for SoftMacCore<P> {
    fn monotonic_micros(&self) -> u32 {
        self.phy.monotonic_micros()
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        self.phy.delay_micros(duration_us).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.phy.fill_random(output)
    }
}

impl<P: RadioPhy + PlatformServices> SoftMacCore<P> {
    pub async fn associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        self.prepare_association(&req)?;

        let request = build_association_request(
            self.pib.next_dsn(),
            &req.coord_address,
            &self.pib.extended_address(),
            &req.capability_info,
        );
        self.transmit_acknowledged(&request).await?;

        if let Some(response) = self.take_pending_association_response().await? {
            return self.finish_association(response).await;
        }
        if let Some(response) = self
            .wait_for_association_response(ASSOCIATION_DIRECT_WAIT_US)
            .await?
        {
            return self.finish_association(response).await;
        }

        for _ in 0..MAX_ASSOCIATION_POLLS {
            let poll = build_data_request(
                self.pib.next_dsn(),
                &req.coord_address,
                &self.pib.extended_address(),
            );
            self.transmit_acknowledged(&poll).await?;

            if let Some(response) = self.take_pending_association_response().await? {
                return self.finish_association(response).await;
            }
            if let Some(response) = self
                .wait_for_association_response(POLL_RESPONSE_WAIT_US)
                .await?
            {
                return self.finish_association(response).await;
            }
            self.delay_micros(ASSOCIATION_POLL_DELAY_US).await;
        }

        Err(MacError::NoData)
    }

    pub async fn disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError> {
        if req.tx_indirect {
            return Err(MacError::Unsupported);
        }
        if self.pib.short_address().0 >= 0xFFF8
            || req.device_address.pan_id() != self.pib.pan_id()
            || match req.device_address {
                MacAddress::Short(_, address) => address.0 >= 0xFFF8,
                MacAddress::Extended(_, address) => address == [0; 8] || address == [0xFF; 8],
            }
        {
            return Err(MacError::InvalidParameter);
        }

        let notification = build_disassociation_notification(
            self.pib.next_dsn(),
            &req.device_address,
            self.pib.short_address(),
            &self.pib.extended_address(),
            req.reason,
        );
        let transmit_result = self.transmit_acknowledged(&notification).await;
        let clear_result = self.clear_association();
        clear_result?;
        transmit_result.map(|_| ())
    }

    pub fn reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            let random = self.next_random_u32();
            return self.reset_pib(random as u8, (random >> 8) as u8);
        }

        Self::apply_full_config(&mut self.phy, &self.pib)?;
        self.pending_rx.clear();
        self.pending_error = None;
        Ok(())
    }

    pub async fn transmit_data(
        &mut self,
        req: McpsDataRequest<'_>,
    ) -> Result<McpsDataConfirm, MacError> {
        if req.tx_options.indirect {
            return Err(MacError::Unsupported);
        }
        if req.tx_options.security_enabled {
            return Err(MacError::SecurityError);
        }
        if req.src_addr_mode == crate::AddressMode::Short && self.pib.short_address().0 >= 0xFFF8 {
            return Err(MacError::InvalidParameter);
        }

        let ack_requested = req.tx_options.ack_tx
            && !matches!(
                req.dst_address,
                MacAddress::Short(_, address) if address.0 == 0xFFFF
            );
        let frame = build_data_frame(
            self.pib.next_dsn(),
            req.src_addr_mode,
            self.pib.short_address(),
            &self.pib.extended_address(),
            &req.dst_address,
            req.payload,
            ack_requested,
        )
        .map_err(|_| MacError::FrameTooLong)?;

        if ack_requested {
            self.transmit_acknowledged(&frame).await?;
        } else {
            self.transmit_csma(&frame).await?;
        }

        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    pub async fn poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        if let Some(frame) = self.take_pending_data().await? {
            return Ok(Some(frame));
        }
        if let Some(error) = self.pending_error.take() {
            return Err(error);
        }

        let own_short = self.pib.short_address();
        let own_extended = self.pib.extended_address();
        let coordinator_short = self.pib.coord_short_address();
        let coordinator_extended = self.pib.coord_extended_address();
        let has_short_source = own_short.0 < 0xFFF8;
        let has_extended_source = own_extended != [0; 8];
        let coordinator = if coordinator_short.0 < 0xFFF8 {
            MacAddress::Short(self.pib.pan_id(), coordinator_short)
        } else if coordinator_extended != [0; 8] {
            MacAddress::Extended(self.pib.pan_id(), coordinator_extended)
        } else {
            return Err(MacError::InvalidParameter);
        };
        if !has_short_source && !has_extended_source {
            return Err(MacError::InvalidParameter);
        }

        let passes = if has_short_source && has_extended_source {
            2
        } else {
            1
        };
        for pass in 0..passes {
            let sequence = self.pib.next_dsn();
            let request = if pass == 0 && has_short_source {
                build_data_request_short(sequence, &coordinator, own_short)
            } else {
                build_data_request(sequence, &coordinator, &own_extended)
            };

            let ack = self.transmit_acknowledged(&request).await?;
            if let Some(frame) = self.take_pending_data().await? {
                return Ok(Some(frame));
            }
            if !ack.frame_pending {
                continue;
            }

            match self.receive_data(POLL_RESPONSE_WAIT_US).await {
                Ok(indication) => return Ok(Some(indication.payload)),
                Err(MacError::NoData) => {}
                Err(error) => return Err(error),
            }
        }

        Ok(None)
    }

    pub async fn receive_data(&mut self, timeout_us: u32) -> Result<McpsDataIndication, MacError> {
        let started_at = self.monotonic_micros();

        for _ in 0..MAX_SCAN_FRAMES_PER_CHANNEL {
            let pending = if let Some(pending) = self.take_pending_entry() {
                Some(pending)
            } else if let Some(error) = self.pending_error.take() {
                return Err(error);
            } else {
                let elapsed = self.monotonic_micros().wrapping_sub(started_at);
                let Some(remaining) = timeout_us.checked_sub(elapsed) else {
                    return Err(MacError::NoData);
                };
                match self.phy.receive(remaining).await {
                    Ok(frame) => frame.map(|frame| PendingRxFrame {
                        frame,
                        software_ack_sent: false,
                    }),
                    Err(PhyError::CrcFailed) => continue,
                    Err(error) => return Err(Self::map_phy_error(error)),
                }
            };

            let Some(pending) = pending else {
                return Err(MacError::NoData);
            };
            if let Some(indication) = self
                .process_received_data(pending.frame, pending.software_ack_sent)
                .await?
            {
                return Ok(indication);
            }
        }

        Err(MacError::NoData)
    }

    pub async fn scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        if req.scan_duration > 14 {
            return Err(MacError::InvalidParameter);
        }
        if req.scan_type == ScanType::Orphan {
            return Err(MacError::Unsupported);
        }

        let original_channel = self.pib.current_channel();
        self.phy
            .set_address_filter(None)
            .map_err(Self::map_phy_error)?;

        let scan_result = self.scan_unfiltered(req).await;
        let restore_result = self
            .set_pib(
                PibAttribute::PhyCurrentChannel,
                PibValue::U8(original_channel),
            )
            .and_then(|()| {
                self.phy
                    .set_address_filter(Self::address_filter(&self.pib))
                    .map_err(Self::map_phy_error)
            });

        restore_result?;
        scan_result
    }

    pub async fn transmit_csma(&mut self, frame: &[u8]) -> Result<(), MacError> {
        let mut backoff_count = 0u8;
        let mut backoff_exponent = self.pib.min_be();

        loop {
            let slots = self.random_backoff_slots(backoff_exponent);
            if slots != 0 {
                self.delay_micros(u32::from(slots) * UNIT_BACKOFF_PERIOD_US)
                    .await;
            }

            match self.phy.try_transmit(frame).await {
                Ok(()) => return Ok(()),
                Err(PhyError::ChannelBusy) if backoff_count < self.pib.max_csma_backoffs() => {
                    backoff_count = backoff_count.saturating_add(1);
                    backoff_exponent =
                        core::cmp::min(backoff_exponent.saturating_add(1), self.pib.max_be());
                }
                Err(error) => return Err(Self::map_phy_error(error)),
            }
        }
    }

    pub async fn transmit_acknowledged(&mut self, frame: &[u8]) -> Result<AckResult, MacError> {
        let sequence = *frame.get(2).ok_or(MacError::InvalidParameter)?;

        for _ in 0..=self.pib.max_frame_retries() {
            self.transmit_csma(frame).await?;
            if let Some(ack) = self.wait_for_ack(sequence).await? {
                return Ok(ack);
            }
        }

        Err(MacError::NoAck)
    }

    fn random_backoff_slots(&mut self, backoff_exponent: u8) -> u16 {
        let random = self.next_random_u32();
        let mask = (1u16 << backoff_exponent) - 1;
        random as u16 & mask
    }

    fn next_random_u32(&mut self) -> u32 {
        let mut value = self.random_state;
        value ^= value << 13;
        value ^= value >> 17;
        value ^= value << 5;
        self.random_state = value.max(1);
        value
    }

    async fn wait_for_ack(&mut self, sequence: u8) -> Result<Option<AckResult>, MacError> {
        let started_at = self.monotonic_micros();

        for _ in 0..MAX_ACK_WINDOW_FRAMES {
            let elapsed = self.monotonic_micros().wrapping_sub(started_at);
            let Some(remaining) = ACK_WAIT_US.checked_sub(elapsed) else {
                return Ok(None);
            };

            match self.phy.receive(remaining).await {
                Ok(Some(frame)) => {
                    let data = frame.as_slice();
                    if data.len() >= 2 && data[0] & 0x07 == 0x02 {
                        if data.len() >= 3 && data[2] == sequence {
                            return Ok(Some(AckResult {
                                frame_pending: data[0] & (1 << 4) != 0,
                            }));
                        }
                        continue;
                    }
                    self.queue_pending_rx(frame, false);
                }
                Ok(None) => return Ok(None),
                Err(PhyError::CrcFailed) => {}
                Err(error) => return Err(Self::map_phy_error(error)),
            }
        }

        Ok(None)
    }

    fn queue_pending_rx(&mut self, frame: PhyRxFrame, software_ack_sent: bool) {
        if self.pending_rx.is_full() {
            let _ = self.pending_rx.pop_front();
        }
        let _ = self.pending_rx.push_back(PendingRxFrame {
            frame,
            software_ack_sent,
        });
    }

    fn take_pending_entry(&mut self) -> Option<PendingRxFrame> {
        self.pending_rx.pop_front()
    }

    async fn process_received_data(
        &mut self,
        frame: PhyRxFrame,
        software_ack_sent: bool,
    ) -> Result<Option<McpsDataIndication>, MacError> {
        let data = frame.as_slice();
        if data.len() < 3 {
            return Ok(None);
        }

        let frame_control = u16::from_le_bytes([data[0], data[1]]);
        if frame_control & 0x07 != 0x01 {
            return Ok(None);
        }

        let (source, destination, payload_offset, security_use) = parse_mac_addresses(data);
        if payload_offset < 3
            || payload_offset > data.len()
            || !self.accepts_destination(&destination)
        {
            return Ok(None);
        }

        let ack_requested = frame_control & (1 << 5) != 0;
        if ack_requested
            && !software_ack_sent
            && self.is_exact_destination(&destination)
            && !self.phy.capabilities().hardware_auto_ack
        {
            self.phy
                .send_ack(data[2], false)
                .await
                .map_err(Self::map_phy_error)?;
        }

        let payload =
            MacFrame::from_slice(&data[payload_offset..]).ok_or(MacError::FrameTooLong)?;
        Ok(Some(McpsDataIndication {
            src_address: source,
            dst_address: destination,
            lqi: frame.lqi,
            payload,
            security_use,
        }))
    }

    async fn take_pending_data(&mut self) -> Result<Option<MacFrame>, MacError> {
        while let Some(pending) = self.take_pending_entry() {
            if let Some(indication) = self
                .process_received_data(pending.frame, pending.software_ack_sent)
                .await?
            {
                return Ok(Some(indication.payload));
            }
        }
        Ok(None)
    }

    fn prepare_association(&mut self, req: &MlmeAssociateRequest) -> Result<(), MacError> {
        if self.pib.extended_address() == [0; 8]
            || self.pib.extended_address() == [0xFF; 8]
            || req.coord_address.pan_id().0 == 0xFFFF
            || match req.coord_address {
                MacAddress::Short(_, address) => address.0 >= 0xFFF8,
                MacAddress::Extended(_, address) => address == [0; 8] || address == [0xFF; 8],
            }
        {
            return Err(MacError::InvalidParameter);
        }

        let mut next = self.pib.clone();
        next.set(PibAttribute::PhyCurrentChannel, PibValue::U8(req.channel))
            .map_err(Self::map_pib_error)?;
        next.set(
            PibAttribute::MacPanId,
            PibValue::PanId(req.coord_address.pan_id()),
        )
        .map_err(Self::map_pib_error)?;
        next.set(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(zigbee_types::ShortAddress(0xFFFF)),
        )
        .map_err(Self::map_pib_error)?;
        match req.coord_address {
            MacAddress::Short(_, address) => {
                next.set(
                    PibAttribute::MacCoordShortAddress,
                    PibValue::ShortAddress(address),
                )
                .map_err(Self::map_pib_error)?;
                next.set(
                    PibAttribute::MacCoordExtendedAddress,
                    PibValue::ExtendedAddress([0; 8]),
                )
                .map_err(Self::map_pib_error)?;
            }
            MacAddress::Extended(_, address) => {
                next.set(
                    PibAttribute::MacCoordShortAddress,
                    PibValue::ShortAddress(zigbee_types::ShortAddress(0xFFFF)),
                )
                .map_err(Self::map_pib_error)?;
                next.set(
                    PibAttribute::MacCoordExtendedAddress,
                    PibValue::ExtendedAddress(address),
                )
                .map_err(Self::map_pib_error)?;
            }
        }
        next.set(PibAttribute::MacAssociatedPanCoord, PibValue::Bool(false))
            .map_err(Self::map_pib_error)?;

        Self::apply_full_config(&mut self.phy, &next)?;
        self.pib = next;
        self.pending_rx.clear();
        self.pending_error = None;
        Ok(())
    }

    fn clear_association(&mut self) -> Result<(), MacError> {
        let mut next = self.pib.clone();
        next.set(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(zigbee_types::ShortAddress(0xFFFF)),
        )
        .map_err(Self::map_pib_error)?;
        next.set(
            PibAttribute::MacPanId,
            PibValue::PanId(zigbee_types::PanId(0xFFFF)),
        )
        .map_err(Self::map_pib_error)?;
        next.set(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(zigbee_types::ShortAddress(0xFFFF)),
        )
        .map_err(Self::map_pib_error)?;
        next.set(
            PibAttribute::MacCoordExtendedAddress,
            PibValue::ExtendedAddress([0; 8]),
        )
        .map_err(Self::map_pib_error)?;
        next.set(PibAttribute::MacAssociatedPanCoord, PibValue::Bool(false))
            .map_err(Self::map_pib_error)?;

        Self::apply_full_config(&mut self.phy, &next)?;
        self.pib = next;
        self.pending_rx.clear();
        self.pending_error = None;
        Ok(())
    }

    async fn take_pending_association_response(
        &mut self,
    ) -> Result<Option<(zigbee_types::ShortAddress, u8)>, MacError> {
        let pending_count = self.pending_rx.len();
        for _ in 0..pending_count {
            let Some(pending) = self.take_pending_entry() else {
                break;
            };
            if let Some(response) = self.association_response_for_us(&pending.frame) {
                if !pending.software_ack_sent {
                    self.acknowledge_received_frame(&pending.frame).await?;
                }
                return Ok(Some(response));
            }
            let _ = self.pending_rx.push_back(pending);
        }
        Ok(None)
    }

    async fn wait_for_association_response(
        &mut self,
        timeout_us: u32,
    ) -> Result<Option<(zigbee_types::ShortAddress, u8)>, MacError> {
        if let Some(response) = self.take_pending_association_response().await? {
            return Ok(Some(response));
        }

        let started_at = self.monotonic_micros();
        for _ in 0..MAX_SCAN_FRAMES_PER_CHANNEL {
            let elapsed = self.monotonic_micros().wrapping_sub(started_at);
            let Some(remaining) = timeout_us.checked_sub(elapsed) else {
                return Ok(None);
            };
            let frame = match self.phy.receive(remaining).await {
                Ok(Some(frame)) => frame,
                Ok(None) => return Ok(None),
                Err(PhyError::CrcFailed) => continue,
                Err(error) => return Err(Self::map_phy_error(error)),
            };

            if let Some(response) = self.association_response_for_us(&frame) {
                self.acknowledge_received_frame(&frame).await?;
                return Ok(Some(response));
            }
            self.retain_data_frame(frame).await?;
        }
        Ok(None)
    }

    fn association_response_for_us(
        &self,
        frame: &PhyRxFrame,
    ) -> Option<(zigbee_types::ShortAddress, u8)> {
        let data = frame.as_slice();
        let (_, destination, payload_offset, _) = parse_mac_addresses(data);
        (payload_offset >= 3
            && payload_offset <= data.len()
            && self.is_exact_destination(&destination))
        .then(|| parse_association_response(data))
        .flatten()
    }

    async fn finish_association(
        &mut self,
        (short_address, status): (zigbee_types::ShortAddress, u8),
    ) -> Result<MlmeAssociateConfirm, MacError> {
        let status = match status {
            0x00 => AssociationStatus::Success,
            0x01 => AssociationStatus::PanAtCapacity,
            _ => AssociationStatus::PanAccessDenied,
        };
        if status == AssociationStatus::Success {
            if short_address.0 >= 0xFFF8 {
                return Err(MacError::AssociationDenied);
            }
            let mut next = self.pib.clone();
            next.set(
                PibAttribute::MacShortAddress,
                PibValue::ShortAddress(short_address),
            )
            .map_err(Self::map_pib_error)?;
            next.set(PibAttribute::MacAssociatedPanCoord, PibValue::Bool(true))
                .map_err(Self::map_pib_error)?;
            Self::apply_full_config(&mut self.phy, &next)?;
            self.pib = next;
            if let Err(error) = self.capture_post_association_data().await {
                self.pending_error = Some(error);
            }
        }
        Ok(MlmeAssociateConfirm {
            short_address,
            status,
        })
    }

    async fn capture_post_association_data(&mut self) -> Result<(), MacError> {
        let started_at = self.monotonic_micros();
        for _ in 0..MAX_SCAN_FRAMES_PER_CHANNEL {
            let elapsed = self.monotonic_micros().wrapping_sub(started_at);
            let Some(remaining) = POST_ASSOCIATION_RX_US.checked_sub(elapsed) else {
                break;
            };
            match self.phy.receive(remaining).await {
                Ok(Some(frame)) => {
                    self.retain_data_frame(frame).await?;
                }
                Ok(None) => break,
                Err(PhyError::CrcFailed) => {}
                Err(error) => return Err(Self::map_phy_error(error)),
            }
        }
        Ok(())
    }

    async fn retain_data_frame(&mut self, frame: PhyRxFrame) -> Result<(), MacError> {
        let data = frame.as_slice();
        if data.len() < 3 || data[0] & 0x07 != 0x01 {
            return Ok(());
        }
        let (_, destination, payload_offset, _) = parse_mac_addresses(data);
        if payload_offset < 3
            || payload_offset > data.len()
            || !self.accepts_destination(&destination)
        {
            return Ok(());
        }
        let software_ack_sent = self.acknowledge_received_frame(&frame).await?;
        self.queue_pending_rx(frame, software_ack_sent);
        Ok(())
    }

    async fn acknowledge_received_frame(&mut self, frame: &PhyRxFrame) -> Result<bool, MacError> {
        let data = frame.as_slice();
        if data.len() < 3 {
            return Ok(false);
        }
        let frame_control = u16::from_le_bytes([data[0], data[1]]);
        let (_, destination, payload_offset, _) = parse_mac_addresses(data);
        if frame_control & (1 << 5) == 0
            || payload_offset < 3
            || payload_offset > data.len()
            || !self.is_exact_destination(&destination)
            || self.phy.capabilities().hardware_auto_ack
        {
            return Ok(false);
        }
        self.phy
            .send_ack(data[2], false)
            .await
            .map_err(Self::map_phy_error)?;
        Ok(true)
    }

    async fn scan_unfiltered(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let duration_us = u32::try_from(scan_duration_us(req.scan_duration))
            .map_err(|_| MacError::InvalidParameter)?;
        let mut pan_descriptors = heapless::Vec::new();
        let mut energy_list = heapless::Vec::new();

        for channel in req.channel_mask.iter() {
            let channel = channel.number();
            self.set_pib(PibAttribute::PhyCurrentChannel, PibValue::U8(channel))?;

            match req.scan_type {
                ScanType::Ed => {
                    let energy = self
                        .phy
                        .energy_detect(duration_us)
                        .await
                        .map_err(Self::map_phy_error)?;
                    let _ = energy_list.push(EdValue { channel, energy });
                }
                ScanType::Active => {
                    let beacon_request = build_beacon_request(self.pib.next_dsn());
                    self.transmit_csma(&beacon_request).await?;
                    self.collect_beacons(channel, duration_us, &mut pan_descriptors)
                        .await?;
                }
                ScanType::Passive => {
                    self.collect_beacons(channel, duration_us, &mut pan_descriptors)
                        .await?;
                }
                ScanType::Orphan => unreachable!(),
            }
        }

        if matches!(req.scan_type, ScanType::Active | ScanType::Passive)
            && pan_descriptors.is_empty()
        {
            return Err(MacError::NoBeacon);
        }

        Ok(MlmeScanConfirm {
            scan_type: req.scan_type,
            pan_descriptors,
            energy_list,
        })
    }

    async fn collect_beacons(
        &mut self,
        channel: u8,
        duration_us: u32,
        descriptors: &mut heapless::Vec<PanDescriptor, { crate::primitives::MAX_PAN_DESCRIPTORS }>,
    ) -> Result<(), MacError> {
        let started_at = self.monotonic_micros();

        for _ in 0..MAX_SCAN_FRAMES_PER_CHANNEL {
            let elapsed = self.monotonic_micros().wrapping_sub(started_at);
            let Some(remaining) = duration_us.checked_sub(elapsed) else {
                break;
            };

            match self.phy.receive(remaining).await {
                Ok(Some(frame)) => {
                    if let Some(descriptor) = parse_beacon(channel, frame.as_slice(), frame.lqi) {
                        Self::upsert_pan_descriptor(descriptors, descriptor);
                    }
                }
                Ok(None) => break,
                Err(PhyError::CrcFailed) => {}
                Err(error) => return Err(Self::map_phy_error(error)),
            }
        }

        Ok(())
    }

    fn upsert_pan_descriptor(
        descriptors: &mut heapless::Vec<PanDescriptor, { crate::primitives::MAX_PAN_DESCRIPTORS }>,
        descriptor: PanDescriptor,
    ) {
        if let Some(existing) = descriptors.iter_mut().find(|existing| {
            existing.channel == descriptor.channel
                && existing.coord_address == descriptor.coord_address
                && existing.zigbee_beacon.extended_pan_id
                    == descriptor.zigbee_beacon.extended_pan_id
        }) {
            if descriptor.lqi > existing.lqi {
                *existing = descriptor;
            }
        } else {
            let _ = descriptors.push(descriptor);
        }
    }
}

impl<P: RadioPhy + PlatformServices> MacDriver for SoftMacCore<P> {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        self.scan(req).await
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        self.associate(req).await
    }

    async fn mlme_associate_response(
        &mut self,
        _rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError> {
        self.disassociate(req).await
    }

    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        self.reset(set_default_pib)
    }

    async fn mlme_start(&mut self, _req: MlmeStartRequest) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        Ok(self.get_pib(attr))
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        self.set_pib(attr, value)
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        self.poll().await
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        self.transmit_data(req).await
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        self.receive_data(DATA_INDICATION_WAIT_US).await
    }

    fn capabilities(&self) -> MacCapabilities {
        let phy = self.phy.capabilities();
        MacCapabilities {
            coordinator: false,
            router: false,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(phy.tx_power_min),
            tx_power_max: TxPower(phy.tx_power_max),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use std::sync::Arc;
    use std::task::Wake;

    use super::*;
    use crate::{AddressMode, PhyCapabilities, TxOptions};
    use zigbee_types::{ChannelMask, PanId, ShortAddress};

    const IEEE: [u8; 8] = [0x02, 1, 2, 3, 4, 5, 6, 7];

    struct TestPhy {
        channel: u8,
        tx_power: i8,
        filter: Option<PhyAddressFilter>,
        fail_channel: Option<u8>,
        tx_attempts: u8,
        tx_results: heapless::Deque<Result<(), PhyError>, 8>,
        rx_frames: heapless::Deque<Result<Option<PhyRxFrame>, PhyError>, 8>,
        delayed_us: u32,
        last_tx: heapless::Vec<u8, { crate::MAX_PHY_FRAME_LEN }>,
        energy: Result<u8, PhyError>,
        capabilities: PhyCapabilities,
        sent_acks: heapless::Vec<(u8, bool), 8>,
    }

    impl TestPhy {
        fn new() -> Self {
            Self {
                channel: 0,
                tx_power: i8::MIN,
                filter: None,
                fail_channel: None,
                tx_attempts: 0,
                tx_results: heapless::Deque::new(),
                rx_frames: heapless::Deque::new(),
                delayed_us: 0,
                last_tx: heapless::Vec::new(),
                energy: Ok(0),
                capabilities: PhyCapabilities {
                    tx_power_min: -8,
                    tx_power_max: 4,
                    ..Default::default()
                },
                sent_acks: heapless::Vec::new(),
            }
        }
    }

    impl RadioPhy for TestPhy {
        fn capabilities(&self) -> PhyCapabilities {
            self.capabilities
        }

        async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
            self.tx_attempts = self.tx_attempts.saturating_add(1);
            self.last_tx.clear();
            self.last_tx
                .extend_from_slice(frame)
                .map_err(|_| PhyError::FrameTooLong)?;
            self.tx_results.pop_front().unwrap_or(Ok(()))
        }

        async fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), PhyError> {
            self.sent_acks
                .push((sequence, frame_pending))
                .map_err(|_| PhyError::Hardware)
        }

        async fn receive(&mut self, _timeout_us: u32) -> Result<Option<PhyRxFrame>, PhyError> {
            self.rx_frames.pop_front().unwrap_or(Ok(None))
        }

        fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
            if self.fail_channel == Some(channel) {
                return Err(PhyError::Hardware);
            }
            self.channel = channel;
            Ok(())
        }

        fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
            self.tx_power = dbm;
            Ok(())
        }

        async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
            self.energy
        }

        fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
            self.filter = filter;
            Ok(())
        }
    }

    impl PlatformServices for TestPhy {
        fn monotonic_micros(&self) -> u32 {
            123
        }

        async fn delay_micros(&mut self, duration_us: u32) {
            self.delayed_us = self.delayed_us.saturating_add(duration_us);
        }

        fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
            output.fill(3);
            Ok(())
        }
    }

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

    fn core() -> SoftMacCore<TestPhy> {
        SoftMacCore::new(TestPhy::new(), MacPib::new(IEEE, 1, 2)).unwrap()
    }

    fn association_response(
        sequence: u8,
        assigned_address: ShortAddress,
        status: u8,
    ) -> heapless::Vec<u8, 24> {
        let mut frame = heapless::Vec::new();
        frame.extend_from_slice(&0x8C63u16.to_le_bytes()).unwrap();
        frame.push(sequence).unwrap();
        frame.extend_from_slice(&0x1234u16.to_le_bytes()).unwrap();
        frame.extend_from_slice(&IEEE).unwrap();
        frame.extend_from_slice(&0x0000u16.to_le_bytes()).unwrap();
        frame.push(0x02).unwrap();
        frame
            .extend_from_slice(&assigned_address.0.to_le_bytes())
            .unwrap();
        frame.push(status).unwrap();
        frame
    }

    fn associate_request() -> MlmeAssociateRequest {
        MlmeAssociateRequest {
            channel: 15,
            coord_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
            capability_info: crate::CapabilityInfo {
                allocate_address: true,
                ..Default::default()
            },
        }
    }

    fn associated_core() -> SoftMacCore<TestPhy> {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        core.set_pib(PibAttribute::MacAssociatedPanCoord, PibValue::Bool(true))
            .unwrap();
        core
    }

    #[test]
    fn constructor_applies_pib_to_phy() {
        let core = core();
        assert_eq!(core.phy().channel, 11);
        assert_eq!(core.phy().tx_power, 0);
        assert_eq!(
            core.phy().filter,
            Some(PhyAddressFilter {
                pan_id: PanId(0xFFFF),
                short_address: ShortAddress(0xFFFF),
                extended_address: IEEE,
            })
        );
    }

    #[test]
    fn failed_phy_update_does_not_commit_pib_state() {
        let mut core = core();
        core.phy_mut().fail_channel = Some(26);

        assert_eq!(
            core.set_pib(PibAttribute::PhyCurrentChannel, PibValue::U8(26)),
            Err(MacError::RadioError)
        );
        assert_eq!(
            core.get_pib(PibAttribute::PhyCurrentChannel),
            PibValue::U8(11)
        );
    }

    #[test]
    fn destination_filter_accepts_exact_and_broadcast_addresses() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();

        let exact = MacAddress::Short(PanId(0x1234), ShortAddress(0x5678));
        let broadcast = MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF));
        let other = MacAddress::Short(PanId(0x1234), ShortAddress(0x9ABC));

        assert!(core.accepts_destination(&exact));
        assert!(core.is_exact_destination(&exact));
        assert!(core.accepts_destination(&broadcast));
        assert!(!core.is_exact_destination(&broadcast));
        assert!(!core.accepts_destination(&other));
    }

    #[test]
    fn unassociated_address_is_not_an_exact_broadcast_match() {
        let core = core();
        let broadcast = MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF));

        assert!(core.accepts_destination(&broadcast));
        assert!(!core.is_exact_destination(&broadcast));
    }

    #[test]
    fn promiscuous_mode_disables_hardware_and_software_filtering() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPromiscuousMode, PibValue::Bool(true))
            .unwrap();

        assert_eq!(core.phy().filter, None);
        assert!(core.accepts_destination(&MacAddress::Short(PanId(0x1111), ShortAddress(0x2222))));
    }

    #[test]
    fn platform_services_delegate_to_phy() {
        let mut core = core();
        let mut random = [0; 4];
        core.fill_random(&mut random).unwrap();

        assert_eq!(core.monotonic_micros(), 123);
        assert_eq!(random, [3; 4]);
    }

    #[test]
    fn csma_retries_busy_channel_with_pib_backoff_policy() {
        let mut core = core();
        core.phy_mut()
            .tx_results
            .push_back(Err(PhyError::ChannelBusy))
            .unwrap();
        core.phy_mut()
            .tx_results
            .push_back(Err(PhyError::ChannelBusy))
            .unwrap();
        core.phy_mut().tx_results.push_back(Ok(())).unwrap();

        block_on(core.transmit_csma(&[1, 2, 3])).unwrap();

        assert_eq!(core.phy().tx_attempts, 3);
        assert!(core.phy().delayed_us >= UNIT_BACKOFF_PERIOD_US);
    }

    #[test]
    fn acknowledged_transmit_retries_until_matching_ack() {
        let mut core = core();
        core.phy_mut().rx_frames.push_back(Ok(None)).unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x12, 0x00, 0x2A], 255).unwrap(),
            )))
            .unwrap();

        let result = block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])).unwrap();

        assert_eq!(
            result,
            AckResult {
                frame_pending: true
            }
        );
        assert_eq!(core.phy().tx_attempts, 2);
    }

    #[test]
    fn ack_wait_preserves_non_ack_frames() {
        let mut core = core();
        let data = PhyRxFrame::from_slice(&[0x41, 0x88, 0x10, 0xAA], 100).unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(data.clone())))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x2A], 255).unwrap(),
            )))
            .unwrap();

        block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])).unwrap();

        assert_eq!(core.take_pending_rx(), Some(data));
    }

    #[test]
    fn acknowledged_transmit_stops_after_configured_retries() {
        let mut core = core();
        core.set_pib(PibAttribute::MacMaxFrameRetries, PibValue::U8(2))
            .unwrap();

        assert_eq!(
            block_on(core.transmit_acknowledged(&[0x61, 0x88, 0x2A])),
            Err(MacError::NoAck)
        );
        assert_eq!(core.phy().tx_attempts, 3);
    }

    #[test]
    fn active_scan_collects_beacon_and_restores_radio_config() {
        let mut core = core();
        let beacon = [
            0x00, 0x80, 0x55, 0xE9, 0xDF, 0x2D, 0x7D, 0xFF, 0xCF, 0x00, 0x00, 0x00, 0x22, 0x84, 1,
            2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0x09,
        ];
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&beacon, 200).unwrap())))
            .unwrap();
        core.phy_mut().rx_frames.push_back(Ok(None)).unwrap();

        let confirm = block_on(core.scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask(1 << 15),
            scan_duration: 3,
        }))
        .unwrap();

        assert_eq!(confirm.pan_descriptors.len(), 1);
        assert_eq!(confirm.pan_descriptors[0].channel, 15);
        assert_eq!(core.phy().channel, 11);
        assert!(core.phy().filter.is_some());
        assert_eq!(&core.phy().last_tx[..3], &[0x03, 0x08, 1]);
    }

    #[test]
    fn ed_scan_uses_phy_measurement_and_restores_channel() {
        let mut core = core();
        core.phy_mut().energy = Ok(177);

        let confirm = block_on(core.scan(MlmeScanRequest {
            scan_type: ScanType::Ed,
            channel_mask: ChannelMask(1 << 20),
            scan_duration: 2,
        }))
        .unwrap();

        assert_eq!(confirm.energy_list.len(), 1);
        assert_eq!(confirm.energy_list[0].channel, 20);
        assert_eq!(confirm.energy_list[0].energy, 177);
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn scan_rejects_invalid_duration_without_changing_filter() {
        let mut core = core();
        let filter = core.phy().filter;

        assert!(matches!(
            block_on(core.scan(MlmeScanRequest {
                scan_type: ScanType::Active,
                channel_mask: ChannelMask(1 << 15),
                scan_duration: 15,
            })),
            Err(MacError::InvalidParameter)
        ));
        assert_eq!(core.phy().filter, filter);
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn transmit_data_uses_shared_frame_builder_and_ack_engine() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        let confirm = block_on(core.transmit_data(McpsDataRequest {
            src_addr_mode: AddressMode::Short,
            dst_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
            payload: &[0xAA, 0xBB],
            msdu_handle: 7,
            tx_options: TxOptions {
                ack_tx: true,
                ..TxOptions::default()
            },
        }))
        .unwrap();

        assert_eq!(confirm.msdu_handle, 7);
        assert_eq!(
            core.phy().last_tx.as_slice(),
            [
                0x61, 0x88, 0x01, 0x34, 0x12, 0x00, 0x00, 0x78, 0x56, 0xAA, 0xBB
            ]
        );
    }

    #[test]
    fn receive_data_sends_software_ack_for_exact_unicast() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x2A,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xCC],
            true,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 180).unwrap())))
            .unwrap();

        let indication = block_on(core.receive_data(5_000)).unwrap();

        assert_eq!(indication.payload.as_slice(), [0xCC]);
        assert_eq!(indication.lqi, 180);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x2A, false)]);
    }

    #[test]
    fn transmit_data_clears_ack_request_for_broadcast() {
        let mut core = core();

        block_on(core.transmit_data(McpsDataRequest {
            src_addr_mode: AddressMode::Extended,
            dst_address: MacAddress::Short(PanId(0x1234), ShortAddress(0xFFFF)),
            payload: &[0xAA],
            msdu_handle: 8,
            tx_options: TxOptions {
                ack_tx: true,
                ..TxOptions::default()
            },
        }))
        .unwrap();

        assert_eq!(core.phy().tx_attempts, 1);
        assert_eq!(core.phy().last_tx[0] & (1 << 5), 0);
    }

    #[test]
    fn receive_data_never_acks_broadcast() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        let incoming = build_data_frame(
            0x2B,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0xFFFF)),
            &[0xDD],
            true,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 170).unwrap())))
            .unwrap();

        let indication = block_on(core.receive_data(5_000)).unwrap();

        assert_eq!(indication.payload.as_slice(), [0xDD]);
        assert!(core.phy().sent_acks.is_empty());
    }

    #[test]
    fn hardware_auto_ack_suppresses_software_ack() {
        let mut core = core();
        core.phy_mut().capabilities.hardware_auto_ack = true;
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x2C,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xEE],
            true,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 160).unwrap())))
            .unwrap();

        block_on(core.receive_data(5_000)).unwrap();

        assert!(core.phy().sent_acks.is_empty());
    }

    #[test]
    fn association_accepts_a_direct_response_and_commits_pib_state() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&response, 210).unwrap())))
            .unwrap();

        let confirm = block_on(core.associate(associate_request())).unwrap();

        assert_eq!(confirm.status, AssociationStatus::Success);
        assert_eq!(confirm.short_address, ShortAddress(0x5678));
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0x5678))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacCoordShortAddress),
            PibValue::ShortAddress(ShortAddress(0x0000))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacAssociatedPanCoord),
            PibValue::Bool(true)
        );
        assert_eq!(core.phy().channel, 15);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x44, false)]);
    }

    #[test]
    fn association_uses_a_response_retained_during_the_ack_window() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&response, 210).unwrap())))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        let confirm = block_on(core.associate(associate_request())).unwrap();

        assert_eq!(confirm.status, AssociationStatus::Success);
        assert_eq!(confirm.short_address, ShortAddress(0x5678));
        assert_eq!(core.phy().tx_attempts, 1);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x44, false)]);
    }

    #[test]
    fn association_polls_for_an_indirect_response() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        for frame in [
            Some(PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap()),
            None,
            Some(PhyRxFrame::from_slice(&[0x02, 0x00, 0x02], 255).unwrap()),
            Some(PhyRxFrame::from_slice(&response, 210).unwrap()),
        ] {
            core.phy_mut().rx_frames.push_back(Ok(frame)).unwrap();
        }

        let confirm = block_on(core.associate(associate_request())).unwrap();

        assert_eq!(confirm.status, AssociationStatus::Success);
        assert_eq!(confirm.short_address, ShortAddress(0x5678));
        assert_eq!(core.phy().tx_attempts, 2);
    }

    #[test]
    fn association_captures_and_acks_post_response_data() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        let transport_key = build_data_frame(
            0x55,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xA5, 0x5A],
            true,
        )
        .unwrap();
        for frame in [
            Some(PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap()),
            Some(PhyRxFrame::from_slice(&response, 210).unwrap()),
            Some(PhyRxFrame::from_slice(&transport_key, 200).unwrap()),
            None,
        ] {
            core.phy_mut().rx_frames.push_back(Ok(frame)).unwrap();
        }

        block_on(core.associate(associate_request())).unwrap();
        let tx_attempts = core.phy().tx_attempts;
        let captured = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(captured.as_slice(), [0xA5, 0x5A]);
        assert_eq!(core.phy().tx_attempts, tx_attempts);
        assert_eq!(
            core.phy().sent_acks.as_slice(),
            [(0x44, false), (0x55, false)]
        );
    }

    #[test]
    fn post_association_capture_error_is_reported_by_the_next_poll() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0x5678), 0);
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&response, 210).unwrap())))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Err(PhyError::Hardware))
            .unwrap();

        let confirm = block_on(core.associate(associate_request())).unwrap();
        let tx_attempts = core.phy().tx_attempts;

        assert_eq!(confirm.status, AssociationStatus::Success);
        assert!(matches!(block_on(core.poll()), Err(MacError::RadioError)));
        assert_eq!(core.phy().tx_attempts, tx_attempts);
    }

    #[test]
    fn association_denial_does_not_assign_a_short_address() {
        let mut core = core();
        let response = association_response(0x44, ShortAddress(0xFFFE), 1);
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&response, 210).unwrap())))
            .unwrap();

        let confirm = block_on(core.associate(associate_request())).unwrap();

        assert_eq!(confirm.status, AssociationStatus::PanAtCapacity);
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacAssociatedPanCoord),
            PibValue::Bool(false)
        );
    }

    #[test]
    fn association_rejects_a_broadcast_coordinator() {
        let mut core = core();
        let mut request = associate_request();
        request.coord_address = MacAddress::Short(PanId(0x1234), ShortAddress::BROADCAST);

        assert!(matches!(
            block_on(core.associate(request)),
            Err(MacError::InvalidParameter)
        ));
        assert_eq!(core.phy().tx_attempts, 0);
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn disassociation_notifies_the_parent_before_clearing_state() {
        let mut core = associated_core();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        block_on(core.disassociate(MlmeDisassociateRequest {
            device_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
            reason: crate::DisassociateReason::DeviceLeave,
            tx_indirect: false,
        }))
        .unwrap();

        assert_eq!(core.phy().tx_attempts, 1);
        assert_eq!(core.phy().last_tx.last(), Some(&0x02));
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacPanId),
            PibValue::PanId(PanId(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacAssociatedPanCoord),
            PibValue::Bool(false)
        );
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn indirect_disassociation_is_rejected_without_changing_state() {
        let mut core = associated_core();

        assert!(matches!(
            block_on(core.disassociate(MlmeDisassociateRequest {
                device_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
                reason: crate::DisassociateReason::DeviceLeave,
                tx_indirect: true,
            })),
            Err(MacError::Unsupported)
        ));
        assert_eq!(core.phy().tx_attempts, 0);
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0x5678))
        );
    }

    #[test]
    fn failed_disassociation_notification_still_clears_local_state() {
        let mut core = associated_core();

        assert!(matches!(
            block_on(core.disassociate(MlmeDisassociateRequest {
                device_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
                reason: crate::DisassociateReason::DeviceLeave,
                tx_indirect: false,
            })),
            Err(MacError::NoAck)
        ));
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacPanId),
            PibValue::PanId(PanId(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacAssociatedPanCoord),
            PibValue::Bool(false)
        );
    }

    #[test]
    fn default_reset_preserves_eui_and_restores_portable_pib_defaults() {
        let mut core = associated_core();
        core.set_pib(PibAttribute::PhyCurrentChannel, PibValue::U8(20))
            .unwrap();

        core.reset(true).unwrap();

        assert_eq!(
            core.get_pib(PibAttribute::MacExtendedAddress),
            PibValue::ExtendedAddress(IEEE)
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::MacPanId),
            PibValue::PanId(PanId(0xFFFF))
        );
        assert_eq!(
            core.get_pib(PibAttribute::PhyCurrentChannel),
            PibValue::U8(11)
        );
        assert_eq!(core.phy().channel, 11);
    }

    #[test]
    fn non_default_reset_preserves_pib_and_clears_transient_receive_state() {
        let mut core = associated_core();
        core.queue_pending_rx(
            PhyRxFrame::from_slice(&[0x01, 0x00, 0x33], 100).unwrap(),
            false,
        );
        core.pending_error = Some(MacError::RadioError);

        core.reset(false).unwrap();

        assert_eq!(
            core.get_pib(PibAttribute::MacShortAddress),
            PibValue::ShortAddress(ShortAddress(0x5678))
        );
        assert!(core.pending_rx.is_empty());
        assert_eq!(core.pending_error, None);
    }

    #[test]
    fn soft_mac_core_implements_the_end_device_mac_driver_surface() {
        fn assert_mac_driver<T: MacDriver>() {}
        assert_mac_driver::<SoftMacCore<TestPhy>>();

        let mut core = core();
        core.set_pib(PibAttribute::PhyTransmitPower, PibValue::I8(-4))
            .unwrap();
        let capabilities = MacDriver::capabilities(&core);

        assert!(!capabilities.coordinator);
        assert!(!capabilities.router);
        assert!(!capabilities.hardware_security);
        assert_eq!(capabilities.max_payload, 102);
        assert_eq!(capabilities.tx_power_min, TxPower(-8));
        assert_eq!(capabilities.tx_power_max, TxPower(4));
        assert_eq!(
            block_on(MacDriver::mlme_get(&core, PibAttribute::MacExtendedAddress)).unwrap(),
            PibValue::ExtendedAddress(IEEE)
        );
    }

    #[test]
    fn poll_returns_none_when_parent_ack_has_no_pending_data() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacExtendedAddress,
            PibValue::ExtendedAddress([0; 8]),
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        assert!(block_on(core.poll()).unwrap().is_none());
        assert_eq!(core.phy().tx_attempts, 1);
    }

    #[test]
    fn poll_returns_indirect_data_when_frame_pending_is_set() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x33,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xA1, 0xB2],
            true,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x12, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 190).unwrap())))
            .unwrap();

        let frame = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(frame.as_slice(), [0xA1, 0xB2]);
        assert_eq!(core.phy().sent_acks.as_slice(), [(0x33, false)]);
    }

    #[test]
    fn poll_consumes_data_queued_during_the_ack_window() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        let incoming = build_data_frame(
            0x33,
            AddressMode::Short,
            ShortAddress(0x0000),
            &[0; 8],
            &MacAddress::Short(PanId(0x1234), ShortAddress(0x5678)),
            &[0xC3],
            false,
        )
        .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(PhyRxFrame::from_slice(&incoming, 190).unwrap())))
            .unwrap();
        core.phy_mut()
            .rx_frames
            .push_back(Ok(Some(
                PhyRxFrame::from_slice(&[0x02, 0x00, 0x01], 255).unwrap(),
            )))
            .unwrap();

        let frame = block_on(core.poll()).unwrap().unwrap();

        assert_eq!(frame.as_slice(), [0xC3]);
        assert!(core.phy().sent_acks.is_empty());
    }

    #[test]
    fn poll_falls_back_from_short_to_extended_source_addressing() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordShortAddress,
            PibValue::ShortAddress(ShortAddress(0x0000)),
        )
        .unwrap();
        core.set_pib(
            PibAttribute::MacCoordExtendedAddress,
            PibValue::ExtendedAddress([0x10; 8]),
        )
        .unwrap();
        for sequence in [0x01, 0x02] {
            core.phy_mut()
                .rx_frames
                .push_back(Ok(Some(
                    PhyRxFrame::from_slice(&[0x02, 0x00, sequence], 255).unwrap(),
                )))
                .unwrap();
        }

        assert!(block_on(core.poll()).unwrap().is_none());
        assert_eq!(core.phy().tx_attempts, 2);
        assert_eq!(core.phy().last_tx[2], 0x02);
        assert_eq!(&core.phy().last_tx[7..15], &IEEE);
    }

    #[test]
    fn poll_rejects_missing_coordinator_addresses() {
        let mut core = core();

        assert!(matches!(
            block_on(core.poll()),
            Err(MacError::InvalidParameter)
        ));
        assert_eq!(core.phy().tx_attempts, 0);
    }

    #[test]
    fn short_source_data_requires_an_assigned_address() {
        let mut core = core();

        assert!(matches!(
            block_on(core.transmit_data(McpsDataRequest {
                src_addr_mode: AddressMode::Short,
                dst_address: MacAddress::Short(PanId(0x1234), ShortAddress(0x0000)),
                payload: &[0xAA],
                msdu_handle: 9,
                tx_options: TxOptions::default(),
            })),
            Err(MacError::InvalidParameter)
        ));
        assert_eq!(core.phy().tx_attempts, 0);
    }
}
