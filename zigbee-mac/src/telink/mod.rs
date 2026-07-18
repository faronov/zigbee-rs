//! Pure-Rust TLSR8258 MAC backend.
//!
//! This backend intentionally supports Zigbee end devices only. It wraps the
//! hardware-proven `tlsr8258-hal` radio path; coordinator and router
//! primitives remain disabled until they have independent hardware evidence.

#[cfg(any(target_arch = "tc32", test))]
use crate::primitives::{PanDescriptor, PanDescriptorList};

#[cfg(any(target_arch = "tc32", test))]
fn upsert_pan_descriptor(descriptors: &mut PanDescriptorList, mut descriptor: PanDescriptor) {
    if let Some(existing) = descriptors.iter_mut().find(|existing| {
        existing.channel == descriptor.channel && existing.coord_address == descriptor.coord_address
    }) {
        descriptor.lqi = descriptor.lqi.max(existing.lqi);
        *existing = descriptor;
        return;
    }

    if descriptors.push(descriptor.clone()).is_ok() {
        return;
    }

    if let Some((weakest_index, weakest)) = descriptors
        .iter()
        .enumerate()
        .min_by_key(|(_, existing)| existing.lqi)
        && descriptor.lqi > weakest.lqi
    {
        descriptors[weakest_index] = descriptor;
    }
}

#[cfg(target_arch = "tc32")]
mod imp {
    use crate::frames::{
        self, build_association_request, build_beacon_request, build_data_frame,
        build_data_request, build_data_request_short, parse_association_response,
        parse_mac_addresses,
    };
    use crate::pib::{PibAttribute, PibPayload, PibValue};
    use crate::primitives::*;
    use crate::{MacCapabilities, MacDriver, MacError, PlatformServices, WrappingTickExtender};
    use tlsr8258_hal::radio::{MAX_MAC_FRAME_LEN, Radio, RawRxOutcome, ReceivedFrame, TxOutcome};
    use tlsr8258_hal::{flash, timer};
    use zigbee_types::*;

    const ACK_WAIT_TICKS: u32 = timer::ms(8);
    const ASSOCIATION_DIRECT_WAIT_TICKS: u32 = timer::ms(500);
    const POST_ASSOCIATION_RX_TICKS: u32 = timer::ms(250);
    const POLL_RESPONSE_WAIT_TICKS: u32 = timer::ms(30);
    const RX_INDICATION_WAIT_TICKS: u32 = timer::ms(5_000);
    const MAX_ASSOCIATION_POLLS: u8 = 32;
    const MAX_RECEIVE_FRAMES: u16 = 32;
    const DEFAULT_MAX_FRAME_RETRIES: u8 = 3;

    #[derive(Clone, Copy, Default)]
    struct AckResult {
        frame_pending: bool,
    }

    #[derive(Clone, Copy)]
    struct AddressFilter {
        pan_id: PanId,
        short_address: ShortAddress,
        extended_address: IeeeAddress,
        promiscuous: bool,
    }

    /// TLSR8258 end-device MAC.
    ///
    /// The application must initialize the TLSR8258 clocks before creating
    /// this value. `new()` reads the factory IEEE address (or the stable
    /// flash-UID fallback) and exclusively acquires the RF block.
    pub struct TelinkMac {
        radio: Radio,
        short_address: ShortAddress,
        pan_id: PanId,
        phy_channel: u8,
        extended_address: IeeeAddress,
        coord_short_address: ShortAddress,
        coord_extended_address: IeeeAddress,
        associated_pan_coord: bool,
        rx_on_when_idle: bool,
        association_permit: bool,
        auto_request: bool,
        beacon_order: u8,
        superframe_order: u8,
        response_wait_time: u8,
        transaction_persistence_time: u16,
        max_frame_total_wait_time: u32,
        dsn: u8,
        bsn: u8,
        beacon_payload: PibPayload,
        max_csma_backoffs: u8,
        min_be: u8,
        max_be: u8,
        max_frame_retries: u8,
        promiscuous: bool,
        tx_power: i8,
        pending_association_response: Option<(ShortAddress, u8)>,
        pending_rx: Option<ReceivedFrame>,
        clock: WrappingTickExtender,
    }

    impl TelinkMac {
        pub fn new() -> Self {
            let radio = Radio::take().expect("TLSR8258 radio already taken");
            Self::from_radio(radio, None)
        }

        pub fn with_extended_address(extended_address: IeeeAddress) -> Self {
            let radio = Radio::take().expect("TLSR8258 radio already taken");
            Self::from_radio(radio, Some(extended_address))
        }

        fn from_radio(mut radio: Radio, address: Option<IeeeAddress>) -> Self {
            radio.init();
            let extended_address = match address {
                Some(address) => address,
                None => {
                    let mut address = [0u8; 8];
                    flash::factory_ieee(&mut address);
                    address
                }
            };
            let now_ticks = timer::now_ticks();
            let dsn = now_ticks as u8;
            let mut mac = Self {
                radio,
                short_address: ShortAddress(0xFFFF),
                pan_id: PanId(0xFFFF),
                phy_channel: 11,
                extended_address,
                coord_short_address: ShortAddress(0xFFFF),
                coord_extended_address: [0; 8],
                associated_pan_coord: false,
                rx_on_when_idle: false,
                association_permit: false,
                auto_request: true,
                beacon_order: 15,
                superframe_order: 15,
                response_wait_time: 32,
                transaction_persistence_time: 0x01F4,
                max_frame_total_wait_time: 0,
                dsn,
                bsn: 0,
                beacon_payload: PibPayload::new(),
                max_csma_backoffs: 4,
                min_be: 3,
                max_be: 5,
                max_frame_retries: DEFAULT_MAX_FRAME_RETRIES,
                promiscuous: false,
                tx_power: 0,
                pending_association_response: None,
                pending_rx: None,
                clock: WrappingTickExtender::new(now_ticks),
            };
            mac.apply_radio_config();
            mac
        }

        fn next_dsn(&mut self) -> u8 {
            let sequence = self.dsn;
            self.dsn = self.dsn.wrapping_add(1);
            sequence
        }

        fn extended_timer_ticks(&self) -> u64 {
            self.clock.extend(timer::now_ticks())
        }

        fn apply_radio_config(&mut self) {
            self.radio.set_channel(self.channel());
            self.radio
                .set_ack_filter(self.pan_id.0, self.short_address.0, self.extended_address);
        }

        fn channel(&self) -> u8 {
            self.phy_channel
        }

        fn set_channel(&mut self, channel: u8) -> Result<(), MacError> {
            if !(11..=26).contains(&channel) {
                return Err(MacError::InvalidParameter);
            }
            self.phy_channel = channel;
            self.radio.set_channel(channel);
            Ok(())
        }

        fn remember_non_ack(&mut self, candidate: Option<ReceivedFrame>) {
            if candidate.is_some() {
                self.pending_rx = candidate;
            }
        }

        fn address_filter(&self) -> AddressFilter {
            AddressFilter {
                pan_id: self.pan_id,
                short_address: self.short_address,
                extended_address: self.extended_address,
                promiscuous: self.promiscuous,
            }
        }

        fn transmit_with_ack(
            &mut self,
            frame: &[u8],
            sequence: u8,
            ack_requested: bool,
        ) -> Result<AckResult, MacError> {
            let attempts = if ack_requested {
                self.max_frame_retries.saturating_add(1)
            } else {
                1
            };
            let mut last_error = MacError::NoAck;

            for _ in 0..attempts {
                match self.radio.transmit(frame) {
                    TxOutcome::Sent => {}
                    TxOutcome::InvalidFrame => return Err(MacError::FrameTooLong),
                    TxOutcome::ChannelAccessFailure => {
                        last_error = MacError::ChannelAccessFailure;
                        continue;
                    }
                    TxOutcome::Timeout => {
                        last_error = MacError::RadioError;
                        continue;
                    }
                }

                if !ack_requested {
                    return Ok(AckResult::default());
                }

                let mut ack = None;
                let mut association_response = None;
                let mut pending = None;
                let mut pending_rank = 0;
                let filter = self.address_filter();
                self.radio
                    .receive_raw_until(ACK_WAIT_TICKS, MAX_RECEIVE_FRAMES, |outcome| {
                        let RawRxOutcome::Frame(received) = outcome else {
                            return false;
                        };
                        let data = received.as_slice();
                        if data.len() >= 3 {
                            let frame_control = u16::from_le_bytes([data[0], data[1]]);
                            if frame_control & 0x07 == 0x02 && data[2] == sequence {
                                ack = Some(AckResult {
                                    frame_pending: frame_control & (1 << 4) != 0,
                                });
                                return true;
                            }
                            let (_, destination, _, _) = parse_mac_addresses(data);
                            if let Some(response) = parse_association_response(data)
                                && Self::accepts_destination(filter, &destination)
                            {
                                association_response = Some(response);
                            } else {
                                let mut candidate_filter = filter;
                                if let Some((short_address, 0)) = association_response {
                                    candidate_filter.short_address = short_address;
                                }
                                if let Some(indication) =
                                    Self::parse_data_indication_for(&received, candidate_filter)
                                {
                                    let rank = if Self::is_exact_destination(
                                        candidate_filter,
                                        &indication.dst_address,
                                    ) {
                                        2
                                    } else {
                                        1
                                    };
                                    if rank > pending_rank {
                                        pending = Some(received);
                                        pending_rank = rank;
                                    }
                                }
                            }
                        }
                        false
                    });
                if association_response.is_some() {
                    self.pending_association_response = association_response;
                }
                self.remember_non_ack(pending);
                if let Some(ack) = ack {
                    return Ok(ack);
                }
                last_error = MacError::NoAck;
            }

            Err(last_error)
        }

        fn take_association_response(&mut self) -> Option<(ShortAddress, u8)> {
            self.pending_association_response.take()
        }

        fn wait_for_association_response(
            &mut self,
            timeout_ticks: u32,
        ) -> Option<(ShortAddress, u8)> {
            if let Some(response) = self.take_association_response() {
                return Some(response);
            }

            let mut response = None;
            let filter = self.address_filter();
            let mut pending = self.pending_rx.take();
            let mut pending_rank = pending
                .as_ref()
                .and_then(|received| Self::parse_data_indication_for(received, filter))
                .map(|indication| {
                    if Self::is_exact_destination(filter, &indication.dst_address) {
                        2
                    } else {
                        1
                    }
                })
                .unwrap_or(0);
            self.radio
                .receive_raw_until(timeout_ticks, MAX_RECEIVE_FRAMES, |outcome| {
                    let RawRxOutcome::Frame(received) = outcome else {
                        return false;
                    };
                    let data = received.as_slice();
                    let (_, destination, _, _) = parse_mac_addresses(data);
                    if Self::accepts_destination(filter, &destination)
                        && let Some(candidate) = parse_association_response(data)
                    {
                        response = Some(candidate);
                        return true;
                    }

                    if let Some(indication) = Self::parse_data_indication_for(&received, filter) {
                        let rank = if Self::is_exact_destination(filter, &indication.dst_address) {
                            2
                        } else {
                            1
                        };
                        if rank > pending_rank {
                            pending = Some(received);
                            pending_rank = rank;
                        }
                    }
                    false
                });
            self.remember_non_ack(pending);
            if let Some((short_address, 0)) = response {
                self.radio
                    .set_ack_filter(self.pan_id.0, short_address.0, self.extended_address);
            }
            response
        }

        fn capture_post_association_frame(&mut self) {
            let filter = self.address_filter();
            let mut pending = self.pending_rx.take();
            let mut pending_rank = pending
                .as_ref()
                .and_then(|received| Self::parse_data_indication_for(received, filter))
                .map(|indication| {
                    if Self::is_exact_destination(filter, &indication.dst_address) {
                        2
                    } else {
                        1
                    }
                })
                .unwrap_or(0);
            if pending_rank < 2 {
                self.radio.receive_raw_until(
                    POST_ASSOCIATION_RX_TICKS,
                    MAX_RECEIVE_FRAMES,
                    |outcome| {
                        let RawRxOutcome::Frame(received) = outcome else {
                            return false;
                        };
                        if let Some(indication) = Self::parse_data_indication_for(&received, filter)
                        {
                            let rank =
                                if Self::is_exact_destination(filter, &indication.dst_address) {
                                    2
                                } else {
                                    1
                                };
                            if rank > pending_rank {
                                pending = Some(received);
                                pending_rank = rank;
                            }
                            return rank == 2;
                        }
                        false
                    },
                );
            }
            self.remember_non_ack(pending);
        }

        fn finish_association(
            &mut self,
            short_address: ShortAddress,
            status: u8,
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
                self.short_address = short_address;
                self.associated_pan_coord = true;
                self.apply_radio_config();
                self.capture_post_association_frame();
            }
            Ok(MlmeAssociateConfirm {
                short_address,
                status,
            })
        }

        fn scan_channel(
            &mut self,
            channel: u8,
            duration_ticks: u64,
            active: bool,
            descriptors: &mut PanDescriptorList,
        ) {
            self.radio.set_channel(channel);
            if active {
                let request = build_beacon_request(self.next_dsn());
                if self.radio.transmit(&request) != TxOutcome::Sent {
                    return;
                }
            }

            let mut remaining = duration_ticks;
            while remaining != 0 {
                let chunk = remaining.min(u32::MAX as u64) as u32;
                let elapsed = self.radio.receive_raw_for(chunk, u16::MAX, |outcome| {
                    let RawRxOutcome::Frame(received) = outcome else {
                        return;
                    };
                    if let Some(descriptor) =
                        frames::parse_beacon(channel, received.as_slice(), received.lqi)
                    {
                        super::upsert_pan_descriptor(descriptors, descriptor);
                    }
                });
                remaining = remaining.saturating_sub(elapsed.max(1) as u64);
            }
        }

        fn receive_data_indication(&mut self, timeout_ticks: u32) -> Option<McpsDataIndication> {
            let filter = self.address_filter();
            let mut indication = None;
            let mut indication_rank = 0;
            if let Some(received) = self.pending_rx.take() {
                if let Some(candidate) = Self::parse_data_indication_for(&received, filter) {
                    if Self::is_exact_destination(filter, &candidate.dst_address) {
                        return Some(candidate);
                    }
                    indication_rank = 1;
                    indication = Some(candidate);
                }
            }

            self.radio
                .receive_raw_until(timeout_ticks, MAX_RECEIVE_FRAMES, |outcome| {
                    let RawRxOutcome::Frame(received) = outcome else {
                        return false;
                    };
                    if let Some(candidate) = Self::parse_data_indication_for(&received, filter) {
                        let rank = if Self::is_exact_destination(filter, &candidate.dst_address) {
                            2
                        } else {
                            1
                        };
                        if rank > indication_rank {
                            indication = Some(candidate);
                            indication_rank = rank;
                        }
                        return rank == 2;
                    }
                    false
                });
            indication
        }

        fn receive_poll_response(&mut self, timeout_ticks: u32) -> Option<McpsDataIndication> {
            let filter = self.address_filter();
            if let Some(received) = self.pending_rx.take()
                && let Some(indication) = Self::parse_data_indication_for(&received, filter)
                && Self::is_exact_destination(filter, &indication.dst_address)
            {
                return Some(indication);
            }

            let mut indication = None;
            self.radio
                .receive_raw_until(timeout_ticks, MAX_RECEIVE_FRAMES, |outcome| {
                    let RawRxOutcome::Frame(received) = outcome else {
                        return false;
                    };
                    if let Some(candidate) = Self::parse_data_indication_for(&received, filter)
                        && Self::is_exact_destination(filter, &candidate.dst_address)
                    {
                        indication = Some(candidate);
                        return true;
                    }
                    false
                });
            indication
        }

        fn parse_data_indication_for(
            received: &ReceivedFrame,
            filter: AddressFilter,
        ) -> Option<McpsDataIndication> {
            let data = received.as_slice();
            if data.len() < 3 {
                return None;
            }
            let frame_control = u16::from_le_bytes([data[0], data[1]]);
            if frame_control & 0x07 != 0x01 {
                return None;
            }
            let (source, destination, payload_offset, security_use) = parse_mac_addresses(data);
            if payload_offset > data.len() || !Self::accepts_destination(filter, &destination) {
                return None;
            }
            Some(McpsDataIndication {
                src_address: source,
                dst_address: destination,
                lqi: received.lqi,
                payload: MacFrame::from_slice(&data[payload_offset..])?,
                security_use,
            })
        }

        fn accepts_destination(filter: AddressFilter, destination: &MacAddress) -> bool {
            if filter.promiscuous {
                return true;
            }
            match destination {
                MacAddress::Short(pan, address) => {
                    (pan.0 == filter.pan_id.0 || pan.0 == 0xFFFF)
                        && (address.0 == filter.short_address.0 || address.0 == 0xFFFF)
                }
                MacAddress::Extended(pan, address) => {
                    (pan.0 == filter.pan_id.0 || pan.0 == 0xFFFF)
                        && *address == filter.extended_address
                }
            }
        }

        fn is_exact_destination(filter: AddressFilter, destination: &MacAddress) -> bool {
            match destination {
                MacAddress::Short(pan, address) => {
                    pan.0 == filter.pan_id.0 && address.0 == filter.short_address.0
                }
                MacAddress::Extended(pan, address) => {
                    pan.0 == filter.pan_id.0 && *address == filter.extended_address
                }
            }
        }

        fn clear_association(&mut self) {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.coord_short_address = ShortAddress(0xFFFF);
            self.coord_extended_address = [0; 8];
            self.associated_pan_coord = false;
            self.pending_association_response = None;
            self.pending_rx = None;
            self.apply_radio_config();
        }
    }

    impl MacDriver for TelinkMac {
        async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
            if req.scan_duration > 14 {
                return Err(MacError::InvalidParameter);
            }
            let saved_channel = self.channel();
            let duration_ticks = crate::pib::scan_duration_us(req.scan_duration)
                .saturating_mul((timer::TICKS_PER_MS / 1_000) as u64);
            let mut pan_descriptors = PanDescriptorList::new();
            let mut energy_list = EdList::new();

            for channel in req.channel_mask.iter() {
                let number = channel.number();
                match req.scan_type {
                    ScanType::Active => {
                        self.scan_channel(number, duration_ticks, true, &mut pan_descriptors);
                    }
                    ScanType::Passive => {
                        self.scan_channel(number, duration_ticks, false, &mut pan_descriptors);
                    }
                    ScanType::Ed => {
                        self.radio.set_channel(number);
                        let _ = energy_list.push(EdValue {
                            channel: number,
                            energy: self.radio.measure_energy(),
                        });
                    }
                    ScanType::Orphan => {
                        self.radio.set_channel(saved_channel);
                        return Err(MacError::Unsupported);
                    }
                }
            }
            self.radio.set_channel(saved_channel);

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

        async fn mlme_associate(
            &mut self,
            req: MlmeAssociateRequest,
        ) -> Result<MlmeAssociateConfirm, MacError> {
            let MacAddress::Short(pan_id, coordinator) = req.coord_address else {
                return Err(MacError::Unsupported);
            };
            self.set_channel(req.channel)?;
            self.pan_id = pan_id;
            self.coord_short_address = coordinator;
            self.short_address = ShortAddress(0xFFFF);
            self.pending_association_response = None;
            self.pending_rx = None;
            self.apply_radio_config();

            let coordinator_address = MacAddress::Short(pan_id, coordinator);
            let sequence = self.next_dsn();
            let request = build_association_request(
                sequence,
                &coordinator_address,
                &self.extended_address,
                &req.capability_info,
            );
            let tx = self.transmit_with_ack(&request, sequence, true);
            if let Some((short_address, status)) = self.take_association_response() {
                return self.finish_association(short_address, status);
            }
            tx?;
            if let Some((short_address, status)) =
                self.wait_for_association_response(ASSOCIATION_DIRECT_WAIT_TICKS)
            {
                return self.finish_association(short_address, status);
            }

            for _ in 0..MAX_ASSOCIATION_POLLS {
                let sequence = self.next_dsn();
                let poll =
                    build_data_request(sequence, &coordinator_address, &self.extended_address);
                let tx = self.transmit_with_ack(&poll, sequence, true);
                if let Some((short_address, status)) = self.take_association_response() {
                    return self.finish_association(short_address, status);
                }
                let _ack = tx?;
                // Some parents transmit the pending Association Response
                // immediately after the Data Request ACK even when the ACK's
                // frame-pending bit is already clear. Keep RX open for the
                // response on every successful poll.
                if let Some((short_address, status)) =
                    self.wait_for_association_response(POLL_RESPONSE_WAIT_TICKS)
                {
                    return self.finish_association(short_address, status);
                }
                timer::sleep_ticks(timer::ms(100));
            }
            Err(MacError::NoData)
        }

        async fn mlme_associate_response(
            &mut self,
            _rsp: MlmeAssociateResponse,
        ) -> Result<(), MacError> {
            Err(MacError::Unsupported)
        }

        async fn mlme_disassociate(
            &mut self,
            _req: MlmeDisassociateRequest,
        ) -> Result<(), MacError> {
            self.clear_association();
            Ok(())
        }

        async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
            if set_default_pib {
                self.clear_association();
                self.phy_channel = 11;
                self.rx_on_when_idle = false;
                self.association_permit = false;
                self.auto_request = true;
                self.beacon_order = 15;
                self.superframe_order = 15;
                self.response_wait_time = 32;
                self.transaction_persistence_time = 0x01F4;
                self.max_frame_total_wait_time = 0;
                self.dsn = timer::now_ticks() as u8;
                self.bsn = 0;
                self.beacon_payload = PibPayload::new();
                self.max_csma_backoffs = 4;
                self.min_be = 3;
                self.max_be = 5;
                self.max_frame_retries = DEFAULT_MAX_FRAME_RETRIES;
                self.promiscuous = false;
                self.tx_power = 0;
            }
            self.apply_radio_config();
            Ok(())
        }

        async fn mlme_start(&mut self, _req: MlmeStartRequest) -> Result<(), MacError> {
            Err(MacError::Unsupported)
        }

        async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
            use PibAttribute::*;
            match attr {
                MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
                MacPanId => Ok(PibValue::PanId(self.pan_id)),
                MacExtendedAddress => Ok(PibValue::ExtendedAddress(self.extended_address)),
                MacCoordShortAddress => Ok(PibValue::ShortAddress(self.coord_short_address)),
                MacCoordExtendedAddress => {
                    Ok(PibValue::ExtendedAddress(self.coord_extended_address))
                }
                MacAssociatedPanCoord => Ok(PibValue::Bool(self.associated_pan_coord)),
                MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
                MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
                MacBeaconOrder => Ok(PibValue::U8(self.beacon_order)),
                MacSuperframeOrder => Ok(PibValue::U8(self.superframe_order)),
                MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
                MacBeaconPayloadLength => {
                    Ok(PibValue::U8(self.beacon_payload.as_slice().len() as u8))
                }
                MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
                MacMaxCsmaBackoffs => Ok(PibValue::U8(self.max_csma_backoffs)),
                MacMinBe => Ok(PibValue::U8(self.min_be)),
                MacMaxBe => Ok(PibValue::U8(self.max_be)),
                MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
                MacMaxFrameTotalWaitTime => Ok(PibValue::U32(self.max_frame_total_wait_time)),
                MacResponseWaitTime => Ok(PibValue::U8(self.response_wait_time)),
                MacDsn => Ok(PibValue::U8(self.dsn)),
                MacBsn => Ok(PibValue::U8(self.bsn)),
                MacTransactionPersistenceTime => {
                    Ok(PibValue::U16(self.transaction_persistence_time))
                }
                MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
                PhyCurrentChannel => Ok(PibValue::U8(self.channel())),
                PhyChannelsSupported => Ok(PibValue::U32(ChannelMask::ALL_2_4GHZ.0)),
                PhyTransmitPower => Ok(PibValue::I8(self.tx_power)),
                PhyCcaMode => Ok(PibValue::U8(1)),
                PhyCurrentPage => Ok(PibValue::U8(0)),
            }
        }

        async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
            use PibAttribute::*;
            match (attr, value) {
                (MacShortAddress, PibValue::ShortAddress(value)) => {
                    self.short_address = value;
                    self.apply_radio_config();
                }
                (MacPanId, PibValue::PanId(value)) => {
                    self.pan_id = value;
                    self.apply_radio_config();
                }
                (MacExtendedAddress, PibValue::ExtendedAddress(value)) => {
                    self.extended_address = value;
                    self.apply_radio_config();
                }
                (MacCoordShortAddress, PibValue::ShortAddress(value)) => {
                    self.coord_short_address = value;
                }
                (MacCoordExtendedAddress, PibValue::ExtendedAddress(value)) => {
                    self.coord_extended_address = value;
                }
                (MacAssociatedPanCoord, PibValue::Bool(value)) => {
                    self.associated_pan_coord = value;
                }
                (MacRxOnWhenIdle, PibValue::Bool(value)) => self.rx_on_when_idle = value,
                (MacAssociationPermit, PibValue::Bool(value)) => {
                    self.association_permit = value;
                }
                (MacBeaconOrder, PibValue::U8(value)) => self.beacon_order = value,
                (MacSuperframeOrder, PibValue::U8(value)) => {
                    self.superframe_order = value;
                }
                (MacBeaconPayload, PibValue::Payload(value)) => {
                    self.beacon_payload = value;
                }
                (MacBeaconPayloadLength, PibValue::U8(value))
                    if value as usize == self.beacon_payload.as_slice().len() => {}
                (MacAutoRequest, PibValue::Bool(value)) => self.auto_request = value,
                (MacMaxCsmaBackoffs, PibValue::U8(value)) => {
                    self.max_csma_backoffs = value;
                }
                (MacMinBe, PibValue::U8(value)) if value <= 8 => self.min_be = value,
                (MacMaxBe, PibValue::U8(value)) if value <= 8 => self.max_be = value,
                (MacMaxFrameRetries, PibValue::U8(value)) => {
                    self.max_frame_retries = value;
                }
                (MacMaxFrameTotalWaitTime, PibValue::U32(value)) => {
                    self.max_frame_total_wait_time = value;
                }
                (MacResponseWaitTime, PibValue::U8(value)) => {
                    self.response_wait_time = value;
                }
                (MacDsn, PibValue::U8(value)) => self.dsn = value,
                (MacBsn, PibValue::U8(value)) => self.bsn = value,
                (MacTransactionPersistenceTime, PibValue::U16(value)) => {
                    self.transaction_persistence_time = value;
                }
                (MacPromiscuousMode, PibValue::Bool(value)) => {
                    self.promiscuous = value;
                }
                (PhyCurrentChannel, PibValue::U8(value)) => self.set_channel(value)?,
                (PhyTransmitPower, PibValue::I8(0)) => self.tx_power = 0,
                (PhyCcaMode, PibValue::U8(1)) | (PhyCurrentPage, PibValue::U8(0)) => {}
                _ => return Err(MacError::InvalidParameter),
            }
            Ok(())
        }

        async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
            self.mlme_poll_timeout(
                (POLL_RESPONSE_WAIT_TICKS / u32::from(timer::TICKS_PER_US)).max(1),
            )
            .await
        }

        async fn mlme_poll_timeout(
            &mut self,
            timeout_us: u32,
        ) -> Result<Option<MacFrame>, MacError> {
            if self.coord_short_address.0 == 0xFFFF {
                return Err(MacError::InvalidParameter);
            }
            let started = self.monotonic_micros();
            let coordinator = MacAddress::Short(self.pan_id, self.coord_short_address);
            let sequence = self.next_dsn();
            let request = build_data_request_short(sequence, &coordinator, self.short_address);
            let ack = self.transmit_with_ack(&request, sequence, true)?;

            if let Some(indication) = self.receive_poll_response(1) {
                return Ok(Some(indication.payload));
            }
            if !ack.frame_pending {
                return Ok(None);
            }
            let elapsed = self.monotonic_micros().wrapping_sub(started);
            let Some(remaining_us) = timeout_us.checked_sub(elapsed) else {
                return Ok(None);
            };
            Ok(self
                .receive_poll_response(timer::us(remaining_us))
                .map(|indication| indication.payload))
        }

        async fn mcps_data(
            &mut self,
            req: McpsDataRequest<'_>,
        ) -> Result<McpsDataConfirm, MacError> {
            if req.tx_options.indirect {
                return Err(MacError::Unsupported);
            }
            if req.tx_options.security_enabled {
                return Err(MacError::SecurityError);
            }
            if req.src_addr_mode == AddressMode::Short && self.short_address.0 >= 0xFFF8 {
                return Err(MacError::InvalidParameter);
            }
            let sequence = self.next_dsn();
            let frame = build_data_frame(
                sequence,
                req.src_addr_mode,
                self.short_address,
                &self.extended_address,
                &req.dst_address,
                req.payload,
                req.tx_options.ack_tx,
            )
            .map_err(|_| MacError::FrameTooLong)?;
            self.transmit_with_ack(&frame, sequence, req.tx_options.ack_tx)?;
            Ok(McpsDataConfirm {
                msdu_handle: req.msdu_handle,
                timestamp: Some(timer::now_ticks()),
            })
        }

        async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
            self.receive_data_indication(RX_INDICATION_WAIT_TICKS)
                .ok_or(MacError::NoData)
        }

        async fn mcps_data_indication_timeout(
            &mut self,
            timeout_us: u32,
        ) -> Result<McpsDataIndication, MacError> {
            self.receive_data_indication(timer::us(timeout_us))
                .ok_or(MacError::NoData)
        }

        fn capabilities(&self) -> MacCapabilities {
            MacCapabilities {
                coordinator: false,
                router: false,
                hardware_security: false,
                max_payload: (MAX_MAC_FRAME_LEN - 23) as u16,
                tx_power_min: TxPower(0),
                tx_power_max: TxPower(0),
            }
        }
    }

    impl PlatformServices for TelinkMac {
        fn monotonic_micros(&self) -> u32 {
            (self.extended_timer_ticks() / u64::from(timer::TICKS_PER_US)) as u32
        }

        async fn delay_micros(&mut self, duration_us: u32) {
            timer::sleep_ticks(timer::us(duration_us));
        }

        fn fill_random(&mut self, _output: &mut [u8]) -> Result<(), MacError> {
            Err(MacError::Unsupported)
        }
    }
}

#[cfg(target_arch = "tc32")]
pub use imp::TelinkMac;

#[cfg(not(target_arch = "tc32"))]
pub struct TelinkMac;

#[cfg(not(target_arch = "tc32"))]
impl TelinkMac {
    pub const fn new() -> Self {
        Self
    }

    pub const fn with_extended_address(_extended_address: [u8; 8]) -> Self {
        Self
    }
}

#[cfg(not(target_arch = "tc32"))]
impl Default for TelinkMac {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{MAX_PAN_DESCRIPTORS, SuperframeSpec, ZigbeeBeaconPayload};
    use zigbee_types::{MacAddress, PanId, ShortAddress};

    fn descriptor(address: u16, lqi: u8, permit_joining: bool) -> PanDescriptor {
        PanDescriptor {
            channel: 15,
            coord_address: MacAddress::Short(PanId(0xDFE9), ShortAddress(address)),
            superframe_spec: SuperframeSpec {
                association_permit: permit_joining,
                ..SuperframeSpec::default()
            },
            lqi,
            security_use: false,
            zigbee_beacon: ZigbeeBeaconPayload {
                protocol_id: 0,
                stack_profile: 2,
                protocol_version: 2,
                router_capacity: true,
                device_depth: 1,
                end_device_capacity: true,
                extended_pan_id: [1; 8],
                tx_offset: [0; 3],
                update_id: 1,
            },
        }
    }

    #[test]
    fn repeated_beacons_do_not_consume_descriptor_slots() {
        let mut descriptors = PanDescriptorList::new();

        upsert_pan_descriptor(&mut descriptors, descriptor(0x1234, 80, false));
        upsert_pan_descriptor(&mut descriptors, descriptor(0x1234, 60, true));

        assert_eq!(descriptors.len(), 1);
        assert_eq!(descriptors[0].lqi, 80);
        assert!(descriptors[0].superframe_spec.association_permit);
    }

    #[test]
    fn full_scan_retains_strongest_unique_parents() {
        let mut descriptors = PanDescriptorList::new();
        for address in 0..MAX_PAN_DESCRIPTORS as u16 {
            upsert_pan_descriptor(&mut descriptors, descriptor(address, address as u8, true));
        }

        upsert_pan_descriptor(&mut descriptors, descriptor(0xCAFE, 200, true));

        assert_eq!(descriptors.len(), MAX_PAN_DESCRIPTORS);
        assert!(descriptors.iter().any(|entry| {
            entry.coord_address == MacAddress::Short(PanId(0xDFE9), ShortAddress(0xCAFE))
        }));
        assert!(!descriptors.iter().any(|entry| {
            entry.coord_address == MacAddress::Short(PanId(0xDFE9), ShortAddress(0))
        }));
    }
}
