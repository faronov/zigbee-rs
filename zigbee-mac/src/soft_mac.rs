//! Portable MAC state layered over a hardware radio PHY.

use zigbee_types::MacAddress;

use crate::frames::{build_beacon_request, parse_beacon};
use crate::pib::scan_duration_us;
use crate::{
    EdValue, MacError, MacPib, MlmeScanConfirm, MlmeScanRequest, PanDescriptor, PhyAddressFilter,
    PhyError, PhyRxFrame, PibAttribute, PibError, PibValue, PlatformServices, RadioPhy, ScanType,
};

const UNIT_BACKOFF_PERIOD_US: u32 = 320;
const ACK_WAIT_US: u32 = 1_200;
const MAX_ACK_WINDOW_FRAMES: u8 = 16;
const MAX_SCAN_FRAMES_PER_CHANNEL: u16 = 256;
const PENDING_RX_CAPACITY: usize = 4;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AckResult {
    pub frame_pending: bool,
}

/// Shared software-MAC state for one radio.
///
/// Protocol operations are added incrementally; existing hardware backends
/// continue implementing [`crate::MacDriver`] directly during migration.
pub struct SoftMacCore<P: RadioPhy> {
    phy: P,
    pib: MacPib,
    pending_rx: heapless::Deque<PhyRxFrame, PENDING_RX_CAPACITY>,
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
        self.random_state = Self::random_seed(&self.pib);
        Ok(())
    }

    pub fn take_pending_rx(&mut self) -> Option<PhyRxFrame> {
        self.pending_rx.pop_front()
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
                    self.queue_pending_rx(frame);
                }
                Ok(None) => return Ok(None),
                Err(PhyError::CrcFailed) => {}
                Err(error) => return Err(Self::map_phy_error(error)),
            }
        }

        Ok(None)
    }

    fn queue_pending_rx(&mut self, frame: PhyRxFrame) {
        if self.pending_rx.is_full() {
            let _ = self.pending_rx.pop_front();
        }
        let _ = self.pending_rx.push_back(frame);
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

#[cfg(test)]
mod tests {
    extern crate std;

    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use std::sync::Arc;
    use std::task::Wake;

    use super::*;
    use crate::PhyCapabilities;
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
            }
        }
    }

    impl RadioPhy for TestPhy {
        fn capabilities(&self) -> PhyCapabilities {
            PhyCapabilities::default()
        }

        async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
            self.tx_attempts = self.tx_attempts.saturating_add(1);
            self.last_tx.clear();
            self.last_tx
                .extend_from_slice(frame)
                .map_err(|_| PhyError::FrameTooLong)?;
            self.tx_results.pop_front().unwrap_or(Ok(()))
        }

        async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
            Ok(())
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
}
