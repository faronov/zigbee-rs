//! TI CC26x2/CC13x2 [`RadioPhy`] for [`SoftMacCore`].
//!
//! The radio is `cc26x2_hal::rf`: the system CPU drives TI's RF core
//! firmware through the doorbell; no TI RF driver, RTOS or MAC library is
//! linked. This module adapts it to the shared software MAC:
//!
//! - **TX** — [`FullTx`]: the RF core runs the whole unslotted CSMA-CA
//!   (`CMD_IEEE_CSMA` with the PIB's macMinBE/macMaxBE/macMaxCSMABackoffs),
//!   the transmission and the Imm-Ack wait (`CMD_IEEE_RX_ACK`); this PHY
//!   repeats the chain for macMaxFrameRetries.
//! - **ACK** — the RF core acknowledges every frame that passes its address
//!   filter (`hardware_auto_ack`), independent of the stack's state. The
//!   Frame Pending bit of an ACK to a Data Request comes from the RF core's
//!   source-match tables, which a parent's `SoftMacCore` arms through
//!   [`RadioPhy::set_ack_frame_pending`] (single Frame Pending owner, so the
//!   router role has no software pending table).
//! - **RX** — frames wait in the RF core's data queue
//!   (`cc26x2_hal::rf::RX_ENTRIES` entries) until [`RadioPhy::receive`]
//!   polls it from the executor.
//! - **AES** — AES-128 for NWK/APS CCM* runs on the CRYPTO engine
//!   (`cc26x2_hal::crypto`), installed by the composition root with
//!   [`Cc26x2RadioPhy::install_aes_engine`] after two FIPS-197 known-answer
//!   tests. There is no software fallback: without an installed engine every
//!   secured frame fails closed with [`Cc26x2AesError::NotInstalled`].
//!
//! Validation: see `docs/book/src/platform-guides/cc26x2.md`.

use cc26x2_hal::crypto::{AesEngine, AesError, CryptoToken, Mmio};
use cc26x2_hal::rf::{self, Csma, Filter, FpAddress, FpError, Radio, RadioError, TxOutcome};
use cc26x2_hal::trng::Trng;
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Timer};

use crate::pib::{MacPib, PibAttribute, PibValue};
use crate::{
    AckResult, FullTx, MAX_PHY_FRAME_LEN, MacError, MacRole, Parent, PhyAddressFilter,
    PhyCapabilities, PhyError, PhyRxFrame, PlatformServices, RadioPhy, SoftMacCore, TxParams,
};
use zigbee_types::{IeeeAddress, MacAddress};

const _: () = assert!(rf::MAX_FRAME == MAX_PHY_FRAME_LEN);

/// Bound of one CSMA + TX + ACK chain (the worst case at macMaxBE 5 and
/// five backoffs is about 25 ms; a stuck RF core is aborted after this).
const TX_CHAIN_TIMEOUT_MS: u64 = 100;

/// CC26x2/CC13x2 radio adapted to [`RadioPhy`].
///
/// `DEFAULT_TX_DBM` is the product's `phyTransmitPower` after construction
/// and every MLME-RESET; it must lie in the chip's PA range.
pub struct Cc26x2RadioPhy<const DEFAULT_TX_DBM: i8 = 0> {
    radio: Radio,
    trng: Trng,
    ieee: IeeeAddress,
    rx_on_when_idle: bool,
    pan_coordinator: bool,
    filter: Option<PhyAddressFilter>,
    last_error: Option<RadioError>,
    aes: Option<AesEngine<Mmio>>,
}

/// Failure of the CRYPTO-engine AES backend.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cc26x2AesError {
    /// The composition root did not install a validated engine.
    NotInstalled,
    /// An engine is already installed.
    AlreadyInstalled,
    /// Known-answer test `n` produced wrong ciphertext.
    KnownAnswerTestFailed(u8),
    /// The engine reported a failure.
    Engine(AesError),
}

/// End-device MAC over [`Cc26x2RadioPhy`].
pub type Cc26x2SoftMac<const DEFAULT_TX_DBM: i8 = 0> = SoftMacCore<Cc26x2RadioPhy<DEFAULT_TX_DBM>>;

/// Parent role: eight queued MAC command events, eight concurrent
/// Association Response transactions, no software Frame Pending table (the
/// RF core source-match tables own it).
pub type Cc26x2RouterRole = Parent<8, 8, 0>;

/// Retained-RX depth of an always-on router.
pub const CC26X2_ROUTER_PENDING_RX: usize = 8;

/// Router MAC: the shared software MAC in the parent role.
pub type Cc26x2RouterMac<const DEFAULT_TX_DBM: i8 = 0> =
    SoftMacCore<Cc26x2RadioPhy<DEFAULT_TX_DBM>, Cc26x2RouterRole, CC26X2_ROUTER_PENDING_RX>;

macro_rules! initial_pib {
    ($phy:expr, $d:expr) => {{
        let mut pib = MacPib::new($phy.ieee, 0, 0);
        pib.set(PibAttribute::PhyTransmitPower, PibValue::I8($d))
            .map_err(|_| MacError::InvalidParameter)?;
        pib
    }};
}

impl<const D: i8> Cc26x2RadioPhy<D> {
    /// Wrap a started radio. `eui64` is the factory address in over-the-air
    /// order (`cc26x2_hal::chip::Identity::eui64`).
    pub fn new(radio: Radio, trng: Trng, eui64: IeeeAddress) -> Result<Self, PhyError> {
        let (min, max) = rf::settings::range(cc26x2_hal::chip::HIGH_PA);
        if D < min || D > max {
            return Err(PhyError::Unsupported);
        }
        Ok(Self {
            radio,
            trng,
            ieee: eui64,
            rx_on_when_idle: false,
            pan_coordinator: false,
            filter: None,
            last_error: None,
            aes: None,
        })
    }

    /// Take the CRYPTO engine as this PHY's AES-128 backend after its
    /// FIPS-197 known-answer tests. Until this succeeds, secured frames
    /// fail closed.
    pub fn install_aes_engine(&mut self, token: CryptoToken) -> Result<(), Cc26x2AesError> {
        if self.aes.is_some() {
            return Err(Cc26x2AesError::AlreadyInstalled);
        }
        let mut engine = AesEngine::new(token).map_err(Cc26x2AesError::Engine)?;
        for (index, (key, input, expected)) in crate::aes_kat::AES_128_KATS.iter().enumerate() {
            let mut block = *input;
            engine
                .encrypt_block(key, &mut block)
                .map_err(Cc26x2AesError::Engine)?;
            if block != *expected {
                return Err(Cc26x2AesError::KnownAnswerTestFailed(index as u8));
            }
        }
        self.aes = Some(engine);
        Ok(())
    }

    pub fn extended_address(&self) -> IeeeAddress {
        self.ieee
    }

    pub fn radio_stats(&self) -> rf::RadioStats {
        self.radio.stats()
    }

    pub fn tx_power(&self) -> rf::settings::TxPower {
        self.radio.tx_power()
    }

    /// Most recent RF core error other than NoAck/ChannelBusy, and the
    /// receiver's status word.
    pub fn last_error(&self) -> (Option<RadioError>, u16) {
        (self.last_error, self.radio.rx_status())
    }

    /// Accept frames without a destination address (PAN coordinator /
    /// network co-processor).
    pub fn set_pan_coordinator(&mut self, enabled: bool) -> Result<(), PhyError> {
        self.pan_coordinator = enabled;
        self.apply_filter()
    }

    #[inline(never)]
    pub fn into_soft_mac(self) -> Result<Cc26x2SoftMac<D>, MacError> {
        let pib = initial_pib!(self, D);
        SoftMacCore::new(self, pib)
    }

    /// Parent-role MAC, normally [`Cc26x2RouterMac`].
    #[inline(never)]
    pub fn into_parent_mac<R: MacRole, const RX: usize>(
        self,
    ) -> Result<SoftMacCore<Self, R, RX>, MacError> {
        let pib = initial_pib!(self, D);
        SoftMacCore::new_custom(self, pib)
    }

    fn fail(&mut self, error: RadioError) -> PhyError {
        self.last_error = Some(error);
        log::warn!(
            "cc26x2: {:?} (rx status 0x{:04X})",
            error,
            self.radio.rx_status()
        );
        map_error(error)
    }

    fn apply_filter(&mut self) -> Result<(), PhyError> {
        let filter = self.filter.map(|f| Filter {
            pan_id: f.pan_id.0,
            short_address: f.short_address.0,
            extended_address: f.extended_address,
            pan_coordinator: self.pan_coordinator,
        });
        self.radio.set_filter(filter).map_err(|e| self.fail(e))
    }

    fn settle(&mut self) {
        if !self.rx_on_when_idle {
            self.radio.rx_off();
        }
    }

    /// One CSMA + TX (+ ACK) chain.
    async fn chain(&mut self, frame: &[u8], csma: Csma) -> Result<TxOutcome, PhyError> {
        if let Err(e) = self.radio.start_tx(frame, csma) {
            return Err(self.fail(e));
        }
        let deadline = Instant::now() + Duration::from_millis(TX_CHAIN_TIMEOUT_MS);
        loop {
            match self.radio.poll_tx() {
                Some(Ok(outcome)) => return Ok(outcome),
                Some(Err(e)) => return Err(self.fail(e)),
                None if Instant::now() >= deadline => {
                    self.radio.abort_tx();
                    return Err(self.fail(RadioError::Timeout));
                }
                None => yield_now().await,
            }
        }
    }
}

impl<const D: i8> RadioPhy for Cc26x2RadioPhy<D> {
    type Tx = FullTx;

    const DEFAULT_TX_POWER_DBM: i8 = D;
    /// Same as the hardware-proven EFR32/nRF backends: EmberZNet parents may
    /// hold the indirect frame behind other traffic.
    const POLL_RESPONSE_WAIT_US: u32 = 1_500_000;
    /// EmberZNet parents queue Rejoin Response / Transport-Key by EUI-64.
    const POLL_EXTENDED_FALLBACK: bool = true;
    /// Every Data Request the RF core auto-acknowledged is marked from its
    /// own appended source-match index: auto-pend set the ACK's Frame
    /// Pending from that same match, and every enabled entry is pending
    /// ([`mark_poll_ack`]). No table change after the ACK can alter it.
    const POLL_ACK_REPORT: crate::PollAckReport = crate::PollAckReport::Exact;

    fn capabilities(&self) -> PhyCapabilities {
        let (min, max) = rf::settings::range(cc26x2_hal::chip::HIGH_PA);
        PhyCapabilities {
            hardware_auto_ack: true,
            hardware_address_filter: true,
            tx_power_min: min,
            tx_power_max: max,
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        let result = self.chain(frame, Csma::SINGLE_CCA).await;
        self.settle();
        match result? {
            TxOutcome::ChannelBusy => Err(PhyError::ChannelBusy),
            _ => Ok(()),
        }
    }

    async fn transmit_offloaded(
        &mut self,
        frame: &[u8],
        ack_requested: bool,
        params: TxParams,
    ) -> Result<Option<AckResult>, PhyError> {
        // The RF core waits for an ACK exactly when the frame requests one.
        if ack_requested != frame.first().is_some_and(|fc| fc & 0x20 != 0) {
            return Err(PhyError::Unsupported);
        }
        let csma = Csma {
            min_be: params.min_be,
            max_be: params.max_be,
            max_backoffs: params.max_csma_backoffs,
        };
        let mut attempts = u16::from(params.max_frame_retries) + 1;
        let result = loop {
            attempts -= 1;
            match self.chain(frame, csma).await {
                Ok(TxOutcome::Sent) => break Ok(None),
                Ok(TxOutcome::Acked { frame_pending }) => {
                    break Ok(Some(AckResult { frame_pending }));
                }
                Ok(TxOutcome::ChannelBusy) => break Err(PhyError::ChannelBusy),
                Ok(TxOutcome::NoAck) if attempts == 0 => break Ok(None),
                Ok(TxOutcome::NoAck) => continue,
                Err(e) => break Err(e),
            }
        };
        // A Frame Pending ACK is followed by a receive window.
        if !matches!(
            result,
            Ok(Some(AckResult {
                frame_pending: true
            }))
        ) {
            self.settle();
        }
        result
    }

    /// Never called: the RF core acknowledges (`hardware_auto_ack`).
    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        if let Err(e) = self.radio.rx_on() {
            return Err(self.fail(e));
        }
        let deadline = Instant::now() + Duration::from_micros(u64::from(timeout_us));
        loop {
            if let Some(info) = self.radio.receive(frame.buffer_mut()) {
                frame.set_len(info.len, crate::lqi::from_rssi_dbm(info.rssi_dbm))?;
                mark_poll_ack(frame, &info);
                #[cfg(feature = "rx-rssi")]
                {
                    frame.rssi_dbm = info.rssi_dbm;
                }
                return Ok(true);
            }
            if Instant::now() >= deadline {
                self.settle();
                return Ok(false);
            }
            yield_now().await;
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        self.radio.set_channel(channel).map_err(|e| self.fail(e))
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        self.radio
            .set_tx_power(dbm)
            .map(|_| ())
            .map_err(|e| self.fail(e))
    }

    async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError> {
        let channel = self.radio.channel();
        if let Err(e) = self.radio.start_energy_detect(channel, duration_us) {
            return Err(self.fail(e));
        }
        Timer::after_micros(u64::from(duration_us)).await;
        let deadline = Instant::now() + Duration::from_millis(20);
        let result = loop {
            match self.radio.poll_energy_detect() {
                Some(Ok(rssi)) => break Ok(crate::lqi::from_rssi_dbm(rssi)),
                Some(Err(e)) => break Err(self.fail(e)),
                None if Instant::now() >= deadline => break Err(self.fail(RadioError::Timeout)),
                None => yield_now().await,
            }
        };
        self.settle();
        result
    }

    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        self.filter = filter;
        self.apply_filter()
    }

    fn set_ack_frame_pending(
        &mut self,
        source: &MacAddress,
        pending: bool,
    ) -> Result<(), PhyError> {
        let address = match *source {
            MacAddress::Short(pan, short) => FpAddress::Short {
                pan_id: pan.0,
                address: short.0,
            },
            MacAddress::Extended(_, ieee) => FpAddress::Extended(ieee),
        };
        self.radio
            .set_frame_pending(address, pending)
            .map_err(|error| match error {
                FpError::TableFull => PhyError::Hardware,
                FpError::Radio(e) => self.fail(e),
            })
    }

    fn clear_ack_frame_pending(&mut self) {
        self.radio.clear_frame_pending();
    }

    fn set_rx_on_when_idle(&mut self, enabled: bool) {
        self.rx_on_when_idle = enabled;
        if enabled {
            // A failure here resurfaces on the next receive or transmit.
            if let Err(e) = self.radio.rx_on() {
                self.fail(e);
            }
        } else {
            self.radio.rx_off();
        }
    }

    /// Receiver off; the RF core stays powered (no standby support yet).
    fn sleep(&mut self) -> Result<(), PhyError> {
        self.radio.rx_off();
        Ok(())
    }

    fn wake(&mut self) -> Result<(), PhyError> {
        if self.rx_on_when_idle {
            self.radio.rx_on().map_err(|e| self.fail(e))?;
        }
        Ok(())
    }
}

/// AES-128 on the CRYPTO engine, fail-closed when it is not installed.
impl<const D: i8> zigbee_crypto::ForwardAesProvider for Cc26x2RadioPhy<D> {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        Cc26x2HardwareAes128 {
            engine: self.aes.as_mut(),
            key: *key,
        }
    }
}

struct Cc26x2HardwareAes128<'engine> {
    engine: Option<&'engine mut AesEngine<Mmio>>,
    key: zigbee_crypto::AesKey,
}

impl zigbee_crypto::Aes128Forward for Cc26x2HardwareAes128<'_> {
    type Error = Cc26x2AesError;

    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
        self.engine
            .as_deref_mut()
            .ok_or(Cc26x2AesError::NotInstalled)?
            .encrypt_block(&self.key, block)
            .map_err(Cc26x2AesError::Engine)
    }
}

impl<const D: i8> PlatformServices for Cc26x2RadioPhy<D> {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.trng.fill(output).map_err(|_| MacError::RadioError)
    }
}

fn map_error(error: RadioError) -> PhyError {
    match error {
        RadioError::InvalidChannel => PhyError::InvalidChannel,
        RadioError::FrameTooLong => PhyError::FrameTooLong,
        RadioError::Power
        | RadioError::Doorbell(_)
        | RadioError::Op(_)
        | RadioError::Timeout
        | RadioError::Off => PhyError::Hardware,
    }
}

/// Carry the RF core's Frame Pending decision with a received Data Request
/// ([`PhyRxFrame::acked_without_frame_pending`]): the child turned its
/// receiver off after an ACK without Frame Pending, so the NWK keeps a frame
/// queued since for its next poll. The decision is the RF core's own
/// per-frame source-match result ([`rf::RxInfo::source_matched`]), appended
/// to the frame by the same matching that set the ACK's bit, so no table
/// change between the ACK and this dequeue can alter it. Call after
/// [`PhyRxFrame::set_len`], which clears the flag.
fn mark_poll_ack(frame: &mut PhyRxFrame, info: &rf::RxInfo) {
    if info.auto_acked && crate::ack_pending::data_request_source(frame.as_slice()).is_some() {
        frame.set_acked_without_frame_pending(!info.source_matched);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Data Request from short 0x12D2 to the parent 0x0000 (PAN 0x1A62).
    const POLL: [u8; 10] = [0x63, 0x88, 0x42, 0x62, 0x1A, 0x00, 0x00, 0xD2, 0x12, 0x04];

    fn info(len: usize, auto_acked: bool, source_matched: bool) -> rf::RxInfo {
        rf::RxInfo {
            len,
            rssi_dbm: -60,
            correlation: 40,
            crc_ok: true,
            auto_acked,
            source_matched,
        }
    }

    /// What the PHY hands the MAC for one dequeued RX entry.
    fn received(frame: &[u8], info: &rf::RxInfo) -> PhyRxFrame {
        let mut out = PhyRxFrame::EMPTY;
        out.buffer_mut()[..frame.len()].copy_from_slice(frame);
        // A stale flag from the slot's previous frame must not survive.
        out.set_acked_without_frame_pending(true);
        out.set_len(info.len, 200).unwrap();
        mark_poll_ack(&mut out, info);
        out
    }

    /// The secured-rejoin sequence of #23 on a CC26x2 parent: the first
    /// poll found no source match (FP = 0), the response was armed, the next
    /// poll matched (FP = 1). Each poll carries its own entry's decision.
    #[test]
    fn each_poll_carries_its_own_source_match() {
        let stale = received(&POLL, &info(POLL.len(), true, false));
        assert!(stale.acked_without_frame_pending());
        assert_eq!(stale.as_slice(), POLL);

        let mut next = POLL;
        next[2] = 0x43;
        let served = received(&next, &info(next.len(), true, true));
        assert!(!served.acked_without_frame_pending());
    }

    /// `PollAckReport::Exact`: every Data Request the RF core acknowledged
    /// carries its decision both ways (short and extended sources), so an
    /// unmarked one was source-matched, i.e. acknowledged with FP = 1.
    #[test]
    fn every_acknowledged_poll_carries_its_decision_both_ways() {
        let parent = MacAddress::Short(zigbee_types::PanId(0x1A62), zigbee_types::ShortAddress(0));
        let extended = crate::frames::build_data_request(0x44, &parent, &[0xC1; 8]);
        for poll in [&POLL[..], &extended[..]] {
            assert!(crate::ack_pending::data_request_source(poll).is_some());
            for source_matched in [false, true] {
                let frame = received(poll, &info(poll.len(), true, source_matched));
                assert_eq!(frame.acked_without_frame_pending(), !source_matched);
            }
        }
    }

    #[test]
    fn only_acknowledged_data_requests_are_marked() {
        // No auto-ACK (promiscuous scan receiver, broadcast destination).
        assert!(!received(&POLL, &info(POLL.len(), false, false)).acked_without_frame_pending());
        // A data frame ACKed with FP = 0 is not a poll.
        let mut data = POLL;
        data[0] = 0x61;
        assert!(!received(&data, &info(data.len(), true, false)).acked_without_frame_pending());
        // An Association Request (command 0x01) is not a poll either.
        let mut association = POLL;
        association[9] = 0x01;
        assert!(
            !received(&association, &info(association.len(), true, false))
                .acked_without_frame_pending()
        );
    }
}
