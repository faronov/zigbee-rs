//! TLSR8258 [`RadioPhy`] backend for the shared [`SoftMacCore`].

use crate::pib::MacPib;
use crate::soft_mac::DEFAULT_PENDING_RX;
use crate::{
    AckResult, EndDevice, FullTx, MacError, MacRole, Parent, PhyAddressFilter, PhyCapabilities,
    PhyError, PhyRxFrame, PlatformServices, RadioPhy, SoftMacCore, TxParams, WrappingTickExtender,
};
#[cfg(feature = "hardware-aes")]
use tlsr8258_hal::aes::{AesEngine, AesError};
use tlsr8258_hal::radio::{
    AckPendingAddress, AckPendingError, CsmaParams, Radio, RawRxOutcome, TX_POWER_MAX_DBM,
    TX_POWER_MIN_DBM, TxOutcome,
};
use tlsr8258_hal::rng::Rng;
#[cfg(all(feature = "hardware-aes", feature = "telink-retention-proof"))]
use tlsr8258_hal::rng::RngError;
use tlsr8258_hal::{flash, timer};
use zigbee_types::{IeeeAddress, MacAddress};

/// Retained-RX depth of the Telink MACs.
///
/// Only frames heard inside an ACK or poll window that are not the awaited
/// frame land here; the HAL's own bounded interrupt queue absorbs bursts in
/// front of it.
pub const TELINK_PENDING_RX: usize = DEFAULT_PENDING_RX;

/// Imm-Ack wait after each transmission, kept at the value proven on
/// hardware by the previous Telink MAC. The window closes as soon as the
/// matching ACK arrives, so only an unacknowledged attempt pays it.
const ACK_WAIT_TICKS: u32 = timer::ms(8);

/// Shared software MAC, end-device role, on the TLSR8258 radio.
pub type TelinkMac = SoftMacCore<TelinkRadioPhy, EndDevice, TELINK_PENDING_RX>;

/// Parent role of the TLSR8258 router: eight queued MAC command events and
/// eight concurrent Association Response transactions. The software Frame
/// Pending table is empty because the HAL's ISR ACK path keeps its own
/// source-match table ([`RadioPhy::set_ack_frame_pending`]).
pub type TelinkRouterRole = Parent<8, 8, 0>;

/// Retained-RX depth of the always-on router: the eight data indications
/// the hand-written router MAC buffered, since an always-on parent hears
/// child and neighbour traffic between runtime receive calls.
pub const TELINK_ROUTER_PENDING_RX: usize = 8;

/// Shared software MAC, parent (router) role, on the TLSR8258 radio.
pub type TelinkRouterMac = SoftMacCore<TelinkRadioPhy, TelinkRouterRole, TELINK_ROUTER_PENDING_RX>;

/// TLSR8258 radio, monotonic clock, RNG and optional AES engine exposed
/// through the portable PHY contract.
///
/// The application must initialise the TLSR8258 clocks before constructing
/// it. Build the MAC with one of the [`SoftMacCore`] constructors below.
pub struct TelinkRadioPhy {
    radio: Radio,
    clock: WrappingTickExtender,
    /// Lazily acquired on the first [`PlatformServices::fill_random`] call so
    /// construction never pays the ADC-sampling cost (or fails) unless random
    /// bytes are actually requested. See `tlsr8258_hal::rng`'s module docs for
    /// exactly what this provides and does not prove about entropy quality.
    rng: Option<Rng>,
    /// Hardware AES-128 accelerator, installed exactly once by the
    /// composition root through [`SoftMacCore::install_aes_engine`] from the
    /// board's exclusive `Aes` token.
    #[cfg(feature = "hardware-aes")]
    aes_engine: Option<AesEngine>,
}

/// Fail-closed causes while recreating hardware behind a retained Telink
/// MAC. No application service or IRQ re-enable is allowed after an error.
#[cfg(all(feature = "hardware-aes", feature = "telink-retention-proof"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelinkRetentionResumeError {
    Radio(MacError),
    AesNotInstalled,
    Aes(AesError),
    Rng(RngError),
}

impl TelinkRadioPhy {
    fn take() -> Result<Self, MacError> {
        let mut radio = Radio::take().ok_or(MacError::RadioError)?;
        radio.init();
        Ok(Self {
            radio,
            clock: WrappingTickExtender::new(timer::now_ticks()),
            rng: None,
            #[cfg(feature = "hardware-aes")]
            aes_engine: None,
        })
    }

    const fn channel_valid(channel: u8) -> bool {
        matches!(channel, 11..=26)
    }
}

impl<R: MacRole, const RX: usize> SoftMacCore<TelinkRadioPhy, R, RX> {
    /// Acquire the RF block and build the MAC with the factory EUI-64 of the
    /// 512-KiB TB-04 layout (or the stable flash-UID fallback).
    pub fn with_factory_address() -> Result<Self, MacError> {
        let mut address = [0u8; 8];
        flash::factory_ieee(&mut address);
        Self::with_extended_address(address)
    }

    /// Verify `geometry` against the fitted JEDEC flash, then build the MAC
    /// with the factory EUI-64 from that geometry's Telink factory sector.
    ///
    /// Non-512-KiB products use this instead of
    /// [`Self::with_factory_address`].
    pub fn new_for_flash_geometry(geometry: flash::FlashGeometry) -> Result<Self, MacError> {
        let mut address = [0u8; 8];
        flash::factory_ieee_for(geometry, &mut address).map_err(|_| MacError::RadioError)?;
        Self::with_extended_address(address)
    }

    /// Acquire the RF block and build the MAC with `extended_address`.
    ///
    /// Fails if the radio singleton was already taken. `macDSN`/`macBSN` and
    /// the CSMA-CA backoff seed come from the free-running timer so they do
    /// not repeat across resets.
    #[inline(never)]
    pub fn with_extended_address(extended_address: IeeeAddress) -> Result<Self, MacError> {
        let phy = TelinkRadioPhy::take()?;
        let seed = timer::now_ticks();
        let pib = MacPib::new(extended_address, seed as u8, (seed >> 8) as u8);
        Self::new_custom(phy, pib)
    }

    /// Take ownership of the AES-128 accelerator and use it for every
    /// subsequent CCM* and AES-MMO operation.
    ///
    /// Call exactly once, right after construction, with the board's single
    /// exclusive `Aes` token. Back-to-back AES-128 known-answer tests,
    /// including a re-key/reuse cycle, run against the real peripheral
    /// before the engine is accepted.
    ///
    /// Kept out of line: the CI symbol gate proves every production image
    /// installs the engine by finding this function.
    #[cfg(feature = "hardware-aes")]
    #[inline(never)]
    pub fn install_aes_engine(
        &mut self,
        aes: tlsr8258_hal::peripherals::Aes,
    ) -> Result<(), AesError> {
        let mut engine = AesEngine::new(aes, AesEngine::DEFAULT_TIMEOUT_ITERATIONS)?;
        engine.self_test()?;
        self.phy_mut().aes_engine = Some(engine);
        Ok(())
    }

    /// Quiesce the RF/DMA block before entering TLSR8258 low power.
    ///
    /// Call only after the MAC operation in progress has completed. Frames
    /// already copied into the MAC's retained-RX queue stay valid.
    #[inline(never)]
    pub fn prepare_for_sleep(&mut self) {
        self.phy_mut().radio.prepare_for_sleep();
    }

    /// Restore the RF/DMA block after TLSR8258 wake and re-apply channel,
    /// TX power, address/ACK filter and `macRxOnWhenIdle` from the PIB.
    ///
    /// Radio init reinitialises Timer0; an in-place SUSPEND coordinator must
    /// rebase application monotonic time after this call and before
    /// interrupts resume.
    #[inline(never)]
    pub fn resume_after_sleep(&mut self) -> Result<(), MacError> {
        self.wake()
    }

    /// Restore every hardware-backed capability after LOW32K reset while
    /// preserving the retained PIB, security-facing MAC counters and the
    /// optional DRBG state.
    #[cfg(all(feature = "hardware-aes", feature = "telink-retention-proof"))]
    #[inline(never)]
    pub fn resume_after_retention(&mut self) -> Result<(), TelinkRetentionResumeError> {
        self.resume_after_sleep()
            .map_err(TelinkRetentionResumeError::Radio)?;
        let phy = self.phy_mut();
        phy.aes_engine
            .as_mut()
            .ok_or(TelinkRetentionResumeError::AesNotInstalled)?
            .resume_after_retention()
            .map_err(TelinkRetentionResumeError::Aes)?;
        if let Some(rng) = phy.rng.as_mut() {
            rng.resume_after_retention()
                .map_err(TelinkRetentionResumeError::Rng)?;
        }
        Ok(())
    }
}

impl RadioPhy for TelinkRadioPhy {
    /// CSMA-CA, ACK wait and retries run synchronously in the HAL with the
    /// RF IRQ masked, so the ACK can never race the ISR queue and frames
    /// heard during backoff or the ACK window stay in the HAL's RX queue
    /// for the next [`Self::receive`].
    type Tx = FullTx;

    #[inline]
    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            // The HAL sends the Imm-Ack from its RX-completion path
            // (`ram_code`, in the RF IRQ while `macRxOnWhenIdle`), using the
            // identity from `set_address_filter` and the Frame Pending table
            // from `set_ack_frame_pending`.
            hardware_auto_ack: true,
            hardware_address_filter: false,
            tx_power_min: TX_POWER_MIN_DBM,
            tx_power_max: TX_POWER_MAX_DBM,
        }
    }

    /// Never called: [`Self::Tx`] is [`FullTx`].
    async fn try_transmit(&mut self, _frame: &[u8]) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    async fn transmit_offloaded(
        &mut self,
        frame: &[u8],
        ack_requested: bool,
        params: TxParams,
    ) -> Result<Option<AckResult>, PhyError> {
        let csma = CsmaParams {
            min_be: params.min_be,
            max_be: params.max_be,
            max_backoffs: params.max_csma_backoffs,
        };
        let (ack_wait, retries) = if ack_requested {
            (ACK_WAIT_TICKS, params.max_frame_retries)
        } else {
            (0, 0)
        };
        for _ in 0..=retries {
            match self.radio.transmit_csma(frame, csma, ack_wait) {
                Ok(Some(frame_pending)) => return Ok(Some(AckResult { frame_pending })),
                Ok(None) => {}
                Err(TxOutcome::ChannelAccessFailure) => return Err(PhyError::ChannelBusy),
                Err(TxOutcome::InvalidFrame) => return Err(PhyError::FrameTooLong),
                Err(_) => return Err(PhyError::Hardware),
            }
        }
        Ok(None)
    }

    /// Never called: [`Self::capabilities`] reports `hardware_auto_ack`.
    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    /// Frames that fail the hardware length/CRC checks are skipped inside the
    /// same bounded window, so this returns only valid frames or `false`.
    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        let mut received = false;
        self.radio
            .receive_raw_until(timer::us(timeout_us), u16::MAX, |outcome| {
                let RawRxOutcome::Frame(rx) = outcome else {
                    return false;
                };
                received = frame.set(rx.as_slice(), rx.lqi).is_ok();
                received
            });
        Ok(received)
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !Self::channel_valid(channel) {
            return Err(PhyError::InvalidChannel);
        }
        self.radio.set_channel(channel);
        Ok(())
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        if self.radio.set_tx_power(dbm) {
            Ok(())
        } else {
            // Reported to the NWK as InvalidParameter, as before.
            Err(PhyError::InvalidChannel)
        }
    }

    async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
        Ok(self.radio.measure_energy())
    }

    /// Program the identity the HAL's ISR ACK path and its bounded RX queue
    /// use. Promiscuous reception (`None`) is a software decision in the
    /// core; the ACK identity stays the device's own.
    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        if let Some(filter) = filter {
            self.radio.set_ack_filter(
                filter.pan_id.0,
                filter.short_address.0,
                filter.extended_address,
            );
        }
        Ok(())
    }

    fn set_ack_frame_pending(
        &mut self,
        source: &MacAddress,
        pending: bool,
    ) -> Result<(), PhyError> {
        let child = match *source {
            MacAddress::Short(pan_id, address) => AckPendingAddress::Short {
                pan_id: pan_id.0,
                address: address.0,
            },
            MacAddress::Extended(pan_id, address) => AckPendingAddress::Extended {
                pan_id: pan_id.0,
                address,
            },
        };
        self.radio
            .set_ack_frame_pending(child, pending)
            .map_err(|error| match error {
                // Mapped to TransactionOverflow by the parent role.
                AckPendingError::TableFull => PhyError::Hardware,
                // Mapped to InvalidParameter.
                AckPendingError::InvalidAddress => PhyError::InvalidChannel,
            })
    }

    fn clear_ack_frame_pending(&mut self) {
        self.radio.clear_ack_frame_pending();
    }

    fn set_rx_on_when_idle(&mut self, enabled: bool) {
        self.radio.set_rx_on_when_idle(enabled);
    }

    fn sleep(&mut self) -> Result<(), PhyError> {
        self.radio.prepare_for_sleep();
        Ok(())
    }

    fn wake(&mut self) -> Result<(), PhyError> {
        self.radio.init();
        Ok(())
    }
}

/// Generic software provider when `hardware-aes` is off. The production
/// Telink sensor and router always enable the hardware feature.
#[cfg(not(feature = "hardware-aes"))]
impl zigbee_crypto::ForwardAesProvider for TelinkRadioPhy {}

/// Hardware AES-128 backend for CCM* and AES-MMO, so the RustCrypto software
/// core is dead-code-eliminated from the image. A missing engine is a
/// firmware wiring bug and traps rather than silently falling back to
/// software.
#[cfg(feature = "hardware-aes")]
impl zigbee_crypto::ForwardAesProvider for TelinkRadioPhy {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        let engine = self
            .aes_engine
            .as_mut()
            .expect("AES engine not installed: call install_aes_engine()");
        zigbee_crypto::tlsr8258::HardwareAes128::new(engine, *key)
    }
}

impl PlatformServices for TelinkRadioPhy {
    fn monotonic_micros(&self) -> u32 {
        (self.clock.extend(timer::now_ticks()) / u64::from(timer::TICKS_PER_US)) as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        timer::sleep_ticks(timer::us(duration_us));
    }

    /// Bytes from `tlsr8258_hal::rng`: a NIST SP 800-90A CTR_DRBG (AES-128,
    /// KAT-verified) seeded from SHA-256-conditioned ADC noise of the
    /// VBAT/GND channel. The entropy quality of that seed has not been
    /// independently measured on hardware (no SP 800-90B assessment).
    ///
    /// Every acquisition or reseed failure is returned as an error, never a
    /// silent fallback to a weaker source. `MacError` has no entropy-source
    /// variant, so the HAL's finer causes collapse to `Unsupported`.
    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        if self.rng.is_none() {
            self.rng = Some(Rng::take().map_err(|_| MacError::Unsupported)?);
        }
        let Some(rng) = self.rng.as_mut() else {
            return Err(MacError::Unsupported);
        };
        rng.fill_bytes(output).map_err(|_| MacError::Unsupported)
    }
}
