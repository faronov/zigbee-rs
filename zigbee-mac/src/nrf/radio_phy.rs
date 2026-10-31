//! nRF52833/nRF52840 [`RadioPhy`] backend for the shared [`SoftMacCore`].

use crate::frames::build_ack;
use crate::pib::MacPib;
use crate::{
    EndDevice, MAX_PHY_FRAME_LEN, MacError, PhyAddressFilter, PhyCapabilities, PhyError,
    PhyRxFrame, PlatformServices, RadioPhy, SoftMacCore,
};
use embassy_futures::select::{Either, select};
use embassy_time::Timer;
use zigbee_types::IeeeAddress;

use super::embassy_nrf::radio::Error as RadioError;
use super::{
    NrfAesError, NrfAesState, NrfEcbToken, NrfHardwareAes128, Packet, Radio, RadioInstance, Rng,
    RngInstance,
};

/// nRF52833/nRF52840 radio and RNG exposed through the portable PHY contract.
pub struct NrfRadioPhy<'a, T: RadioInstance, R: RngInstance> {
    radio: Radio<'a, T>,
    rng: Rng<'a, R>,
    aes: NrfAesState,
}

/// Retained-RX depth of the nRF end-device MAC.
///
/// A channel that already carries an active mesh delivers unrelated
/// broadcast traffic throughout the join, so the queue has to absorb that
/// background while still retaining the APS Transport-Key. Same depth as the
/// legacy `NrfMac` queue it replaces.
pub const NRF_PENDING_RX: usize = 8;

/// Shared software MAC (end-device role) using the nRF radio adapter.
///
/// Construct with [`NrfRadioPhy::into_soft_mac`].
pub type NrfSoftMac<'a, T, R> = SoftMacCore<NrfRadioPhy<'a, T, R>, EndDevice, NRF_PENDING_RX>;

impl<'a, T: RadioInstance, R: RngInstance> NrfRadioPhy<'a, T, R> {
    pub fn new(radio: Radio<'a, T>, rng: Rng<'a, R>) -> Self {
        Self {
            radio,
            rng,
            aes: NrfAesState::new(),
        }
    }

    /// Consume the unique ECB token and install hardware AES after both
    /// startup known-answer tests pass.
    pub fn install_aes_engine(&mut self, token: NrfEcbToken) -> Result<(), NrfAesError> {
        self.aes.install(token)
    }

    /// Factory-programmed EUI-64 from FICR.DEVICEID[0..2]
    /// (0x1000_0060 low word, 0x1000_0064 high word, little-endian).
    pub fn read_ficr_ieee() -> IeeeAddress {
        const FICR_DEVICEID0: *const u32 = 0x1000_0060 as *const u32;
        const FICR_DEVICEID1: *const u32 = 0x1000_0064 as *const u32;
        // SAFETY: FICR is read-only factory information on nRF52833/nRF52840.
        let lo = unsafe { core::ptr::read_volatile(FICR_DEVICEID0) };
        // SAFETY: as above.
        let hi = unsafe { core::ptr::read_volatile(FICR_DEVICEID1) };
        let mut address = [0u8; 8];
        address[0..4].copy_from_slice(&lo.to_le_bytes());
        address[4..8].copy_from_slice(&hi.to_le_bytes());
        address
    }

    /// One hardware-RNG word, e.g. to seed `macDSN`/`macBSN`.
    pub fn random_u32(&mut self) -> u32 {
        let mut entropy = [0u8; 4];
        self.rng.blocking_fill_bytes(&mut entropy);
        u32::from_le_bytes(entropy)
    }

    /// Build the end-device [`NrfSoftMac`] on top of this PHY.
    ///
    /// The EUI-64 comes from FICR and `macDSN`/`macBSN` are seeded from the
    /// hardware RNG, so neither repeats across resets. Install the AES
    /// engine before calling this.
    #[inline(never)]
    pub fn into_soft_mac(mut self) -> Result<NrfSoftMac<'a, T, R>, MacError> {
        let seed = self.random_u32();
        let pib = MacPib::new(Self::read_ficr_ieee(), seed as u8, (seed >> 8) as u8);
        SoftMacCore::new_custom(self, pib)
    }

    /// Stop the RADIO peripheral between sleepy-device poll windows.
    ///
    /// `&mut self` guarantees no Embassy radio future is active. Interrupts
    /// are cleared before requesting DISABLED, and the next Embassy operation
    /// performs the normal DISABLED-to-RX/TX transition.
    pub fn enter_low_power_idle(&mut self) -> Result<(), PhyError> {
        self.radio.clear_all_interrupts();
        if super::force_radio_disabled() {
            Ok(())
        } else {
            Err(PhyError::Hardware)
        }
    }
}

impl<T: RadioInstance, R: RngInstance> RadioPhy for NrfRadioPhy<'_, T, R> {
    /// Software CSMA-CA, ACK wait and retries (no hardware auto-ACK).
    type Tx = crate::SoftwareTx;

    /// Hardware-proven legacy `NrfMac` value: on a busy channel the parent's
    /// indirect transmission competes with coordinator traffic.
    const POLL_RESPONSE_WAIT_US: u32 = 1_500_000;
    /// EmberZNet parents queue Rejoin Response / Transport-Key by EUI-64.
    const POLL_EXTENDED_FALLBACK: bool = true;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            hardware_auto_ack: false,
            hardware_address_filter: false,
            tx_power_min: -20,
            tx_power_max: 8,
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        if frame.len() > MAX_PHY_FRAME_LEN {
            return Err(PhyError::FrameTooLong);
        }

        let mut packet = Packet::new();
        packet.copy_from_slice(frame);
        // Cancel-safe: dropping this future mid-TX stops the radio before the
        // stack-owned packet buffer goes away.
        super::send_packet_cancel_safe(&mut self.radio, &mut packet)
            .await
            .map_err(|error| {
                if error == RadioError::ChannelInUse {
                    PhyError::ChannelBusy
                } else {
                    Self::map_radio_error(error)
                }
            })
    }

    /// Software Imm-Ack from task context.
    ///
    /// Embassy-nrf 0.3 exposes only a CCA-gated transmit, so the ACK goes out
    /// as `CCASTART → TXEN` (~128 µs CCA + ramp-up) after the frame was
    /// handed to the MAC. That stays inside the sender's macAckWaitDuration
    /// (864 µs) in practice but exceeds the nominal 192 µs aTurnaroundTime;
    /// a busy CCA reports [`PhyError::ChannelBusy`] and the sender retries.
    /// Same behaviour as the legacy `NrfMac`; ACK timing remains a
    /// hardware-in-the-loop gate.
    async fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), PhyError> {
        let mut packet = Packet::new();
        packet.copy_from_slice(&build_ack(sequence, frame_pending));
        super::send_packet_cancel_safe(&mut self.radio, &mut packet)
            .await
            .map_err(Self::map_radio_error)
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        let mut packet = Packet::new();
        match select(
            Timer::after_micros(u64::from(timeout_us)),
            self.radio.receive(&mut packet),
        )
        .await
        {
            Either::First(()) => Ok(false),
            Either::Second(Ok(())) => {
                // `Packet::lqi()` is the raw Nordic ED byte; the stack works
                // in IEEE 802.15.4 LQI. Normalize exactly once, here.
                frame.set(packet.as_ref(), crate::lqi::nrf_from_hardware(packet.lqi()))?;
                Ok(true)
            }
            Either::Second(Err(error)) => Err(Self::map_radio_error(error)),
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !(11..=26).contains(&channel) {
            return Err(PhyError::InvalidChannel);
        }
        self.radio.set_channel(channel);
        Ok(())
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        if !matches!(
            dbm,
            -40 | -30 | -20 | -16 | -12 | -8 | -4 | 0 | 2 | 3 | 4 | 5 | 6 | 7 | 8
        ) {
            return Err(PhyError::Unsupported);
        }
        self.radio.set_transmission_power(dbm);
        Ok(())
    }

    async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
        Err(PhyError::Unsupported)
    }

    fn set_address_filter(&mut self, _filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        Ok(())
    }

    fn sleep(&mut self) -> Result<(), PhyError> {
        self.enter_low_power_idle()
    }
}

impl<T: RadioInstance, R: RngInstance> zigbee_crypto::ForwardAesProvider for NrfRadioPhy<'_, T, R> {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        NrfHardwareAes128 {
            engine: self.aes.engine_mut(),
            key: *key,
        }
    }
}
impl<T: RadioInstance, R: RngInstance> PlatformServices for NrfRadioPhy<'_, T, R> {
    fn monotonic_micros(&self) -> u32 {
        embassy_time::Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.rng.blocking_fill_bytes(output);
        Ok(())
    }
}

impl<T: RadioInstance, R: RngInstance> NrfRadioPhy<'_, T, R> {
    fn map_radio_error(error: RadioError) -> PhyError {
        match error {
            RadioError::BufferTooLong => PhyError::FrameTooLong,
            RadioError::ChannelInUse => PhyError::ChannelBusy,
            RadioError::CrcFailed(_) => PhyError::CrcFailed,
            _ => PhyError::Hardware,
        }
    }
}
