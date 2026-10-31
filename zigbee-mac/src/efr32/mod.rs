//! EFR32MG1P radio backend — pure Rust, zero vendor blobs.
//!
//! The EFR32MG1P (Cortex-M4F, e.g. the IKEA TRÅDFRI modules) radio is driven
//! by direct register access in [`driver`]; no RAIL library or GSDK binary
//! is linked. The 802.15.4 MAC itself is the shared [`crate::SoftMacCore`];
//! this module provides only the EFR32MG1-specific pieces underneath it:
//!
//! - [`Efr32RadioPhy`]: the thin [`RadioPhy`] + [`PlatformServices`]
//!   adapter over [`driver`];
//! - [`Efr32Mac`]: the end-device MAC used by `efr32mg1-sensor`, a plain
//!   alias of the shared core in the end-device role;
//! - the factory EUI-64 (DEVINFO.UNIQUEL/H).
//!
//! # Division of work
//! - `SoftMacCore`: scan, association, polling (with the extended-source
//!   fallback poll), unslotted CSMA-CA backoff, `macMaxFrameRetries`, PIB
//!   and address filtering of received frames.
//! - [`Efr32RadioPhy`] ([`AckWaitTx`]): one CCA + TX; `transmit()` re-arms
//!   RX right after TX_DONE and the PHY waits up to 1.5 ms for the ACK.
//! - [`driver`]'s FRC IRQ: the Imm-Ack for received frames that request one
//!   and name this node exactly (`hardware_auto_ack`, never with Frame
//!   Pending), and DSN filtering of received ACKs.

pub mod driver;
mod rac_seq;

use core::future::Future;

use crate::pib::MacPib;
use crate::{
    AckResult, AckWaitTx, EndDevice, MacError, PhyAddressFilter, PhyCapabilities, PhyError,
    PhyRxFrame, PlatformServices, RadioPhy, SoftMacCore,
};
use driver::{Efr32Driver, RadioConfig, RadioError};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use zigbee_types::IeeeAddress;

/// Imm-Ack wait after TX_DONE (hardware-proven legacy value).
const ACK_WAIT: Duration = Duration::from_micros(1_500);

/// EUI-64 used when DEVINFO carries no unique identifier.
const FALLBACK_IEEE: IeeeAddress = [0x00, 0x0D, 0x6F, 0xFF, 0xFE, 0xDE, 0xAD, 0x02];

/// Retained-RX depth of the EFR32MG1 end-device MAC: one slot, like the
/// legacy backend's single association-frame buffer (the TRÅDFRI image has
/// no SRAM or stack to spare).
pub const EFR32_PENDING_RX: usize = 1;

/// Shared software MAC (end-device role) on the EFR32MG1 radio.
pub type Efr32SoftMac = SoftMacCore<Efr32RadioPhy, EndDevice, EFR32_PENDING_RX>;

/// EFR32MG1P radio exposed through the portable PHY contract.
pub struct Efr32RadioPhy {
    driver: Efr32Driver,
    /// The receiver was armed by `transmit()` or a previous live reception
    /// and must not be restarted: the parent's indirect frame may already
    /// be arriving right after the ACK.
    rx_live: bool,
    #[cfg(all(feature = "hardware-aes-efr32mg1", target_arch = "arm"))]
    aes_engine: Option<efr32mg1_hal::crypto::AesEngine>,
}

impl Efr32RadioPhy {
    /// Initialise the radio (channel 11, 0 dBm, broadcast PAN).
    pub fn new() -> Self {
        Self {
            driver: Efr32Driver::new(RadioConfig::default()),
            rx_live: false,
            #[cfg(all(feature = "hardware-aes-efr32mg1", target_arch = "arm"))]
            aes_engine: None,
        }
    }

    /// Consume the CRYPTO token and install its AES-128 engine after two
    /// on-silicon startup KATs.
    #[cfg(all(feature = "hardware-aes-efr32mg1", target_arch = "arm"))]
    pub fn install_aes_engine(
        &mut self,
        peripheral: efr32mg1_hal::peripherals::Crypto,
    ) -> Result<(), efr32mg1_hal::crypto::AesError> {
        let mut engine = efr32mg1_hal::crypto::AesEngine::new(
            peripheral,
            efr32mg1_hal::crypto::AesEngine::DEFAULT_TIMEOUT_ITERATIONS,
        )?;
        engine.self_test()?;
        self.aes_engine = Some(engine);
        Ok(())
    }

    /// Factory EUI-64 from DEVINFO.UNIQUEL/UNIQUEH (0x0FE0_81F0/0x0FE0_81F4),
    /// in over-the-air (little-endian) order.
    pub fn read_factory_ieee() -> IeeeAddress {
        const DI_UNIQUEL_ADDR: usize = 0x0FE0_81F0;
        const DI_UNIQUEH_ADDR: usize = 0x0FE0_81F4;
        // SAFETY: DEVINFO is read-only factory information on EFR32MG1P.
        let lo = unsafe { core::ptr::read_volatile(DI_UNIQUEL_ADDR as *const u32) };
        // SAFETY: as above.
        let hi = unsafe { core::ptr::read_volatile(DI_UNIQUEH_ADDR as *const u32) };
        ieee_from_devinfo(lo, hi)
    }

    /// Build the end-device [`Efr32Mac`] with the factory EUI-64.
    ///
    /// Install the AES engine before calling this.
    pub fn into_soft_mac(self) -> Result<Efr32Mac, MacError> {
        self.into_soft_mac_with_address(Self::read_factory_ieee())
    }

    /// Build the end-device [`Efr32Mac`] with an explicit EUI-64 (lab
    /// identities). `macDSN`/`macBSN` start at 0 as in the legacy backend.
    #[inline(never)]
    pub fn into_soft_mac_with_address(self, ieee: IeeeAddress) -> Result<Efr32Mac, MacError> {
        SoftMacCore::new_custom(self, MacPib::new(ieee, 0, 0))
    }

    /// Print a compact radio snapshot (`efr32-trace`).
    pub fn debug_radio_snapshot(&self, tag: &str) {
        self.driver.debug_snapshot(tag);
    }

    /// FRC-IRQ software-ACK counters `(started, completed, failed, dsn, rac)`.
    pub fn software_ack_snapshot(&self) -> (u32, u32, u32, u8, u8) {
        self.driver.software_ack_snapshot()
    }

    /// Transmit a raw frame without CCA for bring-up diagnostics.
    pub async fn debug_transmit_raw(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        self.rx_live = false;
        self.driver.transmit(frame).await.map_err(map_radio_error)?;
        self.rx_live = true;
        Ok(())
    }
}

impl Default for Efr32RadioPhy {
    fn default() -> Self {
        Self::new()
    }
}

impl RadioPhy for Efr32RadioPhy {
    /// One CCA + TX in the PHY; the FRC IRQ matches the Imm-Ack by DSN.
    type Tx = AckWaitTx;
    /// Hardware-proven legacy poll window.
    const POLL_RESPONSE_WAIT_US: u32 = 1_500_000;
    /// EmberZNet parents queue Rejoin Response / Transport-Key by EUI-64.
    const POLL_EXTENDED_FALLBACK: bool = true;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            // The FRC IRQ acknowledges matching AR frames (never with FP).
            hardware_auto_ack: true,
            hardware_address_filter: false,
            tx_power_min: -20,
            tx_power_max: 19,
        }
    }

    #[allow(clippy::manual_async_fn)] // avoids a second copy of the arguments
    fn try_transmit(&mut self, frame: &[u8]) -> impl Future<Output = Result<(), PhyError>> {
        async move {
            self.rx_live = false;
            if self
                .driver
                .clear_channel_assessment()
                .await
                .map_err(map_radio_error)?
            {
                return Err(PhyError::ChannelBusy);
            }
            // `transmit()` re-arms RX immediately after TX_DONE.
            self.driver.transmit(frame).await.map_err(map_radio_error)?;
            self.rx_live = true;
            Ok(())
        }
    }

    /// Legacy single ACK wait: the first completed reception decides.
    ///
    /// The FRC IRQ already drops ACKs for other DSNs. A non-ACK frame that
    /// completes inside the 1.5 ms window ends the wait as "no ACK" and is
    /// not retained (the single-slot ISR buffer is reused by the retry), as
    /// in the legacy backend.
    #[allow(clippy::manual_async_fn)] // avoids a second copy of the arguments
    fn try_transmit_acknowledged(
        &mut self,
        frame: &[u8],
    ) -> impl Future<Output = Result<Option<AckResult>, PhyError>> {
        async move {
            self.try_transmit(frame).await?;
            if with_timeout(ACK_WAIT, self.driver.wait_rx_done())
                .await
                .is_err()
            {
                return Ok(None);
            }
            match self
                .driver
                .read_rx_frame(|data, _| crate::frames::ack_info(data))
            {
                Ok(ack) => {
                    self.driver.rearm_rx();
                    Ok(matching_ack(ack, frame[2]))
                }
                Err(_) => {
                    self.rx_live = false;
                    Ok(None)
                }
            }
        }
    }

    /// Never called: the FRC IRQ sends every Imm-Ack (`hardware_auto_ack`).
    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    #[allow(clippy::manual_async_fn)] // avoids a second copy of the arguments
    fn receive(
        &mut self,
        timeout_us: u32,
        frame: &mut PhyRxFrame,
    ) -> impl Future<Output = Result<bool, PhyError>> {
        async move {
            let live = self.rx_live;
            // A reception completed while nobody was listening is still in
            // the single-slot ISR buffer.
            if live || !self.driver.take_rx_done() {
                if !live {
                    self.driver.restart_rx().map_err(map_radio_error)?;
                }
                if with_timeout(
                    Duration::from_micros(u64::from(timeout_us)),
                    self.driver.wait_rx_done(),
                )
                .await
                .is_err()
                {
                    self.rx_live = false;
                    return Ok(false);
                }
            }
            // `read_rx_frame` yields at most 125 MAC bytes, so `set` cannot
            // fail.
            match self
                .driver
                .read_rx_frame(|data, rssi| frame.set(data, crate::lqi::from_rssi_dbm(rssi)))
            {
                Ok(Ok(())) => {
                    if live {
                        self.driver.rearm_rx();
                    }
                    Ok(true)
                }
                Ok(Err(error)) => {
                    self.rx_live = false;
                    Err(error)
                }
                Err(error) => {
                    self.rx_live = false;
                    Err(map_radio_error(error))
                }
            }
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !(11..=26).contains(&channel) {
            return Err(PhyError::InvalidChannel);
        }
        if self.driver.retune(channel) {
            self.rx_live = false;
        }
        Ok(())
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        if !(-20..=19).contains(&dbm) {
            return Err(PhyError::Unsupported);
        }
        self.driver.set_power(dbm);
        Ok(())
    }

    /// One CCA-style RSSI sample; `duration_us` is not honoured.
    async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
        self.rx_live = false;
        self.driver
            .clear_channel_assessment()
            .await
            .map_err(map_radio_error)?;
        Ok(energy_from_rssi(self.driver.last_cca_rssi()))
    }

    /// Feeds the FRC-IRQ software-ACK filter; `None` (scan) keeps the last
    /// addresses and only disables the ACK.
    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        let config = self.driver.config();
        let (pan_id, short_address, extended_address) = match filter {
            Some(f) => (f.pan_id.0, f.short_address.0, f.extended_address),
            None => (config.pan_id, config.short_address, config.extended_address),
        };
        let promiscuous = filter.is_none();
        if (
            config.promiscuous,
            config.pan_id,
            config.short_address,
            config.extended_address,
        ) != (promiscuous, pan_id, short_address, extended_address)
        {
            self.driver
                .set_ack_filter(promiscuous, pan_id, short_address, extended_address);
        }
        Ok(())
    }

    /// Force the radio idle and gate its clocks.
    fn sleep(&mut self) -> Result<(), PhyError> {
        self.rx_live = false;
        self.driver.radio_sleep();
        Ok(())
    }

    /// Restore the radio clocks and configuration.
    fn wake(&mut self) -> Result<(), PhyError> {
        self.driver.radio_wake();
        Ok(())
    }
}

#[cfg(any(not(feature = "hardware-aes-efr32mg1"), not(target_arch = "arm")))]
impl zigbee_crypto::ForwardAesProvider for Efr32RadioPhy {}

#[cfg(all(feature = "hardware-aes-efr32mg1", target_arch = "arm"))]
impl zigbee_crypto::ForwardAesProvider for Efr32RadioPhy {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        let engine = self
            .aes_engine
            .as_mut()
            .expect("AES engine not installed: call Efr32RadioPhy::install_aes_engine()");
        zigbee_crypto::efr32mg1::HardwareAes128::new(engine, *key)
    }
}

impl PlatformServices for Efr32RadioPhy {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, _output: &mut [u8]) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }
}

/// EFR32MG1 end-device MAC used by `efr32mg1-sensor` and the lab binaries.
pub type Efr32Mac = Efr32SoftMac;

fn map_radio_error(error: RadioError) -> PhyError {
    match error {
        RadioError::CcaFailure => PhyError::ChannelBusy,
        RadioError::InvalidFrame => PhyError::FrameTooLong,
        RadioError::CrcError | RadioError::RxTimeout => PhyError::CrcFailed,
        RadioError::HardwareError | RadioError::NotInitialized => PhyError::Hardware,
    }
}

/// Frame Pending of the ACK matching `sequence`, from [`crate::frames::ack_info`].
fn matching_ack(ack: Option<(u8, bool)>, sequence: u8) -> Option<AckResult> {
    match ack {
        Some((ack_sequence, frame_pending)) if ack_sequence == sequence => {
            Some(AckResult { frame_pending })
        }
        _ => None,
    }
}

/// Legacy ED mapping: dBm + 100, clamped to the IEEE ED byte.
fn energy_from_rssi(rssi_dbm: i8) -> u8 {
    (i16::from(rssi_dbm) + 100).clamp(0, 255) as u8
}

/// DEVINFO.UNIQUEL/H little-endian; erased or blank DEVINFO yields the
/// fallback EUI-64.
fn ieee_from_devinfo(lo: u32, hi: u32) -> IeeeAddress {
    let mut eui64 = [0u8; 8];
    eui64[..4].copy_from_slice(&lo.to_le_bytes());
    eui64[4..].copy_from_slice(&hi.to_le_bytes());
    if eui64 == [0xFF; 8] || eui64 == [0x00; 8] {
        FALLBACK_IEEE
    } else {
        eui64
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{ack_info, build_ack};

    #[test]
    fn ack_matching_requires_sequence_and_reports_frame_pending() {
        assert_eq!(
            matching_ack(ack_info(&build_ack(7, false)), 7),
            Some(AckResult {
                frame_pending: false
            })
        );
        assert_eq!(
            matching_ack(ack_info(&build_ack(7, true)), 7),
            Some(AckResult {
                frame_pending: true
            })
        );
        assert_eq!(matching_ack(ack_info(&build_ack(8, true)), 7), None);
        // A data frame with the same DSN is not an ACK.
        assert_eq!(matching_ack(ack_info(&[0x41, 0x88, 7]), 7), None);
        assert_eq!(matching_ack(ack_info(&[0x02, 0x00]), 7), None);
    }

    #[test]
    fn devinfo_eui64_is_little_endian_with_blank_fallback() {
        assert_eq!(
            ieee_from_devinfo(0xFEDA_F0F1, 0x020D_6FFF),
            [0xF1, 0xF0, 0xDA, 0xFE, 0xFF, 0x6F, 0x0D, 0x02]
        );
        assert_eq!(ieee_from_devinfo(0, 0), FALLBACK_IEEE);
        assert_eq!(ieee_from_devinfo(u32::MAX, u32::MAX), FALLBACK_IEEE);
    }

    #[test]
    fn energy_detect_scale_matches_legacy_mapping() {
        assert_eq!(energy_from_rssi(-128), 0);
        assert_eq!(energy_from_rssi(-100), 0);
        assert_eq!(energy_from_rssi(-60), 40);
        assert_eq!(energy_from_rssi(127), 227);
    }

    #[test]
    fn radio_errors_map_to_phy_errors() {
        assert_eq!(map_radio_error(RadioError::CrcError), PhyError::CrcFailed);
        assert_eq!(map_radio_error(RadioError::RxTimeout), PhyError::CrcFailed);
        assert_eq!(
            map_radio_error(RadioError::HardwareError),
            PhyError::Hardware
        );
        assert_eq!(
            map_radio_error(RadioError::CcaFailure),
            PhyError::ChannelBusy
        );
    }
}
