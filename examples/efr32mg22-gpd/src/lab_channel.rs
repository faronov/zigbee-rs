//! LAB ONLY (`lab-gpd-channel`, never a product image): transmit on the
//! channel `ZIGBEE_LAB_CHANNEL` (11–26, set at build time) instead of the
//! commissioned one, without touching the GPD journal.
//!
//! A GPD commissioned into one network (e.g. ZHA on channel 15) can then
//! drive a sink in a lab network on another channel that was paired with it
//! remotely (GP Pairing Configuration from a commissioning tool), and the
//! regular image resumes on its commissioned channel afterwards. The radio
//! wrapper pins every `set_channel`, so a commissioning scan (BTN0 long
//! press) would search only the lab channel and persist whatever channel the
//! scan loop reports: do not recommission with this image.
//!
//! Without the feature `Phy` is the board's radio and `wrap` the identity.

#[cfg(not(feature = "lab-gpd-channel"))]
use zigbee_mac::efr32s2::Efr32s2RadioPhy;

#[cfg(not(feature = "lab-gpd-channel"))]
pub type Phy = Efr32s2RadioPhy;

#[cfg(not(feature = "lab-gpd-channel"))]
#[inline(always)]
pub fn wrap(phy: Efr32s2RadioPhy) -> Phy {
    phy
}

#[cfg(feature = "lab-gpd-channel")]
pub use pinned::{Phy, wrap};

#[cfg(feature = "lab-gpd-channel")]
mod pinned {
    use zigbee_mac::efr32s2::Efr32s2RadioPhy;
    use zigbee_mac::{
        AckResult, ForwardAesProvider, MacError, PhyAddressFilter, PhyCapabilities, PhyError,
        PhyRxFrame, PlatformServices, RadioPhy, TxParams,
    };

    /// The lab channel (build-time `ZIGBEE_LAB_CHANNEL`).
    const LAB_CHANNEL: u8 = match option_env!("ZIGBEE_LAB_CHANNEL") {
        Some(value) => {
            let digits = value.as_bytes();
            assert!(
                digits.len() == 2 && digits[0].is_ascii_digit() && digits[1].is_ascii_digit(),
                "ZIGBEE_LAB_CHANNEL: a channel number 11..=26"
            );
            let channel = (digits[0] - b'0') * 10 + (digits[1] - b'0');
            assert!(
                matches!(channel, 11..=26),
                "ZIGBEE_LAB_CHANNEL: a channel number 11..=26"
            );
            channel
        }
        None => panic!("lab-gpd-channel needs ZIGBEE_LAB_CHANNEL=11..26 at build time"),
    };

    /// The board radio with every channel change pinned to [`LAB_CHANNEL`].
    pub struct Phy(Efr32s2RadioPhy);

    pub fn wrap(mut phy: Efr32s2RadioPhy) -> Phy {
        if let Err(error) = phy.set_channel(LAB_CHANNEL) {
            log::error!("[GPD] lab channel {LAB_CHANNEL}: {error:?}");
        }
        log::warn!("[GPD] LAB build: every frame on channel {LAB_CHANNEL} (journal unchanged)");
        Phy(phy)
    }

    impl RadioPhy for Phy {
        type Tx = <Efr32s2RadioPhy as RadioPhy>::Tx;
        const POLL_RESPONSE_WAIT_US: u32 = <Efr32s2RadioPhy as RadioPhy>::POLL_RESPONSE_WAIT_US;
        const POLL_EXTENDED_FALLBACK: bool = <Efr32s2RadioPhy as RadioPhy>::POLL_EXTENDED_FALLBACK;
        const DEFAULT_TX_POWER_DBM: i8 = <Efr32s2RadioPhy as RadioPhy>::DEFAULT_TX_POWER_DBM;

        fn poll_response_wait_us(&self) -> u32 {
            self.0.poll_response_wait_us()
        }
        fn capabilities(&self) -> PhyCapabilities {
            self.0.capabilities()
        }
        async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
            self.0.try_transmit(frame).await
        }
        async fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), PhyError> {
            self.0.send_ack(sequence, frame_pending).await
        }
        async fn receive(
            &mut self,
            timeout_us: u32,
            frame: &mut PhyRxFrame,
        ) -> Result<bool, PhyError> {
            self.0.receive(timeout_us, frame).await
        }
        fn set_channel(&mut self, _channel: u8) -> Result<(), PhyError> {
            self.0.set_channel(LAB_CHANNEL)
        }
        fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
            self.0.set_tx_power(dbm)
        }
        async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError> {
            self.0.energy_detect(duration_us).await
        }
        fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
            self.0.set_address_filter(filter)
        }
        async fn try_transmit_acknowledged(
            &mut self,
            frame: &[u8],
        ) -> Result<Option<AckResult>, PhyError> {
            self.0.try_transmit_acknowledged(frame).await
        }
        async fn transmit_offloaded(
            &mut self,
            frame: &[u8],
            ack_requested: bool,
            params: TxParams,
        ) -> Result<Option<AckResult>, PhyError> {
            self.0
                .transmit_offloaded(frame, ack_requested, params)
                .await
        }
        fn set_rx_on_when_idle(&mut self, enabled: bool) {
            self.0.set_rx_on_when_idle(enabled)
        }
        fn sleep(&mut self) -> Result<(), PhyError> {
            self.0.sleep()
        }
        fn wake(&mut self) -> Result<(), PhyError> {
            self.0.wake()
        }
        fn default_tx_power_dbm(&self) -> i8 {
            self.0.default_tx_power_dbm()
        }
    }

    // The board radio's hardware AES engine (CRYPTOACC); no software AES.
    impl ForwardAesProvider for Phy {
        #[allow(refining_impl_trait)]
        fn forward_cipher(&mut self, key: &[u8; 16]) -> zigbee_mac::efr32s2::Efr32s2Aes128<'_> {
            self.0.forward_cipher(key)
        }
    }

    impl PlatformServices for Phy {
        fn monotonic_micros(&self) -> u32 {
            self.0.monotonic_micros()
        }
        async fn delay_micros(&mut self, duration_us: u32) {
            self.0.delay_micros(duration_us).await
        }
        fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
            self.0.fill_random(output)
        }
    }
}
