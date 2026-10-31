//! EFR32 Series 2 (EFR32MG21 / EFR32MG22) [`RadioPhy`] for [`SoftMacCore`].
//!
//! The radio itself is `efr32s2_hal::radio`: a pure-Rust driver that programs
//! RAC/FRC/MODEM/AGC/SYNTH/BUFC directly from GSDK-verified register values
//! and replaces RAIL's sequencer microcode with CPU code. No RAIL library or
//! vendor blob is linked. This module only adapts that driver to the shared
//! software MAC:
//!
//! - **TX** — [`AckWaitTx`]: one CCA, TX and the ACK wait per attempt
//!   (`Radio::transmit_cca`); CSMA-CA backoff and retries stay in
//!   `SoftMacCore`.
//! - **ACK** — the HAL acknowledges matching frames itself while
//!   [`RadioPhy::receive`] polls it (RX→TX turnaround ≈ 85 µs on xG22), so the
//!   PHY reports `hardware_auto_ack`. Frames are only acknowledged while a
//!   receive window is being polled.
//! - **RX** — polled from the executor (`yield_now`) inside the receive
//!   window; the radio has no RX interrupt path yet.
//!
//! Validation: on EFR32MG22 (BRD4183A) the HAL is hardware-tested through raw
//! TX/RX, beacons, CCA/ED and ACK exchange with a ZHA coordinator. The xG21
//! register tables are not populated yet, so on EFR32MG21
//! [`Efr32s2RadioPhy::new`] returns [`PhyError::Unsupported`].

use efr32s2_hal::radio::{self, AckFilter, Radio, RadioError, RadioToken};
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Timer};

use crate::pib::MacPib;
use crate::{
    AckResult, AckWaitTx, MAX_PHY_FRAME_LEN, MacError, PhyAddressFilter, PhyCapabilities, PhyError,
    PhyRxFrame, PlatformServices, RadioPhy, SoftMacCore,
};
use zigbee_types::IeeeAddress;

const _: () = assert!(radio::MAX_FRAME == MAX_PHY_FRAME_LEN);

/// EFR32 Series 2 radio adapted to [`RadioPhy`].
pub struct Efr32s2RadioPhy {
    radio: Radio,
    ieee: IeeeAddress,
    rx_on_when_idle: bool,
}

/// The EFR32 Series 2 MAC: the shared software MAC over [`Efr32s2RadioPhy`].
pub type Efr32s2SoftMac = SoftMacCore<Efr32s2RadioPhy>;

impl Efr32s2RadioPhy {
    /// Program the 802.15.4 PHY. The HFXO must already be running.
    ///
    /// Returns [`PhyError::Unsupported`] on a chip without validated radio
    /// tables (xG21 today) and [`PhyError::Hardware`] if the radio does not
    /// come up.
    pub fn new(token: RadioToken) -> Result<Self, PhyError> {
        let radio = token.init().map_err(map_radio_error)?;
        Ok(Self {
            radio,
            ieee: efr32s2_hal::devinfo::eui64(),
            rx_on_when_idle: false,
        })
    }

    /// Factory EUI-64 (DEVINFO), over-the-air byte order.
    pub fn extended_address(&self) -> IeeeAddress {
        self.ieee
    }

    /// Wrap in the shared software MAC. Kept out of line so the PHY and the
    /// MAC are not both live in the caller's frame.
    #[inline(never)]
    pub fn into_soft_mac(self) -> Result<Efr32s2SoftMac, MacError> {
        let pib = MacPib::new(self.ieee, 0, 0);
        SoftMacCore::new(self, pib)
    }

    /// Radio debug counters.
    pub fn radio_stats(&self) -> radio::RadioStats {
        self.radio.stats()
    }

    fn ensure_rx(&mut self) -> Result<(), PhyError> {
        if self.radio.is_rx_on() {
            Ok(())
        } else {
            self.radio.rx_on().map_err(map_radio_error)
        }
    }

    /// Drop out of RX after an operation unless `macRxOnWhenIdle` is set.
    fn settle(&mut self) {
        if !self.rx_on_when_idle {
            self.radio.idle();
        }
    }
}

impl RadioPhy for Efr32s2RadioPhy {
    type Tx = AckWaitTx;

    /// EmberZNet parents may hold the indirect frame behind other traffic;
    /// same value as the hardware-proven nRF and PHY62x2 backends.
    const POLL_RESPONSE_WAIT_US: u32 = 1_500_000;
    /// EmberZNet parents queue Rejoin Response / Transport-Key by EUI-64.
    const POLL_EXTENDED_FALLBACK: bool = true;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            hardware_auto_ack: true,
            hardware_address_filter: false,
            tx_power_min: radio::TX_POWER_RANGE_DBM.0,
            tx_power_max: radio::TX_POWER_RANGE_DBM.1,
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        // `transmit_cca` only waits for an ACK when the frame requests one;
        // the software MAC calls this path for frames that do not.
        let result = self.radio.transmit_cca(frame).map(|_| ());
        self.settle();
        result.map_err(map_radio_error)
    }

    async fn try_transmit_acknowledged(
        &mut self,
        frame: &[u8],
    ) -> Result<Option<AckResult>, PhyError> {
        match self.radio.transmit_cca(frame) {
            Ok(outcome) => {
                let ack = outcome
                    .ack_frame_pending
                    .map(|frame_pending| AckResult { frame_pending });
                // A Frame Pending ACK is followed by a receive window: keep
                // the receiver on so the indirect frame is not missed.
                if !matches!(
                    ack,
                    Some(AckResult {
                        frame_pending: true
                    })
                ) {
                    self.settle();
                }
                Ok(ack)
            }
            Err(RadioError::NoAck) => {
                self.settle();
                Ok(None)
            }
            Err(error) => {
                self.settle();
                Err(map_radio_error(error))
            }
        }
    }

    /// Never called: `hardware_auto_ack` is set and the HAL acknowledges
    /// while [`Self::receive`] polls.
    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        self.ensure_rx()?;
        let deadline = Instant::now() + Duration::from_micros(u64::from(timeout_us));
        loop {
            if let Some(rx) = self.radio.receive(frame.buffer_mut()) {
                frame.set_len(rx.len, rx.lqi)?;
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
        if self.radio.channel() == channel {
            return Ok(());
        }
        self.radio.set_channel(channel).map_err(map_radio_error)
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        self.radio.set_tx_power_ddbm(i16::from(dbm) * 10);
        Ok(())
    }

    async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError> {
        let rssi = self
            .radio
            .energy_detect(duration_us)
            .map_err(map_radio_error)?;
        self.settle();
        Ok(rssi.map_or(0, crate::lqi::from_rssi_dbm))
    }

    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        self.radio.set_ack_filter(ack_filter(filter));
        Ok(())
    }

    fn set_rx_on_when_idle(&mut self, enabled: bool) {
        self.rx_on_when_idle = enabled;
        if enabled {
            // A failure here resurfaces on the next receive or transmit.
            let _ = self.radio.rx_on();
        } else {
            self.radio.idle();
        }
    }

    fn sleep(&mut self) -> Result<(), PhyError> {
        self.radio.idle();
        Ok(())
    }

    /// The radio registers and per-channel VCO tuning survive EM2; the next
    /// operation re-arms RX or TX.
    fn wake(&mut self) -> Result<(), PhyError> {
        Ok(())
    }
}

/// Software AES: no EFR32 Series 2 crypto engine backend yet.
impl zigbee_crypto::ForwardAesProvider for Efr32s2RadioPhy {}

impl PlatformServices for Efr32s2RadioPhy {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    /// No TRNG driver yet; the HAL's xorshift is not cryptographic.
    fn fill_random(&mut self, _output: &mut [u8]) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }
}

fn ack_filter(filter: Option<PhyAddressFilter>) -> AckFilter {
    match filter {
        None => AckFilter::default(),
        Some(f) => AckFilter {
            pan_id: Some(f.pan_id.0),
            // 0xFFFE: no short address yet; 0xFFFF: broadcast, never ACKed.
            short_address: (f.short_address.0 < 0xFFFE).then_some(f.short_address.0),
            extended_address: Some(u64::from_le_bytes(f.extended_address)),
        },
    }
}

fn map_radio_error(error: RadioError) -> PhyError {
    match error {
        RadioError::Unsupported => PhyError::Unsupported,
        RadioError::InvalidChannel => PhyError::InvalidChannel,
        RadioError::FrameTooLong => PhyError::FrameTooLong,
        RadioError::ChannelBusy => PhyError::ChannelBusy,
        RadioError::Shutdown
        | RadioError::SynthLock
        | RadioError::TxTimeout
        | RadioError::NoAck => PhyError::Hardware,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use efr32s2_hal::radio::needs_ack;
    use zigbee_types::{PanId, ShortAddress};

    fn filter(short: u16) -> AckFilter {
        ack_filter(Some(PhyAddressFilter {
            pan_id: PanId(0xDFE9),
            short_address: ShortAddress(short),
            extended_address: [0x29, 0x2C, 0x6F, 0xFE, 0xFF, 0x81, 0x8E, 0x58],
        }))
    }

    #[test]
    fn promiscuous_filter_acknowledges_nothing() {
        let data = [0x61, 0x88, 0x10, 0xE9, 0xDF, 0x34, 0x12, 0x00, 0x00];
        assert!(!needs_ack(&data, &ack_filter(None)));
    }

    #[test]
    fn short_and_extended_destinations_are_acknowledged() {
        let to_short = [0x61, 0x88, 0x10, 0xE9, 0xDF, 0x34, 0x12, 0x00, 0x00];
        assert!(needs_ack(&to_short, &filter(0x1234)));
        assert!(!needs_ack(&to_short, &filter(0x4321)));

        // Association response style: extended destination, intra-PAN.
        let to_ext = [
            0x63, 0xCC, 0x11, 0xE9, 0xDF, 0x29, 0x2C, 0x6F, 0xFE, 0xFF, 0x81, 0x8E, 0x58,
        ];
        assert!(needs_ack(&to_ext, &filter(0xFFFE)));
    }

    #[test]
    fn unassigned_short_address_is_never_matched() {
        let to_fffe = [0x61, 0x88, 0x10, 0xE9, 0xDF, 0xFE, 0xFF, 0x00, 0x00];
        assert!(!needs_ack(&to_fffe, &filter(0xFFFE)));
    }

    #[test]
    fn radio_errors_map_to_phy_errors() {
        assert_eq!(
            map_radio_error(RadioError::ChannelBusy),
            PhyError::ChannelBusy
        );
        assert_eq!(
            map_radio_error(RadioError::Unsupported),
            PhyError::Unsupported
        );
        assert_eq!(map_radio_error(RadioError::SynthLock), PhyError::Hardware);
    }
}
