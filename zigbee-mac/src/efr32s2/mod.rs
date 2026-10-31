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
//! - **ACK** — selected by the [`AckMode`] type parameter:
//!   - [`HalAck`] (end devices): the HAL acknowledges matching frames itself
//!     while [`RadioPhy::receive`] polls it (RX→TX turnaround ≈ 85 µs on
//!     xG22), so the PHY reports `hardware_auto_ack`;
//!   - [`SoftwareAck`] (parents, [`Efr32s2RouterMac`]): the HAL never ACKs;
//!     `SoftMacCore` sends each Imm-Ack through [`RadioPhy::send_ack`] with
//!     the Frame Pending bit from its own pending table, which is what a
//!     polling child needs. A frame that already waited longer than
//!     [`SOFTWARE_ACK_DEADLINE_US`] in the RX buffer is dropped unacknowledged
//!     so the sender retries instead of seeing a late ACK.
//!
//!   In both modes frames are only acknowledged while a receive window is
//!   being polled.
//! - **RX** — polled from the executor (`yield_now`) inside the receive
//!   window; the radio has no RX interrupt path yet.
//!
//! Validation: on EFR32MG22 (WSTK + BRD4183A) the sleepy sensor joins ZHA,
//! is interviewed and reports through this PHY. On EFR32MG21 (Home Assistant
//! Connect ZBT-1) the HAL is lab-tested: raw TX/RX, beacons, CCA/ED and ACK
//! exchange with the ZHA coordinator.
//!
//! The board/product selects the default `phyTransmitPower` with the const
//! parameter of [`Efr32s2RadioPhy`] (e.g. a 20 dBm-capable xG21 module that
//! is certified for less).

use efr32s2_hal::radio::{self, AckFilter, Radio, RadioError, RadioToken};
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Timer};

use crate::pib::{MacPib, PibAttribute, PibValue};
use crate::{
    AckResult, AckWaitTx, MAX_PHY_FRAME_LEN, MacError, MacRole, Parent, PhyAddressFilter,
    PhyCapabilities, PhyError, PhyRxFrame, PlatformServices, RadioPhy, SoftMacCore,
};
use zigbee_types::IeeeAddress;

const _: () = assert!(radio::MAX_FRAME == MAX_PHY_FRAME_LEN);

/// Latest point after the end of a received frame at which a software
/// Imm-Ack is still sent: macAckWaitDuration (864 µs) minus the SHR, the HAL
/// RX→TX turnaround, the receive-path processing and one RTCC tick of
/// measurement uncertainty, with margin.
pub const SOFTWARE_ACK_DEADLINE_US: u64 = 400;

mod ack_sealed {
    pub trait Sealed {}
}

/// How [`Efr32s2RadioPhy`] acknowledges received frames. Sealed; see
/// [`HalAck`] and [`SoftwareAck`].
pub trait AckMode: ack_sealed::Sealed {
    #[doc(hidden)]
    const SOFTWARE: bool;
    #[doc(hidden)]
    type State: Default;
    #[doc(hidden)]
    fn soft(state: &mut Self::State) -> Option<&mut SoftAckState>;
    #[doc(hidden)]
    fn soft_ref(state: &Self::State) -> Option<&SoftAckState>;
}

/// End-device ACK mode: the HAL auto-ACKs (Frame Pending always clear) and
/// the PHY reports `hardware_auto_ack`. Zero-sized; the default.
pub struct HalAck;

/// Parent ACK mode: `SoftMacCore` sends every Imm-Ack with its own Frame
/// Pending decision; stale frames are dropped (see the module docs).
pub struct SoftwareAck;

/// Per-PHY state of [`SoftwareAck`].
#[doc(hidden)]
#[derive(Default)]
pub struct SoftAckState {
    filter: AckFilter,
    /// Time of the most recent empty RX-buffer poll while receiving.
    last_idle_us: u64,
    stale_dropped: u32,
}

impl ack_sealed::Sealed for HalAck {}
impl ack_sealed::Sealed for SoftwareAck {}

impl AckMode for HalAck {
    const SOFTWARE: bool = false;
    type State = ();
    #[inline(always)]
    fn soft(_: &mut ()) -> Option<&mut SoftAckState> {
        None
    }
    #[inline(always)]
    fn soft_ref(_: &()) -> Option<&SoftAckState> {
        None
    }
}

impl AckMode for SoftwareAck {
    const SOFTWARE: bool = true;
    type State = SoftAckState;
    #[inline(always)]
    fn soft(state: &mut SoftAckState) -> Option<&mut SoftAckState> {
        Some(state)
    }
    #[inline(always)]
    fn soft_ref(state: &SoftAckState) -> Option<&SoftAckState> {
        Some(state)
    }
}

/// EFR32 Series 2 radio adapted to [`RadioPhy`].
///
/// `DEFAULT_TX_DBM` is the product's `phyTransmitPower` after construction
/// and after every MLME-RESET; it must lie in
/// [`radio::TX_POWER_RANGE_DBM`] (checked at compile time). `A` selects the
/// [`AckMode`]: [`HalAck`] for end devices, [`SoftwareAck`] for parents.
pub struct Efr32s2RadioPhy<const DEFAULT_TX_DBM: i8 = 0, A: AckMode = HalAck> {
    radio: Radio,
    ieee: IeeeAddress,
    rx_on_when_idle: bool,
    /// Set by [`RadioPhy::sleep`]; [`RadioPhy::wake`] then re-initializes
    /// the radio, which EM2 requires.
    asleep: bool,
    /// Most recent HAL error other than `NoAck`/`ChannelBusy` (diagnostics).
    last_error: Option<RadioError>,
    ack: A::State,
}

/// The EFR32 Series 2 MAC: the shared software MAC over [`Efr32s2RadioPhy`].
pub type Efr32s2SoftMac<const DEFAULT_TX_DBM: i8 = 0> =
    SoftMacCore<Efr32s2RadioPhy<DEFAULT_TX_DBM>>;

/// Parent role of an EFR32 Series 2 router: eight queued MAC command events,
/// eight concurrent Association Response transactions and eight children
/// with Frame Pending armed (the core's software table, used because
/// [`SoftwareAck`] sends the Imm-Ack).
pub type Efr32s2RouterRole = Parent<8, 8, 8>;

/// Retained-RX depth of an always-on EFR32 Series 2 router: it hears child
/// and neighbour traffic between runtime receive calls.
pub const EFR32S2_ROUTER_PENDING_RX: usize = 8;

/// The EFR32 Series 2 router MAC: the shared software MAC in the parent role
/// over a [`SoftwareAck`] PHY.
pub type Efr32s2RouterMac<const DEFAULT_TX_DBM: i8 = 0> = SoftMacCore<
    Efr32s2RadioPhy<DEFAULT_TX_DBM, SoftwareAck>,
    Efr32s2RouterRole,
    EFR32S2_ROUTER_PENDING_RX,
>;

impl<const D: i8, A: AckMode> Efr32s2RadioPhy<D, A> {
    /// Program the 802.15.4 PHY. The HFXO must already be running.
    ///
    /// Returns [`PhyError::Hardware`] if the radio does not come up.
    pub fn new(token: RadioToken) -> Result<Self, PhyError> {
        let radio = token.init().map_err(map_radio_error)?;
        Ok(Self {
            radio,
            ieee: efr32s2_hal::devinfo::eui64(),
            rx_on_when_idle: false,
            asleep: false,
            last_error: None,
            ack: A::State::default(),
        })
    }

    /// Factory EUI-64 (DEVINFO), over-the-air byte order.
    pub fn extended_address(&self) -> IeeeAddress {
        self.ieee
    }

    /// Frames dropped unacknowledged because they waited too long in the RX
    /// buffer for a software ACK ([`SoftwareAck`] only; always 0 otherwise).
    pub fn stale_ack_drops(&self) -> u32 {
        A::soft_ref(&self.ack).map_or(0, |state| state.stale_dropped)
    }

    /// Radio debug counters.
    pub fn radio_stats(&self) -> radio::RadioStats {
        self.radio.stats()
    }

    /// Most recent hardware-level radio error (not `NoAck`/`ChannelBusy`)
    /// and the current radio state machine state, for diagnostics.
    pub fn last_error(&self) -> (Option<RadioError>, radio::RacState) {
        (self.last_error, self.radio.rac_state())
    }

    fn fail(&mut self, error: RadioError) -> PhyError {
        if !matches!(error, RadioError::NoAck | RadioError::ChannelBusy) {
            self.last_error = Some(error);
            log::warn!(
                "efr32s2: {:?} (rac {:?}, synth {:?})",
                error,
                self.radio.rac_state(),
                radio::synth_diag()
            );
        }
        map_radio_error(error)
    }

    fn ensure_rx(&mut self) -> Result<(), PhyError> {
        if self.radio.is_rx_on() {
            Ok(())
        } else {
            self.radio.rx_on().map_err(|e| self.fail(e))
        }
    }

    /// Drop out of RX after an operation unless `macRxOnWhenIdle` is set.
    fn settle(&mut self) {
        if !self.rx_on_when_idle {
            self.radio.idle();
        }
    }
}

/// Initial PIB with the board TX power. A macro rather than a helper so the
/// PIB is built in place in the out-of-line constructors (a by-value
/// `Result<MacPib>` return costs a frame copy).
macro_rules! initial_pib {
    ($phy:expr, $d:expr) => {{
        const {
            assert!(
                $d >= radio::TX_POWER_RANGE_DBM.0 && $d <= radio::TX_POWER_RANGE_DBM.1,
                "default TX power outside the chip's PA range"
            )
        };
        let mut pib = MacPib::new($phy.ieee, 0, 0);
        pib.set(PibAttribute::PhyTransmitPower, PibValue::I8($d))
            .map_err(|_| MacError::InvalidParameter)?;
        pib
    }};
}

impl<const D: i8> Efr32s2RadioPhy<D> {
    /// Wrap in the shared software MAC. Kept out of line so the PHY and the
    /// MAC are not both live in the caller's frame.
    #[inline(never)]
    pub fn into_soft_mac(self) -> Result<Efr32s2SoftMac<D>, MacError> {
        let pib = initial_pib!(self, D);
        SoftMacCore::new(self, pib)
    }
}

impl<const D: i8> Efr32s2RadioPhy<D, SoftwareAck> {
    /// Wrap in the shared software MAC in a parent role, normally
    /// [`Efr32s2RouterMac`]. Kept out of line like
    /// [`Efr32s2RadioPhy::into_soft_mac`].
    #[inline(never)]
    pub fn into_parent_mac<R: MacRole, const RX: usize>(
        self,
    ) -> Result<SoftMacCore<Self, R, RX>, MacError> {
        let pib = initial_pib!(self, D);
        SoftMacCore::new_custom(self, pib)
    }
}

impl<const D: i8, A: AckMode> RadioPhy for Efr32s2RadioPhy<D, A> {
    type Tx = AckWaitTx;

    /// Board-selected `phyTransmitPower` after construction and every
    /// MLME-RESET (the HAL itself powers up at 0 dBm).
    const DEFAULT_TX_POWER_DBM: i8 = D;

    /// EmberZNet parents may hold the indirect frame behind other traffic;
    /// same value as the hardware-proven nRF and PHY62x2 backends.
    const POLL_RESPONSE_WAIT_US: u32 = 1_500_000;
    /// EmberZNet parents queue Rejoin Response / Transport-Key by EUI-64.
    const POLL_EXTENDED_FALLBACK: bool = true;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            hardware_auto_ack: !A::SOFTWARE,
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
        result.map_err(|e| self.fail(e))
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
                Err(self.fail(error))
            }
        }
    }

    /// [`SoftwareAck`]: transmit the Imm-Ack the core decided on, right
    /// after the frame was returned by [`Self::receive`]. [`HalAck`] never
    /// gets here (`hardware_auto_ack` is set).
    async fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), PhyError> {
        if !A::SOFTWARE {
            return Err(PhyError::Unsupported);
        }
        self.radio
            .send_ack(sequence, frame_pending)
            .map_err(|e| self.fail(e))
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        // Compile-time false for `HalAck`: no extra register read there.
        let was_on = A::SOFTWARE && self.radio.is_rx_on();
        self.ensure_rx()?;
        if !was_on && let Some(state) = A::soft(&mut self.ack) {
            // Nothing in the buffer can be older than switching RX on.
            state.last_idle_us = Instant::now().as_micros();
        }
        let deadline = Instant::now() + Duration::from_micros(u64::from(timeout_us));
        loop {
            if let Some(rx) = self.radio.receive(frame.buffer_mut()) {
                if let Some(state) = A::soft(&mut self.ack)
                    && software_ack_too_late(
                        state,
                        &frame.buffer_mut()[..rx.len],
                        Instant::now().as_micros(),
                    )
                {
                    state.stale_dropped = state.stale_dropped.wrapping_add(1);
                    continue;
                }
                frame.set_len(rx.len, rx.lqi)?;
                return Ok(true);
            }
            if let Some(state) = A::soft(&mut self.ack) {
                state.last_idle_us = Instant::now().as_micros();
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
        self.radio.set_channel(channel).map_err(|e| self.fail(e))
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        self.radio.set_tx_power_ddbm(i16::from(dbm) * 10);
        Ok(())
    }

    async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError> {
        let rssi = self
            .radio
            .energy_detect(duration_us)
            .map_err(|e| self.fail(e))?;
        self.settle();
        Ok(rssi.map_or(0, crate::lqi::from_rssi_dbm))
    }

    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        let filter = ack_filter(filter);
        match A::soft(&mut self.ack) {
            // The HAL filter stays empty, so the HAL never ACKs on its own.
            Some(state) => state.filter = filter,
            None => self.radio.set_ack_filter(filter),
        }
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

    /// Radio idle. The platform may then enter EM2; [`RadioPhy::wake`]
    /// must run before the next radio operation.
    fn sleep(&mut self) -> Result<(), PhyError> {
        self.radio.idle();
        self.asleep = true;
        Ok(())
    }

    /// Re-initialize the radio after [`RadioPhy::sleep`] (`Radio::resume`,
    /// about 95 µs on xG22). EM2 keeps the PHY register file but the
    /// receiver stays dead without it (lab gate `s2-em2`); the VCO
    /// calibration cache, channel and TX power are kept.
    fn wake(&mut self) -> Result<(), PhyError> {
        if self.asleep {
            self.radio.resume().map_err(|e| self.fail(e))?;
            self.asleep = false;
        }
        if self.rx_on_when_idle {
            self.radio.rx_on().map_err(|e| self.fail(e))?;
        }
        Ok(())
    }
}

/// Software AES: no EFR32 Series 2 crypto engine backend yet.
impl<const D: i8, A: AckMode> zigbee_crypto::ForwardAesProvider for Efr32s2RadioPhy<D, A> {}

impl<const D: i8, A: AckMode> PlatformServices for Efr32s2RadioPhy<D, A> {
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

/// True when `frame` needs an ACK from us but has waited in the RX buffer
/// longer than [`SOFTWARE_ACK_DEADLINE_US`]: it completed after the last
/// empty poll, so `now - last_idle` bounds its age.
fn software_ack_too_late(state: &SoftAckState, frame: &[u8], now_us: u64) -> bool {
    radio::needs_ack(frame, &state.filter)
        && now_us.saturating_sub(state.last_idle_us) > SOFTWARE_ACK_DEADLINE_US
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
    fn late_software_ack_frames_are_dropped_only_when_an_ack_is_due() {
        let state = SoftAckState {
            filter: filter(0x1234),
            last_idle_us: 1_000,
            stale_dropped: 0,
        };
        let to_us = [0x61, 0x88, 0x10, 0xE9, 0xDF, 0x34, 0x12, 0x00, 0x00];
        assert!(!software_ack_too_late(
            &state,
            &to_us,
            1_000 + SOFTWARE_ACK_DEADLINE_US
        ));
        assert!(software_ack_too_late(
            &state,
            &to_us,
            1_001 + SOFTWARE_ACK_DEADLINE_US
        ));
        // Broadcasts and frames for others never need our ACK: never stale.
        let broadcast = [0x41, 0x88, 0x10, 0xE9, 0xDF, 0xFF, 0xFF, 0x00, 0x00];
        assert!(!software_ack_too_late(&state, &broadcast, 1_000_000));
        let other = [0x61, 0x88, 0x10, 0xE9, 0xDF, 0x21, 0x43, 0x00, 0x00];
        assert!(!software_ack_too_late(&state, &other, 1_000_000));
    }

    #[test]
    fn router_mac_is_a_software_ack_parent() {
        fn parent<T: crate::ParentMacDriver>() {}
        parent::<Efr32s2RouterMac<10>>();
        const { assert!(SoftwareAck::SOFTWARE && !HalAck::SOFTWARE) };
        assert_eq!(core::mem::size_of::<<HalAck as AckMode>::State>(), 0);
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
