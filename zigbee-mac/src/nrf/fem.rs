//! nRF52833/nRF52840 RF front-end module (external PA/LNA) control.
//!
//! [`NrfFem`] programs the PPI + GPIOTE plan of [`crate::nrf_fem`]: the PA
//! (CTX) and LNA (CRX) lines follow the RADIO's own events in hardware, so
//! their timing does not depend on interrupt latency or task scheduling.
//! The optional power-down / chip-enable line (CSD, PDN) is CPU-controlled:
//! it is asserted while the radio is in use and released when the PHY goes
//! to sleep.
//!
//! [`NrfFemPhy`] attaches a FEM to either nRF PHY (`NrfRadioPhy`,
//! `NrfIsrRadioPhy`) as a [`RadioPhy`] decorator, so the PHYs themselves stay
//! unchanged without the feature. Through it
//!
//! * `phyTransmitPower` is antenna-referred: the radio is programmed with
//!   [`crate::fem::split_tx_power`]; the result is available from
//!   [`NrfFem::tx_power`] and logged;
//! * received LQI is derived from the antenna-referred level (LNA gain
//!   removed before the IEEE mapping, inside the PHY's receive path); the ISR
//!   PHY's energy CCA threshold is raised by the LNA gain, and its Imm-Ack
//!   switches the PA on at the TXEN instant (FORK of PPI channel 20);
//! * a transmission that fails or is cancelled drops PA and LNA in software
//!   ([`abort`]), so the PA is never left on without a PHYEND;
//! * `sleep()` drops both lines and releases PDN; `wake()` and every
//!   transmit/receive assert PDN again.
//!
//! **Hardware status:** register plan host-tested; not yet run on a board
//! with a FEM. See `docs/book/src/platform-guides/nrf.md`.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use super::embassy_nrf::gpio::{AnyPin, Pin, Port};
use super::embassy_nrf::ppi::{AnyConfigurableChannel, Channel as _};
use super::embassy_nrf::{Peripheral, PeripheralRef, into_ref, peripherals};
use super::{NRF_PENDING_RX, NrfRadioPhy, RadioInstance, RngInstance};
use crate::fem::{FemConfigError, FemParams, TxPowerSplit, rx_level_at_antenna, split_tx_power};
use crate::nrf_fem::{
    FemLayout, FemPin, FemPlan, NRF_ED_UNITS_PER_DB, NRF_FEM_TIMING, NRF_TX_POWER_STEPS,
    PIN_CNF_OUTPUT, PPI_CHENCLR, PPI_CHENSET, plan,
};
use crate::pib::MacPib;
use crate::{
    AckResult, EndDevice, MacError, PhyAddressFilter, PhyCapabilities, PhyError, PhyRxFrame,
    PibAttribute, PibValue, PlatformServices, RadioPhy, SoftMacCore, TxParams,
};
use zigbee_crypto::ForwardAesProvider;
use zigbee_types::{IeeeAddress, MacAddress};

/// GPIOTE task that drops the PA (0 = no FEM attached).
static PA_OFF_TASK: AtomicU32 = AtomicU32::new(0);
/// GPIOTE task that drops the LNA (0 = none).
static LNA_OFF_TASK: AtomicU32 = AtomicU32::new(0);
/// LNA gain removed from received levels.
static RX_GAIN_DB: AtomicU8 = AtomicU8::new(0);
/// One FEM per RADIO.
static ACTIVE: AtomicBool = AtomicBool::new(false);

#[inline(always)]
fn write(address: u32, value: u32) {
    // SAFETY: fixed, always-mapped GPIO/GPIOTE/PPI registers of the
    // nRF52833/nRF52840; the addresses come from a validated `FemPlan` whose
    // pins and channels are owned by the `NrfFem` (Embassy singletons).
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

/// Drop PA and LNA now (software): after a transmission that did not end
/// with PHYEND (error, cancellation) and on sleep. A no-op without an
/// attached FEM.
pub(super) fn abort() {
    for task in [&PA_OFF_TASK, &LNA_OFF_TASK] {
        let address = task.load(Ordering::Relaxed);
        if address != 0 {
            write(address, 1);
        }
    }
}

/// Antenna-referred Nordic ED/LQI byte (1 dB units): the LNA gain removed.
#[inline]
pub(super) fn rx_level(radio_level: u8) -> u8 {
    rx_level_at_antenna(radio_level, RX_GAIN_DB.load(Ordering::Relaxed))
}

mod sealed {
    pub trait Sealed {}

    /// PHY-specific FEM wiring (ISR PHY: Imm-Ack PA fork, CCA threshold).
    pub trait Host {
        fn arm_fem(&mut self, _plan: &crate::nrf_fem::FemPlan, _rx_gain_db: u8) {}
        fn disarm_fem(&mut self) {}
    }
}

/// A GPIOTE channel singleton usable as a FEM line driver.
///
/// Implemented for `GPIOTE_CH0..GPIOTE_CH7`. Taking the singleton by value
/// reserves the channel at compile time: Embassy's GPIOTE driver (and any
/// other user) can no longer claim it.
pub trait FemGpioteChannel: sealed::Sealed + 'static {
    const NUMBER: u8;
}

macro_rules! gpiote_channel {
    ($($channel:ident = $number:expr),*) => {$(
        impl sealed::Sealed for peripherals::$channel {}
        impl FemGpioteChannel for peripherals::$channel {
            const NUMBER: u8 = $number;
        }
    )*};
}

gpiote_channel!(
    GPIOTE_CH0 = 0,
    GPIOTE_CH1 = 1,
    GPIOTE_CH2 = 2,
    GPIOTE_CH3 = 3,
    GPIOTE_CH4 = 4,
    GPIOTE_CH5 = 5,
    GPIOTE_CH6 = 6,
    GPIOTE_CH7 = 7
);

/// One FEM control pin and its active level.
pub struct FemLine<'d> {
    pin: PeripheralRef<'d, AnyPin>,
    active_high: bool,
}

impl<'d> FemLine<'d> {
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, active_high: bool) -> Self {
        into_ref!(pin);
        Self {
            pin: pin.map_into(),
            active_high,
        }
    }

    fn spec(&self) -> FemPin {
        FemPin {
            port: if matches!(self.pin.port(), Port::Port0) {
                0
            } else {
                1
            },
            pin: self.pin.pin(),
            active_high: self.active_high,
        }
    }
}

/// An attached-ready FEM: lines, reserved channels and parameters.
pub struct NrfFem<'d> {
    plan: FemPlan,
    params: FemParams,
    tx_power: Option<TxPowerSplit>,
    powered: bool,
    _lines: [Option<PeripheralRef<'d, AnyPin>>; 3],
    _ppi: [Option<AnyConfigurableChannel>; 4],
}

impl<'d> NrfFem<'d> {
    /// Two-pin FEM (CTX = PA enable, CRX = LNA enable; AT2401C, RFX2401C,
    /// SKY66112-11 two-pin mode) with an optional CPU-controlled `pdn`
    /// (CSD / PDN / chip-enable) line. Takes two GPIOTE channels and four
    /// configurable PPI channels (0..=19).
    #[allow(clippy::too_many_arguments)]
    pub fn two_pin<A: FemGpioteChannel, B: FemGpioteChannel>(
        params: FemParams,
        ctx: FemLine<'d>,
        crx: FemLine<'d>,
        pdn: Option<FemLine<'d>>,
        _gpiote_ctx: A,
        _gpiote_crx: B,
        ppi: [AnyConfigurableChannel; 4],
    ) -> Result<Self, FemConfigError> {
        let layout = FemLayout::TwoPin {
            pa: ctx.spec(),
            lna: crx.spec(),
            gpiote_pa: A::NUMBER,
            gpiote_lna: B::NUMBER,
            ppi: ppi.each_ref().map(|channel| channel.number() as u8),
        };
        let [a, b, c, d] = ppi;
        Self::install(
            params,
            layout,
            [Some(ctx), Some(crx), pdn],
            [Some(a), Some(b), Some(c), Some(d)],
        )
    }

    /// PA-only FEM (one CTX line) with an optional `pdn` line. Takes one
    /// GPIOTE channel and three configurable PPI channels.
    pub fn single_pin<A: FemGpioteChannel>(
        params: FemParams,
        ctx: FemLine<'d>,
        pdn: Option<FemLine<'d>>,
        _gpiote_ctx: A,
        ppi: [AnyConfigurableChannel; 3],
    ) -> Result<Self, FemConfigError> {
        let layout = FemLayout::SinglePin {
            pa: ctx.spec(),
            gpiote_pa: A::NUMBER,
            ppi: ppi.each_ref().map(|channel| channel.number() as u8),
        };
        let [a, b, c] = ppi;
        Self::install(
            params,
            layout,
            [Some(ctx), pdn, None],
            [Some(a), Some(b), Some(c), None],
        )
    }

    fn install(
        params: FemParams,
        layout: FemLayout,
        lines: [Option<FemLine<'d>>; 3],
        ppi: [Option<AnyConfigurableChannel>; 4],
    ) -> Result<Self, FemConfigError> {
        params.validate(NRF_TX_POWER_STEPS, NRF_FEM_TIMING)?;
        // `lines` is [pa, lna, pdn] (two-pin) or [pa, pdn, -] (single-pin).
        let pdn_index = match layout {
            FemLayout::TwoPin { .. } => 2,
            FemLayout::SinglePin { .. } => 1,
        };
        let pdn = lines[pdn_index].as_ref().map(FemLine::spec);
        let plan = plan(layout, pdn)?;
        if ACTIVE.swap(true, Ordering::AcqRel) {
            return Err(FemConfigError::AlreadyActive);
        }

        write(PPI_CHENCLR, plan.channel_mask);
        for pin in &plan.pins {
            write(pin.inactive_register(), pin.mask());
            write(pin.pin_cnf_register(), PIN_CNF_OUTPUT);
        }
        for config in &plan.gpiote {
            write(config.register, config.value);
        }
        for link in &plan.links {
            write(link.eep_register(), link.eep);
            write(link.tep_register(), link.tep);
            write(link.fork_register(), link.fork);
        }
        PA_OFF_TASK.store(plan.pa.off, Ordering::Relaxed);
        LNA_OFF_TASK.store(plan.lna.map_or(0, |lna| lna.off), Ordering::Relaxed);
        RX_GAIN_DB.store(params.rx_gain_db, Ordering::Relaxed);
        core::sync::atomic::compiler_fence(Ordering::SeqCst);
        write(PPI_CHENSET, plan.channel_mask);

        let mut fem = Self {
            plan,
            params,
            tx_power: None,
            powered: false,
            _lines: lines.map(|line| line.map(|line| line.pin)),
            _ppi: ppi,
        };
        fem.power_up();
        log::info!(
            "[nRF FEM] PA +{} dB, LNA +{} dB, cap {} dBm, PPI mask {:#010x}",
            params.tx_gain_db,
            params.rx_gain_db,
            params.max_output_dbm,
            fem.plan.channel_mask
        );
        Ok(fem)
    }

    pub fn params(&self) -> &FemParams {
        &self.params
    }

    /// Last applied transmit power split (`None` before the MAC set one).
    pub fn tx_power(&self) -> Option<TxPowerSplit> {
        self.tx_power
    }

    /// Antenna-referred `(min, max)` transmit power in dBm.
    pub fn output_range(&self) -> (i8, i8) {
        self.params.output_range(NRF_TX_POWER_STEPS)
    }

    /// Split the antenna-referred request; the caller programs `radio_dbm`.
    pub(super) fn apply_tx_power(&mut self, requested_dbm: i8) -> TxPowerSplit {
        let split = split_tx_power(requested_dbm, &self.params, NRF_TX_POWER_STEPS);
        self.tx_power = Some(split);
        log::info!(
            "[nRF FEM] TX power requested {} dBm{} -> radio {} dBm + PA {} dB = {} dBm at antenna",
            split.requested_dbm,
            if split.capped { " (capped)" } else { "" },
            split.radio_dbm,
            self.params.tx_gain_db,
            split.antenna_dbm
        );
        split
    }

    /// Assert the PDN / chip-enable line (radio about to be used). The PA
    /// and LNA lines stay under PPI control. Idempotent.
    pub(super) fn power_up(&mut self) {
        if !self.powered {
            if let Some(pdn) = self.plan.pdn {
                write(pdn.active_register(), pdn.mask());
            }
            self.powered = true;
        }
    }

    /// Radio idle for sleep: PA and LNA off, PDN released. The RADIO must
    /// already be DISABLED.
    pub(super) fn power_down(&mut self) {
        abort();
        if let Some(pdn) = self.plan.pdn {
            write(pdn.inactive_register(), pdn.mask());
        }
        self.powered = false;
    }
}

impl Drop for NrfFem<'_> {
    fn drop(&mut self) {
        write(PPI_CHENCLR, self.plan.channel_mask);
        self.power_down();
        for link in &self.plan.links {
            write(link.eep_register(), 0);
            write(link.tep_register(), 0);
            write(link.fork_register(), 0);
        }
        // GPIOTE releases the pins; GPIO OUT already holds the inactive
        // level written at install time, so the lines do not glitch.
        for config in &self.plan.gpiote {
            write(config.register, 0);
        }
        PA_OFF_TASK.store(0, Ordering::Relaxed);
        LNA_OFF_TASK.store(0, Ordering::Relaxed);
        RX_GAIN_DB.store(0, Ordering::Relaxed);
        ACTIVE.store(false, Ordering::Release);
    }
}

// ── RadioPhy decorator ───────────────────────────────────────

/// An nRF PHY that can carry an [`NrfFem`]: `NrfRadioPhy` and
/// `NrfIsrRadioPhy`.
pub trait NrfFemHost: RadioPhy + ForwardAesProvider + PlatformServices + sealed::Host {}

impl<T: RadioInstance, R: RngInstance> sealed::Host for NrfRadioPhy<'_, T, R> {}
impl<T: RadioInstance, R: RngInstance> NrfFemHost for NrfRadioPhy<'_, T, R> {}

#[cfg(feature = "nrf-isr-phy")]
impl<R: RngInstance> sealed::Host for super::NrfIsrRadioPhy<'_, R> {
    fn arm_fem(&mut self, plan: &FemPlan, rx_gain_db: u8) {
        self.arm_fem(plan.ack_txen_fork(), rx_gain_db);
    }

    fn disarm_fem(&mut self) {
        self.disarm_fem();
    }
}
#[cfg(feature = "nrf-isr-phy")]
impl<R: RngInstance> NrfFemHost for super::NrfIsrRadioPhy<'_, R> {}

/// An nRF PHY with an attached RF front-end module (see the module docs).
///
/// Wrap the PHY after installing its AES engine and before building the
/// MAC; `into_soft_mac` / `into_parent_mac` start the MAC at the FEM's
/// [`FemParams::default_output_dbm`].
pub struct NrfFemPhy<'d, P: NrfFemHost> {
    phy: P,
    fem: NrfFem<'d>,
}

/// End-device MAC on the Embassy PHY with a FEM.
pub type NrfFemSoftMac<'a, T, R> =
    SoftMacCore<NrfFemPhy<'a, NrfRadioPhy<'a, T, R>>, EndDevice, NRF_PENDING_RX>;

/// Router MAC on the ISR PHY with a FEM.
#[cfg(feature = "nrf-isr-phy")]
pub type NrfFemRouterMac<'a, R> = SoftMacCore<
    NrfFemPhy<'a, super::NrfIsrRadioPhy<'a, R>>,
    super::NrfRouterRole,
    { super::NRF_ROUTER_PENDING_RX },
>;

/// End-device MAC on the ISR PHY with a FEM.
#[cfg(feature = "nrf-isr-phy")]
pub type NrfFemIsrSoftMac<'a, R> = SoftMacCore<
    NrfFemPhy<'a, super::NrfIsrRadioPhy<'a, R>>,
    EndDevice,
    { super::NRF_ROUTER_PENDING_RX },
>;

impl<'d, P: NrfFemHost> NrfFemPhy<'d, P> {
    pub fn new(mut phy: P, fem: NrfFem<'d>) -> Self {
        phy.arm_fem(&fem.plan, fem.params.rx_gain_db);
        Self { phy, fem }
    }

    pub fn inner(&self) -> &P {
        &self.phy
    }

    pub fn inner_mut(&mut self) -> &mut P {
        &mut self.phy
    }

    pub fn fem(&self) -> &NrfFem<'d> {
        &self.fem
    }

    /// MAC PIB starting at the FEM's default (antenna-referred) output.
    fn initial_pib(&self, ieee: IeeeAddress, seed: u32) -> Result<MacPib, MacError> {
        let mut pib = MacPib::new(ieee, seed as u8, (seed >> 8) as u8);
        pib.set(
            PibAttribute::PhyTransmitPower,
            PibValue::I8(self.fem.params.default_output_dbm),
        )
        .map_err(|_| MacError::InvalidParameter)?;
        Ok(pib)
    }
}

impl<'a, T: RadioInstance, R: RngInstance> NrfFemPhy<'a, NrfRadioPhy<'a, T, R>> {
    /// Build the end-device MAC (see `NrfRadioPhy::into_soft_mac`).
    #[inline(never)]
    pub fn into_soft_mac(mut self) -> Result<NrfFemSoftMac<'a, T, R>, MacError> {
        let seed = self.phy.random_u32();
        let pib = self.initial_pib(NrfRadioPhy::<T, R>::read_ficr_ieee(), seed)?;
        SoftMacCore::new_custom(self, pib)
    }

    /// `NrfRadioPhy::enter_low_power_idle` plus PA/LNA off and PDN release.
    pub fn enter_low_power_idle(&mut self) -> Result<(), PhyError> {
        let result = self.phy.enter_low_power_idle();
        self.fem.power_down();
        result
    }
}

#[cfg(feature = "nrf-isr-phy")]
impl<'a, R: RngInstance> NrfFemPhy<'a, super::NrfIsrRadioPhy<'a, R>> {
    /// Build the router MAC (see `NrfIsrRadioPhy::into_parent_mac`).
    #[inline(never)]
    pub fn into_parent_mac(mut self) -> Result<NrfFemRouterMac<'a, R>, MacError> {
        let seed = self.phy.random_u32();
        let pib = self.initial_pib(super::NrfIsrRadioPhy::<R>::read_ficr_ieee(), seed)?;
        SoftMacCore::new_custom(self, pib)
    }

    /// Build the always-on end-device MAC on the ISR PHY.
    #[inline(never)]
    pub fn into_soft_mac(mut self) -> Result<NrfFemIsrSoftMac<'a, R>, MacError> {
        let seed = self.phy.random_u32();
        let pib = self.initial_pib(super::NrfIsrRadioPhy::<R>::read_ficr_ieee(), seed)?;
        SoftMacCore::new_custom(self, pib)
    }

    /// The RADIO handler's diagnostic counters.
    pub fn counters(&self) -> super::NrfRadioCounters {
        self.phy.counters()
    }
}

impl<P: NrfFemHost> Drop for NrfFemPhy<'_, P> {
    fn drop(&mut self) {
        self.phy.disarm_fem();
    }
}

/// Drops PA and LNA if a transmission future is cancelled or fails.
struct TxAbortGuard {
    armed: bool,
}

impl TxAbortGuard {
    fn finish<T>(mut self, result: &Result<T, PhyError>) {
        self.armed = result.is_err();
    }
}

impl Drop for TxAbortGuard {
    fn drop(&mut self) {
        if self.armed {
            abort();
        }
    }
}

impl<P: NrfFemHost> RadioPhy for NrfFemPhy<'_, P> {
    type Tx = P::Tx;

    const POLL_RESPONSE_WAIT_US: u32 = P::POLL_RESPONSE_WAIT_US;
    const POLL_EXTENDED_FALLBACK: bool = P::POLL_EXTENDED_FALLBACK;
    const DEFAULT_TX_POWER_DBM: i8 = P::DEFAULT_TX_POWER_DBM;
    #[cfg(feature = "timed-tx")]
    const TIMED_TX: bool = P::TIMED_TX;

    fn poll_response_wait_us(&self) -> u32 {
        self.phy.poll_response_wait_us()
    }

    /// The PHY's capabilities with the antenna-referred TX power range.
    fn capabilities(&self) -> PhyCapabilities {
        let (tx_power_min, tx_power_max) = self.fem.output_range();
        PhyCapabilities {
            tx_power_min,
            tx_power_max,
            ..self.phy.capabilities()
        }
    }

    fn default_tx_power_dbm(&self) -> i8 {
        self.fem.params.default_output_dbm
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        self.fem.power_up();
        let guard = TxAbortGuard { armed: true };
        let result = self.phy.try_transmit(frame).await;
        guard.finish(&result);
        result
    }

    async fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), PhyError> {
        self.fem.power_up();
        let guard = TxAbortGuard { armed: true };
        let result = self.phy.send_ack(sequence, frame_pending).await;
        guard.finish(&result);
        result
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        self.fem.power_up();
        self.phy.receive(timeout_us, frame).await
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        self.phy.set_channel(channel)
    }

    /// `dbm` is the antenna output: capped and split between radio and PA.
    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        let split = self.fem.apply_tx_power(dbm);
        self.phy.set_tx_power(split.radio_dbm)
    }

    /// Energy on the IEEE 0..=255 scale (`ED_RSSISCALE` = 4 per dB),
    /// antenna-referred. Saturated readings (255) stay saturated.
    async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError> {
        self.fem.power_up();
        let energy = self.phy.energy_detect(duration_us).await?;
        if energy == u8::MAX {
            return Ok(energy);
        }
        Ok(energy.saturating_sub(
            self.fem
                .params
                .rx_gain_db
                .saturating_mul(NRF_ED_UNITS_PER_DB),
        ))
    }

    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        self.phy.set_address_filter(filter)
    }

    async fn try_transmit_acknowledged(
        &mut self,
        frame: &[u8],
    ) -> Result<Option<AckResult>, PhyError> {
        self.fem.power_up();
        let guard = TxAbortGuard { armed: true };
        let result = self.phy.try_transmit_acknowledged(frame).await;
        guard.finish(&result);
        result
    }

    async fn transmit_offloaded(
        &mut self,
        frame: &[u8],
        ack_requested: bool,
        params: TxParams,
    ) -> Result<Option<AckResult>, PhyError> {
        self.fem.power_up();
        let guard = TxAbortGuard { armed: true };
        let result = self
            .phy
            .transmit_offloaded(frame, ack_requested, params)
            .await;
        guard.finish(&result);
        result
    }

    fn set_ack_frame_pending(
        &mut self,
        source: &MacAddress,
        pending: bool,
    ) -> Result<(), PhyError> {
        self.phy.set_ack_frame_pending(source, pending)
    }

    fn clear_ack_frame_pending(&mut self) {
        self.phy.clear_ack_frame_pending();
    }

    #[cfg(feature = "timed-tx")]
    async fn transmit_at(&mut self, frame: &[u8], at_us: u32) -> Result<(), PhyError> {
        self.fem.power_up();
        let guard = TxAbortGuard { armed: true };
        let result = self.phy.transmit_at(frame, at_us).await;
        guard.finish(&result);
        result
    }

    #[cfg(feature = "timed-tx")]
    fn arm_gp_reply(&mut self, src_id: u32, frame: &[u8], offset_us: u32) -> Result<(), PhyError> {
        self.phy.arm_gp_reply(src_id, frame, offset_us)
    }

    #[cfg(feature = "timed-tx")]
    fn disarm_gp_reply(&mut self) {
        self.phy.disarm_gp_reply();
    }

    #[cfg(feature = "timed-tx")]
    fn take_gp_reply_outcome(&mut self) -> Option<crate::gp_reply::GpReplyOutcome> {
        self.phy.take_gp_reply_outcome()
    }

    fn set_rx_on_when_idle(&mut self, enabled: bool) {
        self.fem.power_up();
        self.phy.set_rx_on_when_idle(enabled);
    }

    /// The PHY's sleep, then PA/LNA off and PDN released.
    fn sleep(&mut self) -> Result<(), PhyError> {
        let result = self.phy.sleep();
        self.fem.power_down();
        result
    }

    /// PDN asserted before the PHY restarts the receiver (its ≥ 40 µs RX
    /// ramp-up covers the FEM's power-up settle time).
    fn wake(&mut self) -> Result<(), PhyError> {
        self.fem.power_up();
        self.phy.wake()
    }
}

impl<P: NrfFemHost> ForwardAesProvider for NrfFemPhy<'_, P> {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        self.phy.forward_cipher(key)
    }
}

impl<P: NrfFemHost> PlatformServices for NrfFemPhy<'_, P> {
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
