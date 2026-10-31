//! Pure-Rust EFR32 Series 2 IEEE 802.15.4 (2.4 GHz O-QPSK) radio driver.
//!
//! # How the radio is driven
//!
//! On Series 2, RAIL normally runs microcode on the radio sequencer (a
//! Cortex-M0 inside the radio) that rewrites analog RAC/SYNTH registers on
//! every state transition. That microcode is a binary blob and is **not**
//! used here. Instead `RAC.CTRL.SEQCLKDIS` gates the sequencer clock and the
//! application CPU takes over the sequencer's duties:
//!
//! - **State hand-shake.** The radio state machine (RSM) stops in every state
//!   it enters (including the post-reset `SHUTDOWN`) until the matching
//!   `RAC.SEQIF.STATE*` bit is cleared. Hardware-verified on xG22: with the
//!   bit pending, `RXWARM` never reaches `RXSEARCH`, `RXSEARCH` detects no
//!   frames and `RX2RX` never re-arms. [`service_sequencer`] clears those
//!   bits; it runs from the `RAC_RSM` interrupt (`RAC.IF.STATECHANGE`, with
//!   the `rt` feature) and from every busy-wait in this driver. `RXWARM` and
//!   `TXWARM` are held instead: only there does the RSM power the LO path,
//!   so the driver calibrates and locks the synthesizer while the RSM waits
//!   and then releases the warm state itself.
//! - **Analog state.** The CPU writes the per-state analog values itself
//!   (see [`AnalogState`]) before commanding RX or TX.
//! - **VCO calibration.** The synthesizer's hardware capacitor calibration
//!   (`SYNTH.CMD.CAPCALSTART`) does not converge without the sequencer
//!   (it ends at `VCOTUNING = 0x400` with `IF.VCOLOW`). The driver instead
//!   binary-searches `SYNTH.VCOTUNING` with the LO counter
//!   (`SYNTH.LOCNT*`) against `(FREQ ± IFFREQ) >> 13`, which reproduces the
//!   `LOCNTTARGET` the RAIL reference computes, then closes the loop with
//!   `SYNTHSTART` and verifies lock by counting again. The search runs with
//!   the VCDAC pinning the VCO control voltage (`VCDACCTRL.VCDACEN`) so the
//!   result does not depend on the previous lock (see `search_vco`).
//!
//! The hardware radio state machine (RSM), FRC framing, MODEM, AGC, RFCRC and
//! BUFC still do all bit-level work.
//!
//! - TX: the PSDU is written to BUFC buffer 0 behind a PHR; FRC appends the
//!   CRC (`FCD1.CALCCRC`).
//! - RX: FRC drops frames with a bad CRC and stores `[PHR][PSDU - FCS]` in
//!   BUFC buffer 1, followed by a one-byte RSSI trailer (`TRAILRXDATA`).
//! - ACKs are generated and checked in software ([`Radio::receive`] and
//!   [`Radio::transmit_csma`]); turnaround therefore depends on the caller
//!   polling `receive` while it expects acknowledged traffic.
//!
//! The per-chip tables and the radio state hand-shake (xG22: `RAC.SEQIF`
//! with the sequencer clock-gated; xG21: a driver-owned sequencer vector
//! table) live in `xg21.rs` / `xg22.rs`.

#[cfg(feature = "efr32mg21")]
#[path = "xg21.rs"]
mod chip;
#[cfg(feature = "efr32mg22")]
#[path = "xg22.rs"]
mod chip;

use core::cell::UnsafeCell;

use crate::delay::{Deadline, spin_cycles, us_to_cycles};

/// Maximum PSDU length including the 2-byte FCS.
pub const MAX_PSDU: usize = 127;
/// Maximum frame length handed to [`Radio::transmit`] (FCS added by FRC).
pub const MAX_FRAME: usize = MAX_PSDU - 2;

/// Calibrated transmit-power range of this chip in whole dBm `(min, max)`;
/// `(0, 0)` when the chip has no validated PA table.
pub const TX_POWER_RANGE_DBM: (i8, i8) = {
    let table = chip::TX_POWER;
    if table.is_empty() {
        (0, 0)
    } else {
        (
            (table[0].0 / 10) as i8,
            (table[table.len() - 1].0 / 10) as i8,
        )
    }
};

/// Default clear-channel threshold (IEEE 802.15.4 ED mode; RAIL default).
pub const DEFAULT_CCA_THRESHOLD_DBM: i8 = -75;
/// Chip RSSI adjustment added to the AGC reading (see `chip::RSSI_ADJUST_DBM`).
pub const RSSI_ADJUST_DBM: i8 = chip::RSSI_ADJUST_DBM;
/// 802.15.4 2.4 GHz timing (us).
pub const SYMBOL_US: u32 = 16;
pub const UNIT_BACKOFF_US: u32 = 20 * SYMBOL_US;
pub const CCA_US: u32 = 8 * SYMBOL_US;
/// macAckWaitDuration for O-QPSK 2.4 GHz (54 symbols).
pub const ACK_WAIT_US: u32 = 54 * SYMBOL_US;

/// Per-chip register placement that differs between xG21 and xG22.
pub(crate) struct Layout {
    frc: u32,
    agc: u32,
    modem: u32,
    synth: u32,
    rac: u32,
    bufc: u32,
    bufc_buf0: u32,
    bufc_buf_stride: u32,
    agc_rssi: u32,
}

/// Transition state the driver holds: the warm states while it locks the
/// synthesizer (and ramps the PA up), `TxPd` while it ramps the PA down.
#[derive(Clone, Copy)]
pub(crate) enum Warm {
    Rx,
    Tx,
    TxPd,
}

/// One analog state written by the CPU in place of sequencer microcode.
pub(crate) struct AnalogState {
    regs: &'static [u32; chip::STATE_REGS],
    values: [u32; chip::STATE_REGS],
    /// Further `(address, value)` writes of the state (SYNTH VCO DAC/range,
    /// and on xG21 the AGC TX overrides), applied after `regs`.
    extra: &'static [(u32, u32)],
}

const L: Layout = chip::LAYOUT;

// Offsets identical on xG21 and xG22.
const FRC_STATUS: u32 = 0x08;
const FRC_CMD: u32 = 0x2C;
const FRC_TRAILRXDATA: u32 = 0x54;
const FRC_IF: u32 = 0x68;
const FRC_IF_TXDONE: u32 = 1 << 0;
const FRC_IF_TXABORTED: u32 = 1 << 2;
const FRC_IF_TXUF: u32 = 1 << 3;
const FRC_IF_RXOF: u32 = 1 << 8;
const FRC_CMD_RXABORT: u32 = 1 << 0;

const RAC_RXENSRCEN: u32 = 0x08;
const RAC_STATUS: u32 = 0x0C;
const RAC_CMD: u32 = 0x10;
const RAC_CMD_TXEN: u32 = 1 << 0;
const RAC_CMD_CLEARTXEN: u32 = 1 << 3;
const RAC_CMD_TXDIS: u32 = 1 << 5;
const RAC_CMD_CLEARRXOVERFLOW: u32 = 1 << 6;
const RAC_CMD_RXDIS: u32 = 1 << 8;
const RAC_SWRXEN: u32 = 1 << 1;
const RAC_IF: u32 = 0x1C;
const RAC_IEN: u32 = 0x20;
const RAC_IF_STATECHANGE: u32 = 1 << 0;
/// NVIC set-enable register holding `RAC_RSM` (IRQ 37) on xG21 and xG22.
#[cfg(feature = "rt")]
const NVIC_ISER1: u32 = 0xE000_E104;
#[cfg(feature = "rt")]
const RAC_RSM_IRQ_BIT: u32 = 1 << (37 - 32);

const SYNTH_STATUS: u32 = 0x08;
const SYNTH_CMD: u32 = 0x0C;
const SYNTH_FREQ: u32 = 0x34;
const SYNTH_IFFREQ: u32 = 0x38;
const SYNTH_CHCTRL: u32 = 0x40;
const SYNTH_VCOTUNING: u32 = 0x4C;
const SYNTH_VCDACCTRL: u32 = 0x2C;
/// `VCDACCTRL.VCDACEN`: the VCDAC drives the VCO control voltage.
const VCDACCTRL_VCDACEN: u32 = 1 << 6;
const SYNTH_IF: u32 = 0x78;
const SYNTH_LOCNTCTRL: u32 = 0x88;
const SYNTH_LOCNTSTATUS: u32 = 0x8C;
const SYNTH_CMD_START: u32 = 1 << 0;
const SYNTH_CMD_STOP: u32 = 1 << 1;
const SYNTH_CMD_ENABLEIF: u32 = 1 << 2;
const SYNTH_CMD_DISABLEIF: u32 = 1 << 3;
const SYNTH_IF_LOCNTDONE: u32 = 1 << 9;
/// `LOCNTCTRL`: [FCALRUNCLKEN] | NUMCYCLE=4 | ENABLE, plus CLEAR/RUN/READ.
const LOCNT_BASE: u32 = chip::LOCNT_FCALRUNCLKEN | (4 << 4) | (1 << 0);
const LOCNT_CLEAR: u32 = 1 << 1;
const LOCNT_RUN: u32 = 1 << 2;
const LOCNT_READ: u32 = 1 << 3;
const LOCNT_COUNT_MASK: u32 = 0x7_FFFF;
const LOCNT_BUSY: u32 = 1 << 19;
/// VCOTUNING.VCAPSEL used by the RAIL reference on xG22 (also the reset
/// value); the fine 11-bit `VCOTUNING` field alone spans 2.40-2.48 GHz.
const VCAPSEL_DEFAULT: u32 = 16 << 11;
const VCOTUNING_MAX: u32 = 0x7FF;
/// Closed-loop LO count must be within this many counts of the target.
const LOCK_TOLERANCE: u32 = 2;
const VCO_SETTLE_US: u32 = 5;
/// Closed-loop settling allowed for a cached tuning code before searching.
const CACHED_LOCK_US: u32 = 40;
const WARM_SETTLE_US: u32 = 50;

const BUFC_EN: u32 = 0x04;
const BUF_CTRL: u32 = 0x00;
const BUF_ADDR: u32 = 0x04;
const BUF_READOFFSET: u32 = 0x0C;
const BUF_WRITEDATA: u32 = 0x18;
const BUF_STATUS: u32 = 0x20;
const BUF_CMD: u32 = 0x28;
const BUF_CMD_CLEAR: u32 = 1 << 0;
const BUF_STATUS_BYTES: u32 = 0x1FFF;

/// TX buffer: 256 B (size code 2), RX buffer: 512 B (size code 3).
const TX_BUF_SIZE_CODE: u32 = 2;
const RX_BUF_SIZE_CODE: u32 = 3;
/// Bytes of the RX buffer (`RX_BUF_SIZE_CODE` 3 = 512 bytes).
const RX_BUF_LEN: u32 = 512;
/// READOFFSET counts modulo 8 KiB; the buffer wraps at `RX_BUF_LEN`.
const BUF_OFFSET_MASK: u32 = 0x1FFF;
const FRC_TRAILRXDATA_RSSI: u32 = 1 << 0;
/// RX trailer appended by FRC after each PSDU (RSSI byte).
const RX_TRAILER_LEN: usize = 1;

const SYNTH_TIMEOUT_US: u32 = 200;
/// Worst-case 127-byte frame (4.3 ms) plus warm-up margin.
const TX_TIMEOUT_US: u32 = 6_000;
/// The RSM enters `TXPD` right after the last symbol.
const TXPD_WAIT_US: u32 = 100;

/// RAC radio-state-machine states (`RAC.STATUS.STATE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RacState {
    Off,
    RxWarm,
    RxSearch,
    RxFrame,
    RxPowerDown,
    Rx2Rx,
    RxOverflow,
    Rx2Tx,
    TxWarm,
    Tx,
    TxPowerDown,
    Tx2Rx,
    Tx2Tx,
    Shutdown,
    Unknown(u8),
}

impl RacState {
    fn from_raw(raw: u8) -> Self {
        match raw {
            0 => Self::Off,
            1 => Self::RxWarm,
            2 => Self::RxSearch,
            3 => Self::RxFrame,
            4 => Self::RxPowerDown,
            5 => Self::Rx2Rx,
            6 => Self::RxOverflow,
            7 => Self::Rx2Tx,
            8 => Self::TxWarm,
            9 => Self::Tx,
            10 => Self::TxPowerDown,
            11 => Self::Tx2Rx,
            12 => Self::Tx2Tx,
            13 => Self::Shutdown,
            other => Self::Unknown(other),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioError {
    /// No validated register tables for this die (xG21 today).
    Unsupported,
    /// The radio state machine did not leave `SHUTDOWN` after init.
    Shutdown,
    InvalidChannel,
    FrameTooLong,
    /// The synthesizer did not finish capacitor calibration / lock in time.
    SynthLock,
    /// FRC did not report TX completion in time (or aborted/underflowed).
    TxTimeout,
    /// CSMA-CA gave up (`CHANNEL_ACCESS_FAILURE`).
    ChannelBusy,
    /// No matching acknowledgement within macAckWaitDuration.
    NoAck,
}

/// A received frame, without FCS.
#[derive(Debug, Clone, Copy)]
pub struct RxFrame {
    pub len: usize,
    pub rssi_dbm: i8,
    pub lqi: u8,
}

/// Unslotted CSMA-CA parameters (IEEE 802.15.4-2015 6.2.5.1).
#[derive(Debug, Clone, Copy)]
pub struct CsmaConfig {
    pub min_be: u8,
    pub max_be: u8,
    pub max_backoffs: u8,
}

impl Default for CsmaConfig {
    fn default() -> Self {
        Self {
            min_be: 3,
            max_be: 5,
            max_backoffs: 4,
        }
    }
}

/// Outcome of an acknowledged transmission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxOutcome {
    /// `Some(frame_pending)` when an ACK was requested and received.
    pub ack_frame_pending: Option<bool>,
}

/// Local addressing used for software auto-ACK of received frames.
#[derive(Debug, Clone, Copy, Default)]
pub struct AckFilter {
    pub pan_id: Option<u16>,
    pub short_address: Option<u16>,
    pub extended_address: Option<u64>,
}

#[repr(C, align(256))]
struct TxRam(UnsafeCell<[u8; 256]>);
#[repr(C, align(512))]
struct RxRam(UnsafeCell<[u8; 512]>);
// SAFETY: BUFC writes these through DMA; the CPU writes TX_RAM only via BUFC
// WRITEDATA and reads RX_RAM only (volatile) for bytes that BUFC STATUS has
// already reported available. The singleton `RadioToken` guarantees a single
// owner.
unsafe impl Sync for TxRam {}
unsafe impl Sync for RxRam {}

static TX_RAM: TxRam = TxRam(UnsafeCell::new([0; 256]));
static RX_RAM: RxRam = RxRam(UnsafeCell::new([0; 512]));

/// Exclusive ownership of the radio blocks (RAC/FRC/MODEM/AGC/SYNTH/BUFC).
pub struct RadioToken {
    _private: (),
}

impl RadioToken {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    /// Program the 802.15.4 PHY and return an idle radio on channel 11.
    ///
    /// Requires the HFXO to be running (radio reference clock).
    pub fn init(self) -> Result<Radio, RadioError> {
        // `black_box`: an unsupported die must fail at run time without
        // letting the optimiser prune everything downstream of radio init,
        // so the image still links (and is sized) like a working one.
        if !core::hint::black_box(chip::SUPPORTED) {
            return Err(RadioError::Unsupported);
        }
        crate::delay::enable_cycle_counter();
        program_phy();
        leave_shutdown()?;
        let mut radio = Radio {
            _token: self,
            channel: 11,
            power_ddbm: 0,
            rx_enabled: false,
            cca_threshold_dbm: DEFAULT_CCA_THRESHOLD_DBM,
            ack_filter: AckFilter::default(),
            rng: seed(),
            fcal: [[0; 2]; 16],
            stats: RadioStats::default(),
        };
        apply_state(&chip::IDLE);
        radio.set_tx_power_ddbm(0);
        radio.set_channel(11)?;
        Ok(radio)
    }
}

/// Debug counters.
#[derive(Debug, Clone, Copy, Default)]
pub struct RadioStats {
    pub rx_frames: u32,
    pub rx_overflows: u32,
    /// RX buffer cleared after an implausible PHR.
    pub rx_resyncs: u32,
    pub tx_frames: u32,
    pub tx_failures: u32,
    pub acks_sent: u32,
    pub synth_failures: u32,
}

/// Initialized 802.15.4 radio.
pub struct Radio {
    _token: RadioToken,
    channel: u8,
    power_ddbm: i16,
    rx_enabled: bool,
    cca_threshold_dbm: i8,
    ack_filter: AckFilter,
    rng: u32,
    /// Last good `VCOTUNING` per channel, `[rx, tx]`; 0 = not calibrated.
    fcal: [[u16; 2]; 16],
    pub stats: RadioStats,
}

impl Radio {
    pub fn channel(&self) -> u8 {
        self.channel
    }

    /// Tune to an O-QPSK channel 11..=26. Restarts RX if it was enabled.
    pub fn set_channel(&mut self, channel: u8) -> Result<(), RadioError> {
        let freq = synth_freq(channel).ok_or(RadioError::InvalidChannel)?;
        let was_rx = self.rx_enabled;
        self.idle();
        unsafe {
            write(L.synth + SYNTH_CHCTRL, 0);
            write(L.synth + SYNTH_FREQ, freq);
        }
        self.channel = channel;
        if was_rx {
            self.rx_on()?;
        }
        Ok(())
    }

    /// Select the highest calibrated level not above `ddbm` (deci-dBm);
    /// returns the applied level.
    pub fn set_tx_power_ddbm(&mut self, ddbm: i16) -> i16 {
        let (level, ramp, pa) = tx_power_entry(ddbm);
        chip::write_tx_power(ramp, pa);
        self.power_ddbm = level;
        level
    }

    pub fn tx_power_ddbm(&self) -> i16 {
        self.power_ddbm
    }

    pub fn set_cca_threshold_dbm(&mut self, threshold: i8) {
        self.cca_threshold_dbm = threshold;
    }

    pub fn set_ack_filter(&mut self, filter: AckFilter) {
        self.ack_filter = filter;
    }

    pub fn rac_state(&self) -> RacState {
        rac_state_raw()
    }

    pub fn is_rx_on(&self) -> bool {
        self.rx_enabled
    }

    /// Enter continuous receive (RX search) on the current channel.
    pub fn rx_on(&mut self) -> Result<(), RadioError> {
        if self.rx_enabled {
            return Ok(());
        }
        self.start_rx()?;
        self.rx_enabled = true;
        Ok(())
    }

    /// Stop RX/TX and power the analog front end down. Already received
    /// frames stay in the RX buffer.
    pub fn idle(&mut self) {
        self.force_off();
        unsafe { write(L.synth + SYNTH_CMD, SYNTH_CMD_STOP) };
        apply_state(&chip::IDLE);
    }

    /// Bring the RSM to `OFF` (bounded). `RXDIS` alone only takes effect at
    /// the end of a frame being received, and `RXOVERFLOW` is only left
    /// through `CLEARRXOVERFLOW`; a RSM left in either state ignores the
    /// next `SWRXEN`/`TXEN` and the following warm-up stalls (hardware-
    /// observed on xG22 as a permanent `SynthLock` after an RX overflow).
    /// Frames already stored in the RX buffer are kept unless it overflowed.
    fn force_off(&mut self) -> bool {
        self.rx_enabled = false;
        unsafe {
            write(L.rac + RAC_RXENSRCEN, 0);
            write(
                L.rac + RAC_CMD,
                RAC_CMD_RXDIS | RAC_CMD_TXDIS | RAC_CMD_CLEARTXEN,
            );
        }
        self.service_overflow();
        if wait_off_releasing(200) {
            return true;
        }
        unsafe { write(L.frc + FRC_CMD, FRC_CMD_RXABORT) };
        self.service_overflow();
        wait_off_releasing(200)
    }

    /// Re-program the radio after EM2 before any other radio call.
    ///
    /// On xG22 the PHY register file survives EM2 except for a few
    /// sequencer-owned registers, but the receiver does not work again
    /// until it is re-initialized (lab gate `s2-em2`). Keeps channel, TX
    /// power, CCA/ACK settings and the synthesizer calibration cache;
    /// frames still buffered before sleep are dropped. Leaves RX off.
    pub fn resume(&mut self) -> Result<(), RadioError> {
        self.rx_enabled = false;
        program_phy();
        leave_shutdown()?;
        apply_state(&chip::IDLE);
        let power = self.power_ddbm;
        self.set_tx_power_ddbm(power);
        self.set_channel(self.channel)
    }

    fn start_rx(&mut self) -> Result<(), RadioError> {
        if rac_state_raw() != RacState::Off && !self.force_off() {
            self.stats.synth_failures += 1;
            return Err(RadioError::SynthLock);
        }
        apply_state(&chip::RX);
        chip::seq_arm(Warm::Rx);
        unsafe { write(L.rac + RAC_RXENSRCEN, RAC_SWRXEN) };
        let slot = &mut self.fcal[usize::from(self.channel - 11)][0];
        let result = warm_up(RacState::RxWarm, Warm::Rx, true, slot);
        if result.is_err() {
            self.stats.synth_failures += 1;
            self.idle();
        }
        result
    }

    /// Instantaneous RSSI in dBm, `None` while the AGC has no valid value.
    pub fn rssi_dbm(&self) -> Option<i8> {
        rssi_from_reg(unsafe { read(L.agc + L.agc_rssi) })
    }

    /// Peak RSSI over `duration_us` (receiver must be on). Used for both
    /// energy detection and CCA.
    pub fn energy_detect(&mut self, duration_us: u32) -> Result<Option<i8>, RadioError> {
        self.rx_on()?;
        let deadline = Deadline::after_us(duration_us);
        let mut peak: Option<i8> = None;
        loop {
            service_sequencer();
            if let Some(rssi) = self.rssi_dbm() {
                peak = Some(peak.map_or(rssi, |p| p.max(rssi)));
            }
            if deadline.expired() {
                return Ok(peak);
            }
        }
    }

    /// Energy-above-threshold CCA over 8 symbols. `true` = channel clear.
    pub fn cca(&mut self) -> Result<bool, RadioError> {
        // The AGC needs a few us after RX entry before RSSI is valid.
        let deadline = Deadline::after_us(CCA_US * 2);
        loop {
            match self.energy_detect(CCA_US)? {
                Some(peak) => return Ok(peak < self.cca_threshold_dbm),
                None if deadline.expired() => return Ok(false),
                None => {}
            }
        }
    }

    /// Transmit `frame` (MHR + payload, without FCS) immediately, without
    /// CCA and without waiting for an ACK. The receiver is restored
    /// afterwards if it was on.
    pub fn transmit(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        if frame.len() > MAX_FRAME {
            return Err(RadioError::FrameTooLong);
        }
        let resume_rx = self.rx_enabled;
        let result = self.transmit_raw(frame);
        if resume_rx {
            self.rx_enabled = false;
            self.rx_on()?;
        }
        result
    }

    /// Transmit an Imm-Ack for `sequence` with the given Frame Pending bit,
    /// then resume RX if it was on.
    ///
    /// For callers that own ACK policy themselves (a parent MAC answering
    /// data polls with a per-child Frame Pending bit). Such callers keep the
    /// [`AckFilter`] empty so [`receive`](Self::receive) never ACKs on its
    /// own, and must call this promptly: the sender only waits
    /// macAckWaitDuration (864 us) for the ACK.
    pub fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), RadioError> {
        let ack = [0x02 | ((frame_pending as u8) << 4), 0x00, sequence];
        self.transmit(&ack)?;
        self.stats.acks_sent += 1;
        Ok(())
    }

    fn transmit_raw(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        // Leave RX without discarding already stored frames; a frame
        // still arriving is aborted since the caller decided to transmit.
        if !self.force_off() {
            self.idle();
            self.stats.tx_failures += 1;
            return Err(RadioError::TxTimeout);
        }

        let tx = buf(0);
        unsafe {
            write(tx + BUF_CMD, BUF_CMD_CLEAR);
            write(tx + BUF_WRITEDATA, frame.len() as u32 + 2);
            for &byte in frame {
                write(tx + BUF_WRITEDATA, byte as u32);
            }
            write(
                L.frc + FRC_IF + CLR,
                FRC_IF_TXDONE | FRC_IF_TXABORTED | FRC_IF_TXUF,
            );
        }
        apply_state(&chip::TX);
        chip::tx_state_power(self.power_ddbm);
        chip::seq_arm(Warm::Tx);
        chip::seq_arm(Warm::TxPd);
        unsafe { write(L.rac + RAC_CMD, RAC_CMD_TXEN) };
        let slot = &mut self.fcal[usize::from(self.channel - 11)][1];
        if let Err(error) = warm_up(RacState::TxWarm, Warm::Tx, false, slot) {
            self.stats.synth_failures += 1;
            self.stats.tx_failures += 1;
            unsafe { write(L.rac + RAC_CMD, RAC_CMD_CLEARTXEN | RAC_CMD_TXDIS) };
            self.idle();
            return Err(error);
        }
        let deadline = Deadline::after_us(TX_TIMEOUT_US);
        let result = loop {
            service_sequencer();
            let flags = unsafe { read(L.frc + FRC_IF) };
            if flags & FRC_IF_TXDONE != 0 {
                break Ok(());
            }
            if flags & (FRC_IF_TXABORTED | FRC_IF_TXUF) != 0 || deadline.expired() {
                break Err(RadioError::TxTimeout);
            }
        };
        tx_power_down();
        unsafe {
            write(
                L.frc + FRC_IF + CLR,
                FRC_IF_TXDONE | FRC_IF_TXABORTED | FRC_IF_TXUF,
            );
            write(L.rac + RAC_CMD, RAC_CMD_CLEARTXEN | RAC_CMD_TXDIS);
        }
        let deadline = Deadline::after_us(200);
        while self.rac_state() != RacState::Off && !deadline.expired() {
            service_sequencer();
        }
        match result {
            Ok(()) => self.stats.tx_frames += 1,
            Err(_) => self.stats.tx_failures += 1,
        }
        result
    }

    /// One clear-channel assessment, then transmit `frame` and, when it
    /// requests an acknowledgement, wait macAckWaitDuration for it.
    ///
    /// This is the single-attempt primitive a MAC layer's CSMA-CA and retry
    /// loop drives (`SoftMacCore` with `AckWaitTx`). A busy channel returns
    /// [`RadioError::ChannelBusy`] without transmitting; a missing ACK
    /// returns [`RadioError::NoAck`]. Leaves the receiver on afterwards.
    /// Frames received while waiting for the ACK stay queued for
    /// [`receive`](Self::receive).
    pub fn transmit_cca(&mut self, frame: &[u8]) -> Result<TxOutcome, RadioError> {
        if frame.len() > MAX_FRAME {
            return Err(RadioError::FrameTooLong);
        }
        self.rx_on()?;
        if !self.cca()? {
            return Err(RadioError::ChannelBusy);
        }
        let ack_requested = frame.len() >= 3 && frame[0] & 0x20 != 0;
        let sequence = frame.get(2).copied().unwrap_or(0);
        self.transmit_raw(frame)?;
        self.start_rx()?;
        self.rx_enabled = true;
        if !ack_requested {
            return Ok(TxOutcome {
                ack_frame_pending: None,
            });
        }
        // ACK must start within macAckWaitDuration; allow its 352 us airtime.
        let deadline = Deadline::after_us(ACK_WAIT_US + 12 * 32);
        loop {
            service_sequencer();
            self.service_overflow();
            if let Some(found) = self.find_ack(sequence) {
                return Ok(TxOutcome {
                    ack_frame_pending: Some(found),
                });
            }
            if deadline.expired() {
                return Err(RadioError::NoAck);
            }
        }
    }

    /// Unslotted CSMA-CA (IEEE 802.15.4-2015 6.2.5.1) around
    /// [`transmit_cca`](Self::transmit_cca), without retries. For lab
    /// binaries; a MAC layer runs its own CSMA-CA.
    pub fn transmit_csma(
        &mut self,
        frame: &[u8],
        csma: CsmaConfig,
    ) -> Result<TxOutcome, RadioError> {
        let mut be = csma.min_be;
        let mut backoffs = 0u8;
        loop {
            let slots = self.next_random() & ((1u32 << be) - 1);
            self.rx_on()?;
            spin_cycles(us_to_cycles(slots * UNIT_BACKOFF_US));
            match self.transmit_cca(frame) {
                Err(RadioError::ChannelBusy) => {}
                other => return other,
            }
            backoffs += 1;
            if backoffs > csma.max_backoffs {
                return Err(RadioError::ChannelBusy);
            }
            be = (be + 1).min(csma.max_be);
        }
    }

    /// Look for a complete ACK with `sequence` among the queued frames
    /// without copying them. An ACK at the head of the queue is consumed;
    /// one behind other frames stays queued (and is later dropped by
    /// `receive`), so no received frame is lost.
    fn find_ack(&mut self, sequence: u8) -> Option<bool> {
        let rx = buf(1);
        let available = unsafe { read(rx + BUF_STATUS) } & BUF_STATUS_BYTES;
        let start = unsafe { read(rx + BUF_READOFFSET) } & BUF_OFFSET_MASK;
        let mut pos = 0u32;
        while pos < available {
            let offset = start + pos;
            let phr = u32::from(rx_ram_byte(offset)) & 0x7F;
            if phr < 5 {
                // Corrupt; `receive` resynchronises.
                return None;
            }
            let needed = 1 + (phr - 2) + RX_TRAILER_LEN as u32;
            if pos + needed > available {
                return None;
            }
            let fcf0 = rx_ram_byte(offset + 1);
            if phr == 5 && fcf0 & 0x07 == 0x02 && rx_ram_byte(offset + 3) == sequence {
                if pos == 0 {
                    unsafe { write(rx + BUF_READOFFSET, (start + needed) & BUF_OFFSET_MASK) };
                }
                return Some(fcf0 & 0x10 != 0);
            }
            pos += needed;
        }
        None
    }

    /// Fetch the next received frame into `out` (without FCS). Sends a
    /// software ACK first when the frame requests one and is addressed to
    /// this node (see [`set_ack_filter`](Self::set_ack_filter)).
    pub fn receive(&mut self, out: &mut [u8; MAX_FRAME]) -> Option<RxFrame> {
        service_sequencer();
        self.service_overflow();
        let frame = self.read_frame(out, true)?;
        self.stats.rx_frames += 1;
        if self.rx_enabled && needs_ack(&out[..frame.len], &self.ack_filter) {
            let ack = [0x02, 0x00, out[2]];
            if self.transmit_raw(&ack).is_ok() {
                self.stats.acks_sent += 1;
            }
            self.rx_enabled = false;
            let _ = self.rx_on();
        }
        Some(frame)
    }

    fn service_overflow(&mut self) {
        let flags = unsafe { read(L.frc + FRC_IF) };
        if flags & FRC_IF_RXOF != 0 || rac_state_raw() == RacState::RxOverflow {
            self.stats.rx_overflows += 1;
            unsafe {
                write(L.frc + FRC_CMD, FRC_CMD_RXABORT);
                write(buf(1) + BUF_CMD, BUF_CMD_CLEAR);
                write(L.rac + RAC_CMD, RAC_CMD_CLEARRXOVERFLOW);
                write(L.frc + FRC_IF + CLR, FRC_IF_RXOF);
            }
        }
    }

    /// Pop one complete frame from BUFC buffer 1. With `skip_acks`, ACK
    /// frames (already consumed by `transmit_csma` or stale) are dropped.
    fn read_frame(&mut self, out: &mut [u8; MAX_FRAME], skip_acks: bool) -> Option<RxFrame> {
        // Frames are peeked straight from the SRAM backing of buffer 1 and
        // consumed by advancing READOFFSET once complete. Reading through
        // READDATA and rewinding READOFFSET is not used: on xG22 the BUFC
        // read prefetch then returns a stale byte (hardware-observed).
        let rx = buf(1);
        loop {
            let available = unsafe { read(rx + BUF_STATUS) } & BUF_STATUS_BYTES;
            if available == 0 {
                return None;
            }
            let offset = unsafe { read(rx + BUF_READOFFSET) } & BUF_OFFSET_MASK;
            let phr = rx_ram_byte(offset) as usize & 0x7F;
            if phr < 5 {
                // Shorter than an ACK: corrupt length, resynchronise.
                unsafe { write(rx + BUF_CMD, BUF_CMD_CLEAR) };
                self.stats.rx_resyncs += 1;
                return None;
            }
            let stored = phr - 2;
            let needed = (1 + stored + RX_TRAILER_LEN) as u32;
            if available < needed {
                // Still arriving (or awaiting its CRC verdict).
                return None;
            }
            for (i, slot) in out.iter_mut().take(stored).enumerate() {
                *slot = rx_ram_byte(offset + 1 + i as u32);
            }
            let rssi = rssi_adjust(rx_ram_byte(offset + 1 + stored as u32) as i8 as i32);
            unsafe { write(rx + BUF_READOFFSET, (offset + needed) & BUF_OFFSET_MASK) };
            if skip_acks && stored == 3 && out[0] & 0x07 == 0x02 {
                continue;
            }
            return Some(RxFrame {
                len: stored,
                rssi_dbm: rssi,
                lqi: lqi_from_rssi(rssi),
            });
        }
    }

    /// Debug counters since init.
    pub fn stats(&self) -> RadioStats {
        self.stats
    }

    /// FRC status (debug).
    pub fn frc_status(&self) -> u32 {
        unsafe { read(L.frc + FRC_STATUS) }
    }

    /// SYNTH status (debug).
    pub fn synth_status(&self) -> u32 {
        unsafe { read(L.synth + SYNTH_STATUS) }
    }

    /// Bytes queued in the RX buffer (debug).
    pub fn rx_buffered(&self) -> u32 {
        unsafe { read(buf(1) + BUF_STATUS) & BUF_STATUS_BYTES }
    }

    /// Random 32-bit value (xorshift seeded from DEVINFO and RSSI noise;
    /// adequate for CSMA backoff, **not** for cryptography).
    pub fn next_random(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }
}

fn rac_state_raw() -> RacState {
    RacState::from_raw(((unsafe { read(L.rac + RAC_STATUS) } >> 24) & 0xF) as u8)
}

fn seed() -> u32 {
    // EUI-64 low word from DEVINFO plus the free-running cycle counter.
    let eui = unsafe { read(0x0FE0_8048) };
    let s = eui ^ crate::delay::cycles() ^ 0x9E37_79B9;
    if s == 0 { 0x1234_5678 } else { s }
}

/// Program every PHY, framing, buffer and interrupt register (not the
/// per-state analog values, TX power or channel).
fn program_phy() {
    crate::clock::enable_radio_clocks();
    unsafe {
        for &(address, value) in chip::INIT {
            write(address, value);
        }
        let ircal = read(chip::DEVINFO_IRCAL);
        if ircal != u32::MAX {
            write(chip::MODEM_IRCALCOEFWR0, ircal);
            write(chip::MODEM_IRCALCOEFWR1, ircal);
        }
        // One trailer byte per frame: the frame RSSI in dBm.
        write(L.frc + FRC_TRAILRXDATA, FRC_TRAILRXDATA_RSSI);
        write(L.bufc + BUFC_EN, 1);
        let tx = buf(0);
        write(tx + BUF_CTRL, TX_BUF_SIZE_CODE);
        write(tx + BUF_ADDR, TX_RAM.0.get() as u32);
        write(tx + BUF_CMD, BUF_CMD_CLEAR);
        let rx = buf(1);
        write(rx + BUF_CTRL, RX_BUF_SIZE_CODE);
        write(rx + BUF_ADDR, RX_RAM.0.get() as u32);
        write(rx + BUF_CMD, BUF_CMD_CLEAR);
        write(L.frc + FRC_IF + CLR, u32::MAX);
        // Take over the sequencer's state hand-shake: release the RSM
        // from SHUTDOWN now and service every later state change from
        // the RAC_RSM interrupt (and from this driver's busy-waits).
        write(L.rac + RAC_IF + CLR, u32::MAX);
        write(L.rac + RAC_IEN, RAC_IF_STATECHANGE);
        chip::seq_init();
        #[cfg(feature = "rt")]
        write(NVIC_ISER1, RAC_RSM_IRQ_BIT);
    }
}

/// Wait (bounded) for the RSM to leave SHUTDOWN after [`program_phy`].
fn leave_shutdown() -> Result<(), RadioError> {
    let deadline = Deadline::after_us(200);
    while matches!(rac_state_raw(), RacState::Shutdown) {
        service_sequencer();
        if deadline.expired() {
            return Err(RadioError::Shutdown);
        }
    }
    Ok(())
}

fn apply_state(state: &AnalogState) {
    unsafe {
        for (&address, &value) in state.regs.iter().zip(state.values.iter()) {
            write(address, value);
        }
        for &(address, value) in state.extra {
            write(address, value);
        }
    }
}

/// Release the radio state machine from its per-state sequencer wait.
///
/// The RSM stops in each state it enters until that state's sequencer work
/// is acknowledged (normally by RAIL's sequencer microcode, which this
/// driver does not run); see the chip module. Warm states stay held until
/// [`warm_up`] releases them. Bounded and idempotent: safe from the
/// `RAC_RSM` interrupt and from thread context.
pub fn service_sequencer() {
    chip::seq_service();
}

/// Wait up to `us` for the RSM to reach `OFF`, servicing the sequencer and
/// releasing the warm states [`service_sequencer`] holds, so a RSM parked
/// in `RXWARM`/`TXWARM` can still be disabled.
fn wait_off_releasing(us: u32) -> bool {
    let deadline = Deadline::after_us(us);
    loop {
        chip::seq_release(Warm::Rx);
        chip::seq_release(Warm::Tx);
        chip::seq_release(Warm::TxPd);
        service_sequencer();
        if rac_state_raw() == RacState::Off {
            return true;
        }
        if deadline.expired() {
            return false;
        }
    }
}

/// Release a held warm state and make sure the RSM actually leaves it.
/// On xG22 the `SEQIF` flag can be raised a moment after `STATUS` already
/// shows the state; a single clear issued in that window is lost and the
/// RSM stays parked in the warm state (hardware-observed), so keep
/// releasing until the state changes.
fn release_warm(state: RacState, warm: Warm) {
    let deadline = Deadline::after_us(50);
    loop {
        chip::seq_release(warm);
        if rac_state_raw() != state || deadline.expired() {
            return;
        }
    }
}

/// Wait for the RSM to reach `state` (a warm state held by
/// [`service_sequencer`]), lock the synthesizer, then release the state.
/// The warm state is released even on failure so the RSM can be disabled.
fn warm_up(state: RacState, warm: Warm, rx_if: bool, cached: &mut u16) -> Result<(), RadioError> {
    let deadline = Deadline::after_us(SYNTH_TIMEOUT_US);
    while rac_state_raw() != state {
        service_sequencer();
        if deadline.expired() {
            chip::seq_release(warm);
            set_diag(SynthDiag {
                stage: 1,
                // Raw RSM state the warm-up stalled in.
                count: unsafe { read(L.rac + RAC_STATUS) } >> 24 & 0xF,
                ..SynthDiag::default()
            });
            return Err(RadioError::SynthLock);
        }
    }
    spin_cycles(us_to_cycles(WARM_SETTLE_US));
    let result = start_synth(rx_if, cached);
    if result.is_ok() && matches!(warm, Warm::Tx) {
        chip::pa_ramp_up();
    }
    release_warm(state, warm);
    result
}

/// Ramp the PA down in the held `TXPD` state, then let the RSM leave it.
/// Waits at most [`TXPD_WAIT_US`] for the RSM to reach `TXPD` (an aborted
/// transmission may never get there); the state is released either way.
fn tx_power_down() {
    if !chip::PA_RAMP_IN_TXPD {
        return;
    }
    let deadline = Deadline::after_us(TXPD_WAIT_US);
    while rac_state_raw() != RacState::TxPowerDown && !deadline.expired() {
        service_sequencer();
    }
    if rac_state_raw() == RacState::TxPowerDown {
        chip::pa_ramp_down();
    }
    chip::seq_release(Warm::TxPd);
}

/// `RAC_RSM` interrupt: radio state change.
#[cfg(all(feature = "rt", target_arch = "arm", target_os = "none"))]
#[unsafe(no_mangle)]
extern "C" fn RAC_RSM() {
    service_sequencer();
}

/// LO-counter target for the programmed channel, matching the hardware's
/// `SYNTH.LOCNTTARGET` at `NUMCYCLE = 4`: `(FREQ ± IFFREQ) >> 13`.
pub(crate) const fn locnt_target(freq: u32, iffreq_reg: u32, rx_if: bool) -> u32 {
    let mut lo = freq;
    if rx_if {
        let iffreq = iffreq_reg & 0xF_FFFF;
        if iffreq_reg & (1 << 20) != 0 {
            lo += iffreq;
        } else {
            lo -= iffreq;
        }
    }
    lo >> 13
}

/// Count LO cycles over 2^NUMCYCLE reference periods.
fn lo_count() -> Option<u32> {
    unsafe {
        write(L.synth + SYNTH_IF + CLR, SYNTH_IF_LOCNTDONE);
        write(L.synth + SYNTH_LOCNTCTRL, LOCNT_BASE | LOCNT_CLEAR);
        write(L.synth + SYNTH_LOCNTCTRL, LOCNT_BASE | LOCNT_RUN);
    }
    let deadline = Deadline::after_us(20);
    while unsafe { read(L.synth + SYNTH_IF) } & SYNTH_IF_LOCNTDONE == 0 {
        if deadline.expired() {
            return None;
        }
    }
    // READ latches the count across the LO clock domain; it shows up in
    // LOCNTSTATUS a few reference cycles later.
    unsafe {
        write(
            L.synth + SYNTH_LOCNTCTRL,
            LOCNT_BASE | LOCNT_RUN | LOCNT_READ,
        )
    };
    let deadline = Deadline::after_us(10);
    loop {
        let status = unsafe { read(L.synth + SYNTH_LOCNTSTATUS) };
        let count = status & LOCNT_COUNT_MASK;
        if status & LOCNT_BUSY == 0 && count != 0 {
            return Some(count);
        }
        if deadline.expired() {
            return None;
        }
    }
}

/// Last synthesizer calibration, for lab diagnostics.
#[derive(Debug, Clone, Copy, Default)]
pub struct SynthDiag {
    /// LO-counter target (`(FREQ ± IF) >> 13`).
    pub target: u32,
    /// Selected `VCOTUNING` fine code.
    pub tuning: u32,
    /// Last LO count (open loop during search, closed loop after start).
    pub count: u32,
    /// 0 = ok, 1 = warm state not reached, 2 = counter timeout, 3 = no lock.
    pub stage: u8,
    /// The cached tuning code was reused (no search).
    pub cached: bool,
}

static mut SYNTH_DIAG: SynthDiag = SynthDiag {
    target: 0,
    tuning: 0,
    count: 0,
    stage: 0,
    cached: false,
};

/// Diagnostics of the most recent synthesizer calibration.
pub fn synth_diag() -> SynthDiag {
    unsafe { core::ptr::read_volatile(&raw const SYNTH_DIAG) }
}

/// Number of PHY configuration registers written by [`RadioToken::init`].
pub fn config_register_count() -> usize {
    chip::INIT.len()
}

/// Address and current value of PHY configuration register `index`, for
/// lab retention checks across EM2. Reads have no side effects.
pub fn config_register(index: usize) -> Option<(u32, u32)> {
    chip::INIT
        .get(index)
        .map(|&(address, _)| (address, unsafe { read(address) }))
}

fn set_diag(diag: SynthDiag) {
    unsafe { core::ptr::write_volatile(&raw mut SYNTH_DIAG, diag) };
}

/// Calibrate the VCO for the programmed channel and lock the synthesizer.
/// `rx_if` selects the RX IF offset (`ENABLEIF`) or direct TX tuning.
///
/// `cached` holds the last good `VCOTUNING` for this channel and direction
/// (`0` = none). A cached code is tried first and accepted only if the
/// closed loop then counts within `LOCK_TOLERANCE` of the target; otherwise
/// the full search runs. The cache cuts RX->TX turnaround (software ACK)
/// by the ~11 search steps.
fn start_synth(rx_if: bool, cached: &mut u16) -> Result<(), RadioError> {
    unsafe {
        write(L.synth + SYNTH_CMD, SYNTH_CMD_STOP);
        write(
            L.synth + SYNTH_CMD,
            if rx_if {
                SYNTH_CMD_ENABLEIF
            } else {
                SYNTH_CMD_DISABLEIF
            },
        );
    }
    let target = unsafe {
        locnt_target(
            read(L.synth + SYNTH_FREQ),
            read(L.synth + SYNTH_IFFREQ),
            rx_if,
        )
    };
    let mut diag = SynthDiag {
        target,
        ..SynthDiag::default()
    };
    if *cached != 0 {
        diag.tuning = u32::from(*cached);
        if close_loop(target, diag.tuning, &mut diag, CACHED_LOCK_US) {
            diag.stage = 0;
            diag.cached = true;
            set_diag(diag);
            return Ok(());
        }
        *cached = 0;
        unsafe { write(L.synth + SYNTH_CMD, SYNTH_CMD_STOP) };
    }
    // Open-loop frequency falls monotonically with VCOTUNING: find the
    // smallest setting whose count does not exceed the target. The VCO
    // control voltage is pinned by the VCDAC meanwhile (see `search_vco`).
    let vcdac = unsafe { read(L.synth + SYNTH_VCDACCTRL) };
    unsafe { write(L.synth + SYNTH_VCDACCTRL, vcdac | VCDACCTRL_VCDACEN) };
    let found = search_vco(target, &mut diag);
    unsafe { write(L.synth + SYNTH_VCDACCTRL, vcdac) };
    let Some(low) = found else {
        diag.stage = 2;
        set_diag(diag);
        return Err(RadioError::SynthLock);
    };
    diag.tuning = low;
    if close_loop(target, low, &mut diag, SYNTH_TIMEOUT_US) {
        diag.stage = 0;
        set_diag(diag);
        *cached = low as u16;
        Ok(())
    } else {
        diag.stage = 3;
        set_diag(diag);
        Err(RadioError::SynthLock)
    }
}

/// Binary-search the open-loop `VCOTUNING` code whose LO count is closest to
/// `target` from below; `None` if the LO counter does not complete.
///
/// Must run with `VCDACCTRL.VCDACEN` set: the DAC then holds the VCO control
/// voltage at `VCDACVAL` (mid-range), so the code found is the one at which the
/// closed loop will settle in the middle of its tuning range. Without it the
/// control voltage floats at whatever the previous lock left on the loop
/// filter, which makes the code depend on radio history (hardware-observed on
/// xG21: an ED sweep before the first TX biased channel 15 by +14 codes, the
/// VCO range logic then stepped `VCAPSEL` 16 -> 13 on every relock, and RX
/// after TX stayed deaf for ACKs until the loop recovered, ~700 us later).
fn search_vco(target: u32, diag: &mut SynthDiag) -> Option<u32> {
    let (mut low, mut high) = (0u32, VCOTUNING_MAX);
    while low < high {
        let mid = (low + high) / 2;
        unsafe { write(L.synth + SYNTH_VCOTUNING, VCAPSEL_DEFAULT | mid) };
        spin_cycles(us_to_cycles(VCO_SETTLE_US));
        let count = lo_count()?;
        diag.count = count;
        if count > target {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    Some(low)
}

/// Program `tuning`, start the synthesizer loop and wait up to `timeout_us`
/// for the LO count to settle within `LOCK_TOLERANCE` of `target`.
fn close_loop(target: u32, tuning: u32, diag: &mut SynthDiag, timeout_us: u32) -> bool {
    unsafe {
        write(L.synth + SYNTH_VCOTUNING, VCAPSEL_DEFAULT | tuning);
        write(L.synth + SYNTH_CMD, SYNTH_CMD_START);
    }
    let deadline = Deadline::after_us(timeout_us);
    loop {
        if let Some(count) = lo_count() {
            diag.count = count;
            if count.abs_diff(target) <= LOCK_TOLERANCE {
                return true;
            }
        }
        if deadline.expired() {
            return false;
        }
    }
}

/// `SYNTH.FREQ` for an O-QPSK channel: `f / 38.4 MHz * 2^19`.
pub const fn synth_freq(channel: u8) -> Option<u32> {
    if channel < 11 || channel > 26 {
        return None;
    }
    let hz = 2_405_000_000u64 + 5_000_000u64 * (channel as u64 - 11);
    Some(((hz << 19) / 38_400_000) as u32)
}

fn tx_power_entry(ddbm: i16) -> (i16, u8, u16) {
    let mut chosen = chip::TX_POWER[0];
    for &entry in chip::TX_POWER {
        if entry.0 <= ddbm {
            chosen = entry;
        }
    }
    chosen
}

/// Decode `AGC.RSSI`/`AGC.FRAMERSSI` (`[15:6]`, signed quarter-dBm) into
/// dBm, applying the chip's RSSI adjustment. `None` while invalid (0x8000).
pub const fn rssi_from_reg(raw: u32) -> Option<i8> {
    let quarter = (((raw >> 6) & 0x3FF) as i32) << 22 >> 22;
    if quarter == -512 {
        return None;
    }
    Some(rssi_adjust(quarter >> 2))
}

/// Apply [`RSSI_ADJUST_DBM`] to a raw integer RSSI (AGC or RX trailer byte).
pub const fn rssi_adjust(raw_dbm: i32) -> i8 {
    let dbm = raw_dbm + RSSI_ADJUST_DBM as i32;
    if dbm < -128 {
        -128
    } else if dbm > 127 {
        127
    } else {
        dbm as i8
    }
}

/// Linear RSSI→LQI map (-100 dBm → 0, -20 dBm → 255).
pub const fn lqi_from_rssi(rssi: i8) -> u8 {
    let clamped = if rssi < -100 {
        -100
    } else if rssi > -20 {
        -20
    } else {
        rssi as i32
    };
    (((clamped + 100) * 255) / 80) as u8
}

/// Whether a received MPDU (without FCS) requests an ACK from this node.
pub fn needs_ack(frame: &[u8], filter: &AckFilter) -> bool {
    if frame.len() < 3 {
        return false;
    }
    let fcf = u16::from_le_bytes([frame[0], frame[1]]);
    let frame_type = fcf & 0x7;
    if fcf & (1 << 5) == 0 || !(frame_type == 1 || frame_type == 3) {
        return false;
    }
    let dst_mode = (fcf >> 10) & 0x3;
    if frame.len() < 5 {
        return false;
    }
    let dst_pan = u16::from_le_bytes([frame[3], frame[4]]);
    let pan_ok = dst_pan == 0xFFFF || filter.pan_id == Some(dst_pan);
    match dst_mode {
        2 if frame.len() >= 7 => {
            let short = u16::from_le_bytes([frame[5], frame[6]]);
            pan_ok && short != 0xFFFF && filter.short_address == Some(short)
        }
        3 if frame.len() >= 13 => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&frame[5..13]);
            pan_ok && filter.extended_address == Some(u64::from_le_bytes(ext))
        }
        _ => false,
    }
}

fn rx_ram_byte(offset: u32) -> u8 {
    let base = RX_RAM.0.get() as *const u8;
    // SAFETY: the index is reduced modulo the buffer length; the bytes were
    // written by BUFC before STATUS reported them available.
    unsafe { core::ptr::read_volatile(base.add((offset % RX_BUF_LEN) as usize)) }
}

fn buf(index: u32) -> u32 {
    L.bufc + L.bufc_buf0 + index * L.bufc_buf_stride
}

const CLR: u32 = 0x2000;

#[inline]
unsafe fn read(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline]
unsafe fn write(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_frequency_words() {
        assert_eq!(synth_freq(15), Some(0x01F9_3555));
        assert_eq!(synth_freq(10), None);
        assert_eq!(synth_freq(27), None);
        assert!(synth_freq(26).unwrap() > synth_freq(11).unwrap());
    }

    #[test]
    fn tx_power_range_follows_the_pa_table() {
        #[cfg(feature = "efr32mg22")]
        assert_eq!(TX_POWER_RANGE_DBM, (-25, 6));
        #[cfg(feature = "efr32mg21")]
        assert_eq!(TX_POWER_RANGE_DBM, (-30, 20));
    }

    #[test]
    fn rssi_decoding() {
        assert_eq!(rssi_from_reg(0x8000), None);
        // Raw -56 / -29 dBm; RAIL reports these as -106 / -79 dBm on xG22
        // (hardware ground truth, RAIL_GetRssi on the same kit).
        assert_eq!(rssi_from_reg(0xC800), Some(-56 + RSSI_ADJUST_DBM));
        assert_eq!(rssi_from_reg(0xE380), Some(-29 + RSSI_ADJUST_DBM));
        assert_eq!(rssi_adjust(-128), -128);
    }

    #[test]
    fn lqi_is_monotonic_and_clamped() {
        assert_eq!(lqi_from_rssi(-120), 0);
        assert_eq!(lqi_from_rssi(0), 255);
        assert!(lqi_from_rssi(-60) > lqi_from_rssi(-80));
    }

    #[test]
    fn ack_decision() {
        let filter = AckFilter {
            pan_id: Some(0xDFE9),
            short_address: Some(0x1234),
            extended_address: Some(0x0102_0304_0506_0708),
        };
        // Data, ack-req, PAN-compressed, short dst/src.
        let to_me = [0x61, 0x88, 0x07, 0xE9, 0xDF, 0x34, 0x12, 0x00, 0x00];
        assert!(needs_ack(&to_me, &filter));
        let to_other = [0x61, 0x88, 0x07, 0xE9, 0xDF, 0x35, 0x12, 0x00, 0x00];
        assert!(!needs_ack(&to_other, &filter));
        let no_ack_req = [0x41, 0x88, 0x07, 0xE9, 0xDF, 0x34, 0x12, 0x00, 0x00];
        assert!(!needs_ack(&no_ack_req, &filter));
        let broadcast = [0x61, 0x88, 0x07, 0xE9, 0xDF, 0xFF, 0xFF, 0x00, 0x00];
        assert!(!needs_ack(&broadcast, &filter));
        // Association response to our extended address (command frame).
        let mut ext = [0u8; 13];
        ext[0] = 0x63;
        ext[1] = 0xCC;
        ext[3] = 0xE9;
        ext[4] = 0xDF;
        ext[5..13].copy_from_slice(&0x0102_0304_0506_0708u64.to_le_bytes());
        assert!(needs_ack(&ext, &filter));
    }

    #[test]
    #[cfg(feature = "efr32mg22")]
    fn power_table_selection() {
        assert_eq!(tx_power_entry(0), (0, 0x2F, 0x444));
        assert_eq!(tx_power_entry(5), (0, 0x2F, 0x444));
        assert_eq!(tx_power_entry(200), (60, 0x4F, 0x333));
        assert_eq!(tx_power_entry(-1000).1, 0x20);
    }

    #[test]
    fn locnt_target_matches_rail_reference() {
        // xG22 channel 15 snapshots: RX LOCNTTARGET 0xFCB, TX (CW) 0xFC9.
        let freq = synth_freq(15).unwrap();
        assert_eq!(freq, 0x01F9_3555);
        assert_eq!(locnt_target(freq, 0x0010_4911, true), 0xFCB);
        assert_eq!(locnt_target(freq, 0x0010_4911, false), 0xFC9);
    }

    #[test]
    #[cfg(feature = "efr32mg22")]
    fn init_table_is_sorted_and_gates_sequencer() {
        let table = chip::INIT;
        assert!(table.windows(2).all(|w| w[0].0 < w[1].0));
        let rac_ctrl = table.iter().find(|e| e.0 == 0xB802_0014).unwrap().1;
        assert_ne!(rac_ctrl & (1 << 27), 0, "SEQCLKDIS must be set");
        assert_eq!(rac_ctrl & (1 << 26), 0, "CPUWAITDIS would stall the RSM");
        assert_eq!(chip::LAYOUT.bufc, 0xBA00_0000);
    }

    #[test]
    #[cfg(feature = "efr32mg21")]
    fn xg21_tables() {
        let table = chip::INIT;
        assert!(table.windows(2).all(|w| w[0].0 < w[1].0));
        // The sequencer is configured by `seq_init`, never from RAIL state.
        for reg in [0xB802_0064, 0xB802_0068] {
            assert!(table.iter().all(|e| e.0 != reg));
        }
        // Per-state registers are not duplicated in the static table.
        for reg in chip::IDLE.regs {
            assert!(table.iter().all(|e| e.0 != *reg));
        }
        assert!(chip::TX_POWER.windows(2).all(|w| w[0].0 < w[1].0));
        assert_eq!(tx_power_entry(100), (100, 0xDC, 0x37));
        assert_eq!(tx_power_entry(105), (100, 0xDC, 0x37));
        assert_eq!(tx_power_entry(-1000).0, -300);
        assert_eq!(chip::LAYOUT.bufc, 0x5400_4000);
    }
}
