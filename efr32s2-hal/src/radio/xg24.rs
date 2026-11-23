//! xG24 (EFR32MG24) IEEE 802.15.4 2.4 GHz O-QPSK tables — **not available
//! yet**: [`SUPPORTED`] is `false`, so `RadioToken::init` returns
//! `RadioError::Unsupported` before touching any radio register.
//!
//! What is known (GSDK 4.5.0 radio configurator register map
//! `efr32xg24/revA1`, compared field by field with `efr32xg22/revA2`):
//! - FRC/AGC/MODEM/SYNTH/RAC/RFCRC sit at the xG22 addresses, and every
//!   offset the shared driver uses is unchanged (FRC STATUS/CMD/IF/IEN/
//!   TRAILRXDATA, RAC RXENSRCEN/STATUS/CMD/IF/IEN/SEQIF, SYNTH FREQ/IFFREQ/
//!   CHCTRL/VCOTUNING/VCDACCTRL/IF/LOCNTCTRL/LOCNTSTATUS, AGC RSSI/FRAMERSSI).
//! - The radio state machine has the xG22 hand-shake: `RAC.CTRL.SEQCLKDIS`
//!   and `RAC.SEQIF` with identical state bits, so the clock-gated
//!   sequencer approach of `xg22.rs` applies (the functions below are the
//!   xG22 ones).
//! - `SYNTH.LOCNTCTRL.FCALRUNCLKEN` is bit 11 as on xG22.
//! - `MODEM.RAMPCTRL`/`RAMPLEV` moved to 0xD8/0xDC and
//!   `MODEM.IRCALCOEFWR0/1` to 0x278/0x27C.
//! - The analog front end is different: the RAC analog registers moved,
//!   `RAC.RFPATHEN` and `RAC.AGCOVERWRITE` no longer exist, and the PA has
//!   a 10 dBm (MP) and a 20 dBm (HP) variant.
//! - GSDK ships the 802.15.4 PHY as a radio-configurator table
//!   (`rail_util_built_in_phys/efr32xg24/sl_rail_ieee802154_config_{38M4,39,40}MHz.c`),
//!   usable as a cross-check of a capture.
//!
//! What is missing, and why the radio stays disabled: the per-state analog
//! values (idle/RX/TX) that the RAIL sequencer microcode writes, the BUFC
//! base and buffer stride, the PA ramp/power programming of both PA
//! variants, the RSSI offset and the DEVINFO IR-calibration word. On xG21
//! and xG22 these came from SWD register snapshots of RAIL on real silicon
//! (see `docs/book/src/platform-guides/efr32.md`); the same capture is the
//! first hardware gate on the RF-BM-MG24B1 module. Nothing here is guessed
//! from another die.

use super::{AnalogState, CLR, L, Layout, RAC_IF, RAC_IF_STATECHANGE, Warm, read, write};

pub(super) const SUPPORTED: bool = false;

/// `FRC_IRQn` and `RAC_RSM_IRQn` (GSDK 4.5.0 `efr32mg24*.h`).
#[cfg_attr(not(feature = "isr-ack"), allow(dead_code))]
pub(super) const FRC_IRQN: u32 = 33;
#[cfg_attr(not(feature = "rt"), allow(dead_code))]
pub(super) const RAC_RSM_IRQN: u32 = 36;

/// FRC/AGC/MODEM/SYNTH/RAC from the register map. `bufc`, `bufc_buf0` and
/// `bufc_buf_stride` are **unknown** (0) until captured; nothing reads them
/// while [`SUPPORTED`] is false.
pub(super) const LAYOUT: Layout = Layout {
    frc: 0xB800_4000,
    agc: 0xB800_C000,
    modem: 0xB801_4000,
    synth: 0xB801_8000,
    rac: 0xB802_0000,
    bufc: 0,
    bufc_buf0: 0,
    bufc_buf_stride: 0,
    agc_rssi: 0x18,
};

/// DEVINFO IR-calibration word: **unknown** on xG24 (0 = not programmed,
/// so the driver would keep the table value).
pub(super) const DEVINFO_IRCAL: u32 = 0;
/// RSSI adjustment: **unknown** until compared with `RAIL_GetRssi`.
pub(super) const RSSI_ADJUST_DBM: i8 = 0;
pub(super) const MODEM_IRCALCOEFWR0: u32 = 0xB801_4278;
pub(super) const MODEM_IRCALCOEFWR1: u32 = 0xB801_427C;

/// `SYNTH.LOCNTCTRL.FCALRUNCLKEN` (bit 11, as on xG22).
pub(super) const LOCNT_FCALRUNCLKEN: u32 = 1 << 11;

/// Static PHY configuration: empty until captured.
pub(super) const INIT: &[(u32, u32)] = &[];

/// Per-state analog registers: none known yet.
pub(super) const STATE_REGS: usize = 0;

pub(super) const IDLE: AnalogState = AnalogState {
    regs: &[],
    values: [],
    extra: &[],
};
pub(super) const RX: AnalogState = AnalogState {
    regs: &[],
    values: [],
    extra: &[],
};
pub(super) const TX: AnalogState = AnalogState {
    regs: &[],
    values: [],
    extra: &[],
};

/// No validated PA table. The single 0 dBm entry only keeps the shared
/// driver's table lookup total, so the image links and is sized like a
/// working one (an empty table would make every `Ok` path of
/// `RadioToken::init` provably panic, and the optimiser would drop the
/// whole stack). Its ramp values are never programmed while [`SUPPORTED`]
/// is false; `TX_POWER_RANGE_DBM` is `(0, 0)`.
pub(super) const TX_POWER: &[(i16, u8, u16)] = &[(0, 0, 0)];

/// Unreachable while [`SUPPORTED`] is false (`MODEM.RAMPCTRL`/`RAMPLEV`
/// moved to 0xD8/0xDC on xG24; the PA programming itself is unknown).
pub(super) fn write_tx_power(_ramplev: u8, _rampctrl: u16) {
    let _modem = L.modem;
}

#[inline(always)]
pub(super) fn tx_state_power(_ddbm: i16) {}

pub(super) const PA_RAMP_IN_TXPD: bool = false;

#[inline(always)]
pub(super) fn pa_ramp_up() {}

#[inline(always)]
pub(super) fn pa_ramp_down() {}

// ---- Radio state hand-shake (identical to xG22, see module docs) -------

const RAC_SEQIF: u32 = 0x28;
const RAC_SEQIF_ALL: u32 = 0x3FFF_000F;
const RAC_SEQIF_RXWARM: u32 = 1 << 17;
const RAC_SEQIF_TXWARM: u32 = 1 << 24;
const RAC_SEQIF_HELD: u32 = RAC_SEQIF_RXWARM | RAC_SEQIF_TXWARM;

const fn seqif(warm: Warm) -> u32 {
    match warm {
        Warm::Rx => RAC_SEQIF_RXWARM,
        Warm::Tx => RAC_SEQIF_TXWARM,
        Warm::TxPd => 0,
    }
}

pub(super) fn seq_init() {
    unsafe { write(L.rac + RAC_SEQIF + CLR, RAC_SEQIF_ALL) };
}

pub(super) fn seq_service() {
    unsafe {
        write(L.rac + RAC_IF + CLR, RAC_IF_STATECHANGE);
        let pending = read(L.rac + RAC_SEQIF) & RAC_SEQIF_ALL & !RAC_SEQIF_HELD;
        if pending != 0 {
            write(L.rac + RAC_SEQIF + CLR, pending);
        }
    }
}

#[inline(always)]
pub(super) fn seq_arm(_warm: Warm) {}

pub(super) fn seq_release(warm: Warm) {
    unsafe { write(L.rac + RAC_SEQIF + CLR, seqif(warm)) };
}
