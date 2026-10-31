//! xG21 (EFR32MG21) radio tables — not yet captured.
//!
//! The register *layout* below is taken from the GSDK 4.5.0 radio
//! configurator register map for `efr32xg21` C0, but the PHY settings and
//! the per-state analog values have not been captured from an xG21 die yet
//! (they differ from xG22: different PA, SYNTH and AGC block revisions).
//! [`Radio::init`](super::RadioToken::init) therefore fails with
//! [`RadioError::Unsupported`](super::RadioError::Unsupported) on MG21 rather
//! than programming xG22 values into an xG21 radio.

use super::{AnalogState, Layout};

pub(super) const SUPPORTED: bool = false;

pub(super) const LAYOUT: Layout = Layout {
    frc: 0xB800_4000,
    agc: 0xB800_C000,
    modem: 0xB801_4000,
    synth: 0xB801_8000,
    rac: 0xB802_0000,
    bufc: 0x5400_4000,
    bufc_buf0: 0x08,
    bufc_buf_stride: 0x30,
    agc_rssi: 0x10,
    modem_rampctrl: 0xC0,
    modem_ramplev: 0xC4,
};

pub(super) const DEVINFO_IRCAL: u32 = 0;

/// RSSI adjustment (dB). Not yet measured on xG21 (the radio is
/// unsupported until its tables are captured); GSDK's application offset
/// `SL_RAIL_UTIL_RSSI_OFFSET` is -11 dB on xG21, on top of RAIL's chip term.
pub(super) const RSSI_ADJUST_DBM: i8 = 0;
pub(super) const MODEM_IRCALCOEFWR0: u32 = 0;
pub(super) const MODEM_IRCALCOEFWR1: u32 = 0;

pub(super) const INIT: &[(u32, u32)] = &[];

const NO_REGS: [u32; 16] = [0; 16];
const EMPTY: AnalogState = AnalogState {
    regs: &NO_REGS,
    values: [0; 16],
    synth: [(0, 0); 2],
};
pub(super) const IDLE: AnalogState = EMPTY;
pub(super) const RX: AnalogState = EMPTY;
pub(super) const TX: AnalogState = EMPTY;

pub(super) const TX_POWER: &[(i16, u8, u16)] = &[(0, 0, 0)];
