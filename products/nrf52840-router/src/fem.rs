//! Transmit-power policy of the AT2401C front-end variant
//! (`fem-at2401c`). The board crate describes the fitted part (pins, gains,
//! settle times); the product owns what may be radiated.
//!
//! **Hardware-unverified:** no FEM board has run this configuration.
//! Re-derive both values for the actual antenna gain and target market
//! before shipping.

/// Regulatory cap on the conducted output at the antenna port (dBm).
///
/// ETSI EN 300 328 limits 2.4 GHz wideband transmitters to 20 dBm EIRP;
/// 18 dBm conducted leaves 2 dB for the antenna gain. The radio is never
/// programmed so that `radio + PA gain` exceeds this value.
pub const MAX_OUTPUT_DBM: i8 = 18;

/// `phyTransmitPower` installed at start and restored by MLME-RESET
/// (antenna-referred). With the add-on's 22 dB PA this programs the radio
/// at -4 dBm.
pub const DEFAULT_OUTPUT_DBM: i8 = MAX_OUTPUT_DBM;

const _: () = assert!(DEFAULT_OUTPUT_DBM <= MAX_OUTPUT_DBM);
// ETSI EN 300 328: 20 dBm EIRP for 2.4 GHz wideband transmitters.
const _: () = assert!(MAX_OUTPUT_DBM <= 20);
