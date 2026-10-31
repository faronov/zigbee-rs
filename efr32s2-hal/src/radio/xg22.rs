//! xG22 (EFR32MG22, die revision A2) IEEE 802.15.4 2.4 GHz O-QPSK tables.
//!
//! Provenance: register snapshots taken over SWD from an EFR32MG22C224F512IM32
//! (BRD4183A) running GSDK 4.5.0 RAIL with
//! `RAIL_IEEE802154_Config2p4GHzRadio()` (lab-only instrument, not shipped),
//! reduced to the read-write registers whose value differs from the GSDK
//! register-map reset value. Field layouts were checked against the GSDK
//! radio configurator register map (`efr32xg22` A2). The values are register
//! *settings*, not code: no RAIL object code or sequencer microcode is used.
//!
//! Deviations from the snapshot:
//! - `RAC.CTRL` sets `SEQCLKDIS` (bit 27) and clears `CPUWAITDIS` (bit 26):
//!   the radio sequencer is clock-gated, the radio state machine does not
//!   wait for sequencer acknowledgement of state changes, and the CPU
//!   performs the per-state analog switching below. (RAIL uses `0x04000380`:
//!   sequencer clocked, `CPUWAITDIS` set.)
//! - `FRC.TRAILRXDATA` is cleared after the table (no RX trailer bytes; RSSI
//!   is read from `AGC.FRAMERSSI`).
//! - `MODEM.IRCALCOEFWR0/1` are replaced by the per-die DEVINFO word at
//!   [`DEVINFO_IRCAL`] when it is programmed.

use super::{AnalogState, CLR, L, Layout, RAC_IF, RAC_IF_STATECHANGE, Warm, read, write};

pub(super) const SUPPORTED: bool = true;

pub(super) const LAYOUT: Layout = Layout {
    frc: 0xB800_4000,
    agc: 0xB800_C000,
    modem: 0xB801_4000,
    synth: 0xB801_8000,
    rac: 0xB802_0000,
    bufc: 0xBA00_0000,
    bufc_buf0: 0x0C,
    bufc_buf_stride: 0x40,
    agc_rssi: 0x18,
};

/// DEVINFO word holding the 2.4 GHz IR-calibration coefficients.
pub(super) const DEVINFO_IRCAL: u32 = 0x0FE0_8248;

/// RSSI adjustment (dB) applied to `AGC.RSSI`/`FRAMERSSI` and the RX trailer
/// byte. RAIL (GSDK 4.5 librail, `RAIL_GetRssiAlt`) subtracts 200
/// quarter-dB from `AGC.RSSI[15:6]` on xG22 with this radio configuration;
/// the result was checked against `RAIL_GetRssi` and RAIL packet RSSI on
/// BRD4183A (idle -56 raw -> -106 dBm, RAIL -106.25 dBm). The application
/// RSSI offset (`SL_RAIL_UTIL_RSSI_OFFSET`) is 0 on xG22.
pub(super) const RSSI_ADJUST_DBM: i8 = -50;
pub(super) const MODEM_IRCALCOEFWR0: u32 = 0xB801_4288;
pub(super) const MODEM_IRCALCOEFWR1: u32 = 0xB801_428C;

/// `SYNTH.LOCNTCTRL.FCALRUNCLKEN`.
pub(super) const LOCNT_FCALRUNCLKEN: u32 = 1 << 11;
/// MODEM.RAMPCTRL / RAMPLEV offsets (xG22 `MODEM_TypeDef`).
const MODEM_RAMPCTRL: u32 = 0xC4;
const MODEM_RAMPLEV: u32 = 0xCC;

/// Static PHY + MAC-framing configuration, in ascending address order.
pub(super) const INIT: &[(u32, u32)] = &[
    (0xB8004004, 0x00000001), // FRC.EN
    (0xB800400C, 0x00148001), // FRC.DFLCTRL
    (0xB8004010, 0x0000407F), // FRC.MAXLENGTH
    (0xB8004020, 0x000000FF), // FRC.WCNTCMP0
    (0xB8004028, 0x00000004), // FRC.WCNTCMP2
    (0xB8004040, 0x00000000), // FRC.BLOCKRAMADDR
    (0xB8004048, 0x000407A0), // FRC.CTRL
    (0xB800404C, 0x00000060), // FRC.RXCTRL
    (0xB8004054, 0x0000001B), // FRC.TRAILRXDATA
    (0xB8004060, 0x00000000), // FRC.PUNCTCTRL
    (0xB8004084, 0x000000F8), // FRC.SNIFFCTRL
    (0xB80040A8, 0x00000007), // FRC.AUTOCG
    (0xB80040CC, 0x01002000), // FRC.PKTBUFCTRL
    (0xB8004104, 0x00004000), // FRC.FCD0
    (0xB8004108, 0x00004CFF), // FRC.FCD1
    (0xB800410C, 0x00005DFF), // FRC.FCD2
    (0xB8004110, 0x00004DFF), // FRC.FCD3
    (0xB800C004, 0x00000001), // AGC.EN
    (0xB800C020, 0x00093AF5), // AGC.CTRL0
    (0xB800C024, 0x00001800), // AGC.CTRL1
    (0xB800C028, 0x83B380EC), // AGC.CTRL2
    (0xB800C02C, 0x51407543), // AGC.CTRL3
    (0xB800C030, 0x48000FA0), // AGC.CTRL4
    (0xB800C034, 0x00004030), // AGC.CTRL5
    (0xB800C040, 0x0000022E), // AGC.MANGAIN
    (0xB800C050, 0x04301151), // AGC.GAINRANGE
    (0xB800C054, 0xE6092D0E), // AGC.AGCPERIOD
    (0xB800C058, 0x08070654), // AGC.HICNTREGION
    (0xB800C05C, 0x0002B6D1), // AGC.STEPDWN
    (0xB800C064, 0x1C003004), // AGC.GAINSTEPLIM
    (0xB800C068, 0x09183040), // AGC.PNRFATT0
    (0xB800C06C, 0x2079640D), // AGC.PNRFATT1
    (0xB800C070, 0x01FBFCEB), // AGC.PNRFATT2
    (0xB800C08C, 0x00000003), // AGC.MIRRORIF
    (0xB8010004, 0x00000001), // RFCRC.EN
    (0xB8010020, 0x00008408), // RFCRC.POLY
    (0xB8014004, 0x00000001), // MODEM.EN
    (0xB801401C, 0x00000010), // MODEM.MIXCTRL
    (0xB8014020, 0x0413F920), // MODEM.CTRL0
    (0xB8014024, 0x0052C007), // MODEM.CTRL1
    (0xB8014028, 0x00000000), // MODEM.CTRL2
    (0xB801402C, 0x00000000), // MODEM.CTRL3
    (0xB8014058, 0x00FF0264), // MODEM.TXBR
    (0xB801405C, 0x00000841), // MODEM.RXBR
    (0xB8014060, 0x00000001), // MODEM.CF
    (0xB8014064, 0x000807B0), // MODEM.PRE
    (0xB8014068, 0x000000A7), // MODEM.SYNC0
    (0xB8014080, 0x08A0014B), // MODEM.TIMING
    (0xB8014084, 0x744AC39B), // MODEM.DSSS0
    (0xB8014088, 0x000003F0), // MODEM.MODINDEX
    (0xB8014094, 0x30100101), // MODEM.SHAPING0
    (0xB8014098, 0x7F7F7050), // MODEM.SHAPING1
    (0xB80140C4, 0x00000444), // MODEM.RAMPCTRL
    (0xB80140CC, 0x002F2F2F), // MODEM.RAMPLEV
    (0xB80140E0, 0x00000033), // MODEM.DCCOMP
    (0xB80140EC, 0x8BD9F87E), // MODEM.SRCCHF
    (0xB80140FC, 0x000A209C), // MODEM.DSACTRL
    (0xB801410C, 0x0012500E), // MODEM.DIGMIXCTRL
    (0xB8014114, 0x4040BB88), // MODEM.VTTRACK
    (0xB8014128, 0x00001003), // MODEM.CGCLKSTOP
    (0xB8014228, 0x00002001), // MODEM.IRCAL
    (0xB8014288, 0x82458251), // MODEM.IRCALCOEFWR0 (golden; DEVINFO overrides)
    (0xB801428C, 0x82458251), // MODEM.IRCALCOEFWR1 (golden; DEVINFO overrides)
    (0xB8018004, 0x00000001), // SYNTH.EN
    (0xB801802C, 0x00000026), // SYNTH.VCDACCTRL
    (0xB8018038, 0x00104911), // SYNTH.IFFREQ
    (0xB8018044, 0x0000000F), // SYNTH.CHSP
    (0xB8018054, 0x000000C8), // SYNTH.VCORANGE
    (0xB8018058, 0x000000B5), // SYNTH.VCOGAIN
    (0xB801809C, 0x000240EB), // SYNTH.LPFCTRL1CAL
    (0xB80180A0, 0x00037870), // SYNTH.LPFCTRL1RX
    (0xB80180A4, 0x0000C0D5), // SYNTH.LPFCTRL1TX
    (0xB80180A8, 0x01CB4325), // SYNTH.LPFCTRL2RX
    (0xB80180AC, 0x008D2325), // SYNTH.LPFCTRL2TX
    (0xB80180B0, 0x02000300), // SYNTH.DSMCTRLRX
    (0xB80180B4, 0x01000037), // SYNTH.DSMCTRLTX
    (0xB8020004, 0x00000001), // RAC.EN
    (0xB8020014, 0x08000380), // RAC.CTRL (SEQCLKDIS, CPUWAITDIS=0)
    (0xB8020038, 0x00000011), // RAC.SEQCTRL
    (0xB8020048, 0x00000008), // RAC.SR2
    (0xB802004C, 0x40F0CA31), // RAC.SR3
    (0xB8020050, 0x01061A7F), // RAC.STCTRL
    (0xB8020098, 0x02000C00), // RAC.SYNTHREGCTRL
    (0xB80200A0, 0x00004000), // RAC.CHPCTRL
    (0xB80200A4, 0x00000400), // RAC.SYNTHCTRL
    (0xB80200B0, 0x0E200000), // RAC.IFPGACTRL
    (0xB80200B4, 0x10000000), // RAC.PAENCTRL
    (0xB80200BC, 0x16D55524), // RAC.AUXADCTRIM
    (0xB80200E0, 0x0000001E), // RAC.IFADCDBG
    (0xB80200EC, 0x00001200), // RAC.IFADCCAL
    (0xB80200F8, 0x1108213D), // RAC.LNAMIXTRIM0
    (0xB80200FC, 0x00045409), // RAC.LNAMIXTRIM1
    (0xB8020110, 0x000002A4), // RAC.PATRIM0
    (0xB8020114, 0x0000A997), // RAC.PATRIM1
    (0xB8020118, 0x0000102C), // RAC.PATRIM2
    (0xB802011C, 0x00302D2A), // RAC.PATRIM3
    (0xB8020128, 0x00000549), // RAC.PGATRIM
    (0xB802012C, 0x211C1F24), // RAC.PGACAL
    (0xB8020130, 0x02510060), // RAC.PGACTRL
    (0xB8020134, 0x30171A1E), // RAC.RFBIASCAL
    (0xB8020138, 0x00040004), // RAC.RFBIASCTRL
    (0xB8020140, 0x00000000), // RAC.RFPATHEN
    (0xB8020144, 0x00000000), // RAC.RX
    (0xB8020150, 0x00064E29), // RAC.SYTRIM0
    (0xB802015C, 0x00000402), // RAC.SYEN
    (0xB8020168, 0x00000800), // RAC.SYMMDCTRL
    (0xB8020174, 0x00000770), // RAC.XORETIMECTRL
];

/// Number of RAC/analog registers switched per radio state.
pub(super) const STATE_REGS: usize = 16;

/// Analog/RAC registers that the RAIL sequencer rewrites on every state
/// change (diff of the idle, RX-search and TX snapshots).
const RAC_STATE_REGS: [u32; STATE_REGS] = [
    0xB802_0094, // RAC.SYNTHENCTRL
    0xB802_00B4, // RAC.PAENCTRL
    0xB802_00E0, // RAC.IFADCDBG
    0xB802_0108, // RAC.LNAMIXEN
    0xB802_011C, // RAC.PATRIM3
    0xB802_012C, // RAC.PGACAL
    0xB802_0130, // RAC.PGACTRL
    0xB802_013C, // RAC.RADIOEN
    0xB802_0140, // RAC.RFPATHEN
    0xB802_0144, // RAC.RX
    0xB802_0148, // RAC.TX
    0xB802_0150, // RAC.SYTRIM0
    0xB802_015C, // RAC.SYEN
    0xB802_0160, // RAC.SYLOEN
    0xB802_0168, // RAC.SYMMDCTRL
    0xB802_0188, // RAC.AGCOVERWRITE
];
const SYNTH_VCDACCTRL: u32 = 0xB801_802C;
const SYNTH_VCORANGE: u32 = 0xB801_8054;

#[rustfmt::skip]
pub(super) const IDLE: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0000_0000, 0x1000_0000, 0x0000_001E, 0x0000_0000,
        0x0030_2D2A, 0x211C_1F24, 0x0251_0060, 0x0000_0000,
        0x0000_0000, 0x0000_0000, 0x0000_0000, 0x0006_4E29,
        0x0000_0402, 0x0000_0000, 0x0000_0800, 0x0000_0000,
    ],
    extra: &[(SYNTH_VCDACCTRL, 0x26), (SYNTH_VCORANGE, 0xC8)],
};

#[rustfmt::skip]
pub(super) const RX: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0000_0200, 0x1000_0000, 0x0000_001F, 0x0000_0001,
        0x0030_2D2A, 0x1F1D_2123, 0x0251_1E6C, 0x0000_0007,
        0x0000_0006, 0x0000_0656, 0x0000_0000, 0x0006_4E29,
        0x0000_1FBF, 0x0000_0003, 0x0000_0800, 0x0000_0000,
    ],
    extra: &[(SYNTH_VCDACCTRL, 0xA6), (SYNTH_VCORANGE, 0xC8)],
};

#[rustfmt::skip]
pub(super) const TX: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0010_0200, 0x1001_0100, 0x0000_001E, 0x0000_0001,
        0x0030_652A, 0x1F1D_2123, 0x0251_0060, 0x0000_0007,
        0x0000_0014, 0x0000_00B0, 0xC3C8_0000, 0x0006_6E28,
        0x0000_1FBD, 0x0000_0400, 0x0000_0400, 0x0000_07F3,
    ],
    extra: &[(SYNTH_VCDACCTRL, 0xA6), (SYNTH_VCORANGE, 0x158)],
};

/// `(deci-dBm, RAMPLEV, RAMPCTRL)` as programmed by GSDK 4.5.0
/// `RAIL_SetTxPowerDbm` on xG22 with the BRD4183A PA curves
/// (`2P4GIG_HIGHEST`, 3.3 V, 2 us ramp). Ascending power.
pub(super) const TX_POWER: &[(i16, u8, u16)] = &[
    (-250, 0x20, 0x555),
    (-200, 0x21, 0x555),
    (-150, 0x22, 0x555),
    (-120, 0x23, 0x555),
    (-100, 0x24, 0x555),
    (-80, 0x25, 0x555),
    (-70, 0x26, 0x555),
    (-60, 0x27, 0x444),
    (-40, 0x29, 0x444),
    (-30, 0x2A, 0x444),
    (-20, 0x2B, 0x444),
    (-10, 0x2D, 0x444),
    (0, 0x2F, 0x444),
    (10, 0x31, 0x444),
    (20, 0x35, 0x444),
    (30, 0x38, 0x444),
    (40, 0x3B, 0x444),
    (50, 0x45, 0x444),
    (60, 0x4F, 0x333),
];

/// Program the ramp of a [`TX_POWER`] entry (all three ramp levels).
pub(super) fn write_tx_power(ramplev: u8, rampctrl: u16) {
    let lev = u32::from(ramplev);
    unsafe {
        write(L.modem + MODEM_RAMPLEV, lev | (lev << 8) | (lev << 16));
        let ctrl = read(L.modem + MODEM_RAMPCTRL);
        write(
            L.modem + MODEM_RAMPCTRL,
            (ctrl & !0xFFF) | u32::from(rampctrl),
        );
    }
}

/// The PA setting lives entirely in the ramp registers on xG22.
#[inline(always)]
pub(super) fn tx_state_power(_ddbm: i16) {}

/// The xG22 0/6 dBm PA ramps without CPU help; `TXPD` is not held.
pub(super) const PA_RAMP_IN_TXPD: bool = false;

/// See [`PA_RAMP_IN_TXPD`].
#[inline(always)]
pub(super) fn pa_ramp_up() {}

/// See [`pa_ramp_up`].
#[inline(always)]
pub(super) fn pa_ramp_down() {}

// ---- Radio state hand-shake -------------------------------------------
//
// `RAC.CTRL.SEQCLKDIS` gates the sequencer; the RSM instead stops in each
// state it enters until the matching `RAC.SEQIF.STATE*` flag is cleared.

const RAC_SEQIF: u32 = 0x28;
const RAC_SEQIF_ALL: u32 = 0x3FFF_000F;
const RAC_SEQIF_RXWARM: u32 = 1 << 17;
const RAC_SEQIF_TXWARM: u32 = 1 << 24;
/// Warm states are held until the driver has calibrated and locked the
/// synthesizer (the RSM only enables the LO path in RXWARM/TXWARM).
const RAC_SEQIF_HELD: u32 = RAC_SEQIF_RXWARM | RAC_SEQIF_TXWARM;

const fn seqif(warm: Warm) -> u32 {
    match warm {
        Warm::Rx => RAC_SEQIF_RXWARM,
        Warm::Tx => RAC_SEQIF_TXWARM,
        Warm::TxPd => 0,
    }
}

/// Drop any stale state hand-shake.
pub(super) fn seq_init() {
    unsafe { write(L.rac + RAC_SEQIF + CLR, RAC_SEQIF_ALL) };
}

/// Release every state except the held warm states.
pub(super) fn seq_service() {
    unsafe {
        // Clear the host flag first so a state entered after the SEQIF read
        // re-raises the interrupt instead of being lost.
        write(L.rac + RAC_IF + CLR, RAC_IF_STATECHANGE);
        let pending = read(L.rac + RAC_SEQIF) & RAC_SEQIF_ALL & !RAC_SEQIF_HELD;
        if pending != 0 {
            write(L.rac + RAC_SEQIF + CLR, pending);
        }
    }
}

/// Warm states are always held on xG22.
#[inline(always)]
pub(super) fn seq_arm(_warm: Warm) {}

/// Release a held warm state.
pub(super) fn seq_release(warm: Warm) {
    unsafe { write(L.rac + RAC_SEQIF + CLR, seqif(warm)) };
}
