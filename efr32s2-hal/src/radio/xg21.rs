//! xG21 (EFR32MG21, die revision A1) IEEE 802.15.4 2.4 GHz O-QPSK tables.
//!
//! Provenance: register snapshots taken over SWD from an EFR32MG21A020F1024IM32
//! (Home Assistant Connect ZBT-1) running GSDK 4.5.0 RAIL with
//! `RAIL_IEEE802154_Config2p4GHzRadio()` (lab-only instrument linked above the
//! resident Gecko bootloader, not shipped), reduced to the read-write
//! registers whose value differs from the GSDK radio configurator register
//! map (`efr32xg21` C0) reset value. The values are register *settings*, not
//! code: no RAIL object code or RAIL sequencer microcode is used.
//!
//! Differences from xG22 that this module handles:
//! - The xG21 RSM has no `SEQIF` hand-shake and no `RAC.CTRL.SEQCLKDIS`: it
//!   waits for the radio sequencer to finish the vector of every state it
//!   enters. The driver therefore loads its own 64-byte vector table into
//!   sequencer RAM (compact mode, one 4-byte slot per RSM state). Every slot
//!   is `END` except the two warm states, which are parked on a self-`JUMP`
//!   while the CPU locks the synthesizer and released by writing `END` over
//!   that jump. The two encodings were determined on the hardware; nothing
//!   of RAIL's sequencer program is reused.
//! - `RAC.PACTRL` carries the PA power setting and is only non-zero in TX;
//!   `MODEM.RAMPLEV` holds the ramp levels (see [`TX_POWER`]).
//! - `BUFC.BUFn.ADDR` is a 24-bit offset from `BUFC.RAMBASEADDR`, whose reset
//!   value is `0x2000_0000`, so the driver's SRAM addresses work unchanged.
//! - No `CMU.CLKENn` gates, but `CMU.RADIOCLKCTRL.EN` must be set before any
//!   radio register access (see `clock::enable_radio_clocks`).
//!
//! Deviations from the snapshot:
//! - Sequencer scratch/registers (`RAC.SRn`, `RAC.Rn`, `SCRATCHn`,
//!   `VECTADDR`, `SEQCTRL`) are not taken from RAIL; see [`seq_init`].
//! - `FRC.TRAILRXDATA` is overridden after the table (RSSI trailer byte only).
//! - `MODEM.IRCALCOEFWR0/1` are replaced by the per-die DEVINFO word at
//!   [`DEVINFO_IRCAL`] when it is programmed.

use super::{
    AnalogState, CLR, Deadline, L, Layout, RAC_IF, RAC_IF_STATECHANGE, Warm, read, tx_power_entry,
    write,
};

pub(super) const SUPPORTED: bool = true;

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
};

/// DEVINFO word holding the 2.4 GHz IR-calibration coefficients
/// (`0xFC8E_804E` on the captured die, identical to RAIL's programmed value).
pub(super) const DEVINFO_IRCAL: u32 = 0x0FE0_824C;

/// RSSI adjustment (dB) applied to `AGC.RSSI`/`FRAMERSSI` and the RX trailer
/// byte. `RAIL_GetRssi` on the ZBT-1 reads 198 quarter-dB below
/// `AGC.RSSI[15:6]` (the -50 dB chip term, as on xG22), and GSDK's xG21
/// application offset `SL_RAIL_UTIL_RSSI_OFFSET` is -11 dB
/// (`rail_util_rssi/config/efr32xg21`), which the stock ZBT-1 NCP applies.
pub(super) const RSSI_ADJUST_DBM: i8 = -61;
pub(super) const MODEM_IRCALCOEFWR0: u32 = 0xB801_4250;
pub(super) const MODEM_IRCALCOEFWR1: u32 = 0xB801_4254;

/// `SYNTH.LOCNTCTRL` has no `FCALRUNCLKEN` bit on xG21.
pub(super) const LOCNT_FCALRUNCLKEN: u32 = 0;
/// MODEM.RAMPLEV (xG21 `_MODEM_RAMPLEV`, offset 0xC4; RAMPCTRL 0xC0 is static).
const MODEM_RAMPLEV: u32 = 0xC4;

/// Static PHY + MAC-framing configuration, in ascending address order.
pub(super) const INIT: &[(u32, u32)] = &[
    (0xB8004004, 0x00000001), // FRC.EN
    (0xB800400C, 0x00148001), // FRC.DFLCTRL
    (0xB8004010, 0x0000407F), // FRC.MAXLENGTH
    (0xB8004020, 0x0000005C), // FRC.WCNTCMP0
    (0xB8004028, 0x00000004), // FRC.WCNTCMP2
    (0xB8004048, 0x000007A0), // FRC.CTRL
    (0xB800404C, 0x00000060), // FRC.RXCTRL
    (0xB8004054, 0x0000001B), // FRC.TRAILRXDATA
    (0xB8004060, 0x00000000), // FRC.PUNCTCTRL
    (0xB8004084, 0x000000F8), // FRC.SNIFFCTRL
    (0xB80040A8, 0x00000007), // FRC.AUTOCG
    (0xB80040B4, 0x00004000), // FRC.FCD0
    (0xB80040B8, 0x00004CFF), // FRC.FCD1
    (0xB80040BC, 0x00004100), // FRC.FCD2
    (0xB80040C0, 0x00004DFF), // FRC.FCD3
    (0xB800C004, 0x00000001), // AGC.EN
    (0xB800C01C, 0x0008E214), // AGC.CTRL0
    (0xB800C020, 0x00004300), // AGC.CTRL1
    (0xB800C024, 0x03B380EC), // AGC.CTRL2
    (0xB800C028, 0x02A87543), // AGC.CTRL3
    (0xB800C030, 0x0000022E), // AGC.MANGAIN
    (0xB800C050, 0x04304187), // AGC.GAINRANGE
    (0xB800C054, 0xE6092D0E), // AGC.AGCPERIOD
    (0xB800C058, 0x08070654), // AGC.HICNTREGION
    (0xB800C05C, 0x0002B6D1), // AGC.STEPDWN
    (0xB800C064, 0x00003004), // AGC.GAINSTEPLIM
    (0xB8010004, 0x00000001), // RFCRC.EN
    (0xB8010020, 0x00008408), // RFCRC.POLY
    (0xB8014004, 0x00000001), // MODEM.EN
    (0xB801401C, 0x00000010), // MODEM.MIXCTRL
    (0xB8014020, 0x0413F920), // MODEM.CTRL0
    (0xB8014024, 0x00620007), // MODEM.CTRL1
    (0xB8014028, 0x00007038), // MODEM.CTRL2
    (0xB801402C, 0x00000000), // MODEM.CTRL3
    (0xB8014034, 0x0F016800), // MODEM.CTRL5
    (0xB8014038, 0x4024B840), // MODEM.CTRL6
    (0xB8014058, 0x006900FC), // MODEM.TXBR
    (0xB801405C, 0x00000841), // MODEM.RXBR
    (0xB8014060, 0x00000001), // MODEM.CF
    (0xB8014064, 0x000807B0), // MODEM.PRE
    (0xB8014068, 0x000000A7), // MODEM.SYNC0
    (0xB8014080, 0x11E06323), // MODEM.TIMING
    (0xB8014084, 0x744AC39B), // MODEM.DSSS0
    (0xB8014088, 0x000003F0), // MODEM.MODINDEX
    (0xB801408C, 0x00008003), // MODEM.AFC
    (0xB8014090, 0x00000ABE), // MODEM.AFCADJLIM
    (0xB8014094, 0x30100101), // MODEM.SHAPING0
    (0xB8014098, 0x7F7F7050), // MODEM.SHAPING1
    (0xB80140C0, 0x00000679), // MODEM.RAMPCTRL
    (0xB80140C4, 0x00CD4010), // MODEM.RAMPLEV
    (0xB80140E0, 0x00000033), // MODEM.DCCOMP
    (0xB80140EC, 0x8BD9F87E), // MODEM.SRCCHF
    (0xB80140F0, 0x00000B59), // MODEM.INTAFC
    (0xB801410C, 0x0012500E), // MODEM.DIGMIXCTRL
    (0xB8014114, 0x0040BB88), // MODEM.VTTRACK
    (0xB8014128, 0x00001E00), // MODEM.CGCLKSTOP
    (0xB801413C, 0x008C1400), // MODEM.LONGRANGE1
    (0xB8014140, 0x28211A14), // MODEM.LONGRANGE2
    (0xB8014144, 0x423B342E), // MODEM.LONGRANGE3
    (0xB8014148, 0x55435049), // MODEM.LONGRANGE4
    (0xB801414C, 0x0CBA9876), // MODEM.LONGRANGE5
    (0xB8014150, 0x00006928), // MODEM.LONGRANGE6
    (0xB8014158, 0x64282107), // MODEM.COH0
    (0xB801415C, 0x001A1714), // MODEM.COH1
    (0xB8014160, 0x69C80420), // MODEM.COH2
    (0xB8014164, 0x093A20A1), // MODEM.COH3
    (0xB8014228, 0x00002001), // MODEM.IRCAL
    (0xB801423C, 0x00000029), // MODEM.DIGIGAINCTRL
    (0xB8014250, 0xFC8E804E), // MODEM.IRCALCOEFWR0
    (0xB8014254, 0xFC8E804E), // MODEM.IRCALCOEFWR1
    (0xB8018004, 0x00000001), // SYNTH.EN
    (0xB8018038, 0x00104911), // SYNTH.IFFREQ
    (0xB8018044, 0x0000000F), // SYNTH.CHSP
    (0xB8018058, 0x0000005C), // SYNTH.VCOGAIN
    (0xB801809C, 0x000240EB), // SYNTH.LPFCTRL1CAL
    (0xB80180A0, 0x00037870), // SYNTH.LPFCTRL1RX
    (0xB80180A4, 0x0000C0D5), // SYNTH.LPFCTRL1TX
    (0xB80180A8, 0x01CB4315), // SYNTH.LPFCTRL2RX
    (0xB80180AC, 0x008D2315), // SYNTH.LPFCTRL2TX
    (0xB80180B0, 0x02000300), // SYNTH.DSMCTRLRX
    (0xB80180B4, 0x01000037), // SYNTH.DSMCTRLTX
    (0xB8020004, 0x00000001), // RAC.EN
    (0xB8020014, 0x00000380), // RAC.CTRL
    (0xB8020088, 0x04000C00), // RAC.SYNTHREGCTRL
    (0xB8020094, 0x00000400), // RAC.SYNTHCTRL
    (0xB80200A0, 0x0E200000), // RAC.IFPGACTRL
    (0xB80200AC, 0x16D55524), // RAC.AUXADCTRIM
    (0xB80200D0, 0x0000000F), // RAC.IFADCDBG
    (0xB80200D4, 0x08960290), // RAC.IFADCTRIM
    (0xB80200D8, 0x00001200), // RAC.IFADCCAL
    (0xB80200E8, 0x00011507), // RAC.LNAMIXTRIM1
    (0xB80200F4, 0x00000001), // RAC.LNAMIXEN
    (0xB80200FC, 0x1808899E), // RAC.PATRIM0
    (0xB8020110, 0x00000549), // RAC.PGATRIM
    (0xB8020114, 0x1B1B2525), // RAC.PGACAL
    (0xB802011C, 0x30093011), // RAC.RFBIASCAL
    (0xB8020120, 0x00040004), // RAC.RFBIASCTRL
    (0xB802015C, 0x00000760), // RAC.DIGCLKRETIMECTRL
    (0xB8020164, 0x00000760), // RAC.XORETIMECTRL
    (0xB8020174, 0x00000100), // RAC.ANTDIV
];

/// Number of RAC/analog registers switched per radio state.
pub(super) const STATE_REGS: usize = 17;

/// Analog/RAC registers that RAIL rewrites on every state change (diff of
/// the idle, RX-search and TX snapshots).
const RAC_STATE_REGS: [u32; STATE_REGS] = [
    0xB802_0084, // RAC.SYNTHENCTRL
    0xB802_008C, // RAC.VCOCTRL
    0xB802_0090, // RAC.CHPCTRL
    0xB802_00A4, // RAC.PAENCTRL
    0xB802_0100, // RAC.PATRIM1
    0xB802_0104, // RAC.PATRIM2
    0xB802_0108, // RAC.PACTRL (power field added by `tx_state_power`)
    0xB802_0118, // RAC.PGACTRL
    0xB802_0124, // RAC.RADIOEN
    0xB802_0128, // RAC.RFPATHEN1
    0xB802_012C, // RAC.RFPATHEN2
    0xB802_0130, // RAC.RX
    0xB802_0134, // RAC.TX
    0xB802_013C, // RAC.SYTRIM0
    0xB802_0140, // RAC.SYTRIM1
    0xB802_0148, // RAC.SYEN
    0xB802_014C, // RAC.SYLOEN
];
const SYNTH_VCDACCTRL: u32 = 0xB801_802C;
const SYNTH_VCORANGE: u32 = 0xB801_8054;
const AGC_MANGAIN: u32 = 0xB800_C030;
const AGC_PNRFATT0: u32 = 0xB800_C068;
const AGC_LNAMIXCODE0: u32 = 0xB800_C074;
/// AGC receive gain control (RAIL idle/RX values).
const AGC_RX: [(u32, u32); 3] = [
    (AGC_MANGAIN, 0x0000_022E),
    (AGC_PNRFATT0, 0x0610_2040),
    (AGC_LNAMIXCODE0, 0x1572_4BBD),
];
const RAC_PACTRL: u32 = 0xB802_0108;
/// `RAC.PACTRL` TX value without the power field `[23:16]`.
const PACTRL_TX: u32 = 0x0000_0505;

#[rustfmt::skip]
pub(super) const IDLE: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0000_0000, 0x0000_004C, 0x0000_0000, 0x0000_0000,
        0x4000_0980, 0x6987_F98A, 0x0000_0000, 0x0251_0060,
        0x0000_0000, 0x0000_0000, 0x0000_0000, 0x0000_0000,
        0x0000_0000, 0x0030_0029, 0x0001_9FC4, 0x0000_0000,
        0x0000_0009,
    ],
    extra: &[
        (SYNTH_VCDACCTRL, 0x25),
        (SYNTH_VCORANGE, 0xC8),
        AGC_RX[0],
        AGC_RX[1],
        AGC_RX[2],
    ],
};

#[rustfmt::skip]
pub(super) const RX: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0000_0000, 0x0000_004C, 0x0000_0000, 0x0000_0000,
        0x4000_0980, 0x6987_F98A, 0x0000_0000, 0x0251_5E6C,
        0x0000_0007, 0x0000_0004, 0x0000_002E, 0x0000_385E,
        0x0000_0000, 0x0030_0029, 0x0001_9FC4, 0x0000_1FBF,
        0x0000_0009,
    ],
    extra: &[
        (SYNTH_VCDACCTRL, 0xA5),
        (SYNTH_VCORANGE, 0xC8),
        AGC_RX[0],
        AGC_RX[1],
        AGC_RX[2],
    ],
};

/// `RAC.TX` while warming up: PA regulators, bias and slices enabled, PA
/// output (`PAENPAOUT`) still off. `PARAMP` (`RAC.PAENCTRL`) is also left
/// clear; [`pa_ramp_up`] starts the ramp once the synthesizer is locked.
const TX_PA_BIAS: u32 = 0xCC30_0000;

#[rustfmt::skip]
pub(super) const TX: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0010_0000, 0x0000_00AC, 0x0000_4000, 0x0000_0000,
        0x400F_FED0, 0x6987_F99A, PACTRL_TX,   0x0251_0060,
        0x0000_0007, 0x0000_0014, 0x0000_0036, 0x0000_04B0,
        TX_PA_BIAS,  0x0030_2E29, 0x0001_9FD3, 0x0000_1FBD,
        0x0000_0400,
    ],
    // RAIL holds the receiver at manual minimum gain / maximum attenuation
    // while transmitting (the LNA shares the antenna path with the PA).
    extra: &[
        (SYNTH_VCDACCTRL, 0xA5),
        (SYNTH_VCORANGE, 0xCB),
        (AGC_MANGAIN, 0x0000_0001),
        (AGC_PNRFATT0, 0x0000_003F),
        (AGC_LNAMIXCODE0, 0x0000_0000),
    ],
};

/// `(deci-dBm, MODEM.RAMPLEV[23:16], RAC.PACTRL[23:16])` as programmed by
/// GSDK 4.5.0 `RAIL_SetTxPowerDbm` on the ZBT-1 (xG21 20 dBm PA curves,
/// 3.3 V). `MODEM.RAMPCTRL` (`0x679`) and the lower ramp levels
/// (`RAMPLEV[15:0] = 0x4010`) are constant. Ascending power.
pub(super) const TX_POWER: &[(i16, u8, u16)] = &[
    (-300, 0xCD, 0x12),
    (-250, 0xCD, 0x12),
    (-200, 0xCD, 0x12),
    (-150, 0xCD, 0x13),
    (-120, 0xCD, 0x13),
    (-100, 0xCD, 0x14),
    (-80, 0xCD, 0x14),
    (-60, 0xCD, 0x16),
    (-40, 0xCD, 0x17),
    (-30, 0xCD, 0x18),
    (-20, 0xCD, 0x19),
    (-10, 0xCD, 0x1A),
    (0, 0xCD, 0x1B),
    (10, 0xCD, 0x1D),
    (20, 0xCD, 0x1E),
    (30, 0xD9, 0x21),
    (40, 0xD9, 0x23),
    (50, 0xD9, 0x25),
    (60, 0xD9, 0x28),
    (70, 0xD9, 0x2B),
    (80, 0xD9, 0x2C),
    (90, 0xDC, 0x33),
    (100, 0xDC, 0x37),
    (110, 0xDC, 0x3B),
    (120, 0xDC, 0x3F),
    (130, 0xDE, 0x49),
    (140, 0xDE, 0x51),
    (150, 0xDE, 0x58),
    (160, 0xDF, 0x61),
    (170, 0xDE, 0x74),
    (180, 0xDF, 0x8B),
    (190, 0xDF, 0xA3),
    (200, 0xDF, 0xBA),
];

/// `MODEM.RAMPLEV` without the top ramp level.
const RAMPLEV_LOW: u32 = 0x4010;

/// Program the ramp level of a [`TX_POWER`] entry.
pub(super) fn write_tx_power(ramplev: u8, _pa: u16) {
    unsafe {
        write(
            L.modem + MODEM_RAMPLEV,
            RAMPLEV_LOW | (u32::from(ramplev) << 16),
        );
    }
}

/// Apply the PA setting for the selected TX power (deci-dBm) after [`TX`]
/// was applied.
pub(super) fn tx_state_power(ddbm: i16) {
    let (_, _, pa) = tx_power_entry(ddbm);
    unsafe { write(RAC_PACTRL, PACTRL_TX | (u32::from(pa & 0xFF) << 16)) };
}

// ---- PA power ramp ------------------------------------------------------
//
// The xG21 PA output level follows the MODEM ramp (`MODEM.RAMPCTRL`/
// `RAMPLEV`) only while `RAC.PACTRL.PAENPOWERRAMPINGCLK` clocks the PA
// power DAC. Without that clock the PA keeps its reset level and the
// transmitter is silent although FRC/MODEM complete the frame normally,
// and `RAC.RFSTATUS.MODRAMPUPDONE` never drops after `PARAMP` is cleared.
// Sequence traced from GSDK 4.5.0 RAIL on the ZBT-1 (register samples
// during `RAIL_StartTx`):
//   TXWARM: PAENCTRL.PARAMP=1, APC.ENAPCSW=1, then TX.PAENPAOUT=1 and
//           PACTRL.PAENPOWERRAMPINGCLK=1;
//   TX:     PAENPOWERRAMPINGCLK=0, ENAPCSW=0;
//   TXPD:   PAENPOWERRAMPINGCLK=1, PARAMP=0, then PAENPAOUT=0 and
//           PAENPOWERRAMPINGCLK=0 before the RSM leaves TXPD.

const RAC_RFSTATUS: u32 = 0xB802_0098;
const RFSTATUS_MODRAMPUPDONE: u32 = 1 << 0;
const RAC_PAENCTRL: u32 = 0xB802_00A4;
const PAENCTRL_PARAMP: u32 = 1 << 8;
const RAC_APC: u32 = 0xB802_00A8;
const APC_ENAPCSW: u32 = 1 << 2;
const RAC_TX: u32 = 0xB802_0134;
const TX_PAENPAOUT: u32 = 1 << 28;
const PACTRL_PAENPOWERRAMPINGCLK: u32 = 1 << 4;
/// A full ramp takes about 20 us (`MODEM.RAMPCTRL` = `0x679`).
const PA_RAMP_US: u32 = 50;

/// Read-modify-write: the sequencer that RAIL would run is idle, so the
/// CPU is the only writer of these registers.
fn set_bits(address: u32, bits: u32, on: bool) {
    unsafe {
        let value = read(address);
        write(address, if on { value | bits } else { value & !bits });
    }
}

fn wait_ramp(done: bool) {
    let deadline = Deadline::after_us(PA_RAMP_US);
    while unsafe { read(RAC_RFSTATUS) } & RFSTATUS_MODRAMPUPDONE != u32::from(done) {
        if deadline.expired() {
            return;
        }
    }
}

/// The driver holds `TXPD` and calls [`pa_ramp_down`] there.
pub(super) const PA_RAMP_IN_TXPD: bool = true;

/// Ramp the PA up at the end of `TXWARM` (synthesizer locked, state held).
pub(super) fn pa_ramp_up() {
    unsafe { write(RAC_PAENCTRL, PAENCTRL_PARAMP) };
    set_bits(RAC_APC, APC_ENAPCSW, true);
    set_bits(RAC_TX, TX_PAENPAOUT, true);
    set_bits(RAC_PACTRL, PACTRL_PAENPOWERRAMPINGCLK, true);
    wait_ramp(true);
    set_bits(RAC_PACTRL, PACTRL_PAENPOWERRAMPINGCLK, false);
    set_bits(RAC_APC, APC_ENAPCSW, false);
}

/// Ramp the PA down in the held `TXPD` state.
pub(super) fn pa_ramp_down() {
    set_bits(RAC_PACTRL, PACTRL_PAENPOWERRAMPINGCLK, true);
    unsafe { write(RAC_PAENCTRL, 0) };
    wait_ramp(false);
    set_bits(RAC_TX, TX_PAENPAOUT, false);
    set_bits(RAC_PACTRL, PACTRL_PAENPOWERRAMPINGCLK, false);
}

// ---- Radio sequencer --------------------------------------------------

/// Sequencer RAM, also the vector table base (`RAC.VECTADDR`).
const SEQRAM: u32 = 0xB000_0000;
const RAC_VECTADDR: u32 = 0x64;
const RAC_SEQCTRL: u32 = 0x68;
/// `SEQCTRL.COMPACT`: one 4-byte vector slot per RSM state.
const SEQCTRL_COMPACT: u32 = 1;
/// Number of vector slots (RSM states 0..=15).
const SEQ_SLOTS: u32 = 16;
/// Two `END` half-words: the state's sequencer work is done.
const SEQ_END: u32 = 0xF800_F800;
/// `JUMP` to the absolute sequencer byte offset in the upper half-word.
const SEQ_JUMP: u32 = 0x9802;
/// Vector slots of the warm states the CPU holds (RSM state numbers).
const SLOT_RXWARM: u32 = 1;
const SLOT_TXWARM: u32 = 8;
const SLOT_TXPD: u32 = 10;

const fn warm_slot(warm: Warm) -> u32 {
    SEQRAM
        + 4 * match warm {
            Warm::Rx => SLOT_RXWARM,
            Warm::Tx => SLOT_TXWARM,
            Warm::TxPd => SLOT_TXPD,
        }
}

/// Load the vector table (every state ends immediately) and point the
/// sequencer at it.
pub(super) fn seq_init() {
    unsafe {
        for slot in 0..SEQ_SLOTS {
            write(SEQRAM + 4 * slot, SEQ_END);
        }
        write(L.rac + RAC_VECTADDR, SEQRAM);
        write(L.rac + RAC_SEQCTRL, SEQCTRL_COMPACT);
    }
}

/// Acknowledge a state change. The sequencer itself completes every state
/// except an armed warm state.
pub(super) fn seq_service() {
    unsafe { write(L.rac + RAC_IF + CLR, RAC_IF_STATECHANGE) };
}

/// Park the next entry into `warm` until [`seq_release`]. Only call while
/// the RSM is `OFF`.
pub(super) fn seq_arm(warm: Warm) {
    let slot = warm_slot(warm);
    unsafe { write(slot, ((slot - SEQRAM) << 16) | SEQ_JUMP) };
}

/// Let the sequencer finish `warm` (replaces the self-jump by `END`).
pub(super) fn seq_release(warm: Warm) {
    unsafe { write(warm_slot(warm), SEQ_END) };
}
