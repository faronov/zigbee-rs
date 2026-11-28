//! xG24 (EFR32MG24) IEEE 802.15.4 2.4 GHz O-QPSK tables.
//!
//! Provenance: SWD register snapshots of an EFR32MG24B220F1536IM48
//! (BRD4187A Rev A01 on a BRD4001A WSTK, DEVINFO PRODREV 0x19, 39 MHz HFXO,
//! CTUNE 95, DC-DC in bypass) running GSDK 4.5.0 RAILtest 2.16.7 with
//! `RAIL_IEEE802154_Config2p4GHzRadio()` (lab-only instrument, not shipped),
//! taken in idle, RX search and TX (CW tone and 127-byte frames) on every
//! channel and at every HP-PA level RAIL selects for -25..+20 dBm, plus an
//! on-target sampler of RAIL's PA transitions (every ~11 us). The raw
//! dumps, the capture method and a decoded summary are in
//! `tools/efr32s2-lab/captures/xg24-brd4187a`. The 10 dBm (MP) PA values
//! ([`PaPhy`], the MP [`TX_POWER`] table and PA words) come from the same
//! captures on an EFR32MG24B210F1536IM48 (xG24 Explorer Kit EK2703A,
//! BRD2703A A02, 39 MHz, CTUNE 100, DC-DC in bypass, RAIL ramp 10 us):
//! `tools/efr32s2-lab/captures/xg24-ek2703a`. The static table is reduced
//! to the read-write registers whose value differs from the GSDK radio
//! configurator register map (`efr32xg24` rev A1) reset value; 192 of the
//! 208 registers of GSDK's `sl_rail_ieee802154_config_39MHz.c` match the
//! silicon exactly, the other 16 are per-state, per-channel or RAIL runtime
//! calibrations (listed in the capture summary). The values are register
//! *settings*, not code: no RAIL object code or sequencer microcode is used.
//!
//! The radio state machine has the xG22 hand-shake (`RAC.CTRL.SEQCLKDIS`,
//! `RAC.SEQIF`, identical state bits), so the clock-gated sequencer
//! approach of `xg22.rs` carries over. Differences from xG22:
//! - `MODEM` registers moved (`RAMPLEV` 0xDC, `IRCALCOEFWR0/1` 0x278/0x27C);
//!   BUFC is at `0xBA00_0000` with the xG22 buffer layout.
//! - The IF ADC clock follows the LO, so RAIL rescales `MODEM.SRCCHF`
//!   (`SRCRATIO2`, proportional to the LO frequency) and `MODEM.DIGMIXCTRL`
//!   (`DIGMIXFREQ`, inversely proportional) on every channel
//!   ([`select_channel`]).
//! - The 20 dBm (HP) PA takes its power from `RAC.PACTRL` (`TXPASELSLICE`,
//!   `TXPAPOWER`) and its ramp level from `MODEM.RAMPLEV`, which RAIL reads
//!   per slice from DEVINFO ([`DEVINFO_RAMPLEV`]). The PA output follows the
//!   MODEM ramp only while `RAC.TX.TXPAENRAMPCLK` clocks it (CPU ramp in
//!   `TXWARM`/`TXPD`, as on xG21). The power-up order is RAIL's, sampled on
//!   the target ([`pa_ramp_up`]): slices held in `PACTRL.TXPASLICERST` with
//!   manual slice/power selection while the regulators come up. Without it
//!   the output is right but the PA draws more than 95 mA at every level.
//! - `RAC.RFLOCK0` is set by the boot (on this part `RACTX0DBM` and
//!   `RACTX10DBM`: only the 20 dBm PA is usable) and is never written here.
//!
//! Crystal: the BRD4187A capture is at 39 MHz (feature `hfxo-39mhz`). The
//! 38.4 MHz table (feature `hfxo-38m4hz`, SONOFF Dongle-M) differs only in
//! the 21 registers in which GSDK's `sl_rail_ieee802154_config_38M4Hz.c`
//! differs from the 39 MHz file ([`XTAL`]). That GSDK configuration is
//! byte-identical (304 of 304 configuration words, plus the `phyInfo`
//! words) to the one inside the stock Dongle-M EmberZNet 7.4.5 image. The
//! per-channel `SRCRATIO2`/`DIGMIXFREQ` are not stored in either: RAIL
//! computes them per channel, and [`channel_words`] reproduces that
//! computation (exact on all 16 channels of the 39 MHz silicon). The
//! 38.4 MHz path has **not run on our stack** (first HIL pending): it also
//! carries the 39 MHz B220 die's runtime trims ([`PaPhy`]). An xG24 build
//! that selects neither crystal feature keeps [`SUPPORTED`] `false`.
//!
//! PA variants: the HP PA (`efr32mg24-pa20`, BRD4187A, B220) and the MP PA
//! (`efr32mg24-pa10`, EK2703A, B210) were each captured on one die. They
//! differ in the RF path RAIL drives (0 and 1: one RF pin is bonded out),
//! the PA trims and power programming, the PA power-up words and the IR
//! calibration ([`PaPhy`]); the PHY, the per-state values of the shared
//! blocks and the PA power-up order are the same.
//!
//! Deviations from the snapshot:
//! - `RAC.CTRL` sets `SEQCLKDIS` and clears `CPUWAITDIS` (RAIL:
//!   `0x04000380`).
//! - Sequencer scratch (`RAC.SRn`, `SCRATCHn`), interrupt enables/flags,
//!   `FRC.SNIFFCTRL` (PTI, left at reset), `FRC.WCNTCMP2` (RAIL keeps the
//!   channel index there) and `SYNTH.CHSP`/`FREQ` (programmed by the driver)
//!   are not taken from RAIL.
//! - `FRC.TRAILRXDATA` is overridden after the table (RSSI trailer byte).
//! - `MODEM.IRCALCOEFWR0/1` are replaced by the per-die DEVINFO word at
//!   [`DEVINFO_IRCAL`] when it is programmed (RAIL writes that word).

use super::{
    AnalogState, CLR, Deadline, L, Layout, RAC_IF, RAC_IF_STATECHANGE, RacState, SYNTH_CHCTRL,
    Warm, rac_state_raw, read, write,
};
use crate::delay::{cycles, us_to_cycles};

/// The radio runs only when the board names its crystal:
///
/// - `hfxo-39mhz`: tables captured on the silicon and hardware-verified,
///   the 20 dBm (HP) PA on the BRD4187A and the 10 dBm (MP) PA on the
///   EK2703A;
/// - `hfxo-38m4hz` (Dongle-M): the GSDK/stock-EmberZNet 38.4 MHz PHY with
///   RAIL's per-channel words ([`channel_words`]) and the 39 MHz B220
///   die's runtime trims. **First HIL pending**: raw TX/RX, ACK, output and
///   CTUNE offset against the stock image on a real unit (platform guide,
///   hardware gates).
///
/// Any other xG24 build fails `RadioToken::init` with `Unsupported` before
/// touching the radio.
pub(super) const SUPPORTED: bool = cfg!(all(
    any(feature = "efr32mg24-pa20", feature = "efr32mg24-pa10"),
    any(feature = "hfxo-39mhz", feature = "hfxo-38m4hz")
));

/// `FRC_IRQn`, `TIMER0_IRQn` and `RAC_RSM_IRQn` (GSDK 4.5.0
/// `efr32mg24*.h`).
#[cfg_attr(not(feature = "isr-ack"), allow(dead_code))]
pub(super) const FRC_IRQN: u32 = 33;
#[cfg_attr(not(feature = "isr-ack"), allow(dead_code))]
pub(super) const TIMER0_IRQN: u32 = 4;
#[cfg_attr(not(feature = "rt"), allow(dead_code))]
pub(super) const RAC_RSM_IRQN: u32 = 36;

/// Radio block bases; BUFC base, first buffer and stride as read from the
/// silicon (RAIL's buffers 0..3 at `0xBA00_000C + n * 0x40`).
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

/// DEVINFO word holding the 2.4 GHz IR-calibration coefficients; RAIL
/// writes it to `MODEM.IRCALCOEFWR0/1` (`0x80FB_FFEF` on the captured die,
/// stored twice, at 0x248 and 0x24C).
pub(super) const DEVINFO_IRCAL: u32 = 0x0FE0_8248;

/// RSSI adjustment (dB) applied to `AGC.RSSI`/`FRAMERSSI` and the RX trailer
/// byte. RAIL subtracts 200 quarter-dB from `AGC.RSSI[15:6]`, as on xG22:
/// frames from the nRF sniffer at 0 dBm read `FRAMERSSI` -5.75 dBm raw and
/// RAIL reported -56 dBm; the idle `AGC.RSSI` of -47.5 raw read -97.5 dBm
/// through `RAIL_GetRssi`. GSDK's xG24 `SL_RAIL_UTIL_RSSI_OFFSET` is 0.
pub(super) const RSSI_ADJUST_DBM: i8 = -50;
pub(super) const MODEM_IRCALCOEFWR0: u32 = 0xB801_4278;
pub(super) const MODEM_IRCALCOEFWR1: u32 = 0xB801_427C;

/// `SYNTH.LOCNTCTRL.FCALRUNCLKEN` (bit 11, as on xG22).
pub(super) const LOCNT_FCALRUNCLKEN: u32 = 1 << 11;

// ---- Crystal-dependent PHY values ----------------------------------------

/// Registers of the 802.15.4 PHY that depend on the HFXO frequency (GSDK
/// 4.5.0 `rail_util_built_in_phys/efr32xg24`: the 38.4 and 39 MHz tables
/// differ in exactly these).
pub(super) struct XtalPhy {
    agc_gainsteplim0: u32,
    txbr: u32,
    cf: u32,
    antswctrl1: u32,
    /// `MODEM.CHFCOE0n` (= `CHFCOE1n`).
    chfcoe: [u32; 7],
    iffreq: u32,
    /// `MODEM.SRCCHF.SRCRATIO2` and `DIGMIXCTRL.DIGMIXFREQ` per channel
    /// 11..=26.
    srcratio: [u16; 16],
    digmixfreq: [u32; 16],
}

/// 39 MHz (BRD4187A, RF-BM-MG24B1): silicon values; the per-channel
/// columns are the RAIL values read on every channel.
#[cfg(feature = "hfxo-39mhz")]
pub(super) const XTAL: XtalPhy = XtalPhy {
    agc_gainsteplim0: 0x0000_06C0,
    txbr: 0x00F0_0249,
    cf: 0x0000_0002,
    antswctrl1: 0x0006_9069,
    chfcoe: [
        0x365E_63DE,
        0x0007_6FBF,
        0x0016_EA6F,
        0x00CE_30E5,
        0x0ED9_B9B2,
        0x0494_844B,
        0x24A9_1F5A,
    ],
    iffreq: 0x0010_47F1,
    srcratio: [
        0x3C29, 0x3C49, 0x3C69, 0x3C89, 0x3CA9, 0x3CC9, 0x3CE9, 0x3D09, 0x3D29, 0x3D49, 0x3D69,
        0x3D89, 0x3DA9, 0x3DC9, 0x3DE9, 0x3E09,
    ],
    digmixfreq: [
        0x254F8, 0x253BB, 0x2527F, 0x25145, 0x2500C, 0x24ED4, 0x24D9E, 0x24C69, 0x24B35, 0x24A02,
        0x248D1, 0x247A1, 0x24672, 0x24544, 0x24418, 0x242ED,
    ],
};

/// 38.4 MHz (SONOFF Dongle Max, Dongle-M): GSDK 4.5.0 `config_38M4Hz`
/// values, equal to the RAIL configuration of the stock Dongle-M EmberZNet
/// 7.4.5 image; per-channel columns from [`channel_words`] with that
/// configuration's `phyInfo` (`src2Denominator` 60 MHz) and `MODEM.CF`
/// (`DEC0` = 0: decimation 3). Not run on our stack yet.
#[cfg(not(feature = "hfxo-39mhz"))]
pub(super) const XTAL: XtalPhy = {
    let (srcratio, digmixfreq) = channel_words(38_400_000, 0x4911, 60_000_000, 3);
    XtalPhy {
        agc_gainsteplim0: 0x0000_0740,
        txbr: 0x00FF_0264,
        cf: 0x0000_0008,
        antswctrl1: 0x0006_AAAA,
        chfcoe: [
            0x0BFF_E7E6,
            0x000A_A1CD,
            0x006A_06BD,
            0x004D_B05E,
            0x0E42_027D,
            0x0222_B6A5,
            0x34B2_25FF,
        ],
        iffreq: 0x0010_4911,
        srcratio,
        digmixfreq,
    }
};

const X: XtalPhy = XTAL;

// ---- PA-variant and die values ----------------------------------------------

/// Registers that differ between the two captures besides the PA power
/// programming: the PA variant (and with it the RF path: RAIL drives path 0
/// on the 20 dBm B220 and path 1 on the 10 dBm B210, which bond out one RF
/// pin each), and runtime calibrations RAIL wrote on each die. One die per
/// variant was captured (BRD4187A, EK2703A), so the "die" values are that
/// die's.
pub(super) struct PaPhy {
    /// `MODEM.IRCALCOEFWR0/1`. HP: RAIL's value is the DEVINFO word
    /// ([`DEVINFO_IRCAL_APPLIES`]); MP: RAIL leaves all-ones (no IR
    /// correction) although DEVINFO holds a word.
    ircal: u32,
    vcogain: u32,
    lpfctrl2rx: u32,
    lpfctrl2tx: u32,
    vcoctrl: u32,
    auxadctrim: u32,
    ifadccal: u32,
    lnamixtrim0: u32,
    lnamixtrim3: u32,
    lnamixtrim4: u32,
    patrim2: u32,
    patrim3: u32,
    patrim5: u32,
    pgatrim: u32,
    pgacal: u32,
    sytrim1: u32,
    /// `RAC.RFPATHENn` of the bonded RF path (switched per state).
    rfpathen: u32,
    /// `RAC.PATRIM4` in TX (idle and RX: 0x4020).
    tx_patrim4: u32,
    /// `RAC.AGCOVERWRITE1` in TX: maximum manual LNA attenuation of the
    /// active path (`MANLNAMIXRFATT0` or `MANLNAMIXRFATT1`).
    tx_agcoverwrite1: u32,
}

/// 20 dBm (HP) PA, RF path 0: EFR32MG24B220 on the BRD4187A.
#[cfg(not(feature = "efr32mg24-pa10"))]
const P: PaPhy = PaPhy {
    ircal: 0x80FB_FFEF,
    vcogain: 0x0000_00CA,
    lpfctrl2rx: 0x01CB_43A5,
    lpfctrl2tx: 0x008D_23A5,
    vcoctrl: 0x0000_0A4C,
    auxadctrim: 0x16D5_55A2,
    ifadccal: 0x0000_1A00,
    lnamixtrim0: 0x0000_0150,
    lnamixtrim3: 0x0000_020A,
    lnamixtrim4: 0x3109_E702,
    patrim2: 0x0000_0600,
    patrim3: 0x008D_13EA,
    patrim5: 0x001B_1500,
    pgatrim: 0x0000_0F1D,
    pgacal: 0x0000_1F22,
    sytrim1: 0xCFF8_0410,
    rfpathen: 0xB802_0160,
    tx_patrim4: 0x00FF_4420,
    tx_agcoverwrite1: 0x0000_007F,
};

/// 10 dBm (MP) PA, RF path 1: EFR32MG24B210 on the EK2703A (BRD2703A).
/// `PATRIM3`/`SYTRIM1` are the values RAIL leaves after its first MP
/// transmission (`SYLODIVLDOTRIMNDIOTX` 1 instead of 4, no 20 dBm LO
/// delay); `LNAMIXTRIM0` is the reset value (inactive path).
#[cfg(feature = "efr32mg24-pa10")]
const P: PaPhy = PaPhy {
    ircal: 0xFFFF_FFFF,
    vcogain: 0x0000_00A9,
    lpfctrl2rx: 0x01CB_4325,
    lpfctrl2tx: 0x008D_2325,
    vcoctrl: 0x0000_044C,
    auxadctrim: 0x16D5_5522,
    ifadccal: 0x0000_1200,
    lnamixtrim0: 0x0000_0110,
    lnamixtrim3: 0x0000_0288,
    lnamixtrim4: 0x3109_2402,
    patrim2: 0x0000_4011,
    patrim3: 0x003A_10AA,
    patrim5: 0x0017_1900,
    pgatrim: 0x0000_0F19,
    pgacal: 0x0000_2121,
    sytrim1: 0xCFE0_0110,
    rfpathen: 0xB802_0164,
    tx_patrim4: 0x0000_4020,
    tx_agcoverwrite1: 0x007F_0000,
};

/// Whether the shared driver writes the DEVINFO IR-calibration word over
/// [`PaPhy::ircal`] (RAIL does on the HP part only).
pub(super) const DEVINFO_IRCAL_APPLIES: bool = cfg!(not(feature = "efr32mg24-pa10"));

/// RAIL's per-channel `SRCRATIO2`/`DIGMIXFREQ` for channels 11..=26.
///
/// The IF ADC clock follows the LO (`f_LO = (FREQ + IFFREQ) * hfxo / 2^19`,
/// high-side injection), so RAIL recomputes both on every channel from the
/// PHY's `phyInfo`:
///
/// - `SRCRATIO2 = round(2^11 * (f_LO / 8) / src2_denominator)`
///   (`src2Denominator`: 40 MHz in the 39 MHz PHY, 60 MHz in the 38.4 MHz
///   one);
/// - `DIGMIXFREQ = round(IF * dec0 * 2^20 / (f_LO / 64))`, i.e.
///   `IFFREQ * dec0 * 2^26 / (FREQ + IFFREQ)` (GSDK radio configurator,
///   `lynx` `calc_digmixfreq_reg` / `calc_fxo_or_fdec8`; `dec0` = 4 for
///   `MODEM.CF.DEC0` = 2 at 39 MHz, 3 for `DEC0` = 0 at 38.4 MHz).
///
/// The `SRCRATIO2` expression is fitted, not taken from a source: with
/// round-to-nearest both reproduce RAIL's words on all 16 channels of the
/// 39 MHz silicon exactly, and GSDK's channel-11 words at both crystals
/// (test `xg24_channel_words_match_rail`).
#[cfg_attr(feature = "hfxo-39mhz", allow(dead_code))]
pub(super) const fn channel_words(
    hfxo_hz: u64,
    iffreq_word: u64,
    src2_denominator: u64,
    dec0: u64,
) -> ([u16; 16], [u32; 16]) {
    let mut src = [0u16; 16];
    let mut mix = [0u32; 16];
    let mut i = 0;
    while i < 16 {
        let lo = lo_word(11 + i as u64, hfxo_hz) + iffreq_word;
        let den = 2048 * src2_denominator;
        src[i] = ((2 * lo * hfxo_hz + den) / (2 * den)) as u16;
        mix[i] = ((2 * ((iffreq_word * dec0) << 26) + lo) / (2 * lo)) as u32;
        i += 1;
    }
    (src, mix)
}

/// `SYNTH.FREQ` word of an O-QPSK channel (same formula as `synth_freq`).
#[cfg_attr(feature = "hfxo-39mhz", allow(dead_code))]
const fn lo_word(channel: u64, hfxo_hz: u64) -> u64 {
    ((2_405_000_000 + 5_000_000 * (channel - 11)) << 19) / hfxo_hz
}

const MODEM_SRCCHF: u32 = 0xB801_411C;
/// `SRCCHF`: `INTOSR` | `SRCENABLE2`, ratio in `[26:12]`.
const SRCCHF_FIXED: u32 = 0x8800_0000;
const MODEM_DIGMIXCTRL: u32 = 0xB801_413C;
/// `DIGMIXCTRL`: `DIGMIXFB` | `DIGMIXMODE`, frequency in `[19:0]`.
const DIGMIXCTRL_FIXED: u32 = 0x0050_0000;

/// Channel-dependent PHY registers (called with the RSM off, before
/// `SYNTH.FREQ` is written).
pub(super) fn select_channel(channel: u8) {
    let i = usize::from(channel.clamp(11, 26) - 11);
    unsafe {
        write(L.synth + SYNTH_CHCTRL, 0);
        write(
            MODEM_SRCCHF,
            SRCCHF_FIXED | (u32::from(X.srcratio[i]) << 12),
        );
        write(MODEM_DIGMIXCTRL, DIGMIXCTRL_FIXED | X.digmixfreq[i]);
    }
}

/// Static PHY + MAC-framing configuration, in ascending address order.
/// `MODEM.SRCCHF`/`DIGMIXCTRL` are per channel ([`select_channel`]).
pub(super) const INIT: &[(u32, u32)] = &[
    (0xB8004004, 0x00000001),         // FRC.EN
    (0xB800400C, 0x00148001),         // FRC.DFLCTRL
    (0xB8004010, 0x0000407F),         // FRC.MAXLENGTH
    (0xB8004020, 0x000000FF),         // FRC.WCNTCMP0
    (0xB8004040, 0x00000000),         // FRC.BLOCKRAMADDR
    (0xB8004048, 0x030407A0),         // FRC.CTRL
    (0xB800404C, 0x00000060),         // FRC.RXCTRL
    (0xB8004054, 0x0000001B),         // FRC.TRAILRXDATA
    (0xB8004060, 0x00000000),         // FRC.PUNCTCTRL
    (0xB80040A8, 0x00000007),         // FRC.AUTOCG
    (0xB8004108, 0x00004000),         // FRC.FCD0
    (0xB800410C, 0x00004CFF),         // FRC.FCD1
    (0xB8004110, 0x00004100),         // FRC.FCD2
    (0xB8004114, 0x00004DFF),         // FRC.FCD3
    (0xB800C004, 0x00000001),         // AGC.EN
    (0xB800C020, 0x01293214),         // AGC.CTRL0
    (0xB800C024, 0x00004300),         // AGC.CTRL1
    (0xB800C028, 0x03B380EC),         // AGC.CTRL2
    (0xB800C02C, 0x51407543),         // AGC.CTRL3
    (0xB800C030, 0xF8000FA0),         // AGC.CTRL4
    (0xB800C034, 0x00004000),         // AGC.CTRL5
    (0xB800C038, 0x0007AAA8),         // AGC.CTRL6
    (0xB800C044, 0x0000022E),         // AGC.MANGAIN
    (0xB800C054, 0x00303151),         // AGC.GAINRANGE
    (0xB800C058, 0xE60D000E),         // AGC.AGCPERIOD0
    (0xB800C05C, 0x0000002A),         // AGC.AGCPERIOD1
    (0xB800C060, 0x0D0C0B08),         // AGC.HICNTREGION0
    (0xB800C064, 0x0000000D),         // AGC.HICNTREGION1
    (0xB800C068, 0x0002B6D1),         // AGC.STEPDWN
    (0xB800C06C, X.agc_gainsteplim0), // AGC.GAINSTEPLIM0 (crystal)
    (0xB800C070, 0x000010BA),         // AGC.GAINSTEPLIM1
    (0xB800C088, 0x0000007F),         // AGC.PNRFATT5
    (0xB800C08C, 0x00000000),         // AGC.PNRFATT6
    (0xB800C090, 0x00000000),         // AGC.PNRFATT7
    (0xB800C0BC, 0x00000003),         // AGC.MIRRORIF
    (0xB800C0CC, 0x00000001),         // AGC.LNABOOST
    (0xB8010004, 0x00000001),         // RFCRC.EN
    (0xB8010020, 0x00008408),         // RFCRC.POLY
    (0xB8014004, 0x00000001),         // MODEM.EN
    (0xB8014048, 0x00000010),         // MODEM.MIXCTRL
    (0xB801404C, 0x0413F920),         // MODEM.CTRL0
    (0xB8014050, 0x00620007),         // MODEM.CTRL1
    (0xB8014054, 0x00007038),         // MODEM.CTRL2
    (0xB8014058, 0x00000000),         // MODEM.CTRL3
    (0xB8014060, 0x0F016800),         // MODEM.CTRL5
    (0xB8014064, 0x4024B840),         // MODEM.CTRL6
    (0xB8014068, X.txbr),             // MODEM.TXBR (crystal)
    (0xB801406C, 0x00000841),         // MODEM.RXBR
    (0xB8014070, X.cf),               // MODEM.CF (crystal)
    (0xB8014074, 0x000807B0),         // MODEM.PRE
    (0xB8014078, 0x000000A7),         // MODEM.SYNC0
    (0xB8014080, 0x00006323),         // MODEM.TIMING
    (0xB8014084, 0x744AC39B),         // MODEM.DSSS0
    (0xB8014088, 0x000F03F0),         // MODEM.MODINDEX
    (0xB801408C, 0x60008000),         // MODEM.AFC
    (0xB8014090, 0x00000ABE),         // MODEM.AFCADJLIM
    (0xB8014094, 0x30100101),         // MODEM.SHAPING0
    (0xB8014098, 0x7F7F7050),         // MODEM.SHAPING1
    (0xB80140E0, 0x00000200),         // MODEM.ANARAMPCTRL
    (0xB8014110, 0x00051E33),         // MODEM.DCCOMP
    (0xB8014120, 0x00000B59),         // MODEM.INTAFC
    (0xB8014124, 0x078304FF),         // MODEM.DSATHD0
    (0xB801412C, 0x0C6606FF),         // MODEM.DSATHD2
    (0xB8014130, 0x078304FF),         // MODEM.DSATHD3
    (0xB8014134, 0x03FF1388),         // MODEM.DSATHD4
    (0xB8014138, 0xF00A2090),         // MODEM.DSACTRL
    (0xB8014140, 0x00000000),         // MODEM.VITERBIDEMOD
    (0xB8014148, 0x50000000),         // MODEM.VTCORRCFG1
    (0xB801414C, 0x00003B80),         // MODEM.VTTRACK
    (0xB801415C, 0x00001E00),         // MODEM.CGCLKSTOP
    (0xB801416C, 0xC00C1400),         // MODEM.LONGRANGE1
    (0xB8014170, 0x28211A14),         // MODEM.LONGRANGE2
    (0xB8014174, 0x423B342E),         // MODEM.LONGRANGE3
    (0xB8014178, 0x55435049),         // MODEM.LONGRANGE4
    (0xB801417C, 0x0CBA9876),         // MODEM.LONGRANGE5
    (0xB8014180, 0x00007323),         // MODEM.LONGRANGE6
    (0xB8014188, 0x64282117),         // MODEM.COH0
    (0xB801418C, 0x001A1714),         // MODEM.COH1
    (0xB8014190, 0x7DC80420),         // MODEM.COH2
    (0xB8014194, 0x093A20A1),         // MODEM.COH3
    (0xB80141A8, 0x00000029),         // MODEM.DIGIGAINCTRL
    (0xB80141B4, 0x00200000),         // MODEM.REALTIMCFE
    (0xB80141C4, X.antswctrl1),       // MODEM.ANTSWCTRL1 (crystal)
    (0xB80141D0, 0x00000000),         // MODEM.TRECPMPATT
    (0xB80141D4, 0x000000D0),         // MODEM.TRECPMDET
    (0xB80141D8, 0x00020000),         // MODEM.TRECSCFG
    (0xB80141E4, X.chfcoe[0]),        // MODEM.CHFCOE00 (crystal)
    (0xB80141E8, X.chfcoe[1]),        // MODEM.CHFCOE01 (crystal)
    (0xB80141EC, X.chfcoe[2]),        // MODEM.CHFCOE02 (crystal)
    (0xB80141F0, X.chfcoe[3]),        // MODEM.CHFCOE03 (crystal)
    (0xB80141F4, X.chfcoe[4]),        // MODEM.CHFCOE04 (crystal)
    (0xB80141F8, X.chfcoe[5]),        // MODEM.CHFCOE05 (crystal)
    (0xB80141FC, X.chfcoe[6]),        // MODEM.CHFCOE06 (crystal)
    (0xB8014200, X.chfcoe[0]),        // MODEM.CHFCOE10 (crystal)
    (0xB8014204, X.chfcoe[1]),        // MODEM.CHFCOE11 (crystal)
    (0xB8014208, X.chfcoe[2]),        // MODEM.CHFCOE12 (crystal)
    (0xB801420C, X.chfcoe[3]),        // MODEM.CHFCOE13 (crystal)
    (0xB8014210, X.chfcoe[4]),        // MODEM.CHFCOE14 (crystal)
    (0xB8014214, X.chfcoe[5]),        // MODEM.CHFCOE15 (crystal)
    (0xB8014218, X.chfcoe[6]),        // MODEM.CHFCOE16 (crystal)
    (0xB8014224, 0x0000002C),         // MODEM.FRMSCHTIME
    (0xB8014228, 0x3675EE07),         // MODEM.PREFILTCOEFF
    (0xB801422C, 0x40001860),         // MODEM.RXRESTART
    (0xB8014238, 0x00000001),         // MODEM.SQI
    (0xB8014244, 0x00000014),         // MODEM.PHDMODANTDIV
    (0xB801424C, 0x04000008),         // MODEM.PHDMODCTRL
    (0xB8014278, P.ircal),            // MODEM.IRCALCOEFWR0 (PA; HP: DEVINFO overrides)
    (0xB801427C, P.ircal),            // MODEM.IRCALCOEFWR1 (PA; HP: DEVINFO overrides)
    (0xB8014280, 0x00000000),         // MODEM.ADCTRL1
    (0xB8014284, 0x00000081),         // MODEM.ADCTRL2
    (0xB8014308, 0x00000100),         // MODEM.ADFSM21
    (0xB8014400, 0x00000001),         // MODEM.SPARE
    (0xB8018004, 0x00000001),         // SYNTH.EN
    (0xB8018038, X.iffreq),           // SYNTH.IFFREQ (crystal)
    (0xB8018058, P.vcogain),          // SYNTH.VCOGAIN (die)
    (0xB801809C, 0x000240EB),         // SYNTH.LPFCTRL1CAL
    (0xB80180A0, 0x00037870),         // SYNTH.LPFCTRL1RX
    (0xB80180A4, 0x0000C0D5),         // SYNTH.LPFCTRL1TX
    (0xB80180A8, P.lpfctrl2rx),       // SYNTH.LPFCTRL2RX (die)
    (0xB80180AC, P.lpfctrl2tx),       // SYNTH.LPFCTRL2TX (die)
    (0xB80180B0, 0x02000300),         // SYNTH.DSMCTRLRX
    (0xB80180B4, 0x01000037),         // SYNTH.DSMCTRLTX
    (0xB8020004, 0x00000001),         // RAC.EN
    (0xB8020014, 0x08000380),         // RAC.CTRL (SEQCLKDIS, CPUWAITDIS=0)
    (0xB802003C, 0x00000011),         // RAC.SEQCTRL
    (0xB8020054, 0x01061A7F),         // RAC.STCTRL
    (0xB802009C, 0x02000C00),         // RAC.SYNTHREGCTRL
    (0xB80200A0, P.vcoctrl),          // RAC.VCOCTRL (die)
    (0xB80200A4, 0x00004000),         // RAC.CHPCTRL
    (0xB80200B4, 0x0E200000),         // RAC.IFPGACTRL
    (0xB80200C0, 0x00000400),         // RAC.ANTDIV
    (0xB80200C4, P.auxadctrim),       // RAC.AUXADCTRIM (die)
    (0xB80200EC, 0x11512F2C),         // RAC.IFADCTRIM0
    (0xB80200F0, 0x0000012B),         // RAC.IFADCTRIM1
    (0xB80200F4, P.ifadccal),         // RAC.IFADCCAL (die)
    (0xB8020100, P.lnamixtrim0),      // RAC.LNAMIXTRIM0 (RF path)
    #[cfg(feature = "efr32mg24-pa10")]
    (0xB8020104, 0x00000150), // RAC.LNAMIXTRIM1 (RF path 1)
    (0xB8020108, 0x545043D0),         // RAC.LNAMIXTRIM2
    (0xB802010C, P.lnamixtrim3),      // RAC.LNAMIXTRIM3 (die)
    (0xB8020110, P.lnamixtrim4),      // RAC.LNAMIXTRIM4 (die)
    #[cfg(feature = "efr32mg24-pa10")]
    (0xB8020120, 0x000000A4), // RAC.PATRIM0 (MP PA)
    (0xB8020124, 0x00034373),         // RAC.PATRIM1
    (0xB8020128, P.patrim2),          // RAC.PATRIM2 (PA)
    (0xB802012C, P.patrim3),          // RAC.PATRIM3 (PA)
    (0xB8020134, P.patrim5),          // RAC.PATRIM5 (PA)
    (0xB8020148, P.pgatrim),          // RAC.PGATRIM (die)
    (0xB802014C, P.pgacal),           // RAC.PGACAL (die)
    (0xB8020154, 0x30111A17),         // RAC.RFBIASCAL
    (0xB8020158, 0x00040004),         // RAC.RFBIASCTRL
    (0xB8020174, 0x0C1B8169),         // RAC.SYTRIM0
    (0xB8020178, P.sytrim1),          // RAC.SYTRIM1 (PA)
    (0xB8020188, 0x00000050),         // RAC.SYMMDCTRL
    (0xB8020198, 0x00000770),         // RAC.XORETIMECTRL
];

// ---- Per-state analog values ----------------------------------------------

/// Number of RAC/analog registers switched per radio state.
pub(super) const STATE_REGS: usize = 14;

/// RAC registers RAIL's sequencer rewrites on every state change (diff of
/// the idle, RX-search and TX snapshots, identical on all channels).
const RAC_STATE_REGS: [u32; STATE_REGS] = [
    0xB802_0098, // RAC.SYNTHENCTRL
    0xB802_00B8, // RAC.PAENCTRL
    0xB802_00E8, // RAC.IFADCDBG
    0xB802_0118, // RAC.LNAMIXEN
    0xB802_0130, // RAC.PATRIM4
    0xB802_0150, // RAC.PGACTRL
    0xB802_015C, // RAC.RADIOEN
    P.rfpathen,  // RAC.RFPATHEN0 (HP) / RFPATHEN1 (MP)
    0xB802_0168, // RAC.RX
    0xB802_016C, // RAC.TX
    0xB802_0180, // RAC.SYEN
    0xB802_0184, // RAC.SYLOEN
    0xB802_01A0, // RAC.AGCOVERWRITE0
    0xB802_01A4, // RAC.AGCOVERWRITE1
];
const SYNTH_VCDACCTRL: u32 = 0xB801_802C;
const SYNTH_VCORANGE: u32 = 0xB801_8054;
/// `MODEM.IRCAL`: `IRCORREN` | `IRCALEN` in RX only.
const MODEM_IRCAL: u32 = 0xB801_4270;
/// `MODEM.PADEBUG`; RAIL sets `ENMANPAPOWER` | `ENMANPASELSLICE`
/// ([`PADEBUG_MANUAL`]) when it powers the PA up and keeps it through TX:
/// the PA then takes slice and power from `RAC.PACTRL` instead of the
/// MODEM ramp.
const MODEM_PADEBUG: u32 = 0xB801_41B0;
#[cfg_attr(feature = "efr32mg24-pa10", allow(dead_code))]
const PADEBUG_MANUAL: u32 = 0x1800;

/// `RAC.PAENCTRL.INVRAMPCLK`; `PARAMP` is driven by [`pa_ramp_up`].
const PAENCTRL_IDLE: u32 = 0x0001_0000;

#[rustfmt::skip]
pub(super) const IDLE: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0000_0000, PAENCTRL_IDLE, 0x0000_001E, 0x0000_0000,
        0x0000_4020, 0x00A2_80C1, 0x0000_0000, 0x0000_0000,
        0x0006_0000, 0x0001_0000, 0x0000_0002, 0x0000_0000,
        0x0000_1400, 0x3FFF_0000,
    ],
    extra: &[
        (SYNTH_VCDACCTRL, 0x25),
        (SYNTH_VCORANGE, 0xC8),
        (MODEM_IRCAL, 0),
        (MODEM_PADEBUG, 0),
    ],
};

#[rustfmt::skip]
pub(super) const RX: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0000_0600, PAENCTRL_IDLE, 0x0000_001F, 0x0000_0008,
        0x0000_4020, 0x00A2_8EF1, 0x0000_0007, 0x0000_000F,
        0x0006_013E, 0x0001_0000, 0x0000_077B, 0x0000_001E,
        0x0000_1400, 0x3FFF_0000,
    ],
    extra: &[
        (SYNTH_VCDACCTRL, 0xA9),
        (SYNTH_VCORANGE, 0xC8),
        (MODEM_IRCAL, 0x2001),
        (MODEM_PADEBUG, 0),
    ],
};

/// `RAC.TX` while the RSM warms up for TX: PA off (the idle value).
/// [`pa_ramp_up`] powers the PA in RAIL's order once the synthesizer is
/// locked.
const TX_PA_BIAS: u32 = 0x0001_0000;

#[rustfmt::skip]
pub(super) const TX: AnalogState = AnalogState {
    regs: &RAC_STATE_REGS,
    values: [
        0x0010_0600, PAENCTRL_IDLE, 0x0000_001E, 0x0000_0008,
        P.tx_patrim4, 0x00A2_80C1, 0x0000_0007, 0x0000_0042,
        0x0006_00A0, TX_PA_BIAS,   0x0000_077B, 0x0000_080E,
        // RAIL holds the LNA at manual maximum attenuation while it
        // transmits (shared antenna path).
        0x0000_0003, P.tx_agcoverwrite1,
    ],
    extra: &[
        (SYNTH_VCDACCTRL, 0xA5),
        (SYNTH_VCORANGE, 0x158),
        (MODEM_IRCAL, 0),
        // HP: manual PA slice/power (`PADEBUG_MANUAL`) from `pa_ramp_up`.
        (MODEM_PADEBUG, 0),
    ],
};

// ---- PA power ---------------------------------------------------------------

/// `(deci-dBm, MODEM.RAMPLEV, RAC.PACTRL)` as programmed by GSDK 4.5.0
/// `RAIL_SetTxPowerDbm` on the BRD4187A (2P4GIG_HP, 3.3 V, RAIL ramp 6 us),
/// with RAIL's reported output power. `PACTRL` holds `TXPASELSLICE[15:12]`
/// and `TXPAPOWER[7:4]`; the ramp level is the per-die DEVINFO byte of that
/// slice ([`DEVINFO_RAMPLEV`]), the column is the captured die's value and
/// the fallback for an erased DEVINFO. Ascending power. The dBm column is
/// RAIL's PA-curve estimate, not a conducted measurement.
#[cfg(feature = "efr32mg24-pa20")]
pub(super) const TX_POWER: &[(i16, u8, u16)] = &[
    (-249, 0xD6, 0x1010),
    (-158, 0xD6, 0x1020),
    (-77, 0xD6, 0x1040),
    (-60, 0xD6, 0x1050),
    (-43, 0xD6, 0x1060),
    (-32, 0xD6, 0x1070),
    (-22, 0xD6, 0x1080),
    (-12, 0xD6, 0x1090),
    (-2, 0xD6, 0x10A0),
    (9, 0xD6, 0x10C0),
    (15, 0xD6, 0x10D0),
    (27, 0xD6, 0x10F0),
    (39, 0xD5, 0x2020),
    (48, 0xD5, 0x2040),
    (59, 0xD5, 0x2070),
    (67, 0xD5, 0x2090),
    (79, 0xD5, 0x20C0),
    (90, 0xD5, 0x3020),
    (99, 0xD5, 0x3060),
    (108, 0xD5, 0x30A0),
    (118, 0xD5, 0x30E0),
    (130, 0xD5, 0x4080),
    (140, 0xD5, 0x5010),
    (149, 0xD5, 0x5080),
    (159, 0xD4, 0x6010),
    (170, 0xD3, 0x7060),
    (180, 0xD3, 0x80E0),
    (190, 0xD2, 0xA060),
    (200, 0xD1, 0xB0E0),
];

/// `(deci-dBm, MODEM.RAMPLEV, MODEM.RAMPCTRL)` as programmed by GSDK 4.5.0
/// `RAIL_SetTxPowerDbm` on the EK2703A (EFR32MG24B210, 2P4GIG_HIGHEST = MP
/// PA, 3.3 V, RAIL ramp 10 us), RAIL raw levels 0..=79, with RAIL's
/// reported output power (its PA-curve estimate). The MP PA takes its
/// power from the MODEM ramp (`PADEBUG` stays 0, `PACTRL` holds only
/// `TXPASELPREDRVREGVDDPA` in TX): `RAMPLEV` is the level (raw + 16..21),
/// `RAMPCTRL` the per-band ramp rate RAIL switches with it. Ascending power.
#[cfg(feature = "efr32mg24-pa10")]
pub(super) const TX_POWER: &[(i16, u8, u16)] = &[
    (-300, 0x10, 0x999),
    (-229, 0x11, 0x999),
    (-173, 0x12, 0x999),
    (-118, 0x13, 0x999),
    (-62, 0x14, 0x999),
    (-54, 0x15, 0x999),
    (-40, 0x16, 0x888),
    (-28, 0x17, 0x888),
    (-17, 0x18, 0x888),
    (-2, 0x1A, 0x888),
    (5, 0x1B, 0x888),
    (16, 0x1D, 0x888),
    (27, 0x1F, 0x888),
    (38, 0x23, 0x888),
    (49, 0x26, 0x888),
    (60, 0x2B, 0x888),
    (69, 0x31, 0x777),
    (80, 0x3A, 0x777),
    (90, 0x4F, 0x777),
    (100, 0x64, 0x666),
];

/// DEVINFO bytes 0x250..0x25B: HP-PA ramp level per slice 1..=12 (on the
/// captured die `D6 D5 D5 D5 D5 D4 D3 D3 D2 D2 D1 D0`, exactly the
/// `MODEM.RAMPLEV` RAIL programs for each slice).
#[cfg(not(feature = "efr32mg24-pa10"))]
pub(super) const DEVINFO_RAMPLEV: u32 = 0x0FE0_8250;

/// `MODEM.RAMPLEV` offset (0xCC on xG22).
const MODEM_RAMPLEV: u32 = 0xDC;
const RAC_PACTRL: u32 = 0xB802_01C8;
/// `PACTRL.TXPASELPREDRVREGVDDPA`: set by RAIL in TX only.
const PACTRL_TX: u32 = 1 << 10;

/// Ramp level of a `PACTRL` setting: the slice's DEVINFO byte, else
/// `fallback`.
#[cfg(not(feature = "efr32mg24-pa10"))]
fn ramp_level(pa: u16, fallback: u8) -> u8 {
    let slice = u32::from((pa >> 12) & 0xF);
    if slice == 0 {
        return fallback;
    }
    let address = DEVINFO_RAMPLEV + slice - 1;
    // SAFETY: DEVINFO is a read-only information page; byte reads are
    // allowed anywhere in it.
    let level = unsafe { core::ptr::read_volatile(address as *const u8) };
    if level == 0xFF { fallback } else { level }
}

/// Program a [`TX_POWER`] entry: all three ramp levels and the idle
/// `PACTRL` (slice and power).
#[cfg(not(feature = "efr32mg24-pa10"))]
pub(super) fn write_tx_power(ramplev: u8, pa: u16) {
    let lev = u32::from(ramp_level(pa, ramplev));
    unsafe {
        write(L.modem + MODEM_RAMPLEV, lev | (lev << 8) | (lev << 16));
        write(RAC_PACTRL, u32::from(pa));
    }
}

/// `PACTRL` for TX (after [`TX`] was applied).
#[cfg(not(feature = "efr32mg24-pa10"))]
pub(super) fn tx_state_power(ddbm: i16) {
    let (_, _, pa) = super::tx_power_entry(ddbm);
    unsafe { write(RAC_PACTRL, u32::from(pa) | PACTRL_TX) };
}

/// `MODEM.RAMPCTRL` (0x555 at reset, which the HP capture kept).
#[cfg(feature = "efr32mg24-pa10")]
const MODEM_RAMPCTRL: u32 = 0xD8;

/// Program a [`TX_POWER`] entry of the MP PA: the three ramp levels and
/// the ramp rate; `PACTRL` carries no slice or power.
#[cfg(feature = "efr32mg24-pa10")]
pub(super) fn write_tx_power(ramplev: u8, rampctrl: u16) {
    let lev = u32::from(ramplev);
    unsafe {
        write(L.modem + MODEM_RAMPLEV, lev | (lev << 8) | (lev << 16));
        write(L.modem + MODEM_RAMPCTRL, u32::from(rampctrl));
        write(RAC_PACTRL, 0);
    }
}

/// `PACTRL` for TX on the MP PA: `TXPASELPREDRVREGVDDPA` only.
#[cfg(feature = "efr32mg24-pa10")]
pub(super) fn tx_state_power(_ddbm: i16) {
    unsafe { write(RAC_PACTRL, PACTRL_TX) };
}

// ---- PA power ramp ----------------------------------------------------------
//
// The sequence follows RAIL's TX transition, sampled by an on-target loop
// every 11 us: on the BRD4187A (HP) at 0, 5, 10, 15 and 20 dBm, on the
// EK2703A (MP) at -30, -10, 0, 5 and 10 dBm (identical at every level apart
// from the power registers):
//
// 1. `PACTRL.TXPASLICERST` set (with `TXPASELPREDRVREGVDDPA`), on the HP PA
//    `MODEM.PADEBUG` manual slice/power, `RAC.TX` = regulators, bleeders,
//    power/slice enables (+ the 20 dBm `VMID` on HP);
// 2. 11..40 us later `RAC.TX` = output stages with `TXPAENRAMPCLK`, slice
//    reset released, `PAENCTRL.PARAMP` set;
// 3. after the ramp `RAC.TX` = steady value (bleeders and ramp clock off).
//
// `TXPD` reverses it: ramp clock and bleeders on, `PARAMP` cleared, then
// the PA off. Without step 1 (the order the xG21 driver uses) the PA still
// delivers the requested output but draws more than the WSTK AEM's 95 mA
// at every power level. As on xG21, the CPU drives the ramp at the end of
// a held `TXWARM` and in a held `TXPD`.

const RAC_RFSTATUS: u32 = 0xB802_00AC;
const RFSTATUS_MODRAMPUPDONE: u32 = 1 << 0;
const RAC_PAENCTRL: u32 = 0xB802_00B8;
const PAENCTRL_PARAMP: u32 = 1 << 8;
const RAC_TX: u32 = 0xB802_016C;
/// Bounded wait for `MODRAMPUPDONE`. HP: RAIL reports a 6 us ramp
/// (`MODEM.RAMPCTRL` 0x555). MP: RAIL ramps over 10 us and slower at low
/// levels (`RAMPCTRL` 0x999; up to 36 us between sampled steps).
#[cfg(not(feature = "efr32mg24-pa10"))]
const PA_RAMP_US: u32 = 30;
#[cfg(feature = "efr32mg24-pa10")]
const PA_RAMP_US: u32 = 60;
/// `PACTRL.TXPASLICERST`.
const PACTRL_SLICERST: u32 = 1 << 16;
/// Step 1 (HP): `TXPAENREG`, `TXPAENBLEEDPREDRVREG`, `TXPAENBLEEDREG`,
/// `SYCHPBIASTRIMBUFTX`, `TXPAEN20DBMVMID`, `ENPAPOWER`, `ENPASELSLICE`.
#[cfg(not(feature = "efr32mg24-pa10"))]
const TX_PA_PRESTAGE: u32 = 0xC401_3001;
/// Step 2 and `TXPD` (HP): adds `TXPAENRAMPCLK`, `TXPAENPAOUT`,
/// `TXPAENPREDRVREG` and the 20 dBm stages (bits 22..25).
#[cfg(not(feature = "efr32mg24-pa10"))]
const TX_PA_RAMP: u32 = 0xC3C1_F003;
/// Step 3 (HP): steady TX.
#[cfg(not(feature = "efr32mg24-pa10"))]
const TX_PA_ON: u32 = 0xC3C1_C001;
/// HP: slice and power come from `PACTRL` (manual), not the ramp.
#[cfg(not(feature = "efr32mg24-pa10"))]
const TX_PADEBUG: u32 = PADEBUG_MANUAL;
/// Step 1 (MP): as HP without the 20 dBm `VMID`.
#[cfg(feature = "efr32mg24-pa10")]
const TX_PA_PRESTAGE: u32 = 0xC001_3001;
/// Step 2 and `TXPD` (MP): `TXPAENRAMPCLK`, `TXPAENPAOUT`,
/// `TXPAENPREDRVREG`, `TXPAEN10DBM` and `TXPAEN10DBMPREDRV`.
#[cfg(feature = "efr32mg24-pa10")]
const TX_PA_RAMP: u32 = 0xC029_F003;
/// Step 3 (MP): steady TX (RAIL `0xC029C001`).
#[cfg(feature = "efr32mg24-pa10")]
const TX_PA_ON: u32 = 0xC029_C001;
/// MP: the level comes from the MODEM ramp (`RAMPLEV`), `PADEBUG` stays 0.
#[cfg(feature = "efr32mg24-pa10")]
const TX_PADEBUG: u32 = 0;
/// RAIL: pre-stage 11..40 us, ramp clock 11..35 us; the longer values.
/// [`pa_prestage`] runs this long before [`pa_ramp_start`].
pub(super) const PA_PRESTAGE_US: u32 = 40;
const PA_RAMP_CLK_US: u32 = 16;
/// Time from [`pa_ramp_start`] to a steady PA: the ramp clock. The ramp
/// itself is shorter (RAIL: 6 us HP, 10 us MP); [`pa_ramp_finish`] waits
/// for a slower one, up to [`PA_RAMP_US`].
#[cfg_attr(not(feature = "isr-ack"), allow(dead_code))]
pub(super) const PA_RAMP_LEAD_US: u32 = PA_RAMP_CLK_US;

/// Read-modify-write: the sequencer is clock-gated, the CPU is the only
/// writer of these registers.
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

fn delay_us(us: u32) {
    let deadline = Deadline::after_us(us);
    while !deadline.expired() {}
}

/// The driver holds `TXPD` and calls [`pa_ramp_down`] there.
pub(super) const PA_RAMP_IN_TXPD: bool = true;

/// Power and ramp the PA up at the end of `TXWARM` (synthesizer locked,
/// state held). [`tx_state_power`] has loaded `PACTRL`.
pub(super) fn pa_ramp_up() {
    pa_prestage();
    delay_us(PA_PRESTAGE_US);
    pa_ramp();
}

/// Step 1 of [`pa_ramp_up`]: slices in reset, regulators up. The ramp
/// follows [`PA_PRESTAGE_US`] later (the interrupt Imm-Ack returns in
/// between).
pub(super) fn pa_prestage() {
    set_bits(RAC_PACTRL, PACTRL_SLICERST, true);
    unsafe {
        write(MODEM_PADEBUG, TX_PADEBUG);
        write(RAC_TX, TX_PA_PRESTAGE);
    }
}

/// Steps 2 and 3 of [`pa_ramp_up`]: the ramp, then the steady PA.
fn pa_ramp() {
    let started = cycles();
    pa_ramp_start();
    pa_ramp_finish(started);
}

/// Start the ramp (step 2). [`pa_ramp_finish`] may follow up to
/// [`PA_RAMP_LEAD_US`] later without waiting.
pub(super) fn pa_ramp_start() {
    unsafe { write(RAC_TX, TX_PA_RAMP) };
    set_bits(RAC_PACTRL, PACTRL_SLICERST, false);
    set_bits(RAC_PAENCTRL, PAENCTRL_PARAMP, true);
}

/// Wait out the ramp started at cycle stamp `started` (`MODRAMPUPDONE`, at
/// most [`PA_RAMP_US`], and at least the ramp clock), then the steady PA
/// (step 3).
pub(super) fn pa_ramp_finish(started: u32) {
    let elapsed = || cycles().wrapping_sub(started);
    while unsafe { read(RAC_RFSTATUS) } & RFSTATUS_MODRAMPUPDONE == 0
        && elapsed() < us_to_cycles(PA_RAMP_US)
    {}
    while elapsed() < us_to_cycles(PA_RAMP_CLK_US) {}
    unsafe { write(RAC_TX, TX_PA_ON) };
}

/// Ramp the PA down in the held `TXPD` state; [`tx_done`] switches it off.
pub(super) fn pa_ramp_down() {
    unsafe { write(RAC_TX, TX_PA_RAMP) };
    set_bits(RAC_PAENCTRL, PAENCTRL_PARAMP, false);
    wait_ramp(false);
}

/// `RAC.TX` with the PA regulators off and their bleeders on (RAIL's
/// `TXPD` exit, HP frame snapshot `0xC3C1_3000`), then the idle value.
const TX_BLEED: u32 = (TX_PA_ON & !(TX_PAENREG | TX_PAENPAOUT | TX_PAENPREDRVREG)) | TX_PABLEEDERS;
const TX_PAENREG: u32 = 1 << 0;
const TX_PAENPAOUT: u32 = 1 << 14;
const TX_PAENPREDRVREG: u32 = 1 << 15;
/// `TXPAENBLEEDPREDRVREG` | `TXPAENBLEEDREG`.
const TX_PABLEEDERS: u32 = 0x3000;
#[cfg(not(feature = "efr32mg24-pa10"))]
const _: () = assert!(TX_BLEED == 0xC3C1_3000);
const RAC_AGCOVERWRITE0: u32 = 0xB802_01A0;
const RAC_AGCOVERWRITE1: u32 = 0xB802_01A4;

// `tx_done` restores these entries of the idle state by index.
const _: () = assert!(
    RAC_STATE_REGS[9] == RAC_TX
        && RAC_STATE_REGS[12] == RAC_AGCOVERWRITE0
        && RAC_STATE_REGS[13] == RAC_AGCOVERWRITE1
);

/// End of a transmission, after [`pa_ramp_down`] or an aborted frame:
/// release `TXPD` and switch the HP PA off. Its regulators and bias left
/// at the [`TX`] values draw more than the WSTK AEM's 95 mA range with the
/// RSM off (measured on the BRD4187A), so the TX analog state must not
/// outlive the transmission even when no RX or idle state follows.
pub(super) fn tx_done() {
    seq_release(Warm::TxPd);
    unsafe {
        write(RAC_TX, TX_BLEED);
        write(RAC_PAENCTRL, PAENCTRL_IDLE);
        let pactrl = read(RAC_PACTRL);
        write(RAC_PACTRL, pactrl & !(PACTRL_TX | PACTRL_SLICERST));
        write(MODEM_PADEBUG, 0);
        write(RAC_TX, IDLE.values[9]);
        write(RAC_AGCOVERWRITE0, IDLE.values[12]);
        write(RAC_AGCOVERWRITE1, IDLE.values[13]);
    }
}

// ---- Radio state hand-shake (xG22 layout, `TXPD` held too) ---------------

const RAC_SEQIF: u32 = 0x28;
const RAC_SEQIF_ALL: u32 = 0x3FFF_000F;
const RAC_SEQIF_RXWARM: u32 = 1 << 17;
const RAC_SEQIF_TXWARM: u32 = 1 << 24;
const RAC_SEQIF_TXPD: u32 = 1 << 26;
/// Warm states are held until the driver has locked the synthesizer (and
/// ramped the PA up); `TXPD` until it has ramped the PA down.
const RAC_SEQIF_HELD: u32 = RAC_SEQIF_RXWARM | RAC_SEQIF_TXWARM | RAC_SEQIF_TXPD;

const fn seqif(warm: Warm) -> u32 {
    match warm {
        Warm::Rx => RAC_SEQIF_RXWARM,
        Warm::Tx => RAC_SEQIF_TXWARM,
        Warm::TxPd => RAC_SEQIF_TXPD,
    }
}

/// Drop any stale state hand-shake.
pub(super) fn seq_init() {
    unsafe { write(L.rac + RAC_SEQIF + CLR, RAC_SEQIF_ALL) };
}

/// Release every state except the held ones.
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

/// Held states are always held on xG24.
#[inline(always)]
pub(super) fn seq_arm(_warm: Warm) {}

/// Release a held state. `TXPD` is released until the RSM leaves it: the
/// `SEQIF` flag can rise just after `STATUS` shows the state (xG22), and a
/// lost clear would park the RSM there.
pub(super) fn seq_release(warm: Warm) {
    unsafe { write(L.rac + RAC_SEQIF + CLR, seqif(warm)) };
    if matches!(warm, Warm::TxPd) {
        let deadline = Deadline::after_us(20);
        while rac_state_raw() == RacState::TxPowerDown && !deadline.expired() {
            unsafe { write(L.rac + RAC_SEQIF + CLR, RAC_SEQIF_TXPD) };
        }
    }
}
