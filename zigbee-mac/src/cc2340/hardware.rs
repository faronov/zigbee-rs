//! CC2340R5 LRFD register and boot primitives.
//!
//! These operations are direct Rust translations of TI's public
//! `driverlib/setup.c`, `driverlib/lrfd.c`, and `RCL/LRF.c` bring-up paths.

use core::hint::spin_loop;

use super::config::{RegisterWidth, RegisterWrite};

const CKMD_BASE: u32 = 0x4000_1000;
const CLKCTL_BASE: u32 = 0x4002_0000;
const EVTSVT_BASE: u32 = 0x4002_5000;
const SYSTIM_BASE: u32 = 0x4002_2000;
const LRFDDBELL_BASE: u32 = 0x4008_0000;
const LRFDPBE_BASE: u32 = 0x4008_1000;
const LRFDMDM_BASE: u32 = 0x4008_2000;
const LRFDRFE_BASE: u32 = 0x4008_3000;
const LRFDMDM32_BASE: u32 = 0x4008_2400;
const LRFDRFE32_BASE: u32 = 0x4008_3400;
const FCFG_BASE: u32 = 0x4E00_0000;

const PBE_RAM_BASE: u32 = 0x4009_0000;
const BUF_RAM_BASE: u32 = 0x4009_2000;
const MCE_RAM_BASE: u32 = 0x4009_4000;
const RFE_RAM_BASE: u32 = 0x4009_6000;
const TOPSM_RAM_WORDS: usize = 0x1000 / size_of::<u32>();

const FCFG_APP_TRIMS: u32 = FCFG_BASE + 0x330;
const CKMD_HFXTINIT: u32 = CKMD_BASE + 0x118;
const CKMD_HFXTTARG: u32 = CKMD_BASE + 0x11C;
const CKMD_HFTRACKCTL: u32 = CKMD_BASE + 0x0A4;
const CLKCTL_CLKCFG0: u32 = CLKCTL_BASE + 0x00C;
const CLKCTL_CLKENSET0: u32 = CLKCTL_BASE + 0x014;
const CLKCTL_CLKENCLR0: u32 = CLKCTL_BASE + 0x020;
const EVTSVT_CPUIRQ4SEL: u32 = EVTSVT_BASE + 0x414;
const SYSTIM_TIME250N: u32 = SYSTIM_BASE + 0x100;
const LRFDDBELL_CLKCTL: u32 = LRFDDBELL_BASE + 0x004;
const LRFDPBE_ENABLE: u32 = LRFDPBE_BASE;
const LRFDPBE_INIT: u32 = LRFDPBE_BASE + 0x008;
const LRFDPBE_PDREQ: u32 = LRFDPBE_BASE + 0x02C;
const LRFDPBE_API: u32 = LRFDPBE_BASE + 0x030;
const LRFDPBE_FCMD: u32 = LRFDPBE_BASE + 0x1A0;
const LRFDMDM_ENABLE: u32 = LRFDMDM_BASE;
const LRFDMDM_INIT: u32 = LRFDMDM_BASE + 0x008;
const LRFDMDM_PDREQ: u32 = LRFDMDM_BASE + 0x058;
const LRFDRFE_ENABLE: u32 = LRFDRFE_BASE;
const LRFDRFE_INIT: u32 = LRFDRFE_BASE + 0x008;
const LRFDRFE_PDREQ: u32 = LRFDRFE_BASE + 0x00C;
const LRFDRFE_RSSI: u32 = LRFDRFE_BASE + 0x21C;

const PBE_MSGBOX: u32 = BUF_RAM_BASE + 0x004;
const PBE_FIFOCMDADD: u32 = BUF_RAM_BASE + 0x008;

const CLKCTL_LRFD: u32 = 1 << 1;
const RADIO_CLOCKS: u32 =
    (1 << 10) | (1 << 9) | (1 << 8) | (1 << 7) | (1 << 6) | (1 << 3) | (1 << 2) | (1 << 1);
const CLOCK_READY_SPINS: usize = 1_000_000;
const TOPSM_READY_TICKS: u32 = 4_000;
const SYNTH_DIVIDER_READY_TICKS: u32 = 400;
const REFSYS_SETTLE_TICKS: u32 = 120;

const PBE_OP_STOP: u32 = 0x01;
const PBE_FCMD_ADDRESS: u16 = ((LRFDPBE_FCMD & 0x0FFF) >> 2) as u16;
const INVALID_RSSI: u32 = 127;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum HardwareError {
    ClockTimeout,
    ImageTooLarge,
    FactoryTrimUnavailable,
    SynthConfigInvalid,
    SynthDividerTimeout,
    TopsmTimeout,
    HfxtTimeout,
}

#[inline(always)]
fn read8(address: u32) -> u8 {
    unsafe { core::ptr::read_volatile(address as *const u8) }
}

#[inline(always)]
pub(super) fn read16(address: u32) -> u16 {
    unsafe { core::ptr::read_volatile(address as *const u16) }
}

#[inline(always)]
pub(super) fn read32(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline(always)]
pub(super) fn write32(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[inline(always)]
pub(super) fn write16(address: u32, value: u16) {
    unsafe { core::ptr::write_volatile(address as *mut u16, value) }
}

#[inline(always)]
fn update16(address: u32, mask: u16, value: u16) {
    write16(address, (read16(address) & !mask) | (value & mask));
}

#[inline(always)]
fn update32(address: u32, mask: u32, value: u32) {
    write32(address, (read32(address) & !mask) | (value & mask));
}

#[inline(always)]
fn or32(address: u32, value: u32) {
    write32(address, read32(address) | value);
}

/// Apply the early HFXT workaround from TI's `SetupTrimDevice()`.
///
/// TI's C startup calls this before data/BSS initialization. Rust firmware
/// should call it as early as possible; calling it again during radio init is
/// harmless and protects applications with a custom reset path.
pub(crate) fn setup_device_trim() {
    const APP_TRIMS_REVISION: u32 = FCFG_BASE + 0x330;
    const TRIM_STATE_REV_4: u32 = FCFG_BASE + 0x32F;
    const TRIM_STATE_LEGACY: u32 = FCFG_BASE + 0x3AF;

    let revision = read8(APP_TRIMS_REVISION);
    let trim_state = read8(if revision >= 4 {
        TRIM_STATE_REV_4
    } else {
        TRIM_STATE_LEGACY
    });

    if trim_state > 0xFC {
        let initial = 0x23 | (0x23 << 6) | (0x8 << 12) | (0x7F << 16) | (0x12 << 23);
        let target = 0x23 | (0x23 << 6) | (0x3 << 12) | (0x7F << 16) | (0x12 << 23) | (1 << 30);
        write32(CKMD_HFXTINIT, initial);
        write32(CKMD_HFXTTARG, target);
    }
}

/// Start the 48 MHz crystal (HFXT), mirroring TI's `Power_init()` +
/// `PowerCC23X0_startHFXT()`.
///
/// The LRFD synthesizer uses HFXT as its reference. After reset the device
/// runs from HFOSC with `HFXTCTL.EN = 0`; TI applications get HFXT started by
/// the Power driver before RCL is ever used. Without it every PBE operation
/// ends with `ENDCAUSE = ERR_SYNTH (0xFB)` because the PLL cannot lock.
pub(crate) fn start_hfxt() -> Result<(), HardwareError> {
    const CKMD_RIS: u32 = CKMD_BASE + 0x048;
    const CKMD_ICLR: u32 = CKMD_BASE + 0x054;
    const CKMD_HFXTCTL: u32 = CKMD_BASE + 0x084;
    const CKMD_AMPADCCTL: u32 = CKMD_BASE + 0x0A0;
    const CKMD_LDOCTL: u32 = CKMD_BASE + 0x0A8;
    const CKMD_HFXTSTAT: u32 = CKMD_BASE + 0x0E4;
    const CKMD_AMPCFG1: u32 = CKMD_BASE + 0x128;

    const HFXTCTL_EN: u32 = 0x0000_0001;
    const LDOCTL_SWOVR: u32 = 0x8000_0000;
    const LDOCTL_HFXTLVLEN: u32 = 0x0000_0010;
    const LDOCTL_STARTCTL: u32 = 0x0000_0008;
    const LDOCTL_START: u32 = 0x0000_0004;
    const LDOCTL_EN: u32 = 0x0000_0001;
    const AMPADCCTL_SWOVR: u32 = 0x8000_0000;
    const AMPADCCTL_PEAKDETEN: u32 = 0x0002_0000;
    const AMPADCCTL_ADCEN: u32 = 0x0001_0000;
    const AMPADCCTL_SARSTRT: u32 = 0x0000_0001;
    const AMPCFG1_INTERVAL_M: u32 = 0x0000_0FFF;
    const HFXTTARG_IREF_M: u32 = 0x0000_F000;
    const HFXT_TARGET_IREF_MAX: u32 = 8;
    const RIS_ADCBIASUPD: u32 = 0x0000_0040;
    const RIS_AMPSETTLED: u32 = 0x0000_8000;
    const HFXTSTAT_GOOD: u32 = 0x0000_0001;
    const HFTRACKCTL_EN: u32 = 0x8000_0000;
    const HFTRACKCTL_REFCLK_M: u32 = 0x0C00_0000;

    /// Generous bound; TI's typical HFXT startup is a few hundred µs.
    const HFXT_SETTLE_TICKS: u32 = 4 * 20_000; // 20 ms
    const ADC_BIAS_TICKS: u32 = 4 * 1_000; // 1 ms

    if read32(CKMD_HFXTCTL) & HFXTCTL_EN == 0 {
        // Software amplitude control (as TI's Power driver) and max IREF.
        write32(CKMD_AMPCFG1, read32(CKMD_AMPCFG1) & !AMPCFG1_INTERVAL_M);
        update32(CKMD_HFXTTARG, HFXTTARG_IREF_M, HFXT_TARGET_IREF_MAX << 12);

        write32(
            CKMD_LDOCTL,
            LDOCTL_SWOVR | LDOCTL_STARTCTL | LDOCTL_START | LDOCTL_EN,
        );
        wait_ticks(66 * 4)?;
        write32(CKMD_LDOCTL, LDOCTL_SWOVR | LDOCTL_HFXTLVLEN | LDOCTL_EN);

        write32(
            CKMD_AMPADCCTL,
            AMPADCCTL_SWOVR | AMPADCCTL_PEAKDETEN | AMPADCCTL_ADCEN,
        );
        wait_ticks(6 * 4)?;
        write32(CKMD_ICLR, RIS_ADCBIASUPD);
        write32(CKMD_AMPADCCTL, read32(CKMD_AMPADCCTL) | AMPADCCTL_SARSTRT);
        write32(CKMD_AMPADCCTL, read32(CKMD_AMPADCCTL) & !AMPADCCTL_SARSTRT);
        if !wait_until(ADC_BIAS_TICKS, || read32(CKMD_RIS) & RIS_ADCBIASUPD != 0) {
            return Err(HardwareError::HfxtTimeout);
        }
        write32(
            CKMD_AMPADCCTL,
            read32(CKMD_AMPADCCTL) & !(AMPADCCTL_SWOVR | AMPADCCTL_ADCEN),
        );

        write32(CKMD_HFXTCTL, read32(CKMD_HFXTCTL) | HFXTCTL_EN);
    }

    if !wait_until(HFXT_SETTLE_TICKS, || {
        read32(CKMD_RIS) & RIS_AMPSETTLED != 0 && read32(CKMD_HFXTSTAT) & HFXTSTAT_GOOD != 0
    }) {
        return Err(HardwareError::HfxtTimeout);
    }

    // Track HFOSC (and LFOSC calibration) against HFXT, as TI does.
    write32(
        CKMD_HFTRACKCTL,
        (read32(CKMD_HFTRACKCTL) & !HFTRACKCTL_REFCLK_M) | HFTRACKCTL_EN,
    );
    Ok(())
}

/// Enable the HFXT high-performance clock buffer that feeds the LRFD
/// synthesizer reference (TI `hal_enable_clk_buffer()`, called from
/// `RCL_open()`). Without it the RFE synth calibration never completes and
/// the PBE ends every TX/RX with `ENDCAUSE = ERR_SYNTH (0xFB)`.
pub(crate) fn enable_hfxt_clock_buffer() {
    const CKMD_HFXTCTL: u32 = CKMD_BASE + 0x084;
    const HFXTCTL_HPBUFEN: u32 = 0x0000_0002;
    or32(CKMD_HFXTCTL, HFXTCTL_HPBUFEN);
}

pub(crate) fn enable_radio_clocks() -> Result<(), HardwareError> {
    write32(CLKCTL_CLKENSET0, CLKCTL_LRFD);
    for _ in 0..CLOCK_READY_SPINS {
        if read32(CLKCTL_CLKCFG0) & CLKCTL_LRFD == CLKCTL_LRFD {
            write32(LRFDDBELL_CLKCTL, RADIO_CLOCKS);
            return Ok(());
        }
        spin_loop();
    }
    Err(HardwareError::ClockTimeout)
}

pub(crate) fn disable_radio_clocks() {
    write32(LRFDDBELL_CLKCTL, 0);
    write32(CLKCTL_CLKENCLR0, CLKCTL_LRFD);
}

pub(crate) fn route_scheduler_interrupt() {
    write32(EVTSVT_CPUIRQ4SEL, 0x0E);
}

pub(crate) fn load_firmware(pbe: &[u32], mce: &[u32], rfe: &[u32]) -> Result<(), HardwareError> {
    load_image(PBE_RAM_BASE, pbe)?;
    load_image(MCE_RAM_BASE, mce)?;
    load_image(RFE_RAM_BASE, rfe)?;
    Ok(())
}

pub(crate) fn apply_phy_configuration(writes: &[RegisterWrite]) {
    for write in writes {
        match write.width {
            RegisterWidth::U16 => write16(write.address, write.value as u16),
            RegisterWidth::U32 => write32(write.address, write.value),
        }
    }
}

/// Apply the per-die radio trims stored in FCFG.
///
/// This is the nominal-temperature path from TI's `LRF_applyTrim()`. Dynamic
/// temperature compensation can update the same fields later without changing
/// the initial radio bring-up sequence.
pub(crate) fn apply_factory_radio_trim() -> Result<(), HardwareError> {
    const TRIM_REVISION: u32 = 0;
    const TRIM_PA0: u32 = 4;
    const TRIM_ATSTREFH: u32 = 6;
    const TRIM_LNA: u32 = 8;
    const TRIM_IFAMPRFLDO: u32 = 10;
    const TRIM_DIVLDO: u32 = 12;
    const TRIM_TDCLDO: u32 = 14;
    const TRIM_DCOLDO0: u32 = 16;
    const TRIM_IFADCALDO: u32 = 18;
    const TRIM_IFADCDLDO: u32 = 20;
    const TRIM_DCO: u32 = 22;
    const TRIM_VARIANT_NORMAL: u32 = 24;
    const TRIM_VARIANT_DITHER: u32 = 32;
    const TRIM_EXT0: u32 = 44;
    const TRIM_RSSI_OFFSET: u32 = 48;
    const TRIM_DEMIQMC0: u32 = 50;
    const TRIM_IFAMPRFLDO_NORMAL: u32 = 54;

    const RFE_RSSIOFFSET: u32 = LRFDRFE_BASE + 0x088;
    const RFE_SPARE0: u32 = LRFDRFE_BASE + 0x098;
    const RFE_SPARE1: u32 = LRFDRFE_BASE + 0x09C;
    const RFE_LNA: u32 = LRFDRFE_BASE + 0x0B0;
    const RFE_IFAMPRFLDO: u32 = LRFDRFE_BASE + 0x0B4;
    const RFE_PA0: u32 = LRFDRFE_BASE + 0x0B8;
    const RFE_IFADC0: u32 = LRFDRFE_BASE + 0x0C4;
    const RFE_IFADC1: u32 = LRFDRFE_BASE + 0x0C8;
    const RFE_IFADCLF: u32 = LRFDRFE_BASE + 0x0CC;
    const RFE_IFADCQUANT: u32 = LRFDRFE_BASE + 0x0D0;
    const RFE_IFADCALDO: u32 = LRFDRFE_BASE + 0x0D4;
    const RFE_IFADCDLDO: u32 = LRFDRFE_BASE + 0x0D8;
    const RFE_ATSTREFH: u32 = LRFDRFE_BASE + 0x0E4;
    const RFE_DCO: u32 = LRFDRFE_BASE + 0x0E8;
    const RFE_TDCLDO: u32 = LRFDRFE_BASE + 0x0F4;
    const RFE_DCOLDO0: u32 = LRFDRFE_BASE + 0x0F8;
    const MDM_DEMIQMC0: u32 = LRFDMDM_BASE + 0x0F0;

    const RFE_RAM_RTRIMOFF: u32 = RFE_RAM_BASE + 0x820;
    const RFE_RAM_RTRIMMIN: u32 = RFE_RAM_BASE + 0x822;
    const RFE_RAM_DIVLDOI: u32 = RFE_RAM_BASE + 0x828;
    const RFE_RAM_DIVLDOF: u32 = RFE_RAM_BASE + 0x82A;
    const RFE_RAM_DIVLDOIOFF: u32 = RFE_RAM_BASE + 0x82C;
    const RFE_RAM_IFAMPRFLDO_DEFAULT: u32 = RFE_RAM_BASE + 0x836;
    const RFE_RAM_PHYRSSIOFFSET: u32 = RFE_RAM_BASE + 0x842;
    const RFE_RAM_SPARE0_SHADOW: u32 = RFE_RAM_BASE + 0x844;
    const RFE_RAM_SPARE1_SHADOW: u32 = RFE_RAM_BASE + 0x846;
    const RFE_RAM_AGCINFO: u32 = RFE_RAM_BASE + 0x848;

    const IFAMPRFLDO_TRIM_MASK: u32 = 0x0000_FE00;
    const IFADC0_DITHER_MASK: u32 = 0x0000_7C00;
    const DIVLDO_VOUTTRIM_MASK: u16 = 0x7F00;
    const TDCLDO_VOUTTRIM_MASK: u32 = 0x0000_7F00;
    const DCO_TAILRESTRIM_MASK: u32 = 0x0000_0078;
    const DEFAULT_RTRIM_MAX: u32 = 12;

    let revision = trim8(TRIM_REVISION);
    if revision == 0 || revision == u8::MAX {
        return Err(HardwareError::FactoryTrimUnavailable);
    }

    or32(RFE_PA0, trim16(TRIM_PA0) as u32);
    or32(RFE_ATSTREFH, trim16(TRIM_ATSTREFH) as u32);
    or32(RFE_LNA, trim16(TRIM_LNA) as u32);
    or32(RFE_IFAMPRFLDO, trim16(TRIM_IFAMPRFLDO) as u32);
    or32(RFE_IFADCALDO, trim16(TRIM_IFADCALDO) as u32);
    or32(RFE_IFADCDLDO, trim16(TRIM_IFADCDLDO) as u32);

    let normal_ifadc_quant = trim16(TRIM_VARIANT_NORMAL);
    let normal_ifadc0 = trim16(TRIM_VARIANT_NORMAL + 2);
    let normal_ifadc1 = trim16(TRIM_VARIANT_NORMAL + 4);
    let normal_ifadclf = trim16(TRIM_VARIANT_NORMAL + 6);
    or32(RFE_IFADCQUANT, normal_ifadc_quant as u32);
    or32(RFE_IFADC0, normal_ifadc0 as u32);
    or32(RFE_IFADC1, normal_ifadc1 as u32);
    or32(RFE_IFADCLF, normal_ifadclf as u32);

    if revision >= 4 {
        let dither = trim16(TRIM_VARIANT_DITHER + 2) as u32;
        update32(RFE_IFADC0, IFADC0_DITHER_MASK, dither);
        or32(RFE_IFAMPRFLDO, trim8(TRIM_IFAMPRFLDO_NORMAL) as u32);
    }

    write32(MDM_DEMIQMC0, trim16(TRIM_DEMIQMC0) as u32);
    write16(
        RFE_RAM_IFAMPRFLDO_DEFAULT,
        (read32(RFE_IFAMPRFLDO) & IFAMPRFLDO_TRIM_MASK) as u16,
    );

    or32(RFE_DCOLDO0, trim16(TRIM_DCOLDO0) as u32);

    let div_ldo = trim16(TRIM_DIVLDO) & DIVLDO_VOUTTRIM_MASK;
    update16(RFE_RAM_DIVLDOF, DIVLDO_VOUTTRIM_MASK, div_ldo);
    let div_ldo_decoded = ((div_ldo >> 8) ^ 0x40) as u32;
    let div_ldo_offset = (read16(RFE_RAM_DIVLDOIOFF) & 0x007F) as u32;
    let div_ldo_i = (div_ldo_decoded + div_ldo_offset).min(0x7F);
    update16(
        RFE_RAM_DIVLDOI,
        DIVLDO_VOUTTRIM_MASK,
        ((div_ldo_i ^ 0x40) << 8) as u16,
    );

    update32(RFE_TDCLDO, TDCLDO_VOUTTRIM_MASK, trim16(TRIM_TDCLDO) as u32);

    let mut rtrim = ((trim16(TRIM_DCO) as u32) & DCO_TAILRESTRIM_MASK) >> 3;
    if rtrim < DEFAULT_RTRIM_MAX {
        rtrim = (rtrim + (read16(RFE_RAM_RTRIMOFF) as u32 & 0x0F)).min(DEFAULT_RTRIM_MAX);
    }
    rtrim = rtrim.max(read16(RFE_RAM_RTRIMMIN) as u32 & 0x0F);
    update32(RFE_DCO, DCO_TAILRESTRIM_MASK, rtrim << 3);

    let mut rssi_offset = trim8(TRIM_RSSI_OFFSET) as i8 as i32;
    if revision == 4 && rssi_offset <= -4 {
        rssi_offset += 5;
    }
    rssi_offset += (read16(RFE_RAM_PHYRSSIOFFSET) & 0x00FF) as i32;
    write32(RFE_RSSIOFFSET, rssi_offset as u32);

    if revision >= 4 {
        let ext0 = trim32(TRIM_EXT0);
        let fast_agc = read16(RFE_RAM_AGCINFO) & 0x0001 == 0;
        if fast_agc {
            let threshold_offset = sign_extend_nibble((ext0 >> 20) as u8);
            let low_gain_offset = sign_extend_nibble((ext0 >> 24) as u8);
            let high_gain_offset = sign_extend_nibble((ext0 >> 28) as u8);
            let shadow = read16(RFE_RAM_SPARE0_SHADOW);
            let low_gain = ((shadow & 0x000F) as i32 + low_gain_offset).clamp(0, 0x0F);
            let high_gain = (((shadow >> 4) & 0x000F) as i32 + high_gain_offset).clamp(0, 0x0F);
            write32(
                RFE_SPARE0,
                ((shadow & !0x00FF) as u32) | (low_gain | (high_gain << 4)) as u32,
            );

            let spare1_shadow = read16(RFE_RAM_SPARE1_SHADOW);
            let threshold = ((spare1_shadow & 0x00FF) as i32 + threshold_offset).clamp(0, 0xFF);
            write32(
                RFE_SPARE1,
                ((spare1_shadow & !0x00FF) as u32) | threshold as u32,
            );
        } else {
            let magnitude_offset = sign_extend_nibble((ext0 >> 8) as u8);
            let spare1_shadow = read16(RFE_RAM_SPARE1_SHADOW);
            let magnitude = ((spare1_shadow & 0x00FF) as i32 + magnitude_offset).clamp(0, 0xFF);
            write32(
                RFE_SPARE1,
                ((spare1_shadow & !0x00FF) as u32) | magnitude as u32,
            );
        }
    } else {
        write32(RFE_SPARE1, read16(RFE_RAM_SPARE1_SHADOW) as u32);
    }

    Ok(())
}

pub(crate) fn finish_radio_setup() -> Result<(), HardwareError> {
    const RFE_RAM_GRANTPIN: u32 = RFE_RAM_BASE + 0x84A;

    write32(LRFDRFE_RSSI, INVALID_RSSI);
    write16(PBE_FIFOCMDADD, PBE_FCMD_ADDRESS);
    write16(RFE_RAM_GRANTPIN, 0x000F);
    apply_factory_radio_trim()
}

#[inline(always)]
fn trim8(offset: u32) -> u8 {
    read8(FCFG_APP_TRIMS + offset)
}

#[inline(always)]
fn trim16(offset: u32) -> u16 {
    read16(FCFG_APP_TRIMS + offset)
}

#[inline(always)]
fn trim32(offset: u32) -> u32 {
    read32(FCFG_APP_TRIMS + offset)
}

const fn sign_extend_nibble(value: u8) -> i32 {
    ((value & 0x0F) as i8).wrapping_shl(4).wrapping_shr(4) as i32
}

fn load_image(destination: u32, image: &[u32]) -> Result<(), HardwareError> {
    if image.len() > TOPSM_RAM_WORDS {
        return Err(HardwareError::ImageTooLarge);
    }

    for (index, word) in image.iter().copied().enumerate() {
        write32(destination + (index as u32 * 4), word);
    }
    Ok(())
}

pub(crate) fn enable_synth_refsys() -> Result<(), HardwareError> {
    const RFE32_ATSTREF: u32 = LRFDRFE32_BASE + 0x070;
    const BIAS_ENABLE: u32 = 0x0200_0000;

    let atstref = read32(RFE32_ATSTREF);
    if atstref & BIAS_ENABLE == 0 {
        write32(RFE32_ATSTREF, atstref | BIAS_ENABLE);
        wait_ticks(REFSYS_SETTLE_TICKS)?;
    }
    Ok(())
}

pub(crate) fn disable_synth_refsys() {
    const RFE32_ATSTREF: u32 = LRFDRFE32_BASE + 0x070;
    const BIAS_ENABLE: u32 = 0x0200_0000;

    write32(RFE32_ATSTREF, read32(RFE32_ATSTREF) & !BIAS_ENABLE);
}

pub(crate) fn program_frequency(frequency: u32) -> Result<(), HardwareError> {
    const RFE32_PRE1_PRE0: u32 = LRFDRFE32_BASE + 0x080;
    const RFE32_PRE3_PRE2: u32 = LRFDRFE32_BASE + 0x084;
    const RFE32_PLLM0: u32 = LRFDRFE32_BASE + 0x0BC;
    const RFE32_PLLM1: u32 = LRFDRFE32_BASE + 0x0C0;
    const RFE32_CALMMID_CALMCRS: u32 = LRFDRFE32_BASE + 0x0C4;
    const RFE32_DIVIDEND: u32 = LRFDRFE32_BASE + 0x118;
    const RFE32_DIVISOR: u32 = LRFDRFE32_BASE + 0x11C;
    const RFE32_QUOTIENT: u32 = LRFDRFE32_BASE + 0x120;
    const RFE_DIVSTA: u32 = LRFDRFE_BASE + 0x22C;
    const RFE_RAM_K5: u32 = RFE_RAM_BASE + 0x81A;
    const RFE_RAM_RXIF: u32 = RFE_RAM_BASE + 0x81C;
    const RFE_RAM_TXIF: u32 = RFE_RAM_BASE + 0x81E;
    const MDM_DEMMISC0: u32 = LRFDMDM_BASE + 0x0E0;
    const MDM_SPARE3: u32 = LRFDMDM_BASE + 0x130;

    if frequency == 0 {
        return Err(HardwareError::SynthConfigInvalid);
    }

    let compensated_frequency = scale_frequency_with_hfxt(frequency);
    let frequency_div_2_16 = (frequency + (1 << 15)) >> 16;
    if frequency_div_2_16 == 0 {
        return Err(HardwareError::SynthConfigInvalid);
    }

    write32(RFE32_DIVIDEND, 1 << 31);
    write32(RFE32_DIVISOR, frequency_div_2_16);
    write16(RFE_RAM_K5, frequency_div_2_16 as u16);

    let pre3_pre2 = read32(RFE32_PRE3_PRE2);
    let coarse_precal = (pre3_pre2 & 0x0000_0FC0) >> 6;
    let mid_precal = (pre3_pre2 & 0x001F_F000) >> 12;
    if coarse_precal == 0 || mid_precal == 0 {
        return Err(HardwareError::SynthConfigInvalid);
    }
    let cal_m_coarse = find_cal_m(frequency, coarse_precal);
    let cal_m_mid = if coarse_precal == mid_precal {
        cal_m_coarse
    } else {
        find_cal_m(frequency, mid_precal)
    };
    write32(RFE32_CALMMID_CALMCRS, cal_m_coarse | (cal_m_mid << 16));

    let pre1_pre0 = read32(RFE32_PRE1_PRE0);
    let pre_cal0 = pre1_pre0 & 0x3F;
    let pre_cal1 = (pre1_pre0 >> 8) & 0x3F;
    if pre_cal0 == 0 || pre_cal1 == 0 {
        return Err(HardwareError::SynthConfigInvalid);
    }

    let pll_m_base = program_pq(find_pll_m_base(frequency))?;
    let compensated_pll_m = if compensated_frequency == frequency {
        pll_m_base
    } else {
        find_pll_m_base(compensated_frequency)
    };
    write32(RFE32_PLLM0, (compensated_pll_m * pre_cal0) << 2);
    write32(RFE32_PLLM1, (compensated_pll_m * pre_cal1) << 2);

    if !wait_until(SYNTH_DIVIDER_READY_TICKS, || read32(RFE_DIVSTA) & 1 == 0) {
        return Err(HardwareError::SynthDividerTimeout);
    }
    let _inverse_synth_frequency = read32(RFE32_QUOTIENT);

    // The fixed IEEE PHY uses zero RX/TX offsets and zero intermediate
    // frequency, so Foff and CMIXN both remain zero.
    write16(RFE_RAM_RXIF, 0);
    write16(RFE_RAM_TXIF, 0);
    write32(MDM_DEMMISC0, 0);
    write32(MDM_SPARE3, 0);
    Ok(())
}

/// Program the PA through the RFE `SPARE5` shadow register, which the RFE
/// firmware applies at the next TX start (RCL `LRF_programTxPower`).
///
/// `rfe_value` is the 16-bit `LRF_TxPowerTable_Entry.value` encoding, see
/// [`super::config::TxPowerEntry::compensated_rfe_value`].
pub(crate) fn program_tx_power(rfe_value: u16) {
    const RFE_SPARE5: u32 = LRFDRFE_BASE + 0x0AC;
    write32(RFE_SPARE5, u32::from(rfe_value));
}

const PMUD_BASE: u32 = 0x4000_6000;
const PMUD_CTL: u32 = PMUD_BASE;
const PMUD_TEMP: u32 = PMUD_BASE + 0x030;
const PMUD_TEMPUPD: u32 = PMUD_BASE + 0x034;
const PMUD_CTL_CALC_EN: u32 = 0x2;
const PMUD_CTL_MEAS_EN: u32 = 0x1;
const PMUD_TEMPUPD_STA: u32 = 0x1;

/// Die temperature in whole degrees Celsius from the PMUD (BATMON), or
/// `None` while no measurement is available yet.
///
/// Starts the PMUD measurement the same way `BatMonSupportLPF3_init` does if
/// nothing else has. TI's `Temperature_getTemperature` additionally applies a
/// small FCFG-based linearity correction; it is not needed for the PA bias
/// compensation, whose coefficients are at most ~0.7 ib steps per degree.
pub(crate) fn die_temperature_c() -> Option<i32> {
    let control = read32(PMUD_CTL);
    if control & (PMUD_CTL_CALC_EN | PMUD_CTL_MEAS_EN) != (PMUD_CTL_CALC_EN | PMUD_CTL_MEAS_EN) {
        write32(PMUD_CTL, control | PMUD_CTL_CALC_EN | PMUD_CTL_MEAS_EN);
    }
    if read32(PMUD_TEMPUPD) & PMUD_TEMPUPD_STA == 0 {
        return None;
    }
    // TEMP[16:8] signed integer part, [7:6] fraction (ignored).
    let integer = ((read32(PMUD_TEMP) >> 8) & 0x1FF) as i32;
    Some(if integer & 0x100 != 0 {
        integer - 0x200
    } else {
        integer
    })
}

pub(crate) fn enable_radio_cores() {
    write16(PBE_MSGBOX, 0);

    write32(LRFDPBE_INIT, (1 << 2) | 1);
    write32(LRFDPBE_ENABLE, (1 << 2) | 1);
    write32(LRFDMDM_INIT, (1 << 1) | 1);
    write32(LRFDMDM_ENABLE, (1 << 1) | 1);
    write32(LRFDRFE_INIT, 1);
    write32(LRFDRFE_ENABLE, 1);
}

pub(crate) fn wait_for_topsm_ready() -> Result<(), HardwareError> {
    if wait_until(TOPSM_READY_TICKS, || read16(PBE_MSGBOX) != 0) {
        Ok(())
    } else {
        Err(HardwareError::TopsmTimeout)
    }
}

pub(crate) fn disable_radio_cores() {
    write32(LRFDPBE_PDREQ, 1);
    write32(LRFDPBE_ENABLE, 0);
    write32(LRFDPBE_PDREQ, 0);

    write32(LRFDMDM_PDREQ, 1);
    write32(LRFDMDM_ENABLE, 0);
    write32(LRFDMDM_PDREQ, 0);

    write32(LRFDRFE_PDREQ, 1);
    write32(LRFDRFE_ENABLE, 0);
    write32(LRFDRFE_PDREQ, 0);
}

fn find_pll_m_base(frequency: u32) -> u32 {
    const FXTAL_INV_HIGH: u32 = 0x02CB_D3F0;

    let mut pll_m_base = (FXTAL_INV_HIGH >> 16) * (frequency >> 16);
    let mut remainder = ((FXTAL_INV_HIGH >> 16) * (frequency & 0xFFFF)) >> 1;
    remainder += ((FXTAL_INV_HIGH & 0xFFFF) * (frequency >> 16)) >> 1;
    remainder += 1 << 14;
    remainder >>= 15;
    pll_m_base += remainder;
    (pll_m_base + 1) >> 1
}

fn find_cal_m(frequency: u32, predivider: u32) -> u32 {
    const FXTAL_INV_HIGH: u32 = 0x02CB_D3F0;

    let mut reference_inverse = (FXTAL_INV_HIGH >> 4) * predivider;
    reference_inverse = (reference_inverse + (1 << 15)) >> 16;

    let mut cal_m = reference_inverse * ((frequency + (1 << 14)) >> 15);
    cal_m = (cal_m + (1 << 15)) >> 16;
    cal_m
}

fn program_pq(pll_m_base: u32) -> Result<u32, HardwareError> {
    const MDM_BAUD: u32 = LRFDMDM_BASE + 0x0D4;
    const MDM_BAUDPRE: u32 = LRFDMDM_BASE + 0x0D8;
    const MDM_DEMMISC3: u32 = LRFDMDM_BASE + 0x0EC;
    const MDM32_DEMFRAC1_DEMFRAC0: u32 = LRFDMDM32_BASE + 0x0A8;
    const MDM32_DEMFRAC3_DEMFRAC2: u32 = LRFDMDM32_BASE + 0x0AC;

    let baud_pre = read32(MDM_BAUDPRE);
    let rate_word = (read32(MDM_BAUD) << 5) | ((baud_pre & 0x1F00) >> 8);
    let predivider = baud_pre & 0x00FF;
    let dem_misc3 = read32(MDM_DEMMISC3);
    let log2_bde1 = find_log2_bde1(dem_misc3);
    let bde2 = dem_misc3 & 0x001F;
    let log2_pdif_decim = (dem_misc3 & 0x0060) >> 5;
    if rate_word == 0 || predivider == 0 || bde2 == 0 {
        return Err(HardwareError::SynthConfigInvalid);
    }

    let mut left_shift_p = (log2_bde1 + log2_pdif_decim + 4) as i32;
    let mut dem_frac_p = rate_word
        .checked_mul(bde2)
        .ok_or(HardwareError::SynthConfigInvalid)?;
    if dem_frac_p as u64 > (1u64 << 32) / 9 {
        dem_frac_p >>= 1;
        left_shift_p -= 1;
    }
    dem_frac_p = dem_frac_p
        .checked_mul(9)
        .ok_or(HardwareError::SynthConfigInvalid)?;

    let mut dem_frac_q = ((pll_m_base + ((1 << 6) - 1)) >> 6)
        .checked_mul(predivider)
        .ok_or(HardwareError::SynthConfigInvalid)?;
    let zero_count = count_leading_zeros16((dem_frac_q >> 16) as u16);
    let pll_m_shift = 10i32 - zero_count as i32;

    let pll_m_base_rounded;
    if pll_m_shift <= 0 {
        pll_m_base_rounded = pll_m_base;
        dem_frac_q = pll_m_base
            .checked_mul(predivider)
            .ok_or(HardwareError::SynthConfigInvalid)?;
        let left_shift_q = (-pll_m_shift) as u32;
        left_shift_p += left_shift_q as i32;
        dem_frac_q = dem_frac_q
            .checked_shl(left_shift_q)
            .ok_or(HardwareError::SynthConfigInvalid)?;
    } else {
        let shift = pll_m_shift as u32;
        pll_m_base_rounded = ((pll_m_base + (1 << (shift - 1))) >> shift) << shift;
        dem_frac_q = (pll_m_base_rounded >> shift)
            .checked_mul(predivider)
            .ok_or(HardwareError::SynthConfigInvalid)?;
        left_shift_p -= pll_m_shift;
    }

    if left_shift_p >= 0 {
        dem_frac_p = dem_frac_p
            .checked_shl(left_shift_p as u32)
            .ok_or(HardwareError::SynthConfigInvalid)?;
    } else {
        dem_frac_p >>= (-left_shift_p) as u32;
    }
    if dem_frac_p >= dem_frac_q {
        return Err(HardwareError::SynthConfigInvalid);
    }

    write32(MDM32_DEMFRAC1_DEMFRAC0, dem_frac_p);
    write32(MDM32_DEMFRAC3_DEMFRAC2, dem_frac_q);
    Ok(pll_m_base_rounded)
}

const fn count_leading_zeros16(mut value: u16) -> u32 {
    let mut zeros = 0;
    if value >= 0x0100 {
        value >>= 8;
    } else {
        zeros += 8;
    }
    if value >= 0x10 {
        value >>= 4;
    } else {
        zeros += 4;
    }
    if value >= 0x04 {
        value >>= 2;
    } else {
        zeros += 2;
    }
    if value < 0x02 {
        zeros += 1;
    }
    zeros
}

const fn find_log2_bde1(dem_misc3: u32) -> u32 {
    if dem_misc3 & 0x1000 != 0 {
        0
    } else {
        (dem_misc3 & 0x0080) >> 7
    }
}

fn scale_frequency_with_hfxt(frequency: u32) -> u32 {
    const DEFAULT_RATIO: u32 = 0x0040_0000;
    let ratio = read32(CKMD_HFTRACKCTL) & 0x03FF_FFFF;
    if ratio == 0 || ratio == DEFAULT_RATIO {
        return frequency;
    }

    let frequency_high = frequency >> 16;
    let frequency_low = frequency & 0xFFFF;
    let ratio_high = ratio >> 16;
    let ratio_low = ratio & 0xFFFF;
    ((ratio_low * frequency_high
        + ratio_high * frequency_low
        + ((ratio_low * frequency_low) >> 16))
        >> 6)
        + ((ratio_high * frequency_high) << 10)
}

fn wait_ticks(ticks: u32) -> Result<(), HardwareError> {
    let start = read32(SYSTIM_TIME250N);
    for _ in 0..CLOCK_READY_SPINS {
        if read32(SYSTIM_TIME250N).wrapping_sub(start) >= ticks {
            return Ok(());
        }
        spin_loop();
    }
    Err(HardwareError::ClockTimeout)
}

fn wait_until<F>(timeout_ticks: u32, mut ready: F) -> bool
where
    F: FnMut() -> bool,
{
    let start = read32(SYSTIM_TIME250N);
    for _ in 0..CLOCK_READY_SPINS {
        if ready() {
            return true;
        }
        if read32(SYSTIM_TIME250N).wrapping_sub(start) >= timeout_ticks {
            return false;
        }
        spin_loop();
    }
    false
}

pub(crate) fn hard_stop() {
    write32(LRFDPBE_API, PBE_OP_STOP);
}

/// SYSTIM channel 2 is hard-wired to the LRFD `SYSTIM0` event.
const SYSTIM_CH2CFG: u32 = SYSTIM_BASE + 0x114;
const SYSTIM_CH2CC: u32 = SYSTIM_BASE + 0x128;
const LRFDDBELL_ICLR0: u32 = LRFDDBELL_BASE + 0x054;
const LRFDDBELL_SYSTIM0: u32 = 1 << 29;
/// `RCL_SCHEDULER_TRIG_NOW_DELAY`: 50 us in 250 ns SYSTIM ticks.
const PBE_START_TRIGGER_DELAY_TICKS: u32 = 200;

/// Arm the PBE start trigger, like RCL's `hal_setup_start_time()`.
///
/// Every IEEE PBE operation is issued with `OPCFG.START = SYNC`, so after
/// the API command the PBE waits for the LRFD `SYSTIM0` event (SYSTIM
/// channel 2 compare) before it starts TX or RX. RCL always programs that
/// compare, even for "start now" commands (current time + 50 us). Without
/// it the PBE waits forever and no `OPDONE`/`OPERROR` is ever raised.
pub(crate) fn arm_pbe_start_trigger() {
    write32(LRFDDBELL_ICLR0, LRFDDBELL_SYSTIM0);
    let start = read32(SYSTIM_TIME250N).wrapping_add(PBE_START_TRIGGER_DELAY_TICKS);
    // A write to CHnCC arms the channel in compare mode.
    write32(SYSTIM_CH2CC, start);
}

/// Disarm the PBE start/stop trigger, like RCL's `hal_cancel_lrfd_systim0()`.
pub(crate) fn cancel_pbe_start_trigger() {
    write32(SYSTIM_CH2CC, 0);
    write32(SYSTIM_CH2CFG, 1); // MODE = CAPT
    write32(SYSTIM_CH2CFG, 0); // MODE = DIS
    write32(LRFDDBELL_ICLR0, LRFDDBELL_SYSTIM0);
}

pub(crate) fn read_rssi() -> i8 {
    (read32(LRFDRFE_RSSI) & 0xFF) as u8 as i8
}

pub(super) fn timer_ticks() -> u32 {
    read32(SYSTIM_TIME250N)
}

pub(crate) const fn channel_frequency(channel: u8) -> Option<u32> {
    if channel >= 11 && channel <= 26 {
        Some(2_405_000_000 + ((channel - 11) as u32 * 5_000_000))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_ieee_channels_to_frequency() {
        assert_eq!(channel_frequency(11), Some(2_405_000_000));
        assert_eq!(channel_frequency(20), Some(2_450_000_000));
        assert_eq!(channel_frequency(26), Some(2_480_000_000));
        assert_eq!(channel_frequency(10), None);
        assert_eq!(channel_frequency(27), None);
    }

    #[test]
    fn enables_all_rcl_radio_clocks_without_bridge() {
        assert_eq!(RADIO_CLOCKS, 0x07CE);
    }

    #[test]
    fn matches_ti_synth_reference_vectors() {
        assert_eq!(find_pll_m_base(2_405_000_000), 0x00C8_6AAB);
        assert_eq!(find_pll_m_base(2_450_000_000), 0x00CC_2AAB);
        assert_eq!(find_pll_m_base(2_480_000_000), 0x00CE_AAAB);
        assert_eq!(find_cal_m(2_405_000_000, 24), 0x04B3);
        assert_eq!(find_cal_m(2_405_000_000, 48), 0x0964);
    }

    #[test]
    fn decodes_signed_trim_nibbles() {
        assert_eq!(sign_extend_nibble(0x00), 0);
        assert_eq!(sign_extend_nibble(0x07), 7);
        assert_eq!(sign_extend_nibble(0x08), -8);
        assert_eq!(sign_extend_nibble(0x0F), -1);
    }

    #[test]
    fn matches_ti_leading_zero_helper() {
        assert_eq!(count_leading_zeros16(0), 15);
        assert_eq!(count_leading_zeros16(1), 15);
        assert_eq!(count_leading_zeros16(2), 14);
        assert_eq!(count_leading_zeros16(0x8000), 0);
    }
}
