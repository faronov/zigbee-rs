//! IADC0 single conversions of the AVDD supply on EFR32xG22.
//!
//! The register map, field layout and default configuration follow GSDK
//! 4.5.0 `efr32mg22_iadc.h` and `em_iadc.c` (`IADC_init`, `IADC_initSingle`,
//! `IADC_INIT_DEFAULT`, `IADC_CONFIG_DEFAULT`, `IADC_INITSINGLE_DEFAULT`):
//!
//! - IADC clock FSRCO (20 MHz), `HSCLKRATE` 0, `TIMEBASE` 19 (1 µs),
//!   ADC_CLK prescale 1 (10 MHz, the normal-mode maximum);
//! - configuration 0: normal mode, 2× high-speed oversampling, analog
//!   gain 1×, internal 1.21 V band-gap reference, no digital averaging;
//! - `SCALE0` gain/offset calibration computed from the factory `DEVINFO`
//!   words exactly as `IADC_init` does for that configuration (integer math
//!   here, float in the GSDK);
//! - single input AVDD/4 (`PORTPOS` SUPPLY, pin 0) against GND, 12-bit
//!   right-aligned result.
//!
//! A conversion takes a few microseconds. Every hardware wait is bounded and
//! reports [`IadcError::Timeout`], and the IADC is disabled again before a
//! read returns, so it draws nothing while the device sleeps. Each read
//! re-applies the whole configuration, so a read after EM2 does not depend
//! on retained IADC registers.
//!
//! xG21 is not covered: its IADC0 sits at another address and its band-gap
//! reference depends on the chip revision.

use crate::chip::BusClock;

/// Errors reported by the IADC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IadcError {
    /// A bounded hardware wait expired.
    Timeout,
}

/// IADC0 secure-alias base (`IADC0_S_BASE`).
const IADC0_BASE: u32 = 0x4A00_4000;

/// IADC register offsets (`IADC_TypeDef`, configuration 0 only).
pub(crate) mod reg {
    pub const EN: u32 = 0x04;
    pub const CTRL: u32 = 0x08;
    pub const CMD: u32 = 0x0C;
    pub const STATUS: u32 = 0x14;
    pub const CFG0: u32 = 0x48;
    pub const SCALE0: u32 = 0x50;
    pub const SCHED0: u32 = 0x54;
    pub const SINGLEFIFOCFG: u32 = 0x70;
    pub const SINGLEFIFODATA: u32 = 0x74;
    pub const SINGLEFIFOSTAT: u32 = 0x78;
    pub const SINGLE: u32 = 0x98;
}

const EN_EN: u32 = 1 << 0;

const CMD_SINGLESTART: u32 = 1 << 0;
const CMD_SINGLESTOP: u32 = 1 << 1;

const STATUS_SINGLEFIFODV: u32 = 1 << 8;
const STATUS_SINGLEWRITEPENDING: u32 = 1 << 20;
const STATUS_SYNCBUSY: u32 = 1 << 24;

const SINGLEFIFOSTAT_FIFOREADCNT_MASK: u32 = 0x7;

const CTRL_TIMEBASE_SHIFT: u32 = 16;
const CTRL_HSCLKRATE_SHIFT: u32 = 28;

/// `CFG` fields `IADC_init` rewrites; other bits keep their reset value.
const CFG_ADCMODE_MASK: u32 = 0x3;
const CFG_OSRHS_MASK: u32 = 0x1C;
const CFG_ANALOGGAIN_SHIFT: u32 = 12;
const CFG_ANALOGGAIN_MASK: u32 = 0x7000;
const CFG_REFSEL_MASK: u32 = 0x7_0000;
const CFG_DIGAVG_MASK: u32 = 0xE0_0000;
const CFG_TWOSCOMPL_MASK: u32 = 0x3000_0000;
const CFG_REWRITTEN_MASK: u32 = CFG_ADCMODE_MASK
    | CFG_OSRHS_MASK
    | CFG_ANALOGGAIN_MASK
    | CFG_REFSEL_MASK
    | CFG_DIGAVG_MASK
    | CFG_TWOSCOMPL_MASK;
/// Normal mode (0), OSR 2× (0), gain 1× (`ANAGAIN1` = 2), VBGR (0),
/// one-sample average (0), automatic two's complement (0).
const CFG_FIELDS: u32 = 2 << CFG_ANALOGGAIN_SHIFT;

const SCALE_GAIN13LSB_SHIFT: u32 = 18;
const SCALE_GAIN3MSB: u32 = 1 << 31;
const SCALE_OFFSET_MASK: u32 = 0x3_FFFF;

const SINGLE_PINPOS_SHIFT: u32 = 8;
const SINGLE_PORTPOS_SHIFT: u32 = 12;
const SINGLE_PORTPOS_SUPPLY: u32 = 1;
/// Positive input AVDD/4 (`iadcPosInputAvdd`), negative input GND
/// (`iadcNegInputGnd` = 0), configuration 0, no window compare.
const SINGLE_AVDD: u32 =
    (SINGLE_PORTPOS_SUPPLY << SINGLE_PORTPOS_SHIFT) | (0 << SINGLE_PINPOS_SHIFT);

/// `SINGLEFIFOCFG`: right-aligned 12-bit, no ID, data valid after one
/// entry (`DVL` VALID1), no DMA wake-up.
const SINGLEFIFOCFG_RIGHT12_DVL1: u32 = 0;

/// FSRCO, the IADC clock selected here.
const FSRCO_HZ: u32 = 20_000_000;
/// `IADC_calcTimebase`: ADC clock cycles per microsecond, minus one.
const TIMEBASE: u32 = FSRCO_HZ.div_ceil(1_000_000) - 1;
/// `IADC_calcSrcClkPrescale`: FSRCO is already below the 40 MHz limit.
const HSCLKRATE: u32 = 0;
/// `IADC_calcAdcClkPrescale` for the 10 MHz normal-mode analog clock.
const ADC_CLK_PRESCALE: u32 = FSRCO_HZ.div_ceil(10_000_000) - 1;

/// xG22 (Series 2 config 2) band-gap reference used by `IADC_init`.
pub const VBGR_MILLIVOLTS: u32 = 1_210;
/// 12-bit full scale.
const FULL_SCALE: u32 = 4_095;
/// AVDD is measured through the internal divide-by-four.
const AVDD_DIVIDER: u32 = 4;

/// Bound on every polled hardware wait (a conversion completes in a few
/// microseconds; this is several milliseconds at 38.4 MHz).
const MAX_POLLS: u32 = 100_000;

/// Factory calibration words in `DEVINFO` (base 0x0FE0_8000).
const DEVINFO_IADC0GAIN0: u32 = 0x0FE0_8180;
const DEVINFO_IADC0NORMALOFFSETCAL0: u32 = 0x0FE0_818C;

/// Exclusive ownership of IADC0.
pub struct Iadc0Token {
    _private: (),
}

impl Iadc0Token {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    /// Use IADC0 to measure the AVDD supply.
    pub fn into_supply_monitor(self) -> SupplyMonitor {
        SupplyMonitor { _token: self }
    }
}

/// Blocking AVDD measurements on IADC0.
pub struct SupplyMonitor {
    _token: Iadc0Token,
}

impl SupplyMonitor {
    /// Measure AVDD in millivolts (one calibrated 12-bit conversion).
    pub fn avdd_millivolts(&mut self) -> Result<u16, IadcError> {
        self.read_raw().map(avdd_millivolts_from_raw)
    }

    /// One raw 12-bit conversion of AVDD/4 against the 1.21 V reference.
    pub fn read_raw(&mut self) -> Result<u16, IadcError> {
        crate::clock::enable_bus_clock(BusClock::IADC0);
        crate::clock::select_iadc_clock_fsrco();
        // SAFETY: this monitor exclusively owns IADC0 (`Iadc0Token`).
        unsafe {
            disable()?;
            write(reg::CTRL, ctrl_value());
            let cfg = read(reg::CFG0);
            write(reg::CFG0, (cfg & !CFG_REWRITTEN_MASK) | CFG_FIELDS);
            write(
                reg::SCALE0,
                scale_from_calibration(
                    read_abs(DEVINFO_IADC0GAIN0),
                    read_abs(DEVINFO_IADC0NORMALOFFSETCAL0),
                ),
            );
            write(reg::SCHED0, ADC_CLK_PRESCALE);
            write(reg::SINGLEFIFOCFG, SINGLEFIFOCFG_RIGHT12_DVL1);
            write(reg::EN, EN_EN);
            // SINGLE is a WSYNC register: written while enabled.
            write(reg::SINGLE, SINGLE_AVDD);
            let result = wait_clear(STATUS_SINGLEWRITEPENDING)
                .and_then(|()| drain_fifo())
                .and_then(|()| convert());
            if result.is_err() {
                write(reg::CMD, CMD_SINGLESTOP);
            }
            let disabled = disable();
            let raw = result?;
            disabled?;
            Ok(raw)
        }
    }
}

/// `IADC_CTRL` for the FSRCO clock with normal warm-up (power down after
/// each conversion).
const fn ctrl_value() -> u32 {
    (TIMEBASE << CTRL_TIMEBASE_SHIFT) | (HSCLKRATE << CTRL_HSCLKRATE_SHIFT)
}

/// AVDD in millivolts for a raw 12-bit AVDD/4 conversion.
pub const fn avdd_millivolts_from_raw(raw: u16) -> u16 {
    let raw = if raw as u32 > FULL_SCALE {
        FULL_SCALE
    } else {
        raw as u32
    };
    ((raw * AVDD_DIVIDER * VBGR_MILLIVOLTS + FULL_SCALE / 2) / FULL_SCALE) as u16
}

/// `IADC_SCALE` for configuration 0 (normal mode, OSR 2×, gain 1×, VBGR)
/// from `DEVINFO.IADC0GAIN0` and `DEVINFO.IADC0NORMALOFFSETCAL0`, following
/// `IADC_init` (GSDK 4.5.0 `em_iadc.c`) with exact rational arithmetic in
/// place of `float`:
///
/// ```text
/// offset = ANA1NORM * (1.25 / 1.21)
/// offset = offset * 4 + 640 * (256 / 2)
/// if gain bits != 1.0: offset = gain / 32768 * (offset + 524288) - 524288
/// SCALE  = GAIN3MSB | GAIN13LSB << 18 | round(-offset) (18-bit, saturated)
/// ```
pub const fn scale_from_calibration(gain0: u32, normal_offset_cal0: u32) -> u32 {
    let gain = gain0 & 0xFFFF;
    let gain_bits = gain & 0x9FFF;
    let mut scale = (gain_bits & 0x1FFF) << SCALE_GAIN13LSB_SHIFT;
    if gain_bits & 0x8000 != 0 {
        scale |= SCALE_GAIN3MSB;
    }
    let ana1 = (normal_offset_cal0 & 0xFFFF) as u16 as i16 as i64;
    // offset * 4 with the 1.25 V / 1.21 V reference correction, kept as a
    // fraction over `den`.
    let vref = VBGR_MILLIVOLTS as i64;
    let mut num = ana1 * 4 * 1_250 + 81_920 * vref;
    let mut den = vref;
    if scale != SCALE_GAIN3MSB {
        num = gain as i64 * (num + 524_288 * den) - 524_288 * den * 32_768;
        den *= 32_768;
    }
    let offset = round_half_away_from_zero(-num, den);
    let offset = if offset > 131_071 {
        131_071
    } else if offset < -131_072 {
        -131_072
    } else {
        offset
    };
    scale | (offset as i32 as u32 & SCALE_OFFSET_MASK)
}

/// `IADC_ROUND_D2I` for a positive denominator.
const fn round_half_away_from_zero(num: i64, den: i64) -> i64 {
    if num < 0 {
        -((-num + den / 2) / den)
    } else {
        (num + den / 2) / den
    }
}

unsafe fn disable() -> Result<(), IadcError> {
    unsafe {
        let synced = wait_clear(STATUS_SYNCBUSY);
        write(reg::EN, 0);
        synced
    }
}

unsafe fn convert() -> Result<u16, IadcError> {
    unsafe {
        write(reg::CMD, CMD_SINGLESTART);
        for _ in 0..MAX_POLLS {
            if read(reg::STATUS) & STATUS_SINGLEFIFODV != 0 {
                return Ok((read(reg::SINGLEFIFODATA) & 0xFFF) as u16);
            }
        }
        Err(IadcError::Timeout)
    }
}

/// Empty the single FIFO; also checks `SINGLEFIFOSTAT` (errata IADC_E305).
unsafe fn drain_fifo() -> Result<(), IadcError> {
    unsafe {
        for _ in 0..8 {
            if read(reg::STATUS) & STATUS_SINGLEFIFODV == 0
                && read(reg::SINGLEFIFOSTAT) & SINGLEFIFOSTAT_FIFOREADCNT_MASK == 0
            {
                return Ok(());
            }
            let _ = read(reg::SINGLEFIFODATA);
        }
        Err(IadcError::Timeout)
    }
}

unsafe fn wait_clear(mask: u32) -> Result<(), IadcError> {
    for _ in 0..MAX_POLLS {
        if unsafe { read(reg::STATUS) } & mask == 0 {
            return Ok(());
        }
    }
    Err(IadcError::Timeout)
}

#[inline(always)]
unsafe fn read(offset: u32) -> u32 {
    unsafe { read_abs(IADC0_BASE + offset) }
}

#[inline(always)]
unsafe fn read_abs(address: u32) -> u32 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::read_volatile(address as *const u32)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        let _ = address;
        0
    }
}

#[inline(always)]
unsafe fn write(offset: u32, value: u32) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::write_volatile((IADC0_BASE + offset) as *mut u32, value)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = (offset, value);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_map_matches_gsdk() {
        // offsetof(IADC_TypeDef, ...) from GSDK 4.5.0 efr32mg22_iadc.h.
        assert_eq!(IADC0_BASE, 0x4A00_4000);
        assert_eq!(
            [
                reg::EN,
                reg::CTRL,
                reg::CMD,
                reg::STATUS,
                reg::CFG0,
                reg::SCALE0,
                reg::SCHED0,
                reg::SINGLEFIFOCFG,
                reg::SINGLEFIFODATA,
                reg::SINGLEFIFOSTAT,
                reg::SINGLE,
            ],
            [
                0x04, 0x08, 0x0C, 0x14, 0x48, 0x50, 0x54, 0x70, 0x74, 0x78, 0x98
            ]
        );
        assert_eq!(
            BusClock::IADC0,
            BusClock {
                register: 0,
                bit: 10
            }
        );
        // offsetof(DEVINFO_TypeDef, IADC0GAIN0 / IADC0NORMALOFFSETCAL0).
        assert_eq!(DEVINFO_IADC0GAIN0 - 0x0FE0_8000, 0x180);
        assert_eq!(DEVINFO_IADC0NORMALOFFSETCAL0 - 0x0FE0_8000, 0x18C);
    }

    #[test]
    fn configuration_matches_emlib_defaults() {
        // TIMEBASE 19, HSCLKRATE 0, warm-up normal.
        assert_eq!(ctrl_value(), 0x0013_0000);
        assert_eq!(ADC_CLK_PRESCALE, 1);
        // Reset CFG 0x2060 keeps its untouched bits; gain 1× is the reset
        // value too.
        assert_eq!((0x2060 & !CFG_REWRITTEN_MASK) | CFG_FIELDS, 0x2060);
        // iadcPosInputAvdd << PINPOS_SHIFT, iadcNegInputGnd.
        assert_eq!(SINGLE_AVDD, 0x1000);
    }

    #[test]
    fn uncalibrated_scale_is_the_reset_value() {
        // Gain exactly 1.0 and zero offset trim: only the systematic
        // 640 * 128 offset remains, which is _IADC_SCALE_RESETVALUE.
        assert_eq!(scale_from_calibration(0x8000, 0), 0x8002_C000);
    }

    /// The GSDK computation in `f32`, as `IADC_init` performs it.
    fn gsdk_scale(gain0: u32, cal0: u32) -> u32 {
        let gain = (gain0 & 0xFFFF) as u16;
        let tmp = gain as u32 & 0x9FFF;
        let mut scale = tmp.wrapping_shl(18);
        if tmp & 0x8000 != 0 {
            scale |= 1 << 31;
        }
        let mut offset = 0.0f32;
        offset += (cal0 & 0xFFFF) as u16 as i16 as f32;
        offset *= 1.25f32 / (1210.0f32 / 1000.0f32);
        offset = offset * 4.0 + 640.0 * (256.0 / 2.0);
        if scale != 0x8000_0000 {
            offset = (gain as f32 / 32768.0) * (offset + 524_288.0) - 524_288.0;
        }
        let n = -offset;
        let i = if n < 0.0 { n - 0.5 } else { n + 0.5 } as i32;
        if i > 131_071 {
            scale | 0x1FFFF
        } else if i < -131_072 {
            scale | 0x20000
        } else {
            scale | (i as u32 & 0x3FFFF)
        }
    }

    #[test]
    fn calibration_matches_the_gsdk_float_computation() {
        for gain in [0x8000u32, 0x7F00, 0x80FF, 0x8123, 0x7E81, 0x9FFF, 0x6000] {
            for cal in [0u32, 1, 0xFFFF, 0x0040, 0xFFC0, 0x7FFF, 0x8000, 0x1234] {
                let ours = scale_from_calibration(gain, cal);
                let gsdk = gsdk_scale(gain, cal);
                assert_eq!(ours & !SCALE_OFFSET_MASK, gsdk & !SCALE_OFFSET_MASK);
                let delta =
                    ((ours & SCALE_OFFSET_MASK) as i32 - (gsdk & SCALE_OFFSET_MASK) as i32).abs();
                // f32 rounding of values near 2^19 may differ by one LSB.
                assert!(
                    delta <= 1,
                    "gain {gain:#x} cal {cal:#x}: {ours:#x} vs {gsdk:#x}"
                );
            }
        }
    }

    #[test]
    fn upper_devinfo_halves_are_ignored() {
        assert_eq!(
            scale_from_calibration(0xABCD_8000, 0x1234_0000),
            scale_from_calibration(0x8000, 0)
        );
    }

    #[test]
    fn avdd_conversion_uses_the_divider_and_band_gap() {
        assert_eq!(avdd_millivolts_from_raw(0), 0);
        // Full scale is 4 × 1.21 V.
        assert_eq!(avdd_millivolts_from_raw(4095), 4_840);
        // 3.30 V → 3.3 / 4 / 1.21 * 4095 = 2792.
        assert_eq!(avdd_millivolts_from_raw(2792), 3_300);
        // 3.00 V coin cell.
        assert_eq!(avdd_millivolts_from_raw(2538), 3_000);
        // Out-of-range data saturates at full scale.
        assert_eq!(avdd_millivolts_from_raw(0xFFFF), 4_840);
    }

    #[test]
    fn host_read_reports_timeout_instead_of_a_value() {
        // Host MMIO reads 0: SINGLEFIFODV never sets.
        let mut monitor = Iadc0Token::new().into_supply_monitor();
        assert_eq!(monitor.avdd_millivolts(), Err(IadcError::Timeout));
    }
}
