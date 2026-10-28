//! Blocking single-conversion ADC driver (`ADC0`).
//!
//! Mirrors `ADCLPF3_convert()` (TI Drivers) and the `driverlib/adc.c`
//! helpers it calls: MEMCTL0 only, software trigger, auto sampling, single
//! conversion, VDDS reference with the factory offset trim written to
//! `SYS0.TMUTE2` and the factory gain trim applied in software.
//!
//! The ADC clock is gated on only for [`Adc::session`]; the input pad is
//! configured analog (`PORTCFG = ANA`) for as long as the input is owned.
//! Deviation from TI: `GPIO_setConfigAndMux()` additionally sets
//! `INPEN|HYSTEN|WUENSB` on the ADC pad; the digital input buffer and wake
//! are not needed for an analog input and are left off.

use crate::gpio::Dio;

/// `ADC_BASE` (`hw_memmap.h`).
pub const ADC_BASE: u32 = 0x4005_0000;
const RIS0: u32 = 0x030;
const ICLR0: u32 = 0x048;
const CTL0: u32 = 0x100;
const CTL1: u32 = 0x104;
const CTL2: u32 = 0x108;
const SCOMP0: u32 = 0x114;
const REFCFG: u32 = 0x11C;
const MEMCTL0: u32 = 0x180;
const MEMRES0: u32 = 0x280;
const STA: u32 = 0x340;

const CTL0_ENC: u32 = 0x0000_0001;
const CTL0_SCLKDIV_M: u32 = 0x0700_0000;
const CTL1_SC_START: u32 = 0x0000_0100;
const CTL1_TRIGSRC_M: u32 = 0x0000_0001;
const CTL1_SAMPMODE_M: u32 = 0x0010_0000;
const CTL1_CONSEQ_M: u32 = 0x0003_0000;
const CTL2_RES_M: u32 = 0x0000_0006;
const CTL2_STARTADD_M: u32 = 0x001F_0000;
const CTL2_ENDADD_M: u32 = 0x1F00_0000;
const MEMCTL0_CHANSEL_M: u32 = 0x0000_001F;
const MEMCTL0_VRSEL_M: u32 = 0x0000_0300;
const STA_BUSY: u32 = 0x0000_0001;
const RIS0_OVIFG: u32 = 0x0000_0001;
const RIS0_UVIFG: u32 = 0x0000_0040;

/// `FCFG.appTrims.cc23x0r5.adcOffset` (byte 0: VDDS offset, int8).
pub const FCFG_ADC_OFFSET: u32 = crate::identity::FCFG_BASE + 0x384;
/// `FCFG.appTrims.cc23x0r5.adcGainWord0` (bits 15:0: VDDS gain).
pub const FCFG_ADC_GAIN_WORD0: u32 = crate::identity::FCFG_BASE + 0x388;

const SYS0_MUNLOCK: u32 = 0x4000_400C;
const SYS0_TMUTE2: u32 = 0x4000_4808;
const SYS0_TMUTE2_OFFSET_M: u32 = 0x0000_FFF0;
const SYS0_MUNLOCK_KEY: u32 = 0xC5AF_6927;

/// Largest 12-bit code.
pub const MAX_CODE_12BIT: u32 = 0xFFF;
/// Unity gain (`gain / 0x8000`); also used for untrimmed (0xFFFF) parts.
pub const UNITY_GAIN: u16 = 0x8000;

/// ADC clock divider (`ADC_CTL0_SCLKDIV_*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum ClockDivider {
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
    Div16 = 4,
    Div24 = 5,
    Div32 = 6,
    Div48 = 7,
}

impl ClockDivider {
    const fn ctl0_bits(self) -> u32 {
        (self as u32) << 24
    }
}

/// Conversion settings (12-bit, VDDS reference).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdcConfig {
    pub clock_divider: ClockDivider,
    /// `SCOMP0`: sample duration in divided ADC clocks.
    pub sample_cycles: u16,
    /// Nominal VDDS reference in µV used by [`code_to_microvolts`].
    pub reference_uv: u32,
}

impl Default for AdcConfig {
    /// Conservative SysConfig-style defaults: /8, 16 cycles, 3.3 V VDDS.
    fn default() -> Self {
        Self {
            clock_divider: ClockDivider::Div8,
            sample_cycles: 16,
            reference_uv: 3_300_000,
        }
    }
}

/// Conversion failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    /// `RIS0.OVIFG`/`UVIFG` was set (`ADC_STATUS_ERROR`).
    OverOrUnderflow,
    /// `STA.BUSY` never cleared.
    Timeout,
}

/// `ADCAdjustValueForGain()` for 12-bit results.
pub const fn adjust_for_gain(raw: u32, gain: u16) -> u32 {
    let adjusted = (raw * gain as u32 + 0x4000) / 0x8000;
    if adjusted > MAX_CODE_12BIT {
        MAX_CODE_12BIT
    } else {
        adjusted
    }
}

/// `ADCValueToMicrovolts()` for 12-bit results.
pub const fn code_to_microvolts(code: u32, reference_uv: u32) -> u32 {
    let reference = reference_uv >> 4;
    ((code * reference + (MAX_CODE_12BIT >> 1)) / MAX_CODE_12BIT) << 4
}

/// Factory gain word to gain (`0xFFFF` means untrimmed -> unity).
pub const fn gain_from_trim(word: u32) -> u16 {
    let gain = (word & 0xFFFF) as u16;
    if gain == 0xFFFF { UNITY_GAIN } else { gain }
}

/// `ADCSetAdjustmentOffset()`: merge the signed 8-bit offset into TMUTE2.
pub const fn tmute2_with_offset(tmute2: u32, offset_trim_word: u32) -> u32 {
    let offset = (offset_trim_word & 0xFF) as u8 as i8 as i16;
    (tmute2 & !SYS0_TMUTE2_OFFSET_M) | (((offset as i32) << 4) as u32 & SYS0_TMUTE2_OFFSET_M)
}

/// Ownership token for `ADC0` (from [`crate::Core`]).
pub struct Adc0 {
    _private: (),
}

impl Adc0 {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

/// An external analog input: the claimed pad and its internal channel
/// number (`ADCLPF3_HWAttrs.internalChannel`, from the device pin table).
pub struct AdcInput {
    pub pin: Dio,
    pub channel: u8,
}

/// Single-input ADC.
pub struct Adc {
    _token: Adc0,
    input: AdcInput,
    config: AdcConfig,
}

impl Adc {
    pub fn new(token: Adc0, input: AdcInput, config: AdcConfig) -> Self {
        assert!(u32::from(input.channel) <= MEMCTL0_CHANSEL_M);
        #[cfg(target_os = "none")]
        input
            .pin
            .set_ioc(crate::gpio::PinFunction::Analog.portcfg());
        Self {
            _token: token,
            input,
            config,
        }
    }

    pub fn release(self) -> (Adc0, AdcInput) {
        (self._token, self.input)
    }

    pub const fn config(&self) -> AdcConfig {
        self.config
    }

    /// Clock the ADC for the lifetime of the returned session.
    #[cfg(target_os = "none")]
    pub fn session(&mut self) -> AdcSession<'_> {
        crate::clkctl::enable(crate::clkctl::PeripheralClock::Adc0);
        let gain = gain_from_trim(rd_abs(FCFG_ADC_GAIN_WORD0));
        AdcSession { adc: self, gain }
    }
}

/// A powered ADC; the clock is gated again on drop.
#[cfg(target_os = "none")]
pub struct AdcSession<'a> {
    adc: &'a mut Adc,
    gain: u16,
}

#[cfg(target_os = "none")]
impl AdcSession<'_> {
    /// One blocking conversion; returns the raw (untrimmed-gain) code.
    pub fn convert_raw(&mut self) -> Result<u16, AdcError> {
        let config = self.adc.config;
        // ADCDisableConversion(); ADCSetMemctlRange(0, 0).
        modify(CTL0, CTL0_ENC, 0);
        modify(CTL2, CTL2_STARTADD_M | CTL2_ENDADD_M, 0);
        // ADCSetSampleDuration().
        modify(CTL0, CTL0_SCLKDIV_M, config.clock_divider.ctl0_bits());
        wr(SCOMP0, u32::from(config.sample_cycles));
        // ADCSetSamplingMode(AUTO); ADCSetResolution(12 bit).
        modify(CTL1, CTL1_SAMPMODE_M, 0);
        modify(CTL2, CTL2_RES_M, 0);
        // ADCSetInput(VDDS, channel, 0): VRSEL = VDDS (0), REFCFG = REFEN_DIS.
        modify(
            MEMCTL0,
            MEMCTL0_VRSEL_M | MEMCTL0_CHANSEL_M,
            u32::from(self.adc.input.channel),
        );
        wr(REFCFG, 0);
        // ADCSetAdjustmentOffset(VDDS).
        let tmute2 = tmute2_with_offset(rd_abs(SYS0_TMUTE2), rd_abs(FCFG_ADC_OFFSET));
        wr_abs(SYS0_MUNLOCK, SYS0_MUNLOCK_KEY);
        wr_abs(SYS0_TMUTE2, tmute2);
        wr_abs(SYS0_MUNLOCK, 0);
        // ADCSetSequence(SINGLE); ADCSetTriggerSource(SOFTWARE).
        modify(CTL1, CTL1_CONSEQ_M | CTL1_TRIGSRC_M, 0);
        // ADCEnableConversion(); ADCStartConversion(); BUSY rises after
        // 9 cycles on CC23X0, so wait before polling.
        modify(CTL0, CTL0_ENC, CTL0_ENC);
        modify(CTL1, CTL1_SC_START, CTL1_SC_START);
        cortex_m::asm::delay(24);

        let mut idle = false;
        for _ in 0..10_000 {
            if rd(STA) & STA_BUSY == 0 {
                idle = true;
                break;
            }
        }
        let code = rd(MEMRES0) & 0xFFFF;
        let status = rd(RIS0);
        wr(ICLR0, 0xFFFF_FFFF);
        if !idle {
            return Err(AdcError::Timeout);
        }
        if status & (RIS0_OVIFG | RIS0_UVIFG) != 0 {
            return Err(AdcError::OverOrUnderflow);
        }
        Ok(code as u16)
    }

    /// One conversion converted to µV against the nominal reference, after
    /// the factory gain trim (`ADCLPF3_convertToMicroVolts()`).
    pub fn convert_microvolts(&mut self) -> Result<u32, AdcError> {
        let raw = self.convert_raw()?;
        Ok(code_to_microvolts(
            adjust_for_gain(u32::from(raw), self.gain),
            self.adc.config.reference_uv,
        ))
    }
}

#[cfg(target_os = "none")]
impl Drop for AdcSession<'_> {
    fn drop(&mut self) {
        modify(CTL0, CTL0_ENC, 0);
        crate::clkctl::disable(crate::clkctl::PeripheralClock::Adc0);
    }
}

#[cfg(target_os = "none")]
fn rd(offset: u32) -> u32 {
    rd_abs(ADC_BASE + offset)
}

#[cfg(target_os = "none")]
fn wr(offset: u32, value: u32) {
    wr_abs(ADC_BASE + offset, value)
}

#[cfg(target_os = "none")]
fn modify(offset: u32, mask: u32, value: u32) {
    wr(offset, (rd(offset) & !mask) | (value & mask));
}

#[cfg(target_os = "none")]
fn rd_abs(address: u32) -> u32 {
    // SAFETY: fixed ADC/SYS0/FCFG addresses of the CC2340R52.
    unsafe { (address as *const u32).read_volatile() }
}

#[cfg(target_os = "none")]
fn wr_abs(address: u32, value: u32) {
    // SAFETY: fixed ADC/SYS0 addresses of the CC2340R52.
    unsafe { (address as *mut u32).write_volatile(value) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addresses_match_the_headers() {
        assert_eq!(ADC_BASE + CTL0, 0x4005_0100);
        assert_eq!(ADC_BASE + MEMRES0, 0x4005_0280);
        assert_eq!(ADC_BASE + STA, 0x4005_0340);
        // offsetof(fcfg_t, appTrims.cc23x0r5.adcOffset / adcGainWord0).
        assert_eq!(FCFG_ADC_OFFSET, 0x4E00_0384);
        assert_eq!(FCFG_ADC_GAIN_WORD0, 0x4E00_0388);
        assert_eq!(ClockDivider::Div8.ctl0_bits(), 0x0300_0000);
    }

    #[test]
    fn gain_adjustment_matches_driverlib() {
        assert_eq!(adjust_for_gain(2048, UNITY_GAIN), 2048);
        // +1 % gain.
        assert_eq!(adjust_for_gain(2000, 0x8148), 2020);
        // Saturates at full scale.
        assert_eq!(adjust_for_gain(0xFFF, 0x9000), 0xFFF);
        assert_eq!(gain_from_trim(0xFFFF_FFFF), UNITY_GAIN);
        assert_eq!(gain_from_trim(0x1234_7F00), 0x7F00);
    }

    #[test]
    fn microvolt_conversion_matches_driverlib() {
        assert_eq!(code_to_microvolts(0, 3_300_000), 0);
        assert_eq!(code_to_microvolts(0xFFF, 3_300_000), 3_300_000);
        // Half scale: ((2048 * 206250 + 2047) / 4095) << 4.
        assert_eq!(code_to_microvolts(2048, 3_300_000), 103_150 << 4);
    }

    #[test]
    fn offset_trim_is_sign_extended_into_tmute2() {
        assert_eq!(tmute2_with_offset(0xABCD_0000, 0x0000_0003), 0xABCD_0030);
        // -2 -> 0xFFE0 within the 12-bit field.
        assert_eq!(tmute2_with_offset(0x0000_000F, 0x0000_00FE), 0x0000_FFEF);
    }
}
