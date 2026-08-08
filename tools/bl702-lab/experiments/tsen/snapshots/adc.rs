//! Archived BL702 GPADC/TSEN experiment; not a production HAL module.
//!
//! External/VBAT conversion follows the SDK's 12-bit, 3.2 V reference formula
//! and applies valid factory gain trim. TSEN uses its separate factory
//! reference code and the vendor's 16-bit dual-bias measurement sequence.

use core::hint::spin_loop;

use crate::clock::{Clocks, Peripheral, enable_and_reset};
use crate::gpio::{Analog, Pin};
use crate::mmio::{read32, rmw, write32};
use crate::peripherals::Adc;
use crate::timer::delay_us;

const GLB_BASE: u32 = 0x4000_0000;
const AON_BASE: u32 = 0x4000_f000;
const GPIP_BASE: u32 = 0x4000_2000;
const GPADC_CLOCK: u32 = GLB_BASE + 0x0a4;
const COMMAND: u32 = AON_BASE + 0x90c;
const CONFIG1: u32 = AON_BASE + 0x910;
const CONFIG2: u32 = AON_BASE + 0x914;
const OFFSET_CALIBRATION: u32 = AON_BASE + 0x938;
const FIFO_CONFIG: u32 = GPIP_BASE;
const FIFO_DATA: u32 = GPIP_BASE + 0x004;

const GLOBAL_ENABLE: u32 = 1 << 0;
const CONVERSION_START: u32 = 1 << 1;
const SOFT_RESET: u32 = 1 << 2;
const NEGATIVE_CHANNEL_MASK: u32 = 0x1f << 3;
const POSITIVE_CHANNEL_MASK: u32 = 0x1f << 8;
const NEGATIVE_GROUND: u32 = 1 << 13;
const MIC2_DIFFERENTIAL: u32 = 1 << 19;
const VBAT_ENABLE: u32 = 1 << 4;
const FIFO_CLEAR: u32 = 1 << 1;
const FIFO_OVERRUN: u32 = 1 << 5;
const FIFO_UNDERRUN: u32 = 1 << 6;
const FIFO_READY_CLEAR: u32 = 1 << 8;
const FIFO_OVERRUN_CLEAR: u32 = 1 << 9;
const FIFO_UNDERRUN_CLEAR: u32 = 1 << 10;
const FIFO_STATUS_CLEAR: u32 = FIFO_READY_CLEAR | FIFO_OVERRUN_CLEAR | FIFO_UNDERRUN_CLEAR;
const FIFO_COUNT_MASK: u32 = 0x3f << 16;
const FIFO_CONTROL_MASK: u32 = 1 | (0x3 << 22);
const TIMEOUT_ITERATIONS: u32 = 1_000_000;

// TSEN-specific CONFIG1/CONFIG2 fields. Positions and values follow
// ADC_Init()/ADC_Tsen_Init() in the Bouffalo BL702 SDK.
const C1_CAL_OS_EN: u32 = 1;
const C1_CONT_CONV_EN: u32 = 1 << 1;
const C1_RES_SEL_MASK: u32 = 0x7 << 2;
const C1_RES_16_WITH_256_AVERAGE: u32 = 4 << 2;
const C1_CLK_ANA_INV: u32 = 1 << 17;
const C1_CLK_DIV_MASK: u32 = 0x7 << 18;
const C1_CLK_DIV_16: u32 = 4 << 18;
const C1_SCAN_LENGTH_MASK: u32 = 0xf << 21;
const C1_SCAN_EN: u32 = 1 << 25;
const C1_DITHER_EN: u32 = 1 << 26;
const C1_V11_SEL_MASK: u32 = 0x3 << 27;
const C1_V11_1P1V: u32 = 1 << 27;
const C1_V18_SEL_MASK: u32 = 0x3 << 29;
const C1_V18_1P82V: u32 = 2 << 29;

const C2_DIFF_MODE: u32 = 1 << 2;
const C2_VREF_2P0V: u32 = 1 << 3;
const C2_TSEXT_SEL: u32 = 1 << 5; // 0 = internal diode
const C2_TS_EN: u32 = 1 << 6;
const C2_PGA_VCM_MASK: u32 = 0x3 << 7;
const C2_PGA_VCM_1P4V: u32 = 2 << 7; // ADC_Tsen_Init sets vcm=2
const C2_PGA_OS_CAL_MASK: u32 = 0xf << 9;
const C2_PGA_EN: u32 = 1 << 13;
const C2_PGA_VCMI_EN: u32 = 1 << 14;
const C2_CHOP_MODE_MASK: u32 = 0x3 << 15;
const C2_CHOP_AZ_PGA_ON: u32 = 2 << 15;
const C2_BIAS_SEL: u32 = 1 << 17;
const C2_TEST_EN: u32 = 1 << 18;
const C2_TEST_SEL_MASK: u32 = 0x7 << 19;
const C2_PGA2_GAIN_MASK: u32 = 0x7 << 22;
const C2_PGA2_GAIN_1: u32 = 1 << 22;
const C2_PGA1_GAIN_MASK: u32 = 0x7 << 25;
const C2_PGA1_GAIN_1: u32 = 1 << 25;
const C2_DELAY_MASK: u32 = 0x7 << 28;
const C2_DELAY_2: u32 = 2 << 28;
const C2_TSVBE_LOW: u32 = 1 << 31; // 0 = low-bias state (v0), 1 = high-bias (v1)

// TSEN-specific COMMAND (AON_BASE + 0x90C) bit fields.
const CMD_DWA_EN: u32 = 1 << 18;
const CMD_CHIP_SEN_PU: u32 = 1 << 27; // Active low: clear to enable the diode.
const CMD_SEN_SEL_MASK: u32 = 0x3 << 28;
const CMD_SEN_TEST_EN: u32 = 1 << 30;

/// GPADC positive channel for the internal TSEN diode (ADC_CHAN_TSEN_P = 14).
const TSEN_P_CHANNEL: u8 = 14;
const TSEN_SAMPLE_COUNT: usize = 16;
const TSEN_COMMAND_MASK: u32 = NEGATIVE_CHANNEL_MASK
    | POSITIVE_CHANNEL_MASK
    | NEGATIVE_GROUND
    | MIC2_DIFFERENTIAL
    | CONVERSION_START
    | CMD_DWA_EN
    | CMD_CHIP_SEN_PU
    | CMD_SEN_SEL_MASK
    | CMD_SEN_TEST_EN;
const TSEN_COMMAND_VALUE: u32 =
    (23 << 3) | ((TSEN_P_CHANNEL as u32) << 8) | MIC2_DIFFERENTIAL | CMD_DWA_EN;
const TSEN_CONFIG1_MASK: u32 = C1_CAL_OS_EN
    | C1_CONT_CONV_EN
    | C1_RES_SEL_MASK
    | C1_CLK_ANA_INV
    | C1_CLK_DIV_MASK
    | C1_SCAN_LENGTH_MASK
    | C1_SCAN_EN
    | C1_DITHER_EN
    | C1_V11_SEL_MASK
    | C1_V18_SEL_MASK;
const TSEN_CONFIG1_VALUE: u32 =
    C1_RES_16_WITH_256_AVERAGE | C1_CLK_DIV_16 | C1_DITHER_EN | C1_V11_1P1V | C1_V18_1P82V;
const TSEN_CONFIG2_MASK: u32 = C2_DIFF_MODE
    | C2_VREF_2P0V
    | VBAT_ENABLE
    | C2_TSEXT_SEL
    | C2_TS_EN
    | C2_PGA_VCM_MASK
    | C2_PGA_OS_CAL_MASK
    | C2_PGA_EN
    | C2_PGA_VCMI_EN
    | C2_CHOP_MODE_MASK
    | C2_BIAS_SEL
    | C2_TEST_EN
    | C2_TEST_SEL_MASK
    | C2_PGA2_GAIN_MASK
    | C2_PGA1_GAIN_MASK
    | C2_DELAY_MASK
    | C2_TSVBE_LOW;
const TSEN_CONFIG2_VALUE: u32 = C2_VREF_2P0V
    | C2_TS_EN
    | C2_PGA_VCM_1P4V
    | C2_PGA_EN
    | C2_CHOP_AZ_PGA_ON
    | C2_PGA2_GAIN_1
    | C2_PGA1_GAIN_1
    | C2_DELAY_2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sample {
    pub raw: u16,
    pub millivolts_nominal: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DieTemperatureSample {
    pub centi_celsius: i16,
    pub low_bias_code: i32,
    pub high_bias_code: i32,
    pub calibration_refcode: u16,
    #[cfg(feature = "diagnostic-probes")]
    pub registers: AdcRegisterSnapshot,
}

#[cfg(feature = "diagnostic-probes")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdcRegisterSnapshot {
    pub command: u32,
    pub config1: u32,
    pub config2: u32,
}

#[cfg(feature = "diagnostic-probes")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SupplyProbe {
    pub negative_ground: bool,
    pub fifo_code: u16,
    pub raw_12bit: u16,
    pub calibrated_12bit: u16,
    pub millivolts: u16,
}

impl Sample {
    pub const fn from_raw_12bit(raw: u16) -> Self {
        let raw = raw & 0x0fff;
        Self {
            raw,
            millivolts_nominal: ((raw as u32 * 3_200) / 4_096) as u16,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    UnsupportedPin,
    Timeout,
    Fifo,
    Clock,
    /// TSEN eFuse calibration is not programmed or has an invalid parity.
    CalibrationInvalid,
}

pub struct Gpadc {
    _token: Adc,
    gain_denominator: u16,
    gain_trim_valid: bool,
}

impl Gpadc {
    pub fn new(token: Adc, clocks: Clocks) -> Result<Self, AdcError> {
        if clocks.xclk_hz() != 32_000_000 {
            return Err(AdcError::Clock);
        }

        enable_and_reset(Peripheral::Gpip);

        // XCLK, divider 1, enabled. Bit 6 is reserved.
        rmw(GPADC_CLOCK, 0x3f | (1 << 7) | (1 << 8), (1 << 7) | (1 << 8));
        rmw(COMMAND, GLOBAL_ENABLE, 0);
        rmw(COMMAND, GLOBAL_ENABLE, GLOBAL_ENABLE);
        let command = read32(COMMAND);
        write32(COMMAND, command | SOFT_RESET);
        aon_wait();
        write32(COMMAND, command & !SOFT_RESET);

        // SDK normal-mode defaults: V18=1.82 V, V11=1.1 V, /16 ADC clock,
        // 12-bit resolution, no offset calibration or scan/continuous mode.
        let config1_mask = 1
            | (1 << 1)
            | (0x7 << 2)
            | (1 << 17)
            | (0x7 << 18)
            | (0xf << 21)
            | (1 << 25)
            | (1 << 26)
            | (0x3 << 27)
            | (0x3 << 29);
        let config1_value = (4 << 18) | (1 << 27) | (2 << 29);
        rmw(CONFIG1, config1_mask, config1_value);
        aon_wait();

        // Delay=2, PGA gains=1, main bandgap, Vref/PGA chopping, VCM=1.2 V,
        // 3.2 V reference, single-ended input.
        let config2_mask = (1 << 2)
            | (1 << 3)
            | VBAT_ENABLE
            | (0x3 << 7)
            | (0xf << 9)
            | (1 << 13)
            | (1 << 14)
            | (0x3 << 15)
            | (1 << 17)
            | (0x7 << 22)
            | (0x7 << 25)
            | (0x7 << 28);
        let config2_value =
            (1 << 7) | (8 << 9) | (1 << 13) | (2 << 15) | (1 << 22) | (1 << 25) | (2 << 28);
        rmw(CONFIG2, config2_mask, config2_value);
        rmw(
            COMMAND,
            NEGATIVE_GROUND | MIC2_DIFFERENTIAL,
            NEGATIVE_GROUND | MIC2_DIFFERENTIAL,
        );
        rmw(OFFSET_CALIBRATION, 0xffff, 0);

        // FIFO threshold one, DMA disabled, clear stale samples.
        rmw(FIFO_CONFIG, 1 | FIFO_CLEAR | (0x3 << 22), FIFO_CLEAR);
        let gain_denominator = crate::efuse::adc_gain_denominator();
        Ok(Self {
            _token: token,
            gain_denominator: gain_denominator.unwrap_or(2_048),
            gain_trim_valid: gain_denominator.is_some(),
        })
    }

    pub const fn gain_trim_valid(&self) -> bool {
        self.gain_trim_valid
    }

    pub const fn gain_denominator(&self) -> u16 {
        self.gain_denominator
    }

    pub fn read_pin<const N: u8>(&mut self, _pin: &mut Pin<N, Analog>) -> Result<Sample, AdcError> {
        let channel = pin_channel(N).ok_or(AdcError::UnsupportedPin)?;
        self.read_channel(channel).map(Sample::from_raw_12bit)
    }

    /// Read the internal VBAT/2 channel and return the nominal full supply
    /// voltage after applying the documented 2:1 divider.
    pub fn read_supply_mv(&mut self) -> Result<u16, AdcError> {
        rmw(CONFIG2, VBAT_ENABLE, VBAT_ENABLE);
        let result = self.read_channel(18).map(supply_mv_from_raw);
        rmw(CONFIG2, VBAT_ENABLE, 0);
        result
    }

    #[cfg(feature = "diagnostic-probes")]
    pub fn probe_supply(&mut self, negative_ground: bool) -> Result<SupplyProbe, AdcError> {
        let saved_command = read32(COMMAND);
        let saved_config2 = read32(CONFIG2);
        rmw(
            COMMAND,
            NEGATIVE_GROUND,
            if negative_ground { NEGATIVE_GROUND } else { 0 },
        );
        rmw(CONFIG2, VBAT_ENABLE, VBAT_ENABLE);

        let result = self.read_channel_fifo(18).map(|fifo_code| {
            let raw_12bit = fifo_code >> 4;
            let calibrated_12bit =
                (apply_gain_trim(fifo_code, self.gain_denominator) >> 4).min(0x0fff);
            SupplyProbe {
                negative_ground,
                fifo_code,
                raw_12bit,
                calibrated_12bit,
                millivolts: supply_mv_from_raw(calibrated_12bit),
            }
        });

        write32(CONFIG2, saved_config2);
        write32(COMMAND, saved_command);
        result
    }

    /// Read the die temperature from the internal TSEN diode and return the
    /// result in **centi-degrees Celsius** (e.g., 2500 = 25.00 °C).
    ///
    /// This follows the Bouffalo SDK `ADC_Tsen_Init` + vendor measurement
    /// sequence: 16-bit conversion with 256-sample hardware averaging, a
    /// discarded first sample, and 16 software samples at each TSVBE state.
    /// Gain trim is deliberately not applied, matching the SDK behavior.
    ///
    /// Returns `Err(AdcError::CalibrationInvalid)` when the eFuse TSEN
    /// refcode is not programmed or fails its parity check. Other errors
    /// reflect FIFO or timeout faults; the normal GPADC configuration
    /// is restored before returning in all cases.
    pub fn read_die_temp_centi_c(&mut self) -> Result<i16, AdcError> {
        self.read_die_temperature()
            .map(|sample| sample.centi_celsius)
    }

    /// Read the calibrated die temperature together with the averaged raw
    /// codes used to calculate it.
    pub fn read_die_temperature(&mut self) -> Result<DieTemperatureSample, AdcError> {
        self.read_die_temperature_mode(false)
    }

    #[cfg(feature = "diagnostic-probes")]
    pub fn probe_die_temperature_with_negative_ground(
        &mut self,
    ) -> Result<DieTemperatureSample, AdcError> {
        self.read_die_temperature_mode(true)
    }

    fn read_die_temperature_mode(
        &mut self,
        negative_ground: bool,
    ) -> Result<DieTemperatureSample, AdcError> {
        let refcode = crate::efuse::tsen_refcode().ok_or(AdcError::CalibrationInvalid)?;

        // Save registers before modifying them for TSEN mode.
        let saved_command = read32(COMMAND);
        let saved_config1 = read32(CONFIG1);
        let saved_config2 = read32(CONFIG2);
        let saved_offset_calibration = read32(OFFSET_CALIBRATION);
        let saved_fifo_control = read32(FIFO_CONFIG) & FIFO_CONTROL_MASK;

        // The vendor TSEN path starts from a fresh ADC reset. This also avoids
        // carrying VBAT mux state into the internal-diode measurement.
        reset_adc();

        // Internal diode, sensor path enabled (CHIP_SEN_PU is active-low), DWA
        // enabled, and sensor test mux disabled.
        rmw(
            COMMAND,
            TSEN_COMMAND_MASK,
            tsen_command_value(negative_ground),
        );

        // Match ADC_Init() plus ADC_Tsen_Init(): 2.0 V reference, unity PGA,
        // main bandgap, AZ+PGA chop, internal diode, and low-bias state.
        rmw(CONFIG2, TSEN_CONFIG2_MASK, TSEN_CONFIG2_VALUE);

        // 16-bit results with 256-conversion hardware averaging. Disable scan
        // and continuous conversion, retain the documented /16 ADC clock,
        // and enable the dither required by ADC_Tsen_Init().
        rmw(CONFIG1, TSEN_CONFIG1_MASK, TSEN_CONFIG1_VALUE);
        aon_wait();
        rmw(OFFSET_CALIBRATION, 0xffff, 0);
        rmw(FIFO_CONFIG, FIFO_CONTROL_MASK | FIFO_CLEAR, FIFO_CLEAR);
        #[cfg(feature = "diagnostic-probes")]
        let registers = AdcRegisterSnapshot {
            command: read32(COMMAND),
            config1: read32(CONFIG1),
            config2: read32(CONFIG2),
        };

        let result = (|| {
            // The first conversion after TSEN initialization is invalid.
            let _ = self.read_tsen_raw()?;

            // Each FIFO word already averages 256 ADC conversions; the SDK
            // then averages 16 FIFO words at each TSVBE bias state.
            rmw(CONFIG2, C2_TSVBE_LOW, 0);
            let v0 = self.average_tsen_samples()?;
            rmw(CONFIG2, C2_TSVBE_LOW, C2_TSVBE_LOW);
            let v1 = self.average_tsen_samples()?;

            Ok(DieTemperatureSample {
                centi_celsius: tsen_centi_c(v0, v1, refcode),
                low_bias_code: v0,
                high_bias_code: v1,
                calibration_refcode: refcode,
                #[cfg(feature = "diagnostic-probes")]
                registers,
            })
        })();

        // Restore ordinary GPADC state before propagating any conversion
        // error. Reset the TSEN analog state before restoring normal mode.
        rmw(COMMAND, CONVERSION_START, 0);
        reset_adc();
        write32(CONFIG1, saved_config1);
        aon_wait();
        write32(CONFIG2, saved_config2);
        write32(OFFSET_CALIBRATION, saved_offset_calibration);
        rmw(FIFO_CONFIG, FIFO_CONTROL_MASK, saved_fifo_control);
        write32(COMMAND, saved_command);

        result
    }

    fn average_tsen_samples(&mut self) -> Result<i32, AdcError> {
        let mut sum = 0_i32;
        for _ in 0..TSEN_SAMPLE_COUNT {
            sum += i32::from(self.read_tsen_raw()?);
        }
        Ok(sum / TSEN_SAMPLE_COUNT as i32)
    }

    fn read_channel(&mut self, positive: u8) -> Result<u16, AdcError> {
        self.read_channel_fifo(positive).map(|raw| {
            let calibrated = apply_gain_trim(raw, self.gain_denominator);
            (calibrated >> 4).min(0x0fff)
        })
    }

    fn read_channel_fifo(&mut self, positive: u8) -> Result<u16, AdcError> {
        rmw(
            COMMAND,
            POSITIVE_CHANNEL_MASK | NEGATIVE_CHANNEL_MASK,
            (u32::from(positive) << 8) | (23 << 3),
        );
        rmw(CONFIG1, (1 << 1) | (1 << 25), 0);
        clear_fifo_status();
        rmw(COMMAND, CONVERSION_START, 0);
        // The BL702 SDK requires 100 us between clearing and reasserting
        // CONV_START. A shorter delay can return stale or invalid samples.
        delay_us(100);
        rmw(COMMAND, CONVERSION_START, CONVERSION_START);

        for _ in 0..TIMEOUT_ITERATIONS {
            let status = read32(FIFO_CONFIG);
            if status & (FIFO_OVERRUN | FIFO_UNDERRUN) != 0 {
                rmw(COMMAND, CONVERSION_START, 0);
                clear_fifo_status();
                return Err(AdcError::Fifo);
            }
            if status & FIFO_COUNT_MASK != 0 {
                let word = read32(FIFO_DATA);
                rmw(COMMAND, CONVERSION_START, 0);
                return Ok((word & 0xffff) as u16);
            }
            spin_loop();
        }
        rmw(COMMAND, CONVERSION_START, 0);
        Err(AdcError::Timeout)
    }

    /// Single TSEN conversion: returns the raw signed 16-bit FIFO word without
    /// gain trim, matching the HOSAL TSEN implementation.
    ///
    /// The GPADC must already be configured for TSEN mode before calling
    /// this (CONFIG2 TS_EN, DITHER_EN, channel set); TSVBE_LOW is managed
    /// by the caller.
    fn read_tsen_raw(&mut self) -> Result<i16, AdcError> {
        clear_fifo_status();
        rmw(COMMAND, CONVERSION_START, 0);
        delay_us(100);
        rmw(COMMAND, CONVERSION_START, CONVERSION_START);

        for _ in 0..TIMEOUT_ITERATIONS {
            let status = read32(FIFO_CONFIG);
            if status & (FIFO_OVERRUN | FIFO_UNDERRUN) != 0 {
                rmw(COMMAND, CONVERSION_START, 0);
                clear_fifo_status();
                return Err(AdcError::Fifo);
            }
            if status & FIFO_COUNT_MASK != 0 {
                let word = read32(FIFO_DATA);
                rmw(COMMAND, CONVERSION_START, 0);
                return Ok((word & 0xffff) as u16 as i16);
            }
            spin_loop();
        }
        rmw(COMMAND, CONVERSION_START, 0);
        Err(AdcError::Timeout)
    }
}

fn apply_gain_trim(raw: u16, denominator: u16) -> u16 {
    let calibrated = (u32::from(raw) * 2_048 + u32::from(denominator / 2)) / u32::from(denominator);
    calibrated.min(u32::from(u16::MAX)) as u16
}

const fn tsen_command_value(negative_ground: bool) -> u32 {
    TSEN_COMMAND_VALUE | if negative_ground { NEGATIVE_GROUND } else { 0 }
}

const fn supply_mv_from_raw(raw: u16) -> u16 {
    Sample::from_raw_12bit(raw)
        .millivolts_nominal
        .saturating_mul(2)
}

/// Convert two averaged raw 16-bit TSEN readings into centi-degrees Celsius.
///
/// Implements the robust Bouffalo standard-driver formula:
/// `temp_C = (abs(v0 - v1) - refcode) / 7.753`.
pub(crate) fn tsen_centi_c(v0: i32, v1: i32, refcode: u16) -> i16 {
    let delta = (i64::from(v0) - i64::from(v1)).abs() - i64::from(refcode);
    let centi = round_div_symmetric(delta * 100_000, 7_753);
    centi.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16
}

fn round_div_symmetric(value: i64, divisor: i64) -> i64 {
    if value >= 0 {
        (value + divisor / 2) / divisor
    } else {
        -((-value + divisor / 2) / divisor)
    }
}

const fn pin_channel(pin: u8) -> Option<u8> {
    match pin {
        8 => Some(0),
        15 => Some(1),
        17 => Some(2),
        11 => Some(3),
        12 => Some(4),
        14 => Some(5),
        7 => Some(6),
        9 => Some(7),
        18 => Some(8),
        19 => Some(9),
        20 => Some(10),
        21 => Some(11),
        _ => None,
    }
}

fn clear_fifo_status() {
    // The GPIP FIFO clear is self-clearing, while the ready/overrun/underrun
    // latches use the SDK's explicit low-high-low clear sequence.
    rmw(FIFO_CONFIG, FIFO_STATUS_CLEAR, 0);
    rmw(
        FIFO_CONFIG,
        FIFO_CLEAR | FIFO_STATUS_CLEAR,
        FIFO_CLEAR | FIFO_STATUS_CLEAR,
    );
    rmw(FIFO_CONFIG, FIFO_STATUS_CLEAR, 0);
}

fn reset_adc() {
    rmw(COMMAND, GLOBAL_ENABLE | CONVERSION_START, 0);
    let command = read32(COMMAND);
    write32(COMMAND, command | SOFT_RESET);
    aon_wait();
    write32(COMMAND, command & !SOFT_RESET);
    aon_wait();
    rmw(COMMAND, GLOBAL_ENABLE, GLOBAL_ENABLE);
}

#[inline(always)]
fn aon_wait() {
    for _ in 0..8 {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adc_pin_map_matches_generated_header() {
        assert_eq!(pin_channel(8), Some(0));
        assert_eq!(pin_channel(21), Some(11));
        assert_eq!(pin_channel(6), None);
    }

    #[test]
    fn nominal_conversion_matches_sdk_formula() {
        assert_eq!(Sample::from_raw_12bit(0).millivolts_nominal, 0);
        assert_eq!(Sample::from_raw_12bit(2048).millivolts_nominal, 1600);
        assert_eq!(Sample::from_raw_12bit(4095).millivolts_nominal, 3199);
    }

    #[test]
    fn vbat_channel_applies_only_the_documented_two_to_one_divider() {
        assert_eq!(supply_mv_from_raw(0), 0);
        assert_eq!(supply_mv_from_raw(2048), 3200);
        assert_eq!(supply_mv_from_raw(4095), 6398);
    }

    #[test]
    fn gain_trim_uses_the_sdk_fixed_point_coefficient() {
        assert_eq!(apply_gain_trim(4_096, 2_048), 4_096);
        assert_eq!(apply_gain_trim(4_096, 4_096), 2_048);
        assert_eq!(apply_gain_trim(4_096, 1_024), 8_192);
    }

    #[test]
    fn tsen_register_values_match_vendor_configuration() {
        assert_eq!(
            TSEN_CONFIG1_VALUE,
            (4 << 2) | (4 << 18) | (1 << 26) | (1 << 27) | (2 << 29)
        );
        assert_eq!(
            TSEN_CONFIG2_VALUE,
            (1 << 3)
                | (1 << 6)
                | (2 << 7)
                | (1 << 13)
                | (2 << 15)
                | (1 << 22)
                | (1 << 25)
                | (2 << 28)
        );
        assert_eq!(TSEN_COMMAND_VALUE & CMD_CHIP_SEN_PU, 0);
        assert_eq!(
            TSEN_COMMAND_VALUE,
            (23 << 3) | (14 << 8) | (1 << 18) | (1 << 19)
        );
        assert_eq!(tsen_command_value(false) & NEGATIVE_GROUND, 0);
        assert_ne!(tsen_command_value(true) & NEGATIVE_GROUND, 0);
    }

    /// At the calibration refcode (vdelta = refcode) the temperature is 0 °C.
    #[test]
    fn tsen_centi_c_at_refcode_is_zero_degrees() {
        assert_eq!(tsen_centi_c(2303, 0, 2303), 0);
        assert_eq!(tsen_centi_c(0, 2303, 2303), 0);
    }

    /// 25 °C: vdelta ≈ refcode + 25 × 7.753 = 2303 + 193.825 ≈ 2497.
    #[test]
    fn tsen_centi_c_typical_room_temperature() {
        // vdelta = 2497, refcode = 2303 -> delta = 194
        // 194 * 100_000 / 7753 = 2502.26 -> 2502.
        let result = tsen_centi_c(2497, 0, 2303);
        assert_eq!(result, 2502);
        assert_eq!(tsen_centi_c(0, 2497, 2303), result);
    }

    /// Negative temperature: delta < 0 when vdelta < refcode.
    #[test]
    fn tsen_centi_c_negative_temperature() {
        // delta = -77 -> -77 * 100_000 / 7753 = -993.16 -> -993.
        let result = tsen_centi_c(2226, 0, 2303);
        assert_eq!(result, -993);
    }

    /// Result is clamped to the i16 Zigbee wire range.
    #[test]
    fn tsen_centi_c_clamps_to_i16_range() {
        assert_eq!(
            tsen_centi_c(i32::from(i16::MAX), i32::from(i16::MIN), 0),
            i16::MAX
        );
        assert_eq!(tsen_centi_c(0, 0, u16::MAX), i16::MIN);
    }

    /// Fixed-point rounding is symmetric around zero.
    #[test]
    fn tsen_centi_c_rounds_symmetrically() {
        assert_eq!(tsen_centi_c(1, 0, 0), 13);
        assert_eq!(tsen_centi_c(0, 0, 1), -13);
        assert_eq!(round_div_symmetric(5, 2), 3);
        assert_eq!(round_div_symmetric(-5, 2), -3);
    }
}
