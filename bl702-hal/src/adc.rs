//! BL702 GPADC single-ended external input and VBAT/2 measurement.
//!
//! The nominal millivolt conversion follows the SDK's 12-bit, 3.2 V reference
//! formula. Per-die gain trim is not applied, so hardware calibration is still
//! required before treating the converted value as precision metrology.

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
const TIMEOUT_ITERATIONS: u32 = 1_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sample {
    pub raw: u16,
    pub millivolts_nominal: u16,
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

    fn read_channel(&mut self, positive: u8) -> Result<u16, AdcError> {
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
                let calibrated = apply_gain_trim((word & 0xffff) as u16, self.gain_denominator);
                return Ok((calibrated >> 4).min(0x0fff));
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

const fn supply_mv_from_raw(raw: u16) -> u16 {
    Sample::from_raw_12bit(raw)
        .millivolts_nominal
        .saturating_mul(2)
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
}
