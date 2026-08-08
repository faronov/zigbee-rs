//! Archived BL702 eFuse/TSEN experiment; not a production HAL module.
//!
//! Secret-key slots and eFuse programming are intentionally not exposed.

use core::hint::spin_loop;

use crate::mmio::{read32, write32};
use crate::peripherals::Efuse;
use crate::timer::delay_us;

const EF_DATA_BASE: u32 = 0x4000_7000;
const EF_IF_CTRL_0: u32 = EF_DATA_BASE + 0x800;
const EFUSE_REGION_0_WORDS: usize = 32;
const WIFI_MAC_LOW: u32 = EF_DATA_BASE + 0x014;
const WIFI_MAC_HIGH: u32 = EF_DATA_BASE + 0x018;
/// EF_DATA_0_EF_KEY_SLOT_5_W3: ADC gain trim plus TSEN enable fields.
const KEY_SLOT_5_WORD3: u32 = EF_DATA_BASE + 0x078;
/// EF_DATA_0_LOCK: 12-bit TSEN refcode in bits [11:0], parity in bit 12.
const TSEN_CAL_WORD: u32 = EF_DATA_BASE + 0x07C;

const EF_IF_AUTOLOAD_DONE: u32 = 1 << 1;
const EF_IF_BUSY: u32 = 1 << 2;
const EF_IF_TRIGGER: u32 = 1 << 4;
const EF_IF_SAHB_CLOCK: u32 = 1 << 7;
const EF_IF_AUTO_READ_ENABLE: u32 = 1 << 18;
const EF_IF_INTERRUPT_CLEAR: u32 = 1 << 21;
const EF_IF_CONTROL_PROTECT: u32 = 0xbf << 8;
const EF_IF_READ_BASE: u32 = EF_IF_CONTROL_PROTECT | EF_IF_AUTO_READ_ENABLE | EF_IF_INTERRUPT_CLEAR;
const EFUSE_TIMEOUT_ITERATIONS: u32 = 160_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EfuseReloadError {
    BusyTimeout,
    AutoloadTimeout,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationWords {
    pub key_slot_5_word3: u32,
    pub tsen_cal_word: u32,
}

impl CalibrationWords {
    pub const fn adc_gain_denominator(self) -> Option<u16> {
        decode_adc_gain_denominator(self.key_slot_5_word3)
    }

    pub const fn tsen_refcode(self) -> Option<u16> {
        decode_tsen_refcode(self.key_slot_5_word3, self.tsen_cal_word)
    }
}

/// Public eFuse identity reader.
pub struct EfuseReader {
    _token: Efuse,
}

impl EfuseReader {
    pub const fn new(token: Efuse) -> Self {
        Self { _token: token }
    }

    /// Read the eight-byte factory chip identifier loaded into the eFuse
    /// shadow by the boot ROM.
    pub fn chip_id(&self) -> [u8; 8] {
        decode_chip_id(read32(WIFI_MAC_LOW), read32(WIFI_MAC_HIGH))
    }

    /// Read the public ADC/TSEN calibration words from the current shadow.
    pub fn calibration_words(&self) -> CalibrationWords {
        calibration_words()
    }

    /// Reload eFuse region 0 into its AHB shadow using the same sequence as
    /// `EF_Ctrl_Load_Efuse_R0` in the Bouffalo standard driver.
    ///
    /// The boot ROM normally preloads this shadow, but the vendor ADC and TSEN
    /// readers reload it before every calibration read. This operation never
    /// enables programming voltage and cannot alter physical eFuse bits.
    pub fn reload_shadow(&mut self) -> Result<(), EfuseReloadError> {
        reload_region_0()
    }
}

pub(crate) fn adc_gain_denominator() -> Option<u16> {
    calibration_words().adc_gain_denominator()
}

/// Read the 12-bit TSEN temperature-sensor calibration reference code from
/// the eFuse shadow.  Returns `None` when the enable flag is not set or the
/// parity check fails. Both conditions indicate the factory calibration
/// was not programmed and must be treated as invalid.
///
/// Register layout follows EF_Ctrl_Read_TSEN_Trim / bl_efuse_read_tsen_refcode
/// in the Bouffalo SDK:
///   - EF_DATA_0_EF_KEY_SLOT_5_W3 (0x78) bit 0 = enable
///   - EF_DATA_0_LOCK (0x7C) bits [11:0] = 12-bit refcode, bit 12 = parity
pub(crate) fn tsen_refcode() -> Option<u16> {
    calibration_words().tsen_refcode()
}

fn calibration_words() -> CalibrationWords {
    CalibrationWords {
        key_slot_5_word3: read32(KEY_SLOT_5_WORD3),
        tsen_cal_word: read32(TSEN_CAL_WORD),
    }
}

fn reload_region_0() -> Result<(), EfuseReloadError> {
    wait_until_idle().map_err(|()| EfuseReloadError::BusyTimeout)?;

    // Match EF_Ctrl_Sw_AHB_Clk_0 before touching the data shadow.
    write32(EF_IF_CTRL_0, EF_IF_READ_BASE | EF_IF_SAHB_CLOCK);

    // Autoload sets programmed one bits but does not guarantee clearing stale
    // shadow bits first. Preserve the boot-ROM copy so a timeout cannot leave
    // identity or RF calibration fields zeroed.
    let mut previous = [0_u32; EFUSE_REGION_0_WORDS];
    for (index, word) in previous.iter_mut().enumerate() {
        let address = EF_DATA_BASE + (index as u32 * 4);
        *word = read32(address);
        write32(address, 0);
    }

    write32(EF_IF_CTRL_0, EF_IF_READ_BASE);
    write32(EF_IF_CTRL_0, EF_IF_READ_BASE | EF_IF_TRIGGER);
    delay_us(10);

    for _ in 0..EFUSE_TIMEOUT_ITERATIONS {
        let status = read32(EF_IF_CTRL_0);
        if status & EF_IF_BUSY == 0 && status & EF_IF_AUTOLOAD_DONE != 0 {
            write32(EF_IF_CTRL_0, EF_IF_READ_BASE | EF_IF_SAHB_CLOCK);
            return Ok(());
        }
        spin_loop();
    }

    write32(EF_IF_CTRL_0, EF_IF_READ_BASE | EF_IF_SAHB_CLOCK);
    for (index, word) in previous.iter().enumerate() {
        write32(EF_DATA_BASE + (index as u32 * 4), *word);
    }
    Err(EfuseReloadError::AutoloadTimeout)
}

fn wait_until_idle() -> Result<(), ()> {
    for _ in 0..EFUSE_TIMEOUT_ITERATIONS {
        if read32(EF_IF_CTRL_0) & EF_IF_BUSY == 0 {
            return Ok(());
        }
        spin_loop();
    }
    Err(())
}

const fn decode_tsen_refcode(enable_word: u32, cal_word: u32) -> Option<u16> {
    let enabled = enable_word & 1 != 0;
    let refcode = (cal_word & 0x0fff) as u16;
    let parity = ((cal_word >> 12) & 1) as u16;
    if !enabled || (refcode.count_ones() as u16 & 1) != parity {
        return None;
    }
    Some(refcode)
}

const fn decode_chip_id(low: u32, high: u32) -> [u8; 8] {
    let low = low.to_le_bytes();
    let high = high.to_le_bytes();
    [
        low[0], low[1], low[2], low[3], high[0], high[1], high[2], high[3],
    ]
}

const fn decode_adc_gain_denominator(word: u32) -> Option<u16> {
    let value = ((word >> 1) & 0x0fff) as u16;
    let parity = ((word >> 13) & 1) as u16;
    let enabled = word & (1 << 14) != 0;
    if !enabled || (value.count_ones() as u16 & 1) != parity {
        return None;
    }

    if value & 0x0800 != 0 {
        let magnitude = (!value).wrapping_add(1) & 0x0fff;
        Some(2_048 + magnitude)
    } else {
        Some(2_048 - value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chip_id_matches_sdk_word_to_byte_order() {
        assert_eq!(
            decode_chip_id(0x4433_2211, 0x8877_6655),
            [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
        );
    }

    #[test]
    fn efuse_reload_control_words_match_vendor_sequence() {
        assert_eq!(EF_IF_READ_BASE, 0x0024_bf00);
        assert_eq!(EF_IF_READ_BASE | EF_IF_TRIGGER, 0x0024_bf10);
        assert_eq!(EF_IF_READ_BASE | EF_IF_SAHB_CLOCK, 0x0024_bf80);
    }

    #[test]
    fn adc_gain_trim_matches_sdk_signed_encoding_and_parity() {
        let positive = (0x0100 << 1) | (1 << 13) | (1 << 14);
        assert_eq!(decode_adc_gain_denominator(positive), Some(1_792));

        let negative = (0x0f00 << 1) | (1 << 14);
        assert_eq!(decode_adc_gain_denominator(negative), Some(2_304));

        assert_eq!(decode_adc_gain_denominator(positive ^ (1 << 13)), None);
        assert_eq!(decode_adc_gain_denominator(positive & !(1 << 14)), None);
    }

    // Build a test eFuse enable word (bit 0 = enable) and cal word
    // (bits 11:0 = refcode, bit 12 = popcount(refcode) & 1).
    const fn make_tsen_words(refcode: u16, enable: bool) -> (u32, u32) {
        let enable_word = if enable { 1u32 } else { 0u32 };
        let parity = refcode.count_ones() & 1;
        let cal_word = (refcode as u32 & 0x0fff) | (parity << 12);
        (enable_word, cal_word)
    }

    #[test]
    fn tsen_refcode_valid_returns_the_programmed_code() {
        // Typical Bouffalo reset-default refcode 0x8ff = 2303, 9 bits set.
        let (en, cal) = make_tsen_words(0x08ff, true);
        assert_eq!(decode_tsen_refcode(en, cal), Some(0x08ff));
    }

    #[test]
    fn tsen_refcode_disabled_returns_none() {
        let (_, cal) = make_tsen_words(0x08ff, true);
        assert_eq!(decode_tsen_refcode(0, cal), None);
    }

    #[test]
    fn tsen_refcode_parity_error_returns_none() {
        let (en, cal) = make_tsen_words(0x08ff, true);
        // Flip the parity bit.
        assert_eq!(decode_tsen_refcode(en, cal ^ (1 << 12)), None);
    }

    #[test]
    fn tsen_refcode_zero_with_even_parity_disabled_returns_none() {
        // refcode=0 has 0 bits set → even parity; stored parity must be 0.
        // Enable=0 → None regardless.
        assert_eq!(decode_tsen_refcode(0, 0), None);
    }
}
