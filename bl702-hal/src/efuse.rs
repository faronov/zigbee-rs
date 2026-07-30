//! Read-only access to public BL702 eFuse shadow fields.
//!
//! Secret-key slots and eFuse programming are intentionally not exposed.

use crate::mmio::read32;
use crate::peripherals::Efuse;

const EF_DATA_BASE: u32 = 0x4000_7000;
const WIFI_MAC_LOW: u32 = EF_DATA_BASE + 0x014;
const WIFI_MAC_HIGH: u32 = EF_DATA_BASE + 0x018;
const GPADC_GAIN_TRIM: u32 = EF_DATA_BASE + 0x078;

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
}

pub(crate) fn adc_gain_denominator() -> Option<u16> {
    decode_adc_gain_denominator(read32(GPADC_GAIN_TRIM))
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
    fn adc_gain_trim_matches_sdk_signed_encoding_and_parity() {
        let positive = (0x0100 << 1) | (1 << 13) | (1 << 14);
        assert_eq!(decode_adc_gain_denominator(positive), Some(1_792));

        let negative = (0x0f00 << 1) | (1 << 14);
        assert_eq!(decode_adc_gain_denominator(negative), Some(2_304));

        assert_eq!(decode_adc_gain_denominator(positive ^ (1 << 13)), None);
        assert_eq!(decode_adc_gain_denominator(positive & !(1 << 14)), None);
    }
}
