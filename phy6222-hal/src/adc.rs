//! Exclusive, fallible ADC driver for supply-voltage measurement.
//!
//! Enables ADC only during measurement, disables after to save power.

use crate::peripherals::AdcToken;
use crate::regs::*;

/// ADC channel (analog-capable GPIO pins).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    P11 = 2,
    P23 = 3,
    P24 = 4,
    P14 = 5,
    P15 = 6,
    P20 = 7,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    Busy,
    ConversionTimeout,
    NoSamples,
}

/// Exclusively owned PHY62x2 ADC.
pub struct Adc {
    _token: AdcToken,
}

impl Adc {
    pub fn new(token: AdcToken) -> Result<Self, AdcError> {
        if reg_read(ADCC_BASE) != 0 {
            return Err(AdcError::Busy);
        }
        Ok(Self { _token: token })
    }

    /// Read one analog channel in millivolts.
    ///
    /// Clock, analog, mux, and interrupt state are restored on every return
    /// path. Battery chemistry conversion deliberately belongs to the product.
    pub fn read_mv(&mut self, channel: Channel) -> Result<u32, AdcError> {
        let ch = channel as u32;
        let clksel = reg_read(PCRM_CLKSEL);
        let clkhf0 = reg_read(PCRM_CLKHF_CTL0);
        let clkhf1 = reg_read(PCRM_CLKHF_CTL1);
        let sw_clk = reg_read(PCR_SW_CLK);
        let ana_ctl = reg_read(PCRM_ANA_CTL);
        let pmctl = reg_read(AON_PMCTL2_1);

        reg_write(PCRM_CLKSEL, clksel | (1 << 6));
        reg_write(PCRM_CLKHF_CTL0, clkhf0 | (1 << 18));
        reg_write(PCRM_CLKHF_CTL1, clkhf1 | (1 << 7) | (1 << 13));
        reg_write(PCR_SW_CLK, sw_clk | MOD_ADCC_BIT);
        reg_write(PCRM_ANA_CTL, ana_ctl | (1 << 3) | (1 << 0));

        for _ in 0..5_000u32 {
            cortex_m::asm::nop();
        }

        reg_write(AON_PMCTL2_1, pmctl | (1 << (ch + 8)));
        reg_write(PCRM_ADC_CTL4, (reg_read(PCRM_ADC_CTL4) & !0x1F) | 0x01);
        reg_write(PCRM_ADC_CTL0, 1 << ch);
        reg_write(ADCC_BASE + 0x38, 0x1FF);
        reg_write(ADCC_BASE + 0x34, 1 << ch);
        reg_write(ADCC_BASE, 1 << ch);

        let result = if (0..100_000u32).any(|_| {
            let complete = reg_read(ADCC_BASE + 0x3C) & (1 << ch) != 0;
            if !complete {
                cortex_m::asm::nop();
            }
            complete
        }) {
            let ch_buf = ADC_CH_BASE + ch * 0x80;
            let mut sum = 0u32;
            let mut count = 0u32;
            for i in 2..12u32 {
                let raw = reg_read(ch_buf + i * 4) & 0xFFF;
                if raw != 0 {
                    sum += raw;
                    count += 1;
                }
            }
            if let Some(average) = sum.checked_div(count) {
                let scale = match channel {
                    Channel::P15 => 1_710u32,
                    _ => 1_904u32,
                };
                Ok((average * scale) >> 4)
            } else {
                Err(AdcError::NoSamples)
            }
        } else {
            Err(AdcError::ConversionTimeout)
        };

        reg_write(ADCC_BASE, 0);
        reg_write(ADCC_BASE + 0x38, 0x1FF);
        reg_write(AON_PMCTL2_1, pmctl);
        reg_write(PCRM_ANA_CTL, ana_ctl);
        reg_write(PCR_SW_CLK, sw_clk);
        reg_write(PCRM_CLKHF_CTL1, clkhf1);
        reg_write(PCRM_CLKHF_CTL0, clkhf0);
        reg_write(PCRM_CLKSEL, clksel);
        result
    }
}
