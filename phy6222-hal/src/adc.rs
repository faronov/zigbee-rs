//! ADC driver for battery voltage measurement.
//!
//! Enables ADC only during measurement, disables after to save power.

use crate::regs::*;

/// ADC channel (analog-capable GPIO pins).
#[derive(Clone, Copy)]
pub enum Channel {
    P11 = 2,
    P23 = 3,
    P24 = 4,
    P14 = 5,
    P15 = 6,
    P20 = 7,
}

/// Read battery voltage in millivolts.
///
/// Enables clocks + ADC, takes samples, converts to mV, then disables.
pub fn read_battery_mv(channel: Channel) -> u32 {
    let ch = channel as u32;

    // Enable clocks
    reg_write(PCRM_CLKSEL, reg_read(PCRM_CLKSEL) | (1 << 6));
    reg_write(PCRM_CLKHF_CTL0, reg_read(PCRM_CLKHF_CTL0) | (1 << 18));
    reg_write(PCRM_CLKHF_CTL1, reg_read(PCRM_CLKHF_CTL1) | (1 << 7) | (1 << 13));

    // Enable ADCC clock gate
    reg_write(PCR_SW_CLK, reg_read(PCR_SW_CLK) | MOD_ADCC_BIT);

    // Enable ADC + analog LDO
    reg_write(PCRM_ANA_CTL, reg_read(PCRM_ANA_CTL) | (1 << 3) | (1 << 0));

    // Stabilize
    for _ in 0..5_000u32 { cortex_m::asm::nop(); }

    // Configure channel
    let pmctl = reg_read(AON_PMCTL2_1);
    reg_write(AON_PMCTL2_1, pmctl | (1 << (ch + 8)));
    reg_write(PCRM_ADC_CTL4, (reg_read(PCRM_ADC_CTL4) & !0x1F) | 0x01);
    reg_write(PCRM_ADC_CTL0, 1 << ch);

    // Clear and enable interrupts
    reg_write(ADCC_BASE + 0x38, 0x1FF); // intr_clear
    reg_write(ADCC_BASE + 0x34, 1 << ch); // intr_mask
    reg_write(ADCC_BASE + 0x00, 1 << ch); // enable

    // Wait for conversion
    for _ in 0..100_000u32 {
        if reg_read(ADCC_BASE + 0x3C) & (1 << ch) != 0 { break; }
        cortex_m::asm::nop();
    }

    // Read samples (skip first 2 for settling)
    let ch_buf = ADC_CH_BASE + ch * 0x80;
    let mut sum: u32 = 0;
    let mut count: u32 = 0;
    for i in 2..12u32 {
        let raw = reg_read(ch_buf + i * 4) & 0xFFF;
        if raw > 0 { sum += raw; count += 1; }
    }

    // Disable ADC
    reg_write(ADCC_BASE + 0x00, 0);
    reg_write(ADCC_BASE + 0x38, 0x1FF);
    reg_write(AON_PMCTL2_1, pmctl);
    let ana = reg_read(PCRM_ANA_CTL);
    reg_write(PCRM_ANA_CTL, ana & !((1 << 3) | (1 << 0)));
    reg_write(PCR_SW_CLK, reg_read(PCR_SW_CLK) & !MOD_ADCC_BIT);

    if count == 0 { return 0; }
    let avg = sum / count;
    let scale = match channel { Channel::P15 => 1710u32, _ => 1904u32 };
    (avg * scale) >> 4
}

/// Convert millivolts to battery percentage (0-100).
pub fn mv_to_percent(mv: u32) -> u8 {
    if mv >= 3000 { 100 }
    else if mv <= 2000 { 0 }
    else { ((mv - 2000) / 10) as u8 }
}
