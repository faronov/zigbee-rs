//! Exclusive, polled, single-ended ADC measurement.
//!
//! Register model from the PHYPLUS SDK `components/driver/adc/adc.c`
//! (`hal_adc_config`, `set_sampling_resolution_auto`, `hal_adc_value_read`)
//! and pvvx THB2 `bthome_phy6222/source/battery.c`:
//!
//! - per-pin enable bits live in `PCRM ADC_CTL1..3` (bit 20 / bit 4), not in
//!   `ADC_CTL0`;
//! - the 3.2 V input range sets `AON PMCTL2_1` bit `aio + 8` (and clears
//!   bit `aio`), where `aio` is the attenuator index of the pin;
//! - a channel's samples and status appear at the *paired* index
//!   (`ch ^ 1`): `ADC_CH_BASE + slot * 0x80`, status `ADCC intr_status`
//!   bit `slot`; the first two words are discarded and each later word holds
//!   two 12-bit samples;
//! - the ADC clock is the DLL-derived digital 32 MHz clock: `CLKHF_CTL1`
//!   bit 21 (`sel_dig_dble_clk_32M`) must be clear, as THB2 and the SDK
//!   `voice.c` do. The full SDK's `hal_adc_config` writes `[21:20] = 1`
//!   instead; on PB-03F (PHY6252, 48 MHz HCLK, doubler off) that leaves the
//!   ADC without a clock and no conversion ever completes;
//! - nothing is written to `ADCC enable` (offset 0): auto mode samples as soon
//!   as `ANA_CTL` bits 3/0 power the ADC.
//!
//! The ADC is powered only during [`Adc::read_mv`]; every register it touches
//! is restored on every return path. Battery chemistry belongs to the product.

use crate::gpio;
use crate::peripherals::AdcToken;
use crate::regs::*;

/// ADC channel (analog-capable GPIO pins), numbered as SDK `adc_CH_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    P11 = 2,
    P23 = 3,
    P24 = 4,
    P14 = 5,
    P15 = 6,
    P20 = 7,
}

/// How the measured pin is connected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    /// External analog signal (e.g. a supply divider): the pad is floated and
    /// routed to the ADC through `IOMUX Analog_IO_en` (SDK default).
    Analog,
    /// Supply measurement without a divider (THB2 `ADC_PIN_USE_OUT`): the
    /// pin is driven high as a push-pull output, so the ADC reads the pad
    /// at VDD. Use only on a pin that nothing else drives or loads.
    DrivenHigh,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    /// No conversion finished within the polling budget.
    ConversionTimeout,
}

/// Words read per conversion (SDK `MAX_ADC_SAMPLE_SIZE - 2`).
pub const SAMPLE_WORDS: u32 = 30;
/// Samples per conversion: two 12-bit samples per word (SDK
/// `hal_adc_value_read`).
pub const SAMPLE_COUNT: u32 = SAMPLE_WORDS * 2;

const ANA_CTL_ADC_EN: u32 = 1 << 3;
const ANA_CTL_ANALOG_LDO: u32 = 1 << 0;
const ANA_CTL_MICBIAS: u32 = 1 << 23;
const ADCC_INTR_MASK: u32 = ADCC_BASE + 0x34;
const ADCC_INTR_CLEAR: u32 = ADCC_BASE + 0x38;
const ADCC_INTR_STATUS: u32 = ADCC_BASE + 0x3C;
const ADC_CTL_PIN_BITS: u32 = (1 << 20) | (1 << 4);
const CONVERSION_POLLS: u32 = 200_000;

impl Channel {
    /// HAL logical GPIO index of the pin.
    pub const fn gpio(self) -> u8 {
        match self {
            Self::P11 => 7,
            Self::P23 => 14,
            Self::P24 => 15,
            Self::P14 => 8,
            Self::P15 => 9,
            Self::P20 => 13,
        }
    }

    /// Attenuator index in `AON PMCTL2_1` (SDK `set_sampling_resolution`).
    pub const fn attenuator(self) -> u32 {
        match self {
            Self::P11 => 0,
            Self::P23 => 1,
            Self::P24 => 2,
            Self::P14 => 3,
            Self::P15 => 4,
            Self::P20 => 7,
        }
    }

    /// `PCRM ADC_CTLx` register and bit enabling this pin.
    pub const fn enable_bit(self) -> (u32, u32) {
        match self {
            Self::P11 => (PCRM_ADC_CTL1, 20),
            Self::P23 => (PCRM_ADC_CTL1, 4),
            Self::P24 => (PCRM_ADC_CTL2, 20),
            Self::P14 => (PCRM_ADC_CTL2, 4),
            Self::P15 => (PCRM_ADC_CTL3, 20),
            Self::P20 => (PCRM_ADC_CTL3, 4),
        }
    }

    /// Paired buffer/status index where this channel's samples appear.
    pub const fn result_slot(self) -> u32 {
        (self as u32) ^ 1
    }

    /// `IOMUX Analog_IO_en` bit (`Analog_IO_n`, P11 = 0).
    pub const fn analog_io_bit(self) -> u32 {
        self.gpio() as u32 - 7
    }

    /// SDK QFN32 `adc_Lambda` / 1000 for the 3.2 V (attenuated) range.
    pub const fn lambda_k(self) -> u32 {
        match self {
            Self::P11 => 4_520,
            Self::P23 => 4_309,
            Self::P24 => 4_263,
            Self::P14 => 4_483,
            Self::P15 => 4_180,
            Self::P20 => 4_072,
        }
    }
}

/// Convert a sum of `count` 12-bit samples to millivolts.
///
/// SDK `hal_adc_value_cal` without flash calibration:
/// `V = avg / 4096 * lambda * 0.8 / 1e6`, i.e.
/// `mV = avg * lambda_k / 5120`.
pub const fn sum_to_millivolts(channel: Channel, sum: u32, count: u32) -> u32 {
    if count == 0 {
        return 0;
    }
    // 60 * 4095 * 4520 < u32::MAX.
    sum * channel.lambda_k() / (count * 5_120)
}

/// Exclusively owned PHY62x2 ADC.
pub struct Adc {
    _token: AdcToken,
}

impl Adc {
    /// Take ownership and force the ADC into its powered-down state.
    ///
    /// `PCRM` survives warm and debug resets, so an ADC enable left behind by
    /// a previous image is expected; the token is the exclusivity guarantee.
    pub fn new(token: AdcToken) -> Self {
        reg_write(ADCC_INTR_MASK, 0);
        reg_write(
            PCRM_ANA_CTL,
            reg_read(PCRM_ANA_CTL) & !(ANA_CTL_ADC_EN | ANA_CTL_ANALOG_LDO | ANA_CTL_MICBIAS),
        );
        reg_write(ADCC_INTR_CLEAR, 0x1FF);
        Self { _token: token }
    }

    /// Read one channel in millivolts (3.2 V range, uncalibrated).
    pub fn read_mv(&mut self, channel: Channel, input: Input) -> Result<u32, AdcError> {
        let sum = self.read_sum(channel, input)?;
        Ok(sum_to_millivolts(channel, sum, SAMPLE_COUNT))
    }

    /// Sum of [`SAMPLE_COUNT`] raw 12-bit samples.
    pub fn read_sum(&mut self, channel: Channel, input: Input) -> Result<u32, AdcError> {
        let pin = channel.gpio();
        let pin_bit = 1u32 << pin;
        let slot_bit = 1u32 << channel.result_slot();
        let (ctl_reg, ctl_bit) = channel.enable_bit();
        let ctl_regs = [PCRM_ADC_CTL0, PCRM_ADC_CTL1, PCRM_ADC_CTL2, PCRM_ADC_CTL3];

        let saved_ctl = ctl_regs.map(reg_read);
        let ctl4 = reg_read(PCRM_ADC_CTL4);
        let clksel = reg_read(PCRM_CLKSEL);
        let clkhf0 = reg_read(PCRM_CLKHF_CTL0);
        let clkhf1 = reg_read(PCRM_CLKHF_CTL1);
        let sw_clk = reg_read(PCR_SW_CLK);
        let ana_ctl = reg_read(PCRM_ANA_CTL);
        let pmctl = reg_read(AON_PMCTL2_1);
        let analog_io = reg_read(IOMUX_ANALOG_IO_EN);
        let pad_ps0 = reg_read(IOMUX_PAD_PS0);
        let ddr = reg_read(GPIO_SWPORTA_DDR);
        let dr = reg_read(GPIO_SWPORTA_DR);
        let intr_mask = reg_read(ADCC_INTR_MASK);

        // init_adc_batt / hal_adc_config: power down, reset and gate the ADC.
        reg_write(AON_PMCTL2_1, 0);
        reg_write(
            PCRM_ANA_CTL,
            ana_ctl & !(ANA_CTL_ADC_EN | ANA_CTL_ANALOG_LDO | ANA_CTL_MICBIAS),
        );
        reg_write(PCR_SW_CLK, sw_clk | MOD_ADCC_BIT);
        reset_adcc();
        // 1.28 MHz clock, XTAL output, DLL, digital 32 MHz from the DLL (not
        // the doubler), ADC clock enable.
        reg_write(PCRM_CLKSEL, clksel | (1 << 6));
        reg_write(PCRM_CLKHF_CTL0, clkhf0 | (1 << 18));
        reg_write(
            PCRM_CLKHF_CTL1,
            (clkhf1 & !(1 << 21)) | (1 << 7) | (1 << 13),
        );
        // Auto mode.
        reg_write(PCRM_ADC_CTL4, (ctl4 & !(1 << 4)) | 1);
        // 3.2 V range for this pin's attenuator.
        reg_write(AON_PMCTL2_1, 1 << (channel.attenuator() + 8));
        for (reg, saved) in ctl_regs.iter().zip(saved_ctl) {
            reg_write(*reg, saved & !ADC_CTL_PIN_BITS);
        }

        gpio::clear_fmux(pin);
        match input {
            Input::Analog => {
                gpio::set_pull(pin, gpio::Pull::Floating);
                reg_write(
                    IOMUX_ANALOG_IO_EN,
                    analog_io | (1 << channel.analog_io_bit()),
                );
            }
            Input::DrivenHigh => {
                gpio::write(pin, true);
                gpio::set_output(pin);
            }
        }
        reg_write(IOMUX_PAD_PS0, pad_ps0 | pin_bit);

        reg_write(ADCC_INTR_CLEAR, 0x1FF);
        reg_write(ADCC_INTR_MASK, slot_bit);
        reg_write(ctl_reg, reg_read(ctl_reg) | (1 << ctl_bit));
        reg_write(
            PCRM_ANA_CTL,
            (ana_ctl & !ANA_CTL_MICBIAS) | ANA_CTL_ADC_EN | ANA_CTL_ANALOG_LDO,
        );

        let complete = (0..CONVERSION_POLLS).any(|_| reg_read(ADCC_INTR_STATUS) & slot_bit != 0);
        let result = if complete {
            let buffer = ADC_CH_BASE + channel.result_slot() * 0x80;
            Ok((0..SAMPLE_WORDS)
                .map(|n| {
                    let word = reg_read(buffer + (n + 2) * 4);
                    (word & 0xFFF) + ((word >> 16) & 0xFFF)
                })
                .sum())
        } else {
            Err(AdcError::ConversionTimeout)
        };

        // Power down first, then restore everything in reverse order.
        reg_write(PCRM_ANA_CTL, ana_ctl);
        reg_write(ADCC_INTR_MASK, intr_mask);
        reg_write(ADCC_INTR_CLEAR, 0x1FF);
        reg_write(AON_PMCTL2_1, pmctl);
        for (reg, saved) in ctl_regs.iter().zip(saved_ctl) {
            reg_write(*reg, saved);
        }
        reg_write(PCRM_ADC_CTL4, ctl4);
        reg_write(
            GPIO_SWPORTA_DR,
            (reg_read(GPIO_SWPORTA_DR) & !pin_bit) | (dr & pin_bit),
        );
        reg_write(
            GPIO_SWPORTA_DDR,
            (reg_read(GPIO_SWPORTA_DDR) & !pin_bit) | (ddr & pin_bit),
        );
        reg_write(IOMUX_PAD_PS0, pad_ps0);
        reg_write(IOMUX_ANALOG_IO_EN, analog_io);
        reset_adcc();
        reg_write(PCR_SW_CLK, sw_clk);
        reg_write(PCRM_CLKHF_CTL1, clkhf1);
        reg_write(PCRM_CLKHF_CTL0, clkhf0);
        reg_write(PCRM_CLKSEL, clksel);
        result
    }
}

/// SDK `hal_clk_reset(MOD_ADCC)`: pulse the ADCC bit in `SW_RESET0`.
fn reset_adcc() {
    let reset = reg_read(PCR_SW_RESET0);
    reg_write(PCR_SW_RESET0, reset & !MOD_ADCC_BIT);
    reg_write(PCR_SW_RESET0, reset | MOD_ADCC_BIT);
}

const _: () = {
    // Paired result slots (SDK `ch2 = ch odd ? ch - 1 : ch + 1`).
    assert!(Channel::P11.result_slot() == 3);
    assert!(Channel::P14.result_slot() == 4);
    assert!(Channel::P20.result_slot() == 6);
    // Analog_IO numbering from SDK `gpio_pin_e`.
    assert!(Channel::P11.analog_io_bit() == 0);
    assert!(Channel::P14.analog_io_bit() == 1);
    assert!(Channel::P20.analog_io_bit() == 6);
    assert!(Channel::P24.analog_io_bit() == 8);
    // Worst-case sum cannot overflow the conversion.
    assert!(SAMPLE_COUNT * 4_095 <= u32::MAX / 4_520);
    // Full scale is about 3.6 V on the 3.2 V range (THB2: 57,120 / 65,536
    // mV per LSB, i.e. 3,570 mV); a 3.3 V supply is in range.
    assert!(sum_to_millivolts(Channel::P14, SAMPLE_COUNT * 4_095, SAMPLE_COUNT) == 3_585);
    // PB-03F on USB (3.3 V LDO) read 0x0EC0 = 3776 per sample on P14.
    assert!(sum_to_millivolts(Channel::P14, SAMPLE_COUNT * 3_776, SAMPLE_COUNT) == 3_306);
};
