//! System clock (HCLK) selection.
//!
//! The boot ROM leaves the core on whatever clock the ROM chose (32 MHz on
//! the PB-03F factory image). Board time drivers and the radio's LL timing
//! need the real HCLK, so boards call [`init_dll48m`] once at startup and
//! derive every rate from [`hclk_hz`].
//!
//! The switch sequence follows the PHYPLUS SDK `clk_init` /
//! `clk_apply_setting` (`lib/rf/patch.c`), restricted to the DLL 48 MHz case
//! with the SPIF reference clock left unchanged.

use crate::regs::*;

/// `CLKSEL[2:0]` HCLK source encodings (SDK `sysclk_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SysClk {
    Rc32M = 0,
    Dll32M = 1,
    Xtal16M = 2,
    Dll48M = 3,
    Dll64M = 4,
    Dll96M = 5,
    Clk8M = 6,
    Clk4M = 7,
}

impl SysClk {
    pub const fn from_bits(bits: u32) -> Self {
        match bits & 7 {
            0 => Self::Rc32M,
            1 => Self::Dll32M,
            2 => Self::Xtal16M,
            3 => Self::Dll48M,
            4 => Self::Dll64M,
            5 => Self::Dll96M,
            6 => Self::Clk8M,
            _ => Self::Clk4M,
        }
    }

    /// Nominal frequency (SDK `g_hclk_table`).
    pub const fn hz(self) -> u32 {
        match self {
            Self::Rc32M | Self::Dll32M => 32_000_000,
            Self::Xtal16M => 16_000_000,
            Self::Dll48M => 48_000_000,
            Self::Dll64M => 64_000_000,
            Self::Dll96M => 96_000_000,
            Self::Clk8M => 8_000_000,
            Self::Clk4M => 4_000_000,
        }
    }
}

/// Currently selected HCLK source.
pub fn sysclk() -> SysClk {
    SysClk::from_bits(reg_read(PCRM_CLKSEL))
}

/// Current HCLK frequency in Hz.
pub fn hclk_hz() -> u32 {
    sysclk().hz()
}

/// Current HCLK frequency in MHz (the `H` factor of the LL timing registers).
pub fn hclk_mhz() -> u32 {
    hclk_hz() / 1_000_000
}

/// `CLKHF_CTL0[18]`: route the 16 MHz crystal to the digital domain.
const CTL0_XTAL_TO_DIGITAL: u32 = 1 << 18;
/// `CLKSEL[4:3]`: DMA and AES clocks (SDK `_CLK_AES | _CLK_DMA`).
const CLKSEL_DMA_AES: u32 = 0x18;
/// `CLKHF_CTL1[7]`: DLL enable.
const CTL1_DLL_EN: u32 = 1 << 7;
/// `CLKHF_CTL1[14]`: 48 MHz digital clock enable.
const CTL1_DIGCLK_48M: u32 = 1 << 14;
/// SDK `_CLK_SPIF | _CLK_CK802_CPU`.
const SW_CLK_MIN: u32 = 0x8_0001;
/// SDK `_CLK_WDT | _CLK_M0_CPU`.
const SW_CLK1_MIN: u32 = 0x21;

/// Switch HCLK to the DLL 48 MHz output if it is not already selected.
///
/// Additive to the ROM's clock tree: the doubler and 32 MHz digital clock
/// stay enabled because the radio's RX ADC uses the 32 MHz doubler path.
/// Must run before any peripheral derives a baud rate or timer reload from
/// HCLK.
pub fn init_dll48m() {
    if sysclk() == SysClk::Dll48M {
        return;
    }
    cortex_m::interrupt::free(|_| {
        // Settling delay used by `clk_init` (`g_hclk >> 15` loop iterations).
        let mut delay = hclk_hz() >> 15;
        while delay > 0 {
            cortex_m::asm::nop();
            delay -= 1;
        }

        let ctl0 = reg_read(PCRM_CLKHF_CTL0);
        if ctl0 & CTL0_XTAL_TO_DIGITAL == 0 {
            reg_write(PCRM_CLKHF_CTL0, ctl0 | CTL0_XTAL_TO_DIGITAL);
        }
        reg_write(PCRM_CLKSEL, reg_read(PCRM_CLKSEL) | CLKSEL_DMA_AES);

        // Gate peripherals to the minimum set while the source changes.
        let sw_clk = reg_read(PCR_SW_CLK);
        let sw_clk1 = reg_read(PCR_SW_CLK1);
        reg_write(PCR_SW_CLK, SW_CLK_MIN);
        reg_write(PCR_SW_CLK1, SW_CLK1_MIN);

        reg_write(
            PCRM_CLKHF_CTL1,
            reg_read(PCRM_CLKHF_CTL1) | CTL1_DLL_EN | CTL1_DIGCLK_48M,
        );
        for _ in 0..64 {
            cortex_m::asm::nop();
        }
        let clksel = reg_read(PCRM_CLKSEL);
        reg_write(PCRM_CLKSEL, (clksel & !7) | SysClk::Dll48M as u32);

        reg_write(PCR_SW_CLK, sw_clk | SW_CLK_MIN);
        reg_write(PCR_SW_CLK1, sw_clk1 | 1);
    });
}

const _: () = assert!(SysClk::from_bits(0x59).hz() == 32_000_000);
const _: () = assert!(SysClk::from_bits(3).hz() == 48_000_000);
