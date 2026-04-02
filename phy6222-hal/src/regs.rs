//! PHY6222 register addresses and constants.
//!
//! Derived from `mcu_phy_bumbee.h` in the PHY6222 SDK.

// ── Peripheral base addresses ──────────────────────────────────

pub const AP_PCR_BASE: u32 = 0x4000_0000;
pub const AP_WDT_BASE: u32 = 0x4000_2000;
pub const AP_IOMUX_BASE: u32 = 0x4000_3800;
pub const AP_UART0_BASE: u32 = 0x4000_4000;
pub const AP_I2C0_BASE: u32 = 0x4000_5000;
pub const AP_I2C1_BASE: u32 = 0x4000_5800;
pub const AP_SPI0_BASE: u32 = 0x4000_6000;
pub const AP_GPIO_BASE: u32 = 0x4000_8000;
pub const AP_UART1_BASE: u32 = 0x4000_9000;
pub const AP_CACHE_BASE: u32 = 0x4000_C000;
pub const AP_SPIF_BASE: u32 = 0x4000_C800;
pub const AP_AON_BASE: u32 = 0x4000_F000;
pub const AP_PCRM_BASE: u32 = 0x4000_F03C;
pub const ADCC_BASE: u32 = 0x4005_0000;
pub const ADC_CH_BASE: u32 = 0x4005_0400;
pub const FLASH_BASE: u32 = 0x1100_0000;

// ── GPIO registers (base + offset) ────────────────────────────

pub const GPIO_SWPORTA_DR: u32 = AP_GPIO_BASE + 0x00;
pub const GPIO_SWPORTA_DDR: u32 = AP_GPIO_BASE + 0x04;
pub const GPIO_EXT_PORTA: u32 = AP_GPIO_BASE + 0x50;
pub const GPIO_INTEN: u32 = AP_GPIO_BASE + 0x30;
pub const GPIO_INTMASK: u32 = AP_GPIO_BASE + 0x34;
pub const GPIO_INTTYPE_LEVEL: u32 = AP_GPIO_BASE + 0x38;
pub const GPIO_INT_POLARITY: u32 = AP_GPIO_BASE + 0x3C;
pub const GPIO_INT_STATUS: u32 = AP_GPIO_BASE + 0x40;
pub const GPIO_PORTA_EOI: u32 = AP_GPIO_BASE + 0x4C;

// ── I2C registers (DesignWare IP) ──────────────────────────────

pub const I2C_IC_CON: u32 = 0x00;
pub const I2C_IC_TAR: u32 = 0x04;
pub const I2C_IC_DATA_CMD: u32 = 0x10;
pub const I2C_IC_SS_SCL_HCNT: u32 = 0x14;
pub const I2C_IC_SS_SCL_LCNT: u32 = 0x18;
pub const I2C_IC_FS_SCL_HCNT: u32 = 0x1C;
pub const I2C_IC_FS_SCL_LCNT: u32 = 0x20;
pub const I2C_IC_INTR_MASK: u32 = 0x30;
pub const I2C_IC_RAW_INTR_STAT: u32 = 0x34;
pub const I2C_IC_RX_TL: u32 = 0x38;
pub const I2C_IC_TX_TL: u32 = 0x3C;
pub const I2C_IC_CLR_TX_ABRT: u32 = 0x54;
pub const I2C_IC_ENABLE: u32 = 0x6C;
pub const I2C_IC_STATUS: u32 = 0x70;
pub const I2C_IC_TXFLR: u32 = 0x74;
pub const I2C_IC_RXFLR: u32 = 0x78;

// I2C status bits
pub const I2C_STATUS_RFNE: u32 = 0x08;
pub const I2C_STATUS_TFE: u32 = 0x04;
pub const I2C_STATUS_TFNF: u32 = 0x02;

// ── SPIF (flash controller) registers ──────────────────────────

pub const SPIF_CONFIG: u32 = AP_SPIF_BASE + 0x00;
pub const SPIF_FCMD: u32 = AP_SPIF_BASE + 0x90;
pub const SPIF_FCMD_ADDR: u32 = AP_SPIF_BASE + 0x94;
pub const SPIF_FCMD_RDDATA: u32 = AP_SPIF_BASE + 0xA0;
pub const SPIF_FCMD_WRDATA: u32 = AP_SPIF_BASE + 0xA8;

// ── Cache control ──────────────────────────────────────────────

pub const CACHE_CTRL0: u32 = AP_CACHE_BASE + 0x00;
pub const CACHE_BYPASS_REG: u32 = 0x4000_0044;

// ── PCRM (power/clock/ADC control) ────────────────────────────

pub const PCRM_CLKSEL: u32 = AP_PCRM_BASE + 0x00;
pub const PCRM_CLKHF_CTL0: u32 = AP_PCRM_BASE + 0x04;
pub const PCRM_CLKHF_CTL1: u32 = AP_PCRM_BASE + 0x08;
pub const PCRM_ANA_CTL: u32 = AP_PCRM_BASE + 0x0C;
pub const PCRM_ADC_CTL0: u32 = AP_PCRM_BASE + 0x30;
pub const PCRM_ADC_CTL4: u32 = AP_PCRM_BASE + 0x40;

// ── AON (always-on domain) ─────────────────────────────────────

pub const AON_IOCTL0: u32 = AP_AON_BASE + 0x08;
pub const AON_IOCTL1: u32 = AP_AON_BASE + 0x0C;
pub const AON_IOCTL2: u32 = AP_AON_BASE + 0x10;
pub const AON_PMCTL0: u32 = AP_AON_BASE + 0x14;
pub const AON_PMCTL2_1: u32 = AP_AON_BASE + 0x20;

// ── Clock gating (AP_PCR->SW_CLK) ─────────────────────────────

pub const PCR_SW_CLK: u32 = AP_PCR_BASE;

pub const MOD_IOMUX_BIT: u32 = 1 << 7;
pub const MOD_I2C0_BIT: u32 = 1 << 9;
pub const MOD_I2C1_BIT: u32 = 1 << 10;
pub const MOD_GPIO_BIT: u32 = 1 << 13;
pub const MOD_ADCC_BIT: u32 = 1 << 17;
pub const MOD_SPIF_BIT: u32 = 1 << 19;

// ── IOMUX function mux values ──────────────────────────────────

pub const FMUX_IIC0_SCL: u8 = 0;
pub const FMUX_IIC0_SDA: u8 = 1;
pub const FMUX_IIC1_SCL: u8 = 2;
pub const FMUX_IIC1_SDA: u8 = 3;
pub const FMUX_UART0_TX: u8 = 4;
pub const FMUX_UART0_RX: u8 = 5;
pub const FMUX_UART1_TX: u8 = 8;
pub const FMUX_UART1_RX: u8 = 9;
pub const FMUX_PWM0: u8 = 10;
pub const FMUX_SPI0_SCK: u8 = 16;

// ── Register access helpers ────────────────────────────────────

#[inline(always)]
pub fn reg_write(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) };
}

#[inline(always)]
pub fn reg_read(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Read-modify-write a bitfield: reg[hi:lo] = value.
#[inline(always)]
pub fn reg_set_bits(addr: u32, hi: u8, lo: u8, value: u32) {
    let mask = ((1u32 << (hi - lo + 1)) - 1) << lo;
    let old = reg_read(addr);
    reg_write(addr, (old & !mask) | ((value << lo) & mask));
}
