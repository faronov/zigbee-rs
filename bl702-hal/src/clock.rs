//! BL702 peripheral clock gates and reset helpers.

use crate::mmio::{read32, rmw, write32};

const GLB_BASE: u32 = 0x4000_0000;
const HBN_BASE: u32 = 0x4000_f000;
const CLK_CFG2: u32 = GLB_BASE + 0x008;
const CLK_CFG3: u32 = GLB_BASE + 0x00c;
const SWRST_CFG1: u32 = GLB_BASE + 0x014;
const CGEN_CFG1: u32 = GLB_BASE + 0x024;
const HBN_GLOBAL: u32 = HBN_BASE + 0x030;

pub const BOOT_FCLK_HZ: u32 = 32_000_000;

/// Clock frequencies established by the BL702 ROM boot path used for XIP
/// applications.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Clocks {
    fclk_hz: u32,
    bclk_hz: u32,
    xclk_hz: u32,
}

impl Clocks {
    /// Frequencies used by the proven XT-ZB1 ROM-booted XIP path.
    pub const fn rom_boot_32mhz() -> Self {
        Self {
            fclk_hz: BOOT_FCLK_HZ,
            bclk_hz: BOOT_FCLK_HZ,
            xclk_hz: BOOT_FCLK_HZ,
        }
    }

    pub const fn fclk_hz(self) -> u32 {
        self.fclk_hz
    }

    pub const fn bclk_hz(self) -> u32 {
        self.bclk_hz
    }

    pub const fn xclk_hz(self) -> u32 {
        self.xclk_hz
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum Peripheral {
    Gpip = 2,
    Uart0 = 16,
    Spi = 18,
    I2c = 19,
    Pwm = 20,
    Timer = 21,
}

pub(crate) fn enable_and_reset(peripheral: Peripheral) {
    with_shared_registers(|| {
        let bit = 1 << peripheral as u8;
        rmw(CGEN_CFG1, bit, bit);
        write32(SWRST_CFG1, read32(SWRST_CFG1) & !bit);
        reset_dummy_wait();
        write32(SWRST_CFG1, read32(SWRST_CFG1) | bit);
        reset_dummy_wait();
        write32(SWRST_CFG1, read32(SWRST_CFG1) & !bit);
    });
}

pub(crate) fn configure_uart_fclk(divider: u8) {
    debug_assert!(divider <= 7);
    with_shared_registers(|| {
        rmw(CLK_CFG2, 1 << 4, 0);
        rmw(CLK_CFG2, 0x7, u32::from(divider));
        // HBN UART clock source 0 is FCLK.
        rmw(HBN_GLOBAL, 1 << 2, 0);
        rmw(CLK_CFG2, 1 << 4, 1 << 4);
    });
}

pub(crate) fn configure_i2c_clock(divider: u8) {
    with_shared_registers(|| {
        rmw(CLK_CFG3, 0xff << 16, u32::from(divider) << 16);
        rmw(CLK_CFG3, 1 << 24, 1 << 24);
    });
}

pub(crate) fn configure_spi_clock(divider: u8) {
    debug_assert!(divider <= 31);
    with_shared_registers(|| {
        rmw(CLK_CFG3, 0x1f, u32::from(divider));
        rmw(CLK_CFG3, 1 << 8, 1 << 8);
    });
}

fn with_shared_registers<R>(operation: impl FnOnce() -> R) -> R {
    #[cfg(target_arch = "riscv32")]
    {
        riscv::interrupt::free(operation)
    }

    #[cfg(not(target_arch = "riscv32"))]
    {
        operation()
    }
}

#[inline(always)]
fn reset_dummy_wait() {
    #[cfg(target_arch = "riscv32")]
    // SAFETY: These four NOPs reproduce the BL702 SDK's required delay
    // between AHB peripheral reset-register transitions.
    unsafe {
        core::arch::asm!(
            "nop",
            "nop",
            "nop",
            "nop",
            options(nomem, nostack, preserves_flags)
        );
    }

    #[cfg(not(target_arch = "riscv32"))]
    for _ in 0..4 {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proven_boot_clocks_are_32mhz() {
        let clocks = Clocks::rom_boot_32mhz();
        assert_eq!(clocks.fclk_hz(), 32_000_000);
        assert_eq!(clocks.bclk_hz(), 32_000_000);
        assert_eq!(clocks.xclk_hz(), 32_000_000);
    }
}
