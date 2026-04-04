//! GPIO, IOMUX pin mux, and AON pull-up/down configuration.
//!
//! PHY6222 has 23 GPIO pins (non-contiguous: P0-P3, P7, P9-P11, P14-P18,
//! P20, P23-P27, P31-P34). Pin enum values 0-22 map to these physical pins.

use crate::regs::*;

/// Set a pin as output.
pub fn set_output(pin: u8) {
    let ddr = reg_read(GPIO_SWPORTA_DDR);
    reg_write(GPIO_SWPORTA_DDR, ddr | (1 << pin));
}

/// Set a pin as input.
pub fn set_input(pin: u8) {
    let ddr = reg_read(GPIO_SWPORTA_DDR);
    reg_write(GPIO_SWPORTA_DDR, ddr & !(1 << pin));
}

/// Write a pin output value.
pub fn write(pin: u8, high: bool) {
    let dr = reg_read(GPIO_SWPORTA_DR);
    if high {
        reg_write(GPIO_SWPORTA_DR, dr | (1 << pin));
    } else {
        reg_write(GPIO_SWPORTA_DR, dr & !(1 << pin));
    }
}

/// Read a pin input value.
pub fn read(pin: u8) -> bool {
    (reg_read(GPIO_EXT_PORTA) >> pin) & 1 == 1
}

/// Toggle a pin output.
pub fn toggle(pin: u8) {
    let dr = reg_read(GPIO_SWPORTA_DR);
    reg_write(GPIO_SWPORTA_DR, dr ^ (1 << pin));
}

/// Pull-up/down mode.
#[derive(Clone, Copy)]
pub enum Pull {
    Floating = 0,
    StrongPullUp = 1,
    WeakPullUp = 2,
    PullDown = 3,
}

/// Set pull-up/down for a pin.
///
/// Pull configuration is in AON IOCTL[0-2] and PMCTL0 registers.
/// Each pin has a 2-bit field at chip-specific positions.
pub fn set_pull(pin: u8, pull: Pull) {
    let (reg_offset, bit_h, bit_l) = match pin {
        0  => (AON_IOCTL0, 2, 1),
        1  => (AON_IOCTL0, 5, 4),
        2  => (AON_IOCTL0, 8, 7),
        3  => (AON_IOCTL0, 11, 10),
        4  => (AON_IOCTL0, 23, 22),  // P7
        5  => (AON_IOCTL0, 29, 28),  // P9
        6  => (AON_IOCTL1, 2, 1),    // P10
        7  => (AON_IOCTL1, 5, 4),    // P11
        8  => (AON_IOCTL1, 14, 13),  // P14
        9  => (AON_IOCTL1, 17, 16),  // P15
        10 => (AON_IOCTL1, 20, 19),  // P16
        11 => (AON_IOCTL1, 23, 22),  // P17
        12 => (AON_IOCTL1, 26, 25),  // P18
        13 => (AON_IOCTL2, 2, 1),    // P20
        14 => (AON_IOCTL2, 11, 10),  // P23
        15 => (AON_IOCTL2, 14, 13),  // P24
        16 => (AON_IOCTL2, 17, 16),  // P25
        17 => (AON_IOCTL2, 20, 19),  // P26
        18 => (AON_IOCTL2, 23, 22),  // P27
        19 => (AON_PMCTL0, 5, 4),    // P31
        20 => (AON_PMCTL0, 8, 7),    // P32
        21 => (AON_PMCTL0, 11, 10),  // P33
        22 => (AON_PMCTL0, 14, 13),  // P34
        _ => return,
    };

    reg_set_bits(reg_offset, bit_h, bit_l, pull as u32);
}

/// Set IOMUX function for a pin.
///
/// Enables the full-mux path and assigns the given function code
/// (e.g., `FMUX_IIC0_SCL`, `FMUX_UART0_TX`).
pub fn set_fmux(pin: u8, fmux: u8) {
    // Enable IOMUX clock
    let sw_clk = reg_read(PCR_SW_CLK);
    reg_write(PCR_SW_CLK, sw_clk | MOD_IOMUX_BIT);

    // Write 6-bit mux field into gpio_sel register
    let reg_idx = (pin >> 2) as u32;
    let bit_idx = (pin & 3) as u32;
    let shift = bit_idx * 8;
    let mask = 0x3F << shift;

    let sel_addr = AP_IOMUX_BASE + 0x08 + reg_idx * 4;
    let old = reg_read(sel_addr);
    reg_write(sel_addr, (old & !mask) | ((fmux as u32) << shift));

    // Enable full-mux for this pin
    let mux_en_addr = AP_IOMUX_BASE + 0x00;
    let mux_en = reg_read(mux_en_addr);
    reg_write(mux_en_addr, mux_en | (1 << pin));
}

/// Disable IOMUX for a pin (return to GPIO mode).
pub fn clear_fmux(pin: u8) {
    let mux_en_addr = AP_IOMUX_BASE + 0x00;
    let mux_en = reg_read(mux_en_addr);
    reg_write(mux_en_addr, mux_en & !(1 << pin));
}

/// Prepare GPIOs for low-power sleep.
///
/// Sets all output pins high (LEDs off for active-low), then configures
/// all unused pins as inputs with pull-down to prevent leakage.
/// Only `keep_pins` (bitmask) are left as-is (e.g., button with pull-up).
pub fn prepare_for_sleep(keep_pins: u32) {
    let ddr = reg_read(GPIO_SWPORTA_DDR);
    // Turn off all output LEDs (active-low: high = off)
    reg_write(GPIO_SWPORTA_DR, reg_read(GPIO_SWPORTA_DR) | ddr);
    // Set non-kept pins to input with pull-down
    for pin in 0..23u8 {
        if (keep_pins >> pin) & 1 == 0 {
            set_input(pin);
            set_pull(pin, Pull::PullDown);
        }
    }
}
