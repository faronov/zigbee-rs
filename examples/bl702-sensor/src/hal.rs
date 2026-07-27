//! Minimal XT-ZB1 peripherals used by the Zigbee sensor.

use core::hint::spin_loop;

const SYSTEM_CLOCK_HZ: u32 = 32_000_000;

const GLB_BASE: u32 = 0x4000_0000;
const HBN_BASE: u32 = 0x4000_f000;
const EF_DATA_BASE: u32 = 0x4000_7000;
const UART0_BASE: u32 = 0x4000_a000;
const TIMER_BASE: u32 = 0x4000_a500;

const GLB_CLK_CFG2: u32 = GLB_BASE + 0x008;
const GLB_UART_SIG_SEL: u32 = GLB_BASE + 0x0c0;
const GLB_GPIO_14_15_CFG: u32 = GLB_BASE + 0x11c;
const GLB_GPIO_OUTPUT_ENABLE: u32 = GLB_BASE + 0x190;
const HBN_GLOBAL: u32 = HBN_BASE + 0x030;
const EF_WIFI_MAC_LOW: u32 = EF_DATA_BASE + 0x014;
const EF_WIFI_MAC_HIGH: u32 = EF_DATA_BASE + 0x018;

const UART_TX_CONFIG: u32 = UART0_BASE;
const UART_RX_CONFIG: u32 = UART0_BASE + 0x004;
const UART_BIT_PERIOD: u32 = UART0_BASE + 0x008;
const UART_FIFO_CONFIG_0: u32 = UART0_BASE + 0x080;
const UART_FIFO_CONFIG_1: u32 = UART0_BASE + 0x084;
const UART_FIFO_WRITE: u32 = UART0_BASE + 0x088;

const TIMER_CLOCK_CONFIG: u32 = TIMER_BASE;
const TIMER_COMPARE_0: u32 = TIMER_BASE + 0x010;
const TIMER_COMPARE_1: u32 = TIMER_BASE + 0x014;
const TIMER_COMPARE_2: u32 = TIMER_BASE + 0x018;
const TIMER_COUNTER: u32 = TIMER_BASE + 0x02c;
const TIMER_INTERRUPT_ENABLE: u32 = TIMER_BASE + 0x044;
const TIMER_PRELOAD_CONTROL: u32 = TIMER_BASE + 0x05c;
const TIMER_INTERRUPT_CLEAR: u32 = TIMER_BASE + 0x078;
const TIMER_ENABLE: u32 = TIMER_BASE + 0x084;
const TIMER_COUNT_MODE: u32 = TIMER_BASE + 0x088;
const TIMER_CLOCK_DIVIDER: u32 = TIMER_BASE + 0x0bc;

pub fn init() {
    init_uart();
    init_timer();
}

pub fn delay_us(duration_us: u32) {
    let target_cycles = duration_us.saturating_mul(SYSTEM_CLOCK_HZ / 1_000_000);
    let started = cycle_count();
    while cycle_count().wrapping_sub(started) < target_cycles {
        spin_loop();
    }
}

pub fn timer_ticks() -> u32 {
    read32(TIMER_COUNTER)
}

/// Read the eight-byte factory chip identifier loaded from eFuse by the boot ROM.
pub fn chip_id() -> [u8; 8] {
    let low = read32(EF_WIFI_MAC_LOW).to_le_bytes();
    let high = read32(EF_WIFI_MAC_HIGH).to_le_bytes();
    [
        low[0], low[1], low[2], low[3], high[0], high[1], high[2], high[3],
    ]
}

pub fn uart_write(byte: u8) {
    while read32(UART_FIFO_CONFIG_1) & 0xff == 0 {
        spin_loop();
    }
    write32(UART_FIFO_WRITE, u32::from(byte));
}

fn init_uart() {
    // UART0 clock: 32 MHz FCLK, divider 1.
    rmw(GLB_CLK_CFG2, 1 << 4, 0);
    rmw(GLB_CLK_CFG2, 0x7, 0);
    rmw(HBN_GLOBAL, 1 << 2, 0);
    rmw(GLB_CLK_CFG2, 1 << 4, 1 << 4);

    // XT-ZB1 UART0: TX GPIO14, RX GPIO15.
    rmw(GLB_GPIO_OUTPUT_ENABLE, (1 << 14) | (1 << 15), 0);
    write32(GLB_GPIO_14_15_CFG, 0x0717_0717);
    rmw(GLB_UART_SIG_SEL, 0xff00_0000, 0x3200_0000);

    write32(UART_TX_CONFIG, 0);
    write32(UART_RX_CONFIG, 0);
    write32(UART_BIT_PERIOD, 0x000f_000f);
    write32(UART_FIFO_CONFIG_0, 0x0c);
    write32(UART_FIFO_CONFIG_0, 0);
    write32(UART_FIFO_CONFIG_1, 0x0f0f_0000);
    write32(UART_TX_CONFIG, 0x0000_0f05);
    write32(UART_RX_CONFIG, 0x0000_0701);
}

fn init_timer() {
    // TIMER_CH0: 32 MHz FCLK / 32 = 1 MHz, free-running.
    rmw(TIMER_ENABLE, 1 << 1, 0);
    rmw(TIMER_CLOCK_CONFIG, 0x3 << 2, 0);
    rmw(TIMER_CLOCK_DIVIDER, 0xff << 8, 31 << 8);
    rmw(TIMER_COUNT_MODE, 1 << 1, 1 << 1);
    write32(TIMER_PRELOAD_CONTROL, 0);
    write32(TIMER_COMPARE_0, u32::MAX - 2);
    write32(TIMER_COMPARE_1, u32::MAX - 2);
    write32(TIMER_COMPARE_2, u32::MAX - 2);
    write32(TIMER_INTERRUPT_ENABLE, 0);
    write32(TIMER_INTERRUPT_CLEAR, 0x7);
    rmw(TIMER_ENABLE, 1 << 1, 1 << 1);
}

#[inline(always)]
fn cycle_count() -> u32 {
    let value: u32;
    unsafe {
        core::arch::asm!("csrr {}, mcycle", out(reg) value);
    }
    value
}

#[inline(always)]
fn read32(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline(always)]
fn write32(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[inline(always)]
fn rmw(address: u32, mask: u32, value: u32) {
    write32(address, (read32(address) & !mask) | (value & mask));
}
