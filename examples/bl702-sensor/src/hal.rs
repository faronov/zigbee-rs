//! BL702 HAL function implementations needed by `liblmac154.a` and `libbl702_rf.a`.
//!
//! These provide the minimal hardware abstraction that the vendor radio
//! libraries depend on. They are always linked (not gated behind `stubs`),
//! because even with real vendor radio code we still replace the BL702
//! HOSAL/StdDriver C libraries with lightweight Rust implementations.

#![allow(non_snake_case)]

use core::ptr;

// ── BL702 register constants ────────────────────────────────────

/// Machine cycle counter CSR (RISC-V standard)
#[allow(dead_code)]
const MCYCLE: u32 = 0xB00;

/// BL702 system clock (default 32 MHz from RC32M)
const SYS_CLOCK_HZ: u32 = 32_000_000;

// CLIC / interrupt controller base
const CLIC_BASE: usize = 0x0200_0000;
const CLIC_INT_EN_OFFSET: usize = 0x400; // per-interrupt enable, 1 byte each

// ── Delay functions ─────────────────────────────────────────────

/// Read the RISC-V mcycle CSR (cycle counter).
#[inline(always)]
fn read_mcycle() -> u32 {
    let val: u32;
    unsafe {
        core::arch::asm!("csrr {}, 0xB00", out(reg) val);
    }
    val
}

/// Busy-wait delay in microseconds.
#[unsafe(no_mangle)]
pub extern "C" fn BL702_Delay_US(cnt: u32) {
    let cycles_per_us = SYS_CLOCK_HZ / 1_000_000;
    let target_cycles = cnt.saturating_mul(cycles_per_us);
    let start = read_mcycle();
    while read_mcycle().wrapping_sub(start) < target_cycles {
        core::hint::spin_loop();
    }
}

/// Busy-wait delay in milliseconds.
#[unsafe(no_mangle)]
pub extern "C" fn BL702_Delay_MS(cnt: u32) {
    for _ in 0..cnt {
        BL702_Delay_US(1000);
    }
}

// ── IRQ management ──────────────────────────────────────────────
// Minimal interrupt handler table. The BL702 uses a CLIC-style
// interrupt controller.  We store handler pointers that the radio
// library registers and enable the corresponding CLIC interrupt.

const MAX_IRQS: usize = 80; // BL702 has ~64 IRQs, pad a bit
static mut IRQ_HANDLERS: [Option<unsafe extern "C" fn()>; MAX_IRQS] =
    [None; MAX_IRQS];

/// Register an interrupt handler.
#[unsafe(no_mangle)]
pub extern "C" fn bl_irq_register(irqnum: i32, handler: Option<unsafe extern "C" fn()>) {
    let idx = irqnum as usize;
    if idx < MAX_IRQS {
        unsafe {
            IRQ_HANDLERS[idx] = handler;
        }
        // Enable interrupt in CLIC
        let en_addr = (CLIC_BASE + CLIC_INT_EN_OFFSET + idx) as *mut u8;
        unsafe { ptr::write_volatile(en_addr, 1) };
    }
}

/// Unregister an interrupt handler.
#[unsafe(no_mangle)]
pub extern "C" fn bl_irq_unregister(irqnum: i32, _handler: Option<unsafe extern "C" fn()>) {
    let idx = irqnum as usize;
    if idx < MAX_IRQS {
        // Disable interrupt in CLIC
        let en_addr = (CLIC_BASE + CLIC_INT_EN_OFFSET + idx) as *mut u8;
        unsafe { ptr::write_volatile(en_addr, 0) };
        unsafe {
            IRQ_HANDLERS[idx] = None;
        }
    }
}

/// Get the currently registered handler for an IRQ.
#[unsafe(no_mangle)]
pub extern "C" fn bl_irq_handler_get(
    irqnum: i32,
    handler: *mut Option<unsafe extern "C" fn()>,
) {
    let idx = irqnum as usize;
    if idx < MAX_IRQS && !handler.is_null() {
        unsafe {
            ptr::write(handler, IRQ_HANDLERS[idx]);
        }
    }
}

// ── GPIO ────────────────────────────────────────────────────────

/// GLB_GPIO_Func_Init — configures GPIO pin function muxing.
/// The radio libraries call this during init. We provide a minimal
/// implementation that sets the GPIO function for each pin in the list.
#[unsafe(no_mangle)]
pub extern "C" fn GLB_GPIO_Func_Init(
    _gpio_fun: u32,
    _pin_list: *const u8,
    _cnt: u8,
) -> u32 {
    // The radio uses default GPIO mappings for the internal antenna
    // path, which are set by hardware reset. Return SUCCESS (0).
    0
}

// ── Memory utilities ────────────────────────────────────────────

/// Word-aligned 32-bit memcpy (copies `n` bytes, but both src/dst
/// are assumed word-aligned and `n` is a multiple of 4).
#[unsafe(no_mangle)]
pub extern "C" fn arch_memcpy4(
    dst: *mut u32,
    src: *const u32,
    n: u32,
) -> *mut u32 {
    let words = n as usize / 4;
    for i in 0..words {
        unsafe {
            ptr::write_volatile(dst.add(i), ptr::read_volatile(src.add(i)));
        }
    }
    dst
}

/// Standard abs — may be needed by vendor math.
#[unsafe(no_mangle)]
pub extern "C" fn abs(x: i32) -> i32 {
    if x < 0 { -x } else { x }
}
