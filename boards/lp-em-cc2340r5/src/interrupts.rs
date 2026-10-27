//! Device interrupt dispatch for the CC2340R52.
//!
//! `cortex-m-rt` is used without a device crate, so every device vector
//! points at `DefaultHandler`; this module owns it and dispatches by IRQ
//! number (CMSIS numbering, i.e. `hw_ints.h` `INT_*` minus 16).

use cortex_m::interrupt::InterruptNumber;
use cortex_m_rt::exception;
use portable_atomic::{AtomicU32, Ordering};

/// Device interrupts used by this board crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum Irq {
    /// `INT_GPIO_COMB` (21): combined GPIO edge interrupt.
    GpioComb = 5,
    /// `INT_CPUIRQ16` (32): EVTSVT-routed; SYSTIM channel 0 here.
    CpuIrq16 = 16,
}

// SAFETY: the discriminants are valid CC2340R52 NVIC interrupt numbers.
unsafe impl InterruptNumber for Irq {
    fn number(self) -> u16 {
        self as u16
    }
}

static UNEXPECTED: AtomicU32 = AtomicU32::new(0);

/// Number of device interrupts that fired without a handler (each is masked
/// on first occurrence).
pub fn unexpected_count() -> u32 {
    UNEXPECTED.load(Ordering::Relaxed)
}

pub(crate) fn enable(irq: Irq) {
    cortex_m::peripheral::NVIC::unpend(irq);
    // SAFETY: the handlers below are installed for every `Irq` variant and
    // only touch state guarded by critical sections or atomics.
    unsafe { cortex_m::peripheral::NVIC::unmask(irq) };
}

pub(crate) fn unpend(irq: Irq) {
    cortex_m::peripheral::NVIC::unpend(irq);
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    match irqn {
        5 => crate::pins::on_gpio_interrupt(),
        16 => crate::time::on_interrupt(),
        n if n >= 0 => {
            // Mask the unexpected source instead of hanging the device.
            const NVIC_ICER: *mut u32 = 0xE000_E180 as *mut u32;
            unsafe { core::ptr::write_volatile(NVIC_ICER, 1 << (n as u32 & 31)) };
            UNEXPECTED.fetch_add(1, Ordering::Relaxed);
        }
        _ => loop {
            cortex_m::asm::nop();
        },
    }
}
