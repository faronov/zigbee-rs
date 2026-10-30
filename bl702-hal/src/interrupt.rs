//! BL702 SiFive E24 CLIC interrupt mechanism.
//!
//! The BL702 core routes every interrupt through a CLIC. The standard RISC-V
//! `mie` CSR is ignored in CLIC mode; each source has its own enable and
//! pending byte. This module installs one 64-byte-aligned trap entry in
//! CLIC *direct* mode (`mtvec = base | 0b10`), so every trap enters the same
//! handler and the source is read from `mcause[9:0]`. That is the mode used by
//! the vendor FreeRTOS port and by the hardware-proven `bl602-hal` on the
//! identical E24 core; it avoids building and validating a `mtvt` vector
//! table.
//!
//! The level/priority configuration (`cliccfg`, `clicintcfg`) is left at its
//! reset value. Nested interrupts are not supported: the trap entry runs with
//! `mstatus.MIE` cleared by hardware until `mret`.
//!
//! The application owns policy: which sources are enabled and what the
//! installed handler does with them. Exceptions halt in a spin loop, matching
//! the previous `riscv-rt` default abort behavior.

/// Base of the hart-0 CLIC register block.
pub const CLIC_HART0: u32 = 0x0280_0000;
const CLIC_INTIP: u32 = CLIC_HART0;
const CLIC_INTIE: u32 = CLIC_HART0 + 0x400;

/// Number of CLIC source slots cleared during installation.
///
/// BL702 defines 16 core-local sources followed by its peripheral sources
/// (`IRQn_LAST` is below 96). Clearing 96 slots matches the vendor startup
/// and `bl602-hal`.
pub const CLIC_SOURCE_SLOTS: u16 = 96;

const IRQ_NUM_BASE: u16 = 16;

/// TIMER channel 0 (counter `TCR2`, comparators `TMR2_0..2`) interrupt.
pub const TIMER_CH0_IRQ: u16 = IRQ_NUM_BASE + 36;
/// TIMER channel 1 interrupt.
pub const TIMER_CH1_IRQ: u16 = IRQ_NUM_BASE + 37;

/// Mask selecting the CLIC exception/interrupt code from `mcause`.
const MCAUSE_CODE_MASK: u32 = 0x3ff;
const MCAUSE_INTERRUPT: u32 = 1 << 31;

/// Decoded `mcause` value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trap {
    /// CLIC interrupt source number.
    Interrupt(u16),
    /// Synchronous exception code.
    Exception(u16),
}

/// Decode a CLIC-mode `mcause` value.
///
/// In CLIC mode `mcause` also carries the previous privilege, previous
/// interrupt-enable, and previous interrupt level, so the code field must be
/// masked rather than taken from the low 31 bits.
pub const fn decode_mcause(mcause: u32) -> Trap {
    let code = (mcause & MCAUSE_CODE_MASK) as u16;
    if mcause & MCAUSE_INTERRUPT != 0 {
        Trap::Interrupt(code)
    } else {
        Trap::Exception(code)
    }
}

/// Interrupt handler installed by the application. It runs with interrupts
/// globally disabled and must clear the source before returning.
pub type Handler = fn(u16);

#[cfg(target_arch = "riscv32")]
mod imp {
    use core::sync::atomic::{AtomicUsize, Ordering};

    use super::{CLIC_INTIE, CLIC_INTIP, CLIC_SOURCE_SLOTS, Handler, Trap, decode_mcause};

    static HANDLER: AtomicUsize = AtomicUsize::new(0);

    // Caller-saved registers only: the Rust dispatcher follows the C ABI and
    // preserves s0-s11 itself. 16 words keep `sp` 16-byte aligned.
    core::arch::global_asm!(
        ".section .trap.bl702_clic, \"ax\"",
        ".balign 64",
        ".global _bl702_clic_trap",
        "_bl702_clic_trap:",
        "addi sp, sp, -64",
        "sw ra, 0(sp)",
        "sw t0, 4(sp)",
        "sw t1, 8(sp)",
        "sw t2, 12(sp)",
        "sw t3, 16(sp)",
        "sw t4, 20(sp)",
        "sw t5, 24(sp)",
        "sw t6, 28(sp)",
        "sw a0, 32(sp)",
        "sw a1, 36(sp)",
        "sw a2, 40(sp)",
        "sw a3, 44(sp)",
        "sw a4, 48(sp)",
        "sw a5, 52(sp)",
        "sw a6, 56(sp)",
        "sw a7, 60(sp)",
        "csrr a0, mcause",
        "call _bl702_clic_dispatch",
        "lw ra, 0(sp)",
        "lw t0, 4(sp)",
        "lw t1, 8(sp)",
        "lw t2, 12(sp)",
        "lw t3, 16(sp)",
        "lw t4, 20(sp)",
        "lw t5, 24(sp)",
        "lw t6, 28(sp)",
        "lw a0, 32(sp)",
        "lw a1, 36(sp)",
        "lw a2, 40(sp)",
        "lw a3, 44(sp)",
        "lw a4, 48(sp)",
        "lw a5, 52(sp)",
        "lw a6, 56(sp)",
        "lw a7, 60(sp)",
        "addi sp, sp, 64",
        "mret",
    );

    unsafe extern "C" {
        fn _bl702_clic_trap();
    }

    #[unsafe(no_mangle)]
    extern "C" fn _bl702_clic_dispatch(mcause: u32) {
        match decode_mcause(mcause) {
            Trap::Interrupt(irq) => {
                let handler = HANDLER.load(Ordering::Acquire);
                if handler == 0 {
                    // No owner: mask the source so it cannot storm.
                    super::disable(irq);
                    super::clear_pending(irq);
                } else {
                    // SAFETY: `HANDLER` only ever holds a value stored from a
                    // `Handler` function pointer in `install_direct_mode`.
                    let handler: Handler = unsafe { core::mem::transmute(handler) };
                    handler(irq);
                }
            }
            Trap::Exception(_) => loop {
                core::hint::spin_loop();
            },
        }
    }

    pub(super) unsafe fn install_direct_mode(handler: Handler) {
        // SAFETY (caller): `mstatus.MIE` is clear, so no trap can observe a
        // partially configured controller.
        for slot in 0..u32::from(CLIC_SOURCE_SLOTS / 4) {
            crate::mmio::write32(CLIC_INTIE + slot * 4, 0);
            crate::mmio::write32(CLIC_INTIP + slot * 4, 0);
        }
        HANDLER.store(handler as *const () as usize, Ordering::Release);
        let base = _bl702_clic_trap as *const () as usize;
        debug_assert_eq!(base & 0x3f, 0, "CLIC trap entry must be 64-byte aligned");
        // SAFETY: Writing `mtvec` selects CLIC direct mode with the aligned
        // trap entry defined above.
        unsafe {
            core::arch::asm!("csrw mtvec, {}", in(reg) base | 0b10);
        }
    }

    pub(super) fn write_byte(address: u32, value: u8) {
        // SAFETY: CLIC enable/pending bytes are byte-addressable MMIO.
        unsafe { core::ptr::write_volatile(address as *mut u8, value) }
    }

    pub(super) fn read_byte(address: u32) -> u8 {
        // SAFETY: CLIC enable/pending bytes are byte-addressable MMIO.
        unsafe { core::ptr::read_volatile(address as *const u8) }
    }
}

/// Install the CLIC direct-mode trap entry and the application handler.
///
/// All CLIC source enables and pending bits are cleared. Global interrupts
/// are not enabled; the caller does that after enabling its sources.
///
/// # Safety
///
/// Call once during single-threaded startup with `mstatus.MIE` clear. The
/// caller becomes the owner of every CLIC source and of `mtvec`.
#[cfg(target_arch = "riscv32")]
pub unsafe fn install_direct_mode(handler: Handler) {
    // SAFETY: Forwarded caller contract.
    unsafe { imp::install_direct_mode(handler) }
}

/// Enable delivery of one CLIC source.
///
/// # Safety
///
/// The installed handler must clear the source's peripheral status and CLIC
/// pending bit; otherwise the source can trap continuously.
pub unsafe fn enable(irq: u16) {
    if irq < CLIC_SOURCE_SLOTS {
        write_byte(CLIC_INTIE + u32::from(irq), 1);
    }
}

/// Stop delivery of one CLIC source.
pub fn disable(irq: u16) {
    if irq < CLIC_SOURCE_SLOTS {
        write_byte(CLIC_INTIE + u32::from(irq), 0);
    }
}

/// Clear one CLIC pending bit.
pub fn clear_pending(irq: u16) {
    if irq < CLIC_SOURCE_SLOTS {
        write_byte(CLIC_INTIP + u32::from(irq), 0);
    }
}

/// Return whether one CLIC source is pending.
pub fn is_pending(irq: u16) -> bool {
    irq < CLIC_SOURCE_SLOTS && read_byte(CLIC_INTIP + u32::from(irq)) & 1 != 0
}

#[cfg(target_arch = "riscv32")]
fn write_byte(address: u32, value: u8) {
    imp::write_byte(address, value);
}

#[cfg(target_arch = "riscv32")]
fn read_byte(address: u32) -> u8 {
    imp::read_byte(address)
}

#[cfg(not(target_arch = "riscv32"))]
fn write_byte(_address: u32, _value: u8) {}

#[cfg(not(target_arch = "riscv32"))]
fn read_byte(_address: u32) -> u8 {
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vendor_irq_numbers() {
        assert_eq!(TIMER_CH0_IRQ, 52);
        assert_eq!(TIMER_CH1_IRQ, 53);
        assert!(TIMER_CH0_IRQ < CLIC_SOURCE_SLOTS);
    }

    #[test]
    fn decodes_clic_mcause_with_status_fields() {
        // Interrupt 52 with MPP=M (bits 29:28), MPIE (bit 27) and MPIL=0xff.
        let mcause = MCAUSE_INTERRUPT | (0b11 << 28) | (1 << 27) | (0xff << 16) | 52;
        assert_eq!(decode_mcause(mcause), Trap::Interrupt(52));
        // Illegal-instruction exception with the same status fields.
        let mcause = (0b11 << 28) | (1 << 27) | 2;
        assert_eq!(decode_mcause(mcause), Trap::Exception(2));
    }
}
