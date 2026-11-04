//! Stack guard for the XT-ZB1 application linker layout (defense in depth).
//!
//! `link/memory.x` reserves `_stack_guard_start.._stack_guard_end` (64
//! bytes) immediately below the lowest stack address (`__estack`), between
//! `.bss` and the stack. Normal execution never touches it. Every image arms
//! it first thing in `main` and re-checks it from its run loop (the mains
//! roles) or before each idle wait (the sensor), and every image checks it
//! before each flash erase/program ([`crate::storage::MainFlash`]). A
//! damaged guard means the shared task/interrupt stack already left its
//! declared budget, so the firmware stops instead of persisting or
//! transmitting from corrupted RAM.
//!
//! The guard only *detects* an overflow; the stack budget itself is enforced
//! by `tools/bl702-stack-gate.py` and the hardware stack-paint measurement.

/// Distinct from the stack-paint word (`0xA5A55A5A`) and from erased flash.
pub const GUARD_WORD: u32 = 0x5AC3_3CA5;
pub const GUARD_BYTES: usize = 64;

#[cfg(target_arch = "riscv32")]
mod imp {
    use super::GUARD_WORD;

    unsafe extern "C" {
        static mut _stack_guard_start: u32;
        static _stack_guard_end: u32;
    }

    fn guard_words() -> (*mut u32, usize) {
        let start = &raw mut _stack_guard_start;
        let end = &raw const _stack_guard_end;
        let words = (end as usize - start as usize) / core::mem::size_of::<u32>();
        (start, words)
    }

    pub fn arm() {
        let (start, words) = guard_words();
        for index in 0..words {
            // SAFETY: the linker reserves this word-aligned range for the
            // guard and nothing else names it.
            unsafe { core::ptr::write_volatile(start.add(index), GUARD_WORD) };
        }
    }

    pub fn intact() -> bool {
        let (start, words) = guard_words();
        (0..words).all(|index| {
            // SAFETY: as for `arm`.
            unsafe { core::ptr::read_volatile(start.add(index)) == GUARD_WORD }
        })
    }

    #[cold]
    #[inline(never)]
    pub fn tripped() -> ! {
        // Stop every interrupt source (radio RX/ACK re-arm, timer alarm)
        // and halt: no further flash program, radio transmission or
        // protocol processing may run on a corrupted stack/RAM image.
        // SAFETY: single hart; the core never re-enables interrupts here.
        unsafe { core::arch::asm!("csrci mstatus, 8") };
        loop {
            core::hint::spin_loop();
        }
    }
}

/// Host builds have no linker-reserved guard; the guard is always intact.
#[cfg(not(target_arch = "riscv32"))]
mod imp {
    pub fn arm() {}

    pub fn intact() -> bool {
        true
    }

    pub fn tripped() -> ! {
        panic!("BL702 stack guard tripped")
    }
}

/// Write the guard pattern. Call once, first thing in `main`.
pub fn arm() {
    imp::arm();
}

/// `true` while every guard word still holds the pattern.
pub fn intact() -> bool {
    imp::intact()
}

/// Fail closed if the guard was overwritten.
#[inline(never)]
pub fn check() {
    if !intact() {
        imp::tripped();
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn host_guard_is_inert() {
        super::arm();
        assert!(super::intact());
        super::check();
    }

    #[test]
    fn guard_word_differs_from_paint_and_erased_flash() {
        assert_ne!(super::GUARD_WORD, 0xA5A5_5A5A);
        assert_ne!(super::GUARD_WORD, 0xFFFF_FFFF);
        assert_eq!(super::GUARD_BYTES % 4, 0);
    }
}
