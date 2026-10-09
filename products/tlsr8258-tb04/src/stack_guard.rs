//! SVC stack guard for the TB-04 product linker layout (defense in depth).
//!
//! `link/memory.x` reserves `_svc_stack_guard_start.._svc_stack_guard_end`
//! (64 bytes) immediately below `_svc_stack_bottom`. Normal execution never
//! touches it. The firmware writes a fixed pattern there at startup and
//! re-checks it from the main loop and before every flash erase/program. A
//! damaged guard means the SVC stack already left its declared budget, so the
//! firmware stops instead of persisting or transmitting from corrupted RAM.
//!
//! The guard only *detects* an overflow; the stack budget itself is enforced
//! by `tools/tlsr8258-stack-report.py` and the HIL stack-paint regression.

/// Distinct from the HIL paint word (`0xA5A55A5A`) and from erased flash.
pub const GUARD_WORD: u32 = 0x5AC3_3CA5;
pub const GUARD_BYTES: usize = 64;

unsafe extern "C" {
    static mut _svc_stack_guard_start: u32;
    static _svc_stack_guard_end: u32;
}

fn guard_words() -> (*mut u32, usize) {
    let start = &raw mut _svc_stack_guard_start;
    let end = &raw const _svc_stack_guard_end;
    let words = (end as usize - start as usize) / core::mem::size_of::<u32>();
    (start, words)
}

/// Write the guard pattern. Call once, first thing after reset.
pub fn arm() {
    let (start, words) = guard_words();
    for index in 0..words {
        // SAFETY: the linker reserves this word-aligned range for the guard
        // and nothing else names it.
        unsafe { core::ptr::write_volatile(start.add(index), GUARD_WORD) };
    }
}

/// `true` while every guard word still holds the pattern.
pub fn intact() -> bool {
    let (start, words) = guard_words();
    (0..words).all(|index| {
        // SAFETY: as for `arm`.
        unsafe { core::ptr::read_volatile(start.add(index)) == GUARD_WORD }
    })
}

/// Fail closed if the guard was overwritten.
#[inline(never)]
pub fn check() {
    if !intact() {
        tripped();
    }
}

#[cold]
#[inline(never)]
fn tripped() -> ! {
    // Stop every interrupt source (radio RX/TX, timers) and halt: no further
    // flash program, radio transmission or protocol processing may run on a
    // corrupted stack/RAM image.
    tlsr8258_hal::mmio::disable_all_irqs();
    loop {
        core::hint::spin_loop();
    }
}
