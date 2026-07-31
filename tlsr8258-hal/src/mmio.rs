//! Bare register addresses, transcribed from
//! the official SDK and hardware-proven bring-up code. Kept as plain `u32`
//! addresses rather than `*mut`
//! statics so this module compiles (but is inert) on any host target.

pub const REG_BASE: u32 = 0x800000;

// Analog/system control
pub const REG_ANALOG_ADDR: u32 = REG_BASE + 0x0B8; // addr/data/trigger triplet
pub const REG_CLK_EN0: u32 = REG_BASE + 0x063;
pub const REG_CLK_EN1: u32 = REG_BASE + 0x064;
pub const REG_CLK_EN2: u32 = REG_BASE + 0x065;
pub const REG_RST0: u32 = REG_BASE + 0x060;
pub const REG_RST1: u32 = REG_BASE + 0x061;
pub const REG_RST2: u32 = REG_BASE + 0x062;

// Power-down/reboot control (`platform/chip_8258/register.h`:
// `reg_pwdn_ctrl = REG_ADDR8(0x6f)`; see `reset.rs`'s
// `FLD_PWDN_CTRL_REBOOT`).
pub const REG_PWDN_CTRL: u32 = REG_BASE + 0x06f;

// Timer0
pub const REG_TMR_CTRL: u32 = REG_BASE + 0x620;
pub const REG_TMR_STA: u32 = REG_BASE + 0x623;
pub const REG_TMR0_CAPT: u32 = REG_BASE + 0x624;
pub const REG_TMR0_TICK: u32 = REG_BASE + 0x630;

// Timer1 (`platform/chip_8258/register.h`: `reg_tmr1_capt`/`reg_tmr1_tick`,
// same `reg_tmr_ctrl` register Timer0 shares — see `timer.rs`'s
// `FLD_TMR1_EN`/`FLD_TMR1_MODE_MASK`).
pub const REG_TMR1_CAPT: u32 = REG_BASE + 0x628;
pub const REG_TMR1_TICK: u32 = REG_BASE + 0x634;

// CPU IRQ. The sleepy-device path keeps these masked; the always-on router
// enables only the RF/DMA receive sources while idle.
pub const REG_IRQ_MASK: u32 = REG_BASE + 0x640;
pub const REG_IRQ_SRC: u32 = REG_BASE + 0x648;
pub const REG_IRQ_EN: u32 = REG_BASE + 0x643;

// Free-running 32-bit system tick counter (`platform/chip_8258/register.h`:
// `#define reg_system_tick REG_ADDR32(0x740)`, in the "system timer
// registers: 0x740" block). Read-only for this crate's purposes; used by
// `rng.rs` to reproduce the vendor `rand()` MWC generator's XOR-whitening
// against live timing jitter.
pub const REG_SYSTEM_TICK: u32 = REG_BASE + 0x740;

// I-cache preload size fields (written once, at reset, before any RF/timer
// access — see `vectors::__reset` asm).
pub const REG_ICACHE_CFG: u32 = REG_BASE + 0x60C;

// The application's 64 KiB SRAM window (matches `memory.x`'s `RAM` region
// and the boot-ROM code-SRAM alias). Any buffer handed to a DMA-capable
// peripheral (MSPI flash, ADC dfifo2) must live in this range.
pub const SRAM_START: u32 = 0x0084_0000;
pub const SRAM_END: u32 = 0x0085_0000;

/// `true` if the byte range `[ptr, ptr+len)` is fully contained in the
/// TLSR8258 64 KiB SRAM window. Used to reject flash/ADC DMA buffers that
/// point at flash (XIP) or peripheral MMIO instead of SRAM.
pub fn sram_contains(ptr: usize, len: usize) -> bool {
    let Some(end) = ptr.checked_add(len) else {
        return false;
    };
    ptr >= SRAM_START as usize && end <= SRAM_END as usize
}

#[inline(always)]
/// Read one volatile byte from `addr`.
///
/// # Safety
///
/// `addr` must be a valid readable byte address for the current target.
pub unsafe fn r8(addr: u32) -> u8 {
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}
#[inline(always)]
/// Write one volatile byte to `addr`.
///
/// # Safety
///
/// `addr` must be a valid writable byte address for the current target.
pub unsafe fn w8(addr: u32, val: u8) {
    unsafe { core::ptr::write_volatile(addr as *mut u8, val) }
}
#[inline(always)]
/// Read one volatile little-endian 16-bit value from `addr`.
///
/// # Safety
///
/// `addr` must be valid, readable, and suitably aligned for a `u16`.
pub unsafe fn r16(addr: u32) -> u16 {
    unsafe { core::ptr::read_volatile(addr as *const u16) }
}
#[inline(always)]
/// Write one volatile little-endian 16-bit value to `addr`.
///
/// # Safety
///
/// `addr` must be valid, writable, and suitably aligned for a `u16`.
pub unsafe fn w16(addr: u32, val: u16) {
    unsafe { core::ptr::write_volatile(addr as *mut u16, val) }
}
#[inline(always)]
/// Read one volatile little-endian 32-bit value from `addr`.
///
/// # Safety
///
/// `addr` must be valid, readable, and suitably aligned for a `u32`.
pub unsafe fn r32(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}
#[inline(always)]
/// Write one volatile little-endian 32-bit value to `addr`.
///
/// # Safety
///
/// `addr` must be valid, writable, and suitably aligned for a `u32`.
pub unsafe fn w32(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

#[cfg(target_arch = "tc32")]
pub fn disable_all_irqs() {
    unsafe {
        w8(REG_IRQ_EN, 0);
        w32(REG_IRQ_MASK, 0);
    }
}

/// Run `f` with the TLSR8258 global CPU IRQ enable (`REG_IRQ_EN`) saved and
/// cleared, then restore whatever value it held before `f` ran.
///
/// This generalizes the save/disable/restore idiom already used by
/// [`analog_read`]/[`analog_write`] below and `radio::hw`'s
/// `mask_cpu_rx_irq`/`enable_cpu_rx_irq` into a reusable critical-section
/// primitive for callers (e.g. `capture::hw`) that share a plain, non-atomic
/// data structure between an interrupt handler and main-line code on this
/// single-core, no-RTOS target. Nesting is safe: an inner call restores
/// `REG_IRQ_EN` to the (already-disabled) value an outer call saved, so IRQs
/// stay masked for the full outer critical section.
///
/// `f` must be short, bounded, and non-blocking — while it runs, *no*
/// interrupt (RF, DMA, GPIO, timer) can preempt this core.
///
/// This guard is intentionally implemented without RAII because the
/// firmware is built with `panic = "abort"`/no unwinding. Callers must not
/// invoke a closure that can panic on a host build and then expect the
/// simulated IRQ byte to be restored.
#[cfg(target_arch = "tc32")]
pub fn with_irqs_disabled<R>(f: impl FnOnce() -> R) -> R {
    use core::sync::atomic::{Ordering, compiler_fence};

    let previous_irq = unsafe { r8(REG_IRQ_EN) };
    unsafe { w8(REG_IRQ_EN, 0) };
    compiler_fence(Ordering::SeqCst);
    let result = f();
    compiler_fence(Ordering::SeqCst);
    unsafe { w8(REG_IRQ_EN, previous_irq) };
    result
}

// Analog bus (addr/data/trigger triplet at REG_ANALOG_ADDR..+2). Confirmed
// bit-for-bit against Telink's `analog_read`/`analog_write` (compiled body,
// disassembled from `platform/lib/libdrivers_8258.a:analog.o` — the header
// only declares these `extern`): write the address byte, then the trigger
// byte (0x40 = read, 0x60 = write; write also stores the value byte first),
// poll bit0 of the trigger register until it clears, then either read the
// result from the data byte (read) or the operation is complete (write).
// IRQs are disabled/restored around the sequence, matching the vendor body.
pub(crate) const REG_ANALOG_DATA: u32 = REG_ANALOG_ADDR + 1;
pub(crate) const REG_ANALOG_TRIGGER: u32 = REG_ANALOG_ADDR + 2;
const ANALOG_TRIGGER_READ: u8 = 0x40;
/// `crate::clocks`' own early-boot `analog_write` (which cannot call
/// through to this module's [`analog_write`] — see that function's doc for
/// why) reuses this constant instead of re-deriving the same `0x60` byte,
/// so the two implementations cannot silently drift apart on the trigger
/// value itself.
pub(crate) const ANALOG_TRIGGER_WRITE: u8 = 0x60;
/// Generous bound on top of the vendor's own unbounded spin — the analog
/// bus completes in well under 1 us in practice. Also reused by
/// `crate::clocks`' early-boot `analog_write` for the same reason as
/// [`ANALOG_TRIGGER_WRITE`].
pub(crate) const ANALOG_POLL_ITERATIONS: u32 = 100_000;

/// The TLSR8258 analog register bus (used for pull resistors, ADC config,
/// and deep-sleep-retention scratch registers) did not complete within
/// [`ANALOG_POLL_ITERATIONS`] bounded-wait iterations.
///
/// This is a hard failure, not a "assume zero and carry on" situation: a
/// stuck analog bus means whatever register the caller was trying to read
/// or write is in an unknown state, so every caller (gpio/adc/pm) must
/// propagate this rather than silently substituting a default value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnalogError {
    Timeout,
}

#[cfg(target_arch = "tc32")]
fn analog_wait_idle() -> bool {
    for _ in 0..ANALOG_POLL_ITERATIONS {
        if unsafe { r8(REG_ANALOG_TRIGGER) } & 1 == 0 {
            return true;
        }
        unsafe { core::arch::asm!("nop") };
    }
    false
}

/// Read one byte from the TLSR8258 analog register space (used for pull
/// resistor, ADC, and deep-sleep-retention scratch registers).
///
/// Returns `Err(AnalogError::Timeout)` on a bounded-wait timeout rather
/// than hanging forever (stricter than the vendor implementation, which
/// spins unconditionally) or silently substituting a value.
#[cfg(target_arch = "tc32")]
pub fn analog_read(addr: u8) -> Result<u8, AnalogError> {
    with_irqs_disabled(|| {
        unsafe { w8(REG_ANALOG_ADDR, addr) };
        unsafe { w8(REG_ANALOG_TRIGGER, ANALOG_TRIGGER_READ) };
        let ok = analog_wait_idle();
        let result = if ok {
            Ok(unsafe { r8(REG_ANALOG_DATA) })
        } else {
            Err(AnalogError::Timeout)
        };
        unsafe { w8(REG_ANALOG_TRIGGER, 0) };
        result
    })
}

/// Write one byte to the TLSR8258 analog register space. See
/// [`analog_read`] for the protocol and its provenance.
#[cfg(target_arch = "tc32")]
pub fn analog_write(addr: u8, value: u8) -> Result<(), AnalogError> {
    with_irqs_disabled(|| {
        unsafe { w8(REG_ANALOG_ADDR, addr) };
        unsafe { w8(REG_ANALOG_DATA, value) };
        unsafe { w8(REG_ANALOG_TRIGGER, ANALOG_TRIGGER_WRITE) };
        let ok = analog_wait_idle();
        unsafe { w8(REG_ANALOG_TRIGGER, 0) };
        if ok {
            Ok(())
        } else {
            Err(AnalogError::Timeout)
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sram_contains_accepts_full_window() {
        assert!(sram_contains(SRAM_START as usize, 0x10000));
    }

    #[test]
    fn sram_contains_rejects_flash_addresses() {
        assert!(!sram_contains(0x0000_1000, 16));
    }

    #[test]
    fn sram_contains_rejects_overflow() {
        assert!(!sram_contains(usize::MAX - 4, 16));
    }

    #[test]
    fn sram_contains_rejects_past_end() {
        assert!(!sram_contains((SRAM_END - 4) as usize, 16));
    }

    #[test]
    fn sram_contains_accepts_empty_range_at_end() {
        assert!(sram_contains(SRAM_END as usize, 0));
    }
}
