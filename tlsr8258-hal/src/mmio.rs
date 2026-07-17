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

// Timer0
pub const REG_TMR_CTRL: u32 = REG_BASE + 0x620;
pub const REG_TMR_STA: u32 = REG_BASE + 0x623;
pub const REG_TMR0_CAPT: u32 = REG_BASE + 0x624;
pub const REG_TMR0_TICK: u32 = REG_BASE + 0x630;

// CPU IRQ (kept masked/disabled for this firmware — see `platform::vectors`
// and `mac_test`; defined here only for completeness/documentation).
pub const REG_IRQ_MASK: u32 = REG_BASE + 0x640;
pub const REG_IRQ_SRC: u32 = REG_BASE + 0x648;
pub const REG_IRQ_EN: u32 = REG_BASE + 0x643;

// I-cache preload size fields (written once, at reset, before any RF/timer
// access — see `vectors::__reset` asm).
pub const REG_ICACHE_CFG: u32 = REG_BASE + 0x60C;

#[inline(always)]
pub unsafe fn r8(addr: u32) -> u8 {
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}
#[inline(always)]
pub unsafe fn w8(addr: u32, val: u8) {
    unsafe { core::ptr::write_volatile(addr as *mut u8, val) }
}
#[inline(always)]
pub unsafe fn r32(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}
#[inline(always)]
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
