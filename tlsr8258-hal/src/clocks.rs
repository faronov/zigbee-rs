//! Analog/clock bring-up, transcribed from
//! Telink's TLSR8258 startup sequence. Runs from `.ram_code`
//! (preloaded into RAM by the boot ROM) because it touches clock-enable
//! registers before the flash cache/XIP path is known-stable.

#[inline(never)]
#[cfg(target_arch = "tc32")]
#[unsafe(link_section = ".ram_code")]
fn analog_write(addr: u8, value: u8) {
    use super::mmio::{REG_ANALOG_ADDR, r8, w8};
    unsafe {
        w8(REG_ANALOG_ADDR, addr);
        w8(REG_ANALOG_ADDR + 1, value);
        w8(REG_ANALOG_ADDR + 2, 0x60);
        for _ in 0..100_000u32 {
            if r8(REG_ANALOG_ADDR + 2) & 1 == 0 {
                w8(REG_ANALOG_ADDR + 2, 0);
                return;
            }
            core::arch::asm!("nop");
        }
        w8(REG_ANALOG_ADDR + 2, 0); // timeout: clear and continue
    }
}

#[inline(never)]
#[cfg(target_arch = "tc32")]
#[unsafe(link_section = ".ram_code")]
pub fn init() {
    use super::mmio::*;
    analog_write(0x82, 0x64);
    analog_write(0x34, 0x80);
    analog_write(0x06, 0x00);
    analog_write(0x0a, 0x44);
    analog_write(0x0b, 0x38);
    analog_write(0x05, 0x02);
    analog_write(0x8c, 0x02);
    analog_write(0x02, 0xa2);
    analog_write(0x27, 0x00);
    analog_write(0x28, 0x00);
    analog_write(0x29, 0x00);
    analog_write(0x2a, 0x00);
    analog_write(0x01, 0x4c);
    unsafe {
        for _ in 0..20_000u32 {
            core::arch::asm!("nop");
        }
        w8(REG_BASE + 0x066, 0x42);
        for _ in 0..5_000u32 {
            core::arch::asm!("nop");
        }
        w8(REG_CLK_EN0, 0xFF);
        w8(REG_CLK_EN1, 0xFF);
        w8(REG_CLK_EN2, 0xFF);
    }
}
