//! SPIF flash controller: XIP read, page program, sector erase.
//!
//! PHY6222 flash is memory-mapped at `0x11000000` for reads (XIP).
//! Writes and erases go through the SPIF controller at `0x4000C800`.
//! Cache must be bypassed during write/erase operations.

use crate::regs::*;

/// Read bytes from flash via XIP (memory-mapped, no SPIF commands needed).
pub fn read(offset: u32, buf: &mut [u8]) {
    let addr = (FLASH_BASE | (offset & 0x7_FFFF)) as *const u8;
    for (i, b) in buf.iter_mut().enumerate() {
        *b = unsafe { core::ptr::read_volatile(addr.add(i)) };
    }
}

/// Erase a 4 KB flash sector.
pub fn erase_sector(offset: u32) {
    enter_cache_bypass();

    spif_wait_idle();
    reg_write(SPIF_FCMD, 0x0600_0001); // WREN
    spif_wait_idle();
    spif_wait_not_busy();

    reg_write(SPIF_FCMD_ADDR, offset);
    reg_write(SPIF_FCMD, (0x20u32 << 24) | 0x8_0001 | (2 << 16));
    spif_wait_idle();
    spif_wait_not_busy();

    exit_cache_bypass();
    cache_flush();
}

/// Write data to flash (page program, handles 256-byte page boundaries).
pub fn write(offset: u32, data: &[u8]) {
    enter_cache_bypass();

    let mut pos = 0;
    while pos < data.len() {
        let page_boundary = ((offset as usize + pos) | 0xFF) + 1;
        let remaining_in_page = page_boundary - (offset as usize + pos);
        let chunk_len = (data.len() - pos).min(remaining_in_page).min(256);

        // Write Enable
        spif_wait_idle();
        reg_write(SPIF_FCMD, 0x0600_0001);
        spif_wait_idle();
        spif_wait_not_busy();

        // Set address
        reg_write(SPIF_FCMD_ADDR, offset + pos as u32);

        // Write data to FIFO
        let mut i = 0;
        while i < chunk_len {
            let mut word = 0xFFFF_FFFFu32;
            for b in 0..4 {
                if i + b < chunk_len {
                    word &= !(0xFF << (b * 8));
                    word |= (data[pos + i + b] as u32) << (b * 8);
                }
            }
            reg_write(SPIF_FCMD_WRDATA + (i as u32 / 4) * 4, word);
            i += 4;
        }

        // Page Program
        let wr_words = ((chunk_len + 3) / 4) as u32;
        let fcmd = (0x02u32 << 24)
            | 0x8_0001
            | (2 << 16)
            | 0x8000
            | (wr_words.saturating_sub(1) << 12);
        reg_write(SPIF_FCMD, fcmd);
        spif_wait_idle();
        spif_wait_not_busy();

        pos += chunk_len;
    }

    exit_cache_bypass();
    cache_flush();
}

// ── Internal helpers ────────────────────────────────────────────

fn spif_wait_idle() {
    for _ in 0..100_000u32 {
        if reg_read(SPIF_FCMD) & 0x02 == 0 {
            if reg_read(SPIF_CONFIG) & 0x8000_0000 != 0 {
                return;
            }
        }
        cortex_m::asm::nop();
    }
}

fn spif_wait_not_busy() {
    for _ in 0..1_000_000u32 {
        reg_write(SPIF_FCMD, (0x05u32 << 24) | 0x80_0001 | (1 << 20));
        spif_wait_idle();
        let status = reg_read(SPIF_FCMD_RDDATA) & 0xFF;
        if status & 0x01 == 0 { return; }
        cortex_m::asm::nop();
    }
}

fn enter_cache_bypass() {
    cortex_m::interrupt::free(|_| {
        reg_write(CACHE_CTRL0, 0x02);
        reg_write(CACHE_BYPASS_REG, 1);
    });
}

fn exit_cache_bypass() {
    cortex_m::interrupt::free(|_| {
        reg_write(CACHE_CTRL0, 0x00);
        reg_write(CACHE_BYPASS_REG, 0);
    });
}

fn cache_flush() {
    cortex_m::interrupt::free(|_| {
        reg_write(CACHE_CTRL0, 0x02);
        for _ in 0..8 { cortex_m::asm::nop(); }
        reg_write(CACHE_CTRL0, 0x03);
        for _ in 0..8 { cortex_m::asm::nop(); }
        reg_write(CACHE_CTRL0, 0x00);
    });
}
