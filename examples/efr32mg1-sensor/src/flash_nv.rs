//! Flash-backed NV storage for EFR32MG1P.
//!
//! Uses the last 2 flash pages (4 KB each) for persistent Zigbee state.
//! Implements FlashDriver trait for the shared LogStructuredNv engine.
//!
//! # Flash layout (EFR32MG1P: 256 KB, page = 2 KB)
//! ```text
//! Page at 0x3E000: NV page A (2 KB)
//! Page at 0x3E800: NV page B (2 KB)
//! ```
//!
//! EFR32MG1P has 2 KB flash pages, but we use 2 consecutive pages
//! for each NV page to get 4 KB per NV slot.

use zigbee_runtime::log_nv::{FlashDriver, LogStructuredNv};

/// NV page A: near end of 256 KB flash, leaving room for bootloader
const NV_PAGE_A: u32 = 0x0003_E000;
/// NV page B: next 2 KB after page A
const NV_PAGE_B: u32 = 0x0003_E800;

/// EFR32MG1P MSC (Memory System Controller) register base.
const MSC_BASE: u32 = 0x400E_0000;
/// MSC write control register.
const MSC_WRITECTRL: u32 = MSC_BASE + 0x008;
/// MSC address register.
const MSC_ADDRB: u32 = MSC_BASE + 0x010;
/// MSC write data register.
const MSC_WDATA: u32 = MSC_BASE + 0x018;
/// MSC status register.
const MSC_STATUS: u32 = MSC_BASE + 0x01C;
/// MSC command register.
const MSC_WRITECMD: u32 = MSC_BASE + 0x00C;

pub struct Efr32FlashDriver;

impl Efr32FlashDriver {
    pub fn new() -> Self {
        Self
    }

    fn wait_ready(&self) {
        // Wait for MSC to be ready (busy bit clear)
        for _ in 0..100_000u32 {
            let status = unsafe { core::ptr::read_volatile(MSC_STATUS as *const u32) };
            if status & 0x01 != 0 {
                // BUSY bit
                core::hint::spin_loop();
            } else {
                break;
            }
        }
    }
}

impl FlashDriver for Efr32FlashDriver {
    fn read(&self, offset: u32, buf: &mut [u8]) {
        // EFR32 flash is memory-mapped at 0x0000_0000
        for (i, b) in buf.iter_mut().enumerate() {
            *b = unsafe { core::ptr::read_volatile((offset + i as u32) as *const u8) };
        }
    }

    fn write(&mut self, offset: u32, data: &[u8]) {
        // Enable write access
        unsafe {
            core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, 0x01); // WREN
        }

        // Write word-by-word (MSC requires aligned 32-bit writes)
        let mut i = 0usize;
        while i < data.len() {
            let mut word = 0xFFFF_FFFFu32;
            for j in 0..4 {
                if i + j < data.len() {
                    word &= !(0xFF << (j * 8));
                    word |= (data[i + j] as u32) << (j * 8);
                }
            }

            self.wait_ready();
            unsafe {
                core::ptr::write_volatile(MSC_ADDRB as *mut u32, offset + i as u32);
                core::ptr::write_volatile(MSC_WRITECMD as *mut u32, 0x08); // LADDRIM
                core::ptr::write_volatile(MSC_WDATA as *mut u32, word);
                core::ptr::write_volatile(MSC_WRITECMD as *mut u32, 0x01); // WRITETRIG
            }

            i += 4;
        }

        self.wait_ready();

        // Disable write access
        unsafe {
            core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, 0x00);
        }
    }

    fn erase_sector(&mut self, offset: u32) {
        // Enable write/erase access
        unsafe {
            core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, 0x01); // WREN
        }

        self.wait_ready();

        unsafe {
            core::ptr::write_volatile(MSC_ADDRB as *mut u32, offset);
            core::ptr::write_volatile(MSC_WRITECMD as *mut u32, 0x08); // LADDRIM
            core::ptr::write_volatile(MSC_WRITECMD as *mut u32, 0x02); // ERASEPAGE
        }

        self.wait_ready();

        // Disable write access
        unsafe {
            core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, 0x00);
        }
    }

    fn sector_size(&self) -> usize {
        2048 // EFR32MG1P page size = 2 KB
    }
}

pub type Nv = LogStructuredNv<Efr32FlashDriver>;

pub fn create_nv() -> Nv {
    LogStructuredNv::new(Efr32FlashDriver::new(), NV_PAGE_A, NV_PAGE_B)
}
