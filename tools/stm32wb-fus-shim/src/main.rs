//! SRAM-resident STM32WB FUS helper for boards without a debug probe.
//!
//! The ROM USB DFU bootloader (AN3156) can write SRAM1 and jump to a vector
//! table there, but it has no coprocessor (FUS) commands. This helper is
//! that missing piece: hwlab writes it to `0x2000_8000`, writes one
//! parameter block to `0x2002_F000` and leaves DFU into it. The helper
//! performs exactly one FUS step through the IPCC system channel (the same
//! sequence hwlab's stlink plugin drives over SWD), records everything in
//! the result block at `0x2002_F100` and then requests a system reset. With
//! BOOT0 still selected the reset re-enters the ROM bootloader, so the host
//! reads the result block over DFU and decides the next step. Resets caused
//! by the FUS itself (upgrade, delete, start) land in the bootloader in the
//! same way; the host simply runs the helper again.
//!
//! It never erases or programs flash and never writes option bytes:
//! staging coprocessor images below the secure start is the DFU plugin's
//! job, and the FUS is the only path into the secure area.
//!
//! LEDs (P-NUCLEO-WB55 dongle, active high): blue PA4 while working, green
//! PB0 on success, red PB1 on error.

#![no_std]
#![no_main]

use core::arch::asm;
use core::panic::PanicInfo;
use core::ptr::{read_volatile, write_volatile};

// --- fixed host interface ---------------------------------------------------

const PARAM: u32 = 0x2002_F000;
const PARAM_MAGIC: u32 = 0x5048_4D31; // "1MHP"
const RESULT: u32 = 0x2002_F100;
const RESULT_MAGIC: u32 = 0x5253_4D31; // "1MSR"
const RESULT_LEN: u32 = 0x200;

// Result block word offsets.
const R_MAGIC: u32 = 0x00;
const R_OP: u32 = 0x04;
const R_PHASE: u32 = 0x08;
const R_ERROR: u32 = 0x0C;
const R_PWR_CR4: u32 = 0x10;
const R_RCC_CSR: u32 = 0x14;
const R_FLASH_SFR: u32 = 0x18;
const R_FLASH_SRRVR: u32 = 0x1C;
const R_FLASH_IPCCBR: u32 = 0x20;
const R_HCLK: u32 = 0x24;
const R_MODE: u32 = 0x28;
const R_READY: u32 = 0x2C;
const R_IGNORED_EVENTS: u32 = 0x30;
const R_CMD_STATUS: u32 = 0x34;
const R_LAST_STATE: u32 = 0x38;
const R_STATES: u32 = 0x3C;
const R_ELAPSED_MS: u32 = 0x40;
const R_DEVINFO_PTR: u32 = 0x44;
const R_DEVINFO: u32 = 0x80; // 0x60 bytes
const R_HISTORY: u32 = 0xE0; // u16 entries (state | error << 8), changes only
const HISTORY_MAX: u32 = (RESULT_LEN - R_HISTORY) / 2;

// Operations.
const OP_INFO: u32 = 1;
const OP_ENTER_FUS: u32 = 2;
const OP_UPGRADE: u32 = 3;
const OP_DELETE: u32 = 4;
const OP_START_WS: u32 = 5;
const OP_POLL: u32 = 6;

// Phases.
const PH_START: u32 = 0x01;
const PH_C2_BOOTED: u32 = 0x02;
const PH_READY: u32 = 0x03;
const PH_CMD_SENT: u32 = 0x04;
const PH_POLLING: u32 = 0x05;
const PH_WAIT_RESET: u32 = 0x06;
const PH_DONE: u32 = 0xD0;
const PH_FAILED: u32 = 0xEE;

// Errors.
const E_NONE: u32 = 0;
const E_BAD_PARAM: u32 = 0x01;
const E_IPCCBR: u32 = 0x02;
const E_SRAM2A_SECURE: u32 = 0x03;
const E_REUSE_TABLES: u32 = 0x04;
const E_NO_READY: u32 = 0x05;
const E_READY_CODE: u32 = 0x06;
const E_CMD_BUSY: u32 = 0x07;
const E_CMD_TIMEOUT: u32 = 0x08;
const E_CMD_RESPONSE: u32 = 0x09;
const E_NOT_FUS: u32 = 0x0A;
const E_NOT_IDLE: u32 = 0x0B;
const E_REFUSED: u32 = 0x0C;
const E_FUS_ERROR: u32 = 0x0D;
const E_TIMEOUT: u32 = 0x0E;
const E_EVENT_QUEUE: u32 = 0x0F;
const E_RELEASE: u32 = 0x10;
const E_PANIC: u32 = 0x7F;

// --- STM32WB registers -------------------------------------------------------

const RCC_CR: u32 = 0x5800_0000;
const RCC_CFGR: u32 = 0x5800_0008;
const RCC_PLLCFGR: u32 = 0x5800_000C;
const RCC_AHB2ENR: u32 = 0x5800_004C;
const RCC_AHB3ENR: u32 = 0x5800_0050;
const RCC_CSR: u32 = 0x5800_0094;
const PWR_CR4: u32 = 0x5800_040C;
const C2BOOT: u32 = 1 << 15;
const FLASH_IPCCBR: u32 = 0x5800_403C;
const FLASH_SFR: u32 = 0x5800_4080;
const FLASH_SRRVR: u32 = 0x5800_4084;

const IPCC: u32 = 0x5800_0C00;
const IPCC_C1CR: u32 = IPCC;
const IPCC_C1SCR: u32 = IPCC + 0x08;
const IPCC_C1TOC2SR: u32 = IPCC + 0x0C;
const IPCC_C2TOC1SR: u32 = IPCC + 0x1C;
const CH_SYS: u32 = 1 << 1; // channel 2: system command / event
const CH_FREE: u32 = 1 << 3; // channel 4: buffer release

const GPIOA: u32 = 0x4800_0000;
const GPIOB: u32 = 0x4800_0400;

const SYST_CSR: u32 = 0xE000_E010;
const NVIC_ICER: u32 = 0xE000_E180;
const NVIC_ICPR: u32 = 0xE000_E280;
const SCB_VTOR: u32 = 0xE000_ED08;
const SCB_AIRCR: u32 = 0xE000_ED0C;
const SCB_CPACR: u32 = 0xE000_ED88;
const DEMCR: u32 = 0xE000_EDFC;
const DWT_CTRL: u32 = 0xE000_1000;
const DWT_CYCCNT: u32 = 0xE000_1004;

// --- mailbox layout (identical to hwlab's stlink FUS driver) ----------------

const SRAM2A: u32 = 0x2003_0000;
const SRAM2A_END: u32 = 0x2003_8000;
const T_REF: u32 = 0x000;
const T_DEVINFO: u32 = 0x030;
const T_SYS: u32 = 0x0D0;
const T_MM: u32 = 0x0F0;
const T_SYSQ: u32 = 0x200;
const T_FREEQ: u32 = 0x208;
const T_CMDBUF: u32 = 0x210;
const T_SPARE_SYS: u32 = 0x320;
const T_SPARE_BLE: u32 = 0x430;
const T_POOL: u32 = 0x540;
const T_POOL_SIZE: u32 = 0x600;
const T_END: u32 = T_POOL + T_POOL_SIZE;
// Table slots in reference-table order: devinfo, BLE, Thread, sys, mem
// manager, traces, MAC, Zigbee, LLD tests, BLE LLD.
const T_SLOTS: [u32; 10] = [
    T_DEVINFO, 0x090, 0x0B0, T_SYS, T_MM, 0x110, 0x130, 0x150, 0x170, 0x190,
];

const OP_FUS_GET_STATE: u16 = 0xFC52;
const OP_FUS_FW_UPGRADE: u16 = 0xFC54;
const OP_FUS_FW_DELETE: u16 = 0xFC55;
const OP_FUS_START_WS: u16 = 0xFC5A;
const PKT_SYS_CMD: u8 = 0x10;
const EVT_CMD_DONE: u8 = 0x0E;
const EVT_VENDOR: u8 = 0xFF;
const SUBEVT_READY: u16 = 0x9200;
const READY_WS: u32 = 0;
const READY_FUS: u32 = 1;
const STATE_IDLE: u8 = 0x00;
const STATE_NOT_RUNNING: u8 = 0xFE;
const STATE_ERROR: u8 = 0xFF;

#[inline(always)]
fn rd(a: u32) -> u32 {
    unsafe { read_volatile(a as *const u32) }
}
#[inline(always)]
fn wr(a: u32, v: u32) {
    unsafe { write_volatile(a as *mut u32, v) }
}
#[inline(always)]
fn rd8(a: u32) -> u8 {
    unsafe { read_volatile(a as *const u8) }
}
#[inline(always)]
fn wr8(a: u32, v: u8) {
    unsafe { write_volatile(a as *mut u8, v) }
}
#[inline(always)]
fn dsb() {
    unsafe { asm!("dsb", "isb", options(nostack, preserves_flags)) }
}

fn res(off: u32, v: u32) {
    wr(RESULT + off, v);
}

// --- vector table -----------------------------------------------------------

#[repr(C)]
pub struct Vectors {
    sp: u32,
    reset: unsafe extern "C" fn() -> !,
    nmi: unsafe extern "C" fn() -> !,
    hard_fault: unsafe extern "C" fn() -> !,
}

#[unsafe(link_section = ".vector_table")]
#[unsafe(no_mangle)]
#[used]
pub static VECTORS: Vectors = Vectors {
    sp: PARAM,
    reset: shim_reset,
    nmi: shim_fault,
    hard_fault: shim_fault,
};

#[unsafe(no_mangle)]
unsafe extern "C" fn shim_fault() -> ! {
    fail(E_PANIC)
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    fail(E_PANIC)
}

// --- timing -----------------------------------------------------------------

struct Clock {
    per_ms: u32,
}

impl Clock {
    fn new() -> Self {
        wr(DEMCR, rd(DEMCR) | (1 << 24));
        wr(DWT_CYCCNT, 0);
        wr(DWT_CTRL, rd(DWT_CTRL) | 1);
        let hz = hclk();
        res(R_HCLK, hz);
        Self {
            per_ms: (hz / 1000).max(1),
        }
    }
    fn now(&self) -> u32 {
        rd(DWT_CYCCNT)
    }
    fn elapsed_ms(&self, since: u32) -> u32 {
        self.now().wrapping_sub(since) / self.per_ms
    }
    fn delay_ms(&self, ms: u32) {
        let t = self.now();
        while self.elapsed_ms(t) < ms {}
    }
}

/// HCLK1 from the RCC configuration the bootloader left behind.
fn hclk() -> u32 {
    const MSI: [u32; 12] = [
        100_000, 200_000, 400_000, 800_000, 1_000_000, 2_000_000, 4_000_000, 8_000_000, 16_000_000,
        24_000_000, 32_000_000, 48_000_000,
    ];
    let cr = rd(RCC_CR);
    let cfgr = rd(RCC_CFGR);
    let msi = MSI[(((cr >> 4) & 0xF) as usize).min(11)];
    let hse = if cr & (1 << 20) != 0 {
        16_000_000
    } else {
        32_000_000
    };
    let sys = match (cfgr >> 2) & 3 {
        0 => msi,
        1 => 16_000_000,
        2 => hse,
        _ => {
            let p = rd(RCC_PLLCFGR);
            let src = match p & 3 {
                1 => msi,
                2 => 16_000_000,
                3 => 32_000_000,
                _ => 0,
            };
            let m = ((p >> 4) & 7) + 1;
            let n = (p >> 8) & 0x7F;
            let r = ((p >> 29) & 7) + 1;
            src / m * n / r
        }
    };
    let div = match (cfgr >> 4) & 0xF {
        0b0001 => 3,
        0b0010 => 5,
        0b0101 => 6,
        0b0110 => 10,
        0b0111 => 32,
        0b1000 => 2,
        0b1001 => 4,
        0b1010 => 8,
        0b1011 => 16,
        0b1100 => 64,
        0b1101 => 128,
        0b1110 => 256,
        0b1111 => 512,
        _ => 1,
    };
    // A clock we cannot decode would make every timeout wrong; assume the
    // fastest plausible core clock so timeouts only get longer.
    match sys / div {
        0 => 64_000_000,
        hz => hz,
    }
}

// --- LEDs -------------------------------------------------------------------

fn leds_init() {
    wr(RCC_AHB2ENR, rd(RCC_AHB2ENR) | 0b11);
    dsb();
    // PA4, PB0, PB1 as push-pull outputs, low.
    wr(GPIOA + 0x18, 1 << (4 + 16));
    wr(GPIOB + 0x18, (1 << 16) | (1 << 17));
    wr(GPIOA, (rd(GPIOA) & !(3 << 8)) | (1 << 8));
    wr(GPIOB, (rd(GPIOB) & !0xF) | 0b0101);
}

fn led(blue: bool, green: bool, red: bool) {
    wr(GPIOA + 0x18, if blue { 1 << 4 } else { 1 << 20 });
    let set = (green as u32) | ((red as u32) << 1);
    let clr = (!set) & 0b11;
    wr(GPIOB + 0x18, set | (clr << 16));
}

// --- end of run ---------------------------------------------------------------

fn finish(clock: Option<&Clock>, start: u32, phase: u32, err: u32) -> ! {
    if let Some(c) = clock {
        res(R_ELAPSED_MS, c.elapsed_ms(start));
        copy_devinfo();
    }
    res(R_ERROR, err);
    res(R_PHASE, phase);
    led(false, err == E_NONE, err != E_NONE);
    dsb();
    // Let the LED be visible, then go back to the ROM bootloader.
    match clock {
        Some(c) => c.delay_ms(300),
        None => {
            for _ in 0..2_000_000u32 {
                unsafe { asm!("nop") }
            }
        }
    }
    system_reset()
}

fn fail(err: u32) -> ! {
    res(R_ERROR, err);
    res(R_PHASE, PH_FAILED);
    led(false, false, true);
    for _ in 0..2_000_000u32 {
        unsafe { asm!("nop") }
    }
    system_reset()
}

fn system_reset() -> ! {
    dsb();
    wr(SCB_AIRCR, 0x05FA_0004);
    dsb();
    loop {
        unsafe { asm!("nop") }
    }
}

fn copy_devinfo() {
    let ptr = rd(SRAM2A + T_REF);
    res(R_DEVINFO_PTR, ptr);
    if ptr >= SRAM2A && ptr + 0x60 <= SRAM2A_END && ptr & 3 == 0 {
        for i in (0..0x60).step_by(4) {
            res(R_DEVINFO + i, rd(ptr + i));
        }
    }
}

// --- mailbox ----------------------------------------------------------------

struct Mbox {
    cmdbuf: u32,
    sysq: u32,
    freeq: u32,
}

fn in_sram2a(a: u32, len: u32) -> bool {
    a >= SRAM2A && a + len <= SRAM2A_END
}

impl Mbox {
    /// Fresh mailbox and CPU2 boot (C2BOOT clear).
    fn boot() -> Self {
        let base = SRAM2A;
        for off in (0..T_END).step_by(4) {
            wr(base + off, 0);
        }
        for (i, t) in T_SLOTS.iter().enumerate() {
            wr(base + T_REF + 4 * i as u32, base + t);
        }
        wr(base + T_SYS, base + T_CMDBUF);
        wr(base + T_SYS + 4, base + T_SYSQ);
        wr(base + T_MM, base + T_SPARE_BLE);
        wr(base + T_MM + 4, base + T_SPARE_SYS);
        wr(base + T_MM + 8, base + T_POOL);
        wr(base + T_MM + 12, T_POOL_SIZE);
        wr(base + T_MM + 16, base + T_FREEQ);
        for q in [T_SYSQ, T_FREEQ] {
            wr(base + q, base + q);
            wr(base + q + 4, base + q);
        }
        wr(RCC_AHB3ENR, rd(RCC_AHB3ENR) | (1 << 20));
        dsb();
        wr(IPCC_C1CR, 0);
        wr(IPCC_C1SCR, 0x3F);
        dsb();
        wr(PWR_CR4, rd(PWR_CR4) | C2BOOT);
        dsb();
        res(R_MODE, 0);
        res(R_PHASE, PH_C2_BOOTED);
        Self {
            cmdbuf: base + T_CMDBUF,
            sysq: base + T_SYSQ,
            freeq: base + T_FREEQ,
        }
    }

    /// CPU2 was already booted by someone else (the bootloader): use the
    /// buffers its reference table names.
    fn reuse() -> Self {
        let sys = rd(SRAM2A + T_REF + 12);
        let mm = rd(SRAM2A + T_REF + 16);
        if !in_sram2a(sys, 8) || !in_sram2a(mm, 20) {
            fail(E_REUSE_TABLES);
        }
        let (cmdbuf, sysq, freeq) = (rd(sys), rd(sys + 4), rd(mm + 16));
        if !in_sram2a(cmdbuf, 0x110) || !in_sram2a(sysq, 8) || !in_sram2a(freeq, 8) {
            fail(E_REUSE_TABLES);
        }
        wr(RCC_AHB3ENR, rd(RCC_AHB3ENR) | (1 << 20));
        dsb();
        res(R_MODE, 1);
        Self {
            cmdbuf,
            sysq,
            freeq,
        }
    }

    /// One system event as (event code, vendor sub-event, first payload
    /// byte), or None after `timeout_ms`.
    fn event(&self, c: &Clock, timeout_ms: u32) -> Option<(u8, u16, u8)> {
        let t = c.now();
        loop {
            while rd(IPCC_C2TOC1SR) & CH_SYS == 0 {
                if c.elapsed_ms(t) >= timeout_ms {
                    return None;
                }
            }
            let head = self.sysq;
            let node = rd(head);
            if node == head {
                wr(IPCC_C1SCR, CH_SYS); // flag without an event
                continue;
            }
            if !in_sram2a(node, 0x10) {
                fail(E_EVENT_QUEUE);
            }
            let next = rd(node);
            let code = rd8(node + 9);
            let plen = rd8(node + 10);
            let (sub, first) = if code == EVT_VENDOR && plen >= 2 {
                (
                    rd8(node + 11) as u16 | (rd8(node + 12) as u16) << 8,
                    if plen > 2 { rd8(node + 13) } else { 0xFF },
                )
            } else {
                (0, if plen > 0 { rd8(node + 11) } else { 0xFF })
            };
            wr(head, next);
            wr(next + 4, head);
            self.release(c, node);
            if rd(head) == head {
                wr(IPCC_C1SCR, CH_SYS);
            }
            return Some((code, sub, first));
        }
    }

    fn release(&self, c: &Clock, node: u32) {
        let t = c.now();
        while rd(IPCC_C1TOC2SR) & CH_FREE != 0 {
            if c.elapsed_ms(t) >= 1000 {
                fail(E_RELEASE);
            }
        }
        let q = self.freeq;
        let tail = rd(q + 4);
        wr(node, q);
        wr(node + 4, tail);
        wr(q + 4, node);
        wr(tail, node);
        dsb();
        wr(IPCC_C1SCR, CH_FREE << 16);
    }

    /// Wait for the ready event; returns its code (READY_WS / READY_FUS).
    fn ready(&self, c: &Clock, timeout_ms: u32) -> u32 {
        let t = c.now();
        loop {
            let left = timeout_ms.saturating_sub(c.elapsed_ms(t));
            match self.event(c, left) {
                None => fail(E_NO_READY),
                Some((EVT_VENDOR, SUBEVT_READY, code)) => {
                    let code = code as u32;
                    res(R_READY, code);
                    if code != READY_WS && code != READY_FUS {
                        fail(E_READY_CODE);
                    }
                    res(R_PHASE, PH_READY);
                    return code;
                }
                Some(_) => res(R_IGNORED_EVENTS, rd(RESULT + R_IGNORED_EVENTS) + 1),
            }
        }
    }

    /// Send one system command; returns the first four bytes of the
    /// command-complete payload after the opcode (status first).
    fn command(&self, c: &Clock, op: u16) -> [u8; 4] {
        if rd(IPCC_C1TOC2SR) & CH_SYS != 0 {
            fail(E_CMD_BUSY);
        }
        for off in (0..16).step_by(4) {
            wr(self.cmdbuf + off, 0);
        }
        wr8(self.cmdbuf + 8, PKT_SYS_CMD);
        wr8(self.cmdbuf + 9, op as u8);
        wr8(self.cmdbuf + 10, (op >> 8) as u8);
        wr8(self.cmdbuf + 11, 0);
        dsb();
        wr(IPCC_C1SCR, CH_SYS << 16);
        let t = c.now();
        while rd(IPCC_C1TOC2SR) & CH_SYS != 0 {
            if c.elapsed_ms(t) >= 5000 {
                fail(E_CMD_TIMEOUT);
            }
        }
        dsb();
        let b = self.cmdbuf;
        let rop = rd8(b + 4) as u16 | (rd8(b + 5) as u16) << 8;
        let len = rd8(b + 2);
        if rd8(b + 1) != EVT_CMD_DONE || rop != op || len < 3 {
            fail(E_CMD_RESPONSE);
        }
        let n = (len - 3).min(4) as u32;
        let mut p = [0xFFu8; 4];
        for i in 0..n {
            p[i as usize] = rd8(b + 6 + i);
        }
        p
    }

    /// FUS_GET_STATE, recorded in the result block; returns (state, error).
    fn state(&self, c: &Clock) -> (u8, u8) {
        let p = self.command(c, OP_FUS_GET_STATE);
        let (st, ec) = (p[0], if p[1] == 0xFF { 0 } else { p[1] });
        let v = st as u32 | (ec as u32) << 8;
        let n = rd(RESULT + R_STATES);
        let last = rd(RESULT + R_LAST_STATE);
        if n == 0 || last != v {
            let idx = rd(RESULT + R_STATES);
            if idx < HISTORY_MAX {
                let a = RESULT + R_HISTORY + idx * 2;
                wr8(a, st);
                wr8(a + 1, ec);
            }
            res(R_STATES, idx + 1);
        }
        res(R_LAST_STATE, v);
        (st, ec)
    }
}

// --- entry ------------------------------------------------------------------

#[unsafe(no_mangle)]
unsafe extern "C" fn shim_reset() -> ! {
    unsafe {
        asm!("cpsid i", options(nomem, nostack, preserves_flags));
    }
    // Detach from the bootloader: no SysTick, no NVIC sources, our vectors.
    wr(SYST_CSR, 0);
    for i in 0..8 {
        wr(NVIC_ICER + 4 * i, 0xFFFF_FFFF);
        wr(NVIC_ICPR + 4 * i, 0xFFFF_FFFF);
    }
    wr(SCB_VTOR, &VECTORS as *const Vectors as u32);
    wr(SCB_CPACR, rd(SCB_CPACR) | (0xF << 20));
    dsb();
    run()
}

fn run() -> ! {
    leds_init();
    led(true, false, false);

    let magic = rd(PARAM);
    let op = rd(PARAM + 4);
    let timeout_ms = rd(PARAM + 8).clamp(1000, 600_000);
    // Consume the request: a later jump without a new block does nothing.
    wr(PARAM, 0);

    for off in (0..RESULT_LEN).step_by(4) {
        res(off, 0);
    }
    res(R_MAGIC, RESULT_MAGIC);
    res(R_OP, op);
    res(R_READY, 0xFF);
    res(R_PHASE, PH_START);
    let cr4 = rd(PWR_CR4);
    let ipccbr = rd(FLASH_IPCCBR);
    let srrvr = rd(FLASH_SRRVR);
    res(R_PWR_CR4, cr4);
    res(R_RCC_CSR, rd(RCC_CSR));
    res(R_FLASH_SFR, rd(FLASH_SFR));
    res(R_FLASH_SRRVR, srrvr);
    res(R_FLASH_IPCCBR, ipccbr);
    if magic != PARAM_MAGIC || !(OP_INFO..=OP_POLL).contains(&op) {
        finish(None, 0, PH_FAILED, E_BAD_PARAM);
    }
    let c = Clock::new();
    let t0 = c.now();

    if ipccbr & 0x3FFF != 0 {
        finish(Some(&c), t0, PH_FAILED, E_IPCCBR);
    }
    let secure2a = if srrvr >> 23 & 1 != 0 {
        SRAM2A_END
    } else {
        SRAM2A + ((srrvr >> 18 & 0x1F) << 10)
    };

    let (mb, ready) = if cr4 & C2BOOT != 0 {
        // Already booted: the ready event is gone; ask the FUS directly.
        (Mbox::reuse(), 0xFF)
    } else {
        if SRAM2A + T_END > secure2a {
            finish(Some(&c), t0, PH_FAILED, E_SRAM2A_SECURE);
        }
        let mb = Mbox::boot();
        let r = mb.ready(&c, 5000);
        (mb, r)
    };

    let err = match op {
        OP_INFO => {
            if ready != READY_WS {
                mb.state(&c);
            }
            E_NONE
        }
        OP_ENTER_FUS => {
            if ready == READY_WS {
                // Two FUS_GET_STATE make the wireless stack reboot into the
                // FUS (AN5185); that is a system reset back to DFU.
                mb.state(&c);
                mb.state(&c);
                res(R_PHASE, PH_WAIT_RESET);
                c.delay_ms(timeout_ms);
                E_TIMEOUT
            } else {
                mb.state(&c);
                E_NONE
            }
        }
        OP_UPGRADE | OP_DELETE | OP_START_WS => {
            if ready == READY_WS {
                finish(Some(&c), t0, PH_FAILED, E_NOT_FUS);
            }
            let (st, _) = mb.state(&c);
            if st != STATE_IDLE {
                finish(Some(&c), t0, PH_FAILED, E_NOT_IDLE);
            }
            let fop = match op {
                OP_UPGRADE => OP_FUS_FW_UPGRADE,
                OP_DELETE => OP_FUS_FW_DELETE,
                _ => OP_FUS_START_WS,
            };
            let p = mb.command(&c, fop);
            res(R_CMD_STATUS, u32::from_le_bytes(p));
            if p[0] != 0 {
                finish(Some(&c), t0, PH_FAILED, E_REFUSED);
            }
            res(R_PHASE, PH_CMD_SENT);
            if op == OP_START_WS {
                // Never FUS_GET_STATE after START_WS (two would re-enter the
                // FUS); wait for the reset into the stack.
                res(R_PHASE, PH_WAIT_RESET);
                c.delay_ms(timeout_ms);
                E_TIMEOUT
            } else {
                poll(&mb, &c, timeout_ms, true)
            }
        }
        OP_POLL => {
            if ready == READY_WS {
                E_NONE
            } else {
                poll(&mb, &c, timeout_ms, false)
            }
        }
        _ => E_BAD_PARAM,
    };
    let phase = if err == E_NONE { PH_DONE } else { PH_FAILED };
    finish(Some(&c), t0, phase, err)
}

/// FUS_GET_STATE every 100 ms until the FUS is idle, the stack runs, or
/// the FUS reports an error. A system reset by the FUS ends the helper.
/// With `need_activity` an idle state counts only after a busy state or a
/// 15 s grace period: the FUS acknowledges a request before acting on it.
fn poll(mb: &Mbox, c: &Clock, timeout_ms: u32, need_activity: bool) -> u32 {
    res(R_PHASE, PH_POLLING);
    let t = c.now();
    let mut active = !need_activity;
    loop {
        let (st, _) = mb.state(c);
        match st {
            STATE_IDLE if !active && c.elapsed_ms(t) < 15_000 => {}
            STATE_IDLE | STATE_NOT_RUNNING => return E_NONE,
            STATE_ERROR => return E_FUS_ERROR,
            0x10..=0x3F => active = true,
            _ => {}
        }
        if c.elapsed_ms(t) >= timeout_ms {
            return E_TIMEOUT;
        }
        c.delay_ms(100);
    }
}
