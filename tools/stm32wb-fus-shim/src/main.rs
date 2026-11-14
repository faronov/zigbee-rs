//! Flash-resident STM32WB coprocessor sequencer for boards without a debug
//! probe (the P-NUCLEO-WB55 USB dongle, MB1293C).
//!
//! The ROM USB DFU bootloader (AN3156) programs this helper at
//! `0x0800_0000`. With SW2 off BOOT0 every reset (power, RESET button, or a
//! reset raised by the FUS or the wireless stack) starts it with CPU2
//! stopped (PWR_CR4.C2BOOT clear), the same clean start hwlab's SWD driver
//! uses. Each boot it builds a fresh mailbox, boots CPU2, reads what runs
//! there and takes at most one step of the decision table in `lib.rs`:
//! enter the FUS, upgrade the FUS, delete the stack, install the MAC, start
//! the stack. Then it resets (or the FUS does) and decides again. Every
//! boot and every started action is appended to a flash log page, which
//! caps every loop and is read back over DFU afterwards.
//!
//! An SRAM helper started by `dfu-leave` cannot do this: the bootloader
//! boots the wireless stack itself and resets the IPCC when it leaves DFU,
//! after which CPU2 never serves a command (measured, see README.md).
//!
//! Flash writes: only double words into the log page `LOG`, never an erase,
//! never at or above the secure start; option bytes are never touched.
//! Images are staged over DFU and checked here by CRC-32 before use.
//!
//! LEDs (active high): all three for 0.3 s at each start; blue blinking
//! while working; green steady when MAC 1.24.0 runs; green blinking when
//! the MAC image must be staged over DFU; red steady on any other stop.

#![no_std]
#![no_main]

use core::arch::asm;
use core::panic::PanicInfo;
use core::ptr::{read_volatile, write_volatile};

use stm32wb_fus_shim::{
    Action, CRC_MAC_124, Counts, Decision, FLASH_BASE, FUS_LEN, FUS_STAGE, Kind, LOG, LOG_SLOTS,
    MAC_LEN, MAC_STAGE, MAX_BOOTS, Observed, PAGE, Ready, Staged, Stop, classify_fus_stage, crc32,
    decide, encode, scan,
};

// --- errors (logged as Kind::Error) -----------------------------------------

const E_IPCCBR: u16 = 0x02;
const E_SRAM2A_SECURE: u16 = 0x03;
const E_C2BOOT: u16 = 0x04;
const E_NO_READY: u16 = 0x05;
const E_READY_CODE: u16 = 0x06;
const E_CMD_BUSY: u16 = 0x07;
const E_CMD_TIMEOUT: u16 = 0x08;
const E_CMD_RESPONSE: u16 = 0x09;
const E_DEVINFO: u16 = 0x0A;
const E_REFUSED: u16 = 0x0C;
const E_FUS_ERROR: u16 = 0x0D;
const E_TIMEOUT: u16 = 0x0E;
const E_EVENT_QUEUE: u16 = 0x0F;
const E_RELEASE: u16 = 0x10;
const E_NO_RESET: u16 = 0x11;

/// (code, detail)
type Fault = (u16, u32);

// --- STM32WB registers -------------------------------------------------------

const RCC_CR: u32 = 0x5800_0000;
const RCC_CFGR: u32 = 0x5800_0008;
const RCC_PLLCFGR: u32 = 0x5800_000C;
const RCC_AHB2ENR: u32 = 0x5800_004C;
const RCC_AHB3ENR: u32 = 0x5800_0050;
const RCC_CSR: u32 = 0x5800_0094;
const RCC_RMVF: u32 = 1 << 23;
const HSEMEN: u32 = 1 << 19;
const IPCCEN: u32 = 1 << 20;
const PWR_CR4: u32 = 0x5800_040C;
const C2BOOT: u32 = 1 << 15;

const FLASH_KEYR: u32 = 0x5800_4008;
const FLASH_SR: u32 = 0x5800_4010;
const FLASH_CR: u32 = 0x5800_4014;
const FLASH_IPCCBR: u32 = 0x5800_403C;
const FLASH_SFR: u32 = 0x5800_4080;
const FLASH_SRRVR: u32 = 0x5800_4084;
const SR_BSY: u32 = 1 << 16;
const SR_CFGBSY: u32 = 1 << 18;
const SR_CLEAR: u32 = 0xC3FB;
const SR_ERRORS: u32 = 0xC3FA;
const CR_PG: u32 = 1;
const CR_LOCK: u32 = 1 << 31;

const HSEM_R: u32 = 0x5800_1400;
const HSEM_RLR: u32 = HSEM_R + 0x80;
const HSEM_CPU1: u32 = 4 << 8;
const HSEM_LOCK: u32 = 1 << 31;
const SEM_FLASH: u32 = 2;
const SEM_BLOCK_BY_CPU2: u32 = 7;

const IPCC: u32 = 0x5800_0C00;
const IPCC_C1CR: u32 = IPCC;
const IPCC_C1SCR: u32 = IPCC + 0x08;
const IPCC_C1TOC2SR: u32 = IPCC + 0x0C;
const IPCC_C2TOC1SR: u32 = IPCC + 0x1C;
const CH_SYS: u32 = 1 << 1; // channel 2: system command / event
const CH_FREE: u32 = 1 << 3; // channel 4: buffer release

const GPIOA: u32 = 0x4800_0000;
const GPIOB: u32 = 0x4800_0400;

const SCB_VTOR: u32 = 0xE000_ED08;
const SCB_AIRCR: u32 = 0xE000_ED0C;
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
// Reference-table order: devinfo, BLE, Thread, sys, mem manager, traces,
// MAC, Zigbee, LLD tests, BLE LLD.
const T_SLOTS: [u32; 10] = [
    T_DEVINFO, 0x090, 0x0B0, T_SYS, T_MM, 0x110, 0x130, 0x150, 0x170, 0x190,
];
const FUS_TABLE_KEY: u32 = 0xA946_56B9;

const OP_FUS_GET_STATE: u16 = 0xFC52;
const OP_FUS_FW_UPGRADE: u16 = 0xFC54;
const OP_FUS_FW_DELETE: u16 = 0xFC55;
const OP_FUS_START_WS: u16 = 0xFC5A;
const PKT_SYS_CMD: u8 = 0x10;
const EVT_CMD_DONE: u8 = 0x0E;
const EVT_VENDOR: u8 = 0xFF;
const SUBEVT_READY: u16 = 0x9200;
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

fn flash_bytes(addr: u32, len: u32) -> impl Iterator<Item = u8> + Clone {
    (addr..addr + len).map(rd8)
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
    sp: 0x2003_0000,
    reset: seq_reset,
    nmi: seq_fault,
    hard_fault: seq_fault,
};

/// A fault cannot log (the flash state is unknown): red, and stay.
#[unsafe(no_mangle)]
unsafe extern "C" fn seq_fault() -> ! {
    led(false, false, true);
    loop {
        unsafe { asm!("nop") }
    }
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    unsafe { seq_fault() }
}

// --- timing and LEDs ------------------------------------------------------

struct Clock {
    per_ms: u32,
}

impl Clock {
    fn new() -> Self {
        wr(DEMCR, rd(DEMCR) | (1 << 24));
        wr(DWT_CYCCNT, 0);
        wr(DWT_CTRL, rd(DWT_CTRL) | 1);
        Self {
            per_ms: (hclk() / 1000).max(1),
        }
    }
    fn now(&self) -> u32 {
        rd(DWT_CYCCNT)
    }
    fn elapsed_ms(&self, since: u32) -> u32 {
        self.now().wrapping_sub(since) / self.per_ms
    }
    /// Working indication: blue toggles every 250 ms.
    fn tick(&self) {
        let on = (self.now() / self.per_ms / 250) & 1 != 0;
        wr(GPIOA + 0x18, if on { 1 << 4 } else { 1 << 20 });
    }
    fn delay_ms(&self, ms: u32) {
        let t = self.now();
        while self.elapsed_ms(t) < ms {
            self.tick();
        }
    }
}

/// HCLK1 from the RCC configuration (MSI 4 MHz after reset).
fn hclk() -> u32 {
    const MSI: [u32; 12] = [
        100_000, 200_000, 400_000, 800_000, 1_000_000, 2_000_000, 4_000_000, 8_000_000, 16_000_000,
        24_000_000, 32_000_000, 48_000_000,
    ];
    let cr = rd(RCC_CR);
    let cfgr = rd(RCC_CFGR);
    let msi = MSI[(((cr >> 4) & 0xF) as usize).min(11)];
    let sys = match (cfgr >> 2) & 3 {
        0 => msi,
        1 => 16_000_000,
        2 => 32_000_000,
        _ => {
            let p = rd(RCC_PLLCFGR);
            let src = match p & 3 {
                1 => msi,
                2 => 16_000_000,
                3 => 32_000_000,
                _ => 0,
            };
            src / (((p >> 4) & 7) + 1) * ((p >> 8) & 0x7F) / (((p >> 29) & 7) + 1)
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
    // An undecodable clock must only make timeouts longer.
    match sys / div {
        0 => 64_000_000,
        hz => hz,
    }
}

fn leds_init() {
    wr(RCC_AHB2ENR, rd(RCC_AHB2ENR) | 0b11);
    dsb();
    // PA4 blue, PB0 green, PB1 red: push-pull outputs, low.
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

/// Final indication; the sequencer stays here until the next reset.
fn halt(c: &Clock, stop: Stop) -> ! {
    loop {
        let phase = (c.now() / c.per_ms / 500) & 1 != 0;
        match stop {
            Stop::Done => led(false, true, false),
            Stop::WaitMacStaging => led(false, phase, false),
            _ => led(false, false, true),
        }
    }
}

fn system_reset() -> ! {
    dsb();
    wr(SCB_AIRCR, 0x05FA_0004);
    dsb();
    loop {
        unsafe { asm!("nop") }
    }
}

// --- flash log ----------------------------------------------------------------

struct Log {
    next: u32,
}

impl Log {
    /// Appends one record; false when the page is full or the write failed.
    fn put(&mut self, c: &Clock, kind: Kind, arg: u16, val: u32) -> bool {
        if self.next >= LOG_SLOTS {
            return false;
        }
        let a = LOG + self.next * 8;
        let (w0, w1) = encode(kind, arg, val);
        let ok = program_dword(c, a, w0, w1);
        self.next += 1;
        ok
    }
}

fn wait_clear(c: &Clock, reg: u32, mask: u32, ms: u32) -> bool {
    let t = c.now();
    while rd(reg) & mask != 0 {
        if c.elapsed_ms(t) >= ms {
            return false;
        }
    }
    true
}

/// Programs one erased double word in user flash below the secure start,
/// under the flash semaphore (AN5289: HSEM 2, and no CPU2 block on HSEM 7).
fn program_dword(c: &Clock, addr: u32, w0: u32, w1: u32) -> bool {
    let secure = FLASH_BASE + (rd(FLASH_SFR) & 0xFF) * PAGE;
    if !addr.is_multiple_of(8) || addr < LOG || addr + 8 > LOG + PAGE || addr + 8 > secure {
        return false;
    }
    if rd(addr) != u32::MAX || rd(addr + 4) != u32::MAX {
        return false;
    }
    wr(RCC_AHB3ENR, rd(RCC_AHB3ENR) | HSEMEN);
    dsb();
    let t = c.now();
    while rd(HSEM_RLR + 4 * SEM_FLASH) != HSEM_LOCK | HSEM_CPU1 {
        if c.elapsed_ms(t) >= 1000 {
            return false;
        }
    }
    let ok = wait_clear(c, HSEM_R + 4 * SEM_BLOCK_BY_CPU2, HSEM_LOCK, 2000)
        && wait_clear(c, FLASH_SR, SR_BSY | SR_CFGBSY, 1000)
        && {
            if rd(FLASH_CR) & CR_LOCK != 0 {
                wr(FLASH_KEYR, 0x4567_0123);
                wr(FLASH_KEYR, 0xCDEF_89AB);
            }
            wr(FLASH_SR, SR_CLEAR);
            wr(FLASH_CR, rd(FLASH_CR) | CR_PG);
            dsb();
            wr(addr, w0);
            wr(addr + 4, w1);
            dsb();
            let done = wait_clear(c, FLASH_SR, SR_BSY | SR_CFGBSY, 100);
            let err = rd(FLASH_SR) & SR_ERRORS;
            wr(FLASH_CR, (rd(FLASH_CR) & !CR_PG) | CR_LOCK);
            done && err == 0
        };
    wr(HSEM_R + 4 * SEM_FLASH, HSEM_CPU1);
    dsb();
    ok && rd(addr) == w0 && rd(addr + 4) == w1
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
        wr(RCC_AHB3ENR, rd(RCC_AHB3ENR) | IPCCEN);
        dsb();
        wr(IPCC_C1CR, 0);
        wr(IPCC_C1SCR, 0x3F);
        dsb();
        wr(PWR_CR4, rd(PWR_CR4) | C2BOOT);
        dsb();
        Self {
            cmdbuf: base + T_CMDBUF,
            sysq: base + T_SYSQ,
            freeq: base + T_FREEQ,
        }
    }

    /// One system event as (event code, vendor sub-event, first payload
    /// byte), or None after `timeout_ms`.
    fn event(&self, c: &Clock, timeout_ms: u32) -> Result<Option<(u8, u16, u8)>, Fault> {
        let t = c.now();
        loop {
            while rd(IPCC_C2TOC1SR) & CH_SYS == 0 {
                c.tick();
                if c.elapsed_ms(t) >= timeout_ms {
                    return Ok(None);
                }
            }
            let head = self.sysq;
            let node = rd(head);
            if node == head {
                wr(IPCC_C1SCR, CH_SYS); // flag without an event
                continue;
            }
            if !in_sram2a(node, 0x10) {
                return Err((E_EVENT_QUEUE, node));
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
            self.release(c, node)?;
            if rd(head) == head {
                wr(IPCC_C1SCR, CH_SYS);
            }
            return Ok(Some((code, sub, first)));
        }
    }

    fn release(&self, c: &Clock, node: u32) -> Result<(), Fault> {
        if !wait_clear(c, IPCC_C1TOC2SR, CH_FREE, 1000) {
            return Err((E_RELEASE, node));
        }
        let q = self.freeq;
        let tail = rd(q + 4);
        wr(node, q);
        wr(node + 4, tail);
        wr(q + 4, node);
        wr(tail, node);
        dsb();
        wr(IPCC_C1SCR, CH_FREE << 16);
        Ok(())
    }

    /// Waits for the ready event; returns its code (0 = wireless stack,
    /// 1 = FUS).
    fn ready(&self, c: &Clock, timeout_ms: u32) -> Result<Ready, Fault> {
        let t = c.now();
        loop {
            let left = timeout_ms.saturating_sub(c.elapsed_ms(t));
            match self.event(c, left)? {
                None => return Err((E_NO_READY, rd(IPCC_C2TOC1SR))),
                Some((EVT_VENDOR, SUBEVT_READY, 0)) => return Ok(Ready::WirelessStack),
                Some((EVT_VENDOR, SUBEVT_READY, 1)) => return Ok(Ready::Fus),
                Some((EVT_VENDOR, SUBEVT_READY, code)) => return Err((E_READY_CODE, code as u32)),
                Some(_) => {}
            }
        }
    }

    /// Sends one system command; returns the first four bytes of the
    /// command-complete payload after the opcode (status first, 0xFF
    /// beyond the payload) and the payload length.
    fn command(&self, c: &Clock, op: u16) -> Result<([u8; 4], u32), Fault> {
        if rd(IPCC_C1TOC2SR) & CH_SYS != 0 {
            return Err((E_CMD_BUSY, op as u32));
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
            c.tick();
            if c.elapsed_ms(t) >= 5000 {
                return Err((E_CMD_TIMEOUT, op as u32 | rd(IPCC_C2TOC1SR) << 16));
            }
        }
        dsb();
        let b = self.cmdbuf;
        let rop = rd8(b + 4) as u16 | (rd8(b + 5) as u16) << 8;
        let len = rd8(b + 2);
        if rd8(b + 1) != EVT_CMD_DONE || rop != op || len < 3 {
            return Err((E_CMD_RESPONSE, rd(b)));
        }
        let n = (len - 3) as u32;
        let mut p = [0xFFu8; 4];
        for i in 0..n.min(4) {
            p[i as usize] = rd8(b + 6 + i);
        }
        Ok((p, n))
    }

    /// FUS_GET_STATE as (state, error).
    fn state(&self, c: &Clock) -> Result<(u8, u8), Fault> {
        let (p, n) = self.command(c, OP_FUS_GET_STATE)?;
        if n == 0 {
            return Err((E_CMD_RESPONSE, OP_FUS_GET_STATE as u32));
        }
        Ok((p[0], if n > 1 { p[1] } else { 0 }))
    }
}

/// FUS_GET_STATE every 100 ms until the FUS is idle, not running, or in
/// error. A system reset by the FUS ends the helper. With `need_activity`
/// an idle state counts only after a busy state or 15 s: the FUS
/// acknowledges a request before it acts on it. Returns (state, error,
/// elapsed ms).
fn poll(
    mb: &Mbox,
    c: &Clock,
    timeout_ms: u32,
    need_activity: bool,
) -> Result<(u8, u8, u32), Fault> {
    let t = c.now();
    let mut active = !need_activity;
    loop {
        let (st, ec) = mb.state(c)?;
        let el = c.elapsed_ms(t);
        match st {
            STATE_IDLE if !active && el < 15_000 => {}
            STATE_IDLE | STATE_NOT_RUNNING | STATE_ERROR => return Ok((st, ec, el)),
            0x10..=0x3F => active = true,
            _ => {}
        }
        if el >= timeout_ms {
            return Err((E_TIMEOUT, st as u32 | (ec as u32) << 8));
        }
        c.delay_ms(100);
    }
}

// --- entry ------------------------------------------------------------------

#[unsafe(no_mangle)]
unsafe extern "C" fn seq_reset() -> ! {
    unsafe {
        asm!("cpsid i", options(nomem, nostack, preserves_flags));
    }
    wr(SCB_VTOR, &VECTORS as *const Vectors as u32);
    dsb();
    leds_init();
    led(true, true, true);
    let c = Clock::new();
    let t = c.now();
    while c.elapsed_ms(t) < 300 {}
    led(false, false, false);

    let (counts, free) = scan((0..LOG_SLOTS).map(|i| (rd(LOG + i * 8), rd(LOG + i * 8 + 4))));
    let Some(next) = free else {
        // Full or foreign log page: nothing can be recorded.
        halt(&c, Stop::Unexpected)
    };
    let mut log = Log { next };
    let csr = rd(RCC_CSR);
    // CPU2 is still stopped: the first record is written with it halted.
    let cr4 = rd(PWR_CR4);
    if !log.put(&c, Kind::Boot, counts.boots, csr) {
        halt(&c, Stop::Unexpected)
    }
    wr(RCC_CSR, csr | RCC_RMVF);
    let stop = match step(&c, &mut log, counts, cr4) {
        Ok(stop) => stop,
        Err((code, detail)) => {
            log.put(&c, Kind::Error, code, detail);
            Stop::Unexpected
        }
    };
    log.put(&c, Kind::Stop, stop as u16, 0);
    halt(&c, stop)
}

/// One boot: observe, decide, act. Returns only to stop; an action that
/// completes without a reset by the coprocessor resets the system so the
/// next boot decides again from a fresh mailbox.
fn step(c: &Clock, log: &mut Log, counts: Counts, cr4: u32) -> Result<Stop, Fault> {
    if counts.boots >= MAX_BOOTS {
        return Ok(Stop::Cap);
    }
    if cr4 & C2BOOT != 0 {
        return Err((E_C2BOOT, cr4));
    }
    let ipccbr = rd(FLASH_IPCCBR);
    if ipccbr & 0x3FFF != 0 {
        return Err((E_IPCCBR, ipccbr));
    }
    let srrvr = rd(FLASH_SRRVR);
    let secure2a = if srrvr >> 23 & 1 != 0 {
        SRAM2A_END
    } else {
        SRAM2A + ((srrvr >> 18 & 0x1F) << 10)
    };
    if SRAM2A + T_END > secure2a {
        return Err((E_SRAM2A_SECURE, srrvr));
    }

    // Flash checks before CPU2 runs. The MAC region is read only when it
    // lies wholly below the secure start (CPU1 cannot read above it).
    let sfsa = rd(FLASH_SFR) & 0xFF;
    let secure = FLASH_BASE + sfsa * PAGE;
    let fus_stage = if FUS_STAGE + FUS_LEN <= secure {
        classify_fus_stage(flash_bytes(FUS_STAGE, FUS_LEN))
    } else {
        Staged::Other
    };
    c.tick();
    let mac_staged =
        MAC_STAGE + MAC_LEN <= secure && crc32(flash_bytes(MAC_STAGE, MAC_LEN)) == CRC_MAC_124;
    c.tick();

    let mb = Mbox::boot();
    let ready = mb.ready(c, 20_000)?;
    let di = rd(SRAM2A + T_REF);
    if !in_sram2a(di, 0x30) {
        return Err((E_DEVINFO, di));
    }
    let w = |o: u32| rd(di + o);
    let mut o = Observed {
        ready,
        sfsa,
        fus_version: 0,
        stack_version: 0,
        info_stack: 0,
        fus_state: 0,
        fus_error: 0,
        fus_stage,
        mac_staged,
        counts,
    };
    match ready {
        Ready::Fus => {
            if w(0) != FUS_TABLE_KEY {
                return Err((E_DEVINFO, w(0)));
            }
            o.fus_version = w(12);
            o.stack_version = w(20);
            // The FUS may resume an operation after its own reset.
            let (st, ec, el) = poll(&mb, c, 300_000, false)?;
            log.put(c, Kind::State, st as u16 | (ec as u16) << 8, el);
            o.fus_state = st;
            o.fus_error = ec;
        }
        Ready::WirelessStack => {
            o.fus_version = w(4);
            o.stack_version = w(16);
            o.info_stack = w(24);
        }
    }
    log.put(c, Kind::Ready, ready as u16 | (sfsa as u16) << 8, w(0));
    log.put(
        c,
        Kind::FusVersion,
        fus_stage as u16 | (mac_staged as u16) << 8,
        o.fus_version,
    );
    log.put(c, Kind::StackVersion, o.info_stack as u16, o.stack_version);

    let action = match decide(&o) {
        Decision::Stop(s) => return Ok(s),
        Decision::Act(a) => a,
    };
    // Logged before the command so a reset in the middle still counts it.
    if !log.put(c, Kind::Action, action as u16, 0) {
        return Ok(Stop::Unexpected);
    }
    match action {
        Action::EnterFus => {
            // Two FUS_GET_STATE make the wireless stack reboot into the FUS
            // (AN5185); that is a system reset.
            mb.state(c)?;
            mb.state(c)?;
            c.delay_ms(15_000);
            // No reboot into the FUS: retry from a reset (capped by the log).
            log.put(c, Kind::Error, E_NO_RESET, action as u32);
            system_reset()
        }
        Action::Reset => system_reset(),
        Action::StartWs => {
            let (p, _) = mb.command(c, OP_FUS_START_WS)?;
            if p[0] != 0 {
                return Err((E_REFUSED, u32::from_le_bytes(p)));
            }
            // Never FUS_GET_STATE after START_WS (two would re-enter the FUS).
            c.delay_ms(30_000);
            Err((E_NO_RESET, action as u32))
        }
        Action::UpgradeFus | Action::DeleteStack | Action::InstallMac => {
            let op = if action == Action::DeleteStack {
                OP_FUS_FW_DELETE
            } else {
                OP_FUS_FW_UPGRADE
            };
            let (p, _) = mb.command(c, op)?;
            if p[0] != 0 {
                return Err((E_REFUSED, u32::from_le_bytes(p)));
            }
            if action == Action::DeleteStack {
                // A reset before the table reports no stack would abort the
                // delete (hwlab's SWD driver waits the same way).
                let t = c.now();
                while rd(di + 20) != 0 {
                    if c.elapsed_ms(t) >= 300_000 {
                        return Err((E_TIMEOUT, rd(di + 20)));
                    }
                    c.delay_ms(100);
                }
            }
            let (st, ec, el) = poll(&mb, c, 300_000, action != Action::DeleteStack)?;
            log.put(c, Kind::State, st as u16 | (ec as u16) << 8, el);
            if st == STATE_ERROR {
                return Err((E_FUS_ERROR, st as u32 | (ec as u32) << 8));
            }
            // Finished without a reset: decide again from a clean start.
            system_reset()
        }
    }
}
