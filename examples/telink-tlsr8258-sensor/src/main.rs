//! # Telink TLSR8258 Zigbee Temperature Sensor — Pure Rust
//!
//! A `no_std` firmware for the Telink TLSR8258 (tc32 ISA).
//! Zero C dependencies, zero vendor SDK. Everything is Rust.
//!
//! ## Hardware
//! - TB-04-Kit: Telink TLSR8258 (tc32 core, 512KB Flash, 64KB SRAM)
//! - RGB LED: PC4 (Red), PC1 (Green), PB5 (Blue)
//! - Built-in IEEE 802.15.4 + BLE radio
//!
//! ## Building
//! ```bash
//! TC=/tmp/tc32-toolchain/tc32-stage2-x86_64-apple-darwin
//! cd zigbee-rs-fork/examples/telink-tlsr8258-sensor
//! $TC/bin/cargo build --release
//! ```

#![no_std]
#![no_main]

use panic_halt as _;

use zigbee_aps::frames::{
    ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader,
};
use zigbee_aps::{PROFILE_ZDP, ZDO_ENDPOINT};
use zigbee_mac::{MacDriver, MacError};
use zigbee_nwk::frames::{NwkFrameControl, NwkFrameType, NwkHeader};
use zigbee_types::{MacAddress, ShortAddress};
use zigbee_zdo::device_announce::DeviceAnnounce;
use zigbee_zdo::DEVICE_ANNCE;

// Keep debug SRAM well below the stack top (0x00850000). The heavier
// sensor-lite interview path uses enough stack that the old 0x00848C00/0x00848D00
// markers were getting clobbered.
const DBG_BOOT_BASE: u32 = 0x00848400;
const DBG_MODE_BASE: u32 = 0x00848500;


// ── Boot vector table (tc32 ISA — native mnemonics) ──────────
//
// The tc32 LLVM assembler accepts standard Thumb-1 mnemonics and
// encodes them as tc32 opcodes automatically. No more .short!
//
//   0x00: tj __reset    (tc32 unconditional branch)
//   0x08: "KNLT" magic
//   0x0C: 0x00880000 + binary_size/16
//   0x10: tj __irq_stub (tc32 unconditional branch)
//   0x18: binary size
//   0x20: __irq_stub: bx lr
//   0x30: __reset: set SP, jump to Rust _start
core::arch::global_asm!(
    ".section .vectors, \"ax\"",
    ".balign 4",
    ".global _reset_vector",
    "_reset_vector:",
    // ─── Offset 0x00: Header ───
    "tj __reset",
    ".short 0x0000",            // 0x02: file version (0)
    ".word 0x00000000",         // 0x04: reserved
    ".byte 0x4B, 0x4E, 0x4C, 0x54",  // 0x08: "KNLT" magic
    ".word 0x00880000 + _bin_size_div_16",
    "tj __irq",
    ".short 0x0000", ".short 0x0000", ".short 0x0000",
    ".word _bin_size_",
    ".word 0x00000000",         // 0x1C: reserved
    //
    // ─── Offset 0x20: IRQ entry point ───
    // tc32 ISA requires special save/restore via .short opcodes:
    //   0x6BD8 = tpush (save IRQ context)
    //   0x6BD0 = tpop  (restore IRQ context)
    //   0x6900 = treturn (return from IRQ)
    ".globl __irq",
    "__irq:",
    "push {{lr}}",
    "push {{r0, r1, r2, r3, r4, r5, r6, r7}}",
    ".short 0x6BD8",              // tc32: save IRQ context (PSW etc)
    "mov r1, r8",
    "mov r2, r9",
    "mov r3, r10",
    "mov r4, r11",
    "mov r5, r12",
    "push {{r0, r1, r2, r3, r4, r5}}",
    "bl irq_handler",
    "pop {{r0, r1, r2, r3, r4, r5}}",
    "mov r8, r1",
    "mov r9, r2",
    "mov r10, r3",
    "mov r11, r4",
    "mov r12, r5",
    ".short 0x6BD0",              // tc32: restore IRQ context
    "pop {{r0, r1, r2, r3, r4, r5, r6, r7}}",
    ".short 0x6900",              // tc32: return from IRQ
    //
    // ─── __reset ───
    //
    // Official Telink cstartup_8258.S init sequence.
    // Requires tc32-27+ toolchain (ldr= pseudo-instruction support).
    //
    "__reset:",

    // ── 1. Set SVC mode stack pointer ──
    "movs r0, #0x85",
    "lsls r0, r0, #16",       // r0 = 0x850000
    "mov sp, r0",

    // ── 2. Zero I-cache tags (0x840000, 256 bytes = 64 words) ──
    // Unrolled: tc32 LLVM backend has backward branch bugs
    // str Rd,[Rn,#imm] supports imm 0..124 (5-bit * 4)
    "movs r0, #0",
    "ldr r1, =0x840000",
    "str r0, [r1, #0]",   "str r0, [r1, #4]",   "str r0, [r1, #8]",   "str r0, [r1, #12]",
    "str r0, [r1, #16]",  "str r0, [r1, #20]",  "str r0, [r1, #24]",  "str r0, [r1, #28]",
    "str r0, [r1, #32]",  "str r0, [r1, #36]",  "str r0, [r1, #40]",  "str r0, [r1, #44]",
    "str r0, [r1, #48]",  "str r0, [r1, #52]",  "str r0, [r1, #56]",  "str r0, [r1, #60]",
    "str r0, [r1, #64]",  "str r0, [r1, #68]",  "str r0, [r1, #72]",  "str r0, [r1, #76]",
    "str r0, [r1, #80]",  "str r0, [r1, #84]",  "str r0, [r1, #88]",  "str r0, [r1, #92]",
    "str r0, [r1, #96]",  "str r0, [r1, #100]", "str r0, [r1, #104]", "str r0, [r1, #108]",
    "str r0, [r1, #112]", "str r0, [r1, #116]", "str r0, [r1, #120]", "str r0, [r1, #124]",
    "adds r1, #128",
    "str r0, [r1, #0]",   "str r0, [r1, #4]",   "str r0, [r1, #8]",   "str r0, [r1, #12]",
    "str r0, [r1, #16]",  "str r0, [r1, #20]",  "str r0, [r1, #24]",  "str r0, [r1, #28]",
    "str r0, [r1, #32]",  "str r0, [r1, #36]",  "str r0, [r1, #40]",  "str r0, [r1, #44]",
    "str r0, [r1, #48]",  "str r0, [r1, #52]",  "str r0, [r1, #56]",  "str r0, [r1, #60]",
    "str r0, [r1, #64]",  "str r0, [r1, #68]",  "str r0, [r1, #72]",  "str r0, [r1, #76]",
    "str r0, [r1, #80]",  "str r0, [r1, #84]",  "str r0, [r1, #88]",  "str r0, [r1, #92]",
    "str r0, [r1, #96]",  "str r0, [r1, #100]", "str r0, [r1, #104]", "str r0, [r1, #108]",
    "str r0, [r1, #112]", "str r0, [r1, #116]", "str r0, [r1, #120]", "str r0, [r1, #124]",

    // ── 3. Set I-cache config (0x80060C/0D) ──
    // No RAM code section, so ramcode_size = 0
    "ldr r1, =0x80060C",
    "movs r0, #0",
    "strb r0, [r1, #0]",      // 0x80060C = 0
    "movs r0, #1",
    "strb r0, [r1, #1]",      // 0x80060D = 1

    // ── 4. System power ON (0x800060) ──
    "ldr r1, =0x800060",
    "ldr r0, =0xFF000000",
    "str r0, [r1, #0]",       // *(u32*)0x800060 = 0xFF000000
    "movs r0, #0xFF",
    "strb r0, [r1, #4]",      // 0x800064 = 0xFF
    "strb r0, [r1, #5]",      // 0x800065 = 0xFF

    // ── 5. Flash deep wakeup (0xAB via SPI at 0x80000C) ──
    "ldr r1, =0x80000C",
    "movs r0, #0",
    "strb r0, [r1, #1]",      // CS low
    "movs r0, #0xAB",
    "strb r0, [r1, #0]",      // Flash wakeup cmd
    "nop", "nop", "nop", "nop", "nop", "nop",
    "movs r0, #1",
    "strb r0, [r1, #1]",      // CS high

    // ── 6. LED checkpoint: turn on RED to prove __reset completed ──
    // PC_OEN (0x800592): clear bit 1 (PC1=RED output)
    "ldr r1, =0x800592",
    "ldrb r0, [r1, #0]",
    "movs r2, #0x02",
    "bics r0, r2",            // clear bit 1
    "strb r0, [r1, #0]",
    // PC_OUT (0x800593): set bit 1 (RED on)
    "movs r0, #0x02",
    "strb r0, [r1, #1]",

    // ── 6b. Write ASM markers to SRAM (in real SRAM, above BSS) ──
    "ldr r0, =0x848C00",
    "ldr r1, =0xDEADBEEF",
    "str r1, [r0, #0]",      // DBG_BOOT_BASE + 0x00 = DEADBEEF
    "ldr r1, =0x12345678",
    "str r1, [r0, #4]",      // DBG_BOOT_BASE + 0x04 = 12345678

    // ── 7. Jump to Rust _start ──
    "tj _start",
);

// ── Pure Rust startup ──────────────────────────────────────────

unsafe extern "C" {
    static mut _sdata: u8;
    static mut _edata: u8;
    static mut _sbss: u8;
    static mut _ebss: u8;
    static _etext: u8;
    static _stack_top: u8;
    // RAM-code section (HAL functions that must run from RAM)
    static mut _ramcode_start_: u8;
    static mut _ramcode_end_: u8;
    static _ramcode_stored_: u8;
}

/// Entry point — called from __reset after SP setup.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _start() -> ! {
    unsafe {
        // == EARLY MARKER: prove _start ran ==
        mark32(DBG_BOOT_BASE + 0x00, 0xAAAA0001_u32);

        // Disable watchdog
        core::ptr::write_volatile((REG_BASE + 0x622) as *mut u8, 0x00);

        // Copy .ram_code from flash to RAM
        let rc_len = &raw const _ramcode_end_ as usize - &raw const _ramcode_start_ as usize;
        let rc_src = &raw const _ramcode_stored_ as *const u8;
        let rc_dst = &raw mut _ramcode_start_ as *mut u8;
        if rc_len > 0 {
            core::ptr::copy_nonoverlapping(rc_src, rc_dst, rc_len);
        }

        // Copy .data from flash to RAM
        let data_len = &raw const _edata as usize - &raw const _sdata as usize;
        if data_len > 0 {
            let src = &raw const _etext as *const u8;
            let dst = &raw mut _sdata as *mut u8;
            core::ptr::copy_nonoverlapping(src, dst, data_len);
        }

        // Zero .bss
        let bss_len = &raw const _ebss as usize - &raw const _sbss as usize;
        if bss_len > 0 {
            let bss = &raw mut _sbss as *mut u8;
            core::ptr::write_bytes(bss, 0, bss_len);
        }

        mark32(DBG_BOOT_BASE + 0x04, 0xAAAA0002_u32);
    }

    mark32(DBG_BOOT_BASE + 0x08, 0xAAAA0003_u32);
    chip_init();
    mark32(DBG_BOOT_BASE + 0x0C, 0xAAAA0004_u32);
    main_loop();
}

// ── Minimal chip init (replaces HAL startup) ───────────────────
//
// Direct MMIO based on Telink C SDK. No HAL dependency for boot.

#[inline(never)]
fn chip_init() {
    // LED setup for debug checkpoints
    let pc_oen = (REG_BASE + 0x592) as *mut u8;
    let pc_out = (REG_BASE + 0x593) as *mut u8;
    let pb_oen = (REG_BASE + 0x58A) as *mut u8;
    let pb_out = (REG_BASE + 0x58B) as *mut u8;

    unsafe {
        // Enable LED pins as outputs
        core::ptr::write_volatile(pc_oen, core::ptr::read_volatile(pc_oen as *const u8) & !0x12);
        core::ptr::write_volatile(pb_oen, core::ptr::read_volatile(pb_oen as *const u8) & !0x20);
    }
    mark32(DBG_BOOT_BASE + 0x10, 0xC1A00001);

    // ── Step 1: RED = disable IRQ + reset peripherals ──
    set_led(pc_out, pb_out, true, false, false);

    // Disable global IRQ
    unsafe {
        core::ptr::write_volatile((REG_BASE + 0x643) as *mut u8, 0);
    }
    // Reset control: release all peripherals from reset
    unsafe {
        core::ptr::write_volatile((REG_BASE + 0x060) as *mut u8, 0x00);
        core::ptr::write_volatile((REG_BASE + 0x061) as *mut u8, 0x00);
        core::ptr::write_volatile((REG_BASE + 0x062) as *mut u8, 0x00);
        core::ptr::write_volatile((REG_BASE + 0x063) as *mut u8, 0xFF);
        core::ptr::write_volatile((REG_BASE + 0x064) as *mut u8, 0xFF);
        core::ptr::write_volatile((REG_BASE + 0x065) as *mut u8, 0xFF);
    }

    // SKIP analog init — go straight to timer setup
    set_led(pc_out, pb_out, false, false, true); // BLUE = timer setup

    // ── Analog register init (from cpu_wakeup_init disassembly) ──
    analog_write(0x82, 0x64); // PM register
    analog_write(0x34, 0x80); // Bandgap calibration
    analog_write(0x06, 0x00); // Power on digital/analog/baseband
    analog_write(0x0a, 0x44); // XTAL cap (24MHz internal cap)
    analog_write(0x0b, 0x38); // 48M doubler enable
    analog_write(0x05, 0x02); // Power up 24M XTAL oscillator
    analog_write(0x8c, 0x02); // XTAL bias current
    analog_write(0x02, 0xa2); // LDO voltage for RF/analog
    // Clear wakeup status registers
    analog_write(0x27, 0x00);
    analog_write(0x28, 0x00);
    analog_write(0x29, 0x00);
    analog_write(0x2a, 0x00);
    analog_write(0x01, 0x4c); // Power control

    // Wait for crystal to stabilize (~600µs at 24MHz RC = ~14400 cycles)
    for _ in 0..5000_u32 {
        unsafe { core::arch::asm!("nop", "nop", "nop", "nop"); }
    }

    // ── Clock source: switch to 24MHz Crystal ──
    // 0x42 = SYS_CLK_24M_Crystal (from Telink SDK clock.h)
    unsafe {
        core::ptr::write_volatile((REG_BASE + 0x066) as *mut u8, 0x42);
    }

    // Small delay after clock switch for stability
    for _ in 0..1000_u32 {
        unsafe { core::arch::asm!("nop", "nop"); }
    }

    // Now re-enable all clock gates (crystal should allow ZB modem clock)
    unsafe {
        core::ptr::write_volatile((REG_BASE + 0x063) as *mut u8, 0xFF); // CLK_EN0
        core::ptr::write_volatile((REG_BASE + 0x064) as *mut u8, 0xFF); // CLK_EN1
        core::ptr::write_volatile((REG_BASE + 0x065) as *mut u8, 0xFF); // CLK_EN2 (ZB modem!)
    }
    mark32(DBG_BOOT_BASE + 0x14, 0xC1A00002);

    // ── System tick: try starting it ──
    unsafe {
        let tick_ctrl = (REG_BASE + 0x74A) as *mut u8;
        core::ptr::write_volatile(tick_ctrl, 0x01); // START
    }

    // ── Timer0: free-running mode as backup time source ──
    // Timer0 tick at 0x800630 (32-bit), capture at 0x800620
    // Timer control at 0x80062A: bit 0 = TMR0_EN, bits 1:2 = mode (0=free-run)
    unsafe {
        // Set capture to max (free-running)
        core::ptr::write_volatile((REG_BASE + 0x620) as *mut u32, 0xFFFF_FFFF);
        // Clear timer0 tick
        core::ptr::write_volatile((REG_BASE + 0x630) as *mut u32, 0);
        // Enable timer0, free-running mode (bits 1:2 = 0)
        let tmr_ctrl = (REG_BASE + 0x62A) as *mut u8;
        let v = core::ptr::read_volatile(tmr_ctrl as *const u8);
        core::ptr::write_volatile(tmr_ctrl, v | 0x01); // TMR0_EN
    }

    // Enable Timer0 IRQ in mask register (bit 0 = TMR0)
    unsafe {
        let irq_mask = 0x800640 as *mut u32;
        let v = core::ptr::read_volatile(irq_mask as *const u32);
        core::ptr::write_volatile(irq_mask, v | (1 << 0)); // FLD_IRQ_TMR0_EN
    }
    mark32(DBG_BOOT_BASE + 0x18, 0xC1A00003);

    // ── Radio init: 802.15.4 Zigbee mode (custom, no HAL) ──
    radio::init();
    mark32(DBG_BOOT_BASE + 0x1C, 0xC1A00004);

    // Enable global IRQ
    unsafe {
        core::ptr::write_volatile(0x800643 as *mut u8, 1); // REG_IRQ_EN
    }

    set_led(pc_out, pb_out, false, true, false); // GREEN = init complete
}

#[inline(always)]
fn mark32(addr: u32, val: u32) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, val);
    }
}

#[inline(always)]
fn clear_words(base: u32, words: usize) {
    for idx in 0..words {
        mark32(base + (idx as u32 * 4), 0);
    }
}

#[inline(always)]
fn spin_delay(iterations: u32) {
    for _ in 0..iterations {
        unsafe { core::arch::asm!("nop"); }
    }
}

/// IRQ handler — called from assembly __irq via `bl irq_handler`
#[unsafe(no_mangle)]
#[unsafe(link_section = ".ram_code")]
pub extern "C" fn irq_handler() {
    unsafe {
        let irq_src = core::ptr::read_volatile(0x800648 as *const u32); // REG_IRQ_SRC
        let irq_mask = core::ptr::read_volatile(0x800640 as *const u32); // REG_IRQ_MASK
        let pending = irq_src & irq_mask;

        // Timer0 IRQ (bit 0)
        if pending & (1 << 0) != 0 {
            core::ptr::write_volatile(0x800648 as *mut u32, 1 << 0); // ack IRQ source
            core::ptr::write_volatile(0x800623 as *mut u8, 0x01);    // ack timer status
            async_timer::on_alarm_irq();
        }
    }
}

// ── Radio DMA buffers (must be 4-byte aligned, in RAM) ─────────

#[repr(align(4))]
#[allow(dead_code)]
struct DmaBuf([u8; 144]);

static mut RF_RX_BUF: DmaBuf = DmaBuf([0u8; 144]);
static mut RF_TX_BUF: DmaBuf = DmaBuf([0u8; 144]);

/// Write PHY register tables extracted from Telink SDK libdrivers_8258.a rf_drv_init().
/// This is the missing Zigbee PHY init that the HAL radio.rs doesn't include.
fn rf_phy_init_zigbee() {
    unsafe {
        // Diagnostic: mark that PHY init started
        mark32(DBG_BOOT_BASE + 0x10, 0xDEAD0001_u32);

        // Re-enable ALL Zigbee clocks (reset_baseband may have cleared some)
        let clk2_before = core::ptr::read_volatile(0x800065 as *const u8);
        core::ptr::write_volatile(0x800065 as *mut u8, 0xFF); // CLK_EN2: all ZB clocks
        let clk2_after = core::ptr::read_volatile(0x800065 as *const u8);
        // Also clear any ZB resets
        core::ptr::write_volatile(0x800060 as *mut u8, 0x00); // RST0
        core::ptr::write_volatile(0x800061 as *mut u8, 0x00); // RST1
        core::ptr::write_volatile(0x800062 as *mut u8, 0x00); // RST2
        // Also write 0xFF to CLK_EN0 and CLK_EN1
        core::ptr::write_volatile(0x800063 as *mut u8, 0xFF); // CLK_EN0
        core::ptr::write_volatile(0x800064 as *mut u8, 0xFF); // CLK_EN1
        // Store clock diag
        mark32(DBG_BOOT_BASE + 0x20,
            (clk2_before as u32) | ((clk2_after as u32) << 8));

        // tbl_rf_init — common RF PHY init (6 entries)
        reg8w(0x8012D2, 0x9B);
        reg8w(0x8012D3, 0x19);
        reg8w(0x80127B, 0x0E);
        reg8w(0x801276, 0x50);
        reg8w(0x801277, 0x73);
        reg8w(0x800430, 0x3E);

        // tbl_rf_zigbee_250k — Zigbee 250K mode PHY (28 entries)
        // PHY modem/AGC registers (0x801220+)
        reg8w(0x801220, 0x04);
        reg8w(0x801221, 0x2B);
        reg8w(0x801222, 0x43);
        reg8w(0x801223, 0x86);
        reg8w(0x80122A, 0x90);
        // PHY filter registers (0x801254+)
        reg8w(0x801254, 0x0E);
        reg8w(0x801255, 0x09);
        reg8w(0x801256, 0x0C);
        reg8w(0x801257, 0x08);
        reg8w(0x801258, 0x09);
        reg8w(0x801259, 0x0F);
        // RF control registers (0x800400+)
        reg8w(0x800400, 0x13);  // RF mode cfg
        reg8w(0x800420, 0x18);  // RX settle
        reg8w(0x800402, 0x46);
        reg8w(0x800404, 0xC0);
        reg8w(0x800405, 0x04);  // access code length = 4
        reg8w(0x800421, 0x23);
        reg8w(0x800422, 0x04);
        reg8w(0x800408, 0xA7);
        reg8w(0x800409, 0x00);
        reg8w(0x80040A, 0x00);
        reg8w(0x80040B, 0x00);
        // Settle timing registers (0x800460+)
        reg8w(0x800460, 0x36);
        reg8w(0x800461, 0x46);
        reg8w(0x800462, 0x51);
        reg8w(0x800463, 0x61);
        reg8w(0x800464, 0x6D);
        reg8w(0x800465, 0x78);

        // Enable DMA channels 2+3 (RF RX/TX) — from rf_drv_init
        let dma_en = core::ptr::read_volatile(0x800C20 as *const u8);
        core::ptr::write_volatile(0x800C20 as *mut u8, dma_en | 0x0C);

        // Diagnostic: readback PHY registers to RAM for SWire verification
        let r0400 = core::ptr::read_volatile(0x800400 as *const u8);
        let r1220 = core::ptr::read_volatile(0x801220 as *const u8);
        let r12d2 = core::ptr::read_volatile(0x8012D2 as *const u8);
        let r0460 = core::ptr::read_volatile(0x800460 as *const u8);
        mark32(DBG_BOOT_BASE + 0x14,
            (r0400 as u32) | ((r1220 as u32) << 8) | ((r12d2 as u32) << 16) | ((r0460 as u32) << 24));
        // Also test: write 0xAB to 0x800400, read back
        core::ptr::write_volatile(0x800400 as *mut u8, 0xAB);
        let test_rb = core::ptr::read_volatile(0x800400 as *const u8);
        // And test a known-working reg: write 0x55 to unused RAM, read back
        core::ptr::write_volatile((DBG_BOOT_BASE + 0x30) as *mut u8, 0x55);
        let ram_rb = core::ptr::read_volatile((DBG_BOOT_BASE + 0x30) as *const u8);
        mark32(DBG_BOOT_BASE + 0x24,
            (test_rb as u32) | ((ram_rb as u32) << 8));
        // Restore 0x800400 to correct value
        core::ptr::write_volatile(0x800400 as *mut u8, 0x13);
        // Mark PHY init complete
        mark32(DBG_BOOT_BASE + 0x04, 0xBBBB0002_u32);
    }
}

/// Helper: write byte to MMIO register
#[inline(always)]
unsafe fn reg8w(addr: u32, val: u8) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u8, val);
    }
}

// ── Custom radio driver (direct MMIO) ──────────────────────────
//
// Direct register access based on
// Telink SDK rf_drv.h inline functions and disassembled rf_set_channel.

mod radio {
    // RF control registers
    const REG_RF_MODE_CTRL: *mut u8 = 0x800F00 as *mut u8;   // auto mode control
    const REG_RF_SN: *mut u8 = 0x800F01 as *mut u8;          // SN/NESN reset
    const REG_RF_LL_CTRL_0: *mut u8 = 0x800F02 as *mut u8;   // TX/RX enable
    const REG_RF_LL_CTRL_1: *mut u8 = 0x800F03 as *mut u8;   // timestamp / misc
    const REG_RF_TX_SETTLE: *mut u16 = 0x800F04 as *mut u16;  // TX settle time
    const REG_RF_LL_CTRL_3: *mut u8 = 0x800F16 as *mut u8;   // TRX off/scheduled
    const REG_RF_IRQ_MASK: *mut u16 = 0x800F1C as *mut u16;   // RF IRQ mask
    const REG_RF_IRQ_STATUS: *mut u16 = 0x800F20 as *mut u16; // RF IRQ status
    const REG_RF_LL_CTRL_2: *mut u8 = 0x800F15 as *mut u8;   // TX pipe

    // RF analog registers
    const REG_RF_RX_MODE: *mut u8 = 0x800428 as *mut u8;     // RX mode enable
    const REG_RF_CHANNEL: *mut u8 = 0x80040D as *mut u8;     // physical channel
    const REG_RF_RSSI: *const u8 = 0x800441 as *const u8;    // RSSI readback
    const REG_PLL_FINE_TUNE: *mut u16 = 0x8004D6 as *mut u16; // PLL fine divider

    // Modem channel registers (from SDK rf_set_channel disassembly)
    const REG_MODEM_CHN_L: *mut u8 = 0x801244 as *mut u8;    // channel set low
    const REG_MODEM_CHN_H: *mut u8 = 0x801245 as *mut u8;    // channel set high
    const REG_MODEM_BAND: *mut u8 = 0x801229 as *mut u8;     // power band

    // DMA registers
    const REG_DMA2_ADDR: *mut u16 = 0x800C08 as *mut u16;    // RX DMA addr low
    const REG_DMA2_ADDR_HI: *mut u8 = 0x800C42 as *mut u8;   // RX DMA addr high
    const REG_DMA_CHN_EN: *mut u8 = 0x800C20 as *mut u8;     // DMA channel enable
    const REG_DMA_CHN_IRQ_MSK: *mut u8 = 0x800C21 as *mut u8; // DMA IRQ mask

    // Reset register
    const REG_RST1: *mut u8 = 0x800061 as *mut u8;

    // Constants from SDK rf_drv.h
    const RF_TRX_MODE: u8 = 0xE0;
    const RF_TRX_OFF: u8 = 0x45;
    const DEFAULT_TX_SETTLE_US: u16 = 150;

    /// Reset the ZB baseband
    #[inline(always)]
    pub fn reset_baseband() {
        unsafe {
            core::ptr::write_volatile(REG_RST1, 0x01); // FLD_RST1_ZB
            core::ptr::write_volatile(REG_RST1, 0x00);
        }
    }

    /// Set TRX off + auto mode (from SDK rf_set_tx_rx_off + rf_set_tx_rx_off_auto_mode)
    #[inline(always)]
    pub fn set_trx_off() {
        unsafe {
            core::ptr::write_volatile(REG_RF_LL_CTRL_3, 0x29);
            core::ptr::write_volatile(REG_RF_RX_MODE, RF_TRX_MODE); // rx disable
            core::ptr::write_volatile(REG_RF_LL_CTRL_0, RF_TRX_OFF); // reset state machine
        }
    }

    /// Set auto mode
    #[inline(always)]
    pub fn set_auto_mode() {
        unsafe {
            core::ptr::write_volatile(REG_RF_MODE_CTRL, 0x80);
        }
    }

    /// Reset SN/NESN
    #[inline(always)]
    pub fn reset_sn() {
        unsafe {
            core::ptr::write_volatile(REG_RF_SN, 0x3F);
            core::ptr::write_volatile(REG_RF_SN, 0x00);
        }
    }

    /// Clear all RF IRQ status
    #[inline(always)]
    pub fn clear_irq_status() {
        unsafe {
            core::ptr::write_volatile(REG_RF_IRQ_STATUS, 0xFFFF);
        }
    }

    /// Clear RF IRQ mask
    #[inline(always)]
    pub fn clear_irq_mask() {
        unsafe {
            core::ptr::write_volatile(REG_RF_IRQ_MASK, 0);
        }
    }

    /// Set RF IRQ mask for TX and RX done
    #[inline(always)]
    pub fn set_irq_mask_tx_rx() {
        unsafe {
            let v = core::ptr::read_volatile(REG_RF_IRQ_MASK as *const u16);
            core::ptr::write_volatile(REG_RF_IRQ_MASK, v | 0x03); // bit0=TX, bit1=RX
        }
    }

    /// Set TX pipe
    #[inline(always)]
    pub fn set_tx_pipe(pipe: u8) {
        unsafe {
            core::ptr::write_volatile(REG_RF_LL_CTRL_2, 0xF0 | (pipe & 0x0F));
        }
    }

    /// Set TX settle time
    #[inline(always)]
    pub fn set_tx_settle(us: u16) {
        unsafe {
            core::ptr::write_volatile(REG_RF_TX_SETTLE, us.saturating_sub(1) & 0x0FFF);
        }
    }

    /// Set Zigbee channel (11-26) — full implementation from SDK disassembly
    ///
    /// Writes to THREE register sets:
    /// 1. REG_RF_CHANNEL (0x040D) — physical channel number
    /// 2. REG_PLL_FINE_TUNE (0x04D6) — PLL frequency in MHz
    /// 3. REG_MODEM_CHN_L/H (0x1244/1245) — modem channel (freq*4+1 encoded)
    /// 4. REG_MODEM_BAND (0x1229) — frequency-dependent power band
    pub fn set_channel(channel: u8) {
        if !(11..=26).contains(&channel) { return; }

        let physical = ((channel as u16) - 10) * 5; // SDK: LOGICCHANNEL_TO_PHYSICAL
        let freq_mhz: u16 = 2400 + physical;

        // Compute frequency band (from SDK rf_set_channel disassembly)
        let band: u8 = if freq_mhz > 2464 {
            0x0C // channels 23-26
        } else if freq_mhz > 2434 {
            0x10 // channels 17-22
        } else {
            0x14 // channels 11-16
        };

        // Turn off TRX before changing channel (from SDK apply_channel_frequency)
        set_trx_off();

        unsafe {
            // 1. Physical channel register (HAL approach)
            core::ptr::write_volatile(REG_RF_CHANNEL, physical as u8);

            // 2. PLL fine tune register (HAL approach, 826x compatible)
            core::ptr::write_volatile(REG_PLL_FINE_TUNE, freq_mhz);

            // 3. Modem channel registers (SDK approach, from disassembly)
            // Encoding: modem_value = freq_mhz * 4 + 1 = (freq << 2) | 1
            let modem_val: u16 = (freq_mhz << 2) | 1;
            core::ptr::write_volatile(REG_MODEM_CHN_L, modem_val as u8);
            let existing_h = core::ptr::read_volatile(REG_MODEM_CHN_H as *const u8);
            core::ptr::write_volatile(REG_MODEM_CHN_H,
                (existing_h & 0xC0) | ((modem_val >> 8) as u8 & 0x3F));

            // 4. Power band register (from disassembly, bits [5:2])
            let existing_band = core::ptr::read_volatile(REG_MODEM_BAND as *const u8);
            core::ptr::write_volatile(REG_MODEM_BAND,
                (existing_band & 0xC3) | (band & 0x3C));
        }

        // The SDK demo keeps a guard time between channel programming and TX.
        // A short settle delay here avoids arming TX while the PLL is still moving.
        super::spin_delay(2_000);
    }

    /// Enable RX mode (from SDK rf_set_rxmode)
    #[inline(always)]
    pub fn set_rx_mode() {
        unsafe {
            core::ptr::write_volatile(REG_RF_RX_MODE, RF_TRX_MODE | 0x01); // rx enable
            core::ptr::write_volatile(REG_RF_LL_CTRL_0, RF_TRX_OFF | (1 << 5)); // RX enable
        }
    }

    /// Read RSSI in dBm for 802.15.4 (from SDK)
    #[inline(always)]
    pub fn rssi_dbm() -> i8 {
        unsafe { core::ptr::read_volatile(REG_RF_RSSI) as i8 - 110 }
    }

    // DMA size/mode registers
    const REG_DMA2_SIZE: *mut u8 = 0x800C0A as *mut u8;      // RX DMA size (in 16-byte units)
    const REG_DMA2_MODE: *mut u8 = 0x800C0B as *mut u8;      // RX DMA mode (1=single, 3=pingpong)

    // SRX registers (from rf_start_srx disassembly)
    #[allow(dead_code)]
    const REG_RF_SRX_TICK: *mut u32 = 0x800F18 as *mut u32;  // SRX start tick
    #[allow(dead_code)]
    const REG_RF_SRX_TIMEOUT: *mut u32 = 0x800F28 as *mut u32; // SRX timeout

    /// Set RX DMA buffer address
    #[inline(always)]
    pub fn set_rx_buffer(addr: *mut u8) {
        let a = addr as usize;
        unsafe {
            core::ptr::write_volatile(REG_DMA2_ADDR, a as u16);
            core::ptr::write_volatile(REG_DMA2_ADDR_HI, ((a >> 16) as u8) & 0x0F);
        }
    }

    /// Configure RX DMA buffer size and mode
    /// buf_size: total buffer size in bytes (must be multiple of 16)
    pub fn set_rx_dma_config(buf_size: u16) {
        unsafe {
            // Size in 16-byte units (from rf_rx_cfg disassembly: (size >> 4) & 0xFF)
            core::ptr::write_volatile(REG_DMA2_SIZE, ((buf_size >> 4) & 0xFF) as u8);
            // Mode: 1 = single buffer (no pingpong)
            core::ptr::write_volatile(REG_DMA2_MODE, 0x01);
        }
    }

    /// Enable DMA RX channel (bit 2)
    #[inline(always)]
    pub fn enable_dma_rx() {
        unsafe {
            let v = core::ptr::read_volatile(REG_DMA_CHN_EN as *const u8);
            core::ptr::write_volatile(REG_DMA_CHN_EN, v | 0x04); // bit 2 = RF RX
        }
    }

    /// Disable DMA RX channel (bit 2)
    #[inline(always)]
    #[allow(dead_code)]
    pub fn disable_dma_rx() {
        unsafe {
            let v = core::ptr::read_volatile(REG_DMA_CHN_EN as *const u8);
            core::ptr::write_volatile(REG_DMA_CHN_EN, v & !0x04);
        }
    }

    /// Enable DMA TX channel (bit 3)
    #[inline(always)]
    pub fn enable_dma_tx() {
        unsafe {
            let v = core::ptr::read_volatile(REG_DMA_CHN_EN as *const u8);
            core::ptr::write_volatile(REG_DMA_CHN_EN, v | 0x08); // bit 3 = TX
        }
    }

    /// Check if RX is done (packet received)
    /// SDK: rf_rx_finish() → (read_reg8(0xF20) & BIT(0)) == 0x01
    #[inline(always)]
    pub fn rx_done() -> bool {
        unsafe { (core::ptr::read_volatile(REG_RF_IRQ_STATUS as *const u16) & 0x01) != 0 }
    }

    /// Clear RX done flag
    /// SDK: rf_rx_finish_clear_flag() → write_reg8(0xF20, 0x01)
    #[inline(always)]
    pub fn rx_done_clear() {
        unsafe {
            // Write 1 to bit 0 to clear (W1C)
            core::ptr::write_volatile(0x800F20 as *mut u8, 0x01);
        }
    }

    /// Clear TX done flag
    #[inline(always)]
    pub fn tx_done_clear() {
        unsafe {
            core::ptr::write_volatile(0x800F20 as *mut u8, 0x02);
        }
    }

    /// Check if TX is done (packet sent)
    /// SDK: (read_reg8(0xF20) & BIT(1)) == 0x02
    #[inline(always)]
    pub fn tx_done() -> bool {
        unsafe { (core::ptr::read_volatile(REG_RF_IRQ_STATUS as *const u16) & 0x02) != 0 }
    }

    // DMA3 (TX) registers
    const REG_DMA3_ADDR: *mut u16 = 0x800C0C as *mut u16;
    const REG_DMA3_ADDR_HI: *mut u8 = 0x800C43 as *mut u8;
    const REG_DMA3_SIZE: *mut u8 = 0x800C0E as *mut u8;
    const REG_DMA3_MODE: *mut u8 = 0x800C0F as *mut u8;

    /// Set TX DMA buffer address and trigger DMA
    /// Based on disassembly of SDK rf_tx_pkt():
    /// 1. Write 0x04 to 0x800C43 (DMA3 addr high, RAM region)
    /// 2. Write buf addr low 16 bits to 0x800C0C (DMA3 addr low)
    /// 3. Set bit 3 of 0x800C24 (DMA TX ready) to trigger transfer
    #[inline(always)]
    pub fn tx_pkt(addr: *const u8) {
        let a = addr as usize;
        unsafe {
            core::ptr::write_volatile(REG_DMA3_ADDR_HI, 0x04);
            core::ptr::write_volatile(REG_DMA3_ADDR, (a & 0xFFFF) as u16);
            let v = core::ptr::read_volatile(REG_DMA_TX_RDY as *const u8);
            core::ptr::write_volatile(REG_DMA_TX_RDY, v | 0x08);
        }
    }

    // DMA TX ready register
    const REG_DMA_TX_RDY: *mut u8 = 0x800C24 as *mut u8;

    /// Configure TX DMA buffer size and mode
    pub fn set_tx_dma_config(buf_size: u16) {
        unsafe {
            core::ptr::write_volatile(REG_DMA3_SIZE, ((buf_size >> 4) & 0xFF) as u8);
            // TX is memory -> RF, so the DMA direction must stay in the default
            // read-from-memory mode. `0x01` is the RX/write-to-memory direction.
            core::ptr::write_volatile(REG_DMA3_MODE, 0x00);
        }
    }

    /// Enable TX mode (from SDK rf_set_txmode)
    #[inline(always)]
    pub fn set_tx_mode() {
        unsafe {
            core::ptr::write_volatile(REG_RF_LL_CTRL_0, RF_TRX_OFF | (1 << 4));
        }
    }

    /// Validate Zigbee packet length using the B85/TLSR825x SDK macro:
    /// RF_ZIGBEE_PACKET_LENGTH_OK(p) -> p[0] == p[4] + 9
    #[inline(always)]
    pub fn packet_length_ok(buf: *const u8) -> bool {
        unsafe {
            let total_len = core::ptr::read_volatile(buf) as u16;
            let payload_len = core::ptr::read_volatile(buf.add(4)) as u16;
            total_len == payload_len + 9
        }
    }

    /// Validate Zigbee packet CRC using the B85/TLSR825x SDK macro:
    /// RF_ZIGBEE_PACKET_CRC_OK(p) -> (p[p[0]+3] & 0x51) == 0x10
    #[inline(always)]
    pub fn packet_crc_ok(buf: *const u8) -> bool {
        unsafe {
            let total_len = core::ptr::read_volatile(buf) as usize;
            if total_len == 0 || total_len > 130 {
                return false;
            }
            let status_byte = core::ptr::read_volatile(buf.add(total_len + 3));
            (status_byte & 0x51) == 0x10
        }
    }

    /// Get packet RSSI using the SDK offset convention: p[p[0]+2] - 110.
    #[inline(always)]
    pub fn packet_rssi(buf: *const u8) -> i8 {
        unsafe {
            let total_len = core::ptr::read_volatile(buf) as usize;
            if total_len == 0 || total_len > 130 {
                return -110;
            }
            (core::ptr::read_volatile(buf.add(total_len + 2)) as i8).wrapping_sub(110)
        }
    }

    /// Get payload length from byte 4, per the B85/TLSR825x SDK layout.
    #[inline(always)]
    pub fn payload_len(buf: *const u8) -> u8 {
        unsafe { core::ptr::read_volatile(buf.add(4)) }
    }

    /// Clear RX buffer for next packet: p[0]=0, p[4]=0
    #[inline(always)]
    pub fn rx_buf_clear(buf: *mut u8) {
        unsafe {
            core::ptr::write_volatile(buf, 0);
            core::ptr::write_volatile(buf.add(4), 0);
        }
    }

    /// Full radio init: reset, PHY config, DMA, IRQs
    pub fn init() {
        // Step 1: Reset and configure state machine
        reset_baseband();
        set_auto_mode();
        set_trx_off();
        reset_sn();
        clear_irq_status();
        clear_irq_mask();
        set_tx_pipe(0);
        set_tx_settle(DEFAULT_TX_SETTLE_US);

        // Step 2: Write PHY registers (must be AFTER reset_baseband)
        super::rf_phy_init_zigbee();

        // Step 3: Set initial channel
        set_channel(11);

        // Step 4: Configure DMA (size config moved to async_main for codegen stability)
        let rx_ptr = core::ptr::addr_of_mut!(super::RF_RX_BUF) as *mut u8;
        set_rx_buffer(rx_ptr);
        rx_buf_clear(rx_ptr);
        enable_dma_rx();
        enable_dma_tx();
        unsafe {
            let dma_irq = core::ptr::read_volatile(REG_DMA_CHN_IRQ_MSK as *const u8);
            core::ptr::write_volatile(REG_DMA_CHN_IRQ_MSK, dma_irq | 0x0C);
            let ctrl1 = core::ptr::read_volatile(REG_RF_LL_CTRL_1 as *const u8);
            core::ptr::write_volatile(REG_RF_LL_CTRL_1, ctrl1 | (1 << 5));
        }

        // Step 5: Enable RF IRQs
        set_irq_mask_tx_rx();
    }
}

/// Direct analog register write with timeout
#[inline(always)]
fn analog_write(addr: u8, value: u8) {
    let base = (REG_BASE + 0x0B8) as *mut u8;
    unsafe {
        core::ptr::write_volatile(base.add(0), addr);
        core::ptr::write_volatile(base.add(1), value);
        core::ptr::write_volatile(base.add(2), 0x60);
        // Busy-wait with timeout
        for _ in 0..100_000u32 {
            if (core::ptr::read_volatile(base.add(2) as *const u8) & 1) == 0 {
                core::ptr::write_volatile(base.add(2), 0);
                return;
            }
            core::arch::asm!("nop");
        }
        // Timeout — just clear and continue
        core::ptr::write_volatile(base.add(2), 0);
    }
}

/// Direct analog register read with timeout
#[inline(always)]
#[allow(dead_code)]
fn analog_read(addr: u8) -> u8 {
    let base = (REG_BASE + 0x0B8) as *mut u8;
    unsafe {
        core::ptr::write_volatile(base.add(0), addr);
        core::ptr::write_volatile(base.add(2), 0x40);
        for _ in 0..100_000u32 {
            if (core::ptr::read_volatile(base.add(2) as *const u8) & 1) == 0 {
                return core::ptr::read_volatile(base.add(1) as *const u8);
            }
            core::arch::asm!("nop");
        }
        0xFF // timeout
    }
}

/// Set RGB LED state directly
#[inline(always)]
fn set_led(pc_out: *mut u8, pb_out: *mut u8, r: bool, g: bool, b: bool) {
    let mut pc = 0u8;
    if r { pc |= 0x02; }   // RED = PC1
    if b { pc |= 0x10; }   // BLUE = PC4
    unsafe {
        core::ptr::write_volatile(pc_out, pc);
        let pb = core::ptr::read_volatile(pb_out as *const u8);
        core::ptr::write_volatile(pb_out, if g { pb | 0x20 } else { pb & !0x20 }); // GREEN = PB5
    }
}

// ── TLSR8258 register base ─────────────────────────────────────

const REG_BASE: u32 = 0x800000;

// ── Multi-port GPIO driver ─────────────────────────────────────

mod gpio {
    //! Bare-metal GPIO for TLSR8258 — supports all ports (PA-PD).
    //!
    //! Register layout per port (8 bytes each):
    //!   +0: IN   (read input)
    //!   +1: IE   (input enable)
    //!   +2: OEN  (output enable, active-LOW: 0=output)
    //!   +3: OUT  (output data)
    //!   +4: POL  (polarity)
    //!   +5: DS   (drive strength)
    //!   +6: GPIO (GPIO function enable)
    //!   +7: IRQ  (interrupt)

    const REG_BASE: u32 = 0x800000;

    #[derive(Clone, Copy)]
    pub struct Pin {
        pub port_base: u32,
        pub bit: u8,
    }

    // Port base addresses
    #[allow(dead_code)]
    pub const PA: u32 = REG_BASE + 0x580;
    pub const PB: u32 = REG_BASE + 0x588;
    pub const PC: u32 = REG_BASE + 0x590;
    #[allow(dead_code)]
    pub const PD: u32 = REG_BASE + 0x598;

    impl Pin {
        pub const fn new(port_base: u32, bit: u8) -> Self {
            Self { port_base, bit }
        }

        fn mask(self) -> u8 {
            1u8 << self.bit
        }

        /// Configure pin as GPIO output
        pub fn set_output(self) {
            let mask = self.mask();
            unsafe {
                // Enable GPIO function
                let r = (self.port_base + 6) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) | mask);
                // OEN active-low: clear bit = output enabled
                let r = (self.port_base + 2) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) & !mask);
                // Disable input
                let r = (self.port_base + 1) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) & !mask);
            }
        }

        /// Configure pin as GPIO input with input-enable
        #[allow(dead_code)]
        pub fn set_input(self) {
            let mask = self.mask();
            unsafe {
                let r = (self.port_base + 6) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) | mask);
                let r = (self.port_base + 1) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) | mask);
                // OEN active-low: set bit = output disabled
                let r = (self.port_base + 2) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) | mask);
            }
        }

        /// Write pin HIGH or LOW
        pub fn write(self, high: bool) {
            let mask = self.mask();
            unsafe {
                let r = (self.port_base + 3) as *mut u8;
                let v = core::ptr::read_volatile(r);
                core::ptr::write_volatile(r, if high { v | mask } else { v & !mask });
            }
        }

        /// Read input level
        #[allow(dead_code)]
        pub fn read(self) -> bool {
            unsafe {
                let r = (self.port_base + 0) as *const u8;
                (core::ptr::read_volatile(r) >> self.bit) & 1 == 1
            }
        }
    }
}

// ── TB-04-Kit board definition ─────────────────────────────────

mod board {
    //! TB-04-Kit pinout (from official schematic)
    use super::gpio::{self, Pin};

    pub const LED_RED:   Pin = Pin::new(gpio::PC, 1);  // RGB Red
    pub const LED_GREEN: Pin = Pin::new(gpio::PB, 5);  // RGB Green
    pub const LED_BLUE:  Pin = Pin::new(gpio::PC, 4);  // RGB Blue
    #[allow(dead_code)]
    pub const LED_WHITE: Pin = Pin::new(gpio::PD, 4);  // Cool White
    #[allow(dead_code)]
    pub const LED_WARM:  Pin = Pin::new(gpio::PA, 0);  // Warm Yellow
}

// ── Blocking delay (using HAL timer) ───────────────────────────

fn delay_ms(ms: u32) {
    // Timer0 is explicitly started in `chip_init()` and is the only delay source
    // that we have already verified on this board.
    let start = unsafe { core::ptr::read_volatile((REG_BASE + 0x630) as *const u32) };
    let ticks = ms * 24_000;
    loop {
        let now = unsafe { core::ptr::read_volatile((REG_BASE + 0x630) as *const u32) };
        if now.wrapping_sub(start) >= ticks { break; }
    }
}

// ── Polling MAC driver ─────────────────────────────────────────

use zigbee_mac::primitives::*;

const MAX_MAC_FRAME_LEN: usize = 127;
const MAX_MAC_OVERHEAD: usize = 25;
const ACK_WAIT_LOOPS: u32 = 36_000;
const POLL_RESPONSE_LOOPS: u32 = 1_200_000;
const RX_INDICATION_LOOPS: u32 = 12_000_000;

struct RxPacket {
    data: [u8; MAX_MAC_FRAME_LEN],
    len: usize,
    #[allow(dead_code)]
    rssi: i8,
    lqi: u8,
}

/// TLSR8258 MAC backend for first-stage bring-up.
///
/// This intentionally uses polling timeouts instead of Embassy signals. The
/// goal is to stabilize scan/associate/poll on tc32 before adding a real async
/// executor or low-power sleep.
pub struct Tlsr8258Mac {
    rx_buf: *mut u8,
    short_address: zigbee_types::ShortAddress,
    pan_id: zigbee_types::PanId,
    channel: u8,
    extended_address: zigbee_types::IeeeAddress,
    coord_short_address: zigbee_types::ShortAddress,
    coord_extended_address: zigbee_types::IeeeAddress,
    rx_on_when_idle: bool,
    association_permit: bool,
    auto_request: bool,
    dsn: u8,
    bsn: u8,
    max_csma_backoffs: u8,
    min_be: u8,
    max_be: u8,
    max_frame_retries: u8,
    promiscuous: bool,
    tx_power: i8,
    response_wait_time: u8,
    beacon_payload: zigbee_mac::pib::PibPayload,
}

impl Tlsr8258Mac {
    pub fn new() -> Self {
        let rx_buf = core::ptr::addr_of_mut!(RF_RX_BUF) as *mut u8;
        radio::set_rx_dma_config(144);
        radio::set_rx_buffer(rx_buf);
        radio::set_channel(15);

        Self {
            rx_buf,
            short_address: zigbee_types::ShortAddress::BROADCAST,
            pan_id: zigbee_types::PanId::BROADCAST,
            channel: 15,
            extended_address: OUR_EXT_ADDR,
            coord_short_address: zigbee_types::ShortAddress::COORDINATOR,
            coord_extended_address: [0u8; 8],
            rx_on_when_idle: false,
            association_permit: false,
            auto_request: true,
            dsn: 0,
            bsn: 0,
            max_csma_backoffs: 4,
            min_be: 3,
            max_be: 5,
            max_frame_retries: 3,
            promiscuous: false,
            tx_power: 0,
            response_wait_time: 32,
            beacon_payload: zigbee_mac::pib::PibPayload::new(),
        }
    }

    fn next_dsn(&mut self) -> u8 {
        let seq = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
        seq
    }

    fn sync_radio_config(&self) {
        radio::set_channel(self.channel);
        radio::set_rx_buffer(self.rx_buf);
    }

    fn scan_duration_loops(scan_duration: u8) -> u32 {
        let exp = core::cmp::min(scan_duration, 10);
        let ms = ((1u32 << exp) + 1).saturating_mul(15);
        ms.saturating_mul(24_000)
    }

    fn transmit_raw(&self, frame: &[u8]) -> Result<(), MacError> {
        if frame.is_empty() || frame.len() > MAX_MAC_FRAME_LEN {
            return Err(MacError::FrameTooLong);
        }

        write_tx_dma_frame(frame);
        if tx_and_wait() {
            Ok(())
        } else {
            Err(MacError::RadioError)
        }
    }

    fn receive_raw(&self, timeout_loops: u32) -> Result<RxPacket, MacError> {
        receive_packet(self.rx_buf, timeout_loops).ok_or(MacError::NoData)
    }

    fn send_ack(&self, seq: u8, frame_pending: bool) {
        let fc_low = if frame_pending { 0x12u8 } else { 0x02u8 };
        let ack = [fc_low, 0x00, seq];
        let _ = self.transmit_raw(&ack);
    }

    fn active_scan_channel(
        &mut self,
        channel: u8,
        timeout_loops: u32,
    ) -> heapless::Vec<PanDescriptor, 8> {
        let mut found = heapless::Vec::new();
        self.channel = channel;
        self.sync_radio_config();

        let beacon_req = build_beacon_request(self.next_dsn());
        mark32(DBG_MODE_BASE + 0x2C, 0x5CA10000 | channel as u32);
        match self.transmit_raw(&beacon_req) {
            Ok(()) => mark32(DBG_MODE_BASE + 0x30, 0x5CA10001),
            Err(_) => {
                mark32(DBG_MODE_BASE + 0x30, 0x5CA1FFFF);
                return found;
            }
        }

        let deadline = timeout_loops;
        let mut waited = 0u32;
        while waited < deadline {
            let slice = core::cmp::min(240_000, deadline - waited);
            waited = waited.saturating_add(slice);
            if let Ok(pkt) = self.receive_raw(slice) {
                if let Some(desc) = parse_beacon(&pkt.data[..pkt.len], pkt.lqi, channel) {
                    let _ = found.push(desc);
                }
            }
        }
        found
    }

    fn passive_scan_channel(
        &mut self,
        channel: u8,
        timeout_loops: u32,
    ) -> heapless::Vec<PanDescriptor, 8> {
        let mut found = heapless::Vec::new();
        self.channel = channel;
        self.sync_radio_config();

        let mut waited = 0u32;
        while waited < timeout_loops {
            let slice = core::cmp::min(240_000, timeout_loops - waited);
            waited = waited.saturating_add(slice);
            if let Ok(pkt) = self.receive_raw(slice) {
                if let Some(desc) = parse_beacon(&pkt.data[..pkt.len], pkt.lqi, channel) {
                    let _ = found.push(desc);
                }
            }
        }
        found
    }

    fn build_assoc_request(
        &mut self,
        coord: &zigbee_types::MacAddress,
        capability_info: &CapabilityInfo,
    ) -> heapless::Vec<u8, 32> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();
        let dst_mode: u8 = match coord {
            zigbee_types::MacAddress::Short(_, _) => 0b10,
            zigbee_types::MacAddress::Extended(_, _) => 0b11,
        };
        let fc_lo = 0b0110_0011u8;
        let fc_hi = (0b11 << 6) | (0b01 << 4) | (dst_mode << 2);
        let _ = frame.extend_from_slice(&[fc_lo, fc_hi, seq]);
        let _ = frame.extend_from_slice(&coord.pan_id().0.to_le_bytes());
        match coord {
            zigbee_types::MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            zigbee_types::MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }
        let _ = frame.extend_from_slice(&self.extended_address);
        let _ = frame.push(0x01);
        let _ = frame.push(capability_info.to_byte());
        frame
    }

    fn build_data_request(&mut self, coord: &zigbee_types::MacAddress) -> heapless::Vec<u8, 24> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();
        let dst_mode: u8 = match coord {
            zigbee_types::MacAddress::Short(_, _) => 0b10,
            zigbee_types::MacAddress::Extended(_, _) => 0b11,
        };
        let fc_lo = 0b0110_0011u8;
        let fc_hi = (0b11 << 6) | (0b01 << 4) | (dst_mode << 2);
        let _ = frame.extend_from_slice(&[fc_lo, fc_hi, seq]);
        let _ = frame.extend_from_slice(&coord.pan_id().0.to_le_bytes());
        match coord {
            zigbee_types::MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            zigbee_types::MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }
        let _ = frame.extend_from_slice(&self.extended_address);
        let _ = frame.push(0x04);
        frame
    }

    fn build_data_frame(
        &mut self,
        dst_address: &zigbee_types::MacAddress,
        payload: &[u8],
        ack_request: bool,
    ) -> Result<heapless::Vec<u8, 127>, MacError> {
        let mut frame = heapless::Vec::new();
        let mut fc: u16 = 0x0001;
        if ack_request {
            fc |= 0x0020;
        }
        fc |= 0x0040;
        match dst_address {
            zigbee_types::MacAddress::Short(_, _) => fc |= 0x0800,
            zigbee_types::MacAddress::Extended(_, _) => fc |= 0x0C00,
        }
        if self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE {
            fc |= 0x8000;
        } else {
            fc |= 0xC000;
        }

        let _ = frame.extend_from_slice(&fc.to_le_bytes());
        let _ = frame.push(self.next_dsn());
        let _ = frame.extend_from_slice(&dst_address.pan_id().0.to_le_bytes());
        match dst_address {
            zigbee_types::MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            zigbee_types::MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }
        if self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE {
            let _ = frame.extend_from_slice(&self.short_address.0.to_le_bytes());
        } else {
            let _ = frame.extend_from_slice(&self.extended_address);
        }
        if frame.extend_from_slice(payload).is_err() {
            return Err(MacError::FrameTooLong);
        }
        Ok(frame)
    }

    fn csma_transmit(&mut self, frame: &[u8], ack_requested: bool) -> Result<(), MacError> {
        let max_retries = if ack_requested { self.max_frame_retries } else { 0 };
        for attempt in 0..=max_retries {
            self.transmit_raw(frame)?;
            if !ack_requested {
                return Ok(());
            }
            if let Ok(pkt) = self.receive_raw(ACK_WAIT_LOOPS) {
                if is_ack_for(&pkt.data[..pkt.len], frame[2]) {
                    return Ok(());
                }
            }
            if attempt == max_retries {
                return Err(MacError::NoAck);
            }
        }
        Err(MacError::NoAck)
    }
}

impl MacDriver for Tlsr8258Mac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let timeout_loops = Self::scan_duration_loops(req.scan_duration);
        let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();
        let mut energy_list: EdList = heapless::Vec::new();

        for channel in req.channel_mask.iter() {
            let ch = channel.number();
            match req.scan_type {
                ScanType::Active => {
                    let found = self.active_scan_channel(ch, timeout_loops);
                    for desc in found {
                        let _ = pan_descriptors.push(desc);
                    }
                }
                ScanType::Passive => {
                    let found = self.passive_scan_channel(ch, timeout_loops);
                    for desc in found {
                        let _ = pan_descriptors.push(desc);
                    }
                }
                ScanType::Ed => {
                    self.channel = ch;
                    self.sync_radio_config();
                    let _ = energy_list.push(EdValue {
                        channel: ch,
                        energy: radio::rssi_dbm().saturating_add(110) as u8,
                    });
                }
                ScanType::Orphan => {}
            }
        }

        self.sync_radio_config();
        if matches!(req.scan_type, ScanType::Active | ScanType::Passive) && pan_descriptors.is_empty() {
            Err(MacError::NoBeacon)
        } else {
            Ok(MlmeScanConfirm {
                scan_type: req.scan_type,
                pan_descriptors,
                energy_list,
            })
        }
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        mark32(DBG_MODE_BASE + 0x50, 0xA55C0000);
        self.channel = req.channel;
        self.pan_id = req.coord_address.pan_id();
        if let zigbee_types::MacAddress::Short(_, addr) = req.coord_address {
            self.coord_short_address = addr;
        }
        if let zigbee_types::MacAddress::Extended(_, addr) = req.coord_address {
            self.coord_extended_address = addr;
        }
        self.sync_radio_config();

        let assoc = self.build_assoc_request(&req.coord_address, &req.capability_info);
        match self.csma_transmit(&assoc, true) {
            Ok(()) => mark32(DBG_MODE_BASE + 0x54, 0xA55C0001),
            Err(e) => {
                mark32(DBG_MODE_BASE + 0x54, 0xA55CFFFF);
                return Err(e);
            }
        }

        delay_ms(100);
        mark32(DBG_MODE_BASE + 0x58, 0xA55C0002);

        let data_req = self.build_data_request(&req.coord_address);
        match self.csma_transmit(&data_req, true) {
            Ok(()) => mark32(DBG_MODE_BASE + 0x5C, 0xA55C0003),
            Err(e) => {
                mark32(DBG_MODE_BASE + 0x5C, 0xA55CFFFE);
                return Err(e);
            }
        }

        for _ in 0..10 {
            if let Ok(pkt) = self.receive_raw(POLL_RESPONSE_LOOPS) {
                let data = &pkt.data[..pkt.len];
                let fc = if data.len() >= 2 {
                    u16::from_le_bytes([data[0], data[1]])
                } else {
                    0
                };
                let cmd_offset = 3 + addressing_size(fc);
                let cmd_id = data.get(cmd_offset).copied().unwrap_or(0);
                let frame_word = if data.len() >= 4 {
                    u32::from_le_bytes([data[0], data[1], data[2], data[3]])
                } else {
                    0
                };
                mark32(
                    DBG_MODE_BASE + 0x64,
                    (pkt.len as u32)
                        | ((fc as u32) << 8)
                        | ((cmd_id as u32) << 24),
                );
                mark32(DBG_MODE_BASE + 0x68, frame_word);
                if data.len() < 5 {
                    continue;
                }
                if fc & 0x07 != 3 {
                    continue;
                }
                if data.len() < cmd_offset + 4 || data[cmd_offset] != 0x02 {
                    continue;
                }
                if (fc & 0x0020) != 0 {
                    self.send_ack(data[2], false);
                }
                let short_addr = u16::from_le_bytes([data[cmd_offset + 1], data[cmd_offset + 2]]);
                let status = match data[cmd_offset + 3] {
                    0x00 => AssociationStatus::Success,
                    0x01 => AssociationStatus::PanAtCapacity,
                    _ => AssociationStatus::PanAccessDenied,
                };
                if status == AssociationStatus::Success {
                    self.short_address = zigbee_types::ShortAddress(short_addr);
                    self.sync_radio_config();
                }
                mark32(
                    DBG_MODE_BASE + 0x60,
                    0xA55C1000
                        | (status as u32 & 0xFF)
                        | ((short_addr as u32) << 8),
                );
                return Ok(MlmeAssociateConfirm {
                    short_address: zigbee_types::ShortAddress(short_addr),
                    status,
                });
            }
        }

        mark32(DBG_MODE_BASE + 0x60, 0xA55CFFFF);
        Err(MacError::NoAck)
    }

    async fn mlme_associate_response(&mut self, _rsp: MlmeAssociateResponse) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError> {
        let mut frame: heapless::Vec<u8, 32> = heapless::Vec::new();
        let seq = self.next_dsn();
        match &req.device_address {
            zigbee_types::MacAddress::Short(_, _) => {
                let _ = frame.extend_from_slice(&[0x63, 0xC8, seq]);
            }
            zigbee_types::MacAddress::Extended(_, _) => {
                let _ = frame.extend_from_slice(&[0x63, 0xCC, seq]);
            }
        }
        let _ = frame.extend_from_slice(&req.device_address.pan_id().0.to_le_bytes());
        match &req.device_address {
            zigbee_types::MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            zigbee_types::MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }
        let _ = frame.extend_from_slice(&self.extended_address);
        let _ = frame.push(0x03);
        let _ = frame.push(req.reason as u8);
        let _ = self.csma_transmit(&frame, true);
        self.short_address = zigbee_types::ShortAddress::BROADCAST;
        self.pan_id = zigbee_types::PanId::BROADCAST;
        Ok(())
    }

    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            self.short_address = zigbee_types::ShortAddress::BROADCAST;
            self.pan_id = zigbee_types::PanId::BROADCAST;
            self.channel = 15;
            self.rx_on_when_idle = false;
            self.association_permit = false;
            self.auto_request = true;
            self.dsn = 0;
            self.bsn = 0;
            self.promiscuous = false;
        }
        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.channel = req.channel;
        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_get(&self, attr: zigbee_mac::PibAttribute) -> Result<zigbee_mac::PibValue, MacError> {
        use zigbee_mac::PibAttribute::*;
        use zigbee_mac::PibValue;

        match attr {
            MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            MacPanId => Ok(PibValue::PanId(self.pan_id)),
            MacExtendedAddress => Ok(PibValue::ExtendedAddress(self.extended_address)),
            MacCoordShortAddress => Ok(PibValue::ShortAddress(self.coord_short_address)),
            MacCoordExtendedAddress => Ok(PibValue::ExtendedAddress(self.coord_extended_address)),
            MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            MacDsn => Ok(PibValue::U8(self.dsn)),
            MacBsn => Ok(PibValue::U8(self.bsn)),
            MacMaxCsmaBackoffs => Ok(PibValue::U8(self.max_csma_backoffs)),
            MacMinBe => Ok(PibValue::U8(self.min_be)),
            MacMaxBe => Ok(PibValue::U8(self.max_be)),
            MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
            MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            MacResponseWaitTime => Ok(PibValue::U8(self.response_wait_time)),
            MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PhyTransmitPower => Ok(PibValue::U8(self.tx_power as u8)),
            PhyChannelsSupported => Ok(PibValue::U32(zigbee_types::ChannelMask::ALL_2_4GHZ.0)),
            PhyCurrentPage => Ok(PibValue::U8(0)),
            _ => Err(MacError::Unsupported),
        }
    }

    async fn mlme_set(&mut self, attr: zigbee_mac::PibAttribute, val: zigbee_mac::PibValue) -> Result<(), MacError> {
        use zigbee_mac::PibAttribute::*;
        use zigbee_mac::PibValue;

        match (attr, val) {
            (MacShortAddress, PibValue::ShortAddress(v)) => self.short_address = v,
            (MacPanId, PibValue::PanId(v)) => self.pan_id = v,
            (MacExtendedAddress, PibValue::ExtendedAddress(v)) => self.extended_address = v,
            (MacCoordShortAddress, PibValue::ShortAddress(v)) => self.coord_short_address = v,
            (MacCoordExtendedAddress, PibValue::ExtendedAddress(v)) => self.coord_extended_address = v,
            (MacRxOnWhenIdle, PibValue::Bool(v)) => self.rx_on_when_idle = v,
            (MacAssociationPermit, PibValue::Bool(v)) => self.association_permit = v,
            (MacAutoRequest, PibValue::Bool(v)) => self.auto_request = v,
            (MacDsn, PibValue::U8(v)) => self.dsn = v,
            (MacBsn, PibValue::U8(v)) => self.bsn = v,
            (MacMaxCsmaBackoffs, PibValue::U8(v)) => self.max_csma_backoffs = v,
            (MacMinBe, PibValue::U8(v)) => self.min_be = v,
            (MacMaxBe, PibValue::U8(v)) => self.max_be = v,
            (MacMaxFrameRetries, PibValue::U8(v)) => self.max_frame_retries = v,
            (MacPromiscuousMode, PibValue::Bool(v)) => self.promiscuous = v,
            (MacResponseWaitTime, PibValue::U8(v)) => self.response_wait_time = v,
            (MacBeaconPayload, PibValue::Payload(v)) => self.beacon_payload = v,
            (PhyCurrentChannel, PibValue::U8(v)) => self.channel = v,
            (PhyTransmitPower, PibValue::U8(v)) => self.tx_power = v as i8,
            _ => return Err(MacError::Unsupported),
        }
        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        let parent = zigbee_types::MacAddress::Short(self.pan_id, self.coord_short_address);
        let data_req = self.build_data_request(&parent);
        self.csma_transmit(&data_req, true)?;

        match self.receive_raw(POLL_RESPONSE_LOOPS) {
            Ok(pkt) => {
                let data = &pkt.data[..pkt.len];
                if data.len() < 5 {
                    return Ok(None);
                }
                let fc = u16::from_le_bytes([data[0], data[1]]);
                if fc & 0x07 != 1 {
                    return Ok(None);
                }
                if (fc & 0x0020) != 0 {
                    self.send_ack(data[2], false);
                }
                let header_len = 3 + addressing_size(fc);
                if data.len() <= header_len {
                    return Ok(None);
                }
                Ok(MacFrame::from_slice(&data[header_len..]))
            }
            Err(MacError::NoData) => Ok(None),
            Err(e) => Err(e),
        }
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let frame = self.build_data_frame(&req.dst_address, req.payload, req.tx_options.ack_tx)?;
        self.csma_transmit(&frame, req.tx_options.ack_tx)?;
        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        loop {
            let pkt = self.receive_raw(RX_INDICATION_LOOPS)?;
            let data = &pkt.data[..pkt.len];
            if data.len() < 5 {
                continue;
            }
            let fc = u16::from_le_bytes([data[0], data[1]]);
            let frame_type = fc & 0x07;
            if frame_type != 1 && frame_type != 3 {
                continue;
            }
            let src = match parse_source_address(data, fc) {
                Some(addr) => addr,
                None => continue,
            };
            let dst = match parse_dest_address(data, fc) {
                Some(addr) => addr,
                None => continue,
            };
            if !self.promiscuous && !address_matches(&dst, self.pan_id, self.short_address, self.extended_address) {
                continue;
            }
            if (fc & 0x0020) != 0 {
                self.send_ack(data[2], false);
            }
            if frame_type != 1 {
                continue;
            }
            let header_len = 3 + addressing_size(fc);
            if data.len() <= header_len {
                continue;
            }
            if let Some(payload) = MacFrame::from_slice(&data[header_len..]) {
                return Ok(McpsDataIndication {
                    src_address: src,
                    dst_address: dst,
                    lqi: pkt.lqi,
                    payload,
                    security_use: (fc >> 3) & 1 != 0,
                });
            }
        }
    }

    fn capabilities(&self) -> zigbee_mac::MacCapabilities {
        zigbee_mac::MacCapabilities {
            coordinator: false,
            router: false,
            hardware_security: false,
            max_payload: (127 - 2 - MAX_MAC_OVERHEAD) as u16,
            tx_power_min: zigbee_types::TxPower(-20),
            tx_power_max: zigbee_types::TxPower(10),
        }
    }
}

fn write_tx_dma_frame(frame: &[u8]) {
    let tx_buf = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
    let mac_len = frame.len();
    unsafe {
        core::ptr::write_volatile(tx_buf.add(0), (mac_len + 1) as u8);
        core::ptr::write_volatile(tx_buf.add(1), 0);
        core::ptr::write_volatile(tx_buf.add(2), 0);
        core::ptr::write_volatile(tx_buf.add(3), 0);
        core::ptr::write_volatile(tx_buf.add(4), (mac_len + 2) as u8);
        core::ptr::copy_nonoverlapping(frame.as_ptr(), tx_buf.add(5), mac_len);
    }
}

fn receive_packet(rx_buf: *mut u8, timeout_loops: u32) -> Option<RxPacket> {
    radio::set_trx_off();
    radio::rx_done_clear();
    radio::rx_buf_clear(rx_buf);
    radio::set_rx_buffer(rx_buf);
    radio::enable_dma_rx();
    radio::set_rx_mode();

    for _ in 0..timeout_loops {
        if radio::rx_done() {
            radio::rx_done_clear();
            let total_len = unsafe { core::ptr::read_volatile(rx_buf) };
            let payload_len = unsafe { core::ptr::read_volatile(rx_buf.add(4)) };
            let frame_word = unsafe { core::ptr::read_volatile(rx_buf.add(5) as *const u32) };
            let status_byte = if total_len > 0 && total_len <= 130 {
                unsafe { core::ptr::read_volatile(rx_buf.add(total_len as usize + 3)) }
            } else {
                0
            };
            mark32(
                DBG_MODE_BASE + 0x34,
                (total_len as u32)
                    | ((payload_len as u32) << 8)
                    | ((status_byte as u32) << 16),
            );
            mark32(DBG_MODE_BASE + 0x38, frame_word);
            if total_len > 0 && radio::packet_length_ok(rx_buf) && radio::packet_crc_ok(rx_buf) {
                let phy_len = radio::payload_len(rx_buf) as usize;
                mark32(DBG_MODE_BASE + 0x3C, 0x600D0001);
                if (2..=MAX_MAC_FRAME_LEN + 2).contains(&phy_len) {
                    let frame_len = phy_len - 2;
                    let mut data = [0u8; MAX_MAC_FRAME_LEN];
                    unsafe {
                        core::ptr::copy_nonoverlapping(rx_buf.add(5), data.as_mut_ptr(), frame_len);
                    }
                    let rssi = radio::packet_rssi(rx_buf);
                    let lqi = rssi_to_lqi(rssi);
                    radio::set_trx_off();
                    return Some(RxPacket {
                        data,
                        len: frame_len,
                        rssi,
                        lqi,
                    });
                }
                mark32(DBG_MODE_BASE + 0x3C, 0x600D00EE);
            } else {
                mark32(DBG_MODE_BASE + 0x3C, 0x600DFFFF);
            }
            radio::rx_buf_clear(rx_buf);
            radio::set_rx_buffer(rx_buf);
            radio::enable_dma_rx();
            radio::set_rx_mode();
        }
        unsafe { core::arch::asm!("nop"); }
    }
    radio::set_trx_off();
    None
}

fn build_beacon_request(seq: u8) -> [u8; 8] {
    [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]
}

#[cfg(all(not(feature = "sensor"), feature = "diag-assoc"))]
fn build_long_tx_probe(seq: u8) -> [u8; 17] {
    [
        0x41, 0x88, seq, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12,
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    ]
}

fn is_ack_for(data: &[u8], seq: u8) -> bool {
    data.len() >= 3 && (u16::from_le_bytes([data[0], data[1]]) & 0x07) == 0x02 && data[2] == seq
}

fn rssi_to_lqi(rssi: i8) -> u8 {
    let v = (rssi as i16 + 106).clamp(0, 100);
    ((v * 255) / 100) as u8
}

fn address_matches(
    dst: &zigbee_types::MacAddress,
    pan_id: zigbee_types::PanId,
    short_address: zigbee_types::ShortAddress,
    extended_address: zigbee_types::IeeeAddress,
) -> bool {
    match dst {
        zigbee_types::MacAddress::Short(pan, addr) => {
            (pan.0 == pan_id.0 || pan.0 == 0xFFFF)
                && (addr.0 == short_address.0 || addr.0 == 0xFFFF)
        }
        zigbee_types::MacAddress::Extended(pan, addr) => {
            (pan.0 == pan_id.0 || pan.0 == 0xFFFF) && *addr == extended_address
        }
    }
}

fn parse_beacon(frame_data: &[u8], lqi: u8, channel: u8) -> Option<PanDescriptor> {
    if frame_data.len() < 5 {
        return None;
    }

    let fc = u16::from_le_bytes([frame_data[0], frame_data[1]]);
    if fc & 0x07 != 0 {
        return None;
    }

    let superframe_offset = 3 + addressing_size(fc);
    if frame_data.len() < superframe_offset + 4 {
        return None;
    }
    let sf_raw = u16::from_le_bytes([
        frame_data[superframe_offset],
        frame_data[superframe_offset + 1],
    ]);
    let superframe_spec = SuperframeSpec::from_raw(sf_raw);
    let beacon_payload_offset = superframe_offset + 4;
    if frame_data.len() < beacon_payload_offset + 15 {
        return None;
    }

    let coord_address = parse_source_address(frame_data, fc)?;
    let zigbee_beacon = parse_zigbee_beacon(&frame_data[beacon_payload_offset..]);

    Some(PanDescriptor {
        channel,
        coord_address,
        superframe_spec,
        lqi,
        security_use: (fc >> 3) & 1 != 0,
        zigbee_beacon,
    })
}

fn addressing_size(fc: u16) -> usize {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    let mut size = 0;
    match dst_mode {
        0x02 => size += 4,
        0x03 => size += 10,
        _ => {}
    }
    match src_mode {
        0x02 => size += if pan_compress { 2 } else { 4 },
        0x03 => size += if pan_compress { 8 } else { 10 },
        _ => {}
    }
    size
}

fn parse_source_address(data: &[u8], fc: u16) -> Option<zigbee_types::MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    let mut offset = 3;
    let dst_pan = if dst_mode >= 2 && data.len() > offset + 1 {
        let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        Some(pan)
    } else {
        None
    };
    match dst_mode {
        0x02 => offset += 2,
        0x03 => offset += 8,
        _ => {}
    }

    let src_pan = if !pan_compress && src_mode >= 2 && data.len() > offset + 1 {
        let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        pan
    } else {
        dst_pan.unwrap_or(0xFFFF)
    };

    match src_mode {
        0x02 if data.len() >= offset + 2 => {
            let addr = u16::from_le_bytes([data[offset], data[offset + 1]]);
            Some(zigbee_types::MacAddress::Short(
                zigbee_types::PanId(src_pan),
                zigbee_types::ShortAddress(addr),
            ))
        }
        0x03 if data.len() >= offset + 8 => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[offset..offset + 8]);
            Some(zigbee_types::MacAddress::Extended(
                zigbee_types::PanId(src_pan),
                ext,
            ))
        }
        _ => None,
    }
}

fn parse_dest_address(data: &[u8], fc: u16) -> Option<zigbee_types::MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let offset = 3;
    if dst_mode < 2 || data.len() < offset + 2 {
        return None;
    }
    let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
    let addr_offset = offset + 2;
    match dst_mode {
        0x02 if data.len() >= addr_offset + 2 => {
            let addr = u16::from_le_bytes([data[addr_offset], data[addr_offset + 1]]);
            Some(zigbee_types::MacAddress::Short(
                zigbee_types::PanId(pan),
                zigbee_types::ShortAddress(addr),
            ))
        }
        0x03 if data.len() >= addr_offset + 8 => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[addr_offset..addr_offset + 8]);
            Some(zigbee_types::MacAddress::Extended(zigbee_types::PanId(pan), ext))
        }
        _ => None,
    }
}

fn parse_zigbee_beacon(data: &[u8]) -> ZigbeeBeaconPayload {
    let protocol_id = data[0];
    let nwk_info = u16::from_le_bytes([data[1], data[2]]);
    let mut extended_pan_id = [0u8; 8];
    extended_pan_id.copy_from_slice(&data[3..11]);
    let mut tx_offset = [0u8; 3];
    tx_offset.copy_from_slice(&data[11..14]);

    ZigbeeBeaconPayload {
        protocol_id,
        stack_profile: (nwk_info & 0x0F) as u8,
        protocol_version: ((nwk_info >> 4) & 0x0F) as u8,
        router_capacity: (nwk_info >> 10) & 1 != 0,
        device_depth: ((nwk_info >> 11) & 0x0F) as u8,
        end_device_capacity: (nwk_info >> 15) & 1 != 0,
        extended_pan_id,
        tx_offset,
        update_id: data[14],
    }
}

// ── Critical section (single-core, interrupt disable) ──────────

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _critical_section_1_0_acquire() -> u8 {
    // tc32 doesn't have mrs/cpsid — use TLSR8258 IRQ enable register
    unsafe {
        let irq_en = (REG_BASE + 0x643) as *mut u8;
        let prev = core::ptr::read_volatile(irq_en);
        core::ptr::write_volatile(irq_en, 0);
        prev
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _critical_section_1_0_release(restore: u8) {
    if restore != 0 {
        unsafe {
            let irq_en = (REG_BASE + 0x643) as *mut u8;
            core::ptr::write_volatile(irq_en, restore);
        }
    }
}

// ── Async timer (self-contained, no embassy deps) ──────────────
//
// System tick: 32-bit free-running at 16 ticks/µs (24MHz RC).
// System timer IRQ: fires when tick matches capture register.
// Timer future: polls once, sets alarm, yields, woken by IRQ.

mod async_timer {
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};

    // Timer0 at 0x800630, runs at system clock (24MHz RC) = 24 ticks/µs
    const REG_TIMER0_TICK: *const u32 = 0x800630 as *const u32;
    const TICKS_PER_MS: u32 = 24_000;

    pub fn on_alarm_irq() {}

    #[inline(always)]
    pub fn now_ticks() -> u32 {
        unsafe { core::ptr::read_volatile(REG_TIMER0_TICK) }
    }

    pub struct Delay {
        target_tick: u32,
    }

    impl Delay {
        pub fn new(ms: u32) -> Self {
            Self {
                target_tick: now_ticks().wrapping_add(ms.wrapping_mul(TICKS_PER_MS)),
            }
        }
    }

    impl Future for Delay {
        type Output = ();

        fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
            let remaining = self.target_tick.wrapping_sub(now_ticks());
            if remaining == 0 || remaining > 0x8000_0000 {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    pub async fn delay_ms(ms: u32) {
        Delay::new(ms).await
    }
}

// ── Minimal single-task executor ───────────────────────────────

mod executor {
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    fn noop_waker() -> Waker {
        const VTABLE: RawWakerVTable = RawWakerVTable::new(
            |p| RawWaker::new(p, &VTABLE),
            |_| {},
            |_| {},
            |_| {},
        );
        unsafe { Waker::new(core::ptr::null(), &VTABLE) }
    }

    /// Run a single future to completion, busy-polling.
    pub fn block_on<F: Future>(f: F) -> F::Output {
        let mut f = f;
        let mut f = unsafe { Pin::new_unchecked(&mut f) };
        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        loop {
            if let Poll::Ready(val) = f.as_mut().poll(&mut cx) {
                return val;
            }
            // Brief idle for SWire debug access
            for _ in 0..100u32 { unsafe { core::arch::asm!("nop"); } }
        }
    }
}

// ── Beacon request TX (non-async to keep Future small) ─────────

/// Build and transmit a beacon request on the current channel.
/// Returns true if TX completed, false on timeout.
/// Beacon request: FC=0x0803, DstPAN=0xFFFF, DstAddr=0xFFFF, Cmd=0x07
#[inline(never)]
#[cfg(all(not(feature = "sensor"), not(feature = "diag-assoc")))]
fn send_beacon_request(seq: u8) -> bool {
    let tx_buf = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
    let mac_len: u8 = 8; // FC(2) + Seq(1) + DstPAN(2) + DstAddr(2) + CmdID(1)

    unsafe {
        // DMA header: p[0]=len+1, p[1..3]=0
        core::ptr::write_volatile(tx_buf.add(0), mac_len + 1);
        core::ptr::write_volatile(tx_buf.add(1), 0);
        core::ptr::write_volatile(tx_buf.add(2), 0);
        core::ptr::write_volatile(tx_buf.add(3), 0);
        // PSDU length (HW appends 2-byte CRC)
        core::ptr::write_volatile(tx_buf.add(4), mac_len + 2);
        // Frame Control: MAC command, short dst, no src
        // bits 0-2: type=3 (cmd), bit 5: ack_req=0, bits 10-11: dst=2 (short)
        core::ptr::write_volatile(tx_buf.add(5), 0x03); // FC low
        core::ptr::write_volatile(tx_buf.add(6), 0x08); // FC high (dst_mode=2)
        // Sequence number
        core::ptr::write_volatile(tx_buf.add(7), seq);
        // Dest PAN ID = 0xFFFF (broadcast)
        core::ptr::write_volatile(tx_buf.add(8), 0xFF);
        core::ptr::write_volatile(tx_buf.add(9), 0xFF);
        // Dest Address = 0xFFFF (broadcast)
        core::ptr::write_volatile(tx_buf.add(10), 0xFF);
        core::ptr::write_volatile(tx_buf.add(11), 0xFF);
        // Command ID = 0x07 (Beacon Request)
        core::ptr::write_volatile(tx_buf.add(12), 0x07);
    }

    // Configure TX DMA
    radio::set_tx_dma_config(144);

    // Switch to TX mode, then trigger DMA
    radio::tx_done_clear();
    radio::set_tx_mode();
    spin_delay(10_000);
    radio::tx_pkt(tx_buf);

    // Busy-wait for TX done (max ~5ms at 24MHz)
    for _ in 0..120_000u32 {
        if radio::tx_done() {
            radio::tx_done_clear();
            return true;
        }
        unsafe { core::arch::asm!("nop"); }
    }
    false // timeout
}

/// Perform one beacon request + RX scan cycle.
/// Returns (pkt_received, is_beacon, frame_word).
/// Uses busy-wait polling to catch first packet before DMA overwrite.
#[inline(never)]
#[cfg(all(not(feature = "sensor"), not(feature = "diag-assoc")))]
fn scan_one(rx_buf: *mut u8, seq: u8) -> (bool, bool, u32) {
    radio::set_channel(15);
    radio::set_trx_off();
    let tx_ok = send_beacon_request(seq);

    // Switch to RX immediately after TX
    radio::set_trx_off();
    radio::rx_done_clear();
    radio::rx_buf_clear(rx_buf);
    radio::set_rx_buffer(rx_buf);
    radio::enable_dma_rx();
    radio::set_rx_mode();

    let mut last_frame: u32 = 0;
    let mut got_any = false;

    // Busy-poll for ~10ms (240k iterations at ~24MHz = ~10ms)
    for _ in 0..240_000u32 {
        if radio::rx_done() {
            radio::rx_done_clear();
            let total_len = unsafe { core::ptr::read_volatile(rx_buf) };
            if total_len > 0 && radio::packet_length_ok(rx_buf) && radio::packet_crc_ok(rx_buf) {
                got_any = true;
                let frame_word = unsafe { core::ptr::read_volatile(rx_buf.add(4) as *const u32) };
                // Read PAN+addr immediately while buffer is stable
                let pan_word = unsafe { core::ptr::read_volatile(rx_buf.add(8) as *const u32) };
                last_frame = frame_word;
                let fc_lo = (frame_word >> 8) as u8;
                if fc_lo & 0x07 == 0 {
                    // Beacon! Return PAN+coord from the snapshot we took
                    radio::set_trx_off();
                    return (true, true, pan_word);
                }
            }
            // Clear and re-enable RX for next packet
            radio::rx_buf_clear(rx_buf);
            radio::set_rx_buffer(rx_buf);
            radio::enable_dma_rx();
            radio::set_rx_mode();
        }
        unsafe { core::arch::asm!("nop"); }
    }
    radio::set_trx_off();
    if got_any {
        (true, false, last_frame) // got packets but no beacon
    } else {
        let tx_marker = if tx_ok { 0x000000AA_u32 } else { 0x000000FF_u32 };
        (false, false, tx_marker)
    }
}

// ── MAC Association ────────────────────────────────────────────

/// Our 64-bit IEEE extended address (hardcoded for testing).
/// TODO: read from flash at 0x76000 (factory-programmed)
const OUR_EXT_ADDR: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

/// Transmit a raw MAC frame and busy-wait for TX done.
/// `data` points to the TX DMA buffer already filled, starting at byte 0 (DMA header).
#[inline(never)]
fn tx_and_wait() -> bool {
    let tx_buf = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
    radio::set_trx_off();
    radio::set_tx_dma_config(144);
    radio::tx_done_clear();
    radio::set_tx_mode();
    spin_delay(10_000);
    radio::tx_pkt(tx_buf);
    for _ in 0..120_000u32 {
        if radio::tx_done() {
            radio::tx_done_clear();
            mark32(DBG_MODE_BASE + 0x40, 0x700D0001);
            return true;
        }
        unsafe { core::arch::asm!("nop"); }
    }
    mark32(DBG_MODE_BASE + 0x40, 0x700DFFFF);
    mark32(
        DBG_MODE_BASE + 0x44,
        unsafe { core::ptr::read_volatile(0x800F20 as *const u16) as u32 }
            | ((unsafe { core::ptr::read_volatile(0x800C24 as *const u8) as u32 }) << 16)
            | ((unsafe { core::ptr::read_volatile(0x800F02 as *const u8) as u32 }) << 24),
    );
    false
}

/// Send MAC Association Request (command 0x01).
///
/// Frame format (IEEE 802.15.4):
///   FC(2) + Seq(1) + DstPAN(2) + DstShort(2) + SrcExt(8) + CmdID(1) + CapInfo(1) = 17 bytes
///   FC = 0xC823: cmd frame, ack_req, intra-PAN, dst=short, src=extended
#[inline(never)]
#[cfg(not(feature = "sensor"))]
#[allow(dead_code)]
fn send_assoc_request(pan_id: u16, coord_addr: u16, seq: u8) -> bool {
    let tx_buf = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
    let mac_len: u8 = 17; // FC(2)+Seq(1)+DstPAN(2)+DstShort(2)+SrcExt(8)+CmdID(1)+Cap(1)

    unsafe {
        // DMA header
        core::ptr::write_volatile(tx_buf.add(0), mac_len + 1);
        core::ptr::write_volatile(tx_buf.add(1), 0);
        core::ptr::write_volatile(tx_buf.add(2), 0);
        core::ptr::write_volatile(tx_buf.add(3), 0);
        // PHR (payload length + 2 for FCS)
        core::ptr::write_volatile(tx_buf.add(4), mac_len + 2);
        // Frame Control: 0xC823
        // bits 0-2: type=3 (command)
        // bit 5: ack_req=1
        // bits 10-11: dst_addr_mode=2 (short)
        // bits 14-15: src_addr_mode=3 (extended)
        // bit 6: intra_pan=1
        core::ptr::write_volatile(tx_buf.add(5), 0x23); // FC low: cmd + ack_req + intra_pan
        core::ptr::write_volatile(tx_buf.add(6), 0xC8); // FC high: dst=short(2), src=ext(3)
        // Sequence number
        core::ptr::write_volatile(tx_buf.add(7), seq);
        // Dest PAN ID (LE)
        core::ptr::write_volatile(tx_buf.add(8), pan_id as u8);
        core::ptr::write_volatile(tx_buf.add(9), (pan_id >> 8) as u8);
        // Dest Short Address (LE)
        core::ptr::write_volatile(tx_buf.add(10), coord_addr as u8);
        core::ptr::write_volatile(tx_buf.add(11), (coord_addr >> 8) as u8);
        // Source Extended Address (LE, 8 bytes)
        for i in 0..8 {
            core::ptr::write_volatile(tx_buf.add(12 + i), OUR_EXT_ADDR[i]);
        }
        // Command ID = 0x01 (Association Request)
        core::ptr::write_volatile(tx_buf.add(20), 0x01);
        // Capability Information: 0x80 = allocate address
        core::ptr::write_volatile(tx_buf.add(21), 0x80);
    }

    tx_and_wait()
}

/// Send MAC Data Request (command 0x04) to poll coordinator for pending data.
///
/// Before we have a short address, we use extended source address:
///   FC(2) + Seq(1) + DstPAN(2) + DstShort(2) + SrcExt(8) + CmdID(1) = 16 bytes
///   FC = 0xC823: cmd, ack_req, intra-PAN, dst=short, src=extended
#[inline(never)]
#[cfg(not(feature = "sensor"))]
#[allow(dead_code)]
fn send_data_request(pan_id: u16, coord_addr: u16, seq: u8) -> bool {
    let tx_buf = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
    let mac_len: u8 = 16;

    unsafe {
        core::ptr::write_volatile(tx_buf.add(0), mac_len + 1);
        core::ptr::write_volatile(tx_buf.add(1), 0);
        core::ptr::write_volatile(tx_buf.add(2), 0);
        core::ptr::write_volatile(tx_buf.add(3), 0);
        core::ptr::write_volatile(tx_buf.add(4), mac_len + 2);
        core::ptr::write_volatile(tx_buf.add(5), 0x23); // FC low
        core::ptr::write_volatile(tx_buf.add(6), 0xC8); // FC high
        core::ptr::write_volatile(tx_buf.add(7), seq);
        core::ptr::write_volatile(tx_buf.add(8), pan_id as u8);
        core::ptr::write_volatile(tx_buf.add(9), (pan_id >> 8) as u8);
        core::ptr::write_volatile(tx_buf.add(10), coord_addr as u8);
        core::ptr::write_volatile(tx_buf.add(11), (coord_addr >> 8) as u8);
        for i in 0..8 {
            core::ptr::write_volatile(tx_buf.add(12 + i), OUR_EXT_ADDR[i]);
        }
        // Command ID = 0x04 (Data Request)
        core::ptr::write_volatile(tx_buf.add(20), 0x04);
    }

    tx_and_wait()
}

/// Receive one packet with timeout.
/// Returns (got_pkt, total_len). Caller reads rx_buf directly.
#[inline(never)]
#[cfg(not(feature = "sensor"))]
#[allow(dead_code)]
fn rx_with_timeout(rx_buf: *mut u8, timeout_loops: u32) -> (bool, u8) {
    radio::set_trx_off();
    radio::rx_done_clear();
    radio::rx_buf_clear(rx_buf);
    radio::set_rx_buffer(rx_buf);
    radio::enable_dma_rx();
    radio::set_rx_mode();

    for _ in 0..timeout_loops {
        if radio::rx_done() {
            radio::rx_done_clear();
            let total_len = unsafe { core::ptr::read_volatile(rx_buf) };
            if total_len > 0 && radio::packet_length_ok(rx_buf) && radio::packet_crc_ok(rx_buf) {
                radio::set_trx_off();
                return (true, total_len);
            }
            // Bad packet, keep listening
            radio::rx_buf_clear(rx_buf);
            radio::set_rx_buffer(rx_buf);
            radio::enable_dma_rx();
            radio::set_rx_mode();
        }
        unsafe { core::arch::asm!("nop"); }
    }
    radio::set_trx_off();
    (false, 0)
}

/// Try to associate with coordinator. Returns assigned short address or None.
///
/// Protocol:
/// 1. Send Association Request
/// 2. Wait for ACK (or just timeout — auto-ACK may handle it)
/// 3. Wait macResponseWaitTime then poll with Data Request
/// 4. Receive Association Response (command 0x02)
#[inline(never)]
#[cfg(not(feature = "sensor"))]
#[allow(dead_code)]
fn try_associate(rx_buf: *mut u8, pan_id: u16, coord_addr: u16, seq: &mut u8) -> Option<u16> {
    // Step 1: Send Association Request
    radio::set_channel(15);
    radio::set_trx_off();
    if !send_assoc_request(pan_id, coord_addr, *seq) {
        return None;
    }
    *seq = seq.wrapping_add(1);

    // Step 2: Wait briefly for ACK (~2ms)
    // The TLSR8258 radio may auto-ACK, but we just wait a bit
    let _ = rx_with_timeout(rx_buf, 48_000); // ~2ms

    // Step 3: Wait macResponseWaitTime before polling
    // Spec says up to aResponseWaitTime = aBaseSuperframeDuration * aResponseWaitTime symbols
    // For simplicity, wait ~500ms then poll

    // We can't call async delay here (not async fn), so busy-wait ~500ms
    for _ in 0..500u32 {
        for _ in 0..24_000u32 {
            unsafe { core::arch::asm!("nop"); }
        }
    }

    // Step 4: Poll with Data Request (try up to 4 times)
    for _attempt in 0..4u32 {
        radio::set_trx_off();
        if !send_data_request(pan_id, coord_addr, *seq) {
            *seq = seq.wrapping_add(1);
            continue;
        }
        *seq = seq.wrapping_add(1);

        // Listen for response (~50ms)
        let (got_pkt, total_len) = rx_with_timeout(rx_buf, 1_200_000);

        if got_pkt && total_len > 10 {
            // Check if this is a MAC command frame with Association Response
            let fc_lo = unsafe { core::ptr::read_volatile(rx_buf.add(5)) };
            let frame_type = fc_lo & 0x07;

            if frame_type == 3 {
                // MAC command frame — find the command ID
                // Need to parse addressing to find command ID offset
                let fc_hi = unsafe { core::ptr::read_volatile(rx_buf.add(6)) };
                let dst_mode = (fc_hi >> 2) & 0x03;
                let src_mode = (fc_hi >> 6) & 0x03;

                // Calculate command ID offset: 5 (start) + 2 (FC) + 1 (seq)
                let mut offset: usize = 8; // past FC + seq
                // Dest PAN (if dst present)
                if dst_mode > 0 { offset += 2; }
                // Dest addr
                if dst_mode == 2 { offset += 2; } // short
                else if dst_mode == 3 { offset += 8; } // extended
                // Source PAN (if not intra-PAN and src present)
                let intra_pan = (fc_lo >> 6) & 1;
                if intra_pan == 0 && src_mode > 0 { offset += 2; }
                // Source addr
                if src_mode == 2 { offset += 2; }
                else if src_mode == 3 { offset += 8; }

                if (offset as u8) < total_len + 4 {
                    let cmd_id = unsafe { core::ptr::read_volatile(rx_buf.add(offset)) };

                    if cmd_id == 0x02 {
                        // Association Response!
                        // Payload: short_addr(2 LE) + status(1)
                        let short_lo = unsafe { core::ptr::read_volatile(rx_buf.add(offset + 1)) };
                        let short_hi = unsafe { core::ptr::read_volatile(rx_buf.add(offset + 2)) };
                        let status = unsafe { core::ptr::read_volatile(rx_buf.add(offset + 3)) };

                        if status == 0x00 {
                            // Success!
                            return Some((short_hi as u16) << 8 | short_lo as u16);
                        }
                        // Association denied
                        return None;
                    }
                }
            }
        }

        // Wait ~200ms before retry
        for _ in 0..200u32 {
            for _ in 0..24_000u32 {
                unsafe { core::arch::asm!("nop"); }
            }
        }
    }
    None
}

// ── Mode entry points ──────────────────────────────────────────

#[inline(never)]
#[cfg(all(not(feature = "sensor"), not(feature = "diag-assoc")))]
async fn diag_beacon_main(rx_buf: *mut u8) {
    let mut seq: u8 = 0;
    mark32(DBG_MODE_BASE + 0x00, 0xD1A600B0);

    loop {
        mark32(DBG_MODE_BASE + 0x04, seq as u32);
        let (got_pkt, is_beacon, pan_word) = scan_one(rx_buf, seq);
        seq = seq.wrapping_add(1);

        if got_pkt && is_beacon {
            // Beacon found → green
            board::LED_GREEN.write(true);
            board::LED_RED.write(false);

            mark32(DBG_MODE_BASE + 0x10, pan_word);
            mark32(DBG_MODE_BASE + 0x14, 0xBEAC0001);
        }

        async_timer::delay_ms(10).await;
    }
}

#[inline(never)]
#[cfg(all(not(feature = "sensor"), feature = "diag-assoc"))]
async fn diag_assoc_main() {
    mark32(DBG_MODE_BASE + 0x00, 0xD1A600A0);
    let mut mac = Tlsr8258Mac::new();
    mark32(DBG_MODE_BASE + 0x04, 0xD1A600A1);
    radio::set_channel(15);
    match mac.transmit_raw(&build_long_tx_probe(0x5A)) {
        Ok(()) => mark32(DBG_MODE_BASE + 0x48, 0x10A60001),
        Err(_) => mark32(DBG_MODE_BASE + 0x48, 0x10A6FFFF),
    }

    loop {
        mark32(DBG_MODE_BASE + 0x08, 0xD1A600A2);
        let scan = MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: zigbee_types::ChannelMask::ALL_2_4GHZ,
            scan_duration: 3,
        };

        mark32(DBG_MODE_BASE + 0x0C, async_timer::now_ticks());
        match mac.mlme_scan(scan).await {
            Ok(confirm) if !confirm.pan_descriptors.is_empty() => {
                let desc = &confirm.pan_descriptors[0];
                board::LED_RED.write(false);
                board::LED_GREEN.write(true);
                mark32(DBG_MODE_BASE + 0x10, 0xBEAC0001);
                mark32(DBG_MODE_BASE + 0x14, desc.channel as u32);

                let assoc = MlmeAssociateRequest {
                    channel: desc.channel,
                    coord_address: desc.coord_address,
                    capability_info: CapabilityInfo {
                        device_type_ffd: false,
                        mains_powered: false,
                        rx_on_when_idle: false,
                        security_capable: false,
                        allocate_address: true,
                    },
                };

                match mac.mlme_associate(assoc).await {
                    Ok(confirm) if confirm.status == AssociationStatus::Success => {
                        board::LED_BLUE.write(true);
                        mark32(DBG_MODE_BASE + 0x18, 0xA550C000);
                        mark32(DBG_MODE_BASE + 0x1C, confirm.short_address.0 as u32);

                        loop {
                            match mac.mlme_poll().await {
                                Ok(Some(frame)) => {
                                    mark32(DBG_MODE_BASE + 0x20, 0x900D0001);
                                    mark32(DBG_MODE_BASE + 0x24, frame.len() as u32);
                                }
                                Ok(None) => {
                                    mark32(DBG_MODE_BASE + 0x20, 0x900D0000);
                                }
                                Err(_) => {
                                    mark32(DBG_MODE_BASE + 0x20, 0x900DFFFF);
                                }
                            }
                            async_timer::delay_ms(1000).await;
                        }
                    }
                    _ => {
                        board::LED_BLUE.write(false);
                        mark32(DBG_MODE_BASE + 0x18, 0xA550FFFF);
                    }
                }
            }
            _ => {
                board::LED_GREEN.write(false);
                board::LED_RED.write(true);
                mark32(DBG_MODE_BASE + 0x10, 0xBEACFFFF);
            }
        }

        mark32(DBG_MODE_BASE + 0x28, async_timer::now_ticks());
        async_timer::delay_ms(1000).await;
    }
}

#[cfg(feature = "sensor")]
const SENSOR_POLL_INTERVAL_MS: u32 = 1000;
#[cfg(feature = "sensor")]
const SENSOR_ANNOUNCE_PERIOD_POLLS: u8 = 50;

#[cfg(feature = "sensor")]
#[inline(never)]
async fn send_device_annce(
    mac: &mut Tlsr8258Mac,
    nwk_seq: &mut u8,
    aps_seq: &mut u8,
    zdo_seq: &mut u8,
) -> Result<(), MacError> {
    let annce = DeviceAnnounce {
        nwk_addr: mac.short_address,
        ieee_addr: mac.extended_address,
        capability: 0x80,
    };

    let mut zdp_payload = [0u8; 1 + DeviceAnnounce::WIRE_SIZE];
    zdp_payload[0] = *zdo_seq;
    *zdo_seq = zdo_seq.wrapping_add(1);
    let _ = annce.serialize(&mut zdp_payload[1..]);

    let nwk_header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            discover_route: 0,
            multicast: false,
            security: false,
            source_route: false,
            dst_ieee_present: false,
            src_ieee_present: false,
            end_device_initiator: false,
        },
        dst_addr: ShortAddress(0xFFFD),
        src_addr: mac.short_address,
        radius: 30,
        seq_number: *nwk_seq,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    *nwk_seq = nwk_seq.wrapping_add(1);

    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Broadcast as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: Some(ZDO_ENDPOINT),
        group_address: None,
        cluster_id: Some(DEVICE_ANNCE),
        profile_id: Some(PROFILE_ZDP),
        src_endpoint: Some(ZDO_ENDPOINT),
        aps_counter: *aps_seq,
        extended_header: None,
    };
    *aps_seq = aps_seq.wrapping_add(1);

    let mut payload = [0u8; 64];
    let nwk_len = nwk_header.serialize(&mut payload);
    let aps_len = aps_header.serialize(&mut payload[nwk_len..]);
    let total = nwk_len + aps_len + zdp_payload.len();
    if total > payload.len() {
        return Err(MacError::FrameTooLong);
    }
    payload[nwk_len + aps_len..total].copy_from_slice(&zdp_payload);

    mac.mcps_data(zigbee_mac::primitives::McpsDataRequest {
        src_addr_mode: zigbee_mac::primitives::AddressMode::Short,
        dst_address: MacAddress::Short(mac.pan_id, mac.coord_short_address),
        payload: &payload[..total],
        msdu_handle: *aps_seq,
        tx_options: zigbee_mac::primitives::TxOptions {
            ack_tx: false,
            indirect: false,
            security_enabled: false,
        },
    })
    .await?;

    Ok(())
}

#[cfg(feature = "sensor")]
#[inline(never)]
async fn handle_sensor_frame(
    frame: &zigbee_mac::primitives::MacFrame,
) -> bool {
    let data = frame.as_slice();
    let Some((nwk_header, nwk_len)) = NwkHeader::parse(data) else {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA01);
        return false;
    };
    let Some((aps_header, aps_len)) = ApsHeader::parse(&data[nwk_len..]) else {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA02);
        return false;
    };
    let payload = &data[nwk_len + aps_len..];

    mark32(
        DBG_MODE_BASE + 0x70,
        nwk_header.src_addr.0 as u32 | ((nwk_header.dst_addr.0 as u32) << 16),
    );
    mark32(
        DBG_MODE_BASE + 0x74,
        (aps_header.profile_id.unwrap_or(0) as u32)
            | ((aps_header.cluster_id.unwrap_or(0) as u32) << 16),
    );
    if !payload.is_empty() {
        mark32(
            DBG_MODE_BASE + 0x78,
            payload[0] as u32
                | ((aps_header.frame_control.frame_type as u32) << 8)
                | ((aps_header.frame_control.security as u32) << 16),
        );
    }
    true
}

#[inline(never)]
#[cfg(feature = "sensor")]
async fn sensor_main() {
    // Temporary sensor-lite path for tc32-stage2-tc32-31.
    // The pure-Rust runtime join path (`ZigbeeDevice::start/tick`) currently
    // trips a tc32 backend codegen bug, so default `sensor` mode uses the
    // already-validated MAC scan/associate/poll flow directly.
    let mut mac = Tlsr8258Mac::new();
    mark32(DBG_MODE_BASE + 0x00, 0x53E50000);

    loop {
        board::LED_RED.write(true);
        board::LED_GREEN.write(false);
        board::LED_BLUE.write(false);
        mark32(DBG_MODE_BASE + 0x04, 0x53E50001);

        let scan = MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: zigbee_types::ChannelMask(1 << 15),
            scan_duration: 3,
        };

        match mac.mlme_scan(scan).await {
            Ok(confirm) if !confirm.pan_descriptors.is_empty() => {
                let desc = &confirm.pan_descriptors[0];
                board::LED_RED.write(false);
                board::LED_GREEN.write(true);
                mark32(DBG_MODE_BASE + 0x10, 0x53E50010);
                mark32(DBG_MODE_BASE + 0x14, desc.channel as u32);

                let assoc = MlmeAssociateRequest {
                    channel: desc.channel,
                    coord_address: desc.coord_address,
                    capability_info: CapabilityInfo {
                        device_type_ffd: false,
                        mains_powered: false,
                        rx_on_when_idle: false,
                        security_capable: false,
                        allocate_address: true,
                    },
                };

                match mac.mlme_associate(assoc).await {
                    Ok(confirm) if confirm.status == AssociationStatus::Success => {
                        let mut nwk_seq = 0u8;
                        let mut aps_seq = 0u8;
                        let mut zdo_seq = 0u8;
                        let mut announce_polls = 0u8;
                        board::LED_BLUE.write(true);
                        mark32(DBG_MODE_BASE + 0x18, 0x53E5C000);
                        mark32(DBG_MODE_BASE + 0x1C, confirm.short_address.0 as u32);
                        let _ = send_device_annce(
                            &mut mac,
                            &mut nwk_seq,
                            &mut aps_seq,
                            &mut zdo_seq,
                        )
                        .await;

                        loop {
                            announce_polls = announce_polls.wrapping_add(1);
                            match mac.mlme_poll().await {
                                Ok(Some(frame)) => {
                                    mark32(DBG_MODE_BASE + 0x20, 0x53E50021);
                                    mark32(DBG_MODE_BASE + 0x24, frame.len() as u32);
                                    let handled = handle_sensor_frame(&frame).await;
                                    mark32(
                                        DBG_MODE_BASE + 0x78,
                                        if handled { 0x53E5BEEF } else { 0x53E50000 },
                                    );
                                }
                                Ok(None) => {
                                    mark32(DBG_MODE_BASE + 0x20, 0x53E50020);
                                }
                                Err(_) => {
                                    mark32(DBG_MODE_BASE + 0x20, 0x53E5FFFF);
                                }
                            }
                            if announce_polls >= SENSOR_ANNOUNCE_PERIOD_POLLS {
                                announce_polls = 0;
                                let _ = send_device_annce(
                                    &mut mac,
                                    &mut nwk_seq,
                                    &mut aps_seq,
                                    &mut zdo_seq,
                                )
                                .await;
                            }
                            async_timer::delay_ms(SENSOR_POLL_INTERVAL_MS).await;
                        }
                    }
                    Ok(confirm) => {
                        board::LED_BLUE.write(false);
                        mark32(
                            DBG_MODE_BASE + 0x18,
                            0x53E50030 | (confirm.status as u32 & 0xFF),
                        );
                    }
                    Err(_) => {
                        board::LED_BLUE.write(false);
                        mark32(DBG_MODE_BASE + 0x18, 0x53E5FF30);
                    }
                }
            }
            _ => {
                board::LED_GREEN.write(false);
                board::LED_RED.write(true);
                mark32(DBG_MODE_BASE + 0x10, 0x53E5FF10);
            }
        }

        async_timer::delay_ms(1000).await;
    }
}

fn main_loop() -> ! {
    // Setup RGB LED pins
    board::LED_RED.set_output();
    board::LED_GREEN.set_output();
    board::LED_BLUE.set_output();

    mark32(DBG_BOOT_BASE + 0x00, 0xCAFE0001_u32);
    mark32(DBG_BOOT_BASE + 0x08, async_timer::now_ticks());

    #[cfg(all(not(feature = "sensor"), not(feature = "diag-assoc")))]
    let rx_buf = core::ptr::addr_of_mut!(RF_RX_BUF) as *mut u8;
    radio::set_rx_dma_config(144);

    board::LED_RED.write(true);
    clear_words(DBG_MODE_BASE, 20);
    mark32(DBG_MODE_BASE + 0x00, 0xD1A60000);

    #[cfg(feature = "sensor")]
    executor::block_on(sensor_main());

    #[cfg(all(not(feature = "sensor"), feature = "diag-assoc"))]
    executor::block_on(diag_assoc_main());

    #[cfg(all(not(feature = "sensor"), not(feature = "diag-assoc")))]
    executor::block_on(diag_beacon_main(rx_buf));

    loop {}
}
