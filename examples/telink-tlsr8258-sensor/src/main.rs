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

// The `sensor` and `runtime-sensor` builds are alternative top-level entry
// points and share many `static mut` slots, so enabling both at once would
// silently drop large blocks of code through `#[cfg(...)]`. Make the build
// fail loudly instead — typical fix is `--no-default-features --features runtime-sensor`.
#[cfg(all(feature = "sensor", feature = "runtime-sensor"))]
compile_error!(
    "features `sensor` and `runtime-sensor` are mutually exclusive; \
     build with `--no-default-features --features <one>`"
);

// `diag-smoke` is a standalone diagnostic build that exercises flash + Timer0
// + IEEE-from-flash on hardware. It must not coexist with the normal sensor
// stacks, otherwise both startup paths would run.
#[cfg(all(feature = "diag-smoke", any(feature = "sensor", feature = "runtime-sensor")))]
compile_error!(
    "feature `diag-smoke` is mutually exclusive with `sensor`/`runtime-sensor`; \
     build with `--no-default-features --features diag-smoke`"
);

// Custom panic handler that records a panic-sentinel in SRAM so we can detect
// silent panics via the dump tool. Writes:
//   DBG_MODE_BASE + 0xF8 = 0xDEAD_BEEF (panic-was-here)
//   DBG_MODE_BASE + 0xFC = LR (caller's return address ≈ panic site)
// Then spins forever. SRAM is preserved on the no-reset SWire dump.
#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    let lr: u32;
    unsafe {
        core::arch::asm!("mov {0}, lr", out(reg) lr);
        core::ptr::write_volatile(0x0084F1F8 as *mut u32, 0xDEAD_BEEF);
        core::ptr::write_volatile(0x0084F1FC as *mut u32, lr);
    }
    loop {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}

#[cfg(all(feature = "runtime-sensor", not(feature = "sensor")))]
mod runtime_sensor;
#[cfg(feature = "sensor")]
use zigbee_aps::frames::{
    ApsCommandId, ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader,
};
#[cfg(feature = "sensor")]
use zigbee_aps::security::{
    derive_key_load_key, derive_key_transport_key, derive_verify_key_hash, ApsKeyType,
    ApsSecurity, ApsSecurityHeader, KEY_ID_DATA_KEY, KEY_ID_KEY_LOAD, KEY_ID_KEY_TRANSPORT,
    SEC_LEVEL_ENC_MIC_32,
};
#[cfg(feature = "sensor")]
use zigbee_aps::{PROFILE_ZDP, ZDO_ENDPOINT};
use zigbee_mac::{MacDriver, MacError, PlatformServices, WrappingTickExtender};
#[cfg(feature = "sensor")]
use zigbee_nwk::frames::{NwkFrameControl, NwkFrameType, NwkHeader};
#[cfg(feature = "sensor")]
use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};
#[cfg(feature = "sensor")]
use zigbee_types::{MacAddress, ShortAddress};
#[cfg(feature = "sensor")]
use zigbee_zdo::device_announce::DeviceAnnounce;
#[cfg(feature = "sensor")]
use zigbee_zdo::DEVICE_ANNCE;

// Keep SWire markers above the stack. Runtime sensor statics occupy low SRAM,
// so diagnostics must not be pinned into the middle of .bss.
const DBG_BOOT_BASE: u32 = 0x0084F000;
const DBG_MODE_BASE: u32 = 0x0084F100;
// Fresh single-writer window for join-path diagnostics.
// Layout (each offset has exactly one writer in the entire crate):
//   +0x250  assoc enter sentinel: 0xA55C_0000 | (channel<<16) | pan_id<<24
//   +0x254  assoc-req: len<<24 | dsn<<16 | cap<<8 | coord_addr_mode
//   +0x258  assoc-req bytes [0..4] (FCF lo|FCF hi|DSN|PAN lo)
//   +0x25C  assoc-req bytes [4..8] (PAN hi|dst_addr lo|dst_addr hi|src ext[0])
//   +0x260  csma_transmit final return: 0=Ok, 0xFFFF=NoAck, 0xFFFE=RadioError
//   +0x264  csma: (tx_ok_count<<16) | (tx_fail_count<<8) | attempts_executed
//   +0x268  csma: ACK rx info: matched<<24 | len<<16 | dsn<<8 | fcf_lo
//   +0x26C  direct-window iterations executed (0..6)
//   +0x270  direct-window frames seen (cumulative)
//   +0x274  direct-window first-frame: len<<24 | dsn<<16 | fcf_le16
//   +0x278  direct-window parse_assoc_response Some(short_addr) | 0xCAFE_0000
//   +0x27C  post-direct sentinel (after 200ms delay): 0xA55C_200D
//   +0x280  data-req loop: attempts_started (0..6)
//   +0x284  data-req TX result bitmap (bit n = attempt n got Ok)
//   +0x288  data-req inner-loop frames seen (cumulative across all attempts)
//   +0x28C  data-req first frame: len<<24 | dsn<<16 | fcf_le16
//   +0x290  data-req parse_assoc_response Some(short_addr) | 0xCAFE_0000
//   +0x294  mlme_associate final outcome: 0xC0DE_xxxx (success) | 0xDEAD_xxxx (fail)
const DBG_JOIN_BASE: u32 = 0x0084F100 + 0x250;
/// BDB-side debug region (must match `zigbee_bdb::steering::tlnk_dbg::BASE`).
/// We write MAC-layer observations into this region from the MAC backend so
/// the BDB and MAC views can be cross-correlated in a single dump.
const DBG_BDB_BASE: u32 = 0x0084_F450;

// ── MAC-layer instrumentation statics (used by csma_transmit / mlme_poll /
// mlme_associate / mcps_data_indication for the "no Transport-Key arrives"
// investigation). All accessed only from the single-threaded executor, so a
// plain `static mut` with explicit volatile is sufficient — but we use
// `AtomicU32`/`AtomicBool` to satisfy strict-aliasing/UB lints uniformly.
use core::sync::atomic::{AtomicU32, Ordering};
/// Timer0 ticks captured at the moment `mlme_associate` returns Ok. 0 = unset.
static ASSOC_OK_TICKS: AtomicU32 = AtomicU32::new(0);
/// 0 = first-poll delay not yet logged to BDB+0xFC; 1 = logged.
static FIRST_POLL_LOGGED: AtomicU32 = AtomicU32::new(0);
/// Frame_pending bit (0/1) extracted from the last matched MAC ACK.
/// 0xFFFF_FFFF = no ACK matched since last reset.
static LAST_ACK_FP: AtomicU32 = AtomicU32::new(0xFFFF_FFFF);
/// Ring index for BDB+0xE0..+0xEF frame capture (0..3 wraparound).
static FRAME_RING_IDX: AtomicU32 = AtomicU32::new(0);
/// Raw-RX classification ring index (BDB+0x100..+0x11F).
/// Counts every frame returned by `receive_raw` BEFORE any filter is applied.
static RX_RAW_RING_IDX: AtomicU32 = AtomicU32::new(0);
/// Sticky cumulative count of PAN-descriptors with `association_permit=true`
/// observed across ALL scans during device lifetime. Published at MODE+0x1C8.
static STICKY_PERMIT_PANS: AtomicU32 = AtomicU32::new(0);
/// Sticky cumulative count of all PAN-descriptors seen across ALL scans.
/// Published at MODE+0x1CC.
static STICKY_TOTAL_PANS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sensor")]
const SENSOR_NV_FLASH_ADDR: u32 = 0x0007_F000;
#[cfg(feature = "sensor")]
const SENSOR_NV_MAGIC: u32 = 0x4B_57_4E_54; // "TNWK" little-endian.
#[cfg(feature = "sensor")]
const SENSOR_NV_VERSION: u8 = 3;
/// Length of the persisted NV record (bytes). Must fit in one flash page
/// (256 B). v3 adds PAN identity fields; we reserve a 64-byte slot to leave
/// room for future fields without re-bumping the version.
#[cfg(feature = "sensor")]
const SENSOR_NV_RECORD_LEN: usize = 64;
/// Reserve window we add to the persisted frame counter on every save.
/// On reboot we resume from `stored + RESERVE`, which guarantees we never
/// re-use a counter that we may have advertised since the last persist
/// (as long as we persist again before the runtime counter advances by
/// `RESERVE`). Kept small to avoid burning the 32-bit counter space on
/// every reboot — the previous value of `0x2000_0000` exhausted the
/// counter after only ~8 reboots.
#[cfg(feature = "sensor")]
const SENSOR_NWK_COUNTER_RESERVE: u32 = 0x0000_4000;
/// Persist whenever the live frame counter has advanced by this many
/// frames since the last persist. Must be strictly less than
/// `SENSOR_NWK_COUNTER_RESERVE` to preserve the no-replay invariant.
#[cfg(feature = "sensor")]
const SENSOR_NWK_COUNTER_PERSIST_INTERVAL: u32 = 0x0000_0400;
#[cfg(feature = "sensor")]
static mut SENSOR_LAST_PERSISTED_COUNTER: u32 = 0;
const SENSOR_STACK_COMPLIANCE_REVISION: u16 = 22;

#[unsafe(link_section = ".debug_sram")]
#[used]
static mut DEBUG_SRAM: [u8; 512] = [0; 512];

// ── Boot vector table (tc32 ISA — native mnemonics) ──────────
//
// The tc32 LLVM assembler accepts standard Thumb-1 mnemonics and
// encodes them as tc32 opcodes automatically. No more .short!
//
//   0x00: tj __reset    (tc32 unconditional branch)
//   0x08: "KNLT" magic
//   0x0C: 0x00880000 + RAM-code preload size/16
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
    ".word 0x00880000 + _ramcode_size_div_16_align_256_",
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
    "ldr r0, =0x84F1DC",
    "ldr r1, =0xA5510001",
    "str r1, [r0, #0]",
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

    // ── 1. Set IRQ and SVC mode stack pointers ──
    // The TLSR8258 has banked SP registers. Match Telink cstartup_8258.S:
    // initialize IRQ mode SP first, then return to SVC mode for normal code.
    // Stack tops come from memory.x so the linker remains the single source
    // of truth (and the `_ebss <= _svc_stack_bottom` assert guards collisions).
    "ldr r0, =0x12",
    "tmcsr r0",
    "ldr r0, =_irq_stack_top",
    "mov sp, r0",
    "ldr r0, =0x13",
    "tmcsr r0",
    "ldr r0, =_svc_stack_top",
    "mov sp, r0",

    // ── 2. Zero I-cache tags (256 bytes = 64 words) ──
    // Unrolled: tc32 LLVM backend has backward branch bugs
    // str Rd,[Rn,#imm] supports imm 0..124 (5-bit * 4)
    "movs r0, #0",
    "ldr r1, =_ictag_start_",
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
    "ldr r1, =0x80060C",
    "ldr r0, =_ramcode_size_div_256_",
    "strb r0, [r1, #0]",      // 0x80060C = RAM-code preload size / 256
    "adds r0, #1",
    "strb r0, [r1, #1]",      // 0x80060D = next I-cache page

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

    // ── 6. Write ASM markers to SRAM (in real SRAM, above BSS) ──
    "ldr r0, =0x84F000",
    "ldr r1, =0xDEADBEEF",
    "str r1, [r0, #0]",      // DBG_BOOT_BASE + 0x00 = DEADBEEF
    "ldr r1, =0x12345678",
    "str r1, [r0, #4]",      // DBG_BOOT_BASE + 0x04 = 12345678

    // ── 7. Jump to Rust _start ──
    "tjl _start",
);

// ── Pure Rust startup ──────────────────────────────────────────

unsafe extern "C" {
    static mut _sdata: u8;
    static mut _edata: u8;
    static mut _sbss: u8;
    static mut _ebss: u8;
    static _etext: u8;
    static _stack_top: u8;
}

core::arch::global_asm!(
    ".section .vectors.startup, \"ax\"",
    ".global _start",
    ".type _start, %function",
    "_start:",
    "ldr r0, =0x84F008",
    "ldr r1, =0x57A70010",
    "str r1, [r0, #0]",
    "ldr r0, =0x84F00C",
    "ldr r1, =0x57A70011",
    "str r1, [r0, #0]",

    "ldr r0, =_sdata",
    "ldr r1, =_edata",
    "cmp r0, r1",
    "bhs 2f",
    "subs r1, r0",
    "ldr r2, =_etext",
    "1:",
    "ldrb r3, [r2]",
    "strb r3, [r0]",
    "adds r2, #1",
    "adds r0, #1",
    "subs r1, #1",
    "bne 1b",
    "2:",
    "ldr r0, =0x84F010",
    "ldr r1, =0x57A70012",
    "str r1, [r0, #0]",

    "ldr r0, =_sbss",
    "ldr r1, =_ebss",
    "cmp r0, r1",
    "bhs 4f",
    "subs r1, r0",
    "movs r2, #0",
    "3:",
    "strb r2, [r0]",
    "adds r0, #1",
    "subs r1, #1",
    "bne 3b",
    "4:",
    "ldr r0, =0x84F014",
    "ldr r1, =0x57A70013",
    "str r1, [r0, #0]",
    "tjl _rust_entry",
);

/// Rust entry point — called from the assembly startup after RAM init.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_entry() -> ! {
    chip_init();
    main_loop();
}

// ── Minimal chip init (replaces HAL startup) ───────────────────
//
// Direct MMIO based on Telink C SDK. No HAL dependency for boot.

#[cfg(all(feature = "runtime-sensor", not(feature = "sensor")))]
#[inline(never)]
fn chip_init() {
    tlsr8258_hal::clocks::init();
}

#[cfg(not(all(feature = "runtime-sensor", not(feature = "sensor"))))]
#[inline(never)]
fn chip_init() {
    let pc_out = (REG_BASE + 0x593) as *mut u8;
    let pb_out = (REG_BASE + 0x58B) as *mut u8;

    // Leave GPIO LED pins untouched during early boot. On TLSR8258 boards these
    // pins can disturb flash/XIP or stall before peripheral clocks are stable.
    // ── Step 1: RED = disable IRQ + reset peripherals ──
    set_led(pc_out, pb_out, true, false, false);

    // Do not touch REG_IRQ_EN here. On TLSR8258 this early write can stall
    // execution while still fetching from flash through XIP.
    // Do not mass-toggle reset control here. The preload reset path already
    // powers the chip, and touching 0x800060..0x800065 from XIP can stall flash
    // fetch by resetting clock/cache related blocks.

    // SKIP analog init — go straight to timer setup
    set_led(pc_out, pb_out, false, false, true); // BLUE = timer setup

    power_clock_init_ram();

    init_timer0();

    // ── Radio init: 802.15.4 Zigbee mode (custom, no HAL) ──
    radio::init();

    // Enable global IRQ. Bits: 0 = Timer0, 4 = DMA, 13 = RF.
    unsafe {
        let irq_mask = core::ptr::read_volatile(0x800640 as *const u32);
        core::ptr::write_volatile(
            0x800640 as *mut u32,
            irq_mask | (1 << 0) | (1 << 4) | (1 << 13),
        );
        core::ptr::write_volatile(0x800643 as *mut u8, 1); // REG_IRQ_EN
    }

    set_led(pc_out, pb_out, false, true, false); // GREEN = init complete
}

/// Initialize TLSR8258 Timer0 as a free-running 24 MHz system-clock counter
/// with capture/compare IRQ. `now_ticks()` reads the live counter; arming an
/// alarm is done by writing `REG_TMR0_CAPT` and re-enabling the Timer0 bit
/// in `REG_IRQ_MASK`. Registers (from Telink B85 `register_8258.h`):
///   `0x800620` REG_TMR_CTRL   bit 0:    TMR0_EN (run)
///                             bit 1:    TMR1_EN
///                             bit 2:    TMR2_EN
///                             bit 3:    TMR_WD_EN   (watchdog — keep cleared)
///                             bits 4-5: TMR0_MODE (00 = SYS_CLK)
///   `0x800623` REG_TMR_STA    W1C; bit 0 clears Timer0 status / IRQ source
///   `0x800624` REG_TMR0_CAPT  compare register; IRQ fires when tick == capt
///   `0x800630` REG_TMR0_TICK  current tick counter (free-running)
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn init_timer0() {
    unsafe {
        // Stop Timer0 and force SYS_CLK mode without disturbing the watchdog
        // (bit 3) or other timer enables (bits 1-2). Clear TMR0_EN (bit 0)
        // and TMR0_MODE (bits 4-5).
        let ctrl = core::ptr::read_volatile(0x800620 as *const u8);
        core::ptr::write_volatile(0x800620 as *mut u8, ctrl & !0x31);
        core::ptr::write_volatile(0x800630 as *mut u32, 0);
        // Park the alarm far in the future so the IRQ does not fire until
        // `arm_timer0_alarm()` programs a real deadline.
        core::ptr::write_volatile(0x800624 as *mut u32, 0xFFFF_FFFF);
        // Clear any latched status / pending source bit for Timer0.
        core::ptr::write_volatile(0x800623 as *mut u8, 0x01);
        core::ptr::write_volatile(0x800648 as *mut u32, 1 << 0);
        // Re-enable Timer0 in SYS_CLK mode (mode = 00, TMR0_EN = bit 0).
        let ctrl = core::ptr::read_volatile(0x800620 as *const u8);
        core::ptr::write_volatile(0x800620 as *mut u8, (ctrl & !0x30) | 0x01);
    }
}

/// Live state for the Timer0-driven sleep helper. `pending` is set by
/// `on_alarm_irq()` and cleared by `blocking_wait_for_alarm()`. The 32-bit
/// counter is plenty for tracking deadlines up to ~178 s with 24 MHz ticks.
static TIMER0_ALARM_PENDING: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Cumulative Timer0 IRQ count. Incremented inside `on_alarm_irq()`. Loaded
/// (without CAS) by the `diag-smoke` harness to verify the FlashCriticalSection
/// re-enables `REG_IRQ_EN` on drop — the count must advance between flash
/// erase iterations or the IRQ-mask save/restore is broken.
pub(crate) static TIMER0_IRQ_COUNT: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn arm_timer0_alarm(ticks_from_now: u32) {
    use core::sync::atomic::Ordering;
    unsafe {
        let now = core::ptr::read_volatile(0x800630 as *const u32);
        let deadline = now.wrapping_add(ticks_from_now);
        core::ptr::write_volatile(0x800624 as *mut u32, deadline);
        // Clear any stale status before unmasking.
        core::ptr::write_volatile(0x800623 as *mut u8, 0x01);
        core::ptr::write_volatile(0x800648 as *mut u32, 1 << 0);
        TIMER0_ALARM_PENDING.store(false, Ordering::Release);
        // Unmask Timer0 IRQ.
        let mask = core::ptr::read_volatile(0x800640 as *const u32);
        core::ptr::write_volatile(0x800640 as *mut u32, mask | (1 << 0));
    }
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
}

#[inline(never)]
fn blocking_wait_for_alarm() {
    use core::sync::atomic::Ordering;
    while !TIMER0_ALARM_PENDING.load(Ordering::Acquire) {
        unsafe { core::arch::asm!("nop") };
    }
    // Mask the alarm again so the IRQ stays quiet between sleeps.
    unsafe {
        let mask = core::ptr::read_volatile(0x800640 as *const u32);
        core::ptr::write_volatile(0x800640 as *mut u32, mask & !(1 << 0));
    }
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn mark32(addr: u32, val: u32) {
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, val);
    }
}

/// Capture a frame that was received during the assoc-wait window but was
/// NOT an Associate Response (so we silently dropped it). Used to verify the
/// theory that EZSP's queued Transport-Key gets consumed and dropped here.
/// Layout (JOIN_BASE-relative):
///   +0x160: u32 dropped-frame counter (incremented on every drop)
///   +0x164: first-drop u32 packed = (plen<<24) | (seq<<16) | fcf
///   +0x168..+0x178: first 16 bytes of first dropped frame (4 u32 words, LE)
/// First-write-wins for +0x164.. so we always preserve the *first* drop.
#[inline(never)]
fn capture_dropped_assoc_frame(data: &[u8]) {
    unsafe {
        // bump counter
        let cnt_p = (DBG_JOIN_BASE + 0x160) as *mut u32;
        core::ptr::write_volatile(cnt_p, core::ptr::read_volatile(cnt_p).wrapping_add(1));
        // first-write-wins for the FCF+seq+plen word
        let hdr_p = (DBG_JOIN_BASE + 0x164) as *mut u32;
        if core::ptr::read_volatile(hdr_p) == 0 {
            let fcf = u16::from_le_bytes([
                data.first().copied().unwrap_or(0),
                data.get(1).copied().unwrap_or(0),
            ]) as u32;
            let seq = data.get(2).copied().unwrap_or(0) as u32;
            let plen = data.len() as u32 & 0xFF;
            core::ptr::write_volatile(hdr_p, (plen << 24) | (seq << 16) | fcf);
            // also dump first 16 bytes (4 words) for full visibility
            for i in 0u32..4 {
                let off = (i as usize) * 4;
                let w = u32::from_le_bytes([
                    data.get(off).copied().unwrap_or(0),
                    data.get(off + 1).copied().unwrap_or(0),
                    data.get(off + 2).copied().unwrap_or(0),
                    data.get(off + 3).copied().unwrap_or(0),
                ]);
                let p = (DBG_JOIN_BASE + 0x168 + i * 4) as *mut u32;
                core::ptr::write_volatile(p, w);
            }
        }
    }
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn clear_words(base: u32, words: usize) {
    for idx in 0..words {
        mark32(base + (idx as u32 * 4), 0);
    }
}

#[inline(never)]
fn mark_bytes_as_words(base: u32, data: &[u8]) {
    let mut idx = 0usize;
    while idx < data.len() && idx < 64 {
        let b0 = data.get(idx).copied().unwrap_or(0);
        let b1 = data.get(idx + 1).copied().unwrap_or(0);
        let b2 = data.get(idx + 2).copied().unwrap_or(0);
        let b3 = data.get(idx + 3).copied().unwrap_or(0);
        mark32(
            base + idx as u32,
            u32::from_le_bytes([b0, b1, b2, b3]),
        );
        idx += 4;
    }
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(always)]
#[unsafe(link_section = ".ram_code")]
fn mspi_wait() {
    while unsafe { core::ptr::read_volatile(0x80000D as *const u8) } & 0x10 != 0 {}
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(always)]
#[unsafe(link_section = ".ram_code")]
fn mspi_high() {
    unsafe { core::ptr::write_volatile(0x80000D as *mut u8, 0x01) };
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(always)]
#[unsafe(link_section = ".ram_code")]
fn mspi_low() {
    unsafe { core::ptr::write_volatile(0x80000D as *mut u8, 0x00) };
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(always)]
#[unsafe(link_section = ".ram_code")]
fn mspi_write(byte: u8) {
    unsafe { core::ptr::write_volatile(0x80000C as *mut u8, byte) };
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(always)]
#[unsafe(link_section = ".ram_code")]
fn mspi_get() -> u8 {
    unsafe { core::ptr::read_volatile(0x80000C as *const u8) }
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(always)]
#[unsafe(link_section = ".ram_code")]
fn mspi_read() -> u8 {
    mspi_write(0);
    mspi_wait();
    mspi_get()
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn flash_send_cmd(cmd: u8) {
    mspi_high();
    spin_delay(24);
    mspi_low();
    mspi_write(cmd);
    mspi_wait();
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn flash_send_addr(addr: u32) {
    mspi_write((addr >> 16) as u8);
    mspi_wait();
    mspi_write((addr >> 8) as u8);
    mspi_wait();
    mspi_write(addr as u8);
    mspi_wait();
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn flash_wait_done() {
    spin_delay(2_400);
    flash_send_cmd(0x05);
    for _ in 0..10_000_000u32 {
        if mspi_read() & 0x01 == 0 {
            break;
        }
    }
    mspi_high();
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn flash_read_data(addr: u32, out: &mut [u8]) {
    flash_send_cmd(0x03);
    flash_send_addr(addr);
    mspi_write(0x00);
    mspi_wait();
    unsafe { core::ptr::write_volatile(0x80000D as *mut u8, 0x0A) };
    mspi_wait();
    for byte in out.iter_mut() {
        *byte = mspi_get();
        mspi_wait();
    }
    mspi_high();
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn flash_write_cmd(cmd: u8, addr: u32, data: &[u8]) {
    flash_send_cmd(0x06);
    flash_send_cmd(cmd);
    flash_send_addr(addr);
    for byte in data {
        mspi_write(*byte);
        mspi_wait();
    }
    mspi_high();
    flash_wait_done();
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn flash_erase_sector(addr: u32) {
    flash_write_cmd(0x20, addr, &[]);
}

/// RAII guard that disables the global IRQ enable (`REG_IRQ_EN` at `0x800643`)
/// for the lifetime of the guard and restores the prior value on drop.
///
/// Flash erase/program ops on TLSR8258 take milliseconds and run from
/// `.ram_code`. The RF/timer ISRs must not fire mid-cycle: a stray volatile
/// MSPI write from an ISR corrupts the flash command sequence and bricks the
/// NV record. Wrap every erase/program/read sequence in one of these guards.
#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
struct FlashCriticalSection {
    prev: u8,
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
impl FlashCriticalSection {
    #[inline(always)]
    fn enter() -> Self {
        let reg = 0x800643 as *mut u8;
        let prev = unsafe { core::ptr::read_volatile(reg) };
        unsafe { core::ptr::write_volatile(reg, 0) };
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        Self { prev }
    }
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
impl Drop for FlashCriticalSection {
    #[inline(always)]
    fn drop(&mut self) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { core::ptr::write_volatile(0x800643 as *mut u8, self.prev) };
    }
}

#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn flash_page_program(addr: u32, data: &[u8]) {
    let mut written = 0usize;
    while written < data.len() {
        let page_left = 256usize - ((addr as usize + written) & 0xFF);
        let n = (data.len() - written).min(page_left);
        flash_write_cmd(0x02, addr + written as u32, &data[written..written + n]);
        written += n;
    }
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn power_clock_init_ram() {
    mark32(DBG_BOOT_BASE + 0x40, 0xC10C0001);
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
    mark32(DBG_BOOT_BASE + 0x40, 0xC10C0002);

    spin_delay(20_000);
    unsafe {
        core::ptr::write_volatile((REG_BASE + 0x066) as *mut u8, 0x42);
    }
    spin_delay(5_000);
    unsafe {
        core::ptr::write_volatile((REG_BASE + 0x063) as *mut u8, 0xFF);
        core::ptr::write_volatile((REG_BASE + 0x064) as *mut u8, 0xFF);
        core::ptr::write_volatile((REG_BASE + 0x065) as *mut u8, 0xFF);
        let clk = core::ptr::read_volatile((REG_BASE + 0x066) as *const u8);
        let clk2 = core::ptr::read_volatile((REG_BASE + 0x065) as *const u8);
        mark32(DBG_BOOT_BASE + 0x44, clk as u32 | ((clk2 as u32) << 8));
    }
    mark32(DBG_BOOT_BASE + 0x40, 0xC10C0003);
}

#[inline(always)]
fn spin_delay(iterations: u32) {
    for _ in 0..iterations {
        unsafe { core::arch::asm!("nop"); }
    }
}

#[inline(never)]
#[allow(dead_code)]
fn spin_delay_ms(ms: u32) {
    // Fallback CPU spin (kept for emergency bring-up if Timer0 is broken).
    for _ in 0..ms {
        spin_delay(24_000);
    }
}

/// IRQ handler — called from assembly __irq via `bl irq_handler`
#[unsafe(no_mangle)]
#[unsafe(link_section = ".ram_code")]
#[cfg(not(all(feature = "runtime-sensor", not(feature = "sensor"))))]
pub extern "C" fn irq_handler() {
    unsafe {
        static mut IRQ_COUNT: u32 = 0;
        IRQ_COUNT = IRQ_COUNT.wrapping_add(1);
        mark32(DBG_MODE_BASE + 0xE0, 0x1F510000 | (IRQ_COUNT & 0xFFFF));

        let rf_irq_status = core::ptr::read_volatile(0x800F20 as *const u16);
        let rf_irq_mask = core::ptr::read_volatile(0x800F1C as *const u16);
        mark32(
            DBG_MODE_BASE + 0xE4,
            (rf_irq_status as u32) | ((rf_irq_mask as u32) << 16),
        );
        if (rf_irq_status & rf_irq_mask & 0x01) != 0 {
            handle_rf_rx_irq();
        }

        let irq_src = core::ptr::read_volatile(0x800648 as *const u32); // REG_IRQ_SRC
        let irq_mask = core::ptr::read_volatile(0x800640 as *const u32); // REG_IRQ_MASK
        let pending = irq_src & irq_mask;
        mark32(DBG_MODE_BASE + 0xE8, pending);

        // Timer0 IRQ (bit 0)
        if pending & (1 << 0) != 0 {
            core::ptr::write_volatile(0x800648 as *mut u32, 1 << 0); // ack IRQ source
            core::ptr::write_volatile(0x800623 as *mut u8, 0x01);    // ack timer status
            async_timer::on_alarm_irq();
        }

        // DMA IRQ (bit 4), used by RF RX DMA completion.
        if pending & (1 << 4) != 0 {
            core::ptr::write_volatile(0x800648 as *mut u32, 1 << 4);
            let dma_status = core::ptr::read_volatile(0x800C26 as *const u8);
            if dma_status & 0x04 != 0 {
                handle_rf_rx_irq();
                core::ptr::write_volatile(0x800C26 as *mut u8, 0x04);
            }
        }

        // Zigbee/RF IRQ (bit 13). Some modes do not assert this for manual RX,
        // but keep it enabled as a fallback for future auto-mode experiments.
        if pending & (1 << 13) != 0 {
            core::ptr::write_volatile(0x800648 as *mut u32, 1 << 13);
            handle_rf_rx_irq();
        }

        core::ptr::write_volatile(0x800643 as *mut u8, 1);
    }
}

// The reusable TLSR8258 backend is fully polled and leaves CPU interrupts
// disabled. Keep the application-owned vector valid without retaining the
// legacy local RF/DMA interrupt path in runtime-sensor.
#[unsafe(no_mangle)]
#[unsafe(link_section = ".ram_code")]
#[cfg(all(feature = "runtime-sensor", not(feature = "sensor")))]
pub extern "C" fn irq_handler() {}

// ── Radio DMA buffers (must be 4-byte aligned, in RAM) ─────────

#[repr(align(4))]
#[allow(dead_code)]
struct DmaBuf([u8; 144]);

static mut RF_RX_BUF: DmaBuf = DmaBuf([0u8; 144]);
static mut RF_TX_BUF: DmaBuf = DmaBuf([0u8; 144]);

/// Write PHY register tables extracted from Telink SDK libdrivers_8258.a rf_drv_init().
/// This is the missing Zigbee PHY init that the HAL radio.rs doesn't include.
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn rf_phy_init_zigbee() {
    unsafe {
        // Diagnostic: mark that PHY init started
        mark32(DBG_BOOT_BASE + 0x28, 0xDEAD0001_u32);

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
        mark32(DBG_BOOT_BASE + 0x2C,
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
        mark32(DBG_BOOT_BASE + 0x30,
            (r0400 as u32) | ((r1220 as u32) << 8) | ((r12d2 as u32) << 16) | ((r0460 as u32) << 24));
        // Also test: write 0xAB to 0x800400, read back
        core::ptr::write_volatile(0x800400 as *mut u8, 0xAB);
        let test_rb = core::ptr::read_volatile(0x800400 as *const u8);
        // And test a known-working reg: write 0x55 to unused RAM, read back
        core::ptr::write_volatile((DBG_BOOT_BASE + 0x3C) as *mut u8, 0x55);
        let ram_rb = core::ptr::read_volatile((DBG_BOOT_BASE + 0x3C) as *const u8);
        mark32(DBG_BOOT_BASE + 0x34,
            (test_rb as u32) | ((ram_rb as u32) << 8));
        // Restore 0x800400 to correct value
        core::ptr::write_volatile(0x800400 as *mut u8, 0x13);
        // Mark PHY init complete
        mark32(DBG_BOOT_BASE + 0x28, 0xBBBB0002_u32);
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
    const REG_DMA_IRQ_STATUS: *mut u8 = 0x800C26 as *mut u8; // DMA RF RX/TX ready status
    const REG_IRQ_SRC: *mut u32 = 0x800648 as *mut u32;      // CPU IRQ source W1C

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

    /// Mask TX IRQ while a manual ACK is being polled.
    #[inline(always)]
    pub fn mask_tx_irq() -> u16 {
        unsafe {
            let v = core::ptr::read_volatile(REG_RF_IRQ_MASK as *const u16);
            core::ptr::write_volatile(REG_RF_IRQ_MASK, v & !0x02);
            v
        }
    }

    /// Restore TX IRQ mask after a manual ACK.
    #[inline(always)]
    pub fn restore_irq_mask(mask: u16) {
        unsafe {
            core::ptr::write_volatile(REG_RF_IRQ_MASK, mask);
        }
    }

    /// Set RF IRQ mask for TX and RX done
    #[inline(always)]
    pub fn set_irq_mask_tx_rx() {
        unsafe {
            let v = core::ptr::read_volatile(REG_RF_IRQ_MASK as *const u16);
            // Only RX IRQ is needed. TX completion is polled by transmit paths;
            // enabling TX IRQ here races with that polling and can clear status
            // before tx_and_wait() sees it.
            core::ptr::write_volatile(REG_RF_IRQ_MASK, v | 0x01); // bit0=RX
        }
    }

    /// Set TX pipe
    #[inline(always)]
    pub fn set_tx_pipe(pipe: u8) {
        unsafe {
            core::ptr::write_volatile(REG_RF_LL_CTRL_2, 0x10 | (pipe & 0x07));
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
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
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

    /// Disable RX analog path without resetting the whole RF state machine.
    #[inline(always)]
    pub fn disable_rx_mode() {
        unsafe {
            core::ptr::write_volatile(REG_RF_RX_MODE, RF_TRX_MODE);
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
            // RF RX DMA completion is a separate W1C latch. Leaving it set can
            // keep the CPU DMA source pending without producing a new IRQ edge.
            core::ptr::write_volatile(REG_DMA_IRQ_STATUS, 0x04);
            core::ptr::write_volatile(REG_IRQ_SRC, 1 << 4);
        }
    }

    /// Clear TX done flag
    #[inline(always)]
    pub fn tx_done_clear() {
        unsafe {
            core::ptr::write_volatile(0x800F20 as *mut u8, 0x02);
            core::ptr::write_volatile(REG_DMA_IRQ_STATUS, 0x08);
            core::ptr::write_volatile(REG_IRQ_SRC, 1 << 4);
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

    /// Set TX DMA buffer address and trigger RF TX DMA.
    /// Based on drivers_8258 rf_tx_pkt():
    /// 1. Write 0x04 to 0x800C43 (DMA3 addr high, RAM region)
    /// 2. Write buf addr low 16 bits to 0x800C0C (DMA3 addr low)
    /// 3. Set bit 3 of 0x800C5B to trigger RF TX.
    /// 4. Also set DMA TX ready bit 3; this keeps the current XIP bring-up path
    ///    compatible with the previously working manual TX sequence.
    #[inline(always)]
    pub fn tx_pkt(addr: *const u8) {
        let a = addr as usize;
        unsafe {
            core::ptr::write_volatile(REG_DMA3_ADDR_HI, 0x04);
            core::ptr::write_volatile(REG_DMA3_ADDR, (a & 0xFFFF) as u16);
            let v = core::ptr::read_volatile(REG_RF_TX_TRIGGER as *const u8);
            core::ptr::write_volatile(REG_RF_TX_TRIGGER, v | 0x08);
            let dma = core::ptr::read_volatile(REG_DMA_TX_READY as *const u8);
            core::ptr::write_volatile(REG_DMA_TX_READY, dma | 0x08);
        }
    }

    // RF TX trigger register used by rf_tx_pkt() on TLSR8258.
    const REG_RF_TX_TRIGGER: *mut u8 = 0x800C5B as *mut u8;
    const REG_DMA_TX_READY: *mut u8 = 0x800C24 as *mut u8;

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
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    pub fn init() {
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100000);
        // Step 1: Reset and configure state machine
        // Avoid RST1_ZB reset here: writing 0x800061 after XIP is active stalls
        // this target. Keep the boot-time peripheral state for now.
        set_auto_mode();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100001);
        set_trx_off();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100002);
        reset_sn();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100003);
        clear_irq_status();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100004);
        clear_irq_mask();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100005);
        set_tx_pipe(0);
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100006);
        set_tx_settle(DEFAULT_TX_SETTLE_US);
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100007);

        // Step 2: Write PHY registers (must be AFTER reset_baseband)
        super::rf_phy_init_zigbee();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100017);

        // Step 3: Set initial channel
        set_channel(11);
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100008);

        // Step 4: Configure DMA (size config moved to async_main for codegen stability)
        let rx_ptr = core::ptr::addr_of_mut!(super::RF_RX_BUF) as *mut u8;
        set_rx_buffer(rx_ptr);
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA1100009);
        rx_buf_clear(rx_ptr);
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA110000A);
        enable_dma_rx();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA110000B);
        enable_dma_tx();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA110000C);
        unsafe {
            core::ptr::write_volatile(REG_DMA_IRQ_STATUS, 0x0C);
            core::ptr::write_volatile(REG_IRQ_SRC, 1 << 4);
            core::ptr::write_volatile(REG_DMA_CHN_IRQ_MSK, 0x04);
            let ctrl1 = core::ptr::read_volatile(REG_RF_LL_CTRL_1 as *const u8);
            core::ptr::write_volatile(REG_RF_LL_CTRL_1, ctrl1 | (1 << 5));
        }
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA110000D);

        // Step 5: Enable RF IRQs
        set_irq_mask_tx_rx();
        super::mark32(super::DBG_BOOT_BASE + 0x20, 0xA110000E);
    }
}

/// Direct analog register write with timeout
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
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
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
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
    let _ = (pc_out, pb_out, r, g, b);
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
    // Block on a Timer0 IRQ so the wakeup is accurate to ±1 tick (~42 ns)
    // instead of the unbounded NOP-spin slop. `arm_timer0_alarm` clamps to
    // `u32::MAX` ticks (~178 s @ 24 MHz); callers do not exceed that.
    let ticks = ms.saturating_mul(24_000);
    arm_timer0_alarm(ticks);
    blocking_wait_for_alarm();
}

// ── Polling MAC driver ─────────────────────────────────────────

use zigbee_mac::primitives::*;

const MAX_MAC_FRAME_LEN: usize = 127;
const MAX_MAC_OVERHEAD: usize = 25;
const ACK_WAIT_LOOPS: u32 = 36_000;
const POLL_RESPONSE_LOOPS: u32 = 1_200_000;
// Per-call budget for `mcps_data_indication` (the BDB Phase-0 passive RX hook).
// Each loop iteration ≈ 1 µs on TLSR8258 @24 MHz, so 1.2M ≈ ~100 ms wall-clock
// per call. BDB Phase 0 invokes this 4× → ~400 ms total before transitioning
// to Phase 1 (parent_poll). Previously 12_000_000 (~1 s/call → 4 s total),
// which exceeded the TC indirect-transmission lifetime (~3 s) and caused the
// Transport-Key to expire from the router's indirect queue before our first
// Data Request reached the parent. Do NOT increase without re-checking the
// assoc→first-poll latency marker at BDB+0xFC.
const RX_INDICATION_LOOPS: u32 = 1_200_000;

static mut IRQ_RX_DATA: [u8; MAX_MAC_FRAME_LEN] = [0; MAX_MAC_FRAME_LEN];
static mut IRQ_RX_LEN: usize = 0;
static mut IRQ_RX_RSSI: i8 = -110;
static mut IRQ_RX_LQI: u8 = 0;
/// Release/Acquire handshake: ISR sets to 1 *after* writing data/len/rssi/lqi;
/// main loop reads to 0 *after* copying the slot. Plain `static mut` payload
/// accesses are sequenced by this atomic plus a `compiler_fence(SeqCst)`.
static IRQ_RX_PENDING: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);
/// MAC ACK filter packed as `(pan_id as u32) | ((short_addr as u32) << 16)` so
/// the ISR sees a consistent `{pan_id, short_addr}` tuple in a single load.
static MAC_ACK_FILTER: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0xFFFF_FFFF);

struct RxPacket {
    data: [u8; MAX_MAC_FRAME_LEN],
    len: usize,
    #[allow(dead_code)]
    rssi: i8,
    lqi: u8,
}

/// A raw MAC frame queued during `mlme_associate` for later delivery via
/// `mcps_data_indication`. Frames received in the assoc response windows that
/// aren't the Associate Response itself (e.g. Transport-Key) are otherwise
/// silently dropped after the hardware auto-ACKs them. See Step-2 plan.
#[derive(Clone)]
struct PendingPacket {
    data: heapless::Vec<u8, MAX_MAC_FRAME_LEN>,
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
    /// Queue of MAC frames received during `mlme_associate` that weren't the
    /// Associate Response. Drained from the head by `mcps_data_indication`.
    pending_rx: heapless::Deque<PendingPacket, 4>,
    clock: WrappingTickExtender,
}

impl Tlsr8258Mac {
    pub fn new() -> Self {
        let rx_buf = core::ptr::addr_of_mut!(RF_RX_BUF) as *mut u8;
        let now_ticks = async_timer::now_ticks();
        radio::set_rx_dma_config(144);
        radio::set_rx_buffer(rx_buf);
        radio::set_channel(15);
        update_mac_ack_filter(
            zigbee_types::PanId::BROADCAST,
            zigbee_types::ShortAddress::BROADCAST,
        );

        Self {
            rx_buf,
            short_address: zigbee_types::ShortAddress::BROADCAST,
            pan_id: zigbee_types::PanId::BROADCAST,
            channel: 15,
            extended_address: our_ext_addr(),
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
            pending_rx: heapless::Deque::new(),
            clock: WrappingTickExtender::new(now_ticks),
        }
    }

    fn extended_timer_ticks(&self) -> u64 {
        self.clock.extend(async_timer::now_ticks())
    }

    /// Try to enqueue a MAC frame received during `mlme_associate` for later
    /// delivery via `mcps_data_indication`. Frames are filtered by destination
    /// address using the same rule as `mlme_poll`. Diagnostics:
    ///  +0x180  enqueue success count
    ///  +0x184  drain count (incremented in `mcps_data_indication`)
    ///  +0x188  queue-overflow drop count
    ///  +0x18C  filter-rejected drop count
    fn maybe_enqueue_pending(&mut self, data: &[u8], lqi: u8) {
        if data.len() < 5 {
            return;
        }
        let fc = u16::from_le_bytes([data[0], data[1]]);
        let frame_type = fc & 0x07;
        // Only data frames carry NWK payloads (Transport-Key, link status, ...).
        if frame_type != 1 {
            return;
        }
        let Some(dst) = parse_dest_address(data, fc) else {
            return;
        };
        if !address_matches(
            &dst,
            self.pan_id,
            self.short_address,
            self.extended_address,
        ) {
            unsafe {
                let p = (DBG_JOIN_BASE + 0x18C) as *mut u32;
                core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            }
            return;
        }
        let mut buf: heapless::Vec<u8, MAX_MAC_FRAME_LEN> = heapless::Vec::new();
        if buf.extend_from_slice(data).is_err() {
            return;
        }
        match self.pending_rx.push_back(PendingPacket { data: buf, lqi }) {
            Ok(()) => unsafe {
                let p = (DBG_JOIN_BASE + 0x180) as *mut u32;
                core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            },
            Err(_) => unsafe {
                let p = (DBG_JOIN_BASE + 0x188) as *mut u32;
                core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            },
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
        // TLSR8258 needs a short PLL settle window after channel changes.
        spin_delay(2400);
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

    fn receive_raw(&mut self, timeout_loops: u32) -> Result<RxPacket, MacError> {
        let pkt = receive_packet(
            self.rx_buf,
            timeout_loops,
            self.pan_id,
            self.short_address,
            self.extended_address,
        )
        .ok_or(MacError::NoData)?;
        // Pre-filter classification. Captures EVERY frame the radio returns
        // (post hardware-address filter, but BEFORE any software filtering
        // in mlme_poll / mcps_data_indication). Output region:
        //   BDB+0x100..+0x11F : 4-entry ring × 8 bytes (word A | word B)
        //     word A = fcf_lo | (fcf_hi<<8) | (seq<<16) | (dst_pan_hi<<24)
        //     word B = src_short | (dst_short<<16)
        //   BDB+0x120 : count of frames with dst_short == our short
        //   BDB+0x124 : count of broadcasts (dst_short >= 0xFFFC)
        //   BDB+0x128 : count of other-dest frames
        //   BDB+0x12C : count of APS frames with apsFC == 0x21 (TK candidates)
        //   BDB+0x130 : count of sec=0 NWK frames addressed to us (the grail)
        //   BDB+0x134 : raw receive_raw return count (pre-filter total)
        let was_tk_candidate = classify_rx_frame(&pkt.data[..pkt.len], self.short_address.0);
        // Pipeline counters for the TK-delivery path. Recorded at MODE+0x200..+0x20F.
        //   MODE+0x200 : receive_raw() returned a TK candidate (sec=0 + dst==us + apsFC==0x21)
        //   MODE+0x204 : force-enqueue success count (pushed into pending_rx here)
        //   MODE+0x208 : force-enqueue failure (queue full / extend error)
        //   MODE+0x20C : reserved for mcps_data_indication TK drain
        if was_tk_candidate {
            unsafe {
                let p = (DBG_MODE_BASE + 0x200) as *mut u32;
                core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            }
            // Force-enqueue. This bypasses the (already-passing) address filter in
            // maybe_enqueue_pending, but uses the same queue so mcps_data_indication
            // will drain it. If the queue is full we still count the miss.
            let mut buf: heapless::Vec<u8, MAX_MAC_FRAME_LEN> = heapless::Vec::new();
            if buf.extend_from_slice(&pkt.data[..pkt.len]).is_ok() {
                match self.pending_rx.push_back(PendingPacket { data: buf, lqi: pkt.lqi }) {
                    Ok(()) => unsafe {
                        let p = (DBG_MODE_BASE + 0x204) as *mut u32;
                        core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
                    },
                    Err(_) => unsafe {
                        let p = (DBG_MODE_BASE + 0x208) as *mut u32;
                        core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
                    },
                }
            } else {
                unsafe {
                    let p = (DBG_MODE_BASE + 0x208) as *mut u32;
                    core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
                }
            }
        }
        Ok(pkt)
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
        // FCF high byte: src_addr_mode=ext(11) | frame_version=0 (2003 — REQUIRED for
        // assoc-req per IEEE 802.15.4; some coordinators drop FV=1) | dst_addr_mode.
        let fc_hi = (0b11 << 6) | (dst_mode << 2);
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
        // Use the requested capability information as-is (no override hacks).
        let cap_byte = capability_info.to_byte();
        let _ = frame.push(cap_byte);
        // Diagnostic capture: what we ACTUALLY put on the wire.
        //   MODE+0x1E0 : the cap byte (low 8 bits)
        //   MODE+0x1E4 : marker so we can tell the slot is set (0xCAFE_BABE)
        //   MODE+0x1E8 : src_ext LE bytes [0..3] as TX'd
        //   MODE+0x1EC : src_ext LE bytes [4..7] as TX'd
        unsafe {
            core::ptr::write_volatile(
                (DBG_MODE_BASE + 0x1E0) as *mut u32,
                (cap_byte as u32) | ((capability_info.to_byte() as u32) << 8) | 0xCAFE_0000,
            );
            core::ptr::write_volatile(
                (DBG_MODE_BASE + 0x1E4) as *mut u32,
                0xCAFE_BABE,
            );
            let e = &self.extended_address;
            let lo = (e[0] as u32)
                | ((e[1] as u32) << 8)
                | ((e[2] as u32) << 16)
                | ((e[3] as u32) << 24);
            let hi = (e[4] as u32)
                | ((e[5] as u32) << 8)
                | ((e[6] as u32) << 16)
                | ((e[7] as u32) << 24);
            core::ptr::write_volatile((DBG_MODE_BASE + 0x1E8) as *mut u32, lo);
            core::ptr::write_volatile((DBG_MODE_BASE + 0x1EC) as *mut u32, hi);
        }
        frame
    }

    fn build_data_request(&mut self, coord: &zigbee_types::MacAddress) -> heapless::Vec<u8, 24> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();
        let dst_mode: u8 = match coord {
            zigbee_types::MacAddress::Short(_, _) => 0b10,
            zigbee_types::MacAddress::Extended(_, _) => 0b11,
        };
        let src_mode: u8 = if self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE {
            0b10
        } else {
            0b11
        };
        let fc_lo = 0b0110_0011u8;
        // frame_version=0 (2003); match the assoc-req convention used by the parent.
        let fc_hi = (src_mode << 6) | (dst_mode << 2);
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
        if src_mode == 0b10 {
            let _ = frame.extend_from_slice(&self.short_address.0.to_le_bytes());
        } else {
            let _ = frame.extend_from_slice(&self.extended_address);
        }
        let _ = frame.push(0x04);
        frame
    }

    fn parse_assoc_response_packet(&mut self, data: &[u8]) -> Option<MlmeAssociateConfirm> {
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
            (data.len() as u32) | ((fc as u32) << 8) | ((cmd_id as u32) << 24),
        );
        mark32(DBG_MODE_BASE + 0x68, frame_word);
        if data.len() < 5 || fc & 0x07 != 3 {
            return None;
        }
        if data.len() < cmd_offset + 4 || data[cmd_offset] != 0x02 {
            return None;
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
            update_mac_ack_filter(self.pan_id, self.short_address);
        }
        mark32(
            DBG_MODE_BASE + 0x60,
            0xA55C1000 | (status as u32 & 0xFF) | ((short_addr as u32) << 8),
        );
        Some(MlmeAssociateConfirm {
            short_address: zigbee_types::ShortAddress(short_addr),
            status,
        })
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
        let expected_dsn = frame.get(2).copied().unwrap_or(0);
        mark32(
            DBG_MODE_BASE + 0x80,
            0xC50A0000
                | ((expected_dsn as u32) << 8)
                | (max_retries as u32 & 0xFF)
                | ((frame.len() as u32 & 0xFF) << 16),
        );
        mark32(
            DBG_MODE_BASE + 0x84,
            (self.channel as u32) | ((self.pan_id.0 as u32) << 8),
        );
        let mut rx_total: u32 = 0;
        // Per-call counters for join-path window (+0x264 / +0x268).
        let mut tx_ok_count: u32 = 0;
        let mut tx_fail_count: u32 = 0;
        let mut attempts_executed: u32 = 0;
        let mut ack_recorded: bool = false;
        let write_csma_counters = |ok: u32, fail: u32, attempts: u32| {
            mark32(
                DBG_JOIN_BASE + 0x14, // +0x264 = DBG_MODE_BASE+0x250+0x14
                ((ok & 0xFFFF) << 16) | ((fail & 0xFF) << 8) | (attempts & 0xFF),
            );
        };
        for attempt in 0..=max_retries {
            attempts_executed = attempt as u32 + 1;
            let tx_ok = self.transmit_raw(frame).is_ok();
            if tx_ok {
                tx_ok_count += 1;
            } else {
                tx_fail_count += 1;
            }
            write_csma_counters(tx_ok_count, tx_fail_count, attempts_executed);
            mark32(
                DBG_MODE_BASE + 0x88,
                0xC50A2000 | ((attempt as u32) << 8) | (tx_ok as u32),
            );
            if !tx_ok {
                if attempt == max_retries {
                    mark32(DBG_MODE_BASE + 0x9C, 0xC50AFE00 | attempt as u32);
                    mark32(DBG_JOIN_BASE + 0x10, 0xFFFE);
                    return Err(MacError::RadioError);
                }
                continue;
            }
            if !ack_requested {
                mark32(DBG_MODE_BASE + 0x9C, 0xC50A0001);
                mark32(DBG_JOIN_BASE + 0x10, 0x0000_0001);
                return Ok(());
            }
            match self.receive_raw(ACK_WAIT_LOOPS) {
                Ok(pkt) => {
                    rx_total = rx_total.wrapping_add(1);
                    let data = &pkt.data[..pkt.len];
                    let b0 = data.first().copied().unwrap_or(0) as u32;
                    let b1 = data.get(1).copied().unwrap_or(0) as u32;
                    let b2 = data.get(2).copied().unwrap_or(0) as u32;
                    let b3 = data.get(3).copied().unwrap_or(0) as u32;
                    mark32(
                        DBG_MODE_BASE + 0x8C,
                        (pkt.len as u32 & 0xFF)
                            | ((attempt as u32 & 0xFF) << 8)
                            | ((rx_total & 0xFFFF) << 16),
                    );
                    mark32(
                        DBG_MODE_BASE + 0x90,
                        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24),
                    );
                    let matched = is_ack_for(data, expected_dsn);
                    // Capture ACK frame_pending bit (FCF byte 0, bit 4) for
                    // the most recently matched ACK. This lets `mlme_poll`
                    // increment BDB+0x1B8 (fp=1) vs +0x1BC (fp=0) counters.
                    if matched {
                        let fp = ((b0 >> 4) & 0x1) as u32;
                        LAST_ACK_FP.store(fp, Ordering::Relaxed);
                    }
                    if !ack_recorded {
                        ack_recorded = true;
                        mark32(
                            DBG_JOIN_BASE + 0x18, // +0x268
                            ((matched as u32) << 24)
                                | ((pkt.len as u32 & 0xFF) << 16)
                                | ((b2 & 0xFF) << 8)
                                | (b0 & 0xFF),
                        );
                    }
                    mark32(
                        DBG_MODE_BASE + 0x94,
                        0xC50A4000
                            | (matched as u32)
                            | ((b2 as u32) << 8)
                            | ((expected_dsn as u32) << 16),
                    );
                    if matched {
                        mark32(DBG_MODE_BASE + 0x9C, 0xC50A0002 | ((attempt as u32) << 8));
                        mark32(DBG_JOIN_BASE + 0x10, 0x0000_0001);
                        return Ok(());
                    }
                }
                Err(_) => {
                    mark32(
                        DBG_MODE_BASE + 0x94,
                        0xC50A5000 | (attempt as u32),
                    );
                }
            }
            if attempt == max_retries {
                mark32(DBG_MODE_BASE + 0x9C, 0xC50AFFFF);
                mark32(DBG_JOIN_BASE + 0x10, 0xFFFF);
                return Err(MacError::NoAck);
            }
        }
        mark32(DBG_MODE_BASE + 0x9C, 0xC50AFFFE);
        mark32(DBG_JOIN_BASE + 0x10, 0xFFFD);
        Err(MacError::NoAck)
    }
}

impl MacDriver for Tlsr8258Mac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        // Diag: count mlme_scan entries
        unsafe {
            let p = (DBG_JOIN_BASE + 0xCC) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
        mark32(DBG_MODE_BASE + 0x60, 0x5CA10E40);
        mark32(DBG_MODE_BASE + 0x64, req.channel_mask.0);
        let timeout_loops = Self::scan_duration_loops(req.scan_duration);
        let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();
        let mut energy_list: EdList = heapless::Vec::new();
        let mut iter_count: u32 = 0;
        let mut permit_joining_count: u32 = 0;
        let mut coordinator_beacon_count: u32 = 0;

        for channel in req.channel_mask.iter() {
            iter_count = iter_count.wrapping_add(1);
            mark32(DBG_MODE_BASE + 0x68, 0x5CA10100 | channel.number() as u32);
            let ch = channel.number();
            match req.scan_type {
                ScanType::Active => {
                    let found = self.active_scan_channel(ch, timeout_loops);
                    for desc in found {
                        if desc.superframe_spec.association_permit {
                            permit_joining_count = permit_joining_count.wrapping_add(1);
                        }
                        if desc.superframe_spec.pan_coordinator {
                            coordinator_beacon_count = coordinator_beacon_count.wrapping_add(1);
                        }
                        let _ = pan_descriptors.push(desc);
                    }
                }
                ScanType::Passive => {
                    let found = self.passive_scan_channel(ch, timeout_loops);
                    for desc in found {
                        if desc.superframe_spec.association_permit {
                            permit_joining_count = permit_joining_count.wrapping_add(1);
                        }
                        if desc.superframe_spec.pan_coordinator {
                            coordinator_beacon_count = coordinator_beacon_count.wrapping_add(1);
                        }
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
        mark32(DBG_MODE_BASE + 0x6C, 0x5CA10E00 | (iter_count & 0xFF));
        mark32(DBG_MODE_BASE + 0x70, pan_descriptors.len() as u32);
        mark32(DBG_MODE_BASE + 0x78, permit_joining_count);
        mark32(DBG_MODE_BASE + 0x7C, coordinator_beacon_count);
        // Sticky cumulative counters across all scans during device lifetime.
        let prev_total = STICKY_TOTAL_PANS.load(Ordering::Relaxed);
        STICKY_TOTAL_PANS.store(prev_total.wrapping_add(pan_descriptors.len() as u32), Ordering::Relaxed);
        let prev_permit = STICKY_PERMIT_PANS.load(Ordering::Relaxed);
        STICKY_PERMIT_PANS.store(prev_permit.wrapping_add(permit_joining_count), Ordering::Relaxed);
        mark32(DBG_MODE_BASE + 0x1C8, STICKY_PERMIT_PANS.load(Ordering::Relaxed));
        mark32(DBG_MODE_BASE + 0x1CC, STICKY_TOTAL_PANS.load(Ordering::Relaxed));
        if matches!(req.scan_type, ScanType::Active | ScanType::Passive) && pan_descriptors.is_empty() {
            mark32(DBG_MODE_BASE + 0x74, 0x5CA1F00D);
            Err(MacError::NoBeacon)
        } else {
            mark32(DBG_MODE_BASE + 0x74, 0x5CA10C0F);
            Ok(MlmeScanConfirm {
                scan_type: req.scan_type,
                pan_descriptors,
                energy_list,
            })
        }
    }

    /// Capture a frame that was received during the assoc-wait window but was
    /// NOT an Associate Response (so we silently dropped it). Used to verify
    /// the theory that EZSP's queued Transport-Key gets consumed and dropped
    /// here. Layout (JOIN_BASE-relative):
    ///   +0x160: u32 dropped-frame counter (incremented on every drop)
    ///   +0x164: first-drop u32 packed = (plen<<24) | (seq<<16) | fcf
    ///   +0x168..+0x178: first 16 bytes of first dropped frame (4 u32 words, LE)
    /// First-write-wins for +0x164.. so we always preserve the *first* drop.
    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        // Diag: count mlme_associate entries
        unsafe {
            let p = (DBG_JOIN_BASE + 0xC8) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
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

        // === JOIN-PATH DIAG (single writer per offset) ===
        let coord_mode: u32 = match req.coord_address {
            zigbee_types::MacAddress::Short(_, _) => 0x2,
            zigbee_types::MacAddress::Extended(_, _) => 0x3,
        };
        let cap_byte: u32 = req.capability_info.to_byte() as u32;
        let dsn = assoc.get(2).copied().unwrap_or(0) as u32;
        // Sentinel cleanly separated: high 16 bits = 0xA55C, low byte = channel.
        mark32(
            DBG_JOIN_BASE + 0x00,
            0xA55C_0000 | ((self.channel as u32) & 0xFF),
        );
        // Raw pan_id at +0x48 (u16 LE).
        mark32(DBG_JOIN_BASE + 0x48, self.pan_id.0 as u32);
        mark32(
            DBG_JOIN_BASE + 0x04,
            ((assoc.len() as u32 & 0xFF) << 24) | (dsn << 16) | (cap_byte << 8) | coord_mode,
        );
        let w0 = u32::from_le_bytes([
            assoc.first().copied().unwrap_or(0),
            assoc.get(1).copied().unwrap_or(0),
            assoc.get(2).copied().unwrap_or(0),
            assoc.get(3).copied().unwrap_or(0),
        ]);
        let w1 = u32::from_le_bytes([
            assoc.get(4).copied().unwrap_or(0),
            assoc.get(5).copied().unwrap_or(0),
            assoc.get(6).copied().unwrap_or(0),
            assoc.get(7).copied().unwrap_or(0),
        ]);
        mark32(DBG_JOIN_BASE + 0x08, w0);
        mark32(DBG_JOIN_BASE + 0x0C, w1);

        match self.csma_transmit(&assoc, true) {
            Ok(()) => mark32(DBG_MODE_BASE + 0x54, 0xA55C0001),
            Err(e) => {
                mark32(DBG_MODE_BASE + 0x54, 0xA55CFFFF);
                mark32(DBG_JOIN_BASE + 0x44, 0xDEAD_0001); // csma fail
                return Err(e);
            }
        }

        let mut dw_iters: u32 = 0;
        let mut dw_frames: u32 = 0;
        let mut dw_first_logged = false;
        for direct_attempt in 0..6u8 {
            dw_iters = direct_attempt as u32 + 1;
            mark32(DBG_JOIN_BASE + 0x1C, dw_iters); // +0x26C
            if let Ok(pkt) = self.receive_raw(POLL_RESPONSE_LOOPS / 3) {
                dw_frames = dw_frames.wrapping_add(1);
                mark32(DBG_JOIN_BASE + 0x20, dw_frames); // +0x270
                if !dw_first_logged {
                    dw_first_logged = true;
                    let data = &pkt.data[..pkt.len];
                    let fcf = u16::from_le_bytes([
                        data.first().copied().unwrap_or(0),
                        data.get(1).copied().unwrap_or(0),
                    ]) as u32;
                    let dsn0 = data.get(2).copied().unwrap_or(0) as u32;
                    mark32(
                        DBG_JOIN_BASE + 0x24, // +0x274
                        ((pkt.len as u32 & 0xFF) << 24) | (dsn0 << 16) | fcf,
                    );
                }
                mark32(DBG_MODE_BASE + 0x58, 0xA55C0100 | direct_attempt as u32);
                if let Some(confirm) = self.parse_assoc_response_packet(&pkt.data[..pkt.len]) {
                    mark32(
                        DBG_JOIN_BASE + 0x28, // +0x278
                        0xCAFE0000 | confirm.short_address.0 as u32,
                    );
                    mark32(
                        DBG_JOIN_BASE + 0x44,
                        0xC0DE_0000 | confirm.short_address.0 as u32,
                    );
                    // Stamp assoc-OK timestamp for first-poll-delay measurement.
                    let t = unsafe { core::ptr::read_volatile(0x0080_0630 as *const u32) };
                    ASSOC_OK_TICKS.store(t.max(1), Ordering::Relaxed);
                    FIRST_POLL_LOGGED.store(0, Ordering::Relaxed);
                    return Ok(confirm);
                } else {
                    // Not an Associate Response — capture and drop. EZSP may
                    // have queued the Transport-Key as an indirect transaction
                    // back-to-back with the Associate Response; if we drop it
                    // here, the HW already auto-ACKed and the coord will not
                    // retransmit. See Step-1 diagnosis in plan.md.
                    capture_dropped_assoc_frame(&pkt.data[..pkt.len]);
                    // Step 2: enqueue addressed-to-us frames so the BDB layer
                    // can pick them up via `mcps_data_indication` once
                    // associate returns.
                    self.maybe_enqueue_pending(&pkt.data[..pkt.len], pkt.lqi);
                }
            }
        }

        delay_ms(200);
        mark32(DBG_MODE_BASE + 0x58, 0xA55C0002);
        mark32(DBG_JOIN_BASE + 0x2C, 0xA55C200D); // +0x27C

        let mut dr_attempts: u32 = 0;
        let mut dr_tx_bitmap: u32 = 0;
        let mut dr_frames: u32 = 0;
        let mut dr_first_logged = false;
        for poll_attempt in 0..6u8 {
            dr_attempts = poll_attempt as u32 + 1;
            mark32(DBG_JOIN_BASE + 0x30, dr_attempts); // +0x280
            let data_req = self.build_data_request(&req.coord_address);
            // The parent may answer a poll with the Association Response before
            // a standalone MAC ACK is observed. Do not use csma_transmit() here:
            // it waits only for ACK frames and would discard the response.
            match self.transmit_raw(&data_req) {
                Ok(()) => {
                    dr_tx_bitmap |= 1u32 << poll_attempt;
                    mark32(DBG_JOIN_BASE + 0x34, dr_tx_bitmap); // +0x284
                    mark32(DBG_MODE_BASE + 0x5C, 0xA55C0003 | ((poll_attempt as u32) << 8));
                }
                Err(_) => {
                    mark32(DBG_JOIN_BASE + 0x34, dr_tx_bitmap);
                    mark32(DBG_MODE_BASE + 0x5C, 0xA55CFF00 | poll_attempt as u32);
                    delay_ms(250);
                    continue;
                }
            }

            for _ in 0..4 {
                if let Ok(pkt) = self.receive_raw(POLL_RESPONSE_LOOPS) {
                    dr_frames = dr_frames.wrapping_add(1);
                    mark32(DBG_JOIN_BASE + 0x38, dr_frames); // +0x288
                    if !dr_first_logged {
                        dr_first_logged = true;
                        let data = &pkt.data[..pkt.len];
                        let fcf = u16::from_le_bytes([
                            data.first().copied().unwrap_or(0),
                            data.get(1).copied().unwrap_or(0),
                        ]) as u32;
                        let dsn0 = data.get(2).copied().unwrap_or(0) as u32;
                        mark32(
                            DBG_JOIN_BASE + 0x3C, // +0x28C
                            ((pkt.len as u32 & 0xFF) << 24) | (dsn0 << 16) | fcf,
                        );
                    }
                    let data = &pkt.data[..pkt.len];
                    if let Some(confirm) = self.parse_assoc_response_packet(data) {
                        mark32(
                            DBG_JOIN_BASE + 0x40, // +0x290
                            0xCAFE0000 | confirm.short_address.0 as u32,
                        );
                        mark32(
                            DBG_JOIN_BASE + 0x44,
                            0xC0DE_0000 | confirm.short_address.0 as u32,
                        );
                        // Stamp assoc-OK timestamp (data-req window).
                        let t = unsafe { core::ptr::read_volatile(0x0080_0630 as *const u32) };
                        ASSOC_OK_TICKS.store(t.max(1), Ordering::Relaxed);
                        FIRST_POLL_LOGGED.store(0, Ordering::Relaxed);
                        return Ok(confirm);
                    } else {
                        // See note in direct-window loop above.
                        capture_dropped_assoc_frame(data);
                        // Step 2: enqueue addressed-to-us frames.
                        self.maybe_enqueue_pending(data, pkt.lqi);
                    }
                }
            }
            delay_ms(250);
        }

        mark32(DBG_MODE_BASE + 0x60, 0xA55CFFFF);
        mark32(DBG_JOIN_BASE + 0x44, 0xDEAD_0002); // exhausted both windows
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
        // Diag: count mlme_reset (BDB calls this between candidate attempts)
        unsafe {
            let p = (DBG_JOIN_BASE + 0xC0) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
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
        // Diag: count mlme_set calls (BDB calls this many times during steering)
        unsafe {
            let p = (DBG_JOIN_BASE + 0xC4) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
        use zigbee_mac::PibAttribute::*;
        use zigbee_mac::PibValue;

        match (attr, val) {
            (MacShortAddress, PibValue::ShortAddress(v)) => {
                self.short_address = v;
                update_mac_ack_filter(self.pan_id, self.short_address);
            }
            (MacPanId, PibValue::PanId(v)) => {
                self.pan_id = v;
                update_mac_ack_filter(self.pan_id, self.short_address);
            }
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
        // Diag: count BDB-side calls into mlme_poll
        unsafe {
            let p = (DBG_MODE_BASE + 0xC0) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
        let parent = zigbee_types::MacAddress::Short(self.pan_id, self.coord_short_address);
        let data_req = self.build_data_request(&parent);

        // ── Parent identity capture (LAST-write-wins) ──
        // MODE+0x1D0: parent NWK short (the address we send Data Requests to).
        //             Expect this to match the parent_address learned by NWK
        //             during nlme_join (BDB+0xAC, frame-ring src). If it shows
        //             0x0000 then mlme_associate was passed the coord, not the
        //             router that actually accepted us — and the TC's TK relay
        //             will never reach us.
        // MODE+0x1D4: parent IEEE low  word (coord_extended_address[0..4] LE)
        // MODE+0x1D8: parent IEEE high word (coord_extended_address[4..8] LE)
        unsafe {
            core::ptr::write_volatile(
                (DBG_MODE_BASE + 0x1D0) as *mut u32,
                self.coord_short_address.0 as u32,
            );
            let e = &self.coord_extended_address;
            let lo = u32::from_le_bytes([e[0], e[1], e[2], e[3]]);
            let hi = u32::from_le_bytes([e[4], e[5], e[6], e[7]]);
            core::ptr::write_volatile((DBG_MODE_BASE + 0x1D4) as *mut u32, lo);
            core::ptr::write_volatile((DBG_MODE_BASE + 0x1D8) as *mut u32, hi);
        }

        // ── Polling / ACK frame_pending instrumentation ──
        // MODE+0x1B4: Data Request TX attempts (counted before csma_transmit)
        // MODE+0x1B8: ACK observed with frame_pending=1 (parent has data for us)
        // MODE+0x1BC: ACK observed with frame_pending=0 (parent has nothing)
        // BDB+0xFC : (assoc-OK → first poll TX) delay, in milliseconds
        unsafe {
            let p = (DBG_MODE_BASE + 0x1B4) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
        if FIRST_POLL_LOGGED.load(Ordering::Relaxed) == 0 {
            let assoc_t = ASSOC_OK_TICKS.load(Ordering::Relaxed);
            if assoc_t != 0 {
                let now = unsafe { core::ptr::read_volatile(0x0080_0630 as *const u32) };
                let delta_ticks = now.wrapping_sub(assoc_t);
                // 24 MHz tick → ms; cap to u32 max.
                let delta_ms = delta_ticks / 24_000;
                unsafe {
                    core::ptr::write_volatile((DBG_BDB_BASE + 0xFC) as *mut u32, delta_ms);
                }
                FIRST_POLL_LOGGED.store(1, Ordering::Relaxed);
            }
        }

        // Reset the ACK-fp sentinel so we only credit fp counters for an ACK
        // observed during *this* csma call (not a stale one from before).
        LAST_ACK_FP.store(0xFFFF_FFFF, Ordering::Relaxed);
        let tx_res = self.csma_transmit(&data_req, true);
        let fp = LAST_ACK_FP.load(Ordering::Relaxed);
        if fp == 1 {
            unsafe {
                let p = (DBG_MODE_BASE + 0x1B8) as *mut u32;
                core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            }
        } else if fp == 0 {
            unsafe {
                let p = (DBG_MODE_BASE + 0x1BC) as *mut u32;
                core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            }
        }
        tx_res?;

        let mut frames_in_loop: u32 = 0;
        let mut filtered_out: u32 = 0;
        for _ in 0..8 {
            match self.receive_raw(POLL_RESPONSE_LOOPS / 8) {
                Ok(pkt) => {
                let data = &pkt.data[..pkt.len];
                frames_in_loop = frames_in_loop.wrapping_add(1);
                if data.len() < 5 {
                    filtered_out = filtered_out.wrapping_add(1);
                    continue;
                }
                let fc = u16::from_le_bytes([data[0], data[1]]);
                if fc & 0x07 != 1 {
                    filtered_out = filtered_out.wrapping_add(1);
                    continue;
                }
                let Some(dst) = parse_dest_address(data, fc) else {
                    filtered_out = filtered_out.wrapping_add(1);
                    continue;
                };
                if !address_matches(
                    &dst,
                    self.pan_id,
                    self.short_address,
                    self.extended_address,
                ) {
                    filtered_out = filtered_out.wrapping_add(1);
                    continue;
                }
                let header_len = 3 + addressing_size(fc);
                if data.len() <= header_len {
                    filtered_out = filtered_out.wrapping_add(1);
                    continue;
                }
                    // Diag: capture this frame BEFORE returning. This is the
                    // payload BDB will see (after the MAC header is stripped).
                    let payload = &data[header_len..];
                    let plen = payload.len() as u32;
                    let p0 = payload.first().copied().unwrap_or(0) as u32;
                    let p1 = payload.get(1).copied().unwrap_or(0) as u32;
                    let p2 = payload.get(2).copied().unwrap_or(0) as u32;
                    let p3 = payload.get(3).copied().unwrap_or(0) as u32;
                    let p4 = payload.get(4).copied().unwrap_or(0) as u32;
                    let p5 = payload.get(5).copied().unwrap_or(0) as u32;
                    let p6 = payload.get(6).copied().unwrap_or(0) as u32;
                    let p7 = payload.get(7).copied().unwrap_or(0) as u32;
                    unsafe {
                        let cnt_p = (DBG_JOIN_BASE + 0x60) as *mut u32;
                        let cnt = core::ptr::read_volatile(cnt_p).wrapping_add(1);
                        core::ptr::write_volatile(cnt_p, cnt);
                        // First-frame slot only if not yet set.
                        let first_p = (DBG_JOIN_BASE + 0x68) as *mut u32;
                        if core::ptr::read_volatile(first_p) == 0 {
                            core::ptr::write_volatile(first_p, (plen << 24) | (fc as u32));
                            core::ptr::write_volatile(
                                (DBG_JOIN_BASE + 0x6C) as *mut u32,
                                p0 | (p1 << 8) | (p2 << 16) | (p3 << 24),
                            );
                            core::ptr::write_volatile(
                                (DBG_JOIN_BASE + 0x70) as *mut u32,
                                p4 | (p5 << 8) | (p6 << 16) | (p7 << 24),
                            );
                        }
                    }
                    // Always update "last frame" slot.
                    mark32(DBG_JOIN_BASE + 0x74, (plen << 24) | (fc as u32));
                    mark32(
                        DBG_JOIN_BASE + 0x78,
                        p0 | (p1 << 8) | (p2 << 16) | (p3 << 24),
                    );
                    mark32(
                        DBG_JOIN_BASE + 0x7C,
                        p4 | (p5 << 8) | (p6 << 16) | (p7 << 24),
                    );

                    // === NWK-dst-filter capture ===
                    // The previous dump showed only one frame whose NWK
                    // dst_short matched our short. To decode that we need
                    // more bytes than (p0..p7). Capture the FIRST such
                    // frame's first 32 bytes of NWK payload at +0xD0..+0xF8,
                    // and bump counters for "to-us" and "broadcast" (0xFFFC..)
                    // NWK dst routing buckets.
                    if payload.len() >= 4 {
                        let nwk_dst =
                            u16::from_le_bytes([payload[2], payload[3]]);
                        let our_short = self.short_address.0;
                        unsafe {
                            if nwk_dst == our_short && our_short != 0xFFFF {
                                let to_us_p =
                                    (DBG_JOIN_BASE + 0xD0) as *mut u32;
                                core::ptr::write_volatile(
                                    to_us_p,
                                    core::ptr::read_volatile(to_us_p)
                                        .wrapping_add(1),
                                );
                                let first_to_us =
                                    (DBG_JOIN_BASE + 0xD8) as *mut u32;
                                if core::ptr::read_volatile(first_to_us) == 0 {
                                    core::ptr::write_volatile(
                                        first_to_us,
                                        (plen << 24) | (fc as u32),
                                    );
                                    // Dump first 32 bytes of payload
                                    for i in 0..8usize {
                                        let off = i * 4;
                                        let b0 = payload
                                            .get(off)
                                            .copied()
                                            .unwrap_or(0)
                                            as u32;
                                        let b1 = payload
                                            .get(off + 1)
                                            .copied()
                                            .unwrap_or(0)
                                            as u32;
                                        let b2 = payload
                                            .get(off + 2)
                                            .copied()
                                            .unwrap_or(0)
                                            as u32;
                                        let b3 = payload
                                            .get(off + 3)
                                            .copied()
                                            .unwrap_or(0)
                                            as u32;
                                        let word = b0
                                            | (b1 << 8)
                                            | (b2 << 16)
                                            | (b3 << 24);
                                        let p_word = (DBG_JOIN_BASE
                                            + 0xDC
                                            + (i as u32 * 4))
                                            as *mut u32;
                                        core::ptr::write_volatile(p_word, word);
                                    }
                                }
                            } else if nwk_dst >= 0xFFFC {
                                let bcast_p =
                                    (DBG_JOIN_BASE + 0xD4) as *mut u32;
                                core::ptr::write_volatile(
                                    bcast_p,
                                    core::ptr::read_volatile(bcast_p)
                                        .wrapping_add(1),
                                );
                            }
                        }
                    }
                    return Ok(MacFrame::from_slice(&data[header_len..]));
                }
                Err(MacError::NoData) => {}
                Err(e) => return Err(e),
            }
        }
        // No frame returned this call; bump None counter and record loop stats.
        unsafe {
            let none_p = (DBG_JOIN_BASE + 0x64) as *mut u32;
            core::ptr::write_volatile(
                none_p,
                core::ptr::read_volatile(none_p).wrapping_add(1),
            );
            // Last-poll-stats: filtered_out count
            mark32(DBG_JOIN_BASE + 0xA8, (frames_in_loop << 16) | (filtered_out & 0xFFFF));
        }
        Ok(None)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        // Diag: count BDB-side calls into mcps_data (TX path used by NWK announce, etc.)
        unsafe {
            let p = (DBG_JOIN_BASE + 0xB0) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            // Capture destination short/ext low bytes
            let dst_lo: u32 = match &req.dst_address {
                zigbee_types::MacAddress::Short(_, s) => s.0 as u32,
                zigbee_types::MacAddress::Extended(_, e) => {
                    (e[0] as u32) | ((e[1] as u32) << 8) | ((e[2] as u32) << 16) | ((e[3] as u32) << 24)
                }
            };
            mark32(DBG_JOIN_BASE + 0xB4, dst_lo);
            // Capture payload first 4 bytes
            let p0 = req.payload.first().copied().unwrap_or(0) as u32;
            let p1 = req.payload.get(1).copied().unwrap_or(0) as u32;
            let p2 = req.payload.get(2).copied().unwrap_or(0) as u32;
            let p3 = req.payload.get(3).copied().unwrap_or(0) as u32;
            mark32(DBG_JOIN_BASE + 0xB8, p0 | (p1 << 8) | (p2 << 16) | (p3 << 24));
        }
        let frame = self.build_data_frame(&req.dst_address, req.payload, req.tx_options.ack_tx)?;
        self.csma_transmit(&frame, req.tx_options.ack_tx)?;
        // Success counter
        unsafe {
            let p = (DBG_JOIN_BASE + 0xBC) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        self.mcps_data_indication_timeout(5_000_000).await
    }

    async fn mcps_data_indication_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<McpsDataIndication, MacError> {
        // Diag: count BDB-side calls into mcps_data_indication
        unsafe {
            let p = (DBG_MODE_BASE + 0xC4) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
        // Stack high-water scan: starting at _svc_stack_bottom, walk up to
        // find the first word that is no longer the paint sentinel. That's
        // the deepest the stack has been since boot (or since last paint).
        // We record (svc_top - first_non_paint) = bytes of stack used.
        // Result lives at MODE+0x1AC. Only update if we see a NEW high water.
        unsafe {
            const SVC_STACK_BOTTOM: u32 = 0x0084_B400;
            const SVC_STACK_TOP: u32 = 0x0084_E000;
            const STACK_PAINT_PATTERN: u32 = 0xDEAD_BEEF;
            let mut a = SVC_STACK_BOTTOM;
            while a < SVC_STACK_TOP {
                if core::ptr::read_volatile(a as *const u32) != STACK_PAINT_PATTERN {
                    break;
                }
                a = a.wrapping_add(4);
            }
            let used = SVC_STACK_TOP.saturating_sub(a);
            let p = (DBG_MODE_BASE + 0x1AC) as *mut u32;
            let prev = core::ptr::read_volatile(p);
            if used > prev {
                core::ptr::write_volatile(p, used);
            }
        }
        let started = self.monotonic_micros();
        loop {
            // Step 2: drain any frame queued during `mlme_associate` first.
            // Diagnostics: +0x184 = drain count.
            let mut buf: heapless::Vec<u8, MAX_MAC_FRAME_LEN> = heapless::Vec::new();
            let lqi: u8;
            if let Some(pending) = self.pending_rx.pop_front() {
                unsafe {
                    let p = (DBG_JOIN_BASE + 0x184) as *mut u32;
                    core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
                }
                let _ = buf.extend_from_slice(&pending.data);
                lqi = pending.lqi;
            } else {
                if self.monotonic_micros().wrapping_sub(started) >= timeout_us {
                    unsafe {
                        let p = (DBG_JOIN_BASE + 0xA4) as *mut u32;
                        core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
                    }
                    return Err(MacError::NoData);
                }
                let pkt = match self.receive_raw(1) {
                    Ok(pkt) => pkt,
                    Err(MacError::NoData) => continue,
                    Err(error) => return Err(error),
                };
                let _ = buf.extend_from_slice(&pkt.data[..pkt.len]);
                lqi = pkt.lqi;
            }
            let data = &buf[..];
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
            // ── MAC-layer frame capture (BDB+0xD8..+0xEF) ──
            // Record every frame with valid src+dst into a 4-entry ring so we
            // can see what actually arrives during the join window, regardless
            // of whether it later passes the address filter or NWK decrypt.
            // Slot layout: u32 = fc_u16 | (src_short_u16 << 16).
            // Counters:
            //   BDB+0xD8: NWK-unsecured frame count (frame_type=1 AND NWK sec=0)
            //   BDB+0xDC: unicasts addressed to our short address
            {
                let src_short = match src {
                    zigbee_types::MacAddress::Short(_, s) => s.0,
                    zigbee_types::MacAddress::Extended(_, _) => 0xFFFEu16,
                };
                let slot = FRAME_RING_IDX.load(Ordering::Relaxed) & 0x3;
                FRAME_RING_IDX.store(slot.wrapping_add(1), Ordering::Relaxed);
                let val = (fc as u32) | ((src_short as u32) << 16);
                unsafe {
                    let p = (DBG_BDB_BASE + 0xE0 + slot * 4) as *mut u32;
                    core::ptr::write_volatile(p, val);
                }
                // Unicast-to-us check (BDB+0xDC).
                let our_short = self.short_address.0;
                let is_unicast_to_us = matches!(
                    dst,
                    zigbee_types::MacAddress::Short(_, s) if s.0 == our_short && our_short != 0xFFFF && our_short != 0xFFFE
                );
                if is_unicast_to_us {
                    unsafe {
                        let p = (DBG_BDB_BASE + 0xDC) as *mut u32;
                        core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
                    }
                }
            }
            if !self.promiscuous && !address_matches(&dst, self.pan_id, self.short_address, self.extended_address) {
                continue;
            }
            if frame_type != 1 {
                continue;
            }
            let header_len = 3 + addressing_size(fc);
            if data.len() <= header_len {
                continue;
            }
            // BDB+0xD8: NWK-unsecured count. NWK FCF byte 1 bit 1 = security.
            // (Zigbee NWK FCF: u16 LE; security flag at bit 9 → byte 1 bit 1.)
            {
                let mp = &data[header_len..];
                if mp.len() >= 2 {
                    let nwk_sec = (mp[1] >> 1) & 0x1;
                    if nwk_sec == 0 {
                        unsafe {
                            let p = (DBG_BDB_BASE + 0xD8) as *mut u32;
                            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
                        }
                    }
                }
            }
            if let Some(payload) = MacFrame::from_slice(&data[header_len..]) {
                // Diag: capture this frame's payload before returning.
                let plen = payload.len() as u32;
                let pp = payload.as_slice();
                let p0 = pp.first().copied().unwrap_or(0) as u32;
                let p1 = pp.get(1).copied().unwrap_or(0) as u32;
                let p2 = pp.get(2).copied().unwrap_or(0) as u32;
                let p3 = pp.get(3).copied().unwrap_or(0) as u32;
                let p4 = pp.get(4).copied().unwrap_or(0) as u32;
                let p5 = pp.get(5).copied().unwrap_or(0) as u32;
                let p6 = pp.get(6).copied().unwrap_or(0) as u32;
                let p7 = pp.get(7).copied().unwrap_or(0) as u32;
                unsafe {
                    let cnt_p = (DBG_JOIN_BASE + 0x80) as *mut u32;
                    core::ptr::write_volatile(
                        cnt_p,
                        core::ptr::read_volatile(cnt_p).wrapping_add(1),
                    );
                    let first_p = (DBG_JOIN_BASE + 0x84) as *mut u32;
                    if core::ptr::read_volatile(first_p) == 0 {
                        core::ptr::write_volatile(first_p, (plen << 24) | (fc as u32));
                        core::ptr::write_volatile(
                            (DBG_JOIN_BASE + 0x88) as *mut u32,
                            p0 | (p1 << 8) | (p2 << 16) | (p3 << 24),
                        );
                        core::ptr::write_volatile(
                            (DBG_JOIN_BASE + 0x8C) as *mut u32,
                            p4 | (p5 << 8) | (p6 << 16) | (p7 << 24),
                        );
                    }
                }
                mark32(DBG_JOIN_BASE + 0x90, (plen << 24) | (fc as u32));
                mark32(
                    DBG_JOIN_BASE + 0x94,
                    p0 | (p1 << 8) | (p2 << 16) | (p3 << 24),
                );
                mark32(
                    DBG_JOIN_BASE + 0x98,
                    p4 | (p5 << 8) | (p6 << 16) | (p7 << 24),
                );
                // MODE+0x20C: frames returned to BDB from mcps_data_indication.
                unsafe {
                    let p = (DBG_MODE_BASE + 0x20C) as *mut u32;
                    core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
                }
                return Ok(McpsDataIndication {
                    src_address: src,
                    dst_address: dst,
                    lqi,
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

impl PlatformServices for Tlsr8258Mac {
    fn monotonic_micros(&self) -> u32 {
        (self.extended_timer_ticks() / 24) as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        delay_ms(duration_us.saturating_add(999) / 1_000);
    }

    fn fill_random(&mut self, _output: &mut [u8]) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
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

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn transmit_mac_ack(seq: u8, frame_pending: bool, rx_buf: *mut u8) {
    let fc_low = if frame_pending { 0x12u8 } else { 0x02u8 };
    let ack = [fc_low, 0x00, seq];
    write_tx_dma_frame(&ack);
    let tx_buf = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;

    radio::disable_dma_rx();
    radio::disable_rx_mode();
    radio::set_tx_dma_config(144);
    radio::tx_done_clear();
    radio::set_tx_mode();

    radio::rx_buf_clear(rx_buf);
    radio::set_rx_buffer(rx_buf);
    radio::enable_dma_rx();

    // Telink SDK waits ZB_TX_WAIT_US (120 us) from RX->TX switch before ACK TX.
    spin_delay(2_900);
    let irq_mask = radio::mask_tx_irq();
    radio::tx_pkt(tx_buf);
    for _ in 0..24_000u32 {
        if radio::tx_done() {
            radio::tx_done_clear();
            radio::restore_irq_mask(irq_mask);
            radio::set_rx_mode();
            mark32(DBG_MODE_BASE + 0x48, 0xAACC0000 | seq as u32);
            return;
        }
        unsafe { core::arch::asm!("nop"); }
    }
    radio::tx_done_clear();
    radio::restore_irq_mask(irq_mask);
    radio::set_rx_mode();
    mark32(DBG_MODE_BASE + 0x48, 0xAACCFF00 | seq as u32);
}

#[inline(always)]
fn rx_frame_needs_ack(
    rx_buf: *const u8,
    frame_len: usize,
    pan_id: zigbee_types::PanId,
    short_address: zigbee_types::ShortAddress,
    extended_address: zigbee_types::IeeeAddress,
) -> Option<u8> {
    if frame_len < 5 {
        return None;
    }

    let fc = unsafe {
        u16::from_le_bytes([
            core::ptr::read_volatile(rx_buf.add(5)),
            core::ptr::read_volatile(rx_buf.add(6)),
        ])
    };
    if (fc & 0x0020) == 0 {
        return None;
    }

    let dst_mode = (fc >> 10) & 0x03;
    if dst_mode < 2 {
        return None;
    }

    let dst_pan = unsafe {
        u16::from_le_bytes([
            core::ptr::read_volatile(rx_buf.add(8)),
            core::ptr::read_volatile(rx_buf.add(9)),
        ])
    };
    if dst_pan != pan_id.0 && dst_pan != 0xFFFF {
        return None;
    }

    let matched = match dst_mode {
        0x02 if frame_len >= 7 => {
            let dst_short = unsafe {
                u16::from_le_bytes([
                    core::ptr::read_volatile(rx_buf.add(10)),
                    core::ptr::read_volatile(rx_buf.add(11)),
                ])
            };
            dst_short == short_address.0 || dst_short == 0xFFFF
        }
        0x03 if frame_len >= 13 => {
            let mut ok = true;
            for i in 0..8 {
                let b = unsafe { core::ptr::read_volatile(rx_buf.add(10 + i)) };
                ok &= b == extended_address[i];
            }
            ok
        }
        _ => false,
    };

    if matched {
        Some(unsafe { core::ptr::read_volatile(rx_buf.add(7)) })
    } else {
        None
    }
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn update_mac_ack_filter(pan_id: zigbee_types::PanId, short_address: zigbee_types::ShortAddress) {
    let packed = (pan_id.0 as u32) | ((short_address.0 as u32) << 16);
    MAC_ACK_FILTER.store(packed, core::sync::atomic::Ordering::Release);
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn handle_rf_rx_irq() {
    if !radio::rx_done() {
        return;
    }

    let rx_buf = core::ptr::addr_of_mut!(RF_RX_BUF) as *mut u8;
    let total_len = unsafe { core::ptr::read_volatile(rx_buf) };
    let payload_len = unsafe { core::ptr::read_volatile(rx_buf.add(4)) };
    mark32(
        DBG_MODE_BASE + 0x4C,
        0x1A000000 | (total_len as u32) | ((payload_len as u32) << 8),
    );

    // Diag: total frames entering IRQ handler (any rx_done event with any data)
    let irq_total = unsafe {
        let p = (DBG_MODE_BASE + 0xA0) as *mut u32;
        let v = core::ptr::read_volatile(p).wrapping_add(1);
        core::ptr::write_volatile(p, v);
        v
    };
    let _ = irq_total;

    // Diag: capture last status byte and FCF+DSN for *every* IRQ frame
    if total_len > 0 && total_len <= 130 {
        let status_byte =
            unsafe { core::ptr::read_volatile(rx_buf.add(total_len as usize + 3)) };
        let fcf_dsn = unsafe { core::ptr::read_volatile(rx_buf.add(5) as *const u32) };
        mark32(DBG_MODE_BASE + 0xC8, 0xCB000000 | status_byte as u32);
        mark32(DBG_MODE_BASE + 0xCC, fcf_dsn);
    }

    let len_ok = total_len > 0 && radio::packet_length_ok(rx_buf);
    let crc_ok = len_ok && radio::packet_crc_ok(rx_buf);
    if len_ok && !crc_ok {
        unsafe {
            let p = (DBG_MODE_BASE + 0xA8) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
    }
    if crc_ok {
        unsafe {
            let p = (DBG_MODE_BASE + 0xA4) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
    }

    if total_len > 0 && radio::packet_length_ok(rx_buf) && radio::packet_crc_ok(rx_buf) {
        let phy_len = radio::payload_len(rx_buf) as usize;
        if (2..=MAX_MAC_FRAME_LEN + 2).contains(&phy_len) {
            let frame_len = phy_len - 2;
            let filter = MAC_ACK_FILTER.load(core::sync::atomic::Ordering::Acquire);
            let pan_id = zigbee_types::PanId(filter as u16);
            let short_address = zigbee_types::ShortAddress((filter >> 16) as u16);
            let ext_addr = our_ext_addr();
            if let Some(seq) = rx_frame_needs_ack(
                rx_buf,
                frame_len,
                pan_id,
                short_address,
                ext_addr,
            ) {
                transmit_mac_ack(seq, false, rx_buf);
            }

            let rssi = radio::packet_rssi(rx_buf);
            let lqi = rssi_to_lqi(rssi);
            unsafe {
                let dst = core::ptr::addr_of_mut!(IRQ_RX_DATA) as *mut u8;
                core::ptr::copy_nonoverlapping(rx_buf.add(5), dst, frame_len);
                IRQ_RX_LEN = frame_len;
                IRQ_RX_RSSI = rssi;
                IRQ_RX_LQI = lqi;
            }
            // Publish: the Release store pairs with the Acquire load in
            // take_irq_rx_packet() so the data/len/rssi/lqi writes above are
            // visible before the consumer observes PENDING == 1.
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            IRQ_RX_PENDING.store(1, core::sync::atomic::Ordering::Release);
        }
    }

    radio::rx_done_clear();
}

fn take_irq_rx_packet() -> Option<RxPacket> {
    let irq_en = 0x800643 as *mut u8;
    let restore = unsafe { core::ptr::read_volatile(irq_en) };
    mark32(DBG_MODE_BASE + 0xEC, 0x7A1E0000 | restore as u32);
    unsafe { core::ptr::write_volatile(irq_en, 0) };
    // Fence: prevent the compiler from hoisting payload reads above the IRQ
    // disable. The volatile write itself is not enough to order non-volatile
    // accesses in LLVM's memory model.
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    let packet = if IRQ_RX_PENDING.load(core::sync::atomic::Ordering::Acquire) == 0 {
        None
    } else {
        let len = unsafe { IRQ_RX_LEN };
        if len <= MAX_MAC_FRAME_LEN {
            let mut data = [0u8; MAX_MAC_FRAME_LEN];
            unsafe {
                let src = core::ptr::addr_of!(IRQ_RX_DATA) as *const u8;
                core::ptr::copy_nonoverlapping(src, data.as_mut_ptr(), len);
            }
            let rssi = unsafe { IRQ_RX_RSSI };
            let lqi = unsafe { IRQ_RX_LQI };
            IRQ_RX_PENDING.store(0, core::sync::atomic::Ordering::Release);
            Some(RxPacket {
                data,
                len,
                rssi,
                lqi,
            })
        } else {
            IRQ_RX_PENDING.store(0, core::sync::atomic::Ordering::Release);
            None
        }
    };

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    unsafe { core::ptr::write_volatile(irq_en, restore) };
    mark32(DBG_MODE_BASE + 0xF0, 0x7A1F0000 | restore as u32);
    packet
}

fn receive_packet(
    rx_buf: *mut u8,
    timeout_loops: u32,
    pan_id: zigbee_types::PanId,
    short_address: zigbee_types::ShortAddress,
    extended_address: zigbee_types::IeeeAddress,
) -> Option<RxPacket> {
    if let Some(packet) = take_irq_rx_packet() {
        return Some(packet);
    }

    radio::set_trx_off();
    radio::rx_done_clear();
    radio::rx_buf_clear(rx_buf);
    radio::set_rx_buffer(rx_buf);
    radio::enable_dma_rx();
    radio::set_rx_mode();
    spin_delay(2400);

    for _ in 0..timeout_loops {
        if let Some(packet) = take_irq_rx_packet() {
            return Some(packet);
        }
        if radio::rx_done() {
            radio::disable_dma_rx();
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
                    if let Some(seq) = rx_frame_needs_ack(
                        rx_buf,
                        frame_len,
                        pan_id,
                        short_address,
                        extended_address,
                    ) {
                        transmit_mac_ack(seq, false, rx_buf);
                    } else {
                        radio::rx_done_clear();
                    }
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
            radio::rx_done_clear();
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

/// Classify every frame returned by `receive_raw` BEFORE any filter is
/// applied. Records into the BDB+0x100..+0x134 region:
///   +0x100..+0x11F : 4-entry ring × 8 bytes
///   +0x120 : dst_short == our_short  (unicast to us)
///   +0x124 : dst_short >= 0xFFFC     (NWK/MAC broadcast)
///   +0x128 : anything else
///   +0x12C : APS frame with apsFC == 0x21 (TK candidate)
///   +0x130 : NWK sec=0 AND dst_short == us (the holy grail)
///   +0x134 : raw receive_raw return count (pre-filter)
fn classify_rx_frame(data: &[u8], our_short: u16) -> bool {
    let mut is_tk_candidate = false;
    unsafe {
        let p = (DBG_BDB_BASE + 0x134) as *mut u32;
        core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
    }
    if data.len() < 7 {
        return is_tk_candidate;
    }
    let fc = u16::from_le_bytes([data[0], data[1]]);
    let frame_type = fc & 0x07;
    let dst_mode = (fc >> 10) & 0x03;
    // Only data/cmd frames with short destination — that's everything BDB
    // cares about. ACK/beacon just bump the raw counter and exit.
    if (frame_type != 1 && frame_type != 3) || dst_mode != 0x02 {
        return is_tk_candidate;
    }
    let seq = data[2];
    let dst_pan_hi = data[4];
    let dst_short = u16::from_le_bytes([data[5], data[6]]);
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;
    let src_short = if src_mode == 0x02 {
        let off = if pan_compress { 7 } else { 9 };
        if data.len() >= off + 2 {
            u16::from_le_bytes([data[off], data[off + 1]])
        } else {
            0xFFFE
        }
    } else {
        0xFFFE
    };

    let slot = RX_RAW_RING_IDX.load(Ordering::Relaxed) & 0x3;
    RX_RAW_RING_IDX.store(slot.wrapping_add(1), Ordering::Relaxed);
    let word_a = (data[0] as u32)
        | ((data[1] as u32) << 8)
        | ((seq as u32) << 16)
        | ((dst_pan_hi as u32) << 24);
    let word_b = (src_short as u32) | ((dst_short as u32) << 16);
    unsafe {
        let base = DBG_BDB_BASE + 0x100 + slot * 8;
        core::ptr::write_volatile(base as *mut u32, word_a);
        core::ptr::write_volatile((base + 4) as *mut u32, word_b);
    }

    let our_known = our_short != 0xFFFF && our_short != 0xFFFE;
    let is_us = our_known && dst_short == our_short;
    let is_bcast = dst_short >= 0xFFFC;
    unsafe {
        let off = if is_us {
            0x120
        } else if is_bcast {
            0x124
        } else {
            0x128
        };
        let p = (DBG_BDB_BASE + off) as *mut u32;
        core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
    }

    if frame_type != 1 {
        return is_tk_candidate;
    }
    let header_len = 3 + addressing_size(fc);
    if data.len() <= header_len + 1 {
        return is_tk_candidate;
    }
    let nwk = &data[header_len..];
    if nwk.len() < 2 {
        return is_tk_candidate;
    }
    let nwk_fcf = u16::from_le_bytes([nwk[0], nwk[1]]);
    let nwk_sec = (nwk_fcf >> 9) & 0x1;
    if nwk_sec == 0 && is_us {
        unsafe {
            let p = (DBG_BDB_BASE + 0x130) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            // Twin write to MODE region in the SAME unsafe block. If this
            // ever differs from BDB+0x130, we have memory-map confusion.
            let q = (DBG_MODE_BASE + 0x230) as *mut u32;
            core::ptr::write_volatile(q, core::ptr::read_volatile(q).wrapping_add(1));
        }
        // Treat ALL sec=0 unicast data frames to us as TK candidates and
        // force-enqueue. The actual APS classification is unreliable at this
        // layer because Sonoff EZSP wraps the TK in Tunnel commands with
        // varying NWK header layouts (src_ieee included → larger nhl).
        is_tk_candidate = true;
        // Capture first 24 bytes of the NWK payload for off-board analysis.
        // MODE+0x210 : len (low byte) | nwk_fcf (next 16 bits)
        // MODE+0x214..+0x22F : raw nwk bytes 0..27 (7 words)
        unsafe {
            let n = core::cmp::min(nwk.len(), 28);
            let hdr = (n as u32) | ((nwk_fcf as u32) << 8);
            let p = (DBG_MODE_BASE + 0x210) as *mut u32;
            core::ptr::write_volatile(p, hdr);
            for i in 0..7usize {
                let off = i * 4;
                let mut w: u32 = 0;
                for b in 0..4 {
                    if off + b < n {
                        w |= (nwk[off + b] as u32) << (b * 8);
                    }
                }
                let q = (DBG_MODE_BASE + 0x214 + (i as u32) * 4) as *mut u32;
                core::ptr::write_volatile(q, w);
            }
        }
    }

    // Best-effort NWK header walk → start of APS frame.
    // Base: FCF(2) dst(2) src(2) radius(1) seq(1) = 8B.
    let mut nhl: usize = 8;
    let mcast = (nwk_fcf >> 8) & 0x1;
    let src_route = (nwk_fcf >> 10) & 0x1;
    let dst_ieee = (nwk_fcf >> 11) & 0x1;
    let src_ieee = (nwk_fcf >> 12) & 0x1;
    if dst_ieee == 1 {
        nhl += 8;
    }
    if src_ieee == 1 {
        nhl += 8;
    }
    if mcast == 1 {
        nhl += 1;
    }
    if src_route == 1 && nwk.len() > nhl {
        let n = nwk[nhl] as usize;
        nhl += 2 + n * 2;
    }
    if nwk_sec == 1 {
        // NWK auxiliary security header: 14B for std nonce.
        nhl += 14;
    }
    if nwk.len() > nhl {
        let aps_fc = nwk[nhl];
        if aps_fc == 0x21 {
            unsafe {
                let p = (DBG_BDB_BASE + 0x12C) as *mut u32;
                core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            }
        }
    }
    is_tk_candidate
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
        mark32(DBG_MODE_BASE + 0xF4, 0xC5170000 | prev as u32);
        core::ptr::write_volatile(irq_en, 0);
        prev
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _critical_section_1_0_release(restore: u8) {
    mark32(DBG_MODE_BASE + 0xF8, 0xC5180000 | restore as u32);
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
    // Timer0 at 0x800630, runs at system clock (24MHz RC) = 24 ticks/µs
    const REG_TIMER0_TICK: *const u32 = 0x800630 as *const u32;

    pub fn on_alarm_irq() {
        super::TIMER0_ALARM_PENDING
            .store(true, core::sync::atomic::Ordering::Release);
        // Single-writer increment: tc32 has no atomic CAS, but the Timer0
        // ISR cannot re-enter itself, so a load+store is race-free against
        // other ISR instances. The main thread only reads this counter.
        let prev = super::TIMER0_IRQ_COUNT
            .load(core::sync::atomic::Ordering::Relaxed);
        super::TIMER0_IRQ_COUNT
            .store(prev.wrapping_add(1), core::sync::atomic::Ordering::Release);
        // Mask Timer0 IRQ so subsequent unrelated IRQs do not retrigger it.
        unsafe {
            let mask = core::ptr::read_volatile(0x800640 as *const u32);
            core::ptr::write_volatile(0x800640 as *mut u32, mask & !(1 << 0));
        }
    }

    #[inline(always)]
    pub fn now_ticks() -> u32 {
        unsafe { core::ptr::read_volatile(REG_TIMER0_TICK) }
    }

    pub async fn delay_ms(ms: u32) {
        // No real async wakeup yet: our executor busy-polls, so the only
        // power win Timer0 buys us today is precise timing. Block on the
        // hardware alarm rather than spinning a NOP loop.
        super::delay_ms(ms);
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

/// Fallback 64-bit IEEE used when the factory-programmed address at flash
/// `0x76000` is missing or all-FF/all-00. Stored over-the-air little-endian.
/// Two devices ever flashed without a unique factory address will collide on
/// this constant — flag for the smoke test.
// IEEE = 00:12:4B:00:0C:C3:5D:9F (original Telink fallback).
// Test B (bump to …A0) was run in session 5 and confirmed NCP stale per-IEEE
// state is NOT the cause: the same decrypt-fail pattern reproduced exactly
// on a fresh IEEE. Reverted to original to avoid littering ZHA's device DB
// with abandoned …A0 entries. See plan.md "Marker map" + "Test B result"
// for the data behind this decision.
const OUR_EXT_ADDR_FALLBACK: [u8; 8] = [0x9F, 0x5D, 0xC3, 0x0C, 0x00, 0x4B, 0x12, 0x00];

/// Telink factory information sector (per TLSR8258 stack): the 8-byte IEEE
/// MAC address is provisioned at this offset by the production tool.
#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
const TELINK_FACTORY_IEEE_ADDR: u32 = 0x76000;

/// Live IEEE address. Written exactly once by `init_our_ext_addr()` before
/// IRQs are enabled and then treated as read-only by the rest of the
/// firmware (including the RX ISR). Single-writer, multi-reader pattern.
static mut OUR_EXT_ADDR_STORAGE: [u8; 8] = OUR_EXT_ADDR_FALLBACK;

#[inline(always)]
fn our_ext_addr() -> [u8; 8] {
    unsafe { core::ptr::read_volatile(&raw const OUR_EXT_ADDR_STORAGE) }
}

/// Boot-time IEEE load from Telink factory flash sector. Falls back to the
/// compile-time constant when the factory area is unprogrammed (all 0xFF)
/// or zeroed (all 0x00). Must be called from `main()` before any RF or
/// timer IRQ is unmasked.
#[cfg(any(feature = "sensor", feature = "diag-smoke"))]
fn init_our_ext_addr() {
    let mut addr = [0xFFu8; 8];
    {
        let _cs = FlashCriticalSection::enter();
        flash_read_data(TELINK_FACTORY_IEEE_ADDR, &mut addr);
    }
    let all_ff = addr.iter().all(|b| *b == 0xFF);
    let all_00 = addr.iter().all(|b| *b == 0x00);
    if all_ff || all_00 {
        mark32(DBG_MODE_BASE + 0xE8, 0xAEFFAEFF);
        unsafe {
            core::ptr::write_volatile(&raw mut OUR_EXT_ADDR_STORAGE, OUR_EXT_ADDR_FALLBACK);
        }
        return;
    }
    mark32(
        DBG_MODE_BASE + 0xE8,
        u32::from_le_bytes([addr[0], addr[1], addr[2], addr[3]]),
    );
    mark32(
        DBG_MODE_BASE + 0xEC,
        u32::from_le_bytes([addr[4], addr[5], addr[6], addr[7]]),
    );
    unsafe {
        core::ptr::write_volatile(&raw mut OUR_EXT_ADDR_STORAGE, addr);
    }
}

/// Transmit a raw MAC frame and busy-wait for TX done.
/// `data` points to the TX DMA buffer already filled, starting at byte 0 (DMA header).
#[inline(never)]
fn tx_and_wait() -> bool {
    tx_and_wait_inner(true)
}

fn tx_and_wait_inner(settle_guard: bool) -> bool {
    let tx_buf = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
    radio::set_trx_off();
    radio::set_tx_dma_config(144);
    radio::tx_done_clear();
    radio::set_tx_mode();
    if settle_guard {
        spin_delay(10_000);
    }
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
        let ext_addr = our_ext_addr();
        for i in 0..8 {
            core::ptr::write_volatile(tx_buf.add(12 + i), ext_addr[i]);
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
        let ext_addr = our_ext_addr();
        for i in 0..8 {
            core::ptr::write_volatile(tx_buf.add(12 + i), ext_addr[i]);
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
                        rx_on_when_idle: true,
                        security_capable: true,
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
const SENSOR_ENDPOINT: u8 = 1;
#[cfg(feature = "sensor")]
const PROFILE_HOME_AUTOMATION: u16 = 0x0104;
#[cfg(feature = "sensor")]
const DEVICE_TEMPERATURE_SENSOR: u16 = 0x0302;
#[cfg(feature = "sensor")]
const CLUSTER_BASIC: u16 = 0x0000;
#[cfg(feature = "sensor")]
const CLUSTER_IDENTIFY: u16 = 0x0003;
#[cfg(feature = "sensor")]
const CLUSTER_TEMPERATURE_MEASUREMENT: u16 = 0x0402;
#[cfg(feature = "sensor")]
const CLUSTER_RELATIVE_HUMIDITY: u16 = 0x0405;
#[cfg(feature = "sensor")]
const ZDO_NWK_ADDR_REQ: u16 = 0x0000;
#[cfg(feature = "sensor")]
const ZDO_NWK_ADDR_RSP: u16 = 0x8000;
#[cfg(feature = "sensor")]
const ZDO_IEEE_ADDR_REQ: u16 = 0x0001;
#[cfg(feature = "sensor")]
const ZDO_IEEE_ADDR_RSP: u16 = 0x8001;
#[cfg(feature = "sensor")]
const ZDO_NODE_DESC_REQ: u16 = 0x0002;
#[cfg(feature = "sensor")]
const ZDO_NODE_DESC_RSP: u16 = 0x8002;
#[cfg(feature = "sensor")]
const ZDO_POWER_DESC_REQ: u16 = 0x0003;
#[cfg(feature = "sensor")]
const ZDO_POWER_DESC_RSP: u16 = 0x8003;
#[cfg(feature = "sensor")]
const ZDO_SIMPLE_DESC_REQ: u16 = 0x0004;
#[cfg(feature = "sensor")]
const ZDO_SIMPLE_DESC_RSP: u16 = 0x8004;
#[cfg(feature = "sensor")]
const ZDO_ACTIVE_EP_REQ: u16 = 0x0005;
#[cfg(feature = "sensor")]
const ZDO_ACTIVE_EP_RSP: u16 = 0x8005;
#[cfg(feature = "sensor")]
const ZDO_MATCH_DESC_REQ: u16 = 0x0006;
#[cfg(feature = "sensor")]
const ZDO_MATCH_DESC_RSP: u16 = 0x8006;
#[cfg(feature = "sensor")]
const ZDP_STATUS_SUCCESS: u8 = 0x00;
#[cfg(feature = "sensor")]
const ZDP_STATUS_DEVICE_NOT_FOUND: u8 = 0x81;
#[cfg(feature = "sensor")]
const ZDP_STATUS_INVALID_EP: u8 = 0x82;
#[cfg(feature = "sensor")]
const ZDP_STATUS_NO_MATCH: u8 = 0x86;
#[cfg(feature = "sensor")]
const SENSOR_MAC_CAPABILITY: u8 = 0x80 | 0x40 | 0x08; // Allocate address, security-capable, RX-on during join.

// ── Cycle 23: Verify-Key handshake + unicast Device_annce + light MAC TX-Err counters ──
//
// VK hash per ZB-3.0 R22 §B.1.4: HMAC-MMO(TC link key, 0x03).
#[cfg(feature = "sensor")]
fn mac_err_code(e: MacError) -> u32 {
    match e {
        MacError::NoBeacon => 1,
        MacError::InvalidParameter => 2,
        MacError::RadioError => 3,
        MacError::ChannelAccessFailure => 4,
        MacError::NoAck => 5,
        MacError::FrameTooLong => 6,
        MacError::Unsupported => 7,
        MacError::SecurityError => 8,
        _ => 0xFE,
    }
}

#[cfg(feature = "sensor")]
fn mac_result_code(r: &Result<(), MacError>) -> u32 {
    match r {
        Ok(()) => 0,
        Err(e) => 0x8000_0000 | mac_err_code(*e),
    }
}

#[cfg(feature = "sensor")]
fn bump_marker(addr: u32) {
    unsafe {
        let p = addr as *mut u32;
        core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
    }
}

/// Bare-metal APS-Verify-Key sender.
///
/// Builds: NWK-secured(APS-secured(VerifyKey || 0x04 || src_ieee || hash[16]))
/// destined for the TC (0x0000).
///
/// Marker layout (MODE-relative, audited free zone +0x148..+0x167):
///   +0x148  u32 entry counter
///   +0x14C  u32 APS-encrypt OK counter
///   +0x150  u32 NWK-encrypt OK counter
///   +0x154  u32 last MAC TX result (0=Ok, 0x8000_00xx=Err code)
///   +0x158  u32 attempt MAC result (0=Ok, 0x8000_00xx=Err code)
///   +0x164  u32 ConfirmKey RX counter (bumped by handle_sensor_frame)
///
/// MAC-layer counters (shared with other senders, MODE+0x108..+0x117):
///   +0x108  u32 csma_transmit entries
///   +0x10C  u32 csma_transmit OK
///   +0x110  u32 csma_transmit Err
///   +0x114  u32 last csma_transmit Err code
#[cfg(feature = "sensor")]
#[inline(never)]
fn send_verify_key_aps(
    mac: &mut Tlsr8258Mac,
    nwk_security: &mut NwkSecurity,
    aps_security: &mut ApsSecurity,
    nwk_frame_counter: &mut u32,
    nwk_seq: &mut u8,
    aps_seq: &mut u8,
    aps_fc_out: &mut u32,
    hash: &[u8; 16],
) -> Result<(), MacError> {
    bump_marker(DBG_MODE_BASE + 0x148);

    // ── APS plaintext: cmd_id(1) + key_type(1) + src_ieee(8) + hash(16) = 26 B
    let mut plain = [0u8; 26];
    plain[0] = ApsCommandId::VerifyKey as u8;
    plain[1] = 0x04; // KEY_TYPE_TC_LINK
    plain[2..10].copy_from_slice(&mac.extended_address);
    plain[10..26].copy_from_slice(hash);

    // ── APS header (Command, secured, ack-requested) ──
    let aps_counter = *aps_seq;
    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Command as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: true,
            ack_request: true,
            extended_header: false,
        },
        dst_endpoint: None,
        group_address: None,
        cluster_id: None,
        profile_id: None,
        src_endpoint: None,
        aps_counter,
        extended_header: None,
    };
    *aps_seq = aps_seq.wrapping_add(1);

    // ── APS aux header: ENC-MIC-32, key_id=Data(0=TC link), ext_nonce=1 ──
    let aps_sec_hdr = ApsSecurityHeader {
        security_control: SEC_LEVEL_ENC_MIC_32 | (KEY_ID_DATA_KEY << 3) | 0x20,
        frame_counter: *aps_fc_out,
        source_address: Some(mac.extended_address),
        key_seq_number: None,
    };
    *aps_fc_out = aps_fc_out.wrapping_add(1);

    let mut aps_buf = [0u8; 64];
    let aps_hdr_len = aps_header.serialize(&mut aps_buf);
    let aps_aux_len = aps_sec_hdr.serialize(&mut aps_buf[aps_hdr_len..]);
    let aps_aad_end = aps_hdr_len + aps_aux_len;

    let tc_key = *aps_security.default_tc_link_key();
    let ct_mic = match aps_security.encrypt(&aps_buf[..aps_aad_end], &plain, &tc_key, &aps_sec_hdr) {
        Some(ct) => ct,
        None => {
            mark32(DBG_MODE_BASE + 0x154, 0x5A50FF01);
            return Err(MacError::SecurityError);
        }
    };
    bump_marker(DBG_MODE_BASE + 0x14C);
    if aps_aad_end + ct_mic.len() > aps_buf.len() {
        return Err(MacError::FrameTooLong);
    }
    aps_buf[aps_aad_end..aps_aad_end + ct_mic.len()].copy_from_slice(&ct_mic);
    let aps_total = aps_aad_end + ct_mic.len();

    // ── NWK header (secured, unicast to TC) ──
    let nwk_dst = ShortAddress(0x0000);
    let nwk_header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            discover_route: 0,
            multicast: false,
            security: true,
            source_route: false,
            dst_ieee_present: false,
            src_ieee_present: false,
            end_device_initiator: true,
        },
        dst_addr: nwk_dst,
        src_addr: mac.short_address,
        radius: 30,
        seq_number: *nwk_seq,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    *nwk_seq = nwk_seq.wrapping_add(1);

    let key_entry = match nwk_security.active_key().cloned() {
        Some(k) => k,
        None => {
            mark32(DBG_MODE_BASE + 0x154, 0x5A50FF02);
            return Err(MacError::SecurityError);
        }
    };
    let nwk_sec_hdr = NwkSecurityHeader {
        security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
        frame_counter: *nwk_frame_counter,
        source_address: mac.extended_address,
        key_seq_number: key_entry.seq_number,
    };
    bump_nwk_counter(&*mac, &*nwk_security, nwk_frame_counter);

    let mut frame_buf = [0u8; 128];
    let nwk_len = nwk_header.serialize(&mut frame_buf);
    let nwk_aux_len = nwk_sec_hdr.serialize(&mut frame_buf[nwk_len..]);
    let nwk_aad_end = nwk_len + nwk_aux_len;
    let nwk_ct = match nwk_security.encrypt(
        &frame_buf[..nwk_aad_end],
        &aps_buf[..aps_total],
        &key_entry.key,
        &nwk_sec_hdr,
    ) {
        Some(ct) => ct,
        None => {
            mark32(DBG_MODE_BASE + 0x154, 0x5A50FF03);
            return Err(MacError::SecurityError);
        }
    };
    bump_marker(DBG_MODE_BASE + 0x150);
    if nwk_aad_end + nwk_ct.len() > frame_buf.len() {
        return Err(MacError::FrameTooLong);
    }
    frame_buf[nwk_aad_end..nwk_aad_end + nwk_ct.len()].copy_from_slice(&nwk_ct);
    // Clear NWK FCF security-level bits on the wire (per spec they are transmitted as 0).
    frame_buf[nwk_len] &= !0x07;
    let total = nwk_aad_end + nwk_ct.len();

    let next_hop = mac_next_hop_for_nwk_dst(mac, nwk_dst);
    let frame = mac.build_data_frame(
        &MacAddress::Short(mac.pan_id, next_hop),
        &frame_buf[..total],
        true,
    )?;

    bump_marker(DBG_MODE_BASE + 0x108);
    let tx = mac.csma_transmit(&frame, true);
    let code = mac_result_code(&tx);
    mark32(DBG_MODE_BASE + 0x154, code);
    if tx.is_ok() {
        bump_marker(DBG_MODE_BASE + 0x10C);
    } else {
        bump_marker(DBG_MODE_BASE + 0x110);
        mark32(DBG_MODE_BASE + 0x114, code);
    }
    tx
}

#[cfg(feature = "sensor")]
#[inline(never)]
async fn send_device_annce(
    mac: &mut Tlsr8258Mac,
    nwk_security: &mut NwkSecurity,
    nwk_frame_counter: &mut u32,
    nwk_seq: &mut u8,
    aps_seq: &mut u8,
    zdo_seq: &mut u8,
    dst: ShortAddress,
) -> Result<(), MacError> {
    let annce = DeviceAnnounce {
        nwk_addr: mac.short_address,
        ieee_addr: mac.extended_address,
        capability: SENSOR_MAC_CAPABILITY,
    };

    let mut zdp_payload = [0u8; 1 + DeviceAnnounce::WIRE_SIZE];
    zdp_payload[0] = *zdo_seq;
    *zdo_seq = zdo_seq.wrapping_add(1);
    let _ = annce.serialize(&mut zdp_payload[1..]);

    let nwk_dst = dst;
    let is_broadcast = nwk_dst.0 >= 0xFFFC;
    let nwk_header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            discover_route: 0,
            multicast: false,
            security: nwk_security.active_key().is_some(),
            source_route: false,
            dst_ieee_present: false,
            src_ieee_present: false,
            end_device_initiator: true,
        },
        dst_addr: nwk_dst,
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
            delivery_mode: if is_broadcast {
                ApsDeliveryMode::Broadcast as u8
            } else {
                ApsDeliveryMode::Unicast as u8
            },
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

    let mut aps_plain = [0u8; 64];
    let aps_len = aps_header.serialize(&mut aps_plain);
    let aps_total = aps_len + zdp_payload.len();
    if aps_total > aps_plain.len() {
        return Err(MacError::FrameTooLong);
    }
    aps_plain[aps_len..aps_total].copy_from_slice(&zdp_payload);

    let mut payload = [0u8; 96];
    let nwk_len = nwk_header.serialize(&mut payload);
    let total = if nwk_security.active_key().is_some() {
        let Some(key_entry) = nwk_security.active_key().cloned() else {
            return Err(MacError::SecurityError);
        };
        let sec_hdr = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: *nwk_frame_counter,
            source_address: mac.extended_address,
            key_seq_number: key_entry.seq_number,
        };
        bump_nwk_counter(&*mac, &*nwk_security, nwk_frame_counter);

        let sec_len = sec_hdr.serialize(&mut payload[nwk_len..]);
        let aad_len = nwk_len + sec_len;
        let Some(encrypted) =
            nwk_security.encrypt(&payload[..aad_len], &aps_plain[..aps_total], &key_entry.key, &sec_hdr)
        else {
            return Err(MacError::SecurityError);
        };
        if aad_len + encrypted.len() > payload.len() {
            return Err(MacError::FrameTooLong);
        }
        payload[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
        payload[nwk_len] &= !0x07;
        aad_len + encrypted.len()
    } else {
        if nwk_len + aps_total > payload.len() {
            return Err(MacError::FrameTooLong);
        }
        payload[nwk_len..nwk_len + aps_total].copy_from_slice(&aps_plain[..aps_total]);
        nwk_len + aps_total
    };

    let next_hop = mac_next_hop_for_nwk_dst(mac, nwk_dst);
    mac.mcps_data(zigbee_mac::primitives::McpsDataRequest {
        src_addr_mode: zigbee_mac::primitives::AddressMode::Short,
        dst_address: MacAddress::Short(mac.pan_id, next_hop),
        payload: &payload[..total],
        msdu_handle: *aps_seq,
        tx_options: zigbee_mac::primitives::TxOptions {
            ack_tx: next_hop.0 != 0xFFFF,
            indirect: false,
            security_enabled: false,
        },
    })
    .await?;

    Ok(())
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn mac_next_hop_for_nwk_dst(mac: &Tlsr8258Mac, dst_short: ShortAddress) -> ShortAddress {
    if dst_short.0 == 0xFFFF || dst_short.0 == mac.coord_short_address.0 {
        dst_short
    } else {
        mac.coord_short_address
    }
}

#[cfg(feature = "sensor")]
#[inline(never)]
async fn send_network_key_request(
    mac: &mut Tlsr8258Mac,
    nwk_seq: &mut u8,
    aps_seq: &mut u8,
) -> Result<(), MacError> {
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
            end_device_initiator: true,
        },
        dst_addr: ShortAddress(0x0000),
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
            frame_type: ApsFrameType::Command as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: None,
        group_address: None,
        cluster_id: None,
        profile_id: None,
        src_endpoint: None,
        aps_counter: *aps_seq,
        extended_header: None,
    };
    *aps_seq = aps_seq.wrapping_add(1);

    let mut payload = [0u8; 40];
    let nwk_len = nwk_header.serialize(&mut payload);
    let aps_len = aps_header.serialize(&mut payload[nwk_len..]);
    let cmd_off = nwk_len + aps_len;
    if cmd_off + 2 > payload.len() {
        return Err(MacError::FrameTooLong);
    }
    payload[cmd_off] = ApsCommandId::RequestKey as u8;
    payload[cmd_off + 1] = 0x01; // Standard network key.
    let total = cmd_off + 2;

    let next_hop = mac_next_hop_for_nwk_dst(mac, ShortAddress(0x0000));
    mac.mcps_data(zigbee_mac::primitives::McpsDataRequest {
        src_addr_mode: zigbee_mac::primitives::AddressMode::Short,
        dst_address: MacAddress::Short(mac.pan_id, next_hop),
        payload: &payload[..total],
        msdu_handle: *aps_seq,
        tx_options: zigbee_mac::primitives::TxOptions {
            ack_tx: true,
            indirect: false,
            security_enabled: false,
        },
    })
    .await?;
    mark32(DBG_MODE_BASE + 0xB4, 0xA75C0400);
    Ok(())
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn send_zdo_response_raw(
    mac: &mut Tlsr8258Mac,
    nwk_security: &mut NwkSecurity,
    nwk_frame_counter: &mut u32,
    dst_short: ShortAddress,
    cluster_id: u16,
    zdp_payload: &[u8],
    nwk_seq: &mut u8,
    aps_seq: &mut u8,
) -> Result<(), MacError> {
    let nwk_header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            discover_route: 0,
            multicast: false,
            security: nwk_security.active_key().is_some(),
            source_route: false,
            dst_ieee_present: false,
            src_ieee_present: false,
            end_device_initiator: true,
        },
        dst_addr: dst_short,
        src_addr: mac.short_address,
        radius: 30,
        seq_number: *nwk_seq,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    *nwk_seq = nwk_seq.wrapping_add(1);

    let aps_counter = *aps_seq;
    let aps_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Data as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: Some(ZDO_ENDPOINT),
        group_address: None,
        cluster_id: Some(cluster_id),
        profile_id: Some(PROFILE_ZDP),
        src_endpoint: Some(ZDO_ENDPOINT),
        aps_counter,
        extended_header: None,
    };
    *aps_seq = aps_seq.wrapping_add(1);

    let mut aps_plain = [0u8; 64];
    let aps_len = aps_header.serialize(&mut aps_plain);
    let aps_total = aps_len + zdp_payload.len();
    if aps_total > aps_plain.len() {
        return Err(MacError::FrameTooLong);
    }
    aps_plain[aps_len..aps_total].copy_from_slice(zdp_payload);

    let mut payload = [0u8; 96];
    let nwk_len = nwk_header.serialize(&mut payload);
    let total = if nwk_security.active_key().is_some() {
        let Some(key_entry) = nwk_security.active_key().cloned() else {
            mark32(DBG_MODE_BASE + 0x94, 0x5A50FF01);
            return Err(MacError::SecurityError);
        };
        let sec_hdr = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: *nwk_frame_counter,
            source_address: mac.extended_address,
            key_seq_number: key_entry.seq_number,
        };
        bump_nwk_counter(&*mac, &*nwk_security, nwk_frame_counter);

        let sec_len = sec_hdr.serialize(&mut payload[nwk_len..]);
        let aad_len = nwk_len + sec_len;
        let Some(encrypted) =
            nwk_security.encrypt(&payload[..aad_len], &aps_plain[..aps_total], &key_entry.key, &sec_hdr)
        else {
            mark32(DBG_MODE_BASE + 0x94, 0x5A50FF02);
            return Err(MacError::SecurityError);
        };
        if aad_len + encrypted.len() > payload.len() {
            return Err(MacError::FrameTooLong);
        }
        payload[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
        payload[nwk_len] &= !0x07;
        aad_len + encrypted.len()
    } else {
        if nwk_len + aps_total > payload.len() {
            return Err(MacError::FrameTooLong);
        }
        payload[nwk_len..nwk_len + aps_total].copy_from_slice(&aps_plain[..aps_total]);
        nwk_len + aps_total
    };

    let next_hop = mac_next_hop_for_nwk_dst(mac, dst_short);
    let frame = mac.build_data_frame(
        &MacAddress::Short(mac.pan_id, next_hop),
        &payload[..total],
        true,
    )?;
    mac.csma_transmit(&frame, true)?;

    Ok(())
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn send_aps_ack_raw(
    mac: &mut Tlsr8258Mac,
    nwk_security: &mut NwkSecurity,
    nwk_frame_counter: &mut u32,
    dst_short: ShortAddress,
    request_header: &ApsHeader,
    nwk_seq: &mut u8,
) -> Result<(), MacError> {
    let (Some(dst_endpoint), Some(src_endpoint), Some(cluster_id), Some(profile_id)) = (
        request_header.src_endpoint,
        request_header.dst_endpoint,
        request_header.cluster_id,
        request_header.profile_id,
    ) else {
        return Ok(());
    };

    let nwk_header = NwkHeader {
        frame_control: NwkFrameControl {
            frame_type: NwkFrameType::Data as u8,
            protocol_version: 0x02,
            discover_route: 0,
            multicast: false,
            security: nwk_security.active_key().is_some(),
            source_route: false,
            dst_ieee_present: false,
            src_ieee_present: false,
            end_device_initiator: true,
        },
        dst_addr: dst_short,
        src_addr: mac.short_address,
        radius: 30,
        seq_number: *nwk_seq,
        dst_ieee: None,
        src_ieee: None,
        multicast_control: None,
        source_route: None,
    };
    *nwk_seq = nwk_seq.wrapping_add(1);

    let ack_header = ApsHeader {
        frame_control: ApsFrameControl {
            frame_type: ApsFrameType::Ack as u8,
            delivery_mode: ApsDeliveryMode::Unicast as u8,
            ack_format: false,
            security: false,
            ack_request: false,
            extended_header: false,
        },
        dst_endpoint: Some(dst_endpoint),
        group_address: None,
        cluster_id: Some(cluster_id),
        profile_id: Some(profile_id),
        src_endpoint: Some(src_endpoint),
        aps_counter: request_header.aps_counter,
        extended_header: None,
    };

    let mut aps_plain = [0u8; 16];
    let aps_total = ack_header.serialize(&mut aps_plain);

    let mut payload = [0u8; 80];
    let nwk_len = nwk_header.serialize(&mut payload);
    let total = if nwk_security.active_key().is_some() {
        let Some(key_entry) = nwk_security.active_key().cloned() else {
            return Err(MacError::SecurityError);
        };
        let sec_hdr = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: *nwk_frame_counter,
            source_address: mac.extended_address,
            key_seq_number: key_entry.seq_number,
        };
        bump_nwk_counter(&*mac, &*nwk_security, nwk_frame_counter);

        let sec_len = sec_hdr.serialize(&mut payload[nwk_len..]);
        let aad_len = nwk_len + sec_len;
        let Some(encrypted) =
            nwk_security.encrypt(&payload[..aad_len], &aps_plain[..aps_total], &key_entry.key, &sec_hdr)
        else {
            return Err(MacError::SecurityError);
        };
        if aad_len + encrypted.len() > payload.len() {
            return Err(MacError::FrameTooLong);
        }
        payload[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
        payload[nwk_len] &= !0x07;
        aad_len + encrypted.len()
    } else {
        if nwk_len + aps_total > payload.len() {
            return Err(MacError::FrameTooLong);
        }
        payload[nwk_len..nwk_len + aps_total].copy_from_slice(&aps_plain[..aps_total]);
        nwk_len + aps_total
    };

    let next_hop = mac_next_hop_for_nwk_dst(mac, dst_short);
    let frame = mac.build_data_frame(
        &MacAddress::Short(mac.pan_id, next_hop),
        &payload[..total],
        true,
    )?;
    mac.csma_transmit(&frame, false)?;
    Ok(())
}

#[cfg(feature = "sensor")]
fn push_le16(buf: &mut [u8], off: &mut usize, value: u16) -> Option<()> {
    if *off + 2 > buf.len() {
        return None;
    }
    buf[*off..*off + 2].copy_from_slice(&value.to_le_bytes());
    *off += 2;
    Some(())
}

#[cfg(feature = "sensor")]
fn push_u8(buf: &mut [u8], off: &mut usize, value: u8) -> Option<()> {
    if *off >= buf.len() {
        return None;
    }
    buf[*off] = value;
    *off += 1;
    Some(())
}

#[cfg(feature = "sensor")]
fn cluster_supported(cluster: u16) -> bool {
    cluster == CLUSTER_BASIC
        || cluster == CLUSTER_IDENTIFY
        || cluster == CLUSTER_TEMPERATURE_MEASUREMENT
        || cluster == CLUSTER_RELATIVE_HUMIDITY
}

#[cfg(feature = "sensor")]
fn match_desc_matches(req: &[u8]) -> bool {
    if req.len() < 6 {
        return false;
    }
    let profile = u16::from_le_bytes([req[2], req[3]]);
    if profile != PROFILE_HOME_AUTOMATION {
        return false;
    }

    let in_count = req[4] as usize;
    let mut off = 5;
    if req.len() < off + in_count * 2 + 1 {
        return false;
    }
    if in_count == 0 {
        return true;
    }
    for _ in 0..in_count {
        let cluster = u16::from_le_bytes([req[off], req[off + 1]]);
        if cluster_supported(cluster) {
            return true;
        }
        off += 2;
    }

    false
}

#[cfg(feature = "sensor")]
fn build_zdo_response(
    cluster_id: u16,
    req_payload: &[u8],
    short_addr: ShortAddress,
    ieee_addr: [u8; 8],
    is_broadcast: bool,
    out: &mut [u8],
) -> Option<(u16, usize)> {
    if req_payload.is_empty() {
        return None;
    }

    let tsn = req_payload[0];
    let req = &req_payload[1..];
    let mut off = 0usize;
    push_u8(out, &mut off, tsn)?;

    match cluster_id {
        ZDO_NWK_ADDR_REQ => {
            if req.len() < 10 {
                return None;
            }
            let mut requested_ieee = [0u8; 8];
            requested_ieee.copy_from_slice(&req[..8]);
            let matched = requested_ieee == ieee_addr;
            // Per ZB §2.4.4.1.1.5: on broadcast NWK_addr_req, silently drop when
            // the target IEEE does not match. Spamming DEVICE_NOT_FOUND to every
            // broadcast on the PAN fills the parent's TX queue and may starve
            // out legitimate downlink (e.g. Node_Desc_req from the TC).
            if !matched && is_broadcast {
                return None;
            }
            push_u8(
                out,
                &mut off,
                if matched {
                    ZDP_STATUS_SUCCESS
                } else {
                    ZDP_STATUS_DEVICE_NOT_FOUND
                },
            )?;
            out.get_mut(off..off + 8)?.copy_from_slice(&requested_ieee);
            off += 8;
            push_le16(out, &mut off, if matched { short_addr.0 } else { 0x0000 })?;
            Some((ZDO_NWK_ADDR_RSP, off))
        }
        ZDO_IEEE_ADDR_REQ => {
            if req.len() < 4 {
                return None;
            }
            let requested_short = u16::from_le_bytes([req[0], req[1]]);
            let matched = requested_short == short_addr.0;
            // Per ZB §2.4.4.1.2.5: same rule for broadcast IEEE_addr_req.
            if !matched && is_broadcast {
                return None;
            }
            push_u8(
                out,
                &mut off,
                if matched {
                    ZDP_STATUS_SUCCESS
                } else {
                    ZDP_STATUS_DEVICE_NOT_FOUND
                },
            )?;
            out.get_mut(off..off + 8)?
                .copy_from_slice(if matched { &ieee_addr } else { &[0u8; 8] });
            off += 8;
            push_le16(out, &mut off, requested_short)?;
            Some((ZDO_IEEE_ADDR_RSP, off))
        }
        ZDO_NODE_DESC_REQ => {
            if req.len() < 2 {
                return None;
            }
            // NWKAddrOfInterest must equal our short, else this isn't for us.
            let target = u16::from_le_bytes([req[0], req[1]]);
            if target != short_addr.0 {
                return None;
            }
            push_u8(out, &mut off, ZDP_STATUS_SUCCESS)?;
            push_le16(out, &mut off, short_addr.0)?;
            push_u8(out, &mut off, 0x02)?; // End device, no complex/user descriptor.
            push_u8(out, &mut off, 0x40)?; // 2.4 GHz band.
            push_u8(out, &mut off, SENSOR_MAC_CAPABILITY)?;
            push_le16(out, &mut off, 0x0000)?; // Manufacturer code.
            push_u8(out, &mut off, 127)?; // Max buffer size.
            push_le16(out, &mut off, 127)?; // Max incoming transfer.
            push_le16(
                out,
                &mut off,
                SENSOR_STACK_COMPLIANCE_REVISION << 9,
            )?; // Server mask: stack compliance revision.
            push_le16(out, &mut off, 127)?; // Max outgoing transfer.
            push_u8(out, &mut off, 0x00)?; // Descriptor capabilities.
            Some((ZDO_NODE_DESC_RSP, off))
        }
        ZDO_POWER_DESC_REQ => {
            if req.len() < 2 {
                return None;
            }
            let target = u16::from_le_bytes([req[0], req[1]]);
            if target != short_addr.0 {
                return None;
            }
            push_u8(out, &mut off, ZDP_STATUS_SUCCESS)?;
            push_le16(out, &mut off, short_addr.0)?;
            push_u8(out, &mut off, 0x40)?; // Battery power source available.
            push_u8(out, &mut off, 0xC4)?; // Battery, 100%.
            Some((ZDO_POWER_DESC_RSP, off))
        }
        ZDO_ACTIVE_EP_REQ => {
            if req.len() < 2 {
                return None;
            }
            let target = u16::from_le_bytes([req[0], req[1]]);
            if target != short_addr.0 {
                return None;
            }
            push_u8(out, &mut off, ZDP_STATUS_SUCCESS)?;
            push_le16(out, &mut off, short_addr.0)?;
            push_u8(out, &mut off, 1)?;
            push_u8(out, &mut off, SENSOR_ENDPOINT)?;
            Some((ZDO_ACTIVE_EP_RSP, off))
        }
        ZDO_SIMPLE_DESC_REQ => {
            if req.len() < 3 {
                return None;
            }
            let target = u16::from_le_bytes([req[0], req[1]]);
            if target != short_addr.0 {
                return None;
            }
            let requested_ep = req[2];
            push_u8(
                out,
                &mut off,
                if requested_ep == SENSOR_ENDPOINT {
                    ZDP_STATUS_SUCCESS
                } else {
                    ZDP_STATUS_INVALID_EP
                },
            )?;
            push_le16(out, &mut off, short_addr.0)?;
            if requested_ep != SENSOR_ENDPOINT {
                push_u8(out, &mut off, 0)?;
                return Some((ZDO_SIMPLE_DESC_RSP, off));
            }
            push_u8(out, &mut off, 16)?; // Simple descriptor byte length.
            push_u8(out, &mut off, SENSOR_ENDPOINT)?;
            push_le16(out, &mut off, PROFILE_HOME_AUTOMATION)?;
            push_le16(out, &mut off, DEVICE_TEMPERATURE_SENSOR)?;
            push_u8(out, &mut off, 0)?;
            push_u8(out, &mut off, 4)?;
            push_le16(out, &mut off, CLUSTER_BASIC)?;
            push_le16(out, &mut off, CLUSTER_IDENTIFY)?;
            push_le16(out, &mut off, CLUSTER_TEMPERATURE_MEASUREMENT)?;
            push_le16(out, &mut off, CLUSTER_RELATIVE_HUMIDITY)?;
            push_u8(out, &mut off, 0)?;
            Some((ZDO_SIMPLE_DESC_RSP, off))
        }
        ZDO_MATCH_DESC_REQ => {
            let matched = match_desc_matches(req);
            // Match_Desc is typically broadcast; non-matching devices MUST NOT reply.
            if !matched {
                return None;
            }
            push_u8(out, &mut off, ZDP_STATUS_SUCCESS)?;
            push_le16(out, &mut off, short_addr.0)?;
            push_u8(out, &mut off, 1)?;
            push_u8(out, &mut off, SENSOR_ENDPOINT)?;
            Some((ZDO_MATCH_DESC_RSP, off))
        }
        _ => None,
    }
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn decrypt_sensor_aps_payload(
    aps_security: &mut ApsSecurity,
    aps_data: &[u8],
    aps_header_len: usize,
) -> Option<heapless::Vec<u8, 128>> {
    let Some((sec_hdr, sec_len)) = ApsSecurityHeader::parse(&aps_data[aps_header_len..]) else {
        mark32(DBG_MODE_BASE + 0xB0, 0xA75CFF01);
        return None;
    };
    let aad_end = aps_header_len + sec_len;
    if aps_data.len() < aad_end {
        mark32(DBG_MODE_BASE + 0xB0, 0xA75CFF02);
        return None;
    }

    let key_id = ApsSecurityHeader::key_identifier(sec_hdr.security_control);
    mark32(
        DBG_MODE_BASE + 0xB4,
        key_id as u32 | ((sec_hdr.frame_counter & 0x00FF_FFFF) << 8),
    );

    let key = match key_id {
        KEY_ID_DATA_KEY => *aps_security.default_tc_link_key(),
        KEY_ID_KEY_TRANSPORT => derive_key_transport_key(aps_security.default_tc_link_key()),
        KEY_ID_KEY_LOAD => derive_key_load_key(aps_security.default_tc_link_key()),
        _ => {
            mark32(DBG_MODE_BASE + 0xB0, 0xA75CFF03);
            return None;
        }
    };

    let replay_key_type = if key_id == KEY_ID_DATA_KEY {
        ApsKeyType::TrustCenterLinkKey
    } else {
        ApsKeyType::NetworkKey
    };
    if let Some(src) = &sec_hdr.source_address {
        if !aps_security.check_frame_counter(src, replay_key_type, sec_hdr.frame_counter) {
            mark32(DBG_MODE_BASE + 0xB0, 0xA75CFF04);
            return None;
        }
    }

    let ciphertext = &aps_data[aad_end..];
    let mut patched_aad = [0u8; 64];
    if aad_end > patched_aad.len() {
        mark32(DBG_MODE_BASE + 0xB0, 0xA75CFF05);
        return None;
    }
    patched_aad[..aad_end].copy_from_slice(&aps_data[..aad_end]);
    patched_aad[aps_header_len] =
        (patched_aad[aps_header_len] & !0x07) | SEC_LEVEL_ENC_MIC_32;

    let plain = aps_security
        .decrypt(&patched_aad[..aad_end], ciphertext, &key, &sec_hdr)
        .or_else(|| aps_security.decrypt(&aps_data[..aad_end], ciphertext, &key, &sec_hdr))
        .or_else(|| {
            if key_id == KEY_ID_KEY_TRANSPORT {
                let tc_key = *aps_security.default_tc_link_key();
                aps_security
                    .decrypt(&patched_aad[..aad_end], ciphertext, &tc_key, &sec_hdr)
                    .or_else(|| {
                        aps_security.decrypt(&aps_data[..aad_end], ciphertext, &tc_key, &sec_hdr)
                    })
            } else {
                None
            }
        })?;

    if let Some(src) = &sec_hdr.source_address {
        aps_security.commit_frame_counter(src, replay_key_type, sec_hdr.frame_counter);
    }
    mark32(DBG_MODE_BASE + 0xB0, 0xA75C0000 | plain.len() as u32);
    Some(plain)
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn decrypt_initial_nwk_payload(
    nwk_security: &NwkSecurity,
    aps_security: &ApsSecurity,
    data: &[u8],
    nwk_len: usize,
    sec_len: usize,
    sec_hdr: &NwkSecurityHeader,
) -> Option<heapless::Vec<u8, 128>> {
    let aad_len = nwk_len + sec_len;
    if data.len() < aad_len || aad_len > 64 {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA07);
        return None;
    }

    let ciphertext = &data[aad_len..];
    let mut aad = [0u8; 64];
    aad[..aad_len].copy_from_slice(&data[..aad_len]);

    let mut patched_aad = aad;
    patched_aad[nwk_len] = (patched_aad[nwk_len] & !0x07) | SEC_LEVEL_ENC_MIC_32;

    let tc_key = *aps_security.default_tc_link_key();
    if let Some(plain) = nwk_security.decrypt(&patched_aad[..aad_len], ciphertext, &tc_key, sec_hdr)
    {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5C101);
        return Some(plain);
    }
    if let Some(plain) = nwk_security.decrypt(&aad[..aad_len], ciphertext, &tc_key, sec_hdr) {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5C102);
        return Some(plain);
    }

    let key_transport = derive_key_transport_key(&tc_key);
    if let Some(plain) =
        nwk_security.decrypt(&patched_aad[..aad_len], ciphertext, &key_transport, sec_hdr)
    {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5C103);
        return Some(plain);
    }
    if let Some(plain) = nwk_security.decrypt(&aad[..aad_len], ciphertext, &key_transport, sec_hdr)
    {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5C104);
        return Some(plain);
    }

    None
}

#[cfg(feature = "sensor")]
fn sensor_nv_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0xA5, |acc, byte| acc.wrapping_add(*byte).rotate_left(1))
}

/// PAN identity captured at persist time. Lets a future boot resume on the
/// same network (channel + pan_id + parent) without re-running an active
/// scan. Currently stored in v3 NV records but not yet consumed by the
/// startup path (the existing flow does a full scan + associate, then
/// loads the persisted key on success).
#[cfg(feature = "sensor")]
#[derive(Clone, Copy)]
struct PanIdentity {
    pan_id: u16,
    short_address: u16,
    parent_short_address: u16,
    ext_pan_id: [u8; 8],
    channel: u8,
}

#[cfg(feature = "sensor")]
impl PanIdentity {
    fn from_mac(mac: &Tlsr8258Mac) -> Self {
        Self {
            pan_id: mac.pan_id.0,
            short_address: mac.short_address.0,
            parent_short_address: mac.coord_short_address.0,
            ext_pan_id: mac.coord_extended_address,
            channel: mac.channel,
        }
    }
    const NONE: Self = Self {
        pan_id: 0xFFFF,
        short_address: 0xFFFF,
        parent_short_address: 0xFFFF,
        ext_pan_id: [0xFF; 8],
        channel: 0,
    };
}

#[cfg(feature = "sensor")]
#[allow(dead_code)]
struct LoadedNvState {
    frame_counter: u32,
    /// Populated only for v3+ records. v1/v2 records were saved before PAN
    /// identity was tracked, so they return `None` here and the device
    /// falls back to scan-on-boot.
    pan: Option<PanIdentity>,
}

/// Increment the NWK outgoing frame counter and persist a fresh
/// `(key, seq, counter + RESERVE, pan)` record to flash if the live counter
/// has advanced by at least `SENSOR_NWK_COUNTER_PERSIST_INTERVAL` since the
/// last save. Each persist costs one flash sector erase + page program, so
/// we rate-limit by the interval. Called from the 3 NWK-secured send paths.
#[cfg(feature = "sensor")]
#[inline(never)]
fn bump_nwk_counter(mac: &Tlsr8258Mac, nwk_security: &NwkSecurity, nwk_frame_counter: &mut u32) {
    *nwk_frame_counter = nwk_frame_counter.wrapping_add(1);
    let last = unsafe { core::ptr::read_volatile(&raw const SENSOR_LAST_PERSISTED_COUNTER) };
    if nwk_frame_counter.wrapping_sub(last) < SENSOR_NWK_COUNTER_PERSIST_INTERVAL {
        return;
    }
    let Some(key_entry) = nwk_security.active_key() else { return };
    persist_network_state(
        &key_entry.key,
        key_entry.seq_number,
        nwk_frame_counter.wrapping_add(SENSOR_NWK_COUNTER_RESERVE),
        PanIdentity::from_mac(mac),
    );
    unsafe {
        core::ptr::write_volatile(&raw mut SENSOR_LAST_PERSISTED_COUNTER, *nwk_frame_counter);
    }
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn persist_network_state(
    key: &[u8; 16],
    key_seq: u8,
    next_frame_counter: u32,
    pan: PanIdentity,
) {
    let mut record = [0xFFu8; SENSOR_NV_RECORD_LEN];
    record[0..4].copy_from_slice(&SENSOR_NV_MAGIC.to_le_bytes());
    record[4] = SENSOR_NV_VERSION;
    record[5] = key_seq;
    record[6..22].copy_from_slice(key);
    record[22..26].copy_from_slice(&next_frame_counter.to_le_bytes());
    // v3 extension: PAN identity.
    record[26..28].copy_from_slice(&pan.pan_id.to_le_bytes());
    record[28..30].copy_from_slice(&pan.short_address.to_le_bytes());
    record[30..32].copy_from_slice(&pan.parent_short_address.to_le_bytes());
    record[32..40].copy_from_slice(&pan.ext_pan_id);
    record[40] = pan.channel;
    // record[41..62] reserved (0xFF) for future fields.
    record[62] = sensor_nv_checksum(&record[..62]);
    record[63] = 0xFF;

    {
        let _cs = FlashCriticalSection::enter();
        flash_erase_sector(SENSOR_NV_FLASH_ADDR);
        flash_page_program(SENSOR_NV_FLASH_ADDR, &record);
    }
    mark32(DBG_MODE_BASE + 0xB8, 0xA75C0300 | key_seq as u32);
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn load_persisted_network_state(nwk_security: &mut NwkSecurity) -> Option<LoadedNvState> {
    let mut record = [0xFFu8; SENSOR_NV_RECORD_LEN];
    {
        let _cs = FlashCriticalSection::enter();
        flash_read_data(SENSOR_NV_FLASH_ADDR, &mut record);
    }

    let magic = u32::from_le_bytes([record[0], record[1], record[2], record[3]]);
    if magic != SENSOR_NV_MAGIC {
        mark32(DBG_MODE_BASE + 0xB8, 0xA75CF001);
        return None;
    }
    let version = record[4];

    // v1: 22 B payload + checksum at [22]. No PAN identity, no counter.
    if version == 1 {
        if sensor_nv_checksum(&record[..22]) != record[22] {
            mark32(DBG_MODE_BASE + 0xB8, 0xA75CF002);
            return None;
        }
        let key_seq = record[5];
        let mut key = [0u8; 16];
        key.copy_from_slice(&record[6..22]);
        nwk_security.set_network_key(key, key_seq);
        let next_frame_counter = SENSOR_NWK_COUNTER_RESERVE;
        // Migrate to v3 with empty PAN identity; PAN fields will be filled
        // on the next bump_nwk_counter once the device re-associates.
        persist_network_state(
            &key,
            key_seq,
            next_frame_counter.wrapping_add(SENSOR_NWK_COUNTER_RESERVE),
            PanIdentity::NONE,
        );
        mark32(DBG_MODE_BASE + 0xB8, 0xA75C1200 | key_seq as u32);
        return Some(LoadedNvState { frame_counter: next_frame_counter, pan: None });
    }

    // v2: 26 B payload + checksum at [26]. Counter, no PAN identity.
    if version == 2 {
        if sensor_nv_checksum(&record[..26]) != record[26] {
            mark32(DBG_MODE_BASE + 0xB8, 0xA75CF002);
            return None;
        }
        let key_seq = record[5];
        let mut key = [0u8; 16];
        key.copy_from_slice(&record[6..22]);
        let stored_frame_counter =
            u32::from_le_bytes([record[22], record[23], record[24], record[25]]);
        let next_frame_counter = stored_frame_counter.max(SENSOR_NWK_COUNTER_RESERVE);
        nwk_security.set_network_key(key, key_seq);
        persist_network_state(
            &key,
            key_seq,
            next_frame_counter.wrapping_add(SENSOR_NWK_COUNTER_RESERVE),
            PanIdentity::NONE,
        );
        mark32(DBG_MODE_BASE + 0xB8, 0xA75C0200 | key_seq as u32);
        return Some(LoadedNvState { frame_counter: next_frame_counter, pan: None });
    }

    // v3: 62 B payload + checksum at [62].
    if version != SENSOR_NV_VERSION {
        mark32(DBG_MODE_BASE + 0xB8, 0xA75CF003);
        return None;
    }
    if sensor_nv_checksum(&record[..62]) != record[62] {
        mark32(DBG_MODE_BASE + 0xB8, 0xA75CF002);
        return None;
    }

    let key_seq = record[5];
    let mut key = [0u8; 16];
    key.copy_from_slice(&record[6..22]);
    let stored_frame_counter =
        u32::from_le_bytes([record[22], record[23], record[24], record[25]]);
    let next_frame_counter = stored_frame_counter.max(SENSOR_NWK_COUNTER_RESERVE);
    let pan_id = u16::from_le_bytes([record[26], record[27]]);
    let short_address = u16::from_le_bytes([record[28], record[29]]);
    let parent_short_address = u16::from_le_bytes([record[30], record[31]]);
    let mut ext_pan_id = [0u8; 8];
    ext_pan_id.copy_from_slice(&record[32..40]);
    let channel = record[40];
    let pan = PanIdentity { pan_id, short_address, parent_short_address, ext_pan_id, channel };
    nwk_security.set_network_key(key, key_seq);

    persist_network_state(
        &key,
        key_seq,
        next_frame_counter.wrapping_add(SENSOR_NWK_COUNTER_RESERVE),
        pan,
    );
    mark32(DBG_MODE_BASE + 0xB8, 0xA75C0300 | key_seq as u32);
    Some(LoadedNvState { frame_counter: next_frame_counter, pan: Some(pan) })
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn persist_network_key(
    key: &[u8; 16],
    key_seq: u8,
    next_frame_counter: u32,
    pan: PanIdentity,
) {
    persist_network_state(
        key,
        key_seq,
        next_frame_counter.wrapping_add(SENSOR_NWK_COUNTER_RESERVE),
        pan,
    );
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn install_transport_key(
    cmd_payload: &[u8],
    mac: &Tlsr8258Mac,
    nwk_security: &mut NwkSecurity,
    nwk_frame_counter: &mut u32,
) -> bool {
    if cmd_payload.len() < 17 {
        mark32(DBG_MODE_BASE + 0xB8, 0xA75CFF10 | ((cmd_payload.len() as u32) << 16));
        return false;
    }

    let key_type = cmd_payload[0];
    if key_type != 0x01 {
        mark32(DBG_MODE_BASE + 0xB8, 0xA75CFF20 | key_type as u32);
        return false;
    }

    let mut key = [0u8; 16];
    key.copy_from_slice(&cmd_payload[1..17]);
    let key_seq = if cmd_payload.len() > 17 { cmd_payload[17] } else { 0 };
    nwk_security.set_network_key(key, key_seq);
    if *nwk_frame_counter < SENSOR_NWK_COUNTER_RESERVE {
        *nwk_frame_counter = SENSOR_NWK_COUNTER_RESERVE;
    }
    persist_network_key(&key, key_seq, *nwk_frame_counter, PanIdentity::from_mac(mac));
    mark32(DBG_MODE_BASE + 0xB8, 0xA75C0100 | key_seq as u32);
    true
}

#[cfg(feature = "sensor")]
#[inline(always)]
fn nwk_addressed_to_sensor(dst: ShortAddress, own: ShortAddress) -> bool {
    dst == own || matches!(dst.0, 0xFFFC..=0xFFFF)
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn handle_sensor_frame(
    mac: &mut Tlsr8258Mac,
    nwk_security: &mut NwkSecurity,
    aps_security: &mut ApsSecurity,
    frame: &zigbee_mac::primitives::MacFrame,
    nwk_seq: &mut u8,
    aps_seq: &mut u8,
    nwk_frame_counter: &mut u32,
) -> bool {
    let data = frame.as_slice();
    mark_bytes_as_words(DBG_MODE_BASE + 0xC0, data);
    if data.len() >= 8 {
        let raw_fc = u16::from_le_bytes([data[0], data[1]]);
        mark32(
            DBG_MODE_BASE + 0x80,
            raw_fc as u32 | ((data.len() as u32) << 16),
        );
    }
    let Some((nwk_header, nwk_len)) = NwkHeader::parse(data) else {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA01);
        return false;
    };
    if nwk_security.active_key().is_none()
        && !nwk_addressed_to_sensor(nwk_header.dst_addr, mac.short_address)
    {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA0B);
        return false;
    }
    mark32(
        DBG_MODE_BASE + 0x84,
        nwk_len as u32
            | ((nwk_header.frame_control.security as u32) << 8)
            | ((nwk_header.frame_control.frame_type as u32) << 16),
    );
    if nwk_header.frame_control.frame_type != NwkFrameType::Data as u8 {
        let prev = unsafe { core::ptr::read_volatile((DBG_MODE_BASE + 0x70) as *const u32) };
        mark32(DBG_MODE_BASE + 0x70, prev.wrapping_add(1));
        return true;
    }
    {
        let prev = unsafe { core::ptr::read_volatile((DBG_MODE_BASE + 0x74) as *const u32) };
        mark32(DBG_MODE_BASE + 0x74, prev.wrapping_add(1));
    }
    if data.len() >= nwk_len + 4 {
        mark32(
            DBG_MODE_BASE + 0x88,
            u32::from_le_bytes([
                data[nwk_len],
                data[nwk_len + 1],
                data[nwk_len + 2],
                data[nwk_len + 3],
            ]),
        );
    }
    let decrypted;
    let aps_data = if nwk_header.frame_control.security {
        let Some((sec_hdr, sec_len)) = NwkSecurityHeader::parse(&data[nwk_len..]) else {
            mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA04);
            return false;
        };
        mark32(
            DBG_MODE_BASE + 0x8C,
            sec_hdr.key_seq_number as u32 | ((sec_hdr.frame_counter & 0x00FF_FFFF) << 8),
        );
        if let Some(key_entry) = nwk_security.key_by_seq(sec_hdr.key_seq_number).cloned() {
            if !nwk_security.check_frame_counter(&sec_hdr.source_address, sec_hdr.frame_counter) {
                mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA06);
                return false;
            }

            let aad_len = nwk_len + sec_len;
            if data.len() < aad_len {
                mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA07);
                return false;
            }
            let mut aad = [0u8; 64];
            if aad_len > aad.len() {
                mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA08);
                return false;
            }
            aad[..aad_len].copy_from_slice(&data[..aad_len]);
            aad[nwk_len] = (aad[nwk_len] & !0x07) | 0x05;

            let Some(plain) =
                nwk_security.decrypt(&aad[..aad_len], &data[aad_len..], &key_entry.key, &sec_hdr)
            else {
                mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA09);
                return false;
            };
            nwk_security.commit_frame_counter(&sec_hdr.source_address, sec_hdr.frame_counter);
            decrypted = plain;
            mark_bytes_as_words(DBG_MODE_BASE + 0x100, decrypted.as_slice());
            mark32(DBG_MODE_BASE + 0x6C, 0x53E5C001);
        } else {
            let Some(plain) =
                decrypt_initial_nwk_payload(nwk_security, aps_security, data, nwk_len, sec_len, &sec_hdr)
            else {
                mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA05);
                return false;
            };
            decrypted = plain;
            mark_bytes_as_words(DBG_MODE_BASE + 0x100, decrypted.as_slice());
        }
        decrypted.as_slice()
    } else {
        let plain = &data[nwk_len..];
        mark_bytes_as_words(DBG_MODE_BASE + 0x100, plain);
        plain
    };

    let Some((aps_header, aps_len)) = ApsHeader::parse(aps_data) else {
        mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA02);
        return false;
    };
    let decrypted_aps_payload;
    let payload = if aps_header.frame_control.security {
        let Some(plain) = decrypt_sensor_aps_payload(aps_security, aps_data, aps_len) else {
            mark32(DBG_MODE_BASE + 0x6C, 0x53E5AA0A);
            return false;
        };
        decrypted_aps_payload = plain;
        decrypted_aps_payload.as_slice()
    } else {
        &aps_data[aps_len..]
    };

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

    if aps_header.frame_control.frame_type == ApsFrameType::Command as u8 {
        if payload.is_empty() {
            return true;
        }
        match ApsCommandId::from_u8(payload[0]) {
            Some(ApsCommandId::TransportKey) => {
                mark32(DBG_MODE_BASE + 0xBC, 0xA75C0500);
                install_transport_key(&payload[1..], &*mac, nwk_security, nwk_frame_counter);
            }
            Some(ApsCommandId::ConfirmKey) => {
                // Cycle 23: TC accepted our Verify-Key. ZHA should now register the device.
                mark32(DBG_MODE_BASE + 0xBC, 0xA75C0010);
                bump_marker(DBG_MODE_BASE + 0x164);
                // Capture first byte of ConfirmKey payload (status) into low half of marker.
                if payload.len() > 1 {
                    let p = (DBG_MODE_BASE + 0x164) as *mut u32;
                    unsafe {
                        let prev = core::ptr::read_volatile(p);
                        core::ptr::write_volatile(
                            p,
                            (prev & 0xFFFF_0000) | (payload[1] as u32) | ((payload.len() as u32) << 8),
                        );
                    }
                }
            }
            Some(cmd) => {
                mark32(DBG_MODE_BASE + 0xBC, 0xA75C0000 | cmd as u32);
            }
            None => {
                mark32(DBG_MODE_BASE + 0xBC, 0xA75CFF00 | payload[0] as u32);
            }
        }
        return true;
    }

    if nwk_security.active_key().is_none() {
        mark32(DBG_MODE_BASE + 0x90, 0x5A50AA0C);
        return true;
    }

    if aps_header.frame_control.frame_type == ApsFrameType::Data as u8
        && aps_header.frame_control.ack_request
    {
        match send_aps_ack_raw(
            mac,
            nwk_security,
            nwk_frame_counter,
            nwk_header.src_addr,
            &aps_header,
            nwk_seq,
        ) {
            Ok(()) => mark32(DBG_MODE_BASE + 0x98, 0xA9000000 | aps_header.aps_counter as u32),
            Err(_) => mark32(DBG_MODE_BASE + 0x98, 0xA9FF0000 | aps_header.aps_counter as u32),
        }
    }

    if aps_header.profile_id != Some(PROFILE_ZDP)
        || aps_header.dst_endpoint != Some(ZDO_ENDPOINT)
        || payload.is_empty()
    {
        return true;
    }

    let Some(cluster_id) = aps_header.cluster_id else {
        return true;
    };

    if cluster_id == DEVICE_ANNCE {
        mark32(DBG_MODE_BASE + 0x90, 0x5A501300);
        return true;
    }

    let is_broadcast = nwk_header.dst_addr.0 >= 0xFFFC;
    let mut zdp_response = [0u8; 32];
    let Some((rsp_cluster, rsp_len)) =
        build_zdo_response(
            cluster_id,
            payload,
            mac.short_address,
            mac.extended_address,
            is_broadcast,
            &mut zdp_response,
        )
    else {
        mark32(DBG_MODE_BASE + 0x90, 0x5A50AA00 | cluster_id as u32);
        return true;
    };

    mark32(DBG_MODE_BASE + 0x90, 0x5A500000 | cluster_id as u32);
    match send_zdo_response_raw(
        mac,
        nwk_security,
        nwk_frame_counter,
        nwk_header.src_addr,
        rsp_cluster,
        &zdp_response[..rsp_len],
        nwk_seq,
        aps_seq,
    ) {
        Ok(()) => mark32(DBG_MODE_BASE + 0x94, 0x5A500000 | rsp_cluster as u32),
        Err(_) => mark32(DBG_MODE_BASE + 0x94, 0x5A50FFFF),
    }

    true
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn wait_for_transport_key(
    mac: &mut Tlsr8258Mac,
    nwk_security: &mut NwkSecurity,
    aps_security: &mut ApsSecurity,
    nwk_seq: &mut u8,
    aps_seq: &mut u8,
    nwk_frame_counter: &mut u32,
) -> bool {
    const PASSIVE_RX_ATTEMPTS: u8 = 100;
    const PASSIVE_RX_LOOPS: u32 = 240_000;
    const MAX_POLL_ROUNDS: u8 = 8;
    const MAX_EMPTY_POLL_ROUNDS: u8 = 4;

    mark32(DBG_MODE_BASE + 0x28, 0x4B455900);

    for attempt in 0..PASSIVE_RX_ATTEMPTS {
        match mac.receive_raw(PASSIVE_RX_LOOPS) {
            Ok(pkt) => {
                mark32(DBG_MODE_BASE + 0x20, 0x4B455910 | attempt as u32);
                mark32(DBG_MODE_BASE + 0x24, pkt.len as u32);
                if let Some(frame) = zigbee_mac::primitives::MacFrame::from_slice(&pkt.data[..pkt.len])
                {
                    let _ = handle_sensor_frame(
                        mac,
                        nwk_security,
                        aps_security,
                        &frame,
                        nwk_seq,
                        aps_seq,
                        nwk_frame_counter,
                    );
                    if nwk_security.active_key().is_some() {
                        mark32(DBG_MODE_BASE + 0x28, 0x4B455901);
                        return true;
                    }
                }
            }
            Err(_) => {
                mark32(DBG_MODE_BASE + 0x20, 0x4B45FF10 | attempt as u32);
            }
        }
    }

    let parent_addr = mac.coord_short_address;
    let mut empty_rounds = 0u8;
    let mut data_frames = 0u8;

    for round in 0..MAX_POLL_ROUNDS {
        let mut got_data = false;

        match executor::block_on(mac.mlme_poll()) {
            Ok(Some(frame)) => {
                got_data = true;
                data_frames = data_frames.wrapping_add(1);
                mark32(DBG_MODE_BASE + 0x20, 0x4B455A00 | round as u32);
                mark32(DBG_MODE_BASE + 0x24, frame.len() as u32);
                let _ = handle_sensor_frame(
                    mac,
                    nwk_security,
                    aps_security,
                    &frame,
                    nwk_seq,
                    aps_seq,
                    nwk_frame_counter,
                );
                if nwk_security.active_key().is_some() {
                    mark32(DBG_MODE_BASE + 0x28, 0x4B455902);
                    return true;
                }
            }
            Ok(None) => {
                mark32(DBG_MODE_BASE + 0x20, 0x4B450000 | round as u32);
            }
            Err(_) => {
                mark32(DBG_MODE_BASE + 0x20, 0x4B45F000 | round as u32);
            }
        }

        if nwk_security.active_key().is_some() {
            mark32(DBG_MODE_BASE + 0x28, 0x4B455903);
            return true;
        }

        mac.coord_short_address = ShortAddress(0x0000);
        match executor::block_on(mac.mlme_poll()) {
            Ok(Some(frame)) => {
                got_data = true;
                data_frames = data_frames.wrapping_add(1);
                mark32(DBG_MODE_BASE + 0x20, 0x4B455B00 | round as u32);
                mark32(DBG_MODE_BASE + 0x24, frame.len() as u32);
                let _ = handle_sensor_frame(
                    mac,
                    nwk_security,
                    aps_security,
                    &frame,
                    nwk_seq,
                    aps_seq,
                    nwk_frame_counter,
                );
                if nwk_security.active_key().is_some() {
                    mac.coord_short_address = parent_addr;
                    mark32(DBG_MODE_BASE + 0x28, 0x4B455904);
                    return true;
                }
            }
            Ok(None) => {
                mark32(DBG_MODE_BASE + 0x20, 0x4B450100 | round as u32);
            }
            Err(_) => {
                mark32(DBG_MODE_BASE + 0x20, 0x4B45F100 | round as u32);
            }
        }
        mac.coord_short_address = parent_addr;

        if got_data {
            empty_rounds = 0;
        } else {
            empty_rounds = empty_rounds.saturating_add(1);
            if empty_rounds >= MAX_EMPTY_POLL_ROUNDS {
                break;
            }
        }

        executor::block_on(async_timer::delay_ms(100));
    }

    mark32(
        DBG_MODE_BASE + 0x28,
        0x4B45E000 | ((data_frames as u32) << 8) | empty_rounds as u32,
    );
    false
}

#[cfg(feature = "sensor")]
#[inline(never)]
fn sensor_main() -> ! {
    // Temporary sensor-lite path for tc32-stage2-tc32-31.
    // The pure-Rust runtime join path (`ZigbeeDevice::start/tick`) currently
    // trips a tc32 backend codegen bug, so default `sensor` mode uses the
    // already-validated MAC scan/associate/poll flow directly.
    init_our_ext_addr();
    let mut mac = Tlsr8258Mac::new();
    mark32(DBG_MODE_BASE + 0x00, 0x53E50000);
    mark32(
        DBG_MODE_BASE + 0xA0,
        u32::from_le_bytes([
            mac.extended_address[0],
            mac.extended_address[1],
            mac.extended_address[2],
            mac.extended_address[3],
        ]),
    );
    mark32(
        DBG_MODE_BASE + 0xA4,
        u32::from_le_bytes([
            mac.extended_address[4],
            mac.extended_address[5],
            mac.extended_address[6],
            mac.extended_address[7],
        ]),
    );
    let mut rejected_parent: Option<u16> = None;

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

        match executor::block_on(mac.mlme_scan(scan)) {
            Ok(confirm) if !confirm.pan_descriptors.is_empty() => {
                let mut selected_idx: Option<usize> = None;
                let mut selected_score = 0u16;
                for (idx, candidate) in confirm.pan_descriptors.iter().enumerate() {
                    if !candidate.superframe_spec.association_permit
                        || !candidate.zigbee_beacon.end_device_capacity
                    {
                        continue;
                    }
                    if let (Some(rejected), MacAddress::Short(_, addr)) =
                        (rejected_parent, candidate.coord_address)
                    {
                        if addr.0 == rejected {
                            continue;
                        }
                    }
                    let is_short_coord = match candidate.coord_address {
                        MacAddress::Short(_, addr) => addr.0 == 0x0000,
                        MacAddress::Extended(_, _) => false,
                    };
                    let coordinator_bonus = if candidate.superframe_spec.pan_coordinator || is_short_coord {
                        0xF000
                    } else {
                        0
                    };
                    let depth_score =
                        (15u16.saturating_sub(candidate.zigbee_beacon.device_depth as u16)) << 8;
                    let score = coordinator_bonus | depth_score | candidate.lqi as u16;
                    if selected_idx.is_none() || score > selected_score {
                        selected_idx = Some(idx);
                        selected_score = score;
                    }
                }
                let Some(selected_idx) = selected_idx else {
                    board::LED_GREEN.write(false);
                    board::LED_RED.write(true);
                    mark32(DBG_MODE_BASE + 0x10, 0x53E5FF11);
                    continue;
                };
                let desc = &confirm.pan_descriptors[selected_idx];
                board::LED_RED.write(false);
                board::LED_GREEN.write(true);
                mark32(DBG_MODE_BASE + 0x10, 0x53E50010);
                mark32(DBG_MODE_BASE + 0x14, desc.channel as u32);
                let selected_parent = match desc.coord_address {
                    MacAddress::Short(_, addr) => addr.0 as u32,
                    MacAddress::Extended(_, _) => 0xFFFF,
                };
                mark32(
                    DBG_MODE_BASE + 0x98,
                    (confirm.pan_descriptors.len() as u32)
                        | ((selected_idx as u32) << 8)
                        | (selected_parent << 16),
                );
                mark32(
                    DBG_MODE_BASE + 0x9C,
                    (desc.lqi as u32)
                        | ((desc.superframe_spec.pan_coordinator as u32) << 8)
                        | ((desc.superframe_spec.association_permit as u32) << 9)
                        | ((desc.zigbee_beacon.router_capacity as u32) << 10)
                        | ((desc.zigbee_beacon.end_device_capacity as u32) << 11)
                        | ((desc.zigbee_beacon.device_depth as u32) << 16),
                );

                let assoc = MlmeAssociateRequest {
                    channel: desc.channel,
                    coord_address: desc.coord_address,
	                    capability_info: CapabilityInfo {
	                        device_type_ffd: false,
	                        mains_powered: false,
	                        rx_on_when_idle: true,
	                        security_capable: true,
	                        allocate_address: true,
	                    },
                };

                match executor::block_on(mac.mlme_associate(assoc)) {
                    Ok(confirm) if confirm.status == AssociationStatus::Success => {
                        rejected_parent = None;
                        let mut nwk_seq = 0u8;
                        let mut aps_seq = 0u8;
                        let mut zdo_seq = 0u8;
                        let mut nwk_security = NwkSecurity::new();
                        let persisted_frame_counter =
                            load_persisted_network_state(&mut nwk_security);
                        let mut aps_security = ApsSecurity::new();
                        let mut aps_fc_out: u32 = 1; // Cycle 23: outgoing APS frame counter for TC link key
                        let mut nwk_frame_counter = persisted_frame_counter
                            .as_ref()
                            .map(|s| s.frame_counter)
                            .unwrap_or(1);
                        let mut announce_polls = 0u8;
                        let mut announced;
                        board::LED_BLUE.write(true);
                        mark32(DBG_MODE_BASE + 0x18, 0x53E5C000);
                        mark32(DBG_MODE_BASE + 0x1C, confirm.short_address.0 as u32);
                        mark32(
                            DBG_MODE_BASE + 0xA8,
                            mac.short_address.0 as u32 | ((mac.pan_id.0 as u32) << 16),
                        );
                        mark32(DBG_MODE_BASE + 0xAC, mac.coord_short_address.0 as u32);
                        mark32(DBG_MODE_BASE + 0x2C, 0x53E5A000);

                        if persisted_frame_counter.is_none() {
                            mark32(DBG_MODE_BASE + 0x2C, 0x53E5A001);
                            // The trust center starts key transport only after it learns
                            // about the newly associated device. Do not answer interview
                            // requests until the Transport-Key installs the NWK key.
                            let _ = executor::block_on(send_device_annce(
                                &mut mac,
                                &mut nwk_security,
                                &mut nwk_frame_counter,
                                &mut nwk_seq,
                                &mut aps_seq,
                                &mut zdo_seq,
                                ShortAddress(0xFFFD),
                            ));
                            mark32(DBG_MODE_BASE + 0x2C, 0x53E5A002);
                            mark32(DBG_MODE_BASE + 0x28, 0x53E50029);
                            let _ = executor::block_on(send_network_key_request(
                                &mut mac,
                                &mut nwk_seq,
                                &mut aps_seq,
                            ));
                            mark32(DBG_MODE_BASE + 0x2C, 0x53E5A003);

                            if !wait_for_transport_key(
                                &mut mac,
                                &mut nwk_security,
                                &mut aps_security,
                                &mut nwk_seq,
                                &mut aps_seq,
                                &mut nwk_frame_counter,
                            ) {
                                mark32(DBG_MODE_BASE + 0x2C, 0x53E5A004);
                                board::LED_BLUE.write(false);
                                mark32(DBG_MODE_BASE + 0x18, 0x53E5FF40);
                                continue;
                            }
                            mark32(DBG_MODE_BASE + 0x2C, 0x53E5A005);
                        } else {
                            mark32(DBG_MODE_BASE + 0x2C, 0x53E5A006);
                            mark32(DBG_MODE_BASE + 0x28, 0x53E50228);
                        }

                        // ── Cycle 23: post-TK announce + Verify-Key handshake ──
                        // Step 1: broadcast Device_annce (0xFFFD)
                        bump_marker(DBG_MODE_BASE + 0x128);
                        let da_bcast = executor::block_on(send_device_annce(
                            &mut mac,
                            &mut nwk_security,
                            &mut nwk_frame_counter,
                            &mut nwk_seq,
                            &mut aps_seq,
                            &mut zdo_seq,
                            ShortAddress(0xFFFD),
                        ))
                        .is_ok();
                        if da_bcast { bump_marker(DBG_MODE_BASE + 0x12C); }
                        announced = da_bcast;
                        mark32(
                            DBG_MODE_BASE + 0x28,
                            if announced { 0x53E50028 } else { 0x53E5FF28 },
                        );

                        // Step 2: unicast Device_annce to TC (0x0000)
                        bump_marker(DBG_MODE_BASE + 0x130);
                        if executor::block_on(send_device_annce(
                            &mut mac,
                            &mut nwk_security,
                            &mut nwk_frame_counter,
                            &mut nwk_seq,
                            &mut aps_seq,
                            &mut zdo_seq,
                            ShortAddress(0x0000),
                        )).is_ok() {
                            bump_marker(DBG_MODE_BASE + 0x134);
                        }

                        // Step 3: unicast Device_annce to parent (if not TC)
                        let parent = mac.coord_short_address;
                        if parent.0 != 0x0000 && parent.0 != 0xFFFF {
                            bump_marker(DBG_MODE_BASE + 0x138);
                            if executor::block_on(send_device_annce(
                                &mut mac,
                                &mut nwk_security,
                                &mut nwk_frame_counter,
                                &mut nwk_seq,
                                &mut aps_seq,
                                &mut zdo_seq,
                                parent,
                            )).is_ok() {
                                bump_marker(DBG_MODE_BASE + 0x13C);
                            }
                        }

                        // Step 4: APS Verify-Key (single attempt) with the
                        // spec-correct HMAC-MMO_{TC link key}(our_ieee_le) hash
                        // per ZB-3.0 R22 §4.4.11.2 / §B.1.4. Per §4.4.10 ZHA
                        // will not register the device until the VK ->
                        // ConfirmKey handshake completes.
                        {
                            let tc_link_key = *aps_security.default_tc_link_key();
                            let vk_hash = derive_verify_key_hash(&tc_link_key);
                            let res = send_verify_key_aps(
                                &mut mac,
                                &mut nwk_security,
                                &mut aps_security,
                                &mut nwk_frame_counter,
                                &mut nwk_seq,
                                &mut aps_seq,
                                &mut aps_fc_out,
                                &vk_hash,
                            );
                            mark32(DBG_MODE_BASE + 0x158, mac_result_code(&res));
                            // Drain any inbound frames (ConfirmKey may arrive immediately).
                            for _ in 0..16 {
                                if let Ok(indication) =
                                    executor::block_on(mac.mcps_data_indication())
                                {
                                    let _ = handle_sensor_frame(
                                        &mut mac,
                                        &mut nwk_security,
                                        &mut aps_security,
                                        &indication.payload,
                                        &mut nwk_seq,
                                        &mut aps_seq,
                                        &mut nwk_frame_counter,
                                    );
                                } else {
                                    break;
                                }
                            }
                            executor::block_on(async_timer::delay_ms(2000));
                        }

                        loop {
                            announce_polls = announce_polls.wrapping_add(1);
                            for _ in 0..8 {
                                if let Ok(indication) = executor::block_on(mac.mcps_data_indication()) {
                                    mark32(DBG_MODE_BASE + 0x20, 0x53E50022);
                                    mark32(DBG_MODE_BASE + 0x24, indication.payload.len() as u32);
                                    let handled = handle_sensor_frame(
                                        &mut mac,
                                        &mut nwk_security,
                                        &mut aps_security,
                                        &indication.payload,
                                        &mut nwk_seq,
                                        &mut aps_seq,
                                        &mut nwk_frame_counter,
                                    );
                                    mark32(
                                        DBG_MODE_BASE + 0x78,
                                        if handled { 0x53E5BEEF } else { 0x53E50000 },
                                    );
                                }
                            }
                            match executor::block_on(mac.mlme_poll()) {
                                Ok(Some(frame)) => {
                                    mark32(DBG_MODE_BASE + 0x20, 0x53E50021);
                                    mark32(DBG_MODE_BASE + 0x24, frame.len() as u32);
                                    let handled = handle_sensor_frame(
                                        &mut mac,
                                        &mut nwk_security,
                                        &mut aps_security,
                                        &frame,
                                        &mut nwk_seq,
                                        &mut aps_seq,
                                        &mut nwk_frame_counter,
                                    );
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
                            if !announced && nwk_security.active_key().is_some() {
                                announced = executor::block_on(send_device_annce(
                                    &mut mac,
                                    &mut nwk_security,
                                    &mut nwk_frame_counter,
                                    &mut nwk_seq,
                                    &mut aps_seq,
                                    &mut zdo_seq,
                                    ShortAddress(0xFFFD),
                                ))
                                .is_ok();
                                mark32(
                                    DBG_MODE_BASE + 0x28,
                                    if announced { 0x53E50028 } else { 0x53E5FF28 },
                                );
                            }
                            if announce_polls >= SENSOR_ANNOUNCE_PERIOD_POLLS {
                                announce_polls = 0;
                                if nwk_security.active_key().is_some() {
                                    let _ = executor::block_on(send_device_annce(
                                        &mut mac,
                                        &mut nwk_security,
                                        &mut nwk_frame_counter,
                                        &mut nwk_seq,
                                        &mut aps_seq,
                                        &mut zdo_seq,
                                        ShortAddress(0xFFFD),
                                    ));
                                }
                            }
                            executor::block_on(async_timer::delay_ms(SENSOR_POLL_INTERVAL_MS));
                        }
                    }
                    Ok(confirm) => {
                        if let MacAddress::Short(_, addr) = desc.coord_address {
                            rejected_parent = Some(addr.0);
                        }
                        board::LED_BLUE.write(false);
                        mark32(
                            DBG_MODE_BASE + 0x18,
                            0x53E50030 | (confirm.status as u32 & 0xFF),
                        );
                    }
                    Err(_) => {
                        if let MacAddress::Short(_, addr) = desc.coord_address {
                            rejected_parent = Some(addr.0);
                        }
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

        executor::block_on(async_timer::delay_ms(1000));
    }
}

// Kept as source-level history for the legacy local MAC investigation. The
// runtime-sensor entry point below uses the reusable TLSR8258 backend.
#[cfg(any())]
#[inline(never)]
fn runtime_sensor_main() -> ! {
    mark32(DBG_MODE_BASE + 0x30, 0x52540010);
    let mac = Tlsr8258Mac::new();
    mark32(DBG_MODE_BASE + 0x30, 0x52540011);

    static mut DEVICE_STORAGE: MaybeUninit<ZigbeeDevice<Tlsr8258Mac>> = MaybeUninit::uninit();
    static mut BASIC_STORAGE: MaybeUninit<BasicCluster> = MaybeUninit::uninit();
    static mut TEMP_STORAGE: MaybeUninit<TemperatureCluster> = MaybeUninit::uninit();
    static mut HUM_STORAGE: MaybeUninit<HumidityCluster> = MaybeUninit::uninit();
    static mut POWER_STORAGE: MaybeUninit<PowerConfigCluster> = MaybeUninit::uninit();
    static mut IDENTIFY_STORAGE: MaybeUninit<IdentifyCluster> = MaybeUninit::uninit();

    let basic_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(BASIC_STORAGE).cast::<BasicCluster>();
        ptr.write(BasicCluster::new(
            b"Zigbee-RS",
            b"TLSR8258-Sensor",
            b"20260513",
            b"0.1.0",
        ));
        &mut *ptr
    };
    basic_cluster.set_power_source(0x03);
    mark32(DBG_MODE_BASE + 0x30, 0x52540012);

    let temp_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(TEMP_STORAGE).cast::<TemperatureCluster>();
        ptr.write(TemperatureCluster::new(-4000, 12500));
        &mut *ptr
    };
    let hum_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(HUM_STORAGE).cast::<HumidityCluster>();
        ptr.write(HumidityCluster::new(0, 10000));
        &mut *ptr
    };
    let power_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(POWER_STORAGE).cast::<PowerConfigCluster>();
        ptr.write(PowerConfigCluster::new());
        &mut *ptr
    };
    let identify_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(IDENTIFY_STORAGE).cast::<IdentifyCluster>();
        ptr.write(IdentifyCluster::new());
        &mut *ptr
    };

    temp_cluster.set_temperature(2250);
    hum_cluster.set_humidity(5000);
    power_cluster.set_battery_voltage(30);
    power_cluster.set_battery_percentage(200);
    mark32(DBG_MODE_BASE + 0x30, 0x52540013);

    let builder = ZigbeeDevice::builder(mac);
    mark32(DBG_MODE_BASE + 0x30, 0x52540020);
    let builder = builder.device_type(DeviceType::EndDevice);
    mark32(DBG_MODE_BASE + 0x30, 0x52540021);
    let builder = builder.power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        });
    mark32(DBG_MODE_BASE + 0x30, 0x52540022);
    let builder = builder.manufacturer("Zigbee-RS");
    mark32(DBG_MODE_BASE + 0x30, 0x52540023);
    let builder = builder.model("TLSR8258-Sensor");
    mark32(DBG_MODE_BASE + 0x30, 0x52540024);
    let builder = builder.sw_build("0.1.0");
    mark32(DBG_MODE_BASE + 0x30, 0x52540025);
    let builder = builder.channels(zigbee_types::ChannelMask(1 << 15));
    mark32(DBG_MODE_BASE + 0x30, 0x52540026);
    let builder = builder.endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000)
                .cluster_server(0x0003)
                .cluster_server(0x0001)
                .cluster_server(0x0402)
                .cluster_server(0x0405)
        });
    mark32(DBG_MODE_BASE + 0x30, 0x52540027);
    let device = builder.build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) });
    mark32(DBG_MODE_BASE + 0x178, device as *const _ as u32);
    {
        // Dump device.bdb.attributes addresses + raw bytes immediately after build_into
        let bdb_ptr = device.bdb() as *const _ as u32;
        mark32(DBG_MODE_BASE + 0x17C, bdb_ptr);
    }
    mark32(DBG_MODE_BASE + 0x30, 0x52540014);

    // Call start() directly (instead of via UserAction::Join through tick())
    // to keep the await-future state small. Routing start() through tick()
    // produces a future that overflows the 8 KiB SVC stack on TC32 and
    // corrupts bdb.attributes (observed: commissioning_mode flips from 0x02
    // → 0xC0, cap from 0x0B → 0xD1).
    mark32(DBG_MODE_BASE + 0x40, 0x53AA_0100);
    // Pre-sentinel: confirm start() either returns (overwriting this) or is stuck.
    mark32(DBG_MODE_BASE + 0x1A0, 0xBA50_0000);
    // Bounded retry loop around device.start() — early scans can race a
    // late-arriving permit-join broadcast on the coordinator and return
    // SteeringFailure. Retry up to 10 times with 5s sleep between attempts.
    // On Ok: stamp success at MODE+0x1A0 and break.
    // On Err: capture last raw status at MODE+0x1C0, bump attempt count at
    // MODE+0x1C4. InitFailed is not retried (hardware/init bug).
    let mut start_addr: Option<u16> = None;
    let mut attempts: u32 = 0;
    loop {
        attempts = attempts.wrapping_add(1);
        mark32(DBG_MODE_BASE + 0x1C4, attempts);
        match executor::block_on(device.start()) {
            Ok(addr) => {
                mark32(DBG_MODE_BASE + 0x1A0, 0x53AA_0200);
                mark32(DBG_MODE_BASE + 0x1A4, addr as u32);
                start_addr = Some(addr);
                break;
            }
            Err(StartError::InitFailed) => {
                mark32(DBG_MODE_BASE + 0x1C0, 0xFA10_0000);
                mark32(DBG_MODE_BASE + 0x1A0, 0x53AA_FA10);
                break;
            }
            Err(StartError::CommissioningFailed(status)) => {
                let raw = status as u32;
                mark32(DBG_MODE_BASE + 0x1C0, 0xFA00_0000 | raw);
                if attempts >= 10 {
                    mark32(DBG_MODE_BASE + 0x1A0, 0x53AA_FA00 | raw);
                    break;
                }
                // Wait 5s before next attempt so a late permit-join broadcast
                // has time to propagate to the coordinator-side beacon flag.
                delay_ms(5000);
            }
            Err(StartError::PersistenceFailed(_)) => {
                // This legacy start() path has no store, so persistence
                // failures are unreachable unless its lifecycle changes.
                mark32(DBG_MODE_BASE + 0x1C0, 0xFA20_0000);
                mark32(DBG_MODE_BASE + 0x1A0, 0x53AA_FA20);
                break;
            }
        }
    }
    let _ = start_addr;
    mark32(DBG_MODE_BASE + 0x30, 0x52540015);

    // ---- Manual Device_annce fallback ----------------------------------
    // start() reported success (MODE+0x1A0 = 0x53AA_0200) but our previous
    // dump showed BDB+0x1D4 = 0, i.e. the steering FSM's Device_annce TX
    // (steering.rs ≈ line 643) was never executed. Without Device_annce the
    // coordinator never adds us to its routing tables and ZHA never sees us.
    //
    // Retry up to 5× with 2 s spacing; failures here are non-fatal — the
    // tick() loop has its own periodic retry that may succeed later.
    //
    // Markers:
    //   MODE+0x50 = attempt count
    //   MODE+0x54 = 0x53E5_AC00 when broadcast accepted by NWK
    //   MODE+0x58 = 0xDEAD_0000 | ZdpStatus on error (last error wins)
    //   MODE+0x5C = 0x5EE0_AC00 if any attempt succeeded
    {
        let mut dann_attempts: u32 = 0;
        let mut dann_ok = false;
        while dann_attempts < 5 {
            dann_attempts = dann_attempts.wrapping_add(1);
            mark32(DBG_MODE_BASE + 0x50, dann_attempts);
            match executor::block_on(device.send_device_annce()) {
                Ok(()) => {
                    mark32(DBG_MODE_BASE + 0x54, 0x53E5_AC00);
                    dann_ok = true;
                    break;
                }
                Err(e) => {
                    mark32(DBG_MODE_BASE + 0x58, 0xDEAD_0000 | ((e as u8) as u32));
                }
            }
            delay_ms(2000);
        }
        if dann_ok {
            mark32(DBG_MODE_BASE + 0x5C, 0x5EE0_AC00);
        }
    }
    // --------------------------------------------------------------------

    let mut clusters = [
        ClusterRef {
            endpoint: 1,
            cluster: basic_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: temp_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: hum_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: power_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: identify_cluster,
        },
    ];
    mark32(DBG_MODE_BASE + 0x30, 0x52540016);

    let mut tick_counter: u32 = 0;
    let mut first_tick: bool = true;
    loop {
        mark32(DBG_MODE_BASE + 0x30, 0x52540001);
        match executor::block_on(device.tick(1, &mut clusters)) {
            TickResult::Event(StackEvent::Joined {
                short_address,
                channel,
                pan_id,
            }) => {
                board::LED_RED.write(false);
                board::LED_GREEN.write(true);
                board::LED_BLUE.write(true);
                mark32(DBG_MODE_BASE + 0x30, 0x5254C000);
                mark32(DBG_MODE_BASE + 0x34, short_address as u32 | ((pan_id as u32) << 16));
                mark32(DBG_MODE_BASE + 0x38, channel as u32);
                if first_tick { mark32(DBG_MODE_BASE + 0x9C, 0xF1559000); }
            }
            TickResult::Event(StackEvent::CommissioningComplete { success }) => {
                board::LED_GREEN.write(false);
                board::LED_BLUE.write(false);
                board::LED_RED.write(true);
                mark32(
                    DBG_MODE_BASE + 0x30,
                    if success { 0x5254C001 } else { 0x5254FFFF },
                );
                if first_tick {
                    mark32(DBG_MODE_BASE + 0x9C, if success { 0xF155CC01 } else { 0xF155FFFF });
                }
            }
            TickResult::Event(_) => {
                mark32(DBG_MODE_BASE + 0x30, 0x52540002);
                if first_tick { mark32(DBG_MODE_BASE + 0x9C, 0xF1550002); }
            }
            TickResult::RunAgain(ms) => {
                mark32(DBG_MODE_BASE + 0x30, 0x52540003);
                mark32(DBG_MODE_BASE + 0x3C, ms);
                if first_tick { mark32(DBG_MODE_BASE + 0x9C, 0xF1550003); }
            }
            TickResult::Idle => {
                board::LED_GREEN.write(device.is_joined());
                mark32(DBG_MODE_BASE + 0x30, 0x52540004);
                if first_tick { mark32(DBG_MODE_BASE + 0x9C, 0xF1550004); }
            }
        }
        first_tick = false;

        // Poll parent for indirect frames + dispatch any received APS frames
        // through the runtime (ZDO + ZCL ReadAttr / ConfigureReporting / etc.).
        if device.is_joined() {
            for _ in 0..4u8 {
                match executor::block_on(device.poll()) {
                    Ok(Some(ind)) => {
                        mark32(DBG_MODE_BASE + 0x40, 0x52540030);
                        mark32(DBG_MODE_BASE + 0x44, ind.payload.len() as u32);
                        let _ = executor::block_on(
                            device.process_incoming(&ind, &mut clusters),
                        );
                        mark32(DBG_MODE_BASE + 0x40, 0x52540031);
                    }
                    Ok(None) => {
                        mark32(DBG_MODE_BASE + 0x40, 0x52540032);
                        break;
                    }
                    Err(_) => {
                        mark32(DBG_MODE_BASE + 0x40, 0x5254FFFE);
                        break;
                    }
                }
            }
        }

        tick_counter = tick_counter.wrapping_add(1);
        mark32(DBG_MODE_BASE + 0x48, tick_counter);
        // Fast poll while joining/interview, slower once steady-state.
        let delay = if device.is_joined() { 250 } else { 100 };
        executor::block_on(async_timer::delay_ms(delay));
    }
}

#[cfg(all(feature = "runtime-sensor", not(feature = "sensor")))]
fn main_loop() -> ! {
    runtime_sensor::run();
}

#[cfg(not(all(feature = "runtime-sensor", not(feature = "sensor"))))]
fn main_loop() -> ! {
    // Setup RGB LED pins
    board::LED_RED.set_output();
    board::LED_GREEN.set_output();
    board::LED_BLUE.set_output();

    mark32(DBG_BOOT_BASE + 0x00, 0xCAFE0001_u32);
    #[cfg(not(all(feature = "runtime-sensor", not(feature = "sensor"))))]
    mark32(DBG_BOOT_BASE + 0x08, async_timer::now_ticks());

    #[cfg(all(
        not(feature = "sensor"),
        not(feature = "runtime-sensor"),
        not(feature = "diag-assoc"),
        not(feature = "diag-smoke"),
    ))]
    let rx_buf = core::ptr::addr_of_mut!(RF_RX_BUF) as *mut u8;
    #[cfg(not(all(feature = "runtime-sensor", not(feature = "sensor"))))]
    radio::set_rx_dma_config(144);

    board::LED_RED.write(true);
    // TLSR8258 does not clear retained SRAM on soft reset, so reset the
    // example-owned diagnostic window before recording this boot.
    clear_words(DBG_MODE_BASE, 416);
    mark32(DBG_MODE_BASE + 0x00, 0xD1A60000);

    // ── Stack high-water instrumentation ──────────────────────────────────
    // Paint the unused portion of the SVC stack with a sentinel so we can
    // later scan upward from `_svc_stack_bottom` to find the deepest point
    // the stack has ever reached (= first non-sentinel word). The scan and
    // result write happen in `mcps_data_indication` on every entry; that
    // covers all radio-driven call depths (including AES-CCM under BDB).
    //
    // SVC region per memory.x: [0x0084_C000 .. 0x0084_E000) = 8 KiB.
    // We use a local variable's address as an SP proxy — anything below it
    // (lower address) is currently unused and safe to paint.
    unsafe {
        const SVC_STACK_BOTTOM: u32 = 0x0084_B400;
        const STACK_PAINT_PATTERN: u32 = 0xDEAD_BEEF;
        let sp_proxy: u32 = 0;
        let sp_approx = (&sp_proxy as *const u32 as u32) & !3;
        // Leave a 64-byte guard between the paint region and current SP so
        // we never clobber the active call frame.
        let paint_end = sp_approx.saturating_sub(64);
        if paint_end > SVC_STACK_BOTTOM {
            let mut a = SVC_STACK_BOTTOM;
            while a < paint_end {
                core::ptr::write_volatile(a as *mut u32, STACK_PAINT_PATTERN);
                a = a.wrapping_add(4);
            }
        }
        // Record paint extent: MODE+0x1A8 holds paint_end (full 32-bit),
        // MODE+0x1B0 holds sp_approx (full 32-bit). MODE+0x1AC is the
        // running high-water mark in bytes (updated by mcps_data_indication).
        mark32(DBG_MODE_BASE + 0x1A8, paint_end);
        mark32(DBG_MODE_BASE + 0x1AC, 0);
        mark32(DBG_MODE_BASE + 0x1B0, sp_approx);
    }

    #[cfg(feature = "sensor")]
    sensor_main();

    #[cfg(all(
        not(feature = "sensor"),
        not(feature = "runtime-sensor"),
        feature = "diag-smoke",
    ))]
    diag_smoke_main();

    #[cfg(all(
        not(feature = "sensor"),
        not(feature = "runtime-sensor"),
        not(feature = "diag-smoke"),
        feature = "diag-assoc",
    ))]
    executor::block_on(diag_assoc_main());

    #[cfg(all(
        not(feature = "sensor"),
        not(feature = "runtime-sensor"),
        not(feature = "diag-smoke"),
        not(feature = "diag-assoc"),
    ))]
    executor::block_on(diag_beacon_main(rx_buf));

    #[cfg(all(not(feature = "sensor"), not(feature = "runtime-sensor")))]
    loop {}
}

// ─── Hardware smoke harness ─────────────────────────────────────────────────
//
// `diag-smoke` is the on-device validation of the batch-1/2/3 safety fixes.
// It exercises the four behaviours that cannot be checked by `cargo build`:
//
//   1. IEEE-from-flash: read the factory MAC at `0x76000` and expose it via
//      debug SRAM so the operator can confirm it matches the chip's etched
//      address (and that the all-FF/all-00 fallback path is *not* taken on a
//      genuine Telink chip).
//   2. FlashCriticalSection: erase + program + read-back of a scratch sector
//      while Timer0 IRQs are firing at 1 kHz. Success criteria: 10 iterations
//      with no observed corruption (`erase_ok==10`, `program_ok==10`,
//      `readback_ok==10`) and an ever-incrementing Timer0 IRQ counter
//      (proves the IRQ-mask save/restore in FlashCriticalSection actually
//      restored, not just left them masked).
//   3. Timer0 alarm path: `arm_timer0_alarm` + `blocking_wait_for_alarm`
//      complete inside the expected tick budget every iteration.
//   4. `.ram_code` callability under load: `flash_erase_sector` /
//      `flash_page_program` are `.ram_code` routines; if boot-ROM preload
//      ever silently fails they will hang on the first iteration. Reaching
//      the "done" marker at all is the success signal for `ramcode-validate`.
//
// Debug-SRAM map (`DBG_MODE_BASE = 0x0084F100`):
//   +0x00  marker `0xD1A6_0000` (set before this fn runs)
//   +0x10  marker `0x5_03E_0001` — entered diag_smoke_main
//   +0x14  IEEE low  (LE u32 of bytes [0..4])
//   +0x18  IEEE high (LE u32 of bytes [4..8])
//   +0x1C  IEEE source (0x1A1A_FFFF=all-FF fallback, 0x0000_0000=all-00 fallback,
//          else 0xFAC7_F1A5=factory)
//   +0x20  iteration counter (u32, increments per loop)
//   +0x24  erase_ok count
//   +0x28  program_ok count
//   +0x2C  readback_ok count
//   +0x30  timer_irq_count (snapshot at end of each iteration)
//   +0x40  marker `0x5_03E_DEAD` — fn returned (should never happen — main
//          loops forever).
#[cfg(feature = "diag-smoke")]
fn diag_smoke_main() -> ! {
    use core::sync::atomic::Ordering;

    const SMOKE_BASE: u32 = DBG_MODE_BASE + 0x10;
    const SMOKE_SECTOR: u32 = 0x0007_E000;
    const SMOKE_PATTERN: [u8; 16] = [
        0xA5, 0x5A, 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE,
        0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    ];

    init_our_ext_addr();
    init_timer0();

    let ieee = our_ext_addr();
    mark32(SMOKE_BASE + 0x00, 0x503E_0001);
    mark32(
        SMOKE_BASE + 0x04,
        u32::from_le_bytes([ieee[0], ieee[1], ieee[2], ieee[3]]),
    );
    mark32(
        SMOKE_BASE + 0x08,
        u32::from_le_bytes([ieee[4], ieee[5], ieee[6], ieee[7]]),
    );
    let all_ff = ieee.iter().all(|b| *b == 0xFF);
    let all_00 = ieee.iter().all(|b| *b == 0x00);
    let src = if all_ff {
        0x1A1A_FFFFu32
    } else if all_00 {
        0x0000_0000u32
    } else if ieee == OUR_EXT_ADDR_FALLBACK {
        0xFA11_BAC4u32
    } else {
        0xFAC7_F1A5u32
    };
    mark32(SMOKE_BASE + 0x0C, src);

    // Enable IRQs globally so Timer0 can fire during flash ops.
    unsafe { core::ptr::write_volatile(0x800643 as *mut u8, 1) };

    let mut iter: u32 = 0;
    let mut erase_ok: u32 = 0;
    let mut program_ok: u32 = 0;
    let mut readback_ok: u32 = 0;

    loop {
        iter = iter.wrapping_add(1);
        mark32(SMOKE_BASE + 0x10, iter);

        // Arm a 1 ms tick so Timer0 IRQs fire during the flash sequence.
        arm_timer0_alarm(24_000);

        {
            let _cs = FlashCriticalSection::enter();
            flash_erase_sector(SMOKE_SECTOR);
        }
        // Read back: expect all 0xFF after erase.
        let mut buf = [0u8; 16];
        {
            let _cs = FlashCriticalSection::enter();
            flash_read_data(SMOKE_SECTOR, &mut buf);
        }
        if buf.iter().all(|b| *b == 0xFF) {
            erase_ok = erase_ok.wrapping_add(1);
        }
        mark32(SMOKE_BASE + 0x14, erase_ok);

        // Program a known pattern and read it back.
        let mut payload = [0u8; 16];
        payload.copy_from_slice(&SMOKE_PATTERN);
        payload[0] ^= (iter & 0xFF) as u8;
        {
            let _cs = FlashCriticalSection::enter();
            flash_page_program(SMOKE_SECTOR, &payload);
        }
        program_ok = program_ok.wrapping_add(1);
        mark32(SMOKE_BASE + 0x18, program_ok);

        let mut rb = [0u8; 16];
        {
            let _cs = FlashCriticalSection::enter();
            flash_read_data(SMOKE_SECTOR, &mut rb);
        }
        if rb == payload {
            readback_ok = readback_ok.wrapping_add(1);
        }
        mark32(SMOKE_BASE + 0x1C, readback_ok);

        // Wait for the Timer0 alarm we armed at the top of the iteration.
        // Success: IRQ count must have advanced — proves FlashCriticalSection
        // re-enabled REG_IRQ_EN on drop.
        blocking_wait_for_alarm();
        let now_irq = TIMER0_IRQ_COUNT.load(Ordering::Relaxed);
        mark32(SMOKE_BASE + 0x20, now_irq);
        // Cheap heartbeat — toggling LED proves we never wedged inside flash.
        if (iter & 0x01) == 0 {
            board::LED_BLUE.write(true);
        } else {
            board::LED_BLUE.write(false);
        }
    }
}

#[cfg(feature = "diag-smoke")]
#[allow(dead_code)]
fn diag_smoke_unused_marker() {}
