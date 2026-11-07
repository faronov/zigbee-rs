//! TOP/CPU/modem power-down light sleep (`SleepMode::TopPowerDown`).
//!
//! Port of ESP-IDF v5.5.1 `esp_sleep_cpu_retention` (`lowpower/port/esp32h2/
//! sleep_cpu.c`, `sleep_cpu_asm.S`) and of the H2 software-triggered REGDMA
//! system retention (`SOC_PM_RETENTION_SW_TRIGGER_REGDMA`,
//! `soc/esp32h2/system_retention_periph.c`, `sleep_clock.c`, BTBB and
//! 802.15.4 MAC lists), with the CPU doing the copies.
//!
//! Sequence (all of it in `.rwtext`, the I-cache suspended):
//!
//! 1. save CPU-domain frames (PLIC, CLINT, INTPRI, cache wrap-around), the
//!    non-critical CSRs and every TOP/modem region into [`FRAMES`];
//! 2. point `LP_AON_STORE8` (`RTC_SLEEP_WAKE_STUB_ADDR_REG`) at
//!    `esp32h2_sleep_cpu_restore` and save the critical CPU frame;
//! 3. request sleep. A reject returns here with nothing lost. A wake resets
//!    the CPU; the ROM jumps to the restore stub, which reloads the critical
//!    frame and returns from the save call a second time with
//!    `pmufunc & 3 == 3`;
//! 4. on that wake: PCR (at XTAL), MWDT0/1 flash-boot off, modem clocks,
//!    `pmu_sleep_finish` bias fix, analog, CPU clock (BBPLL), SPI1/SPI0,
//!    IO_MUX/GPIO, interrupt matrix, HP_SYSTEM, TEE (+ M4 mode 0), HP_APM,
//!    SYSTIMER, BTBB and the 802.15.4 MAC, the USB-Serial-JTAG pad word,
//!    CPU frames and CSRs, then the
//!    I-cache is invalidated and resumed.
//!
//! Conscious differences from IDF (see `esp32h2-sleep/README.md`): PCR is
//! saved after the switch to XTAL (so restoring it cannot glitch the bus
//! clock), the 802.15.4 copy skips `IEEE802154_COMMAND`, a SYSTIMER
//! comparator's period mode is toggled only when it was set, and the whole
//! restore runs before the first flash access instead of after the clock
//! restore.

use core::ptr::{read_volatile, write_volatile};

use super::restore_sleep_bias;
use super::{ClockConfig, SavedAnalog, latch_bus_clock, request_sleep, restore_clock};
use crate::retention::{
    CPU_WORDS, CRITICAL_FRAME_WORDS, MODEM_CLOCK_WORDS, NONCRITICAL_CSR_WORDS, PCR_WORDS,
    RADIO_WORDS, SPI_MEM_WORDS, SYSTEM_WORDS, SYSTIMER_BASE, SYSTIMER_WORDS,
};

/// `LP_AON_STORE8_REG`: the ROM jumps here after a CPU power-down wake.
const WAKE_STUB_ADDR_REG: usize = 0x600B_1020;
/// `LP_AON_EXT_WAKEUP_CNTL_REG`.
const EXT_WAKEUP_CNTL_REG: usize = 0x600B_1040;
/// TIMG0/TIMG1 MWDT: `TIMG_WDTCONFIG0_REG`, `TIMG_WDTWPROTECT_REG`.
const TIMG0_WDTCONFIG0_REG: usize = 0x6000_9048;
const TIMG0_WDTWPROTECT_REG: usize = 0x6000_9064;
const TIMG1_OFFSET: usize = 0x1000;
/// `USB_SERIAL_JTAG_CONF0_REG`: pad enable, D+ pull-up and PHY selection.
/// USB-Serial-JTAG sits in TOP; IDF backs this word up around a TOP
/// power-down (`sleep_console_usj_pad_backup/restore`) so a disabled pad
/// stays disabled.
const USB_SERIAL_JTAG_CONF0_REG: usize = 0x6000_F018;
const WDT_WKEY: u32 = 0x50D8_3AA1;
const WDT_FLASHBOOT_MOD_EN: u32 = 1 << 14;
/// `TEE_M4_MODE_CTRL_REG` (IDF `tee_apm_highpri_regs_retention`).
const TEE_M4_MODE_CTRL_REG: usize = 0x6009_8010;

/// Retention buffers. Plain `.bss` in the application RAM segment, below
/// the ROM's wake stack in `dram2` (checked by
/// `tools/esp32h2-sleep-ram-check.py`).
#[repr(C, align(4))]
struct Frames {
    pcr: [u32; PCR_WORDS],
    spi_mem: [u32; SPI_MEM_WORDS],
    system: [u32; SYSTEM_WORDS],
    modem_clock: [u32; MODEM_CLOCK_WORDS],
    radio: [u32; RADIO_WORDS],
    systimer: [u32; SYSTIMER_WORDS],
    cpu: [u32; CPU_WORDS],
    csr: [u32; NONCRITICAL_CSR_WORDS],
}

static mut FRAMES: Frames = Frames {
    pcr: [0; PCR_WORDS],
    spi_mem: [0; SPI_MEM_WORDS],
    system: [0; SYSTEM_WORDS],
    modem_clock: [0; MODEM_CLOCK_WORDS],
    radio: [0; RADIO_WORDS],
    systimer: [0; SYSTIMER_WORDS],
    cpu: [0; CPU_WORDS],
    csr: [0; NONCRITICAL_CSR_WORDS],
};

/// Critical CPU frame; word offsets are fixed by the assembly below.
#[unsafe(no_mangle)]
static mut ESP32H2_SLEEP_CPU_FRAME: [u32; CRITICAL_FRAME_WORDS] = [0; CRITICAL_FRAME_WORDS];

/// Word index of `pmufunc` in [`ESP32H2_SLEEP_CPU_FRAME`].
const PMUFUNC: usize = 38;

// Frame layout (byte offsets): ra 0, sp 4, gp 8, tp 12, t0 16, t1 20, t2 24,
// s0 28, s1 32, a0..a7 36..64, s2..s11 68..104, t3..t6 108..120,
// mstatus 124, mtvec 128, mcause 132, mtval 136, mie 140, mip 144,
// mepc 148, pmufunc 152.
core::arch::global_asm!(
    r#"
    .section .rwtext.esp32h2_sleep_cpu, "ax", @progbits
    .global esp32h2_sleep_cpu_save
    .type esp32h2_sleep_cpu_save, @function
    .balign 4
esp32h2_sleep_cpu_save:
    csrw    mscratch, t0
    la      t0, ESP32H2_SLEEP_CPU_FRAME
    sw      ra, 0(t0)
    sw      sp, 4(t0)
    sw      gp, 8(t0)
    sw      tp, 12(t0)
    sw      t1, 20(t0)
    sw      t2, 24(t0)
    sw      s0, 28(t0)
    sw      s1, 32(t0)
    mv      a0, t0
    sw      a0, 36(t0)
    sw      a1, 40(t0)
    sw      a2, 44(t0)
    sw      a3, 48(t0)
    sw      a4, 52(t0)
    sw      a5, 56(t0)
    sw      a6, 60(t0)
    sw      a7, 64(t0)
    sw      s2, 68(t0)
    sw      s3, 72(t0)
    sw      s4, 76(t0)
    sw      s5, 80(t0)
    sw      s6, 84(t0)
    sw      s7, 88(t0)
    sw      s8, 92(t0)
    sw      s9, 96(t0)
    sw      s10, 100(t0)
    sw      s11, 104(t0)
    sw      t3, 108(t0)
    sw      t4, 112(t0)
    sw      t5, 116(t0)
    sw      t6, 120(t0)
    csrr    t1, mstatus
    sw      t1, 124(t0)
    csrr    t2, mtvec
    sw      t2, 128(t0)
    csrr    t3, mcause
    sw      t3, 132(t0)
    csrr    t1, mtval
    sw      t1, 136(t0)
    csrr    t2, mie
    sw      t2, 140(t0)
    csrr    t3, mip
    sw      t3, 144(t0)
    csrr    t1, mepc
    sw      t1, 148(t0)
    /* pmufunc[1:0] = 1: going to sleep */
    li      t1, ~0x3
    lw      t2, 152(t0)
    and     t2, t1, t2
    ori     t2, t2, 0x1
    sw      t2, 152(t0)
    mv      t3, t0
    csrr    t0, mscratch
    lw      t1, 20(t3)
    lw      t2, 24(t3)
    lw      t3, 108(t3)
    ret
    .size esp32h2_sleep_cpu_save, . - esp32h2_sleep_cpu_save

    .global esp32h2_sleep_cpu_restore
    .type esp32h2_sleep_cpu_restore, @function
    .balign 4
esp32h2_sleep_cpu_restore:
    la      t0, ESP32H2_SLEEP_CPU_FRAME
    /* pmufunc[1:0] = 3: woken from a CPU power-down */
    lw      t1, 152(t0)
    ori     t1, t1, 0x3
    sw      t1, 152(t0)
    lw      t2, 148(t0)
    csrw    mepc, t2
    lw      t3, 144(t0)
    csrw    mip, t3
    lw      t1, 140(t0)
    csrw    mie, t1
    lw      t2, 124(t0)
    csrw    mstatus, t2
    lw      t3, 128(t0)
    csrw    mtvec, t3
    lw      t1, 132(t0)
    csrw    mcause, t1
    lw      t2, 136(t0)
    csrw    mtval, t2
    lw      t6, 120(t0)
    lw      t5, 116(t0)
    lw      t4, 112(t0)
    lw      t3, 108(t0)
    lw      s11, 104(t0)
    lw      s10, 100(t0)
    lw      s9, 96(t0)
    lw      s8, 92(t0)
    lw      s7, 88(t0)
    lw      s6, 84(t0)
    lw      s5, 80(t0)
    lw      s4, 76(t0)
    lw      s3, 72(t0)
    lw      s2, 68(t0)
    lw      a7, 64(t0)
    lw      a6, 60(t0)
    lw      a5, 56(t0)
    lw      a4, 52(t0)
    lw      a3, 48(t0)
    lw      a2, 44(t0)
    lw      a1, 40(t0)
    lw      a0, 36(t0)
    lw      s1, 32(t0)
    lw      s0, 28(t0)
    lw      t2, 24(t0)
    lw      t1, 20(t0)
    lw      tp, 12(t0)
    lw      gp, 8(t0)
    lw      sp, 4(t0)
    lw      ra, 0(t0)
    lw      t0, 16(t0)
    ret
    .size esp32h2_sleep_cpu_restore, . - esp32h2_sleep_cpu_restore
"#
);

unsafe extern "C" {
    /// Saves the critical frame; returns its address. Returns a second time,
    /// through `esp32h2_sleep_cpu_restore`, after a CPU power-down wake.
    fn esp32h2_sleep_cpu_save() -> *mut u32;
    fn esp32h2_sleep_cpu_restore();
}

// ---------------------------------------------------------------------------
// ROM cache control (`esp32h2.rom.ld`)

#[inline(always)]
fn cache_suspend() -> u32 {
    let f: unsafe extern "C" fn() -> u32 = unsafe { core::mem::transmute::<usize, _>(0x4000_066C) };
    unsafe { f() }
}

#[inline(always)]
fn cache_resume(autoload: u32) {
    let f: unsafe extern "C" fn(u32) = unsafe { core::mem::transmute::<usize, _>(0x4000_0670) };
    unsafe { f(autoload) }
}

#[inline(always)]
fn cache_invalidate_all() {
    let f: unsafe extern "C" fn() = unsafe { core::mem::transmute::<usize, _>(0x4000_0620) };
    unsafe { f() }
}

// ---------------------------------------------------------------------------
// Register copies. Volatile accesses, so they are never turned into a
// `memcpy` call (which would live in flash).

#[inline(always)]
fn reg_read(addr: usize) -> u32 {
    unsafe { read_volatile(addr as *const u32) }
}

#[inline(always)]
fn reg_write(addr: usize, value: u32) {
    unsafe { write_volatile(addr as *mut u32, value) }
}

macro_rules! save_regions {
    (($buf:expr), [$(($name:ident, $base:expr, $words:expr)),* $(,)?]) => {{
        let buf: *mut u32 = $buf;
        let mut offset = 0usize;
        $(
            let mut i = 0usize;
            while i < $words {
                unsafe {
                    write_volatile(
                        buf.add(offset.wrapping_add(i)),
                        reg_read(($base as usize).wrapping_add(4 * i)),
                    );
                }
                i = i.wrapping_add(1);
            }
            offset = offset.wrapping_add($words);
        )*
        let _ = offset;
    }};
}

macro_rules! restore_regions {
    (($buf:expr), [$(($name:ident, $base:expr, $words:expr)),* $(,)?]) => {{
        let buf: *const u32 = $buf;
        let mut offset = 0usize;
        $(
            let mut i = 0usize;
            while i < $words {
                reg_write(
                    ($base as usize).wrapping_add(4 * i),
                    unsafe { read_volatile(buf.add(offset.wrapping_add(i))) },
                );
                i = i.wrapping_add(1);
            }
            offset = offset.wrapping_add($words);
        )*
        let _ = offset;
    }};
}

/// Pointer to the first word of a [`FRAMES`] field.
macro_rules! frame {
    ($field:ident) => {
        unsafe { (&raw mut FRAMES.$field).cast::<u32>() }
    };
}

// ---------------------------------------------------------------------------
// Non-critical CSRs (`rv_core_noncritical_regs_save/restore`)

macro_rules! csr_read {
    ($csr:literal) => {{
        let value: u32;
        unsafe { core::arch::asm!("csrr {0}, {csr}", out(reg) value, csr = const $csr) };
        value
    }};
}

macro_rules! csr_write {
    ($csr:literal, $value:expr) => {{
        let value: u32 = $value;
        unsafe { core::arch::asm!("csrw {csr}, {0}", in(reg) value, csr = const $csr) };
    }};
}

/// CSR numbers in IDF's restore order: mscratch, mideleg, misa, tselect,
/// tdata1, tdata2, tcontrol, pmpaddr0..15, pmpcfg0..3, PMA address 0..15
/// (0xBD0), PMA config 0..15 (0xBC0), utvec, ustatus, uepc, ucause,
/// mpcer, mpcmr, mpccr, cpu_testbus_ctrl, upcer, upcmr, upccr, ugpio_oen,
/// ugpio_in, ugpio_out.
macro_rules! noncritical_csrs {
    ($cb:ident, $arg:tt) => {
        $cb!(
            $arg,
            [
                0x340, 0x303, 0x301, 0x7A0, 0x7A1, 0x7A2, 0x7A5, 0x3B0, 0x3B1, 0x3B2, 0x3B3, 0x3B4,
                0x3B5, 0x3B6, 0x3B7, 0x3B8, 0x3B9, 0x3BA, 0x3BB, 0x3BC, 0x3BD, 0x3BE, 0x3BF, 0x3A0,
                0x3A1, 0x3A2, 0x3A3, 0xBD0, 0xBD1, 0xBD2, 0xBD3, 0xBD4, 0xBD5, 0xBD6, 0xBD7, 0xBD8,
                0xBD9, 0xBDA, 0xBDB, 0xBDC, 0xBDD, 0xBDE, 0xBDF, 0xBC0, 0xBC1, 0xBC2, 0xBC3, 0xBC4,
                0xBC5, 0xBC6, 0xBC7, 0xBC8, 0xBC9, 0xBCA, 0xBCB, 0xBCC, 0xBCD, 0xBCE, 0xBCF, 0x005,
                0x000, 0x041, 0x042, 0x7E0, 0x7E1, 0x7E2, 0x7E3, 0x800, 0x801, 0x802, 0x803, 0x804,
                0x805,
            ]
        )
    };
}

macro_rules! save_csrs {
    (($buf:expr), [$($csr:literal),* $(,)?]) => {{
        let buf: *mut u32 = $buf;
        let mut i = 0usize;
        $(
            unsafe { write_volatile(buf.add(i), csr_read!($csr)) };
            i = i.wrapping_add(1);
        )*
        let _ = i;
    }};
}

macro_rules! restore_csrs {
    (($buf:expr), [$($csr:literal),* $(,)?]) => {{
        let buf: *const u32 = $buf;
        let mut i = 0usize;
        $(
            csr_write!($csr, unsafe { read_volatile(buf.add(i)) });
            i = i.wrapping_add(1);
        )*
        let _ = i;
    }};
}

// ---------------------------------------------------------------------------
// SYSTIMER (`systimer_regs_retention`)

// Register addresses are computed, never indexed from a `const` table
// (which could be placed in flash `.rodata`).
const ST_CONF: usize = SYSTIMER_BASE as usize;
/// `SYSTIMER_UNITn_OP_REG`.
const fn st_unit_op(unit: usize) -> usize {
    ST_CONF + 0x04 + 4 * unit
}
/// `SYSTIMER_UNITn_LOAD_HI_REG` (LO at +4).
const fn st_unit_load_hi(unit: usize) -> usize {
    ST_CONF + 0x0C + 8 * unit
}
/// `SYSTIMER_UNITn_VALUE_HI_REG` (LO at +4).
const fn st_unit_value_hi(unit: usize) -> usize {
    ST_CONF + 0x40 + 8 * unit
}
/// `SYSTIMER_UNITn_LOAD_REG`.
const fn st_unit_load(unit: usize) -> usize {
    ST_CONF + 0x5C + 4 * unit
}
const ST_TARGET0_HI: usize = ST_CONF + 0x1C;
const ST_TARGET_CONF: usize = ST_CONF + 0x34;
const ST_COMP_LOAD: usize = ST_CONF + 0x50;
const ST_INT_ENA: usize = ST_CONF + 0x64;
const ST_UNIT_UPDATE: u32 = 1 << 30;
const ST_UNIT_VALUE_VALID: u32 = 1 << 29;
const ST_PERIOD_MODE: u32 = 1 << 30;
/// Frame word indices: CONF, unit0 hi/lo, unit1 hi/lo, TARGET0_HI..
/// TARGET2_CONF (9), INT_ENA.
const ST_F_CONF: usize = 0;
const ST_F_UNIT: usize = 1;
const ST_F_TARGET: usize = 5;
const ST_F_INT_ENA: usize = 14;

#[inline(always)]
fn systimer_save(buf: *mut u32) {
    let put = |index: usize, value: u32| unsafe { write_volatile(buf.add(index), value) };
    put(ST_F_CONF, reg_read(ST_CONF));
    // Latch each unit (UPDATE, wait VALUE_VALID), then copy hi/lo.
    let mut unit = 0;
    while unit < 2 {
        reg_write(st_unit_op(unit), ST_UNIT_UPDATE);
        while reg_read(st_unit_op(unit)) & ST_UNIT_VALUE_VALID == 0 {}
        put(ST_F_UNIT + 2 * unit, reg_read(st_unit_value_hi(unit)));
        put(
            ST_F_UNIT + 2 * unit + 1,
            reg_read(st_unit_value_hi(unit) + 4),
        );
        unit += 1;
    }
    let mut i = 0;
    while i < 9 {
        put(ST_F_TARGET + i, reg_read(ST_TARGET0_HI + 4 * i));
        i += 1;
    }
    put(ST_F_INT_ENA, reg_read(ST_INT_ENA));
}

#[inline(always)]
fn systimer_restore(buf: *const u32) {
    let get = |index: usize| unsafe { read_volatile(buf.add(index)) };
    // Units: LOAD_HI/LO from the saved values, then UNIT_LOAD.
    let mut unit = 0;
    while unit < 2 {
        reg_write(st_unit_load_hi(unit), get(ST_F_UNIT + 2 * unit));
        reg_write(st_unit_load_hi(unit) + 4, get(ST_F_UNIT + 2 * unit + 1));
        reg_write(st_unit_load(unit), 1);
        unit += 1;
    }
    // Targets, periods and their CONF words.
    let mut i = 0;
    while i < 9 {
        reg_write(ST_TARGET0_HI + 4 * i, get(ST_F_TARGET + i));
        i += 1;
    }
    // COMPn_LOAD latches each comparator's target.
    let mut comp = 0;
    while comp < 3 {
        reg_write(ST_COMP_LOAD + 4 * comp, 1);
        comp += 1;
    }
    // A periodic comparator restarts its period on a 0 -> 1 toggle.
    let mut comp = 0;
    while comp < 3 {
        let conf = get(ST_F_TARGET + 6 + comp);
        if conf & ST_PERIOD_MODE != 0 {
            let addr = ST_TARGET_CONF + 4 * comp;
            reg_write(addr, conf & !ST_PERIOD_MODE);
            reg_write(addr, conf);
        }
        comp += 1;
    }
    reg_write(ST_CONF, get(ST_F_CONF));
    reg_write(ST_INT_ENA, get(ST_F_INT_ENA));
}

// ---------------------------------------------------------------------------
// Entry points

/// `ext1_wakeup_prepare`: LP IO selection and level (pads configured by the
/// caller as LP IO inputs). An empty selection disables EXT1.
pub(super) fn configure_ext1(sel: u8, high: bool) {
    const STATUS_CLR: u32 = 1 << 14;
    const SEL_SHIFT: u32 = 15;
    const LV_SHIFT: u32 = 23;
    let fields = (0xFF << SEL_SHIFT) | (0xFF << LV_SHIFT);
    let level = if high { u32::from(sel) } else { 0 };
    let value = (reg_read(EXT_WAKEUP_CNTL_REG) & !fields)
        | (u32::from(sel) << SEL_SHIFT)
        | (level << LV_SHIFT);
    reg_write(EXT_WAKEUP_CNTL_REG, value | STATUS_CLR);
}

pub(super) struct Outcome {
    pub rejected: bool,
    pub powered_down: bool,
}

#[inline(always)]
fn disable_flashboot_wdt(timg_offset: usize) {
    reg_write(TIMG0_WDTWPROTECT_REG + timg_offset, WDT_WKEY);
    reg_write(
        TIMG0_WDTCONFIG0_REG + timg_offset,
        reg_read(TIMG0_WDTCONFIG0_REG + timg_offset) & !WDT_FLASHBOOT_MOD_EN,
    );
    reg_write(TIMG0_WDTWPROTECT_REG + timg_offset, 0);
}

/// `esp_sleep_cpu_retention` + system retention around `pmu_sleep_start`.
///
/// # Safety
///
/// Called by `prepare_and_sleep` with the PMU configured for a TOP/CPU/
/// modem power-down, the CPU at XTAL and interrupts masked by the caller's
/// critical section.
#[unsafe(link_section = ".rwtext")]
#[inline(never)]
pub(super) unsafe fn sleep_top_pd(
    wake: u32,
    reject: u32,
    saved: Option<SavedAnalog>,
    clock: ClockConfig,
) -> Outcome {
    let mstatus: u32;
    unsafe { core::arch::asm!("csrrci {0}, mstatus, 8", out(reg) mstatus) };
    // `suspend_cache`: no flash access from here until `cache_resume`.
    let autoload = cache_suspend();

    crate::__h2_cpu_regions!(save_regions, (frame!(cpu)));
    noncritical_csrs!(save_csrs, (frame!(csr)));
    crate::__h2_pcr_regions!(save_regions, (frame!(pcr)));
    crate::__h2_spi_mem_regions!(save_regions, (frame!(spi_mem)));
    crate::__h2_system_regions!(save_regions, (frame!(system)));
    crate::__h2_modem_clock_regions!(save_regions, (frame!(modem_clock)));
    crate::__h2_radio_regions!(save_regions, (frame!(radio)));
    systimer_save(frame!(systimer));
    let usj_conf0 = reg_read(USB_SERIAL_JTAG_CONF0_REG);

    reg_write(
        WAKE_STUB_ADDR_REG,
        esp32h2_sleep_cpu_restore as unsafe extern "C" fn() as usize as u32,
    );
    let frame = unsafe { esp32h2_sleep_cpu_save() };
    let pmufunc = unsafe { read_volatile(frame.add(PMUFUNC)) };

    let outcome = if pmufunc & 0x3 == 0x1 {
        // Going to sleep. Returns only on a reject (or a wake that kept the
        // CPU powered); nothing was lost.
        let rejected = request_sleep(false, wake, reject);
        restore_sleep_bias();
        if let Some(saved) = saved {
            saved.restore();
        }
        restore_clock(clock);
        Outcome {
            rejected,
            powered_down: false,
        }
    } else {
        // Woken through the ROM and the restore stub, at XTAL.
        // CPU domain first, as IDF's `esp_sleep_cpu_retention` does before
        // `pmu_sleep_finish`: the power-down reset `mpcer`/`mpcmr`, which
        // stops the cycle counter the ROM `ets_delay_us` spins on (the
        // clock restore below would never return).
        noncritical_csrs!(restore_csrs, (frame!(csr)));
        crate::__h2_cpu_regions!(restore_regions, (frame!(cpu)));
        crate::__h2_pcr_regions!(restore_regions, (frame!(pcr)));
        latch_bus_clock();
        // `misc_modules_wake_prepare`: the flash-boot MWDT comes back armed.
        // IDF clears it on TIMG0 only; TIMG1 is cleared too (conscious
        // difference: harmless, and this crate cannot know TIMG1's reset
        // value was never armed).
        disable_flashboot_wdt(0);
        disable_flashboot_wdt(TIMG1_OFFSET);
        // Modem clocks before the analog I2C master and the radio.
        crate::__h2_modem_clock_regions!(restore_regions, (frame!(modem_clock)));
        // `pmu_sleep_finish`.
        restore_sleep_bias();
        if let Some(saved) = saved {
            saved.restore();
        }
        restore_clock(clock);
        // Flash controller; the MMU table itself is retained.
        crate::__h2_spi_mem_regions!(restore_regions, (frame!(spi_mem)));
        crate::__h2_system_regions!(restore_regions, (frame!(system)));
        reg_write(TEE_M4_MODE_CTRL_REG, 0);
        systimer_restore(frame!(systimer));
        crate::__h2_radio_regions!(restore_regions, (frame!(radio)));
        reg_write(USB_SERIAL_JTAG_CONF0_REG, usj_conf0);
        Outcome {
            rejected: false,
            powered_down: true,
        }
    };

    // A reject (or a wake with the CPU powered) lost nothing; a power-down
    // wake restored the CPU domain on entry to its branch.
    if outcome.powered_down {
        cache_invalidate_all();
    }
    cache_resume(autoload);
    reg_write(WAKE_STUB_ADDR_REG, 0);
    if mstatus & 8 != 0 {
        unsafe { core::arch::asm!("csrsi mstatus, 8") };
    }
    outcome
}
