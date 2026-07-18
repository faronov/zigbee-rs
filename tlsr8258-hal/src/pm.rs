//! TLSR8258 power management: deep-sleep-retention scratch registers **and**
//! a pure-Rust RC-32k, timer-wake, SRAM-`LOW32K`-retention suspend path.
//!
//! # Provenance / evidence tiers
//!
//! Every hardware sequence below is transcribed from the relocation-aware
//! disassembly of Telink's own `libdrivers_8258.a` (never linked, never
//! copied as C) plus the open-source boot primitive
//! `platform/boot/8258/cstartup_8258.S`. Each item is tagged with the
//! confidence tier used when reproducing it:
//!
//! * **T1 (bit-exact)** — the register address, value, and ordering are read
//!   directly out of the object disassembly (`objdump -d -r`), including the
//!   `.data` initialisers of the vendor's tuning globals
//!   (`g_pm_early_wakeup_time_us = {0x0555,0x044c,0x04d8,0x06e5}`,
//!   `g_pm_r_delay_us = {0x03e8,0x03e8}`,
//!   `g_pm_xtal_stable_suspend_nopnum = 0xc8`,
//!   `g_pm_xtal_stable_loopnum = 0x0a`). These reproduce the vendor byte for
//!   byte for the LOW32K / RC-32k / timer path.
//! * **T2 (interpreted)** — a heuristic whose *effect* is clear but whose
//!   exact intent is inferred (e.g. the `pm_wait_xtal_ready` clock-stability
//!   spin). These are reproduced faithfully but with the vendor's
//!   unbounded spin / `start_reboot` failure handling replaced by a bounded
//!   wait returning a typed [`PmError`] — never an infinite loop, never a
//!   silent reboot.
//! * **skipped** — vendor bookkeeping that is provably unnecessary for a
//!   pure-Rust SRAM-resident implementation and is documented as such:
//!     - the **instruction-cache self-patch** in `sleep_start`
//!       (`*(u32*)(0x840058 + (ictag<<8)) = 0x06c006c0` around
//!       `start_suspend`). The vendor runs `sleep_start` out of *cached
//!       flash* and must NOP-patch the cache line that survives the flash
//!       deep-power-down window. Our `sleep_start`, `start_suspend` and the
//!       analog helpers they call all live in `.ram_code` (true SRAM), so no
//!       cache line is ever fetched from flash during the DPD window and the
//!       self-patch has nothing to patch.
//!     - `tl_multi_addr = reg 0x63e` (multi-address boot bookkeeping) — this
//!       firmware owns its own boot/link layout and does not use Telink's
//!       multi-image boot selector.
//!
//! # Scope / non-goals
//!
//! This module implements two hardware-entry paths, both **RC-32k
//! timekeeping + timer wake**:
//! * **`DEEPSLEEP_MODE_RET_SRAM_LOW32K` (0x07)** — reset-on-wake with the low
//!   32 KB of SRAM retained ([`cpu_sleep_timer_rc`]). The reset startup must
//!   detect analog `0x7e != 0`, skip `.data` copy / `.bss` clear, and re-enter
//!   the application through its retention-wake path.
//! * **`SUSPEND_MODE` (0x00)** — resume in place, all SRAM powered
//!   ([`cpu_suspend_timer_rc`]). This was the first hardware gate because it
//!   does not depend on LOW32K retention. It is derived bit-exactly from the
//!   mode-0 branch of `cpu_sleep_wakeup_32k_rc` (`+0xd6`/`+0x102`/`+0x3a0`)
//!   and is **not** aliased onto the LOW32K path: analog `0x7e = 0`, no
//!   analog `0x02` manipulation, extra analog `0x04 = 0x48`, `0x2b = 0x5e`,
//!   `0x2c` tail `0x96`, `0x07 |= 0x04`, `0x7f = 1`, system reg `0x602 = 8`,
//!   and the mode-0 early-wake offset (`g_pm_early_wakeup_time_us[0] = 0x0555`,
//!   `<< 4`). Everything the two modes share (wake-source arm, `0x66`
//!   save/restore, `0x20`/`0x1f`, the 32k target math, the comparator
//!   handshake, `sleep_start`, and all post-wake accounting) is executed by a
//!   single common code path parameterised by a small `ModeProfile`.
//!
//! It is deliberately **not** wired into the sensor application yet, and makes
//! **no** current-draw claims. Pad wake, comparator wake, external-32k-crystal
//! timekeeping and plain `DEEPSLEEP_MODE` (reboot-on-wake) remain out of scope
//! for this landing.
//!
//! # Bit-field rmw correction (T1) — `tbclrs` is full-register BIC
//!
//! The vendor programs several analog wake-config fields with a
//! read-modify-write that **clears a low bit-field, then ORs a new value**.
//! In TC32 (Thumb-like) this is `tmovs r3,#<mask>; tbclrs r0,r3`, and
//! `tbclrs`/BIC is a *full-register* `Rd &= ~Rs` — the mask is the whole
//! operand, not a bit index. Three sites use this:
//! * analog `0x07` — `cpu_sleep_wakeup_32k_rc+0x150`, mask `0x07`, then
//!   `| 0x04` (SUSPEND) / `| 0x01` (LOW32K);
//! * analog `0x02` — `+0x2ea`, mask `0x07`, then `| 0x05` (LOW32K only);
//! * analog `0x05` — `clock_32k_init+0x24`, mask `0x03`, then `| 0x02`
//!   (32k source-select).
//!
//! An earlier revision clamped with bit-index masks (`0x80`/`0x08`) instead,
//! which is **only** equivalent when the pre-existing field bits are already
//! zero; otherwise it leaves stale low bits and yields a bit-inexact
//! wake-config value. The `0x07` write is in the SUSPEND path and was the
//! hardware-proven cause of the initial no-wake result. The rmw is now
//! performed by the host-tested [`calc::rmw_low3_field`] /
//! [`calc::rmw_32k_src_field`]. After this correction and the exact vendor
//! xtal-stability delay, both full-SRAM SUSPEND and reset-on-wake LOW32K
//! completed repeated four-cycle timer-wake tests on TLSR8258 silicon.
//!
//! # Failure-cleanup contract
//!
//! Both entry points save the global IRQ-enable register (`0x643`) and restore
//! it on every returning path. Once system register `0x66` is saved/cleared,
//! it is likewise restored before any return. Full-SRAM SUSPEND returns in
//! place and completes the normal post-wake cleanup. Successful LOW32K does
//! not return: the hardware resets, and the retention-aware startup releases
//! flash from deep-power-down and reinitializes the required hardware.
//! Pre-entry failures restore state and return without triggering sleep.
//!
//! # Cold-boot system-timer requirement (T1)
//!
//! The whole suspend path reads the free-running 16 MHz system timer
//! `reg_system_tick` (`0x740`): for the "now" tick, inside
//! `pm_wait_xtal_ready`, and in the post-wake wake-tick wait. The vendor
//! starts that timer during `cpu_wakeup_init` (`+0xe8`: write `1`, i.e.
//! `FLD_SYSTEM_TICK_START = BIT(0)`, to `reg_system_tick_ctrl` `0x74f`). This
//! crate's pure-Rust cold boot does **not** call `cpu_wakeup_init`:
//! `clocks::init` mirrors only the analog LDO block, and `now_ticks()` reads a
//! *different* peripheral (Timer0), so `0x740` is otherwise never started.
//!
//! Therefore the application **must** call [`system_timer_init`] once at cold
//! boot before invoking either suspend entry. To avoid a hidden global side
//! effect, the suspend body does **not** auto-start the timer; it performs a
//! bounded [`system_timer_ready`] check and returns
//! [`PmError::SystemTimerNotRunning`] if the counter is not advancing. The
//! separate hardware 32k-calibration readiness (register `0x750 != 0`, driven
//! by [`rc_32k_init_and_cal`]) is still gated independently inside the body.
//!
//! # Hardware-proven scope and remaining risks
//!
//! * Runtime RC-32k calibration, the `0x20`/`0x1f` wake timing, comparator
//!   target conversion, and the vendor xtal-stability delay are all exercised
//!   by the successful hardware timer-wake tests.
//! * Analog register `0x5a1` is treated as GPIO **port-E input-enable** and
//!   is **saved/restored** across the DPD window rather than being written
//!   back as the vendor's hard-coded `0x0f`; repeated hardware wakes retained
//!   correct operation with this safer behavior.
//! * System register `0x602` (written `= 8` on the SUSPEND path only, T1
//!   `+0x3a0`) has an unidentified function. The vendor never restores it, so
//!   it is written verbatim and left set; if it turns out to affect normal
//!   operation this needs a save/restore.
//! * The RC-32k PM comparator target is the undocumented/internal register at
//!   `0x754`, confirmed by relocation-aware `pm_32k_rc.o` disassembly and the
//!   independent `modern-tc32/tlsr82xx` `REG_SYSTEM_32K_TICK_CAL` map.
//!   `register.h` also defines the normal system-timer wake register at
//!   `0x748`; that is a distinct register and is not used by this PM path.

// ---------------------------------------------------------------------------
// Deep-sleep-retention analog scratch registers (unchanged public surface)
// ---------------------------------------------------------------------------

/// Analog scratch registers that survive a power cycle (`DEEP_ANA_REG0/1`,
/// `platform/chip_8258/pm.h`). Reset only by a full power cycle.
pub const DEEP_ANA_REG0: u8 = 0x3A;
pub const DEEP_ANA_REG1: u8 = 0x3B;

/// Analog scratch register that survives deep sleep (with or without SRAM
/// retention) but is reset by watchdog, chip reset, or the RESET pin.
/// Telink reserves this one for its own `SYS_NEED_REINIT_EXT32K` /
/// `SYS_DEEP_SLEEP_FLAG` bookkeeping (`SYS_DEEP_ANA_REG` in `pm.h`) — avoid
/// it unless you also own that bookkeeping.
pub const DEEP_ANA_REG2: u8 = 0x3C;

/// Analog scratch registers reset by watchdog, chip reset, or the RESET
/// pin (but *not* by deep sleep) — free for application use, matching
/// `DEEP_ANA_REG6..DEEP_ANA_REG10` in `platform/chip_8258/pm.h`.
pub const DEEP_ANA_REG6: u8 = 0x35;
pub const DEEP_ANA_REG7: u8 = 0x36;
pub const DEEP_ANA_REG8: u8 = 0x37;
pub const DEEP_ANA_REG9: u8 = 0x38;
pub const DEEP_ANA_REG10: u8 = 0x39;

/// Application-safe retention scratch registers — the *only* addresses
/// [`read_retention`]/[`write_retention`] can reach. These map 1:1 to
/// [`DEEP_ANA_REG6`]..[`DEEP_ANA_REG10`] above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RetentionRegister {
    Reg6 = DEEP_ANA_REG6,
    Reg7 = DEEP_ANA_REG7,
    Reg8 = DEEP_ANA_REG8,
    Reg9 = DEEP_ANA_REG9,
    Reg10 = DEEP_ANA_REG10,
}

impl RetentionRegister {
    #[cfg_attr(not(target_arch = "tc32"), allow(dead_code))]
    const fn address(self) -> u8 {
        self as u8
    }
}

/// Read an application-owned retention scratch byte.
#[cfg(target_arch = "tc32")]
pub fn read_retention(reg: RetentionRegister) -> Result<u8, super::mmio::AnalogError> {
    super::mmio::analog_read(reg.address())
}

/// Write an application-owned retention scratch byte.
#[cfg(target_arch = "tc32")]
pub fn write_retention(reg: RetentionRegister, value: u8) -> Result<(), super::mmio::AnalogError> {
    super::mmio::analog_write(reg.address(), value)
}

/// Wakeup source bits, matching `SleepWakeupSrc_TypeDef` in
/// `platform/chip_8258/pm.h`. This is the value written to analog register
/// `0x26` (T1: `cpu_sleep_wakeup_32k_rc+0xdc`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WakeupSource {
    Pad = 1 << 4,
    Core = 1 << 5,
    Timer = 1 << 6,
    Comparator = 1 << 7,
}

/// MCU status after reset, matching `pm_mcu_status` in
/// `platform/chip_8258/pm.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum McuStatus {
    Boot = 0,
    DeepRetentionBack = 1,
    DeepBack = 2,
}

// ---------------------------------------------------------------------------
// Sleep mode / wake status constants and errors
// ---------------------------------------------------------------------------

/// SRAM-retention deep-sleep mode value written verbatim to analog register
/// `0x7e` (T1: `cpu_sleep_wakeup_32k_rc+0x30a`, source `[sp+4] = sleep_mode`).
/// Retains the low 32 KiB of SRAM and reboots through the retention-aware
/// startup path after a timer wake.
pub const DEEPSLEEP_MODE_RET_SRAM_LOW32K: u8 = 0x07;

/// Wake-status flag bits as read back from analog register `0x44`. Confirmed
/// T1 from `cpu_sleep_wakeup_32k_rc+0x288` (`(reg44 << 30)` sign test selects
/// the timer bit, i.e. `BIT(1)`) and the `0x0f` clear mask at `+0x2d0`.
pub const WAKEUP_STATUS_COMPARATOR: u8 = 1 << 0;
pub const WAKEUP_STATUS_TIMER: u8 = 1 << 1;
pub const WAKEUP_STATUS_CORE: u8 = 1 << 2;
pub const WAKEUP_STATUS_PAD: u8 = 1 << 3;

/// `reg_system_tick_ctrl` (`0x74f`) field bits, T1 from
/// `platform/chip_8258/register.h`:
/// `FLD_SYSTEM_TICK_START = BIT(0)`, `FLD_SYSTEM_TICK_RUNNING = BIT(1)`.
///
/// Writing [`FLD_SYSTEM_TICK_START`] starts the free-running 16 MHz system
/// timer at `reg_system_tick` (`0x740`) that the suspend path reads. This is
/// exactly the `1` the vendor writes at `cpu_wakeup_init+0xe8` on cold boot.
pub const FLD_SYSTEM_TICK_START: u8 = 1 << 0;
/// `reg_system_tick_ctrl` running-status bit (`BIT(1)`).
pub const FLD_SYSTEM_TICK_RUNNING: u8 = 1 << 1;

/// Errors returned by the suspend path. No variant is ever produced by a
/// silent fallback — each corresponds to an explicit rejected input or a
/// *bounded* hardware wait that expired.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PmError {
    /// Requested sleep duration was zero ticks.
    ZeroDuration,
    /// Requested sleep is shorter than the vendor early-wake floor
    /// ([`calc::MIN_SLEEP_TICKS`], ≈1.765 ms). The vendor busy-waits in this
    /// case; we reject it so the caller does not think it slept.
    TooShort,
    /// Requested sleep exceeds [`calc::MAX_SLEEP_TICKS`] (≈230 s), inside the
    /// vendor's own abort threshold of `0xE000_0000` ticks.
    TooLong,
    /// The hardware 32k calibration register `0x750` never became non-zero
    /// within the bounded wait, so tick conversion is impossible.
    CalibrationNotReady,
    /// The analog register bus timed out (bounded spin) during setup.
    AnalogTimeout,
    /// The `0x74f` 32k-tick program handshake (bit 3) did not clear.
    TickHandshakeTimeout,
    /// `pm_wait_xtal_ready` never observed the system clock advancing.
    SystemClockNotStable,
    /// The 16 MHz system timer (`reg_system_tick`, `0x740`) is not running:
    /// its counter did not advance within the bounded readiness window. The
    /// caller must run [`system_timer_init`] once at cold boot before using
    /// the suspend path (the pure-Rust boot does not start it, and the repo's
    /// `now_ticks()` uses a different peripheral).
    SystemTimerNotRunning,
    /// After wake the system tick never reached the requested wake tick.
    WakeTickTimeout,
    /// LOW32K entry returned to its caller instead of resetting through the
    /// retention-aware startup path.
    RetentionReturned,
}

/// Result of a suspend attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeStatus {
    raw: u8,
    entered_low_power: bool,
}

impl WakeStatus {
    /// Raw analog `0x44` status nibble read after wake.
    pub fn raw(&self) -> u8 {
        self.raw
    }
    /// `true` if the core actually entered the low-power state (no wake
    /// source was already pending when suspend was armed).
    pub fn entered_low_power(&self) -> bool {
        self.entered_low_power
    }
    pub fn woke_by_timer(&self) -> bool {
        self.raw & WAKEUP_STATUS_TIMER != 0
    }
    pub fn woke_by_pad(&self) -> bool {
        self.raw & WAKEUP_STATUS_PAD != 0
    }
    pub fn woke_by_core(&self) -> bool {
        self.raw & WAKEUP_STATUS_CORE != 0
    }
    pub fn woke_by_comparator(&self) -> bool {
        self.raw & WAKEUP_STATUS_COMPARATOR != 0
    }
}

/// Read-only pre-suspend diagnostic snapshot.
///
/// This is produced by [`suspend_debug_snapshot`], which performs **exactly**
/// the same reads and computations as the SUSPEND entry path up to (but not
/// including) any register write, wake-source arming, `0x66` manipulation, or
/// `sleep_start`. It never enters low power. Use it on hardware to verify the
/// derived comparator target and the raw analog wake-config inputs before
/// committing to a real suspend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PmDebugSnapshot {
    /// 16-bit hardware 32k calibration read from register `0x750`
    /// (16 MHz-ticks-per-32k-period, hardware-populated after RC-32k cal).
    pub calib: u16,
    /// Free-running 16 MHz system tick (`reg 0x740`) sampled as "now".
    pub now_sys_tick: u32,
    /// Reference 16 MHz tick used by the target math (`now + 0x230`).
    pub tick_cur: u32,
    /// Current 32k counter (`pm_get_32k_tick`, analog `0x40..0x43`).
    pub tick_32k_cur: u32,
    /// `wakeup_tick - now` (the requested sleep length in 16 MHz ticks).
    pub duration_ticks: u32,
    /// `wakeup_tick - early_wake_offset` (the early-wake-adjusted target).
    pub adjusted: u32,
    /// `true` if the long (divide-before-shift) target path is selected.
    pub kind_is_long: bool,
    /// The 32k comparator target that would be written to `reg 0x754`.
    pub target32k: u32,
    /// Delta the comparator must count in 32k units (`target32k -
    /// tick_32k_cur`).
    pub target_delta_32k: u32,
    /// Raw analog `0x07` value **before** the wake-config rmw (so the caller
    /// can confirm whether the low 3-bit field was already non-zero — the
    /// condition under which the historical `& !0x80` mask bug diverged).
    pub reg_07_raw: u8,
    /// Raw analog `0x02` value (LOW32K field source), read-only.
    pub reg_02_raw: u8,
    /// Raw analog `0x05` value (32k source-select), read-only.
    pub reg_05_raw: u8,
    /// Computed analog `0x20` value that would be written.
    pub reg_20: u8,
    /// Computed analog `0x1f` value that would be written.
    pub reg_1f: u8,
    /// Computed analog `0x2c` value that would be written (timer source).
    pub reg_2c: u8,
}

// ---------------------------------------------------------------------------
// Pure, host-testable calculations (no MMIO)
// ---------------------------------------------------------------------------

/// Whether a validated sleep duration takes the short (16 MHz-scaled) or long
/// (32k-scaled) tick-conversion path, mirroring the vendor's `pm_long_suspend`
/// selection (T1: threshold `0x0FF0_0000` at `cpu_sleep_wakeup_32k_rc+0x66`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SuspendKind {
    Short,
    Long,
}

pub mod calc {
    //! Bit-exact arithmetic lifted from `cpu_sleep_wakeup_32k_rc`. All values
    //! use absolute 16 MHz system-timer ticks and the 16-bit hardware 32k
    //! calibration read from register `0x750`. Every function is total and
    //! host tested.
    use super::{PmError, SuspendKind};

    /// Early-wake floor: `g_pm_early_wakeup_time_us[6..7] = 0x06e5` µs, scaled
    /// `<< 4` (×16 ticks/µs). T1: `cpu_sleep_wakeup_32k_rc+0x54`.
    pub const MIN_SLEEP_TICKS: u32 = 0x06e5 << 4; // 28_240 ticks ≈ 1.765 ms

    /// Long-suspend selection threshold. T1: `+0x66` (`0xff << 20`).
    pub const LONG_SUSPEND_THRESHOLD: u32 = 0x0FF0_0000; // ≈16.71 s

    /// Vendor hard abort threshold. T1: `+0x4a` (`0xe0 << 24`). Documented for
    /// reference; [`MAX_SLEEP_TICKS`] is stricter.
    pub const VENDOR_ABORT_TICKS: u32 = 0xE000_0000; // ≈234.88 s

    /// Public maximum accepted sleep: 230 s in ticks, strictly inside
    /// [`VENDOR_ABORT_TICKS`].
    pub const MAX_SLEEP_TICKS: u32 = 230_000_000u32.wrapping_mul(16); // 3_680_000_000

    // Compile-time proof that the public limit stays inside the vendor's own
    // hard-abort threshold.
    const _: () = assert!(MAX_SLEEP_TICKS < VENDOR_ABORT_TICKS);

    /// LOW32K-retention early-wake offset subtracted from the absolute wake
    /// tick: `g_pm_early_wakeup_time_us[2..3] = 0x044c` µs, `<< 4`. T1: the
    /// LOW32K entry reads element index 1 at `cpu_sleep_wakeup_32k_rc+0x2ba`.
    pub const EARLY_WAKEUP_RET_TICKS: u32 = 0x044c << 4; // 17_600

    /// Plain-SUSPEND (mode 0) early-wake offset:
    /// `g_pm_early_wakeup_time_us[0..1] = 0x0555` µs, `<< 4`. T1: the mode-0
    /// entry reads element index 0 at `cpu_sleep_wakeup_32k_rc+0xd6`
    /// (`.data` bytes `55 05 4c 04 …`, verified against `pm.o`).
    pub const EARLY_WAKEUP_SUSPEND_TICKS: u32 = 0x0555 << 4; // 21_840

    /// Setup latency added to "now" to form `tick_cur`. T1: `+0xa4` (`0x8c<<2`).
    pub const TICK_SETUP_OFFSET: u32 = 0x230; // 560

    /// Offset added to `tick_cur` when re-seeding the system tick after wake.
    /// T1: `+0x248` (`+0x41` then `+0xff`).
    pub const POST_WAKE_TICK_OFFSET: u32 = 320;

    /// `g_pm_r_delay_us[2..3] = 0x03e8` (1000). Feeds analog reg `0x1f`.
    pub const R_DELAY_US: u32 = 0x03e8;

    /// Constant numerator base for analog reg `0x20`. T1: `+0x17e` (`0xfa<<8`).
    pub const ANALOG20_BASE: u32 = 0xfa00; // 64_000

    /// Validate a sleep *duration* (ticks) and classify its conversion path.
    ///
    /// Rejects zero, too-short (`< MIN_SLEEP_TICKS`) and too-long
    /// (`> MAX_SLEEP_TICKS`) explicitly.
    pub const fn classify_duration(ticks: u32) -> Result<SuspendKind, PmError> {
        if ticks == 0 {
            Err(PmError::ZeroDuration)
        } else if ticks < MIN_SLEEP_TICKS {
            Err(PmError::TooShort)
        } else if ticks > MAX_SLEEP_TICKS {
            Err(PmError::TooLong)
        } else if ticks > LONG_SUSPEND_THRESHOLD {
            Ok(SuspendKind::Long)
        } else {
            Ok(SuspendKind::Short)
        }
    }

    /// Compose the analog `0x2c` wake-control byte.
    ///
    /// T1: `cpu_sleep_wakeup_32k_rc+0x12a..0x142`:
    /// `((!(src&0x80))<<3) | !(src&0xC0) | tail`, where for the LOW32K path
    /// `tail = 0x16 | 0x40 = 0x56` (T1: `+0x306`,`+0x312`).
    pub const fn compose_reg_2c(wakeup_src: u8, tail: u8) -> u8 {
        let b_c0 = if wakeup_src & 0xC0 == 0 { 1u8 } else { 0 };
        let b_80 = if wakeup_src & 0x80 == 0 { 1u8 } else { 0 };
        (b_80 << 3) | b_c0 | tail
    }

    /// Read-modify-write of an analog 3-bit field (bits `[2:0]`): clear the
    /// field, then OR in `field`. T1: `tmovs r3,#7; tbclrs r0,r3; ... tors`.
    /// Thumb `BIC` (`tbclrs`) is a full-register `Rd &= ~Rs`, so the clear
    /// mask is `0x07` — used for analog `0x07` (`+0x150`) and `0x02`
    /// (`+0x2ea`). Historically this used the wrong mask `0x80`, which left
    /// stale low bits and produced a bit-inexact wake-config value.
    pub const fn rmw_low3_field(old: u8, field: u8) -> u8 {
        (old & !0x07) | (field & 0x07)
    }

    /// Read-modify-write of the analog `0x05` 32k source-select field (bits
    /// `[1:0]`): clear then OR. T1: `tmovs r3,#3; tbclrs r5,r3` in
    /// `clock_32k_init+0x24` (mask `0x03`, full-register BIC).
    pub const fn rmw_32k_src_field(old: u8, field: u8) -> u8 {
        (old & !0x03) | (field & 0x03)
    }

    /// Analog `0x2c` tail constant for the LOW32K retention path.
    /// T1: `+0x312..0x31c` (`0x16 | 0x40`).
    pub const REG_2C_TAIL_LOW32K: u8 = 0x16 | 0x40; // 0x56

    /// Analog `0x2c` tail constant for the plain-SUSPEND (mode 0) path.
    /// T1: `+0x116` (`r3 = 0x96` stored to `[sp+4]`, then composed identically
    /// to the LOW32K tail via `+0x12a..0x142`).
    pub const REG_2C_TAIL_SUSPEND: u8 = 0x96;

    /// Analog `0x2b` value for the LOW32K retention path (T1: `+0x31e`).
    pub const REG_2B_LOW32K: u8 = 0xde;

    /// Analog `0x2b` value for the plain-SUSPEND (mode 0) path (T1: `+0x11a`,
    /// `r1 = 0x5e`).
    pub const REG_2B_SUSPEND: u8 = 0x5e;

    /// Low bits OR'd into analog `0x07` (after clearing bit 7) for the LOW32K
    /// path. T1: `[sp+8] = 1` at `+0x302`, applied by the shared rmw `+0x148`.
    pub const REG_07_BITS_LOW32K: u8 = 0x01;

    /// Low bits OR'd into analog `0x07` (after clearing bit 7) for the
    /// plain-SUSPEND path. T1: `[sp+8] = 4` at `+0x112`, applied by the shared
    /// rmw `+0x148`.
    pub const REG_07_BITS_SUSPEND: u8 = 0x04;

    /// Analog register `0x20` value = `0x7f - (0xfa00 + calib/2) / calib`.
    /// T1: `cpu_sleep_wakeup_32k_rc+0x17e..0x196`.
    pub fn analog_20_value(calib: u16) -> Option<u8> {
        if calib == 0 {
            return None;
        }
        let calib = calib as u32;
        let fp = calib >> 1;
        let q = (ANALOG20_BASE + fp) / calib;
        Some(0x7fu32.wrapping_sub(q) as u8)
    }

    /// Analog register `0x1f` value = `((R_DELAY_US<<7) + calib/2) / calib`.
    /// T1: `cpu_sleep_wakeup_32k_rc+0x19a..0x1b6`.
    pub fn analog_1f_value(calib: u16) -> Option<u8> {
        if calib == 0 {
            return None;
        }
        let calib = calib as u32;
        let fp = calib >> 1;
        Some((((R_DELAY_US << 7) + fp) / calib) as u8)
    }

    /// 32k wake-target tick, short path.
    /// `tick_32k_cur + (((adjusted-tick_cur)<<4) + calib/2) / calib`.
    /// T1: `cpu_sleep_wakeup_32k_rc+0x34e..0x360`.
    pub fn target_32k_short(
        adjusted: u32,
        tick_cur: u32,
        calib: u16,
        tick_32k_cur: u32,
    ) -> Option<u32> {
        if calib == 0 {
            return None;
        }
        let calib = calib as u32;
        let fp = calib >> 1;
        let d = adjusted.wrapping_sub(tick_cur);
        Some(tick_32k_cur.wrapping_add((d.wrapping_shl(4).wrapping_add(fp)) / calib))
    }

    /// 32k wake-target tick, long path (divide before shift to avoid overflow).
    /// `tick_32k_cur + ((adjusted-tick_cur)/calib)<<4`.
    /// T1: `cpu_sleep_wakeup_32k_rc+0x1c6..0x1d6`.
    pub fn target_32k_long(
        adjusted: u32,
        tick_cur: u32,
        calib: u16,
        tick_32k_cur: u32,
    ) -> Option<u32> {
        if calib == 0 {
            return None;
        }
        let calib = calib as u32;
        let d = adjusted.wrapping_sub(tick_cur);
        Some(tick_32k_cur.wrapping_add((d / calib).wrapping_shl(4)))
    }

    /// Elapsed 16 MHz ticks since suspend, short path:
    /// `(calib * (now32k - tick_32k_cur)) >> 4`.
    /// T1: `cpu_sleep_wakeup_32k_rc+0x33c..0x344`.
    pub fn elapsed_ticks_short(now32k: u32, tick_32k_cur: u32, calib: u16) -> u32 {
        let e = now32k.wrapping_sub(tick_32k_cur);
        (calib as u32).wrapping_mul(e) >> 4
    }

    /// Elapsed 16 MHz ticks since suspend, long path:
    /// `calib * ((now32k - tick_32k_cur) >> 4)`.
    /// T1: `cpu_sleep_wakeup_32k_rc+0x238..0x244`.
    pub fn elapsed_ticks_long(now32k: u32, tick_32k_cur: u32, calib: u16) -> u32 {
        let e = now32k.wrapping_sub(tick_32k_cur) >> 4;
        (calib as u32).wrapping_mul(e)
    }

    /// Absolute wake tick minus a mode-specific early-wake offset.
    /// T1: `cpu_sleep_wakeup_32k_rc+0x2c4` (LOW32K) / `+0xd6` (SUSPEND) — the
    /// only difference between the two modes here is which
    /// `g_pm_early_wakeup_time_us` element supplies `offset`.
    pub const fn adjusted_wake_tick_with(wakeup_tick: u32, offset: u32) -> u32 {
        wakeup_tick.wrapping_sub(offset)
    }

    /// LOW32K-retention adjusted wake tick.
    pub const fn adjusted_wake_tick(wakeup_tick: u32) -> u32 {
        adjusted_wake_tick_with(wakeup_tick, EARLY_WAKEUP_RET_TICKS)
    }

    /// Plain-SUSPEND (mode 0) adjusted wake tick.
    pub const fn adjusted_wake_tick_suspend(wakeup_tick: u32) -> u32 {
        adjusted_wake_tick_with(wakeup_tick, EARLY_WAKEUP_SUSPEND_TICKS)
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn duration_rejects_zero() {
            assert_eq!(classify_duration(0), Err(PmError::ZeroDuration));
        }

        #[test]
        fn duration_rejects_too_short() {
            assert_eq!(
                classify_duration(MIN_SLEEP_TICKS - 1),
                Err(PmError::TooShort)
            );
            // exactly the floor is accepted (vendor uses `>=`).
            assert_eq!(classify_duration(MIN_SLEEP_TICKS), Ok(SuspendKind::Short));
        }

        #[test]
        fn duration_rejects_too_long() {
            assert_eq!(
                classify_duration(MAX_SLEEP_TICKS + 1),
                Err(PmError::TooLong)
            );
        }

        #[test]
        fn duration_selects_short_then_long_at_threshold() {
            assert_eq!(
                classify_duration(LONG_SUSPEND_THRESHOLD),
                Ok(SuspendKind::Short)
            );
            assert_eq!(
                classify_duration(LONG_SUSPEND_THRESHOLD + 1),
                Ok(SuspendKind::Long)
            );
        }

        #[test]
        fn min_and_early_constants_match_extracted_defaults() {
            assert_eq!(MIN_SLEEP_TICKS, 28_240);
            assert_eq!(EARLY_WAKEUP_RET_TICKS, 17_600);
            assert_eq!(EARLY_WAKEUP_SUSPEND_TICKS, 21_840);
            assert_eq!(MAX_SLEEP_TICKS, 3_680_000_000);
            assert_eq!(ANALOG20_BASE, 64_000);
        }

        #[test]
        fn reg_2c_matches_vendor_for_timer_low32k() {
            // Timer source = 0x40 (WakeupSource::Timer), tail 0x56 → 0x5e.
            assert_eq!(compose_reg_2c(TIMER_SRC, REG_2C_TAIL_LOW32K), 0x5e);
        }

        #[test]
        fn rmw_low3_field_clears_low_three_bits_not_bit7() {
            // Field write must fully replace bits [2:0], regardless of the
            // stale low bits — the historical `& !0x80` bug did not.
            assert_eq!(rmw_low3_field(0x00, 0x04), 0x04);
            assert_eq!(rmw_low3_field(0x07, 0x04), 0x04); // stale 0b111 cleared
            assert_eq!(rmw_low3_field(0x03, 0x04), 0x04); // stale 0b011 cleared
            assert_eq!(rmw_low3_field(0x03, 0x01), 0x01); // LOW32K field = 1
            // Upper bits (including bit 7) are preserved untouched.
            assert_eq!(rmw_low3_field(0xF3, 0x04), 0xF4);
            assert_eq!(rmw_low3_field(0x83, 0x04), 0x84);
            // Contrast with the buggy `& !0x80` mask, which would leave 0x07.
            assert_ne!(rmw_low3_field(0x03, 0x04), (0x03 & !0x80) | 0x04);
        }

        #[test]
        fn rmw_32k_src_field_clears_low_two_bits() {
            assert_eq!(rmw_32k_src_field(0x00, 0x02), 0x02);
            assert_eq!(rmw_32k_src_field(0x03, 0x02), 0x02); // stale 0b11 cleared
            assert_eq!(rmw_32k_src_field(0xF1, 0x02), 0xF2); // upper preserved
        }

        #[test]
        fn reg_2c_matches_vendor_for_timer_suspend() {
            // Timer source = 0x40, SUSPEND tail 0x96 → 0x96 | (1<<3) = 0x9e.
            assert_eq!(compose_reg_2c(TIMER_SRC, REG_2C_TAIL_SUSPEND), 0x9e);
        }

        #[test]
        fn suspend_mode_constants_differ_from_low32k() {
            // Guard against accidentally aliasing the two mode profiles.
            assert_ne!(REG_2B_SUSPEND, REG_2B_LOW32K);
            assert_ne!(REG_2C_TAIL_SUSPEND, REG_2C_TAIL_LOW32K);
            assert_ne!(REG_07_BITS_SUSPEND, REG_07_BITS_LOW32K);
            assert_ne!(EARLY_WAKEUP_SUSPEND_TICKS, EARLY_WAKEUP_RET_TICKS);
            assert_eq!(REG_2B_SUSPEND, 0x5e);
            assert_eq!(REG_07_BITS_SUSPEND, 0x04);
            assert_eq!(REG_07_BITS_LOW32K, 0x01);
        }

        // WakeupSource::Timer as a plain byte to keep the test in this module.
        const TIMER_SRC: u8 = 1 << 6;

        #[test]
        fn reg_2c_comparator_only_sets_both_bits_zero() {
            // src has bit7 set → both !() terms are 0.
            assert_eq!(compose_reg_2c(0x80, 0x00), 0x00);
        }

        #[test]
        fn reg_2c_no_source_sets_both_bits() {
            // src == 0 → !(src&0xC0)=1, !(src&0x80)=1 → (1<<3)|1 = 0x09.
            assert_eq!(compose_reg_2c(0x00, 0x00), 0x09);
        }

        #[test]
        fn analog_regs_reject_zero_calib() {
            assert_eq!(analog_20_value(0), None);
            assert_eq!(analog_1f_value(0), None);
            assert_eq!(target_32k_short(1, 0, 0, 0), None);
            assert_eq!(target_32k_long(1, 0, 0, 0), None);
        }

        #[test]
        fn analog_20_value_matches_formula() {
            // calib = 500: fp=250, q=(64000+250)/500=128, 0x7f-128 = -1 → 0xff.
            assert_eq!(analog_20_value(500), Some(0xffu8));
            // calib = 1000: fp=500, q=(64000+500)/1000=64, 0x7f-64=63=0x3f.
            assert_eq!(analog_20_value(1000), Some(0x3f));
        }

        #[test]
        fn analog_1f_value_matches_formula() {
            // calib=1000: fp=500, (1000<<7 + 500)/1000 = 128500/1000 = 128 → 0x80.
            assert_eq!(analog_1f_value(1000), Some(0x80));
            // calib=500: fp=250, (128000+250)/500 = 256 → truncated u8 = 0x00.
            assert_eq!(analog_1f_value(500), Some(0x00));
        }

        #[test]
        fn target_short_and_long_agree_when_no_shift_overflow() {
            // With small durations both orders give the same result.
            let calib = 488u16;
            let tick_cur = 1_000u32;
            let adjusted = tick_cur + 4_880; // 10 * calib
            let base = 7_000u32;
            let s = target_32k_short(adjusted, tick_cur, calib, base).unwrap();
            // short includes the +calib/2 rounding term; long does not.
            let long = target_32k_long(adjusted, tick_cur, calib, base).unwrap();
            assert_eq!(long, base + 10 * 16); // (4880/488)=10, <<4
            // short: ((4880<<4)+244)/488 = (78080+244)/488 = 160 (+base)
            assert_eq!(s, base + 160);
        }

        #[test]
        fn elapsed_short_and_long_roundtrip_reasonably() {
            let calib = 488u16;
            // ~1 s at 32k ≈ 32768 32k-ticks; short = 488*32768>>4.
            let now32k = 32_768u32;
            let base = 0u32;
            let s = elapsed_ticks_short(now32k, base, calib);
            let l = elapsed_ticks_long(now32k, base, calib);
            assert_eq!(s, (488u32 * 32_768) >> 4);
            assert_eq!(l, 488u32 * (32_768 >> 4));
            // Both approximate 1 s (16e6 ticks) to within the calib error.
            assert!(s > 900_000 && s < 1_100_000);
            assert!(l > 900_000 && l < 1_100_000);
        }

        #[test]
        fn adjusted_wake_tick_subtracts_offset() {
            assert_eq!(adjusted_wake_tick(100_000), 100_000 - 17_600);
            assert_eq!(adjusted_wake_tick_suspend(100_000), 100_000 - 21_840);
            assert_eq!(adjusted_wake_tick_with(100_000, 7), 99_993);
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware entry (tc32 only): RC-32k / LOW32K / timer-wake suspend
// ---------------------------------------------------------------------------

#[cfg(target_arch = "tc32")]
mod hw {
    use super::calc;
    use super::{
        FLD_SYSTEM_TICK_RUNNING, FLD_SYSTEM_TICK_START, PmDebugSnapshot, PmError, SuspendKind,
        WakeStatus, WakeupSource,
    };
    use crate::mmio::{REG_ANALOG_ADDR, r8, r32, w8, w32};

    // -- register addresses (all T1 from the disassembly literal pools) ------
    const REG_MSPI_DATA: u32 = 0x0080_000c; // flash JEDEC command byte
    const REG_MSPI_CTRL: u32 = 0x0080_000d; // flash CS / manual-mode strobe
    const REG_SUSPEND: u32 = 0x0080_006f; // start_suspend trigger (write 0x81)
    const REG_SYS_CTRL_66: u32 = 0x0080_0066; // saved/cleared/restored (0x66)
    const REG_SYS_602: u32 = 0x0080_0602; // SUSPEND-only wake enable (see risks)
    const REG_GPIO_PE_IE: u32 = 0x0080_05a1; // port-E input enable (see risks)
    const REG_IRQ_EN_643: u32 = 0x0080_0643; // global IRQ enable save/restore
    const REG_SYS_TICK: u32 = 0x0080_0740; // 16 MHz system tick (clock_time)
    const REG_32K_CALIB: u32 = 0x0080_0750; // hardware 32k calibration (u16)
    const REG_32K_CMD_74C: u32 = 0x0080_074c; // 32k timer command / mode
    const REG_32K_WAKE_TICK: u32 = 0x0080_0754; // internal RC-32k PM target
    // reg_system_tick_ctrl (0x74f). BIT0 = FLD_SYSTEM_TICK_START (start the
    // 16 MHz system timer), BIT1 = RUNNING status, BIT3 = the 32k-tick
    // program handshake used while arming the comparator.
    const REG_SYS_TICK_CTRL_74F: u32 = 0x0080_074f;
    const SYS_TICK_PROGRAM_HANDSHAKE: u8 = 1 << 3; // BIT3

    // Analog register indices (written via the analog bus).
    const ANA_02: u8 = 0x02;
    const ANA_04: u8 = 0x04; // SUSPEND-only: written 0x48
    const ANA_05: u8 = 0x05; // 32k source-select (read-only in diagnostics)
    const ANA_07: u8 = 0x07;
    const ANA_1F: u8 = 0x1f;
    const ANA_20: u8 = 0x20;
    const ANA_26: u8 = 0x26; // wake source enable
    const ANA_2B: u8 = 0x2b;
    const ANA_2C: u8 = 0x2c;
    const ANA_34: u8 = 0x34; // LDO bracket (0x87 enter / 0x80 exit)
    const ANA_44: u8 = 0x44; // wake status (write 0x0f clears)
    const ANA_7E: u8 = 0x7e; // retention mode config
    const ANA_7F: u8 = 0x7f;
    const ANA_82: u8 = 0x82; // LDO bracket (0x0c enter / 0x64 exit)

    // Bounded-wait budgets.
    const ANALOG_SPIN: u32 = 100_000;
    const CALIB_READY_SPIN: u32 = 200_000;
    const TICK_HANDSHAKE_SPIN: u32 = 100_000;
    const WAKE_TICK_SPIN: u32 = 2_000_000;
    // System-timer readiness: at 16 MHz the counter advances every ~62.5 ns,
    // so any real running timer changes 0x740 within a handful of NOPs; this
    // bound is generous.
    const SYS_TICK_READY_SPIN: u32 = 10_000;
    // T1 vendor `.data` defaults.
    const XTAL_STABLE_SUSPEND_NOPS: u32 = 0xc8; // 200
    const XTAL_STABLE_LOOPNUM: u32 = 0x0a; // 10

    #[inline(always)]
    fn nop() {
        unsafe { core::arch::asm!("nop") };
    }

    /// `.ram_code` analog-bus write with a bounded spin. Does *not* touch the
    /// global IRQ enable (the suspend entry owns that save/restore), unlike
    /// `crate::mmio::analog_write`. Returns `false` on bounded timeout but
    /// always clears the trigger so the bus is left idle.
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn ana_w(addr: u8, value: u8) -> bool {
        unsafe {
            w8(REG_ANALOG_ADDR, addr);
            w8(REG_ANALOG_ADDR + 1, value);
            w8(REG_ANALOG_ADDR + 2, 0x60);
            let mut i = 0;
            while i < ANALOG_SPIN {
                if r8(REG_ANALOG_ADDR + 2) & 1 == 0 {
                    w8(REG_ANALOG_ADDR + 2, 0);
                    return true;
                }
                nop();
                i += 1;
            }
            w8(REG_ANALOG_ADDR + 2, 0);
            false
        }
    }

    /// `.ram_code` analog-bus read with a bounded spin.
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn ana_r(addr: u8) -> Option<u8> {
        unsafe {
            w8(REG_ANALOG_ADDR, addr);
            w8(REG_ANALOG_ADDR + 2, 0x40);
            let mut i = 0;
            while i < ANALOG_SPIN {
                if r8(REG_ANALOG_ADDR + 2) & 1 == 0 {
                    let v = r8(REG_ANALOG_ADDR + 1);
                    w8(REG_ANALOG_ADDR + 2, 0);
                    return Some(v);
                }
                nop();
                i += 1;
            }
            w8(REG_ANALOG_ADDR + 2, 0);
            None
        }
    }

    /// The suspend trigger. T1: `cstartup_8258.S:start_suspend` — write
    /// `0x81` to `0x80006f`, then 64 exact `tmov r8,r8` instructions. These
    /// are semantic no-ops, but the silicon-critical post-trigger sequence is
    /// reproduced instruction-for-instruction instead of substituting the
    /// distinct TC32 `nop` opcode. The vendor's instruction-cache self-patch
    /// is intentionally omitted: this function and everything it is reached
    /// from live in `.ram_code`, so no flash cache line is fetched across the
    /// trigger.
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn start_suspend() {
        unsafe {
            w8(REG_SUSPEND, 0x81);
            // Exact post-trigger sequence from cstartup_8258.S.
            core::arch::asm!(
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
                "mov r8, r8; mov r8, r8; mov r8, r8; mov r8, r8",
            );
        }
    }

    /// T1: `pm.o:.ram_code:sleep_start`. Deep-power-down flash, bracket the
    /// analog LDO, enter suspend, then release flash and restore the system
    /// LDO — all from SRAM. Deviation (documented in module docs): analog
    /// `0x5a1` (GPIO PE IE) is **saved and restored** rather than being
    /// written back to the vendor's hard-coded `0x0f`.
    ///
    /// # Failure contract
    ///
    /// Every analog-bus write is bounded and its success tracked. On every
    /// returning path this function completes the mandatory hardware cleanup:
    ///
    /// * flash Release-from-deep-power-down (JEDEC `0xAB`),
    /// * GPIO PE-IE restore,
    /// * the LDO exit bracket (`0x82 = 0x64`, `0x34 = 0x80`).
    ///
    /// Successful LOW32K entry resets before this function returns; its
    /// retention-aware startup performs the flash release instead. If either
    /// **enter** LDO bracket write failed, `start_suspend` is skipped, cleanup
    /// still runs, and [`PmError::AnalogTimeout`] is returned.
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn sleep_start() -> Result<(), PmError> {
        unsafe {
            let mut analog_ok = true;

            // ---- ENTER: LDO bracket + flash deep-power-down ----
            analog_ok &= ana_w(ANA_34, 0x87);

            // Flash deep-power-down (JEDEC 0xB9).
            w8(REG_MSPI_CTRL, 0);
            w8(REG_MSPI_DATA, 0xB9);
            // short settle (vendor loops to 1: 2 iterations)
            nop();
            nop();
            w8(REG_MSPI_CTRL, 1);

            // Save GPIO PE IE, drive it low across the window.
            let saved_pe_ie = r8(REG_GPIO_PE_IE);
            w8(REG_GPIO_PE_IE, 0);

            analog_ok &= ana_w(ANA_82, 0x0c);

            // ---- enter low power (resumes here after wake). Only trigger if
            //      both enter brackets programmed cleanly. ----
            if analog_ok {
                start_suspend();
            }

            // ---- EXIT: mandatory cleanup, always executed ----
            let c1 = ana_w(ANA_82, 0x64);

            // Restore the *original* PE IE (deviation from vendor 0x0f).
            w8(REG_GPIO_PE_IE, saved_pe_ie);

            // Flash release from deep-power-down (JEDEC 0xAB).
            w8(REG_MSPI_CTRL, 0);
            w8(REG_MSPI_DATA, 0xAB);
            nop();
            nop();
            w8(REG_MSPI_CTRL, 1);

            // LDO exit bracket + stabilisation NOP window (T1: 200).
            let c2 = ana_w(ANA_34, 0x80);
            let mut i = 0;
            while i < XTAL_STABLE_SUSPEND_NOPS {
                nop();
                i += 1;
            }

            if analog_ok && c1 && c2 {
                Ok(())
            } else {
                Err(PmError::AnalogTimeout)
            }
        }
    }

    /// T1: `pm.o:.ram_code:pm_get_32k_tick`. Assemble the 32-bit 32k tick from
    /// analog regs `0x43..0x40` (big-endian) with the vendor's consecutive
    /// read debounce, but bounded: two reads differing by ≤1 (or the
    /// XOR==1 rollover case) are accepted; otherwise the last read is used
    /// after the bound (the counter is monotonic, so this cannot fabricate a
    /// value out of range — it only bounds the debounce).
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn pm_get_32k_tick() -> Result<u32, PmError> {
        let read_once = || -> Option<u32> {
            let b3 = ana_r(0x43)? as u32;
            let b2 = ana_r(0x42)? as u32;
            let b1 = ana_r(0x41)? as u32;
            let b0 = ana_r(0x40)? as u32;
            Some((b3 << 24) | (b2 << 16) | (b1 << 8) | b0)
        };
        let mut prev = read_once().ok_or(PmError::AnalogTimeout)?;
        let mut i = 0u32;
        loop {
            let cur = read_once().ok_or(PmError::AnalogTimeout)?;
            if cur == prev || cur.wrapping_sub(prev) <= 1 || (cur ^ prev) == 1 {
                return Ok(cur);
            }
            prev = cur;
            i += 1;
            if i >= 64 {
                return Ok(cur);
            }
        }
    }

    /// T2: `pm.o:.ram_code:pm_wait_xtal_ready`. Bounded reproduction of the
    /// vendor clock-stability spin. The delay is deliberately a 60-iteration
    /// volatile load/increment/store loop, not 60 NOPs: the vendor then checks
    /// whether more than 20 us (320 system ticks) elapsed across that heavier
    /// loop. Replacing it with NOPs made the interval too short and falsely
    /// reported every successful hardware wake as unstable.
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn pm_wait_xtal_ready() -> Result<(), PmError> {
        let mut i = 0u32;
        while i < XTAL_STABLE_LOOPNUM {
            let t0 = unsafe { r32(REG_SYS_TICK) };
            let mut j = 0u32;
            while unsafe { core::ptr::read_volatile(core::ptr::addr_of!(j)) } <= 0x3b {
                let next =
                    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(j)) }.wrapping_add(1);
                unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(j), next) };
            }
            let dt = unsafe { r32(REG_SYS_TICK) }.wrapping_sub(t0);
            if dt > 320 {
                return Ok(());
            }
            i += 1;
        }
        Err(PmError::SystemClockNotStable)
    }

    /// The bit-exact register differences between the two RC-32k timer-wake
    /// entry paths, decoded from `cpu_sleep_wakeup_32k_rc`. **Everything not
    /// listed here is shared byte-for-byte** between the two modes (wake
    /// source arm `0x26`, status clear `0x44`, `0x66` save/clear/restore,
    /// analog `0x20`/`0x1f`, the 32k target math, the `0x74c`/`0x754`/`0x74f`
    /// comparator handshake, `sleep_start`, and all post-wake accounting).
    ///
    /// This is a parameterisation of the two genuinely-distinct branches, not
    /// an aliasing of one onto the other: each field is a value the vendor
    /// object writes on exactly one of the two paths.
    #[derive(Clone, Copy)]
    struct ModeProfile {
        /// Analog `0x7e` retention-mode selector. LOW32K `0x07` (T1 `+0x30a`)
        /// / SUSPEND `0x00` (T1 `+0x10a`).
        reg_7e: u8,
        /// Analog `0x2b`. LOW32K `0xde` (T1 `+0x31e`) / SUSPEND `0x5e`
        /// (T1 `+0x11a`).
        reg_2b: u8,
        /// Tail OR'd into the composed analog `0x2c`. LOW32K `0x56`
        /// (T1 `+0x312`) / SUSPEND `0x96` (T1 `+0x116`).
        reg_2c_tail: u8,
        /// Low bits OR'd into analog `0x07` after clearing bit 7. LOW32K
        /// `0x01` / SUSPEND `0x04` (shared rmw T1 `+0x148`, source `[sp+8]`).
        reg_07_bits: u8,
        /// Analog `0x7f`. LOW32K `0x00` (T1 `+0x168`) / SUSPEND `0x01`
        /// (T1 `+0x3a6`).
        reg_7f: u8,
        /// Early-wake offset subtracted from the absolute wake tick. LOW32K
        /// `0x044c<<4` (idx 1) / SUSPEND `0x0555<<4` (idx 0).
        early_wakeup_ticks: u32,
        /// LOW32K only: apply the analog `0x02` rmw `(v & !0x07) | 0x05`
        /// (T1 `+0x2ea`: `tmovs r3,#7; tbclrs r0,r3` = clear the low 3-bit
        /// field, then OR the new field value). SUSPEND never touches `0x02`.
        touch_reg_02: bool,
        /// SUSPEND only: write analog `0x04 = 0x48` (T1 `+0x102`).
        write_reg_04: bool,
        /// SUSPEND only: write system register `0x602 = 0x08` (T1 `+0x3a0`).
        write_reg_602: bool,
    }

    /// `DEEPSLEEP_MODE_RET_SRAM_LOW32K` (0x07): SRAM-LOW32K retention path.
    const PROFILE_LOW32K: ModeProfile = ModeProfile {
        reg_7e: super::DEEPSLEEP_MODE_RET_SRAM_LOW32K,
        reg_2b: calc::REG_2B_LOW32K,
        reg_2c_tail: calc::REG_2C_TAIL_LOW32K,
        reg_07_bits: calc::REG_07_BITS_LOW32K,
        reg_7f: 0x00,
        early_wakeup_ticks: calc::EARLY_WAKEUP_RET_TICKS,
        touch_reg_02: true,
        write_reg_04: false,
        write_reg_602: false,
    };

    /// `SUSPEND_MODE` (0x00): plain suspend, no LOW32K-specific analog config.
    const PROFILE_SUSPEND: ModeProfile = ModeProfile {
        reg_7e: 0x00,
        reg_2b: calc::REG_2B_SUSPEND,
        reg_2c_tail: calc::REG_2C_TAIL_SUSPEND,
        reg_07_bits: calc::REG_07_BITS_SUSPEND,
        reg_7f: 0x01,
        early_wakeup_ticks: calc::EARLY_WAKEUP_SUSPEND_TICKS,
        touch_reg_02: false,
        write_reg_04: true,
        write_reg_602: true,
    };

    /// Run `body` with the global IRQ-enable register (`0x643`) saved and
    /// cleared, restoring it on every path that returns to the caller. T1:
    /// `cpu_sleep_wakeup_32k_rc+0x1c` / `+0x29e`.
    #[inline(always)]
    fn with_irq_saved<T, F>(body: F) -> Result<T, PmError>
    where
        F: FnOnce() -> Result<T, PmError>,
    {
        let saved_irq = unsafe { r8(REG_IRQ_EN_643) };
        unsafe { w8(REG_IRQ_EN_643, 0) };
        let result = body();
        // Restore on every returning path, including every typed error.
        unsafe { w8(REG_IRQ_EN_643, saved_irq) };
        result
    }

    /// Enter `DEEPSLEEP_MODE_RET_SRAM_LOW32K` and wake by the 32k RC timer at
    /// the absolute 16 MHz system tick `wakeup_tick`.
    ///
    /// The caller must have already run [`super::rc_32k_init_and_cal`] (RC-32k
    /// selected and calibrated) so that register `0x750` is valid.
    ///
    /// A successful entry never returns: timer wake resets the core, preserves
    /// LOW32K SRAM, and re-enters the application's retention-aware startup.
    /// Pre-entry failures return a typed [`PmError`]. If the hardware
    /// unexpectedly resumes in place, the function returns
    /// [`PmError::RetentionReturned`] rather than pretending LOW32K succeeded.
    pub fn cpu_sleep_timer_rc(wakeup_tick: u32) -> Result<core::convert::Infallible, PmError> {
        with_irq_saved(|| match suspend_body(wakeup_tick, &PROFILE_LOW32K) {
            Ok(_) => Err(PmError::RetentionReturned),
            Err(error) => Err(error),
        })
    }

    /// Enter plain `SUSPEND_MODE` (0x00) and wake by the 32k RC timer at the
    /// absolute 16 MHz system tick `wakeup_tick`.
    ///
    /// SUSPEND resumes in place with all SRAM powered, so it does **not**
    /// depend on LOW32K retention. It is
    /// derived bit-exactly from the `cpu_sleep_wakeup_32k_rc` mode-0 branch
    /// (`+0xd6`/`+0x102`/`+0x3a0`), not aliased onto the LOW32K path: analog
    /// `0x7e = 0`, no analog `0x02` manipulation, analog `0x04 = 0x48`,
    /// `0x2b = 0x5e`, `0x2c` tail `0x96`, `0x07 |= 0x04`, `0x7f = 1`, system
    /// register `0x602 = 8`, and the mode-0 early-wake offset `0x0555<<4`.
    ///
    /// Returns the decoded [`WakeStatus`] after the in-place wake. All waits
    /// are bounded, failures are typed, and IRQ + `0x66` are restored
    /// unconditionally before returning.
    pub fn cpu_suspend_timer_rc(wakeup_tick: u32) -> Result<WakeStatus, PmError> {
        with_irq_saved(|| suspend_body(wakeup_tick, &PROFILE_SUSPEND))
    }

    /// Start the free-running 16 MHz system timer (`reg_system_tick`, `0x740`)
    /// by writing [`FLD_SYSTEM_TICK_START`] (BIT0) to `reg_system_tick_ctrl`
    /// (`0x74f`) — the exact cold-boot write the vendor performs at
    /// `cpu_wakeup_init+0xe8`.
    ///
    /// This **must** be called once at cold boot before using the suspend
    /// path: the pure-Rust boot (`clocks::init`) mirrors only the analog LDO
    /// block of `cpu_wakeup_init` and does not start this timer, and the
    /// repo's `now_ticks()` reads a different peripheral (Timer0). The suspend
    /// path reads `0x740` for its "now" tick, its xtal-ready wait, and its
    /// post-wake tick wait, all of which stall forever if the timer is idle.
    ///
    /// Idempotent and bounded: after the start write it confirms the counter
    /// is actually advancing via [`system_timer_ready`]. Returns
    /// [`PmError::SystemTimerNotRunning`] if it never advances.
    pub fn system_timer_init() -> Result<(), PmError> {
        // T1: exact cpu_wakeup_init+0xe8 write (r2=1 -> [0x80074f]=1).
        unsafe { w8(REG_SYS_TICK_CTRL_74F, FLD_SYSTEM_TICK_START) };
        system_timer_ready()
    }

    /// Bounded readiness check for the 16 MHz system timer: confirms the
    /// counter at `0x740` advances within [`SYS_TICK_READY_SPIN`] iterations.
    ///
    /// Success also implies (and is corroborated by) the `RUNNING` status bit;
    /// the counter-advance test is the primary, self-clocked evidence. Returns
    /// [`PmError::SystemTimerNotRunning`] on timeout — never spins forever.
    pub fn system_timer_ready() -> Result<(), PmError> {
        unsafe {
            let start = r32(REG_SYS_TICK);
            let mut i = 0u32;
            loop {
                if r32(REG_SYS_TICK) != start {
                    return Ok(());
                }
                // Corroborating status bit, if the counter read happened to be
                // sampled between two identical values on a very slow bus.
                if r8(REG_SYS_TICK_CTRL_74F) & FLD_SYSTEM_TICK_RUNNING != 0
                    && r32(REG_SYS_TICK) != start
                {
                    return Ok(());
                }
                i += 1;
                if i >= SYS_TICK_READY_SPIN {
                    return Err(PmError::SystemTimerNotRunning);
                }
            }
        }
    }

    /// Read-only diagnostic: compute and return the SUSPEND-path
    /// [`PmDebugSnapshot`] for `wakeup_tick` **without** writing any register,
    /// arming any wake source, touching `0x66`, or entering low power.
    ///
    /// It mirrors the exact reads/computations of the SUSPEND entry
    /// ([`cpu_suspend_timer_rc`]) up to the first register write, so the
    /// returned `target32k`, `reg_20/1f/2c`, and raw `reg_07/02/05` values are
    /// what a real suspend would use. Intended for on-hardware verification of
    /// the derived comparator target and calibration before committing to a
    /// suspend. All hardware waits are bounded; every failure is typed.
    pub fn suspend_debug_snapshot(wakeup_tick: u32) -> Result<PmDebugSnapshot, PmError> {
        system_timer_ready()?;

        // Hardware 32k calibration (bounded), identical to suspend_body.
        let calib = {
            let mut c = 0u16;
            let mut i = 0u32;
            while i < CALIB_READY_SPIN {
                c = unsafe { r8(REG_32K_CALIB) as u16 | ((r8(REG_32K_CALIB + 1) as u16) << 8) };
                if c != 0 {
                    break;
                }
                nop();
                i += 1;
            }
            if c == 0 {
                return Err(PmError::CalibrationNotReady);
            }
            c
        };

        let now = unsafe { r32(REG_SYS_TICK) };
        let duration = wakeup_tick.wrapping_sub(now);
        let kind = calc::classify_duration(duration)?;
        let tick_cur = now.wrapping_add(calc::TICK_SETUP_OFFSET);
        let tick_32k_cur = pm_get_32k_tick()?;
        let adjusted =
            calc::adjusted_wake_tick_with(wakeup_tick, PROFILE_SUSPEND.early_wakeup_ticks);

        let target32k = match kind {
            SuspendKind::Short => calc::target_32k_short(adjusted, tick_cur, calib, tick_32k_cur),
            SuspendKind::Long => calc::target_32k_long(adjusted, tick_cur, calib, tick_32k_cur),
        }
        .ok_or(PmError::CalibrationNotReady)?;

        // Raw, non-destructive analog reads (no writes performed).
        let reg_07_raw = ana_r(ANA_07).ok_or(PmError::AnalogTimeout)?;
        let reg_02_raw = ana_r(ANA_02).ok_or(PmError::AnalogTimeout)?;
        let reg_05_raw = ana_r(ANA_05).ok_or(PmError::AnalogTimeout)?;

        let src = WakeupSource::Timer as u8;
        Ok(PmDebugSnapshot {
            calib,
            now_sys_tick: now,
            tick_cur,
            tick_32k_cur,
            duration_ticks: duration,
            adjusted,
            kind_is_long: matches!(kind, SuspendKind::Long),
            target32k,
            target_delta_32k: target32k.wrapping_sub(tick_32k_cur),
            reg_07_raw,
            reg_02_raw,
            reg_05_raw,
            reg_20: calc::analog_20_value(calib).unwrap_or(0),
            reg_1f: calc::analog_1f_value(calib).unwrap_or(0),
            reg_2c: calc::compose_reg_2c(src, PROFILE_SUSPEND.reg_2c_tail),
        })
    }

    /// Pre-entry analog configuration (everything from the mode-specific
    /// analog writes through the 32k target computation). Fully fallible: on
    /// **any** bounded analog-bus timeout it returns without having triggered
    /// suspend, so the caller can restore saved state safely. Runs while flash
    /// is still powered, hence flash-resident (not `.ram_code`).
    #[allow(clippy::too_many_arguments)]
    fn prepare_suspend(
        profile: &ModeProfile,
        src: u8,
        calib: u16,
        adjusted: u32,
        tick_cur: u32,
        tick_32k_cur: u32,
        kind: SuspendKind,
    ) -> Result<u32, PmError> {
        // step 1: mode-specific analog 0x02 rmw (LOW32K) or 0x04 write (SUSPEND)
        if profile.touch_reg_02 {
            // (v & ~0x07) | 0x05  (T1: +0x2ea `tbclrs r0,#7` clears the low
            // 3-bit field; +0x2ec/+0x2ee `| 5` sets it). Host-tested via
            // `calc::rmw_low3_field`.
            let v02 = calc::rmw_low3_field(ana_r(ANA_02).ok_or(PmError::AnalogTimeout)?, 0x05);
            if !ana_w(ANA_02, v02) {
                return Err(PmError::AnalogTimeout);
            }
        }
        if profile.write_reg_04 {
            // analog 0x04 = 0x48  (T1: +0x102)
            if !ana_w(ANA_04, 0x48) {
                return Err(PmError::AnalogTimeout);
            }
        }

        // step 2: retention-mode selector (T1: +0x30a / +0x10a)
        if !ana_w(ANA_7E, profile.reg_7e) {
            return Err(PmError::AnalogTimeout);
        }

        // step 3/4: wake control regs 0x2b / 0x2c (T1: +0x124 / +0x142)
        if !ana_w(ANA_2B, profile.reg_2b) {
            return Err(PmError::AnalogTimeout);
        }
        if !ana_w(ANA_2C, calc::compose_reg_2c(src, profile.reg_2c_tail)) {
            return Err(PmError::AnalogTimeout);
        }

        // step 5: reg 0x07 rmw (T1: +0x150 `tmovs r3,#7; tbclrs r0,r3` clears
        // the low 3-bit field, then +0x152/+0x154 OR the profile field value).
        // Host-tested via `calc::rmw_low3_field`. A wrong clear mask leaves
        // stale bits in the wake-config field 0x07[2:0].
        let v07 = calc::rmw_low3_field(
            ana_r(ANA_07).ok_or(PmError::AnalogTimeout)?,
            profile.reg_07_bits,
        );
        if !ana_w(ANA_07, v07) {
            return Err(PmError::AnalogTimeout);
        }

        // step 6: SUSPEND-only system reg 0x602 = 8 (T1: +0x3a0), then 0x7f
        if profile.write_reg_602 {
            unsafe { w8(REG_SYS_602, 0x08) };
        }
        if !ana_w(ANA_7F, profile.reg_7f) {
            return Err(PmError::AnalogTimeout);
        }

        // step 8/9: analog wake-timing regs 0x20 / 0x1f (T1: +0x194 / +0x1b6)
        let v20 = calc::analog_20_value(calib).ok_or(PmError::CalibrationNotReady)?;
        if !ana_w(ANA_20, v20) {
            return Err(PmError::AnalogTimeout);
        }
        let v1f = calc::analog_1f_value(calib).ok_or(PmError::CalibrationNotReady)?;
        if !ana_w(ANA_1F, v1f) {
            return Err(PmError::AnalogTimeout);
        }

        // step 10: 32k wake target (T1: +0x34e short / +0x1c6 long)
        match kind {
            SuspendKind::Short => calc::target_32k_short(adjusted, tick_cur, calib, tick_32k_cur),
            SuspendKind::Long => calc::target_32k_long(adjusted, tick_cur, calib, tick_32k_cur),
        }
        .ok_or(PmError::CalibrationNotReady)
    }

    /// Program the 32k comparator with a bounded handshake (T1: +0x1d8..0x210).
    fn program_32k_comparator(target32k: u32) -> Result<(), PmError> {
        unsafe {
            w8(REG_32K_CMD_74C, 0x2c);
            w32(REG_32K_WAKE_TICK, target32k);
            w8(REG_SYS_TICK_CTRL_74F, SYS_TICK_PROGRAM_HANDSHAKE); // BIT3
            for _ in 0..16 {
                nop();
            }
            let mut i = 0u32;
            loop {
                if r8(REG_SYS_TICK_CTRL_74F) & SYS_TICK_PROGRAM_HANDSHAKE == 0 {
                    break;
                }
                i += 1;
                if i >= TICK_HANDSHAKE_SPIN {
                    return Err(PmError::TickHandshakeTimeout);
                }
            }
            w8(REG_32K_CMD_74C, 0x20);
        }
        Ok(())
    }

    /// Shared body for both the LOW32K and SUSPEND timer-wake paths,
    /// parameterised by `profile`. Runs with the global IRQ already saved by
    /// [`with_irq_saved`].
    ///
    /// # Cleanup contract
    ///
    /// Once register `0x66` has been saved (and cleared), it is **restored on
    /// every exit path**:
    /// * a pre-entry analog-prep or comparator-handshake failure restores
    ///   `0x66`, best-effort disarms the wake source, and returns without
    ///   triggering suspend ([`PmError::AnalogTimeout`] /
    ///   [`PmError::TickHandshakeTimeout`]);
    /// * a `sleep_start` or post-wake analog failure completes **all**
    ///   mandatory cleanup (32k-timer disable handshake, system-clock restore,
    ///   `0x66` restore) *first*, then surfaces the error.
    fn suspend_body(wakeup_tick: u32, profile: &ModeProfile) -> Result<WakeStatus, PmError> {
        // The entire path relies on the free-running 16 MHz system timer
        // (0x740): `now`, the xtal-ready wait, and the post-wake tick wait all
        // read it. The pure-Rust boot never starts it, so require it running
        // explicitly (no hidden auto-start side effect). Bounded; no state
        // saved yet, so a bail-out here is clean.
        system_timer_ready()?;

        // Wait for the hardware 32k calibration to be valid (T1: +0x2e spin,
        // here bounded). No saved state yet — a bail-out here is clean.
        let calib = {
            let mut c = 0u16;
            let mut i = 0u32;
            while i < CALIB_READY_SPIN {
                c = unsafe { r8(REG_32K_CALIB) as u16 | ((r8(REG_32K_CALIB + 1) as u16) << 8) };
                if c != 0 {
                    break;
                }
                nop();
                i += 1;
            }
            if c == 0 {
                return Err(PmError::CalibrationNotReady);
            }
            c
        };

        // Duration bounds + short/long selection (T1: +0x48..0x66).
        let now = unsafe { r32(REG_SYS_TICK) };
        let duration = wakeup_tick.wrapping_sub(now);
        let kind = calc::classify_duration(duration)?;

        // Reference ticks (T1: +0x9e..0xb2).
        let tick_cur = now.wrapping_add(calc::TICK_SETUP_OFFSET);
        let tick_32k_cur = pm_get_32k_tick()?;
        let adjusted = calc::adjusted_wake_tick_with(wakeup_tick, profile.early_wakeup_ticks);

        // --- arm wake source + clear status (T1: +0x2c8 / +0xdc) ---
        let src = WakeupSource::Timer as u8; // 0x40
        if !ana_w(ANA_26, src) {
            return Err(PmError::AnalogTimeout);
        }
        if !ana_w(ANA_44, 0x0f) {
            return Err(PmError::AnalogTimeout);
        }

        // --- save + clear reg 0x66 (T1: +0x2d8 / +0xec). From HERE ON, 0x66
        //     MUST be restored on every exit path. ---
        let saved_66 = unsafe { r8(REG_SYS_CTRL_66) };
        unsafe { w8(REG_SYS_CTRL_66, 0) };

        // Pre-entry analog preparation. On failure: restore 0x66, best-effort
        // disarm the wake source, and return WITHOUT triggering suspend.
        let target32k =
            match prepare_suspend(profile, src, calib, adjusted, tick_cur, tick_32k_cur, kind) {
                Ok(t) => t,
                Err(e) => {
                    let _ = ana_w(ANA_26, 0); // best-effort disarm
                    unsafe { w8(REG_SYS_CTRL_66, saved_66) };
                    return Err(e);
                }
            };

        // Program the 32k comparator (still pre-entry, bounded handshake).
        if let Err(e) = program_32k_comparator(target32k) {
            let _ = ana_w(ANA_26, 0);
            unsafe { w8(REG_SYS_CTRL_66, saved_66) };
            return Err(e);
        }

        // Only enter if no wake source is already pending (T1: +0x216..+0x220
        // `analog_read(0x44); tshftls r3,r0,#28` tests **bit 3 only** (0x08),
        // entering suspend iff that bit is clear).
        let pre = match ana_r(ANA_44) {
            Some(v) => v,
            None => {
                let _ = ana_w(ANA_26, 0);
                unsafe { w8(REG_SYS_CTRL_66, saved_66) };
                return Err(PmError::AnalogTimeout);
            }
        };
        let entered = pre & 0x08 == 0;

        // The first error seen during entry or post-wake cleanup. We keep
        // running all mandatory cleanup, then surface it after 0x66 restore.
        let mut deferred: Result<(), PmError> = Ok(());

        if entered {
            if let Err(e) = sleep_start() {
                deferred = Err(e);
            }
        }

        // --- post-wake mandatory cleanup (always runs) ---

        // 32k elapsed → system-tick re-seed (T1: +0x22a..0x26e). If the tick
        // read fails we record the error but still perform the MMIO cleanup;
        // the hardware system timer kept counting, so we leave 0x740 as-is.
        match pm_get_32k_tick() {
            Ok(now32k) => {
                let elapsed = match kind {
                    SuspendKind::Short => calc::elapsed_ticks_short(now32k, tick_32k_cur, calib),
                    SuspendKind::Long => calc::elapsed_ticks_long(now32k, tick_32k_cur, calib),
                };
                let reseed = tick_cur
                    .wrapping_add(elapsed)
                    .wrapping_add(calc::POST_WAKE_TICK_OFFSET);
                unsafe { w32(REG_SYS_TICK, reseed) };
            }
            Err(e) => {
                if deferred.is_ok() {
                    deferred = Err(e);
                }
            }
        }

        // 32k-timer disable handshake (MMIO, always) T1: +0x250..0x272.
        unsafe {
            w8(REG_32K_CMD_74C, 0);
            for _ in 0..6 {
                nop();
            }
            w8(REG_32K_CMD_74C, 0x92);
            for _ in 0..4 {
                nop();
            }
            w8(REG_SYS_TICK_CTRL_74F, FLD_SYSTEM_TICK_START); // restart 16 MHz tick (BIT0)
        }

        // Restore the system clock before continuing (T1/T2: +0x274).
        if let Err(e) = pm_wait_xtal_ready() {
            if deferred.is_ok() {
                deferred = Err(e);
            }
        }

        // Restore reg 0x66 — mandatory, infallible (T1: +0x278).
        unsafe { w8(REG_SYS_CTRL_66, saved_66) };

        // Surface any error now that all mandatory cleanup has completed.
        deferred?;

        // Read final wake status (T1: +0x282).
        let status = ana_r(ANA_44).ok_or(PmError::AnalogTimeout)?;

        // If the timer fired, wait (bounded) for the system tick to reach the
        // requested wake tick (T1: +0x288..0x29c).
        if status & super::WAKEUP_STATUS_TIMER != 0 {
            let mut i = 0u32;
            loop {
                let d = unsafe { r32(REG_SYS_TICK) }.wrapping_sub(wakeup_tick);
                if d <= 0x4000_0000 {
                    break;
                }
                i += 1;
                if i >= WAKE_TICK_SPIN {
                    return Err(PmError::WakeTickTimeout);
                }
            }
        }

        Ok(WakeStatus {
            raw: status,
            entered_low_power: entered,
        })
    }
}

#[cfg(target_arch = "tc32")]
pub use hw::{
    cpu_sleep_timer_rc, cpu_suspend_timer_rc, suspend_debug_snapshot, system_timer_init,
    system_timer_ready,
};

// ---------------------------------------------------------------------------
// RC-32k selection + calibration (runs before suspend, flash still powered)
// ---------------------------------------------------------------------------

/// `CLK_32K_RC` selector value for [`rc_32k_init_and_cal`], matching
/// `clock.h`'s `Clk32K_TypeDef`.
pub const CLK_32K_RC: u8 = 0;

/// Select the RC 32k source and run the RC-32k calibration, exactly as
/// `clock.o:clock_32k_init(CLK_32K_RC)` followed by `clock.o:rc_32k_cal`
/// (T1). Every analog operation is bounded and returns a typed error.
///
/// This runs *before* any suspend, while flash is powered, so it uses the
/// crate's shared (IRQ-bracketed) analog helpers rather than the `.ram_code`
/// ones. It leaves register `0x750` in a state where it will become valid for
/// the timer path.
#[cfg(target_arch = "tc32")]
pub fn rc_32k_init_and_cal() -> Result<(), super::mmio::AnalogError> {
    use super::mmio::{analog_read, analog_write};

    // --- clock_32k_init(CLK_32K_RC) (T1: clock.o:clock_32k_init, mode 0) ---
    // r6 = analog_read(0x2d); r5 = analog_read(0x05) & ~0x03;
    // analog_write(0x2d, (r6 & 0x7f) | (mode<<7));   // mode 0 → bit7 clear
    // // RC branch: analog_write(0x05, r5 | 0x02);
    // T1 +0x20..0x24: `tmovs r3,#3; tbclrs r5,r3` = clear the low 2-bit 32k
    // source-select field (mask 0x03, Thumb BIC full-register), then `| 2`.
    let r2d = analog_read(0x2d)?;
    let r05 = calc::rmw_32k_src_field(analog_read(0x05)?, 0x02);
    analog_write(0x2d, r2d & 0x7f)?;
    analog_write(0x05, r05)?;

    rc_32k_cal()
}

/// T1: `clock.o:rc_32k_cal`. Kick the RC-32k calibration state machine, wait
/// (bounded) for the done bit, then latch the measured trim into `0x31/0x32`.
#[cfg(target_arch = "tc32")]
pub fn rc_32k_cal() -> Result<(), super::mmio::AnalogError> {
    use super::mmio::{AnalogError, analog_read, analog_write};

    analog_write(0x30, 0x60)?;
    analog_write(0xc6, 0xf6)?;
    analog_write(0xc6, 0xf7)?;

    // wait for the calibration-done bit (0x40) of reg 0xcf (T1: +0x1c loop).
    // Bounded, unlike the vendor's unconditional spin.
    let mut i = 0u32;
    loop {
        if analog_read(0xcf)? & 0x40 != 0 {
            break;
        }
        i += 1;
        if i >= 200_000 {
            return Err(AnalogError::Timeout);
        }
    }

    let cal_lo = analog_read(0xc9)?;
    analog_write(0x32, cal_lo)?;
    let cal_hi = analog_read(0xca)?;
    analog_write(0x31, cal_hi)?;
    analog_write(0xc6, 0xf6)?;
    analog_write(0x30, 0x20)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn system_tick_ctrl_field_bits_match_register_header() {
        // T1: platform/chip_8258/register.h
        //   FLD_SYSTEM_TICK_START   = BIT(0)
        //   FLD_SYSTEM_TICK_RUNNING = BIT(1)
        assert_eq!(FLD_SYSTEM_TICK_START, 1);
        assert_eq!(FLD_SYSTEM_TICK_RUNNING, 2);
        // The two fields are distinct single bits.
        assert_eq!(FLD_SYSTEM_TICK_START & FLD_SYSTEM_TICK_RUNNING, 0);
    }

    #[test]
    fn retention_registers_are_distinct() {
        let regs = [
            DEEP_ANA_REG0,
            DEEP_ANA_REG1,
            DEEP_ANA_REG2,
            DEEP_ANA_REG6,
            DEEP_ANA_REG7,
            DEEP_ANA_REG8,
            DEEP_ANA_REG9,
            DEEP_ANA_REG10,
        ];
        for (i, a) in regs.iter().enumerate() {
            for b in &regs[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn retention_register_addresses_match_documented_scratch_registers() {
        assert_eq!(RetentionRegister::Reg6.address(), DEEP_ANA_REG6);
        assert_eq!(RetentionRegister::Reg7.address(), DEEP_ANA_REG7);
        assert_eq!(RetentionRegister::Reg8.address(), DEEP_ANA_REG8);
        assert_eq!(RetentionRegister::Reg9.address(), DEEP_ANA_REG9);
        assert_eq!(RetentionRegister::Reg10.address(), DEEP_ANA_REG10);
    }

    #[test]
    fn retention_register_cannot_name_power_cycle_or_reserved_registers() {
        let selectable = [
            RetentionRegister::Reg6.address(),
            RetentionRegister::Reg7.address(),
            RetentionRegister::Reg8.address(),
            RetentionRegister::Reg9.address(),
            RetentionRegister::Reg10.address(),
        ];
        for reserved in [DEEP_ANA_REG0, DEEP_ANA_REG1, DEEP_ANA_REG2] {
            assert!(!selectable.contains(&reserved));
        }
    }

    #[test]
    fn wakeup_source_bits_are_disjoint_powers_of_two() {
        let sources = [
            WakeupSource::Pad,
            WakeupSource::Core,
            WakeupSource::Timer,
            WakeupSource::Comparator,
        ];
        let mut seen = 0u8;
        for source in sources {
            let bit = source as u8;
            assert_eq!(bit & (bit - 1), 0, "not a single bit: {bit:#x}");
            assert_eq!(seen & bit, 0, "overlapping wakeup bit: {bit:#x}");
            seen |= bit;
        }
    }

    #[test]
    fn timer_source_matches_analog_0x26_encoding() {
        // The value written to analog 0x26 for a timer wake is 0x40.
        assert_eq!(WakeupSource::Timer as u8, 0x40);
    }

    #[test]
    fn wake_status_decode_reports_only_requested_bits() {
        let s = WakeStatus {
            raw: WAKEUP_STATUS_TIMER,
            entered_low_power: true,
        };
        assert!(s.woke_by_timer());
        assert!(!s.woke_by_pad());
        assert!(!s.woke_by_core());
        assert!(!s.woke_by_comparator());
        assert!(s.entered_low_power());
        assert_eq!(s.raw(), 0x02);
    }

    #[test]
    fn wake_status_no_entry_when_source_already_pending() {
        let s = WakeStatus {
            raw: WAKEUP_STATUS_PAD,
            entered_low_power: false,
        };
        assert!(!s.entered_low_power());
        assert!(s.woke_by_pad());
    }

    #[test]
    fn deepsleep_mode_low32k_value_is_analog_7e_payload() {
        assert_eq!(DEEPSLEEP_MODE_RET_SRAM_LOW32K, 0x07);
    }

    #[test]
    fn status_bits_are_disjoint_low_nibble() {
        let all =
            WAKEUP_STATUS_COMPARATOR | WAKEUP_STATUS_TIMER | WAKEUP_STATUS_CORE | WAKEUP_STATUS_PAD;
        assert_eq!(all, 0x0f);
    }
}
