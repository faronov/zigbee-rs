//! EFR32MG1 Series 1 (EFR32xG1) low-energy wake timer and EM2 entry.
//!
//! This module is the first "safe" power-management building block towards
//! a future Embassy `EM2`-aware executor. It deliberately does **not** touch
//! flash/NVM or the radio, and it does not replace the SysTick Embassy time
//! driver used by production firmware (`examples/efr32mg1-sensor/src/time_driver.rs`).
//!
//! # Why RTCC and not a generic LETIMER/CRYOTIMER
//!
//! GSDK 4.5.0's `sleeptimer` component auto-selects its hardware backend
//! per-part. For EFR32xG1 (`_SILICON_LABS_GECKO_INTERNAL_SDID_80`), the RTCC
//! branch is compiled in (`platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.c`)
//! and the reference `zigbee_sensor_tradfri_*.slcp` projects leave
//! `SL_SLEEPTIMER_PERIPHERAL` at `SL_SLEEPTIMER_PERIPHERAL_DEFAULT`, which
//! resolves to RTCC on this part. RTCC also sits on the CMU "LFE" branch
//! (`CMU_LFECLKEN0_RTCC`), and `sli_sleeptimer_set_pm_em_requirement()` in
//! that same file treats both `CMU_LFECLKSEL_LFE_LFRCO` and `..._LFXO` as
//! EM2-capable sources for it — i.e. RTCC-from-LFRCO is the Silicon-Labs
//! -sanctioned Series-1 EM2 wake source, not a guess.
//!
//! # Why LFRCO and not LFXO
//!
//! The native reference project
//! (`~/efr32mg1p-bme280-zigbee-sensor/firmware/config/`) ships no
//! `sl_device_init_lfxo_config.h`, i.e. the LFXO device-init component is not
//! included and no external 32.768 kHz crystal is wired/trimmed for the
//! TRÅDFRI module in that proven build. LFRCO requires no board-specific
//! crystal, is always present, and is explicitly accepted by the sleeptimer
//! EM2-requirement check above. Its nominal frequency, `EFR32_LFRCO_FREQ`, is
//! defined as exactly `32768` Hz in
//! `platform/Device/SiliconLabs/EFR32MG1P/Source/system_efr32mg1p.c`
//! (`SystemLFRCOClockGet()`), which is the constant used throughout this
//! module — not a guessed datasheet value.
//!
//! # DCDC LN handshake workaround
//!
//! `_SILICON_LABS_GECKO_INTERNAL_SDID_80` parts have an errata
//! (`ERRATA_FIX_DCDC_LNHS_BLOCK_ENABLE` in `platform/emlib/src/em_emu.c`)
//! where the DCDC low-noise (LN) handshake can wedge. GSDK's own fix
//! (`dcdcHsFixLnBlock()`) spin-waits forever on an undocumented status peek:
//! `#define EMU_DCDCSTATUS (*(volatile uint32_t *)(EMU_BASE + 0x7C))`,
//! bit 16. The reference `app.c` (`app_apply_dcdc_lnhs_workaround()`) uses
//! the identical address/bit but replaces the unbounded spin with an
//! unconditional bypass, called every AF-init and runtime-poll cycle so a
//! wedge can never block IRQs or sleeptimer wake-ups. This module reproduces
//! that same address/bit (cross-checked against GSDK 4.5.0's own source, not
//! guessed) and the same "bypass instead of spin" strategy, but as a bounded,
//! typed-error operation (see [`apply_dcdc_lnhs_workaround`]).

// ── Pure, hardware-independent helpers (host-testable) ──────────

/// Errors from the power-management HAL. All hardware waits in this module
/// are bounded; none of them can hang forever.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PmError {
    /// LFRCO did not report ready within the bounded poll budget.
    LfrcoStartTimeout,
    /// CMU did not clear the LFE clock-enable sync-busy bit in time.
    LfeClockSyncTimeout,
    /// RTCC registers were accessible, but its free-running counter did not
    /// start after enabling the LFE clock and RTCC.
    RtccStartTimeout,
    /// EMU DCDC control register transfer did not clear busy in time.
    DcdcSyncTimeout,
    /// No RTCC wake event (and no observed deadline crossing) within the
    /// bounded number of WFI attempts.
    WakeTimeout,
}

/// Why `sleep_until_wake` returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeCause {
    /// The RTCC CC0 compare-match ISR incremented the wake-event counter.
    RtccCompare,
    /// The RTCC free-running counter crossed the deadline, but the ISR
    /// counter did not move — the core woke for some other reason (or the
    /// interrupt was otherwise not observed). Reported instead of silently
    /// treated as success so callers can log a WARN.
    Spurious,
}

/// Runs `poll` up to `max_attempts` times, returning as soon as
/// it reports success. This isolates bounded-retry bookkeeping from the
/// actual hardware access so the exact same retry logic used by every wait
/// in this module can be unit-tested on host.
pub fn poll_bounded<F: FnMut() -> bool>(max_attempts: u32, mut poll: F) -> Result<(), ()> {
    for _ in 0..max_attempts {
        if poll() {
            return Ok(());
        }
    }
    Err(())
}

/// Converts a millisecond duration to RTCC ticks at `clock_hz`, using a
/// 64-bit intermediate so multi-second durations never overflow `u32` math.
pub const fn ms_to_ticks(ms: u32, clock_hz: u32) -> u32 {
    (((ms as u64) * (clock_hz as u64)) / 1000) as u32
}

/// Converts an RTCC tick count back to milliseconds at `clock_hz`.
pub const fn ticks_to_ms(ticks: u32, clock_hz: u32) -> u32 {
    (((ticks as u64) * 1000) / (clock_hz as u64)) as u32
}

/// Computes a deadline on a 32-bit free-running counter that wraps at
/// `2^32`. Wrapping add is intentional and correct here.
pub const fn deadline_from_now(now: u32, ticks_from_now: u32) -> u32 {
    now.wrapping_add(ticks_from_now)
}

/// Converts an absolute wake deadline to the RTCC output-compare register
/// value. EFR32xG1 raises the compare interrupt when `CNT == CCV + 1`, so the
/// programmed value is one tick before the requested deadline.
pub const fn compare_from_deadline(deadline: u32) -> u32 {
    deadline.wrapping_sub(1)
}

/// Wrap-safe "has `now` reached `deadline`" check for a 32-bit free-running
/// counter, using the standard signed-subtraction idiom so a single
/// wraparound between `now` and `deadline` is still handled correctly.
pub const fn ticks_reached(now: u32, deadline: u32) -> bool {
    (now.wrapping_sub(deadline) as i32) >= 0
}

/// Wrap-safe elapsed-ticks calculation (`now - before`), valid across a
/// single wraparound of the 32-bit counter.
pub const fn elapsed_ticks(before: u32, now: u32) -> u32 {
    now.wrapping_sub(before)
}

/// Checks that `elapsed_ticks` is within `tolerance_percent` of
/// `expected_ticks`. Used to sanity-check that RTCC actually advanced by
/// roughly the requested amount across an EM2 sleep (LFRCO is not
/// crystal-accurate, so an exact match is not expected).
pub fn progressed_within_tolerance(elapsed: u32, expected: u32, tolerance_percent: u32) -> bool {
    let tolerance = ((expected as u64) * (tolerance_percent as u64)) / 100;
    let lo = (expected as u64).saturating_sub(tolerance);
    let hi = (expected as u64) + tolerance;
    let elapsed = elapsed as u64;
    elapsed >= lo && elapsed <= hi
}

/// Classifies a wake based on whether the RTCC ISR event counter moved.
pub const fn classify_wake(events_before: u32, events_after: u32) -> WakeCause {
    if events_before != events_after {
        WakeCause::RtccCompare
    } else {
        WakeCause::Spurious
    }
}

/// A single SRAM canary mismatch, reported with enough detail to log which
/// canary slot was corrupted and what value it held instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanaryMismatch {
    pub index: usize,
    pub expected: u32,
    pub actual: u32,
}

/// Deterministic, per-slot canary pattern. Uses a fixed-point multiplicative
/// hash (Knuth's golden-ratio constant) purely to give each slot a distinct,
/// recomputable, non-trivial-to-accidentally-match pattern; there is no
/// cryptographic requirement here.
pub const fn canary_pattern(seed: u32, index: usize) -> u32 {
    seed ^ (index as u32).wrapping_mul(0x9E37_79B1)
}

/// Validates a slice of canary words against the expected per-slot pattern,
/// returning the first mismatch found (if any).
pub fn validate_canaries(seed: u32, values: &[u32]) -> Result<(), CanaryMismatch> {
    for (index, &actual) in values.iter().enumerate() {
        let expected = canary_pattern(seed, index);
        if actual != expected {
            return Err(CanaryMismatch {
                index,
                expected,
                actual,
            });
        }
    }
    Ok(())
}

// ── 64-bit monotonic extension and long-deadline helpers ──────────
//
// Added for the Embassy RTCC time driver
// (`examples/efr32mg1-sensor/src/time_driver.rs`). These are pure,
// host-testable extensions of the 32-bit RTCC primitives above; nothing
// here is read by `diag-em2` or changes any existing function's behavior.

/// RTCC `IF`/`IEN`/`IFC` bit for the free-running counter overflow flag
/// (bit 0). Cross-checked against GSDK 4.5.0 `efr32mg1p_rtcc.h`
/// (`RTCC_IF_OF`). Kept as a separate public constant from the private,
/// already-proven `hw::RTCC_IF_CC0` used by `handle_interrupt`/`arm_wake` —
/// duplicated on purpose so nothing here can ever change what `diag-em2`
/// already exercises on hardware.
pub const RTCC_IF_OF_BIT: u32 = 1 << 0;

/// RTCC `IF`/`IEN`/`IFC` bit for the channel-0 compare-match flag (bit 1).
/// Same numeric value as `hw::RTCC_IF_CC0`; see [`RTCC_IF_OF_BIT`] for why
/// it is intentionally re-declared here instead of shared.
pub const RTCC_IF_CC0_BIT: u32 = 1 << 1;

/// Which RTCC interrupt sources were latched the moment they were sampled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RtccFlags {
    pub cc0: bool,
    pub overflow: bool,
}

/// Pure decode of a raw `RTCC_IF` register snapshot. Split out from the
/// hardware read/clear so it is host-testable on its own.
pub const fn decode_pending_flags(raw_if: u32) -> RtccFlags {
    RtccFlags {
        cc0: raw_if & RTCC_IF_CC0_BIT != 0,
        overflow: raw_if & RTCC_IF_OF_BIT != 0,
    }
}

/// Extends a 32-bit free-running counter reading to 64 bits given the
/// current software "epoch" (count of overflow wraps observed so far) and
/// whether the hardware overflow flag is *currently* latched.
///
/// `overflow_pending` covers the case where the counter has already
/// wrapped in hardware but the overflow ISR (which increments the epoch
/// and clears the flag) has not run yet — e.g. because the caller is
/// inside a critical section. In that case the wrap is still "owed" to the
/// epoch, so it is added in locally rather than trusting the stale
/// software counter.
pub const fn extend_to_64(epoch: u32, low: u32, overflow_pending: bool) -> u64 {
    let epoch64 = if overflow_pending {
        (epoch as u64) + 1
    } else {
        epoch as u64
    };
    (epoch64 << 32) | (low as u64)
}

/// Resolves a 64-bit "now" from an (epoch-before, low-before,
/// overflow-pending, low-after, epoch-after) sample set, or reports that the
/// sample was torn (the
/// overflow ISR ran *during* sampling, changing the epoch) and must be
/// retaken.
///
/// This is the pure decision core of `hw::now64`'s retry loop: as long as
/// `epoch_before == epoch_after`, no overflow ISR executed anywhere between
/// the two epoch reads. The second low-word sample is returned. A wrap is
/// considered owed either when the hardware overflow flag was observed or
/// when `low_after < low_before`, which covers a wrap occurring after the
/// flag sample but before the second counter sample.
pub const fn now64_from_samples(
    epoch_before: u32,
    epoch_after: u32,
    low_before: u32,
    low_after: u32,
    overflow_pending: bool,
) -> Option<u64> {
    if epoch_before != epoch_after {
        None
    } else {
        Some(extend_to_64(
            epoch_before,
            low_after,
            overflow_pending || low_after < low_before,
        ))
    }
}

/// Minimum ticks-from-now this HAL will ever arm a compare for.
///
/// Mirrors GSDK 4.5.0's own `sleeptimer_hal_rtcc.c`
/// `SLEEPTIMER_COMPARE_MIN_DIFF` (`2 + 1`): writing `CC0_CCV` crosses from
/// the core clock domain into the low-frequency RTCC domain and needs a
/// few LFRCO ticks to synchronize. Arming a compare closer than this to
/// `now` risks the free-running counter reaching the target before the
/// write has taken effect, which would silently miss the match until the
/// counter wraps ~36 hours later. Not a guess: the identical value and
/// justification are in GSDK's own `sleeptimer_hal_set_compare()` comment.
pub const MIN_ARM_TICKS: u32 = 2 + 1;

/// Maximum ticks-from-now this HAL will arm a single compare for: half the
/// 32-bit counter range, leaving comfortable margin below the point where
/// `deadline_from_now`/`ticks_reached` wraparound math would become
/// ambiguous. A caller with a deadline further out than this must be
/// re-armed at least once more after this "hop" fires (see
/// `ticks_from_now_clamped` and the Embassy driver's `rearm`).
pub const MAX_ARM_TICKS: u32 = u32::MAX / 2;

/// Given the current 64-bit time and a target 64-bit deadline, returns how
/// many raw ticks a single hardware compare should be armed for:
///
/// - `None` if `deadline` has already been reached (caller should wake
///   immediately without touching hardware at all).
/// - `Some(MIN_ARM_TICKS)` if the deadline is nearer than the safe
///   compare-write synchronization margin.
/// - `Some(MAX_ARM_TICKS)` if the deadline is further out than a single
///   32-bit compare can represent unambiguously (the caller must treat the
///   resulting wake as an intermediate hop and re-arm).
/// - `Some(remaining)` otherwise.
pub const fn ticks_from_now_clamped(now: u64, deadline: u64) -> Option<u32> {
    if deadline <= now {
        return None;
    }
    let remaining = deadline - now;
    Some(if remaining > MAX_ARM_TICKS as u64 {
        MAX_ARM_TICKS
    } else if remaining < MIN_ARM_TICKS as u64 {
        MIN_ARM_TICKS
    } else {
        remaining as u32
    })
}

// ── Hardware access (real target only) ───────────────────────────

#[cfg(target_arch = "arm")]
mod hw {
    use super::{poll_bounded, PmError, WakeCause};
    use core::sync::atomic::{AtomicU32, Ordering};

    // CMU (shared base with efr32mg1_hal::clock; offsets cross-checked
    // against GSDK 4.5.0 `efr32mg1p_cmu.h` `CMU_TypeDef`).
    const CMU_BASE: u32 = 0x400E_4000;
    const CMU_OSCENCMD: u32 = CMU_BASE + 0x060;
    const CMU_LFECLKSEL: u32 = CMU_BASE + 0x088;
    const CMU_STATUS: u32 = CMU_BASE + 0x090;
    const CMU_HFBUSCLKEN0: u32 = CMU_BASE + 0x0B0;
    const CMU_LFECLKEN0: u32 = CMU_BASE + 0x0F0;
    const CMU_HFPRESC: u32 = CMU_BASE + 0x100;
    const CMU_SYNCBUSY: u32 = CMU_BASE + 0x140;

    const CMU_OSCENCMD_LFRCOEN: u32 = 1 << 6;
    const CMU_STATUS_LFRCORDY: u32 = 1 << 7;
    const CMU_LFECLKSEL_LFE_MASK: u32 = 0x7;
    const CMU_LFECLKSEL_LFE_LFRCO: u32 = 0x1;
    const CMU_HFBUSCLKEN0_LE: u32 = 1 << 0;
    const CMU_LFECLKEN0_RTCC: u32 = 1 << 0;
    const CMU_HFPRESC_HFCLKLEPRESC_MASK: u32 = 1 << 24;
    const CMU_HFPRESC_HFCLKLEPRESC_DIV2: u32 = 0;
    const CMU_SYNCBUSY_LFECLKEN0: u32 = 1 << 16;

    /// `EFR32_LFRCO_FREQ` from GSDK 4.5.0
    /// `platform/Device/SiliconLabs/EFR32MG1P/Source/system_efr32mg1p.c`.
    pub const LFRCO_HZ: u32 = 32_768;

    // RTCC (offsets cross-checked against GSDK 4.5.0 `efr32mg1p_rtcc.h`
    // `RTCC_TypeDef` / `RTCC_CC_TypeDef`).
    const RTCC_BASE: u32 = 0x4004_2000;
    const RTCC_CTRL: u32 = RTCC_BASE + 0x00;
    const RTCC_CNT: u32 = RTCC_BASE + 0x08;
    const RTCC_IF: u32 = RTCC_BASE + 0x18;
    const RTCC_IFC: u32 = RTCC_BASE + 0x20;
    const RTCC_IEN: u32 = RTCC_BASE + 0x24;
    const RTCC_CC0_CTRL: u32 = RTCC_BASE + 0x40;
    const RTCC_CC0_CCV: u32 = RTCC_BASE + 0x44;

    const RTCC_CTRL_ENABLE: u32 = 1 << 0;
    const RTCC_CC_CTRL_MODE_OUTPUTCOMPARE: u32 = 0x2;
    const RTCC_IF_CC0: u32 = 1 << 1;
    const RTCC_INTERRUPT_MASK: u32 = 0x7FF;

    /// Software wrap counter ("epoch"), incremented once per RTCC overflow
    /// (`RTCC_IF_OF`) by whichever `RTCC` handler calls
    /// [`bump_wrap_count`] — only the Embassy time driver in
    /// `examples/efr32mg1-sensor/src/time_driver.rs` does; `diag-em2` never
    /// enables the overflow interrupt (see [`enable_overflow_interrupt`]),
    /// so this stays at `0` for that binary.
    static WRAP_COUNT: AtomicU32 = AtomicU32::new(0);

    // EMU (offsets cross-checked against GSDK 4.5.0 `efr32mg1p_emu.h`
    // `EMU_TypeDef`). `EMU_DCDCSTATUS` at `EMU_BASE + 0x7C` is not part of
    // the documented struct but is the literal macro GSDK's own
    // `em_emu.c:303` uses for the SDID-80 LN-handshake errata check.
    const EMU_BASE: u32 = 0x400E_3000;
    const EMU_PWRLOCK: u32 = EMU_BASE + 0x34;
    const EMU_DCDCCTRL: u32 = EMU_BASE + 0x40;
    const EMU_DCDCCLIMCTRL: u32 = EMU_BASE + 0x54;
    const EMU_DCDCSYNC: u32 = EMU_BASE + 0x78;
    const EMU_DCDCSTATUS: u32 = EMU_BASE + 0x7C;

    const EMU_DCDCCTRL_DCDCMODE_MASK: u32 = 0x3;
    const EMU_DCDCCTRL_DCDCMODE_BYPASS: u32 = 0x0;
    const EMU_DCDCCTRL_DCDCMODE_LOWNOISE: u32 = 0x1;
    const EMU_DCDCCLIMCTRL_BYPLIMEN: u32 = 1 << 13;
    const EMU_DCDCSYNC_DCDCCTRLBUSY: u32 = 1 << 0;
    const EMU_DCDCSTATUS_LNRUNNING: u32 = 1 << 16;
    const EMU_PWRLOCK_UNLOCK_KEY: u32 = 0x0000_ADE8;
    const EMU_PWRLOCK_LOCKED_VALUE: u32 = 0x0000_0001;
    const EMU_PWRLOCK_LOCK_VALUE: u32 = 0x0000_0000;

    // ARMv7-M core (CMSIS `core_cm4.h`): SCB_BASE = 0xE000_E000 + 0xD00,
    // SCR at +0x10, SLEEPDEEP at bit 2.
    const SCB_SCR: u32 = 0xE000_ED10;
    const SCB_SCR_SLEEPDEEP: u32 = 1 << 2;

    const DEFAULT_TIMEOUT: u32 = 1_000_000;
    /// Bounded number of extra WFI attempts allowed if a wake occurs before
    /// our own RTCC ISR observably fires (e.g. a debugger-induced wake).
    const DEFAULT_MAX_SPURIOUS_WAKES: u32 = 8;

    static WAKE_EVENTS: AtomicU32 = AtomicU32::new(0);

    #[inline]
    unsafe fn read(address: u32) -> u32 {
        unsafe { core::ptr::read_volatile(address as *const u32) }
    }

    #[inline]
    unsafe fn write(address: u32, value: u32) {
        unsafe { core::ptr::write_volatile(address as *mut u32, value) }
    }

    #[inline]
    unsafe fn modify(address: u32, mask: u32, value: u32) {
        let current = unsafe { read(address) };
        unsafe { write(address, (current & !mask) | (value & mask)) };
    }

    fn flag_set(address: u32, mask: u32) -> bool {
        unsafe { read(address) & mask == mask }
    }

    fn flag_clear(address: u32, mask: u32) -> bool {
        unsafe { read(address) & mask == 0 }
    }

    /// Brings up LFRCO, routes it to RTCC over the CMU "LFE" branch, and
    /// configures RTCC channel 0 as a free-running output-compare wake
    /// source. Does not touch flash, NVM, or the radio.
    pub fn init() -> Result<(), PmError> {
        // EFR32xG1 gates CPU access to all low-energy peripheral registers
        // behind HFBUSCLKEN0.LE. GSDK's device initialization enables
        // `cmuClock_HFLE` before its sleeptimer HAL reaches RTCC; this
        // standalone HAL must do the equivalent explicitly.
        unsafe {
            modify(
                CMU_HFPRESC,
                CMU_HFPRESC_HFCLKLEPRESC_MASK,
                CMU_HFPRESC_HFCLKLEPRESC_DIV2,
            );
            modify(CMU_HFBUSCLKEN0, CMU_HFBUSCLKEN0_LE, CMU_HFBUSCLKEN0_LE);
        }

        unsafe { write(CMU_OSCENCMD, CMU_OSCENCMD_LFRCOEN) };
        poll_bounded(DEFAULT_TIMEOUT, || {
            flag_set(CMU_STATUS, CMU_STATUS_LFRCORDY)
        })
        .map_err(|_| PmError::LfrcoStartTimeout)?;

        unsafe {
            modify(
                CMU_LFECLKSEL,
                CMU_LFECLKSEL_LFE_MASK,
                CMU_LFECLKSEL_LFE_LFRCO,
            );
            modify(CMU_LFECLKEN0, CMU_LFECLKEN0_RTCC, CMU_LFECLKEN0_RTCC);
        }
        poll_bounded(DEFAULT_TIMEOUT, || {
            flag_clear(CMU_SYNCBUSY, CMU_SYNCBUSY_LFECLKEN0)
        })
        .map_err(|_| PmError::LfeClockSyncTimeout)?;

        unsafe {
            write(RTCC_IEN, 0);
            write(RTCC_IFC, RTCC_INTERRUPT_MASK);
            write(RTCC_CC0_CTRL, RTCC_CC_CTRL_MODE_OUTPUTCOMPARE);
            write(RTCC_CTRL, RTCC_CTRL_ENABLE);
        }
        let initial_counter = now();
        poll_bounded(DEFAULT_TIMEOUT, || {
            flag_set(RTCC_CTRL, RTCC_CTRL_ENABLE) && now() != initial_counter
        })
        .map_err(|_| PmError::RtccStartTimeout)?;
        Ok(())
    }

    /// Reads the free-running 32-bit RTCC counter (LFRCO ticks since RTCC
    /// was enabled, wrapping at `2^32`).
    pub fn now() -> u32 {
        unsafe { read(RTCC_CNT) }
    }

    /// Arms CC0 to fire `ticks_from_now` RTCC ticks in the future and
    /// returns the absolute deadline (for the caller's own bookkeeping /
    /// progression checks).
    #[inline(never)]
    pub fn arm_wake(ticks_from_now: u32) -> u32 {
        let deadline = super::deadline_from_now(now(), ticks_from_now);
        unsafe {
            write(RTCC_CC0_CCV, super::compare_from_deadline(deadline));
            write(RTCC_IFC, RTCC_IF_CC0);
            modify(RTCC_IEN, RTCC_IF_CC0, RTCC_IF_CC0);
        }
        deadline
    }

    /// Disables the CC0 wake interrupt without touching the free-running
    /// counter.
    #[inline(never)]
    pub fn disarm_wake() {
        unsafe { modify(RTCC_IEN, RTCC_IF_CC0, 0) };
    }

    /// Must be called from the application's `RTCC` interrupt handler (see
    /// `examples/efr32mg1-sensor` `diag-em2` wiring). Clears the hardware
    /// flag and records the wake event.
    pub fn handle_interrupt() {
        unsafe { write(RTCC_IFC, RTCC_IF_CC0) };
        WAKE_EVENTS.fetch_add(1, Ordering::SeqCst);
    }

    /// Current wake-event count, as observed by [`handle_interrupt`].
    pub fn wake_events() -> u32 {
        WAKE_EVENTS.load(Ordering::SeqCst)
    }

    /// Enables the RTCC overflow (`OF`) interrupt on top of whatever
    /// [`init`] already configured. Never called by `diag-em2` (which only
    /// calls [`init`]), so this cannot regress its hardware-proven
    /// behavior; only the Embassy time driver calls it, to keep its 64-bit
    /// epoch (see [`now64`]) advancing across RTCC's ~36-hour 32-bit
    /// wraparound even while no CC0 wake is armed.
    pub fn enable_overflow_interrupt() {
        unsafe { modify(RTCC_IEN, super::RTCC_IF_OF_BIT, super::RTCC_IF_OF_BIT) };
    }

    /// True if the RTCC overflow flag is currently latched in hardware,
    /// regardless of whether the `OF` interrupt is enabled/unmasked. This
    /// is a pure peek — it does not clear the flag — used by [`now64`] to
    /// detect a wrap that hardware has already recorded but whose ISR
    /// (which calls [`bump_wrap_count`]) has not run yet.
    pub fn overflow_flag_pending() -> bool {
        flag_set(RTCC_IF, super::RTCC_IF_OF_BIT)
    }

    /// Reads and clears whichever of the CC0-compare and overflow flags
    /// are currently pending, atomically with respect to each other (a
    /// single `IF` read followed by a single `IFC` write of exactly the
    /// bits observed). Intended to be the *only* thing the Embassy time
    /// driver's `RTCC` handler reads directly from hardware; it then calls
    /// [`bump_wrap_count`] itself when the returned flags say `overflow`.
    pub fn take_pending_flags() -> super::RtccFlags {
        let raw = unsafe { read(RTCC_IF) };
        unsafe {
            write(
                RTCC_IFC,
                raw & (super::RTCC_IF_CC0_BIT | super::RTCC_IF_OF_BIT),
            )
        };
        super::decode_pending_flags(raw)
    }

    /// Advances the 64-bit epoch by one wrap. Must be called (at most once
    /// per observed overflow) after [`take_pending_flags`] reports
    /// `overflow: true`; see the time driver's `RTCC` handler.
    pub fn bump_wrap_count() -> u32 {
        WRAP_COUNT.fetch_add(1, Ordering::SeqCst) + 1
    }

    /// Current wrap ("epoch") counter, as advanced by [`bump_wrap_count`].
    pub fn wrap_count() -> u32 {
        WRAP_COUNT.load(Ordering::SeqCst)
    }

    /// Race-safe 64-bit extension of [`now`] across RTCC's 32-bit
    /// wraparound (`2^32` ticks at [`LFRCO_HZ`] ≈ 36.4 hours).
    ///
    /// Retries if the wrap counter itself changed while sampling (meaning
    /// the overflow ISR executed *during* the read — only possible when
    /// interrupts are not masked); otherwise falls back to
    /// [`overflow_flag_pending`] to catch a wrap that has already happened
    /// in hardware but whose ISR has not run yet (the case when called
    /// from inside a critical section, e.g. from `schedule_wake`). See
    /// [`now64_from_samples`] for the pure decision logic this wraps.
    #[inline(never)]
    pub fn now64() -> u64 {
        loop {
            let epoch_before = wrap_count();
            let low_before = now();
            let pending = overflow_flag_pending();
            let low_after = now();
            let epoch_after = wrap_count();
            if let Some(value) =
                super::now64_from_samples(epoch_before, epoch_after, low_before, low_after, pending)
            {
                return value;
            }
        }
    }

    fn dcdc_mode() -> u32 {
        unsafe { read(EMU_DCDCCTRL) & EMU_DCDCCTRL_DCDCMODE_MASK }
    }

    fn dcdc_ln_running() -> bool {
        unsafe { read(EMU_DCDCSTATUS) & EMU_DCDCSTATUS_LNRUNNING != 0 }
    }

    fn wait_dcdc_sync() -> Result<(), PmError> {
        poll_bounded(DEFAULT_TIMEOUT, || {
            flag_clear(EMU_DCDCSYNC, EMU_DCDCSYNC_DCDCCTRLBUSY)
        })
        .map_err(|_| PmError::DcdcSyncTimeout)
    }

    /// Series-1 (SDID 80) DCDC LN-handshake safety gate. If, and only if,
    /// the DCDC is currently programmed for low-noise mode but the hardware
    /// has not reported the handshake as running, forces EM0 DCDC to bypass
    /// so a wedged handshake can never block IRQs or RTCC wake-ups. This is
    /// a no-op (and returns immediately) whenever DCDC is not in low-noise
    /// mode, which is the reset default and is expected on every diag-em2
    /// boot since this diagnostic never programs DCDC itself.
    pub fn apply_dcdc_lnhs_workaround() -> Result<(), PmError> {
        if dcdc_mode() != EMU_DCDCCTRL_DCDCMODE_LOWNOISE || dcdc_ln_running() {
            return Ok(());
        }

        let was_locked = unsafe { read(EMU_PWRLOCK) } == EMU_PWRLOCK_LOCKED_VALUE;
        unsafe { write(EMU_PWRLOCK, EMU_PWRLOCK_UNLOCK_KEY) };

        wait_dcdc_sync()?;
        unsafe {
            modify(
                EMU_DCDCCLIMCTRL,
                EMU_DCDCCLIMCTRL_BYPLIMEN,
                EMU_DCDCCLIMCTRL_BYPLIMEN,
            )
        };
        wait_dcdc_sync()?;
        unsafe {
            modify(
                EMU_DCDCCTRL,
                EMU_DCDCCTRL_DCDCMODE_MASK,
                EMU_DCDCCTRL_DCDCMODE_BYPASS,
            )
        };
        wait_dcdc_sync()?;
        unsafe { modify(EMU_DCDCCLIMCTRL, EMU_DCDCCLIMCTRL_BYPLIMEN, 0) };

        if was_locked {
            unsafe { write(EMU_PWRLOCK, EMU_PWRLOCK_LOCK_VALUE) };
        }
        Ok(())
    }

    #[inline(always)]
    fn wfi() {
        // SAFETY: WFI is a hint instruction; it only suspends execution
        // until the next interrupt/event and has no memory side effects.
        unsafe { core::arch::asm!("wfi", options(nomem, nostack, preserves_flags)) };
    }

    /// Applies the DCDC safety gate, sets `SLEEPDEEP`, and executes one
    /// `WFI`. Returns once, either because an interrupt fired or (rarely)
    /// because the core was never actually asleep (e.g. a pending unmasked
    /// interrupt at call time).
    fn enter_em2_once() -> Result<(), PmError> {
        apply_dcdc_lnhs_workaround()?;
        unsafe { modify(SCB_SCR, SCB_SCR_SLEEPDEEP, SCB_SCR_SLEEPDEEP) };
        wfi();
        unsafe { modify(SCB_SCR, SCB_SCR_SLEEPDEEP, 0) };
        Ok(())
    }

    /// Sleeps in EM2 until the RTCC deadline previously armed with
    /// [`arm_wake`] is reached, retrying WFI up to `max_spurious_wakes`
    /// times if the core wakes for an unrelated reason first. Bounded: this
    /// can never hang forever — it returns [`PmError::WakeTimeout`] once the
    /// retry budget is exhausted after unrelated interrupts without observing
    /// progress towards `deadline`. As with any hardware sleep primitive, a
    /// completely missing wake source cannot be converted into a software
    /// timeout while the core remains suspended in `WFI`.
    pub fn sleep_until_wake(deadline: u32, max_spurious_wakes: u32) -> Result<WakeCause, PmError> {
        let before = wake_events();
        let mut attempts = 0u32;
        loop {
            enter_em2_once()?;
            let after = wake_events();
            if after != before {
                return Ok(super::classify_wake(before, after));
            }
            if super::ticks_reached(now(), deadline) {
                // The counter crossed the deadline even though our own ISR
                // did not observably increment it. Report this rather than
                // silently treating it as a clean RTCC wake.
                return Ok(WakeCause::Spurious);
            }
            attempts += 1;
            if attempts > max_spurious_wakes {
                return Err(PmError::WakeTimeout);
            }
        }
    }

    /// Convenience wrapper: arms CC0 `ticks_from_now` ticks out and sleeps
    /// until it fires (or the default spurious-wake budget is exhausted).
    pub fn sleep_for_ticks(ticks_from_now: u32) -> Result<(u32, WakeCause), PmError> {
        let deadline = arm_wake(ticks_from_now);
        let cause = sleep_until_wake(deadline, DEFAULT_MAX_SPURIOUS_WAKES)?;
        disarm_wake();
        Ok((deadline, cause))
    }

    /// Like [`sleep_for_ticks`], but classifies success purely from the
    /// free-running counter reaching `deadline` (via [`super::ticks_reached`])
    /// instead of [`wake_events`]/[`handle_interrupt`].
    ///
    /// [`sleep_until_wake`]'s classification depends on the caller's `RTCC`
    /// handler being [`handle_interrupt`] — true for `diag-em2`, but *not*
    /// true for any profile that links
    /// `examples/efr32mg1-sensor/src/time_driver.rs`, whose own `RTCC`
    /// handler owns the vector instead and never touches `WAKE_EVENTS`.
    /// This variant works correctly under either handler (or, for that
    /// matter, no RTCC-specific bookkeeping at all): it only trusts the
    /// hardware counter itself, so `diag-rtcc-time`'s explicit EM2 phase
    /// can safely reuse the exact same bounded `enter_em2_once` loop as
    /// `diag-em2` while the time driver's ISR is installed.
    pub fn sleep_for_ticks_polled(ticks_from_now: u32) -> Result<u32, PmError> {
        let deadline = arm_wake(ticks_from_now);
        let mut attempts = 0u32;
        loop {
            enter_em2_once()?;
            if super::ticks_reached(now(), deadline) {
                disarm_wake();
                return Ok(deadline);
            }
            attempts += 1;
            if attempts > DEFAULT_MAX_SPURIOUS_WAKES {
                disarm_wake();
                return Err(PmError::WakeTimeout);
            }
        }
    }

    /// True if `SCB.SCR.SLEEPDEEP` is currently set. Used only for
    /// diagnostic verification (e.g. `diag-rtcc-time` asserting it never
    /// leaves EM2 armed by accident) — [`enter_em2_once`] already clears
    /// this unconditionally on every return, whatever woke the core.
    pub fn sleepdeep_is_set() -> bool {
        flag_set(SCB_SCR, SCB_SCR_SLEEPDEEP)
    }
}

#[cfg(target_arch = "arm")]
pub use hw::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ms_to_ticks_and_back_round_trips_at_lfrco_rate() {
        let hz = 32_768u32;
        for ms in [0u32, 1, 30, 500, 1_000, 60_000] {
            let ticks = ms_to_ticks(ms, hz);
            let back = ticks_to_ms(ticks, hz);
            // Integer division loses sub-tick precision; allow +/-1 ms.
            assert!(back.abs_diff(ms) <= 1, "ms={ms} ticks={ticks} back={back}");
        }
    }

    #[test]
    fn ms_to_ticks_does_not_overflow_for_large_durations() {
        // 1 hour at LFRCO rate must not overflow the u64 intermediate or
        // truncate incorrectly when narrowed back to u32.
        let hz = 32_768u32;
        let one_hour_ms = 3_600_000u32;
        let ticks = ms_to_ticks(one_hour_ms, hz);
        assert_eq!(ticks, 117_964_800);
    }

    #[test]
    fn deadline_from_now_wraps_correctly() {
        let now = u32::MAX - 10;
        let deadline = deadline_from_now(now, 20);
        assert_eq!(deadline, 9);
    }

    #[test]
    fn compare_value_accounts_for_rtcc_plus_one_match() {
        assert_eq!(compare_from_deadline(1_000), 999);
        assert_eq!(compare_from_deadline(0), u32::MAX);
    }

    #[test]
    fn ticks_reached_handles_wraparound() {
        let deadline = u32::MAX - 5;
        assert!(!ticks_reached(u32::MAX - 6, deadline));
        assert!(ticks_reached(u32::MAX - 5, deadline));
        assert!(ticks_reached(u32::MAX, deadline));
        // Wrapped past the deadline.
        assert!(ticks_reached(4, deadline));
    }

    #[test]
    fn ticks_reached_is_false_long_before_deadline() {
        assert!(!ticks_reached(0, 1_000));
        assert!(ticks_reached(1_000, 1_000));
        assert!(ticks_reached(1_001, 1_000));
    }

    #[test]
    fn elapsed_ticks_handles_wraparound() {
        assert_eq!(elapsed_ticks(u32::MAX - 1, 3), 5);
        assert_eq!(elapsed_ticks(100, 150), 50);
    }

    #[test]
    fn progressed_within_tolerance_accepts_lfrco_drift() {
        // LFRCO is not crystal-accurate; allow +/-5% and confirm both edges.
        assert!(progressed_within_tolerance(950, 1_000, 5));
        assert!(progressed_within_tolerance(1_050, 1_000, 5));
        assert!(!progressed_within_tolerance(900, 1_000, 5));
        assert!(!progressed_within_tolerance(1_100, 1_000, 5));
    }

    #[test]
    fn progressed_within_tolerance_zero_percent_requires_exact_match() {
        assert!(progressed_within_tolerance(1_000, 1_000, 0));
        assert!(!progressed_within_tolerance(999, 1_000, 0));
    }

    #[test]
    fn classify_wake_detects_isr_progress() {
        assert_eq!(classify_wake(3, 3), WakeCause::Spurious);
        assert_eq!(classify_wake(3, 4), WakeCause::RtccCompare);
        // Counter itself can wrap; any change counts as progress.
        assert_eq!(classify_wake(u32::MAX, 0), WakeCause::RtccCompare);
    }

    #[test]
    fn canary_pattern_is_stable_and_distinct_per_slot() {
        let seed = 0xC0FF_EE00u32;
        let p0 = canary_pattern(seed, 0);
        let p1 = canary_pattern(seed, 1);
        assert_ne!(p0, p1);
        // Recomputing must be deterministic.
        assert_eq!(p0, canary_pattern(seed, 0));
    }

    #[test]
    fn validate_canaries_accepts_matching_pattern() {
        let seed = 0xC0FF_EE00u32;
        let values: [u32; 4] = core::array::from_fn(|i| canary_pattern(seed, i));
        assert_eq!(validate_canaries(seed, &values), Ok(()));
    }

    #[test]
    fn validate_canaries_reports_first_mismatch_with_details() {
        let seed = 0xC0FF_EE00u32;
        let mut values: [u32; 4] = core::array::from_fn(|i| canary_pattern(seed, i));
        values[2] ^= 1;
        let err = validate_canaries(seed, &values).unwrap_err();
        assert_eq!(err.index, 2);
        assert_eq!(err.expected, canary_pattern(seed, 2));
        assert_eq!(err.actual, err.expected ^ 1);
    }

    #[test]
    fn poll_bounded_succeeds_within_budget() {
        let mut calls = 0u32;
        let result = poll_bounded(5, || {
            calls += 1;
            calls == 3
        });
        assert_eq!(result, Ok(()));
        assert_eq!(calls, 3);
    }

    #[test]
    fn poll_bounded_exhausts_budget_and_reports_error() {
        let mut calls = 0u32;
        let result = poll_bounded(4, || {
            calls += 1;
            false
        });
        assert_eq!(result, Err(()));
        assert_eq!(calls, 4);
    }

    #[test]
    fn poll_bounded_zero_budget_never_calls_poll() {
        let mut calls = 0u32;
        let result = poll_bounded(0, || {
            calls += 1;
            true
        });
        assert_eq!(result, Err(()));
        assert_eq!(calls, 0);
    }

    #[test]
    fn pm_error_variants_are_distinguishable_and_debug_formattable() {
        // Exercises `PmError` on host builds too (the hardware module that
        // returns it is compiled only for `target_arch = "arm"`), so this
        // type stays covered even where the real EM2 driver is compiled out.
        let errors = [
            PmError::LfrcoStartTimeout,
            PmError::LfeClockSyncTimeout,
            PmError::RtccStartTimeout,
            PmError::DcdcSyncTimeout,
            PmError::WakeTimeout,
        ];
        for (i, a) in errors.iter().enumerate() {
            for (j, b) in errors.iter().enumerate() {
                assert_eq!(i == j, a == b);
            }
            assert!(!format!("{a:?}").is_empty());
        }
    }

    // ── 64-bit monotonic extension / long-deadline tests ────────────

    #[test]
    fn decode_pending_flags_reads_both_bits_independently() {
        assert_eq!(decode_pending_flags(0), RtccFlags::default());
        assert_eq!(
            decode_pending_flags(RTCC_IF_CC0_BIT),
            RtccFlags {
                cc0: true,
                overflow: false
            }
        );
        assert_eq!(
            decode_pending_flags(RTCC_IF_OF_BIT),
            RtccFlags {
                cc0: false,
                overflow: true
            }
        );
        assert_eq!(
            decode_pending_flags(RTCC_IF_CC0_BIT | RTCC_IF_OF_BIT),
            RtccFlags {
                cc0: true,
                overflow: true
            }
        );
        // Higher (reserved/other-channel) bits must not leak into either.
        assert_eq!(
            decode_pending_flags(0x7FF),
            RtccFlags {
                cc0: true,
                overflow: true
            }
        );
    }

    #[test]
    fn extend_to_64_without_pending_overflow_just_concatenates() {
        assert_eq!(extend_to_64(0, 0, false), 0);
        assert_eq!(extend_to_64(0, 0x1234_5678, false), 0x1234_5678);
        assert_eq!(extend_to_64(1, 0, false), 1u64 << 32);
        assert_eq!(
            extend_to_64(0xABCD, 0x0102_0304, false),
            (0xABCDu64 << 32) | 0x0102_0304
        );
    }

    #[test]
    fn extend_to_64_with_pending_overflow_adds_one_epoch() {
        // A wrap that hardware has already recorded but that our epoch
        // counter has not yet been bumped for must still land in the next
        // epoch, regardless of `low`'s value.
        assert_eq!(extend_to_64(0, 0, true), 1u64 << 32);
        assert_eq!(
            extend_to_64(0, 0xFFFF_FFFF, true),
            (1u64 << 32) | 0xFFFF_FFFF
        );
        assert_eq!(extend_to_64(41, 7, true), (42u64 << 32) | 7);
    }

    #[test]
    fn now64_from_samples_resolves_when_epoch_is_stable() {
        assert_eq!(
            now64_from_samples(3, 3, 100, 101, false),
            Some(extend_to_64(3, 101, false))
        );
        assert_eq!(
            now64_from_samples(3, 3, 100, 101, true),
            Some(extend_to_64(3, 101, true))
        );
    }

    #[test]
    fn now64_from_samples_detects_wrap_after_pending_flag_sample() {
        assert_eq!(
            now64_from_samples(3, 3, u32::MAX - 1, 2, false),
            Some(extend_to_64(3, 2, true))
        );
    }

    #[test]
    fn now64_from_samples_reports_torn_read_when_epoch_moved() {
        // The overflow ISR ran and bumped the epoch while we were sampling
        // `low`/`overflow_pending` — the caller must retake the sample
        // rather than trust either half.
        assert_eq!(now64_from_samples(3, 4, 100, 101, false), None);
        assert_eq!(now64_from_samples(3, 4, 100, 101, true), None);
    }

    #[test]
    fn now64_extension_is_monotonic_across_a_simulated_wrap() {
        // Just before the wrap: epoch N, low near u32::MAX.
        let before = extend_to_64(5, u32::MAX - 2, false);
        // Hardware wraps; ISR has not run yet (pending=true), counter is
        // now small again.
        let during = extend_to_64(5, 1, true);
        // ISR has now run: epoch bumped, flag cleared.
        let after = extend_to_64(6, 5, false);
        assert!(before < during, "before={before} during={during}");
        assert!(during < after, "during={during} after={after}");
    }

    #[test]
    fn ticks_from_now_clamped_reports_none_at_or_past_deadline() {
        assert_eq!(ticks_from_now_clamped(100, 100), None);
        assert_eq!(ticks_from_now_clamped(100, 50), None);
    }

    #[test]
    fn ticks_from_now_clamped_enforces_minimum_compare_margin() {
        // GSDK's own sleeptimer HAL refuses to arm closer than this to
        // avoid missing the compare match across the LFE sync boundary.
        assert_eq!(ticks_from_now_clamped(100, 101), Some(MIN_ARM_TICKS));
        assert_eq!(ticks_from_now_clamped(100, 102), Some(MIN_ARM_TICKS));
        assert_eq!(ticks_from_now_clamped(100, 103), Some(MIN_ARM_TICKS));
        // One past the minimum margin: no clamping needed.
        assert_eq!(ticks_from_now_clamped(100, 104), Some(4));
    }

    #[test]
    fn ticks_from_now_clamped_passes_through_mid_range_deadlines() {
        assert_eq!(ticks_from_now_clamped(0, 32_768), Some(32_768));
        assert_eq!(
            ticks_from_now_clamped(1_000, 1_000 + 3_600 * 32_768),
            Some(3_600 * 32_768)
        );
    }

    #[test]
    fn ticks_from_now_clamped_caps_long_deadlines_for_a_later_rearm() {
        let now = 10u64;
        let far_future = now + (MAX_ARM_TICKS as u64) * 3 + 1;
        assert_eq!(ticks_from_now_clamped(now, far_future), Some(MAX_ARM_TICKS));
    }

    #[test]
    fn ticks_from_now_clamped_never_exceeds_max_arm_ticks_even_near_u64_max() {
        let now = 0u64;
        let deadline = u64::MAX;
        assert_eq!(ticks_from_now_clamped(now, deadline), Some(MAX_ARM_TICKS));
    }

    #[test]
    fn min_and_max_arm_ticks_leave_a_valid_non_overlapping_window() {
        assert!(MIN_ARM_TICKS < MAX_ARM_TICKS);
        // The GSDK-derived safety margin should be small (a handful of
        // ticks), not a large fraction of the representable window.
        assert!(MIN_ARM_TICKS <= 8);
    }
}
