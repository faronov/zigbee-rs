//! EM2 (deep sleep) entry for EFR32 Series 2.
//!
//! Follows GSDK 4.5.0 `em_emu.c` `EMU_EnterEM2`:
//!
//! * Series 2 restores the HF oscillators on demand in hardware, so no CMU
//!   save/restore is needed; the only `EMU_Save`/`EMU_Restore` work on
//!   xG22 is the DPLL, which this HAL does not use (SYSCLK = HFXO).
//! * xG22 (`_SILICON_LABS_GECKO_INTERNAL_SDID_205`) executes the `WFI`
//!   from RAM inside a critical section and, with a debugger attached,
//!   spins a few cycles after wake (`ramWFI`). This module does the same
//!   on both dies: the RAM function is a few bytes and harmless on xG21.
//!
//! Wake sources in EM2 are the LF peripherals (RTCC here) and GPIO ports
//! A/B. The radio, TIMER0 and USARTs are stopped; register contents are
//! retained, but callers must re-validate radio state after wake.
//!
//! The DC-DC converter is left in its reset (bypass) state, so EM2 current
//! is higher than the datasheet's DC-DC figure.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::rtcc::{self, Channel};

const EMU_CTRL: u32 = crate::chip::base::EMU + 0x074;
const EMU_CTRL_EM2DBGEN: u32 = 1 << 0;
const SCB_SCR: u32 = 0xE000_ED10;
const SCB_SCR_SLEEPDEEP: u32 = 1 << 2;
const SET: u32 = 0x1000;
const CLR: u32 = 0x2000;

/// Wakes of the blocking sleep that were neither the deadline nor the
/// caller's interrupt condition (e.g. the Embassy alarm on CC0) before
/// [`sleep_em2`] gives up. Each one costs one ISR run.
pub const MAX_FOREIGN_WAKES: u32 = 10_000;

static EM2_ENTRIES: AtomicU32 = AtomicU32::new(0);

/// Number of EM2 entries so far (wrapping); lab and power diagnostics.
pub fn em2_entries() -> u32 {
    EM2_ENTRIES.load(Ordering::Relaxed)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepOutcome {
    /// The RTCC deadline was reached.
    Deadline,
    /// `interrupted()` became true first.
    Interrupted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerError {
    /// More than [`MAX_FOREIGN_WAKES`] unrelated wakes.
    WakeStorm,
}

/// Keep the debug interface alive in EM2 (`EMU_CTRL.EM2DBGEN`).
///
/// Disable for representative current measurements; enable while
/// debugging. GSDK enables it by default in debug builds.
pub fn set_em2_debug(enable: bool) {
    unsafe { write(EMU_CTRL + if enable { SET } else { CLR }, EMU_CTRL_EM2DBGEN) };
}

/// Enter EM2 once unless `abort()` is true with interrupts masked.
///
/// Returns `false` without sleeping if `abort()` held. Any enabled
/// interrupt (pending in the NVIC) ends the sleep; its handler runs after
/// this function restores PRIMASK.
pub fn enter_em2_unless(abort: impl FnOnce() -> bool) -> bool {
    let primask = disable_interrupts();
    if abort() {
        restore_interrupts(primask);
        return false;
    }
    unsafe {
        write_reg(SCB_SCR, read_reg(SCB_SCR) | SCB_SCR_SLEEPDEEP);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        ram_wfi();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        write_reg(SCB_SCR, read_reg(SCB_SCR) & !SCB_SCR_SLEEPDEEP);
    }
    EM2_ENTRIES.fetch_add(1, Ordering::Relaxed);
    restore_interrupts(primask);
    true
}

/// Sleep in EM2 for `ticks` RTCC ticks or until `interrupted()`.
///
/// Uses RTCC CC1 (CC0 stays with the Embassy time driver). The RTCC NVIC
/// line must be enabled and its handler must clear the flags
/// ([`rtcc::take_flags`]); the `time-driver` feature provides one.
/// CC1 is disarmed on every return path.
pub fn sleep_em2(
    ticks: u32,
    mut interrupted: impl FnMut() -> bool,
) -> Result<SleepOutcome, PowerError> {
    let deadline = rtcc::arm(Channel::Cc1, ticks);
    let mut foreign = 0u32;
    let outcome = loop {
        if interrupted() {
            break Ok(SleepOutcome::Interrupted);
        }
        if rtcc::reached(rtcc::now(), deadline) {
            break Ok(SleepOutcome::Deadline);
        }
        enter_em2_unless(|| interrupted() || rtcc::reached(rtcc::now(), deadline));
        if !rtcc::reached(rtcc::now(), deadline) && !interrupted() {
            foreign += 1;
            if foreign > MAX_FOREIGN_WAKES {
                break Err(PowerError::WakeStorm);
            }
        }
    };
    rtcc::disarm(Channel::Cc1);
    outcome
}

/// `WFI` executed from SRAM (GSDK xG22 `ramWFI`), plus the post-wake
/// settle loop GSDK runs when `DHCSR.C_DEBUGEN` is set.
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[unsafe(link_section = ".data.efr32s2_ram_wfi")]
#[inline(never)]
unsafe extern "C" fn ram_wfi() {
    unsafe {
        core::arch::asm!(
            "dsb",
            "wfi",
            "ldr {t}, [{dhcsr}]",
            "lsls {t}, {t}, #31",
            "beq 2f",
            "movs {t}, #24",
            "1:",
            "subs {t}, {t}, #1",
            "bne 1b",
            "2:",
            dhcsr = in(reg) 0xE000_EDF0u32,
            t = out(reg) _,
            options(nostack),
        );
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
unsafe fn ram_wfi() {}

#[inline(always)]
fn disable_interrupts() -> bool {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        let active = cortex_m::register::primask::read().is_active();
        cortex_m::interrupt::disable();
        active
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    false
}

#[inline(always)]
fn restore_interrupts(was_active: bool) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    if was_active {
        unsafe { cortex_m::interrupt::enable() };
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = was_active;
}

#[inline(always)]
unsafe fn read_reg(address: u32) -> u32 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::read_volatile(address as *const u32)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        let _ = address;
        0
    }
}

#[inline(always)]
unsafe fn write_reg(address: u32, value: u32) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::write_volatile(address as *mut u32, value)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = (address, value);
}

unsafe fn write(address: u32, value: u32) {
    unsafe { write_reg(address, value) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emu_ctrl_matches_gsdk() {
        // GSDK 4.5.0: offsetof(EMU_TypeDef, CTRL) = 0x74, EM2DBGEN bit 0.
        assert_eq!(EMU_CTRL, 0x4000_4074);
        assert_eq!(EMU_CTRL_EM2DBGEN, 1);
    }

    #[test]
    fn abort_skips_sleep() {
        assert!(!enter_em2_unless(|| true));
        assert!(enter_em2_unless(|| false));
    }
}
