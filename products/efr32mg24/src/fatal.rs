//! Fatal-fault policy: report, record the cause, controlled reset.
//!
//! Every reset the image takes because something went wrong goes through
//! `reset`, which records the [`Fatal`] cause in the Gecko
//! `BootloaderResetCause_t` word at `SRAM_BASE` and resets the chip. The
//! next boot takes the record (`take_previous`): the NCP reports
//! `EVT_NCP_READY.reset_reason` 6 (fatal, [`zrhp_reset_reason`]) instead of
//! 2 (software, which stays the answer to a host `SYS_RESET`), and both
//! images log the cause.
//!
//! The causes:
//!
//! * a poisoned Secure Engine mailbox (`efr32s2_hal::se`): a TRNG command
//!   that was submitted and never answered, or refused, leaves every later
//!   SE request failing until reset. On xG24 the TRNG is the NCP's only
//!   entropy source, so the NCP could never again form a network or
//!   generate a key, while it would otherwise keep running and answering
//!   the host. Both compositions run [`check_platform`] after every
//!   main-loop step and log the fault first (NCP: the ZRHP LOG ring,
//!   serviced for a bounded window; Router: the log UART);
//! * a panic and a HardFault (the compositions' handlers; nothing is
//!   logged, the handlers only record and reset);
//! * an unrecoverable router-app error (`Supervisor::reset` of the
//!   Router, after the app's own `Fatal` diagnostic).
//!
//! `link/memory.x` keeps that word out of the image's RAM so start-up does
//! not overwrite it; the resident bootloader starts the application for
//! every reason except its bootload ones, so it passes the record on.
//! Build-only: no MG24 has run this path (it needs an SE that stops
//! answering, which has not been observed).

use efr32s2_hal::gecko_bootloader::reason_enters_bootloader;
use efr32s2_hal::reset::ResetCause;

/// A fault after which the image resets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fatal {
    /// The Secure Engine mailbox is poisoned: the TRNG (and any later SE
    /// service) fails until reset.
    SecureEnginePoisoned,
    /// The panic handler ran.
    Panic,
    /// The HardFault handler ran.
    HardFault,
    /// The Router's supervisor was asked to reset after an unrecoverable
    /// router-app error.
    Application,
}

/// EmberZNet `RESET_FATAL` group (GSDK `reset-def.h`, base 0x09); zigbee-rs
/// causes use extension codes from 0x80, above the GSDK ones.
const RESET_FATAL_BASE: u16 = 0x0900;
const SECURE_ENGINE_POISONED: u16 = RESET_FATAL_BASE | 0x80;
const PANIC: u16 = RESET_FATAL_BASE | 0x81;
const HARD_FAULT: u16 = RESET_FATAL_BASE | 0x82;
const APPLICATION: u16 = RESET_FATAL_BASE | 0x83;

impl Fatal {
    /// Every cause, for the checks below and the tests.
    pub const ALL: [Self; 4] = [
        Self::SecureEnginePoisoned,
        Self::Panic,
        Self::HardFault,
        Self::Application,
    ];

    /// The `BootloaderResetCause_t.reason` recorded for this fault.
    pub const fn reset_reason(self) -> u16 {
        match self {
            Self::SecureEnginePoisoned => SECURE_ENGINE_POISONED,
            Self::Panic => PANIC,
            Self::HardFault => HARD_FAULT,
            Self::Application => APPLICATION,
        }
    }

    /// The fault recorded with `reason`, if it is one of ours.
    pub const fn from_reset_reason(reason: u16) -> Option<Self> {
        match reason {
            SECURE_ENGINE_POISONED => Some(Self::SecureEnginePoisoned),
            PANIC => Some(Self::Panic),
            HARD_FAULT => Some(Self::HardFault),
            APPLICATION => Some(Self::Application),
            _ => None,
        }
    }
}

// Every recorded fault must restart the application, not the bootloader,
// and must decode back to itself.
const _: () = {
    let mut index = 0;
    while index < Fatal::ALL.len() {
        let fatal = Fatal::ALL[index];
        let reason = fatal.reset_reason();
        assert!(!reason_enters_bootloader(reason));
        assert!(matches!(
            Fatal::from_reset_reason(reason),
            Some(decoded) if decoded as u8 == fatal as u8
        ));
        index += 1;
    }
};

/// The policy: which platform state is fatal.
pub const fn check(secure_engine_poisoned: bool) -> Result<(), Fatal> {
    if secure_engine_poisoned {
        Err(Fatal::SecureEnginePoisoned)
    } else {
        Ok(())
    }
}

/// The fault the previous boot recorded, from this boot's reset cause and
/// the reason word. Only a software reset can carry one: SRAM is undefined
/// after power-on and a pin or watchdog reset did not go through
/// `reset`.
pub fn previous(cause: ResetCause, reason: Option<u16>) -> Option<Fatal> {
    match cause {
        ResetCause::Software => reason.and_then(Fatal::from_reset_reason),
        _ => None,
    }
}

/// `EVT_NCP_READY.reset_reason` (ZRHP spec 7.1): 6 (fatal) after a
/// recorded fault or a CPU lockup, otherwise from `EMU_RSTCAUSE`. The
/// RESET pin is driven by the host (Dongle-M ESP32 GPIO15, dev-board
/// ESP32-S3 or J-Link): a host request. The NCP never enters EM4, so an
/// EM4 wake-up is as unknown (0) as an empty cause.
pub fn zrhp_reset_reason(cause: ResetCause, previous: Option<Fatal>) -> u8 {
    if previous.is_some() {
        return 6;
    }
    match cause {
        ResetCause::PowerOn => 1,
        ResetCause::Software => 2,
        ResetCause::Watchdog => 3,
        ResetCause::BrownOut => 4,
        ResetCause::Lockup => 6,
        ResetCause::Pin => 7,
        ResetCause::Em4Wakeup | ResetCause::Unknown => 0,
    }
}

/// [`check`] on the live platform state.
pub fn check_platform() -> Result<(), Fatal> {
    check(efr32s2_hal::se::is_poisoned())
}

/// Take the record of the previous boot and clear it. Call once, early,
/// after `efr32s2_hal::reset::take_reset_cause`.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn take_previous(cause: ResetCause) -> Option<Fatal> {
    // SAFETY: link/memory.x starts RAM at 0x2000_0004, so the first SRAM
    // word is the reset-cause record only (asserted there and by
    // examples/efr32mg24-ncp/tools/verify-layout.py).
    let reason = unsafe { efr32s2_hal::gecko_bootloader::take_reset_reason() };
    previous(cause, reason)
}

/// Record `fatal` and reset the chip; the caller reports it first if it
/// can. Usable from the panic and HardFault handlers: it masks interrupts,
/// writes one SRAM word and requests `SYSRESETREQ`, nothing else. Kept out
/// of line: CI requires this symbol in every image with an SE client.
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[inline(never)]
pub fn reset(fatal: Fatal) -> ! {
    efr32s2_hal::gecko_bootloader::reset_with_reason(fatal.reset_reason())
}

#[cfg(test)]
mod tests {
    use super::*;
    use efr32s2_hal::gecko_bootloader::{reset_cause_reason, reset_cause_word};

    #[test]
    fn poisoned_secure_engine_is_fatal() {
        assert_eq!(check(false), Ok(()));
        assert_eq!(check(true), Err(Fatal::SecureEnginePoisoned));
        // Host build: no mailbox, never poisoned.
        assert_eq!(check_platform(), Ok(()));
    }

    #[test]
    fn recorded_faults_survive_the_reason_word_round_trip() {
        let codes = [0x0980, 0x0981, 0x0982, 0x0983];
        for (fatal, code) in Fatal::ALL.into_iter().zip(codes) {
            assert_eq!(fatal.reset_reason(), code);
            assert!(!reason_enters_bootloader(code));
            let word = reset_cause_word(fatal.reset_reason());
            assert_eq!(
                previous(ResetCause::Software, reset_cause_reason(word)),
                Some(fatal)
            );
        }
        // Neighbouring codes are not ours.
        assert_eq!(Fatal::from_reset_reason(0x097F), None);
        assert_eq!(Fatal::from_reset_reason(0x0984), None);
    }

    #[test]
    fn panic_and_hard_fault_are_reported_as_fatal_not_software() {
        for fatal in [Fatal::Panic, Fatal::HardFault] {
            let word = reset_cause_word(fatal.reset_reason());
            let recorded = previous(ResetCause::Software, reset_cause_reason(word));
            assert_eq!(recorded, Some(fatal));
            assert_eq!(zrhp_reset_reason(ResetCause::Software, recorded), 6);
        }
        // A host SYS_RESET (plain SYSRESETREQ, invalidated word) stays 2.
        let invalid = u32::from(efr32s2_hal::gecko_bootloader::RESET_SIGNATURE_INVALID) << 16;
        let recorded = previous(ResetCause::Software, reset_cause_reason(invalid));
        assert_eq!(recorded, None);
        assert_eq!(zrhp_reset_reason(ResetCause::Software, recorded), 2);
    }

    #[test]
    fn only_a_software_reset_with_our_reason_is_a_previous_fault() {
        let ours = Some(Fatal::SecureEnginePoisoned.reset_reason());
        for cause in [
            ResetCause::PowerOn,
            ResetCause::BrownOut,
            ResetCause::Pin,
            ResetCause::Watchdog,
            ResetCause::Lockup,
            ResetCause::Em4Wakeup,
            ResetCause::Unknown,
        ] {
            assert_eq!(previous(cause, ours), None);
        }
        // Bootloader reasons (GO, BADIMAGE) and a missing record are not.
        assert_eq!(previous(ResetCause::Software, Some(0x0201)), None);
        assert_eq!(previous(ResetCause::Software, Some(0x0203)), None);
        assert_eq!(previous(ResetCause::Software, None), None);
    }

    #[test]
    fn zrhp_reset_reason_reports_a_recorded_fault_as_fatal() {
        for fatal in Fatal::ALL {
            assert_eq!(zrhp_reset_reason(ResetCause::Software, Some(fatal)), 6);
        }
        assert_eq!(zrhp_reset_reason(ResetCause::PowerOn, None), 1);
        assert_eq!(zrhp_reset_reason(ResetCause::Software, None), 2);
        assert_eq!(zrhp_reset_reason(ResetCause::Watchdog, None), 3);
        assert_eq!(zrhp_reset_reason(ResetCause::BrownOut, None), 4);
        // A lockup is a fault, never 5 (firmware update).
        assert_eq!(zrhp_reset_reason(ResetCause::Lockup, None), 6);
        assert_eq!(zrhp_reset_reason(ResetCause::Pin, None), 7);
        assert_eq!(zrhp_reset_reason(ResetCause::Em4Wakeup, None), 0);
        assert_eq!(zrhp_reset_reason(ResetCause::Unknown, None), 0);
    }
}
