//! Reset cause (`EMU_RSTCAUSE`) and software reset.
//!
//! `EMU_RSTCAUSE` (offset 0x94) and `EMU_CMD.RSTCAUSECLR` (offset 0x70,
//! bit 17) are at the same place on xG21, xG22 and xG24; the bits used here
//! (POR, PIN, EM4, WDOG0, LOCKUP, SYSREQ, the brown-out group) have the same
//! positions on the three dies (GSDK 4.5.0 `efr32mg2{1,2,4}_emu.h`).

const EMU_CMD: u32 = crate::chip::base::EMU + 0x070;
const EMU_RSTCAUSE: u32 = crate::chip::base::EMU + 0x094;
const EMU_CMD_RSTCAUSECLR: u32 = 1 << 17;

const POR: u32 = 1 << 0;
const PIN: u32 = 1 << 1;
const EM4: u32 = 1 << 2;
const WDOG0: u32 = 1 << 3;
const WDOG1: u32 = 1 << 4;
const LOCKUP: u32 = 1 << 5;
const SYSREQ: u32 = 1 << 6;
/// DVDD, DVDDLE, DEC, AVDD and IOVDD0 brown-out detectors.
const BROWNOUT: u32 = 0x1F << 7;

/// Why the chip last reset (highest-priority cause wins).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetCause {
    PowerOn,
    BrownOut,
    Pin,
    Watchdog,
    Lockup,
    /// `SYSRESETREQ` (software reset, including a return from the Gecko
    /// bootloader).
    Software,
    Em4Wakeup,
    /// No recognised bit (e.g. a cleared register).
    Unknown,
}

/// Decode a raw `EMU_RSTCAUSE` value. Power-on and brown-out leave other
/// bits set too, so they are checked first.
pub const fn decode(raw: u32) -> ResetCause {
    if raw & POR != 0 {
        ResetCause::PowerOn
    } else if raw & BROWNOUT != 0 {
        ResetCause::BrownOut
    } else if raw & (WDOG0 | WDOG1) != 0 {
        ResetCause::Watchdog
    } else if raw & LOCKUP != 0 {
        ResetCause::Lockup
    } else if raw & SYSREQ != 0 {
        ResetCause::Software
    } else if raw & EM4 != 0 {
        ResetCause::Em4Wakeup
    } else if raw & PIN != 0 {
        ResetCause::Pin
    } else {
        ResetCause::Unknown
    }
}

/// Read and clear `EMU_RSTCAUSE` (call once, early at boot).
pub fn take_reset_cause() -> (ResetCause, u32) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        let raw = unsafe { core::ptr::read_volatile(EMU_RSTCAUSE as *const u32) };
        unsafe { core::ptr::write_volatile(EMU_CMD as *mut u32, EMU_CMD_RSTCAUSECLR) };
        (decode(raw), raw)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        let _ = (EMU_CMD, EMU_RSTCAUSE, EMU_CMD_RSTCAUSECLR);
        (ResetCause::Unknown, 0)
    }
}

/// Software reset (`SYSRESETREQ`).
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn system_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_priorities() {
        assert_eq!(decode(POR | PIN), ResetCause::PowerOn);
        assert_eq!(decode(1 << 7), ResetCause::BrownOut);
        assert_eq!(decode(WDOG0 | PIN), ResetCause::Watchdog);
        assert_eq!(decode(SYSREQ), ResetCause::Software);
        assert_eq!(decode(PIN), ResetCause::Pin);
        assert_eq!(decode(0), ResetCause::Unknown);
        assert_eq!((EMU_CMD, EMU_RSTCAUSE), (0x4000_4070, 0x4000_4094));
    }
}
