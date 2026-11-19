//! Reset cause and system reset (`inc/hw_aon_pmctl.h` `RESETCTL`,
//! driverlib `SysCtrlResetSourceGet` / `SysCtrlSystemReset`).

use crate::mmio::{self, AON_PMCTL_BASE};

const RESETCTL: u32 = AON_PMCTL_BASE + 0x28;
const RESETCTL_SYSRESET: u32 = 1 << 31;

/// Root cause of the last system reset (`RESETCTL.RESET_SRC`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetSource {
    PowerOn,
    /// The RESET_N pin (a USB dongle's bridge DTR/RTS circuit).
    Pin,
    /// Brown-out on VDDS.
    VddsLoss,
    /// Brown-out on VDDR.
    VddrLoss,
    /// SCLK_LF/MF/HF loss detected.
    ClockLoss,
    /// `SYSRESET` (software) or the power-management timeout.
    SystemReset,
    /// PRCM warm reset (CPU reset request, watchdog, lockup).
    WarmReset,
}

impl ResetSource {
    /// Decode the 3-bit `RESET_SRC` field.
    pub const fn from_field(field: u32) -> Self {
        match field & 0x7 {
            0 => Self::PowerOn,
            1 => Self::Pin,
            2 => Self::VddsLoss,
            4 => Self::VddrLoss,
            5 => Self::ClockLoss,
            6 => Self::SystemReset,
            7 => Self::WarmReset,
            // 3 is reserved; report it as the closest brown-out cause.
            _ => Self::VddsLoss,
        }
    }

    /// Read the cause of the current boot.
    pub fn read() -> Self {
        Self::from_field(mmio::read(RESETCTL) >> 1)
    }
}

/// Full system reset (MCU, AUX and RF core), as `SysCtrlSystemReset`.
pub fn system_reset() -> ! {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    cortex_m::interrupt::disable();
    mmio::modify(RESETCTL, 0, RESETCTL_SYSRESET);
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_source_field_matches_the_header() {
        // RESET_SRC values from hw_aon_pmctl.h, already shifted down by 1.
        assert_eq!(ResetSource::from_field(0x0 >> 1), ResetSource::PowerOn);
        assert_eq!(ResetSource::from_field(0x2 >> 1), ResetSource::Pin);
        assert_eq!(ResetSource::from_field(0x4 >> 1), ResetSource::VddsLoss);
        assert_eq!(ResetSource::from_field(0x8 >> 1), ResetSource::VddrLoss);
        assert_eq!(ResetSource::from_field(0xA >> 1), ResetSource::ClockLoss);
        assert_eq!(ResetSource::from_field(0xC >> 1), ResetSource::SystemReset);
        assert_eq!(ResetSource::from_field(0xE >> 1), ResetSource::WarmReset);
    }
}
