//! Truthful BL702 power-state entry.
//!
//! Plain CPU idle is available. PDS and HBN are deliberately rejected until a
//! platform integration restores clocks, radio calibration/state, monotonic
//! time, flash state, and wake errata on hardware.

use crate::peripherals::Power;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerMode {
    Idle,
    Pds,
    Hbn,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerError {
    /// The required platform wake-and-restore path has not been proven.
    RestorePathUnproven,
    UnsupportedTarget,
}

pub struct PowerManager {
    _token: Power,
}

impl PowerManager {
    pub const fn new(token: Power) -> Self {
        Self { _token: token }
    }

    pub fn enter(&mut self, mode: PowerMode) -> Result<(), PowerError> {
        enter_mode(mode)
    }
}

fn enter_mode(mode: PowerMode) -> Result<(), PowerError> {
    match mode {
        PowerMode::Idle => idle(),
        PowerMode::Pds | PowerMode::Hbn => Err(PowerError::RestorePathUnproven),
    }
}

fn idle() -> Result<(), PowerError> {
    #[cfg(target_arch = "riscv32")]
    {
        // SAFETY: WFI only waits for an enabled interrupt and does not alter
        // peripheral or memory ownership.
        unsafe {
            core::arch::asm!("wfi");
        }
        Ok(())
    }

    #[cfg(not(target_arch = "riscv32"))]
    {
        Err(PowerError::UnsupportedTarget)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deep_states_are_not_claimed_supported() {
        assert_eq!(
            enter_mode(PowerMode::Pds),
            Err(PowerError::RestorePathUnproven)
        );
        assert_eq!(
            enter_mode(PowerMode::Hbn),
            Err(PowerError::RestorePathUnproven)
        );
    }
}
