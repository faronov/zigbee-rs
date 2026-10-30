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

/// BL702 mask-ROM function table (`ROM_APITABLE`).
#[cfg(target_arch = "riscv32")]
const ROM_API_TABLE: u32 = 0x2101_8800;
/// `ROM_API_INDEX_GLB_SW_POR_Reset` in the BL702 SDK `bl702_romdriver.h`.
#[cfg(any(target_arch = "riscv32", test))]
const ROM_GLB_SW_POR_RESET: usize = 36;

/// Software power-on reset of the whole chip.
///
/// Uses the mask-ROM `GLB_SW_POR_Reset` routine (the SDK's `hal_por_reset`):
/// it switches the root clock back to RC32M and asserts the system, CPU and
/// power-on reset bits in `GLB_SWRST_CFG2`. The chip then re-runs the boot
/// ROM, which checks the flash boot header and starts the image at its
/// `img_start` (the bootloader on products that have one). Callers must have
/// persisted every piece of state they need first.
pub fn software_reset() -> ! {
    #[cfg(target_arch = "riscv32")]
    {
        type PorReset = unsafe extern "C" fn() -> i32;
        // SAFETY: Mask interrupts so no handler runs while the ROM switches
        // the clock tree. The table index matches the BL702 SDK enum and the
        // routine does not return once the reset is asserted.
        unsafe {
            core::arch::asm!("csrci mstatus, 8");
            let entry = core::ptr::read_volatile(
                (ROM_API_TABLE + (ROM_GLB_SW_POR_RESET as u32) * 4) as *const u32,
            );
            let reset: PorReset = core::mem::transmute(entry as usize);
            reset();
        }
    }
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn por_reset_rom_index_matches_bl702_sdk_enum() {
        assert_eq!(ROM_GLB_SW_POR_RESET, 36);
    }

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
