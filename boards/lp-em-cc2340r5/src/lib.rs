//! Pure-Rust board support for LP-EM-CC2340R5 with a CC2340R52.
//!
//! This crate owns only the LaunchPad's physical wiring (LEDs, buttons, the
//! DC/DC inductor). The CC2340R52 chip services — power, standby, time,
//! flash, identity, BATMON, reset and the peripheral drivers — live in
//! [`cc2340r5_hal`] and are re-exported here under their historical paths.
//! Application lifecycle adapters belong to a product or firmware
//! composition crate. It does not link TI Drivers, RCL, FreeRTOS, or ZBOSS.

#![no_std]

pub use cc2340r5_hal::{MCU_CLOCK_HZ, TARGET_PART};
pub use cc2340r5_hal::{batmon, flash, identity, power, standby, timebase};
#[cfg(target_os = "none")]
pub use cc2340r5_hal::{interrupts, reset};

pub mod pins;

pub const BOARD_NAME: &str = "LP-EM-CC2340R5";

/// The LaunchPad populates the DC/DC inductor.
pub const VDDR_REGULATOR: cc2340r5_hal::VddrRegulator = cc2340r5_hal::VddrRegulator::DcDc;

/// All exclusively owned board resources.
#[cfg(target_os = "none")]
pub struct Resources {
    /// Active-high LED1 on DIO7.
    pub led1: pins::Led1,
    /// Active-high LED2 on DIO6.
    pub led2: pins::Led2,
    /// Active-low BTN1 on DIO13.
    pub button1: pins::Button1,
    /// Active-low BTN2 on DIO14.
    pub button2: pins::Button2,
    pub flash: flash::InternalFlash,
    /// On-die VDDS and temperature monitor, measuring continuously.
    pub batmon: batmon::Batmon,
    pub reset: reset::SystemReset,
}

/// Claim and initialize the board exactly once.
///
/// [`cc2340r5_hal::take_core`] selects the DC/DC regulator,
/// flash-LDO-off-in-idle and the LFOSC LF clock, sets up the RTC standby
/// wake source and the tickless Embassy SYSTIM driver; the LaunchPad LEDs
/// and buttons are then claimed and configured. Button press interrupts are
/// configured (edge detection) but left masked until the application calls
/// `Button::enable_press_interrupt`.
#[cfg(target_os = "none")]
pub fn take() -> Option<Resources> {
    let cc2340r5_hal::Core {
        mut pins,
        flash,
        batmon,
        reset,
        ..
    } = cc2340r5_hal::take_core(VDDR_REGULATOR)?;

    Some(Resources {
        led1: pins.led(false).ok()?,
        led2: pins.led(false).ok()?,
        button1: pins.button().ok()?,
        button2: pins.button().ok()?,
        flash,
        batmon,
        reset,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn target_clock_is_the_verified_cc2340r52_rate() {
        assert_eq!(MCU_CLOCK_HZ, 48_000_000);
        assert_eq!(TARGET_PART, "CC2340R52");
    }

    #[test]
    fn manifest_excludes_the_shared_sensor_application() {
        assert!(!include_str!("../Cargo.toml").contains("sensor-sed-app"));
    }
}
