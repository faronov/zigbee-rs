//! Pure-Rust board support for LP-EM-CC2340R5 with a CC2340R52.
//!
//! This crate owns only physical board resources and the verified CC2340R52
//! timer/flash/identity services. Application lifecycle adapters belong to a
//! product or firmware composition crate. It does not link TI Drivers, RCL,
//! FreeRTOS, or ZBOSS.

#![no_std]

pub mod flash;
pub mod identity;
pub mod pins;

#[cfg(target_os = "none")]
pub mod reset;
#[cfg(target_os = "none")]
mod time;

#[cfg(target_os = "none")]
use portable_atomic::{AtomicBool, Ordering};

pub const BOARD_NAME: &str = "LP-EM-CC2340R5";
pub const TARGET_PART: &str = "CC2340R52";

/// CC2340R52 MCU clock selected by the ROM startup code.
pub const MCU_CLOCK_HZ: u32 = 48_000_000;

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
    pub reset: reset::SystemReset,
}

#[cfg(target_os = "none")]
static TAKEN: AtomicBool = AtomicBool::new(false);

/// Claim and initialize the board exactly once.
///
/// IOC muxing, direction, pulls, and the Embassy SysTick driver are
/// initialized before the physical resources are returned.
#[cfg(target_os = "none")]
pub fn take() -> Option<Resources> {
    if TAKEN
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return None;
    }

    time::init();

    Some(Resources {
        led1: pins::Led::configure(false),
        led2: pins::Led::configure(false),
        button1: pins::Button::configure(),
        button2: pins::Button::configure(),
        flash: flash::InternalFlash::new(),
        reset: reset::SystemReset::new(),
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
