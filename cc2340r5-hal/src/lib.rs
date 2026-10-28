//! Pure-Rust chip services and peripheral drivers for the CC2340R52.
//!
//! This crate owns everything that is a property of the chip rather than of
//! a board: power/clock configuration, standby, the tickless Embassy time
//! driver, interrupt dispatch, BATMON, internal flash, factory identity,
//! reset, GPIO/IOC pin ownership, and the I2C, ADC and LGPT PWM drivers.
//! Board crates map physical wiring onto it through `take_core`. It does
//! not link TI Drivers, RCL, FreeRTOS, or ZBOSS; the register sequences
//! mirror the TI SimpleLink Low Power F3 SDK sources cited in each module.
//!
//! The default `runtime` feature provides the application runtime: the
//! Embassy time driver, interrupt dispatch, standby and `take_core`.
//! Without it (bootloaders) only `take_boot` is available, which touches no
//! clock, interrupt or power configuration and leaves the chip as the ROM
//! started it.

#![no_std]
// Host builds exist only for the unit tests; register paths are target-only.
// Bootloader builds (no `runtime`) leave the application drivers unused.
#![cfg_attr(
    any(not(target_os = "none"), not(feature = "runtime")),
    allow(dead_code)
)]

pub mod adc;
pub mod batmon;
pub mod clkctl;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod identity;
#[cfg(all(target_os = "none", feature = "runtime"))]
pub mod interrupts;
pub mod lgpt;
pub mod power;
pub mod standby;
pub mod timebase;

#[cfg(target_os = "none")]
pub mod reset;
#[cfg(all(target_os = "none", feature = "runtime"))]
mod time;

#[cfg(target_os = "none")]
use portable_atomic::{AtomicBool, Ordering};

pub use power::VddrRegulator;

pub const TARGET_PART: &str = "CC2340R52";

/// CC2340R52 MCU clock selected by the ROM startup code.
pub const MCU_CLOCK_HZ: u32 = 48_000_000;

/// Chip-level resources, each owned exactly once.
#[cfg(all(target_os = "none", feature = "runtime"))]
pub struct Core {
    /// DIO pad ownership; boards build LEDs, buttons and peripheral pins
    /// from it.
    pub pins: gpio::Pins,
    pub flash: flash::InternalFlash,
    /// On-die VDDS and temperature monitor, measuring continuously.
    pub batmon: batmon::Batmon,
    pub reset: reset::SystemReset,
    /// `I2C0` controller (clock gated until used).
    pub i2c0: i2c::I2c0,
    /// `ADC0` (clock gated until used).
    pub adc0: adc::Adc0,
    /// `LGPT1` timer (clock gated until used).
    pub lgpt1: lgpt::Lgpt1,
}

#[cfg(target_os = "none")]
static TAKEN: AtomicBool = AtomicBool::new(false);

/// Claim and initialize the chip exactly once.
///
/// Mutually exclusive with [`take_boot`]: the chip can be claimed once.
///
/// The VDDR regulator (`regulator`, a board property), flash-LDO-off-in-idle
/// and the LFOSC LF clock are selected first ([`power`]), the RTC is set up
/// as standby wake source ([`standby`]) and the tickless Embassy SYSTIM
/// driver is started. Peripheral clocks stay gated; no pad is configured
/// until the board claims it.
#[cfg(all(target_os = "none", feature = "runtime"))]
pub fn take_core(regulator: VddrRegulator) -> Option<Core> {
    if TAKEN
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return None;
    }

    power::init(regulator);
    standby::init();
    time::init();

    Some(Core {
        pins: gpio::Pins::new(),
        flash: flash::InternalFlash::new(),
        batmon: batmon::Batmon::init(),
        reset: reset::SystemReset::new(),
        i2c0: i2c::I2c0::new(),
        adc0: adc::Adc0::new(),
        lgpt1: lgpt::Lgpt1::new(),
    })
}

/// The minimal resources a bootloader needs.
#[cfg(target_os = "none")]
pub struct BootCore {
    pub pins: gpio::Pins,
    pub flash: flash::InternalFlash,
    pub reset: reset::SystemReset,
}

/// Claim the chip for a bootloader exactly once.
///
/// Unlike `take_core` this changes no power, clock, standby or interrupt
/// configuration, so the application starts from the ROM boot state and
/// performs its own initialization. Flash operations mask interrupts and
/// rely on the ROM HAPI only.
#[cfg(target_os = "none")]
pub fn take_boot() -> Option<BootCore> {
    if TAKEN
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return None;
    }
    Some(BootCore {
        pins: gpio::Pins::new(),
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
        assert_eq!(i2c::I2C_CLOCK_HZ, MCU_CLOCK_HZ);
        assert_eq!(lgpt::LGPT_CLOCK_HZ, MCU_CLOCK_HZ);
    }

    #[test]
    fn manifest_has_no_zigbee_or_application_dependency() {
        let manifest = include_str!("../Cargo.toml");
        let dependencies = &manifest[manifest.find("[dependencies]").unwrap()..];
        assert!(!dependencies.contains("zigbee"));
        assert!(!dependencies.contains("sensor-sed-app"));
    }
}
