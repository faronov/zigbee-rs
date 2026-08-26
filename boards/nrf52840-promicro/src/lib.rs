//! Physical nice!nano-compatible nRF52840 ProMicro wiring.
//!
//! The installed Adafruit nRF52 UF2 bootloader board definition specifies one
//! active-high LED on P0.15 and no user button.

#![no_std]

use embassy_nrf::{gpio, peripherals};

pub const STATUS_LED_ACTIVE_LOW: bool = false;
pub const HAS_USER_BUTTON: bool = false;

/// P0.15 status LED, initially off.
pub fn status_led(pin: peripherals::P0_15) -> gpio::Output<'static> {
    gpio::Output::new(pin, gpio::Level::Low, gpio::OutputDrive::Standard)
}

/// Disable the installed S140 SoftDevice before Embassy takes RTC1/RADIO.
///
/// Returns the Nordic status code from `sd_softdevice_disable` (SVC 17).
///
/// # Safety
///
/// The caller must execute from an S140 deployment with the SoftDevice
/// enabled, before initializing any peripheral used by the SoftDevice.
pub unsafe fn disable_softdevice() -> u32 {
    let result: u32;
    unsafe {
        core::arch::asm!(
            "svc 17",
            lateout("r0") result,
            lateout("r1") _,
            lateout("r2") _,
            lateout("r3") _,
            lateout("r12") _,
            options(preserves_flags)
        );
    }
    result
}
