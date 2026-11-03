//! TLSR8258 adapters for the platform-independent sleepy-sensor lifecycle.
//!
//! ```text
//! application  ← apps/sensor-sed
//! product      ← identity, clusters, indicator, flash layout, persistence
//! board        ← physical pin wiring
//! chip adapter ← this crate (sleep transactions, LOW32K hand-off, supervisor)
//! chip HAL     ← tlsr8258-hal, tlsr8258-rt
//! ```
//!
//! Composition roots build [`sensor_sed_app::SensorApp`] from [`TelinkWake`],
//! [`TelinkSupervisor`] and [`TelinkNoDiagnostics`]. The reset-on-wake image
//! additionally enables the `retention` feature and uses [`retention`].

#![no_std]

#[cfg(all(target_arch = "tc32", feature = "retention"))]
pub mod retention;
#[cfg(target_arch = "tc32")]
mod supervisor;
#[cfg(target_arch = "tc32")]
mod wake;

#[cfg(target_arch = "tc32")]
pub use supervisor::{TelinkNoDiagnostics, TelinkSupervisor};
#[cfg(target_arch = "tc32")]
pub use wake::{TelinkWake, TelinkWakeError};

/// Cold-boot clock bring-up for sleepy images: system clocks, the
/// independent 16 MHz system timer and the calibrated RC32K wake source used
/// by every sleep transaction. Returns `false` if any step cannot be
/// verified; the caller must then fail closed before constructing the MAC.
#[cfg(target_arch = "tc32")]
pub fn init_sleep_clocks() -> bool {
    tlsr8258_hal::clocks::init().is_ok()
        && tlsr8258_hal::pm::system_timer_init().is_ok()
        && tlsr8258_hal::pm::rc_32k_init_and_cal().is_ok()
}
