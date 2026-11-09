//! RAM-resident ESP32-H2 PMU light and deep sleep, ported from ESP-IDF v5.5.1
//! (`sleep_modes.c`, `port/esp32h2/pmu_sleep.c`, `pmu_param.c`,
//! `lowpower/port/esp32h2/sleep_cpu.c`, `soc/esp32h2/system_retention_periph.c`).
//!
//! Two light-sleep modes exist:
//!
//! * [`SleepMode::Retained`] (default, hardware-proven through the vendor
//!   esp-hal patch it replaces): every digital domain stays powered at the
//!   0.6 V retention voltage; XTAL and RC_FAST may stop.
//! * [`SleepMode::TopPowerDown`] (feature `top-pd`, experimental): the TOP,
//!   CPU and modem domains are powered off. Before sleep the CPU copies the
//!   registers ESP-IDF's REGDMA/CPU retention keeps into RAM buffers
//!   ([`retention`]); on wake the ROM jumps to this crate's restore stub,
//!   which copies them back before any flash access.
//!
//! The pure modules (admission, PMU words, analog settings, timing and the
//! retention tables) are host-tested. The register backend exists only for
//! `riscv32` targets.
#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(target_arch = "riscv32"), allow(dead_code))]

mod analog;
mod clock;
mod error;
mod mode;
mod power;
pub mod retention;
mod timing;

#[cfg(target_arch = "riscv32")]
mod hw;

pub use analog::DbiasFuses;
pub use error::{CalibrationFailure, SleepError};
pub use mode::{Ext1Wake, Request, SleepMode, TOP_PD_SUPPORTED};
pub use power::PowerDown;

#[cfg(target_arch = "riscv32")]
pub use hw::{deep_sleep, light_sleep};

/// Chip facts the sleep path needs, read once by the owner (eFuse access is
/// flash-resident and must not run inside the sleep sequence).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChipInfo {
    /// PMU voltage eFuses and block version.
    pub fuses: DbiasFuses,
    /// Chip revision as `major * 100 + minor` (ECO2+ divides RC_FAST
    /// calibration by 32).
    pub revision: u16,
}

/// Outcome of an admitted light sleep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Wake {
    /// The PMU rejected the request (a wake source was already pending);
    /// no domain was powered down.
    pub rejected: bool,
    /// `PMU_SLP_WAKEUP_STATUS0` (wake cause bits).
    pub wake_cause: u32,
    /// `PMU_SLP_WAKEUP_STATUS1` (reject cause bits).
    pub reject_cause: u32,
    /// TOP/CPU/modem were powered off and this crate restored their
    /// registers. The caller must re-apply any state outside the retained
    /// set (see `esp32h2-sleep/README.md`) before using those peripherals.
    pub top_powered_down: bool,
}
