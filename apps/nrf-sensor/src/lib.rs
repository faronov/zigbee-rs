//! Nordic adapters for the platform-independent sleepy-sensor lifecycle.
//!
//! This is the *application* layer of the stack documented in
//! `docs/book/src/getting-started/architecture.md`:
//!
//! ```text
//! application  ← apps/sensor-sed
//! product      ← identity, flash layout, persistence, battery chemistry
//! board        ← physical pin wiring
//! chip adapter ← this crate (Embassy GPIO/time/reset, SAADC, radio sleep)
//! chip HAL     ← embassy-nrf
//! ```
//!
//! [`SensorApp`] is a source-compatible wrapper for the not-yet-migrated
//! nRF52833 composition root. New roots should construct
//! [`sensor_sed_app::SensorApp`] directly from [`NrfWakeController`],
//! [`NrfStatus`], [`NrfSupervisor`], and [`NrfBattery`].

#![no_std]

pub mod app;
pub mod battery;
pub mod diagnostics;
pub mod environment;
pub mod platform;

pub use app::SensorApp;
pub use battery::{BatteryPolicy, NrfBattery};
pub use diagnostics::{NrfDiagnostics, persistence_failure};
pub use environment::OnChipTemperature;
pub use platform::{NrfStatus, NrfSupervisor, NrfWakeController, SensorMac};
pub use sensor_sed_app::{EnvironmentReading, EnvironmentSink, EnvironmentSource, policy};
