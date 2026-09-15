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
//! Composition roots construct [`sensor_sed_app::SensorApp`] directly from
//! [`NrfWakeController`], [`NrfStatus`], [`NrfSupervisor`], and
//! [`NrfBattery`].

#![no_std]

pub mod battery;
pub mod diagnostics;
pub mod environment;
pub mod platform;

pub use battery::{BatteryPolicy, NrfBattery};
pub use diagnostics::{NrfDiagnostics, persistence_failure};
pub use environment::OnChipTemperature;
pub use platform::{
    NrfPolarityStatus, NrfStatus, NrfSupervisor, NrfTimerWakeController, NrfWakeController,
    SensorMac,
};
pub use sensor_sed_app::{EnvironmentReading, EnvironmentSink, EnvironmentSource, policy};
