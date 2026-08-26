//! Reusable Zigbee sleepy-end-device sensor lifecycle.
//!
//! This crate owns commissioning, resume/rejoin handling, poll windows,
//! reporting cadence, button semantics, status indication, and durable
//! security checkpoints. It is deliberately unaware of any chip, board, or
//! product:
//!
//! - [`WakeController`] owns rollover-safe monotonic time, user wake input,
//!   and the atomic MAC wait/readiness transition;
//! - [`StatusSink`] maps semantic lifecycle status to fitted indicators;
//! - [`Supervisor`] owns reset and watchdog servicing;
//! - [`EnvironmentSource`] and [`BatterySource`] supply fitted measurements;
//! - [`OtaLifecycle`] is explicitly paired with the selected profile;
//! - [`Diagnostics`] renders typed lifecycle events through a product-selected
//!   transport;
//! - `zigbee-runtime` supplies the typed profile and product-selected
//!   [`SecurityStateStore`](zigbee_runtime::security_store::SecurityStateStore).
//!
//! All capabilities are static generic parameters. There is no allocation,
//! trait object, runtime hardware discovery, or platform `cfg` in this crate.

#![no_std]

pub mod app;
pub mod battery;
pub mod capabilities;
pub mod diagnostics;
pub mod environment;
pub mod ota;
pub mod parts;
pub mod policy;

pub use app::{SensorApp, SensorAppError, SensorLifecycleError};
pub use battery::{
    BatteryReading, BatterySource, BlockingBattery, BlockingBatterySource, FixedBattery,
};
pub use capabilities::{
    NoStatus, SensorStatus, StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
pub use diagnostics::{DiagnosticEvent, Diagnostics};
pub use environment::{
    BlockingEnvironment, BlockingEnvironmentSource, EnvironmentReading, EnvironmentSink,
    EnvironmentSource, EnvironmentalSensorProfile,
};
pub use ota::{
    NoOta, NonOtaComponent, NonOtaProfile, OtaActivationOutcome, OtaEventOutcome, OtaLifecycle,
    OtaServiceOutcome, is_ota_event,
};
pub use parts::SensorSedParts;
pub use policy::{
    ButtonPolicy, ForceReportAction, JoinOnlyAction, NoUserAction, SensorPolicy, ShortPressAction,
    SleepDepth, StatusPolicy, ToggleJoinAction, UserActionPolicy,
};
