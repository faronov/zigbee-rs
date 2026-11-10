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
//! - [`SensorMeasurement`] (any [`EnvironmentSource`], or a product-specific
//!   source such as a plant sensor's) and [`BatterySource`] supply fitted
//!   measurements;
//! - [`OtaLifecycle`] is explicitly paired with the selected profile;
//! - [`Diagnostics`] renders typed lifecycle events through a product-selected
//!   transport;
//! - `zigbee-runtime` supplies the typed profile and product-selected
//!   [`SecurityStateStore`](zigbee_runtime::security_store::SecurityStateStore).
//!
//! All capabilities are static generic parameters. There is no allocation,
//! trait object, runtime hardware discovery, or platform `cfg` in this crate.

#![no_std]

pub mod anomaly;
pub mod app;
pub mod battery;
pub mod capabilities;
pub mod diagnostics;
pub mod environment;
pub mod measurement;
pub mod ota;
#[cfg(feature = "ota")]
pub mod ota_client;
pub mod parts;
pub mod policy;
pub mod reset_cause;

pub use anomaly::{
    ANOMALY_RECORD_LEN, AnomalyJournal, AnomalyReason, AnomalyRecord, AwakeBound, BoundStep,
};
pub use app::{FindingBindingTargetError, SensorApp, SensorAppError, SensorLifecycleError};
pub use battery::{
    BatteryReading, BatterySource, BlockingBattery, BlockingBatterySource, FixedBattery, NoBattery,
};
pub use capabilities::{
    NoStatus, SensorStatus, StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
pub use diagnostics::{DiagnosticEvent, Diagnostics};
pub use environment::{
    BlockingEnvironment, BlockingEnvironmentSource, EnvironmentReading, EnvironmentSink,
    EnvironmentSource, EnvironmentalSensorProfile,
};
pub use measurement::{NoMeasurement, SampleTrigger, SensorMeasurement, SensorProfile};
pub use ota::{
    NoOta, NonOtaComponent, NonOtaProfile, OtaActivationOutcome, OtaEventOutcome, OtaLifecycle,
    OtaServiceOutcome, is_ota_event,
};
pub use parts::SensorSedParts;
pub use policy::{
    ButtonPolicy, ForceReportAction, JOIN_RETRY_MAX_MS, JoinOnlyAction, MAX_JOIN_RETRY_JITTER_PCT,
    MAX_WAIT_MS, NoUserAction, PARENT_INDIRECT_HOLD_MARGIN_MS, PARENT_INDIRECT_HOLD_MIN_MS,
    RESPONSIVE_SLOW_POLL_MS, RecoveryPolicy, SensorPolicy, ShortPressAction, SleepDepth,
    StatusPolicy, ToggleJoinAction, UserActionPolicy,
};
pub use reset_cause::{
    BootRecord, RETAINED_WORDS, RapidResetPolicy, ResetCause, ResetKind, RetainedBoot,
};
