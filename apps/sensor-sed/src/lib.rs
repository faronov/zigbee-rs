//! Reusable Zigbee sleepy-end-device sensor lifecycle.
//!
//! This crate owns commissioning, resume/rejoin handling, poll windows,
//! reporting cadence, button semantics, status indication, and durable
//! security checkpoints. It is deliberately unaware of any chip, board, or
//! product:
//!
//! - [`LifecyclePlatform`] supplies the monotonic clock, button/LED behavior,
//!   bounded delays, and reset mechanism;
//! - [`RadioPower`] supplies the MAC-specific pre-sleep transition;
//! - [`EnvironmentSource`] and [`BatterySource`] supply fitted measurements;
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
pub mod policy;

pub use app::SensorApp;
pub use battery::{BatteryReading, BatterySource};
pub use capabilities::{LifecyclePlatform, RadioPower, WakeReason};
pub use diagnostics::{DiagnosticEvent, Diagnostics};
pub use environment::{EnvironmentReading, EnvironmentSink, EnvironmentSource};
