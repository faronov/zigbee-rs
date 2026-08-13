//! Shared Zigbee sleepy-end-device sensor application for Nordic nRF52
//! products.
//!
//! This is the *application* layer of the stack documented in
//! `docs/book/src/getting-started/architecture.md`:
//!
//! ```text
//! application  ← this crate (lifecycle, polling, reporting, button, LED)
//! product      ← identity, flash layout, persistence, battery chemistry
//! board        ← physical pin wiring
//! chip HAL     ← embassy-nrf (clocks, GPIO, SAADC, NVMC, radio)
//! ```
//!
//! It owns the complete production lifecycle that used to live in
//! `examples/nrf52840-sensor/src/app.rs`, moved here verbatim in behavior
//! and generalized over the three things that legitimately differ between
//! nRF sensor products:
//!
//! - `S: SecurityStateStore` — the product's crash-safe security journal
//!   over its own protected flash partition;
//! - `C: ProfileComponent + EnvironmentSink` — the product's cluster
//!   component (with or without a Pressure Measurement cluster);
//! - `E: EnvironmentSource` — the fitted environmental sensor (on-chip
//!   `TEMP`, or an external I2C part wired by the board crate);
//! - `B: BatteryPolicy` — the product's battery chemistry curve, bound by
//!   the composition root so this crate never depends on a product.
//!
//! Everything else — commissioning, silent resume, bounded secure-rejoin
//! retry with factory-reset fallback, the four-round MAC poll window,
//! fast/slow polling, interview detection, Device_annce retries, Identify
//! blink, durable checkpointing, and the button semantics — is identical
//! for every nRF sensor and is *not* duplicated per platform.
//!
//! The crate is `no_std` and builds only for `thumbv7em-none-eabihf`; the
//! pure poll arbitration in [`policy`] is additionally mirrored into the
//! workspace host test crate (`tests/src/nrf_sensor_policy_tests.rs`).

#![no_std]

pub mod app;
pub mod battery;
pub mod environment;
pub mod policy;

pub use app::{SensorApp, SensorMac, persistence_failure};
pub use battery::BatteryPolicy;
pub use environment::{EnvironmentReading, EnvironmentSink, EnvironmentSource, OnChipTemperature};
