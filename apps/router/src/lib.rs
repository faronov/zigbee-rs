//! Reusable always-on Zigbee end-device, router, and coordinator lifecycle.
//!
//! The public frontends encode the role matrix at compile time:
//!
//! - [`AlwaysOnEndDeviceApp`] owns a non-routing
//!   [`zigbee_runtime::role::EndDevice`] whose receiver remains on while
//!   idle;
//! - the `router` feature additionally exposes `RelayRouterApp`,
//!   `ParentRouterApp`, and `CoordinatorApp`, all backed by a real
//!   [`zigbee_mac::ParentMacDriver`]-capable MAC.
//!
//! Startup and pending-action ticks are also selected statically:
//! End Device and router frontends can construct only Network Steering
//! futures, while the coordinator frontend alone can construct Network
//! Formation and persisted-PAN restart futures.
//! Relay and parent frontends also expose an urgent journal-aware factory
//! reset operation that a product can call directly before `step()`.
//!
//! The crate owns commissioning/resume/rejoin, bounded receive/tick
//! scheduling, durable security checkpoints, and parent child-table
//! lifecycle. It does not own platform startup, pins, fitted peripherals,
//! product identity, or profile behavior. Every integration capability is
//! statically selected; there is no allocator, trait object, runtime role
//! switch, or generic platform provider.

#![no_std]

mod app;
mod capabilities;
mod children;
mod diagnostics;
mod error;
mod observer;
mod parts;
mod policy;

pub use app::{AlwaysOnEndDeviceApp, StepEvents};
#[cfg(feature = "router")]
pub use app::{CoordinatorApp, ParentRouterApp, RelayRouterApp};
pub use capabilities::{
    NoStatus, NoSupervisor, NodeArchetype, RouterStatus, StatusSink, Supervisor,
};
pub use children::{NoChildren, PersistentChildren};
pub use diagnostics::{
    DiagnosticEvent, Diagnostics, NoDiagnostics, StackEventSummary, summarize_stack_event,
};
pub use error::RouterAppError;
pub use observer::{NoObserver, RouterObserver};
pub use parts::RouterParts;
pub use policy::RouterPolicy;
