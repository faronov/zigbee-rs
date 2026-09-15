//! Reusable always-on Zigbee end-device, router, and coordinator lifecycle.
//!
//! The public frontends encode the role matrix at compile time:
//!
//! - [`AlwaysOnEndDeviceApp`] owns a non-routing
//!   [`zigbee_runtime::role::EndDevice`] whose receiver remains on while
//!   idle;
//! - the `router` feature additionally exposes `RelayRouterApp`,
//!   `ParentRouterApp`, `DistributedRouterApp`, and `CoordinatorApp`, all
//!   backed by a real [`zigbee_mac::ParentMacDriver`]-capable MAC.
//! - `trust-center` adds `TrustCenterCoordinatorApp`, including durable
//!   link-key transactions and centralized Network-Key rotation.
//!
//! Startup and pending-action ticks are also selected statically:
//! End Device, relay, and parent frontends can construct only Network
//! Steering futures; the distributed-router frontend can construct only
//! distributed Network Formation/restart futures; and the coordinator
//! frontend can construct only centralized Network Formation/restart futures.
//! Relay and parent frontends also expose an urgent journal-aware factory
//! reset operation that a product can call directly before `step()`.
//!
//! The crate owns commissioning/resume/rejoin, bounded receive/tick
//! scheduling, durable security checkpoints, APS binding/group persistence,
//! and parent child-table lifecycle. It does not own platform startup, pins,
//! fitted peripherals, product identity, or profile behavior. Every
//! integration capability is statically selected; there is no allocator,
//! trait object, runtime role switch, or generic platform provider.

#![no_std]

/// Await a future through a `dyn Future` reference so its generated code stays
/// out of the awaiting coroutine.
///
/// `.await` on an `async fn` embeds *both* the callee's state and its code in
/// the caller's coroutine, so a lifecycle step that is awaited from several
/// textual places is emitted once per call site. The joined router step awaits
/// the same persistence and child-lifecycle steps two or three times each
/// (before receive, after parent-command staging, and after the tick), which
/// made one flattened `step_joined` coroutine dominate the Telink router image.
///
/// Pinning the future here and erasing it to `Pin<&mut dyn Future>` keeps its
/// state pinned in this frame — no allocation, no extra transient stack — while
/// forcing the callee's body to be emitted once, out of line. This mirrors the
/// identical `zigbee-runtime` idiom used in `ZigbeeDevice::tick_joined`.
macro_rules! await_out_of_line {
    ($future:expr) => {{
        let future = core::pin::pin!($future);
        let future: core::pin::Pin<&mut dyn core::future::Future<Output = _>> = future;
        future.await
    }};
}

mod app;
mod capabilities;
mod children;
mod diagnostics;
mod error;
mod observer;
mod parts;
mod policy;

#[cfg(feature = "trust-center")]
pub use app::TrustCenterCoordinatorApp;
pub use app::{AlwaysOnEndDeviceApp, NoApsTables, PersistentApsTables, StepEvents};
#[doc(hidden)]
pub use app::{ApsRestore, ApsTableLifecycle};
#[cfg(feature = "router")]
pub use app::{CoordinatorApp, DistributedRouterApp, ParentRouterApp, RelayRouterApp};
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
