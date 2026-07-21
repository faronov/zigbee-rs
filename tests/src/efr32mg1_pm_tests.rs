//! Host-runnable mirror of `efr32mg1-hal`'s pure power-management logic.
//!
//! `efr32mg1-hal` targets `thumbv7em-none-eabi` only and is excluded from
//! the root workspace, so it is not built here as a normal dependency.
//! This file `#[path]`-includes the *exact same* `pm.rs` source instead of
//! duplicating its logic: the module's hardware-touching code is gated
//! behind `#[cfg(target_arch = "arm")]` and compiles out entirely on the
//! host, leaving only the pure tick/deadline/canary/bounded-poll helpers
//! (and their `#[cfg(test)]` unit tests) to build and run here.
#[path = "../../efr32mg1-hal/src/pm.rs"]
mod efr32mg1_pm;
