//! Host-runnable mirror of the nRF52840 sensor's pure poll-delay
//! arbitration logic.
//!
//! `examples/nrf52840-sensor` targets `thumbv7em-none-eabihf` only and is
//! excluded from the root workspace, so it is not built here as a normal
//! dependency. This file `#[path]`-includes the *exact same* `policy.rs`
//! source instead of duplicating its logic — see
//! `tests/src/efr32mg1_pm_tests.rs` for the same technique applied to
//! `efr32mg1-hal`'s power-management module.
#[path = "../../examples/nrf52840-sensor/src/policy.rs"]
mod nrf52840_policy;
