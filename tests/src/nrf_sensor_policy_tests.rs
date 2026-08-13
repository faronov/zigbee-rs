//! Host-runnable mirror of the shared nRF sensor application's pure
//! poll-delay arbitration logic.
//!
//! `apps/nrf-sensor` targets `thumbv7em-none-eabihf` only (it depends on
//! `embassy-nrf`) and is excluded from the root workspace, so it is not
//! built here as a normal dependency. This file `#[path]`-includes the
//! *exact same* `policy.rs` source instead of duplicating its logic — see
//! `tests/src/efr32mg1_pm_tests.rs` for the same technique applied to
//! `efr32mg1-hal`'s power-management module.
//!
//! Both the nRF52840 and nRF52833 sensor firmwares run this file verbatim.
#[path = "../../apps/nrf-sensor/src/policy.rs"]
mod nrf_sensor_policy;
