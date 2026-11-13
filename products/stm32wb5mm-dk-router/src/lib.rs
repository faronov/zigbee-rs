//! Product configuration for the STM32WB5MM-DK Zigbee Router.
//!
//! The product owns the Zigbee identity, the DK's optional sensor profile,
//! its button and touch gestures, and its display pages. The flash layout,
//! journals, router policy, gesture classifier and status palette are shared
//! with the other 1 MiB STM32WB55 routers in `products/stm32wb-router`
//! (re-exported here). Physical wiring lives in `boards/stm32wb5mm-dk`; the
//! coprocessor MAC transport and dual-core flash access live in
//! `stm32wb-hal`; commissioning and runtime lifecycle live in `apps/router`.

#![no_std]

pub mod actions;
pub mod measure;
pub mod motion;
pub mod router;
pub mod screen;
#[cfg(any(feature = "sensors", feature = "sound"))]
pub mod sensor_profile;
pub mod touch;

#[cfg(target_os = "none")]
pub use stm32wb_router_product::storage;
pub use stm32wb_router_product::{
    ENDPOINT, MANUFACTURER, RECOMMISSION_AFTER_LEAVE_MS, button, indication, layout, role,
};

pub const ROUTER_MODEL: &str = "STM32WB5MM-DK-Router";
pub const DATE_CODE: &str = "20261113";
pub const SW_BUILD: &str = "0.1.0";
