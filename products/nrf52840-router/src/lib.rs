//! Product configuration for the nRF52840-DK mains roles: the always-on
//! Zigbee End Device and the Zigbee Router.
//!
//! The product owns both Zigbee identities, the Basic/Identify profiles,
//! bounded always-on scheduling policies, semantic LED1 mapping, Button 1
//! gestures, the crash-safe security journal at the top of flash, and the
//! router's child/APS journals directly below it. Physical DK wiring remains
//! in `boards/nrf52840-dk`; commissioning and runtime lifecycle remain in
//! `apps/router`.
//!
//! ## Flash layout (both role images link the same map, `link/memory.x`)
//!
//! | Region | Range |
//! |--------|-------|
//! | Application | `0x0000_0000..0x000F_A000` (1000 KiB) |
//! | Router child/APS journals | `0x000F_A000..0x000F_E000` |
//! | Security journal | `0x000F_E000..0x0010_0000` |

#![no_std]

pub mod button;
#[cfg(target_os = "none")]
mod partition;
pub mod policy;
pub mod profile;
pub mod router;
pub mod status;
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "nRF52840-AlwaysOn-ED";
pub const DATE_CODE: &str = "20260405";
pub const SW_BUILD: &str = "0.1.0";

pub const ENDPOINT: u8 = 1;

/// Security-journal role tags. The router image selects [`role::ROUTER`]
/// before resuming, so it never resumes state persisted by another role
/// image; the always-on End Device image predates role tags and does not
/// check them (leave the network or factory-reset before switching a
/// commissioned board from the router to the End Device image).
pub mod role {
    pub const ROUTER: u16 = 0x5401;
}

pub const ROUTER_MODEL: &str = "nRF52840-Router";

/// After a Leave the router stays factory-new until Button 1 is pressed or
/// this delay elapses.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;
