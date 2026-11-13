//! Product configuration for the STM32WB5MM-DK Zigbee Router.
//!
//! The product owns the Zigbee identity, the Range Extender profile, the
//! bounded router policy, the button gestures, and the internal-flash layout
//! of the three crash-safe journals directly below the CPU2 secure area.
//! Physical wiring lives in `boards/stm32wb5mm-dk`; the coprocessor MAC
//! transport and dual-core flash access live in `stm32wb-hal`; commissioning
//! and runtime lifecycle live in `apps/router`.
//!
//! ## Flash layout (`link/memory.x`, [`layout`])
//!
//! | Region | Range |
//! |--------|-------|
//! | Application | `0x0800_0000..0x080D_5000` (852 KiB) |
//! | Child-table journal | `0x080D_5000..0x080D_9000` |
//! | APS binding/group journal | `0x080D_9000..0x080D_D000` |
//! | Security journal | `0x080D_D000..0x080E_1000` |
//! | CPU2 secure area (FUS + 802.15.4 MAC 1.24.0) | `0x080E_1000..0x0810_0000` |
//!
//! Each journal sees 8 KiB of logical flash stored as one 32-bit word per
//! 64-bit ECC double-word ([`stm32wb_hal`'s `WordPerDoubleWord`]), i.e. 16 KiB
//! of physical flash. The start-up code refuses to touch flash if the FUS
//! reports a secure area that starts below [`layout::NV_END`].

#![no_std]

pub mod actions;
pub mod button;
pub mod indication;
pub mod layout;
pub mod measure;
pub mod motion;
pub mod router;
pub mod screen;
#[cfg(any(feature = "sensors", feature = "sound"))]
pub mod sensor_profile;
#[cfg(target_os = "none")]
pub mod storage;
pub mod touch;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const ROUTER_MODEL: &str = "STM32WB5MM-DK-Router";
pub const DATE_CODE: &str = "20261113";
pub const SW_BUILD: &str = "0.1.0";

pub const ENDPOINT: u8 = 1;

/// Security-journal role tags (shared convention across products).
pub mod role {
    pub const ROUTER: u16 = 0x5201;
}

/// After a Leave the router stays factory-new until button B1 is pressed or
/// this delay elapses.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;
