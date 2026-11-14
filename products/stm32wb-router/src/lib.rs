//! Product policy shared by the 1 MiB STM32WB55 Zigbee routers
//! (STM32WB5MM-DK, Nucleo-WB55, USB dongle).
//!
//! Every such product runs CPU2's 802.15.4 MAC 1.24.0 under FUS 1.2.0, which
//! puts the secure area at `0x080E_1000` (SFSA = 0xE1), and keeps its
//! network state in the same crash-safe journals directly below it. This
//! crate owns that common policy so each product only adds its identity
//! and its button map:
//!
//! - [`layout`] and `link/memory.x`: the internal-flash layout;
//! - `storage` (target only): the four journals on that layout;
//! - [`policy`]: the bounded router policy and the plain Range Extender
//!   profile;
//! - [`button`], [`actions`]: gesture classification and the shared action
//!   vocabulary;
//! - [`indication`]: the status palette (RGB or three discrete LEDs).
//!
//! Physical wiring lives in the board crates; the coprocessor transport and
//! dual-core flash access in `stm32wb-hal`; commissioning and the runtime
//! lifecycle in `apps/router`.
//!
//! ## Flash layout (`link/memory.x`, [`layout`])
//!
//! | Region | Range |
//! |--------|-------|
//! | Application | `0x0800_0000..0x080D_3000` (844 KiB) |
//! | Settings journal | `0x080D_3000..0x080D_5000` |
//! | Child-table journal | `0x080D_5000..0x080D_9000` |
//! | APS binding/group journal | `0x080D_9000..0x080D_D000` |
//! | Security journal | `0x080D_D000..0x080E_1000` |
//! | CPU2 secure area (FUS + 802.15.4 MAC 1.24.0) | `0x080E_1000..0x0810_0000` |
//!
//! Each journal sees half its physical size as logical flash, stored as one
//! 32-bit word per 64-bit ECC double-word (`stm32wb_hal`'s
//! `WordPerDoubleWord`). The start-up code refuses to touch flash if the
//! FUS reports a secure area that starts below [`layout::NV_END`].

#![no_std]

pub mod actions;
pub mod button;
pub mod indication;
pub mod layout;
pub mod policy;
#[cfg(target_os = "none")]
pub mod storage;

/// Security-journal role tags (shared convention across products).
pub mod role {
    pub const ROUTER: u16 = 0x5201;
}

/// Manufacturer name of every zigbee-rs product.
pub const MANUFACTURER: &str = "Zigbee-RS";

/// The Range Extender endpoint of the plain router profile.
pub const ENDPOINT: u8 = 1;

/// After a Leave the router stays factory-new until its commissioning
/// button is pressed or this delay elapses.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;
