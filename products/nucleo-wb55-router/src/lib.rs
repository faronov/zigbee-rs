//! Product configuration for the Nucleo-WB55 Zigbee Router.
//!
//! A plain mains-powered Range Extender on endpoint 1 (Basic, Identify):
//! the flash layout, journals, router policy, profile, gesture classifier
//! and status palette come from the shared STM32WB router kit
//! (`products/stm32wb-router`, re-exported here). Physical wiring lives in
//! `boards/nucleo-wb55`.
//!
//! | Button | Short press | Hold 3 s |
//! |--------|-------------|----------|
//! | SW1 | factory-new: commission; joined: permit join 180 s | journal-aware factory reset |
//! | SW2 | identify 10 s | close permit join |
//! | SW3 | restart (resume from the journals) | - |
//!
//! LED1 blue, LED2 green and LED3 red show the status palette
//! ([`indication::STATUS_PALETTE`]): white = all three, amber = red+green.

#![no_std]

use stm32wb_router_product::actions::{Action, ButtonMap};

#[cfg(target_os = "none")]
pub use stm32wb_router_product::storage;
pub use stm32wb_router_product::{
    ENDPOINT, MANUFACTURER, RECOMMISSION_AFTER_LEAVE_MS, actions, button, indication, layout,
    policy, role,
};

pub const ROUTER_MODEL: &str = "Nucleo-WB55-Router";
pub const DATE_CODE: &str = "20261113";
pub const SW_BUILD: &str = "0.1.0";

pub const SW1: ButtonMap = ButtonMap {
    name: "SW1",
    short_press: Some(Action::CommissionOrPermitJoin),
    long_hold: Some(Action::FactoryReset),
};
pub const SW2: ButtonMap = ButtonMap {
    name: "SW2",
    short_press: Some(Action::Identify),
    long_hold: Some(Action::ClosePermitJoin),
};
pub const SW3: ButtonMap = ButtonMap {
    name: "SW3",
    short_press: Some(Action::Restart),
    long_hold: None,
};
/// SW1, SW2, SW3.
pub static BUTTONS: [ButtonMap; 3] = [SW1, SW2, SW3];

#[cfg(test)]
mod tests {
    use super::*;
    use stm32wb_router_product::actions::factory_reset_holds;

    #[test]
    fn only_a_sw1_hold_resets() {
        assert_eq!(factory_reset_holds(&BUTTONS), 1);
        assert_eq!(SW1.long_hold, Some(Action::FactoryReset));
    }

    #[test]
    fn sw1_commissions() {
        assert_eq!(SW1.short_press, Some(Action::CommissionOrPermitJoin));
    }

    #[test]
    fn identity_fits_basic_strings() {
        // ZCL character strings of Basic are at most 32 octets.
        assert!(ROUTER_MODEL.len() <= 32 && MANUFACTURER.len() <= 32);
        assert_eq!(DATE_CODE.len(), 8);
    }
}
