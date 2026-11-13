//! Product configuration for the P-NUCLEO-WB55 USB dongle Zigbee Router.
//!
//! A plain mains-powered Range Extender on endpoint 1 (Basic, Identify):
//! the flash layout, journals, router policy, profile, gesture classifier
//! and status palette come from the shared STM32WB router kit
//! (`products/stm32wb-router`, re-exported here). Physical wiring lives in
//! `boards/wb55-usb-dongle`.
//!
//! | Button | Short press | Hold 3 s |
//! |--------|-------------|----------|
//! | SW1 | factory-new: commission; joined: permit join 180 s | journal-aware factory reset |
//!
//! Identify is driven from the coordinator (Identify cluster). LED1 blue,
//! LED2 green and LED3 red show the status palette
//! ([`indication::STATUS_PALETTE`]): white = all three, amber = red+green.
//! The log goes to the dongle's own USB port (CDC ACM).

#![no_std]

use stm32wb_router_product::actions::{Action, ButtonMap};

#[cfg(target_os = "none")]
pub use stm32wb_router_product::storage;
pub use stm32wb_router_product::{
    ENDPOINT, MANUFACTURER, RECOMMISSION_AFTER_LEAVE_MS, actions, button, indication, layout,
    policy, role,
};

pub const ROUTER_MODEL: &str = "WB55-Dongle-Router";
pub const DATE_CODE: &str = "20261113";
pub const SW_BUILD: &str = "0.1.0";

/// USB identity of the log port: the pid.codes test VID/PID pair, which
/// is reserved for exactly this kind of non-commercial device.
pub const USB_VID: u16 = 0x1209;
pub const USB_PID: u16 = 0x0001;

pub const SW1: ButtonMap = ButtonMap {
    name: "SW1",
    short_press: Some(Action::CommissionOrPermitJoin),
    long_hold: Some(Action::FactoryReset),
};
/// The dongle's only button.
pub static BUTTONS: [ButtonMap; 1] = [SW1];

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
