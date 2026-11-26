//! BSP for the xG24 Explorer Kit EK2703A (board BRD2703A).
//!
//! Wiring is from GSDK 4.5.0 `hardware/board/config/brd2703a` (revision
//! A02): LED0 PA4 and LED1 PA7 (active high), BTN0 PB2 and BTN1 PB3 (active
//! low), VCOM USART0 TX PA5 / RX PA6 to the on-board J-Link OB (RTS PA8 and
//! CTS PA9 are routed too; GSDK's RAILtest runs the VCOM without flow
//! control, and so does this crate), 39 MHz HFXO with board CTUNE 100,
//! 32.768 kHz LFXO on PD0/PD1, the 10 dBm PA to the on-board antenna. There is no
//! VCOM enable line and no current monitor.
//!
//! The lab unit carries an **EFR32MG24B210F1536IM48** (DEVINFO `PART`
//! 0x011804BA, rev 0x19); `RAC.RFLOCK0` locks its 20 dBm PA, RAIL drives RF
//! path 1. Its USERDATA page holds no `MFG_CTUNE` token, so CTUNE 100 is
//! the GSDK board value, not a per-board calibration.
//!
//! Fitted but not exposed: the mikroBUS and Qwiic connectors.
//!
//! The crate exposes fitted resources only; it owns no Zigbee runtime,
//! memory layout or persistence policy.

#![no_std]

pub mod resources;

pub use efr32s2_hal::flash::Efr32s2Flash;
use efr32s2_hal::{
    clock::{ClockControl, ClockError, HfxoConfig, SystemClocks},
    devinfo,
    gpio::Port,
};

pub const BOARD_NAME: &str = "xG24 Explorer Kit EK2703A";
pub const MCU_PART: &str = "EFR32MG24B210F1536IM48";
/// HFXO crystal (GSDK `brd2703a_a02.slcc`: `HFXO_FREQ` 39 MHz).
pub const HCLK_HZ: u32 = 39_000_000;
/// GSDK board default (`SL_DEVICE_INIT_HFXO_CTUNE` 100). Not measured on
/// our unit (no `MFG_CTUNE` token).
pub const DEFAULT_HFXO_CTUNE: u16 = 100;
/// Fitted internal flash (EFR32MG24B210F**1536**).
pub const INTERNAL_FLASH_CAPACITY: usize = 1536 * 1024;
/// The part's PA (B210: 10 dBm MP PA; `RAC.RFLOCK0` locks the 20 dBm PA).
pub const PA_MAX_DBM: i8 = 10;
pub const HAS_LFXO: bool = true;

pub const LED0_PORT: Port = Port::A;
pub const LED0_PIN: u8 = 4;
pub const LED1_PORT: Port = Port::A;
pub const LED1_PIN: u8 = 7;
pub const BUTTON0_PORT: Port = Port::B;
pub const BUTTON0_PIN: u8 = 2;
pub const BUTTON1_PORT: Port = Port::B;
pub const BUTTON1_PIN: u8 = 3;
pub const VCOM_TX_PORT: Port = Port::A;
pub const VCOM_TX_PIN: u8 = 5;
pub const VCOM_RX_PORT: Port = Port::A;
pub const VCOM_RX_PIN: u8 = 6;
pub const VCOM_BAUD: u32 = 115_200;

/// HFXO CTUNE: factory `MFG_CTUNE` token, else [`DEFAULT_HFXO_CTUNE`].
pub fn hfxo_ctune() -> u16 {
    devinfo::mfg_ctune().map_or(DEFAULT_HFXO_CTUNE, u16::from)
}

/// Start the 39 MHz HFXO before radio, UART or time-driver use.
pub fn init_clocks(mut clocks: ClockControl) -> Result<SystemClocks, ClockError> {
    clocks.configure_hfxo(HfxoConfig {
        frequency_hz: HCLK_HZ,
        ctune: hfxo_ctune(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_matches_hal_selection() {
        assert_eq!(HCLK_HZ, efr32s2_hal::clock::HFXO_HZ);
        assert_eq!(efr32s2_hal::chip::CHIP, efr32s2_hal::chip::Chip::Mg24);
        const { assert!(INTERNAL_FLASH_CAPACITY <= efr32s2_hal::flash::MAX_FLASH_CAPACITY) };
        // The MP-PA table tops out at the part's 10 dBm.
        assert_eq!(efr32s2_hal::radio::TX_POWER_RANGE_DBM.1, PA_MAX_DBM);
    }
}
