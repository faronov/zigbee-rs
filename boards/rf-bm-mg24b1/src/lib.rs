//! BSP for an RF-star RF-BM-MG24B1 module wired as a development board.
//!
//! Module facts (RF-star datasheet RF-BM-MG24B1/B2/B2I V1.0, March 2025):
//! EFR32MG24A410F1536IM48-B (+10 dBm MP PA, 1536 KiB flash, 256 KiB RAM),
//! 39 MHz HFXO and 32.768 kHz LFXO crystals on the module, PCB antenna.
//! The datasheet asks for HFXO CTUNE 92 on B1/B2 (the module has no
//! external load capacitors); that value has **not been measured** on our
//! units, so [`hfxo_ctune`] prefers a programmed `MFG_CTUNE` token.
//!
//! Lab wiring (module pin → net), the hub's `S3_MG24_DEV` profile:
//!
//! | pin | MCU | use |
//! |---|---|---|
//! | 12 | PA05 | USART0 TX → host RX (ESP32-S3 SuperMini) |
//! | 13 | PA06 | USART0 RX ← host TX |
//! | 14 | PA07 | optional RTS (not used: no flow control) |
//! | 15 | PA08 | optional CTS (not used: no flow control) |
//! | 17 | PA02 | SWDIO (J-Link via hwlab) |
//! | 18 | PA01 | SWCLK |
//! | 10 | PA03 | SWO |
//! | 36 | RESET_N | J-Link and ESP32-S3 GPIO6 (open-drain), internal pull-up |
//! | 34/35 | VDD/GND | 3.3 V |
//!
//! No LED or button is fitted. The SWD/SWO pins are never claimed, so the
//! debugger keeps working. Nothing about the flash contents of a new
//! module is known (blank, RF-star demo or a bootloader): inspect over SWD
//! before writing.

#![no_std]

pub mod resources;

pub use efr32s2_hal::flash::Efr32s2Flash;
use efr32s2_hal::{
    clock::{ClockControl, ClockError, HfxoConfig, SystemClocks},
    devinfo,
    gpio::Port,
};

pub const BOARD_NAME: &str = "RF-BM-MG24B1 dev board";
pub const MCU_PART: &str = "EFR32MG24A410F1536IM48";
/// HFXO crystal (module datasheet: 39 MHz).
pub const HCLK_HZ: u32 = 39_000_000;
/// RF-star datasheet section 4: CTUNE 92 for RF-BM-MG24B1/B2.
/// UNVERIFIED on our units (measure the carrier offset on hardware).
pub const DEFAULT_HFXO_CTUNE: u16 = 92;
/// Fitted internal flash (EFR32MG24A410F**1536**).
pub const INTERNAL_FLASH_CAPACITY: usize = 1536 * 1024;
/// Maximum TX power of the A410 part's PA (datasheet: +10 dBm).
pub const PA_MAX_DBM: i8 = 10;
/// The module carries a 32.768 kHz LFXO crystal (not used yet: the time
/// base runs on the LFRCO).
pub const HAS_LFXO: bool = true;

pub const UART_TX_PORT: Port = Port::A;
pub const UART_TX_PIN: u8 = 5;
pub const UART_RX_PORT: Port = Port::A;
pub const UART_RX_PIN: u8 = 6;
pub const UART_RTS_PORT: Port = Port::A;
pub const UART_RTS_PIN: u8 = 7;
pub const UART_CTS_PORT: Port = Port::A;
pub const UART_CTS_PIN: u8 = 8;

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
    }
}
