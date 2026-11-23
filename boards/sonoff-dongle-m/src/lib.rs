//! BSP for the radio side of the SONOFF Dongle Max ("Dongle-M").
//!
//! The Dongle-M is an ESP32 host (Ethernet/PoE/USB/Wi-Fi) with an
//! EFR32MG24 radio; the two talk over one UART without RTS/CTS. The
//! EFR32 side has no USB of its own, so everything here is reached through
//! the ESP32. Sources (no unit has been inspected yet):
//!
//! - SONOFF hardware specification: **EFR32MG24A420F1536IM48** (20 dBm HP
//!   PA, 1536 KiB flash, 256 KiB RAM).
//! - SONOFF ESPHome guide: ESP32 GPIO13 = RX from the MG24, GPIO17 = TX to
//!   the MG24, 115200 8N1 default (460800/921600 selectable), GPIO15 drives
//!   the MG24 RESET (active low), GPIO12 high during reset enters the MG24
//!   Gecko bootloader (UART XMODEM at 115200).
//! - Community firmware builds for this board (Nerivec
//!   `silabs-firmware-builder`, `manifests/sonoff/sonoff_dongle-m_*.yaml`):
//!   MG24 **TX = PC01, RX = PC02**, no flow control, HFXO 38.4 MHz with
//!   CTUNE 100. Those builds use EUSART1; this crate routes USART0 to the
//!   same pins (any-pin routing). The pins, crystal and CTUNE are
//!   **UNVERIFIED** against a unit.
//! - Stock firmware: EmberZNet NCP (`zigbee_ncp_uart_hw_donglem_v7.4.5.gbl`)
//!   above a Gecko bootloader whose application starts at `0x0800_6000`
//!   (24 KiB bootloader). The stock and community GBLs carry no signature
//!   or encryption tags, and unsigned community GBLs are flashed through
//!   SONOFF's flasher, so the bootloader does not enforce signed images.
//!
//! **Unknown, to be measured on a unit:** the MG24 pin of the
//! bootloader-entry line (and its polarity at the MG24), any MG24 LED or
//! button (none documented), the RF path/antenna switch, the stock TX
//! power, SWD test pads, and whether an LFXO is fitted. None of these are
//! exposed until they are known.

#![no_std]

pub mod resources;

pub use efr32s2_hal::flash::Efr32s2Flash;
use efr32s2_hal::{
    clock::{ClockControl, ClockError, HfxoConfig, SystemClocks},
    devinfo,
    gpio::Port,
};

pub const BOARD_NAME: &str = "SONOFF Dongle-M";
pub const MCU_PART: &str = "EFR32MG24A420F1536IM48";
/// HFXO crystal: 38.4 MHz per the community manifests (UNVERIFIED).
pub const HCLK_HZ: u32 = 38_400_000;
/// CTUNE of the community manifests (UNVERIFIED); a programmed
/// `MFG_CTUNE` token on the unit takes precedence.
pub const DEFAULT_HFXO_CTUNE: u16 = 100;
/// Fitted internal flash (EFR32MG24A420F**1536**).
pub const INTERNAL_FLASH_CAPACITY: usize = 1536 * 1024;
/// Maximum TX power of the A420 part's PA (+20 dBm). The certified limit
/// of this product is not published.
pub const PA_MAX_DBM: i8 = 20;
/// Stock Gecko bootloader size (application at `0x0800_6000`).
pub const GECKO_BOOTLOADER_SIZE: u32 = 0x6000;
/// Line rate of the stock bootloader (XMODEM) and of the stock NCP.
pub const STOCK_BAUD: u32 = 115_200;

pub const UART_TX_PORT: Port = Port::C;
pub const UART_TX_PIN: u8 = 1;
pub const UART_RX_PORT: Port = Port::C;
pub const UART_RX_PIN: u8 = 2;

/// HFXO CTUNE: factory `MFG_CTUNE` token, else [`DEFAULT_HFXO_CTUNE`].
pub fn hfxo_ctune() -> u16 {
    devinfo::mfg_ctune().map_or(DEFAULT_HFXO_CTUNE, u16::from)
}

/// Start the 38.4 MHz HFXO before radio, UART or time-driver use.
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
