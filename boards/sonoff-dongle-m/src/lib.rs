//! BSP for the radio side of the SONOFF Dongle Max ("Dongle-M").
//!
//! The Dongle-M is an ESP32 host (Ethernet/PoE/USB/Wi-Fi) with an
//! EFR32MG24 radio; the two talk over one UART without RTS/CTS. The
//! EFR32 side has no USB of its own, so everything here is reached through
//! the ESP32. Sources (one unit probed read-only through the ESP32; nothing
//! read over SWD):
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
//!   same pins (any-pin routing; GSDK 4.5.0 routes USART0 TX/RX to the same
//!   PC01/PC02 for the SPI flash of the BRD4187C, `brd4187c_brd4001a/
//!   sl_mx25_flash_shutdown_usart_config.h`). The pins, crystal and CTUNE
//!   are **UNVERIFIED** against a unit.
//! - Stock firmware `zigbee_ncp_uart_hw_donglem_v7.4.5.gbl` (SHA-256
//!   `44b81c1e…a311fa`): EmberZNet 7.4.5 NCP linked at `0x0800_6000`
//!   (24 KiB bootloader), signature type NONE. Its RAIL configuration is
//!   GSDK's 802.15.4 PHY for a **38.4 MHz** crystal, byte for byte (the
//!   plain PHY, not the FEM or antenna-diversity variants); its PA
//!   configuration is `RAIL_TX_POWER_MODE_2P4GIG_HIGHEST`, 3300 mV, 10 µs
//!   ramp (GSDK defaults); its EmberZNet NVM3 sits at
//!   `0x0817_6000..0x0817_E000`.
//! - Read-only probe of a unit (stock firmware, through the ESP32): Gecko
//!   Bootloader v2.04.02 (menu `1. upload gbl / 2. run / 3. ebl info`),
//!   entered with GPIO12 high during a GPIO15 reset; EmberZNet 7.4.5.0,
//!   EZSP 13; EZSP EUI-64 `B0E8E8FFFE5D4D03`. Whether that EUI is the
//!   DEVINFO one or an `MFG_CUSTOM_EUI_64` token was not read; the NCP
//!   follows EmberZNet's precedence either way
//!   (`efr32s2_hal::devinfo::ieee_address`).
//!
//! **Unknown, to be measured on a unit:** the `MFG_CTUNE` token (the
//! precedence of [`hfxo_ctune_with_source`] is the stock one), the MG24 pin
//! of the bootloader-entry line (and its polarity at the MG24), any MG24
//! LED or button (none documented), the RF path/antenna switch, SWD test
//! pads, and whether an LFXO is fitted. None of these are exposed until
//! they are known.

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
/// Maximum TX power of the A420 part's PA (+20 dBm; the stock image
/// selects the highest PA). The certified limit of this product is not
/// published.
pub const PA_MAX_DBM: i8 = 20;
/// Stock Gecko bootloader size (application at `0x0800_6000`).
pub const GECKO_BOOTLOADER_SIZE: u32 = 0x6000;
/// Line rate of the stock bootloader (XMODEM) and of the stock NCP.
pub const STOCK_BAUD: u32 = 115_200;

pub const UART_TX_PORT: Port = Port::C;
pub const UART_TX_PIN: u8 = 1;
pub const UART_RX_PORT: Port = Port::C;
pub const UART_RX_PIN: u8 = 2;

/// HFXO CTUNE and where it came from, with GSDK's precedence: the module
/// calibration (`DEVINFO.MODXOCAL`, Silicon Labs modules only), the
/// factory `MFG_CTUNE` token, then [`DEFAULT_HFXO_CTUNE`].
pub fn hfxo_ctune_with_source() -> (u16, devinfo::CtuneSource) {
    devinfo::hfxo_ctune(DEFAULT_HFXO_CTUNE)
}

/// HFXO CTUNE ([`hfxo_ctune_with_source`]).
pub fn hfxo_ctune() -> u16 {
    hfxo_ctune_with_source().0
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
        // 38.4 MHz PHY selected (`hfxo-38m4hz`): the radio is supported and
        // the HP-PA table tops out at the A420's 20 dBm.
        assert_eq!(efr32s2_hal::radio::TX_POWER_RANGE_DBM, (-24, PA_MAX_DBM));
    }
}
