//! BSP for the BRD4187 xG24 +20 dBm radio board on the BRD4001A Wireless
//! Starter Kit (WSTK).
//!
//! Wiring is from GSDK 4.5.0 `hardware/board/config/brd4187{a,c}_brd4001a`
//! (identical pin maps): LED0 PB2 and LED1 PB4 (active high), BTN0 PB1 and
//! BTN1 PB3 (active low, WSTK pull-ups), VCOM USART0 TX PA8 / RX PA9 (RTS
//! PA0, CTS PB5, unused) with the WSTK VCOM enable on PB0, 39 MHz HFXO with
//! board CTUNE 95, the 20 dBm PA path to the PCB antenna.
//!
//! The lab unit (BRD4187A Rev A01, WSTK board id) carries an
//! **EFR32MG24B220F1536IM48** (DEVINFO `PART` 0x011804C4, rev 0x19), not
//! the A020 GSDK lists for revision A; its USERDATA page holds no
//! `MFG_CTUNE` token and DEVINFO no module calibration, so CTUNE 95 is the
//! GSDK board value (`sl_device_init_hfxo_config.h`), not a per-board
//! calibration.
//!
//! Fitted but not exposed: the MX25R8035F SPI flash, the Si7021 sensor and
//! the memory LCD (WSTK), and the PTI pins (PD4/PD5).
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

pub const BOARD_NAME: &str = "BRD4187 on BRD4001A";
pub const MCU_PART: &str = "EFR32MG24B220F1536IM48";
/// HFXO crystal (GSDK `brd4187*.slcc`: `HFXO_FREQ` 39 MHz).
pub const HCLK_HZ: u32 = 39_000_000;
/// GSDK board default (`SL_DEVICE_INIT_HFXO_CTUNE` 95). Not measured on our
/// unit (no `MFG_CTUNE` token).
pub const DEFAULT_HFXO_CTUNE: u16 = 95;
/// Fitted internal flash (EFR32MG24B220F**1536**).
pub const INTERNAL_FLASH_CAPACITY: usize = 1536 * 1024;
/// The part's PA (B220: 20 dBm HP PA; `RAC.RFLOCK0` locks the 0/10 dBm PAs).
pub const PA_MAX_DBM: i8 = 20;
pub const HAS_LFXO: bool = true;

pub const LED0_PORT: Port = Port::B;
pub const LED0_PIN: u8 = 2;
pub const LED1_PORT: Port = Port::B;
pub const LED1_PIN: u8 = 4;
pub const BUTTON0_PORT: Port = Port::B;
pub const BUTTON0_PIN: u8 = 1;
pub const BUTTON1_PORT: Port = Port::B;
pub const BUTTON1_PIN: u8 = 3;
pub const VCOM_ENABLE_PORT: Port = Port::B;
pub const VCOM_ENABLE_PIN: u8 = 0;
pub const VCOM_TX_PORT: Port = Port::A;
pub const VCOM_TX_PIN: u8 = 8;
pub const VCOM_RX_PORT: Port = Port::A;
pub const VCOM_RX_PIN: u8 = 9;
/// VCOM line rate. The BRD4001A board controller runs its side of the VCOM
/// UART at its own configured rate (default 115 200, set from the WSTK
/// admin console) rather than the host's CDC line coding: measured with
/// the ZRHP NCP, 115 200 is clean, 460 800 and 921 600 fail with framing
/// errors on both ends.
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
        // The HP-PA table tops out at the part's 20 dBm.
        assert_eq!(efr32s2_hal::radio::TX_POWER_RANGE_DBM.1, PA_MAX_DBM);
    }
}
