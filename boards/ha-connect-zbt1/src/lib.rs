//! BSP for the Home Assistant Connect ZBT-1 (formerly SkyConnect).
//!
//! EFR32MG21A020F512IM32 behind a CP2102N USB-UART bridge. Wiring is from
//! the Nabu Casa firmware-builder manifests
//! (`manifests/nabucasa/skyconnect/*.yaml`): USART0 TX PA6, RX PA5,
//! CTS PB1, RTS PB0, 115200 8N1 with hardware flow control. No user LED
//! or button is wired to the EFR32 that this crate can confirm, so none is
//! exposed.
//!
//! HFXO tuning follows the GSDK precedence: the factory `MFG_CTUNE` token
//! in the user-data page when programmed, otherwise the xG21 device_init
//! default 140 (`sl_device_init_hfxo_config.h`, sdid200).
//!
//! The shipped Gecko Bootloader occupies `0x0..0x4000` and has no GPIO
//! activation; once an application replaces the NCP, the only ways back
//! into it are SWD or [`efr32s2_hal::gecko_bootloader::reboot_to_bootloader`]
//! from the running application. The product owns that policy and the
//! memory layout; this crate exposes fitted resources only.

#![no_std]

pub mod resources;

pub use efr32s2_hal::flash::Efr32s2Flash;
use efr32s2_hal::{
    clock::{ClockControl, ClockError, HfxoConfig, SystemClocks},
    devinfo,
    gpio::{Input, Pin, Port, PushPull},
    usart::UartTx,
};

pub const BOARD_NAME: &str = "Home Assistant Connect ZBT-1";
pub const MCU_PART: &str = "EFR32MG21A020F512IM32";
pub const HCLK_HZ: u32 = 38_400_000;
/// xG21 GSDK default used when the `MFG_CTUNE` token is erased.
pub const DEFAULT_HFXO_CTUNE: u16 = 140;
/// Resident Gecko Bootloader size; the application starts here.
pub const BOOTLOADER_SIZE: u32 = 0x4000;

pub const UART_TX_PORT: Port = Port::A;
pub const UART_TX_PIN: u8 = 6;
pub const UART_RX_PORT: Port = Port::A;
pub const UART_RX_PIN: u8 = 5;
pub const UART_CTS_PORT: Port = Port::B;
pub const UART_CTS_PIN: u8 = 1;
pub const UART_RTS_PORT: Port = Port::B;
pub const UART_RTS_PIN: u8 = 0;
pub const UART_BAUD: u32 = 115_200;

/// HFXO CTUNE for this unit: factory token, else the GSDK default.
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

/// Host UART through the CP2102N. Transmit-only for now (the HAL has no
/// UART receiver yet); RTS is held asserted and CTS/RX stay reserved.
pub struct HostUart {
    uart: UartTx,
    _rts: Pin<PushPull>,
    _cts: Pin<Input>,
    _rx: Pin<Input>,
}

impl HostUart {
    pub(crate) fn new(uart: UartTx, rts: Pin<PushPull>, cts: Pin<Input>, rx: Pin<Input>) -> Self {
        Self {
            uart,
            _rts: rts,
            _cts: cts,
            _rx: rx,
        }
    }

    #[inline]
    pub fn uart(&mut self) -> &mut UartTx {
        &mut self.uart
    }
}
