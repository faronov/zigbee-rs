//! BSP for the Tuya ZS3L Zigbee module wired to a host MCU (for example the
//! ESP32-S3 of a Tuya T3E-class wall panel).
//!
//! Module facts (Tuya "ZS3L Module Datasheet"): EFR32MG21A020F768IM32-B
//! (+20 dBm PA, 768 KiB flash, 64 KiB RAM), 38.4 MHz HFXO, PCB antenna,
//! no LFXO crystal is documented.
//!
//! | module pin | MCU | use |
//! |---|---|---|
//! | 11 / 10 | PD00 / PD01 | gateway UART TX → host RX / RX ← host TX (`uart-gateway`) |
//! | 6 / 7 | PA03 / PA04 | gateway CTS / RTS (Tuya: flow control on by default; not used here) |
//! | 16 / 15 | PA05 / PA06 | TXD / RXD pads (`uart-pa05-pa06`) |
//! | 2 | PC01 | held low at reset: Gecko bootloader (Tuya gateway firmware) |
//! | 1 | RST | active-low reset, driven by the host |
//! | 18 / 19 | SWDIO / SWCLK | "burning" pads, not routed by default |
//!
//! **Unverified**: which route a given product (e.g. the T3E variant with a
//! ZS3L instead of the TLSR8258 ZT3L) wires to its host; the HFXO CTUNE of
//! the module (no `MFG_CTUNE` token is known); whether the resident
//! bootloader keeps the PC01 GPIO activation. Inspect a unit over SWD
//! before writing it. No unit has run any image of this crate.

#![no_std]

pub mod resources;

#[cfg(all(feature = "uart-gateway", feature = "uart-pa05-pa06"))]
compile_error!("tuya-zs3l: enable exactly one host-UART route");
#[cfg(not(any(feature = "uart-gateway", feature = "uart-pa05-pa06")))]
compile_error!("tuya-zs3l: enable a host-UART route (`uart-gateway` or `uart-pa05-pa06`)");

pub use efr32s2_hal::flash::Efr32s2Flash;
use efr32s2_hal::{
    clock::{ClockControl, ClockError, HfxoConfig, SystemClocks},
    devinfo,
    gpio::Port,
};

pub const BOARD_NAME: &str = "Tuya ZS3L";
pub const MCU_PART: &str = "EFR32MG21A020F768IM32";
pub const HCLK_HZ: u32 = 38_400_000;
/// CTUNE when the `MFG_CTUNE` token is erased. UNVERIFIED for this module:
/// the ZBT-1 (also an A020 part) runs at 140; measure the carrier offset.
pub const DEFAULT_HFXO_CTUNE: u16 = 140;
/// Fitted internal flash (EFR32MG21A020F**768**).
pub const INTERNAL_FLASH_CAPACITY: usize = 768 * 1024;
/// Usable SRAM of the F768 part (DEVINFO `MSIZE.SRAM` 64 KiB).
pub const SRAM_SIZE: usize = 64 * 1024;
/// Maximum TX power of the A020 part's PA (+20 dBm).
pub const PA_MAX_DBM: i8 = 20;

#[cfg(feature = "uart-gateway")]
mod route {
    use efr32s2_hal::gpio::Port;
    pub const TX: (Port, u8) = (Port::D, 0);
    pub const RX: (Port, u8) = (Port::D, 1);
}
#[cfg(feature = "uart-pa05-pa06")]
mod route {
    use efr32s2_hal::gpio::Port;
    pub const TX: (Port, u8) = (Port::A, 5);
    pub const RX: (Port, u8) = (Port::A, 6);
}

pub const UART_TX_PORT: Port = route::TX.0;
pub const UART_TX_PIN: u8 = route::TX.1;
pub const UART_RX_PORT: Port = route::RX.0;
pub const UART_RX_PIN: u8 = route::RX.1;
/// Gateway-route flow-control pins (claimed so nothing drives them).
pub const UART_CTS_PORT: Port = Port::A;
pub const UART_CTS_PIN: u8 = 3;
pub const UART_RTS_PORT: Port = Port::A;
pub const UART_RTS_PIN: u8 = 4;
/// Bootloader-entry pin (low during reset) of Tuya's gateway firmware.
pub const BOOTLOAD_PORT: Port = Port::C;
pub const BOOTLOAD_PIN: u8 = 1;

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
        assert_eq!(efr32s2_hal::chip::CHIP, efr32s2_hal::chip::Chip::Mg21);
        const { assert!(INTERNAL_FLASH_CAPACITY <= efr32s2_hal::flash::MAX_FLASH_CAPACITY) };
        const { assert!(INTERNAL_FLASH_CAPACITY.is_multiple_of(efr32s2_hal::flash::FLASH_PAGE_SIZE)) };
    }

    #[test]
    fn host_route_pins_are_distinct_from_flow_control_and_bootload() {
        let pins = [
            (UART_TX_PORT, UART_TX_PIN),
            (UART_RX_PORT, UART_RX_PIN),
            (UART_CTS_PORT, UART_CTS_PIN),
            (UART_RTS_PORT, UART_RTS_PIN),
            (BOOTLOAD_PORT, BOOTLOAD_PIN),
        ];
        for (index, pin) in pins.iter().enumerate() {
            for other in &pins[index + 1..] {
                assert!(pin != other);
            }
        }
    }
}
