//! BSP for CC2652P / CC2652P7 USB-UART Zigbee dongles.
//!
//! **`zbdongle-p`** — Sonoff ZBDongle-P (ITead "Zigbee 3.0 USB Dongle
//! Plus"): CC2652P1F (7x7 mm RGZ, 352 KiB flash, 80 KiB SRAM, +20 dBm PA)
//! behind a CP2102 USB-UART bridge. Identified on the lab unit over the ROM
//! bootloader (FCFG1 wafer 0xBB41, `USER_ID.PA` = 1, 44 x 8 KiB flash,
//! EUI-64 00:12:4B:00:20:CD:1D:7D) and from its shipped CCFG.
//!
//! **`cc2652p7-dongle`** — a CC2652P7 dongle wired the same way (UART0 on
//! DIO12/DIO13, backdoor DIO15, LAUNCHXL-CC1352P-2-style RF path). No such
//! unit has been inspected: this variant is build-only.
//!
//! Wiring:
//!
//! | signal | DIO | evidence |
//! |---|---|---|
//! | UART0 RX (from host) | 12 | ROM bootloader and Z-Stack MT answered through the CP2102 (the ROM BSL serial pins are fixed at DIO12/13) |
//! | UART0 TX (to host) | 13 | same |
//! | ROM BSL backdoor | 15, active low | shipped CCFG `BL_CONFIG` = `0xC5FE0FC5` |
//! | RESET / backdoor control | CP2102 DTR/RTS | measured: (DTR=0, RTS=1) holds RESET, (DTR=1, RTS=0) pulls DIO15 low |
//! | 32.768 kHz crystal | — | shipped CCFG `SCLK_LF_OPTION` = XOSC_LF |
//!
//! The CP2102's RTS/DTR drive RESET and the backdoor, so there is no UART
//! hardware flow control. No LED or button is wired to the CC2652P that
//! this crate can confirm, so none is exposed. The product owns the memory
//! layout and persistence; this crate exposes fitted resources only.

#![no_std]

#[cfg(all(feature = "zbdongle-p", feature = "cc2652p7-dongle"))]
compile_error!("cc26x2-dongle: enable exactly one board variant");
#[cfg(not(any(feature = "zbdongle-p", feature = "cc2652p7-dongle")))]
compile_error!("cc26x2-dongle: enable one board variant (zbdongle-p or cc2652p7-dongle)");

pub use cc26x2_hal;
use cc26x2_hal::ccfg::{Backdoor, Ccfg, LfClock};
use cc26x2_hal::uart::{Uart0, Uart0Pins, UartError};
use core::sync::atomic::{AtomicBool, Ordering};

#[cfg(feature = "zbdongle-p")]
pub const BOARD_NAME: &str = "Sonoff ZBDongle-P";
#[cfg(feature = "cc2652p7-dongle")]
pub const BOARD_NAME: &str = "CC2652P7 USB dongle";

pub const MCU_PART: &str = cc26x2_hal::chip::PART;

pub const UART_RX_DIO: u8 = 12;
pub const UART_TX_DIO: u8 = 13;
/// ROM bootloader backdoor (active low).
pub const BSL_BACKDOOR_DIO: u8 = 15;
/// Default host/log rate: the rate the shipped Z-Stack and the ROM
/// bootloader tooling use on this dongle.
pub const UART_BAUD: u32 = 115_200;

/// The board's physical CCFG options: 32.768 kHz crystal and the serial
/// ROM bootloader on DIO15 active low. Identical to the shipped CCFG of the
/// inspected ZBDongle-P (`cc26x2_hal::ccfg` test).
pub const CCFG: Ccfg = Ccfg::ti_default()
    .lf_clock(LfClock::Crystal)
    .rom_bootloader(Backdoor {
        pin: BSL_BACKDOOR_DIO,
        active_high: false,
    });

const _: () = assert!(CCFG.check_recoverable().is_ok());

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Complete singleton resource set.
pub struct BoardResources {
    pub host_uart: HostUartToken,
    /// RF core (needs XOSC_HF: `cc26x2_hal::setup::start_xosc_hf`).
    pub radio: cc26x2_hal::rf::RadioToken,
    pub rtc: cc26x2_hal::aon_rtc::RtcToken,
    pub trng: cc26x2_hal::trng::TrngToken,
    /// Raw internal flash; the product bounds it to its partitions.
    pub flash: cc26x2_hal::flash::Flash,
}

/// UART0 on DIO12/DIO13 (unopened).
pub struct HostUartToken {
    token: cc26x2_hal::uart::Uart0Token,
    pins: Uart0Pins,
}

impl HostUartToken {
    /// Open 8N1 at `baud` without flow control.
    pub fn open(self, baud: u32) -> Result<Uart0, UartError> {
        Uart0::new(self.token, self.pins, baud)
    }
}

impl BoardResources {
    /// Acquire all board resources once.
    pub fn take() -> Option<Self> {
        if TAKEN.swap(true, Ordering::AcqRel) {
            return None;
        }
        let cc26x2_hal::Peripherals {
            mut pins,
            uart0,
            rtc,
            trng,
            flash,
            radio,
        } = cc26x2_hal::Peripherals::take()?;
        let tx = pins.dio(UART_TX_DIO)?;
        let rx = pins.dio(UART_RX_DIO)?;
        Some(Self {
            host_uart: HostUartToken {
                token: uart0,
                pins: Uart0Pins {
                    tx,
                    rx: Some(rx),
                    flow: None,
                },
            },
            radio,
            rtc,
            trng,
            flash,
        })
    }
}
