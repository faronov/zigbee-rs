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
//! | 32.768 kHz crystal | — | shipped CCFG `SCLK_LF_OPTION` = XOSC_LF; not running on some V1.3 units (the images fall back to RCOSC_LF at boot, `cc26x2_hal::setup::start_lf_clock`) |
//! | RF switch, +20 dBm PA path | 29 (high = PA) | Koenkk Z-Stack adapter table ("RF switch DIO_29: 20dBm PA"); the shipped image is the LAUNCHXL-CC1352P-2 build, which drives it low for receive and the standard PA |
//!
//! The RF switch line is driven by [`BoardRadio`]: low for receive and the
//! standard PA, RF core high-PA enable (RFC_GPO3) when the +20 dBm PA is
//! selected. Left floating (the first bring-up images) the switch state is
//! undefined.
//!
//! The same table lists optional UART flow control (DIO18 RTS, DIO19 CTS,
//! through an on-board DIP switch); it is not verified on the lab unit and
//! not used — the CP2102's RTS/DTR also drive RESET and the backdoor. No LED
//! or button is wired to the CC2652P that this crate can confirm, so none is
//! exposed. The product owns the memory
//! layout and persistence; this crate exposes fitted resources only.

#![no_std]

#[cfg(all(feature = "zbdongle-p", feature = "cc2652p7-dongle"))]
compile_error!("cc26x2-dongle: enable exactly one board variant");
#[cfg(not(any(feature = "zbdongle-p", feature = "cc2652p7-dongle")))]
compile_error!("cc26x2-dongle: enable one board variant (zbdongle-p or cc2652p7-dongle)");

pub use cc26x2_hal;
use cc26x2_hal::ccfg::{Backdoor, Ccfg, LfClock};
use cc26x2_hal::gpio::RfControlPin;
use cc26x2_hal::rf::RfSwitch;
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

/// RF front-end switch: line selecting the +20 dBm PA output. On the
/// ZBDongle-P only DIO29 is wired (Koenkk Z-Stack adapter table: "RF switch
/// DIO_29: 20dBm PA"; the dongle runs the LAUNCHXL-CC1352P-2 image, which
/// drives it low for the standard PA and receive). Left floating, the
/// switch state is undefined.
#[cfg(feature = "zbdongle-p")]
pub const RF_HIGH_PA_DIO: Option<u8> = Some(29);
/// RF front-end switch: line selecting the 2.4 GHz standard path.
#[cfg(feature = "zbdongle-p")]
pub const RF_24GHZ_DIO: Option<u8> = None;
/// Assumed RF-STAR RF-BM-2652P4I / LaunchPad wiring (DIO28 2.4 GHz,
/// DIO29 +20 dBm PA); not confirmed on hardware.
#[cfg(feature = "cc2652p7-dongle")]
pub const RF_HIGH_PA_DIO: Option<u8> = Some(29);
/// See [`RF_HIGH_PA_DIO`].
#[cfg(feature = "cc2652p7-dongle")]
pub const RF_24GHZ_DIO: Option<u8> = Some(28);
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
    /// RF core and its front-end switch (needs XOSC_HF:
    /// `cc26x2_hal::setup::start_xosc_hf`).
    pub radio: BoardRadio,
    pub rtc: cc26x2_hal::aon_rtc::RtcToken,
    pub trng: cc26x2_hal::trng::TrngToken,
    /// Raw internal flash; the product bounds it to its partitions.
    pub flash: cc26x2_hal::flash::Flash,
}

/// The RF core with this board's front-end switch lines.
pub struct BoardRadio {
    token: cc26x2_hal::rf::RadioToken,
    rf_switch: cc26x2_hal::rf::RfSwitch,
}

impl BoardRadio {
    /// Power up the radio at `tx_power_dbm` with the switch set for the
    /// selected PA.
    pub fn init(
        self,
        tx_power_dbm: i8,
    ) -> Result<cc26x2_hal::rf::Radio, cc26x2_hal::rf::RadioError> {
        self.token.init_with_switch(tx_power_dbm, self.rf_switch)
    }
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
        let mut switch_pin = |dio: Option<u8>| match dio {
            Some(n) => pins.dio(n).map(|d| Some(RfControlPin::new(d))),
            None => Some(None),
        };
        let rf_switch = RfSwitch {
            high_pa: switch_pin(RF_HIGH_PA_DIO)?,
            path_24ghz: switch_pin(RF_24GHZ_DIO)?,
        };
        Some(Self {
            host_uart: HostUartToken {
                token: uart0,
                pins: Uart0Pins {
                    tx,
                    rx: Some(rx),
                    flow: None,
                },
            },
            radio: BoardRadio {
                token: radio,
                rf_switch,
            },
            rtc,
            trng,
            flash,
        })
    }
}
