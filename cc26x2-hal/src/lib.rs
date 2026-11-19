//! Pure-Rust chip services, peripherals and IEEE 802.15.4 RF core driver for
//! the TI CC26x2/CC13x2 family (Cortex-M4F): CC2652R/RB/P and CC1352P (rev E,
//! `cc13x2_cc26x2`), CC2652R7/P7 and CC1352P7 (`cc13x2x7_cc26x2x7`).
//!
//! Exactly one chip feature must be enabled. Register addresses and field
//! layouts follow the SimpleLink Low Power F2 SDK 8.33.00.16 device headers
//! (`source/ti/devices/<family>/inc`), and the start-up sequence mirrors its
//! driverlib `setup.c`. Factory trims and flash programming call the chip's
//! boot ROM (driverlib ROM table and HAPI), exactly as TI's own start-up does;
//! no TI driver, RTOS or radio library is linked. The only vendored TI code is
//! the IEEE 802.15.4 CPE patch image (BSD-3-Clause, `vendor/ti`).
//!
//! Module map:
//!
//! | module | role |
//! |---|---|
//! | [`chip`] | compile-time variant, FCFG1 identity and the runtime family check |
//! | [`ccfg`] | the Customer Configuration (CCFG) builder the product links at the end of flash |
//! | [`setup`] | `SetupTrimDevice`, XOSC_HF switch and LF clock start |
//! | [`prcm`] | power domains and peripheral clock gates |
//! | [`gpio`] | IOC pin muxing and push-pull outputs |
//! | [`uart`] | UART0 (blocking TX, interrupt RX ring) |
//! | [`aon_rtc`] / `time_driver` | AON RTC and the Embassy time driver |
//! | [`trng`] | TRNG entropy |
//! | [`flash`] | internal flash through the ROM HAPI |
//! | [`rf`] | RF core doorbell, IEEE 802.15.4 commands, PA tables |

#![no_std]
// MMIO helpers are only reachable on the target; host builds compile the
// pure logic for the unit tests.
#![cfg_attr(not(all(target_arch = "arm", target_os = "none")), allow(dead_code))]

#[cfg(all(feature = "family-x2", feature = "family-x2x7"))]
compile_error!("cc26x2-hal: enable exactly one chip feature");
#[cfg(not(any(feature = "family-x2", feature = "family-x2x7")))]
compile_error!(
    "cc26x2-hal: enable one chip feature (cc2652r, cc2652rb, cc2652p, cc1352p, cc2652r7, cc2652p7, cc1352p7)"
);

#[cfg(feature = "cc2652rb")]
compile_error!(
    "cc26x2-hal: CC2652RB (crystal-less, HPOSC/BAW) needs HPOSC frequency compensation, which is not implemented"
);

pub mod aon_rtc;
pub mod ccfg;
pub mod chip;
pub mod flash;
pub mod gpio;
mod mmio;
pub mod prcm;
pub mod rf;
pub mod setup;
#[cfg(feature = "time-driver")]
pub mod time_driver;
pub mod trng;
pub mod uart;
pub mod vectors;

use core::sync::atomic::{AtomicBool, Ordering};

/// CPU clock after reset (RCOSC_HF) and after the XOSC_HF switch.
pub const MCU_CLOCK_HZ: u32 = 48_000_000;

/// Chip resources, each owned exactly once.
pub struct Peripherals {
    pub pins: gpio::Pins,
    pub uart0: uart::Uart0Token,
    pub rtc: aon_rtc::RtcToken,
    pub trng: trng::TrngToken,
    pub flash: flash::Flash,
    pub radio: rf::RadioToken,
}

static TAKEN: AtomicBool = AtomicBool::new(false);

impl Peripherals {
    /// Take the chip resources once. Returns `None` on a second call.
    pub fn take() -> Option<Self> {
        if TAKEN.swap(true, Ordering::AcqRel) {
            return None;
        }
        Some(Self {
            pins: gpio::Pins::new(),
            uart0: uart::Uart0Token(()),
            rtc: aon_rtc::RtcToken(()),
            trng: trng::TrngToken(()),
            flash: flash::Flash::new(),
            radio: rf::RadioToken(()),
        })
    }
}
