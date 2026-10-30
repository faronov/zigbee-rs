//! Exclusively owned resources mapped to the PB-03F-Kit wiring.

use crate::InternalFlash;
use crate::pins;
use phy6222_hal::Peripherals;
use phy6222_hal::adc::{Adc, AdcError, Channel, Input};
use phy6222_hal::flash::FlashError;
use phy6222_hal::gpio::{self, Pull};
use phy6222_hal::peripherals::{AdcToken, Uart0Token};
use phy6222_hal::uart::{Config as UartConfig, Uart0};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardError {
    AlreadyTaken,
    Flash(FlashError),
}

pub struct Resources {
    pub flash: InternalFlash,
    pub leds: Leds,
    pub user_button: UserButton,
    pub console: Console,
    pub supply_monitor: SupplyMonitorToken,
}

impl Resources {
    /// Take every fitted resource. All LEDs start off.
    ///
    /// Call after [`crate::time::init`] so HCLK-derived rates are final.
    pub fn take() -> Result<Self, BoardError> {
        let Peripherals {
            adc,
            flash,
            i2c0: _,
            i2c1: _,
            uart0,
        } = Peripherals::take().ok_or(BoardError::AlreadyTaken)?;

        let mut leds = Leds {
            red: Led::new(pins::LED_RED),
            green: Led::new(pins::LED_GREEN),
            blue: Led::new(pins::LED_BLUE),
            cool_white: Led::new(pins::LED_COOL_WHITE),
            warm_white: Led::new(pins::LED_WARM_WHITE),
        };
        leds.all_off();

        gpio::clear_fmux(pins::USER_BUTTON);
        gpio::set_input(pins::USER_BUTTON);
        gpio::set_pull(pins::USER_BUTTON, Pull::StrongPullUp);

        gpio::clear_fmux(pins::SUPPLY_SENSE);
        gpio::set_input(pins::SUPPLY_SENSE);
        gpio::set_pull(pins::SUPPLY_SENSE, Pull::PullDown);

        Ok(Self {
            flash: InternalFlash::new(flash).map_err(BoardError::Flash)?,
            leds,
            user_button: UserButton { _private: () },
            console: Console { uart: uart0 },
            supply_monitor: SupplyMonitorToken { adc },
        })
    }
}

/// One kit LED driven directly by a GPIO.
///
/// Polarity is [`LED_ACTIVE_HIGH`]; it has not been confirmed visually on
/// hardware yet.
pub struct Led {
    pin: u8,
}

/// Kit LED drive polarity.
pub const LED_ACTIVE_HIGH: bool = true;

impl Led {
    fn new(pin: u8) -> Self {
        gpio::clear_fmux(pin);
        gpio::set_output(pin);
        Self { pin }
    }

    pub fn set_on(&mut self, on: bool) {
        gpio::write(self.pin, on == LED_ACTIVE_HIGH);
    }
}

/// The RGB LED plus the cool- and warm-white LEDs.
pub struct Leds {
    pub red: Led,
    pub green: Led,
    pub blue: Led,
    pub cool_white: Led,
    pub warm_white: Led,
}

impl Leds {
    pub fn all_off(&mut self) {
        self.red.set_on(false);
        self.green.set_on(false);
        self.blue.set_on(false);
        self.cool_white.set_on(false);
        self.warm_white.set_on(false);
    }
}

/// Active-low user button on P15 (strong pull-up enabled).
pub struct UserButton {
    _private: (),
}

impl UserButton {
    pub fn is_pressed(&self) -> bool {
        !gpio::read(pins::USER_BUTTON)
    }
}

/// UART0 routed to the on-board CH340 (P9 TX, P10 RX), 115200 8N1.
pub struct Console {
    uart: Uart0Token,
}

impl Console {
    pub fn into_uart(self) -> Uart0 {
        Uart0::new(
            self.uart,
            UartConfig {
                tx_pin: pins::UART0_TX,
                rx_pin: Some(pins::UART0_RX),
                baud: pins::CONSOLE_BAUD,
            },
        )
    }
}

/// Exclusive token for supply measurement on P14 (ADC channel P14).
pub struct SupplyMonitorToken {
    adc: AdcToken,
}

impl SupplyMonitorToken {
    pub fn into_monitor(self) -> SupplyMonitor {
        SupplyMonitor {
            adc: Adc::new(self.adc),
        }
    }
}

const _: () = assert!(Channel::P14.gpio() == pins::SUPPLY_SENSE);

/// Module supply voltage: P14 is driven high while the ADC samples it.
pub struct SupplyMonitor {
    adc: Adc,
}

impl SupplyMonitor {
    pub fn read_millivolts(&mut self) -> Result<u32, AdcError> {
        self.adc.read_mv(Channel::P14, Input::DrivenHigh)
    }
}
