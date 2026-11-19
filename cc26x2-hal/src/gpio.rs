//! DIO pads: IOC muxing (`inc/hw_ioc.h`) and GPIO outputs (`inc/hw_gpio.h`).
//!
//! The GPIO module lives in the PERIPH power domain; [`Output`] powers it on
//! the first time an output is created.

use core::convert::Infallible;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::mmio::{self, GPIO_BASE, IOC_BASE};
use crate::prcm::{self, Domain, Gate};

/// Number of DIO pads on the 7x7 mm RGZ package.
pub const DIO_COUNT: u8 = 31;

/// IOC port IDs (`driverlib/ioc.h`).
pub(crate) const PORT_GPIO: u32 = 0x00;
pub(crate) const PORT_UART0_RX: u32 = 0x0F;
pub(crate) const PORT_UART0_TX: u32 = 0x10;
pub(crate) const PORT_UART0_CTS: u32 = 0x11;
pub(crate) const PORT_UART0_RTS: u32 = 0x12;
/// RF core data out 0: LNA enable (receiving).
pub(crate) const PORT_RFC_GPO0: u32 = 0x2F;
/// RF core data out 3: high-power PA enable (transmitting). TI routes the
/// high-PA switch to GPO3 instead of GPO1 (PA enable is sometimes not
/// de-asserted on GPO1).
pub(crate) const PORT_RFC_GPO3: u32 = 0x32;

const IOCFG_IE: u32 = 1 << 29;
const PULL_DOWN: u32 = 1 << 13;
const PULL_UP: u32 = 2 << 13;
const NO_PULL: u32 = 3 << 13;

const DOUTSET: u32 = GPIO_BASE + 0x90;
const DOUTCLR: u32 = GPIO_BASE + 0xA0;
const DOUTTGL: u32 = GPIO_BASE + 0xB0;
const DIN: u32 = GPIO_BASE + 0xC0;
const DOE: u32 = GPIO_BASE + 0xD0;

static CLAIMED: AtomicU32 = AtomicU32::new(0);

/// Input pull selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pull {
    None,
    Up,
    Down,
}

impl Pull {
    const fn bits(self) -> u32 {
        match self {
            Pull::None => NO_PULL,
            Pull::Up => PULL_UP,
            Pull::Down => PULL_DOWN,
        }
    }
}

/// Ownership of every DIO pad; boards claim the pads they wire.
pub struct Pins(());

impl Pins {
    pub(crate) const fn new() -> Self {
        Self(())
    }

    /// Claim DIO `n` once. `None` if out of range or already claimed.
    pub fn dio(&mut self, n: u8) -> Option<Dio> {
        if n >= DIO_COUNT {
            return None;
        }
        let bit = 1 << n;
        if CLAIMED.fetch_or(bit, Ordering::AcqRel) & bit != 0 {
            return None;
        }
        Some(Dio(n))
    }
}

/// One claimed DIO pad.
#[derive(Debug, PartialEq, Eq)]
pub struct Dio(u8);

impl Dio {
    pub const fn number(&self) -> u8 {
        self.0
    }

    pub(crate) fn configure(&self, port: u32, input: bool, pull: Pull) {
        let cfg = port | pull.bits() | if input { IOCFG_IE } else { 0 };
        mmio::write(IOC_BASE + 4 * u32::from(self.0), cfg);
    }

    /// Push-pull output, initially `high`.
    pub fn into_output(self, high: bool) -> Output {
        ensure_gpio();
        let bit = 1u32 << self.0;
        mmio::write(if high { DOUTSET } else { DOUTCLR }, bit);
        self.configure(PORT_GPIO, false, Pull::None);
        mmio::modify(DOE, 0, bit);
        Output(self)
    }

    /// Digital input with `pull`.
    pub fn into_input(self, pull: Pull) -> Input {
        ensure_gpio();
        mmio::modify(DOE, 1 << self.0, 0);
        self.configure(PORT_GPIO, true, pull);
        Input(self)
    }
}

/// A pad that an RF front-end switch control line is wired to: driven as a
/// GPIO level or handed to an RF core output.
#[derive(Debug, PartialEq, Eq)]
pub struct RfControlPin(Dio);

impl RfControlPin {
    pub fn new(dio: Dio) -> Self {
        ensure_gpio();
        let pin = Self(dio);
        pin.drive(false);
        pin
    }

    /// GPIO output at `high`.
    pub(crate) fn drive(&self, high: bool) {
        let bit = 1u32 << (self.0).0;
        mmio::write(if high { DOUTSET } else { DOUTCLR }, bit);
        self.0.configure(PORT_GPIO, false, Pull::None);
        mmio::modify(DOE, 0, bit);
    }

    /// Output follows RF core signal `port` (`PORT_RFC_GPO*`).
    pub(crate) fn follow_rfc(&self, port: u32) {
        self.0.configure(port, false, Pull::None);
    }

    pub const fn number(&self) -> u8 {
        (self.0).0
    }
}

fn ensure_gpio() {
    if !prcm::is_on(Domain::Periph) {
        prcm::power_on(Domain::Periph);
    }
    prcm::enable(Gate::Gpio);
    prcm::load();
}

/// GPIO push-pull output.
pub struct Output(Dio);

impl Output {
    pub fn set(&mut self, high: bool) {
        mmio::write(if high { DOUTSET } else { DOUTCLR }, 1 << (self.0).0);
    }

    pub fn toggle(&mut self) {
        mmio::write(DOUTTGL, 1 << (self.0).0);
    }
}

impl embedded_hal::digital::ErrorType for Output {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for Output {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.set(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Infallible> {
        self.set(true);
        Ok(())
    }
}

/// GPIO input.
pub struct Input(Dio);

impl Input {
    pub fn is_high(&self) -> bool {
        mmio::read(DIN) & (1 << (self.0).0) != 0
    }
}

impl embedded_hal::digital::ErrorType for Input {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Input {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(Input::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(!Input::is_high(self))
    }
}
