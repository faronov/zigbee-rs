//! CC2340R52 GPIO, IOC pin configuration and exclusive DIO ownership.
//!
//! Every DIO is claimed at most once through [`Pins`] (part of
//! [`crate::Core`]); typed [`Led`]/[`Button`] resources and the peripheral
//! drivers ([`crate::i2c`], [`crate::adc`], [`crate::lgpt`]) are built from
//! claimed pins, so one pad cannot be driven by two owners. Which DIO is
//! wired to what is board knowledge and lives in the board crate.

use core::marker::PhantomData;

/// Number of DIO pads on the CC2340R52 (DIO0..=DIO25, RGE package).
pub const DIO_COUNT: u8 = 26;
/// SWDIO (DIO16) and SWCLK (DIO17) stay reserved for the debug port.
pub const SWD_DIOS: [u8; 2] = [16, 17];

pub(crate) const GPIO_BASE: u32 = 0x4002_3000;
pub(crate) const GPIO_DOUTSET: u32 = GPIO_BASE + 0x210;
pub(crate) const GPIO_DOUTCLR: u32 = GPIO_BASE + 0x220;
pub(crate) const GPIO_DOESET: u32 = GPIO_BASE + 0x510;
pub(crate) const GPIO_DOECLR: u32 = GPIO_BASE + 0x520;
#[cfg(target_os = "none")]
const GPIO_DIN: u32 = GPIO_BASE + 0x700;
#[cfg(target_os = "none")]
const GPIO_MIS: u32 = GPIO_BASE + 0x54;
#[cfg(target_os = "none")]
const GPIO_ICLR: u32 = GPIO_BASE + 0x64;
#[cfg(target_os = "none")]
const GPIO_IMSET: u32 = GPIO_BASE + 0x6C;

pub(crate) const IOC_BASE: u32 = 0x4000_3000;
pub(crate) const IOC_DIO0: u32 = IOC_BASE + 0x100;

/// `IOC_IOCn_HYSTEN`.
pub const IOC_HYSTERESIS_ENABLE: u32 = 0x4000_0000;
/// `IOC_IOCn_INPEN`.
pub const IOC_INPUT_ENABLE: u32 = 0x2000_0000;
/// `IOC_IOCn_IOMODE_OPEND` (open drain).
pub const IOC_OPEN_DRAIN: u32 = 0x0200_0000;
/// `IOC_IOCn_WUENSB`.
pub const IOC_STANDBY_WAKE_ENABLE: u32 = 0x0004_0000;
/// `IOC_IOCn_PULLCTL_PULL_UP`.
pub const IOC_PULL_UP: u32 = 0x0000_4000;
/// `IOC_IOCn_PULLCTL_PULL_DOWN`.
pub const IOC_PULL_DOWN: u32 = 0x0000_2000;
/// `IOC_IOCn_WUCFGSD_WAKE_LOW`: wake from SHUTDOWN while the pad is low.
pub const IOC_SHUTDOWN_WAKE_LOW: u32 = 0x0020_0000;
/// `IOC.IOCn.EDGEDET = EDGE_NEG`: latch falling edges into `GPIO.RIS`.
pub const IOC_EDGE_NEGATIVE: u32 = 0x0001_0000;
/// `IOC_IOCn_PORTCFG_M`.
pub const IOC_PORTCFG_MASK: u32 = 0x0000_0007;

/// `IOC.IOCn.PORTCFG` pin function (`GPIO_MUX_PORTCFG_*`). The valid
/// function number for a peripheral signal on a given DIO comes from the
/// device pin-mux table (SysConfig); the board crate supplies it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PinFunction {
    Gpio = 0,
    Pfunc1 = 1,
    Pfunc2 = 2,
    Pfunc3 = 3,
    Pfunc4 = 4,
    Pfunc5 = 5,
    Analog = 6,
}

impl PinFunction {
    pub const fn portcfg(self) -> u32 {
        self as u32
    }
}

/// Why a DIO could not be claimed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinError {
    /// The DIO does not exist on the CC2340R52.
    Invalid(u8),
    /// The DIO is reserved for SWD.
    Reserved(u8),
    /// The DIO was already claimed by another owner.
    AlreadyClaimed(u8),
}

/// Exclusive ownership of the DIO pads. Obtained once from
/// [`crate::take_core`].
pub struct Pins {
    claimed: u32,
}

impl Pins {
    pub(crate) const fn new() -> Self {
        Self { claimed: 0 }
    }

    /// Claim one DIO for a peripheral driver.
    pub fn claim(&mut self, dio: u8) -> Result<Dio, PinError> {
        if dio >= DIO_COUNT {
            return Err(PinError::Invalid(dio));
        }
        if SWD_DIOS.contains(&dio) {
            return Err(PinError::Reserved(dio));
        }
        let mask = 1u32 << dio;
        if self.claimed & mask != 0 {
            return Err(PinError::AlreadyClaimed(dio));
        }
        self.claimed |= mask;
        Ok(Dio { number: dio })
    }

    /// Bit mask of every claimed DIO.
    pub const fn claimed_mask(&self) -> u32 {
        self.claimed
    }

    /// Claim `DIO` and configure it as an active-high LED output.
    #[cfg(target_os = "none")]
    pub fn led<const DIO: u8>(&mut self, initially_on: bool) -> Result<Led<DIO>, PinError> {
        self.claim(DIO)?;
        Ok(Led::configure(initially_on))
    }

    /// Claim `DIO` and configure it as an active-low button with pull-up,
    /// falling-edge detection and standby wake.
    #[cfg(target_os = "none")]
    pub fn button<const DIO: u8>(&mut self) -> Result<Button<DIO>, PinError> {
        self.claim(DIO)?;
        Ok(Button::configure())
    }
}

/// A claimed DIO, consumed by a peripheral driver.
#[derive(Debug, PartialEq, Eq)]
pub struct Dio {
    number: u8,
}

impl Dio {
    pub const fn number(&self) -> u8 {
        self.number
    }

    pub(crate) const fn mask(&self) -> u32 {
        1u32 << self.number
    }

    /// Write this pad's IOC configuration.
    #[cfg(target_os = "none")]
    pub(crate) fn set_ioc(&self, configuration: u32) {
        write_ioc(self.number, configuration);
    }

    /// Drive the output latch.
    #[cfg(target_os = "none")]
    pub(crate) fn set_output(&self, high: bool) {
        write_output(self.number, high);
    }

    /// Enable or disable the GPIO output driver.
    #[cfg(target_os = "none")]
    pub(crate) fn set_output_enable(&self, enable: bool) {
        write_register(if enable { GPIO_DOESET } else { GPIO_DOECLR }, self.mask());
    }
}

/// Explicit IOC value for an ordinary push-pull GPIO output.
///
/// TI's LPF3 GPIO output configuration enables the input buffer so software
/// can read the driven level. Pulls, edge detection, inversion, and alternate
/// peripheral functions remain disabled.
pub const LED_IOC_CONFIGURATION: u32 = IOC_INPUT_ENABLE;

/// Explicit IOC value for an active-low button with an internal pull-up.
///
/// Falling-edge detection feeds the `GPIO_COMB` interrupt (press), and
/// `WUENSB` lets the same pin wake the device from standby.
pub const BUTTON_IOC_CONFIGURATION: u32 = IOC_HYSTERESIS_ENABLE
    | IOC_INPUT_ENABLE
    | IOC_STANDBY_WAKE_ENABLE
    | IOC_PULL_UP
    | IOC_EDGE_NEGATIVE;

pub struct Output;
pub struct InputPullUp;

/// Active-high LED output. Built only through [`Pins::led`], so a DIO cannot
/// be configured simultaneously in two modes.
pub struct Led<const DIO: u8> {
    _mode: PhantomData<Output>,
}

impl<const DIO: u8> Led<DIO> {
    #[cfg(target_os = "none")]
    pub(crate) fn configure(initially_on: bool) -> Self {
        assert_valid_dio(DIO);
        write_output(DIO, initially_on);
        write_ioc(DIO, LED_IOC_CONFIGURATION);
        write_register(GPIO_DOESET, bit(DIO));
        Self { _mode: PhantomData }
    }

    #[cfg(target_os = "none")]
    pub fn on(&mut self) {
        write_register(GPIO_DOUTSET, bit(DIO));
    }

    #[cfg(target_os = "none")]
    pub fn off(&mut self) {
        write_register(GPIO_DOUTCLR, bit(DIO));
    }

    #[cfg(target_os = "none")]
    pub fn set(&mut self, on: bool) {
        if on {
            self.on();
        } else {
            self.off();
        }
    }

    #[cfg(target_os = "none")]
    pub fn is_on(&self) -> bool {
        read_input(DIO)
    }
}

/// Active-low button with an internal pull-up. Built only through
/// [`Pins::button`].
pub struct Button<const DIO: u8> {
    _mode: PhantomData<InputPullUp>,
}

impl<const DIO: u8> Button<DIO> {
    #[cfg(target_os = "none")]
    pub(crate) fn configure() -> Self {
        assert_valid_dio(DIO);
        write_register(GPIO_DOECLR, bit(DIO));
        write_ioc(DIO, BUTTON_IOC_CONFIGURATION);
        Self { _mode: PhantomData }
    }

    #[cfg(target_os = "none")]
    pub fn is_pressed(&self) -> bool {
        !read_input(DIO)
    }

    /// Bit of this button in the masks returned by [`take_button_edges`]
    /// and [`wait_for_button_edge`].
    pub const fn edge_mask(&self) -> u32 {
        1u32 << DIO
    }

    /// Let a press wake the chip from SHUTDOWN
    /// ([`crate::power::shutdown`]); the wake-up is a full reset.
    #[cfg(target_os = "none")]
    pub fn enable_shutdown_wake(&mut self) {
        write_ioc(DIO, BUTTON_IOC_CONFIGURATION | IOC_SHUTDOWN_WAKE_LOW);
    }

    /// Route this button's press (falling edge) to the `GPIO_COMB` IRQ.
    ///
    /// Mirrors TI `GPIO_enableInt()`: stale edges cached in `RIS` are
    /// cleared before the mask is set.
    #[cfg(all(target_os = "none", feature = "runtime"))]
    pub fn enable_press_interrupt(&mut self) {
        cortex_m::interrupt::free(|_| {
            write_register(GPIO_ICLR, bit(DIO));
            write_register(GPIO_IMSET, bit(DIO));
        });
        crate::interrupts::enable(crate::interrupts::Irq::GpioComb);
    }
}

#[cfg(all(target_os = "none", feature = "runtime"))]
static BUTTON_EDGES: portable_atomic::AtomicU32 = portable_atomic::AtomicU32::new(0);
#[cfg(all(target_os = "none", feature = "runtime"))]
static BUTTON_WAKER: cortex_m::interrupt::Mutex<core::cell::RefCell<Option<core::task::Waker>>> =
    cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(None));

/// `GPIO_COMB` handler (TI `GPIO_hwiIntFxn()`): latch and clear the masked
/// edges, then wake the waiting task.
#[cfg(all(target_os = "none", feature = "runtime"))]
pub(crate) fn on_gpio_interrupt() {
    let events = unsafe { core::ptr::read_volatile(GPIO_MIS as *const u32) };
    write_register(GPIO_ICLR, events);
    if events != 0 {
        BUTTON_EDGES.fetch_or(events, portable_atomic::Ordering::AcqRel);
        cortex_m::interrupt::free(|cs| {
            if let Some(waker) = BUTTON_WAKER.borrow(cs).borrow_mut().take() {
                waker.wake();
            }
        });
    }
}

/// Take (and clear) the press edges latched since the last call.
#[cfg(all(target_os = "none", feature = "runtime"))]
pub fn take_button_edges() -> u32 {
    BUTTON_EDGES.swap(0, portable_atomic::Ordering::AcqRel)
}

/// Wait for at least one press edge and return the latched edge mask.
///
/// Uses the interrupt only; the CPU may sleep for the whole wait.
#[cfg(all(target_os = "none", feature = "runtime"))]
pub async fn wait_for_button_edge() -> u32 {
    core::future::poll_fn(|cx| {
        let edges = take_button_edges();
        if edges != 0 {
            return core::task::Poll::Ready(edges);
        }
        cortex_m::interrupt::free(|cs| {
            *BUTTON_WAKER.borrow(cs).borrow_mut() = Some(cx.waker().clone());
        });
        // Close the edge-between-check-and-registration race.
        let edges = take_button_edges();
        if edges != 0 {
            core::task::Poll::Ready(edges)
        } else {
            core::task::Poll::Pending
        }
    })
    .await
}

#[cfg(target_os = "none")]
const fn bit(dio: u8) -> u32 {
    1u32 << dio
}

#[cfg(target_os = "none")]
fn assert_valid_dio(dio: u8) {
    assert!(dio < DIO_COUNT);
}

#[cfg(target_os = "none")]
fn write_ioc(dio: u8, configuration: u32) {
    write_register(IOC_DIO0 + u32::from(dio) * 4, configuration);
}

#[cfg(target_os = "none")]
fn write_output(dio: u8, high: bool) {
    write_register(if high { GPIO_DOUTSET } else { GPIO_DOUTCLR }, bit(dio));
}

#[cfg(target_os = "none")]
fn read_input(dio: u8) -> bool {
    let value = unsafe { core::ptr::read_volatile(GPIO_DIN as *const u32) };
    value & bit(dio) != 0
}

#[cfg(target_os = "none")]
pub(crate) fn write_register(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pins_are_claimed_exactly_once() {
        let mut pins = Pins::new();
        let sda = pins.claim(8).unwrap();
        assert_eq!(sda.number(), 8);
        assert_eq!(pins.claim(8), Err(PinError::AlreadyClaimed(8)));
        assert_eq!(pins.claim(DIO_COUNT), Err(PinError::Invalid(DIO_COUNT)));
        assert_eq!(pins.claim(16), Err(PinError::Reserved(16)));
        assert_eq!(pins.claim(17), Err(PinError::Reserved(17)));
        pins.claim(25).unwrap();
        assert_eq!(pins.claimed_mask(), (1 << 8) | (1 << 25));
    }

    #[test]
    fn register_addresses_match_hw_memmap() {
        assert_eq!(IOC_DIO0, 0x4000_3100);
        assert_eq!(GPIO_DOUTSET, 0x4002_3210);
        assert_eq!(GPIO_DOECLR, 0x4002_3520);
    }

    #[test]
    fn pin_functions_match_gpio_mux_portcfg() {
        assert_eq!(PinFunction::Gpio.portcfg(), 0);
        assert_eq!(PinFunction::Pfunc2.portcfg(), 2);
        assert_eq!(PinFunction::Pfunc4.portcfg(), 4);
        assert_eq!(PinFunction::Analog.portcfg(), 6);
    }

    #[test]
    fn led_and_button_ioc_modes_are_explicit() {
        assert_eq!(LED_IOC_CONFIGURATION & IOC_PORTCFG_MASK, 0);
        assert_eq!(LED_IOC_CONFIGURATION & IOC_INPUT_ENABLE, IOC_INPUT_ENABLE);
        assert_eq!(
            BUTTON_IOC_CONFIGURATION,
            IOC_HYSTERESIS_ENABLE
                | IOC_INPUT_ENABLE
                | IOC_STANDBY_WAKE_ENABLE
                | IOC_PULL_UP
                | IOC_EDGE_NEGATIVE
        );
        // EDGEDET = EDGE_NEG (hw_ioc.h IOC_IOC13_EDGEDET_EDGE_NEG).
        assert_eq!(BUTTON_IOC_CONFIGURATION & 0x0003_0000, 0x0001_0000);
        assert_eq!(BUTTON_IOC_CONFIGURATION & IOC_PORTCFG_MASK, 0);
    }
}
