use tlsr8258_hal::gpio::Pin;
#[cfg(target_arch = "tc32")]
use tlsr8258_hal::gpio::{self, GpioError, Port};

pub struct Led(Pin);

/// Logical output levels for the fitted TB-04 RGB status LED.
///
/// This intentionally contains no GPIO identity. The board owns the physical
/// LED routing, including the reset-on-wake recovery path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusLedState {
    red: bool,
    green: bool,
    blue: bool,
}

impl StatusLedState {
    pub const fn new(red: bool, green: bool, blue: bool) -> Self {
        Self { red, green, blue }
    }
}

impl Led {
    pub(crate) const fn new(pin: Pin) -> Self {
        Self(pin)
    }

    #[cfg(target_arch = "tc32")]
    fn configure_output(&self, high: bool) -> Result<(), GpioError> {
        gpio::set_function_gpio(&self.0);
        gpio::write(&self.0, high);
        gpio::set_output_enable(&self.0, true);
        gpio::set_input_enable(&self.0, false)
    }

    #[cfg(target_arch = "tc32")]
    pub fn write(&self, high: bool) {
        gpio::write(&self.0, high);
    }
}

/// Exclusively-owned view of the three fitted TB-04 status LEDs.
pub struct StatusLeds {
    pub red: Led,
    pub green: Led,
    pub blue: Led,
}

impl StatusLeds {
    pub(crate) const fn new(red: Pin, green: Pin, blue: Pin) -> Self {
        Self {
            red: Led::new(red),
            green: Led::new(green),
            blue: Led::new(blue),
        }
    }

    #[cfg(target_arch = "tc32")]
    pub fn init(&self) -> Result<(), GpioError> {
        self.red.configure_output(true)?;
        self.green.configure_output(false)?;
        self.blue.configure_output(false)
    }

    /// Restore the fitted LED outputs after a reset-on-wake transition.
    ///
    /// # Safety
    ///
    /// The caller must ensure that reset has discarded every live
    /// [`StatusLeds`] owner. This operation reconstructs temporary owners for
    /// the TB-04's PC1 (red), PB5 (green), and PC4 (blue) LED pads.
    #[cfg(target_arch = "tc32")]
    pub unsafe fn restore_after_reset(state: StatusLedState) -> Result<(), GpioError> {
        fn restore(pin: Pin, high: bool) -> Result<(), GpioError> {
            gpio::set_function_gpio(&pin);
            gpio::write(&pin, high);
            gpio::set_output_enable(&pin, true);
            gpio::set_input_enable(&pin, false)
        }

        // SAFETY: The caller guarantees reset discarded every live owner;
        // these temporary views are used only to restore the fitted LEDs.
        let red = unsafe { Pin::steal(Port::C, 1) };
        // SAFETY: See the safety contract above; PB5 is distinct from PC1.
        let green = unsafe { Pin::steal(Port::B, 5) };
        // SAFETY: See the safety contract above; PC4 is distinct from PC1/PB5.
        let blue = unsafe { Pin::steal(Port::C, 4) };

        restore(red, state.red)?;
        restore(green, state.green)?;
        restore(blue, state.blue)
    }
}

#[cfg(test)]
mod tests {
    use super::StatusLedState;

    #[test]
    fn status_led_state_keeps_each_semantic_channel() {
        let state = StatusLedState::new(true, false, true);
        assert!(state.red);
        assert!(!state.green);
        assert!(state.blue);
    }
}
