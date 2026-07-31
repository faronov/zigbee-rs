use tlsr8258_hal::gpio::Pin;
#[cfg(target_arch = "tc32")]
use tlsr8258_hal::gpio::{self, GpioError};

pub struct Led(Pin);

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
}
