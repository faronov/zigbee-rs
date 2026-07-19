use tlsr8258_hal::gpio::{self, GpioError, Pin, Port};

#[derive(Clone, Copy)]
pub struct Led(Pin);

impl Led {
    const fn new(port: Port, bit: u8) -> Self {
        Self(Pin::new(port, bit))
    }

    fn configure_output(self, high: bool) -> Result<(), GpioError> {
        gpio::set_function_gpio(self.0);
        gpio::write(self.0, high);
        gpio::set_output_enable(self.0, true);
        gpio::set_input_enable(self.0, false)
    }

    pub fn write(self, high: bool) {
        gpio::write(self.0, high);
    }
}

pub const LED_RED: Led = Led::new(Port::C, 1);
pub const LED_GREEN: Led = Led::new(Port::B, 5);
pub const LED_BLUE: Led = Led::new(Port::C, 4);

pub fn configure_status_leds() -> Result<(), GpioError> {
    LED_RED.configure_output(true)?;
    LED_GREEN.configure_output(false)?;
    LED_BLUE.configure_output(false)
}
