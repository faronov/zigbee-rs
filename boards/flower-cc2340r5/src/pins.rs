//! Physical wiring of the flower board (E104-BT55SP module, CC2340R5 RGE).
//!
//! Taken from the reference SysConfig (`on_off_switch_ota_client_onchip.syscfg`
//! and its generated `ti_drivers_config.c`). DIO0–DIO2 are not bonded out on
//! the RGE package; DIO16/DIO17 are SWD.

pub use cc2340r5_hal::gpio::*;

/// Green status LED (`CONFIG_GPIO_GLED`), active high.
pub const GREEN_LED_DIO: u8 = 3;
/// Red status LED (`CONFIG_GPIO_RLED`), active high.
pub const RED_LED_DIO: u8 = 4;
/// SW1 (`CONFIG_GPIO_BTN1`), active low with the internal pull-up.
pub const BUTTON_DIO: u8 = 20;

/// `I2C0` SDA and its pin-mux function.
pub const I2C_SDA_DIO: u8 = 8;
pub const I2C_SDA_FUNCTION: PinFunction = PinFunction::Pfunc4;
/// `I2C0` SCL and its pin-mux function.
pub const I2C_SCL_DIO: u8 = 6;
pub const I2C_SCL_FUNCTION: PinFunction = PinFunction::Pfunc2;

/// Soil-probe excitation: `LGPT1` channel 0.
pub const SOIL_PWM_DIO: u8 = 24;
pub const SOIL_PWM_FUNCTION: PinFunction = PinFunction::Pfunc2;
/// Soil-probe detector output: ADC internal channel 10.
pub const SOIL_ADC_DIO: u8 = 21;
pub const SOIL_ADC_CHANNEL: u8 = 10;

/// Physical green LED resource.
pub type GreenLed = Led<GREEN_LED_DIO>;
/// Physical red LED resource.
pub type RedLed = Led<RED_LED_DIO>;
/// Physical SW1 resource.
pub type UserButton = Button<BUTTON_DIO>;

/// DIOs that do not exist on the CC2340R5 RGE (5x5 mm) package.
pub const RGE_UNBONDED_DIOS: [u8; 3] = [0, 1, 2];

/// Every DIO this board claims.
pub const BOARD_DIOS: [u8; 7] = [
    GREEN_LED_DIO,
    RED_LED_DIO,
    BUTTON_DIO,
    I2C_SDA_DIO,
    I2C_SCL_DIO,
    SOIL_PWM_DIO,
    SOIL_ADC_DIO,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_pins_are_unique_bonded_and_not_swd() {
        for (index, pin) in BOARD_DIOS.iter().enumerate() {
            assert!(*pin < DIO_COUNT);
            assert!(!SWD_DIOS.contains(pin));
            assert!(!RGE_UNBONDED_DIOS.contains(pin));
            assert!(!BOARD_DIOS[..index].contains(pin));
        }
    }

    #[test]
    fn peripheral_functions_match_the_reference_pin_mux() {
        assert_eq!(I2C_SDA_FUNCTION.portcfg(), 4);
        assert_eq!(I2C_SCL_FUNCTION.portcfg(), 2);
        assert_eq!(SOIL_PWM_FUNCTION.portcfg(), 2);
        assert_eq!(SOIL_ADC_CHANNEL, 10);
    }
}
