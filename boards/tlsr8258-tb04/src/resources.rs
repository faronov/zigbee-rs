//! Typed ownership for fitted TB-04 resources.
//!
//! The documented board wiring covers the three status LEDs and internal
//! flash. No board-level I2C or SPI constructor is exposed:
//! the module brings GPIOs out, but does not hard-wire a sensor bus or external
//! SPI device. Applications may use the generic HAL pin-group constructors.

use crate::leds::StatusLeds;

pub const LED_PWM_HZ: u32 = 1_000;
pub const SYSTEM_CLOCK_HZ: u32 = 24_000_000;

pub struct BoardResources {
    /// Mutually exclusive I2C/SPI controller and the non-LED route pins.
    pub serial: SerialResources,
    /// PC1/PB5/PC4: choose direct GPIO LEDs or PWM0/PWM5/PWM2.
    pub lighting: LightingToken,
    /// Exclusive ownership of the fitted 512 KiB flash.
    #[cfg(target_arch = "tc32")]
    pub flash: OnboardFlash,
}

impl BoardResources {
    pub fn take() -> Option<Self> {
        let peripherals = tlsr8258_hal::peripherals::Peripherals::take()?;
        let tlsr8258_hal::peripherals::Pins {
            pa2,
            pa3,
            pa4,
            pb5,
            pb6,
            pb7,
            pc1,
            pc2,
            pc3,
            pc4,
            pd7,
            ..
        } = peripherals.pins;
        Some(Self {
            serial: SerialResources {
                controller: peripherals.serial,
                pa2,
                pa3,
                pa4,
                pb6,
                pb7,
                pc2,
                pc3,
                pd7,
            },
            lighting: LightingToken {
                pwm: peripherals.pwm,
                red: pc1,
                green: pb5,
                blue: pc4,
            },
            #[cfg(target_arch = "tc32")]
            flash: OnboardFlash(()),
        })
    }
}

/// Pins participating in the supported I2C/SPI route groups.
///
/// Consuming `controller` for either driver makes I2C and SPI mutually
/// exclusive. The non-`Clone` pins likewise prevent overlapping PA/PB routes
/// from being configured twice by safe code.
pub struct SerialResources {
    pub controller: tlsr8258_hal::peripherals::SerialController,
    pub pa2: tlsr8258_hal::gpio::Pin,
    pub pa3: tlsr8258_hal::gpio::Pin,
    pub pa4: tlsr8258_hal::gpio::Pin,
    pub pb6: tlsr8258_hal::gpio::Pin,
    pub pb7: tlsr8258_hal::gpio::Pin,
    pub pc2: tlsr8258_hal::gpio::Pin,
    pub pc3: tlsr8258_hal::gpio::Pin,
    pub pd7: tlsr8258_hal::gpio::Pin,
}

/// Mutually-exclusive ownership of the fitted RGB/status LED pins.
pub struct LightingToken {
    pwm: tlsr8258_hal::peripherals::Pwm,
    red: tlsr8258_hal::gpio::Pin,
    green: tlsr8258_hal::gpio::Pin,
    blue: tlsr8258_hal::gpio::Pin,
}

impl LightingToken {
    pub fn into_status_leds(self) -> StatusLeds {
        StatusLeds::new(self.red, self.green, self.blue)
    }

    /// Configure all three documented LED pins as hardware PWM outputs.
    #[cfg(target_arch = "tc32")]
    pub fn into_rgb_pwm(self) -> Result<RgbPwm, tlsr8258_hal::pwm::PwmError> {
        RgbPwm::new(self)
    }
}

#[cfg(target_arch = "tc32")]
pub struct OnboardFlash(());

/// Shared-frequency PWM ownership for the fitted RGB LED.
#[cfg(target_arch = "tc32")]
pub struct RgbPwm {
    pwm: tlsr8258_hal::pwm::Pwm,
}

#[cfg(target_arch = "tc32")]
impl RgbPwm {
    fn new(resources: LightingToken) -> Result<Self, tlsr8258_hal::pwm::PwmError> {
        use tlsr8258_hal::pwm::{Channel, Config, Polarity, Pwm};

        let mut pwm = Pwm::new(
            resources.pwm,
            Config {
                reference_hz: SYSTEM_CLOCK_HZ,
                frequency_hz: LED_PWM_HZ,
            },
        )?;
        // Preserve the existing board crate's color names/pins.
        pwm.configure_channel(Channel::Pwm0, resources.red, Polarity::ActiveHigh)?;
        pwm.configure_channel(Channel::Pwm5, resources.green, Polarity::ActiveHigh)?;
        pwm.configure_channel(Channel::Pwm2, resources.blue, Polarity::ActiveHigh)?;
        pwm.enable(Channel::Pwm0)?;
        pwm.enable(Channel::Pwm5)?;
        pwm.enable(Channel::Pwm2)?;
        Ok(Self { pwm })
    }

    pub const fn max_duty_cycle(&self) -> u16 {
        self.pwm.max_duty_cycle()
    }

    pub const fn actual_frequency_hz(&self) -> u32 {
        self.pwm.actual_frequency_hz()
    }

    pub fn set_red(&mut self, duty: u16) -> Result<(), tlsr8258_hal::pwm::PwmError> {
        self.pwm
            .set_duty_cycle(tlsr8258_hal::pwm::Channel::Pwm0, duty)
    }

    pub fn set_green(&mut self, duty: u16) -> Result<(), tlsr8258_hal::pwm::PwmError> {
        self.pwm
            .set_duty_cycle(tlsr8258_hal::pwm::Channel::Pwm5, duty)
    }

    pub fn set_blue(&mut self, duty: u16) -> Result<(), tlsr8258_hal::pwm::PwmError> {
        self.pwm
            .set_duty_cycle(tlsr8258_hal::pwm::Channel::Pwm2, duty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn board_resources_are_single_take() {
        assert!(BoardResources::take().is_some());
        assert!(BoardResources::take().is_none());
    }

    #[test]
    fn peripheral_token_is_zero_sized_but_pin_owners_are_not() {
        assert_eq!(size_of::<tlsr8258_hal::peripherals::SerialController>(), 0);
        assert!(size_of::<LightingToken>() > 0);
        assert!(size_of::<SerialResources>() > 0);
        #[cfg(target_arch = "tc32")]
        assert_eq!(size_of::<OnboardFlash>(), 0);
    }

    #[test]
    fn board_pwm_defaults_match_the_24mhz_clock() {
        assert_eq!(SYSTEM_CLOCK_HZ, 24_000_000);
        assert_eq!(LED_PWM_HZ, 1_000);
    }
}
