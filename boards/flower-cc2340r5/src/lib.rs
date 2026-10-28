//! Pure-Rust board support for koptserg's "flower" soil-moisture board: a
//! CC2340R5 RGE in an Ebyte E104-BT55SP module
//! (<https://github.com/koptserg/flower_CC2340R5_RGE>).
//!
//! This crate owns only the board's physical wiring: two status LEDs, SW1,
//! the `I2C0` bus to the optional TMP102/BH1750/OPT3001 parts, and the soil
//! probe's excitation PWM and detector ADC. The CC2340R5 chip services come
//! from [`cc2340r5_hal`]. Detecting which optional I2C parts are fitted,
//! calibration and Zigbee behavior belong to the product crate.

#![no_std]

pub use cc2340r5_hal::{MCU_CLOCK_HZ, TARGET_PART};
pub use cc2340r5_hal::{adc, batmon, flash, i2c, identity, lgpt, power, standby, timebase};
#[cfg(target_os = "none")]
pub use cc2340r5_hal::{interrupts, reset};

pub mod pins;
pub mod soil;

pub use soil::SoilProbe;

pub const BOARD_NAME: &str = "koptserg flower (E104-BT55SP, CC2340R5 RGE)";

/// The E104-BT55SP module populates the DC/DC inductor (reference SysConfig
/// keeps the default DC/DC regulator).
pub const VDDR_REGULATOR: cc2340r5_hal::VddrRegulator = cc2340r5_hal::VddrRegulator::DcDc;

/// Reference I2C rate (`I2C_Params_init()` default, 100 kHz).
pub const I2C_SPEED: i2c::Speed = i2c::Speed::Standard100k;

/// All exclusively owned board resources.
#[cfg(target_os = "none")]
pub struct Resources {
    /// Active-high green LED on DIO3.
    pub green_led: pins::GreenLed,
    /// Active-high red LED on DIO4.
    pub red_led: pins::RedLed,
    /// Active-low SW1 on DIO20.
    pub button: pins::UserButton,
    /// `I2C0` on SDA DIO8 / SCL DIO6 with internal pull-ups; clock gated
    /// except during a transaction.
    pub i2c: i2c::I2c,
    /// Soil-probe excitation and detector.
    pub soil: SoilProbe,
    pub flash: flash::InternalFlash,
    /// On-die VDDS and temperature monitor, measuring continuously.
    pub batmon: batmon::Batmon,
    pub reset: reset::SystemReset,
}

/// Claim and initialize the board exactly once.
///
/// [`cc2340r5_hal::take_core`] selects the DC/DC regulator and the LFOSC LF
/// clock (the board has no 32-kHz crystal; DIO3/DIO4 drive the LEDs) and
/// starts the time driver. LEDs start off. The button's press interrupt is
/// configured but masked until the application enables it. I2C, ADC and
/// LGPT1 stay clock gated until a measurement uses them.
#[cfg(target_os = "none")]
pub fn take() -> Option<Resources> {
    use cc2340r5_hal::adc::{Adc, AdcInput};
    use cc2340r5_hal::i2c::{I2c, I2cPins};
    use cc2340r5_hal::lgpt::Pwm;

    let cc2340r5_hal::Core {
        mut pins,
        flash,
        batmon,
        reset,
        i2c0,
        adc0,
        lgpt1,
    } = cc2340r5_hal::take_core(VDDR_REGULATOR)?;

    let i2c = I2c::new(
        i2c0,
        I2cPins {
            sda: pins.claim(pins::I2C_SDA_DIO).ok()?,
            sda_function: pins::I2C_SDA_FUNCTION,
            scl: pins.claim(pins::I2C_SCL_DIO).ok()?,
            scl_function: pins::I2C_SCL_FUNCTION,
        },
        I2C_SPEED,
    );
    let pwm = Pwm::new(
        lgpt1,
        pins.claim(pins::SOIL_PWM_DIO).ok()?,
        pins::SOIL_PWM_FUNCTION,
        soil::EXCITATION_HZ,
        soil::EXCITATION_DUTY_PERCENT,
    )
    .ok()?;
    let adc = Adc::new(
        adc0,
        AdcInput {
            pin: pins.claim(pins::SOIL_ADC_DIO).ok()?,
            channel: pins::SOIL_ADC_CHANNEL,
        },
        soil::ADC_CONFIG,
    );

    Some(Resources {
        green_led: pins.led(false).ok()?,
        red_led: pins.led(false).ok()?,
        button: pins.button().ok()?,
        i2c,
        soil: SoilProbe::new(pwm, adc),
        flash,
        batmon,
        reset,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn target_is_the_cc2340r52_family_part() {
        assert_eq!(MCU_CLOCK_HZ, 48_000_000);
        assert_eq!(TARGET_PART, "CC2340R52");
        assert_eq!(I2C_SPEED.hz(), 100_000);
    }

    #[test]
    fn manifest_has_no_zigbee_or_application_dependency() {
        let manifest = include_str!("../Cargo.toml");
        let dependencies = &manifest[manifest.find("[dependencies]").unwrap()..];
        assert!(!dependencies.contains("zigbee"));
        assert!(!dependencies.contains("sensor-sed-app"));
    }
}
