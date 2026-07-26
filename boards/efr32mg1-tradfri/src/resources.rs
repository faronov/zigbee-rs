//! Typed board resource ownership with mutual-exclusion guarantees.
//!
//! [`BoardResources::take()`] returns the complete set of peripheral tokens
//! that this board can provide. Each token is consumed exactly once to obtain
//! the corresponding driver, enforcing that:
//!
//! - PA0 is configured as **either** a direct GPIO LED **or** a TIMER0 PWM
//!   output, never both simultaneously.
//! - The external flash bus is owned as **either** direct USART0 SPI access
//!   **or** acknowledged as bootloader-managed (Gecko Bootloader uses its own
//!   internal SPI driver for the same physical bus).
//! - ADC0 supply measurement, I2C0 sensor bus, and PB13 button each have a
//!   single owner.
//!
//! Production firmware and peripheral diagnostics should consume these tokens.
//! The lower-level constructors in the parent module remain available for
//! compatibility with diagnostics that have not yet migrated.
//!
//! Chip-internal radio, RTCC, and internal-flash ownership stays in the
//! platform/HAL and product layers; those are not board-wiring resources.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::SupplyError;
use crate::{
    Button, FlashSpiResources, Led, LedPwm, SensorI2c, SupplyMonitor, flash_spi, led_pwm,
    sensor_i2c, supply_monitor,
};
use efr32mg1_hal::{i2c::I2cError, pwm::PwmError, spi::SpiError};

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Complete set of TRADFRI board peripheral tokens.
///
/// Obtain this once via [`BoardResources::take()`]. Each field is a token that
/// must be consumed to initialize the corresponding driver. Unused tokens are
/// simply dropped — the linker eliminates the dead driver code.
pub struct BoardResources {
    /// PA0 output: choose GPIO LED or TIMER0 PWM (mutually exclusive).
    pub pa0: Pa0Output,
    /// PB13 user button with interrupt support.
    pub button: ButtonToken,
    /// ADC0 AVDD supply rail measurement.
    pub supply_adc: SupplyAdcToken,
    /// I2C0 sensor bus (PC10 SDA, PC11 SCL).
    pub sensor_i2c: SensorI2cToken,
    /// External SPI flash bus: choose direct USART0 or bootloader-managed.
    pub external_flash: ExternalFlashBus,
}

impl BoardResources {
    /// Take the board resource set. Returns `None` if already taken.
    ///
    /// This is the recommended entry point for production firmware.
    /// Call it once during startup to obtain typed exclusive ownership of
    /// all fitted peripherals.
    pub fn take() -> Option<Self> {
        if TAKEN.swap(true, Ordering::AcqRel) {
            return None;
        }
        Some(Self {
            pa0: Pa0Output(()),
            button: ButtonToken(()),
            supply_adc: SupplyAdcToken(()),
            sensor_i2c: SensorI2cToken(()),
            external_flash: ExternalFlashBus(()),
        })
    }
}

/// Token for PA0 — the board LED output pin.
///
/// Consumed by exactly one of:
/// - [`Pa0Output::into_led()`] — direct GPIO push-pull output.
/// - [`Pa0Output::into_led_pwm()`] — TIMER0 CC0 PWM routed to PA0.
///
/// Both use the same physical pin; using both simultaneously would produce
/// undefined output behavior.
///
/// ```compile_fail
/// use efr32mg1_tradfri::resources::BoardResources;
///
/// let board = BoardResources::take().unwrap();
/// let _led = board.pa0.into_led();
/// let _pwm = board.pa0.into_led_pwm(); // PA0 was already consumed
/// ```
pub struct Pa0Output(());

impl Pa0Output {
    /// Configure PA0 as a direct GPIO push-pull LED (active high).
    ///
    /// This consumes the PA0 token. TIMER0 PWM cannot subsequently be
    /// configured on this pin.
    pub fn into_led(self) -> Led {
        let led = Led::new();
        led.init();
        led
    }

    /// Configure PA0 as TIMER0 CC0 PWM output at the board's default 1 kHz.
    ///
    /// This consumes the PA0 token. Direct GPIO LED control cannot
    /// subsequently be used on this pin.
    pub fn into_led_pwm(self) -> Result<LedPwm, PwmError> {
        led_pwm()
    }
}

/// Token for PB13 — the user button.
pub struct ButtonToken(());

impl ButtonToken {
    /// Initialize PB13 as input with pull-up, filter, and falling-edge
    /// interrupt configured.
    pub fn into_button(self) -> Button {
        let button = Button::new();
        button.init();
        button
    }
}

/// Token for ADC0 AVDD supply measurement.
pub struct SupplyAdcToken(());

impl SupplyAdcToken {
    /// Initialize ADC0 for AVDD supply rail measurement.
    pub fn into_supply_monitor(self) -> Result<SupplyMonitor, SupplyError> {
        supply_monitor()
    }
}

/// Token for I2C0 sensor bus (PC10/PC11).
pub struct SensorI2cToken(());

impl SensorI2cToken {
    /// Initialize I2C0 with internal pull-ups at the board's 10 kHz default.
    pub fn into_sensor_i2c(self) -> Result<SensorI2c, I2cError> {
        sensor_i2c()
    }
}

/// Token for the external flash bus (USART0 SPI to MX25R8035F).
///
/// This bus is shared between two mutually exclusive access paths:
/// - Direct USART0 SPI access from application code.
/// - The resident Gecko Bootloader's internal SPI driver (used during OTA).
///
/// They must NOT operate concurrently. Consuming this token into one path
/// prevents accidental use of the other.
///
/// ```compile_fail
/// use efr32mg1_tradfri::resources::BoardResources;
///
/// let board = BoardResources::take().unwrap();
/// let _spi = board.external_flash.into_direct_spi();
/// let _bootloader = board.external_flash.into_bootloader_managed();
/// ```
pub struct ExternalFlashBus(());

impl ExternalFlashBus {
    /// Consume for direct USART0 SPI access (PD13 CLK, PD14 MISO, PD15
    /// MOSI, PB11 CS).
    ///
    /// Do not use this while the Gecko Bootloader's storage API is active.
    pub fn into_direct_spi(self) -> Result<FlashSpiResources, SpiError> {
        flash_spi()
    }

    /// Acknowledge that external flash is managed by the Gecko Bootloader.
    ///
    /// The returned [`BootloaderFlashAccess`] marker proves exclusive
    /// ownership of the flash bus path was yielded to the bootloader.
    /// Use `efr32mg1_hal::bootloader::Bootloader` for actual storage
    /// operations.
    pub fn into_bootloader_managed(self) -> BootloaderFlashAccess {
        BootloaderFlashAccess(())
    }
}

/// Marker proving that external flash bus ownership was yielded to the
/// resident Gecko Bootloader's storage driver.
///
/// Holding this prevents construction of direct SPI flash access through
/// the typed resource path. The bootloader's own SPI management is
/// activated/deactivated via `Bootloader::init()` / `Bootloader::deinit()`.
#[must_use = "pass this ownership marker to the product OTA backend"]
pub struct BootloaderFlashAccess(());

impl BootloaderFlashAccess {
    /// Reclaim the external flash bus token (e.g., after OTA completes).
    ///
    /// # Safety
    ///
    /// The caller must ensure the bootloader's storage driver has been
    /// deinitialized (`Bootloader::deinit()`) before reclaiming bus access.
    pub unsafe fn reclaim(self) -> ExternalFlashBus {
        ExternalFlashBus(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn board_resources_can_only_be_taken_once() {
        assert!(BoardResources::take().is_some());
        assert!(BoardResources::take().is_none());
    }

    #[test]
    fn ownership_tokens_are_zero_sized() {
        assert_eq!(size_of::<Pa0Output>(), 0);
        assert_eq!(size_of::<ButtonToken>(), 0);
        assert_eq!(size_of::<SupplyAdcToken>(), 0);
        assert_eq!(size_of::<SensorI2cToken>(), 0);
        assert_eq!(size_of::<ExternalFlashBus>(), 0);
        assert_eq!(size_of::<BootloaderFlashAccess>(), 0);
    }
}
