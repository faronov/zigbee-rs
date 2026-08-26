//! Exclusive BRD4181A board resources.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32mg21_hal::{
    clock::ClockControl,
    gpio::{Disabled, Level, Pin, Pull},
    peripherals::Peripherals,
};

use crate::{
    BUTTON0_PIN, BUTTON0_PORT, Button0, Efr32mg21Flash, LED0_PIN, LED0_PORT, Led0,
    configure_button_interrupt,
};

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Complete singleton resource set for BRD4181A on BRD4001A.
pub struct BoardResources {
    pub clocks: ClockControl,
    pub led0: Led0Token,
    pub button0: Button0Token,
    /// Raw, exclusive internal-flash controller. Product code bounds it to
    /// its selected partitions before using it for persistence.
    pub flash: Efr32mg21Flash,
}

impl BoardResources {
    /// Acquire all supported board resources once.
    pub fn take() -> Option<Self> {
        if TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return None;
        }

        let mut peripherals = match Peripherals::take() {
            Some(peripherals) => peripherals,
            None => {
                TAKEN.store(false, Ordering::Release);
                return None;
            }
        };

        // SAFETY: BoardResources is a singleton and these are two distinct
        // fitted pins in the selected BRD4181A configuration.
        let led0 = unsafe { peripherals.gpio.claim_pin(LED0_PORT, LED0_PIN) }.ok()?;
        // SAFETY: same singleton argument; PD2 is distinct from PB0.
        let button0 = unsafe { peripherals.gpio.claim_pin(BUTTON0_PORT, BUTTON0_PIN) }.ok()?;

        Some(Self {
            clocks: peripherals.clocks,
            led0: Led0Token(led0),
            button0: Button0Token(button0),
            flash: peripherals.flash,
        })
    }
}

/// Exclusive PB0 ownership.
pub struct Led0Token(Pin<Disabled>);

impl Led0Token {
    /// Configure PB0 as an initially-off, active-high push-pull LED.
    pub fn into_led(self) -> Led0 {
        Led0::from_pin(self.0.into_push_pull(Level::Low))
    }
}

/// Exclusive PD2 ownership.
pub struct Button0Token(Pin<Disabled>);

impl Button0Token {
    /// Configure PD2 as a plain input with falling-edge interrupt.
    ///
    /// BRD4001A provides the active-low button's external bias. No internal
    /// pull is enabled, matching Silicon Labs' BRD4181A board configuration.
    pub fn into_button(self) -> Button0 {
        let mut pin = self.0.into_input(Pull::None);
        configure_button_interrupt(&mut pin);
        Button0::from_pin(pin)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_resources_are_singleton() {
        assert!(BoardResources::take().is_some());
        assert!(BoardResources::take().is_none());
    }
}
