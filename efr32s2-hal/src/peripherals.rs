//! Exclusive ownership tokens for the EFR32 Series 2 peripherals used by the BSP.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::{
    clock::ClockControl,
    flash::Efr32s2Flash,
    gpio::Gpio,
    i2c::I2c0Token,
    radio::RadioToken,
    usart::{Instance, Usart},
};

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Uniquely owned chip resources.
///
/// The board support crate consumes this value and turns it into narrower
/// board-level resources. In particular, there is only one internal-flash
/// owner, so the two persistence formats cannot alias the MSC concurrently.
pub struct Peripherals {
    pub clocks: ClockControl,
    pub gpio: Gpio,
    pub flash: Efr32s2Flash,
    pub usart0: Usart,
    pub usart1: Usart,
    pub i2c0: I2c0Token,
    pub radio: RadioToken,
}

impl Peripherals {
    /// Acquire the singleton peripheral set once.
    ///
    /// On xG22 this also opens the GPIO, HFXO, SYSCFG and MSC bus clocks
    /// (all gated at reset) so the returned tokens are usable.
    pub fn take() -> Option<Self> {
        TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .ok()
            .map(|_| {
                for clock in crate::chip::BASE_BUS_CLOCKS {
                    crate::clock::enable_bus_clock(clock);
                }
            })
            .map(|_| Self {
                clocks: ClockControl::new(),
                gpio: Gpio::new(),
                flash: Efr32s2Flash::new(),
                usart0: Usart::new(Instance::Usart0),
                usart1: Usart::new(Instance::Usart1),
                i2c0: I2c0Token::new(),
                radio: RadioToken::new(),
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn singleton_and_tokens_are_zero_sized() {
        let _peripherals = Peripherals::take().expect("first take");
        assert!(Peripherals::take().is_none());
        assert_eq!(size_of::<ClockControl>(), 0);
        assert_eq!(size_of::<Gpio>(), 0);
        assert_eq!(size_of::<Efr32s2Flash>(), 0);
        assert_eq!(size_of::<RadioToken>(), 0);
        assert_eq!(size_of::<I2c0Token>(), 0);
    }
}
