//! Exclusive ownership tokens for the EFR32MG21 peripherals used by the BSP.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::{clock::ClockControl, flash::Efr32mg21Flash, gpio::Gpio};

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Uniquely owned chip resources.
///
/// The board support crate consumes this value and turns it into narrower
/// board-level resources. In particular, there is only one internal-flash
/// owner, so the two persistence formats cannot alias the MSC concurrently.
pub struct Peripherals {
    pub clocks: ClockControl,
    pub gpio: Gpio,
    pub flash: Efr32mg21Flash,
}

impl Peripherals {
    /// Acquire the singleton peripheral set once.
    pub fn take() -> Option<Self> {
        TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .ok()
            .map(|_| Self {
                clocks: ClockControl::new(),
                gpio: Gpio::new(),
                flash: Efr32mg21Flash::new(),
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
        assert_eq!(size_of::<Efr32mg21Flash>(), 0);
    }
}
