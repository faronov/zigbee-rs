//! USB-powered kit: no battery measurement is published.

use core::convert::Infallible;
use sensor_sed_app::{BatteryReading, BlockingBatterySource};

/// Battery source for a product without a battery: never yields a sample,
/// so the Power Configuration attributes stay at their defaults.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoBattery;

impl BlockingBatterySource for NoBattery {
    type Error = Infallible;

    fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error> {
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn never_publishes_a_battery_sample() {
        assert_eq!(NoBattery.sample(), Ok(None));
    }
}
