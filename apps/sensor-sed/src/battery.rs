//! Product/board supplied battery measurement capability.

use core::convert::Infallible;

use zigbee_runtime::profile::BatteryMeasurement;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryReading {
    pub millivolts: u32,
    pub measurement: BatteryMeasurement,
}

/// A statically selected asynchronous battery backend.
#[allow(async_fn_in_trait)]
pub trait BatterySource {
    type Error;

    /// `Ok(None)` means this product has no sample to publish now.
    async fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error>;
}

pub trait BlockingBatterySource {
    type Error;

    fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error>;
}

pub struct BlockingBattery<T>(T);

impl<T> BlockingBattery<T> {
    pub const fn new(inner: T) -> Self {
        Self(inner)
    }

    pub fn into_inner(self) -> T {
        self.0
    }
}

impl<T: BlockingBatterySource> BatterySource for BlockingBattery<T> {
    type Error = T::Error;

    async fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error> {
        self.0.sample()
    }
}

/// Fixed supply reading used by powered devkits that advertise Power
/// Configuration but have no battery ADC.
#[derive(Debug, Clone, Copy)]
pub struct FixedBattery {
    reading: BatteryReading,
}

impl FixedBattery {
    pub const fn new(millivolts: u32, measurement: BatteryMeasurement) -> Self {
        Self {
            reading: BatteryReading {
                millivolts,
                measurement,
            },
        }
    }
}

impl BatterySource for FixedBattery {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error> {
        Ok(Some(self.reading))
    }
}
