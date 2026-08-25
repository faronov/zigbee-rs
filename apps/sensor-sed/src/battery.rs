//! Product/board supplied battery measurement capability.

use zigbee_runtime::profile::BatteryMeasurement;

/// One battery sample ready for both diagnostics and ZCL mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryReading {
    pub millivolts: u32,
    pub measurement: BatteryMeasurement,
}

/// A statically selected battery measurement backend.
#[allow(async_fn_in_trait)]
pub trait BatterySource {
    async fn sample(&mut self) -> BatteryReading;
}
