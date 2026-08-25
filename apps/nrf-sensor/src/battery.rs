//! Battery chemistry binding.
//!
//! The *policy* (curve endpoints, cell chemistry, SAADC scaling) belongs to
//! the product crate — see `products/nrf52840-sensor/src/battery.rs` and
//! `products/nrf52833-sensor/src/battery.rs`. This trait only lets the
//! composition root hand that product-owned policy to the shared
//! application without this crate ever depending on a product (which would
//! invert the layer ordering).
//!
//! Implementations are expected to be zero-sized marker types, so the calls
//! monomorphize away to the same direct arithmetic the original
//! single-product firmware inlined.

use core::{convert::Infallible, marker::PhantomData};

use embassy_nrf::saadc::Saadc;
use sensor_sed_app::{BatteryReading, BatterySource};
use zigbee_runtime::profile::BatteryMeasurement;

/// Product-owned conversion from a raw SAADC VDD sample.
pub trait BatteryPolicy {
    /// Supply voltage in millivolts, used for the diagnostic log line.
    fn millivolts(raw_sample: i16) -> u32;

    /// ZCL Power Configuration measurement (100 mV units and ZCL
    /// half-percent remaining capacity).
    fn measurement(raw_sample: i16) -> BatteryMeasurement;
}

/// Nordic SAADC battery backend bound to a product-owned conversion policy.
pub struct NrfBattery<B> {
    saadc: Saadc<'static, 1>,
    policy: PhantomData<B>,
}

impl<B> NrfBattery<B> {
    pub const fn new(saadc: Saadc<'static, 1>) -> Self {
        Self {
            saadc,
            policy: PhantomData,
        }
    }
}

impl<B: BatteryPolicy> BatterySource for NrfBattery<B> {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error> {
        let mut samples = [0i16; 1];
        self.saadc.sample(&mut samples).await;
        Ok(Some(BatteryReading {
            millivolts: B::millivolts(samples[0]),
            measurement: B::measurement(samples[0]),
        }))
    }
}
