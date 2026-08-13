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

use zigbee_runtime::profile::BatteryMeasurement;

/// Product-owned conversion from a raw SAADC VDD sample.
pub trait BatteryPolicy {
    /// Supply voltage in millivolts, used for the diagnostic log line.
    fn millivolts(raw_sample: i16) -> u32;

    /// ZCL Power Configuration measurement (100 mV units and ZCL
    /// half-percent remaining capacity).
    fn measurement(raw_sample: i16) -> BatteryMeasurement;
}
