//! Identify on the board's WS2812 for the sleepy sensor (feature
//! `identify-led`).
//!
//! The sleepy image has no status palette: the LED stays dark except while
//! the device identifies (ZCL Identify time, or a Trigger Effect), when it
//! blinks magenta with the lifecycle's Identify phase. The RMT channel only
//! transmits on a change, so a non-identifying device never touches the
//! line after boot.

use esp32_zigbee_devkit::status_led::{StatusLed, StatusLedError, StatusLedPin};
use sensor_sed_app::{SensorStatus, StatusSink};

/// The Identify colour: the mains roles' identify magenta, dim.
const IDENTIFY: (u8, u8, u8) = (16, 0, 16);

/// [`StatusSink`] lighting the WS2812 only for Identify.
pub struct IdentifyLed {
    led: Option<StatusLed>,
    lit: bool,
}

impl IdentifyLed {
    /// Take the RMT peripheral and the LED pin; an RMT error leaves the
    /// sink without an LED (Identify still runs, unseen).
    pub fn new(rmt: esp_hal::peripherals::RMT<'static>, pin: StatusLedPin) -> Self {
        let led = StatusLed::new(rmt, pin)
            .map_err(|error: StatusLedError| log::warn!("identify LED unavailable: {:?}", error))
            .ok();
        Self { led, lit: false }
    }
}

impl StatusSink for IdentifyLed {
    fn set(&mut self, status: SensorStatus) {
        let on = matches!(status, SensorStatus::Identifying { on: true });
        if on == self.lit {
            return;
        }
        if let Some(led) = self.led.as_mut() {
            let (r, g, b) = if on { IDENTIFY } else { (0, 0, 0) };
            if led.write(r, g, b).is_ok() {
                self.lit = on;
            }
        }
    }
}
