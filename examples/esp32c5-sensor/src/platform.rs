//! ESP32-C5 capabilities for the shared sleepy-sensor application.

use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, EnvironmentReading, EnvironmentSource, Supervisor,
};

/// Why [`C5Environment`] has no reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnvironmentError {
    /// esp-hal 1.1 has no ESP32-C5 TSENS driver and the product's
    /// calibrated die-temperature path (`chip_temperature`) is not ported to
    /// the C5; the devkit fits no other sensor. Every sample fails with
    /// this instead of reporting an invented value.
    NoTemperatureSource,
}

/// Environment of the ESP32-C5-DevKitC-1: nothing is measured yet (see
/// [`EnvironmentError::NoTemperatureSource`]). The image still exercises
/// the sleepy lifecycle, polling, reporting plumbing and OTA.
#[derive(Debug, Default)]
pub struct C5Environment;

impl EnvironmentSource for C5Environment {
    type Error = EnvironmentError;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        Err(EnvironmentError::NoTemperatureSource)
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct EspSupervisor;

impl Supervisor for EspSupervisor {
    fn heartbeat(&mut self) {
        esp32_zigbee_devkit_product::ota_boot::poll();
    }

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        esp_hal::system::software_reset()
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct EspDiagnostics;

impl Diagnostics for EspDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        esp_println::println!("[ESP32-C5] {:?}", event);
        esp32_zigbee_devkit_product::ota_boot::observe(&event);
    }
}
