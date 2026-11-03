//! Supervisor and diagnostics for images without a watchdog or log sink.

use sensor_sed_app::{DiagnosticEvent, Diagnostics, Supervisor};

/// No watchdog: an unrecoverable lifecycle error parks the CPU.
#[derive(Debug, Default, Clone, Copy)]
pub struct TelinkSupervisor;

impl Supervisor for TelinkSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        loop {
            tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
        }
    }
}

/// Discards lifecycle diagnostics.
#[derive(Debug, Default, Clone, Copy)]
pub struct TelinkNoDiagnostics;

impl Diagnostics for TelinkNoDiagnostics {
    fn record(&mut self, _event: DiagnosticEvent) {}
}
