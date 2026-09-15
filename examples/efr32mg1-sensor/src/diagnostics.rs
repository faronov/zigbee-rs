//! Compact RTT rendering for the shared sensor lifecycle.

use sensor_sed_app::{DiagnosticEvent, Diagnostics};

#[derive(Debug, Default, Clone, Copy)]
pub struct RttDiagnostics;

impl Diagnostics for RttDiagnostics {
    #[inline(never)]
    fn record(&mut self, event: DiagnosticEvent) {
        let report_stack = matches!(
            &event,
            DiagnosticEvent::JoinedOrResumed { .. }
                | DiagnosticEvent::CommissioningComplete { success: true }
                | DiagnosticEvent::InterviewConfigurationComplete { .. }
        );
        rtt_target::rprintln!("[EFR32][lifecycle] {:?}", event);
        if report_stack {
            rtt_target::rprintln!(
                "[EFR32][supervisor] STACK_HIGH_WATER {}",
                crate::platform::stack_high_water_bytes()
            );
        }
    }
}
