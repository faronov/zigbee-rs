//! Compact `defmt` rendering for shared lifecycle diagnostics.

use sensor_sed_app::{DiagnosticEvent, Diagnostics};
use zigbee_runtime::security_store::SecurityStoreError;

#[derive(Debug, Default, Clone, Copy)]
pub struct NrfDiagnostics;

impl Diagnostics for NrfDiagnostics {
    #[inline(always)]
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::SecurityFailure(error) => match error {
                SecurityStoreError::NotFound => {
                    defmt::error!("Security persistence failed: not found")
                }
                SecurityStoreError::Corrupt => {
                    defmt::error!("Security persistence failed: corrupt state")
                }
                SecurityStoreError::Full => {
                    defmt::error!("Security persistence failed: full")
                }
                SecurityStoreError::Hardware => {
                    defmt::error!("Security persistence failed: hardware")
                }
                SecurityStoreError::CounterExhausted => {
                    defmt::error!("Security persistence failed: counter exhausted")
                }
                SecurityStoreError::GenerationExhausted => {
                    defmt::error!("Security persistence failed: generation exhausted")
                }
            },
            DiagnosticEvent::ProfileFailure(error) => {
                defmt::error!("Profile error: {:?}", defmt::Debug2Format(&error))
            }
            DiagnosticEvent::JoinedOrResumed {
                short_address,
                channel,
                pan_id,
            } => defmt::info!(
                "Joined/resumed network: addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address,
                channel,
                pan_id
            ),
            DiagnosticEvent::ZigbeeInitializationFailed => {
                defmt::warn!("Zigbee initialization failed")
            }
            DiagnosticEvent::CommissioningFailed { status } => {
                defmt::warn!("Commissioning failed: status=0x{:02X}", status)
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                defmt::info!("Secure rejoin succeeded: addr=0x{:04X}", short_address)
            }
            DiagnosticEvent::SecureRejoinInitializationFailed => {
                defmt::warn!("Secure rejoin initialization failed")
            }
            DiagnosticEvent::SecureRejoinFailed { status } => {
                defmt::warn!("Secure rejoin failed: status=0x{:02X}", status)
            }
            DiagnosticEvent::FactoryResetInitializationFailed => {
                defmt::warn!("Factory reset initialization failed")
            }
            DiagnosticEvent::FactoryResetFailed { status } => {
                defmt::warn!("Factory reset failed: status=0x{:02X}", status)
            }
            DiagnosticEvent::SecureRejoinPending { failures } => {
                defmt::warn!("Secure rejoin pending — will retry (failures={})", failures)
            }
            DiagnosticEvent::SecureRejoinLimitReached { failures } => defmt::warn!(
                "Secure rejoin failed {} times — resetting and rejoining fresh",
                failures
            ),
            DiagnosticEvent::EnvironmentReadFailed => {
                defmt::warn!("Environmental sensor read failed")
            }
            DiagnosticEvent::Battery {
                millivolts,
                percentage,
            } => defmt::info!("Battery: {}mV ({}%)", millivolts, percentage),
            DiagnosticEvent::DefaultReportingConfigured => {
                defmt::info!("Default reporting configured")
            }
            DiagnosticEvent::FastPollStarted { duration_secs } => {
                defmt::info!("Fast poll ON ({}s) — post-join", duration_secs)
            }
            DiagnosticEvent::Joined {
                short_address,
                channel,
                pan_id,
            } => defmt::info!(
                "Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address,
                channel,
                pan_id
            ),
            DiagnosticEvent::Left => defmt::info!("Left network"),
            DiagnosticEvent::AttributeReport {
                src_addr,
                endpoint,
                cluster_id,
                attr_id,
            } => defmt::info!(
                "Attribute report src=0x{:04X} ep={} cluster=0x{:04X} attr=0x{:04X}",
                src_addr,
                endpoint,
                cluster_id,
                attr_id
            ),
            DiagnosticEvent::ReportingConfigured {
                cluster_id,
                configured,
                expected,
            } => defmt::info!(
                "Remote ConfigureReporting: cluster=0x{:04X} {}/{} clusters",
                cluster_id,
                configured,
                expected
            ),
            DiagnosticEvent::InterviewConfigurationComplete {
                configured,
                expected,
            } => defmt::info!(
                "Interview configuration complete: {}/{} clusters",
                configured,
                expected
            ),
            DiagnosticEvent::ReportingRejected {
                cluster_id,
                configured,
                expected,
            } => defmt::warn!(
                "Remote ConfigureReporting rejected: cluster=0x{:04X} ({}/{} clusters)",
                cluster_id,
                configured,
                expected
            ),
            DiagnosticEvent::UnhandledCommand {
                src_addr,
                cluster_id,
                command_id,
            } => defmt::info!(
                "Unhandled command src=0x{:04X} cluster=0x{:04X} cmd=0x{:02X}",
                src_addr,
                cluster_id,
                command_id
            ),
            DiagnosticEvent::CommissioningComplete { success: true } => {
                defmt::info!("Commissioning: ok")
            }
            DiagnosticEvent::CommissioningComplete { success: false } => {
                defmt::info!("Commissioning: failed")
            }
            DiagnosticEvent::DefaultResponse {
                src_addr,
                cluster_id,
                command_id,
                status,
            } => defmt::info!(
                "Default response src=0x{:04X} cluster=0x{:04X} cmd=0x{:02X} status=0x{:02X}",
                src_addr,
                cluster_id,
                command_id,
                status
            ),
            DiagnosticEvent::PermitJoinChanged { open } => {
                defmt::info!("Permit join changed: open={}", open)
            }
            DiagnosticEvent::ReportSent => defmt::info!("Report sent"),
            DiagnosticEvent::OtaEventIgnored => {
                defmt::debug!("OTA event ignored — this firmware build has no OTA client")
            }
            DiagnosticEvent::LeaveRequested => {
                defmt::info!("Leave requested by coordinator — resetting and rejoining")
            }
            DiagnosticEvent::BasicResetToFactoryDefaults => {
                defmt::info!("Basic cluster attributes reset to factory defaults")
            }
            DiagnosticEvent::RejoinRequested => {
                defmt::info!("Coordinator requested secure rejoin")
            }
            DiagnosticEvent::DeviceAnnounceRetry { retries_left } => {
                defmt::info!("Re-sending Device_annce ({} left)", retries_left)
            }
            DiagnosticEvent::JoinRetry { attempt } => {
                defmt::info!("Not joined — retrying (attempt {})…", attempt)
            }
            DiagnosticEvent::FactoryResetRequested => defmt::info!("FACTORY RESET"),
            DiagnosticEvent::SecurityResetRebooting => {
                defmt::info!("Security state reset — rebooting")
            }
            DiagnosticEvent::ForceReport {
                configured,
                expected,
            } => defmt::info!(
                "Button → force report (interview configuration {}/{})",
                configured,
                expected
            ),
            DiagnosticEvent::ButtonJoin => defmt::info!("Button → join"),
            DiagnosticEvent::FastPollStopped {
                configured,
                expected,
            } => defmt::info!(
                "Fast poll OFF — remote client configured {}/{} report clusters",
                configured,
                expected
            ),
            DiagnosticEvent::RadioSleepPreparationFailed => {
                defmt::warn!("Failed to disable RADIO before poll sleep")
            }
        }
    }
}

pub fn persistence_failure(error: SecurityStoreError) -> ! {
    let mut diagnostics = NrfDiagnostics;
    diagnostics.record(DiagnosticEvent::SecurityFailure(error));
    core::panic!("security persistence failure");
}
