//! Typed lifecycle diagnostics with a statically selected backend.

use zigbee_runtime::profile::ProfileError;
use zigbee_runtime::security_store::SecurityStoreError;

use crate::environment::EnvironmentReading;

/// Semantic lifecycle diagnostics emitted by [`crate::SensorApp`].
///
/// Keeping formatting outside the shared crate avoids coupling every product
/// to one transport and lets compact embedded formats such as `defmt` retain
/// their flash-size advantage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticEvent {
    SecurityFailure(SecurityStoreError),
    ProfileFailure(ProfileError),
    JoinedOrResumed {
        short_address: u16,
        channel: u8,
        pan_id: u16,
    },
    ZigbeeInitializationFailed,
    CommissioningFailed {
        status: u8,
    },
    SecureRejoinSucceeded {
        short_address: u16,
    },
    SecureRejoinInitializationFailed,
    SecureRejoinFailed {
        status: u8,
    },
    FactoryResetInitializationFailed,
    FactoryResetFailed {
        status: u8,
    },
    SecureRejoinPending {
        failures: u8,
    },
    SecureRejoinLimitReached {
        failures: u8,
    },
    Environment(EnvironmentReading),
    EnvironmentReadFailed,
    /// A product-specific [`crate::SensorMeasurement`] updated one cluster's
    /// measured value. `value` is the raw ZCL attribute value.
    Measurement {
        cluster_id: u16,
        value: i32,
    },
    /// A product-specific measurement for `cluster_id` failed; the previous
    /// cluster value was kept.
    MeasurementFailed {
        cluster_id: u16,
    },
    BatteryReadFailed,
    Battery {
        millivolts: u32,
        percentage: u8,
    },
    DefaultReportingConfigured,
    FastPollStarted {
        duration_ms: u32,
    },
    Joined {
        short_address: u16,
        channel: u8,
        pan_id: u16,
    },
    Left,
    AttributeReport {
        src_addr: u16,
        endpoint: u8,
        cluster_id: u16,
        attr_id: u16,
    },
    ReportingConfigured {
        cluster_id: u16,
        configured: usize,
        expected: usize,
    },
    InterviewConfigurationComplete {
        configured: usize,
        expected: usize,
    },
    ReportingRejected {
        cluster_id: u16,
        configured: usize,
        expected: usize,
    },
    UnhandledCommand {
        src_addr: u16,
        cluster_id: u16,
        command_id: u8,
    },
    CommissioningComplete {
        success: bool,
    },
    DefaultResponse {
        src_addr: u16,
        cluster_id: u16,
        command_id: u8,
        status: u8,
    },
    PermitJoinChanged {
        open: bool,
    },
    ReportSent,
    UnexpectedOtaEvent,
    LeaveRequested,
    BasicResetToFactoryDefaults,
    RejoinRequested,
    DeviceAnnounceRetry {
        retries_left: u8,
    },
    JoinRetry {
        attempt: u8,
    },
    /// An unjoined commissioning attempt failed; the next one starts after
    /// `delay_ms` of sleep (exponential backoff, capped at 30 minutes).
    JoinBackoff {
        delay_ms: u32,
    },
    /// The device left its network by request and is factory-new; it sleeps
    /// without searching for a network until the user presses the button.
    WaitingForButton,
    FactoryResetRequested,
    SecurityResetRebooting,
    ForceReport {
        configured: usize,
        expected: usize,
    },
    ButtonJoin,
    ButtonLeave,
    FastPollStopped {
        configured: usize,
        expected: usize,
    },
    WakeFailed,
}

/// Compile-time-selected diagnostics sink.
pub trait Diagnostics {
    fn record(&mut self, event: DiagnosticEvent);
}
