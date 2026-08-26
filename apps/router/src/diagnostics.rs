//! Typed lifecycle diagnostics with a statically selected backend.

use zigbee_runtime::child_store::ChildStoreError;
use zigbee_runtime::event_loop::{StackEvent, StartError};

use crate::{NodeArchetype, RouterAppError};

/// Copyable application-facing summary of every current [`StackEvent`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StackEventSummary {
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
    CommandReceived {
        src_addr: u16,
        source_endpoint: u8,
        endpoint: u8,
        cluster_id: u16,
        command_id: u8,
        seq_number: u8,
    },
    CommissioningComplete {
        success: bool,
    },
    DefaultResponse {
        src_addr: u16,
        endpoint: u8,
        cluster_id: u16,
        command_id: u8,
        status: u8,
    },
    ReportingConfigured {
        src_addr: u16,
        source_endpoint: u8,
        endpoint: u8,
        cluster_id: u16,
        configured_clusters: usize,
    },
    PermitJoinChanged {
        open: bool,
    },
    ReportSent,
    OtaImageAvailable {
        version: u32,
        size: u32,
    },
    OtaProgress {
        percent: u8,
    },
    OtaComplete,
    OtaFailed,
    OtaDelayedActivation {
        delay_secs: u32,
    },
    BasicResetToFactoryDefaults,
    LeaveRequested,
    RejoinRequested,
}

/// Summarize a stack event without taking ownership from [`crate::StepEvents`].
///
/// The match is deliberately exhaustive so adding a runtime event requires an
/// explicit application decision.
pub fn summarize_stack_event(event: &StackEvent) -> StackEventSummary {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => StackEventSummary::Joined {
            short_address: *short_address,
            channel: *channel,
            pan_id: *pan_id,
        },
        StackEvent::Left => StackEventSummary::Left,
        StackEvent::AttributeReport {
            src_addr,
            endpoint,
            cluster_id,
            attr_id,
        } => StackEventSummary::AttributeReport {
            src_addr: *src_addr,
            endpoint: *endpoint,
            cluster_id: *cluster_id,
            attr_id: *attr_id,
        },
        StackEvent::CommandReceived {
            src_addr,
            source_endpoint,
            endpoint,
            cluster_id,
            command_id,
            seq_number,
            ..
        } => StackEventSummary::CommandReceived {
            src_addr: *src_addr,
            source_endpoint: *source_endpoint,
            endpoint: *endpoint,
            cluster_id: *cluster_id,
            command_id: *command_id,
            seq_number: *seq_number,
        },
        StackEvent::CommissioningComplete { success } => {
            StackEventSummary::CommissioningComplete { success: *success }
        }
        StackEvent::DefaultResponse {
            src_addr,
            endpoint,
            cluster_id,
            command_id,
            status,
        } => StackEventSummary::DefaultResponse {
            src_addr: *src_addr,
            endpoint: *endpoint,
            cluster_id: *cluster_id,
            command_id: *command_id,
            status: *status,
        },
        StackEvent::ReportingConfigured {
            src_addr,
            source_endpoint,
            endpoint,
            cluster_id,
            configured_clusters,
        } => StackEventSummary::ReportingConfigured {
            src_addr: *src_addr,
            source_endpoint: *source_endpoint,
            endpoint: *endpoint,
            cluster_id: *cluster_id,
            configured_clusters: *configured_clusters,
        },
        StackEvent::PermitJoinChanged { open } => {
            StackEventSummary::PermitJoinChanged { open: *open }
        }
        StackEvent::ReportSent => StackEventSummary::ReportSent,
        StackEvent::OtaImageAvailable { version, size } => StackEventSummary::OtaImageAvailable {
            version: *version,
            size: *size,
        },
        StackEvent::OtaProgress { percent } => StackEventSummary::OtaProgress { percent: *percent },
        StackEvent::OtaComplete => StackEventSummary::OtaComplete,
        StackEvent::OtaFailed => StackEventSummary::OtaFailed,
        StackEvent::OtaDelayedActivation { delay_secs } => {
            StackEventSummary::OtaDelayedActivation {
                delay_secs: *delay_secs,
            }
        }
        StackEvent::BasicResetToFactoryDefaults => StackEventSummary::BasicResetToFactoryDefaults,
        StackEvent::LeaveRequested => StackEventSummary::LeaveRequested,
        StackEvent::RejoinRequested => StackEventSummary::RejoinRequested,
    }
}

/// Semantic lifecycle diagnostics emitted by the shared router application.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticEvent {
    InitializationStarted {
        archetype: NodeArchetype,
    },
    DefaultReportingConfigured,
    CommissioningAttempt {
        archetype: NodeArchetype,
        attempt: u32,
    },
    StartFailed {
        error: StartError,
    },
    RetryScheduled {
        attempt: u32,
        delay_ms: u32,
    },
    NetworkReady {
        archetype: NodeArchetype,
        short_address: u16,
        channel: u8,
        pan_id: u16,
    },
    SecurityCheckpoint {
        changed: bool,
    },
    ChildrenRestored {
        count: usize,
    },
    ChildTableDiscarded {
        error: ChildStoreError,
    },
    ChildTableSaved,
    ChildTableCleared,
    FrameReceived,
    StackEvent(StackEventSummary),
    RunAgain {
        delay_ms: u32,
    },
    SecureRejoinSucceeded {
        short_address: u16,
    },
    SecureRejoinFailed {
        error: StartError,
        failures: u8,
    },
    SecureRejoinRetryFailed {
        failures: u8,
    },
    SecureRejoinPending {
        failures: u8,
    },
    SecureRejoinLimitReached {
        failures: u8,
    },
    FactoryReset,
    Fatal(RouterAppError),
}

pub trait Diagnostics {
    fn record(&mut self, event: DiagnosticEvent);
}

#[derive(Debug, Default, Clone, Copy)]
pub struct NoDiagnostics;

impl Diagnostics for NoDiagnostics {
    fn record(&mut self, _event: DiagnosticEvent) {}
}
